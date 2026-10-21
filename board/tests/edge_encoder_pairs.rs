//! The MaD machine's encoder through the Edge carrier's line receiver `U25`
//! to the module's `P9`, `P10` and `P11`: `boards/projects/edge-ec32-ds2.toml`
//! with an encoder on `J20`, run live, counted one step at a time.
//!
//! `J20` takes an RS-422 encoder: `A±` on pins 1 and 2, `B±` on 3 and 4, the
//! index `ZI±` on 9 and 10, `EN_GND` on 5; pins 7 and 8 are `U25`'s enables
//! (`Z+` its `G`, `Z−` its `~G`, which `JP4` ties to `EN_GND`). `JP2`, `JP3`
//! and `JP5` tie `A−`, `B−` and `ZI−` to `EN_GND` for a single-ended encoder.
//! `U25` (an AM26LV32, TI SLLS202H) feeds the isolator `IC16`, whose outputs
//! are `P9`, `P10` and `P11` on the module's fingers.
//!
//! The encoder is `embsim_models::machine::QuadratureEncoder`: as an RS-422
//! line driver, the pairs MaD's machine file presents, read by the receiver
//! as a valid differential each way; single-ended on the `+` legs with the
//! jumpers closed — push-pull, or the NPN open collector the user's machine
//! has (a Taiss 600 P/R optical encoder: A and B, no complements, no index)
//! — each low has no differential, which the receiver reads as its
//! fail-safe high (SLLS202H §8.4.1, Table 8-1) and the engine reports as
//! read inside its band (`Finding::PairInBand`). That is why
//! `MIGRATING-MAD.md` §4 wires the pairs, and §6 says what the board would
//! need to read the open collector.
//!
//! Stepped (`TESTING.md` rule 9), its own binary: a suite lock, the clock
//! stepped before the project builds the converter (its protocol thread
//! joins the clock as an actor), the system started with time held, the
//! case's thread a registered actor, one virtual settle past every
//! soft-start, then one short settle per count; no `QuiescenceTimeout` at
//! the end, and each case waits for the converter's thread to end before
//! the next re-anchors the clock.

use std::path::PathBuf;
use std::sync::{Mutex, MutexGuard};
use std::time::{Duration, Instant};

use embsim_board::{
    EndpointRef, Finding, Harness, JumperState, Level, NetState, Project, Scenario, SystemHandle,
};
use embsim_boards::catalog::StandardCatalog;
use embsim_boards::p2::P2_RESTART_DELAY_NS;
use embsim_core::virtual_clock::{self, ClockMode};
use embsim_models::ads122u04_component::PUMP_POLL_VIRTUAL_US;
use embsim_models::machine::quadrature_encoder::Config as EncoderConfig;
use embsim_models::machine::{IndexConfig, OutputStage, QuadratureEncoder};
use embsim_models::oscillator::TG2520SMN_START_UP_NS;
use embsim_models::rail::AP62301_SOFT_START_NS;
use rstest::rstest;
use vibes_behaviour::{behaviour, expect, Test};

static SUITE_LOCK: Mutex<()> = Mutex::new(());

fn suite_lock() -> MutexGuard<'static, ()> {
    SUITE_LOCK.lock().unwrap_or_else(|poisoned| {
        SUITE_LOCK.clear_poison();
        poisoned.into_inner()
    })
}

/// The virtual time the run is handed before the first count: 10.1 ms, past
/// the longest chain the boards arm from their supplies (the module's bucks'
/// 2.5 ms soft-start, the P2's 3 ms restart, the TCXO's 1 ms start-up), as
/// `edge_project_live.rs` settles it.
const SETTLE_NS: u64 = 10_100_000;
const _: () =
    assert!(SETTLE_NS > AP62301_SOFT_START_NS + P2_RESTART_DELAY_NS + TG2520SMN_START_UP_NS);

/// The virtual time each count is handed before the pins are read. Nothing
/// between the encoder and `P9` arms an instant (the receiver and the
/// isolator project their inputs), so any span reads the settled state;
/// 7 µs keeps every deadline off the converter pump's 250 µs polls for the
/// counts a case walks (checked as it walks).
const STEP_NS: u64 = 7_000;

/// How long, in wall time, a case waits for the converter's protocol thread
/// to end after the system shuts down: sized for a hang, not a speed
/// (`TESTING.md` rule 3).
const THREAD_END_HANG: Duration = Duration::from_secs(20);

/// The test's encoder geometry: an index once every 8 counts, one count
/// wide, active high. Rig geometry, the test's own.
const INDEX: IndexConfig = IndexConfig {
    counts_per_revolution: 8,
    width_counts: 1,
    active_level: Level::High,
};

/// The counts each case walks, one step at a time: forward through more
/// than two quadrature cycles and an index, then back through zero.
const WALK: [i64; 14] = [1, 2, 3, 4, 5, 6, 7, 8, 9, 8, 7, 0, -1, -9];

/// How an encoder sits on `J20`.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Wiring {
    /// Each channel a pair, `JP2`, `JP3` and `JP5` open.
    Pairs,
    /// Each channel push-pull on its `+` leg, `JP2`, `JP3` and `JP5`
    /// closed.
    SingleEnded,
    /// `A` and `B` NPN open collectors on their `+` legs and no index, as
    /// the user's Taiss encoder is: `JP2` and `JP3` closed, `JP5` open,
    /// nothing on `ZI±`.
    OpenCollector,
}

/// A running Edge project with an encoder on `J20`.
struct Rig {
    system: SystemHandle,
    encoder: embsim_models::machine::EncoderInput,
    actor: Option<virtual_clock::Actor>,
    at_ns: u64,
    actors_before: usize,
    _lock: MutexGuard<'static, ()>,
}

impl Rig {
    fn start(wiring: Wiring) -> Self {
        let lock = suite_lock();
        let actors_before = virtual_clock::registered_actors();
        virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);
        let path: PathBuf = [
            env!("CARGO_MANIFEST_DIR"),
            "..",
            "boards",
            "projects",
            "edge-ec32-ds2.toml",
        ]
        .iter()
        .collect();
        let config = match wiring {
            Wiring::Pairs => EncoderConfig::new(1.0).with_index(INDEX).with_complements(),
            Wiring::SingleEnded => EncoderConfig::new(1.0).with_index(INDEX),
            Wiring::OpenCollector => {
                EncoderConfig::new(1.0).with_output(OutputStage::NpnOpenCollector)
            }
        };
        let encoder = QuadratureEncoder::new(config).expect("a valid encoder");
        let input = encoder.input();
        let ep = |endpoint: &str| EndpointRef::parse(endpoint).expect("endpoint parses");
        let mut harness = Harness::new()
            .connect(ep("ENC.A"), ep("EDGE.J20.1"))
            .connect(ep("ENC.B"), ep("EDGE.J20.3"));
        if wiring != Wiring::OpenCollector {
            harness = harness.connect(ep("ENC.Z"), ep("EDGE.J20.9"));
        }
        // `JP4` asserts `U25`'s active-low enable either way.
        let mut scenario = Scenario::default().jumper("EDGE.JP4", JumperState::Closed);
        let (legs, index_leg) = match wiring {
            Wiring::Pairs => {
                harness = harness
                    .connect(ep("ENC.A-"), ep("EDGE.J20.2"))
                    .connect(ep("ENC.B-"), ep("EDGE.J20.4"))
                    .connect(ep("ENC.Z-"), ep("EDGE.J20.10"));
                (JumperState::Open, JumperState::Open)
            }
            Wiring::SingleEnded => (JumperState::Closed, JumperState::Closed),
            Wiring::OpenCollector => (JumperState::Closed, JumperState::Open),
        };
        for jumper in ["EDGE.JP2", "EDGE.JP3"] {
            scenario = scenario.jumper(jumper, legs);
        }
        scenario = scenario.jumper("EDGE.JP5", index_leg);
        let system = Project::load(&path)
            .expect("the project loads")
            .instantiate(&StandardCatalog)
            .expect("the project builds with the standard catalog alone")
            .component("ENC", Box::new(encoder))
            .harness(harness)
            .scenario(scenario)
            .hold_time()
            .start()
            .expect("the three boards and the encoder start");
        let actor = virtual_clock::register_actor("edge-encoder-pairs-case");
        system.release_time();
        virtual_clock::wait_virtual_ns(SETTLE_NS);
        Self {
            system,
            encoder: input,
            actor: Some(actor),
            at_ns: SETTLE_NS,
            actors_before,
            _lock: lock,
        }
    }

    /// Move the encoder to `count`, one count at a time, and settle.
    fn walk_to(&mut self, count: i64) {
        self.encoder.set_position_counts(count);
        self.at_ns += STEP_NS;
        assert!(
            !self.at_ns.is_multiple_of(PUMP_POLL_VIRTUAL_US * 1_000),
            "a count's deadline at {} ns falls on the converter's poll",
            self.at_ns
        );
        virtual_clock::wait_virtual_ns(STEP_NS);
    }

    /// The level a module pin's net carries, driven.
    fn level(&self, net: &str) -> Level {
        match self.system.net_state(net) {
            Some(NetState::Driven(level)) => level,
            other => panic!("{net} must be driven, got {other:?}"),
        }
    }

    /// `P9`, `P10` and `P11`: what the firmware's encoder pins read.
    fn pins(&self) -> (Level, Level, Level) {
        (
            self.level("EDGE.P9"),
            self.level("EDGE.P10"),
            self.level("EDGE.P11"),
        )
    }

    /// End the case; the findings the run made.
    fn finish(mut self) -> Vec<Finding> {
        drop(self.actor.take());
        let findings = self.system.findings();
        let stalled: Vec<&Finding> = findings
            .iter()
            .filter(|finding| matches!(finding, Finding::QuiescenceTimeout { .. }))
            .collect();
        assert!(
            stalled.is_empty(),
            "the engine stopped waiting for the case's thread: {stalled:?}"
        );
        // `U25`'s inputs declare SLLS202H's input range: no wiring here
        // takes one above it (its supply, from `SC_5V`, is reported).
        let inputs_over: Vec<&Finding> = findings
            .iter()
            .filter(|finding| {
                matches!(finding, Finding::PinAboveRecommended { part, pin, .. }
                    if part == "EDGE.U25" && pin != "16")
            })
            .collect();
        assert!(inputs_over.is_empty(), "{inputs_over:?}");
        self.system.shutdown();
        let start = Instant::now();
        while virtual_clock::registered_actors() > self.actors_before {
            assert!(
                start.elapsed() < THREAD_END_HANG,
                "the converter's protocol thread outlived its system: {:?}",
                virtual_clock::registered_actor_names()
            );
            std::thread::sleep(Duration::from_millis(1));
        }
        findings
    }
}

/// `U25`'s pairs the run reported as read inside their band, as `(pin,
/// inverting)`, each with the fail-safe high it took.
fn u25_pairs_reported(findings: &[Finding]) -> Vec<(String, String)> {
    findings
        .iter()
        .filter_map(|finding| match finding {
            Finding::PairInBand {
                part,
                pin,
                inverting,
                output,
                ..
            } => {
                assert_eq!(part, "EDGE.U25", "{finding}");
                assert_eq!(*output, Level::High, "{finding}");
                Some((pin.clone(), inverting.clone()))
            }
            _ => None,
        })
        .collect()
}

/// A `U25` pair by its inputs' numbers.
fn pair(pin: &str, inverting: &str) -> (String, String) {
    (pin.to_string(), inverting.to_string())
}

#[rstest]
fn an_rs422_encoders_pairs_count_at_the_modules_encoder_pins() {
    behaviour!(Test {
        id: "edge.encoder-pairs-to-p9-p10",
        covers: Some("models/src/machine/quadrature_encoder.rs#QuadratureEncoder"),
        given: "the MaD machine's three boards as a project, an encoder presenting each channel \
                and its index as a complementary pair on the carrier's encoder connector, the \
                single-ended ground jumpers open and the receiver enabled, walked one count at \
                a time forward through two quadrature cycles and back through zero",
    });
    expect!(
        "quadrature-on-p9-p10",
        "at every count the module's P9 and P10 read the encoder's A and B levels, so each \
         count changes exactly one of them in the encoder's order",
        "the carrier's line receiver reads each pair's difference, plus or minus the \
         encoder's swing, as a valid level and the isolator after it passes the level to the \
         module"
    );
    expect!(
        "index-on-p11",
        "the module's P11 reads the encoder's index, high at the count where it fires and low \
         elsewhere"
    );
    expect!(
        "nothing-reported",
        "no pair of the line receiver is reported as read inside its band",
        "a pair's two legs change at one instant, so the receiver spends no time between \
         them"
    );
    let mut rig = Rig::start(Wiring::Pairs);
    let encoder = rig.encoder.clone();
    let expected = |rig: &Rig| {
        let (a, b, z) = encoder
            .driven_levels()
            .expect("the encoder has attached and driven its phase");
        assert_eq!(rig.pins(), (a, b, z), "at count {}", encoder.count());
    };
    expected(&rig);
    let mut previous = rig.pins();
    for count in WALK {
        let walking_from = encoder.count();
        let step = (count - walking_from).signum();
        let mut at = walking_from;
        while at != count {
            at += step;
            rig.walk_to(at);
            expected(&rig);
            let now = rig.pins();
            let changed = u8::from(now.0 != previous.0) + u8::from(now.1 != previous.1);
            assert_eq!(changed, 1, "one of P9 and P10 changes a count, at {at}");
            previous = now;
        }
    }
    // The index fired at counts 0 and 8, and the walk ended at -9, where
    // it does not.
    assert_eq!(rig.pins().2, Level::Low);
    rig.walk_to(-8);
    assert_eq!(rig.pins().2, Level::High, "the index at count -8");
    let findings = rig.finish();
    assert!(u25_pairs_reported(&findings).is_empty(), "{findings:?}");
}

#[rstest]
fn a_single_ended_encoder_reads_high_at_the_modules_encoder_pins() {
    behaviour!(Test {
        id: "edge.single-ended-encoder-fails-safe",
        covers: Some("models/src/am26lv32.rs#Am26lv32"),
        given: "the MaD machine's three boards as a project, an encoder driving each channel \
                single-ended on its pair's positive leg with the carrier's ground jumpers \
                closed on the negative legs, walked one count at a time through two \
                quadrature cycles",
    });
    expect!(
        "stuck-high",
        "the module's P9, P10 and P11 read high at every count",
        "a low on the positive leg leaves the pair with no differential, and the AM26LV32 \
         answers an input with no valid level with its fail-safe high (SLLS202H section \
         8.4.1, Table 8-1)"
    );
    expect!(
        "pairs-reported",
        "the run reports each of the line receiver's three wired pairs once as read inside \
         its band, with the high its output took"
    );
    let mut rig = Rig::start(Wiring::SingleEnded);
    let high = (Level::High, Level::High, Level::High);
    assert_eq!(rig.pins(), high, "at count 0, A and B low");
    for count in 1..=9 {
        rig.walk_to(count);
        assert_eq!(rig.pins(), high, "at count {count}");
    }
    let findings = rig.finish();
    assert_eq!(
        u25_pairs_reported(&findings),
        [pair("2", "1"), pair("6", "7"), pair("10", "9")],
        "{findings:?}"
    );
}

#[rstest]
fn an_open_collector_encoder_reads_high_at_the_modules_encoder_pins_and_is_reported() {
    behaviour!(Test {
        id: "edge.open-collector-encoder-fails-safe",
        covers: Some("models/src/machine/quadrature_encoder.rs#OutputStage::NpnOpenCollector"),
        given: "the MaD machine's three boards as a project, an encoder's NPN open-collector A \
                and B on the positive legs of the carrier's encoder connector, its ground \
                jumpers closed on the negative legs, walked through two quadrature cycles",
    });
    expect!(
        "stuck-high",
        "the module's P9 and P10 read high at every count",
        "the carrier has no pull-up on the positive legs: a released output rests at the \
         receiver's own input bias, a valid high against the grounded negative leg, and a \
         sinking one leaves the pair no difference, which the AM26LV32 reads as its fail-safe \
         high"
    );
    expect!(
        "pairs-reported",
        "the run reports the line receiver's A and B pairs once each as read inside the band \
         with a common-mode voltage applied, their outputs high",
        "SLLS202H guarantees no level for a pair inside 0.2 volts with an external \
         common-mode voltage applied (Table 8-1, section 8.4.1)"
    );
    expect!(
        "open-pairs-silent",
        "the line receiver's index pair and its unused fourth pair are not reported",
        "a pair nothing drives rests at the receiver's own bias, its guaranteed fail-safe"
    );
    let mut rig = Rig::start(Wiring::OpenCollector);
    let high = (Level::High, Level::High);
    let ab = |rig: &Rig| (rig.pins().0, rig.pins().1);
    assert_eq!(ab(&rig), high, "at count 0, A and B low");
    for count in 1..=9 {
        rig.walk_to(count);
        assert_eq!(ab(&rig), high, "at count {count}");
    }
    let findings = rig.finish();
    assert_eq!(
        u25_pairs_reported(&findings),
        [pair("2", "1"), pair("6", "7")],
        "{findings:?}"
    );
}
