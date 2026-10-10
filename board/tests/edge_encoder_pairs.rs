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
//! The encoder is `embsim_models::machine::QuadratureEncoder`: with its
//! complements, the pair MaD's machine presents, read by the receiver as a
//! valid differential each way; without them, single-ended on the `+` legs
//! with the jumpers closed, each low has no differential and reads as the
//! receiver's fail-safe high (SLLS202H §8.4.1, Table 8-1), which is why
//! `MIGRATING-MAD.md` §4 wires the pairs.
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
use embsim_models::machine::{IndexConfig, QuadratureEncoder};
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
#[derive(Debug, Clone, Copy)]
enum Wiring {
    /// Each channel a pair, `JP2`, `JP3` and `JP5` open.
    Pairs,
    /// Each channel on its `+` leg, `JP2`, `JP3` and `JP5` closed.
    SingleEnded,
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
        let mut config = EncoderConfig::new(1.0).with_index(INDEX);
        if matches!(wiring, Wiring::Pairs) {
            config = config.with_complements();
        }
        let encoder = QuadratureEncoder::new(config).expect("a valid encoder");
        let input = encoder.input();
        let ep = |endpoint: &str| EndpointRef::parse(endpoint).expect("endpoint parses");
        let mut harness = Harness::new()
            .connect(ep("ENC.A"), ep("EDGE.J20.1"))
            .connect(ep("ENC.B"), ep("EDGE.J20.3"))
            .connect(ep("ENC.Z"), ep("EDGE.J20.9"));
        // `JP4` asserts `U25`'s active-low enable either way.
        let mut scenario = Scenario::default().jumper("EDGE.JP4", JumperState::Closed);
        let legs = match wiring {
            Wiring::Pairs => {
                harness = harness
                    .connect(ep("ENC.A-"), ep("EDGE.J20.2"))
                    .connect(ep("ENC.B-"), ep("EDGE.J20.4"))
                    .connect(ep("ENC.Z-"), ep("EDGE.J20.10"));
                JumperState::Open
            }
            Wiring::SingleEnded => JumperState::Closed,
        };
        for jumper in ["EDGE.JP2", "EDGE.JP3", "EDGE.JP5"] {
            scenario = scenario.jumper(jumper, legs);
        }
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

    fn finish(mut self) {
        drop(self.actor.take());
        let stalled: Vec<Finding> = self
            .system
            .findings()
            .into_iter()
            .filter(|finding| matches!(finding, Finding::QuiescenceTimeout { .. }))
            .collect();
        assert!(
            stalled.is_empty(),
            "the engine stopped waiting for the case's thread: {stalled:?}"
        );
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
    }
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
    rig.finish();
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
    let mut rig = Rig::start(Wiring::SingleEnded);
    let high = (Level::High, Level::High, Level::High);
    assert_eq!(rig.pins(), high, "at count 0, A and B low");
    for count in 1..=9 {
        rig.walk_to(count);
        assert_eq!(rig.pins(), high, "at count {count}");
    }
    rig.finish();
}
