//! The engine's watch on a declared differential pair
//! (`PinDecl::with_pair`, [`DifferentialPair`], `NODES.md` §20): a part
//! declares the pair and the band its datasheet gives no level in, and the
//! **engine** watches it — `System::build` over its settled snapshot, the
//! live engine after every pass that moves either input or their reference
//! — raising [`Finding::PairInBand`] once per pair. The part under test
//! declares a pair and does nothing else: it reads nothing and reports
//! nothing, so every finding here is the engine's.
//!
//! Bench components only: a scripted bench holding the pair's two inputs
//! (`SUP.P`, `SUP.N`) and their reference (`SUP.REF`), harnessed to the
//! part's `P`, `N` and `GND`, nothing else on those nets, so each reads
//! exactly what the bench holds it at. Stepped (`TESTING.md` rule 9): a
//! suite lock, the clock re-anchored stepped, the system started with time
//! held, the case's thread a registered actor, one virtual settle past the
//! script's last instant, no `QuiescenceTimeout` at the end.

use std::sync::{Mutex, MutexGuard};

use embsim_board::{
    AttachError, Component, ComponentNetIo, DifferentialPair, Finding, Harness, Level, PinDecl,
    System, TheveninDrive, Volts,
};
use embsim_core::virtual_clock::{self, ClockMode};
use rstest::rstest;
use vibes_behaviour::{behaviour, expect, Test};

static SUITE_LOCK: Mutex<()> = Mutex::new(());

fn suite_lock() -> MutexGuard<'static, ()> {
    SUITE_LOCK.lock().unwrap_or_else(|poisoned| {
        SUITE_LOCK.clear_poison();
        poisoned.into_inner()
    })
}

/// The band the part declares: ±0.2 V, the fail-safe high inside it.
const BAND: Volts = 0.2;

/// The script's instants, 1 ms apart, and the settle past its last.
const STEP_NS: u64 = 1_000_000;
const SETTLE_NS: u64 = 10 * STEP_NS;

/// The part under test: `P` read against `N`, both measured against
/// `GND`, declaring the pair and nothing else — no port, no sense, no
/// drive.
struct Declares {
    pins: [PinDecl; 3],
}

impl Declares {
    fn new() -> Self {
        Self {
            pins: [
                PinDecl::analog("P")
                    .with_reference("GND")
                    .with_pair(DifferentialPair {
                        inverting: "N",
                        band: BAND,
                        fail_safe: Level::High,
                    }),
                PinDecl::analog("N").with_reference("GND"),
                PinDecl::power_in("GND"),
            ],
        }
    }
}

impl Component for Declares {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }
    fn attach(&mut self, _io: ComponentNetIo) -> Result<(), AttachError> {
        Ok(())
    }
}

fn held(volts: Volts) -> TheveninDrive {
    TheveninDrive {
        volts,
        impedance: 1.0,
    }
}

/// The bench's pins, in the order [`Idle`] and the script name them.
const BENCH_PINS: [&str; 3] = ["P", "N", "REF"];

/// What the bench holds `P`, `N` and `REF` at before its script (`None`
/// released).
type Idle = [Option<Volts>; 3];

/// One script entry: at step `.0` (one per millisecond), bench pin `.1`
/// held at `.2`.
type Step = (u64, &'static str, Volts);

/// The bench: [`BENCH_PINS`] from their idle, then the script's entries,
/// each step's in order.
struct Bench {
    pins: [PinDecl; 3],
    script: Vec<Step>,
}

impl Component for Bench {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }
    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let handles = [io.pin("P")?, io.pin("N")?, io.pin("REF")?];
        let script = self.script.clone();
        let mut steps: Vec<u64> = script.iter().map(|&(step, _, _)| step).collect();
        steps.dedup();
        for step in steps {
            io.schedule_at_ns(step * STEP_NS);
        }
        io.on_wake_ns(move |now_ns| {
            if now_ns % STEP_NS != 0 {
                return;
            }
            let step = now_ns / STEP_NS;
            for &(at, pin, volts) in &script {
                if at != step {
                    continue;
                }
                let index = BENCH_PINS
                    .iter()
                    .position(|&name| name == pin)
                    .expect("a bench pin");
                handles[index].set_drive(Some(held(volts)));
            }
        });
        Ok(())
    }
}

fn system(idle: Idle, script: Vec<Step>) -> System {
    let pin = |index: usize| PinDecl::power_out(BENCH_PINS[index]).with_idle(idle[index].map(held));
    let bench = Bench {
        pins: [pin(0), pin(1), pin(2)],
        script,
    };
    System::new()
        .component("SUP", Box::new(bench))
        .component("DUT", Box::new(Declares::new()))
        .harness(
            Harness::new()
                .connect_str("SUP.P", "DUT.P")
                .expect("endpoints parse")
                .connect_str("SUP.N", "DUT.N")
                .expect("endpoints parse")
                .connect_str("SUP.REF", "DUT.GND")
                .expect("endpoints parse"),
        )
}

/// The engine's pair findings, in the order it raised them.
fn in_band(findings: &[Finding]) -> Vec<Finding> {
    findings
        .iter()
        .filter(|finding| matches!(finding, Finding::PairInBand { .. }))
        .cloned()
        .collect()
}

/// Build the bench, and run it live past its script: each path's pair
/// findings.
fn both_paths(idle: Idle, script: Vec<Step>) -> (Vec<Finding>, Vec<Finding>) {
    let _lock = suite_lock();
    virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);
    let built = system(idle, script.clone())
        .build()
        .expect("the bench builds");
    let built = in_band(built.diagnostics().findings());
    let live = system(idle, script)
        .hold_time()
        .start()
        .expect("the bench starts");
    let actor = virtual_clock::register_actor("pair-watch-case");
    live.release_time();
    virtual_clock::wait_virtual_ns(SETTLE_NS);
    let findings = live.findings();
    let stalled: Vec<&Finding> = findings
        .iter()
        .filter(|finding| matches!(finding, Finding::QuiescenceTimeout { .. }))
        .collect();
    assert!(
        stalled.is_empty(),
        "the engine stopped waiting: {stalled:?}"
    );
    drop(actor);
    live.shutdown();
    (built, in_band(&findings))
}

/// Assert `found` is exactly one finding for `DUT`'s pair, `volts` apart
/// at `common_mode` against its reference.
fn assert_one(found: &[Finding], volts: Volts, common_mode: Volts) {
    let [Finding::PairInBand {
        part,
        pin,
        inverting,
        volts: read,
        common_mode: mode,
        band,
        output,
    }] = found
    else {
        panic!("one finding, not {found:?}");
    };
    assert_eq!(
        (part.as_str(), pin.as_str(), inverting.as_str()),
        ("DUT", "P", "N")
    );
    assert!((read - volts).abs() < 1e-9, "the difference {read} V");
    assert!(
        (mode - common_mode).abs() < 1e-9,
        "the common mode {mode} V"
    );
    assert_eq!(*band, BAND);
    assert_eq!(*output, Level::High);
}

#[rstest]
fn the_engine_reports_a_pair_once_whatever_it_reads_after() {
    behaviour!(Test {
        id: "engine.pair-in-band-once",
        covers: Some("board/src/pairs.rs#PairWatch::judge"),
        given: "a part that only declares a differential pair with a 0.2 volt band, the bench \
                moving its inputs from a volt apart to 0.1, 0.05, 1 and then 0 volts apart",
    });
    expect!(
        "first-reading-only",
        "the run reports the pair once, with the first difference and common-mode voltage it \
         held inside the band, and nothing for its later readings there",
        "the part's output takes the same fail-safe level at every reading inside the band, \
         so an encoder's every low would otherwise be a report of its own"
    );
    let (_, live) = both_paths(
        [Some(2.0), Some(1.0), Some(0.0)],
        vec![(1, "P", 1.1), (2, "P", 1.05), (3, "P", 2.0), (4, "P", 1.0)],
    );
    assert_one(&live, 1.1 - 1.0, (1.1 + 1.0) / 2.0);
}

#[rstest]
fn the_engine_reads_a_pair_again_when_only_its_reference_moves() {
    behaviour!(Test {
        id: "engine.pair-reference-watched",
        covers: Some("board/src/engine.rs#EngineCore::watch_pairs"),
        given: "a part that only declares a differential pair against its ground, the bench \
                holding the inputs at 1.1 and 1.0 volts and the ground at no voltage, then at \
                0.25 volts",
    });
    expect!(
        "unmeasured-silent",
        "while the ground has no voltage, the build reports nothing about the pair",
        "a pair measured against a ground with no voltage has no common-mode voltage to name"
    );
    expect!(
        "reported-when-ground-held",
        "once the ground is held, the run reports the pair with its common-mode voltage \
         measured from the ground, 0.8 volts",
        "the engine reads a pair again when its reference's net moves, as when either input's \
         does"
    );
    let (built, live) = both_paths([Some(1.1), Some(1.0), None], vec![(1, "REF", 0.25)]);
    assert_eq!(built, Vec::new(), "build: {built:?}");
    assert_one(&live, 1.1 - 1.0, (1.1 + 1.0) / 2.0 - 0.25);
}
