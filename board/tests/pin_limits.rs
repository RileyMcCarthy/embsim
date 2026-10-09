//! Declared pin limits (`PinDecl::with_limits`, [`PinLimits`]): a part
//! declares the recommended operating range and absolute maximum its
//! datasheet gives a pin, and the **engine** checks them against the
//! solved net — `System::build` over its settled snapshot, the live engine
//! after every pass that moves the pin's net or its reference — raising
//! [`Finding::PinAboveRecommended`] once per excursion. The part under
//! test declares limits and does nothing else: it reads nothing and
//! reports nothing, so every finding here is the engine's.
//!
//! Bench components only: a scripted supply (`SUP.OUT`) and a held
//! reference (`SUP.REF`), harnessed to the part's `VCC` and `GND`. Stepped
//! (`TESTING.md` rule 9): a suite lock, the clock re-anchored stepped, the
//! system started with time held, the case's thread a registered actor,
//! one virtual settle past the script's last instant, no
//! `QuiescenceTimeout` at the end.

use std::sync::{Mutex, MutexGuard};

use embsim_board::{
    AttachError, Component, ComponentNetIo, Finding, Harness, PinDecl, PinLimits, System,
    TheveninDrive, Volts,
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

/// The limits the part declares: 3 V to 3.6 V recommended, 4 V absolute.
const LIMITS: PinLimits = PinLimits {
    recommended: (3.0, 3.6),
    absolute_max: Some(4.0),
    note: "a test note",
};

/// The script's instants, 1 ms apart, and the settle past its last.
const STEP_NS: u64 = 1_000_000;
const SETTLE_NS: u64 = 10 * STEP_NS;

/// The part under test: `VCC` measured against `GND`, declaring
/// [`LIMITS`], and nothing else — no sense, no drive.
struct Declares {
    pins: [PinDecl; 2],
}

impl Declares {
    fn new() -> Self {
        Self {
            pins: [
                PinDecl::power_in("VCC")
                    .with_reference("GND")
                    .with_limits(LIMITS),
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

fn rail(volts: Volts) -> TheveninDrive {
    TheveninDrive {
        volts,
        impedance: 1.0,
    }
}

/// The bench: `OUT` from `out` (released when `None`), then `script`'s
/// voltages one per millisecond; `REF` held at `reference`.
struct Bench {
    pins: [PinDecl; 2],
    script: Vec<Volts>,
}

impl Component for Bench {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }
    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let out = io.pin("OUT")?;
        let script = self.script.clone();
        for step in 1..=script.len() as u64 {
            io.schedule_at_ns(step * STEP_NS);
        }
        io.on_wake_ns(move |now_ns| {
            let step = (now_ns / STEP_NS) as usize;
            if now_ns % STEP_NS == 0 && (1..=script.len()).contains(&step) {
                out.set_drive(Some(rail(script[step - 1])));
            }
        });
        Ok(())
    }
}

fn system(out: Option<Volts>, reference: Volts, script: Vec<Volts>) -> System {
    let bench = Bench {
        pins: [
            PinDecl::power_out("OUT").with_idle(out.map(rail)),
            PinDecl::power_out("REF").with_idle(Some(rail(reference))),
        ],
        script,
    };
    System::new()
        .component("SUP", Box::new(bench))
        .component("DUT", Box::new(Declares::new()))
        .harness(
            Harness::new()
                .connect_str("SUP.OUT", "DUT.VCC")
                .expect("endpoints parse")
                .connect_str("SUP.REF", "DUT.GND")
                .expect("endpoints parse"),
        )
}

/// The engine's limit findings, in the order it raised them.
fn above(findings: &[Finding]) -> Vec<Finding> {
    findings
        .iter()
        .filter(|finding| matches!(finding, Finding::PinAboveRecommended { .. }))
        .cloned()
        .collect()
}

/// The finding for `DUT.VCC` at `volts` against its reference.
fn at(volts: Volts) -> Finding {
    Finding::PinAboveRecommended {
        part: "DUT".to_string(),
        pin: "VCC".to_string(),
        volts,
        min: 3.0,
        max: 3.6,
        absolute_max: Some(4.0),
        note: "a test note".to_string(),
    }
}

/// Build the bench, and run it live past its script: each path's limit
/// findings.
fn both_paths(
    out: Option<Volts>,
    reference: Volts,
    script: Vec<Volts>,
) -> (Vec<Finding>, Vec<Finding>) {
    let _lock = suite_lock();
    virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);
    let built = system(out, reference, script.clone())
        .build()
        .expect("the bench builds");
    let built = above(built.diagnostics().findings());
    let live = system(out, reference, script)
        .hold_time()
        .start()
        .expect("the bench starts");
    let actor = virtual_clock::register_actor("pin-limits-case");
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
    (built, above(&findings))
}

#[rstest]
#[case::inside_the_range(Some(3.3), 0.0, None)]
#[case::at_the_maximum(Some(3.6), 0.0, None)]
#[case::above_the_range(Some(3.75), 0.0, Some(3.75))]
#[case::beyond_the_absolute_maximum(Some(4.5), 0.0, Some(4.5))]
#[case::inside_against_its_reference(Some(4.75), 1.5, None)]
#[case::above_against_its_reference(Some(5.25), 1.5, Some(3.75))]
#[case::no_voltage(None, 0.0, None)]
fn the_engine_checks_a_pins_declared_limits_against_its_net(
    #[case] out: Option<Volts>,
    #[case] reference: Volts,
    #[case] reported: Option<Volts>,
) {
    behaviour!(Test {
        id: "engine.pin-limits-checked",
        covers: Some("board/src/limits.rs#LimitWatch::observe"),
        given: "a part declaring 3 to 3.6 volts recommended and 4 volts absolute on its supply \
                pin and doing nothing else, its supply and ground held or its supply released",
    });
    expect!(
        "above-reported",
        "a supply above 3.6 volts is reported once by the build and once live, with its voltage \
         and the declared limits",
        "the part declares what its datasheet allows, and the engine compares it with the \
         solved net"
    );
    expect!(
        "inside-silent",
        "a supply inside the range, exactly at 3.6 volts, or naming no voltage is not reported"
    );
    expect!(
        "against-reference",
        "measured against a 1.5 volt ground, 5.25 volts is reported at 3.75 volts and 4.75 \
         volts is not"
    );
    let (built, live) = both_paths(out, reference, Vec::new());
    let expected: Vec<Finding> = reported.into_iter().map(at).collect();
    assert_eq!(built, expected, "build");
    assert_eq!(live, expected, "live");
}

#[rstest]
fn the_engine_reports_each_excursion_once() {
    behaviour!(Test {
        id: "engine.pin-limits-once-per-excursion",
        covers: Some("board/src/engine.rs#EngineCore::check_limits"),
        given: "a part declaring 3 to 3.6 volts recommended on its supply pin, run live while \
                its supply moves from 3.3 to 5, 5.5, 3.3 and 3.9 volts",
    });
    expect!(
        "once-per-excursion",
        "the run reports the supply at 5 volts and then at 3.9 volts, and nothing else",
        "an excursion is reported at the first voltage above the range; a supply that stays \
         above is the same excursion, and one that returns and rises again is a new one"
    );
    let (built, live) = both_paths(Some(3.3), 0.0, vec![5.0, 5.5, 3.3, 3.9]);
    assert_eq!(built, Vec::new(), "build: the bench idles at 3.3 V");
    assert_eq!(live, vec![at(5.0), at(3.9)]);
}
