//! The `scripted-source` bench kind, live: a pin driven through steps at
//! their instants.
//!
//! The header board (`projects/header.net`: a connector `J1` and a 10 kΩ
//! resistor `R1` from `SIG` to `GND`) on the bench's 0 V, a scripted source
//! of 10 kΩ wired to its signal pin, and — added from Rust, as a scope — an
//! analog reader on the same pin that records every voltage it is handed
//! with the instant the engine handed it. The divider halves each step.
//!
//! Stepped (`TESTING.md` rule 9), its own binary: the clock stepped at 0,
//! the system started with time held (the source counts its steps from the
//! instant it starts), the case's thread a registered actor, every read at
//! an instant no wake lands on, no `QuiescenceTimeout` at the end.

use std::path::PathBuf;
use std::sync::{Arc, Mutex};

use embsim_board::{
    AttachError, Component, ComponentNetIo, EndpointRef, Finding, Harness, NetState, PinDecl,
    Project, SystemHandle,
};
use embsim_boards::catalog::CatalogSet;
use embsim_core::virtual_clock::{self, ClockMode};
use rstest::rstest;
use vibes_behaviour::{behaviour, expect, Test};

/// The header board's `R1`, and the source's own impedance: equal, so the
/// signal reads half of what the source drives.
const OHMS: f64 = 10_000.0;

/// The source's steps: instant (ns after the system starts) and volts.
const STEPS: [(u64, f64); 3] = [(1_000_000, 3.3), (2_000_000, 1.2), (3_000_000, 0.0)];

/// The project: the header board on the bench's 0 V, the source on its
/// signal pin.
const PROJECT: &str = r#"
[[board]]
name = "HDR"
kind = "netlist"
netlist = "header.net"

[[component]]
name = "SRC"
kind = "scripted-source"
[component.options]
ohms = 10000.0
steps = [["1ms", 3.3], ["2ms", 1.2], ["3ms", 0.0]]

[[wire]]
from = "SRC.OUT"
to = "HDR.J1.1"

[[wire]]
from = "BENCH.GND"
to = "HDR.J1.2"
volts = 0.0
"#;

/// Every voltage the scope's pin is handed, with its instant.
type Trace = Arc<Mutex<Vec<(u64, Option<f64>)>>>;

/// An analog reader recording what it is handed.
struct Scope {
    pins: [PinDecl; 1],
    trace: Trace,
}

impl Component for Scope {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let trace = Arc::clone(&self.trace);
        io.on_sense("IN", move |sense| {
            trace.lock().unwrap().push((sense.at_ns, sense.volts));
        })
    }
}

fn signal(system: &SystemHandle) -> NetState {
    system.net_state("HDR.SIG").expect("HDR.SIG is a net")
}

fn assert_reads(state: NetState, volts: f64, when: &str) {
    match state {
        NetState::Analog(v) => assert!((v - volts).abs() < 1e-9, "{when}: {v} V, not {volts} V"),
        other => panic!("{when}: {other:?}, not {volts} V"),
    }
}

#[rstest]
fn each_step_lands_at_its_own_instant_and_holds_until_the_next() {
    behaviour!(Test {
        id: "scripted-source.steps-at-their-instants",
        covers: Some("board/src/scripted_source.rs#ScriptedSource"),
        given: "a 10 kilohm scripted source into a 10 kilohm resistor to 0 volts, stepping to \
                3.3 volts at one millisecond, 1.2 at two and 0 at three",
    });
    expect!(
        "released-before",
        "before the first step the source drives nothing, and the pin reads the resistor's 0 \
         volts"
    );
    expect!(
        "lands-at-instant",
        "each step reaches the pin at exactly its instant, counted from the system's start, \
         divided between the source's impedance and the resistor",
        "a step is one drive published on a wake the source armed for its own instant"
    );
    expect!(
        "holds-between",
        "a nanosecond before each step the pin still reads the step before it",
        "after its last step the source holds what it drives"
    );
    let projects: PathBuf = [env!("CARGO_MANIFEST_DIR"), "projects"].iter().collect();
    let trace: Trace = Arc::new(Mutex::new(Vec::new()));
    let scope = Scope {
        pins: [PinDecl::analog("IN")],
        trace: Arc::clone(&trace),
    };

    virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);
    let system = Project::parse(PROJECT)
        .expect("the text is a project")
        .relative_to(projects)
        .instantiate(&CatalogSet::new())
        .expect("the project builds")
        .component("SCOPE", Box::new(scope))
        .harness(Harness::new().connect(
            EndpointRef::parse("SCOPE.IN").expect("endpoint parses"),
            EndpointRef::parse("HDR.J1.1").expect("endpoint parses"),
        ))
        .hold_time()
        .start()
        .expect("the bench starts");
    let actor = virtual_clock::register_actor("scripted-source-case");
    system.release_time();
    let origin = virtual_clock::virtual_ns();
    assert_eq!(origin, 0, "the system starts at the clock's 0");

    let mut before = 0.0;
    for (at_ns, volts) in STEPS {
        virtual_clock::wait_until_ns(at_ns - 1);
        assert_reads(signal(&system), before, &format!("1 ns before {at_ns} ns"));
        virtual_clock::wait_until_ns(at_ns + 1);
        let divided = volts * OHMS / (OHMS + OHMS);
        assert_reads(signal(&system), divided, &format!("1 ns after {at_ns} ns"));
        before = divided;
    }
    virtual_clock::wait_until_ns(STEPS[2].0 + 1_000_000);
    assert_reads(signal(&system), 0.0, "a millisecond after the last step");

    // What the scope was handed, after the build's own delivery: one
    // voltage per step, at the step's instant.
    let handed: Vec<(u64, f64)> = trace
        .lock()
        .unwrap()
        .iter()
        .filter(|(at_ns, _)| *at_ns > 0)
        .map(|(at_ns, volts)| (*at_ns, volts.expect("the pin is always reached")))
        .collect();
    let expected: Vec<(u64, f64)> = STEPS
        .iter()
        .map(|(at_ns, volts)| (*at_ns, volts / 2.0))
        .collect();
    assert_eq!(handed.len(), expected.len(), "{handed:?}");
    for ((at, volts), (want_at, want_volts)) in handed.iter().zip(&expected) {
        assert_eq!(at, want_at, "{handed:?}");
        assert!((volts - want_volts).abs() < 1e-9, "{handed:?}");
    }

    let stalled: Vec<Finding> = system
        .findings()
        .into_iter()
        .filter(|finding| matches!(finding, Finding::QuiescenceTimeout { .. }))
        .collect();
    assert!(
        stalled.is_empty(),
        "the engine stopped waiting for the case's thread: {stalled:?}"
    );
    drop(actor);
    system.shutdown();
}

#[rstest]
#[case::no_ohms(
    "steps = [[\"1ms\", 1.0]]",
    "options.ohms is the source's output impedance"
)]
#[case::zero_ohms(
    "ohms = 0\nsteps = [[\"1ms\", 1.0]]",
    "ohms = 0 is not a source impedance"
)]
#[case::no_steps("ohms = 50.0", "options.steps is a list of [\"instant\", volts] pairs")]
#[case::backwards(
    "ohms = 50.0\nsteps = [[\"2ms\", 1.0], [\"1ms\", 0.0]]",
    "step 1 lands at 1000000 ns, not after step 0 at 2000000 ns"
)]
#[case::no_unit("ohms = 50.0\nsteps = [[\"2\", 1.0]]", "has no unit")]
#[case::unknown(
    "ohms = 50.0\nsteps = [[\"1ms\", 1.0]]\nslope = 3",
    "unknown option \"slope\""
)]
fn a_script_that_names_no_impedance_or_no_order_is_refused(
    #[case] options: &str,
    #[case] says: &str,
) {
    behaviour!(Test {
        id: "scripted-source.refusals",
        covers: Some("boards/src/catalog.rs#scripted_source"),
        given: "a scripted source with no impedance, a zero impedance, no steps, steps out of \
                order, an instant without a unit, or an option the kind does not take",
    });
    expect!(
        "refused-saying-why",
        "the project is refused before it starts, naming the component and what to fix",
        "a source's impedance is the scenario's to name, and the kind has no default"
    );
    let text = format!(
        "[[component]]\nname = \"SRC\"\nkind = \"scripted-source\"\n[component.options]\n\
         {options}\n"
    );
    let message = Project::parse(&text)
        .expect("the text is a project")
        .instantiate(&CatalogSet::new())
        .expect_err("the script is refused")
        .to_string();
    assert!(
        message.contains("component SRC (kind \"scripted-source\")"),
        "{message}"
    );
    assert!(message.contains(says), "{says:?} missing from:\n{message}");
}
