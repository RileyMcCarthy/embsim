//! The `prop-plug-reset` bench kind and `host-serial`'s `DTR`, from a
//! project.
//!
//! A `host-serial` port, `A`, on a 3.3 V rail, its `DTR` wired to a Prop
//! Plug's reset stage, `PLUG`, whose open-collector `RES` a scripted source
//! pulls up through 10 kΩ, as a board's reset line is. Nothing opens the
//! PTY, so `DTR` rests where the port leaves it: deasserted, which is high
//! on an adapter with the usual active-low `DTR` and low with
//! `invert_dtr = true`. A level that never rises from low fires no pulse,
//! so `RES` stays pulled up and the plug's report says it never pulsed.
//! Scopes on `A.DTR` and `PLUG.RES` record every voltage they are handed.
//!
//! The refusals check the two kinds' option names: a `host-serial` whose
//! `invert_dtr` is not true or false, and a `prop-plug-reset` given an
//! option, since it takes none.
//!
//! Stepped (`TESTING.md` rule 9), its own binary: the clock stepped, the
//! system started with time held, the case's thread a registered actor.
//! The live cases take a suite lock: the virtual clock is process-global.

use std::path::PathBuf;
use std::sync::{Arc, Mutex, MutexGuard};

use embsim_board::{
    AttachError, Component, ComponentNetIo, EndpointRef, Harness, PinDecl, Project, Reports,
};
use embsim_boards::catalog::CatalogSet;
use embsim_core::virtual_clock::{self, ClockMode};
use rstest::rstest;
use vibes_behaviour::{behaviour, expect, Test};

/// How long the case runs: the rail and the pull-up are up well before.
const RUN_NS: u64 = 5_000_000;

/// One live case at a time: the virtual clock is process-global, and each
/// case re-anchors it stepped (`TESTING.md` rule 9).
static SUITE_LOCK: Mutex<()> = Mutex::new(());

fn suite_lock() -> MutexGuard<'static, ()> {
    SUITE_LOCK.lock().unwrap_or_else(|poisoned| {
        SUITE_LOCK.clear_poison();
        poisoned.into_inner()
    })
}

/// A PTY path of this test's own.
fn link(end: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("embsim-prop-plug-{}", std::process::id()));
    std::fs::create_dir_all(&dir).expect("the PTY directory can be made");
    dir.join(format!("tty.{end}"))
}

/// Every voltage a scope's pin is handed, with its instant.
type Trace = Arc<Mutex<Vec<(u64, Option<f64>)>>>;

/// An analog reader recording what it is handed.
struct Scope {
    pins: [PinDecl; 1],
    trace: Trace,
}

impl Scope {
    fn new() -> (Self, Trace) {
        let trace: Trace = Arc::default();
        (
            Self {
                pins: [PinDecl::analog("IN")],
                trace: Arc::clone(&trace),
            },
            trace,
        )
    }
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

/// The voltages a scope was handed after the build's own delivery.
fn handed(trace: &Trace) -> Vec<(u64, f64)> {
    trace
        .lock()
        .unwrap()
        .iter()
        .filter_map(|&(at_ns, volts)| volts.map(|volts| (at_ns, volts)))
        .collect()
}

fn ep(text: &str) -> EndpointRef {
    EndpointRef::parse(text).expect("endpoint parses")
}

#[rstest]
#[case::factory_polarity(false)]
#[case::inverted(true)]
fn a_resting_dtr_holds_the_plug_off_and_its_report_says_it_never_pulsed(#[case] inverted: bool) {
    behaviour!(Test {
        id: "prop-plug-reset.project",
        covers: Some("boards/src/catalog.rs#prop_plug_reset"),
        given: "a host serial port on a 3.3 volt rail, with the usual active-low DTR or set for \
                an adapter that inverts it, its DTR wired to a Prop Plug's reset stage whose \
                reset line is pulled up through 10 kilohms, and no host opening the port",
    });
    expect!(
        "dtr-rests-deasserted",
        "DTR rests deasserted at the port's rail: 3.3 volts, or 0 volts on the inverting adapter",
        "an adapter's DTR is active low unless its chip is set to invert it"
    );
    expect!(
        "res-stays-up",
        "the reset line reads its pull-up's 3.3 volts the whole run",
        "the stage pulses only on DTR rising from low, and a resting DTR never rises"
    );
    expect!("report-never", "the plug's summary says it never pulsed");

    let path = link(if inverted { "inverted" } else { "factory" });
    let invert = if inverted { "invert_dtr = true" } else { "" };
    let text = format!(
        r#"[[component]]
name = "A"
kind = "host-serial"
[component.options]
baud = 115200
path = {path:?}
{invert}

[[component]]
name = "PLUG"
kind = "prop-plug-reset"

[[component]]
name = "PULL"
kind = "scripted-source"
[component.options]
ohms = 10000.0
steps = [["1us", 3.3]]

[[wire]]
from = "A.DTR"
to = "PLUG.DTR"

[[wire]]
from = "PULL.OUT"
to = "PLUG.RES"

[[wire]]
from = "BENCH.3V3"
to = "A.VIO"
volts = 3.3

[[wire]]
from = "BENCH.GND"
to = "A.GND"
volts = 0.0

[[wire]]
from = "BENCH.GND"
to = "PLUG.VSS"
"#
    );
    let (dtr_scope, dtr) = Scope::new();
    let (res_scope, res) = Scope::new();

    let _suite = suite_lock();
    virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);
    let reports = Reports::new();
    let system = Project::parse(&text)
        .expect("the text is a project")
        .instantiate_with(&CatalogSet::new(), &reports)
        .expect("the bench builds")
        .component("DTRSCOPE", Box::new(dtr_scope))
        .component("RESSCOPE", Box::new(res_scope))
        .harness(
            Harness::new()
                .connect(ep("DTRSCOPE.IN"), ep("A.DTR"))
                .connect(ep("RESSCOPE.IN"), ep("PLUG.RES")),
        )
        .hold_time()
        .start()
        .expect("the bench starts");
    let actor = virtual_clock::register_actor("prop-plug-reset-case");
    system.release_time();
    virtual_clock::wait_until_ns(RUN_NS);

    let dtr = handed(&dtr);
    let resting = if inverted { 0.0 } else { 3.3 };
    assert!(
        !dtr.is_empty() && dtr.iter().all(|&(_, volts)| (volts - resting).abs() < 1e-6),
        "dtr-rests-deasserted: {dtr:?}"
    );
    let res = handed(&res);
    assert!(
        !res.is_empty() && res.iter().all(|&(_, volts)| (volts - 3.3).abs() < 1e-6),
        "res-stays-up: {res:?}"
    );
    let reports = reports.take();
    let plug = reports
        .iter()
        .find(|report| report.subject() == "PLUG")
        .expect("the plug reports");
    assert_eq!(
        plug.summary(),
        ["RES never pulsed".to_string()],
        "report-never"
    );
    drop(actor);
    system.shutdown();
}

#[rstest]
#[case::dtr_not_a_bool(
    "A",
    "host-serial",
    "baud = 115200\ninvert_dtr = \"yes\"",
    "options.invert_dtr is true or false"
)]
#[case::plug_takes_none(
    "PLUG",
    "prop-plug-reset",
    "pulse_us = 20",
    "unknown option \"pulse_us\""
)]
fn a_dtr_polarity_that_is_not_a_switch_or_an_option_the_plug_lacks_is_refused(
    #[case] name: &str,
    #[case] kind: &str,
    #[case] options: &str,
    #[case] says: &str,
) {
    behaviour!(Test {
        id: "prop-plug-reset.refusals",
        covers: Some("boards/src/catalog.rs#prop_plug_reset"),
        given: "a host serial port whose DTR polarity switch is set to the word yes, or a Prop \
                Plug's reset stage given a pulse length",
    });
    expect!(
        "refused-saying-why",
        "the project is refused before it starts, naming the component and what to fix",
        "the stage's pulse is the plug's own, so the kind takes no options"
    );
    let text = format!(
        "[[component]]\nname = \"{name}\"\nkind = \"{kind}\"\n[component.options]\n{options}\n"
    );
    let message = Project::parse(&text)
        .expect("the text is a project")
        .instantiate(&CatalogSet::new())
        .expect_err("the entry is refused")
        .to_string();
    assert!(
        message.contains(&format!("component {name} (kind \"{kind}\")")),
        "{message}"
    );
    assert!(message.contains(says), "{says:?} missing from:\n{message}");
}
