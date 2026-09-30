//! The DS2 force-gauge add-on as a project: its KiCad netlist export, its
//! ADS122U04 assigned by part name, the bench supplies its own tests use
//! (`board/tests/ds2_regressions.rs`), all in `projects/ds2-addon.toml`.
//!
//! Live and stepped, its own binary: the ADC's protocol thread is a
//! registered clock actor for the rest of the process (`TESTING.md` rule
//! 5), so the clock is stepped before the project builds the part, and the
//! case's thread joins as an actor before time is released.

use std::path::PathBuf;

use embsim_board::{Finding, NetState, Project, SenseKind};
use embsim_boards::catalog::StandardCatalog;
use embsim_core::virtual_clock::{self, ClockMode};
use embsim_models::ads122u04_component::PUMP_POLL_VIRTUAL_US;
use rstest::rstest;
use vibes_behaviour::{behaviour, expect, Test};

/// The virtual time the case hands the engine before it reads: 1.1 ms,
/// four and a bit of the ADC pump's 250 µs polls, so no poll falls on the
/// deadline. What the case reads — the supplies and the build's findings —
/// is fixed from the instant the system assembles; the window is the
/// harness's, and any span reads the same.
const SETTLE_NS: u64 = 1_100_000;
const _: () = assert!(!SETTLE_NS.is_multiple_of(PUMP_POLL_VIRTUAL_US * 1_000));

/// The bench's 3.3 V, on both of the add-on's supplies.
const BENCH_VOLTS: f64 = 3.3;

#[rstest]
fn the_ds2_project_powers_the_adc_from_its_bench_supplies() {
    behaviour!(Test {
        id: "project.ds2-addon-bench",
        covers: Some("board/src/project.rs#Project::instantiate"),
        given: "the force-gauge add-on built from its KiCad netlist by a project file, its \
                converter assigned by part name, 3.3 volts and ground wired onto its digital \
                and analog supply connectors, run live",
    });
    expect!(
        "converter-is-the-model",
        "the converter is a live part of the running system",
        "the converter is the one part the netlist leaves unclassified, and the file assigns \
         it by its symbol's part name"
    );
    expect!(
        "digital-supply",
        "the converter's digital supply rail reads the bench's 3.3 volts"
    );
    expect!(
        "analog-supply",
        "the converter's analog supply rail reads the bench's 3.3 volts",
        "the add-on brings its analog domain out on its second connector and generates \
         nothing on it"
    );
    expect!(
        "no-supply-lint",
        "no supply net is reported unsourced, no domain is reported measured against \
         nothing, and no regulator is reported down"
    );
    expect!(
        "floating-reset-found",
        "the converter's reset input is reported floating",
        "the board leaves the converter's active-low reset on a net no other pin shares, as \
         the add-on's own build tests find"
    );
    // Stepped before the project builds the converter, whose protocol
    // thread joins the clock the moment it exists.
    virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);
    let path: PathBuf = [env!("CARGO_MANIFEST_DIR"), "projects", "ds2-addon.toml"]
        .iter()
        .collect();
    let system = Project::load(&path)
        .expect("the project loads")
        .instantiate(&StandardCatalog)
        .expect("the project builds")
        .hold_time()
        .start()
        .expect("the add-on starts");
    let actor = virtual_clock::register_actor("ds2-project-case");
    system.release_time();
    virtual_clock::wait_virtual_ns(SETTLE_NS);

    assert!(
        system
            .component_refs()
            .any(|reference| reference.ends_with("U1")),
        "U1 is a component: {:?}",
        system.component_refs().collect::<Vec<_>>()
    );
    assert_eq!(
        system.net_state("DS2Addon.+3V3"),
        Some(NetState::Analog(BENCH_VOLTS))
    );
    assert_eq!(
        system.net_state("DS2Addon.VDDA"),
        Some(NetState::Analog(BENCH_VOLTS))
    );

    let findings = system.findings();
    let lints: Vec<&Finding> = findings
        .iter()
        .filter(|finding| {
            matches!(
                finding,
                Finding::PowerNetUnsourced { .. }
                    | Finding::UnreferencedDomain { .. }
                    | Finding::RailDown { .. }
            )
        })
        .collect();
    assert_eq!(lints, Vec::<&Finding>::new());
    assert!(
        findings.contains(&Finding::FloatingSense {
            net: "DS2Addon.~RESET".to_string(),
            kind: SenseKind::Digital,
        }),
        "the one-pin reset net is reported floating; got {findings:?}"
    );
    let stalled: Vec<&Finding> = findings
        .iter()
        .filter(|finding| matches!(finding, Finding::QuiescenceTimeout { .. }))
        .collect();
    assert!(
        stalled.is_empty(),
        "the engine stopped waiting for the case's thread: {stalled:?}"
    );
    drop(actor);
    system.shutdown();
}
