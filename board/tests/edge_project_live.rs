//! The MaD machine's three boards as a project, live:
//! `boards/projects/edge-ec32-ds2.toml` run from the bench supplies it
//! wires, past every soft-start, with the standard catalog alone: every
//! part of the three boards is the catalog's, the Edge board's RS-422 line
//! driver and receiver (`am26ls31`, `am26lv32`) included.
//!
//! Stepped (`TESTING.md` rule 9), its own binary: the clock stepped before
//! the project builds the converter (its protocol thread joins the clock as
//! an actor), the system started with time held, the case's thread a
//! registered actor, one virtual settle longer than every instant the
//! boards arm, no `QuiescenceTimeout` at the end.

use std::path::PathBuf;

use embsim_board::{Finding, NetState, Project};
use embsim_boards::catalog::StandardCatalog;
use embsim_boards::p2::P2_RESTART_DELAY_NS;
use embsim_core::virtual_clock::{self, ClockMode};
use embsim_models::ads122u04_component::PUMP_POLL_VIRTUAL_US;
use embsim_models::am26lv32::AM26LV32_SUPPLY_NOTE;
use embsim_models::oscillator::TG2520SMN_START_UP_NS;
use embsim_models::rail::{
    AP62301_SOFT_START_NS, AP62301_V_FB_VOLTS, UCC12040_VISO_SEL_TO_VISO_VOLTS, XL1509_5V0_VOLTS,
};
use rstest::rstest;
use vibes_behaviour::{behaviour, expect, Test};

/// The virtual time the run is handed before it reads: 10.1 ms, past the
/// longest chain the boards arm from their supplies — the module's bucks'
/// 2.5 ms soft-start, the P2's 3 ms restart and the TCXO's 1 ms start-up —
/// and off the converter pump's 250 µs polls, so no armed instant falls on
/// the deadline.
const SETTLE_NS: u64 = 10_100_000;
const _: () =
    assert!(SETTLE_NS > AP62301_SOFT_START_NS + P2_RESTART_DELAY_NS + TG2520SMN_START_UP_NS);
const _: () = assert!(!SETTLE_NS.is_multiple_of(PUMP_POLL_VIRTUAL_US * 1_000));

/// `U402`'s setpoint from its divider, `R401` 13.3 kΩ over `R403` 10.5 kΩ
/// at `V_FB` = 0.800 V: 1.8133 V.
const U402_V_SET: f64 = AP62301_V_FB_VOLTS * (1.0 + 13.3 / 10.5);

fn volts(state: Option<NetState>, net: &str) -> f64 {
    match state {
        Some(NetState::Analog(volts)) => volts,
        other => panic!("{net}: expected an analog voltage, got {other:?}"),
    }
}

#[rstest]
fn the_seated_module_and_the_add_on_run_from_the_carriers_rails() {
    behaviour!(Test {
        id: "project.edge-ec32-ds2-live",
        covers: Some("board/src/project.rs#Project::instantiate"),
        given: "the MaD machine's three boards as a project, the bench giving the carrier 12 \
                volts and its servo supply and nothing to the module, run live past every \
                soft-start",
    });
    expect!(
        "carrier-rail",
        "the carrier's 5 volt rail reads its buck's 5 volts"
    );
    expect!(
        "module-from-carrier",
        "the module's core rail reads its own buck's 1.813 volts, within a millivolt",
        "the module takes its 5 volts from the carrier's rail through the socket's two 5V \
         fingers, and its buck reads its feedback divider"
    );
    expect!(
        "add-on-from-carrier",
        "the add-on's digital supply reads the carrier's isolated 5 volts",
        "the carrier's isolated converter for the force domain is strapped for 5 volts and \
         reaches the add-on over the cable"
    );
    expect!(
        "receiver-over-range",
        "U25, the carrier's line receiver, is reported once for its supply at 5 volts, above \
         its recommended 3.6 volts",
        "the Edge board runs its AM26LV32 from 5 volts, inside its 6 volt absolute maximum \
         but above its recommended range, where its open-input bias is not characterised"
    );
    let path: PathBuf = [
        env!("CARGO_MANIFEST_DIR"),
        "..",
        "boards",
        "projects",
        "edge-ec32-ds2.toml",
    ]
    .iter()
    .collect();
    virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);
    let system = Project::load(&path)
        .expect("the project loads")
        .instantiate(&StandardCatalog)
        .expect("the project builds with the standard catalog alone")
        .hold_time()
        .start()
        .expect("the three boards start");
    let actor = virtual_clock::register_actor("edge-project-live-case");
    system.release_time();
    virtual_clock::wait_virtual_ns(SETTLE_NS);

    let carrier = volts(system.net_state("EDGE.+5V"), "EDGE.+5V");
    assert!(
        (carrier - XL1509_5V0_VOLTS).abs() < 1e-9,
        "EDGE.+5V reads {carrier}"
    );
    let core = volts(system.net_state("EC32.Common_VDD"), "EC32.Common_VDD");
    assert!((core - U402_V_SET).abs() < 1e-3, "Common_VDD reads {core}");
    let addon = volts(system.net_state("DS2.+3V3"), "DS2.+3V3");
    assert!(
        (addon - UCC12040_VISO_SEL_TO_VISO_VOLTS).abs() < 1e-9,
        "DS2.+3V3 reads {addon}"
    );

    let findings = system.findings();
    let over_range: Vec<&Finding> = findings
        .iter()
        .filter(|finding| matches!(finding, Finding::SupplyOutsideRecommended { .. }))
        .collect();
    assert_eq!(
        over_range,
        [&Finding::SupplyOutsideRecommended {
            part: "EDGE.U25".to_string(),
            pin: "16".to_string(),
            volts: XL1509_5V0_VOLTS,
            min: 3.0,
            max: 3.6,
            note: AM26LV32_SUPPLY_NOTE.to_string(),
        }],
        "{findings:?}"
    );

    let stalled: Vec<Finding> = findings
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
