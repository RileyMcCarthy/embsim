//! The P2-EC32MB netlist project live: its power tree from the carrier's
//! fingers, as `board/tests/power_tree.rs` holds the module's.
//!
//! `ec32-netlist.toml` builds the module from its netlist and powers it the
//! way a carrier does — 5 V on `J203` fingers 41/42, 0 V on 43–45 — in its
//! own `[[wire]]`s. Run past the bucks' soft-start, every bank rail and both
//! buck rails read what the module as the board library builds it reads
//! under the same supply, and what `power_tree.rs` asserts of it.
//!
//! Stepped (`TESTING.md` rule 9), its own binary: a suite lock, the clock
//! re-anchored stepped, the system started with time held, the case's
//! thread a registered actor, one virtual settle longer than every instant
//! the module arms, no `QuiescenceTimeout` at the end.

use std::collections::BTreeMap;
use std::path::PathBuf;
use std::sync::{Mutex, MutexGuard};

use embsim_board::{EndpointRef, Finding, Harness, NetState, Project, System, SystemHandle};
use embsim_boards::catalog::StandardCatalog;
use embsim_boards::ec32mb::Ec32mb;
use embsim_boards::p2::{P2Package, P2_RESTART_DELAY_NS};
use embsim_core::virtual_clock::{self, ClockMode};
use embsim_models::oscillator::TG2520SMN_START_UP_NS;
use embsim_models::rail::{AP62301_SOFT_START_NS, AP62301_V_FB_VOLTS};
use rstest::rstest;
use vibes_behaviour::{behaviour, expect, Test};

static SUITE_LOCK: Mutex<()> = Mutex::new(());

fn suite_lock() -> MutexGuard<'static, ()> {
    SUITE_LOCK.lock().unwrap_or_else(|poisoned| {
        SUITE_LOCK.clear_poison();
        poisoned.into_inner()
    })
}

/// The virtual time a run is handed before its rails are read: 10 ms,
/// longer than the longest chain the module arms from its supply — the
/// bucks' 2.5 ms soft-start, then the P2's 3 ms restart after its reset
/// releases, and the TCXO's 1 ms start-up — so the read is the module at
/// rest and no armed instant falls on the deadline.
const SETTLE_NS: u64 = 10_000_000;
const _: () =
    assert!(SETTLE_NS > AP62301_SOFT_START_NS + P2_RESTART_DELAY_NS + TG2520SMN_START_UP_NS);

/// The module's eight bank rails, by net.
const VIO_RAILS: [&str; 8] = [
    "VIO_00_07",
    "VIO_08_15",
    "VIO_16_23",
    "VIO_24_31",
    "VIO_32_39",
    "VIO_40_47",
    "VIO_48_55",
    "VIO_56_63",
];

/// The LDOs' 3.3 V, from their value.
const LDO_V_SET: f64 = 3.3;
/// `U402`'s setpoint from its divider, `R401` 13.3 kΩ over `R403` 10.5 kΩ
/// at `V_FB` = 0.800 V: 1.8133 V.
const U402_V_SET: f64 = AP62301_V_FB_VOLTS * (1.0 + 13.3 / 10.5);
/// `U403`'s, `R402` 37.4 kΩ over `R404` 10.5 kΩ: 3.6495 V.
const U403_V_SET: f64 = AP62301_V_FB_VOLTS * (1.0 + 37.4 / 10.5);

fn ep(endpoint: &str) -> EndpointRef {
    EndpointRef::parse(endpoint).expect("endpoint parses")
}

/// Start `system` held, join its clock as an actor, release time, settle
/// past every soft-start, and read every module rail; then check the engine
/// never stopped waiting for the case and shut the system down.
fn rails_at_rest(system: System) -> BTreeMap<String, NetState> {
    virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);
    let system: SystemHandle = system.hold_time().start().expect("the module starts");
    let actor = virtual_clock::register_actor("ec32-project-power-case");
    system.release_time();
    virtual_clock::wait_virtual_ns(SETTLE_NS);

    let rails = VIO_RAILS
        .iter()
        .map(|rail| rail.to_string())
        .chain(["Common_VDD".to_string(), "Common_LDOin".to_string()])
        .map(|rail| {
            let state = system
                .net_state(&format!("EC32.{rail}"))
                .unwrap_or_else(|| panic!("{rail} is a net"));
            (rail, state)
        })
        .collect();

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
    rails
}

fn volts(rails: &BTreeMap<String, NetState>, rail: &str) -> f64 {
    match rails[rail] {
        NetState::Analog(volts) => volts,
        other => panic!("{rail}: expected an analog voltage, got {other:?}"),
    }
}

#[rstest]
fn the_ec32_netlist_project_powers_its_rails_from_the_carrier_fingers() {
    behaviour!(Test {
        id: "project.ec32-netlist-power-tree",
        covers: Some("board/src/project.rs#Project::instantiate"),
        given: "the P2-EC32MB built from its netlist by a project whose wires put 5 volts \
                on its two 5V edge fingers and 0 volts on its three GND fingers, run past \
                the soft-start",
    });
    expect!(
        "bank-rails",
        "every one of the eight bank rails reads 3.3 volts within a millivolt",
        "the low-dropout regulators' value names 3.3 volts, and their input, the second \
         buck's rail, is up"
    );
    expect!(
        "buck-rails",
        "the core rail reads 1.813 volts and the regulators' input rail 3.649 volts, each \
         within a millivolt",
        "each buck reads its own feedback divider from the netlist when it attaches"
    );
    expect!(
        "same-as-the-library-module",
        "every rail reads exactly what the module as the board library builds it reads under \
         the same supply"
    );
    let _suite = suite_lock();
    let path: PathBuf = [env!("CARGO_MANIFEST_DIR"), "projects", "ec32-netlist.toml"]
        .iter()
        .collect();
    let project = Project::load(&path).expect("the project loads");
    let from_project = rails_at_rest(
        project
            .instantiate(&StandardCatalog)
            .expect("the project builds"),
    );

    for rail in VIO_RAILS {
        let v = volts(&from_project, rail);
        assert!((v - LDO_V_SET).abs() < 1e-3, "{rail} reads {v}");
    }
    let core = volts(&from_project, "Common_VDD");
    assert!((core - U402_V_SET).abs() < 1e-3, "Common_VDD reads {core}");
    assert!((core - 1.813).abs() < 1e-3, "Common_VDD reads {core}");
    let ldo_in = volts(&from_project, "Common_LDOin");
    assert!(
        (ldo_in - U403_V_SET).abs() < 1e-3,
        "Common_LDOin reads {ldo_in}"
    );
    assert!((ldo_in - 3.649).abs() < 1e-3, "Common_LDOin reads {ldo_in}");

    let library = Ec32mb::new()
        .with_p2(|_decl| Box::new(P2Package::held_in_reset()))
        .build()
        .expect("the module builds");
    let from_library = rails_at_rest(
        System::new().board("EC32", library).harness(
            Harness::new()
                .power(ep("CARRIER.5V"), ep("EC32.J203.41"), 5.0)
                .power(ep("CARRIER.5Vb"), ep("EC32.J203.42"), 5.0)
                .power(ep("CARRIER.GND"), ep("EC32.J203.43"), 0.0)
                .power(ep("CARRIER.GNDb"), ep("EC32.J203.44"), 0.0)
                .power(ep("CARRIER.GNDc"), ep("EC32.J203.45"), 0.0),
        ),
    );
    assert_eq!(from_project, from_library);
}
