//! The power tree as rails — the parts half of `NODES.md` §8 phase 4, on
//! the wire.
//!
//! Every regulator on the three reference boards is a model now: the
//! P2-EC32MB's two AP62301 bucks (one registry key, two setpoints from
//! their own feedback dividers), its eight NCP114 LDOs and its STM1061
//! brownout detector; the MaD Edge board's two XL1509 bucks and its two
//! UCC12040 isolated DC/DCs. A rail's output is a declared terminal that
//! publishes `V(reference) + v_set` from the instant its input and enable
//! allow plus the datasheet's soft-start — a real scheduled instant — and
//! is released otherwise, with the build naming why. These cases hold
//! that on the boards themselves:
//!
//! * the module from its two `J203` fingers and nothing stuck: every
//!   `VIO_a_b` at 3.3 V within a millivolt and `Common_VDD` at 1.813 V,
//!   from the instant the bucks' 2.5 ms soft-start elapses, `U402` and
//!   `U403` different from one key;
//! * the P2's reset: floating while both its pull-up rail and the
//!   detector's supply are down, pulled up from the instant the rails
//!   rise, and sunk by the detector while the core rail is held under
//!   its threshold — and, with one pad of the pull-up lifted, floating
//!   after the rails rise too, reported by the P2's `RESN` sense (the
//!   "module never boots" failure a carrier sees); the debug-serial pins
//!   beside the transmit pin read at their 100 kΩ pull-ups once the bank
//!   rail is up;
//! * a domain measured against nothing is reported — the servo isolator's
//!   unwired secondary ground on the Edge board — and the one-wire fix
//!   clears it;
//! * a mechanical pad on a net a pin drives is reported, and the two
//!   boards' holes are not;
//! * a supply pin with no capacitor between its node and its reference's
//!   is reported, one across them clears it, and neither reference board
//!   raises it;
//! * a rail with no input is reported down naming the input;
//! * the current instrument on a bench sink against a module pull-up reads
//!   `(3.3 − v) / R` to a nanoamp;
//! * the build snapshot is the live system's state before its first wake
//!   under each board's reference harness — every rail down in both.
//!
//! The assembled machine — the add-on on its cable, whose ADC starts a
//! protocol thread that lives for the rest of the process (`TESTING.md`
//! rule 5) — is `power_tree_machine.rs`, its own binary; the add-on alone
//! under its rails is asserted in `ds2_regressions.rs`.
//!
//! Stepped mode throughout (`TESTING.md` rule 9), own binary.

use std::collections::HashMap;
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};

use embsim_board::{
    netlist, AttachError, Board, Component, ComponentNetIo, EndpointRef, Finding, Harness, Level,
    NetId, NetState, PartRegistry, PinDecl, PinHandle, PinReference, RailDownReason, Scenario,
    SenseKind, System, SystemHandle, TheveninDrive,
};
use embsim_boards::ec32mb::{Ec32mb, BROWNOUT_DETECTOR_PART, BUCK_PART, LDO_PART, NETLIST};
use embsim_boards::p2::P2Package;
use embsim_core::virtual_clock::{self, ClockMode};
use embsim_models::rail::{
    self, Rail, RailMonitor, RailState, AP62301_PINS_BY_FUNCTION, AP62301_SOFT_START_NS,
    AP62301_V_FB_VOLTS, NCP114_PINS_BY_FUNCTION,
};
use embsim_models::supervisor::{self, DetectorMonitor, VoltageDetector, STM1061_PINS_BY_FUNCTION};
use rstest::rstest;
use vibes_behaviour::{behaviour, expect, Test};

mod machine_parts;
use machine_parts::{bench_rails, edge_board, shipped_ec32mb_board};

// ============================================================
// Plumbing
// ============================================================

static SUITE_LOCK: Mutex<()> = Mutex::new(());

fn suite_lock() -> MutexGuard<'static, ()> {
    SUITE_LOCK.lock().unwrap_or_else(|poisoned| {
        SUITE_LOCK.clear_poison();
        poisoned.into_inner()
    })
}

fn stepped() -> MutexGuard<'static, ()> {
    let guard = suite_lock();
    virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);
    guard
}

fn wait_for(mut pred: impl FnMut() -> bool, timeout: Duration) -> bool {
    let start = Instant::now();
    while start.elapsed() < timeout {
        if pred() {
            return true;
        }
        std::thread::sleep(Duration::from_millis(1));
    }
    pred()
}

const SETTLE: Duration = Duration::from_secs(5);

fn ep(endpoint: &str) -> EndpointRef {
    EndpointRef::parse(endpoint).expect("endpoint parses")
}

/// Bitwise state equality (a NaN-carrying state never equals itself under
/// the derived `PartialEq`).
fn same_state(a: NetState, b: NetState) -> bool {
    match (a, b) {
        (NetState::Analog(x), NetState::Analog(y)) => x.total_cmp(&y).is_eq(),
        (NetState::Pulled(la, xa), NetState::Pulled(lb, xb)) => {
            la == lb && xa.total_cmp(&xb).is_eq()
        }
        _ => a == b,
    }
}

/// Every module rail at its setpoint: the eight bank rails at 3.3 V and
/// the two buck rails at their dividers', within a millivolt. What a case
/// waits on before it reads any of them — the LDOs publish one after
/// another as their senses are delivered, and an LDO whose `IN` and `EN`
/// share the buck's rail publishes its active discharge (0 V) between the
/// two deliveries at the same instant — so a poller waiting on one rail
/// reading anything can see another still floating, or the 0 V.
fn module_rails_up(system: &SystemHandle) -> bool {
    let at = |net: &str, volts: f64| matches!(system.net_state(net), Some(NetState::Analog(v)) if (v - volts).abs() < 1e-3);
    VIO_RAILS
        .iter()
        .all(|rail| at(&format!("EC32MB.{rail}"), LDO_V_SET))
        && at("EC32MB.Common_VDD", U402_V_SET)
        && at("EC32MB.Common_LDOin", U403_V_SET)
}

/// The module's rails as they read now, for a failure message.
fn module_rail_states(system: &SystemHandle) -> Vec<String> {
    VIO_RAILS
        .iter()
        .map(|rail| format!("EC32MB.{rail}"))
        .chain([
            "EC32MB.Common_VDD".to_string(),
            "EC32MB.Common_LDOin".to_string(),
        ])
        .map(|net| format!("{net}={:?}", system.net_state(&net)))
        .collect()
}

/// The voltage a live net reads, or a panic naming what it read instead.
fn volts(system: &SystemHandle, net: &str) -> f64 {
    match system.net_state(net) {
        Some(NetState::Analog(v)) => v,
        other => panic!("{net}: expected an analog voltage, got {other:?}"),
    }
}

/// The module powered the way a carrier powers it: 5 V into the two `5V`
/// fingers and 0 V into the three `GND` fingers of `J203`
/// (`p2_ec32mb.net`: fingers 41/42 `5V`, 43/44/45 `GND`).
fn module_carrier_rails(module: &str) -> Harness {
    Harness::new()
        .power(ep("CARRIER.5V"), ep(&format!("{module}.J203.41")), 5.0)
        .power(ep("CARRIER.5Vb"), ep(&format!("{module}.J203.42")), 5.0)
        .power(ep("CARRIER.GND"), ep(&format!("{module}.J203.43")), 0.0)
        .power(ep("CARRIER.GNDb"), ep(&format!("{module}.J203.44")), 0.0)
        .power(ep("CARRIER.GNDc"), ep(&format!("{module}.J203.45")), 0.0)
}

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

/// `U402`'s setpoint from its divider, `R401` 13.3 kΩ over `R403` 10.5 kΩ
/// at `V_FB` = 0.800 V (DS41958 Eq. 8): 1.8133 V.
const U402_V_SET: f64 = AP62301_V_FB_VOLTS * (1.0 + 13.3 / 10.5);
/// `U403`'s, `R402` 37.4 kΩ over `R404` 10.5 kΩ: 3.6495 V.
const U403_V_SET: f64 = AP62301_V_FB_VOLTS * (1.0 + 37.4 / 10.5);
/// The LDOs' 3.3 V, from their value.
const LDO_V_SET: f64 = 3.3;
/// The reset pull-up `R100`, 10.5 kΩ (`p2_ec32mb.net`).
const RESET_PULL_UP_OHMS: f64 = 10_500.0;
/// The debug-serial pull-ups `R305`/`R306`, 100 kΩ to `VIO_56_63`
/// (`p2_ec32mb.net`).
const DEBUG_SERIAL_PULL_UP_OHMS: f64 = 100_000.0;

/// The module as `embsim-boards` ships it (a P2 package held in reset in
/// the processor slot), with the power parts' monitors captured by
/// reference as they are built.
#[derive(Clone, Default)]
struct WatchedModule {
    rails: Arc<Mutex<HashMap<String, RailMonitor>>>,
    detector: Arc<Mutex<Option<DetectorMonitor>>>,
}

impl WatchedModule {
    fn board(&self) -> Board {
        let mut registry: PartRegistry = Ec32mb::new()
            .with_p2(|_decl| Box::new(P2Package::held_in_reset()))
            .registry();
        let rails = Arc::clone(&self.rails);
        registry.register(BUCK_PART, move |decl| {
            let rail = Rail::new(rail::Config::ap62301(), &AP62301_PINS_BY_FUNCTION)
                .expect("the AP62301 table carries every role");
            rails
                .lock()
                .unwrap()
                .insert(decl.reference.clone(), rail.monitor());
            Box::new(rail)
        });
        let rails = Arc::clone(&self.rails);
        registry.register(LDO_PART, move |decl| {
            let config = rail::Config::ncp114_from_value(&decl.value).expect("3.3 V");
            let rail = Rail::new(config, &NCP114_PINS_BY_FUNCTION)
                .expect("the NCP114 table carries every role");
            rails
                .lock()
                .unwrap()
                .insert(decl.reference.clone(), rail.monitor());
            Box::new(rail)
        });
        let detector = Arc::clone(&self.detector);
        registry.register(BROWNOUT_DETECTOR_PART, move |_decl| {
            let part =
                VoltageDetector::new(supervisor::Config::stm1061n16(), &STM1061_PINS_BY_FUNCTION);
            *detector.lock().unwrap() = Some(part.monitor());
            Box::new(part)
        });
        let parsed = netlist::parse(NETLIST).expect("the module netlist parses");
        Board::from_netlist(parsed, &registry).expect("the module builds")
    }

    fn rail(&self, reference: &str) -> RailMonitor {
        self.rails
            .lock()
            .unwrap()
            .get(reference)
            .cloned()
            .unwrap_or_else(|| panic!("{reference} was built"))
    }

    fn detector(&self) -> DetectorMonitor {
        self.detector
            .lock()
            .unwrap()
            .clone()
            .expect("U404 was built")
    }
}

/// A bench pad the test drives from its own thread, with the current
/// instrument on it.
struct Pad {
    pins: [PinDecl; 1],
    handle: Arc<Mutex<Option<PinHandle>>>,
}

impl Component for Pad {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }
    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        *self.handle.lock().unwrap() = Some(io.pin("P")?);
        io.on_branch("P", |_| {})
    }
}

/// A bench output that idles driven high — the pin the mechanical-pad
/// fixture puts a pad on.
struct Driver {
    pins: [PinDecl; 1],
}

impl Component for Driver {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }
    fn attach(&mut self, _io: ComponentNetIo) -> Result<(), AttachError> {
        Ok(())
    }
}

// ============================================================
// The module's power tree
// ============================================================

/// The module from its two fingers, nothing stuck: every rail at its
/// setpoint from the instant the bucks' soft-start elapses.
#[rstest]
fn the_ec32mb_power_tree_reads_its_rails_from_the_j203_fingers() {
    behaviour!(Test {
        id: "power.ec32mb-rails-from-the-fingers",
        covers: Some("models/src/rail.rs#Rail::attach"),
        given: "the P2-EC32MB with 5 volts on its two 5V edge fingers and 0 volts on its \
                three GND fingers, nothing stuck, started with time held and then released",
    });
    expect!(
        "down-before-the-first-wake",
        "with time held every bank rail and both buck rails float and every regulator \
         reports itself down",
        "the bucks' datasheet soft-start is 2.5 milliseconds and no wake has fired"
    );
    expect!(
        "bank-rails-at-three-volts-three",
        "once time runs every one of the eight bank rails reads 3.3 volts within a \
         millivolt",
        "the LDOs' value names 3.3 volts and their input, the second buck's rail, is up"
    );
    expect!(
        "core-rail-from-its-divider",
        "the core rail reads 1.813 volts and the LDO input rail 3.649 volts, each within a \
         millivolt",
        "one registry key builds both bucks and each reads its own feedback divider at \
         attach: 0.8 volts times one plus 13.3 over 10.5, and times one plus 37.4 over 10.5"
    );
    expect!(
        "at-the-soft-start-instant",
        "both bucks and all eight LDOs report their rail up from exactly 2.5 milliseconds \
         after the input arrived",
        "the bucks step up at their soft-start's end and the LDOs, which name no start-up \
         time, step with them"
    );
    expect!(
        "no-contention",
        "no net contends",
        "every rail is one declared terminal and nothing else holds it"
    );
    expect!(
        "debug-serial-at-the-pull-ups",
        "once the rails rise the two debug-serial pins read pulled high through their 100 \
         kilohm resistors",
        "their pull-ups return to a bank rail the LDOs have raised, and the P2 held in reset \
         releases every pad — the build snapshot has them floating for want of the rail"
    );
    let _guard = stepped();
    let watched = WatchedModule::default();
    let system = System::new()
        .board("EC32MB", watched.board())
        .harness(module_carrier_rails("EC32MB"))
        .hold_time()
        .start()
        .expect("the module starts");

    // Held: the attach-time cascade settles — the bucks see their input
    // and arm their soft-start — and no wake fires.
    for buck in ["U402", "U403"] {
        assert!(
            wait_for(
                || matches!(watched.rail(buck).state(), RailState::Rising { .. }),
                SETTLE
            ),
            "{buck}: {:?}",
            watched.rail(buck).state()
        );
        assert_eq!(
            watched.rail(buck).state(),
            RailState::Rising {
                at_ns: AP62301_SOFT_START_NS
            },
            "{buck} arms its soft-start from the instant the input arrived, t = 0"
        );
    }
    for rail in VIO_RAILS.iter().map(|r| format!("EC32MB.{r}")).chain([
        "EC32MB.Common_VDD".to_string(),
        "EC32MB.Common_LDOin".to_string(),
    ]) {
        assert_eq!(
            system.net_state(&rail),
            Some(NetState::Floating),
            "{rail} before the first wake"
        );
    }
    for ldo in 501..=508 {
        assert!(
            matches!(
                watched.rail(&format!("U{ldo}")).state(),
                RailState::Down {
                    input_up: false,
                    ..
                }
            ),
            "U{ldo}: {:?}",
            watched.rail(&format!("U{ldo}")).state()
        );
    }

    system.release_time();
    // Wait on every voltage asserted, not on one rail reading anything:
    // the eight LDOs publish one after another as their senses are
    // delivered, so a poller can see one bank rail up while another is
    // still floating — and an LDO whose `IN` and `EN` share the buck's rail
    // is delivered `IN` first, publishing its 100 Ω active discharge (input
    // up, enable still floating-to-off), 0 V, before the `EN` delivery
    // lifts it to 3.3 V, at the same instant.
    assert!(
        wait_for(|| module_rails_up(&system), SETTLE),
        "the module's rails rise to their setpoints; got {:?}",
        module_rail_states(&system)
    );
    for rail in VIO_RAILS {
        let v = volts(&system, &format!("EC32MB.{rail}"));
        assert!((v - LDO_V_SET).abs() < 1e-3, "{rail} reads {v}");
    }
    let core = volts(&system, "EC32MB.Common_VDD");
    assert!((core - U402_V_SET).abs() < 1e-3, "Common_VDD reads {core}");
    assert!((core - 1.813).abs() < 1e-3, "Common_VDD reads {core}");
    let ldo_in = volts(&system, "EC32MB.Common_LDOin");
    assert!(
        (ldo_in - U403_V_SET).abs() < 1e-3,
        "Common_LDOin reads {ldo_in}"
    );
    assert!((ldo_in - 3.649).abs() < 1e-3, "Common_LDOin reads {ldo_in}");

    // Two setpoints from one key, and the instant each rail rose.
    let u402 = watched.rail("U402");
    let u403 = watched.rail("U403");
    assert!((u402.v_set().unwrap() - U402_V_SET).abs() < 1e-12);
    assert!((u403.v_set().unwrap() - U403_V_SET).abs() < 1e-12);
    for (reference, monitor, v_set) in [("U402", &u402, U402_V_SET), ("U403", &u403, U403_V_SET)] {
        match monitor.state() {
            RailState::Up { volts, since_ns } => {
                assert!(
                    (volts - v_set).abs() < 1e-12,
                    "{reference} publishes {volts}"
                );
                assert_eq!(since_ns, AP62301_SOFT_START_NS, "{reference}");
            }
            other => panic!("{reference}: {other:?}"),
        }
    }
    for ldo in 501..=508 {
        match watched.rail(&format!("U{ldo}")).state() {
            RailState::Up { volts, since_ns } => {
                assert_eq!(volts, LDO_V_SET, "U{ldo}");
                assert_eq!(
                    since_ns, AP62301_SOFT_START_NS,
                    "U{ldo} steps with its input"
                );
            }
            other => panic!("U{ldo}: {other:?}"),
        }
    }
    assert!(
        !system
            .findings()
            .iter()
            .any(|f| matches!(f, Finding::Contention { .. })),
        "{:?}",
        system.findings()
    );
    // The debug-serial pins beside the transmit pin: `R305`/`R306`, 100 kΩ
    // to `VIO_56_63`, and a released pad on each (the snapshot in
    // `ec32mb_module.rs` reads them floating, the rail down).
    for net in ["EC32MB.P2_IO62_TXD", "EC32MB.P2_IO63_RXD"] {
        assert_eq!(
            system.net_state(net),
            Some(NetState::Pulled(Level::High, DEBUG_SERIAL_PULL_UP_OHMS)),
            "{net} at its pull-up once the bank rail is up"
        );
    }
    eprintln!(
        "power tree: escalated solves {} at the rails' rise",
        system.escalated_solves()
    );
    system.shutdown();
}

/// With one pad of the reset pull-up `R100` lifted the reset node floats
/// after the rails rise as before them, and the P2's `RESN` sense reports
/// it: the failure a carrier sees as "the module never boots". The detector
/// releases its output once the core rail is up, so nothing else holds the
/// node.
#[rstest]
fn the_reset_node_floats_without_its_pull_up_once_the_rails_are_up() {
    behaviour!(Test {
        id: "power.p2-reset-without-its-pull-up",
        covers: Some("board/src/system.rs#Scenario::pin_detach"),
        given: "the P2-EC32MB powered from its J203 fingers with one pad of the 10.5 kilohm \
                reset pull-up lifted, started with time held and then released past the \
                bucks' soft-start",
    });
    expect!(
        "reset-floats-with-the-rails-up",
        "once the rails have risen the reset node reads floating and the P2's reset sense \
         reports the float",
        "with one pad lifted the pull-up reaches nothing, and the detector, its supply past \
         the release threshold, drives nothing"
    );
    expect!(
        "the-rail-itself-rose",
        "the pull-up's bank rail reads 3.3 volts",
        "the fault is the lifted pad alone"
    );
    let _guard = stepped();
    let watched = WatchedModule::default();
    let system = System::new()
        .board("EC32MB", watched.board())
        .harness(module_carrier_rails("EC32MB"))
        .scenario(Scenario::default().pin_detach("EC32MB.R100.1"))
        .hold_time()
        .start()
        .expect("the module starts");
    assert!(
        wait_for(
            || matches!(watched.rail("U402").state(), RailState::Rising { .. }),
            SETTLE
        ),
        "the attach-time cascade settles with the buck armed"
    );
    system.release_time();
    assert!(
        wait_for(
            || watched.detector().decided_at_ns() == Some(AP62301_SOFT_START_NS),
            SETTLE
        ),
        "the detector decides at the rails' rise; got {:?}",
        watched.detector().decided_at_ns()
    );
    assert_eq!(watched.detector().asserted(), Some(false));
    assert!(
        wait_for(|| module_rails_up(&system), SETTLE),
        "the module's rails rise to their setpoints; got {:?}",
        module_rail_states(&system)
    );
    assert_eq!(
        system.net_state("EC32MB.P2_RESN"),
        Some(NetState::Floating),
        "the reset node with its pull-up lifted"
    );
    assert!(
        system.findings().contains(&Finding::FloatingSense {
            net: "EC32MB.P2_RESN".to_string(),
            kind: SenseKind::Digital,
        }),
        "the P2's RESN sense reports the float; got {:?}",
        system.findings()
    );
    system.shutdown();
}

/// The P2's reset node through the rails' rise: floating while the
/// pull-up's rail and the detector's supply are both down, pulled up from
/// the instant the rails rise — the detector, its supply stepping from
/// nothing to 1.813 V, past its 1.68 V release threshold, holds nothing —
/// and sunk by the detector while the core rail is held under its
/// threshold.
#[rstest]
fn p2_resn_is_sunk_until_common_vdd_crosses_1v6() {
    behaviour!(Test {
        id: "power.p2-reset-through-the-rails",
        covers: Some("models/src/supervisor.rs#VoltageDetector::attach"),
        given: "the P2-EC32MB powered from its J203 fingers, and beside it the module with \
                its core rail held from the bench under and over the detector's threshold",
    });
    expect!(
        "floats-before-the-rails",
        "with time held the reset node floats and the detector reports no verdict",
        "the 10.5 kilohm pull-up returns to a bank rail the bucks have not raised, and a \
         detector with no supply guarantees nothing under 0.7 volts"
    );
    expect!(
        "pulled-up-at-the-instant",
        "once the rails rise the reset node reads pulled high through the 10.5 kilohms",
        "the pull-up's rail rises with the bucks"
    );
    expect!(
        "detector-decides-at-the-instant",
        "the detector releases with its verdict dated exactly 2.5 milliseconds after the \
         input arrived",
        "the core rail steps to 1.813 volts, past the 1.68 volt release threshold, at the \
         buck's soft-start instant"
    );
    expect!(
        "sunk-under-the-threshold",
        "with the core rail held at 1.2 volts the detector sinks the reset node low",
        "1.2 volts is above the 0.7 volts the output is guaranteed from and under the 1.6 \
         volt detect threshold"
    );
    expect!(
        "released-over-the-threshold",
        "with the core rail held at 1.8 volts the reset node reads pulled high",
        "1.8 volts is over the release threshold"
    );
    let _guard = stepped();

    let watched = WatchedModule::default();
    let system = System::new()
        .board("EC32MB", watched.board())
        .harness(module_carrier_rails("EC32MB"))
        .hold_time()
        .start()
        .expect("the module starts");
    assert!(
        wait_for(
            || matches!(watched.rail("U402").state(), RailState::Rising { .. }),
            SETTLE
        ),
        "the attach-time cascade settles with the buck armed"
    );
    assert_eq!(system.net_state("EC32MB.P2_RESN"), Some(NetState::Floating));
    assert_eq!(watched.detector().asserted(), None, "no supply, no verdict");

    system.release_time();
    assert!(
        wait_for(
            || system.net_state("EC32MB.P2_RESN")
                == Some(NetState::Pulled(Level::High, RESET_PULL_UP_OHMS)),
            SETTLE
        ),
        "{:?}",
        system.net_state("EC32MB.P2_RESN")
    );
    let detector = watched.detector();
    assert_eq!(detector.asserted(), Some(false));
    assert_eq!(
        detector.pending(),
        None,
        "no crossing to delay: released from nothing"
    );
    assert_eq!(
        detector.decided_at_ns(),
        Some(AP62301_SOFT_START_NS),
        "decided at the instant the core rail stepped up"
    );
    assert_eq!(
        detector.drive_count(),
        0,
        "released throughout: nothing to drive"
    );
    system.shutdown();

    // The core rail held from the bench, the bank rail with it (no 5 V on
    // the fingers, so the bucks stay down and the stuck nets are the only
    // sources): the comparator either side of its threshold.
    for (core_volts, expected) in [
        (1.2, NetState::Driven(Level::Low)),
        (1.8, NetState::Pulled(Level::High, RESET_PULL_UP_OHMS)),
    ] {
        let watched = WatchedModule::default();
        let system = System::new()
            .board("EC32MB", watched.board())
            .harness(Harness::new().power(ep("CARRIER.GND"), ep("EC32MB.J203.43"), 0.0))
            .scenario(
                Scenario::default()
                    .net_stuck("EC32MB.Common_VDD", core_volts)
                    .net_stuck("EC32MB.VIO_56_63", 3.3),
            )
            .start()
            .expect("the module starts");
        // Wait on the verdict, not on the node alone: at 1.8 V the node
        // reads pulled high from the build (the stuck bank rail through
        // the pull-up, the detector's output idle released), before the
        // detector's supply senses have been delivered on the engine
        // thread and its verdict exists.
        assert!(
            wait_for(
                || watched.detector().asserted() == Some(core_volts < 1.6)
                    && system.net_state("EC32MB.P2_RESN") == Some(expected),
                SETTLE
            ),
            "core at {core_volts} V: verdict {:?}, node {:?}",
            watched.detector().asserted(),
            system.net_state("EC32MB.P2_RESN")
        );
        assert_eq!(
            watched.detector().asserted(),
            Some(core_volts < 1.6),
            "core at {core_volts} V"
        );
        assert_eq!(
            system.net_state("EC32MB.P2_RESN"),
            Some(expected),
            "core at {core_volts} V"
        );
        system.shutdown();
    }
}

// ============================================================
// The build lints
// ============================================================

/// A domain measured against nothing is reported, and the one-wire fix
/// clears it (the add-on under its own rails, every domain referenced,
/// reports none: `ds2_regressions.rs`).
#[rstest]
fn an_isolated_domain_whose_return_nothing_ties_is_reported() {
    behaviour!(Test {
        id: "lint.unreferenced-domain",
        covers: Some("board/src/system.rs#lint_build"),
        given: "the MaD Edge board alone under its bench rails",
    });
    expect!(
        "orphan-ground-reported",
        "the servo isolator's secondary side is reported as a domain measured against \
         nothing",
        "its supply is live on the servo rail while its ground is on a net nothing ties, so \
         the supply has no reference to be read against"
    );
    expect!(
        "isolated-rails-down-for-want-of-a-return",
        "both isolated DC/DCs report their output down naming their isolated ground",
        "an isolated output is measured against its own return, which nothing on the board \
         or this bench ties down"
    );
    expect!(
        "fixed-by-one-wire",
        "with the isolator's secondary ground tied to the isolated ground the report is gone",
    );
    let edge = System::new()
        .board("EdgeBoard", edge_board())
        .harness(bench_rails("EdgeBoard"))
        .build()
        .expect("builds");
    let orphan = Finding::UnreferencedDomain {
        part: "EdgeBoard.IC14".to_string(),
        pin: "16".to_string(),
        reference: "9".to_string(),
    };
    assert!(
        edge.diagnostics().contains(&orphan),
        "{:?}",
        edge.diagnostics().findings()
    );
    for part in ["EdgeBoard.IC3", "EdgeBoard.IC4"] {
        assert!(
            edge.diagnostics().contains(&Finding::RailDown {
                part: part.to_string(),
                pin: "14".to_string(),
                reason: RailDownReason::ReferenceUnheld {
                    pin: "15".to_string()
                },
            }),
            "{part}: {:?}",
            edge.diagnostics().findings()
        );
    }
    let fixed = System::new()
        .board("EdgeBoard", edge_board())
        .harness(bench_rails("EdgeBoard"))
        .scenario(Scenario::default().pin_short("EdgeBoard.IC14.9", "EdgeBoard.U24.8"))
        .build()
        .expect("builds");
    assert!(!fixed.diagnostics().contains(&orphan));
}

/// A mechanical pad on a net a pin drives is reported; the module's holes
/// on its ground and the Edge board's on the shield and on nothing are not.
#[rstest]
fn a_mechanical_pad_on_a_net_a_pin_drives_is_reported() {
    behaviour!(Test {
        id: "lint.mechanical-pad-on-a-driven-net",
        covers: Some("board/src/system.rs#lint_build"),
        given: "a bench board with a mounting hole on the net a push-pull output drives, the \
                P2-EC32MB from its J203 fingers, and the Edge board under its bench rails",
    });
    expect!(
        "driven-pad-reported",
        "the bench hole is reported on the driven net, naming the driving pin",
        "a pad the schematic meant to be ground or nothing loads a driver"
    );
    expect!(
        "holes-on-ground-and-on-nothing-report-nothing",
        "neither board reports a mechanical pad",
        "the module's holes sit on its ground, a terminal the carrier holds, and the Edge \
         board's on the shield net and on no net, which no pin drives"
    );
    const FIXTURE: &str = r#"(export (version "E")
  (components
    (comp (ref "U1") (value "driver") (libsource (lib "Bench") (part "DRIVER")))
    (comp (ref "H1") (value "MountingHole_Pad") (libsource (lib "Mechanical") (part "MountingHole_Pad"))))
  (nets
    (net (code "1") (name "OUT") (node (ref "U1") (pin "1")) (node (ref "H1") (pin "1")))))"#;
    let mut registry = PartRegistry::new();
    registry.register("DRIVER", |_| {
        Box::new(Driver {
            pins: [PinDecl::digital_out("1")],
        })
    });
    let board = Board::from_netlist(netlist::parse(FIXTURE).expect("parses"), &registry)
        .expect("classifies");
    let bench = System::new().board("B", board).build().expect("builds");
    assert!(
        bench.diagnostics().findings().iter().any(|f| matches!(
            f,
            Finding::MechanicalOnDrivenNet { part, net, drivers }
                if part == "B.H1" && net == "B.OUT" && drivers.len() == 1
                    && drivers[0].reference == "U1" && drivers[0].pin == "1"
        )),
        "{:?}",
        bench.diagnostics().findings()
    );

    let module = System::new()
        .board("EC32MB", shipped_ec32mb_board())
        .harness(module_carrier_rails("EC32MB"))
        .build()
        .expect("builds");
    let edge = System::new()
        .board("EdgeBoard", edge_board())
        .harness(bench_rails("EdgeBoard"))
        .build()
        .expect("builds");
    for (name, built) in [("EC32MB", &module), ("EdgeBoard", &edge)] {
        assert!(
            !built
                .diagnostics()
                .findings()
                .iter()
                .any(|f| matches!(f, Finding::MechanicalOnDrivenNet { .. })),
            "{name}: {:?}",
            built.diagnostics().findings()
        );
    }
}

/// A rail with no input is reported down naming the input: the bare
/// module, nothing on its fingers.
#[rstest]
fn rails_with_no_input_are_reported_down_at_build() {
    behaviour!(Test {
        id: "lint.rail-down-names-the-input",
        covers: Some("board/src/system.rs#lint_build"),
        given: "the P2-EC32MB built with nothing on its edge fingers",
    });
    expect!(
        "bucks-name-vin",
        "both bucks report their output down for want of their input pin",
    );
    expect!(
        "ldos-name-in",
        "all eight LDOs report their output down for want of their input pin",
        "the LDOs' input is the second buck's output, itself down"
    );
    let built = System::new()
        .board("EC32MB", shipped_ec32mb_board())
        .build()
        .expect("builds");
    for buck in ["U402", "U403"] {
        assert!(
            built.diagnostics().contains(&Finding::RailDown {
                part: format!("EC32MB.{buck}"),
                pin: "SW".to_string(),
                reason: RailDownReason::InputUnsourced {
                    pin: "VIN".to_string()
                },
            }),
            "{buck}: {:?}",
            built.diagnostics().findings()
        );
    }
    for ldo in 501..=508 {
        assert!(
            built.diagnostics().contains(&Finding::RailDown {
                part: format!("EC32MB.U{ldo}"),
                pin: "OUT".to_string(),
                reason: RailDownReason::InputUnsourced {
                    pin: "IN".to_string()
                },
            }),
            "U{ldo}: {:?}",
            built.diagnostics().findings()
        );
    }
}

/// A bench part with one supply pin measured against its ground pin: the
/// decoupling lint's fixture.
struct Supplied {
    pins: [PinDecl; 2],
    references: [PinReference; 1],
}

impl Supplied {
    fn new() -> Self {
        Self {
            pins: [PinDecl::power_in("VCC"), PinDecl::power_in("GND")],
            references: [PinReference {
                pin: "VCC",
                reference: "GND",
            }],
        }
    }
}

impl Component for Supplied {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }
    fn references(&self) -> &[PinReference] {
        &self.references
    }
    fn attach(&mut self, _io: ComponentNetIo) -> Result<(), AttachError> {
        Ok(())
    }
}

/// A supply pin with no capacitor between its node and its reference's is
/// reported; one across the two clears it; a capacitor to any other node
/// is none; and neither reference board raises it.
#[rstest]
fn a_supply_pin_with_no_capacitor_to_its_reference_is_reported() {
    behaviour!(Test {
        id: "lint.undecoupled-power-pin",
        covers: Some("board/src/system.rs#lint_build"),
        given: "a bench part's supply pin measured against its ground pin, with no capacitor, \
                one across the two nets, or one to a third net; and the two reference boards",
    });
    expect!(
        "missing-capacitor-reported",
        "with no capacitor between the supply net and the ground net the pin is reported as \
         undecoupled, naming the part, the pin and its reference",
        "a supply pin the layout does not decouple is what the lint exists to name"
    );
    expect!(
        "capacitor-across-clears-it",
        "a two-pin capacitor between the supply net and the ground net clears the report, \
         and one from the supply net to a third net does not",
        "only a capacitor across the pin and its reference decouples it"
    );
    expect!(
        "reference-boards-raise-none",
        "both boards build with no undecoupled supply pin reported",
        "every regulator, detector and isolator input on the two boards has a fitted \
         capacitor to its reference"
    );
    const BARE: &str = r#"(export (version "E")
  (components
    (comp (ref "U1") (value "supplied") (libsource (lib "Bench") (part "SUPPLIED"))))
  (nets
    (net (code "1") (name "VCC") (node (ref "U1") (pin "VCC")))
    (net (code "2") (name "GND") (node (ref "U1") (pin "GND")))))"#;
    const DECOUPLED: &str = r#"(export (version "E")
  (components
    (comp (ref "U1") (value "supplied") (libsource (lib "Bench") (part "SUPPLIED")))
    (comp (ref "C1") (value "100nF") (libsource (lib "Device") (part "C"))))
  (nets
    (net (code "1") (name "VCC") (node (ref "U1") (pin "VCC")) (node (ref "C1") (pin "1")))
    (net (code "2") (name "GND") (node (ref "U1") (pin "GND")) (node (ref "C1") (pin "2")))))"#;
    const ELSEWHERE: &str = r#"(export (version "E")
  (components
    (comp (ref "U1") (value "supplied") (libsource (lib "Bench") (part "SUPPLIED")))
    (comp (ref "C1") (value "100nF") (libsource (lib "Device") (part "C"))))
  (nets
    (net (code "1") (name "VCC") (node (ref "U1") (pin "VCC")) (node (ref "C1") (pin "1")))
    (net (code "2") (name "GND") (node (ref "U1") (pin "GND")))
    (net (code "3") (name "OTHER") (node (ref "C1") (pin "2")))))"#;
    let undecoupled = Finding::UndecoupledPowerPin {
        part: "B.U1".to_string(),
        pin: "VCC".to_string(),
        reference: "GND".to_string(),
    };
    for (fixture, reported) in [(BARE, true), (DECOUPLED, false), (ELSEWHERE, true)] {
        let mut registry = PartRegistry::new();
        registry.register("SUPPLIED", |_| Box::new(Supplied::new()));
        let board = Board::from_netlist(netlist::parse(fixture).expect("parses"), &registry)
            .expect("classifies");
        let bench = System::new()
            .board("B", board)
            .harness(
                Harness::new()
                    .power(ep("BENCH.VCC"), ep("B.U1.VCC"), 3.3)
                    .power(ep("BENCH.GND"), ep("B.U1.GND"), 0.0),
            )
            .build()
            .expect("builds");
        assert_eq!(
            bench.diagnostics().contains(&undecoupled),
            reported,
            "{fixture}\n{:?}",
            bench.diagnostics().findings()
        );
    }

    let module = System::new()
        .board("EC32MB", shipped_ec32mb_board())
        .harness(module_carrier_rails("EC32MB"))
        .build()
        .expect("builds");
    let edge = System::new()
        .board("EdgeBoard", edge_board())
        .harness(bench_rails("EdgeBoard"))
        .build()
        .expect("builds");
    for (name, built) in [("EC32MB", &module), ("EdgeBoard", &edge)] {
        let raised: Vec<&Finding> = built
            .diagnostics()
            .findings()
            .iter()
            .filter(|f| matches!(f, Finding::UndecoupledPowerPin { .. }))
            .collect();
        assert_eq!(raised, Vec::<&Finding>::new(), "{name}");
    }
}

// ============================================================
// The I-V port on a real board
// ============================================================

/// A bench pad sinking the module's `P63` debug-serial line low against its
/// 100 kΩ pull-up to a real 3.3 V rail: the current into the pad is the
/// pull-up's, `(3.3 − v) / 100 kΩ`, to a nanoamp.
#[rstest]
fn sense_current_into_a_low_sink_on_the_module_reads_the_pull_ups_current() {
    behaviour!(Test {
        id: "power.sink-current-on-a-real-board",
        covers: Some("board/src/component.rs#PinHandle::sense_current"),
        given: "the P2-EC32MB from its J203 fingers, its rails risen, and a bench pad on the \
                P63 finger sinking 0 volts through 25 ohms against the module's 100 kilohm \
                pull-up",
    });
    expect!(
        "pull-up-current",
        "the current into the pad equals the rail's 3.3 volts less the node voltage over \
         the 100 kilohms, within a nanoamp",
        "the instrument reads the pad's own drive current from the solved node voltage, and \
         the only path into the node is the pull-up"
    );
    let _guard = stepped();
    let pad = Pad {
        pins: [PinDecl::digital_out("P").with_idle(embsim_board::IdleDrive::Released)],
        handle: Arc::default(),
    };
    let pad_handle = Arc::clone(&pad.handle);
    let system = System::new()
        .board("EC32MB", shipped_ec32mb_board())
        .component("PAD", Box::new(pad))
        .harness(module_carrier_rails("EC32MB").connect(ep("PAD.P"), ep("EC32MB.J203.49")))
        .start()
        .expect("starts");
    assert!(
        wait_for(|| module_rails_up(&system), SETTLE),
        "the module's rails rise to their setpoints; got {:?}",
        module_rail_states(&system)
    );
    let pad = pad_handle.lock().unwrap().clone().expect("attached");
    pad.set_drive(Some(TheveninDrive {
        volts: 0.0,
        impedance: 25.0,
    }));
    let node = "EC32MB.P2_IO63_RXD";
    // Wait on the pair that is asserted — the sunk node and the pad's
    // current from the same solve — not on the node alone: the state and
    // current tables are published separately.
    let sunk = || -> Option<(f64, f64)> {
        match (system.net_state(node), system.pin_current("PAD.P")) {
            (Some(NetState::Analog(v)), Some(amps)) if v < 0.1 && amps > 3e-5 => Some((v, amps)),
            _ => None,
        }
    };
    assert!(
        wait_for(|| sunk().is_some(), SETTLE),
        "the pad sinks the line against the pull-up; node {:?}, current {:?}, rail {:?}, \
         escalated solves {}, findings {:?}",
        system.net_state(node),
        system.pin_current("PAD.P"),
        system.net_state("EC32MB.VIO_56_63"),
        system.escalated_solves(),
        system.findings()
    );
    let (v, into_pad) = sunk().expect("just observed");
    let expected = (LDO_V_SET - v) / 100_000.0;
    assert!(
        (into_pad - expected).abs() < 1e-9,
        "into the pad {into_pad} A against (3.3 − {v}) / 100 kΩ = {expected} A"
    );
    assert!(into_pad > 3e-5, "{into_pad}");
    system.shutdown();
}

// ============================================================
// Build == live before the first wake, under the reference harnesses
// ============================================================

/// The build snapshot equals the live system's state before its first
/// wake under each board's reference harness — every rail with a
/// soft-start down in both, every rail without one up in both.
#[rstest]
#[case::ec32mb_from_its_fingers(
    "EC32MB",
    shipped_ec32mb_board as fn() -> Board,
    module_carrier_rails as fn(&str) -> Harness
)]
#[case::edge_under_the_bench_rails(
    "EdgeBoard",
    edge_board as fn() -> Board,
    bench_rails as fn(&str) -> Harness
)]
fn the_build_snapshot_is_the_pre_wake_state_under_the_reference_harness(
    #[case] name: &str,
    #[case] build: fn() -> Board,
    #[case] rails: fn(&str) -> Harness,
) {
    behaviour!(Test {
        id: "power.build-equals-pre-wake-under-the-rails",
        covers: Some("board/src/system.rs#System::build"),
        given: "the P2-EC32MB from its J203 fingers, or the MaD Edge board under its bench \
                rails, analyzed at build and then started live with time held",
    });
    expect!(
        "same-states",
        "every net's live state, once the parts' attach-time drives have settled, is \
         exactly the state the build snapshot recorded for it",
        "a rail with a soft-start is down in both — its wake has not fired — and a rail \
         without one is up in both, published from the attach-time cascade"
    );
    expect!("settled", "the build reports nothing unsettled",);
    let _guard = stepped();
    let built = System::new()
        .board(name, build())
        .harness(rails(name))
        .build()
        .expect("builds");
    assert!(
        !built
            .diagnostics()
            .findings()
            .iter()
            .any(|f| matches!(f, Finding::BuildNotSettled { .. })),
        "{:?}",
        built.diagnostics().findings()
    );
    let live = System::new()
        .board(name, build())
        .harness(rails(name))
        .hold_time()
        .start()
        .expect("starts");
    let mismatches = || -> Vec<(String, NetState, Option<NetState>)> {
        built
            .nets()
            .iter()
            .enumerate()
            .filter_map(|(i, net)| {
                let live_state = live.net_state_of(NetId(i));
                (live_state.is_none_or(|s| !same_state(s, net.state)))
                    .then(|| (net.name.clone(), net.state, live_state))
            })
            .collect()
    };
    assert!(
        wait_for(|| mismatches().is_empty(), SETTLE),
        "{name}: the live system rests where the build said; still differing: {:?}",
        mismatches()
    );
    live.release_time();
    live.shutdown();
}
