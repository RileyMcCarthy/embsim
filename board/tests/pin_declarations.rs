//! Pin declarations (`NODES.md` §11, `PinDecl`; §12 item 5, the task that
//! retired `PinKind`): a pin is a `PinRole` and the static facts declared
//! beside it — its idle drive, its thresholds and the supply they are
//! relative to, the reference it is measured against, whether it can source
//! or sink. These cases hold the three the plan names:
//!
//! * a receiver whose thresholds are declared relative to its supply
//!   reports them scaled by the voltage that supply reads, instant by
//!   instant — 0.3/0.7 of a bank at 3.3 V, then of the same bank browned out
//!   to 1.8 V — an absolute declaration beside it is unchanged, and a supply
//!   that reads no voltage scales nothing;
//! * a reference or a supply naming a pin the part does not declare fails
//!   the build as a facade mismatch naming it, for a netlist part and a
//!   bench component alike, and thresholds a supplied pin cannot hold are
//!   refused;
//! * a power-out pin that names its reference is a declared terminal: its
//!   own one-node cluster, holding exactly what it publishes.
//!
//! Stepped mode throughout (`TESTING.md` rule 9), own binary.

use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};

use embsim_board::{
    jesd8c01_lvcmos_thresholds, AttachError, Board, BoardError, Clamp, ClampRail, Component,
    ComponentNetIo, DeadBand, EndpointRef, Harness, InputPort, Level, NetState, PartRegistry,
    PinDecl, Scenario, System, SystemError, TheveninDrive, Thresholds,
};
use embsim_boards::p2::P2_PAD_THRESHOLDS;
use embsim_core::virtual_clock::{self, ClockMode};
use rstest::rstest;
use vibes_behaviour::{behaviour, expect, Test};

// ============================================================
// Plumbing
// ============================================================

static SUITE_LOCK: Mutex<()> = Mutex::new(());

fn stepped() -> MutexGuard<'static, ()> {
    let guard = SUITE_LOCK.lock().unwrap_or_else(|poisoned| {
        SUITE_LOCK.clear_poison();
        poisoned.into_inner()
    });
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

/// A figure equal to `expected` volts to within the solve's quantum.
fn close(actual: f64, expected: f64) -> bool {
    (actual - expected).abs() < 1e-9
}

// ============================================================
// Fixtures
// ============================================================

/// A bench supply: `OUT`, a power-out pin measured against its own `GND`,
/// holding `idle` from the build and publishing `script`'s drives at their
/// instants — a bank rail that browns out and then drops.
struct Supply {
    pins: [PinDecl; 2],
    script: Vec<(u64, Option<TheveninDrive>)>,
}

impl Supply {
    fn new(idle: TheveninDrive, script: Vec<(u64, Option<TheveninDrive>)>) -> Self {
        Self {
            pins: [
                PinDecl::power_out("OUT")
                    .with_idle(Some(idle))
                    .with_reference("GND"),
                PinDecl::power_in("GND"),
            ],
            script,
        }
    }
}

impl Component for Supply {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }
    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let out = io.pin("OUT")?;
        let script = self.script.clone();
        for &(at_ns, _) in &script {
            io.schedule_at_ns(at_ns);
        }
        io.on_wake_ns(move |now_ns| {
            for (at_ns, drive) in &script {
                if *at_ns == now_ns {
                    out.set_drive(*drive);
                }
            }
        });
        Ok(())
    }
}

/// What the receiver records at each delivery of its supply: the relative
/// input's thresholds and the absolute input's, in volts.
type Reports = Arc<Mutex<Vec<(Option<Thresholds>, Option<Thresholds>)>>>;

/// A receiver on a bank supply `VIO` against `GND`: `REL` declares the P2
/// pad's thresholds, **relative** to `VIO`; `ABS` declares the JESD8C.01
/// pair, **absolute**. Every time `VIO` is delivered it records what the
/// two inputs' thresholds read.
struct Receiver {
    pins: [PinDecl; 4],
    reports: Reports,
}

impl Receiver {
    fn new(reports: Reports) -> Self {
        Self {
            pins: [
                PinDecl::power_in("VIO").with_reference("GND"),
                PinDecl::power_in("GND"),
                PinDecl::digital_in("REL", P2_PAD_THRESHOLDS)
                    .with_supply("VIO")
                    .with_reference("GND"),
                PinDecl::digital_in("ABS", jesd8c01_lvcmos_thresholds(DeadBand::Unknown))
                    .with_reference("GND"),
            ],
            reports,
        }
    }
}

impl Component for Receiver {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }
    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let relative = io.pin("REL")?;
        let absolute = io.pin("ABS")?;
        let reports = Arc::clone(&self.reports);
        io.on_net_report("VIO", move |_| {
            reports
                .lock()
                .unwrap()
                .push((relative.thresholds(), absolute.thresholds()));
        })
    }
}

/// A part whose pins are exactly `pins`.
struct Declared {
    pins: Vec<PinDecl>,
}

impl Component for Declared {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }
    fn attach(&mut self, _io: ComponentNetIo) -> Result<(), AttachError> {
        Ok(())
    }
}

/// One part `U1` with pins `1` and `2`, each on a net of its own.
const TWO_PINS: &str = r#"(export (version "E")
  (components
    (comp (ref "U1") (value "part") (libsource (lib "Bench") (part "PART"))))
  (nets
    (net (code "1") (name "A") (node (ref "U1") (pin "1")))
    (net (code "2") (name "B") (node (ref "U1") (pin "2")))))"#;

/// A rail `U1` (`OUT`, `GND`) feeding a sensor `U2` through 10 kΩ.
const RAIL: &str = r#"(export (version "E")
  (components
    (comp (ref "U1") (value "rail") (libsource (lib "Bench") (part "RAIL")))
    (comp (ref "U2") (value "sensor") (libsource (lib "Bench") (part "SENSOR")))
    (comp (ref "R1") (value "10k") (libsource (lib "Device") (part "R"))))
  (nets
    (net (code "1") (name "RAIL") (node (ref "U1") (pin "OUT")) (node (ref "R1") (pin "1")))
    (net (code "2") (name "LOAD") (node (ref "R1") (pin "2")) (node (ref "U2") (pin "1")))
    (net (code "3") (name "GND") (node (ref "U1") (pin "GND")))))"#;

/// How a declaration reaches the build: on a netlist part, or on a bench
/// component with no netlist.
#[derive(Debug, Clone, Copy)]
enum Route {
    Netlist,
    Bench,
}

/// Build `pins` as `U1` by `route`, returning the error the build refuses
/// them with.
fn refused(pins: Vec<PinDecl>, route: Route) -> BoardError {
    match route {
        Route::Netlist => {
            let mut registry = PartRegistry::new();
            registry.register("PART", move |_| Box::new(Declared { pins: pins.clone() }));
            Board::from_netlist(embsim_board::netlist::parse(TWO_PINS).unwrap(), &registry)
                .expect_err("the declaration is refused")
        }
        Route::Bench => {
            let error = System::new()
                .component("U1", Box::new(Declared { pins }))
                .build()
                .expect_err("the declaration is refused");
            match error {
                SystemError::Board { error, .. } => error,
                other => panic!("expected a board error, got {other:?}"),
            }
        }
    }
}

// ============================================================
// The cases
// ============================================================

/// A receiver's relative thresholds follow the supply it reads: the bank
/// starts at 3.3 V, browns out to 1.8 V at 1 ms and is released at 2 ms.
#[rstest]
fn relative_thresholds_scale_with_the_supply_they_are_declared_against() {
    behaviour!(Test {
        id: "pin.relative-thresholds-scale-with-supply",
        covers: Some("board/src/component.rs#PinHandle::thresholds"),
        given: "one input declaring 0.3 and 0.7 of its supply and one declaring 0.8 and 2.0 \
                volts, as the supply steps from 3.3 volts to 1.8 to off",
    });
    expect!(
        "scaled-at-3v3",
        "at 3.3 volts the first input's thresholds read 0.99 and 2.31 volts",
        "thresholds declared relative to a supply are fractions of the voltage that supply \
         reads, against the pin's reference"
    );
    expect!(
        "scaled-at-1v8",
        "after the brown-out the first input's thresholds read 0.54 and 1.26 volts",
        "the fractions are applied to the supply at each instant, so a brown-out moves the \
         thresholds with it"
    );
    expect!(
        "absolute-unchanged",
        "the other input's thresholds read exactly its declared volts at every step",
        "a pin that names no supply declares its thresholds in volts"
    );
    expect!(
        "no-supply-no-thresholds",
        "once the supply is off the first input reports no thresholds at all",
        "a supply that reads no voltage has nothing to scale by, and no threshold is invented \
         for it"
    );
    let _guard = stepped();
    let reports: Reports = Arc::default();
    let bank = |volts: f64| TheveninDrive {
        volts,
        impedance: 0.1,
    };
    let live = System::new()
        .component(
            "SUP",
            Box::new(Supply::new(
                bank(3.3),
                vec![(1_000_000, Some(bank(1.8))), (2_000_000, None)],
            )),
        )
        .component("RX", Box::new(Receiver::new(Arc::clone(&reports))))
        .harness(
            Harness::new()
                .connect(ep("SUP.OUT"), ep("RX.VIO"))
                .connect(ep("SUP.GND"), ep("RX.GND"))
                .power(ep("BENCH.0V"), ep("RX.GND"), 0.0),
        )
        .start()
        .expect("starts");
    let seen = |pred: &dyn Fn(Option<Thresholds>) -> bool| {
        reports
            .lock()
            .unwrap()
            .iter()
            .any(|(relative, _)| pred(*relative))
    };
    let at = |v_il: f64, v_ih: f64| {
        move |t: Option<Thresholds>| t.is_some_and(|t| close(t.v_il, v_il) && close(t.v_ih, v_ih))
    };
    assert!(
        wait_for(|| seen(&at(0.99, 2.31)), SETTLE),
        "{:?}",
        reports.lock().unwrap()
    );
    assert!(
        wait_for(|| seen(&at(0.54, 1.26)), SETTLE),
        "{:?}",
        reports.lock().unwrap()
    );
    assert!(
        wait_for(|| seen(&|t: Option<Thresholds>| t.is_none()), SETTLE),
        "{:?}",
        reports.lock().unwrap()
    );
    let reports = reports.lock().unwrap().clone();
    let relative: Vec<Option<(f64, f64)>> = reports
        .iter()
        .map(|(t, _)| t.map(|t| (t.v_il, t.v_ih)))
        .collect();
    let first_brown_out = relative
        .iter()
        .position(|t| t.is_some_and(|(v_il, _)| close(v_il, 0.54)))
        .expect("seen above");
    assert!(
        relative[..first_brown_out]
            .iter()
            .all(|t| t.is_some_and(|(v_il, v_ih)| close(v_il, 0.99) && close(v_ih, 2.31))),
        "the 3.3 V reports precede the brown-out: {relative:?}"
    );
    assert!(
        reports
            .iter()
            .all(|(_, absolute)| *absolute == Some(jesd8c01_lvcmos_thresholds(DeadBand::Unknown))),
        "{reports:?}"
    );
    live.shutdown();
}

/// A reference or a supply that names a pin the part does not declare
/// fails the build, by either route a declaration arrives.
#[rstest]
#[case::reference(PinDecl::digital_out("1").with_reference("GND"))]
#[case::supply(PinDecl::digital_in("1", P2_PAD_THRESHOLDS).with_supply("VIO"))]
fn a_declaration_naming_an_undeclared_pin_is_a_facade_mismatch(
    #[case] pin: PinDecl,
    #[values(Route::Netlist, Route::Bench)] route: Route,
) {
    behaviour!(Test {
        id: "pin.declaration-names-a-declared-pin",
        covers: Some("board/src/board.rs#validate_pin_declarations"),
        given: "a part whose pin is measured against, or scaled by, a pin the part does not \
                declare, brought to the build as a netlist part or as a bench component",
    });
    expect!(
        "facade-mismatch",
        "the build fails as a pin-facade mismatch naming the part and the missing pin",
        "a reference or supply is a declaration about two of the part's own pins, checked \
         against its facade like a branch's pins"
    );
    let _guard = stepped();
    let named = pin.reference.or(pin.supply).expect("the case names one");
    let error = refused(vec![pin, PinDecl::passive("2")], route);
    assert!(
        matches!(
            &error,
            BoardError::PinFacadeMismatch { reference, pin } if reference == "U1" && pin == named
        ),
        "{error:?}"
    );
}

/// Thresholds declared against a supply are fractions of it; figures that
/// are not — an absolute pair handed to a supplied pin — are refused.
#[rstest]
fn thresholds_a_supplied_pin_cannot_hold_are_refused(
    #[values(Route::Netlist, Route::Bench)] route: Route,
) {
    behaviour!(Test {
        id: "pin.supplied-thresholds-are-fractions",
        covers: Some("board/src/component.rs#validate_declarations"),
        given: "an input that names its supply pin and declares the fixed 0.8 and 2.0 volt \
                pair, brought to the build as a netlist part or as a bench component",
    });
    expect!(
        "declaration-refused",
        "the build fails naming the part and the input",
        "a pin that names a supply declares its thresholds as fractions of that supply, so a \
         figure above one is a pair in volts that would be read as a multiple of the supply"
    );
    let _guard = stepped();
    let error = refused(
        vec![
            PinDecl::digital_in("1", jesd8c01_lvcmos_thresholds(DeadBand::Unknown))
                .with_supply("2"),
            PinDecl::power_in("2"),
        ],
        route,
    );
    assert!(
        matches!(
            &error,
            BoardError::InvalidDeclaration { reference, pin, .. } if reference == "U1" && pin == "1"
        ),
        "{error:?}"
    );
}

/// A power-out pin that names its reference is a declared terminal.
#[rstest]
fn a_power_out_pin_with_a_reference_is_a_terminal() {
    behaviour!(Test {
        id: "pin.referenced-power-out-is-a-terminal",
        covers: Some("board/src/system.rs#add_pin_descriptor"),
        given: "a rail whose output pin is measured against its own ground pin and holds 3.3 \
                volts, feeding a sensed node through 10 kilohms, with a 0 volt ground",
    });
    expect!(
        "own-cluster",
        "the rail's net is a cluster of its own",
        "a power-out pin's net is a declared terminal: a cluster boundary, whatever else it \
         declares"
    );
    expect!(
        "holds-its-voltage",
        "the rail's net reads exactly 3.3 volts and the sensed node reads pulled high through \
         the 10 kilohms"
    );
    let _guard = stepped();
    let mut registry = PartRegistry::new();
    registry.register("RAIL", |_| {
        Box::new(Declared {
            pins: vec![
                PinDecl::power_out("OUT")
                    .with_idle(Some(TheveninDrive {
                        volts: 3.3,
                        impedance: 0.1,
                    }))
                    .with_reference("GND"),
                PinDecl::power_in("GND"),
            ],
        })
    });
    registry.register("SENSOR", |_| {
        Box::new(Declared {
            pins: vec![PinDecl::digital_in(
                "1",
                jesd8c01_lvcmos_thresholds(DeadBand::Unknown),
            )],
        })
    });
    let board =
        Board::from_netlist(embsim_board::netlist::parse(RAIL).unwrap(), &registry).unwrap();
    let built = System::new()
        .board("B", board)
        .scenario(Scenario::default().net_stuck("B.GND", 0.0))
        .build()
        .expect("builds");
    let rail = built.net_id("B.RAIL").expect("the rail net");
    assert!(
        built
            .cluster_roots()
            .iter()
            .any(|cluster| cluster == &[rail]),
        "{:?}",
        built.cluster_roots()
    );
    let state = |name: &str| built.nets()[built.net_id(name).unwrap().0].state;
    assert_eq!(state("B.RAIL"), NetState::Analog(3.3));
    assert_eq!(state("B.LOAD"), NetState::Pulled(Level::High, 10_000.0));
}

/// A clamp whose knee is negative: no diode has one.
const NEGATIVE_KNEE: Clamp = Clamp {
    to: ClampRail::Supply,
    vf: -0.7,
    r_d: 10.0,
};

/// An input port or a clamp the solver cannot stamp is refused, whichever
/// route brings it.
#[rstest]
#[case::port_on_a_rail(PinDecl::power_out("1").with_idle(None).with_input(InputPort { v_bias: 0.8, r_in: 12_000.0 }))]
#[case::port_without_resistance(PinDecl::analog("1").with_input(InputPort { v_bias: 0.8, r_in: 0.0 }))]
#[case::port_without_bias(PinDecl::analog("1").with_input(InputPort { v_bias: f64::NAN, r_in: 12_000.0 }))]
#[case::port_stronger_than_a_pull(PinDecl::analog("1").with_input(InputPort { v_bias: 0.8, r_in: 470.0 }))]
#[case::clamp_with_negative_knee(PinDecl::analog("1").with_supply("2").with_clamps(&[NEGATIVE_KNEE]))]
fn a_port_or_clamp_the_solver_cannot_stamp_is_refused(
    #[case] pin: PinDecl,
    #[values(Route::Netlist, Route::Bench)] route: Route,
) {
    behaviour!(Test {
        id: "pin.unstampable-port-or-clamp",
        covers: Some("board/src/component.rs#validate_declarations"),
        given: "a pin declaring an input port on a rail output, a port with no resistance, \
                with less than a kilohm or with no bias voltage, or a clamp diode with a \
                negative knee",
    });
    expect!(
        "declaration-refused",
        "the build fails naming the part and the pin",
        "the solver stamps a port as a source behind its resistance, ranked as a pull that \
         never contends, and a clamp as a diode: a figure it cannot stamp is a declaration \
         error, found before anything runs"
    );
    let _guard = stepped();
    let error = refused(vec![pin, PinDecl::power_in("2")], route);
    assert!(
        matches!(
            &error,
            BoardError::InvalidDeclaration { reference, pin, .. } if reference == "U1" && pin == "1"
        ),
        "{error:?}"
    );
}
