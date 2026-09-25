//! The P2-EC32MB's clock chain, live: the TCXO `X100` publishes its 20 MHz
//! as a **rate** at its start-up instant, the rate crosses the coupling
//! capacitor `C132`, the oscillator buffer `U101` relays it stage by stage
//! and rests its self-biased stage mid-rail in one pass, and the P2's `XI`
//! net carries the rate — `NODES.md` §8 phase 2's proof for the
//! oscillator, the gate's rate mode and rate routing through a capacitor.
//!
//! Beside it, the AC-coupling rule on a bench fixture: a capacitor too
//! small to couple the rate stops it, with a finding that names the
//! capacitor, the rate and the two impedances it compared. And the
//! terminal rule on two more: a declared terminal — a ground held by the
//! scenario — is an AC short to its reference, so a rate coupled into it
//! is shunted there rather than forwarded through the next capacitor, and
//! two sources that merely share decoupling to it do not face each other;
//! the same node left undeclared is one more node on the path.
//!
//! Every case runs in stepped mode (`TESTING.md` rule 9), in its own binary
//! (rule 5): the cases pin the process-global clock and its mode.

mod machine_parts;

use std::collections::HashMap;
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};

use embsim_board::netlist;
use embsim_board::{
    AttachError, Board, Component, ComponentNetIo, EndpointRef, Finding, Harness, IdleDrive,
    NetState, PartRegistry, PinDecl, PinKind, PulseDirection, PulseSegment, PulseTrain, PulseTx,
    Scenario, StreamRole, System, SystemHandle,
};
use embsim_boards::ec32mb::{Ec32mb, INVERTER_PART, NETLIST, TCXO_HZ, TCXO_PART};
use embsim_boards::p2::{P2Package, P2PackageHandle};
use embsim_core::virtual_clock::{self, ClockMode};
use embsim_models::logic_gate::{
    self, LogicGate, LogicGateMonitor, Mode, LVC2G04_PINS_BY_FUNCTION, LVC2G04_R_OH_OHMS,
};
use embsim_models::oscillator::{self, Oscillator, OscillatorMonitor, TG2520SMN_START_UP_NS};
use embsim_models::rail::AP62301_SOFT_START_NS;
use rstest::rstest;
use vibes_behaviour::{behaviour, expect, Test};

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

fn stepped() {
    virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);
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
const MODULE: &str = "EC32MB";

/// Every train a probe was delivered.
type Trains = Arc<Mutex<Vec<PulseTrain>>>;
/// Every state a probe's net took.
type States = Arc<Mutex<Vec<NetState>>>;

/// A bench pulse sink: one pin that records every train it is delivered
/// and every state its net takes.
struct RateProbe {
    pins: [PinDecl; 1],
    trains: Trains,
    states: States,
}

impl RateProbe {
    /// A probe whose one pin is `number`, logging into the given vectors.
    fn logging(number: &'static str, trains: Trains, states: States) -> Self {
        Self {
            pins: [PinDecl {
                number,
                name: None,
                kind: PinKind::DigitalIn,
                stream: Some(StreamRole::PulseSink),
                drive_impedance: None,
                idle: IdleDrive::KindDefault,
            }],
            trains,
            states,
        }
    }

    fn new() -> (Self, Trains, States) {
        let trains = Arc::new(Mutex::new(Vec::new()));
        let states = Arc::new(Mutex::new(Vec::new()));
        let probe = Self::logging("CLK", Arc::clone(&trains), Arc::clone(&states));
        (probe, trains, states)
    }
}

impl Component for RateProbe {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let pin = self.pins[0].number;
        let trains = Arc::clone(&self.trains);
        io.on_pulse(pin, move |train| trains.lock().unwrap().push(train))?;
        let states = Arc::clone(&self.states);
        io.on_sense(pin, move |state| states.lock().unwrap().push(state))?;
        Ok(())
    }
}

/// The module as `embsim-boards` ships it, with monitors captured on the
/// TCXO and the two inverters as they are built.
#[derive(Clone, Default)]
struct Watched {
    tcxo: Arc<Mutex<Option<OscillatorMonitor>>>,
    gates: Arc<Mutex<HashMap<String, LogicGateMonitor>>>,
    /// The P2 package filling `U100`, held in reset: what it was delivered.
    p2: Arc<Mutex<Option<P2PackageHandle>>>,
}

impl Watched {
    fn board(&self) -> Board {
        let p2 = Arc::clone(&self.p2);
        let mut registry: PartRegistry = Ec32mb::new()
            .with_p2(move |_decl| {
                let package = P2Package::held_in_reset();
                *p2.lock().unwrap() = Some(package.handle());
                Box::new(package)
            })
            .registry();
        let tcxo = Arc::clone(&self.tcxo);
        registry.register(TCXO_PART, move |decl| {
            let config = oscillator::Config::from_value(&decl.value).expect("20 MHz");
            let part = Oscillator::new(config);
            *tcxo.lock().unwrap() = Some(part.monitor());
            Box::new(part)
        });
        let gates = Arc::clone(&self.gates);
        registry.register(INVERTER_PART, move |decl| {
            let gate = LogicGate::new(logic_gate::Config::lvc2g04(), &LVC2G04_PINS_BY_FUNCTION)
                .expect("valid");
            gates
                .lock()
                .unwrap()
                .insert(decl.reference.clone(), gate.monitor());
            Box::new(gate)
        });
        let parsed = netlist::parse(NETLIST).expect("the module netlist parses");
        Board::from_netlist(parsed, &registry).expect("the module builds")
    }

    fn gate(&self, reference: &str) -> LogicGateMonitor {
        self.gates
            .lock()
            .unwrap()
            .get(reference)
            .cloned()
            .unwrap_or_else(|| panic!("{reference} was built"))
    }
}

fn ep(endpoint: &str) -> EndpointRef {
    EndpointRef::parse(endpoint).expect("endpoint parses")
}

fn state(system: &SystemHandle, net: &str) -> NetState {
    system
        .net_state(&format!("{MODULE}.{net}"))
        .unwrap_or_else(|| panic!("net {net} exists"))
}

// ============================================================
// The module's clock chain
// ============================================================

/// `X100` → `C132` → `U101.2A` (self-biased through `R101`) → `2Y` → `1A`
/// → `1Y` → `XTAL_XI`, live, with a probe on `XTAL_XI`.
#[rstest]
fn the_tcxo_rate_reaches_xi_across_the_coupling_capacitor() {
    behaviour!(Test {
        id: "oscillator.rate-reaches-xi",
        covers: Some("models/src/oscillator.rs#Oscillator"),
        given: "the P2-EC32MB module started live with its rails at their unmodelled level, \
                a pulse probe on the P2's XI net, and virtual time then released",
    });
    expect!(
        "held-before-start-up",
        "with time held the P2's XI net floats and the probe has been delivered no rate",
        "the oscillator publishes at its datasheet start-up instant, a scheduled wake, so \
         before time moves the chain is where the build analysis left it"
    );
    expect!(
        "twenty-megahertz-after-the-rails",
        "once time runs, the probe on XI is delivered a 20 megahertz rate that began 3.5 \
         milliseconds after the carrier's 5 volts arrived",
        "the core buck's datasheet soft-start is 2.5 ms and the TCXO's start-up 1.0 ms from \
         its supply, and the two inverter stages relay the rate across the coupling capacitor"
    );
    expect!(
        "mid-rail-fixed-point",
        "the self-biased input, the feedback node and the XI net all rest at half the \
         inverters' supply",
        "a stage carrying a rate drives the rate's time-average through its output \
         resistance, and a 100 kilohm feedback resistor carries that average back to the \
         input"
    );
    expect!(
        "one-drive-per-stage",
        "each inverter stage drives its output exactly once and relays the rate exactly \
         once",
        "the rate arrives as one event, the stage answers with one drive and one relay, and \
         the level its own drive puts on its input changes nothing"
    );
    expect!(
        "clock-chain-escalates-nothing",
        "the only solves are the polarity FET's at the start pass and each feedback \
         divider's when its rail rose",
        "the clock chain is projections; a divider between two terminals is solved the once \
         its rail steps"
    );
    expect!(
        "package-takes-the-crystal",
        "the P2 package in the processor slot reports a 20 megahertz crystal once the rate \
         reaches XI, and reports none while time is held",
        "the crystal a P2 multiplies is whatever rate the board puts on XI, and the package \
         is what hands it to a core"
    );
    expect!(
        "xo-still-floats",
        "the P2's XO net still floats",
        "the vendor drives XI from the external TCXO and leaves XO, the crystal driver, \
         unused; the package releases it"
    );

    let _lock = suite_lock();
    stepped();
    let watched = Watched::default();
    let (probe, trains, _states) = RateProbe::new();
    // The module powered as a carrier powers it, 5 V and 0 V on the `J203`
    // fingers: the TCXO runs from `Common_VDD`, the core buck's output,
    // which is a real rail now and rises 2.5 ms after the input arrives.
    let system = System::new()
        .board(MODULE, watched.board())
        .component("PROBE", Box::new(probe))
        .harness(
            Harness::new()
                .connect_str("PROBE.CLK", &format!("{MODULE}.U100.XI"))
                .expect("endpoints parse")
                .power(ep("CARRIER.5V"), ep(&format!("{MODULE}.J203.41")), 5.0)
                .power(ep("CARRIER.GND"), ep(&format!("{MODULE}.J203.43")), 0.0),
        )
        .hold_time()
        .start()
        .expect("the module starts");

    // Held: the attach-time cascade has settled, no wake has fired.
    assert!(watched.tcxo.lock().unwrap().is_some(), "the TCXO was built");
    assert_eq!(
        watched.gates.lock().unwrap().len(),
        2,
        "both inverters were built"
    );
    std::thread::sleep(Duration::from_millis(50));
    assert_eq!(state(&system, "XTAL_XI"), NetState::Floating);
    assert!(trains.lock().unwrap().is_empty(), "no rate before start-up");
    let p2 = watched
        .p2
        .lock()
        .unwrap()
        .clone()
        .expect("the package was built");
    assert_eq!(p2.crystal_hz(), None, "no crystal before start-up");
    let tcxo = watched.tcxo.lock().unwrap().clone().expect("built");
    assert!(!tcxo.is_running());

    // One solve before time moves: the polarity FET's element cluster, the
    // carrier's 5 V turning its channel on at the start pass.
    assert_eq!(system.escalated_solves(), 1, "the FET's cluster, once");
    system.release_time();
    assert!(
        wait_for(
            || trains
                .lock()
                .unwrap()
                .iter()
                .any(|t| t.pulses.freq_hz == TCXO_HZ),
            SETTLE
        ),
        "the rate reaches XI; delivered {:?}, findings {:?}",
        trains.lock().unwrap(),
        system.findings()
    );
    let delivered = *trains.lock().unwrap().last().unwrap();
    assert_eq!(delivered.pulses.freq_hz, 20_000_000);
    assert_eq!(delivered.direction, PulseDirection::Forward);
    assert_eq!(
        delivered.pulses.since_us,
        (AP62301_SOFT_START_NS + TG2520SMN_START_UP_NS) / 1_000,
        "published at the start-up instant: t_SS after the input arrived at t = 0, when the \
         core rail rose and the TCXO saw its supply, then t_str"
    );
    assert_eq!(tcxo.publish_count(), 1, "one publish, nothing per edge");

    // The DC operating point the rate leaves behind.
    let u101 = watched.gate("U101");
    let rail = u101.config().nominal_supply_volts;
    for net in ["Net-(U101-2A)", "Net-(U101-2Y)", "XTAL_XI"] {
        assert!(
            wait_for(
                || state(&system, net) == NetState::Analog(rail * 0.5),
                SETTLE
            ),
            "{net} rests mid-rail; got {:?}",
            state(&system, net)
        );
    }
    assert_eq!(u101.mode(1), Mode::Rate, "2A→2Y carries the rate");
    assert_eq!(u101.mode(0), Mode::Rate, "1A→1Y carries the rate");
    assert_eq!(
        u101.output_drive(1).map(|d| d.impedance),
        Some(LVC2G04_R_OH_OHMS),
        "the average is driven through the datasheet output resistance"
    );
    assert_eq!(
        u101.drive_count(1),
        1,
        "one drive: no sense→drive iteration"
    );
    assert_eq!(u101.drive_count(0), 1);
    assert_eq!(u101.train_count(1), 1);
    assert_eq!(u101.train_count(0), 1);
    // The clock chain escalates nothing: the two solves since the start
    // pass are the bucks' feedback dividers, one each, when their rails
    // rose at t_SS — a one-node cluster between two terminals, its two
    // sources within a factor of ten of each other.
    assert_eq!(
        system.escalated_solves(),
        3,
        "the FET at the start pass and the two dividers at t_SS; nothing for the clock chain"
    );
    assert!(
        !system
            .findings()
            .iter()
            .any(|f| matches!(f, Finding::PulseNotCoupled { .. })),
        "C132 couples 20 MHz: {:?}",
        system.findings()
    );

    assert_eq!(
        p2.crystal_hz(),
        Some(u64::from(TCXO_HZ)),
        "the package hands its core the rate on XI as the crystal"
    );

    assert_eq!(state(&system, "XTAL_XO"), NetState::Floating);
    drop(system);
}

// ============================================================
// The AC-coupling rule, on a bench fixture
// ============================================================

/// A pulse source on one side of a capacitor, a sink and a 1 kΩ to ground
/// on the other. The value is what the case says.
const COUPLING_FIXTURE: &str = r#"(export (version "E")
  (components
    (comp (ref "X1") (value "Src"))
    (comp (ref "C1") (value "10pF") (libsource (lib "Device") (part "C_Small") (description "")))
    (comp (ref "R1") (value "1k") (libsource (lib "Device") (part "R_Small") (description "")))
    (comp (ref "U1") (value "Snk")))
  (nets
    (net (code "1") (name "OSC") (class "Default")
      (node (ref "X1") (pin "OUT") (pintype "output"))
      (node (ref "C1") (pin "1") (pintype "passive")))
    (net (code "2") (name "IN") (class "Default")
      (node (ref "C1") (pin "2") (pintype "passive"))
      (node (ref "R1") (pin "1") (pintype "passive"))
      (node (ref "U1") (pin "A") (pintype "input")))
    (net (code "3") (name "GND") (class "Default")
      (node (ref "R1") (pin "2") (pintype "passive")))))"#;

/// A source that publishes one 20 MHz train when the system starts.
struct Src {
    pins: [PinDecl; 1],
    tx: Option<PulseTx>,
}

impl Component for Src {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        self.tx = Some(io.pulse_tx("OUT")?);
        Ok(())
    }

    fn start(&mut self) {
        self.tx.as_ref().unwrap().set_train(PulseTrain {
            pulses: PulseSegment {
                emitted: 0,
                freq_hz: 20_000_000,
                total: None,
                since_us: 0,
            },
            direction: PulseDirection::Forward,
        });
    }
}

fn coupling_board(fixture: &str, delivered: Arc<Mutex<Vec<PulseTrain>>>) -> Board {
    let mut registry = PartRegistry::new();
    registry.register("Src", |_decl| {
        Box::new(Src {
            pins: [PinDecl {
                number: "OUT",
                name: None,
                kind: PinKind::DigitalOut,
                stream: Some(StreamRole::PulseSource),
                drive_impedance: None,
                idle: IdleDrive::Released,
            }],
            tx: None,
        })
    });
    registry.register("Snk", move |_decl| {
        Box::new(RateProbe::logging(
            "A",
            Arc::clone(&delivered),
            Arc::new(Mutex::new(Vec::new())),
        ))
    });
    let parsed = netlist::parse(fixture).expect("the fixture parses");
    Board::from_netlist(parsed, &registry).expect("the fixture builds")
}

/// `1/(2π·f·C)` against `R_far / 10`: 10 pF at 20 MHz is 796 Ω against a
/// 1 kΩ node — not coupled; 100 nF is 80 mΩ — coupled.
#[rstest]
#[case::too_small("10pF", false)]
#[case::large_enough("100nF", true)]
fn a_rate_crosses_a_capacitor_only_when_its_reactance_is_small_against_the_far_node(
    #[case] value: &str,
    #[case] couples: bool,
) {
    behaviour!(Test {
        id: "engine.rate-through-capacitor",
        covers: Some("board/src/engine.rs#Resolver::route_pulses"),
        given: "a 20 megahertz pulse source reaching a pulse sink only through a series \
                capacitor, the sink's node tied to ground through 1 kilohm, the capacitor \
                10 picofarads or 100 nanofarads",
    });
    expect!(
        "coupled-when-reactance-is-small",
        "with 100 nanofarads the sink is delivered the rate",
        "the capacitor's reactance at the rate is far below a tenth of the resistance at \
         the node it feeds, so for the signal it is a short"
    );
    expect!(
        "stopped-when-reactance-is-large",
        "with 10 picofarads the sink is delivered nothing, and the run reports which \
         capacitor stopped which rate with the two impedances it compared",
        "796 ohms of reactance against a 1 kilohm node divides the signal away"
    );
    expect!(
        "dc-stays-open",
        "at DC the sink's node is ground through its resistor either way",
        "a coupling capacitor is a path for a rate and never a conduction edge"
    );

    let _lock = suite_lock();
    stepped();
    let delivered = Arc::new(Mutex::new(Vec::new()));
    let system = System::new()
        .board(
            "B",
            coupling_board(COUPLING_FIXTURE, Arc::clone(&delivered)),
        )
        .scenario(
            Scenario::default()
                .value_override("B.C1", value)
                .net_stuck("B.GND", 0.0),
        )
        .start()
        .expect("the fixture starts");

    let not_coupled = || {
        system.findings().into_iter().find_map(|f| match f {
            Finding::PulseNotCoupled {
                net,
                capacitor,
                hz,
                reactance_ohms,
                far_ohms,
            } => Some((net, capacitor, hz, reactance_ohms, far_ohms)),
            _ => None,
        })
    };
    if couples {
        assert!(
            wait_for(|| !delivered.lock().unwrap().is_empty(), SETTLE),
            "the rate crosses 100 nF: findings {:?}",
            system.findings()
        );
        assert_eq!(delivered.lock().unwrap()[0].pulses.freq_hz, 20_000_000);
        assert_eq!(not_coupled(), None);
    } else {
        assert!(
            wait_for(|| not_coupled().is_some(), SETTLE),
            "the crossing is reported: {:?}",
            system.findings()
        );
        let (net, capacitor, hz, reactance, far) = not_coupled().unwrap();
        assert_eq!(net, "B.IN");
        assert_eq!(capacitor, "C1");
        assert_eq!(hz, 20_000_000);
        assert!(
            (reactance - 795.77).abs() < 0.01,
            "1/(2π·20e6·10e-12) = 795.77 Ω"
        );
        assert_eq!(far, 1_000.0);
        std::thread::sleep(Duration::from_millis(50));
        assert!(delivered.lock().unwrap().is_empty(), "nothing crossed");
    }
    assert_eq!(
        system.net_state("B.IN"),
        Some(NetState::Pulled(embsim_board::Level::Low, 1_000.0)),
        "at DC the capacitor is open and the node is ground through R1"
    );
    drop(system);
}

// ============================================================
// A terminal is a barrier to rate routing
// ============================================================

/// A source decoupled to `GND` through `C1`, a sink whose node is biased to
/// the same `GND` through `R1` and decoupled to it through `C2`: the two
/// capacitors meet only at the ground node.
const SHUNT_FIXTURE: &str = r#"(export (version "E")
  (components
    (comp (ref "X1") (value "Src"))
    (comp (ref "C1") (value "100nF") (libsource (lib "Device") (part "C_Small") (description "")))
    (comp (ref "C2") (value "100nF") (libsource (lib "Device") (part "C_Small") (description "")))
    (comp (ref "R1") (value "1k") (libsource (lib "Device") (part "R_Small") (description "")))
    (comp (ref "U1") (value "Snk")))
  (nets
    (net (code "1") (name "OSC") (class "Default")
      (node (ref "X1") (pin "OUT") (pintype "output"))
      (node (ref "C1") (pin "1") (pintype "passive")))
    (net (code "2") (name "GND") (class "Default")
      (node (ref "C1") (pin "2") (pintype "passive"))
      (node (ref "C2") (pin "1") (pintype "passive"))
      (node (ref "R1") (pin "2") (pintype "passive")))
    (net (code "3") (name "IN") (class "Default")
      (node (ref "C2") (pin "2") (pintype "passive"))
      (node (ref "R1") (pin "1") (pintype "passive"))
      (node (ref "U1") (pin "A") (pintype "input")))))"#;

/// Two sources, each decoupled to `GND` through its own 100 nF, and a sink
/// on the first source's own net, which `R1` biases to `GND` so the net
/// carries a DC level for the sink's route (a source that drives no level
/// leaves its own net floating, and a floating net on a conduction route
/// carries no signal — the capacitor-coupled sink is the case that reads
/// through a float).
const SHARED_DECOUPLING_FIXTURE: &str = r#"(export (version "E")
  (components
    (comp (ref "X1") (value "Src"))
    (comp (ref "X2") (value "Src"))
    (comp (ref "C1") (value "100nF") (libsource (lib "Device") (part "C_Small") (description "")))
    (comp (ref "C2") (value "100nF") (libsource (lib "Device") (part "C_Small") (description "")))
    (comp (ref "R1") (value "1k") (libsource (lib "Device") (part "R_Small") (description "")))
    (comp (ref "U1") (value "Snk")))
  (nets
    (net (code "1") (name "OSC1") (class "Default")
      (node (ref "X1") (pin "OUT") (pintype "output"))
      (node (ref "C1") (pin "1") (pintype "passive"))
      (node (ref "R1") (pin "1") (pintype "passive"))
      (node (ref "U1") (pin "A") (pintype "input")))
    (net (code "2") (name "GND") (class "Default")
      (node (ref "C1") (pin "2") (pintype "passive"))
      (node (ref "C2") (pin "2") (pintype "passive"))
      (node (ref "R1") (pin "2") (pintype "passive")))
    (net (code "3") (name "OSC2") (class "Default")
      (node (ref "X2") (pin "OUT") (pintype "output"))
      (node (ref "C2") (pin "1") (pintype "passive")))))"#;

/// The findings a run reports about pulse routing, for the assertions
/// below.
fn routing_findings(system: &SystemHandle) -> Vec<Finding> {
    system
        .findings()
        .into_iter()
        .filter(|f| {
            matches!(
                f,
                Finding::StreamMismatch { .. } | Finding::PulseNotCoupled { .. }
            )
        })
        .collect()
}

/// A ground held by the scenario shunts the rate coupled into it; the same
/// node left undeclared is one more node between two capacitors in series.
#[rstest]
#[case::held_ground(true, false)]
#[case::undeclared_node(false, true)]
fn a_terminal_shunts_a_rate_coupled_into_it(#[case] held: bool, #[case] crosses: bool) {
    behaviour!(Test {
        id: "engine.terminal-shunts-a-coupled-rate",
        covers: Some("board/src/engine.rs#Resolver::route_pulses"),
        given: "a 20 megahertz pulse source reaching a pulse sink only through two 100 \
                nanofarad capacitors in series with a ground node between them, the sink's \
                node biased to that ground through 1 kilohm, and the ground either held at 0 \
                volts by the scenario or left undeclared",
    });
    expect!(
        "shunted-at-a-terminal",
        "with the ground held at 0 volts the sink is delivered nothing and the run raises no \
         routing finding",
        "a held node is an AC short to its reference, and a rate coupled into it ends there"
    );
    expect!(
        "crossed-when-undeclared",
        "with the ground undeclared the sink is delivered the 20 megahertz rate",
        "an undeclared node is one more node on the path, and two capacitors in series still \
         couple the rate"
    );
    expect!(
        "dc-stays-open",
        "at DC the sink's node is its ground through the resistor either way",
        "a coupling capacitor is a path for a rate and never a conduction edge"
    );

    let _lock = suite_lock();
    stepped();
    let delivered = Arc::new(Mutex::new(Vec::new()));
    let scenario = if held {
        Scenario::default().net_stuck("B.GND", 0.0)
    } else {
        Scenario::default()
    };
    let system = System::new()
        .board("B", coupling_board(SHUNT_FIXTURE, Arc::clone(&delivered)))
        .scenario(scenario)
        .start()
        .expect("the fixture starts");

    if crosses {
        assert!(
            wait_for(|| !delivered.lock().unwrap().is_empty(), SETTLE),
            "two capacitors in series couple the rate through an undeclared node: findings {:?}",
            system.findings()
        );
        assert_eq!(delivered.lock().unwrap()[0].pulses.freq_hz, 20_000_000);
    } else {
        std::thread::sleep(Duration::from_millis(50));
        assert!(
            delivered.lock().unwrap().is_empty(),
            "a rate coupled into a held ground is shunted there: {:?}",
            delivered.lock().unwrap()
        );
        assert_eq!(routing_findings(&system), Vec::<Finding>::new());
    }
    let expected = if held {
        NetState::Pulled(embsim_board::Level::Low, 1_000.0)
    } else {
        NetState::Floating
    };
    assert_eq!(
        system.net_state("B.IN"),
        Some(expected),
        "at DC both capacitors are open and the node is its ground through R1"
    );
    drop(system);
}

/// Two sources that share decoupling to a held ground do not face each
/// other; through the same node undeclared they do.
#[rstest]
#[case::held_ground(true)]
#[case::undeclared_node(false)]
fn two_sources_decoupled_to_one_terminal_do_not_face_each_other(#[case] held: bool) {
    behaviour!(Test {
        id: "engine.shared-decoupling-is-not-a-fight",
        covers: Some("board/src/engine.rs#Resolver::route_pulses"),
        given: "two 20 megahertz pulse sources each decoupled to one ground node through 100 \
                nanofarads, a pulse sink on the first source's net, which 1 kilohm biases to \
                that ground, and the ground either held at 0 volts by the scenario or left \
                undeclared",
    });
    expect!(
        "no-mismatch-at-a-terminal",
        "with the ground held at 0 volts the run reports no sources facing each other",
        "the two capacitors meet only at a held node, which shunts each rate and carries \
         neither across to the other source"
    );
    expect!(
        "delivered-at-a-terminal",
        "with the ground held at 0 volts the sink is delivered its own source's rate",
        "the sink shares the first source's net, and the shunt at the ground touches nothing \
         on it"
    );
    expect!(
        "facing-when-undeclared",
        "with the ground undeclared the run reports the two sources facing each other on the \
         first source's net",
        "an undeclared node between two capacitors is a path from one source to the other"
    );
    expect!(
        "undelivered-when-facing",
        "with the ground undeclared the sink is delivered nothing",
        "a net two sources reach carries neither cleanly"
    );

    let _lock = suite_lock();
    stepped();
    let delivered = Arc::new(Mutex::new(Vec::new()));
    let scenario = if held {
        Scenario::default().net_stuck("B.GND", 0.0)
    } else {
        Scenario::default()
    };
    let system = System::new()
        .board(
            "B",
            coupling_board(SHARED_DECOUPLING_FIXTURE, Arc::clone(&delivered)),
        )
        .scenario(scenario)
        .start()
        .expect("the fixture starts");

    let facing: Vec<(String, Vec<String>)> = system
        .findings()
        .into_iter()
        .filter_map(|f| match f {
            Finding::StreamMismatch { net, producers } => Some((
                net,
                producers
                    .iter()
                    .map(|p| format!("{}.{}", p.reference, p.pin))
                    .collect(),
            )),
            _ => None,
        })
        .collect();
    if held {
        assert_eq!(facing, Vec::<(String, Vec<String>)>::new());
        assert!(
            wait_for(|| !delivered.lock().unwrap().is_empty(), SETTLE),
            "the sink on X1's own net is delivered X1's rate: findings {:?}",
            system.findings()
        );
        assert_eq!(delivered.lock().unwrap()[0].pulses.freq_hz, 20_000_000);
        assert_eq!(routing_findings(&system), Vec::<Finding>::new());
    } else {
        assert_eq!(
            facing,
            vec![(
                "B.OSC1".to_string(),
                vec!["X1.OUT".to_string(), "X2.OUT".to_string()]
            )],
            "through an undeclared node the two sources reach each other"
        );
        std::thread::sleep(Duration::from_millis(50));
        assert!(
            delivered.lock().unwrap().is_empty(),
            "a net two sources face on carries neither"
        );
    }
    drop(system);
}
