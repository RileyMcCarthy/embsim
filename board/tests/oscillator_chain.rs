//! The P2-EC32MB's clock chain, live: the TCXO `X100` drives its 20 MHz as a
//! **periodic drive** at its start-up instant, the rate crosses the coupling
//! capacitor `C132`, the oscillator buffer `U101` relays it stage by stage,
//! settling its self-biased stage in one pass, and the P2's `XI` net carries
//! the square wave — `NODES.md` §8 phase 2's proof for the oscillator, the
//! gate's rate mode and a rate through a capacitor, re-expressed in
//! `NODES.md` §12 item 5 for the one drive type (`Drive::Periodic`,
//! `sil-unified-drive.md` step 4).
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
    jesd8c01_lvcmos_thresholds, AttachError, Board, Component, ComponentNetIo, DeadBand, Drive,
    EndpointRef, Finding, Harness, Level, NetState, PartRegistry, PeriodicSchedule, PinDecl,
    PinHandle, Scenario, System, SystemHandle, TheveninDrive,
};
use embsim_boards::ec32mb::{Ec32mb, INVERTER_PART, NETLIST, TCXO_HZ, TCXO_PART};
use embsim_boards::p2::{P2Package, P2PackageHandle};
use embsim_core::virtual_clock::{self, ClockMode};
use embsim_models::logic_gate::{
    self, LogicGate, LogicGateMonitor, Mode, LVC2G04_PINS_BY_FUNCTION, LVC2G04_R_OH_OHMS,
    LVC2G04_R_OL_OHMS,
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

/// Every segment a probe's net carried.
type Trains = Arc<Mutex<Vec<PeriodicSchedule>>>;
/// Every state a probe's net took.
type States = Arc<Mutex<Vec<NetState>>>;

/// A bench probe: one sensed pin that records every state its net takes,
/// and every segment a square wave on it carries.
struct RateProbe {
    pins: [PinDecl; 1],
    trains: Trains,
    states: States,
}

impl RateProbe {
    /// A probe whose one pin is `number`, logging into the given vectors.
    fn logging(number: &'static str, trains: Trains, states: States) -> Self {
        Self {
            pins: [PinDecl::digital_in(
                number,
                jesd8c01_lvcmos_thresholds(DeadBand::Unknown),
            )],
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
        let states = Arc::clone(&self.states);
        io.on_net_report(pin, move |state| {
            if let NetState::Periodic { segment, .. } = state {
                trains.lock().unwrap().push(segment);
            }
            states.lock().unwrap().push(state);
        })?;
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
                a probe on the P2's XI net, and virtual time then released",
    });
    expect!(
        "held-before-start-up",
        "with time held the P2's XI net floats and the probe has been delivered no rate",
        "the oscillator publishes at its datasheet start-up instant, a scheduled wake, so \
         before time moves the chain is where the build analysis left it"
    );
    expect!(
        "twenty-megahertz-after-the-rails",
        "once time runs, the XI net carries a 20 megahertz rate that began 3.5 milliseconds \
         after the carrier's 5 volts arrived",
        "the core buck's datasheet soft-start is 2.5 ms and the TCXO's start-up 1.0 ms from \
         its supply, and the two inverter stages relay the rate across the coupling capacitor"
    );
    expect!(
        "chain-carries-the-segment",
        "the self-biased input carries the TCXO's small swing, the feedback node and XI a \
         full swing, all three with the TCXO's one segment",
        "the coupling capacitor passes the TCXO's swing, each inverter stage drives its own \
         two output levels around the segment it relays, and the 100 kilohm feedback resistor \
         yields to the rate arriving across the capacitor"
    );
    expect!(
        "tcxo-net-floats-at-dc",
        "the TCXO's own output net floats",
        "the datasheet names no DC level or source impedance for the clipped-sine output, so \
         its clock sources nothing at DC"
    );
    expect!(
        "one-drive-per-stage",
        "each inverter stage drives its output exactly once, and that drive relays the rate \
         between the part's own output levels",
        "the rate arrives as one state, the stage answers with one drive, and the square wave \
         its own drive puts back on its input is the same segment"
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
            || trains.lock().unwrap().iter().any(|t| t.freq_hz == TCXO_HZ),
            SETTLE
        ),
        "the rate reaches XI; delivered {:?}, findings {:?}",
        trains.lock().unwrap(),
        system.findings()
    );
    let delivered = *trains.lock().unwrap().last().unwrap();
    assert_eq!(delivered.freq_hz, 20_000_000);
    assert_eq!(
        delivered.since_ns,
        AP62301_SOFT_START_NS + TG2520SMN_START_UP_NS,
        "published at the start-up instant: t_SS after the input arrived at t = 0, when the \
         core rail rose and the TCXO saw its supply, then t_str"
    );
    assert_eq!(tcxo.publish_count(), 1, "one publish, nothing per edge");

    // The square wave the rate puts on every node of the chain: the TCXO's
    // own swing on the self-biased input, the inverters' full swing on the
    // feedback node and on XI, one segment throughout.
    let u101 = watched.gate("U101");
    // The gate's `V_CC` as its supply pin is handed it: the bank rail it
    // sits on, against the ground the fingers hold at 0 V.
    let NetState::Analog(rail) = state(&system, "VIO_24_31") else {
        panic!("U101's supply is a rail: {:?}", state(&system, "VIO_24_31"));
    };
    for (net, hi, lo) in [
        ("Net-(U101-2A)", Level::Low, Level::Low),
        ("Net-(U101-2Y)", Level::High, Level::Low),
        ("XTAL_XI", Level::High, Level::Low),
    ] {
        let expected = NetState::Periodic {
            hi,
            lo,
            segment: delivered,
        };
        assert!(
            wait_for(|| state(&system, net) == expected, SETTLE),
            "{net} carries the TCXO's segment; got {:?}",
            state(&system, net)
        );
    }
    assert_eq!(
        state(&system, "Net-(X100-OUT)"),
        NetState::Floating,
        "the TCXO's clock sources nothing at DC"
    );
    assert_eq!(u101.mode(1), Mode::Rate, "2A→2Y carries the rate");
    assert_eq!(u101.mode(0), Mode::Rate, "1A→1Y carries the rate");
    assert_eq!(
        u101.output(1),
        Some(Drive::Periodic {
            hi: TheveninDrive {
                volts: rail,
                impedance: LVC2G04_R_OH_OHMS,
            },
            lo: TheveninDrive {
                volts: 0.0,
                impedance: LVC2G04_R_OL_OHMS,
            },
            segment: delivered,
        }),
        "the rate is relayed between the datasheet's own output ports"
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
            .any(|f| matches!(f, Finding::PeriodicNotCoupled { .. })),
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

/// The module's two 74LVC2G04s as the build finds them: `U101`'s second
/// stage has `R101` (100 kΩ) from `2Y` back to `2A`, so its input is
/// self-biased and relays the TCXO's 0.8 V swing coupled onto it; its first
/// stage (`1A` on the `2Y` node) and both stages of the LED buffer `U601`
/// have no resistor back from their output, so a clock must cross their
/// input thresholds to be relayed.
#[rstest]
fn only_the_oscillator_buffers_fed_back_stage_is_self_biased() {
    behaviour!(Test {
        id: "logic-gate.self-bias-from-the-netlist",
        covers: Some("models/src/logic_gate.rs#LogicGate"),
        given: "the P2-EC32MB module built from its vendor netlist, whose oscillator inverter \
                has a 100 kilohm resistor from its second output back to its second input",
    });
    expect!(
        "fed-back-stage-self-biased",
        "that stage's input is treated as self-biased: any running clock it carries is relayed",
        "the resistor holds the input at the stage's own switching point, so a swing coupled \
         onto it crosses that point every cycle"
    );
    expect!(
        "other-stages-plain",
        "the oscillator inverter's first stage and both stages of the LED inverter are plain \
         inputs, whose clock must cross their thresholds"
    );

    let _lock = suite_lock();
    stepped();
    let watched = Watched::default();
    let system = System::new()
        .board(MODULE, watched.board())
        .harness(
            Harness::new()
                .power(ep("CARRIER.5V"), ep(&format!("{MODULE}.J203.41")), 5.0)
                .power(ep("CARRIER.GND"), ep(&format!("{MODULE}.J203.43")), 0.0),
        )
        .hold_time()
        .start()
        .expect("the module starts");
    let u101 = watched.gate("U101");
    let u601 = watched.gate("U601");
    assert!(u101.self_biased(1), "R101 joins 2Y back to 2A");
    assert!(!u101.self_biased(0), "nothing joins 1Y back to 1A");
    assert!(
        !u601.self_biased(0) && !u601.self_biased(1),
        "the LED buffer"
    );
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

/// A clock buffer that drives one 20 MHz square wave, rail to rail at
/// 25 Ω, when the system starts.
struct Src {
    pins: [PinDecl; 1],
    out: Option<PinHandle>,
}

impl Component for Src {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        self.out = Some(io.pin("OUT")?);
        Ok(())
    }

    fn start(&mut self) {
        self.out.as_ref().unwrap().drive(Drive::Periodic {
            hi: TheveninDrive {
                volts: 3.3,
                impedance: 25.0,
            },
            lo: TheveninDrive {
                volts: 0.0,
                impedance: 25.0,
            },
            segment: PeriodicSchedule {
                emitted: 0,
                freq_hz: 20_000_000,
                total: None,
                since_ns: 0,
            },
        });
    }
}

fn coupling_board(fixture: &str, delivered: Trains) -> Board {
    let mut registry = PartRegistry::new();
    registry.register("Src", |_decl| {
        Box::new(Src {
            pins: [PinDecl::digital_out("OUT").with_idle(None)],
            out: None,
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
        covers: Some("board/src/engine.rs#overlay_arrivals"),
        given: "a 20 megahertz clock reaching a sensing pin only through a series capacitor, \
                the sensing pin's node tied to ground through 1 kilohm, the capacitor 10 \
                picofarads or 100 nanofarads",
    });
    expect!(
        "coupled-when-reactance-is-small",
        "with 100 nanofarads the sensing pin's node carries the clock's rate",
        "the capacitor's reactance at the rate is far below a tenth of the resistance at \
         the node it feeds, so for the signal it is a short"
    );
    expect!(
        "stopped-when-reactance-is-large",
        "with 10 picofarads the sensing pin is handed no rate, and the run reports which \
         capacitor stopped which rate with the two impedances it compared",
        "796 ohms of reactance against a 1 kilohm node divides the signal away"
    );
    expect!(
        "dc-stays-open",
        "with the rate refused, the sensing pin's node is ground through its 1 kilohm \
         resistor alone",
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
            Finding::PeriodicNotCoupled {
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
        assert_eq!(delivered.lock().unwrap()[0].freq_hz, 20_000_000);
        assert!(
            matches!(
                system.net_state("B.IN"),
                Some(NetState::Periodic { segment, .. }) if segment.freq_hz == 20_000_000
            ),
            "the far node carries the rate: {:?}",
            system.net_state("B.IN")
        );
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
        assert_eq!(
            system.net_state("B.IN"),
            Some(NetState::Pulled(Level::Low, 1_000.0)),
            "at DC the capacitor is open and the node is ground through R1"
        );
    }
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

/// The findings a run reports about the clocks, for the assertions below:
/// a fight, or a rate a capacitor refused.
fn routing_findings(system: &SystemHandle) -> Vec<Finding> {
    system
        .findings()
        .into_iter()
        .filter(|f| {
            matches!(
                f,
                Finding::Contention { .. } | Finding::PeriodicNotCoupled { .. }
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
        covers: Some("board/src/engine.rs#overlay_arrivals"),
        given: "a 20 megahertz clock reaching a sensing pin only through two 100 nanofarad \
                capacitors in series with a ground node between them, the sensing pin's node \
                biased to that ground through 1 kilohm, and the ground either held at 0 volts \
                by the scenario or left undeclared",
    });
    expect!(
        "shunted-at-a-terminal",
        "with the ground held at 0 volts the sensing pin is handed no rate and the run \
         reports neither a fight nor a refused rate",
        "a held node is an AC short to its reference, and a rate coupled into it ends there"
    );
    expect!(
        "crossed-when-undeclared",
        "with the ground undeclared the sensing pin's node carries the 20 megahertz rate",
        "an undeclared node is one more node on the path, and two capacitors in series still \
         couple the rate"
    );
    expect!(
        "dc-stays-open",
        "with the ground held, the sensing pin's node is that ground through its 1 kilohm \
         resistor",
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
        assert_eq!(delivered.lock().unwrap()[0].freq_hz, 20_000_000);
    } else {
        std::thread::sleep(Duration::from_millis(50));
        assert!(
            delivered.lock().unwrap().is_empty(),
            "a rate coupled into a held ground is shunted there: {:?}",
            delivered.lock().unwrap()
        );
        assert_eq!(routing_findings(&system), Vec::<Finding>::new());
        assert_eq!(
            system.net_state("B.IN"),
            Some(NetState::Pulled(Level::Low, 1_000.0)),
            "at DC both capacitors are open and the node is its ground through R1"
        );
    }
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
        covers: Some("board/src/engine.rs#overlay_arrivals"),
        given: "two 20 megahertz clocks each decoupled to one ground node through 100 \
                nanofarads, a sensing pin on the first clock's net, which 1 kilohm biases to \
                that ground, and the ground either held at 0 volts by the scenario or left \
                undeclared",
    });
    expect!(
        "no-mismatch-at-a-terminal",
        "with the ground held at 0 volts the run reports no fight between the clocks",
        "the two capacitors meet only at a held node, which shunts each rate and carries \
         neither across to the other clock"
    );
    expect!(
        "delivered-at-a-terminal",
        "with the ground held at 0 volts the sensing pin's net carries its own clock's rate",
        "the sensing pin shares the first clock's net, and the shunt at the ground touches \
         nothing on it"
    );
    expect!(
        "facing-when-undeclared",
        "undeclared, it is a fight: one contention finding names both clocks where the sensing \
         pin reads",
        "an undeclared node between two capacitors is a path from one clock to the other, and \
         two rates on one net are contention"
    );
    expect!(
        "undelivered-when-facing",
        "with the ground undeclared the first clock's net ends in contention, carrying no rate",
        "a net two clocks reach carries neither cleanly"
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

    let facing = || -> Vec<(String, Vec<String>)> {
        system
            .findings()
            .into_iter()
            .filter_map(|f| match f {
                Finding::Contention { net, drivers } => Some((
                    net,
                    drivers
                        .iter()
                        .map(|p| format!("{}.{}", p.reference, p.pin))
                        .collect(),
                )),
                _ => None,
            })
            .collect()
    };
    if held {
        assert!(
            wait_for(|| !delivered.lock().unwrap().is_empty(), SETTLE),
            "the sensing pin on X1's own net sees X1's rate: findings {:?}",
            system.findings()
        );
        assert_eq!(delivered.lock().unwrap()[0].freq_hz, 20_000_000);
        std::thread::sleep(Duration::from_millis(50));
        assert_eq!(facing(), Vec::<(String, Vec<String>)>::new());
        assert_eq!(routing_findings(&system), Vec::<Finding>::new());
    } else {
        let on_osc1 = || {
            facing()
                .into_iter()
                .find(|(net, _)| net == "B.OSC1")
                .map(|(_, mut pins)| {
                    pins.sort();
                    pins
                })
        };
        assert!(
            wait_for(|| on_osc1().is_some(), SETTLE),
            "through an undeclared node the two clocks reach each other: {:?}",
            system.findings()
        );
        assert_eq!(
            on_osc1(),
            Some(vec!["X1.OUT".to_string(), "X2.OUT".to_string()])
        );
        assert!(
            wait_for(
                || system.net_state("B.OSC1") == Some(NetState::Contention),
                SETTLE
            ),
            "a net two clocks reach carries neither: {:?}",
            system.net_state("B.OSC1")
        );
    }
    drop(system);
}
