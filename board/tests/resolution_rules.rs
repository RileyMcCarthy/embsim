//! The resolution rules the node ↔ net interface needs (`NODES.md` §10
//! "Resolution", §11 contract line 4, §12 item 5 — the rules task), on the
//! wire:
//!
//! * a cluster solved because an analog reader asked publishes the
//!   operating point **and** reports rule 2's fights beside it — the reader
//!   is handed the voltage by its `Sense`, so a finding no longer costs it
//!   the voltage (phase 1's operating-point precedence, retired);
//! * a root exactly one source reaches is handed that source's open-circuit
//!   voltage without a solve, even by an analog reader (`DESIGN.md` rule
//!   8); a second source reaching it is what asks the solver;
//! * a pin's declared input port is a permanent weak source at the pin —
//!   the AM26LV32's open-input fail-safe reads its idle level through its
//!   own 12 kΩ to its own bias — and a declared clamp is a diode branch to
//!   the pin's supply, ending on the rail's terminal;
//! * an open drain on a net no pull-up reaches is a build finding.
//!
//! Stepped mode (`TESTING.md` rule 9) for every live case; its own binary.

use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};

use embsim_board::{
    jesd8c01_lvcmos_thresholds, AttachError, Board, Clamp, ClampRail, Component, ComponentNetIo,
    DeadBand, Drive, EndpointRef, Finding, Harness, Level, NetState, PartRegistry, PinDecl,
    PinHandle, PinRef, Scenario, Sense, System, TheveninDrive, Volts,
};
use embsim_core::virtual_clock::{self, ClockMode};
use rstest::rstest;
use vibes_behaviour::{behaviour, expect, Test};

mod machine_parts;

use machine_parts::{
    Rs422Receiver, AM26LV32_INPUT_OHMS, AM26LV32_OPEN_A_VOLTS, AM26LV32_OPEN_B_VOLTS,
};

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

/// A push-pull pad's strength in these rigs: 25 Ω, the crate's push-pull
/// impedance the P2's and the gates' pads drive at.
const PAD_OHMS: f64 = 25.0;

/// A pin the test drives from its own thread, idling at `idle`.
struct Driver {
    pins: [PinDecl; 1],
    handle: Arc<Mutex<Option<PinHandle>>>,
}

impl Driver {
    fn new(idle: Option<TheveninDrive>) -> (Self, Arc<Mutex<Option<PinHandle>>>) {
        let handle = Arc::new(Mutex::new(None));
        (
            Self {
                pins: [PinDecl::digital_out("Q").with_idle(idle)],
                handle: Arc::clone(&handle),
            },
            handle,
        )
    }
}

impl Component for Driver {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        *self.handle.lock().unwrap() = Some(io.pin("Q")?);
        Ok(())
    }
}

/// Every voltage an analog reader was handed.
type Handed = Arc<Mutex<Vec<Option<Volts>>>>;

/// An analog reader — a sense with no thresholds, an ADC input — on pin
/// `pin`, recording every voltage it is handed.
struct Reader {
    pins: [PinDecl; 1],
    handed: Handed,
}

impl Reader {
    fn new(pin: &'static str) -> (Self, Handed) {
        let handed: Handed = Arc::default();
        (
            Self {
                pins: [PinDecl::analog(pin)],
                handed: Arc::clone(&handed),
            },
            handed,
        )
    }
}

impl Component for Reader {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let handed = Arc::clone(&self.handed);
        let pin = self.pins[0].number;
        io.on_sense(pin, move |sense: Sense| {
            handed.lock().unwrap().push(sense.volts);
        })
    }
}

fn last(handed: &Handed) -> Option<Option<Volts>> {
    handed.lock().unwrap().last().copied()
}

/// A part whose pins are exactly `pins`, and nothing else.
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

/// A rail part holding 3.3 V on `OUT` against its `GND`, an analog reader
/// `U2`, and one 10 kΩ resistor between the rail and the reader's node.
const ONE_RESISTOR: &str = r#"(export (version "E")
  (components
    (comp (ref "U1") (value "rail") (libsource (lib "Bench") (part "RAIL")))
    (comp (ref "U2") (value "adc") (libsource (lib "Bench") (part "READER")))
    (comp (ref "R1") (value "10k") (libsource (lib "Device") (part "R"))))
  (nets
    (net (code "1") (name "RAIL") (node (ref "U1") (pin "OUT")) (node (ref "R1") (pin "1")))
    (net (code "2") (name "AIN") (node (ref "R1") (pin "2")) (node (ref "U2") (pin "1")))
    (net (code "3") (name "GND") (node (ref "U1") (pin "GND")))))"#;

/// The bench rail's output: 3.3 V behind 0.1 Ω.
const RAIL: TheveninDrive = TheveninDrive {
    volts: 3.3,
    impedance: 0.1,
};

fn rail_pins() -> Vec<PinDecl> {
    vec![
        PinDecl::power_out("OUT")
            .with_idle(Some(RAIL))
            .with_reference("GND"),
        PinDecl::power_in("GND"),
    ]
}

// ============================================================
// Rule 2's fights beside the operating point
// ============================================================

/// Two push-pull pads fight over a node an analog reader reads.
#[rstest]
fn a_fought_node_under_an_analog_reader_reports_its_fight_beside_its_voltage() {
    behaviour!(Test {
        id: "rules.fought-node-under-analog-reader",
        covers: Some("board/src/engine.rs#Resolver::resolve_cluster"),
        given: "a 25 ohm pad driving 3.3 volts and a 25 ohm pad driving 0 volts on one node, \
                read by an analog input",
    });
    expect!(
        "handed-operating-point",
        "the analog input is handed 1.65 volts, and the node reads that voltage",
        "the two pads divide the rail between them; the reader asked for the voltage"
    );
    expect!(
        "fight-reported",
        "the fight is reported as contention naming both pads, with the ambiguous level at \
         1.65 volts beside it",
        "a reader asking for the voltage does not hide a fault on the node it reads"
    );
    let _guard = stepped();
    let (hi, _) = Driver::new(Some(TheveninDrive {
        volts: 3.3,
        impedance: PAD_OHMS,
    }));
    let (lo, _) = Driver::new(Some(TheveninDrive {
        volts: 0.0,
        impedance: PAD_OHMS,
    }));
    let (adc, handed) = Reader::new("IN");
    let system = System::new()
        .component("HI", Box::new(hi))
        .component("LO", Box::new(lo))
        .component("ADC", Box::new(adc))
        .harness(
            Harness::new()
                .connect(ep("ADC.IN"), ep("HI.Q"))
                .connect(ep("LO.Q"), ep("HI.Q")),
        )
        .start()
        .expect("the bench starts");
    assert!(
        wait_for(
            || last(&handed).is_some_and(|v| v.is_some_and(|v| (v - 1.65).abs() < 1e-9)),
            SETTLE
        ),
        "{:?}",
        handed.lock().unwrap()
    );
    assert!(
        matches!(
            system.net_state("ADC.IN"),
            Some(NetState::Analog(v)) if (v - 1.65).abs() < 1e-9
        ),
        "{:?}",
        system.net_state("ADC.IN")
    );
    let findings = system.findings();
    let drivers = findings
        .iter()
        .find_map(|f| match f {
            Finding::Contention { drivers, .. } => Some(drivers.clone()),
            _ => None,
        })
        .unwrap_or_else(|| panic!("a contention finding: {findings:?}"));
    assert_eq!(drivers.len(), 2, "{drivers:?}");
    assert!(drivers.contains(&PinRef::new("HI", "Q")), "{drivers:?}");
    assert!(drivers.contains(&PinRef::new("LO", "Q")), "{drivers:?}");
    assert!(
        findings.iter().any(|f| matches!(
            f,
            Finding::AmbiguousLevel { volts, .. } if (volts - 1.65).abs() < 1e-9
        )),
        "{findings:?}"
    );
}

/// A rail and a short to 0 V on one net an analog reader reads: two
/// terminal sources fighting on one root.
#[rstest]
fn a_terminal_fought_under_an_analog_reader_reports_its_fight_beside_its_voltage() {
    behaviour!(Test {
        id: "rules.terminal-fight-under-analog-reader",
        covers: Some("board/src/engine.rs#Resolver::resolve_cluster"),
        given: "a rail holding 3.3 volts with a short to 0 volts injected on its net, the net \
                read directly by an analog input",
    });
    expect!(
        "operating-point",
        "the net reads the fight's 1.65 volts",
        "the reader asked for the operating point the two sources settle at"
    );
    expect!(
        "fight-reported",
        "one contention is reported on the rail net, naming no pin, with the ambiguous level \
         at 1.65 volts beside it",
        "two declared sources that disagree on one net are a fault whoever reads it"
    );
    let _guard = stepped();
    let mut registry = PartRegistry::new();
    registry.register("RAIL", |_| Box::new(Declared { pins: rail_pins() }));
    registry.register("READER", |_| {
        Box::new(Declared {
            pins: vec![PinDecl::analog("1")],
        })
    });
    const ON_THE_RAIL: &str = r#"(export (version "E")
  (components
    (comp (ref "U1") (value "rail") (libsource (lib "Bench") (part "RAIL")))
    (comp (ref "U2") (value "adc") (libsource (lib "Bench") (part "READER"))))
  (nets
    (net (code "1") (name "RAIL") (node (ref "U1") (pin "OUT")) (node (ref "U2") (pin "1")))
    (net (code "2") (name "GND") (node (ref "U1") (pin "GND")))))"#;
    let board = Board::from_netlist(
        embsim_board::netlist::parse(ON_THE_RAIL).unwrap(),
        &registry,
    )
    .unwrap();
    let built = System::new()
        .board("B", board)
        .scenario(
            Scenario::default()
                .net_stuck("B.GND", 0.0)
                .net_stuck("B.RAIL", 0.0),
        )
        .build()
        .expect("builds");
    let state = built.nets()[built.net_id("B.RAIL").unwrap().0].state;
    assert!(
        matches!(state, NetState::Analog(v) if (v - 1.65).abs() < 1e-9),
        "{state:?}"
    );
    let findings = built.diagnostics().findings();
    assert!(
        findings.contains(&Finding::Contention {
            net: "B.RAIL".to_string(),
            drivers: Vec::new(),
        }),
        "{findings:?}"
    );
    assert!(
        findings.iter().any(|f| matches!(
            f,
            Finding::AmbiguousLevel { net, volts } if net == "B.RAIL" && (volts - 1.65).abs() < 1e-9
        )),
        "{findings:?}"
    );
}

// ============================================================
// The single-source rule
// ============================================================

/// One rail, one resistor, one analog reader — and then a second source.
#[rstest]
fn one_source_reaching_an_analog_reader_is_its_open_circuit_voltage_unsolved() {
    behaviour!(Test {
        id: "rules.single-source-unsolved",
        covers: Some("board/src/engine.rs#Resolver::resolve_cluster"),
        given: "a 3.3 volt rail reaching an analog input through one 10 kilohm resistor, and \
                later a 25 ohm pad driving the input's node to 0 volts",
    });
    expect!(
        "open-circuit-volts",
        "the input is handed exactly 3.3 volts, and the engine has solved nothing",
        "one source reaching a node is the node's voltage: nothing else is connected, so no \
         current flows through the resistor"
    );
    expect!(
        "second-source-solves",
        "once the pad drives, the input is handed the divided voltage, 3.3 volts times 25 \
         over 10 025 ohms, from exactly one solve",
        "a node two sources reach sits between them, and only the solve knows where"
    );
    let _guard = stepped();
    let (adc, handed) = Reader::new("1");
    let adc = Arc::new(Mutex::new(Some(adc)));
    let mut registry = PartRegistry::new();
    registry.register("RAIL", |_| Box::new(Declared { pins: rail_pins() }));
    registry.register("READER", move |_| {
        Box::new(adc.lock().unwrap().take().expect("one reader"))
    });
    let board = Board::from_netlist(
        embsim_board::netlist::parse(ONE_RESISTOR).unwrap(),
        &registry,
    )
    .unwrap();
    let (pad, pad_handle) = Driver::new(None);
    let system = System::new()
        .board("B", board)
        .component("PAD", Box::new(pad))
        .harness(Harness::new().connect(ep("PAD.Q"), ep("B.U2.1")))
        .scenario(Scenario::default().net_stuck("B.GND", 0.0))
        .start()
        .expect("the bench starts");
    assert!(
        wait_for(|| last(&handed) == Some(Some(3.3)), SETTLE),
        "{:?}",
        handed.lock().unwrap()
    );
    assert_eq!(system.net_state("B.AIN"), Some(NetState::Analog(3.3)));
    assert_eq!(system.escalated_solves(), 0, "one source solves nothing");

    let pad = pad_handle
        .lock()
        .unwrap()
        .clone()
        .expect("the pad attached");
    pad.drive(Drive::Thevenin(TheveninDrive {
        volts: 0.0,
        impedance: PAD_OHMS,
    }));
    let divided = 3.3 * PAD_OHMS / (10_000.0 + PAD_OHMS);
    assert!(
        wait_for(
            || last(&handed).is_some_and(|v| v.is_some_and(|v| (v - divided).abs() < 1e-12)),
            SETTLE
        ),
        "{divided} V: {:?}",
        handed.lock().unwrap()
    );
    assert_eq!(system.escalated_solves(), 1, "two sources solve once");
}

// ============================================================
// Declarations the solver stamps
// ============================================================

/// An `AM26LV32` alone on a bench: powered at 3.3 V against a held ground,
/// enabled through `G` tied to its supply, channel 1's inputs on nets of
/// their own that nothing else touches, every other pin on a stub.
const RECEIVER: &str = r#"(export (version "E")
  (components
    (comp (ref "U1") (value "AM26LV32") (libsource (lib "Interface") (part "AM26LV32"))))
  (nets
    (net (code "1") (name "1B") (node (ref "U1") (pin "1")))
    (net (code "2") (name "1A") (node (ref "U1") (pin "2")))
    (net (code "3") (name "1Y") (node (ref "U1") (pin "3")))
    (net (code "4") (name "VDD") (node (ref "U1") (pin "4")) (node (ref "U1") (pin "12")) (node (ref "U1") (pin "16")))
    (net (code "5") (name "2Y") (node (ref "U1") (pin "5")))
    (net (code "6") (name "2A") (node (ref "U1") (pin "6")))
    (net (code "7") (name "2B") (node (ref "U1") (pin "7")))
    (net (code "8") (name "GND") (node (ref "U1") (pin "8")))
    (net (code "9") (name "3B") (node (ref "U1") (pin "9")))
    (net (code "10") (name "3A") (node (ref "U1") (pin "10")))
    (net (code "11") (name "3Y") (node (ref "U1") (pin "11")))
    (net (code "13") (name "4Y") (node (ref "U1") (pin "13")))
    (net (code "14") (name "4A") (node (ref "U1") (pin "14")))
    (net (code "15") (name "4B") (node (ref "U1") (pin "15")))))"#;

/// The receiver's supply on the bench: 3.3 V, inside `V_CC` 3 V to 3.6 V
/// (TI SLLS202H, §6.3).
const RECEIVER_VCC: Volts = 3.3;

/// An AM26LV32 with channel 1's inputs open, and then a pad on the A input.
#[rstest]
fn an_open_am26lv32_input_reads_its_own_bias_and_the_failsafe_holds_the_output_high() {
    behaviour!(Test {
        id: "rules.input-port-failsafe",
        covers: Some("board/src/system.rs#add_pin_descriptor"),
        given: "an AM26LV32 line receiver powered at 3.3 volts and enabled, channel 1's two \
                inputs open, and later a 25 ohm pad driving the A input to 0 volts",
    });
    expect!(
        "open-inputs-at-bias",
        "the open A input sits at 0.83 volts and the open B input at 0.70 volts, with \
         nothing solved",
        "each input is 12 kilohms to its own open-circuit voltage, the datasheet's, and \
         nothing else reaches it: the fail-safe rests on that bias"
    );
    expect!(
        "failsafe-high",
        "the channel's output is driven high",
        "an open pair reads 130 millivolts, inside the plus or minus 200 millivolt band the \
         fail-safe answers high"
    );
    expect!(
        "driver-overrides-port",
        "the pad pulls the A input to within 2 millivolts of 0 volts and the output goes low",
        "a 12 kilohm port is a pull: any real driver on the line wins"
    );
    let _guard = stepped();
    let mut registry = PartRegistry::new();
    registry.register("AM26LV32", |_| Box::new(Rs422Receiver::new(RECEIVER_VCC)));
    let board =
        Board::from_netlist(embsim_board::netlist::parse(RECEIVER).unwrap(), &registry).unwrap();
    let (pad, pad_handle) = Driver::new(None);
    let system = System::new()
        .board("B", board)
        .component("PAD", Box::new(pad))
        .harness(
            Harness::new()
                .power(ep("BENCH.VDD"), ep("B.U1.16"), RECEIVER_VCC)
                .connect(ep("PAD.Q"), ep("B.U1.2")),
        )
        .scenario(Scenario::default().net_stuck("B.GND", 0.0))
        .start()
        .expect("the bench starts");
    let state = |net: &str| system.net_state(net);
    assert!(
        wait_for(
            || state("B.1Y") == Some(NetState::Driven(Level::High)),
            SETTLE
        ),
        "{:?}",
        state("B.1Y")
    );
    assert_eq!(state("B.1A"), Some(NetState::Analog(AM26LV32_OPEN_A_VOLTS)));
    assert_eq!(state("B.1B"), Some(NetState::Analog(AM26LV32_OPEN_B_VOLTS)));
    assert_eq!(
        system.escalated_solves(),
        0,
        "each open input has one source"
    );

    let pad = pad_handle
        .lock()
        .unwrap()
        .clone()
        .expect("the pad attached");
    pad.drive(Drive::Thevenin(TheveninDrive {
        volts: 0.0,
        impedance: PAD_OHMS,
    }));
    assert!(
        wait_for(
            || state("B.1Y") == Some(NetState::Driven(Level::Low)),
            SETTLE
        ),
        "{:?}",
        state("B.1Y")
    );
    let pulled = AM26LV32_OPEN_A_VOLTS * PAD_OHMS / (AM26LV32_INPUT_OHMS + PAD_OHMS);
    assert!(
        matches!(state("B.1A"), Some(NetState::Analog(v)) if (v - pulled).abs() < 1e-12),
        "{pulled} V: {:?}",
        state("B.1A")
    );
    assert!(pulled < 0.002, "{pulled}");
}

/// A bench clamp: a knee and a dynamic resistance for the fixture, no
/// part's (none of the three boards' datasheets gives a clamp curve).
const BENCH_CLAMP: Clamp = Clamp {
    to: ClampRail::Supply,
    vf: 0.7,
    r_d: 10.0,
};

/// A pin clamped to a 3.3 V supply, driven through 1 kΩ to 3 V and then
/// to 5 V.
#[rstest]
fn a_clamped_pin_is_held_one_knee_above_its_supply() {
    behaviour!(Test {
        id: "rules.clamp-to-supply",
        covers: Some("board/src/system.rs#System::build"),
        given: "an input with a declared clamp diode to its 3.3 volt supply pin, knee 0.7 \
                volts behind 10 ohms, driven through 1 kilohm first to 3 volts and then to 5 \
                volts",
    });
    expect!(
        "below-the-knee",
        "at 3 volts the clamp is off: the input reads the drive's 3 volts and the pin \
         carries no current worth a microamp"
    );
    expect!(
        "clamped",
        "at 5 volts the input is held one knee and 10 ohms' drop above the supply, 4.0099 \
         volts, carrying 0.99 milliamps",
        "a clamp is always stamped: the diode to the supply conducts once the pin rises a \
         knee above it"
    );
    expect!(
        "terminal-no-union",
        "the supply's net stays a cluster of its own: the clamp ends on the rail's terminal",
    );
    let _guard = stepped();
    let pins = || {
        vec![
            PinDecl::analog("IN")
                .with_supply("VCC")
                .with_reference("GND")
                .with_clamps(&[BENCH_CLAMP]),
            PinDecl::power_in("VCC").with_reference("GND"),
            PinDecl::power_in("GND"),
        ]
    };
    let harness = || {
        Harness::new()
            .power(ep("BENCH.VCC"), ep("PART.VCC"), 3.3)
            .power(ep("BENCH.GND"), ep("PART.GND"), 0.0)
            .connect(ep("SRC.Q"), ep("PART.IN"))
    };
    let source = |volts: Volts| {
        Driver::new(Some(TheveninDrive {
            volts,
            impedance: 1_000.0,
        }))
    };

    // The build: the supply stays a terminal of its own.
    let built = System::new()
        .component("PART", Box::new(Declared { pins: pins() }))
        .component("SRC", Box::new(source(3.0).0))
        .harness(harness())
        .build()
        .expect("builds");
    let vcc = built.net_id("PART.VCC").expect("the supply net");
    let input = built.net_id("PART.IN").expect("the input net");
    let cluster_of = |net| {
        built
            .cluster_roots()
            .iter()
            .find(|c| c.iter().any(|&root| built.nets_are_merged(root, net)))
            .cloned()
            .unwrap_or_else(|| panic!("{net:?} is in a cluster: {:?}", built.cluster_roots()))
    };
    assert_eq!(cluster_of(vcc).len(), 1, "{:?}", built.cluster_roots());
    assert!(
        !cluster_of(input)
            .iter()
            .any(|&root| built.nets_are_merged(root, vcc)),
        "{:?}",
        built.cluster_roots()
    );

    let (src, src_handle) = source(3.0);
    let system = System::new()
        .component("PART", Box::new(Declared { pins: pins() }))
        .component("SRC", Box::new(src))
        .harness(harness())
        .start()
        .expect("the bench starts");
    let volts_at = || match system.net_state("PART.IN") {
        Some(NetState::Analog(v)) => Some(v),
        _ => None,
    };
    assert!(
        wait_for(
            || volts_at().is_some_and(|v| (v - 3.0).abs() < 1e-6),
            SETTLE
        ),
        "{:?}",
        system.net_state("PART.IN")
    );
    assert!(
        system
            .pin_current("PART.IN")
            .is_some_and(|a| a.abs() < 1e-6),
        "{:?}",
        system.pin_current("PART.IN")
    );

    let src = src_handle
        .lock()
        .unwrap()
        .clone()
        .expect("the source attached");
    src.drive(Drive::Thevenin(TheveninDrive {
        volts: 5.0,
        impedance: 1_000.0,
    }));
    let knee = 3.3 + BENCH_CLAMP.vf;
    let clamped = knee + (5.0 - knee) * BENCH_CLAMP.r_d / (1_000.0 + BENCH_CLAMP.r_d);
    assert!(
        wait_for(
            || volts_at().is_some_and(|v| (v - clamped).abs() < 1e-6),
            SETTLE
        ),
        "{clamped} V: {:?}",
        system.net_state("PART.IN")
    );
    let amps = (5.0 - clamped) / 1_000.0;
    assert!(
        system
            .pin_current("PART.IN")
            .is_some_and(|a| (a - amps).abs() < 1e-9),
        "{amps} A: {:?}",
        system.pin_current("PART.IN")
    );
}

// ============================================================
// The missing pull-up
// ============================================================

/// An open-drain output `U1` and, but for the no-connect case, a logic
/// input `U2` reading its net; `R1` 10 kΩ from that net to a rail `U3`
/// (3.3 V against its ground) or to ground itself, as the case wires it.
fn open_drain_bench(r1_to: Option<&str>, reader: bool) -> String {
    let mut comps = String::from(
        r#"(comp (ref "U1") (value "od") (libsource (lib "Bench") (part "OD")))
    (comp (ref "U3") (value "rail") (libsource (lib "Bench") (part "RAIL")))"#,
    );
    let mut out = String::from(r#"(node (ref "U1") (pin "1"))"#);
    let mut rail = String::from(r#"(node (ref "U3") (pin "OUT"))"#);
    let mut gnd = String::from(r#"(node (ref "U3") (pin "GND"))"#);
    if reader {
        comps.push_str(
            r#"
    (comp (ref "U2") (value "in") (libsource (lib "Bench") (part "SENSOR")))"#,
        );
        out.push_str(r#" (node (ref "U2") (pin "1"))"#);
    }
    if let Some(to) = r1_to {
        comps.push_str(
            r#"
    (comp (ref "R1") (value "10k") (libsource (lib "Device") (part "R")))"#,
        );
        out.push_str(r#" (node (ref "R1") (pin "1"))"#);
        let far = r#" (node (ref "R1") (pin "2"))"#;
        match to {
            "RAIL" => rail.push_str(far),
            _ => gnd.push_str(far),
        }
    }
    format!(
        r#"(export (version "E")
  (components
    {comps})
  (nets
    (net (code "1") (name "OUT") {out})
    (net (code "2") (name "RAIL") {rail})
    (net (code "3") (name "GND") {gnd})))"#
    )
}

/// An open-drain output on a bench, with and without its pull-up.
#[rstest]
#[case::read_with_no_resistor(None, true, true)]
#[case::pulled_down_only(Some("GND"), true, true)]
#[case::pulled_up(Some("RAIL"), true, false)]
#[case::no_connect(None, false, false)]
fn an_open_drain_no_pull_up_reaches_is_a_finding(
    #[case] r1_to: Option<&str>,
    #[case] reader: bool,
    #[case] raised: bool,
) {
    behaviour!(Test {
        id: "rules.open-drain-without-pull-up",
        covers: Some("board/src/system.rs#open_drains_without_pull_up"),
        given: "an open-drain output read by a logic input, with no resistor, 10 kilohms to \
                ground, or 10 kilohms to a 3.3 volt rail on its net; or wired to nothing",
    });
    expect!(
        "missing-pull-up-reported",
        "with no resistor, and with only the pull-down, the build reports the open-drain \
         pin as having no pull-up, naming the part, the pin and its net",
        "released, the output leaves its net to whatever else reaches it; with nothing to \
         pull it high, the input reading it never sees a high level"
    );
    expect!(
        "pulled-up-silent",
        "a pull-up to the rail silences the report, and so does having no other part on \
         the net"
    );
    let _guard = stepped();
    let mut registry = PartRegistry::new();
    registry.register("OD", |_| {
        Box::new(Declared {
            pins: vec![PinDecl::digital_out("1").sink_only()],
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
    registry.register("RAIL", |_| Box::new(Declared { pins: rail_pins() }));
    let netlist = open_drain_bench(r1_to, reader);
    let board = Board::from_netlist(
        embsim_board::netlist::parse(&netlist).expect("the bench parses"),
        &registry,
    )
    .expect("the bench classifies");
    let built = System::new()
        .board("B", board)
        .scenario(Scenario::default().net_stuck("B.GND", 0.0))
        .build()
        .expect("builds");
    let missing: Vec<Finding> = built
        .diagnostics()
        .findings()
        .iter()
        .filter(|f| matches!(f, Finding::OpenDrainWithoutPullUp { .. }))
        .cloned()
        .collect();
    let expected = if raised {
        vec![Finding::OpenDrainWithoutPullUp {
            part: "B.U1".to_string(),
            pin: "1".to_string(),
            net: "B.OUT".to_string(),
        }]
    } else {
        Vec::new()
    };
    assert_eq!(missing, expected, "{:?}", built.diagnostics().findings());
}
