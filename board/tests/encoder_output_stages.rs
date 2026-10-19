//! `embsim_models::machine::QuadratureEncoder`'s output stages on a bench
//! (`NODES.md` §20): push-pull, the RS-422 line driver, and the NPN open
//! collector — what each presents at both levels, an open collector with
//! and without a pull-up, an open collector into the TI AM26LV32 line
//! receiver (`embsim_models::am26lv32`, TI SLLS202H) with its negative
//! inputs grounded, as the MaD Edge carrier's jumpers ground them, and with
//! the pull-ups and the bias such an encoder needs, and a step train
//! carried as a rate counting through every stage.
//!
//! The receiver is placed by the standard catalog's number (`AM26LV32IDR`,
//! the Edge carrier's `U25`), from 3.3 V, both enables asserted. Every
//! resistor value here is the test's own board, stated where it is set.
//!
//! Stepped (`TESTING.md` rule 9): a suite lock, the clock re-anchored
//! stepped, the system started with time held, the case's thread a
//! registered actor, every read after a virtual settle, no
//! `QuiescenceTimeout` at the end.

use std::sync::{Arc, Mutex, MutexGuard};

use embsim_board::{
    jesd8c01_lvcmos_thresholds, netlist, AttachError, Board, Component, ComponentNetIo, DeadBand,
    Drive, EndpointRef, Finding, Harness, InputPort, Level, NetState, PeriodicSchedule, PinDecl,
    PinHandle, Scenario, System, SystemHandle, TheveninDrive, Volts,
};
use embsim_boards::catalog::StandardCatalog;
use embsim_core::virtual_clock::{self, Actor, ClockMode};
use embsim_models::machine::quadrature_encoder::{Config as EncoderConfig, EncoderInput};
use embsim_models::machine::stepper_motor::{self, DEFAULT_OBSERVE_INTERVAL_US, DEFAULT_TAU_S};
use embsim_models::machine::{OutputStage, QuadratureEncoder, StepperMotor};
use rstest::rstest;
use vibes_behaviour::{behaviour, expect, Test};

static SUITE_LOCK: Mutex<()> = Mutex::new(());

fn suite_lock() -> MutexGuard<'static, ()> {
    SUITE_LOCK.lock().unwrap_or_else(|poisoned| {
        SUITE_LOCK.clear_poison();
        poisoned.into_inner()
    })
}

fn ep(endpoint: &str) -> EndpointRef {
    EndpointRef::parse(endpoint).expect("endpoint parses")
}

/// The virtual time a count is handed before the pins are read. Nothing
/// between the encoder and what reads it arms an instant (the receivers
/// project their inputs, and no propagation delay is modelled), so any
/// span reads the settled state.
const STEP_NS: u64 = 7_000;

/// `(A, B)` for a count, A leading B: 0 (L, L), 1 (H, L), 2 (H, H),
/// 3 (L, H) — the quadrature the encoder walks.
fn quadrature(count: i64) -> (Level, Level) {
    match count.rem_euclid(4) {
        0 => (Level::Low, Level::Low),
        1 => (Level::High, Level::Low),
        2 => (Level::High, Level::High),
        _ => (Level::Low, Level::High),
    }
}

/// Start `system` with time held, register the case's thread, release
/// time and settle one count's span.
fn start(system: System, actor: &str) -> (SystemHandle, Actor) {
    virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);
    let system = system.hold_time().start().expect("the bench starts");
    let actor = virtual_clock::register_actor(actor);
    system.release_time();
    virtual_clock::wait_virtual_ns(STEP_NS);
    (system, actor)
}

/// Assert the engine never stopped waiting for the case's thread, and shut
/// the bench down; the findings the run made.
fn finish(system: SystemHandle, actor: Actor) -> Vec<Finding> {
    drop(actor);
    let findings = system.findings();
    let stalled: Vec<&Finding> = findings
        .iter()
        .filter(|finding| matches!(finding, Finding::QuiescenceTimeout { .. }))
        .collect();
    assert!(
        stalled.is_empty(),
        "the engine stopped waiting: {stalled:?}"
    );
    system.shutdown();
    findings
}

/// The level a net's state reads as, driven or pulled.
fn level(state: Option<NetState>, net: &str) -> Level {
    match state {
        Some(NetState::Driven(level)) | Some(NetState::Pulled(level, _)) => level,
        other => panic!("{net} must read a level, got {other:?}"),
    }
}

// ============================================================
// Each stage's levels and impedance
// ============================================================

/// The probe's port on each pin it reads: 10 kΩ to 1 V, so a source on the
/// pin's net shows its open-circuit voltage and its impedance in the
/// voltage the two settle at, and a pin nothing else drives rests at 1 V.
const PROBE_PORT: InputPort = InputPort {
    v_bias: 1.0,
    r_in: 10_000.0,
};

/// The encoder's rail and impedance in this test: not the defaults, so the
/// configuration's numbers are what reach the nets.
const RAIL: Volts = 5.0;
const OHMS: f64 = 47.0;

/// What each probed pin was last handed, in [`PROBED`] order.
type Probed = Arc<Mutex<[Option<Volts>; 4]>>;

/// The pins the probe reads: the encoder's `A`, `B`, `A-` and `B-`.
const PROBED: [&str; 4] = ["A", "B", "A-", "B-"];

/// An instrument reading the voltage on each of its pins through
/// [`PROBE_PORT`].
struct Probe {
    pins: [PinDecl; 4],
    volts: Probed,
}

impl Probe {
    fn new() -> Self {
        let pin = |number| PinDecl::analog(number).with_input(PROBE_PORT);
        Self {
            pins: PROBED.map(pin),
            volts: Probed::default(),
        }
    }
}

impl Component for Probe {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        for (index, pin) in PROBED.into_iter().enumerate() {
            let volts = Arc::clone(&self.volts);
            io.on_sense(pin, move |sense| volts.lock().unwrap()[index] = sense.volts)?;
        }
        Ok(())
    }
}

/// The voltage a source of `volts` behind `ohms` and the probe's port
/// settle at on one net.
fn against_probe(volts: Volts, ohms: f64) -> Volts {
    (volts / ohms + PROBE_PORT.v_bias / PROBE_PORT.r_in) / (1.0 / ohms + 1.0 / PROBE_PORT.r_in)
}

/// What an output of `stage` presents at `level`, as the probe reads it.
fn expected_at(stage: OutputStage, level: Level) -> Volts {
    match (stage, level) {
        (OutputStage::NpnOpenCollector, Level::High) => PROBE_PORT.v_bias,
        (_, Level::High) => against_probe(RAIL, OHMS),
        (_, Level::Low) => against_probe(0.0, OHMS),
    }
}

#[rstest]
#[case::push_pull(OutputStage::PushPull)]
#[case::rs422(OutputStage::Rs422)]
#[case::npn_open_collector(OutputStage::NpnOpenCollector)]
fn each_encoder_output_stage_presents_its_levels_and_impedance(#[case] stage: OutputStage) {
    behaviour!(Test {
        id: "encoder.output-stage-levels",
        covers: Some("models/src/machine/quadrature_encoder.rs#OutputStage"),
        given: "an encoder at 5 volts behind 47 ohms through each of its three output stages, \
                each output loaded by 10 kilohms to 1 volt, at a count with both channels low \
                and one with both high",
    });
    expect!(
        "push-pull-ports",
        "push-pull and line-driver outputs sit at the configured rail while high and at 0 \
         volts while low, behind the configured impedance either way",
        "each is a Thevenin port at the configured rail and impedance"
    );
    expect!(
        "minus-legs-inverse",
        "the line driver's minus legs sit at the inverse of their channels, behind the same \
         47 ohms"
    );
    expect!(
        "open-collector-sinks-or-releases",
        "an open collector sits at 0 volts behind 47 ohms while low and leaves its output at \
         the probe's own 1 volt while high",
        "the transistor sinks to the encoder's ground while on and is off, sourcing nothing, \
         while the phase is high"
    );
    let _lock = suite_lock();
    let encoder = QuadratureEncoder::new(EncoderConfig {
        high_volts: RAIL,
        drive_impedance_ohms: OHMS,
        ..EncoderConfig::new(1.0).with_output(stage)
    })
    .expect("a valid encoder");
    let input = encoder.input();
    let probe = Probe::new();
    let probed = Arc::clone(&probe.volts);
    let mut harness = Harness::new()
        .connect(ep("ENC.A"), ep("PROBE.A"))
        .connect(ep("ENC.B"), ep("PROBE.B"));
    if stage.has_complements() {
        harness = harness
            .connect(ep("ENC.A-"), ep("PROBE.A-"))
            .connect(ep("ENC.B-"), ep("PROBE.B-"));
    }
    let system = System::new()
        .component("ENC", Box::new(encoder))
        .component("PROBE", Box::new(probe))
        .harness(harness);
    let (system, actor) = start(system, "encoder-output-stage-levels-case");
    for count in [0, 1, 2] {
        input.set_position_counts(count);
        virtual_clock::wait_virtual_ns(STEP_NS);
        if count == 1 {
            continue;
        }
        let (a, b) = quadrature(count);
        let read = *probed.lock().unwrap();
        let close = |got: Option<Volts>, want: Volts, pin: &str| {
            let got = got.unwrap_or_else(|| panic!("{pin} names a voltage at {count}"));
            assert!(
                (got - want).abs() < 1e-9,
                "{} {pin} at count {count}: {got} V, not {want} V",
                stage.name()
            );
        };
        close(read[0], expected_at(stage, a), "A");
        close(read[1], expected_at(stage, b), "B");
        if stage.has_complements() {
            let not = |level| match level {
                Level::High => Level::Low,
                Level::Low => Level::High,
            };
            close(read[2], expected_at(stage, not(a)), "A-");
            close(read[3], expected_at(stage, not(b)), "B-");
        } else {
            // Nothing drives the probe's other two pins: each rests at its
            // own bias.
            close(read[2], PROBE_PORT.v_bias, "unwired A-");
            close(read[3], PROBE_PORT.v_bias, "unwired B-");
        }
    }
    finish(system, actor);
}

// ============================================================
// The open collector, with and without a pull-up
// ============================================================

/// A board reading two outputs with a 74LVC2G04 dual inverter (placed by
/// `74LVC2G04GW,125`), with a 4.7 kΩ pull-up from each input to its
/// 3.3 V supply when `pulled_up`.
///
/// `J1`: 1 `ENC_A`, 2 `ENC_B`, 3 `+3V3`, 4 `GND`, 5 `NA`, 6 `NB`. `U1`
/// (NXP 74LVC2G04 Table 3): 1 `1A`, 2 `GND`, 3 `2A`, 4 `2Y`, 5 `V_CC`,
/// 6 `1Y`.
fn inverter_netlist(pulled_up: bool) -> String {
    let pull = |reference: &str, pin: &str| -> String {
        if pulled_up {
            format!(" (node (ref \"{reference}\") (pin \"{pin}\"))")
        } else {
            String::new()
        }
    };
    let resistors = if pulled_up {
        "    (comp (ref \"R1\") (value \"4700\") (libsource (lib \"Device\") (part \"R\")))\n\
         \x20   (comp (ref \"R2\") (value \"4700\") (libsource (lib \"Device\") (part \"R\")))\n"
    } else {
        ""
    };
    format!(
        "(export (version \"E\")\n  (components\n\
         \x20   (comp (ref \"J1\") (value \"Conn_01x06\") (libsource (lib \"Connector\") (part \"Conn_01x06\")))\n\
         \x20   (comp (ref \"U1\") (value \"74LVC2G04GW,125\") (libsource (lib \"Logic\") (part \"74LVC2G04GW,125\")))\n\
         {resistors})\n  (nets\n\
         \x20   (net (code \"1\") (name \"ENC_A\") (node (ref \"J1\") (pin \"1\")) (node (ref \"U1\") (pin \"1\")){})\n\
         \x20   (net (code \"2\") (name \"ENC_B\") (node (ref \"J1\") (pin \"2\")) (node (ref \"U1\") (pin \"3\")){})\n\
         \x20   (net (code \"3\") (name \"+3V3\") (node (ref \"J1\") (pin \"3\")) (node (ref \"U1\") (pin \"5\")){}{})\n\
         \x20   (net (code \"4\") (name \"GND\") (node (ref \"J1\") (pin \"4\")) (node (ref \"U1\") (pin \"2\")))\n\
         \x20   (net (code \"5\") (name \"NA\") (node (ref \"J1\") (pin \"5\")) (node (ref \"U1\") (pin \"6\")))\n\
         \x20   (net (code \"6\") (name \"NB\") (node (ref \"J1\") (pin \"6\")) (node (ref \"U1\") (pin \"4\")))))\n",
        pull("R1", "1"),
        pull("R2", "1"),
        pull("R1", "2"),
        pull("R2", "2"),
    )
}

/// An open-collector encoder on the inverter board.
fn open_collector_on_inverters(pulled_up: bool) -> (System, EncoderInput) {
    let board = Board::from_netlist(
        netlist::parse(&inverter_netlist(pulled_up)).expect("the reader's netlist parses"),
        &StandardCatalog::base_registry(),
    )
    .expect("the catalog places the inverter by its number");
    let encoder =
        QuadratureEncoder::new(EncoderConfig::new(1.0).with_output(OutputStage::NpnOpenCollector))
            .expect("a valid encoder");
    let input = encoder.input();
    let system = System::new()
        .board("RX", board)
        .component("ENC", Box::new(encoder))
        .harness(
            Harness::new()
                .power(ep("BENCH.VCC"), ep("RX.J1.3"), 3.3)
                .connect(ep("ENC.A"), ep("RX.J1.1"))
                .connect(ep("ENC.B"), ep("RX.J1.2")),
        )
        .scenario(Scenario::default().net_stuck("RX.GND", 0.0));
    (system, input)
}

#[rstest]
#[case::with_pull_ups(true)]
#[case::without_pull_ups(false)]
fn an_open_collector_encoder_reads_high_only_where_a_pull_up_holds_it(#[case] pulled_up: bool) {
    behaviour!(Test {
        id: "encoder.open-collector-pull-up",
        covers: Some("models/src/machine/quadrature_encoder.rs#OutputStage::NpnOpenCollector"),
        given: "an open-collector encoder read by a logic inverter, with and without a 4.7 \
                kilohm pull-up on each output to 3.3 volts, at a count with both channels low \
                and one with both high",
    });
    expect!(
        "low-sinks",
        "while a channel is low its output is driven low and the inverter drives its output \
         high"
    );
    expect!(
        "high-pulled-up",
        "with the pull-ups, a high channel's output is pulled high and the inverter drives \
         its output low"
    );
    expect!(
        "high-floats",
        "without them, a high channel's output floats, and the build reports each output's \
         missing pull-up",
        "an open collector that is off sources nothing, and the encoder has no pull-up of \
         its own"
    );
    let _lock = suite_lock();
    let built = open_collector_on_inverters(pulled_up)
        .0
        .build()
        .expect("the bench builds");
    let unpulled: Vec<&Finding> = built
        .diagnostics()
        .findings()
        .iter()
        .filter(|finding| matches!(finding, Finding::OpenDrainWithoutPullUp { .. }))
        .collect();
    if pulled_up {
        assert!(unpulled.is_empty(), "{unpulled:?}");
    } else {
        let pins: Vec<(&str, &str)> = unpulled
            .iter()
            .map(|finding| match finding {
                Finding::OpenDrainWithoutPullUp { part, pin, .. } => (part.as_str(), pin.as_str()),
                _ => unreachable!(),
            })
            .collect();
        assert_eq!(pins, [("ENC", "A"), ("ENC", "B")]);
    }

    let (system, input) = open_collector_on_inverters(pulled_up);
    let (system, actor) = start(system, "encoder-open-collector-pull-up-case");
    let state = |net: &str| system.net_state(net);
    // Count 0: both channels low.
    for (input_net, output_net) in [("RX.ENC_A", "RX.NA"), ("RX.ENC_B", "RX.NB")] {
        assert_eq!(
            state(input_net),
            Some(NetState::Driven(Level::Low)),
            "{input_net}"
        );
        assert_eq!(
            level(state(output_net), output_net),
            Level::High,
            "{output_net}"
        );
    }
    // Count 2: both channels high, both transistors off.
    input.set_position_counts(2);
    virtual_clock::wait_virtual_ns(STEP_NS);
    for (input_net, output_net) in [("RX.ENC_A", "RX.NA"), ("RX.ENC_B", "RX.NB")] {
        if pulled_up {
            assert_eq!(
                level(state(input_net), input_net),
                Level::High,
                "{input_net}"
            );
            assert!(
                matches!(state(input_net), Some(NetState::Pulled(Level::High, _))),
                "{input_net}: {:?}",
                state(input_net)
            );
            assert_eq!(
                level(state(output_net), output_net),
                Level::Low,
                "{output_net}"
            );
        } else {
            assert_eq!(state(input_net), Some(NetState::Floating), "{input_net}");
        }
    }
    finish(system, actor);
}

// ============================================================
// Into an AM26LV32
// ============================================================

/// How the receiver's first two pairs are wired around the encoder.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Pairs {
    /// Each negative input on the board's ground and no pull-up: the Edge
    /// carrier's `JP2`/`JP3` closed.
    Grounded,
    /// Each positive input pulled up through 4.7 kΩ to the encoder's 5 V
    /// supply, each negative input held at half of it by two 10 kΩ
    /// resistors: what a board reading an open-collector encoder needs.
    PulledAndBiased,
}

/// The encoder's supply in [`Pairs::PulledAndBiased`]: the 5 V the Taiss
/// encoder's 5–24 V range starts at.
const ENCODER_SUPPLY: Volts = 5.0;

/// The receiver board: `U1` (`AM26LV32IDR`, SLLS202H Table 5-1) from 3.3 V
/// on `J1.5`, `G` on its supply and `G̅` on ground, both enabling; pairs 1
/// and 2 on `J1.1`–`J1.4` wired as `pairs` says, the encoder's supply on
/// `J1.7`; pairs 3 and 4 open.
///
/// `J1`: 1 `A1` (`1A`), 2 `B1` (`1B`), 3 `A2` (`2A`), 4 `B2` (`2B`),
/// 5 `VCC`, 6 `GND`, 7 `ENC_V`, 8 `Y1`, 9 `Y2`.
fn receiver_netlist(pairs: Pairs) -> String {
    let biased = pairs == Pairs::PulledAndBiased;
    let node =
        |reference: &str, pin: &str| format!(" (node (ref \"{reference}\") (pin \"{pin}\"))");
    let when = |on: bool, nodes: &[(&str, &str)]| -> String {
        if on {
            nodes.iter().map(|(r, p)| node(r, p)).collect()
        } else {
            String::new()
        }
    };
    let mut comps = vec![
        "(comp (ref \"J1\") (value \"Conn_01x09\") (libsource (lib \"Connector\") (part \"Conn_01x09\")))".to_string(),
        "(comp (ref \"U1\") (value \"AM26LV32IDR\") (libsource (lib \"Interface\") (part \"AM26LV32IDR\")))".to_string(),
    ];
    if biased {
        for (reference, value) in [
            ("R1", "4700"),
            ("R2", "4700"),
            ("R3", "10000"),
            ("R4", "10000"),
            ("R5", "10000"),
            ("R6", "10000"),
        ] {
            comps.push(format!(
                "(comp (ref \"{reference}\") (value \"{value}\") (libsource (lib \"Device\") (part \"R\")))"
            ));
        }
    }
    // R1, R2: the pull-ups from A1, A2 to ENC_V. R3/R4 and R5/R6: each a
    // divider from ENC_V to GND whose midpoint is B1 or B2.
    let grounded = !biased;
    let nets: Vec<(&str, String)> = vec![
        (
            "A1",
            node("J1", "1") + &node("U1", "2") + &when(biased, &[("R1", "1")]),
        ),
        (
            "A2",
            node("J1", "3") + &node("U1", "6") + &when(biased, &[("R2", "1")]),
        ),
        (
            "B1",
            when(
                biased,
                &[("J1", "2"), ("U1", "1"), ("R3", "1"), ("R4", "1")],
            ),
        ),
        (
            "B2",
            when(
                biased,
                &[("J1", "4"), ("U1", "7"), ("R5", "1"), ("R6", "1")],
            ),
        ),
        (
            "ENC_V",
            node("J1", "7")
                + &when(
                    biased,
                    &[("R1", "2"), ("R2", "2"), ("R3", "2"), ("R5", "2")],
                ),
        ),
        (
            "GND",
            node("J1", "6")
                + &node("U1", "8")
                + &node("U1", "12")
                + &when(
                    grounded,
                    &[("J1", "2"), ("U1", "1"), ("J1", "4"), ("U1", "7")],
                )
                + &when(biased, &[("R4", "2"), ("R6", "2")]),
        ),
        (
            "VCC",
            node("J1", "5") + &node("U1", "16") + &node("U1", "4"),
        ),
        ("Y1", node("J1", "8") + &node("U1", "3")),
        ("Y2", node("J1", "9") + &node("U1", "5")),
        ("A3", node("U1", "10")),
        ("B3", node("U1", "9")),
        ("Y3", node("U1", "11")),
        ("A4", node("U1", "14")),
        ("B4", node("U1", "15")),
        ("Y4", node("U1", "13")),
    ];
    let nets: String = nets
        .into_iter()
        .filter(|(_, nodes)| !nodes.is_empty())
        .enumerate()
        .map(|(code, (name, nodes))| {
            format!(
                "    (net (code \"{}\") (name \"{name}\"){nodes})\n",
                code + 1
            )
        })
        .collect();
    let comps: String = comps.iter().map(|c| format!("    {c}\n")).collect();
    format!("(export (version \"E\")\n  (components\n{comps})\n  (nets\n{nets}))\n")
}

/// The receiver board as `pairs` wires it, its supplies on the bench.
fn receiver_system(pairs: Pairs) -> System {
    let board = Board::from_netlist(
        netlist::parse(&receiver_netlist(pairs)).expect("the receiver's netlist parses"),
        &StandardCatalog::base_registry(),
    )
    .expect("the catalog places the receiver by its number");
    let mut harness = Harness::new()
        .power(ep("BENCH.VCC"), ep("RX.J1.5"), 3.3)
        .power(ep("BENCH.GND"), ep("RX.J1.6"), 0.0);
    if pairs == Pairs::PulledAndBiased {
        harness = harness.power(ep("BENCH.ENC_V"), ep("RX.J1.7"), ENCODER_SUPPLY);
    }
    System::new().board("RX", board).harness(harness)
}

/// The encoder's outputs onto the receiver's positive inputs, and with a
/// line driver its minus legs onto the negative ones.
fn encoder_onto_receiver(stage: OutputStage) -> Harness {
    let harness = Harness::new()
        .connect(ep("ENC.A"), ep("RX.J1.1"))
        .connect(ep("ENC.B"), ep("RX.J1.3"));
    if stage.has_complements() {
        harness
            .connect(ep("ENC.A-"), ep("RX.J1.2"))
            .connect(ep("ENC.B-"), ep("RX.J1.4"))
    } else {
        harness
    }
}

/// The pairs the run reported as read inside their band, as
/// `(part, pin, inverting, output)`.
fn pairs_reported(findings: &[Finding]) -> Vec<(String, String, String, Level)> {
    findings
        .iter()
        .filter_map(|finding| match finding {
            Finding::PairInBand {
                part,
                pin,
                inverting,
                output,
                ..
            } => Some((part.clone(), pin.clone(), inverting.clone(), *output)),
            _ => None,
        })
        .collect()
}

/// The pins the run reported above their declared recommended range
/// (`Finding::PinAboveRecommended`), as `(part, pin)`.
fn limits_reported(findings: &[Finding]) -> Vec<(String, String)> {
    findings
        .iter()
        .filter_map(|finding| match finding {
            Finding::PinAboveRecommended { part, pin, .. } => Some((part.clone(), pin.clone())),
            _ => None,
        })
        .collect()
}

/// Walk an open-collector encoder on the receiver board one count at a
/// time through `walk`, asserting `Y1` and `Y2` at every count with
/// `expected`; the findings the run made.
fn walk_open_collector(
    pairs: Pairs,
    walk: &[i64],
    expected: impl Fn(i64) -> (Level, Level),
) -> Vec<Finding> {
    let encoder =
        QuadratureEncoder::new(EncoderConfig::new(1.0).with_output(OutputStage::NpnOpenCollector))
            .expect("a valid encoder");
    let input = encoder.input();
    let system = receiver_system(pairs)
        .component("ENC", Box::new(encoder))
        .harness(encoder_onto_receiver(OutputStage::NpnOpenCollector));
    let (system, actor) = start(system, "encoder-into-am26lv32-case");
    let outputs = |count: i64| {
        let read = (
            level(system.net_state("RX.Y1"), "RX.Y1"),
            level(system.net_state("RX.Y2"), "RX.Y2"),
        );
        assert_eq!(read, expected(count), "Y1 and Y2 at count {count}");
    };
    outputs(0);
    for &target in walk {
        let mut at = input.count();
        while at != target {
            at += (target - at).signum();
            input.set_position_counts(at);
            virtual_clock::wait_virtual_ns(STEP_NS);
            outputs(at);
        }
    }
    finish(system, actor)
}

#[rstest]
fn an_open_collector_encoder_into_an_am26lv32_with_grounded_minus_legs_reads_high_and_is_reported()
{
    behaviour!(Test {
        id: "encoder.open-collector-into-am26lv32-grounded",
        covers: Some("models/src/am26lv32.rs#AM26LV32_PAIRS"),
        given: "an open-collector encoder on the positive inputs of an AM26LV32's first two \
                pairs, the negative inputs grounded and no pull-up, walked through two \
                quadrature cycles",
    });
    expect!(
        "stuck-high",
        "both of the receiver's outputs read high at every count",
        "a low sinks the positive input to ground beside the grounded negative one, leaving \
         the pair no difference, which the receiver reads as its fail-safe high; released, \
         the input rests at the receiver's own 0.83 volt bias, a valid high"
    );
    expect!(
        "pairs-reported",
        "the run reports each of the two pairs once, by its inputs, as read inside the band \
         with a common-mode voltage applied and its output high",
        "SLLS202H guarantees no level for a pair inside 0.2 volts with an external \
         common-mode voltage applied (Table 8-1, section 8.4.1)"
    );
    expect!(
        "open-pairs-silent",
        "the receiver's two pairs nothing is wired to are not reported"
    );
    let _lock = suite_lock();
    let findings = walk_open_collector(Pairs::Grounded, &[9], |_| (Level::High, Level::High));
    let reported = pairs_reported(&findings);
    let pair = |pin: &str, inverting: &str| {
        (
            "RX.U1".to_string(),
            pin.to_string(),
            inverting.to_string(),
            Level::High,
        )
    };
    assert_eq!(reported, [pair("2", "1"), pair("6", "7")], "{findings:?}");
}

#[rstest]
fn with_pull_ups_and_biased_minus_legs_an_am26lv32_counts_an_open_collector_encoder() {
    behaviour!(Test {
        id: "encoder.open-collector-into-am26lv32-biased",
        covers: Some("models/src/machine/quadrature_encoder.rs#OutputStage::NpnOpenCollector"),
        given: "an open-collector encoder on an AM26LV32's first two pairs, each positive \
                input pulled up to the encoder's 5 volt supply through 4.7 kilohms and each \
                negative input held at half that supply, walked through two quadrature cycles \
                and back through zero",
    });
    expect!(
        "counts",
        "at every count the receiver's two outputs read the encoder's A and B levels",
        "the pull-up lifts a released output well above the negative input and a sinking \
         one pulls it well below, a difference of about 1.9 volts either way"
    );
    expect!(
        "nothing-reported",
        "no pair is reported as read inside its band"
    );
    expect!(
        "inputs-in-range",
        "no pin is reported above its recommended range",
        "a released output pulled up to 5 volts against the receiver's 12 kilohm input rests \
         near 3.8 volts, inside the minus 0.3 to 5.5 volt input range SLLS202H recommends"
    );
    let _lock = suite_lock();
    let findings = walk_open_collector(Pairs::PulledAndBiased, &[9, -2], quadrature);
    assert!(
        pairs_reported(&findings).is_empty(),
        "{:?}",
        pairs_reported(&findings)
    );
    assert!(
        limits_reported(&findings).is_empty(),
        "{:?}",
        limits_reported(&findings)
    );
}

// ============================================================
// A step train carried as a rate, through every stage
// ============================================================

/// Steps (and encoder counts) per millimetre: MaD's machine's, 4
/// microsteps × 2048 steps a revolution (`stepper_motor`'s provenance).
const STEPS_PER_MM: f64 = 8_192.0;

/// The train's rate: at most one count an observation, so every count
/// reaches the receiver at an instant of its own.
const TRAIN_HZ: u32 = 500;
const _: () = assert!((TRAIN_HZ as u64) * DEFAULT_OBSERVE_INTERVAL_US < 1_000_000);

/// The steps the train carries.
const TRAIN_STEPS: u64 = 102;

/// The virtual time after the train starts before the case reads: the
/// train, then twenty of the drive's time constants for the carriage to
/// come to rest, then a quarter observation more, between two of the
/// drive's observations.
const REST_NS: u64 = TRAIN_STEPS * 1_000_000_000 / TRAIN_HZ as u64
    + (20.0 * DEFAULT_TAU_S * 1e9) as u64
    + DEFAULT_OBSERVE_INTERVAL_US * 250;

/// The controller's ports: the push-pull default's 25 Ω, at 3.3 V logic.
const HIGH: TheveninDrive = TheveninDrive {
    volts: 3.3,
    impedance: 25.0,
};
const LOW: TheveninDrive = TheveninDrive {
    volts: 0.0,
    impedance: 25.0,
};

type Handles = Arc<Mutex<Vec<PinHandle>>>;

/// The controller: `STEP` resting low, `DIR` and `ENA` high.
struct Controller {
    pins: [PinDecl; 3],
    handles: Handles,
}

impl Component for Controller {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let mut handles = self.handles.lock().unwrap();
        for pin in ["STEP", "DIR", "ENA"] {
            handles.push(io.pin(pin)?);
        }
        Ok(())
    }
}

/// What the probe saw: each delivery of the engine's report of `Y1` and
/// `Y2`, in the engine's order.
type Reports = Arc<Mutex<Vec<(&'static str, NetState)>>>;

/// The probe on the receiver's outputs, recording the engine's report.
struct OutputProbe {
    pins: [PinDecl; 2],
    reports: Reports,
}

impl Component for OutputProbe {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        for pin in ["Y1", "Y2"] {
            let reports = Arc::clone(&self.reports);
            io.on_net_report(pin, move |state| reports.lock().unwrap().push((pin, state)))?;
        }
        Ok(())
    }
}

/// Changes of driven level on the receiver's outputs the probe has seen.
fn transitions(reports: &Reports) -> usize {
    let reports = reports.lock().unwrap();
    let mut last: [Option<Level>; 2] = [None, None];
    let mut changes = 0;
    for (pin, state) in reports.iter() {
        let NetState::Driven(level) = *state else {
            continue;
        };
        let slot = &mut last[usize::from(*pin == "Y2")];
        if slot.is_some_and(|previous| previous != level) {
            changes += 1;
        }
        *slot = Some(level);
    }
    changes
}

#[rstest]
#[case::push_pull(OutputStage::PushPull)]
#[case::rs422(OutputStage::Rs422)]
#[case::npn_open_collector(OutputStage::NpnOpenCollector)]
fn a_step_train_carried_as_a_rate_counts_through_every_encoder_output_stage(
    #[case] stage: OutputStage,
) {
    behaviour!(Test {
        id: "encoder.output-stage-step-train",
        covers: Some("models/src/machine/quadrature_encoder.rs#OutputStage"),
        given: "a drive turning an encoder 102 steps forward by a train carried as a rate, the \
                encoder's outputs through each output stage on an AM26LV32 whose positive \
                inputs are pulled up to 5 volts and negative inputs held at half of it or \
                driven by the line driver's minus legs",
    });
    expect!(
        "count-is-the-steps",
        "at rest the encoder's count is the 102 steps the train carried"
    );
    expect!(
        "receiver-reads-the-count",
        "the receiver's two outputs read that count's A and B levels"
    );
    expect!(
        "one-change-a-count",
        "every count reaches the receiver as one change of one of its outputs, 102 in all",
        "a train at half a step an observation moves the encoder one count at a time, each \
         at an instant of its own; a line driver's two legs are published at that instant \
         and the receiver passes through no other level between them"
    );
    expect!(
        "nothing-reported",
        "no pair is reported as read inside its band"
    );
    let _lock = suite_lock();
    let drive = StepperMotor::new(stepper_motor::Config {
        load_loss: 0.0,
        ..stepper_motor::Config::new(STEPS_PER_MM)
    })
    .expect("the drive's configuration is valid");
    let encoder = QuadratureEncoder::new(EncoderConfig::new(STEPS_PER_MM).with_output(stage))
        .expect("the encoder's configuration is valid");
    let input = encoder.input();
    {
        let input = input.clone();
        drive
            .shaft()
            .on_position_change(move |mm| input.set_position_mm(mm));
    }
    let handles = Handles::default();
    let controller = Controller {
        pins: [
            PinDecl::digital_out("STEP").with_idle(Some(LOW)),
            PinDecl::digital_out("DIR").with_idle(Some(HIGH)),
            PinDecl::digital_out("ENA").with_idle(Some(HIGH)),
        ],
        handles: Arc::clone(&handles),
    };
    let reports = Reports::default();
    let digital = jesd8c01_lvcmos_thresholds(DeadBand::Unknown);
    let probe = OutputProbe {
        pins: [
            PinDecl::digital_in("Y1", digital),
            PinDecl::digital_in("Y2", digital),
        ],
        reports: Arc::clone(&reports),
    };
    let system = receiver_system(Pairs::PulledAndBiased)
        .component("CTRL", Box::new(controller))
        .component("DRIVE", Box::new(drive))
        .component("ENC", Box::new(encoder))
        .component("PROBE", Box::new(probe))
        .harness(
            encoder_onto_receiver(stage)
                .connect(ep("CTRL.STEP"), ep("DRIVE.STEP"))
                .connect(ep("CTRL.DIR"), ep("DRIVE.DIR"))
                .connect(ep("CTRL.ENA"), ep("DRIVE.ENA"))
                .connect(ep("RX.J1.8"), ep("PROBE.Y1"))
                .connect(ep("RX.J1.9"), ep("PROBE.Y2")),
        );
    let (system, actor) = start(system, "encoder-output-stage-step-train-case");
    let outputs = || {
        (
            level(system.net_state("RX.Y1"), "RX.Y1"),
            level(system.net_state("RX.Y2"), "RX.Y2"),
        )
    };
    assert_eq!(outputs(), quadrature(0), "at rest before any step");
    let before = transitions(&reports);
    let step = handles.lock().unwrap()[0].clone();
    step.drive(Drive::Periodic {
        hi: HIGH,
        lo: LOW,
        segment: PeriodicSchedule {
            emitted: 0,
            freq_hz: TRAIN_HZ,
            total: Some(TRAIN_STEPS),
            since_ns: virtual_clock::virtual_ns(),
        },
    });
    virtual_clock::wait_virtual_ns(REST_NS);
    assert_eq!(input.count(), TRAIN_STEPS as i64, "{}", stage.name());
    assert_eq!(
        outputs(),
        quadrature(TRAIN_STEPS as i64),
        "{}",
        stage.name()
    );
    assert_eq!(
        transitions(&reports) - before,
        TRAIN_STEPS as usize,
        "{}",
        stage.name()
    );
    assert_eq!(input.snapped_updates(), 0, "every count walked");
    let findings = finish(system, actor);
    assert!(
        pairs_reported(&findings).is_empty(),
        "{}: {:?}",
        stage.name(),
        pairs_reported(&findings)
    );
}
