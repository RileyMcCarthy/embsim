//! The MaD EdgeBoard as a [`Board`]: 168 components, 16 distinct active part
//! types, three hierarchical sheets, and the P2 module socket that makes it the
//! machine's backplane.
//!
//! # What this binary is for
//!
//! 1. **A real mixed-signal board classifies and builds** — the auto tier's 86
//!    passives, 24 boundaries, 5 jumpers and 4 ignored mechanicals, plus 49
//!    registered components across 16 part types, each with a pin facade the
//!    build validates against the netlist in both directions.
//! 2. **The power topology is honest about what the board cannot generate.** A
//!    bare board reports exactly the domains that arrive over a cable, and
//!    bench straps clear exactly those.
//! 3. **The encoder/servo RS-422 pair works, live.** The differential driver
//!    and receiver ([`machine_parts::Rs422Driver`],
//!    [`machine_parts::Rs422Receiver`]) are the two parts on this board with
//!    real behavior, because the motion path is the one place where "the wire
//!    is connected" is not the interesting claim. Their tests drive the real
//!    netlist's nets and read the real netlist's nets.
//! 4. **The force-gauge UART crosses the isolation barrier as a byte route.**
//!    The isolator is a stream hop, not an opaque break — which is what lets
//!    `machine_system.rs` route a byte from the P2 to the ADC.
//!
//! # Source
//!
//! `fixtures/mad_edge.net` — `kicad-cli sch export netlist` of the MaD repo's
//! `Hardware/EdgeBoard/KiCad/MaD_Edge.kicad_sch` (provenance in the fixture
//! header). Part classification decisions are documented in
//! `machine_parts/mod.rs`.

mod machine_parts;

use std::collections::{BTreeSet, HashMap, HashSet};
use std::sync::Once;
use std::time::{Duration, Instant};

use rstest::rstest;

use embsim_board::{
    Board, Finding, JumperState, Level, NetState, PinRef, Scenario, SenseKind, StreamRole, System,
    SystemHandle,
};
use machine_parts::{
    bench_rails, edge_board, edge_polarity_fet_conducting, encoder_jumpers_closed, iso6731_pins,
    FORCE_GAUGE_BAUD_HZ,
};

/// Registered (non-passive, non-boundary, non-ignored) component count: the 16
/// active part types' instances.
const EXPECTED_REGISTERED: usize = 49;

/// The engine's timer wheel and any paced stream sample the process-global
/// virtual clock, and `init` re-anchors it — so it runs once per binary.
fn ensure_clock() {
    static CLOCK: Once = Once::new();
    CLOCK.call_once(|| embsim_core::virtual_clock::init(1.0, 1_000_000));
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

/// Map every pin to the board-local net name that owns it.
fn net_of_pin(board: &Board) -> HashMap<PinRef, String> {
    let mut map = HashMap::new();
    for net in board.nets() {
        for node in &net.nodes {
            map.insert(node.clone(), net.name.clone());
        }
    }
    map
}

fn net_named(map: &HashMap<PinRef, String>, reference: &str, pin: &str) -> String {
    map.get(&PinRef::new(reference, pin))
        .cloned()
        .unwrap_or_else(|| panic!("{reference}.{pin} is not on any net"))
}

// ============================================================
// Build and classification census
// ============================================================

/// The board builds, and every part lands in the tier the registry intends.
///
/// The census is the interesting half. 168 components resolve into 86
/// auto-classified passives, 24 boundaries (23 standard connector symbols plus
/// the declared `P2_EDGE_MODULE_SOCKET`), 5 jumpers, 4 ignored mounting holes
/// and 49 registered components — and because every registered component's
/// facade is checked against the netlist in both directions, this test failing
/// after a schematic edit is the intended outcome, not a nuisance.
#[rstest]
fn the_edgeboard_builds_with_every_part_classified() {
    let board = edge_board();
    let registered: BTreeSet<&str> = board.component_refs().collect();
    assert_eq!(
        registered.len(),
        EXPECTED_REGISTERED,
        "registered components: {registered:?}"
    );

    // The three modeled parts and one instance of each stub type.
    for reference in [
        "U24",  // AM26LS31CD  — RS-422 driver (modeled)
        "U25",  // AM26LV32xD  — RS-422 receiver (modeled)
        "IC5",  // ISO6731DWR  — force-gauge UART isolator (modeled)
        "IC1",  // ISO6742DWR
        "IC3",  // UCC12040DVER
        "IC14", // ISO6741DWR
        "IC15", // ISO6721BDR
        "IC16", // ISO6740FDWR
        "IC6",  // NSI50010YT1G
        "U4",   // 6N137
        "U5",   // VO2631
        "U9",   // SN74LVC1G14DBV
        "U1",   // XL1509
        "U3",   // APM4953
        "SW1",  // SW_Push
        "Q1",   // 2N3904
    ] {
        assert!(
            registered.contains(reference),
            "{reference} must be a registered component"
        );
    }

    // Boundaries, jumpers and mounting holes are NOT components — they are
    // classified, not instantiated.
    for reference in ["J3", "J9", "J20", "J21", "JP1", "JP4", "H5"] {
        assert!(
            !registered.contains(reference),
            "{reference} classifies in the auto tier, not the registry"
        );
    }
}

/// The board's own edge-socket symbol is a *boundary*, declared as one — so a
/// harness can plug the P2 module into all 80 of its pins without the symbol
/// needing a component or a per-board stub entry.
#[rstest]
fn the_module_socket_is_a_declared_boundary_with_eighty_pins() {
    let board = edge_board();
    let map = net_of_pin(&board);
    for pin in 1..=80u32 {
        assert!(
            map.contains_key(&PinRef::new("J3", pin.to_string())),
            "J3.{pin} must be on a net"
        );
    }
    assert_eq!(map.keys().filter(|p| p.reference == "J3").count(), 80);

    // Undeclared, the same symbol fails the build — the declaration is load
    // bearing, not decoration. (`register_boundary` has no "unregister", so
    // this rebuilds the registry from the same helper with the declaration
    // suppressed.)
    let registry = machine_parts::edge_registry_without_socket();
    let parsed = embsim_board::netlist::parse(include_str!("fixtures/mad_edge.net")).unwrap();
    let error = Board::from_netlist(parsed, &registry)
        .expect_err("an unclassified 80-pin socket symbol must fail the build");
    assert!(
        error.to_string().contains("P2_EDGE_MODULE_SOCKET"),
        "the error must name the part; got {error}"
    );
}

// ============================================================
// Power topology
// ============================================================

/// Unsourced power domains on a bare board, as one sorted list.
fn unsourced_domains(diagnostics: &embsim_board::Diagnostics) -> BTreeSet<String> {
    diagnostics
        .findings()
        .iter()
        .filter_map(|f| match f {
            Finding::PowerNetUnsourced { net } => Some(net.clone()),
            _ => None,
        })
        .collect()
}

/// The unlabeled net carrying `IC14`'s secondary-side ground pins — a real
/// schematic defect this suite found, kept as an assertion rather than swept
/// under a tolerance. See [`the_servo_isolator_secondary_ground_is_unconnected`].
const IC14_ORPHAN_GROUND: &str = "EdgeBoard.Net-(IC14-GND2_1)";

/// A bare board reports exactly the domains it cannot generate: its own input
/// supply and the isolated domains that arrive over a cable. Everything the
/// board makes for itself — `+5V` and `+3.3V` from the two XL1509 bucks through
/// their output inductors, and the isolated I/O and force-gauge domains from the
/// two UCC12040 DC/DC modules — is already sourced with nothing plugged in.
///
/// That list is the board's external dependency surface, derived from the
/// netlist rather than written down: `V_IN` (the screw-terminal input), `GND`
/// (the same connector's return), the Raspberry-Pi header's domain, the servo
/// domain on J21, the isolated servo-serial domain on J23 — and one entry that
/// is a board defect rather than a cable.
#[rstest]
fn a_bare_board_reports_only_the_domains_that_arrive_over_a_cable() {
    let system = System::new()
        .board("EdgeBoard", edge_board())
        .build()
        .expect("a bare board still builds");

    assert_eq!(
        unsourced_domains(system.diagnostics()),
        BTreeSet::from([
            "EdgeBoard./MaD_Edge_Sheet2/RPI_5V".to_string(),
            "EdgeBoard./MaD_Edge_Sheet2/RPI_GND".to_string(),
            "EdgeBoard./MaD_Edge_Sheet3/EN_GND".to_string(),
            "EdgeBoard./MaD_Edge_Sheet3/ISS_5V".to_string(),
            "EdgeBoard./MaD_Edge_Sheet3/ISS_GND".to_string(),
            "EdgeBoard./MaD_Edge_Sheet3/SC_5V".to_string(),
            "EdgeBoard.GND".to_string(),
            "EdgeBoard.V_IN".to_string(),
            IC14_ORPHAN_GROUND.to_string(),
        ])
    );

    // The board's own regulators do source their outputs, so these must NOT
    // appear above: a regression here means a facade lost its `PowerOut`.
    for rail in [
        "EdgeBoard.+5V",
        "EdgeBoard.+3.3V",
        "EdgeBoard./MaD_Edge_Sheet2/5V_IO",
        "EdgeBoard./MaD_Edge_Sheet2/GND_IO",
        "EdgeBoard./MaD_Edge_Sheet2/IFG_5V",
        "EdgeBoard./MaD_Edge_Sheet2/IFG_GND",
    ] {
        assert!(
            !unsourced_domains(system.diagnostics()).contains(rail),
            "{rail} is generated on-board"
        );
    }
}

/// The bench straps clear everything the machine's cables would, and leave the
/// two ports nothing is plugged into still reported. The straps go through the
/// board's own connector pins, so this is the rig a bring-up bench builds, not
/// an engine back door.
#[rstest]
fn bench_straps_clear_the_domains_they_feed() {
    let system = System::new()
        .board("EdgeBoard", edge_board())
        .harness(bench_rails("EdgeBoard"))
        .scenario(edge_polarity_fet_conducting(
            Scenario::default(),
            "EdgeBoard",
        ))
        .build()
        .expect("the strapped board builds");

    assert_eq!(
        unsourced_domains(system.diagnostics()),
        BTreeSet::from([
            // Nothing is plugged into the Raspberry-Pi header (J4) or the
            // isolated servo-serial port (J23).
            "EdgeBoard./MaD_Edge_Sheet2/RPI_5V".to_string(),
            "EdgeBoard./MaD_Edge_Sheet2/RPI_GND".to_string(),
            "EdgeBoard./MaD_Edge_Sheet3/ISS_5V".to_string(),
            "EdgeBoard./MaD_Edge_Sheet3/ISS_GND".to_string(),
            // No cable can clear this one — the pins are not on any rail.
            IC14_ORPHAN_GROUND.to_string(),
        ])
    );

    // The main input reaches V_IN only because the polarity FET is expressed as
    // conducting; without that scenario the board is dark behind it.
    let dark = System::new()
        .board("EdgeBoard", edge_board())
        .harness(bench_rails("EdgeBoard"))
        .build()
        .expect("builds");
    assert!(
        unsourced_domains(dark.diagnostics()).contains("EdgeBoard.V_IN"),
        "with the polarity FET off, V_IN must stay unsourced"
    );
}

// ============================================================
// The force-gauge UART across the isolation barrier
// ============================================================

/// The force-gauge path hop by hop, on this board alone: from the P2's socket
/// pins through the ISO6731 to connector J9, and back.
///
/// | Direction | P2 pin (socket finger) | isolator | connector |
/// |---|---|---|---|
/// | MCU → gauge | `P2` (J3.38) | `INC` → `OUTC` | `IFG_TX` on J9.4 |
/// | gauge → MCU | `P0` (J3.40) | `INA` ← `OUTA` | `IFG_RX` on J9.2 |
/// | `~DRDY` → MCU | `P1` (J3.39) | `INB` → `OUTB` | `IFG_INT` on J9.3 |
///
/// The MCU-side pin numbers are the reason `mcu_component.rs`'s force-gauge
/// channel is `{rx: P0, tx: P2}`: that is what this schematic wired.
#[rstest]
fn the_force_gauge_uart_crosses_the_isolator_between_socket_and_connector() {
    let board = edge_board();
    let map = net_of_pin(&board);

    // MCU transmit: the P2's P2 pin, the socket finger, and the isolator input.
    let mcu_tx = net_named(&map, "IC5", "12");
    assert_eq!(mcu_tx, "P2");
    assert_eq!(net_named(&map, "J3", "38"), mcu_tx);
    // ... out the isolated side to the connector.
    let gauge_rx = net_named(&map, "IC5", "5");
    assert_eq!(gauge_rx, "/MaD_Edge_Sheet2/IFG_TX");
    assert_eq!(net_named(&map, "J9", "4"), gauge_rx);

    // Gauge transmit: connector in, P0 out.
    let gauge_tx = net_named(&map, "IC5", "3");
    assert_eq!(gauge_tx, "/MaD_Edge_Sheet2/IFG_RX");
    assert_eq!(net_named(&map, "J9", "2"), gauge_tx);
    let mcu_rx = net_named(&map, "IC5", "14");
    assert_eq!(mcu_rx, "P0");
    assert_eq!(net_named(&map, "J3", "40"), mcu_rx);

    // The ADC's ~DRDY rides the third channel onto P1.
    assert_eq!(net_named(&map, "IC5", "4"), "/MaD_Edge_Sheet2/IFG_INT");
    assert_eq!(net_named(&map, "J9", "3"), "/MaD_Edge_Sheet2/IFG_INT");
    assert_eq!(net_named(&map, "IC5", "13"), "P1");
    assert_eq!(net_named(&map, "J3", "39"), "P1");
}

/// The isolator's two UART channels are declared as a stream *hop*: the input
/// pin consumes routed bytes and the output pin produces them, both at the
/// force-gauge baud. That declaration is what keeps a byte route derivable
/// across the barrier instead of dying at it.
#[rstest]
fn the_isolator_declares_its_uart_channels_as_stream_hops() {
    let pins = iso6731_pins(FORCE_GAUGE_BAUD_HZ);
    let role = |number: &str| {
        pins.iter()
            .find(|p| p.number == number)
            .unwrap_or_else(|| panic!("pin {number} declared"))
            .stream
    };
    let consumer = Some(StreamRole::Consumer {
        baud_hz: FORCE_GAUGE_BAUD_HZ,
    });
    let producer = Some(StreamRole::Producer {
        baud_hz: FORCE_GAUGE_BAUD_HZ,
    });

    assert_eq!(role("12"), consumer, "INC consumes the MCU's transmit");
    assert_eq!(role("5"), producer, "OUTC reproduces it isolated-side");
    assert_eq!(role("3"), consumer, "INA consumes the gauge's transmit");
    assert_eq!(role("14"), producer, "OUTA reproduces it MCU-side");
    // The ~DRDY channel is levels, not bytes.
    assert_eq!(role("4"), None);
    assert_eq!(role("13"), None);
    assert_eq!(pins.len(), 16);
}

// ============================================================
// The encoder / servo RS-422 pair, live
// ============================================================

/// Start the board with the servo domain powered, the encoder jumpers as given,
/// and optional ideal sources injected on named nets (the stand-in for whatever
/// the isolators or the encoder would drive).
fn start_servo_domain(jumpers_closed: bool, sources: &[(&str, f64)]) -> SystemHandle {
    ensure_clock();
    let mut scenario = edge_polarity_fet_conducting(Scenario::default(), "EdgeBoard");
    if jumpers_closed {
        scenario = encoder_jumpers_closed(scenario, "EdgeBoard");
    }
    for (net, volts) in sources {
        scenario = scenario.net_stuck(net, *volts);
    }
    System::new()
        .board("EdgeBoard", edge_board())
        .harness(bench_rails("EdgeBoard"))
        .scenario(scenario)
        .start()
        .expect("the servo-domain system starts")
}

fn settled_state(system: &SystemHandle, net: &str, expected: NetState) -> NetState {
    wait_for(|| system.net_state(net) == Some(expected), SETTLE);
    system
        .net_state(net)
        .unwrap_or_else(|| panic!("net {net} exists"))
}

/// The AM26LS31 turns one logic input into a complementary pair on the real
/// netlist's `SC_PUL±` nets — the step signal the machine's stepper driver
/// receives. Reversing the input reverses both legs.
///
/// The input arrives at `Net-(IC14-OUTA)`, the isolator output the schematic
/// feeds the driver's `1A` from; injecting there is what a driven `P8` would do
/// once the servo isolator is modeled.
#[rstest]
#[case::high(3.3, Level::High, Level::Low)]
#[case::low(0.0, Level::Low, Level::High)]
fn the_rs422_driver_makes_a_complementary_pair_on_the_step_nets(
    #[case] input_volts: f64,
    #[case] expect_true: Level,
    #[case] expect_complement: Level,
) {
    let system = start_servo_domain(true, &[("EdgeBoard.Net-(IC14-OUTA)", input_volts)]);
    assert_eq!(
        settled_state(
            &system,
            "EdgeBoard./MaD_Edge_Sheet3/SC_PUL+",
            NetState::Driven(expect_true)
        ),
        NetState::Driven(expect_true),
        "SC_PUL+ follows the driver input"
    );
    assert_eq!(
        settled_state(
            &system,
            "EdgeBoard./MaD_Edge_Sheet3/SC_PUL-",
            NetState::Driven(expect_complement)
        ),
        NetState::Driven(expect_complement),
        "SC_PUL- is its complement"
    );
    system.shutdown();
}

/// The direction channel is the driver's second wired channel and behaves
/// identically — proof the model is per-channel and not one hard-wired path.
#[rstest]
fn the_rs422_driver_second_channel_drives_the_direction_pair() {
    let system = start_servo_domain(true, &[("EdgeBoard.Net-(IC14-OUTB)", 3.3)]);
    assert_eq!(
        settled_state(
            &system,
            "EdgeBoard./MaD_Edge_Sheet3/SC_DIR+",
            NetState::Driven(Level::High)
        ),
        NetState::Driven(Level::High)
    );
    assert_eq!(
        settled_state(
            &system,
            "EdgeBoard./MaD_Edge_Sheet3/SC_DIR-",
            NetState::Driven(Level::Low)
        ),
        NetState::Driven(Level::Low)
    );
    system.shutdown();
}

/// With the servo domain unpowered the driver releases both legs: the pair
/// floats rather than presenting a plausible idle level. A stepper driver
/// staring at a floating differential pair is a real failure mode and the
/// engine reports it as one.
#[rstest]
fn an_unpowered_rs422_driver_releases_the_pair() {
    ensure_clock();
    let system = System::new()
        .board("EdgeBoard", edge_board())
        // Main rails only — nothing on J21, so SC_5V is dark.
        .harness(
            embsim_board::Harness::new()
                .power(
                    machine_parts::ep("BENCH.GND"),
                    machine_parts::ep("EdgeBoard.J2.2"),
                    0.0,
                )
                .power(
                    machine_parts::ep("BENCH.3V3"),
                    machine_parts::ep("EdgeBoard.J19.1"),
                    3.3,
                ),
        )
        .scenario(Scenario::default().net_stuck("EdgeBoard.Net-(IC14-OUTA)", 3.3))
        .start()
        .expect("starts");

    assert_eq!(
        settled_state(
            &system,
            "EdgeBoard./MaD_Edge_Sheet3/SC_PUL+",
            NetState::Floating
        ),
        NetState::Floating,
        "an unpowered differential driver must not drive"
    );
    assert!(
        system.findings().iter().any(|f| matches!(
            f,
            Finding::PowerNetUnsourced { net } if net == "EdgeBoard./MaD_Edge_Sheet3/SC_5V"
        )),
        "and the reason must be reported; got {:?}",
        system.findings()
    );
    system.shutdown();
}

/// The AM26LV32 turns the encoder's differential pair back into one logic level,
/// from the *solved node voltages* of the real `A+`/`A-` nets — and rides its
/// datasheet failsafe when the pair carries no differential.
///
/// | `A+` | `A-` | V_ID | `1Y` | why |
/// |---|---|---|---|---|
/// | 3.3 V | 0 V (JP2 closed) | +3.3 V | high | above V_IT+ |
/// | 0 V | 3.3 V (JP2 open, injected) | −3.3 V | low | below V_IT− |
/// | 0 V | 0 V (JP2 closed) | 0 V | high | input failsafe |
#[rstest]
#[case::forward(true, &[("EdgeBoard./MaD_Edge_Sheet3/A+", 3.3)], Level::High)]
#[case::reverse(false, &[("EdgeBoard./MaD_Edge_Sheet3/A+", 0.0), ("EdgeBoard./MaD_Edge_Sheet3/A-", 3.3)], Level::Low)]
#[case::failsafe(true, &[("EdgeBoard./MaD_Edge_Sheet3/A+", 0.0)], Level::High)]
fn the_rs422_receiver_decodes_the_encoder_pair_with_a_failsafe(
    #[case] jumpers_closed: bool,
    #[case] sources: &[(&str, f64)],
    #[case] expected: Level,
) {
    // JP4 (`Z_GND`) must be closed for the receiver to be enabled at all; the
    // reverse case needs JP2 open, so it closes JP4 on its own.
    let mut sources = sources.to_vec();
    let system = if jumpers_closed {
        start_servo_domain(true, &sources)
    } else {
        ensure_clock();
        let mut scenario = edge_polarity_fet_conducting(Scenario::default(), "EdgeBoard")
            .jumper("EdgeBoard.JP4", JumperState::Closed);
        for (net, volts) in sources.drain(..) {
            scenario = scenario.net_stuck(net, volts);
        }
        System::new()
            .board("EdgeBoard", edge_board())
            .harness(bench_rails("EdgeBoard"))
            .scenario(scenario)
            .start()
            .expect("starts")
    };

    // The receiver's channel-1 output is the isolator input the P2 reads as P9.
    assert_eq!(
        settled_state(
            &system,
            "EdgeBoard.Net-(IC16-INA)",
            NetState::Driven(expected)
        ),
        NetState::Driven(expected),
        "the receiver output for this differential"
    );
    system.shutdown();
}

/// The board wires the encoder's index pair to the receiver's **enable** pins,
/// so the `Z_GND` jumper (JP4) is what turns the encoder inputs on. Open, all
/// four receiver outputs are high-Z and the encoder is silent — a wiring trap
/// worth having a test for, since nothing about "the encoder cable is plugged
/// in" suggests a jumper is involved.
#[rstest]
fn the_z_ground_jumper_is_what_enables_the_encoder_receiver() {
    // JP4 open: enables unasserted (`Z+` and `Z-` both floating), outputs
    // released.
    let system = start_servo_domain(false, &[("EdgeBoard./MaD_Edge_Sheet3/A+", 3.3)]);
    assert_eq!(
        settled_state(&system, "EdgeBoard.Net-(IC16-INA)", NetState::Floating),
        NetState::Floating,
        "with JP4 open the receiver must be disabled"
    );
    system.shutdown();

    // JP4 closed: `Z-` sits at the isolated ground, asserting the active-low
    // enable, and the same differential now reads high.
    let system = start_servo_domain(true, &[("EdgeBoard./MaD_Edge_Sheet3/A+", 3.3)]);
    assert_eq!(
        settled_state(
            &system,
            "EdgeBoard.Net-(IC16-INA)",
            NetState::Driven(Level::High)
        ),
        NetState::Driven(Level::High),
        "closing JP4 enables the receiver"
    );
    system.shutdown();
}

/// Channel 4 of the receiver has its inputs marked no-connect while its output
/// is wired to the encoder isolator — the datasheet's open-input failsafe, on
/// this board by construction. The `4Y` net must sit high, not float.
#[rstest]
fn the_unwired_receiver_channel_rides_its_open_input_failsafe() {
    let board = edge_board();
    let map = net_of_pin(&board);
    // The netlist's own account: 4A/4B on no-connect stubs, 4Y wired.
    assert!(net_named(&map, "U25", "14").starts_with("unconnected-("));
    assert!(net_named(&map, "U25", "15").starts_with("unconnected-("));
    assert_eq!(net_named(&map, "U25", "13"), "Net-(IC16-IND)");

    let system = start_servo_domain(true, &[]);
    assert_eq!(
        settled_state(
            &system,
            "EdgeBoard.Net-(IC16-IND)",
            NetState::Driven(Level::High)
        ),
        NetState::Driven(Level::High),
        "an open differential input fails safe high"
    );
    system.shutdown();
}

// ============================================================
// What a bare board leaves floating
// ============================================================

/// The eight isolated digital inputs (P16..P23, two per VO2631 optocoupler)
/// idle through 1 kΩ pull-ups — but the pull-ups are referenced to
/// **`VIO_16_23`, the P2's own I/O-bank rail**, which the EdgeBoard does not
/// generate: it arrives through socket finger 58 from the module's on-board
/// LDO. So on a bare board, fully bench-strapped, all eight inputs float.
///
/// That is not a defect and it is not obvious. It means an end switch cannot be
/// read with the module unplugged — the input stage is powered by the very chip
/// that reads it. `machine_system.rs` asserts the other half: with the module
/// in its socket the same eight nets sit at their pull-ups.
#[rstest]
fn isolated_inputs_float_until_the_module_supplies_their_bank_rail() {
    let board = edge_board();
    let map = net_of_pin(&board);
    // The pull-up's far side is the P2 bank rail, and that rail's only other
    // member on this board is the socket finger.
    assert_eq!(net_named(&map, "R5", "2"), "P16");
    assert_eq!(net_named(&map, "R5", "1"), "VIO_16_23");
    assert_eq!(net_named(&map, "J3", "58"), "VIO_16_23");

    let system = System::new()
        .board("EdgeBoard", edge_board())
        .harness(bench_rails("EdgeBoard"))
        .scenario(edge_polarity_fet_conducting(
            Scenario::default(),
            "EdgeBoard",
        ))
        .build()
        .expect("builds");

    for pin in 16..=23u32 {
        let net = format!("EdgeBoard.P{pin}");
        let state = system
            .nets()
            .iter()
            .find(|n| n.name == net)
            .map(|n| n.state)
            .unwrap_or_else(|| panic!("net {net} exists"));
        assert_eq!(
            state,
            NetState::Floating,
            "{net} has no source until the module powers VIO_16_23"
        );
        assert!(
            system.diagnostics().contains(&Finding::FloatingSense {
                net: net.clone(),
                kind: SenseKind::Digital,
            }),
            "and the P2-side sense must say so"
        );
    }
}

/// **A real defect in the EdgeBoard schematic**, surfaced by the build and kept
/// here as an assertion.
///
/// `IC14` (the ISO6741 carrying the servo control signals) has its
/// secondary-side ground pins — `GND2_1` (pin 9) and `GND2_2` (pin 15) — on an
/// unlabeled net whose only other member is the decoupling capacitor `C26`.
/// They are never tied to `EN_GND`, the isolated ground that its own supply
/// (`SC_5V`), the RS-422 driver `U24`, the receiver `U25`, the encoder isolator
/// `IC16` and the servo connector `J21` all share.
///
/// An isolator's secondary ground is the return for its secondary outputs, so
/// as drawn `IC14`'s three isolated-side outputs have no return path to the
/// driver inputs they feed. On the bench the servo channel would be
/// intermittent or dead depending on leakage — the sort of fault that costs
/// days, and exactly the class of bug the DS2 add-on's floating `~RESET` was.
///
/// If this assertion ever fails, the schematic was fixed: delete the case and
/// the `IC14_ORPHAN_GROUND` entries in the power-topology tests above.
#[rstest]
fn the_servo_isolator_secondary_ground_is_unconnected() {
    let board = edge_board();
    let map = net_of_pin(&board);

    let ground = net_named(&map, "IC14", "9");
    assert_eq!(
        ground, "Net-(IC14-GND2_1)",
        "an unlabeled net, i.e. no label"
    );
    assert_eq!(net_named(&map, "IC14", "15"), ground);

    let holders: BTreeSet<&str> = board
        .nets()
        .iter()
        .find(|n| n.name == ground)
        .expect("net exists")
        .nodes
        .iter()
        .map(|p| p.reference.as_str())
        .collect();
    assert_eq!(
        holders,
        BTreeSet::from(["C26", "IC14"]),
        "only the decoupling cap keeps it company"
    );

    // Its supply IS on the isolated rail, which is what makes the missing
    // ground a mismatch rather than a deliberate second domain.
    assert_eq!(net_named(&map, "IC14", "16"), "/MaD_Edge_Sheet3/SC_5V");
    assert_eq!(net_named(&map, "C26", "1"), "/MaD_Edge_Sheet3/SC_5V");
    // And the parts it drives reference EN_GND.
    assert_eq!(net_named(&map, "U24", "8"), "/MaD_Edge_Sheet3/EN_GND");

    // Which is why no amount of bench strapping clears the finding.
    let system = System::new()
        .board("EdgeBoard", edge_board())
        .harness(bench_rails("EdgeBoard"))
        .scenario(edge_polarity_fet_conducting(
            Scenario::default(),
            "EdgeBoard",
        ))
        .build()
        .expect("builds");
    assert!(
        unsourced_domains(system.diagnostics()).contains(IC14_ORPHAN_GROUND),
        "the orphaned ground survives every strap; got {:?}",
        unsourced_domains(system.diagnostics())
    );

    // Tying it to EN_GND — the one-wire fix — clears it.
    let fixed = System::new()
        .board("EdgeBoard", edge_board())
        .harness(bench_rails("EdgeBoard"))
        .scenario(
            edge_polarity_fet_conducting(Scenario::default(), "EdgeBoard")
                .pin_short("EdgeBoard.IC14.9", "EdgeBoard.U24.8"),
        )
        .build()
        .expect("builds");
    assert!(
        !unsourced_domains(fixed.diagnostics()).contains(IC14_ORPHAN_GROUND),
        "shorting IC14.GND2 to the isolated ground is the fix"
    );
}

/// The socket pins for the P2 signals the module keeps for itself — P40..P57
/// and the V40/V48 bank supplies — have nothing behind them on this board
/// either. The EdgeBoard breaks them out (its socket symbol draws all 80
/// fingers) and the module never connects them, so they resolve floating in the
/// assembled system. Asserting it here means the finding in
/// `machine_system.rs` is understood rather than tolerated.
/// The P40..P57 fingers the EC32MB module consumes internally (four PSRAMs,
/// plus their common CLK/CE) still appear on the EdgeBoard's socket — and on
/// this board two other loads sit on them as well:
///
/// - `J24` (GPIO_EXT_2) breaks out **P40..P47**, and
/// - `IC2` (the RPI-serial isolator) sits on **P52..P55**.
///
/// On an EC32MB module those pins are not free: the Rev B product guide
/// routes P40-P57 to the on-module 32 MB RAM (P56 = PSRAM CLK, P57 = CE).
/// So this board can only use that header and that isolator on a plain
/// `P2-EC` module, or on an EC32MB whose firmware never brings up the PSRAM
/// — which is exactly how the machine runs today.
///
/// The exact-holder assertion below pins that conflict: if a board revision
/// moves those signals (or a module change frees the pins), this test fails
/// and the mapping gets revisited deliberately.
#[rstest]
fn the_socket_breaks_out_pins_the_module_never_connects() {
    let board = edge_board();
    let map = net_of_pin(&board);
    for pin in 40..=57u32 {
        let net = net_named(&map, "J3", &socket_finger_for(pin).to_string());
        assert_eq!(net, format!("P{pin}"));
        let holders: HashSet<&str> = board
            .nets()
            .iter()
            .find(|n| n.name == net)
            .expect("net exists")
            .nodes
            .iter()
            .map(|p| p.reference.as_str())
            .collect();
        // Exclusivity, not mere membership: J3 is how the net was looked up,
        // so `contains("J3")` would be true by construction. Asserting the
        // EXACT holder set is what makes this test able to fail — and what
        // it pins is a real conflict in the shipped hardware (see below).
        let expected: HashSet<&str> = match pin {
            // GPIO_EXT_2 breaks these out to a header...
            40..=47 => HashSet::from(["J3", "J24"]),
            // ...and the RPI-serial isolator sits on these.
            52..=55 => HashSet::from(["J3", "IC2"]),
            _ => HashSet::from(["J3"]),
        };
        assert_eq!(
            holders, expected,
            "P{pin}'s holders changed; net {net} holds {holders:?}"
        );
    }
}

/// Finger number for a P2 signal on the 80-way card edge, for the P40..P57
/// block (`P40` = 76 descending, with the V40/V48 supplies interleaved at 77
/// and 67).
fn socket_finger_for(pin: u32) -> u32 {
    match pin {
        40..=47 => 76 - (pin - 40),
        48..=55 => 66 - (pin - 48),
        56 => 56,
        57 => 55,
        other => panic!("no mapping for P{other}"),
    }
}
