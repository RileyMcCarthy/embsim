//! **The whole machine as a netlist-grounded system.**
//!
//! Three real boards and four machine components, assembled from their EDA
//! netlists and connected only by harnesses:
//!
//! ```text
//!            ┌───────────────────────┐
//!            │ EC32MB module         │  p2_ec32mb.net   114 comps
//!            │  U100 = P2 (McuComp.) │
//!            └──────── J203 ─────────┘  80-way card edge
//!                      │ 58 fingers, 1:1
//!            ┌──────── J3 ───────────┐
//!            │ EdgeBoard             │  mad_edge.net    168 comps
//!            │  IC5  force UART iso  │
//!            │  IC14 servo iso       │
//!            │  U24  RS-422 driver   │
//!            │  U25  RS-422 receiver │
//!            └─ J9 ─ J21 ─ J20 ─ J16/J15
//!               │     │     │     │
//!               │     │     │     └── END_U / END_L   embsim-models
//!               │     │     └──────── ENC             embsim-models
//!               │     └────────────── MOTOR           embsim-models
//!               └──────────────────── DS2Addon        ds2_addon.net  31 comps
//!                                      U1 = ADS122U04 (live model)
//! ```
//!
//! # What is asserted
//!
//! This is a **build and topology** milestone, not a motion test — behavioral
//! motion belongs to the consuming repo, which owns the plant. What belongs
//! here is that the assembly *is what the schematics say*:
//!
//! - the system builds with no unexpected findings across ~313 components;
//! - the card-edge correspondence holds for all 58 declared fingers, so the
//!   module's silicon and the EdgeBoard's nets are one graph;
//! - the P2's force-gauge serial route reaches the ADC through the isolator and
//!   the cable — asserted hop by hop *and* exercised with a real command/reply
//!   byte exchange, because a route that resolves but does not carry bytes is
//!   not a route;
//! - the motion-path nets connect P2 pins to the motor and encoder components
//!   through the isolators and the RS-422 pair.
//!
//! # Part and harness decisions
//!
//! All of them, with reasoning, are in `machine_parts/mod.rs`: the per-board
//! classification census, the stub pin-kind rule, the three modeled parts'
//! datasheet provenance, the finger↔socket numbering, and the three places
//! where the silkscreen and the netlist disagree.

mod machine_parts;

use std::collections::{BTreeSet, HashMap, HashSet};
use std::sync::{Arc, Mutex, Once};
use std::time::{Duration, Instant};

use rstest::rstest;

use embsim_board::{
    BuiltSystem, Finding, JumperState, Level, NetState, PinRef, Scenario, System, SystemHandle,
};
use embsim_models::machine::{
    end_switch, quadrature_encoder, stepper_motor, ActuationSense, EndSwitch, QuadratureEncoder,
    StepperMotor,
};
use embsim_peripherals::serial;
use machine_parts::{
    bench_rails, ds2_board, ec32mb_board, edge_board, edge_fingers, edge_polarity_fet_conducting,
    encoder_jumpers_closed, force_domain_rails, force_gauge_harness, machine_harness,
    module_polarity_fet_conducting, module_socket_harness,
};

/// Board names used throughout.
const MODULE: &str = "EC32MB";
const EDGE: &str = "EdgeBoard";
const DS2: &str = "DS2Addon";

fn ensure_clock() {
    static CLOCK: Once = Once::new();
    // 50× so the 115.2 kbaud stream pacing stays sub-millisecond in wall time.
    CLOCK.call_once(|| embsim_core::virtual_clock::init(50.0, 1_000_000));
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

// ============================================================
// The machine, assembled
// ============================================================

/// The scenario every assembly shares: both boards' reverse-polarity pass FETs
/// conducting, the encoder's ground/enable jumpers closed, the DS2 add-on's
/// input jumpers closed, and the reset bodge the DS2 board needs (its `~RESET`
/// net has exactly one pin — the July 2026 bench bug, still true of the
/// netlist).
fn machine_scenario() -> Scenario {
    let scenario = module_polarity_fet_conducting(Scenario::default(), MODULE);
    let scenario = edge_polarity_fet_conducting(scenario, EDGE);
    let scenario = encoder_jumpers_closed(scenario, EDGE);
    scenario
        .jumper(&format!("{DS2}.JP1"), JumperState::Closed)
        .jumper(&format!("{DS2}.JP2"), JumperState::Closed)
        .pin_short(&format!("{DS2}.U1.3"), &format!("{DS2}.U1.13"))
}

/// Every harness: the module in its socket, the force-gauge cable, the machine
/// cables, and the bench supplies for both the primary and the isolated
/// domains.
fn machine_system() -> System {
    let mut system = System::new()
        .board(MODULE, ec32mb_board())
        .board(EDGE, edge_board())
        .board(DS2, ds2_board())
        .harness(module_socket_harness(MODULE, EDGE))
        .harness(force_gauge_harness(EDGE, DS2))
        .harness(machine_harness(EDGE))
        .harness(bench_rails(EDGE))
        .harness(force_domain_rails(DS2))
        .scenario(machine_scenario());

    // The machine components. The motor's shaft feeds the encoder — the one
    // physics coupling this test needs, because it is what makes MOTOR and ENC
    // one axis rather than two unrelated parts.
    let motor = StepperMotor::new(stepper_motor::Config::new(80.0)).expect("motor config");
    let encoder =
        QuadratureEncoder::new(quadrature_encoder::Config::new(80.0)).expect("encoder config");
    {
        let input = encoder.input();
        motor
            .shaft()
            .on_position_change(move |mm| input.set_position_mm(mm));
    }
    let upper = EndSwitch::new(end_switch::Config::new(120.0, ActuationSense::Increasing))
        .expect("upper end-switch config");
    let lower = EndSwitch::new(end_switch::Config::new(0.0, ActuationSense::Decreasing))
        .expect("lower end-switch config");

    system = system
        .component("MOTOR", Box::new(motor))
        .component("ENC", Box::new(encoder))
        .component("END_U", Box::new(upper))
        .component("END_L", Box::new(lower));
    system
}

fn build_machine() -> BuiltSystem {
    let _guard = machine_parts::lock_module_instance();
    machine_system().build().expect("the machine builds")
}

/// Pin → system net name, across every board.
fn net_of_pin(system: &BuiltSystem) -> HashMap<(String, PinRef), String> {
    let mut map = HashMap::new();
    for net in system.nets() {
        // System net names are "Board.NETNAME"; the board is the prefix up to
        // the first dot, which is unambiguous because board names contain none.
        let (board, _) = net.name.split_once('.').unwrap_or((net.name.as_str(), ""));
        for node in &net.nodes {
            map.insert((board.to_string(), node.clone()), net.name.clone());
        }
    }
    map
}

fn net_named(
    map: &HashMap<(String, PinRef), String>,
    board: &str,
    reference: &str,
    pin: &str,
) -> String {
    map.get(&(board.to_string(), PinRef::new(reference, pin)))
        .cloned()
        .unwrap_or_else(|| panic!("{board}.{reference}.{pin} is not on any net"))
}

fn state_of(system: &BuiltSystem, net: &str) -> NetState {
    system
        .nets()
        .iter()
        .find(|n| n.name == net)
        .map(|n| n.state)
        .unwrap_or_else(|| panic!("net {net} exists"))
}

// ============================================================
// The build itself
// ============================================================

/// The whole machine builds, and nothing fights over a net.
///
/// ~313 netlist components across three boards, four harnesses, four bench
/// components, and one scenario. Every registered component's pin facade was
/// validated against its netlist in both directions to get here, so a
/// classification or facade regression anywhere fails this first.
///
/// This used to also assert no `StreamMismatch`. It no longer can: the
/// assembled machine declares no `StreamRole` anywhere now that the force-gauge
/// UART is carried as levels, so that finding is unreachable and asserting its
/// absence would be a guard that can never fail. The link's health is asserted
/// where it is now observable — the two UART nets resolve to a level rather
/// than floating or contending, below.
#[rstest]
fn the_whole_machine_builds_without_contention() {
    let system = build_machine();
    let findings = system.diagnostics().findings();

    let contention: Vec<&Finding> = findings
        .iter()
        .filter(|f| matches!(f, Finding::Contention { .. }))
        .collect();
    assert!(
        contention.is_empty(),
        "no two parts may fight over a net; got {contention:?}"
    );
}

/// The unsourced power domains of the assembled machine are exactly the ports
/// nothing is plugged into, plus one board defect — the same list
/// `edgeboard.rs` derives for a bare board, minus everything the module and the
/// cables now supply.
///
/// | Domain | Why it is still unsourced |
/// |---|---|
/// | `RPI_5V` / `RPI_GND` | no Raspberry Pi on the J4 header |
/// | `ISS_5V` / `ISS_GND` | nothing on the isolated servo-serial port J23 |
/// | `Net-(IC14-GND2_1)` | the servo isolator's secondary ground is unwired on the schematic (see `edgeboard.rs`) |
#[rstest]
fn the_assembled_machine_leaves_only_the_unplugged_ports_unsourced() {
    let system = build_machine();
    let unsourced: BTreeSet<String> = system
        .diagnostics()
        .findings()
        .iter()
        .filter_map(|f| match f {
            Finding::PowerNetUnsourced { net } => Some(net.clone()),
            _ => None,
        })
        .collect();

    assert_eq!(
        unsourced,
        BTreeSet::from([
            "EdgeBoard./MaD_Edge_Sheet2/RPI_5V".to_string(),
            "EdgeBoard./MaD_Edge_Sheet2/RPI_GND".to_string(),
            "EdgeBoard./MaD_Edge_Sheet3/ISS_5V".to_string(),
            "EdgeBoard./MaD_Edge_Sheet3/ISS_GND".to_string(),
            "EdgeBoard.Net-(IC14-GND2_1)".to_string(),
        ])
    );

    // Everything the module supplies is now live — including the P2 bank rails
    // the EdgeBoard's own pull-ups hang off.
    for rail in ["EC32MB.VIO_16_23", "EdgeBoard./MaD_Edge_Sheet3/SC_5V"] {
        assert!(
            !unsourced.contains(rail),
            "{rail} must be sourced in the assembled machine"
        );
    }
}

// ============================================================
// The card edge: finger ↔ socket pin, all 58
// ============================================================

/// The module and the EdgeBoard are one graph across the card edge. For every
/// declared finger, the module's net and the EdgeBoard's net resolve to the
/// same state — which is only possible if the harness merged them, since the
/// engine performs **no implicit net-name merging across boards**.
///
/// Two of the 58 are asymmetric enough to name individually: the P2's bridged
/// UART transmit pin drives finger 38 (so both sides read `Driven(High)`, the
/// idle line), and the force isolator drives finger 40 from the EdgeBoard side
/// (so the drive crosses the socket in the other direction).
#[rstest]
fn every_declared_finger_is_one_node_across_the_socket() {
    let system = build_machine();
    let map = net_of_pin(&system);

    let mut checked = 0usize;
    for finger in edge_fingers() {
        let module_net = net_named(&map, MODULE, "J203", &finger.to_string());
        let edge_net = net_named(&map, EDGE, "J3", &finger.to_string());
        assert_eq!(
            state_of(&system, &module_net),
            state_of(&system, &edge_net),
            "finger {finger}: {module_net} and {edge_net} must be one node"
        );
        checked += 1;
    }
    assert_eq!(checked, 58);

    // The two sides agree on what each finger *is*, not merely that it is one
    // node: for the free P0..P37 block, the P2 pin on the module's net and the
    // signal label on the EdgeBoard's net are the same number.
    for pin in 0..=37u32 {
        let finger = 40 - pin;
        let module_net = net_named(&map, MODULE, "U100", &format!("P{pin}"));
        assert_eq!(
            module_net,
            net_named(&map, MODULE, "J203", &finger.to_string()),
            "the module puts P{pin} on finger {finger}"
        );
        assert_eq!(
            net_named(&map, EDGE, "J3", &finger.to_string()),
            format!("EdgeBoard.P{pin}"),
            "and the EdgeBoard labels socket pin {finger} P{pin}"
        );
    }

    // Module → EdgeBoard: the P2's bridged TX pin idles high on both sides.
    assert_eq!(
        state_of(&system, "EC32MB.P2_IO2"),
        NetState::Driven(Level::High)
    );
    assert_eq!(
        state_of(&system, "EdgeBoard.P2"),
        NetState::Driven(Level::High)
    );
    // EdgeBoard → module: the force isolator's MCU-side output drives P0.
    assert_eq!(
        state_of(&system, "EdgeBoard.P0"),
        state_of(&system, "EC32MB.P2_IO0")
    );
    assert!(
        matches!(state_of(&system, "EC32MB.P2_IO0"), NetState::Driven(_)),
        "the isolator drives the P2's force-gauge RX pin; got {:?}",
        state_of(&system, "EC32MB.P2_IO0")
    );
}

/// The socket pins for signals the module keeps for itself stay dark: the
/// EdgeBoard breaks out all 80 fingers, the module connects 58, and every
/// position in the P40..P57 / V40 / V48 block the PSRAMs own resolves floating.
/// Asserting it makes the absence deliberate — a consumer wiring something to
/// socket pin 76 gets a finding, not silence.
#[rstest]
fn socket_pins_the_module_never_connects_stay_floating() {
    let system = build_machine();
    let map = net_of_pin(&system);
    let harnessed: BTreeSet<u32> = edge_fingers().collect();

    let mut checked = 0usize;
    for finger in 1..=80u32 {
        if harnessed.contains(&finger) {
            continue;
        }
        let net = net_named(&map, EDGE, "J3", &finger.to_string());
        // Fingers 1/2 are the socket's own no-connect pads (their EdgeBoard net
        // is an `unconnected-(…)` stub); everything else in the block is a
        // broken-out signal or bank supply with nothing behind it.
        assert_eq!(
            state_of(&system, &net),
            NetState::Floating,
            "socket pin {finger} ({net}) has nothing behind it"
        );
        checked += 1;
    }
    assert_eq!(checked, 22, "80 fingers less the 58 the module connects");
}

// ============================================================
// The force-gauge route: topology, then bytes
// ============================================================

/// The force-gauge serial route from the P2 to the ADC, hop by hop across three
/// boards and two cables. Every hop is a net the netlists declare; nothing here
/// is a wire this test invented.
///
/// | # | Hop | Net |
/// |---|---|---|
/// | 1 | P2 `P2` pin | `EC32MB.P2_IO2` |
/// | 2 | card edge finger 38 | (same net, merged) `EdgeBoard.P2` |
/// | 3 | isolator input `IC5.INC` | `EdgeBoard.P2` |
/// | 4 | isolator output `IC5.OUTC` | `EdgeBoard./MaD_Edge_Sheet2/IFG_TX` |
/// | 5 | force cable J9.4 → J1.3 | (merged) `DS2Addon.Net-(J1-Pin_3)` |
/// | 6 | 47 Ω series `R3` | `DS2Addon.Net-(U1-RX)` |
/// | 7 | ADC `U1.16` (RX) | `DS2Addon.Net-(U1-RX)` |
///
/// and the reply runs `U1.15` → `R4` → J1.4 → J9.2 → `IC5.INA` → `IC5.OUTA` →
/// `EdgeBoard.P0` → finger 40 → the P2's `P0` pin.
#[rstest]
fn the_force_gauge_route_reaches_the_adc_through_isolator_and_cable() {
    let system = build_machine();
    let map = net_of_pin(&system);

    // Outbound: MCU transmit.
    let mcu_tx = net_named(&map, MODULE, "U100", "P2");
    assert_eq!(mcu_tx, "EC32MB.P2_IO2");
    assert_eq!(net_named(&map, MODULE, "J203", "38"), mcu_tx);
    let edge_tx = net_named(&map, EDGE, "J3", "38");
    assert_eq!(edge_tx, "EdgeBoard.P2");
    assert_eq!(net_named(&map, EDGE, "IC5", "12"), edge_tx);
    let isolated_tx = net_named(&map, EDGE, "IC5", "5");
    assert_eq!(isolated_tx, "EdgeBoard./MaD_Edge_Sheet2/IFG_TX");
    assert_eq!(net_named(&map, EDGE, "J9", "4"), isolated_tx);
    let ds2_in = net_named(&map, DS2, "J1", "3");
    assert_eq!(ds2_in, "DS2Addon.Net-(J1-Pin_3)");
    assert_eq!(net_named(&map, DS2, "R3", "1"), ds2_in);
    let adc_rx = net_named(&map, DS2, "R3", "2");
    assert_eq!(adc_rx, "DS2Addon.Net-(U1-RX)");
    assert_eq!(net_named(&map, DS2, "U1", "16"), adc_rx);

    // Inbound: the ADC's reply.
    let adc_tx = net_named(&map, DS2, "U1", "15");
    assert_eq!(net_named(&map, DS2, "R4", "2"), adc_tx);
    let ds2_out = net_named(&map, DS2, "R4", "1");
    assert_eq!(net_named(&map, DS2, "J1", "4"), ds2_out);
    let isolated_rx = net_named(&map, EDGE, "J9", "2");
    assert_eq!(isolated_rx, "EdgeBoard./MaD_Edge_Sheet2/IFG_RX");
    assert_eq!(net_named(&map, EDGE, "IC5", "3"), isolated_rx);
    let edge_rx = net_named(&map, EDGE, "IC5", "14");
    assert_eq!(edge_rx, "EdgeBoard.P0");
    assert_eq!(net_named(&map, EDGE, "J3", "40"), edge_rx);
    assert_eq!(net_named(&map, MODULE, "U100", "P0"), "EC32MB.P2_IO0");

    // And the ADC's ~DRDY rides the isolator's third channel onto the P2's P1.
    assert_eq!(
        net_named(&map, DS2, "J1", "5"),
        net_named(&map, DS2, "R5", "2")
    );
    assert_eq!(
        net_named(&map, EDGE, "J9", "3"),
        "EdgeBoard./MaD_Edge_Sheet2/IFG_INT"
    );
    assert_eq!(net_named(&map, EDGE, "IC5", "13"), "EdgeBoard.P1");
}

/// The route carries bytes. A `SYNC` + `RDATA` command written through the
/// firmware's own HAL serial call crosses the bridged MCU channel, the card
/// edge, the isolation barrier, the force cable and two 47 Ω series resistors
/// into the ADC — and the ADC's three-byte conversion comes all the way back.
///
/// This is the whole force path of the real machine, assembled from three
/// netlists with no hand wiring anywhere: 313 components, and the only reason
/// the byte arrives is that every hop resolved.
///
/// The MCU is in facade mode, so the bytes go in and out through the
/// process-default peripheral bank exactly as the HAL trampolines would — which
/// is why this test holds the module-instance lock for its whole duration.
#[rstest]
fn the_force_path_carries_a_command_and_a_conversion_end_to_end() {
    ensure_clock();
    let _guard = machine_parts::lock_module_instance();
    // The runtime's role: size the default instance's serial bank before
    // wiring, exactly as `Emulator::run` does. Channel 0 is the bridged
    // force-gauge channel.
    serial::init(1);

    let system: SystemHandle = machine_system().start().expect("the machine starts");

    // The link is closed end to end: both UART nets carry a level, not a
    // float and not a fight. This is what "routes cleanly" means once the
    // payload is on the net — there is no route to complain about any more,
    // only a wire that either reaches the other end or does not.
    for net in [
        "EdgeBoard.P2",
        "EdgeBoard./MaD_Edge_Sheet2/IFG_TX",
        "DS2Addon.Net-(U1-RX)",
        "DS2Addon.Net-(U1-TX)",
    ] {
        let state = system.net_state(net);
        assert!(
            matches!(
                state,
                Some(NetState::Driven(_) | NetState::Pulled(_, _) | NetState::Analog(_))
            ),
            "{net} must carry a level for a byte to cross it; got {state:?}"
        );
    }

    // SYNC + RDATA (0x55 0x10 — TI SBAS752B §8.5.3.4), written the way the
    // firmware writes it.
    serial::transmit_data(0, &[0x55, 0x10]);
    let mut reply: Vec<u8> = Vec::new();
    let arrived = wait_for(
        || {
            while let Some(byte) = serial::receive_byte(0) {
                reply.push(byte);
            }
            reply.len() >= 3
        },
        Duration::from_secs(20),
    );
    assert!(
        arrived,
        "the ADC's conversion must reach the P2 through the whole machine; got {reply:?}"
    );
    assert_eq!(
        reply.len(),
        3,
        "exactly one conversion frame; got {reply:?}"
    );

    drop(system);
    serial::reset();
}

// ============================================================
// The motion path
// ============================================================

/// The step/direction path from the P2 to the motor, hop by hop: MCU pin →
/// card edge → servo isolator → RS-422 driver → differential pair →
/// connector J21 → the motor component's `STEP` pin.
///
/// The isolator (`IC14`) is topology-only in this slice, so the chain is
/// asserted as connectivity rather than as a pulse arriving; the RS-422 pair's
/// own behavior is exercised live in `edgeboard.rs`. What this establishes is
/// that the firmware pin the MaD consumer drives for `STEP` really is the one
/// wired to the machine's motor.
#[rstest]
fn the_step_path_connects_a_p2_pin_to_the_motor_component() {
    let system = build_machine();
    let map = net_of_pin(&system);

    // P8 is the step pin: MCU → finger 32 → EdgeBoard P8 → IC14.INA.
    let step_mcu = net_named(&map, MODULE, "U100", "P8");
    assert_eq!(step_mcu, "EC32MB.P2_IO8");
    assert_eq!(net_named(&map, MODULE, "J203", "32"), step_mcu);
    assert_eq!(net_named(&map, EDGE, "J3", "32"), "EdgeBoard.P8");
    assert_eq!(net_named(&map, EDGE, "IC14", "3"), "EdgeBoard.P8");

    // Isolator output → RS-422 driver input → the differential pair.
    let isolated_step = net_named(&map, EDGE, "IC14", "14");
    assert_eq!(net_named(&map, EDGE, "U24", "1"), isolated_step);
    let pulse_plus = net_named(&map, EDGE, "U24", "2");
    let pulse_minus = net_named(&map, EDGE, "U24", "3");
    assert_eq!(pulse_plus, "EdgeBoard./MaD_Edge_Sheet3/SC_PUL+");
    assert_eq!(pulse_minus, "EdgeBoard./MaD_Edge_Sheet3/SC_PUL-");
    assert_eq!(net_named(&map, EDGE, "J21", "2"), pulse_plus);
    assert_eq!(net_named(&map, EDGE, "J21", "3"), pulse_minus);

    // And the cable puts the motor's STEP pin on the same node as SC_PUL+.
    assert_eq!(
        state_of(&system, pulse_plus.as_str()),
        state_of(&system, "MOTOR.STEP"),
        "the motor's STEP pin and SC_PUL+ must be one node"
    );

    // The direction pin (P7) takes the driver's second channel.
    assert_eq!(net_named(&map, EDGE, "IC14", "4"), "EdgeBoard.P7");
    assert_eq!(net_named(&map, MODULE, "J203", "33"), "EC32MB.P2_IO7");
    let dir_plus = net_named(&map, EDGE, "U24", "6");
    assert_eq!(dir_plus, "EdgeBoard./MaD_Edge_Sheet3/SC_DIR+");
    assert_eq!(
        state_of(&system, dir_plus.as_str()),
        state_of(&system, "MOTOR.DIR")
    );
}

/// The encoder path in the other direction: the encoder component's `A` output
/// → connector J20 → the RS-422 receiver's differential input → the encoder
/// isolator → the P2's `P9` pin, through finger 31.
///
/// The receiver is a modeled part, so this chain is live end to end on the
/// EdgeBoard side: the encoder drives `A+`, the board's `A_GND` jumper holds
/// `A-` at the isolated ground, and the receiver's output resolves — asserted
/// here as a driven state rather than merely a connection.
#[rstest]
fn the_encoder_path_connects_the_encoder_component_to_a_p2_pin() {
    let system = build_machine();
    let map = net_of_pin(&system);

    // Encoder A → J20.1 → A+ → the receiver's 1A input.
    let a_plus = net_named(&map, EDGE, "J20", "1");
    assert_eq!(a_plus, "EdgeBoard./MaD_Edge_Sheet3/A+");
    assert_eq!(net_named(&map, EDGE, "U25", "2"), a_plus);
    assert_eq!(
        state_of(&system, a_plus.as_str()),
        state_of(&system, "ENC.A"),
        "the encoder's A output and A+ must be one node"
    );
    // The A_GND jumper holds the complementary leg at the isolated ground, so
    // the receiver sees a real differential.
    assert_eq!(
        net_named(&map, EDGE, "JP2", "1"),
        "EdgeBoard./MaD_Edge_Sheet3/A-"
    );
    assert_eq!(
        net_named(&map, EDGE, "JP2", "2"),
        "EdgeBoard./MaD_Edge_Sheet3/EN_GND"
    );

    // Receiver output → encoder isolator input → the P2's P9 pin.
    let received = net_named(&map, EDGE, "U25", "3");
    assert_eq!(net_named(&map, EDGE, "IC16", "3"), received);
    assert_eq!(net_named(&map, EDGE, "IC16", "14"), "EdgeBoard.P9");
    assert_eq!(net_named(&map, EDGE, "J3", "31"), "EdgeBoard.P9");
    assert_eq!(net_named(&map, MODULE, "U100", "P9"), "EC32MB.P2_IO9");

    // And the receiver is actually driving it — a connected-but-dead path would
    // pass every assertion above.
    assert!(
        matches!(state_of(&system, received.as_str()), NetState::Driven(_)),
        "the differential receiver must drive its output; got {:?}",
        state_of(&system, received.as_str())
    );

    // Channel B is the second half of the quadrature pair.
    assert_eq!(
        state_of(&system, "EdgeBoard./MaD_Edge_Sheet3/B+"),
        state_of(&system, "ENC.B")
    );
}

/// The end-of-travel switches sit on the isolated-input loops that reach the
/// P2 through the current-regulator/optocoupler chain, and — now that the
/// module supplies `VIO_16_23` — those P2 pins idle at their 1 kΩ pull-ups
/// instead of floating (the other half of `edgeboard.rs`'s
/// `isolated_inputs_float_until_the_module_supplies_their_bank_rail`).
///
/// The connector labels are crossed with the net labels on this board: the
/// upper end switch's net (`IEND_U`) is on J16, silkscreened `Door`. The
/// harness follows the nets, because the optocoupler chain and therefore the
/// firmware's pin assignment do (see `machine_parts::machine_harness`).
#[rstest]
fn the_end_switch_loops_reach_the_p2_through_pulled_up_inputs() {
    let system = build_machine();
    let map = net_of_pin(&system);

    // The upper switch's loop: J16 → current regulator IC9 → opto U6 → P19.
    let upper_high = net_named(&map, EDGE, "J16", "2");
    assert_eq!(upper_high, "EdgeBoard./MaD_Edge_Sheet2/IEND_U+");
    assert_eq!(net_named(&map, EDGE, "IC9", "2"), upper_high);
    let upper_low = net_named(&map, EDGE, "J16", "1");
    assert_eq!(upper_low, "EdgeBoard./MaD_Edge_Sheet2/IEND_U-");
    assert_eq!(net_named(&map, EDGE, "U6", "3"), upper_low);
    assert_eq!(net_named(&map, EDGE, "U6", "6"), "EdgeBoard.P19");

    // The switch component is wired onto those two nets. Assert net
    // IDENTITY (the harness merged them into one electrical node), not
    // equal NetState: this loop carries no rail, so both sides resolve
    // Floating and a state comparison would pass even with no harness at
    // all. `net_id` first, so a renamed net fails loudly rather than
    // reading as "not wired".
    for (edge_net, switch_pin) in [
        (upper_high.as_str(), "END_U.COM"),
        (upper_low.as_str(), "END_U.NO"),
    ] {
        assert!(
            system.net_id(edge_net).is_some(),
            "{edge_net} must exist to compare identity"
        );
        assert!(
            system.net_id(switch_pin).is_some(),
            "{switch_pin} must exist to compare identity"
        );
        assert!(
            system.names_are_merged(edge_net, switch_pin),
            "the harness must merge {switch_pin} onto {edge_net} — they are \
             one node once the switch is plugged in"
        );
    }
    // And the negative: the switch's two terminals are NOT the same node
    // (a merge bug that shorted them would otherwise go unnoticed).
    assert!(
        !system.names_are_merged("END_U.COM", "END_U.NO"),
        "the switch's own terminals must stay distinct nodes"
    );

    // The lower switch is the same shape on J15 → IC10 → U7 → P20.
    assert_eq!(
        net_named(&map, EDGE, "J15", "2"),
        "EdgeBoard./MaD_Edge_Sheet2/IEND_L+"
    );
    assert_eq!(net_named(&map, EDGE, "U7", "7"), "EdgeBoard.P20");

    // With the module plugged in, the eight isolated inputs are pulled up.
    for pin in 16..=23u32 {
        let net = format!("EdgeBoard.P{pin}");
        assert!(
            matches!(state_of(&system, &net), NetState::Pulled(Level::High, _)),
            "{net} must idle at its pull-up now the module powers VIO_16_23; got {:?}",
            state_of(&system, &net)
        );
    }
}

// ============================================================
// The machine components are one axis
// ============================================================

/// Solved node voltage of a net on the live path, or a panic naming the state.
fn live_volts(system: &SystemHandle, net: &str) -> Option<f64> {
    match system.net_state(net) {
        Some(NetState::Analog(volts)) => Some(volts),
        _ => None,
    }
}

/// The encoder's channel drives the EdgeBoard's `A+` net, and — because the
/// RS-422 receiver declares its differential inputs as **analog** pins — that
/// net resolves through the cluster solver to a node *voltage*, not to a digital
/// projection. Reading `Analog(0.0)` / `Analog(3.3)` here rather than
/// `Driven(Low)` / `Driven(High)` is the receiver's ±200 mV threshold getting
/// the numbers it needs, and is the difference between this pair and an ordinary
/// logic net (compare `EdgeBoard.Net-(IC16-INA)`, the receiver's *output*, which
/// is an ordinary driven net).
///
/// This is deliberately the only behavioral motion assertion here — enough to
/// show the components form an axis rather than sitting side by side. The plant
/// itself (step rates, lag, load) belongs to the consuming repo, and
/// `embsim-models`' own suite covers the Gray-code sequencing.
#[rstest]
fn moving_the_shaft_changes_the_encoder_nets_the_board_reads() {
    ensure_clock();
    let _guard = machine_parts::lock_module_instance();
    serial::init(1);

    // A separate assembly so the shaft handle survives into the test.
    let encoder = QuadratureEncoder::new(quadrature_encoder::Config::new(80.0)).expect("config");
    let input = encoder.input();
    let phases: Arc<Mutex<Vec<i64>>> = Arc::new(Mutex::new(Vec::new()));
    {
        let log = Arc::clone(&phases);
        input.on_count_change(move |count| log.lock().unwrap().push(count));
    }

    let system = System::new()
        .board(MODULE, ec32mb_board())
        .board(EDGE, edge_board())
        .harness(module_socket_harness(MODULE, EDGE))
        .harness(bench_rails(EDGE))
        .harness(embsim_board::Harness::new().connect(
            machine_parts::ep("ENC.A"),
            machine_parts::ep("EdgeBoard.J20.1"),
        ))
        .scenario(encoder_jumpers_closed(
            edge_polarity_fet_conducting(
                module_polarity_fet_conducting(Scenario::default(), MODULE),
                EDGE,
            ),
            EDGE,
        ))
        .component("ENC", Box::new(encoder))
        .start()
        .expect("the axis starts");

    const A_PLUS: &str = "EdgeBoard./MaD_Edge_Sheet3/A+";
    let settled_at = |volts: f64| {
        wait_for(
            || live_volts(&system, A_PLUS).is_some_and(|v| (v - volts).abs() < 1e-3),
            Duration::from_secs(5),
        )
    };

    // Count 0 is A low; one count of the ×4 decode takes it high.
    assert!(
        settled_at(0.0),
        "the encoder drives its count-0 phase onto the board's A+ net; got {:?}",
        system.net_state(A_PLUS)
    );
    input.set_position_counts(1);
    assert!(
        settled_at(3.3),
        "one count moves the board's A+ net to the encoder's high rail; got {:?}",
        system.net_state(A_PLUS)
    );
    assert_eq!(*phases.lock().unwrap().last().expect("a count change"), 1);

    // The complementary leg stays at the isolated ground through the closed
    // A_GND jumper, so the receiver's differential is the full swing.
    assert_eq!(
        live_volts(&system, "EdgeBoard./MaD_Edge_Sheet3/A-"),
        Some(0.0),
        "JP2 holds A- at the isolated ground"
    );

    drop(system);
    serial::reset();
}

// ============================================================
// Census
// ============================================================

/// The assembled system's scale, so a fixture swap that silently drops a board
/// cannot pass the tests above.
#[rstest]
fn the_assembled_system_has_the_expected_scale() {
    let system = build_machine();

    // Nets: 100 (module) + 243 (EdgeBoard) + 25 (DS2) + the machine
    // components' own pins + the bench/harness externals.
    assert!(
        system.nets().len() >= 100 + 243 + 25,
        "every board's nets must be present; got {}",
        system.nets().len()
    );

    // The machine components' pins are addressable as nets in their own right.
    let names: HashSet<&str> = system.nets().iter().map(|n| n.name.as_str()).collect();
    for pin in [
        "MOTOR.STEP",
        "MOTOR.DIR",
        "MOTOR.ENA",
        "ENC.A",
        "ENC.B",
        "END_U.COM",
        "END_U.NO",
        "END_L.COM",
        "END_L.NO",
    ] {
        assert!(names.contains(pin), "{pin} must be a net");
    }
}
