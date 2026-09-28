//! The Parallax P2-EC32MB module as a [`Board`] — a whole vendor module
//! ingested from its netlist, with the P2 itself as an [`McuComponent`].
//!
//! # What this binary is for
//!
//! Three things the other fixtures cannot exercise:
//!
//! 1. **A netlist with no `libsource`.** The module's netlist was transcribed
//!    from the vendor's schematic PDF, so no component names a symbol library.
//!    Classification runs on `PartRegistry::classify_unnamed_by_reference`
//!    (reference-designator prefixes) with the registry keyed on the `value`
//!    field. The acceptance bar is blunt: **114 components, zero
//!    unclassified-part errors.**
//! 2. **The MCU as a component, at package scale.** `U100`'s 86 netlist pins
//!    are validated against the facade in both directions, and the P2's 64 I/O
//!    pins are reachable by name.
//! 3. **A card-edge boundary.** `J203`'s 80 fingers are the module's entire
//!    consumer-facing surface, and which finger carries what is the thing a
//!    consumer gets wrong. Every claim below is checked against the Rev B
//!    product guide's account of the pin map.
//!
//! # Sources
//!
//! - `fixtures/p2_ec32mb.net` — the module netlist (provenance in its header:
//!   Parallax P2-EC32MB Rev B schematic, 29 Mar 2022, CC BY-SA 4.0).
//! - Parallax "P2 Edge Module with 32MB RAM" Rev B product guide, for the pin
//!   map claims quoted at each assertion: P0-P39 free, P40-P57 consumed by the
//!   module's PSRAM (P56 = CLK, P57 = ~CE), P58-P61 shared with the boot flash
//!   and microSD socket, P62/P63 the programming/debug serial port, P38/P39
//!   the buffered on-module LEDs, and a 10 K pull-up on RESn.

mod machine_parts;

use std::collections::{BTreeSet, HashMap, HashSet};

use rstest::rstest;
use vibes_behaviour::{behaviour, expect, Test};

use embsim_board::netlist::parse;
use embsim_board::{
    Board, Component, Finding, NetState, PartClass, PinRef, RailDownReason, SenseKind, System,
};
use machine_parts::{ec32mb_board, ec32mb_registry, edge_fingers, ep, p2_edge_module};

const EC32MB: &str = include_str!("fixtures/p2_ec32mb.net");

/// Component count of the transcribed module netlist.
const EXPECTED_COMPONENTS: usize = 114;
/// Net count.
const EXPECTED_NETS: usize = 100;
/// Total `(node …)` membership entries.
const EXPECTED_NODES: usize = 453;

// ============================================================
// Helpers
// ============================================================

/// Map every pin to the (board-local) net name that owns it.
fn net_of_pin(board: &Board) -> HashMap<PinRef, String> {
    let mut map = HashMap::new();
    for net in board.nets() {
        for node in &net.nodes {
            map.insert(node.clone(), net.name.clone());
        }
    }
    map
}

/// The net owning `reference.pin`, or a panic naming what was missing.
fn net_named(map: &HashMap<PinRef, String>, reference: &str, pin: &str) -> String {
    map.get(&PinRef::new(reference, pin))
        .cloned()
        .unwrap_or_else(|| panic!("{reference}.{pin} is not on any net"))
}

/// References owning a pin on the named net.
fn members<'a>(board: &'a Board, net: &str) -> Vec<&'a PinRef> {
    board
        .nets()
        .iter()
        .find(|n| n.name == net)
        .unwrap_or_else(|| panic!("net {net:?} exists"))
        .nodes
        .iter()
        .collect()
}

// ============================================================
// Classification: the acceptance bar
// ============================================================

/// The whole module classifies. With no libsource anywhere, this is the test
/// that the reference-designator fallback plus a value-keyed registry is
/// enough to build a 114-component vendor board with every part a node — and,
/// because `Board::from_netlist` validates every registered component's
/// facade and every switch's poles against the netlist in both directions,
/// that all the hand-written pin tables match the transcription exactly.
#[rstest]
fn the_module_builds_with_no_unclassified_parts() {
    let parsed = parse(EC32MB).expect("the module fixture parses");
    assert_eq!(parsed.version, "E");
    assert_eq!(parsed.components.len(), EXPECTED_COMPONENTS);
    assert_eq!(parsed.nets.len(), EXPECTED_NETS);
    assert_eq!(
        parsed.nets.iter().map(|n| n.nodes.len()).sum::<usize>(),
        EXPECTED_NODES
    );
    // Not one component names a symbol library — the condition the fallback
    // exists for.
    assert!(
        parsed.components.iter().all(|c| c.part.is_empty()),
        "the transcribed netlist carries no libsource part names"
    );

    let board = Board::from_netlist(parsed, &ec32mb_registry())
        .expect("the module builds with no unclassified-part errors");

    // The 20 registered components: the P2, two inverters, the TCXO, the
    // flash, four PSRAMs, two bucks, the brownout detector and eight LDOs
    // — every one a model. The polarity FET `U401` and the
    // white LEDs `D601`/`D602` are elements by specification, not
    // components. Everything else is an auto-classified primitive, a
    // boundary, a switch or a mechanical node.
    let registered: BTreeSet<&str> = board.component_refs().collect();
    assert_eq!(
        registered,
        BTreeSet::from([
            "U100", "U101", "U301", "U302", "U303", "U304", "U305", "U402", "U403", "U404", "U501",
            "U502", "U503", "U504", "U505", "U506", "U507", "U508", "U601", "X100",
        ]),
        "exactly the module's active silicon is registered"
    );
    for element in ["U401", "D601", "D602"] {
        assert!(
            matches!(board.node_class(element), Some(PartClass::Pwl { .. })),
            "{element} is an element by specification: {:?}",
            board.node_class(element)
        );
    }
    // Every one of the 114 parts is a node.
    assert_eq!(board.nodes().count(), EXPECTED_COMPONENTS);
    assert!(
        matches!(board.node_class("S301"), Some(PartClass::Switch { poles }) if poles.len() == 4)
    );
    assert!(
        matches!(board.node_class("J101"), Some(PartClass::Switch { poles }) if poles.len() == 1)
    );
    for mechanical in ["J701", "J702", "PCB", "NC_Net"] {
        assert_eq!(
            board.node_class(mechanical),
            Some(&PartClass::Mechanical),
            "{mechanical}"
        );
    }
}

/// Without the fallback the same netlist is entirely unclassifiable — the
/// counterfactual that makes the opt-in worth its surface.
#[rstest]
fn without_the_reference_fallback_the_module_does_not_build() {
    let mut registry = ec32mb_registry();
    registry.classify_unnamed_by_reference(false);
    let parsed = parse(EC32MB).expect("fixture parses");
    let error = Board::from_netlist(parsed, &registry)
        .expect_err("no part names and no fallback cannot classify a resistor");
    assert!(
        error.to_string().contains("classification"),
        "expected a classification failure, got {error}"
    );
}

// ============================================================
// The P2's own pins
// ============================================================

/// Every one of the P2's 64 I/O pins is on a net, and the two the force-gauge
/// UART bridges carry their stream roles from the HAL table. The facade's
/// non-I/O pins (core supply, sixteen bank supplies, RESN/TEST and the crystal
/// pair) are all present too — the build would have refused otherwise, so this
/// pins the *count* that makes "86 pins" a fact rather than a coincidence.
#[rstest]
fn every_p2_io_pin_is_reachable() {
    let board = ec32mb_board();
    let map = net_of_pin(&board);

    for pin in 0..=63u32 {
        let name = format!("P{pin}");
        assert!(
            map.contains_key(&PinRef::new("U100", &name)),
            "U100.{name} must be on a net"
        );
    }
    for pin in [
        "VDD",
        "GND",
        "TEST",
        "RESN",
        "XI",
        "XO",
        "VIO_0_3",
        "VIO_60_63",
    ] {
        assert!(
            map.contains_key(&PinRef::new("U100", pin)),
            "U100.{pin} must be on a net"
        );
    }
    assert_eq!(
        map.keys().filter(|p| p.reference == "U100").count(),
        86,
        "the P2 package facade is 64 I/O + VDD + GND + TEST + RESN + XI + XO + 16 bank supplies"
    );

    // The bridged force-gauge channel sits on two of the package's pads,
    // which the package declares like every other pad: bidirectional and
    // released until the core drives one.
    let p2 = p2_edge_module("p2");
    let rx = p2
        .pins()
        .iter()
        .find(|p| p.number == "P0")
        .expect("P0 declared");
    let tx = p2
        .pins()
        .iter()
        .find(|p| p.number == "P2")
        .expect("P2 declared");
    // The channel carries levels, so neither pin declares a byte route: the
    // framing lives in the MCU component, and what is on the net is edges.
    assert!(rx.reads_when_subscribed());
    assert_eq!(rx.idle, None);
    assert!(tx.reads_when_subscribed());
    assert_eq!(
        tx.idle, None,
        "P2 is the force-gauge TX pin: the bridge drives it from the START instant, the \
         package declares it released like every pad"
    );
}

// ============================================================
// The J203 card edge, against the product guide
// ============================================================

/// The whole finger map in one table: which J203 finger sits on which net, and
/// which P2 pin (if any) that net reaches. Every row is a claim the Rev B
/// product guide also makes.
#[rstest]
// P0..P37 descend from finger 40 to finger 3 — the guide's "P0-P39 free" block.
#[case::p0(40, "P2_IO0", Some("P0"))]
#[case::p1(39, "P2_IO1", Some("P1"))]
#[case::p16(24, "P2_IO16", Some("P16"))]
#[case::p31(9, "P2_IO31", Some("P31"))]
#[case::p37(3, "P2_IO37", Some("P37"))]
// P38/P39 wrap onto the module's back edge and also drive the on-module LEDs.
#[case::p38(80, "P2_IO38", Some("P38"))]
#[case::p39(79, "P2_IO39", Some("P39"))]
// P58..P61: shared with the boot flash and the microSD socket.
#[case::p58(54, "P2_IO58", Some("P58"))]
#[case::p59(53, "P2_IO59", Some("P59"))]
#[case::p60(52, "P2_IO60", Some("P60"))]
#[case::p61(51, "P2_IO61", Some("P61"))]
// P62/P63: the programming/debug serial port.
#[case::p62(50, "P2_IO62_TXD", Some("P62"))]
#[case::p63(49, "P2_IO63_RXD", Some("P63"))]
// Reset arrives through a 1 kΩ series resistor, so finger 46 is one hop away
// from the P2's RESN pin (see `reset_chain_matches_the_guide`).
#[case::resn(46, "P2_RESN_PROTECTED", None)]
// Supplies: 5 V in, per-bank I/O rails out, grounds.
#[case::vin_a(41, "VIN_Edge", None)]
#[case::vin_b(42, "VIN_Edge", None)]
#[case::gnd(43, "GND", Some("GND"))]
#[case::v00(47, "VIO_00_07", Some("VIO_0_3"))]
#[case::v08(48, "VIO_08_15", Some("VIO_8_11"))]
#[case::v16(58, "VIO_16_23", Some("VIO_16_19"))]
#[case::v24(68, "VIO_24_31", Some("VIO_24_27"))]
#[case::v32(78, "VIO_32_39", Some("VIO_32_35"))]
#[case::v56(57, "VIO_56_63", Some("VIO_56_59"))]
fn edge_fingers_land_on_the_nets_the_guide_names(
    #[case] finger: u32,
    #[case] net: &str,
    #[case] p2_pin: Option<&str>,
) {
    let board = ec32mb_board();
    let map = net_of_pin(&board);
    assert_eq!(
        net_named(&map, "J203", &finger.to_string()),
        net,
        "finger {finger}"
    );
    if let Some(pin) = p2_pin {
        assert_eq!(
            net_named(&map, "U100", pin),
            net,
            "finger {finger} must share its net with U100.{pin}"
        );
    }
}

/// The guide's "P40-P57 are used by the module's PSRAM" is a statement about
/// the *card edge*: those P2 pins exist, they are wired to the four PSRAMs, and
/// the corresponding fingers carry nothing at all. Both halves are asserted —
/// the pins are busy, and the fingers are absent from the graph — because a
/// consumer that assumed finger 76 was "P40, free" would find a socket pin
/// wired to nothing rather than a conflict.
#[rstest]
fn psram_owned_pins_are_not_available_on_the_edge() {
    let board = ec32mb_board();
    let map = net_of_pin(&board);

    // Every J203 finger the netlist declares a node for.
    let declared: HashSet<u32> = board
        .nets()
        .iter()
        .flat_map(|n| n.nodes.iter())
        .filter(|node| node.reference == "J203")
        .filter_map(|node| node.pin.parse::<u32>().ok())
        .collect();

    // The 20 fingers with no node at all: the P40..P57 signals (P56/P57 are
    // the PSRAM CLK/CE) and the V40/V48 bank supplies the module consumes
    // internally. The vendor's two NC pads are NOT in this set — they are
    // declared on `NC_Net`, asserted just below.
    let mut absent: Vec<u32> = (1..=80).filter(|f| !declared.contains(f)).collect();
    absent.sort_unstable();
    assert_eq!(
        absent,
        vec![55, 56, 59, 60, 61, 62, 63, 64, 65, 66, 67, 69, 70, 71, 72, 73, 74, 75, 76, 77],
        "the module's no-connect fingers"
    );
    // Fingers 1 and 2 ARE declared — on the vendor's `NC_Net` layout node, not
    // on a signal.
    assert_eq!(net_named(&map, "J203", "1"), "NC_Net");
    assert_eq!(net_named(&map, "J203", "2"), "NC_Net");

    // And the pins themselves are busy: each P40..P57 net holds PSRAM pins and
    // no J203 finger.
    for pin in 40..=57u32 {
        let net = net_named(&map, "U100", &format!("P{pin}"));
        let holders: HashSet<&str> = members(&board, &net)
            .iter()
            .map(|p| p.reference.as_str())
            .collect();
        assert!(
            holders.iter().any(|r| r.starts_with("U30")),
            "P{pin} must reach a PSRAM (U302..U305); net {net} holds {holders:?}"
        );
        assert!(
            !holders.contains("J203"),
            "P{pin} must NOT reach the card edge; net {net} holds {holders:?}"
        );
    }

    // The guide names two of them specifically: P56 is the shared PSRAM clock
    // and P57 the shared chip-enable, so those two nets reach all four parts.
    for (pin, expected) in [("P56", 4), ("P57", 4)] {
        let net = net_named(&map, "U100", pin);
        let psrams = members(&board, &net)
            .iter()
            .filter(|p| p.reference.starts_with("U30") && p.reference != "U301")
            .count();
        assert_eq!(psrams, expected, "{pin} is shared across all four PSRAMs");
    }
}

/// P58..P61 are shared, not free: the guide gives them to the boot flash and
/// the microSD socket, and the fingers are brought out anyway so a carrier can
/// use them when the module is not booting from flash.
#[rstest]
fn boot_flash_and_microsd_share_p58_through_p61() {
    let board = ec32mb_board();
    let map = net_of_pin(&board);
    for (pin, finger, sharer) in [
        ("P58", "54", "U301"), // flash DO(IO1), also through R304 to the SD DAT0
        ("P59", "53", "U301"), // flash DI(IO0) + SD CMD/MOSI
        ("P60", "52", "U301"), // flash CLK + SD CD/DAT3/CS
        ("P61", "51", "J301"), // SD CLK (flash CS goes via the DIP switch)
    ] {
        let net = net_named(&map, "U100", pin);
        assert_eq!(net_named(&map, "J203", finger), net);
        let holders: HashSet<&str> = members(&board, &net)
            .iter()
            .map(|p| p.reference.as_str())
            .collect();
        assert!(
            holders.contains(sharer),
            "{pin} must be shared with {sharer}; net {net} holds {holders:?}"
        );
    }
}

/// The debug serial port: fingers 50/49 are P62/P63, each with a 100 kΩ
/// pull-up to the P56-P63 bank rail so the port idles defined with nothing
/// plugged in.
#[rstest]
fn debug_serial_fingers_carry_p62_and_p63_with_pull_ups() {
    let board = ec32mb_board();
    let map = net_of_pin(&board);

    for (finger, pin, resistor) in [("50", "P62", "R305"), ("49", "P63", "R306")] {
        let net = net_named(&map, "J203", finger);
        assert_eq!(net_named(&map, "U100", pin), net);
        let holders: HashSet<&str> = members(&board, &net)
            .iter()
            .map(|p| p.reference.as_str())
            .collect();
        assert!(
            holders.contains(resistor),
            "the {pin} finger must carry its pull-up {resistor}; net {net} holds {holders:?}"
        );
        // The pull-up's other end is the P56-P63 bank rail.
        let other = if net_named(&map, resistor, "1") == net {
            "2"
        } else {
            "1"
        };
        assert_eq!(net_named(&map, resistor, other), "VIO_56_63");
    }
}

/// The reset chain, exactly as the netlist's own provenance note describes it:
/// finger 46 → `R201` (1 kΩ) → `P2_RESN` ← `R100` (10.5 kΩ) → `VIO_56_63`,
/// with the brownout detector also on `P2_RESN`.
///
/// The product guide says "10 K pull-up on RESn", which is true of the P2's
/// reset *node* — but the finger is one series resistor away from it, and the
/// fitted pull-up is 10.5 kΩ. Both details matter to anyone reasoning about a
/// carrier's own reset circuit, which is why the chain is asserted hop by hop
/// rather than as "finger 46 is pulled up".
#[rstest]
fn reset_chain_matches_the_guide() {
    let board = ec32mb_board();
    let map = net_of_pin(&board);

    // Hop 1: the finger and the series resistor.
    assert_eq!(net_named(&map, "J203", "46"), "P2_RESN_PROTECTED");
    assert_eq!(net_named(&map, "R201", "1"), "P2_RESN_PROTECTED");
    // Hop 2: the resistor's far side is the P2's reset node, shared with the
    // pull-up and the brownout detector.
    assert_eq!(net_named(&map, "R201", "2"), "P2_RESN");
    assert_eq!(net_named(&map, "U100", "RESN"), "P2_RESN");
    assert_eq!(net_named(&map, "R100", "1"), "P2_RESN");
    assert_eq!(net_named(&map, "U404", "OUT"), "P2_RESN");
    // Hop 3: the pull-up's far side is the P56-P63 bank rail.
    assert_eq!(net_named(&map, "R100", "2"), "VIO_56_63");

    // And the values are the fitted ones, parsed by the auto tier from the
    // reference-designator classification.
    let parsed = parse(EC32MB).unwrap();
    let value = |reference: &str| {
        parsed
            .components
            .iter()
            .find(|c| c.reference == reference)
            .map(|c| c.value.clone())
            .unwrap_or_default()
    };
    assert_eq!(value("R201"), "1K");
    assert_eq!(value("R100"), "10.5K");
}

/// P38/P39 drive the on-module LEDs *as well as* their fingers — the guide's
/// "buffered LEDs" note. Anything a carrier does with those two pins is
/// visible on the module.
#[rstest]
fn p38_and_p39_also_reach_the_on_module_led_buffer() {
    let board = ec32mb_board();
    let map = net_of_pin(&board);
    for (pin, buffer_pin) in [("P38", "1A"), ("P39", "2A")] {
        let net = net_named(&map, "U100", pin);
        assert_eq!(net_named(&map, "U601", buffer_pin), net);
    }
}

// ============================================================
// The module as a system: power tree and reset, resolved
// ============================================================

/// Build the module alone as a system, powered the way a carrier powers it:
/// 5 V into the two `5V` fingers and 0 V into the three `GND` fingers.
/// Everything else — the reverse-polarity FET, bucks, inductors, eight
/// LDOs, sixteen bank rails — comes from the netlist: the FET is an
/// element whose channel the solve turns on (its gate is on the carrier's
/// ground, 5 V below its source), so nothing in the scenario says it
/// conducts.
fn powered_module() -> embsim_board::BuiltSystem {
    let harness = embsim_board::Harness::new()
        .power(ep("CARRIER.5V"), ep("EC32MB.J203.41"), 5.0)
        .power(ep("CARRIER.5Vb"), ep("EC32MB.J203.42"), 5.0)
        .power(ep("CARRIER.GND"), ep("EC32MB.J203.43"), 0.0)
        .power(ep("CARRIER.GNDb"), ep("EC32MB.J203.44"), 0.0)
        .power(ep("CARRIER.GNDc"), ep("EC32MB.J203.45"), 0.0);
    let _guard = machine_parts::lock_module_instance();
    System::new()
        .board("EC32MB", ec32mb_board())
        .harness(harness)
        .build()
        .expect("the powered module resolves")
}

/// The polarity FET passes the carrier's 5 V to the protected input on its
/// own: with the drain on the 5 V fingers and the gate on the carrier's
/// ground, the gate sits 5 V below the source once the body diode has
/// lifted it, the channel turns on, and the protected rail reads the
/// input less nothing (no load draws through the 36 mΩ channel: a rail
/// model senses its input and loads it with nothing, `NODES.md` §2). No
/// `pin_short` stands in for it.
#[rstest]
fn the_polarity_fet_passes_the_carrier_input_to_the_protected_rail() {
    behaviour!(Test {
        id: "ec32mb.polarity-fet-passes-the-input",
        covers: Some("board/src/cluster.rs#QuasiStaticMna::solve"),
        given: "the P2-EC32MB module built with 5 volts on its 5V edge fingers and 0 volts on \
                its GND fingers, and no scenario line about its reverse-polarity FET",
    });
    expect!(
        "protected-rail-at-the-input",
        "the protected input rail behind the FET reads the carrier's 5 volts",
        "the FET's gate is on the carrier's ground, 5 volts below the source the body diode \
         lifts, so the channel conducts and only its on-resistance stands between the two"
    );
    expect!(
        "no-current-without-a-load",
        "the FET carries no current",
        "the bucks behind the rail sense their input and load it with nothing: a rail's \
         input current is not modelled"
    );
    let system = powered_module();
    let protected = system.nets()[system.net_id("EC32MB.VIN_Edge_Protected").unwrap().0].state;
    assert!(
        matches!(protected, NetState::Analog(v) if (v - 5.0).abs() < 1e-6),
        "{protected:?}"
    );
    let channel = system
        .pin_current("EC32MB.U401.S")
        .expect("the FET's cluster solved");
    assert!(channel.abs() < 1e-9, "{channel}");
}

/// The whole power tree hangs off two fingers — 5 V fingers → polarity FET
/// → two bucks → their output inductors → `Common_VDD` and `Common_LDOin`
/// → eight LDOs → sixteen P2 bank-supply pins — and every hop of it is a
/// netlist component the registry classified, not a hand-written wire.
/// Nothing contends. But the tree has a clock: the bucks are AP62301s with
/// a 2.5 ms soft-start, and a build snapshot is the state before the first
/// wake, so in it every module rail is down and says why
/// ([`Finding::RailDown`]) — the bucks holding their outputs with their
/// input up (the soft-start), the LDOs with no input (the buck's down
/// rail) — and every load on them is an unsourced power net.
/// `power_tree.rs` steps the module past the soft-start and reads every
/// rail at its setpoint.
#[rstest]
fn the_powered_module_holds_its_rails_down_before_the_soft_start_without_contention() {
    behaviour!(Test {
        id: "ec32mb.power-tree-before-the-soft-start",
        covers: Some("board/src/system.rs#lint_build"),
        given: "the P2-EC32MB module analyzed at build with 5 volts on its 5V edge fingers \
                and 0 volts on its GND fingers",
    });
    expect!(
        "no-contention",
        "no two parts fight over any net",
        "the module has one driver, the P2's bridged transmit pin, and every rail is one \
         declared terminal"
    );
    expect!(
        "bucks-in-soft-start",
        "both bucks report their output down with their input sourced — held by the part",
        "the polarity FET passes the carrier's 5 volts to their input, and their 2.5 \
         millisecond soft-start has not elapsed before the first wake"
    );
    expect!(
        "ldos-without-input",
        "each of the eight LDOs reports its output down for want of its input",
        "the LDOs' input is the second buck's output, down until its soft-start elapses"
    );
    expect!(
        "loads-unsourced",
        "the unsourced power nets are exactly the two buck rails and the eight bank rails",
        "every load on the module hangs off a rail that has not risen"
    );
    let system = powered_module();
    let findings = system.diagnostics().findings();

    let contention: Vec<&Finding> = findings
        .iter()
        .filter(|f| matches!(f, Finding::Contention { .. }))
        .collect();
    assert!(
        contention.is_empty(),
        "the module has exactly one driver (the P2's bridged TX pin); got {contention:?}"
    );
    for buck in ["U402", "U403"] {
        assert!(
            system.diagnostics().contains(&Finding::RailDown {
                part: format!("EC32MB.{buck}"),
                pin: "SW".to_string(),
                reason: RailDownReason::HeldDown,
            }),
            "{buck}: {findings:?}"
        );
    }
    for ldo in 501..=508 {
        assert!(
            system.diagnostics().contains(&Finding::RailDown {
                part: format!("EC32MB.U{ldo}"),
                pin: "OUT".to_string(),
                reason: RailDownReason::InputUnsourced {
                    pin: "IN".to_string()
                },
            }),
            "U{ldo}: {findings:?}"
        );
    }
    let unsourced: BTreeSet<String> = findings
        .iter()
        .filter_map(|f| match f {
            Finding::PowerNetUnsourced { net } => Some(net.clone()),
            _ => None,
        })
        .collect();
    assert_eq!(
        unsourced,
        BTreeSet::from([
            "EC32MB.Common_VDD".to_string(),
            "EC32MB.Common_LDOin".to_string(),
            "EC32MB.VIO_00_07".to_string(),
            "EC32MB.VIO_08_15".to_string(),
            "EC32MB.VIO_16_23".to_string(),
            "EC32MB.VIO_24_31".to_string(),
            "EC32MB.VIO_32_39".to_string(),
            "EC32MB.VIO_40_47".to_string(),
            "EC32MB.VIO_48_55".to_string(),
            "EC32MB.VIO_56_63".to_string(),
        ])
    );
}

/// Before the rails rise the P2's reset node floats: its 10.5 kΩ pull-up
/// `R100` returns to `VIO_56_63`, the LDO `U508`'s output, and the brownout
/// detector `U404` that would hold it low runs from `Common_VDD` — both
/// down in the build snapshot (the bucks' soft-start), and a detector with
/// no supply drives nothing (the STM1061 guarantees its output only from
/// 0.7 V). So the P2's `RESN` sense reports the float. The pulled-up node
/// and the lifted-pad failure are live claims — `power_tree.rs`.
#[rstest]
fn the_reset_node_floats_before_the_rails_rise() {
    behaviour!(Test {
        id: "ec32mb.reset-node-before-the-rails",
        covers: Some("board/src/system.rs#System::build"),
        given: "the P2-EC32MB module analyzed at build with 5 volts on its 5V edge fingers \
                and 0 volts on its GND fingers",
    });
    expect!(
        "reset-floats",
        "the P2's reset node floats and its reset input reports the float",
        "the pull-up's rail and the brownout detector's supply are both bank rails the \
         bucks have not raised before the first wake"
    );
    let system = powered_module();
    let reset = system
        .nets()
        .iter()
        .find(|n| n.name == "EC32MB.P2_RESN")
        .expect("the reset net exists");
    assert_eq!(reset.state, NetState::Floating, "{:?}", reset.state);
    assert!(
        system.diagnostics().contains(&Finding::FloatingSense {
            net: "EC32MB.P2_RESN".to_string(),
            kind: SenseKind::Digital,
        }),
        "the P2's RESN sense reports the float; got {:?}",
        system.diagnostics().findings()
    );
}

/// Three honest floating nets the build analysis *should* produce: two
/// the clock chain explains, one the DIP switch does.
///
/// - `XTAL_XI`: the TCXO publishes its 20 MHz as a rate one millisecond
///   after its supply comes up — a scheduled wake, which the build analysis
///   (the state before any wake) has not reached — so at build the buffer
///   has nothing to relay and the P2's `XI` net floats, and since `XI` is
///   a sense the package declares, the float is a reported finding. Live,
///   once time runs, `XI` carries the rate: `oscillator_chain.rs` proves it.
/// - `XTAL_XO`: unused (the vendor's own note), a one-pin net by design;
///   the package's `XO` is the crystal driver, a released output, so the
///   net floats and nothing senses it.
/// - `P2_IO59`: the guide's P59 pull-up/pull-down is *selected by the DIP
///   switch*, and with every pole open (as shipped) neither resistor reaches
///   the pin — so the boot strap floats. A pad is a released bidirectional
///   pin, so the float is the net's state, read by the core when it
///   samples the strap. Closing a pole is
///   `Scenario::switch("EC32MB.S301", pole, JumperState::Closed)`
///   (`one_pipeline.rs` closes each and reads the strap it selects).
#[rstest]
fn the_clock_chain_rests_before_start_up_and_the_open_dip_switch_floats_its_strap() {
    let system = powered_module();
    let state = |net: &str| {
        system
            .nets()
            .iter()
            .find(|n| n.name == net)
            .map(|n| n.state)
            .unwrap_or_else(|| panic!("{net} exists"))
    };
    assert!(
        system.diagnostics().contains(&Finding::FloatingSense {
            net: "EC32MB.XTAL_XI".to_string(),
            kind: SenseKind::Digital,
        }),
        "before the TCXO's start-up instant the buffer relays nothing"
    );
    assert_eq!(
        state("EC32MB.XTAL_XO"),
        NetState::Floating,
        "XO is unused on this module"
    );
    assert_eq!(
        state("EC32MB.P2_IO59"),
        NetState::Floating,
        "with every DIP gang open the P59 strap floats; got {:?}",
        system.diagnostics().findings()
    );
}

/// Every driver on the module is a registered model, and before the rails
/// rise none drives: the P2 has not started — its package runs the core
/// only the datasheet's restart delay after its reset releases, and a chip
/// in reset floats every pad — so its bridged transmit pad presents
/// nothing, and nothing else on 114 components drives a net in the
/// snapshot: the live parts rest released (the boot flash's data-out is at
/// high impedance while its `~CS` sits at the pull-up, W25Q128JV §4.1; the
/// gates have no level to answer; the PSRAMs are deselected), the brownout
/// detector's open-drain output is released with its supply down (the
/// STM1061 guarantees nothing under 0.7 V), and the rails are terminals,
/// not drivers. The debug-serial pins beside the transmit pin float here:
/// their 100 kΩ pull-ups return to `VIO_56_63`, an LDO output the bucks'
/// soft-start has not raised — `power_tree.rs` reads them at the pull-ups
/// once it has.
#[rstest]
fn every_driver_is_a_registered_model() {
    behaviour!(Test {
        id: "ec32mb.every-driver-a-model",
        covers: Some("board/src/system.rs#System::build"),
        given: "the P2-EC32MB module analyzed at build with 5 volts on its 5V edge fingers \
                and 0 volts on its GND fingers",
    });
    expect!(
        "no-net-driven",
        "no net on the module is push-pull driven, and the P2's transmit line floats",
        "every output rests released before the first wake: the P2 not yet started, the \
         flash deselected, the gates without a level, the detector without a supply"
    );
    expect!(
        "debug-serial-floats",
        "the two debug-serial pins float",
        "their pull-ups return to a bank rail the bucks' soft-start has not raised"
    );
    let system = powered_module();
    let tx = system
        .nets()
        .iter()
        .find(|n| n.name == "EC32MB.P2_IO2")
        .expect("the P2 TX net exists");
    assert_eq!(
        tx.state,
        NetState::Floating,
        "the bridged UART TX floats until the P2 starts"
    );

    let driven = system
        .nets()
        .iter()
        .filter(|n| matches!(n.state, NetState::Driven(_)))
        .map(|n| n.name.as_str())
        .collect::<Vec<_>>();
    assert_eq!(
        driven,
        Vec::<&str>::new(),
        "no net on the module is push-pull driven"
    );

    for net in ["EC32MB.P2_IO62_TXD", "EC32MB.P2_IO63_RXD"] {
        let state = system
            .nets()
            .iter()
            .find(|n| n.name == net)
            .map(|n| n.state)
            .expect("net exists");
        assert_eq!(
            state,
            NetState::Floating,
            "{net} floats until its pull-up rail rises"
        );
    }
}

// ============================================================
// The card edge as a harness surface
// ============================================================

/// Every finger the harness builder claims is real, and it claims all of them
/// bar the module's no-connects — so a system description can wire the module
/// into a carrier without a translation table (see
/// `machine_parts::module_socket_harness`).
#[rstest]
fn the_harness_finger_list_covers_every_declared_finger() {
    let board = ec32mb_board();
    let declared: BTreeSet<u32> = board
        .nets()
        .iter()
        .flat_map(|n| n.nodes.iter())
        .filter(|node| node.reference == "J203")
        .filter_map(|node| node.pin.parse::<u32>().ok())
        .collect();
    let harnessed: BTreeSet<u32> = edge_fingers().collect();

    assert_eq!(harnessed.len(), 58);
    assert!(
        harnessed.is_subset(&declared),
        "the harness must only claim fingers the netlist declares"
    );
    // The two the harness deliberately leaves out are the vendor NC pads.
    let skipped: Vec<u32> = declared.difference(&harnessed).copied().collect();
    assert_eq!(skipped, vec![1, 2]);
}

/// A drive applied to the P2's TX pin reaches the card edge — the module is
/// transparent between silicon and finger, which is what makes it usable as a
/// board in a bigger system.
#[rstest]
fn a_p2_drive_reaches_its_edge_finger() {
    let _guard = machine_parts::lock_module_instance();
    let harness = embsim_board::Harness::new()
        .power(ep("CARRIER.GND"), ep("EC32MB.J203.43"), 0.0)
        // A carrier-side pull-down on the P0 finger: proof the finger and the
        // P2 pin are one node, from the other direction.
        .power(ep("CARRIER.PULL"), ep("EC32MB.J203.40"), 0.0);
    let system = System::new()
        .board("EC32MB", ec32mb_board())
        .harness(harness)
        .build()
        .expect("builds");

    let p0 = system
        .nets()
        .iter()
        .find(|n| n.name == "EC32MB.P2_IO0")
        .expect("the P0 net exists");
    assert_eq!(
        p0.state,
        NetState::Analog(0.0),
        "a carrier source on finger 40 lands on the P2's P0 pin"
    );
}

// ============================================================
// Fixture provenance
// ============================================================

/// The committed fixture keeps the vendor's provenance header **verbatim** —
/// including the CC BY-SA attribution the schematic's license requires and the
/// note that this file is a static artifact, never regenerated by a tool. The
/// embsim-side note is appended below it, and neither block changes what the
/// parser sees (`;` comments are skipped wherever whitespace is legal).
#[rstest]
fn the_fixture_keeps_its_vendor_provenance_and_license() {
    assert!(
        EC32MB.starts_with("; ====="),
        "the vendor banner must stay first"
    );
    for claim in [
        "P2-EC32MB-RevB-SCHEMATIC.pdf",
        "Copyright 2022 Parallax Incorporated",
        "Creative Commons Attribution-ShareAlike 4.0 International",
        "STATIC ARTIFACT",
        "embsim-board fixture note (appended; the vendor provenance above is verbatim)",
    ] {
        assert!(EC32MB.contains(claim), "the header must state {claim:?}");
    }

    // Stripping every comment line leaves an identical parse.
    let stripped: String = EC32MB
        .lines()
        .filter(|line| !line.trim_start().starts_with(';'))
        .collect::<Vec<_>>()
        .join("\n");
    assert_eq!(parse(&stripped).unwrap(), parse(EC32MB).unwrap());
}
