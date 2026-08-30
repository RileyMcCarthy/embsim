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

use embsim_board::netlist::parse;
use embsim_board::{
    Board, Component, Finding, Level, NetState, PinRef, Scenario, SenseKind, System,
};
use machine_parts::{
    ec32mb_board, ec32mb_registry, edge_fingers, ep, module_polarity_fet_conducting, P2EdgeModule,
    EC32MB_STUB_REFS,
};

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
/// that the reference-designator fallback plus a ten-entry value-keyed registry
/// is enough to build a 114-component vendor board — and, because
/// `Board::from_netlist_with_stubs` validates every registered component's
/// facade against the netlist in both directions, that all 22 hand-written pin
/// tables match the transcription exactly.
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

    let board = Board::from_netlist_with_stubs(parsed, &ec32mb_registry(), &EC32MB_STUB_REFS)
        .expect("the module builds with no unclassified-part errors");

    // The 22 registered components: the P2, two inverters, the TCXO, the DIP
    // switch, the flash, four PSRAMs, the polarity FET, two bucks, the
    // brownout detector and eight LDOs. Everything else is an auto-classified
    // primitive, a boundary, or one of the two BOM-only reference designators.
    let registered: BTreeSet<&str> = board.component_refs().collect();
    assert_eq!(
        registered,
        BTreeSet::from([
            "U100", "U101", "U301", "U302", "U303", "U304", "U305", "U401", "U402", "U403", "U404",
            "U501", "U502", "U503", "U504", "U505", "U506", "U507", "U508", "U601", "X100", "S301",
        ]),
        "exactly the module's active silicon is registered"
    );
}

/// Without the fallback the same netlist is entirely unclassifiable — the
/// counterfactual that makes the opt-in worth its surface.
#[rstest]
fn without_the_reference_fallback_the_module_does_not_build() {
    let mut registry = ec32mb_registry();
    registry.classify_unnamed_by_reference(false);
    let parsed = parse(EC32MB).expect("fixture parses");
    let error = Board::from_netlist_with_stubs(parsed, &registry, &EC32MB_STUB_REFS)
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

    // The bridged force-gauge channel, as `P2EdgeModule` declares it.
    let p2 = P2EdgeModule::new("p2");
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
    assert!(
        matches!(
            rx.stream,
            Some(embsim_board::StreamRole::Consumer { baud_hz: 115_200 })
        ),
        "P0 is the force-gauge RX pin at the table baud; got {:?}",
        rx.stream
    );
    assert!(
        matches!(
            tx.stream,
            Some(embsim_board::StreamRole::Producer { baud_hz: 115_200 })
        ),
        "P2 is the force-gauge TX pin at the table baud; got {:?}",
        tx.stream
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
/// 5 V into the two `5V` fingers, 0 V into the three `GND` fingers, and the
/// reverse-polarity FET conducting. Everything else — bucks, inductors,
/// eight LDOs, sixteen bank rails — comes from the netlist.
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
        .scenario(module_polarity_fet_conducting(
            Scenario::default(),
            "EC32MB",
        ))
        .build()
        .expect("the powered module resolves")
}

/// The whole power tree resolves from two fingers: no supply pin anywhere on
/// the module reports an unsourced rail, and nothing contends.
///
/// That is a real chain — 5 V fingers → polarity FET → two bucks → their
/// output inductors → `Common_VDD` and `Common_LDOin` → eight LDOs → sixteen
/// P2 bank-supply pins — and every hop of it is a netlist component the
/// registry classified, not a hand-written wire.
#[rstest]
fn the_powered_module_sources_every_rail_without_contention() {
    let system = powered_module();
    let findings = system.diagnostics().findings();

    let unsourced: Vec<&Finding> = findings
        .iter()
        .filter(|f| matches!(f, Finding::PowerNetUnsourced { .. }))
        .collect();
    assert!(
        unsourced.is_empty(),
        "every rail must be sourced through the module's own power tree; got {unsourced:?}"
    );
    let contention: Vec<&Finding> = findings
        .iter()
        .filter(|f| matches!(f, Finding::Contention { .. }))
        .collect();
    assert!(
        contention.is_empty(),
        "the module has exactly one driver (the P2's bridged TX pin); got {contention:?}"
    );
}

/// The P2's reset pin is held high by the module's own 10.5 kΩ pull-up — not
/// floating, and not driven by anything. Cutting the pull-up out (a lifted
/// `R100` pad) makes the reset node float and the P2's `RESN` sense report it:
/// the failure mode a carrier sees as "the module never boots".
#[rstest]
fn the_reset_node_is_pulled_up_and_floats_without_the_pull_up() {
    let system = powered_module();
    let reset = system
        .nets()
        .iter()
        .find(|n| n.name == "EC32MB.P2_RESN")
        .expect("the reset net exists");
    assert!(
        matches!(reset.state, NetState::Pulled(Level::High, _)),
        "the 10.5 kΩ pull-up must hold reset released; got {:?}",
        reset.state
    );
    assert!(
        !system.diagnostics().contains(&Finding::FloatingSense {
            net: "EC32MB.P2_RESN".to_string(),
            kind: SenseKind::Digital,
        }),
        "a pulled-up reset must not report floating"
    );

    // Lift one pad of the pull-up.
    let harness = embsim_board::Harness::new()
        .power(ep("CARRIER.5V"), ep("EC32MB.J203.41"), 5.0)
        .power(ep("CARRIER.GND"), ep("EC32MB.J203.43"), 0.0);
    let _guard = machine_parts::lock_module_instance();
    let broken = System::new()
        .board("EC32MB", ec32mb_board())
        .harness(harness)
        .scenario(
            module_polarity_fet_conducting(Scenario::default(), "EC32MB")
                .pin_detach("EC32MB.R100.1"),
        )
        .build()
        .expect("the module still builds with a lifted pad");
    assert!(
        broken.diagnostics().contains(&Finding::FloatingSense {
            net: "EC32MB.P2_RESN".to_string(),
            kind: SenseKind::Digital,
        }),
        "with R100 lifted the reset node must float; got {:?}",
        broken.diagnostics().findings()
    );
}

/// Two honest floating reports the module *should* produce, and one the DIP
/// switch explains.
///
/// - `XTAL_XI` / `XTAL_XO`: the emulator models no oscillator, so the TCXO and
///   its inverter buffer are stubs and the P2's crystal pins see nothing. On
///   real silicon `XI` is driven by the TCXO chain and `XO` is unused (the
///   vendor's own note), so `XTAL_XO` is a one-pin net by design.
/// - `P2_IO59`: the guide's P59 pull-up/pull-down is *selected by the DIP
///   switch*, and with every gang open neither resistor conducts — so the boot
///   strap floats. Turning a gang on is a switch-position scenario the engine
///   does not model yet, and this assertion is what will change when it does.
#[rstest]
fn unmodeled_oscillator_and_open_dip_switch_float_their_nets() {
    let system = powered_module();
    let floating = |net: &str| {
        system.diagnostics().contains(&Finding::FloatingSense {
            net: net.to_string(),
            kind: SenseKind::Digital,
        })
    };
    assert!(floating("EC32MB.XTAL_XI"), "the oscillator chain is a stub");
    assert!(floating("EC32MB.XTAL_XO"), "XO is unused on this module");
    assert!(
        floating("EC32MB.P2_IO59"),
        "with every DIP gang open the P59 strap floats; got {:?}",
        system.diagnostics().findings()
    );
}

/// The P2's bridged transmit pin is the module's only driver, and it idles high
/// — a UART line at rest — while the debug-serial pins next to it sit at their
/// pull-ups. Nothing else on 114 components drives a net, which is the point of
/// the stub pin-kind rule.
#[rstest]
fn the_bridged_tx_pin_is_the_only_driver() {
    let system = powered_module();
    let tx = system
        .nets()
        .iter()
        .find(|n| n.name == "EC32MB.P2_IO2")
        .expect("the P2 TX net exists");
    assert_eq!(
        tx.state,
        NetState::Driven(Level::High),
        "the bridged UART TX idles high"
    );

    let driven = system
        .nets()
        .iter()
        .filter(|n| matches!(n.state, NetState::Driven(_)))
        .map(|n| n.name.as_str())
        .collect::<Vec<_>>();
    assert_eq!(
        driven,
        vec!["EC32MB.P2_IO2"],
        "exactly one net on the module is push-pull driven"
    );

    // The debug-serial pins are pulled, not driven — their 100 kΩ resistors.
    for net in ["EC32MB.P2_IO62_TXD", "EC32MB.P2_IO63_RXD"] {
        let state = system
            .nets()
            .iter()
            .find(|n| n.name == net)
            .map(|n| n.state)
            .expect("net exists");
        assert!(
            matches!(state, NetState::Pulled(Level::High, _)),
            "{net} must sit at its pull-up; got {state:?}"
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
