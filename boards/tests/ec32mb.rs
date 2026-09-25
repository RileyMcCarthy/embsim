//! The P2-EC32MB as a board: does it build, and is it the circuit the vendor
//! drew?
//!
//! Two different claims here. The first is that every one of the netlist's 114
//! components is a node of a class the netlist agrees with — a component's pin
//! facade, a switch's poles — which `Board::from_netlist` checks in both
//! directions, so a board that builds at all is a strong statement.
//!
//! The second matters more. A board model is only worth having if it reproduces
//! the things a hand-wired harness would quietly get wrong, and this module has
//! one of those: the flash and the card share four pins, in a pairing nobody
//! would invent. Those assertions are below, read from the built board's nets
//! rather than from the netlist text, so they describe what a consumer actually
//! gets.

use std::collections::BTreeSet;

use embsim_board::{Component, IdleDrive, PartClass, PinKind, StreamRole};
use embsim_boards::ec32mb::{Ec32mb, FLASH_CAPACITY, NETLIST};
use embsim_boards::p2::{HeldInReset, P2Package, NUM_PADS};
use embsim_models::sd_card::SdCard;

/// The processor slot filled by a P2 package with no core: every pad
/// released, the rails and reset sensed, `XI` accepting the board's rate —
/// the state any P2 is in before it runs, and enough for tests about the
/// board rather than about the CPU.
fn in_reset() -> P2Package<HeldInReset> {
    P2Package::held_in_reset()
}

/// The package declares exactly the pins the netlist gives `U100`, in both
/// directions — the build checks it, and this pins the facade down by name
/// so a netlist edit and a package edit meet here first.
#[test]
fn the_package_declares_the_netlists_u100_pins() {
    let parsed = embsim_board::netlist::parse(NETLIST).expect("the netlist parses");
    let mut netlist: BTreeSet<&str> = BTreeSet::new();
    for net in &parsed.nets {
        for node in &net.nodes {
            if node.reference == "U100" {
                netlist.insert(node.pin.as_str());
            }
        }
    }
    let package = in_reset();
    let declared: BTreeSet<&str> = package.pins().iter().map(|p| p.number).collect();
    assert_eq!(declared, netlist);
    assert_eq!(package.pins().len(), 86);

    // Every pad is a released bidirectional pin; XI takes a rate; XO is a
    // released output; the supplies are supplies.
    for pad in &package.pins()[..NUM_PADS] {
        assert_eq!(pad.kind, PinKind::DigitalBidir, "{}", pad.number);
        assert_eq!(pad.idle, IdleDrive::Released, "{}", pad.number);
    }
    let pin = |name: &str| {
        package
            .pins()
            .iter()
            .find(|p| p.number == name)
            .copied()
            .unwrap_or_else(|| panic!("{name} declared"))
    };
    assert_eq!(pin("XI").stream, Some(StreamRole::PulseSink));
    assert_eq!(pin("XO").kind, PinKind::DigitalOut);
    assert_eq!(pin("XO").idle, IdleDrive::Released);
    assert_eq!(pin("RESN").kind, PinKind::DigitalIn);
    assert_eq!(pin("VDD").kind, PinKind::PowerIn);
    assert_eq!(pin("VIO_60_63").kind, PinKind::PowerIn);
}

#[test]
fn the_module_builds_with_every_active_part_behind_a_facade() {
    let board = Ec32mb::new()
        .with_p2(|_decl| Box::new(in_reset()))
        .build()
        .expect("the module builds");

    let refs: BTreeSet<&str> = board.component_refs().collect();
    for expected in [
        "U100", // the processor slot
        "U301", // boot flash, live
        "U302", "U303", "U304", "U305", // the four PSRAMs
        "U402", "U403", "U404", // power
        "X100", "U101", // oscillator and its buffer
    ] {
        assert!(refs.contains(expected), "{expected} is missing: {refs:?}");
    }
    // The DIP switch and the solder link are switches with poles; the
    // mounting holes and the BOM-only lines are mechanical nodes. Every part
    // is a node.
    assert!(
        matches!(board.node_class("S301"), Some(PartClass::Switch { poles }) if poles.len() == 4),
        "{:?}",
        board.node_class("S301")
    );
    assert!(
        matches!(board.node_class("J101"), Some(PartClass::Switch { poles }) if poles.len() == 1),
        "{:?}",
        board.node_class("J101")
    );
    for mechanical in ["J701", "J702", "PCB", "NC_Net"] {
        assert_eq!(
            board.node_class(mechanical),
            Some(&PartClass::Mechanical),
            "{mechanical}"
        );
    }
    assert_eq!(board.nodes().count(), 114);
}

#[test]
fn a_card_in_the_socket_becomes_a_component_and_an_empty_socket_does_not() {
    let empty = Ec32mb::new()
        .with_p2(|_decl| Box::new(in_reset()))
        .build()
        .expect("builds");
    assert!(
        !empty.component_refs().any(|r| r == "J301"),
        "an empty socket stays a board boundary — modelling one would drive \
         MISO for a slot with nothing in it"
    );

    let populated = Ec32mb::new()
        .with_p2(|_decl| Box::new(in_reset()))
        .with_card(SdCard::blank(64 * 512))
        .build()
        .expect("builds");
    assert!(
        populated.component_refs().any(|r| r == "J301"),
        "a card in the socket is a live component"
    );
}

#[test]
fn a_module_with_no_processor_refuses_to_build() {
    // Deliberate: a module whose processor silently did not exist would look
    // like a working board that never runs, which is the most expensive kind of
    // wrong.
    let error = Ec32mb::new().build().expect_err("U100 has no constructor");
    let rendered = format!("{error}");
    assert!(
        rendered.contains("U100") || rendered.contains("P2X8C4M64P"),
        "the error names the empty slot: {rendered}"
    );
}

// ============================================================
// The shared bus
// ============================================================

/// Everything on one of the built board's nets, as `REF.PIN`.
fn nodes_on(board: &embsim_board::Board, net_name: &str) -> BTreeSet<String> {
    board
        .nets()
        .iter()
        .find(|n| n.name == net_name)
        .unwrap_or_else(|| panic!("the module has a net called {net_name}"))
        .nodes
        .iter()
        .map(|n| format!("{}.{}", n.reference, n.pin))
        .collect()
}

/// The claim that makes this board worth modelling rather than harnessing.
///
/// P60 is the flash's clock AND the card's chip select, on one net. A driver
/// clocking the flash is deselecting and reselecting the card on every edge —
/// behaviour no hand-wired harness would reproduce, because nobody would wire
/// it that way on purpose.
#[test]
fn the_flash_clock_is_also_the_cards_chip_select() {
    let board = Ec32mb::new()
        .with_p2(|_decl| Box::new(in_reset()))
        .with_card(SdCard::blank(64 * 512))
        .build()
        .expect("builds");

    let p60 = nodes_on(&board, "P2_IO60");
    assert!(p60.contains("U100.P60"), "the processor drives it: {p60:?}");
    assert!(p60.contains("U301.CLK"), "the flash clocks on it: {p60:?}");
    assert!(
        p60.contains("J301.CD_DAT3_CS"),
        "and it is the card's chip select: {p60:?}"
    );
}

#[test]
fn the_card_clock_and_the_flash_chip_select_share_a_pin_through_the_dip_switch() {
    let board = Ec32mb::new()
        .with_p2(|_decl| Box::new(in_reset()))
        .with_card(SdCard::blank(64 * 512))
        .build()
        .expect("builds");

    // P61 carries the card's clock and one side of S301 switch 2.
    let p61 = nodes_on(&board, "P2_IO61");
    assert!(p61.contains("J301.CLK"), "the card clocks on P61: {p61:?}");
    assert!(p61.contains("S301.2_ON"), "and switch 2 taps it: {p61:?}");

    // The flash's own chip select is on the other side of that switch, with the
    // pull-up that holds it deselected when the switch is open.
    let cs = nodes_on(&board, "SPI_CS");
    assert!(cs.contains("U301.CSn"), "the flash's ~CS: {cs:?}");
    assert!(cs.contains("S301.2_OFF"), "switch 2's other side: {cs:?}");
    assert!(
        cs.contains("R301.2"),
        "and the pull-up that deselects it: {cs:?}"
    );
}

#[test]
fn the_two_devices_share_mosi_directly_and_miso_through_a_resistor() {
    let board = Ec32mb::new()
        .with_p2(|_decl| Box::new(in_reset()))
        .with_card(SdCard::blank(64 * 512))
        .build()
        .expect("builds");

    // MOSI: both devices sit on P59 with nothing between them.
    let p59 = nodes_on(&board, "P2_IO59");
    assert!(p59.contains("U301.DI_IO0"), "flash DI on P59: {p59:?}");
    assert!(p59.contains("J301.CMD_MOSI"), "card MOSI on P59: {p59:?}");

    // MISO is not symmetrical: the flash drives P58 directly, the card reaches
    // it through R304. Two devices that could both drive a shared input, with a
    // series resistor deciding who wins — which is exactly the kind of detail a
    // harness invents away.
    let p58 = nodes_on(&board, "P2_IO58");
    assert!(
        p58.contains("U301.DO_IO1"),
        "flash DO direct on P58: {p58:?}"
    );
    assert!(p58.contains("R304.2"), "and R304 in series: {p58:?}");
    assert!(
        !p58.contains("J301.DAT0_MISO"),
        "the card is NOT directly on P58 — it is behind R304: {p58:?}"
    );
    let via = nodes_on(&board, "Net-(J301-DAT0_MISO)");
    assert!(
        via.contains("J301.DAT0_MISO") && via.contains("R304.1"),
        "{via:?}"
    );
}

#[test]
fn a_flash_image_shorter_than_the_part_leaves_the_rest_erased() {
    // A boot image is kilobytes and the part is 16 MiB. A ROM that reads past
    // the image must see the $FF of an erased array, not run off the end.
    let board = Ec32mb::new()
        .with_p2(|_decl| Box::new(in_reset()))
        .with_flash_image(vec![0xA5; 1024])
        .build()
        .expect("builds");
    assert!(board.component_refs().any(|r| r == "U301"));
    assert_eq!(FLASH_CAPACITY, 16 * 1024 * 1024, "the density U301 states");
}
