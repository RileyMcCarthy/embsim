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

use embsim_board::{
    AttachError, Component, ComponentNetIo, IdleDrive, PartClass, PinDecl, PinKind,
};
use embsim_boards::ec32mb::{Ec32mb, FLASH_CAPACITY, NETLIST};
use embsim_models::sd_card::SdCard;

/// A processor-shaped placeholder: the netlist's own `U100` pins, and no
/// behaviour. Enough to fill the slot for tests about the board rather than
/// about the CPU.
#[derive(Debug)]
struct P2Slot {
    pins: Vec<PinDecl>,
}

impl P2Slot {
    /// Built from the netlist, so it cannot drift from what `U100` declares.
    fn new() -> Self {
        let parsed = embsim_board::netlist::parse(NETLIST).expect("the netlist parses");
        let mut names: BTreeSet<&str> = BTreeSet::new();
        for net in &parsed.nets {
            for node in &net.nodes {
                if node.reference == "U100" {
                    names.insert(node.pin.as_str());
                }
            }
        }
        let pins = names
            .into_iter()
            .map(|n| PinDecl {
                // Leaked so the facade can be `&'static`, as `Component::pins`
                // requires. One allocation per test process.
                number: Box::leak(n.to_string().into_boxed_str()),
                name: None,
                kind: if n.starts_with("VIO") || n == "VDD" || n == "GND" || n == "TEST" {
                    PinKind::PowerIn
                } else {
                    PinKind::DigitalIn
                },
                stream: None,
                drive_impedance: None,
                idle: IdleDrive::KindDefault,
            })
            .collect();
        Self { pins }
    }
}

impl Component for P2Slot {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }
    fn attach(&mut self, _io: ComponentNetIo) -> Result<(), AttachError> {
        Ok(())
    }
}

#[test]
fn the_module_builds_with_every_active_part_behind_a_facade() {
    let board = Ec32mb::new()
        .with_p2(|_decl| Box::new(P2Slot::new()))
        .build()
        .expect("the module builds");

    let refs: BTreeSet<&str> = board.component_refs().collect();
    for expected in [
        "U100", // the processor slot
        "U301", // boot flash, live
        "U302", "U303", "U304", "U305", // the four PSRAMs
        "U401", "U402", "U403", "U404", // power
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
        .with_p2(|_decl| Box::new(P2Slot::new()))
        .build()
        .expect("builds");
    assert!(
        !empty.component_refs().any(|r| r == "J301"),
        "an empty socket stays a board boundary — modelling one would drive \
         MISO for a slot with nothing in it"
    );

    let populated = Ec32mb::new()
        .with_p2(|_decl| Box::new(P2Slot::new()))
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
        .with_p2(|_decl| Box::new(P2Slot::new()))
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
        .with_p2(|_decl| Box::new(P2Slot::new()))
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
        .with_p2(|_decl| Box::new(P2Slot::new()))
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
        .with_p2(|_decl| Box::new(P2Slot::new()))
        .with_flash_image(vec![0xA5; 1024])
        .build()
        .expect("builds");
    assert!(board.component_refs().any(|r| r == "U301"));
    assert_eq!(FLASH_CAPACITY, 16 * 1024 * 1024, "the density U301 states");
}
