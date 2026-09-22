//! The P2-EC32MB module's boot flash as a LIVE component, not a stub.
//!
//! `ec32mb_module.rs` builds the whole module with every part but the P2 as a
//! pin-facade stub, which proves the netlist classifies and resolves. This
//! binary swaps one stub — `U301`, the Winbond W25Q128JVSIM — for the real
//! [`SpiNorFlashComponent`] and asserts the module still builds.
//!
//! # What that actually proves
//!
//! `Board::from_netlist_with_stubs` validates every registered component's pin
//! facade against the netlist in BOTH directions, and it matches on
//! `PinDecl::number` verbatim. A component whose facade is right for the part
//! but wrong for the netlist's identifier convention fails here and nowhere
//! else: the model's own unit tests never build a board, so they pass while
//! the component cannot be mounted at all.
//!
//! That is not hypothetical. The facade was first written with the datasheet's
//! pin NUMBERS ("1".."8", §3.3 p.5), which is correct for the part and correct
//! for a KiCad export — and wrong for this netlist, which was transcribed from
//! the vendor schematic PDF and names `U301`'s pins by function (`CSn`,
//! `DO_IO1`, `VSS`, …). This test is the one that says so.
//!
//! # Sources
//!
//! - `fixtures/p2_ec32mb.net` — `(comp (ref "U301"))`, value
//!   `"SPI Flash 16MB (128Mb)"`, MPN `W25Q128JVSIM TR`, manufacturer Winbond.
//! - Winbond W25Q128JV datasheet, Revision F (27 March 2018).

mod machine_parts;

use std::collections::BTreeSet;

use embsim_board::{netlist, Board, PartRegistry};
use embsim_models::spi_flash::{SpiNorFlash, JEDEC_ID_W25Q128JV_IM};
use embsim_models::spi_flash_component::SpiNorFlashComponent;
use machine_parts::{ec32mb_registry, EC32MB_STUB_REFS};

/// 128 M-bit = 16 MiB, the density the netlist's value field states.
const W25Q128_CAPACITY: usize = 16 * 1024 * 1024;

/// The netlist's `value` for `U301`, which is the registry key.
const FLASH_PART: &str = "SPI Flash 16MB (128Mb)";

/// The module registry with `U301` live instead of stubbed.
fn registry_with_live_flash() -> PartRegistry {
    let mut registry = ec32mb_registry();
    // Re-registering the same key replaces the stub.
    registry.register(FLASH_PART, |_decl| {
        Box::new(SpiNorFlashComponent::new(SpiNorFlash::blank(
            W25Q128_CAPACITY,
        )))
    });
    registry
}

#[test]
fn the_module_builds_with_a_live_flash_in_place_of_the_stub() {
    let parsed =
        netlist::parse(include_str!("fixtures/p2_ec32mb.net")).expect("the EC32MB fixture parses");
    let board =
        Board::from_netlist_with_stubs(parsed, &registry_with_live_flash(), &EC32MB_STUB_REFS)
            .expect("the live flash's pin facade matches U301 in both directions");

    let registered: BTreeSet<&str> = board.component_refs().collect();
    assert!(
        registered.contains("U301"),
        "the boot flash is a registered component, not a stub skipped over"
    );
}

#[test]
fn a_facade_keyed_by_pin_number_does_not_mount_on_this_netlist() {
    use embsim_models::spi_flash_component::SPI_FLASH_PINS_SOIC8;

    let parsed =
        netlist::parse(include_str!("fixtures/p2_ec32mb.net")).expect("the EC32MB fixture parses");
    let mut registry = ec32mb_registry();
    registry.register(FLASH_PART, |_decl| {
        Box::new(
            SpiNorFlashComponent::new(SpiNorFlash::blank(W25Q128_CAPACITY))
                .with_pins(&SPI_FLASH_PINS_SOIC8),
        )
    });

    let error = Board::from_netlist_with_stubs(parsed, &registry, &EC32MB_STUB_REFS)
        .expect_err("pin \"1\" is not a pin this netlist has");
    let rendered = format!("{error}");
    assert!(
        rendered.contains("U301"),
        "the error names the part that cannot be mounted: {rendered}"
    );
}

#[test]
fn the_part_is_the_one_the_module_ships() {
    const FIXTURE: &str = include_str!("fixtures/p2_ec32mb.net");
    let parsed = netlist::parse(FIXTURE).expect("the EC32MB fixture parses");
    let u301 = parsed
        .components
        .iter()
        .find(|c| c.reference == "U301")
        .expect("the module has a boot flash");
    assert_eq!(
        u301.value, FLASH_PART,
        "and it is the part the registry keys on"
    );

    // The JEDEC ID is variant-specific and easy to get wrong: W25Q128JV-IM and
    // -IQ differ in the second byte (7018h against 4018h, §8.1.1 p.21), and
    // `SIM` decodes as package S / temperature I / special option M (§11,
    // p.74). A master that identifies the part by its ID would reject one
    // while accepting the other. The MPN is a netlist FIELD, which the parser
    // does not retain, so this reads the fixture text it came from.
    assert!(
        FIXTURE.contains("W25Q128JVSIM"),
        "the netlist still names the -IM part the default ID is chosen for"
    );
    assert_eq!(
        JEDEC_ID_W25Q128JV_IM,
        [0xEF, 0x70, 0x18],
        "manufacturer EFh, device ID 7018h"
    );
}
