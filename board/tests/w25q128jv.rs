//! The Winbond W25Q128JV as a LIVE board component.
//!
//! [`SpiNorFlashComponent`] is generic, so what it needs proving against is a
//! real part on a real netlist. This binary uses the one netlist in the tree
//! that carries a serial NOR flash — the Parallax P2-EC32MB module, whose
//! `U301` is a `W25Q128JVSIM` — and swaps that stub for the live component.
//! The board is the fixture here; the part is the subject.
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
fn the_part_mounts_on_a_real_netlist_in_place_of_its_stub() {
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
fn the_default_jedec_id_matches_the_ordering_option_on_the_fixture() {
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

// ============================================================
// Driven over real nets
// ============================================================
//
// The tests above prove the facade mounts. This one proves the PART answers
// when its pins are nets rather than method calls — which is a different
// question, because the engine never resolves a drive inline. A master that
// drives the clock and reads the data line without letting the engine run in
// between reads a stale bit, and the bytes come back shifted.
//
// That is the constraint every bit-banged device on a net inherits, and the
// one a QEMU-hosted CPU will have to satisfy at each pin edge.

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use embsim_board::{
    digital_drive, level_of, ComponentNetIo, Harness, Level, PinHandle, PinKind, System,
};
use embsim_core::virtual_clock;
use embsim_models::spi_flash_component::SPI_FLASH_PINS_BY_FUNCTION;

/// The four lines a SPI master owns, captured when the engine wires them.
#[derive(Default)]
struct MasterPins {
    cs: Option<PinHandle>,
    clk: Option<PinHandle>,
    di: Option<PinHandle>,
    dout: Option<PinHandle>,
}

/// A bare bit-banging SPI master: four pins and no behaviour of its own, so
/// the test thread can drive the bus a level at a time and observe exactly
/// what a firmware loop would.
struct BitBangMaster {
    pins: [embsim_board::PinDecl; 4],
    handles: Arc<Mutex<MasterPins>>,
}

impl BitBangMaster {
    fn new(handles: Arc<Mutex<MasterPins>>) -> Self {
        let decl = |number, name, kind| embsim_board::PinDecl {
            number,
            name: Some(name),
            kind,
            stream: None,
            drive_impedance: None,
        };
        Self {
            pins: [
                decl("1", "CS", PinKind::DigitalOut),
                decl("2", "CLK", PinKind::DigitalOut),
                decl("3", "DI", PinKind::DigitalOut),
                decl("4", "DO", PinKind::DigitalIn),
            ],
            handles,
        }
    }
}

impl embsim_board::Component for BitBangMaster {
    fn pins(&self) -> &[embsim_board::PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), embsim_board::AttachError> {
        let mut slot = self.handles.lock().expect("master pins");
        slot.cs = Some(io.pin("CS")?);
        slot.clk = Some(io.pin("CLK")?);
        slot.di = Some(io.pin("DI")?);
        slot.dout = Some(io.pin("DO")?);
        Ok(())
    }
}

/// Drive a level and then LET THE ENGINE RUN. Every edge goes through here:
/// the drive is enqueued, the engine resolves it and delivers the flash's
/// sense callback, and the flash's answering drive is resolved in turn. Skip
/// the wait and the next sense reads what was there before.
fn drive_and_settle(pin: &PinHandle, level: Level) {
    pin.set_drive(Some(digital_drive(level)));
    std::thread::sleep(Duration::from_millis(2));
}

fn sense_bit(pin: &PinHandle) -> bool {
    // A pull-up would decide a released line; here the flash drives DO
    // whenever it is selected, so a missing level means the handshake failed.
    level_of(pin.sense()) == Some(Level::High)
}

/// Shift a byte out to the device, MSB first.
fn send(pins: &MasterPins, byte: u8) {
    let (clk, di) = (pins.clk.as_ref().unwrap(), pins.di.as_ref().unwrap());
    for i in (0..8).rev() {
        let level = if (byte >> i) & 1 != 0 {
            Level::High
        } else {
            Level::Low
        };
        drive_and_settle(di, level);
        drive_and_settle(clk, Level::High);
        drive_and_settle(clk, Level::Low);
    }
}

/// Clock a byte in, MSB first: pulse, then sample — the order a bit-banging
/// master uses, and the one the model's bit presentation is built for.
fn recv(pins: &MasterPins) -> u8 {
    let (clk, dout) = (pins.clk.as_ref().unwrap(), pins.dout.as_ref().unwrap());
    let mut byte = 0u8;
    for _ in 0..8 {
        drive_and_settle(clk, Level::High);
        drive_and_settle(clk, Level::Low);
        byte = (byte << 1) | u8::from(sense_bit(dout));
    }
    byte
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

/// The control for the test below: without the yield, the same sequence reads
/// nothing at all.
///
/// This is what makes the positive test mean something. The master enqueues
/// every edge and never lets the engine run, so when it senses DO nothing has
/// been resolved — not even the chip select — and it reads the released line.
/// `FFh FFh FFh` is exactly the signature of a part that is not fitted, which
/// is the failure this costs you: not a corrupt byte, a device that appears
/// absent.
///
/// The assertion is the robust one. Observed is `[FF, FF, FF]` on every run,
/// but a loaded machine might let the engine resolve some prefix of the
/// drives; what cannot happen is the engine keeping up with all 24 unyielded
/// edges, so "not the right answer" is the claim that holds.
#[test]
fn without_a_yield_between_edges_the_part_reads_as_absent() {
    virtual_clock::init(50.0, 1_000_000);

    let handles = Arc::new(Mutex::new(MasterPins::default()));
    let harness = Harness::new()
        .connect_str("MASTER.CS", "FLASH.CSn")
        .expect("endpoints parse")
        .connect_str("MASTER.CLK", "FLASH.CLK")
        .expect("endpoints parse")
        .connect_str("MASTER.DI", "FLASH.DI_IO0")
        .expect("endpoints parse")
        .connect_str("MASTER.DO", "FLASH.DO_IO1")
        .expect("endpoints parse");
    let _system = System::new()
        .component("MASTER", Box::new(BitBangMaster::new(Arc::clone(&handles))))
        .component(
            "FLASH",
            Box::new(
                SpiNorFlashComponent::new(SpiNorFlash::blank(1024))
                    .with_pins(&SPI_FLASH_PINS_BY_FUNCTION),
            ),
        )
        .harness(harness)
        .start()
        .expect("the bench system starts");
    assert!(wait_for(
        || handles.lock().expect("master pins").clk.is_some(),
        Duration::from_secs(5)
    ));

    let pins = handles.lock().expect("master pins");
    // Every drive enqueued, nothing settled.
    let rush = |p: &PinHandle, l: Level| p.set_drive(Some(digital_drive(l)));
    rush(pins.clk.as_ref().unwrap(), Level::Low);
    rush(pins.cs.as_ref().unwrap(), Level::High);
    rush(pins.cs.as_ref().unwrap(), Level::Low);
    for i in (0..8).rev() {
        let level = if (0x9Fu8 >> i) & 1 != 0 {
            Level::High
        } else {
            Level::Low
        };
        rush(pins.di.as_ref().unwrap(), level);
        rush(pins.clk.as_ref().unwrap(), Level::High);
        rush(pins.clk.as_ref().unwrap(), Level::Low);
    }
    let mut id = [0u8; 3];
    for byte in id.iter_mut() {
        for _ in 0..8 {
            rush(pins.clk.as_ref().unwrap(), Level::High);
            rush(pins.clk.as_ref().unwrap(), Level::Low);
            *byte = (*byte << 1) | u8::from(sense_bit(pins.dout.as_ref().unwrap()));
        }
    }

    assert_ne!(
        id,
        [0xEF, 0x70, 0x18],
        "a master that never yields cannot have read the real ID"
    );
}

#[test]
fn the_part_answers_a_jedec_id_read_driven_bit_by_bit_over_nets() {
    virtual_clock::init(50.0, 1_000_000);

    let handles = Arc::new(Mutex::new(MasterPins::default()));
    let harness = Harness::new()
        .connect_str("MASTER.CS", "FLASH.CSn")
        .expect("endpoints parse")
        .connect_str("MASTER.CLK", "FLASH.CLK")
        .expect("endpoints parse")
        .connect_str("MASTER.DI", "FLASH.DI_IO0")
        .expect("endpoints parse")
        .connect_str("MASTER.DO", "FLASH.DO_IO1")
        .expect("endpoints parse");

    let _system = System::new()
        .component("MASTER", Box::new(BitBangMaster::new(Arc::clone(&handles))))
        .component(
            "FLASH",
            Box::new(
                SpiNorFlashComponent::new(SpiNorFlash::blank(1024))
                    .with_pins(&SPI_FLASH_PINS_BY_FUNCTION),
            ),
        )
        .harness(harness)
        .start()
        .expect("the bench system starts");

    assert!(
        wait_for(
            || handles.lock().expect("master pins").clk.is_some(),
            Duration::from_secs(5)
        ),
        "the master's pins are wired at attach"
    );

    let pins = handles.lock().expect("master pins");
    // Idle high, then select (~CS is active low).
    drive_and_settle(pins.clk.as_ref().unwrap(), Level::Low);
    drive_and_settle(pins.cs.as_ref().unwrap(), Level::High);
    drive_and_settle(pins.cs.as_ref().unwrap(), Level::Low);

    send(&pins, 0x9F);
    let id = [recv(&pins), recv(&pins), recv(&pins)];

    drive_and_settle(pins.cs.as_ref().unwrap(), Level::High);

    assert_eq!(
        id,
        [0xEF, 0x70, 0x18],
        "the JEDEC triple survived four nets and an engine round trip per edge"
    );
}
