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
//! `Board::from_netlist` validates every registered component's pin
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

use embsim_board::{jesd8c01_lvcmos_thresholds, netlist, Board, DeadBand, PartRegistry};
use embsim_models::spi_flash::{SpiNorFlash, JEDEC_ID_W25Q128JV_IM};
use embsim_models::spi_flash_component::SpiNorFlashComponent;
use machine_parts::ec32mb_registry;

/// 128 M-bit = 16 MiB, the density the netlist's value field states.
const W25Q128_CAPACITY: usize = 16 * 1024 * 1024;

/// The netlist's `value` for `U301`, which is the registry key.
const FLASH_PART: &str = "SPI Flash 16MB (128Mb)";

/// The module registry with this binary's own `U301` in place of the blank
/// part the registry ships.
fn registry_with_live_flash() -> PartRegistry {
    let mut registry = ec32mb_registry();
    // Re-registering the same key replaces the registry's part.
    registry.register(FLASH_PART, |_decl| {
        Box::new(SpiNorFlashComponent::new(SpiNorFlash::blank(
            W25Q128_CAPACITY,
        )))
    });
    registry
}

#[test]
fn the_part_mounts_on_a_real_netlist() {
    let parsed =
        netlist::parse(include_str!("fixtures/p2_ec32mb.net")).expect("the EC32MB fixture parses");
    let board = Board::from_netlist(parsed, &registry_with_live_flash())
        .expect("the live flash's pin facade matches U301 in both directions");

    let registered: BTreeSet<&str> = board.component_refs().collect();
    assert!(
        registered.contains("U301"),
        "the boot flash is a registered component of the module"
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

    let error = Board::from_netlist(parsed, &registry)
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
    digital_drive, level_of, ComponentNetIo, Harness, Level, NetState, PinHandle, System,
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
        Self {
            pins: [
                embsim_board::PinDecl::digital_out("1").with_name("CS"),
                embsim_board::PinDecl::digital_out("2").with_name("CLK"),
                embsim_board::PinDecl::digital_out("3").with_name("DI"),
                embsim_board::PinDecl::digital_in(
                    "4",
                    jesd8c01_lvcmos_thresholds(DeadBand::Unknown),
                )
                .with_name("DO"),
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
    level_of(pin.net_report()) == Some(Level::High)
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

/// Assert or release chip select, which is active low.
fn select(pins: &MasterPins, selected: bool) {
    drive_and_settle(
        pins.cs.as_ref().unwrap(),
        if selected { Level::Low } else { Level::High },
    );
}

/// One complete transaction: select, shift bytes out, deselect.
fn transact(pins: &MasterPins, bytes: &[u8]) {
    select(pins, true);
    for &b in bytes {
        send(pins, b);
    }
    select(pins, false);
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
    let harness = powered(Harness::new())
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
    let harness = powered(Harness::new())
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

/// The flash's supply from the bench: `VCC` at 3.3 V against `VSS` held at
/// 0 V. The part's inputs read through 0.3/0.7 × `VCC` against `VSS`, and
/// neither is implicit (`DESIGN.md` rule 6): a flash with no supply reads no
/// level, as one on the bench would.
fn powered(harness: Harness) -> Harness {
    let ep = |endpoint: &str| embsim_board::EndpointRef::parse(endpoint).expect("endpoint parses");
    harness.power(ep("BENCH.GND"), ep("FLASH.VSS"), 0.0).power(
        ep("BENCH.VCC"),
        ep("FLASH.VCC"),
        3.3,
    )
}

/// Bring up a master opposite a flash on a bench system, and hand back the
/// master's pins. The system is returned too: dropping it stops the engine.
fn bench(flash: SpiNorFlash) -> (embsim_board::SystemHandle, Arc<Mutex<MasterPins>>) {
    virtual_clock::init(50.0, 1_000_000);
    let handles = Arc::new(Mutex::new(MasterPins::default()));
    let harness = powered(Harness::new())
        .connect_str("MASTER.CS", "FLASH.CSn")
        .expect("endpoints parse")
        .connect_str("MASTER.CLK", "FLASH.CLK")
        .expect("endpoints parse")
        .connect_str("MASTER.DI", "FLASH.DI_IO0")
        .expect("endpoints parse")
        .connect_str("MASTER.DO", "FLASH.DO_IO1")
        .expect("endpoints parse");
    let system = System::new()
        .component("MASTER", Box::new(BitBangMaster::new(Arc::clone(&handles))))
        .component(
            "FLASH",
            Box::new(SpiNorFlashComponent::new(flash).with_pins(&SPI_FLASH_PINS_BY_FUNCTION)),
        )
        .harness(harness)
        .start()
        .expect("the bench system starts");
    assert!(wait_for(
        || handles.lock().expect("master pins").clk.is_some(),
        Duration::from_secs(5)
    ));
    (system, handles)
}

#[test]
fn an_image_flashed_into_the_part_reads_back_over_the_net() {
    // The shape a bootloader cares about: an image sits in the part, and the
    // first bytes of it come back from a read at a 24-bit address.
    let mut image = vec![0xFF; 4096];
    image[0x100..0x104].copy_from_slice(&[0xDE, 0xAD, 0xBE, 0xEF]);
    let (_system, handles) = bench(SpiNorFlash::with_image(image));
    let pins = handles.lock().expect("master pins");
    drive_and_settle(pins.clk.as_ref().unwrap(), Level::Low);
    select(&pins, false);

    select(&pins, true);
    for byte in [0x03, 0x00, 0x01, 0x00] {
        send(&pins, byte);
    }
    let got = [recv(&pins), recv(&pins), recv(&pins), recv(&pins)];
    select(&pins, false);

    assert_eq!(
        got,
        [0xDE, 0xAD, 0xBE, 0xEF],
        "a read at $000100 streams the image from that offset"
    );
}

#[test]
fn a_program_lands_in_the_part_and_reads_back_over_the_net() {
    // The write path, end to end and entirely over nets: enable, program,
    // deselect to commit, then read it back the way a verifier would.
    let (_system, handles) = bench(SpiNorFlash::blank(4096));
    let pins = handles.lock().expect("master pins");
    drive_and_settle(pins.clk.as_ref().unwrap(), Level::Low);
    select(&pins, false);

    transact(&pins, &[0x06]); // Write Enable (§8.2.1, p.24)
    transact(&pins, &[0x02, 0x00, 0x00, 0x08, 0x11, 0x22]);

    select(&pins, true);
    for byte in [0x03, 0x00, 0x00, 0x08] {
        send(&pins, byte);
    }
    let got = [recv(&pins), recv(&pins)];
    select(&pins, false);

    assert_eq!(
        got,
        [0x11, 0x22],
        "programmed bytes are readable through the same four nets"
    );
}

#[test]
fn a_program_without_write_enable_changes_nothing_over_the_net() {
    // The latch is the part's own protection, and it has to survive the trip
    // through the engine intact: an unlatched program is discarded here
    // exactly as it is on silicon (§7.1.2, p.13).
    let (_system, handles) = bench(SpiNorFlash::blank(4096));
    let pins = handles.lock().expect("master pins");
    drive_and_settle(pins.clk.as_ref().unwrap(), Level::Low);
    select(&pins, false);

    transact(&pins, &[0x02, 0x00, 0x00, 0x08, 0xAA]); // no $06 first

    select(&pins, true);
    for byte in [0x03, 0x00, 0x00, 0x08] {
        send(&pins, byte);
    }
    let got = recv(&pins);
    select(&pins, false);

    assert_eq!(
        got, 0xFF,
        "the byte was never written, so the cell is erased"
    );
}

// ============================================================
// The sequence a Propeller 2 boot ROM actually issues
// ============================================================
//
// Replayed frame for frame from Parallax's `ROM_Booter_v33k.spin2` — `try_spi`
// at lines 239-269 and the framing helpers `spi_cmd`/`spi_in` at 343-364, as
// vendored in the consumer tree this model was ported from. It is not an
// invented sequence and not a paraphrase of one.
//
// No CPU is involved: the ROM's frames are replayed by the bench master. That
// proves the PART answers what a boot ROM asks, which is a different claim
// from "the ROM boots" — running the ROM needs a P2 core, and there is none
// in this workspace.

/// `spi_cmd`: raise CS, lower it, then shift `bits` bits of `value` out MSB
/// first, MSB-justified into 32 bits — so a byte command is `(byte, 8)` and
/// the ROM's read frame is `($03000000, 32)`, opcode and 24-bit address in one
/// transaction.
fn spi_cmd(pins: &MasterPins, value: u32, bits: u32) {
    let (cs, clk, di) = (
        pins.cs.as_ref().unwrap(),
        pins.clk.as_ref().unwrap(),
        pins.di.as_ref().unwrap(),
    );
    drive_and_settle(cs, Level::High);
    drive_and_settle(cs, Level::Low);
    let justified = if bits == 8 { value << 24 } else { value };
    for i in 0..bits {
        let level = if (justified >> (31 - i)) & 1 != 0 {
            Level::High
        } else {
            Level::Low
        };
        drive_and_settle(di, level);
        drive_and_settle(clk, Level::High);
        drive_and_settle(clk, Level::Low);
    }
}

/// `spi_in`: eight clocks, sampling after each pulse — and crucially WITHOUT
/// touching chip select, so the byte continues the transaction the preceding
/// `spi_cmd` opened. The ROM's own comment notes it samples "from before
/// `drvh`", the beat-late input this model's bit presentation is built for.
fn spi_in(pins: &MasterPins) -> u8 {
    let (clk, dout) = (pins.clk.as_ref().unwrap(), pins.dout.as_ref().unwrap());
    let mut byte = 0u8;
    for _ in 0..8 {
        drive_and_settle(clk, Level::High);
        drive_and_settle(clk, Level::Low);
        byte = (byte << 1) | u8::from(sense_bit(dout));
    }
    byte
}

/// Everything `try_spi` does before it decides a device is there: the three
/// all-ones bursts that exit quad and dual mode, reset-enable, reset,
/// write-disable, then read-status. Returns the status byte the ROM gates on.
fn rom_probe(pins: &MasterPins) -> u8 {
    drive_and_settle(pins.clk.as_ref().unwrap(), Level::Low);
    drive_and_settle(pins.cs.as_ref().unwrap(), Level::High);

    // `neg pb,#1` then callpa #2 / #8 / #16 — all ones, to leave quad/dual.
    for bits in [2u32, 8, 16] {
        spi_cmd(pins, u32::MAX, bits);
    }
    spi_cmd(pins, 0x66, 8); // reset-enable
    spi_cmd(pins, 0x99, 8); // reset
    spi_cmd(pins, 0x04, 8); // write-disable, "to clear WEL"
    spi_cmd(pins, 0x05, 8); // read-status
    spi_in(pins)
}

#[test]
fn the_part_passes_the_boot_roms_presence_check() {
    let (_system, handles) = bench(SpiNorFlash::blank(4096));
    let pins = handles.lock().expect("master pins");

    let status = rom_probe(&pins);

    // `testbn x,#1 wz` / `if_nz jmp #.fail` — WEL high means NO SPI MEMORY to
    // the ROM. This is the bit that decides whether a board boots from flash
    // at all, and it is why $04 is issued first.
    assert_eq!(status & 0b10, 0, "WEL clear, so the ROM sees a device");
    // `testbn x,#0 wz` / `if_nz jmp #.wait` — BUSY high means poll again.
    assert_eq!(status & 0b01, 0, "BUSY clear, so the ROM stops polling");
}

#[test]
fn a_part_left_write_enabled_reads_as_absent_to_the_boot_rom() {
    // The same probe with the write-disable omitted, after something has set
    // the latch. The ROM would take the WEL bit for "no SPI memory" and fall
    // through to its next boot source — a failure that looks like missing
    // hardware rather than a protocol error.
    let (_system, handles) = bench(SpiNorFlash::blank(4096));
    let pins = handles.lock().expect("master pins");
    drive_and_settle(pins.clk.as_ref().unwrap(), Level::Low);
    drive_and_settle(pins.cs.as_ref().unwrap(), Level::High);

    spi_cmd(&pins, 0x06, 8); // write-enable: sets WEL
    spi_cmd(&pins, 0x05, 8);
    let status = spi_in(&pins);

    assert_eq!(
        status & 0b10,
        0b10,
        "WEL is set, which the ROM reads as absent"
    );
}

#[test]
fn the_boot_roms_read_frame_streams_the_image_from_zero() {
    // `mov pa,#32` / `callpb #$03,#spi_cmd` is ONE 32-bit frame: the opcode
    // and a 24-bit address of zero. The ROM then holds CS low and clocks out
    // $400 bytes; this reads the first eight, which is the framing claim —
    // the volume is the ROM's business, not the part's.
    let mut image = vec![0xFF; 4096];
    image[..8].copy_from_slice(b"Prop");
    let (_system, handles) = bench(SpiNorFlash::with_image(image));
    let pins = handles.lock().expect("master pins");

    let status = rom_probe(&pins);
    assert_eq!(status & 0b11, 0, "the probe passed before the read");

    spi_cmd(&pins, 0x0300_0000, 32);
    let mut got = [0u8; 8];
    for byte in got.iter_mut() {
        *byte = spi_in(&pins);
    }
    drive_and_settle(pins.cs.as_ref().unwrap(), Level::High);

    assert_eq!(
        &got, b"Prop\x01\x02\x03\x04",
        "opcode and 24-bit address in one frame, then a stream with CS held low"
    );
}

/// A deselected part's data-out is at high impedance (W25Q128JV §4.1 "Chip
/// Select (/CS)", p.9): on a bench with no pull-up the line floats until the
/// master selects the part, carries the part's bit while it is selected, and
/// floats again when the master deselects it.
#[test]
fn the_data_line_floats_while_the_part_is_deselected() {
    let (_system, handles) = bench(SpiNorFlash::with_image(vec![0xA5; 64]));
    let pins = handles.lock().expect("master pins");
    let dout = pins.dout.as_ref().unwrap();

    // The master idles ~CS high, so the part comes up deselected.
    std::thread::sleep(Duration::from_millis(5));
    assert_eq!(
        dout.net_report(),
        NetState::Floating,
        "with ~CS high the part drives nothing onto DO"
    );

    drive_and_settle(pins.clk.as_ref().unwrap(), Level::Low);
    drive_and_settle(pins.cs.as_ref().unwrap(), Level::Low);
    assert!(
        matches!(dout.net_report(), NetState::Driven(_)),
        "selected, the part presents its bit on DO: {:?}",
        dout.net_report()
    );

    drive_and_settle(pins.cs.as_ref().unwrap(), Level::High);
    assert_eq!(
        dout.net_report(),
        NetState::Floating,
        "deselected again, DO is released"
    );
}
