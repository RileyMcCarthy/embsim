//! The W25Q128JV's data-out on a bus it shares, read at settled instants on
//! the stepped clock.
//!
//! The flash drives DO only while it shifts data, status or an ID out
//! (`SpiNorFlash::drives_do`): through the instruction and the address it is
//! at high impedance, as it is while deselected. On a bench that is a line
//! that floats until the first data bit. On a P2-EC32MB it is what lets the
//! card speak at all. The module's flash `/CS` is the card's clock (`P61`,
//! through `S301` position 2, FLASH) and the flash's clock is the card's
//! chip select (`P60`), so with the switch closed every pulse of the card's
//! clock selects the flash, whose own clock does not move; the flash's DO
//! and the card's MISO are one net (`P58`, the card through `R304`, 240 Ω).
//! A part that drove DO whenever it was selected would hold `P58` through
//! every low half of the card's clock, 25 Ω against the card's 265 Ω, and a
//! master reading the card's bit before the rising edge would read the
//! flash's.
//!
//! # Time
//!
//! Each case runs alone on the stepped clock (`TESTING.md` rules 5 and 9),
//! as `sd_card_spi.rs` does: the suite lock, the clock re-anchored stepped,
//! the system started with time held, the case's thread a virtual-clock
//! actor, and every wait a virtual [`settle`]. Neither the flash nor the
//! card arms an instant — each answers an edge from its sense callback at
//! the edge's own instant — and the module's power tree has settled before
//! the first edge ([`POWER_UP_NS`]), so every read between two settles is
//! the bus at rest. [`Bench::finish`] asserts the engine never stopped
//! waiting for the case.

mod machine_parts;

use std::sync::{Arc, Mutex, MutexGuard};

use embsim_board::{
    digital_drive, jesd8c01_lvcmos_thresholds, level_of, netlist, AttachError, Board, Component,
    ComponentNetIo, DeadBand, EndpointRef, Finding, Harness, JumperState, Level, NetState, PinDecl,
    PinHandle, Scenario, System, SystemHandle,
};
use embsim_boards::ec32mb::{FLASH_SELECT_POLE, FLASH_SELECT_SWITCH, NETLIST};
use embsim_core::virtual_clock::{self, Actor, ClockMode};
use embsim_models::sd_card::{SdCard, BLOCK_LEN};
use embsim_models::sd_card_component::SdCardComponent;
use embsim_models::spi_flash::SpiNorFlash;
use embsim_models::spi_flash_component::{
    FlashView, SpiNorFlashComponent, SPI_FLASH_PINS_BY_FUNCTION,
};
use machine_parts::ec32mb_registry;
use vibes_behaviour::{behaviour, expect, Test};

/// The netlist's `value` for `U301`, the registry key.
const FLASH_PART: &str = "SPI Flash 16MB (128Mb)";
/// The netlist's `value` for `J301`, the registry key.
const SOCKET_PART: &str = "MicroSD Socket";
const MODULE: &str = "EC32MB";

/// The virtual time the master hands the engine after each edge: 1 µs.
/// Nothing on either rig arms an instant after power-up, so a settle only
/// has to let the engine drain the instant. It must not be zero: a zero
/// wait returns without parking.
const SETTLE_NS: u64 = 1_000;
const _: () = assert!(SETTLE_NS > 0);

/// Board time before the module's first edge: past the bucks' 2.5 ms
/// soft-start, when the bank rails reach 3.3 V, and past the package's
/// START instant 3 ms after the reset releases (no core runs in it).
const POWER_UP_NS: u64 = 6_000_000;

static SUITE_LOCK: Mutex<()> = Mutex::new(());

fn suite_lock() -> MutexGuard<'static, ()> {
    SUITE_LOCK.lock().unwrap_or_else(|poisoned| {
        SUITE_LOCK.clear_poison();
        poisoned.into_inner()
    })
}

fn settle() {
    virtual_clock::wait_virtual_ns(SETTLE_NS);
}

fn ep(endpoint: &str) -> EndpointRef {
    EndpointRef::parse(endpoint).expect("endpoint parses")
}

// ============================================================
// The master
// ============================================================

#[derive(Default)]
struct MasterPins {
    cs: Option<PinHandle>,
    clk: Option<PinHandle>,
    di: Option<PinHandle>,
    dout: Option<PinHandle>,
}

/// Four pins and no behaviour of its own: the case's thread drives the bus a
/// level at a time.
struct BitBangMaster {
    pins: [PinDecl; 4],
    handles: Arc<Mutex<MasterPins>>,
}

impl BitBangMaster {
    fn new(handles: Arc<Mutex<MasterPins>>) -> Self {
        Self {
            pins: [
                PinDecl::digital_out("1").with_name("CS"),
                PinDecl::digital_out("2").with_name("CLK"),
                PinDecl::digital_out("3").with_name("DI"),
                PinDecl::digital_in("4", jesd8c01_lvcmos_thresholds(DeadBand::Unknown))
                    .with_name("DO"),
            ],
            handles,
        }
    }
}

impl Component for BitBangMaster {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let mut slot = self.handles.lock().expect("master pins");
        slot.cs = Some(io.pin("CS")?);
        slot.clk = Some(io.pin("CLK")?);
        slot.di = Some(io.pin("DI")?);
        slot.dout = Some(io.pin("DO")?);
        Ok(())
    }
}

/// Drive a level, then let the engine apply it and every answer to it.
fn drive_and_settle(pin: &PinHandle, level: Level) {
    pin.set_drive(Some(digital_drive(level)));
    settle();
    assert_eq!(
        level_of(pin.net_report()),
        Some(level),
        "the engine applied a drive of {level:?}"
    );
}

fn level(bit: bool) -> Level {
    if bit {
        Level::High
    } else {
        Level::Low
    }
}

/// A running rig and the case's hold on it. Field order is drop order: the
/// actor goes first, so a case that panics stops holding the barrier before
/// its system shuts down, and the suite lock last.
struct Bench {
    actor: Actor,
    system: SystemHandle,
    handles: Arc<Mutex<MasterPins>>,
    _suite: MutexGuard<'static, ()>,
}

impl Bench {
    fn start(
        system: System,
        handles: Arc<Mutex<MasterPins>>,
        suite: MutexGuard<'static, ()>,
    ) -> Self {
        let system = system.hold_time().start().expect("the rig starts");
        let actor = virtual_clock::register_actor("shared-spi-bus-case");
        system.release_time();
        settle();
        assert!(
            handles.lock().expect("master pins").clk.is_some(),
            "the master's pins are wired at attach"
        );
        Self {
            actor,
            system,
            handles,
            _suite: suite,
        }
    }

    fn finish(self) {
        let Bench {
            actor,
            system,
            _suite,
            ..
        } = self;
        let stalled: Vec<Finding> = system
            .findings()
            .into_iter()
            .filter(|f| matches!(f, Finding::QuiescenceTimeout { .. }))
            .collect();
        assert!(
            stalled.is_empty(),
            "the engine stopped waiting for the case's thread, so a settled read \
             may have raced the bus: {stalled:?}"
        );
        drop(actor);
        system.shutdown();
    }
}

// ============================================================
// The flash alone
// ============================================================

#[test]
fn a_read_leaves_do_floating_until_its_first_data_bit() {
    behaviour!(Test {
        id: "w25q128jv.do-released-until-data",
        covers: Some("models/src/spi_flash.rs#SpiNorFlash::drives_do"),
        given: "a W25Q128JV flash alone on a bench with no pull-up on its data-out line, \
                selected, sent a Read Data instruction and a 24-bit address, then clocked \
                for one byte",
    });
    expect!(
        "floats-through-instruction",
        "the data-out line floats from the moment the part is selected until the last \
         address bit is in",
        "the part reads data or status out on that line and nothing else"
    );
    expect!(
        "drives-data",
        "from the next clock the line carries the addressed byte, most significant bit first"
    );
    expect!(
        "floats-deselected",
        "the line floats again once the part is deselected"
    );

    let suite = suite_lock();
    virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);
    let mut image = vec![0xFF; 64];
    image[0x20] = 0xA5;
    let handles = Arc::new(Mutex::new(MasterPins::default()));
    let harness = Harness::new()
        .power(ep("BENCH.GND"), ep("FLASH.VSS"), 0.0)
        .power(ep("BENCH.VCC"), ep("FLASH.VCC"), 3.3)
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
            Box::new(
                SpiNorFlashComponent::new(SpiNorFlash::with_image(image))
                    .with_pins(&SPI_FLASH_PINS_BY_FUNCTION),
            ),
        )
        .harness(harness);
    let bench = Bench::start(system, Arc::clone(&handles), suite);
    let pins = bench.handles.lock().expect("master pins");
    let (cs, clk, di, dout) = (
        pins.cs.as_ref().unwrap(),
        pins.clk.as_ref().unwrap(),
        pins.di.as_ref().unwrap(),
        pins.dout.as_ref().unwrap(),
    );

    drive_and_settle(clk, Level::Low);
    drive_and_settle(cs, Level::High);
    drive_and_settle(cs, Level::Low);
    let mut states = vec![dout.net_report()];
    // Read Data at $000020: 32 pulses, the line read after each edge.
    for byte in [0x03u8, 0x00, 0x00, 0x20] {
        for i in (0..8).rev() {
            drive_and_settle(di, level((byte >> i) & 1 != 0));
            drive_and_settle(clk, Level::High);
            states.push(dout.net_report());
            drive_and_settle(clk, Level::Low);
            states.push(dout.net_report());
        }
    }
    assert_eq!(
        states,
        vec![NetState::Floating; 65],
        "selected, DO floats through the instruction and the address"
    );

    // One byte out: pulse, then sample, the order the model presents for.
    let mut got = 0u8;
    for _ in 0..8 {
        drive_and_settle(clk, Level::High);
        drive_and_settle(clk, Level::Low);
        let NetState::Driven(bit) = dout.net_report() else {
            panic!(
                "DO is driven while the part shifts data out: {:?}",
                dout.net_report()
            );
        };
        got = (got << 1) | u8::from(bit == Level::High);
    }
    assert_eq!(got, 0xA5, "the byte at $000020, MSB first");

    drive_and_settle(cs, Level::High);
    assert_eq!(
        dout.net_report(),
        NetState::Floating,
        "deselected: released"
    );
    drop(pins);
    bench.finish();
}

// ============================================================
// The module's shared bus
// ============================================================

/// One byte each way in SPI mode 3, the mode the P2's SD driver uses
/// (flexspin's `sdmm.cc`: the clock idles high): the falling edge, on which
/// the card presents its bit; the master's bit on MOSI; the master's sample
/// of MISO, in the low half of the pulse; the rising edge, on which both
/// sample.
fn exchange_mode3(pins: &MasterPins, mosi: u8) -> u8 {
    let (clk, di, dout) = (
        pins.clk.as_ref().unwrap(),
        pins.di.as_ref().unwrap(),
        pins.dout.as_ref().unwrap(),
    );
    let mut miso = 0u8;
    for i in (0..8).rev() {
        drive_and_settle(clk, Level::Low);
        drive_and_settle(di, level((mosi >> i) & 1 != 0));
        // A released MISO reads as the pull-up the driver sets would.
        let bit = level_of(dout.net_report()).is_none_or(|l| l == Level::High);
        miso = (miso << 1) | u8::from(bit);
        drive_and_settle(clk, Level::High);
    }
    miso
}

/// A command frame, then up to ten bytes polled for the response's first
/// byte (bit 7 clear), then `more` bytes after it: `send_cmd` in `sdmm.cc`.
fn card_command(pins: &MasterPins, cmd: u8, arg: u32, crc: u8, more: usize) -> Vec<u8> {
    let a = arg.to_be_bytes();
    for b in [0x40 | cmd, a[0], a[1], a[2], a[3], crc] {
        exchange_mode3(pins, b);
    }
    let mut reply = Vec::new();
    for _ in 0..10 {
        let b = exchange_mode3(pins, 0xFF);
        if b & 0x80 == 0 {
            reply.push(b);
            break;
        }
    }
    if !reply.is_empty() {
        reply.extend((0..more).map(|_| exchange_mode3(pins, 0xFF)));
    }
    reply
}

#[test]
fn the_card_answers_on_the_bus_it_shares_with_a_selected_flash() {
    behaviour!(Test {
        id: "ec32mb.card-answers-beside-selected-flash",
        covers: Some("models/src/spi_flash.rs#SpiNorFlash::drives_do"),
        given: "a powered P2-EC32MB with its flash select switch closed and a card in its \
                socket, and a master clocking the card on the four shared fingers with the \
                clock idling high",
    });
    expect!(
        "idle-reply",
        "the master reads the card's reply to the reset command: idle",
        "on this module the card's clock is the flash's chip select, so every clock pulse \
         selects the flash while the master reads the card"
    );
    expect!(
        "check-pattern",
        "the master reads the card's reply to the interface-condition command, which \
         echoes the voltage range and the check pattern it was sent"
    );
    expect!(
        "flash-idle",
        "the flash, selected by every pulse and clocked by none, decodes no instruction"
    );
    expect!(
        "card-alone-on-miso",
        "the card is the only part driving the shared data line",
        "the flash's data-out is at high impedance until it has data to shift out"
    );

    let suite = suite_lock();
    virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);

    let flash = SpiNorFlashComponent::new(SpiNorFlash::blank(16 * 1024 * 1024))
        .with_pins(&SPI_FLASH_PINS_BY_FUNCTION);
    let flash_view: FlashView = flash.view();
    let flash_slot = Mutex::new(Some(flash));
    let card = SdCardComponent::new(SdCard::blank(64 * BLOCK_LEN));
    let card_handle = card.card();
    let card_slot = Mutex::new(Some(card));
    let mut registry = ec32mb_registry();
    registry.register(FLASH_PART, move |_decl| {
        Box::new(
            flash_slot
                .lock()
                .unwrap()
                .take()
                .expect("U301 is built once"),
        )
    });
    registry.register(SOCKET_PART, move |_decl| {
        Box::new(
            card_slot
                .lock()
                .unwrap()
                .take()
                .expect("J301 is built once"),
        )
    });
    let board = Board::from_netlist(netlist::parse(NETLIST).expect("parses"), &registry)
        .expect("the module builds with a card and a live flash");

    let handles = Arc::new(Mutex::new(MasterPins::default()));
    let finger = |pin: u32| format!("{MODULE}.J203.{pin}");
    let harness = Harness::new()
        // 5 V and 0 V into the module's fingers (`p2_ec32mb.net`: 41/42
        // and 43/44/45); everything else is the module's own power tree.
        .power(ep("CARRIER.5V"), ep(&finger(41)), 5.0)
        .power(ep("CARRIER.5Vb"), ep(&finger(42)), 5.0)
        .power(ep("CARRIER.GND"), ep(&finger(43)), 0.0)
        .power(ep("CARRIER.GNDb"), ep(&finger(44)), 0.0)
        .power(ep("CARRIER.GNDc"), ep(&finger(45)), 0.0)
        // P61, the card's clock; P60, its chip select; P59, MOSI; P58, MISO.
        .connect_str("MASTER.CLK", &finger(51))
        .expect("endpoints parse")
        .connect_str("MASTER.CS", &finger(52))
        .expect("endpoints parse")
        .connect_str("MASTER.DI", &finger(53))
        .expect("endpoints parse")
        .connect_str("MASTER.DO", &finger(54))
        .expect("endpoints parse");
    let system = System::new()
        .board(MODULE, board)
        .component("MASTER", Box::new(BitBangMaster::new(Arc::clone(&handles))))
        .harness(harness)
        .scenario(Scenario::default().switch(
            &format!("{MODULE}.{FLASH_SELECT_SWITCH}"),
            FLASH_SELECT_POLE,
            JumperState::Closed,
        ));
    let bench = Bench::start(system, Arc::clone(&handles), suite);
    virtual_clock::wait_virtual_ns(POWER_UP_NS);

    let pins = bench.handles.lock().expect("master pins");
    // `disk_initialize`: the card deselected, the clock idling high, then
    // 80 clocks.
    drive_and_settle(pins.cs.as_ref().unwrap(), Level::High);
    drive_and_settle(pins.di.as_ref().unwrap(), Level::High);
    drive_and_settle(pins.clk.as_ref().unwrap(), Level::High);
    for _ in 0..10 {
        exchange_mode3(&pins, 0xFF);
    }
    // `select`: chip select low, one byte of clocks, then the frames.
    drive_and_settle(pins.cs.as_ref().unwrap(), Level::Low);
    exchange_mode3(&pins, 0xFF);
    let idle = card_command(&pins, 0, 0, 0x95, 0);
    let r7 = card_command(&pins, 8, 0x0000_01AA, 0x87, 4);
    drive_and_settle(pins.cs.as_ref().unwrap(), Level::High);
    drop(pins);

    assert_eq!(idle, vec![0x01], "CMD0's R1: idle");
    assert_eq!(
        r7,
        vec![0x01, 0x00, 0x00, 0x01, 0xAA],
        "CMD8's R7: idle, then the voltage range and the check pattern echoed"
    );
    assert_eq!(
        card_handle.lock().expect("card").commands,
        vec![0, 8],
        "the card took both frames"
    );
    assert_eq!(
        flash_view.commands(),
        Vec::<u8>::new(),
        "the flash saw no clock edge while selected"
    );
    let fights: Vec<Finding> = bench
        .system
        .findings()
        .into_iter()
        .filter(|f| {
            matches!(f, Finding::Contention { net, .. }
                if net.ends_with(".P2_IO58") || net.ends_with(".Net-(J301-DAT0_MISO)"))
        })
        .collect();
    assert!(
        fights.is_empty(),
        "P58 and the card's side of R304 are the card's: {fights:?}"
    );
    bench.finish();
}
