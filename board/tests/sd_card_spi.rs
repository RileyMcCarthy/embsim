//! An SD card in SPI mode as a LIVE board component.
//!
//! [`SdCardComponent`] is generic, so what it needs proving against is a real
//! socket on a real netlist plus the protocol a real driver speaks. This binary
//! does both: it mounts the card on the one netlist in the tree that carries a
//! card socket — the Parallax P2-EC32MB module, whose `J301` is a Molex
//! `473092651` microSD socket — and then drives the SPI-mode initialisation and
//! block transfers over nets, bit by bit.
//!
//! # What the mounting test actually proves
//!
//! `Board::from_netlist` validates every registered component's pin
//! facade against the netlist in BOTH directions, matching on `PinDecl::number`
//! verbatim. A facade that is right for the *part* and wrong for the netlist's
//! identifier convention fails here and nowhere else: a model's own unit tests
//! never build a board, so they pass while the component cannot be mounted at
//! all. The flash on this same module was written with datasheet pin numbers
//! and was unmountable for exactly that reason.
//!
//! A socket adds a second way to get it wrong. `J301` has eight nodes — the
//! four SPI signals, `VDD`, and three grounds the connector shell contributes
//! (`VSS`, `GND1`, `GND2`) — and it does **not** have `DAT1`, `DAT2`, `SW1` or
//! `SW2`, which this board leaves unconnected. An unconnected pin has no node,
//! so a facade that declares the card's full pinout cannot mount on a socket
//! wired for SPI. Hence two facades, and a test for each.
//!
//! # Sources
//!
//! - `fixtures/p2_ec32mb.net` — `(comp (ref "J301"))`, value `"MicroSD Socket"`,
//!   MPN `473092651`, manufacturer Molex.
//! - SD Physical Layer Simplified Specification, for the SPI-mode command set,
//!   response formats and pin roles.
//! - ChaN's `sdmm.cc`, the reference SPI-mode driver, for the order a real host
//!   actually sends those commands in.

mod machine_parts;

use std::collections::BTreeSet;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use embsim_board::{
    digital_drive, level_of, netlist, AttachError, Board, Component, ComponentNetIo, Harness,
    IdleDrive, Level, PartRegistry, PinDecl, PinHandle, PinKind, System,
};
use embsim_core::virtual_clock;
use embsim_models::sd_card::{SdCard, BLOCK_LEN};
use embsim_models::sd_card_component::{
    SdCardComponent, SD_CARD_PINS_BY_FUNCTION, SD_CARD_PINS_MICROSD, SD_CARD_PINS_SPI_ONLY,
};
use machine_parts::ec32mb_registry;

/// The netlist's `value` for `J301`, which is the registry key.
const SOCKET_PART: &str = "MicroSD Socket";

/// Big enough for the blocks these tests touch, small enough to allocate freely.
const CARD_CAPACITY: usize = 64 * BLOCK_LEN;

// ============================================================
// Mounting on a real netlist
// ============================================================

fn registry_with_live_card(pins: &'static [PinDecl]) -> PartRegistry {
    let mut registry = ec32mb_registry();
    registry.register(SOCKET_PART, move |_decl| {
        Box::new(SdCardComponent::blank(CARD_CAPACITY).with_pins(pins))
    });
    registry
}

#[test]
fn the_card_mounts_on_a_real_socket_in_place_of_a_board_boundary() {
    let parsed =
        netlist::parse(include_str!("fixtures/p2_ec32mb.net")).expect("the EC32MB fixture parses");
    let board = Board::from_netlist(parsed, &registry_with_live_card(&SD_CARD_PINS_BY_FUNCTION))
        .expect("the live card's pin facade matches J301 in both directions");

    let registered: BTreeSet<&str> = board.component_refs().collect();
    assert!(
        registered.contains("J301"),
        "the card socket is a registered component, not a J-prefixed boundary \
         skipped over"
    );
}

#[test]
fn the_full_card_pinout_does_not_mount_on_a_socket_wired_for_spi() {
    let parsed =
        netlist::parse(include_str!("fixtures/p2_ec32mb.net")).expect("the EC32MB fixture parses");
    let error = Board::from_netlist(parsed, &registry_with_live_card(&SD_CARD_PINS_MICROSD))
        .expect_err("pin \"1\" is not a pin this netlist has, and DAT1/DAT2 have no nodes");
    let rendered = format!("{error}");
    assert!(
        rendered.contains("J301"),
        "the error names the part that cannot be mounted: {rendered}"
    );
}

#[test]
fn the_socket_facade_declares_exactly_the_nodes_the_netlist_gives_j301() {
    const FIXTURE: &str = include_str!("fixtures/p2_ec32mb.net");
    let parsed = netlist::parse(FIXTURE).expect("the EC32MB fixture parses");
    let j301 = parsed
        .components
        .iter()
        .find(|c| c.reference == "J301")
        .expect("the module has a card socket");
    assert_eq!(
        j301.value, SOCKET_PART,
        "and it is the part the registry keys on"
    );

    // The nodes the netlist actually gives J301, gathered from the nets rather
    // than assumed. DAT1/DAT2/SW1/SW2 are absent because the board leaves them
    // unconnected, which is why the microSD facade cannot mount here.
    let mut nodes: BTreeSet<&str> = BTreeSet::new();
    for net in &parsed.nets {
        for node in &net.nodes {
            if node.reference == "J301" {
                nodes.insert(node.pin.as_str());
            }
        }
    }
    let declared: BTreeSet<&str> = SD_CARD_PINS_BY_FUNCTION.iter().map(|p| p.number).collect();
    assert_eq!(
        declared, nodes,
        "the by-function facade is exactly this socket's connected pins"
    );
}

// ============================================================
// The protocol, without any wires
// ============================================================
//
// These drive `SdCard` directly, a byte at a time, in the order ChaN's
// `sdmm.cc` sends them. They are the fast check that the state machine is
// right; the net-driven tests below are the check that the adapter is.

/// Clock idle bytes until the card stops holding the line busy — what a real
/// driver's `wait_ready` does, and what a host MUST do before its next command:
/// a card still draining a response reads the command's first byte as one more
/// response clock and never sees the frame at all.
fn powered_card() -> SdCard {
    let mut card = SdCard::blank(CARD_CAPACITY);
    // A card powers up deselected; the host asserts CS before its first
    // command. Saying so here rather than in the model's constructor is what
    // keeps the model honest about the part.
    card.set_selected(true);
    card
}

fn wait_ready(card: &mut SdCard) {
    for _ in 0..16 {
        if card.xfer(0xFF) == 0xFF {
            return;
        }
    }
    panic!("the card never went ready");
}

/// Send a 6-byte command frame and clock out `n` response bytes, skipping the
/// `$FF` idles a card holds the line at while it thinks.
fn command(card: &mut SdCard, cmd: u8, arg: u32, want: usize) -> Vec<u8> {
    let a = arg.to_be_bytes();
    for b in [0x40 | cmd, a[0], a[1], a[2], a[3], 0x01] {
        card.xfer(b);
    }
    let mut out = Vec::new();
    // A real host polls for the response byte; bit 7 clear is how it finds one.
    for _ in 0..16 {
        let b = card.xfer(0xFF);
        if out.is_empty() && b & 0x80 != 0 {
            continue;
        }
        out.push(b);
        if out.len() == want {
            break;
        }
    }
    out
}

#[test]
fn the_card_answers_the_initialisation_sequence_a_real_driver_sends() {
    let mut card = powered_card();

    // CMD0 GO_IDLE_STATE -> R1 with the idle bit set.
    assert_eq!(command(&mut card, 0, 0, 1), vec![0x01], "CMD0 reports idle");

    // CMD8 SEND_IF_COND -> R7: R1 then four bytes, the last echoing the check
    // pattern. This is what makes a host treat the card as SDv2 and use block
    // addressing rather than byte offsets.
    let r7 = command(&mut card, 8, 0x0000_01AA, 5);
    assert_eq!(r7[0], 0x01, "still idle");
    assert_eq!(r7[3], 0x01, "voltage range accepted");
    assert_eq!(r7[4], 0xAA, "the check pattern comes back unchanged");

    // CMD55 + ACMD41 -> initialisation completes and R1 goes to zero, which is
    // the poll a host spins on.
    assert_eq!(command(&mut card, 55, 0, 1), vec![0x00]);
    assert_eq!(command(&mut card, 41, 0x4000_0000, 1), vec![0x00]);
    assert!(card.initialised, "ACMD41 is what marks the card ready");

    // CMD58 READ_OCR -> R3, CCS set: a block-addressed (SDHC) card.
    let r3 = command(&mut card, 58, 0, 5);
    assert_eq!(r3[0], 0x00, "ready");
    assert_eq!(r3[1] & 0x40, 0x40, "CCS set means block addressing");

    assert_eq!(
        card.commands,
        vec![0, 8, 55, 0x80 | 41, 58],
        "every command is recorded in order, ACMD41 flagged as an app command"
    );
}

#[test]
fn an_unsupported_command_is_refused_rather_than_answered() {
    let mut card = powered_card();
    // CMD59 CRC_ON_OFF is deliberately not modelled. Reporting it illegal is
    // what lets a host fall back; pretending would hide the gap.
    assert_eq!(
        command(&mut card, 59, 1, 1),
        vec![0x04],
        "R1 illegal-command bit"
    );
}

#[test]
fn a_block_written_with_cmd24_reads_back_with_cmd17() {
    let mut card = powered_card();
    let payload: Vec<u8> = (0..BLOCK_LEN).map(|i| (i % 251) as u8).collect();

    // CMD24 WRITE_BLOCK: R1, then the host sends a start token, the payload and
    // two CRC bytes, and the card answers with the data-accepted token.
    assert_eq!(command(&mut card, 24, 7, 1), vec![0x00]);
    card.xfer(0xFE); // start token
    for &b in &payload {
        card.xfer(b);
    }
    card.xfer(0xFF);
    card.xfer(0xFF);
    assert_eq!(
        card.xfer(0xFF) & 0x1F,
        0x05,
        "bits 3:0 = %0101 means the data was accepted"
    );
    wait_ready(&mut card);

    // And it landed at block 7, not somewhere else.
    assert_eq!(&card.blocks[7 * BLOCK_LEN..8 * BLOCK_LEN], &payload[..]);

    // CMD17 READ_SINGLE_BLOCK: R1, then a start token and 512 bytes.
    assert_eq!(command(&mut card, 17, 7, 1), vec![0x00]);
    let mut token = 0xFF;
    for _ in 0..8 {
        token = card.xfer(0xFF);
        if token != 0xFF {
            break;
        }
    }
    assert_eq!(token, 0xFE, "the data-block start token");
    let read: Vec<u8> = (0..BLOCK_LEN).map(|_| card.xfer(0xFF)).collect();
    assert_eq!(read, payload, "the block comes back byte for byte");

    assert_eq!(card.writes, vec![7]);
    assert_eq!(card.reads, vec![7]);
}

#[test]
fn deselecting_frees_a_card_left_waiting_for_a_block_that_never_came() {
    // The failure this guards is a real one and it is silent: a host sends
    // CMD24, times out waiting for the card to be ready and gives up WITHOUT
    // sending the data token. The card sits in its receive phase, swallowing
    // every later command frame as payload, and the bus is dead for the rest of
    // the run. Raising chip select is the only thing that frees it, which is
    // why `set_selected` is a method rather than a public field.
    let mut card = powered_card();
    assert_eq!(command(&mut card, 24, 1, 1), vec![0x00]);

    // The host walks away. A command sent now is eaten as block payload.
    assert_eq!(
        command(&mut card, 0, 0, 1),
        Vec::<u8>::new(),
        "the card is mid-block and answers nothing"
    );

    card.set_selected(false);
    card.set_selected(true);
    assert_eq!(
        command(&mut card, 0, 0, 1),
        vec![0x01],
        "after a deselect the card takes commands again"
    );
}

#[test]
fn a_multi_block_read_streams_until_cmd12_stops_it() {
    let mut image = vec![0u8; CARD_CAPACITY];
    // Stamp each block with its own index so a mis-addressed read is visible.
    for block in 0..CARD_CAPACITY / BLOCK_LEN {
        image[block * BLOCK_LEN] = block as u8;
    }
    let mut card = SdCard::with_image(image);
    card.set_selected(true);

    assert_eq!(command(&mut card, 18, 4, 1), vec![0x00], "CMD18 accepted");
    let mut first_bytes = Vec::new();
    for _ in 0..3 {
        // Clock until the start token, then take the block's first byte and
        // skip the rest plus its CRC.
        let mut token = 0xFF;
        for _ in 0..8 {
            token = card.xfer(0xFF);
            if token != 0xFF {
                break;
            }
        }
        assert_eq!(token, 0xFE, "each block arrives with its own start token");
        first_bytes.push(card.xfer(0xFF));
        for _ in 1..BLOCK_LEN + 2 {
            card.xfer(0xFF);
        }
    }
    assert_eq!(
        first_bytes,
        vec![4, 5, 6],
        "consecutive blocks, without the host re-addressing"
    );

    assert_eq!(command(&mut card, 12, 0, 1), vec![0x00], "CMD12 stops it");
    assert_eq!(card.reads, vec![4, 5, 6]);
}

// ============================================================
// Driven over real nets
// ============================================================
//
// The tests above prove the facade mounts and the state machine answers. These
// prove the CARD answers when its pins are nets rather than method calls —
// a different question, because the engine never resolves a drive inline. A
// master that drives the clock and reads the data line without letting the
// engine run in between reads a stale bit, and the bytes come back shifted.

/// The four lines a SPI master owns, captured when the engine wires them.
#[derive(Default)]
struct MasterPins {
    cs: Option<PinHandle>,
    clk: Option<PinHandle>,
    di: Option<PinHandle>,
    dout: Option<PinHandle>,
}

/// A bare bit-banging SPI master: four pins and no behaviour of its own, so the
/// test thread can drive the bus a level at a time and observe exactly what a
/// firmware loop would.
struct BitBangMaster {
    pins: [PinDecl; 4],
    handles: Arc<Mutex<MasterPins>>,
}

impl BitBangMaster {
    fn new(handles: Arc<Mutex<MasterPins>>) -> Self {
        let decl = |number, name, kind| PinDecl {
            number,
            name: Some(name),
            kind,
            stream: None,
            drive_impedance: None,
            idle: IdleDrive::KindDefault,
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

/// Drive a level and then LET THE ENGINE RUN. Every edge goes through here: the
/// drive is enqueued, the engine resolves it and delivers the card's sense
/// callback, and the card's answering drive is resolved in turn. Skip the wait
/// and the next sense reads what was there before.
///
/// The wait is a CONDITION, not a duration: sensing our own pin reads the
/// resolved net, so this returns as soon as the engine has actually applied the
/// drive. A fixed sleep long enough to be safe on a loaded machine costs
/// milliseconds per edge, and a 512-byte block is 12 288 of them — the
/// difference between a test that runs in a second and one that runs in a
/// minute and a quarter.
fn drive_and_settle(pin: &PinHandle, level: Level) {
    pin.set_drive(Some(digital_drive(level)));
    for _ in 0..SETTLE_POLLS {
        if level_of(pin.sense()) == Some(level) {
            return;
        }
        std::thread::sleep(SETTLE_POLL);
    }
    panic!("the engine never applied a drive of {level:?}");
}

/// How long each poll of [`drive_and_settle`] waits, and how many it allows
/// before giving up — together, a generous ceiling on how long the engine may
/// take to resolve one drive.
const SETTLE_POLL: Duration = Duration::from_micros(50);
const SETTLE_POLLS: u32 = 2_000;

/// Drop the clock and give the card time to answer.
///
/// [`drive_and_settle`] can only wait on a condition it can observe, and what
/// it observes is OUR drive landing — which says nothing about whether the
/// card's sense callback has run and its answering DO drive has been resolved.
/// The falling edge is the one where that matters, because it is the edge the
/// card presents its next bit on, so this is the one place a fixed wait is
/// unavoidable.
///
/// Waiting here rather than in every `drive_and_settle` is what keeps the
/// 512-byte tests to seconds: one fixed wait per bit instead of three.
fn clock_low(pin: &PinHandle) {
    drive_and_settle(pin, Level::Low);
    std::thread::sleep(CARD_ANSWER);
}

/// How long the card gets to put its next bit on the wire.
const CARD_ANSWER: Duration = Duration::from_micros(300);

/// A released MISO has no level at all — the card drives nothing when it is
/// deselected, exactly as a real one does. A bench without a pull-up therefore
/// reads `None`, and this resolves that the way the missing resistor would: to
/// a one, which is the idle bit an SD driver expects.
fn sense_bit(pin: &PinHandle) -> bool {
    level_of(pin.sense()).is_none_or(|level| level == Level::High)
}

/// Exchange one byte, MSB first, the way the wire actually does it: both ends
/// sample on the rising edge, so the master reads the bit the card has already
/// presented BEFORE it raises the clock, and the card presents the next one on
/// the falling edge.
///
/// Getting this order wrong costs the first bit of every byte — the symptom is
/// a response stream shifted one bit left, which reads as plausible garbage
/// rather than as an obvious failure.
fn exchange(pins: &MasterPins, mosi: u8) -> u8 {
    let (clk, di, dout) = (
        pins.clk.as_ref().unwrap(),
        pins.di.as_ref().unwrap(),
        pins.dout.as_ref().unwrap(),
    );
    let mut miso = 0u8;
    for i in (0..8).rev() {
        drive_and_settle(
            di,
            if (mosi >> i) & 1 != 0 {
                Level::High
            } else {
                Level::Low
            },
        );
        miso = (miso << 1) | u8::from(sense_bit(dout));
        drive_and_settle(clk, Level::High);
        clock_low(clk);
    }
    miso
}

/// Send a command frame over the nets and clock out its response, skipping the
/// idles the card holds the line at while it thinks.
fn net_command(pins: &MasterPins, cmd: u8, arg: u32, want: usize) -> Vec<u8> {
    let a = arg.to_be_bytes();
    for b in [0x40 | cmd, a[0], a[1], a[2], a[3], 0x01] {
        exchange(pins, b);
    }
    let mut out = Vec::new();
    for _ in 0..16 {
        let b = exchange(pins, 0xFF);
        if out.is_empty() && b & 0x80 != 0 {
            continue;
        }
        out.push(b);
        if out.len() == want {
            break;
        }
    }
    out
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

/// Bring up a four-wire bench with the card on it, and wait for the master's
/// pins to be wired.
fn spi_bench(
    card: SdCard,
) -> (
    Arc<Mutex<MasterPins>>,
    Arc<Mutex<SdCard>>,
    embsim_board::SystemHandle,
) {
    virtual_clock::init(50.0, 1_000_000);

    let handles = Arc::new(Mutex::new(MasterPins::default()));
    let harness = Harness::new()
        .connect_str("MASTER.CS", "CARD.CS")
        .expect("endpoints parse")
        .connect_str("MASTER.CLK", "CARD.CLK")
        .expect("endpoints parse")
        .connect_str("MASTER.DI", "CARD.MOSI")
        .expect("endpoints parse")
        .connect_str("MASTER.DO", "CARD.MISO")
        .expect("endpoints parse");

    let component = SdCardComponent::new(card).with_pins(&SD_CARD_PINS_SPI_ONLY);
    let card_handle = component.card();
    let system = System::new()
        .component("MASTER", Box::new(BitBangMaster::new(Arc::clone(&handles))))
        .component("CARD", Box::new(component))
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
    (handles, card_handle, system)
}

/// Idle the clock low and take the card out of, then into, chip select — the
/// state a driver establishes before its first command.
fn open_bus(pins: &MasterPins) {
    clock_low(pins.clk.as_ref().unwrap());
    drive_and_settle(pins.di.as_ref().unwrap(), Level::High);
    drive_and_settle(pins.cs.as_ref().unwrap(), Level::High);
    drive_and_settle(pins.cs.as_ref().unwrap(), Level::Low);
    // Selection is also an edge the card answers: it presents the first bit of
    // whatever it has queued.
    std::thread::sleep(CARD_ANSWER);
}

#[test]
fn the_card_initialises_over_nets_driven_bit_by_bit() {
    let (handles, card, _system) = spi_bench(SdCard::blank(CARD_CAPACITY));
    let pins = handles.lock().expect("master pins");
    open_bus(&pins);

    assert_eq!(
        net_command(&pins, 0, 0, 1),
        vec![0x01],
        "CMD0 reports idle over four nets and an engine round trip per edge"
    );
    let r7 = net_command(&pins, 8, 0x0000_01AA, 5);
    assert_eq!(r7[4], 0xAA, "the check pattern survived the wire");
    assert_eq!(net_command(&pins, 55, 0, 1), vec![0x00]);
    assert_eq!(net_command(&pins, 41, 0x4000_0000, 1), vec![0x00]);

    assert!(
        card.lock().expect("card").initialised,
        "the card behind the nets really did initialise"
    );
}

#[test]
fn a_block_written_over_nets_reads_back_over_nets() {
    // A recognisable payload: a mis-shifted byte stream would not reproduce it.
    let payload: Vec<u8> = (0..BLOCK_LEN).map(|i| (i * 7 % 251) as u8).collect();

    let (handles, card, _system) = spi_bench(SdCard::blank(CARD_CAPACITY));
    let pins = handles.lock().expect("master pins");
    open_bus(&pins);

    // Write block 3.
    assert_eq!(net_command(&pins, 24, 3, 1), vec![0x00], "CMD24 accepted");
    exchange(&pins, 0xFE);
    for &b in &payload {
        exchange(&pins, b);
    }
    exchange(&pins, 0xFF);
    exchange(&pins, 0xFF);
    assert_eq!(
        exchange(&pins, 0xFF) & 0x1F,
        0x05,
        "the card accepted the block it was clocked"
    );
    // Drain the busy byte, as a driver's wait_ready does.
    for _ in 0..16 {
        if exchange(&pins, 0xFF) == 0xFF {
            break;
        }
    }

    // It landed in the image, at the block asked for.
    assert_eq!(
        &card.lock().expect("card").blocks[3 * BLOCK_LEN..4 * BLOCK_LEN],
        &payload[..],
        "512 bytes crossed the wire intact"
    );

    // And reads back the same way.
    assert_eq!(net_command(&pins, 17, 3, 1), vec![0x00], "CMD17 accepted");
    let mut token = 0xFF;
    for _ in 0..8 {
        token = exchange(&pins, 0xFF);
        if token != 0xFF {
            break;
        }
    }
    assert_eq!(token, 0xFE, "the data-block start token");
    let read: Vec<u8> = (0..BLOCK_LEN).map(|_| exchange(&pins, 0xFF)).collect();
    assert_eq!(read, payload, "and came back byte for byte");
}

/// The control that makes the tests above mean something.
///
/// The master enqueues every edge and never lets the engine run, so when it
/// senses MISO nothing has been resolved — not even the chip select — and it
/// reads the released line. All-ones is exactly the signature of an empty
/// socket, which is the failure this costs you: not a corrupt byte, a card that
/// appears absent.
#[test]
fn without_a_yield_between_edges_the_card_reads_as_absent() {
    let (handles, _card, _system) = spi_bench(SdCard::blank(CARD_CAPACITY));
    let pins = handles.lock().expect("master pins");

    let rush = |p: &PinHandle, l: Level| p.set_drive(Some(digital_drive(l)));
    rush(pins.clk.as_ref().unwrap(), Level::Low);
    rush(pins.cs.as_ref().unwrap(), Level::High);
    rush(pins.cs.as_ref().unwrap(), Level::Low);
    // CMD0, then clock for the R1 that should come back as $01.
    for byte in [0x40u8, 0, 0, 0, 0, 0x95, 0xFF, 0xFF] {
        for i in (0..8).rev() {
            rush(
                pins.di.as_ref().unwrap(),
                if (byte >> i) & 1 != 0 {
                    Level::High
                } else {
                    Level::Low
                },
            );
            rush(pins.clk.as_ref().unwrap(), Level::High);
            rush(pins.clk.as_ref().unwrap(), Level::Low);
        }
    }
    let mut r1 = 0u8;
    for _ in 0..8 {
        rush(pins.clk.as_ref().unwrap(), Level::High);
        rush(pins.clk.as_ref().unwrap(), Level::Low);
        r1 = (r1 << 1) | u8::from(sense_bit(pins.dout.as_ref().unwrap()));
    }

    assert_ne!(
        r1, 0x01,
        "a master that never yields cannot have read the idle response"
    );
}

/// The counters exist so a test can assert the bus MOVED, rather than inferring
/// it from the absence of an error — a card that answered nothing and a bench
/// that never clocked look identical from the response alone.
#[test]
fn the_adapter_counts_the_edges_and_bytes_it_actually_saw() {
    virtual_clock::init(50.0, 1_000_000);

    let handles = Arc::new(Mutex::new(MasterPins::default()));
    let harness = Harness::new()
        .connect_str("MASTER.CS", "CARD.CS")
        .expect("endpoints parse")
        .connect_str("MASTER.CLK", "CARD.CLK")
        .expect("endpoints parse")
        .connect_str("MASTER.DI", "CARD.MOSI")
        .expect("endpoints parse")
        .connect_str("MASTER.DO", "CARD.MISO")
        .expect("endpoints parse");

    let component = SdCardComponent::blank(CARD_CAPACITY).with_pins(&SD_CARD_PINS_SPI_ONLY);
    let counters = component.counters();
    let _system = System::new()
        .component("MASTER", Box::new(BitBangMaster::new(Arc::clone(&handles))))
        .component("CARD", Box::new(component))
        .harness(harness)
        .start()
        .expect("the bench system starts");
    assert!(wait_for(
        || handles.lock().expect("master pins").clk.is_some(),
        Duration::from_secs(5)
    ));

    let pins = handles.lock().expect("master pins");
    open_bus(&pins);
    for b in [0x40u8, 0, 0, 0, 0, 0x95] {
        exchange(&pins, b);
    }

    use std::sync::atomic::Ordering;
    assert_eq!(
        counters.bytes.load(Ordering::Relaxed),
        6,
        "six command bytes, counted where they were assembled"
    );
    assert_eq!(
        counters.edges.load(Ordering::Relaxed),
        6 * 16,
        "two edges per bit, and none counted while deselected"
    );
}

#[test]
fn a_card_powers_up_deselected_and_ignores_the_bus_until_it_is_addressed() {
    // The power-up sequence a host runs — clock bursts with CS held high — must
    // not be mistaken for traffic. A model that powered up selected would
    // assemble a command frame out of those dummy bytes.
    let mut card = SdCard::blank(CARD_CAPACITY);
    assert!(
        !card.selected,
        "a card idles deselected under its CS pull-up"
    );

    for _ in 0..80 {
        assert_eq!(card.xfer(0xFF), 0xFF, "a deselected card holds the line");
    }
    // Even a well-formed command frame is ignored while CS is high.
    for b in [0x40u8, 0, 0, 0, 0, 0x95] {
        card.xfer(b);
    }
    assert!(
        card.commands.is_empty(),
        "nothing on the bus reaches a card that has not been addressed"
    );

    card.set_selected(true);
    assert_eq!(
        command(&mut card, 0, 0, 1),
        vec![0x01],
        "and now it answers"
    );
}

/// The two halves together: a filesystem image built in memory, mounted on the
/// card model, read back over four nets.
///
/// Either piece alone is half a card — a block device with nothing on it, or an
/// image with nothing to read it. This is the path a guest actually takes to
/// its first sector, and the assertion is the one a FAT driver makes: the boot
/// signature at the end of sector 0, and a volume that says FAT16.
#[test]
fn a_fat16_image_built_in_memory_reads_back_over_the_wire() {
    use embsim_models::fat16::{build, Dir};

    let mut root = Dir::new();
    root.file("hello.txt", b"mounted".to_vec());
    root.dir("data");
    // The smallest geometry that is unambiguously FAT16; see `fat16`'s docs on
    // why the cluster count, not the label, decides the type.
    let image = build(32 * 1024 * 1024, &root).expect("a FAT16 image builds");

    let (handles, _card, _system) = spi_bench(SdCard::with_image(image.clone()));
    let pins = handles.lock().expect("master pins");
    open_bus(&pins);

    assert_eq!(
        net_command(&pins, 17, 0, 1),
        vec![0x00],
        "CMD17 for sector 0"
    );
    let mut token = 0xFF;
    for _ in 0..8 {
        token = exchange(&pins, 0xFF);
        if token != 0xFF {
            break;
        }
    }
    assert_eq!(token, 0xFE, "the data-block start token");
    let sector: Vec<u8> = (0..BLOCK_LEN).map(|_| exchange(&pins, 0xFF)).collect();

    assert_eq!(
        &sector[510..512],
        &[0x55, 0xAA],
        "the boot signature a FAT driver looks for first"
    );
    assert_eq!(
        &sector[54..59],
        b"FAT16",
        "and the filesystem type in the boot sector"
    );
    assert_eq!(
        sector,
        image[..BLOCK_LEN],
        "the sector on the wire is the sector in the image, byte for byte"
    );
}
