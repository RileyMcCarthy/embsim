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
//!
//! # Time
//!
//! The cases that drive the card over nets run alone, on the stepped clock
//! (`TESTING.md` rules 5 and 9), the pattern `isolation_bridge.rs` set: each
//! takes the suite lock, re-anchors the clock stepped, starts its bench with
//! time held, registers its thread as a virtual-clock actor and releases
//! time. The master then waits after every edge with [`settle`] — a park on
//! the virtual clock, during which the engine applies the edge, delivers
//! it to the card and applies the card's answer — and the thread holds the
//! engine still between two settles, so every read is of the bus at rest.
//! [`Bench::finish`] asserts the engine never stopped waiting for the case.
//!
//! These cases once waited on the wall clock: a poll for the master's own
//! drive and a fixed 300 µs for the card's answer to a falling clock edge.
//! Under load the answer had not landed when the master next read: the poll
//! for its data-in drive returns as soon as the net reads the level — at
//! once when a bit repeats the one before — and 300 µs was shorter than the
//! engine took to deliver the edge and apply the answer. A block read back
//! with one to four bytes wrong, every wrong bit equal to the bit before it
//! (`NODES.md` §12 item 5, the flake record's Open list and the performance
//! and stepped-tests record).

mod machine_parts;

use std::collections::BTreeSet;
use std::sync::{Arc, Mutex, MutexGuard};

use embsim_board::{
    digital_drive, jesd8c01_lvcmos_thresholds, level_of, netlist, AttachError, Board, Component,
    ComponentNetIo, DeadBand, Finding, Harness, Level, PartRegistry, PinDecl, PinHandle, Scenario,
    System, SystemHandle,
};
use embsim_core::virtual_clock::{self, Actor, ClockMode};
use embsim_models::sd_card::{SdCard, BLOCK_LEN};
use embsim_models::sd_card_component::{
    SdCardComponent, SD_CARD_PINS_BY_FUNCTION, SD_CARD_PINS_MICROSD, SD_CARD_PINS_SPI_ONLY,
};
use machine_parts::ec32mb_registry;
use vibes_behaviour::{behaviour, expect, Test};

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

/// One bench at a time: the virtual clock is process-global, and each bench
/// re-anchors it in stepped mode (`TESTING.md` rule 5). The cases that drive
/// the card model directly run no engine and take no lock.
static SUITE_LOCK: Mutex<()> = Mutex::new(());

fn suite_lock() -> MutexGuard<'static, ()> {
    SUITE_LOCK.lock().unwrap_or_else(|poisoned| {
        SUITE_LOCK.clear_poison();
        poisoned.into_inner()
    })
}

/// The virtual time the master hands the engine after each edge: 1 µs.
///
/// Nothing on this bench arms an instant — the card adapter answers each
/// edge from its sense callback at the edge's own instant, and the master
/// is the test thread — so a settle only has to let the engine drain the
/// instant, and any span reads the same. It must not be zero: a zero wait
/// returns without parking (`virtual_clock::wait_virtual_ns`), and a
/// thread that never parks holds the engine still.
const SETTLE_NS: u64 = 1_000;
const _: () = assert!(SETTLE_NS > 0);

/// Park the master's thread for [`SETTLE_NS`] of virtual time and return with
/// the bus at rest.
///
/// The thread is a registered actor ([`Bench`]), and the stepped engine
/// advances only while every actor is parked: here it applies every drive
/// the master queued before the call, delivers the card's sense callbacks,
/// applies the card's answering drive, and then releases the thread — and it
/// does nothing more until the thread parks again.
fn settle() {
    virtual_clock::wait_virtual_ns(SETTLE_NS);
}

/// Drive a level and then LET THE ENGINE RUN. Every edge goes through here: the
/// drive is enqueued, the engine resolves it and delivers the card's sense
/// callback, and the card's answering drive is resolved in turn. Skip the wait
/// and the next sense reads what was there before.
///
/// The wait is [`settle`], so when it returns the card has answered too —
/// on a falling clock edge, the edge the card presents its next bit on, its
/// DO drive has been applied — and sensing our own pin reads the level we
/// drove, exactly.
fn drive_and_settle(pin: &PinHandle, level: Level) {
    pin.set_drive(Some(digital_drive(level)));
    settle();
    assert_eq!(
        level_of(pin.net_report()),
        Some(level),
        "the engine applied a drive of {level:?}"
    );
}

/// A released MISO has no level at all — the card drives nothing when it is
/// deselected, exactly as a real one does. A bench without a pull-up therefore
/// reads `None`, and this resolves that the way the missing resistor would: to
/// a one, which is the idle bit an SD driver expects.
fn sense_bit(pin: &PinHandle) -> bool {
    level_of(pin.net_report()).is_none_or(|level| level == Level::High)
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
        drive_and_settle(clk, Level::Low);
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

/// A running four-wire bench and the case's hold on it.
///
/// Field order is drop order, and it is load-bearing: the case's actor
/// registration goes first, so a case that panics stops holding the
/// engine's barrier before its system shuts down, and the suite lock goes
/// last, after the engine has been joined.
struct Bench {
    /// The case's thread as a registered virtual-clock actor, from the
    /// started bench to [`Bench::finish`]: what makes [`settle`] exact.
    actor: Actor,
    system: SystemHandle,
    handles: Arc<Mutex<MasterPins>>,
    _suite: MutexGuard<'static, ()>,
}

impl Bench {
    /// End the case: the engine must never have stopped waiting for the
    /// case's thread (a `QuiescenceTimeout` would mean a settled read may
    /// have raced the bus), then the thread leaves the barrier and the
    /// system shuts down.
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

/// Bring up a four-wire bench with `component` on it: take the suite lock,
/// re-anchor the clock in stepped mode, start the bench with time held,
/// register the case's thread as an actor, release time and settle — the
/// bench is handed back at rest, the master's pins wired at attach.
fn start_bench(component: SdCardComponent) -> Bench {
    let suite = suite_lock();
    virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);

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

    let system = System::new()
        .component("MASTER", Box::new(BitBangMaster::new(Arc::clone(&handles))))
        .component(
            "CARD",
            Box::new(component.with_pins(&SD_CARD_PINS_SPI_ONLY)),
        )
        .harness(harness)
        .hold_time()
        .start()
        .expect("the bench system starts");
    let actor = virtual_clock::register_actor("sd-card-spi-case");
    system.release_time();
    settle();

    assert!(
        handles.lock().expect("master pins").clk.is_some(),
        "the master's pins are wired at attach"
    );
    Bench {
        actor,
        system,
        handles,
        _suite: suite,
    }
}

/// [`start_bench`] with `card` behind the adapter, and a handle on the card.
fn spi_bench(card: SdCard) -> (Bench, Arc<Mutex<SdCard>>) {
    let component = SdCardComponent::new(card);
    let card_handle = component.card();
    (start_bench(component), card_handle)
}

/// Idle the clock low and take the card out of, then into, chip select — the
/// state a driver establishes before its first command. Selection is also an
/// edge the card answers — it presents the first bit of whatever it has
/// queued — and the chip select's settle applies that answer.
fn open_bus(pins: &MasterPins) {
    drive_and_settle(pins.clk.as_ref().unwrap(), Level::Low);
    drive_and_settle(pins.di.as_ref().unwrap(), Level::High);
    drive_and_settle(pins.cs.as_ref().unwrap(), Level::High);
    drive_and_settle(pins.cs.as_ref().unwrap(), Level::Low);
}

#[test]
fn the_card_initialises_over_nets_driven_bit_by_bit() {
    let (bench, card) = spi_bench(SdCard::blank(CARD_CAPACITY));
    let pins = bench.handles.lock().expect("master pins");
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
    drop(pins);
    bench.finish();
}

#[test]
fn the_numbered_microsd_pinout_answers_cmd0() {
    behaviour!(Test {
        id: "sd-card.numbered-pinout-answers-idle",
        covers: Some("models/src/sd_card_component.rs#SD_CARD_PINS_MICROSD"),
        given: "a microSD card whose pins are numbered 1 through 8 on a bench netlist, powered at 3.3 volts, and sent the go-idle command",
    });
    expect!("idle", "the card answers that it is idle");
    let suite = suite_lock();
    virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);
    const NUMBERED: &str = r#"(export (version "E")
  (components
    (comp (ref "J1") (value "card") (libsource (lib "Connector") (part "microSD"))))
  (nets
    (net (code "1") (name "DAT2") (node (ref "J1") (pin "1")))
    (net (code "2") (name "CS") (node (ref "J1") (pin "2")))
    (net (code "3") (name "DI") (node (ref "J1") (pin "3")))
    (net (code "4") (name "VDD") (node (ref "J1") (pin "4")))
    (net (code "5") (name "CLK") (node (ref "J1") (pin "5")))
    (net (code "6") (name "VSS") (node (ref "J1") (pin "6")))
    (net (code "7") (name "DO") (node (ref "J1") (pin "7")))
    (net (code "8") (name "DAT1") (node (ref "J1") (pin "8")))))"#;
    let mut registry = PartRegistry::new();
    registry.register("microSD", |_| {
        Box::new(SdCardComponent::blank(CARD_CAPACITY).with_pins(&SD_CARD_PINS_MICROSD))
    });
    let board = Board::from_netlist(netlist::parse(NUMBERED).expect("parses"), &registry)
        .expect("the numbered facade mounts");
    let handles = Arc::new(Mutex::new(MasterPins::default()));
    // The netlist numbers the pads. The adapter still looks the clock up
    // by the name CLK, which is the alias on pad 5.
    let harness = Harness::new()
        .connect_str("MASTER.CS", "CARD.J1.2")
        .expect("cs")
        .connect_str("MASTER.CLK", "CARD.J1.5")
        .expect("clk")
        .connect_str("MASTER.DI", "CARD.J1.3")
        .expect("di")
        .connect_str("MASTER.DO", "CARD.J1.7")
        .expect("do");
    let system = System::new()
        .board("CARD", board)
        .component("MASTER", Box::new(BitBangMaster::new(Arc::clone(&handles))))
        .harness(harness)
        .scenario(
            Scenario::default()
                .net_stuck("CARD.VSS", 0.0)
                .net_stuck("CARD.VDD", 3.3),
        )
        .hold_time()
        .start()
        .expect("the numbered card starts");
    let actor = virtual_clock::register_actor("sd-numbered-pinout");
    system.release_time();
    settle();
    let bench = Bench {
        actor,
        system,
        handles,
        _suite: suite,
    };
    let pins = bench.handles.lock().expect("master pins");
    open_bus(&pins);
    assert_eq!(net_command(&pins, 0, 0, 1), vec![0x01]);
    drop(pins);
    bench.finish();
}

#[test]
fn a_block_written_over_nets_reads_back_over_nets() {
    // A recognisable payload: a mis-shifted byte stream would not reproduce it.
    let payload: Vec<u8> = (0..BLOCK_LEN).map(|i| (i * 7 % 251) as u8).collect();

    let (bench, card) = spi_bench(SdCard::blank(CARD_CAPACITY));
    let pins = bench.handles.lock().expect("master pins");
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
    drop(pins);
    bench.finish();
}

/// The control that makes the tests above mean something.
///
/// The master enqueues every edge and never lets the engine run, so when it
/// senses MISO nothing has been resolved — not even the chip select — and it
/// reads the released line. All-ones is exactly the signature of an empty
/// socket, which is the failure this costs you: not a corrupt byte, a card that
/// appears absent.
///
/// On the stepped clock "never lets the engine run" is exact: the master is
/// a registered actor that never parks between its edges, and the engine
/// advances nothing while an actor is running, so every bit it senses is
/// the line as the settle after start-up left it — released.
#[test]
fn without_a_yield_between_edges_the_card_reads_as_absent() {
    let (bench, _card) = spi_bench(SdCard::blank(CARD_CAPACITY));
    let pins = bench.handles.lock().expect("master pins");

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
    assert_eq!(
        r1, 0xFF,
        "every bit it sensed is the released line: the empty socket's signature"
    );
    drop(pins);
    bench.finish();
}

/// The counters exist so a test can assert the bus MOVED, rather than inferring
/// it from the absence of an error — a card that answered nothing and a bench
/// that never clocked look identical from the response alone.
#[test]
fn the_adapter_counts_the_edges_and_bytes_it_actually_saw() {
    let component = SdCardComponent::blank(CARD_CAPACITY);
    let counters = component.counters();
    let bench = start_bench(component);

    let pins = bench.handles.lock().expect("master pins");
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
    drop(pins);
    bench.finish();
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

    let (bench, _card) = spi_bench(SdCard::with_image(image.clone()));
    let pins = bench.handles.lock().expect("master pins");
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
    drop(pins);
    bench.finish();
}
