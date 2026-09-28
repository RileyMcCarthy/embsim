//! Board-engine adapter: [`crate::sd_card::SdCard`] as a live
//! [`embsim_board::Component`] — four wires, no shortcuts.
//!
//! The protocol model stays a pure byte-level state machine; this is the seam
//! that puts it on a netlist and turns clock edges into bytes, so a *system
//! description* decides which pins it sits on rather than the model hardcoding
//! them:
//!
//! ```text
//!  net engine                      adapter                   device model
//!  ──────────                      ───────                   ────────────
//!  CS  on_sense ──active low──► set_selected(!high) ──► phase reset
//!  DI  on_sense ─────────────► remembered, sampled at the next clock edge
//!  CLK on_sense ──8 edges────► xfer(byte) ───────────► command / response
//!  DO  ◄── digital_drive(bit of peek_miso()) ◄── on each leading edge
//! ```
//!
//! # Why the card must speak before it has finished listening
//!
//! SPI is simultaneous. The card's outgoing bits are on the wire while the
//! host's incoming byte is still arriving, so a bit-level adapter needs the
//! outgoing byte at the *start* of an exchange, not the end.
//! [`SdCard::peek_miso`] provides it, and doing so is exact rather than an
//! approximation: in SPI mode a card's response is queued by an earlier
//! command and never depends on the byte arriving now.
//!
//! # Clock edges
//!
//! Both SPI modes an SD card accepts — mode 0 and mode 3 — **sample on the
//! rising edge and change on the falling edge**; they differ only in the level
//! the clock idles at. So this adapter needs no mode setting: it presents a bit
//! on every falling edge and samples MOSI on every rising one, which is correct
//! for either. A host that inverts its clock output (CPOL = 1) gets the same
//! byte stream as one that does not.
//!
//! # Idle, and the pull-up the host is assuming
//!
//! A deselected card releases MISO — this adapter drives nothing at all rather
//! than driving high, because that is what a real card does. Nothing then
//! drives that net, so **the bench needs the pull-up the host firmware
//! assumes**; SPI-mode SD drivers habitually enable one on the receive pin for
//! exactly this reason. Modelled as a bench resistor rather than pretended
//! away, because "MISO reads high when no card answers" is the behaviour card
//! detection depends on.
//!
//! # Chip select is active low HERE, not in the model
//!
//! [`SdCard`] takes an asserted/not-asserted boolean. Inverting CS is the
//! adapter's job because active-low is a property of the wiring.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

use embsim_board::{
    digital_drive, level_of, AttachError, Component, ComponentNetIo, Level, PinDecl, PinHandle,
    PinKind,
};
use tracing::trace;

use crate::sd_card::SdCard;

const fn pin(number: &'static str, name: &'static str, kind: PinKind) -> PinDecl {
    PinDecl {
        number,
        name: Some(name),
        kind,
        stream: None,
        drive_impedance: None,
    }
}

/// A pin whose netlist identifier already IS the name the adapter uses, so it
/// needs no alias. Declaring one anyway makes the handle table insert the same
/// key twice, which a bench system rejects as a duplicate endpoint.
const fn pin_unaliased(number: &'static str, kind: PinKind) -> PinDecl {
    PinDecl {
        number,
        name: None,
        kind,
        stream: None,
        drive_impedance: None,
    }
}

/// The 8-pin microSD facade with card pin NUMBERS as identifiers, for a
/// netlist that numbers its pins — as an EDA export does.
///
/// The SPI-mode roles are the SD Physical Layer Simplified Specification's:
/// `CD/DAT3` becomes chip select, `CMD` becomes DI, `DAT0` becomes DO, and
/// `DAT1`/`DAT2` are unused. Both are declared and **never driven**, which is
/// what a card in SPI mode does with them.
pub const SD_CARD_PINS_MICROSD: [PinDecl; 8] = [
    pin("1", "DAT2", PinKind::DigitalIn),
    pin("2", "CS", PinKind::DigitalIn),
    pin("3", "DI", PinKind::DigitalIn),
    pin("4", "VDD", PinKind::PowerIn),
    pin_unaliased("5", PinKind::DigitalIn), // CLK, aliased below
    pin("6", "VSS", PinKind::PowerIn),
    pin("7", "DO", PinKind::DigitalOut),
    pin("8", "DAT1", PinKind::DigitalIn),
];

/// The same card keyed by FUNCTION, which is how a netlist transcribed from a
/// schematic names socket pins.
///
/// Unlike [`SD_CARD_PINS_MICROSD`] this describes a **socket**, not a bare
/// card: it carries the shell grounds a connector adds (`GND1`, `GND2`) and
/// omits `DAT1`/`DAT2`, which a socket wired for SPI leaves unconnected — and
/// an unconnected pin has no node, so declaring it would fail validation.
pub const SD_CARD_PINS_BY_FUNCTION: [PinDecl; 8] = [
    pin("CD_DAT3_CS", "CS", PinKind::DigitalIn),
    pin("CMD_MOSI", "DI", PinKind::DigitalIn),
    pin("DAT0_MISO", "DO", PinKind::DigitalOut),
    pin_unaliased("CLK", PinKind::DigitalIn),
    pin_unaliased("VDD", PinKind::PowerIn),
    pin_unaliased("VSS", PinKind::PowerIn),
    pin_unaliased("GND1", PinKind::PowerIn),
    pin_unaliased("GND2", PinKind::PowerIn),
];

/// A bare four-wire facade, for a bench netlist that names the SPI signals and
/// nothing else.
pub const SD_CARD_PINS_SPI_ONLY: [PinDecl; 4] = [
    pin_unaliased("CLK", PinKind::DigitalIn),
    pin_unaliased("CS", PinKind::DigitalIn),
    pin("MOSI", "DI", PinKind::DigitalIn),
    pin("MISO", "DO", PinKind::DigitalOut),
];

/// What the card has seen and is saying.
///
/// Shared between the sense callbacks, which the engine delivers serially from
/// one thread — so this mutex is never contended by the engine with itself,
/// only with a consumer reading the image out.
#[derive(Debug, Default)]
struct Wire {
    clk_high: bool,
    selected: bool,
    /// The last level seen on DI. Sampled at the clock's rising edge; the host
    /// sets it up beforehand, so the engine delivers that sense first.
    mosi: bool,
    /// Bits taken from MOSI this byte, MSB first.
    in_bits: u8,
    in_count: u32,
    /// The byte being shifted out, and how far through it we are.
    out_byte: u8,
    out_count: u32,
}

/// Observable counts, so a test can assert the bus actually moved rather than
/// inferring it from the absence of an error.
#[derive(Debug, Default)]
pub struct Counters {
    /// Clock edges seen while selected.
    pub edges: AtomicU64,
    /// Whole bytes exchanged.
    pub bytes: AtomicU64,
}

/// An SD card on a board.
pub struct SdCardComponent {
    card: Arc<Mutex<SdCard>>,
    wire: Arc<Mutex<Wire>>,
    counters: Arc<Counters>,
    pins: &'static [PinDecl],
}

impl std::fmt::Debug for SdCardComponent {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SdCardComponent").finish_non_exhaustive()
    }
}

impl SdCardComponent {
    /// Mount `card` as a board component, with the by-function facade a
    /// transcribed netlist uses. Call [`Self::with_pins`] for another.
    pub fn new(card: SdCard) -> Self {
        Self {
            card: Arc::new(Mutex::new(card)),
            wire: Arc::new(Mutex::new(Wire {
                // An undriven data line idles high on its pull-up, and a host
                // that clocks before driving DI shifts in ones.
                mosi: true,
                ..Wire::default()
            })),
            counters: Arc::new(Counters::default()),
            pins: &SD_CARD_PINS_BY_FUNCTION,
        }
    }

    /// A blank card of `bytes` capacity.
    pub fn blank(bytes: usize) -> Self {
        Self::new(SdCard::blank(bytes))
    }

    /// Declare a different pin facade — [`SD_CARD_PINS_MICROSD`] for a netlist
    /// that numbers pins, [`SD_CARD_PINS_SPI_ONLY`] for a four-wire bench.
    #[must_use]
    pub fn with_pins(mut self, pins: &'static [PinDecl]) -> Self {
        self.pins = pins;
        self
    }

    /// A view of the counters that survives handing the component to a
    /// `System`.
    pub fn counters(&self) -> Arc<Counters> {
        Arc::clone(&self.counters)
    }

    /// The card itself, for reading back what a host wrote.
    pub fn card(&self) -> Arc<Mutex<SdCard>> {
        Arc::clone(&self.card)
    }

    /// The disk image as writes have left it.
    pub fn image_bytes(&self) -> Vec<u8> {
        self.card.lock().expect("card mutex").blocks.clone()
    }

    /// Every command the host has issued, in order, ACMDs flagged with bit 7.
    pub fn commands(&self) -> Vec<u8> {
        self.card.lock().expect("card mutex").commands.clone()
    }

    /// Block addresses read, oldest first.
    pub fn reads(&self) -> Vec<u32> {
        self.card.lock().expect("card mutex").reads.clone()
    }

    /// Block addresses written, oldest first.
    pub fn writes(&self) -> Vec<u32> {
        self.card.lock().expect("card mutex").writes.clone()
    }
}

/// Put the bit the card is currently presenting on DO, or release the line
/// when the card is not selected.
///
/// A deselected card is genuinely high-impedance, so this drives `None` rather
/// than a high level: whether the net then reads high is the bench's pull-up to
/// decide, not this component's to assert.
fn drive_data_out(wire: &Wire, data_out: &PinHandle) {
    if !wire.selected {
        data_out.set_drive(None);
        return;
    }
    let bit = wire.out_byte >> (7 - wire.out_count.min(7)) & 1 != 0;
    data_out.set_drive(Some(digital_drive(if bit {
        Level::High
    } else {
        Level::Low
    })));
}

impl Component for SdCardComponent {
    fn pins(&self) -> &[PinDecl] {
        self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let data_out = io.pin("DO")?;

        {
            let wire = self.wire.lock().expect("wire mutex");
            // Released until selected.
            drive_data_out(&wire, &data_out);
        }

        // Chip select. A deselected card releases DO and forgets where it was
        // in a byte — the next selection starts a fresh exchange.
        {
            let (card, wire, data_out) = (
                Arc::clone(&self.card),
                Arc::clone(&self.wire),
                data_out.clone(),
            );
            io.on_sense("CS", move |state| {
                let Some(level) = level_of(state) else {
                    // Floating chip select is not a state a card can act on;
                    // hold, and let the engine's diagnostics report it.
                    trace!(?state, "SD card: CS has no level; holding selection");
                    return;
                };
                let selected = level == Level::Low;
                let mut wire = wire.lock().expect("wire mutex");
                if wire.selected == selected {
                    return;
                }
                wire.selected = selected;
                wire.in_count = 0;
                wire.out_count = 0;
                let mut card = card.lock().expect("card mutex");
                card.set_selected(selected);
                wire.out_byte = card.peek_miso();
                drop(card);
                drive_data_out(&wire, &data_out);
            })?;
        }

        // DI: remembered only. The card samples it at the clock edge, so a
        // level change on its own moves nothing. A floating DI keeps its last
        // value rather than guessing: the host is mid-transfer and about to
        // drive it again.
        {
            let wire = Arc::clone(&self.wire);
            io.on_sense("DI", move |state| {
                if let Some(level) = level_of(state) {
                    wire.lock().expect("wire mutex").mosi = level == Level::High;
                }
            })?;
        }

        // The clock is the engine of the whole exchange.
        {
            let (card, wire, counters, data_out) = (
                Arc::clone(&self.card),
                Arc::clone(&self.wire),
                Arc::clone(&self.counters),
                data_out.clone(),
            );
            io.on_sense("CLK", move |state| {
                let Some(level) = level_of(state) else {
                    return;
                };
                let high = level == Level::High;
                let mut wire = wire.lock().expect("wire mutex");
                if wire.clk_high == high || !wire.selected {
                    // Track the level even while deselected, so the first edge
                    // after a selection is judged against where the clock
                    // actually is rather than against `false`.
                    wire.clk_high = high;
                    return;
                }
                wire.clk_high = high;
                counters.edges.fetch_add(1, Ordering::Relaxed);

                if high {
                    // Trailing edge: the host's bit is stable.
                    wire.in_bits = (wire.in_bits << 1) | u8::from(wire.mosi);
                    wire.in_count += 1;
                    if wire.in_count == 8 {
                        wire.in_count = 0;
                        let byte = wire.in_bits;
                        wire.in_bits = 0;
                        // The card consumes the byte; what it returns is what
                        // has already been shifted out, bit by bit.
                        card.lock().expect("card mutex").xfer(byte);
                        counters.bytes.fetch_add(1, Ordering::Relaxed);
                    }
                    // Move to the next outgoing bit only after sampling, so a
                    // byte boundary lands in the same place both ways.
                    wire.out_count += 1;
                    if wire.out_count == 8 {
                        wire.out_count = 0;
                        wire.out_byte = card.lock().expect("card mutex").peek_miso();
                    }
                } else {
                    // Leading edge: present the bit this pulse carries.
                    drive_data_out(&wire, &data_out);
                }
            })?;
        }

        Ok(())
    }
}
