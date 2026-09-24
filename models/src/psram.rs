//! Model: the **AP Memory APS6404L-3SQR** 64 Mbit QSPI pseudo-SRAM in its
//! **SPI mode** — the four `PSRAM 64Mbit` parts `U302`–`U305` on a Parallax
//! P2-EC32MB module — device side, bit-level, bus-agnostic, on the shift
//! engine the serial NOR flash uses ([`crate::spi_shift::ByteShifter`]).
//!
//! The ROM and the firmware never touch these parts; that is not a reason
//! for them to be a facade (`DESIGN.md` rule 1). What is modelled is what a
//! master would meet: chip select, the serial command set answered from an
//! array, and a data-out line that is high-impedance until the part has
//! something to say.
//!
//! # Datasheet provenance
//!
//! AP Memory **"APS6404L-3SQR QSPI PSRAM" — APM SPI 3V PSRAM Datasheet,
//! Rev. 2.3, Apr 30, 2020** (24 pages):
//!
//! - **Organization** — 64 Mb, 8 M × 8 bits; "Addressable Bit Range:
//!   A\[22:0\]"; page size 1024 bytes (p. 1 "Specifications"; §8.1, §8.2).
//!   [`APS6404L_CAPACITY`], [`APS6404L_PAGE_SIZE`].
//! - **Supply** — `V_DD` 2.7 V to 3.6 V (p. 1). Not gated here (see the
//!   simplifications).
//! - **Drive strength** — "50 Ω Output Drive Strength LVCMOS" (p. 1
//!   "Features"); "The device powers up in 50 Ω" (§8.3).
//!   [`APS6404L_OUTPUT_OHMS`].
//! - **Pins** — §3.1, SOP / USON, top view: 1 `/CE`, 2 `SO/SIO[1]`,
//!   3 `SIO[2]`, 4 `VSS`, 5 `SI/SIO[0]`, 6 `SCLK`, 7 `SIO[3]`, 8 `VDD`.
//!   Table 2 "Signals Table": in SPI mode `SI/SIO[0]` is the serial input,
//!   `SO/SIO[1]` the serial output, `SIO[2]` and `SIO[3]` are `--`
//!   (unused). [`PSRAM_PINS_SOP8`], [`PSRAM_PINS_BY_FUNCTION`].
//! - **Power-on** — "The device powers up in SPI Mode. It is required to
//!   have CE# high before beginning any operations" (§8.4).
//! - **Command set** — §8.5 "Command/Address Latching Truth Table", the
//!   SPI-mode (QE = 0) column: Read `'h03` (serial command, serial address,
//!   0 wait cycles, max 33 MHz); Fast Read `'h0B` (8 wait cycles); Write
//!   `'h02` (0 wait cycles); Enter Quad Mode `'h35`; Reset Enable `'h66`;
//!   Reset `'h99`; Wrap Boundary Toggle `'hC0`; Read ID `'h9F` (serial,
//!   0 wait cycles, 33 MHz). Fast Read Quad `'hEB` and Quad Write `'h38`
//!   take quad address and data and are out of scope with QPI.
//! - **Read** — §10.1 Figure 6 "SPI Read 'h03": eight command bits, a
//!   24-bit address, then data out from that address, the output high-Z
//!   until the first data bit; "For all reads, data will be available
//!   `t_ACLK` after the falling edge of CLK". §10.1 Figure 7: Fast Read
//!   inserts eight wait cycles between the address and the data.
//! - **Write** — §10.2 Figure 9 "SPI Write 'h02": command, 24-bit address,
//!   data bytes in, linearly.
//! - **Read ID** — §10.4 Figure 12 "SPI Read ID 'h9F": command, a 24-bit
//!   address (its value unused), then `MF ID ('h0D)`, `KGD ('h5D)`, then
//!   `EID[47:45]` (density) and `EID[44:0]`. Table 4 "Known Good Die":
//!   `'b0101_1101` = PASS, `'b0101_0101` = FAIL, "Default is FAIL die, and
//!   only mark PASS after all tests passed" — a part that shipped is PASS.
//!   [`APS6404L_MF_ID`], [`APS6404L_KGD_PASS`].
//! - **Termination** — §8.6 "All Reads & Writes must be completed by
//!   raising CE# high"; Figure 3: `SO` goes high-Z `t_HZ` after CE# rises.
//! - **Bursts** — §9: linear burst is the default and crosses the 1 KB page
//!   boundary "one time only in a burst"; Wrap 32 is selected by `'hC0`.
//!
//! # Deliberate simplifications
//!
//! - **QPI mode is out of scope.** Enter Quad Mode (`'h35`) is recognised
//!   and recorded, and from then on the part answers nothing and releases
//!   `SO` — a serial master that switched the part to quad IO cannot talk
//!   to it serially any more, which is the truthful part of the behaviour;
//!   the quad transfers themselves are not modelled, nor is Exit Quad Mode
//!   (`'hF5`, a QPI-only command). A new part is the way back.
//! - **Reset** (`'h66` then `'h99`, §12) returns the part to its power-up
//!   state: SPI mode, linear burst; the array is kept (the datasheet's
//!   reset does not say the array is cleared). A `'h99` without an
//!   enabling `'h66` is ignored.
//! - **Wrap Boundary Toggle** (`'hC0`) is recorded and ignored: every
//!   burst is linear over the whole array, and one that runs past the end
//!   wraps to address 0 — the datasheet limits a burst to one page
//!   crossing and `t_CEM`, neither of which is modelled, so a master that
//!   overruns reads valid data here and undefined data on the part.
//! - **Timing** — `t_PU` (150 µs after `V_DD`), `t_CEM`, `t_ACLK`, `t_HZ`
//!   and the clock-rate limits are not modelled; a bit is presented for
//!   the rising edge it belongs to, the engine's presentation rule
//!   ([`crate::spi_flash`]), which a master sampling on the rising edge
//!   reads exactly as it reads a bit the part changed after the previous
//!   falling edge.
//! - **No supply gate.** `VDD`/`VSS` are declared and not sensed, as on
//!   the flash; the part answers whenever it is selected.
//! - **The EID** carries no datasheet value here: Figure 12 names its
//!   fields and no number, so it defaults to zeros and a consumer that
//!   knows its part's EID sets it ([`Psram::with_eid`]).

use std::sync::{Arc, Mutex};

use embsim_board::net::LOGIC_HIGH_VOLTS;
use embsim_board::{
    level_of, AttachError, Component, ComponentNetIo, IdleDrive, Level, Ohms, PinDecl, PinHandle,
    PinKind, TheveninDrive,
};
use tracing::trace;

use crate::spi_shift::ByteShifter;

// ============================================================
// Datasheet constants
// ============================================================

/// 64 Mb = 8 M × 8 bits (p. 1 "Organization"), 8 MiB.
pub const APS6404L_CAPACITY: usize = 8 * 1024 * 1024;

/// A[22:0] (§8.1): the address a burst wraps within.
const ADDRESS_MASK: u32 = 0x007F_FFFF;

/// "Page Size: 1024 bytes" (p. 1; §8.2, CA\[9:0\]).
pub const APS6404L_PAGE_SIZE: u32 = 1024;

/// The manufacturer ID byte a Read ID returns first (§10.4 Figure 12,
/// "MF ID ('h0D)").
pub const APS6404L_MF_ID: u8 = 0x0D;

/// The Known Good Die byte of a passing part (Table 4: `'b0101_1101`).
pub const APS6404L_KGD_PASS: u8 = 0x5D;

/// Output drive strength: 50 Ω (p. 1 "Features"; §8.3 "The device powers
/// up in 50 Ω").
pub const APS6404L_OUTPUT_OHMS: Ohms = 50.0;

/// Wait cycles a Fast Read (`'h0B`) inserts after the address: 8 (§8.5),
/// one byte of clocks.
const FAST_READ_WAIT_BYTES: u32 = 1;

/// How many decoded commands and read/write records to retain.
const DIAG_LIMIT: usize = 4096;

// ============================================================
// The device
// ============================================================

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
enum Phase {
    #[default]
    Command,
    /// Collecting the 24-bit address of a read; `wait` bytes of wait cycles
    /// follow it.
    ReadAddress { have: u32, wait: u32 },
    /// Wait cycles before data out.
    Wait { left: u32 },
    /// Data out from `addr`.
    Reading,
    /// Collecting the 24-bit address of a write.
    WriteAddress { have: u32 },
    /// Every further byte is written.
    Writing,
    /// Collecting the (unused) 24-bit address of a Read ID.
    IdAddress { have: u32 },
    /// Shifting the ID out: MF ID, KGD, then the EID.
    Id { index: u32 },
}

/// An APS6404L in SPI mode.
#[derive(Debug)]
pub struct Psram {
    array: Vec<u8>,
    eid: [u8; 6],
    selected: bool,
    shift: ByteShifter,
    phase: Phase,
    addr_acc: u32,
    addr: u32,
    /// Whether the out byte being shifted is data (a read, the ID) — the
    /// bytes shifted through the command, address and wait phases are
    /// never on the line.
    out_is_data: bool,
    /// Whether the bit presented on the line belongs to a data byte: `SO`
    /// leaves high-Z with the first data bit (Figures 6, 7, 12), one edge
    /// after the phase turns to data.
    presented_is_data: bool,
    /// Enter Quad Mode was issued: the part no longer answers serially.
    quad: bool,
    /// Reset Enable was the last command: the next `'h99` resets.
    reset_enabled: bool,

    /// Every command opcode decoded, in order.
    pub commands: Vec<u8>,
    /// Starting addresses of reads served, oldest first.
    pub reads: Vec<u32>,
    /// `(address, len)` of each write, oldest first.
    pub writes: Vec<(u32, u32)>,
}

impl Default for Psram {
    fn default() -> Self {
        Self::new()
    }
}

impl Psram {
    /// A part whose array reads all zeros. The array is the part's real
    /// 8 MiB; the pages are only touched when written.
    pub fn new() -> Self {
        Self {
            array: vec![0u8; APS6404L_CAPACITY],
            eid: [0; 6],
            selected: false,
            shift: ByteShifter::new(),
            phase: Phase::Command,
            addr_acc: 0,
            addr: 0,
            out_is_data: false,
            presented_is_data: false,
            quad: false,
            reset_enabled: false,
            commands: Vec::new(),
            reads: Vec::new(),
            writes: Vec::new(),
        }
    }

    /// The 48-bit EID a Read ID returns after the MF ID and KGD bytes
    /// (§10.4 Figure 12 names its fields and no value; zeros by default).
    pub fn with_eid(mut self, eid: [u8; 6]) -> Self {
        self.eid = eid;
        self
    }

    /// Capacity in bytes.
    pub fn capacity(&self) -> usize {
        self.array.len()
    }

    /// The byte at `addr`.
    pub fn byte(&self, addr: u32) -> u8 {
        self.array[(addr & ADDRESS_MASK) as usize]
    }

    /// Whether the part has been switched to quad IO, which this model does
    /// not serve.
    pub fn in_quad_mode(&self) -> bool {
        self.quad
    }

    /// The bit on `SO`, or `None` while the line is high-impedance: the
    /// part deselected (§8.6 Figure 3), between the command and the data
    /// (Figures 6, 9, 12: `SO` is high-Z through the command and address),
    /// or the part in quad mode.
    pub fn so(&self) -> Option<bool> {
        if !self.selected || self.quad || !self.presented_is_data {
            return None;
        }
        Some(self.shift.dout())
    }

    /// Whether the phase shifts data out.
    const fn phase_is_data(&self) -> bool {
        matches!(self.phase, Phase::Reading | Phase::Id { .. })
    }

    /// Chip select changed. `selected` is the ASSERTED sense — the adapter
    /// inverts the active-low `/CE`. A deselect terminates the command
    /// (§8.6).
    pub fn set_selected(&mut self, selected: bool) {
        if self.selected == selected {
            return;
        }
        self.selected = selected;
        self.phase = Phase::Command;
        self.addr_acc = 0;
        self.out_is_data = false;
        self.presented_is_data = false;
        let first = self.next_out();
        self.shift.begin(first);
    }

    /// Move the clock to `high`, sampling `si`. Idempotent in the level.
    pub fn clock(&mut self, high: bool, si: bool) {
        if !self.selected || self.quad {
            self.shift.track_clock(high);
            return;
        }
        let Some(edge) = self.shift.clock(high, si) else {
            return;
        };
        // The bit now on the line belongs to the byte loaded at the last
        // boundary.
        self.presented_is_data = self.out_is_data;
        if let Some(byte) = edge.byte_in {
            self.consume(byte);
        }
        if edge.finished_out.is_some() {
            let next = self.next_out();
            self.out_is_data = self.phase_is_data();
            self.shift.load_out(next);
        }
    }

    /// The byte to shift out next, by phase. Bytes shifted out while `SO`
    /// is high-Z are never seen.
    fn next_out(&mut self) -> u8 {
        match self.phase {
            Phase::Reading => {
                let byte = self.byte(self.addr);
                self.addr = (self.addr + 1) & ADDRESS_MASK;
                byte
            }
            Phase::Id { index } => {
                let byte = match index {
                    0 => APS6404L_MF_ID,
                    1 => APS6404L_KGD_PASS,
                    n => self.eid.get((n - 2) as usize).copied().unwrap_or(0),
                };
                self.phase = Phase::Id { index: index + 1 };
                byte
            }
            _ => 0,
        }
    }

    fn consume(&mut self, byte: u8) {
        self.phase = match self.phase {
            Phase::Command => {
                if self.commands.len() < DIAG_LIMIT {
                    self.commands.push(byte);
                }
                let enabling = byte == 0x66;
                let phase = match byte {
                    // §8.5, SPI mode column.
                    0x03 => Phase::ReadAddress { have: 0, wait: 0 },
                    0x0B => Phase::ReadAddress {
                        have: 0,
                        wait: FAST_READ_WAIT_BYTES,
                    },
                    0x02 => Phase::WriteAddress { have: 0 },
                    0x9F => Phase::IdAddress { have: 0 },
                    0x35 => {
                        // Enter Quad Mode: from here the part answers only
                        // quad transfers, which are out of scope.
                        self.quad = true;
                        Phase::Command
                    }
                    0x66 => Phase::Command, // Reset Enable
                    0x99 => {
                        // Reset, armed by the Reset Enable before it (§12).
                        if self.reset_enabled {
                            self.quad = false;
                        }
                        Phase::Command
                    }
                    // Wrap Boundary Toggle: recorded, not modelled.
                    0xC0 => Phase::Command,
                    _ => Phase::Command,
                };
                self.reset_enabled = enabling;
                phase
            }
            Phase::ReadAddress { have, wait } => {
                self.addr_acc = (self.addr_acc << 8) | u32::from(byte);
                if have == 2 {
                    self.addr = self.addr_acc & ADDRESS_MASK;
                    self.addr_acc = 0;
                    if self.reads.len() < DIAG_LIMIT {
                        self.reads.push(self.addr);
                    }
                    if wait == 0 {
                        Phase::Reading
                    } else {
                        Phase::Wait { left: wait }
                    }
                } else {
                    Phase::ReadAddress {
                        have: have + 1,
                        wait,
                    }
                }
            }
            Phase::Wait { left } => {
                if left <= 1 {
                    Phase::Reading
                } else {
                    Phase::Wait { left: left - 1 }
                }
            }
            Phase::WriteAddress { have } => {
                self.addr_acc = (self.addr_acc << 8) | u32::from(byte);
                if have == 2 {
                    self.addr = self.addr_acc & ADDRESS_MASK;
                    self.addr_acc = 0;
                    if self.writes.len() < DIAG_LIMIT {
                        self.writes.push((self.addr, 0));
                    }
                    Phase::Writing
                } else {
                    Phase::WriteAddress { have: have + 1 }
                }
            }
            Phase::Writing => {
                let addr = self.addr;
                self.array[addr as usize] = byte;
                if let Some(last) = self.writes.last_mut() {
                    last.1 += 1;
                }
                self.addr = (addr + 1) & ADDRESS_MASK;
                Phase::Writing
            }
            Phase::IdAddress { have } => {
                if have == 2 {
                    Phase::Id { index: 0 }
                } else {
                    Phase::IdAddress { have: have + 1 }
                }
            }
            other => other,
        };
    }
}

// ============================================================
// Pin facades
// ============================================================

const fn pin(number: &'static str, name: Option<&'static str>, kind: PinKind) -> PinDecl {
    PinDecl {
        number,
        name,
        kind,
        stream: None,
        drive_impedance: None,
        idle: IdleDrive::KindDefault,
    }
}

/// `SO`: driven only while the part has data out; released otherwise.
const fn so_pin(number: &'static str, name: Option<&'static str>) -> PinDecl {
    PinDecl {
        number,
        name,
        kind: PinKind::DigitalOut,
        stream: None,
        drive_impedance: None,
        idle: IdleDrive::Released,
    }
}

/// The facade keyed by **function**, as the P2-EC32MB netlist names
/// `U302`–`U305`'s pins. `NC_EP` is the package's exposed pad (tied per
/// the vendor's routing note), passive. `SIO2`/`SIO3` are unused in SPI
/// mode (Table 2) and sensed by nothing.
pub const PSRAM_PINS_BY_FUNCTION: [PinDecl; 9] = [
    pin("VSS", None, PinKind::PowerIn),
    pin("VDD", None, PinKind::PowerIn),
    pin("NC_EP", None, PinKind::Passive),
    pin("SCLK", None, PinKind::DigitalIn),
    pin("CEn", Some("/CE"), PinKind::DigitalIn),
    pin("SI_SIO0", Some("SI"), PinKind::DigitalIn),
    so_pin("SO_SIO1", Some("SO")),
    pin("SIO2", None, PinKind::DigitalIn),
    pin("SIO3", None, PinKind::DigitalIn),
];

/// The SOP-8 / USON-8 facade by pin **number** (§3.1): 1 `/CE`,
/// 2 `SO/SIO[1]`, 3 `SIO[2]`, 4 `VSS`, 5 `SI/SIO[0]`, 6 `SCLK`, 7 `SIO[3]`,
/// 8 `VDD`.
pub const PSRAM_PINS_SOP8: [PinDecl; 8] = [
    pin("1", Some("/CE"), PinKind::DigitalIn),
    so_pin("2", Some("SO")),
    pin("3", Some("SIO2"), PinKind::DigitalIn),
    pin("4", Some("VSS"), PinKind::PowerIn),
    pin("5", Some("SI"), PinKind::DigitalIn),
    pin("6", Some("SCLK"), PinKind::DigitalIn),
    pin("7", Some("SIO3"), PinKind::DigitalIn),
    pin("8", Some("VDD"), PinKind::PowerIn),
];

// ============================================================
// Component
// ============================================================

#[derive(Debug)]
struct Shared {
    psram: Psram,
    /// The last level seen on SI, sampled at the clock edge.
    si: bool,
}

/// An APS6404L on a board.
pub struct PsramComponent {
    shared: Arc<Mutex<Shared>>,
    pins: &'static [PinDecl],
}

impl std::fmt::Debug for PsramComponent {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PsramComponent").finish_non_exhaustive()
    }
}

/// A view of the part that survives handing the component to a `System`.
#[derive(Clone)]
pub struct PsramView {
    shared: Arc<Mutex<Shared>>,
}

impl std::fmt::Debug for PsramView {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PsramView").finish_non_exhaustive()
    }
}

impl PsramView {
    /// Every command opcode the master has issued, in order.
    pub fn commands(&self) -> Vec<u8> {
        self.shared
            .lock()
            .expect("psram mutex")
            .psram
            .commands
            .clone()
    }

    /// Starting addresses of the reads served, oldest first.
    pub fn reads(&self) -> Vec<u32> {
        self.shared.lock().expect("psram mutex").psram.reads.clone()
    }

    /// `(address, len)` of each write, oldest first.
    pub fn writes(&self) -> Vec<(u32, u32)> {
        self.shared
            .lock()
            .expect("psram mutex")
            .psram
            .writes
            .clone()
    }

    /// The byte at `addr`.
    pub fn byte(&self, addr: u32) -> u8 {
        self.shared.lock().expect("psram mutex").psram.byte(addr)
    }
}

impl PsramComponent {
    /// Mount `psram` with the by-function facade a transcribed netlist
    /// uses; [`Self::with_pins`] for a numbered one.
    pub fn new(psram: Psram) -> Self {
        Self {
            shared: Arc::new(Mutex::new(Shared {
                psram,
                // An undriven input reads as one, as the flash assumes.
                si: true,
            })),
            pins: &PSRAM_PINS_BY_FUNCTION,
        }
    }

    /// Declare a different pin facade — [`PSRAM_PINS_SOP8`] for a netlist
    /// that identifies pins by number. `attach` looks its pins up as
    /// `/CE`, `SCLK`, `SI` and `SO`.
    pub fn with_pins(mut self, pins: &'static [PinDecl]) -> Self {
        self.pins = pins;
        self
    }

    /// A view that survives handing this component to a `System`.
    pub fn view(&self) -> PsramView {
        PsramView {
            shared: Arc::clone(&self.shared),
        }
    }
}

/// Publish the part's `SO` level, or release the line while it is high-Z.
fn publish_so(shared: &Mutex<Shared>, so: &PinHandle) {
    let drive = shared
        .lock()
        .expect("psram mutex")
        .psram
        .so()
        .map(|bit| TheveninDrive {
            volts: if bit { LOGIC_HIGH_VOLTS } else { 0.0 },
            impedance: APS6404L_OUTPUT_OHMS,
        });
    so.set_drive(drive);
}

impl Component for PsramComponent {
    fn pins(&self) -> &[PinDecl] {
        self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let so = io.pin("SO")?;

        {
            let shared = Arc::clone(&self.shared);
            io.on_sense("SI", move |state| {
                if let Some(level) = level_of(state) {
                    shared.lock().expect("psram mutex").si = level == Level::High;
                }
            })?;
        }
        {
            let shared = Arc::clone(&self.shared);
            let so = so.clone();
            io.on_sense("/CE", move |state| {
                let Some(level) = level_of(state) else {
                    trace!(?state, "PSRAM: /CE has no level; holding selection");
                    return;
                };
                shared
                    .lock()
                    .expect("psram mutex")
                    .psram
                    .set_selected(level == Level::Low);
                publish_so(&shared, &so);
            })?;
        }
        {
            let shared = Arc::clone(&self.shared);
            let so = so.clone();
            io.on_sense("SCLK", move |state| {
                let Some(level) = level_of(state) else {
                    trace!(?state, "PSRAM: SCLK has no level; no edge");
                    return;
                };
                {
                    let mut guard = shared.lock().expect("psram mutex");
                    let si = guard.si;
                    guard.psram.clock(level == Level::High, si);
                }
                publish_so(&shared, &so);
            })?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn send(psram: &mut Psram, byte: u8) {
        for i in (0..8).rev() {
            let bit = (byte >> i) & 1 != 0;
            psram.clock(true, bit);
            psram.clock(false, bit);
        }
    }

    /// Pulse then sample, as a bit-banging master does.
    fn recv(psram: &mut Psram) -> u8 {
        let mut b = 0u8;
        for _ in 0..8 {
            psram.clock(true, true);
            psram.clock(false, true);
            b = (b << 1) | u8::from(psram.so().expect("data out"));
        }
        b
    }

    fn tx(psram: &mut Psram, bytes: &[u8]) {
        psram.set_selected(true);
        for &b in bytes {
            send(psram, b);
        }
        psram.set_selected(false);
    }

    #[test]
    fn read_id_answers_the_manufacturer_and_a_passing_die_after_a_dummy_address() {
        let mut psram = Psram::new().with_eid([0x60, 0, 0, 0, 0, 0]);
        psram.set_selected(true);
        assert_eq!(psram.so(), None, "high-Z through the command");
        send(&mut psram, 0x9F);
        for _ in 0..3 {
            send(&mut psram, 0xAA);
            assert_eq!(psram.so(), None, "and through the address");
        }
        assert_eq!(recv(&mut psram), 0x0D, "MF ID");
        assert_eq!(recv(&mut psram), 0x5D, "KGD: a shipped part passed");
        assert_eq!(recv(&mut psram), 0x60, "then EID[47:40]");
        psram.set_selected(false);
        assert_eq!(psram.so(), None, "released on deselect");
        assert_eq!(psram.commands, vec![0x9F]);
    }

    #[test]
    fn a_write_then_a_read_round_trips_through_the_array() {
        let mut psram = Psram::new();
        tx(&mut psram, &[0x02, 0x12, 0x34, 0x56, 0xDE, 0xAD, 0xBE]);
        assert_eq!(psram.writes, vec![(0x12_3456, 3)]);
        assert_eq!(psram.byte(0x12_3456), 0xDE);
        psram.set_selected(true);
        for b in [0x03, 0x12, 0x34, 0x56] {
            send(&mut psram, b);
        }
        assert_eq!(
            [recv(&mut psram), recv(&mut psram), recv(&mut psram)],
            [0xDE, 0xAD, 0xBE]
        );
        assert_eq!(psram.reads, vec![0x12_3456]);
    }

    #[test]
    fn a_fast_read_waits_eight_cycles_before_the_data() {
        let mut psram = Psram::new();
        tx(&mut psram, &[0x02, 0x00, 0x00, 0x00, 0x5A]);
        psram.set_selected(true);
        for b in [0x0B, 0x00, 0x00, 0x00] {
            send(&mut psram, b);
        }
        assert_eq!(psram.so(), None, "wait cycles: nothing out yet");
        send(&mut psram, 0xFF); // the eight wait cycles
        assert_eq!(recv(&mut psram), 0x5A);
    }

    #[test]
    fn a_burst_past_the_end_of_the_array_wraps_to_zero() {
        let mut psram = Psram::new();
        tx(&mut psram, &[0x02, 0x00, 0x00, 0x00, 0x11]);
        tx(&mut psram, &[0x02, 0x7F, 0xFF, 0xFF, 0x22]);
        psram.set_selected(true);
        for b in [0x03, 0x7F, 0xFF, 0xFF] {
            send(&mut psram, b);
        }
        assert_eq!([recv(&mut psram), recv(&mut psram)], [0x22, 0x11]);
    }

    #[test]
    fn entering_quad_mode_silences_the_serial_side_until_a_reset() {
        let mut psram = Psram::new();
        tx(&mut psram, &[0x35]);
        assert!(psram.in_quad_mode());
        psram.set_selected(true);
        for b in [0x9F, 0, 0, 0] {
            send(&mut psram, b);
        }
        assert_eq!(psram.so(), None, "quad mode: no serial answer");
        psram.set_selected(false);
        // A serial reset cannot be decoded in quad mode either.
        tx(&mut psram, &[0x66]);
        tx(&mut psram, &[0x99]);
        assert!(
            psram.in_quad_mode(),
            "a quad-mode part does not hear a serial reset"
        );
        assert_eq!(psram.commands, vec![0x35]);
    }

    #[test]
    fn a_reset_needs_its_enable_first() {
        let mut psram = Psram::new();
        tx(&mut psram, &[0x99]);
        tx(&mut psram, &[0x66]);
        tx(&mut psram, &[0xC0]);
        tx(&mut psram, &[0x99]);
        assert_eq!(psram.commands, vec![0x99, 0x66, 0xC0, 0x99]);
        assert!(
            !psram.reset_enabled,
            "a reset enable is spent by the next command"
        );
    }

    #[test]
    fn the_facades_name_what_attach_looks_up() {
        for pins in [&PSRAM_PINS_BY_FUNCTION[..], &PSRAM_PINS_SOP8[..]] {
            for id in ["/CE", "SCLK", "SI", "SO"] {
                assert!(
                    pins.iter().any(|p| p.number == id || p.name == Some(id)),
                    "{id} is declared"
                );
            }
            let so = pins
                .iter()
                .find(|p| p.number == "SO" || p.name == Some("SO"))
                .unwrap();
            assert_eq!(so.kind, PinKind::DigitalOut);
            assert_eq!(so.idle, IdleDrive::Released);
        }
        let ids: Vec<_> = PSRAM_PINS_BY_FUNCTION.iter().map(|p| p.number).collect();
        assert_eq!(
            ids,
            ["VSS", "VDD", "NC_EP", "SCLK", "CEn", "SI_SIO0", "SO_SIO1", "SIO2", "SIO3"],
            "the P2-EC32MB netlist's identifiers"
        );
    }
}
