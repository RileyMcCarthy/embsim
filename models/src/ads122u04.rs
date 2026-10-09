//! Model: ADS122U04 — TI's 24-bit delta-sigma ADC with a UART interface: its
//! serial protocol, and the register file that sets up every conversion.
//!
//! The model takes commands over a serial pipe, keeps the five configuration
//! registers the host writes, and answers each conversion from them: the
//! input multiplexer chooses what is converted, the gain and the reference set
//! its scale. The analog pins reach it as voltages, each measured against
//! `AVSS` ([`Ads122u04::sense`]); the component that mounts it on a board
//! (`crate::ads122u04_component`) hands it what the engine solves on them.
//!
//! The ADC has no concept of force or sensors — it only knows voltage.
//! Has no knowledge of serial drivers or MCU peripherals. Communicates
//! through an internal pipe: firmware writes to one end, this model reads
//! from the other end and writes responses back.
//!
//! # Datasheet provenance
//!
//! Modeled against **TI SBAS752B (May 2017, revised Oct 2018)** — the
//! ADS122U04 datasheet; section/page citations below are to that revision.
//! Governing sections: §6.3 recommended operating conditions (p.4), §8.3.2
//! the gain stage (p.22–24), §8.3.3 the voltage reference (p.24), §8.4
//! functional modes (p.31–33), §8.5.1 UART interface (p.34), §8.5.2 data
//! format (p.35), §8.5.3 commands (p.36–37), §8.5.4 data read modes
//! (p.37–39), §8.6 register map (p.40–45).
//!
//! # A conversion, from the registers
//!
//! A conversion reads the register file as it stands when the conversion is
//! answered (§8.6.2):
//!
//! - **`MUX[3:0]`** (configuration register 0, bits 7:4; §8.6.2.1 Table 18,
//!   p.41) chooses the input: one of the eight pin pairs, a pin against
//!   `AVSS`, the reference or the analog supply divided by four, or both
//!   inputs shorted to mid-supply.
//! - **`GAIN[2:0]`** (register 0, bits 3:1; Table 18) is a gain of
//!   2^`GAIN`. **`PGA_BYPASS`** (bit 0) leaves it as it is: the PGA can be
//!   bypassed only at gains 1, 2 and 4, which the switched-capacitor stage
//!   provides without it, and stays on at 8 to 128 whatever the bit says
//!   (Table 18; §8.3.2, Table 9, p.22). What the PGA's bypass does change is
//!   the absolute input range, which the model does not police (below). The
//!   multiplexer settings that bypass the PGA themselves — a pin against
//!   `AVSS` (`1000`–`1011`) and the two monitors (`1100`, `1101`) — leave
//!   only the switched-capacitor stage, so a gain above 4 is 4 (§8.3.2.2,
//!   p.24; Table 9's switched-capacitor column).
//! - **`VREF[1:0]`** (configuration register 1, bits 2:1; §8.6.2.2 Table 19,
//!   p.42) is the reference: `00` the internal 2.048 V, `01` the
//!   `REFP` − `REFN` pair, `10` and `11` the analog supply, read as the
//!   sensed `AVDD` − `AVSS` (§8.3.3, p.24).
//! - **`TS`** (register 1, bit 0) is temperature-sensor mode, whose code is
//!   the die temperature (§8.3.10): nothing on a board names that, so the
//!   model holds its last code (below).
//! - **`DR[2:0]`** and **`MODE`** (register 1, bits 7:5 and 4; Table 20,
//!   p.42) pace the automatic data read mode's output, turbo mode at twice
//!   the normal rate; **`CM`** (bit 3) is one conversion per START/SYNC or
//!   conversions without end (§8.4.2, p.32); **`AUTO`** (configuration
//!   register 3, bit 0; §8.6.2.4 Table 22, p.44) sends each conversion
//!   unprompted (§8.5.4.2) instead of on RDATA (§8.5.4.1).
//!
//! Every register is 00h after power-on, the `RESET` pin or the RESET
//! command (§8.6.1, p.40; §8.4.1, p.31): `AIN0` against `AIN1`, gain 1, the
//! PGA on, the internal 2.048 V reference, 20 SPS in normal mode, single-shot,
//! manual data read. A part the host never writes converts as that.
//!
//! The code is §8.5.2's Equation 8 (p.35): `1 LSB = (2 · VREF / Gain) /
//! 2^24`, so `code = VIN · Gain · 2^23 / VREF`, truncated toward zero, and
//! clipped at 7FFFFFh / 800000h ("The output clips at these codes for
//! signals that exceed full-scale").
//!
//! ## No voltage, no new code
//!
//! A pin the engine resolves to no voltage — floating, or never yet sensed —
//! is held at the last voltage it had. When what the registers select still
//! names no voltage (a pin never sensed, a reference below its minimum —
//! 0.75 V for `REFP` − `REFN` and 2.3 V for the analog supply, §6.3 — the
//! reserved multiplexer setting, or temperature-sensor mode), the model
//! answers with the last code it converted, 000000h after a reset. The
//! datasheet names no code for those cases and the model invents none
//! (`DESIGN.md` rule 6); holding is this model's policy.
//!
//! ## Deliberate simplifications (byte-pipe fidelity)
//!
//! - No baud-rate auto-detection (§8.5.1.4): the transport is a byte pipe,
//!   so sync-word timing measurement has nothing to measure. The sync byte
//!   itself IS required before every command, as the datasheet specifies.
//! - No interface idle timeout (§8.5.1.5, ~32760·t_MOD): a real host that
//!   stalls mid-command resets the interface; the model waits forever.
//! - No td(RSRX) post-RESET delay enforcement (§8.5.3.1).
//! - A conversion is the inputs as they are when it is answered: no digital
//!   filter, settling or conversion latency (§8.3.5, §8.3.6), and Table 14's
//!   ideal codes (no noise, INL, offset or gain error but the configured
//!   [`Config::zero_offset`]). RDATA answers so in any state, single-shot and
//!   power-down included. A register write during a conversion does not
//!   restart it (§8.4.2).
//! - The absolute input range (§8.3.2.1, Equations 6 and 7; `AVSS` − 0.1 V
//!   to `AVDD` + 0.1 V with the PGA bypassed, §8.3.2.2) is not policed: an
//!   input outside it converts as if inside.
//! - The registers are stored and read back as written, read-only bits
//!   included; the IDACs, burn-out sources, GPIOs, `DRDY` and the
//!   DCNT/CRC/BCS output framing (§8.6.2.3) are unmodeled.

use std::os::fd::{BorrowedFd, RawFd};
use std::sync::atomic::{AtomicI32, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};

use embsim_board::Volts;
use tracing::{debug, info, trace, warn};

// ============================================================
// ADS122U04 Protocol Constants
// ============================================================

/// Synchronization word — must precede every command (SBAS752B §8.5.1.4,
/// p.34: "The host must always transmit the synchronization word first").
const SYNC_BYTE: u8 = 0x55;
/// RESET = `0000 011x` (SBAS752B §8.5.3 Table 15, p.36).
const CMD_RESET: u8 = 0x06;
/// START/SYNC = `0000 100x` (SBAS752B §8.5.3 Table 15, p.36).
const CMD_START: u8 = 0x08;
/// POWERDOWN = `0000 001x` (SBAS752B §8.5.3 Table 15, p.36).
const CMD_POWERDOWN: u8 = 0x02;

/// Five 8-bit configuration registers, 00h–04h (SBAS752B §8.6.1 Table 16,
/// p.40).
const REGISTER_COUNT: usize = 5;

/// Index of configuration register 0 — `MUX[3:0]` (bits 7:4), `GAIN[2:0]`
/// (bits 3:1), `PGA_BYPASS` (bit 0) (SBAS752B §8.6.2.1 Table 18, p.41).
const REG_CONFIG0: usize = 0;

/// Index of configuration register 1 — `DR[2:0]` (bits 7:5), `MODE` (bit 4),
/// `CM` (bit 3), `VREF[1:0]` (bits 2:1), `TS` (bit 0) (SBAS752B §8.6.2.2
/// Table 19, p.42).
const REG_CONFIG1: usize = 1;

/// Index of configuration register 3 — bit 0 selects automatic (1) vs manual
/// (0) data read mode (SBAS752B §8.6.2.4 Table 22, p.44).
const REG_CONFIG3: usize = 3;

/// Configuration register 1's `CM` bit: 1 is continuous conversion mode, 0
/// single-shot (SBAS752B Table 19, p.42).
const CONFIG1_CM: u8 = 1 << 3;

/// Configuration register 1's `TS` bit: temperature-sensor mode (SBAS752B
/// Table 19, p.42).
const CONFIG1_TS: u8 = 1 << 0;

/// Configuration register 3's `AUTO` bit: automatic data read mode (SBAS752B
/// Table 22, p.44).
const CONFIG3_AUTO: u8 = 1 << 0;

/// The full-scale code count, 2^23: `+FS` is `2^23` LSBs (SBAS752B §8.5.2
/// Equation 8, p.35: `1 LSB = +FS / 2^23`).
const FULL_SCALE_CODES: f64 = 8_388_608.0; // 2^23

/// +FS − 1 LSB, the largest code (7FFFFFh, SBAS752B §8.5.2 Table 14, p.35).
const CODE_MAX: i64 = 0x7F_FFFF;
/// −FS, the smallest code (800000h, SBAS752B §8.5.2 Table 14, p.35).
const CODE_MIN: i64 = -0x80_0000;

/// The internal voltage reference, 2.048 V (SBAS752B §8.3.3, p.24; §6.5
/// `VREF`) — what `VREF[1:0]` = `00`, its reset value, selects (§8.6.2.2
/// Table 19, p.42).
pub const INTERNAL_VREF_VOLTS: Volts = 2.048;

/// The smallest differential reference, `V(REFP) − V(REFN)`: 0.75 V
/// (SBAS752B §6.3 Recommended Operating Conditions, `VREF` min, p.4). Below
/// it the external reference names no conversion.
pub const REFERENCE_MIN_VOLTS: Volts = 0.75;

/// The smallest analog and digital supply, `AVDD − AVSS` and `DVDD − DGND`:
/// 2.3 V (SBAS752B §6.3 Recommended Operating Conditions, p.4). The analog
/// supply as the reference names no conversion below it, and the component's
/// power gate counts a rail below it as down.
pub const ADS122U04_SUPPLY_MIN_VOLTS: Volts = 2.3;

/// The largest gain the switched-capacitor stage gives on its own, 4: what is
/// left when the PGA is bypassed (SBAS752B §8.3.2 Table 9, p.22; §8.3.2.2,
/// p.24: "In case gain is set to greater than 4, the device limits gain to
/// 4").
const SWITCHED_CAPACITOR_GAIN_MAX: f64 = 4.0;

/// The divisor of the two supply monitors, `(V(REFP) − V(REFN)) / 4` and
/// `(AVDD − AVSS) / 4` (SBAS752B §8.6.2.1 Table 18, `MUX` `1100` and `1101`,
/// p.41).
const MONITOR_DIVISOR: f64 = 4.0;

/// Compute the conversion interval (virtual µs) from the contents of CONFIG1.
///
/// CONFIG1 bit layout (SBAS752B §8.6.2.2 Fig. 70, p.42):
///   [7:5] DR   — data rate (Table 20, p.42: 000=20 SPS … 110=1000 SPS)
///   [4]   MODE — 0 = normal, 1 = turbo (512-kHz modulator, doubles the rate)
///
/// Reserved DR code (111) falls back to the fastest configured rate
/// (datasheet marks it Reserved; the fallback is a documented model choice).
fn conversion_interval_us(reg_config1: u8) -> u64 {
    let dr = (reg_config1 >> 5) & 0b111;
    let turbo = ((reg_config1 >> 4) & 0b1) == 1;
    let normal_us: u64 = match dr {
        0b000 => 50_000, // 20 SPS
        0b001 => 22_222, // 45 SPS
        0b010 => 11_111, // 90 SPS
        0b011 => 5_714,  // 175 SPS
        0b100 => 3_030,  // 330 SPS
        0b101 => 1_667,  // 600 SPS
        0b110 => 1_000,  // 1000 SPS
        _ => 1_000,
    };
    if turbo {
        normal_us / 2
    } else {
        normal_us
    }
}

// ============================================================
// The conversion: what the registers select
// ============================================================

/// An analog pin a conversion can read, each a voltage against `AVSS`
/// (SBAS752B p.3 pin functions): the four inputs, the reference pair, and
/// the analog supply.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AnalogPin {
    /// `AIN0`, pin 11.
    Ain0,
    /// `AIN1`, pin 10.
    Ain1,
    /// `AIN2`, pin 7.
    Ain2,
    /// `AIN3`, pin 6.
    Ain3,
    /// `REFP`, pin 9.
    Refp,
    /// `REFN`, pin 8.
    Refn,
    /// `AVDD`, pin 12: the analog supply, `AVDD − AVSS`.
    Avdd,
}

/// How many [`AnalogPin`]s there are.
const ANALOG_PIN_COUNT: usize = 7;

/// The voltage each [`AnalogPin`] last had, against `AVSS`; `None` until it
/// is first sensed.
type PinVolts = [Option<Volts>; ANALOG_PIN_COUNT];

impl AnalogPin {
    const fn index(self) -> usize {
        self as usize
    }
}

/// `MUX[3:0]`, configuration register 0's bits 7:4 (SBAS752B Table 18).
const fn mux(reg_config0: u8) -> u8 {
    reg_config0 >> 4
}

/// The multiplexer settings that bypass the PGA whatever `PGA_BYPASS` says:
/// a pin against `AVSS` (`1000`–`1011`, SBAS752B §8.3.2.2, p.24) and the two
/// monitors (`1100`, `1101`, Table 18's "(PGA bypassed)").
const fn mux_bypasses_pga(mux: u8) -> bool {
    matches!(mux, 0b1000..=0b1101)
}

/// The gain a conversion applies: `GAIN[2:0]` selects 2^`GAIN`, 1 to 128
/// (SBAS752B Table 18). `PGA_BYPASS` changes none of them: gains 1, 2 and 4
/// are the switched-capacitor stage's with or without the PGA, and the PGA
/// is always on at 8 to 128 (Table 18; §8.3.2, Table 9). A multiplexer
/// setting that bypasses the PGA leaves only the switched-capacitor stage,
/// so it limits the gain to 4 (§8.3.2.2).
fn conversion_gain(reg_config0: u8) -> f64 {
    let gain = f64::from(1u8 << ((reg_config0 >> 1) & 0b111));
    if mux_bypasses_pga(mux(reg_config0)) {
        gain.min(SWITCHED_CAPACITOR_GAIN_MAX)
    } else {
        gain
    }
}

/// The differential input `VIN = V(AINP) − V(AINN)` the multiplexer selects
/// (SBAS752B §8.6.2.1 Table 18, p.41), from the pins' voltages against
/// `AVSS`. `None` when a pin it reads has never been sensed, or for the
/// reserved setting.
fn input_volts(mux: u8, pins: &PinVolts) -> Option<Volts> {
    use AnalogPin::{Ain0, Ain1, Ain2, Ain3, Avdd, Refn, Refp};
    let at = |pin: AnalogPin| pins[pin.index()];
    let between = |p: AnalogPin, n: AnalogPin| Some(at(p)? - at(n)?);
    match mux {
        0b0000 => between(Ain0, Ain1),
        0b0001 => between(Ain0, Ain2),
        0b0010 => between(Ain0, Ain3),
        0b0011 => between(Ain1, Ain0),
        0b0100 => between(Ain1, Ain2),
        0b0101 => between(Ain1, Ain3),
        0b0110 => between(Ain2, Ain3),
        0b0111 => between(Ain3, Ain2),
        // AINN = AVSS: the pin's own voltage, which is measured against it.
        0b1000 => at(Ain0),
        0b1001 => at(Ain1),
        0b1010 => at(Ain2),
        0b1011 => at(Ain3),
        0b1100 => Some(between(Refp, Refn)? / MONITOR_DIVISOR),
        0b1101 => Some(at(Avdd)? / MONITOR_DIVISOR),
        // AINP and AINN both at (AVDD + AVSS) / 2: no difference.
        0b1110 => Some(0.0),
        _ => None,
    }
}

/// The reference `VREF[1:0]` selects (SBAS752B §8.6.2.2 Table 19, p.42;
/// §8.3.3, p.24): the internal 2.048 V, `V(REFP) − V(REFN)`, or the analog
/// supply `AVDD − AVSS` as sensed. `None` when the selected pins have never
/// been sensed, or put the reference below its minimum (§6.3).
fn reference_volts(reg_config1: u8, pins: &PinVolts) -> Option<Volts> {
    let at = |pin: AnalogPin| pins[pin.index()];
    match (reg_config1 >> 1) & 0b11 {
        0b00 => Some(INTERNAL_VREF_VOLTS),
        0b01 => Some(at(AnalogPin::Refp)? - at(AnalogPin::Refn)?)
            .filter(|&vref| vref >= REFERENCE_MIN_VOLTS),
        _ => at(AnalogPin::Avdd).filter(|&avdd| avdd >= ADS122U04_SUPPLY_MIN_VOLTS),
    }
}

/// The 24-bit code a conversion gives, from the register file and the pins'
/// voltages: SBAS752B §8.5.2 Equation 8 (p.35), `code = VIN · Gain · 2^23 /
/// VREF`, truncated toward zero, `zero_offset` added, clipped at 7FFFFFh /
/// 800000h. `None` when what the registers select names no voltage
/// (module docs, "No voltage, no new code").
fn conversion_code(
    registers: &[u8; REGISTER_COUNT],
    pins: &PinVolts,
    zero_offset: i32,
) -> Option<i32> {
    let reg_config0 = registers[REG_CONFIG0];
    let reg_config1 = registers[REG_CONFIG1];
    if reg_config1 & CONFIG1_TS != 0 {
        return None;
    }
    let vin = input_volts(mux(reg_config0), pins)?;
    let vref = reference_volts(reg_config1, pins)?;
    let code = (vin * conversion_gain(reg_config0) * FULL_SCALE_CODES) / vref;
    let with_offset = (code as i64) + i64::from(zero_offset);
    Some(with_offset.clamp(CODE_MIN, CODE_MAX) as i32)
}

/// A 24-bit code as the chip sends it: two's complement, least significant
/// byte first (SBAS752B §8.5.3.4 NOTE, p.36: "Data words are transmitted
/// least significant byte first"; §8.5.2, p.35).
fn code_bytes(code: i32) -> [u8; 3] {
    let value = code as u32;
    [
        (value & 0xFF) as u8,
        ((value >> 8) & 0xFF) as u8,
        ((value >> 16) & 0xFF) as u8,
    ]
}

// ============================================================
// Configuration
// ============================================================

/// ADS122U04 model configuration: what no register sets.
#[derive(Debug, Clone, Default)]
pub struct Config {
    /// A code added to every conversion, before clipping — a bench's
    /// measured offset. Zero is Table 14's ideal (SBAS752B §8.5.2, p.35).
    pub zero_offset: i32,
}

// ============================================================
// The device: register file, command parser, conversion state
// ============================================================

/// Parser state for the byte-by-byte protocol.
// justification: the shared `Wait*` prefix is deliberate — each variant names
// what the byte-by-byte parser is *waiting for* next, so the prefix carries
// meaning rather than being noise. Renaming would obscure the state machine.
#[allow(clippy::enum_variant_names)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ParseState {
    /// Waiting for sync byte (0x55)
    WaitSync,
    /// Got sync, waiting for command byte
    WaitCommand,
    /// Writing a register: got sync + command, waiting for data byte
    WaitWriteData { register: usize },
}

/// What the chip sends back for one command.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Reply {
    /// RREG: the register's byte (SBAS752B §8.5.3.5, p.37).
    Register(u8),
    /// RDATA, or a conversion in automatic data read mode: the code.
    Conversion(i32),
}

impl Reply {
    fn write_to(self, fd: RawFd) {
        match self {
            Self::Register(value) => write_bytes(fd, &[value]),
            Self::Conversion(code) => write_bytes(fd, &code_bytes(code)),
        }
    }
}

/// Everything the chip holds: the register file, the command parser, whether
/// it is converting, the pins' last voltages and the last code.
#[derive(Debug)]
struct Device {
    registers: [u8; REGISTER_COUNT],
    parse: ParseState,
    /// START/SYNC has started conversions (SBAS752B §8.4.2, p.32): one in
    /// single-shot mode, without end in continuous mode.
    converting: bool,
    /// The analog pins' voltages, against `AVSS`. The board's, not the
    /// chip's: a reset leaves them.
    pins: PinVolts,
    /// The last code converted, answered while what the registers select
    /// names no voltage.
    held_code: i32,
    zero_offset: i32,
}

impl Device {
    /// The chip after power-on: every register 00h, not converting (SBAS752B
    /// §8.4.1, p.31), no pin sensed yet.
    fn new(zero_offset: i32) -> Self {
        Self {
            registers: [0u8; REGISTER_COUNT],
            parse: ParseState::WaitSync,
            converting: false,
            pins: [None; ANALOG_PIN_COUNT],
            held_code: 0,
            zero_offset,
        }
    }

    /// A reset — power-on, the `RESET` pin or the RESET command — sets every
    /// register to its default, 00h (SBAS752B §8.6.1, p.40), and leaves the
    /// device in its low-power state waiting for START/SYNC (§8.4.1, p.31;
    /// the §8.4 flow chart, p.31). The interface starts again at the
    /// synchronization word.
    fn reset(&mut self) {
        self.registers = [0u8; REGISTER_COUNT];
        self.parse = ParseState::WaitSync;
        self.converting = false;
        self.held_code = 0;
    }

    /// Convert the selected input now: the code the registers and the pins
    /// give, or the held one (module docs, "No voltage, no new code").
    fn convert(&mut self) -> i32 {
        if let Some(code) = conversion_code(&self.registers, &self.pins, self.zero_offset) {
            self.held_code = code;
        }
        self.held_code
    }

    /// The conversion automatic data read mode sends unprompted, if one is
    /// due at `now_us` (SBAS752B §8.5.4.2, p.39: the device "automatically
    /// outputs the latest conversion data … as soon as a conversion
    /// completes"); in manual mode results are fetched with RDATA. The
    /// interval is `DR` and `MODE`'s (Table 20). In single-shot mode the one
    /// conversion START/SYNC began ends the conversions (§8.4.2.1, p.32).
    fn automatic_conversion(&mut self, now_us: u64, last_us: &mut u64) -> Option<i32> {
        let auto_mode = self.registers[REG_CONFIG3] & CONFIG3_AUTO != 0;
        if !(self.converting && auto_mode) {
            return None;
        }
        let reg_config1 = self.registers[REG_CONFIG1];
        if now_us < *last_us + conversion_interval_us(reg_config1) {
            return None;
        }
        *last_us = now_us;
        if reg_config1 & CONFIG1_CM == 0 {
            self.converting = false;
        }
        Some(self.convert())
    }

    /// Process a single received byte, advance the state machine, and say
    /// what the chip answers.
    fn receive(&mut self, byte: u8) -> Option<Reply> {
        let (next, reply) = match self.parse {
            ParseState::WaitSync => {
                if byte == SYNC_BYTE {
                    (ParseState::WaitCommand, None)
                } else {
                    trace!("ADS122U04: discarding non-sync byte 0x{:02x}", byte);
                    (ParseState::WaitSync, None)
                }
            }
            ParseState::WaitCommand => self.command(byte),
            ParseState::WaitWriteData { register } => {
                if register < REGISTER_COUNT {
                    self.registers[register] = byte;
                    debug!("ADS122U04: WREG reg={} val=0x{:02x}", register, byte);
                }
                (ParseState::WaitSync, None)
            }
        };
        self.parse = next;
        reply
    }

    /// The command byte that followed a sync word.
    fn command(&mut self, command: u8) -> (ParseState, Option<Reply>) {
        if command == CMD_RESET {
            // §8.5.3.1 (p.36) + §8.6.1 (p.40): reset restores default
            // register values (all 0) and, per the §8.4 flow chart
            // (p.31), returns the device to the non-converting state.
            debug!("ADS122U04: RESET");
            self.reset();
            return (ParseState::WaitSync, None);
        }
        if command == CMD_START {
            // §8.5.3.2 (p.36): starts a single conversion in single-shot
            // mode, or converting continuously in continuous mode. (Digital-
            // filter restart on repeated START is not modeled.)
            debug!("ADS122U04: START/SYNC");
            self.converting = true;
            return (ParseState::WaitSync, None);
        }
        if command == CMD_POWERDOWN {
            // §8.5.3.3 (p.36): power-down stops conversions but holds
            // all register values.
            debug!("ADS122U04: POWERDOWN");
            self.converting = false;
            return (ParseState::WaitSync, None);
        }
        let upper_nibble = (command >> 4) & 0x0F;
        let register = ((command >> 1) & 0x0F) as usize;
        match upper_nibble {
            0b0001 => {
                // RDATA = `0001 xxxx` (§8.5.3 Table 15, p.36): "loads the
                // output shift register with the most recent conversion
                // result" (§8.5.3.4, p.36) — the manual data read mode
                // fetch (§8.5.4.1, p.38), request/response framed.
                trace!("ADS122U04: RDATA");
                (
                    ParseState::WaitSync,
                    Some(Reply::Conversion(self.convert())),
                )
            }
            0b0010 => {
                // RREG = `0010 rrrx` (§8.5.3 Table 15, p.36): replies
                // one byte; a nonexistent register reads back 00h
                // (§8.5.3.5, p.37).
                let value = self.registers.get(register).copied().unwrap_or(0);
                trace!("ADS122U04: RREG reg={} val=0x{:02x}", register, value);
                (ParseState::WaitSync, Some(Reply::Register(value)))
            }
            0b0100 => {
                // WREG = `0100 rrrx dddd dddd` (§8.5.3 Table 15, p.36):
                // one data byte follows; writes to a nonexistent register
                // are ignored (§8.5.3.6, p.37). (Digital-filter restart
                // on writes to regs 0–3 is not modeled.)
                trace!("ADS122U04: WREG reg={} (waiting for data)", register);
                (ParseState::WaitWriteData { register }, None)
            }
            _ => {
                // A command byte the chip does not recognise did not come
                // from the driver -- the driver only ever sends legal ones.
                // It means the link corrupted a byte, and the visible
                // symptom is a *timeout* in the driver waiting for a reply
                // that was never provoked. That is very hard to diagnose
                // from the far end, so say so rather than hiding it at
                // trace level; on a healthy link this never fires.
                warn!(
                    "ADS122U04: unknown command 0x{:02x} (corrupted on the wire?)",
                    command
                );
                (ParseState::WaitSync, None)
            }
        }
    }
}

// ============================================================
// ADS122U04 instance
// ============================================================

pub struct Ads122u04 {
    /// The register file, parser and conversion state, shared by the
    /// protocol thread and whoever senses the pins.
    device: Mutex<Device>,
    /// Model-side FD (this model reads/writes).
    model_fd: AtomicI32,
}

impl Ads122u04 {
    /// Create a new ADS122U04 model instance, as the chip leaves power-on
    /// reset (every register 00h). Creates an internal pipe pair and starts
    /// the protocol handler thread, which ends when the firmware's end of
    /// the pipe is closed.
    /// Returns `(Arc<Self>, firmware_fd)` — wire firmware_fd to the serial driver.
    pub fn new(config: Config) -> (Arc<Self>, RawFd) {
        info!("ADS122U04: init zero_offset={}", config.zero_offset);

        let (model_fd, firmware_fd) = create_pipe_pair();

        let instance = Arc::new(Self {
            device: Mutex::new(Device::new(config.zero_offset)),
            model_fd: AtomicI32::new(model_fd),
        });

        // Start the protocol handler thread
        let adc = Arc::clone(&instance);
        std::thread::Builder::new()
            .name("ads122u04".into())
            .spawn(move || protocol_loop(&adc))
            .expect("Failed to start ADS122U04 thread");

        debug!(
            "ADS122U04 model initialized (firmware_fd={}, model_fd={})",
            firmware_fd, model_fd
        );
        (instance, firmware_fd)
    }

    /// An analog pin's voltage, against `AVSS`: what the next conversion
    /// that selects the pin reads, until the next call for that pin. This is
    /// the input from whatever mounts the model on a board.
    pub fn sense(&self, pin: AnalogPin, volts: Volts) {
        self.device().pins[pin.index()] = Some(volts);
        trace!("ADS122U04: {:?} = {:.6} V", pin, volts);
    }

    /// A reset by power-on or by the `RESET` pin (SBAS752B §8.4.1.1,
    /// §8.4.1.2, p.32): every register to its default, conversions stopped,
    /// the interface waiting for a synchronization word.
    pub fn reset(&self) {
        self.device().reset();
    }

    fn device(&self) -> MutexGuard<'_, Device> {
        self.device
            .lock()
            .expect("the device state is never poisoned")
    }
}

// ============================================================
// Internal pipe creation
// ============================================================

/// Create a bidirectional pipe pair using socketpair.
/// Returns (model_fd, firmware_fd) as raw file descriptors.
fn create_pipe_pair() -> (RawFd, RawFd) {
    let mut fds = [0i32; 2];
    let ret = unsafe { libc::socketpair(libc::AF_UNIX, libc::SOCK_STREAM, 0, fds.as_mut_ptr()) };
    assert_eq!(ret, 0, "Failed to create ADS122U04 socket pair");

    // Both sides non-blocking: the model polls its end, and the firmware HAL's
    // receive-timeout semantics depend on EAGAIN (a blocking firmware fd would
    // hang a zero-timeout drain/poll forever instead of returning "no data").
    for fd in fds {
        unsafe {
            let flags = libc::fcntl(fd, libc::F_GETFL);
            libc::fcntl(fd, libc::F_SETFL, flags | libc::O_NONBLOCK);
        }
    }

    (fds[0], fds[1])
}

// ============================================================
// Protocol loop
// ============================================================

/// Main protocol loop — runs on its own thread, until the firmware's end of
/// the pipe is closed (end of file on the model's end).
///
/// The thread registers as an `embsim_core::virtual_clock` **actor**: in
/// stepped clock mode the scheduler must not advance virtual time while this
/// loop is mid-iteration, or a conversion could be emitted at an instant that
/// depends on host scheduling (`DETERMINISM.md` T1 §4). Registration is free in
/// free-running mode. Ending at end of file ends the actor with the part: a
/// system that drops the component leaves no actor behind to hold a later
/// system's clock.
///
/// `DETERMINISM.md` T1 §4 recommends option (a) for this loop — fold it onto
/// the engine wheel and delete the thread — and D1 did exactly that for the
/// *component* adapter's output pump. It is deliberately **not** done for this
/// loop: [`Ads122u04`] is a standalone model that owns a socketpair and has no
/// engine handle (the hand-wired consumer path constructs it directly, with no
/// board in the picture). Turning it into a pollable state machine belongs with
/// the in-process transport of Phase D2, which is what removes the fd this loop
/// exists to service. Option (b) — a registered actor parking on the virtual
/// clock — is what D1 ships.
fn protocol_loop(adc: &Ads122u04) {
    let _actor = embsim_core::virtual_clock::register_actor("ads122u04-protocol");
    let model_fd = adc.model_fd.load(Ordering::Relaxed);
    if model_fd < 0 {
        warn!("ADS122U04: model_fd not initialized");
        return;
    }

    let mut last_conversion_us: u64 = 0;

    // SAFETY: `model_fd` is owned by the model's pipe pair and stays open for the
    // life of this loop; the borrow never outlives it.
    let borrowed = unsafe { BorrowedFd::borrow_raw(model_fd) };
    loop {
        // Drain every byte the firmware has sent since the last wake, rather than
        // one per sleep interval. A real ADS122U04 answers RDATA at UART speed, so
        // consuming a single byte per 250 µs throttles the link to ~4 kB/s. A
        // free-running poller (the firmware's force-gauge task issues RDATA
        // back-to-back, bounded only by round-trip latency) then outruns the model
        // and builds an unbounded command backlog: each queued byte adds 250 µs of
        // latency, so after ~4000 bytes the reply lands after the firmware's 1 s
        // read timeout and the gauge is declared unresponsive. Draining keeps
        // latency bounded by the firmware's own request rate.
        let mut ended = false;
        loop {
            let mut byte = [0u8; 1];
            match nix::unistd::read(borrowed, &mut byte) {
                Ok(1) => {
                    // The reply is written with the device unlocked: a full
                    // socket waits for the engine to drain it, and the engine
                    // must be able to sense a pin meanwhile.
                    let reply = adc.device().receive(byte[0]);
                    if let Some(reply) = reply {
                        reply.write_to(model_fd);
                    }
                }
                // End of file: the firmware's end is closed, and the part with it.
                Ok(_) => {
                    debug!("ADS122U04: the firmware's end closed; protocol thread ends");
                    ended = true;
                    break;
                }
                // No data queued: nothing more to consume this wake.
                Err(nix::errno::Errno::EAGAIN) => break,
                Err(e) => {
                    warn!("ADS122U04: read error: {}", e);
                    ended = true;
                    break;
                }
            }
        }
        if ended {
            break;
        }

        let now_us = embsim_core::virtual_clock::virtual_us();
        let due = adc
            .device()
            .automatic_conversion(now_us, &mut last_conversion_us);
        if let Some(code) = due {
            trace!("ADS122U04: automatic conversion {}", code);
            Reply::Conversion(code).write_to(model_fd);
        }

        // Park briefly to avoid busy-looping. Must be substantially finer than
        // the configured conversion interval so we don't miss the 1000 SPS edge
        // (1 ms period). 250 µs gives at least 4× oversample at the fastest
        // non-turbo rate the firmware supports. The cadence is virtual because
        // the interval it oversamples is — and in stepped mode this park is
        // where the registered actor above releases the quiescence barrier.
        embsim_core::virtual_clock::wait_virtual_us(250);
    }
    // This loop is the model's end's only reader and writer, and it is done.
    adc.model_fd.store(-1, Ordering::Relaxed);
    // SAFETY: `model_fd` came from `create_pipe_pair` and is closed once, here.
    unsafe { libc::close(model_fd) };
}

/// Write bytes to the model's FD.
fn write_bytes(fd: RawFd, data: &[u8]) {
    let borrowed = unsafe { BorrowedFd::borrow_raw(fd) };
    let mut written = 0;
    while written < data.len() {
        match nix::unistd::write(borrowed, &data[written..]) {
            Ok(n) => written += n,
            Err(nix::errno::Errno::EAGAIN) => {
                std::thread::yield_now();
            }
            Err(e) => {
                warn!("ADS122U04: write error: {}", e);
                break;
            }
        }
    }
}

// ============================================================
// Tests
// ============================================================
//
// These exercise the PRIVATE pure logic (`conversion_interval_us`,
// `conversion_code` and its parts) and the PRIVATE device state machine
// (`Device::receive`, `Device::automatic_conversion`) directly, driving bytes
// by hand and inspecting the register file and the replies. The live
// `protocol_loop` thread is exercised by the back-to-back regression; the
// proving tests on a board are `board/tests/ads122u04_registers.rs`.

#[cfg(test)]
mod tests {
    use rstest::rstest;

    use super::*;

    /// A reference of exactly 2 V (`REFP` 2.5 V, `REFN` 0.5 V, both exact in
    /// binary), so `2^23 / VREF` is `2^22` and the arithmetic stays exact.
    const EXACT_REFP: Volts = 2.5;
    const EXACT_REFN: Volts = 0.5;

    /// Configuration register 1 selecting the external reference
    /// (`VREF[1:0]` = `01`, Table 19).
    const CONFIG1_VREF_EXTERNAL: u8 = 0b01 << 1;
    /// Configuration register 1 selecting the analog supply
    /// (`VREF[1:0]` = `10`, Table 19).
    const CONFIG1_VREF_AVDD: u8 = 0b10 << 1;

    /// A device with every pin sensed: `AIN0`..`AIN3` at the given volts,
    /// the exact 2 V reference pair, and `AVDD` at 3.3 V.
    fn device_with(ain: [Volts; 4], zero_offset: i32) -> Device {
        let mut device = Device::new(zero_offset);
        for (pin, volts) in [
            (AnalogPin::Ain0, ain[0]),
            (AnalogPin::Ain1, ain[1]),
            (AnalogPin::Ain2, ain[2]),
            (AnalogPin::Ain3, ain[3]),
            (AnalogPin::Refp, EXACT_REFP),
            (AnalogPin::Refn, EXACT_REFN),
            (AnalogPin::Avdd, 3.3),
        ] {
            device.pins[pin.index()] = Some(volts);
        }
        device
    }

    /// Feed bytes; return every reply, in order.
    fn feed(device: &mut Device, bytes: &[u8]) -> Vec<Reply> {
        bytes.iter().filter_map(|&b| device.receive(b)).collect()
    }

    fn wreg(register: u8, value: u8) -> [u8; 3] {
        [SYNC_BYTE, 0x40 | (register << 1), value]
    }

    fn rdata(device: &mut Device) -> i32 {
        match feed(device, &[SYNC_BYTE, 0x10])[..] {
            [Reply::Conversion(code)] => code,
            ref other => panic!("RDATA answers one conversion, got {other:?}"),
        }
    }

    /// Read from `fd` until `n` bytes arrive or `deadline` elapses.
    fn read_n_until(fd: RawFd, n: usize, deadline: std::time::Duration) -> Vec<u8> {
        let mut out = Vec::with_capacity(n);
        let mut buf = [0u8; 1024];
        // SAFETY: `fd` is kept open by the caller's pipe pair for the whole read.
        let borrowed = unsafe { BorrowedFd::borrow_raw(fd) };
        let start = std::time::Instant::now();
        while out.len() < n && start.elapsed() < deadline {
            match nix::unistd::read(borrowed, &mut buf) {
                Ok(0) => break,
                Ok(k) => out.extend_from_slice(&buf[..k]),
                Err(nix::errno::Errno::EAGAIN) => std::thread::yield_now(),
                Err(_) => break,
            }
        }
        out
    }

    /// REGRESSION: the loop must consume everything the firmware has sent since
    /// the last wake, not one byte per sleep interval.
    ///
    /// Firmware polls this chip in manual-read mode — it issues SYNC+RDATA
    /// back-to-back, bounded only by round-trip latency, never by a timer. When
    /// the loop took a single byte per 250 µs sleep the link was throttled to
    /// ~4 kB/s, so a free-running poller outran the model and every queued byte
    /// added another 250 µs of latency. The backlog grows without bound: after a
    /// few thousand bytes the reply to the *current* request arrives later than
    /// the firmware's read timeout, the gauge is declared unresponsive, and a
    /// running test is aborted by the resulting fault.
    ///
    /// 800 frames is 1600 request bytes = 400 ms of pure sleeping under the old
    /// behaviour, so the 150 ms deadline cannot be met by anything that paces
    /// itself per byte, while draining answers them in a couple of wakes.
    #[test]
    fn back_to_back_rdata_is_answered_without_falling_behind() {
        const FRAMES: usize = 800;
        const RDATA: u8 = 0x10;

        let (_adc, fw_fd) = Ads122u04::new(Config::default());

        let mut req = Vec::with_capacity(FRAMES * 2);
        for _ in 0..FRAMES {
            req.extend_from_slice(&[SYNC_BYTE, RDATA]);
        }
        write_bytes(fw_fd, &req);

        let got = read_n_until(fw_fd, FRAMES * 3, std::time::Duration::from_millis(150));
        assert_eq!(
            got.len(),
            FRAMES * 3,
            "every RDATA must be answered promptly: got {} of {} bytes — the model \
             is pacing itself per byte and falling behind the poller",
            got.len(),
            FRAMES * 3
        );
    }

    /// Closing the firmware's end ends the protocol thread, and with it the
    /// model's last reference held there.
    #[test]
    fn the_protocol_thread_ends_when_the_firmware_end_closes() {
        let (adc, fw_fd) = Ads122u04::new(Config::default());
        // SAFETY: `fw_fd` was handed to this test alone and is closed once.
        unsafe { libc::close(fw_fd) };
        let start = std::time::Instant::now();
        while Arc::strong_count(&adc) > 1 && start.elapsed() < std::time::Duration::from_secs(10) {
            std::thread::sleep(std::time::Duration::from_millis(1));
        }
        assert_eq!(
            Arc::strong_count(&adc),
            1,
            "the protocol thread must drop its handle on end of file"
        );
    }

    // ── conversion_interval_us: DR table + turbo halving ──

    /// Every data-rate code maps to the datasheet interval, the reserved code
    /// (111) falls back to the fastest rate, and bit 4 (turbo) halves it.
    #[rstest]
    fn conversion_interval_full_dr_table() {
        // (DR code in bits 7:5) → expected normal-mode interval (µs).
        let table: [(u8, u64); 8] = [
            (0b000, 50_000),
            (0b001, 22_222),
            (0b010, 11_111),
            (0b011, 5_714),
            (0b100, 3_030),
            (0b101, 1_667),
            (0b110, 1_000),
            (0b111, 1_000), // reserved → fastest fallback
        ];
        for (dr, expected) in table {
            let reg = dr << 5; // turbo bit (4) clear
            assert_eq!(
                conversion_interval_us(reg),
                expected,
                "DR {dr:#05b} normal-mode interval"
            );
            // Set the turbo bit (bit 4): interval halves.
            let turbo_reg = reg | (1 << 4);
            assert_eq!(
                conversion_interval_us(turbo_reg),
                expected / 2,
                "DR {dr:#05b} turbo-mode interval"
            );
        }
    }

    /// Turbo bit in isolation halves the default (DR=000) interval and the
    /// lowest data rate, independent of the other low bits which must be
    /// ignored.
    #[rstest]
    fn conversion_interval_ignores_low_bits() {
        // DR=000, turbo=0, but assorted junk in bits 3:0 — must not matter.
        assert_eq!(conversion_interval_us(0b0000_1111), 50_000);
        assert_eq!(conversion_interval_us(0b0000_0111), 50_000);
        // DR=000, turbo=1, junk low bits.
        assert_eq!(conversion_interval_us(0b0001_0101), 25_000);
    }

    // ── conversion_code: the registers' transfer function ──

    /// At reset (every register 00h) a conversion is `AIN0 − AIN1` at gain 1
    /// against the internal 2.048 V reference.
    #[rstest]
    fn at_reset_the_code_is_ain0_minus_ain1_over_the_internal_reference() {
        let mut device = device_with([1.65 + 0.008, 1.65, 0.0, 0.0], 0);
        let vin = (1.65 + 0.008) - 1.65;
        let expected = ((vin * FULL_SCALE_CODES) / INTERNAL_VREF_VOLTS) as i32;
        assert_eq!(rdata(&mut device), expected);
        assert!(
            (expected - 32_768).abs() <= 1,
            "8 mV is ~2^15 codes, got {expected}"
        );
    }

    /// Each `GAIN[2:0]` multiplies the code by 2^`GAIN` (Table 18), against
    /// an exact 2 V reference so every product is exact.
    #[rstest]
    #[case::g1(0b000, 1)]
    #[case::g2(0b001, 2)]
    #[case::g4(0b010, 4)]
    #[case::g8(0b011, 8)]
    #[case::g16(0b100, 16)]
    #[case::g32(0b101, 32)]
    #[case::g64(0b110, 64)]
    #[case::g128(0b111, 128)]
    fn gain_multiplies_the_code(#[case] gain_code: u8, #[case] gain: i32) {
        // VIN 2^-10 V against 2 V: 2^-10 · 2^23 / 2 = 4096 codes at gain 1.
        let mut device = device_with([1.0 + 1.0 / 1024.0, 1.0, 0.0, 0.0], 0);
        feed(&mut device, &wreg(1, CONFIG1_VREF_EXTERNAL));
        feed(&mut device, &wreg(0, gain_code << 1));
        assert_eq!(rdata(&mut device), 4_096 * gain);
        // PGA_BYPASS changes no gain.
        feed(&mut device, &wreg(0, (gain_code << 1) | 1));
        assert_eq!(rdata(&mut device), 4_096 * gain, "PGA_BYPASS set");
    }

    /// Every pin pair, pin-against-AVSS, monitor and the mid-supply short
    /// reads what Table 18 says (at gain 1, so every input is inside full
    /// scale and each reads its own code).
    #[rstest]
    #[case::ain0_ain1(0b0000, 1.0 - 0.5)]
    #[case::ain0_ain2(0b0001, 1.0 - 0.25)]
    #[case::ain0_ain3(0b0010, 1.0 - 0.125)]
    #[case::ain1_ain0(0b0011, 0.5 - 1.0)]
    #[case::ain1_ain2(0b0100, 0.5 - 0.25)]
    #[case::ain1_ain3(0b0101, 0.5 - 0.125)]
    #[case::ain2_ain3(0b0110, 0.25 - 0.125)]
    #[case::ain3_ain2(0b0111, 0.125 - 0.25)]
    #[case::ain0_avss(0b1000, 1.0)]
    #[case::ain1_avss(0b1001, 0.5)]
    #[case::ain2_avss(0b1010, 0.25)]
    #[case::ain3_avss(0b1011, 0.125)]
    #[case::ref_monitor(0b1100, (EXACT_REFP - EXACT_REFN) / 4.0)]
    #[case::avdd_monitor(0b1101, 3.3 / 4.0)]
    #[case::shorted(0b1110, 0.0)]
    fn the_multiplexer_selects_its_input(#[case] mux: u8, #[case] vin: Volts) {
        let mut device = device_with([1.0, 0.5, 0.25, 0.125], 0);
        feed(&mut device, &wreg(1, CONFIG1_VREF_EXTERNAL));
        feed(&mut device, &wreg(0, mux << 4));
        let code = ((vin * FULL_SCALE_CODES) / (EXACT_REFP - EXACT_REFN)) as i32;
        assert!(
            code.abs() < 0x7F_FFFF,
            "MUX {mux:#06b} stays inside full scale"
        );
        assert_eq!(rdata(&mut device), code, "MUX {mux:#06b}");
    }

    /// A bypassing multiplexer setting limits a gain above 4 to 4 and leaves
    /// 1, 2 and 4 as they are (§8.3.2.2).
    #[rstest]
    fn a_pga_bypassing_input_limits_the_gain_to_four() {
        for (gain_code, gain) in [
            (0b000u8, 1.0),
            (0b001, 2.0),
            (0b010, 4.0),
            (0b011, 4.0),
            (0b111, 4.0),
        ] {
            assert_eq!(conversion_gain((0b1000 << 4) | (gain_code << 1)), gain);
            assert_eq!(conversion_gain(gain_code << 1), f64::from(1u8 << gain_code));
        }
    }

    /// `VREF[1:0]` = `10` and `11` both convert against the sensed analog
    /// supply.
    #[rstest]
    #[case::vref_10(0b10)]
    #[case::vref_11(0b11)]
    fn the_analog_supply_reference_is_the_sensed_supply(#[case] vref: u8) {
        let mut device = device_with([1.0 + 1.0 / 1024.0, 1.0, 0.0, 0.0], 0);
        device.pins[AnalogPin::Avdd.index()] = Some(4.0);
        feed(&mut device, &wreg(1, vref << 1));
        // 2^-10 · 2^23 / 4 = 2048.
        assert_eq!(rdata(&mut device), 2_048);
    }

    /// Nothing selected names a voltage: the code is held — 000000h before
    /// the first conversion, the last one after.
    #[rstest]
    fn a_selection_that_names_no_voltage_holds_the_last_code() {
        let mut device = Device::new(0);
        assert_eq!(rdata(&mut device), 0, "no pin sensed yet: 000000h");
        device.pins[AnalogPin::Ain0.index()] = Some(1.0 + 1.0 / 1024.0);
        assert_eq!(rdata(&mut device), 0, "AIN1 never sensed: still held");
        device.pins[AnalogPin::Ain1.index()] = Some(1.0);
        let live = rdata(&mut device);
        assert_ne!(live, 0);
        // The external reference, its pins never sensed: held.
        feed(&mut device, &wreg(1, CONFIG1_VREF_EXTERNAL));
        assert_eq!(rdata(&mut device), live);
        // A reference pair below 0.75 V: held.
        device.pins[AnalogPin::Refp.index()] = Some(1.0);
        device.pins[AnalogPin::Refn.index()] = Some(0.5);
        assert_eq!(rdata(&mut device), live);
        // The analog supply below 2.3 V: held.
        device.pins[AnalogPin::Avdd.index()] = Some(2.0);
        feed(&mut device, &wreg(1, CONFIG1_VREF_AVDD));
        assert_eq!(rdata(&mut device), live);
        // Temperature-sensor mode: held.
        feed(&mut device, &wreg(1, CONFIG1_TS));
        assert_eq!(rdata(&mut device), live);
        // The reserved multiplexer setting: held.
        feed(&mut device, &wreg(1, 0));
        feed(&mut device, &wreg(0, 0b1111 << 4));
        assert_eq!(rdata(&mut device), live);
        // A reset clears the held code.
        feed(&mut device, &[SYNC_BYTE, CMD_RESET]);
        device.pins[AnalogPin::Ain1.index()] = None;
        assert_eq!(rdata(&mut device), 0);
    }

    /// Zero volts maps exactly to the configured zero offset, which is added
    /// before clipping.
    #[rstest]
    fn the_zero_offset_is_added_before_clipping() {
        let mut device = device_with([1.0, 1.0, 0.0, 0.0], 1_234);
        assert_eq!(rdata(&mut device), 1_234);
        let mut device = device_with([1.0, 1.0 - 2.0 / 1024.0, 0.0, 0.0], 5_000);
        feed(&mut device, &wreg(1, CONFIG1_VREF_EXTERNAL));
        // 2^-9 V against 2 V: 2^-9 · 2^22 = 8192 codes, then the offset.
        assert_eq!(rdata(&mut device), 8_192 + 5_000);
        let mut device = device_with([1.0, 1.0, 0.0, 0.0], 0x7F_FFFF);
        device.pins[AnalogPin::Ain0.index()] = Some(1.5);
        assert_eq!(rdata(&mut device), 0x7F_FFFF, "clipped, not wrapped");
    }

    /// SBAS752B §8.5.2 (p.35): "A positive full-scale input … produces an
    /// output code of 7FFFFFh and a negative full-scale input … 800000h. The
    /// output clips at these codes for signals that exceed full-scale."
    #[rstest]
    fn the_code_clips_at_full_scale() {
        let mut device = device_with([3.0, 0.0, 0.0, 0.0], 0);
        assert_eq!(rdata(&mut device), 0x7F_FFFF);
        let mut device = device_with([0.0, 3.0, 0.0, 0.0], 0);
        assert_eq!(rdata(&mut device), -0x80_0000);
    }

    // ── Device::receive: protocol state machine ──

    /// SYNC then START starts conversions.
    #[rstest]
    fn sync_then_start_starts_converting() {
        let mut device = Device::new(0);
        device.receive(SYNC_BYTE);
        assert!(!device.converting);
        device.receive(CMD_START);
        assert!(device.converting, "START must start conversions");
    }

    /// RESET stops conversions and zeroes all registers; the pins stay.
    #[rstest]
    fn reset_clears_converting_and_registers() {
        let mut device = device_with([1.0, 0.5, 0.0, 0.0], 0);
        device.registers = [0xAA; REGISTER_COUNT];
        device.converting = true;
        feed(&mut device, &[SYNC_BYTE, CMD_RESET]);
        assert!(!device.converting, "RESET stops conversions");
        assert_eq!(
            device.registers, [0u8; REGISTER_COUNT],
            "RESET zeroes registers"
        );
        assert_eq!(device.pins[AnalogPin::Ain0.index()], Some(1.0));
    }

    /// The pin and power-on reset do what the command does, and start the
    /// interface at a sync word: a command byte that was waiting for its
    /// data is forgotten.
    #[rstest]
    fn the_pin_reset_restores_defaults_and_the_interface() {
        let (adc, _fw) = Ads122u04::new(Config::default());
        {
            let mut device = adc.device();
            feed(&mut device, &wreg(0, 0x0E));
            feed(&mut device, &[SYNC_BYTE, CMD_START, SYNC_BYTE, 0x42]);
            assert_eq!(device.parse, ParseState::WaitWriteData { register: 1 });
        }
        adc.reset();
        let device = adc.device();
        assert_eq!(device.registers, [0u8; REGISTER_COUNT]);
        assert!(!device.converting);
        assert_eq!(device.parse, ParseState::WaitSync);
    }

    /// POWERDOWN stops conversions (registers untouched).
    #[rstest]
    fn powerdown_stops_converting() {
        let mut device = Device::new(0);
        device.registers = [0x11; REGISTER_COUNT];
        device.converting = true;
        feed(&mut device, &[SYNC_BYTE, CMD_POWERDOWN]);
        assert!(!device.converting, "POWERDOWN stops conversions");
        assert_eq!(
            device.registers, [0x11; REGISTER_COUNT],
            "POWERDOWN keeps registers"
        );
    }

    /// WREG writes the data byte into the register selected by `(cmd>>1)&0xF`.
    #[rstest]
    fn wreg_writes_selected_register() {
        let mut device = Device::new(0);
        feed(&mut device, &wreg(2, 0x9C));
        assert_eq!(device.registers[2], 0x9C);
        assert_eq!(device.registers[0], 0x00, "other registers untouched");
        feed(&mut device, &wreg(1, 0x07));
        feed(&mut device, &wreg(0, 0x33));
        assert_eq!(device.registers, [0x33, 0x07, 0x9C, 0, 0]);
    }

    /// RREG answers the register's byte and returns to WaitSync; an
    /// out-of-range register reads 00h.
    #[rstest]
    fn rreg_answers_the_register_and_recovers() {
        let mut device = Device::new(0);
        device.registers[3] = 0x5A;
        assert_eq!(
            feed(&mut device, &[SYNC_BYTE, 0x26]),
            [Reply::Register(0x5A)]
        );
        assert_eq!(device.parse, ParseState::WaitSync);
        let out_of_range = 0x20 | ((REGISTER_COUNT as u8) << 1);
        assert_eq!(
            feed(&mut device, &[SYNC_BYTE, out_of_range]),
            [Reply::Register(0)]
        );
        feed(&mut device, &[SYNC_BYTE, CMD_START]);
        assert!(device.converting, "the parser recovered");
    }

    /// WREG to an out-of-range register index is ignored.
    #[rstest]
    fn wreg_invalid_register_is_ignored() {
        let mut device = Device::new(0);
        feed(&mut device, &wreg(REGISTER_COUNT as u8, 0xFF));
        assert_eq!(device.registers, [0u8; REGISTER_COUNT]);
        assert_eq!(device.parse, ParseState::WaitSync);
    }

    /// Without a preceding SYNC, a command byte is discarded.
    #[rstest]
    fn non_sync_byte_stays_in_waitsync() {
        let mut device = Device::new(0);
        feed(&mut device, &[0x00, CMD_START]);
        assert!(!device.converting);
        assert_eq!(device.parse, ParseState::WaitSync);
    }

    /// An unknown command returns to WaitSync without side effects.
    #[rstest]
    fn unknown_command_returns_to_waitsync() {
        let mut device = Device::new(0);
        assert!(feed(&mut device, &[SYNC_BYTE, 0xF0]).is_empty());
        assert!(!device.converting);
        assert_eq!(device.registers, [0u8; REGISTER_COUNT]);
        feed(&mut device, &[SYNC_BYTE, CMD_START]);
        assert!(device.converting);
    }

    // ── automatic data read mode ──

    /// In automatic data read mode a continuous conversion is sent every
    /// data-rate interval; in single-shot mode START sends exactly one.
    #[rstest]
    #[case::continuous(CONFIG1_CM, 3)]
    #[case::single_shot(0, 1)]
    fn automatic_mode_sends_conversions_at_the_data_rate(#[case] cm: u8, #[case] expected: usize) {
        let mut device = device_with([1.0 + 1.0 / 1024.0, 1.0, 0.0, 0.0], 0);
        // 1000 SPS (DR 110), the external reference, AUTO.
        feed(
            &mut device,
            &wreg(1, (0b110 << 5) | cm | CONFIG1_VREF_EXTERNAL),
        );
        feed(&mut device, &wreg(3, CONFIG3_AUTO));
        let mut last = 0;
        assert_eq!(
            device.automatic_conversion(1_000, &mut last),
            None,
            "not started"
        );
        feed(&mut device, &[SYNC_BYTE, CMD_START]);
        let sent: Vec<i32> = (1..=3)
            .filter_map(|ms| device.automatic_conversion(ms * 1_000, &mut last))
            .collect();
        assert_eq!(sent, vec![4_096; expected]);
        // Within one interval of the last: nothing.
        assert_eq!(device.automatic_conversion(3_500, &mut last), None);
    }

    /// Manual data read mode sends nothing unprompted.
    #[rstest]
    fn manual_mode_sends_nothing_unprompted() {
        let mut device = device_with([1.0, 0.5, 0.0, 0.0], 0);
        feed(&mut device, &wreg(1, CONFIG1_CM));
        feed(&mut device, &[SYNC_BYTE, CMD_START]);
        let mut last = 0;
        assert_eq!(device.automatic_conversion(100_000, &mut last), None);
    }

    // ── code_bytes: little-endian 24-bit packing ──

    /// The low, middle, then high byte of the 24-bit code; only the low 24
    /// bits go on the wire.
    #[rstest]
    #[case::positive(0x12_3456, [0x56, 0x34, 0x12])]
    #[case::zero(0, [0, 0, 0])]
    #[case::negative(-1, [0xFF, 0xFF, 0xFF])]
    #[case::wide(0xAABB_CCDDu32 as i32, [0xDD, 0xCC, 0xBB])]
    fn codes_go_least_significant_byte_first(#[case] code: i32, #[case] bytes: [u8; 3]) {
        assert_eq!(code_bytes(code), bytes);
    }

    /// `Config` derives `Clone`/`Debug`/`Default`.
    #[rstest]
    fn config_clone_debug_default() {
        let c = Config { zero_offset: -7 };
        assert_eq!(c.clone().zero_offset, -7);
        assert_eq!(Config::default().zero_offset, 0);
        assert!(format!("{c:?}").contains("zero_offset"));
    }
}
