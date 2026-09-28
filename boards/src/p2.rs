//! The Propeller 2 **package** as a node: [`P2Package`], the 86-pin
//! `P2X8C4M64P` facade around a *core* that runs the chip.
//!
//! The package is what a board sees: 64 I/O pads, the core and bank
//! supplies, reset and test, and the crystal pair. It declares those pins
//! once, delivers what the board puts on them to the core, and puts on the
//! nets what the core drives. The core is whatever executes instructions —
//! QEMU's target (`embsim-p2-qemu`), an instruction-set simulator, or the
//! native firmware image behind [`McuComponent`] — and it never sees a net:
//! it is handed [`P2Pads`], which is exactly the surface `NODES.md` §11
//! gives a node, narrowed to the pads.
//!
//! A P2 with **no** core is a real state too: every pad released, the
//! rails and `RESN` sensed, `XI` accepting the board's rate. That is what
//! any P2 is before it runs, and [`P2Package::held_in_reset`] is it — the
//! node that fills a module's processor slot for tests about the board.
//!
//! # What the package declares (`NODES.md` §2, "MCU node (P2)")
//!
//! | Pins | Kind | Idle |
//! |---|---|---|
//! | `P0`..`P63` | `DigitalBidir` | **released** — a pad out of reset floats |
//! | `VDD`, `GND`, the 16 `VIO_a_b` | `PowerIn` (sensed) | — |
//! | `RESN`, `TEST` | `DigitalIn` (`RESN` sensed) | — |
//! | `XI` | `DigitalIn` + [`StreamRole::PulseSink`]: the rate delivered here **is the crystal** | — |
//! | `XO` | `DigitalOut`, released | the crystal driver, unused with an external clock |
//!
//! # The START gate
//!
//! A core does not run until the chip can: the package holds the core's
//! start — [`P2Core::start`] and every wake the core asks for through
//! [`P2Pads`] — until `RESN` reads released **and** `VDD` reads a voltage
//! inside the datasheet's core-supply window ([`P2_VDD_MIN_VOLTS`] to
//! [`P2_VDD_MAX_VOLTS`]). The instant both hold is the START instant:
//! the core is started there (its clock counts from it), its held wakes
//! land there, and [`P2PackageHandle::start_state`] reports it. While the
//! gate is closed the handle reports `Held` with the two inputs as last
//! read — the reason — and the package says so once at `tracing::info`
//! level (the shape the crystal stall in `embsim-p2-qemu` reports in). A
//! `VDD` that names a level and no voltage (a strong digital source, a
//! pull) is outside the window: the package invents no voltage for it.
//! The gate is the package's, so the QEMU core, an ISS and the native
//! firmware image are all held the same way; the reset state a core
//! records through [`P2Pads::on_reset`] is information, never its own
//! gate.
//!
//! The gate also waits for the package's own knowledge of the banks: it
//! does not open until every one of the sixteen `VIO_a_b` senses has
//! delivered at least once ([`BankSupplies::all_delivered`]), so a core
//! that publishes a pad at its START instant reads a populated bank table,
//! never the "nothing has told me yet" every bank starts in. The senses
//! deliver once at registration, on the engine thread, in registration
//! order, while `System::start` runs [`Component::start`] on the caller's
//! thread; the package registers the sixteen bank senses **before** the
//! two the gate reads, so the gate opens at the reset delivery with the
//! table already told, and it checks the table as well, so neither thread
//! can start a core against an empty one (without both, one run in five
//! of `p2_package` read an unpowered bank at START; the phase-4 review
//! record in `NODES.md` §8 has the measurements). Every bank delivery
//! gives the gate its chance, so the condition can never stall it.
//!
//! # Pads drive high at their bank's supply
//!
//! Each `VIO_a_b` pin powers the four pads `a..=b` ([`PADS_PER_BANK`]),
//! and a pad driven high is a source at **that** pin's voltage, whatever
//! it reads — 3.3 V on the P2-EC32MB, whose LDOs source every bank; a
//! bench rail at any other voltage — never a nominal figure. The package
//! senses the sixteen bank pins and keeps them in a [`BankSupplies`]
//! table a core reads through [`P2Pads::bank_supplies`] when it publishes
//! a pad. A bank whose supply pin reaches **no voltage** powers no driver:
//! a pad the core drives there, high or low, presents nothing to its net
//! (released), and the package reports the bank once, by its supply pin
//! (`tracing::warn`, and [`P2PackageHandle::unpowered_banks_driven`]).
//! The build already names such a pin: an unsourced `VIO_a_b` is a
//! [`embsim_board::Finding::PowerNetUnsourced`] on its net.
//!
//! # Pad drive strength (the `WRPIN` pin-configuration field)
//!
//! A pad is a Thevenin source at the strength the guest configured. The
//! `WRPIN` word's pin-configuration field `%PPPPPPPPPPPPP` (bits 20:8)
//! is, in its logic modes, `%0000_CIO_HHH_LLL`: `HHH` (bits 13:11) is the
//! drive while `OUT` = 1 and `LLL` (bits 10:8) the drive while `OUT` = 0,
//! each one of
//!
//! | field | mode | here |
//! |---|---|---|
//! | `%000` | fast | [`P2_FAST_OHMS`] (a named placeholder, see its TODO) |
//! | `%001` | 1.5 kΩ | [`P2_PULL_1K5_OHMS`] |
//! | `%010` | 15 kΩ | [`P2_PULL_15K_OHMS`] |
//! | `%011` | 150 kΩ | [`P2_PULL_150K_OHMS`] |
//! | `%100` | 1 mA | not mapped ([`PadDrive::CurrentSource`]) |
//! | `%101` | 100 µA | not mapped |
//! | `%110` | 10 µA | not mapped |
//! | `%111` | float | released |
//!
//! Source: Parallax, *Propeller 2 (P2X8C4M64P) Silicon Documentation*,
//! rev. v35, "Smart Pins" → "Pin Configuration Modes" — the
//! `%0000_CIO_HHH_LLL` logic-mode word and its `HHH`/`LLL` drive table
//! (fast, 1.5k, 15k, 150k, 1mA, 100uA, 10uA, float). The three resistive
//! modes are the resistances the document names; the current-source modes
//! are **not** mapped to a resistor (a V/I approximation would be an
//! invention, `DESIGN.md` rule 6) and reach a core as
//! [`PadDrive::CurrentSource`], an unimplemented path the core logs once.
//! `Drive::Current` is the encoding they take when a caller exists.
//!
//! # Datasheet
//!
//! The supply figures cite Parallax, *Propeller 2 (P2X8C4M64P) Datasheet*,
//! © Parallax Inc. 2022/11/01 (`Propeller2-P2X8C4M64P-Datasheet-20221101.pdf`):
//! "System Characteristics" → "DC Characteristics" (p. 47) for the `Vdd`
//! window, and "Pin Descriptions" (p. 6) for the bank grouping and the
//! `RESN` pin.
//!
//! # What this package does not do yet
//!
//! - **The restart delay after reset.** The datasheet's `RESN` row (Pin
//!   Descriptions, p. 6) says the chip "restarts 3 ms after RESn
//!   transitions from low to high". The gate opens the instant its
//!   conditions hold, with no delay: the package has no wake of its own
//!   on every core (a native core registers its wake handler directly on
//!   the net I/O), so the delay would hold one kind of core and not
//!   another. `NODES.md` §8, the phase-4 record, carries the decision.
//! - **A rail dropping mid-run.** A `VDD` that leaves its window or a
//!   `RESN` that falls after the START instant is delivered to the core
//!   as a reset state and changes nothing else: no core has a reset entry
//!   yet, and a pad change would be an invention. The plan's
//!   `Finding::BrownoutWithoutReset` lands with that entry.
//! - **A bank supply changing after START.** A `VIO_a_b` that rises,
//!   drops or moves once the core runs updates the [`BankSupplies`] table
//!   and nothing else: a pad the core already drives keeps the drive it
//!   published, at the old voltage, until the core next publishes it (a
//!   guest's next pad write reads the table then). Re-publishing every
//!   driven pad of the bank at the new voltage is the same entry point
//!   as the native core's pads, phase 5's.
//! - **The native core's pads** ([`McuComponent`]) drive through the HAL
//!   bridges at the crate's nominal [`LOGIC_HIGH_VOLTS`], not through
//!   [`BankSupplies`]: the bridges publish on the pads' handles directly.
//!   Phase 5's interface (`NODES.md` §11) routes every drive through one
//!   entry point, where the bank voltage applies to it too.

use std::sync::atomic::{AtomicU16, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

use embsim_board::{
    level_of, AttachError, Component, ComponentNetIo, IdleDrive, Level, McuComponent, NetState,
    Ohms, PinDecl, PinHandle, PinKind, PulseTrain, StreamRole, TheveninDrive, Volts,
};
use embsim_core::virtual_clock;

/// The crate's nominal logic high, [`embsim_board::net::LOGIC_HIGH_VOLTS`]:
/// what the native core's HAL bridges drive at (see the module docs). A
/// pad a core publishes through [`BankSupplies`] drives at its bank's
/// supply instead.
pub const LOGIC_HIGH_VOLTS: Volts = embsim_board::net::LOGIC_HIGH_VOLTS;

/// I/O pads on the package.
pub const NUM_PADS: usize = 64;

/// Pads one `VIO_a_b` pin powers: the bank supplies come "in groups of 4:
/// Pxx through Pyy" (P2X8C4M64P Datasheet, Pin Descriptions, p. 6, the
/// `Vxxyy` row), which is also what the pin names on the module's netlist
/// say.
pub const PADS_PER_BANK: usize = 4;

/// Bank supply pins on the package.
pub const NUM_BANKS: usize = NUM_PADS / PADS_PER_BANK;

/// Pins on the package: the 64 pads, the 16 bank supplies and the 6 others.
pub const NUM_PINS: usize = NUM_PADS + NUM_BANKS + OTHER_PINS.len();

/// The lower edge of the core supply window the START gate holds the core
/// to: `Vdd`, Core Supply Voltage, **min 1.7 V** (typ 1.8 V) — P2X8C4M64P
/// Datasheet, "System Characteristics" → "DC Characteristics", p. 47.
pub const P2_VDD_MIN_VOLTS: Volts = 1.7;

/// The upper edge of the same window: `Vdd`, Core Supply Voltage,
/// **max 1.9 V** (the same table row, p. 47).
pub const P2_VDD_MAX_VOLTS: Volts = 1.9;

// ============================================================
// Pad drive strengths
// ============================================================

/// A pad in **fast** drive mode (`%000`), as a Thevenin impedance.
///
/// TODO(provenance): a placeholder, the crate's default push-pull
/// impedance. The value to derive is in the P2X8C4M64P Datasheet's "DC
/// Characteristics" table (p. 47): `Voh` relative to `Vxxyy` is −6 / −170
/// / −580 mV sourcing 1 / 10 / 30 mA and `Vol` 15 / 160 / 510 mV sinking
/// the same, so the fast driver's effective source resistance is 17–19 Ω
/// across the rated range. Every projection on the three reference boards
/// ranks a fast pad against pulls of 10 kΩ and more, so the placeholder
/// decides nothing today; the figure moves with phase 5's `PinDecl`
/// rewrite, where the pad's strength is a declaration.
pub const P2_FAST_OHMS: Ohms = embsim_board::net::DEFAULT_PUSH_PULL_IMPEDANCE;

/// The `%001` drive mode: 1.5 kΩ (Silicon Doc, pin configuration table).
pub const P2_PULL_1K5_OHMS: Ohms = 1_500.0;
/// The `%010` drive mode: 15 kΩ (Silicon Doc, pin configuration table).
pub const P2_PULL_15K_OHMS: Ohms = 15_000.0;
/// The `%011` drive mode: 150 kΩ (Silicon Doc, pin configuration table).
pub const P2_PULL_150K_OHMS: Ohms = 150_000.0;

/// Bit position of the `HHH` field (the drive while `OUT` = 1) in a
/// `WRPIN` word: bits 13:11.
pub const P_HIGH_SHIFT: u32 = 11;
/// Bit position of the `LLL` field (the drive while `OUT` = 0): bits 10:8.
pub const P_LOW_SHIFT: u32 = 8;
/// The pin-configuration field's mode selector, bits 20:17: `%0000` is a
/// logic mode (`%0000_CIO_HHH_LLL`); anything else is a DAC, ADC or
/// comparator mode whose low bits mean something else.
const P_MODE_SELECT_MASK: u32 = 0x001E_0000;

/// `WRPIN` pin-configuration constants for the eight `OUT` = 1 drive
/// modes, as the Spin2 compiler names them (`P_HIGH_*`).
pub const P_HIGH_FAST: u32 = 0b000 << P_HIGH_SHIFT;
/// See [`P_HIGH_FAST`].
pub const P_HIGH_1K5: u32 = 0b001 << P_HIGH_SHIFT;
/// See [`P_HIGH_FAST`].
pub const P_HIGH_15K: u32 = 0b010 << P_HIGH_SHIFT;
/// See [`P_HIGH_FAST`].
pub const P_HIGH_150K: u32 = 0b011 << P_HIGH_SHIFT;
/// See [`P_HIGH_FAST`].
pub const P_HIGH_1MA: u32 = 0b100 << P_HIGH_SHIFT;
/// See [`P_HIGH_FAST`].
pub const P_HIGH_100UA: u32 = 0b101 << P_HIGH_SHIFT;
/// See [`P_HIGH_FAST`].
pub const P_HIGH_10UA: u32 = 0b110 << P_HIGH_SHIFT;
/// See [`P_HIGH_FAST`].
pub const P_HIGH_FLOAT: u32 = 0b111 << P_HIGH_SHIFT;

/// `WRPIN` pin-configuration constants for the eight `OUT` = 0 drive
/// modes (`P_LOW_*`).
pub const P_LOW_FAST: u32 = 0b000 << P_LOW_SHIFT;
/// See [`P_LOW_FAST`].
pub const P_LOW_1K5: u32 = 0b001 << P_LOW_SHIFT;
/// See [`P_LOW_FAST`].
pub const P_LOW_15K: u32 = 0b010 << P_LOW_SHIFT;
/// See [`P_LOW_FAST`].
pub const P_LOW_150K: u32 = 0b011 << P_LOW_SHIFT;
/// See [`P_LOW_FAST`].
pub const P_LOW_1MA: u32 = 0b100 << P_LOW_SHIFT;
/// See [`P_LOW_FAST`].
pub const P_LOW_100UA: u32 = 0b101 << P_LOW_SHIFT;
/// See [`P_LOW_FAST`].
pub const P_LOW_10UA: u32 = 0b110 << P_LOW_SHIFT;
/// See [`P_LOW_FAST`].
pub const P_LOW_FLOAT: u32 = 0b111 << P_LOW_SHIFT;

/// One of the eight drive modes a `HHH`/`LLL` field selects.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PadMode {
    /// `%000`: the fast push-pull driver.
    Fast,
    /// `%001`: 1.5 kΩ.
    Ohms1k5,
    /// `%010`: 15 kΩ.
    Ohms15k,
    /// `%011`: 150 kΩ.
    Ohms150k,
    /// `%100`: a 1 mA current source (not mapped).
    Current1mA,
    /// `%101`: a 100 µA current source (not mapped).
    Current100uA,
    /// `%110`: a 10 µA current source (not mapped).
    Current10uA,
    /// `%111`: the pad floats.
    Float,
}

impl PadMode {
    /// The mode a three-bit field selects.
    pub const fn from_field(bits: u32) -> Self {
        match bits & 0b111 {
            0b000 => PadMode::Fast,
            0b001 => PadMode::Ohms1k5,
            0b010 => PadMode::Ohms15k,
            0b011 => PadMode::Ohms150k,
            0b100 => PadMode::Current1mA,
            0b101 => PadMode::Current100uA,
            0b110 => PadMode::Current10uA,
            _ => PadMode::Float,
        }
    }

    /// The mode's Thevenin impedance, for the modes that have one.
    pub const fn ohms(self) -> Option<Ohms> {
        match self {
            PadMode::Fast => Some(P2_FAST_OHMS),
            PadMode::Ohms1k5 => Some(P2_PULL_1K5_OHMS),
            PadMode::Ohms15k => Some(P2_PULL_15K_OHMS),
            PadMode::Ohms150k => Some(P2_PULL_150K_OHMS),
            PadMode::Current1mA | PadMode::Current100uA | PadMode::Current10uA | PadMode::Float => {
                None
            }
        }
    }

    /// Whether the mode is one of the current sources the package does not
    /// map.
    pub const fn is_current_source(self) -> bool {
        matches!(
            self,
            PadMode::Current1mA | PadMode::Current100uA | PadMode::Current10uA
        )
    }
}

/// The `(OUT = 1, OUT = 0)` drive modes a `WRPIN` word configures.
///
/// A word whose mode selector (bits 20:17) is not `%0000` is a DAC, ADC or
/// comparator configuration whose low bits are not drive modes; a pad the
/// guest drives in one of those reads as **fast** both ways — the drive a
/// pad had before any of this was decoded, and the smallest claim about a
/// mode the package does not model.
pub const fn pad_modes(cfg: u32) -> (PadMode, PadMode) {
    if cfg & P_MODE_SELECT_MASK != 0 {
        return (PadMode::Fast, PadMode::Fast);
    }
    (
        PadMode::from_field(cfg >> P_HIGH_SHIFT),
        PadMode::from_field(cfg >> P_LOW_SHIFT),
    )
}

/// What a pad the guest is driving presents to its net.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PadDrive {
    /// High-impedance: `DIR` clear, the float mode, or a bank with no
    /// supply.
    Released,
    /// A Thevenin source: the level through the mode's impedance.
    Thevenin(TheveninDrive),
    /// One of the current-source modes — not mapped. A core treats it as
    /// released and says so once.
    CurrentSource(PadMode),
}

/// The drive a pad with `DIR` set presents, from its `WRPIN` word and its
/// `OUT` bit: high at `high_volts` — the pad's bank supply, see
/// [`BankSupplies::pad_drive`] — through the `HHH` mode's impedance, low
/// at 0 V through the `LLL` mode's. A pad with `DIR` clear is
/// [`PadDrive::Released`] whatever the word says.
pub fn pad_drive(cfg: u32, dir: bool, out: bool, high_volts: Volts) -> PadDrive {
    if !dir {
        return PadDrive::Released;
    }
    let (high, low) = pad_modes(cfg);
    let mode = if out { high } else { low };
    match mode.ohms() {
        Some(impedance) => PadDrive::Thevenin(TheveninDrive {
            volts: if out { high_volts } else { 0.0 },
            impedance,
        }),
        None if mode.is_current_source() => PadDrive::CurrentSource(mode),
        None => PadDrive::Released,
    }
}

// ============================================================
// The facade
// ============================================================

/// `P0`..`P63`, as the netlists name them.
static PAD_NAMES: [&str; NUM_PADS] = [
    "P0", "P1", "P2", "P3", "P4", "P5", "P6", "P7", "P8", "P9", "P10", "P11", "P12", "P13", "P14",
    "P15", "P16", "P17", "P18", "P19", "P20", "P21", "P22", "P23", "P24", "P25", "P26", "P27",
    "P28", "P29", "P30", "P31", "P32", "P33", "P34", "P35", "P36", "P37", "P38", "P39", "P40",
    "P41", "P42", "P43", "P44", "P45", "P46", "P47", "P48", "P49", "P50", "P51", "P52", "P53",
    "P54", "P55", "P56", "P57", "P58", "P59", "P60", "P61", "P62", "P63",
];

/// The name of pad `pin` (taken modulo 64) on the facade.
pub fn pin_name(pin: u8) -> &'static str {
    PAD_NAMES[usize::from(pin & 63)]
}

/// The sixteen bank supply pins, in bank order: `VIO_0_3` powers `P0`–`P3`,
/// `VIO_4_7` the next four, and so on ([`PADS_PER_BANK`]).
static BANK_PINS: [&str; NUM_BANKS] = [
    "VIO_0_3",
    "VIO_4_7",
    "VIO_8_11",
    "VIO_12_15",
    "VIO_16_19",
    "VIO_20_23",
    "VIO_24_27",
    "VIO_28_31",
    "VIO_32_35",
    "VIO_36_39",
    "VIO_40_43",
    "VIO_44_47",
    "VIO_48_51",
    "VIO_52_55",
    "VIO_56_59",
    "VIO_60_63",
];

/// The bank pad `pin` (taken modulo 64) belongs to: `0..NUM_BANKS`.
pub const fn bank_of(pin: u8) -> usize {
    ((pin & 63) as usize) / PADS_PER_BANK
}

/// The supply pin of bank `bank` (taken modulo 16), as the netlists name
/// it.
pub fn bank_pin_name(bank: usize) -> &'static str {
    BANK_PINS[bank % NUM_BANKS]
}

/// The package's pins beside the pads and the bank supplies, as the
/// P2-EC32MB netlist normalises them: the core rail and ground, the reset
/// and test inputs, and the crystal pair.
const OTHER_PINS: [(&str, PinKind); 6] = [
    ("GND", PinKind::PowerIn),
    ("VDD", PinKind::PowerIn),
    ("RESN", PinKind::DigitalIn),
    ("TEST", PinKind::DigitalIn),
    ("XI", PinKind::DigitalIn),
    ("XO", PinKind::DigitalOut),
];

/// The chip's facade: 64 bidirectional pads that idle released, and the 22
/// package pins — the supplies as `PowerIn`, `RESN` and `TEST` sensed, `XI`
/// a pulse sink whose delivered rate is the crystal, `XO` a released
/// output. This is what the P2-EC32MB's `U100` slot expects, in both
/// directions.
pub fn p2x8c4m64p_pins() -> Vec<PinDecl> {
    let mut pins = Vec::with_capacity(NUM_PINS);
    for name in PAD_NAMES {
        pins.push(PinDecl::new(name, PinKind::DigitalBidir).with_idle(IdleDrive::Released));
    }
    for (name, kind) in OTHER_PINS {
        let pin = PinDecl::new(name, kind);
        pins.push(match name {
            "XI" => pin.with_stream(StreamRole::PulseSink),
            "XO" => pin.with_idle(IdleDrive::Released),
            _ => pin,
        });
    }
    for name in BANK_PINS {
        pins.push(PinDecl::new(name, PinKind::PowerIn));
    }
    pins
}

// ============================================================
// What the package delivers to its core
// ============================================================

/// The voltage a supply pin's net names, or nothing: an `Analog(v)` is a
/// voltage — a terminal at the pin, or a solved operating point; a
/// digital projection (`Driven`, `Pulled`) names a level and no voltage;
/// `Floating` and `Contention` name nothing. The package invents none.
fn named_volts(state: NetState) -> Option<Volts> {
    match state {
        NetState::Analog(v) if v.is_finite() => Some(v),
        _ => None,
    }
}

/// The reset inputs as the package reads them: `RESN` projected through
/// the engine's own level rule ([`level_of`]) — `Some(High)` a released
/// reset, `Some(Low)` a held one, `None` nothing reaching the pin — and
/// `VDD` both as that level and as the voltage its net names, which is
/// what the START gate holds the core to.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct P2ResetState {
    /// `RESN` (active low).
    pub resn: Option<Level>,
    /// `VDD`, the core supply, as a level.
    pub vdd: Option<Level>,
    /// `VDD` as the voltage its net names, `None` when it names no voltage
    /// (nothing reaches the pin, or a source with a level and no voltage).
    pub vdd_volts: Option<Volts>,
}

impl P2ResetState {
    /// Whether `VDD` names a voltage inside the datasheet's core-supply
    /// window, [`P2_VDD_MIN_VOLTS`] to [`P2_VDD_MAX_VOLTS`] inclusive.
    pub fn vdd_in_window(&self) -> bool {
        self.vdd_volts
            .is_some_and(|v| (P2_VDD_MIN_VOLTS..=P2_VDD_MAX_VOLTS).contains(&v))
    }

    /// Whether the chip is out of reset: `RESN` released **and** `VDD`
    /// inside its window — the START gate's condition.
    pub fn out_of_reset(&self) -> bool {
        self.resn == Some(Level::High) && self.vdd_in_window()
    }
}

/// Whether the core has been started, as [`P2PackageHandle::start_state`]
/// reports it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum StartState {
    /// The START gate is closed: the core has not run, and the reset
    /// inputs as last read say why.
    Held {
        /// The inputs the gate is waiting on.
        reset: P2ResetState,
    },
    /// The core was started at this virtual instant, and runs from it.
    Started {
        /// The START instant, nanoseconds.
        at_ns: u64,
    },
}

/// The crystal a delivered train on `XI` is: its rate, or `None` for a
/// held train (no clock reaches the pin).
pub fn crystal_of(train: &PulseTrain) -> Option<u64> {
    (train.pulses.freq_hz > 0).then(|| u64::from(train.pulses.freq_hz))
}

// ============================================================
// The bank supplies
// ============================================================

/// A bank's supply as an `f64`'s bits: NaN is no voltage, which no supply
/// can name (`named_volts` passes finite voltages only).
const NO_SUPPLY_BITS: u64 = f64::NAN.to_bits();

/// Every bank's bit in the table's masks.
const ALL_BANKS_MASK: u16 = ((1u32 << NUM_BANKS) - 1) as u16;

struct BankTable {
    /// Per bank, the voltage its supply pin's net names, as bits.
    volts: [AtomicU64; NUM_BANKS],
    /// Banks whose supply sense has delivered at least once, one bit per
    /// bank: what the START gate waits for before a core may run
    /// ([`BankSupplies::all_delivered`]).
    delivered: AtomicU16,
    /// Banks a core has driven a pad in while their supply named no
    /// voltage, one bit per bank: the report, made once per bank.
    driven_unpowered: AtomicU16,
}

/// The sixteen bank supplies as the package last sensed them, shared with
/// the core: what a pad drives high at.
///
/// A core publishes a pad through [`BankSupplies::pad_drive`], which puts
/// the bank's voltage into the `WRPIN` decode ([`pad_drive`]) — and
/// releases the pad, reporting the bank once, when its supply names no
/// voltage. Cheap on the core's path: one atomic load per pad decode.
#[derive(Clone)]
pub struct BankSupplies {
    inner: Arc<BankTable>,
}

impl std::fmt::Debug for BankSupplies {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let volts: Vec<Option<Volts>> = (0..NUM_BANKS).map(|b| self.volts(b)).collect();
        f.debug_struct("BankSupplies")
            .field("volts", &volts)
            .field("all_delivered", &self.all_delivered())
            .field("driven_unpowered", &self.unpowered_banks_driven())
            .finish()
    }
}

impl Default for BankSupplies {
    fn default() -> Self {
        Self::unpowered()
    }
}

impl BankSupplies {
    /// Every bank with no supply and no sense delivered: what a package's
    /// table holds before its senses deliver, and what a core holds before
    /// it is attached.
    pub fn unpowered() -> Self {
        Self {
            inner: Arc::new(BankTable {
                volts: std::array::from_fn(|_| AtomicU64::new(NO_SUPPLY_BITS)),
                delivered: AtomicU16::new(0),
                driven_unpowered: AtomicU16::new(0),
            }),
        }
    }

    /// Every bank at `volts`: a bench fixture for a core's own unit tests,
    /// standing in for a package whose sixteen supply pins all read the
    /// same rail.
    pub fn held_at(volts: Volts) -> Self {
        let table = Self::unpowered();
        for bank in 0..NUM_BANKS {
            table.set(bank, Some(volts));
        }
        table
    }

    /// A bank's supply sense delivered: record what the pin names, and
    /// that the bank has been told.
    fn set(&self, bank: usize, volts: Option<Volts>) {
        let bank = bank % NUM_BANKS;
        let bits = volts.map_or(NO_SUPPLY_BITS, f64::to_bits);
        self.inner.volts[bank].store(bits, Ordering::Relaxed);
        self.inner
            .delivered
            .fetch_or(1u16 << bank, Ordering::Relaxed);
    }

    /// Every bank's supply sense has delivered at least once, so the table
    /// says what each supply pin reads rather than what nothing has told
    /// it yet. A package's senses deliver once at registration, so this
    /// holds from the moment the sixteen are registered; the START gate
    /// does not open before it, which is what lets a core publish a pad
    /// at its START instant against a populated table. A bench table
    /// ([`BankSupplies::held_at`]) is delivered in full.
    pub fn all_delivered(&self) -> bool {
        self.inner.delivered.load(Ordering::Relaxed) == ALL_BANKS_MASK
    }

    /// The voltage bank `bank`'s supply pin names, or `None` while it
    /// names no voltage.
    pub fn volts(&self, bank: usize) -> Option<Volts> {
        let v = f64::from_bits(self.inner.volts[bank % NUM_BANKS].load(Ordering::Relaxed));
        v.is_finite().then_some(v)
    }

    /// The drive pad `pin` presents from its `WRPIN` word and its
    /// `DIR`/`OUT` bits, high at its bank's supply ([`pad_drive`]). A pad
    /// with `DIR` set in a bank whose supply names no voltage presents
    /// nothing, and the bank is reported once, by its supply pin.
    pub fn pad_drive(&self, pin: u8, cfg: u32, dir: bool, out: bool) -> PadDrive {
        if !dir {
            return PadDrive::Released;
        }
        let bank = bank_of(pin);
        match self.volts(bank) {
            Some(high_volts) => pad_drive(cfg, dir, out, high_volts),
            None => {
                let bit = 1u16 << bank;
                let before = self.inner.driven_unpowered.fetch_or(bit, Ordering::Relaxed);
                if before & bit == 0 {
                    tracing::warn!(
                        pad = pin_name(pin),
                        supply = bank_pin_name(bank),
                        "p2: a pad was driven in a bank whose supply pin reaches no voltage; \
                         its driver has no supply and the pad presents nothing to its net"
                    );
                }
                PadDrive::Released
            }
        }
    }

    /// The banks a core has driven a pad in while their supply named no
    /// voltage, ascending — each reported once (`tracing::warn`) when it
    /// first happened.
    pub fn unpowered_banks_driven(&self) -> Vec<usize> {
        let bits = self.inner.driven_unpowered.load(Ordering::Relaxed);
        (0..NUM_BANKS).filter(|b| bits & (1 << b) != 0).collect()
    }
}

// ============================================================
// The pads a core is handed
// ============================================================

type CrystalCallback = Box<dyn Fn(Option<u64>) + Send + Sync>;
type ResetCallback = Box<dyn Fn(P2ResetState) + Send + Sync>;
type WakeCallback = Arc<dyn Fn(u64) + Send + Sync>;

/// What a core subscribed to through [`P2Pads`], collected during its
/// attach and frozen when the package installs its own senses.
#[derive(Default)]
struct Subscribers {
    crystal: Vec<CrystalCallback>,
    reset: Vec<ResetCallback>,
}

/// The package's last-delivered facts, readable through
/// [`P2PackageHandle`].
#[derive(Default)]
struct PackageState {
    crystal_hz: Option<u64>,
    reset: P2ResetState,
}

/// The START gate: what the core asked for before it may run, and
/// whether it runs.
#[derive(Default)]
struct Gate {
    /// [`Component::start`] has run: the system is live and the core may
    /// be started the moment the inputs allow.
    start_requested: bool,
    /// The START instant, once the core was started.
    started_at_ns: Option<u64>,
    /// The core's wake handler, registered through [`P2Pads::on_wake_ns`].
    core_wake: Option<WakeCallback>,
    /// Wakes the core asked for through [`P2Pads::schedule_at_ns`] before
    /// it was started; forwarded at the START instant, no earlier.
    held_wakes: Vec<u64>,
}

/// A core's whole surface: its 64 pads, the package's two facts (the
/// crystal on `XI`, the reset inputs), the bank supplies, and the engine's
/// wake scheduling. Handed to [`P2Core::attach`] once, by the package; a
/// core keeps the handles it needs and subscribes to what it wants
/// delivered. (The native core is the exception that takes the underlying
/// net I/O whole — see [`McuComponent`]'s `P2Core` impl.)
///
/// A pad sense is also the declaration that the core **reads** that pad:
/// every pad is a released bidirectional pin, an input until driven, and a
/// pad the core subscribes to whose net floats is reported as
/// [`embsim_board::Finding::FloatingSense`] — a pad nothing reads floats
/// without one. A core subscribes to what it samples: the QEMU core to all
/// 64 (a guest may `testp` any of them), the native core to the pads its
/// HAL tables bridge, a core held in reset to none.
///
/// Wakes go through the START gate: a wake asked for before the core is
/// started is held and lands at the START instant; one asked for after
/// is the engine's at once. The wake handler is registered once, through
/// [`P2Pads::on_wake_ns`], and delivered only after the core was started.
///
/// Everything here goes through the one interface a node has
/// (`NODES.md` §11): pad handles publish drives, pad senses deliver the
/// resolved net, and the package facts are the package's own senses
/// projected once and fanned out. A core never reaches a net it has no
/// pad on.
#[derive(Clone)]
pub struct P2Pads {
    io: ComponentNetIo,
    subscribers: Arc<Mutex<Subscribers>>,
    gate: Arc<Mutex<Gate>>,
    banks: BankSupplies,
}

impl std::fmt::Debug for P2Pads {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("P2Pads").finish_non_exhaustive()
    }
}

impl P2Pads {
    /// The handle a core drives pad `pin` (`0..64`) through.
    pub fn pad(&self, pin: u8) -> Result<PinHandle, AttachError> {
        self.io.pin(pin_name(pin))
    }

    /// The bank supplies, as the package senses them: what a pad drives
    /// high at ([`BankSupplies::pad_drive`]).
    pub fn bank_supplies(&self) -> BankSupplies {
        self.banks.clone()
    }

    /// Subscribe to the resolved state of pad `pin`'s net: delivered once
    /// at registration and on every change, on the engine thread.
    pub fn on_pad_sense(
        &self,
        pin: u8,
        callback: impl Fn(NetState) + Send + 'static,
    ) -> Result<(), AttachError> {
        self.io.on_sense(pin_name(pin), callback)
    }

    /// Subscribe to the crystal: the rate delivered on `XI`, `Some(hz)`
    /// when a train with a rate reaches the pin and `None` when it is held.
    /// Delivered on the engine thread whenever the train changes.
    pub fn on_crystal(&self, callback: impl Fn(Option<u64>) + Send + Sync + 'static) {
        self.subscribers
            .lock()
            .expect("subscriber list never poisoned")
            .crystal
            .push(Box::new(callback));
    }

    /// Subscribe to the reset inputs (`RESN` and `VDD`, projected):
    /// delivered on the engine thread whenever either changes. Information
    /// for the core; the START gate that acts on them is the package's.
    pub fn on_reset(&self, callback: impl Fn(P2ResetState) + Send + Sync + 'static) {
        self.subscribers
            .lock()
            .expect("subscriber list never poisoned")
            .reset
            .push(Box::new(callback));
    }

    /// The core's wake handler (one per package, last registration wins;
    /// see [`ComponentNetIo::on_wake_ns`]). Delivered on the engine thread
    /// with the current virtual nanosecond, only once the core is started.
    pub fn on_wake_ns(&self, callback: impl Fn(u64) + Send + Sync + 'static) {
        self.gate
            .lock()
            .expect("start gate never poisoned")
            .core_wake = Some(Arc::new(callback));
    }

    /// Arm a one-shot wake at an absolute virtual nanosecond. Before the
    /// core is started the request is held and lands at the START instant
    /// (or at `at_ns` if that is later); after, it is the engine's at once.
    pub fn schedule_at_ns(&self, at_ns: u64) {
        let mut gate = self.gate.lock().expect("start gate never poisoned");
        if gate.started_at_ns.is_some() {
            drop(gate);
            self.io.schedule_at_ns(at_ns);
        } else {
            gate.held_wakes.push(at_ns);
        }
    }
}

/// What runs inside a [`P2Package`].
///
/// The trait is deliberately small: a core is attached once to its
/// [`P2Pads`] — pad handles, pad senses, the crystal and reset
/// subscriptions, the bank supplies, wake scheduling — and started once,
/// at the START instant. The package owns the pin declarations, the
/// package-level senses and the gate; the core owns execution.
pub trait P2Core: Send + Sync {
    /// Take the pads. Runs at build, before the package is shared.
    fn attach(&mut self, pads: P2Pads) -> Result<(), AttachError>;

    /// Begin execution the core owns, at the START instant: after every
    /// component has attached and started ([`Component::start`], the live
    /// path only) **and** the package's gate has opened — `RESN` released
    /// and `VDD` inside its window. The current virtual nanosecond is the
    /// START instant the core's clock counts from. Runs before any wake
    /// the core asked for is delivered, on the thread the gate opened on:
    /// the engine thread when a sense opened it, the starting thread when
    /// the inputs already allowed it at start.
    fn start(&mut self) {}
}

/// A package with no core: the state any P2 is in before it runs. Every
/// pad released, the rails and `RESN` sensed, `XI` accepting the rate.
#[derive(Debug, Default, Clone, Copy)]
pub struct HeldInReset;

impl P2Core for HeldInReset {
    fn attach(&mut self, _pads: P2Pads) -> Result<(), AttachError> {
        Ok(())
    }
}

/// The native firmware image is a core too: [`McuComponent`] already
/// speaks the node interface, so the package hands it the pads' net I/O
/// and it bridges the channels its HAL tables name, exactly as it would
/// on a board of its own. The package still declares every pin and still
/// runs its own `XI`/`RESN`/`VDD`/`VIO` senses beside it, and its START
/// gate holds the firmware entry ([`McuComponent`]'s `start`) as it holds
/// any core's.
///
/// What it is handed is the package's **whole** net I/O — the handle table
/// carries `XI`, `XO`, `RESN`, `VDD` and the `VIO` pins beside the 64 pads
/// — and the narrowing to the pads is by what the core names, not by a
/// filter: its HAL tables name pads (`P0`, `P2`, …) and nothing else. The
/// interface is the same one every node has, so nothing is reachable that
/// a node could not reach; the package-level facts still arrive through
/// the package's own senses, as for any core. Its wake handler and its
/// schedules go to the engine directly, so the gate holds its start and
/// nothing else, and its pads drive at [`LOGIC_HIGH_VOLTS`] through the
/// HAL bridges (the module docs say why).
impl P2Core for McuComponent {
    fn attach(&mut self, pads: P2Pads) -> Result<(), AttachError> {
        Component::attach(self, pads.io)
    }

    fn start(&mut self) {
        Component::start(self);
    }
}

/// A view of a package's delivered facts that outlives handing it to a
/// `System`.
#[derive(Clone)]
pub struct P2PackageHandle {
    state: Arc<Mutex<PackageState>>,
    gate: Arc<Mutex<Gate>>,
    banks: BankSupplies,
}

impl std::fmt::Debug for P2PackageHandle {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("P2PackageHandle")
            .field("crystal_hz", &self.crystal_hz())
            .field("reset", &self.reset())
            .field("start", &self.start_state())
            .field("banks", &self.banks)
            .finish()
    }
}

impl P2PackageHandle {
    /// The crystal, as last delivered on `XI`: `Some(hz)` while a rate
    /// reaches the pin.
    pub fn crystal_hz(&self) -> Option<u64> {
        self.state
            .lock()
            .expect("package state never poisoned")
            .crystal_hz
    }

    /// The reset inputs, as last read.
    pub fn reset(&self) -> P2ResetState {
        self.state
            .lock()
            .expect("package state never poisoned")
            .reset
    }

    /// Whether the core has been started, and when — or why not yet.
    pub fn start_state(&self) -> StartState {
        match self
            .gate
            .lock()
            .expect("start gate never poisoned")
            .started_at_ns
        {
            Some(at_ns) => StartState::Started { at_ns },
            None => StartState::Held {
                reset: self.reset(),
            },
        }
    }

    /// The START instant, once the core was started.
    pub fn started_at_ns(&self) -> Option<u64> {
        match self.start_state() {
            StartState::Started { at_ns } => Some(at_ns),
            StartState::Held { .. } => None,
        }
    }

    /// The voltage bank `bank`'s supply pin names, as last sensed.
    pub fn bank_volts(&self, bank: usize) -> Option<Volts> {
        self.banks.volts(bank)
    }

    /// The banks the core drove a pad in while their supply named no
    /// voltage ([`BankSupplies::unpowered_banks_driven`]).
    pub fn unpowered_banks_driven(&self) -> Vec<usize> {
        self.banks.unpowered_banks_driven()
    }
}

/// The `P2X8C4M64P` package around a core.
pub struct P2Package<C> {
    pins: Vec<PinDecl>,
    /// The core, shared with the gate: it is started from whichever thread
    /// opens the gate.
    core: Arc<Mutex<C>>,
    state: Arc<Mutex<PackageState>>,
    gate: Arc<Mutex<Gate>>,
    banks: BankSupplies,
    /// The package's own net I/O, kept from attach for the gate's wake
    /// forwarding.
    io: Option<ComponentNetIo>,
}

impl<C: std::fmt::Debug> std::fmt::Debug for P2Package<C> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("P2Package")
            .field("pins", &self.pins.len())
            .field("core", &*self.core.lock().expect("core never poisoned"))
            .finish()
    }
}

impl P2Package<HeldInReset> {
    /// A package with no core: every pad released, rails and `RESN` sensed,
    /// `XI` accepting the rate — the state any P2 is in before it runs.
    pub fn held_in_reset() -> Self {
        Self::new(HeldInReset)
    }
}

impl P2Package<McuComponent> {
    /// The package around the native firmware image.
    pub fn native(mcu: McuComponent) -> Self {
        Self::new(mcu)
    }
}

impl<C: P2Core + 'static> P2Package<C> {
    /// The package around `core`.
    pub fn new(core: C) -> Self {
        Self {
            pins: p2x8c4m64p_pins(),
            core: Arc::new(Mutex::new(core)),
            state: Arc::new(Mutex::new(PackageState::default())),
            gate: Arc::new(Mutex::new(Gate::default())),
            banks: BankSupplies::unpowered(),
            io: None,
        }
    }

    /// A view of the package's delivered facts that outlives handing it to
    /// a `System`.
    pub fn handle(&self) -> P2PackageHandle {
        P2PackageHandle {
            state: Arc::clone(&self.state),
            gate: Arc::clone(&self.gate),
            banks: self.banks.clone(),
        }
    }

    /// Open the gate if it can be opened: the system is live
    /// (`start_requested`), the core is not yet started, the inputs allow
    /// it, and every bank sense has delivered ([`BankSupplies::all_delivered`],
    /// so the core's first pad publish reads a populated table). Starts the
    /// core at `now`, then forwards every wake it held. Returns whether the
    /// core is started (now or before).
    ///
    /// Both conditions are read under the gate's lock, on whichever thread
    /// asks — the engine thread from a `RESN`, `VDD` or bank delivery, the
    /// caller's from `Component::start` — so the gate opens exactly once,
    /// at the first delivery after which all of them hold.
    fn try_begin(
        gate: &Mutex<Gate>,
        core: &Mutex<C>,
        state: &Mutex<PackageState>,
        banks: &BankSupplies,
        io: &ComponentNetIo,
        now: u64,
    ) -> bool {
        let held = {
            let mut gate = gate.lock().expect("start gate never poisoned");
            if gate.started_at_ns.is_some() {
                return true;
            }
            let reset = state.lock().expect("package state never poisoned").reset;
            if !gate.start_requested || !reset.out_of_reset() || !banks.all_delivered() {
                return false;
            }
            gate.started_at_ns = Some(now);
            std::mem::take(&mut gate.held_wakes)
        };
        tracing::info!(
            at_ns = now,
            "p2: START — RESN released and VDD inside its window; the core runs from here"
        );
        // Before any wake the core asked for is delivered: the core's
        // clock counts from this instant.
        core.lock().expect("core never poisoned").start();
        for at_ns in held {
            io.schedule_at_ns(at_ns.max(now));
        }
        true
    }
}

impl<C: P2Core + 'static> Component for P2Package<C> {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        self.io = Some(io.clone());

        // The gate's wake forwarder, registered ahead of the core: a core
        // that routes its handler through `P2Pads::on_wake_ns` is delivered
        // from here once started; a core that registers its own handler
        // directly on the net I/O (the native core) replaces this one —
        // last registration wins — and keeps its wakes as they are.
        {
            let gate = Arc::clone(&self.gate);
            io.on_wake_ns(move |now| {
                let callback = {
                    let gate = gate.lock().expect("start gate never poisoned");
                    gate.started_at_ns.and(gate.core_wake.clone())
                };
                if let Some(callback) = callback {
                    callback(now);
                }
            });
        }

        let subscribers = Arc::new(Mutex::new(Subscribers::default()));
        self.core
            .lock()
            .expect("core never poisoned")
            .attach(P2Pads {
                io: io.clone(),
                subscribers: Arc::clone(&subscribers),
                gate: Arc::clone(&self.gate),
                banks: self.banks.clone(),
            })?;
        // The core has subscribed to what it wants; freeze the lists so
        // delivery never holds a lock across a callback.
        let Subscribers { crystal, reset } =
            std::mem::take(&mut *subscribers.lock().expect("subscriber list never poisoned"));
        let crystal: Arc<[CrystalCallback]> = crystal.into();
        let reset: Arc<[ResetCallback]> = reset.into();

        // XI: the rate delivered here is the crystal. One projection, then
        // every subscriber.
        {
            let state = Arc::clone(&self.state);
            io.on_pulse("XI", move |train| {
                let hz = crystal_of(&train);
                state
                    .lock()
                    .expect("package state never poisoned")
                    .crystal_hz = hz;
                for callback in crystal.iter() {
                    callback(hz);
                }
            })?;
        }

        // The bank supplies: what each bank's pads drive high at. Registered
        // BEFORE the two senses the gate reads: the engine delivers a sense
        // once at registration, in registration order, so every bank has
        // delivered by the time `RESN` or `VDD` can open the gate, and the
        // gate opens at the reset delivery itself with the table populated
        // — a core that publishes a pad at its START instant reads it. Each
        // delivery marks its bank told and gives the gate its chance too,
        // so the gate (which also checks `BankSupplies::all_delivered`)
        // cannot stall on the table whatever the order. Registered after
        // the two, the gate opened at the sixteenth bank's delivery instead
        // and a reader between the two deliveries saw `Held` with an
        // out-of-reset reason (measured 5/30, the phase-4 review record).
        for (bank, name) in BANK_PINS.iter().enumerate() {
            let banks = self.banks.clone();
            let state = Arc::clone(&self.state);
            let gate = Arc::clone(&self.gate);
            let core = Arc::clone(&self.core);
            let io_for_gate = io.clone();
            io.on_sense(name, move |sensed| {
                banks.set(bank, named_volts(sensed));
                Self::try_begin(
                    &gate,
                    &core,
                    &state,
                    &banks,
                    &io_for_gate,
                    virtual_clock::virtual_ns(),
                );
            })?;
        }

        // RESN and VDD: each sense updates its half, delivers the pair,
        // and gives the START gate its chance.
        for (pin, is_resn) in [("RESN", true), ("VDD", false)] {
            let state = Arc::clone(&self.state);
            let reset = Arc::clone(&reset);
            let gate = Arc::clone(&self.gate);
            let core = Arc::clone(&self.core);
            let banks = self.banks.clone();
            let io_for_gate = io.clone();
            io.on_sense(pin, move |sensed| {
                let level = level_of(sensed);
                let snapshot = {
                    let mut state = state.lock().expect("package state never poisoned");
                    if is_resn {
                        state.reset.resn = level;
                    } else {
                        state.reset.vdd = level;
                        state.reset.vdd_volts = named_volts(sensed);
                    }
                    state.reset
                };
                for callback in reset.iter() {
                    callback(snapshot);
                }
                Self::try_begin(
                    &gate,
                    &core,
                    &state,
                    &banks,
                    &io_for_gate,
                    virtual_clock::virtual_ns(),
                );
            })?;
        }
        Ok(())
    }

    fn start(&mut self) {
        let io = self.io.as_ref().expect("start runs after attach").clone();
        self.gate
            .lock()
            .expect("start gate never poisoned")
            .start_requested = true;
        let now = if virtual_clock::is_initialized() {
            virtual_clock::virtual_ns()
        } else {
            0
        };
        if !Self::try_begin(&self.gate, &self.core, &self.state, &self.banks, &io, now) {
            let reset = self
                .state
                .lock()
                .expect("package state never poisoned")
                .reset;
            tracing::info!(
                ?reset,
                vdd_window = format_args!("{P2_VDD_MIN_VOLTS}..={P2_VDD_MAX_VOLTS} V"),
                "p2: the core is held at start: RESN must read released and VDD a voltage \
                 inside its window; the START gate opens when they do"
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embsim_board::{PulseDirection, PulseSegment};
    use rstest::rstest;

    #[test]
    fn the_facade_is_the_ec32mb_u100_slot() {
        let pins = p2x8c4m64p_pins();
        assert_eq!(pins.len(), 86);
        assert_eq!(NUM_PINS, 86);
        assert_eq!(NUM_BANKS, 16);
        assert_eq!(pins[0].number, "P0");
        assert_eq!(pins[63].number, "P63");
        for pad in &pins[..NUM_PADS] {
            assert_eq!(pad.kind, PinKind::DigitalBidir, "{}", pad.number);
            assert_eq!(pad.idle, IdleDrive::Released, "{}", pad.number);
            assert_eq!(pad.stream, None, "{}", pad.number);
        }
        let pin = |name: &str| pins.iter().find(|p| p.number == name).copied().unwrap();
        assert_eq!(pin("XI").kind, PinKind::DigitalIn);
        assert_eq!(pin("XI").stream, Some(StreamRole::PulseSink));
        assert_eq!(pin("XO").kind, PinKind::DigitalOut);
        assert_eq!(pin("XO").idle, IdleDrive::Released);
        assert_eq!(pin("RESN").kind, PinKind::DigitalIn);
        assert_eq!(pin("TEST").kind, PinKind::DigitalIn);
        for rail in ["VDD", "GND", "VIO_0_3", "VIO_56_59", "VIO_60_63"] {
            assert_eq!(pin(rail).kind, PinKind::PowerIn, "{rail}");
        }
        assert_eq!(pin_name(0), "P0");
        assert_eq!(pin_name(63), "P63");
        assert_eq!(pin_name(64), "P0", "taken modulo 64");
    }

    /// Each `VIO_a_b` pin powers the four pads `a..=b`.
    #[rstest]
    #[case(0, 0, "VIO_0_3")]
    #[case(3, 0, "VIO_0_3")]
    #[case(4, 1, "VIO_4_7")]
    #[case(8, 2, "VIO_8_11")]
    #[case(59, 14, "VIO_56_59")]
    #[case(63, 15, "VIO_60_63")]
    fn a_pad_belongs_to_the_bank_its_supply_pin_names(
        #[case] pin: u8,
        #[case] bank: usize,
        #[case] supply: &str,
    ) {
        assert_eq!(bank_of(pin), bank);
        assert_eq!(bank_pin_name(bank), supply);
        let name = pin_name(pin);
        let (a, b) = supply
            .trim_start_matches("VIO_")
            .split_once('_')
            .map(|(a, b)| (a.parse::<u8>().unwrap(), b.parse::<u8>().unwrap()))
            .unwrap();
        assert!((a..=b).contains(&pin), "{name} in {supply}");
    }

    /// Bits 13:11 select the drive while `OUT` = 1 and bits 10:8 while
    /// `OUT` = 0; the three resistive modes are their resistances, fast is
    /// the placeholder, float is released, and a current source is the
    /// unmapped variant.
    #[rstest]
    #[case::fast_high(P_HIGH_FAST | P_LOW_FAST, true, PadDrive::Thevenin(TheveninDrive { volts: 3.3, impedance: P2_FAST_OHMS }))]
    #[case::fast_low(P_HIGH_FAST | P_LOW_FAST, false, PadDrive::Thevenin(TheveninDrive { volts: 0.0, impedance: P2_FAST_OHMS }))]
    #[case::pull_up_15k(P_HIGH_15K, true, PadDrive::Thevenin(TheveninDrive { volts: 3.3, impedance: 15_000.0 }))]
    #[case::pull_up_1k5(P_HIGH_1K5, true, PadDrive::Thevenin(TheveninDrive { volts: 3.3, impedance: 1_500.0 }))]
    #[case::pull_up_150k(P_HIGH_150K, true, PadDrive::Thevenin(TheveninDrive { volts: 3.3, impedance: 150_000.0 }))]
    #[case::pull_down_15k(P_LOW_15K, false, PadDrive::Thevenin(TheveninDrive { volts: 0.0, impedance: 15_000.0 }))]
    #[case::open_drain_high(P_HIGH_FLOAT | P_LOW_FAST, true, PadDrive::Released)]
    #[case::open_drain_low(P_HIGH_FLOAT | P_LOW_FAST, false, PadDrive::Thevenin(TheveninDrive { volts: 0.0, impedance: P2_FAST_OHMS }))]
    #[case::high_mode_does_not_apply_low(P_HIGH_15K | P_LOW_FAST, false, PadDrive::Thevenin(TheveninDrive { volts: 0.0, impedance: P2_FAST_OHMS }))]
    #[case::current_1ma(P_HIGH_1MA, true, PadDrive::CurrentSource(PadMode::Current1mA))]
    #[case::current_100ua(P_HIGH_100UA, true, PadDrive::CurrentSource(PadMode::Current100uA))]
    #[case::current_10ua_low(P_LOW_10UA, false, PadDrive::CurrentSource(PadMode::Current10uA))]
    #[case::adc_mode_word_is_fast(0x0010_0000 | P_HIGH_15K, true, PadDrive::Thevenin(TheveninDrive { volts: 3.3, impedance: P2_FAST_OHMS }))]
    fn a_wrpin_word_is_the_pads_thevenin(
        #[case] cfg: u32,
        #[case] out: bool,
        #[case] expected: PadDrive,
    ) {
        assert_eq!(pad_drive(cfg, true, out, 3.3), expected);
    }

    #[rstest]
    #[case::fast(P_HIGH_FAST)]
    #[case::pull(P_HIGH_15K)]
    #[case::current(P_HIGH_1MA)]
    fn a_pad_with_dir_clear_is_released_whatever_its_word(#[case] cfg: u32) {
        assert_eq!(pad_drive(cfg, false, true, 3.3), PadDrive::Released);
        assert_eq!(pad_drive(cfg, false, false, 3.3), PadDrive::Released);
    }

    /// A pad drives high at its bank's supply, whatever that reads; a bank
    /// whose supply names no voltage powers no driver — high or low, the
    /// pad presents nothing — and is reported once.
    #[test]
    fn a_pad_drives_at_its_banks_supply_and_nothing_in_an_unpowered_bank() {
        let banks = BankSupplies::unpowered();
        banks.set(1, Some(1.8));
        banks.set(15, Some(3.3));
        assert_eq!(banks.volts(1), Some(1.8));
        assert_eq!(banks.volts(0), None);

        assert_eq!(
            banks.pad_drive(4, P_HIGH_FAST | P_LOW_FAST, true, true),
            PadDrive::Thevenin(TheveninDrive {
                volts: 1.8,
                impedance: P2_FAST_OHMS
            })
        );
        assert_eq!(
            banks.pad_drive(63, P_HIGH_15K, true, true),
            PadDrive::Thevenin(TheveninDrive {
                volts: 3.3,
                impedance: P2_PULL_15K_OHMS
            })
        );
        assert_eq!(banks.unpowered_banks_driven(), Vec::<usize>::new());

        // Bank 0 has no supply: driven high or low, the pad is released.
        assert_eq!(
            banks.pad_drive(0, P_HIGH_FAST | P_LOW_FAST, true, true),
            PadDrive::Released
        );
        assert_eq!(
            banks.pad_drive(2, P_HIGH_FAST | P_LOW_FAST, true, false),
            PadDrive::Released
        );
        // DIR clear in an unpowered bank is no drive and no report.
        assert_eq!(
            banks.pad_drive(9, P_HIGH_FAST, false, true),
            PadDrive::Released
        );
        assert_eq!(banks.unpowered_banks_driven(), vec![0]);
        assert_eq!(bank_pin_name(0), "VIO_0_3");

        let bench = BankSupplies::held_at(3.3);
        for bank in 0..NUM_BANKS {
            assert_eq!(bench.volts(bank), Some(3.3));
        }
    }

    /// The table knows whether every bank has been told: not until the
    /// sixteenth sense delivers — a bank that delivers "no voltage" counts
    /// as told — and the bench fixture is told in full.
    #[test]
    fn the_bank_table_is_delivered_only_once_every_bank_has_been_told() {
        let banks = BankSupplies::unpowered();
        assert!(!banks.all_delivered());
        for bank in 0..NUM_BANKS - 1 {
            banks.set(bank, if bank % 2 == 0 { Some(3.3) } else { None });
            assert!(
                !banks.all_delivered(),
                "bank {bank} told, one still to come"
            );
        }
        banks.set(NUM_BANKS - 1, None);
        assert!(
            banks.all_delivered(),
            "every bank told, the last of them nothing"
        );
        assert_eq!(banks.volts(NUM_BANKS - 1), None);
        assert!(BankSupplies::held_at(1.8).all_delivered());
        assert_eq!(ALL_BANKS_MASK, 0xFFFF);
    }

    #[test]
    fn the_spin2_constants_land_in_their_fields() {
        assert_eq!(P_HIGH_15K, 0x1000);
        assert_eq!(P_HIGH_FLOAT, 0x3800);
        assert_eq!(P_LOW_15K, 0x200);
        assert_eq!(P_LOW_FLOAT, 0x700);
        assert_eq!(
            pad_modes(P_HIGH_15K | P_LOW_FLOAT),
            (PadMode::Ohms15k, PadMode::Float)
        );
    }

    #[test]
    fn a_train_with_a_rate_is_the_crystal_and_a_held_one_is_none() {
        let train = PulseTrain {
            pulses: PulseSegment {
                emitted: 0,
                freq_hz: 20_000_000,
                total: None,
                since_us: 1_000,
            },
            direction: PulseDirection::Forward,
        };
        assert_eq!(crystal_of(&train), Some(20_000_000));
        assert_eq!(crystal_of(&PulseTrain::IDLE), None);
    }

    /// The core-supply window is the datasheet's `Vdd` row, 1.7 V to 1.9 V
    /// inclusive; out of reset needs `RESN` released and `VDD` inside it.
    #[rstest]
    #[case::nominal(Some(1.8), true)]
    #[case::at_the_minimum(Some(1.7), true)]
    #[case::at_the_maximum(Some(1.9), true)]
    #[case::the_modules_core_rail(Some(1.8133), true)]
    #[case::just_under(Some(1.699), false)]
    #[case::just_over(Some(1.901), false)]
    #[case::a_logic_rail(Some(3.3), false)]
    #[case::held_low(Some(1.2), false)]
    #[case::nothing(None, false)]
    fn vdd_is_in_its_window_between_the_datasheets_bounds(
        #[case] vdd_volts: Option<Volts>,
        #[case] in_window: bool,
    ) {
        assert_eq!(P2_VDD_MIN_VOLTS, 1.7);
        assert_eq!(P2_VDD_MAX_VOLTS, 1.9);
        let state = P2ResetState {
            resn: Some(Level::High),
            vdd: vdd_volts.map(|v| if v >= 1.65 { Level::High } else { Level::Low }),
            vdd_volts,
        };
        assert_eq!(state.vdd_in_window(), in_window);
        assert_eq!(state.out_of_reset(), in_window);
        assert!(
            !P2ResetState {
                resn: Some(Level::Low),
                ..state
            }
            .out_of_reset(),
            "a held reset is never out of reset"
        );
        assert!(!P2ResetState {
            resn: None,
            ..state
        }
        .out_of_reset());
    }

    #[test]
    fn out_of_reset_needs_resn_released_and_vdd_in_window() {
        let both = P2ResetState {
            resn: Some(Level::High),
            vdd: Some(Level::High),
            vdd_volts: Some(1.8),
        };
        assert!(both.out_of_reset());
        assert!(!P2ResetState {
            resn: Some(Level::Low),
            ..both
        }
        .out_of_reset());
        // A level with no voltage names nothing the window can be read
        // against.
        assert!(!P2ResetState {
            vdd_volts: None,
            ..both
        }
        .out_of_reset());
        assert!(!P2ResetState::default().out_of_reset());
        assert_eq!(named_volts(NetState::Analog(1.8)), Some(1.8));
        assert_eq!(named_volts(NetState::Driven(Level::High)), None);
        assert_eq!(named_volts(NetState::Pulled(Level::High, 10_500.0)), None);
        assert_eq!(named_volts(NetState::Floating), None);
        assert_eq!(named_volts(NetState::Analog(f64::NAN)), None);
    }
}
