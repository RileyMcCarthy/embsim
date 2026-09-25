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
//! | Pins | Role and declarations | Idle |
//! |---|---|---|
//! | `P0`..`P63` | `Signal`, bidirectional: sources and sinks, reads through [`P2_PAD_THRESHOLDS`] (0.3/0.7 × its bank's `VIO_a_b`) against `GND` | **released** — a pad out of reset floats |
//! | `VDD`, the 16 `VIO_a_b` | `PowerIn` (sensed), measured against `GND` | — |
//! | `GND` | `PowerIn` | — |
//! | `RESN`, `TEST` | `Signal` senses through [`P2_UNBANKED_INPUT_THRESHOLDS`] (`RESN` read by the START gate) | — |
//! | `XI` | a `Signal` sense: the segment a periodic net carries here **is the crystal** | — |
//! | `XO` | `Signal` output, released | the crystal driver, unused with an external clock |
//!
//! # The START gate
//!
//! A core does not run until the chip can. The chip's reset **releases**
//! the instant `RESN` reads released **and** `VDD` reads a voltage inside
//! the datasheet's core-supply window ([`P2_VDD_MIN_VOLTS`] to
//! [`P2_VDD_MAX_VOLTS`]); the chip starts the datasheet's restart delay
//! later ([`P2_RESTART_DELAY_NS`], 3 ms: "Propeller restarts 3 ms after
//! RESn transitions from low to high", Pin Descriptions, p. 6), if the
//! reset has stayed released the whole time — a release shorter than the
//! delay starts nothing, and the delay counts again from the next one.
//! That instant is the START instant: the package counts the delay on a
//! wake of its **own**, starts the core there (its clock counts from it),
//! lands the wakes it held there, and [`P2PackageHandle::start_state`]
//! reports it. Before it the handle reports `Held` with the two inputs as
//! last read — the reason — or `Restarting` with the release instant and
//! the start it leads to, and the package says so at `tracing::info` level
//! (the shape the crystal stall in `embsim-p2-qemu` reports in). A `VDD`
//! that names a level and no voltage (a strong digital source, a pull) is
//! outside the window: the package invents no voltage for it.
//!
//! The gate is the package's, and it holds **every** core the same way —
//! the QEMU core, an ISS, the native firmware image: a core's wake handler
//! and its schedules go through one entry point, the package's
//! [`WakeGate`] behind the net I/O every core is handed
//! ([`ComponentNetIo::with_wake_gate`]), whether the core schedules
//! through [`P2Pads`] or on the net I/O itself. The engine holds one wake
//! handler for the package, the package's, which counts the restart delay
//! and forwards the core's wakes once it runs. The reset state a core
//! records through [`P2Pads::on_reset`] is information, never its own
//! gate.
//!
//! The gate also waits for the package's own knowledge of the banks: the
//! reset is not counted released until every one of the sixteen `VIO_a_b`
//! senses has delivered at least once ([`BankSupplies::all_delivered`]),
//! so a core that publishes a pad at its START instant reads a populated
//! bank table, never the "nothing has told me yet" every bank starts in.
//! The senses deliver once at registration, on the engine thread, and the
//! package registers the sixteen bank senses **before** the two the gate
//! reads, so the release is counted at the reset delivery with the table
//! already told (the phase-4 review record in `NODES.md` §8 has the
//! measurements of the race this ordering closed). Every bank delivery
//! gives the gate its chance, so the condition can never stall it.
//!
//! # A brownout without a reset
//!
//! Once the core runs, the package watches its supply: `VDD` leaving its
//! window while `RESN` is not asserted — the fault a reset supervisor
//! exists to prevent — is reported as
//! [`StartState::BrownoutWithoutReset`] (with a `tracing::warn`, once) and
//! the core is held from that instant ([`P2Core::reset`]): no wake reaches
//! it again, and its pads keep what they last published. A `RESN`
//! asserted first is a reset the chip was told about, and the package
//! delivers it to the core as information.
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
//! [`embsim_board::Finding::PowerNetUnsourced`] on its net. The native
//! core's bridged pads drive the same way, through
//! [`McuComponent::host_pads`] (its `P2Core` impl), in the fast mode
//! [`NATIVE_PAD_MODE`], from the START instant on.
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
//! | `%000` | fast | [`P2_FAST_OHMS`], 17.99 Ω (fitted to the datasheet's `Voh`/`Vol` rows) |
//! | `%001` | 1.5 kΩ | [`P2_PULL_1K5_OHMS`] |
//! | `%010` | 15 kΩ | [`P2_PULL_15K_OHMS`] |
//! | `%011` | 150 kΩ | [`P2_PULL_150K_OHMS`] |
//! | `%100` | 1 mA | not mapped ([`PadDrive::CurrentSource`], see below) |
//! | `%101` | 100 µA | not mapped |
//! | `%110` | 10 µA | not mapped |
//! | `%111` | float | released |
//!
//! Source: Parallax, *Propeller 2 (P2X8C4M64P) Silicon Documentation*,
//! rev. v35, "Smart Pins" → "Pin Configuration Modes" — the
//! `%0000_CIO_HHH_LLL` logic-mode word and its `HHH`/`LLL` drive table
//! (fast, 1.5k, 15k, 150k, 1mA, 100uA, 10uA, float). The three resistive
//! modes are the resistances the document names; the fast mode is fitted
//! to the datasheet's output table ([`P2_FAST_OHMS`]).
//!
//! The current-source modes are **not** mapped: they reach a core as
//! [`PadDrive::CurrentSource`], an unimplemented path the core logs once,
//! and the pad presents nothing. The datasheet names their currents — 1 mA,
//! 100 µA, 10 µA (P2X8C4M64P Datasheet, Features, p. 2, and the smart-pin
//! mode table, p. 24) — but no compliance: how close to its bank rail the
//! source can pull its pad. `Drive::Current`, the encoding a current takes
//! (`NODES.md` §10), is an ideal injection with no shunt, so a 1 mA pad
//! into a 10 kΩ pull-down would put 10 V on a 3.3 V pad; mapping one needs
//! a compliance figure no document gives, and no caller asks for one.
//!
//! # Datasheet
//!
//! The supply figures cite Parallax, *Propeller 2 (P2X8C4M64P) Datasheet*,
//! © Parallax Inc. 2022/11/01 (`Propeller2-P2X8C4M64P-Datasheet-20221101.pdf`):
//! "System Characteristics" → "DC Characteristics" (p. 47) for the `Vdd`
//! window, and "Pin Descriptions" (p. 6) for the bank grouping and the
//! `RESN` pin.
//!
//! # What this package does not do
//!
//! - **A reset asserted after START.** A `RESN` that falls once the core
//!   runs is delivered to the core as a reset state and changes nothing
//!   else: a restart needs a core entry that re-runs the boot from the ROM
//!   (the QEMU core's machine state, a native image's statics), which no
//!   core has. The datasheet's own restart (Rebooting, p. 35) is the entry
//!   it would implement.
//! - **A bank supply changing after START.** A `VIO_a_b` that rises,
//!   drops or moves once the core runs updates the [`BankSupplies`] table
//!   and nothing else: a pad the core already drives keeps the drive it
//!   published, at the old voltage, until the core next publishes it (a
//!   guest's next pad write, a native bridge's next level, reads the table
//!   then). Re-publishing a bank's driven pads at the new voltage is a
//!   core-side re-publish of what it last drove; no board moves a bank
//!   rail after START.
//! - **The current-source pad modes** (above): no compliance figure, and
//!   no caller.

use std::sync::atomic::{AtomicU16, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

use embsim_board::{
    jesd8c01_lvcmos_thresholds, Amps, AttachError, Component, ComponentNetIo, DeadBand,
    DigitalReceiver, Level, McuComponent, Ohms, PinDecl, PinHandle, Sense, TheveninDrive,
    Thresholds, Volts, WakeGate, WakeHandler,
};
use embsim_core::virtual_clock;

/// The crate's nominal logic high, [`embsim_board::net::LOGIC_HIGH_VOLTS`]:
/// what an [`McuComponent`] on a board of its own drives at. A pad any
/// core publishes inside the package — the native core's included —
/// drives at its bank's supply instead ([`BankSupplies`]).
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

/// How long after its reset releases the chip starts: "Propeller restarts
/// 3 ms after RESn transitions from low to high" — P2X8C4M64P Datasheet,
/// "Pin Descriptions", p. 6, the `RESN` row. The package counts it from
/// the instant its reset releases — `RESN` reads released with `VDD`
/// inside its window — and starts the core at the end of it, if the reset
/// has stayed released the whole time.
pub const P2_RESTART_DELAY_NS: u64 = 3_000_000;

/// A pad's input logic threshold, **relative** to its bank's supply and
/// measured against `GND`: `Vih`, Input Logic Threshold, min `Vxxyy` ×
/// 0.3, typ × 0.5, max × 0.7 (P2X8C4M64P Datasheet, DC Characteristics,
/// p. 47) — the threshold lies somewhere in the band, so an input at or
/// below 0.3 × `Vxxyy` reads low on every part and one at or above 0.7 ×
/// `Vxxyy` high. The logic input mode names no hysteresis (the Schmitt
/// modes are a `WRPIN` choice the declaration does not see), so between
/// the two the pad reads no level ([`DeadBand::Unknown`]) and the core
/// keeps the input bit it has — what it does with a floating or fought pad.
pub const P2_PAD_THRESHOLDS: Thresholds = Thresholds::new(0.3, 0.7, 0.0, DeadBand::Unknown);

/// `RESN`'s, `TEST`'s and `XI`'s thresholds: the JEDEC JESD8C.01 3.3 V
/// LVCMOS pair ([`jesd8c01_lvcmos_thresholds`]), absolute. The datasheet's
/// one threshold row (DC Characteristics, p. 47) is given as a fraction of
/// a bank's `Vxxyy`, and these three pins belong to no bank (Pin
/// Descriptions, p. 6: `RESN` is pulled up "to 3.3 V", `TEST` tied to
/// ground, `XI` takes a crystal or an oscillator's output), so the pair
/// the engine's own dead band projects a net through stands in, stated.
pub const P2_UNBANKED_INPUT_THRESHOLDS: Thresholds = jesd8c01_lvcmos_thresholds(DeadBand::Unknown);

// ============================================================
// Pad drive strengths
// ============================================================

/// The fast driver sourcing, as `(current, drop below Vxxyy)` pairs:
/// `Voh` (relative to `Vxxyy`) −6 / −170 / −580 mV sourcing 1 / 10 /
/// 30 mA — P2X8C4M64P Datasheet, "DC Characteristics" (the table's
/// continuation), p. 48, the "Typ" column (25 °C) at a 3.3 V supply.
pub const P2_FAST_SOURCE_POINTS: [(Amps, Volts); 3] =
    [(0.001, 0.006), (0.010, 0.170), (0.030, 0.580)];

/// The fast driver sinking, as `(current, rise above GND)` pairs: `Vol`
/// (relative to GND) 15 / 160 / 510 mV sinking 1 / 10 / 30 mA — the same
/// table, p. 48, "Typ".
pub const P2_FAST_SINK_POINTS: [(Amps, Volts); 3] =
    [(0.001, 0.015), (0.010, 0.160), (0.030, 0.510)];

/// The least-squares resistance through the origin of a set of
/// `(current, drop)` points, `Σ V·I / Σ I²`: the one `R` of `V = I·R`
/// that fits them best — a Thevenin port whose open-circuit voltage is the
/// rail it drives to has that `R` as its only parameter.
pub const fn fitted_ohms(points: &[(Amps, Volts)]) -> Ohms {
    let mut volt_amps = 0.0;
    let mut amps_squared = 0.0;
    let mut k = 0;
    while k < points.len() {
        let (amps, volts) = points[k];
        volt_amps += volts * amps;
        amps_squared += amps * amps;
        k += 1;
    }
    volt_amps / amps_squared
}

/// Both of the fast driver's tables in one list, sourcing then sinking:
/// what [`P2_FAST_OHMS`] is fitted to.
const P2_FAST_POINTS: [(Amps, Volts); 6] = [
    P2_FAST_SOURCE_POINTS[0],
    P2_FAST_SOURCE_POINTS[1],
    P2_FAST_SOURCE_POINTS[2],
    P2_FAST_SINK_POINTS[0],
    P2_FAST_SINK_POINTS[1],
    P2_FAST_SINK_POINTS[2],
];

/// A pad in **fast** drive mode (`%000`), as a Thevenin impedance:
/// **17.99 Ω**, derived, not chosen.
///
/// A pad is a Thevenin port — high at its bank's `Vxxyy`, low at `GND` —
/// so its open-circuit voltages are the rails and its impedance is the one
/// parameter left. The datasheet gives the fast driver's output voltage
/// at three currents each way ([`P2_FAST_SOURCE_POINTS`],
/// [`P2_FAST_SINK_POINTS`]: P2X8C4M64P Datasheet, "DC Characteristics",
/// p. 48); the impedance is the least-squares fit of `V = I·R` to all six
/// ([`fitted_ohms`]): `Σ V·I / Σ I²` = 0.036021 W ÷ 0.002002 A² = 17.99 Ω.
/// The individual ratios run 6–19.3 Ω (a MOSFET driver is not a
/// resistor; the 1 mA points are the low end, the 30 mA source point the
/// high one) and the fit weights the larger currents where the figure
/// matters. One figure serves both levels: the pad-mode table names one
/// "fast" mode, and the two sides' own fits (19.09 Ω sourcing, 16.90 Ω
/// sinking) differ by less than the table's own spread. The ranking it
/// feeds is by factors of ten — a fast pad against the boards' 10 kΩ and
/// larger pulls, or a fight between two pads within the ratio either way
/// — so no projection on the reference boards moves from the 25 Ω
/// placeholder this replaced. The six points are the table's typical
/// column at a 3.3 V supply, and the one figure serves a 1.8 V bank too:
/// the datasheet gives no 1.8 V row, and nothing is invented for one
/// (`NODES.md` §12 item 5, the P2 task's decision (7)).
pub const P2_FAST_OHMS: Ohms = fitted_ohms(&P2_FAST_POINTS);

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
    /// `%100`: a 1 mA current source (P2X8C4M64P Datasheet, p. 24; not
    /// mapped — the module docs say why).
    Current1mA,
    /// `%101`: a 100 µA current source (the same table; not mapped).
    Current100uA,
    /// `%110`: a 10 µA current source (the same table; not mapped).
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

/// The pad a facade name (`"P0"`..`"P63"`) names, or `None` for any other
/// pin.
pub fn pad_of(name: &str) -> Option<u8> {
    PAD_NAMES
        .iter()
        .position(|pad| *pad == name)
        .and_then(|pad| u8::try_from(pad).ok())
}

/// The pin-configuration word the native core's pads drive in: `0`, fast
/// both ways (`%0000_CIO_HHH_LLL` with `HHH` = `LLL` = `%000`, Silicon
/// Documentation v35, "Pin Configuration Modes") — the word a pad holds
/// until a `WRPIN` changes it, and the HAL tables the native core bridges
/// name no drive mode that would.
pub const NATIVE_PAD_MODE: u32 = P_HIGH_FAST | P_LOW_FAST;

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
const OTHER_PINS: [PinDecl; 6] = [
    PinDecl::power_in("GND"),
    // "All VDD and all Vxxyy pins must have closely-located bypass caps to
    // GND" (Minimal Connections, p. 7).
    PinDecl::power_in("VDD").with_reference("GND"),
    PinDecl::digital_in("RESN", P2_UNBANKED_INPUT_THRESHOLDS).with_reference("GND"),
    PinDecl::digital_in("TEST", P2_UNBANKED_INPUT_THRESHOLDS).with_reference("GND"),
    PinDecl::digital_in("XI", P2_UNBANKED_INPUT_THRESHOLDS).with_reference("GND"),
    PinDecl::digital_out("XO").with_idle(None),
];

/// The chip's facade: 64 bidirectional pads that idle released, each
/// reading through [`P2_PAD_THRESHOLDS`] of its own bank's `VIO_a_b` pin
/// against `GND`, and the 22 package pins — the supplies as power-in pins
/// measured against `GND`, `RESN` and `TEST` sensed, `XI` sensed (the rate
/// of the square wave on it the crystal), `XO` a released output. This is
/// what the P2-EC32MB's `U100` slot expects, in both directions.
pub fn p2x8c4m64p_pins() -> Vec<PinDecl> {
    let mut pins = Vec::with_capacity(NUM_PINS);
    for (pad, name) in PAD_NAMES.iter().enumerate() {
        pins.push(
            PinDecl::digital_out(name)
                .with_idle(None)
                .with_thresholds(P2_PAD_THRESHOLDS)
                .with_supply(bank_pin_name(pad / PADS_PER_BANK))
                .with_reference("GND"),
        );
    }
    pins.extend(OTHER_PINS);
    for name in BANK_PINS {
        pins.push(PinDecl::power_in(name).with_reference("GND"));
    }
    pins
}

// ============================================================
// What the package delivers to its core
// ============================================================

/// The reset inputs as the package reads them: `RESN` projected through
/// its declared thresholds ([`P2_UNBANKED_INPUT_THRESHOLDS`]) — `Some(High)`
/// a released reset, `Some(Low)` a held one, `None` nothing reaching the
/// pin or a voltage the pair guarantees neither level at — and `VDD` as the
/// voltage it is handed against `GND`, which is what the START gate holds
/// the core to. `VDD` declares no thresholds: it is handed volts only.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct P2ResetState {
    /// `RESN` (active low).
    pub resn: Option<Level>,
    /// `VDD` against `GND`, `None` when it names no voltage (nothing
    /// reaches the pin, nothing holds `GND`, a clock).
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
    /// The reset released and the chip is counting out the datasheet's
    /// restart delay ([`P2_RESTART_DELAY_NS`]): the core starts at
    /// `starts_at_ns` if the reset stays released until then.
    Restarting {
        /// The instant `RESN` read released with `VDD` inside its window.
        released_at_ns: u64,
        /// The START instant it leads to.
        starts_at_ns: u64,
    },
    /// The core was started at this virtual instant, and runs from it.
    Started {
        /// The START instant, nanoseconds.
        at_ns: u64,
    },
    /// The core was started, then `VDD` left its window at `at_ns` with
    /// `RESN` not asserted — a brownout without a reset, the fault a reset
    /// supervisor exists to prevent. The package holds the core from that
    /// instant ([`P2Core::reset`]) and reports it once (`tracing::warn`).
    BrownoutWithoutReset {
        /// The START instant the core ran from.
        started_at_ns: u64,
        /// The instant `VDD` left its window.
        at_ns: u64,
        /// The reset inputs as the package read them then.
        reset: P2ResetState,
    },
}

/// The crystal `XI` is handed: the rate of the square wave it carries
/// ([`Sense::periodic`]), or `None` for anything else — a held segment, a
/// level, a floating or fought pin (no clock reaches it).
pub fn crystal_of(sense: &Sense) -> Option<u64> {
    sense
        .periodic
        .map(|clock| clock.segment.freq_hz)
        .filter(|&hz| hz > 0)
        .map(u64::from)
}

// ============================================================
// The bank supplies
// ============================================================

/// A bank's supply as an `f64`'s bits: NaN is no voltage, which no supply
/// can name (a sense's voltage is finite or absent).
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
/// The core's wake handler, shared with the package's forwarder. The lock
/// is never contended: the engine thread is the only caller.
type CoreWake = Arc<Mutex<WakeHandler>>;

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
    /// [`Component::start`] has run: the system is live, and the reset
    /// counts as released — the restart delay starts — the moment the
    /// inputs allow.
    start_requested: bool,
    /// The instant the reset released — `RESN` released and `VDD` inside
    /// its window, with the system live and every bank told — while the
    /// core waits out the restart delay; `None` while the reset holds, and
    /// cleared if it re-asserts before the delay is out.
    released_at_ns: Option<u64>,
    /// The START instant, once the core was started.
    started_at_ns: Option<u64>,
    /// The instant `VDD` left its window after the START instant with
    /// `RESN` not asserted, and the inputs then: the core is held from it.
    brownout: Option<(u64, P2ResetState)>,
    /// The core's wake handler, however it registered it: through
    /// [`P2Pads::on_wake_ns`], or on the net I/O the package handed it
    /// ([`CoreWakes`]).
    core_wake: Option<CoreWake>,
    /// Wakes the core asked for before it was started; forwarded at the
    /// START instant, no earlier.
    held_wakes: Vec<u64>,
    /// Periodic wakes the core asked for before it was started; armed at
    /// the START instant, their periods counted from there.
    held_periods: Vec<u64>,
}

/// The core's time, as the package hosts it: the [`WakeGate`] behind the
/// net I/O every core is handed ([`P2Pads`], and the native core's
/// [`ComponentNetIo`]). A wake handler is the core's, delivered by the
/// package's own forwarder once the core is started; a wake asked for
/// before the START instant is held and lands there (or at its own instant
/// if that is later); one asked for after goes to the engine at once. One
/// entry point, so every core — the QEMU target, an ISS, the native
/// firmware image — is held the same way.
struct CoreWakes {
    gate: Arc<Mutex<Gate>>,
    /// The package's own net I/O: where a wake goes once the core runs.
    io: ComponentNetIo,
}

impl WakeGate for CoreWakes {
    fn on_wake_ns(&self, handler: WakeHandler) {
        self.gate
            .lock()
            .expect("start gate never poisoned")
            .core_wake = Some(Arc::new(Mutex::new(handler)));
    }

    fn schedule_at_ns(&self, at_ns: u64) {
        let mut gate = self.gate.lock().expect("start gate never poisoned");
        if gate.brownout.is_some() {
            // A held core is woken no more.
        } else if gate.started_at_ns.is_some() {
            drop(gate);
            self.io.schedule_at_ns(at_ns);
        } else {
            gate.held_wakes.push(at_ns);
        }
    }

    fn schedule_every_ns(&self, period_ns: u64) {
        let mut gate = self.gate.lock().expect("start gate never poisoned");
        if gate.brownout.is_some() {
            // A held core is woken no more.
        } else if gate.started_at_ns.is_some() {
            drop(gate);
            self.io.schedule_every_ns(period_ns);
        } else {
            gate.held_periods.push(period_ns);
        }
    }
}

/// A core's whole surface: its 64 pads, the package's two facts (the
/// crystal on `XI`, the reset inputs), the bank supplies, and wake
/// scheduling through the package's START gate. Handed to
/// [`P2Core::attach`] once, by the package; a core keeps the handles it
/// needs and subscribes to what it wants delivered. (The native core takes
/// the underlying net I/O whole — see [`McuComponent`]'s `P2Core` impl —
/// and that I/O's wakes go through the same gate.)
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
/// The same holds for a core that schedules on the net I/O it was handed:
/// that I/O is the package's, its wakes routed through the gate
/// ([`ComponentNetIo::with_wake_gate`]).
///
/// Everything here goes through the one interface a node has
/// (`NODES.md` §11): pad handles publish drives, pad senses deliver the
/// resolved net, and the package facts are the package's own senses
/// projected once and fanned out. A core never reaches a net it has no
/// pad on.
#[derive(Clone)]
pub struct P2Pads {
    /// The package's net I/O, its wakes routed through the START gate
    /// ([`CoreWakes`]).
    io: ComponentNetIo,
    subscribers: Arc<Mutex<Subscribers>>,
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

    /// Subscribe to the level pad `pin` reads: its sense projected through
    /// the pad's declared thresholds, [`P2_PAD_THRESHOLDS`] of its bank's
    /// `VIO_a_b` at the instant, chosen by the level it last read —
    /// delivered once at registration and on every change of the pad's
    /// net, on the engine thread. `None` is a pad that reads no level: a
    /// floating net, a voltage inside the band the datasheet guarantees
    /// neither level in, a bank whose supply names no voltage, a clock.
    pub fn on_pad_sense(
        &self,
        pin: u8,
        callback: impl Fn(Option<Level>) + Send + 'static,
    ) -> Result<(), AttachError> {
        let receiver = DigitalReceiver::new(self.io.pin(pin_name(pin))?);
        self.io
            .on_sense(pin_name(pin), move |sense| callback(receiver.read(&sense)))
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
        self.io.on_wake_ns(callback);
    }

    /// Arm a one-shot wake at an absolute virtual nanosecond. Before the
    /// core is started the request is held and lands at the START instant
    /// (or at `at_ns` if that is later); after, it is the engine's at once.
    pub fn schedule_at_ns(&self, at_ns: u64) {
        self.io.schedule_at_ns(at_ns);
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
    /// path only) **and** the datasheet's restart delay
    /// ([`P2_RESTART_DELAY_NS`]) has run out since the reset released —
    /// `RESN` released and `VDD` inside its window the whole time. The
    /// current virtual nanosecond is the START instant the core's clock
    /// counts from. Runs before any wake the core asked for is delivered,
    /// on the engine thread, inside the package's own wake.
    fn start(&mut self) {}

    /// Stop executing: the chip left its operating conditions while this
    /// core ran — `VDD` out of its window with `RESN` not asserted, the
    /// brownout a reset supervisor exists to prevent
    /// ([`StartState::BrownoutWithoutReset`]). Called once, on the engine
    /// thread, at the instant the package reads it; the package delivers
    /// no wake to the core after it and never starts it again. What the
    /// silicon does out of its window the datasheet does not say, so a
    /// core changes nothing it has published — its pads keep their drives
    /// — and runs nothing more. A core that cannot stop (the native
    /// firmware image, on a thread of its own) says so.
    fn reset(&mut self) {}
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
/// gate holds the firmware entry ([`McuComponent`]'s `start`) and every
/// wake the core asks for, as it holds any core's.
///
/// What it is handed is the package's **whole** net I/O — the handle table
/// carries `XI`, `XO`, `RESN`, `VDD` and the `VIO` pins beside the 64 pads
/// — and the narrowing to the pads is by what the core names, not by a
/// filter: its HAL tables name pads (`P0`, `P2`, …) and nothing else. The
/// interface is the same one every node has, so nothing is reachable that
/// a node could not reach; the package-level facts still arrive through
/// the package's own senses, as for any core. Its wake handler and its
/// schedules go through the START gate ([`ComponentNetIo::with_wake_gate`]).
///
/// Its pads are the package's like any core's ([`McuComponent::host_pads`]):
/// a bridged pad drives through [`BankSupplies::pad_drive`] in
/// [`NATIVE_PAD_MODE`] — high at its bank's `VIO_a_b`, at
/// [`P2_FAST_OHMS`], nothing in a bank whose supply names no voltage, the
/// bank reported once — and nothing before the START instant, where the
/// package runs the core's `start` and the bridged outputs present their
/// power-on state (a chip in reset floats every pad).
impl P2Core for McuComponent {
    fn attach(&mut self, pads: P2Pads) -> Result<(), AttachError> {
        let banks = pads.bank_supplies();
        self.host_pads(Arc::new(move |pin, level| {
            let pad = pad_of(pin)?;
            match banks.pad_drive(pad, NATIVE_PAD_MODE, true, level == Level::High) {
                PadDrive::Thevenin(drive) => Some(drive),
                PadDrive::Released | PadDrive::CurrentSource(_) => None,
            }
        }));
        Component::attach(self, pads.io)
    }

    fn start(&mut self) {
        Component::start(self);
    }

    /// The native firmware runs on a thread of its own and cannot be
    /// stopped from here: the package stops delivering its wakes, and says
    /// that the firmware itself runs on.
    fn reset(&mut self) {
        tracing::warn!(
            mcu = self.name(),
            "p2: the native core cannot be held — its firmware runs on its own thread and \
             keeps running; only its wakes stop"
        );
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
        let (started, released, brownout) = {
            let gate = self.gate.lock().expect("start gate never poisoned");
            (gate.started_at_ns, gate.released_at_ns, gate.brownout)
        };
        match (started, released) {
            (Some(started_at_ns), _) => match brownout {
                Some((at_ns, reset)) => StartState::BrownoutWithoutReset {
                    started_at_ns,
                    at_ns,
                    reset,
                },
                None => StartState::Started {
                    at_ns: started_at_ns,
                },
            },
            (None, Some(released_at_ns)) => StartState::Restarting {
                released_at_ns,
                starts_at_ns: released_at_ns.saturating_add(P2_RESTART_DELAY_NS),
            },
            (None, None) => StartState::Held {
                reset: self.reset(),
            },
        }
    }

    /// The START instant, once the core was started (whether or not it
    /// was held by a brownout since).
    pub fn started_at_ns(&self) -> Option<u64> {
        match self.start_state() {
            StartState::Started { at_ns } => Some(at_ns),
            StartState::BrownoutWithoutReset { started_at_ns, .. } => Some(started_at_ns),
            StartState::Held { .. } | StartState::Restarting { .. } => None,
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

    /// Whether the reset is released as far as the gate is concerned: the
    /// system is live, `RESN` reads released with `VDD` inside its window,
    /// and every bank sense has delivered ([`BankSupplies::all_delivered`],
    /// so the core's first pad publish reads a populated table).
    fn released(gate: &Gate, state: &Mutex<PackageState>, banks: &BankSupplies) -> bool {
        let reset = state.lock().expect("package state never poisoned").reset;
        gate.start_requested && reset.out_of_reset() && banks.all_delivered()
    }

    /// Follow the reset inputs before the START instant: the instant the
    /// reset releases, count out the restart delay — arm the package's own
    /// wake at `now + P2_RESTART_DELAY_NS` — and if the reset re-asserts
    /// before it is out, forget the release, so a glitch shorter than the
    /// delay starts nothing. Called on every `RESN`, `VDD` and bank
    /// delivery (the engine thread) and at `Component::start` (the
    /// starting thread), under the gate's lock, so a release is counted
    /// exactly once.
    fn follow_reset(
        gate: &Mutex<Gate>,
        state: &Mutex<PackageState>,
        banks: &BankSupplies,
        io: &ComponentNetIo,
        now: u64,
    ) {
        let arm_at = {
            let mut gate = gate.lock().expect("start gate never poisoned");
            if gate.started_at_ns.is_some() {
                return;
            }
            match (Self::released(&gate, state, banks), gate.released_at_ns) {
                (true, None) => {
                    gate.released_at_ns = Some(now);
                    Some(now.saturating_add(P2_RESTART_DELAY_NS))
                }
                (false, Some(released_at_ns)) => {
                    gate.released_at_ns = None;
                    tracing::info!(
                        released_at_ns,
                        at_ns = now,
                        "p2: the reset re-asserted inside the restart delay; the chip does not \
                         start until it releases again"
                    );
                    None
                }
                _ => None,
            }
        };
        if let Some(at_ns) = arm_at {
            tracing::info!(
                released_at_ns = now,
                starts_at_ns = at_ns,
                "p2: the reset released; the chip restarts after the datasheet's 3 ms delay"
            );
            if virtual_clock::is_initialized() {
                io.schedule_at_ns(at_ns);
            } else {
                tracing::warn!(
                    "p2: no virtual clock to count the restart delay on; the core stays held"
                );
            }
        }
    }

    /// Watch the running chip's supply: `VDD` leaving its window after the
    /// START instant while `RESN` is not asserted (it reads high or
    /// nothing) is a brownout without a reset. Record it once, say so, and
    /// hold the core ([`P2Core::reset`]): no wake reaches it again. A
    /// `RESN` asserted first is a reset the chip was told about, and the
    /// package does nothing with it after START but deliver it.
    fn watch_brownout(gate: &Mutex<Gate>, core: &Mutex<C>, reset: P2ResetState, now: u64) {
        {
            let mut gate = gate.lock().expect("start gate never poisoned");
            if gate.started_at_ns.is_none()
                || gate.brownout.is_some()
                || reset.vdd_in_window()
                || reset.resn == Some(Level::Low)
            {
                return;
            }
            gate.brownout = Some((now, reset));
        }
        tracing::warn!(
            at_ns = now,
            ?reset,
            vdd_window = format_args!("{P2_VDD_MIN_VOLTS}..={P2_VDD_MAX_VOLTS} V"),
            "p2: brownout without reset — VDD left its window while the core ran and RESN was \
             not asserted; the core is held from here"
        );
        core.lock().expect("core never poisoned").reset();
    }

    /// The package's own wake, before the START instant: if the reset
    /// released at least the restart delay ago and is released still,
    /// start the core at `now` and forward every wake it held. Returns
    /// whether the core is started (now or before).
    fn try_begin(
        gate: &Mutex<Gate>,
        core: &Mutex<C>,
        state: &Mutex<PackageState>,
        banks: &BankSupplies,
        io: &ComponentNetIo,
        now: u64,
    ) -> bool {
        let (held, periods) = {
            let mut gate = gate.lock().expect("start gate never poisoned");
            if gate.started_at_ns.is_some() {
                return true;
            }
            let Some(released_at_ns) = gate.released_at_ns else {
                return false;
            };
            if now < released_at_ns.saturating_add(P2_RESTART_DELAY_NS)
                || !Self::released(&gate, state, banks)
            {
                return false;
            }
            gate.started_at_ns = Some(now);
            (
                std::mem::take(&mut gate.held_wakes),
                std::mem::take(&mut gate.held_periods),
            )
        };
        tracing::info!(
            at_ns = now,
            "p2: START — the restart delay after the reset released is out; the core runs \
             from here"
        );
        // Before any wake the core asked for is delivered: the core's
        // clock counts from this instant.
        core.lock().expect("core never poisoned").start();
        for at_ns in held {
            io.schedule_at_ns(at_ns.max(now));
        }
        for period_ns in periods {
            io.schedule_every_ns(period_ns);
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

        // The package's wake: the one handler the engine holds for this
        // node. Before the START instant every wake is the package's own —
        // the end of a restart delay — since the gate holds the core's;
        // from it on every wake is the core's. The core's handler — however
        // the core registered it, through `P2Pads` or on the net I/O it was
        // handed, both of which route through the gate (`CoreWakes`) — is
        // delivered from here.
        {
            let gate = Arc::clone(&self.gate);
            let core = Arc::clone(&self.core);
            let state = Arc::clone(&self.state);
            let banks = self.banks.clone();
            let io_for_gate = io.clone();
            io.on_wake_ns(move |now| {
                let callback = {
                    let gate = gate.lock().expect("start gate never poisoned");
                    if gate.brownout.is_some() {
                        return;
                    }
                    gate.started_at_ns.and(gate.core_wake.clone())
                };
                match callback {
                    Some(callback) => (callback.lock().expect("core wake never poisoned"))(now),
                    None => {
                        Self::try_begin(&gate, &core, &state, &banks, &io_for_gate, now);
                    }
                }
            });
        }

        let core_io = io.clone().with_wake_gate(Arc::new(CoreWakes {
            gate: Arc::clone(&self.gate),
            io: io.clone(),
        }));
        let subscribers = Arc::new(Mutex::new(Subscribers::default()));
        self.core
            .lock()
            .expect("core never poisoned")
            .attach(P2Pads {
                io: core_io,
                subscribers: Arc::clone(&subscribers),
                banks: self.banks.clone(),
            })?;
        // The core has subscribed to what it wants; freeze the lists so
        // delivery never holds a lock across a callback.
        let Subscribers { crystal, reset } =
            std::mem::take(&mut *subscribers.lock().expect("subscriber list never poisoned"));
        let crystal: Arc<[CrystalCallback]> = crystal.into();
        let reset: Arc<[ResetCallback]> = reset.into();

        // XI: the rate of the square wave here is the crystal. One
        // projection, then every subscriber — on a change of crystal only,
        // so a net whose levels move under one segment tells no one.
        {
            let state = Arc::clone(&self.state);
            io.on_sense("XI", move |sensed| {
                let hz = crystal_of(&sensed);
                {
                    let mut state = state.lock().expect("package state never poisoned");
                    if state.crystal_hz == hz {
                        return;
                    }
                    state.crystal_hz = hz;
                }
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
            let io_for_gate = io.clone();
            io.on_sense(name, move |sensed| {
                banks.set(bank, sensed.volts);
                Self::follow_reset(
                    &gate,
                    &state,
                    &banks,
                    &io_for_gate,
                    virtual_clock::virtual_ns(),
                );
            })?;
        }

        // RESN and VDD: each sense updates its half, delivers the pair,
        // and the gate follows it — a release starts the restart delay, a
        // re-assertion inside it cancels it.
        for (pin, is_resn) in [("RESN", true), ("VDD", false)] {
            let state = Arc::clone(&self.state);
            let reset = Arc::clone(&reset);
            let gate = Arc::clone(&self.gate);
            let core = Arc::clone(&self.core);
            let banks = self.banks.clone();
            let io_for_gate = io.clone();
            let receiver = DigitalReceiver::new(io.pin(pin)?);
            io.on_sense(pin, move |sensed| {
                let snapshot = {
                    let mut state = state.lock().expect("package state never poisoned");
                    if is_resn {
                        state.reset.resn = receiver.read(&sensed);
                    } else {
                        state.reset.vdd_volts = sensed.volts;
                    }
                    state.reset
                };
                for callback in reset.iter() {
                    callback(snapshot);
                }
                let now = virtual_clock::virtual_ns();
                Self::follow_reset(&gate, &state, &banks, &io_for_gate, now);
                Self::watch_brownout(&gate, &core, snapshot, now);
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
        Self::follow_reset(&self.gate, &self.state, &self.banks, &io, now);
        let released = self
            .gate
            .lock()
            .expect("start gate never poisoned")
            .released_at_ns
            .is_some();
        if !released {
            let reset = self
                .state
                .lock()
                .expect("package state never poisoned")
                .reset;
            tracing::info!(
                ?reset,
                vdd_window = format_args!("{P2_VDD_MIN_VOLTS}..={P2_VDD_MAX_VOLTS} V"),
                "p2: the core is held at start: RESN must read released and VDD a voltage \
                 inside its window; the chip restarts 3 ms after they do"
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embsim_board::PeriodicSchedule;
    use rstest::rstest;

    #[test]
    fn the_facade_is_the_ec32mb_u100_slot() {
        let pins = p2x8c4m64p_pins();
        assert_eq!(pins.len(), 86);
        assert_eq!(NUM_PINS, 86);
        assert_eq!(NUM_BANKS, 16);
        assert_eq!(pins[0].number, "P0");
        assert_eq!(pins[63].number, "P63");
        for (index, pad) in pins[..NUM_PADS].iter().enumerate() {
            assert!(
                pad.reads_when_subscribed(),
                "{} is bidirectional",
                pad.number
            );
            assert_eq!(pad.idle, None, "{}", pad.number);
            assert_eq!(pad.thresholds, Some(P2_PAD_THRESHOLDS), "{}", pad.number);
            assert_eq!(pad.supply, Some(bank_pin_name(index / PADS_PER_BANK)));
            assert_eq!(pad.reference, Some("GND"));
        }
        let pin = |name: &str| pins.iter().find(|p| p.number == name).copied().unwrap();
        let digital = Some(embsim_board::SenseKind::Digital);
        assert_eq!(pin("XI").senses_at_build(), digital);
        assert!(pin("XO").drives());
        assert_eq!(pin("XO").idle, None);
        assert_eq!(pin("RESN").senses_at_build(), digital);
        assert_eq!(pin("TEST").senses_at_build(), digital);
        for rail in ["VDD", "GND", "VIO_0_3", "VIO_56_59", "VIO_60_63"] {
            assert_eq!(pin(rail).role, embsim_board::PinRole::PowerIn, "{rail}");
        }
        for rail in ["VDD", "VIO_0_3", "VIO_60_63"] {
            assert_eq!(pin(rail).reference, Some("GND"), "{rail}");
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
    /// the datasheet fit, float is released, and a current source is the
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

    /// The fast pad's impedance is the least-squares fit of the
    /// datasheet's six output figures, and it lies inside the span of
    /// their individual ratios at the 10 and 30 mA rows.
    #[test]
    fn the_fast_impedance_is_the_fit_of_the_datasheet_figures() {
        let ratio = |(amps, volts): (Amps, Volts)| volts / amps;
        assert!((P2_FAST_OHMS - 0.036_021 / 0.002_002).abs() < 1e-12);
        assert!((P2_FAST_OHMS - 17.9925).abs() < 1e-4, "{P2_FAST_OHMS}");
        let rows: Vec<Ohms> = P2_FAST_SOURCE_POINTS[1..]
            .iter()
            .chain(&P2_FAST_SINK_POINTS[1..])
            .map(|&p| ratio(p))
            .collect();
        let (lo, hi) = rows
            .iter()
            .fold((f64::MAX, f64::MIN), |(lo, hi), &r| (lo.min(r), hi.max(r)));
        assert!((16.0..=19.34).contains(&lo) && (16.0..=19.34).contains(&hi));
        assert!(lo <= P2_FAST_OHMS && P2_FAST_OHMS <= hi);
        assert!((fitted_ohms(&P2_FAST_SOURCE_POINTS) - 19.0869).abs() < 1e-4);
        assert!((fitted_ohms(&P2_FAST_SINK_POINTS) - 16.8981).abs() < 1e-4);
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
    fn a_square_wave_with_a_rate_is_the_crystal_and_a_held_one_is_none() {
        let clock = |freq_hz| Sense {
            volts: None,
            periodic: Some(embsim_board::PeriodicSense {
                hi: Some(0.8),
                lo: Some(0.0),
                segment: PeriodicSchedule {
                    emitted: 0,
                    freq_hz,
                    total: None,
                    since_ns: 1_000_000,
                },
            }),
            at_ns: 0,
        };
        let at = |volts| Sense {
            volts,
            periodic: None,
            at_ns: 0,
        };
        assert_eq!(crystal_of(&clock(20_000_000)), Some(20_000_000));
        assert_eq!(crystal_of(&clock(0)), None);
        assert_eq!(crystal_of(&at(Some(3.3))), None);
        assert_eq!(crystal_of(&at(None)), None);
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
            vdd_volts: Some(1.8),
        };
        assert!(both.out_of_reset());
        assert!(!P2ResetState {
            resn: Some(Level::Low),
            ..both
        }
        .out_of_reset());
        // A `VDD` that names no voltage names nothing the window can be
        // read against.
        assert!(!P2ResetState {
            vdd_volts: None,
            ..both
        }
        .out_of_reset());
        assert!(!P2ResetState::default().out_of_reset());
    }
}
