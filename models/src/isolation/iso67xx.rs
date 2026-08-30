//! Model: the TI **ISO674x / ISO673x / ISO672x** general-purpose digital
//! isolator family — one parameterized part covering the channel counts,
//! channel directions, and default-output options the family ships.
//!
//! ```text
//!            side 1                   barrier                   side 2
//!            ──────                   ───────                   ──────
//!   VCC1 ──────┤                         ║                         ├────── VCC2
//!   INA  ──────┤ input buffer ═══════════║═══════════ output buffer ├────── OUTA
//!   OUTD ──────┤ output buffer ══════════║════════════ input buffer ├────── IND
//!   EN1  ──────┤ (gates side-1 outputs)  ║  (gates side-2 outputs) ─┤────── EN2
//!   GND1 ──────┤                         ║                         ├────── GND2
//! ```
//!
//! # Datasheet provenance
//!
//! | Devices | Document | Revision |
//! |---|---|---|
//! | ISO6740, ISO6741, ISO6742 (+F) | TI **SLLSFJ6G** | Dec 2019, revised Jan 2023 |
//! | ISO6731 (+F) | TI **SLASEY9B** | Dec 2019, revised Feb 2023 |
//! | ISO6720B, ISO6721B, ISO6721RB (+F) | TI **SLLSFJ0F** | Jan 2020, revised Feb 2023 |
//!
//! What each governs:
//!
//! - **Pinouts** — [`Variant`]'s tables are the pin-configuration figures and
//!   pin-function tables: SLLSFJ6G Figures 6-1/6-2/6-3 and Table 6-1 (DW-16),
//!   SLASEY9B Figure 5-1 and Table 5-1 (DW-16), SLLSFJ0F Figures 5-1/5-2/5-3
//!   and Table 5-1 (D-8).
//! - **Channel behavior and every power case** — SLLSFJ6G **§9.4 Table 9-2,
//!   "Function Table"** (SLLSFJ0F §8.4 Table 8-2 is the same table for the
//!   dual-channel parts). Reproduced by [`Iso67xx`] row for row; see
//!   "Function table" below.
//! - **Powered-up / powered-down thresholds** — Table 9-2 note (1): `PU` is
//!   `VCC >= 1.71 V`, `PD` is `VCC <= 1.05 V`. [`DEFAULT_SUPPLY_MIN_VOLTS`].
//! - **Input thresholds** — SLLSFJ6G §7.3: `V_IH = 0.7 x VCCI`,
//!   `V_IL = 0.3 x VCCI`. [`DEFAULT_VIH_RATIO`] / [`DEFAULT_VIL_RATIO`].
//! - **Output drive strength** — SLLSFJ6G §7.11 (3.3-V supply):
//!   `V_OH >= VCCO - 0.2 V` at `I_OH = -2 mA` and `V_OL <= 0.2 V` at
//!   `I_OL = 2 mA`, so the worst-case output source impedance is
//!   `0.2 V / 2 mA = 100 Ohm`. [`DEFAULT_OUTPUT_IMPEDANCE_OHMS`].
//! - **The `F` suffix** — SLLSFJ6G §3: "In the event of input power or signal
//!   loss, the default output is high for devices without suffix F and low for
//!   devices with suffix F." [`Config::fail_safe`].
//!
//! ## Function table
//!
//! Table 9-2, and how each row lands here. `VCCI` is the *input* side's
//! supply, `VCCO` the *output* side's — which side is which is per channel,
//! because these parts mix directions.
//!
//! | VCCI | VCCO | INx | ENx | Datasheet OUTx | Modeled as |
//! |---|---|---|---|---|---|
//! | PU | PU | H / L | H or open | follows the input | drives the input's level |
//! | PU | PU | open | H or open | default | drives the default level |
//! | X | PU | X | L | Z | releases the pin (high-Z) |
//! | PD | PU | X | H or open | default | drives the default level |
//! | X | PD | X | X | undetermined | releases the pin (high-Z) |
//!
//! Two rows deserve their reasoning stated rather than assumed:
//!
//! - **"INx open" is any input the engine will not put a level on.** A
//!   floating net, a fought-over net, and a node voltage inside the
//!   `V_IL`..`V_IH` dead band all mean the same thing to the input buffer, and
//!   the datasheet's answer for all of them is the default output state. This
//!   is what makes an isolator fed by a dead upstream part still present a
//!   *defined* output — the behavior the F suffix is bought for.
//! - **"Undetermined" is modeled as high impedance.** With its own supply
//!   down, the output buffer has nothing to drive from, and inventing a level
//!   for it would hide the failure. Releasing lets the engine report the truth
//!   (`FloatingSense`, or whatever the board's own pull does), which is the
//!   same choice [`crate::ads122u04_component`] makes for an unpowered chip.
//!
//! # Channel roles
//!
//! A channel carries a **level** by default. Two other roles exist because the
//! isolator is a hop on a path, and an opaque hop breaks the path:
//!
//! - [`ChannelRole::Pulse`] — the input pin declares
//!   [`StreamRole::PulseSink`], the output pin [`StreamRole::PulseSource`],
//!   and a received [`PulseTrain`] is republished verbatim. This is what lets
//!   a step clock cross the barrier: the segment carries its own anchor and
//!   count, so relaying it costs **one engine event per rate change**, not one
//!   per edge, and the downstream count stays bit-identical to the firmware's.
//! - [`ChannelRole::Serial`] — the input pin is a stream
//!   [`StreamRole::Consumer`], the output a [`StreamRole::Producer`], and
//!   bytes are relayed one for one. A serial channel does **not** drive the
//!   output pin's level: the pacer owns that pin, and the producer's idle
//!   drive is the engine's.
//!
//! # Deliberate simplifications (not modeled)
//!
//! - **Propagation delay** (11 ns typical, SLLSFJ6G §7.18), pulse-width
//!   distortion, channel-to-channel skew, and the **default output delay time
//!   from input power loss**. A level crosses in the same engine iteration it
//!   arrives.
//! - **CMTI** (±150 kV/µs typical) and every other isolation-barrier
//!   characteristic. An isolator's *isolation* is what a netlist-structural
//!   engine gets for free by never connecting the two sides' nets; its
//!   *rating* is not a behavior.
//! - **The undetermined supply windows** — Table 9-2 note (2) leaves the
//!   outputs undefined for `1.05 V < VCC < 1.71 V` and
//!   `1.89 V < VCC < 2.25 V`. This model has one threshold: at or above
//!   [`Config::supply_min_volts`] the side is up, below it the side is down.
//!   A rail parked in either window therefore gets a defined answer where the
//!   datasheet gives none.
//! - **A strongly driven input weakly powering a floating VCC** through the
//!   internal protection diode (Table 9-2 note (3)). An unpowered side stays
//!   unpowered here however hard its inputs are driven.
//! - **Supply voltage is read as the VCC net's own node voltage**, not as
//!   `VCCx - GNDx`. The two sides' grounds are separate nets in the netlist
//!   and the engine solves both against one global reference, so an isolated
//!   ground sitting at a different potential is not represented — the same
//!   simplification [`crate::ads122u04_component`] makes.
//! - **Supply current, level translation limits, ESD, and thermals.**

use std::sync::{Arc, Mutex};

use embsim_board::{
    AttachError, Component, ComponentNetIo, Level, NetState, Ohms, PinDecl, PinHandle, PinKind,
    PulseTrain, PulseTx, StreamRole, TheveninDrive, Volts,
};

use super::{
    level_drive, rail_volts, require_positive, supply_up, threshold_level, PartConfigError,
};

// ============================================================
// Datasheet constants
// ============================================================

/// Supply at or above which a side counts as **powered up**: `PU` is
/// `VCC >= 1.71 V` (SLLSFJ6G §9.4 Table 9-2, note 1 — the same number as the
/// rising UVLO threshold maximum in §7.3).
pub const DEFAULT_SUPPLY_MIN_VOLTS: Volts = 1.71;

/// Nominal supply used to project thresholds and drive levels while a VCC net
/// has no numeric solve.
pub const DEFAULT_NOMINAL_SUPPLY_VOLTS: Volts = 3.3;

/// `V_IH = 0.7 x VCCI` (SLLSFJ6G §7.3 Recommended Operating Conditions).
pub const DEFAULT_VIH_RATIO: f64 = 0.7;

/// `V_IL = 0.3 x VCCI` (SLLSFJ6G §7.3 Recommended Operating Conditions).
pub const DEFAULT_VIL_RATIO: f64 = 0.3;

/// Worst-case output source impedance: `V_OH >= VCCO - 0.2 V` at
/// `I_OH = -2 mA` and `V_OL <= 0.2 V` at `I_OL = 2 mA` (SLLSFJ6G §7.11,
/// 3.3-V supply) — `0.2 V / 2 mA = 100 Ohm`.
///
/// This is a *bound*, not a measurement: the real output stage is stiffer.
/// It is the default because a guaranteed number is the only one the
/// datasheet gives, and it stays well clear of the engine's
/// `ESCALATION_IMPEDANCE_RATIO` against any ordinary pull-up.
pub const DEFAULT_OUTPUT_IMPEDANCE_OHMS: Ohms = 100.0;

// ============================================================
// Channels and sides
// ============================================================

/// One isolation channel. The family names channels `A`..`D` and each carries
/// its own direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Channel {
    /// Channel A.
    A,
    /// Channel B.
    B,
    /// Channel C.
    C,
    /// Channel D.
    D,
}

impl Channel {
    /// Dense index, for the per-channel arrays.
    const fn index(self) -> usize {
        match self {
            Channel::A => 0,
            Channel::B => 1,
            Channel::C => 2,
            Channel::D => 3,
        }
    }

    /// Name as the datasheet spells it.
    pub const fn label(self) -> &'static str {
        match self {
            Channel::A => "A",
            Channel::B => "B",
            Channel::C => "C",
            Channel::D => "D",
        }
    }

    /// Every channel the family can name, in order.
    pub const ALL: [Channel; 4] = [Channel::A, Channel::B, Channel::C, Channel::D];
}

/// Which galvanically isolated side of the part a pin belongs to.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Side {
    /// Side 1: `VCC1` / `GND1` / `EN1`.
    One,
    /// Side 2: `VCC2` / `GND2` / `EN2`.
    Two,
}

impl Side {
    const fn index(self) -> usize {
        match self {
            Side::One => 0,
            Side::Two => 1,
        }
    }
}

/// What a declared pin is for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Role {
    Vcc(Side),
    Gnd(Side),
    Input(Channel, Side),
    Output(Channel, Side),
    Enable(Side),
    NoConnect,
}

/// One row of a variant's pin table.
#[derive(Debug, Clone, Copy)]
struct PinSpec {
    number: &'static str,
    name: &'static str,
    role: Role,
}

const fn spec(number: &'static str, name: &'static str, role: Role) -> PinSpec {
    PinSpec { number, name, role }
}

// ============================================================
// Variants
// ============================================================

/// A member of the family, identified by its channel count and channel-
/// direction map. The `F` (fail-safe-low) option is orthogonal and lives on
/// [`Config::fail_safe`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Variant {
    /// ISO6720B: dual channel, both forward. D-8 (SLLSFJ0F Figure 5-1).
    Iso6720,
    /// ISO6721B: dual channel, one forward and one reverse. D-8
    /// (SLLSFJ0F Figure 5-2).
    Iso6721,
    /// ISO6721RB: dual channel, the mirrored direction map of
    /// [`Variant::Iso6721`]. D-8 (SLLSFJ0F Figure 5-3).
    Iso6721R,
    /// ISO6731: triple channel, two forward and one reverse. DW-16
    /// (SLASEY9B Figure 5-1).
    Iso6731,
    /// ISO6740: quad channel, all four forward. DW-16
    /// (SLLSFJ6G Figure 6-1).
    Iso6740,
    /// ISO6741: quad channel, three forward and one reverse. DW-16
    /// (SLLSFJ6G Figure 6-2).
    Iso6741,
    /// ISO6742: quad channel, two forward and two reverse. DW-16
    /// (SLLSFJ6G Figure 6-3).
    Iso6742,
}

/// SLLSFJ0F Figure 5-1, Table 5-1 — ISO6720B, D-8.
const ISO6720_PINS: [PinSpec; 8] = [
    spec("1", "VCC1", Role::Vcc(Side::One)),
    spec("2", "INA", Role::Input(Channel::A, Side::One)),
    spec("3", "INB", Role::Input(Channel::B, Side::One)),
    spec("4", "GND1", Role::Gnd(Side::One)),
    spec("5", "GND2", Role::Gnd(Side::Two)),
    spec("6", "OUTB", Role::Output(Channel::B, Side::Two)),
    spec("7", "OUTA", Role::Output(Channel::A, Side::Two)),
    spec("8", "VCC2", Role::Vcc(Side::Two)),
];

/// SLLSFJ0F Figure 5-2, Table 5-1 — ISO6721B, D-8.
const ISO6721_PINS: [PinSpec; 8] = [
    spec("1", "VCC1", Role::Vcc(Side::One)),
    spec("2", "OUTA", Role::Output(Channel::A, Side::One)),
    spec("3", "INB", Role::Input(Channel::B, Side::One)),
    spec("4", "GND1", Role::Gnd(Side::One)),
    spec("5", "GND2", Role::Gnd(Side::Two)),
    spec("6", "OUTB", Role::Output(Channel::B, Side::Two)),
    spec("7", "INA", Role::Input(Channel::A, Side::Two)),
    spec("8", "VCC2", Role::Vcc(Side::Two)),
];

/// SLLSFJ0F Figure 5-3, Table 5-1 — ISO6721RB, D-8.
const ISO6721R_PINS: [PinSpec; 8] = [
    spec("1", "VCC1", Role::Vcc(Side::One)),
    spec("2", "INA", Role::Input(Channel::A, Side::One)),
    spec("3", "OUTB", Role::Output(Channel::B, Side::One)),
    spec("4", "GND1", Role::Gnd(Side::One)),
    spec("5", "GND2", Role::Gnd(Side::Two)),
    spec("6", "INB", Role::Input(Channel::B, Side::Two)),
    spec("7", "OUTA", Role::Output(Channel::A, Side::Two)),
    spec("8", "VCC2", Role::Vcc(Side::Two)),
];

/// SLASEY9B Figure 5-1, Table 5-1 — ISO6731, DW-16.
const ISO6731_PINS: [PinSpec; 16] = [
    spec("1", "VCC1", Role::Vcc(Side::One)),
    spec("2", "GND1_1", Role::Gnd(Side::One)),
    spec("3", "INA", Role::Input(Channel::A, Side::One)),
    spec("4", "INB", Role::Input(Channel::B, Side::One)),
    spec("5", "OUTC", Role::Output(Channel::C, Side::One)),
    spec("6", "NC_1", Role::NoConnect),
    spec("7", "EN1", Role::Enable(Side::One)),
    spec("8", "GND1_2", Role::Gnd(Side::One)),
    spec("9", "GND2_1", Role::Gnd(Side::Two)),
    spec("10", "EN2", Role::Enable(Side::Two)),
    spec("11", "NC_2", Role::NoConnect),
    spec("12", "INC", Role::Input(Channel::C, Side::Two)),
    spec("13", "OUTB", Role::Output(Channel::B, Side::Two)),
    spec("14", "OUTA", Role::Output(Channel::A, Side::Two)),
    spec("15", "GND2_2", Role::Gnd(Side::Two)),
    spec("16", "VCC2", Role::Vcc(Side::Two)),
];

/// SLLSFJ6G Figure 6-1, Table 6-1 — ISO6740, DW-16. Note there is no `EN1`:
/// with every channel forward, side 1 has no outputs to enable.
const ISO6740_PINS: [PinSpec; 16] = [
    spec("1", "VCC1", Role::Vcc(Side::One)),
    spec("2", "GND1_1", Role::Gnd(Side::One)),
    spec("3", "INA", Role::Input(Channel::A, Side::One)),
    spec("4", "INB", Role::Input(Channel::B, Side::One)),
    spec("5", "INC", Role::Input(Channel::C, Side::One)),
    spec("6", "IND", Role::Input(Channel::D, Side::One)),
    spec("7", "NC", Role::NoConnect),
    spec("8", "GND1_2", Role::Gnd(Side::One)),
    spec("9", "GND2_1", Role::Gnd(Side::Two)),
    spec("10", "EN2", Role::Enable(Side::Two)),
    spec("11", "OUTD", Role::Output(Channel::D, Side::Two)),
    spec("12", "OUTC", Role::Output(Channel::C, Side::Two)),
    spec("13", "OUTB", Role::Output(Channel::B, Side::Two)),
    spec("14", "OUTA", Role::Output(Channel::A, Side::Two)),
    spec("15", "GND2_2", Role::Gnd(Side::Two)),
    spec("16", "VCC2", Role::Vcc(Side::Two)),
];

/// SLLSFJ6G Figure 6-2, Table 6-1 — ISO6741, DW-16.
const ISO6741_PINS: [PinSpec; 16] = [
    spec("1", "VCC1", Role::Vcc(Side::One)),
    spec("2", "GND1_1", Role::Gnd(Side::One)),
    spec("3", "INA", Role::Input(Channel::A, Side::One)),
    spec("4", "INB", Role::Input(Channel::B, Side::One)),
    spec("5", "INC", Role::Input(Channel::C, Side::One)),
    spec("6", "OUTD", Role::Output(Channel::D, Side::One)),
    spec("7", "EN1", Role::Enable(Side::One)),
    spec("8", "GND1_2", Role::Gnd(Side::One)),
    spec("9", "GND2_1", Role::Gnd(Side::Two)),
    spec("10", "EN2", Role::Enable(Side::Two)),
    spec("11", "IND", Role::Input(Channel::D, Side::Two)),
    spec("12", "OUTC", Role::Output(Channel::C, Side::Two)),
    spec("13", "OUTB", Role::Output(Channel::B, Side::Two)),
    spec("14", "OUTA", Role::Output(Channel::A, Side::Two)),
    spec("15", "GND2_2", Role::Gnd(Side::Two)),
    spec("16", "VCC2", Role::Vcc(Side::Two)),
];

/// SLLSFJ6G Figure 6-3, Table 6-1 — ISO6742, DW-16.
const ISO6742_PINS: [PinSpec; 16] = [
    spec("1", "VCC1", Role::Vcc(Side::One)),
    spec("2", "GND1_1", Role::Gnd(Side::One)),
    spec("3", "INA", Role::Input(Channel::A, Side::One)),
    spec("4", "INB", Role::Input(Channel::B, Side::One)),
    spec("5", "OUTC", Role::Output(Channel::C, Side::One)),
    spec("6", "OUTD", Role::Output(Channel::D, Side::One)),
    spec("7", "EN1", Role::Enable(Side::One)),
    spec("8", "GND1_2", Role::Gnd(Side::One)),
    spec("9", "GND2_1", Role::Gnd(Side::Two)),
    spec("10", "EN2", Role::Enable(Side::Two)),
    spec("11", "IND", Role::Input(Channel::D, Side::Two)),
    spec("12", "INC", Role::Input(Channel::C, Side::Two)),
    spec("13", "OUTB", Role::Output(Channel::B, Side::Two)),
    spec("14", "OUTA", Role::Output(Channel::A, Side::Two)),
    spec("15", "GND2_2", Role::Gnd(Side::Two)),
    spec("16", "VCC2", Role::Vcc(Side::Two)),
];

impl Variant {
    /// The variant's pin table.
    fn pin_specs(self) -> &'static [PinSpec] {
        match self {
            Variant::Iso6720 => &ISO6720_PINS,
            Variant::Iso6721 => &ISO6721_PINS,
            Variant::Iso6721R => &ISO6721R_PINS,
            Variant::Iso6731 => &ISO6731_PINS,
            Variant::Iso6740 => &ISO6740_PINS,
            Variant::Iso6741 => &ISO6741_PINS,
            Variant::Iso6742 => &ISO6742_PINS,
        }
    }

    /// Variant name, as spelled in the enum.
    pub const fn label(self) -> &'static str {
        match self {
            Variant::Iso6720 => "Iso6720",
            Variant::Iso6721 => "Iso6721",
            Variant::Iso6721R => "Iso6721R",
            Variant::Iso6731 => "Iso6731",
            Variant::Iso6740 => "Iso6740",
            Variant::Iso6741 => "Iso6741",
            Variant::Iso6742 => "Iso6742",
        }
    }

    /// The channels this variant carries, in `A`..`D` order, with the side
    /// each one's input and output sit on.
    pub fn channels(self) -> Vec<(Channel, Side, Side)> {
        let specs = self.pin_specs();
        Channel::ALL
            .into_iter()
            .filter_map(|channel| {
                let mut input = None;
                let mut output = None;
                for spec in specs {
                    match spec.role {
                        Role::Input(c, side) if c == channel => input = Some(side),
                        Role::Output(c, side) if c == channel => output = Some(side),
                        _ => {}
                    }
                }
                Some((channel, input?, output?))
            })
            .collect()
    }

    /// True when this variant carries `channel`.
    pub fn has_channel(self, channel: Channel) -> bool {
        self.pin_specs()
            .iter()
            .any(|spec| matches!(spec.role, Role::Input(c, _) if c == channel))
    }

    /// Parse a variant out of an orderable part number, ignoring the package
    /// and reel suffix: `"ISO6741DWR"`, `"ISO6740FDWR"`, `"ISO6721BDR"`.
    ///
    /// Returns the variant and whether the `F` (fail-safe-low) option is
    /// present. `None` for anything that is not a recognized family member,
    /// so a consumer registry can fall through to its own handling.
    pub fn from_part_name(part: &str) -> Option<(Variant, bool)> {
        let upper = part.trim().to_ascii_uppercase();
        let rest = upper.strip_prefix("ISO")?;
        let (digits, rest) = rest.split_at_checked(4)?;
        let (variant, rest) = match digits {
            "6720" => (Variant::Iso6720, rest),
            "6721" => match rest.strip_prefix('R') {
                Some(rest) => (Variant::Iso6721R, rest),
                None => (Variant::Iso6721, rest),
            },
            "6731" => (Variant::Iso6731, rest),
            "6740" => (Variant::Iso6740, rest),
            "6741" => (Variant::Iso6741, rest),
            "6742" => (Variant::Iso6742, rest),
            _ => return None,
        };
        // The dual-channel parts carry a family letter `B` that may sit on
        // either side of the fail-safe `F` (`ISO6721FBD`, `ISO6721BD`); no
        // package code in the family starts with `F`.
        let rest = rest.strip_prefix('B').unwrap_or(rest);
        Some((variant, rest.starts_with('F')))
    }
}

// ============================================================
// Configuration
// ============================================================

/// What one channel carries across the barrier.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChannelRole {
    /// A logic level: sense the input net, drive the output net. The default.
    Level,
    /// A rate-carried pulse train (a step clock). See the module docs.
    Pulse,
    /// A UART byte stream at `baud_hz`.
    Serial {
        /// Byte pacing rate declared on both stream pins.
        baud_hz: u32,
    },
}

/// Isolator configuration. Build with [`Config::new`] and relax the fields a
/// particular part needs.
#[derive(Debug, Clone)]
pub struct Config {
    /// Family member (channel count and direction map).
    pub variant: Variant,
    /// The `F` option: the default output state is **low** instead of high
    /// (SLLSFJ6G §3).
    pub fail_safe: bool,
    /// Supply at or above which a side is powered up
    /// ([`DEFAULT_SUPPLY_MIN_VOLTS`]).
    pub supply_min_volts: Volts,
    /// Supply voltage assumed while a VCC net has no numeric solve
    /// ([`DEFAULT_NOMINAL_SUPPLY_VOLTS`]).
    pub nominal_supply_volts: Volts,
    /// `V_IH` as a fraction of the input supply ([`DEFAULT_VIH_RATIO`]).
    pub vih_ratio: f64,
    /// `V_IL` as a fraction of the input supply ([`DEFAULT_VIL_RATIO`]).
    pub vil_ratio: f64,
    /// Output Thevenin source impedance
    /// ([`DEFAULT_OUTPUT_IMPEDANCE_OHMS`]).
    pub output_impedance_ohms: Ohms,
    /// Per-channel role, indexed `A`..`D`.
    roles: [ChannelRole; 4],
}

impl Config {
    /// A variant with every channel carrying a level and every parameter at
    /// its datasheet default.
    pub fn new(variant: Variant) -> Self {
        Self {
            variant,
            fail_safe: false,
            supply_min_volts: DEFAULT_SUPPLY_MIN_VOLTS,
            nominal_supply_volts: DEFAULT_NOMINAL_SUPPLY_VOLTS,
            vih_ratio: DEFAULT_VIH_RATIO,
            vil_ratio: DEFAULT_VIL_RATIO,
            output_impedance_ohms: DEFAULT_OUTPUT_IMPEDANCE_OHMS,
            roles: [ChannelRole::Level; 4],
        }
    }

    /// A configuration for an orderable part number
    /// ([`Variant::from_part_name`]), with the `F` option applied.
    ///
    /// This is the hook a consumer's [`embsim_board::PartRegistry`] wants: a
    /// netlist's `libsource` part name goes in, a configured isolator comes
    /// out, and `ISO6740FDWR` gets its fail-safe-low default without anyone
    /// re-deriving it from the suffix.
    pub fn from_part_name(part: &str) -> Option<Self> {
        let (variant, fail_safe) = Variant::from_part_name(part)?;
        Some(Self {
            fail_safe,
            ..Self::new(variant)
        })
    }

    /// Set the `F` (fail-safe-low default output) option.
    pub fn fail_safe(mut self, fail_safe: bool) -> Self {
        self.fail_safe = fail_safe;
        self
    }

    /// Carry `channel` as a rate-carried pulse train instead of a level.
    pub fn with_pulse_channel(mut self, channel: Channel) -> Self {
        self.roles[channel.index()] = ChannelRole::Pulse;
        self
    }

    /// Carry `channel` as a UART byte stream instead of a level.
    pub fn with_serial_channel(mut self, channel: Channel, baud_hz: u32) -> Self {
        self.roles[channel.index()] = ChannelRole::Serial { baud_hz };
        self
    }

    /// The role configured for a channel.
    pub fn role(&self, channel: Channel) -> ChannelRole {
        self.roles[channel.index()]
    }

    /// The default output state: low for an `F` part, high otherwise
    /// (SLLSFJ6G §3, §9.4 Table 9-2).
    pub fn default_level(&self) -> Level {
        if self.fail_safe {
            Level::Low
        } else {
            Level::High
        }
    }

    fn validate(&self) -> Result<(), PartConfigError> {
        require_positive("supply_min_volts", self.supply_min_volts)?;
        require_positive("nominal_supply_volts", self.nominal_supply_volts)?;
        require_positive("output_impedance_ohms", self.output_impedance_ohms)?;
        require_positive("vih_ratio", self.vih_ratio)?;
        require_positive("vil_ratio", self.vil_ratio)?;
        if self.vil_ratio >= self.vih_ratio {
            return Err(PartConfigError::InvertedThresholds {
                vil_ratio: self.vil_ratio,
                vih_ratio: self.vih_ratio,
            });
        }
        // A role assigned to a channel the variant does not have is a system
        // description bug, not something to drop on the floor.
        for channel in Channel::ALL {
            if self.roles[channel.index()] != ChannelRole::Level
                && !self.variant.has_channel(channel)
            {
                return Err(PartConfigError::NoSuchChannel {
                    variant: self.variant.label(),
                    channel: channel.label(),
                });
            }
        }
        Ok(())
    }
}

// ============================================================
// Wiring
// ============================================================

/// One channel resolved against the variant's pin table.
#[derive(Debug, Clone, Copy)]
struct Wiring {
    channel: Channel,
    input_pin: &'static str,
    output_pin: &'static str,
    input_side: Side,
    output_side: Side,
    role: ChannelRole,
}

// ============================================================
// Core state
// ============================================================

/// Everything the channels read, written by sense callbacks on the engine
/// thread.
#[derive(Debug)]
struct CoreState {
    /// Last published state of each side's `VCC` net.
    vcc: [NetState; 2],
    /// Last published state of each side's enable net. A side with no enable
    /// pin stays [`NetState::Floating`], which the datasheet reads as "open"
    /// and therefore enabled.
    enable: [NetState; 2],
    /// Last published state of each channel's input net.
    input: [NetState; 4],
    /// Output pin handles, `None` until attach (and in unit tests, where the
    /// drive decision is bookkeeping only).
    output: [Option<PinHandle>; 4],
    /// Last drive applied per channel: `None` = never applied,
    /// `Some(None)` = released, `Some(Some(d))` = driving `d`.
    applied: [Option<Option<TheveninDrive>>; 4],
    /// Pulse publishers, per channel.
    pulse_tx: [Option<PulseTx>; 4],
    /// Last train received on a pulse channel's input.
    train_in: [Option<PulseTrain>; 4],
    /// Last train published on a pulse channel's output.
    train_out: [Option<PulseTrain>; 4],
    /// Count of `set_drive` calls actually issued — the event-cost meter.
    drives: u64,
    /// Count of `set_train` calls actually issued.
    trains: u64,
}

impl CoreState {
    fn new() -> Self {
        Self {
            vcc: [NetState::Floating; 2],
            enable: [NetState::Floating; 2],
            input: [NetState::Floating; 4],
            output: Default::default(),
            applied: Default::default(),
            pulse_tx: Default::default(),
            train_in: Default::default(),
            train_out: Default::default(),
            drives: 0,
            trains: 0,
        }
    }
}

/// Shared state behind the component and every [`Iso67xxMonitor`].
#[derive(Debug)]
struct Core {
    config: Config,
    wiring: Vec<Wiring>,
    state: Mutex<CoreState>,
}

impl Core {
    /// Whether a side's supply is up.
    fn side_up(&self, state: &CoreState, side: Side) -> bool {
        supply_up(state.vcc[side.index()], self.config.supply_min_volts)
    }

    /// Whether a side's outputs are enabled. `ENx` high **or open** enables
    /// (SLLSFJ6G Table 6-1); low disables. A fought-over enable is treated as
    /// disabled — the conservative reading, and the one that does not invent
    /// a winner.
    fn side_enabled(&self, state: &CoreState, side: Side) -> bool {
        let enable = state.enable[side.index()];
        if matches!(enable, NetState::Floating) {
            return true;
        }
        let rail = rail_volts(state.vcc[side.index()], self.config.nominal_supply_volts);
        threshold_level(
            enable,
            self.config.vil_ratio * rail,
            self.config.vih_ratio * rail,
        ) == Some(Level::High)
    }

    /// Whether the output buffer can drive at all: its own supply up and its
    /// side enabled (Table 9-2 rows 3 and 5).
    fn output_live(&self, state: &CoreState, wiring: &Wiring) -> bool {
        self.side_up(state, wiring.output_side) && self.side_enabled(state, wiring.output_side)
    }

    /// Whether the channel is actually relaying: the output stage live *and*
    /// the input side powered.
    fn passing(&self, state: &CoreState, wiring: &Wiring) -> bool {
        self.output_live(state, wiring) && self.side_up(state, wiring.input_side)
    }

    /// The level the output presents, given a live output stage: the input's
    /// level when there is one, the default state otherwise (Table 9-2 rows
    /// 1, 2 and 4).
    fn output_level(&self, state: &CoreState, wiring: &Wiring) -> Level {
        if !self.side_up(state, wiring.input_side) {
            return self.config.default_level();
        }
        let rail = rail_volts(
            state.vcc[wiring.input_side.index()],
            self.config.nominal_supply_volts,
        );
        threshold_level(
            state.input[wiring.channel.index()],
            self.config.vil_ratio * rail,
            self.config.vih_ratio * rail,
        )
        .unwrap_or_else(|| self.config.default_level())
    }

    /// The drive a level or pulse channel's output pin should present.
    fn desired_drive(&self, state: &CoreState, wiring: &Wiring) -> Option<TheveninDrive> {
        if !self.output_live(state, wiring) {
            return None;
        }
        let rail = rail_volts(
            state.vcc[wiring.output_side.index()],
            self.config.nominal_supply_volts,
        );
        Some(level_drive(
            self.output_level(state, wiring),
            rail,
            self.config.output_impedance_ohms,
        ))
    }

    /// Apply a channel's output — **only when it changed**.
    ///
    /// This is the whole event-cost discipline: a repeater that re-drove on
    /// every delivery would multiply engine resolutions by its channel count
    /// and by every unrelated supply wobble.
    fn apply_level(&self, state: &mut CoreState, wiring: &Wiring) {
        // A serial channel's output pin belongs to the byte pacer; the
        // producer's idle drive is the engine's and this model does not fight
        // it (see the module docs).
        if matches!(wiring.role, ChannelRole::Serial { .. }) {
            return;
        }
        let index = wiring.channel.index();
        let desired = self.desired_drive(state, wiring);
        if state.applied[index] == Some(desired) {
            return;
        }
        state.applied[index] = Some(desired);
        state.drives += 1;
        if let Some(pin) = &state.output[index] {
            pin.set_drive(desired);
        }
    }

    /// Republish a pulse channel's train — again, only when it changed.
    ///
    /// A relayed segment is passed through **verbatim**: it carries its own
    /// anchor (`since_us`) and accumulated count, so forwarding it neither
    /// re-bases nor double-counts, and the downstream plant folds exactly what
    /// the source published. A channel that is not passing publishes a held
    /// train once, rather than leaving the last rate running forever.
    fn apply_train(&self, state: &mut CoreState, wiring: &Wiring) {
        if wiring.role != ChannelRole::Pulse {
            return;
        }
        let index = wiring.channel.index();
        let desired = if self.passing(state, wiring) {
            state.train_in[index]
        } else {
            state.train_in[index].map(|_| PulseTrain::IDLE)
        };
        let Some(train) = desired else {
            return;
        };
        if state.train_out[index] == Some(train) {
            return;
        }
        state.train_out[index] = Some(train);
        state.trains += 1;
        if let Some(tx) = &state.pulse_tx[index] {
            tx.set_train(train);
        }
    }

    /// Re-evaluate one channel's outputs.
    fn refresh(&self, state: &mut CoreState, wiring: &Wiring) {
        self.apply_level(state, wiring);
        self.apply_train(state, wiring);
    }

    /// Re-evaluate every channel — for a supply or enable change, which is
    /// the only input that is not per channel.
    fn refresh_all(&self, state: &mut CoreState) {
        for wiring in &self.wiring {
            self.refresh(state, wiring);
        }
    }
}

// ============================================================
// Monitor handle
// ============================================================

/// Cheap cloneable read handle onto a live [`Iso67xx`].
///
/// Cloned out of the component *before* it is handed to
/// [`embsim_board::System`], exactly like
/// [`crate::machine::EndSwitchActuator`]. An isolator has nothing to actuate,
/// so this handle only reads: what each channel is presenting, whether it is
/// passing, and how much engine traffic it has cost.
#[derive(Clone, Debug)]
pub struct Iso67xxMonitor {
    core: Arc<Core>,
}

impl Iso67xxMonitor {
    /// The drive the channel's output pin is presenting, or `None` when the
    /// pin is released (unpowered output side, or disabled by `ENx`).
    pub fn output_drive(&self, channel: Channel) -> Option<TheveninDrive> {
        self.core.state.lock().unwrap().applied[channel.index()].flatten()
    }

    /// The logic level the channel's output is presenting, or `None` when the
    /// pin is released.
    pub fn output_level(&self, channel: Channel) -> Option<Level> {
        let state = self.core.state.lock().unwrap();
        let wiring = self.core.wiring.iter().find(|w| w.channel == channel)?;
        // A released pin has no level; only a driving one does.
        state.applied[channel.index()].flatten()?;
        Some(self.core.output_level(&state, wiring))
    }

    /// Whether the channel is relaying its input (both sides powered, output
    /// side enabled). A channel that is *not* passing may still be driving —
    /// its default output state.
    pub fn is_passing(&self, channel: Channel) -> bool {
        let state = self.core.state.lock().unwrap();
        self.core
            .wiring
            .iter()
            .find(|w| w.channel == channel)
            .is_some_and(|wiring| self.core.passing(&state, wiring))
    }

    /// The train last published on a pulse channel's output.
    pub fn relayed_train(&self, channel: Channel) -> Option<PulseTrain> {
        self.core.state.lock().unwrap().train_out[channel.index()]
    }

    /// Total `set_drive` calls this part has issued since construction.
    ///
    /// The event-cost meter: a level change on one channel costs exactly one,
    /// and an unchanged re-evaluation costs zero.
    pub fn drive_count(&self) -> u64 {
        self.core.state.lock().unwrap().drives
    }

    /// Total `set_train` calls this part has issued since construction.
    pub fn train_count(&self) -> u64 {
        self.core.state.lock().unwrap().trains
    }

    /// The configuration in force.
    pub fn config(&self) -> &Config {
        &self.core.config
    }
}

// ============================================================
// Component
// ============================================================

/// A TI ISO674x / ISO673x / ISO672x digital isolator as a live board-engine
/// component.
///
/// ```rust
/// use embsim_board::Component;
/// use embsim_models::isolation::iso67xx::{Config, Iso67xx};
/// use embsim_models::isolation::{Channel, Variant};
///
/// // The MaD EdgeBoard's IC14, straight off its netlist part name.
/// let config = Config::from_part_name("ISO6741DWR").expect("a family member");
/// assert_eq!(config.variant, Variant::Iso6741);
/// assert!(!config.fail_safe);
///
/// // STEP crosses as a rate-carried train, not as edges.
/// let isolator = Iso67xx::new(config.with_pulse_channel(Channel::A)).expect("valid");
/// assert_eq!(isolator.pins().len(), 16);
/// ```
#[derive(Debug)]
pub struct Iso67xx {
    pins: Vec<PinDecl>,
    core: Arc<Core>,
}

impl Iso67xx {
    /// Create an isolator from a validated configuration.
    pub fn new(config: Config) -> Result<Self, PartConfigError> {
        config.validate()?;
        let specs = config.variant.pin_specs();
        let wiring = wiring_for(&config);
        let pins = specs.iter().map(|spec| declare(spec, &config)).collect();
        tracing::info!(
            variant = config.variant.label(),
            fail_safe = config.fail_safe,
            channels = wiring.len(),
            "iso67xx: init"
        );
        Ok(Self {
            pins,
            core: Arc::new(Core {
                config,
                wiring,
                state: Mutex::new(CoreState::new()),
            }),
        })
    }

    /// A read handle onto this isolator.
    pub fn monitor(&self) -> Iso67xxMonitor {
        Iso67xxMonitor {
            core: Arc::clone(&self.core),
        }
    }

    /// The validated configuration in force.
    pub fn config(&self) -> &Config {
        &self.core.config
    }
}

/// Resolve every channel the variant carries against its pin table.
fn wiring_for(config: &Config) -> Vec<Wiring> {
    let specs = config.variant.pin_specs();
    config
        .variant
        .channels()
        .into_iter()
        .map(|(channel, input_side, output_side)| {
            let find = |want_input: bool| {
                specs
                    .iter()
                    .find(|spec| match spec.role {
                        Role::Input(c, _) => want_input && c == channel,
                        Role::Output(c, _) => !want_input && c == channel,
                        _ => false,
                    })
                    .map(|spec| spec.number)
                    .expect("channels() only reports channels with both pins")
            };
            Wiring {
                channel,
                input_pin: find(true),
                output_pin: find(false),
                input_side,
                output_side,
                role: config.role(channel),
            }
        })
        .collect()
}

/// Turn one pin-table row into a [`PinDecl`], applying the channel's role.
fn declare(spec: &PinSpec, config: &Config) -> PinDecl {
    let (kind, stream) = match spec.role {
        Role::Vcc(_) | Role::Gnd(_) => (PinKind::PowerIn, None),
        // A no-connect pad is declared (the netlist has a node for it, on an
        // `unconnected-(...)` net) but contributes and senses nothing, so a
        // deliberately dangling pin raises no finding.
        Role::NoConnect => (PinKind::Passive, None),
        Role::Enable(_) => (PinKind::DigitalIn, None),
        Role::Input(channel, _) => (
            PinKind::DigitalIn,
            match config.role(channel) {
                ChannelRole::Level => None,
                ChannelRole::Pulse => Some(StreamRole::PulseSink),
                ChannelRole::Serial { baud_hz } => Some(StreamRole::Consumer { baud_hz }),
            },
        ),
        Role::Output(channel, _) => (
            PinKind::DigitalOut,
            match config.role(channel) {
                ChannelRole::Level => None,
                ChannelRole::Pulse => Some(StreamRole::PulseSource),
                ChannelRole::Serial { baud_hz } => Some(StreamRole::Producer { baud_hz }),
            },
        ),
    };
    PinDecl {
        number: spec.number,
        name: Some(spec.name),
        kind,
        stream,
        // Applied per drive: the impedance is configuration, not a
        // `&'static` constant.
        drive_impedance: None,
    }
}

impl Component for Iso67xx {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        // 1. Claim the output pins and settle every channel to its
        //    unpowered state. The engine gave each `DigitalOut` an idle-high
        //    drive at assembly, so releasing them is the first thing that must
        //    happen — an isolator with no rails yet drives nothing.
        {
            let mut state = self.core.state.lock().unwrap();
            for wiring in &self.core.wiring {
                let index = wiring.channel.index();
                state.output[index] = Some(io.pin(wiring.output_pin)?);
                if wiring.role == ChannelRole::Pulse {
                    state.pulse_tx[index] = Some(io.pulse_tx(wiring.output_pin)?);
                }
            }
            self.core.refresh_all(&mut state);
        }

        // 2. Supplies and enables first, so a level or byte delivered before
        //    the rails are known cannot slip through the gate.
        for spec in self.core.config.variant.pin_specs() {
            match spec.role {
                Role::Vcc(side) => {
                    let core = Arc::clone(&self.core);
                    io.on_sense(spec.number, move |sensed| {
                        let mut state = core.state.lock().unwrap();
                        state.vcc[side.index()] = sensed;
                        core.refresh_all(&mut state);
                    })?;
                }
                Role::Enable(side) => {
                    let core = Arc::clone(&self.core);
                    io.on_sense(spec.number, move |sensed| {
                        let mut state = core.state.lock().unwrap();
                        state.enable[side.index()] = sensed;
                        core.refresh_all(&mut state);
                    })?;
                }
                _ => {}
            }
        }

        // 3. Per-channel inputs. Each channel subscribes only to its own
        //    input, so one transition costs one drive rather than one per
        //    channel.
        for wiring in &self.core.wiring {
            let wiring = *wiring;
            let index = wiring.channel.index();

            {
                let core = Arc::clone(&self.core);
                io.on_sense(wiring.input_pin, move |sensed| {
                    let mut state = core.state.lock().unwrap();
                    state.input[index] = sensed;
                    core.refresh(&mut state, &wiring);
                })?;
            }

            match wiring.role {
                ChannelRole::Level => {}
                ChannelRole::Pulse => {
                    let core = Arc::clone(&self.core);
                    io.on_pulse(wiring.input_pin, move |train| {
                        let mut state = core.state.lock().unwrap();
                        state.train_in[index] = Some(train);
                        core.apply_train(&mut state, &wiring);
                    })?;
                }
                ChannelRole::Serial { .. } => {
                    let tx = io.stream_tx(wiring.output_pin)?;
                    let core = Arc::clone(&self.core);
                    io.on_byte(wiring.input_pin, move |byte| {
                        if core.passing(&core.state.lock().unwrap(), &wiring) {
                            tx.write(&[byte]);
                        } else {
                            tracing::trace!(
                                byte,
                                channel = wiring.channel.label(),
                                "iso67xx: byte dropped (a side is unpowered or disabled)"
                            );
                        }
                    })?;
                }
            }
        }
        Ok(())
    }
}

// ============================================================
// Tests
// ============================================================
//
// The function table, the pin tables and the drive-on-change discipline are
// exercised here without an engine (the output handles are absent, so drives
// are bookkeeping only). What the *nets* do — a level crossing the barrier on
// the real EdgeBoard netlist, and the engine-event budget of a step train
// crossing it — lives in `board/tests/isolation_bridge.rs`.

#[cfg(test)]
mod tests {
    use rstest::rstest;

    use super::*;

    const V3V3: NetState = NetState::Analog(3.3);
    const V5V: NetState = NetState::Analog(5.0);
    const DOWN: NetState = NetState::Analog(0.0);

    fn isolator(config: Config) -> (Iso67xx, Iso67xxMonitor) {
        let isolator = Iso67xx::new(config).expect("valid config");
        let monitor = isolator.monitor();
        (isolator, monitor)
    }

    /// Drive the shared state directly, as the sense callbacks would.
    fn set_vcc(iso: &Iso67xx, side: Side, state: NetState) {
        let mut guard = iso.core.state.lock().unwrap();
        guard.vcc[side.index()] = state;
        iso.core.refresh_all(&mut guard);
    }

    fn set_enable(iso: &Iso67xx, side: Side, state: NetState) {
        let mut guard = iso.core.state.lock().unwrap();
        guard.enable[side.index()] = state;
        iso.core.refresh_all(&mut guard);
    }

    fn set_input(iso: &Iso67xx, channel: Channel, state: NetState) {
        let mut guard = iso.core.state.lock().unwrap();
        guard.input[channel.index()] = state;
        let wiring = *iso
            .core
            .wiring
            .iter()
            .find(|w| w.channel == channel)
            .expect("channel exists");
        iso.core.refresh(&mut guard, &wiring);
    }

    /// Both rails up, so the part is in its normal-operation row.
    fn power_up(iso: &Iso67xx) {
        set_vcc(iso, Side::One, V3V3);
        set_vcc(iso, Side::Two, V3V3);
    }

    // -- pin tables -------------------------------------------------

    /// Every variant's facade covers its package exactly once, names no pin
    /// twice, and pairs every channel input with an output on the other side.
    #[rstest]
    #[case::iso6720(Variant::Iso6720, 8, 2)]
    #[case::iso6721(Variant::Iso6721, 8, 2)]
    #[case::iso6721r(Variant::Iso6721R, 8, 2)]
    #[case::iso6731(Variant::Iso6731, 16, 3)]
    #[case::iso6740(Variant::Iso6740, 16, 4)]
    #[case::iso6741(Variant::Iso6741, 16, 4)]
    #[case::iso6742(Variant::Iso6742, 16, 4)]
    fn pin_tables_are_complete_and_consistent(
        #[case] variant: Variant,
        #[case] pins: usize,
        #[case] channels: usize,
    ) {
        let (iso, _) = isolator(Config::new(variant));
        assert_eq!(iso.pins().len(), pins);

        let mut numbers: Vec<&str> = iso.pins().iter().map(|p| p.number).collect();
        numbers.sort_unstable_by_key(|n| n.parse::<u32>().expect("numeric pin"));
        let expected: Vec<String> = (1..=pins).map(|n| n.to_string()).collect();
        assert_eq!(numbers, expected, "{variant:?} must cover its package");

        let mut names: Vec<&str> = iso
            .pins()
            .iter()
            .map(|p| p.name.expect("every pin is named"))
            .collect();
        names.sort_unstable();
        let unique = names.len();
        names.dedup();
        assert_eq!(names.len(), unique, "{variant:?} must not name a pin twice");

        let map = variant.channels();
        assert_eq!(map.len(), channels);
        for (channel, input_side, output_side) in map {
            assert_ne!(
                input_side,
                output_side,
                "{variant:?} channel {} must cross the barrier",
                channel.label()
            );
        }
    }

    /// The direction maps the datasheets state: how many channels run side 1
    /// to side 2, and how many run back.
    #[rstest]
    #[case::iso6720(Variant::Iso6720, 2, 0)]
    #[case::iso6721(Variant::Iso6721, 1, 1)]
    #[case::iso6721r(Variant::Iso6721R, 1, 1)]
    #[case::iso6731(Variant::Iso6731, 2, 1)]
    #[case::iso6740(Variant::Iso6740, 4, 0)]
    #[case::iso6741(Variant::Iso6741, 3, 1)]
    #[case::iso6742(Variant::Iso6742, 2, 2)]
    fn channel_direction_maps_match_the_datasheets(
        #[case] variant: Variant,
        #[case] forward: usize,
        #[case] reverse: usize,
    ) {
        let map = variant.channels();
        assert_eq!(
            map.iter().filter(|(_, i, _)| *i == Side::One).count(),
            forward
        );
        assert_eq!(
            map.iter().filter(|(_, i, _)| *i == Side::Two).count(),
            reverse
        );
    }

    /// The EdgeBoard's own two isolators, pin for pin against its netlist.
    #[rstest]
    #[case::ic14_iso6741(Variant::Iso6741, &[("3", "INA"), ("4", "INB"), ("5", "INC"),
        ("6", "OUTD"), ("7", "EN1"), ("10", "EN2"), ("11", "IND"), ("12", "OUTC"),
        ("13", "OUTB"), ("14", "OUTA"), ("1", "VCC1"), ("16", "VCC2")])]
    #[case::ic16_iso6740(Variant::Iso6740, &[("3", "INA"), ("4", "INB"), ("5", "INC"),
        ("6", "IND"), ("7", "NC"), ("10", "EN2"), ("11", "OUTD"), ("12", "OUTC"),
        ("13", "OUTB"), ("14", "OUTA"), ("1", "VCC1"), ("16", "VCC2")])]
    fn pin_names_match_the_edgeboard_netlist(
        #[case] variant: Variant,
        #[case] expect: &[(&str, &str)],
    ) {
        let (iso, _) = isolator(Config::new(variant));
        for (number, name) in expect {
            let decl = iso
                .pins()
                .iter()
                .find(|p| p.number == *number)
                .unwrap_or_else(|| panic!("pin {number}"));
            assert_eq!(decl.name, Some(*name), "pin {number}");
        }
    }

    // -- part-name parsing ------------------------------------------

    #[rstest]
    #[case::ic14("ISO6741DWR", Some((Variant::Iso6741, false)))]
    #[case::ic16("ISO6740FDWR", Some((Variant::Iso6740, true)))]
    #[case::ic1("ISO6742DWR", Some((Variant::Iso6742, false)))]
    #[case::ic15("ISO6721BDR", Some((Variant::Iso6721, false)))]
    #[case::ic5("ISO6731DWR", Some((Variant::Iso6731, false)))]
    #[case::fail_safe_dual("ISO6721FBD", Some((Variant::Iso6721, true)))]
    #[case::fail_safe_dual_other_order("ISO6721BFD", Some((Variant::Iso6721, true)))]
    #[case::reverse_dual("ISO6721RBD", Some((Variant::Iso6721R, false)))]
    #[case::bare("ISO6740", Some((Variant::Iso6740, false)))]
    #[case::lowercase("iso6740fdwr", Some((Variant::Iso6740, true)))]
    #[case::other_family("ISO7741DWR", None)]
    #[case::not_an_isolator("AM26LS31CD", None)]
    #[case::truncated("ISO67", None)]
    fn part_names_parse_to_variant_and_fail_safe(
        #[case] part: &str,
        #[case] expect: Option<(Variant, bool)>,
    ) {
        assert_eq!(Variant::from_part_name(part), expect);
    }

    #[rstest]
    fn config_from_part_name_applies_the_fail_safe_option() {
        let config = Config::from_part_name("ISO6740FDWR").expect("a family member");
        assert_eq!(config.variant, Variant::Iso6740);
        assert_eq!(config.default_level(), Level::Low);
        let config = Config::from_part_name("ISO6741DWR").expect("a family member");
        assert_eq!(config.default_level(), Level::High);
        assert!(Config::from_part_name("VO2631").is_none());
    }

    // -- configuration validation -----------------------------------

    #[rstest]
    fn a_role_on_a_channel_the_variant_lacks_is_rejected() {
        let error = Iso67xx::new(Config::new(Variant::Iso6721).with_pulse_channel(Channel::D))
            .expect_err("ISO6721 has no channel D");
        assert_eq!(
            error,
            PartConfigError::NoSuchChannel {
                variant: "Iso6721",
                channel: "D"
            }
        );
        // A *level* role on an absent channel is the default for every
        // channel, so it must not be an error.
        assert!(Iso67xx::new(Config::new(Variant::Iso6721)).is_ok());
    }

    #[rstest]
    #[case::zero_impedance(Config { output_impedance_ohms: 0.0, ..Config::new(Variant::Iso6741) })]
    #[case::negative_supply(Config { supply_min_volts: -1.0, ..Config::new(Variant::Iso6741) })]
    #[case::nan_nominal(Config { nominal_supply_volts: f64::NAN, ..Config::new(Variant::Iso6741) })]
    fn invalid_parameters_are_rejected(#[case] config: Config) {
        assert!(Iso67xx::new(config).is_err());
    }

    #[rstest]
    fn inverted_thresholds_are_rejected() {
        let config = Config {
            vil_ratio: 0.7,
            vih_ratio: 0.3,
            ..Config::new(Variant::Iso6741)
        };
        assert_eq!(
            Iso67xx::new(config).expect_err("inverted"),
            PartConfigError::InvertedThresholds {
                vil_ratio: 0.7,
                vih_ratio: 0.3
            }
        );
    }

    // -- the function table -----------------------------------------

    /// Row 1: both sides up, input at a level — the output follows it.
    #[rstest]
    #[case::high(V3V3, Level::High)]
    #[case::low(DOWN, Level::Low)]
    #[case::driven_high(NetState::Driven(Level::High), Level::High)]
    #[case::pulled_low(NetState::Pulled(Level::Low, 4_700.0), Level::Low)]
    fn a_powered_channel_follows_its_input(#[case] input: NetState, #[case] expect: Level) {
        let (iso, monitor) = isolator(Config::new(Variant::Iso6741));
        power_up(&iso);
        set_input(&iso, Channel::A, input);
        assert_eq!(monitor.output_level(Channel::A), Some(expect));
        assert!(monitor.is_passing(Channel::A));
    }

    /// Row 2: an input the engine will not put a level on is "open", and the
    /// output goes to its default state — high for a plain part, low for an
    /// `F` part.
    #[rstest]
    #[case::floating(NetState::Floating)]
    #[case::contention(NetState::Contention)]
    #[case::dead_band(NetState::Analog(1.65))]
    fn an_undecidable_input_gets_the_default_output(#[case] input: NetState) {
        for (fail_safe, expect) in [(false, Level::High), (true, Level::Low)] {
            let (iso, monitor) = isolator(Config::new(Variant::Iso6740).fail_safe(fail_safe));
            power_up(&iso);
            set_input(&iso, Channel::A, input);
            assert_eq!(
                monitor.output_level(Channel::A),
                Some(expect),
                "fail_safe = {fail_safe}, input = {input:?}"
            );
        }
    }

    /// Row 4: the input side unpowered — the output still drives, at the
    /// default state. This is the whole point of the `F` option: `ISO6740F`
    /// presents a defined LOW when the far side dies.
    #[rstest]
    #[case::plain(false, Level::High)]
    #[case::fail_safe(true, Level::Low)]
    fn an_unpowered_input_side_gets_the_default_output(
        #[case] fail_safe: bool,
        #[case] expect: Level,
    ) {
        let (iso, monitor) = isolator(Config::new(Variant::Iso6740).fail_safe(fail_safe));
        power_up(&iso);
        set_input(&iso, Channel::A, V3V3);
        assert_eq!(monitor.output_level(Channel::A), Some(Level::High));

        set_vcc(&iso, Side::One, DOWN);
        assert_eq!(monitor.output_level(Channel::A), Some(expect));
        assert!(!monitor.is_passing(Channel::A));
    }

    /// Row 5: the *output* side unpowered — the pin is released. The bench
    /// behavior worth reproducing: an isolator with only one side powered
    /// passes nothing, whichever side that is.
    #[rstest]
    #[case::input_side_down(Side::One)]
    #[case::output_side_down(Side::Two)]
    fn one_side_powered_passes_nothing(#[case] down: Side) {
        let (iso, monitor) = isolator(Config::new(Variant::Iso6741));
        power_up(&iso);
        set_input(&iso, Channel::A, DOWN);
        assert_eq!(monitor.output_level(Channel::A), Some(Level::Low));

        set_vcc(&iso, down, DOWN);
        assert!(!monitor.is_passing(Channel::A));
        if down == Side::Two {
            assert_eq!(
                monitor.output_drive(Channel::A),
                None,
                "an unpowered output buffer drives nothing"
            );
        }
    }

    /// A floating supply is a down supply: the engine never invents a value
    /// for an unsourced net, so a system description that forgot a rail gets
    /// a dead isolator rather than a working one.
    #[rstest]
    fn a_floating_supply_is_a_down_supply() {
        let (iso, monitor) = isolator(Config::new(Variant::Iso6741));
        set_vcc(&iso, Side::One, V3V3);
        // Side 2 never gets a rail.
        set_input(&iso, Channel::A, V3V3);
        assert_eq!(monitor.output_drive(Channel::A), None);
        assert!(!monitor.is_passing(Channel::A));
    }

    /// Row 3: `ENx` low puts that side's outputs into high impedance;
    /// high **or open** enables them (SLLSFJ6G Table 6-1).
    #[rstest]
    #[case::open(NetState::Floating, true)]
    #[case::high(V3V3, true)]
    #[case::driven_high(NetState::Driven(Level::High), true)]
    #[case::low(DOWN, false)]
    #[case::driven_low(NetState::Driven(Level::Low), false)]
    #[case::contention(NetState::Contention, false)]
    fn the_enable_pin_gates_its_own_sides_outputs(#[case] enable: NetState, #[case] enabled: bool) {
        let (iso, monitor) = isolator(Config::new(Variant::Iso6741));
        power_up(&iso);
        set_input(&iso, Channel::A, V3V3);
        set_enable(&iso, Side::Two, enable);
        assert_eq!(monitor.output_drive(Channel::A).is_some(), enabled);

        // EN2 governs side-2 outputs only: channel D's output is on side 1.
        assert!(
            monitor.output_drive(Channel::D).is_some(),
            "EN2 must not gate a side-1 output"
        );
    }

    /// The drive tracks the *output* side's rail, which is what makes these
    /// parts level translators: a 3.3 V input presents 5 V out.
    #[rstest]
    fn the_output_drives_its_own_sides_rail() {
        let (iso, monitor) = isolator(Config::new(Variant::Iso6741));
        set_vcc(&iso, Side::One, V3V3);
        set_vcc(&iso, Side::Two, V5V);
        set_input(&iso, Channel::A, V3V3);
        assert_eq!(
            monitor.output_drive(Channel::A),
            Some(TheveninDrive {
                volts: 5.0,
                impedance: DEFAULT_OUTPUT_IMPEDANCE_OHMS
            })
        );
    }

    /// `V_IH = 0.7 x VCCI` scales with the *input* side's rail: 2.4 V is a
    /// high against 3.3 V (2.31 V) and a dead-band nothing against 5 V
    /// (3.5 V), so at 5 V the output falls to its default.
    #[rstest]
    fn input_thresholds_track_the_input_side_rail() {
        let (iso, monitor) = isolator(Config::new(Variant::Iso6740).fail_safe(true));
        set_vcc(&iso, Side::Two, V3V3);
        set_vcc(&iso, Side::One, V3V3);
        set_input(&iso, Channel::A, NetState::Analog(2.4));
        assert_eq!(monitor.output_level(Channel::A), Some(Level::High));

        set_vcc(&iso, Side::One, V5V);
        assert_eq!(monitor.output_level(Channel::A), Some(Level::Low));
    }

    // -- event cost --------------------------------------------------

    /// One transition on one channel costs exactly one drive, and an
    /// unchanged re-evaluation costs none.
    ///
    /// This is the discipline the whole model is built around: a four-channel
    /// repeater that re-drove every output on every delivery would multiply
    /// engine resolutions fourfold for a signal only one channel carries.
    #[rstest]
    fn a_channel_transition_costs_exactly_one_drive() {
        // Fail-safe, so the settled default is LOW and driving the input high
        // really is a transition.
        let (iso, monitor) = isolator(Config::new(Variant::Iso6741).fail_safe(true));
        power_up(&iso);
        let settled = monitor.drive_count();

        set_input(&iso, Channel::A, V3V3);
        let after_first = monitor.drive_count();
        assert_eq!(after_first, settled + 1, "one transition, one drive");

        // The same level again, ten times over: no engine traffic at all.
        for _ in 0..10 {
            set_input(&iso, Channel::A, V3V3);
        }
        assert_eq!(
            monitor.drive_count(),
            after_first,
            "an unchanged input must cost nothing"
        );

        // A different analog voltage that projects to the same level is also
        // no change.
        set_input(&iso, Channel::A, NetState::Analog(3.0));
        assert_eq!(monitor.drive_count(), after_first);

        // The other channels never moved.
        set_input(&iso, Channel::A, DOWN);
        assert_eq!(monitor.drive_count(), after_first + 1);
    }

    /// A supply change re-evaluates every channel, but only the ones whose
    /// drive actually changes cost anything.
    #[rstest]
    fn a_supply_change_costs_at_most_one_drive_per_channel() {
        let (iso, monitor) = isolator(Config::new(Variant::Iso6741));
        power_up(&iso);
        for channel in Channel::ALL {
            set_input(&iso, channel, V3V3);
        }
        let settled = monitor.drive_count();

        set_vcc(&iso, Side::Two, DOWN);
        // Three channels have their output on side 2; channel D's is on
        // side 1 and is untouched.
        assert_eq!(monitor.drive_count(), settled + 3);

        // Re-delivering the same rail state costs nothing.
        set_vcc(&iso, Side::Two, DOWN);
        assert_eq!(monitor.drive_count(), settled + 3);
    }

    // -- pulse relay --------------------------------------------------

    fn train(freq_hz: u32, since_us: u64) -> PulseTrain {
        use embsim_board::{PulseDirection, PulseSegment};
        PulseTrain {
            pulses: PulseSegment {
                emitted: 0,
                freq_hz,
                total: None,
                since_us,
            },
            direction: PulseDirection::Forward,
        }
    }

    fn deliver_train(iso: &Iso67xx, channel: Channel, train: PulseTrain) {
        let mut guard = iso.core.state.lock().unwrap();
        guard.train_in[channel.index()] = Some(train);
        let wiring = *iso
            .core
            .wiring
            .iter()
            .find(|w| w.channel == channel)
            .expect("channel exists");
        iso.core.apply_train(&mut guard, &wiring);
    }

    /// A pulse channel relays its segment **verbatim** — same rate, same
    /// anchor, same accumulated count — so the downstream count cannot drift
    /// from the source's.
    #[rstest]
    fn a_pulse_channel_relays_its_segment_verbatim() {
        let config = Config::new(Variant::Iso6741).with_pulse_channel(Channel::A);
        let (iso, monitor) = isolator(config);
        power_up(&iso);

        let segment = train(8_192, 1_000);
        deliver_train(&iso, Channel::A, segment);
        assert_eq!(monitor.relayed_train(Channel::A), Some(segment));
        // The relayed train integrates identically to the source's.
        assert_eq!(segment.emitted_at(1_001_000), 8_192);
    }

    /// A rate change costs one relay; re-delivering the same segment costs
    /// none. This is why a step train crossing the barrier does not scale
    /// engine traffic with the step rate.
    #[rstest]
    fn relaying_costs_one_event_per_rate_change() {
        let config = Config::new(Variant::Iso6741).with_pulse_channel(Channel::A);
        let (iso, monitor) = isolator(config);
        power_up(&iso);

        deliver_train(&iso, Channel::A, train(8_192, 1_000));
        assert_eq!(monitor.train_count(), 1);
        deliver_train(&iso, Channel::A, train(8_192, 1_000));
        assert_eq!(
            monitor.train_count(),
            1,
            "an unchanged train relays nothing"
        );
        deliver_train(&iso, Channel::A, train(16_384, 2_000));
        assert_eq!(monitor.train_count(), 2, "a rate change relays once");
    }

    /// A channel that stops passing holds its train — once — rather than
    /// leaving the last rate running across a dead barrier forever.
    #[rstest]
    fn losing_a_supply_holds_the_relayed_train() {
        let config = Config::new(Variant::Iso6741).with_pulse_channel(Channel::A);
        let (iso, monitor) = isolator(config);
        power_up(&iso);
        deliver_train(&iso, Channel::A, train(8_192, 1_000));

        set_vcc(&iso, Side::One, DOWN);
        assert_eq!(monitor.relayed_train(Channel::A), Some(PulseTrain::IDLE));
        let held = monitor.train_count();
        set_vcc(&iso, Side::Two, DOWN);
        assert_eq!(monitor.train_count(), held, "already held; no second event");
    }

    /// A pulse channel declares the stream roles that make the route form,
    /// and a level channel declares none.
    #[rstest]
    fn channel_roles_shape_the_stream_declarations() {
        let config = Config::new(Variant::Iso6741)
            .with_pulse_channel(Channel::A)
            .with_serial_channel(Channel::B, 115_200);
        let (iso, _) = isolator(config);
        let stream = |number: &str| {
            iso.pins()
                .iter()
                .find(|p| p.number == number)
                .expect("pin")
                .stream
        };
        assert_eq!(stream("3"), Some(StreamRole::PulseSink)); // INA
        assert_eq!(stream("14"), Some(StreamRole::PulseSource)); // OUTA
        assert_eq!(stream("4"), Some(StreamRole::Consumer { baud_hz: 115_200 })); // INB
        assert_eq!(
            stream("13"),
            Some(StreamRole::Producer { baud_hz: 115_200 })
        ); // OUTB
        assert_eq!(stream("5"), None); // INC, a level channel
        assert_eq!(stream("12"), None); // OUTC
    }

    /// A serial channel leaves its output pin's level to the byte pacer.
    #[rstest]
    fn a_serial_channel_never_drives_its_output_level() {
        let config = Config::new(Variant::Iso6741).with_serial_channel(Channel::A, 115_200);
        let (iso, monitor) = isolator(config);
        power_up(&iso);
        let settled = monitor.drive_count();
        set_input(&iso, Channel::A, DOWN);
        assert_eq!(
            monitor.drive_count(),
            settled,
            "a serial channel's output pin belongs to the byte pacer"
        );
        assert_eq!(monitor.output_drive(Channel::A), None);
    }
}
