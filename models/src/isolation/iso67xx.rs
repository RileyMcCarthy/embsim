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
//! - **"INx open" is any input the receiver reads no level on.** A floating
//!   net, a net fought to a voltage inside the `V_IL`..`V_IH` dead band, and
//!   any other voltage inside it all mean the same thing to the input buffer
//!   ([`embsim_board::DeadBand::Unknown`]), and the datasheet's answer for all
//!   of them is the default output state. This
//!   is what makes an isolator fed by a dead upstream part still present a
//!   *defined* output — the behavior the F suffix is bought for.
//! - **"Undetermined" is modeled as high impedance.** With its own supply
//!   down, the output buffer has nothing to drive from, and inventing a level
//!   for it would hide the failure. Releasing lets the engine report the truth
//!   (`FloatingSense`, or whatever the board's own pull does), which is the
//!   same choice [`crate::ads122u04_component`] makes for an unpowered chip.
//!
//! # A clock crosses as a clock
//!
//! Every channel is just a channel: it senses its input net and drives its
//! output net. When the input net carries a square wave — a
//! [`embsim_board::PeriodicSense`], a step clock's
//! [`embsim_board::Drive::Periodic`] resolved on the P2's side — the channel
//! drives its output as a
//! [`embsim_board::Drive::Periodic`] too: its **own** output ports (the
//! output side's rail and ground behind [`Config::output_impedance_ohms`])
//! around the input's segment **forwarded verbatim** — the source's own
//! rate, accumulated count, ceiling and anchor, never re-anchored at the
//! instant the isolator re-drove. That is a decision, not a discovery
//! (`sil-unified-drive.md`, "The consequence worth deciding deliberately"):
//! the output depends on the far side's rail, so a supply that moves re-drives
//! the channel, and a re-anchoring relay would quietly desynchronise the
//! carriage; forwarding the segment keeps the downstream count bit-identical
//! to the firmware's, and relaying it costs **one drive per rate change**, not
//! one per edge. The channel relays a running square wave only where its
//! two phases settle to two levels through the channel's own input
//! thresholds, `0.3/0.7 × VCCI` ([`embsim_board::PeriodicSense::rate`], as
//! every consumer of a rate takes it: the input then switches every cycle);
//! a held segment — the stop that ends a relayed train and carries its
//! final count — is forwarded where its phases settle to two levels the
//! same way. Any other wave is the level the input reads from it
//! ([`embsim_board::Sense::level`]: one level where both phases settle to
//! it) or, where it reads none — a phase inside the dead band — an open
//! input, and the output presents the default state. A channel that stops
//! passing (its input side down) presents its default state, a level, and
//! the clock stops at the barrier.
//!
//! A UART needs no role of its own. It used to have one — the input pin a
//! stream `Consumer`, the output a `Producer`, bytes relayed one for one and
//! the output pin's *level* left to the byte pacer — because a byte crossing
//! the barrier was routed rather than carried. Now that a UART's bits are on
//! the net, a serial channel is a level channel: the isolator repeats edges
//! and never learns what they spell.
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
//! - **Supply voltage is read as `VCCx − GNDx`** — each side's supply pin
//!   is measured against that side's ground ([`embsim_board::Sense`]), and
//!   a side whose ground nothing holds is down — but the outputs **drive**
//!   that voltage above 0 V in the engine's frame, not above `GNDx`: exact
//!   while a side's ground sits at 0 V there, which every board's grounds
//!   do. An isolated ground at another potential is not represented.
//! - **Supply current, level translation limits, ESD, and thermals.**

use std::sync::{Arc, Mutex};

use embsim_board::{
    AttachError, Component, ComponentNetIo, DeadBand, Drive, Level, Ohms, PeriodicSchedule,
    PinDecl, PinHandle, Sense, TheveninDrive, Thresholds, Volts,
};

use super::{level_drive, require_positive, supply_volts, PartConfigError};

// ============================================================
// Datasheet constants
// ============================================================

/// Supply at or above which a side counts as **powered up**: `PU` is
/// `VCC >= 1.71 V` (SLLSFJ6G §9.4 Table 9-2, note 1 — the same number as the
/// rising UVLO threshold maximum in §7.3).
pub const DEFAULT_SUPPLY_MIN_VOLTS: Volts = 1.71;

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
    /// `V_IH` as a fraction of the input supply ([`DEFAULT_VIH_RATIO`]).
    pub vih_ratio: f64,
    /// `V_IL` as a fraction of the input supply ([`DEFAULT_VIL_RATIO`]).
    pub vil_ratio: f64,
    /// Output Thevenin source impedance
    /// ([`DEFAULT_OUTPUT_IMPEDANCE_OHMS`]).
    pub output_impedance_ohms: Ohms,
}

impl Config {
    /// A variant with every channel carrying a level and every parameter at
    /// its datasheet default.
    pub fn new(variant: Variant) -> Self {
        Self {
            variant,
            fail_safe: false,
            supply_min_volts: DEFAULT_SUPPLY_MIN_VOLTS,
            vih_ratio: DEFAULT_VIH_RATIO,
            vil_ratio: DEFAULT_VIL_RATIO,
            output_impedance_ohms: DEFAULT_OUTPUT_IMPEDANCE_OHMS,
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

    /// The inputs' (and enables') thresholds, **relative** to the input
    /// side's supply: `V_IL`/`V_IH` as [`Self::vil_ratio`]/[`Self::vih_ratio`]
    /// of `VCCI` (SLLSFJ6G §7.3), no hysteresis named, and between the two
    /// the function table's indeterminate input — no level
    /// ([`DeadBand::Unknown`]).
    pub fn input_thresholds(&self) -> Thresholds {
        Thresholds::new(self.vil_ratio, self.vih_ratio, 0.0, DeadBand::Unknown)
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
        require_positive("output_impedance_ohms", self.output_impedance_ohms)?;
        require_positive("vih_ratio", self.vih_ratio)?;
        require_positive("vil_ratio", self.vil_ratio)?;
        if self.vil_ratio >= self.vih_ratio {
            return Err(PartConfigError::InvertedThresholds {
                vil_ratio: self.vil_ratio,
                vih_ratio: self.vih_ratio,
            });
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
}

// ============================================================
// Core state
// ============================================================

/// Everything the channels read, written by sense callbacks on the engine
/// thread.
#[derive(Debug)]
struct CoreState {
    /// What each side's `VCC` pin was last handed (against that side's
    /// ground).
    vcc: [Sense; 2],
    /// What each side's enable pin was last handed. A side with no enable
    /// pin keeps [`NOTHING`] — no source reaches it — which the datasheet
    /// reads as "open" and therefore enabled.
    enable: [Sense; 2],
    /// The level each side's enable last read — its receiver's last level.
    enable_level: [Option<Level>; 2],
    /// What each channel's input pin was last handed.
    input: [Sense; 4],
    /// The level each channel's input last read — its receiver's last
    /// level.
    input_level: [Option<Level>; 4],
    /// Output pin handles, `None` until attach (and in unit tests, where the
    /// drive decision is bookkeeping only).
    output: [Option<PinHandle>; 4],
    /// Last drive applied per channel: `None` = never applied,
    /// `Some(None)` = released — every output from power-on, as declared —
    /// `Some(Some(d))` = driving `d`.
    applied: [Option<Option<Drive>>; 4],
    /// Count of level drives (a level or a release) actually issued — the
    /// event-cost meter.
    drives: u64,
    /// Count of periodic drives actually issued: one per relayed rate
    /// change.
    trains: u64,
}

/// A pin nothing has been handed yet: no voltage, no clock — what the
/// engine hands a net no source reaches.
const NOTHING: Sense = Sense {
    volts: None,
    periodic: None,
    at_ns: 0,
};

impl CoreState {
    fn new() -> Self {
        Self {
            vcc: [NOTHING; 2],
            enable: [NOTHING; 2],
            enable_level: [None; 2],
            input: [NOTHING; 4],
            input_level: [None; 4],
            output: Default::default(),
            // Every output as declared: released.
            applied: [Some(None); 4],
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
    /// A side's supply voltage when it is up, against that side's ground.
    fn side_rail(&self, state: &CoreState, side: Side) -> Option<Volts> {
        supply_volts(&state.vcc[side.index()], self.config.supply_min_volts)
    }

    /// Whether a side's supply is up.
    fn side_up(&self, state: &CoreState, side: Side) -> bool {
        self.side_rail(state, side).is_some()
    }

    /// The level a sense projects to through the channel thresholds of a
    /// side powered at `rail` — the receiver's projection, chosen by its
    /// `last` level. A side that is down projects nothing.
    fn project(&self, sense: &Sense, rail: Option<Volts>, last: Option<Level>) -> Option<Level> {
        sense.level(&self.config.input_thresholds().scaled(rail?), last)
    }

    /// A side's `VCC` pin was handed `sensed`: every threshold on that
    /// side moves with it, so every receiver re-projects, then every
    /// channel re-evaluates.
    fn on_vcc(&self, state: &mut CoreState, side: Side, sensed: Sense) {
        state.vcc[side.index()] = sensed;
        self.reproject(state);
        self.refresh_all(state);
    }

    /// A side's enable pin was handed `sensed`.
    fn on_enable(&self, state: &mut CoreState, side: Side, sensed: Sense) {
        state.enable[side.index()] = sensed;
        self.reproject(state);
        self.refresh_all(state);
    }

    /// A channel's input pin was handed `sensed`: its receiver projects it,
    /// and that channel alone re-evaluates.
    fn on_input(&self, state: &mut CoreState, wiring: &Wiring, sensed: Sense) {
        let index = wiring.channel.index();
        state.input[index] = sensed;
        let rail = self.side_rail(state, wiring.input_side);
        state.input_level[index] = self.project(&sensed, rail, state.input_level[index]);
        self.refresh(state, wiring);
    }

    /// Re-project a side's enable and every input on it, keeping each
    /// receiver's last level — after its supply or its own sense moved.
    fn reproject(&self, state: &mut CoreState) {
        for side in [Side::One, Side::Two] {
            let rail = self.side_rail(state, side);
            let i = side.index();
            state.enable_level[i] = self.project(&state.enable[i], rail, state.enable_level[i]);
        }
        for wiring in &self.wiring {
            let rail = self.side_rail(state, wiring.input_side);
            let i = wiring.channel.index();
            state.input_level[i] = self.project(&state.input[i], rail, state.input_level[i]);
        }
    }

    /// Whether a side's outputs are enabled. `ENx` high **or open** enables
    /// (SLLSFJ6G Table 6-1); low disables. Open is a pin no source reaches
    /// — handed no voltage and no clock. A fought-over enable, or one inside
    /// its dead band, is treated as disabled — the conservative reading,
    /// and the one that does not invent a winner.
    fn side_enabled(&self, state: &CoreState, side: Side) -> bool {
        let enable = &state.enable[side.index()];
        if enable.volts.is_none() && enable.periodic.is_none() {
            return true;
        }
        state.enable_level[side.index()] == Some(Level::High)
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
        state.input_level[wiring.channel.index()].unwrap_or_else(|| self.config.default_level())
    }

    /// The drive a channel's output pin should present: released while the
    /// output stage is not live; a square wave forwarded **verbatim** while
    /// the channel is passing a clock — a running segment whose phases
    /// settle to two levels through the input's thresholds
    /// ([`embsim_board::PeriodicSense::rate`]), or a held one whose phases
    /// do (the module docs, "A clock crosses as a clock"); otherwise the
    /// level the function table gives for the level the input reads. The
    /// ports are the output side's rail against its ground and that ground,
    /// in the engine's frame while the ground sits at 0 V there — every
    /// board's grounds do; a ground offset is not modelled.
    fn desired_drive(&self, state: &CoreState, wiring: &Wiring) -> Option<Drive> {
        if !self.output_live(state, wiring) {
            return None;
        }
        let rail = self.side_rail(state, wiring.output_side)?;
        let port = |level| level_drive(level, rail, self.config.output_impedance_ohms);
        if let (Some(input_rail), Some(clock)) = (
            self.side_rail(state, wiring.input_side),
            state.input[wiring.channel.index()].periodic,
        ) {
            let thresholds = self.config.input_thresholds().scaled(input_rail);
            // Two levels need both phases outside the dead band, on
            // opposite sides, whatever the input read before.
            if let (Some(hi), Some(lo)) = clock.levels(&thresholds, None) {
                let relayed =
                    clock.rate(&thresholds).is_some() || (clock.segment.freq_hz == 0 && hi != lo);
                if relayed {
                    return Some(Drive::Periodic {
                        hi: port(hi),
                        lo: port(lo),
                        segment: clock.segment,
                    });
                }
            }
        }
        Some(Drive::Thevenin(port(self.output_level(state, wiring))))
    }

    /// Apply a channel's output — **only when it changed**.
    ///
    /// This is the whole event-cost discipline: a repeater that re-drove on
    /// every delivery would multiply engine resolutions by its channel count
    /// and by every unrelated supply wobble. A relayed clock is one drive per
    /// rate change, the segment carried as the source published it.
    fn refresh(&self, state: &mut CoreState, wiring: &Wiring) {
        let index = wiring.channel.index();
        let desired = self.desired_drive(state, wiring);
        if state.applied[index] == Some(desired) {
            return;
        }
        state.applied[index] = Some(desired);
        if matches!(desired, Some(Drive::Periodic { .. })) {
            state.trains += 1;
        } else {
            state.drives += 1;
        }
        if let Some(pin) = &state.output[index] {
            match desired {
                Some(drive) => pin.drive(drive),
                None => pin.release(),
            }
        }
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
    /// The level drive the channel's output pin is presenting, or `None`
    /// when the pin is released (unpowered output side, or disabled by
    /// `ENx`) or relaying a clock ([`Self::relayed_segment`]).
    pub fn output_drive(&self, channel: Channel) -> Option<TheveninDrive> {
        match self.core.state.lock().unwrap().applied[channel.index()].flatten() {
            Some(Drive::Thevenin(drive)) => Some(drive),
            _ => None,
        }
    }

    /// The logic level the channel's output is presenting, or `None` when the
    /// pin is released.
    pub fn output_level(&self, channel: Channel) -> Option<Level> {
        let state = self.core.state.lock().unwrap();
        let wiring = self.core.wiring.iter().find(|w| w.channel == channel)?;
        // A released pin has no level, and neither does a relayed clock;
        // only a level drive does.
        let Some(Drive::Thevenin(_)) = state.applied[channel.index()].flatten() else {
            return None;
        };
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

    /// The segment the channel's output is relaying as a clock right now —
    /// the input's own, verbatim — or `None` while it presents a level or is
    /// released.
    pub fn relayed_segment(&self, channel: Channel) -> Option<PeriodicSchedule> {
        match self.core.state.lock().unwrap().applied[channel.index()].flatten() {
            Some(Drive::Periodic { segment, .. }) => Some(segment),
            _ => None,
        }
    }

    /// Total level drives (a level or a release) this part has issued since
    /// construction.
    ///
    /// The event-cost meter: a level change on one channel costs exactly one,
    /// and an unchanged re-evaluation costs zero.
    pub fn drive_count(&self) -> u64 {
        self.core.state.lock().unwrap().drives
    }

    /// Total periodic drives this part has issued since construction: one per
    /// relayed rate change.
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
/// // Every channel is a channel: a level crosses as a level, a clock as a
/// // clock.
/// let isolator = Iso67xx::new(config).expect("valid");
/// assert_eq!(isolator.pins().len(), 16);
/// ```
#[derive(Debug)]
pub struct Iso67xx {
    /// The pin table ([`declare`]): each side's supply measured against
    /// that side's first ground pin — the declaration the build's domain
    /// lint reads (`embsim_board::Finding::UnreferencedDomain`): a side
    /// whose supply a source reaches while its ground floats is a domain
    /// measured against nothing — and every input and enable reading
    /// through the datasheet's ratios of its own side's supply.
    pins: Vec<PinDecl>,
    core: Arc<Core>,
}

impl Iso67xx {
    /// Create an isolator from a validated configuration.
    pub fn new(config: Config) -> Result<Self, PartConfigError> {
        config.validate()?;
        let specs = config.variant.pin_specs();
        let wiring = wiring_for(&config);
        let pins = specs
            .iter()
            .map(|spec| declare(spec, specs, &config))
            .collect();
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
            }
        })
        .collect()
}

/// Turn one pin-table row into a [`PinDecl`]: a side's supply measured
/// against that side's first ground pin, an input or enable reading
/// through the configuration's `V_IL`/`V_IH` ratios of its own side's
/// supply (SLLSFJ6G §7.3, [`DEFAULT_VIL_RATIO`]/[`DEFAULT_VIH_RATIO`]; the
/// datasheet names no input hysteresis) against that side's ground.
fn declare(spec: &PinSpec, specs: &[PinSpec], config: &Config) -> PinDecl {
    let side_pin = |want: Role| specs.iter().find(|s| s.role == want).map(|s| s.number);
    let referenced = |pin: PinDecl, side: Side| match side_pin(Role::Gnd(side)) {
        Some(gnd) => pin.with_reference(gnd),
        None => pin,
    };
    let pin = match spec.role {
        Role::Vcc(side) => referenced(PinDecl::power_in(spec.number), side),
        Role::Gnd(_) => PinDecl::power_in(spec.number),
        // A no-connect pad is declared (the netlist has a node for it, on an
        // `unconnected-(...)` net) but contributes and senses nothing, so a
        // deliberately dangling pin raises no finding.
        Role::NoConnect => PinDecl::passive(spec.number),
        Role::Enable(side) | Role::Input(_, side) => {
            let pin = referenced(
                PinDecl::digital_in(spec.number, config.input_thresholds()),
                side,
            );
            match side_pin(Role::Vcc(side)) {
                Some(vcc) => pin.with_supply(vcc),
                None => pin,
            }
        }
        // The drive impedance is applied per drive: it is configuration,
        // not a `&'static` constant.
        // Released from power-on: an isolator with no rails yet drives
        // nothing, and the declaration says so.
        Role::Output(..) => PinDecl::digital_out(spec.number).with_idle(None),
    };
    pin.with_name(spec.name)
}

impl Component for Iso67xx {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        // 1. Claim the output pins. Each output is declared released from
        //    power-on, which is every channel's unpowered state, so there is
        //    nothing to publish until a rail or an input moves it.
        {
            let mut state = self.core.state.lock().unwrap();
            for wiring in &self.core.wiring {
                let index = wiring.channel.index();
                state.output[index] = Some(io.pin(wiring.output_pin)?);
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
                        core.on_vcc(&mut state, side, sensed);
                    })?;
                }
                Role::Enable(side) => {
                    let core = Arc::clone(&self.core);
                    io.on_sense(spec.number, move |sensed| {
                        let mut state = core.state.lock().unwrap();
                        core.on_enable(&mut state, side, sensed);
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
            let core = Arc::clone(&self.core);
            io.on_sense(wiring.input_pin, move |sensed| {
                let mut state = core.state.lock().unwrap();
                core.on_input(&mut state, &wiring, sensed);
            })?;
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

    /// A pin handed `volts` against its ground.
    const fn at(volts: Volts) -> Sense {
        Sense {
            volts: Some(volts),
            periodic: None,
            at_ns: 0,
        }
    }

    const V3V3: Sense = at(3.3);
    const V5V: Sense = at(5.0);
    const DOWN: Sense = at(0.0);

    fn isolator(config: Config) -> (Iso67xx, Iso67xxMonitor) {
        let isolator = Iso67xx::new(config).expect("valid config");
        let monitor = isolator.monitor();
        (isolator, monitor)
    }

    /// Drive the shared state directly, as the sense callbacks would.
    fn set_vcc(iso: &Iso67xx, side: Side, sensed: Sense) {
        let mut guard = iso.core.state.lock().unwrap();
        iso.core.on_vcc(&mut guard, side, sensed);
    }

    fn set_enable(iso: &Iso67xx, side: Side, sensed: Sense) {
        let mut guard = iso.core.state.lock().unwrap();
        iso.core.on_enable(&mut guard, side, sensed);
    }

    fn set_input(iso: &Iso67xx, channel: Channel, sensed: Sense) {
        let mut guard = iso.core.state.lock().unwrap();
        let wiring = *iso
            .core
            .wiring
            .iter()
            .find(|w| w.channel == channel)
            .expect("channel exists");
        iso.core.on_input(&mut guard, &wiring, sensed);
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
    #[case::zero_impedance(Config { output_impedance_ohms: 0.0, ..Config::new(Variant::Iso6741) })]
    #[case::negative_supply(Config { supply_min_volts: -1.0, ..Config::new(Variant::Iso6741) })]
    #[case::nan_supply(Config { supply_min_volts: f64::NAN, ..Config::new(Variant::Iso6741) })]
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
    #[case::at_vih(at(2.31), Level::High)]
    #[case::under_vil(at(0.98), Level::Low)]
    fn a_powered_channel_follows_its_input(#[case] input: Sense, #[case] expect: Level) {
        let (iso, monitor) = isolator(Config::new(Variant::Iso6741));
        power_up(&iso);
        set_input(&iso, Channel::A, input);
        assert_eq!(monitor.output_level(Channel::A), Some(expect));
        assert!(monitor.is_passing(Channel::A));
    }

    /// Row 2: an input the receiver reads no level on is "open", and the
    /// output goes to its default state — high for a plain part, low for an
    /// `F` part: a floating input, and one fought or resting inside the
    /// dead band (two 25 Ω drivers settle at 1.65 V).
    #[rstest]
    #[case::floating(Sense { volts: None, periodic: None, at_ns: 0 })]
    #[case::dead_band(at(1.65))]
    fn an_undecidable_input_gets_the_default_output(#[case] input: Sense) {
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
    #[case::open(Sense { volts: None, periodic: None, at_ns: 0 }, true)]
    #[case::high(V3V3, true)]
    #[case::at_vih(at(2.31), true)]
    #[case::low(DOWN, false)]
    #[case::fought_in_the_band(at(1.65), false)]
    fn the_enable_pin_gates_its_own_sides_outputs(#[case] enable: Sense, #[case] enabled: bool) {
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
        set_input(&iso, Channel::A, at(2.4));
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
        set_input(&iso, Channel::A, at(3.0));
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

    // -- clock relay --------------------------------------------------

    fn segment(freq_hz: u32, since_ns: u64) -> PeriodicSchedule {
        PeriodicSchedule {
            emitted: 0,
            freq_hz,
            total: None,
            since_ns,
        }
    }

    /// A square wave swinging rail to rail against the input's ground.
    fn clock(freq_hz: u32, since_ns: u64) -> Sense {
        Sense {
            volts: None,
            periodic: Some(embsim_board::PeriodicSense {
                hi: Some(3.3),
                lo: Some(0.0),
                segment: segment(freq_hz, since_ns),
            }),
            at_ns: 0,
        }
    }

    /// A channel handed a square wave relays its segment **verbatim** —
    /// same rate, same anchor, same accumulated count — between its own
    /// output ports, so the downstream count cannot drift from the
    /// source's.
    #[rstest]
    fn a_clock_crosses_with_its_segment_verbatim() {
        let (iso, monitor) = isolator(Config::new(Variant::Iso6741));
        power_up(&iso);

        set_input(&iso, Channel::A, clock(8_192, 1_000_000));
        let segment = segment(8_192, 1_000_000);
        assert_eq!(monitor.relayed_segment(Channel::A), Some(segment));
        assert_eq!(monitor.output_drive(Channel::A), None, "no level: a clock");
        // The relayed segment integrates identically to the source's.
        assert_eq!(segment.emitted_at_ns(1_001_000_000), 8_192);
        // Between the output side's own rail and ground.
        let applied = iso.core.state.lock().unwrap().applied[Channel::A.index()];
        let Some(Some(Drive::Periodic { hi, lo, .. })) = applied else {
            panic!("a periodic drive: {applied:?}");
        };
        assert_eq!((hi.volts, lo.volts), (3.3, 0.0));
        assert_eq!(hi.impedance, iso.core.config.output_impedance_ohms);
    }

    /// A square wave swinging `hi`/`lo` volts against the input's ground.
    fn swinging(freq_hz: u32, hi: Volts, lo: Volts) -> Sense {
        Sense {
            volts: None,
            periodic: Some(embsim_board::PeriodicSense {
                hi: Some(hi),
                lo: Some(lo),
                segment: segment(freq_hz, 1_000_000),
            }),
            at_ns: 0,
        }
    }

    /// A channel relays a clock only where its phases settle to two levels
    /// through the input's thresholds (0.99 V / 2.31 V at 3.3 V): a
    /// 0 V / 1.2 V wave puts its high phase in the dead band, so the input
    /// reads no level from it — an open input, the default state, high for
    /// a plain part and low for an `F` part; a 0 V / 0.9 V wave is a steady
    /// low, relayed as that level by both. A held segment whose phases
    /// cross — the stop that ends a relayed train — is forwarded verbatim.
    #[rstest]
    #[case::high_phase_in_the_band_plain(swinging(8_192, 1.2, 0.0), false, Level::High)]
    #[case::high_phase_in_the_band_fail_safe(swinging(8_192, 1.2, 0.0), true, Level::Low)]
    #[case::a_steady_low_plain(swinging(8_192, 0.9, 0.0), false, Level::Low)]
    #[case::a_steady_low_fail_safe(swinging(8_192, 0.9, 0.0), true, Level::Low)]
    fn a_clock_that_does_not_cross_the_input_is_a_level(
        #[case] input: Sense,
        #[case] fail_safe: bool,
        #[case] expect: Level,
    ) {
        let (iso, monitor) = isolator(Config::new(Variant::Iso6741).fail_safe(fail_safe));
        power_up(&iso);
        set_input(&iso, Channel::A, input);
        assert_eq!(monitor.relayed_segment(Channel::A), None, "no relay");
        assert_eq!(monitor.train_count(), 0);
        assert_eq!(monitor.output_level(Channel::A), Some(expect));
        assert_eq!(
            monitor.output_drive(Channel::A).map(|drive| drive.volts),
            Some(if expect == Level::High { 3.3 } else { 0.0 })
        );

        // The same channel handed a held segment that crosses forwards it.
        set_input(&iso, Channel::A, clock(0, 2_000_000));
        assert_eq!(
            monitor.relayed_segment(Channel::A),
            Some(segment(0, 2_000_000))
        );
    }

    /// A rate change costs one relay; re-delivering the same segment costs
    /// none. This is why a step train crossing the barrier does not scale
    /// engine traffic with the step rate.
    #[rstest]
    fn relaying_costs_one_event_per_rate_change() {
        let (iso, monitor) = isolator(Config::new(Variant::Iso6741));
        power_up(&iso);

        set_input(&iso, Channel::A, clock(8_192, 1_000_000));
        assert_eq!(monitor.train_count(), 1);
        set_input(&iso, Channel::A, clock(8_192, 1_000_000));
        assert_eq!(
            monitor.train_count(),
            1,
            "an unchanged segment relays nothing"
        );
        set_input(&iso, Channel::A, clock(16_384, 2_000_000));
        assert_eq!(monitor.train_count(), 2, "a rate change relays once");
    }

    /// A channel that stops passing presents its default state, a level:
    /// the clock stops at a dead barrier rather than running on.
    #[rstest]
    fn losing_the_input_side_stops_the_relayed_clock() {
        let (iso, monitor) = isolator(Config::new(Variant::Iso6741));
        power_up(&iso);
        set_input(&iso, Channel::A, clock(8_192, 1_000_000));

        set_vcc(&iso, Side::One, DOWN);
        assert_eq!(monitor.relayed_segment(Channel::A), None);
        assert_eq!(monitor.output_level(Channel::A), Some(Level::High));
        let trains = monitor.train_count();
        set_vcc(&iso, Side::One, DOWN);
        assert_eq!(monitor.train_count(), trains, "no second event");
    }

    /// Every channel declares plain pins: a level channel and a clock
    /// channel are the same channel, an input a sense and an output a
    /// drive.
    #[rstest]
    fn every_channel_declares_plain_pins() {
        let (iso, _) = isolator(Config::new(Variant::Iso6741));
        let pin = |number: &str| *iso.pins().iter().find(|p| p.number == number).expect("pin");
        let ina = pin("3");
        assert_eq!(
            ina.senses_at_build(),
            Some(embsim_board::SenseKind::Digital)
        );
        assert_eq!(
            ina.thresholds,
            Some(Thresholds::new(
                DEFAULT_VIL_RATIO,
                DEFAULT_VIH_RATIO,
                0.0,
                DeadBand::Unknown
            )),
            "INA reads through the datasheet's ratios"
        );
        assert_eq!(ina.supply, Some("1"), "of side 1's VCC1");
        let outa = pin("14");
        assert!(outa.drives() && outa.senses_at_build().is_none()); // OUTA
    }
}
