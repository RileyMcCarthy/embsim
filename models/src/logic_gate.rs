//! Model: single-input **CMOS logic gates** — the inverters between an MCU
//! pin and the thing it drives. Two parts, one model:
//!
//! - **NXP 74LVC2G04**, dual inverter (`U101`, the P2-EC32MB's oscillator
//!   buffer, and `U601`, its LED buffer);
//! - **TI SN74LVC1G14**, single Schmitt-trigger inverter (the 21
//!   front-panel LED drivers `U9`–`U34` on the MaD EdgeBoard).
//!
//! Per channel: the input is sensed and projected through the part's own
//! thresholds with hysteresis; the output is driven at the datasheet's
//! output impedance after the datasheet's propagation delay, as a
//! **scheduled instant** — never in the same pass as the input edge. A
//! channel whose input is handed a *rate* — an [`embsim_board::PeriodicSense`] with
//! a running segment, from a step clock or an oscillator, whose two phases
//! cross the input's own thresholds ([`embsim_board::PeriodicSense::rate`]) —
//! is in **rate mode**: it drives its output as an
//! [`embsim_board::Drive::Periodic`], its own high port (`V_CC` behind
//! `R_OH`) and low port (0 V behind `R_OL`) around the input's segment
//! relayed verbatim, at once — which is what lets a self-biased stage (an
//! inverter with a resistor from its output back to its input, AC-coupled
//! to an oscillator) settle in one pass, with no sense→drive iteration: the
//! square wave its own output carries back through the feedback resistor is
//! the same segment it is relaying. A clock whose phases settle to one
//! level through the input's thresholds is that level to the channel, and
//! one with a phase that settles to none is no level (level mode, below).
//! See [`Mode`].
//!
//! **A self-biased input** is the one exception, found at attach: a channel
//! whose input node has a resistor to its own output node
//! ([`embsim_board::ComponentNetIo::resistors_at`], read once) relays every
//! running segment its input carries, whatever its phases. That resistor
//! returns the output's average to the input, so the input rests at the
//! stage's own switching point, and a swing coupled onto it through a
//! capacitor is a swing *around* that point: it crosses it every cycle,
//! however small. The engine hands a coupled node its source's swing, which
//! the capacitor has stripped of any DC level (`NODES.md` §10, the periodic
//! row), so the input's thresholds — absolute, measured from ground — have
//! nothing to place it against. The P2-EC32MB's `U101` is the case: `R101`
//! (100 kΩ) from `2Y` back to `2A`, and `C132` coupling the TCXO's 0.8 V
//! clipped sine onto `2A`. The 74LVC2G04 datasheet (Rev. 13) has no
//! application section and says nothing of operation around the switching
//! point, so this is the circuit's reasoning, not a figure: the stage's
//! gain there is not modelled, only that the swing crosses it.
//!
//! # Datasheet provenance
//!
//! **74LVC2G04** — Nexperia *74LVC2G04 Dual inverter, Product data sheet,
//! Rev. 13 — 15 August 2023*:
//!
//! - Table 3 "Pin description" (GW / SOT363-2 package): 1 `1A`, 2 `GND`,
//!   3 `2A`, 4 `2Y`, 5 `V_CC`, 6 `1Y`. [`LVC2G04_PINS_SOT363`]; the
//!   P2-EC32MB netlist names the same pins by function,
//!   [`LVC2G04_PINS_BY_FUNCTION`].
//! - Table 4 "Function table": `nY` is the complement of `nA`.
//! - Table 6 "Recommended operating conditions": `V_CC` 1.65 V to 5.5 V.
//!   [`LVC2G04_SUPPLY_MIN_VOLTS`].
//! - Table 7 "Static characteristics", `V_CC` = 2.7 V to 3.6 V: `V_IH` min
//!   2.0 V, `V_IL` max 0.8 V ([`LVC2G04_HIGH_AT_VOLTS`],
//!   [`LVC2G04_LOW_AT_VOLTS`]); `V_OH` min 2.3 V at `I_O` = −24 mA,
//!   `V_CC` = 3.0 V, and `V_OL` max 0.55 V at `I_O` = 24 mA, `V_CC` = 3.0 V
//!   — the worst-case output impedances `(3.0 − 2.3) / 0.024` and
//!   `0.55 / 0.024` ([`LVC2G04_R_OH_OHMS`], [`LVC2G04_R_OL_OHMS`]). Note
//!   \[1\]: typical values are at `V_CC` = 3.3 V ([`LVC2G04_NOMINAL_SUPPLY_VOLTS`]).
//!   The general description says "Schmitt-trigger action at all inputs";
//!   the table gives the two thresholds and no separate hysteresis figure,
//!   so the band between `V_IL` and `V_IH` is where the input holds its
//!   last level.
//! - Table 8 "Dynamic characteristics": `t_pd`, `V_CC` = 3.0 V to 3.6 V,
//!   −40 °C to +85 °C, max 4.1 ns (`C_L` = 50 pF, Table 10).
//!   [`LVC2G04_T_PD_NS`].
//!
//! **SN74LVC1G14** — Texas Instruments *SN74LVC1G14 Single Schmitt-Trigger
//! Inverter*, **SCES218AA**, April 1999 – revised October 2025:
//!
//! - Table 4-1 "Pin Functions" (DBV / SOT-23-5): 1 `N.C.`, 2 `A`, 3 `GND`,
//!   4 `Y`, 5 `V_CC`; `Y = A̅` (§3). [`LVC1G14_PINS_SOT23`].
//! - §5.3 "Recommended Operating Conditions": `V_CC` 1.65 V to 5.5 V.
//!   [`LVC1G14_SUPPLY_MIN_VOLTS`].
//! - §5.5 "Electrical Characteristics", `V_CC` = 3 V (the row nearest the
//!   3.3 V the boards run at; the table has no 3.3 V row), DBV package,
//!   −40 °C to 85 °C: `V_T+` 1.5 V min / 1.87 V max, `V_T−` 0.84 V min /
//!   1.14 V max, hysteresis `ΔV_T` 0.56 V to 0.87 V. The model flips high
//!   only at `V_T+` **max** and low only at `V_T−` **min** — the widest
//!   band any part of the family is guaranteed not to have flipped in
//!   ([`LVC1G14_HIGH_AT_VOLTS`], [`LVC1G14_LOW_AT_VOLTS`]). `V_OH` 2.3 V
//!   min at `I_OH` = −24 mA, `V_CC` = 3 V; `V_OL` 0.55 V max at `I_OL` =
//!   24 mA, `V_CC` = 3 V ([`LVC1G14_R_OH_OHMS`], [`LVC1G14_R_OL_OHMS`]).
//!   Note (1): typical values at `V_CC` = 3.3 V.
//! - §5.6 "Switching Characteristics: −40 °C to 85 °C": `t_pd`, A to Y,
//!   `V_CC` = 3.3 V ± 0.3 V, `C_L` = 15 pF, 1.5 ns min / 4.6 ns max.
//!   [`LVC1G14_T_PD_NS`].
//!
//! Each `t_pd` is the datasheet **maximum** — the conservative bound, the
//! same choice the isolators make for their output impedance.
//!
//! # Deliberate simplifications
//!
//! - **An input with no level releases the output.** TI's §5.3 note (1)
//!   says unused inputs must be held at `V_CC` or GND; neither datasheet
//!   says what an open input produces, and the engine invents no level
//!   (`DESIGN.md` rule 6). A floating input has none; a node voltage inside
//!   the threshold band holds the SN74LVC1G14's last level — the Schmitt
//!   trigger's hysteresis ([`embsim_board::DeadBand::HoldLast`]) — and gives
//!   the 74LVC2G04, whose Table 7 guarantees neither level there, none
//!   ([`embsim_board::DeadBand::Unknown`]). A fought input is handed the
//!   voltage the fight settled at and reads through the same rule.
//! - **Rate mode relays the segment, not the waveform**: the output's two
//!   phases are the datasheet's own high and low ports, the inversion is
//!   not modelled (phase is not, `NODES.md` §10), and neither is the duty —
//!   a segment has a rate and no duty.
//! - **Output impedance is the worst case at 24 mA**, one number per
//!   level; the `V_OH`/`I_OH` curves (TI Figure 5-1/5-2) are not modelled.
//! - **`V_CC` is the supply pin's voltage against the gate's `GND` pin**,
//!   and the output drives it above 0 V in the engine's frame — exact while
//!   `GND` sits at 0 V there, as the isolators assume. A supply below the
//!   minimum, or one that names no voltage, releases every output; the
//!   partial-power-down `I_off` behaviour is not modelled beyond that.
//! - **Input rise/fall-rate limits, input capacitance and `C_pd`** are not
//!   modelled.

use std::sync::{Arc, Mutex};

use embsim_board::{
    AttachError, Component, ComponentNetIo, DeadBand, Drive, Level, Ohms, PeriodicSchedule,
    PinDecl, PinHandle, PinRole, Sense, TheveninDrive, Thresholds, Volts,
};
use embsim_core::virtual_clock;

use crate::isolation::{require_positive, supply_volts, PartConfigError};

// ============================================================
// Datasheet constants
// ============================================================

/// 74LVC2G04 `V_IH` min at `V_CC` = 2.7 V to 3.6 V: 2.0 V (Table 7).
pub const LVC2G04_HIGH_AT_VOLTS: Volts = 2.0;
/// 74LVC2G04 `V_IL` max at `V_CC` = 2.7 V to 3.6 V: 0.8 V (Table 7).
pub const LVC2G04_LOW_AT_VOLTS: Volts = 0.8;
/// 74LVC2G04 input hysteresis as a separate figure: none — Table 7 gives
/// the two thresholds and no `ΔV_T` (the band between them is where the
/// input holds its last level).
pub const LVC2G04_HYSTERESIS_VOLTS: Volts = 0.0;
/// 74LVC2G04 high-level output impedance: `(3.0 V − V_OH min 2.3 V) /
/// 24 mA` (Table 7, `I_O` = −24 mA, `V_CC` = 3.0 V).
pub const LVC2G04_R_OH_OHMS: Ohms = (3.0 - 2.3) / 0.024;
/// 74LVC2G04 low-level output impedance: `V_OL max 0.55 V / 24 mA`
/// (Table 7, `I_O` = 24 mA, `V_CC` = 3.0 V).
pub const LVC2G04_R_OL_OHMS: Ohms = 0.55 / 0.024;
/// 74LVC2G04 `t_pd` max at `V_CC` = 3.0 V to 3.6 V, −40 °C to +85 °C:
/// 4.1 ns (Table 8; `C_L` = 50 pF, Table 10). Integer nanoseconds are what
/// the engine schedules; 4.1 ns → 5 ns, the first instant at or after the
/// bound (the maximum is a bound, and an instant before it is inside it).
pub const LVC2G04_T_PD_NS: u64 = 5;
/// 74LVC2G04 `V_CC` min, recommended operating conditions: 1.65 V (Table 6).
pub const LVC2G04_SUPPLY_MIN_VOLTS: Volts = 1.65;
/// 74LVC2G04 typical-value supply: 3.3 V (Table 7 note \[1\]).
pub const LVC2G04_NOMINAL_SUPPLY_VOLTS: Volts = 3.3;

/// SN74LVC1G14 `V_T+` max at `V_CC` = 3 V, DBV package: 1.87 V (§5.5).
pub const LVC1G14_HIGH_AT_VOLTS: Volts = 1.87;
/// SN74LVC1G14 `V_T−` min at `V_CC` = 3 V, DBV package: 0.84 V (§5.5).
pub const LVC1G14_LOW_AT_VOLTS: Volts = 0.84;
/// SN74LVC1G14 hysteresis `ΔV_T` min at `V_CC` = 3 V, DBV package: 0.56 V
/// (§5.5).
pub const LVC1G14_HYSTERESIS_VOLTS: Volts = 0.56;
/// SN74LVC1G14 high-level output impedance: `(3 V − V_OH min 2.3 V) /
/// 24 mA` (§5.5, `I_OH` = −24 mA, `V_CC` = 3 V).
pub const LVC1G14_R_OH_OHMS: Ohms = (3.0 - 2.3) / 0.024;
/// SN74LVC1G14 low-level output impedance: `V_OL max 0.55 V / 24 mA`
/// (§5.5, `I_OL` = 24 mA, `V_CC` = 3 V).
pub const LVC1G14_R_OL_OHMS: Ohms = 0.55 / 0.024;
/// SN74LVC1G14 `t_pd` max, A to Y, `V_CC` = 3.3 V ± 0.3 V, `C_L` = 15 pF,
/// −40 °C to 85 °C: 4.6 ns (§5.6); 4.6 rounds to 5 on the engine's
/// integer-nanosecond wheel.
pub const LVC1G14_T_PD_NS: u64 = 5;
/// SN74LVC1G14 `V_CC` min, recommended operating conditions: 1.65 V (§5.3).
pub const LVC1G14_SUPPLY_MIN_VOLTS: Volts = 1.65;
/// SN74LVC1G14 typical-value supply: 3.3 V (§5.5 note (1)).
pub const LVC1G14_NOMINAL_SUPPLY_VOLTS: Volts = 3.3;

// ============================================================
// Configuration
// ============================================================

/// Gate configuration: the electrical numbers, one set per part.
#[derive(Debug, Clone, PartialEq)]
pub struct Config {
    /// The part, for diagnostics.
    pub part: &'static str,
    /// An input at or above this voltage is high (`V_IH` / `V_T+`).
    pub high_at_volts: Volts,
    /// An input at or below this voltage is low (`V_IL` / `V_T−`); between
    /// the two the input holds its last level.
    pub low_at_volts: Volts,
    /// The datasheet's input hysteresis `ΔV_T`, where it names one apart
    /// from the two thresholds (0 where it does not) — declared on the
    /// input pins' thresholds ([`embsim_board::Thresholds::hysteresis`]).
    pub hysteresis_volts: Volts,
    /// What an input reads strictly between the two thresholds once the
    /// hysteresis has moved them ([`embsim_board::Thresholds::dead_band`]):
    /// a Schmitt input keeps its state; a plain CMOS input reads no level.
    pub dead_band: DeadBand,
    /// Output impedance driving high.
    pub r_oh_ohms: Ohms,
    /// Output impedance driving low.
    pub r_ol_ohms: Ohms,
    /// Propagation delay, input edge to output drive.
    pub t_pd_ns: u64,
    /// Supply at or above which the part operates.
    pub supply_min_volts: Volts,
    /// `Y = A̅` when true; a buffer when false.
    pub inverting: bool,
}

impl Config {
    /// The NXP 74LVC2G04 dual inverter.
    pub const fn lvc2g04() -> Self {
        Self {
            part: "74LVC2G04",
            high_at_volts: LVC2G04_HIGH_AT_VOLTS,
            low_at_volts: LVC2G04_LOW_AT_VOLTS,
            hysteresis_volts: LVC2G04_HYSTERESIS_VOLTS,
            // Table 7 gives `V_IL` max and `V_IH` min and nothing between:
            // neither level is guaranteed there.
            dead_band: DeadBand::Unknown,
            r_oh_ohms: LVC2G04_R_OH_OHMS,
            r_ol_ohms: LVC2G04_R_OL_OHMS,
            t_pd_ns: LVC2G04_T_PD_NS,
            supply_min_volts: LVC2G04_SUPPLY_MIN_VOLTS,
            inverting: true,
        }
    }

    /// The TI SN74LVC1G14 single Schmitt-trigger inverter.
    pub const fn lvc1g14() -> Self {
        Self {
            part: "SN74LVC1G14",
            high_at_volts: LVC1G14_HIGH_AT_VOLTS,
            low_at_volts: LVC1G14_LOW_AT_VOLTS,
            hysteresis_volts: LVC1G14_HYSTERESIS_VOLTS,
            // A Schmitt trigger: between `V_T−` and `V_T+` it keeps the
            // state it is in (§5.5, the hysteresis `ΔV_T`).
            dead_band: DeadBand::HoldLast,
            r_oh_ohms: LVC1G14_R_OH_OHMS,
            r_ol_ohms: LVC1G14_R_OL_OHMS,
            t_pd_ns: LVC1G14_T_PD_NS,
            supply_min_volts: LVC1G14_SUPPLY_MIN_VOLTS,
            inverting: true,
        }
    }

    /// The inputs' thresholds, **absolute** against the ground pin, as both
    /// datasheets give them over the supply range: `V_IL`/`V_T−`,
    /// `V_IH`/`V_T+`, the hysteresis and the dead-band policy.
    pub fn input_thresholds(&self) -> Thresholds {
        Thresholds::new(
            self.low_at_volts,
            self.high_at_volts,
            self.hysteresis_volts,
            self.dead_band,
        )
    }

    fn validate(&self) -> Result<(), PartConfigError> {
        require_positive("high_at_volts", self.high_at_volts)?;
        require_positive("low_at_volts", self.low_at_volts)?;
        require_positive("r_oh_ohms", self.r_oh_ohms)?;
        require_positive("r_ol_ohms", self.r_ol_ohms)?;
        require_positive("supply_min_volts", self.supply_min_volts)?;
        if self.low_at_volts >= self.high_at_volts {
            return Err(PartConfigError::InvertedThresholds {
                vil_ratio: self.low_at_volts,
                vih_ratio: self.high_at_volts,
            });
        }
        Ok(())
    }
}

// ============================================================
// Pin tables
// ============================================================

/// What a pin of the package is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GateRole {
    /// The supply.
    Vcc,
    /// Ground.
    Gnd,
    /// No internal connection.
    NoConnect,
    /// Channel `n`'s input.
    Input(usize),
    /// Channel `n`'s output.
    Output(usize),
}

/// One pin of a gate package: its netlist identifier, an alias, its role.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GatePin {
    /// The identifier the netlist uses (a number, or a function on a
    /// transcribed netlist).
    pub number: &'static str,
    /// An alias, when the identifier is not already the function.
    pub name: Option<&'static str>,
    /// The pin's role.
    pub role: GateRole,
}

const fn gate_pin(number: &'static str, name: Option<&'static str>, role: GateRole) -> GatePin {
    GatePin { number, name, role }
}

/// 74LVC2G04 keyed by **function**, as the P2-EC32MB netlist names
/// `U101`/`U601`'s pins.
pub const LVC2G04_PINS_BY_FUNCTION: [GatePin; 6] = [
    gate_pin("GND", None, GateRole::Gnd),
    gate_pin("1A", None, GateRole::Input(0)),
    gate_pin("1Y", None, GateRole::Output(0)),
    gate_pin("2A", None, GateRole::Input(1)),
    gate_pin("2Y", None, GateRole::Output(1)),
    gate_pin("VCC", None, GateRole::Vcc),
];

/// 74LVC2G04 in the GW (SOT363-2 / TSSOP6) package, by pin number
/// (Table 3): 1 `1A`, 2 `GND`, 3 `2A`, 4 `2Y`, 5 `V_CC`, 6 `1Y`.
pub const LVC2G04_PINS_SOT363: [GatePin; 6] = [
    gate_pin("1", Some("1A"), GateRole::Input(0)),
    gate_pin("2", Some("GND"), GateRole::Gnd),
    gate_pin("3", Some("2A"), GateRole::Input(1)),
    gate_pin("4", Some("2Y"), GateRole::Output(1)),
    gate_pin("5", Some("VCC"), GateRole::Vcc),
    gate_pin("6", Some("1Y"), GateRole::Output(0)),
];

/// SN74LVC1G14 in the DBV (SOT-23-5) package, by pin number (Table 4-1):
/// 1 `N.C.`, 2 `A`, 3 `GND`, 4 `Y`, 5 `V_CC`.
pub const LVC1G14_PINS_SOT23: [GatePin; 5] = [
    gate_pin("1", Some("NC"), GateRole::NoConnect),
    gate_pin("2", Some("A"), GateRole::Input(0)),
    gate_pin("3", Some("GND"), GateRole::Gnd),
    gate_pin("4", Some("Y"), GateRole::Output(0)),
    gate_pin("5", Some("VCC"), GateRole::Vcc),
];

/// Turn one pin-table row into a [`PinDecl`]. Inputs are senses reading
/// through the part's own thresholds — absolute, as both datasheets give
/// them over their supply range — against the ground pin; the supply is
/// measured against the ground pin; outputs rest released until the gate
/// drives them, `t_pd` after their first input.
fn declare(pin: &GatePin, table: &[GatePin], config: &Config) -> PinDecl {
    let gnd = table
        .iter()
        .find(|p| p.role == GateRole::Gnd)
        .map(|p| p.number);
    let referenced = |decl: PinDecl| match gnd {
        Some(gnd) => decl.with_reference(gnd),
        None => decl,
    };
    let decl = match pin.role {
        GateRole::Vcc => referenced(PinDecl::power_in(pin.number)),
        GateRole::Gnd => PinDecl::power_in(pin.number),
        GateRole::NoConnect => PinDecl::passive(pin.number),
        GateRole::Input(_) => {
            referenced(PinDecl::digital_in(pin.number, config.input_thresholds()))
        }
        GateRole::Output(_) => PinDecl::digital_out(pin.number).with_idle(None),
    };
    match pin.name {
        Some(name) => decl.with_name(name),
        None => decl,
    }
}

// ============================================================
// Core
// ============================================================

/// What a channel is doing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    /// The output follows the input's level through the thresholds, `t_pd`
    /// later.
    Level,
    /// The input carries a rate: the output is a periodic drive between
    /// the part's own ports, relaying the input's segment.
    Rate,
}

#[derive(Debug)]
struct ChannelState {
    input_pin: &'static str,
    output_pin: &'static str,
    input: Sense,
    /// The input's last projected level — the hysteresis memory.
    last_level: Option<Level>,
    /// A resistor joins the input's node to the output's (found at attach,
    /// from the build topology): the input is biased at the stage's own
    /// switching point, and any running segment it carries is relayed.
    self_biased: bool,
    mode: Mode,
    /// The last drive asked for, whether applied yet or still pending.
    /// Starts as the declaration's released idle, so a channel whose input
    /// has no level asks for nothing at attach.
    requested: Option<Option<Drive>>,
    /// The drive the output pin currently presents; starts as the
    /// declaration's released idle ([`PinDecl::idle`] `None`).
    applied: Option<Option<Drive>>,
    /// Drives scheduled for a future instant, in order.
    pending: Vec<(u64, Option<Drive>)>,
    output: Option<PinHandle>,
    /// Drives issued, level and periodic alike.
    drives: u64,
    /// Periodic drives issued: one per relayed rate change.
    trains: u64,
}

#[derive(Debug)]
struct State {
    vcc: Sense,
    channels: Vec<ChannelState>,
    io: Option<ComponentNetIo>,
}

#[derive(Debug)]
struct Core {
    config: Config,
    state: Mutex<State>,
}

/// A pin nothing has been handed yet: no voltage, no clock.
const NOTHING: Sense = Sense {
    volts: None,
    periodic: None,
    at_ns: 0,
};

impl Core {
    /// `V_CC` against `GND` when the part is powered: at or above its
    /// minimum.
    fn rail(&self, state: &State) -> Option<Volts> {
        supply_volts(&state.vcc, self.config.supply_min_volts)
    }

    /// The output's high port: `V_CC` behind `R_OH`.
    fn high_port(&self, rail: Volts) -> TheveninDrive {
        TheveninDrive {
            volts: rail,
            impedance: self.config.r_oh_ohms,
        }
    }

    /// The output's low port: 0 V behind `R_OL`.
    fn low_port(&self) -> TheveninDrive {
        TheveninDrive {
            volts: 0.0,
            impedance: self.config.r_ol_ohms,
        }
    }

    /// The drive a level-mode channel wants for its current input.
    fn level_drive(&self, state: &State, index: usize) -> Option<Drive> {
        let rail = self.rail(state)?;
        let level = state.channels[index].last_level?;
        let out = if self.config.inverting {
            match level {
                Level::High => Level::Low,
                Level::Low => Level::High,
            }
        } else {
            level
        };
        Some(Drive::Thevenin(match out {
            Level::High => self.high_port(rail),
            Level::Low => self.low_port(),
        }))
    }

    /// The drive a rate-mode channel presents: the part's own two ports
    /// around the input's segment, relayed verbatim.
    fn rate_drive(&self, state: &State, segment: PeriodicSchedule) -> Option<Drive> {
        let rail = self.rail(state)?;
        Some(Drive::Periodic {
            hi: self.high_port(rail),
            lo: self.low_port(),
            segment,
        })
    }

    /// Present `drive` on the output now, on change only.
    fn apply(&self, state: &mut State, index: usize, drive: Option<Drive>) {
        let channel = &mut state.channels[index];
        channel.requested = Some(drive);
        if channel.applied == Some(drive) {
            return;
        }
        Self::present(channel, drive);
    }

    /// Put `drive` on a channel's output pin, counting it.
    fn present(channel: &mut ChannelState, drive: Option<Drive>) {
        channel.applied = Some(drive);
        channel.drives += 1;
        if matches!(drive, Some(Drive::Periodic { .. })) {
            channel.trains += 1;
        }
        if let Some(pin) = &channel.output {
            match drive {
                Some(drive) => pin.drive(drive),
                None => pin.release(),
            }
        }
    }

    /// Ask for `drive` on the output `t_pd` from now — the propagation
    /// delay as a scheduled instant. A request identical to the last one
    /// schedules nothing.
    fn schedule(&self, state: &mut State, index: usize, drive: Option<Drive>) {
        if state.channels[index].requested == Some(drive) {
            return;
        }
        state.channels[index].requested = Some(drive);
        let deadline = virtual_clock::virtual_ns().saturating_add(self.config.t_pd_ns);
        state.channels[index].pending.push((deadline, drive));
        if let Some(io) = &state.io {
            io.schedule_at_ns(deadline);
        }
    }

    /// Re-evaluate a level-mode channel and schedule its output.
    fn refresh_level(&self, state: &mut State, index: usize) {
        let channel = &state.channels[index];
        let last = channel
            .input
            .level(&self.config.input_thresholds(), channel.last_level);
        state.channels[index].last_level = last;
        let drive = self.level_drive(state, index);
        self.schedule(state, index, drive);
    }

    /// The rate channel `channel`'s input carries, if it carries one: the
    /// running segment of a square wave whose two phases settle to two
    /// levels through the input's own thresholds
    /// ([`embsim_board::PeriodicSense::rate`]) — or, on a self-biased input,
    /// any running segment (the module docs). A held segment is no rate.
    fn rate_of(&self, channel: &ChannelState) -> Option<PeriodicSchedule> {
        let clock = channel.input.periodic?;
        if channel.self_biased {
            return (clock.segment.freq_hz > 0).then_some(clock.segment);
        }
        clock.rate(&self.config.input_thresholds())
    }

    /// Re-evaluate one channel from its input: rate mode — at once, the
    /// relay applied in the same pass — while the input carries a rate,
    /// level mode (through `t_pd`) otherwise.
    fn refresh(&self, state: &mut State, index: usize) {
        match self.rate_of(&state.channels[index]) {
            Some(segment) => {
                state.channels[index].mode = Mode::Rate;
                state.channels[index].pending.clear();
                let drive = self.rate_drive(state, segment);
                self.apply(state, index, drive);
            }
            None => {
                state.channels[index].mode = Mode::Level;
                self.refresh_level(state, index);
            }
        }
    }

    /// The input net changed.
    fn on_input(&self, state: &mut State, index: usize, sensed: Sense) {
        state.channels[index].input = sensed;
        self.refresh(state, index);
    }

    /// The supply net changed: every channel re-evaluates.
    fn on_supply(&self, state: &mut State, sensed: Sense) {
        state.vcc = sensed;
        for index in 0..state.channels.len() {
            self.refresh(state, index);
        }
    }

    /// A wake: apply every drive whose instant has come, latest last.
    fn on_wake(&self, state: &mut State, now_ns: u64) {
        for index in 0..state.channels.len() {
            let due: Vec<Option<Drive>> = {
                let channel = &mut state.channels[index];
                let split = channel.pending.partition_point(|(at, _)| *at <= now_ns);
                channel.pending.drain(..split).map(|(_, d)| d).collect()
            };
            if let Some(&drive) = due.last() {
                let channel = &mut state.channels[index];
                if channel.applied != Some(drive) {
                    Self::present(channel, drive);
                }
            }
        }
    }
}

// ============================================================
// Monitor
// ============================================================

/// Cheap cloneable read handle onto a live [`LogicGate`].
#[derive(Clone, Debug)]
pub struct LogicGateMonitor {
    core: Arc<Core>,
}

impl LogicGateMonitor {
    /// What channel `index` is doing.
    pub fn mode(&self, index: usize) -> Mode {
        self.core.state.lock().unwrap().channels[index].mode
    }

    /// Whether channel `index`'s input is self-biased — a resistor from its
    /// output node back to its input node, found at attach — and so relays
    /// every running segment its input carries (the module docs).
    pub fn self_biased(&self, index: usize) -> bool {
        self.core.state.lock().unwrap().channels[index].self_biased
    }

    /// The drive channel `index`'s output presents, or `None` when released.
    pub fn output(&self, index: usize) -> Option<Drive> {
        self.core.state.lock().unwrap().channels[index]
            .applied
            .flatten()
    }

    /// The level drive channel `index`'s output presents, or `None` when it
    /// is released or relaying a rate ([`Self::relayed_segment`]).
    pub fn output_drive(&self, index: usize) -> Option<TheveninDrive> {
        match self.output(index) {
            Some(Drive::Thevenin(drive)) => Some(drive),
            _ => None,
        }
    }

    /// Drives channel `index` has issued, level and periodic alike — the
    /// event-cost meter: one per output change, none for a re-evaluation
    /// that changed nothing.
    pub fn drive_count(&self, index: usize) -> u64 {
        self.core.state.lock().unwrap().channels[index].drives
    }

    /// Periodic drives channel `index` has issued: one per relayed rate
    /// change.
    pub fn train_count(&self, index: usize) -> u64 {
        self.core.state.lock().unwrap().channels[index].trains
    }

    /// The segment channel `index`'s output is relaying right now, or
    /// `None` while it presents a level or is released.
    pub fn relayed_segment(&self, index: usize) -> Option<PeriodicSchedule> {
        match self.output(index) {
            Some(Drive::Periodic { segment, .. }) => Some(segment),
            _ => None,
        }
    }

    /// The configuration in force.
    pub fn config(&self) -> &Config {
        &self.core.config
    }
}

// ============================================================
// Component
// ============================================================

/// A CMOS inverter / buffer package as a live board-engine component.
///
/// ```rust
/// use embsim_board::Component;
/// use embsim_models::logic_gate::{Config, LogicGate, LVC2G04_PINS_BY_FUNCTION};
///
/// // The P2-EC32MB's U101, its pins as the vendor netlist names them.
/// let gate = LogicGate::new(Config::lvc2g04(), &LVC2G04_PINS_BY_FUNCTION).expect("valid");
/// assert_eq!(gate.pins().len(), 6);
/// ```
#[derive(Debug)]
pub struct LogicGate {
    pins: Vec<PinDecl>,
    core: Arc<Core>,
}

impl LogicGate {
    /// A gate from a validated configuration and a pin table.
    pub fn new(config: Config, pins: &'static [GatePin]) -> Result<Self, PartConfigError> {
        config.validate()?;
        let mut inputs: Vec<(usize, &'static str)> = Vec::new();
        let mut outputs: Vec<(usize, &'static str)> = Vec::new();
        for pin in pins {
            match pin.role {
                GateRole::Input(i) => inputs.push((i, pin.number)),
                GateRole::Output(i) => outputs.push((i, pin.number)),
                _ => {}
            }
        }
        let count = inputs.len().min(outputs.len());
        let channels = (0..count)
            .map(|index| {
                let input_pin = inputs
                    .iter()
                    .find(|(i, _)| *i == index)
                    .map(|(_, n)| *n)
                    .expect("every channel has an input");
                let output_pin = outputs
                    .iter()
                    .find(|(i, _)| *i == index)
                    .map(|(_, n)| *n)
                    .expect("every channel has an output");
                ChannelState {
                    input_pin,
                    output_pin,
                    input: NOTHING,
                    last_level: None,
                    self_biased: false,
                    mode: Mode::Level,
                    requested: Some(None),
                    applied: Some(None),
                    pending: Vec::new(),
                    output: None,
                    drives: 0,
                    trains: 0,
                }
            })
            .collect();
        Ok(Self {
            pins: pins.iter().map(|pin| declare(pin, pins, &config)).collect(),
            core: Arc::new(Core {
                config,
                state: Mutex::new(State {
                    vcc: NOTHING,
                    channels,
                    io: None,
                }),
            }),
        })
    }

    /// A read handle onto this gate.
    pub fn monitor(&self) -> LogicGateMonitor {
        LogicGateMonitor {
            core: Arc::clone(&self.core),
        }
    }

    /// The validated configuration in force.
    pub fn config(&self) -> &Config {
        &self.core.config
    }
}

impl Component for LogicGate {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let (vcc_pin, channels): (
            Option<&'static str>,
            Vec<(usize, &'static str, &'static str)>,
        ) = {
            let mut state = self.core.state.lock().unwrap();
            state.io = Some(io.clone());
            for channel in &mut state.channels {
                channel.output = Some(io.pin(channel.output_pin)?);
                // A build-time topology query, read once: a resistor from
                // this channel's output node back to its input node biases
                // the input at the stage's own switching point.
                let output = io.node(channel.output_pin)?;
                channel.self_biased = io
                    .resistors_at(channel.input_pin)?
                    .iter()
                    .any(|resistor| resistor.far == output);
            }
            (
                self.pins
                    .iter()
                    .find(|p| p.role == PinRole::PowerIn && p.answers_to("VCC"))
                    .map(|p| p.number),
                state
                    .channels
                    .iter()
                    .enumerate()
                    .map(|(i, c)| (i, c.input_pin, c.output_pin))
                    .collect(),
            )
        };

        let core = Arc::clone(&self.core);
        io.on_wake_ns(move |now_ns| {
            let mut state = core.state.lock().unwrap();
            core.on_wake(&mut state, now_ns);
        });

        // The supply first, so an input delivered before the rail is known
        // cannot drive from a part that is not powered.
        let vcc_pin = vcc_pin.ok_or_else(|| AttachError::Failed {
            message: "the gate's pin table declares no VCC".to_string(),
        })?;
        let core = Arc::clone(&self.core);
        io.on_sense(vcc_pin, move |sensed| {
            let mut state = core.state.lock().unwrap();
            core.on_supply(&mut state, sensed);
        })?;

        for (index, input, _) in channels {
            let core = Arc::clone(&self.core);
            io.on_sense(input, move |sensed| {
                let mut state = core.state.lock().unwrap();
                core.on_input(&mut state, index, sensed);
            })?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use rstest::rstest;

    use embsim_board::PeriodicSense;

    use super::*;

    /// A sense handed `volts`.
    fn at(volts: Volts) -> Sense {
        Sense {
            volts: Some(volts),
            periodic: None,
            at_ns: 0,
        }
    }

    const V3V3: Sense = Sense {
        volts: Some(3.3),
        periodic: None,
        at_ns: 0,
    };

    fn gate(config: Config, pins: &'static [GatePin]) -> (LogicGate, Arc<Core>) {
        let gate = LogicGate::new(config, pins).expect("valid");
        let core = Arc::clone(&gate.core);
        (gate, core)
    }

    #[rstest]
    fn the_datasheet_numbers_are_the_ones_cited() {
        let lvc2g04 = Config::lvc2g04();
        assert!((lvc2g04.r_oh_ohms - 29.1667).abs() < 1e-3, "0.7 V / 24 mA");
        assert!((lvc2g04.r_ol_ohms - 22.9167).abs() < 1e-3, "0.55 V / 24 mA");
        assert_eq!((lvc2g04.low_at_volts, lvc2g04.high_at_volts), (0.8, 2.0));
        assert_eq!(
            lvc2g04.t_pd_ns, 5,
            "4.1 ns, the first integer instant at or after it"
        );
        let lvc1g14 = Config::lvc1g14();
        assert_eq!((lvc1g14.low_at_volts, lvc1g14.high_at_volts), (0.84, 1.87));
        assert_eq!(lvc1g14.t_pd_ns, 5);
    }

    /// The Schmitt projection: a node voltage inside the band holds the
    /// last level in both directions; outside it flips; no voltage is no
    /// level. A fought input is handed the voltage the fight settled at
    /// (two 25 Ω drivers at 1.65 V) and holds through it.
    #[rstest]
    #[case::rising_inside_band_holds_low(Some(1.5), Some(Level::Low), Some(Level::Low))]
    #[case::rising_above_t_plus(Some(1.9), Some(Level::Low), Some(Level::High))]
    #[case::falling_inside_band_holds_high(Some(1.0), Some(Level::High), Some(Level::High))]
    #[case::falling_below_t_minus(Some(0.8), Some(Level::High), Some(Level::Low))]
    #[case::inside_band_with_no_memory(Some(1.5), None, None)]
    #[case::floating(None, Some(Level::High), None)]
    #[case::fought_holds(Some(1.65), Some(Level::Low), Some(Level::Low))]
    #[case::driven_low(Some(0.0), Some(Level::High), Some(Level::Low))]
    fn a_schmitt_input_holds_inside_its_band(
        #[case] volts: Option<Volts>,
        #[case] last: Option<Level>,
        #[case] expect: Option<Level>,
    ) {
        let sensed = Sense {
            volts,
            periodic: None,
            at_ns: 0,
        };
        assert_eq!(
            sensed.level(&Config::lvc1g14().input_thresholds(), last),
            expect
        );
    }

    /// The plain CMOS input reads nothing inside its band, whatever it read
    /// last: Table 7 guarantees neither level there.
    #[rstest]
    #[case::from_high(Some(Level::High))]
    #[case::from_low(Some(Level::Low))]
    fn a_plain_input_reads_no_level_inside_its_band(#[case] last: Option<Level>) {
        assert_eq!(
            at(1.4).level(&Config::lvc2g04().input_thresholds(), last),
            None
        );
    }

    /// An input transition requests the inverted level `t_pd` later; the
    /// wake at that instant applies it; an earlier wake applies nothing.
    #[rstest]
    fn the_output_changes_t_pd_after_the_input() {
        let (_gate, core) = gate(Config::lvc1g14(), &LVC1G14_PINS_SOT23);
        let mut state = core.state.lock().unwrap();
        core.on_supply(&mut state, V3V3);
        core.on_input(&mut state, 0, at(3.3));
        let (deadline, drive) = state.channels[0].pending[0];
        assert_eq!(
            drive,
            Some(Drive::Thevenin(TheveninDrive {
                volts: 0.0,
                impedance: LVC1G14_R_OL_OHMS
            }))
        );
        core.on_wake(&mut state, deadline - 1);
        assert_eq!(state.channels[0].applied, Some(None), "nothing before t_pd");
        core.on_wake(&mut state, deadline);
        assert_eq!(state.channels[0].applied, Some(drive));
        assert_eq!(state.channels[0].drives, 1);

        // The same level again asks for nothing new.
        core.on_input(&mut state, 0, at(3.1));
        assert!(state.channels[0].pending.is_empty());
    }

    /// The segment of a square wave at `freq_hz`.
    fn segment_at(freq_hz: u32) -> PeriodicSchedule {
        PeriodicSchedule {
            emitted: 0,
            freq_hz,
            total: None,
            since_ns: 1_000_000,
        }
    }

    /// A square wave with a running segment, swinging `hi`/`lo` volts.
    fn swinging(freq_hz: u32, hi: Volts, lo: Volts) -> Sense {
        Sense {
            volts: None,
            periodic: Some(PeriodicSense {
                hi: Some(hi),
                lo: Some(lo),
                segment: segment_at(freq_hz),
            }),
            at_ns: 0,
        }
    }

    /// A rail-to-rail square wave: 3.3 V and 0 V, across both of the
    /// 74LVC2G04's input thresholds (0.8 V / 2.0 V, Table 7).
    fn clock(freq_hz: u32) -> Sense {
        swinging(freq_hz, 3.3, 0.0)
    }

    /// The TCXO's clipped sine as a coupled input is handed it: 0.8 V of
    /// swing above 0 V — at or below `V_IL` in both phases.
    fn tcxo(freq_hz: u32) -> Sense {
        swinging(freq_hz, 0.8, 0.0)
    }

    /// A rate on the input puts the channel in rate mode at once: the
    /// output is the part's own two ports around the input's segment,
    /// relayed verbatim; the same segment again changes nothing.
    #[rstest]
    fn a_rate_on_the_input_is_relayed_between_the_parts_own_ports() {
        let (_gate, core) = gate(Config::lvc2g04(), &LVC2G04_PINS_BY_FUNCTION);
        let mut state = core.state.lock().unwrap();
        core.on_supply(&mut state, V3V3);
        core.on_input(&mut state, 1, clock(20_000_000));
        let segment = segment_at(20_000_000);
        assert_eq!(state.channels[1].mode, Mode::Rate);
        assert_eq!(
            state.channels[1].applied,
            Some(Some(Drive::Periodic {
                hi: TheveninDrive {
                    volts: 3.3,
                    impedance: LVC2G04_R_OH_OHMS
                },
                lo: TheveninDrive {
                    volts: 0.0,
                    impedance: LVC2G04_R_OL_OHMS
                },
                segment,
            }))
        );
        assert_eq!((state.channels[1].drives, state.channels[1].trains), (1, 1));
        assert!(state.channels[1].pending.is_empty(), "no t_pd for a rate");

        // The same segment at other levels that still cross both
        // thresholds changes nothing.
        core.on_input(&mut state, 1, swinging(20_000_000, 2.5, 0.5));
        assert_eq!(state.channels[1].drives, 1, "no sense→drive iteration");

        // A held segment ends rate mode: the input has no level, so the
        // output is released `t_pd` later.
        core.on_input(&mut state, 1, clock(0));
        assert_eq!(state.channels[1].mode, Mode::Level);
        assert_eq!(state.channels[1].pending.len(), 1);
    }

    /// A clock whose phases do not both cross the input's thresholds is no
    /// rate to a plain input: 0 V / 1.2 V puts the high phase inside the
    /// 74LVC2G04's band, where it reads no level, so the channel stays in
    /// level mode with no level and its output is released; the TCXO's
    /// 0.8 V swing sits at or below `V_IL` in both phases, a steady low,
    /// so the output is driven high `t_pd` later.
    #[rstest]
    #[case::high_phase_in_the_band(swinging(20_000_000, 1.2, 0.0), None)]
    #[case::both_phases_low(tcxo(20_000_000), Some(Level::Low))]
    fn a_clock_that_does_not_cross_a_plain_input_is_no_rate(
        #[case] input: Sense,
        #[case] level: Option<Level>,
    ) {
        let (_gate, core) = gate(Config::lvc2g04(), &LVC2G04_PINS_BY_FUNCTION);
        let mut state = core.state.lock().unwrap();
        core.on_supply(&mut state, V3V3);
        core.on_input(&mut state, 1, input);
        assert_eq!(state.channels[1].mode, Mode::Level);
        assert_eq!(state.channels[1].last_level, level);
        assert_eq!(state.channels[1].trains, 0, "nothing relayed");
        let expected = level.map(|_| {
            Drive::Thevenin(TheveninDrive {
                volts: 3.3,
                impedance: LVC2G04_R_OH_OHMS,
            })
        });
        match expected {
            Some(drive) => assert_eq!(state.channels[1].pending[0].1, Some(drive)),
            None => assert!(
                state.channels[1].pending.is_empty(),
                "released is what the output already is"
            ),
        }
    }

    /// A self-biased input relays the swing coupled onto it whatever its
    /// phases: the TCXO's 0.8 V is a rate there, and the same segment at
    /// the full swing its own output carries back through the feedback
    /// resistor changes nothing.
    #[rstest]
    fn a_self_biased_input_relays_the_swing_coupled_onto_it() {
        let (_gate, core) = gate(Config::lvc2g04(), &LVC2G04_PINS_BY_FUNCTION);
        let mut state = core.state.lock().unwrap();
        state.channels[1].self_biased = true;
        core.on_supply(&mut state, V3V3);
        core.on_input(&mut state, 1, tcxo(20_000_000));
        assert_eq!(state.channels[1].mode, Mode::Rate);
        assert!(matches!(
            state.channels[1].applied,
            Some(Some(Drive::Periodic { segment, .. })) if segment == segment_at(20_000_000)
        ));
        core.on_input(&mut state, 1, clock(20_000_000));
        assert_eq!(state.channels[1].drives, 1, "no sense→drive iteration");
        // A held segment is no rate there either.
        core.on_input(&mut state, 1, tcxo(0));
        assert_eq!(state.channels[1].mode, Mode::Level);
    }

    /// Unpowered, every output is released and no rate crosses.
    #[rstest]
    fn an_unpowered_gate_drives_nothing() {
        let (_gate, core) = gate(Config::lvc2g04(), &LVC2G04_PINS_SOT363);
        let mut state = core.state.lock().unwrap();
        core.on_input(&mut state, 0, at(0.0));
        assert!(
            state.channels[0].pending.is_empty(),
            "released is what the output already is"
        );
        core.on_input(&mut state, 0, clock(1_000));
        assert_eq!(state.channels[0].applied, Some(None));
        assert_eq!(state.channels[0].drives, 0);
    }

    #[rstest]
    fn the_pin_tables_declare_the_roles_the_engine_routes() {
        let gate = LogicGate::new(Config::lvc1g14(), &LVC1G14_PINS_SOT23).unwrap();
        let pin = |n: &str| *gate.pins().iter().find(|p| p.number == n).unwrap();
        assert_eq!(
            pin("2").senses_at_build(),
            Some(embsim_board::SenseKind::Digital)
        );
        assert_eq!(
            pin("2").thresholds,
            Some(Thresholds::new(
                LVC1G14_LOW_AT_VOLTS,
                LVC1G14_HIGH_AT_VOLTS,
                LVC1G14_HYSTERESIS_VOLTS,
                DeadBand::HoldLast
            )),
            "the Schmitt input declares the datasheet's V_T-, V_T+ and ΔV_T"
        );
        assert_eq!(pin("2").reference, Some("3"), "against GND");
        assert!(pin("4").drives());
        assert_eq!(pin("4").idle, None);
        assert_eq!(pin("1").role, embsim_board::PinRole::Passive);
        assert_eq!(pin("5").role, embsim_board::PinRole::PowerIn);
        assert_eq!(pin("5").reference, Some("3"));
    }
}
