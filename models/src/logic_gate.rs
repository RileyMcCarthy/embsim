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
//! channel whose input carries a *rate* (a [`StreamRole::PulseSink`] that a
//! step clock or an oscillator reaches) is in **rate mode**: it re-publishes
//! the rate on its output ([`StreamRole::PulseSource`]) and drives the output
//! at the rate's time-average through the output impedance, the level sense
//! ignored — which is what lets a self-biased stage (an inverter with a
//! resistor from its output back to its input, AC-coupled to an oscillator)
//! rest at its mid-rail fixed point in one pass, with no sense→drive
//! iteration. See [`Mode`].
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
//! - **A floating or fought-over input has no level and the output is
//!   released.** TI's §5.3 note (1) says unused inputs must be held at
//!   `V_CC` or GND; neither datasheet says what an open input produces, and
//!   the engine invents no level (`DESIGN.md` rule 6). A node voltage inside
//!   the threshold band holds the input's last level — the hysteresis.
//! - **Rate mode** takes 50 % as the rate's duty (a [`PulseTrain`] carries
//!   no duty) and drives the average, `V_CC / 2`, through the larger of the
//!   two output impedances. The datasheet has no figure for the output
//!   resting mid-rail; the larger bound is the conservative one.
//! - **Output impedance is the worst case at 24 mA**, one number per
//!   level; the `V_OH`/`I_OH` curves (TI Figure 5-1/5-2) are not modelled.
//! - **`V_CC` is the supply net's own node voltage**, or the nominal 3.3 V
//!   when the engine has only a digital projection of it — the same
//!   simplification the isolators make. A supply below the minimum releases
//!   every output; the partial-power-down `I_off` behaviour is not modelled
//!   beyond that.
//! - **Input rise/fall-rate limits, input capacitance and `C_pd`** are not
//!   modelled.

use std::sync::{Arc, Mutex};

use embsim_board::{
    AttachError, Component, ComponentNetIo, IdleDrive, Level, NetState, Ohms, PinDecl, PinHandle,
    PinKind, PulseTrain, PulseTx, StreamRole, TheveninDrive, Volts,
};
use embsim_core::virtual_clock;

use crate::isolation::{rail_volts, require_positive, supply_up, PartConfigError};

// ============================================================
// Datasheet constants
// ============================================================

/// 74LVC2G04 `V_IH` min at `V_CC` = 2.7 V to 3.6 V: 2.0 V (Table 7).
pub const LVC2G04_HIGH_AT_VOLTS: Volts = 2.0;
/// 74LVC2G04 `V_IL` max at `V_CC` = 2.7 V to 3.6 V: 0.8 V (Table 7).
pub const LVC2G04_LOW_AT_VOLTS: Volts = 0.8;
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

/// The duty a rate-mode channel assumes for the rate's time-average: a
/// [`PulseTrain`] carries no duty, and 50 % is the mid-point of the
/// oscillator's symmetry specification (see [`crate::oscillator`]).
pub const RATE_MODE_DUTY: f64 = 0.5;

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
    /// Output impedance driving high.
    pub r_oh_ohms: Ohms,
    /// Output impedance driving low.
    pub r_ol_ohms: Ohms,
    /// Propagation delay, input edge to output drive.
    pub t_pd_ns: u64,
    /// Supply at or above which the part operates.
    pub supply_min_volts: Volts,
    /// Supply voltage assumed while the supply net has no numeric solve.
    pub nominal_supply_volts: Volts,
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
            r_oh_ohms: LVC2G04_R_OH_OHMS,
            r_ol_ohms: LVC2G04_R_OL_OHMS,
            t_pd_ns: LVC2G04_T_PD_NS,
            supply_min_volts: LVC2G04_SUPPLY_MIN_VOLTS,
            nominal_supply_volts: LVC2G04_NOMINAL_SUPPLY_VOLTS,
            inverting: true,
        }
    }

    /// The TI SN74LVC1G14 single Schmitt-trigger inverter.
    pub const fn lvc1g14() -> Self {
        Self {
            part: "SN74LVC1G14",
            high_at_volts: LVC1G14_HIGH_AT_VOLTS,
            low_at_volts: LVC1G14_LOW_AT_VOLTS,
            r_oh_ohms: LVC1G14_R_OH_OHMS,
            r_ol_ohms: LVC1G14_R_OL_OHMS,
            t_pd_ns: LVC1G14_T_PD_NS,
            supply_min_volts: LVC1G14_SUPPLY_MIN_VOLTS,
            nominal_supply_volts: LVC1G14_NOMINAL_SUPPLY_VOLTS,
            inverting: true,
        }
    }

    /// The impedance a rate-mode channel drives the average through: the
    /// larger of the two output impedances.
    pub fn rate_mode_ohms(&self) -> Ohms {
        self.r_oh_ohms.max(self.r_ol_ohms)
    }

    fn validate(&self) -> Result<(), PartConfigError> {
        require_positive("high_at_volts", self.high_at_volts)?;
        require_positive("low_at_volts", self.low_at_volts)?;
        require_positive("r_oh_ohms", self.r_oh_ohms)?;
        require_positive("r_ol_ohms", self.r_ol_ohms)?;
        require_positive("supply_min_volts", self.supply_min_volts)?;
        require_positive("nominal_supply_volts", self.nominal_supply_volts)?;
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

/// Turn one pin-table row into a [`PinDecl`]. Inputs are senses that also
/// accept a routed rate; outputs are pulse sources that rest released until
/// the gate drives them, `t_pd` after their first input.
fn declare(pin: &GatePin) -> PinDecl {
    let (kind, stream, idle) = match pin.role {
        GateRole::Vcc | GateRole::Gnd => (PinKind::PowerIn, None, IdleDrive::KindDefault),
        GateRole::NoConnect => (PinKind::Passive, None, IdleDrive::KindDefault),
        GateRole::Input(_) => (
            PinKind::DigitalIn,
            Some(StreamRole::PulseSink),
            IdleDrive::KindDefault,
        ),
        GateRole::Output(_) => (
            PinKind::DigitalOut,
            Some(StreamRole::PulseSource),
            IdleDrive::Released,
        ),
    };
    PinDecl {
        number: pin.number,
        name: pin.name,
        kind,
        stream,
        drive_impedance: None,
        idle,
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
    /// The input carries a rate: the output re-publishes it and rests at
    /// its time-average.
    Rate,
}

#[derive(Debug)]
struct ChannelState {
    input_pin: &'static str,
    output_pin: &'static str,
    input: NetState,
    /// The input's last projected level — the hysteresis memory.
    last_level: Option<Level>,
    mode: Mode,
    /// The last drive asked for, whether applied yet or still pending.
    /// Starts as the declaration's released idle, so a channel whose input
    /// has no level asks for nothing at attach.
    requested: Option<Option<TheveninDrive>>,
    /// The drive the output pin currently presents; starts as the
    /// declaration's released idle ([`IdleDrive::Released`]).
    applied: Option<Option<TheveninDrive>>,
    /// Drives scheduled for a future instant, in order.
    pending: Vec<(u64, Option<TheveninDrive>)>,
    output: Option<PinHandle>,
    tx: Option<PulseTx>,
    published_train: Option<PulseTrain>,
    drives: u64,
    trains: u64,
}

#[derive(Debug)]
struct State {
    vcc: NetState,
    channels: Vec<ChannelState>,
    io: Option<ComponentNetIo>,
}

#[derive(Debug)]
struct Core {
    config: Config,
    state: Mutex<State>,
}

/// The level a sensed input projects to through the thresholds, holding
/// the last level inside the band; none for a node with no level.
fn project(state: NetState, last: Option<Level>, low_at: Volts, high_at: Volts) -> Option<Level> {
    match state {
        NetState::Driven(level) | NetState::Pulled(level, _) => Some(level),
        NetState::Analog(volts) if volts.is_nan() => None,
        NetState::Analog(volts) if volts >= high_at => Some(Level::High),
        NetState::Analog(volts) if volts <= low_at => Some(Level::Low),
        NetState::Analog(_) => last,
        NetState::Floating | NetState::Contention => None,
    }
}

impl Core {
    fn powered(&self, state: &State) -> bool {
        supply_up(state.vcc, self.config.supply_min_volts)
    }

    fn rail(&self, state: &State) -> Volts {
        rail_volts(state.vcc, self.config.nominal_supply_volts)
    }

    /// The drive a level-mode channel wants for its current input.
    fn level_drive(&self, state: &State, index: usize) -> Option<TheveninDrive> {
        if !self.powered(state) {
            return None;
        }
        let level = state.channels[index].last_level?;
        let out = if self.config.inverting {
            match level {
                Level::High => Level::Low,
                Level::Low => Level::High,
            }
        } else {
            level
        };
        Some(match out {
            Level::High => TheveninDrive {
                volts: self.rail(state),
                impedance: self.config.r_oh_ohms,
            },
            Level::Low => TheveninDrive {
                volts: 0.0,
                impedance: self.config.r_ol_ohms,
            },
        })
    }

    /// The drive a rate-mode channel rests at: the rate's average through
    /// the larger output impedance.
    fn rate_drive(&self, state: &State) -> Option<TheveninDrive> {
        if !self.powered(state) {
            return None;
        }
        Some(TheveninDrive {
            volts: self.rail(state) * RATE_MODE_DUTY,
            impedance: self.config.rate_mode_ohms(),
        })
    }

    /// Present `drive` on the output now, on change only.
    fn apply(&self, state: &mut State, index: usize, drive: Option<TheveninDrive>) {
        let channel = &mut state.channels[index];
        channel.requested = Some(drive);
        if channel.applied == Some(drive) {
            return;
        }
        channel.applied = Some(drive);
        channel.drives += 1;
        if let Some(pin) = &channel.output {
            pin.set_drive(drive);
        }
    }

    /// Ask for `drive` on the output `t_pd` from now — the propagation
    /// delay as a scheduled instant. A request identical to the last one
    /// schedules nothing.
    fn schedule(&self, state: &mut State, index: usize, drive: Option<TheveninDrive>) {
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
        let last = project(
            channel.input,
            channel.last_level,
            self.config.low_at_volts,
            self.config.high_at_volts,
        );
        state.channels[index].last_level = last;
        let drive = self.level_drive(state, index);
        self.schedule(state, index, drive);
    }

    /// Publish a train on the output, on change only.
    fn publish(&self, state: &mut State, index: usize, train: PulseTrain) {
        let channel = &mut state.channels[index];
        if channel.published_train == Some(train) {
            return;
        }
        channel.published_train = Some(train);
        channel.trains += 1;
        if let Some(tx) = &channel.tx {
            tx.set_train(train);
        }
    }

    /// A rate arrived on a channel's input.
    fn on_train(&self, state: &mut State, index: usize, train: PulseTrain) {
        if train.pulses.freq_hz > 0 {
            state.channels[index].mode = Mode::Rate;
            state.channels[index].pending.clear();
            let drive = self.rate_drive(state);
            self.apply(state, index, drive);
            let relayed = if self.powered(state) {
                train
            } else {
                PulseTrain::IDLE
            };
            self.publish(state, index, relayed);
        } else {
            state.channels[index].mode = Mode::Level;
            self.publish(state, index, PulseTrain::IDLE);
            self.refresh_level(state, index);
        }
    }

    /// The input net changed.
    fn on_input(&self, state: &mut State, index: usize, sensed: NetState) {
        state.channels[index].input = sensed;
        if state.channels[index].mode == Mode::Level {
            self.refresh_level(state, index);
        }
    }

    /// The supply net changed: every channel re-evaluates.
    fn on_supply(&self, state: &mut State, sensed: NetState) {
        state.vcc = sensed;
        for index in 0..state.channels.len() {
            match state.channels[index].mode {
                Mode::Rate => {
                    let drive = self.rate_drive(state);
                    self.apply(state, index, drive);
                    let train = if self.powered(state) {
                        state.channels[index]
                            .published_train
                            .filter(|t| t.pulses.freq_hz > 0)
                    } else {
                        None
                    };
                    if let Some(train) = train {
                        self.publish(state, index, train);
                    } else if !self.powered(state) {
                        self.publish(state, index, PulseTrain::IDLE);
                    }
                }
                Mode::Level => self.refresh_level(state, index),
            }
        }
    }

    /// A wake: apply every drive whose instant has come, latest last.
    fn on_wake(&self, state: &mut State, now_ns: u64) {
        for index in 0..state.channels.len() {
            let due: Vec<Option<TheveninDrive>> = {
                let channel = &mut state.channels[index];
                let split = channel.pending.partition_point(|(at, _)| *at <= now_ns);
                channel.pending.drain(..split).map(|(_, d)| d).collect()
            };
            if let Some(&drive) = due.last() {
                let channel = &mut state.channels[index];
                if channel.applied != Some(drive) {
                    channel.applied = Some(drive);
                    channel.drives += 1;
                    if let Some(pin) = &channel.output {
                        pin.set_drive(drive);
                    }
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

    /// The drive channel `index`'s output presents, or `None` when released.
    pub fn output_drive(&self, index: usize) -> Option<TheveninDrive> {
        self.core.state.lock().unwrap().channels[index]
            .applied
            .flatten()
    }

    /// `set_drive` calls channel `index` has issued — the event-cost meter:
    /// one per output change, none for a re-evaluation that changed nothing.
    pub fn drive_count(&self, index: usize) -> u64 {
        self.core.state.lock().unwrap().channels[index].drives
    }

    /// `set_train` calls channel `index` has issued.
    pub fn train_count(&self, index: usize) -> u64 {
        self.core.state.lock().unwrap().channels[index].trains
    }

    /// The train channel `index` last published on its output.
    pub fn relayed_train(&self, index: usize) -> Option<PulseTrain> {
        self.core.state.lock().unwrap().channels[index].published_train
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
                    input: NetState::Floating,
                    last_level: None,
                    mode: Mode::Level,
                    requested: Some(None),
                    applied: Some(None),
                    pending: Vec::new(),
                    output: None,
                    tx: None,
                    published_train: None,
                    drives: 0,
                    trains: 0,
                }
            })
            .collect();
        Ok(Self {
            pins: pins.iter().map(declare).collect(),
            core: Arc::new(Core {
                config,
                state: Mutex::new(State {
                    vcc: NetState::Floating,
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
                channel.tx = Some(io.pulse_tx(channel.output_pin)?);
            }
            (
                self.pins
                    .iter()
                    .find(|p| {
                        p.kind == PinKind::PowerIn && (p.number == "VCC" || p.name == Some("VCC"))
                    })
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
            let core = Arc::clone(&self.core);
            io.on_pulse(input, move |train| {
                let mut state = core.state.lock().unwrap();
                core.on_train(&mut state, index, train);
            })?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use embsim_board::{PulseDirection, PulseSegment};
    use rstest::rstest;

    use super::*;

    const V3V3: NetState = NetState::Analog(3.3);

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
        assert!((lvc1g14.rate_mode_ohms() - lvc1g14.r_oh_ohms).abs() < f64::EPSILON);
    }

    /// The Schmitt projection: a node voltage inside the band holds the
    /// last level in both directions; outside it flips; no level is none.
    #[rstest]
    #[case::rising_inside_band_holds_low(NetState::Analog(1.5), Some(Level::Low), Some(Level::Low))]
    #[case::rising_above_t_plus(NetState::Analog(1.9), Some(Level::Low), Some(Level::High))]
    #[case::falling_inside_band_holds_high(
        NetState::Analog(1.0),
        Some(Level::High),
        Some(Level::High)
    )]
    #[case::falling_below_t_minus(NetState::Analog(0.8), Some(Level::High), Some(Level::Low))]
    #[case::inside_band_with_no_memory(NetState::Analog(1.5), None, None)]
    #[case::floating(NetState::Floating, Some(Level::High), None)]
    #[case::contention(NetState::Contention, Some(Level::Low), None)]
    #[case::driven(NetState::Driven(Level::Low), Some(Level::High), Some(Level::Low))]
    fn a_schmitt_input_holds_inside_its_band(
        #[case] sensed: NetState,
        #[case] last: Option<Level>,
        #[case] expect: Option<Level>,
    ) {
        let c = Config::lvc1g14();
        assert_eq!(
            project(sensed, last, c.low_at_volts, c.high_at_volts),
            expect
        );
    }

    /// An input transition requests the inverted level `t_pd` later; the
    /// wake at that instant applies it; an earlier wake applies nothing.
    #[rstest]
    fn the_output_changes_t_pd_after_the_input() {
        let (_gate, core) = gate(Config::lvc1g14(), &LVC1G14_PINS_SOT23);
        let mut state = core.state.lock().unwrap();
        core.on_supply(&mut state, V3V3);
        core.on_input(&mut state, 0, NetState::Driven(Level::High));
        let (deadline, drive) = state.channels[0].pending[0];
        assert_eq!(
            drive,
            Some(TheveninDrive {
                volts: 0.0,
                impedance: LVC1G14_R_OL_OHMS
            })
        );
        core.on_wake(&mut state, deadline - 1);
        assert_eq!(state.channels[0].applied, Some(None), "nothing before t_pd");
        core.on_wake(&mut state, deadline);
        assert_eq!(state.channels[0].applied, Some(drive));
        assert_eq!(state.channels[0].drives, 1);

        // The same level again asks for nothing new.
        core.on_input(&mut state, 0, NetState::Pulled(Level::High, 1_000.0));
        assert!(state.channels[0].pending.is_empty());
    }

    /// A rate on the input puts the channel in rate mode at once: the
    /// output rests at the average through the larger impedance and the
    /// train is relayed verbatim; the level sense is then ignored.
    #[rstest]
    fn a_rate_on_the_input_is_relayed_and_the_output_rests_mid_rail() {
        let (_gate, core) = gate(Config::lvc2g04(), &LVC2G04_PINS_BY_FUNCTION);
        let mut state = core.state.lock().unwrap();
        core.on_supply(&mut state, V3V3);
        let train = PulseTrain {
            pulses: PulseSegment {
                emitted: 0,
                freq_hz: 20_000_000,
                total: None,
                since_us: 1_000,
            },
            direction: PulseDirection::Forward,
        };
        core.on_train(&mut state, 1, train);
        assert_eq!(state.channels[1].mode, Mode::Rate);
        assert_eq!(
            state.channels[1].applied,
            Some(Some(TheveninDrive {
                volts: 1.65,
                impedance: LVC2G04_R_OH_OHMS
            }))
        );
        assert_eq!(state.channels[1].published_train, Some(train));
        assert_eq!(state.channels[1].drives, 1);

        // The level on the input is now the average the gate itself made —
        // and it changes nothing.
        core.on_input(&mut state, 1, NetState::Analog(1.65));
        assert_eq!(state.channels[1].drives, 1, "no sense→drive iteration");
        assert!(state.channels[1].pending.is_empty());

        // A held train ends rate mode and the relay.
        core.on_train(&mut state, 1, PulseTrain::IDLE);
        assert_eq!(state.channels[1].mode, Mode::Level);
        assert_eq!(state.channels[1].published_train, Some(PulseTrain::IDLE));
    }

    /// Unpowered, every output is released and no train crosses.
    #[rstest]
    fn an_unpowered_gate_drives_nothing() {
        let (_gate, core) = gate(Config::lvc2g04(), &LVC2G04_PINS_SOT363);
        let mut state = core.state.lock().unwrap();
        core.on_input(&mut state, 0, NetState::Driven(Level::Low));
        assert!(
            state.channels[0].pending.is_empty(),
            "released is what the output already is"
        );
        let train = PulseTrain {
            pulses: PulseSegment {
                emitted: 0,
                freq_hz: 1_000,
                total: None,
                since_us: 0,
            },
            direction: PulseDirection::Forward,
        };
        core.on_train(&mut state, 0, train);
        assert_eq!(state.channels[0].applied, Some(None));
        assert_eq!(state.channels[0].drives, 0);
        assert_eq!(state.channels[0].published_train, Some(PulseTrain::IDLE));
    }

    #[rstest]
    fn the_pin_tables_declare_the_roles_the_engine_routes() {
        let gate = LogicGate::new(Config::lvc1g14(), &LVC1G14_PINS_SOT23).unwrap();
        let pin = |n: &str| *gate.pins().iter().find(|p| p.number == n).unwrap();
        assert_eq!(pin("2").stream, Some(StreamRole::PulseSink));
        assert_eq!(pin("4").stream, Some(StreamRole::PulseSource));
        assert_eq!(pin("4").idle, IdleDrive::Released);
        assert_eq!(pin("1").kind, PinKind::Passive);
        assert_eq!(pin("5").kind, PinKind::PowerIn);
    }
}
