//! Model: a **voltage detector** — the STMicroelectronics **STM1061**, the
//! `U404` brownout detector on a Parallax P2-EC32MB module, holding the
//! P2's `RESN` low while the 1.8 V core rail is under its threshold
//! (`NODES.md` §2, the Supervisor row).
//!
//! A comparator with hysteresis on an open-drain output: the output sinks
//! to `V_SS` through its on-resistance while the supply, read against the
//! part's own `V_SS` pin, is below the detect threshold `V_TH−`, and is
//! released once it rises past the release threshold `V_TH+ = V_TH− +
//! V_HYST`; between the two it holds its last state. Below the supply the
//! datasheet guarantees the output for, the output is **released**: the
//! part has no bias to sink with, and no level is invented for it
//! (`DESIGN.md` rule 6). The two propagation delays the datasheet names —
//! detect and release — are scheduled instants: one drive per crossing,
//! at the crossing plus the delay. There is no reset timeout: the STM1061
//! is a voltage detector, not a reset supervisor.
//!
//! # Datasheet provenance
//!
//! STMicroelectronics **STM1061 Low Power Voltage Detector**, the copy
//! reachable while this was written: Rev 2.0, October 2005 (ST's current
//! revision is Rev 4, July 2006; the figures cited are the electrical
//! characteristics both carry — re-check the page numbers against Rev 4
//! when it can be fetched).
//!
//! - **Pins** — Table 2 Pin Functions (p.6): SOT23-3 pin 1 `~OUT`
//!   ("Active-Low Open Drain Output"), pin 2 `V_SS`, pin 3 `V_CC`. The
//!   module's transcribed netlist names them `OUT`, `VSS`, `VCC`.
//! - **Operation** (p.6): "asserts an output signal (~OUT) whenever V_CC
//!   goes below the Voltage Detect Threshold (V_TH−). The output signal
//!   (~OUT) stays asserted until V_CC goes above the Voltage Detect
//!   Release (V_TH+). Output voltage (V_OUT) is guaranteed valid down to
//!   V_CC = 0.7 V at 25 °C." Features (p.1): "GUARANTEED ~OUT ASSERTION
//!   DOWN TO V_CC = 0.7V"; Table 5 (p.13) `V_CC` operating voltage 0.7–6.0 V.
//!   [`STM1061_V_CC_VALID_VOLTS`].
//! - **Thresholds** — Table 9 Factory-Trimmed Thresholds (p.18), suffix
//!   16: `V_TH−` 1.568 min / 1.600 typ / 1.632 max V (±2 %).
//!   [`STM1061N16_V_TH_MINUS_VOLTS`]. Table 5 (p.13): `V_TH+` = `V_TH−` +
//!   `V_HYST`; `V_HYST` 0.02·V_TH− min, 0.05·V_TH− typ, 0.08·V_TH− max.
//!   [`STM1061_V_HYST_RATIO`]. Figure 10 (p.8) labels the 1.6 V part's
//!   release voltage 1.68 V, which is the typical.
//! - **Output** — Table 5 (p.13) `I_OUT`, N-channel output current, reset
//!   asserted, `V_DS` = 0.5 V: 1.0 min / 1.7 typ mA at `V_CC` = 1.0 V,
//!   3.0 / 14 mA at 2.0 V, 5.0 / 22 mA at 3.0 V. The sink is modelled as
//!   the resistance the smallest cited condition gives, `V_DS / I_OUT` at
//!   `V_CC` = 1.0 V: the output is only ever asserted while the supply is
//!   below the threshold, where that is the row that applies.
//!   [`STM1061_R_OL_OHMS`].
//! - **Delays** — Table 5 (p.13): `t_PD`, V_CC to ~OUT detect delay, 25 µs
//!   typ (V_CC falling from V_TH− + 100 mV to V_TH− − 100 mV at 10 mV/µs);
//!   `t_PR`, V_CC to ~OUT release delay, 30 µs typ, 200 µs max (V_CC
//!   rising from V_TH+ − 100 mV to V_TH+ + 100 mV). [`STM1061_T_PD_NS`],
//!   [`STM1061_T_PR_NS`].
//! - **Part** — Table 8 Ordering Information (p.17): `STM1061N16WX6F` =
//!   open-drain active-low, 1.6 V threshold, SOT23-3, −40 to 85 °C,
//!   tape and reel — the module's `U404`.
//!
//! # Deliberate simplifications
//!
//! - **Thresholds are typicals**; the ±2 % trim tolerance and the hysteresis
//!   range are recorded, not applied.
//! - **The sink resistance is one number**, taken at the lowest cited
//!   supply; the output current's growth with `V_CC` (Figure 14, p.10) is
//!   not followed. Against the module's 10.5 kΩ pull-up any value in the
//!   table projects the same `Driven(Low)`.
//! - **Transient immunity** (Figure 11, p.9: glitches under the curve
//!   raise no reset) is not modelled — the engine has no glitch to offer;
//!   a step through the threshold is a crossing.
//! - **A step from below `V_CC(valid)` straight past `V_TH+`** — the
//!   module's core rail, a released terminal until its buck's soft-start
//!   elapses, then 1.813 V — never shows the asserted state: the output
//!   goes from undefined (released) to released, with no crossing to
//!   delay. The asserted window is observable only with the supply held
//!   between 0.7 V and the threshold.

use std::sync::{Arc, Mutex};

use embsim_board::{
    AttachError, Component, ComponentNetIo, Ohms, PinDecl, PinHandle, Sense, TheveninDrive, Volts,
};
use embsim_core::virtual_clock;

// ============================================================
// Datasheet constants
// ============================================================

/// STM1061N16 `V_TH−`, detect voltage, 1.600 V typ (Table 9, p.18; 1.568
/// min, 1.632 max).
pub const STM1061N16_V_TH_MINUS_VOLTS: Volts = 1.600;
/// STM1061 threshold trim tolerance, ±2 % (Table 5 / Table 9). Recorded.
pub const STM1061_V_TH_TOLERANCE: f64 = 0.02;
/// STM1061 `V_HYST` as a fraction of `V_TH−`: 0.05 typ (Table 5, p.13;
/// 0.02 min, 0.08 max).
pub const STM1061_V_HYST_RATIO: f64 = 0.05;
/// STM1061 minimum `V_CC` for a guaranteed output, 0.7 V (Features p.1;
/// Operation p.6; Table 5 p.13 operating voltage 0.7–6.0 V).
pub const STM1061_V_CC_VALID_VOLTS: Volts = 0.7;
/// STM1061 `I_OUT` at `V_CC` = 1.0 V, `V_DS` = 0.5 V, reset asserted:
/// 1.7 mA typ (Table 5, p.13).
pub const STM1061_I_OUT_AT_1V_AMPS: f64 = 1.7e-3;
/// The `V_DS` the output current is cited at, 0.5 V (Table 5, p.13).
pub const STM1061_I_OUT_V_DS_VOLTS: Volts = 0.5;
/// STM1061 output sink resistance while asserted, `V_DS / I_OUT` at
/// `V_CC` = 1.0 V: ≈ 294 Ω.
pub const STM1061_R_OL_OHMS: Ohms = STM1061_I_OUT_V_DS_VOLTS / STM1061_I_OUT_AT_1V_AMPS;
/// STM1061 `t_PD`, V_CC-to-~OUT detect delay, 25 µs typ (Table 5, p.13).
pub const STM1061_T_PD_NS: u64 = 25_000;
/// STM1061 `t_PR`, V_CC-to-~OUT release delay, 30 µs typ (Table 5, p.13;
/// 200 µs max).
pub const STM1061_T_PR_NS: u64 = 30_000;

// ============================================================
// Configuration
// ============================================================

/// Voltage detector configuration.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Config {
    /// The detect threshold `V_TH−`, against the part's `V_SS`.
    pub v_th_minus_volts: Volts,
    /// The hysteresis: the output releases at `V_TH− + V_HYST`.
    pub v_hyst_volts: Volts,
    /// The supply below which the output is undefined and released.
    pub v_cc_valid_volts: Volts,
    /// The sink resistance while asserted.
    pub r_ol_ohms: Ohms,
    /// The detect delay: the output asserts this long after the supply
    /// falls through `V_TH−`.
    pub t_pd_ns: u64,
    /// The release delay: the output releases this long after the supply
    /// rises through `V_TH+`.
    pub t_pr_ns: u64,
}

impl Config {
    /// An STM1061 with the given `V_TH−` (the suffix's threshold) at the
    /// datasheet typicals.
    pub const fn stm1061(v_th_minus_volts: Volts) -> Self {
        Self {
            v_th_minus_volts,
            v_hyst_volts: v_th_minus_volts * STM1061_V_HYST_RATIO,
            v_cc_valid_volts: STM1061_V_CC_VALID_VOLTS,
            r_ol_ohms: STM1061_R_OL_OHMS,
            t_pd_ns: STM1061_T_PD_NS,
            t_pr_ns: STM1061_T_PR_NS,
        }
    }

    /// The STM1061N16: the 1.6 V threshold the module's `U404` is.
    pub const fn stm1061n16() -> Self {
        Self::stm1061(STM1061N16_V_TH_MINUS_VOLTS)
    }

    /// The release threshold `V_TH+`.
    pub fn v_th_plus_volts(&self) -> Volts {
        self.v_th_minus_volts + self.v_hyst_volts
    }
}

// ============================================================
// Pin facades
// ============================================================

/// What a detector's pin is for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DetectorRole {
    /// The monitored supply (`PowerIn`), read against [`DetectorRole::Vss`].
    Vcc,
    /// The ground reference (`PowerIn`).
    Vss,
    /// The open-drain active-low output (sinks only, released).
    Out,
}

/// One row of a detector's pin table.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DetectorPin {
    /// The identifier the netlist uses.
    pub number: &'static str,
    /// An alias, when the identifier is not already the function.
    pub name: Option<&'static str>,
    /// The pin's role.
    pub role: DetectorRole,
}

const fn detector_pin(
    number: &'static str,
    name: Option<&'static str>,
    role: DetectorRole,
) -> DetectorPin {
    DetectorPin { number, name, role }
}

/// STM1061 keyed by **function**, as the P2-EC32MB netlist names `U404`'s
/// pins.
pub const STM1061_PINS_BY_FUNCTION: [DetectorPin; 3] = [
    detector_pin("VCC", None, DetectorRole::Vcc),
    detector_pin("VSS", None, DetectorRole::Vss),
    detector_pin("OUT", None, DetectorRole::Out),
];

/// STM1061 in the SOT23-3 package by pin number (Table 2, p.6): 1 `~OUT`,
/// 2 `V_SS`, 3 `V_CC`.
pub const STM1061_PINS_SOT23: [DetectorPin; 3] = [
    detector_pin("1", Some("OUT"), DetectorRole::Out),
    detector_pin("2", Some("VSS"), DetectorRole::Vss),
    detector_pin("3", Some("VCC"), DetectorRole::Vcc),
];

/// Turn one pin-table row into a [`PinDecl`]: `V_CC` measured against
/// `V_SS` — the reference the build's domain lints read
/// (`embsim_board::Finding::UnreferencedDomain`) — and `~OUT` the
/// open-drain output the part is (it sinks at `R_OL`, published per drive,
/// and releases), resting released.
fn declare(pin: &DetectorPin, vss: &'static str) -> PinDecl {
    let decl = match pin.role {
        DetectorRole::Vcc => PinDecl::power_in(pin.number).with_reference(vss),
        DetectorRole::Vss => PinDecl::power_in(pin.number),
        DetectorRole::Out => PinDecl::digital_out(pin.number).sink_only(),
    };
    match pin.name {
        Some(name) => decl.with_name(name),
        None => decl,
    }
}

// ============================================================
// Core
// ============================================================

/// A pin nothing has been handed yet: no voltage, no clock.
const NOTHING: Sense = Sense {
    volts: None,
    periodic: None,
    at_ns: 0,
};

/// What the comparator decides.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Verdict {
    /// The supply is below the guaranteed range: undefined, released.
    Undefined,
    /// Below `V_TH−` (or held from below inside the band): sinking.
    Asserted,
    /// At or above `V_TH+` (or held from above inside the band): released.
    Released,
}

#[derive(Debug)]
struct State {
    /// `V_CC` as handed: against `V_SS`.
    vcc: Sense,
    /// `V_SS` as handed: in the engine's frame (it declares no reference),
    /// the voltage the output sinks to.
    vss: Sense,
    verdict: Verdict,
    /// The instant the verdict last changed.
    decided_at_ns: Option<u64>,
    /// A crossing whose drive is scheduled: `(instant, asserted)`.
    pending: Option<(u64, bool)>,
    /// What the output pin holds: `None` released.
    published: Option<TheveninDrive>,
    output: Option<PinHandle>,
    io: Option<ComponentNetIo>,
    drives: u64,
}

#[derive(Debug)]
struct Core {
    config: Config,
    state: Mutex<State>,
}

impl Core {
    /// The comparator on `V_CC − V_SS`, the voltage `V_CC` is handed: none
    /// — nothing reaches `V_CC`, nothing holds `V_SS`, a clock names no
    /// supply (the wildcard audit, `NODES.md` §12 item 5) — is undefined.
    fn verdict(&self, state: &State) -> Verdict {
        let Some(supply) = state.vcc.volts else {
            return Verdict::Undefined;
        };
        if supply < self.config.v_cc_valid_volts {
            Verdict::Undefined
        } else if supply < self.config.v_th_minus_volts {
            Verdict::Asserted
        } else if supply >= self.config.v_th_plus_volts() {
            Verdict::Released
        } else {
            match state.verdict {
                // Inside the band from below, or from nothing: the part
                // powered up under the threshold and has not released.
                Verdict::Undefined | Verdict::Asserted => Verdict::Asserted,
                Verdict::Released => Verdict::Released,
            }
        }
    }

    fn publish(&self, state: &mut State, drive: Option<TheveninDrive>) {
        if state.published == drive {
            return;
        }
        state.published = drive;
        state.drives += 1;
        if let Some(out) = &state.output {
            out.set_drive(drive);
        }
    }

    /// The open-drain output sinking to `VSS` behind `R_OL` — at the
    /// voltage `VSS` names, and released where it names none: no ground is
    /// implied (`DESIGN.md` rule 6).
    fn sink_drive(&self, state: &State) -> Option<TheveninDrive> {
        state.vss.volts.map(|volts| TheveninDrive {
            volts,
            impedance: self.config.r_ol_ohms,
        })
    }

    /// A supply reading arrived: decide, and either drive at once (a part
    /// waking up asserted, or losing its bias) or schedule the crossing's
    /// delay.
    fn on_supply(&self, state: &mut State, now_ns: u64) {
        let previous = state.verdict;
        let next = self.verdict(state);
        state.verdict = next;
        if previous != next {
            state.decided_at_ns = Some(now_ns);
        }
        match (previous, next) {
            (_, Verdict::Undefined) => {
                state.pending = None;
                self.publish(state, None);
            }
            (Verdict::Undefined, Verdict::Asserted) => {
                state.pending = None;
                let drive = self.sink_drive(state);
                self.publish(state, drive);
            }
            (Verdict::Undefined, Verdict::Released) => {
                state.pending = None;
                self.publish(state, None);
            }
            (Verdict::Released, Verdict::Asserted) => {
                self.schedule(state, now_ns.saturating_add(self.config.t_pd_ns), true);
            }
            (Verdict::Asserted, Verdict::Released) => {
                self.schedule(state, now_ns.saturating_add(self.config.t_pr_ns), false);
            }
            (Verdict::Asserted, Verdict::Asserted) => {
                // The ground may have moved: the sink follows it.
                if state.pending.is_none() {
                    let drive = self.sink_drive(state);
                    self.publish(state, drive);
                }
            }
            (Verdict::Released, Verdict::Released) => {}
        }
    }

    fn schedule(&self, state: &mut State, at_ns: u64, asserted: bool) {
        state.pending = Some((at_ns, asserted));
        if let Some(io) = &state.io {
            io.schedule_at_ns(at_ns);
        }
    }

    fn on_wake(&self, state: &mut State, now_ns: u64) {
        let Some((at_ns, asserted)) = state.pending else {
            return;
        };
        if now_ns < at_ns {
            return;
        }
        state.pending = None;
        // The crossing that armed this may have been reversed since: the
        // verdict now is what the output follows.
        let drive = match (asserted, state.verdict) {
            (true, Verdict::Asserted) => self.sink_drive(state),
            (false, Verdict::Released) => None,
            _ => return,
        };
        self.publish(state, drive);
    }
}

// ============================================================
// Monitor
// ============================================================

/// Cheap cloneable read handle onto a live [`VoltageDetector`].
#[derive(Clone, Debug)]
pub struct DetectorMonitor {
    core: Arc<Core>,
}

impl DetectorMonitor {
    /// Whether the output is asserted (sinking): `None` while the supply
    /// is below the guaranteed range and the output undefined.
    pub fn asserted(&self) -> Option<bool> {
        match self.core.state.lock().unwrap().verdict {
            Verdict::Undefined => None,
            Verdict::Asserted => Some(true),
            Verdict::Released => Some(false),
        }
    }

    /// A crossing whose drive is scheduled and not yet published: the
    /// instant, and whether it asserts.
    pub fn pending(&self) -> Option<(u64, bool)> {
        self.core.state.lock().unwrap().pending
    }

    /// The instant the comparator's verdict last changed — the crossing
    /// itself, before any delay. `None` while nothing has been decided.
    pub fn decided_at_ns(&self) -> Option<u64> {
        self.core.state.lock().unwrap().decided_at_ns
    }

    /// Drives published since construction: one per crossing.
    pub fn drive_count(&self) -> u64 {
        self.core.state.lock().unwrap().drives
    }

    /// The configuration in force.
    pub fn config(&self) -> &Config {
        &self.core.config
    }
}

// ============================================================
// Component
// ============================================================

/// A voltage detector as a live board-engine component.
///
/// ```rust
/// use embsim_board::Component;
/// use embsim_models::supervisor::{Config, VoltageDetector, STM1061_PINS_BY_FUNCTION};
///
/// let detector = VoltageDetector::new(Config::stm1061n16(), &STM1061_PINS_BY_FUNCTION);
/// assert_eq!(detector.pins().len(), 3);
/// assert!((detector.monitor().config().v_th_plus_volts() - 1.68).abs() < 1e-9);
/// ```
#[derive(Debug)]
pub struct VoltageDetector {
    pins: Vec<PinDecl>,
    table: &'static [DetectorPin],
    core: Arc<Core>,
}

impl VoltageDetector {
    /// A detector with `config` behind the pin table `table`, which must
    /// carry one pin of each role.
    pub fn new(config: Config, table: &'static [DetectorPin]) -> Self {
        let pin_of = |role: DetectorRole| {
            table
                .iter()
                .find(|p| p.role == role)
                .map(|p| p.number)
                .expect("a detector table carries one pin of each role")
        };
        Self {
            pins: table
                .iter()
                .map(|p| declare(p, pin_of(DetectorRole::Vss)))
                .collect(),
            table,
            core: Arc::new(Core {
                config,
                state: Mutex::new(State {
                    vcc: NOTHING,
                    vss: NOTHING,
                    verdict: Verdict::Undefined,
                    decided_at_ns: None,
                    pending: None,
                    published: None,
                    output: None,
                    io: None,
                    drives: 0,
                }),
            }),
        }
    }

    /// A read handle onto this detector.
    pub fn monitor(&self) -> DetectorMonitor {
        DetectorMonitor {
            core: Arc::clone(&self.core),
        }
    }

    fn pin_number(&self, role: DetectorRole) -> &'static str {
        self.table
            .iter()
            .find(|p| p.role == role)
            .map(|p| p.number)
            .expect("checked at new")
    }
}

impl Component for VoltageDetector {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        {
            let mut state = self.core.state.lock().unwrap();
            state.output = Some(io.pin(self.pin_number(DetectorRole::Out))?);
            state.io = Some(io.clone());
        }
        let core = Arc::clone(&self.core);
        io.on_wake_ns(move |now_ns| {
            let mut state = core.state.lock().unwrap();
            core.on_wake(&mut state, now_ns);
        });
        // The ground first, so the supply is read against it.
        for role in [DetectorRole::Vss, DetectorRole::Vcc] {
            let core = Arc::clone(&self.core);
            io.on_sense(self.pin_number(role), move |sensed| {
                let mut state = core.state.lock().unwrap();
                match role {
                    DetectorRole::Vss => state.vss = sensed,
                    _ => state.vcc = sensed,
                }
                core.on_supply(&mut state, virtual_clock::virtual_ns());
            })?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use rstest::rstest;

    use super::*;

    /// A pin handed `volts`.
    fn handed(volts: Option<Volts>) -> Sense {
        Sense {
            volts,
            periodic: None,
            at_ns: 0,
        }
    }

    #[rstest]
    fn the_n16_part_carries_the_datasheet_typicals() {
        let config = Config::stm1061n16();
        assert_eq!(config.v_th_minus_volts, 1.6);
        assert!((config.v_th_plus_volts() - 1.68).abs() < 1e-12);
        assert_eq!(config.v_cc_valid_volts, 0.7);
        assert!((config.r_ol_ohms - 294.117_647).abs() < 1e-3);
        assert_eq!(config.t_pd_ns, 25_000);
        assert_eq!(config.t_pr_ns, 30_000);
    }

    /// The core without an engine: undefined below 0.7 V (released),
    /// asserted at once when the part wakes under the threshold, held
    /// inside the band, released `t_PR` after crossing `V_TH+`, and
    /// asserted again `t_PD` after falling through `V_TH−`.
    #[rstest]
    fn the_comparator_has_hysteresis_and_delays_its_crossings() {
        let detector = VoltageDetector::new(Config::stm1061n16(), &STM1061_PINS_BY_FUNCTION);
        let core = Arc::clone(&detector.core);
        let mut state = core.state.lock().unwrap();
        state.vss = handed(Some(0.0));

        state.vcc = handed(Some(0.5));
        core.on_supply(&mut state, 0);
        assert_eq!(state.verdict, Verdict::Undefined);
        assert_eq!(state.published, None);

        state.vcc = handed(Some(1.2));
        core.on_supply(&mut state, 1);
        assert_eq!(state.verdict, Verdict::Asserted);
        assert_eq!(
            state.published,
            Some(TheveninDrive {
                volts: 0.0,
                impedance: STM1061_R_OL_OHMS
            }),
            "waking under the threshold asserts at once"
        );
        assert_eq!(state.pending, None);

        state.vcc = handed(Some(1.65));
        core.on_supply(&mut state, 2);
        assert_eq!(
            state.verdict,
            Verdict::Asserted,
            "inside the band from below"
        );
        assert_eq!(state.drives, 1);

        state.vcc = handed(Some(1.813));
        core.on_supply(&mut state, 1_000);
        assert_eq!(state.verdict, Verdict::Released);
        assert_eq!(state.pending, Some((1_000 + STM1061_T_PR_NS, false)));
        assert!(state.published.is_some(), "still sinking until t_PR");
        core.on_wake(&mut state, 1_000 + STM1061_T_PR_NS - 1);
        assert!(state.published.is_some());
        core.on_wake(&mut state, 1_000 + STM1061_T_PR_NS);
        assert_eq!(state.published, None);

        state.vcc = handed(Some(1.62));
        core.on_supply(&mut state, 2_000);
        assert_eq!(
            state.verdict,
            Verdict::Released,
            "inside the band from above"
        );

        state.vcc = handed(Some(1.5));
        core.on_supply(&mut state, 3_000);
        assert_eq!(state.pending, Some((3_000 + STM1061_T_PD_NS, true)));
        core.on_wake(&mut state, 3_000 + STM1061_T_PD_NS);
        assert!(state.published.is_some());
        assert_eq!(state.drives, 3);
    }

    /// An asserted output sinks to the voltage `VSS` names and is released
    /// where `VSS` names none: no ground is implied. (On a board `VCC` is
    /// measured against `VSS`, so a `VSS` naming nothing hands `VCC`
    /// nothing and the verdict is undefined first; the state is set here
    /// directly to reach the sink with no ground under it.)
    #[rstest]
    #[case::held_ground(Some(0.0), Some(TheveninDrive { volts: 0.0, impedance: STM1061_R_OL_OHMS }))]
    #[case::ground_above_zero(Some(0.2), Some(TheveninDrive { volts: 0.2, impedance: STM1061_R_OL_OHMS }))]
    #[case::no_ground(None, None)]
    fn an_asserted_output_sinks_to_the_voltage_vss_names(
        #[case] vss: Option<Volts>,
        #[case] expected: Option<TheveninDrive>,
    ) {
        let detector = VoltageDetector::new(Config::stm1061n16(), &STM1061_PINS_BY_FUNCTION);
        let core = Arc::clone(&detector.core);
        let mut state = core.state.lock().unwrap();
        state.vss = handed(vss);
        state.vcc = handed(Some(1.2));
        core.on_supply(&mut state, 0);
        assert_eq!(state.verdict, Verdict::Asserted, "1.2 V is under V_TH-");
        assert_eq!(state.published, expected);
    }

    /// A crossing reversed before its delay elapses publishes nothing.
    #[rstest]
    fn a_reversed_crossing_publishes_nothing_at_its_instant() {
        let detector = VoltageDetector::new(Config::stm1061n16(), &STM1061_PINS_BY_FUNCTION);
        let core = Arc::clone(&detector.core);
        let mut state = core.state.lock().unwrap();
        state.vss = handed(Some(0.0));
        state.vcc = handed(Some(1.2));
        core.on_supply(&mut state, 0);
        state.vcc = handed(Some(1.8));
        core.on_supply(&mut state, 10);
        let (at_ns, _) = state.pending.unwrap();
        state.vcc = handed(Some(1.2));
        core.on_supply(&mut state, 20);
        core.on_wake(&mut state, at_ns);
        assert!(state.published.is_some(), "asserted throughout");
        assert_eq!(state.drives, 1);
    }

    #[rstest]
    fn the_facades_declare_a_released_open_drain_output_referenced_to_vss() {
        for table in [&STM1061_PINS_BY_FUNCTION[..], &STM1061_PINS_SOT23[..]] {
            let detector = VoltageDetector::new(Config::stm1061n16(), table);
            let out = detector
                .pins()
                .iter()
                .find(|p| p.drives())
                .expect("an output");
            assert_eq!(out.idle, None);
            assert!(out.can_sink && !out.can_source, "open drain");
            let referenced: Vec<_> = detector
                .pins()
                .iter()
                .filter(|p| p.reference.is_some())
                .collect();
            assert_eq!(referenced.len(), 1, "V_CC against V_SS");
        }
    }
}
