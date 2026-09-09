//! Machine components — the physical world as harness-attached
//! [`embsim_board::Component`]s.
//!
//! Everything in [`crate::ads122u04_component`] lives *on a PCB*. The parts
//! here live *off* it: the motors, encoders, and end switches a machine
//! harness connects to MCU pins. Each one is a `Component` with a real pin
//! facade, so a system description wires it exactly like a chip:
//!
//! ```text
//!   MCU pin facade                  machine components
//!   ──────────────                  ──────────────────
//!   P{step} ──STEP──►┐
//!   P{dir}  ──DIR───►┤ stepper_motor::StepperMotor ──shaft (mm)──┐
//!   P{ena}  ──ENA───►┘                                           │
//!                                                                ▼
//!   P{a}   ◄──A──────┐ quadrature_encoder::QuadratureEncoder ◄────┤
//!   P{b}   ◄──B──────┤ (Gray-code A/B, optional index Z)          │
//!   P{z}   ◄──Z──────┘                                           │
//!                                                                ▼
//!   P{end} ◄──NO─────── end_switch::EndSwitch ◄───────────────────┘
//!                       (COM + NO dry contact, hysteresis, bounce)
//! ```
//!
//! # Shape of every component here
//!
//! - **Never polls.** Inputs arrive as [`embsim_board::ComponentNetIo::on_sense`]
//!   callbacks; time-driven work uses the engine's timer wheel
//!   (`schedule_at` / `schedule_every`). Idle components cost nothing
//!   (`BOARD_ENGINE.md`, "Execution model").
//! - **State is computed at read time**, in closed form, from the virtual
//!   clock — never integrated per tick. `virtual_us` is a counter the engine
//!   advances; a per-tick integrator's answer would depend on how often
//!   anyone looked, while a closed form does not.
//! - **Split into a `Component` and a cheap handle.** The `Component` is
//!   moved into the [`embsim_board::System`]; the handle
//!   ([`stepper_motor::MotorShaft`], [`quadrature_encoder::EncoderInput`],
//!   [`end_switch::EndSwitchActuator`]) is cloned out beforehand and is how
//!   physics chains and tests read/drive the part afterwards.
//! - **Pin identity is the signal name** (`"STEP"`, `"A"`, `"COM"`), matching
//!   [`embsim_board::McuComponent`]'s `"P{n}"` convention: a bench-attached
//!   component's pins become nets named `"{name}.{pin}"`
//!   ([`embsim_board::System::component`]), so a harness reads
//!   `"MOTOR.STEP"`. A netlist that *places* one of these parts must name the
//!   matching pins through KiCad `pinfunction` (the netlist pin identity is
//!   "pinfunction if present, else pin number").
//!
//! # Provenance convention
//!
//! Every module here carries the physics header `BOARD_ENGINE.md` ("Model
//! provenance convention") requires: the governing equation, where each
//! parameter came from, and what is deliberately **not** modeled. The
//! reference consumer throughout is MaD's tensile tester — its hand-wired
//! plant (`SIL/MaDSim/src/wiring.rs`) and its EdgeBoard schematic
//! (`Hardware/EdgeBoard/KiCad/MaD_Edge_Sheet3.kicad_sch`) are the cited
//! sources.

pub mod end_switch;
pub mod quadrature_encoder;
pub mod stepper_motor;

pub use end_switch::{ActuationSense, BounceConfig, EndSwitch, EndSwitchActuator};
pub use quadrature_encoder::{ChannelOrder, EncoderInput, IndexConfig, QuadratureEncoder};
pub use stepper_motor::{MotorShaft, StepEdge, StepperMotor};

use std::fmt;

use embsim_board::{Level, NetState, Volts};

// ============================================================
// Shared electrical constants
// ============================================================

/// Default logic-input threshold: a sensed node voltage at or above this
/// projects to [`Level::High`], below it to [`Level::Low`].
///
/// 1.5 V is the mid-rail of a 3.3 V system and matches the net engine's own
/// digital projection threshold, so a component's own reading of an
/// [`NetState::Analog`] node agrees with the engine's `Driven`/`Pulled`
/// projection of the same node. Real parts specify asymmetric `V_IL`/`V_IH`
/// with a dead band; that projection (and its
/// [`embsim_board::Finding::AmbiguousLevel`]) is an engine-side slice, so
/// every component here takes the threshold as configuration rather than
/// hard-coding a family.
pub const DEFAULT_INPUT_THRESHOLD_VOLTS: Volts = 1.5;

/// Default open-circuit voltage a machine component drives for a logic high
/// (3.3 V logic — the reference consumer's I/O rail).
pub const DEFAULT_HIGH_VOLTS: Volts = 3.3;

// ============================================================
// Level projection
// ============================================================

/// Project a resolved [`NetState`] onto a logic level, or `None` when the
/// engine offers no defensible one.
///
/// The engine never invents a value for an unsourced or fought-over node
/// (`BOARD_ENGINE.md`, "Net state model"), and neither does this helper:
/// [`NetState::Floating`] and [`NetState::Contention`] return `None` so each
/// component can choose its own documented behavior (a stepper drive holds
/// the last DIR it latched; a floating ENA is *not* an enable).
///
/// - [`NetState::Driven`] / [`NetState::Pulled`] carry the level directly.
/// - [`NetState::Analog`] is compared against `threshold_volts`. A NaN node
///   voltage — which the resolver filters before publication, so this is
///   defense in depth — is treated as no level at all rather than silently
///   projecting low.
pub fn digital_level(state: NetState, threshold_volts: Volts) -> Option<Level> {
    match state {
        NetState::Driven(level) | NetState::Pulled(level, _) => Some(level),
        NetState::Analog(volts) if volts.is_nan() => None,
        NetState::Analog(volts) if volts >= threshold_volts => Some(Level::High),
        NetState::Analog(_) => Some(Level::Low),
        NetState::Floating | NetState::Contention => None,
    }
}

/// The other logic level.
pub(crate) fn invert(level: Level) -> Level {
    match level {
        Level::High => Level::Low,
        Level::Low => Level::High,
    }
}

// ============================================================
// Configuration errors
// ============================================================

/// A machine component's configuration was rejected at construction.
///
/// Every component here validates its `Config` in `new` rather than clamping
/// silently: a zero time constant or an inverted hysteresis band is a system
/// description bug, and a plant that quietly substitutes a different number
/// is worse than one that refuses to build.
#[derive(Debug, Clone, PartialEq)]
pub enum MachineConfigError {
    /// A scale or time parameter must be finite and strictly positive.
    NotPositive {
        /// Field name as spelled in the `Config` struct.
        field: &'static str,
        /// The offending value.
        value: f64,
    },
    /// A fractional-loss parameter must be finite and in `[0, 1)`.
    NotAFraction {
        /// Field name as spelled in the `Config` struct.
        field: &'static str,
        /// The offending value.
        value: f64,
    },
    /// A count parameter must be non-zero.
    Zero {
        /// Field name as spelled in the `Config` struct.
        field: &'static str,
    },
    /// The release point sits on the *actuated* side of the operate point,
    /// so the contact could never reopen.
    InvertedHysteresis {
        /// Configured actuation sense, as spelled in
        /// [`end_switch::ActuationSense`].
        sense: &'static str,
        /// Configured operate position (mm).
        operate_mm: f64,
        /// Configured release position (mm).
        release_mm: f64,
    },
}

impl fmt::Display for MachineConfigError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            MachineConfigError::NotPositive { field, value } => {
                write!(f, "{field} must be finite and > 0 (got {value})")
            }
            MachineConfigError::NotAFraction { field, value } => {
                write!(f, "{field} must be finite and in [0, 1) (got {value})")
            }
            MachineConfigError::Zero { field } => write!(f, "{field} must be non-zero"),
            MachineConfigError::InvertedHysteresis {
                sense,
                operate_mm,
                release_mm,
            } => write!(
                f,
                "inverted hysteresis for {sense} actuation: release {release_mm} mm is on the \
                 actuated side of operate {operate_mm} mm, so the contact could never reopen"
            ),
        }
    }
}

impl std::error::Error for MachineConfigError {}

/// Validate a finite, strictly positive parameter.
pub(crate) fn require_positive(field: &'static str, value: f64) -> Result<(), MachineConfigError> {
    if value.is_finite() && value > 0.0 {
        Ok(())
    } else {
        Err(MachineConfigError::NotPositive { field, value })
    }
}

/// Validate a finite fractional parameter in `[0, 1)`.
pub(crate) fn require_fraction(field: &'static str, value: f64) -> Result<(), MachineConfigError> {
    if value.is_finite() && (0.0..1.0).contains(&value) {
        Ok(())
    } else {
        Err(MachineConfigError::NotAFraction { field, value })
    }
}

// ============================================================
// Tests
// ============================================================

#[cfg(test)]
mod tests {
    use rstest::rstest;

    use super::*;

    /// Digital projections carry their level through unchanged; analog nodes
    /// are compared against the declared threshold.
    #[rstest]
    #[case::driven_high(NetState::Driven(Level::High), Some(Level::High))]
    #[case::driven_low(NetState::Driven(Level::Low), Some(Level::Low))]
    #[case::pulled_high(NetState::Pulled(Level::High, 10_000.0), Some(Level::High))]
    #[case::pulled_low(NetState::Pulled(Level::Low, 4_700.0), Some(Level::Low))]
    #[case::analog_above(NetState::Analog(3.3), Some(Level::High))]
    #[case::analog_at_threshold(NetState::Analog(1.5), Some(Level::High))]
    #[case::analog_below(NetState::Analog(1.49), Some(Level::Low))]
    #[case::analog_zero(NetState::Analog(0.0), Some(Level::Low))]
    fn digital_level_projects_sourced_nets(#[case] state: NetState, #[case] expect: Option<Level>) {
        assert_eq!(digital_level(state, DEFAULT_INPUT_THRESHOLD_VOLTS), expect);
    }

    /// The engine never invents a value for an unsourced or fought-over node,
    /// so neither does the projection — each component picks its own
    /// documented behavior for `None`.
    #[rstest]
    #[case::floating(NetState::Floating)]
    #[case::contention(NetState::Contention)]
    #[case::nan(NetState::Analog(f64::NAN))]
    fn digital_level_refuses_to_guess(#[case] state: NetState) {
        assert_eq!(digital_level(state, DEFAULT_INPUT_THRESHOLD_VOLTS), None);
    }

    #[rstest]
    fn invert_swaps_levels() {
        assert_eq!(invert(Level::High), Level::Low);
        assert_eq!(invert(Level::Low), Level::High);
    }

    /// `PartialEq` on the error's `f64` payload is IEEE, so a NaN case cannot
    /// be compared with `assert_eq!` — the variant, field, and payload are
    /// destructured instead (`total_cmp` makes NaN compare equal to itself).
    #[rstest]
    #[case::zero(0.0)]
    #[case::negative(-1.0)]
    #[case::nan(f64::NAN)]
    #[case::infinite(f64::INFINITY)]
    fn require_positive_rejects(#[case] value: f64) {
        match require_positive("tau_s", value) {
            Err(MachineConfigError::NotPositive {
                field,
                value: reported,
            }) => {
                assert_eq!(field, "tau_s");
                assert!(reported.total_cmp(&value).is_eq());
            }
            other => panic!("expected NotPositive, got {other:?}"),
        }
    }

    #[rstest]
    fn require_positive_accepts_finite_positive() {
        assert_eq!(require_positive("tau_s", 0.02), Ok(()));
    }

    #[rstest]
    #[case::one(1.0)]
    #[case::above_one(1.5)]
    #[case::negative(-0.1)]
    #[case::nan(f64::NAN)]
    fn require_fraction_rejects(#[case] value: f64) {
        match require_fraction("load_loss", value) {
            Err(MachineConfigError::NotAFraction {
                field,
                value: reported,
            }) => {
                assert_eq!(field, "load_loss");
                assert!(reported.total_cmp(&value).is_eq());
            }
            other => panic!("expected NotAFraction, got {other:?}"),
        }
    }

    #[rstest]
    #[case::zero(0.0)]
    #[case::typical(0.15)]
    #[case::nearly_one(0.999)]
    fn require_fraction_accepts(#[case] value: f64) {
        assert_eq!(require_fraction("load_loss", value), Ok(()));
    }

    /// Errors render their fields, so a failed system description says what
    /// to fix.
    #[rstest]
    fn errors_display_their_fields() {
        assert!(MachineConfigError::NotPositive {
            field: "steps_per_mm",
            value: 0.0
        }
        .to_string()
        .contains("steps_per_mm"));
        assert!(MachineConfigError::Zero {
            field: "counts_per_revolution"
        }
        .to_string()
        .contains("counts_per_revolution"));
        let inverted = MachineConfigError::InvertedHysteresis {
            sense: "Increasing",
            operate_mm: 10.0,
            release_mm: 11.0,
        }
        .to_string();
        assert!(inverted.contains("Increasing") && inverted.contains("11"));
    }
}
