//! Isolation and interface components — the parts that stand *between* an MCU
//! pin and the machine.
//!
//! [`crate::machine`] models the physical world off the board; this module
//! models the row of parts a signal has to survive on its way there. On the
//! reference machine every hop between the P2 and the motor, the encoder or an
//! end switch crosses one of them, and while they are topology-only stubs
//! neither a level nor a pulse train reaches the other side:
//!
//! ```text
//!   P2 pin                 this module                          machine
//!   ──────                 ───────────                          ───────
//!   P8  STEP ──┐
//!   P7  DIR  ──┼──► iso67xx::Iso67xx (IC14, ISO6741) ──┬──────► MOTOR.STEP/DIR
//!   P6  ENA  ──┘                                       └─► Q1 (pwl_library::mmbt3904)
//!                                                                └──► MOTOR.ENA
//!   P9..P12  ◄──── iso67xx::Iso67xx (IC16, ISO6740F) ◄──────────── encoder
//!
//!   P18/P19  ◄──── opto::Opto (U6) ◄── IC9 (pwl_library::nsi50010) ◄── end switch
//! ```
//!
//! Only the digital isolators are models here. The transistor, the
//! current regulators and the optocouplers' LEDs are **piecewise-linear
//! elements** the engine solves ([`crate::pwl_library`]: `Q1` is a
//! base–emitter diode and a gated collector, `IC6`–`IC13` are two-region
//! regulating branches), and the optocouplers are [`crate::opto::Opto`],
//! whose LED is a branch the part declares and whose output is a sink that
//! releases. `NODES.md` §8 phase 3 turned each from a Thevenin stand-in
//! into the element it is.
//!
//! # Shape of every component here
//!
//! The rules of [`crate::machine`] apply unchanged — no polling, read-time
//! state, a `Component` plus a cheap cloneable handle, pin identity is the
//! signal name — plus two that these parts make load-bearing:
//!
//! - **Supply-gated, like [`crate::ads122u04_component`].** Every part senses
//!   its own supply pins and decides from the *datasheet's* function table
//!   what it does when a supply is down. A digital isolator with one side
//!   unpowered does not pass a signal; that is the whole point of modeling it
//!   rather than shorting across it.
//! - **Drive on change only.** A repeater that re-drives its output every time
//!   any input event arrives multiplies engine resolutions by its channel
//!   count. Every output here compares the [`TheveninDrive`] it is about to
//!   apply against the one already applied and does nothing when they match,
//!   and every channel subscribes only to *its own* input, so one transition
//!   on one channel costs one drive. `board/tests/isolation_bridge.rs` asserts
//!   the resulting engine-event budget.
//!
//! # Elements, not chains
//!
//! Two parts of the end-switch path — the current regulator and the
//! optocoupler's LED — live in a **series current loop**. A loop is a
//! cluster solve, and the engine solves it: the regulator's two-region
//! branch and the LED's diode branch are stamped with the contact and the
//! rail around them, their regions chosen by the flip loop, and the loop
//! current read off the solution (`BuiltSystem::branch_current("Board.IC9")`
//! and the current instrument the opto subscribes on its anode). What the
//! bench asks — *is the loop closed, and at what current?* — is answered
//! by the operating point: 10 mA in regulation, not the current a
//! resistive stand-in would carry. The "drive one terminal from the other"
//! chain that once stood in for the loop, and the output impedances tuned
//! so its Thevenin sources cleared the engine's strength ratio, are gone
//! with it: source-strength projection ranks a board's pull-up as a pull
//! against any datasheet sink, so an output resistance is the datasheet's
//! bound and nothing else.
//!
//! # Registering the parts
//!
//! A consumer registers each part with one `register` line and nothing
//! else — the pin facades here *are* the datasheet pin tables, so the build
//! validates them against the netlist in both directions:
//!
//! ```rust
//! use embsim_board::{PartRegistry, registry::normalize_part};
//! use embsim_models::isolation::{iso67xx, Iso67xx};
//! use embsim_models::opto::Opto;
//! use embsim_models::pwl_library;
//!
//! let mut registry = PartRegistry::new();
//!
//! // One arm covers the whole isolator family: the orderable part number
//! // carries the variant and the fail-safe option.
//! for part in ["ISO6742DWR", "ISO6741DWR", "ISO6740FDWR", "ISO6721BDR"] {
//!     registry.register(part, |decl| {
//!         let name = normalize_part(decl);
//!         let config = iso67xx::Config::from_part_name(&name).expect("an ISO67xx");
//!         // A step clock crosses any channel as a clock, not as edges.
//!         Box::new(Iso67xx::new(config).expect("a valid isolator"))
//!     });
//! }
//! // The optocouplers, by part; the transistor, the regulators, the
//! // diodes and the LEDs by manufacturer part number, from the library.
//! registry.register("VO2631", |_| Box::new(Opto::vo2631()));
//! registry.register("6N137", |_| Box::new(Opto::lite_on_6n137()));
//! pwl_library::register(&mut registry);
//! ```
//!
//! Two things the *system description* must then supply, because a promoted
//! part gates on them where a stub did not:
//!
//! 1. **Both sides of every isolator need a rail.** A supply the engine leaves
//!    floating is a down supply, so a barrier with only one strapped side
//!    passes nothing — correctly, and loudly.
//! 2. **A current loop needs a source.** An optocoupler driven from a
//!    dry-contact loop stays dark until the loop has an EMF in it; on the
//!    reference board the end-switch loop is drawn closed and unpowered, so the
//!    harness has to say what a working machine provides.
//!
//! `board/tests/isolation_bridge.rs` is the worked example of all of it.

pub mod iso67xx;

pub use iso67xx::{Channel, Iso67xx, Iso67xxMonitor, Side, Variant};

use std::fmt;

use embsim_board::{Level, Sense, TheveninDrive, Volts};

// ============================================================
// Configuration errors
// ============================================================

/// A part's configuration was rejected at construction.
///
/// Like [`crate::machine::MachineConfigError`], every `Config` here validates
/// in `new` rather than clamping silently: a zero threshold or a negative
/// drive impedance is a system-description bug, and a part that quietly
/// substitutes a different number is worse than one that refuses to build.
#[derive(Debug, Clone, PartialEq)]
pub enum PartConfigError {
    /// A voltage, current, or impedance parameter must be finite and strictly
    /// positive.
    NotPositive {
        /// Field name as spelled in the `Config` struct.
        field: &'static str,
        /// The offending value.
        value: f64,
    },
    /// The low-level input threshold sits at or above the high-level one, so
    /// no input voltage could ever be unambiguous.
    InvertedThresholds {
        /// Configured `V_IL` fraction of the input supply.
        vil_ratio: f64,
        /// Configured `V_IH` fraction of the input supply.
        vih_ratio: f64,
    },
}

impl fmt::Display for PartConfigError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            PartConfigError::NotPositive { field, value } => {
                write!(f, "{field} must be finite and > 0 (got {value})")
            }
            PartConfigError::InvertedThresholds {
                vil_ratio,
                vih_ratio,
            } => write!(
                f,
                "inverted input thresholds: V_IL ratio {vil_ratio} must be below V_IH ratio \
                 {vih_ratio}, or no input voltage is ever unambiguous"
            ),
        }
    }
}

impl std::error::Error for PartConfigError {}

/// Validate a finite, strictly positive parameter.
pub(crate) fn require_positive(field: &'static str, value: f64) -> Result<(), PartConfigError> {
    if value.is_finite() && value > 0.0 {
        Ok(())
    } else {
        Err(PartConfigError::NotPositive { field, value })
    }
}

// ============================================================
// Supply gating
// ============================================================

/// The voltage a sensed supply pin is at when it is **up** — at or above a
/// part's own minimum against the pin's reference — and `None` when it is
/// not.
///
/// A supply that names no voltage is down: floating, a clock (a periodic
/// net names no operating voltage), fought for half of every cycle, a node
/// only an unmodelled rail reaches, or a ground the pin is measured against
/// that nothing holds. The engine never invents a value for such a node
/// (`DESIGN.md` rule 6), so neither does a part: a system description that
/// forgot a rail — or its return — gets a down part, which is exactly the
/// failure it should get. [`crate::ads122u04_component`]'s gate reads its
/// supplies the same way.
pub fn supply_volts(sense: &Sense, min_volts: Volts) -> Option<Volts> {
    sense.volts.filter(|&volts| volts >= min_volts)
}

/// Whether a sensed supply pin is up ([`supply_volts`]).
pub fn supply_up(sense: &Sense, min_volts: Volts) -> bool {
    supply_volts(sense, min_volts).is_some()
}

// ============================================================
// Drives
// ============================================================

/// A push-pull drive of `level` against a rail at `rail`, through
/// `impedance`.
pub(crate) fn level_drive(level: Level, rail: Volts, impedance: f64) -> TheveninDrive {
    TheveninDrive {
        volts: match level {
            Level::High => rail,
            Level::Low => 0.0,
        },
        impedance,
    }
}

// ============================================================
// Tests
// ============================================================

#[cfg(test)]
mod tests {
    use rstest::rstest;

    use super::*;

    /// A sense handed `volts`.
    fn handed(volts: Option<Volts>) -> Sense {
        Sense {
            volts,
            periodic: None,
            at_ns: 0,
        }
    }

    #[rstest]
    #[case::above(Some(3.3), Some(3.3))]
    #[case::at_minimum(Some(1.71), Some(1.71))]
    #[case::below(Some(1.0), None)]
    #[case::zero(Some(0.0), None)]
    #[case::no_voltage(None, None)]
    fn a_supply_is_up_at_its_voltage_from_the_minimum(
        #[case] volts: Option<Volts>,
        #[case] expect: Option<Volts>,
    ) {
        assert_eq!(supply_volts(&handed(volts), 1.71), expect);
        assert_eq!(supply_up(&handed(volts), 1.71), expect.is_some());
    }

    #[rstest]
    fn level_drive_sources_the_rail_or_ground() {
        assert_eq!(
            level_drive(Level::High, 5.0, 100.0),
            TheveninDrive {
                volts: 5.0,
                impedance: 100.0
            }
        );
        assert_eq!(
            level_drive(Level::Low, 5.0, 100.0),
            TheveninDrive {
                volts: 0.0,
                impedance: 100.0
            }
        );
    }

    #[rstest]
    #[case::zero(0.0)]
    #[case::negative(-1.0)]
    #[case::nan(f64::NAN)]
    #[case::infinite(f64::INFINITY)]
    fn require_positive_rejects(#[case] value: f64) {
        match require_positive("vf_volts", value) {
            Err(PartConfigError::NotPositive {
                field,
                value: reported,
            }) => {
                assert_eq!(field, "vf_volts");
                assert!(reported.total_cmp(&value).is_eq());
            }
            other => panic!("expected NotPositive, got {other:?}"),
        }
    }

    #[rstest]
    fn errors_display_their_fields() {
        assert!(PartConfigError::NotPositive {
            field: "ireg_ma",
            value: 0.0
        }
        .to_string()
        .contains("ireg_ma"));
        assert!(PartConfigError::InvertedThresholds {
            vil_ratio: 0.7,
            vih_ratio: 0.3
        }
        .to_string()
        .contains("0.7"));
    }
}
