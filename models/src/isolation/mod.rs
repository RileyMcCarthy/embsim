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
//!   P6  ENA  ──┘                                       └─► npn_switch::NpnSwitch (Q1)
//!                                                                └──► MOTOR.ENA
//!   P9..P12  ◄──── iso67xx::Iso67xx (IC16, ISO6740F) ◄──────────── encoder
//!
//!   P18/P19  ◄──── vo2631::Vo2631 (U6) ◄── nsi50010::Nsi50010 (IC9) ◄── end switch
//! ```
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
//! # A chain, not a mesh
//!
//! Two parts here ([`nsi50010`], [`vo2631`]) live in a **series current loop**,
//! and the board engine cannot solve one. Every driver in the net model is a
//! Thevenin source (`BOARD_ENGINE.md`, "Net state model") and a
//! [`embsim_board::Component`] has no way to contribute a resistive edge
//! *between two of its own pins* — that is the "transducer components
//! contribute parameterized primitives" slice, which is described in the
//! design doc and not built. A two-terminal element modeled as a drive on
//! *both* terminals would make each terminal's source depend on the other's
//! solved voltage: a fixed point the engine would chase one resolution per
//! iteration, which is exactly the event cost this module exists to avoid.
//!
//! So each two-terminal part drives **one** terminal — the one facing the rest
//! of the branch — from the terminal that faces its own stiff end:
//!
//! ```text
//!   rail ─── NSI50010 ─── (shared node) ─── VO2631 LED ─── return
//!            drives ────────►             ◄──────── drives
//!            (from its anode)              (from its cathode)
//! ```
//!
//! Neither source depends on the node it drives, so the solve is one pass and
//! the shared node's voltage is what tells the regulator how much overhead it
//! has. What this buys is the question the bench actually asks — *is the loop
//! closed?* — answered electrically rather than by fiat. What it costs is
//! stated per part: the modeled branch current is the one the resistive
//! equivalent carries, not the one the regulator would hold.

pub mod iso67xx;
pub mod npn_switch;
pub mod nsi50010;
pub mod vo2631;

pub use iso67xx::{Channel, Iso67xx, Iso67xxMonitor, Side, Variant};
pub use npn_switch::{NpnSwitch, NpnSwitchMonitor};
pub use nsi50010::{Nsi50010, Nsi50010Regulator};
pub use vo2631::{OptoChannel, Vo2631, Vo2631Monitor};

use std::fmt;

use embsim_board::{Level, NetState, TheveninDrive, Volts};

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
    /// A channel was named that the configured device variant does not have.
    NoSuchChannel {
        /// Variant name as spelled in [`iso67xx::Variant`].
        variant: &'static str,
        /// The channel that was asked for.
        channel: &'static str,
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
            PartConfigError::NoSuchChannel { variant, channel } => {
                write!(f, "{variant} has no channel {channel}")
            }
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

/// Whether a sensed supply net is up, against a part's own minimum.
///
/// The projection matches [`crate::ads122u04_component`]'s `supply_ok`, which
/// is the reference for supply gating in this workspace: a numeric solve is
/// compared to the minimum; a rail known only as a digital projection (an
/// unmodeled-voltage [`embsim_board::PinKind::PowerOut`] presenting
/// `Pulled(High)`) counts as up; a rail held low, floating, or fought over
/// does not.
///
/// The engine never invents a value for an unsourced node, so a *floating*
/// supply is a down supply — which is exactly the failure a system description
/// that forgot a rail should get.
pub fn supply_up(state: NetState, min_volts: Volts) -> bool {
    match state {
        NetState::Analog(volts) => volts >= min_volts,
        NetState::Driven(Level::High) | NetState::Pulled(Level::High, _) => true,
        NetState::Driven(Level::Low) | NetState::Pulled(Level::Low, _) => false,
        NetState::Floating | NetState::Contention => false,
    }
}

/// The voltage a supply rail is at, or `nominal` when the engine has only a
/// digital projection of it.
///
/// `Driven`/`Pulled` carry a level, not volts; an output driven "to the rail"
/// against such a supply has to pick some number, and the part's configured
/// nominal is the honest one to pick (and to say so).
pub fn rail_volts(state: NetState, nominal: Volts) -> Volts {
    match state {
        NetState::Analog(volts) if volts.is_finite() => volts,
        _ => nominal,
    }
}

// ============================================================
// Input projection
// ============================================================

/// Project a sensed net onto a logic level against an **asymmetric**
/// `V_IL`/`V_IH` pair, returning `None` inside the dead band.
///
/// [`crate::machine::digital_level`] takes one threshold because a machine
/// part has no logic family; a real interface IC specifies two, as fractions
/// of its own input supply, with a dead band between them where the datasheet
/// promises nothing. This helper keeps that promise: a node voltage inside the
/// band returns `None`, and each part chooses its documented behavior (the
/// isolators fall to their default output state, which is what the TI function
/// table says an *indeterminate* input does).
///
/// `Floating` and `Contention` are `None` for the same reason they are in
/// [`crate::machine::digital_level`] — the engine never invents a value for an
/// unsourced or fought-over node, and neither does a part model.
///
/// Projecting the dead band as `None` rather than as
/// [`embsim_board::Finding::AmbiguousLevel`] is deliberate: that finding is an
/// engine-side slice (`BOARD_ENGINE.md`, deferred features), so the part
/// resolves the ambiguity locally and documents how.
pub fn threshold_level(state: NetState, vil: Volts, vih: Volts) -> Option<Level> {
    match state {
        NetState::Driven(level) | NetState::Pulled(level, _) => Some(level),
        NetState::Analog(volts) if volts.is_nan() => None,
        NetState::Analog(volts) if volts >= vih => Some(Level::High),
        NetState::Analog(volts) if volts <= vil => Some(Level::Low),
        NetState::Analog(_) => None,
        NetState::Floating | NetState::Contention => None,
    }
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

    #[rstest]
    #[case::analog_above(NetState::Analog(3.3), true)]
    #[case::analog_at_minimum(NetState::Analog(1.71), true)]
    #[case::analog_below(NetState::Analog(1.0), false)]
    #[case::analog_zero(NetState::Analog(0.0), false)]
    #[case::driven_high(NetState::Driven(Level::High), true)]
    #[case::driven_low(NetState::Driven(Level::Low), false)]
    #[case::pulled_high(NetState::Pulled(Level::High, 10_000.0), true)]
    #[case::pulled_low(NetState::Pulled(Level::Low, 10_000.0), false)]
    #[case::floating(NetState::Floating, false)]
    #[case::contention(NetState::Contention, false)]
    fn supply_up_matches_the_ads122u04_projection(#[case] state: NetState, #[case] expect: bool) {
        assert_eq!(supply_up(state, 1.71), expect);
    }

    #[rstest]
    #[case::solved(NetState::Analog(5.0), 5.0)]
    #[case::projected_high(NetState::Driven(Level::High), 3.3)]
    #[case::floating(NetState::Floating, 3.3)]
    #[case::nan(NetState::Analog(f64::NAN), 3.3)]
    fn rail_volts_falls_back_to_the_nominal(#[case] state: NetState, #[case] expect: Volts) {
        assert!((rail_volts(state, 3.3) - expect).abs() < 1e-12);
    }

    /// The dead band between `V_IL` and `V_IH` is `None`, not a guess.
    #[rstest]
    #[case::high(NetState::Analog(3.0), Some(Level::High))]
    #[case::at_vih(NetState::Analog(2.31), Some(Level::High))]
    #[case::dead_band(NetState::Analog(1.65), None)]
    #[case::at_vil(NetState::Analog(0.99), Some(Level::Low))]
    #[case::low(NetState::Analog(0.0), Some(Level::Low))]
    #[case::driven(NetState::Driven(Level::High), Some(Level::High))]
    #[case::pulled(NetState::Pulled(Level::Low, 4_700.0), Some(Level::Low))]
    #[case::floating(NetState::Floating, None)]
    #[case::contention(NetState::Contention, None)]
    #[case::nan(NetState::Analog(f64::NAN), None)]
    fn threshold_level_refuses_the_dead_band(
        #[case] state: NetState,
        #[case] expect: Option<Level>,
    ) {
        // 0.3 x 3.3 = 0.99, 0.7 x 3.3 = 2.31 (SLLSFJ6G section 7.3).
        assert_eq!(threshold_level(state, 0.99, 2.31), expect);
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
        assert!(PartConfigError::NoSuchChannel {
            variant: "Iso6721",
            channel: "D"
        }
        .to_string()
        .contains("Iso6721"));
    }
}
