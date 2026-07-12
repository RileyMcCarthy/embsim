//! Net state model — one mechanism, digital as a projection.
//!
//! Every driver is a Thevenin source (voltage + impedance); nets connected
//! through passives form clusters (see [`crate::cluster`]); resolution always
//! happens at cluster granularity, and the familiar digital states are a
//! **derived view** of the solved node voltage — not a parallel mechanism, so
//! "a pull-up is just a resistor" causes no ambiguity.
//!
//! This module owns the shared identity/value types the rest of the crate
//! builds on: [`NetId`], [`PinRef`], [`NetState`], [`TheveninDrive`].

// ============================================================
// Units
// ============================================================

/// Resistance in ohms.
pub type Ohms = f64;

/// Voltage in volts.
pub type Volts = f64;

/// Default push-pull digital drive impedance (overridable per
/// [`crate::component::PinDecl::drive_impedance`]).
pub const DEFAULT_PUSH_PULL_IMPEDANCE: Ohms = 25.0;

/// Series-resistance collapse threshold for stream routing: series passives
/// below this value collapse into a serial link route.
pub const STREAM_COLLAPSE_THRESHOLD: Ohms = 1_000.0;

/// Impedance-escalation ratio: a competing path whose Thevenin impedance is
/// within this factor of the strongest driver's escalates the net to the
/// cluster solver instead of the digital short-circuit path.
pub const ESCALATION_IMPEDANCE_RATIO: f64 = 10.0;

// ============================================================
// Identity
// ============================================================

/// Index of a resolved net within a built board/system (dense, build-assigned).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct NetId(pub usize);

/// One component pin as named by the netlist: `(reference, pin number)`.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct PinRef {
    /// Component reference designator (`"U1"`).
    pub reference: String,
    /// Netlist pin number (`"3"`).
    pub pin: String,
}

impl PinRef {
    /// Convenience constructor.
    pub fn new(reference: impl Into<String>, pin: impl Into<String>) -> Self {
        Self {
            reference: reference.into(),
            pin: pin.into(),
        }
    }
}

// ============================================================
// State
// ============================================================

/// Logic level of a rail-adjacent net.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Level {
    /// At or near the low rail.
    Low,
    /// At or near the high rail.
    High,
}

/// Resolved state of a net — the digital variants are projections of the
/// solved node voltage, never a parallel mechanism.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum NetState {
    /// No source reaches this node (MNA singular for the node).
    Floating,
    /// Solved V within `V_OL`/`V_OH` of a rail, dominated by one push-pull source.
    Driven(Level),
    /// Rail-adjacent V dominated by a resistive path of the given Thevenin impedance.
    Pulled(Level, Ohms),
    /// None of the above projections apply — raw node voltage.
    Analog(Volts),
    /// ≥ 2 push-pull sources fighting (directly or through collapsed
    /// low-value series resistance).
    Contention,
}

/// A Thevenin drive contribution from one pin: source voltage + impedance.
///
/// Push-pull digital drivers default to [`DEFAULT_PUSH_PULL_IMPEDANCE`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TheveninDrive {
    /// Open-circuit source voltage.
    pub volts: Volts,
    /// Source impedance.
    pub impedance: Ohms,
}

// ============================================================
// Resolved net
// ============================================================

/// A resolved net in a built board/system: identity, membership, and the
/// state assigned by the most recent resolution pass.
#[derive(Debug, Clone, PartialEq)]
pub struct Net {
    /// Build-assigned dense index.
    pub id: NetId,
    /// Netlist net name (`"AIN0"`); harness-merged nets keep a joined name.
    pub name: String,
    /// Member pins.
    pub nodes: Vec<PinRef>,
    /// State from the most recent resolution pass (build-time pass included).
    pub state: NetState,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pin_ref_equality_is_structural() {
        assert_eq!(PinRef::new("U1", "3"), PinRef::new("U1", "3"));
        assert_ne!(PinRef::new("U1", "3"), PinRef::new("U1", "4"));
    }

    #[test]
    fn net_state_projections_compare() {
        assert_eq!(NetState::Driven(Level::High), NetState::Driven(Level::High));
        assert_ne!(
            NetState::Driven(Level::High),
            NetState::Pulled(Level::High, 4_700.0)
        );
    }
}
