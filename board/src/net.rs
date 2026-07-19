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

/// Series-resistance collapse threshold: series passives whose accumulated
/// resistance stays below this value collapse into a serial link route, and
/// disagreeing push-pull drivers coupled below it resolve to
/// [`NetState::Contention`] rather than a divided voltage.
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
    use rstest::rstest;

    use super::*;

    #[rstest]
    #[case::same_pin("U1", "3", "U1", "3", true)]
    #[case::diff_pin("U1", "3", "U1", "4", false)]
    #[case::diff_ref("U1", "3", "U2", "3", false)]
    fn pin_ref_equality_is_structural(
        #[case] r1: &str,
        #[case] p1: &str,
        #[case] r2: &str,
        #[case] p2: &str,
        #[case] eq: bool,
    ) {
        let a = PinRef::new(r1, p1);
        let b = PinRef::new(r2, p2);
        assert_eq!(a == b, eq);
    }

    /// Net-state identity matrix — digital projections must not collapse into
    /// each other under `PartialEq` (so diagnostics and tests can distinguish
    /// Driven vs Pulled vs Contention vs Analog vs Floating).
    #[rstest]
    #[case::driven_h(NetState::Driven(Level::High))]
    #[case::driven_l(NetState::Driven(Level::Low))]
    #[case::pulled_h(NetState::Pulled(Level::High, 4_700.0))]
    #[case::pulled_l(NetState::Pulled(Level::Low, 10_000.0))]
    #[case::floating(NetState::Floating)]
    #[case::contention(NetState::Contention)]
    #[case::analog(NetState::Analog(1.65))]
    fn net_state_equals_self(#[case] state: NetState) {
        let copy = state;
        assert_eq!(state, copy);
    }

    #[rstest]
    #[case::driven_vs_pulled(NetState::Driven(Level::High), NetState::Pulled(Level::High, 4_700.0))]
    #[case::driven_h_vs_l(NetState::Driven(Level::High), NetState::Driven(Level::Low))]
    #[case::pulled_impedance(
        NetState::Pulled(Level::High, 1_000.0),
        NetState::Pulled(Level::High, 4_700.0)
    )]
    #[case::floating_vs_contention(NetState::Floating, NetState::Contention)]
    #[case::analog_vs_driven(NetState::Analog(3.3), NetState::Driven(Level::High))]
    fn net_state_projections_are_distinct(#[case] a: NetState, #[case] b: NetState) {
        assert_ne!(a, b);
    }

    #[rstest]
    #[case::default_pp(DEFAULT_PUSH_PULL_IMPEDANCE, 25.0)]
    #[case::stream_collapse(STREAM_COLLAPSE_THRESHOLD, 1_000.0)]
    #[case::escalation(ESCALATION_IMPEDANCE_RATIO, 10.0)]
    fn published_thresholds_match_design_doc(#[case] actual: f64, #[case] expected: f64) {
        assert!((actual - expected).abs() < f64::EPSILON);
    }

    #[rstest]
    fn thevenin_drive_is_copy_eq() {
        let d = TheveninDrive {
            volts: 3.3,
            impedance: DEFAULT_PUSH_PULL_IMPEDANCE,
        };
        let d2 = d;
        assert_eq!(d, d2);
    }

    #[rstest]
    fn net_struct_holds_membership_and_state() {
        let net = Net {
            id: NetId(0),
            name: "NET".into(),
            nodes: vec![PinRef::new("U1", "1"), PinRef::new("R1", "1")],
            state: NetState::Floating,
        };
        assert_eq!(net.nodes.len(), 2);
        assert_eq!(net.state, NetState::Floating);
    }
}
