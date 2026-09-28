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

/// Current in amperes (a [`crate::Drive::Current`] injection, a branch
/// current).
pub type Amps = f64;

/// Default push-pull digital drive impedance (overridable per
/// [`crate::component::PinDecl::drive_impedance`]).
pub const DEFAULT_PUSH_PULL_IMPEDANCE: Ohms = 25.0;

/// Rail a push-pull digital output drives High at.
///
/// One value for the whole crate rather than a per-part knob: a component on
/// another rail models the level shifter *as a component*, which is the same
/// answer `mcu` has always given for its GPIO outputs.
pub const LOGIC_HIGH_VOLTS: Volts = 3.3;

/// Threshold a digital input applies to a numerically solved net. Matches the
/// engine's own digital projection.
pub const LOGIC_THRESHOLD_VOLTS: Volts = 1.5;

/// Series-resistance collapse threshold: series passives whose accumulated
/// resistance stays below this value collapse into a pulse route — a step
/// clock crosses them, an isolation resistor stops it. The same value is the
/// weak-drive boundary of the projection ([`WEAK_DRIVE_OHMS`]).
pub const STREAM_COLLAPSE_THRESHOLD: Ohms = 1_000.0;

/// Impedance-escalation ratio: among the sources reaching one node, a source
/// this many times weaker (by total ohms) than the strongest loses to it —
/// the node takes the strongest's level and the fight is a
/// [`crate::Finding::Contention`]; sources closer than this disagree
/// *numerically*, and the node escalates to the cluster solver for its
/// divided voltage (`NODES.md` "Three rules the taxonomy rests on", rule 2).
pub const ESCALATION_IMPEDANCE_RATIO: f64 = 10.0;

/// Weak-drive boundary: a source whose **total** ohms — its own impedance
/// plus the series path to the node it reaches — is at or above this value
/// is a *pull*. A pull sets a node's level only when nothing stronger
/// reaches it, and it never contends: a 15 kΩ pad against a 30 Ω sink is a
/// pull-up losing to a driver, and a 10.5 kΩ pull-up against a 25 Ω pad is
/// the pad's node. One value with [`STREAM_COLLAPSE_THRESHOLD`] by design:
/// the resistance below which two nets are one node for signalling is the
/// resistance below which a source is a driver of that node.
pub const WEAK_DRIVE_OHMS: Ohms = STREAM_COLLAPSE_THRESHOLD;

/// AC-coupling margin: a rate crosses a coupling capacitor only while the
/// capacitor's reactance at that rate, `1/(2π·f·C)`, is at most the far
/// node's resistance divided by this — the same factor of ten that decides
/// a source has lost ([`ESCALATION_IMPEDANCE_RATIO`]), applied to the
/// impedance divider a series capacitor forms with the node it feeds. A
/// crossing that fails is [`crate::Finding::PulseNotCoupled`] and the train
/// stops at the capacitor.
///
/// The physical bound is the divider's −3 dB point, `X_C ≤ R_far` (a ratio
/// of 1): there `|H| = R / √(R² + X_C²)` is 0.71 and a 3.3 V swing arrives
/// as 2.3 V, above any LVCMOS hysteresis band. Ten is embsim's conservative
/// constant, borrowed from the strength ranking rather than derived — at
/// this margin a 10 pF capacitor into 1 kΩ at 20 MHz (`X_C` = 796 Ω,
/// `|H|` ≈ 0.78, 2.6 V of swing) is refused although a Schmitt input would
/// pass it. Phase 5 (`NODES.md` §8) may derive the margin from the
/// receiver's declared thresholds instead (pass fraction ≥ hysteresis /
/// `V_pp`); until then the number is a stated choice, not a measurement.
pub const COUPLING_REACTANCE_RATIO: f64 = ESCALATION_IMPEDANCE_RATIO;

/// Upper bound of a valid logic low at a 3.3 V LVCMOS input, `V_IL(max)`:
/// JEDEC JESD8C.01 (3.3 V LVCMOS interface standard), DC input
/// specifications, `V_IL` max = 0.8 V. With [`V_IH`] it bounds the dead
/// band a solved node voltage is projected through: a divided voltage at or
/// below this is a low, strictly between the two is
/// [`NetState::Contention`] with a [`crate::Finding::AmbiguousLevel`].
pub const V_IL: Volts = 0.8;

/// Lower bound of a valid logic high at a 3.3 V LVCMOS input, `V_IH(min)`:
/// JEDEC JESD8C.01, DC input specifications, `V_IH` min = 2.0 V. See
/// [`V_IL`].
pub const V_IH: Volts = 2.0;

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
    /// A strong push-pull source (under [`WEAK_DRIVE_OHMS`]) on the node
    /// itself wins it.
    Driven(Level),
    /// The winning source reaches the node through resistance: its level,
    /// and the ohms of the winner's series path (a weak pad's own impedance
    /// included — a 15 kΩ pad is a resistor to its rail).
    Pulled(Level, Ohms),
    /// None of the above projections apply — raw node voltage: an ideal
    /// source on the node, a solved operating point, a divided voltage that
    /// is a valid level.
    Analog(Volts),
    /// Sources of comparable strength disagree and the voltage they fight
    /// to lies strictly inside the [`V_IL`]/[`V_IH`] dead band: neither
    /// level. Always beside a [`crate::Finding::Contention`].
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

// ============================================================
// Digital projection
// ============================================================

/// The logic level a resolved net presents, or `None` when the engine refuses
/// to give one.
///
/// A floating or contended net has no level, and nothing downstream may invent
/// one: an input with no level holds whatever value its owner last saw. That
/// refusal is the whole reason `Floating` and `Contention` are states rather
/// than a defaulted `Low`.
pub fn level_of(state: NetState) -> Option<Level> {
    match state {
        NetState::Driven(level) | NetState::Pulled(level, _) => Some(level),
        NetState::Analog(volts) => Some(if volts >= LOGIC_THRESHOLD_VOLTS {
            Level::High
        } else {
            Level::Low
        }),
        NetState::Floating | NetState::Contention => None,
    }
}

/// The Thevenin contribution of a push-pull digital output at `level`.
pub fn digital_drive(level: Level) -> TheveninDrive {
    TheveninDrive {
        volts: match level {
            Level::High => LOGIC_HIGH_VOLTS,
            Level::Low => 0.0,
        },
        impedance: DEFAULT_PUSH_PULL_IMPEDANCE,
    }
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

    /// A digital input projects exactly what the engine will commit to, and
    /// refuses to invent a level where the engine gave none.
    #[rstest]
    #[case::driven_high(NetState::Driven(Level::High), Some(Level::High))]
    #[case::driven_low(NetState::Driven(Level::Low), Some(Level::Low))]
    #[case::pulled(NetState::Pulled(Level::High, 10_000.0), Some(Level::High))]
    #[case::analog_above(NetState::Analog(3.0), Some(Level::High))]
    #[case::analog_below(NetState::Analog(0.4), Some(Level::Low))]
    #[case::floating(NetState::Floating, None)]
    #[case::contention(NetState::Contention, None)]
    fn input_projection_never_invents_a_level(
        #[case] state: NetState,
        #[case] expect: Option<Level>,
    ) {
        assert_eq!(level_of(state), expect);
    }

    /// A push-pull output drives the rail, or ground, at the default strength.
    #[rstest]
    fn a_digital_output_drives_the_rail() {
        assert_eq!(
            digital_drive(Level::High),
            TheveninDrive {
                volts: LOGIC_HIGH_VOLTS,
                impedance: DEFAULT_PUSH_PULL_IMPEDANCE
            }
        );
        assert_eq!(
            digital_drive(Level::Low),
            TheveninDrive {
                volts: 0.0,
                impedance: DEFAULT_PUSH_PULL_IMPEDANCE
            }
        );
    }

    #[rstest]
    #[case::default_pp(DEFAULT_PUSH_PULL_IMPEDANCE, 25.0)]
    #[case::stream_collapse(STREAM_COLLAPSE_THRESHOLD, 1_000.0)]
    #[case::escalation(ESCALATION_IMPEDANCE_RATIO, 10.0)]
    #[case::weak_drive(WEAK_DRIVE_OHMS, 1_000.0)]
    #[case::coupling(COUPLING_REACTANCE_RATIO, 10.0)]
    #[case::v_il(V_IL, 0.8)]
    #[case::v_ih(V_IH, 2.0)]
    fn published_thresholds_match_design_doc(#[case] actual: f64, #[case] expected: f64) {
        assert!((actual - expected).abs() < f64::EPSILON);
    }

    /// The dead band brackets the digital threshold: its low edge projects
    /// low, its high edge high, and the threshold itself lies between them —
    /// so a solved voltage the digital projection calls high can still be
    /// an ambiguous level.
    #[rstest]
    #[case::low_edge(V_IL, Level::Low)]
    #[case::threshold(LOGIC_THRESHOLD_VOLTS, Level::High)]
    #[case::high_edge(V_IH, Level::High)]
    fn the_dead_band_brackets_the_logic_threshold(#[case] volts: Volts, #[case] expect: Level) {
        assert_eq!(level_of(NetState::Analog(volts)), Some(expect));
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
