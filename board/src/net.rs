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
//!
//! A step clock or an oscillator is one more drive, not a second channel:
//! a [`crate::Drive::Periodic`] resolves phase by phase through the same
//! rule-2 ranking, and its net publishes [`NetState::Periodic`] — one state
//! per constant-rate segment, never one per edge (`NODES.md` §10 row 3;
//! `DESIGN.md` rule 2, "no second channel").

pub use embsim_peripherals::pulse_out::PeriodicSchedule;

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

/// Default push-pull digital drive impedance: what a push-pull output idles
/// behind ([`crate::component::PinDecl::digital_out`]), overridable per pin
/// ([`crate::component::PinDecl::with_impedance`]).
pub const DEFAULT_PUSH_PULL_IMPEDANCE: Ohms = 25.0;

/// Rail a push-pull digital output drives High at.
///
/// One value for the whole crate rather than a per-part knob: a component on
/// another rail models the level shifter *as a component*, which is the same
/// answer `mcu` has always given for its GPIO outputs.
pub const LOGIC_HIGH_VOLTS: Volts = 3.3;

/// The engine's own single split of a solved voltage into a level for its
/// report ([`level_of`] on [`NetState::Analog`]): mid-rail of the 3.3 V
/// logic rail. A receiver never reads through it — a node projects its own
/// [`crate::Sense`] through its declared thresholds.
pub const LOGIC_THRESHOLD_VOLTS: Volts = 1.5;

/// The coupled reach: how far a periodic drive reaches across a coupling
/// capacitor — the conduction resistance a coupled rate
/// may accumulate, on either side of the capacitors it crosses, before the
/// node it arrives at is no longer the source's for signalling (the
/// engine's AC reach, `Resolver::ensure_reach`). The same value is the
/// weak-drive boundary of the projection ([`WEAK_DRIVE_OHMS`]).
pub const COUPLED_REACH_OHMS: Ohms = 1_000.0;

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
/// the pad's node. One value with [`COUPLED_REACH_OHMS`] by design:
/// the resistance below which two nets are one node for signalling is the
/// resistance below which a source is a driver of that node.
pub const WEAK_DRIVE_OHMS: Ohms = COUPLED_REACH_OHMS;

/// AC-coupling margin: a rate crosses a coupling capacitor only while the
/// capacitor's reactance at that rate, `1/(2π·f·C)`, is at most the far
/// node's resistance divided by this — the same factor of ten that decides
/// a source has lost ([`ESCALATION_IMPEDANCE_RATIO`]), applied to the
/// impedance divider a series capacitor forms with the node it feeds. A
/// crossing that fails is [`crate::Finding::PeriodicNotCoupled`] and the rate
/// stops at the capacitor.
///
/// The physical bound is the divider's −3 dB point, `X_C ≤ R_far` (a ratio
/// of 1): there `|H| = R / √(R² + X_C²)` is 0.71 and a 3.3 V swing arrives
/// as 2.3 V, above any LVCMOS hysteresis band. Ten is embsim's conservative
/// constant, borrowed from the strength ranking rather than derived — at
/// this margin a 10 pF capacitor into 1 kΩ at 20 MHz (`X_C` = 796 Ω,
/// `|H|` ≈ 0.78, 2.6 V of swing) is refused although a Schmitt input would
/// pass it. The capacitor work (`NODES.md` §12 item 6, "Capacitors") may
/// derive the margin from the receiver's declared thresholds instead (pass
/// fraction ≥ hysteresis / `V_pp`); until then the number is a stated
/// choice, not a measurement.
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
    /// level. Always beside a [`crate::Finding::Contention`]. A periodic
    /// drive fought by a comparable source — for half of every cycle — or
    /// two periodic drives on one root are this too.
    Contention,
    /// A square wave: the level the node takes in a [`crate::Drive::Periodic`]'s
    /// high phase, the level in its low phase, and the integer schedule that
    /// says when each phase is — one state for a whole constant-rate segment,
    /// so a step clock costs one publish per rate change and not one per
    /// edge (`NODES.md` §10 row 3). The levels are the engine's report of
    /// each phase — its own rule-2 projection through the net-level pair,
    /// one level for both where the swing sits inside one of its bands (a
    /// receiver projects the phase voltages it is handed through its own
    /// thresholds, [`crate::Sense::level`]); a coupled node — reached
    /// across a capacitor by the AC rule — carries the source's own phase
    /// levels, the swing a capacitor passes undivided, not the DC bias it
    /// blocks. A periodic node has no single level in the report
    /// ([`level_of`] → `None`) and, running, no single operating point; a
    /// consumer that needs the count integrates the segment itself
    /// ([`PeriodicSchedule::emitted_at_ns`], the same integer arithmetic the pulse
    /// peripheral hands the firmware). A segment whose rate is zero is a
    /// held clock: the state still carries its final count, and the node
    /// rests at its low phase's voltage, which is what a sensing pin is
    /// handed as its DC voltage.
    Periodic {
        /// The node's level in the drive's high phase.
        hi: Level,
        /// The node's level in the drive's low phase.
        lo: Level,
        /// The rate, count, ceiling and anchor — compared by identity
        /// (anchor included) by the sense change gate, so a segment is
        /// delivered once and never re-delivered as time passes.
        segment: PeriodicSchedule,
    },
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
    /// The voltage the same pass resolved the net to — what a sensing pin
    /// is handed, measured against its reference ([`crate::Sense`]); the
    /// engine's own report stays [`Self::state`].
    pub(crate) volts: NetVolts,
}

/// The voltage a resolved net sits at, beside its [`NetState`]: what the
/// engine hands a sensing pin (`NODES.md` §10, "Delivered to a sensing
/// pin"; [`crate::Sense`]), in the engine's frame — volts against the one
/// 0 V every published voltage is in, which a pin's sense then measures
/// against the pin's own reference. The engine's report form stays
/// [`NetState`]: this is not a second report, it is the number a
/// [`NetState::Driven`] or [`NetState::Pulled`] projection keeps and does
/// not print.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub(crate) struct NetVolts {
    /// The node's voltage at this instant: the winning source's
    /// open-circuit voltage where rule 2 projected the node (a pull's rail,
    /// a pad's drive), the solved voltage where its cluster solved — a
    /// fight's operating point included, the voltage the
    /// [`crate::Finding::AmbiguousLevel`] beside it names — and `None`
    /// where no source names one: a floating node, a node only an
    /// unmodelled rail reaches (it names no voltage — `DESIGN.md` rule 6),
    /// a periodic node (see [`Self::phases`]), and a node with two
    /// operating points (a clock fought for half of every cycle, two rates
    /// meeting).
    pub(crate) dc: Option<Volts>,
    /// A [`NetState::Periodic`] node's voltage in the drive's high phase
    /// and in its low phase, each as [`Self::dc`] says for that phase's
    /// resolution — for a node reached across a coupling capacitor, the
    /// source's two port voltages, the swing the capacitor passes.
    pub(crate) phases: Option<(Option<Volts>, Option<Volts>)>,
}

impl NetVolts {
    /// A node at one voltage, or at none.
    pub(crate) const fn dc(volts: Option<Volts>) -> Self {
        Self {
            dc: volts,
            phases: None,
        }
    }
}

// ============================================================
// Digital projection
// ============================================================

/// The logic level a resolved net presents in the **engine's own report**, or
/// `None` when the engine refuses to give one — the projection its findings,
/// its event log and its periodic states' phase levels use (the JESD8C.01
/// pair's dead band decides `Contention`; [`LOGIC_THRESHOLD_VOLTS`] splits an
/// `Analog` state). A node never reads through it: it projects its own
/// [`crate::Sense`] through its declared thresholds ([`crate::Sense::level`]).
///
/// A floating or contended net has no level, and nothing may invent one.
/// That refusal is the whole reason `Floating` and `Contention` are states
/// and not a defaulted `Low`. A periodic net has two levels and no single
/// one: the rate is not edges, and a consumer that wants the clock reads the
/// segment.
pub fn level_of(state: NetState) -> Option<Level> {
    match state {
        NetState::Driven(level) | NetState::Pulled(level, _) => Some(level),
        NetState::Analog(volts) => Some(if volts >= LOGIC_THRESHOLD_VOLTS {
            Level::High
        } else {
            Level::Low
        }),
        NetState::Floating | NetState::Contention | NetState::Periodic { .. } => None,
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

    /// A periodic state swinging rail to rail at `freq_hz` from `since_ns`.
    fn periodic(freq_hz: u32, since_ns: u64) -> NetState {
        NetState::Periodic {
            hi: Level::High,
            lo: Level::Low,
            segment: PeriodicSchedule {
                emitted: 0,
                freq_hz,
                total: None,
                since_ns,
            },
        }
    }

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
    #[case::periodic(periodic(8_192, 1_000))]
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
    #[case::periodic_vs_driven(periodic(8_192, 0), NetState::Driven(Level::High))]
    #[case::periodic_rate(periodic(8_192, 0), periodic(16_384, 0))]
    #[case::periodic_anchor(periodic(8_192, 0), periodic(8_192, 1))]
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
    #[case::periodic(periodic(8_192, 0), None)]
    #[case::held_periodic(periodic(0, 0), None)]
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
    #[case::coupled_reach(COUPLED_REACH_OHMS, 1_000.0)]
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
            volts: NetVolts::default(),
        };
        assert_eq!(net.nodes.len(), 2);
        assert_eq!(net.state, NetState::Floating);
    }
}
