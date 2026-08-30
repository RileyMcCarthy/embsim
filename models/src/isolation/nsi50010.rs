//! Model: the **onsemi NSI50010Y** two-terminal constant-current regulator —
//! the part that lights an optocoupler's LED from an end-switch loop.
//!
//! ```text
//!   rail ──┤A   NSI50010   K├──── (shared node) ──── opto LED ──── switch ──── return
//!                  │
//!                  └─ regulates the loop current to Ireg(SS), and reports it
//! ```
//!
//! # Datasheet provenance
//!
//! **onsemi NSI50010Y/D, Rev. 3 (September 2025)** — "Constant Current
//! Regulator & LED Driver, 50 V, 10 mA ± 30%, 460 mW". Governs:
//!
//! - **The package and terminal map**: SOD-123 (CASE 425, STYLE 1), a
//!   two-terminal device with **pin 1 = Cathode, pin 2 = Anode**. That is the
//!   facade [`NSI50010_PINS`] declares, and it matches the MaD EdgeBoard
//!   netlist's own `pinfunction` labels for `IC9` (`1` = `K`, `2` = `A`).
//! - **The regulation current**: Electrical Characteristics,
//!   `Ireg(SS)` = 7.0 / **10** / 13 mA at `Vak = 7.5 V`.
//!   [`DEFAULT_REGULATION_MA`], [`DEFAULT_REGULATION_VAK_VOLTS`].
//! - **The turn-on curve**: Features / description — "The CCR turns on
//!   immediately and is at **40% of regulation with only 0.5 V Vak**" — and
//!   Electrical Characteristics `Voverhead` = **1.8 V** typical, note 2:
//!   "Voverhead is typical value for **80%** Ireg(SS)". Those two points plus
//!   the 100% point at 7.5 V are [`TURN_ON_CURVE`].
//! - **Reverse blocking**: Maximum Ratings, `VR` = 500 mV. A reverse-biased
//!   CCR is not a conductor, so the reported current is zero for `Vak <= 0`.
//!
//! # Why this is a Thevenin element and not a current source
//!
//! The board engine has no current-source primitive: every driver is a
//! Thevenin source (`BOARD_ENGINE.md`, "Net state model"), and an ideal
//! current source has no finite Thevenin equivalent. So the regulator is
//! presented to the engine as its **large-signal Thevenin equivalent at the
//! datasheet's own operating point**:
//!
//! ```text
//!   R = Vak / Ireg(SS) = 7.5 V / 10 mA = 750 Ohm
//! ```
//!
//! and it drives its **cathode** from its sensed **anode** through that
//! resistance. Driving one terminal from the other is what keeps the solve to
//! a single pass — see the module docs of [`crate::isolation`], "A chain, not
//! a mesh".
//!
//! The **regulated** current, which is what an LED consumer wants, is computed
//! separately at read time from the solved `Vak` against [`TURN_ON_CURVE`] and
//! clamped at `Ireg(SS)`. That is [`Nsi50010Regulator::current_ma`], and it is
//! deliberately *not* fed back into the node solve.
//!
//! ## Deliberate simplifications (not modeled)
//!
//! - **The two views disagree away from the operating point.** The node
//!   voltages are those of a 750 Ohm resistor; the reported current is the
//!   regulator's. At `Vak = 7.5 V` they agree exactly (10 mA); below about
//!   2 V of overhead the resistive equivalent carries less than the regulator
//!   would, and above 7.5 V it would carry more while the report stays
//!   clamped. Making them agree needs a current-source primitive in the
//!   cluster solver, which the design doc does not have.
//! - **Reverse conduction.** The reported current is zero below `Vak = 0`, but
//!   the modeled Thevenin element is symmetric and will still conduct
//!   backwards through its 750 Ohm if a system description reverse-biases it.
//!   The real part blocks (`VR` = 500 mV).
//! - **The negative temperature coefficient**, the ±30% part-to-part spread
//!   (`Ireg(SS)` 7.0..13 mA), self-heating, the 460 mW / 208 mW dissipation
//!   limits and the 50 V `Vak` maximum. One nominal current, no derating, no
//!   damage.
//! - **Pulse current** (`Ireg(P)` = 10.5 mA typical) and the 2.5 pF terminal
//!   capacitance: this model is quasi-static, like the engine.

use std::sync::{Arc, Mutex};

use embsim_board::{
    AttachError, Component, ComponentNetIo, NetState, Ohms, PinDecl, PinHandle, PinKind,
    TheveninDrive, Volts,
};
use embsim_core::event::Observers;

use super::{require_positive, PartConfigError};

// ============================================================
// Datasheet constants
// ============================================================

/// Steady-state regulation current, typical: `Ireg(SS)` = 10 mA
/// (NSI50010Y/D Rev. 3, Electrical Characteristics).
pub const DEFAULT_REGULATION_MA: f64 = 10.0;

/// The `Vak` that regulation current is specified at: 7.5 V
/// (NSI50010Y/D Rev. 3, Electrical Characteristics).
pub const DEFAULT_REGULATION_VAK_VOLTS: Volts = 7.5;

/// The turn-on curve, as `(Vak volts, fraction of Ireg(SS))`, interpolated
/// linearly between the points and flat above the last one.
///
/// Every point is the datasheet's:
///
/// | `Vak` | Fraction | Source |
/// |---|---|---|
/// | 0.0 V | 0 % | a two-terminal device with no bias carries nothing |
/// | 0.5 V | 40 % | "at 40% of regulation with only 0.5 V Vak" (Features) |
/// | 1.8 V | 80 % | `Voverhead` typ, note 2: "typical value for 80% Ireg(SS)" |
/// | 7.5 V | 100 % | the `Ireg(SS)` test condition |
pub const TURN_ON_CURVE: [(Volts, f64); 4] = [(0.0, 0.0), (0.5, 0.40), (1.8, 0.80), (7.5, 1.00)];

/// Default series Thevenin impedance: `Vak / Ireg(SS)` = 7.5 V / 10 mA.
pub const DEFAULT_SERIES_IMPEDANCE_OHMS: Ohms =
    DEFAULT_REGULATION_VAK_VOLTS / (DEFAULT_REGULATION_MA * 1e-3);

/// Smallest change in reported current (mA) that publishes an observer event.
///
/// A regulator sitting in a solved cluster sees its `Vak` move by microvolts
/// whenever anything else on the board does. Quantizing the *report* means a
/// consumer chain (`ccr -> LED`) is driven by real changes rather than by
/// arithmetic noise — the same drive-on-change discipline the isolators apply
/// to their pins. 1 µA is four decimal places below the part's own ±30%
/// tolerance.
pub const DEFAULT_REPORT_EPSILON_MA: f64 = 0.001;

// ============================================================
// Pin facade
// ============================================================

/// SOD-123 (CASE 425, STYLE 1) two-terminal facade: pin 1 cathode, pin 2
/// anode (NSI50010Y/D Rev. 3, package drawing).
///
/// Both terminals are [`PinKind::Analog`] so they take part in the cluster
/// solve. The **cathode** is the driven one: it is the terminal facing the
/// rest of the branch, and the anode is the stiff end this part sources from
/// (see [`crate::isolation`], "A chain, not a mesh"). The engine gives an
/// `Analog` pin a released drive slot, so an unbiased regulator contributes
/// nothing without any attach-time release.
pub const NSI50010_PINS: [PinDecl; 2] = [
    PinDecl {
        number: "1",
        name: Some("K"),
        kind: PinKind::Analog,
        stream: None,
        drive_impedance: None,
    },
    PinDecl {
        number: "2",
        name: Some("A"),
        kind: PinKind::Analog,
        stream: None,
        drive_impedance: None,
    },
];

// ============================================================
// Configuration
// ============================================================

/// Regulator configuration.
#[derive(Debug, Clone)]
pub struct Config {
    /// Steady-state regulation current in mA ([`DEFAULT_REGULATION_MA`]).
    pub regulation_ma: f64,
    /// Series Thevenin impedance presented to the engine
    /// ([`DEFAULT_SERIES_IMPEDANCE_OHMS`]).
    pub series_impedance_ohms: Ohms,
    /// Report quantization in mA ([`DEFAULT_REPORT_EPSILON_MA`]).
    pub report_epsilon_ma: f64,
}

impl Default for Config {
    fn default() -> Self {
        Self::new()
    }
}

impl Config {
    /// The NSI50010Y at its datasheet typicals.
    pub fn new() -> Self {
        Self {
            regulation_ma: DEFAULT_REGULATION_MA,
            series_impedance_ohms: DEFAULT_SERIES_IMPEDANCE_OHMS,
            report_epsilon_ma: DEFAULT_REPORT_EPSILON_MA,
        }
    }

    /// A different member of the CCR family (NSI50010 is the 10 mA part;
    /// NSI50350 is 35 mA, and so on). The series impedance is re-derived from
    /// the same `Vak` operating point so the two views still agree there.
    pub fn with_regulation_ma(mut self, regulation_ma: f64) -> Self {
        self.regulation_ma = regulation_ma;
        self.series_impedance_ohms = DEFAULT_REGULATION_VAK_VOLTS / (regulation_ma * 1e-3);
        self
    }

    fn validate(&self) -> Result<(), PartConfigError> {
        require_positive("regulation_ma", self.regulation_ma)?;
        require_positive("series_impedance_ohms", self.series_impedance_ohms)?;
        require_positive("report_epsilon_ma", self.report_epsilon_ma)?;
        Ok(())
    }
}

/// The fraction of regulation current the datasheet's turn-on curve gives at
/// `vak`: 0 at or below 0 V, linearly interpolated through [`TURN_ON_CURVE`],
/// and saturated at 1.0 above the last point.
pub fn turn_on_fraction(vak: Volts) -> f64 {
    if !vak.is_finite() || vak <= 0.0 {
        return 0.0;
    }
    for window in TURN_ON_CURVE.windows(2) {
        let (v0, f0) = window[0];
        let (v1, f1) = window[1];
        if vak <= v1 {
            return f0 + (f1 - f0) * (vak - v0) / (v1 - v0);
        }
    }
    1.0
}

// ============================================================
// Core
// ============================================================

#[derive(Debug)]
struct RegulatorState {
    anode: NetState,
    cathode: NetState,
    cathode_pin: Option<PinHandle>,
    /// Last applied drive: `None` = never applied, `Some(None)` = released.
    applied: Option<Option<TheveninDrive>>,
    /// Last reported current (mA).
    reported_ma: f64,
    drives: u64,
    reports: u64,
}

struct Core {
    config: Config,
    state: Mutex<RegulatorState>,
    on_current_change: Observers<f64>,
}

impl std::fmt::Debug for Core {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Core")
            .field("config", &self.config)
            .field("state", &self.state)
            .field("observers", &self.on_current_change.len())
            .finish()
    }
}

/// The numeric node voltage of a sensed terminal, or `None` when the engine
/// has no defensible one. A regulator with an unsolved terminal is not a
/// regulator: it conducts nothing and reports nothing.
fn node_volts(state: NetState) -> Option<Volts> {
    match state {
        NetState::Analog(volts) if volts.is_finite() => Some(volts),
        _ => None,
    }
}

impl Core {
    /// The regulated current (mA) for the presently solved terminals.
    fn current_ma(&self, state: &RegulatorState) -> f64 {
        let (Some(anode), Some(cathode)) = (node_volts(state.anode), node_volts(state.cathode))
        else {
            return 0.0;
        };
        self.config.regulation_ma * turn_on_fraction(anode - cathode)
    }

    /// Apply `mutate` to the state, refresh the drive and the report, then
    /// publish **outside the lock**.
    ///
    /// Observers are consumer chains (`ccr -> LED`), so one of them may well
    /// reach back into this part; emitting under the state lock would make
    /// that a deadlock. [`crate::machine::end_switch`] publishes the same way
    /// and for the same reason.
    fn update(&self, mutate: impl FnOnce(&mut RegulatorState)) {
        let publish = {
            let mut state = self.state.lock().unwrap();
            mutate(&mut state);
            self.refresh(&mut state)
        };
        if let Some(current) = publish {
            self.on_current_change.emit(current);
        }
    }

    /// Drive the cathode from the anode, and decide whether the regulated
    /// current is worth publishing — both only on change. Returns the current
    /// to publish, if any.
    fn refresh(&self, state: &mut RegulatorState) -> Option<f64> {
        let desired = node_volts(state.anode).map(|volts| TheveninDrive {
            volts,
            impedance: self.config.series_impedance_ohms,
        });
        if state.applied != Some(desired) {
            state.applied = Some(desired);
            state.drives += 1;
            if let Some(pin) = &state.cathode_pin {
                pin.set_drive(desired);
            }
        }

        let current = self.current_ma(state);
        if (current - state.reported_ma).abs() < self.config.report_epsilon_ma {
            return None;
        }
        state.reported_ma = current;
        state.reports += 1;
        Some(current)
    }
}

// ============================================================
// Handle
// ============================================================

/// Cheap cloneable handle onto a live [`Nsi50010`] — the current side of the
/// chain.
///
/// Cloned out of the component before it is handed to
/// [`embsim_board::System`], then wired to whatever the regulator feeds,
/// exactly like a motor shaft feeding an encoder:
///
/// ```rust,ignore
/// let led = opto.led_input(OptoChannel::Two);
/// ccr.regulator().on_current_change(move |ma| led.set_current_ma(ma));
/// ```
#[derive(Clone, Debug)]
pub struct Nsi50010Regulator {
    core: Arc<Core>,
}

impl Nsi50010Regulator {
    /// The current (mA) the regulator is presently holding — the datasheet
    /// turn-on curve evaluated at the solved `Vak`, clamped at `Ireg(SS)`.
    ///
    /// Zero when either terminal has no numeric solve (an open loop, a
    /// missing rail) or when the part is not forward-biased.
    pub fn current_ma(&self) -> f64 {
        let state = self.core.state.lock().unwrap();
        self.core.current_ma(&state)
    }

    /// The presently solved `Vak`, or `None` when a terminal is unsolved.
    pub fn vak_volts(&self) -> Option<Volts> {
        let state = self.core.state.lock().unwrap();
        Some(node_volts(state.anode)? - node_volts(state.cathode)?)
    }

    /// Subscribe to changes in the regulated current (mA). Publications are
    /// quantized by [`Config::report_epsilon_ma`]; multiple subscribers are
    /// appended, never overwritten.
    pub fn on_current_change(&self, callback: impl Fn(f64) + Send + 'static) {
        self.core.on_current_change.subscribe(callback);
    }

    /// `set_drive` calls issued since construction — the event-cost meter.
    pub fn drive_count(&self) -> u64 {
        self.core.state.lock().unwrap().drives
    }

    /// Current-change publications issued since construction.
    pub fn report_count(&self) -> u64 {
        self.core.state.lock().unwrap().reports
    }
}

// ============================================================
// Component
// ============================================================

/// An onsemi NSI50010Y constant-current regulator as a live board-engine
/// component.
#[derive(Debug)]
pub struct Nsi50010 {
    core: Arc<Core>,
}

impl Nsi50010 {
    /// Create a regulator from a validated configuration.
    pub fn new(config: Config) -> Result<Self, PartConfigError> {
        config.validate()?;
        tracing::info!(
            regulation_ma = config.regulation_ma,
            series_impedance_ohms = config.series_impedance_ohms,
            "nsi50010: init"
        );
        Ok(Self {
            core: Arc::new(Core {
                config,
                state: Mutex::new(RegulatorState {
                    anode: NetState::Floating,
                    cathode: NetState::Floating,
                    cathode_pin: None,
                    applied: None,
                    reported_ma: 0.0,
                    drives: 0,
                    reports: 0,
                }),
                on_current_change: Observers::new(),
            }),
        })
    }

    /// The current-side handle.
    pub fn regulator(&self) -> Nsi50010Regulator {
        Nsi50010Regulator {
            core: Arc::clone(&self.core),
        }
    }

    /// The validated configuration in force.
    pub fn config(&self) -> &Config {
        &self.core.config
    }
}

impl Component for Nsi50010 {
    fn pins(&self) -> &[PinDecl] {
        &NSI50010_PINS
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        self.core.state.lock().unwrap().cathode_pin = Some(io.pin("1")?);
        {
            let core = Arc::clone(&self.core);
            io.on_sense("2", move |sensed| core.update(|state| state.anode = sensed))?;
        }
        {
            let core = Arc::clone(&self.core);
            io.on_sense("1", move |sensed| {
                core.update(|state| state.cathode = sensed)
            })?;
        }
        Ok(())
    }
}

// ============================================================
// Tests
// ============================================================

#[cfg(test)]
mod tests {
    use rstest::rstest;

    use super::*;

    fn regulator() -> (Nsi50010, Nsi50010Regulator) {
        let ccr = Nsi50010::new(Config::new()).expect("valid config");
        let handle = ccr.regulator();
        (ccr, handle)
    }

    fn set_terminals(ccr: &Nsi50010, anode: NetState, cathode: NetState) {
        ccr.core.update(|state| {
            state.anode = anode;
            state.cathode = cathode;
        });
    }

    /// The series impedance is exactly the datasheet operating point.
    #[rstest]
    fn the_series_impedance_is_the_datasheet_operating_point() {
        assert!((DEFAULT_SERIES_IMPEDANCE_OHMS - 750.0).abs() < 1e-9);
        // ... and the two views agree there: 7.5 V across 750 Ohm is 10 mA.
        let (ccr, handle) = regulator();
        set_terminals(&ccr, NetState::Analog(7.5), NetState::Analog(0.0));
        assert!((handle.current_ma() - DEFAULT_REGULATION_MA).abs() < 1e-9);
        let drive = ccr.core.state.lock().unwrap().applied.flatten().unwrap();
        assert!((7.5 / drive.impedance * 1_000.0 - DEFAULT_REGULATION_MA).abs() < 1e-9);
    }

    /// Every anchor of the datasheet turn-on curve, plus the saturation above
    /// it and the reverse-bias floor below.
    #[rstest]
    #[case::reverse(-1.0, 0.0)]
    #[case::zero(0.0, 0.0)]
    #[case::immediate_turn_on(0.5, 0.40)]
    #[case::half_way_to_the_knee(1.15, 0.60)]
    #[case::voltage_overhead(1.8, 0.80)]
    #[case::operating_point(7.5, 1.00)]
    #[case::well_above(48.0, 1.00)]
    fn the_turn_on_curve_hits_its_datasheet_anchors(#[case] vak: Volts, #[case] fraction: f64) {
        assert!(
            (turn_on_fraction(vak) - fraction).abs() < 1e-9,
            "Vak = {vak}: got {}, want {fraction}",
            turn_on_fraction(vak)
        );
    }

    #[rstest]
    fn a_non_finite_vak_carries_nothing() {
        assert_eq!(turn_on_fraction(f64::NAN), 0.0);
        assert_eq!(turn_on_fraction(f64::NEG_INFINITY), 0.0);
        // Infinity is not a node voltage the resolver can produce; the model
        // refuses it rather than saturating on it.
        assert_eq!(turn_on_fraction(f64::INFINITY), 0.0);
    }

    /// The bench question: a closed loop pulls the cathode down, the CCR sees
    /// overhead and regulates; an open loop leaves the cathode unsolved and
    /// the CCR carries nothing.
    #[rstest]
    fn a_closed_loop_regulates_and_an_open_one_does_not() {
        let (ccr, handle) = regulator();

        // Closed: the return path holds the cathode near ground.
        set_terminals(&ccr, NetState::Analog(24.0), NetState::Analog(4.0));
        assert_eq!(handle.vak_volts(), Some(20.0));
        assert!((handle.current_ma() - 10.0).abs() < 1e-9);

        // Open: nothing sources the cathode node, so the engine floats it.
        set_terminals(&ccr, NetState::Analog(24.0), NetState::Floating);
        assert_eq!(handle.vak_volts(), None);
        assert_eq!(handle.current_ma(), 0.0);

        // Closed but with no rail: no anode solve, no current, no drive.
        set_terminals(&ccr, NetState::Floating, NetState::Analog(0.0));
        assert_eq!(handle.current_ma(), 0.0);
        assert_eq!(ccr.core.state.lock().unwrap().applied, Some(None));
    }

    /// An unbiased regulator conducts nothing: the cathode drive is released
    /// whenever the anode has no numeric solve.
    #[rstest]
    #[case::floating(NetState::Floating)]
    #[case::contention(NetState::Contention)]
    #[case::digital_projection(NetState::Driven(embsim_board::Level::High))]
    fn an_unsolved_anode_releases_the_cathode(#[case] anode: NetState) {
        let (ccr, _handle) = regulator();
        set_terminals(&ccr, anode, NetState::Analog(0.0));
        assert_eq!(ccr.core.state.lock().unwrap().applied, Some(None));
    }

    /// Drive on change only: re-delivering the same anode solve costs no
    /// engine traffic, and neither does a `Vak` wobble below the report
    /// epsilon.
    #[rstest]
    fn refreshing_an_unchanged_regulator_costs_nothing() {
        let (ccr, handle) = regulator();
        set_terminals(&ccr, NetState::Analog(24.0), NetState::Analog(4.0));
        let drives = handle.drive_count();
        let reports = handle.report_count();

        for _ in 0..10 {
            set_terminals(&ccr, NetState::Analog(24.0), NetState::Analog(4.0));
        }
        assert_eq!(handle.drive_count(), drives);
        assert_eq!(handle.report_count(), reports);

        // A microvolt of cluster noise moves Vak but not the report.
        set_terminals(&ccr, NetState::Analog(24.0), NetState::Analog(4.000_000_1));
        assert_eq!(handle.report_count(), reports);

        // A real change costs exactly one of each.
        set_terminals(&ccr, NetState::Analog(12.0), NetState::Analog(4.0));
        assert_eq!(handle.drive_count(), drives + 1);
        assert_eq!(handle.report_count(), reports);
        set_terminals(&ccr, NetState::Analog(12.0), NetState::Analog(11.0));
        assert_eq!(handle.report_count(), reports + 1);
    }

    /// Observers see the regulated current, not the resistive one.
    #[rstest]
    fn observers_receive_the_regulated_current() {
        let (ccr, handle) = regulator();
        let log = Arc::new(Mutex::new(Vec::new()));
        let sink = Arc::clone(&log);
        handle.on_current_change(move |ma| sink.lock().unwrap().push(ma));

        set_terminals(&ccr, NetState::Analog(24.0), NetState::Analog(0.0));
        assert_eq!(log.lock().unwrap().as_slice(), &[10.0]);
        // The equivalent resistor would be carrying 24 V / 750 Ohm = 32 mA;
        // the regulator reports its regulation current instead. Squeeze the
        // overhead to the datasheet's `Voverhead` (1.8 V) and the report
        // follows the curve down to 80%.
        set_terminals(&ccr, NetState::Analog(24.0), NetState::Analog(22.2));
        assert_eq!(log.lock().unwrap().len(), 2);
        assert!((log.lock().unwrap()[1] - 8.0).abs() < 1e-9);
    }

    #[rstest]
    fn the_facade_is_the_two_sod123_terminals() {
        let (ccr, _) = regulator();
        assert_eq!(ccr.pins().len(), 2);
        assert_eq!(ccr.pins()[0].number, "1");
        assert_eq!(ccr.pins()[0].name, Some("K"));
        assert_eq!(ccr.pins()[1].number, "2");
        assert_eq!(ccr.pins()[1].name, Some("A"));
    }

    #[rstest]
    fn a_larger_family_member_rederives_its_impedance() {
        let config = Config::new().with_regulation_ma(35.0);
        assert!((config.series_impedance_ohms - 7.5 / 0.035).abs() < 1e-9);
        assert!(Nsi50010::new(config).is_ok());
    }

    #[rstest]
    #[case::zero_current(Config { regulation_ma: 0.0, ..Config::new() })]
    #[case::negative_impedance(Config { series_impedance_ohms: -1.0, ..Config::new() })]
    #[case::nan_epsilon(Config { report_epsilon_ma: f64::NAN, ..Config::new() })]
    fn invalid_parameters_are_rejected(#[case] config: Config) {
        assert!(Nsi50010::new(config).is_err());
    }
}
