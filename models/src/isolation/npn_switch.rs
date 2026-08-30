//! Model: a small-signal **NPN transistor used as a saturated switch** — the
//! part that sits between an isolator output and a load it has to pull down.
//!
//! ```text
//!                     load (an ENA input, a relay, an LED)
//!                        │
//!            base ──/\/\──┤ C      collector: shorted to the emitter when on,
//!   (from an isolator)    │        released when off
//!                       B─┤
//!                         │ E
//!                        ─┴─ emitter (usually the isolated ground)
//! ```
//!
//! # Provenance: a *generic* NPN with a cited parameter source
//!
//! This is deliberately not a device model. On the reference machine the
//! netlist calls the part `NPN` — `Q1` on the MaD EdgeBoard carries
//! `(value "NPN")` with `(libsource (part "2N3904"))`, an `MMBT3904` in
//! SOT-23, and its own datasheet field points at **onsemi 2N3903/D, Rev. 9
//! (August 2021)**. So the *behavior* here is a switch, and every number it
//! needs comes from that datasheet's 2N3904 column:
//!
//! - **`VBE(sat)` = 0.65 V minimum** at `IC = 10 mA, IB = 1.0 mA`
//!   (2N3903/D Rev. 9, Electrical Characteristics, ON Characteristics) —
//!   [`DEFAULT_VBE_ON_VOLTS`], the base-emitter drop at which the model
//!   considers the device driven.
//! - **`VCE(sat)` = 0.2 V maximum** at the same operating point —
//!   [`DEFAULT_SATURATION_OHMS`] is `VCE(sat) / IC = 0.2 V / 10 mA = 20 Ω`,
//!   so the saturated collector sits *exactly* `VCE(sat)` above the emitter at
//!   the specified 10 mA, and proportionally less below it.
//! - **`hFE` = 100 minimum** at `IC = 10 mA` — quoted here only to say what
//!   the model does **not** do with it (below).
//!
//! # It is a saturated switch, and that is all
//!
//! The device has two states, decided by one comparison:
//!
//! | `V_B - V_E` | Collector |
//! |---|---|
//! | `>= vbe_on_volts` | driven to `V_E` through `saturation_ohms` |
//! | `< vbe_on_volts`, or either terminal unsolved | released (high impedance) |
//!
//! A base drive the engine will not put a level on — a floating base net, a
//! fought-over one — is **off**, not on: the engine never invents a value for
//! an unsourced node, and a transistor whose base is not driven does not
//! conduct.
//!
//! ## Deliberate simplifications (not modeled)
//!
//! Stating these is the point of calling it a saturated switch:
//!
//! - **No forced beta.** The model does not check that `IB >= IC / hFE`; it
//!   assumes the base resistor was sized to saturate the device. A design
//!   error that under-drives the base reads here as a clean switch.
//! - **No active region.** There is no `IC = hFE x IB` between off and
//!   saturated, so the part is never an amplifier, only a switch.
//! - **No base current at all.** The base pin is a pure sense: it draws
//!   nothing from whatever drives it, so a base-drive network's own loading is
//!   invisible.
//! - **No leakage** (`ICEX`, `IBL` = 50 nA), **no breakdown**
//!   (`V(BR)CEO` = 40 V), **no `IC` maximum** (0.2 A), and no thermal
//!   behavior. An off device is an open circuit and an on device never
//!   fails.
//! - **No storage or switching time** (`ton`/`toff`, tens of nanoseconds).
//! - **`VCE(sat)` is a fixed resistance**, so the saturation drop scales with
//!   collector current where the real part's is nearly constant. At the cited
//!   10 mA it is exact.

use std::sync::{Arc, Mutex};

use embsim_board::{
    AttachError, Component, ComponentNetIo, NetState, Ohms, PinDecl, PinHandle, PinKind,
    TheveninDrive, Volts,
};

use super::{require_positive, PartConfigError};

// ============================================================
// Datasheet constants
// ============================================================

/// Base-emitter drop at which the device is taken to be driven:
/// `VBE(sat)` = 0.65 V minimum at `IC = 10 mA, IB = 1.0 mA`
/// (2N3903/D Rev. 9, 2N3904 column).
pub const DEFAULT_VBE_ON_VOLTS: Volts = 0.65;

/// Saturated collector-emitter resistance: `VCE(sat) / IC` = 0.2 V / 10 mA
/// (2N3903/D Rev. 9, 2N3904 column) — 20 Ω.
pub const DEFAULT_SATURATION_OHMS: Ohms = 20.0;

// ============================================================
// Pin facade
// ============================================================

/// Which netlist pin number is which terminal.
///
/// A generic transistor symbol's pin numbering is a property of the *symbol*,
/// not of the silicon: KiCad's `Q_NPN_BCE` numbers base/collector/emitter
/// 1/2/3, `Q_NPN_CBE` numbers them 2/1/3, and the TO-92 / SOT-23 package
/// drawings in 2N3903/D put emitter/base/collector at 1/2/3. So the mapping
/// is configuration, with the package order as the default because that is
/// what the reference netlist uses (`Q1` pins 1 `E`, 2 `B`, 3 `C`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Pinout {
    /// Netlist pin number of the base.
    pub base: &'static str,
    /// Netlist pin number of the collector.
    pub collector: &'static str,
    /// Netlist pin number of the emitter.
    pub emitter: &'static str,
}

impl Pinout {
    /// `1` = emitter, `2` = base, `3` = collector — the 2N3903/D package
    /// order, and the MaD EdgeBoard's `Q1`.
    pub const EBC: Pinout = Pinout {
        emitter: "1",
        base: "2",
        collector: "3",
    };

    /// `1` = base, `2` = collector, `3` = emitter — KiCad's `Q_*_BCE`
    /// symbols.
    pub const BCE: Pinout = Pinout {
        base: "1",
        collector: "2",
        emitter: "3",
    };

    /// `1` = collector, `2` = base, `3` = emitter — KiCad's `Q_*_CBE`
    /// symbols.
    pub const CBE: Pinout = Pinout {
        collector: "1",
        base: "2",
        emitter: "3",
    };
}

impl Default for Pinout {
    fn default() -> Self {
        Pinout::EBC
    }
}

// ============================================================
// Configuration
// ============================================================

/// Switch configuration.
#[derive(Debug, Clone)]
pub struct Config {
    /// Terminal-to-pin-number mapping ([`Pinout::EBC`]).
    pub pinout: Pinout,
    /// `V_B - V_E` at which the device turns on
    /// ([`DEFAULT_VBE_ON_VOLTS`]).
    pub vbe_on_volts: Volts,
    /// Saturated collector-emitter resistance
    /// ([`DEFAULT_SATURATION_OHMS`]).
    pub saturation_ohms: Ohms,
}

impl Default for Config {
    fn default() -> Self {
        Self::new()
    }
}

impl Config {
    /// A 2N3904-class switch on the package pin order.
    pub fn new() -> Self {
        Self {
            pinout: Pinout::EBC,
            vbe_on_volts: DEFAULT_VBE_ON_VOLTS,
            saturation_ohms: DEFAULT_SATURATION_OHMS,
        }
    }

    /// Use a different terminal-to-pin mapping.
    pub fn with_pinout(mut self, pinout: Pinout) -> Self {
        self.pinout = pinout;
        self
    }

    fn validate(&self) -> Result<(), PartConfigError> {
        require_positive("vbe_on_volts", self.vbe_on_volts)?;
        require_positive("saturation_ohms", self.saturation_ohms)?;
        Ok(())
    }
}

// ============================================================
// Core
// ============================================================

#[derive(Debug)]
struct SwitchState {
    base: NetState,
    emitter: NetState,
    collector_pin: Option<PinHandle>,
    /// Last applied drive: `None` = never applied, `Some(None)` = released.
    applied: Option<Option<TheveninDrive>>,
    drives: u64,
}

#[derive(Debug)]
struct Core {
    config: Config,
    pins: [PinDecl; 3],
    state: Mutex<SwitchState>,
}

/// The numeric node voltage of a sensed terminal, or `None`.
fn node_volts(state: NetState) -> Option<Volts> {
    match state {
        NetState::Analog(volts) if volts.is_finite() => Some(volts),
        // A base or emitter known only as a digital projection still has a
        // usable sense: the resolver put it at a rail. `Driven(Low)` /
        // `Pulled(Low)` is 0 V; a *high* projection carries no number, so the
        // caller falls back on [`Core::vbe_volts`]'s own rule.
        NetState::Driven(embsim_board::Level::Low)
        | NetState::Pulled(embsim_board::Level::Low, _) => Some(0.0),
        _ => None,
    }
}

impl Core {
    /// `V_B - V_E`, or `None` when the engine has no defensible answer.
    ///
    /// A base projected `Driven(High)`/`Pulled(High)` has no node voltage, but
    /// it does have a level, and a high base against a solved emitter is a
    /// driven base. That case returns the turn-on threshold exactly, so the
    /// comparison below reads it as on.
    fn vbe_volts(&self, state: &SwitchState) -> Option<Volts> {
        let emitter = node_volts(state.emitter)?;
        match state.base {
            NetState::Driven(embsim_board::Level::High)
            | NetState::Pulled(embsim_board::Level::High, _) => Some(self.config.vbe_on_volts),
            other => Some(node_volts(other)? - emitter),
        }
    }

    /// Whether the device is saturated.
    fn saturated(&self, state: &SwitchState) -> bool {
        self.vbe_volts(state)
            .is_some_and(|vbe| vbe >= self.config.vbe_on_volts)
    }

    /// The collector drive: a resistive short to the emitter when saturated,
    /// released when off.
    fn desired_drive(&self, state: &SwitchState) -> Option<TheveninDrive> {
        if !self.saturated(state) {
            return None;
        }
        Some(TheveninDrive {
            volts: node_volts(state.emitter)?,
            impedance: self.config.saturation_ohms,
        })
    }

    /// Apply the collector drive — only on change.
    fn refresh(&self, state: &mut SwitchState) {
        let desired = self.desired_drive(state);
        if state.applied == Some(desired) {
            return;
        }
        state.applied = Some(desired);
        state.drives += 1;
        if let Some(pin) = &state.collector_pin {
            pin.set_drive(desired);
        }
    }
}

// ============================================================
// Monitor handle
// ============================================================

/// Cheap cloneable read handle onto a live [`NpnSwitch`].
#[derive(Clone, Debug)]
pub struct NpnSwitchMonitor {
    core: Arc<Core>,
}

impl NpnSwitchMonitor {
    /// Whether the device is saturated (the collector path is on).
    pub fn is_on(&self) -> bool {
        let state = self.core.state.lock().unwrap();
        self.core.saturated(&state)
    }

    /// The presently sensed `V_B - V_E`, or `None` when a terminal is
    /// unsolved.
    pub fn vbe_volts(&self) -> Option<Volts> {
        let state = self.core.state.lock().unwrap();
        self.core.vbe_volts(&state)
    }

    /// The drive the collector is presenting, or `None` when released.
    pub fn collector_drive(&self) -> Option<TheveninDrive> {
        self.core.state.lock().unwrap().applied.flatten()
    }

    /// `set_drive` calls issued since construction — the event-cost meter.
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

/// A small-signal NPN transistor as a saturated switch, on the board engine.
#[derive(Debug)]
pub struct NpnSwitch {
    core: Arc<Core>,
}

impl NpnSwitch {
    /// Create a switch from a validated configuration.
    pub fn new(config: Config) -> Result<Self, PartConfigError> {
        config.validate()?;
        let pins = [
            declare(config.pinout.base, "B", PinKind::DigitalIn),
            declare(config.pinout.collector, "C", PinKind::DigitalOut),
            declare(config.pinout.emitter, "E", PinKind::DigitalIn),
        ];
        tracing::info!(
            vbe_on_volts = config.vbe_on_volts,
            saturation_ohms = config.saturation_ohms,
            "npn_switch: init"
        );
        Ok(Self {
            core: Arc::new(Core {
                config,
                pins,
                state: Mutex::new(SwitchState {
                    base: NetState::Floating,
                    emitter: NetState::Floating,
                    collector_pin: None,
                    applied: None,
                    drives: 0,
                }),
            }),
        })
    }

    /// A read handle onto this switch.
    pub fn monitor(&self) -> NpnSwitchMonitor {
        NpnSwitchMonitor {
            core: Arc::clone(&self.core),
        }
    }

    /// The validated configuration in force.
    pub fn config(&self) -> &Config {
        &self.core.config
    }
}

/// The collector is [`PinKind::DigitalOut`] because that is the only kind
/// that can source a net, but the device idles **off** — [`NpnSwitch::attach`]
/// releases it first thing, exactly as an open switch contact does.
const fn declare(number: &'static str, name: &'static str, kind: PinKind) -> PinDecl {
    PinDecl {
        number,
        name: Some(name),
        kind,
        stream: None,
        drive_impedance: None,
    }
}

impl Component for NpnSwitch {
    fn pins(&self) -> &[PinDecl] {
        &self.core.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let pinout = self.core.config.pinout;
        {
            let mut state = self.core.state.lock().unwrap();
            state.collector_pin = Some(io.pin(pinout.collector)?);
            // A transistor with no base drive is off, and the engine gave the
            // collector an idle-high drive at assembly; release it now.
            self.core.refresh(&mut state);
        }
        {
            let core = Arc::clone(&self.core);
            io.on_sense(pinout.emitter, move |sensed| {
                let mut state = core.state.lock().unwrap();
                state.emitter = sensed;
                core.refresh(&mut state);
            })?;
        }
        {
            let core = Arc::clone(&self.core);
            io.on_sense(pinout.base, move |sensed| {
                let mut state = core.state.lock().unwrap();
                state.base = sensed;
                core.refresh(&mut state);
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
    use embsim_board::Level;
    use rstest::rstest;

    use super::*;

    fn switch(config: Config) -> (NpnSwitch, NpnSwitchMonitor) {
        let switch = NpnSwitch::new(config).expect("valid config");
        let monitor = switch.monitor();
        (switch, monitor)
    }

    fn bias(switch: &NpnSwitch, base: NetState, emitter: NetState) {
        let mut state = switch.core.state.lock().unwrap();
        state.base = base;
        state.emitter = emitter;
        switch.core.refresh(&mut state);
    }

    /// The default facade is the package order the reference netlist uses.
    #[rstest]
    fn the_default_facade_is_the_package_pin_order() {
        let (switch, _) = switch(Config::new());
        let map: Vec<(&str, Option<&str>)> =
            switch.pins().iter().map(|p| (p.number, p.name)).collect();
        assert_eq!(
            map,
            vec![("2", Some("B")), ("3", Some("C")), ("1", Some("E"))]
        );
        // Only the collector can source a net.
        assert_eq!(switch.pins()[1].kind, PinKind::DigitalOut);
        assert_eq!(switch.pins()[0].kind, PinKind::DigitalIn);
        assert_eq!(switch.pins()[2].kind, PinKind::DigitalIn);
    }

    #[rstest]
    #[case::ebc(Pinout::EBC, "2", "3", "1")]
    #[case::bce(Pinout::BCE, "1", "2", "3")]
    #[case::cbe(Pinout::CBE, "2", "1", "3")]
    fn the_pinout_is_configuration(
        #[case] pinout: Pinout,
        #[case] base: &str,
        #[case] collector: &str,
        #[case] emitter: &str,
    ) {
        let (switch, _) = switch(Config::new().with_pinout(pinout));
        let numbers: Vec<&str> = switch.pins().iter().map(|p| p.number).collect();
        assert_eq!(numbers, vec![base, collector, emitter]);
        let mut sorted = numbers;
        sorted.sort_unstable();
        assert_eq!(sorted, vec!["1", "2", "3"], "the facade covers the package");
    }

    /// The one comparison the model makes: `V_B - V_E` against `VBE(sat)`.
    #[rstest]
    #[case::well_above(3.3, 0.0, true)]
    #[case::at_threshold(0.65, 0.0, true)]
    #[case::just_below(0.649, 0.0, false)]
    #[case::grounded_base(0.0, 0.0, false)]
    #[case::lifted_emitter(3.3, 3.0, false)]
    #[case::lifted_emitter_still_on(3.3, 2.6, true)]
    fn the_base_emitter_drop_decides(
        #[case] base: Volts,
        #[case] emitter: Volts,
        #[case] on: bool,
    ) {
        let (switch, monitor) = switch(Config::new());
        bias(&switch, NetState::Analog(base), NetState::Analog(emitter));
        assert_eq!(monitor.is_on(), on, "V_BE = {}", base - emitter);
    }

    /// A saturated device is a resistive short to its own emitter, so the
    /// collector sits `VCE(sat)` above it at the cited 10 mA.
    #[rstest]
    fn saturation_is_a_short_to_the_emitter() {
        let (switch, monitor) = switch(Config::new());
        bias(&switch, NetState::Analog(3.3), NetState::Analog(0.0));
        assert_eq!(
            monitor.collector_drive(),
            Some(TheveninDrive {
                volts: 0.0,
                impedance: DEFAULT_SATURATION_OHMS
            })
        );
        // 10 mA through 20 Ohm is exactly the datasheet's VCE(sat) = 0.2 V.
        assert!((0.010 * DEFAULT_SATURATION_OHMS - 0.2).abs() < 1e-12);

        // The short follows the emitter, not ground: an emitter lifted to
        // 2.0 V takes the collector with it.
        bias(&switch, NetState::Analog(3.3), NetState::Analog(2.0));
        assert_eq!(
            monitor.collector_drive(),
            Some(TheveninDrive {
                volts: 2.0,
                impedance: DEFAULT_SATURATION_OHMS
            })
        );
    }

    /// An off device is an open circuit: the collector is released and
    /// whatever pulls the load decides its level.
    #[rstest]
    fn an_off_device_releases_the_collector() {
        let (switch, monitor) = switch(Config::new());
        bias(&switch, NetState::Analog(3.3), NetState::Analog(0.0));
        assert!(monitor.is_on());
        bias(&switch, NetState::Analog(0.0), NetState::Analog(0.0));
        assert_eq!(monitor.collector_drive(), None);
    }

    /// A base the engine will not put a level on is **off**. A transistor
    /// whose base is not driven does not conduct, and a model that guessed
    /// otherwise would turn a wiring error into working motion.
    #[rstest]
    #[case::floating_base(NetState::Floating, NetState::Analog(0.0))]
    #[case::contended_base(NetState::Contention, NetState::Analog(0.0))]
    #[case::nan_base(NetState::Analog(f64::NAN), NetState::Analog(0.0))]
    #[case::floating_emitter(NetState::Analog(3.3), NetState::Floating)]
    #[case::contended_emitter(NetState::Analog(3.3), NetState::Contention)]
    fn an_undecidable_terminal_leaves_the_device_off(
        #[case] base: NetState,
        #[case] emitter: NetState,
    ) {
        let (switch, monitor) = switch(Config::new());
        bias(&switch, NetState::Analog(3.3), NetState::Analog(0.0));
        assert!(monitor.is_on());
        bias(&switch, base, emitter);
        assert!(!monitor.is_on());
        assert_eq!(monitor.collector_drive(), None);
    }

    /// A digitally projected base and emitter still decide the device: an
    /// isolator output that resolves `Driven(High)` into a grounded emitter is
    /// a driven base.
    #[rstest]
    #[case::driven_high(NetState::Driven(Level::High), true)]
    #[case::pulled_high(NetState::Pulled(Level::High, 4_700.0), true)]
    #[case::driven_low(NetState::Driven(Level::Low), false)]
    #[case::pulled_low(NetState::Pulled(Level::Low, 10_000.0), false)]
    fn a_digitally_projected_base_still_decides(#[case] base: NetState, #[case] on: bool) {
        let (switch, monitor) = switch(Config::new());
        bias(&switch, base, NetState::Driven(Level::Low));
        assert_eq!(monitor.is_on(), on);
    }

    /// Drive on change only.
    #[rstest]
    fn refreshing_an_unchanged_switch_costs_nothing() {
        let (switch, monitor) = switch(Config::new());
        bias(&switch, NetState::Analog(0.0), NetState::Analog(0.0));
        let drives = monitor.drive_count();

        for _ in 0..10 {
            bias(&switch, NetState::Analog(0.0), NetState::Analog(0.0));
        }
        assert_eq!(monitor.drive_count(), drives);

        bias(&switch, NetState::Analog(3.3), NetState::Analog(0.0));
        assert_eq!(monitor.drive_count(), drives + 1);
        // A different base voltage that is still a saturating one costs
        // nothing: the collector drive is derived from the emitter.
        bias(&switch, NetState::Analog(5.0), NetState::Analog(0.0));
        assert_eq!(monitor.drive_count(), drives + 1);
    }

    #[rstest]
    fn vbe_is_reported_for_diagnosis() {
        let (switch, monitor) = switch(Config::new());
        bias(&switch, NetState::Analog(3.3), NetState::Analog(0.5));
        assert!((monitor.vbe_volts().expect("solved") - 2.8).abs() < 1e-9);
        bias(&switch, NetState::Floating, NetState::Analog(0.5));
        assert_eq!(monitor.vbe_volts(), None);
    }

    #[rstest]
    #[case::zero_vbe(Config { vbe_on_volts: 0.0, ..Config::new() })]
    #[case::negative_saturation(Config { saturation_ohms: -1.0, ..Config::new() })]
    fn invalid_parameters_are_rejected(#[case] config: Config) {
        assert!(NpnSwitch::new(config).is_err());
    }
}
