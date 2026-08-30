//! Model: the **Vishay VO2631** dual-channel 10 MBd optocoupler — an LED per
//! channel on the input side, an open-collector detector on the output side.
//!
//! ```text
//!            LED side                barrier             detector side
//!            ────────                ───────             ─────────────
//!   A1 ──┤>|── C1 ═════════════════════║════════════ VO1 ├──┐ open collector,
//!   A2 ──┤>|── C2 ═════════════════════║════════════ VO2 ├──┘ pulled up on the board
//!                                      ║                 VCC / GND
//! ```
//!
//! # Datasheet provenance
//!
//! **Vishay document number 80412, Rev. 1.1 (27-Feb-2025)** — "High Speed
//! Optocoupler, Dual Channel, 10 MBd" (VO2630 / VO2631 / VO4661). Governs:
//!
//! - **The pinout**: DIP-8 — 1 Anode 1, 2 Cathode 1, 3 Cathode 2, 4 Anode 2,
//!   5 GND, 6 VO2, 7 VO1, 8 VCC. [`VO2631_PINS`], and the MaD EdgeBoard
//!   netlist labels `U6`'s pins with exactly those names.
//! - **The truth table** (positive logic): **LED on → output L, LED off →
//!   output H**. An optocoupler is an inverter with a current-mode input, and
//!   this model is that table plus the conditions under which it holds.
//! - **The switching current**: Electrical Characteristics, input threshold
//!   current `ITH` = 2.1 mA typical, **5 mA maximum** at
//!   `VO = 0.6 V, VCC = 5.5 V, IOL = 13 mA`; Recommended Operating Conditions,
//!   input current low level `IFL` = 0..250 µA and high level `IFH` = 5..15 mA.
//!   [`DEFAULT_THRESHOLD_MA`].
//! - **The output stage**: "The detectors features an open drain outputs",
//!   `VOL` = 0.09 V typical / 0.60 V maximum at `IOL = 13 mA`, and the
//!   Recommended Operating Conditions' output pull-up `RL` = 330..4000 Ω.
//!   [`DEFAULT_OUTPUT_IMPEDANCE_OHMS`] is `VOL(typ) / IOL`.
//! - **The supply**: Recommended Operating Conditions, `VCC` = 4.5..5.5 V.
//!   [`DEFAULT_SUPPLY_MIN_VOLTS`].
//! - **The LED**: `VF` = 1.38 V typical at `IF = 10 mA`. The forward branch is
//!   modeled as that operating point's **static resistance**,
//!   `VF / IF = 138 Ω` ([`DEFAULT_LED_RESISTANCE_OHMS`]), which is what makes
//!   the LED loop conduct in the net engine at all.
//!
//! # What "uncurrented" means, and why it matters
//!
//! An open-collector output that is **not** sinking is not driving anything —
//! it is released, and the board's own pull-up decides the level. So:
//!
//! - **LED dark** (no forward current, or its loop is open) → the output pin
//!   is released → the net resolves to whatever the pull-up gives, typically
//!   [`embsim_board::NetState::Pulled`] high. On a board with no pull-up it
//!   floats, and the engine's `FloatingSense` is the honest report rather than
//!   an invented idle level.
//! - **VCC down** → the detector cannot sink either → also released. An
//!   unpowered optocoupler does not hold its output low.
//! - **LED lit at or above `ITH`** → the output sinks: driven to 0 V through
//!   [`Config::output_impedance_ohms`].
//!
//! # The LED branch
//!
//! Each channel's LED senses **both** its terminals and drives its **anode**
//! from the solved cathode through [`Config::led_resistance_ohms`]. Driving
//! the anode (not the cathode) is what makes the part face its own return
//! path: the regulator upstream drives the shared node from the rail, this
//! part drives it from the return, and the node between them solves in one
//! pass. See [`crate::isolation`], "A chain, not a mesh".
//!
//! The forward current the truth table is evaluated against is read at *read
//! time* from the same two solved terminals — `(V_A - V_K) / R_LED`, floored
//! at zero.
//!
//! ## Deliberate simplifications (not modeled)
//!
//! - **The LED's exponential I–V law.** At `IF = 10 mA` the modeled drop is
//!   exactly the datasheet's 1.38 V; away from it the branch is ohmic where
//!   the real diode is logarithmic, so a 5 V loop reads high and a 1 V loop
//!   reads low. The `ΔVF/ΔT` = −1.5 mV/K tempco and the `BVR` = 5 V reverse
//!   breakdown are absent for the same reason.
//! - **The 250 µA..5 mA indeterminate band.** `IFL` guarantees the output
//!   stays high up to 250 µA and `ITH`(max) guarantees it switches by 5 mA;
//!   between them the datasheet promises nothing. The model uses one
//!   threshold, defaulting to the *guaranteed* switching point, so a current
//!   in the band reads as dark — the conservative reading.
//! - **Propagation delay** (`tPLH`/`tPHL` 40..50 ns typical), pulse-width
//!   distortion, `tPSK` skew, and the 11 ns / 2.3 ns edge rates. A channel
//!   switches in the engine iteration its current changes.
//! - **CMTI** (5 kV/µs for this part number), the 5300 V_RMS isolation rating,
//!   `ICC` supply current, LED aging and efficiency drift over life, and the
//!   50 mA output current maximum.

use std::sync::{Arc, Mutex};

use embsim_board::{
    AttachError, Component, ComponentNetIo, NetState, Ohms, PinDecl, PinHandle, PinKind,
    TheveninDrive, Volts,
};

use super::{require_positive, supply_up, PartConfigError};

// ============================================================
// Datasheet constants
// ============================================================

/// Forward current at or above which the output switches low: `ITH` maximum,
/// 5 mA (80412 Rev. 1.1, Electrical Characteristics).
///
/// The *maximum* rather than the 2.1 mA typical: it is the current the
/// datasheet guarantees switching at, so a model that switched at the typical
/// would claim behavior the part does not promise.
pub const DEFAULT_THRESHOLD_MA: f64 = 5.0;

/// Minimum `VCC` for the detector side: 4.5 V (80412 Rev. 1.1, Recommended
/// Operating Conditions, `VCC` = 4.5..5.5 V).
pub const DEFAULT_SUPPLY_MIN_VOLTS: Volts = 4.5;

/// Output sink impedance: `VOL(typ) / IOL` = 0.09 V / 13 mA = 6.92 Ω
/// (80412 Rev. 1.1, Electrical Characteristics — `VOL` at `VCC = 5.5 V`,
/// `IF = 5 mA`, `IOL` sinking 13 mA).
///
/// The typical rather than the 0.60 V maximum, because the maximum
/// (46.2 Ω) is within a factor of ten of the datasheet's own minimum pull-up
/// (`RL` = 330 Ω) and would push an ordinary open-collector net into the
/// engine's cluster solver instead of resolving it as a clean
/// [`embsim_board::NetState::Driven`] low. Set
/// [`Config::output_impedance_ohms`] to 46.2 to model the worst case
/// deliberately.
pub const DEFAULT_OUTPUT_IMPEDANCE_OHMS: Ohms = 0.09 / 0.013;

/// LED forward-branch resistance: `VF / IF` = 1.38 V / 10 mA = 138 Ω
/// (80412 Rev. 1.1, Electrical Characteristics, `VF` typical at
/// `IF = 10 mA`).
pub const DEFAULT_LED_RESISTANCE_OHMS: Ohms = 1.38 / 0.010;

// ============================================================
// Channels
// ============================================================

/// One of the part's two independent channels.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum OptoChannel {
    /// Channel 1: `A1` (pin 1) / `C1` (pin 2) in, `VO1` (pin 7) out.
    One,
    /// Channel 2: `A2` (pin 4) / `C2` (pin 3) in, `VO2` (pin 6) out.
    Two,
}

impl OptoChannel {
    const fn index(self) -> usize {
        match self {
            OptoChannel::One => 0,
            OptoChannel::Two => 1,
        }
    }

    /// Anode, cathode and output pin numbers (80412 Rev. 1.1 pin
    /// connections).
    const fn pins(self) -> (&'static str, &'static str, &'static str) {
        match self {
            OptoChannel::One => ("1", "2", "7"),
            OptoChannel::Two => ("4", "3", "6"),
        }
    }

    /// Both channels, in order.
    pub const ALL: [OptoChannel; 2] = [OptoChannel::One, OptoChannel::Two];
}

// ============================================================
// Pin facade
// ============================================================

/// DIP-8 pinout per 80412 Rev. 1.1: 1 Anode 1, 2 Cathode 1, 3 Cathode 2,
/// 4 Anode 2, 5 GND, 6 VO2, 7 VO1, 8 VCC.
///
/// The four LED terminals are [`PinKind::Analog`] so the forward branch takes
/// part in the cluster solve; the two outputs are [`PinKind::DigitalOut`]
/// because that is the only kind that can source a net — an *open-collector*
/// output idles released, which [`Vo2631::attach`] establishes as its first
/// action (the engine assigns every `DigitalOut` an idle-high drive at
/// assembly).
pub const VO2631_PINS: [PinDecl; 8] = [
    decl("1", "A1", PinKind::Analog),
    decl("2", "C1", PinKind::Analog),
    decl("3", "C2", PinKind::Analog),
    decl("4", "A2", PinKind::Analog),
    decl("5", "GND", PinKind::PowerIn),
    decl("6", "VO2", PinKind::DigitalOut),
    decl("7", "VO1", PinKind::DigitalOut),
    decl("8", "VCC", PinKind::PowerIn),
];

const fn decl(number: &'static str, name: &'static str, kind: PinKind) -> PinDecl {
    PinDecl {
        number,
        name: Some(name),
        kind,
        stream: None,
        drive_impedance: None,
    }
}

// ============================================================
// Configuration
// ============================================================

/// Optocoupler configuration.
#[derive(Debug, Clone)]
pub struct Config {
    /// Forward current (mA) at or above which the output switches low
    /// ([`DEFAULT_THRESHOLD_MA`]).
    pub threshold_ma: f64,
    /// Minimum detector-side supply ([`DEFAULT_SUPPLY_MIN_VOLTS`]).
    pub supply_min_volts: Volts,
    /// Output sink impedance ([`DEFAULT_OUTPUT_IMPEDANCE_OHMS`]).
    pub output_impedance_ohms: Ohms,
    /// LED forward-branch resistance ([`DEFAULT_LED_RESISTANCE_OHMS`]).
    pub led_resistance_ohms: Ohms,
}

impl Default for Config {
    fn default() -> Self {
        Self::new()
    }
}

impl Config {
    /// The VO2631 at its datasheet numbers.
    pub fn new() -> Self {
        Self {
            threshold_ma: DEFAULT_THRESHOLD_MA,
            supply_min_volts: DEFAULT_SUPPLY_MIN_VOLTS,
            output_impedance_ohms: DEFAULT_OUTPUT_IMPEDANCE_OHMS,
            led_resistance_ohms: DEFAULT_LED_RESISTANCE_OHMS,
        }
    }

    fn validate(&self) -> Result<(), PartConfigError> {
        require_positive("threshold_ma", self.threshold_ma)?;
        require_positive("supply_min_volts", self.supply_min_volts)?;
        require_positive("output_impedance_ohms", self.output_impedance_ohms)?;
        require_positive("led_resistance_ohms", self.led_resistance_ohms)?;
        Ok(())
    }
}

// ============================================================
// Core
// ============================================================

#[derive(Debug)]
struct OptoState {
    /// Detector-side supply.
    vcc: NetState,
    /// Per channel: anode, cathode, output pin, LED anode drive, output drive.
    anode: [NetState; 2],
    cathode: [NetState; 2],
    output_pin: [Option<PinHandle>; 2],
    anode_pin: [Option<PinHandle>; 2],
    /// Last applied drives: `None` = never applied, `Some(None)` = released.
    applied_output: [Option<Option<TheveninDrive>>; 2],
    applied_anode: [Option<Option<TheveninDrive>>; 2],
    drives: u64,
}

#[derive(Debug)]
struct Core {
    config: Config,
    state: Mutex<OptoState>,
}

/// The numeric node voltage of a sensed terminal, or `None`.
fn node_volts(state: NetState) -> Option<Volts> {
    match state {
        NetState::Analog(volts) if volts.is_finite() => Some(volts),
        _ => None,
    }
}

impl Core {
    /// Forward current (mA) through a channel's LED, read from the solved
    /// terminals. Zero when either terminal is unsolved (an open loop) or the
    /// branch is reverse-biased.
    fn forward_ma(&self, state: &OptoState, channel: OptoChannel) -> f64 {
        let index = channel.index();
        let (Some(anode), Some(cathode)) = (
            node_volts(state.anode[index]),
            node_volts(state.cathode[index]),
        ) else {
            return 0.0;
        };
        ((anode - cathode) / self.config.led_resistance_ohms * 1_000.0).max(0.0)
    }

    /// Whether the detector is powered.
    fn powered(&self, state: &OptoState) -> bool {
        supply_up(state.vcc, self.config.supply_min_volts)
    }

    /// The output drive: sinking when the detector is powered and the LED is
    /// at or above threshold, released otherwise (80412 Rev. 1.1 truth table
    /// plus the open-collector stage).
    fn desired_output(&self, state: &OptoState, channel: OptoChannel) -> Option<TheveninDrive> {
        if !self.powered(state) {
            return None;
        }
        if self.forward_ma(state, channel) < self.config.threshold_ma {
            return None;
        }
        Some(TheveninDrive {
            volts: 0.0,
            impedance: self.config.output_impedance_ohms,
        })
    }

    /// The LED branch's drive on its own anode: the cathode's solved voltage
    /// through the forward-branch resistance, or released when the return
    /// path has no solve (an open switch loop).
    fn desired_anode(&self, state: &OptoState, channel: OptoChannel) -> Option<TheveninDrive> {
        node_volts(state.cathode[channel.index()]).map(|volts| TheveninDrive {
            volts,
            impedance: self.config.led_resistance_ohms,
        })
    }

    /// Apply both of a channel's drives — only on change.
    fn refresh(&self, state: &mut OptoState, channel: OptoChannel) {
        let index = channel.index();

        let anode = self.desired_anode(state, channel);
        if state.applied_anode[index] != Some(anode) {
            state.applied_anode[index] = Some(anode);
            state.drives += 1;
            if let Some(handle) = &state.anode_pin[index] {
                handle.set_drive(anode);
            }
        }

        let output = self.desired_output(state, channel);
        if state.applied_output[index] != Some(output) {
            state.applied_output[index] = Some(output);
            state.drives += 1;
            if let Some(handle) = &state.output_pin[index] {
                handle.set_drive(output);
            }
        }
    }

    fn refresh_all(&self, state: &mut OptoState) {
        for channel in OptoChannel::ALL {
            self.refresh(state, channel);
        }
    }
}

// ============================================================
// Monitor handle
// ============================================================

/// Cheap cloneable read handle onto a live [`Vo2631`].
#[derive(Clone, Debug)]
pub struct Vo2631Monitor {
    core: Arc<Core>,
}

impl Vo2631Monitor {
    /// Forward current (mA) presently through a channel's LED.
    pub fn forward_ma(&self, channel: OptoChannel) -> f64 {
        let state = self.core.state.lock().unwrap();
        self.core.forward_ma(&state, channel)
    }

    /// Whether a channel's LED is lit past the switching threshold.
    pub fn is_lit(&self, channel: OptoChannel) -> bool {
        self.forward_ma(channel) >= self.core.config.threshold_ma
    }

    /// The drive a channel's output is presenting, or `None` when the
    /// open-collector stage is released (dark LED, or unpowered detector).
    pub fn output_drive(&self, channel: OptoChannel) -> Option<TheveninDrive> {
        self.core.state.lock().unwrap().applied_output[channel.index()].flatten()
    }

    /// Whether a channel is sinking its output low.
    pub fn is_sinking(&self, channel: OptoChannel) -> bool {
        self.output_drive(channel).is_some()
    }

    /// Whether the detector side is powered.
    pub fn is_powered(&self) -> bool {
        let state = self.core.state.lock().unwrap();
        self.core.powered(&state)
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

/// A Vishay VO2631 dual optocoupler as a live board-engine component.
#[derive(Debug)]
pub struct Vo2631 {
    core: Arc<Core>,
}

impl Vo2631 {
    /// Create an optocoupler from a validated configuration.
    pub fn new(config: Config) -> Result<Self, PartConfigError> {
        config.validate()?;
        tracing::info!(
            threshold_ma = config.threshold_ma,
            supply_min_volts = config.supply_min_volts,
            "vo2631: init"
        );
        Ok(Self {
            core: Arc::new(Core {
                config,
                state: Mutex::new(OptoState {
                    vcc: NetState::Floating,
                    anode: [NetState::Floating; 2],
                    cathode: [NetState::Floating; 2],
                    output_pin: Default::default(),
                    anode_pin: Default::default(),
                    applied_output: Default::default(),
                    applied_anode: Default::default(),
                    drives: 0,
                }),
            }),
        })
    }

    /// A read handle onto this optocoupler.
    pub fn monitor(&self) -> Vo2631Monitor {
        Vo2631Monitor {
            core: Arc::clone(&self.core),
        }
    }

    /// The validated configuration in force.
    pub fn config(&self) -> &Config {
        &self.core.config
    }
}

impl Component for Vo2631 {
    fn pins(&self) -> &[PinDecl] {
        &VO2631_PINS
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        // Claim the driven pins and settle both channels. The engine gave each
        // `DigitalOut` an idle-high drive at assembly, and an open-collector
        // output idles *released* — so this must be the first thing that
        // happens, exactly as for a normally-open switch contact.
        {
            let mut state = self.core.state.lock().unwrap();
            for channel in OptoChannel::ALL {
                let (anode, _cathode, output) = channel.pins();
                state.anode_pin[channel.index()] = Some(io.pin(anode)?);
                state.output_pin[channel.index()] = Some(io.pin(output)?);
            }
            self.core.refresh_all(&mut state);
        }

        // The detector supply first: a lit LED delivered before `VCC` is known
        // must not sink through an unpowered output stage.
        {
            let core = Arc::clone(&self.core);
            io.on_sense("8", move |sensed| {
                let mut state = core.state.lock().unwrap();
                state.vcc = sensed;
                core.refresh_all(&mut state);
            })?;
        }

        for channel in OptoChannel::ALL {
            let (anode, cathode, _output) = channel.pins();
            {
                let core = Arc::clone(&self.core);
                io.on_sense(anode, move |sensed| {
                    let mut state = core.state.lock().unwrap();
                    state.anode[channel.index()] = sensed;
                    core.refresh(&mut state, channel);
                })?;
            }
            {
                let core = Arc::clone(&self.core);
                io.on_sense(cathode, move |sensed| {
                    let mut state = core.state.lock().unwrap();
                    state.cathode[channel.index()] = sensed;
                    core.refresh(&mut state, channel);
                })?;
            }
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

    const VCC_5V: NetState = NetState::Analog(5.0);

    fn opto() -> (Vo2631, Vo2631Monitor) {
        let opto = Vo2631::new(Config::new()).expect("valid config");
        let monitor = opto.monitor();
        (opto, monitor)
    }

    fn set_vcc(opto: &Vo2631, vcc: NetState) {
        let mut state = opto.core.state.lock().unwrap();
        state.vcc = vcc;
        opto.core.refresh_all(&mut state);
    }

    fn set_led(opto: &Vo2631, channel: OptoChannel, anode: NetState, cathode: NetState) {
        let mut state = opto.core.state.lock().unwrap();
        state.anode[channel.index()] = anode;
        state.cathode[channel.index()] = cathode;
        opto.core.refresh(&mut state, channel);
    }

    /// Drive a channel's LED at a chosen forward current by putting the right
    /// voltage across the modeled branch.
    fn light(opto: &Vo2631, channel: OptoChannel, milliamps: f64) {
        let drop = milliamps * 1e-3 * DEFAULT_LED_RESISTANCE_OHMS;
        set_led(opto, channel, NetState::Analog(drop), NetState::Analog(0.0));
    }

    #[rstest]
    fn the_facade_is_the_dip8_pinout() {
        let (opto, _) = opto();
        let names: Vec<(&str, Option<&str>)> =
            opto.pins().iter().map(|p| (p.number, p.name)).collect();
        assert_eq!(
            names,
            vec![
                ("1", Some("A1")),
                ("2", Some("C1")),
                ("3", Some("C2")),
                ("4", Some("A2")),
                ("5", Some("GND")),
                ("6", Some("VO2")),
                ("7", Some("VO1")),
                ("8", Some("VCC")),
            ]
        );
    }

    /// The datasheet's constants, spelled out so a change to one is visible.
    #[rstest]
    fn the_datasheet_constants_are_what_they_claim() {
        // VOL(typ) 0.09 V at IOL 13 mA.
        assert!((DEFAULT_OUTPUT_IMPEDANCE_OHMS - 6.923).abs() < 1e-3);
        // VF(typ) 1.38 V at IF 10 mA.
        assert!((DEFAULT_LED_RESISTANCE_OHMS - 138.0).abs() < 1e-6);
        // ITH(max).
        assert!((DEFAULT_THRESHOLD_MA - 5.0).abs() < 1e-12);
    }

    /// The truth table: LED on → output low, LED off → output released for the
    /// board's pull-up to decide.
    #[rstest]
    #[case::dark(0.0, false)]
    #[case::ifl_max(0.25, false)]
    #[case::in_the_indeterminate_band(2.5, false)]
    #[case::at_ith_max(5.0, true)]
    #[case::ifh_nominal(10.0, true)]
    fn the_output_follows_the_forward_current(#[case] milliamps: f64, #[case] sinking: bool) {
        let (opto, monitor) = opto();
        set_vcc(&opto, VCC_5V);
        light(&opto, OptoChannel::One, milliamps);
        assert!(
            (monitor.forward_ma(OptoChannel::One) - milliamps).abs() < 1e-6,
            "the modeled branch must carry the current it was biased for"
        );
        assert_eq!(monitor.is_sinking(OptoChannel::One), sinking);
        if sinking {
            assert_eq!(
                monitor.output_drive(OptoChannel::One),
                Some(TheveninDrive {
                    volts: 0.0,
                    impedance: DEFAULT_OUTPUT_IMPEDANCE_OHMS
                })
            );
        }
    }

    /// An uncurrented LED side leaves the output released, so the board's own
    /// pull-up owns the level. The model never invents an idle high of its
    /// own.
    #[rstest]
    fn an_uncurrented_led_releases_the_output() {
        let (opto, monitor) = opto();
        set_vcc(&opto, VCC_5V);
        light(&opto, OptoChannel::One, 10.0);
        assert!(monitor.is_sinking(OptoChannel::One));

        light(&opto, OptoChannel::One, 0.0);
        assert_eq!(monitor.output_drive(OptoChannel::One), None);
    }

    /// An **unpowered** detector cannot sink, however brightly the LED is
    /// lit — the other half of "pulled to its rail, not held low".
    #[rstest]
    #[case::rail_at_zero(NetState::Analog(0.0))]
    #[case::below_minimum(NetState::Analog(3.3))]
    #[case::floating(NetState::Floating)]
    #[case::contention(NetState::Contention)]
    fn an_unpowered_detector_never_sinks(#[case] vcc: NetState) {
        let (opto, monitor) = opto();
        set_vcc(&opto, VCC_5V);
        light(&opto, OptoChannel::One, 10.0);
        assert!(monitor.is_sinking(OptoChannel::One));

        set_vcc(&opto, vcc);
        assert!(!monitor.is_powered());
        assert_eq!(monitor.output_drive(OptoChannel::One), None);
        // The LED is still lit; the detector simply cannot act on it.
        assert!(monitor.is_lit(OptoChannel::One));
    }

    /// An open loop — the end switch not made — leaves the LED's return path
    /// unsolved, so the branch carries nothing and the anode drive is
    /// released.
    #[rstest]
    fn an_open_return_path_carries_no_current() {
        let (opto, monitor) = opto();
        set_vcc(&opto, VCC_5V);
        set_led(
            &opto,
            OptoChannel::One,
            NetState::Analog(24.0),
            NetState::Floating,
        );
        assert_eq!(monitor.forward_ma(OptoChannel::One), 0.0);
        assert!(!monitor.is_sinking(OptoChannel::One));
        assert_eq!(
            opto.core.state.lock().unwrap().applied_anode[0],
            Some(None),
            "an unsolved return releases the forward branch"
        );
    }

    /// A reverse-biased LED carries nothing (and does not report a negative
    /// current).
    #[rstest]
    fn a_reverse_biased_led_carries_nothing() {
        let (opto, monitor) = opto();
        set_vcc(&opto, VCC_5V);
        set_led(
            &opto,
            OptoChannel::One,
            NetState::Analog(0.0),
            NetState::Analog(5.0),
        );
        assert_eq!(monitor.forward_ma(OptoChannel::One), 0.0);
        assert!(!monitor.is_sinking(OptoChannel::One));
    }

    /// The two channels are independent: lighting one never moves the other's
    /// output.
    #[rstest]
    fn the_channels_are_independent() {
        let (opto, monitor) = opto();
        set_vcc(&opto, VCC_5V);
        light(&opto, OptoChannel::One, 10.0);
        assert!(monitor.is_sinking(OptoChannel::One));
        assert!(!monitor.is_sinking(OptoChannel::Two));

        light(&opto, OptoChannel::Two, 10.0);
        assert!(monitor.is_sinking(OptoChannel::One));
        assert!(monitor.is_sinking(OptoChannel::Two));
    }

    /// Drive on change only: re-delivering the same LED bias costs no engine
    /// traffic.
    #[rstest]
    fn refreshing_an_unchanged_channel_costs_nothing() {
        let (opto, monitor) = opto();
        set_vcc(&opto, VCC_5V);
        light(&opto, OptoChannel::One, 10.0);
        let drives = monitor.drive_count();

        for _ in 0..10 {
            light(&opto, OptoChannel::One, 10.0);
        }
        assert_eq!(monitor.drive_count(), drives);

        // A brighter LED that is still above threshold costs nothing at all:
        // the forward branch's drive is derived from the *return* node, which
        // has not moved, and the output is already sinking.
        light(&opto, OptoChannel::One, 12.0);
        assert_eq!(monitor.drive_count(), drives);

        // Crossing the threshold costs exactly one output drive.
        light(&opto, OptoChannel::One, 0.0);
        assert_eq!(monitor.drive_count(), drives + 1);
    }

    #[rstest]
    #[case::zero_threshold(Config { threshold_ma: 0.0, ..Config::new() })]
    #[case::negative_led(Config { led_resistance_ohms: -1.0, ..Config::new() })]
    #[case::nan_supply(Config { supply_min_volts: f64::NAN, ..Config::new() })]
    fn invalid_parameters_are_rejected(#[case] config: Config) {
        assert!(Vo2631::new(config).is_err());
    }
}
