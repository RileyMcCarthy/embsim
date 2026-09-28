//! Model: an **optocoupler** — an LED on the input side, an open-collector
//! (open-drain) detector on the output side — as one part for every
//! optocoupler on the reference boards: the Vishay VO2631 dual channel
//! (`U5`–`U8`, the isolated digital inputs) and the Lite-On 6N137 single
//! channel with an enable (`U4`, the charge-pump drive).
//!
//! ```text
//!            LED side                barrier             detector side
//!            ────────                ───────             ─────────────
//!   A ──┤>|── C  ═════════════════════║════════════ VO ├── open collector,
//!                                     ║                   pulled up on the board
//!                                     ║              VCC / GND (/ EN)
//! ```
//!
//! # The LED is a branch the part owns
//!
//! The input LED is a piecewise-linear branch declared by the component
//! ([`Component::branches`], `NODES.md` §2 the Opto row and §11): the
//! engine stamps it into the loop's cluster solve with the current
//! regulator, the contact and the rail around it, chooses whether it
//! conducts, and hands the part the branch current through the current
//! instrument ([`ComponentNetIo::on_branch`]). The part never drives its
//! LED terminals and never reads a voltage off them: it is told the
//! current, and the current is what its truth table is written in. This
//! is the transducer primitive `BOARD_ENGINE.md` describes — a
//! two-terminal element contributed by a part — and it retires the
//! "drive one terminal from the other" chain that stood in for a loop
//! the engine could not solve.
//!
//! # The output is a sink that releases
//!
//! An open-collector output that is **not** sinking drives nothing — it is
//! released, and the board's own pull-up decides the level (`NODES.md` §2,
//! the open-drain row). The output sinks 0 V through the datasheet's
//! `V_OL / I_OL` while every one of these holds:
//!
//! - the LED carries at least the input threshold current `I_TH`;
//! - the detector supply reads up ([`crate::isolation::supply_up`]);
//! - an enable pin, where the part has one, does not read low.
//!
//! Otherwise the output is released. Every drive is applied on change
//! only (the drive-on-change rule of [`crate::isolation`]).
//!
//! # Datasheet provenance
//!
//! Each configuration cites its own sheet at its constants:
//! [`Config::vo2631`] — Vishay document 80412, Rev. 1.1 (27-Feb-2025);
//! [`Config::lite_on_6n137`] — Lite-On "6N137 – High Speed 10MBd
//! Optocouplers", Aug 2008. Where a sheet gives a range the configuration
//! takes the bound that guarantees the behaviour: the **maximum** input
//! threshold (the current the part is guaranteed to have switched by), the
//! **maximum** low-level output voltage over its test current (the weakest
//! sink the part is guaranteed to be — the bound the logic gates use for
//! their output resistance too; source-strength projection ranks a 1 kΩ
//! pull-up as a pull against it whatever its value, so no output impedance
//! is tuned to the ranking), the **typical** forward voltage (one point,
//! stated as the drop at any current — see `pwl_library`).
//!
//! ## Deliberate simplifications (not modeled)
//!
//! - **The LED's exponential I–V law.** The drop is the tabulated `V_F`
//!   at any current (a vertical on-segment); tempco and reverse breakdown
//!   are absent.
//! - **The indeterminate band** between the guaranteed-off input current
//!   and `I_TH`: one threshold, the guaranteed switching point, so a
//!   current in the band reads as dark.
//! - **The enable's indeterminate band** between `V_EL` and `V_EH` (the
//!   6N137): a voltage in it reads as disabled — the bound that guarantees
//!   the outputs follow the LED is `V_EH`, and the model takes the
//!   guaranteed bound, as it does for the LED. A level (`Driven(High)`, a
//!   pull-up) is enabled; an open enable follows.
//! - **Propagation delay, pulse-width distortion, edge rates**: a channel
//!   switches in the pass its current changes.
//! - **CMTI, isolation rating, supply current, aging.**

use std::sync::{Arc, Mutex};

use embsim_board::{
    Amps, AttachError, Branch, Component, ComponentNetIo, IdleDrive, Level, NetState, Ohms,
    PinDecl, PinHandle, PinKind, PwlCurve, TheveninDrive, Volts,
};

use crate::isolation::{require_positive, supply_up, PartConfigError};

// ============================================================
// Vishay VO2631 datasheet constants
// ============================================================

/// VO2631: forward current at or above which the output switches low,
/// `I_TH` **5 mA maximum** (2.1 mA typical) at `V_O` = 0.6 V, `V_CC` =
/// 5.5 V, `I_OL` = 13 mA (Vishay 80412 Rev. 1.1, Electrical
/// Characteristics). The maximum: the current the datasheet guarantees
/// switching at.
pub const VO2631_THRESHOLD_AMPS: Amps = 5e-3;

/// VO2631: the LED's forward drop, `V_F` **1.38 V typical** at `I_F` =
/// 10 mA (80412 Rev. 1.1, Electrical Characteristics).
pub const VO2631_LED_VF_VOLTS: Volts = 1.38;

/// VO2631: minimum detector supply, `V_CC` **4.5 V** (80412 Rev. 1.1,
/// Recommended Operating Conditions, `V_CC` = 4.5..5.5 V).
pub const VO2631_SUPPLY_MIN_VOLTS: Volts = 4.5;

/// VO2631: output sink impedance, `V_OL` **0.60 V maximum** at `I_OL` =
/// 13 mA (80412 Rev. 1.1, Electrical Characteristics; 0.09 V typical) —
/// 46.2 Ω, the weakest sink the part is guaranteed to be.
pub const VO2631_OUTPUT_IMPEDANCE_OHMS: Ohms = 0.60 / 0.013;

// ============================================================
// Lite-On 6N137 datasheet constants
// ============================================================

/// 6N137: input threshold current, `I_FTH` **5 mA maximum** (3 mA
/// typical) at `V_CC` = 5.5 V, `V_O` = 0.5 V, `I_OL` = 13 mA, `V_E` = 2.0 V
/// (Lite-On 6N137, Aug 2008, "Transfer Characteristics (DC)").
pub const LITE_ON_6N137_THRESHOLD_AMPS: Amps = 5e-3;

/// 6N137: the LED's forward drop, `V_F` **1.45 V typical** (1.7 V
/// maximum) at `I_F` = 10 mA (Lite-On 6N137, "Electrical–Optical
/// Characteristics", Input).
pub const LITE_ON_6N137_LED_VF_VOLTS: Volts = 1.45;

/// 6N137: output sink impedance, `V_OL` **0.6 V maximum** (0.35 V
/// typical) at `I_OL` = 13 mA, `I_F` = 5 mA, `V_E` = 2.0 V (Lite-On 6N137,
/// "Transfer Characteristics (DC)") — 46.2 Ω.
pub const LITE_ON_6N137_OUTPUT_IMPEDANCE_OHMS: Ohms = 0.6 / 0.013;

/// 6N137: the enable pin's low level, `V_EL` **0.8 V maximum** (Lite-On
/// 6N137, "Electrical–Optical Characteristics", Output). An enable at or
/// below it holds the output high (released); the truth table's `NC` row
/// says an open enable follows the input.
pub const LITE_ON_6N137_ENABLE_LOW_MAX_VOLTS: Volts = 0.8;

/// 6N137: the enable pin's high level, `V_EH` **2.0 V minimum** (Lite-On
/// 6N137, "Electrical–Optical Characteristics", Output). An enable at or
/// above it lets the output follow the LED; a voltage between `V_EL` and
/// `V_EH` is the band the datasheet guarantees nothing in, and reads as
/// disabled (the module docs, "Deliberate simplifications").
pub const LITE_ON_6N137_ENABLE_HIGH_MIN_VOLTS: Volts = 2.0;

/// 6N137: minimum detector supply, **4.5 V**. The Lite-On sheet
/// characterises the detector at `V_CC` = 5 V (typicals) and 5.5 V
/// (limits) and tabulates no recommended range; 4.5 V is the industry
/// 6N137's recommended `V_CC` minimum (Broadcom/Avago 6N137 / HCPL-2601
/// family datasheet AV02-0940EN, Recommended Operating Conditions,
/// `V_CC` = 4.5..5.5 V), cited across manufacturers for the same
/// registered part number and said so here.
pub const LITE_ON_6N137_SUPPLY_MIN_VOLTS: Volts = 4.5;

// ============================================================
// Channels
// ============================================================

/// One of a part's channels, by position: a VO2631 has two, a 6N137 one.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum OptoChannel {
    /// The first channel: `A1`/`C1` in, `VO1` out on a VO2631; the only
    /// channel of a 6N137.
    One,
    /// The second channel: `A2`/`C2` in, `VO2` out on a VO2631.
    Two,
}

impl OptoChannel {
    const fn index(self) -> usize {
        match self {
            OptoChannel::One => 0,
            OptoChannel::Two => 1,
        }
    }

    /// Both positions, in order.
    pub const ALL: [OptoChannel; 2] = [OptoChannel::One, OptoChannel::Two];
}

/// The pins of one channel: LED anode, LED cathode, open-collector output.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ChannelPins {
    /// The LED anode pin id.
    pub anode: &'static str,
    /// The LED cathode pin id.
    pub cathode: &'static str,
    /// The open-collector output pin id.
    pub output: &'static str,
}

// ============================================================
// Configuration
// ============================================================

/// An optocoupler's configuration: its facade and its datasheet numbers.
#[derive(Debug, Clone)]
pub struct Config {
    /// The part, for the log.
    pub part: &'static str,
    /// The pin facade, in the order the datasheet numbers the pins.
    pub pins: Vec<PinDecl>,
    /// The channels, in position order.
    pub channels: Vec<ChannelPins>,
    /// The detector supply pin id.
    pub vcc: &'static str,
    /// The enable pin id, for a part that has one.
    pub enable: Option<&'static str>,
    /// The LED's forward drop, volts.
    pub led_vf_volts: Volts,
    /// The LED's on-segment slope, ohms (0 = vertical).
    pub led_r_d_ohms: Ohms,
    /// Forward current at or above which a channel's output sinks, amperes.
    pub threshold_amps: Amps,
    /// Minimum detector supply, volts.
    pub supply_min_volts: Volts,
    /// Output sink impedance, ohms.
    pub output_impedance_ohms: Ohms,
    /// The enable pin's guaranteed high level, volts: an enable voltage at
    /// or above it lets the outputs follow the LEDs; below it — the low
    /// level and the band above it alike — holds them released.
    pub enable_high_min_volts: Volts,
}

const fn pin(number: &'static str, name: &'static str, kind: PinKind) -> PinDecl {
    PinDecl {
        number,
        name: Some(name),
        kind,
        stream: None,
        drive_impedance: None,
        idle: IdleDrive::KindDefault,
    }
}

/// An LED terminal: a passive terminal of the branch the part declares.
const fn led(number: &'static str, name: &'static str) -> PinDecl {
    pin(number, name, PinKind::Passive)
}

/// An open-collector output: the one kind that can source a net, released
/// at attach because a sink that is not sinking drives nothing.
const fn open_collector(number: &'static str, name: &'static str) -> PinDecl {
    PinDecl {
        idle: IdleDrive::Released,
        ..pin(number, name, PinKind::DigitalOut)
    }
}

impl Config {
    /// The Vishay VO2631 at its datasheet numbers. DIP-8 per 80412
    /// Rev. 1.1: 1 Anode 1, 2 Cathode 1, 3 Cathode 2, 4 Anode 2, 5 GND,
    /// 6 VO2, 7 VO1, 8 VCC — the names the MaD EdgeBoard netlist gives
    /// `U5`–`U8`'s pins.
    pub fn vo2631() -> Self {
        Self {
            part: "VO2631",
            pins: vec![
                led("1", "A1"),
                led("2", "C1"),
                led("3", "C2"),
                led("4", "A2"),
                pin("5", "GND", PinKind::PowerIn),
                open_collector("6", "VO2"),
                open_collector("7", "VO1"),
                pin("8", "VCC", PinKind::PowerIn),
            ],
            channels: vec![
                ChannelPins {
                    anode: "1",
                    cathode: "2",
                    output: "7",
                },
                ChannelPins {
                    anode: "4",
                    cathode: "3",
                    output: "6",
                },
            ],
            vcc: "8",
            enable: None,
            led_vf_volts: VO2631_LED_VF_VOLTS,
            led_r_d_ohms: 0.0,
            threshold_amps: VO2631_THRESHOLD_AMPS,
            supply_min_volts: VO2631_SUPPLY_MIN_VOLTS,
            output_impedance_ohms: VO2631_OUTPUT_IMPEDANCE_OHMS,
            enable_high_min_volts: 0.0,
        }
    }

    /// The Lite-On 6N137 at its datasheet numbers. DIP-8 per the Lite-On
    /// sheet's "Pin Define": 1 NC, 2 Anode, 3 Cathode, 4 NC, 5 GND, 6 Vo,
    /// 7 VE (enable), 8 Vcc. The facade carries **no pin 4**: the KiCad
    /// symbol the MaD EdgeBoard's `U4` is drawn with has none, and a
    /// facade names the netlist's pins exactly.
    pub fn lite_on_6n137() -> Self {
        Self {
            part: "6N137",
            pins: vec![
                pin("1", "NC", PinKind::Passive),
                led("2", "A"),
                led("3", "C"),
                pin("5", "GND", PinKind::PowerIn),
                open_collector("6", "VO"),
                pin("7", "EN", PinKind::DigitalIn),
                pin("8", "VCC", PinKind::PowerIn),
            ],
            channels: vec![ChannelPins {
                anode: "2",
                cathode: "3",
                output: "6",
            }],
            vcc: "8",
            enable: Some("7"),
            led_vf_volts: LITE_ON_6N137_LED_VF_VOLTS,
            led_r_d_ohms: 0.0,
            threshold_amps: LITE_ON_6N137_THRESHOLD_AMPS,
            supply_min_volts: LITE_ON_6N137_SUPPLY_MIN_VOLTS,
            output_impedance_ohms: LITE_ON_6N137_OUTPUT_IMPEDANCE_OHMS,
            enable_high_min_volts: LITE_ON_6N137_ENABLE_HIGH_MIN_VOLTS,
        }
    }

    fn validate(&self) -> Result<(), PartConfigError> {
        require_positive("led_vf_volts", self.led_vf_volts)?;
        require_positive("threshold_amps", self.threshold_amps)?;
        require_positive("supply_min_volts", self.supply_min_volts)?;
        require_positive("output_impedance_ohms", self.output_impedance_ohms)?;
        if !(self.led_r_d_ohms.is_finite() && self.led_r_d_ohms >= 0.0) {
            return Err(PartConfigError::NotPositive {
                field: "led_r_d_ohms",
                value: self.led_r_d_ohms,
            });
        }
        Ok(())
    }

    /// The LED branches, one per channel, anode to cathode.
    fn branches(&self) -> Vec<Branch> {
        self.channels
            .iter()
            .map(|channel| Branch {
                a: channel.anode,
                b: channel.cathode,
                curve: PwlCurve::Diode {
                    vf: self.led_vf_volts,
                    r_d: self.led_r_d_ohms,
                },
                control: None,
            })
            .collect()
    }
}

// ============================================================
// Core
// ============================================================

#[derive(Debug)]
struct OptoState {
    /// Detector-side supply.
    vcc: NetState,
    /// The enable pin's net, for a part with one.
    enable: Option<NetState>,
    /// Per channel: the LED's branch current as last delivered.
    led: Vec<Option<Amps>>,
    /// Per channel: the output pin.
    output: Vec<Option<PinHandle>>,
    /// Per channel: the last applied output drive — `None` never applied,
    /// `Some(None)` released.
    applied: Vec<Option<Option<TheveninDrive>>>,
    /// Drives issued since construction.
    drives: u64,
}

#[derive(Debug)]
struct Core {
    config: Config,
    state: Mutex<OptoState>,
}

impl Core {
    fn powered(&self, state: &OptoState) -> bool {
        supply_up(state.vcc, self.config.supply_min_volts)
    }

    /// Whether the enable lets the outputs follow the LEDs: a part with
    /// no enable always does; an enable that reads low, or a voltage under
    /// the guaranteed high level `V_EH`, holds them released; a high level
    /// or an open one follows (the 6N137 truth table's `NC` row).
    fn enabled(&self, state: &OptoState) -> bool {
        match state.enable {
            None => true,
            Some(NetState::Driven(Level::Low) | NetState::Pulled(Level::Low, _)) => false,
            Some(NetState::Analog(volts)) => volts >= self.config.enable_high_min_volts,
            Some(_) => true,
        }
    }

    fn lit(&self, state: &OptoState, index: usize) -> bool {
        state
            .led
            .get(index)
            .copied()
            .flatten()
            .is_some_and(|amps| amps >= self.config.threshold_amps)
    }

    /// The output drive: sinking while the detector is powered, enabled
    /// and the LED is at or above threshold; released otherwise.
    fn desired(&self, state: &OptoState, index: usize) -> Option<TheveninDrive> {
        (self.powered(state) && self.enabled(state) && self.lit(state, index)).then_some(
            TheveninDrive {
                volts: 0.0,
                impedance: self.config.output_impedance_ohms,
            },
        )
    }

    fn refresh(&self, state: &mut OptoState, index: usize) {
        let drive = self.desired(state, index);
        if state.applied[index] != Some(drive) {
            state.applied[index] = Some(drive);
            state.drives += 1;
            if let Some(handle) = &state.output[index] {
                handle.set_drive(drive);
            }
        }
    }

    fn refresh_all(&self, state: &mut OptoState) {
        for index in 0..self.config.channels.len() {
            self.refresh(state, index);
        }
    }
}

// ============================================================
// Monitor handle
// ============================================================

/// Cheap cloneable read handle onto a live [`Opto`].
#[derive(Clone, Debug)]
pub struct OptoMonitor {
    core: Arc<Core>,
}

impl OptoMonitor {
    /// The current through a channel's LED, anode to cathode, as the
    /// engine last delivered it; `None` before any delivery, for a channel
    /// the part does not have, or while the loop's cluster has no solve.
    pub fn forward_amps(&self, channel: OptoChannel) -> Option<Amps> {
        self.core
            .state
            .lock()
            .unwrap()
            .led
            .get(channel.index())
            .copied()
            .flatten()
    }

    /// Whether a channel's LED carries at least the input threshold.
    pub fn is_lit(&self, channel: OptoChannel) -> bool {
        let state = self.core.state.lock().unwrap();
        self.core.lit(&state, channel.index())
    }

    /// The drive a channel's output is presenting, or `None` when the
    /// open-collector stage is released.
    pub fn output_drive(&self, channel: OptoChannel) -> Option<TheveninDrive> {
        self.core
            .state
            .lock()
            .unwrap()
            .applied
            .get(channel.index())
            .copied()
            .flatten()
            .flatten()
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

    /// Whether the enable lets the outputs follow the LEDs.
    pub fn is_enabled(&self) -> bool {
        let state = self.core.state.lock().unwrap();
        self.core.enabled(&state)
    }

    /// Drives issued since construction — the event-cost meter.
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

/// An optocoupler as a live board-engine component: its LEDs are branches
/// the engine solves, its outputs sinks that release.
#[derive(Debug)]
pub struct Opto {
    core: Arc<Core>,
    branches: Vec<Branch>,
}

impl Opto {
    /// An optocoupler from a validated configuration.
    pub fn new(config: Config) -> Result<Self, PartConfigError> {
        config.validate()?;
        tracing::info!(
            part = config.part,
            threshold_amps = config.threshold_amps,
            supply_min_volts = config.supply_min_volts,
            "opto: init"
        );
        let channels = config.channels.len();
        let branches = config.branches();
        Ok(Self {
            core: Arc::new(Core {
                config,
                state: Mutex::new(OptoState {
                    vcc: NetState::Floating,
                    enable: None,
                    led: vec![None; channels],
                    output: vec![None; channels],
                    applied: vec![None; channels],
                    drives: 0,
                }),
            }),
            branches,
        })
    }

    /// A Vishay VO2631.
    pub fn vo2631() -> Self {
        Self::new(Config::vo2631()).expect("the datasheet configuration is valid")
    }

    /// A Lite-On 6N137.
    pub fn lite_on_6n137() -> Self {
        Self::new(Config::lite_on_6n137()).expect("the datasheet configuration is valid")
    }

    /// A read handle onto this optocoupler.
    pub fn monitor(&self) -> OptoMonitor {
        OptoMonitor {
            core: Arc::clone(&self.core),
        }
    }

    /// The validated configuration in force.
    pub fn config(&self) -> &Config {
        &self.core.config
    }
}

impl Component for Opto {
    fn pins(&self) -> &[PinDecl] {
        &self.core.config.pins
    }

    fn branches(&self) -> &[Branch] {
        &self.branches
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let channels = self.core.config.channels.clone();
        {
            let mut state = self.core.state.lock().unwrap();
            for (index, channel) in channels.iter().enumerate() {
                state.output[index] = Some(io.pin(channel.output)?);
            }
        }
        // The detector supply and the enable first, so an LED current
        // delivered before either is known does not sink through an
        // unpowered or disabled stage.
        {
            let core = Arc::clone(&self.core);
            io.on_sense(self.core.config.vcc, move |sensed| {
                let mut state = core.state.lock().unwrap();
                state.vcc = sensed;
                core.refresh_all(&mut state);
            })?;
        }
        if let Some(enable) = self.core.config.enable {
            let core = Arc::clone(&self.core);
            io.on_sense(enable, move |sensed| {
                let mut state = core.state.lock().unwrap();
                state.enable = Some(sensed);
                core.refresh_all(&mut state);
            })?;
        }
        // The LED currents, from the solve the loop is part of.
        for (index, channel) in channels.iter().enumerate() {
            let core = Arc::clone(&self.core);
            io.on_branch(channel.anode, move |amps| {
                let mut state = core.state.lock().unwrap();
                state.led[index] = amps;
                core.refresh(&mut state, index);
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

    fn vo2631() -> (Opto, OptoMonitor) {
        let opto = Opto::vo2631();
        let monitor = opto.monitor();
        (opto, monitor)
    }

    fn set_vcc(opto: &Opto, vcc: NetState) {
        let mut state = opto.core.state.lock().unwrap();
        state.vcc = vcc;
        opto.core.refresh_all(&mut state);
    }

    fn set_led(opto: &Opto, channel: OptoChannel, amps: Option<Amps>) {
        let mut state = opto.core.state.lock().unwrap();
        state.led[channel.index()] = amps;
        opto.core.refresh(&mut state, channel.index());
    }

    fn set_enable(opto: &Opto, enable: NetState) {
        let mut state = opto.core.state.lock().unwrap();
        state.enable = Some(enable);
        opto.core.refresh_all(&mut state);
    }

    #[rstest]
    fn the_vo2631_facade_is_the_dip8_pinout_with_one_led_branch_per_channel() {
        let (opto, _) = vo2631();
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
        let branches: Vec<(&str, &str)> = opto.branches().iter().map(|b| (b.a, b.b)).collect();
        assert_eq!(branches, vec![("1", "2"), ("4", "3")]);
        for branch in opto.branches() {
            assert_eq!(
                branch.curve,
                PwlCurve::Diode {
                    vf: VO2631_LED_VF_VOLTS,
                    r_d: 0.0
                }
            );
        }
        for output in ["6", "7"] {
            let pin = opto.pins().iter().find(|p| p.number == output).unwrap();
            assert_eq!(pin.kind, PinKind::DigitalOut);
            assert_eq!(pin.idle, IdleDrive::Released);
        }
    }

    #[rstest]
    fn the_6n137_facade_has_one_channel_and_an_enable_and_no_pin_4() {
        let opto = Opto::lite_on_6n137();
        let numbers: Vec<&str> = opto.pins().iter().map(|p| p.number).collect();
        assert_eq!(numbers, vec!["1", "2", "3", "5", "6", "7", "8"]);
        let branches: Vec<(&str, &str)> = opto.branches().iter().map(|b| (b.a, b.b)).collect();
        assert_eq!(branches, vec![("2", "3")]);
        assert_eq!(opto.config().enable, Some("7"));
    }

    /// The truth table: the output sinks only with the LED at or above
    /// threshold and the detector powered.
    #[rstest]
    #[case::dark_unpowered(None, NetState::Floating, false)]
    #[case::dark_powered(Some(0.0), NetState::Analog(5.0), false)]
    #[case::under_threshold(Some(4.9e-3), NetState::Analog(5.0), false)]
    #[case::at_threshold(Some(5e-3), NetState::Analog(5.0), true)]
    #[case::lit_unpowered(Some(10e-3), NetState::Floating, false)]
    #[case::lit_supply_low(Some(10e-3), NetState::Analog(4.0), false)]
    #[case::lit_powered(Some(10e-3), NetState::Analog(5.0), true)]
    #[case::lit_projected_rail(Some(10e-3), NetState::Pulled(Level::High, 100.0), true)]
    fn the_output_sinks_only_lit_and_powered(
        #[case] amps: Option<Amps>,
        #[case] vcc: NetState,
        #[case] sinks: bool,
    ) {
        let (opto, monitor) = vo2631();
        set_vcc(&opto, vcc);
        set_led(&opto, OptoChannel::Two, amps);
        assert_eq!(monitor.is_sinking(OptoChannel::Two), sinks);
        assert!(!monitor.is_sinking(OptoChannel::One));
        if sinks {
            assert_eq!(
                monitor.output_drive(OptoChannel::Two),
                Some(TheveninDrive {
                    volts: 0.0,
                    impedance: VO2631_OUTPUT_IMPEDANCE_OHMS
                })
            );
        }
    }

    /// The 6N137's enable: low holds the output released, high or open
    /// lets it follow the LED.
    #[rstest]
    #[case::low(NetState::Driven(Level::Low), false)]
    #[case::analog_low(NetState::Analog(0.5), false)]
    #[case::analog_in_the_band(NetState::Analog(1.5), false)]
    #[case::analog_at_v_eh(NetState::Analog(LITE_ON_6N137_ENABLE_HIGH_MIN_VOLTS), true)]
    #[case::high(NetState::Driven(Level::High), true)]
    #[case::analog_high(NetState::Analog(5.0), true)]
    #[case::open(NetState::Floating, true)]
    fn the_enable_holds_the_output_released_only_when_low(
        #[case] enable: NetState,
        #[case] sinks: bool,
    ) {
        let opto = Opto::lite_on_6n137();
        let monitor = opto.monitor();
        set_vcc(&opto, NetState::Analog(5.0));
        set_led(&opto, OptoChannel::One, Some(10e-3));
        assert!(
            monitor.is_sinking(OptoChannel::One),
            "enabled by default: no enable seen"
        );
        set_enable(&opto, enable);
        assert_eq!(monitor.is_sinking(OptoChannel::One), sinks);
        assert_eq!(monitor.is_enabled(), sinks);
    }

    /// Drive on change: re-delivering the same current costs no drive.
    #[rstest]
    fn a_repeated_delivery_costs_no_drive() {
        let (opto, monitor) = vo2631();
        set_vcc(&opto, NetState::Analog(5.0));
        set_led(&opto, OptoChannel::One, Some(10e-3));
        let after_on = monitor.drive_count();
        set_led(&opto, OptoChannel::One, Some(10e-3));
        set_led(&opto, OptoChannel::One, Some(12e-3));
        assert_eq!(monitor.drive_count(), after_on);
        set_led(&opto, OptoChannel::One, Some(1e-3));
        assert_eq!(monitor.drive_count(), after_on + 1);
        assert!(!monitor.is_sinking(OptoChannel::One));
    }

    #[rstest]
    fn the_datasheet_constants_are_the_derived_figures() {
        assert!((VO2631_OUTPUT_IMPEDANCE_OHMS - 46.154).abs() < 1e-3);
        assert!((LITE_ON_6N137_OUTPUT_IMPEDANCE_OHMS - 46.154).abs() < 1e-3);
        assert!(Config::vo2631().validate().is_ok());
        assert!(Config::lite_on_6n137().validate().is_ok());
        let mut bad = Config::vo2631();
        bad.threshold_amps = 0.0;
        assert!(bad.validate().is_err());
    }
}
