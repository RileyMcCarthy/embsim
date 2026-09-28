//! Model: a **clock oscillator** whose output is a *rate* — the Seiko Epson
//! **TG2016SMN / TG2520SMN** temperature-compensated crystal oscillator
//! (TCXO), the `X100` on a Parallax P2-EC32MB module.
//!
//! An oscillator's information is its frequency, and its edge count runs
//! ahead of everything else on a board: 20 MHz is forty million edges a
//! second, each of which would be a drive, a resolution pass and a sense
//! delivery. So the output pin publishes **one**
//! [`embsim_board::Drive::Periodic`] at the frequency parsed from the part's
//! value — the encoding a step clock uses (`NODES.md` §2, the crystal /
//! oscillator row; §10 row 3), one drive, nothing per edge — and retracts
//! it with a held segment when the supply drops.
//!
//! # Datasheet provenance
//!
//! Seiko Epson product brief **"TG2016SMN / TG2520SMN — TCXO / VC-TCXO,
//! high stability / low noise"** (© Seiko Epson Corporation 2025; product
//! numbers TG2016SMN X1G005441xxxx25, TG2520SMN X1G005421xxxx27), the
//! "Specifications (characteristics)" table and the "Product Name (Standard
//! form)" ordering key:
//!
//! - **Frequency** — field ③ of the ordering key, printed as
//!   `26.000000MHz` in the standard form; the module netlist's value
//!   `TG2520SMN 20.0000M-ECGNNM3` carries it as `20.0000M`.
//!   [`parse_oscillator_hz`] reads both spellings.
//! - **Supply** — "Supply voltage range: 1.7 V to 3.63 V" (remarks column
//!   of `V_CC`); option letter ④ selects the nominal, `E` = 1.8 V ± 0.1 V,
//!   which is the module's part. [`TG2520SMN_SUPPLY_MIN_VOLTS`],
//!   [`TG2520SMN_NOMINAL_SUPPLY_VOLTS`].
//! - **Start-up time** — `t_str` 1.0 ms Max, "t = 0 at 90 % V_CC".
//!   [`TG2520SMN_START_UP_NS`]. The model publishes its rate at exactly
//!   that instant after it sees its supply up: a scheduled wake, never a
//!   state committed ahead of virtual time (`DESIGN.md` rule 3).
//! - **Output** — option ② `S`: clipped sine wave; `V_pp` 0.8 V Min, peak
//!   to peak; symmetry 40 % to 60 % "GND level (DC cut)"; output load
//!   10 kΩ // 10 pF with "DC cut capacitor = 0.01 µF". The datasheet
//!   characterises the output **into a DC-blocking capacitor** and names
//!   no DC level and no source impedance for it — so the periodic drive's
//!   two phases are **released** (an infinite impedance: they source
//!   nothing at DC), the pin rests released before start-up, and the net on
//!   its side of the coupling capacitor reads floating at DC, which is what
//!   a node with a capacitor and nothing else on it is. The phases' two
//!   voltages are the datasheet's swing — `V_pp` min above the GND level
//!   the symmetry is specified at ([`TG2520SMN_VPP_MIN_VOLTS`]) — which is
//!   what the rate carries across the capacitor by the engine's AC-coupling
//!   rule (`embsim_board::engine`, `overlay_arrivals`): a swing below any
//!   logic threshold, which is why the module squares it with a
//!   self-biased inverter.
//! - **Pins** — the pin map: 1 `N.C.` ("please keep N.C. pin OPEN
//!   condition or GND connection"), 2 `GND`, 3 `OUT`, 4 `V_CC`.
//!
//! # Deliberate simplifications
//!
//! - **Frequency tolerance, aging and the temperature characteristic**
//!   (±0.5 ppm) are not modelled: the published rate is the nominal.
//! - **Symmetry** is not carried by the drive (a [`PeriodicSchedule`] has a
//!   rate and no duty).
//! - **The supply threshold is one value.** The datasheet's `t_str` counts
//!   from 90 % of `V_CC`; here a supply at or above the minimum operating
//!   voltage is up, and the start-up clock runs from the instant the model
//!   sees it so.

use std::sync::{Arc, Mutex};

use embsim_board::{
    AttachError, Component, ComponentNetIo, Drive, PeriodicSchedule, PinDecl, PinHandle, Sense,
    TheveninDrive, Volts,
};
use embsim_core::virtual_clock;

use crate::isolation::supply_up;

// ============================================================
// Datasheet constants
// ============================================================

/// `t_str`, start-up time: 1.0 ms Max (TG2520SMN specifications table,
/// "Start-up time", condition "t = 0 at 90 % V_CC").
pub const TG2520SMN_START_UP_NS: u64 = 1_000_000;

/// The bottom of the supply voltage range, 1.7 V ("Supply voltage range:
/// 1.7 V to 3.63 V", the `V_CC` row's remarks).
pub const TG2520SMN_SUPPLY_MIN_VOLTS: Volts = 1.7;

/// The `E` supply option, 1.8 V typical (ordering key field ④, "E: 1.8"),
/// which the module's `…-ECGNNM3` part is.
pub const TG2520SMN_NOMINAL_SUPPLY_VOLTS: Volts = 1.8;

/// Output voltage, clipped sine: `V_pp` 0.8 V Min, peak to peak, with the
/// symmetry specified at the GND level, DC cut (TG2520SMN specifications
/// table, "Output voltage" and "Symmetry"). The high phase of the periodic
/// drive sits this far above the low phase's 0 V — the swing the coupling
/// capacitor passes.
pub const TG2520SMN_VPP_MIN_VOLTS: Volts = 0.8;

// ============================================================
// The value field
// ============================================================

/// The frequency an oscillator's value field names, in hertz.
///
/// Reads the number-with-unit token of a value such as
/// `"TG2520SMN 20.0000M-ECGNNM3"` (the P2-EC32MB netlist) or
/// `"TG2520SMN 26.000000MHz E C G N N M"` (the datasheet's standard form):
/// the first token that is a decimal number followed by `M` or `MHz`,
/// case-insensitively, with any ordering suffix after a `-` ignored.
/// `None` when no token says a frequency.
pub fn parse_oscillator_hz(value: &str) -> Option<u32> {
    value.split_whitespace().find_map(|token| {
        // "20.0000M-ECGNNM3" → "20.0000M".
        let token = token.split('-').next().unwrap_or(token);
        let lower = token.to_ascii_lowercase();
        let digits = lower
            .strip_suffix("mhz")
            .or_else(|| lower.strip_suffix('m'))?;
        if digits.is_empty()
            || !digits.chars().all(|c| c.is_ascii_digit() || c == '.')
            || digits.matches('.').count() > 1
            || !digits.chars().any(|c| c.is_ascii_digit())
        {
            return None;
        }
        let megahertz: f64 = digits.parse().ok()?;
        let hertz = (megahertz * 1e6).round();
        (hertz > 0.0 && hertz <= f64::from(u32::MAX)).then_some(hertz as u32)
    })
}

// ============================================================
// Configuration
// ============================================================

/// Oscillator configuration.
#[derive(Debug, Clone, PartialEq)]
pub struct Config {
    /// The nominal output frequency.
    pub hz: u32,
    /// Start-up time from the supply reaching its minimum to the first
    /// published rate ([`TG2520SMN_START_UP_NS`]).
    pub start_up_ns: u64,
    /// Supply at or above which the part runs
    /// ([`TG2520SMN_SUPPLY_MIN_VOLTS`]).
    pub supply_min_volts: Volts,
}

impl Config {
    /// A TG2520SMN at `hz`.
    pub const fn tg2520smn(hz: u32) -> Self {
        Self {
            hz,
            start_up_ns: TG2520SMN_START_UP_NS,
            supply_min_volts: TG2520SMN_SUPPLY_MIN_VOLTS,
        }
    }

    /// A TG2520SMN at the frequency its value field names
    /// ([`parse_oscillator_hz`]).
    pub fn from_value(value: &str) -> Option<Self> {
        parse_oscillator_hz(value).map(Self::tg2520smn)
    }
}

// ============================================================
// Pin facades
// ============================================================

/// The output pin: rests released — the datasheet characterises the
/// clipped-sine output into a DC-blocking capacitor and names no DC level
/// for it, so none is driven.
const fn out_pin(number: &'static str) -> PinDecl {
    PinDecl::digital_out(number).with_idle(None)
}

/// The 2520 / 2016 package keyed by **function**, as the P2-EC32MB netlist
/// names `X100`'s pins (`NC_GND` is the vendor's option pad, pin 1). The
/// supply is measured against the ground pin.
pub const TCXO_PINS_BY_FUNCTION: [PinDecl; 4] = [
    PinDecl::power_in("VCC").with_reference("GND"),
    PinDecl::power_in("GND"),
    out_pin("OUT"),
    PinDecl::passive("NC_GND"),
];

/// The same package keyed by pin **number** (the datasheet's pin map:
/// 1 `N.C.`, 2 `GND`, 3 `OUT`, 4 `V_CC`), for a KiCad export.
pub const TCXO_PINS_NUMBERED: [PinDecl; 4] = [
    PinDecl::passive("1").with_name("NC"),
    PinDecl::power_in("2").with_name("GND"),
    out_pin("3").with_name("OUT"),
    PinDecl::power_in("4").with_name("VCC").with_reference("2"),
];

// ============================================================
// Core
// ============================================================

#[derive(Debug)]
struct State {
    /// The supply, as last sensed (against `GND`).
    supply: Sense,
    /// The deadline armed for the current start-up, if one is pending.
    armed_at: Option<u64>,
    /// The segment currently published: `Some(segment)` while running.
    published: Option<PeriodicSchedule>,
    out: Option<PinHandle>,
    /// Drives issued — the event-cost meter.
    publishes: u64,
}

#[derive(Debug)]
struct Core {
    config: Config,
    state: Mutex<State>,
}

impl Core {
    fn up(&self, state: &State) -> bool {
        supply_up(&state.supply, self.config.supply_min_volts)
    }

    /// Publish a segment once, on change only: a periodic drive whose two
    /// phases are released at DC and swing by the datasheet's `V_pp` (the
    /// module docs), a held segment when the part stops.
    fn publish(&self, state: &mut State, segment: Option<PeriodicSchedule>) {
        if state.published == segment {
            return;
        }
        state.published = segment;
        state.publishes += 1;
        if let Some(out) = &state.out {
            out.drive(clock_drive(segment.unwrap_or(PeriodicSchedule::IDLE)));
        }
    }

    /// The supply changed: arm the start-up on the way up, retract the rate
    /// on the way down.
    fn on_supply(&self, state: &mut State, sensed: Sense, io: &ComponentNetIo) {
        let was_up = self.up(state);
        state.supply = sensed;
        let up = self.up(state);
        match (was_up, up) {
            (false, true) => {
                let deadline = virtual_clock::virtual_ns().saturating_add(self.config.start_up_ns);
                state.armed_at = Some(deadline);
                io.schedule_at_ns(deadline);
            }
            (true, false) => {
                state.armed_at = None;
                // A held train — the retraction is a superseding publish;
                // a part that never ran published nothing and retracts
                // nothing.
                if state.published.is_some() {
                    self.publish(state, None);
                }
            }
            _ => {}
        }
    }

    /// A wake: the start-up elapsed for the supply that armed it.
    fn on_wake(&self, state: &mut State, now_ns: u64) {
        let Some(deadline) = state.armed_at else {
            return;
        };
        if now_ns < deadline || !self.up(state) {
            return;
        }
        state.armed_at = None;
        let segment = PeriodicSchedule {
            emitted: 0,
            freq_hz: self.config.hz,
            total: None,
            since_ns: now_ns,
        };
        self.publish(state, Some(segment));
    }
}

/// The periodic drive the output presents for `segment`: two phases the
/// datasheet names no source impedance for — released at DC — swinging
/// [`TG2520SMN_VPP_MIN_VOLTS`] above 0 V.
pub fn clock_drive(segment: PeriodicSchedule) -> Drive {
    Drive::Periodic {
        hi: TheveninDrive {
            volts: TG2520SMN_VPP_MIN_VOLTS,
            impedance: f64::INFINITY,
        },
        lo: TheveninDrive {
            volts: 0.0,
            impedance: f64::INFINITY,
        },
        segment,
    }
}

// ============================================================
// Monitor
// ============================================================

/// Cheap cloneable read handle onto a live [`Oscillator`].
#[derive(Clone, Debug)]
pub struct OscillatorMonitor {
    core: Arc<Core>,
}

impl OscillatorMonitor {
    /// The segment currently published, `None` while the part is not
    /// running.
    pub fn published(&self) -> Option<PeriodicSchedule> {
        self.core.state.lock().unwrap().published
    }

    /// Whether the part is running (a rate is published).
    pub fn is_running(&self) -> bool {
        self.published().is_some()
    }

    /// Total drives issued since construction: one per start, one per
    /// retraction, nothing per edge.
    pub fn publish_count(&self) -> u64 {
        self.core.state.lock().unwrap().publishes
    }

    /// The configuration in force.
    pub fn config(&self) -> &Config {
        &self.core.config
    }
}

// ============================================================
// Component
// ============================================================

/// A clock oscillator as a live board-engine component.
///
/// ```rust
/// use embsim_board::Component;
/// use embsim_models::oscillator::{Config, Oscillator};
///
/// // The P2-EC32MB's X100, straight off its netlist value.
/// let config = Config::from_value("TG2520SMN 20.0000M-ECGNNM3").expect("a frequency");
/// assert_eq!(config.hz, 20_000_000);
/// let tcxo = Oscillator::new(config);
/// assert_eq!(tcxo.pins().len(), 4);
/// ```
#[derive(Debug)]
pub struct Oscillator {
    pins: &'static [PinDecl],
    core: Arc<Core>,
}

impl Oscillator {
    /// An oscillator with the by-function facade a transcribed netlist
    /// uses; [`Self::with_pins`] for a numbered one.
    pub fn new(config: Config) -> Self {
        Self {
            pins: &TCXO_PINS_BY_FUNCTION,
            core: Arc::new(Core {
                config,
                state: Mutex::new(State {
                    supply: Sense {
                        volts: None,
                        periodic: None,
                        at_ns: 0,
                    },
                    armed_at: None,
                    published: None,
                    out: None,
                    publishes: 0,
                }),
            }),
        }
    }

    /// Declare a different pin facade — [`TCXO_PINS_NUMBERED`] for a netlist
    /// that identifies pins by number. Its pins must be named `VCC`, `GND`
    /// and `OUT` (by number or alias), which is what `attach` looks up.
    pub fn with_pins(mut self, pins: &'static [PinDecl]) -> Self {
        self.pins = pins;
        self
    }

    /// A read handle onto this oscillator.
    pub fn monitor(&self) -> OscillatorMonitor {
        OscillatorMonitor {
            core: Arc::clone(&self.core),
        }
    }
}

impl Component for Oscillator {
    fn pins(&self) -> &[PinDecl] {
        self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        self.core.state.lock().unwrap().out = Some(io.pin("OUT")?);

        let core = Arc::clone(&self.core);
        io.on_wake_ns(move |now_ns| {
            let mut state = core.state.lock().unwrap();
            core.on_wake(&mut state, now_ns);
        });

        let core = Arc::clone(&self.core);
        let scheduler = io.clone();
        io.on_sense("VCC", move |sensed| {
            let mut state = core.state.lock().unwrap();
            core.on_supply(&mut state, sensed, &scheduler);
        })?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use rstest::rstest;

    use super::*;

    /// A supply handed `volts`.
    fn supply(volts: Option<f64>) -> Sense {
        Sense {
            volts,
            periodic: None,
            at_ns: 0,
        }
    }

    #[rstest]
    #[case::module_value("TG2520SMN 20.0000M-ECGNNM3", Some(20_000_000))]
    #[case::datasheet_standard_form("TG2520SMN 26.000000MHz E C G N N M", Some(26_000_000))]
    #[case::bare_megahertz("16M", Some(16_000_000))]
    #[case::fractional("19.2MHz", Some(19_200_000))]
    #[case::lower_case("12.288mhz", Some(12_288_000))]
    #[case::kilohertz_is_not_read("32.768kHz", None)]
    #[case::no_frequency("TCXO", None)]
    #[case::unit_alone("M", None)]
    #[case::two_points("1.2.3M", None)]
    fn the_value_field_names_the_frequency(#[case] value: &str, #[case] hz: Option<u32>) {
        assert_eq!(parse_oscillator_hz(value), hz);
    }

    #[rstest]
    fn the_module_part_is_a_20_megahertz_tcxo_with_the_datasheet_start_up() {
        let config = Config::from_value("TG2520SMN 20.0000M-ECGNNM3").unwrap();
        assert_eq!(config.hz, 20_000_000);
        assert_eq!(config.start_up_ns, 1_000_000, "t_str 1.0 ms Max");
        assert_eq!(config.supply_min_volts, 1.7);
    }

    /// The core without an engine: the supply coming up arms the start-up,
    /// the wake at its deadline publishes the rate once, a supply drop
    /// retracts it once, and a wake before the deadline publishes nothing.
    #[rstest]
    fn the_rate_is_published_once_at_the_start_up_instant_and_retracted_on_a_drop() {
        let osc = Oscillator::new(Config::tg2520smn(20_000_000));
        let core = Arc::clone(&osc.core);
        let mut state = core.state.lock().unwrap();
        let io = ComponentNetIo::default();

        core.on_supply(&mut state, supply(Some(1.8)), &io);
        let deadline = state.armed_at.expect("armed on the way up");
        core.on_wake(&mut state, deadline - 1);
        assert_eq!(state.published, None, "nothing before the deadline");
        core.on_wake(&mut state, deadline);
        let segment = state.published.expect("running at the deadline");
        assert_eq!(segment.freq_hz, 20_000_000);
        assert_eq!(state.publishes, 1);

        core.on_supply(&mut state, supply(Some(1.8)), &io);
        assert_eq!(
            state.publishes, 1,
            "a supply that stays up publishes nothing new"
        );

        core.on_supply(&mut state, supply(Some(1.0)), &io);
        assert_eq!(state.published, None, "retracted below the minimum");
        assert_eq!(state.publishes, 2);
    }

    /// A supply that drops during the start-up cancels it: the wake that
    /// was armed fires into a part whose supply is down.
    #[rstest]
    fn a_supply_that_drops_during_start_up_never_publishes() {
        let osc = Oscillator::new(Config::tg2520smn(20_000_000));
        let core = Arc::clone(&osc.core);
        let mut state = core.state.lock().unwrap();
        let io = ComponentNetIo::default();
        core.on_supply(&mut state, supply(Some(1.8)), &io);
        let deadline = state.armed_at.unwrap();
        core.on_supply(&mut state, supply(None), &io);
        core.on_wake(&mut state, deadline);
        assert_eq!(state.published, None);
        assert_eq!(state.publishes, 0);
    }

    #[rstest]
    fn the_facades_name_the_pins_attach_looks_up() {
        for pins in [&TCXO_PINS_BY_FUNCTION, &TCXO_PINS_NUMBERED] {
            let out = pins
                .iter()
                .find(|p| p.number == "OUT" || p.name == Some("OUT"))
                .expect("an OUT pin");
            assert!(out.drives());
            assert_eq!(out.idle, None, "no DC level is driven");
            assert!(pins
                .iter()
                .any(|p| p.number == "VCC" || p.name == Some("VCC")));
        }
    }

    /// The clock the output drives is released in both phases at DC and
    /// swings by the datasheet's minimum peak-to-peak.
    #[rstest]
    fn the_clock_is_released_at_dc_and_swings_by_v_pp() {
        let Drive::Periodic { hi, lo, segment } = clock_drive(PeriodicSchedule::IDLE) else {
            panic!("a periodic drive");
        };
        assert_eq!((hi.volts, lo.volts), (0.8, 0.0));
        assert!(!hi.impedance.is_finite() && !lo.impedance.is_finite());
        assert_eq!(segment, PeriodicSchedule::IDLE);
    }
}
