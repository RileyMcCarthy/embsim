//! Model: the TI **AM26LV32** low-voltage quadruple differential line
//! receiver — four RS-422 pairs turned into logic outputs, the encoder
//! `A±` / `B±` / `ZI±` pairs on the MaD Edge board (`U25`).
//!
//! ```text
//!   nA ──┐                  G  (active high) ─┐
//!        ├─▷──► nY          ~G (active low)  ─┴─ common to all four receivers
//!   nB ──┘
//! ```
//!
//! # Datasheet provenance
//!
//! Texas Instruments, *AM26LV32 Low-Voltage, High-Speed Quadruple
//! Differential Line Receiver*, **SLLS202H** (May 1995, revised August
//! 2023). The C grade (0 °C to 70 °C) and the I grade (−40 °C to 85 °C)
//! share every electrical figure below, so the catalog places both grades'
//! ordering codes on the one kind.
//!
//! - **Pinout** — §5, Figure 5-1 and Table 5-1, the D (SOIC) and NS (SO)
//!   packages: 1 `1B`, 2 `1A`, 3 `1Y`, 4 `G`, 5 `2Y`, 6 `2A`, 7 `2B`,
//!   8 `GND`, 9 `3B`, 10 `3A`, 11 `3Y`, 12 `G̅`, 13 `4Y`, 14 `4A`, 15 `4B`,
//!   16 `V_CC`. [`AM26LV32_PINS`].
//! - **Channel behaviour and the enables** — §8.4, Table 8-1 "Function
//!   Table (Each Receiver)": with `G` high or `G̅` low, `Y` is high for
//!   `V_ID` ≥ 0.2 V and low for `V_ID` ≤ −0.2 V; with `G` low and `G̅` high
//!   it is high impedance. The enable is common to all four receivers (§3).
//! - **Differential thresholds** — §6.5: `V_IT+` max 0.2 V, `V_IT−` min
//!   −0.2 V. [`AM26LV32_VID_THRESHOLD_VOLTS`].
//! - **Enable thresholds** — §6.3 Recommended Operating Conditions:
//!   `V_IH(EN)` min 2 V, `V_IL(EN)` max 0.8 V, absolute against `GND`.
//!   [`AM26LV32_ENABLE_THRESHOLDS`].
//! - **The fail-safe** — §8.4.1: an open input pair, a 100 Ω terminated
//!   one whose driver is off, and a shorted one each put the output high
//!   (Table 8-1's "Open, shorted, or terminated" row). Each input is a port
//!   of its own (`NODES.md` §10), the open fail-safe's mechanism: `r_I`
//!   12 kΩ typical (§6.5, [`AM26LV32_INPUT_OHMS`]) to the input's
//!   open-circuit voltage, 0.83 V on an A input and 0.70 V on a B input,
//!   read off Figure 9-2 "RS422 Port Open-Circuit Voltage vs V_CC" (§9.2.3),
//!   flat from `V_CC` 1.6 V to 3.6 V ([`AM26LV32_OPEN_A_VOLTS`],
//!   [`AM26LV32_OPEN_B_VOLTS`]). An open pair therefore reads +130 mV:
//!   high, by the fail-safe, through the part's own bias and no other
//!   source.
//! - **Output drive, from the part's own supply** — §6.5: `V_OH` min 2.4 V
//!   at `I_OH` = −5 mA and `V_OL` max 0.5 V at `I_OL` = 5 mA, over the
//!   recommended supply range, whose minimum is 3 V (§6.3). Each output is
//!   a Thevenin port: high is `V_CC` (the supply pin against `GND`) behind
//!   `R_OH = (3 V − 2.4 V) / 5 mA = 120 Ω` ([`AM26LV32_R_OH_OHMS`]), low is
//!   0 V behind `R_OL = 0.5 V / 5 mA = 100 Ω` ([`AM26LV32_R_OL_OHMS`]) —
//!   the worst case each line guarantees, met exactly at its test
//!   condition, as `crate::am26ls31` reads its own lines.
//! - **Supply** — §6.3: `V_CC` 3 V min, 3.3 V nominal, **3.6 V max**; §6.1
//!   absolute maximum 6 V. Under 3 V the outputs are released. Above 3.6 V
//!   the part runs — `V_OH` follows the supply. The supply pin **declares**
//!   the range and the absolute maximum ([`AM26LV32_VCC_LIMITS`]), and the
//!   engine checks them against the solved net: it raises
//!   [`embsim_board::Finding::PinAboveRecommended`] once each time the
//!   supply rises above 3.6 V, noting that the open-input bias Figure 9-2
//!   draws is not characterised there ([`AM26LV32_SUPPLY_NOTE`]). The Edge
//!   board runs `U25` from `SC_5V`, so it is raised there.
//!
//! # Deliberate simplifications
//!
//! - **Inside the ±200 mV band the output is high.** Table 8-1 calls a
//!   driven pair inside the band indeterminate; the fail-safe of §8.4.1
//!   makes the open, shorted and idle-terminated pairs — every pair that
//!   rests inside it with no driver fighting the bias — high, and the model
//!   reads every in-band pair as that. An input with no voltage (a clock,
//!   which names no operating voltage, or a node measured against a ground
//!   nothing holds) is read the same way.
//! - **The ports are declarations**, stamped once and never republished
//!   (`NODES.md` §10): they hold the flat-band figure whatever the supply,
//!   so the ports' fall below `V_CC` 1.6 V (Figure 9-2) and their unknown
//!   behaviour above 3.6 V are not modelled. `r_I`'s 7 kΩ minimum is not
//!   modelled.
//! - **No hysteresis.** The 50 mV typical input hysteresis (§1) has no
//!   guaranteed figure and is not modelled.
//! - **`V_CC` is the supply pin's voltage against `GND`**, and the outputs
//!   drive it above 0 V in the engine's frame — exact while `GND` sits at
//!   0 V there, as the line driver and the isolators assume.
//! - **Not modelled**: propagation delay (`t_PLH`/`t_PHL` 20 ns max, §6.6)
//!   and skew, the enable and disable times, the common-mode input range
//!   (−0.3 V to 5.5 V, §6.3), the output current limit (±25 mA absolute,
//!   §6.1), supply current, ESD and thermals.

use std::sync::{Arc, Mutex};

use embsim_board::{
    AttachError, Component, ComponentNetIo, DeadBand, DigitalReceiver, Drive, InputPort, Level,
    Ohms, PinDecl, PinHandle, PinLimits, TheveninDrive, Thresholds, Volts,
};

// ============================================================
// Datasheet constants
// ============================================================

/// `G` and `G̅` read against `GND`, absolute: `V_IL(EN)` max 0.8 V,
/// `V_IH(EN)` min 2 V (SLLS202H §6.3). No enable hysteresis is given, so
/// between the two neither level is guaranteed ([`DeadBand::Unknown`]).
pub const AM26LV32_ENABLE_THRESHOLDS: Thresholds =
    Thresholds::new(0.8, 2.0, 0.0, DeadBand::Unknown);

/// The differential thresholds' magnitude: `V_IT+` max +0.2 V, `V_IT−` min
/// −0.2 V (SLLS202H §6.5).
pub const AM26LV32_VID_THRESHOLD_VOLTS: Volts = 0.200;

/// `r_I`, the input resistance: 12 kΩ typical (SLLS202H §6.5; 7 kΩ
/// minimum). Each input's own port.
pub const AM26LV32_INPUT_OHMS: Ohms = 12_000.0;

/// The open-circuit voltage of an A (non-inverting) input, 0.83 V: Figure
/// 9-2, "RS422 Port Open-Circuit Voltage vs V_CC" (SLLS202H §9.2.3), curve
/// A, flat from `V_CC` 1.6 V to 3.6 V.
pub const AM26LV32_OPEN_A_VOLTS: Volts = 0.83;

/// The open-circuit voltage of a B (inverting) input, 0.70 V: the same
/// figure's curve B.
pub const AM26LV32_OPEN_B_VOLTS: Volts = 0.70;

/// An A input's own port: [`AM26LV32_INPUT_OHMS`] to
/// [`AM26LV32_OPEN_A_VOLTS`].
pub const AM26LV32_A_PORT: InputPort = InputPort {
    v_bias: AM26LV32_OPEN_A_VOLTS,
    r_in: AM26LV32_INPUT_OHMS,
};

/// A B input's own port: [`AM26LV32_INPUT_OHMS`] to
/// [`AM26LV32_OPEN_B_VOLTS`].
pub const AM26LV32_B_PORT: InputPort = InputPort {
    v_bias: AM26LV32_OPEN_B_VOLTS,
    r_in: AM26LV32_INPUT_OHMS,
};

/// `V_CC` minimum: 3 V (SLLS202H §6.3). Below it the outputs are released.
pub const AM26LV32_VCC_MIN_VOLTS: Volts = 3.0;

/// `V_CC` maximum recommended: 3.6 V (SLLS202H §6.3). Above it the part
/// runs and the engine raises [`embsim_board::Finding::PinAboveRecommended`]
/// from [`AM26LV32_VCC_LIMITS`].
pub const AM26LV32_VCC_MAX_VOLTS: Volts = 3.6;

/// `V_CC`'s absolute maximum rating: 6 V (SLLS202H §6.1).
pub const AM26LV32_VCC_ABS_MAX_VOLTS: Volts = 6.0;

/// `V_OH` min: 2.4 V at `I_OH` = −5 mA (SLLS202H §6.5).
pub const AM26LV32_VOH_MIN_VOLTS: Volts = 2.4;

/// `V_OL` max: 0.5 V at `I_OL` = 5 mA (SLLS202H §6.5).
pub const AM26LV32_VOL_MAX_VOLTS: Volts = 0.5;

/// The current both output lines are tested at, amperes: `I_OH` = −5 mA
/// and `I_OL` = 5 mA (SLLS202H §6.5; §6.3 gives the same as the
/// recommended output currents).
pub const AM26LV32_OUTPUT_TEST_AMPS: f64 = 0.005;

/// High-level output impedance: `(V_CC min 3 V − V_OH min 2.4 V) / 5 mA` =
/// 120 Ω (SLLS202H §6.3, §6.5).
pub const AM26LV32_R_OH_OHMS: Ohms =
    (AM26LV32_VCC_MIN_VOLTS - AM26LV32_VOH_MIN_VOLTS) / AM26LV32_OUTPUT_TEST_AMPS;

/// Low-level output impedance: `V_OL max 0.5 V / 5 mA` = 100 Ω (SLLS202H
/// §6.5).
pub const AM26LV32_R_OL_OHMS: Ohms = AM26LV32_VOL_MAX_VOLTS / AM26LV32_OUTPUT_TEST_AMPS;

/// What a supply above [`AM26LV32_VCC_MAX_VOLTS`] costs, as the finding
/// says it: the open-input bias the fail-safe rests on is drawn only up to
/// 3.6 V (SLLS202H Figure 9-2). The finding names the 6 V absolute maximum
/// (§6.1) from the declaration itself.
pub const AM26LV32_SUPPLY_NOTE: &str =
    "the part runs, but its open-input bias is not characterised above 3.6 V";

/// `V_CC`'s declared operating limits, against `GND`: the recommended 3 V to
/// 3.6 V (SLLS202H §6.3) and the 6 V absolute maximum (§6.1), which the
/// engine checks against the supply pin's solved net
/// ([`embsim_board::PinLimits`]).
pub const AM26LV32_VCC_LIMITS: PinLimits = PinLimits {
    recommended: (AM26LV32_VCC_MIN_VOLTS, AM26LV32_VCC_MAX_VOLTS),
    absolute_max: Some(AM26LV32_VCC_ABS_MAX_VOLTS),
    note: AM26LV32_SUPPLY_NOTE,
};

// ============================================================
// Pin table
// ============================================================

/// A differential input: an analog reader measured against `GND`, with
/// its own port.
const fn input(number: &'static str, name: &'static str, port: InputPort) -> PinDecl {
    PinDecl::analog(number)
        .with_input(port)
        .with_reference("8")
        .with_name(name)
}

/// An enable, reading its thresholds against `GND`.
const fn enable(number: &'static str, name: &'static str) -> PinDecl {
    PinDecl::digital_in(number, AM26LV32_ENABLE_THRESHOLDS)
        .with_reference("8")
        .with_name(name)
}

/// A three-state output, released until the part drives it.
const fn output(number: &'static str, name: &'static str) -> PinDecl {
    PinDecl::digital_out(number).with_idle(None).with_name(name)
}

/// The AM26LV32's 16-pin table, by pin number (SLLS202H §5, Figure 5-1 and
/// Table 5-1: the D and NS packages). The supply is measured against
/// `GND`.
pub const AM26LV32_PINS: [PinDecl; 16] = [
    input("1", "1B", AM26LV32_B_PORT),
    input("2", "1A", AM26LV32_A_PORT),
    output("3", "1Y"),
    enable("4", "G"),
    output("5", "2Y"),
    input("6", "2A", AM26LV32_A_PORT),
    input("7", "2B", AM26LV32_B_PORT),
    PinDecl::power_in("8").with_name("GND"),
    input("9", "3B", AM26LV32_B_PORT),
    input("10", "3A", AM26LV32_A_PORT),
    output("11", "3Y"),
    enable("12", "~G"),
    output("13", "4Y"),
    input("14", "4A", AM26LV32_A_PORT),
    input("15", "4B", AM26LV32_B_PORT),
    PinDecl::power_in("16")
        .with_reference("8")
        .with_name("VCC")
        .with_limits(AM26LV32_VCC_LIMITS),
];

/// `(A, B, Y)` for each of the four receivers (Table 5-1).
pub const AM26LV32_CHANNELS: [(&str, &str, &str); 4] = [
    ("2", "1", "3"),
    ("6", "7", "5"),
    ("10", "9", "11"),
    ("14", "15", "13"),
];

/// The active-high enable `G` and the active-low enable `G̅` (Table 5-1).
const ENABLE_HIGH: &str = "4";
const ENABLE_LOW: &str = "12";
const VCC: &str = "16";

// ============================================================
// Core
// ============================================================

/// What an output presents: a level at the supply it is driven from, or
/// released (`None`).
type Out = Option<(Level, Volts)>;

#[derive(Debug, Default)]
struct State {
    /// `V_CC` against `GND`, when it names a voltage.
    vcc: Option<Volts>,
    enable_high: Option<Level>,
    enable_low: Option<Level>,
    /// `(A volts, B volts)` per channel, against `GND`.
    inputs: [(Option<Volts>, Option<Volts>); 4],
    outputs: Vec<PinHandle>,
    /// What each output last published (`None` in the outer option before
    /// the first publish); an unchanged output is not re-issued.
    applied: [Option<Out>; 4],
}

#[derive(Debug)]
struct Core {
    state: Mutex<State>,
}

/// An output port: high is `V_CC` behind [`AM26LV32_R_OH_OHMS`], low is
/// 0 V behind [`AM26LV32_R_OL_OHMS`] (SLLS202H §6.5).
fn port(level: Level, vcc: Volts) -> TheveninDrive {
    match level {
        Level::High => TheveninDrive {
            volts: vcc,
            impedance: AM26LV32_R_OH_OHMS,
        },
        Level::Low => TheveninDrive {
            volts: 0.0,
            impedance: AM26LV32_R_OL_OHMS,
        },
    }
}

/// The level a channel's pair resolves to (Table 8-1), the fail-safe
/// covering the band and an input with no voltage (§8.4.1).
fn channel_level((a, b): (Option<Volts>, Option<Volts>)) -> Level {
    match (a, b) {
        (Some(a), Some(b)) if a - b <= -AM26LV32_VID_THRESHOLD_VOLTS => Level::Low,
        _ => Level::High,
    }
}

impl Core {
    /// Table 8-1: the outputs are active when `G` is high or `G̅` is low.
    fn enabled(state: &State) -> bool {
        state.enable_high == Some(Level::High) || state.enable_low == Some(Level::Low)
    }

    /// Re-drive every output whose level changed: from its pair when the
    /// part is powered and enabled, released otherwise.
    fn apply(state: &mut State) {
        let rail = state
            .vcc
            .filter(|&vcc| vcc >= AM26LV32_VCC_MIN_VOLTS)
            .filter(|_| Self::enabled(state));
        for channel in 0..state.outputs.len() {
            let desired: Out = rail.map(|vcc| (channel_level(state.inputs[channel]), vcc));
            if state.applied[channel] == Some(desired) {
                continue;
            }
            state.applied[channel] = Some(desired);
            let y = &state.outputs[channel];
            match desired {
                Some((level, vcc)) => y.drive(Drive::Thevenin(port(level, vcc))),
                None => y.release(),
            }
        }
    }
}

// ============================================================
// Component
// ============================================================

/// The TI AM26LV32 as a live board-engine component — see the module docs.
///
/// ```rust
/// use embsim_board::Component;
/// use embsim_models::am26lv32::Am26lv32;
///
/// let receiver = Am26lv32::new();
/// assert_eq!(receiver.pins().len(), 16);
/// ```
#[derive(Debug)]
pub struct Am26lv32 {
    core: Arc<Core>,
}

impl Default for Am26lv32 {
    fn default() -> Self {
        Self::new()
    }
}

impl Am26lv32 {
    /// A receiver; its outputs drive from whatever its own `V_CC` pin is at.
    pub fn new() -> Self {
        Self {
            core: Arc::new(Core {
                state: Mutex::new(State::default()),
            }),
        }
    }
}

impl Component for Am26lv32 {
    fn pins(&self) -> &[PinDecl] {
        &AM26LV32_PINS
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        // Output handles first: a sense callback registered below fires at
        // once with the current state and must find them.
        {
            let mut state = self.core.state.lock().unwrap();
            for (_, _, y) in AM26LV32_CHANNELS {
                state.outputs.push(io.pin(y)?);
            }
        }

        let core = Arc::clone(&self.core);
        io.on_sense(VCC, move |sensed| {
            let mut state = core.state.lock().unwrap();
            state.vcc = sensed.volts;
            Core::apply(&mut state);
        })?;
        for (pin, active_high) in [(ENABLE_HIGH, true), (ENABLE_LOW, false)] {
            let core = Arc::clone(&self.core);
            let receiver = DigitalReceiver::new(io.pin(pin)?);
            io.on_sense(pin, move |sensed| {
                let mut state = core.state.lock().unwrap();
                let level = receiver.read(&sensed);
                if active_high {
                    state.enable_high = level;
                } else {
                    state.enable_low = level;
                }
                Core::apply(&mut state);
            })?;
        }
        for (channel, (a, b, _)) in AM26LV32_CHANNELS.into_iter().enumerate() {
            for (pin, is_a) in [(a, true), (b, false)] {
                let core = Arc::clone(&self.core);
                io.on_sense(pin, move |sensed| {
                    let mut state = core.state.lock().unwrap();
                    if is_a {
                        state.inputs[channel].0 = sensed.volts;
                    } else {
                        state.inputs[channel].1 = sensed.volts;
                    }
                    Core::apply(&mut state);
                })?;
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use rstest::rstest;

    use super::*;

    /// The two ports meet the datasheet's output lines exactly at their
    /// test condition.
    #[rstest]
    fn the_output_ports_meet_the_datasheet_lines() {
        let amps =
            |drive: TheveninDrive, load_volts: Volts| (drive.volts - load_volts) / drive.impedance;
        let high = port(Level::High, AM26LV32_VCC_MIN_VOLTS);
        let sourced = amps(high, AM26LV32_VOH_MIN_VOLTS);
        assert!(
            (sourced - AM26LV32_OUTPUT_TEST_AMPS).abs() < 1e-12,
            "{sourced}"
        );
        let low = port(Level::Low, AM26LV32_VCC_MIN_VOLTS);
        let sunk = amps(low, AM26LV32_VOL_MAX_VOLTS);
        assert!((sunk + AM26LV32_OUTPUT_TEST_AMPS).abs() < 1e-12, "{sunk}");
        assert!((AM26LV32_R_OH_OHMS - 120.0).abs() < 1e-9);
        assert!((AM26LV32_R_OL_OHMS - 100.0).abs() < 1e-9);
    }

    /// Every pin of the 16-pin table, once; every channel's pins are two
    /// analog inputs with their own ports and an output that rests
    /// released.
    #[rstest]
    fn the_pin_table_is_figure_5_1() {
        let numbers: Vec<&str> = AM26LV32_PINS.iter().map(|pin| pin.number).collect();
        let expected: Vec<String> = (1..=16).map(|n| n.to_string()).collect();
        assert_eq!(numbers, expected);
        let pin = |number: &str| {
            AM26LV32_PINS
                .iter()
                .find(|pin| pin.number == number)
                .expect("a pin of the table")
        };
        for (a, b, y) in AM26LV32_CHANNELS {
            assert_eq!(pin(a).input, Some(AM26LV32_A_PORT), "{a}");
            assert_eq!(pin(b).input, Some(AM26LV32_B_PORT), "{b}");
            for input in [a, b] {
                assert!(pin(input).thresholds.is_none() && !pin(input).can_source);
            }
            assert!(pin(y).can_source && pin(y).can_sink, "{y}");
            assert_eq!(pin(y).idle, None, "{y} rests released");
        }
        assert_eq!(pin(VCC).reference, Some("8"));
        // SLLS202H §6.3 and §6.1: 3 V to 3.6 V recommended, 6 V absolute.
        let limits = pin(VCC).limits.expect("VCC declares its limits");
        assert_eq!(limits.recommended, (3.0, 3.6));
        assert_eq!(limits.absolute_max, Some(6.0));
    }
}
