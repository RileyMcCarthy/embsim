//! Model: the TI **AM26LS31** quadruple differential line driver — one
//! logic input per channel turned into a complementary RS-422 pair, the
//! servo step and direction pairs on the MaD Edge board (`U24`).
//!
//! ```text
//!   nA ──►▷──┬──► nY        G  (active high) ─┐
//!            └─○► nZ        ~G (active low)  ─┴─ common to all four drivers
//! ```
//!
//! # Datasheet provenance
//!
//! Texas Instruments, *AM26LS31 Quadruple Differential Line Driver*,
//! **SLLS114N** (January 1979, revised June 2026). The model is the
//! AM26LS31**C**'s: the supply range and the output lines below are that
//! grade's, so the catalog seats the kind on `AM26LS31C…` parts only
//! (the M grade runs from 4.5 V, §5.3, and its lines are another part's).
//!
//! - **Pinout** — §4, Figure 4-1 and Table 4-1, shared by the D (SOIC), DB
//!   (SSOP), N (PDIP), NS (SO), J (CDIP) and W (CFP) packages: 1 `1A`,
//!   2 `1Y`, 3 `1Z`, 4 `G`, 5 `2Z`, 6 `2Y`, 7 `2A`, 8 `GND`, 9 `3A`,
//!   10 `3Y`, 11 `3Z`, 12 `G̅`, 13 `4Z`, 14 `4Y`, 15 `4A`, 16 `V_CC`.
//!   [`AM26LS31_PINS`]. The 20-pin FK (LCCC) package (Figure 4-2) is not
//!   tabulated.
//! - **Channel behaviour and the enables** — §7.4, Table 7-1 "Function
//!   Table (Each Driver)": with `G` high or `G̅` low, `Y` follows `A` and
//!   `Z` is its complement; with `G` low and `G̅` high both outputs are high
//!   impedance. §7.3.1 says the same in words: "Setting either G to a logic
//!   HIGH or G̅ to a logic LOW enables the transmitter outputs." The enable
//!   is common to all four drivers (§3).
//! - **Input thresholds** — §5.3 Recommended Operating Conditions: `V_IH`
//!   min 2 V, `V_IL` max 0.8 V, the TTL levels, absolute against `GND`, for
//!   `A` and both enables. No hysteresis is given, so between the two
//!   neither level is guaranteed. [`AM26LS31_INPUT_THRESHOLDS`].
//! - **Output drive, from the part's own supply** — §5.5 Electrical
//!   Characteristics: `V_OH` min 2.5 V at `V_CC` = MIN, `I_OH` = −20 mA, and
//!   `V_OL` max 0.5 V at `V_CC` = MIN, `I_OL` = 20 mA; `V_CC` MIN is 4.75 V
//!   for the C grade (§5.5 note (1), §5.3). Each output is a Thevenin port:
//!   high is `V_CC` (the supply pin against `GND`) behind
//!   `R_OH = (4.75 V − 2.5 V) / 20 mA = 112.5 Ω`
//!   ([`AM26LS31_R_OH_OHMS`]), low is 0 V behind
//!   `R_OL = 0.5 V / 20 mA = 25 Ω` ([`AM26LS31_R_OL_OHMS`]) — the worst
//!   case each line guarantees, met exactly at its own test condition, the
//!   same reading `crate::logic_gate` gives its parts' `V_OH`/`V_OL` lines.
//!   The high port agrees with the short-circuit line: shorted to ground at
//!   `V_CC` MAX (5.25 V) it sources `5.25 V / 112.5 Ω` ≈ 47 mA, inside
//!   `I_OS`'s 30 mA to 150 mA.
//! - **Power** — §5.5 `V_POR`, the power-on-reset threshold with `V_CC`
//!   rising: 2.27 V min, 2.48 V typ, **3.04 V max** (PDIP, SO, SOIC and SSOP
//!   packages). At or above the maximum every part is out of reset and
//!   driving; below it the model releases every output — the high-impedance
//!   power-off state §7.3.2 and §3 describe ("in the high-impedance state in
//!   the power-off condition"). [`AM26LS31_VPOR_MAX_VOLTS`].
//!
//! # A clock crosses as a clock
//!
//! A channel whose `A` carries a square wave ([`embsim_board::PeriodicSense`])
//! whose two phases settle to two different levels through the input
//! thresholds — running, or a held segment that still crosses — drives its
//! pair as two [`Drive::Periodic`]s around that segment: `Y` the input's
//! phases at the part's own ports, `Z` their complement. A wave whose
//! phases settle to one level is that level, and one with a phase inside
//! the threshold band is no level. The same "relay when it crosses"
//! contract as the ISO67xx and the logic gates.
//!
//! # Deliberate simplifications
//!
//! - **An input with no level releases its channel's pair.** The function
//!   table names only H and L inputs; an open input, or one inside the
//!   `V_IL`..`V_IH` band, has neither, and the engine invents no level
//!   (`DESIGN.md` rule 6). The input structure the simplified schematic
//!   draws (§3) is not modelled as a bias, so an open `A` is no level here.
//!   The same holds for an enable: an enable with no level enables nothing,
//!   and the outputs are driven only when the other enable's level does.
//! - **Unused channels are the board's.** All four channels are modelled
//!   and every pin is declared as the datasheet has it; a board that leaves
//!   a channel unwired leaves its input with no level, so that pair rests
//!   released.
//! - **One supply threshold.** `V_BOR`, the falling threshold (2.03 V to
//!   2.26 V), and the 350 mV hysteresis between it and `V_POR` are not
//!   modelled: the outputs drive at or above `V_POR` max and release below
//!   it, rising or falling.
//! - **`V_CC` is the supply pin's voltage against `GND`**, and the outputs
//!   drive it above 0 V in the engine's frame — exact while `GND` sits at
//!   0 V there, as the isolators and the logic gates assume.
//! - **Not modelled**: propagation delay (`t_PLH`/`t_PHL` 20 ns max, §5.6)
//!   and output skew, the enable and disable times, the output current
//!   limit beyond the two ports, the `V_OH`/`V_OL` curves (Figures 5-5 to
//!   5-8), supply current, ESD and thermals.

use std::sync::{Arc, Mutex};

use embsim_board::{
    AttachError, Component, ComponentNetIo, DeadBand, DigitalReceiver, Drive, Level, Ohms,
    PeriodicSchedule, PinDecl, PinHandle, Sense, TheveninDrive, Thresholds, Volts,
};

use crate::isolation::supply_volts;

// ============================================================
// Datasheet constants
// ============================================================

/// `A` and the two enables read against `GND` at the TTL levels, absolute:
/// `V_IL` max 0.8 V, `V_IH` min 2 V (SLLS114N §5.3). No hysteresis is
/// given, so between the two neither level is guaranteed
/// ([`DeadBand::Unknown`]).
pub const AM26LS31_INPUT_THRESHOLDS: Thresholds = Thresholds::new(0.8, 2.0, 0.0, DeadBand::Unknown);

/// `V_CC` minimum for the C grade: 4.75 V (SLLS114N §5.3; §5.5 note (1),
/// the `V_CC` = MIN the output lines are tested at).
pub const AM26LS31_VCC_MIN_VOLTS: Volts = 4.75;

/// `V_CC` maximum for the C grade: 5.25 V (SLLS114N §5.3; §5.5 note (1),
/// the `V_CC` = MAX the short-circuit line is tested at).
pub const AM26LS31_VCC_MAX_VOLTS: Volts = 5.25;

/// `V_OH` min: 2.5 V at `V_CC` = MIN, `I_OH` = −20 mA (SLLS114N §5.5).
pub const AM26LS31_VOH_MIN_VOLTS: Volts = 2.5;

/// `V_OL` max: 0.5 V at `V_CC` = MIN, `I_OL` = 20 mA (SLLS114N §5.5).
pub const AM26LS31_VOL_MAX_VOLTS: Volts = 0.5;

/// The current both output lines are tested at, amperes: `I_OH` = −20 mA
/// and `I_OL` = 20 mA (SLLS114N §5.5; §5.3 gives the same as the
/// recommended output currents).
pub const AM26LS31_OUTPUT_TEST_AMPS: f64 = 0.020;

/// `I_OS`, the short-circuit output current at `V_CC` = MAX, amperes: 30 mA
/// min, 150 mA max in magnitude (SLLS114N §5.5).
pub const AM26LS31_IOS_AMPS: (f64, f64) = (0.030, 0.150);

/// High-level output impedance: `(V_CC MIN 4.75 V − V_OH min 2.5 V) /
/// 20 mA` = 112.5 Ω (SLLS114N §5.5, the `V_OH` line at its test condition).
pub const AM26LS31_R_OH_OHMS: Ohms =
    (AM26LS31_VCC_MIN_VOLTS - AM26LS31_VOH_MIN_VOLTS) / AM26LS31_OUTPUT_TEST_AMPS;

/// Low-level output impedance: `V_OL max 0.5 V / 20 mA` = 25 Ω (SLLS114N
/// §5.5, the `V_OL` line at its test condition).
pub const AM26LS31_R_OL_OHMS: Ohms = AM26LS31_VOL_MAX_VOLTS / AM26LS31_OUTPUT_TEST_AMPS;

/// `V_POR` max with `V_CC` rising: 3.04 V (SLLS114N §5.5, PDIP, SO, SOIC and
/// SSOP packages) — the supply at which every part is out of reset and
/// driving. Below it the outputs are released.
pub const AM26LS31_VPOR_MAX_VOLTS: Volts = 3.04;

// ============================================================
// Pin table
// ============================================================

/// An input — `A` or an enable — reading the TTL levels against `GND`.
const fn input(number: &'static str, name: &'static str) -> PinDecl {
    PinDecl::digital_in(number, AM26LS31_INPUT_THRESHOLDS)
        .with_reference("8")
        .with_name(name)
}

/// A three-state output, released from power-on: an unpowered part is high
/// impedance (SLLS114N §7.3.2), and the declaration says so.
const fn output(number: &'static str, name: &'static str) -> PinDecl {
    PinDecl::digital_out(number).with_idle(None).with_name(name)
}

/// The AM26LS31's 16-pin table, by pin number (SLLS114N §4, Figure 4-1 and
/// Table 4-1: the D, DB, N, NS, J and W packages). The supply is measured
/// against `GND`.
pub const AM26LS31_PINS: [PinDecl; 16] = [
    input("1", "1A"),
    output("2", "1Y"),
    output("3", "1Z"),
    input("4", "G"),
    output("5", "2Z"),
    output("6", "2Y"),
    input("7", "2A"),
    PinDecl::power_in("8").with_name("GND"),
    input("9", "3A"),
    output("10", "3Y"),
    output("11", "3Z"),
    input("12", "~G"),
    output("13", "4Z"),
    output("14", "4Y"),
    input("15", "4A"),
    PinDecl::power_in("16").with_reference("8").with_name("VCC"),
];

/// `(A, Y, Z)` for each of the four drivers (Table 4-1).
pub const AM26LS31_CHANNELS: [(&str, &str, &str); 4] = [
    ("1", "2", "3"),
    ("7", "6", "5"),
    ("9", "10", "11"),
    ("15", "14", "13"),
];

/// The active-high enable `G` and the active-low enable `G̅` (Table 4-1).
const ENABLE_HIGH: &str = "4";
const ENABLE_LOW: &str = "12";
const VCC: &str = "16";

// ============================================================
// Core
// ============================================================

/// What one channel's input asks its pair to present: a level, or a
/// relayed clock.
#[derive(Debug, Clone, Copy, PartialEq)]
enum ChannelIn {
    /// `Y` at `level`, `Z` at its complement.
    Level(Level),
    /// `Y` swings `hi`/`lo` around `segment`; `Z` is the complement of each
    /// phase.
    Periodic {
        hi: Level,
        lo: Level,
        segment: PeriodicSchedule,
    },
}

/// What a pair presents: the channel's input at the supply it is driven
/// from, or released (`None`).
type PairOut = Option<(ChannelIn, Volts)>;

#[derive(Debug, Default)]
struct State {
    vcc: Option<Sense>,
    enable_high: Option<Level>,
    enable_low: Option<Level>,
    inputs: [Option<ChannelIn>; 4],
    outputs: Vec<(PinHandle, PinHandle)>,
    /// What each pair last published (`None` in the outer option before
    /// the first publish). An unchanged pair is not re-issued: it would
    /// resolve nothing and cost an engine event per sense.
    applied: [Option<PairOut>; 4],
}

#[derive(Debug, Default)]
struct Core {
    state: Mutex<State>,
}

/// The other level.
fn invert(level: Level) -> Level {
    match level {
        Level::High => Level::Low,
        Level::Low => Level::High,
    }
}

/// An output port: high is `V_CC` behind [`AM26LS31_R_OH_OHMS`], low is
/// 0 V behind [`AM26LS31_R_OL_OHMS`] (SLLS114N §5.5).
fn port(level: Level, vcc: Volts) -> TheveninDrive {
    match level {
        Level::High => TheveninDrive {
            volts: vcc,
            impedance: AM26LS31_R_OH_OHMS,
        },
        Level::Low => TheveninDrive {
            volts: 0.0,
            impedance: AM26LS31_R_OL_OHMS,
        },
    }
}

impl Core {
    /// `V_CC` against `GND` when the part is out of reset: at or above
    /// [`AM26LS31_VPOR_MAX_VOLTS`].
    fn rail(state: &State) -> Option<Volts> {
        state
            .vcc
            .as_ref()
            .and_then(|vcc| supply_volts(vcc, AM26LS31_VPOR_MAX_VOLTS))
    }

    /// Table 7-1: the outputs are active when `G` is high or `G̅` is low.
    fn enabled(state: &State) -> bool {
        state.enable_high == Some(Level::High) || state.enable_low == Some(Level::Low)
    }

    /// Re-drive every pair whose output changed: from its input when the
    /// part is powered and enabled, released otherwise.
    fn apply(state: &mut State) {
        let rail = Self::rail(state).filter(|_| Self::enabled(state));
        for channel in 0..state.outputs.len() {
            let desired: PairOut = rail.and_then(|vcc| state.inputs[channel].map(|i| (i, vcc)));
            if state.applied[channel] == Some(desired) {
                continue;
            }
            state.applied[channel] = Some(desired);
            let (y, z) = &state.outputs[channel];
            match desired {
                Some((ChannelIn::Level(level), vcc)) => {
                    y.drive(Drive::Thevenin(port(level, vcc)));
                    z.drive(Drive::Thevenin(port(invert(level), vcc)));
                }
                Some((ChannelIn::Periodic { hi, lo, segment }, vcc)) => {
                    y.drive(Drive::Periodic {
                        hi: port(hi, vcc),
                        lo: port(lo, vcc),
                        segment,
                    });
                    z.drive(Drive::Periodic {
                        hi: port(invert(hi), vcc),
                        lo: port(invert(lo), vcc),
                        segment,
                    });
                }
                None => {
                    y.release();
                    z.release();
                }
            }
        }
    }
}

/// What a channel's `A` asks its pair to present: a relayed crossing clock
/// (running, or a held segment whose phases still cross), else a single
/// level, else nothing.
fn channel_in(receiver: &DigitalReceiver, sensed: &Sense) -> Option<ChannelIn> {
    if let Some(clock) = sensed.periodic {
        if let (Some(hi), Some(lo)) = clock.levels(&AM26LS31_INPUT_THRESHOLDS, None) {
            let crosses = clock.rate(&AM26LS31_INPUT_THRESHOLDS).is_some()
                || (clock.segment.freq_hz == 0 && hi != lo);
            if crosses {
                // Keep the receiver's last level in step with the delivery,
                // so a later single-level read starts from a known last.
                let _ = receiver.read(sensed);
                return Some(ChannelIn::Periodic {
                    hi,
                    lo,
                    segment: clock.segment,
                });
            }
        }
    }
    receiver.read(sensed).map(ChannelIn::Level)
}

// ============================================================
// Component
// ============================================================

/// The TI AM26LS31 as a live board-engine component — see the module docs.
///
/// ```rust
/// use embsim_board::Component;
/// use embsim_models::am26ls31::Am26ls31;
///
/// let driver = Am26ls31::new();
/// assert_eq!(driver.pins().len(), 16);
/// ```
#[derive(Debug, Default)]
pub struct Am26ls31 {
    core: Arc<Core>,
}

impl Am26ls31 {
    /// A driver; its outputs drive from whatever its own `V_CC` pin is at.
    pub fn new() -> Self {
        Self::default()
    }
}

impl Component for Am26ls31 {
    fn pins(&self) -> &[PinDecl] {
        &AM26LS31_PINS
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        // Output handles first: a sense callback registered below fires at
        // once with the current state and must find them.
        {
            let mut state = self.core.state.lock().unwrap();
            for (_, y, z) in AM26LS31_CHANNELS {
                state.outputs.push((io.pin(y)?, io.pin(z)?));
            }
        }

        let core = Arc::clone(&self.core);
        io.on_sense(VCC, move |sensed| {
            let mut state = core.state.lock().unwrap();
            state.vcc = Some(sensed);
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
        for (channel, (a, _, _)) in AM26LS31_CHANNELS.into_iter().enumerate() {
            let core = Arc::clone(&self.core);
            let receiver = DigitalReceiver::new(io.pin(a)?);
            io.on_sense(a, move |sensed| {
                let mut state = core.state.lock().unwrap();
                state.inputs[channel] = channel_in(&receiver, &sensed);
                Core::apply(&mut state);
            })?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use rstest::rstest;

    use super::*;

    /// The two ports meet the datasheet's output lines exactly at their
    /// test condition, and the high port's short-circuit current at the
    /// top of the supply range is inside `I_OS`.
    #[rstest]
    fn the_output_ports_meet_the_datasheet_lines() {
        let loaded = |drive: TheveninDrive, load_volts: Volts| {
            let amps = (drive.volts - load_volts) / drive.impedance;
            (load_volts, amps)
        };
        // High at V_CC MIN, sourcing 20 mA, sits at V_OH min.
        let high = port(Level::High, AM26LS31_VCC_MIN_VOLTS);
        let (_, amps) = loaded(high, AM26LS31_VOH_MIN_VOLTS);
        assert!((amps - AM26LS31_OUTPUT_TEST_AMPS).abs() < 1e-12, "{amps}");
        // Low, sinking 20 mA, sits at V_OL max.
        let low = port(Level::Low, AM26LS31_VCC_MIN_VOLTS);
        let (_, amps) = loaded(low, AM26LS31_VOL_MAX_VOLTS);
        assert!((amps + AM26LS31_OUTPUT_TEST_AMPS).abs() < 1e-12, "{amps}");
        // Shorted at V_CC MAX.
        let short = AM26LS31_VCC_MAX_VOLTS / AM26LS31_R_OH_OHMS;
        assert!(
            (AM26LS31_IOS_AMPS.0..=AM26LS31_IOS_AMPS.1).contains(&short),
            "{short}"
        );
        assert!((AM26LS31_R_OH_OHMS - 112.5).abs() < 1e-9);
        assert!((AM26LS31_R_OL_OHMS - 25.0).abs() < 1e-9);
    }

    /// Every pin of the 16-pin table, once; every channel's pins are an
    /// input and two outputs.
    #[rstest]
    fn the_pin_table_is_figure_4_1() {
        let numbers: Vec<&str> = AM26LS31_PINS.iter().map(|pin| pin.number).collect();
        let expected: Vec<String> = (1..=16).map(|n| n.to_string()).collect();
        assert_eq!(numbers, expected);
        let pin = |number: &str| {
            AM26LS31_PINS
                .iter()
                .find(|pin| pin.number == number)
                .expect("a pin of the table")
        };
        for (a, y, z) in AM26LS31_CHANNELS {
            assert!(pin(a).thresholds.is_some() && !pin(a).can_source, "{a}");
            for out in [y, z] {
                assert!(pin(out).can_source && pin(out).can_sink, "{out}");
                assert_eq!(pin(out).idle, None, "{out} rests released");
            }
        }
        assert_eq!(pin(VCC).reference, Some("8"));
    }
}
