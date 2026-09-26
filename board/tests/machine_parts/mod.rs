// Shared across three integration binaries (`ec32mb_module`, `edgeboard`,
// `machine_system`), each of which uses a different subset — Cargo compiles a
// private copy of this module into every one of them, so anything the *other*
// binaries use is dead code here.
#![allow(dead_code)]

//! Part library and board/harness builders for the netlist-grounded machine
//! tests: the P2-EC32MB module, the MaD EdgeBoard, and the harnesses that
//! plug them into each other and into the machine.
//!
//! `BOARD_ENGINE.md` puts consumer-side specifics — "part registry entries,
//! harness files, plant models" — in the *consuming* repo. This module is
//! their transitional home: the reference consumer's parts, expressed against
//! the two committed netlist fixtures, so the engine has something real to be
//! tested against before MaD grows its own registry.
//!
//! # Netlist sources
//!
//! | Fixture | Board | Notes |
//! |---|---|---|
//! | `fixtures/p2_ec32mb.net` | Parallax P2-EC32MB Rev B module | transcribed from the vendor PDF; **no `libsource`** |
//! | `fixtures/mad_edge.net` | MaD EdgeBoard (3 sheets) | `kicad-cli sch export netlist` |
//! | `fixtures/ds2_addon.net` | MaD DS2 force-gauge add-on | `kicad-cli sch export netlist` |
//!
//! # Every part a node
//!
//! `DESIGN.md` rule 1 admits no stub tier, and since `NODES.md` §8 phase 4
//! none is needed here: every registered part on the three boards is a
//! model or an element by specification, and a part with nothing behind it
//! is a build error naming it. The engine takes electrical descriptors
//! from the component facade and never from the schematic (see the
//! `netlist` module docs), which is what lets a model *correct* a symbol:
//! the EdgeBoard's `XL1509` symbol draws all eight pins as `input`,
//! including VIN, OUT and the four grounds, and the rail model declares
//! what the part is.
//!
//! # Modeled parts (real behavior)
//!
//! Three parts carry behavior here, each with the datasheet header
//! `BOARD_ENGINE.md` ("Model provenance convention") requires:
//!
//! - [`Rs422Driver`] — TI AM26LS31, the servo step/direction pair;
//! - [`Rs422Receiver`] — TI AM26LV32, the encoder A/B/ZI pairs;
//! - [`SerialIsolator`] — TI ISO6731, the isolated force-gauge UART.
//!
//! The rest come from `embsim-models`: the other four ISO67xx isolators
//! (`IC1`, `IC2`, `IC14` with its STEP channel carrying a rate, `IC15`,
//! `IC16`), the 21 SN74LVC1G14 LED drivers and the five optocouplers
//! (`U4` a 6N137, `U5`–`U8` VO2631s) on the Edge board; the TCXO, the two
//! 74LVC2G04 inverters, the four PSRAMs and the boot flash `U301` (blank,
//! the 16 MiB part `embsim-boards` ships the module with; `w25q128jv.rs`
//! re-registers it with each case's own image) on the module. The
//! nonlinear parts are elements registered by specification from
//! `embsim_models::pwl_library`, keyed on the manufacturer part number the
//! netlists carry: on the Edge board the polarity FET `U3` with its body
//! diode, the transistor `Q1`, the eight current regulators `IC6`–`IC13`,
//! the Schottky diodes `D1`/`D2` and the 21 indicator LEDs; on the module
//! the polarity FET `U401` and the white LEDs `D601`/`D602`. The power
//! parts are rails (`embsim_models::rail`) and a detector
//! (`embsim_models::supervisor`): on the module the bucks `U402`/`U403`
//! (one registry key, each reading its own feedback divider at attach),
//! the LDOs `U501`–`U508` and the detector `U404`; on the Edge board the
//! bucks `U1`/`U2` (their version from the value) and the isolated DC/DCs
//! `IC3`/`IC4` (their setpoint from the `SEL` strap). Everything else is
//! an auto-classified primitive.

use std::sync::{Arc, Mutex, MutexGuard};

use embsim_board::mcu::SerialChannelConfig;
use embsim_board::registry::normalize_part;
use embsim_board::{
    AttachError, Board, Component, ComponentDecl, ComponentNetIo, DeadBand, DigitalReceiver,
    EndpointRef, Harness, InputPort, JumperState, Level, McuComponent, Ohms, PartRegistry, PinDecl,
    PinHandle, Scenario, SwitchPole, TheveninDrive, Thresholds, Volts,
};
use embsim_boards::ec32mb::{FLASH_CAPACITY, FLASH_PART};
use embsim_boards::p2::P2Package;
use embsim_models::isolation::{iso67xx, Iso67xx};
use embsim_models::logic_gate::{self, LogicGate, LVC1G14_PINS_SOT23, LVC2G04_PINS_BY_FUNCTION};
use embsim_models::opto::Opto;
use embsim_models::oscillator::{self, Oscillator};
use embsim_models::psram::{Psram, PsramComponent};
use embsim_models::pwl_library;
use embsim_models::rail::{
    self, Rail, AP62301_PINS_BY_FUNCTION, NCP114_PINS_BY_FUNCTION, UCC12040_PINS_SOIC16,
    XL1509_PINS_SOP8,
};
use embsim_models::spi_flash::SpiNorFlash;
use embsim_models::spi_flash_component::{SpiNorFlashComponent, SPI_FLASH_PINS_BY_FUNCTION};
use embsim_models::supervisor::{self, VoltageDetector, STM1061_PINS_BY_FUNCTION};

// ============================================================
// Pin-declaration helpers
// ============================================================

/// A pin the component senses and never drives, reading through
/// `thresholds` — the part's own datasheet figures.
pub const fn dig_in(number: &'static str, thresholds: Thresholds) -> PinDecl {
    PinDecl::digital_in(number, thresholds)
}

/// A push-pull output pin (idles `Driven(High)` until the component drives).
pub const fn dig_out(number: &'static str) -> PinDecl {
    PinDecl::digital_out(number)
}

/// A pin whose *voltage* the component needs — an analog reader, declaring
/// no thresholds, whose cluster is solved — a differential receiver input,
/// an ADC input.
pub const fn analog(number: &'static str) -> PinDecl {
    PinDecl::analog(number)
}

/// A rail the part consumes.
pub const fn pwr_in(number: &'static str) -> PinDecl {
    PinDecl::power_in(number)
}

/// A rail the part generates (regulator/DC-DC output, isolated-domain
/// reference). The net is a declared terminal — a cluster of its own and a
/// boundary of every cluster around it (`NODES.md` §8 phase 4) — held at
/// whatever the part publishes.
pub const fn pwr_out(number: &'static str) -> PinDecl {
    PinDecl::power_out(number)
}

/// A terminal that contributes nothing electrical.
pub const fn passive(number: &'static str) -> PinDecl {
    PinDecl::passive(number)
}

/// A pin the schematic marks no-connect. Spelled distinctly from [`passive`]
/// so the tables read as documentation: the facade must still declare the pin
/// (the netlist has a node for it, on an `unconnected-(…)` stub net), and
/// declaring it passive keeps a deliberately dangling pad out of the findings.
pub const fn nc(number: &'static str) -> PinDecl {
    passive(number)
}

// ============================================================
// Shared electrical helpers
// ============================================================

/// Push-pull source impedance the modeled outputs drive through.
const OUTPUT_IMPEDANCE_OHMS: Ohms = 25.0;

/// Drive for a level at a rail.
fn drive(level: Level, rail_volts: Volts) -> TheveninDrive {
    TheveninDrive {
        volts: match level {
            Level::High => rail_volts,
            Level::Low => 0.0,
        },
        impedance: OUTPUT_IMPEDANCE_OHMS,
    }
}

// ============================================================
// AM26LS31 — quad differential line driver (RS-422/RS-485)
// ============================================================

//
// Provenance
//   Part      : Texas Instruments AM26LS31C, "AM26LS31 Quadruple Differential
//               Line Driver" (TI literature number SLLS114N, revised June
//               2026; see the citation note at the end of this block).
//   Governs   : the datasheet's **function table** — input A and the two
//               enables G / ~G against outputs Y / Z — which is the whole of
//               the behavior modeled here.
//   Instance  : MaD EdgeBoard U24 (`AM26LS31CD`), sheet `MaD_Edge_Sheet3`,
//               driving the servo step/direction pairs `SC_PUL±` / `SC_DIR±`
//               out of connector J21.
//
// Behavior modeled
//   Per channel, when the outputs are enabled: Y follows the channel's A
//   input and Z is its complement — the differential pair. When disabled or
//   unpowered, both outputs are released to high-Z (the function table's
//   high-impedance row).
//   Enable is the datasheet's OR structure: outputs are active when G is high
//   OR ~G is low, and high-Z only when G is low AND ~G is high. On the
//   EdgeBoard both enables are strapped active (G to the isolated 5 V rail,
//   ~G to the isolated ground), so the driver is unconditionally on — which
//   this model reproduces rather than assumes.
//
// Deliberately NOT modeled
//   * Propagation delay and channel skew (tens of nanoseconds): far below one
//     bit of any step train the machine produces, and the engine has no
//     sub-microsecond scheduling granularity to express it.
//   * The output stage's drive current, short-circuit limit, and output
//     voltage protection: an unloaded Thevenin source at
//     OUTPUT_IMPEDANCE_OHMS stands in for it.
//   * V_OH tracking the supply. Outputs drive at the configured rail voltage;
//     a rail-referred V_OH is the regulator-model slice.
//   * Channels 3 and 4. The EdgeBoard schematic marks 3A/3Y/3Z/4A/4Y/4Z
//     no-connect, so they are declared `Passive` (see the module docs) — the
//     part has them, this board does not use them.
//
// Citation note
//   The behavior above is the function table, which is stable across every
//   revision of this part. The literature number and revision were pinned
//   when the input thresholds were declared (SLLS114N §5.3, see
//   `AM26LS31_INPUT_THRESHOLDS`); when this model is promoted out of the test
//   tree, add per-behavior "(§x.y, p.N)" citations the way `embsim-models`'
//   ADS122U04 model does against SBAS752B.
//

/// The AM26LS31's inputs — A and the two enables — read against GND at the
/// datasheet's TTL levels, absolute: `V_IL` max 0.8 V, `V_IH` min 2 V (TI
/// SLLS114N, §5.3 Recommended Operating Conditions); no hysteresis is named,
/// so between the two neither level is guaranteed ([`DeadBand::Unknown`]).
pub const AM26LS31_INPUT_THRESHOLDS: Thresholds = Thresholds::new(0.8, 2.0, 0.0, DeadBand::Unknown);

/// The AM26LS31's power-on-reset threshold, `V_POR` max with `V_CC` rising:
/// 3.04 V (TI SLLS114N, §5.5 Electrical Characteristics) — the supply at
/// which every part is out of reset and driving. Below it the outputs are
/// released.
pub const AM26LS31_VPOR_MAX_VOLTS: Volts = 3.04;

/// Pin facade of the `AM26LS31CD` (SOIC-16), pin numbers as the EdgeBoard
/// netlist names them.
#[rustfmt::skip]
pub const AM26LS31_PINS: [PinDecl; 16] = [
    dig_in("1", AM26LS31_INPUT_THRESHOLDS),   // 1A  — channel-1 input
    dig_out("2"),  // 1Y  — channel-1 true output
    dig_out("3"),  // 1Z  — channel-1 complement
    dig_in("4", AM26LS31_INPUT_THRESHOLDS),   // G   — active-high enable
    dig_out("5"),  // 2Z  — channel-2 complement
    dig_out("6"),  // 2Y  — channel-2 true output
    dig_in("7", AM26LS31_INPUT_THRESHOLDS),   // 2A  — channel-2 input
    pwr_in("8"),   // GND
    nc("9"),       // 3A  — unused on this board
    nc("10"),      // 3Y
    nc("11"),      // 3Z
    dig_in("12", AM26LS31_INPUT_THRESHOLDS),  // ~G  — active-low enable
    nc("13"),      // 4Z
    nc("14"),      // 4Y
    nc("15"),      // 4A
    pwr_in("16"),  // VDD
];

/// `(input, true output, complement output)` for the two channels this board
/// wires.
const AM26LS31_CHANNELS: [(&str, &str, &str); 2] = [("1", "2", "3"), ("7", "6", "5")];

/// Mutable driver state. Every field is written only from engine-thread sense
/// callbacks, so the mutex is uncontended in practice and exists to keep the
/// component `Sync`.
#[derive(Default)]
struct DriverState {
    powered: bool,
    enable_high: Option<Level>,
    enable_low: Option<Level>,
    inputs: [Option<Level>; 2],
    outputs: Vec<(PinHandle, PinHandle)>,
    /// What each channel's pair last published: `Some(level)` driving `Y`
    /// at `level` (and `Z` at its complement), `None` released; `None` in
    /// the outer option before the first publish. A pair re-issued
    /// unchanged would resolve nothing and cost an engine event per sense.
    applied: [Option<Option<Level>>; 2],
}

struct DriverCore {
    rail_volts: Volts,
    state: Mutex<DriverState>,
}

impl DriverCore {
    /// True when the SLLS114N enable structure has the outputs active: G high
    /// OR ~G low.
    fn enabled(state: &DriverState) -> bool {
        state.enable_high == Some(Level::High) || state.enable_low == Some(Level::Low)
    }

    /// Re-drive every channel from the current inputs (or release when
    /// disabled/unpowered) — only a channel whose output changed, as the
    /// isolator and the gates publish.
    fn apply(&self, state: &mut DriverState) {
        let active = state.powered && Self::enabled(state);
        for (channel, (y, z)) in state.outputs.iter().enumerate() {
            // Disabled, unpowered, or an input with no defensible level:
            // high-Z, never a guessed differential.
            let desired = if active { state.inputs[channel] } else { None };
            if state.applied[channel] == Some(desired) {
                continue;
            }
            state.applied[channel] = Some(desired);
            match desired {
                Some(level) => {
                    y.set_drive(Some(drive(level, self.rail_volts)));
                    z.set_drive(Some(drive(invert(level), self.rail_volts)));
                }
                None => {
                    y.set_drive(None);
                    z.set_drive(None);
                }
            }
        }
    }
}

/// The other logic level.
fn invert(level: Level) -> Level {
    match level {
        Level::High => Level::Low,
        Level::Low => Level::High,
    }
}

/// TI AM26LS31 quad differential line driver — see the provenance block above.
pub struct Rs422Driver {
    core: Arc<DriverCore>,
}

impl Rs422Driver {
    /// A driver whose outputs swing between 0 V and `rail_volts`.
    pub fn new(rail_volts: Volts) -> Self {
        Self {
            core: Arc::new(DriverCore {
                rail_volts,
                state: Mutex::new(DriverState::default()),
            }),
        }
    }
}

impl std::fmt::Debug for Rs422Driver {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Rs422Driver")
            .field("rail_volts", &self.core.rail_volts)
            .finish()
    }
}

impl Component for Rs422Driver {
    fn pins(&self) -> &[PinDecl] {
        &AM26LS31_PINS
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        // Output handles first: a sense callback registered below fires
        // immediately with the current state and must find them.
        {
            let mut state = self.core.state.lock().unwrap();
            for (_, y, z) in AM26LS31_CHANNELS {
                state.outputs.push((io.pin(y)?, io.pin(z)?));
            }
        }

        let core = Arc::clone(&self.core);
        io.on_sense("16", move |rail| {
            let mut state = core.state.lock().unwrap();
            state.powered = rail.volts.is_some_and(|v| v >= AM26LS31_VPOR_MAX_VOLTS);
            core.apply(&mut state);
        })?;
        for (pin, slot) in [("4", true), ("12", false)] {
            let core = Arc::clone(&self.core);
            let receiver = DigitalReceiver::new(io.pin(pin)?);
            io.on_sense(pin, move |sensed| {
                let mut state = core.state.lock().unwrap();
                let level = receiver.read(&sensed);
                if slot {
                    state.enable_high = level;
                } else {
                    state.enable_low = level;
                }
                core.apply(&mut state);
            })?;
        }
        for (channel, (a, _, _)) in AM26LS31_CHANNELS.into_iter().enumerate() {
            let core = Arc::clone(&self.core);
            let receiver = DigitalReceiver::new(io.pin(a)?);
            io.on_sense(a, move |sensed| {
                let mut state = core.state.lock().unwrap();
                state.inputs[channel] = receiver.read(&sensed);
                core.apply(&mut state);
            })?;
        }
        Ok(())
    }
}

// ============================================================
// AM26LV32 — quad differential line receiver (RS-422/RS-423)
// ============================================================

//
// Provenance
//   Part      : Texas Instruments AM26LV32, "Low-Voltage Quadruple
//               Differential Line Receiver" (TI literature number SLLS202H,
//               revised August 2023: the ±200 mV thresholds and the input
//               resistance are §6.5 Electrical Characteristics, the
//               fail-safe §8.4.1, each input's open-circuit voltage Figure
//               9-2 of §9.2.3).
//   Governs   : the differential input thresholds (V_IT± = ±200 mV), the
//               enable structure (G / ~G, same OR form as the AM26LS31), and
//               the input failsafe that forces Y high for open, shorted, or
//               idle-terminated inputs.
//   Instance  : MaD EdgeBoard U25 (`AM26LV32xD`), sheet `MaD_Edge_Sheet3`,
//               receiving the encoder's `A±` / `B±` / `ZI±` pairs from
//               connector J20.
//
// Behavior modeled
//   Per channel, when enabled and powered: V_ID = V(A) − V(B) is taken from
//   the *solved node voltages* (the input pins are analog readers — senses
//   with no thresholds), and Y is driven high for V_ID >= +200 mV, low for
//   V_ID <= −200 mV. Inside the ±200 mV band — the shorted-pair and
//   idle-terminated cases — and whenever either leg has no defensible voltage,
//   the datasheet's failsafe forces Y **high**. Disabled or unpowered releases
//   Y to high-Z.
//   Each input declares its own port (`InputPort`, `NODES.md` §10), the open
//   fail-safe's mechanism (§8.4.1): r_I 12 kΩ typical (§6.5) to the input's
//   open-circuit voltage, 0.83 V on an A input and 0.70 V on a B input —
//   read off Figure 9-2, "RS422 Port Open-Circuit Voltage vs V_CC", flat
//   from V_CC 1.6 V to 3.6 V. An open pair therefore reads +130 mV, inside
//   the band: high, by the failsafe, through the part's own bias and no
//   other source.
//   Channel 4 is exactly that failsafe case on this board: 4A/4B are marked
//   no-connect while 4Y is wired to the encoder isolator, so 4Y sits high.
//
// Board note worth stating, because it looks like a mistake and is not
//   The EdgeBoard wires the encoder's index pair to the receiver's *enable*
//   pins — `Z+` to G (pin 4) and `Z−` to ~G (pin 12) — rather than to a
//   receiver channel. Closing the board's `Z_GND` jumper (JP4) ties `Z−` to
//   the isolated ground, which asserts ~G low and enables all four channels
//   unconditionally. That jumper is therefore load-bearing for the encoder
//   path, and the machine system description closes it.
//
// Deliberately NOT modeled
//   * Propagation delay (tens of nanoseconds) and hysteresis around V_IT, for
//     the reasons given on the driver.
//   * Input common-mode range, and the ports' fall below V_CC 1.6 V
//     (Figure 9-2: 0.42 V / 0.35 V at 0.8 V): a port is a declaration,
//     stamped once and never republished, so it holds its flat-band figure
//     whatever the supply. The bias is stamped in the engine's frame, exact
//     while GND sits at 0 V. r_I's minimum, 7 kΩ, is not modeled.
//   * The supply-range check beyond its minimum — an over-range VDD is a
//     rail finding, not a receiver behavior; under `V_CC` min 3 V (SLLS202H
//     §6.3) the outputs are released.
//

/// The AM26LV32's enables, G and ~G, read against GND, absolute: `V_IL(EN)`
/// max 0.8 V, `V_IH(EN)` min 2 V (TI SLLS202H, §6.3 Recommended Operating
/// Conditions); no hysteresis is named, so between the two neither level is
/// guaranteed ([`DeadBand::Unknown`]).
pub const AM26LV32_ENABLE_THRESHOLDS: Thresholds =
    Thresholds::new(0.8, 2.0, 0.0, DeadBand::Unknown);

/// The AM26LV32's input resistance, `r_I` 12 kΩ typical (TI SLLS202H, §6.5
/// Electrical Characteristics; 7 kΩ minimum): each input's own port.
pub const AM26LV32_INPUT_OHMS: Ohms = 12_000.0;

/// The open-circuit voltage of an A (non-inverting) input, 0.83 V: read off
/// Figure 9-2, "RS422 Port Open-Circuit Voltage vs V_CC" (TI SLLS202H,
/// §9.2.3 Application Curve), curve A, flat from `V_CC` 1.6 V to 3.6 V.
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

/// Pin facade of the `AM26LV32xD` (SOIC-16), pin numbers as the EdgeBoard
/// netlist names them.
#[rustfmt::skip]
pub const AM26LV32_PINS: [PinDecl; 16] = [
    analog("1").with_input(AM26LV32_B_PORT),   // 1B
    analog("2").with_input(AM26LV32_A_PORT),   // 1A
    dig_out("3"),  // 1Y
    dig_in("4", AM26LV32_ENABLE_THRESHOLDS),   // G   — active-high enable (wired to the encoder's Z+)
    dig_out("5"),  // 2Y
    analog("6").with_input(AM26LV32_A_PORT),   // 2A
    analog("7").with_input(AM26LV32_B_PORT),   // 2B
    pwr_in("8"),   // GND
    analog("9").with_input(AM26LV32_B_PORT),   // 3B
    analog("10").with_input(AM26LV32_A_PORT),  // 3A
    dig_out("11"), // 3Y
    dig_in("12", AM26LV32_ENABLE_THRESHOLDS),  // ~G  — active-low enable (wired to the encoder's Z−)
    dig_out("13"), // 4Y
    nc("14"),      // 4A  — no-connect: channel 4 rides the input failsafe
    nc("15"),      // 4B
    pwr_in("16"),  // VDD
];

/// `(A, B, Y)` per channel; `None` inputs are the no-connect channel.
const AM26LV32_CHANNELS: [(Option<&str>, Option<&str>, &str); 4] = [
    (Some("2"), Some("1"), "3"),
    (Some("6"), Some("7"), "5"),
    (Some("10"), Some("9"), "11"),
    (None, None, "13"),
];

/// The AM26LV32's supply minimum, `V_CC` min 3 V (TI SLLS202H, §6.3
/// Recommended Operating Conditions): below it the outputs are released.
pub const AM26LV32_VCC_MIN_VOLTS: Volts = 3.0;

/// SLLS202H differential input threshold magnitude: V_IT+ <= +200 mV,
/// V_IT- >= -200 mV.
const VID_THRESHOLD_VOLTS: Volts = 0.200;

#[derive(Default)]
struct ReceiverState {
    powered: bool,
    enable_high: Option<Level>,
    enable_low: Option<Level>,
    /// `(A volts, B volts)` per channel.
    inputs: [(Option<Volts>, Option<Volts>); 4],
    outputs: Vec<PinHandle>,
}

struct ReceiverCore {
    rail_volts: Volts,
    state: Mutex<ReceiverState>,
}

impl ReceiverCore {
    /// SLLS202H enable structure — identical OR form to the driver's.
    fn enabled(state: &ReceiverState) -> bool {
        state.enable_high == Some(Level::High) || state.enable_low == Some(Level::Low)
    }

    /// The level a channel's differential pair resolves to, with the
    /// datasheet's failsafe covering open, shorted, and idle pairs.
    fn channel_level(inputs: (Option<Volts>, Option<Volts>)) -> Level {
        match inputs {
            (Some(a), Some(b)) => {
                let vid = a - b;
                if vid >= VID_THRESHOLD_VOLTS {
                    Level::High
                } else if vid <= -VID_THRESHOLD_VOLTS {
                    Level::Low
                } else {
                    Level::High // input failsafe (|V_ID| < 200 mV)
                }
            }
            // Open input: the same failsafe.
            _ => Level::High,
        }
    }

    fn apply(&self, state: &mut ReceiverState) {
        let active = state.powered && Self::enabled(state);
        for (channel, y) in state.outputs.iter().enumerate() {
            if active {
                y.set_drive(Some(drive(
                    Self::channel_level(state.inputs[channel]),
                    self.rail_volts,
                )));
            } else {
                y.set_drive(None);
            }
        }
    }
}

/// TI AM26LV32 quad differential line receiver — see the provenance block
/// above.
pub struct Rs422Receiver {
    core: Arc<ReceiverCore>,
}

impl Rs422Receiver {
    /// A receiver whose outputs swing between 0 V and `rail_volts`.
    pub fn new(rail_volts: Volts) -> Self {
        Self {
            core: Arc::new(ReceiverCore {
                rail_volts,
                state: Mutex::new(ReceiverState::default()),
            }),
        }
    }
}

impl std::fmt::Debug for Rs422Receiver {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Rs422Receiver")
            .field("rail_volts", &self.core.rail_volts)
            .finish()
    }
}

impl Component for Rs422Receiver {
    fn pins(&self) -> &[PinDecl] {
        &AM26LV32_PINS
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        {
            let mut state = self.core.state.lock().unwrap();
            for (_, _, y) in AM26LV32_CHANNELS {
                state.outputs.push(io.pin(y)?);
            }
        }

        let core = Arc::clone(&self.core);
        io.on_sense("16", move |rail| {
            let mut state = core.state.lock().unwrap();
            state.powered = rail.volts.is_some_and(|v| v >= AM26LV32_VCC_MIN_VOLTS);
            core.apply(&mut state);
        })?;
        for (pin, is_active_high) in [("4", true), ("12", false)] {
            let core = Arc::clone(&self.core);
            let receiver = DigitalReceiver::new(io.pin(pin)?);
            io.on_sense(pin, move |sensed| {
                let mut state = core.state.lock().unwrap();
                let level = receiver.read(&sensed);
                if is_active_high {
                    state.enable_high = level;
                } else {
                    state.enable_low = level;
                }
                core.apply(&mut state);
            })?;
        }
        for (channel, (a, b, _)) in AM26LV32_CHANNELS.into_iter().enumerate() {
            for (pin, is_a) in [(a, true), (b, false)] {
                let Some(pin) = pin else { continue };
                let core = Arc::clone(&self.core);
                io.on_sense(pin, move |sensed| {
                    let mut state = core.state.lock().unwrap();
                    let volts = sensed.volts;
                    if is_a {
                        state.inputs[channel].0 = volts;
                    } else {
                        state.inputs[channel].1 = volts;
                    }
                    core.apply(&mut state);
                })?;
            }
        }
        Ok(())
    }
}

// ============================================================
// ISO6731 — triple-channel digital isolator (the force-gauge UART)
// ============================================================

//
// Provenance
//   Part      : Texas Instruments ISO6731, one of the "ISO67xx High-Speed,
//               Robust-EMC Reinforced and Basic Digital Isolators" family;
//               the ISO6731 variant is triple-channel, 2 forward + 1 reverse.
//               **Document number not recorded** — unlike the two TI interface
//               parts above, no literature number is asserted here rather than
//               guessing one. Pin the datasheet (number, revision, and the
//               function/channel-map section) before this model leaves the
//               test tree; the channel map below is instead cited to the
//               *board*, whose netlist independently shows which pins are
//               inputs and which outputs.
//   Governs   : the channel directions (INA/INB on side 1 driving OUTA/OUTB
//               on side 2; INC on side 2 driving OUTC on side 1 — confirmed
//               against the EdgeBoard netlist's own wiring), the
//               transparent-repeater behavior, and the default output state
//               when a side loses power.
//   Instance  : MaD EdgeBoard IC5 (`ISO6731DWR`), sheet `MaD_Edge_Sheet2`.
//               Side 1 sits in the isolated force-gauge domain
//               (`IFG_5V` / `IFG_GND`, brought out on connector J9); side 2
//               is the P2's 3.3 V domain. The three channels are the
//               force-gauge UART and its data-ready line:
//                 P2 → INC → OUTC → IFG_TX  (MCU transmit, into the gauge)
//                 IFG_RX → INA → OUTA → P0  (gauge transmit, into the MCU)
//                 IFG_INT → INB → OUTB → P1 (the ADC's ~DRDY)
//
// Behavior modeled
//   A transparent repeater: three identical level channels, each sensing its
//   input net and driving its output net. That is what the part is — it has no
//   idea a UART is on two of its channels, and it used to be told, because the
//   force-gauge channels repeated *bytes* over stream pins so the isolator
//   could be a hop on the engine's derived byte route. Now that the UART is on
//   the net as levels, the special case is gone and all three channels are the
//   same three lines of code.
//   Repeating requires both sides powered. With either rail down the channel
//   is dead and its output is released.
//
// Deliberately NOT modeled
//   * Propagation delay (nanoseconds for this family) and pulse-width
//     distortion, against a 115.2 kbaud bit time of 8.7 µs.
//   * The datasheet's *default output* behavior on the failed side (outputs
//     go high when the input side is unpowered). Here an unpowered side
//     simply stops repeating and releases, which keeps the engine's
//     `PowerNetUnsourced` / `FloatingSense` reports as the account of the
//     failure instead of a plausible-looking idle-high line.
//   * The EN1/EN2 enable pins: this board marks both no-connect, so the
//     facade declares them passive and the model has no enable input.
//   * Common-mode transient immunity, isolation rating, and every other
//     safety characteristic — an isolator's *isolation* is exactly what a
//     netlist-structural engine gets for free by never connecting the nets.
//

/// The ISO67xx family's input thresholds, **relative** to the input side's
/// supply: `V_IL` 0.3 × VCCI, `V_IH` 0.7 × VCCI (SLLSFJ6G §7.3, the model
/// crate's [`iso67xx::DEFAULT_VIL_RATIO`]/[`iso67xx::DEFAULT_VIH_RATIO`]).
const ISO6731_INPUT_THRESHOLDS: Thresholds = Thresholds::new(
    embsim_models::isolation::iso67xx::DEFAULT_VIL_RATIO,
    embsim_models::isolation::iso67xx::DEFAULT_VIH_RATIO,
    0.0,
    DeadBand::Unknown,
);

/// An input on side 1 (`VCC1` against `GND1_1`) or side 2 (`VCC2` against
/// `GND2_1`).
const fn iso_in(number: &'static str, vcc: &'static str, gnd: &'static str) -> PinDecl {
    dig_in(number, ISO6731_INPUT_THRESHOLDS)
        .with_supply(vcc)
        .with_reference(gnd)
}

/// Pin facade of the `ISO6731DWR` (SOIC-16 wide), pin numbers as the
/// EdgeBoard netlist names them.
#[rustfmt::skip]
pub fn iso6731_pins() -> Vec<PinDecl> {
    vec![
        pwr_in("1"),    // VCC1   — isolated side
        pwr_in("2"),    // GND1_1
        iso_in("3", "1", "2"),    // INA    — gauge transmit in
        iso_in("4", "1", "2"),    // INB    — gauge ~DRDY in
        dig_out("5"),   // OUTC   — MCU transmit out (isolated side)
        nc("6"),        // NC_1
        nc("7"),        // EN1
        pwr_in("8"),    // GND1_2
        pwr_in("9"),    // GND2_1
        nc("10"),       // EN2
        nc("11"),       // NC_2
        iso_in("12", "16", "9"),  // INC    — MCU transmit in
        dig_out("13"),  // OUTB   — ~DRDY out
        dig_out("14"),  // OUTA   — gauge transmit out
        pwr_in("15"),   // GND2_2
        pwr_in("16"),   // VCC2   — MCU side
    ]
}

/// The three repeated channels, `(input pin, output pin)`, as the EdgeBoard
/// wires them: MCU transmit, gauge transmit, and the ADC's `~DRDY`.
const ISO6731_CHANNELS: [(&str, &str); 3] = [("12", "5"), ("3", "14"), ("4", "13")];

/// The side each channel's output is on: `OUTC` on side 1, `OUTA`/`OUTB` on
/// side 2 (index 0 is side 1).
const ISO6731_OUTPUT_SIDE: [usize; 3] = [0, 1, 1];

struct IsolatorCore {
    state: Mutex<IsolatorState>,
}

#[derive(Default)]
struct IsolatorState {
    /// Each side's supply when it is up: `VCC1`, `VCC2`, at or above the
    /// family's powered-up threshold.
    rails: [Option<Volts>; 2],
    /// Per channel: the level last sensed on its input, and its output pin.
    level_in: [Option<Level>; 3],
    level_out: [Option<PinHandle>; 3],
}

impl IsolatorCore {
    fn live(state: &IsolatorState) -> bool {
        state.rails.iter().all(Option::is_some)
    }

    /// Re-drive one channel's output from its input, at its own side's
    /// supply — the family's level translation.
    fn apply(&self, state: &mut IsolatorState, channel: usize) {
        let Some(out) = state.level_out[channel].clone() else {
            return;
        };
        let rail = state.rails[ISO6731_OUTPUT_SIDE[channel]];
        match (Self::live(state), state.level_in[channel], rail) {
            (true, Some(level), Some(rail)) => out.set_drive(Some(drive(level, rail))),
            _ => out.set_drive(None),
        }
    }

    /// Re-drive every channel — for a supply change, which affects all three.
    fn apply_all(&self, state: &mut IsolatorState) {
        for channel in 0..ISO6731_CHANNELS.len() {
            self.apply(state, channel);
        }
    }
}

/// TI ISO6731 triple-channel digital isolator — see the provenance block
/// above.
pub struct SerialIsolator {
    pins: Vec<PinDecl>,
    core: Arc<IsolatorCore>,
}

impl SerialIsolator {
    /// An isolator whose repeated outputs drive their own side's supply.
    pub fn new() -> Self {
        Self {
            pins: iso6731_pins(),
            core: Arc::new(IsolatorCore {
                state: Mutex::new(IsolatorState::default()),
            }),
        }
    }
}

impl Default for SerialIsolator {
    fn default() -> Self {
        Self::new()
    }
}

impl std::fmt::Debug for SerialIsolator {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SerialIsolator").finish_non_exhaustive()
    }
}

impl Component for SerialIsolator {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        {
            let mut state = self.core.state.lock().unwrap();
            for (channel, (_, output)) in ISO6731_CHANNELS.iter().enumerate() {
                state.level_out[channel] = Some(io.pin(output)?);
            }
        }

        // Rail senses first: a level delivered before the rails are known must
        // not slip through the power gate.
        for (pin, side) in [("1", 0usize), ("16", 1usize)] {
            let core = Arc::clone(&self.core);
            io.on_sense(pin, move |rail| {
                let mut state = core.state.lock().unwrap();
                state.rails[side] = rail
                    .volts
                    .filter(|&v| v >= iso67xx::DEFAULT_SUPPLY_MIN_VOLTS);
                core.apply_all(&mut state);
            })?;
        }

        // Each channel subscribes only to its own input, so one transition
        // costs one drive rather than one per channel.
        for (channel, (input, _)) in ISO6731_CHANNELS.iter().enumerate() {
            let core = Arc::clone(&self.core);
            let receiver = DigitalReceiver::new(io.pin(input)?);
            io.on_sense(input, move |sensed| {
                let mut state = core.state.lock().unwrap();
                state.level_in[channel] = receiver.read(&sensed);
                core.apply(&mut state, channel);
            })?;
        }
        Ok(())
    }
}

// ============================================================
// P2-EC32MB module: the P2 behind its full package facade
// ============================================================

/// The reference consumer's force-gauge serial channel, as its HAL config
/// table declares it: RX on P0, TX on P2, 115.2 kbaud. Shared with
/// `board/tests/mcu_component.rs` and MaD's own HAL-table test.
pub const FORCE_GAUGE_CHANNEL: SerialChannelConfig = SerialChannelConfig {
    rx_pin: 0,
    tx_pin: 2,
    baud: 115_200,
};

/// The 64 `"P{n}"` pin names. `PinDecl` needs `&'static str`, so they are
/// spelled out (the same table [`McuComponent`] keeps privately).
#[rustfmt::skip]
/// The P2 as the EC32MB module's `U100`: the [`P2Package`] around the
/// native [`McuComponent`].
///
/// The two live at different altitudes and the package is the seam between
/// them. `McuComponent` declares exactly the pins its emulated peripherals
/// bridge — two, for the bridged force-gauge UART — because that is what it
/// knows. The *board* knows the package: 64 pads, a core supply, sixteen
/// bank supplies, `RESN`, `TEST`, and the crystal pair, all of which the
/// netlist has nodes for and all of which the build validates in both
/// directions. The package declares those (every pad a released
/// bidirectional pin, `XI` a rate sink, `XO` a released output) and hands
/// the MCU the pads' net I/O, so it finds `"P2"` and `"P0"` in the handle
/// table and bridges them exactly as it would on a board of its own.
pub type P2EdgeModule = P2Package<McuComponent>;

/// Build the module's P2 with the force-gauge channel bridged.
pub fn p2_edge_module(name: &str) -> P2EdgeModule {
    let channel = FORCE_GAUGE_CHANNEL;
    let mcu = McuComponent::builder(name)
        .serial_table(vec![channel])
        .bridge_serial(0)
        .build()
        .expect("the force-gauge channel is in the table and inside P63");
    P2Package::native(mcu)
}

/// A [`P2EdgeModule`] in facade mode bridges HAL serial channel 0 into the
/// **process-default** peripheral instance, so only one system carrying one
/// may exist at a time inside a test binary. Tests that build such a system
/// take this lock (poison-recovering, like the paced-stream suites).
pub fn lock_module_instance() -> MutexGuard<'static, ()> {
    static LOCK: Mutex<()> = Mutex::new(());
    LOCK.lock().unwrap_or_else(|poisoned| {
        LOCK.clear_poison();
        poisoned.into_inner()
    })
}

// ============================================================
// P2-EC32MB module: classification
// ============================================================
//
// Classification, part by part. The module's netlist carries no `libsource`,
// so the registry runs with `classify_unnamed_by_reference(true)` and keys the
// other parts on their `value` field:
//
//   auto (reference prefix)  85 passives (C×66, R×15, L×2, D×2 LEDs) and the
//                            two J-prefixed pad/socket symbols the fallback
//                            can name — J203 (the 80-finger card edge) and
//                            J301 (microSD socket) — as boundaries.
//   real model               U100, the P2 (see `P2EdgeModule`); U301, the
//                            boot flash (`embsim_models::spi_flash`, blank);
//                            X100, the TCXO (`embsim_models::oscillator`);
//                            U101 and U601, the dual inverters
//                            (`embsim_models::logic_gate`); U302–U305, the
//                            PSRAMs (`embsim_models::psram`); U402/U403,
//                            the bucks, and U501–U508, the LDOs
//                            (`embsim_models::rail`); U404, the brownout
//                            detector (`embsim_models::supervisor`).
//   switch (by value)        S301, the four-way DIP switch, four poles by
//                            `<position>_ON`/`<position>_OFF`; J101, the
//                            oscillator-option solder link, one pole. Every
//                            pole open by default; a scenario closes one.
//   mechanical (by value)    J701/J702 (mounting holes, tied to GND), PCB
//                            (the raw board, no nodes) and NC_Net (a layout
//                            node): pads and nothing electrical.
//   element (by MPN)         U401, the Si3417DV polarity FET: its channel
//                            and body diode as piecewise-linear branches;
//                            D601/D602, the IN-S63AS5UW white LEDs — from
//                            `embsim_models::pwl_library`, keyed on the
//                            `MPN` field the transcription carries.
//
// The power tree is real, and it has a clock: from the carrier's 5 V on
// `J203` the bucks rise 2.5 ms later (the AP62301's soft-start) and the
// LDOs step with them, so a build snapshot — the state before the first
// wake — has every module rail down and says so (`Finding::RailDown`).
// `board/tests/power_tree.rs` starts the module and steps past it.

/// `DIP Switch 4 way` — the module's option switch (S301): four poles, one
/// per printed position *n*, between the netlist's `<n>_ON` and `<n>_OFF`
/// pin ids (the pairing the pin-function labels state: `"FLASH (ON side)"` /
/// `"FLASH (OFF side)"` on position 2). All open by default, as shipped;
/// position *n* is pole `n - 1` to `Scenario::switch`. Closing pole 1
/// (FLASH) ties `P2_IO61` to the flash `~CS`; pole 2 is the P59 pull-up
/// (`R302`), pole 3 the P59 pull-down (`R303`).
pub fn dip_switch_poles() -> Vec<SwitchPole> {
    (1..=4)
        .map(|n| SwitchPole::open(format!("{n}_ON"), format!("{n}_OFF")))
        .collect()
}

/// `Solder Link Pads` — J101, the oscillator-option link between `P2_IO32`
/// and the TCXO's `NC/GND` option pad: one pole, open as shipped.
pub fn solder_link_poles() -> Vec<SwitchPole> {
    vec![SwitchPole::open("1", "2")]
}

/// Part registry for `fixtures/p2_ec32mb.net`.
pub fn ec32mb_registry() -> PartRegistry {
    let mut registry = PartRegistry::new();
    // The netlist was transcribed from the vendor PDF and has no libsource, so
    // the auto tier keys on reference-designator prefixes and the registry on
    // the `value` field.
    registry.classify_unnamed_by_reference(true);

    registry.register("P2X8C4M64P", |_decl| Box::new(p2_edge_module("p2")));
    registry.register_switch("DIP Switch 4 way", dip_switch_poles());
    registry.register_switch("Solder Link Pads", solder_link_poles());
    registry.register_mechanical("Mounting Hole Vss");
    registry.register_mechanical("PCB for P2 EC Module");
    registry.register_mechanical("Layout node");
    // The TCXO at the frequency its value names, the two dual inverters,
    // the four PSRAMs — the same models `embsim-boards` ships the module
    // with.
    registry.register("TG2520SMN 20.0000M-ECGNNM3", |decl: &ComponentDecl| {
        let config = oscillator::Config::from_value(&decl.value)
            .unwrap_or_else(|| panic!("{}: the TCXO value names no frequency", decl.reference));
        Box::new(Oscillator::new(config))
    });
    registry.register("74LVC2G04GW,125", |_decl| {
        Box::new(
            LogicGate::new(logic_gate::Config::lvc2g04(), &LVC2G04_PINS_BY_FUNCTION)
                .expect("the datasheet configuration is valid"),
        )
    });
    registry.register("PSRAM 64Mbit", |_decl| {
        Box::new(PsramComponent::new(Psram::new()))
    });
    // The boot flash, live and blank — the same part `embsim-boards`
    // registers, so the two registries agree about `U301`; a test that
    // wants an image re-registers the key with its own (`w25q128jv.rs`).
    registry.register(FLASH_PART, |_decl| {
        Box::new(
            SpiNorFlashComponent::new(SpiNorFlash::blank(FLASH_CAPACITY))
                .with_pins(&SPI_FLASH_PINS_BY_FUNCTION),
        )
    });
    // The elements by specification: `U401` (a Si3417DV by its `MPN`
    // field) and the white LEDs `D601`/`D602`, from the element library —
    // as `embsim-boards` registers them.
    pwl_library::register(&mut registry);
    // The power tree — the same models `embsim-boards` registers: two
    // bucks from one key, each reading its own feedback divider at attach;
    // eight LDOs at the voltage their value names; the detector.
    registry.register("DCDC 3A SOT563", |_decl| {
        Box::new(
            Rail::new(rail::Config::ap62301(), &AP62301_PINS_BY_FUNCTION)
                .expect("the AP62301 table carries every role"),
        )
    });
    registry.register("LDO 300mA, 3.3V", |decl: &ComponentDecl| {
        let config = rail::Config::ncp114_from_value(&decl.value)
            .unwrap_or_else(|| panic!("{}: the LDO value names no voltage", decl.reference));
        Box::new(
            Rail::new(config, &NCP114_PINS_BY_FUNCTION)
                .expect("the NCP114 table carries every role"),
        )
    });
    registry.register("Voltage Detector 1.6V", |_decl| {
        Box::new(VoltageDetector::new(
            supervisor::Config::stm1061n16(),
            &STM1061_PINS_BY_FUNCTION,
        ))
    });
    registry
}

/// Build the P2-EC32MB module as a [`Board`].
pub fn ec32mb_board() -> Board {
    let parsed = embsim_board::netlist::parse(include_str!("../fixtures/p2_ec32mb.net"))
        .expect("the EC32MB fixture parses");
    Board::from_netlist(parsed, &ec32mb_registry()).expect("the EC32MB module builds")
}

/// The P2-EC32MB as `embsim-boards` ships it — its own registry, a blank
/// boot flash, an empty card socket — with the processor slot filled by a
/// P2 package held in reset ([`P2Package::held_in_reset`]): every pad
/// released, the rails and `RESN` sensed, `XI` accepting the rate, and no
/// core. It touches no process-global peripheral bank, unlike
/// [`P2EdgeModule`], so a test that wants the module as shipped and nothing
/// running in it can use this without the module-instance lock.
pub fn shipped_ec32mb_board() -> Board {
    embsim_boards::ec32mb::Ec32mb::new()
        .with_p2(|_decl| Box::new(P2Package::held_in_reset()))
        .build()
        .expect("the module builds")
}

// ============================================================
// MaD EdgeBoard: classification
// ============================================================
//
// Classification, part by part (this netlist is a real KiCad export, so every
// tier keys on the libsource part name as usual):
//
//   auto        R_Small ×32, C_Small ×29, LED ×21, D_Schottky_Small ×2 (SS36),
//               L_Small ×2 — the 86 passives; Conn_01x0n / Screw_Terminal ×23
//               plus the declared P2_EDGE_MODULE_SOCKET — the 24 boundaries;
//               Jumper_2_Open ×4 + Jumper_3_Open ×1 — the stateful shorts;
//               SW_Push ×1 (SW1, the reset button) — a one-pole switch, open;
//               MountingHole_Pad ×4 — mechanical nodes.
//               (86 + 24 + 5 + 1 + 4 + 48 registered = the netlist's 168.)
//   real model  AM26LS31CD (U24) and AM26LV32xD (U25), the encoder/servo
//               RS-422 pair; ISO6731DWR (IC5), the force-gauge UART isolator;
//               ISO6742DWR (IC1, IC2), ISO6741DWR (IC14), ISO6721BDR (IC15)
//               and ISO6740FDWR (IC16), the `embsim_models::isolation`
//               family model configured from each part name; SN74LVC1G14DBV
//               ×21 (U9–U34), the Schmitt inverters driving the front-panel
//               LEDs (`embsim_models::logic_gate`); 6N137 (U4) and VO2631
//               ×4 (U5–U8), the optocouplers (`embsim_models::opto`);
//               XL1509 (U1 5 V, U2 3.3 V), the bucks behind the polarity
//               FET, and UCC12040DVER (IC3, IC4), the isolated DC/DCs
//               (`embsim_models::rail`).
//   element     the elements by specification (`embsim_models::pwl_library`).

/// `UCC12040DVER` — TI isolated 500 mW DC/DC module (IC3: the isolated I/O
/// domain `5V_IO`/`GND_IO`; IC4: the force-gauge domain `IFG_5V`/`IFG_GND`).
/// Both tie `SEL` to `VISO`: the 5.0 V setpoint (SNVSBO5B Table 5-1). The
/// rail model declares `VISO` **and** its `GNDS` return as terminals — an
/// isolated DC/DC generates a whole domain, its ground included — and the
/// isolated ground is held by whatever the board or the harness ties it
/// to: nothing on the Edge board ties either, so on the board alone both
/// isolated rails stay down with their reference named
/// (`Finding::RailDown`), and a harness that declares the domain's return
/// ([`force_domain_rails`] does for the force gauge's, over the cable) is
/// what lets the rail come up.
pub const UCC12040_PART: &str = "UCC12040DVER";

/// `XL1509` — 2 A step-down converter (U1: +5 V, U2: +3.3 V), the version
/// read from the value (`XL1509-5V`, `XL1509-3.3V`).
///
/// The rail model **corrects the schematic symbol**, which draws all eight
/// pins as `input` — including `VIN`, the four grounds, and `OUT`.
/// Electrical descriptors come from the component, never from the netlist
/// (see the `netlist` module docs), and `OUT` as a `PowerOut` terminal is
/// what makes the board's rails reachable through the output inductors.
pub const XL1509_PART: &str = "XL1509";

/// Baud the EdgeBoard's isolated force-gauge UART runs at — the ADS122U04's
/// fixed 115.2 kbaud, which is also [`FORCE_GAUGE_CHANNEL`]'s.
pub const FORCE_GAUGE_BAUD_HZ: u32 = FORCE_GAUGE_CHANNEL.baud;

/// Rail voltage of the isolated servo domain (`SC_5V`), from connector J21.
pub const SERVO_RAIL_VOLTS: Volts = 5.0;

/// Rail voltage of the P2's I/O domain.
pub const LOGIC_RAIL_VOLTS: Volts = 3.3;

/// Part registry for `fixtures/mad_edge.net`.
pub fn edge_registry() -> PartRegistry {
    let mut registry = edge_registry_without_socket();
    // The board's own edge-socket symbol is where the P2 module plugs in.
    registry.register_boundary("P2_EDGE_MODULE_SOCKET");
    registry
}

/// [`edge_registry`] without the `P2_EDGE_MODULE_SOCKET` boundary declaration,
/// so a test can show that the declaration is what classifies the board's
/// project-library socket symbol.
pub fn edge_registry_without_socket() -> PartRegistry {
    let mut registry = PartRegistry::new();

    // Modeled parts.
    registry.register("AM26LS31CD", |_decl| {
        Box::new(Rs422Driver::new(SERVO_RAIL_VOLTS))
    });
    registry.register("AM26LV32xD", |_decl| {
        Box::new(Rs422Receiver::new(SERVO_RAIL_VOLTS))
    });
    registry.register("ISO6731DWR", |_decl| Box::new(SerialIsolator::new()));

    // The other ISO67xx isolators, configured straight from their part
    // names — `ISO6740FDWR` picks up its fail-safe-low default without
    // anyone re-deriving it from the suffix. `IC14`'s STEP channel carries
    // the servo step clock as a clock, like any channel handed one.
    for part in ["ISO6742DWR", "ISO6741DWR", "ISO6740FDWR", "ISO6721BDR"] {
        registry.register(part, move |decl: &ComponentDecl| {
            let name = normalize_part(decl);
            let config = iso67xx::Config::from_part_name(&name)
                .unwrap_or_else(|| panic!("{name} is an ISO67xx"));
            Box::new(Iso67xx::new(config).expect("a valid isolator configuration"))
        });
    }
    // The 21 Schmitt inverters driving the front-panel LEDs.
    registry.register("SN74LVC1G14DBV", |_decl| {
        Box::new(
            LogicGate::new(logic_gate::Config::lvc1g14(), &LVC1G14_PINS_SOT23)
                .expect("the datasheet configuration is valid"),
        )
    });

    // The optocouplers: `U4` (a Lite-On 6N137, the charge-pump drive) and
    // `U5`–`U8` (Vishay VO2631, the eight isolated digital-input loops),
    // each an LED branch the engine solves and a sink that releases.
    registry.register("6N137", |_decl| Box::new(Opto::lite_on_6n137()));
    registry.register("VO2631", |_decl| Box::new(Opto::vo2631()));
    // The elements by specification (`NODES.md` §8 phase 3): the polarity
    // FET `U3` with its body diode, the transistor `Q1`, the eight
    // current regulators `IC6`–`IC13`, the two Schottky diodes `D1`/`D2`
    // and the 21 indicator LEDs, every one keyed on the manufacturer part
    // number the export carries.
    pwl_library::register(&mut registry);

    // The power tree: the two bucks at the version their value names, the
    // two isolated DC/DCs at the setpoint their `SEL` strap selects.
    registry.register(XL1509_PART, |decl: &ComponentDecl| {
        let config = rail::Config::xl1509_from_value(&decl.value).unwrap_or_else(|| {
            panic!(
                "{}: the value {:?} names no fixed version",
                decl.reference, decl.value
            )
        });
        Box::new(Rail::new(config, &XL1509_PINS_SOP8).expect("the XL1509 table carries every role"))
    });
    registry.register(UCC12040_PART, |_decl| {
        Box::new(
            Rail::new(rail::Config::ucc12040(), &UCC12040_PINS_SOIC16)
                .expect("the UCC12040 table carries every role"),
        )
    });
    registry
}

/// Build the MaD EdgeBoard as a [`Board`].
pub fn edge_board() -> Board {
    let parsed = embsim_board::netlist::parse(include_str!("../fixtures/mad_edge.net"))
        .expect("the EdgeBoard fixture parses");
    Board::from_netlist(parsed, &edge_registry()).expect("the EdgeBoard builds")
}

/// Build the DS2 force-gauge add-on as a [`Board`], with the live ADS122U04
/// component (`embsim-models`) as its `U1`.
pub fn ds2_board() -> Board {
    use embsim_models::ads122u04::Config;
    let mut registry = PartRegistry::new();
    registry.register("ADS122U04", |_decl| {
        Box::new(embsim_models::ads122u04_component::Ads122u04Component::new(
            Config {
                vref_mv: 2_048.0,
                gain: 1.0,
                zero_offset: 0,
            },
        ))
    });
    let parsed = embsim_board::netlist::parse(include_str!("../fixtures/ds2_addon.net"))
        .expect("the DS2Addon fixture parses");
    Board::from_netlist(parsed, &registry).expect("the DS2Addon builds")
}

// ============================================================
// Harnesses
// ============================================================

/// Parse a dotted harness endpoint.
pub fn ep(endpoint: &str) -> EndpointRef {
    EndpointRef::parse(endpoint).expect("endpoint parses")
}

/// The J203 edge fingers the module's netlist declares a node for, in finger
/// order: 3..=54 (the P0..P37 block, the 5 V and GND fingers, the reset
/// finger, the first two I/O-bank supplies, and P58..P63) plus 57, 58, 68, 78,
/// 79 and 80.
///
/// The 22 fingers left out — 1, 2, 55, 56, 59..67 and 69..77 — are the ones
/// the harness never wires. Fingers 1 and 2 are the vendor's `NC` pads: they
/// *are* declared in the module netlist (on its `NC_Net`, which
/// `ec32mb_module.rs` asserts), just reserved and not to be connected. The
/// other 20 are the P40..P57 signals — including P56/P57, the PSRAM CLK and
/// CE — and the V40/V48 bank supplies, which the module consumes internally
/// for its four PSRAMs (Rev B product guide: P40-P57 are routed to the
/// on-module 32 MB RAM). A socket pin with nothing behind it is the correct
/// model of a finger that carries nothing.
pub fn edge_fingers() -> impl Iterator<Item = u32> {
    (3..=54).chain([57, 58, 68, 78, 79, 80])
}

/// The module-to-EdgeBoard interconnect: every declared J203 finger wired to
/// the J3 socket pin of the same number.
///
/// **Numbering.** Both netlists number the 80-way card edge by *finger*
/// number — the module's `J203` because the transcription took the printed
/// edge labels as pin functions and the finger numbers as pin ids, the
/// EdgeBoard's `J3` because its socket symbol is drawn the same way. So the
/// mapping is the identity, and the harness can say `J203.46 ↔ J3.46`
/// directly. That is a property of these two netlists, not of card-edge
/// connectors in general: had either side numbered its symbol 1..40 per row,
/// or counted from the other end, this function would carry the translation
/// table instead. The tests assert the correspondence rather than trusting it —
/// `ec32mb_module.rs` checks the module side finger by finger against the
/// vendor product guide, and `machine_system.rs` checks that every declared
/// finger resolves to one node across the socket *and* that both sides label
/// the free P0..P37 block with the same pin numbers.
pub fn module_socket_harness(module: &str, edge: &str) -> Harness {
    let mut harness = Harness::new();
    for finger in edge_fingers() {
        harness = harness.connect(
            ep(&format!("{module}.J203.{finger}")),
            ep(&format!("{edge}.J3.{finger}")),
        );
    }
    harness
}

/// The force-gauge cable: EdgeBoard `J9` (isolated force domain) to the DS2
/// add-on's `J1`.
///
/// | EdgeBoard J9 | net | DS2 J1 | net |
/// |---|---|---|---|
/// | 1 | `IFG_5V` | 1 | `+3V3` |
/// | 5 | `IFG_GND` | 2 | `GND` |
/// | 4 | `IFG_TX` | 3 | ADC RX (through R3) |
/// | 2 | `IFG_RX` | 4 | ADC TX (through R4) |
/// | 3 | `IFG_INT` | 5 | ADC `~DRDY` (through R5) |
///
/// The supply label disagrees across the connector — the EdgeBoard calls the
/// isolated rail `IFG_5V`, the add-on calls the same wire `+3V3` — because the
/// UCC12040 is strapped for the ADC's 3.3 V domain and the EdgeBoard net kept
/// the family name. The wire is one net either way; the *voltage* comes from
/// whatever sources it (see [`bench_rails`]), not from either label.
pub fn force_gauge_harness(edge: &str, ds2: &str) -> Harness {
    Harness::new()
        .connect(ep(&format!("{edge}.J9.1")), ep(&format!("{ds2}.J1.1")))
        .connect(ep(&format!("{edge}.J9.5")), ep(&format!("{ds2}.J1.2")))
        .connect(ep(&format!("{edge}.J9.4")), ep(&format!("{ds2}.J1.3")))
        .connect(ep(&format!("{edge}.J9.2")), ep(&format!("{ds2}.J1.4")))
        .connect(ep(&format!("{edge}.J9.3")), ep(&format!("{ds2}.J1.5")))
}

/// The machine cables: the servo/stepper drive on `J21`, the encoder on `J20`,
/// and the two end-of-travel switches on `J16` / `J15`.
///
/// Three mapping decisions are worth stating, because the netlist and the
/// silkscreen do not agree and the netlist wins:
///
/// 1. **The motor takes the `+` leg of the differential pair.** `SC_PUL±` and
///    `SC_DIR±` leave J21 as RS-422 pairs driven by [`Rs422Driver`]; a real
///    stepper driver's own receiver turns each pair back into one logic
///    signal. `embsim-models`' `StepperMotor` stands in for driver *and*
///    motor, so it reads `SC_PUL+` / `SC_DIR+` and the complementary legs go
///    unread — the pair is still generated and asserted, it is simply
///    terminated by a model that only needs half of it.
/// 2. **The encoder drives the `+` leg, with `−` grounded by the board's own
///    jumpers.** `QuadratureEncoder` has single-ended `A`/`B` outputs, so
///    they land on `A+`/`B+`; JP2/JP3 (`A_GND`/`B_GND`) tie `A−`/`B−` to the
///    isolated ground, which is exactly what those jumpers are on the board
///    for. Closing JP4 (`Z_GND`) additionally asserts the receiver's
///    active-low enable — see [`Rs422Receiver`]'s board note. All three are
///    scenario state, so [`encoder_jumpers_closed`] carries them.
/// 3. **The end-switch connector labels are crossed with their nets.** J14 is
///    silkscreened `ENDUpper` but wired to `IDOOR±`, while J16 is
///    silkscreened `Door` and wired to `IEND_U±`. The isolator/opto chain
///    follows the *nets* (`IEND_U−` → U6 → `P19`, `IEND_L−` → U7 → `P20`,
///    `IDOOR−` → U7 → `P21`), and the firmware reads those pins, so this
///    harness follows the nets too: the upper end switch plugs into J16 and
///    the lower into J15. Worth fixing on the board; worth *knowing* now.
pub fn machine_harness(edge: &str) -> Harness {
    Harness::new()
        // Servo/stepper drive.
        .connect(ep(&format!("{edge}.J21.2")), ep("MOTOR.STEP"))
        .connect(ep(&format!("{edge}.J21.5")), ep("MOTOR.DIR"))
        .connect(ep(&format!("{edge}.J21.7")), ep("MOTOR.ENA"))
        // Encoder.
        .connect(ep("ENC.A"), ep(&format!("{edge}.J20.1")))
        .connect(ep("ENC.B"), ep(&format!("{edge}.J20.3")))
        // End of travel: upper on J16 (net IEND_U), lower on J15 (net IEND_L).
        .connect(ep(&format!("{edge}.J16.2")), ep("END_U.COM"))
        .connect(ep(&format!("{edge}.J16.1")), ep("END_U.NO"))
        .connect(ep(&format!("{edge}.J15.2")), ep("END_L.COM"))
        .connect(ep(&format!("{edge}.J15.1")), ep("END_L.NO"))
}

/// Bench supply straps for the EdgeBoard, through the board's own connector
/// pins — the rig a bring-up bench actually builds: the main input on the
/// screw terminal and the isolated servo domain on its connector. Nothing
/// else: `+5V` and `+3.3V` come from the board's own bucks `U1`/`U2`
/// behind the polarity FET `U3`, the instant the 12 V arrives (the XL1509
/// names no soft-start), so a strap on either would be a second declared
/// source on a rail the board generates.
///
/// | Endpoint | Net | Volts | Domain |
/// |---|---|---|---|
/// | `J2.1` | `Net-(J2-Pin_1)` | 12.0 | main input, ahead of the polarity FET |
/// | `J2.2` | `GND` | 0.0 | primary ground |
/// | `J21.1` | `SC_5V` | 5.0 | isolated servo domain |
/// | `J21.8` | `EN_GND` | 0.0 | isolated servo/encoder ground |
///
/// The isolated I/O domain (`5V_IO`/`GND_IO`, from `IC3`) is not strapped
/// and its return is tied to nothing on the board, so on the board alone
/// that rail stays down with its reference named — `DESIGN.md` rule 6, no
/// implicit ground: a test that needs the domain declares its return
/// through a connector pin (`J10.2`, `J5.3`–`J8.3`).
pub fn bench_rails(edge: &str) -> Harness {
    Harness::new()
        .power(ep("BENCH.12V"), ep(&format!("{edge}.J2.1")), 12.0)
        .power(ep("BENCH.GND"), ep(&format!("{edge}.J2.2")), 0.0)
        .power(
            ep("BENCH.SERVO5V"),
            ep(&format!("{edge}.J21.1")),
            SERVO_RAIL_VOLTS,
        )
        .power(ep("BENCH.SERVOGND"), ep(&format!("{edge}.J21.8")), 0.0)
}

// ============================================================
// Scenario fragments
// ============================================================

/// Close the encoder's ground/enable jumpers: JP2 (`A_GND`), JP3 (`B_GND`) and
/// JP4 (`Z_GND`).
///
/// These three are what let a single-ended encoder drive an RS-422 receiver —
/// they tie the `−` leg of each pair to the isolated ground so the receiver
/// sees a real differential, and JP4 additionally asserts the receiver's
/// active-low enable, which the board wires to `Z−`. See [`machine_harness`]
/// item 2 and [`Rs422Receiver`]'s board note.
pub fn encoder_jumpers_closed(scenario: Scenario, edge: &str) -> Scenario {
    scenario
        .jumper(&format!("{edge}.JP2"), JumperState::Closed)
        .jumper(&format!("{edge}.JP3"), JumperState::Closed)
        .jumper(&format!("{edge}.JP4"), JumperState::Closed)
}

/// Isolated force-domain straps for the DS2 add-on **on its own**, applied
/// on its connectors (`J1.1`/`J1.2`, `J2.1`/`J2.2`): the add-on has no
/// regulator, so on the bench its `+3V3` and analog supply are declared
/// here. Mirrors the DS2 bench rig in `board/tests/ds2_regressions.rs`.
/// With the add-on on the Edge board's cable the digital rail is the
/// board's `IC4` (5.0 V, `SEL` to `VISO`), so the assembled machine uses
/// [`force_domain_ground`] instead — a 3.3 V strap there would be a second
/// declared source on a 5 V rail.
pub fn force_domain_rails(ds2: &str) -> Harness {
    Harness::new()
        .power(
            ep("BENCH.IFG3V3"),
            ep(&format!("{ds2}.J1.1")),
            LOGIC_RAIL_VOLTS,
        )
        .power(ep("BENCH.IFGGND"), ep(&format!("{ds2}.J1.2")), 0.0)
        .power(
            ep("BENCH.VDDA"),
            ep(&format!("{ds2}.J2.1")),
            LOGIC_RAIL_VOLTS,
        )
        .power(ep("BENCH.AGND"), ep(&format!("{ds2}.J2.2")), 0.0)
}

/// The isolated force domain's **references** for the assembled machine:
/// the domain's return, tied down over the cable on the add-on's `J1.2`
/// (`IFG_GND`), and the add-on's analog supply and return on `J2`, which
/// nothing on either board generates. The domain's 5 V is the Edge board's
/// `IC4`, 750 µs after its input arrives (the UCC12040's rise time), so in
/// a build snapshot it is down and said so (`Finding::RailDown`). No
/// ground is implicit (`DESIGN.md` rule 6): an isolated domain's reference
/// is a harness terminal, as the primary bench return is.
pub fn force_domain_ground(ds2: &str) -> Harness {
    Harness::new()
        .power(ep("BENCH.IFGGND"), ep(&format!("{ds2}.J1.2")), 0.0)
        .power(
            ep("BENCH.VDDA"),
            ep(&format!("{ds2}.J2.1")),
            LOGIC_RAIL_VOLTS,
        )
        .power(ep("BENCH.AGND"), ep(&format!("{ds2}.J2.2")), 0.0)
}
