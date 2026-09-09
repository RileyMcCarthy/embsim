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
//! # Stub pin kinds — the rule this module follows everywhere
//!
//! Most parts here are **topology-only stubs**: they declare a real pin facade
//! (which the board build validates against the netlist in both directions, so
//! a symbol change breaks a test instead of going unnoticed) and no behavior.
//! Their pin kinds are not a stylistic choice:
//!
//! - **Power pins are declared truthfully** — [`PinKind::PowerIn`] for a rail
//!   the part consumes, [`PinKind::PowerOut`] for one it generates. Unsourced
//!   rails are the highest-value diagnostic this slice produces (the DS2
//!   bench's unstrapped AVDD is the reference case), and that only works if
//!   supply pins say what they are.
//! - **Signal pins are declared [`PinKind::DigitalIn`]** — a *sense*,
//!   contributing no drive — even where the real part drives. A stub has no
//!   behavior with which to decide a level, and the engine's idle-high default
//!   for a [`PinKind::DigitalOut`] would fabricate a drive the part does not
//!   produce (and could manufacture [`embsim_board::Finding::Contention`] against a real
//!   driver). Declared as senses they still participate in resolution and
//!   still yield honest [`embsim_board::Finding::FloatingSense`] reports, while every actual
//!   drive in the system comes from a modeled component.
//! - **Pins the schematic marks no-connect are [`PinKind::Passive`]**
//!   ([`nc`]), so an intentionally dangling pad raises nothing.
//!
//! The engine takes electrical descriptors from the component facade and never
//! from the schematic (see the `netlist` module docs), which is what lets these
//! tables *correct* a symbol: the EdgeBoard's `XL1509` symbol draws all eight
//! pins as `input`, including VIN, OUT and the four grounds. [`XL1509_PINS`]
//! declares what the part is.
//!
//! # Modeled parts (real behavior)
//!
//! Three parts carry behavior, each with the datasheet header
//! `BOARD_ENGINE.md` ("Model provenance convention") requires:
//!
//! - [`Rs422Driver`] — TI AM26LS31, the servo step/direction pair;
//! - [`Rs422Receiver`] — TI AM26LV32, the encoder A/B/ZI pairs;
//! - [`SerialIsolator`] — TI ISO6731, the isolated force-gauge UART.
//!
//! Everything else on both boards is a stub or an auto-classified primitive.

use std::sync::{Arc, Mutex, MutexGuard};

use embsim_board::mcu::SerialChannelConfig;
use embsim_board::{
    AttachError, Board, Component, ComponentNetIo, EndpointRef, Harness, JumperState, Level,
    McuComponent, NetState, Ohms, PartRegistry, PinDecl, PinHandle, PinKind, Scenario, StreamRole,
    TheveninDrive, Volts,
};

// ============================================================
// Pin-declaration helpers
// ============================================================

/// A pin the component senses and never drives.
pub const fn dig_in(number: &'static str) -> PinDecl {
    PinDecl {
        number,
        name: None,
        kind: PinKind::DigitalIn,
        stream: None,
        drive_impedance: None,
    }
}

/// A push-pull output pin (idles `Driven(High)` until the component drives).
pub const fn dig_out(number: &'static str) -> PinDecl {
    PinDecl {
        number,
        name: None,
        kind: PinKind::DigitalOut,
        stream: None,
        drive_impedance: None,
    }
}

/// A pin whose *voltage* the component needs (participates in the cluster
/// solve) — a differential receiver input, an ADC input.
pub const fn analog(number: &'static str) -> PinDecl {
    PinDecl {
        number,
        name: None,
        kind: PinKind::Analog,
        stream: None,
        drive_impedance: None,
    }
}

/// A rail the part consumes.
pub const fn pwr_in(number: &'static str) -> PinDecl {
    PinDecl {
        number,
        name: None,
        kind: PinKind::PowerIn,
        stream: None,
        drive_impedance: None,
    }
}

/// A rail the part generates (regulator/DC-DC output, isolated-domain
/// reference). Registers the net as sourced at an unmodeled voltage — enough
/// to clear [`embsim_board::Finding::PowerNetUnsourced`], not enough for a
/// component that gates on a rail *voltage*; see [`bench_rails`].
pub const fn pwr_out(number: &'static str) -> PinDecl {
    PinDecl {
        number,
        name: None,
        kind: PinKind::PowerOut,
        stream: None,
        drive_impedance: None,
    }
}

/// A terminal that contributes nothing electrical.
pub const fn passive(number: &'static str) -> PinDecl {
    PinDecl {
        number,
        name: None,
        kind: PinKind::Passive,
        stream: None,
        drive_impedance: None,
    }
}

/// A pin the schematic marks no-connect. Spelled distinctly from [`passive`]
/// so the tables read as documentation: the facade must still declare the pin
/// (the netlist has a node for it, on an `unconnected-(…)` stub net), and
/// declaring it passive keeps a deliberately dangling pad out of the findings.
pub const fn nc(number: &'static str) -> PinDecl {
    passive(number)
}

/// A UART transmit pin: drives the net and paces bytes onto the derived route.
pub const fn stream_out(number: &'static str, baud_hz: u32) -> PinDecl {
    PinDecl {
        number,
        name: None,
        kind: PinKind::DigitalOut,
        stream: Some(StreamRole::Producer { baud_hz }),
        drive_impedance: None,
    }
}

/// A UART receive pin: senses the net and consumes routed bytes.
pub const fn stream_in(number: &'static str, baud_hz: u32) -> PinDecl {
    PinDecl {
        number,
        name: None,
        kind: PinKind::DigitalIn,
        stream: Some(StreamRole::Consumer { baud_hz }),
        drive_impedance: None,
    }
}

// ============================================================
// Topology-only stub
// ============================================================

/// A part with a real pin facade and no behavior. See the module docs for how
/// its pin kinds are chosen and why.
#[derive(Debug)]
pub struct StubPart {
    pins: &'static [PinDecl],
}

impl StubPart {
    /// A stub declaring `pins`.
    pub const fn new(pins: &'static [PinDecl]) -> Self {
        Self { pins }
    }
}

impl Component for StubPart {
    fn pins(&self) -> &[PinDecl] {
        self.pins
    }

    fn attach(&mut self, _io: ComponentNetIo) -> Result<(), AttachError> {
        Ok(())
    }
}

/// Register `part` as a [`StubPart`] declaring `pins`.
pub fn register_stub(registry: &mut PartRegistry, part: &str, pins: &'static [PinDecl]) {
    registry.register(part, move |_decl| Box::new(StubPart::new(pins)));
}

// ============================================================
// Shared electrical helpers
// ============================================================

/// Logic level a resolved net state defensibly implies, or `None`
/// ([`NetState::Floating`] / [`NetState::Contention`] — the engine invents no
/// value and neither does a model).
fn level_of(state: NetState, threshold_volts: Volts) -> Option<Level> {
    match state {
        NetState::Driven(level) | NetState::Pulled(level, _) => Some(level),
        NetState::Analog(volts) if volts.is_nan() => None,
        NetState::Analog(volts) if volts >= threshold_volts => Some(Level::High),
        NetState::Analog(_) => Some(Level::Low),
        NetState::Floating | NetState::Contention => None,
    }
}

/// Node voltage a resolved state defensibly implies, or `None`. A digital
/// projection is mapped back to its rail (`rail_volts` / 0 V) so a
/// differential model can compare a solved node against a driven one.
fn volts_of(state: NetState, rail_volts: Volts) -> Option<Volts> {
    match state {
        NetState::Driven(Level::High) | NetState::Pulled(Level::High, _) => Some(rail_volts),
        NetState::Driven(Level::Low) | NetState::Pulled(Level::Low, _) => Some(0.0),
        NetState::Analog(volts) if volts.is_finite() => Some(volts),
        _ => None,
    }
}

/// Mid-rail logic threshold for the 3.3 V / 5 V parts here.
const LOGIC_THRESHOLD_VOLTS: Volts = 1.5;

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
//               Line Driver" (TI literature number SLLS103; see the citation
//               note at the end of this block).
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
//   revision of this part. The literature number is recorded from memory and
//   the revision letter is deliberately absent: when this model is promoted
//   out of the test tree, re-check the number and pin the exact revision, and
//   add per-behavior "(§x.y, p.N)" citations the way
//   `embsim-models`' ADS122U04 model does against SBAS752B. Behavior with no
//   datasheet basis is a defect; behavior with an unverified *document number*
//   is a gap in the paperwork, and this note is it.
//

/// Pin facade of the `AM26LS31CD` (SOIC-16), pin numbers as the EdgeBoard
/// netlist names them.
#[rustfmt::skip]
pub const AM26LS31_PINS: [PinDecl; 16] = [
    dig_in("1"),   // 1A  — channel-1 input
    dig_out("2"),  // 1Y  — channel-1 true output
    dig_out("3"),  // 1Z  — channel-1 complement
    dig_in("4"),   // G   — active-high enable
    dig_out("5"),  // 2Z  — channel-2 complement
    dig_out("6"),  // 2Y  — channel-2 true output
    dig_in("7"),   // 2A  — channel-2 input
    pwr_in("8"),   // GND
    nc("9"),       // 3A  — unused on this board
    nc("10"),      // 3Y
    nc("11"),      // 3Z
    dig_in("12"),  // ~G  — active-low enable
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
}

struct DriverCore {
    rail_volts: Volts,
    state: Mutex<DriverState>,
}

impl DriverCore {
    /// True when the SLLS103 enable structure has the outputs active: G high
    /// OR ~G low.
    fn enabled(state: &DriverState) -> bool {
        state.enable_high == Some(Level::High) || state.enable_low == Some(Level::Low)
    }

    /// Re-drive every channel from the current inputs (or release when
    /// disabled/unpowered).
    fn apply(&self, state: &mut DriverState) {
        let active = state.powered && Self::enabled(state);
        for (channel, (y, z)) in state.outputs.iter().enumerate() {
            match (active, state.inputs[channel]) {
                (true, Some(level)) => {
                    y.set_drive(Some(drive(level, self.rail_volts)));
                    z.set_drive(Some(drive(invert(level), self.rail_volts)));
                }
                // Disabled, unpowered, or an input with no defensible level:
                // high-Z, never a guessed differential.
                _ => {
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
            state.powered = level_of(rail, LOGIC_THRESHOLD_VOLTS) == Some(Level::High);
            core.apply(&mut state);
        })?;
        for (pin, slot) in [("4", true), ("12", false)] {
            let core = Arc::clone(&self.core);
            io.on_sense(pin, move |sensed| {
                let mut state = core.state.lock().unwrap();
                let level = level_of(sensed, LOGIC_THRESHOLD_VOLTS);
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
            io.on_sense(a, move |sensed| {
                let mut state = core.state.lock().unwrap();
                state.inputs[channel] = level_of(sensed, LOGIC_THRESHOLD_VOLTS);
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
//               Differential Line Receiver" (TI literature number SLLS329;
//               see the citation note on `Rs422Driver`, which applies here
//               too).
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
//   the *solved node voltages* (the input pins are declared `Analog`, so they
//   join the cluster solve), and Y is driven high for V_ID >= +200 mV, low for
//   V_ID <= −200 mV. Inside the ±200 mV band — the shorted-pair and
//   idle-terminated cases — and whenever either leg has no defensible voltage
//   (an open input), the datasheet's failsafe forces Y **high**. Disabled or
//   unpowered releases Y to high-Z.
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
//   * Input common-mode range and the receiver's own input resistance: the
//     input pins are high-Z senses, so a real termination network across the
//     pair comes from the netlist's own resistors, not from here.
//   * The supply-range check — an out-of-range VDD is a rail finding, not a
//     receiver behavior.
//

/// Pin facade of the `AM26LV32xD` (SOIC-16), pin numbers as the EdgeBoard
/// netlist names them.
#[rustfmt::skip]
pub const AM26LV32_PINS: [PinDecl; 16] = [
    analog("1"),   // 1B
    analog("2"),   // 1A
    dig_out("3"),  // 1Y
    dig_in("4"),   // G   — active-high enable (wired to the encoder's Z+)
    dig_out("5"),  // 2Y
    analog("6"),   // 2A
    analog("7"),   // 2B
    pwr_in("8"),   // GND
    analog("9"),   // 3B
    analog("10"),  // 3A
    dig_out("11"), // 3Y
    dig_in("12"),  // ~G  — active-low enable (wired to the encoder's Z−)
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

/// SLLS329 differential input threshold magnitude: V_IT+ <= +200 mV,
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
    /// SLLS329 enable structure — identical OR form to the driver's.
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
            state.powered = level_of(rail, LOGIC_THRESHOLD_VOLTS) == Some(Level::High);
            core.apply(&mut state);
        })?;
        for (pin, is_active_high) in [("4", true), ("12", false)] {
            let core = Arc::clone(&self.core);
            io.on_sense(pin, move |sensed| {
                let mut state = core.state.lock().unwrap();
                let level = level_of(sensed, LOGIC_THRESHOLD_VOLTS);
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
                let rail = self.core.rail_volts;
                io.on_sense(pin, move |sensed| {
                    let mut state = core.state.lock().unwrap();
                    let volts = volts_of(sensed, rail);
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

/// Pin facade of the `ISO6731DWR` (SOIC-16 wide), pin numbers as the
/// EdgeBoard netlist names them.
#[rustfmt::skip]
pub fn iso6731_pins() -> Vec<PinDecl> {
    vec![
        pwr_in("1"),    // VCC1   — isolated side
        pwr_in("2"),    // GND1_1
        dig_in("3"),    // INA    — gauge transmit in
        dig_in("4"),    // INB    — gauge ~DRDY in
        dig_out("5"),   // OUTC   — MCU transmit out (isolated side)
        nc("6"),        // NC_1
        nc("7"),        // EN1
        pwr_in("8"),    // GND1_2
        pwr_in("9"),    // GND2_1
        nc("10"),       // EN2
        nc("11"),       // NC_2
        dig_in("12"),   // INC    — MCU transmit in
        dig_out("13"),  // OUTB   — ~DRDY out
        dig_out("14"),  // OUTA   — gauge transmit out
        pwr_in("15"),   // GND2_2
        pwr_in("16"),   // VCC2   — MCU side
    ]
}

/// The three repeated channels, `(input pin, output pin)`, as the EdgeBoard
/// wires them: MCU transmit, gauge transmit, and the ADC's `~DRDY`.
const ISO6731_CHANNELS: [(&str, &str); 3] = [("12", "5"), ("3", "14"), ("4", "13")];

struct IsolatorCore {
    rail_volts: Volts,
    state: Mutex<IsolatorState>,
}

#[derive(Default)]
struct IsolatorState {
    side1_powered: bool,
    side2_powered: bool,
    /// Per channel: the level last sensed on its input, and its output pin.
    level_in: [Option<Level>; 3],
    level_out: [Option<PinHandle>; 3],
}

impl IsolatorCore {
    fn live(state: &IsolatorState) -> bool {
        state.side1_powered && state.side2_powered
    }

    /// Re-drive one channel's output from its input.
    fn apply(&self, state: &mut IsolatorState, channel: usize) {
        let Some(out) = state.level_out[channel].clone() else {
            return;
        };
        match (Self::live(state), state.level_in[channel]) {
            (true, Some(level)) => out.set_drive(Some(drive(level, self.rail_volts))),
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
    /// An isolator whose repeated outputs drive `rail_volts`.
    pub fn new(rail_volts: Volts) -> Self {
        Self {
            pins: iso6731_pins(),
            core: Arc::new(IsolatorCore {
                rail_volts,
                state: Mutex::new(IsolatorState::default()),
            }),
        }
    }
}

impl std::fmt::Debug for SerialIsolator {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SerialIsolator")
            .field("rail_volts", &self.core.rail_volts)
            .finish()
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
        for (pin, is_side1) in [("1", true), ("16", false)] {
            let core = Arc::clone(&self.core);
            io.on_sense(pin, move |rail| {
                let mut state = core.state.lock().unwrap();
                let up = level_of(rail, LOGIC_THRESHOLD_VOLTS) == Some(Level::High);
                if is_side1 {
                    state.side1_powered = up;
                } else {
                    state.side2_powered = up;
                }
                core.apply_all(&mut state);
            })?;
        }

        // Each channel subscribes only to its own input, so one transition
        // costs one drive rather than one per channel.
        for (channel, (input, _)) in ISO6731_CHANNELS.iter().enumerate() {
            let core = Arc::clone(&self.core);
            io.on_sense(input, move |sensed| {
                let mut state = core.state.lock().unwrap();
                state.level_in[channel] = level_of(sensed, LOGIC_THRESHOLD_VOLTS);
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
const P2_IO_NAMES: [&str; 64] = [
    "P0",  "P1",  "P2",  "P3",  "P4",  "P5",  "P6",  "P7",
    "P8",  "P9",  "P10", "P11", "P12", "P13", "P14", "P15",
    "P16", "P17", "P18", "P19", "P20", "P21", "P22", "P23",
    "P24", "P25", "P26", "P27", "P28", "P29", "P30", "P31",
    "P32", "P33", "P34", "P35", "P36", "P37", "P38", "P39",
    "P40", "P41", "P42", "P43", "P44", "P45", "P46", "P47",
    "P48", "P49", "P50", "P51", "P52", "P53", "P54", "P55",
    "P56", "P57", "P58", "P59", "P60", "P61", "P62", "P63",
];

/// The 16 per-bank I/O supply pins the P2 package brings out, as the module
/// netlist names them (each bank's eight pins are drawn as two symbol pins).
#[rustfmt::skip]
const P2_VIO_NAMES: [&str; 16] = [
    "VIO_0_3",   "VIO_4_7",   "VIO_8_11",  "VIO_12_15",
    "VIO_16_19", "VIO_20_23", "VIO_24_27", "VIO_28_31",
    "VIO_32_35", "VIO_36_39", "VIO_40_43", "VIO_44_47",
    "VIO_48_51", "VIO_52_55", "VIO_56_59", "VIO_60_63",
];

/// The P2 as the EC32MB module's `U100`: an [`McuComponent`] wearing the full
/// physical pin facade the vendor symbol draws.
///
/// The two live at different altitudes and this adapter is the seam between
/// them. `McuComponent` declares exactly the pins its emulated peripherals
/// bridge — two, for the bridged force-gauge UART — because that is what it
/// knows. The *board* knows the package: 64 I/O pins, a core supply, sixteen
/// bank supplies, `RESN`, `TEST`, and the crystal pair, all of which the
/// netlist has nodes for and all of which the build validates in both
/// directions. So this component declares the package and delegates behavior:
///
/// - [`Component::pins`] is the union — the MCU's two bridged UART pins (plain
///   digital: the channel carries levels, so there is no byte route to
///   declare), every other I/O pin as a sense (a P2 pin is high-Z out of reset,
///   and nothing in this slice drives it), the supplies as
///   [`PinKind::PowerIn`];
/// - [`Component::attach`] and [`Component::start`] hand straight through to
///   the `McuComponent`, which finds `"P2"` and `"P0"` in the handle table it
///   is given and bridges them exactly as it would on a board of its own.
///
/// `XO` is declared a *sense*, not an output: the emulator models no
/// oscillator, and on this module the pin is unused anyway (the vendor drives
/// `XI` from an external TCXO). The resulting `FloatingSense` on `XTAL_XO` is
/// the honest report, and `ec32mb_module.rs` asserts it.
pub struct P2EdgeModule {
    pins: Vec<PinDecl>,
    mcu: McuComponent,
}

impl P2EdgeModule {
    /// Build the adapter with the force-gauge channel bridged.
    pub fn new(name: &str) -> Self {
        let channel = FORCE_GAUGE_CHANNEL;
        let mut pins = Vec::with_capacity(86);
        for (index, number) in P2_IO_NAMES.into_iter().enumerate() {
            let index = index as u32;
            // The UART's TX pin is the only one this slice drives; the RX pin
            // reads levels like every other I/O.
            if index == channel.tx_pin {
                pins.push(dig_out(number));
            } else {
                pins.push(dig_in(number));
            }
        }
        pins.push(pwr_in("VDD"));
        pins.push(pwr_in("GND"));
        pins.extend(P2_VIO_NAMES.into_iter().map(pwr_in));
        // TEST is a mode strap (tied to GND on this module) and RESN an
        // active-low input; XI is driven by the module's oscillator chain and
        // XO is unused — all four are senses.
        pins.push(dig_in("TEST"));
        pins.push(dig_in("RESN"));
        pins.push(dig_in("XI"));
        pins.push(dig_in("XO"));

        let mcu = McuComponent::builder(name)
            .serial_table(vec![channel])
            .bridge_serial(0)
            // The UART crosses the card edge, an isolation barrier, a cable and
            // two 47 ohm series resistors to reach the ADC — every one of them
            // a thing that can only affect a signal that is actually on the
            // net. So the channel carries levels, like the part at the far end.
            .serial_on_levels()
            .build()
            .expect("the force-gauge channel is in the table and inside P63");

        Self { pins, mcu }
    }

    /// The wrapped MCU component (peripheral instance, entry state).
    pub fn mcu(&self) -> &McuComponent {
        &self.mcu
    }
}

impl std::fmt::Debug for P2EdgeModule {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("P2EdgeModule")
            .field("pins", &self.pins.len())
            .field("mcu", &self.mcu)
            .finish()
    }
}

impl Component for P2EdgeModule {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        self.mcu.attach(io)
    }

    fn start(&mut self) {
        self.mcu.start();
    }
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
// P2-EC32MB module: stub facades
// ============================================================
//
// Classification, part by part. The module's netlist carries no `libsource`,
// so the registry runs with `classify_unnamed_by_reference(true)` and keys the
// active parts on their `value` field:
//
//   auto (reference prefix)  85 passives (C×66, R×15, L×2, D×2 LEDs) and the
//                            five J-prefixed pad/socket symbols — J203 (the
//                            80-finger card edge), J301 (microSD socket),
//                            J101 (the oscillator-option solder link) and the
//                            two mounting-hole pads J701/J702 — as boundaries.
//   real model               U100, the P2 (see `P2EdgeModule`).
//   stub                     every other active part, below.
//   per-board stub list      PCB and NC_Net: BOM/layout-only reference
//                            designators with no electrical existence at all
//                            (PCB has no nodes; NC_Net has one). They are the
//                            documented escape rather than fake components.
//
// Why the rest are stubs and not models: none of them is on a signal path any
// consumer test drives. The PSRAMs and the flash answer a QSPI controller the
// firmware does not exercise in SIL; the LDOs, bucks, polarity FET and
// brownout detector matter as *power topology*, which their PowerIn/PowerOut
// declarations already express; the TCXO and its inverter buffer matter only
// as the reason XTAL_XI exists. Promoting any of them is additive — the
// facades below are already the netlist-validated boundary.

/// `74LVC2G04GW,125` — NXP dual inverter (U101 oscillator buffer, U601 LED
/// buffer). Outputs are senses per the module docs' stub rule.
pub const INVERTER_2G04_PINS: [PinDecl; 6] = [
    pwr_in("GND"),
    dig_in("1A"),
    dig_in("1Y"),
    dig_in("2A"),
    dig_in("2Y"),
    pwr_in("VCC"),
];

/// `TG2520SMN 20.0000M-ECGNNM3` — EPSON 20 MHz TCXO (X100). `NC_GND` is the
/// vendor's option pad, brought to the `J101` solder link.
pub const TCXO_PINS: [PinDecl; 4] = [
    pwr_in("VCC"),
    pwr_in("GND"),
    dig_in("OUT"),
    passive("NC_GND"),
];

/// `SPI Flash 16MB (128Mb)` — Winbond W25Q128JV (U301).
pub const SPI_FLASH_PINS: [PinDecl; 8] = [
    pwr_in("VSS"),
    pwr_in("VCC"),
    dig_in("CLK"),
    dig_in("CSn"),
    dig_in("DI_IO0"),
    dig_in("DO_IO1"),
    dig_in("HOLDn"),
    dig_in("WPn"),
];

/// `PSRAM 64Mbit` — AP Memory APS6404L (U302..U305). `NC_EP` is the exposed
/// pad, tied per the vendor's asymmetric routing note.
pub const PSRAM_PINS: [PinDecl; 9] = [
    pwr_in("VSS"),
    pwr_in("VDD"),
    passive("NC_EP"),
    dig_in("SCLK"),
    dig_in("CEn"),
    dig_in("SI_SIO0"),
    dig_in("SO_SIO1"),
    dig_in("SIO2"),
    dig_in("SIO3"),
];

/// `P Mosfet 30V 8A` — Vishay SI3417DV reverse-polarity pass FET (U401). Its
/// conducting channel is not modeled; the system description expresses it with
/// a `pin_short` (see [`module_polarity_fet_conducting`]).
pub const POLARITY_FET_PINS: [PinDecl; 3] = [dig_in("G"), passive("D"), passive("S")];

/// `DCDC 3A SOT563` — Diodes AP62301Z buck (U402, U403). `SW` is declared
/// [`PinKind::PowerOut`]: it is the switching node the output inductor
/// integrates into a rail, and marking it a source is what makes the module's
/// power tree reachable.
pub const BUCK_PINS: [PinDecl; 5] = [
    pwr_in("VIN"),
    pwr_in("GND"),
    passive("BST"),
    dig_in("FB"),
    pwr_out("SW"),
];

/// `Voltage Detector 1.6V` — STMicro STM1061 brownout detector (U404). `~OUT`
/// is open-drain and unmodeled, so it is a sense.
pub const BROWNOUT_PINS: [PinDecl; 3] = [pwr_in("VCC"), pwr_in("VSS"), dig_in("OUT")];

/// `LDO 300mA, 3.3V` — OnSemi NCP114 (U501..U508), one per P2 I/O bank pair.
pub const LDO_PINS: [PinDecl; 5] = [
    dig_in("EN"),
    pwr_in("IN"),
    pwr_in("GND"),
    pwr_in("GND_P"),
    pwr_out("OUT"),
];

/// `DIP Switch 4 way` — the module's option switch (S301), pin ids
/// `<position>_<OFF|ON>`.
///
/// All eight terminals are passive: a four-gang switch is not a two-terminal
/// jumper, so the auto tier cannot classify it and the engine has no
/// scenario primitive for a ganged position yet. The consequence is visible
/// and asserted — with every gang open, the `P59` pull-up/pull-down and the
/// flash chip-select strap do not conduct.
pub const DIP_SWITCH_PINS: [PinDecl; 8] = [
    passive("1_ON"),
    passive("1_OFF"),
    passive("2_ON"),
    passive("2_OFF"),
    passive("3_ON"),
    passive("3_OFF"),
    passive("4_ON"),
    passive("4_OFF"),
];

/// Reference designators the module netlist declares that have no electrical
/// existence: `PCB` is the raw board (no nodes at all) and `NC_Net` a layout
/// node. Passed to [`Board::from_netlist_with_stubs`].
pub const EC32MB_STUB_REFS: [&str; 2] = ["PCB", "NC_Net"];

/// Part registry for `fixtures/p2_ec32mb.net`.
pub fn ec32mb_registry() -> PartRegistry {
    let mut registry = PartRegistry::new();
    // The netlist was transcribed from the vendor PDF and has no libsource, so
    // the auto tier keys on reference-designator prefixes and the registry on
    // the `value` field.
    registry.classify_unnamed_by_reference(true);

    registry.register("P2X8C4M64P", |_decl| Box::new(P2EdgeModule::new("p2")));
    register_stub(&mut registry, "74LVC2G04GW,125", &INVERTER_2G04_PINS);
    register_stub(&mut registry, "TG2520SMN 20.0000M-ECGNNM3", &TCXO_PINS);
    register_stub(&mut registry, "SPI Flash 16MB (128Mb)", &SPI_FLASH_PINS);
    register_stub(&mut registry, "PSRAM 64Mbit", &PSRAM_PINS);
    register_stub(&mut registry, "P Mosfet 30V 8A", &POLARITY_FET_PINS);
    register_stub(&mut registry, "DCDC 3A SOT563", &BUCK_PINS);
    register_stub(&mut registry, "Voltage Detector 1.6V", &BROWNOUT_PINS);
    register_stub(&mut registry, "LDO 300mA, 3.3V", &LDO_PINS);
    register_stub(&mut registry, "DIP Switch 4 way", &DIP_SWITCH_PINS);
    registry
}

/// Build the P2-EC32MB module as a [`Board`].
pub fn ec32mb_board() -> Board {
    let parsed = embsim_board::netlist::parse(include_str!("../fixtures/p2_ec32mb.net"))
        .expect("the EC32MB fixture parses");
    Board::from_netlist_with_stubs(parsed, &ec32mb_registry(), &EC32MB_STUB_REFS)
        .expect("the EC32MB module builds")
}

// ============================================================
// MaD EdgeBoard: stub facades
// ============================================================
//
// Classification, part by part (this netlist is a real KiCad export, so every
// tier keys on the libsource part name as usual):
//
//   auto        R_Small ×32, C_Small ×29, LED ×21, D_Schottky_Small ×2 (SS36),
//               L_Small ×2 — the 86 passives; Conn_01x0n / Screw_Terminal ×23
//               plus the declared P2_EDGE_MODULE_SOCKET — the 24 boundaries;
//               Jumper_2_Open ×4 + Jumper_3_Open ×1 — the stateful shorts;
//               MountingHole_Pad ×4 — ignored.
//               (86 + 24 + 5 + 4 + 49 registered = the netlist's 168.)
//   real model  AM26LS31CD (U24) and AM26LV32xD (U25), the encoder/servo
//               RS-422 pair; ISO6731DWR (IC5), the force-gauge UART isolator.
//   stub        the other 13 active types, below.
//
// The ISO674x/ISO672x symbols draw every pin as `passive`; their facades here
// keep that (topology-only) rather than inventing directions, because on this
// board they are all *transparent* barriers between an MCU pin and a connector
// pin, and topology is exactly what a build test needs from them. Promoting
// one to a repeater is a two-line change against `SerialIsolator`'s shape.

/// `ISO6742DWR` — TI quad digital isolator, 2 forward + 2 reverse (IC1: the
/// isolated GPIO block; IC2: the Raspberry-Pi UART/GPIO block).
#[rustfmt::skip]
pub const ISO6742_PINS: [PinDecl; 16] = [
    pwr_in("1"),  // VCC1
    pwr_in("2"),  // GND1_1
    dig_in("3"),  // INA
    dig_in("4"),  // INB
    dig_in("5"),  // OUTC
    dig_in("6"),  // OUTD
    nc("7"),      // EN1
    pwr_in("8"),  // GND1_2
    pwr_in("9"),  // GND2_1
    nc("10"),     // EN2
    dig_in("11"), // IND
    dig_in("12"), // INC
    dig_in("13"), // OUTB
    dig_in("14"), // OUTA
    pwr_in("15"), // GND2_2
    pwr_in("16"), // VCC2
];

/// `ISO6741DWR` — TI quad digital isolator, 3 forward + 1 reverse (IC14: the
/// servo control block on sheet 3).
#[rustfmt::skip]
pub const ISO6741_PINS: [PinDecl; 16] = [
    pwr_in("1"),  // VCC1
    pwr_in("2"),  // GND1_1
    dig_in("3"),  // INA
    dig_in("4"),  // INB
    dig_in("5"),  // INC
    dig_in("6"),  // OUTD
    nc("7"),      // EN1
    pwr_in("8"),  // GND1_2
    pwr_in("9"),  // GND2_1
    nc("10"),     // EN2
    dig_in("11"), // IND
    dig_in("12"), // OUTC
    dig_in("13"), // OUTB
    dig_in("14"), // OUTA
    pwr_in("15"), // GND2_2
    pwr_in("16"), // VCC2
];

/// `ISO6740FDWR` — TI quad digital isolator, 4 forward with fail-safe
/// (IC16: the encoder block, carrying the RS-422 receiver's outputs across to
/// the P2).
#[rustfmt::skip]
pub const ISO6740_PINS: [PinDecl; 16] = [
    pwr_in("1"),  // VCC1
    pwr_in("2"),  // GND1_1
    dig_in("3"),  // INA
    dig_in("4"),  // INB
    dig_in("5"),  // INC
    dig_in("6"),  // IND
    nc("7"),      // NC
    pwr_in("8"),  // GND1_2
    pwr_in("9"),  // GND2_1
    nc("10"),     // EN2
    dig_in("11"), // OUTD
    dig_in("12"), // OUTC
    dig_in("13"), // OUTB
    dig_in("14"), // OUTA
    pwr_in("15"), // GND2_2
    pwr_in("16"), // VCC2
];

/// `ISO6721BDR` — TI dual digital isolator, 1 forward + 1 reverse (IC15: the
/// isolated servo-serial UART on sheet 3).
#[rustfmt::skip]
pub const ISO6721_PINS: [PinDecl; 8] = [
    pwr_in("1"), // VCC1
    dig_in("2"), // OUTA
    dig_in("3"), // INB
    pwr_in("4"), // GND1
    pwr_in("5"), // GND2
    dig_in("6"), // OUTB
    dig_in("7"), // INA
    pwr_in("8"), // VCC2
];

/// `UCC12040DVER` — TI isolated 500 mW DC/DC module (IC3: the isolated I/O
/// domain; IC4: the force-gauge domain).
///
/// `VISO` **and** the three `GNDS` pins are declared [`PinKind::PowerOut`]:
/// an isolated DC/DC generates a whole domain, its ground reference included,
/// so both terminals of that domain are sourced by this part. Declaring GNDS
/// as an input instead would leave every isolated ground permanently
/// unsourced and bury the real findings in noise.
#[rustfmt::skip]
pub const UCC12040_PINS: [PinDecl; 16] = [
    dig_in("1"),   // EN
    pwr_in("2"),   // GNDP
    pwr_in("3"),   // VINP
    dig_in("4"),   // SYNC
    nc("5"),       // SYNC_OK
    passive("6"),  // NC_1
    passive("7"),  // NC_2
    passive("8"),  // NC_3
    pwr_out("9"),  // GNDS_1
    passive("10"), // NC_4
    passive("11"), // NC_5
    passive("12"), // NC_6
    dig_in("13"),  // SEL
    pwr_out("14"), // VISO
    pwr_out("15"), // GNDS_2
    pwr_out("16"), // GNDS_3
];

/// `NSI50010YT1G` — OnSemi 10 mA constant-current LED driver (IC6..IC13), one
/// per isolated digital input loop. A two-terminal current regulator with no
/// DC path modeled.
pub const NSI50010_PINS: [PinDecl; 2] = [passive("1"), passive("2")];

/// `6N137` — high-speed optocoupler (U4: the charge-pump drive). `VO` is
/// open-collector and unmodeled, hence a sense.
#[rustfmt::skip]
pub const OPTO_6N137_PINS: [PinDecl; 7] = [
    nc("1"),      // NC
    passive("2"), // A  — LED anode
    passive("3"), // C  — LED cathode
    pwr_in("5"),  // GND
    dig_in("6"),  // VO (open collector)
    dig_in("7"),  // EN
    pwr_in("8"),  // VCC
];

/// `VO2631` — Vishay dual high-speed optocoupler (U5..U8), receiving the eight
/// isolated digital-input loops. `VO1`/`VO2` are open-collector: declaring
/// them senses lets the board's own pull-up resistors set the idle level,
/// which is what the netlist actually describes.
#[rustfmt::skip]
pub const VO2631_PINS: [PinDecl; 8] = [
    passive("1"), // A1
    passive("2"), // C1
    passive("3"), // C2
    passive("4"), // A2
    pwr_in("5"),  // GND
    dig_in("6"),  // VO2 (open collector)
    dig_in("7"),  // VO1 (open collector)
    pwr_in("8"),  // VCC
];

/// `SN74LVC1G14DBV` — TI single Schmitt-trigger inverter (21 instances, one
/// per front-panel status LED).
#[rustfmt::skip]
pub const SN74LVC1G14_PINS: [PinDecl; 5] = [
    nc("1"),     // NC
    dig_in("2"), // A
    pwr_in("3"), // GND
    dig_in("4"), // Y
    pwr_in("5"), // VCC
];

/// `XL1509` — 2 A step-down converter (U1: +5 V, U2: +3.3 V).
///
/// This facade **corrects the schematic symbol**, which draws all eight pins
/// as `input` — including `VIN`, the four grounds, and `OUT`. Electrical
/// descriptors come from the component, never from the netlist (see the
/// `netlist` module docs), and getting `OUT` right as a
/// [`PinKind::PowerOut`] is what makes the board's rails reachable through
/// the output inductors.
#[rustfmt::skip]
pub const XL1509_PINS: [PinDecl; 8] = [
    pwr_in("1"),  // VIN
    pwr_out("2"), // OUT — the switching node into L1/L2
    dig_in("3"),  // FDB
    dig_in("4"),  // ~ON
    pwr_in("5"),  // GND
    pwr_in("6"),  // GND
    pwr_in("7"),  // GND
    pwr_in("8"),  // GND
];

/// `APM4953` — dual P-channel MOSFET (U3), the board's reverse-polarity pass
/// element. Only one half is wired, its two drain fingers joined.
#[rustfmt::skip]
pub const APM4953_PINS: [PinDecl; 4] = [
    passive("1"), // S1
    dig_in("2"),  // G1
    passive("7"), // D1
    passive("8"), // D1
];

/// `SW_Push` — the manual reset button (SW1), a two-terminal contact. Pressing
/// it is a scenario `pin_short`, not a component behavior.
pub const SW_PUSH_PINS: [PinDecl; 2] = [passive("1"), passive("2")];

/// `2N3904` — NPN transistor (Q1), the open-collector sink half of the servo
/// enable's `TTL-SINK` option.
pub const NPN_PINS: [PinDecl; 3] = [passive("1"), passive("2"), passive("3")];

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
    registry.register("ISO6731DWR", |_decl| {
        Box::new(SerialIsolator::new(LOGIC_RAIL_VOLTS))
    });

    // Topology-only stubs.
    register_stub(&mut registry, "ISO6742DWR", &ISO6742_PINS);
    register_stub(&mut registry, "ISO6741DWR", &ISO6741_PINS);
    register_stub(&mut registry, "ISO6740FDWR", &ISO6740_PINS);
    register_stub(&mut registry, "ISO6721BDR", &ISO6721_PINS);
    register_stub(&mut registry, "UCC12040DVER", &UCC12040_PINS);
    register_stub(&mut registry, "NSI50010YT1G_1", &NSI50010_PINS);
    register_stub(&mut registry, "6N137", &OPTO_6N137_PINS);
    register_stub(&mut registry, "VO2631", &VO2631_PINS);
    register_stub(&mut registry, "SN74LVC1G14DBV", &SN74LVC1G14_PINS);
    register_stub(&mut registry, "XL1509", &XL1509_PINS);
    register_stub(&mut registry, "APM4953", &APM4953_PINS);
    register_stub(&mut registry, "SW_Push", &SW_PUSH_PINS);
    register_stub(&mut registry, "2N3904", &NPN_PINS);
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
/// pins — the rig a bring-up bench actually builds.
///
/// Why straps at all when the board has regulators: a [`PinKind::PowerOut`]
/// pin registers its net as sourced at an *unmodeled* voltage, which clears
/// [`embsim_board::Finding::PowerNetUnsourced`] but carries no level, so a
/// component that gates on a rail voltage (every modeled part here does) would
/// read nothing. A numeric source on the rail is what turns the power topology
/// into a voltage. Regulator models are a later slice; until then the straps
/// say out loud what voltage each domain is at.
///
/// | Endpoint | Net | Volts | Domain |
/// |---|---|---|---|
/// | `J2.1` | `Net-(J2-Pin_1)` | 12.0 | main input, ahead of the polarity FET |
/// | `J2.2` | `GND` | 0.0 | primary ground |
/// | `J19.1` | `+3.3V` | 3.3 | P2 I/O logic |
/// | `J22.1` | `+5V` | 5.0 | pre-regulator 5 V |
/// | `J21.1` | `SC_5V` | 5.0 | isolated servo domain |
/// | `J21.8` | `EN_GND` | 0.0 | isolated servo/encoder ground |
pub fn bench_rails(edge: &str) -> Harness {
    Harness::new()
        .power(ep("BENCH.12V"), ep(&format!("{edge}.J2.1")), 12.0)
        .power(ep("BENCH.GND"), ep(&format!("{edge}.J2.2")), 0.0)
        .power(
            ep("BENCH.3V3"),
            ep(&format!("{edge}.J19.1")),
            LOGIC_RAIL_VOLTS,
        )
        .power(ep("BENCH.5V"), ep(&format!("{edge}.J22.1")), 5.0)
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

/// The module's reverse-polarity pass FET (`U401`), conducting.
///
/// A P-channel FET in a polarity-protection position is a *switch*, and the
/// fault algebra's `pin_short` — "union these two pins' nets" — is exactly the
/// primitive for a closed switch. Without it the module's whole power tree is
/// unreachable from the 5 V edge fingers, which is also the correct answer for
/// a board whose protection FET is off.
pub fn module_polarity_fet_conducting(scenario: Scenario, module: &str) -> Scenario {
    scenario.pin_short(&format!("{module}.U401.S"), &format!("{module}.U401.D"))
}

/// The EdgeBoard's reverse-polarity pass FET (`U3`, an APM4953 half),
/// conducting — the same primitive as [`module_polarity_fet_conducting`],
/// between `S1` (pin 1, on `V_IN`) and one of the joined drain fingers (pin 7,
/// on the J2 input net).
pub fn edge_polarity_fet_conducting(scenario: Scenario, edge: &str) -> Scenario {
    scenario.pin_short(&format!("{edge}.U3.1"), &format!("{edge}.U3.7"))
}

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

/// Isolated force-domain straps, applied on the DS2 side of the cable (the
/// add-on's `J1.1`/`J1.2`) so the whole isolated rail — the add-on's `+3V3`
/// and the EdgeBoard's `IFG_5V`, one net once the cable is in — carries the
/// 3.3 V the ADC needs. Mirrors the DS2 bench rig in
/// `board/tests/ds2_regressions.rs`.
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
