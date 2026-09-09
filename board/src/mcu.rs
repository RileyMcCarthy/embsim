//! McuComponent — the MCU as a [`Component`] (`BOARD_ENGINE.md`, "The MCU as
//! a component"): HAL-table-shaped configs in, physical pins out, bridges into
//! the `embsim-peripherals` banks, and — given a firmware entry — its own
//! execution.
//!
//! # Bridged channels
//!
//! Every channel is **opt-in per channel** through [`McuBuilder`], so a
//! consumer that bridges nothing keeps today's behavior exactly:
//!
//! | Channel | Builder | Pins | Direction |
//! |---|---|---|---|
//! | serial | [`McuBuilder::bridge_serial`] | TX + RX, stream roles (or plain digital, with [`McuBuilder::serial_on_levels`]) | both |
//! | GPIO | [`McuBuilder::bridge_gpio`] | one, per declared direction | firmware → net **and** net → firmware |
//! | pulse-out | [`McuBuilder::bridge_pulse_out`] | one STEP pin, [`StreamRole::PulseSource`] | firmware → net |
//! | encoder | [`McuBuilder::bridge_encoder`] | A + B phase pins | net → firmware |
//!
//! Together these close the "hand-wired motion" seam: a step train reaches a
//! motor component's pin, an enable/direction GPIO reaches it as a real drive
//! (and an endstop reaches the firmware's GPIO bank), and the counts an
//! encoder component produces land in the firmware's encoder bank instead of a
//! consumer-written value.
//!
//! ## The step train is a rate, not edges
//!
//! At the reference machine's 8192 steps/mm, one mm/s of carriage speed is
//! 8192 STEP edges/s; a realistic traverse is hundreds of thousands. Driving
//! those as pin transitions would put one drive command, one cluster
//! resolution and one sense delivery through the single-writer engine *per
//! step*. So the pulse-out bridge does not synthesize edges at all: it
//! forwards the peripheral's
//! [`on_rate_change`](embsim_peripherals::pulse_out::PulseOut::on_rate_change)
//! events onto a [`StreamRole::PulseSource`] pin as [`PulseTrain`] segments —
//! **one engine event per rate change** — and the consumer integrates at read
//! time. Exact step counts survive: [`PulseTrain::emitted_at`] is the same
//! integer arithmetic `HAL_pulseOut_run` hands the firmware, so an encoder fed
//! from the train cannot drift from the firmware's own view. The fidelity this
//! trades away (no edges, no pulse width, no per-edge DIR sampling) is
//! enumerated on [`PulseTrain`].
//!
//! A pulse channel may name a bridged GPIO **output** channel as its direction
//! source ([`McuBuilder::bridge_pulse_out_with_direction`], which also takes
//! the direction that channel's *active* state means). A change of that GPIO
//! re-publishes the train, re-based at the change instant, so the pulses
//! before and after keep their own signs and the running count stays exact.
//! Without a direction source every segment is [`PulseDirection::Forward`] and
//! the sink takes direction from its own DIR pin — which is what a real
//! step/direction drive does, and the wiring to prefer.
//!
//! # Two modes
//!
//! - **Owned-execution mode** ([`McuBuilder::entry`] given): the component
//!   creates its own [`PeripheralInstance`] at build; [`McuComponent::attach`]
//!   installs the channel FDs there; [`Component::start`] — which
//!   [`crate::System::start`] calls only after *every* component has
//!   attached — spawns the entry on a thread bound to that instance, so all
//!   HAL free functions the firmware calls (and threads it spawns through
//!   the HAL, via `system::start_thread` inheritance) route to this MCU's
//!   peripherals. This is the "engine spawns the firmware entry" inversion
//!   from `BOARD_ENGINE.md` point 1.
//! - **Facade mode** (no entry): [`McuComponent::attach`] installs the FDs
//!   into the *calling thread's* instance — the default one when the
//!   consumer boots firmware through `embsim_runtime::Emulator::run` on the
//!   main thread, which keeps existing boot flows working unchanged.
//!
//! Limits: one firmware *image* links once per process (its C statics are
//! process-global even though its HAL peripherals are not), so two
//! owned-execution MCUs must run distinct images. Channels the consumer does
//! not bridge stay hand-wired — reach them through
//! [`McuComponent::instance`].
//!
//! # Shutdown
//!
//! A firmware entry typically never returns, so its thread is **detached**:
//! dropping the [`crate::SystemHandle`] joins the engine and the serial
//! pumps and disconnects the channel FDs (the firmware thread then sees
//! "not connected", never a closed descriptor), but does not attempt to
//! join the entry thread — process teardown reclaims it. The entry thread's
//! binding guard holds its own `Arc` to the instance, so peripherals stay
//! valid for the thread's whole life regardless of drop order.
//!
//! # Config structs are deliberate duplicates
//!
//! [`SerialChannelConfig`] / [`GpioChannelConfig`] /
//! [`PulseOutChannelConfig`] / [`EncoderChannelConfig`] mirror the structs
//! `embsim-memory-inspect`'s `hal_tables` module decodes from a firmware
//! archive. They are duplicated here **on purpose**: `board` must not depend
//! on `memory-inspect` (the tools crate is an optional read path, not an
//! engine dependency), and `memory-inspect` must stay board-agnostic. The
//! consumer maps one struct into the other field-by-field — a three-line
//! cost that keeps the dependency graph acyclic and both crates standalone.
//!
//! # Pin naming
//!
//! Every referenced physical pin is declared as `"P{n}"` (`"P0"`..`"P63"`),
//! matching the bench-rig endpoint convention (`P2EVAL.P0`). Netlists that
//! place an `McuComponent` must reference its pins by these names
//! (`(node (ref "U1") (pin "P2"))`).
//!
//! # Serial bridge mechanics
//!
//! Per bridged channel, [`McuComponent::attach`] creates a non-blocking
//! `socketpair` (the same pattern as `embsim-models`' ADS122U04 pipe pair —
//! the firmware HAL's receive-timeout semantics depend on `EAGAIN`):
//!
//! ```text
//!  firmware HAL serial ──fd──┐                      ┌── net engine ──┐
//!    transmit_data ──────────┤ socketpair ├─ pump ──► StreamTx "P{tx}"
//!    receive_*     ◄─────────┤            ├◄─ on_byte("P{rx}") ◄─────┘
//! ```
//!
//! - **MCU → net**: a small named thread (`"mcu-{name}-ch{n}"`) polls the
//!   component-side FD and `StreamTx::write`s whatever the firmware
//!   transmitted out the TX pin. A dedicated thread (rather than an engine
//!   `schedule_every` poll) keeps FD I/O off the net-engine thread — nothing
//!   on a net-resolution path may block — and matches the models crate's
//!   existing `protocol_loop` reader-thread pattern.
//! - **Net → MCU**: bytes delivered to the RX pin's `on_byte` (engine
//!   thread) are written non-blockingly to the component-side FD; the
//!   firmware reads them from its end. A full pipe drops the byte with a
//!   trace — the engine thread never blocks on a slow firmware.
//! - **Baud comes from the table**: the TX/RX pins declare
//!   `Producer`/`Consumer { baud_hz }` from [`SerialChannelConfig::baud`],
//!   so the net engine paces the wire from the firmware's own config — the
//!   emulator invents no default. The peripheral bank's own `set_baud`
//!   pacing is deliberately left untouched (unpaced unless the consumer
//!   overrides it, e.g. MaD's `MAD_SIM_BAUD` test override): the wire is
//!   paced in exactly one place.
//!
//! ## Or as levels, which is the point of the net
//!
//! [`McuBuilder::serial_on_levels`] replaces that byte route with edges. The
//! TX/RX pins become plain [`PinKind::DigitalOut`] / [`PinKind::DigitalIn`]
//! with no [`StreamRole`] at all, and a byte becomes a start bit, eight data
//! bits and a stop bit at the table baud — driven by
//! `serial_levels::SerialLevelBridge` out of the component's wake handler, and
//! decoded on the way back from the RX pin's resolved state.
//!
//! It matters because the byte route lets the net decide *reachability* and
//! nothing else: a byte crossing it cannot be corrupted by a driver fighting
//! it, cannot notice the line was floating, and cannot break. On levels it can,
//! and `board/tests/serial_levels.rs` shows exactly that — the same byte, the
//! same code, one wire with a second driver on it, and only the contended one
//! fails.
//!
//! The two encodings do not interoperate (a `Producer` has nothing to say to a
//! `DigitalIn`), which is why this is a flag rather than the only path.
//! - **Shutdown**: dropping the component flags every pump, joins its
//!   thread (bounded by the poll timeout), disconnects the channel from the
//!   peripheral bank, and closes both FDs — no detached-thread leak.
//!   [`crate::system::SystemHandle`] drops the engine before its components,
//!   so no `on_byte` delivery can race the FD close.
//!
//! # Ordering with today's boot flow
//!
//! The peripheral serial bank must be sized (`serial::init(count)`) before
//! the bridged channels carry traffic — in today's `Emulator::run` that
//! happens before project wiring, so consumers should `System::start` from
//! their wiring step (or any point after peripheral init), exactly where the
//! hand-wired `init_channel_fd` calls live now.

use std::os::fd::{BorrowedFd, RawFd};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, Weak};
use std::thread::JoinHandle;

use embsim_core::virtual_clock;
use embsim_peripherals::instance::PeripheralInstance;
use embsim_peripherals::pulse_out::PulseSegment;

use crate::component::{
    AttachError, Component, ComponentNetIo, PinDecl, PinKind, PulseDirection, PulseTrain, PulseTx,
    StreamRole,
};
use crate::net::{Level, NetState, TheveninDrive, Volts, DEFAULT_PUSH_PULL_IMPEDANCE};
use crate::serial_levels::SerialLevelBridge;
use crate::uart::{FramingError, UartFraming};

// ============================================================
// Config structs (duplicated from memory-inspect on purpose)
// ============================================================

/// One serial channel's wiring: physical RX/TX pins and configured baud.
/// Mirrors `embsim_memory_inspect::hal_tables::SerialChannelConfig` (see the
/// module docs for why the duplication is deliberate).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SerialChannelConfig {
    /// Physical RX pin index (0..=63).
    pub rx_pin: u32,
    /// Physical TX pin index (0..=63).
    pub tx_pin: u32,
    /// Configured baud rate in bits per second — the net engine paces the
    /// derived byte route at this rate.
    pub baud: u32,
}

/// One GPIO channel's wiring. Mirrors
/// `embsim_memory_inspect::hal_tables::GpioChannelConfig`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GpioChannelConfig {
    /// Physical pin index (0..=63).
    pub pin: u32,
    /// `true` when the channel's active state drives the pin low — the
    /// open-collector / active-low convention. Honored in **both** bridge
    /// directions: an active output drives the pin low, and a low pin reads
    /// back as active.
    pub active_low: bool,
}

/// One pulse-output channel's wiring: the STEP pin. Mirrors
/// `embsim_memory_inspect::hal_tables::PulseOutConfig`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PulseOutChannelConfig {
    /// Physical pin index (0..=63) the step clock leaves on.
    pub pin: u32,
}

/// One quadrature-encoder channel's wiring: the A/B phase pins. Mirrors
/// `embsim_memory_inspect::hal_tables::EncoderConfig`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EncoderChannelConfig {
    /// Physical A-phase pin index (0..=63).
    pub pin_a: u32,
    /// Physical B-phase pin index (0..=63).
    pub pin_b: u32,
}

/// Which bridged GPIO output channel stamps a pulse train's direction, and
/// which way the train counts while that channel is active. Built by
/// [`McuBuilder::bridge_pulse_out_with_direction`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct DirectionSource {
    /// HAL GPIO channel index supplying the direction.
    channel: usize,
    /// Direction the train carries while that channel reads active.
    active_direction: PulseDirection,
}

/// Electrical direction of a declared GPIO channel pin (the HAL tables do
/// not encode direction, so the builder takes it per channel).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GpioDirection {
    /// The MCU senses this pin ([`PinKind::DigitalIn`]): the net drives, and a
    /// bridged channel writes what it reads into the peripheral GPIO bank —
    /// what an endstop needs.
    Input,
    /// The MCU drives this pin ([`PinKind::DigitalOut`]): a bridged channel
    /// turns every firmware write into a net drive.
    Output,
}

// ============================================================
// Electrical defaults for bridged pins
// ============================================================

/// Open-circuit voltage a bridged GPIO output drives for a logic high.
/// Matches the engine's own idle-high default; a consumer that needs another
/// rail models the level shifter as a component rather than retuning this.
const MCU_OUTPUT_HIGH_VOLTS: Volts = 3.3;

/// Logic threshold applied to an [`NetState::Analog`] net when a bridged input
/// (GPIO or encoder phase) projects it to a level. Matches the engine's own
/// digital projection threshold.
pub(crate) const MCU_INPUT_THRESHOLD_VOLTS: Volts = 1.5;

/// Project a resolved net state to a logic level, or `None` when the engine
/// refuses to give one (floating / contended). The engine never invents a
/// level, and neither does the bridge: an input with no level holds the last
/// value the firmware saw rather than inventing a released state.
pub(crate) fn level_of(state: NetState) -> Option<Level> {
    match state {
        NetState::Driven(level) | NetState::Pulled(level, _) => Some(level),
        NetState::Analog(volts) => Some(if volts >= MCU_INPUT_THRESHOLD_VOLTS {
            Level::High
        } else {
            Level::Low
        }),
        NetState::Floating | NetState::Contention => None,
    }
}

/// The Thevenin contribution of a bridged output at `level`.
pub(crate) fn output_drive(level: Level) -> TheveninDrive {
    TheveninDrive {
        volts: match level {
            Level::High => MCU_OUTPUT_HIGH_VOLTS,
            Level::Low => 0.0,
        },
        impedance: DEFAULT_PUSH_PULL_IMPEDANCE,
    }
}

/// The pin level a GPIO channel's `active` state drives, honoring
/// `active_low`.
fn level_of_active(active: bool, active_low: bool) -> Level {
    if active != active_low {
        Level::High
    } else {
        Level::Low
    }
}

/// The `active` state a pin level reads back as, honoring `active_low`.
fn active_of_level(level: Level, active_low: bool) -> bool {
    (level == Level::High) != active_low
}

/// The direction a pulse train carries given its direction GPIO's `active`
/// state and the machine's declared mapping for the active state.
fn direction_of_active(active: bool, active_direction: PulseDirection) -> PulseDirection {
    if active {
        active_direction
    } else {
        match active_direction {
            PulseDirection::Forward => PulseDirection::Reverse,
            PulseDirection::Reverse => PulseDirection::Forward,
        }
    }
}

// ============================================================
// Pin names ("P0".."P63")
// ============================================================

/// The 64 physical pin names. `PinDecl` requires `&'static str`, so the full
/// set is spelled out once.
#[rustfmt::skip]
const PIN_NAMES: [&str; 64] = [
    "P0",  "P1",  "P2",  "P3",  "P4",  "P5",  "P6",  "P7",
    "P8",  "P9",  "P10", "P11", "P12", "P13", "P14", "P15",
    "P16", "P17", "P18", "P19", "P20", "P21", "P22", "P23",
    "P24", "P25", "P26", "P27", "P28", "P29", "P30", "P31",
    "P32", "P33", "P34", "P35", "P36", "P37", "P38", "P39",
    "P40", "P41", "P42", "P43", "P44", "P45", "P46", "P47",
    "P48", "P49", "P50", "P51", "P52", "P53", "P54", "P55",
    "P56", "P57", "P58", "P59", "P60", "P61", "P62", "P63",
];

/// The `"P{n}"` name of a physical pin, or `None` past the P63 ceiling.
fn pin_name(pin: u32) -> Option<&'static str> {
    PIN_NAMES.get(pin as usize).copied()
}

// ============================================================
// Builder
// ============================================================

/// Builder for [`McuComponent`]: the serial table (as read from the
/// firmware's HAL config tables), which channels to bridge, and any GPIO
/// channels to declare.
#[derive(Default)]
pub struct McuBuilder {
    name: String,
    serial_table: Vec<SerialChannelConfig>,
    bridged_serial: Vec<usize>,
    serial_on_levels: bool,
    gpio: Vec<(GpioChannelConfig, GpioDirection)>,
    gpio_table: Vec<GpioChannelConfig>,
    bridged_gpio: Vec<(usize, GpioDirection)>,
    pulse_out_table: Vec<PulseOutChannelConfig>,
    /// `(pulse channel, optional direction source)`.
    bridged_pulse_out: Vec<(usize, Option<DirectionSource>)>,
    encoder_table: Vec<EncoderChannelConfig>,
    bridged_encoder: Vec<usize>,
    entry: Option<Box<dyn FnOnce() + Send>>,
}

impl std::fmt::Debug for McuBuilder {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("McuBuilder")
            .field("name", &self.name)
            .field("serial_table", &self.serial_table)
            .field("bridged_serial", &self.bridged_serial)
            .field("serial_on_levels", &self.serial_on_levels)
            .field("gpio", &self.gpio)
            .field("gpio_table", &self.gpio_table)
            .field("bridged_gpio", &self.bridged_gpio)
            .field("pulse_out_table", &self.pulse_out_table)
            .field("bridged_pulse_out", &self.bridged_pulse_out)
            .field("encoder_table", &self.encoder_table)
            .field("bridged_encoder", &self.bridged_encoder)
            .field("entry", &self.entry.as_ref().map(|_| "FnOnce"))
            .finish()
    }
}

impl McuBuilder {
    /// Start building an MCU named `name` (used for pump-thread names and
    /// diagnostics).
    pub fn new(name: &str) -> Self {
        Self {
            name: name.to_string(),
            ..Self::default()
        }
    }

    /// Provide the serial wiring table, indexed by HAL channel number —
    /// typically decoded from the firmware archive via
    /// `embsim_memory_inspect::hal_tables::read_serial_table` and mapped
    /// into this crate's [`SerialChannelConfig`].
    pub fn serial_table(mut self, table: Vec<SerialChannelConfig>) -> Self {
        self.serial_table = table;
        self
    }

    /// Bridge one serial channel: declare its TX/RX pins (with stream roles
    /// at the table baud) and pump its bytes to/from the peripheral serial
    /// bank at attach. Channels not bridged are not declared at all.
    pub fn bridge_serial(mut self, channel: usize) -> Self {
        self.bridged_serial.push(channel);
        self
    }

    /// Carry every bridged serial channel as **levels** rather than as bytes.
    ///
    /// The default routes a firmware TX byte to a
    /// [`StreamRole::Producer`] pin, which delivers it to a reachable
    /// consumer as a byte: the net decides *who is connected*, but the payload
    /// never becomes a level, so it cannot be corrupted by a fighting driver
    /// and cannot notice the line was floating. With this set, the TX and RX
    /// pins are plain [`PinKind::DigitalOut`] / [`PinKind::DigitalIn`], the
    /// byte becomes a start bit, eight data bits and a stop bit at the table
    /// baud, and the peer decodes the edges back.
    ///
    /// Two consequences worth expecting:
    ///
    /// - **A byte now takes a byte's worth of virtual time to arrive.** It did
    ///   before too (the byte route paces at the same baud), but the *first*
    ///   edge is now visible a bit period before the byte lands.
    /// - **The peer must also speak levels.** A [`StreamRole::Producer`] on
    ///   the other end of the wire has nothing to say to a `DigitalIn`; the
    ///   two encodings do not interoperate, which is why this is a flag and
    ///   not yet the only path.
    pub fn serial_on_levels(mut self) -> Self {
        self.serial_on_levels = true;
        self
    }

    /// Declare one GPIO channel's pin with the given direction, **without**
    /// bridging it: the pin appears in the facade (so a netlist may reference
    /// it and the engine resolves it) but nothing is wired to the peripheral
    /// GPIO bank. Use [`McuBuilder::gpio_table`] +
    /// [`McuBuilder::bridge_gpio`] for a live channel.
    pub fn gpio(mut self, config: GpioChannelConfig, direction: GpioDirection) -> Self {
        self.gpio.push((config, direction));
        self
    }

    /// Provide the GPIO wiring table, indexed by HAL channel number —
    /// typically decoded from the firmware archive via
    /// `embsim_memory_inspect::hal_tables::read_gpio_table`.
    pub fn gpio_table(mut self, table: Vec<GpioChannelConfig>) -> Self {
        self.gpio_table = table;
        self
    }

    /// Bridge one GPIO channel to its physical pin, in the given direction.
    ///
    /// - [`GpioDirection::Output`]: every firmware write
    ///   (`gpio::set_active` / `toggle_active`) becomes a push-pull drive on
    ///   the pin, at the channel's `active_low` polarity. The channel's
    ///   power-on state is driven at attach, so the net is never left floating
    ///   by an MCU that has not written yet.
    /// - [`GpioDirection::Input`]: whatever the net resolves to is written
    ///   into the peripheral bank (as an *external* write, so it never
    ///   re-enters the firmware's own change callback), at the same polarity —
    ///   this is what makes an endstop component visible to firmware. A net
    ///   with no logic level (floating, contended) holds the last value and
    ///   traces; the engine never invents a level, so neither does the bridge.
    ///
    /// A bridged output channel takes over the peripheral bank's single
    /// `on_change` slot for that channel — that is the point of opting in, and
    /// a consumer's own hand-wired callback on the same channel would be
    /// replaced.
    pub fn bridge_gpio(mut self, channel: usize, direction: GpioDirection) -> Self {
        self.bridged_gpio.push((channel, direction));
        self
    }

    /// Provide the pulse-output wiring table, indexed by HAL channel number —
    /// typically decoded from the firmware archive via
    /// `embsim_memory_inspect::hal_tables::read_pulse_out_table`.
    pub fn pulse_out_table(mut self, table: Vec<PulseOutChannelConfig>) -> Self {
        self.pulse_out_table = table;
        self
    }

    /// Bridge one pulse-output channel to its STEP pin as a rate-carried
    /// [`PulseTrain`] (see the module docs for why a rate and not edges).
    ///
    /// The pin is declared [`PinKind::DigitalOut`] with
    /// [`StreamRole::PulseSource`], so it also holds a resolvable idle level;
    /// the *train* rides the derived pulse route to every reachable
    /// [`StreamRole::PulseSink`]. Every segment is
    /// [`PulseDirection::Forward`] — use
    /// [`McuBuilder::bridge_pulse_out_with_direction`] when the sink has no
    /// DIR pin of its own.
    pub fn bridge_pulse_out(mut self, channel: usize) -> Self {
        self.bridged_pulse_out.push((channel, None));
        self
    }

    /// Bridge a pulse-output channel and stamp each segment's direction from a
    /// bridged GPIO **output** channel's active state.
    ///
    /// `active_direction` says which way the train counts while that channel
    /// is *active* (the channel's own `active_low` polarity already decides
    /// what "active" means electrically); the inactive state carries the
    /// opposite. It is an explicit argument because the mapping is a machine
    /// convention, not a fact about the silicon — a default here would
    /// silently invert an axis on half the machines that use it.
    ///
    /// A change on `direction_gpio_channel` re-publishes the train re-based at
    /// the change instant, so the pulses on each side of the reversal keep
    /// their own sign and the running count stays exact. The direction channel
    /// must itself be bridged as [`GpioDirection::Output`] — otherwise the
    /// build fails with [`McuBuildError::DirectionChannelNotBridged`], because
    /// the bridge hooks the same `on_change` slot rather than silently
    /// installing a second, conflicting callback.
    ///
    /// Reach for this only when the sink has no DIR pin of its own; a
    /// step/direction drive wired the real way takes its direction from its
    /// own DIR input and needs nothing here.
    pub fn bridge_pulse_out_with_direction(
        mut self,
        channel: usize,
        direction_gpio_channel: usize,
        active_direction: PulseDirection,
    ) -> Self {
        self.bridged_pulse_out.push((
            channel,
            Some(DirectionSource {
                channel: direction_gpio_channel,
                active_direction,
            }),
        ));
        self
    }

    /// Provide the encoder wiring table, indexed by HAL channel number —
    /// typically decoded from the firmware archive via
    /// `embsim_memory_inspect::hal_tables::read_encoder_table`.
    pub fn encoder_table(mut self, table: Vec<EncoderChannelConfig>) -> Self {
        self.encoder_table = table;
        self
    }

    /// Bridge one encoder channel: declare its A/B phase pins as inputs and
    /// ×4-decode the quadrature they carry into the peripheral encoder bank,
    /// so firmware reads counts produced by a real encoder component.
    ///
    /// Decoding is the standard Gray-code walk — `(A,B)` cycling
    /// `(0,0) → (1,0) → (1,1) → (0,1)` counts up, the reverse counts down.
    /// A phase pair that jumps two states (both channels changing between
    /// deliveries — a snapped encoder position, or an unwired phase) is a
    /// missed transition: it is traced and **not** counted, so a defect shows
    /// up as a count that stops tracking rather than one that walks the wrong
    /// way.
    ///
    /// Each counted transition **increments the bank's existing value** rather
    /// than writing an absolute position, which is what real quadrature
    /// hardware does: the firmware owns the counter register, so a homing
    /// `HAL_encoder_set` re-bases the count and subsequent motion continues
    /// from there.
    ///
    /// Which is also why **the count has no absolute meaning until firmware
    /// homes it.** The phase pair the bridge sees at attach is only a seed,
    /// and the encoder component on the other side of the harness will drive
    /// its own power-on phase shortly afterwards; the transitions between the
    /// two are real and are counted. That boot offset is the physically
    /// correct answer for a quadrature counter — a datum comes from a homing
    /// move, not from the bridge inventing one.
    pub fn bridge_encoder(mut self, channel: usize) -> Self {
        self.bridged_encoder.push(channel);
        self
    }

    /// Give the MCU its firmware entry (typically a closure calling the
    /// consumer's `extern "C"` entry, e.g. `mad_begin()`), switching the
    /// component into **owned-execution mode**: it creates its own
    /// [`PeripheralInstance`], `attach` installs the channel FDs there
    /// instead of the calling thread's instance, and
    /// [`Component::start`] spawns `entry` on a thread bound to it — so
    /// every HAL free function the firmware calls (and every thread it
    /// spawns through the HAL) routes to this component's peripherals.
    ///
    /// The entry typically never returns; its thread is detached (see the
    /// module docs' shutdown notes). One firmware *image* still links once
    /// per process (its C statics are process-global) — two entry-mode
    /// components must run distinct images.
    ///
    /// Without an entry the component stays in **facade mode**: today's
    /// boot flow (`embsim_runtime::Emulator::run` executing the entry on
    /// the caller's thread against the default instance) works unchanged.
    pub fn entry(mut self, entry: impl FnOnce() + Send + 'static) -> Self {
        self.entry = Some(Box::new(entry));
        self
    }

    /// Validate the configuration and build the component.
    ///
    /// Fails when a bridged channel is missing from the serial table, a
    /// referenced pin is past P63, or two declarations claim the same
    /// physical pin.
    pub fn build(self) -> Result<McuComponent, McuBuildError> {
        let mut pins: Vec<PinDecl> = Vec::new();
        let mut claimed: Vec<u32> = Vec::new();
        let mut claim = |pin: u32| -> Result<&'static str, McuBuildError> {
            let name = pin_name(pin).ok_or(McuBuildError::PinOutOfRange { pin })?;
            if claimed.contains(&pin) {
                return Err(McuBuildError::DuplicatePin { pin });
            }
            claimed.push(pin);
            Ok(name)
        };

        let mut bridges: Vec<SerialBridge> = Vec::new();
        for &channel in &self.bridged_serial {
            let config =
                *self
                    .serial_table
                    .get(channel)
                    .ok_or(McuBuildError::UnknownSerialChannel {
                        channel,
                        table_len: self.serial_table.len(),
                    })?;
            // UART TX transmits onto the net; RX takes what reaches it. On
            // levels both are plain digital pins with no stream role at all —
            // the framing lives in the component, not in the route.
            let (tx_stream, rx_stream) = if self.serial_on_levels {
                (None, None)
            } else {
                (
                    Some(StreamRole::Producer {
                        baud_hz: config.baud,
                    }),
                    Some(StreamRole::Consumer {
                        baud_hz: config.baud,
                    }),
                )
            };
            pins.push(PinDecl {
                number: claim(config.tx_pin)?,
                name: None,
                kind: PinKind::DigitalOut,
                stream: tx_stream,
                drive_impedance: None,
            });
            pins.push(PinDecl {
                number: claim(config.rx_pin)?,
                name: None,
                kind: PinKind::DigitalIn,
                stream: rx_stream,
                drive_impedance: None,
            });
            bridges.push(SerialBridge { channel, config });
        }

        let gpio_kind = |direction: GpioDirection| match direction {
            GpioDirection::Input => PinKind::DigitalIn,
            GpioDirection::Output => PinKind::DigitalOut,
        };

        for (config, direction) in &self.gpio {
            pins.push(PinDecl {
                number: claim(config.pin)?,
                name: None,
                kind: gpio_kind(*direction),
                stream: None,
                drive_impedance: None,
            });
        }

        let mut gpio_bridges: Vec<GpioBridge> = Vec::new();
        for &(channel, direction) in &self.bridged_gpio {
            let config =
                *self
                    .gpio_table
                    .get(channel)
                    .ok_or(McuBuildError::UnknownGpioChannel {
                        channel,
                        table_len: self.gpio_table.len(),
                    })?;
            let pin = claim(config.pin)?;
            pins.push(PinDecl {
                number: pin,
                name: None,
                kind: gpio_kind(direction),
                stream: None,
                drive_impedance: None,
            });
            gpio_bridges.push(GpioBridge {
                channel,
                config,
                direction,
                pin,
            });
        }

        let mut pulse_bridges: Vec<PulseBridge> = Vec::new();
        for &(channel, direction_gpio) in &self.bridged_pulse_out {
            let config = *self.pulse_out_table.get(channel).ok_or(
                McuBuildError::UnknownPulseOutChannel {
                    channel,
                    table_len: self.pulse_out_table.len(),
                },
            )?;
            if let Some(source) = direction_gpio {
                let bridged_as_output = gpio_bridges
                    .iter()
                    .any(|b| b.channel == source.channel && b.direction == GpioDirection::Output);
                if !bridged_as_output {
                    return Err(McuBuildError::DirectionChannelNotBridged {
                        pulse_channel: channel,
                        gpio_channel: source.channel,
                    });
                }
            }
            let pin = claim(config.pin)?;
            // A step clock is a push-pull output that also holds a resolvable
            // idle level; the train itself rides the pulse route.
            pins.push(PinDecl {
                number: pin,
                name: None,
                kind: PinKind::DigitalOut,
                stream: Some(StreamRole::PulseSource),
                drive_impedance: None,
            });
            pulse_bridges.push(PulseBridge {
                channel,
                pin,
                direction_gpio,
            });
        }

        let mut encoder_bridges: Vec<EncoderBridge> = Vec::new();
        for &channel in &self.bridged_encoder {
            let config =
                *self
                    .encoder_table
                    .get(channel)
                    .ok_or(McuBuildError::UnknownEncoderChannel {
                        channel,
                        table_len: self.encoder_table.len(),
                    })?;
            let pin_a = claim(config.pin_a)?;
            let pin_b = claim(config.pin_b)?;
            for pin in [pin_a, pin_b] {
                pins.push(PinDecl {
                    number: pin,
                    name: None,
                    kind: PinKind::DigitalIn,
                    stream: None,
                    drive_impedance: None,
                });
            }
            encoder_bridges.push(EncoderBridge {
                channel,
                pin_a,
                pin_b,
            });
        }

        // Owned-execution mode: the component gets its own peripheral
        // instance up front so `attach` has a stable target before `start`
        // spawns the entry.
        let own_instance = self
            .entry
            .is_some()
            .then(|| Arc::new(PeripheralInstance::new()));

        Ok(McuComponent {
            name: self.name,
            pins,
            bridges,
            serial_on_levels: self.serial_on_levels,
            gpio_bridges,
            pulse_bridges,
            encoder_bridges,
            pumps: Vec::new(),
            shutdown: Arc::new(AtomicBool::new(false)),
            instance: None,
            own_instance,
            entry: Mutex::new(self.entry),
            entry_thread: None,
        })
    }
}

// ============================================================
// Component
// ============================================================

/// One bridged serial channel, prepared at build.
#[derive(Debug, Clone, Copy)]
struct SerialBridge {
    /// HAL serial channel index in the peripheral bank.
    channel: usize,
    /// The channel's wiring/baud from the firmware table.
    config: SerialChannelConfig,
}

/// Where a channel's firmware TX bytes go: the pin's byte route, or the level
/// bridge that frames them into edges.
type TxSink = Box<dyn Fn(&[u8]) + Send>;

/// One live level-carried serial channel: the framer, and where its decoded
/// bytes go.
struct LevelChannel {
    /// HAL serial channel index (diagnostics).
    channel: usize,
    /// Component side of the firmware's pipe pair.
    component_fd: RawFd,
    /// The framer driving TX bits and decoding RX edges.
    level: Arc<SerialLevelBridge>,
}

/// Hand decoded frames to the firmware's read side.
///
/// Runs on the engine thread, so the write must never block: a full pipe drops
/// the byte with a trace, exactly like a UART overrun on hardware. A frame
/// that did not decode is *not* delivered — a receiver hands its driver bytes,
/// not framing errors — but it is logged, because a stop-bit failure means the
/// line was contended or the baud rates disagree.
fn deliver_rx(
    component_fd: RawFd,
    channel: usize,
    frames: impl IntoIterator<Item = Result<u8, FramingError>>,
) {
    for frame in frames {
        let byte = match frame {
            Ok(byte) => byte,
            Err(error) => {
                tracing::debug!(channel, ?error, "RX frame dropped: bad framing on the wire");
                continue;
            }
        };
        // SAFETY: `component_fd` stays open until the owning component drops,
        // which happens only after the engine (and with it this callback) has
        // shut down — see `SystemHandle`'s documented drop order.
        let fd = unsafe { BorrowedFd::borrow_raw(component_fd) };
        if let Err(e) = nix::unistd::write(fd, &[byte]) {
            tracing::trace!(
                channel,
                error = %e,
                "RX byte dropped (firmware-side pipe not writable)"
            );
        }
    }
}

/// One bridged GPIO channel, prepared at build.
#[derive(Debug, Clone, Copy)]
struct GpioBridge {
    /// HAL GPIO channel index in the peripheral bank.
    channel: usize,
    /// The channel's pin and polarity from the firmware table.
    config: GpioChannelConfig,
    /// Which way this channel is wired.
    direction: GpioDirection,
    /// The `"P{n}"` pin name (validated at build).
    pin: &'static str,
}

/// One bridged pulse-output channel, prepared at build.
#[derive(Debug, Clone, Copy)]
struct PulseBridge {
    /// HAL pulse-out channel index in the peripheral bank.
    channel: usize,
    /// The `"P{n}"` STEP pin name (validated at build).
    pin: &'static str,
    /// Bridged GPIO output channel supplying the train's direction, if any.
    direction_gpio: Option<DirectionSource>,
}

/// One bridged encoder channel, prepared at build.
#[derive(Debug, Clone, Copy)]
struct EncoderBridge {
    /// HAL encoder channel index in the peripheral bank.
    channel: usize,
    /// The `"P{n}"` A-phase pin name (validated at build).
    pin_a: &'static str,
    /// The `"P{n}"` B-phase pin name (validated at build).
    pin_b: &'static str,
}

/// Shared state of one live pulse bridge: the pin's write half, the channel
/// it mirrors, and the direction currently stamped onto its segments.
///
/// Holds the peripheral instance **weakly**: the instance owns this bridge's
/// `on_rate_change` callback, so a strong handle here would be a reference
/// cycle that leaks the whole peripheral bank.
struct PulseBridgeState {
    tx: PulseTx,
    channel: usize,
    instance: Weak<PeripheralInstance>,
    /// `true` = [`PulseDirection::Reverse`]. Only ever written by the bridged
    /// direction GPIO's change hook; [`PulseDirection::Forward`] without one.
    reverse: AtomicBool,
    shutdown: Arc<AtomicBool>,
}

impl PulseBridgeState {
    /// The direction currently stamped onto published segments.
    fn direction(&self) -> PulseDirection {
        if self.reverse.load(Ordering::Relaxed) {
            PulseDirection::Reverse
        } else {
            PulseDirection::Forward
        }
    }

    /// Publish a segment onto the STEP pin with the current direction.
    fn publish(&self, segment: PulseSegment) {
        if self.shutdown.load(Ordering::Relaxed) {
            return;
        }
        self.tx.set_train(PulseTrain {
            pulses: segment,
            direction: self.direction(),
        });
    }

    /// Apply a new direction, re-publishing the channel's *current* segment
    /// re-based at this instant so pulses already emitted keep the old sign
    /// and the running count stays exact across a reversal.
    fn set_direction(&self, direction: PulseDirection) {
        let reverse = direction == PulseDirection::Reverse;
        if self.shutdown.load(Ordering::Relaxed)
            || self.reverse.swap(reverse, Ordering::Relaxed) == reverse
        {
            return;
        }
        let Some(instance) = self.instance.upgrade() else {
            return;
        };
        // No clock yet means no elapsed time to re-base against, so the
        // un-advanced segment is the right (and only) answer.
        let now = if virtual_clock::is_initialized() {
            virtual_clock::virtual_us()
        } else {
            0
        };
        self.publish(instance.pulse_out.segment(self.channel).rebased_at(now));
    }
}

/// ×4 quadrature decode state for one bridged encoder channel.
///
/// `(A, B)` levels are known independently — the engine delivers one sense per
/// net — so both must have arrived before any transition can be read.
#[derive(Debug, Default)]
struct QuadratureState {
    a: Option<bool>,
    b: Option<bool>,
    /// Last decoded Gray position (0..=3), once both phases were known.
    position: Option<u8>,
}

impl QuadratureState {
    /// Gray position of the current phase pair, if both are known:
    /// `(0,0) → 0`, `(1,0) → 1`, `(1,1) → 2`, `(0,1) → 3` — the order that
    /// counts up.
    fn gray_position(&self) -> Option<u8> {
        Some(match (self.a?, self.b?) {
            (false, false) => 0,
            (true, false) => 1,
            (true, true) => 2,
            (false, true) => 3,
        })
    }

    /// Read the current phase pair as a **count delta**, returning `Some(±1)`
    /// when the pair advanced one Gray state. `None` means "nothing to
    /// publish": a phase still unknown, no transition, or a first observation
    /// (which seeds the detector, exactly as `on_sense`'s
    /// deliver-at-registration is not a transition).
    ///
    /// A delta, not an absolute count, because the counter it feeds is the
    /// firmware's: real quadrature hardware *increments a register*, and
    /// firmware writes that register directly when it homes
    /// (`HAL_encoder_set`). Publishing an absolute count would silently undo
    /// every such write on the next edge.
    fn step(&mut self) -> Result<Option<i32>, QuadratureSlip> {
        let Some(position) = self.gray_position() else {
            return Ok(None);
        };
        let Some(previous) = self.position.replace(position) else {
            // First time both phases are known: seed, never count.
            return Ok(None);
        };
        match (position + 4 - previous) % 4 {
            0 => Ok(None),
            1 => Ok(Some(1)),
            3 => Ok(Some(-1)),
            // Both channels changed between deliveries: a real encoder cannot
            // do that, so the direction is unrecoverable.
            _ => Err(QuadratureSlip { previous, position }),
        }
    }
}

/// A two-state jump on a quadrature pair — a missed transition.
#[derive(Debug, Clone, Copy)]
struct QuadratureSlip {
    previous: u8,
    position: u8,
}

/// Live pump state for one bridged channel (exists after attach).
struct Pump {
    /// HAL serial channel index (for the disconnect on drop).
    channel: usize,
    /// Shutdown flag shared with the pump thread and the RX callback.
    shutdown: Arc<AtomicBool>,
    /// The pump thread; joined on drop.
    thread: Option<JoinHandle<()>>,
    /// Component-side FD: the pump reads firmware TX from it, the RX
    /// callback writes net bytes into it.
    component_fd: RawFd,
    /// Firmware-side FD, installed into the peripheral serial bank.
    firmware_fd: RawFd,
}

/// The MCU as a board component: its boundary is its physical pins; its
/// bridged serial channels connect the `embsim-peripherals` serial bank to
/// net-engine stream routes. Build one with [`McuBuilder`].
pub struct McuComponent {
    name: String,
    pins: Vec<PinDecl>,
    bridges: Vec<SerialBridge>,
    /// Carry serial as levels rather than as routed bytes
    /// ([`McuBuilder::serial_on_levels`]).
    serial_on_levels: bool,
    gpio_bridges: Vec<GpioBridge>,
    pulse_bridges: Vec<PulseBridge>,
    encoder_bridges: Vec<EncoderBridge>,
    pumps: Vec<Pump>,
    /// Set on drop. Every callback this component installs into a peripheral
    /// bank — GPIO change, pulse rate change — checks it first: those banks
    /// outlive the component in facade mode (they are the process default),
    /// and a stale callback must be inert rather than driving a dead engine.
    shutdown: Arc<AtomicBool>,
    /// The peripheral instance the channel FDs were installed into: the
    /// component's own instance in owned-execution mode, otherwise the
    /// attach thread's instance (the default one in today's boot flow).
    instance: Option<Arc<PeripheralInstance>>,
    /// Owned-execution mode only: the instance this MCU's firmware runs
    /// against, created at build so `attach` and `start` agree on it.
    own_instance: Option<Arc<PeripheralInstance>>,
    /// The firmware entry, consumed by [`Component::start`]. The `Mutex`
    /// exists only to keep the component `Sync` (a bare `FnOnce` box is
    /// not); it is accessed exclusively through `&mut self`.
    entry: Mutex<Option<Box<dyn FnOnce() + Send>>>,
    /// The spawned entry thread. Never joined — a firmware entry typically
    /// never returns; see the module docs' shutdown notes.
    entry_thread: Option<JoinHandle<()>>,
}

impl std::fmt::Debug for McuComponent {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("McuComponent")
            .field("name", &self.name)
            .field("pins", &self.pins)
            .field("bridges", &self.bridges)
            .field("serial_on_levels", &self.serial_on_levels)
            .field("gpio_bridges", &self.gpio_bridges)
            .field("pulse_bridges", &self.pulse_bridges)
            .field("encoder_bridges", &self.encoder_bridges)
            .finish()
    }
}

impl McuComponent {
    /// Start building an MCU component named `name`.
    pub fn builder(name: &str) -> McuBuilder {
        McuBuilder::new(name)
    }

    /// The component's name (thread naming, diagnostics).
    pub fn name(&self) -> &str {
        &self.name
    }

    /// The peripheral instance this MCU's firmware runs against — `Some`
    /// only in owned-execution mode (an entry was given). Consumers use it
    /// to reach peripherals the pin facade does not bridge yet (hand-wired
    /// GPIO/encoder/pulse callbacks during the migration window).
    pub fn instance(&self) -> Option<&Arc<PeripheralInstance>> {
        self.own_instance.as_ref()
    }

    /// Whether the spawned firmware entry thread is still running. `false`
    /// before [`Component::start`] and after an entry that returned.
    pub fn entry_running(&self) -> bool {
        self.entry_thread.as_ref().is_some_and(|t| !t.is_finished())
    }
}

impl Component for McuComponent {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        // Owned-execution mode targets the component's own instance; facade
        // mode targets the calling thread's instance (the process default in
        // today's Emulator::run boot flow).
        let instance = match &self.own_instance {
            Some(own) => Arc::clone(own),
            None => embsim_peripherals::instance::current(),
        };

        let mut level_channels: Vec<LevelChannel> = Vec::new();
        for bridge in &self.bridges {
            // Validated at build: both pins are <= P63.
            let tx_name = pin_name(bridge.config.tx_pin).expect("validated at build");
            let rx_name = pin_name(bridge.config.rx_pin).expect("validated at build");

            let (component_fd, firmware_fd) =
                create_pipe_pair().map_err(|detail| AttachError::Failed {
                    message: format!(
                        "mcu {:?} channel {}: cannot create serial pipe pair: {detail}",
                        self.name, bridge.channel
                    ),
                })?;
            // Record the pump immediately so Drop reclaims the FDs even if a
            // later step of this attach fails.
            let shutdown = Arc::new(AtomicBool::new(false));
            self.pumps.push(Pump {
                channel: bridge.channel,
                shutdown: Arc::clone(&shutdown),
                thread: None,
                component_fd,
                firmware_fd,
            });

            instance.serial.init_channel_fd(bridge.channel, firmware_fd);

            // Net → MCU. On the byte path the route hands over whole bytes; on
            // levels the pin's resolved state is fed to a framer and whatever
            // it decodes lands on the firmware's read side. Both run on the
            // engine thread, so the write must never block: a full pipe drops
            // the byte with a trace.
            let channel = bridge.channel;
            let tx_sink: TxSink = if self.serial_on_levels {
                let level = Arc::new(SerialLevelBridge::new(
                    UartFraming::new_8n1(bridge.config.baud),
                    io.pin(tx_name)?,
                    io.clone(),
                    Arc::clone(&shutdown),
                ));
                // Hold the line at idle before the firmware runs: a peer that
                // saw it floating would have no reference for the first start
                // bit's falling edge.
                level.idle();
                {
                    let level = Arc::clone(&level);
                    let shutdown = Arc::clone(&shutdown);
                    io.on_sense(rx_name, move |state| {
                        if shutdown.load(Ordering::Relaxed) {
                            return;
                        }
                        deliver_rx(component_fd, channel, level.receive_sense(state));
                    })?;
                }
                level_channels.push(LevelChannel {
                    channel,
                    component_fd,
                    level: Arc::clone(&level),
                });
                Box::new(move |bytes: &[u8]| {
                    let shed = level.transmit(bytes);
                    if shed > 0 {
                        tracing::trace!(channel, shed, "TX bytes shed: the line is behind");
                    }
                })
            } else {
                let tx = io.stream_tx(tx_name)?;
                {
                    let shutdown = Arc::clone(&shutdown);
                    io.on_byte(rx_name, move |byte| {
                        if shutdown.load(Ordering::Relaxed) {
                            return;
                        }
                        deliver_rx(component_fd, channel, [Ok(byte)]);
                    })?;
                }
                Box::new(move |bytes: &[u8]| tx.write(bytes))
            };

            // MCU → net: a named pump thread moves firmware TX bytes into
            // whichever sink this channel uses (see the module docs for why a
            // thread and not an engine poll).
            let thread = std::thread::Builder::new()
                .name(format!("mcu-{}-ch{}", self.name, bridge.channel))
                .spawn({
                    let shutdown = Arc::clone(&shutdown);
                    move || pump_loop(component_fd, &*tx_sink, &shutdown)
                })
                .map_err(|e| AttachError::Failed {
                    message: format!(
                        "mcu {:?} channel {}: cannot spawn pump thread: {e}",
                        self.name, bridge.channel
                    ),
                })?;
            self.pumps.last_mut().expect("pushed above").thread = Some(thread);

            tracing::debug!(
                mcu = %self.name,
                channel = bridge.channel,
                tx = tx_name,
                rx = rx_name,
                baud = bridge.config.baud,
                on_levels = self.serial_on_levels,
                "serial channel bridged"
            );
        }

        // One wake handler drives every level channel's bit clock and closes
        // any receive frame whose tail carried no transition. It is registered
        // once for the whole component (the engine keeps one per component),
        // so each channel re-arms the shared timer for its own next instant.
        if !level_channels.is_empty() {
            let channels = Arc::new(level_channels);
            let shutdown = Arc::clone(&self.shutdown);
            let scheduler = io.clone();
            io.on_wake_ns(move |now_ns| {
                if shutdown.load(Ordering::Relaxed) {
                    return;
                }
                for channel in channels.iter() {
                    let (bytes, next) = channel.level.service(now_ns);
                    deliver_rx(channel.component_fd, channel.channel, bytes);
                    if let Some(at) = next {
                        scheduler.schedule_at_ns(at);
                    }
                }
            });
        }

        // Pulse bridges first: a GPIO channel that supplies a train's
        // direction hooks the same `on_change` slot the GPIO bridge installs,
        // so its state must exist before that slot is written.
        let mut direction_of_gpio: Vec<(DirectionSource, Arc<PulseBridgeState>)> = Vec::new();
        for bridge in &self.pulse_bridges {
            let state = Arc::new(PulseBridgeState {
                tx: io.pulse_tx(bridge.pin)?,
                channel: bridge.channel,
                instance: Arc::downgrade(&instance),
                reverse: AtomicBool::new(false),
                shutdown: Arc::clone(&self.shutdown),
            });
            // Establish the channel on the net before the firmware runs, so a
            // sink attaching later has a baseline to fold against. The bank's
            // own segment rather than a synthetic idle — normally identical to
            // [`PulseSegment::IDLE`], but correct even if a channel is already
            // running when this component attaches. Reads state, never the
            // clock, so it is safe before `virtual_clock::init`.
            state.publish(instance.pulse_out.segment(bridge.channel));
            {
                let state = Arc::clone(&state);
                instance
                    .pulse_out
                    .on_rate_change(bridge.channel, move |segment| state.publish(segment));
            }
            if let Some(source) = bridge.direction_gpio {
                direction_of_gpio.push((source, Arc::clone(&state)));
            }
            tracing::debug!(
                mcu = %self.name,
                channel = bridge.channel,
                step = bridge.pin,
                direction_gpio = ?bridge.direction_gpio,
                "pulse-out channel bridged to a step pin"
            );
        }

        for bridge in &self.gpio_bridges {
            let active_low = bridge.config.active_low;
            match bridge.direction {
                GpioDirection::Output => {
                    let handle = io.pin(bridge.pin)?;
                    // Drive the channel's power-on state now: an MCU that has
                    // not written yet must still present a level, not float.
                    let initial = instance.gpio.get_active(bridge.channel);
                    handle.set_drive(Some(output_drive(level_of_active(initial, active_low))));

                    // `(train, direction while this channel is active)` for
                    // every pulse channel that named this GPIO as its source.
                    let directions: Vec<(Arc<PulseBridgeState>, PulseDirection)> =
                        direction_of_gpio
                            .iter()
                            .filter(|(source, _)| source.channel == bridge.channel)
                            .map(|(source, state)| (Arc::clone(state), source.active_direction))
                            .collect();
                    // The train's direction must already agree with the pin
                    // the bridge just drove, or the first segment published
                    // before any GPIO write would carry the wrong sign.
                    for (state, active_direction) in &directions {
                        state.set_direction(direction_of_active(initial, *active_direction));
                    }
                    let shutdown = Arc::clone(&self.shutdown);
                    instance.gpio.on_change(bridge.channel, move |active| {
                        if shutdown.load(Ordering::Relaxed) {
                            return;
                        }
                        handle.set_drive(Some(output_drive(level_of_active(active, active_low))));
                        // Same write, one meaning further on: this channel is
                        // a step train's direction.
                        for (state, active_direction) in &directions {
                            state.set_direction(direction_of_active(active, *active_direction));
                        }
                    });
                }
                GpioDirection::Input => {
                    let instance = Arc::clone(&instance);
                    let shutdown = Arc::clone(&self.shutdown);
                    let channel = bridge.channel;
                    let pin = bridge.pin;
                    io.on_sense(bridge.pin, move |state| {
                        if shutdown.load(Ordering::Relaxed) {
                            return;
                        }
                        match level_of(state) {
                            // An *external* write: it must not re-enter the
                            // firmware's own change callback, which is what
                            // `set_state` (unlike `set_active`) guarantees.
                            Some(level) => instance
                                .gpio
                                .set_state(channel, active_of_level(level, active_low)),
                            None => tracing::trace!(
                                pin,
                                ?state,
                                "GPIO input has no logic level; holding the last value"
                            ),
                        }
                    })?;
                }
            }
            tracing::debug!(
                mcu = %self.name,
                channel = bridge.channel,
                pin = bridge.pin,
                direction = ?bridge.direction,
                active_low,
                "GPIO channel bridged to a pin"
            );
        }

        for bridge in &self.encoder_bridges {
            let state = Arc::new(Mutex::new(QuadratureState::default()));
            for (pin, is_a) in [(bridge.pin_a, true), (bridge.pin_b, false)] {
                let state = Arc::clone(&state);
                let instance = Arc::clone(&instance);
                let shutdown = Arc::clone(&self.shutdown);
                let channel = bridge.channel;
                io.on_sense(pin, move |net_state| {
                    if shutdown.load(Ordering::Relaxed) {
                        return;
                    }
                    let Some(level) = level_of(net_state) else {
                        tracing::trace!(
                            pin,
                            ?net_state,
                            "encoder phase has no logic level; holding the last count"
                        );
                        return;
                    };
                    let high = level == Level::High;
                    let stepped = {
                        let mut state = state.lock().expect("quadrature state never poisoned");
                        if is_a {
                            state.a = Some(high);
                        } else {
                            state.b = Some(high);
                        }
                        state.step()
                    };
                    match stepped {
                        // Increment the firmware's own counter register, so a
                        // homing `HAL_encoder_set` re-bases the count instead
                        // of being overwritten by the next edge.
                        Ok(Some(delta)) => {
                            let current = instance.encoder.value(channel);
                            instance.encoder.set(channel, current.saturating_add(delta));
                        }
                        Ok(None) => {}
                        Err(slip) => tracing::warn!(
                            channel,
                            pin,
                            from = slip.previous,
                            to = slip.position,
                            "encoder phase jumped two quadrature states; the transition \
                             is unrecoverable and was NOT counted"
                        ),
                    }
                })?;
            }
            tracing::debug!(
                mcu = %self.name,
                channel = bridge.channel,
                a = bridge.pin_a,
                b = bridge.pin_b,
                "encoder channel bridged to a quadrature pin pair"
            );
        }

        self.instance = Some(instance);
        Ok(())
    }

    fn start(&mut self) {
        // Facade mode: nothing to run. (`get_mut`: the mutex is a Sync
        // shim, never contended — see the field docs.)
        let Some(entry) = self.entry.get_mut().expect("never poisoned").take() else {
            return;
        };
        let instance = Arc::clone(
            self.own_instance
                .as_ref()
                .expect("an entry always builds with its own instance"),
        );
        let thread = std::thread::Builder::new()
            .name(format!("mcu-{}-entry", self.name))
            .spawn(move || {
                // Route every peripheral free function on this thread — and,
                // via `system::start_thread` inheritance, every thread the
                // firmware spawns through the HAL — to this component's
                // instance. The entry typically never returns, so the guard
                // lives for the thread's life (and its `Arc` keeps the
                // instance alive even if the component drops first).
                let _bind = embsim_peripherals::instance::bind_current_thread(instance);
                entry();
            })
            .expect("spawn the MCU entry thread");
        tracing::info!(mcu = %self.name, "firmware entry spawned on component-owned instance");
        self.entry_thread = Some(thread);
    }
}

impl Drop for McuComponent {
    fn drop(&mut self) {
        // Neutralize every callback this component left in a peripheral bank
        // before anything else: in facade mode those banks are the process
        // default and outlive us.
        self.shutdown.store(true, Ordering::Relaxed);
        // Flag every pump first so all threads wind down concurrently, then
        // join and reclaim. Join latency is bounded by the poll timeout.
        for pump in &self.pumps {
            pump.shutdown.store(true, Ordering::Relaxed);
        }
        for pump in &mut self.pumps {
            if let Some(thread) = pump.thread.take() {
                let _ = thread.join();
            }
            // Disconnect the peripheral bank before closing its FD so the
            // firmware side sees "not connected", never a closed descriptor.
            if let Some(instance) = &self.instance {
                instance.serial.init_channel_fd(pump.channel, -1);
            }
            // SAFETY: both FDs were created by this component's attach and
            // are not used past this point: the pump thread is joined, the
            // engine (RX callback) shut down before component drop, and the
            // peripheral bank was just disconnected.
            unsafe {
                libc::close(pump.component_fd);
                libc::close(pump.firmware_fd);
            }
        }
    }
}

// ============================================================
// Pump internals
// ============================================================

/// Poll timeout for the pump thread: the upper bound on shutdown latency,
/// comfortably finer than any protocol timeout the firmware runs.
const PUMP_POLL_TIMEOUT_MS: i32 = 10;

/// Read chunk for draining firmware TX bytes.
const PUMP_READ_CHUNK: usize = 256;

/// Pump thread body: wait (bounded) for the component-side FD to become
/// readable, drain it, and hand the bytes to `sink` — the TX pin's byte route,
/// or the level bridge that frames them into edges. Exits when the shutdown
/// flag is set, the peer end closes, or the FD errors.
fn pump_loop(component_fd: RawFd, sink: &dyn Fn(&[u8]), shutdown: &AtomicBool) {
    let mut buf = [0u8; PUMP_READ_CHUNK];
    while !shutdown.load(Ordering::Relaxed) {
        let mut pollfd = libc::pollfd {
            fd: component_fd,
            events: libc::POLLIN,
            revents: 0,
        };
        // SAFETY: `pollfd` is a valid, exclusively borrowed array of one.
        let rc = unsafe { libc::poll(&mut pollfd, 1, PUMP_POLL_TIMEOUT_MS) };
        if rc < 0 {
            let err = std::io::Error::last_os_error();
            if err.kind() == std::io::ErrorKind::Interrupted {
                continue;
            }
            tracing::debug!(error = %err, "serial pump poll failed; stopping");
            return;
        }
        if rc == 0 {
            continue; // timeout — re-check the shutdown flag
        }

        // Drain everything available. The FD is non-blocking, so the inner
        // loop always terminates at EAGAIN.
        // SAFETY: `component_fd` stays open until the owning component joins
        // this thread.
        let fd = unsafe { BorrowedFd::borrow_raw(component_fd) };
        loop {
            match nix::unistd::read(fd, &mut buf) {
                Ok(0) => return, // peer end closed
                Ok(n) => sink(&buf[..n]),
                Err(nix::errno::Errno::EAGAIN) => break,
                Err(nix::errno::Errno::EINTR) => continue,
                Err(e) => {
                    tracing::debug!(error = %e, "serial pump read failed; stopping");
                    return;
                }
            }
        }
    }
}

/// Create a bidirectional non-blocking pipe pair (AF_UNIX socketpair) —
/// the models crate's ADS122U04 pattern, with errors surfaced instead of
/// asserted. Returns `(component_fd, firmware_fd)`.
///
/// Both sides are non-blocking: the firmware HAL's receive-timeout semantics
/// depend on `EAGAIN`, and the pump/RX-callback sides must never block.
fn create_pipe_pair() -> Result<(RawFd, RawFd), String> {
    let mut fds = [0i32; 2];
    // SAFETY: `fds` is a valid 2-slot output buffer for socketpair.
    let rc = unsafe { libc::socketpair(libc::AF_UNIX, libc::SOCK_STREAM, 0, fds.as_mut_ptr()) };
    if rc != 0 {
        return Err(format!("socketpair: {}", std::io::Error::last_os_error()));
    }
    for fd in fds {
        // SAFETY: `fd` is a live descriptor just returned by socketpair.
        let ok = unsafe {
            let flags = libc::fcntl(fd, libc::F_GETFL);
            flags >= 0 && libc::fcntl(fd, libc::F_SETFL, flags | libc::O_NONBLOCK) >= 0
        };
        if !ok {
            let err = std::io::Error::last_os_error();
            // SAFETY: both descriptors are live and owned here.
            unsafe {
                libc::close(fds[0]);
                libc::close(fds[1]);
            }
            return Err(format!("fcntl O_NONBLOCK: {err}"));
        }
    }
    Ok((fds[0], fds[1]))
}

// ============================================================
// Errors
// ============================================================

/// [`McuBuilder::build`] failure.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum McuBuildError {
    /// A bridged channel index is not in the provided serial table.
    UnknownSerialChannel {
        /// The requested channel.
        channel: usize,
        /// How many entries the table has.
        table_len: usize,
    },
    /// A bridged channel index is not in the provided GPIO table.
    UnknownGpioChannel {
        /// The requested channel.
        channel: usize,
        /// How many entries the table has.
        table_len: usize,
    },
    /// A bridged channel index is not in the provided pulse-out table.
    UnknownPulseOutChannel {
        /// The requested channel.
        channel: usize,
        /// How many entries the table has.
        table_len: usize,
    },
    /// A bridged channel index is not in the provided encoder table.
    UnknownEncoderChannel {
        /// The requested channel.
        channel: usize,
        /// How many entries the table has.
        table_len: usize,
    },
    /// A pulse channel named a direction GPIO channel that is not itself
    /// bridged as a [`GpioDirection::Output`] — the direction hook shares
    /// that channel's single `on_change` slot, so it cannot be installed on
    /// its own.
    DirectionChannelNotBridged {
        /// The pulse channel that asked for a direction source.
        pulse_channel: usize,
        /// The GPIO channel it named.
        gpio_channel: usize,
    },
    /// A referenced physical pin is past the P63 ceiling.
    PinOutOfRange {
        /// The offending pin index.
        pin: u32,
    },
    /// Two declarations claim the same physical pin.
    DuplicatePin {
        /// The doubly-claimed pin index.
        pin: u32,
    },
}

impl std::fmt::Display for McuBuildError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            McuBuildError::UnknownSerialChannel { channel, table_len } => write!(
                f,
                "serial channel {channel} is not in the table ({table_len} entries)"
            ),
            McuBuildError::UnknownGpioChannel { channel, table_len } => write!(
                f,
                "GPIO channel {channel} is not in the table ({table_len} entries)"
            ),
            McuBuildError::UnknownPulseOutChannel { channel, table_len } => write!(
                f,
                "pulse-out channel {channel} is not in the table ({table_len} entries)"
            ),
            McuBuildError::UnknownEncoderChannel { channel, table_len } => write!(
                f,
                "encoder channel {channel} is not in the table ({table_len} entries)"
            ),
            McuBuildError::DirectionChannelNotBridged {
                pulse_channel,
                gpio_channel,
            } => write!(
                f,
                "pulse channel {pulse_channel} takes its direction from GPIO channel \
                 {gpio_channel}, which is not bridged as an output"
            ),
            McuBuildError::PinOutOfRange { pin } => {
                write!(f, "pin {pin} is past the P63 ceiling")
            }
            McuBuildError::DuplicatePin { pin } => {
                write!(f, "pin P{pin} is claimed by more than one declaration")
            }
        }
    }
}

impl std::error::Error for McuBuildError {}

// ============================================================
// Tests
// ============================================================

#[cfg(test)]
mod tests {
    use rstest::rstest;

    use super::*;

    /// The reference consumer's force-gauge channel shape.
    const FG: SerialChannelConfig = SerialChannelConfig {
        rx_pin: 0,
        tx_pin: 2,
        baud: 115_200,
    };

    /// A bridged serial channel declares its TX pin as a stream producer and
    /// its RX pin as a stream consumer, both at the table baud, named "P{n}".
    #[rstest]
    fn bridged_serial_channel_declares_stream_pins() {
        let mcu = McuComponent::builder("p2")
            .serial_table(vec![FG])
            .bridge_serial(0)
            .build()
            .expect("builds");

        let tx = mcu
            .pins()
            .iter()
            .find(|p| p.number == "P2")
            .expect("TX pin declared");
        assert_eq!(tx.kind, PinKind::DigitalOut);
        assert_eq!(tx.stream, Some(StreamRole::Producer { baud_hz: 115_200 }));

        let rx = mcu
            .pins()
            .iter()
            .find(|p| p.number == "P0")
            .expect("RX pin declared");
        assert_eq!(rx.kind, PinKind::DigitalIn);
        assert_eq!(rx.stream, Some(StreamRole::Consumer { baud_hz: 115_200 }));
    }

    /// Channels that are not bridged are not declared at all — the facade
    /// stays minimal this slice.
    #[rstest]
    fn unbridged_channels_declare_no_pins() {
        let main = SerialChannelConfig {
            rx_pin: 53,
            tx_pin: 55,
            baud: 2_000_000,
        };
        let mcu = McuComponent::builder("p2")
            .serial_table(vec![FG, main])
            .bridge_serial(0)
            .build()
            .expect("builds");
        assert_eq!(mcu.pins().len(), 2, "only the bridged channel's pins");
        assert!(mcu.pins().iter().all(|p| p.number != "P53"));
        assert!(mcu.pins().iter().all(|p| p.number != "P55"));
    }

    /// GPIO declarations map direction to pin kind, carry no stream role,
    /// and default (no declaration) means no pin.
    #[rstest]
    fn gpio_declarations_follow_direction() {
        let mcu = McuComponent::builder("p2")
            .gpio(
                GpioChannelConfig {
                    pin: 6,
                    active_low: false,
                },
                GpioDirection::Output,
            )
            .gpio(
                GpioChannelConfig {
                    pin: 16,
                    active_low: true,
                },
                GpioDirection::Input,
            )
            .build()
            .expect("builds");

        let ena = mcu.pins().iter().find(|p| p.number == "P6").expect("P6");
        assert_eq!(ena.kind, PinKind::DigitalOut);
        assert_eq!(ena.stream, None);
        let esd = mcu.pins().iter().find(|p| p.number == "P16").expect("P16");
        assert_eq!(esd.kind, PinKind::DigitalIn);
    }

    /// Builder validation: unknown channel, out-of-range pin, and duplicate
    /// pin claims each fail loudly with the matching error.
    #[rstest]
    fn builder_validation_errors() {
        assert_eq!(
            McuComponent::builder("p2")
                .serial_table(vec![FG])
                .bridge_serial(1)
                .build()
                .unwrap_err(),
            McuBuildError::UnknownSerialChannel {
                channel: 1,
                table_len: 1
            }
        );

        assert_eq!(
            McuComponent::builder("p2")
                .serial_table(vec![SerialChannelConfig {
                    rx_pin: 0,
                    tx_pin: 64,
                    baud: 9600
                }])
                .bridge_serial(0)
                .build()
                .unwrap_err(),
            McuBuildError::PinOutOfRange { pin: 64 }
        );

        assert_eq!(
            McuComponent::builder("p2")
                .serial_table(vec![FG])
                .bridge_serial(0)
                .gpio(
                    GpioChannelConfig {
                        pin: 2,
                        active_low: false
                    },
                    GpioDirection::Output
                )
                .build()
                .unwrap_err(),
            McuBuildError::DuplicatePin { pin: 2 }
        );
    }

    /// The pin-name table covers exactly P0..=P63.
    #[rstest]
    fn pin_names_cover_the_p2_pin_space() {
        assert_eq!(pin_name(0), Some("P0"));
        assert_eq!(pin_name(63), Some("P63"));
        assert_eq!(pin_name(64), None);
        for (i, name) in PIN_NAMES.iter().enumerate() {
            assert_eq!(*name, format!("P{i}"));
        }
    }

    /// Build errors render their fields.
    #[rstest]
    fn error_display() {
        assert!(McuBuildError::UnknownSerialChannel {
            channel: 3,
            table_len: 2
        }
        .to_string()
        .contains('3'));
        assert!(McuBuildError::PinOutOfRange { pin: 99 }
            .to_string()
            .contains("99"));
        assert!(McuBuildError::DuplicatePin { pin: 2 }
            .to_string()
            .contains("P2"));
        assert!(McuBuildError::DirectionChannelNotBridged {
            pulse_channel: 0,
            gpio_channel: 4,
        }
        .to_string()
        .contains('4'));
        for error in [
            McuBuildError::UnknownGpioChannel {
                channel: 1,
                table_len: 0,
            },
            McuBuildError::UnknownPulseOutChannel {
                channel: 2,
                table_len: 0,
            },
            McuBuildError::UnknownEncoderChannel {
                channel: 3,
                table_len: 0,
            },
        ] {
            assert!(!error.to_string().is_empty());
        }
    }

    // ========================================================
    // Motion channel facades
    // ========================================================

    /// A bridged pulse-out channel declares exactly one STEP pin: a push-pull
    /// output (so the net still resolves to a level) carrying the pulse route.
    #[rstest]
    fn a_bridged_pulse_channel_declares_one_step_pin() {
        let mcu = McuComponent::builder("p2")
            .pulse_out_table(vec![PulseOutChannelConfig { pin: 8 }])
            .bridge_pulse_out(0)
            .build()
            .expect("builds");

        assert_eq!(mcu.pins().len(), 1);
        let step = &mcu.pins()[0];
        assert_eq!(step.number, "P8");
        assert_eq!(step.kind, PinKind::DigitalOut);
        assert_eq!(step.stream, Some(StreamRole::PulseSource));
    }

    /// A bridged encoder channel declares its phase pair as sensed inputs with
    /// no channel role: the quadrature is decoded from plain net levels.
    #[rstest]
    fn a_bridged_encoder_channel_declares_a_sensed_phase_pair() {
        let mcu = McuComponent::builder("p2")
            .encoder_table(vec![EncoderChannelConfig {
                pin_a: 20,
                pin_b: 21,
            }])
            .bridge_encoder(0)
            .build()
            .expect("builds");

        let names: Vec<&str> = mcu.pins().iter().map(|p| p.number).collect();
        assert_eq!(names, ["P20", "P21"]);
        for pin in mcu.pins() {
            assert_eq!(pin.kind, PinKind::DigitalIn);
            assert_eq!(pin.stream, None);
        }
    }

    /// A pulse channel may only take its direction from a GPIO channel that is
    /// itself bridged as an output — the hook shares that channel's single
    /// `on_change` slot, so a silent second installation is refused.
    #[rstest]
    fn a_direction_source_must_be_a_bridged_gpio_output() {
        let build = |direction: GpioDirection| {
            McuComponent::builder("p2")
                .pulse_out_table(vec![PulseOutChannelConfig { pin: 8 }])
                .gpio_table(vec![GpioChannelConfig {
                    pin: 9,
                    active_low: false,
                }])
                .bridge_gpio(0, direction)
                .bridge_pulse_out_with_direction(0, 0, PulseDirection::Reverse)
                .build()
        };
        assert!(build(GpioDirection::Output).is_ok());
        assert_eq!(
            build(GpioDirection::Input).unwrap_err(),
            McuBuildError::DirectionChannelNotBridged {
                pulse_channel: 0,
                gpio_channel: 0,
            }
        );

        // Naming a channel that is not bridged at all fails the same way.
        assert_eq!(
            McuComponent::builder("p2")
                .pulse_out_table(vec![PulseOutChannelConfig { pin: 8 }])
                .bridge_pulse_out_with_direction(0, 3, PulseDirection::Forward)
                .build()
                .unwrap_err(),
            McuBuildError::DirectionChannelNotBridged {
                pulse_channel: 0,
                gpio_channel: 3,
            }
        );
    }

    /// Every motion channel validates its index against its own table, and pin
    /// collisions across *different* channel kinds are caught too.
    #[rstest]
    fn motion_channel_validation_errors() {
        assert_eq!(
            McuComponent::builder("p2")
                .bridge_pulse_out(0)
                .build()
                .unwrap_err(),
            McuBuildError::UnknownPulseOutChannel {
                channel: 0,
                table_len: 0
            }
        );
        assert_eq!(
            McuComponent::builder("p2")
                .bridge_encoder(2)
                .build()
                .unwrap_err(),
            McuBuildError::UnknownEncoderChannel {
                channel: 2,
                table_len: 0
            }
        );
        assert_eq!(
            McuComponent::builder("p2")
                .gpio_table(vec![])
                .bridge_gpio(0, GpioDirection::Input)
                .build()
                .unwrap_err(),
            McuBuildError::UnknownGpioChannel {
                channel: 0,
                table_len: 0
            }
        );
        // STEP on P8 and an encoder phase on P8: one physical pin, two owners.
        assert_eq!(
            McuComponent::builder("p2")
                .pulse_out_table(vec![PulseOutChannelConfig { pin: 8 }])
                .encoder_table(vec![EncoderChannelConfig { pin_a: 8, pin_b: 9 }])
                .bridge_pulse_out(0)
                .bridge_encoder(0)
                .build()
                .unwrap_err(),
            McuBuildError::DuplicatePin { pin: 8 }
        );
    }

    // ========================================================
    // Polarity + level projection
    // ========================================================

    /// `active_low` is honored in both bridge directions and round-trips: the
    /// level an active state drives reads back as that same active state.
    #[rstest]
    #[case::active_high(false, true, Level::High)]
    #[case::active_high_inactive(false, false, Level::Low)]
    #[case::active_low(true, true, Level::Low)]
    #[case::active_low_inactive(true, false, Level::High)]
    fn gpio_polarity_round_trips(
        #[case] active_low: bool,
        #[case] active: bool,
        #[case] expect: Level,
    ) {
        assert_eq!(level_of_active(active, active_low), expect);
        assert_eq!(active_of_level(expect, active_low), active);
    }

    /// The bridge projects exactly what the engine will commit to, and refuses
    /// to invent a level where the engine gave none.
    #[rstest]
    #[case::driven_high(NetState::Driven(Level::High), Some(Level::High))]
    #[case::driven_low(NetState::Driven(Level::Low), Some(Level::Low))]
    #[case::pulled(NetState::Pulled(Level::High, 10_000.0), Some(Level::High))]
    #[case::analog_above(NetState::Analog(3.0), Some(Level::High))]
    #[case::analog_below(NetState::Analog(0.4), Some(Level::Low))]
    #[case::floating(NetState::Floating, None)]
    #[case::contention(NetState::Contention, None)]
    fn input_projection_never_invents_a_level(
        #[case] state: NetState,
        #[case] expect: Option<Level>,
    ) {
        assert_eq!(level_of(state), expect);
    }

    /// A direction GPIO's active state maps to the declared direction and its
    /// inactive state to the opposite — the mapping is the consumer's, not a
    /// hard-coded polarity.
    #[rstest]
    #[case::active_means_reverse(PulseDirection::Reverse, true, PulseDirection::Reverse)]
    #[case::inactive_means_forward(PulseDirection::Reverse, false, PulseDirection::Forward)]
    #[case::active_means_forward(PulseDirection::Forward, true, PulseDirection::Forward)]
    #[case::inactive_means_reverse(PulseDirection::Forward, false, PulseDirection::Reverse)]
    fn a_direction_gpio_maps_to_the_declared_direction(
        #[case] active_direction: PulseDirection,
        #[case] active: bool,
        #[case] expect: PulseDirection,
    ) {
        assert_eq!(direction_of_active(active, active_direction), expect);
    }

    // ========================================================
    // Quadrature decode
    // ========================================================

    /// Feed a phase sequence into the decoder and sum the deltas it emits.
    fn decode(sequence: &[(bool, bool)]) -> Result<i32, QuadratureSlip> {
        let mut state = QuadratureState::default();
        let mut count = 0;
        for &(a, b) in sequence {
            state.a = Some(a);
            state.b = Some(b);
            if let Some(delta) = state.step()? {
                count += delta;
            }
        }
        Ok(count)
    }

    /// One full Gray cycle in each direction is ×4 counts, signed by the walk
    /// order. The first observation only seeds the detector.
    #[rstest]
    fn a_gray_cycle_counts_four_in_the_direction_it_walks() {
        let up = [
            (false, false),
            (true, false),
            (true, true),
            (false, true),
            (false, false),
        ];
        assert_eq!(decode(&up).expect("clean walk"), 4);
        let mut down = up;
        down.reverse();
        assert_eq!(decode(&down).expect("clean walk"), -4);
    }

    /// A phase pair that does not move counts nothing, however often it is
    /// re-delivered (the engine re-delivers on any net change).
    #[rstest]
    fn a_repeated_phase_pair_counts_nothing() {
        assert_eq!(
            decode(&[(true, false), (true, false), (true, false)]).expect("no transition"),
            0
        );
    }

    /// Both channels changing between deliveries is physically impossible for
    /// a real encoder, so the direction is unrecoverable and the decoder says
    /// so rather than guessing a plausible ±1.
    #[rstest]
    #[case::diagonal_up((false, false), (true, true))]
    #[case::diagonal_down((true, false), (false, true))]
    fn a_two_state_jump_is_reported_not_guessed(
        #[case] from: (bool, bool),
        #[case] to: (bool, bool),
    ) {
        let slip = decode(&[from, to]).expect_err("a two-state jump is a slip");
        assert_ne!(slip.previous, slip.position);
        assert!(format!("{slip:?}").contains("QuadratureSlip"));
    }
}
