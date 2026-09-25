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
//! | serial | [`McuBuilder::bridge_serial`] | TX + RX, plain digital | both |
//! | GPIO | [`McuBuilder::bridge_gpio`] | one, per declared direction | firmware → net **and** net → firmware |
//! | pulse-out | [`McuBuilder::bridge_pulse_out`] | one STEP pin, driven [`Drive::Periodic`] | firmware → net |
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
//! events onto the STEP pin as a [`Drive::Periodic`] — the pad's high and low
//! ports and the peripheral's own [`PeriodicSchedule`], **one drive per rate
//! change** — resolved on the net like every other drive, so a fought step
//! line is `Contention`, and the consumer integrates at read time. Exact step
//! counts survive: [`PeriodicSchedule::emitted_at_ns`] is the same integer
//! arithmetic `HAL_pulseOut_run` hands the firmware, so an encoder fed from
//! the segment cannot drift from the firmware's own view. The fidelity this
//! trades away (no edges, no pulse width, no per-edge DIR sampling) is
//! enumerated on [`Drive::Periodic`].
//!
//! The wire carries no direction. A step/direction drive takes it from its
//! own DIR pin — a bridged GPIO output the firmware drives like any other —
//! at the instant the DIR net changes, folding the one segment the STEP pin
//! carries from the peripheral's own anchor.
//!
//! # Hosted in a package
//!
//! Inside a package that decides when the chip runs and what its pads drive
//! at — `embsim-boards`' P2 package, whose `P2Core` impl calls
//! [`McuComponent::host_pads`] — the bridges publish nothing until
//! [`Component::start`], which the package runs at the chip's START
//! instant; there every bridged output presents its power-on state, and
//! every drive after it goes through the package's ports (a pad high at its
//! bank's supply, nothing in a bank with none). The wakes the serial bridges
//! arm go through the package's gate too (the net I/O it hands the core is
//! its own, [`ComponentNetIo::with_wake_gate`]), so a byte the firmware
//! queues before START goes out after it. On a board of its own an MCU
//! publishes from attach, at [`crate::net::digital_drive`]'s nominal ports.
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
//!    transmit_data ──────────┤ socketpair ├─ pump ──► framer → "P{tx}"
//!    receive_*     ◄─────────┤            ├◄─ deframer ← "P{rx}" ◄───┘
//! ```
//!
//! - **MCU → net**: a small named thread (`"mcu-{name}-ch{n}"`) polls the
//!   component-side FD and hands whatever the firmware transmitted to the
//!   channel's framer, which clocks it out the TX pin one bit at a time. A
//!   dedicated thread (rather than an engine `schedule_every` poll) keeps FD
//!   I/O off the net-engine thread — nothing on a net-resolution path may
//!   block — and matches the models crate's existing `protocol_loop`
//!   reader-thread pattern.
//! - **Net → MCU**: the RX pin's resolved state feeds a deframer on the engine
//!   thread, and whatever it decodes is written non-blockingly to the
//!   component-side FD; the firmware reads it from its end. A full pipe drops
//!   the byte with a trace — the engine thread never blocks on a slow
//!   firmware.
//! - **Baud comes from the table**: the framing is 8N1 at
//!   [`SerialChannelConfig::baud`], so the wire clocks at the firmware's own
//!   config — the emulator invents no default. The peripheral bank's own
//!   `set_baud` pacing is deliberately left untouched (unpaced unless the
//!   consumer overrides it, e.g. MaD's `MAD_SIM_BAUD` test override): the wire
//!   is paced in exactly one place.
//!
//! ## The bits are on the net
//!
//! The TX/RX pins are a plain push-pull output and a plain digital input
//! ([`PinDecl::digital_out`] / [`PinDecl::digital_in`]) with no channel role
//! at all, and a byte becomes a start bit, eight data
//! bits and a stop bit at the table baud — driven by
//! [`crate::SerialLevelBridge`] out of the component's wake handler, and
//! decoded on the way back from the RX pin's resolved state.
//!
//! It used to be a byte *route*, which let the net decide reachability and
//! nothing else: a byte crossing it could not be corrupted by a driver
//! fighting it, could not notice the line was floating, and could not break.
//! On levels it can, and `board/tests/serial_levels.rs` shows exactly that —
//! the same byte, the same code, one wire with a second driver on it, and only
//! the contended one fails.
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
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;

use embsim_peripherals::instance::PeripheralInstance;
use embsim_peripherals::pulse_out::PeriodicSchedule;

use crate::component::{
    jesd8c01_lvcmos_thresholds, AttachError, Component, ComponentNetIo, DeadBand, Drive, PinDecl,
    PinHandle, Thresholds,
};
use crate::net::{Level, TheveninDrive};
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

/// Electrical direction of a declared GPIO channel pin (the HAL tables do
/// not encode direction, so the builder takes it per channel).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GpioDirection {
    /// The MCU senses this pin ([`PinDecl::digital_in`]): the net drives, and a
    /// bridged channel writes what it reads into the peripheral GPIO bank —
    /// what an endstop needs.
    Input,
    /// The MCU drives this pin ([`PinDecl::digital_out`]): a bridged channel
    /// turns every firmware write into a net drive.
    Output,
}

// ============================================================
// Electrical defaults for bridged pins
// ============================================================

/// Open-circuit voltage a bridged GPIO output drives for a logic high.
// The digital projection a bridged pin uses in both directions lives in
// [`crate::net`]: it is the engine's own, not the MCU's, and the serial level
// bridge needs the same one. An input with no level holds the last value the
// firmware saw rather than inventing a released state.
use crate::net::digital_drive as output_drive;

/// The thresholds every bridged input declares: the JEDEC JESD8C.01 3.3 V
/// LVCMOS pair ([`jesd8c01_lvcmos_thresholds`]), absolute. The pins are the
/// P2's pads, whose own threshold is relative — `V_IH` between 0.3 and 0.7
/// × the bank's `Vxxyy` (P2X8C4M64P Datasheet, DC Characteristics, p. 47)
/// — but an MCU on a board of its own declares no bank-supply pin for a
/// relative declaration to scale by, so the pair the engine's own dead
/// band already projects through stands in for it, stated. Inside a P2
/// package the package's own declarations apply instead — each pad
/// relative to its bank's `VIO_a_b` — since it is the package that
/// declares the pins ([`McuComponent::host_pads`]).
const BRIDGED_INPUT_THRESHOLDS: Thresholds = jesd8c01_lvcmos_thresholds(DeadBand::Unknown);

/// How a package that **hosts** this MCU drives its pads
/// ([`McuComponent::host_pads`]): the Thevenin port pad `pin` (`"P{n}"`)
/// presents for a level, or `None` for a pad whose driver has no supply.
/// The P2 package (`embsim-boards`) answers from its bank supplies — a pad
/// high at its `VIO_a_b` pin's voltage, at the fast drive strength —
/// exactly as it answers the QEMU core's pads.
pub type PadPorts = Arc<dyn Fn(&'static str, Level) -> Option<TheveninDrive> + Send + Sync>;

/// The drive a bridged pad presents for `level`: the host's port for it
/// ([`PadPorts`]), or — an MCU on a board of its own — the crate's
/// push-pull digital drive.
fn pad_drive(ports: Option<&PadPorts>, pin: &'static str, level: Level) -> Option<TheveninDrive> {
    match ports {
        Some(ports) => ports(pin, level),
        None => Some(output_drive(level)),
    }
}

/// A bridged output's power-on publish, run at attach for an MCU on a
/// board of its own and at [`Component::start`] for a hosted one.
type PowerOn = Box<dyn Fn() + Send + Sync>;

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
    gpio: Vec<(GpioChannelConfig, GpioDirection)>,
    gpio_table: Vec<GpioChannelConfig>,
    bridged_gpio: Vec<(usize, GpioDirection)>,
    pulse_out_table: Vec<PulseOutChannelConfig>,
    bridged_pulse_out: Vec<usize>,
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

    /// Bridge one pulse-output channel to its STEP pin as a
    /// [`Drive::Periodic`] (see the module docs for why a rate and not
    /// edges).
    ///
    /// The pin is declared a push-pull output; every rate change the
    /// peripheral makes is one periodic drive on it — the pad's high and low
    /// ports ([`crate::net::digital_drive`], the same the GPIO bridge drives)
    /// and the peripheral's own segment — resolved on the net like any
    /// drive. The direction is the DIR pin's, a GPIO channel bridged on its
    /// own.
    pub fn bridge_pulse_out(mut self, channel: usize) -> Self {
        self.bridged_pulse_out.push(channel);
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
            // Two plain digital pins: the UART is framed onto the net as
            // levels, so there is no byte route to declare.
            pins.push(PinDecl::digital_out(claim(config.tx_pin)?));
            pins.push(PinDecl::digital_in(
                claim(config.rx_pin)?,
                BRIDGED_INPUT_THRESHOLDS,
            ));
            bridges.push(SerialBridge { channel, config });
        }

        let gpio_pin = |number: &'static str, direction: GpioDirection| match direction {
            GpioDirection::Input => PinDecl::digital_in(number, BRIDGED_INPUT_THRESHOLDS),
            GpioDirection::Output => PinDecl::digital_out(number),
        };

        for (config, direction) in &self.gpio {
            pins.push(gpio_pin(claim(config.pin)?, *direction));
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
            pins.push(gpio_pin(pin, direction));
            gpio_bridges.push(GpioBridge {
                channel,
                config,
                direction,
                pin,
            });
        }

        let mut pulse_bridges: Vec<PulseBridge> = Vec::new();
        for &channel in &self.bridged_pulse_out {
            let config = *self.pulse_out_table.get(channel).ok_or(
                McuBuildError::UnknownPulseOutChannel {
                    channel,
                    table_len: self.pulse_out_table.len(),
                },
            )?;
            let pin = claim(config.pin)?;
            // A step clock is a push-pull output whose every rate change is
            // one periodic drive on it.
            pins.push(PinDecl::digital_out(pin));
            pulse_bridges.push(PulseBridge { channel, pin });
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
                pins.push(PinDecl::digital_in(pin, BRIDGED_INPUT_THRESHOLDS));
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
            gpio_bridges,
            pulse_bridges,
            encoder_bridges,
            pumps: Vec::new(),
            shutdown: Arc::new(AtomicBool::new(false)),
            instance: None,
            own_instance,
            entry: Mutex::new(self.entry),
            entry_thread: None,
            pad_ports: None,
            running: Arc::new(AtomicBool::new(false)),
            power_on: Vec::new(),
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

/// Shared state of one live pulse bridge: the STEP pin's handle.
struct PulseBridgeState {
    pin: PinHandle,
    name: &'static str,
    ports: Option<PadPorts>,
    shutdown: Arc<AtomicBool>,
    running: Arc<AtomicBool>,
}

impl PulseBridgeState {
    /// Publish a segment onto the STEP pin: one periodic drive, between the
    /// pad's own high and low ports — released when either has no supply.
    fn publish(&self, segment: PeriodicSchedule) {
        if self.shutdown.load(Ordering::Relaxed) || !self.running.load(Ordering::Relaxed) {
            return;
        }
        match step_drive(self.ports.as_ref(), self.name, segment) {
            Some(drive) => self.pin.drive(drive),
            None => self.pin.release(),
        }
    }
}

/// The periodic drive a bridged STEP pin presents for `segment`: the pad's
/// own high and low ports ([`pad_drive`] — for an MCU on a board of its
/// own [`crate::net::digital_drive`], the Thevenin every bridged GPIO
/// output of this MCU drives) alternated by the peripheral's own schedule;
/// `None` when either port has no supply.
fn step_drive(
    ports: Option<&PadPorts>,
    pin: &'static str,
    segment: PeriodicSchedule,
) -> Option<Drive> {
    Some(Drive::Periodic {
        hi: pad_drive(ports, pin, Level::High)?,
        lo: pad_drive(ports, pin, Level::Low)?,
        segment,
    })
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
    /// The host's ports for the bridged pads ([`Self::host_pads`]); `None`
    /// for an MCU on a board of its own.
    pad_ports: Option<PadPorts>,
    /// Whether the bridged outputs publish: from attach for an MCU on a
    /// board of its own, from [`Component::start`] for a hosted one.
    running: Arc<AtomicBool>,
    /// A hosted MCU's power-on publishes, held from attach to
    /// [`Component::start`].
    power_on: Vec<PowerOn>,
}

impl std::fmt::Debug for McuComponent {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("McuComponent")
            .field("name", &self.name)
            .field("pins", &self.pins)
            .field("bridges", &self.bridges)
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

    /// Host this MCU inside a package that decides when it runs and what
    /// its pads drive at (`embsim-boards`' P2 package): every bridged pad
    /// drives through `ports` — the level's port as the host reads it at
    /// the instant of the drive, released when the host says its driver
    /// has no supply — and nothing is published before
    /// [`Component::start`], which the host runs at the instant the chip
    /// can run. At start the bridged outputs present their power-on state
    /// (the GPIO outputs' levels, the serial lines' idle, the step
    /// channels' segments), then the firmware entry runs. A chip in reset
    /// drives nothing: that is the state the pads rest in until then.
    /// Call before [`Component::attach`].
    pub fn host_pads(&mut self, ports: PadPorts) {
        self.pad_ports = Some(ports);
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
        // An MCU on a board of its own publishes from attach; a hosted one
        // from the start its host runs it at, its power-on publishes held
        // until then.
        let hosted = self.pad_ports.is_some();
        self.running.store(!hosted, Ordering::Relaxed);
        let mut power_on: Vec<PowerOn> = Vec::new();

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

            // Net → MCU: the RX pin's resolved state is fed to a framer, and
            // whatever it decodes lands on the firmware's read side. Runs on
            // the engine thread, so the write must never block: a full pipe
            // drops the byte with a trace.
            let channel = bridge.channel;
            let mut line = SerialLevelBridge::new(
                UartFraming::new_8n1(bridge.config.baud),
                io.pin(tx_name)?,
                io.clone(),
                Arc::clone(&shutdown),
            );
            if let Some(ports) = &self.pad_ports {
                let ports = Arc::clone(ports);
                line = line.with_ports(move |level| ports(tx_name, level));
            }
            let level = Arc::new(line);
            // Hold the line at idle before the firmware runs: a peer that saw
            // it floating would have no reference for the first start bit's
            // falling edge. A hosted MCU does so at its start, the first
            // instant it drives anything.
            if hosted {
                let level = Arc::clone(&level);
                power_on.push(Box::new(move || level.idle()));
            } else {
                level.idle();
            }
            {
                let level = Arc::clone(&level);
                let shutdown = Arc::clone(&shutdown);
                let rx = io.pin(rx_name)?;
                io.on_sense(rx_name, move |sense| {
                    if shutdown.load(Ordering::Relaxed) {
                        return;
                    }
                    deliver_rx(component_fd, channel, level.receive_sense(&rx, &sense));
                })?;
            }
            level_channels.push(LevelChannel {
                channel,
                component_fd,
                level: Arc::clone(&level),
            });
            let tx_sink: TxSink = Box::new(move |bytes: &[u8]| {
                let shed = level.transmit(bytes);
                if shed > 0 {
                    tracing::trace!(channel, shed, "TX bytes shed: the line is behind");
                }
            });

            // MCU → net: a named pump thread moves firmware TX bytes into the
            // framer (see the module docs for why a thread and not an engine
            // poll).
            let pump_thread_name = format!("mcu-{}-ch{}", self.name, bridge.channel);
            let pump_actor_name = pump_thread_name.clone();
            let thread = std::thread::Builder::new()
                .name(pump_thread_name)
                .spawn({
                    let shutdown = Arc::clone(&shutdown);
                    move || pump_loop(&pump_actor_name, component_fd, &*tx_sink, &shutdown)
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
            io.on_wake_ns(move |now_ns| {
                if shutdown.load(Ordering::Relaxed) {
                    return;
                }
                for channel in channels.iter() {
                    // Each bridge arms its own next wake, so N channels cost N
                    // wheel entries only when N channels actually have work.
                    let bytes = channel.level.service(now_ns);
                    deliver_rx(channel.component_fd, channel.channel, bytes);
                }
            });
        }

        for bridge in &self.pulse_bridges {
            let state = Arc::new(PulseBridgeState {
                pin: io.pin(bridge.pin)?,
                name: bridge.pin,
                ports: self.pad_ports.clone(),
                shutdown: Arc::clone(&self.shutdown),
                running: Arc::clone(&self.running),
            });
            // Establish the channel on the net before the firmware runs, so a
            // sink attaching later has a baseline to fold against. The bank's
            // own segment rather than a synthetic idle — normally identical to
            // [`PeriodicSchedule::IDLE`], but correct even if a channel is already
            // running when this component attaches. Reads state, never the
            // clock, so it is safe before `virtual_clock::init`. A hosted MCU
            // does so at its start, from the segment the bank holds then.
            if hosted {
                let state = Arc::clone(&state);
                let instance = Arc::clone(&instance);
                let channel = bridge.channel;
                power_on.push(Box::new(move || {
                    state.publish(instance.pulse_out.segment(channel));
                }));
            } else {
                state.publish(instance.pulse_out.segment(bridge.channel));
            }
            {
                let state = Arc::clone(&state);
                instance
                    .pulse_out
                    .on_rate_change(bridge.channel, move |segment| state.publish(segment));
            }
            tracing::debug!(
                mcu = %self.name,
                channel = bridge.channel,
                step = bridge.pin,
                "pulse-out channel bridged to a step pin"
            );
        }

        for bridge in &self.gpio_bridges {
            let active_low = bridge.config.active_low;
            match bridge.direction {
                GpioDirection::Output => {
                    let handle = io.pin(bridge.pin)?;
                    let pin = bridge.pin;
                    let ports = self.pad_ports.clone();
                    // Drive the channel's power-on state: an MCU that has not
                    // written yet must still present a level, not float — from
                    // attach on a board of its own, from its start when hosted.
                    {
                        let handle = handle.clone();
                        let ports = ports.clone();
                        let instance = Arc::clone(&instance);
                        let channel = bridge.channel;
                        let publish = move || {
                            let initial = instance.gpio.get_active(channel);
                            handle.set_drive(pad_drive(
                                ports.as_ref(),
                                pin,
                                level_of_active(initial, active_low),
                            ));
                        };
                        if hosted {
                            power_on.push(Box::new(publish));
                        } else {
                            publish();
                        }
                    }
                    let shutdown = Arc::clone(&self.shutdown);
                    let running = Arc::clone(&self.running);
                    instance.gpio.on_change(bridge.channel, move |active| {
                        if shutdown.load(Ordering::Relaxed) || !running.load(Ordering::Relaxed) {
                            return;
                        }
                        handle.set_drive(pad_drive(
                            ports.as_ref(),
                            pin,
                            level_of_active(active, active_low),
                        ));
                    });
                }
                GpioDirection::Input => {
                    let instance = Arc::clone(&instance);
                    let shutdown = Arc::clone(&self.shutdown);
                    let channel = bridge.channel;
                    let pin = bridge.pin;
                    let handle = io.pin(bridge.pin)?;
                    io.on_sense(bridge.pin, move |sense| {
                        if shutdown.load(Ordering::Relaxed) {
                            return;
                        }
                        // The pin's own projection, its last level the one
                        // the firmware's register holds.
                        let last = level_of_active(instance.gpio.get_active(channel), active_low);
                        match handle.level(&sense, Some(last)) {
                            // An *external* write: it must not re-enter the
                            // firmware's own change callback, which is what
                            // `set_state` (unlike `set_active`) guarantees.
                            Some(level) => instance
                                .gpio
                                .set_state(channel, active_of_level(level, active_low)),
                            None => tracing::trace!(
                                pin,
                                ?sense,
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
                let handle = io.pin(pin)?;
                io.on_sense(pin, move |sense| {
                    if shutdown.load(Ordering::Relaxed) {
                        return;
                    }
                    let last = {
                        let state = state.lock().expect("quadrature state never poisoned");
                        if is_a {
                            state.a
                        } else {
                            state.b
                        }
                    }
                    .map(|high| if high { Level::High } else { Level::Low });
                    let Some(level) = handle.level(&sense, last) else {
                        tracing::trace!(
                            pin,
                            ?sense,
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
        self.power_on = power_on;
        Ok(())
    }

    fn start(&mut self) {
        // A hosted MCU's first drives: the host runs `start` at the instant
        // the chip can run, and every bridged output presents its power-on
        // state there, before the firmware entry runs.
        if !self.running.swap(true, Ordering::Relaxed) {
            for publish in self.power_on.drain(..) {
                publish();
            }
        }
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

/// How often the pump drains the firmware's TX FD, in **virtual** microseconds.
///
/// This was a 10 ms wall-clock `poll(2)` timeout, justified as "comfortably
/// finer than any protocol timeout the firmware runs". That comparison does not
/// hold: the firmware's protocol timeouts are counted in VIRTUAL microseconds,
/// and the two clocks are not related by any fixed ratio. Unpaced
/// (`--speed 0`), virtual time advances as fast as the host can spin the parked
/// actors, so 10 ms of wall latency here is an unbounded amount of virtual time
/// — enough to blow through a 1 s firmware timeout while this thread is simply
/// waiting for a timeslice.
///
/// 100 µs matches `Serial::receive_data_timeout`'s own RX poll interval, so the
/// pump never becomes the slower half of a firmware round trip.
const PUMP_POLL_INTERVAL_US: u64 = 100;

/// Read chunk for draining firmware TX bytes.
const PUMP_READ_CHUNK: usize = 256;

/// Pump thread body: drain the component-side FD and hand the bytes to `sink` —
/// the TX pin's byte route, or the level bridge that frames them into edges.
/// Exits when the shutdown flag is set, the peer end closes, or the FD errors.
///
/// # Why this thread is a virtual-clock actor
///
/// This is the MCU→net half of the firmware's serial bridge, and it is the only
/// hop on a firmware↔model byte path that is not the firmware, the engine or a
/// device model — each of which already registers (`ads122u04-protocol`,
/// `reader-cog`, the cogs via `system::start_thread`, and the engine's time
/// authority). Left unregistered, it was invisible to the quiescence barrier:
/// the scheduler would advance virtual time while this thread had not yet been
/// scheduled by the OS to forward a byte the firmware had already written.
///
/// On a contended host that is not a small error. The firmware asks its ADC for
/// a conversion and waits 1 **virtual** second for the reply; unpaced, that
/// second can pass in a sliver of wall time, so the gauge is declared
/// unresponsive, `dev_forceGauge` drops into its error state, and the recorded
/// sample stream thins out and stretches — a measurement of the runner rather
/// than of the machine.
///
/// Registering makes the barrier wait for this hop like any other: virtual time
/// cannot pass this thread's deadline while it has bytes to move.
fn pump_loop(name: &str, component_fd: RawFd, sink: &dyn Fn(&[u8]), shutdown: &AtomicBool) {
    let _actor = embsim_core::virtual_clock::register_actor(name);
    let mut buf = [0u8; PUMP_READ_CHUNK];
    // SAFETY: `component_fd` stays open until the owning component joins this
    // thread.
    let fd = unsafe { BorrowedFd::borrow_raw(component_fd) };
    while !shutdown.load(Ordering::Relaxed) {
        // Drain everything available. The FD is non-blocking, so this always
        // terminates at EAGAIN.
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
        // Park on the virtual clock rather than blocking on the FD: parking is
        // what publishes this thread's deadline to the quiescence barrier, and
        // a `poll(2)` wall-clock block publishes nothing. Also bounds shutdown
        // latency, as the old poll timeout did.
        embsim_core::virtual_clock::wait_virtual_us(PUMP_POLL_INTERVAL_US);
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
        assert_eq!(*tx, PinDecl::digital_out("P2"));

        let rx = mcu
            .pins()
            .iter()
            .find(|p| p.number == "P0")
            .expect("RX pin declared");
        assert_eq!(*rx, PinDecl::digital_in("P0", BRIDGED_INPUT_THRESHOLDS));
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
        assert_eq!(*ena, PinDecl::digital_out("P6"));
        let esd = mcu.pins().iter().find(|p| p.number == "P16").expect("P16");
        assert_eq!(*esd, PinDecl::digital_in("P16", BRIDGED_INPUT_THRESHOLDS));
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
    /// output whose rate changes are periodic drives on it.
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
        assert_eq!(*step, PinDecl::digital_out("P8"));
        assert_eq!(step.idle, Some(output_drive(Level::High)));
    }

    /// The STEP drive is the pad's own high and low ports around the
    /// peripheral's segment, carried whole.
    #[rstest]
    fn a_step_drive_swings_between_the_pads_own_ports() {
        let segment = PeriodicSchedule {
            emitted: 7,
            freq_hz: 20_000,
            total: Some(4_000),
            since_ns: 1_000_000,
        };
        assert_eq!(
            step_drive(None, "P8", segment),
            Some(Drive::Periodic {
                hi: output_drive(Level::High),
                lo: output_drive(Level::Low),
                segment,
            })
        );
        // A hosted pad swings between its host's ports, and presents
        // nothing when either has no supply.
        let at_bank: PadPorts = Arc::new(|_pin, level| {
            Some(TheveninDrive {
                volts: if level == Level::High { 1.8 } else { 0.0 },
                impedance: 18.0,
            })
        });
        assert_eq!(
            step_drive(Some(&at_bank), "P8", segment),
            Some(Drive::Periodic {
                hi: TheveninDrive {
                    volts: 1.8,
                    impedance: 18.0
                },
                lo: TheveninDrive {
                    volts: 0.0,
                    impedance: 18.0
                },
                segment,
            })
        );
        let unpowered: PadPorts = Arc::new(|_pin, _level| None);
        assert_eq!(step_drive(Some(&unpowered), "P8", segment), None);
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
            assert_eq!(pin.senses_at_build(), Some(crate::SenseKind::Digital));
        }
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
