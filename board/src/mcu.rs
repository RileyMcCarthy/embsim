//! McuComponent — the MCU as a [`Component`] (`BOARD_ENGINE.md`, "The MCU as
//! a component"), **force-path slice**: HAL-table-shaped configs in, physical
//! stream pins out, socketpair bridges into the `embsim-peripherals` serial
//! bank.
//!
//! # Slice scope (and what is deferred)
//!
//! This slice bridges **serial channels only**. Firmware keeps booting
//! exactly as today — `embsim_runtime::Emulator::run` executes the entry on
//! the caller's thread against the **default** `PeripheralInstance` — so the
//! "engine spawns the firmware entry on a component-owned thread" inversion
//! from `BOARD_ENGINE.md` point 1 is **deferred**, and with it per-component
//! peripheral-instance wiring: [`McuComponent::attach`] installs its channel
//! FDs into the *calling thread's* instance (the default instance in today's
//! boot flow) and will target its own instance when the entry-inversion
//! slice lands. GPIO channels can be *declared* (pin facade only, direction
//! per channel); encoder and pulse-out channels stay consumer-hand-wired and
//! are not declared at all this slice.
//!
//! # Config structs are deliberate duplicates
//!
//! [`SerialChannelConfig`] / [`GpioChannelConfig`] mirror the structs
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
use std::sync::Arc;
use std::thread::JoinHandle;

use embsim_peripherals::instance::PeripheralInstance;

use crate::component::{
    AttachError, Component, ComponentNetIo, PinDecl, PinKind, StreamRole, StreamTx,
};

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
    /// `true` when the channel's active state drives the pin low. Carried
    /// for the GPIO-bridging slice; unused while GPIO is declaration-only.
    pub active_low: bool,
}

/// Electrical direction of a declared GPIO channel pin (the HAL tables do
/// not encode direction, so the builder takes it per channel).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GpioDirection {
    /// The MCU senses this pin ([`PinKind::DigitalIn`]).
    Input,
    /// The MCU drives this pin ([`PinKind::DigitalOut`]).
    Output,
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
#[derive(Debug, Default)]
pub struct McuBuilder {
    name: String,
    serial_table: Vec<SerialChannelConfig>,
    bridged_serial: Vec<usize>,
    gpio: Vec<(GpioChannelConfig, GpioDirection)>,
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

    /// Declare one GPIO channel's pin with the given direction. Declaration
    /// only this slice — the behavioral bridge into the peripheral GPIO bank
    /// arrives with a later slice; undeclared channels stay hand-wired.
    pub fn gpio(mut self, config: GpioChannelConfig, direction: GpioDirection) -> Self {
        self.gpio.push((config, direction));
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
            // UART TX transmits onto the net; RX consumes routed bytes.
            pins.push(PinDecl {
                number: claim(config.tx_pin)?,
                name: None,
                kind: PinKind::DigitalOut,
                stream: Some(StreamRole::Producer {
                    baud_hz: config.baud,
                }),
                drive_impedance: None,
            });
            pins.push(PinDecl {
                number: claim(config.rx_pin)?,
                name: None,
                kind: PinKind::DigitalIn,
                stream: Some(StreamRole::Consumer {
                    baud_hz: config.baud,
                }),
                drive_impedance: None,
            });
            bridges.push(SerialBridge { channel, config });
        }

        for (config, direction) in &self.gpio {
            pins.push(PinDecl {
                number: claim(config.pin)?,
                name: None,
                kind: match direction {
                    GpioDirection::Input => PinKind::DigitalIn,
                    GpioDirection::Output => PinKind::DigitalOut,
                },
                stream: None,
                drive_impedance: None,
            });
        }

        Ok(McuComponent {
            name: self.name,
            pins,
            bridges,
            pumps: Vec::new(),
            instance: None,
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
    pumps: Vec<Pump>,
    /// The peripheral instance the channel FDs were installed into (the
    /// attach thread's instance — the default one in today's boot flow).
    instance: Option<Arc<PeripheralInstance>>,
}

impl std::fmt::Debug for McuComponent {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("McuComponent")
            .field("name", &self.name)
            .field("pins", &self.pins)
            .field("bridges", &self.bridges)
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
}

impl Component for McuComponent {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        // The calling thread's peripheral instance — the process default in
        // today's boot flow (see the module docs; per-component instances
        // arrive with the entry-inversion slice).
        let instance = embsim_peripherals::instance::current();

        for bridge in &self.bridges {
            // Validated at build: both pins are <= P63.
            let tx_name = pin_name(bridge.config.tx_pin).expect("validated at build");
            let rx_name = pin_name(bridge.config.rx_pin).expect("validated at build");

            let tx = io.stream_tx(tx_name)?;
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

            // Net → MCU: routed bytes land on the firmware's read side.
            // Runs on the engine thread — the write must never block, so a
            // full pipe drops the byte with a trace.
            {
                let shutdown = Arc::clone(&shutdown);
                let channel = bridge.channel;
                io.on_byte(rx_name, move |byte| {
                    if shutdown.load(Ordering::Relaxed) {
                        return;
                    }
                    // SAFETY: `component_fd` stays open until the owning
                    // component drops, which happens only after the engine
                    // (and with it this callback) has shut down — see
                    // `SystemHandle`'s documented drop order.
                    let fd = unsafe { BorrowedFd::borrow_raw(component_fd) };
                    if let Err(e) = nix::unistd::write(fd, &[byte]) {
                        tracing::trace!(
                            channel,
                            error = %e,
                            "RX byte dropped (firmware-side pipe not writable)"
                        );
                    }
                })?;
            }

            // MCU → net: a named pump thread moves firmware TX bytes onto
            // the stream route (see the module docs for why a thread and
            // not an engine poll).
            let thread = std::thread::Builder::new()
                .name(format!("mcu-{}-ch{}", self.name, bridge.channel))
                .spawn({
                    let shutdown = Arc::clone(&shutdown);
                    move || pump_loop(component_fd, &tx, &shutdown)
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
                "serial channel bridged to stream pins"
            );
        }

        self.instance = Some(instance);
        Ok(())
    }
}

impl Drop for McuComponent {
    fn drop(&mut self) {
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
/// readable, drain it, and stream the bytes out the TX pin. Exits when the
/// shutdown flag is set, the peer end closes, or the FD errors.
fn pump_loop(component_fd: RawFd, tx: &StreamTx, shutdown: &AtomicBool) {
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
                Ok(n) => tx.write(&buf[..n]),
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
    use super::*;

    /// The reference consumer's force-gauge channel shape.
    const FG: SerialChannelConfig = SerialChannelConfig {
        rx_pin: 0,
        tx_pin: 2,
        baud: 115_200,
    };

    /// A bridged serial channel declares its TX pin as a stream producer and
    /// its RX pin as a stream consumer, both at the table baud, named "P{n}".
    #[test]
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
    #[test]
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
    #[test]
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
    #[test]
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
    #[test]
    fn pin_names_cover_the_p2_pin_space() {
        assert_eq!(pin_name(0), Some("P0"));
        assert_eq!(pin_name(63), Some("P63"));
        assert_eq!(pin_name(64), None);
        for (i, name) in PIN_NAMES.iter().enumerate() {
            assert_eq!(*name, format!("P{i}"));
        }
    }

    /// Build errors render their fields.
    #[test]
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
    }
}
