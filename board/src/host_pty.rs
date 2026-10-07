//! The host's end of a serial link, as a board component.
//!
//! A PTY on one side, two digital pins on the other. Whatever a host program
//! writes to the PTY is framed onto a net as edges; whatever the net carries
//! back is deframed and handed to the host.
//!
//! # Why a net and not a file descriptor
//!
//! The straightforward way to give a simulated device a serial port is to hand
//! the PTY master straight to whatever models its UART — the bytes never touch
//! a net, and it works. What it cannot do is go wrong the way a wire goes
//! wrong. Here the same descriptor sits behind a [`SerialLevelBridge`], so the
//! host's traffic crosses the same net the device's does, at the same rate,
//! framed the same way, and a baud mismatch or a broken idle level produces
//! the framing errors it would on a bench rather than being defined away.
//!
//! # Threading
//!
//! Nothing on a net-resolution path may block on a file descriptor, so reading
//! the host is a pump thread's job. The engine thread queues guest bytes and
//! makes one non-blocking write attempt (`deliver`); the pump drains whatever
//! the PTY would not take — see that function for why dropping instead would
//! be much worse than it looks. The pump reads the host only while the wire
//! queue has room, and counts every byte it has to shed. On the host's own
//! rail ([`HostPty::open_on_rail`]) it reads nothing until the engine has
//! delivered the rail pins' first reading: the engine does that on its own
//! thread after `attach`, and a byte read before then would be shed as if
//! the host had no power, at whatever wall-clock instant the pump won the
//! race. The host's bytes wait in the PTY instead.

use std::collections::VecDeque;
use std::os::fd::{AsRawFd, BorrowedFd, RawFd};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;

use crate::net::{Level, TheveninDrive, Volts, DEFAULT_PUSH_PULL_IMPEDANCE};
use crate::uart::{FramingError, UartFraming};
use crate::{
    jesd8c01_lvcmos_thresholds, AttachError, Component, ComponentNetIo, DeadBand, PinDecl,
    SerialLevelBridge,
};
use embsim_core::serial_pty::Pty;

/// Poll timeout for the pump thread: the bound on shutdown latency, finer than
/// any protocol timeout the firmware runs.
const PUMP_POLL_TIMEOUT_MS: i32 = 10;

/// Read chunk for draining host bytes.
const PUMP_READ_CHUNK: usize = 256;

/// Write chunk for pushing guest bytes at the host.
const PUMP_WRITE_CHUNK: usize = 4096;

/// How many outbound bytes to hold when the host is not reading fast enough.
///
/// Generous, because the cost of being wrong is asymmetric: a delayed byte is
/// invisible to a protocol with its own timeouts, while a *dropped* byte
/// silently corrupts the frame it was part of and every framing decision after
/// it. Only a host that has genuinely stopped reading reaches this.
const OUTBOUND_MAX: usize = 1 << 20;

/// Counts a [`HostPty`] publishes so a test can see a dropped byte.
///
/// Taken with [`HostPty::counters`] before the component is moved into a
/// system. Every field is monotonic.
pub struct HostPtyCounters {
    /// Guest bytes the PTY accepted.
    pub to_host: AtomicU64,
    /// Host bytes accepted onto the wire.
    pub from_host: AtomicU64,
    /// Frames the wire delivered that failed their stop bit.
    pub framing_errors: AtomicU64,
    /// Guest bytes discarded because the host stopped reading.
    pub dropped_outbound: AtomicU64,
    /// Host bytes the wire queue would not take.
    pub shed_inbound: AtomicU64,
}

impl std::fmt::Debug for HostPtyCounters {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("HostPtyCounters")
            .field("to_host", &self.to_host.load(Ordering::Relaxed))
            .field("from_host", &self.from_host.load(Ordering::Relaxed))
            .field(
                "framing_errors",
                &self.framing_errors.load(Ordering::Relaxed),
            )
            .field(
                "dropped_outbound",
                &self.dropped_outbound.load(Ordering::Relaxed),
            )
            .field("shed_inbound", &self.shed_inbound.load(Ordering::Relaxed))
            .finish()
    }
}

/// The pins of a host whose I/O rail is its own `VIO` pin, against its own
/// `GND` ([`HostPty::open_on_rail`]).
const ON_RAIL_PINS: [PinDecl; 4] = [
    // Driven at the sensed `VIO` above `GND`, through the bridge's ports;
    // released while `VIO` reads no voltage.
    PinDecl::digital_out("TX")
        .with_idle(None)
        .with_reference("GND"),
    // The host's receiver: JESD8C.01's LVCMOS/LVTTL pair, against the
    // host's own ground.
    PinDecl::digital_in("RX", jesd8c01_lvcmos_thresholds(DeadBand::Unknown)).with_reference("GND"),
    PinDecl::power_in("VIO").with_reference("GND"),
    PinDecl::power_in("GND"),
];

/// What the host's rail pins last read: `VIO` against `GND`, and `GND` in
/// the engine's frame; and whether the engine has delivered each yet.
#[derive(Debug, Default, Clone, Copy)]
struct Rail {
    vio: Option<Volts>,
    gnd: Option<Volts>,
    vio_read: bool,
    gnd_read: bool,
}

impl Rail {
    /// The port the TX pin presents for `level`: `GND`, or `VIO` above it,
    /// behind the push-pull default; `None` (released) while either reads
    /// no voltage.
    fn port(self, level: Level) -> Option<TheveninDrive> {
        let (vio, gnd) = (self.vio?, self.gnd?);
        Some(TheveninDrive {
            volts: match level {
                Level::High => gnd + vio,
                Level::Low => gnd,
            },
            impedance: DEFAULT_PUSH_PULL_IMPEDANCE,
        })
    }
}

/// A serial link whose far end is a PTY the host can open.
pub struct HostPty {
    pins: Vec<PinDecl>,
    /// Whether TX drives at the sensed `VIO` above `GND`
    /// ([`Self::open_on_rail`]) or at the crate's logic rail.
    on_rail: bool,
    framing: UartFraming,
    /// Kept alive for the component's life: dropping it closes the PTY and
    /// removes the symlink.
    pty: Pty,
    counters: Arc<HostPtyCounters>,
    shutdown: Arc<AtomicBool>,
    pump: Option<JoinHandle<()>>,
    /// Guest bytes the PTY has not accepted yet. See `deliver`.
    outbound: Arc<Mutex<VecDeque<u8>>>,
}

impl std::fmt::Debug for HostPty {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("HostPty")
            .field("symlink", &self.pty.symlink_path)
            .field("framing", &self.framing)
            .finish()
    }
}

impl HostPty {
    /// Open a PTY at `symlink_path` and frame it at `baud_hz`.
    ///
    /// The pins are named `"TX"` (what the host sends, driven onto the net)
    /// and `"RX"` (what the host receives), from the *host's* point of view —
    /// so a harness reads `HOST.TX → MCU.RX` the way a cable does.
    pub fn open(symlink_path: &str, baud_hz: u32) -> std::io::Result<Self> {
        Self::opened(
            symlink_path,
            baud_hz,
            vec![
                PinDecl::digital_out("TX"),
                // The host's end is a bench adapter no datasheet here
                // describes: its receiver reads at the 3.3 V LVCMOS pair
                // the link signals at.
                PinDecl::digital_in("RX", jesd8c01_lvcmos_thresholds(DeadBand::Unknown)),
            ],
            false,
        )
    }

    /// Open a PTY at `symlink_path`, framed at `baud_hz`, for a host whose
    /// I/O rail and ground are pins of its own: `TX`, `RX`, `VIO` and `GND`.
    ///
    /// `TX` drives a high at `VIO` above `GND` and a low at `GND`, behind
    /// the push-pull default, and is released while `VIO` reads no voltage;
    /// `RX` reads JESD8C.01's 0.8 V / 2.0 V pair against `GND` — the pair a
    /// 3.3 V LVCMOS input and a 5 V TTL input both take. A project wires
    /// the host's real rail to `VIO` (a Raspberry Pi's 3.3 V), so a board
    /// that expects another sees the margin it really has.
    pub fn open_on_rail(symlink_path: &str, baud_hz: u32) -> std::io::Result<Self> {
        Self::opened(symlink_path, baud_hz, ON_RAIL_PINS.to_vec(), true)
    }

    fn opened(
        symlink_path: &str,
        baud_hz: u32,
        pins: Vec<PinDecl>,
        on_rail: bool,
    ) -> std::io::Result<Self> {
        Ok(Self {
            pins,
            on_rail,
            framing: UartFraming::new_8n1(baud_hz),
            pty: Pty::new(symlink_path)?,
            counters: Arc::new(HostPtyCounters {
                to_host: AtomicU64::new(0),
                from_host: AtomicU64::new(0),
                framing_errors: AtomicU64::new(0),
                dropped_outbound: AtomicU64::new(0),
                shed_inbound: AtomicU64::new(0),
            }),
            outbound: Arc::new(Mutex::new(VecDeque::new())),
            shutdown: Arc::new(AtomicBool::new(false)),
            pump: None,
        })
    }

    /// The path a host opens to reach this link.
    pub fn symlink_path(&self) -> &str {
        &self.pty.symlink_path
    }

    /// The counters, shared with the pump after this component is moved.
    pub fn counters(&self) -> Arc<HostPtyCounters> {
        Arc::clone(&self.counters)
    }

    /// The framing the link is clocked at.
    pub fn framing(&self) -> UartFraming {
        self.framing
    }
}

impl Component for HostPty {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let rail = Arc::new(Mutex::new(Rail::default()));
        // Whether the pump may read the host: at once on the fixed logic
        // rail; on the host's own rail, once the engine has delivered both
        // rail pins' first reading. The live engine delivers those on its
        // own thread after `attach` returns, so a pump that read before
        // then would shed the host's bytes as if it had no power, at a
        // wall-clock instant: the bytes wait in the PTY instead.
        let rail_known = Arc::new(AtomicBool::new(!self.on_rail));
        let mut bridge = SerialLevelBridge::new(
            self.framing,
            io.pin("TX")?,
            io.clone(),
            Arc::clone(&self.shutdown),
        );
        if self.on_rail {
            let rail = Arc::clone(&rail);
            bridge = bridge.with_ports(move |level| {
                rail.lock()
                    .expect("the rail reading is never poisoned")
                    .port(level)
            });
        }
        let bridge = Arc::new(bridge);
        if self.on_rail {
            // Unpowered until `VIO` reads a voltage: a host with no rail
            // drives nothing.
            bridge.set_output_enabled(false);
            for pin in ["VIO", "GND"] {
                let (rail, bridge) = (Arc::clone(&rail), Arc::clone(&bridge));
                let rail_known = Arc::clone(&rail_known);
                io.on_sense(pin, move |sense| {
                    let (powered, known) = {
                        let mut rail = rail.lock().expect("the rail reading is never poisoned");
                        if pin == "VIO" {
                            rail.vio = sense.volts;
                            rail.vio_read = true;
                        } else {
                            rail.gnd = sense.volts;
                            rail.gnd_read = true;
                        }
                        (rail.vio.is_some(), rail.vio_read && rail.gnd_read)
                    };
                    bridge.set_output_enabled(powered);
                    bridge.ports_changed();
                    if known {
                        rail_known.store(true, Ordering::Release);
                    }
                })?;
            }
        }
        // An idle asynchronous line still drives: without it the far end has no
        // reference against which the first start bit is a falling edge.
        bridge.idle();

        let master: RawFd = self.pty.master.as_raw_fd();
        let counters = Arc::clone(&self.counters);

        // Net → host: whatever the wire spells goes out the PTY.
        {
            let (bridge, shutdown) = (Arc::clone(&bridge), Arc::clone(&self.shutdown));
            let (outbound, counters) = (Arc::clone(&self.outbound), Arc::clone(&counters));
            let rx = io.pin("RX")?;
            io.on_sense("RX", move |sense| {
                if shutdown.load(Ordering::Relaxed) {
                    return;
                }
                deliver(
                    master,
                    &outbound,
                    &counters,
                    bridge.receive_sense(&rx, &sense),
                );
            })?;
        }
        {
            let (bridge, shutdown) = (Arc::clone(&bridge), Arc::clone(&self.shutdown));
            let (outbound, counters) = (Arc::clone(&self.outbound), Arc::clone(&counters));
            io.on_wake_ns(move |now_ns| {
                if shutdown.load(Ordering::Relaxed) {
                    return;
                }
                deliver(master, &outbound, &counters, bridge.service(now_ns));
            });
        }

        // Host → net: a pump thread, for the same reason the MCU bridge uses
        // one — nothing on a net-resolution path may block on a file
        // descriptor.
        let shutdown = Arc::clone(&self.shutdown);
        let thread = std::thread::Builder::new()
            .name("host-pty-pump".to_string())
            .spawn({
                let (outbound, counters) = (Arc::clone(&self.outbound), counters);
                move || {
                    pump_loop(
                        master,
                        &bridge,
                        &rail_known,
                        &shutdown,
                        &outbound,
                        &counters,
                    );
                }
            })
            .map_err(|e| AttachError::Failed {
                message: format!("host PTY: cannot spawn pump thread: {e}"),
            })?;
        self.pump = Some(thread);
        Ok(())
    }
}

impl Drop for HostPty {
    fn drop(&mut self) {
        self.shutdown.store(true, Ordering::Relaxed);
        if let Some(pump) = self.pump.take() {
            let _ = pump.join();
        }
    }
}

/// Hand deframed bytes to the host.
///
/// Runs on the engine thread, so this must never block. It must not *drop*
/// either: a PTY master holds only a few kilobytes, and a firmware streaming
/// samples fills it routinely — not because the host has stopped reading, but
/// because it has not read *this millisecond*. Dropping there does not look
/// like a slow host to the protocol above; it looks like a corrupt frame, and
/// then like every subsequent framing decision being wrong.
///
/// So bytes queue, and the pump thread drains what the PTY would not take.
fn deliver(
    master: RawFd,
    outbound: &Mutex<VecDeque<u8>>,
    counters: &HostPtyCounters,
    frames: Vec<Result<u8, FramingError>>,
) {
    let mut queue = outbound.lock().expect("outbound queue never poisoned");
    for frame in frames {
        match frame {
            Ok(byte) => queue.push_back(byte),
            Err(error) => {
                counters.framing_errors.fetch_add(1, Ordering::Relaxed);
                tracing::debug!(?error, "host PTY: frame dropped (bad framing on the wire)")
            }
        }
    }
    drain_outbound(master, &mut queue, counters);
}

/// Write as much of the queue as the PTY will accept, and keep the rest.
///
/// Returns having written nothing if the master is full; that is the normal
/// case under load, not an error.
fn drain_outbound(master: RawFd, queue: &mut VecDeque<u8>, counters: &HostPtyCounters) {
    while !queue.is_empty() {
        let take = queue.len().min(PUMP_WRITE_CHUNK);
        let chunk: Vec<u8> = queue.iter().take(take).copied().collect();
        // SAFETY: the master descriptor is owned by the `HostPty` that
        // installed this callback, and the engine is joined before the
        // component drops (`SystemHandle`'s documented order).
        let fd = unsafe { BorrowedFd::borrow_raw(master) };
        match nix::unistd::write(fd, &chunk) {
            Ok(0) => break,
            Ok(written) => {
                queue.drain(..written);
                counters
                    .to_host
                    .fetch_add(written as u64, Ordering::Relaxed);
            }
            // The PTY is full; the pump retries. EWOULDBLOCK is the same
            // errno as EAGAIN on every platform this builds for.
            Err(nix::errno::Errno::EAGAIN) => break,
            Err(e) => {
                tracing::debug!(error = %e, "host PTY: write failed");
                break;
            }
        }
    }
    // Only a host that has truly stopped reading gets here. Counted, not
    // logged, so a test can assert `dropped_outbound` is zero.
    if queue.len() > OUTBOUND_MAX {
        let excess = queue.len() - OUTBOUND_MAX;
        queue.drain(..excess);
        counters
            .dropped_outbound
            .fetch_add(excess as u64, Ordering::Relaxed);
    }
}

/// Read whatever the host wrote and frame it onto the wire, once
/// `rail_known` says the bridge knows whether it is powered.
fn pump_loop(
    master: RawFd,
    bridge: &SerialLevelBridge,
    rail_known: &AtomicBool,
    shutdown: &AtomicBool,
    outbound: &Mutex<VecDeque<u8>>,
    counters: &HostPtyCounters,
) {
    // Bytes the wire would take now: none until the rail is known, so a
    // byte is never read to be shed for a rail not yet delivered.
    let wire_room = || {
        if rail_known.load(Ordering::Acquire) {
            bridge.tx_room()
        } else {
            0
        }
    };
    let mut buf = [0u8; PUMP_READ_CHUNK];
    while !shutdown.load(Ordering::Relaxed) {
        // Anything the engine could not hand over goes now. `POLLOUT` only
        // when there is something waiting, so an idle link still blocks in
        // `poll` rather than spinning. `POLLIN` only while the wire queue
        // has room: reading a byte the queue will shed loses it.
        let pending = {
            let mut queue = outbound.lock().expect("outbound queue never poisoned");
            drain_outbound(master, &mut queue, counters);
            !queue.is_empty()
        };
        let room = wire_room();
        let mut events = 0;
        if room > 0 {
            events |= libc::POLLIN;
        }
        if pending {
            events |= libc::POLLOUT;
        }
        let mut pollfd = libc::pollfd {
            fd: master,
            events,
            revents: 0,
        };
        // SAFETY: `pollfd` is a valid, exclusively borrowed array of one.
        let rc = unsafe { libc::poll(&mut pollfd, 1, PUMP_POLL_TIMEOUT_MS) };
        if rc < 0 {
            let err = std::io::Error::last_os_error();
            if err.kind() == std::io::ErrorKind::Interrupted {
                continue;
            }
            tracing::debug!(error = %err, "host PTY: poll failed; pump stopping");
            return;
        }
        if rc == 0 || room == 0 || pollfd.revents & libc::POLLIN == 0 {
            continue; // timeout, a full wire queue, or nothing to read
        }
        // SAFETY: the master stays open until the owning component joins this
        // thread in `Drop`.
        let fd = unsafe { BorrowedFd::borrow_raw(master) };
        loop {
            let room = wire_room();
            if room == 0 {
                break;
            }
            let cap = room.min(PUMP_READ_CHUNK);
            match nix::unistd::read(fd, &mut buf[..cap]) {
                Ok(0) => break, // no host attached yet; poll again
                Ok(n) => {
                    let shed = bridge.transmit(&buf[..n]);
                    if shed > 0 {
                        counters
                            .shed_inbound
                            .fetch_add(shed as u64, Ordering::Relaxed);
                        tracing::warn!(shed, "host PTY: inbound bytes shed");
                    }
                    counters
                        .from_host
                        .fetch_add(n.saturating_sub(shed) as u64, Ordering::Relaxed);
                }
                Err(nix::errno::Errno::EAGAIN) => break,
                Err(nix::errno::Errno::EINTR) => continue,
                Err(e) => {
                    tracing::debug!(error = %e, "host PTY: read failed; pump stopping");
                    return;
                }
            }
        }
    }
}
