//! The component: a [`Guest`] on two pins, its clock slaved to the board's.

use std::collections::VecDeque;
use std::io;
use std::os::fd::RawFd;
use std::sync::atomic::{AtomicBool, AtomicI32, AtomicU64, Ordering};
use std::sync::{mpsc, Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use embsim_board::uart::{FramingError, UartFraming};
use embsim_board::{AttachError, Component, ComponentNetIo, PinDecl, PinKind, SerialLevelBridge};
use embsim_core::virtual_clock;

use crate::guest::Guest;

/// The default slice: how far the board runs between two runs of the guest.
///
/// The slice is the latency a byte can wait before the guest may react to
/// it, so shorter is better until the freeze/thaw cost dominates. Measured
/// on an Apple Silicon host (QEMU 11, HVF): one freeze/thaw costs 0.5 ms of
/// host time and lets the guest live at least ~0.3 ms even with no window,
/// with microseconds of jitter. One millisecond is the USB frame period —
/// the latency a real USB serial link has anyway — at a freeze/thaw cost of
/// about half the slice, which the board (far slower still) hides.
pub const DEFAULT_SLICE: Duration = Duration::from_millis(1);

/// The longest slice the node accepts.
///
/// The node's thread is awake — holding virtual time still — for a slice
/// plus two freeze/thaw round trips, and the engine's quiescence barrier
/// gives up on an actor that stays awake past `System::quiescence_timeout`
/// (five seconds by default), permanently. One second keeps a wide margin.
pub const MAX_SLICE: Duration = Duration::from_secs(1);

/// Bytes waiting for a guest that is not draining its port. Past this the
/// oldest are shed and counted — a guest that has not read a megabyte is
/// not coming back for the head of it, so unlike the bridge's TX queue
/// (which sheds newest, like a full FIFO) this drops from the front.
const OUTBOUND_MAX: usize = 1 << 20;

/// Bytes read from the guest's port and not yet on the line. A megabyte is
/// five seconds of a 2 Mbaud line; a guest that gets that far ahead of it
/// is not talking to the board any more, and the oldest are shed, counted.
const INBOUND_MAX: usize = 1 << 20;

/// Chunk sizes for the serial socket.
const READ_CHUNK: usize = 4096;
const WRITE_CHUNK: usize = 4096;

/// The pump's poll timeout: the bound on how long shutdown waits for it.
const PUMP_POLL_MS: i32 = 10;

/// How long [`Component::start`] waits for the actor thread to register with
/// the virtual clock. Registration is a mutex and a map insert; this bound
/// only exists so a failed spawn cannot hang assembly.
const REGISTER_TIMEOUT: Duration = Duration::from_secs(5);

/// Counters a test or an operator can read while the node runs.
#[derive(Debug, Default)]
pub struct NodeStats {
    slices: AtomicU64,
    clocked: AtomicU64,
    guest_ns: AtomicU64,
    virtual_ns: AtomicU64,
    from_guest: AtomicU64,
    to_guest: AtomicU64,
    shed: AtomicU64,
    disconnected: AtomicBool,
}

impl NodeStats {
    /// Slices in which the guest ran.
    pub fn slices(&self) -> u64 {
        self.slices.load(Ordering::Relaxed)
    }

    /// Slices whose length was taken from the guest's own clock
    /// ([`crate::Guest::clock_ns`]) rather than the node's stopwatch.
    pub fn clocked(&self) -> u64 {
        self.clocked.load(Ordering::Relaxed)
    }

    /// Wall time the guest has been allowed to run, in nanoseconds — its own
    /// clock's progress, for a hardware-virtualised guest.
    pub fn guest_ns(&self) -> u64 {
        self.guest_ns.load(Ordering::Relaxed)
    }

    /// Virtual time that has passed since the node started, in nanoseconds.
    pub fn virtual_ns(&self) -> u64 {
        self.virtual_ns.load(Ordering::Relaxed)
    }

    /// How far the board's clock is ahead of the guest's (negative: behind).
    pub fn skew_ns(&self) -> i64 {
        self.virtual_ns() as i64 - self.guest_ns() as i64
    }

    /// Bytes the guest sent that reached the line.
    pub fn from_guest(&self) -> u64 {
        self.from_guest.load(Ordering::Relaxed)
    }

    /// Bytes delivered to the guest's port.
    pub fn to_guest(&self) -> u64 {
        self.to_guest.load(Ordering::Relaxed)
    }

    /// Bytes shed because a queue overflowed: the guest stopped reading its
    /// port, or wrote more than a megabyte the line has not carried yet.
    /// Zero in a healthy run.
    pub fn shed(&self) -> u64 {
        self.shed.load(Ordering::Relaxed)
    }

    /// Whether the guest's serial port went away (the process exited).
    pub fn disconnected(&self) -> bool {
        self.disconnected.load(Ordering::Relaxed)
    }
}

/// The guest, shared between the node and its actor thread.
///
/// The actor holds the lock for the length of a slice; the node takes the
/// guest out on drop, which is what shuts it down.
type GuestSlot = Arc<Mutex<Option<Box<dyn Guest>>>>;

/// A byte queue between two threads.
type ByteQueue = Arc<Mutex<VecDeque<u8>>>;

/// The guest's serial descriptor as the reader sees it, or -1 while unplugged.
///
/// The reader cannot ask the guest directly: the actor holds that lock for a
/// whole slice, so a reader that locked per poll would stall for a slice at a
/// time. An atomic is the handoff.
type SerialFd = Arc<AtomicI32>;

/// The cable, as a thing a test can pull.
///
/// A handle onto one node's serial attachment and nothing else: the guest sits
/// behind the same mutex the pump uses, so a caller here cannot reach the rest
/// of it, and cannot resume or pause a guest the actor is metering.
///
/// The timing works out for free. The pump locks the slot for exactly one
/// slice and drops it, so a call made from any other thread waits for the
/// current slice to finish and then runs BETWEEN slices -- never against a
/// guest that is mid-run.
#[derive(Clone)]
pub struct LinkControl {
    guest: GuestSlot,
    fd: SerialFd,
}

impl std::fmt::Debug for LinkControl {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("LinkControl").finish_non_exhaustive()
    }
}

impl LinkControl {
    /// Pull the cable: the guest's serial port detaches as if unplugged.
    pub fn unplug(&self) -> io::Result<()> {
        self.set(false)
    }

    /// Put it back. The port re-enumerates in the guest.
    pub fn plug(&self) -> io::Result<()> {
        self.set(true)
    }

    /// Whether the port is attached right now.
    pub fn is_plugged(&self) -> bool {
        self.guest
            .lock()
            .expect("guest slot never poisoned")
            .as_ref()
            .is_some_and(|g| g.serial_attached())
    }

    fn set(&self, attached: bool) -> io::Result<()> {
        let mut slot = self.guest.lock().expect("guest slot never poisoned");
        match slot.as_mut() {
            Some(guest) => {
                guest.set_serial_attached(attached)?;
                // Publish the new descriptor before releasing the guest, so
                // the reader never polls one that has just been closed.
                self.fd.store(guest.serial_fd(), Ordering::Release);
                Ok(())
            }
            // The node was dropped and took the guest with it. Saying so beats
            // reporting success for a cable that no longer has a machine on
            // the other end.
            None => Err(io::Error::new(
                io::ErrorKind::NotConnected,
                "the guest is gone",
            )),
        }
    }
}

/// A computer on the board.
///
/// Two pins, named from the computer's point of view: `TX` is what it
/// sends (driven onto the net), `RX` what it hears. See the crate docs for
/// the timing model.
///
/// Three threads touch the guest's serial socket, each in one direction:
/// the engine thread deframes the board's bytes into `outbound`; the actor
/// writes `outbound` to the socket during a slice; and a pump thread reads
/// the socket into `inbound` at all times, from which the actor feeds the
/// bridge as fast as the line drains. The pump is not an optimisation: QEMU
/// writes the guest's serial bytes to the socket *blocking*, under its big
/// lock, so a socket nobody reads stalls the vCPU and with it QMP — and a
/// node waiting on `stop` while not reading the socket would wait forever.
pub struct QemuNode {
    pins: [PinDecl; 2],
    framing: UartFraming,
    slice_ns: u64,
    guest: GuestSlot,
    shutdown: Arc<AtomicBool>,
    bridge: Option<Arc<SerialLevelBridge>>,
    outbound: ByteQueue,
    inbound: ByteQueue,
    stats: Arc<NodeStats>,
    serial_fd: SerialFd,
    pump: Option<JoinHandle<()>>,
    started: bool,
}

impl std::fmt::Debug for QemuNode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("QemuNode")
            .field("slice_ns", &self.slice_ns)
            .field("started", &self.started)
            .field("stats", &self.stats)
            .finish()
    }
}

impl QemuNode {
    /// A node around a guest whose serial line runs 8N1 at `baud_hz`.
    pub fn new(guest: Box<dyn Guest>, baud_hz: u32) -> Self {
        Self {
            pins: [
                PinDecl {
                    number: "TX",
                    name: None,
                    kind: PinKind::DigitalOut,
                    stream: None,
                    drive_impedance: None,
                },
                PinDecl {
                    number: "RX",
                    name: None,
                    kind: PinKind::DigitalIn,
                    stream: None,
                    drive_impedance: None,
                },
            ],
            framing: UartFraming::new_8n1(baud_hz),
            slice_ns: DEFAULT_SLICE.as_nanos() as u64,
            guest: Arc::new(Mutex::new(Some(guest))),
            shutdown: Arc::new(AtomicBool::new(false)),
            serial_fd: Arc::new(AtomicI32::new(-1)),
            bridge: None,
            outbound: Arc::new(Mutex::new(VecDeque::new())),
            inbound: Arc::new(Mutex::new(VecDeque::new())),
            stats: Arc::new(NodeStats::default()),
            pump: None,
            started: false,
        }
    }

    /// Set the slice (default [`DEFAULT_SLICE`], at most [`MAX_SLICE`]).
    /// Shorter slices bound the skew tighter and cost proportionally more
    /// freeze/thaw overhead.
    pub fn with_slice(mut self, slice: Duration) -> Self {
        let slice = if slice > MAX_SLICE {
            tracing::warn!(?slice, ?MAX_SLICE, "qemu node: slice clamped");
            MAX_SLICE
        } else {
            slice
        };
        self.slice_ns = slice.as_nanos().max(1) as u64;
        self
    }

    /// The node's counters, readable from any thread.
    /// A handle for unplugging and replugging this node's serial port.
    ///
    /// Cloneable and safe to keep across the node's lifetime -- once the node
    /// is dropped, every call reports `NotConnected` rather than panicking.
    pub fn link(&self) -> LinkControl {
        LinkControl {
            guest: Arc::clone(&self.guest),
            fd: Arc::clone(&self.serial_fd),
        }
    }

    pub fn stats(&self) -> Arc<NodeStats> {
        Arc::clone(&self.stats)
    }
}

impl Component for QemuNode {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let bridge = Arc::new(SerialLevelBridge::new(
            self.framing,
            io.pin("TX")?,
            io.clone(),
            Arc::clone(&self.shutdown),
        ));
        // Drive the mark level before the first byte so the peer has an edge
        // reference; a floating line decodes as garbage.
        bridge.idle();

        {
            let (bridge, outbound, stats, shutdown) = (
                Arc::clone(&bridge),
                Arc::clone(&self.outbound),
                Arc::clone(&self.stats),
                Arc::clone(&self.shutdown),
            );
            io.on_sense("RX", move |state| {
                if shutdown.load(Ordering::Relaxed) {
                    return;
                }
                deliver(&outbound, &stats, bridge.receive_sense(state));
            })?;
        }
        {
            let (bridge, outbound, stats, shutdown) = (
                Arc::clone(&bridge),
                Arc::clone(&self.outbound),
                Arc::clone(&self.stats),
                Arc::clone(&self.shutdown),
            );
            io.on_wake_ns(move |now_ns| {
                if shutdown.load(Ordering::Relaxed) {
                    return;
                }
                deliver(&outbound, &stats, bridge.service(now_ns));
            });
        }
        self.bridge = Some(bridge);
        Ok(())
    }

    fn start(&mut self) {
        if self.started {
            tracing::error!("QemuNode::start: already started");
            return;
        }
        let Some(bridge) = self.bridge.clone() else {
            tracing::error!("QemuNode::start: not attached");
            return;
        };
        let fd = match self
            .guest
            .lock()
            .expect("guest slot never poisoned")
            .as_ref()
        {
            Some(guest) => guest.serial_fd(),
            None => {
                tracing::error!("QemuNode::start: no guest");
                return;
            }
        };
        self.serial_fd.store(fd, Ordering::Release);
        self.started = true;

        // The pump: reads the guest's port for as long as the node lives.
        {
            let (inbound, stats, shutdown, serial_fd) = (
                Arc::clone(&self.inbound),
                Arc::clone(&self.stats),
                Arc::clone(&self.shutdown),
                Arc::clone(&self.serial_fd),
            );
            match thread::Builder::new()
                .name("embsim-qemu-pump".into())
                .spawn(move || pump_main(&serial_fd, &inbound, &stats, &shutdown))
            {
                Ok(handle) => self.pump = Some(handle),
                Err(e) => {
                    tracing::error!(error = %e, "QemuNode::start: could not spawn the pump thread");
                    return;
                }
            }
        }

        let context = ActorContext {
            guest: Arc::clone(&self.guest),
            bridge,
            outbound: Arc::clone(&self.outbound),
            inbound: Arc::clone(&self.inbound),
            stats: Arc::clone(&self.stats),
            shutdown: Arc::clone(&self.shutdown),
            slice_ns: self.slice_ns,
        };
        let (registered_tx, registered_rx) = mpsc::channel();
        let spawned = thread::Builder::new()
            .name("embsim-qemu-actor".into())
            .spawn(move || actor_main(context, registered_tx));
        if let Err(e) = spawned {
            tracing::error!(error = %e, "QemuNode::start: could not spawn the actor thread");
            return;
        }
        // The actor must be registered before time is released, or its first
        // slice could race an advance already in flight (new actors start
        // runnable, but only once the scheduler knows about them).
        if registered_rx.recv_timeout(REGISTER_TIMEOUT).is_err() {
            tracing::error!("QemuNode::start: the actor thread never registered");
        }
    }
}

impl Drop for QemuNode {
    fn drop(&mut self) {
        self.shutdown.store(true, Ordering::Relaxed);
        // The pump exits within one poll timeout once the flag is up; join it
        // before the guest (and its descriptor) goes away.
        if let Some(pump) = self.pump.take() {
            let _ = pump.join();
        }
        // Components drop after the engine is joined, and the engine only
        // exits once every actor has parked — so the actor is parked here,
        // the guest is not mid-slice, and taking it out of the slot is safe.
        // Dropping the guest is what shuts it down (a `QemuVm` quits QEMU).
        // The actor thread itself stays parked on a deadline the board will
        // never reach; it is not joined — the same detach an MCU's firmware
        // thread gets — and if a clock re-init ever wakes it, it finds the
        // slot empty and exits.
        let guest = self.guest.lock().expect("guest slot never poisoned").take();
        drop(guest);
    }
}

/// Everything the actor thread works with.
struct ActorContext {
    guest: GuestSlot,
    bridge: Arc<SerialLevelBridge>,
    outbound: ByteQueue,
    inbound: ByteQueue,
    stats: Arc<NodeStats>,
    shutdown: Arc<AtomicBool>,
    slice_ns: u64,
}

/// The metering loop.
///
/// Park at the next slice boundary; when the engine gets there, run the
/// guest for as much wall time as the board has advanced that the guest has
/// not yet lived; freeze it; repeat. `owed_ns` is the closed loop: a slice
/// the guest overran (poll's millisecond granularity, a late `stop`) is paid
/// back by a shorter run next time, so skew is bounded over a whole run.
fn actor_main(context: ActorContext, registered: mpsc::Sender<()>) {
    let _actor = virtual_clock::register_actor("qemu-node");
    let _ = registered.send(());

    let mut last_virtual_ns = virtual_clock::virtual_ns();
    let mut owed_ns: i64 = 0;
    // When the guest can read its own clock, each slice's true length is the
    // difference between its reading and the previous one, and replaces the
    // stopwatch estimate booked for that previous slice.
    let mut last_clock: Option<(u64, u64)> = None; // (guest clock, estimate booked)
    while !context.shutdown.load(Ordering::Relaxed) {
        virtual_clock::wait_until_ns(last_virtual_ns + context.slice_ns);
        if context.shutdown.load(Ordering::Relaxed) {
            break;
        }
        let now_ns = virtual_clock::virtual_ns();
        if now_ns < last_virtual_ns {
            // The clock was re-initialised under us (tests do this). Resync
            // rather than owe the guest a negative eternity.
            last_virtual_ns = now_ns;
            owed_ns = 0;
            continue;
        }
        let advanced = now_ns - last_virtual_ns;
        last_virtual_ns = now_ns;
        context
            .stats
            .virtual_ns
            .fetch_add(advanced, Ordering::Relaxed);
        owed_ns += advanced as i64;
        // The line has drained since the last slice: put what the guest sent
        // meanwhile on it, at this slice's virtual instant.
        feed_bridge(&context.inbound, &context.bridge, &context.stats);
        if owed_ns <= 0 {
            continue;
        }
        let mut slot = context.guest.lock().expect("guest slot never poisoned");
        let Some(guest) = slot.as_mut() else {
            break; // the node was dropped and took the guest with it
        };
        let budget = Duration::from_nanos(owed_ns as u64);
        let outcome = run_slice(guest.as_mut(), &context.outbound, &context.stats, budget);
        drop(slot);
        // Whatever the guest sent during its slice goes on the line at the
        // same virtual instant — the engine has not moved while we were awake.
        feed_bridge(&context.inbound, &context.bridge, &context.stats);
        match outcome {
            Ok((lived, clock)) => {
                let lived_ns = lived.as_nanos() as u64;
                if let Some(clock) = clock {
                    if let Some((prev_clock, booked)) = last_clock {
                        // The exact length of the previous slice; correct
                        // what the stopwatch booked for it.
                        let exact = clock.saturating_sub(prev_clock);
                        let delta = exact as i64 - booked as i64;
                        owed_ns -= delta;
                        if delta >= 0 {
                            context
                                .stats
                                .guest_ns
                                .fetch_add(delta as u64, Ordering::Relaxed);
                        } else {
                            context
                                .stats
                                .guest_ns
                                .fetch_sub(delta.unsigned_abs(), Ordering::Relaxed);
                        }
                    }
                    last_clock = Some((clock, lived_ns));
                    context.stats.clocked.fetch_add(1, Ordering::Relaxed);
                }
                owed_ns -= lived_ns as i64;
                context
                    .stats
                    .guest_ns
                    .fetch_add(lived_ns, Ordering::Relaxed);
                context.stats.slices.fetch_add(1, Ordering::Relaxed);
            }
            Err(e) => {
                tracing::error!(error = %e, "qemu node: the guest failed mid-slice; leaving it frozen");
                break;
            }
        }
    }
    if let Some(guest) = context
        .guest
        .lock()
        .expect("guest slot never poisoned")
        .as_mut()
    {
        let _ = guest.pause();
    }
    tracing::info!(
        slices = context.stats.slices(),
        skew_ns = context.stats.skew_ns(),
        "qemu node: actor exiting"
    );
}

/// Run the guest for `budget` of wall time, writing it the board's bytes
/// meanwhile. Returns how long it ran by the node's stopwatch, and the
/// guest's own clock reading at the start of the run if it offers one.
fn run_slice(
    guest: &mut dyn Guest,
    outbound: &Mutex<VecDeque<u8>>,
    stats: &NodeStats,
    budget: Duration,
) -> io::Result<(Duration, Option<u64>)> {
    let fd = guest.serial_fd();
    if fd < 0 {
        // Unplugged. Anything the board sent meanwhile is DISCARDED rather
        // than held: a cable that is out does not buffer, and delivering the
        // backlog on replug would be a fiction no real port performs -- and
        // the app's reconnect path would then see a burst that never happened.
        outbound
            .lock()
            .expect("outbound queue never poisoned")
            .clear();
    } else {
        // Hand the guest what the board sent while it was frozen. Whatever the
        // socket will not take yet goes on the first POLLOUT below.
        drain_outbound(fd, outbound, stats)?;
    }
    guest.resume()?;
    let start = Instant::now();
    let clock = guest.clock_ns();
    let outcome = service_writes(fd, outbound, stats, start, budget);
    let pause = guest.pause();
    let lived = start.elapsed();
    outcome?;
    pause?;
    Ok((lived, clock))
}

/// The body of a slice: write the guest its bytes until the budget is spent.
fn service_writes(
    fd: RawFd,
    outbound: &Mutex<VecDeque<u8>>,
    stats: &NodeStats,
    start: Instant,
    budget: Duration,
) -> io::Result<()> {
    loop {
        let elapsed = start.elapsed();
        if elapsed >= budget {
            return Ok(());
        }
        let remaining = budget - elapsed;
        let want_out = !outbound
            .lock()
            .expect("outbound queue never poisoned")
            .is_empty();
        let mut pollfd = libc::pollfd {
            fd,
            events: if want_out { libc::POLLOUT } else { 0 },
            revents: 0,
        };
        // poll() counts milliseconds; rounding up overruns a slice by under a
        // millisecond, which the owed-time loop takes back on the next one.
        let whole_ms =
            remaining.as_millis() + u128::from(!remaining.subsec_nanos().is_multiple_of(1_000_000));
        let timeout_ms = whole_ms.min(i32::MAX as u128) as i32;
        // With nothing to write, poll on no descriptors: a portable sleep for
        // the rest of the budget.
        let nfds = libc::nfds_t::from(want_out);
        // SAFETY: `pollfd` is a valid, initialised array of one element for
        // the duration of the call (and `nfds` is at most one), and `fd` is
        // owned by the guest, which outlives this slice.
        let ready = unsafe { libc::poll(&mut pollfd, nfds, timeout_ms) };
        if ready < 0 {
            let e = io::Error::last_os_error();
            if e.kind() == io::ErrorKind::Interrupted {
                continue;
            }
            return Err(e);
        }
        if ready == 0 {
            continue;
        }
        if pollfd.revents & (libc::POLLERR | libc::POLLNVAL) != 0 {
            stats.disconnected.store(true, Ordering::Relaxed);
            return Err(io::Error::new(
                io::ErrorKind::BrokenPipe,
                "the guest's serial port went away",
            ));
        }
        if pollfd.revents & libc::POLLOUT != 0 {
            drain_outbound(fd, outbound, stats)?;
        }
    }
}

/// The pump: read the guest's port into `inbound` for as long as the node
/// lives, whether or not the guest is running or the line has room.
fn pump_main(
    serial_fd: &SerialFd,
    inbound: &Mutex<VecDeque<u8>>,
    stats: &NodeStats,
    shutdown: &AtomicBool,
) {
    let mut buf = [0u8; READ_CHUNK];
    while !shutdown.load(Ordering::Relaxed) {
        // Re-read every pass. The descriptor changes when the cable is pulled
        // and again when it is put back, and a reader holding the old one
        // would poll a closed fd, get POLLNVAL and declare the guest dead --
        // which is exactly what a deliberate unplug used to do here.
        let fd = serial_fd.load(Ordering::Acquire);
        if fd < 0 {
            // Unplugged, not gone. Wait for it to come back rather than
            // ending the pump; the node has to survive the outage for a
            // reconnect test to have anything to reconnect to.
            std::thread::sleep(Duration::from_millis(PUMP_POLL_MS as u64));
            continue;
        }
        let mut pollfd = libc::pollfd {
            fd,
            events: libc::POLLIN,
            revents: 0,
        };
        // SAFETY: `pollfd` is a valid, initialised array of one element for
        // the duration of the call, and `fd` stays open until the node has
        // joined this thread.
        let ready = unsafe { libc::poll(&mut pollfd, 1, PUMP_POLL_MS) };
        if ready < 0 {
            if io::Error::last_os_error().kind() == io::ErrorKind::Interrupted {
                continue;
            }
            break;
        }
        if ready == 0 {
            continue;
        }
        let mut got_data = false;
        if pollfd.revents & libc::POLLIN != 0 {
            // SAFETY: `buf` is a valid writable buffer of READ_CHUNK bytes and
            // `fd` is a live descriptor.
            let n = unsafe { libc::read(fd, buf.as_mut_ptr().cast(), READ_CHUNK) };
            if n < 0 {
                let e = io::Error::last_os_error();
                if e.kind() == io::ErrorKind::WouldBlock || e.kind() == io::ErrorKind::Interrupted {
                    continue;
                }
                break;
            }
            if n == 0 {
                // EOF. If the descriptor has been replaced since this pass
                // began, the cable was pulled -- go round and pick up the new
                // one. Otherwise the guest really did close its port.
                if serial_fd.load(Ordering::Acquire) != fd {
                    continue;
                }
                break;
            }
            got_data = true;
            let mut queue = inbound.lock().expect("inbound queue never poisoned");
            queue.extend(&buf[..n as usize]);
            if queue.len() > INBOUND_MAX {
                let excess = queue.len() - INBOUND_MAX;
                queue.drain(..excess);
                stats.shed.fetch_add(excess as u64, Ordering::Relaxed);
            }
        }
        // A peer that hung up with bytes still queued reports HUP alongside
        // IN; those are read first (the loop comes back here) and only then
        // is the hang-up the end.
        if pollfd.revents & (libc::POLLHUP | libc::POLLERR | libc::POLLNVAL) != 0 && !got_data {
            // Same distinction as the EOF above: a descriptor that has been
            // replaced since this poll began was unplugged, not lost.
            if serial_fd.load(Ordering::Acquire) != fd {
                continue;
            }
            break;
        }
    }
    if !shutdown.load(Ordering::Relaxed) {
        stats.disconnected.store(true, Ordering::Relaxed);
        tracing::warn!("qemu node: the guest's serial port went away");
    }
}

/// Move what the pump has read onto the line, as much as the bridge has room
/// for. Called only while the actor is awake, so the bytes are anchored at
/// the current virtual instant; the rest waits for the line to drain.
fn feed_bridge(inbound: &Mutex<VecDeque<u8>>, bridge: &SerialLevelBridge, stats: &NodeStats) {
    let mut queue = inbound.lock().expect("inbound queue never poisoned");
    let take = queue.len().min(bridge.tx_room());
    if take == 0 {
        return;
    }
    let chunk: Vec<u8> = queue.drain(..take).collect();
    // Not stamped here: the engine anchors the first bit when it services
    // the bridge's wake, at this slice's virtual instant.
    let shed = bridge.transmit(&chunk);
    stats
        .from_guest
        .fetch_add((chunk.len() - shed) as u64, Ordering::Relaxed);
    if shed > 0 {
        // Cannot happen: at most `tx_room()` bytes were taken. Counted anyway.
        stats.shed.fetch_add(shed as u64, Ordering::Relaxed);
    }
}

/// Queue deframed bytes for the guest.
///
/// Runs on the engine thread, so it must not block and must not drop: the
/// guest is usually frozen when the board speaks, and a frame dropped here
/// looks to the protocol above like corruption, not like a slow host.
fn deliver(
    outbound: &Mutex<VecDeque<u8>>,
    stats: &NodeStats,
    frames: Vec<Result<u8, FramingError>>,
) {
    let mut queue = outbound.lock().expect("outbound queue never poisoned");
    for frame in frames {
        match frame {
            Ok(byte) => queue.push_back(byte),
            Err(error) => {
                tracing::debug!(?error, "qemu node: frame dropped (bad framing on the wire)")
            }
        }
    }
    if queue.len() > OUTBOUND_MAX {
        let excess = queue.len() - OUTBOUND_MAX;
        queue.drain(..excess);
        stats.shed.fetch_add(excess as u64, Ordering::Relaxed);
    }
}

/// Write as much of the queue as the socket accepts; keep the rest.
///
/// A socket that is merely full is the normal case under load and not an
/// error; any other failure is, because retrying it for the rest of the
/// slice would spin on `POLLOUT`.
fn drain_outbound(fd: RawFd, outbound: &Mutex<VecDeque<u8>>, stats: &NodeStats) -> io::Result<()> {
    let mut queue = outbound.lock().expect("outbound queue never poisoned");
    while !queue.is_empty() {
        let take = queue.len().min(WRITE_CHUNK);
        let chunk: Vec<u8> = queue.iter().take(take).copied().collect();
        // SAFETY: `chunk` is a valid buffer of `take` bytes and `fd` is a live
        // descriptor owned by the guest for the node's lifetime.
        let n = unsafe { libc::write(fd, chunk.as_ptr().cast(), chunk.len()) };
        if n > 0 {
            queue.drain(..n as usize);
            stats.to_guest.fetch_add(n as u64, Ordering::Relaxed);
            continue;
        }
        if n == 0 {
            return Ok(());
        }
        let e = io::Error::last_os_error();
        return match e.kind() {
            io::ErrorKind::Interrupted => continue,
            io::ErrorKind::WouldBlock => Ok(()),
            _ => Err(e),
        };
    }
    Ok(())
}
