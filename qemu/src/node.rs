//! The component: a [`Guest`] on a host's serial pins, its clock metered by
//! the board's.

use std::collections::VecDeque;
use std::fmt;
use std::io;
use std::os::fd::RawFd;
use std::sync::atomic::{AtomicBool, AtomicI32, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use embsim_board::uart::{FramingError, UartFraming};
use embsim_board::{
    AttachError, Component, ComponentNetIo, HostRailLine, PinDecl, SerialLevelBridge,
    HOST_RAIL_PINS,
};
use embsim_core::virtual_clock;

use crate::guest::Guest;

/// The default quantum: how far the board's clock runs between two slices
/// of the guest.
///
/// The quantum is the latency a byte can wait before the guest may react
/// to it, and the most the two clocks are apart, so shorter is better until
/// the stop/cont round trip dominates. Measured on an Apple M2 (QEMU 11.1.1,
/// HVF, load average 4–5, 2026-10-07; `tests/real_qemu.rs`'s
/// `the_metering_floor_is_below_the_default_quantum`, an in-guest counter
/// read after each stop, three runs): a `stop` and `cont` back to back cost
/// 0.26–0.29 ms of host time (median) and let the guest live 0.14–0.15 ms
/// (0.12–0.16 ms, tenth to ninetieth percentile) with no window between
/// them — the floor — and a window of `w` from 0.25 ms to 5 ms lets it live
/// `w` to `w` + 0.03 ms. Through the node, 199 slices of 1 ms took 0.228 s
/// of host time (1.14 ms a slice) and the guest's counter said it lived
/// 199.35 ms of the board's 199 ms. One millisecond is the USB full-speed
/// frame period — the latency a real USB serial adapter has anyway — seven
/// times the floor, at an overhead of about a seventh of the quantum, which
/// a board far slower than real time hides.
pub const DEFAULT_QUANTUM: Duration = Duration::from_millis(1);

/// The longest quantum the node accepts.
///
/// Each slice holds the engine — and with it every other part — for a
/// quantum of host time, and the guest lags the board by up to a quantum.
/// A second is longer than any timeout a host's serial stack is likely to
/// hold against its port, so a longer one meters nothing a host can see.
pub const MAX_QUANTUM: Duration = Duration::from_secs(1);

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

/// Makes the guest a node starts with no guest: called once, at its first
/// slice, on the engine's thread with virtual time held (booting a VM and
/// warming it up on host time is nobody's simulated time). The error is
/// the reason, as the node's failure says it.
pub type Boot = Box<dyn FnOnce() -> Result<Box<dyn Guest>, String> + Send>;

/// Counters a test, a report or an operator reads while the node runs.
#[derive(Default)]
pub struct NodeStats {
    slices: AtomicU64,
    clocked: AtomicU64,
    guest_ns: AtomicU64,
    virtual_ns: AtomicU64,
    from_guest: AtomicU64,
    to_guest: AtomicU64,
    shed: AtomicU64,
    framing_errors: AtomicU64,
    disconnected: AtomicBool,
    booted: AtomicBool,
    boot_wall_ns: AtomicU64,
    boot_at_ns: AtomicU64,
    peak_lead_ns: AtomicU64,
    peak_overrun_ns: AtomicU64,
    unpowered: AtomicU64,
    failure: Mutex<Option<String>>,
}

impl fmt::Debug for NodeStats {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("NodeStats")
            .field("slices", &self.slices())
            .field("clocked", &self.clocked())
            .field("guest_ns", &self.guest_ns())
            .field("virtual_ns", &self.virtual_ns())
            .field("from_guest", &self.from_guest())
            .field("to_guest", &self.to_guest())
            .field("shed", &self.shed())
            .field("framing_errors", &self.framing_errors())
            .field("disconnected", &self.disconnected())
            .field("peak_lead_ns", &self.peak_lead_ns())
            .field("peak_overrun_ns", &self.peak_overrun_ns())
            .field("unpowered", &self.unpowered())
            .field("failure", &self.failure())
            .finish()
    }
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

    /// Time the guest has been allowed to run, in nanoseconds — its own
    /// clock's progress, for a hardware-virtualised guest.
    pub fn guest_ns(&self) -> u64 {
        self.guest_ns.load(Ordering::Relaxed)
    }

    /// Virtual time that has passed since the node started, in nanoseconds,
    /// as far as its last slice.
    pub fn virtual_ns(&self) -> u64 {
        self.virtual_ns.load(Ordering::Relaxed)
    }

    /// How far the board's clock is ahead of the guest's (negative: behind),
    /// as of the node's last slice.
    pub fn skew_ns(&self) -> i64 {
        self.virtual_ns() as i64 - self.guest_ns() as i64
    }

    /// The furthest the guest's clock has been ahead of the board's at the
    /// end of a slice, by the node's books, in nanoseconds.
    ///
    /// A slice ends when its `stop` takes, so the guest leaves every slice
    /// ahead by that slice's overrun ([`Self::peak_overrun_ns`]), and the
    /// node pays a lead back by not running the guest until the board has
    /// caught up. A `stop` that QEMU answers late — a loaded host, a retried
    /// round trip — leaves the guest that much ahead, and the board's bytes
    /// reach it that much later by the guest's own clock.
    pub fn peak_lead_ns(&self) -> u64 {
        self.peak_lead_ns.load(Ordering::Relaxed)
    }

    /// The most host time any one slice ran past its budget, by the node's
    /// stopwatch, in nanoseconds: the sleep's overshoot and the `stop`'s
    /// round trip, retries included.
    pub fn peak_overrun_ns(&self) -> u64 {
        self.peak_overrun_ns.load(Ordering::Relaxed)
    }

    /// Bytes the guest sent while its line's `VIO` read no voltage. A host
    /// with no rail sends nothing, so they were shed (and counted in
    /// [`Self::shed`] too); the first is logged as an error, since it is
    /// most often a line whose `VIO` and `GND` nobody wired.
    pub fn unpowered(&self) -> u64 {
        self.unpowered.load(Ordering::Relaxed)
    }

    /// Bytes the guest sent that reached the line.
    pub fn from_guest(&self) -> u64 {
        self.from_guest.load(Ordering::Relaxed)
    }

    /// Bytes delivered to the guest's port.
    pub fn to_guest(&self) -> u64 {
        self.to_guest.load(Ordering::Relaxed)
    }

    /// Bytes shed because a queue overflowed (the guest stopped reading its
    /// port, or wrote more than a megabyte the line has not carried yet),
    /// or because the guest sent while its line's rail read no voltage.
    /// Zero in a healthy run.
    pub fn shed(&self) -> u64 {
        self.shed.load(Ordering::Relaxed)
    }

    /// Frames the wire delivered that failed their stop bit.
    pub fn framing_errors(&self) -> u64 {
        self.framing_errors.load(Ordering::Relaxed)
    }

    /// Whether the guest's serial port went away (the process exited).
    pub fn disconnected(&self) -> bool {
        self.disconnected.load(Ordering::Relaxed)
    }

    /// How long the guest took to boot on host time, and the virtual
    /// instant it was frozen at when it had, once a node that booted its
    /// guest itself ([`QemuNode::booting`]) has.
    pub fn booted(&self) -> Option<(Duration, u64)> {
        self.booted.load(Ordering::Acquire).then(|| {
            (
                Duration::from_nanos(self.boot_wall_ns.load(Ordering::Relaxed)),
                self.boot_at_ns.load(Ordering::Relaxed),
            )
        })
    }

    /// Why the node stopped running its guest, once it has: the guest did
    /// not boot, failed mid-slice, or ran further ahead of the board than
    /// [`QemuNode::with_max_lead`] allows. No slice runs after it; the
    /// guest goes down when the node is dropped.
    pub fn failure(&self) -> Option<String> {
        self.failure
            .lock()
            .expect("the failure is never poisoned")
            .clone()
    }

    fn fail(&self, why: String, left: Left) {
        let left = match left {
            Left::Frozen => "the guest is left frozen",
            Left::MaybeRunning => {
                "the guest could not be frozen and may still be running until the node is dropped"
            }
            Left::NoGuest => "there is no guest",
        };
        tracing::error!(%why, "qemu node: {left}");
        let mut failure = self.failure.lock().expect("the failure is never poisoned");
        failure.get_or_insert(why);
    }
}

/// What a failure left of the guest, as the node's log says it: a guest
/// is frozen only once a `stop` has been answered.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Left {
    /// The last freeze was answered, or the guest was never thawed.
    Frozen,
    /// A thaw or a freeze failed: the guest's state is unknown.
    MaybeRunning,
    /// The guest was never made.
    NoGuest,
}

/// A slice that failed, and what it left of the guest.
struct SliceFailure {
    error: io::Error,
    left: Left,
}

/// The guest, or what makes it, shared between the node, its slices and
/// its [`LinkControl`].
#[derive(Default)]
struct Slot {
    guest: Option<Box<dyn Guest>>,
    boot: Option<Boot>,
}

type GuestSlot = Arc<Mutex<Slot>>;

/// A byte queue between two threads.
type ByteQueue = Arc<Mutex<VecDeque<u8>>>;

/// The guest's serial descriptor as the pump sees it, or -1 while there is
/// none (unplugged, or not booted yet).
///
/// The pump cannot ask the guest directly: a slice holds that lock for a
/// whole quantum, so a pump that locked per poll would stall a quantum at a
/// time. An atomic is the handoff.
type SerialFd = Arc<AtomicI32>;

/// The cable, as a thing a test can pull.
///
/// A handle onto one node's serial attachment and nothing else: the guest sits
/// behind the same mutex a slice holds, so a caller here cannot reach the rest
/// of it, and cannot resume or pause a guest the node is metering.
///
/// The timing works out for free. A slice locks the guest for exactly one
/// quantum and drops it, so a call made from any other thread waits for the
/// current slice to finish and then runs BETWEEN slices -- never against a
/// guest that is mid-run.
#[derive(Clone)]
pub struct LinkControl {
    guest: GuestSlot,
    fd: SerialFd,
}

impl fmt::Debug for LinkControl {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
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
            .guest
            .as_ref()
            .is_some_and(|g| g.serial_attached())
    }

    fn set(&self, attached: bool) -> io::Result<()> {
        let mut slot = self.guest.lock().expect("guest slot never poisoned");
        match slot.guest.as_mut() {
            Some(guest) => {
                guest.set_serial_attached(attached)?;
                // Publish the new descriptor before releasing the guest, so
                // the pump never polls one that has just been closed.
                self.fd.store(guest.serial_fd(), Ordering::Release);
                Ok(())
            }
            // Not booted yet, or the node was dropped and took the guest
            // with it. Saying so beats reporting success for a cable that
            // has no machine on the other end.
            None => Err(io::Error::new(
                io::ErrorKind::NotConnected,
                "the node has no running guest",
            )),
        }
    }
}

/// A computer on the board.
///
/// The pins are a host's serial line at its own rail
/// ([`HOST_RAIL_PINS`], [`HostRailLine`]), named from the computer's side,
/// as `host-serial`'s are: `TX` what it sends, driven at the sensed `VIO`
/// above the sensed `GND` and released while `VIO` reads no voltage; `RX`
/// what it hears, read through JESD8C.01's 0.8 V / 2.0 V pair against
/// `GND`. The guest's serial line stands for a host's adapter, which no
/// datasheet here describes: its receiver reads the pair a 3.3 V LVCMOS
/// input and a 5 V TTL input both take, and the project wires the rail the
/// adapter really has.
///
/// # Metering
///
/// The node meters the guest with its own wakes, through the one interface
/// every part has: in [`Component::start`] it asks to be woken one quantum
/// on, and at each wake it runs a **slice** — it lets the guest run for as
/// much host time as the board's clock has advanced since the last slice
/// and the guest has not yet lived, freezes it again over QMP, and asks to
/// be woken a quantum on. The slice runs on the engine's thread inside the
/// wake, so the engine cannot advance the board while the guest runs, and
/// the guest is frozen whenever the engine does. The guest's clock
/// therefore advances only while the board's does, at the same rate. A
/// closed loop carries the guest's owed time from slice to slice — a slice
/// the guest overran (a late `stop`) is paid back by not running it until
/// the board has caught up — and a guest that can read its own clock
/// ([`Guest::clock_ns`]) has each slice booked from it, so the books do not
/// drift: its clock lags the board's by at most a quantum and leads it by
/// at most the last slice's overrun ([`NodeStats::peak_lead_ns`],
/// [`QemuNode::with_max_lead`]). A guest whose clock cannot be read is
/// booked by the node's stopwatch, which runs from QEMU's answer to `cont`
/// to its answer to `stop`, not from the one taking to the other, and
/// drifts from the guest's clock without a bound the node can state.
///
/// Bytes the board sends while the guest is frozen wait in a queue and are
/// written to the guest's port at the start of its next slice; bytes the
/// guest sends during a slice enter the line at the slice's virtual instant,
/// in order, at the line's baud. A pump thread reads the guest's port at all
/// times: QEMU writes the guest's serial bytes to its socket *blocking*,
/// under its big lock, so a socket nobody reads would stall the vCPU and
/// with it the `stop` the next slice ends with.
pub struct QemuNode {
    framing: UartFraming,
    quantum_ns: u64,
    max_lead_ns: Option<u64>,
    guest: GuestSlot,
    shutdown: Arc<AtomicBool>,
    outbound: ByteQueue,
    inbound: ByteQueue,
    stats: Arc<NodeStats>,
    serial_fd: SerialFd,
    meter: Option<Arc<Meter>>,
    pump: Option<JoinHandle<()>>,
    started: bool,
}

impl fmt::Debug for QemuNode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("QemuNode")
            .field("quantum_ns", &self.quantum_ns)
            .field("started", &self.started)
            .field("stats", &self.stats)
            .finish()
    }
}

impl QemuNode {
    /// A node around a guest whose serial line runs 8N1 at `baud_hz`.
    pub fn new(guest: Box<dyn Guest>, baud_hz: u32) -> Self {
        Self::with_slot(
            Slot {
                guest: Some(guest),
                boot: None,
            },
            baud_hz,
        )
    }

    /// A node that makes its guest with `boot` at its first slice, on the
    /// engine's thread with virtual time held there: what a project's VM
    /// kinds build, so that `embsim check` boots nothing and a run boots
    /// the VM before the board's clock passes its first quantum. A boot
    /// that fails is the node's [`NodeStats::failure`].
    pub fn booting(boot: Boot, baud_hz: u32) -> Self {
        Self::with_slot(
            Slot {
                guest: None,
                boot: Some(boot),
            },
            baud_hz,
        )
    }

    fn with_slot(slot: Slot, baud_hz: u32) -> Self {
        Self {
            framing: UartFraming::new_8n1(baud_hz),
            quantum_ns: DEFAULT_QUANTUM.as_nanos() as u64,
            max_lead_ns: None,
            guest: Arc::new(Mutex::new(slot)),
            shutdown: Arc::new(AtomicBool::new(false)),
            outbound: Arc::new(Mutex::new(VecDeque::new())),
            inbound: Arc::new(Mutex::new(VecDeque::new())),
            stats: Arc::new(NodeStats::default()),
            serial_fd: Arc::new(AtomicI32::new(-1)),
            meter: None,
            pump: None,
            started: false,
        }
    }

    /// Set the quantum (default [`DEFAULT_QUANTUM`], clamped to
    /// [`MAX_QUANTUM`] and to at least a nanosecond). Shorter quanta bound
    /// the lag tighter and cost proportionally more stop/cont round trips.
    pub fn with_quantum(mut self, quantum: Duration) -> Self {
        let quantum = if quantum > MAX_QUANTUM {
            tracing::warn!(?quantum, ?MAX_QUANTUM, "qemu node: quantum clamped");
            MAX_QUANTUM
        } else {
            quantum
        };
        self.quantum_ns = quantum.as_nanos().max(1) as u64;
        self
    }

    /// The quantum, in nanoseconds of virtual time.
    pub fn quantum_ns(&self) -> u64 {
        self.quantum_ns
    }

    /// Stop the node, as a failure saying so, the first time the guest
    /// ends a slice further ahead of the board than `lead`
    /// ([`NodeStats::peak_lead_ns`]). By default no lead stops it: the
    /// node pays a lead back and reports the largest.
    pub fn with_max_lead(mut self, lead: Duration) -> Self {
        self.max_lead_ns = Some(lead.as_nanos().min(u128::from(u64::MAX)) as u64);
        self
    }

    /// The framing the line is clocked at.
    pub fn framing(&self) -> UartFraming {
        self.framing
    }

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

    /// The node's counters, readable from any thread.
    pub fn stats(&self) -> Arc<NodeStats> {
        Arc::clone(&self.stats)
    }
}

impl Component for QemuNode {
    fn pins(&self) -> &[PinDecl] {
        &HOST_RAIL_PINS
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let line = HostRailLine::attach(&io, self.framing, Arc::clone(&self.shutdown))?;
        let meter = Arc::new(Meter {
            quantum_ns: self.quantum_ns,
            max_lead_ns: self.max_lead_ns,
            guest: Arc::clone(&self.guest),
            line,
            io: io.clone(),
            outbound: Arc::clone(&self.outbound),
            inbound: Arc::clone(&self.inbound),
            stats: Arc::clone(&self.stats),
            serial_fd: Arc::clone(&self.serial_fd),
            shutdown: Arc::clone(&self.shutdown),
            books: Mutex::new(Books::default()),
        });
        {
            let meter = Arc::clone(&meter);
            let rx = io.pin("RX")?;
            io.on_sense("RX", move |sense| {
                if meter.shutdown.load(Ordering::Relaxed) {
                    return;
                }
                meter.deliver(meter.line.bridge().receive_sense(&rx, &sense));
            })?;
        }
        {
            let meter = Arc::clone(&meter);
            io.on_wake_ns(move |now_ns| meter.on_wake(now_ns));
        }
        self.meter = Some(meter);
        Ok(())
    }

    fn start(&mut self) {
        if std::mem::replace(&mut self.started, true) {
            tracing::error!("QemuNode::start: already started");
            return;
        }
        let Some(meter) = self.meter.clone() else {
            tracing::error!("QemuNode::start: not attached");
            return;
        };
        let fd = self
            .guest
            .lock()
            .expect("guest slot never poisoned")
            .guest
            .as_ref()
            .map_or(-1, |guest| guest.serial_fd());
        self.serial_fd.store(fd, Ordering::Release);

        // The pump: reads the guest's port for as long as the node lives.
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
                self.stats.fail(
                    format!("could not spawn the serial pump thread: {e}"),
                    Left::Frozen,
                );
                return;
            }
        }

        // Started with time held: the first slice is a quantum after the
        // instant the system starts, as every part's first wake is anchored.
        meter.arm_first(virtual_clock::virtual_ns());
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
        // Components drop after the engine is joined, so no slice is running
        // and none will: the guest is frozen, and taking it out of the slot is
        // safe. Dropping the guest is what shuts it down (a `QemuVm` quits
        // QEMU).
        let slot = std::mem::take(&mut *self.guest.lock().expect("guest slot never poisoned"));
        drop(slot);
    }
}

/// The node's metering books, kept between slices.
#[derive(Debug, Default)]
struct Books {
    /// The instant the next slice is due at; `None` before the node starts
    /// and after it fails.
    next_slice_ns: Option<u64>,
    /// The instant of the last slice (or of the start).
    last_virtual_ns: u64,
    /// Host time the guest is owed: virtual time that has passed minus the
    /// time it lived.
    owed_ns: i64,
    /// The guest's own clock at the last slice it was read in.
    last_clock_ns: Option<u64>,
    /// What the stopwatch booked since that reading: replaced by the exact
    /// span at the next one.
    booked_since_clock_ns: u64,
}

/// Everything a slice works with, shared by the node's wake and sense
/// callbacks once it is attached.
struct Meter {
    quantum_ns: u64,
    max_lead_ns: Option<u64>,
    guest: GuestSlot,
    line: HostRailLine,
    io: ComponentNetIo,
    outbound: ByteQueue,
    inbound: ByteQueue,
    stats: Arc<NodeStats>,
    serial_fd: SerialFd,
    shutdown: Arc<AtomicBool>,
    books: Mutex<Books>,
}

impl Meter {
    /// Anchor the books at `now_ns` and arm the first slice a quantum on.
    fn arm_first(&self, now_ns: u64) {
        let next = now_ns + self.quantum_ns;
        {
            let mut books = self.books.lock().expect("the books are never poisoned");
            books.last_virtual_ns = now_ns;
            books.next_slice_ns = Some(next);
        }
        self.io.schedule_at_ns(next);
    }

    /// The node's one wake handler: the line's bit instants and frame
    /// deadlines, and the slices. A wake is a slice when one is due; the
    /// bridge dedups its own instants, and a slice already run at an
    /// instant moves the next one on, so a repeat at an instant is only the
    /// line's.
    fn on_wake(&self, now_ns: u64) {
        if self.shutdown.load(Ordering::Relaxed) {
            return;
        }
        self.deliver(self.line.bridge().service(now_ns));
        let due = self
            .books
            .lock()
            .expect("the books are never poisoned")
            .next_slice_ns
            .is_some_and(|at| now_ns >= at);
        if due {
            self.slice(now_ns);
        }
    }

    /// One slice, at `now_ns`: the guest runs for what it is owed, the
    /// board's clock held where it is.
    fn slice(&self, now_ns: u64) {
        let mut books = self.books.lock().expect("the books are never poisoned");
        let advanced = now_ns.saturating_sub(books.last_virtual_ns);
        books.last_virtual_ns = now_ns;
        self.stats.virtual_ns.fetch_add(advanced, Ordering::Relaxed);
        books.owed_ns += advanced as i64;
        // What the guest sent since the last slice goes on the line at this
        // slice's instant.
        self.feed_bridge();
        if books.owed_ns > 0 {
            let budget = Duration::from_nanos(books.owed_ns as u64);
            let outcome = {
                let mut slot = self.guest.lock().expect("guest slot never poisoned");
                match self.ready(&mut slot, now_ns) {
                    Ok(guest) => run_slice(guest.as_mut(), &self.outbound, &self.stats, budget)
                        .map_err(|failure| {
                            (
                                format!("the guest failed mid-slice: {}", failure.error),
                                failure.left,
                            )
                        }),
                    Err(why) => Err((why, Left::NoGuest)),
                }
            };
            // Whatever the guest sent during its slice goes on the line at
            // the same virtual instant: the engine has not moved.
            self.feed_bridge();
            match outcome {
                Ok((lived, clock)) => self.book(&mut books, lived, clock),
                Err((why, left)) => {
                    books.next_slice_ns = None;
                    self.stats.fail(why, left);
                    return;
                }
            }
            // The guest left the slice when its `stop` took: ahead of the
            // board by what it outlived its budget, which the slices that
            // follow pay back by not running it.
            let lead =
                (self.stats.guest_ns() as i64 - self.stats.virtual_ns() as i64).max(0) as u64;
            self.stats.peak_lead_ns.fetch_max(lead, Ordering::Relaxed);
            if let Some(max) = self.max_lead_ns.filter(|&max| lead > max) {
                books.next_slice_ns = None;
                self.stats.fail(
                    format!(
                        "the guest ended a slice {} ahead of the board, past the {} the node \
                         allows: the slice's `stop` took that long to be answered",
                        span(lead),
                        span(max),
                    ),
                    Left::Frozen,
                );
                return;
            }
        }
        let next = now_ns + self.quantum_ns;
        books.next_slice_ns = Some(next);
        drop(books);
        self.io.schedule_at_ns(next);
    }

    /// The guest, booted first if the node makes its own.
    fn ready<'s>(&self, slot: &'s mut Slot, now_ns: u64) -> Result<&'s mut Box<dyn Guest>, String> {
        if slot.guest.is_none() {
            let boot = slot
                .boot
                .take()
                .ok_or_else(|| "the node has no guest".to_string())?;
            let started = Instant::now();
            let guest = boot().map_err(|why| format!("the guest did not boot: {why}"))?;
            self.serial_fd.store(guest.serial_fd(), Ordering::Release);
            slot.guest = Some(guest);
            self.stats
                .boot_wall_ns
                .store(started.elapsed().as_nanos() as u64, Ordering::Relaxed);
            self.stats.boot_at_ns.store(now_ns, Ordering::Relaxed);
            self.stats.booted.store(true, Ordering::Release);
            tracing::info!(
                boot = ?started.elapsed(),
                at_ns = now_ns,
                "qemu node: guest booted on host time; the board's clock meters it from here"
            );
        }
        Ok(slot.guest.as_mut().expect("booted above"))
    }

    /// Book a slice the guest lived `lived` of by the stopwatch, its own
    /// clock reading `clock` at the slice's start when it has one.
    fn book(&self, books: &mut Books, lived: Duration, clock: Option<u64>) {
        let lived_ns = lived.as_nanos() as u64;
        match clock {
            Some(clock) => {
                if let Some(previous) = books.last_clock_ns {
                    // The guest's own span since the last reading replaces
                    // what the stopwatch booked for it: whatever latency the
                    // reading has, it has at both ends and cancels.
                    let exact = clock.saturating_sub(previous);
                    let delta = exact as i64 - books.booked_since_clock_ns as i64;
                    books.owed_ns -= delta;
                    if delta >= 0 {
                        self.stats
                            .guest_ns
                            .fetch_add(delta as u64, Ordering::Relaxed);
                    } else {
                        self.stats
                            .guest_ns
                            .fetch_sub(delta.unsigned_abs(), Ordering::Relaxed);
                    }
                }
                books.last_clock_ns = Some(clock);
                books.booked_since_clock_ns = lived_ns;
                self.stats.clocked.fetch_add(1, Ordering::Relaxed);
            }
            None => books.booked_since_clock_ns += lived_ns,
        }
        books.owed_ns -= lived_ns as i64;
        self.stats.guest_ns.fetch_add(lived_ns, Ordering::Relaxed);
        self.stats.slices.fetch_add(1, Ordering::Relaxed);
    }

    /// Move what the pump has read onto the line, as much as the bridge has
    /// room for, once the engine has read the line's rail: until then the
    /// bytes wait, as a PTY host's wait in its PTY. Called only on the
    /// engine's thread, so the first bit is anchored at the current virtual
    /// instant; the rest waits for the line to drain.
    fn feed_bridge(&self) {
        if !self.line.rail_known() {
            return;
        }
        let bridge: &SerialLevelBridge = self.line.bridge();
        let mut queue = self.inbound.lock().expect("inbound queue never poisoned");
        let take = queue.len().min(bridge.tx_room());
        if take == 0 {
            return;
        }
        let chunk: Vec<u8> = queue.drain(..take).collect();
        drop(queue);
        let shed = bridge.transmit(&chunk);
        self.stats
            .from_guest
            .fetch_add((chunk.len() - shed) as u64, Ordering::Relaxed);
        if shed > 0 {
            // The rail reads no voltage: a host with no rail sends nothing.
            self.stats.shed.fetch_add(shed as u64, Ordering::Relaxed);
            if self
                .stats
                .unpowered
                .fetch_add(shed as u64, Ordering::Relaxed)
                == 0
            {
                tracing::error!(
                    shed,
                    "qemu node: the guest sent while its line's VIO read no voltage, and \
                     its bytes were shed; a host's line is driven from its own rail, so \
                     wire the host's I/O rail to VIO and its return to GND"
                );
            }
        }
    }

    /// Queue deframed bytes for the guest.
    ///
    /// Runs on the engine thread, so it must not block and must not drop:
    /// the guest is usually frozen when the board speaks, and a frame dropped
    /// here looks to the protocol above like corruption, not like a slow
    /// host.
    fn deliver(&self, frames: Vec<Result<u8, FramingError>>) {
        if frames.is_empty() {
            return;
        }
        let mut queue = self.outbound.lock().expect("outbound queue never poisoned");
        for frame in frames {
            match frame {
                Ok(byte) => queue.push_back(byte),
                Err(error) => {
                    self.stats.framing_errors.fetch_add(1, Ordering::Relaxed);
                    tracing::debug!(?error, "qemu node: frame dropped (bad framing on the wire)");
                }
            }
        }
        if queue.len() > OUTBOUND_MAX {
            let excess = queue.len() - OUTBOUND_MAX;
            queue.drain(..excess);
            self.stats.shed.fetch_add(excess as u64, Ordering::Relaxed);
        }
    }
}

/// Run the guest for `budget` of host time, writing it the board's bytes
/// meanwhile. Returns how long it ran by the node's stopwatch, and the
/// guest's own clock reading at the start of the run if it offers one; or
/// the failure, explained by the guest ([`Guest::explain`]), and whether a
/// `stop` was answered after it.
fn run_slice(
    guest: &mut dyn Guest,
    outbound: &Mutex<VecDeque<u8>>,
    stats: &NodeStats,
    budget: Duration,
) -> Result<(Duration, Option<u64>), SliceFailure> {
    let failed = |guest: &mut dyn Guest, error: io::Error, left: Left| SliceFailure {
        error: guest.explain(error),
        left,
    };
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
        if let Err(e) = drain_outbound(fd, outbound, stats) {
            // Not thawed yet: still frozen.
            return Err(failed(guest, e, Left::Frozen));
        }
    }
    if let Err(e) = guest.resume() {
        // The thaw may have landed without its answer: freeze it again if
        // it can be.
        let left = if guest.pause().is_ok() {
            Left::Frozen
        } else {
            Left::MaybeRunning
        };
        return Err(failed(guest, e, left));
    }
    let start = Instant::now();
    let clock = guest.clock_ns();
    let outcome = hold_slice(fd, outbound, stats, start, budget);
    let pause = guest.pause();
    let lived = start.elapsed();
    stats.peak_overrun_ns.fetch_max(
        lived.saturating_sub(budget).as_nanos() as u64,
        Ordering::Relaxed,
    );
    match (outcome, pause) {
        (Ok(()), Ok(())) => Ok((lived, clock)),
        (Err(e), pause) => {
            let left = if pause.is_ok() {
                Left::Frozen
            } else {
                Left::MaybeRunning
            };
            Err(failed(guest, e, left))
        }
        (Ok(()), Err(e)) => Err(failed(guest, e, Left::MaybeRunning)),
    }
}

/// A span of time as a report prints it: `1.250 ms`.
fn span(ns: u64) -> String {
    format!("{:.3} ms", ns as f64 / 1e6)
}

/// The body of a slice: write the guest its bytes until the budget is spent.
///
/// Nothing reaches `outbound` while a slice runs — the board's bytes are
/// queued on the engine's thread, which is running this — so once the queue
/// is written the rest of the budget is a plain wait: the guest's own host
/// time, which is what the slice is. A full socket is waited on with
/// `poll(2)`, whose millisecond timeout can overrun the budget by under a
/// millisecond; the owed-time loop takes that back next slice.
fn hold_slice(
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
        let want_out = fd >= 0
            && !outbound
                .lock()
                .expect("outbound queue never poisoned")
                .is_empty();
        if !want_out {
            // The guest's slice of host time, not a simulated wait: the
            // board's clock is held while it passes.
            thread::sleep(remaining);
            return Ok(());
        }
        let mut pollfd = libc::pollfd {
            fd,
            events: libc::POLLOUT,
            revents: 0,
        };
        let whole_ms =
            remaining.as_millis() + u128::from(!remaining.subsec_nanos().is_multiple_of(1_000_000));
        let timeout_ms = whole_ms.min(i32::MAX as u128) as i32;
        // SAFETY: `pollfd` is a valid, initialised array of one element for
        // the duration of the call, and `fd` is owned by the guest, which
        // outlives this slice.
        let ready = unsafe { libc::poll(&mut pollfd, 1, timeout_ms) };
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
        // Re-read every pass. The descriptor appears when the guest boots,
        // changes when the cable is pulled and again when it is put back,
        // and a reader holding the old one would poll a closed fd, get
        // POLLNVAL and declare the guest dead.
        let fd = serial_fd.load(Ordering::Acquire);
        if fd < 0 {
            // Unplugged or not booted, not gone. Wait for it rather than
            // ending the pump; the node has to survive the outage for a
            // reconnect test to have anything to reconnect to.
            thread::sleep(Duration::from_millis(PUMP_POLL_MS as u64));
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
