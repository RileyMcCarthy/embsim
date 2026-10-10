//! The component: the host's Chrome on a host's serial pins, its pages'
//! clocks metered by the board's.

use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::fmt;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use serde_json::{json, Value};

use embsim_board::uart::{FramingError, UartFraming};
use embsim_board::{
    AttachError, Component, ComponentNetIo, HostRailLine, PinDecl, SerialLevelBridge,
    HOST_RAIL_PINS,
};
use embsim_core::virtual_clock;

use crate::base64;
use crate::browser::{Browse, Browser, Drained, Granted};

/// The default quantum: how far the board's clock runs between two slices.
///
/// The quantum is the latency a byte can wait before the page may react to
/// it, and the most the two clocks are apart, so shorter is better until a
/// slice's round trips dominate. Measured on an Apple M2 with Chrome 153
/// (hc_design, the shipped MaD app, load average 17–24): a grant and the
/// clock read cost 0.42 + 0.29 ms of host time, about 0.72 ms a slice
/// (median), 1.1 ms (90th percentile). One millisecond is the USB
/// full-speed frame period, the latency a USB serial adapter has anyway.
pub const DEFAULT_QUANTUM: Duration = Duration::from_millis(1);

/// The longest quantum the node accepts: a page lags the board by up to a
/// quantum, and a second is longer than any timeout a page is likely to
/// hold against its port.
pub const MAX_QUANTUM: Duration = Duration::from_secs(1);

/// How long one grant may hold before the run fails as stuck, by default.
/// Chrome holds a `pauseIfNetworkFetchesPending` budget while a fetch is in
/// flight and while a task runs (a 1.5 s fetch held a 1 ms grant for 1.5 s,
/// hc_antiCdp), so the bound is seconds, not a quantum.
pub const DEFAULT_STUCK_AFTER: Duration = Duration::from_secs(30);

/// How long the drain barrier waits for a page's consumer to come back for
/// more before it lets the slice go on. It waits only while a reader holds
/// the port's stream; a consumer that reads in a way the probe does not see
/// (`pipeTo`, a transform stream) is let go after [`DRAIN_STRIKES`] waits
/// in a row that ran out.
pub const DRAIN_BOUND: Duration = Duration::from_secs(1);

/// Barrier waits in a row that run out before the barrier is turned off for
/// that page, and the report says so.
pub const DRAIN_STRIKES: u32 = 3;

/// Bytes held for a page that is not reading yet, or for the line. A
/// megabyte is five seconds of a 2 Mbaud line; past it the oldest are shed,
/// counted.
const QUEUE_MAX: usize = 1 << 20;

/// A page clock that goes back, or forward by this much more than it was
/// granted in one slice, has a new origin (a navigation to a new renderer
/// process): the node re-anchors its books there and counts it.
const DISCONTINUITY_MS: f64 = 1_000.0;

/// The serial port's identity on a USB bus, as Chrome's `getInfo()` reports
/// it.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct UsbIds {
    /// `usbVendorId`, if the adapter has one.
    pub vendor: Option<u16>,
    /// `usbProductId`, if the adapter has one.
    pub product: Option<u16>,
}

/// What the node is set to: how it reaches Chrome and how it meters it.
#[derive(Debug, Clone)]
pub struct Settings {
    /// Launch Chrome or attach to one.
    pub browse: Browse,
    /// The quantum.
    pub quantum: Duration,
    /// Fail the run once a page leads the board by more than this.
    pub max_lead: Option<Duration>,
    /// How long a grant may hold before the run fails as stuck.
    pub stuck_after: Duration,
    /// A page to open in the first tab, once it is held.
    pub url: Option<String>,
    /// The port's USB identity.
    pub usb: UsbIds,
    /// Whether every origin may use the port without a prompt, as Chrome's
    /// `SerialAllowUsbDevicesForUrls` policy grants it.
    pub granted: bool,
}

impl Settings {
    /// Settings for `browse`, metered at the defaults.
    pub fn new(browse: Browse) -> Self {
        Self {
            browse,
            quantum: DEFAULT_QUANTUM,
            max_lead: None,
            stuck_after: DEFAULT_STUCK_AFTER,
            url: None,
            usb: UsbIds::default(),
            granted: false,
        }
    }
}

/// Host time per slice, in 10 µs buckets to 100 ms.
const BUCKET_NS: u64 = 10_000;
const BUCKETS: usize = 10_000;

/// Counters a test, a report or an operator reads while the node runs.
pub struct NodeStats {
    slices: AtomicU64,
    clocked: AtomicU64,
    skipped: AtomicU64,
    granted_ns: AtomicU64,
    board_ns: AtomicU64,
    lived_ns: AtomicU64,
    peak_lead_ns: AtomicU64,
    peak_overrun_ns: AtomicU64,
    reanchored: AtomicU64,
    stuck: AtomicU64,
    from_page: AtomicU64,
    to_page: AtomicU64,
    shed: AtomicU64,
    unpowered: AtomicU64,
    unheard: AtomicU64,
    flushed: AtomicU64,
    framing_errors: AtomicU64,
    mismatched_opens: AtomicU64,
    mismatched_bytes: AtomicU64,
    drain_waits: AtomicU64,
    drain_timeouts: AtomicU64,
    drain_off: AtomicU64,
    unplugs: AtomicU64,
    plugs: AtomicU64,
    booted: AtomicBool,
    boot_wall_ns: AtomicU64,
    boot_at_ns: AtomicU64,
    host: Mutex<Vec<u32>>,
    said: Mutex<Said>,
    failure: Mutex<Option<String>>,
}

/// What only the report reads: what it has been told once.
#[derive(Debug, Default, Clone)]
pub(crate) struct Said {
    pub version: String,
    pub endpoint: String,
    pub pages: u64,
    pub workers: u64,
    pub workers_unmetered: u64,
    pub workers_unprobed: u64,
    pub mismatch: Option<String>,
}

impl Default for NodeStats {
    fn default() -> Self {
        Self {
            slices: AtomicU64::new(0),
            clocked: AtomicU64::new(0),
            skipped: AtomicU64::new(0),
            granted_ns: AtomicU64::new(0),
            board_ns: AtomicU64::new(0),
            lived_ns: AtomicU64::new(0),
            peak_lead_ns: AtomicU64::new(0),
            peak_overrun_ns: AtomicU64::new(0),
            reanchored: AtomicU64::new(0),
            stuck: AtomicU64::new(0),
            from_page: AtomicU64::new(0),
            to_page: AtomicU64::new(0),
            shed: AtomicU64::new(0),
            unpowered: AtomicU64::new(0),
            unheard: AtomicU64::new(0),
            flushed: AtomicU64::new(0),
            framing_errors: AtomicU64::new(0),
            mismatched_opens: AtomicU64::new(0),
            mismatched_bytes: AtomicU64::new(0),
            drain_waits: AtomicU64::new(0),
            drain_timeouts: AtomicU64::new(0),
            drain_off: AtomicU64::new(0),
            unplugs: AtomicU64::new(0),
            plugs: AtomicU64::new(0),
            booted: AtomicBool::new(false),
            boot_wall_ns: AtomicU64::new(0),
            boot_at_ns: AtomicU64::new(0),
            host: Mutex::new(vec![0; BUCKETS + 1]),
            said: Mutex::new(Said::default()),
            failure: Mutex::new(None),
        }
    }
}

impl fmt::Debug for NodeStats {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("NodeStats")
            .field("slices", &self.slices())
            .field("clocked", &self.clocked())
            .field("skipped", &self.skipped())
            .field("granted_ns", &self.granted_ns())
            .field("board_ns", &self.board_ns())
            .field("lived_ns", &self.lived_ns())
            .field("peak_lead_ns", &self.peak_lead_ns())
            .field("peak_overrun_ns", &self.peak_overrun_ns())
            .field("from_page", &self.from_page())
            .field("to_page", &self.to_page())
            .field("shed", &self.shed())
            .field("unheard", &self.unheard())
            .field("drain_waits", &self.drain_waits())
            .field("drain_timeouts", &self.drain_timeouts())
            .field("failure", &self.failure())
            .finish()
    }
}

macro_rules! counter {
    ($($(#[$doc:meta])* $name:ident),* $(,)?) => {
        $(
            $(#[$doc])*
            pub fn $name(&self) -> u64 {
                self.$name.load(Ordering::Relaxed)
            }
        )*
    };
}

impl NodeStats {
    counter! {
        /// Slices run: one a quantum, from the first.
        slices,
        /// Slices whose page clock the node read and booked from.
        clocked,
        /// Grants skipped because a page was already ahead of the board:
        /// a lead paid back.
        skipped,
        /// Virtual time granted to pages, summed, in nanoseconds.
        granted_ns,
        /// The board's time since the first page was booked, at the last
        /// slice, in nanoseconds.
        board_ns,
        /// What that page's own clock says it lived over the same span.
        lived_ns,
        /// The furthest a page's clock has been ahead of the board's at a
        /// slice, in nanoseconds. Chrome moves a page's clock outside a
        /// budget at a worker's birth and at storage calls (hc_antiCdp);
        /// the node pays a lead back by skipping grants.
        peak_lead_ns,
        /// The most a page's clock passed its budget in one slice, in
        /// nanoseconds.
        peak_overrun_ns,
        /// Times a page's clock jumped (backwards, or more than a second
        /// past its budget) and the books were re-anchored.
        reanchored,
        /// Grants that did not expire within the bound (the run fails at
        /// the first).
        stuck,
        /// Bytes the page sent that reached the line.
        from_page,
        /// Bytes the board sent that were handed to the page.
        to_page,
        /// Bytes shed: a queue past its bound, a page writing to a port it
        /// does not hold, a line with no rail, or a port opened at a rate
        /// or framing other than the line's.
        shed,
        /// Bytes the page sent while its line's `VIO` read no voltage
        /// (also in [`Self::shed`]).
        unpowered,
        /// Bytes the board sent while no page had the port open, or the
        /// cable was out.
        unheard,
        /// Bytes discarded by a flush the page asked for (a cancelled
        /// read, an aborted write, a close).
        flushed,
        /// Frames the wire delivered that failed their stop bit.
        framing_errors,
        /// Times a page opened the port at a rate or framing other than
        /// the line's.
        mismatched_opens,
        /// Bytes shed because of such an open (also in [`Self::shed`]).
        mismatched_bytes,
        /// Slices that waited for a page's consumer to read what it was
        /// handed before the next grant.
        drain_waits,
        /// Of those, waits that ran out.
        drain_timeouts,
        /// Pages whose barrier was turned off after waits that ran out.
        drain_off,
        /// Cable pulls.
        unplugs,
        /// Cable re-inserts.
        plugs,
    }

    /// How a page's clock stands against the board's at the last slice,
    /// in nanoseconds: positive, the page is ahead.
    pub fn lead_ns(&self) -> i64 {
        self.lived_ns() as i64 - self.board_ns() as i64
    }

    /// How long reaching Chrome took on host time, and the virtual instant
    /// the board was held at meanwhile, once it has.
    pub fn booted(&self) -> Option<(Duration, u64)> {
        self.booted.load(Ordering::Acquire).then(|| {
            (
                Duration::from_nanos(self.boot_wall_ns.load(Ordering::Relaxed)),
                self.boot_at_ns.load(Ordering::Relaxed),
            )
        })
    }

    /// Host time per slice at the `p`th quantile (0.5 the median), to
    /// 10 µs; `None` before a slice has run.
    pub fn host_per_slice(&self, p: f64) -> Option<Duration> {
        let host = self.host.lock().expect("the histogram is never poisoned");
        let total: u64 = host.iter().map(|&n| u64::from(n)).sum();
        if total == 0 {
            return None;
        }
        let rank = ((p.clamp(0.0, 1.0) * total as f64).ceil() as u64).max(1);
        let mut seen = 0;
        for (bucket, &n) in host.iter().enumerate() {
            seen += u64::from(n);
            if seen >= rank {
                return Some(Duration::from_nanos((bucket as u64 + 1) * BUCKET_NS));
            }
        }
        None
    }

    fn record_host(&self, took: Duration) {
        let bucket = ((took.as_nanos() as u64) / BUCKET_NS).min(BUCKETS as u64) as usize;
        let mut host = self.host.lock().expect("the histogram is never poisoned");
        host[bucket] = host[bucket].saturating_add(1);
    }

    /// Why the node stopped, once it has: Chrome could not be reached, a
    /// grant stuck, a page led past the bound, or the browser went away.
    pub fn failure(&self) -> Option<String> {
        self.failure
            .lock()
            .expect("the failure is never poisoned")
            .clone()
    }

    pub(crate) fn said(&self) -> Said {
        self.said.lock().expect("never poisoned").clone()
    }

    fn fail(&self, why: String) {
        tracing::error!(%why, "chrome-cdp: the node stopped");
        self.failure
            .lock()
            .expect("the failure is never poisoned")
            .get_or_insert(why);
    }

    fn add(&self, counter: &AtomicU64, n: u64) {
        counter.fetch_add(n, Ordering::Relaxed);
    }
}

/// A cable pull or re-insert, done at the next slice boundary.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LinkOp {
    /// Pull the cable: the port goes away.
    Unplug,
    /// Put it back: the port comes back as a new `SerialPort`.
    Plug,
}

/// The cable, as a thing a test can pull: the same operation a page's
/// `__embsim.link('unplug')` asks for, done at the node's next slice.
#[derive(Debug, Clone)]
pub struct LinkControl {
    ops: Arc<Mutex<VecDeque<LinkOp>>>,
}

impl LinkControl {
    /// Pull the cable at the next slice.
    pub fn unplug(&self) {
        self.ops
            .lock()
            .expect("never poisoned")
            .push_back(LinkOp::Unplug);
    }

    /// Put it back at the next slice.
    pub fn plug(&self) {
        self.ops
            .lock()
            .expect("never poisoned")
            .push_back(LinkOp::Plug);
    }
}

/// The host's Chrome on the board: Web Serial in its pages, its far end
/// this line, every page's clock and its dedicated workers' metered by the
/// board's over the Chrome DevTools Protocol.
///
/// The pins are a host's serial line at its own rail ([`HOST_RAIL_PINS`],
/// [`HostRailLine`]), named from the host's side as `host-serial`'s are,
/// so a project swaps one kind for the other without touching a wire.
///
/// # Metering
///
/// The node meters with its own wakes. In [`Component::start`] it asks to
/// be woken a quantum on; at each wake that is a slice it runs one on the
/// engine's thread, the board held there:
///
/// 1. Chrome is launched or attached at the first slice, and every page
///    and dedicated worker is held from birth (`Target.setAutoAttach` with
///    `waitForDebuggerOnStart`): a page's clock paused, the binding and the
///    Web Serial shim in before its first script; a worker on the page's
///    clock (`advance` with a budget that never runs out).
/// 2. Each page is granted what it is owed — the board's time since the
///    page was first booked, less what the page's own clock says it lived
///    — as a `pauseIfNetworkFetchesPending` budget, and the node waits for
///    `virtualTimeBudgetExpired`; a grant that does not expire within
///    [`Settings::stuck_after`] fails the run, saying why.
/// 3. What the page sent during the grant goes onto the line at this
///    instant; what it asked of the port is answered.
/// 4. One `Runtime.evaluate` hands the page the board's bytes and the
///    answers, and reads its clock, which the node books from: a page
///    ahead of the board is paid back by skipping its grants, and a lead
///    past [`Settings::max_lead`] fails the run.
/// 5. The drain barrier: when bytes were handed to a page whose port's
///    stream is held, the node waits until the page's consumer — in the
///    page, or in a dedicated worker the stream was transferred to — has
///    called `read()` again, before the next grant.
/// 6. The next wake is armed a quantum on.
///
/// JavaScript runs in no virtual time; the page's clock advances only in
/// grants. A page's bytes reach the line at most a quantum after it sent
/// them by its clock (two when they cross from a worker), and the board's
/// reach the page at most a quantum after they left the wire.
pub struct CdpNode {
    framing: UartFraming,
    baud_hz: u32,
    settings: Settings,
    shutdown: Arc<AtomicBool>,
    stats: Arc<NodeStats>,
    ops: Arc<Mutex<VecDeque<LinkOp>>>,
    meter: Option<Arc<Meter>>,
    started: bool,
}

impl fmt::Debug for CdpNode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("CdpNode")
            .field("framing", &self.framing)
            .field("settings", &self.settings)
            .field("started", &self.started)
            .finish_non_exhaustive()
    }
}

impl CdpNode {
    /// A node whose line runs 8N1 at `baud_hz`, set as `settings` says.
    pub fn new(settings: Settings, baud_hz: u32) -> Self {
        let quantum = settings.quantum.clamp(Duration::from_nanos(1), MAX_QUANTUM);
        Self {
            framing: UartFraming::new_8n1(baud_hz),
            baud_hz,
            settings: Settings {
                quantum,
                ..settings
            },
            shutdown: Arc::new(AtomicBool::new(false)),
            stats: Arc::new(NodeStats::default()),
            ops: Arc::new(Mutex::new(VecDeque::new())),
            meter: None,
            started: false,
        }
    }

    /// The node's counters, readable from any thread.
    pub fn stats(&self) -> Arc<NodeStats> {
        Arc::clone(&self.stats)
    }

    /// The cable.
    pub fn link(&self) -> LinkControl {
        LinkControl {
            ops: Arc::clone(&self.ops),
        }
    }

    /// The framing the line is clocked at.
    pub fn framing(&self) -> UartFraming {
        self.framing
    }

    /// The settings it runs with.
    pub fn settings(&self) -> &Settings {
        &self.settings
    }
}

impl Component for CdpNode {
    fn pins(&self) -> &[PinDecl] {
        &HOST_RAIL_PINS
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let line = HostRailLine::attach(&io, self.framing, Arc::clone(&self.shutdown))?;
        let capacity = line.bridge().tx_room();
        let meter = Arc::new(Meter {
            baud_hz: self.baud_hz,
            settings: self.settings.clone(),
            quantum_ns: self.settings.quantum.as_nanos() as u64,
            line,
            capacity,
            io: io.clone(),
            stats: Arc::clone(&self.stats),
            ops: Arc::clone(&self.ops),
            shutdown: Arc::clone(&self.shutdown),
            rx: Mutex::new(RxSide::default()),
            state: Mutex::new(State::default()),
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
            tracing::error!("CdpNode::start: already started");
            return;
        }
        let Some(meter) = self.meter.clone() else {
            tracing::error!("CdpNode::start: not attached");
            return;
        };
        // Started with time held: the first slice is a quantum after the
        // instant the system starts.
        meter.arm_first(virtual_clock::virtual_ns());
    }
}

impl Drop for CdpNode {
    fn drop(&mut self) {
        self.shutdown.store(true, Ordering::Relaxed);
        // Components drop after the engine is joined, so no slice runs; the
        // browser (and a Chrome the node launched) goes with the state.
        if let Some(meter) = self.meter.take() {
            let browser = meter
                .state
                .lock()
                .expect("the state is never poisoned")
                .browser
                .take();
            drop(browser);
        }
    }
}

// ============================================================
// The metering
// ============================================================

/// Whether, and how, the board's bytes are heard.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
enum Listening {
    /// No page has the port open, or the cable is out.
    #[default]
    Nobody,
    /// A page has it open at the line's rate and framing.
    Open,
    /// A page has it open at another rate or framing.
    Mismatched,
}

/// The board's side of the receive path, shared with the engine's sense
/// callbacks.
#[derive(Debug, Default)]
struct RxSide {
    listening: Listening,
    /// Bytes for the page that holds the port, not handed over yet.
    hold: VecDeque<u8>,
}

/// Why the port stopped being held.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Released {
    /// The page closed it, or its document went.
    Closed,
    /// The cable was pulled.
    Unplugged,
}

/// The page document that has the port open.
#[derive(Debug, Clone)]
struct Owner {
    page: String,
    doc: String,
    matches_line: bool,
    buffer_size: u64,
    reading: bool,
}

/// One page's books: its clock against the board's.
#[derive(Debug, Default, Clone, Copy)]
struct Books {
    /// The board instant and the page's clock (ms) it was first booked at.
    anchor: Option<(u64, f64)>,
    /// What the page has lived since its anchor, by its clock.
    lived_ns: u64,
    /// The budget granted this slice, in nanoseconds.
    granted_ns: u64,
}

/// One page's barrier.
#[derive(Debug, Default, Clone, Copy)]
struct Barrier {
    misses: u32,
    off: bool,
}

/// The node's state between slices, on the engine's thread.
#[derive(Debug, Default)]
struct State {
    browser: Option<Browser>,
    next_slice_ns: Option<u64>,
    books: BTreeMap<String, Books>,
    barriers: BTreeMap<String, Barrier>,
    /// What each page is answered at its next slice.
    replies: BTreeMap<String, Vec<Value>>,
    plugged: bool,
    gen: u32,
    owner: Option<Owner>,
    granted: BTreeSet<String>,
    /// Bytes the page sent, waiting for room on the line.
    tx: VecDeque<u8>,
    /// Drains (`WritableStream.close()`) waiting for the line to go quiet:
    /// page, document, request.
    drains: Vec<(String, String, u64)>,
}

struct Meter {
    baud_hz: u32,
    settings: Settings,
    quantum_ns: u64,
    line: HostRailLine,
    /// The bridge's transmit queue when empty.
    capacity: usize,
    io: ComponentNetIo,
    stats: Arc<NodeStats>,
    ops: Arc<Mutex<VecDeque<LinkOp>>>,
    shutdown: Arc<AtomicBool>,
    rx: Mutex<RxSide>,
    state: Mutex<State>,
}

/// A request's answer.
fn ok(id: u64, v: Value) -> Value {
    json!({ "id": id, "ok": true, "v": v })
}

fn refused(id: u64, name: &str, message: &str) -> Value {
    json!({ "id": id, "ok": false, "name": name, "message": message })
}

const OPEN_ERROR: &str = "Failed to open serial port.";
const NOT_SELECTED: &str = "No port selected by the user.";

impl Meter {
    fn arm_first(&self, now_ns: u64) {
        let next = now_ns + self.quantum_ns;
        {
            let mut state = self.state.lock().expect("the state is never poisoned");
            state.next_slice_ns = Some(next);
            state.plugged = true;
        }
        self.io.schedule_at_ns(next);
    }

    /// The node's one wake handler: the line's bit instants and frame
    /// deadlines, and the slices.
    fn on_wake(&self, now_ns: u64) {
        if self.shutdown.load(Ordering::Relaxed) {
            return;
        }
        self.deliver(self.line.bridge().service(now_ns));
        let due = self
            .state
            .lock()
            .expect("the state is never poisoned")
            .next_slice_ns
            .is_some_and(|at| now_ns >= at);
        if due {
            self.slice(now_ns);
        }
    }

    /// Deframed bytes from the board: held for the page that has the port
    /// open, or counted as unheard.
    fn deliver(&self, frames: Vec<Result<u8, FramingError>>) {
        if frames.is_empty() {
            return;
        }
        let mut rx = self.rx.lock().expect("never poisoned");
        for frame in frames {
            match frame {
                Ok(byte) => match rx.listening {
                    Listening::Open => rx.hold.push_back(byte),
                    Listening::Mismatched => {
                        self.stats.add(&self.stats.shed, 1);
                        self.stats.add(&self.stats.mismatched_bytes, 1);
                    }
                    Listening::Nobody => self.stats.add(&self.stats.unheard, 1),
                },
                Err(error) => {
                    self.stats.add(&self.stats.framing_errors, 1);
                    tracing::debug!(
                        ?error,
                        "chrome-cdp: frame dropped (bad framing on the wire)"
                    );
                }
            }
        }
        if rx.hold.len() > QUEUE_MAX {
            let excess = rx.hold.len() - QUEUE_MAX;
            rx.hold.drain(..excess);
            self.stats.add(&self.stats.shed, excess as u64);
        }
    }

    /// One slice at `now_ns`.
    fn slice(&self, now_ns: u64) {
        let started = Instant::now();
        let mut state = self.state.lock().expect("the state is never poisoned");
        let booting = state.browser.is_none();
        let outcome = self.run_slice(&mut state, now_ns);
        match outcome {
            Ok(()) => {
                self.stats.add(&self.stats.slices, 1);
                // The boot's host time is reported apart (`booted`).
                if !booting {
                    self.stats.record_host(started.elapsed());
                }
                let next = now_ns + self.quantum_ns;
                state.next_slice_ns = Some(next);
                drop(state);
                self.io.schedule_at_ns(next);
            }
            Err(why) => {
                state.next_slice_ns = None;
                self.stats.fail(why);
            }
        }
    }

    fn run_slice(&self, state: &mut State, now_ns: u64) -> Result<(), String> {
        if state.browser.is_none() {
            state.browser = Some(self.boot(now_ns)?);
        }
        {
            let browser = state.browser.as_mut().expect("booted above");
            if let Err(e) = browser.drain_socket() {
                return Err(self.gone(browser, e));
            }
        }

        // 1. Each page is granted what it is owed.
        let sessions: Vec<String> = browser_of(state).pages.keys().cloned().collect();
        let mut budgets = Vec::new();
        for session in &sessions {
            let books = state.books.entry(session.clone()).or_default();
            books.granted_ns = 0;
            let Some((anchor_ns, _)) = books.anchor else {
                continue;
            };
            let owed = now_ns.saturating_sub(anchor_ns) as i64 - books.lived_ns as i64;
            if owed > 0 {
                books.granted_ns = owed as u64;
                budgets.push((session.clone(), owed as f64 / 1e6));
            } else {
                self.stats.add(&self.stats.skipped, 1);
            }
        }
        if !budgets.is_empty() {
            let browser = state.browser.as_mut().expect("booted above");
            let granted = match browser.grant(&budgets, self.settings.stuck_after) {
                Ok(granted) => granted,
                Err(e) => return Err(self.gone(browser, e)),
            };
            if let Granted::Stuck { session, budget_ms } = granted {
                self.stats.add(&self.stats.stuck, 1);
                return Err(format!(
                    "a grant stuck: the page{} was granted {budget_ms:.3} ms of virtual time and \
                     its budget did not expire within {:.1} s of host time; Chrome holds a \
                     budget while a network fetch is in flight and while a task runs, so a \
                     fetch that never completes or a task that never ends holds it",
                    page_url(browser, &session),
                    self.settings.stuck_after.as_secs_f64()
                ));
            }
            for (session, _) in &budgets {
                if let Some(books) = state.books.get(session) {
                    self.stats.add(&self.stats.granted_ns, books.granted_ns);
                }
            }
        }
        if let Some(page) = browser_of(state).pages.values().find(|page| page.crashed) {
            return Err(format!(
                "a page crashed{}",
                page_url_of(page.origin.as_deref())
            ));
        }

        // 2. What the pages said, and the cable, at this instant.
        let gone = std::mem::take(&mut state.browser.as_mut().expect("booted above").gone);
        for session in gone {
            state.books.remove(&session);
            state.barriers.remove(&session);
            state.replies.remove(&session);
            if state.owner.as_ref().is_some_and(|o| o.page == session) {
                self.release(state, Released::Closed);
            }
        }
        let inboxes: Vec<(String, Vec<Value>)> = state
            .browser
            .as_mut()
            .expect("booted above")
            .pages
            .values_mut()
            .map(|page| (page.session.clone(), std::mem::take(&mut page.inbox)))
            .collect();
        let ops: Vec<LinkOp> = self.ops.lock().expect("never poisoned").drain(..).collect();
        for op in ops {
            self.link(state, op);
        }
        for (session, inbox) in inboxes {
            for message in inbox {
                self.take(state, &session, message);
            }
        }

        // 3. The page's bytes onto the line; drains that have gone quiet.
        self.feed_line(state);
        if state.tx.is_empty() && self.line.bridge().tx_idle() {
            for (page, _doc, id) in std::mem::take(&mut state.drains) {
                state
                    .replies
                    .entry(page)
                    .or_default()
                    .push(ok(id, Value::Null));
            }
        }

        // 4. Each page's side of the slice, and 5. the barrier.
        let sessions: Vec<String> = browser_of(state).pages.keys().cloned().collect();
        let mut clocked = false;
        for session in sessions {
            clocked |= self.page_slice(state, &session, now_ns)?;
        }
        if clocked {
            self.stats.add(&self.stats.clocked, 1);
        }
        let browser = browser_of(state);
        let mut said = self.stats.said.lock().expect("never poisoned");
        said.pages = browser.pages_seen;
        said.workers = browser.workers_seen;
        said.workers_unmetered = browser.workers_unmetered;
        said.workers_unprobed = browser.workers_unprobed;
        Ok(())
    }

    /// Reach Chrome, the board held at `now_ns`.
    fn boot(&self, now_ns: u64) -> Result<Browser, String> {
        let started = Instant::now();
        let config = json!({
            "usbVendorId": self.settings.usb.vendor,
            "usbProductId": self.settings.usb.product,
        });
        let shim = crate::SHIM.replace("__EMBSIM_CONFIG__", &config.to_string());
        let mut browser = Browser::boot(&self.settings.browse, shim)
            .map_err(|why| format!("Chrome could not be reached: {why}"))?;
        if let Some(url) = &self.settings.url {
            let deadline = Instant::now() + Duration::from_secs(5);
            while browser.pages.is_empty() && Instant::now() < deadline {
                browser
                    .pump(Instant::now() + Duration::from_millis(20))
                    .map_err(|e| self.gone(&browser, e))?;
            }
            browser.open(url)?;
        }
        self.stats
            .boot_wall_ns
            .store(started.elapsed().as_nanos() as u64, Ordering::Relaxed);
        self.stats.boot_at_ns.store(now_ns, Ordering::Relaxed);
        {
            let mut said = self.stats.said.lock().expect("never poisoned");
            said.version = browser.version.clone();
            said.endpoint = browser.endpoint.clone();
        }
        self.stats.booted.store(true, Ordering::Release);
        tracing::info!(
            took = ?started.elapsed(),
            at_ns = now_ns,
            version = %browser.version,
            "chrome-cdp: Chrome reached on host time; the board's clock meters it from here"
        );
        Ok(browser)
    }

    /// Why a DevTools failure stops the node: what Chrome did, if it was
    /// ours and exited.
    fn gone(&self, browser: &Browser, e: crate::devtools::CdpError) -> String {
        let exited = browser
            .process
            .as_ref()
            .is_some_and(|process| process.leader_exited());
        if exited {
            format!("Chrome exited mid-run ({e})")
        } else {
            format!("Chrome stopped answering over DevTools: {e}")
        }
    }

    /// Pull or re-insert the cable.
    fn link(&self, state: &mut State, op: LinkOp) {
        match op {
            LinkOp::Unplug if state.plugged => {
                state.plugged = false;
                self.stats.add(&self.stats.unplugs, 1);
                self.release(state, Released::Unplugged);
            }
            LinkOp::Plug if !state.plugged => {
                state.plugged = true;
                state.gen += 1;
                self.stats.add(&self.stats.plugs, 1);
            }
            _ => {}
        }
    }

    /// The port is no longer held: what waits each way is discarded — a
    /// closing port flushes both ways ([`Released::Closed`]), and a cable
    /// that is out carries nothing ([`Released::Unplugged`]).
    fn release(&self, state: &mut State, why: Released) {
        state.owner = None;
        let tx = state.tx.len() as u64;
        state.tx.clear();
        state.drains.clear();
        let mut rx = self.rx.lock().expect("never poisoned");
        let held = rx.hold.len() as u64;
        rx.hold.clear();
        rx.listening = Listening::Nobody;
        match why {
            Released::Closed => self.stats.add(&self.stats.flushed, tx + held),
            Released::Unplugged => {
                self.stats.add(&self.stats.shed, tx);
                self.stats.add(&self.stats.unheard, held);
            }
        }
    }

    /// One thing a page said through its binding.
    fn take(&self, state: &mut State, session: &str, message: Value) {
        let doc = message["doc"].as_str().unwrap_or_default().to_string();
        let current = state
            .browser
            .as_ref()
            .and_then(|b| b.pages.get(session))
            .and_then(|page| page.doc.clone());
        match message["k"].as_str() {
            Some("hello") => {
                // A new document: a port the old one held is closed with it.
                if state
                    .owner
                    .as_ref()
                    .is_some_and(|o| o.page == session && o.doc != doc)
                {
                    self.release(state, Released::Closed);
                }
                state.replies.remove(session);
                state.barriers.remove(session);
                state.drains.retain(|(page, _, _)| page != session);
            }
            Some("link") => match message["op"].as_str() {
                Some("unplug") => self.link(state, LinkOp::Unplug),
                Some("plug") => self.link(state, LinkOp::Plug),
                _ => {}
            },
            _ if current.as_deref() != Some(doc.as_str()) => {
                // A document that has gone: nothing it asked is answered,
                // and what it wrote goes nowhere.
                if message["k"] == "tx" {
                    let n = base64::decode(message["b"].as_str().unwrap_or_default()).len();
                    self.stats.add(&self.stats.shed, n as u64);
                }
            }
            Some("reading") => {
                if let Some(owner) = state
                    .owner
                    .as_mut()
                    .filter(|o| o.page == session && o.doc == doc)
                {
                    owner.reading = true;
                }
            }
            Some("tx") => {
                let bytes = base64::decode(message["b"].as_str().unwrap_or_default());
                let n = bytes.len() as u64;
                match &state.owner {
                    Some(owner) if owner.page == session && owner.doc == doc && state.plugged => {
                        if owner.matches_line {
                            state.tx.extend(bytes);
                            if state.tx.len() > QUEUE_MAX {
                                let excess = state.tx.len() - QUEUE_MAX;
                                state.tx.drain(..excess);
                                self.stats.add(&self.stats.shed, excess as u64);
                            }
                        } else {
                            self.stats.add(&self.stats.shed, n);
                            self.stats.add(&self.stats.mismatched_bytes, n);
                        }
                    }
                    _ => self.stats.add(&self.stats.shed, n),
                }
            }
            Some("req") => {
                let id = message["id"].as_u64().unwrap_or(0);
                if let Some(reply) = self.request(state, session, &doc, id, &message) {
                    state
                        .replies
                        .entry(session.to_string())
                        .or_default()
                        .push(reply);
                }
            }
            _ => {}
        }
    }

    /// Answer a request, or `None` while it waits (a drain).
    fn request(
        &self,
        state: &mut State,
        session: &str,
        doc: &str,
        id: u64,
        m: &Value,
    ) -> Option<Value> {
        let origin = m["origin"].as_str().unwrap_or_default().to_string();
        let holds = state
            .owner
            .as_ref()
            .is_some_and(|o| o.page == session && o.doc == doc);
        Some(match m["op"].as_str().unwrap_or_default() {
            "getPorts" => {
                let permitted = self.settings.granted || state.granted.contains(&origin);
                if state.plugged && permitted {
                    ok(id, json!([state.gen]))
                } else {
                    ok(id, json!([]))
                }
            }
            "requestPort" => {
                let usb = self.settings.usb;
                let matches = |f: &Value| {
                    !f["bluetooth"].as_bool().unwrap_or(false)
                        && f["usbVendorId"]
                            .as_u64()
                            .is_none_or(|v| usb.vendor.map(u64::from) == Some(v))
                        && f["usbProductId"]
                            .as_u64()
                            .is_none_or(|p| usb.product.map(u64::from) == Some(p))
                };
                let filters = m["filters"].as_array().cloned().unwrap_or_default();
                if state.plugged && (filters.is_empty() || filters.iter().any(matches)) {
                    state.granted.insert(origin);
                    ok(id, json!(state.gen))
                } else {
                    refused(id, "NotFoundError", NOT_SELECTED)
                }
            }
            "open" => {
                let gen = m["gen"].as_u64().unwrap_or(u64::MAX);
                if !state.plugged || gen != u64::from(state.gen) || state.owner.is_some() {
                    refused(id, "NetworkError", OPEN_ERROR)
                } else {
                    let baud = m["baudRate"].as_u64().unwrap_or(0);
                    let data_bits = m["dataBits"].as_u64().unwrap_or(8);
                    let stop_bits = m["stopBits"].as_u64().unwrap_or(1);
                    let parity = m["parity"].as_str().unwrap_or("none").to_string();
                    let matches_line = baud == u64::from(self.baud_hz)
                        && data_bits == 8
                        && stop_bits == 1
                        && parity == "none";
                    if !matches_line {
                        self.stats.add(&self.stats.mismatched_opens, 1);
                        let said = format!(
                            "a page opened the port at {baud} baud, {data_bits} data bits, \
                             parity {parity}, {stop_bits} stop bit{}; the line runs at {} baud \
                             8N1, so what crosses it is shed",
                            if stop_bits == 1 { "" } else { "s" },
                            self.baud_hz
                        );
                        tracing::error!("chrome-cdp: {said}");
                        self.stats.said.lock().expect("never poisoned").mismatch = Some(said);
                    }
                    state.owner = Some(Owner {
                        page: session.to_string(),
                        doc: doc.to_string(),
                        matches_line,
                        buffer_size: m["bufferSize"].as_u64().unwrap_or(255).max(1),
                        reading: false,
                    });
                    let mut rx = self.rx.lock().expect("never poisoned");
                    rx.hold.clear();
                    rx.listening = if matches_line {
                        Listening::Open
                    } else {
                        Listening::Mismatched
                    };
                    ok(id, Value::Null)
                }
            }
            "close" => {
                if holds {
                    self.release(state, Released::Closed);
                }
                ok(id, Value::Null)
            }
            "flush" => {
                if holds {
                    if m["dir"] == "rx" {
                        let mut rx = self.rx.lock().expect("never poisoned");
                        self.stats.add(&self.stats.flushed, rx.hold.len() as u64);
                        rx.hold.clear();
                        if let Some(owner) = state.owner.as_mut() {
                            owner.reading = false;
                        }
                    } else {
                        self.stats.add(&self.stats.flushed, state.tx.len() as u64);
                        state.tx.clear();
                    }
                }
                ok(id, Value::Null)
            }
            "drain" => {
                if holds {
                    state
                        .drains
                        .push((session.to_string(), doc.to_string(), id));
                    return None;
                }
                ok(id, Value::Null)
            }
            "setSignals" => ok(id, Value::Null),
            "getSignals" => ok(
                id,
                // No pin carries DCD, CTS, RI or DSR: an adapter's inputs
                // that are wired to nothing read inactive.
                json!({
                    "dataCarrierDetect": false,
                    "clearToSend": false,
                    "ringIndicator": false,
                    "dataSetReady": false,
                }),
            ),
            "forget" => {
                state.granted.remove(&origin);
                ok(id, Value::Null)
            }
            other => refused(
                id,
                "NotSupportedError",
                &format!("embsim's port does not do {other:?}"),
            ),
        })
    }

    /// Move the page's bytes onto the line, as much as it has room for,
    /// once the engine has read the line's rail.
    fn feed_line(&self, state: &mut State) {
        if !self.line.rail_known() || state.tx.is_empty() {
            return;
        }
        let bridge: &SerialLevelBridge = self.line.bridge();
        let take = state.tx.len().min(bridge.tx_room());
        if take == 0 {
            return;
        }
        let chunk: Vec<u8> = state.tx.drain(..take).collect();
        let shed = bridge.transmit(&chunk);
        self.stats
            .add(&self.stats.from_page, (chunk.len() - shed) as u64);
        if shed > 0 {
            self.stats.add(&self.stats.shed, shed as u64);
            if self
                .stats
                .unpowered
                .fetch_add(shed as u64, Ordering::Relaxed)
                == 0
            {
                tracing::error!(
                    shed,
                    "chrome-cdp: the page sent while its line's VIO read no voltage, and its \
                     bytes were shed; a host's line is driven from its own rail, so wire the \
                     host's I/O rail to VIO and its return to GND"
                );
            }
        }
    }

    /// One page's side of the slice. Whether its clock was read.
    fn page_slice(&self, state: &mut State, session: &str, now_ns: u64) -> Result<bool, String> {
        let (doc, origin) = {
            let browser = state.browser.as_ref().expect("booted above");
            let Some(page) = browser.pages.get(session) else {
                return Ok(false);
            };
            (page.doc.clone(), page.origin.clone())
        };
        let owner = state
            .owner
            .clone()
            .filter(|o| o.page == session && Some(&o.doc) == doc.as_ref());
        // The board's bytes, for the document that holds the port and is
        // reading it.
        let rx_bytes: Vec<u8> = match &owner {
            Some(o) if o.reading => {
                let mut rx = self.rx.lock().expect("never poisoned");
                rx.hold.drain(..).collect()
            }
            _ => Vec::new(),
        };
        let backlog = state.tx.len() + self.capacity.saturating_sub(self.line.bridge().tx_room());
        let window = owner.as_ref().map_or(255, |o| {
            let per_quantum = self.baud_hz as u64 * self.quantum_ns / 1_000_000_000 / 10;
            o.buffer_size.max(2 * per_quantum)
        });
        let granted = self.settings.granted
            || origin
                .as_ref()
                .is_some_and(|origin| state.granted.contains(origin));
        let arg = json!({
            "gen": state.gen,
            "plugged": state.plugged,
            "granted": granted,
            "window": window,
            "backlog": backlog,
            "replies": state.replies.remove(session).unwrap_or_default(),
            "rx": if rx_bytes.is_empty() { Value::Null } else { Value::String(base64::encode(&rx_bytes)) },
        });
        let browser = state.browser.as_mut().expect("booted above");
        let base = browser.reads(session);
        let answer = match browser.slice(session, &arg) {
            Ok(answer) => answer,
            Err(e) => return Err(self.gone(browser, e)),
        };
        let Some(answer) = answer else {
            // No document to evaluate in: the answers and the bytes wait
            // for the next slice.
            if let Some(replies) = arg["replies"].as_array().filter(|r| !r.is_empty()) {
                let queued = state.replies.entry(session.to_string()).or_default();
                let later = std::mem::take(queued);
                queued.extend(replies.iter().cloned());
                queued.extend(later);
            }
            if !rx_bytes.is_empty() {
                let mut rx = self.rx.lock().expect("never poisoned");
                for byte in rx_bytes.into_iter().rev() {
                    rx.hold.push_front(byte);
                }
            }
            let books = state.books.entry(session.to_string()).or_default();
            books.lived_ns += books.granted_ns;
            return Ok(false);
        };
        if let Some(error) = answer["error"].as_str() {
            tracing::error!(%error, "chrome-cdp: the page's shim failed a slice");
        }
        self.stats.add(&self.stats.to_page, rx_bytes.len() as u64);
        if let Some(t) = answer["t"].as_f64() {
            self.book(state, session, t, now_ns)?;
        }
        if let Some(owner) = state
            .owner
            .as_mut()
            .filter(|o| o.page == session && Some(&o.doc) == doc.as_ref())
        {
            owner.reading = answer["reading"].as_bool().unwrap_or(owner.reading);
        }
        // The barrier: the bytes taken, and the consumer back for more.
        let reads = answer["reads"].as_u64().unwrap_or(0);
        let locked = answer["locked"].as_bool().unwrap_or(false);
        let barrier = state.barriers.entry(session.to_string()).or_default();
        if !rx_bytes.is_empty() && locked && reads > 0 && !barrier.off {
            let doc = doc.unwrap_or_default();
            self.stats.add(&self.stats.drain_waits, 1);
            let browser = state.browser.as_mut().expect("booted above");
            let drained = match browser.drain(session, &doc, base + reads, DRAIN_BOUND) {
                Ok(drained) => drained,
                Err(e) => return Err(self.gone(browser, e)),
            };
            let barrier = state.barriers.entry(session.to_string()).or_default();
            match drained {
                Drained::Met | Drained::Released => barrier.misses = 0,
                Drained::TimedOut => {
                    self.stats.add(&self.stats.drain_timeouts, 1);
                    barrier.misses += 1;
                    if barrier.misses >= DRAIN_STRIKES {
                        barrier.off = true;
                        self.stats.add(&self.stats.drain_off, 1);
                        tracing::warn!(
                            "chrome-cdp: a page's consumer did not come back for more in {} \
                             waits in a row; the drain barrier is off for that page (it reads \
                             the port in a way the probe does not see: pipeTo, a transform)",
                            DRAIN_STRIKES
                        );
                    }
                }
            }
        }
        Ok(true)
    }

    /// Book a page's clock reading `t_ms` at board instant `now_ns`.
    fn book(&self, state: &mut State, session: &str, t_ms: f64, now_ns: u64) -> Result<(), String> {
        let first = state
            .books
            .iter()
            .filter_map(|(s, b)| b.anchor.map(|(at, _)| (at, s.clone())))
            .min()
            .map(|(_, s)| s);
        let books = state.books.entry(session.to_string()).or_default();
        let Some((anchor_ns, anchor_ms)) = books.anchor else {
            books.anchor = Some((now_ns, t_ms));
            books.lived_ns = 0;
            return Ok(());
        };
        let before = books.lived_ns;
        let lived_ms = t_ms - anchor_ms;
        let advanced_ms = lived_ms - before as f64 / 1e6;
        let granted_ms = books.granted_ns as f64 / 1e6;
        if advanced_ms < 0.0 || advanced_ms > granted_ms + DISCONTINUITY_MS {
            // A new origin for the page's clock: book it level with the
            // board from here.
            self.stats.add(&self.stats.reanchored, 1);
            let board = now_ns.saturating_sub(anchor_ns);
            books.anchor = Some((anchor_ns, t_ms - board as f64 / 1e6));
            books.lived_ns = board;
            return Ok(());
        }
        books.lived_ns = (lived_ms * 1e6).round().max(0.0) as u64;
        let overrun = (advanced_ms - granted_ms) * 1e6;
        if overrun > 0.0 {
            self.stats
                .peak_overrun_ns
                .fetch_max(overrun.round() as u64, Ordering::Relaxed);
        }
        let board = now_ns.saturating_sub(anchor_ns);
        let lead = books.lived_ns as i64 - board as i64;
        if first.as_deref().is_none_or(|s| s == session) {
            self.stats.board_ns.store(board, Ordering::Relaxed);
            self.stats.lived_ns.store(books.lived_ns, Ordering::Relaxed);
        }
        if lead > 0 {
            self.stats
                .peak_lead_ns
                .fetch_max(lead as u64, Ordering::Relaxed);
            if let Some(max) = self
                .settings
                .max_lead
                .filter(|max| lead as u128 > max.as_nanos())
            {
                return Err(format!(
                    "a page's clock ran {} ahead of the board, past the {} the node allows: \
                     Chrome moves a page's clock outside its budget at a worker's birth and at \
                     storage calls, and the node pays a lead back only by skipping grants",
                    span(lead as u64),
                    span(max.as_nanos() as u64)
                ));
            }
        }
        Ok(())
    }
}

/// The browser, once booted.
fn browser_of(state: &State) -> &Browser {
    state.browser.as_ref().expect("booted at the first slice")
}

/// `" at http://…"`, the page's origin as an error names it.
fn page_url(browser: &Browser, session: &str) -> String {
    page_url_of(
        browser
            .pages
            .get(session)
            .and_then(|page| page.origin.as_deref()),
    )
}

fn page_url_of(origin: Option<&str>) -> String {
    origin.map(|o| format!(" at {o}")).unwrap_or_default()
}

/// A span as a report prints it: `1.250 ms`.
pub(crate) fn span(ns: u64) -> String {
    format!("{:.3} ms", ns as f64 / 1e6)
}
