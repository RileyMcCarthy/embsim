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
use crate::browser::{Browse, Browser, Drained, Granted, PageTarget};
use crate::devtools::CdpError;

/// The default quantum: how far the board's clock runs between two slices.
///
/// The quantum is the latency a byte can wait before the page may react to
/// it, and the most the two clocks are apart, so shorter is better until a
/// slice's round trips dominate. A grant and the clock read cost about
/// 0.72 ms of host time a slice (median) and 1.1 ms (90th percentile) with
/// the shipped MaD app on an Apple M2 (`NODES.md` §15, evidence E1). One
/// millisecond is the USB full-speed frame period, the latency a USB serial
/// adapter has anyway.
pub const DEFAULT_QUANTUM: Duration = Duration::from_millis(1);

/// The longest quantum the node accepts: a page lags the board by up to a
/// quantum, and a second is longer than any timeout a page is likely to
/// hold against its port.
pub const MAX_QUANTUM: Duration = Duration::from_secs(1);

/// How long one grant may hold before the run fails as stuck, by default.
/// Chrome holds a `pauseIfNetworkFetchesPending` budget while a fetch is in
/// flight and while a task runs (a 1.5 s fetch held a 1 ms grant for 1.5 s,
/// `NODES.md` §15, evidence E6), so the bound is seconds, not a quantum.
pub const DEFAULT_STUCK_AFTER: Duration = Duration::from_secs(30);

/// How long the drain barrier waits for a worker that owns the port's
/// stream to come back for more, by default, before it lets the slice go
/// on. A worker that reads in a way the probe does not see (`pipeTo`, a
/// transform stream) is let go after [`DRAIN_STRIKES`] waits in a row that
/// ran out.
pub const DRAIN_BOUND: Duration = Duration::from_secs(1);

/// Barrier waits in a row that run out before the barrier is turned off for
/// that page, and the report says so.
pub const DRAIN_STRIKES: u32 = 3;

/// Bytes held for a page that is not reading yet, or for the line, at
/// least. A megabyte is five seconds of a 2 Mbaud line; past it (or past
/// two writer windows, whichever is more) the oldest are shed, counted.
const QUEUE_MAX: usize = 1 << 20;

/// A clock's resolution, in ms: Chrome clamps `performance.now()` to 5 µs
/// in a cross-origin-isolated document and to 100 µs in any other.
const ISOLATED_RESOLUTION_MS: f64 = 0.005;
const RESOLUTION_MS: f64 = 0.1;

/// How long the node watches for a Chrome it launched to exit, once its
/// socket failed, before it says which.
const EXIT_GRACE: Duration = Duration::from_millis(500);

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
/// Made with [`Settings::new`]; [`CdpNode::new`] refuses what cannot run.
#[derive(Debug, Clone)]
#[non_exhaustive]
pub struct Settings {
    /// Launch Chrome or attach to one.
    pub browse: Browse,
    /// The quantum: more than zero, at most [`MAX_QUANTUM`].
    pub quantum: Duration,
    /// Fail the run once a page leads the board by more than this (more
    /// than zero).
    pub max_lead: Option<Duration>,
    /// How long a grant, or a page's side of a slice, may hold before the
    /// run fails as stuck (more than zero).
    pub stuck_after: Duration,
    /// How long the drain barrier waits for a worker to come back for more
    /// (more than zero).
    pub drain_bound: Duration,
    /// A page to open once it is held: in a launched Chrome's first tab, or
    /// in a new window of a browser the node attached to.
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
            drain_bound: DRAIN_BOUND,
            url: None,
            usb: UsbIds::default(),
            granted: false,
        }
    }
}

/// Why [`CdpNode::new`] refused its settings.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SettingsError(String);

impl fmt::Display for SettingsError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for SettingsError {}

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
    shared: AtomicU64,
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
    signals: AtomicU64,
    flow_control_opens: AtomicU64,
    drain_waits: AtomicU64,
    drain_timeouts: AtomicU64,
    drain_off: AtomicU64,
    unplugs: AtomicU64,
    plugs: AtomicU64,
    late_at_attach: AtomicU64,
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
    pub workers_serial: u64,
    pub mismatch: Option<String>,
    pub signals: Option<String>,
    pub flow_control: Option<String>,
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
            shared: AtomicU64::new(0),
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
            signals: AtomicU64::new(0),
            flow_control_opens: AtomicU64::new(0),
            drain_waits: AtomicU64::new(0),
            drain_timeouts: AtomicU64::new(0),
            drain_off: AtomicU64::new(0),
            unplugs: AtomicU64::new(0),
            plugs: AtomicU64::new(0),
            late_at_attach: AtomicU64::new(0),
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
            .field("reanchored", &self.reanchored())
            .field("shared", &self.shared())
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
        /// What that page lived over the same span, booked from the time
        /// it was granted and its clock's jumps past it.
        lived_ns,
        /// The furthest a page's clock has been ahead of the board's at a
        /// slice, in nanoseconds. Chrome moves a page's clock outside a
        /// budget at a worker's birth and at storage calls (`NODES.md` §15,
        /// evidence E5); the node pays a lead back by skipping grants.
        peak_lead_ns,
        /// The most a page's clock passed its budget in one slice, in
        /// nanoseconds, beyond the clock's resolution.
        peak_overrun_ns,
        /// Times a page's books were set level with the board again: at
        /// each new document (a navigation, a reload), and wherever a
        /// page's clock went back.
        reanchored,
        /// Slices in which pages that share one clock (a page and a window
        /// it opened, in one renderer) were granted once between them.
        shared,
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
        /// `setSignals` calls that asserted DTR, RTS or a break, which no
        /// pin of the line carries.
        signals,
        /// Opens that asked for hardware (RTS/CTS) flow control, which no
        /// pin of the line carries.
        flow_control_opens,
        /// Slices that waited for a worker that owns the port's stream to
        /// read what it was handed before the next grant.
        drain_waits,
        /// Of those, waits that ran out.
        drain_timeouts,
        /// Pages whose barrier was turned off after waits that ran out.
        drain_off,
        /// Cable pulls.
        unplugs,
        /// Cable re-inserts.
        plugs,
        /// Documents already open when the node reached the browser, whose
        /// scripts ran before the shim was installed in them.
        late_at_attach,
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

    /// Pages the node has held so far.
    pub fn pages_held(&self) -> u64 {
        self.said().pages
    }

    /// Dedicated workers the node has held so far.
    pub fn workers_held(&self) -> u64 {
        self.said().workers
    }

    /// Dedicated workers that asked for their own `navigator.serial`,
    /// which is Chrome's: the board's port is the page's.
    pub fn workers_with_serial(&self) -> u64 {
        self.said().workers_serial
    }

    /// The browser's DevTools HTTP endpoint (`http://127.0.0.1:PORT`), once
    /// the node has reached it: where a harness connects
    /// (`connectOverCDP`).
    pub fn devtools_endpoint(&self) -> Option<String> {
        let said = self.said();
        (!said.endpoint.is_empty()).then_some(said.endpoint)
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
/// 2. Each clock is granted what its pages are owed — the board's time
///    since a page's document was first booked, less what the page has
///    lived since — as a `pauseIfNetworkFetchesPending` budget, and the
///    node waits for `virtualTimeBudgetExpired`; pages that share one
///    clock (one renderer's main thread) are granted once between them. A
///    grant that does not expire within [`Settings::stuck_after`] fails
///    the run, saying why.
/// 3. What the page sent during the grant goes onto the line at this
///    instant; what it asked of the port is answered.
/// 4. One `Runtime.evaluate` hands the page the board's bytes and the
///    answers, and reads its clock. A page lives exactly what it is
///    granted, unless its clock passed the budget by more than its
///    resolution: then that lead is booked, paid back by skipping its
///    grants, and a lead past [`Settings::max_lead`] fails the run.
/// 5. The drain barrier: when bytes were handed to a page whose port's
///    stream was transferred to a dedicated worker, the node waits until
///    the worker has called `read()` again, before the next grant.
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
    /// A node whose line runs 8N1 at `baud_hz`, set as `settings` says;
    /// refused, saying why, when a setting cannot run: a zero rate, a
    /// quantum of zero or past [`MAX_QUANTUM`], or a zero bound.
    pub fn new(settings: Settings, baud_hz: u32) -> Result<Self, SettingsError> {
        let refuse = |why: &str| Err(SettingsError(why.to_string()));
        if baud_hz == 0 {
            return refuse("the line's rate is 0 baud; a line has a rate");
        }
        if settings.quantum.is_zero() || settings.quantum > MAX_QUANTUM {
            return Err(SettingsError(format!(
                "the quantum is {:?}; it is more than 0 and at most {:?}",
                settings.quantum, MAX_QUANTUM
            )));
        }
        if settings.max_lead.is_some_and(|lead| lead.is_zero()) {
            return refuse("max_lead is 0; Chrome moves a clock outside its budget now and then");
        }
        if settings.stuck_after.is_zero() {
            return refuse("stuck_after is 0; a grant takes some host time");
        }
        if settings.drain_bound.is_zero() {
            return refuse("drain_bound is 0; a worker takes some host time to read");
        }
        Ok(Self {
            framing: UartFraming::new_8n1(baud_hz),
            baud_hz,
            settings,
            shutdown: Arc::new(AtomicBool::new(false)),
            stats: Arc::new(NodeStats::default()),
            ops: Arc::new(Mutex::new(VecDeque::new())),
            meter: None,
            started: false,
        })
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
    /// The page closed it, or its document went, or it forgot the port.
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

/// One page's books: its clock against the board's, for one document.
#[derive(Debug, Default, Clone)]
struct Books {
    /// The document they are kept for: the shim's id, or the clock's
    /// origin in a document with no shim.
    doc: Option<String>,
    /// The board instant the page was first booked at.
    anchor_ns: Option<u64>,
    /// The page's clock (its document's `performance.now()`, ms) at the
    /// last read.
    last_ms: f64,
    /// What the page has lived since its anchor: the time it was granted,
    /// and its clock's jumps past a budget.
    lived_ns: u64,
    /// The budget its clock was granted this slice, in nanoseconds.
    granted_ns: u64,
}

impl Books {
    /// What the page is owed at `now_ns`: positive, it is behind.
    fn owed(&self, now_ns: u64) -> Option<i64> {
        self.anchor_ns
            .map(|anchor| now_ns.saturating_sub(anchor) as i64 - self.lived_ns as i64)
    }
}

/// A page clock read at a slice.
#[derive(Debug, Clone)]
struct Reading {
    /// `performance.now()`, ms.
    t_ms: f64,
    /// The document it is read in.
    doc: String,
    /// The clock's resolution, ms.
    resolution_ms: f64,
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

/// Why a task that never ends holds a page, as a failure says it.
const NEVER_ENDS: &str = "a task that never ends holds it (a loop that waits on the page's own \
                          clock never ends, since the clock does not move while a task runs)";

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
            if let Some(why) = browser.navigation_failure() {
                return Err(why);
            }
        }
        crashed(browser_of(state))?;

        // 1. Each clock is granted what its pages are owed.
        let budgets = self.budgets(state, now_ns);
        if !budgets.is_empty() {
            let browser = state.browser.as_mut().expect("booted above");
            let granted = match browser.grant(&budgets, self.settings.stuck_after) {
                Ok(granted) => granted,
                Err(e) => return Err(self.gone(browser, e)),
            };
            if let Granted::Stuck { session, budget_ms } = granted {
                self.stats.add(&self.stats.stuck, 1);
                return Err(stuck(
                    browser.pages.get(&session),
                    budget_ms,
                    self.settings.stuck_after,
                ));
            }
        }
        crashed(browser_of(state))?;

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
                self.take(state, &session, message)?;
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
        said.workers_serial = browser.workers_serial;
        Ok(())
    }

    /// Each clock's budget this slice, in ms, set on the page that takes
    /// it; every page's `granted_ns` set to its clock's.
    ///
    /// Pages whose main thread is one renderer's share one virtual clock
    /// (Chrome reports one `virtualTimeTicksBase` for them): a budget to
    /// any of them advances them all, and two budgets advance them twice.
    /// So a clock is granted once, what its most-owed page is owed, through
    /// each of its pages in turn (a page that is never granted keeps its
    /// next document's load waiting).
    fn budgets(&self, state: &mut State, now_ns: u64) -> Vec<(String, f64)> {
        let mut clocks: BTreeMap<String, Vec<String>> = BTreeMap::new();
        for page in browser_of(state).pages.values() {
            let key = match page.clock {
                Some(clock) => format!("clock {clock}"),
                None => format!("page {}", page.session),
            };
            clocks.entry(key).or_default().push(page.session.clone());
        }
        let turn = self.stats.slices() as usize;
        let mut budgets = Vec::new();
        for pages in clocks.values() {
            let owed = pages
                .iter()
                .filter_map(|session| state.books.get(session).and_then(|b| b.owed(now_ns)))
                .max();
            for session in pages {
                state.books.entry(session.clone()).or_default().granted_ns = 0;
            }
            let Some(owed) = owed else {
                continue;
            };
            if owed <= 0 {
                self.stats.add(&self.stats.skipped, 1);
                continue;
            }
            if pages.len() > 1 {
                self.stats.add(&self.stats.shared, 1);
            }
            for session in pages {
                state.books.entry(session.clone()).or_default().granted_ns = owed as u64;
            }
            self.stats.add(&self.stats.granted_ns, owed as u64);
            budgets.push((pages[turn % pages.len()].clone(), owed as f64 / 1e6));
        }
        budgets
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
            let launched = matches!(self.settings.browse, Browse::Launch(_));
            if launched {
                let deadline = Instant::now() + Duration::from_secs(5);
                while browser.pages.is_empty() && Instant::now() < deadline {
                    browser
                        .pump(Instant::now() + Duration::from_millis(20))
                        .map_err(|e| self.gone(&browser, e))?;
                }
            }
            browser.open(url, launched)?;
        }
        browser.booted();
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
            endpoint = %browser.endpoint,
            "chrome-cdp: Chrome reached on host time; the board's clock meters it from here"
        );
        Ok(browser)
    }

    /// Why a DevTools failure stops the node: what Chrome did, if it was
    /// ours and exited. A socket that failed is watched a moment for the
    /// process's exit, which it can precede.
    fn gone(&self, browser: &Browser, e: CdpError) -> String {
        let socket_failed = matches!(e, CdpError::Io(_) | CdpError::Closed);
        let exited = browser.process.as_ref().is_some_and(|process| {
            let until = Instant::now()
                + if socket_failed {
                    EXIT_GRACE
                } else {
                    Duration::ZERO
                };
            loop {
                if process.leader_exited() {
                    return true;
                }
                if Instant::now() >= until {
                    return false;
                }
                std::thread::sleep(Duration::from_millis(10));
            }
        });
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
    fn take(&self, state: &mut State, session: &str, message: Value) -> Result<(), String> {
        let doc = message["doc"].as_str().unwrap_or_default().to_string();
        let (current, at_boot) = state
            .browser
            .as_ref()
            .and_then(|b| b.pages.get(session))
            .map_or((None, false), |page| (page.doc.clone(), page.at_boot));
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
                if message["late"] == true {
                    let href = message["href"].as_str().unwrap_or_default();
                    if at_boot {
                        self.stats.add(&self.stats.late_at_attach, 1);
                    } else {
                        return Err(format!(
                            "a page at {href} ran its own scripts before the node held it: \
                             Chrome does not hold a page made with a URL (Target.createTarget \
                             with a url, /json/new), so its first document lived on host time \
                             with Chrome's own Web Serial; make the page at about:blank and \
                             navigate it, as Playwright's newPage and goto do"
                        ));
                    }
                }
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
                            let bound = QUEUE_MAX.max(2 * self.window(owner) as usize);
                            state.tx.extend(bytes);
                            if state.tx.len() > bound {
                                let excess = state.tx.len() - bound;
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
        Ok(())
    }

    /// The writer's window for the port's owner: how many bytes may wait
    /// for the line before a write waits — the port's `bufferSize`, or two
    /// quanta of the line's bytes when that is more, so a page writing at
    /// the line's rate keeps it busy across a slice's round trip.
    fn window(&self, owner: &Owner) -> u64 {
        let per_quantum = self.baud_hz as u64 * self.quantum_ns / 1_000_000_000 / 10;
        owner.buffer_size.max(2 * per_quantum)
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
                    if m["flowControl"] == "hardware" {
                        self.stats.add(&self.stats.flow_control_opens, 1);
                        let said = "a page opened the port with hardware flow control: the \
                                    line has no RTS or CTS pin, so nothing paces its bytes"
                            .to_string();
                        tracing::warn!("chrome-cdp: {said}");
                        self.stats
                            .said
                            .lock()
                            .expect("never poisoned")
                            .flow_control
                            .get_or_insert(said);
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
            "setSignals" => {
                let asserted: Vec<&str> = [
                    ("dataTerminalReady", "DTR"),
                    ("requestToSend", "RTS"),
                    ("break", "a break"),
                ]
                .into_iter()
                .filter(|(key, _)| m["signals"][key] == true)
                .map(|(_, name)| name)
                .collect();
                if !asserted.is_empty() {
                    self.stats.add(&self.stats.signals, 1);
                    let said = format!(
                        "a page asserted {} (setSignals): the line has no modem-control pins, \
                         so nothing on the board saw it",
                        asserted.join(" and ")
                    );
                    tracing::warn!("chrome-cdp: {said}");
                    self.stats
                        .said
                        .lock()
                        .expect("never poisoned")
                        .signals
                        .get_or_insert(said);
                }
                ok(id, Value::Null)
            }
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
                // A permission the policy grants cannot be revoked, and the
                // port stays; otherwise the origin loses the port, and an
                // open one is closed, as Chrome closes an origin's
                // connections when its permission goes.
                let closed = !self.settings.granted && holds;
                if !self.settings.granted {
                    state.granted.remove(&origin);
                }
                if closed {
                    self.release(state, Released::Closed);
                }
                ok(id, json!({ "closed": closed }))
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
        let window = owner.as_ref().map_or(255, |o| self.window(o));
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
        let answer = match browser.slice(session, &arg, self.settings.stuck_after) {
            Ok(answer) => answer,
            Err(CdpError::Timeout { after, .. }) => {
                // What the page said meanwhile (a dialog it opened) names
                // why it did not return.
                let _ = browser.drain_socket();
                return Err(unanswered(browser.pages.get(session), after));
            }
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
        if let Some(page) = browser.pages.get_mut(session) {
            page.hidden = answer["hidden"] == true;
        }
        self.stats.add(&self.stats.to_page, rx_bytes.len() as u64);
        if let Some(reading) = reading_of(&answer) {
            self.book(state, session, reading, now_ns)?;
        }
        if let Some(owner) = state
            .owner
            .as_mut()
            .filter(|o| o.page == session && Some(&o.doc) == doc.as_ref())
        {
            owner.reading = answer["reading"].as_bool().unwrap_or(owner.reading);
        }
        // The barrier: armed when the port's stream was transferred to a
        // worker, whose read of the bytes Chrome's clock does not wait for.
        let reads = answer["reads"].as_u64().unwrap_or(0);
        let transferred = answer["transferred"].as_bool().unwrap_or(false);
        let barrier = state.barriers.entry(session.to_string()).or_default();
        if !rx_bytes.is_empty() && transferred && reads > 0 && !barrier.off {
            let doc = doc.unwrap_or_default();
            self.stats.add(&self.stats.drain_waits, 1);
            let browser = state.browser.as_mut().expect("booted above");
            let drained =
                match browser.drain(session, &doc, base + reads, self.settings.drain_bound) {
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
                            "chrome-cdp: a worker that owns the port's stream did not come back \
                             for more in {} waits in a row; the drain barrier is off for that \
                             page (the worker reads the port in a way the probe does not see: \
                             pipeTo, a transform)",
                            DRAIN_STRIKES
                        );
                    }
                }
            }
        }
        Ok(true)
    }

    /// Book a page's clock `reading` at board instant `now_ns`.
    ///
    /// A page lives exactly what its clock was granted: Chrome's virtual
    /// time advances by the budget, and the clock the page reads is that
    /// time clamped to its resolution, with a fuzz below it, so two
    /// readings differ by up to twice the resolution more or less than the
    /// time between them. So the books take the grant, and the reading only
    /// past it by more than three times the resolution — a jump Chrome made
    /// outside the budget, a lead. A new
    /// document starts level with the board: its clock has an origin of
    /// its own, and a navigation into a new renderer starts it anywhere.
    fn book(
        &self,
        state: &mut State,
        session: &str,
        reading: Reading,
        now_ns: u64,
    ) -> Result<(), String> {
        let first = state
            .books
            .iter()
            .filter_map(|(s, b)| b.anchor_ns.map(|at| (at, s.clone())))
            .min()
            .map(|(_, s)| s);
        let books = state.books.entry(session.to_string()).or_default();
        let Some(anchor_ns) = books.anchor_ns else {
            books.anchor_ns = Some(now_ns);
            books.lived_ns = 0;
            books.last_ms = reading.t_ms;
            books.doc = Some(reading.doc);
            return Ok(());
        };
        let board = now_ns.saturating_sub(anchor_ns);
        let tolerance = 3.0 * reading.resolution_ms;
        let advanced_ms = reading.t_ms - books.last_ms;
        let new_doc = books.doc.as_deref() != Some(reading.doc.as_str());
        if new_doc || advanced_ms < -tolerance {
            // A new document, or a clock that went back: level with the
            // board from here.
            self.stats.add(&self.stats.reanchored, 1);
            books.lived_ns = board;
            books.last_ms = reading.t_ms;
            books.doc = Some(reading.doc);
            return Ok(());
        }
        let granted_ms = books.granted_ns as f64 / 1e6;
        let excess_ms = advanced_ms - granted_ms;
        books.lived_ns += if excess_ms > tolerance {
            let excess = (excess_ms * 1e6).round() as u64;
            self.stats
                .peak_overrun_ns
                .fetch_max(excess, Ordering::Relaxed);
            books.granted_ns + excess
        } else if excess_ms < -tolerance {
            (advanced_ms.max(0.0) * 1e6).round() as u64
        } else {
            books.granted_ns
        };
        books.last_ms = reading.t_ms;
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

/// A page's clock as its slice answer reads it: the shim's document and
/// whether it is cross-origin isolated, or, in a document with no shim,
/// the clock's origin.
fn reading_of(answer: &Value) -> Option<Reading> {
    let t_ms = answer["t"].as_f64()?;
    let (doc, isolated) = match answer["doc"].as_str() {
        Some(doc) => (doc.to_string(), answer["iso"] == true),
        None => (
            format!("origin {}", answer["o"].as_f64().unwrap_or_default()),
            false,
        ),
    };
    Some(Reading {
        t_ms,
        doc,
        resolution_ms: if isolated {
            ISOLATED_RESOLUTION_MS
        } else {
            RESOLUTION_MS
        },
    })
}

/// A page that crashed stops the run, before a grant waits on it.
fn crashed(browser: &Browser) -> Result<(), String> {
    match browser.pages.values().find(|page| page.crashed) {
        Some(page) => Err(format!(
            "a page crashed{}: its renderer process went away mid-run",
            page.at()
        )),
        None => Ok(()),
    }
}

/// Why a page holds a grant or a slice: a dialog nobody answered, or a
/// task that never ends.
fn holding(page: Option<&PageTarget>) -> String {
    match page.and_then(|page| page.dialog.as_ref()) {
        Some(dialog) => format!(
            "it shows a {} dialog ({:?}) that nobody answered; a harness answers a page's \
             dialogs (Playwright dismisses them unless a test handles them)",
            dialog.kind, dialog.message
        ),
        None => NEVER_ENDS.to_string(),
    }
}

/// A grant that did not expire, as the failure says it.
fn stuck(page: Option<&PageTarget>, budget_ms: f64, after: Duration) -> String {
    let why = if page.is_some_and(|page| page.dialog.is_some()) {
        holding(page)
    } else {
        format!(
            "Chrome holds a budget while a network fetch is in flight and while a task runs: a \
             fetch that never completes, or {NEVER_ENDS}"
        )
    };
    format!(
        "a grant stuck: the page{} was granted {budget_ms:.3} ms of virtual time and its budget \
         did not expire within {:.1} s of host time{}; {why}",
        page.map(PageTarget::at).unwrap_or_default(),
        after.as_secs_f64(),
        if page.is_some_and(|page| page.hidden) {
            " (the page is hidden, a background tab)"
        } else {
            ""
        },
    )
}

/// A page's side of a slice that did not return, as the failure says it.
fn unanswered(page: Option<&PageTarget>, after: Duration) -> String {
    format!(
        "the main thread of the page{} did not return within {:.1} s of host time; {}",
        page.map(PageTarget::at).unwrap_or_default(),
        after.as_secs_f64(),
        holding(page)
    )
}

/// The browser, once booted.
fn browser_of(state: &State) -> &Browser {
    state.browser.as_ref().expect("booted at the first slice")
}

/// A span as a report prints it: `1.250 ms`.
pub(crate) fn span(ns: u64) -> String {
    format!("{:.3} ms", ns as f64 / 1e6)
}
