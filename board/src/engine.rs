//! Live single-writer net engine + timer wheel (`BOARD_ENGINE.md`,
//! "Execution model (single-writer net engine)").
//!
//! All net state is owned by **one engine thread**. Everything else —
//! firmware cores, model threads, sense callbacks — interacts with it through
//! two queue-fed paths:
//!
//! - **Drives are enqueued**, never applied inline:
//!   [`crate::component::PinHandle::set_drive`] reserves a global enqueue
//!   sequence number and posts an MPSC message
//!   `(endpoint id, new drive or release, enqueue seq)`. The engine dequeues,
//!   re-serializes by that sequence (the authoritative event order, even when
//!   two threads race the channel itself), resolves the affected nets, and
//!   updates net state.
//! - **Senses are delivered** from the engine thread with **no engine lock
//!   held**. Re-entrancy contract: a sense callback MAY drive a pin; that
//!   drive is enqueued and resolved in a *later* engine iteration — never
//!   inline — so driver → net → sense → drive feedback loops are well-defined
//!   and deadlock-free by construction.
//!
//! Time-driven behavior is engine-owned: components request wakeups via
//! [`crate::component::ComponentNetIo::schedule_at`] /
//! [`crate::component::ComponentNetIo::schedule_every`], served by a timer
//! wheel keyed to the virtual-clock **counter**. Idle components cost nothing.
//! The engine is the **time authority** (`EngineCore::run_stepped_iteration`):
//! it waits for every registered actor to park, drains its queue, fires every
//! entry due at `now`, then [`embsim_core::virtual_clock::advance_to`]
//! `min(wheel head, earliest park)`.
//! Optional wall pacing after a jump (`init(speed)` with `speed > 0`) is how
//! a playground feels real-time; tests use `speed <= 0` so jumps are instant.
//! Time is held until `System::start` has attached every component
//! (`Command::ReleaseTime`). Time-sensitive state must be computed at
//! *read time*, never integrated per tick.
//!
//! Build-time analysis and live resolution share **one code path**: the
//! crate-internal `Resolver` in this module is populated by `System`
//! assembly and driven
//! either once (the `System::build` analysis pass) or continuously by the
//! engine thread (`System::start`), so the two can never disagree on
//! semantics. Escalation is part of that shared path: when the digital fast
//! path detects a competing source within
//! [`crate::net::ESCALATION_IMPEDANCE_RATIO`] of the strongest driver, the
//! whole conduction cluster goes through the [`ClusterSolver`]
//! ([`crate::cluster::QuasiStaticMna`] by default).
//!
//! **Pulse routing** is engine-owned and **derived from net resolution, never
//! installed beside it**: the shared `Resolver` routes each
//! [`crate::StreamRole::PulseSource`] to the sinks reachable through its net
//! and through series passives whose accumulated resistance stays below
//! [`STREAM_COLLAPSE_THRESHOLD`]. The routing pass runs at build, at engine
//! spawn, and on any topology-affecting change; two sources reachable from
//! each other raise [`Finding::StreamMismatch`] and neither routes.
//!
//! There used to be a **byte** route beside it, carrying UART traffic from a
//! `Producer` pin to reachable `Consumer`s. It is gone. The net decided who
//! was connected and then the payload went around the resolution, so a byte
//! could not be corrupted by a fighting driver or notice a floating line —
//! and every mechanism it bypassed (a level crossing a series resistor, a
//! wake delivered on time) turned out to be broken in ways nothing could see.
//! Bytes are framed onto the net as levels now, by
//! [`crate::SerialLevelBridge`]. A rate is the one encoding left that is
//! exactly lossless without being a waveform.
//!
//! **Failure containment**: component-provided callbacks (sense, wake,
//! pulse, topology) are panic-contained — a panic is reported as a
//! [`Finding::CallbackPanic`] and the engine stays alive, so one
//! misbehaving component never silently ends net service for the rest of
//! the system. Requests that need the virtual clock before
//! `virtual_clock::init` has run are dropped with a
//! [`Finding::VirtualClockUninitialized`] instead of panicking the engine
//! thread, and [`EngineHandle::is_alive`] reports engine-thread health.
//!
//! # Review rule: no unordered iteration on an engine path
//!
//! **Never iterate a `HashMap` or `HashSet` on an engine path without an
//! explicit sort.** `std`'s hasher is randomly seeded — and `RandomState::new`
//! re-keys on *every* map construction, so a map built per call has a fresh
//! order per call. Any hash order that reaches a published value therefore
//! makes the engine irreproducible, and the effect is often a last-bit float
//! difference rather than an obvious reordering. The three sanctioned shapes:
//!
//! 1. **Dense index** — walk the `Vec` (`self.slots`, `self.streams`,
//!    `self.nets`, `0..n`) and use the map only for keyed lookups. Preferred:
//!    it needs no sort and the order is meaningful.
//! 2. **Explicit sort** — `collect()` the keys, `sort_unstable()`, then
//!    iterate (`driver_roots`, `extra_clusters`, `path_roots`, `fighting`).
//! 3. **Membership only** — a set used purely as a dedup gate or a
//!    `contains`/`len` test, never iterated into an output. These carry an
//!    inline `// hash-order: …` comment saying why order cannot escape.
//!
//! Grep gate: `\.values\(\)|\.keys\(\)|\.iter\(\)` over a `HashMap`/`HashSet`
//! in this module, `cluster.rs`, or `system.rs` should show only shapes 2 and
//! 3. `DETERMINISM.md` (Phase D0 item (d)) is the authoritative statement of
//! this rule; `BOARD_ENGINE.md` cross-references it.

use std::cmp::Reverse;
use std::collections::{BTreeMap, BinaryHeap, HashMap, HashSet};
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::sync::atomic::{AtomicU64, AtomicUsize, Ordering};
use std::sync::mpsc::{self, Receiver, RecvTimeoutError, Sender};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::Duration;

use embsim_core::virtual_clock;

use crate::cluster::{
    Cluster, ClusterInputs, ClusterResistor, ClusterSolution, ClusterSolver, ClusterSource,
};
use crate::component::{PulseTrain, StreamRole};
use crate::diagnostics::{CallbackKind, Diagnostics, Finding, SenseKind};
use crate::event_log::{EngineEvent, EventLog};
use crate::net::{
    Level, Net, NetId, NetState, Ohms, PinRef, TheveninDrive, Volts, ESCALATION_IMPEDANCE_RATIO,
    STREAM_COLLAPSE_THRESHOLD,
};
use crate::system::EdgeFaultKind;

// ============================================================
// Constants
// ============================================================

/// Digital projection threshold: a source voltage at or above this projects
/// to [`Level::High`], below it to [`Level::Low`]. Matches the build-time
/// rail heuristic used for `Pulled` levels. Declared `V_IH`/`V_IL` dead-band
/// handling (`AmbiguousLevel`) is the cluster-solver slice.
const DIGITAL_LEVEL_THRESHOLD_VOLTS: Volts = 1.5;

/// Open-circuit voltage assumed for an idle-high push-pull driver until
/// component-declared rails (`PowerOut` voltages) land. Documented
/// simplification: the build-time pass only consumes the *level* projection
/// of this value, so the exact figure only reaches escalated cluster solves.
pub(crate) const DEFAULT_HIGH_LEVEL_VOLTS: Volts = 3.3;

/// Max commands handled before returning to the timer wheel. A live flood
/// can keep `try_recv` non-empty forever; without a cap, time never jumps.
const COMMAND_DRAIN_BATCH_MAX: usize = 64;

/// How many consecutive *capped* drain passes may happen at one instant before
/// the engine gives up and lets time advance with commands still queued.
///
/// The cap above bounds work per pass, which is what keeps the wheel
/// responsive. Letting time advance the moment a single pass fills up is a
/// different thing, and it is wrong: the commands left behind were issued for
/// the instant being abandoned, and they get applied at the next one instead.
/// A drive that lands a whole bit period late reframes a UART byte — the
/// receiver locks onto the following bit as its start, an all-zero byte
/// decodes as `$80` with a low stop bit, and the driver waiting for it reports
/// a timeout rather than a corruption. That is what a quadrature encoder does
/// to a force-gauge link on the same engine: at 8192 counts/mm and 38 mm/s it
/// walks ~311 counts per millisecond observe and publishes a drive per changed
/// channel, so every burst is many times the cap.
///
/// So drain the burst out at the instant it belongs to, and keep the escape
/// hatch for a producer that really never stops: 64 passes is ~4096 commands,
/// comfortably past any one instant's burst here and still a bound.
const COMMAND_DRAIN_CAPPED_PASSES_MAX: usize = 64;

/// Stepped mode: how long the engine waits for every registered actor to park
/// before declaring the barrier wedged
/// ([`virtual_clock::await_quiescence`] → [`Finding::QuiescenceTimeout`]).
///
/// Wall-clock on purpose — it is not part of the simulation, it is the escape
/// hatch for an actor that never parks. Generous, because reaching it means a
/// defect: a run that trips it is not reproducible, and says so. Overridable
/// per system with [`crate::System::quiescence_timeout`].
pub const STEPPED_QUIESCENCE_TIMEOUT: Duration = Duration::from_secs(5);

/// Stepped mode: how long the engine parks on its command queue when there is
/// nothing to advance to (empty wheel, no pending park deadline).
///
/// Observationally inert — the loop body does nothing when nothing is due — so
/// this poll cadence cannot reach any recorded event. It exists so that an
/// actor registering, or parking, while the engine is idle is noticed without
/// a second wakeup channel into the clock.
const STEPPED_IDLE_POLL: Duration = Duration::from_millis(2);

// ============================================================
// Identity
// ============================================================

/// Dense index of one drive-capable pin endpoint, assigned at assembly.
///
/// Public because it is one of the canonical identities the determinism event
/// log records (`DETERMINISM.md`, "Trace normalization spec": canonicalize
/// identity to the dense ids that already exist — never a thread name, never a
/// pointer, never a `HashMap` order).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct EndpointId(pub usize);

/// Dense index of one attached component, assigned at assembly (keys the
/// timer wheel's wakeup delivery).
///
/// Public for the same reason as [`EndpointId`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ComponentId(pub usize);

// ============================================================
// Engine commands + client link
// ============================================================

/// Sense delivery callback: called from the engine thread with no engine
/// lock held.
pub(crate) type SenseCallback = Box<dyn Fn(NetState) + Send>;

/// Timer-wheel wakeup callback: called from the engine thread with the
/// sampled virtual time (µs); no engine lock held.
pub(crate) type WakeCallback = Box<dyn Fn(u64) + Send>;

/// Topology-change callback (stream-routing seam): called from the engine
/// thread with the new topology epoch; no engine lock held.
pub(crate) type TopologyCallback = Box<dyn Fn(u64) + Send>;

/// Pulse-train delivery callback: called from the engine thread once per
/// **rate change** routed to a pulse sink; no engine lock held. Never once
/// per pulse — that is the whole point of the representation
/// ([`crate::component::StreamRole::PulseSource`]).
pub(crate) type PulseCallback = Box<dyn Fn(PulseTrain) + Send>;

/// One message on the engine's MPSC command queue.
pub(crate) enum Command {
    /// A pin drive (`None` releases to high-Z), stamped with its enqueue
    /// sequence number — the authoritative event order.
    Drive {
        /// Global enqueue sequence reserved at `set_drive` time.
        seq: u64,
        /// Target endpoint.
        endpoint: EndpointId,
        /// New Thevenin contribution, or release.
        drive: Option<TheveninDrive>,
    },
    /// Subscribe a sense callback to one net. The current state is delivered
    /// once at registration (so never-driven nets are reported immediately,
    /// before any traffic), then on every state change.
    RegisterSense {
        /// Net to observe.
        net: NetId,
        /// Delivery callback.
        callback: SenseCallback,
    },
    /// Register the wakeup handler for a component (last registration wins).
    RegisterWake {
        /// Owning component.
        component: ComponentId,
        /// Wakeup callback.
        callback: WakeCallback,
    },
    /// One-shot wakeup at an absolute virtual time (ns).
    ScheduleAt {
        /// Component whose wake handler fires.
        component: ComponentId,
        /// Absolute virtual deadline (ns). Past deadlines fire immediately.
        at_ns: u64,
    },
    /// Periodic wakeup every `period_ns` of virtual time.
    ScheduleEvery {
        /// Component whose wake handler fires.
        component: ComponentId,
        /// Virtual period (ns); zero is rejected with a warning.
        period_ns: u64,
    },
    /// Subscribe to net-graph topology changes (stream-routing seam). The
    /// current epoch is delivered once at registration.
    RegisterTopologyObserver {
        /// Notification callback.
        callback: TopologyCallback,
    },
    /// A pulse source published a new constant-rate segment. Delivered to the
    /// sinks on the source's derived route (gated by net resolution, like
    /// stream bytes), and retained so a sink registering later sees the
    /// channel's current state. Carries no enqueue sequence for the same
    /// reason [`Command::StreamWrite`] does not: per-source order is this
    /// channel's order, and cross-source ordering is not meaningful.
    PulseUpdate {
        /// Source endpoint.
        endpoint: EndpointId,
        /// The segment that just began.
        train: PulseTrain,
    },
    /// Subscribe a pulse-train callback to a pulse sink endpoint.
    RegisterPulseSink {
        /// Sink endpoint.
        endpoint: EndpointId,
        /// Delivery callback.
        callback: PulseCallback,
    },
    /// Stepped mode only: the system is fully assembled — every component has
    /// attached and started — so the engine may begin advancing virtual time.
    ///
    /// Without this barrier a two-component system is not reproducible: the
    /// engine could advance between one component's `schedule_every` and the
    /// next's, so the second component's period would be anchored at a
    /// different instant from run to run. Sent once by `System::start` after
    /// the `Component::start` loop; a no-op in free-running mode, where time
    /// runs regardless.
    ReleaseTime,
    /// Stop the engine loop; pending drives and timers are discarded.
    Shutdown,
}

/// Attach-time drives recorded on the inert (build-time) link, in issue
/// order: the build pass applies them before it resolves for real.
pub(crate) type IdleDriveLog = Arc<Mutex<Vec<(EndpointId, Option<TheveninDrive>)>>>;

/// Cloneable client half of the engine: command sender, the global drive
/// sequence counter, and the engine-published net-state table.
///
/// An **inert** link (`tx == None`) is what the build-time analysis path
/// hands out: senses read the build-resolved snapshot, schedules are traced
/// and dropped, and drives are *recorded* so the build pass can apply a
/// component's idle drive before it publishes findings (a component that
/// releases a `DigitalOut` pin at attach must not be analyzed as if it were
/// driving the engine's idle-high default).
#[derive(Debug, Clone, Default)]
pub(crate) struct EngineLink {
    /// Command queue into the engine thread; `None` on the inert build path.
    ///
    /// The **data plane**: drives, stream writes, registrations. Its drain is
    /// capped ([`COMMAND_DRAIN_BATCH_MAX`]) so a flood cannot starve the wheel.
    pub(crate) tx: Option<Sender<Command>>,
    /// The **control plane**: requests that create a deadline
    /// ([`Command::ScheduleAt`], [`Command::ScheduleEvery`]) and the wake
    /// registration they depend on.
    ///
    /// Separate because the two need opposite treatment. A drive may wait —
    /// deferring it costs nothing but latency. A scheduling request may not:
    /// until the engine handles it, the wheel does not know about the deadline
    /// and virtual time can step straight over it. Sharing one queue means the
    /// capped drain can strand a schedule behind a drive flood, and raising the
    /// cap to reach it just re-creates the starvation the cap prevents. So the
    /// control plane gets its own queue and is drained **in full** every pass;
    /// only the data plane is capped.
    pub(crate) control_tx: Option<Sender<Command>>,
    /// Global drive enqueue sequence, shared by every clone of this link.
    pub(crate) drive_seq: Arc<AtomicU64>,
    /// Scheduling requests sent but not yet handled by the engine.
    ///
    /// Virtual time may not advance while this is non-zero: a `ScheduleAt`
    /// still in the queue is a deadline the engine cannot see, and advancing
    /// past it delivers the wake late. That is invisible for a component whose
    /// events are milliseconds apart and fatal for one clocking a UART bit
    /// every 8.68 µs — the whole byte then lands at a single instant.
    ///
    /// Only *scheduling* is counted. Drives may pile up as deep as they like
    /// without holding time back, which is what keeps the
    /// [`COMMAND_DRAIN_BATCH_MAX`] anti-starvation cap doing its job.
    pub(crate) pending_schedules: Arc<AtomicUsize>,
    /// Engine-published resolved state per net (build snapshot when inert).
    pub(crate) states: Arc<Mutex<Vec<NetState>>>,
    /// Inert path only: drives issued during attach, in issue order, for the
    /// build pass to apply before it resolves for real.
    pub(crate) recorded_drives: Option<IdleDriveLog>,
}

impl EngineLink {
    /// Inert link over a fixed state snapshot (the build-time analysis
    /// path), recording attach-time drives into `recorded_drives`.
    pub(crate) fn inert(states: Arc<Mutex<Vec<NetState>>>, recorded_drives: IdleDriveLog) -> Self {
        Self {
            tx: None,
            control_tx: None,
            drive_seq: Arc::new(AtomicU64::new(0)),
            pending_schedules: Arc::new(AtomicUsize::new(0)),
            states,
            recorded_drives: Some(recorded_drives),
        }
    }

    /// Send a control-plane command: one that creates or depends on a
    /// deadline. Counted in flight from before the send until the engine
    /// handles it, so virtual time cannot step over a deadline that has been
    /// requested but not yet armed — a window that is nanoseconds wide from
    /// the engine's own callbacks and a thread hop wide from anywhere else.
    pub(crate) fn send_control(&self, command: Command) -> bool {
        self.pending_schedules.fetch_add(1, Ordering::AcqRel);
        match &self.control_tx {
            Some(tx) => {
                if tx.send(command).is_err() {
                    self.pending_schedules.fetch_sub(1, Ordering::AcqRel);
                    tracing::debug!("net engine has shut down; control command dropped");
                    return false;
                }
                true
            }
            None => {
                self.pending_schedules.fetch_sub(1, Ordering::AcqRel);
                tracing::debug!("inert link; control command dropped");
                false
            }
        }
    }

    /// Send a command to the engine. Returns `false` (after a trace) when the
    /// link is inert or the engine has shut down — never blocks, never panics.
    pub(crate) fn send(&self, command: Command) -> bool {
        match &self.tx {
            Some(tx) => {
                if tx.send(command).is_err() {
                    tracing::debug!("net engine has shut down; command dropped");
                    false
                } else {
                    true
                }
            }
            None => {
                // Drives are recorded (the build pass applies them before it
                // resolves); everything else is genuinely dropped.
                if let (
                    Command::Drive {
                        endpoint, drive, ..
                    },
                    Some(log),
                ) = (&command, &self.recorded_drives)
                {
                    log.lock()
                        .expect("drive log never poisoned")
                        .push((*endpoint, *drive));
                    return true;
                }
                tracing::debug!("inert engine link (build-time analysis path); command dropped");
                false
            }
        }
    }

    /// Reserve the next global drive sequence number.
    pub(crate) fn next_drive_seq(&self) -> u64 {
        self.drive_seq.fetch_add(1, Ordering::Relaxed)
    }
}

// ============================================================
// Shared resolver (build-time analysis AND live resolution)
// ============================================================

/// Union-find over global net indices.
pub(crate) struct Dsu {
    parent: Vec<usize>,
}

impl Dsu {
    /// Disjoint singletons `0..n`.
    pub(crate) fn new(n: usize) -> Self {
        Self {
            parent: (0..n).collect(),
        }
    }

    /// Current capacity.
    pub(crate) fn len(&self) -> usize {
        self.parent.len()
    }

    /// Extend capacity to at least `n` singletons.
    pub(crate) fn grow(&mut self, n: usize) {
        while self.parent.len() < n {
            self.parent.push(self.parent.len());
        }
    }

    /// Root of `x`, with path halving.
    pub(crate) fn find(&mut self, mut x: usize) -> usize {
        while self.parent[x] != x {
            self.parent[x] = self.parent[self.parent[x]];
            x = self.parent[x];
        }
        x
    }

    /// Merge the sets containing `a` and `b`.
    pub(crate) fn union(&mut self, a: usize, b: usize) {
        let (ra, rb) = (self.find(a), self.find(b));
        if ra != rb {
            self.parent[rb] = ra;
        }
    }
}

/// One drive-capable pin's slot: net membership plus the drive it currently
/// contributes (`None` = released / high-Z / pure sense).
struct DriveSlot {
    net: usize,
    pin: PinRef,
    drive: Option<TheveninDrive>,
}

/// One serial-capable pin registered for stream routing.
struct StreamPin {
    endpoint: EndpointId,
    net: usize,
    role: StreamRole,
    pin: PinRef,
}

/// One derived source→sinks pulse route (see [`Resolver::route_pulses`]).
pub(crate) struct PulseRouteSpec {
    /// Pulse source endpoint the route originates at.
    pub(crate) source: EndpointId,
    /// Sink endpoints reachable through the collapsed link.
    pub(crate) sinks: Vec<EndpointId>,
    /// Identity roots of every net the collapsed link spans — delivery is
    /// gated on their resolved state, exactly as stream bytes are.
    pub(crate) path_roots: Vec<usize>,
}

/// Build-time description of one [`crate::Scenario::edge_fault`] injector.
///
/// The live engine turns each of these into an [`EdgeFaultState`] that counts
/// matching drive edges and opens the float/stuck/contention window.
#[derive(Debug, Clone)]
pub(crate) struct EdgeFaultSpec {
    /// Pin endpoint to count (and suppress, for Float). `None` = every drive
    /// whose slot sits on [`Self::net`].
    pub endpoint: Option<EndpointId>,
    /// Net the stuck/contention ideal source attaches to.
    pub net: usize,
    /// Level-domain effect while the window is open.
    pub kind: EdgeFaultKind,
    /// Matching drives to skip before the window opens.
    pub after_edges: u64,
    /// Matching drives the window stays open for.
    pub edge_count: u64,
}

/// Live counter + window state for one [`EdgeFaultSpec`].
struct EdgeFaultState {
    spec: EdgeFaultSpec,
    /// Matching drive edges observed so far.
    seen: u64,
}

impl EdgeFaultState {
    fn new(spec: EdgeFaultSpec) -> Self {
        Self { spec, seen: 0 }
    }

    /// Whether the window covers the *next* matching drive (index `seen`).
    fn active(&self) -> bool {
        self.spec.edge_count > 0
            && self.seen >= self.spec.after_edges
            && self.seen < self.spec.after_edges.saturating_add(self.spec.edge_count)
    }

    /// Does this drive match the injector's target?
    fn matches(&self, endpoint: EndpointId, net: usize) -> bool {
        match self.spec.endpoint {
            Some(ep) => ep == endpoint,
            None => net == self.spec.net,
        }
    }

    fn tick(&mut self) {
        self.seen = self.seen.saturating_add(1);
    }
}

/// Resolution state shared by the build-time pass and the live engine:
/// topology (identity merges, conduction edges, static sources, senses) plus
/// the per-endpoint drive table the live path mutates. `resolve` recomputes
/// every net's [`NetState`] from the current table — one code path, so
/// build-time analysis and live resolution cannot fork semantics.
pub(crate) struct Resolver {
    /// Union-find of net *identity* merges (harness wires, pin shorts).
    identity: Dsu,
    /// Conduction edges (resistors, inductors, closed jumpers): (a, b, ohms).
    edges: Vec<(usize, usize, f64)>,
    /// Drive-capable endpoints, indexed by [`EndpointId`].
    slots: Vec<DriveSlot>,
    power_sources: Vec<(usize, Volts)>,
    stuck_sources: Vec<(usize, Volts)>,
    digital_senses: Vec<usize>,
    analog_senses: Vec<usize>,
    power_senses: Vec<usize>,
    /// Serial-capable pins, in registration order (stream routing).
    streams: Vec<StreamPin>,
    /// Scenario [`crate::Scenario::edge_fault`] injectors (build-time specs).
    edge_faults: Vec<EdgeFaultSpec>,
    /// Endpoints whose drive is suppressed by an active Float window.
    drive_suppressed: Vec<bool>,
    /// Transient ideal sources from active Stuck/Contention windows.
    edge_stuck_sources: Vec<(usize, Volts)>,
    net_count: usize,
    /// Everything about the board that does not change between drives —
    /// clusters, roots, path resistances — derived once per topology and
    /// reused by every pass. `None` until the first pass builds it.
    topology: Option<Topology>,
    /// Bumped by every topology-changing input, so a cache built against an
    /// older board is never trusted.
    topology_version: u64,
    /// Dense cluster ids whose drive table changed since the last pass.
    dirty: Vec<usize>,
}

/// Drive-independent structure of a board, derived once per topology.
struct Topology {
    /// [`Resolver::topology_version`] this was built against.
    version: u64,
    /// Net count it was built for.
    n: usize,
    /// Identity root of every net.
    root_of: Vec<usize>,
    /// Dense conduction-cluster id of every net.
    cluster_index: Vec<usize>,
    /// Clusters in ascending cluster-root order.
    clusters: Vec<ClusterTopo>,
}

/// One conduction cluster's members, in the orders the pass iterates them.
struct ClusterTopo {
    /// Member nets, ascending.
    nets: Vec<usize>,
    /// Member identity roots (nets that are their own root), ascending.
    roots: Vec<usize>,
    /// Identity-collapsed conduction edges within the cluster, in
    /// declaration order.
    edges: Vec<(usize, usize, f64)>,
    /// Sum of the cluster's declared edge resistances (the `Pulled` bound).
    edge_total_ohms: f64,
    /// Drive-capable endpoints in the cluster, ascending.
    slots: Vec<usize>,
    /// Power-rail sources in the cluster as `(root, volts)`, in declaration order.
    power: Vec<(usize, Volts)>,
    /// `net_stuck` sources in the cluster as `(root, volts)`, in declaration order.
    stuck: Vec<(usize, Volts)>,
    /// Digital sense pins in the cluster as `(registration position, net)`.
    digital_senses: Vec<(usize, usize)>,
    /// Analog sense pins in the cluster as `(registration position, net)`.
    analog_senses: Vec<(usize, usize)>,
    /// Power sense pins in the cluster as `(registration position, net)`.
    power_senses: Vec<(usize, usize)>,
    /// Minimum series resistance between roots, `roots.len()` square,
    /// row-major by position in `roots`; `INFINITY` where no path exists.
    dist: Vec<f64>,
}

/// Findings of one pass, each with the key that orders it the way a full
/// walk of the board would have reported it (contention by first net index,
/// floating senses by kind then registration, power senses by
/// registration), so a pass over any subset of clusters reports in the same
/// relative order as a pass over all of them.
#[derive(Default)]
struct PassFindings {
    contention: Vec<(usize, Finding)>,
    floating: Vec<((usize, usize), Finding)>,
    power: Vec<(usize, Finding)>,
}

impl PassFindings {
    fn emit(mut self, diagnostics: &mut Diagnostics) {
        self.contention.sort_by_key(|(key, _)| *key);
        self.floating.sort_by_key(|(key, _)| *key);
        self.power.sort_by_key(|(key, _)| *key);
        let contention = self.contention.into_iter().map(|(_, f)| f);
        let floating = self.floating.into_iter().map(|(_, f)| f);
        let power = self.power.into_iter().map(|(_, f)| f);
        for finding in contention.chain(floating).chain(power) {
            diagnostics.report(finding);
        }
    }
}

/// Whether two drive contributions are the same (bitwise on the voltages,
/// so `NaN` compares equal to itself and a re-driven rail is a no-op).
fn same_drive(a: &Option<TheveninDrive>, b: &Option<TheveninDrive>) -> bool {
    match (a, b) {
        (None, None) => true,
        (Some(x), Some(y)) => {
            x.volts.total_cmp(&y.volts).is_eq() && x.impedance.total_cmp(&y.impedance).is_eq()
        }
        _ => false,
    }
}

impl Resolver {
    /// New resolver over `net_count` nets with the given identity merges.
    pub(crate) fn new(net_count: usize, identity: Dsu) -> Self {
        Self {
            identity,
            edges: Vec::new(),
            slots: Vec::new(),
            power_sources: Vec::new(),
            stuck_sources: Vec::new(),
            digital_senses: Vec::new(),
            analog_senses: Vec::new(),
            power_senses: Vec::new(),
            streams: Vec::new(),
            edge_faults: Vec::new(),
            drive_suppressed: Vec::new(),
            edge_stuck_sources: Vec::new(),
            net_count,
            topology: None,
            topology_version: 0,
            dirty: Vec::new(),
        }
    }

    /// Add a conduction edge between two nets.
    pub(crate) fn add_edge(&mut self, a: usize, b: usize, ohms: f64) {
        self.topology_version += 1;
        self.edges.push((a, b, ohms));
    }

    /// Register a drive-capable endpoint with its initial contribution
    /// (idle-high for push-pull digital at build; `None` for sense pins).
    pub(crate) fn add_endpoint(
        &mut self,
        net: usize,
        pin: PinRef,
        initial: Option<TheveninDrive>,
    ) -> EndpointId {
        self.topology_version += 1;
        self.slots.push(DriveSlot {
            net,
            pin,
            drive: initial,
        });
        EndpointId(self.slots.len() - 1)
    }

    /// Replace an endpoint's drive contribution (`None` releases to high-Z).
    /// Live path only; the next pass sees the new table.
    ///
    /// Returns whether the table changed. An identical drive is a no-op that
    /// marks nothing dirty — a card re-asserting the level it already holds,
    /// or a pin re-driven high on every clock edge, costs no resolution.
    pub(crate) fn set_drive(&mut self, endpoint: EndpointId, drive: Option<TheveninDrive>) -> bool {
        let Some(slot) = self.slots.get_mut(endpoint.0) else {
            tracing::warn!(endpoint = endpoint.0, "drive for unknown endpoint dropped");
            return false;
        };
        if same_drive(&slot.drive, &drive) {
            return false;
        }
        slot.drive = drive;
        let net = slot.net;
        if let Some(topology) = self
            .topology
            .as_ref()
            .filter(|t| t.version == self.topology_version && net < t.cluster_index.len())
        {
            let cluster = topology.cluster_index[net];
            if !self.dirty.contains(&cluster) {
                self.dirty.push(cluster);
            }
        }
        // With no usable cache the next pass is a full one anyway.
        true
    }

    /// Add a power-rail source (harness power endpoint or `PowerOut` pin).
    pub(crate) fn add_power_source(&mut self, net: usize, volts: Volts) {
        self.topology_version += 1;
        self.power_sources.push((net, volts));
    }

    /// Add a `net_stuck` fault source.
    pub(crate) fn add_stuck_source(&mut self, net: usize, volts: Volts) {
        self.topology_version += 1;
        self.stuck_sources.push((net, volts));
    }

    /// Register a scenario edge-level fault injector.
    pub(crate) fn add_edge_fault(&mut self, spec: EdgeFaultSpec) {
        self.edge_faults.push(spec);
    }

    /// Build-time specs the live engine arms as [`EdgeFaultState`]s.
    pub(crate) fn edge_faults(&self) -> &[EdgeFaultSpec] {
        &self.edge_faults
    }

    /// Net index a drive endpoint sits on, if the endpoint exists.
    pub(crate) fn endpoint_net(&self, endpoint: EndpointId) -> Option<usize> {
        self.slots.get(endpoint.0).map(|s| s.net)
    }

    /// Every drive-capable endpoint whose slot sits on `net`, ascending.
    pub(crate) fn endpoints_on_net(&self, net: usize) -> Vec<EndpointId> {
        self.slots
            .iter()
            .enumerate()
            .filter(|(_, slot)| slot.net == net)
            .map(|(si, _)| EndpointId(si))
            .collect()
    }

    /// Drive contribution visible to resolution (Float windows suppress).
    fn effective_drive(&self, si: usize) -> Option<TheveninDrive> {
        if self.drive_suppressed.get(si).copied().unwrap_or(false) {
            return None;
        }
        self.slots.get(si).and_then(|s| s.drive)
    }

    /// Apply Float suppress masks and Stuck/Contention ideal sources from the
    /// live injector snapshot. `float_endpoints` is the full set of endpoints
    /// any Float injector may suppress; `suppressed_now` is which of them are
    /// active this tick. Marks affected clusters dirty on change.
    pub(crate) fn apply_edge_fault_effects(
        &mut self,
        float_endpoints: &[EndpointId],
        suppressed_now: &[EndpointId],
        stuck: &[(usize, Volts)],
    ) -> bool {
        let mut changed = false;
        let need = float_endpoints
            .iter()
            .map(|ep| ep.0 + 1)
            .max()
            .unwrap_or(0)
            .max(self.drive_suppressed.len());
        if self.drive_suppressed.len() < need {
            self.drive_suppressed.resize(need, false);
        }
        let active: HashSet<usize> = suppressed_now.iter().map(|ep| ep.0).collect();
        for &endpoint in float_endpoints {
            if endpoint.0 >= self.drive_suppressed.len() {
                self.drive_suppressed.resize(endpoint.0 + 1, false);
            }
            let on = active.contains(&endpoint.0);
            if self.drive_suppressed[endpoint.0] != on {
                self.drive_suppressed[endpoint.0] = on;
                changed = true;
                if let Some(net) = self.endpoint_net(endpoint) {
                    self.mark_net_dirty(net);
                }
            }
        }
        let mut new_stuck = stuck.to_vec();
        new_stuck.sort_by(|a, b| a.0.cmp(&b.0).then_with(|| a.1.total_cmp(&b.1)));
        if self.edge_stuck_sources != new_stuck {
            let nets: Vec<usize> = self
                .edge_stuck_sources
                .iter()
                .chain(new_stuck.iter())
                .map(|&(net, _)| net)
                .collect();
            for net in nets {
                self.mark_net_dirty(net);
            }
            self.edge_stuck_sources = new_stuck;
            changed = true;
        }
        changed
    }

    fn mark_net_dirty(&mut self, net: usize) {
        if let Some(topology) = self
            .topology
            .as_ref()
            .filter(|t| t.version == self.topology_version && net < t.cluster_index.len())
        {
            let cluster = topology.cluster_index[net];
            if !self.dirty.contains(&cluster) {
                self.dirty.push(cluster);
            }
        }
    }

    /// Register a digital sense pin (floating-sense findings).
    pub(crate) fn add_digital_sense(&mut self, net: usize) {
        self.topology_version += 1;
        self.digital_senses.push(net);
    }

    /// Register an analog sense pin (floating-sense findings).
    pub(crate) fn add_analog_sense(&mut self, net: usize) {
        self.topology_version += 1;
        self.analog_senses.push(net);
    }

    /// Register a power sense pin (`PowerNetUnsourced` findings).
    pub(crate) fn add_power_sense(&mut self, net: usize) {
        self.topology_version += 1;
        self.power_senses.push(net);
    }

    /// Register a serial-capable pin for stream routing.
    pub(crate) fn add_stream_pin(
        &mut self,
        endpoint: EndpointId,
        net: usize,
        role: StreamRole,
        pin: PinRef,
    ) {
        self.streams.push(StreamPin {
            endpoint,
            net,
            role,
            pin,
        });
    }

    /// The identity root of every net, after harness/pin-short merges:
    /// two nets are electrically the same node iff their roots are equal.
    ///
    /// Net *names* deliberately survive a merge (each keeps its own board's
    /// label), so name comparison cannot answer "are these connected?" —
    /// this can. Snapshotted for [`crate::BuiltSystem`] so connectivity
    /// claims are assertable without a live engine.
    pub(crate) fn identity_roots(&mut self, net_count: usize) -> Vec<usize> {
        (0..net_count).map(|i| self.identity.find(i)).collect()
    }

    /// Whether the cached topology describes the current inputs and `n` nets.
    fn topology_is_current(&self, n: usize) -> bool {
        self.topology
            .as_ref()
            .is_some_and(|t| t.version == self.topology_version && t.n == n)
    }

    /// Rebuild the topology cache if the inputs changed since it was built.
    fn ensure_topology(&mut self, n: usize) {
        if self.topology_is_current(n) {
            return;
        }
        self.identity.grow(self.net_count.max(n));
        let topology = self.build_topology(n);
        self.topology = Some(topology);
        self.dirty.clear();
    }

    /// Derive every drive-independent structure of the board once: identity
    /// roots, conduction clusters (dense ids in ascending cluster-root
    /// order), each cluster's nets, roots, edges, endpoints, sources and
    /// senses, and the minimum series resistance between each pair of its
    /// roots. Resolution passes are pure lookups over this afterwards.
    fn build_topology(&mut self, n: usize) -> Topology {
        let root_of: Vec<usize> = (0..n).map(|i| self.identity.find(i)).collect();

        // Conduction clusters: identity merges are 0-ohm, conduction edges
        // connect within a cluster without merging identity.
        let mut conduction = Dsu::new(n);
        for (i, &root) in root_of.iter().enumerate() {
            conduction.union(root, i);
        }
        for (a, b, _ohms) in &self.edges {
            conduction.union(root_of[*a], root_of[*b]);
        }
        let cluster_of: Vec<usize> = (0..n).map(|i| conduction.find(i)).collect();
        let mut cluster_roots: Vec<usize> = cluster_of.clone();
        cluster_roots.sort_unstable();
        cluster_roots.dedup();
        let cluster_index: Vec<usize> = cluster_of
            .iter()
            .map(|c| {
                cluster_roots
                    .binary_search(c)
                    .expect("every net has a cluster")
            })
            .collect();

        // Identity-collapsed conduction edges (self-loops dropped) for path
        // impedance and escalated-cluster extraction.
        let root_edges: Vec<(usize, usize, f64)> = self
            .edges
            .iter()
            .map(|(a, b, ohms)| (root_of[*a], root_of[*b], *ohms))
            .filter(|(a, b, _)| a != b)
            .collect();

        let clusters = (0..cluster_roots.len())
            .map(|cid| {
                let nets: Vec<usize> = (0..n).filter(|&i| cluster_index[i] == cid).collect();
                let roots: Vec<usize> = nets.iter().copied().filter(|&i| root_of[i] == i).collect();
                let edges: Vec<(usize, usize, f64)> = root_edges
                    .iter()
                    .filter(|(a, _, _)| cluster_index[*a] == cid)
                    .copied()
                    .collect();
                // The sum of the cluster's edge resistances, self-loops
                // included: the upper bound the `Pulled` projection reports.
                let edge_total_ohms: f64 = self
                    .edges
                    .iter()
                    .filter(|(a, b, _)| cluster_index[*a] == cid || cluster_index[*b] == cid)
                    .map(|(_, _, ohms)| ohms)
                    .sum();
                let slots: Vec<usize> = (0..self.slots.len())
                    .filter(|&si| cluster_index[self.slots[si].net] == cid)
                    .collect();
                let sources_in = |list: &[(usize, Volts)]| -> Vec<(usize, Volts)> {
                    list.iter()
                        .filter(|(net, _)| cluster_index[*net] == cid)
                        .map(|(net, volts)| (root_of[*net], *volts))
                        .collect()
                };
                let senses_in = |list: &[usize]| -> Vec<(usize, usize)> {
                    list.iter()
                        .enumerate()
                        .filter(|(_, net)| cluster_index[**net] == cid)
                        .map(|(pos, net)| (pos, *net))
                        .collect()
                };
                // Minimum series resistance between every pair of the
                // cluster's roots; INFINITY where no resistive path exists.
                let k = roots.len();
                let mut dist = vec![f64::INFINITY; k * k];
                for (ia, &ra) in roots.iter().enumerate() {
                    let from = min_path_ohms(&edges, ra);
                    for (ib, rb) in roots.iter().enumerate() {
                        if let Some(&ohms) = from.get(rb) {
                            dist[ia * k + ib] = ohms;
                        }
                    }
                }
                ClusterTopo {
                    nets,
                    roots,
                    edges,
                    edge_total_ohms,
                    slots,
                    power: sources_in(&self.power_sources),
                    stuck: sources_in(&self.stuck_sources),
                    digital_senses: senses_in(&self.digital_senses),
                    analog_senses: senses_in(&self.analog_senses),
                    power_senses: senses_in(&self.power_senses),
                    dist,
                }
            })
            .collect();

        Topology {
            version: self.topology_version,
            n,
            root_of,
            cluster_index,
            clusters,
        }
    }

    /// Run one full resolution pass: assign every net a [`NetState`] from the
    /// current drive table and report findings. Identity-merged nets share
    /// state; conduction clusters share sourced-ness. Clusters where a
    /// competing source sits within [`ESCALATION_IMPEDANCE_RATIO`] of the
    /// strongest driver escalate to `solver`.
    ///
    /// Clusters are electrically independent, so the pass is the union of
    /// one [`Self::resolve_cluster`] per cluster — the same routine the live
    /// path runs on just the clusters a drive touched
    /// ([`Self::resolve_dirty`]). One code path, two scopes.
    pub(crate) fn resolve(
        &mut self,
        nets: &mut [Net],
        diagnostics: &mut Diagnostics,
        solver: &dyn ClusterSolver,
    ) {
        let n = nets.len();
        self.ensure_topology(n);
        let topology = self.topology.take().expect("ensure_topology built it");
        let mut findings = PassFindings::default();
        for cid in 0..topology.clusters.len() {
            self.resolve_cluster(&topology, cid, nets, &mut findings, solver);
        }
        findings.emit(diagnostics);
        self.topology = Some(topology);
        self.dirty.clear();
    }

    /// The nets the next [`Self::resolve_dirty`] will touch, ascending: the
    /// members of every cluster whose drive table changed — or every net,
    /// when the topology changed and the next pass must be a full one.
    pub(crate) fn dirty_scope(&self, n: usize) -> Vec<usize> {
        if !self.topology_is_current(n) {
            return (0..n).collect();
        }
        let topology = self.topology.as_ref().expect("current");
        let mut scope: Vec<usize> = self
            .dirty
            .iter()
            .flat_map(|&cid| topology.clusters[cid].nets.iter().copied())
            .collect();
        scope.sort_unstable();
        scope.dedup();
        scope
    }

    /// Resolve only the clusters a drive changed since the last pass (the
    /// scope [`Self::dirty_scope`] announced), reporting their findings.
    /// Every other net keeps its state, which is exactly what the full pass
    /// would have recomputed for it. Falls back to a full pass when the
    /// topology changed underneath.
    pub(crate) fn resolve_dirty(
        &mut self,
        nets: &mut [Net],
        diagnostics: &mut Diagnostics,
        solver: &dyn ClusterSolver,
    ) {
        if !self.topology_is_current(nets.len()) {
            self.resolve(nets, diagnostics, solver);
            return;
        }
        if self.dirty.is_empty() {
            return;
        }
        let topology = self.topology.take().expect("current");
        let mut dirty = std::mem::take(&mut self.dirty);
        dirty.sort_unstable();
        dirty.dedup();
        let mut findings = PassFindings::default();
        for &cid in &dirty {
            self.resolve_cluster(&topology, cid, nets, &mut findings, solver);
        }
        findings.emit(diagnostics);
        self.topology = Some(topology);
    }

    /// Resolve one conduction cluster from the current drive table: the
    /// digital fast path, contention through collapsed series resistance,
    /// escalation to the cluster solver, state assignment for the cluster's
    /// nets, and the cluster's findings.
    ///
    /// Everything here is cluster-local by construction — every cross-net
    /// rule walks conduction edges or identity roots, and neither crosses a
    /// cluster boundary — which is what makes resolving a subset exact.
    /// Iteration is over dense, ascending indices throughout (never a hash
    /// walk), so a pass is bit-for-bit reproducible; see `DETERMINISM.md`.
    fn resolve_cluster(
        &self,
        topology: &Topology,
        cid: usize,
        nets: &mut [Net],
        findings: &mut PassFindings,
        solver: &dyn ClusterSolver,
    ) {
        let c = &topology.clusters[cid];
        let root_of = &topology.root_of;
        let k = c.roots.len();
        let pos_of_root = |root: usize| -> usize {
            c.roots
                .iter()
                .position(|&r| r == root)
                .expect("a cluster's sources sit on its own roots")
        };

        // Sources per conduction cluster. NaN ("sourced at an unmodeled
        // voltage") rails mark the cluster sourced but carry no numeric
        // level: an unmodeled rail must not mask a real 0 V rail on the same
        // cluster. An injected `net_stuck` is an *ideal* source and counts
        // for the level heuristic exactly like a rail.
        let mut cluster_power: Option<Volts> = None;
        let mut cluster_sourced = false;
        // Scenario edge-fault Stuck/Contention sources that land in this cluster.
        let edge_stuck_here: Vec<(usize, Volts)> = self
            .edge_stuck_sources
            .iter()
            .filter_map(|&(net, volts)| {
                let root = root_of.get(net).copied()?;
                if c.roots.contains(&root) {
                    Some((root, volts))
                } else {
                    None
                }
            })
            .collect();
        let stuck_iter = c
            .stuck
            .iter()
            .copied()
            .chain(edge_stuck_here.iter().copied());

        for (_, volts) in c.power.iter().copied().chain(stuck_iter.clone()) {
            if !volts.is_nan() && cluster_power.is_none() {
                cluster_power = Some(volts);
            }
            cluster_sourced = true;
        }

        // Driving endpoints per identity root, in endpoint order; drivers
        // also source their cluster. Float windows suppress via effective_drive.
        let mut net_drivers: Vec<(usize, Vec<usize>)> = Vec::new();
        for &si in &c.slots {
            let slot = &self.slots[si];
            if self.effective_drive(si).is_none() {
                continue;
            }
            let root = root_of[slot.net];
            match net_drivers.iter_mut().find(|(r, _)| *r == root) {
                Some((_, slots)) => slots.push(si),
                None => net_drivers.push((root, vec![si])),
            }
            cluster_sourced = true;
        }
        let drivers_of = |root: usize| -> Option<&[usize]> {
            net_drivers
                .iter()
                .find(|(r, _)| *r == root)
                .map(|(_, slots)| slots.as_slice())
        };
        let drive_of = |si: usize| {
            self.effective_drive(si)
                .expect("net_drivers only holds driving slots")
        };
        // (has a High driver, has a Low driver) on a root: the two facts the
        // fast path asks of a driver set.
        let levels_of = |root: usize| -> (bool, bool) {
            let mut high = false;
            let mut low = false;
            for &si in drivers_of(root).unwrap_or(&[]) {
                match level_of_volts(drive_of(si).volts) {
                    Level::High => high = true,
                    Level::Low => low = true,
                }
            }
            (high, low)
        };

        // Direct source levels per identity root (power/stuck beat drivers
        // for the fast-path state projection); NaN rails skipped.
        let mut direct_volts: Vec<(usize, Volts)> = Vec::new();
        for (root, volts) in c.power.iter().copied().chain(stuck_iter.clone()) {
            if volts.is_nan() || direct_volts.iter().any(|(r, _)| *r == root) {
                continue;
            }
            direct_volts.push((root, volts));
        }
        let direct_of = |root: usize| -> Option<Volts> {
            direct_volts
                .iter()
                .find(|(r, _)| *r == root)
                .map(|(_, v)| *v)
        };

        // Every Thevenin source in the cluster: drivers in endpoint order,
        // then rails and stuck faults as ideal 0-ohm sources. This order is
        // the SPICE card order the cluster solver stamps (determinism).
        let mut sources: Vec<ClusterSource> = Vec::new();
        for &si in &c.slots {
            let slot = &self.slots[si];
            if let Some(drive) = self.effective_drive(si) {
                sources.push(ClusterSource {
                    node: NetId(root_of[slot.net]),
                    volts: drive.volts,
                    impedance: drive.impedance,
                });
            }
        }
        for (root, volts) in c.power.iter().copied().chain(stuck_iter) {
            if volts.is_nan() {
                continue;
            }
            sources.push(ClusterSource {
                node: NetId(root),
                volts,
                impedance: 0.0,
            });
        }

        let mut driver_roots: Vec<usize> = net_drivers.iter().map(|(r, _)| *r).collect();
        driver_roots.sort_unstable();

        // -- contention through collapsed series resistance ------------------
        // Disagreeing push-pull sources coupled through series resistance
        // below STREAM_COLLAPSE_THRESHOLD resolve to Contention, not to a
        // divided voltage: for signaling purposes the collapsed link is one
        // node. Rails and stuck faults through the same resistance still
        // escalate to the divided-voltage solve below.
        let mut contended: Vec<(usize, Vec<usize>)> = Vec::new();
        for (i, &ra) in driver_roots.iter().enumerate() {
            let pa = pos_of_root(ra);
            for &rb in &driver_roots[i + 1..] {
                let ohms = c.dist[pa * k + pos_of_root(rb)];
                // `INFINITY` (no resistive path) is never coupled.
                if ohms >= STREAM_COLLAPSE_THRESHOLD {
                    continue;
                }
                let (ha, la) = levels_of(ra);
                let (hb, lb) = levels_of(rb);
                if (ha || hb) && (la || lb) {
                    for root in [ra, rb] {
                        let fighting = match contended.iter_mut().find(|(r, _)| *r == root) {
                            Some((_, fighting)) => fighting,
                            None => {
                                contended.push((root, Vec::new()));
                                &mut contended.last_mut().expect("just pushed").1
                            }
                        };
                        fighting.extend_from_slice(drivers_of(ra).unwrap_or(&[]));
                        fighting.extend_from_slice(drivers_of(rb).unwrap_or(&[]));
                    }
                }
            }
        }
        for (_, fighting) in &mut contended {
            fighting.sort_unstable();
            fighting.dedup();
        }
        let contended_drivers = |root: usize| -> Option<&[usize]> {
            contended
                .iter()
                .find(|(r, _)| *r == root)
                .map(|(_, fighting)| fighting.as_slice())
        };

        // -- escalation: digital fast path vs cluster solver ---------------
        // A driver-bearing root escalates the cluster when a source at a
        // DIFFERENT level is reachable with Thevenin impedance within
        // ESCALATION_IMPEDANCE_RATIO of the strongest driver. Disagreeing
        // push-pull drivers on one node stay on the Contention fast path.
        let solve = || -> ClusterSolution {
            let nodes: Vec<NetId> = c.roots.iter().map(|&r| NetId(r)).collect();
            let resistors: Vec<ClusterResistor> = c
                .edges
                .iter()
                .map(|(a, b, ohms)| ClusterResistor {
                    a: NetId(*a),
                    b: NetId(*b),
                    ohms: *ohms,
                })
                .collect();
            let inputs = ClusterInputs {
                sources: sources.clone(),
            };
            solver.solve(&Cluster { nodes, resistors }, &inputs)
        };
        let mut escalated: Option<ClusterSolution> = None;
        for &root in &driver_roots {
            let slots = drivers_of(root).expect("driver root has drivers");
            let (high, low) = levels_of(root);
            if high && low {
                continue; // direct push-pull fight: Contention fast path
            }
            let level = if high { Level::High } else { Level::Low };
            let strongest: Ohms = slots
                .iter()
                .map(|&si| drive_of(si).impedance)
                .fold(f64::INFINITY, f64::min);

            let mut competing = f64::INFINITY;
            // Ideal sources directly on this node (net_stuck / power rail).
            if let Some(v) = direct_of(root) {
                if !v.is_nan() && level_of_volts(v) != level {
                    competing = 0.0;
                }
            }
            // Sources reachable through conduction edges within the cluster.
            let pr = pos_of_root(root);
            for source in &sources {
                if source.node.0 == root || level_of_volts(source.volts) == level {
                    continue;
                }
                let path = c.dist[pr * k + pos_of_root(source.node.0)];
                if path.is_finite() {
                    competing = competing.min(path + source.impedance);
                }
            }
            if competing <= strongest * ESCALATION_IMPEDANCE_RATIO {
                escalated = Some(solve());
                break;
            }
        }

        // -- escalation beyond driver roots ---------------------------------
        // Ideal sources fighting (rails and stuck faults are 0 Ω, so no
        // impedance gate): disagreeing levels on one root project Contention
        // there; disagreeing levels anywhere in the cluster escalate so
        // intermediate nodes get their divided voltage. Analog senses: a
        // cluster with a registered analog sense and any numeric source
        // escalates, so an ADC input always reads the solved voltage.
        let mut ideal_roots: Vec<(usize, bool, bool)> = Vec::new();
        let (mut cluster_high, mut cluster_low) = (false, false);
        for (root, volts) in c.power.iter().chain(c.stuck.iter()) {
            if volts.is_nan() {
                continue;
            }
            let high = level_of_volts(*volts) == Level::High;
            match ideal_roots.iter_mut().find(|(r, _, _)| r == root) {
                Some((_, h, l)) => {
                    *h |= high;
                    *l |= !high;
                }
                None => ideal_roots.push((*root, high, !high)),
            }
            cluster_high |= high;
            cluster_low |= !high;
        }
        let ideal_contended =
            |root: usize| ideal_roots.iter().any(|(r, h, l)| *r == root && *h && *l);
        let needs_solve =
            (cluster_high && cluster_low) || (!c.analog_senses.is_empty() && !sources.is_empty());
        if needs_solve && escalated.is_none() {
            escalated = Some(solve());
        }

        // -- state assignment -----------------------------------------------
        for &i in &c.nets {
            let root = root_of[i];
            let state = if contended_drivers(root).is_some() || ideal_contended(root) {
                NetState::Contention
            } else if let Some(solution) = &escalated {
                solution.state_of(NetId(root)).unwrap_or_else(|| {
                    tracing::warn!(net = %nets[i].name, "cluster solver omitted a node; reporting Floating");
                    NetState::Floating
                })
            } else if let Some(v) = direct_of(root) {
                NetState::Analog(v)
            } else if let Some(slots) = drivers_of(root) {
                let (high, low) = levels_of(root);
                if high && low {
                    NetState::Contention
                } else {
                    NetState::Driven(level_of_volts(drive_of(slots[0]).volts))
                }
            } else if cluster_sourced {
                // Reached only through conduction edges: project as pulled
                // toward the cluster's source — a *driver* counts, not only a
                // declared rail, or a driven signal would stop arriving the
                // moment it crossed a series resistor.
                let level = match cluster_power {
                    Some(v) => level_of_volts(v),
                    None => cluster_driver_level(&self.slots, &c.slots, Level::High),
                };
                NetState::Pulled(level, c.edge_total_ohms)
            } else {
                NetState::Floating
            };
            nets[i].state = state;
        }

        // -- findings ---------------------------------------------------------
        // Contention per identity root (deduped), keyed by the first net
        // index that reports it; cross-root fights name every fighting
        // driver, direct fights the root's own, ideal-source fights the net.
        let mut reported_contention: Vec<usize> = Vec::new();
        for &i in &c.nets {
            let root = root_of[i];
            if nets[i].state == NetState::Contention && !reported_contention.contains(&root) {
                reported_contention.push(root);
                let drivers = contended_drivers(root)
                    .or_else(|| drivers_of(root))
                    .map(|slots| slots.iter().map(|&si| self.slots[si].pin.clone()).collect())
                    .unwrap_or_default();
                findings.contention.push((
                    i,
                    Finding::Contention {
                        net: nets[root.min(i)].name.clone(),
                        drivers,
                    },
                ));
            }
        }
        // Floating senses (deduped per (identity root, kind)), keyed by
        // registration order within the kind.
        let mut reported_floating: Vec<(usize, SenseKind)> = Vec::new();
        for (senses, kind, kind_order) in [
            (&c.digital_senses, SenseKind::Digital, 0usize),
            (&c.analog_senses, SenseKind::Analog, 1usize),
        ] {
            for &(pos, net) in senses {
                let root = root_of[net];
                if nets[net].state == NetState::Floating
                    && !reported_floating.contains(&(root, kind))
                {
                    reported_floating.push((root, kind));
                    findings.floating.push((
                        (kind_order, pos),
                        Finding::FloatingSense {
                            net: nets[net].name.clone(),
                            kind,
                        },
                    ));
                }
            }
        }
        // Power senses on an unsourced cluster (deduped per identity root).
        let mut reported_power: Vec<usize> = Vec::new();
        for &(pos, net) in &c.power_senses {
            let root = root_of[net];
            if !cluster_sourced && !reported_power.contains(&root) {
                reported_power.push(root);
                findings.power.push((
                    pos,
                    Finding::PowerNetUnsourced {
                        net: nets[net].name.clone(),
                    },
                ));
            }
        }
    }

    /// The resolver as it was before the per-cluster rewrite: one global pass.
    /// Kept, test-only, as the reference the new pass is checked against.
    #[cfg(test)]
    pub(crate) fn resolve_reference(
        &mut self,
        nets: &mut [Net],
        diagnostics: &mut Diagnostics,
        solver: &dyn ClusterSolver,
    ) {
        self.identity.grow(self.net_count.max(nets.len()));
        let n = nets.len();

        // Pre-resolve identity roots so the remaining passes are pure lookups.
        let root_of: Vec<usize> = (0..n).map(|i| self.identity.find(i)).collect();

        // Conduction clusters: identity merges are 0-ohm, conduction edges
        // connect within a cluster without merging identity.
        let mut conduction = Dsu::new(n);
        for (i, &root) in root_of.iter().enumerate() {
            conduction.union(root, i);
        }
        for (a, b, _ohms) in &self.edges {
            conduction.union(root_of[*a], root_of[*b]);
        }
        let cluster_of: Vec<usize> = (0..n).map(|i| conduction.find(i)).collect();

        // Identity-collapsed conduction edges (self-loops dropped) for path
        // impedance and escalated-cluster extraction.
        let root_edges: Vec<(usize, usize, f64)> = self
            .edges
            .iter()
            .map(|(a, b, ohms)| (root_of[*a], root_of[*b], *ohms))
            .filter(|(a, b, _)| a != b)
            .collect();

        // Sources per conduction cluster. NaN ("sourced at an unmodeled
        // voltage") rails mark their cluster sourced but carry no numeric
        // level, so they never enter `cluster_power` (the `Pulled` level
        // heuristic) — an unmodeled rail must not mask a real 0 V rail on
        // the same cluster.
        let mut cluster_power: HashMap<usize, Volts> = HashMap::new();
        let mut cluster_sourced: HashSet<usize> = HashSet::new();
        for (net, volts) in &self.power_sources {
            let c = cluster_of[*net];
            if !volts.is_nan() {
                cluster_power.entry(c).or_insert(*volts);
            }
            cluster_sourced.insert(c);
        }
        for (net, volts) in &self.stuck_sources {
            // An injected `net_stuck` is an *ideal* source, so it counts for
            // the level heuristic exactly like a rail. Leaving it out let a
            // 25 Ω driver behind kilohms of series resistance out-vote a 0 Ω
            // short — the opposite of what fault injection is for.
            if !volts.is_nan() {
                cluster_power.entry(cluster_of[*net]).or_insert(*volts);
            }
            cluster_sourced.insert(cluster_of[*net]);
        }

        // Driving endpoints: per identity-merged net, detect contention;
        // drivers also source their conduction cluster.
        let mut net_drivers: HashMap<usize, Vec<usize>> = HashMap::new();
        for (si, slot) in self.slots.iter().enumerate() {
            if slot.drive.is_none() {
                continue;
            }
            net_drivers.entry(root_of[slot.net]).or_default().push(si);
            cluster_sourced.insert(cluster_of[slot.net]);
        }
        let drive_of = |si: usize| {
            self.slots[si]
                .drive
                .expect("net_drivers only holds driving slots")
        };

        // Direct source levels per identity root (power/stuck beat drivers
        // for the fast-path state projection). NaN rails are skipped here
        // like every other consumer: state assignment would otherwise
        // publish `Analog(NaN)`, and NaN defeats the sense change gate
        // (`Analog(NaN) != Analog(NaN)`), re-delivering senses on every
        // pass — a NaN-sourced net instead projects `Pulled` through the
        // `cluster_sourced` fallback below.
        let mut direct_volts: HashMap<usize, Volts> = HashMap::new();
        for (net, volts) in self.power_sources.iter().chain(self.stuck_sources.iter()) {
            if volts.is_nan() {
                continue;
            }
            direct_volts.entry(root_of[*net]).or_insert(*volts);
        }

        // Every Thevenin source per conduction cluster (drivers at their
        // declared impedance; power rails and stuck faults as ideal 0-ohm
        // sources; NaN "unmodeled voltage" rails are skipped — they source
        // the cluster but cannot enter a numeric solve).
        //
        // **Determinism (load-bearing):** iterate the DENSE drive table, never
        // `net_drivers` (a `HashMap`). This `Vec`'s order is the SPICE card
        // order [`crate::cluster::QuasiStaticMna::solve`] stamps, so a hash walk would
        // make the deck (and, for a linear solver, last-bit voltages) depend
        // on a per-process hasher seed. Slots and `net_drivers`' member lists
        // are both built in endpoint order. See `DETERMINISM.md`.
        let mut cluster_sources: HashMap<usize, Vec<ClusterSource>> = HashMap::new();
        for slot in &self.slots {
            let Some(drive) = slot.drive else {
                continue;
            };
            cluster_sources
                .entry(cluster_of[slot.net])
                .or_default()
                .push(ClusterSource {
                    node: NetId(root_of[slot.net]),
                    volts: drive.volts,
                    impedance: drive.impedance,
                });
        }
        for (net, volts) in self.power_sources.iter().chain(self.stuck_sources.iter()) {
            if volts.is_nan() {
                continue;
            }
            cluster_sources
                .entry(cluster_of[*net])
                .or_default()
                .push(ClusterSource {
                    node: NetId(root_of[*net]),
                    volts: *volts,
                    impedance: 0.0,
                });
        }

        // -- contention through collapsed series resistance ------------------
        // Disagreeing push-pull sources coupled through series resistance
        // below STREAM_COLLAPSE_THRESHOLD resolve to Contention, not to a
        // divided voltage (net rules, `BOARD_ENGINE.md` "Net state model"):
        // for signaling purposes the collapsed link is one node — this is
        // the crossed-TX/RX case. Power rails and stuck faults through the
        // same resistance still escalate to the divided-voltage solve below
        // (a pull-up fighting a driver is a divider, not a fight between
        // two push-pull outputs).
        let mut contended: HashMap<usize, Vec<usize>> = HashMap::new();
        {
            // hash-order shape 2: keys collected then sorted, so the pair
            // walk below is in root order.
            let mut driver_roots: Vec<usize> = net_drivers.keys().copied().collect();
            driver_roots.sort_unstable();
            // hash-order shape 3: this set is only ever asked for `.len()`
            // (below, "does this pair disagree?"), never iterated.
            let levels_of = |root: usize| -> HashSet<Level> {
                net_drivers[&root]
                    .iter()
                    .map(|&si| level_of_volts(drive_of(si).volts))
                    .collect()
            };
            for (i, &ra) in driver_roots.iter().enumerate() {
                let dist = min_path_ohms(&root_edges, ra);
                for &rb in &driver_roots[i + 1..] {
                    let coupled = dist
                        .get(&rb)
                        .is_some_and(|&ohms| ohms < STREAM_COLLAPSE_THRESHOLD);
                    if !coupled {
                        continue;
                    }
                    let mut union = levels_of(ra);
                    union.extend(levels_of(rb));
                    if union.len() > 1 {
                        for root in [ra, rb] {
                            let fighting = contended.entry(root).or_default();
                            fighting.extend(net_drivers[&ra].iter().copied());
                            fighting.extend(net_drivers[&rb].iter().copied());
                        }
                    }
                }
            }
            // hash-order shape 3: `values_mut` mutates each Vec in place —
            // which Vec is visited first cannot affect any of them, and each
            // is sorted here so the reported driver list is canonical.
            for fighting in contended.values_mut() {
                fighting.sort_unstable();
                fighting.dedup();
            }
        }

        // -- escalation: digital fast path vs cluster solver ---------------
        // A driver-bearing root escalates its whole conduction cluster when a
        // source at a DIFFERENT level (agreeing sources cannot divide the
        // node) is reachable with Thevenin impedance within
        // ESCALATION_IMPEDANCE_RATIO of the strongest driver. Disagreeing
        // push-pull drivers on one node stay on the Contention fast path.
        // Roots contended through collapsed resistance still escalate their
        // cluster, so the mid-rail voltage stays available to the solve —
        // but their own projection below is Contention.
        let solve_cluster = |cluster: usize| -> ClusterSolution {
            let nodes: Vec<NetId> = (0..n)
                .filter(|&i| root_of[i] == i && cluster_of[i] == cluster)
                .map(NetId)
                .collect();
            let resistors: Vec<ClusterResistor> = root_edges
                .iter()
                .filter(|(a, _, _)| cluster_of[*a] == cluster)
                .map(|(a, b, ohms)| ClusterResistor {
                    a: NetId(*a),
                    b: NetId(*b),
                    ohms: *ohms,
                })
                .collect();
            let inputs = ClusterInputs {
                sources: cluster_sources.get(&cluster).cloned().unwrap_or_default(),
            };
            solver.solve(&Cluster { nodes, resistors }, &inputs)
        };
        // hash-order: `escalated` is keyed access only (`contains_key`, `get`,
        // `entry`) and never iterated. `driver_roots` is shape 2 — the escalation
        // decision below runs in root order, so which cluster wins the
        // `contains_key` short-circuit is fixed.
        let mut escalated: HashMap<usize, ClusterSolution> = HashMap::new();
        let mut driver_roots: Vec<usize> = net_drivers.keys().copied().collect();
        driver_roots.sort_unstable();
        for root in driver_roots {
            let cluster = cluster_of[root];
            if escalated.contains_key(&cluster) {
                continue;
            }
            let slots = &net_drivers[&root];
            // hash-order shape 3: `.len()` gates, and the `.next()` below runs
            // only when the set holds exactly one element — so iteration order
            // has nothing to choose between.
            let levels: HashSet<Level> = slots
                .iter()
                .map(|&si| level_of_volts(drive_of(si).volts))
                .collect();
            if levels.len() > 1 {
                continue; // direct push-pull fight: Contention fast path
            }
            let level = *levels.iter().next().expect("driver root has drivers");
            let strongest: Ohms = slots
                .iter()
                .map(|&si| drive_of(si).impedance)
                .fold(f64::INFINITY, f64::min);

            let mut competing = f64::INFINITY;
            // Ideal sources directly on this node (net_stuck / power rail).
            if let Some(v) = direct_volts.get(&root) {
                if !v.is_nan() && level_of_volts(*v) != level {
                    competing = 0.0;
                }
            }
            // Sources reachable through conduction edges within the cluster.
            let dist = min_path_ohms(&root_edges, root);
            // Order-independent by arithmetic: `f64::min` over a set of
            // finite values gives the same result in any order, so this
            // consumer of `cluster_sources` is safe regardless. The MNA
            // accumulation is the one that is not — see its assembly above.
            if let Some(sources) = cluster_sources.get(&cluster) {
                for source in sources {
                    if source.node.0 == root || level_of_volts(source.volts) == level {
                        continue;
                    }
                    if let Some(path) = dist.get(&source.node.0) {
                        competing = competing.min(path + source.impedance);
                    }
                }
            }

            if competing <= strongest * ESCALATION_IMPEDANCE_RATIO {
                escalated.insert(cluster, solve_cluster(cluster));
            }
        }

        // -- escalation beyond driver roots ---------------------------------
        // The driver loop above cannot see clusters with no push-pull
        // driver, so two more triggers reach the solver (`BOARD_ENGINE.md`
        // "Analog clusters" / fault algebra):
        //
        // - **Ideal sources fighting**: power rails and `net_stuck` faults
        //   are 0 Ω sources — they pin their own node, so no impedance-ratio
        //   gate applies. Disagreeing levels on one identity root project
        //   Contention there (a stuck-at-0 shorting a 3.3 V rail must be
        //   observable, never a silent first-source-wins projection);
        //   disagreeing levels anywhere in a cluster (a resistor divider
        //   between rails) escalate so intermediate nodes get their divided
        //   voltage rather than the `Pulled` fallback.
        // - **Analog senses**: a cluster containing a registered analog
        //   sense and any numeric source escalates, so an ADC input always
        //   reads the solved node voltage. Digital-only pulled nets keep
        //   their fast-path `Pulled` projection.
        let mut ideal_root_levels: HashMap<usize, HashSet<Level>> = HashMap::new();
        let mut ideal_cluster_levels: HashMap<usize, HashSet<Level>> = HashMap::new();
        for (net, volts) in self.power_sources.iter().chain(self.stuck_sources.iter()) {
            if volts.is_nan() {
                continue;
            }
            let level = level_of_volts(*volts);
            ideal_root_levels
                .entry(root_of[*net])
                .or_default()
                .insert(level);
            ideal_cluster_levels
                .entry(cluster_of[*net])
                .or_default()
                .insert(level);
        }
        // hash-order shape 3: `ideal_contended` is only ever `.contains`-ed
        // during state assignment; it is never iterated into an output.
        let ideal_contended: HashSet<usize> = ideal_root_levels
            .iter()
            .filter(|(_, levels)| levels.len() > 1)
            .map(|(&root, _)| root)
            .collect();
        // hash-order shape 2: collected from a map, then sorted + deduped
        // below, so the solve order over extra clusters is cluster order.
        let mut extra_clusters: Vec<usize> = ideal_cluster_levels
            .iter()
            .filter(|(_, levels)| levels.len() > 1)
            .map(|(&cluster, _)| cluster)
            .collect();
        extra_clusters.extend(
            self.analog_senses
                .iter()
                .map(|&net| cluster_of[net])
                .filter(|cluster| cluster_sources.contains_key(cluster)),
        );
        extra_clusters.sort_unstable();
        extra_clusters.dedup();
        for cluster in extra_clusters {
            escalated
                .entry(cluster)
                .or_insert_with(|| solve_cluster(cluster));
        }

        // -- state assignment -----------------------------------------------
        for (i, net) in nets.iter_mut().enumerate() {
            let root = root_of[i];
            let cluster = cluster_of[i];

            let state = if contended.contains_key(&root) || ideal_contended.contains(&root) {
                NetState::Contention
            } else if let Some(solution) = escalated.get(&cluster) {
                solution.state_of(NetId(root)).unwrap_or_else(|| {
                    tracing::warn!(net = %net.name, "cluster solver omitted a node; reporting Floating");
                    NetState::Floating
                })
            } else if let Some(v) = direct_volts.get(&root) {
                NetState::Analog(*v)
            } else if let Some(slots) = net_drivers.get(&root) {
                let levels: HashSet<Level> = slots
                    .iter()
                    .map(|&si| level_of_volts(drive_of(si).volts))
                    .collect();
                if levels.len() > 1 {
                    NetState::Contention
                } else {
                    NetState::Driven(level_of_volts(drive_of(slots[0]).volts))
                }
            } else if cluster_sourced.contains(&cluster) {
                // Reached only through conduction edges: project as pulled
                // toward the cluster's source. Exact series resistance is the
                // cluster-solver slice; this pass reports the sum of the
                // cluster's edge resistances as an upper bound.
                let total: f64 = self
                    .edges
                    .iter()
                    .filter(|(a, b, _)| cluster_of[*a] == cluster || cluster_of[*b] == cluster)
                    .map(|(_, _, ohms)| ohms)
                    .sum();
                // "Toward the cluster's source" includes a *driver*, not only a
                // declared rail. Defaulting to High whenever no rail was
                // declared makes a driven signal stop arriving the moment it
                // crosses a series resistor — invisible while bytes bypassed
                // the net, and fatal once a UART's bits have to get through an
                // ESD resistor to reach the part on the other side.
                let level = match cluster_power.get(&cluster) {
                    Some(v) => level_of_volts(*v),
                    None => {
                        cluster_driver_level_ref(&self.slots, &cluster_of, cluster, Level::High)
                    }
                };
                NetState::Pulled(level, total)
            } else {
                NetState::Floating
            };
            net.state = state;
        }

        // -- findings ---------------------------------------------------------
        // Contention (per identity root, deduped).
        // hash-order shape 3: every `reported*` set below is a dedup gate —
        // `.insert()` returning false suppresses a duplicate. The findings
        // themselves are emitted while walking dense indices, so their order is
        // net order, not hash order.
        let mut reported_contention: HashSet<usize> = HashSet::new();
        for i in 0..n {
            let root = root_of[i];
            if nets[i].state == NetState::Contention && reported_contention.insert(root) {
                // Cross-root fights (through collapsed resistance) report
                // every fighting driver; direct fights report the root's
                // own. Ideal-source fights (a stuck fault vs a power rail)
                // have no driver pins to name — the finding carries the net.
                let drivers = contended
                    .get(&root)
                    .or_else(|| net_drivers.get(&root))
                    .map(|slots| slots.iter().map(|&si| self.slots[si].pin.clone()).collect())
                    .unwrap_or_default();
                diagnostics.report(Finding::Contention {
                    net: nets[root.min(i)].name.clone(),
                    drivers,
                });
            }
        }

        // Floating senses (deduped per (identity root, kind)).
        let mut reported: HashSet<(usize, SenseKind)> = HashSet::new();
        for (senses, kind) in [
            (&self.digital_senses, SenseKind::Digital),
            (&self.analog_senses, SenseKind::Analog),
        ] {
            for &net in senses {
                let root = root_of[net];
                if nets[net].state == NetState::Floating && reported.insert((root, kind)) {
                    diagnostics.report(Finding::FloatingSense {
                        net: nets[net].name.clone(),
                        kind,
                    });
                }
            }
        }

        // Power senses: unsourced clusters (deduped per identity root).
        let mut reported_power: HashSet<usize> = HashSet::new();
        for &net in &self.power_senses {
            let root = root_of[net];
            if !cluster_sourced.contains(&cluster_of[net]) && reported_power.insert(root) {
                diagnostics.report(Finding::PowerNetUnsourced {
                    net: nets[net].name.clone(),
                });
            }
        }
    }

    /// Derive the **pulse** routes from the current net topology — the
    /// step-clock analogue of [`Resolver::route_streams`], over the same
    /// collapsed-conduction reachability ([`STREAM_COLLAPSE_THRESHOLD`]), so a
    /// step signal that passes through series resistors or an isolator's
    /// short-circuit stub reaches the drive exactly like a byte route does.
    ///
    /// Two pulse sources reachable from each other raise
    /// [`Finding::StreamMismatch`] once per pair and neither routes: two step
    /// clocks driving one line is the same class of wiring error as two UART
    /// transmitters, and the underlying net additionally resolves
    /// `Contention` on its own.
    ///
    /// Runs wherever `route_streams` runs, from the same pass, so pulse routes
    /// can never outlive the topology they were derived from.
    pub(crate) fn route_pulses(
        &mut self,
        nets: &[Net],
        diagnostics: &mut Diagnostics,
    ) -> Vec<PulseRouteSpec> {
        self.identity.grow(self.net_count.max(nets.len()));
        let n = nets.len();
        let root_of: Vec<usize> = (0..n).map(|i| self.identity.find(i)).collect();
        let root_edges: Vec<(usize, usize, f64)> = self
            .edges
            .iter()
            .map(|(a, b, ohms)| (root_of[*a], root_of[*b], *ohms))
            .filter(|(a, b, _)| a != b)
            .collect();

        let mut routes = Vec::new();
        // hash-order shape 3: dedup gate for the paired mismatch report.
        let mut reported_pairs: HashSet<(usize, usize)> = HashSet::new();
        for (si, source) in self.streams.iter().enumerate() {
            if source.role != StreamRole::PulseSource {
                continue;
            }
            let origin = root_of[source.net];
            let dist = min_path_ohms(&root_edges, origin);
            let reachable = |net: usize| {
                dist.get(&root_of[net])
                    .is_some_and(|&ohms| ohms < STREAM_COLLAPSE_THRESHOLD)
            };

            let facing: Vec<usize> = self
                .streams
                .iter()
                .enumerate()
                .filter(|(oi, other)| {
                    *oi != si && other.role == StreamRole::PulseSource && reachable(other.net)
                })
                .map(|(oi, _)| oi)
                .collect();
            if !facing.is_empty() {
                for oi in facing {
                    let pair = (si.min(oi), si.max(oi));
                    if reported_pairs.insert(pair) {
                        diagnostics.report(Finding::StreamMismatch {
                            net: nets[origin].name.clone(),
                            producers: vec![
                                self.streams[pair.0].pin.clone(),
                                self.streams[pair.1].pin.clone(),
                            ],
                        });
                    }
                }
                continue;
            }

            let sinks: Vec<EndpointId> = self
                .streams
                .iter()
                .filter(|s| s.role == StreamRole::PulseSink && reachable(s.net))
                .map(|s| s.endpoint)
                .collect();
            // Same deliberately conservative gate as the byte routes: every
            // identity root within the collapse radius, sorted.
            // hash-order shape 2: `min_path_ohms` values are order-independent
            // and the collected keys are sorted here.
            let mut path_roots: Vec<usize> = dist
                .iter()
                .filter(|(_, &ohms)| ohms < STREAM_COLLAPSE_THRESHOLD)
                .map(|(&root, _)| root)
                .collect();
            path_roots.sort_unstable();
            routes.push(PulseRouteSpec {
                source: source.endpoint,
                sinks,
                path_roots,
            });
        }
        routes
    }
}

/// NaN-robust state equality for the sense change gate. [`NetState`]'s
/// derived `PartialEq` compares `f64` payloads with IEEE semantics, under
/// which `Analog(NaN) != Analog(NaN)` — a NaN-carrying state would read as
/// "changed" on every resolution pass, re-delivering senses forever (and a
/// sense callback that drives on delivery would then livelock the engine).
/// The resolver never publishes NaN (unmodeled rails are filtered before
/// state assignment), so this gate is defense in depth, not the primary
/// guarantee.
fn same_state(a: &NetState, b: &NetState) -> bool {
    match (a, b) {
        (NetState::Analog(x), NetState::Analog(y)) => x.total_cmp(y).is_eq(),
        (NetState::Pulled(la, xa), NetState::Pulled(lb, xb)) => {
            la == lb && xa.total_cmp(xb).is_eq()
        }
        _ => a == b,
    }
}

#[cfg(test)]
fn cluster_driver_level_ref(
    slots: &[DriveSlot],
    cluster_of: &[usize],
    cluster: usize,
    fallback: Level,
) -> Level {
    let mut level: Option<Level> = None;
    for slot in slots {
        let Some(drive) = slot.drive else { continue };
        if cluster_of[slot.net] != cluster {
            continue;
        }
        let this = level_of_volts(drive.volts);
        match level {
            None => level = Some(this),
            Some(seen) if seen == this => {}
            Some(_) => return fallback, // drivers disagree
        }
    }
    level.unwrap_or(fallback)
}

/// Digital projection of a source voltage (NaN — an unmodeled rail — never
/// reaches this: callers skip NaN sources).
/// The level every driver in a conduction cluster agrees on, or `fallback`
/// when the cluster has no drivers or its drivers disagree.
///
/// Disagreement is deliberately *not* reported as contention here: drivers on
/// one identity root are already checked for that, and drivers separated by
/// real resistance are what the escalated cluster solver exists to arbitrate.
/// This arm only runs when neither applied, so the honest answer is "no single
/// level", and the caller's fallback stands.
///
/// # What this is not
///
/// It is **distance-blind**, like the `Pulled` projection it feeds: every
/// driver in the conduction cluster votes equally, however much resistance
/// lies between it and the net being projected. And it is consulted only when
/// the cluster has no ideal source at all — a cluster that also touches a rail
/// or an injected fault takes that source's level regardless of what any
/// driver is doing.
///
/// Both are properties of the coarse `Pulled` path, not of this function: that
/// path reports the *sum* of the cluster's edge resistances as an upper bound
/// and makes no attempt at a divider. A cluster where the answer genuinely
/// depends on the ratio is one the impedance-escalation rule should hand to
/// [`QuasiStaticMna`], and this arm never runs for it.
fn cluster_driver_level(slots: &[DriveSlot], cluster_slots: &[usize], fallback: Level) -> Level {
    let mut level: Option<Level> = None;
    for &si in cluster_slots {
        let Some(drive) = slots[si].drive else {
            continue;
        };
        let this = level_of_volts(drive.volts);
        match level {
            None => level = Some(this),
            Some(seen) if seen == this => {}
            Some(_) => return fallback, // drivers disagree
        }
    }
    level.unwrap_or(fallback)
}

/// Digital projection of a source voltage (NaN — an unmodeled rail — never
/// reaches this: callers skip NaN sources).
fn level_of_volts(volts: Volts) -> Level {
    if volts >= DIGITAL_LEVEL_THRESHOLD_VOLTS {
        Level::High
    } else {
        Level::Low
    }
}

/// Minimum series resistance from `from` to every root reachable through the
/// identity-collapsed conduction edges (relaxation to fixpoint; edge weights
/// are non-negative, so this terminates).
fn min_path_ohms(root_edges: &[(usize, usize, f64)], from: usize) -> HashMap<usize, f64> {
    let mut dist: HashMap<usize, f64> = HashMap::new();
    dist.insert(from, 0.0);
    loop {
        let mut changed = false;
        for (a, b, ohms) in root_edges {
            if let Some(da) = dist.get(a).copied() {
                let candidate = da + ohms;
                if dist.get(b).is_none_or(|&db| candidate < db) {
                    dist.insert(*b, candidate);
                    changed = true;
                }
            }
            if let Some(db) = dist.get(b).copied() {
                let candidate = db + ohms;
                if dist.get(a).is_none_or(|&da| candidate < da) {
                    dist.insert(*a, candidate);
                    changed = true;
                }
            }
        }
        if !changed {
            break;
        }
    }
    dist
}

// ============================================================
// Timer wheel
// ============================================================

/// What one timer-wheel entry fires when its deadline passes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum TimerTarget {
    /// A component wakeup (`schedule_at` / `schedule_every`).
    Wake(ComponentId),
}

/// One armed wakeup. Ordered by `(deadline_ns, seq)` so simultaneous and
/// late deadlines fire in schedule order.
#[derive(Debug, Clone, Copy, Eq)]
struct TimerEntry {
    deadline_ns: u64,
    seq: u64,
    target: TimerTarget,
    /// `Some(period)` re-arms after firing (periodic wakes only); `None`
    /// is one-shot.
    period_ns: Option<u64>,
}

impl PartialEq for TimerEntry {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other) == std::cmp::Ordering::Equal
    }
}

impl Ord for TimerEntry {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        (self.deadline_ns, self.seq).cmp(&(other.deadline_ns, other.seq))
    }
}

impl PartialOrd for TimerEntry {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

// ============================================================
// Engine core (owned by the engine thread)
// ============================================================

/// Live per-source pulse route: the derived sinks and the delivery gate.
///
/// Deliberately has no queue and no pacing slot — a pulse channel carries a
/// *rate*, so there is nothing in flight between rate changes.
struct LivePulseRoute {
    /// Sink endpoints on the collapsed link.
    sinks: Vec<EndpointId>,
    /// Identity roots of the nets the link spans (delivery gate).
    path_roots: Vec<usize>,
}

/// All net state, owned exclusively by the engine thread. The only shared
/// pieces are the published `states` table and the cumulative `diagnostics`
/// bus — both locked only for the duration of a write, never across a
/// callback.
struct EngineCore {
    resolver: Resolver,
    nets: Vec<Net>,
    solver: Box<dyn ClusterSolver>,
    states: Arc<Mutex<Vec<NetState>>>,
    diagnostics: Arc<Mutex<Diagnostics>>,
    // hash-order: every map below is **keyed access only** — `get`, `entry`,
    // `insert`, `contains_key`. None is iterated. Sense delivery walks
    // `self.nets` by index and the per-net callbacks are a `Vec` in
    // registration order; `reroute_channels` walks `self.streams` in
    // registration order. Adding an iteration over any of these needs a sort
    // (see the module's review rule).
    sense_subs: HashMap<usize, Vec<SenseCallback>>,
    wake_subs: HashMap<usize, WakeCallback>,
    /// Live pulse routes, keyed by source endpoint index. Rebuilt on every
    /// routing pass.
    pulse_routes: HashMap<usize, LivePulseRoute>,
    /// Pulse-train subscriptions, keyed by sink endpoint index.
    pulse_subs: HashMap<usize, Vec<PulseCallback>>,
    /// The latest train published by each pulse source, keyed by source
    /// endpoint index. Retained across routing passes so a sink registering
    /// after the source published still learns the channel's current state
    /// (the once-at-registration contract `on_sense` honors).
    pulse_state: HashMap<usize, PulseTrain>,
    topology_observers: Vec<TopologyCallback>,
    topology_epoch: u64,
    wheel: BinaryHeap<Reverse<TimerEntry>>,
    /// `(component, deadline)` of every wake entry on the wheel that has not
    /// fired yet. A wake carries only its instant, so two entries for the
    /// same component at the same instant are one wake delivered twice --
    /// pure cost, and a compounding one: a callback that re-requests a
    /// deadline on every wake (a pulse train's next edge, a bus clock's next
    /// half period) then fires once per *previous* wake at that instant,
    /// each of which re-requests again. Measured before this set: ~500 wheel
    /// entries per stepper edge during an SD burst, a quarter of the engine
    /// thread in `BinaryHeap::pop`, and virtual time at 0.0001x.
    /// hash-order: keyed access only (`insert`, `remove`), never iterated.
    armed_wakes: HashSet<(usize, u64)>,
    timer_seq: u64,
    /// Drives received but not yet applicable (a lower enqueue seq is still
    /// in flight); applied strictly in seq order.
    pending_drives: BTreeMap<u64, (EndpointId, Option<TheveninDrive>)>,
    next_drive_seq: u64,
    /// Stepped mode: has [`Command::ReleaseTime`] arrived? Virtual time is held
    /// at its initial value until it does, so every component's first schedule
    /// is anchored at the same instant.
    clock_released: bool,
    /// Scheduling requests sent but not yet handled — see
    /// [`EngineLink::pending_schedules`]. Time does not advance while it is
    /// non-zero.
    pending_schedules: Arc<AtomicUsize>,
    /// Stepped mode: reported [`Finding::QuiescenceTimeout`] already? The
    /// finding is deduped on the cumulative bus anyway; this keeps the engine
    /// from re-waiting the full timeout on every iteration once an actor is
    /// known to be wedged.
    quiescence_stalled: bool,
    /// How long [`Self::await_actor_quiescence`] waits before declaring the
    /// barrier wedged ([`STEPPED_QUIESCENCE_TIMEOUT`] unless overridden).
    quiescence_timeout: Duration,
    /// Gap seq currently being watched, and how many consecutive idle polls
    /// have seen it. Skip after ~200 ms of idle (100 polls).
    stepped_gap_logged: Option<(u64, u32)>,
    /// Determinism Oracle 1 (`crate::event_log`). Disabled by default; every
    /// recording site is closure-guarded, so an off log costs one `Option`
    /// check.
    event_log: EventLog,
    /// Live edge-level fault injectors armed from scenario specs.
    edge_faults: Vec<EdgeFaultState>,
}

impl EngineCore {
    /// Report one finding straight onto the cumulative live bus, deduped
    /// like [`Self::merge_findings`] (once per distinct occurrence).
    fn report_finding(&self, finding: Finding) {
        let mut cumulative = self.diagnostics.lock().unwrap();
        if !cumulative.contains(&finding) {
            self.event_log
                .record(|| EngineEvent::Finding(finding.clone()));
            cumulative.report(finding);
        }
    }

    /// Invoke a component-provided callback with panic containment: a panic
    /// is caught, reported as a [`Finding::CallbackPanic`] naming the
    /// subscriber, and the engine thread stays alive — one misbehaving
    /// component must not silently end net service for every other
    /// component (net state would freeze at the last publication with the
    /// only symptom a join-time error, potentially hours later).
    fn deliver_contained(&self, kind: CallbackKind, subscriber: &str, deliver: impl FnOnce()) {
        if catch_unwind(AssertUnwindSafe(deliver)).is_err() {
            tracing::error!(
                kind = ?kind,
                subscriber,
                "component callback panicked; contained, engine continues"
            );
            self.report_finding(Finding::CallbackPanic {
                kind,
                subscriber: subscriber.to_string(),
            });
        }
    }

    /// True when `virtual_clock::init` has run. When it has not, the
    /// request that needed it is dropped loudly (error trace + structured
    /// finding) instead of letting `virtual_us` panic the engine thread
    /// into a silent zombie.
    fn clock_ready(&self, context: &str) -> bool {
        if virtual_clock::is_initialized() {
            return true;
        }
        tracing::error!(context, "virtual clock not initialized; request dropped");
        self.report_finding(Finding::VirtualClockUninitialized {
            context: context.to_string(),
        });
        false
    }

    /// One resolution pass: recompute all net states, publish them, merge
    /// new findings, then deliver sense callbacks for changed nets — with no
    /// lock held during delivery.
    fn resolve_and_publish(&mut self) {
        let old: Vec<NetState> = self.nets.iter().map(|n| n.state).collect();
        let mut pass = Diagnostics::new();
        self.resolver
            .resolve(&mut self.nets, &mut pass, self.solver.as_ref());

        {
            let mut shared = self.states.lock().unwrap();
            shared.clear();
            shared.extend(self.nets.iter().map(|n| n.state));
        }
        self.merge_findings(&pass);

        // Sense delivery: engine thread, no lock held. A callback that
        // drives a pin enqueues; the drive lands in a later iteration.
        //
        // The event log records the net-state *sequence* here — changed nets
        // only, walked in net index order — plus each sense delivery. Both are
        // in the set `DETERMINISM.md` claims T0 already determines (resolution
        // is pure; per-net callbacks are a `Vec` in registration order).
        for (i, net) in self.nets.iter().enumerate() {
            let changed = old.get(i).is_none_or(|prev| !same_state(prev, &net.state));
            if changed {
                self.event_log.record(|| EngineEvent::NetResolved {
                    net: NetId(i),
                    state: net.state,
                });
                if let Some(subs) = self.sense_subs.get(&i) {
                    for callback in subs {
                        self.event_log.record(|| EngineEvent::SenseDelivered {
                            net: NetId(i),
                            state: net.state,
                        });
                        self.deliver_contained(CallbackKind::Sense, &net.name, || {
                            callback(net.state);
                        });
                    }
                }
            }
        }
    }

    /// Merge one pass's findings into the cumulative live bus. The bus is
    /// cumulative: a finding is reported once per distinct occurrence, not
    /// once per pass.
    fn merge_findings(&self, pass: &Diagnostics) {
        if pass.is_empty() {
            return;
        }
        let mut cumulative = self.diagnostics.lock().unwrap();
        for finding in pass.findings() {
            // `report` is the novelty test: it returns true only the first
            // time a finding appears, which is also the only time it is worth
            // logging. Resolution re-reports every finding that still holds,
            // so logging unconditionally here is what produced 18 k lines/s.
            if cumulative.report(finding.clone()) {
                self.event_log
                    .record(|| EngineEvent::Finding(finding.clone()));
                Diagnostics::log(finding);
            }
        }
    }

    /// (Re-)derive the pulse routes from the current topology and merge the
    /// routing findings (`StreamMismatch`). Runs at spawn and on any
    /// topology-affecting change, so a route can never outlive the topology it
    /// was derived from.
    ///
    /// Pulse routes carry nothing in flight (a rate, not a queue), so a
    /// re-derivation loses no pulses: each source's current train is retained
    /// in `pulse_state` and handed to any sink that registers afterwards.
    fn reroute_channels(&mut self) {
        let mut pass = Diagnostics::new();
        let pulse_specs = self.resolver.route_pulses(&self.nets, &mut pass);
        self.merge_findings(&pass);
        let epoch = self.topology_epoch;
        self.event_log.record(|| EngineEvent::Reroute { epoch });
        self.pulse_routes = pulse_specs
            .into_iter()
            .map(|spec| {
                (
                    spec.source.0,
                    LivePulseRoute {
                        sinks: spec.sinks,
                        path_roots: spec.path_roots,
                    },
                )
            })
            .collect();
    }

    /// Publish a pulse source's new constant-rate segment: retain it as the
    /// channel's current state, then deliver it to every routed sink.
    ///
    /// Gated by net resolution exactly as stream bytes are — a link whose nets
    /// resolve `Contention`/`Floating` cannot carry a clean step clock, so the
    /// train is retained but not delivered (see [`PulseTrain`]'s fidelity
    /// limits for what that does and does not model).
    fn pulse_update(&mut self, source: EndpointId, train: PulseTrain) {
        self.pulse_state.insert(source.0, train);
        let Some(route) = self.pulse_routes.get(&source.0) else {
            tracing::debug!(
                endpoint = source.0,
                "pulse train not delivered: source has no valid route"
            );
            return;
        };
        let sinks = route.sinks.clone();
        let path_roots = route.path_roots.clone();
        if !self.route_is_signal_capable(&path_roots) {
            tracing::debug!(
                endpoint = source.0,
                "pulse train not delivered: a net on the route is not signal-capable"
            );
            return;
        }
        for sink in sinks {
            self.deliver_pulse_to(source, sink, train);
        }
    }

    /// Deliver one train to one sink's callbacks, panic-contained, recording
    /// the wire event.
    fn deliver_pulse_to(&self, source: EndpointId, sink: EndpointId, train: PulseTrain) {
        let Some(subs) = self.pulse_subs.get(&sink.0) else {
            return;
        };
        self.event_log.record(|| EngineEvent::PulseUpdate {
            source,
            sink,
            train,
        });
        let subscriber = self
            .resolver
            .streams
            .iter()
            .find(|s| s.endpoint == sink)
            .map(|s| format!("{}.{}", s.pin.reference, s.pin.pin))
            .unwrap_or_else(|| format!("pulse sink endpoint {}", sink.0));
        for callback in subs {
            self.deliver_contained(CallbackKind::Pulse, &subscriber, || callback(train));
        }
    }

    /// Whether every net a derived route spans currently projects a usable
    /// signal (the shared byte/pulse delivery gate).
    fn route_is_signal_capable(&self, path_roots: &[usize]) -> bool {
        path_roots.iter().all(|&root| {
            matches!(
                self.nets.get(root).map(|net| net.state),
                Some(NetState::Driven(_) | NetState::Pulled(_, _) | NetState::Analog(_))
            )
        })
    }

    /// Apply buffered drives strictly in enqueue-seq order, resolving after
    /// each so senses observe the authoritative event order.
    ///
    /// Only the cluster the drive belongs to is re-resolved (every other
    /// net's state is provably unchanged), and a drive identical to the one
    /// already held resolves nothing at all.
    fn apply_ready_drives(&mut self) {
        while let Some((endpoint, drive)) = self.pending_drives.remove(&self.next_drive_seq) {
            let seq = self.next_drive_seq;
            self.next_drive_seq += 1;
            self.event_log.record(|| EngineEvent::DriveApplied {
                seq,
                endpoint,
                drive,
            });
            let drive_changed = self.resolver.set_drive(endpoint, drive);
            // Mask/stuck for *this* edge (index == seen), resolve, then tick.
            let net = self.resolver.endpoint_net(endpoint);
            let fault_changed = self.sync_edge_faults();
            if drive_changed || fault_changed {
                self.resolve_and_publish_dirty();
            }
            if let Some(net) = net {
                for fault in &mut self.edge_faults {
                    if fault.matches(endpoint, net) {
                        fault.tick();
                    }
                }
            }
        }
    }

    /// Push the current injector window onto the resolver (Float masks +
    /// Stuck/Contention ideal sources). Returns whether resolution must run.
    fn sync_edge_faults(&mut self) -> bool {
        let mut float_endpoints: Vec<EndpointId> = Vec::new();
        let mut suppressed_now: Vec<EndpointId> = Vec::new();
        let mut stuck: Vec<(usize, Volts)> = Vec::new();
        for fault in &self.edge_faults {
            let active = fault.active();
            match fault.spec.kind {
                EdgeFaultKind::Float => {
                    if let Some(ep) = fault.spec.endpoint {
                        float_endpoints.push(ep);
                        if active {
                            suppressed_now.push(ep);
                        }
                    } else {
                        // Net-targeted Float: suppress every slot on that net.
                        for ep in self.resolver.endpoints_on_net(fault.spec.net) {
                            float_endpoints.push(ep);
                            if active {
                                suppressed_now.push(ep);
                            }
                        }
                    }
                }
                EdgeFaultKind::Stuck(volts) => {
                    // Force the level: suppress the target pin (if any) and
                    // inject an ideal source — same algebra as net_stuck.
                    if let Some(ep) = fault.spec.endpoint {
                        float_endpoints.push(ep);
                        if active {
                            suppressed_now.push(ep);
                        }
                    }
                    if active {
                        stuck.push((fault.spec.net, volts));
                    }
                }
                EdgeFaultKind::Contention(volts) => {
                    // Leave drivers in place; the ideal source fights them.
                    if active {
                        stuck.push((fault.spec.net, volts));
                    }
                }
            }
        }
        float_endpoints.sort_by_key(|ep| ep.0);
        float_endpoints.dedup();
        suppressed_now.sort_by_key(|ep| ep.0);
        suppressed_now.dedup();
        self.resolver
            .apply_edge_fault_effects(&float_endpoints, &suppressed_now, &stuck)
    }

    /// [`Self::resolve_and_publish`] scoped to the clusters the drives since
    /// the last pass touched: the same resolution, publication and delivery,
    /// over the nets that can have changed.
    fn resolve_and_publish_dirty(&mut self) {
        let scope = self.resolver.dirty_scope(self.nets.len());
        if scope.is_empty() {
            return;
        }
        let old: Vec<NetState> = scope.iter().map(|&i| self.nets[i].state).collect();
        let mut pass = Diagnostics::new();
        self.resolver
            .resolve_dirty(&mut self.nets, &mut pass, self.solver.as_ref());

        {
            let mut shared = self.states.lock().unwrap();
            if shared.len() == self.nets.len() {
                for &i in &scope {
                    shared[i] = self.nets[i].state;
                }
            } else {
                shared.clear();
                shared.extend(self.nets.iter().map(|n| n.state));
            }
        }
        self.merge_findings(&pass);

        // Changed nets in ascending index order, per-net callbacks in
        // registration order — the sequence the full pass records.
        for (k, &i) in scope.iter().enumerate() {
            let net = &self.nets[i];
            if same_state(&old[k], &net.state) {
                continue;
            }
            self.event_log.record(|| EngineEvent::NetResolved {
                net: NetId(i),
                state: net.state,
            });
            if let Some(subs) = self.sense_subs.get(&i) {
                for callback in subs {
                    self.event_log.record(|| EngineEvent::SenseDelivered {
                        net: NetId(i),
                        state: net.state,
                    });
                    self.deliver_contained(CallbackKind::Sense, &net.name, || {
                        callback(net.state);
                    });
                }
            }
        }
    }

    /// Fire every wheel entry whose deadline has passed, in `(deadline,
    /// schedule)` order; returns how many fired.
    ///
    /// `now` is the instant the engine last advanced to, so a wake fires at
    /// its scheduled deadline. Stream targets deliver their route's due
    /// bytes instead of a wake callback.
    fn fire_due_timers(&mut self) -> usize {
        let mut fired = 0usize;
        while let Some(&Reverse(head)) = self.wheel.peek() {
            let now = virtual_clock::virtual_ns();
            if head.deadline_ns > now {
                break;
            }
            self.wheel.pop();
            fired += 1;
            match head.target {
                TimerTarget::Wake(component) => {
                    // Cleared before delivery: a callback asking to be woken
                    // again at this same instant is asking for a *new* wake.
                    // A second entry for the same instant (a periodic one
                    // pushed beside a one-shot, or a re-request delivered
                    // under an earlier entry) finds the key gone and is not
                    // delivered; its period, if any, still re-arms below.
                    let deliver = self.armed_wakes.remove(&(component.0, head.deadline_ns));
                    if !deliver {
                        // fall through to the periodic re-arm
                    } else if let Some(callback) = self.wake_subs.get(&component.0) {
                        self.event_log.record(|| EngineEvent::Wake { component });
                        let subscriber = format!("component {}", component.0);
                        self.deliver_contained(CallbackKind::Wake, &subscriber, || {
                            callback(now);
                        });
                    } else {
                        tracing::debug!(
                            component = component.0,
                            "timer fired for a component with no wake handler"
                        );
                    }
                    if let Some(period) = head.period_ns {
                        let mut next = head.deadline_ns.saturating_add(period);
                        if next <= now {
                            next = now.saturating_add(period);
                        }
                        self.arm(next, head.target, Some(period));
                    }
                }
            }
        }
        fired
    }

    /// Push a wheel entry.
    fn arm(&mut self, deadline_ns: u64, target: TimerTarget, period_ns: Option<u64>) {
        // One wake per component per instant (see `armed_wakes`). A periodic
        // entry is always pushed: it carries the period the chain continues
        // on, which a one-shot already armed at the same instant does not.
        let TimerTarget::Wake(component) = target;
        let fresh = self.armed_wakes.insert((component.0, deadline_ns));
        if !fresh && period_ns.is_none() {
            return;
        }
        let seq = self.timer_seq;
        self.timer_seq += 1;
        self.wheel.push(Reverse(TimerEntry {
            deadline_ns,
            seq,
            target,
            period_ns,
        }));
    }

    /// Handle one command; returns `true` on shutdown.
    fn handle(&mut self, command: Command) -> bool {
        match command {
            Command::Drive {
                seq,
                endpoint,
                drive,
            } => {
                self.pending_drives.insert(seq, (endpoint, drive));
                self.apply_ready_drives();
            }
            Command::RegisterSense { net, callback } => {
                let state = self
                    .nets
                    .get(net.0)
                    .map(|n| n.state)
                    .unwrap_or(NetState::Floating);
                let subscriber = self
                    .nets
                    .get(net.0)
                    .map(|n| n.name.clone())
                    .unwrap_or_else(|| format!("net {}", net.0));
                // Deliver the current state once at registration, so e.g. a
                // floating ~RESET is reported before any traffic.
                self.event_log
                    .record(|| EngineEvent::SenseDelivered { net, state });
                self.deliver_contained(CallbackKind::Sense, &subscriber, || callback(state));
                self.sense_subs.entry(net.0).or_default().push(callback);
            }
            Command::RegisterWake {
                component,
                callback,
            } => {
                self.wake_subs.insert(component.0, callback);
            }
            Command::ScheduleAt { component, at_ns } => {
                // Wheel entries make the run loop read the virtual clock;
                // gate here (and at every other arm entry point) so a
                // missing init fails the request, never the engine thread.
                if self.clock_ready("schedule_at") {
                    self.arm(at_ns, TimerTarget::Wake(component), None);
                }
            }
            Command::ScheduleEvery {
                component,
                period_ns,
            } => {
                if period_ns == 0 {
                    tracing::warn!(component = component.0, "schedule_every(0) ignored");
                } else if self.clock_ready("schedule_every") {
                    let now = virtual_clock::virtual_ns();
                    self.arm(
                        now.saturating_add(period_ns),
                        TimerTarget::Wake(component),
                        Some(period_ns),
                    );
                }
            }
            Command::RegisterTopologyObserver { callback } => {
                let epoch = self.topology_epoch;
                self.deliver_contained(CallbackKind::Topology, "topology observer", || {
                    callback(epoch);
                });
                self.topology_observers.push(callback);
            }
            Command::PulseUpdate { endpoint, train } => {
                self.pulse_update(endpoint, train);
            }
            Command::RegisterPulseSink { endpoint, callback } => {
                self.pulse_subs
                    .entry(endpoint.0)
                    .or_default()
                    .push(callback);
                // Once-at-registration delivery, mirroring `RegisterSense`: a
                // sink that attaches after its source published must not wait
                // for the next rate change to learn the channel's state.
                // hash-order: `pulse_routes` is walked in *source endpoint*
                // order, and at most one source routes to a given sink (two
                // would have raised StreamMismatch and neither would route),
                // so the sort only pins which "cannot happen" case wins.
                let mut sources: Vec<usize> = self
                    .pulse_routes
                    .iter()
                    .filter(|(_, route)| route.sinks.contains(&endpoint))
                    .map(|(&source, _)| source)
                    .collect();
                sources.sort_unstable();
                for source in sources {
                    if let Some(&train) = self.pulse_state.get(&source) {
                        self.deliver_pulse_to(EndpointId(source), endpoint, train);
                    }
                }
            }
            Command::ReleaseTime => {
                self.clock_released = true;
            }
            Command::Shutdown => return true,
        }
        false
    }

    /// Engine thread body. Virtual time is a counter; this loop is the time
    /// authority. `--speed` only paces the host after
    /// [`embsim_core::virtual_clock::advance_to`].
    fn run(mut self, rx: Receiver<Command>, control_rx: Receiver<Command>) {
        loop {
            if self.run_stepped_iteration(&rx, &control_rx) {
                break;
            }
        }
    }

    /// Handle every control-plane command waiting, with no cap: a deadline the
    /// engine has not armed is one it can step over. Returns the number
    /// handled, and `true` on shutdown.
    fn drain_control(&mut self, control_rx: &Receiver<Command>) -> (usize, bool) {
        let mut handled = 0usize;
        loop {
            match control_rx.try_recv() {
                Ok(command) => {
                    handled += 1;
                    self.pending_schedules.fetch_sub(1, Ordering::AcqRel);
                    if self.handle(command) {
                        return (handled, true);
                    }
                }
                Err(mpsc::TryRecvError::Empty) => return (handled, false),
                Err(mpsc::TryRecvError::Disconnected) => return (handled, true),
            }
        }
    }

    /// One iteration — the engine as the **time authority**
    /// (`DETERMINISM.md` T1 §3). Returns `true` to stop.
    ///
    /// 1. **Quiesce.** Wait for every registered actor to park. Only then is
    ///    the set of pending work stable enough to reason about.
    /// 2. **Drain the control queue in full**, then the data queue, at most
    ///    [`COMMAND_DRAIN_BATCH_MAX`] per pass. A live flood can keep the data
    ///    queue non-empty forever; the cap lets a future wheel deadline still
    ///    jump. Scheduling is never capped — see
    ///    [`EngineLink::control_tx`] for why the two planes are separate.
    /// 3. **Fire every wheel entry due at `now`**, in `(deadline, seq)` order.
    /// 4. Loop 1–3 to a fixpoint, or break after a full batch when the next
    ///    deadline is in the future so time can move.
    /// 5. **Advance** to `min(wheel head, earliest pending park deadline)`.
    ///    With nothing to advance to, park on the command queue — a scripted
    ///    stimulus thread is not an actor, and waiting for it is the normal
    ///    idle state, not a wedge (see "Deviations from the design doc" in
    ///    `DETERMINISM.md`).
    ///
    /// The resulting per-instant order is fixed: released actors run to their
    /// next park first, then the engine drains and fires. What is *not* fixed
    /// is the interleaving of two actors released at the same instant — they
    /// race `next_drive_seq`, exactly as `DETERMINISM.md` says T1 does not fix.
    fn run_stepped_iteration(
        &mut self,
        rx: &Receiver<Command>,
        control_rx: &Receiver<Command>,
    ) -> bool {
        let mut capped_passes = 0usize;
        loop {
            self.await_actor_quiescence();
            let mut work = 0usize;
            let mut drained = 0usize;
            let (control, stop) = self.drain_control(control_rx);
            if stop {
                return true;
            }
            work += control;
            loop {
                if drained >= COMMAND_DRAIN_BATCH_MAX {
                    break;
                }
                match rx.try_recv() {
                    Ok(command) => {
                        work += 1;
                        drained += 1;
                        if self.handle(command) {
                            return true;
                        }
                    }
                    Err(mpsc::TryRecvError::Empty) => break,
                    Err(mpsc::TryRecvError::Disconnected) => return true,
                }
            }
            work += self.fire_due_timers();
            if work == 0 {
                break;
            }
            if drained >= COMMAND_DRAIN_BATCH_MAX {
                capped_passes += 1;
                if capped_passes >= COMMAND_DRAIN_CAPPED_PASSES_MAX {
                    let now = virtual_clock::virtual_ns();
                    if self
                        .next_virtual_deadline()
                        .is_some_and(|deadline| deadline > now)
                    {
                        break;
                    }
                }
            } else {
                capped_passes = 0;
            }
        }

        // The control queue is drained in full above, so anything still
        // counted here is a request whose send has not landed yet — a window
        // a few instructions wide. Hold time rather than step over a deadline
        // that is already on its way; that is how a whole UART byte ends up on
        // the wire at a single instant.
        //
        // Wait for it rather than spinning: the enqueuer is between its
        // increment and its `send`, so blocking on the control queue is
        // exactly the right thing to block on.
        if self.pending_schedules.load(Ordering::Acquire) > 0 {
            return match control_rx.recv_timeout(STEPPED_IDLE_POLL) {
                Ok(command) => {
                    self.pending_schedules.fetch_sub(1, Ordering::AcqRel);
                    self.handle(command)
                }
                Err(RecvTimeoutError::Timeout) => false,
                Err(RecvTimeoutError::Disconnected) => true,
            };
        }
        let now = virtual_clock::virtual_ns();
        let next = match (self.clock_released, self.next_virtual_deadline()) {
            // Time is held until the system has finished assembling.
            (false, _) => None,
            (true, next) => next,
        };
        match next {
            Some(deadline) if deadline > now => {
                if let Err(error) = virtual_clock::advance_to_ns(deadline) {
                    // Unreachable while this engine is the only scheduler:
                    // `next_virtual_deadline` is >= now by construction and the
                    // mode was stepped one instruction ago. Report rather than
                    // panic — an engine thread that dies is a silent zombie.
                    tracing::error!(?error, deadline, now, "stepped clock advance rejected");
                }
                false
            }
            // Something is already due at `now`: the fixpoint loop above will
            // fire it on the next pass.
            Some(_) => false,
            None => {
                self.warn_on_stepped_drive_gap();
                // Two queues, no combined wait: block on the data queue for a
                // short poll, but take a control command first if one is
                // already there. A control command that arrives mid-park waits
                // out at most one `STEPPED_IDLE_POLL`, which is the same
                // latency the data queue has always had while idle.
                let (control, stop) = self.drain_control(control_rx);
                if stop {
                    return true;
                }
                if control > 0 {
                    return false;
                }
                match rx.recv_timeout(STEPPED_IDLE_POLL) {
                    Ok(command) => self.handle(command),
                    Err(RecvTimeoutError::Timeout) => false,
                    Err(RecvTimeoutError::Disconnected) => true,
                }
            }
        }
    }

    /// Skip a reserved-but-never-sent drive seq after it has survived many
    /// idle polls. A live enqueuer covers reserve→send in nanoseconds; one
    /// or two idle observations can still be a healthy race, so wait
    /// [`GAP_IDLE_POLLS`] (~200 ms) before skipping.
    fn warn_on_stepped_drive_gap(&mut self) {
        let Some((&lowest, _)) = self.pending_drives.first_key_value() else {
            self.stepped_gap_logged = None;
            return;
        };
        let missing = self.next_drive_seq;
        const GAP_IDLE_POLLS: u32 = 100; // ~200 ms at STEPPED_IDLE_POLL
        match self.stepped_gap_logged {
            Some((seq, n)) if seq == missing => {
                if n + 1 < GAP_IDLE_POLLS {
                    self.stepped_gap_logged = Some((missing, n + 1));
                    return;
                }
            }
            _ => {
                self.stepped_gap_logged = Some((missing, 1));
                return;
            }
        }
        tracing::error!(
            missing_seq = missing,
            lowest_buffered = lowest,
            buffered = self.pending_drives.len(),
            "idle with drives buffered behind a missing enqueue seq; skipping the gap"
        );
        self.report_finding(Finding::DriveSeqGap { seq: missing });
        self.next_drive_seq = lowest;
        self.apply_ready_drives();
        self.stepped_gap_logged = None;
    }

    /// Stepped mode: block until every registered actor is parked, reporting a
    /// [`Finding::QuiescenceTimeout`] if one never does.
    ///
    /// An actor that never parks cannot be waited out — it would hang the
    /// engine — and it cannot be stepped over either, because whatever it is
    /// about to do belongs at the current instant. So the engine reports it
    /// loudly and carries on with a *degraded* guarantee: the finding is the
    /// marker that this run is no longer reproducible.
    fn await_actor_quiescence(&mut self) {
        if self.quiescence_stalled {
            return; // already reported; do not re-wait the full timeout
        }
        match virtual_clock::await_quiescence(self.quiescence_timeout) {
            virtual_clock::Quiescence::Reached { .. } => {}
            virtual_clock::Quiescence::Stalled { actors } => {
                tracing::error!(
                    ?actors,
                    "stepped clock: actor(s) never parked at a virtual deadline; \
                     virtual time is advancing without them and this run is NOT \
                     reproducible (DETERMINISM.md T1 §4)"
                );
                self.quiescence_stalled = true;
                self.report_finding(Finding::QuiescenceTimeout { actors });
            }
        }
    }

    /// Stepped mode: the next virtual instant (ns) anything is waiting for — the
    /// earlier of the wheel head and the earliest pending park deadline across
    /// every waiter (registered actor or not; an unregistered waiter cannot
    /// hold time back, but it must still be released).
    fn next_virtual_deadline(&self) -> Option<u64> {
        let wheel = self.wheel.peek().map(|&Reverse(head)| head.deadline_ns);
        let parked = virtual_clock::scheduler_state().next_deadline_ns;
        match (wheel, parked) {
            (Some(a), Some(b)) => Some(a.min(b)),
            (a, b) => a.or(b),
        }
    }
}

// ============================================================
// Engine handle
// ============================================================

/// Owning handle to a running net-engine thread.
///
/// Created by `System::start`. Dropping the handle sends a shutdown command
/// and joins the engine thread — in-flight sense/wake callbacks complete
/// first, and joining cannot deadlock because callbacks are delivered with
/// no engine lock held and drives from callbacks never block.
#[derive(Debug)]
pub struct EngineHandle {
    link: EngineLink,
    diagnostics: Arc<Mutex<Diagnostics>>,
    event_log: EventLog,
    join: Option<JoinHandle<()>>,
    _time: virtual_clock::TimeAuthority,
}

impl EngineHandle {
    /// Start the engine thread over an assembled topology. The initial full
    /// resolution pass **and** the initial stream-routing pass run
    /// synchronously *before* the thread starts, so never-driven nets are
    /// reported and routing findings (`StreamMismatch`) are populated by the
    /// time this returns — before any traffic. Those two passes therefore land
    /// in `event_log` from the *calling* thread, which is still single-writer:
    /// the engine thread does not exist yet.
    ///
    /// # Panics
    /// Panics if the OS refuses to spawn the engine thread.
    pub(crate) fn spawn(
        resolver: Resolver,
        nets: Vec<Net>,
        solver: Box<dyn ClusterSolver>,
        event_log: EventLog,
        quiescence_timeout: Option<Duration>,
    ) -> Self {
        let states: Arc<Mutex<Vec<NetState>>> =
            Arc::new(Mutex::new(nets.iter().map(|n| n.state).collect()));
        let diagnostics = Arc::new(Mutex::new(Diagnostics::new()));
        let pending_schedules: Arc<AtomicUsize> = Arc::new(AtomicUsize::new(0));
        let (tx, rx) = mpsc::channel();
        let (control_tx, control_rx) = mpsc::channel();

        let core_edge_faults: Vec<EdgeFaultState> = resolver
            .edge_faults()
            .iter()
            .cloned()
            .map(EdgeFaultState::new)
            .collect();
        let mut core = EngineCore {
            resolver,
            nets,
            solver,
            states: Arc::clone(&states),
            diagnostics: Arc::clone(&diagnostics),
            sense_subs: HashMap::new(),
            wake_subs: HashMap::new(),
            pulse_routes: HashMap::new(),
            pulse_subs: HashMap::new(),
            pulse_state: HashMap::new(),
            topology_observers: Vec::new(),
            topology_epoch: 0,
            wheel: BinaryHeap::new(),
            armed_wakes: HashSet::new(),
            timer_seq: 0,
            pending_drives: BTreeMap::new(),
            next_drive_seq: 0,
            clock_released: false,
            pending_schedules: Arc::clone(&pending_schedules),
            quiescence_stalled: false,
            quiescence_timeout: quiescence_timeout.unwrap_or(STEPPED_QUIESCENCE_TIMEOUT),
            stepped_gap_logged: None,
            event_log: event_log.clone(),
            edge_faults: core_edge_faults,
        };
        // Arm injectors before the first resolve so after_edges=0 windows
        // cover idle / pre-drive state (Stuck/Contention) correctly.
        core.sync_edge_faults();
        core.resolve_and_publish();
        // Byte pipes and pulse routes are derived from net resolution, never
        // installed beside it: the routing pass runs against the just-resolved
        // nets.
        core.reroute_channels();

        let time_authority = virtual_clock::take_time_authority();

        let join = std::thread::Builder::new()
            .name("embsim-board-net-engine".to_string())
            .spawn(move || core.run(rx, control_rx))
            .expect("failed to spawn the net-engine thread");

        Self {
            link: EngineLink {
                tx: Some(tx),
                control_tx: Some(control_tx),
                drive_seq: Arc::new(AtomicU64::new(0)),
                pending_schedules,
                states,
                // Live path: drives go to the engine, never to a log.
                recorded_drives: None,
            },
            diagnostics,
            event_log,
            join: Some(join),
            _time: time_authority,
        }
    }

    /// Cloneable client link for attaching components.
    pub(crate) fn link(&self) -> EngineLink {
        self.link.clone()
    }

    /// Tell the engine the system is fully assembled, so virtual time may
    /// begin advancing ([`Command::ReleaseTime`]). Called once by
    /// `System::start` after every component has attached and started.
    pub(crate) fn release_time(&self) {
        self.link.send(Command::ReleaseTime);
    }

    /// Handle to this engine's determinism event log (`crate::event_log`).
    /// Reads empty when the log was never enabled.
    pub fn event_log(&self) -> EventLog {
        self.event_log.clone()
    }

    /// Most recently published state of one net.
    pub fn net_state(&self, net: NetId) -> Option<NetState> {
        self.link.states.lock().unwrap().get(net.0).copied()
    }

    /// Snapshot of the cumulative live findings (initial resolution pass
    /// included).
    pub fn findings(&self) -> Vec<Finding> {
        self.diagnostics.lock().unwrap().findings().to_vec()
    }

    /// True while the engine thread is alive and serving commands.
    /// Component callbacks are panic-contained, so this going false means
    /// the engine itself failed — net state is frozen at its last
    /// publication and every drive/schedule is being dropped. Consumers
    /// and tests use this to detect engine death promptly instead of at
    /// drop-join time.
    pub fn is_alive(&self) -> bool {
        self.join.as_ref().is_some_and(|join| !join.is_finished())
    }

    /// Stream-routing seam (later slice): observe net-graph topology
    /// changes. The observer runs on the engine thread with no engine lock
    /// held; the current epoch is delivered once at registration, and the
    /// engine will notify on every future topology-affecting change (jumper
    /// toggles, fault injection, harness swaps) once live mutation lands.
    #[allow(dead_code)] // stream-routing slice consumes this seam
    pub(crate) fn subscribe_topology(&self, callback: TopologyCallback) {
        self.link
            .send(Command::RegisterTopologyObserver { callback });
    }
}

impl Drop for EngineHandle {
    fn drop(&mut self) {
        if let Some(tx) = self.link.tx.take() {
            // Ignore the error: the thread may already have exited.
            let _ = tx.send(Command::Shutdown);
        }
        if let Some(join) = self.join.take() {
            if join.join().is_err() {
                tracing::error!("net-engine thread panicked");
            }
        }
    }
}

// ============================================================
// Tests
// ============================================================

#[cfg(test)]
mod tests {
    use rstest::rstest;

    use super::*;
    use crate::cluster::QuasiStaticMna;
    use std::sync::atomic::AtomicBool;
    use std::sync::Mutex as StdMutex;
    use std::time::Instant;

    /// Timer tests re-anchor the process-global virtual clock; serialize
    /// them (poison-recovering, like the `virtual_clock` reference suite).
    static CLOCK_LOCK: StdMutex<()> = StdMutex::new(());

    fn lock_clock() -> std::sync::MutexGuard<'static, ()> {
        CLOCK_LOCK.lock().unwrap_or_else(|p| {
            CLOCK_LOCK.clear_poison();
            p.into_inner()
        })
    }

    fn wait_for(mut pred: impl FnMut() -> bool, timeout: Duration) -> bool {
        let start = Instant::now();
        while start.elapsed() < timeout {
            if pred() {
                return true;
            }
            std::thread::sleep(Duration::from_millis(1));
        }
        pred()
    }

    fn nets(count: usize) -> Vec<Net> {
        (0..count)
            .map(|i| Net {
                id: NetId(i),
                name: format!("N{i}"),
                nodes: Vec::new(),
                state: NetState::Floating,
            })
            .collect()
    }

    fn high() -> TheveninDrive {
        TheveninDrive {
            volts: 3.3,
            impedance: 25.0,
        }
    }

    fn low() -> TheveninDrive {
        TheveninDrive {
            volts: 0.0,
            impedance: 25.0,
        }
    }

    fn sense_log(handle: &EngineHandle, net: NetId) -> Arc<StdMutex<Vec<NetState>>> {
        let log: Arc<StdMutex<Vec<NetState>>> = Arc::new(StdMutex::new(Vec::new()));
        let sink = Arc::clone(&log);
        handle.link().send(Command::RegisterSense {
            net,
            callback: Box::new(move |state| sink.lock().unwrap().push(state)),
        });
        log
    }

    /// Drives are applied in enqueue-seq order — the authoritative event
    /// order — even when they arrive on the channel out of order (two racing
    /// enqueuers can interleave reserve-then-send).
    #[rstest]
    fn drives_apply_in_enqueue_seq_order_despite_arrival_order() {
        let mut resolver = Resolver::new(1, Dsu::new(1));
        let e0 = resolver.add_endpoint(0, PinRef::new("U1", "1"), None);
        let e1 = resolver.add_endpoint(0, PinRef::new("U2", "1"), None);
        let handle = EngineHandle::spawn(
            resolver,
            nets(1),
            Box::new(QuasiStaticMna),
            EventLog::disabled(),
            None,
        );
        let log = sense_log(&handle, NetId(0));

        // seq 1 arrives FIRST; the engine must hold it until seq 0 lands.
        handle.link().send(Command::Drive {
            seq: 1,
            endpoint: e1,
            drive: Some(low()),
        });
        std::thread::sleep(Duration::from_millis(20));
        assert_eq!(
            handle.net_state(NetId(0)),
            Some(NetState::Floating),
            "an out-of-order drive must not be applied early"
        );
        handle.link().send(Command::Drive {
            seq: 0,
            endpoint: e0,
            drive: Some(high()),
        });

        assert!(
            wait_for(|| log.lock().unwrap().len() == 3, Duration::from_secs(5)),
            "expected registration + two resolutions; got {:?}",
            log.lock().unwrap()
        );
        assert_eq!(
            *log.lock().unwrap(),
            vec![
                NetState::Floating,            // delivered at registration
                NetState::Driven(Level::High), // seq 0 applied first
                NetState::Contention,          // then seq 1
            ]
        );
        // The contention finding fired on the live bus.
        assert!(handle
            .findings()
            .iter()
            .any(|f| matches!(f, Finding::Contention { net, .. } if net == "N0")));
    }

    /// Two threads racing the public drive path: every drive is applied
    /// individually (no coalescing, no loss) in a valid enqueue order.
    #[rstest]
    fn racing_public_drives_all_apply_individually() {
        let mut resolver = Resolver::new(1, Dsu::new(1));
        let e0 = resolver.add_endpoint(0, PinRef::new("U1", "1"), None);
        let e1 = resolver.add_endpoint(0, PinRef::new("U2", "1"), None);
        let handle = EngineHandle::spawn(
            resolver,
            nets(1),
            Box::new(QuasiStaticMna),
            EventLog::disabled(),
            None,
        );
        let log = sense_log(&handle, NetId(0));

        let link = handle.link();
        let h0 = crate::component::PinHandle::wired(NetId(0), Some(e0), None, link.clone());
        let h1 = crate::component::PinHandle::wired(NetId(0), Some(e1), None, link);
        let t0 = std::thread::spawn(move || h0.set_drive(Some(high())));
        let t1 = std::thread::spawn(move || h1.set_drive(Some(low())));
        t0.join().unwrap();
        t1.join().unwrap();

        assert!(
            wait_for(|| log.lock().unwrap().len() == 3, Duration::from_secs(5)),
            "both drives must resolve individually; got {:?}",
            log.lock().unwrap()
        );
        let observed = log.lock().unwrap().clone();
        assert_eq!(observed[0], NetState::Floating);
        assert!(
            observed[1] == NetState::Driven(Level::High)
                || observed[1] == NetState::Driven(Level::Low),
            "first-applied drive resolves alone: {observed:?}"
        );
        assert_eq!(observed[2], NetState::Contention);
        assert_eq!(handle.net_state(NetId(0)), Some(NetState::Contention));
    }

    /// The re-entrancy contract: a sense callback drives a pin; the drive is
    /// enqueued (NOT applied inline) and lands in a later engine iteration.
    /// The full driver → net → sense → drive loop converges without
    /// deadlocking.
    #[rstest]
    fn sense_callback_drive_is_enqueued_not_inline_and_loop_converges() {
        let mut resolver = Resolver::new(2, Dsu::new(2));
        let e_a = resolver.add_endpoint(0, PinRef::new("U1", "1"), None);
        let e_a2 = resolver.add_endpoint(0, PinRef::new("U3", "1"), None);
        let e_b = resolver.add_endpoint(1, PinRef::new("U2", "1"), None);
        let handle = EngineHandle::spawn(
            resolver,
            nets(2),
            Box::new(QuasiStaticMna),
            EventLog::disabled(),
            None,
        );
        let link = handle.link();

        // Sense on net A: on High, snapshot net B (must still be un-driven —
        // proof the feedback drive is not applied inline), then drive B.
        let b_at_sense_time: Arc<StdMutex<Option<NetState>>> = Arc::new(StdMutex::new(None));
        {
            let states = Arc::clone(&link.states);
            let snapshot = Arc::clone(&b_at_sense_time);
            let hb = crate::component::PinHandle::wired(NetId(1), Some(e_b), None, link.clone());
            link.send(Command::RegisterSense {
                net: NetId(0),
                callback: Box::new(move |state| {
                    if state == NetState::Driven(Level::High) {
                        let b_now = states.lock().unwrap()[1];
                        *snapshot.lock().unwrap() = Some(b_now);
                        hb.set_drive(Some(high()));
                    }
                }),
            });
        }
        // Sense on net B closes the loop: drive A again (same value — the
        // loop converges because an unchanged state is not re-delivered).
        let b_log = {
            let log: Arc<StdMutex<Vec<NetState>>> = Arc::new(StdMutex::new(Vec::new()));
            let sink = Arc::clone(&log);
            let ha = crate::component::PinHandle::wired(NetId(0), Some(e_a2), None, link.clone());
            link.send(Command::RegisterSense {
                net: NetId(1),
                callback: Box::new(move |state| {
                    sink.lock().unwrap().push(state);
                    if state == NetState::Driven(Level::High) {
                        ha.set_drive(Some(high()));
                    }
                }),
            });
            log
        };

        let ha = crate::component::PinHandle::wired(NetId(0), Some(e_a), None, link);
        ha.set_drive(Some(high()));

        assert!(
            wait_for(
                || handle.net_state(NetId(1)) == Some(NetState::Driven(Level::High)),
                Duration::from_secs(5)
            ),
            "feedback drive must land in a later iteration"
        );
        assert_eq!(
            *b_at_sense_time.lock().unwrap(),
            Some(NetState::Floating),
            "at sense time the fed-back drive must NOT have been applied inline"
        );
        // Converged: B saw exactly registration + one transition.
        std::thread::sleep(Duration::from_millis(30));
        assert_eq!(
            *b_log.lock().unwrap(),
            vec![NetState::Floating, NetState::Driven(Level::High)]
        );
        assert_eq!(
            handle.net_state(NetId(0)),
            Some(NetState::Driven(Level::High))
        );
    }

    /// An unmodeled-voltage rail (`PowerOut` registers `f64::NAN` until
    /// regulator models land) must never publish `Analog(NaN)`: NaN defeats
    /// the sense change gate (`Analog(NaN) != Analog(NaN)` under IEEE
    /// semantics), so every resolution pass would re-deliver the rail's
    /// sense — and a sense callback that drives a pin on delivery would
    /// livelock the engine.
    #[rstest]
    fn unmodeled_power_rail_publishes_stable_non_nan_state() {
        let mut resolver = Resolver::new(2, Dsu::new(2));
        // The `PowerOut` registration path (`system.rs` add_pin_descriptor).
        resolver.add_power_source(0, f64::NAN);
        resolver.add_digital_sense(0);
        let e1 = resolver.add_endpoint(1, PinRef::new("U1", "1"), None);
        let handle = EngineHandle::spawn(
            resolver,
            nets(2),
            Box::new(QuasiStaticMna),
            EventLog::disabled(),
            None,
        );
        let rail_log = sense_log(&handle, NetId(0));
        let sig_log = sense_log(&handle, NetId(1));

        // Ten drives on the UNRELATED net: ten full resolution passes.
        for seq in 0..10 {
            handle.link().send(Command::Drive {
                seq,
                endpoint: e1,
                drive: Some(if seq % 2 == 0 { high() } else { low() }),
            });
        }
        assert!(
            wait_for(
                || sig_log.lock().unwrap().len() == 11,
                Duration::from_secs(5)
            ),
            "registration + ten transitions on the driven net; got {:?}",
            sig_log.lock().unwrap()
        );
        // The rail projected a stable non-NaN state, delivered exactly once
        // (at registration): the ten passes must not re-deliver it.
        assert_eq!(
            *rail_log.lock().unwrap(),
            vec![NetState::Pulled(Level::High, 0.0)],
            "an unmodeled rail projects Pulled, never Analog(NaN)"
        );
    }

    /// A sense callback on the unmodeled (`PowerOut`-sourced) rail that
    /// drives a pin on every delivery — a level-shifter/mirror component.
    /// If the rail's state ever read as "changed" (the
    /// `Analog(NaN) != Analog(NaN)` trap), every resolution pass would
    /// re-deliver the sense, whose drive forces another pass: a livelock at
    /// 100% CPU. The loop must converge to exactly the one registration
    /// delivery.
    #[rstest]
    fn nan_rail_sense_feedback_loop_converges() {
        let mut resolver = Resolver::new(2, Dsu::new(2));
        resolver.add_power_source(0, f64::NAN);
        let e1 = resolver.add_endpoint(1, PinRef::new("U1", "1"), None);
        let handle = EngineHandle::spawn(
            resolver,
            nets(2),
            Box::new(QuasiStaticMna),
            EventLog::disabled(),
            None,
        );
        let link = handle.link();

        let deliveries = Arc::new(AtomicU64::new(0));
        {
            let deliveries = Arc::clone(&deliveries);
            let pin = crate::component::PinHandle::wired(NetId(1), Some(e1), None, link.clone());
            link.send(Command::RegisterSense {
                net: NetId(0),
                callback: Box::new(move |_| {
                    deliveries.fetch_add(1, Ordering::Relaxed);
                    pin.set_drive(Some(high()));
                }),
            });
        }
        assert!(
            wait_for(
                || handle.net_state(NetId(1)) == Some(NetState::Driven(Level::High)),
                Duration::from_secs(5)
            ),
            "the registration-delivery drive must land"
        );
        // Settle, then require convergence: the drive's resolution pass must
        // not have re-delivered the (unchanged) rail state.
        std::thread::sleep(Duration::from_millis(30));
        assert_eq!(
            deliveries.load(Ordering::Relaxed),
            1,
            "the rail sense must fire exactly once (registration)"
        );
        assert_eq!(
            handle.net_state(NetId(0)),
            Some(NetState::Pulled(Level::High, 0.0))
        );
    }

    /// A solver that stamps a sentinel state, so escalation wiring is
    /// asserted independent of the (evolving) analog solver implementation.
    struct RecordingSolver {
        calls: Arc<StdMutex<Vec<Vec<NetId>>>>,
    }

    impl ClusterSolver for RecordingSolver {
        fn solve(&self, cluster: &Cluster, _inputs: &ClusterInputs) -> ClusterSolution {
            self.calls.lock().unwrap().push(cluster.nodes.clone());
            ClusterSolution {
                node_states: cluster
                    .nodes
                    .iter()
                    .map(|&n| (n, NetState::Analog(42.0)))
                    .collect(),
            }
        }
    }

    /// A competing source within ESCALATION_IMPEDANCE_RATIO of the strongest
    /// driver escalates the whole cluster through the ClusterSolver; a weak
    /// competing path (or an agreeing one) stays on the digital fast path.
    #[rstest]
    fn competing_path_within_ratio_escalates_to_cluster_solver() {
        let calls: Arc<StdMutex<Vec<Vec<NetId>>>> = Arc::new(StdMutex::new(Vec::new()));
        let solver = RecordingSolver {
            calls: Arc::clone(&calls),
        };

        // 25 Ω driver low vs 3.3 V through 47 Ω: escalates (47 <= 250).
        let mut resolver = Resolver::new(2, Dsu::new(2));
        resolver.add_endpoint(0, PinRef::new("U1", "1"), Some(low()));
        resolver.add_edge(0, 1, 47.0);
        resolver.add_power_source(1, 3.3);
        let mut net_table = nets(2);
        let mut diags = Diagnostics::new();
        resolver.resolve(&mut net_table, &mut diags, &solver);
        assert_eq!(calls.lock().unwrap().len(), 1, "cluster must escalate");
        assert!(calls.lock().unwrap()[0].contains(&NetId(0)));
        assert!(calls.lock().unwrap()[0].contains(&NetId(1)));
        assert_eq!(net_table[0].state, NetState::Analog(42.0));
        assert_eq!(net_table[1].state, NetState::Analog(42.0));

        // Same fight through 47 kΩ: fast path (47_000 > 250).
        calls.lock().unwrap().clear();
        let mut resolver = Resolver::new(2, Dsu::new(2));
        resolver.add_endpoint(0, PinRef::new("U1", "1"), Some(low()));
        resolver.add_edge(0, 1, 47_000.0);
        resolver.add_power_source(1, 3.3);
        let mut net_table = nets(2);
        let mut diags = Diagnostics::new();
        resolver.resolve(&mut net_table, &mut diags, &solver);
        assert!(
            calls.lock().unwrap().is_empty(),
            "weak path must not escalate"
        );
        assert_eq!(net_table[0].state, NetState::Driven(Level::Low));
        assert_eq!(net_table[1].state, NetState::Analog(3.3));

        // Agreeing source through 47 Ω: no divided node, no escalation.
        let mut resolver = Resolver::new(2, Dsu::new(2));
        resolver.add_endpoint(0, PinRef::new("U1", "1"), Some(high()));
        resolver.add_edge(0, 1, 47.0);
        resolver.add_power_source(1, 3.3);
        let mut net_table = nets(2);
        let mut diags = Diagnostics::new();
        resolver.resolve(&mut net_table, &mut diags, &solver);
        assert!(
            calls.lock().unwrap().is_empty(),
            "agreeing source must not escalate"
        );
        assert_eq!(net_table[0].state, NetState::Driven(Level::High));
    }

    /// One recorded source list: `(volts, impedance)` per [`ClusterSource`], in
    /// the order the resolver handed them to the solver.
    type SourceList = Vec<(Volts, Ohms)>;

    /// Accumulated source lists, one per solver call.
    type SourceListLog = Arc<StdMutex<Vec<SourceList>>>;

    /// Records the exact [`ClusterSource`] list the resolver hands the solver
    /// — i.e. the accumulation order of `matrix[c][c] += g` / `rhs[c] += i` —
    /// then defers to the real solver so states stay meaningful.
    struct SourceOrderSolver {
        seen: SourceListLog,
    }

    impl ClusterSolver for SourceOrderSolver {
        fn solve(&self, cluster: &Cluster, inputs: &ClusterInputs) -> ClusterSolution {
            self.seen.lock().unwrap().push(
                inputs
                    .sources
                    .iter()
                    .map(|s| (s.volts, s.impedance))
                    .collect(),
            );
            QuasiStaticMna.solve(cluster, inputs)
        }
    }

    /// Build one conduction cluster of `drivers` agreeing push-pull drivers
    /// (each on its own net, distinct impedances so the recorded source list
    /// is a distinguishable permutation) plus one analog sense that forces
    /// the cluster through the solver.
    fn one_cluster_many_drivers(drivers: usize) -> (Resolver, Vec<Net>) {
        let count = drivers + 1;
        let mut resolver = Resolver::new(count, Dsu::new(count));
        for i in 0..drivers {
            resolver.add_endpoint(
                i,
                PinRef::new("U1", "1"),
                Some(TheveninDrive {
                    volts: 3.3,
                    // Distinct, non-round impedances: the recorded list is
                    // only a fingerprint of iteration order if its entries
                    // are distinguishable.
                    impedance: 25.0 + i as f64,
                }),
            );
        }
        // Chain every net into ONE conduction cluster. All drivers agree on
        // level, so this is not the contention fast path.
        for i in 0..drivers {
            resolver.add_edge(i, i + 1, 10.0);
        }
        // An analog sense in a sourced cluster always escalates to the solver
        // (`resolve`, "escalation beyond driver roots").
        resolver.add_analog_sense(drivers);
        (resolver, nets(count))
    }

    /// The per-cluster source list must be assembled by walking the **dense**
    /// drive table, so its order is endpoint order and nothing else. It used
    /// to be built by iterating `net_drivers.values()` — a `HashMap` — and
    /// that `Vec` order is the float accumulation order inside
    /// [`QuasiStaticMna::solve`] (`DETERMINISM.md`, "One real hash-order
    /// defect").
    ///
    /// This is the *direct* regression gate for the fix: it asserts the exact
    /// permutation rather than a downstream numeric consequence. With eight
    /// distinguishable sources, a `HashMap` iteration order that happened to
    /// come out ascending is a 1-in-8! (≈ 2.5e-5) fluke, so a reintroduced
    /// hash walk fails here in-process — which bit-exactness alone cannot do
    /// (see the sibling test).
    #[rstest]
    #[case::two(2)]
    #[case::three(3)]
    #[case::eight(8)]
    fn cluster_sources_follow_dense_endpoint_order(#[case] drivers: usize) {
        let seen: SourceListLog = Arc::new(StdMutex::new(Vec::new()));
        let solver = SourceOrderSolver {
            seen: Arc::clone(&seen),
        };

        let (mut resolver, mut net_table) = one_cluster_many_drivers(drivers);
        let mut diags = Diagnostics::new();
        resolver.resolve(&mut net_table, &mut diags, &solver);

        let recorded = seen.lock().unwrap().clone();
        assert_eq!(
            recorded.len(),
            1,
            "the analog sense must escalate exactly one cluster; got {recorded:?}"
        );
        let expected: SourceList = (0..drivers).map(|i| (3.3, 25.0 + i as f64)).collect();
        assert_eq!(
            recorded[0], expected,
            "cluster sources must arrive in dense endpoint order, not hash order"
        );
    }

    /// The numeric consequence of the ordering fix: a cluster whose sources
    /// **share one analog supernode** solves to a **bit-identical** node voltage
    /// across many repeated resolutions.
    ///
    /// Two things about this are worth recording, because both correct a
    /// prediction in `DETERMINISM.md`:
    ///
    /// 1. **The defect needed sources on the same supernode**, not merely two
    ///    sources in a cluster. Order only reaches a float through the
    ///    accumulations `matrix[c][c] += g` / `rhs[c] += i`
    ///    ([`QuasiStaticMna::solve`]), so it takes ≥ 2 sources on distinct
    ///    identity roots that a **0 Ω conduction edge** merges into one
    ///    supernode — which is exactly this topology. Sources on separate
    ///    supernodes each stamp their own diagonal and cannot disagree.
    /// 2. **In-process repetition does catch it.** The doc predicted it could
    ///    not, on the theory that one process shares one `HashMap` seed. That
    ///    is not how `std` behaves: `RandomState::new` bumps its thread-local
    ///    key on *every* `HashMap::new`, and `resolve` builds a fresh
    ///    `net_drivers` map per call — so the iteration order varied per pass,
    ///    and this test failed 12/12 processes with the defect in place
    ///    (observed spread: 4 distinct bit patterns, ~4 ULP around
    ///    2.894382877392857 V). Multi-process comparison stays worthwhile for
    ///    hash order held in a *long-lived* map and for cross-architecture
    ///    float differences, but it is not the only net that catches this.
    #[rstest]
    fn repeated_resolutions_of_a_multi_source_cluster_are_bit_identical() {
        // Six agreeing drivers at awkward (volts, Ω), each on its own net,
        // all merged into ONE supernode by 0 Ω edges — so every driver's
        // Norton conductance and injection accumulate into the same
        // `matrix[c][c]` / `rhs[c]`. A 0 V rail 37 Ω away escalates the
        // cluster, and the solved voltage has a full mantissa to disagree in.
        let build = || {
            let mut resolver = Resolver::new(7, Dsu::new(7));
            let drivers = [
                (3.31, 23.7),
                (3.29, 31.1),
                (3.27, 47.3),
                (3.33, 19.9),
                (3.19, 29.3),
                (3.23, 41.7),
            ];
            for (i, (volts, impedance)) in drivers.into_iter().enumerate() {
                resolver.add_endpoint(
                    i,
                    PinRef::new("U1", "1"),
                    Some(TheveninDrive { volts, impedance }),
                );
            }
            for i in 0..5 {
                resolver.add_edge(i, i + 1, 0.0);
            }
            resolver.add_edge(5, 6, 37.0);
            resolver.add_power_source(6, 0.0);
            (resolver, nets(7))
        };

        let sources_seen: SourceListLog = Arc::new(StdMutex::new(Vec::new()));
        let solver = SourceOrderSolver {
            seen: Arc::clone(&sources_seen),
        };

        let mut first: Option<Volts> = None;
        for pass in 0..64 {
            let (mut resolver, mut net_table) = build();
            let mut diags = Diagnostics::new();
            resolver.resolve(&mut net_table, &mut diags, &solver);
            let NetState::Analog(volts) = net_table[0].state else {
                panic!(
                    "pass {pass}: the cluster must solve numerically, got {:?}",
                    net_table[0].state
                );
            };
            match first {
                None => first = Some(volts),
                Some(expected) => assert!(
                    volts.total_cmp(&expected).is_eq(),
                    "pass {pass}: node voltage drifted from {expected:?} to {volts:?} \
                     — resolution is not a pure function of the drive table"
                ),
            }
        }

        // Not vacuous: every pass really did put ≥ 2 sources through the
        // solver, in dense endpoint order. Without this the test could pass on
        // a topology that never escalates, or one whose cluster carries a
        // single source (no accumulation order to get wrong).
        let recorded = sources_seen.lock().unwrap().clone();
        assert_eq!(recorded.len(), 64, "every pass must escalate exactly once");
        let expected: SourceList = vec![
            (3.31, 23.7),
            (3.29, 31.1),
            (3.27, 47.3),
            (3.33, 19.9),
            (3.19, 29.3),
            (3.23, 41.7),
            (0.0, 0.0), // the ideal rail, appended after the drivers
        ];
        for (pass, sources) in recorded.iter().enumerate() {
            assert_eq!(
                *sources, expected,
                "pass {pass}: source order must be dense endpoint order"
            );
        }
    }

    /// A net reached only through a series resistor carries the **driver's**
    /// level, not a default.
    ///
    /// The projection used to take its level from the cluster's *power* source
    /// alone, so a cluster fed by a signal driver — which has none — read
    /// `Pulled(High)` whichever way the driver was pointing. Nothing noticed
    /// while UART bytes were routed around the resolution entirely; the moment
    /// a UART's *bits* had to cross the DS2Addon's 47 ohm series resistors to
    /// reach the ADC, every frame arrived as 0xFF.
    #[rstest]
    #[case::low(low(), Level::Low)]
    #[case::high(high(), Level::High)]
    fn a_driven_level_crosses_a_series_resistor(
        #[case] drive: TheveninDrive,
        #[case] expect: Level,
    ) {
        // driver —47 ohm— far
        let mut resolver = Resolver::new(2, Dsu::new(2));
        let endpoint = resolver.add_endpoint(0, PinRef::new("U1", "1"), None);
        resolver.add_edge(0, 1, 47.0);
        resolver.set_drive(endpoint, Some(drive));

        let mut net_table = nets(2);
        let mut diags = Diagnostics::new();
        resolver.resolve(&mut net_table, &mut diags, &QuasiStaticMna);

        assert_eq!(net_table[0].state, NetState::Driven(expect));
        assert_eq!(
            net_table[1].state,
            NetState::Pulled(expect, 47.0),
            "the far side of a series resistor must follow the driver"
        );
    }

    #[rstest]
    #[case::pullup_3v3(3.3, low(), "driver Low, 10k pull-up to 3.3")]
    #[case::pulldown_gnd(0.0, high(), "driver High, 10k pull-down to GND")]
    fn zz_probe_rail_vs_driver(
        #[case] rail: f64,
        #[case] drive: TheveninDrive,
        #[case] label: &str,
    ) {
        // driver(net0) —47— mid(net1) —10k— rail(net2)
        let mut resolver = Resolver::new(3, Dsu::new(3));
        let endpoint = resolver.add_endpoint(0, PinRef::new("U1", "1"), None);
        resolver.add_edge(0, 1, 47.0);
        resolver.add_edge(1, 2, 10_000.0);
        resolver.add_power_source(2, rail);
        resolver.add_digital_sense(1);
        resolver.set_drive(endpoint, Some(drive));

        let mut net_table = nets(3);
        let mut diags = Diagnostics::new();
        resolver.resolve(&mut net_table, &mut diags, &QuasiStaticMna);
        eprintln!(
            "PROBE[{label}]: n0={:?} n1={:?} n2={:?} findings={}",
            net_table[0].state,
            net_table[1].state,
            net_table[2].state,
            diags.findings().len()
        );
    }

    /// An injected `net_stuck` fault is an **ideal** source, so it beats a
    /// driver reached through resistance — the same way a declared rail does.
    ///
    /// `cluster_sourced` absorbed stuck faults but threw their voltage away,
    /// so once the projection learned to fall back to the cluster's drivers, a
    /// 25 Ω driver behind kilohms of series resistance could out-vote a 0 Ω
    /// short. Fault injection exists to be observable; losing to a driver is
    /// the opposite.
    #[rstest]
    fn an_injected_short_outvotes_a_driver_reached_through_resistance() {
        // stuck(3.3 V) —47 Ω— mid —10 kΩ— driver(Low, 25 Ω)
        let build = |stuck: bool| {
            let mut resolver = Resolver::new(3, Dsu::new(3));
            let endpoint = resolver.add_endpoint(2, PinRef::new("U1", "1"), None);
            resolver.add_edge(0, 1, 47.0);
            resolver.add_edge(1, 2, 10_000.0);
            resolver.set_drive(endpoint, Some(low()));
            if stuck {
                resolver.add_stuck_source(0, 3.3);
            }
            let mut net_table = nets(3);
            let mut diags = Diagnostics::new();
            resolver.resolve(&mut net_table, &mut diags, &QuasiStaticMna);
            net_table[1].state
        };

        assert!(
            matches!(build(false), NetState::Pulled(Level::Low, _)),
            "with no fault the driver is the only source; got {:?}",
            build(false)
        );
        assert!(
            matches!(build(true), NetState::Pulled(Level::High, _)),
            "a 0 ohm short must win over a driver behind 10 kohm; got {:?}",
            build(true)
        );
    }

    /// A `net_stuck` fault fighting a power rail on the SAME root is two
    /// disagreeing ideal sources: the root projects Contention with a
    /// finding — never a silent first-source-wins `Analog(3.3)` (fault
    /// algebra: an injected short-to-ground must be observable).
    #[rstest]
    fn stuck_fault_fighting_a_power_rail_projects_contention() {
        let mut resolver = Resolver::new(1, Dsu::new(1));
        resolver.add_power_source(0, 3.3);
        resolver.add_stuck_source(0, 0.0);
        let mut net_table = nets(1);
        let mut diags = Diagnostics::new();
        resolver.resolve(&mut net_table, &mut diags, &QuasiStaticMna);
        assert_eq!(net_table[0].state, NetState::Contention);
        assert!(
            diags
                .findings()
                .iter()
                .any(|f| matches!(f, Finding::Contention { net, .. } if net == "N0")),
            "the short must raise a finding; got {:?}",
            diags.findings()
        );

        // Agreeing stuck + rail (the reset bodge) stays on the fast path.
        let mut resolver = Resolver::new(1, Dsu::new(1));
        resolver.add_power_source(0, 3.3);
        resolver.add_stuck_source(0, 3.3);
        let mut net_table = nets(1);
        let mut diags = Diagnostics::new();
        resolver.resolve(&mut net_table, &mut diags, &QuasiStaticMna);
        assert_eq!(net_table[0].state, NetState::Analog(3.3));
        assert!(diags.is_empty(), "agreeing ideal sources must not contend");
    }

    /// The doc's canonical analog cluster — a resistor divider between rails
    /// — has no push-pull driver anywhere, yet must reach the cluster solver
    /// and report the divided node voltage, not the `Pulled` upper-bound
    /// fallback.
    #[rstest]
    fn sourced_divider_without_a_driver_reaches_the_cluster_solver() {
        // 3.3 V —4.7 kΩ— mid —4.7 kΩ— 0 V: V_mid = 1.65 V.
        let mut resolver = Resolver::new(3, Dsu::new(3));
        resolver.add_power_source(0, 3.3);
        resolver.add_power_source(2, 0.0);
        resolver.add_edge(0, 1, 4_700.0);
        resolver.add_edge(1, 2, 4_700.0);
        let mut net_table = nets(3);
        let mut diags = Diagnostics::new();
        resolver.resolve(&mut net_table, &mut diags, &QuasiStaticMna);
        let NetState::Analog(v_mid) = net_table[1].state else {
            panic!(
                "midpoint must solve numerically, got {:?}",
                net_table[1].state
            );
        };
        assert!(
            (v_mid - 1.65).abs() < 1e-6,
            "hand check 1.65 V, got {v_mid}"
        );
        assert!(
            diags.is_empty(),
            "a divider is not a fault: {:?}",
            diags.findings()
        );
    }

    /// An analog sense (ADC input) escalates its sourced cluster and reads
    /// the solved node voltage; the same topology with a digital-only sense
    /// keeps the fast-path `Pulled` projection (escalating it would erase
    /// the meaningful pull-up view).
    #[rstest]
    fn analog_sense_escalates_sourced_cluster_but_pull_up_stays_pulled() {
        // 3.3 V rail —4.7 kΩ— AIN (no load: solves to the rail's OCV).
        let mut resolver = Resolver::new(2, Dsu::new(2));
        resolver.add_power_source(0, 3.3);
        resolver.add_edge(0, 1, 4_700.0);
        resolver.add_analog_sense(1);
        let mut net_table = nets(2);
        let mut diags = Diagnostics::new();
        resolver.resolve(&mut net_table, &mut diags, &QuasiStaticMna);
        assert!(
            matches!(net_table[1].state, NetState::Analog(v) if (v - 3.3).abs() < 1e-6),
            "analog sense must read the solved voltage; got {:?}",
            net_table[1].state
        );

        // Same topology, digital sense only: the Pulled projection stands.
        let mut resolver = Resolver::new(2, Dsu::new(2));
        resolver.add_power_source(0, 3.3);
        resolver.add_edge(0, 1, 4_700.0);
        resolver.add_digital_sense(1);
        let mut net_table = nets(2);
        let mut diags = Diagnostics::new();
        resolver.resolve(&mut net_table, &mut diags, &QuasiStaticMna);
        assert_eq!(net_table[1].state, NetState::Pulled(Level::High, 4_700.0));
    }

    /// Register a wake callback appending sampled timestamps to a log.
    fn wake_log(handle: &EngineHandle, component: ComponentId) -> Arc<StdMutex<Vec<u64>>> {
        let log: Arc<StdMutex<Vec<u64>>> = Arc::new(StdMutex::new(Vec::new()));
        let sink = Arc::clone(&log);
        handle.link().send_control(Command::RegisterWake {
            component,
            callback: Box::new(move |now_ns| sink.lock().unwrap().push(now_ns)),
        });
        log
    }

    fn empty_engine() -> EngineHandle {
        let handle = empty_engine_held();
        handle.release_time();
        handle
    }

    /// An empty engine with virtual time **held**: it drains and arms, but
    /// nothing fires until [`EngineHandle::release_time`]. A test that must
    /// get several requests in before any of them is served needs this —
    /// otherwise it races the engine, which is free to serve the first
    /// request while the rest are still in flight.
    fn empty_engine_held() -> EngineHandle {
        EngineHandle::spawn(
            Resolver::new(0, Dsu::new(0)),
            Vec::new(),
            Box::new(QuasiStaticMna),
            EventLog::disabled(),
            None,
        )
    }

    /// The wheel resolves deadlines a *bit period* apart, not a microsecond.
    ///
    /// This is why the timebase is nanoseconds: at 2 Mbaud a UART bit is 500 ns,
    /// so eight of them fit inside a single microsecond. With a µs wheel all
    /// eight of these deadlines would be the same instant, and a synthesized
    /// waveform would collapse to one edge.
    #[rstest]
    fn the_wheel_separates_sub_microsecond_deadlines() {
        let _g = lock_clock();
        virtual_clock::init(0.0, 1_000_000);
        let handle = empty_engine();
        let log = wake_log(&handle, ComponentId(0));

        const BIT_NS: u64 = 500; // one bit at 2,000,000 baud
        let start = virtual_clock::virtual_ns();
        let deadlines: Vec<u64> = (1..=8).map(|i| start + i * BIT_NS).collect();
        for &at_ns in &deadlines {
            handle.link().send_control(Command::ScheduleAt {
                component: ComponentId(0),
                at_ns,
            });
        }

        assert!(
            wait_for(|| log.lock().unwrap().len() == 8, Duration::from_secs(5)),
            "every bit deadline must fire; got {:?}",
            log.lock().unwrap()
        );
        // Stepped time advances *to* each deadline, so the stamps are exact.
        assert_eq!(*log.lock().unwrap(), deadlines);
    }

    /// Re-requesting a deadline already armed for the same component is a
    /// no-op: the wake is delivered once, not once per request. A callback
    /// that re-arms its next edge on every wake would otherwise fire once
    /// per previous wake at that instant, and compound.
    #[rstest]
    fn a_deadline_armed_repeatedly_wakes_once() {
        let _g = lock_clock();
        virtual_clock::init(0.0, 1_000_000);
        // Time held while the requests go in: a delivered wake frees its
        // instant to be armed again (that is the contract), so a test that
        // let the engine run would be asserting on a race, not on dedupe.
        let handle = empty_engine_held();
        let log = wake_log(&handle, ComponentId(0));

        let now = virtual_clock::virtual_ns();
        let first = now + 1_000_000;
        let second = now + 2_000_000;
        for _ in 0..50 {
            handle.link().send_control(Command::ScheduleAt {
                component: ComponentId(0),
                at_ns: first,
            });
        }
        handle.link().send_control(Command::ScheduleAt {
            component: ComponentId(0),
            at_ns: second,
        });
        // Every request is now queued ahead of the release, so all 50 are
        // deduped against one armed entry.
        handle.release_time();

        assert!(
            wait_for(|| log.lock().unwrap().len() >= 2, Duration::from_secs(5)),
            "both distinct deadlines must fire; got {:?}",
            log.lock().unwrap()
        );
        std::thread::sleep(Duration::from_millis(50));
        assert_eq!(*log.lock().unwrap(), vec![first, second]);
    }

    /// A one-shot armed at the instant a periodic entry is due does not eat
    /// the periodic chain: both fire there (one wake), and the period goes on.
    #[rstest]
    fn a_periodic_chain_survives_a_one_shot_at_its_instant() {
        let _g = lock_clock();
        virtual_clock::init(0.0, 1_000_000);
        // Held, so the one-shot and the periodic entry land on the same
        // instant by construction rather than by winning a race.
        let handle = empty_engine_held();
        let log = wake_log(&handle, ComponentId(0));

        let now = virtual_clock::virtual_ns();
        let period = 1_000_000u64;
        handle.link().send_control(Command::ScheduleAt {
            component: ComponentId(0),
            at_ns: now + period,
        });
        handle.link().send_control(Command::ScheduleEvery {
            component: ComponentId(0),
            period_ns: period,
        });
        handle.release_time();

        assert!(
            wait_for(|| log.lock().unwrap().len() >= 3, Duration::from_secs(5)),
            "the chain must continue past the shared instant; got {:?}",
            log.lock().unwrap()
        );
        let stamps = log.lock().unwrap().clone();
        assert_eq!(
            &stamps[..3],
            &[now + period, now + 2 * period, now + 3 * period]
        );
    }

    /// `schedule_at` fires exactly once, at-or-after its virtual deadline.
    #[rstest]
    fn one_shot_timer_fires_once_at_virtual_deadline() {
        let _g = lock_clock();
        virtual_clock::init(0.0, 1_000_000);
        let handle = empty_engine();
        let log = wake_log(&handle, ComponentId(0));

        let now = virtual_clock::virtual_ns();
        let deadline = now + 100_000_000; // 100 virtual ms
        handle.link().send_control(Command::ScheduleAt {
            component: ComponentId(0),
            at_ns: deadline,
        });

        assert!(
            wait_for(|| !log.lock().unwrap().is_empty(), Duration::from_secs(5)),
            "one-shot must fire"
        );
        assert!(
            log.lock().unwrap()[0] >= deadline,
            "sampled wake time must be at/after the deadline"
        );
        std::thread::sleep(Duration::from_millis(20));
        assert_eq!(log.lock().unwrap().len(), 1, "one-shot fires exactly once");
    }

    /// `schedule_every` keeps firing with non-decreasing sampled timestamps.
    #[rstest]
    fn periodic_timer_fires_repeatedly_against_scaled_clock() {
        let _g = lock_clock();
        virtual_clock::init(0.0, 1_000_000);
        let handle = empty_engine();
        let log = wake_log(&handle, ComponentId(0));

        handle.link().send_control(Command::ScheduleEvery {
            component: ComponentId(0),
            period_ns: 20_000_000,
        });
        assert!(
            wait_for(|| log.lock().unwrap().len() >= 3, Duration::from_secs(5)),
            "periodic must fire repeatedly; got {:?}",
            log.lock().unwrap()
        );
        let stamps = log.lock().unwrap().clone();
        assert!(
            stamps.windows(2).all(|w| w[0] <= w[1]),
            "sampled timestamps must be non-decreasing: {stamps:?}"
        );
    }

    /// Deadlines already in the past fire immediately, in deadline order.
    #[rstest]
    fn late_wakeups_fire_immediately_in_deadline_order() {
        let _g = lock_clock();
        virtual_clock::init(0.0, 1_000_000);
        let handle = empty_engine();

        let order: Arc<StdMutex<Vec<u32>>> = Arc::new(StdMutex::new(Vec::new()));
        for (component, tag) in [(ComponentId(0), 0u32), (ComponentId(1), 1u32)] {
            let sink = Arc::clone(&order);
            handle.link().send_control(Command::RegisterWake {
                component,
                callback: Box::new(move |_| sink.lock().unwrap().push(tag)),
            });
        }
        // Schedule the LATER deadline first; firing order must follow the
        // deadlines, not the schedule order.
        handle.link().send_control(Command::ScheduleAt {
            component: ComponentId(1),
            at_ns: 2_000,
        });
        handle.link().send_control(Command::ScheduleAt {
            component: ComponentId(0),
            at_ns: 1_000,
        });

        assert!(
            wait_for(|| order.lock().unwrap().len() == 2, Duration::from_secs(5)),
            "both late one-shots must fire"
        );
        assert_eq!(*order.lock().unwrap(), vec![0, 1]);
    }

    /// Dropping the handle with pending far-future timers joins promptly —
    /// no detached thread, no deadlock against the parked wheel.
    #[rstest]
    fn shutdown_joins_cleanly_with_pending_timers() {
        let _g = lock_clock();
        virtual_clock::init(0.0, 1_000_000);
        let handle = empty_engine();
        let _log = wake_log(&handle, ComponentId(0));
        handle.link().send_control(Command::ScheduleAt {
            component: ComponentId(0),
            at_ns: virtual_clock::virtual_ns() + 60_000_000_000, // one virtual minute out
        });
        std::thread::sleep(Duration::from_millis(10)); // let the engine park on the deadline

        let start = Instant::now();
        drop(handle); // sends Shutdown + joins
        assert!(
            start.elapsed() < Duration::from_secs(5),
            "drop must not wait for the pending timer"
        );
    }

    /// Virtual time must not run past a wake that has been *requested* but not
    /// yet handled.
    ///
    /// The command drain is capped ([`COMMAND_DRAIN_BATCH_MAX`]) so a drive
    /// flood cannot starve the wheel — but a `ScheduleAt` still sitting in that
    /// queue is a deadline the wheel cannot see, and advancing past it delivers
    /// the wake late. For a component whose events are milliseconds apart that
    /// is invisible. For one clocking a UART bit every 8.68 µs it is fatal: the
    /// remaining bits are all overdue at the new instant, so the rest of the
    /// byte goes onto the wire at a single point in time and arrives as
    /// garbage. That is exactly how the whole-machine force path failed.
    ///
    /// Here a component re-arms itself one bit period ahead from inside its own
    /// wake handler while a flood keeps the command queue non-empty. The
    /// delivered timestamps must stay on the 500 ns grid.
    #[rstest]
    fn time_does_not_run_past_a_schedule_still_in_flight() {
        let _g = lock_clock();
        virtual_clock::init(0.0, 1_000_000);
        let mut resolver = Resolver::new(1, Dsu::new(1));
        let e0 = resolver.add_endpoint(0, PinRef::new("U1", "1"), None);
        let handle = EngineHandle::spawn(
            resolver,
            nets(1),
            Box::new(QuasiStaticMna),
            EventLog::disabled(),
            None,
        );
        handle.release_time();

        const BIT_NS: u64 = 500; // 2,000,000 baud
        const BITS: usize = 20;
        let log: Arc<StdMutex<Vec<u64>>> = Arc::new(StdMutex::new(Vec::new()));
        {
            let sink = Arc::clone(&log);
            let link = handle.link();
            handle.link().send_control(Command::RegisterWake {
                component: ComponentId(0),
                callback: Box::new(move |now_ns| {
                    let mut seen = sink.lock().unwrap();
                    if seen.len() >= BITS {
                        return;
                    }
                    seen.push(now_ns);
                    let next = now_ns + BIT_NS;
                    drop(seen);
                    link.send_control(Command::ScheduleAt {
                        component: ComponentId(0),
                        at_ns: next,
                    });
                }),
            });
        }

        // A flood that keeps the drain loop hitting its cap the whole time.
        let stop = Arc::new(AtomicBool::new(false));
        let flood = {
            let stop = Arc::clone(&stop);
            let pin = crate::component::PinHandle::wired(NetId(0), Some(e0), None, handle.link());
            std::thread::spawn(move || {
                let mut level = false;
                while !stop.load(Ordering::Relaxed) {
                    level = !level;
                    pin.set_drive(Some(if level { high() } else { low() }));
                }
            })
        };

        handle.link().send_control(Command::ScheduleAt {
            component: ComponentId(0),
            at_ns: virtual_clock::virtual_ns() + BIT_NS,
        });
        let done = wait_for(
            || log.lock().unwrap().len() >= BITS,
            Duration::from_secs(10),
        );
        stop.store(true, Ordering::Relaxed);
        flood.join().unwrap();
        assert!(done, "the bit clock must keep ticking under a drive flood");

        let stamps = log.lock().unwrap().clone();
        let gaps: Vec<u64> = stamps.windows(2).map(|w| w[1] - w[0]).collect();
        assert!(
            gaps.iter().all(|&gap| gap == BIT_NS),
            "every wake must land one bit period after the last; got gaps {gaps:?}"
        );
    }

    /// The timer wheel must not starve under sustained command load: a busy
    /// protocol thread enqueuing drives in a tight loop keeps the channel
    /// non-empty, yet a due wake must still fire — drain is capped at
    /// [`COMMAND_DRAIN_BATCH_MAX`] so time can jump.
    #[rstest]
    fn sustained_drive_flood_does_not_starve_the_timer_wheel() {
        let _g = lock_clock();
        virtual_clock::init(0.0, 1_000_000);
        let mut resolver = Resolver::new(1, Dsu::new(1));
        let e0 = resolver.add_endpoint(0, PinRef::new("U1", "1"), None);
        let handle = EngineHandle::spawn(
            resolver,
            nets(1),
            Box::new(QuasiStaticMna),
            EventLog::disabled(),
            None,
        );
        handle.release_time();
        let log = wake_log(&handle, ComponentId(0));

        let stop = Arc::new(AtomicBool::new(false));
        let flood = {
            let stop = Arc::clone(&stop);
            let pin = crate::component::PinHandle::wired(NetId(0), Some(e0), None, handle.link());
            std::thread::spawn(move || {
                let mut level = false;
                while !stop.load(Ordering::Relaxed) {
                    level = !level;
                    pin.set_drive(Some(if level { high() } else { low() }));
                }
            })
        };

        handle.link().send_control(Command::ScheduleAt {
            component: ComponentId(0),
            at_ns: virtual_clock::virtual_ns() + 10_000_000, // 10 virtual ms
        });
        let fired = wait_for(|| !log.lock().unwrap().is_empty(), Duration::from_secs(5));
        stop.store(true, Ordering::Relaxed);
        flood.join().unwrap();
        assert!(
            fired,
            "a due wake must fire while the drive flood is sustained"
        );
    }

    /// A reserved-but-never-sent drive seq (the enqueuing thread died
    /// between `next_drive_seq` and the channel send) must not wedge every
    /// later drive forever: after the idle-poll skip the engine reports
    /// [`Finding::DriveSeqGap`] naming the missing seq and applies the
    /// buffered drives.
    #[rstest]
    fn missing_drive_seq_is_skipped_after_a_bounded_wait() {
        let mut resolver = Resolver::new(1, Dsu::new(1));
        let e0 = resolver.add_endpoint(0, PinRef::new("U1", "1"), None);
        let handle = EngineHandle::spawn(
            resolver,
            nets(1),
            Box::new(QuasiStaticMna),
            EventLog::disabled(),
            None,
        );
        let link = handle.link();

        // Reserve seq 0 and "die" before sending it; seq 1 arrives normally.
        assert_eq!(link.next_drive_seq(), 0);
        link.send(Command::Drive {
            seq: link.next_drive_seq(),
            endpoint: e0,
            drive: Some(high()),
        });

        // Ordering holds while the watchdog waits on the gap...
        std::thread::sleep(Duration::from_millis(50));
        assert_eq!(handle.net_state(NetId(0)), Some(NetState::Floating));
        // ...then the gap is skipped, loudly, and the buffered drive lands.
        assert!(
            wait_for(
                || handle.net_state(NetId(0)) == Some(NetState::Driven(Level::High)),
                Duration::from_secs(5)
            ),
            "the buffered drive must apply once the gap is skipped"
        );
        assert!(
            handle.findings().contains(&Finding::DriveSeqGap { seq: 0 }),
            "the skip must name the missing seq; got {:?}",
            handle.findings()
        );
    }

    /// A panicking sense callback is contained: the finding names the net,
    /// the engine stays alive ([`EngineHandle::is_alive`]), and other
    /// subscribers keep being served — one misbehaving component must not
    /// end net service for the rest of the system.
    #[rstest]
    fn sense_callback_panic_is_contained_and_reported() {
        let mut resolver = Resolver::new(1, Dsu::new(1));
        let e0 = resolver.add_endpoint(0, PinRef::new("U1", "1"), None);
        let handle = EngineHandle::spawn(
            resolver,
            nets(1),
            Box::new(QuasiStaticMna),
            EventLog::disabled(),
            None,
        );
        let link = handle.link();

        link.send(Command::RegisterSense {
            net: NetId(0),
            callback: Box::new(|_| panic!("component bug")),
        });
        let log = sense_log(&handle, NetId(0)); // well-behaved subscriber
        link.send(Command::Drive {
            seq: 0,
            endpoint: e0,
            drive: Some(high()),
        });

        assert!(
            wait_for(|| log.lock().unwrap().len() == 2, Duration::from_secs(5)),
            "the well-behaved subscriber must keep receiving; got {:?}",
            log.lock().unwrap()
        );
        assert!(
            handle.is_alive(),
            "a contained callback panic must not kill the engine"
        );
        assert!(
            handle.findings().contains(&Finding::CallbackPanic {
                kind: CallbackKind::Sense,
                subscriber: "N0".to_string(),
            }),
            "the panic must surface as a finding; got {:?}",
            handle.findings()
        );
    }

    /// Inert handles (build-time analysis path) are safe no-ops.
    #[rstest]
    fn inert_handles_are_safe_noops() {
        let handle = crate::component::PinHandle::new(NetId(0));
        handle.set_drive(Some(high())); // dropped with a trace, no panic
        assert_eq!(handle.sense(), NetState::Floating);

        let io = crate::component::ComponentNetIo::default();
        io.schedule_at(0);
        io.schedule_every(1_000);
        io.on_wake(|_| {});
    }

    /// Disagreeing push-pull drivers coupled through series resistance below
    /// STREAM_COLLAPSE_THRESHOLD resolve to Contention (net rules: for
    /// signaling purposes the collapsed link is one node); the same fight
    /// through resistance at/above the threshold does not contend.
    #[rstest]
    fn disagreeing_drivers_through_collapsed_resistance_resolve_contention() {
        // 25 Ω high vs 25 Ω low through 47 Ω: Contention on both roots.
        let mut resolver = Resolver::new(2, Dsu::new(2));
        resolver.add_endpoint(0, PinRef::new("U1", "1"), Some(high()));
        resolver.add_endpoint(1, PinRef::new("U2", "1"), Some(low()));
        resolver.add_edge(0, 1, 47.0);
        let mut net_table = nets(2);
        let mut diags = Diagnostics::new();
        resolver.resolve(&mut net_table, &mut diags, &QuasiStaticMna);
        assert_eq!(net_table[0].state, NetState::Contention);
        assert_eq!(net_table[1].state, NetState::Contention);
        assert!(
            diags.findings().iter().any(|f| matches!(
                f,
                Finding::Contention { drivers, .. }
                    if drivers.contains(&PinRef::new("U1", "1"))
                        && drivers.contains(&PinRef::new("U2", "1"))
            )),
            "the finding must name both fighting drivers; got {:?}",
            diags.findings()
        );

        // Same fight through the threshold value itself: no contention (the
        // bound is strict); neither net projects Contention.
        let mut resolver = Resolver::new(2, Dsu::new(2));
        resolver.add_endpoint(0, PinRef::new("U1", "1"), Some(high()));
        resolver.add_endpoint(1, PinRef::new("U2", "1"), Some(low()));
        resolver.add_edge(0, 1, STREAM_COLLAPSE_THRESHOLD);
        let mut net_table = nets(2);
        let mut diags = Diagnostics::new();
        resolver.resolve(&mut net_table, &mut diags, &QuasiStaticMna);
        assert_ne!(net_table[0].state, NetState::Contention);
        assert_ne!(net_table[1].state, NetState::Contention);
    }

    /// Pulse routes collapse series passives below the threshold: a step clock
    /// through 47 Ω resistors reaches the drive, one behind a 4.7 kΩ isolation
    /// resistor does not.
    #[rstest]
    fn pulse_routes_collapse_series_passives_and_ignore_byte_roles() {
        // source(0) --47Ω-- (1) --47Ω-- sink(2), a sink behind 4.7 kΩ that
        // must NOT route, and a UART consumer that is not a pulse sink at all.
        let mut resolver = Resolver::new(5, Dsu::new(5));
        let source = resolver.add_endpoint(0, PinRef::new("MCU", "P8"), Some(high()));
        let near = resolver.add_endpoint(2, PinRef::new("DRV", "STEP"), None);
        let far = resolver.add_endpoint(3, PinRef::new("FAR", "STEP"), None);
        resolver.add_edge(0, 1, 47.0);
        resolver.add_edge(1, 2, 47.0);
        resolver.add_edge(0, 3, 4_700.0);
        resolver.add_stream_pin(source, 0, StreamRole::PulseSource, PinRef::new("MCU", "P8"));
        resolver.add_stream_pin(near, 2, StreamRole::PulseSink, PinRef::new("DRV", "STEP"));
        resolver.add_stream_pin(far, 3, StreamRole::PulseSink, PinRef::new("FAR", "STEP"));

        let net_table = nets(5);
        let mut diags = Diagnostics::new();
        let routes = resolver.route_pulses(&net_table, &mut diags);
        assert!(diags.is_empty(), "no mismatch: {:?}", diags.findings());
        assert_eq!(routes.len(), 1);
        assert_eq!(routes[0].source, source);
        assert_eq!(
            routes[0].sinks,
            vec![near],
            "only the sink inside the collapse radius routes"
        );
    }

    /// Two step clocks driving one line is the same wiring error as two UART
    /// transmitters: reported once per pair, and neither routes.
    #[rstest]
    fn facing_pulse_sources_raise_stream_mismatch_and_do_not_route() {
        let mut resolver = Resolver::new(3, Dsu::new(3));
        let a = resolver.add_endpoint(0, PinRef::new("MCU", "P8"), Some(high()));
        let b = resolver.add_endpoint(1, PinRef::new("ALT", "P9"), Some(high()));
        let sink = resolver.add_endpoint(2, PinRef::new("DRV", "STEP"), None);
        resolver.add_edge(0, 1, 47.0);
        resolver.add_edge(1, 2, 47.0);
        resolver.add_stream_pin(a, 0, StreamRole::PulseSource, PinRef::new("MCU", "P8"));
        resolver.add_stream_pin(b, 1, StreamRole::PulseSource, PinRef::new("ALT", "P9"));
        resolver.add_stream_pin(sink, 2, StreamRole::PulseSink, PinRef::new("DRV", "STEP"));

        let net_table = nets(3);
        let mut diags = Diagnostics::new();
        let routes = resolver.route_pulses(&net_table, &mut diags);
        assert!(routes.is_empty(), "facing pulse sources must not route");
        assert_eq!(
            diags.len(),
            1,
            "one finding per pair: {:?}",
            diags.findings()
        );
        assert!(diags.findings().iter().any(|f| matches!(
            f,
            Finding::StreamMismatch { producers, .. }
                if producers.contains(&PinRef::new("MCU", "P8"))
                    && producers.contains(&PinRef::new("ALT", "P9"))
        )));
    }

    /// A source with nowhere to send still routes (with no sinks) rather than
    /// vanishing, so its train is retained and a sink attaching later can be
    /// served — and a lone sink is inert, not an error.
    #[rstest]
    fn a_pulse_source_with_no_sink_routes_to_nobody() {
        let mut resolver = Resolver::new(2, Dsu::new(2));
        let source = resolver.add_endpoint(0, PinRef::new("MCU", "P8"), Some(high()));
        let orphan = resolver.add_endpoint(1, PinRef::new("DRV", "STEP"), None);
        resolver.add_stream_pin(source, 0, StreamRole::PulseSource, PinRef::new("MCU", "P8"));
        resolver.add_stream_pin(orphan, 1, StreamRole::PulseSink, PinRef::new("DRV", "STEP"));

        let net_table = nets(2);
        let mut diags = Diagnostics::new();
        let routes = resolver.route_pulses(&net_table, &mut diags);
        assert_eq!(routes.len(), 1, "the source still has a route");
        assert!(
            routes[0].sinks.is_empty(),
            "an unconnected sink is not reachable"
        );
        assert!(
            diags.is_empty(),
            "and nothing is wrong: {:?}",
            diags.findings()
        );
    }

    // ============================================================
    // Incremental resolve oracle
    // ============================================================
    //
    // The live path resolves only the cluster a drive touched. The claim
    // that this equals a full pass rests on clusters being electrically
    // independent; this pins it on random boards — merges, resistive edges,
    // drivers at every impedance, rails, stuck faults, senses — under random
    // drive sequences, comparing every net's state and the cumulative
    // findings against a resolver that recomputes the whole board each time.
    mod incremental_oracle {
        use super::*;
        use proptest::prelude::*;

        #[derive(Debug, Clone)]
        struct Spec {
            n: usize,
            merges: Vec<(usize, usize)>,
            edges: Vec<(usize, usize, f64)>,
            endpoints: Vec<(usize, Option<TheveninDrive>)>,
            power: Vec<(usize, Volts)>,
            stuck: Vec<(usize, Volts)>,
            digital_senses: Vec<usize>,
            analog_senses: Vec<usize>,
            power_senses: Vec<usize>,
        }

        fn drive_strategy() -> impl Strategy<Value = Option<TheveninDrive>> {
            prop_oneof![
                Just(None),
                (
                    prop_oneof![Just(0.0f64), Just(3.3), Just(5.0)],
                    prop_oneof![Just(25.0f64), Just(470.0), Just(15_000.0), Just(100_000.0)],
                )
                    .prop_map(|(volts, impedance)| Some(TheveninDrive { volts, impedance })),
            ]
        }

        fn spec_strategy() -> impl Strategy<Value = Spec> {
            (2usize..10).prop_flat_map(|n| {
                (
                    prop::collection::vec((0..n, 0..n), 0..3),
                    prop::collection::vec(
                        (
                            0..n,
                            0..n,
                            prop_oneof![
                                Just(0.0f64),
                                Just(47.0),
                                Just(470.0),
                                Just(4_700.0),
                                Just(15_000.0)
                            ],
                        ),
                        0..4,
                    ),
                    prop::collection::vec((0..n, drive_strategy()), 1..8),
                    prop::collection::vec(
                        (0..n, prop_oneof![Just(0.0f64), Just(3.3), Just(f64::NAN)]),
                        0..3,
                    ),
                    prop::collection::vec((0..n, prop_oneof![Just(0.0f64), Just(3.3)]), 0..2),
                    prop::collection::vec(0..n, 0..3),
                    prop::collection::vec(0..n, 0..3),
                    prop::collection::vec(0..n, 0..2),
                )
                    .prop_map(
                        move |(
                            merges,
                            edges,
                            endpoints,
                            power,
                            stuck,
                            digital_senses,
                            analog_senses,
                            power_senses,
                        )| Spec {
                            n,
                            merges,
                            edges,
                            endpoints,
                            power,
                            stuck,
                            digital_senses,
                            analog_senses,
                            power_senses,
                        },
                    )
            })
        }

        fn build(spec: &Spec) -> (Resolver, Vec<EndpointId>) {
            let mut identity = Dsu::new(spec.n);
            for &(a, b) in &spec.merges {
                identity.union(a, b);
            }
            let mut resolver = Resolver::new(spec.n, identity);
            for &(a, b, ohms) in &spec.edges {
                resolver.add_edge(a, b, ohms);
            }
            let ids: Vec<EndpointId> = spec
                .endpoints
                .iter()
                .enumerate()
                .map(|(i, (net, drive))| {
                    resolver.add_endpoint(*net, PinRef::new("U", format!("{i}")), *drive)
                })
                .collect();
            for &(net, volts) in &spec.power {
                resolver.add_power_source(net, volts);
            }
            for &(net, volts) in &spec.stuck {
                resolver.add_stuck_source(net, volts);
            }
            for &net in &spec.digital_senses {
                resolver.add_digital_sense(net);
            }
            for &net in &spec.analog_senses {
                resolver.add_analog_sense(net);
            }
            for &net in &spec.power_senses {
                resolver.add_power_sense(net);
            }
            (resolver, ids)
        }

        fn bus_of(bus: &Diagnostics) -> Vec<String> {
            let mut found: Vec<String> = bus.findings().iter().map(|f| format!("{f:?}")).collect();
            found.sort();
            found
        }

        fn merge_into(bus: &mut Diagnostics, pass: &Diagnostics) {
            for finding in pass.findings() {
                bus.report(finding.clone());
            }
        }

        proptest! {
            #![proptest_config(ProptestConfig { cases: 2000, ..ProptestConfig::default() })]
            #[test]
            fn resolving_only_the_touched_cluster_matches_a_full_pass(
                spec in spec_strategy(),
                ops in prop::collection::vec((0usize..8, drive_strategy()), 1..12),
            ) {
                let (mut incremental, ids) = build(&spec);
                let (mut full, _) = build(&spec); // the OLD algorithm, one global pass
                let mut nets_inc = nets(spec.n);
                let mut nets_full = nets(spec.n);
                let mut bus_inc = Diagnostics::new();
                let mut bus_full = Diagnostics::new();

                let mut pass = Diagnostics::new();
                incremental.resolve(&mut nets_inc, &mut pass, &QuasiStaticMna);
                merge_into(&mut bus_inc, &pass);
                let mut pass = Diagnostics::new();
                full.resolve_reference(&mut nets_full, &mut pass, &QuasiStaticMna);
                merge_into(&mut bus_full, &pass);
                for i in 0..spec.n {
                    prop_assert!(
                        same_state(&nets_inc[i].state, &nets_full[i].state),
                        "initial pass, net {i}: new {:?}, reference {:?}",
                        nets_inc[i].state,
                        nets_full[i].state
                    );
                }

                for (step, (slot, drive)) in ops.into_iter().enumerate() {
                    let endpoint = ids[slot % ids.len()];
                    let changed = incremental.set_drive(endpoint, drive);
                    let _ = full.set_drive(endpoint, drive);

                    let mut pass = Diagnostics::new();
                    incremental.resolve_dirty(&mut nets_inc, &mut pass, &QuasiStaticMna);
                    merge_into(&mut bus_inc, &pass);
                    let mut pass = Diagnostics::new();
                    full.resolve_reference(&mut nets_full, &mut pass, &QuasiStaticMna);
                    merge_into(&mut bus_full, &pass);

                    for i in 0..spec.n {
                        prop_assert!(
                            same_state(&nets_inc[i].state, &nets_full[i].state),
                            "step {step} (changed={changed}) net {i}: incremental {:?}, reference {:?}",
                            nets_inc[i].state,
                            nets_full[i].state
                        );
                    }
                    prop_assert_eq!(bus_of(&bus_inc), bus_of(&bus_full), "findings after step {}", step);
                }
            }
        }
    }
}
