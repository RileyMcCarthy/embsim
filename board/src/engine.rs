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
//! semantics. Projection is part of that shared path, and it has one form
//! (`NODES.md` "Three rules the taxonomy rests on", rule 2): every source
//! reaching a node is ranked by its **total ohms** — its own impedance plus
//! the series path to the node — a source at or above
//! [`crate::net::WEAK_DRIVE_OHMS`] is a pull that never contends, a source
//! [`crate::net::ESCALATION_IMPEDANCE_RATIO`] times weaker than the strongest
//! loses to it with a [`Finding::Contention`], and disagreeing sources closer
//! than that send the conduction cluster through the [`ClusterSolver`]
//! ([`crate::cluster::QuasiStaticMna`] by default) for the divided voltage,
//! projected through the [`crate::net::V_IL`]/[`crate::net::V_IH`] dead band.
//! See `project_root` in this module.
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
use std::sync::{Arc, Mutex, Weak};
use std::thread::JoinHandle;
use std::time::Duration;

use embsim_core::virtual_clock;

use crate::cluster::{
    Cluster, ClusterElement, ClusterInjection, ClusterInputs, ClusterResistor, ClusterSolution,
    ClusterSolver, ClusterSource, ClusterTerminal, IDEAL_SOURCE_FLOOR_OHMS,
};
use crate::component::{Drive, PinHandle, PulseTrain, PwlCurve, RegionTest, StreamRole};
use crate::diagnostics::{CallbackKind, Diagnostics, Finding, SenseKind};
use crate::event_log::{EngineEvent, EventLog};
use crate::net::{
    Amps, Level, Net, NetId, NetState, Ohms, PinRef, TheveninDrive, Volts,
    COUPLING_REACTANCE_RATIO, ESCALATION_IMPEDANCE_RATIO, STREAM_COLLAPSE_THRESHOLD, V_IH, V_IL,
    WEAK_DRIVE_OHMS,
};

// ============================================================
// Constants
// ============================================================

/// Digital projection threshold for a *source's* open-circuit voltage: at or
/// above this a source is a [`Level::High`] source, below it a
/// [`Level::Low`] one — the level a `Driven`/`Pulled` projection carries.
/// A *solved* node voltage is projected through the [`V_IL`]/[`V_IH`] dead
/// band instead ([`project_root`]).
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

/// Pin-current delivery callback ([`crate::ComponentNetIo::on_branch`]):
/// called from the engine thread with no lock held, once at registration
/// and then whenever a solve changes the current
/// ([`PinHandle::sense_current`]).
pub(crate) type CurrentCallback = Box<dyn Fn(Option<Amps>) + Send>;

/// One message on the engine's MPSC command queue.
pub(crate) enum Command {
    /// A pin drive (`None` releases to high-Z), stamped with its enqueue
    /// sequence number — the authoritative event order.
    Drive {
        /// Global enqueue sequence reserved at `set_drive` time.
        seq: u64,
        /// Target endpoint.
        endpoint: EndpointId,
        /// New contribution — Thevenin or current injection — or release.
        drive: Option<Drive>,
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
    /// A subscription declares `net` **read**: as a digital sense, by a
    /// released bidirectional pad (`DigitalBidir` declared
    /// `IdleDrive::Released`, an input until its owner drives it) ahead of
    /// its [`Command::RegisterSense`]; or as a current instrument
    /// ([`Command::RegisterCurrent`]), whose cluster must solve to have a
    /// current at all. A sense joins the senses of its kind — a floating
    /// one is a [`Finding::FloatingSense`]; an instrument joins the
    /// instruments, which escalate their cluster and report nothing
    /// ([`ReadKind`]). Either way the engine re-resolves once per drain
    /// batch after handling any.
    DeclareRead {
        /// The net the pin reads.
        net: NetId,
        /// What the net is read as.
        kind: ReadKind,
    },
    /// Subscribe a current callback to a pin ([`crate::ComponentNetIo::on_branch`]).
    /// The current is delivered once at registration, then whenever a solve
    /// changes it.
    RegisterCurrent {
        /// The pin, carrying the endpoint and branch terms the current is
        /// summed from.
        handle: PinHandle,
        /// Delivery callback.
        callback: CurrentCallback,
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
    /// sinks on the source's derived route (gated by net resolution), and
    /// retained so a sink registering later sees the channel's current state.
    /// Carries no enqueue sequence: per-source order *is* this channel's
    /// order, and cross-source ordering is not meaningful. (The deleted
    /// byte-route `StreamWrite` command used the same rule.)
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
pub(crate) type IdleDriveLog = Arc<Mutex<Vec<(EndpointId, Option<Drive>)>>>;

/// Sense subscriptions made on the inert build-time path, in registration
/// order, so `System::build`'s fixed point can deliver the states its
/// replayed attach drives change (the live engine would).
///
/// The build owns the one strong reference; every inert [`EngineLink`]
/// holds a [`Weak`] to the same log ([`EngineLink::recorded_senses`]). A
/// recorded callback captures the `PinHandle`s its component gave it, and
/// each of those carries an `EngineLink` — a strong link here would make
/// log → callback → handle → link → log a cycle that outlives the build and
/// pins every model's captured state (the flash model's image among it)
/// for the life of the process.
#[derive(Clone, Default)]
pub(crate) struct SenseLog(pub(crate) Arc<Mutex<Vec<RecordedSense>>>);

/// What a [`Command::DeclareRead`] declares a net read as.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ReadKind {
    /// A digital sense (a pad's level): a floating one is reported. (An
    /// analog sense is declared by its pin's kind at build, never live.)
    Digital,
    /// A current instrument ([`crate::ComponentNetIo::on_branch`]):
    /// escalates its cluster to a solve — only a solved cluster has a
    /// current — and nothing else. No floating-sense finding (an open loop
    /// is not a floating input), and rule 2's fights are still reported.
    Instrument,
}

/// One sense subscription the inert build link recorded: the net, whether
/// the subscribing pin is a released bidirectional pad that now reads it
/// (the build declares such a net a digital sense, as the live engine's
/// [`Command::DeclareRead`] does), and the callback — a state callback, or
/// a current instrument's, which declares the net an instrument
/// ([`ReadKind::Instrument`]).
pub(crate) struct RecordedSense {
    pub(crate) net: NetId,
    pub(crate) reads: bool,
    pub(crate) callback: RecordedCallback,
}

/// What a recorded subscription delivers.
pub(crate) enum RecordedCallback {
    /// A net-state sense ([`crate::ComponentNetIo::on_sense`]).
    State(SenseCallback),
    /// A pin-current instrument ([`crate::ComponentNetIo::on_branch`]): the
    /// handle it reads through, the callback, and the reading it was last
    /// delivered, so the build's fixed point delivers changes only.
    Current {
        handle: PinHandle,
        callback: CurrentCallback,
        last: Option<Amps>,
    },
}

/// The currents the last solves produced, per drive endpoint and per
/// element, published beside the net states: `None` where no solve has one
/// (a cluster resolved by projection alone). Indexed by [`EndpointId`] and
/// element index; read by [`PinHandle::sense_current`].
#[derive(Debug, Clone, Default, PartialEq)]
pub(crate) struct CurrentTable {
    /// Current **into** each drive endpoint from its net.
    pub(crate) endpoints: Vec<Option<Amps>>,
    /// Current through each element, from its `a` to its `b`.
    pub(crate) elements: Vec<Option<Amps>>,
}

/// The inert link's view of a [`SenseLog`]: alive while the build runs,
/// dead — and a recording silently skipped — once the build has dropped it.
pub(crate) type WeakSenseLog = Weak<Mutex<Vec<RecordedSense>>>;

impl std::fmt::Debug for SenseLog {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let count = self.0.lock().map(|log| log.len()).unwrap_or(0);
        f.debug_tuple("SenseLog").field(&count).finish()
    }
}

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
    /// Engine-published currents ([`CurrentTable`]; build snapshot when
    /// inert).
    pub(crate) currents: Arc<Mutex<CurrentTable>>,
    /// Inert path only: drives issued during attach, in issue order, for the
    /// build pass to apply before it resolves for real.
    pub(crate) recorded_drives: Option<IdleDriveLog>,
    /// Inert path only: sense subscriptions, in registration order, for the
    /// build pass's fixed point to deliver changed states to. A weak
    /// reference on purpose — see [`SenseLog`].
    pub(crate) recorded_senses: Option<WeakSenseLog>,
}

impl EngineLink {
    /// Inert link over a fixed state snapshot (the build-time analysis
    /// path), recording attach-time drives into `recorded_drives` and sense
    /// subscriptions into `recorded_senses`, which the caller keeps alive
    /// for as long as it wants recordings.
    pub(crate) fn inert(
        states: Arc<Mutex<Vec<NetState>>>,
        currents: Arc<Mutex<CurrentTable>>,
        recorded_drives: IdleDriveLog,
        recorded_senses: &SenseLog,
    ) -> Self {
        Self {
            tx: None,
            control_tx: None,
            drive_seq: Arc::new(AtomicU64::new(0)),
            pending_schedules: Arc::new(AtomicUsize::new(0)),
            states,
            currents,
            recorded_drives: Some(recorded_drives),
            recorded_senses: Some(Arc::downgrade(&recorded_senses.0)),
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
/// contributes (`None` = released / high-Z / pure sense). Always holds the
/// normalised form ([`normalise_drive`]): a Thevenin drive here has a finite
/// impedance, a current injection a finite value.
struct DriveSlot {
    net: usize,
    pin: PinRef,
    drive: Option<Drive>,
}

/// One piecewise-linear element: a [`crate::Branch`] with its pins resolved
/// to nets, and the part it belongs to, for the finding that names it.
struct Element {
    a: usize,
    b: usize,
    curve: PwlCurve,
    control: Option<(usize, RegionTest)>,
    /// `Board.Reference` of the part declaring the branch.
    reference: String,
}

impl Element {
    /// The nets the element touches, in `a`, `b`, control order.
    fn nets(&self) -> impl Iterator<Item = usize> {
        [Some(self.a), Some(self.b), self.control.map(|(net, _)| net)]
            .into_iter()
            .flatten()
    }
}

/// Where an element lives, given which identity roots are declared
/// terminals: the cluster of its **first non-terminal** net (`a`, `b`,
/// control order), and the terminal roots it names that are not that
/// cluster's own — its *foreign* constants, the conducting ends apart
/// from the control. A terminal on a conducting end sources the cluster
/// (a rail behind a diode is a rail); a terminal on the control alone
/// is read by the region test and sources nothing (a gate is a sense).
/// An element whose every net is a terminal has no home: it sits between
/// constants, changes no voltage, and is in no solve.
struct ElementHome {
    /// The identity root the element's cluster is found by.
    root: usize,
    /// The terminal roots among the element's two ends, in `a`, `b` order.
    terminal_ends: Vec<usize>,
    /// The control's root, when it is a terminal.
    terminal_control: Option<usize>,
}

fn element_home(
    element: &Element,
    root_of: &[usize],
    is_terminal: impl Fn(usize) -> bool,
) -> Option<ElementHome> {
    let root = element
        .nets()
        .map(|net| root_of[net])
        .find(|&r| !is_terminal(r))?;
    let terminal_ends = [element.a, element.b]
        .into_iter()
        .map(|net| root_of[net])
        .filter(|&r| is_terminal(r))
        .collect();
    let terminal_control = element
        .control
        .map(|(net, _)| root_of[net])
        .filter(|&r| is_terminal(r));
    Some(ElementHome {
        root,
        terminal_ends,
        terminal_control,
    })
}

/// The currents one pass computed, per endpoint and per element in the
/// clusters it touched — `None` for a cluster resolved by projection alone.
/// Applied to the resolver's tables after the pass ([`Resolver::apply_currents`]).
#[derive(Default)]
struct PassCurrents {
    endpoints: Vec<(usize, Option<Amps>)>,
    elements: Vec<(usize, Option<Amps>)>,
}

/// One serial-capable pin registered for stream routing.
struct StreamPin {
    endpoint: EndpointId,
    net: usize,
    role: StreamRole,
    pin: PinRef,
}

/// One coupling capacitor between two nets — an **AC** path a rate crosses
/// and a DC open (`NODES.md` §2, the Crystal / oscillator row: "rate routing
/// crosses a capacitor at 0 Ω; DC resolution keeps it open"). Never a
/// conduction edge.
struct CouplingCapacitor {
    a: usize,
    b: usize,
    farads: f64,
    /// The capacitor's reference designator, for the finding that names it.
    reference: String,
}

/// One coupling capacitor a routed train crosses on its way to a sink, with
/// the far node's resistance the reactance is judged against at delivery.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct CouplingCrossing {
    /// The capacitor's reference designator.
    pub(crate) capacitor: String,
    /// Identity root of the net on the far side of the capacitor.
    pub(crate) far_root: usize,
    /// The capacitor's parsed value.
    pub(crate) farads: f64,
    /// The far node's resistance estimate: the smallest conduction edge
    /// incident to its root, or `+∞` when nothing resistive touches it (a
    /// lone CMOS input, whose input resistance is what the datasheet says
    /// it is — large). See [`Resolver::route_pulses`] for why an estimate.
    pub(crate) far_ohms: f64,
}

/// A pulse sink reached only across one or more coupling capacitors.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct CoupledSink {
    /// The sink endpoint.
    pub(crate) sink: EndpointId,
    /// Identity roots of the sink's own conduction segment — the nets on the
    /// far side of the last capacitor, within the collapse radius. The
    /// delivery gate for a coupled sink spans these alone: the capacitor is
    /// what sources the far node, so a `Floating` DC state there is not a
    /// barrier, while `Contention` (a fought node clamps the coupled
    /// signal) is.
    pub(crate) gate_roots: Vec<usize>,
    /// The capacitors crossed, source side first.
    pub(crate) couplings: Vec<CouplingCrossing>,
}

/// One derived source→sinks pulse route (see [`Resolver::route_pulses`]).
pub(crate) struct PulseRouteSpec {
    /// Pulse source endpoint the route originates at.
    pub(crate) source: EndpointId,
    /// Sink endpoints reachable through the collapsed conduction link.
    pub(crate) sinks: Vec<EndpointId>,
    /// Identity roots of every net the collapsed link spans — delivery is
    /// gated on their resolved state, exactly as stream bytes are.
    pub(crate) path_roots: Vec<usize>,
    /// Sinks reached across a coupling capacitor: delivered per sink, after
    /// the reactance rule and their own gate ([`CoupledSink`]).
    pub(crate) coupled: Vec<CoupledSink>,
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
    /// Coupling capacitors: AC paths for rate routing only, never
    /// conduction (see [`CouplingCapacitor`]).
    couplings: Vec<CouplingCapacitor>,
    /// Drive-capable endpoints, indexed by [`EndpointId`].
    slots: Vec<DriveSlot>,
    /// Piecewise-linear elements, in declaration order — the order their
    /// region tests are evaluated in within a cluster.
    elements: Vec<Element>,
    /// The current into each endpoint from the last solve of its cluster
    /// (`None` where the cluster resolved by projection).
    endpoint_currents: Vec<Option<Amps>>,
    /// The current through each element from the last solve of its cluster.
    element_currents: Vec<Option<Amps>>,
    power_sources: Vec<(usize, Volts)>,
    stuck_sources: Vec<(usize, Volts)>,
    digital_senses: Vec<usize>,
    analog_senses: Vec<usize>,
    /// Current instruments' nets ([`ReadKind::Instrument`]): each
    /// escalates its cluster to a solve and is otherwise invisible — no
    /// floating-sense finding, no precedence over rule 2's fight findings.
    current_instruments: Vec<usize>,
    power_senses: Vec<usize>,
    /// Whether a pass since the last publication stored a current that
    /// differs from the one it replaced — the only time the shared table
    /// is worth copying (`DESIGN.md` rule 8: a boot that never solves
    /// publishes no current table).
    currents_changed: bool,
    /// Pulse-capable pins, in registration order (pulse routing).
    streams: Vec<StreamPin>,
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
    /// How many times a pass escalated a cluster to the [`ClusterSolver`]
    /// (`DESIGN.md` rule 8: a solve runs only where sources within a factor
    /// of ten disagree or an analog sense asks — everything else is a
    /// projection). One relaxed increment per matrix built, nothing on the
    /// projection path. Shared with the engine handle so a test can assert
    /// a run's escalation count as the budget it is.
    escalated_solves: Arc<AtomicU64>,
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
    /// Drive-capable endpoints in the cluster, ascending.
    slots: Vec<usize>,
    /// Piecewise-linear elements in the cluster, in declaration order.
    elements: Vec<usize>,
    /// The declared terminals **outside** the cluster that its elements'
    /// conducting ends stamp against, as `(root, volts)` in source
    /// declaration order — the foreign constants of the solve, which
    /// source the cluster (NaN for an unmodelled rail). Empty for a cluster
    /// without elements.
    foreign: Vec<(usize, Volts)>,
    /// The declared terminals outside the cluster that its elements'
    /// **controls alone** read — constants for the region tests, sourcing
    /// nothing.
    foreign_controls: Vec<(usize, Volts)>,
    /// Power-rail sources in the cluster as `(root, volts)`, in declaration order.
    power: Vec<(usize, Volts)>,
    /// `net_stuck` sources in the cluster as `(root, volts)`, in declaration order.
    stuck: Vec<(usize, Volts)>,
    /// Digital sense pins in the cluster as `(registration position, net)`.
    digital_senses: Vec<(usize, usize)>,
    /// Analog sense pins in the cluster as `(registration position, net)`.
    analog_senses: Vec<(usize, usize)>,
    /// Current instruments in the cluster as `(registration position, net)`.
    current_instruments: Vec<(usize, usize)>,
    /// Power sense pins in the cluster as `(registration position, net)`.
    power_senses: Vec<(usize, usize)>,
    /// Minimum series resistance between roots, `roots.len()` square,
    /// row-major by position in `roots`; `INFINITY` where no path exists.
    dist: Vec<f64>,
}

/// Findings of one pass, each with the key that orders it the way a full
/// walk of the board would have reported it (a fight's contention and its
/// ambiguous level by first net index, floating senses by kind then
/// registration, power senses by registration, stranded injections by
/// endpoint), so a pass over any subset of clusters reports in the same
/// relative order as a pass over all of them.
#[derive(Default)]
struct PassFindings {
    contention: Vec<(usize, Finding)>,
    floating: Vec<((usize, usize), Finding)>,
    power: Vec<(usize, Finding)>,
    injection: Vec<(usize, Finding)>,
    /// Non-convergent element clusters, by first net index.
    nonconvergent: Vec<(usize, Finding)>,
}

impl PassFindings {
    fn emit(mut self, diagnostics: &mut Diagnostics) {
        // Stable sorts: a root's `AmbiguousLevel` is pushed right behind its
        // `Contention` under the same key and stays there.
        self.contention.sort_by_key(|(key, _)| *key);
        self.floating.sort_by_key(|(key, _)| *key);
        self.power.sort_by_key(|(key, _)| *key);
        self.injection.sort_by_key(|(key, _)| *key);
        self.nonconvergent.sort_by_key(|(key, _)| *key);
        let contention = self.contention.into_iter().map(|(_, f)| f);
        let floating = self.floating.into_iter().map(|(_, f)| f);
        let power = self.power.into_iter().map(|(_, f)| f);
        let injection = self.injection.into_iter().map(|(_, f)| f);
        let nonconvergent = self.nonconvergent.into_iter().map(|(_, f)| f);
        for finding in contention
            .chain(floating)
            .chain(power)
            .chain(injection)
            .chain(nonconvergent)
        {
            diagnostics.report(finding);
        }
    }
}

/// Whether two drive contributions are the same (bitwise on the floats, so
/// `NaN` compares equal to itself and a re-driven rail is a no-op).
fn same_drive(a: &Option<Drive>, b: &Option<Drive>) -> bool {
    match (a, b) {
        (None, None) => true,
        (Some(Drive::Thevenin(x)), Some(Drive::Thevenin(y))) => {
            x.volts.total_cmp(&y.volts).is_eq() && x.impedance.total_cmp(&y.impedance).is_eq()
        }
        (Some(Drive::Current { amps: x }), Some(Drive::Current { amps: y })) => {
            x.total_cmp(y).is_eq()
        }
        _ => false,
    }
}

/// The form a drive takes in the slot table. A Thevenin drive behind a
/// non-finite impedance *is* a released pin — `NODES.md` §10, "`ohms = ∞` is
/// normalised to released at the slot, never ranked" — so it becomes `None`
/// here, before it can source a cluster, rank against anything, or
/// escalate a solve. A non-finite injection is dropped the same way.
fn normalise_drive(drive: Option<Drive>) -> Option<Drive> {
    match drive {
        Some(Drive::Thevenin(t)) if !t.impedance.is_finite() => None,
        Some(Drive::Current { amps }) if !amps.is_finite() => None,
        other => other,
    }
}

impl Resolver {
    /// New resolver over `net_count` nets with the given identity merges.
    pub(crate) fn new(net_count: usize, identity: Dsu) -> Self {
        Self {
            identity,
            edges: Vec::new(),
            couplings: Vec::new(),
            slots: Vec::new(),
            elements: Vec::new(),
            endpoint_currents: Vec::new(),
            element_currents: Vec::new(),
            power_sources: Vec::new(),
            stuck_sources: Vec::new(),
            digital_senses: Vec::new(),
            analog_senses: Vec::new(),
            current_instruments: Vec::new(),
            power_senses: Vec::new(),
            currents_changed: false,
            streams: Vec::new(),
            net_count,
            topology: None,
            topology_version: 0,
            dirty: Vec::new(),
            escalated_solves: Arc::new(AtomicU64::new(0)),
        }
    }

    /// The shared escalation counter, for the engine handle to read after
    /// the resolver has moved to the engine thread.
    pub(crate) fn escalation_counter(&self) -> Arc<AtomicU64> {
        Arc::clone(&self.escalated_solves)
    }

    /// How many cluster solves every pass so far has escalated to the
    /// [`ClusterSolver`] (see the field).
    pub(crate) fn escalated_solves(&self) -> u64 {
        self.escalated_solves.load(Ordering::SeqCst)
    }

    /// The census of the board's conduction clusters: the identity roots
    /// each holds, one entry per cluster in ascending cluster-root order,
    /// roots ascending within. A root is one electrical node after harness
    /// and `pin_short` merges, so an entry's length is the size `m` of the
    /// matrix an escalated solve of that cluster would build. Snapshotted
    /// for [`crate::BuiltSystem`] so a board's cluster sizes are assertable
    /// as the engine-cost bound they are (`DESIGN.md` rule 4).
    pub(crate) fn cluster_roots(&mut self, net_count: usize) -> Vec<Vec<NetId>> {
        self.ensure_topology(net_count);
        self.topology
            .as_ref()
            .expect("ensure_topology built it")
            .clusters
            .iter()
            .map(|c| c.roots.iter().map(|&r| NetId(r)).collect())
            .collect()
    }

    /// Add a conduction edge between two nets.
    pub(crate) fn add_edge(&mut self, a: usize, b: usize, ohms: f64) {
        self.topology_version += 1;
        self.edges.push((a, b, ohms));
    }

    /// Add a piecewise-linear element between two nets, with an optional
    /// control net ([`crate::Branch`] resolved to nets), returning its
    /// index. An element is a **membership edge among its non-terminal
    /// nets**: its two ends and its control net — so a gate is in-cluster
    /// and no cross-cluster dirtying is needed — join one conduction
    /// cluster, a declared terminal among them stays outside as a foreign
    /// constant of that cluster's solve (`build_topology`), and a cluster
    /// holding an element always solves (its regions have no projection
    /// form). It is not a path for source-strength ranking: the linear
    /// sources around it rank as they do, and the solve decides the rest.
    pub(crate) fn add_element(
        &mut self,
        a: usize,
        b: usize,
        curve: PwlCurve,
        control: Option<(usize, RegionTest)>,
        reference: String,
    ) -> usize {
        self.topology_version += 1;
        self.elements.push(Element {
            a,
            b,
            curve,
            control,
            reference,
        });
        self.element_currents.push(None);
        self.elements.len() - 1
    }

    /// The currents the last passes produced ([`CurrentTable`]), for the
    /// engine to publish beside the net states.
    pub(crate) fn current_table(&self) -> CurrentTable {
        let mut endpoints = self.endpoint_currents.clone();
        endpoints.resize(self.slots.len(), None);
        CurrentTable {
            endpoints,
            elements: self.element_currents.clone(),
        }
    }

    /// Copy the current table into `table` — no allocation once the table
    /// has its size, which is every publication after the first.
    pub(crate) fn copy_current_table_into(&self, table: &mut CurrentTable) {
        table.endpoints.clear();
        table.endpoints.extend_from_slice(&self.endpoint_currents);
        table.endpoints.resize(self.slots.len(), None);
        table.elements.clear();
        table.elements.extend_from_slice(&self.element_currents);
    }

    /// Whether a pass since the last call stored a current different from
    /// the one it replaced; clears the flag.
    pub(crate) fn take_currents_changed(&mut self) -> bool {
        std::mem::take(&mut self.currents_changed)
    }

    /// Store the currents one pass computed, noting whether any differs
    /// from what was stored. A pass over clusters that resolved by
    /// projection alone carries nothing here and costs nothing.
    fn apply_currents(&mut self, pass: PassCurrents) {
        if pass.endpoints.is_empty() && pass.elements.is_empty() {
            return;
        }
        if self.endpoint_currents.len() < self.slots.len() {
            self.endpoint_currents.resize(self.slots.len(), None);
        }
        for (endpoint, amps) in pass.endpoints {
            if !same_current(&self.endpoint_currents[endpoint], &amps) {
                self.endpoint_currents[endpoint] = amps;
                self.currents_changed = true;
            }
        }
        for (element, amps) in pass.elements {
            if !same_current(&self.element_currents[element], &amps) {
                self.element_currents[element] = amps;
                self.currents_changed = true;
            }
        }
    }

    /// Add a coupling capacitor between two nets: an AC path for rate
    /// routing ([`Resolver::route_pulses`]), never a conduction edge.
    pub(crate) fn add_coupling(&mut self, a: usize, b: usize, farads: f64, reference: String) {
        self.topology_version += 1;
        self.couplings.push(CouplingCapacitor {
            a,
            b,
            farads,
            reference,
        });
    }

    /// Register a drive-capable endpoint with its initial Thevenin
    /// contribution (idle-high for push-pull digital at build; `None` for
    /// sense pins).
    pub(crate) fn add_endpoint(
        &mut self,
        net: usize,
        pin: PinRef,
        initial: Option<TheveninDrive>,
    ) -> EndpointId {
        self.add_endpoint_with(net, pin, initial.map(Drive::Thevenin))
    }

    /// [`Self::add_endpoint`] for any initial [`Drive`].
    pub(crate) fn add_endpoint_with(
        &mut self,
        net: usize,
        pin: PinRef,
        initial: Option<Drive>,
    ) -> EndpointId {
        self.topology_version += 1;
        self.slots.push(DriveSlot {
            net,
            pin,
            drive: normalise_drive(initial),
        });
        EndpointId(self.slots.len() - 1)
    }

    /// Replace an endpoint's drive contribution (`None` releases to high-Z;
    /// so does a Thevenin drive behind a non-finite impedance, see
    /// [`normalise_drive`]). Live path only; the next pass sees the new table.
    ///
    /// Returns whether the table changed. An identical drive is a no-op that
    /// marks nothing dirty — a card re-asserting the level it already holds,
    /// or a pin re-driven high on every clock edge, costs no resolution.
    pub(crate) fn set_drive(&mut self, endpoint: EndpointId, drive: Option<Drive>) -> bool {
        let drive = normalise_drive(drive);
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

    /// Register a current instrument's net: its cluster solves on every
    /// pass that touches it, and nothing is reported for it.
    pub(crate) fn add_current_instrument(&mut self, net: usize) {
        self.topology_version += 1;
        self.current_instruments.push(net);
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

        // The declared terminals — rails (modelled or not) and stuck
        // faults — as the roots no path continues past, and no element
        // unions through.
        let terminal_roots: Vec<usize> = self
            .power_sources
            .iter()
            .chain(self.stuck_sources.iter())
            .map(|(net, _)| root_of[*net])
            .collect();
        let is_terminal = |root: usize| terminal_roots.contains(&root);

        // Conduction clusters: identity merges are 0-ohm, conduction edges
        // connect within a cluster without merging identity, and an
        // element is a membership edge among its **non-terminal** nets —
        // its two ends and its control, so a gate is in-cluster. A
        // declared terminal is a constant, and a constant is a boundary: an
        // element touching one stamps against it as a foreign constant of
        // its own cluster rather than joining the terminal's (`NODES.md`
        // §8 phase 3, the parts record — what keeps a polarity FET whose
        // gate is on the stuck ground out of the ground cluster's solves).
        let mut conduction = Dsu::new(n);
        for (i, &root) in root_of.iter().enumerate() {
            conduction.union(root, i);
        }
        for (a, b, _ohms) in &self.edges {
            conduction.union(root_of[*a], root_of[*b]);
        }
        let homes: Vec<Option<ElementHome>> = self
            .elements
            .iter()
            .map(|element| element_home(element, &root_of, is_terminal))
            .collect();
        for (element, home) in self.elements.iter().zip(&homes) {
            let Some(home) = home else {
                continue;
            };
            for net in element.nets() {
                let root = root_of[net];
                if !is_terminal(root) {
                    conduction.union(home.root, root);
                }
            }
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
                let slots: Vec<usize> = (0..self.slots.len())
                    .filter(|&si| cluster_index[self.slots[si].net] == cid)
                    .collect();
                let elements: Vec<usize> = (0..self.elements.len())
                    .filter(|&ei| {
                        homes[ei]
                            .as_ref()
                            .is_some_and(|home| cluster_index[home.root] == cid)
                    })
                    .collect();
                // The terminal roots the cluster's elements name outside
                // it — conducting ends, and controls that are not also an
                // end — each with every source declared on it, in the
                // sources' own order (a fought terminal arrives as the
                // fight it is).
                let mut foreign_roots: Vec<usize> = Vec::new();
                let mut control_roots: Vec<usize> = Vec::new();
                for &ei in &elements {
                    let home = homes[ei].as_ref().expect("a homed element");
                    for &root in &home.terminal_ends {
                        if cluster_index[root] != cid && !foreign_roots.contains(&root) {
                            foreign_roots.push(root);
                        }
                    }
                    if let Some(root) = home.terminal_control {
                        if cluster_index[root] != cid && !control_roots.contains(&root) {
                            control_roots.push(root);
                        }
                    }
                }
                let sources_on = |roots: &[usize]| -> Vec<(usize, Volts)> {
                    self.power_sources
                        .iter()
                        .chain(self.stuck_sources.iter())
                        .map(|(net, volts)| (root_of[*net], *volts))
                        .filter(|(root, _)| roots.contains(root))
                        .collect()
                };
                let foreign = sources_on(&foreign_roots);
                let foreign_controls = sources_on(
                    &control_roots
                        .iter()
                        .copied()
                        .filter(|root| !foreign_roots.contains(root))
                        .collect::<Vec<_>>(),
                );
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
                // cluster's roots, ending at but never crossing a terminal;
                // INFINITY where no such path exists.
                let k = roots.len();
                let mut dist = vec![f64::INFINITY; k * k];
                for (ia, &ra) in roots.iter().enumerate() {
                    let from = min_path_ohms(&edges, ra, &terminal_roots);
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
                    slots,
                    elements,
                    foreign,
                    foreign_controls,
                    power: sources_in(&self.power_sources),
                    stuck: sources_in(&self.stuck_sources),
                    digital_senses: senses_in(&self.digital_senses),
                    analog_senses: senses_in(&self.analog_senses),
                    current_instruments: senses_in(&self.current_instruments),
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
    /// state; conduction clusters share sourced-ness. A cluster where
    /// disagreeing sources of comparable strength reach one root, where an
    /// analog sense asks, or where a current is injected escalates to
    /// `solver` (see [`project_root`]).
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
        let mut currents = PassCurrents::default();
        for cid in 0..topology.clusters.len() {
            self.resolve_cluster(&topology, cid, nets, &mut findings, &mut currents, solver);
        }
        findings.emit(diagnostics);
        self.apply_currents(currents);
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
        let mut currents = PassCurrents::default();
        for &cid in &dirty {
            self.resolve_cluster(&topology, cid, nets, &mut findings, &mut currents, solver);
        }
        findings.emit(diagnostics);
        self.apply_currents(currents);
        self.topology = Some(topology);
    }

    /// Resolve one conduction cluster from the current drive table: the
    /// sources each root is reached by, rule 2's ranking of them
    /// ([`project_root`]), the cluster solve where the ranking, an analog
    /// sense, a current injection or an element asks for it, state
    /// assignment for the cluster's nets, the cluster's findings and the
    /// currents its solve produced.
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
        currents: &mut PassCurrents,
        solver: &dyn ClusterSolver,
    ) {
        let c = &topology.clusters[cid];
        let root_of = &topology.root_of;
        let k = c.roots.len();
        let has_elements = !c.elements.is_empty();
        let pos_of_root = |root: usize| -> usize {
            c.roots
                .iter()
                .position(|&r| r == root)
                .expect("a cluster's sources sit on its own roots")
        };

        // Every Thevenin source in the cluster, in canonical order: drivers
        // in endpoint order, then rails and stuck faults as ideal 0 Ω
        // sources. This order is the SPICE card order the cluster solver
        // stamps (determinism), and the tie-break order of rule 2's ranking.
        // Beside each source, the slot it came from (`None` for a terminal).
        // NaN ("sourced at an unmodeled voltage") rails source the cluster
        // — a PowerIn on it is not unsourced — but carry no voltage to rank:
        // they are the fallback presentation of a root nothing numeric
        // reaches. Current injections are collected apart: they reach
        // nothing and rank nowhere; they are stamped into the solve. The
        // numeric terminals are collected apart too: they rank as ideal
        // sources and enter the solve as constants.
        let mut cluster_sourced = false;
        let mut sources: Vec<ClusterSource> = Vec::new();
        let mut source_slots: Vec<Option<usize>> = Vec::new();
        let mut terminals: Vec<ClusterTerminal> = Vec::new();
        let mut injections: Vec<ClusterInjection> = Vec::new();
        let mut injection_slots: Vec<usize> = Vec::new();
        for &si in &c.slots {
            let slot = &self.slots[si];
            match slot.drive {
                Some(Drive::Thevenin(drive)) => {
                    sources.push(ClusterSource {
                        node: NetId(root_of[slot.net]),
                        volts: drive.volts,
                        impedance: drive.impedance,
                    });
                    source_slots.push(Some(si));
                    cluster_sourced = true;
                }
                Some(Drive::Current { amps }) => {
                    injections.push(ClusterInjection {
                        node: NetId(root_of[slot.net]),
                        amps,
                    });
                    injection_slots.push(si);
                }
                None => {}
            }
        }
        let slot_source_count = sources.len();
        let mut unmodelled_roots: Vec<usize> = Vec::new();
        for (root, volts) in c.power.iter().chain(c.stuck.iter()) {
            cluster_sourced = true;
            if volts.is_nan() {
                unmodelled_roots.push(*root);
                continue;
            }
            sources.push(ClusterSource {
                node: NetId(*root),
                volts: *volts,
                impedance: 0.0,
            });
            source_slots.push(None);
            // A constant of the solve only in a cluster with elements; a
            // linear cluster's terminals stay the ideal sources above and
            // no table is built for them (`cluster.rs`, "Terminals are
            // constants").
            if has_elements {
                terminals.push(ClusterTerminal {
                    node: NetId(*root),
                    volts: *volts,
                });
            }
        }
        // The terminals outside the cluster its elements stamp against:
        // they source the cluster (a rail behind a diode is a rail) and
        // enter the solve as constants, but rank nowhere — no resistive
        // path leads to them, so no root is "reached" by one — and an
        // unmodelled rail behind an element reaches nothing at all. A
        // terminal a control alone reads is a constant for its region
        // test and sources nothing.
        let mut foreign_numeric = false;
        for &(root, volts) in &c.foreign {
            cluster_sourced = true;
            if volts.is_nan() {
                continue;
            }
            foreign_numeric = true;
            terminals.push(ClusterTerminal {
                node: NetId(root),
                volts,
            });
        }
        for &(root, volts) in &c.foreign_controls {
            if volts.is_finite() {
                terminals.push(ClusterTerminal {
                    node: NetId(root),
                    volts,
                });
            }
        }
        // The cluster's elements, with their nets as roots, in declaration
        // order — the order the flip loop evaluates them in.
        let elements: Vec<ClusterElement> = c
            .elements
            .iter()
            .map(|&ei| {
                let element = &self.elements[ei];
                ClusterElement {
                    a: NetId(root_of[element.a]),
                    b: NetId(root_of[element.b]),
                    curve: element.curve,
                    control: element
                        .control
                        .map(|(net, test)| (NetId(root_of[net]), test)),
                }
            })
            .collect();
        debug_assert_eq!(has_elements, !elements.is_empty());

        // The cluster solve — built at most once per pass, on demand. In a
        // cluster with elements the terminals enter as constants; in a
        // linear cluster they stay the ideal sources they always were (see
        // `cluster.rs`, "Terminals are constants", for why the split).
        let mut solution: Option<ClusterSolution> = None;
        let solve = |this: &Self| -> ClusterSolution {
            if has_elements {
                this.solve_cluster(
                    c,
                    &sources[..slot_source_count],
                    &terminals,
                    &injections,
                    &elements,
                    solver,
                )
            } else {
                this.solve_cluster(c, &sources, &[], &injections, &elements, solver)
            }
        };

        // Operating-point precedence. An analog sense reads a voltage, a
        // current injection has no projection form (its effect is `I · R`
        // along whatever the node is tied to), and an element's region has
        // none either — so any of them asks for the cluster's operating
        // point: every root reached by a numeric source publishes the solved
        // voltage. Rule 2's fight findings are not raised in a cluster an
        // analog sense or an injection escalated — the fight is visible as
        // the voltage the analog reader is handed, and `NetState::Contention`
        // would hand it nothing (the `nominal_analog_cluster` and
        // `net_stuck_shared_node` goldens pin this; it retires with
        // `NODES.md` §10's `Sense { volts }`). In a cluster with elements
        // the ranking still applies to the linear sources around them: the
        // states are the solve's, and the fights among the strong sources
        // are reported. A current instrument escalates the cluster too —
        // only a solved cluster has a current — and nothing else: it takes
        // no precedence, so the fights are reported beside the operating
        // point it is handed, as in an element cluster.
        let injected = injections.iter().any(|i| i.amps != 0.0);
        let precedence = !c.analog_senses.is_empty() || injected;
        let on_request = (!sources.is_empty() || foreign_numeric)
            && (precedence || !c.current_instruments.is_empty() || has_elements);
        if on_request {
            solution = Some(solve(self));
        }

        // Per root, by position in `c.roots`: the state, and rule 2's
        // findings — the strong sources fighting on it, and the solved
        // voltage that fell inside the dead band.
        let mut root_states: Vec<NetState> = Vec::with_capacity(k);
        let mut root_fights: Vec<(usize, Vec<usize>)> = Vec::new();
        let mut root_ambiguous: Vec<(usize, Volts)> = Vec::new();
        for (pr, &root) in c.roots.iter().enumerate() {
            let mut reaching: Vec<ReachingSource> = Vec::new();
            for (source, slot) in sources.iter().zip(&source_slots) {
                let path = c.dist[pr * k + pos_of_root(source.node.0)];
                if !path.is_finite() {
                    continue; // no resistive path: does not reach this root
                }
                reaching.push(ReachingSource {
                    slot: *slot,
                    volts: source.volts,
                    impedance: source.impedance,
                    path,
                });
            }
            // Nothing numeric reaches the root: an unmodelled rail that does
            // presents as up through the path to it (the supply gates read
            // `Pulled(High)` as a rail that is there); otherwise the root
            // floats.
            let unmodelled_or_floating = || {
                let nearest = unmodelled_roots
                    .iter()
                    .map(|&r| c.dist[pr * k + pos_of_root(r)])
                    .filter(|d| d.is_finite())
                    .fold(f64::INFINITY, f64::min);
                if nearest.is_finite() {
                    NetState::Pulled(Level::High, nearest)
                } else {
                    NetState::Floating
                }
            };
            let state = if let (true, Some(solution)) = (has_elements, solution.as_ref()) {
                // An element cluster: the solve decided every root, the
                // elements' far sides included (no resistive path reaches
                // those, so the ranking has nothing to say about them). The
                // ranking's fights among the linear sources are still
                // reported.
                if !reaching.is_empty() {
                    let mut solved = || match solution.state_of(NetId(root)) {
                        Some(NetState::Analog(v)) => Some(v),
                        _ => None,
                    };
                    let outcome = project_root(&reaching, &mut solved);
                    if let Some(fighting) = outcome.fight {
                        root_fights.push((root, fighting));
                    }
                    if let Some(volts) = outcome.ambiguous {
                        root_ambiguous.push((root, volts));
                    }
                }
                // No operating point: every non-terminal root floats
                // (`NODES.md` §7), an unmodelled rail's path notwithstanding.
                match solution.state_of(NetId(root)) {
                    Some(NetState::Analog(v)) => NetState::Analog(v),
                    _ if !solution.converged => NetState::Floating,
                    _ => unmodelled_or_floating(),
                }
            } else if reaching.is_empty() {
                unmodelled_or_floating()
            } else if let (true, Some(solution)) = (on_request, solution.as_ref()) {
                // Escalated by an instrument alone: the operating point is
                // published and rule 2's fights are reported beside it.
                if !precedence {
                    let mut solved = || match solution.state_of(NetId(root)) {
                        Some(NetState::Analog(v)) => Some(v),
                        _ => None,
                    };
                    let outcome = project_root(&reaching, &mut solved);
                    if let Some(fighting) = outcome.fight {
                        root_fights.push((root, fighting));
                    }
                    if let Some(volts) = outcome.ambiguous {
                        root_ambiguous.push((root, volts));
                    }
                }
                solution.state_of(NetId(root)).unwrap_or_else(|| {
                    tracing::warn!(net = %nets[root].name, "cluster solver omitted a node; reporting Floating");
                    NetState::Floating
                })
            } else {
                let mut solved = || -> Option<Volts> {
                    let solution = solution.get_or_insert_with(|| solve(self));
                    match solution.state_of(NetId(root)) {
                        Some(NetState::Analog(v)) => Some(v),
                        _ => None,
                    }
                };
                let outcome = project_root(&reaching, &mut solved);
                if let Some(fighting) = outcome.fight {
                    root_fights.push((root, fighting));
                }
                if let Some(volts) = outcome.ambiguous {
                    root_ambiguous.push((root, volts));
                }
                outcome.state
            };
            root_states.push(state);
        }

        // -- currents -------------------------------------------------------
        // What the solve, when there was one, says flows into every
        // endpoint and through every element of the cluster. A cluster
        // resolved by projection alone clears a stale reading once and is
        // otherwise silent here: the ROM boot's every pass is such a
        // cluster, and it must pay nothing for a table it never fills
        // (`DESIGN.md` rule 8).
        for &si in &c.slots {
            let amps = solution.as_ref().and_then(|solution| {
                let slot = &self.slots[si];
                match slot.drive {
                    Some(Drive::Thevenin(drive)) => {
                        match solution.state_of(NetId(root_of[slot.net])) {
                            Some(NetState::Analog(v)) => Some(
                                (v - drive.volts) / drive.impedance.max(IDEAL_SOURCE_FLOOR_OHMS),
                            ),
                            _ => None,
                        }
                    }
                    Some(Drive::Current { amps }) => Some(-amps),
                    None => match solution.state_of(NetId(root_of[slot.net])) {
                        Some(NetState::Analog(_)) => Some(0.0),
                        _ => None,
                    },
                }
            });
            if amps.is_some() || self.endpoint_currents.get(si).is_some_and(Option::is_some) {
                currents.endpoints.push((si, amps));
            }
        }
        for (position, &ei) in c.elements.iter().enumerate() {
            let amps = solution
                .as_ref()
                .and_then(|solution| solution.branch_currents.get(position).copied().flatten());
            if amps.is_some() || self.element_currents.get(ei).is_some_and(Option::is_some) {
                currents.elements.push((ei, amps));
            }
        }

        // -- state assignment -----------------------------------------------
        for &i in &c.nets {
            nets[i].state = root_states[pos_of_root(root_of[i])];
        }

        // -- findings ---------------------------------------------------------
        // A fight per identity root (deduped), keyed by the first net index
        // that carries it, naming the strong sources' pins (a terminal has
        // none); its ambiguous level, if any, right behind it.
        let mut reported_fights: Vec<usize> = Vec::new();
        for &i in &c.nets {
            let root = root_of[i];
            let Some((_, fighting)) = root_fights.iter().find(|(r, _)| *r == root) else {
                continue;
            };
            if reported_fights.contains(&root) {
                continue;
            }
            reported_fights.push(root);
            let name = nets[root.min(i)].name.clone();
            findings.contention.push((
                i,
                Finding::Contention {
                    net: name.clone(),
                    drivers: fighting
                        .iter()
                        .map(|&si| self.slots[si].pin.clone())
                        .collect(),
                },
            ));
            if let Some((_, volts)) = root_ambiguous.iter().find(|(r, _)| *r == root) {
                findings.contention.push((
                    i,
                    Finding::AmbiguousLevel {
                        net: name,
                        volts: *volts,
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
        // Power senses on an unsourced cluster, or floating behind an
        // element in a sourced one — a rail blocked by an off diode or an
        // off channel is no rail at the pin (deduped per identity root). A
        // cluster whose solve found no operating point floats every node
        // and is named once, by `NonConvergent`, not once more per power
        // pin (a cluster that never solved — sourced by an unmodelled rail
        // alone — reports as before).
        let non_convergent = solution.as_ref().is_some_and(|s| !s.converged);
        let mut reported_power: Vec<usize> = Vec::new();
        for &(pos, net) in &c.power_senses {
            let root = root_of[net];
            let unsourced = !cluster_sourced
                || (has_elements && !non_convergent && nets[net].state == NetState::Floating);
            if unsourced && !reported_power.contains(&root) {
                reported_power.push(root);
                findings.power.push((
                    pos,
                    Finding::PowerNetUnsourced {
                        net: nets[net].name.clone(),
                    },
                ));
            }
        }
        // A current injected where no Thevenin source reaches: the node has
        // no return path, stays Floating, and the injection went nowhere.
        for (injection, &si) in injections.iter().zip(&injection_slots) {
            if injection.amps == 0.0 {
                continue;
            }
            let pr = pos_of_root(injection.node.0);
            let reached = sources
                .iter()
                .any(|s| c.dist[pr * k + pos_of_root(s.node.0)].is_finite());
            if !reached {
                let slot = &self.slots[si];
                findings.injection.push((
                    si,
                    Finding::CurrentIntoFloatingNode {
                        net: nets[slot.net].name.clone(),
                        pin: slot.pin.clone(),
                    },
                ));
            }
        }
        // An element cluster whose flip loop found no consistent regions:
        // its nodes float (assigned above, from the solution) and the
        // finding names the elements, keyed by the cluster's first net.
        if let Some(solution) = solution.as_ref().filter(|s| has_elements && !s.converged) {
            let first = c.nets[0];
            findings.nonconvergent.push((
                first,
                Finding::NonConvergent {
                    cluster: nets[first].name.clone(),
                    elements: c
                        .elements
                        .iter()
                        .map(|&ei| self.elements[ei].reference.clone())
                        .collect(),
                    solves: solution.solves,
                },
            ));
        }
    }

    /// Escalate one cluster to the [`ClusterSolver`]: its roots as nodes,
    /// its identity-collapsed edges, the slot sources in canonical order,
    /// the terminals as constants, the current injections and the elements.
    /// Counted, because every solve is a cost the fast path did not pay
    /// ([`Self::escalated_solves`]).
    fn solve_cluster(
        &self,
        c: &ClusterTopo,
        sources: &[ClusterSource],
        terminals: &[ClusterTerminal],
        injections: &[ClusterInjection],
        elements: &[ClusterElement],
        solver: &dyn ClusterSolver,
    ) -> ClusterSolution {
        // The cluster's roots, then the numeric foreign terminals its
        // elements stamp against — nodes of the solve, constants by the
        // terminals handed over, never members whose state this cluster
        // publishes. An unmodelled rail stays outside: an element naming
        // it is dropped by the solver, as a branch to no voltage is.
        let mut nodes: Vec<NetId> = c.roots.iter().map(|&r| NetId(r)).collect();
        for &(root, volts) in &c.foreign {
            if volts.is_finite() && !nodes.contains(&NetId(root)) {
                nodes.push(NetId(root));
            }
        }
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
            sources: sources.to_vec(),
            injections: injections.to_vec(),
            terminals: terminals.to_vec(),
            elements: elements.to_vec(),
        };
        // Sequentially consistent on purpose: the count is read from another
        // thread (`EngineHandle::escalated_solves`, the ROM boot's budget
        // assertion) and must not lag the solves it counts. One increment
        // per escalated solve and nothing on the projection path, so the
        // ordering costs nothing measurable.
        self.escalated_solves.fetch_add(1, Ordering::SeqCst);
        solver.solve(&Cluster { nodes, resistors }, &inputs)
    }

    /// The resolver as it was before the per-cluster rewrite: one global pass
    /// over every net, the clusters found on the fly and the path
    /// resistances recomputed per root. Kept, test-only, as the reference the
    /// per-cluster pass is checked against. It shares [`project_root`] — the
    /// rule is one function — and nothing else: no topology cache, no dirty
    /// scope, no cluster tables.
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

        // The declared terminals: path barriers, and what no element
        // unions through.
        let terminal_roots: Vec<usize> = self
            .power_sources
            .iter()
            .chain(self.stuck_sources.iter())
            .map(|(net, _)| root_of[*net])
            .collect();
        let is_terminal = |root: usize| terminal_roots.contains(&root);

        // Conduction clusters: identity merges are 0-ohm, conduction edges
        // connect within a cluster without merging identity, elements are
        // membership edges among their non-terminal nets (ends and
        // control), as `build_topology` has it.
        let mut conduction = Dsu::new(n);
        for (i, &root) in root_of.iter().enumerate() {
            conduction.union(root, i);
        }
        for (a, b, _ohms) in &self.edges {
            conduction.union(root_of[*a], root_of[*b]);
        }
        let homes: Vec<Option<ElementHome>> = self
            .elements
            .iter()
            .map(|element| element_home(element, &root_of, is_terminal))
            .collect();
        for (element, home) in self.elements.iter().zip(&homes) {
            let Some(home) = home else {
                continue;
            };
            for net in element.nets() {
                let root = root_of[net];
                if !is_terminal(root) {
                    conduction.union(home.root, root);
                }
            }
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

        // Sources per cluster in canonical order — a dense walk of the slot
        // table, then rails, then faults — each beside the slot it came from;
        // injections likewise; the roots of NaN rails; the sourced clusters;
        // the numeric terminals and the elements per cluster.
        //
        // **Determinism (load-bearing):** iterate the DENSE drive table. This
        // `Vec`'s order is the SPICE card order
        // [`crate::cluster::QuasiStaticMna::solve`] stamps, so a hash walk
        // would make the deck (and, for a linear solver, last-bit voltages)
        // depend on a per-process hasher seed. See `DETERMINISM.md`.
        // hash-order: every map below is keyed access only.
        let mut cluster_sources: HashMap<usize, Vec<(ClusterSource, Option<usize>)>> =
            HashMap::new();
        let mut cluster_injections: HashMap<usize, Vec<(ClusterInjection, usize)>> = HashMap::new();
        let mut cluster_terminals: HashMap<usize, Vec<ClusterTerminal>> = HashMap::new();
        let mut cluster_elements: HashMap<usize, Vec<(ClusterElement, usize)>> = HashMap::new();
        let mut cluster_sourced: HashSet<usize> = HashSet::new();
        let mut unmodelled_roots: Vec<usize> = Vec::new();
        // The foreign terminal roots per cluster — conducting ends, and
        // controls — in first-appearance order (keyed access only; walked
        // in element order below).
        let mut cluster_foreign: HashMap<usize, Vec<usize>> = HashMap::new();
        let mut cluster_foreign_controls: HashMap<usize, Vec<usize>> = HashMap::new();
        for (ei, element) in self.elements.iter().enumerate() {
            let Some(home) = &homes[ei] else {
                continue;
            };
            let cluster = cluster_of[home.root];
            cluster_elements.entry(cluster).or_default().push((
                ClusterElement {
                    a: NetId(root_of[element.a]),
                    b: NetId(root_of[element.b]),
                    curve: element.curve,
                    control: element
                        .control
                        .map(|(net, test)| (NetId(root_of[net]), test)),
                },
                ei,
            ));
            for &root in &home.terminal_ends {
                if cluster_of[root] != cluster {
                    let foreign = cluster_foreign.entry(cluster).or_default();
                    if !foreign.contains(&root) {
                        foreign.push(root);
                    }
                }
            }
            if let Some(root) = home.terminal_control {
                if cluster_of[root] != cluster {
                    let controls = cluster_foreign_controls.entry(cluster).or_default();
                    if !controls.contains(&root) {
                        controls.push(root);
                    }
                }
            }
        }
        for (si, slot) in self.slots.iter().enumerate() {
            match slot.drive {
                Some(Drive::Thevenin(drive)) => {
                    cluster_sources
                        .entry(cluster_of[slot.net])
                        .or_default()
                        .push((
                            ClusterSource {
                                node: NetId(root_of[slot.net]),
                                volts: drive.volts,
                                impedance: drive.impedance,
                            },
                            Some(si),
                        ));
                    cluster_sourced.insert(cluster_of[slot.net]);
                }
                Some(Drive::Current { amps }) => {
                    cluster_injections
                        .entry(cluster_of[slot.net])
                        .or_default()
                        .push((
                            ClusterInjection {
                                node: NetId(root_of[slot.net]),
                                amps,
                            },
                            si,
                        ));
                }
                None => {}
            }
        }
        for (net, volts) in self.power_sources.iter().chain(self.stuck_sources.iter()) {
            cluster_sourced.insert(cluster_of[*net]);
            if volts.is_nan() {
                unmodelled_roots.push(root_of[*net]);
                continue;
            }
            cluster_sources.entry(cluster_of[*net]).or_default().push((
                ClusterSource {
                    node: NetId(root_of[*net]),
                    volts: *volts,
                    impedance: 0.0,
                },
                None,
            ));
            cluster_terminals
                .entry(cluster_of[*net])
                .or_default()
                .push(ClusterTerminal {
                    node: NetId(root_of[*net]),
                    volts: *volts,
                });
        }
        // The foreign constants: every source declared on a terminal root
        // an element cluster names outside itself, in source order. A
        // conducting end's sources the cluster and enters its solve; a
        // control's alone is a constant for the region test. Neither
        // ranks anywhere.
        let mut cluster_foreign_nodes: HashMap<usize, Vec<NetId>> = HashMap::new();
        let mut foreign_numeric: HashSet<usize> = HashSet::new();
        for cluster in cluster_elements.keys().copied() {
            let foreign = cluster_foreign.get(&cluster).cloned().unwrap_or_default();
            let controls: Vec<usize> = cluster_foreign_controls
                .get(&cluster)
                .map(|controls| {
                    controls
                        .iter()
                        .copied()
                        .filter(|root| !foreign.contains(root))
                        .collect()
                })
                .unwrap_or_default();
            for (net, volts) in self.power_sources.iter().chain(self.stuck_sources.iter()) {
                let root = root_of[*net];
                let conducting = foreign.contains(&root);
                if !conducting && !controls.contains(&root) {
                    continue;
                }
                if conducting {
                    cluster_sourced.insert(cluster);
                }
                if volts.is_nan() {
                    continue;
                }
                cluster_terminals
                    .entry(cluster)
                    .or_default()
                    .push(ClusterTerminal {
                        node: NetId(root),
                        volts: *volts,
                    });
                if conducting {
                    foreign_numeric.insert(cluster);
                    let nodes = cluster_foreign_nodes.entry(cluster).or_default();
                    if !nodes.contains(&NetId(root)) {
                        nodes.push(NetId(root));
                    }
                }
            }
        }
        // hash-order shape 3: membership only.
        let analog_clusters: HashSet<usize> = self
            .analog_senses
            .iter()
            .map(|&net| cluster_of[net])
            .collect();
        let instrument_clusters: HashSet<usize> = self
            .current_instruments
            .iter()
            .map(|&net| cluster_of[net])
            .collect();
        let terminal_roots: Vec<usize> = self
            .power_sources
            .iter()
            .chain(self.stuck_sources.iter())
            .map(|(net, _)| root_of[*net])
            .collect();

        let solve_cluster = |cluster: usize| -> ClusterSolution {
            let mut nodes: Vec<NetId> = (0..n)
                .filter(|&i| root_of[i] == i && cluster_of[i] == cluster)
                .map(NetId)
                .collect();
            if let Some(foreign) = cluster_foreign_nodes.get(&cluster) {
                nodes.extend(foreign.iter().copied());
            }
            let resistors: Vec<ClusterResistor> = root_edges
                .iter()
                .filter(|(a, _, _)| cluster_of[*a] == cluster)
                .map(|(a, b, ohms)| ClusterResistor {
                    a: NetId(*a),
                    b: NetId(*b),
                    ohms: *ohms,
                })
                .collect();
            // Terminals are constants in a cluster with elements and ideal
            // sources in a linear one, as `resolve_cluster` hands them.
            let with_elements = cluster_elements.contains_key(&cluster);
            let inputs = ClusterInputs {
                sources: cluster_sources
                    .get(&cluster)
                    .map(|sources| {
                        sources
                            .iter()
                            .filter(|(_, slot)| !with_elements || slot.is_some())
                            .map(|(s, _)| *s)
                            .collect()
                    })
                    .unwrap_or_default(),
                injections: cluster_injections
                    .get(&cluster)
                    .map(|injections| injections.iter().map(|(i, _)| *i).collect())
                    .unwrap_or_default(),
                terminals: if with_elements {
                    cluster_terminals.get(&cluster).cloned().unwrap_or_default()
                } else {
                    Vec::new()
                },
                elements: cluster_elements
                    .get(&cluster)
                    .map(|elements| elements.iter().map(|(e, _)| *e).collect())
                    .unwrap_or_default(),
            };
            solver.solve(&Cluster { nodes, resistors }, &inputs)
        };
        let has_elements = |cluster: usize| -> bool { cluster_elements.contains_key(&cluster) };
        // Operating-point precedence: an analog sense or an injection
        // silences rule 2's fights; an instrument or an element does not.
        let precedence = |cluster: usize| -> bool {
            analog_clusters.contains(&cluster)
                || cluster_injections
                    .get(&cluster)
                    .is_some_and(|injections| injections.iter().any(|(i, _)| i.amps != 0.0))
        };
        let on_request = |cluster: usize| -> bool {
            (cluster_sources.contains_key(&cluster) || foreign_numeric.contains(&cluster))
                && (precedence(cluster)
                    || instrument_clusters.contains(&cluster)
                    || has_elements(cluster))
        };
        // hash-order: `escalated`, `root_state`, `root_fights` and
        // `root_ambiguous` are keyed access only (`entry`, `get`, index) —
        // the walks that fill and read them are over dense indices.
        let mut escalated: HashMap<usize, ClusterSolution> = HashMap::new();
        let mut root_state: HashMap<usize, NetState> = HashMap::new();
        let mut root_fights: HashMap<usize, Vec<usize>> = HashMap::new();
        let mut root_ambiguous: HashMap<usize, Volts> = HashMap::new();
        for root in (0..n).filter(|&i| root_of[i] == i) {
            let cluster = cluster_of[root];
            let dist = min_path_ohms(&root_edges, root, &terminal_roots);
            let reaching: Vec<ReachingSource> = cluster_sources
                .get(&cluster)
                .map(|sources| {
                    sources
                        .iter()
                        .filter_map(|(source, slot)| {
                            let path = *dist.get(&source.node.0)?;
                            path.is_finite().then_some(ReachingSource {
                                slot: *slot,
                                volts: source.volts,
                                impedance: source.impedance,
                                path,
                            })
                        })
                        .collect()
                })
                .unwrap_or_default();
            let unmodelled_or_floating = || {
                let nearest = unmodelled_roots
                    .iter()
                    .filter(|&&r| cluster_of[r] == cluster)
                    .filter_map(|r| dist.get(r).copied())
                    .filter(|d| d.is_finite())
                    .fold(f64::INFINITY, f64::min);
                if nearest.is_finite() {
                    NetState::Pulled(Level::High, nearest)
                } else {
                    NetState::Floating
                }
            };
            let state = if has_elements(cluster) && on_request(cluster) {
                let solution = escalated
                    .entry(cluster)
                    .or_insert_with(|| solve_cluster(cluster));
                if !reaching.is_empty() {
                    let mut solved = || match solution.state_of(NetId(root)) {
                        Some(NetState::Analog(v)) => Some(v),
                        _ => None,
                    };
                    let outcome = project_root(&reaching, &mut solved);
                    if let Some(fighting) = outcome.fight {
                        root_fights.insert(root, fighting);
                    }
                    if let Some(volts) = outcome.ambiguous {
                        root_ambiguous.insert(root, volts);
                    }
                }
                match solution.state_of(NetId(root)) {
                    Some(NetState::Analog(v)) => NetState::Analog(v),
                    _ if !solution.converged => NetState::Floating,
                    _ => unmodelled_or_floating(),
                }
            } else if reaching.is_empty() {
                unmodelled_or_floating()
            } else if on_request(cluster) {
                let solution = escalated
                    .entry(cluster)
                    .or_insert_with(|| solve_cluster(cluster));
                if !precedence(cluster) {
                    let mut solved = || match solution.state_of(NetId(root)) {
                        Some(NetState::Analog(v)) => Some(v),
                        _ => None,
                    };
                    let outcome = project_root(&reaching, &mut solved);
                    if let Some(fighting) = outcome.fight {
                        root_fights.insert(root, fighting);
                    }
                    if let Some(volts) = outcome.ambiguous {
                        root_ambiguous.insert(root, volts);
                    }
                }
                solution.state_of(NetId(root)).unwrap_or(NetState::Floating)
            } else {
                let mut solved = || -> Option<Volts> {
                    match escalated
                        .entry(cluster)
                        .or_insert_with(|| solve_cluster(cluster))
                        .state_of(NetId(root))
                    {
                        Some(NetState::Analog(v)) => Some(v),
                        _ => None,
                    }
                };
                let outcome = project_root(&reaching, &mut solved);
                if let Some(fighting) = outcome.fight {
                    root_fights.insert(root, fighting);
                }
                if let Some(volts) = outcome.ambiguous {
                    root_ambiguous.insert(root, volts);
                }
                outcome.state
            };
            root_state.insert(root, state);
        }

        // -- state assignment -----------------------------------------------
        for (i, net) in nets.iter_mut().enumerate() {
            net.state = root_state[&root_of[i]];
        }

        // -- findings ---------------------------------------------------------
        // hash-order shape 3: every `reported*` set below is a dedup gate —
        // `.insert()` returning false suppresses a duplicate. The findings
        // themselves are emitted while walking dense indices, so their order is
        // net order, not hash order.
        let mut reported_fights: HashSet<usize> = HashSet::new();
        for i in 0..n {
            let root = root_of[i];
            let Some(fighting) = root_fights.get(&root) else {
                continue;
            };
            if !reported_fights.insert(root) {
                continue;
            }
            let name = nets[root.min(i)].name.clone();
            diagnostics.report(Finding::Contention {
                net: name.clone(),
                drivers: fighting
                    .iter()
                    .map(|&si| self.slots[si].pin.clone())
                    .collect(),
            });
            if let Some(&volts) = root_ambiguous.get(&root) {
                diagnostics.report(Finding::AmbiguousLevel { net: name, volts });
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

        // Power senses: unsourced clusters, or floating behind an element
        // in a sourced one (deduped per identity root).
        let mut reported_power: HashSet<usize> = HashSet::new();
        for &net in &self.power_senses {
            let root = root_of[net];
            let cluster = cluster_of[net];
            let non_convergent = escalated.get(&cluster).is_some_and(|s| !s.converged);
            let unsourced = !cluster_sourced.contains(&cluster)
                || (has_elements(cluster)
                    && !non_convergent
                    && nets[net].state == NetState::Floating);
            if unsourced && reported_power.insert(root) {
                diagnostics.report(Finding::PowerNetUnsourced {
                    net: nets[net].name.clone(),
                });
            }
        }

        // Injections where no Thevenin source reaches (dense walk of slots).
        for slot in &self.slots {
            let Some(Drive::Current { amps }) = slot.drive else {
                continue;
            };
            if amps == 0.0 {
                continue;
            }
            let dist = min_path_ohms(&root_edges, root_of[slot.net], &terminal_roots);
            let reached = cluster_sources
                .get(&cluster_of[slot.net])
                .is_some_and(|sources| {
                    sources
                        .iter()
                        .any(|(s, _)| dist.get(&s.node.0).is_some_and(|d| d.is_finite()))
                });
            if !reached {
                diagnostics.report(Finding::CurrentIntoFloatingNode {
                    net: nets[slot.net].name.clone(),
                    pin: slot.pin.clone(),
                });
            }
        }

        // Non-convergent element clusters, in order of their first net.
        let mut nonconvergent: Vec<(usize, Finding)> = Vec::new();
        for (cluster, solution) in &escalated {
            if solution.converged {
                continue;
            }
            let Some(elements) = cluster_elements.get(cluster) else {
                continue;
            };
            let first = (0..n)
                .find(|&i| cluster_of[i] == *cluster)
                .expect("a cluster has a net");
            nonconvergent.push((
                first,
                Finding::NonConvergent {
                    cluster: nets[first].name.clone(),
                    elements: elements
                        .iter()
                        .map(|(_, ei)| self.elements[*ei].reference.clone())
                        .collect(),
                    solves: solution.solves,
                },
            ));
        }
        // hash-order shape 2: collected from a map walk, sorted here.
        nonconvergent.sort_by_key(|(first, _)| *first);
        for (_, finding) in nonconvergent {
            diagnostics.report(finding);
        }
    }

    /// Derive the **pulse** routes from the current net topology — over the
    /// same collapsed-conduction reachability ([`STREAM_COLLAPSE_THRESHOLD`]),
    /// so a step signal that passes through series resistors or an isolator's
    /// short-circuit stub still reaches the drive.
    ///
    /// Two pulse sources reachable from each other raise
    /// [`Finding::StreamMismatch`] once per pair and neither routes: two step
    /// clocks driving one line is the same class of wiring error as two UART
    /// transmitters, and the underlying net additionally resolves
    /// `Contention` on its own.
    ///
    /// # Across a coupling capacitor
    ///
    /// A rate also crosses a **coupling capacitor** ([`Resolver::add_coupling`]:
    /// the module's TCXO reaches its buffer only through `C132`, and a
    /// capacitor is a DC open in every solve). Such a sink is a
    /// [`CoupledSink`]: the capacitors on its path are recorded with the far
    /// node's resistance estimate, and the crossing is judged **at delivery**,
    /// when the train's rate is known — the AC-coupling rule is
    /// `1/(2π·f·C) ≤ R_far / `[`COUPLING_REACTANCE_RATIO`], else the train
    /// stops at the capacitor with [`Finding::PulseNotCoupled`]. `R_far` is an
    /// estimate, deliberately cheap: the smallest conduction edge incident to
    /// the far node (the resistor that biases it, which is the Thevenin
    /// resistance of a self-biased stage to within its driver's few ohms), or
    /// `+∞` when nothing resistive touches it — a lone CMOS input. A strong
    /// driver on the far node is not in the estimate: that is a DC fight the
    /// resolution already reports on the node itself. The far side's own
    /// conduction segment is the coupled sink's delivery gate; the DC state
    /// of the source side is not, because the capacitor decouples it.
    ///
    /// # A terminal is a barrier here too
    ///
    /// Both reaches — the conduction path and the capacitor walk — end at a
    /// declared terminal and never continue past one, exactly as
    /// [`min_path_ohms`] does for projection (phase 1's decision (b) in
    /// `NODES.md` §8). Physically a terminal is an AC short to its own
    /// reference: a rate coupled into a stuck ground or a rail is shunted
    /// there, not forwarded through the next decoupling capacitor to
    /// whatever else hangs off that rail, and two sources that merely share
    /// decoupling to one terminal do not face each other. The source's own
    /// root is never a barrier, as in the path matrix.
    ///
    /// Rebuild on every topology-affecting change so pulse routes can never
    /// outlive the graph they were derived from.
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
        // Coupling capacitors between roots; one across a single root (both
        // ends merged) couples nothing.
        let root_couplings: Vec<(usize, usize, usize)> = self
            .couplings
            .iter()
            .enumerate()
            .map(|(ci, c)| (root_of[c.a], root_of[c.b], ci))
            .filter(|(a, b, _)| a != b)
            .collect();
        // The declared terminals — rails (modelled or not) and stuck
        // faults — as the roots neither reach continues past (the same
        // list `resolve` ranks sources against).
        let terminal_roots: Vec<usize> = self
            .power_sources
            .iter()
            .chain(self.stuck_sources.iter())
            .map(|(net, _)| root_of[*net])
            .collect();
        // The far-node resistance estimate, per root: the smallest
        // conduction edge touching it.
        let smallest_edge_at = |root: usize| -> f64 {
            root_edges
                .iter()
                .filter(|(a, b, _)| *a == root || *b == root)
                .map(|(_, _, ohms)| *ohms)
                .fold(f64::INFINITY, f64::min)
        };

        let mut routes = Vec::new();
        // hash-order shape 3: dedup gate for the paired mismatch report.
        let mut reported_pairs: HashSet<(usize, usize)> = HashSet::new();
        for (si, source) in self.streams.iter().enumerate() {
            if source.role != StreamRole::PulseSource {
                continue;
            }
            let origin = root_of[source.net];
            let dist = min_path_ohms(&root_edges, origin, &terminal_roots);
            let reachable = |net: usize| {
                dist.get(&root_of[net])
                    .is_some_and(|&ohms| ohms < STREAM_COLLAPSE_THRESHOLD)
            };
            // Reachability with the coupling capacitors as 0 Ω AC links:
            // a superset of `dist`, each entry carrying the capacitors on
            // its path with the root each was crossed into.
            let coupled_reach =
                coupled_reach(&root_edges, &root_couplings, origin, &terminal_roots);
            let coupled_to = |net: usize| -> Option<&Vec<(usize, usize)>> {
                coupled_reach
                    .get(&root_of[net])
                    .filter(|(ohms, path)| *ohms < STREAM_COLLAPSE_THRESHOLD && !path.is_empty())
                    .map(|(_, path)| path)
            };

            // Two sources that reach each other — by conduction or across
            // a capacitor — face each other.
            let facing: Vec<usize> = self
                .streams
                .iter()
                .enumerate()
                .filter(|(oi, other)| {
                    *oi != si
                        && other.role == StreamRole::PulseSource
                        && (reachable(other.net) || coupled_to(other.net).is_some())
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
            // Same deliberately conservative collapse gate as before: every
            // identity root within the collapse radius, sorted.
            // hash-order shape 2: `min_path_ohms` values are order-independent
            // and the collected keys are sorted here.
            let mut path_roots: Vec<usize> = dist
                .iter()
                .filter(|(_, &ohms)| ohms < STREAM_COLLAPSE_THRESHOLD)
                .map(|(&root, _)| root)
                .collect();
            path_roots.sort_unstable();

            // Sinks reached only across a capacitor, each with the crossings
            // on its path and the gate of its own conduction segment.
            let coupled: Vec<CoupledSink> = self
                .streams
                .iter()
                .filter(|s| s.role == StreamRole::PulseSink && !reachable(s.net))
                .filter_map(|s| {
                    let path = coupled_to(s.net)?;
                    let couplings: Vec<CouplingCrossing> = path
                        .iter()
                        .map(|&(ci, far_root)| {
                            // The far side is the root the walk crossed
                            // this capacitor into, recorded at the crossing.
                            let capacitor = &self.couplings[ci];
                            CouplingCrossing {
                                capacitor: capacitor.reference.clone(),
                                far_root,
                                farads: capacitor.farads,
                                far_ohms: smallest_edge_at(far_root),
                            }
                        })
                        .collect();
                    let last_far = couplings.last().map(|c| c.far_root)?;
                    let mut gate_roots: Vec<usize> =
                        min_path_ohms(&root_edges, last_far, &terminal_roots)
                            .iter()
                            .filter(|(_, &ohms)| ohms < STREAM_COLLAPSE_THRESHOLD)
                            .map(|(&root, _)| root)
                            .collect();
                    gate_roots.sort_unstable();
                    Some(CoupledSink {
                        sink: s.endpoint,
                        gate_roots,
                        couplings,
                    })
                })
                .collect();
            routes.push(PulseRouteSpec {
                source: source.endpoint,
                sinks,
                path_roots,
                coupled,
            });
        }
        routes
    }
}

/// Reachability from `from` over the conduction root-edges **and** the
/// coupling capacitors as 0 Ω links: for every reached root, the series ohms
/// of its cheapest path and the capacitors that path crossed, source side
/// first, each with the root it was crossed **into** — the far side, fixed
/// at the crossing rather than reconstructed afterwards from depths (a
/// capacitor reached from both ends at equal depth has no depth-defined far
/// side). A root in `terminals` may be reached but is never relaxed out of,
/// over either kind of link (`from` itself excepted), as in
/// [`min_path_ohms`]. Relaxation to a fixpoint; a strictly cheaper path
/// replaces, an equal one does not, so the result is a function of the edge
/// lists' order alone.
fn coupled_reach(
    root_edges: &[(usize, usize, f64)],
    root_couplings: &[(usize, usize, usize)],
    from: usize,
    terminals: &[usize],
) -> HashMap<usize, (f64, Vec<(usize, usize)>)> {
    let passable = |root: usize| root == from || !terminals.contains(&root);
    let mut reach: HashMap<usize, (f64, Vec<(usize, usize)>)> = HashMap::new();
    reach.insert(from, (0.0, Vec::new()));
    loop {
        let mut changed = false;
        for (a, b, ohms) in root_edges {
            for (x, y) in [(*a, *b), (*b, *a)] {
                if !passable(x) {
                    continue;
                }
                let Some((dx, path)) = reach.get(&x).cloned() else {
                    continue;
                };
                let candidate = dx + ohms;
                if reach.get(&y).is_none_or(|(dy, _)| candidate < *dy) {
                    reach.insert(y, (candidate, path));
                    changed = true;
                }
            }
        }
        for (a, b, ci) in root_couplings {
            for (x, y) in [(*a, *b), (*b, *a)] {
                if !passable(x) {
                    continue;
                }
                let Some((dx, path)) = reach.get(&x).cloned() else {
                    continue;
                };
                // A path never crosses the same capacitor twice.
                if path.iter().any(|(crossed, _)| crossed == ci) {
                    continue;
                }
                if reach.get(&y).is_none_or(|(dy, _)| dx < *dy) {
                    let mut crossed = path;
                    crossed.push((*ci, y));
                    reach.insert(y, (dx, crossed));
                    changed = true;
                }
            }
        }
        if !changed {
            break;
        }
    }
    reach
}

/// NaN-robust state equality for the sense change gate. [`NetState`]'s
/// derived `PartialEq` compares `f64` payloads with IEEE semantics, under
/// which `Analog(NaN) != Analog(NaN)` — a NaN-carrying state would read as
/// "changed" on every resolution pass, re-delivering senses forever (and a
/// sense callback that drives on delivery would then livelock the engine).
/// The resolver never publishes NaN (unmodeled rails are filtered before
/// state assignment), so this gate is defense in depth, not the primary
/// guarantee.
pub(crate) fn same_state(a: &NetState, b: &NetState) -> bool {
    match (a, b) {
        (NetState::Analog(x), NetState::Analog(y)) => x.total_cmp(y).is_eq(),
        (NetState::Pulled(la, xa), NetState::Pulled(lb, xb)) => {
            la == lb && xa.total_cmp(xb).is_eq()
        }
        _ => a == b,
    }
}

// ============================================================
// Source-strength projection (rule 2)
// ============================================================

/// One Thevenin source reaching a root, as rule 2 ranks it (`NODES.md`
/// "Three rules the taxonomy rests on", 2): by **total ohms** — its own
/// impedance plus the minimum series resistance from its root to the ranked
/// one (a terminal: the path alone).
#[derive(Debug, Clone, Copy, PartialEq)]
struct ReachingSource {
    /// The drive slot of a pad; `None` for a terminal (a rail or a
    /// `net_stuck` fault, both ideal).
    slot: Option<usize>,
    /// Open-circuit voltage.
    volts: Volts,
    /// The slot's own impedance; 0 for a terminal.
    impedance: Ohms,
    /// Minimum series resistance from the source's root to the ranked root:
    /// 0 when the source sits on it (or reaches it through 0 Ω edges).
    path: Ohms,
}

impl ReachingSource {
    fn total(&self) -> Ohms {
        self.impedance + self.path
    }

    fn level(&self) -> Level {
        level_of_volts(self.volts)
    }

    fn on_root(&self) -> bool {
        self.path == 0.0
    }

    /// A pull ([`WEAK_DRIVE_OHMS`] or more in total) never contends; it sets
    /// the level only when nothing stronger reaches the root.
    fn is_pull(&self) -> bool {
        self.total() >= WEAK_DRIVE_OHMS
    }

    /// The ohms a `Pulled` projection reports with this source as the
    /// winner: its series path, plus its own impedance when that impedance
    /// is itself weak — a 15 kΩ pad is a resistor to its rail, a 25 Ω pad
    /// is a driver.
    fn pulled_ohms(&self) -> Ohms {
        if self.impedance >= WEAK_DRIVE_OHMS {
            self.total()
        } else {
            self.path
        }
    }
}

/// What rule 2 decided for one root.
#[derive(Debug, Clone, PartialEq)]
struct RootOutcome {
    state: NetState,
    /// The strong sources on the root, as the slots to name in the
    /// `Contention` finding (a terminal has none), when they fought: a
    /// strong source disagreed and lost, or disagreeing strong sources
    /// solved.
    fight: Option<Vec<usize>>,
    /// The solved voltage, when it fell inside the dead band.
    ambiguous: Option<Volts>,
}

/// Rule 2 — source-strength projection, in one form — for one root.
///
/// `reaching` is every Thevenin source with a resistive path to the root, in
/// canonical order (slots by endpoint, then rails, then faults); `solved`
/// yields the root's voltage from the cluster solve, built on demand and
/// cached by the caller (`None` when the solver has nothing for the root).
///
/// 1. Every source is ranked by total ohms; the strongest sets the bar
///    (ties go to the earliest in canonical order).
/// 2. A pull — total at or above [`WEAK_DRIVE_OHMS`] — never contends: when
///    the strongest source is not a pull, the pulls are out of the contest.
/// 3. A source [`ESCALATION_IMPEDANCE_RATIO`] times the bar or more loses;
///    equal totals never lose (only ideal sources at 0 Ω tie). A strong
///    source that lost while disagreeing with the winner is a fight.
/// 4. The contest agrees → the strongest wins: a strong slot on the root is
///    `Driven(level)`, a terminal on the root `Analog(volts)`, anything
///    else `Pulled(level, ohms)` with the winner's series path (plus its own
///    impedance when that is itself weak, [`ReachingSource::pulled_ohms`]).
///    A slot source whose open-circuit voltage lies strictly inside the
///    [`V_IL`]/[`V_IH`] dead band has no level to project and is
///    `Analog(volts)` either way — the time-average a rate-mode gate drives,
///    the mid-rail rest of a self-biased stage.
/// 5. The contest disagrees → the root solves. Among strong sources, a
///    voltage strictly inside the [`V_IL`]/[`V_IH`] dead band is
///    `Contention` with `AmbiguousLevel`, outside it `Analog(v)`, either
///    way a fight. Among pulls alone (a divider between two rails) it is
///    `Analog(v)` and no finding.
fn project_root(
    reaching: &[ReachingSource],
    solved: &mut dyn FnMut() -> Option<Volts>,
) -> RootOutcome {
    let quiet = |state| RootOutcome {
        state,
        fight: None,
        ambiguous: None,
    };
    let Some(strongest) = reaching
        .iter()
        .min_by(|a, b| a.total().total_cmp(&b.total()))
    else {
        return quiet(NetState::Floating);
    };
    let bar = strongest.total();
    let loses =
        |s: &ReachingSource| s.total() > bar && s.total() >= bar * ESCALATION_IMPEDANCE_RATIO;
    let contest_is_strong = !strongest.is_pull();
    let contends = |s: &ReachingSource| !loses(s) && (!contest_is_strong || !s.is_pull());
    let level = strongest.level();
    let disagree = reaching.iter().any(|s| contends(s) && s.level() != level);
    let strong_slots = || -> Vec<usize> {
        reaching
            .iter()
            .filter(|s| !s.is_pull())
            .filter_map(|s| s.slot)
            .collect()
    };
    if !disagree {
        let silenced = contest_is_strong
            && reaching
                .iter()
                .any(|s| loses(s) && !s.is_pull() && s.level() != level);
        let state = if strongest.on_root() && strongest.slot.is_none() {
            NetState::Analog(strongest.volts)
        } else if V_IL < strongest.volts && strongest.volts < V_IH {
            // A slot source resting inside the dead band — a clock's
            // time-average, a self-biased stage — has no level to project:
            // the node is at its open-circuit voltage, and says so.
            NetState::Analog(strongest.volts)
        } else if strongest.on_root() && strongest.impedance < WEAK_DRIVE_OHMS {
            NetState::Driven(level)
        } else {
            NetState::Pulled(level, strongest.pulled_ohms())
        };
        return RootOutcome {
            state,
            fight: silenced.then(strong_slots),
            ambiguous: None,
        };
    }
    match solved() {
        None => quiet(NetState::Floating),
        Some(volts) if !contest_is_strong => quiet(NetState::Analog(volts)),
        Some(volts) if V_IL < volts && volts < V_IH => RootOutcome {
            state: NetState::Contention,
            fight: Some(strong_slots()),
            ambiguous: Some(volts),
        },
        Some(volts) => RootOutcome {
            state: NetState::Analog(volts),
            fight: Some(strong_slots()),
            ambiguous: None,
        },
    }
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
///
/// A path may **end** at a root in `terminals` but never continue past one:
/// a declared terminal — a rail, a harness supply, a `net_stuck` — holds its
/// node, so nothing on the far side of it sees a source on the near side
/// (`NODES.md` "Three rules the taxonomy rests on", 1: a terminal is a
/// cluster boundary; this is that rule in the path matrix, ahead of the
/// cluster split phase 4 makes of it). Without it the module's P59 pad,
/// driven high through the P59 pull-down to ground, would reach the core
/// rail's feedback divider on the other side of ground and rank there as a
/// pull disagreeing with ground — a divider solve on every MOSI edge of the
/// ROM boot. `from` itself is never a barrier: a terminal's own paths out
/// are what its dependents rank it by.
fn min_path_ohms(
    root_edges: &[(usize, usize, f64)],
    from: usize,
    terminals: &[usize],
) -> HashMap<usize, f64> {
    let passable = |root: usize| root == from || !terminals.contains(&root);
    let mut dist: HashMap<usize, f64> = HashMap::new();
    dist.insert(from, 0.0);
    loop {
        let mut changed = false;
        for (a, b, ohms) in root_edges {
            if let Some(da) = dist.get(a).copied().filter(|_| passable(*a)) {
                let candidate = da + ohms;
                if dist.get(b).is_none_or(|&db| candidate < db) {
                    dist.insert(*b, candidate);
                    changed = true;
                }
            }
            if let Some(db) = dist.get(b).copied().filter(|_| passable(*b)) {
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

/// One current instrument: the pin it reads through, its callback, and the
/// current it was last delivered (bitwise, so a repeat is not a change).
struct CurrentSub {
    handle: PinHandle,
    callback: CurrentCallback,
    last: Option<Amps>,
}

/// Whether two current readings are the same: bitwise on the value, so
/// `NaN` cannot read as a change forever, after folding `-0.0` to `0.0` —
/// the zero of a released slot and the negated zero of a `Drive::Current
/// { amps: 0.0 }` are one reading.
fn same_current(a: &Option<Amps>, b: &Option<Amps>) -> bool {
    match (a, b) {
        (None, None) => true,
        (Some(x), Some(y)) => (x + 0.0).total_cmp(&(y + 0.0)).is_eq(),
        _ => false,
    }
}

/// Live per-source pulse route: the derived sinks and the delivery gate.
///
/// Deliberately has no queue and no pacing slot — a pulse channel carries a
/// *rate*, so there is nothing in flight between rate changes.
struct LivePulseRoute {
    /// Sink endpoints on the collapsed link.
    sinks: Vec<EndpointId>,
    /// Identity roots of the nets the link spans (delivery gate).
    path_roots: Vec<usize>,
    /// Sinks reached across a coupling capacitor, judged per delivery.
    coupled: Vec<CoupledSink>,
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
    /// The currents the last solves produced, published beside `states`
    /// after every pass ([`CurrentTable`]).
    currents: Arc<Mutex<CurrentTable>>,
    /// Current instruments ([`Command::RegisterCurrent`]), in registration
    /// order, each with the value it was last delivered.
    current_subs: Vec<CurrentSub>,
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
    pending_drives: BTreeMap<u64, (EndpointId, Option<Drive>)>,
    next_drive_seq: u64,
    /// Has [`Command::ReleaseTime`] arrived? Virtual time is held at its
    /// initial value until it does, so every component's first schedule is
    /// anchored at the same instant. In both pacing modes: virtual time is
    /// only the counter this engine advances, and the gate sits before its
    /// only advance.
    clock_released: bool,
    /// Nets [`Command::DeclareRead`] declared read since the last full pass,
    /// with the kind of sense each becomes. Applied to the resolver
    /// together, after the drain batch, and answered with one full pass: a
    /// declaration is a topology input, so applying each as it arrived
    /// would leave the cache stale for every drive handled after it in the
    /// same batch — a full resolution and a topology rebuild per
    /// attach-time drive on a board with 64 pads.
    pending_reads: Vec<(usize, ReadKind)>,
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
        let currents_published = self.publish_currents();
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
        if currents_published {
            self.deliver_currents();
        }
    }

    /// Publish the currents the resolver's last passes produced beside the
    /// net states — only when a pass changed one, so a pass over clusters
    /// that resolved by projection alone (every pass of the ROM boot)
    /// copies nothing and locks nothing. Returns whether it published.
    fn publish_currents(&mut self) -> bool {
        if !self.resolver.take_currents_changed() {
            return false;
        }
        self.resolver
            .copy_current_table_into(&mut self.currents.lock().unwrap());
        true
    }

    /// Deliver every current instrument whose reading changed on the table
    /// just published, in registration order, with no lock held. Nothing
    /// to walk when nothing subscribed.
    fn deliver_currents(&mut self) {
        if self.current_subs.is_empty() {
            return;
        }
        for index in 0..self.current_subs.len() {
            let now = self.current_subs[index].handle.sense_current();
            if same_current(&self.current_subs[index].last, &now) {
                continue;
            }
            self.current_subs[index].last = now;
            let subscriber = self
                .nets
                .get(self.current_subs[index].handle.net().0)
                .map(|n| n.name.clone())
                .unwrap_or_else(|| format!("net {}", self.current_subs[index].handle.net().0));
            let callback = &self.current_subs[index].callback;
            self.deliver_contained(CallbackKind::Sense, &subscriber, || callback(now));
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
                        coupled: spec.coupled,
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
        let coupled = route.coupled.clone();
        if self.route_is_signal_capable(&path_roots) {
            for sink in sinks {
                self.deliver_pulse_to(source, sink, train);
            }
        } else {
            tracing::debug!(
                endpoint = source.0,
                "pulse train not delivered: a net on the route is not signal-capable"
            );
        }
        for sink in &coupled {
            self.deliver_coupled(source, sink, train);
        }
    }

    /// Deliver a train to a sink on the far side of one or more coupling
    /// capacitors: every crossing must pass the AC-coupling rule at the
    /// train's rate, and the sink's own segment must not be fought over.
    ///
    /// A held train (no rate) crosses unconditionally — a capacitor cannot
    /// refuse a stop, and the sink must learn of it.
    fn deliver_coupled(&self, source: EndpointId, sink: &CoupledSink, train: PulseTrain) {
        let hz = f64::from(train.pulses.freq_hz);
        if hz > 0.0 {
            for crossing in &sink.couplings {
                let reactance_ohms = 1.0 / (2.0 * std::f64::consts::PI * hz * crossing.farads);
                if reactance_ohms > crossing.far_ohms / COUPLING_REACTANCE_RATIO {
                    let net = self
                        .nets
                        .get(crossing.far_root)
                        .map(|n| n.name.clone())
                        .unwrap_or_default();
                    self.report_finding(Finding::PulseNotCoupled {
                        net,
                        capacitor: crossing.capacitor.clone(),
                        hz: train.pulses.freq_hz,
                        reactance_ohms,
                        far_ohms: crossing.far_ohms,
                    });
                    return;
                }
            }
        }
        // The far side of a capacitor may float at DC — the capacitor is
        // what sources it — but a fought node clamps what crosses.
        let clamped = sink.gate_roots.iter().any(|&root| {
            matches!(
                self.nets.get(root).map(|net| net.state),
                Some(NetState::Contention)
            )
        });
        if clamped {
            tracing::debug!(
                endpoint = source.0,
                "pulse train not delivered across a capacitor: the far side is fought over"
            );
            return;
        }
        self.deliver_pulse_to(source, sink.sink, train);
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
            if self.resolver.set_drive(endpoint, drive) {
                self.resolve_and_publish_dirty();
            }
        }
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
        let currents_published = self.publish_currents();
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
        if currents_published {
            self.deliver_currents();
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
            Command::DeclareRead { net, kind } => {
                self.pending_reads.push((net.0, kind));
            }
            Command::RegisterCurrent { handle, callback } => {
                // Once at registration, like a sense; the value it read is
                // what later passes are compared against.
                let now = handle.sense_current();
                let subscriber = self
                    .nets
                    .get(handle.net().0)
                    .map(|n| n.name.clone())
                    .unwrap_or_else(|| format!("net {}", handle.net().0));
                self.deliver_contained(CallbackKind::Sense, &subscriber, || callback(now));
                self.current_subs.push(CurrentSub {
                    handle,
                    callback,
                    last: now,
                });
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
                    .filter(|(_, route)| {
                        route.sinks.contains(&endpoint)
                            || route.coupled.iter().any(|c| c.sink == endpoint)
                    })
                    .map(|(&source, _)| source)
                    .collect();
                sources.sort_unstable();
                for source in sources {
                    let Some(&train) = self.pulse_state.get(&source) else {
                        continue;
                    };
                    let route = &self.pulse_routes[&source];
                    if route.sinks.contains(&endpoint) {
                        self.deliver_pulse_to(EndpointId(source), endpoint, train);
                    } else if let Some(sink) = route.coupled.iter().find(|c| c.sink == endpoint) {
                        self.deliver_coupled(EndpointId(source), sink, train);
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
            // Nets declared read since the last pass: declared together,
            // then one full resolution reports what floats under them.
            if !self.pending_reads.is_empty() {
                for (net, kind) in std::mem::take(&mut self.pending_reads) {
                    match kind {
                        ReadKind::Digital => self.resolver.add_digital_sense(net),
                        ReadKind::Instrument => self.resolver.add_current_instrument(net),
                    }
                }
                self.resolve_and_publish();
                work += 1;
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
    /// The resolver's escalation counter (`Resolver::escalated_solves`),
    /// readable after the resolver has moved to the engine thread.
    escalated_solves: Arc<AtomicU64>,
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
        let currents: Arc<Mutex<CurrentTable>> = Arc::new(Mutex::new(resolver.current_table()));
        let diagnostics = Arc::new(Mutex::new(Diagnostics::new()));
        let pending_schedules: Arc<AtomicUsize> = Arc::new(AtomicUsize::new(0));
        let (tx, rx) = mpsc::channel();
        let (control_tx, control_rx) = mpsc::channel();
        let escalated_solves = resolver.escalation_counter();

        let mut core = EngineCore {
            resolver,
            nets,
            solver,
            states: Arc::clone(&states),
            currents: Arc::clone(&currents),
            current_subs: Vec::new(),
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
            pending_reads: Vec::new(),
            pending_schedules: Arc::clone(&pending_schedules),
            quiescence_stalled: false,
            quiescence_timeout: quiescence_timeout.unwrap_or(STEPPED_QUIESCENCE_TIMEOUT),
            stepped_gap_logged: None,
            event_log: event_log.clone(),
        };
        core.resolve_and_publish();
        // Pulse routes are derived from net resolution, never installed beside
        // it: the routing pass runs against the just-resolved nets.
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
                currents,
                // Live path: drives and senses go to the engine, never to a
                // log.
                recorded_drives: None,
                recorded_senses: None,
            },
            diagnostics,
            event_log,
            escalated_solves,
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

    /// The currents the engine's last solves produced ([`CurrentTable`]).
    pub(crate) fn currents(&self) -> CurrentTable {
        self.link.currents.lock().unwrap().clone()
    }

    /// Snapshot of the cumulative live findings (initial resolution pass
    /// included).
    pub fn findings(&self) -> Vec<Finding> {
        self.diagnostics.lock().unwrap().findings().to_vec()
    }

    /// How many cluster solves the engine has escalated to the
    /// [`ClusterSolver`] so far, the initial resolution pass included. A
    /// solve runs only where sources within a factor of ten disagree or an
    /// analog sense asks; everything else is a projection (`DESIGN.md`
    /// rule 8) — so on a board with neither this reads 0 for a whole run,
    /// and a test can hold it to that as the budget it is.
    pub fn escalated_solves(&self) -> u64 {
        self.escalated_solves.load(Ordering::SeqCst)
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
    use vibes_behaviour::{behaviour, expect, Test};

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
        behaviour!(Test {
            id: "engine.drives-apply-in-issue-order",
            covers: Some("board/src/engine.rs#EngineCore::apply_ready_drives"),
            given: "two pins on one net whose drives reach the engine in the opposite order to the one they were issued in",
        });
        expect!("later-drive-waits", "the drive issued second takes no effect while the one issued first is still on its way");
        expect!(
            "issue-order",
            "observers see the net driven by the first drive and then in contention once the second lands",
            "the order drives were issued is the authoritative event order, whatever order they cross the queue in",
        );
        expect!("contention-reported", "the fight between the two pins is reported as a contention finding while the engine runs");
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
            drive: Some(Drive::Thevenin(low())),
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
            drive: Some(Drive::Thevenin(high())),
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
        behaviour!(Test {
            id: "engine.racing-drives-all-apply",
            covers: Some("board/src/component.rs#PinHandle::set_drive"),
            given: "two threads each driving its own pin on one shared net at the same moment",
        });
        expect!("applied-individually", "each drive is applied on its own: the net is seen driven by whichever landed first before it is seen contended");
        expect!(
            "ends-contended",
            "once both have landed the net is in contention"
        );
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
        behaviour!(Test {
            id: "engine.sense-drive-deferred",
            covers: Some("board/src/engine.rs#EngineCore::resolve_and_publish"),
            given: "two nets whose senses each drive the other's pin high on seeing a high, closing a feedback loop",
        });
        expect!(
            "deferred",
            "a drive issued from inside a sense takes effect in a later pass, after that sense has returned",
            "senses are delivered with no engine lock held, so a loop through driver, net and sense is deadlock-free by construction",
        );
        expect!("converges", "the loop settles: each net is seen to change exactly once and then nothing more is delivered");
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
        behaviour!(Test {
            id: "engine.unmodelled-rail-stable-state",
            covers: Some("board/src/engine.rs#Resolver::resolve_cluster"),
            given: "a sense on a power rail whose voltage is not modelled, while an unrelated net is driven back and forth ten times",
        });
        expect!(
            "delivered-once",
            "the rail's sense is delivered once, at registration, reading pulled high, and none of the ten passes deliver it again",
            "a rail with an unmodelled voltage must still project a definite level that compares equal to itself across passes, so its senses fire only on a real change",
        );
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
                drive: Some(Drive::Thevenin(if seq % 2 == 0 { high() } else { low() })),
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
        behaviour!(Test {
            id: "engine.unmodelled-rail-feedback-converges",
            covers: Some("board/src/engine.rs#EngineCore::resolve_and_publish_dirty"),
            given: "a sense on a rail whose voltage is not modelled, which drives a pin every time it is delivered",
        });
        expect!(
            "drive-lands",
            "the drive made from the registration delivery takes effect on its pin"
        );
        expect!("delivered-once", "the sense fires exactly once, at registration, and the pass its own drive causes leaves it silent");
        expect!(
            "definite-level",
            "the rail reads as pulled high with no series resistance"
        );
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
        fn solve(&self, cluster: &Cluster, inputs: &ClusterInputs) -> ClusterSolution {
            self.calls.lock().unwrap().push(cluster.nodes.clone());
            ClusterSolution {
                node_states: cluster
                    .nodes
                    .iter()
                    .map(|&n| (n, NetState::Analog(42.0)))
                    .collect(),
                regions: vec![crate::cluster::Region::Off; inputs.elements.len()],
                branch_currents: vec![None; inputs.elements.len()],
                converged: true,
                solves: 1,
                unknowns: cluster.nodes.len(),
            }
        }
    }

    /// A competing source within ESCALATION_IMPEDANCE_RATIO of the strongest
    /// source on a node sends the cluster through the ClusterSolver, and the
    /// node takes the solved voltage; a weak competing path (or an agreeing
    /// one) stays on the projection path.
    #[rstest]
    fn competing_path_within_ratio_escalates_to_cluster_solver() {
        behaviour!(Test {
            id: "engine.competing-path-escalates",
            covers: Some("board/src/engine.rs#project_root"),
            given: "a push-pull pin driving one net, joined by a series resistor to a net carrying a 3.3 volt rail",
        });
        expect!("close-opposing-rail-solved", "a low driver with the rail within ten times its own impedance sends the cluster through the analog solver exactly once");
        expect!(
            "driver-net-takes-solved",
            "the driver's net takes the solved voltage"
        );
        expect!("rail-holds-its-node", "the rail's own net stays at the rail's voltage, the driver reaching it through the resistor having lost");
        expect!("distant-opposing-rail-digital", "a low driver with the rail beyond that ratio keeps its digital level, the rail's net keeps its voltage, and nothing is solved");
        expect!("agreeing-rail-digital", "a high driver agreeing with the rail keeps its digital level however close the rail is, and nothing is solved");
        let calls: Arc<StdMutex<Vec<Vec<NetId>>>> = Arc::new(StdMutex::new(Vec::new()));
        let solver = RecordingSolver {
            calls: Arc::clone(&calls),
        };

        // 25 Ω driver low vs 3.3 V through 47 Ω: the rail reaches the
        // driver's node at 47 Ω, within 10 × 25 — a solve. The sentinel
        // 42 V sits above the dead band, so the driver's node reads it as
        // a voltage. The rail's node is held by the ideal rail; the driver
        // reaching it at 72 Ω lost.
        let mut resolver = Resolver::new(2, Dsu::new(2));
        resolver.add_endpoint(0, PinRef::new("U1", "1"), Some(low()));
        resolver.add_edge(0, 1, 47.0);
        resolver.add_power_source(1, 3.3);
        let mut net_table = nets(2);
        let mut diags = Diagnostics::new();
        resolver.resolve(&mut net_table, &mut diags, &solver);
        assert_eq!(calls.lock().unwrap().len(), 1, "cluster must escalate once");
        assert!(calls.lock().unwrap()[0].contains(&NetId(0)));
        assert!(calls.lock().unwrap()[0].contains(&NetId(1)));
        assert_eq!(net_table[0].state, NetState::Analog(42.0));
        assert_eq!(net_table[1].state, NetState::Analog(3.3));

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
        /// Records the slot sources in the order handed over, then the
        /// terminals — the constants a rail or a fault enters the solve as —
        /// each as the `(volts, 0 Ω)` ideal source it ranks as.
        fn solve(&self, cluster: &Cluster, inputs: &ClusterInputs) -> ClusterSolution {
            self.seen.lock().unwrap().push(
                inputs
                    .sources
                    .iter()
                    .map(|s| (s.volts, s.impedance))
                    .chain(inputs.terminals.iter().map(|t| (t.volts, 0.0)))
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
        behaviour!(Test {
            id: "engine.cluster-sources-in-pin-order",
            covers: Some("board/src/engine.rs#Resolver::resolve_cluster"),
            given: "several agreeing drivers on separate nets, chained by resistors into one cluster that carries an analog sense",
        });
        expect!(
            &format!("pin-order-with-{drivers}-drivers"),
            &format!(
                "with {drivers} drivers, the cluster is solved once, its sources handed over in the order their pins were registered"
            ),
            "the order the solver receives sources is the order their contributions are summed, and a summation order that varies between passes moves the last bits of the solved voltage",
        );
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
        behaviour!(Test {
            id: "engine.repeated-solves-bit-identical",
            covers: Some("board/src/engine.rs#Resolver::resolve_cluster"),
            given: "six drivers at assorted voltages and impedances tied into one node by zero-ohm links, with a ground rail 37 ohms away, resolved from scratch sixty-four times",
        });
        expect!(
            "bit-identical",
            "every pass solves the node to the same voltage, bit for bit",
            "resolution is a pure function of the drive table, so a run can be replayed and compared bit for bit",
        );
        expect!(
            "source-order-stable",
            "every pass hands the solver the same list: the drivers in pin order, then the rail"
        );
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
        behaviour!(Test {
            id: "engine.driven-level-crosses-series-resistor",
            covers: Some("board/src/engine.rs#project_root"),
            given: "a pin driving one net, and a second net reached from it only through a 47 ohm series resistor",
        });
        let level = match expect {
            Level::Low => "low",
            Level::High => "high",
        };
        expect!(
            &format!("{level}-level-crosses"),
            &format!("driven {level}, the far net reads {level} as well, pulled through the resistor's 47 ohms"),
            "a driven signal keeps its level across a series resistor, so a receiver behind one sees every bit its driver sends",
        );
        // driver —47 ohm— far
        let mut resolver = Resolver::new(2, Dsu::new(2));
        let endpoint = resolver.add_endpoint(0, PinRef::new("U1", "1"), None);
        resolver.add_edge(0, 1, 47.0);
        resolver.set_drive(endpoint, Some(Drive::Thevenin(drive)));

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

    /// A net between a driver and a pull-up reads the driver: the rail
    /// reaches it through 10 kΩ, the driver through 47 Ω, and the winner's
    /// path is what the projection reports — not the sum of every resistor
    /// in the cluster.
    #[rstest]
    #[case::pullup_3v3(3.3, low(), Level::Low)]
    #[case::pulldown_gnd(0.0, high(), Level::High)]
    fn a_driver_through_a_small_resistor_outvotes_a_pull_to_the_opposite_rail(
        #[case] rail: f64,
        #[case] drive: TheveninDrive,
        #[case] expect: Level,
    ) {
        behaviour!(Test {
            id: "engine.driver-outvotes-pull-on-middle-net",
            covers: Some("board/src/engine.rs#project_root"),
            given: "a driver behind 47 ohms and a 10 kilohm pull to a rail at the opposite level, meeting on one net",
        });
        expect!(
            "middle-reads-driver",
            "the net between them is pulled to the driver's level through the driver's 47 ohms",
            "a source is ranked by its own impedance plus the path to the net, so a pad 72 ohms away outranks a rail 10 kilohms away, and the ohms reported are the winner's path",
        );
        expect!(
            "rail-holds-its-net",
            "the rail's own net stays at the rail's voltage"
        );
        expect!(
            "nothing-reported",
            "nothing is reported: a pull is not a fight"
        );
        // driver(net0) —47— mid(net1) —10k— rail(net2)
        let mut resolver = Resolver::new(3, Dsu::new(3));
        let endpoint = resolver.add_endpoint(0, PinRef::new("U1", "1"), None);
        resolver.add_edge(0, 1, 47.0);
        resolver.add_edge(1, 2, 10_000.0);
        resolver.add_power_source(2, rail);
        resolver.add_digital_sense(1);
        resolver.set_drive(endpoint, Some(Drive::Thevenin(drive)));

        let mut net_table = nets(3);
        let mut diags = Diagnostics::new();
        resolver.resolve(&mut net_table, &mut diags, &QuasiStaticMna);
        assert_eq!(net_table[0].state, NetState::Driven(expect));
        assert_eq!(net_table[1].state, NetState::Pulled(expect, 47.0));
        assert_eq!(net_table[2].state, NetState::Analog(rail));
        assert!(diags.is_empty(), "{:?}", diags.findings());
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
        behaviour!(Test {
            id: "engine.injected-short-outvotes-driver",
            covers: Some("board/src/engine.rs#Resolver::resolve_cluster"),
            given: "a low-driving pin behind 10 kilohms, a middle net, and 47 ohms beyond it a far net where a short to 3.3 volts can be injected",
        });
        expect!(
            "driver-alone",
            "with nothing injected, the middle net is pulled low by the driver"
        );
        expect!(
            "short-wins",
            "with the short injected, the middle net is pulled high; the zero-ohm fault outvotes the driver behind ten kilohms",
            "an injected short is an ideal source and ranks like a declared rail, so a fault someone injected is visible from every net it reaches",
        );
        // stuck(3.3 V) —47 Ω— mid —10 kΩ— driver(Low, 25 Ω)
        let build = |stuck: bool| {
            let mut resolver = Resolver::new(3, Dsu::new(3));
            let endpoint = resolver.add_endpoint(2, PinRef::new("U1", "1"), None);
            resolver.add_edge(0, 1, 47.0);
            resolver.add_edge(1, 2, 10_000.0);
            resolver.set_drive(endpoint, Some(Drive::Thevenin(low())));
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
        behaviour!(Test {
            id: "engine.stuck-fault-vs-rail",
            covers: Some("board/src/engine.rs#Resolver::resolve_cluster"),
            given: "a 3.3 volt rail with a short injected onto the same net",
        });
        expect!(
            "opposing-short-contends",
            "a short to ground puts the net in contention",
            "two ideal sources that disagree are a fight the operator must see, whichever was declared first",
        );
        expect!(
            "opposing-short-reported",
            "the fight is reported as a contention finding"
        );
        expect!("agreeing-short-quiet", "a short to the rail's own voltage leaves the net at that voltage with nothing reported");
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
        behaviour!(Test {
            id: "engine.divider-reaches-solver",
            covers: Some("board/src/engine.rs#Resolver::resolve_cluster"),
            given: "two equal 4.7 kilohm resistors in series between a 3.3 volt rail and ground, with no push-pull driver anywhere",
        });
        expect!(
            "midpoint-solved",
            "the midpoint reads the divided voltage, 1.65 volts"
        );
        expect!("nothing-reported", "nothing is reported for it");
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
        behaviour!(Test {
            id: "engine.analog-sense-escalates",
            covers: Some("board/src/engine.rs#Resolver::resolve_cluster"),
            given: "a 3.3 volt rail reaching an unloaded input through a 4.7 kilohm pull-up",
        });
        expect!(
            "analog-reads-solved",
            "an analog sense on the input reads the solved voltage, the rail's full 3.3 volts"
        );
        expect!(
            "digital-stays-pulled",
            "a digital sense on the same input reads it as pulled high through the 4.7 kilohms",
            "a digital reader wants the pull-up view, which a numeric solve would replace with a bare voltage",
        );
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
        behaviour!(Test {
            id: "engine.wheel-separates-sub-microsecond",
            covers: Some("board/src/engine.rs#EngineCore::fire_due_timers"),
            given: "eight wakeups requested 500 nanoseconds apart, one bit period at 2 megabaud",
        });
        expect!(
            "each-at-its-instant",
            "all eight fire, each stamped with exactly its own requested instant",
            "at 2 megabaud eight bit edges fit inside one microsecond, and a timebase that cannot tell them apart collapses a synthesised waveform to one edge",
        );
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
        behaviour!(Test {
            id: "engine.repeated-deadline-wakes-once",
            covers: Some("board/src/engine.rs#EngineCore::arm"),
            given: "one component requesting the same wakeup instant fifty times and a later instant once, all before time is released",
        });
        expect!(
            "one-wake-per-instant",
            "the component is woken once at the first instant and once at the second",
            "one wake per component per instant, so a handler that re-arms itself from every wake stays at one wake per edge",
        );
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
        behaviour!(Test {
            id: "engine.periodic-survives-one-shot",
            covers: Some("board/src/engine.rs#EngineCore::fire_due_timers"),
            given: "a one-shot wakeup requested for exactly the instant a component's periodic wakeup is first due, with time held while both are armed",
        });
        expect!(
            "chain-continues",
            "the component is woken once at the shared instant and again at each following period"
        );
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
        behaviour!(Test {
            id: "engine.one-shot-fires-once",
            covers: Some("board/src/engine.rs#EngineCore::fire_due_timers"),
            given: "a single wakeup requested 100 virtual milliseconds ahead",
        });
        expect!(
            "at-or-after",
            "the wake is delivered stamped at or after the requested instant"
        );
        expect!("exactly-once", "it is delivered exactly once");
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
        behaviour!(Test {
            id: "engine.periodic-keeps-firing",
            covers: Some("board/src/engine.rs#EngineCore::fire_due_timers"),
            given: "a wakeup requested every 20 virtual milliseconds",
        });
        expect!(
            "keeps-firing",
            "the component keeps being woken, period after period"
        );
        expect!(
            "stamps-monotonic",
            "the delivered timestamps never go backwards"
        );
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
        behaviour!(Test {
            id: "engine.late-wakes-in-deadline-order",
            covers: Some("board/src/engine.rs#EngineCore::fire_due_timers"),
            given: "two components requesting wakeups at instants already in the past, the later instant requested first",
        });
        expect!(
            "deadline-order",
            "both fire, the earlier deadline first, whichever was requested first"
        );
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
        behaviour!(Test {
            id: "engine.shutdown-with-pending-timers",
            covers: Some("board/src/engine.rs#EngineHandle::drop"),
            given: "an engine parked on a wakeup a virtual minute away when its handle is dropped",
        });
        expect!(
            "prompt-shutdown",
            "the engine shuts down and is joined promptly, the pending wakeup abandoned"
        );
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
        behaviour!(Test {
            id: "engine.time-waits-for-in-flight-schedule",
            covers: Some("board/src/engine.rs#EngineCore::run_stepped_iteration"),
            given: "a component re-arming itself one bit period ahead from inside its own wake handler, while another thread floods the engine with drives",
        });
        expect!(
            "keeps-ticking",
            "the bit clock keeps ticking under the flood, twenty wakes delivered"
        );
        expect!(
            "grid-kept",
            "every wake lands exactly one bit period after the one before it",
            "virtual time waits for every requested wakeup to be armed before it advances, so a bit clock is never delivered late and a byte's edges keep their spacing",
        );
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
        behaviour!(Test {
            id: "engine.wheel-survives-drive-flood",
            covers: Some("board/src/engine.rs#EngineCore::run_stepped_iteration"),
            given: "a wakeup requested 10 virtual milliseconds ahead while another thread floods the engine with drives without pause",
        });
        expect!(
            "wake-fires",
            "the wakeup is still delivered while the flood is sustained",
            "drives are applied in bounded batches, so time can always advance to the next deadline however busy the drivers are",
        );
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
        behaviour!(Test {
            id: "engine.drive-order-gap-skipped",
            covers: Some("board/src/engine.rs#EngineCore::warn_on_stepped_drive_gap"),
            given: "a reserved place in the drive order that is never filled, with the next drive arriving normally behind it",
        });
        expect!(
            "later-drive-waits",
            "for a short while the later drive waits, in case the missing one is merely late"
        );
        expect!(
            "gap-skipped",
            "after a bounded wait the gap is skipped and the waiting drive takes effect"
        );
        expect!(
            "gap-reported",
            "the skip is reported as a finding naming the missing place in the order"
        );
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
            drive: Some(Drive::Thevenin(high())),
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
        behaviour!(Test {
            id: "engine.sense-crash-contained",
            covers: Some("board/src/engine.rs#EngineCore::deliver_contained"),
            given: "a sense handler that crashes on every delivery, beside a well-behaved sense on the same net, as the net is driven",
        });
        expect!(
            "others-served",
            "the well-behaved sense keeps receiving deliveries"
        );
        expect!(
            "engine-alive",
            "the engine stays alive",
            "net service for every other component survives one component's misbehaviour",
        );
        expect!(
            "crash-reported",
            "the crash is reported as a finding naming the net"
        );
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
            drive: Some(Drive::Thevenin(high())),
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
        behaviour!(Test {
            id: "engine.inert-handles-are-no-ops",
            covers: Some("board/src/engine.rs#EngineLink::send"),
            given: "a pin handle and a component's timing interface that are attached to no engine",
        });
        expect!(
            "quiet-no-ops",
            "driving the pin and requesting wakeups do nothing and raise no error"
        );
        expect!("senses-floating", "the pin reads as floating");
        let handle = crate::component::PinHandle::new(NetId(0));
        handle.set_drive(Some(high())); // dropped with a trace, no panic
        assert_eq!(handle.sense(), NetState::Floating);

        let io = crate::component::ComponentNetIo::default();
        io.schedule_at(0);
        io.schedule_every(1_000);
        io.on_wake(|_| {});
    }

    /// Disagreeing push-pull drivers coupled through a small series
    /// resistance fight numerically: each net sits at the voltage the loop
    /// current puts it at, and each side is projected through the dead band
    /// on its own. The crossed-TX/RX bench case (`cluster_handchecks`
    /// hand-checks the same loop). The same pair a weak-drive resistance
    /// apart are two pulls of each other and do not fight at all.
    #[rstest]
    fn disagreeing_drivers_across_a_small_resistor_solve_and_project_each_side_through_the_dead_band(
    ) {
        behaviour!(Test {
            id: "engine.disagreeing-drivers-across-resistor",
            covers: Some("board/src/engine.rs#project_root"),
            given: "a pin driving high and a pin driving low on separate nets joined by a 47 ohm series resistor",
        });
        expect!(
            "high-side-reads-its-voltage",
            "the high driver's net reads the divided voltage the loop current leaves it at, 2.45 volts, a valid high",
            "two drivers 72 ohms apart are within a factor of ten of each other, so the fight is solved and each net is projected through the dead band on its own",
        );
        expect!(
            "low-side-in-contention",
            "the low driver's net, at 0.85 volts, is in contention and reports that voltage as ambiguous"
        );
        expect!(
            "both-named",
            "the contention finding on each net names both fighting pins"
        );
        expect!(
            "weak-apart-no-fight",
            "through exactly 1 kilohm, the weak-drive limit itself, each driver keeps its own level and nothing is reported"
        );
        // 25 Ω high vs 25 Ω low through 47 Ω: loop current 3.3/97 A;
        // V_a = 3.3·72/97 = 2.4495 V (≥ V_IH), V_b = 3.3·25/97 = 0.8505 V
        // (inside the 0.8–2.0 V band).
        let mut resolver = Resolver::new(2, Dsu::new(2));
        resolver.add_endpoint(0, PinRef::new("U1", "1"), Some(high()));
        resolver.add_endpoint(1, PinRef::new("U2", "1"), Some(low()));
        resolver.add_edge(0, 1, 47.0);
        let mut net_table = nets(2);
        let mut diags = Diagnostics::new();
        resolver.resolve(&mut net_table, &mut diags, &QuasiStaticMna);
        assert!(
            matches!(net_table[0].state, NetState::Analog(v) if (v - 3.3 * 72.0 / 97.0).abs() < 1e-9),
            "the high side sits at the loop voltage; got {:?}",
            net_table[0].state
        );
        assert_eq!(net_table[1].state, NetState::Contention);
        for net in ["N0", "N1"] {
            assert!(
                diags.findings().iter().any(|f| matches!(
                    f,
                    Finding::Contention { net: n, drivers }
                        if n == net
                            && drivers.contains(&PinRef::new("U1", "1"))
                            && drivers.contains(&PinRef::new("U2", "1"))
                )),
                "{net}: the finding must name both fighting drivers; got {:?}",
                diags.findings()
            );
        }
        assert!(
            diags.findings().iter().any(|f| matches!(
                f,
                Finding::AmbiguousLevel { net, volts }
                    if net == "N1" && (volts - 3.3 * 25.0 / 97.0).abs() < 1e-9
            )),
            "the low side's voltage is inside the dead band; got {:?}",
            diags.findings()
        );

        // The same pair exactly WEAK_DRIVE_OHMS apart: each reaches the
        // other at 1025 Ω — a pull, which never contends.
        let mut resolver = Resolver::new(2, Dsu::new(2));
        resolver.add_endpoint(0, PinRef::new("U1", "1"), Some(high()));
        resolver.add_endpoint(1, PinRef::new("U2", "1"), Some(low()));
        resolver.add_edge(0, 1, WEAK_DRIVE_OHMS);
        let mut net_table = nets(2);
        let mut diags = Diagnostics::new();
        resolver.resolve(&mut net_table, &mut diags, &QuasiStaticMna);
        assert_eq!(net_table[0].state, NetState::Driven(Level::High));
        assert_eq!(net_table[1].state, NetState::Driven(Level::Low));
        assert!(
            diags.is_empty(),
            "two pulls do not fight: {:?}",
            diags.findings()
        );
    }

    /// Pulse routes collapse series passives below the threshold: a step clock
    /// through 47 Ω resistors reaches the drive, one behind a 4.7 kΩ isolation
    /// resistor does not.
    #[rstest]
    fn pulse_routes_collapse_series_passives_and_ignore_byte_roles() {
        behaviour!(Test {
            id: "engine.pulse-route-collapses-passives",
            covers: Some("board/src/engine.rs#Resolver::route_pulses"),
            given: "a step-clock source reaching one drive input through two 47 ohm resistors and another input through a 4.7 kilohm isolation resistor",
        });
        expect!(
            "near-input-only",
            "one route is derived, carrying the source to the input behind the small resistors alone",
            "series resistance below 1 kilohm is part of the link, and an isolation resistor marks where a step clock stops",
        );
        expect!("nothing-reported", "nothing is reported");
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
        behaviour!(Test {
            id: "engine.facing-pulse-sources",
            covers: Some("board/src/engine.rs#Resolver::route_pulses"),
            given:
                "two step-clock sources on one line, 47 ohms apart, with a drive input beyond them",
        });
        expect!("neither-routes", "neither source gets a route");
        expect!(
            "one-finding-per-pair",
            "exactly one finding is raised, naming both sources as a mismatched pair",
            "two step clocks driving one line is the same wiring error as two transmitters on one serial line",
        );
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
        behaviour!(Test {
            id: "engine.pulse-source-without-sink",
            covers: Some("board/src/engine.rs#Resolver::route_pulses"),
            given: "a step-clock source with nothing connected to it, and a drive input on another net with nothing connected either",
        });
        expect!(
            "route-with-no-inputs",
            "the source keeps a route with no inputs on it",
            "a source's current train is retained on its route, so an input attached later can be served straight away",
        );
        expect!("nothing-reported", "nothing is reported for either");
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
            endpoints: Vec<(usize, Option<Drive>)>,
            power: Vec<(usize, Volts)>,
            stuck: Vec<(usize, Volts)>,
            digital_senses: Vec<usize>,
            analog_senses: Vec<usize>,
            /// Current instruments' nets.
            instruments: Vec<usize>,
            power_senses: Vec<usize>,
            /// Piecewise-linear elements: `(a, b, curve, control)`.
            elements: Vec<RandomElement>,
        }

        /// One random element: `(a, b, curve, control)`.
        type RandomElement = (usize, usize, PwlCurve, Option<(usize, RegionTest)>);

        fn element_strategy(n: usize) -> impl Strategy<Value = RandomElement> {
            let curve = prop_oneof![
                Just(PwlCurve::Diode { vf: 0.75, r_d: 0.0 }),
                Just(PwlCurve::Diode { vf: 2.0, r_d: 5.0 }),
                Just(PwlCurve::Channel { r_on: 1.0 }),
                Just(PwlCurve::Channel { r_on: 20.0 }),
            ];
            let control = prop_oneof![
                1 => Just(None),
                3 => (
                    0..n,
                    prop_oneof![
                        Just(RegionTest::AtLeast(2.0)),
                        Just(RegionTest::AtLeast(1.5)),
                        Just(RegionTest::AtMost(-2.0)),
                        Just(RegionTest::AtMost(1.5)),
                    ]
                )
                    .prop_map(Some),
            ];
            (0..n, 0..n, curve, control)
        }

        fn drive_strategy() -> impl Strategy<Value = Option<Drive>> {
            prop_oneof![
                2 => Just(None),
                6 => (
                    prop_oneof![Just(0.0f64), Just(3.3), Just(5.0)],
                    prop_oneof![
                        Just(25.0f64),
                        Just(100.0),
                        Just(470.0),
                        Just(15_000.0),
                        Just(100_000.0),
                        Just(f64::INFINITY)
                    ],
                )
                    .prop_map(|(volts, impedance)| Some(Drive::Thevenin(TheveninDrive {
                        volts,
                        impedance
                    }))),
                1 => prop_oneof![Just(-1e-3f64), Just(0.0), Just(100e-6), Just(1e-3)]
                    .prop_map(|amps| Some(Drive::Current { amps })),
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
                    prop::collection::vec(0..n, 0..2),
                    prop::collection::vec(element_strategy(n), 0..3),
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
                            instruments,
                            power_senses,
                            elements,
                        )| Spec {
                            n,
                            merges,
                            edges,
                            endpoints,
                            power,
                            stuck,
                            digital_senses,
                            analog_senses,
                            instruments,
                            power_senses,
                            elements,
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
                    resolver.add_endpoint_with(*net, PinRef::new("U", format!("{i}")), *drive)
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
            for &net in &spec.instruments {
                resolver.add_current_instrument(net);
            }
            for &net in &spec.power_senses {
                resolver.add_power_sense(net);
            }
            for (i, &(a, b, curve, control)) in spec.elements.iter().enumerate() {
                resolver.add_element(a, b, curve, control, format!("B.D{i}"));
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

        /// Declared once, outside the case loop: the body below runs two
        /// thousand times, and the ledger wants one line per expectation.
        #[test]
        fn resolving_only_the_touched_cluster_matches_a_full_pass() {
            behaviour!(Test {
                id: "engine.incremental-resolve-matches-full-pass",
                covers: Some("board/src/engine.rs#Resolver::resolve_dirty"),
                given: "a random board of up to nine nets with shorts, resistors, drivers at assorted impedances including released and infinite ones, current injections, rails, injected faults, senses, and diodes and switched channels with random control pins, under a random sequence of drive changes",
            });
            expect!(
                "states-match",
                "after the first pass and after every change, each net's state equals what a full pass over the whole board gives",
                "clusters are electrically independent, so re-resolving only the cluster a drive touched is exact",
            );
            expect!(
                "findings-match",
                "after every change, the accumulated findings equal those of the full pass",
            );
            touched_cluster_cases();
        }

        proptest! {
            #![proptest_config(ProptestConfig { cases: 2000, ..ProptestConfig::default() })]
            fn touched_cluster_cases(
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

    // Piecewise-linear elements at the resolver (`NODES.md` §8 phase 3):
    // what the resolver hands the solver for an element cluster, and what
    // it publishes back.
    mod elements {
        use super::*;
        use crate::cluster::{ClusterTerminal, GMIN_OHMS};

        /// A solver that records what it is handed and answers with the
        /// real one.
        struct Spy {
            inputs: Arc<StdMutex<Vec<ClusterInputs>>>,
            unknowns: Arc<StdMutex<Vec<usize>>>,
        }

        impl ClusterSolver for Spy {
            fn solve(&self, cluster: &Cluster, inputs: &ClusterInputs) -> ClusterSolution {
                self.inputs.lock().unwrap().push(inputs.clone());
                let solution = QuasiStaticMna.solve(cluster, inputs);
                self.unknowns.lock().unwrap().push(solution.unknowns);
                solution
            }
        }

        /// The LED chain of `NODES.md` §2 rule 1 at the resolver: an
        /// inverter's output pad (net 0), 220 Ω to the anode (net 1), the
        /// LED to ground (net 2, a declared rail at 0 V). The resolver hands
        /// the rail to the solver as a constant, so the matrix has two
        /// unknowns, and publishes the solve on every net.
        #[rstest]
        fn an_led_chain_to_a_terminal_solves_with_two_unknowns() {
            behaviour!(Test {
                id: "engine.led-chain-solves-with-two-unknowns",
                covers: Some("board/src/engine.rs#Resolver::resolve_cluster"),
                given: "a pad driving 3.3 volts through 25 ohms, joined by 220 ohms to an LED's \
                        anode, the LED's cathode on a declared 0 volt rail",
            });
            expect!(
                "two-unknowns",
                "the solve builds a matrix of two unknowns, the pad's net and the anode",
                "a declared terminal enters an element cluster's solve as a constant, so only \
                 the nodes between the terminals are solved for"
            );
            expect!(
                "rail-handed-as-a-constant",
                "the rail reaches the solver as a constant, and the pad is the only source it is \
                 handed",
            );
            expect!(
                "chain-published",
                "the pad's net reads its driven voltage less the pad's own drop, the anode reads \
                 the LED's knee and the rail reads exactly 0 volts",
            );
            expect!(
                "one-escalation",
                "the pass escalates the cluster once",
                "an element cluster always solves, and once per pass"
            );
            let mut resolver = Resolver::new(3, Dsu::new(3));
            resolver.add_endpoint(0, PinRef::new("U9", "Y"), Some(high()));
            resolver.add_edge(0, 1, 220.0);
            resolver.add_element(
                1,
                2,
                PwlCurve::Diode { vf: 2.0, r_d: 0.0 },
                None,
                "B.D3".into(),
            );
            resolver.add_power_source(2, 0.0);
            let spy = Spy {
                inputs: Arc::new(StdMutex::new(Vec::new())),
                unknowns: Arc::new(StdMutex::new(Vec::new())),
            };
            let mut net_table = nets(3);
            let mut diags = Diagnostics::new();
            resolver.resolve(&mut net_table, &mut diags, &spy);
            assert_eq!(*spy.unknowns.lock().unwrap(), vec![2]);
            let handed = spy.inputs.lock().unwrap();
            assert_eq!(handed.len(), 1);
            assert_eq!(
                handed[0].terminals,
                vec![ClusterTerminal {
                    node: NetId(2),
                    volts: 0.0
                }]
            );
            assert_eq!(handed[0].sources.len(), 1, "the pad alone is a source");
            assert_eq!(handed[0].elements.len(), 1);
            // I = (3.3 − 2.0) / (25 + 220).
            let current = 1.3 / 245.0;
            let NetState::Analog(y) = net_table[0].state else {
                panic!("{:?}", net_table[0].state);
            };
            assert!((y - (3.3 - 25.0 * current)).abs() < 1e-6, "{y}");
            let NetState::Analog(anode) = net_table[1].state else {
                panic!("{:?}", net_table[1].state);
            };
            assert!((anode - 2.0).abs() < 1e-6, "{anode}");
            assert_eq!(net_table[2].state, NetState::Analog(0.0));
            assert_eq!(resolver.escalated_solves(), 1);
            assert!(diags.is_empty(), "{:?}", diags.findings());
        }

        /// The far side of an off diode is joined to the cluster by the
        /// element — the solve reaches it through the leakage — and floats.
        #[rstest]
        fn the_far_side_of_an_off_diode_is_in_the_cluster_and_floats() {
            behaviour!(Test {
                id: "engine.off-diode-far-side-floats",
                covers: Some("board/src/engine.rs#Resolver::build_topology"),
                given: "a diode whose anode net a pad drives high and whose cathode net carries \
                        nothing else",
            });
            expect!(
                "one-cluster",
                "the two nets are one conduction cluster",
                "an element is a membership edge: its far side is solved with its near side"
            );
            expect!(
                "cathode-floats",
                "the cathode net floats and the anode net reads the pad's voltage",
                "only the diode's gigaohm of leakage reaches the cathode, which is no source"
            );
            let mut resolver = Resolver::new(2, Dsu::new(2));
            resolver.add_endpoint(0, PinRef::new("U9", "Y"), Some(high()));
            resolver.add_element(
                0,
                1,
                PwlCurve::Diode { vf: 0.75, r_d: 0.0 },
                None,
                "B.D1".into(),
            );
            assert_eq!(resolver.cluster_roots(2), vec![vec![NetId(0), NetId(1)]]);
            let mut net_table = nets(2);
            let mut diags = Diagnostics::new();
            resolver.resolve(&mut net_table, &mut diags, &QuasiStaticMna);
            assert_eq!(net_table[0].state, NetState::Analog(3.3));
            assert_eq!(net_table[1].state, NetState::Floating);
            let _ = GMIN_OHMS;
        }
    }

    // Rule 2 — source-strength projection (`NODES.md` "Three rules the
    // taxonomy rests on", 2; `project_root`): the proof cases of phase 1's
    // engine half, each on the smallest topology that exhibits it.
    mod source_strength {
        use super::*;

        fn thevenin(volts: Volts, impedance: Ohms) -> Option<TheveninDrive> {
            Some(TheveninDrive { volts, impedance })
        }

        /// Whether `diags` holds a contention finding on `net` naming exactly
        /// `pins`.
        fn contention_naming(diags: &Diagnostics, net: &str, pins: &[PinRef]) -> bool {
            diags.findings().iter().any(|f| {
                matches!(
                    f,
                    Finding::Contention { net: n, drivers }
                        if n == net
                            && drivers.len() == pins.len()
                            && pins.iter().all(|p| drivers.contains(p))
                )
            })
        }

        /// A pull never contends: the 15 kΩ pad is a resistor to its rail,
        /// and the 30 Ω sink is the node's driver.
        #[rstest]
        fn a_weak_pad_high_against_a_strong_sink_low_is_driven_low() {
            behaviour!(Test {
                id: "engine.pull-loses-to-driver-quietly",
                covers: Some("board/src/engine.rs#project_root"),
                given: "a pad pulling a net high through 15 kilohms and a pin sinking the same net low at 30 ohms",
            });
            expect!(
                "driven-low",
                "the net is driven low",
                "a source of a kilohm or more in total is a pull, which sets a level only where nothing stronger reaches",
            );
            expect!("nothing-reported", "nothing is reported");
            expect!("zero-solves", "the pass costs no cluster solve");
            let mut resolver = Resolver::new(1, Dsu::new(1));
            resolver.add_endpoint(0, PinRef::new("P2", "P28"), thevenin(3.3, 15_000.0));
            resolver.add_endpoint(0, PinRef::new("U2", "SDA"), thevenin(0.0, 30.0));
            let mut net_table = nets(1);
            let mut diags = Diagnostics::new();
            resolver.resolve(&mut net_table, &mut diags, &QuasiStaticMna);
            assert_eq!(net_table[0].state, NetState::Driven(Level::Low));
            assert!(diags.is_empty(), "{:?}", diags.findings());
            assert_eq!(resolver.escalated_solves(), 0);
        }

        /// The lone weak pad is the only source of its node, and what it
        /// reports is the resistor it is.
        #[rstest]
        fn a_weak_pad_alone_pulls_its_net_through_its_own_impedance() {
            behaviour!(Test {
                id: "engine.weak-pad-alone-is-a-pull",
                covers: Some("board/src/engine.rs#project_root"),
                given: "a pad pulling an otherwise unsourced net high through 15 kilohms",
            });
            expect!(
                "pulled-through-pad",
                "the net is pulled high through the pad's 15 kilohms",
                "a pad of a kilohm or more is a resistor to its rail, and the ohms reported are that resistor",
            );
            let mut resolver = Resolver::new(1, Dsu::new(1));
            resolver.add_endpoint(0, PinRef::new("P2", "P28"), thevenin(3.3, 15_000.0));
            let mut net_table = nets(1);
            let mut diags = Diagnostics::new();
            resolver.resolve(&mut net_table, &mut diags, &QuasiStaticMna);
            assert_eq!(net_table[0].state, NetState::Pulled(Level::High, 15_000.0));
            assert!(diags.is_empty(), "{:?}", diags.findings());
        }

        /// A source resting inside the dead band — a rate-mode gate driving
        /// a clock's average, a self-biased stage — has no level to
        /// project: its node reads the voltage, on the root and through a
        /// resistor alike.
        #[rstest]
        #[case::on_the_root(0.0, NetState::Analog(1.65))]
        #[case::through_a_resistor(100_000.0, NetState::Analog(1.65))]
        fn a_source_inside_the_dead_band_projects_its_voltage(
            #[case] series_ohms: Ohms,
            #[case] expected: NetState,
        ) {
            behaviour!(Test {
                id: "engine.mid-band-source-is-analog",
                covers: Some("board/src/engine.rs#project_root"),
                given: "a pin driving 1.65 volts through 29 ohms as the only source of a net, on the net itself or through a 100 kilohm resistor",
            });
            expect!(
                "analog-not-a-level",
                "the net reads 1.65 volts, neither driven nor pulled to a level",
                "a voltage strictly between the low and high input thresholds is not a logic level, so the node reports its open-circuit voltage",
            );
            expect!("nothing-reported", "nothing is reported");
            expect!("zero-solves", "the pass costs no cluster solve");
            let count = if series_ohms > 0.0 { 2 } else { 1 };
            let mut resolver = Resolver::new(count, Dsu::new(count));
            resolver.add_endpoint(0, PinRef::new("U101", "2Y"), thevenin(1.65, 29.0));
            let read = if series_ohms > 0.0 {
                resolver.add_edge(0, 1, series_ohms);
                1
            } else {
                0
            };
            let mut net_table = nets(count);
            let mut diags = Diagnostics::new();
            resolver.resolve(&mut net_table, &mut diags, &QuasiStaticMna);
            assert_eq!(net_table[read].state, expected);
            assert!(diags.is_empty(), "{:?}", diags.findings());
            assert_eq!(resolver.escalated_solves(), 0);
        }

        /// Two drivers within a factor of ten of each other disagreeing:
        /// solved, and the voltage published when it is a valid level.
        #[rstest]
        fn comparable_drivers_disagreeing_solve_to_the_divided_voltage() {
            behaviour!(Test {
                id: "engine.comparable-drivers-solve",
                covers: Some("board/src/engine.rs#project_root"),
                given: "a pin driving a net high at 25 ohms and another driving it low at 100 ohms",
            });
            expect!(
                "divided-voltage",
                "the net reads the divided voltage, 2.64 volts, a valid high",
                "sources within a factor of ten of each other are solved, and a solved voltage outside the dead band is published as the voltage it is",
            );
            expect!(
                "fight-reported",
                "the fight is reported as exactly one finding, a contention naming both pins"
            );
            expect!("one-solve", "the pass costs exactly one cluster solve");
            let mut resolver = Resolver::new(1, Dsu::new(1));
            resolver.add_endpoint(0, PinRef::new("U1", "1"), thevenin(3.3, 25.0));
            resolver.add_endpoint(0, PinRef::new("U2", "1"), thevenin(0.0, 100.0));
            let mut net_table = nets(1);
            let mut diags = Diagnostics::new();
            resolver.resolve(&mut net_table, &mut diags, &QuasiStaticMna);
            // (3.3/25 + 0/100) / (1/25 + 1/100) = 0.132 / 0.05 = 2.64 V.
            assert!(
                matches!(net_table[0].state, NetState::Analog(v) if (v - 2.64).abs() < 1e-9),
                "got {:?}",
                net_table[0].state
            );
            assert_eq!(diags.len(), 1, "{:?}", diags.findings());
            assert!(contention_naming(
                &diags,
                "N0",
                &[PinRef::new("U1", "1"), PinRef::new("U2", "1")]
            ));
            assert_eq!(resolver.escalated_solves(), 1);
        }

        /// Two equal push-pulls meet at mid-rail, inside the dead band.
        #[rstest]
        fn equal_drivers_disagreeing_sit_in_the_dead_band() {
            behaviour!(Test {
                id: "engine.equal-drivers-contend",
                covers: Some("board/src/engine.rs#project_root"),
                given: "two 25 ohm pins driving one net to opposite levels",
            });
            expect!(
                "contention",
                "the net is in contention",
                "the fight solves to 1.65 volts, strictly inside the 0.8 to 2.0 volt dead band, which is neither level",
            );
            expect!(
                "ambiguous-level",
                "the 1.65 volts they fight to is reported as an ambiguous level beside the contention finding that names both pins"
            );
            let mut resolver = Resolver::new(1, Dsu::new(1));
            resolver.add_endpoint(0, PinRef::new("U1", "1"), Some(high()));
            resolver.add_endpoint(0, PinRef::new("U2", "1"), Some(low()));
            let mut net_table = nets(1);
            let mut diags = Diagnostics::new();
            resolver.resolve(&mut net_table, &mut diags, &QuasiStaticMna);
            assert_eq!(net_table[0].state, NetState::Contention);
            assert!(contention_naming(
                &diags,
                "N0",
                &[PinRef::new("U1", "1"), PinRef::new("U2", "1")]
            ));
            assert!(
                diags.findings().iter().any(|f| matches!(
                    f,
                    Finding::AmbiguousLevel { net, volts } if net == "N0" && (volts - 1.65).abs() < 1e-9
                )),
                "{:?}",
                diags.findings()
            );
        }

        /// A driver against an ideal source on its own node reads the ideal
        /// source, and the fight is reported.
        #[rstest]
        fn a_driver_against_an_injected_short_on_its_own_net_reads_the_short() {
            behaviour!(Test {
                id: "engine.driver-vs-short-on-own-net",
                covers: Some("board/src/engine.rs#project_root"),
                given: "a pin driving a net low at 25 ohms, with a short to 3.3 volts injected on that net",
            });
            expect!(
                "reads-the-short",
                "the net reads 3.3 volts",
                "an ideal source outranks any pad, and firmware must see a short to a rail on a pin it drives",
            );
            expect!(
                "fight-reported",
                "the fight is reported as contention naming the driver"
            );
            let mut resolver = Resolver::new(1, Dsu::new(1));
            resolver.add_endpoint(0, PinRef::new("U1", "1"), Some(low()));
            resolver.add_stuck_source(0, 3.3);
            let mut net_table = nets(1);
            let mut diags = Diagnostics::new();
            resolver.resolve(&mut net_table, &mut diags, &QuasiStaticMna);
            assert_eq!(net_table[0].state, NetState::Analog(3.3));
            assert!(
                contention_naming(&diags, "N0", &[PinRef::new("U1", "1")]),
                "{:?}",
                diags.findings()
            );
        }

        /// The flash-versus-card shape on the module's shared P58: the far
        /// driver reaches through a series resistor and lands at ten times
        /// the near one's total ohms.
        #[rstest]
        #[case::near_high_far_low(high(), low(), Level::High, Level::Low)]
        #[case::near_low_far_high(low(), high(), Level::Low, Level::High)]
        fn a_driver_ten_times_further_away_loses_with_a_finding(
            #[case] near: TheveninDrive,
            #[case] far: TheveninDrive,
            #[case] near_level: Level,
            #[case] far_level: Level,
        ) {
            behaviour!(Test {
                id: "engine.ten-times-weaker-loses",
                covers: Some("board/src/engine.rs#project_root"),
                given: "a pin driving a net at 25 ohms, and a second pin driving the opposite level that reaches the net through a 240 ohm series resistor",
            });
            expect!(
                "each-net-reads-its-own-driver",
                "each net is driven to the level of the pin on it",
                "a source at ten times the strongest's total ohms or more loses to it; here each pin reaches the other's net at 265 ohms against 25",
            );
            expect!(
                "fight-reported-on-both",
                "the fight is reported on both nets as contention naming both pins"
            );
            expect!("zero-solves", "the pass costs no cluster solve");
            let mut resolver = Resolver::new(2, Dsu::new(2));
            resolver.add_endpoint(0, PinRef::new("U301", "DO"), Some(near));
            resolver.add_endpoint(1, PinRef::new("J301", "DAT0"), Some(far));
            resolver.add_edge(0, 1, 240.0);
            let mut net_table = nets(2);
            let mut diags = Diagnostics::new();
            resolver.resolve(&mut net_table, &mut diags, &QuasiStaticMna);
            assert_eq!(net_table[0].state, NetState::Driven(near_level));
            assert_eq!(net_table[1].state, NetState::Driven(far_level));
            let both = [PinRef::new("U301", "DO"), PinRef::new("J301", "DAT0")];
            assert!(
                contention_naming(&diags, "N0", &both),
                "{:?}",
                diags.findings()
            );
            assert!(
                contention_naming(&diags, "N1", &both),
                "{:?}",
                diags.findings()
            );
            assert_eq!(resolver.escalated_solves(), 0);
        }

        /// The open-drain bus: a pull-up and any number of sinks.
        #[rstest]
        fn a_wired_and_reads_low_when_any_sink_pulls_and_the_pull_up_otherwise() {
            behaviour!(Test {
                id: "engine.wired-and",
                covers: Some("board/src/engine.rs#project_root"),
                given: "a net pulled to 3.3 volts through 4.7 kilohms with two open-drain outputs on it",
            });
            expect!(
                "pull-up-in-charge",
                "with both outputs released the net is pulled high through the 4.7 kilohms"
            );
            expect!(
                "either-sink-wins",
                "with either output sinking at 25 ohms the net is driven low, with nothing reported",
                "a pull-up of kilohms is a pull and never contends with a sink",
            );
            expect!(
                "both-sinks-agree",
                "with both sinking the net is driven low, with nothing reported"
            );
            let mut resolver = Resolver::new(2, Dsu::new(2));
            resolver.add_power_source(0, 3.3);
            resolver.add_edge(0, 1, 4_700.0);
            let a = resolver.add_endpoint(1, PinRef::new("U1", "SDA"), None);
            let b = resolver.add_endpoint(1, PinRef::new("U2", "SDA"), None);
            let mut net_table = nets(2);
            let mut diags = Diagnostics::new();
            resolver.resolve(&mut net_table, &mut diags, &QuasiStaticMna);
            assert_eq!(net_table[1].state, NetState::Pulled(Level::High, 4_700.0));

            for (sink_a, sink_b) in [(true, false), (false, true), (true, true)] {
                resolver.set_drive(a, sink_a.then(|| Drive::Thevenin(low())));
                resolver.set_drive(b, sink_b.then(|| Drive::Thevenin(low())));
                let mut diags = Diagnostics::new();
                resolver.resolve_dirty(&mut net_table, &mut diags, &QuasiStaticMna);
                assert_eq!(
                    net_table[1].state,
                    NetState::Driven(Level::Low),
                    "sinks ({sink_a}, {sink_b})"
                );
                assert!(diags.is_empty(), "{:?}", diags.findings());
            }
            resolver.set_drive(a, None);
            resolver.set_drive(b, None);
            let mut diags = Diagnostics::new();
            resolver.resolve_dirty(&mut net_table, &mut diags, &QuasiStaticMna);
            assert_eq!(net_table[1].state, NetState::Pulled(Level::High, 4_700.0));
        }

        /// `ohms = ∞` is normalised to released at the slot.
        #[rstest]
        fn an_infinite_impedance_drive_is_a_release() {
            behaviour!(Test {
                id: "engine.infinite-impedance-is-released",
                covers: Some("board/src/engine.rs#normalise_drive"),
                given: "a pin publishing 3.3 volts behind an infinite impedance onto a net another pin drives low at 25 ohms",
            });
            expect!(
                "driven-low",
                "the net is driven low, with nothing reported",
                "a drive behind an infinite impedance is a released pin: it is never ranked against anything",
            );
            expect!("zero-solves", "the pass costs no cluster solve");
            expect!(
                "same-as-release",
                "publishing the infinite drive onto a released pin leaves the drive table as it was, so nothing is re-resolved"
            );
            let calls: Arc<StdMutex<Vec<Vec<NetId>>>> = Arc::new(StdMutex::new(Vec::new()));
            let solver = RecordingSolver {
                calls: Arc::clone(&calls),
            };
            let mut resolver = Resolver::new(1, Dsu::new(1));
            let ghost = resolver.add_endpoint(0, PinRef::new("U1", "1"), None);
            resolver.add_endpoint(0, PinRef::new("U2", "1"), Some(low()));
            assert!(
                !resolver.set_drive(
                    ghost,
                    Some(Drive::Thevenin(TheveninDrive {
                        volts: 3.3,
                        impedance: f64::INFINITY,
                    }))
                ),
                "released to released is no change"
            );
            let mut net_table = nets(1);
            let mut diags = Diagnostics::new();
            resolver.resolve(&mut net_table, &mut diags, &solver);
            assert_eq!(net_table[0].state, NetState::Driven(Level::Low));
            assert!(diags.is_empty(), "{:?}", diags.findings());
            assert!(
                calls.lock().unwrap().is_empty(),
                "the solver must not be called"
            );
            assert_eq!(resolver.escalated_solves(), 0);

            // Driven for real, then released by the infinite form: a change.
            assert!(resolver.set_drive(ghost, Some(Drive::Thevenin(high()))));
            assert!(resolver.set_drive(
                ghost,
                Some(Drive::Thevenin(TheveninDrive {
                    volts: 3.3,
                    impedance: f64::NEG_INFINITY,
                }))
            ));
            let mut diags = Diagnostics::new();
            resolver.resolve_dirty(&mut net_table, &mut diags, &solver);
            assert_eq!(net_table[0].state, NetState::Driven(Level::Low));
        }

        /// A rail no model has put a voltage on (a `PowerOut` awaiting its
        /// regulator model) is still a rail that is there.
        #[rstest]
        fn an_unmodelled_rail_presents_as_up_through_its_path() {
            behaviour!(Test {
                id: "engine.unmodelled-rail-is-up",
                covers: Some("board/src/engine.rs#Resolver::resolve_cluster"),
                given: "a supply pin whose voltage no model declares, reaching an input through a 4.7 kilohm resistor",
            });
            expect!(
                "pulled-high-through-path",
                "the input is pulled high through the 4.7 kilohms",
                "a rail without a declared voltage sources its cluster as up, and the path to it is what the input sees",
            );
            expect!(
                "rail-net-up",
                "the rail's own net reads pulled high through nothing"
            );
            let mut resolver = Resolver::new(2, Dsu::new(2));
            resolver.add_power_source(0, f64::NAN);
            resolver.add_edge(0, 1, 4_700.0);
            resolver.add_digital_sense(1);
            let mut net_table = nets(2);
            let mut diags = Diagnostics::new();
            resolver.resolve(&mut net_table, &mut diags, &QuasiStaticMna);
            assert_eq!(net_table[1].state, NetState::Pulled(Level::High, 4_700.0));
            assert_eq!(net_table[0].state, NetState::Pulled(Level::High, 0.0));
            assert!(diags.is_empty(), "{:?}", diags.findings());
        }

        /// A current injection has no projection form: its cluster is solved
        /// and the node sits at `I · R` above the terminal.
        #[rstest]
        #[case::source_1ma(1e-3, 1.0)]
        #[case::sink_100ua(-100e-6, -0.1)]
        fn a_current_injection_into_a_resistor_to_a_terminal_reads_i_times_r(
            #[case] amps: f64,
            #[case] expect: Volts,
        ) {
            behaviour!(Test {
                id: "engine.current-injection-reads-i-times-r",
                covers: Some("board/src/engine.rs#Resolver::resolve_cluster"),
                given: "a pin injecting a current into a net tied to ground through 1 kilohm",
            });
            expect!(
                "i-times-r",
                "the net reads the injected current times the kilohm above ground",
                "an injected current has no level of its own, so its cluster is solved and the net is the operating point",
            );
            expect!("one-solve", "the pass costs exactly one cluster solve");
            expect!("ground-holds", "the ground net stays at 0 volts");
            let mut resolver = Resolver::new(2, Dsu::new(2));
            resolver.add_power_source(0, 0.0);
            resolver.add_edge(0, 1, 1_000.0);
            resolver.add_endpoint_with(1, PinRef::new("IC6", "K"), Some(Drive::Current { amps }));
            let mut net_table = nets(2);
            let mut diags = Diagnostics::new();
            resolver.resolve(&mut net_table, &mut diags, &QuasiStaticMna);
            assert!(
                matches!(net_table[1].state, NetState::Analog(v) if (v - expect).abs() < 1e-6),
                "got {:?}",
                net_table[1].state
            );
            assert!(
                matches!(net_table[0].state, NetState::Analog(v) if v.abs() < 1e-6),
                "got {:?}",
                net_table[0].state
            );
            assert_eq!(resolver.escalated_solves(), 1);
            assert!(diags.is_empty(), "{:?}", diags.findings());
        }

        /// A current source reaches nothing by itself.
        #[rstest]
        fn a_current_injected_where_nothing_reaches_leaves_the_net_floating() {
            behaviour!(Test {
                id: "engine.current-into-floating-net",
                covers: Some("board/src/engine.rs#Resolver::resolve_cluster"),
                given: "a pin injecting 1 milliampere into a net no rail or driver reaches, with a digital input on that net",
            });
            expect!(
                "floating",
                "the net floats",
                "a current source has no open-circuit voltage: with no return path the injection goes nowhere",
            );
            expect!(
                "injection-reported",
                "the stranded injection is reported, naming the net and the injecting pin"
            );
            expect!(
                "input-reported",
                "the input on the net is reported as floating"
            );
            expect!("zero-solves", "the pass costs no cluster solve");
            let mut resolver = Resolver::new(1, Dsu::new(1));
            resolver.add_endpoint_with(
                0,
                PinRef::new("IC6", "K"),
                Some(Drive::Current { amps: 1e-3 }),
            );
            resolver.add_digital_sense(0);
            let mut net_table = nets(1);
            let mut diags = Diagnostics::new();
            resolver.resolve(&mut net_table, &mut diags, &QuasiStaticMna);
            assert_eq!(net_table[0].state, NetState::Floating);
            assert!(diags.contains(&Finding::CurrentIntoFloatingNode {
                net: "N0".to_string(),
                pin: PinRef::new("IC6", "K"),
            }));
            assert!(diags.contains(&Finding::FloatingSense {
                net: "N0".to_string(),
                kind: SenseKind::Digital,
            }));
            assert_eq!(resolver.escalated_solves(), 0);
        }

        /// The injection travels the same queue as every other drive.
        #[rstest]
        fn a_current_drive_reaches_the_live_engine_like_any_other() {
            behaviour!(Test {
                id: "engine.current-drive-live",
                covers: Some("board/src/component.rs#PinHandle::drive"),
                given: "a live engine with a net tied to ground through 1 kilohm, and a pin on that net publishing a 1 milliampere injection",
            });
            expect!("i-times-r-live", "the net's published state becomes 1 volt");
            expect!(
                "release-restores",
                "releasing the pin returns the net to ground's pull through the kilohm"
            );
            let mut resolver = Resolver::new(2, Dsu::new(2));
            resolver.add_power_source(0, 0.0);
            resolver.add_edge(0, 1, 1_000.0);
            let endpoint = resolver.add_endpoint(1, PinRef::new("IC6", "K"), None);
            let handle = EngineHandle::spawn(
                resolver,
                nets(2),
                Box::new(QuasiStaticMna),
                EventLog::disabled(),
                None,
            );
            let pin =
                crate::component::PinHandle::wired(NetId(1), Some(endpoint), None, handle.link());
            pin.drive(Drive::Current { amps: 1e-3 });
            assert!(
                wait_for(
                    || matches!(handle.net_state(NetId(1)), Some(NetState::Analog(v)) if (v - 1.0).abs() < 1e-6),
                    Duration::from_secs(5)
                ),
                "got {:?}",
                handle.net_state(NetId(1))
            );
            pin.release();
            assert!(
                wait_for(
                    || handle.net_state(NetId(1)) == Some(NetState::Pulled(Level::Low, 1_000.0)),
                    Duration::from_secs(5)
                ),
                "got {:?}",
                handle.net_state(NetId(1))
            );
        }

        /// A declared terminal holds its node: what reaches it stops there.
        /// The module's shape — a pad driven high through the P59 pull-down
        /// to ground, and the core rail's feedback divider on the other side
        /// of ground.
        #[rstest]
        fn a_source_does_not_reach_past_a_terminal() {
            behaviour!(Test {
                id: "engine.terminal-is-a-path-barrier",
                covers: Some("board/src/engine.rs#min_path_ohms"),
                given: "a pad driving high at 25 ohms into a 10.5 kilohm resistor to ground, with a second net hanging off ground through another 10.5 kilohms",
            });
            expect!(
                "far-net-reads-ground",
                "the net beyond ground is pulled low through its own 10.5 kilohms",
                "a declared terminal is a boundary: a path may end at ground but never continue past it, so the pad is no source of the far net",
            );
            expect!("pad-net-driven", "the pad's own net is driven high");
            expect!("zero-solves", "the pass costs no cluster solve");
            // pad(net0) —10.5k— GND(net1, stuck 0 V) —10.5k— far(net2)
            let mut resolver = Resolver::new(3, Dsu::new(3));
            resolver.add_endpoint(0, PinRef::new("U100", "P59"), Some(high()));
            resolver.add_edge(0, 1, 10_500.0);
            resolver.add_stuck_source(1, 0.0);
            resolver.add_edge(1, 2, 10_500.0);
            resolver.add_digital_sense(2);
            let mut net_table = nets(3);
            let mut diags = Diagnostics::new();
            resolver.resolve(&mut net_table, &mut diags, &QuasiStaticMna);
            assert_eq!(net_table[0].state, NetState::Driven(Level::High));
            assert_eq!(net_table[1].state, NetState::Analog(0.0));
            assert_eq!(net_table[2].state, NetState::Pulled(Level::Low, 10_500.0));
            assert_eq!(resolver.escalated_solves(), 0);
            assert!(diags.is_empty(), "{:?}", diags.findings());
        }

        /// A cluster an analog sense asked to solve publishes its operating
        /// point on every root; the two analog goldens
        /// (`nominal_analog_cluster`, `net_stuck_shared_node`) pin the same
        /// rule on the wire.
        #[rstest]
        fn an_analog_sense_reads_the_operating_point_of_a_fought_node() {
            behaviour!(Test {
                id: "engine.analog-sense-reads-fought-node",
                covers: Some("board/src/engine.rs#Resolver::resolve_cluster"),
                given: "two pins driving one net to opposite levels at 25 ohms, with an analog input on that net",
            });
            expect!(
                "voltage-delivered",
                "the net publishes the solved mid-rail voltage, 1.65 volts, and nothing is reported for it",
                "an analog reader is handed the operating point of its cluster, whatever the fight on it; a contention state would hand it nothing",
            );
            let mut resolver = Resolver::new(1, Dsu::new(1));
            resolver.add_endpoint(0, PinRef::new("U1", "1"), Some(high()));
            resolver.add_endpoint(0, PinRef::new("U2", "1"), Some(low()));
            resolver.add_analog_sense(0);
            let mut net_table = nets(1);
            let mut diags = Diagnostics::new();
            resolver.resolve(&mut net_table, &mut diags, &QuasiStaticMna);
            assert!(
                matches!(net_table[0].state, NetState::Analog(v) if (v - 1.65).abs() < 1e-9),
                "got {:?}",
                net_table[0].state
            );
            assert!(diags.is_empty(), "{:?}", diags.findings());
        }
    }
}
