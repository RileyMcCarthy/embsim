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
//! **A step clock is a drive.** A [`Drive::Periodic`] — two Thevenin phases
//! and the integer segment that alternates them — sits in the slot table
//! like any other drive and resolves through rule 2 **phase by phase**: a
//! cluster a periodic drive sits in is projected (and, where comparable
//! sources disagree, solved) once with every periodic source at its high
//! port and once at its low port, and each root combines the two
//! (`combine_phases`) into [`NetState::Periodic`] with each phase's level. A
//! comparable static source, or a second periodic source, is
//! [`NetState::Contention`] — a sustained fight for half of every cycle.
//! Across a **coupling capacitor** the rate still crosses, by the AC rule of
//! [`COUPLING_REACTANCE_RATIO`] (`overlay_arrivals`): the far root carries
//! the source's segment and phase levels, a declared terminal is a barrier,
//! a fought far node clamps the crossing, and a crossing that fails is
//! [`Finding::PeriodicNotCoupled`].
//!
//! There used to be a **pulse channel** beside the drives — its own pin
//! roles, write handle, subscription, routing pass and delivery — and before
//! it a **byte** route carrying UART traffic. Both are gone. The net decided
//! who was connected and then the payload went around the resolution, so a
//! step line fought by a stuck driver showed nothing and a byte could not
//! notice a floating line. Bytes are framed onto the net as levels by
//! [`crate::SerialLevelBridge`]; a rate is the one encoding kept — because
//! 820 000 edges a second was measured (`DESIGN.md` §6) — and it is an
//! encoding of [`Drive`], not a channel.
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
//! 1. **Dense index** — walk the `Vec` (`self.slots`, `self.periodic_slots`,
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
use crate::component::{Drive, PinHandle, PwlCurve, RegionTest};
use crate::diagnostics::{CallbackKind, Diagnostics, Finding, SenseKind};
use crate::event_log::{EngineEvent, EventLog};
use crate::net::{
    level_of, Amps, Level, Net, NetId, NetState, NetVolts, Ohms, PeriodicSchedule, PinRef,
    TheveninDrive, Volts, COUPLED_REACH_OHMS, COUPLING_REACTANCE_RATIO, ESCALATION_IMPEDANCE_RATIO,
    V_IH, V_IL, WEAK_DRIVE_OHMS,
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
/// lock held, with the net's resolution as the pass published it.
pub(crate) type SenseCallback = Box<dyn Fn(&Delivery) + Send>;

/// What the engine hands a sense subscription at a delivery: the net's
/// state, the voltage behind it, and — for a pin measured against a
/// reference on another net — the reference's, all from the pass being
/// delivered. The subscription measures its pin's [`crate::Sense`] from
/// it (`PinHandle::measure`); an instrument reads the state alone.
#[derive(Debug, Clone, Copy)]
pub(crate) struct Delivery {
    pub(crate) state: NetState,
    pub(crate) node: NetVolts,
    pub(crate) reference: Option<NetVolts>,
}

/// Timer-wheel wakeup callback: called from the engine thread with the
/// sampled virtual time (µs); no engine lock held.
pub(crate) type WakeCallback = Box<dyn Fn(u64) + Send>;

/// Topology-change callback (the topology seam): called from the engine
/// thread with the new topology epoch; no engine lock held.
pub(crate) type TopologyCallback = Box<dyn Fn(u64) + Send>;

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
    /// before any traffic), then on every change of the net — its state, or
    /// the voltage it is at — of its `reference`'s, and of its `supply`'s:
    /// a pin's [`crate::Sense`] is measured against its reference, so a
    /// reference that moves moves what the pin is handed, and a relative
    /// threshold scales with its supply, so a supply that moves moves the
    /// level the receiver projects the same voltage to.
    RegisterSense {
        /// Net to observe.
        net: NetId,
        /// The net the subscribing pin's sense is measured against, when it
        /// declares a reference on another net.
        reference: Option<NetId>,
        /// The net of the supply the subscribing pin's thresholds are
        /// relative to, when it declares one on a third net
        /// ([`crate::PinHandle::thresholds`]).
        supply: Option<NetId>,
        /// Delivery callback.
        callback: SenseCallback,
    },
    /// A subscription declares `net` **read**: as a digital sense, by a
    /// released bidirectional pad
    /// ([`crate::PinDecl::reads_when_subscribed`], an input until its owner
    /// drives it) ahead of
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
    /// Subscribe to net-graph topology changes (the topology seam). The
    /// current epoch is delivered once at registration.
    RegisterTopologyObserver {
        /// Notification callback.
        callback: TopologyCallback,
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
pub(crate) type RecordedDriveLog = Arc<Mutex<Vec<(EndpointId, Option<Drive>)>>>;

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
    /// analog sense is declared by its pin's declarations at build, never
    /// live: [`crate::PinDecl::senses_at_build`].)
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
    /// The net the subscribing pin's sense is measured against, when it
    /// declares a reference on another net: its moves re-deliver.
    pub(crate) reference: Option<NetId>,
    /// The net of the supply the pin's thresholds are relative to, when it
    /// declares one on a third net: its moves re-deliver too.
    pub(crate) supply: Option<NetId>,
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

/// The two tables a pass publishes, the net states and the voltages behind
/// them ([`EngineLink::states`], [`EngineLink::volts`]).
pub(crate) type PublishedTables = (Arc<Mutex<Vec<NetState>>>, Arc<VoltsTable>);

/// The voltage behind every net, as the last pass published it, read
/// without a lock: three atomic words a net — the node's voltage and a
/// periodic node's two phases — a `None` stored as [`VoltsTable::NONE`]
/// and a node with no phases as [`VoltsTable::NO_PHASES`] in its high
/// word. The engine thread is its one writer and every delivery reads it
/// between writes, so a read inside a callback is one pass's. A read from
/// another thread (a pin's own [`crate::PinHandle::sense`],
/// [`crate::PinHandle::thresholds`]) is **not** one snapshot: each word is
/// as some pass published it, the three words of one net may come from two
/// passes, and the state table beside them is locked and published apart —
/// a periodic state may be read with the next pass's DC voltage. No model
/// reads off the engine thread (every `DigitalReceiver::read` and
/// `PinHandle::level` in the tree runs in an `on_sense` callback); a
/// per-net generation word is the fix if one ever must.
#[derive(Debug, Default)]
pub(crate) struct VoltsTable {
    cells: Vec<[AtomicU64; 3]>,
}

impl VoltsTable {
    /// `None`: the canonical quiet NaN's bits, which no voltage the
    /// resolver publishes is (it publishes finite voltages only).
    const NONE: u64 = 0x7ff8_0000_0000_0000;
    /// A node with no phases: a NaN with a payload no arithmetic produces.
    const NO_PHASES: u64 = 0x7ff8_0000_0000_0dc0;

    /// A table holding `volts` for each net in order.
    pub(crate) fn of(volts: impl IntoIterator<Item = NetVolts>) -> Self {
        Self {
            cells: volts
                .into_iter()
                .map(|v| {
                    let [a, b, c] = Self::words(v);
                    [AtomicU64::new(a), AtomicU64::new(b), AtomicU64::new(c)]
                })
                .collect(),
        }
    }

    fn word(volts: Option<Volts>) -> u64 {
        volts.map_or(Self::NONE, f64::to_bits)
    }

    fn volts(word: u64) -> Option<Volts> {
        if word == Self::NONE {
            None
        } else {
            Some(f64::from_bits(word))
        }
    }

    fn words(volts: NetVolts) -> [u64; 3] {
        match volts.phases {
            None => [Self::word(volts.dc), Self::NO_PHASES, Self::NONE],
            Some((hi, lo)) => [Self::word(volts.dc), Self::word(hi), Self::word(lo)],
        }
    }

    /// Publish net `net`'s voltage (the engine thread only).
    pub(crate) fn store(&self, net: usize, volts: NetVolts) {
        if let Some(cell) = self.cells.get(net) {
            for (slot, word) in cell.iter().zip(Self::words(volts)) {
                slot.store(word, Ordering::Relaxed);
            }
        }
    }

    /// Net `net`'s voltage as last published; a net the table does not
    /// hold names none.
    pub(crate) fn load(&self, net: usize) -> NetVolts {
        let Some([dc, hi, lo]) = self.cells.get(net) else {
            return NetVolts::default();
        };
        let dc = Self::volts(dc.load(Ordering::Relaxed));
        match hi.load(Ordering::Relaxed) {
            Self::NO_PHASES => NetVolts::dc(dc),
            hi => NetVolts {
                dc,
                phases: Some((Self::volts(hi), Self::volts(lo.load(Ordering::Relaxed)))),
            },
        }
    }

    /// Net `net`'s node voltage alone, as last published.
    pub(crate) fn dc(&self, net: usize) -> Option<Volts> {
        match self.cells.get(net) {
            Some([dc, _, _]) => Self::volts(dc.load(Ordering::Relaxed)),
            None => None,
        }
    }
}

/// Cloneable client half of the engine: command sender, the global drive
/// sequence counter, and the engine-published net-state table.
///
/// An **inert** link (`tx == None`) is what the build-time analysis path
/// hands out: senses read the build-resolved snapshot, schedules are traced
/// and dropped, and drives are *recorded* so the build pass can apply a
/// component's idle drive before it publishes findings (a component that
/// releases a push-pull output at attach must not be analyzed as if it were
/// driving its declared idle-high).
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
    /// Engine-published voltage per net, beside [`Self::states`] and
    /// written under the same pass — what a sense is handed
    /// ([`crate::Sense`]; build snapshot when inert).
    pub(crate) volts: Arc<VoltsTable>,
    /// Engine-published currents ([`CurrentTable`]; build snapshot when
    /// inert).
    pub(crate) currents: Arc<Mutex<CurrentTable>>,
    /// Inert path only: drives issued during attach, in issue order, for the
    /// build pass to apply before it resolves for real.
    pub(crate) recorded_drives: Option<RecordedDriveLog>,
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
        (states, volts): PublishedTables,
        currents: Arc<Mutex<CurrentTable>>,
        recorded_drives: RecordedDriveLog,
        recorded_senses: &SenseLog,
    ) -> Self {
        Self {
            tx: None,
            control_tx: None,
            drive_seq: Arc::new(AtomicU64::new(0)),
            pending_schedules: Arc::new(AtomicUsize::new(0)),
            states,
            volts,
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

/// A declared terminal source: a harness supply or a `PowerOut` pin
/// ([`Resolver::add_power_source`], [`Resolver::add_terminal_endpoint`]),
/// or a `net_stuck` fault ([`Resolver::add_stuck_source`]), by position in
/// its list. The canonical order of the sources is every power source,
/// then every stuck fault, each in declaration order.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub(crate) enum TerminalId {
    Power(usize),
    Stuck(usize),
}

/// What a declared terminal source holds its net at (`NODES.md` "Three
/// rules the taxonomy rests on", 1). A terminal is declared once, at build
/// — a `PowerOut` pin, a harness `power(V)` endpoint, a `net_stuck` — and
/// is a cluster boundary whatever it holds; *what* it holds is the one
/// thing about it that moves at run time.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum TerminalDrive {
    /// Nothing: the terminal sources no voltage — a rail that is down, a
    /// `PowerOut` its part released. Its node floats and nothing reaches
    /// its dependents through it; a bench strap on the same net sources
    /// it without a fight.
    Released,
    /// Sourced at a voltage no model declares (the `PowerOut` of a part
    /// that is still a facade, `f64::NAN` in the source table): clears
    /// `PowerNetUnsourced` and presents as up through the path to it,
    /// ranks nowhere.
    Unmodelled,
    /// Held at a voltage.
    Volts(Volts),
}

impl TerminalDrive {
    /// The source-table encoding: a NaN voltage is an unmodelled rail.
    fn from_volts(volts: Volts) -> Self {
        if volts.is_nan() {
            Self::Unmodelled
        } else {
            Self::Volts(volts)
        }
    }

    /// The published drive of a `PowerOut` pin's slot, as the terminal it
    /// holds: released, a current injection or a periodic drive is
    /// released (a current into a terminal is not a rail, and a rail is not
    /// a clock), a Thevenin drive is its open-circuit voltage (NaN =
    /// unmodelled), the impedance recorded on the slot for the I-V port and
    /// not solved (`NODES.md` §2, the Regulator row).
    fn from_slot(drive: Option<Drive>) -> Self {
        match drive {
            Some(Drive::Thevenin(t)) => Self::from_volts(t.volts),
            Some(Drive::Current { .. }) | Some(Drive::Periodic { .. }) | None => Self::Released,
        }
    }

    /// The slot drive that holds this terminal state at 0 Ω — released is
    /// `None`, unmodelled the NaN-volt Thevenin a `PowerOut` has always
    /// meant. A `PowerOut` pin's declared idle
    /// ([`crate::PinDecl::idle`]) passes its own drive, impedance included,
    /// instead.
    pub(crate) fn idle_slot_drive(self) -> Option<Drive> {
        match self {
            Self::Released => None,
            Self::Unmodelled => Some(Drive::Thevenin(TheveninDrive {
                volts: f64::NAN,
                impedance: 0.0,
            })),
            Self::Volts(volts) => Some(Drive::Thevenin(TheveninDrive {
                volts,
                impedance: 0.0,
            })),
        }
    }
}

/// One declared terminal source: the net it is declared on and what it
/// holds. A `PowerOut` pin's source is written through its drive slot
/// ([`DriveSlot::terminal`] points at it); a harness supply's and a
/// `net_stuck`'s through [`Resolver::set_terminal`].
struct TerminalSource {
    net: usize,
    drive: TerminalDrive,
}

/// What a terminal's root is held at once its sources are reconciled: the
/// one voltage every dependent solve takes as its constant and every
/// dependent root ranks as an ideal source — assigned once, at the
/// terminal's own cluster, never per dependent.
#[derive(Debug, Clone, Copy, PartialEq)]
struct TerminalState {
    drive: TerminalDrive,
    /// Two of its sources disagreed: the voltage is the fight's own
    /// operating point (the Norton mid-value at the ideal floor) and the
    /// terminal's cluster reports the fight, once.
    fought: bool,
}

/// One drive-capable pin's slot: net membership plus the drive it currently
/// contributes (`None` = released / high-Z / pure sense). Always holds the
/// normalised form ([`normalise_drive`]): a Thevenin drive here has a finite
/// impedance, a current injection a finite value; a periodic drive keeps
/// both its phases as published, a phase behind a non-finite impedance
/// sourcing nothing in that phase ([`phase_port`]).
struct DriveSlot {
    net: usize,
    pin: PinRef,
    drive: Option<Drive>,
    /// A `PowerOut` pin's slot: its drive is what the terminal holds, never
    /// a source of the cluster it sits in (a rail is a constant, not a
    /// driver), and it carries no current the solve accounts for.
    terminal: Option<TerminalId>,
    /// A pin's declared input port ([`crate::InputPort`]): a permanent
    /// source no drive changes — the pin's own load, never a driver.
    port: bool,
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

/// What one pass produces beside the net states: its findings and its
/// currents, accumulated cluster by cluster.
#[derive(Default)]
struct PassOutput {
    findings: PassFindings,
    currents: PassCurrents,
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
    /// it is — large). See [`Resolver::ensure_reach`] for why an estimate.
    pub(crate) far_ohms: f64,
}

/// One root a slot's rate reaches across one or more coupling capacitors —
/// the slot's **AC reach**, a function of the topology alone
/// ([`Resolver::ensure_reach`]): the root, and the capacitors on the
/// cheapest path to it, source side first.
#[derive(Debug, Clone, PartialEq)]
struct CoupledRoot {
    /// The identity root reached.
    root: usize,
    /// The capacitors crossed, source side first.
    crossings: Vec<CouplingCrossing>,
}

/// A periodic drive's rate arriving at a root across coupling capacitors:
/// the root, the slot it comes from, the crossings the AC rule judges, the
/// levels its two phases project to at the source ([`level_of_volts`] — the
/// swing a capacitor passes undivided, not the DC bias it blocks) with the
/// two port voltages they project from, and its segment.
#[derive(Debug, Clone, PartialEq)]
struct Arrival {
    root: usize,
    slot: usize,
    crossings: Vec<CouplingCrossing>,
    hi: Level,
    lo: Level,
    /// The source's high and low port voltages: the swing a sensing pin on
    /// the far node is handed (`NetVolts::phases`). A port's voltage is the
    /// swing whatever its impedance — across a capacitor only the AC
    /// component crosses, so a port released at DC still names one (the
    /// TCXO's clipped sine, `V_pp` in its high port, `NODES.md` §10) — and
    /// a voltage that is not finite names none ([`Arrival::swing_of`]).
    swing: (Option<Volts>, Option<Volts>),
    segment: PeriodicSchedule,
}

impl Arrival {
    /// A periodic drive's two port voltages as the swing a coupling
    /// capacitor passes: each port's open-circuit voltage, whatever its
    /// impedance, and none for a port whose voltage is not a number (an
    /// unmodelled rail's NaN) — nothing is invented for it (`DESIGN.md`
    /// rule 6).
    fn swing_of(hi: TheveninDrive, lo: TheveninDrive) -> (Option<Volts>, Option<Volts>) {
        let finite = |volts: Volts| volts.is_finite().then_some(volts);
        (finite(hi.volts), finite(lo.volts))
    }
}

/// The lists one cluster's pass fills — its sources in slot order, the
/// sources reaching one root, every root's state and voltage — kept on the
/// resolver between passes so a pass over a cluster no periodic drive sits
/// in allocates none of them: every edge the ROM boot resolves is such a
/// pass, and an allocation there is paid per edge (`NODES.md` §12 item 5,
/// the review's edge cost). Taken whole at the start of
/// [`Resolver::resolve_cluster`] and put back at its end; nothing else
/// reads it.
#[derive(Debug, Default)]
struct ClusterScratch {
    sources: Vec<ClusterSource>,
    source_slots: Vec<usize>,
    reaching: Vec<ReachingSource>,
    root_states: Vec<NetState>,
    root_volts: Vec<NetVolts>,
}

/// Resolution state shared by the build-time pass and the live engine:
/// topology (identity merges, conduction edges, static sources, senses) plus
/// the per-endpoint drive table the live path mutates. `resolve` recomputes
/// every net's [`NetState`] from the current table — one code path, so
/// build-time analysis and live resolution cannot fork semantics.
pub(crate) struct Resolver {
    /// Union-find of net *identity* merges (harness wires, pin shorts).
    identity: Dsu,
    /// The per-cluster lists a pass fills and empties, kept between passes
    /// at their capacity ([`ClusterScratch`]).
    scratch: std::cell::RefCell<ClusterScratch>,
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
    /// The declared terminal sources: harness supplies and `PowerOut`
    /// pins, in declaration order ([`TerminalId::Power`]).
    power_sources: Vec<TerminalSource>,
    /// The `net_stuck` faults, in declaration order ([`TerminalId::Stuck`]).
    /// The canonical order of the terminal sources is these after the
    /// power sources.
    stuck_sources: Vec<TerminalSource>,
    digital_senses: Vec<usize>,
    analog_senses: Vec<usize>,
    /// Current instruments' nets ([`ReadKind::Instrument`]): each
    /// escalates its cluster to a solve and is otherwise invisible — no
    /// floating-sense finding.
    current_instruments: Vec<usize>,
    power_senses: Vec<usize>,
    /// Whether a pass since the last publication stored a current that
    /// differs from the one it replaced — the only time the shared table
    /// is worth copying (`DESIGN.md` rule 8: a boot that never solves
    /// publishes no current table).
    currents_changed: bool,
    /// The slots holding a [`Drive::Periodic`], ascending — the sources
    /// whose AC reach the coupling rule walks. Empty on a board with no
    /// clock, which keeps the coupling rule off the fast path
    /// (`DESIGN.md` rule 8).
    periodic_slots: Vec<usize>,
    /// The AC reach of each slot a periodic drive has sat in, keyed by
    /// slot, with the topology version it was walked against
    /// ([`Resolver::ensure_reach`]). hash-order: keyed access only.
    coupled_cache: HashMap<usize, (u64, Vec<CoupledRoot>)>,
    /// The clusters any periodic slot's AC reach touches, ascending, with
    /// the topology version they were gathered against; `None` whenever the
    /// set of periodic slots changed. A dirty pass none of whose clusters is
    /// in it has no arrival to gather — every edge of a boot on a board with
    /// a clock — and pays one scan of its dirty list for that.
    reach_clusters: Option<(u64, Vec<usize>)>,
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
    /// What every terminal of the cached topology holds its root at, by
    /// position in [`Topology::terminals`] — decided by the pass that
    /// resolved the terminal's own cluster and read by every dependent's
    /// pass after it (`NODES.md` "Three rules the taxonomy rests on", 1:
    /// "its state is assigned once"). Rebuilt by the first full pass over a
    /// topology.
    terminal_states: Vec<TerminalState>,
    /// Whether `terminal_states` describes the cached topology: false from
    /// a rebuild until a full pass has decided every terminal, so a dirty
    /// pass never reads a terminal no pass has resolved.
    terminals_resolved: bool,
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
    /// The declared terminals, in ascending root order — one per root a
    /// terminal source is declared on, however many sources share it.
    terminals: Vec<TerminalTopo>,
    /// The terminal roots, ascending — the barriers of every path walk,
    /// the AC reach's included.
    terminal_roots: Vec<usize>,
    /// Identity-collapsed conduction edges, self-loops dropped, in
    /// declaration order: the walk a periodic drive's AC reach relaxes
    /// over.
    root_edges: Vec<(usize, usize, f64)>,
    /// Coupling capacitors between distinct roots as `(a, b, index)`, in
    /// declaration order (one across a single root couples nothing).
    root_couplings: Vec<(usize, usize, usize)>,
    /// The far-node resistance estimate per root: the smallest conduction
    /// edge touching it. hash-order: keyed access only.
    smallest_edge_at: HashMap<usize, f64>,
}

/// One declared terminal (`NODES.md` "Three rules the taxonomy rests on",
/// 1): a root held by a declared source — a `PowerOut` pin, a harness
/// `power(V)` endpoint, a `net_stuck` — which is its own one-root cluster
/// and a boundary of every cluster around it. Membership is fixed at build:
/// the terminal is declared whatever its sources hold, so a rail that is
/// down is a released terminal, not a member of its load's cluster.
struct TerminalTopo {
    /// Its identity root.
    root: usize,
    /// The dense id of its own cluster — exactly one root, this one.
    cluster: usize,
    /// The sources declared on it, in canonical order (power sources then
    /// stuck faults, each in declaration order): what
    /// [`decide_terminal`] reconciles into the one voltage it holds.
    sources: Vec<TerminalId>,
    /// The fan-out: the dense ids of every cluster that reads the terminal
    /// — through a conduction edge ending on it, an element's conducting
    /// end on it, or an element's control on it — ascending. A change to
    /// what the terminal holds dirties these with the terminal's own
    /// cluster ([`Resolver::mark_terminal_dirty`]), which is what keeps
    /// `resolve_dirty` equal to a full pass.
    dependents: Vec<usize>,
}

/// One conduction cluster's members, in the orders the pass iterates them.
struct ClusterTopo {
    /// Member nets, ascending.
    nets: Vec<usize>,
    /// Member identity roots (nets that are their own root), ascending.
    roots: Vec<usize>,
    /// Identity-collapsed conduction edges with a member root at one end
    /// or both, in declaration order. The other end of an edge may be a
    /// boundary terminal's root: the edge belongs to the cluster of its
    /// non-terminal end, and an edge between two terminals belongs to no
    /// cluster (it sits between two constants and changes nothing).
    edges: Vec<(usize, usize, f64)>,
    /// Drive-capable endpoints in the cluster, ascending — the slots that
    /// source it. A `PowerOut` pin's slot is not among them: what it holds
    /// is its terminal's, read through `terminal`/`boundary`.
    slots: Vec<usize>,
    /// Piecewise-linear elements in the cluster, in declaration order.
    elements: Vec<usize>,
    /// `Some(position in Topology::terminals)` when the cluster is a
    /// declared terminal's own: exactly one root, the terminal's.
    terminal: Option<usize>,
    /// The terminals (positions in [`Topology::terminals`], ascending) the
    /// cluster's edges end on and its elements' conducting ends name:
    /// each enters the cluster's solve as a Dirichlet constant and its
    /// ranking as an ideal source through the path to it, and is never a
    /// member. A rail behind a diode is a rail: a boundary terminal sources
    /// the cluster.
    boundary: Vec<usize>,
    /// The terminals the cluster's elements' **controls alone** read (no
    /// edge or conducting end touches them): constants for the region
    /// tests, sourcing nothing and ranking nowhere.
    boundary_controls: Vec<usize>,
    /// Digital sense pins in the cluster as `(registration position, net)`.
    digital_senses: Vec<(usize, usize)>,
    /// Analog sense pins in the cluster as `(registration position, net)`.
    analog_senses: Vec<(usize, usize)>,
    /// Current instruments in the cluster as `(registration position, net)`.
    current_instruments: Vec<(usize, usize)>,
    /// Power sense pins in the cluster as `(registration position, net)`.
    power_senses: Vec<(usize, usize)>,
    /// Minimum series resistance from every root (rows, by position in
    /// `roots`) to every root and then to every boundary terminal
    /// (columns: `roots`, then `boundary`, `roots.len() + boundary.len()`
    /// wide), ending at but never crossing a terminal; `INFINITY` where no
    /// path exists.
    dist: Vec<f64>,
}

impl ClusterTopo {
    /// Width of one row of `dist`.
    fn columns(&self) -> usize {
        self.roots.len() + self.boundary.len()
    }
}

/// Rule 1's "assigned once": what a terminal holds its root at, from its
/// declared sources alone — shared by the per-cluster pass and the
/// test-only reference, so the rule is one function.
///
/// A released source holds nothing; an unmodelled one (a `PowerOut` still
/// a facade) makes the terminal unmodelled when no voltage is declared on
/// it; every declared voltage counts, and when they agree (`==`, so `0.0`
/// and `-0.0` are one voltage) the terminal holds it. A bench strap onto a
/// released or unmodelled rail therefore sources it with no fight. When
/// declared voltages disagree — a rail against a `net_stuck` — the
/// terminal is **fought**: `solve_fight` is handed the voltages in
/// canonical order and answers with the fight's operating point (the
/// solver's Norton mid-value at the ideal floor), which the terminal then
/// holds for every dependent while its own cluster reports the fight once.
fn decide_terminal(
    drives: impl Iterator<Item = TerminalDrive>,
    solve_fight: impl FnOnce(&[Volts]) -> Volts,
) -> TerminalState {
    let mut numeric: Vec<Volts> = Vec::new();
    let mut unmodelled = false;
    for drive in drives {
        match drive {
            TerminalDrive::Released => {}
            TerminalDrive::Unmodelled => unmodelled = true,
            TerminalDrive::Volts(v) => numeric.push(v),
        }
    }
    let Some(&first) = numeric.first() else {
        return TerminalState {
            drive: if unmodelled {
                TerminalDrive::Unmodelled
            } else {
                TerminalDrive::Released
            },
            fought: false,
        };
    };
    if numeric.iter().all(|&v| v == first) {
        return TerminalState {
            drive: TerminalDrive::Volts(first),
            fought: false,
        };
    }
    TerminalState {
        drive: TerminalDrive::from_volts(solve_fight(&numeric)),
        fought: true,
    }
}

/// The terminal ideal sources of one cluster's ranking: the volts each
/// holds and the `dist` column it is reached through.
struct TerminalSourceColumn {
    volts: Volts,
    column: usize,
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
    /// Rates a coupling capacitor refused, by the far root.
    coupling: Vec<(usize, Finding)>,
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
        self.coupling.sort_by_key(|(key, _)| *key);
        let coupling = self.coupling.into_iter().map(|(_, f)| f);
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
            .chain(coupling)
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
        (Some(Drive::Thevenin(x)), Some(Drive::Thevenin(y))) => same_thevenin(x, y),
        (Some(Drive::Current { amps: x }), Some(Drive::Current { amps: y })) => {
            x.total_cmp(y).is_eq()
        }
        (
            Some(Drive::Periodic {
                hi: xh,
                lo: xl,
                segment: xs,
            }),
            Some(Drive::Periodic {
                hi: yh,
                lo: yl,
                segment: ys,
            }),
        ) => same_thevenin(xh, yh) && same_thevenin(xl, yl) && xs == ys,
        _ => false,
    }
}

/// Bitwise Thevenin equality (see [`same_drive`]).
fn same_thevenin(x: &TheveninDrive, y: &TheveninDrive) -> bool {
    x.volts.total_cmp(&y.volts).is_eq() && x.impedance.total_cmp(&y.impedance).is_eq()
}

/// The form a drive takes in the slot table. A Thevenin drive behind a
/// non-finite impedance *is* a released pin — `NODES.md` §10, "`ohms = ∞` is
/// normalised to released at the slot, never ranked" — so it becomes `None`
/// here, before it can source a cluster, rank against anything, or
/// escalate a solve. A non-finite injection is dropped the same way. A
/// periodic drive is kept whole: a phase behind a non-finite impedance
/// sources nothing in that phase ([`phase_port`]), which is the per-phase
/// form of the same rule.
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
            scratch: std::cell::RefCell::default(),
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
            periodic_slots: Vec::new(),
            coupled_cache: HashMap::new(),
            reach_clusters: None,
            net_count,
            topology: None,
            topology_version: 0,
            dirty: Vec::new(),
            terminal_states: Vec::new(),
            terminals_resolved: false,
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

    /// A periodic drive changed on `slot` (or stopped being one): every
    /// cluster its rate reaches across a coupling capacitor is dirty too —
    /// the coupling rule's fan-out, as a terminal's change dirties its
    /// dependents. With no usable topology the next pass is a full one.
    fn mark_coupled_dirty(&mut self, slot: usize) {
        let Some(topology) = self
            .topology
            .take_if(|t| t.version == self.topology_version && self.slots[slot].net < t.n)
        else {
            return;
        };
        self.ensure_reach(slot, &topology);
        let reached: Vec<usize> = self.coupled_cache[&slot]
            .1
            .iter()
            .map(|coupled| topology.cluster_index[coupled.root])
            .collect();
        for cluster in reached {
            if !self.dirty.contains(&cluster) {
                self.dirty.push(cluster);
            }
        }
        self.topology = Some(topology);
    }

    /// The AC reach of one slot: every root a rate on it reaches across one
    /// or more coupling capacitors within the collapse radius
    /// ([`COUPLED_REACH_OHMS`] of conduction ohms, the capacitors at
    /// 0 Ω), ascending, each with the crossings on its cheapest path — a
    /// function of the topology alone, walked once per topology version
    /// and cached. A root the slot's own conduction cluster holds is never
    /// in it (rule 2 decides that cluster), and neither is a declared
    /// terminal, which the walk never continues past either (phase 1's
    /// decision (b)): a rate coupled into a stuck ground or a rail is
    /// shunted there.
    ///
    /// `R_far` is an estimate, deliberately cheap: the smallest conduction
    /// edge incident to the far root (the resistor that biases it, which is
    /// the Thevenin resistance of a self-biased stage to within its
    /// driver's few ohms), or `+∞` when nothing resistive touches it — a
    /// lone CMOS input. A strong driver on the far node is not in the
    /// estimate: that is a DC fight the node itself reports.
    fn ensure_reach(&mut self, slot: usize, topology: &Topology) {
        if let Some((version, _)) = self.coupled_cache.get(&slot) {
            if *version == topology.version {
                return;
            }
        }
        let from = topology.root_of[self.slots[slot].net];
        let own_cluster = topology.cluster_index[from];
        let is_terminal = |root: usize| topology.terminal_roots.binary_search(&root).is_ok();
        let reach = coupled_reach(
            &topology.root_edges,
            &topology.root_couplings,
            from,
            &topology.terminal_roots,
        );
        // hash-order shape 2: the reached roots are collected and sorted.
        let mut roots: Vec<CoupledRoot> = reach
            .into_iter()
            .filter(|(root, (ohms, path))| {
                *ohms < COUPLED_REACH_OHMS
                    && !path.is_empty()
                    && !is_terminal(*root)
                    && topology.cluster_index[*root] != own_cluster
            })
            .map(|(root, (_, path))| CoupledRoot {
                root,
                crossings: path
                    .iter()
                    .map(|&(ci, far_root)| {
                        let capacitor = &self.couplings[ci];
                        CouplingCrossing {
                            capacitor: capacitor.reference.clone(),
                            far_root,
                            farads: capacitor.farads,
                            far_ohms: topology
                                .smallest_edge_at
                                .get(&far_root)
                                .copied()
                                .unwrap_or(f64::INFINITY),
                        }
                    })
                    .collect(),
            })
            .collect();
        roots.sort_by_key(|coupled| coupled.root);
        self.coupled_cache.insert(slot, (topology.version, roots));
    }

    /// The periodic rates arriving across coupling capacitors at the roots
    /// of the clusters `wanted` names (every cluster, for `None`), as
    /// `(cluster, arrival)` in ascending cluster then slot order. Nothing
    /// to walk on a board with no clock.
    fn arrivals(&mut self, topology: &Topology, wanted: Option<&[usize]>) -> Vec<(usize, Arrival)> {
        let mut arrivals: Vec<(usize, Arrival)> = Vec::new();
        if self.periodic_slots.is_empty() {
            return arrivals;
        }
        let fresh = self
            .reach_clusters
            .as_ref()
            .is_some_and(|(version, _)| *version == topology.version);
        if !fresh {
            let mut clusters: Vec<usize> = Vec::new();
            for at in 0..self.periodic_slots.len() {
                let slot = self.periodic_slots[at];
                self.ensure_reach(slot, topology);
                clusters.extend(
                    self.coupled_cache[&slot]
                        .1
                        .iter()
                        .map(|coupled| topology.cluster_index[coupled.root]),
                );
            }
            clusters.sort_unstable();
            clusters.dedup();
            self.reach_clusters = Some((topology.version, clusters));
        }
        if let (Some(wanted), Some((_, reached))) = (wanted, &self.reach_clusters) {
            if !wanted.iter().any(|c| reached.binary_search(c).is_ok()) {
                return arrivals;
            }
        }
        // Read in place: a pass whose dirty clusters no clock reaches — every
        // edge of a boot on a board with a clock — allocates nothing here.
        for &si in &self.periodic_slots {
            let Some(Drive::Periodic { hi, lo, segment }) = self.slots[si].drive else {
                continue;
            };
            let Some((_, reach)) = self.coupled_cache.get(&si) else {
                continue;
            };
            for coupled in reach {
                let cluster = topology.cluster_index[coupled.root];
                if wanted.is_some_and(|wanted| wanted.binary_search(&cluster).is_err()) {
                    continue;
                }
                arrivals.push((
                    cluster,
                    Arrival {
                        root: coupled.root,
                        slot: si,
                        crossings: coupled.crossings.clone(),
                        hi: level_of_volts(hi.volts),
                        lo: level_of_volts(lo.volts),
                        swing: Arrival::swing_of(hi, lo),
                        segment,
                    },
                ));
            }
        }
        // Stable: slots stay ascending within a cluster.
        arrivals.sort_by_key(|(cluster, _)| *cluster);
        arrivals
    }

    /// Add a coupling capacitor between two nets: an AC path a periodic
    /// drive crosses ([`overlay_arrivals`]), never a conduction edge.
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
            terminal: None,
            port: false,
        });
        let endpoint = EndpointId(self.slots.len() - 1);
        self.note_periodic(endpoint.0);
        endpoint
    }

    /// Stamp a pin's declared input port ([`crate::InputPort`]): a
    /// permanent Thevenin source on the pin's net — `v_bias` behind `r_in`
    /// — ranked by rule 2 like any source and never republished (`NODES.md`
    /// §10). Its slot has no endpoint a component holds.
    pub(crate) fn add_port(&mut self, net: usize, pin: PinRef, port: TheveninDrive) {
        self.topology_version += 1;
        self.slots.push(DriveSlot {
            net,
            pin,
            drive: normalise_drive(Some(Drive::Thevenin(port))),
            terminal: None,
            port: true,
        });
    }

    /// Keep [`Self::periodic_slots`] in step with a slot's drive. A
    /// `PowerOut` pin's slot is its terminal's, and a rail is not a clock
    /// ([`TerminalDrive::from_slot`]): a periodic drive there sources
    /// nothing, across a capacitor or otherwise.
    fn note_periodic(&mut self, slot: usize) {
        let slot_ref = &self.slots[slot];
        let periodic =
            slot_ref.terminal.is_none() && matches!(slot_ref.drive, Some(Drive::Periodic { .. }));
        match self.periodic_slots.binary_search(&slot) {
            Ok(at) if !periodic => {
                self.periodic_slots.remove(at);
                self.reach_clusters = None;
            }
            Err(at) if periodic => {
                self.periodic_slots.insert(at, slot);
                self.reach_clusters = None;
            }
            _ => {}
        }
    }

    /// Replace an endpoint's drive contribution (`None` releases to high-Z;
    /// so does a Thevenin drive behind a non-finite impedance, see
    /// [`normalise_drive`]). Live path only; the next pass sees the new table.
    ///
    /// Returns whether the table changed. An identical drive is a no-op that
    /// marks nothing dirty — a card re-asserting the level it already holds,
    /// or a pin re-driven high on every clock edge, costs no resolution.
    ///
    /// On a `PowerOut` pin's slot ([`Resolver::add_terminal_endpoint`]) the
    /// drive is what the pin's terminal holds ([`TerminalDrive::from_slot`]),
    /// and a change to that dirties the terminal's own cluster and every
    /// cluster in its fan-out — never the slot's cluster alone.
    pub(crate) fn set_drive(&mut self, endpoint: EndpointId, drive: Option<Drive>) -> bool {
        let drive = normalise_drive(drive);
        let Some(slot) = self.slots.get_mut(endpoint.0) else {
            tracing::warn!(endpoint = endpoint.0, "drive for unknown endpoint dropped");
            return false;
        };
        if same_drive(&slot.drive, &drive) {
            return false;
        }
        let was_periodic = matches!(slot.drive, Some(Drive::Periodic { .. }));
        slot.drive = drive;
        let net = slot.net;
        let terminal = slot.terminal;
        self.note_periodic(endpoint.0);
        if terminal.is_none() && (was_periodic || matches!(drive, Some(Drive::Periodic { .. }))) {
            self.mark_coupled_dirty(endpoint.0);
        }
        if let Some(id) = terminal {
            // A rail takes one of the three encodings: a current into a
            // terminal is not a rail and a rail is not a clock, so either
            // releases it — and the part that published it hears why.
            if matches!(drive, Some(Drive::Current { .. } | Drive::Periodic { .. })) {
                tracing::warn!(
                    endpoint = endpoint.0,
                    ?drive,
                    "a rail's terminal holds a Thevenin drive only: this drive releases it"
                );
            }
            let held = TerminalDrive::from_slot(drive);
            if self.terminal_source(id).drive != held {
                self.terminal_source_mut(id).drive = held;
                self.mark_terminal_dirty(id);
            }
            return true;
        }
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

    /// Add a power-rail source (a harness power endpoint) holding `volts`
    /// (`NaN` = sourced at an unmodelled voltage).
    pub(crate) fn add_power_source(&mut self, net: usize, volts: Volts) -> TerminalId {
        self.topology_version += 1;
        self.power_sources.push(TerminalSource {
            net,
            drive: TerminalDrive::from_volts(volts),
        });
        TerminalId::Power(self.power_sources.len() - 1)
    }

    /// Add a `net_stuck` fault source.
    pub(crate) fn add_stuck_source(&mut self, net: usize, volts: Volts) -> TerminalId {
        self.topology_version += 1;
        self.stuck_sources.push(TerminalSource {
            net,
            drive: TerminalDrive::from_volts(volts),
        });
        TerminalId::Stuck(self.stuck_sources.len() - 1)
    }

    /// Register a `PowerOut` pin: its net is a declared terminal from
    /// build on (membership is fixed at build, `NODES.md` §2 rule 3), held
    /// at `idle` until the part publishes, and the returned slot is how
    /// the part drives it — a [`Drive::Thevenin`] sets the voltage the
    /// terminal holds (the impedance is recorded on the slot for the I-V
    /// port, not solved), a release lets it go ([`TerminalDrive::from_slot`]).
    /// The slot is never a source of the cluster it sits in and carries no
    /// current: a rail is a constant, and its current spans clusters.
    pub(crate) fn add_terminal_endpoint(
        &mut self,
        net: usize,
        pin: PinRef,
        idle: Option<Drive>,
    ) -> EndpointId {
        self.topology_version += 1;
        let id = TerminalId::Power(self.power_sources.len());
        // The slot holds the idle drive in the encoding `set_drive`
        // compares against, so the part's first publish — a release
        // included — is a change: an unmodelled idle is the NaN-volt
        // Thevenin `PowerOut` has always meant, a released one is `None`,
        // and a declared Thevenin keeps its impedance on the slot — the
        // I-V port's record, never solved — as a published one does
        // (`TerminalDrive::idle_slot_drive` for the first two).
        let drive = normalise_drive(idle);
        self.slots.push(DriveSlot {
            net,
            pin,
            drive,
            terminal: Some(id),
            port: false,
        });
        self.power_sources.push(TerminalSource {
            net,
            drive: TerminalDrive::from_slot(drive),
        });
        EndpointId(self.slots.len() - 1)
    }

    /// Change what a declared terminal source holds its net at — the
    /// oracle's entry today, and the harness's the day a scenario re-sets
    /// a supply live; a `PowerOut` pin's part goes through its slot
    /// ([`Self::set_drive`]). Returns whether it changed; a change dirties
    /// the terminal's cluster and its fan-out.
    #[cfg_attr(not(test), allow(dead_code))]
    pub(crate) fn set_terminal(&mut self, id: TerminalId, drive: TerminalDrive) -> bool {
        if self.terminal_source(id).drive == drive {
            return false;
        }
        self.terminal_source_mut(id).drive = drive;
        self.mark_terminal_dirty(id);
        true
    }

    /// Every terminal source in canonical order: power sources, then
    /// stuck faults, each in declaration order.
    fn terminal_sources(&self) -> impl Iterator<Item = &TerminalSource> {
        self.power_sources.iter().chain(self.stuck_sources.iter())
    }

    fn terminal_source(&self, id: TerminalId) -> &TerminalSource {
        match id {
            TerminalId::Power(i) => &self.power_sources[i],
            TerminalId::Stuck(i) => &self.stuck_sources[i],
        }
    }

    fn terminal_source_mut(&mut self, id: TerminalId) -> &mut TerminalSource {
        match id {
            TerminalId::Power(i) => &mut self.power_sources[i],
            TerminalId::Stuck(i) => &mut self.stuck_sources[i],
        }
    }

    /// The declared terminal roots — every root a terminal source is
    /// declared on, whatever it holds — sorted and deduplicated: the roots
    /// no path continues past, no edge or element unions through, and
    /// each of which is a cluster of its own.
    fn terminal_roots(&self, root_of: &[usize]) -> Vec<usize> {
        let mut roots: Vec<usize> = self
            .terminal_sources()
            .map(|source| root_of[source.net])
            .collect();
        roots.sort_unstable();
        roots.dedup();
        roots
    }

    /// The fan-out walk: a changed terminal dirties its own cluster and
    /// every cluster that reads it. With no usable cache the next pass is
    /// a full one anyway.
    fn mark_terminal_dirty(&mut self, id: TerminalId) {
        let net = self.terminal_source(id).net;
        let Some(topology) = self
            .topology
            .as_ref()
            .filter(|t| t.version == self.topology_version && net < t.root_of.len())
        else {
            return;
        };
        let root = topology.root_of[net];
        let Ok(position) = topology.terminals.binary_search_by_key(&root, |t| t.root) else {
            return;
        };
        let terminal = &topology.terminals[position];
        let mut touched: Vec<usize> = Vec::with_capacity(1 + terminal.dependents.len());
        touched.push(terminal.cluster);
        touched.extend(terminal.dependents.iter().copied());
        for cluster in touched {
            if !self.dirty.contains(&cluster) {
                self.dirty.push(cluster);
            }
        }
    }

    /// The fight of a terminal's own sources, solved as the one-node
    /// cluster it is: the numeric sources as the ideal 0 Ω sources they
    /// rank as, Norton-stamped at the ideal floor by the same solver every
    /// cluster uses — counted, because it is an escalation.
    fn solve_fight(&self, root: usize, volts: &[Volts], solver: &dyn ClusterSolver) -> Volts {
        let node = NetId(root);
        let inputs = ClusterInputs {
            sources: volts
                .iter()
                .map(|&v| ClusterSource {
                    node,
                    volts: v,
                    impedance: 0.0,
                })
                .collect(),
            ..ClusterInputs::default()
        };
        self.escalated_solves.fetch_add(1, Ordering::SeqCst);
        match solver
            .solve(
                &Cluster {
                    nodes: vec![node],
                    resistors: Vec::new(),
                },
                &inputs,
            )
            .state_of(node)
        {
            Some(NetState::Analog(v)) => v,
            _ => f64::NAN,
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

    /// Register a current instrument's net: its cluster solves on every
    /// pass that touches it, and nothing is reported for it.
    pub(crate) fn add_current_instrument(&mut self, net: usize) {
        self.topology_version += 1;
        self.current_instruments.push(net);
    }

    /// Declare nets read at run time — a released pad's subscription, a
    /// current instrument ([`Command::DeclareRead`]) — and mark the clusters
    /// they sit in dirty, so the next [`Self::resolve_dirty`] resolves those
    /// clusters under their new reads and nothing else. A read changes what
    /// its own cluster reports or solves and no other cluster's inputs, so
    /// the pass is exact; and it makes the declaration's cost independent
    /// of how the declarations were batched — a full pass per batch
    /// re-solved every element cluster on the board once per batch, and
    /// the batches were the attaching thread's timing. With no topology
    /// built yet (or one out of date) the reads join the lists the next
    /// build reads, as [`Self::add_digital_sense`] does.
    pub(crate) fn declare_reads(&mut self, reads: &[(usize, ReadKind)]) {
        let current = self.terminals_resolved
            && self
                .topology
                .as_ref()
                .is_some_and(|t| t.version == self.topology_version);
        for &(net, kind) in reads {
            let list = match kind {
                ReadKind::Digital => &mut self.digital_senses,
                ReadKind::Instrument => &mut self.current_instruments,
            };
            let position = list.len();
            list.push(net);
            if !current {
                continue;
            }
            let topology = self.topology.as_mut().expect("current");
            let Some(&cid) = topology.cluster_index.get(net) else {
                continue;
            };
            let cluster = &mut topology.clusters[cid];
            match kind {
                ReadKind::Digital => cluster.digital_senses.push((position, net)),
                ReadKind::Instrument => cluster.current_instruments.push((position, net)),
            }
            if !self.dirty.contains(&cid) {
                self.dirty.push(cid);
            }
        }
        if !current {
            self.topology_version += 1;
        }
    }

    /// The pins whose slots drive the identity root `root` right now: a
    /// Thevenin or periodic drive on a non-terminal slot (a `PowerOut`
    /// pin's slot is its terminal's, not a driver; an input port is the
    /// pin's own load, not a driver). The build's
    /// mechanical-pad lint asks ([`crate::Finding::MechanicalOnDrivenNet`]).
    pub(crate) fn driving_pins(&self, root_of: &[usize], root: usize) -> Vec<PinRef> {
        self.slots
            .iter()
            .filter(|slot| {
                slot.terminal.is_none()
                    && !slot.port
                    && matches!(
                        slot.drive,
                        Some(Drive::Thevenin(_)) | Some(Drive::Periodic { .. })
                    )
                    && root_of.get(slot.net).copied() == Some(root)
            })
            .map(|slot| slot.pin.clone())
            .collect()
    }

    /// The identity roots a resistive path reaches from the identity root
    /// `root` — rule 2's reach, a declared terminal ending every path: the
    /// roots of its cluster and the terminals on the cluster's boundary at
    /// a finite series resistance, `root` itself first. The build's
    /// pull-up lint asks ([`crate::Finding::OpenDrainWithoutPullUp`]).
    pub(crate) fn reached_roots(&mut self, net_count: usize, root: usize) -> Vec<usize> {
        self.ensure_topology(net_count);
        let topology = self.topology.as_ref().expect("ensure_topology built it");
        let c = &topology.clusters[topology.cluster_index[root]];
        let Some(pr) = c.roots.iter().position(|&r| r == root) else {
            return vec![root];
        };
        let columns = c.columns();
        let row = &c.dist[pr * columns..(pr + 1) * columns];
        let mut reached = vec![root];
        reached.extend(
            c.roots
                .iter()
                .zip(row)
                .filter(|&(&r, d)| r != root && d.is_finite())
                .map(|(&r, _)| r),
        );
        reached.extend(
            c.boundary
                .iter()
                .zip(&row[c.roots.len()..])
                .filter(|(_, d)| d.is_finite())
                .map(|(&position, _)| topology.terminals[position].root),
        );
        reached
    }

    /// Register a power sense pin (`PowerNetUnsourced` findings).
    pub(crate) fn add_power_sense(&mut self, net: usize) {
        self.topology_version += 1;
        self.power_senses.push(net);
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
    /// A rebuilt cache has no terminal decided yet: the next pass must be a
    /// full one ([`Self::resolve`]), which `dirty_scope`/`resolve_dirty`
    /// honour through `terminals_resolved`.
    fn ensure_topology(&mut self, n: usize) {
        if self.topology_is_current(n) {
            return;
        }
        self.identity.grow(self.net_count.max(n));
        let topology = self.build_topology(n);
        self.terminal_states.clear();
        self.terminals_resolved = false;
        self.topology = Some(topology);
        self.dirty.clear();
    }

    /// Derive every drive-independent structure of the board once: identity
    /// roots, the declared terminals, conduction clusters (dense ids in
    /// ascending cluster-root order), each cluster's nets, roots, edges,
    /// endpoints, boundary terminals and senses, the minimum series
    /// resistance between each of its roots and each root and boundary
    /// terminal, and each terminal's fan-out. Resolution passes are pure
    /// lookups over this afterwards.
    fn build_topology(&mut self, n: usize) -> Topology {
        let root_of: Vec<usize> = (0..n).map(|i| self.identity.find(i)).collect();

        // The declared terminals — rails (modelled, unmodelled or
        // released) and stuck faults — as the roots no path continues
        // past, no edge or element unions through, and each of which is a
        // cluster of its own (`NODES.md` "Three rules the taxonomy rests
        // on", 1).
        let terminal_roots = self.terminal_roots(&root_of);
        let is_terminal = |root: usize| terminal_roots.binary_search(&root).is_ok();
        let terminal_position = |root: usize| terminal_roots.binary_search(&root).ok();

        // Conduction clusters: identity merges are 0-ohm, a conduction
        // edge between two non-terminal roots connects them without merging
        // identity, and an element is a membership edge among its
        // **non-terminal** nets — its two ends and its control, so a gate
        // is in-cluster. A declared terminal is a constant, and a constant
        // is a boundary: an edge ending on one belongs to the cluster of
        // its other end, an element touching one stamps against it as a
        // boundary constant of its own cluster, and the terminal's root
        // joins nothing.
        let mut conduction = Dsu::new(n);
        for (i, &root) in root_of.iter().enumerate() {
            conduction.union(root, i);
        }
        for (a, b, _ohms) in &self.edges {
            let (ra, rb) = (root_of[*a], root_of[*b]);
            if !is_terminal(ra) && !is_terminal(rb) {
                conduction.union(ra, rb);
            }
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
        let clusters: Vec<ClusterTopo> = (0..cluster_roots.len())
            .map(|cid| {
                let nets: Vec<usize> = (0..n).filter(|&i| cluster_index[i] == cid).collect();
                let roots: Vec<usize> = nets.iter().copied().filter(|&i| root_of[i] == i).collect();
                // An edge belongs to the cluster of its non-terminal
                // end(s): both ends when neither is a terminal (one
                // cluster, by the union above), one end when the other is
                // a boundary terminal, no cluster when both are.
                let edges: Vec<(usize, usize, f64)> = root_edges
                    .iter()
                    .filter(|(a, b, _)| {
                        (cluster_index[*a] == cid && !is_terminal(*a))
                            || (cluster_index[*b] == cid && !is_terminal(*b))
                    })
                    .copied()
                    .collect();
                let slots: Vec<usize> = (0..self.slots.len())
                    .filter(|&si| {
                        self.slots[si].terminal.is_none()
                            && cluster_index[self.slots[si].net] == cid
                    })
                    .collect();
                let elements: Vec<usize> = (0..self.elements.len())
                    .filter(|&ei| {
                        homes[ei]
                            .as_ref()
                            .is_some_and(|home| cluster_index[home.root] == cid)
                    })
                    .collect();
                // A terminal's own cluster is its root alone.
                let terminal = roots.iter().find_map(|&root| terminal_position(root));
                debug_assert!(terminal.is_none() || roots.len() == 1);
                // The boundary: the terminals the edges end on and the
                // elements' conducting ends name; then the terminals the
                // controls alone read.
                let mut boundary: Vec<usize> = Vec::new();
                for (a, b, _) in &edges {
                    for root in [*a, *b] {
                        if let Some(position) = terminal_position(root) {
                            if !boundary.contains(&position) {
                                boundary.push(position);
                            }
                        }
                    }
                }
                for &ei in &elements {
                    let home = homes[ei].as_ref().expect("a homed element");
                    for &root in &home.terminal_ends {
                        let position = terminal_position(root).expect("a terminal end");
                        if !boundary.contains(&position) {
                            boundary.push(position);
                        }
                    }
                }
                boundary.sort_unstable();
                let mut boundary_controls: Vec<usize> = Vec::new();
                for &ei in &elements {
                    let home = homes[ei].as_ref().expect("a homed element");
                    if let Some(root) = home.terminal_control {
                        let position = terminal_position(root).expect("a terminal control");
                        if !boundary.contains(&position) && !boundary_controls.contains(&position) {
                            boundary_controls.push(position);
                        }
                    }
                }
                boundary_controls.sort_unstable();
                let senses_in = |list: &[usize]| -> Vec<(usize, usize)> {
                    list.iter()
                        .enumerate()
                        .filter(|(_, net)| cluster_index[**net] == cid)
                        .map(|(pos, net)| (pos, *net))
                        .collect()
                };
                // Minimum series resistance from every root of the
                // cluster to every root and every boundary terminal,
                // ending at but never crossing a terminal; INFINITY where
                // no such path exists.
                let k = roots.len();
                let columns = k + boundary.len();
                let mut dist = vec![f64::INFINITY; k * columns];
                for (ia, &ra) in roots.iter().enumerate() {
                    let from = min_path_ohms(&edges, ra, &terminal_roots);
                    for (ib, rb) in roots.iter().enumerate() {
                        if let Some(&ohms) = from.get(rb) {
                            dist[ia * columns + ib] = ohms;
                        }
                    }
                    for (jb, &position) in boundary.iter().enumerate() {
                        if let Some(&ohms) = from.get(&terminal_roots[position]) {
                            dist[ia * columns + k + jb] = ohms;
                        }
                    }
                }
                ClusterTopo {
                    nets,
                    roots,
                    edges,
                    slots,
                    elements,
                    terminal,
                    boundary,
                    boundary_controls,
                    digital_senses: senses_in(&self.digital_senses),
                    analog_senses: senses_in(&self.analog_senses),
                    current_instruments: senses_in(&self.current_instruments),
                    power_senses: senses_in(&self.power_senses),
                    dist,
                }
            })
            .collect();

        // The terminals: their sources in canonical order, their own
        // cluster, and the fan-out — every cluster whose boundary or
        // controls name them.
        let terminals: Vec<TerminalTopo> = terminal_roots
            .iter()
            .enumerate()
            .map(|(position, &root)| {
                let sources: Vec<TerminalId> = (0..self.power_sources.len())
                    .map(TerminalId::Power)
                    .chain((0..self.stuck_sources.len()).map(TerminalId::Stuck))
                    .filter(|&id| root_of[self.terminal_source(id).net] == root)
                    .collect();
                let dependents: Vec<usize> = (0..clusters.len())
                    .filter(|&cid| {
                        clusters[cid].boundary.binary_search(&position).is_ok()
                            || clusters[cid]
                                .boundary_controls
                                .binary_search(&position)
                                .is_ok()
                    })
                    .collect();
                TerminalTopo {
                    root,
                    cluster: cluster_index[root],
                    sources,
                    dependents,
                }
            })
            .collect();

        // Coupling capacitors between roots, and the far-node resistance
        // estimate the AC rule judges a crossing against
        // ([`Resolver::ensure_reach`]).
        let root_couplings: Vec<(usize, usize, usize)> = self
            .couplings
            .iter()
            .enumerate()
            .map(|(ci, c)| (root_of[c.a], root_of[c.b], ci))
            .filter(|(a, b, _)| a != b)
            .collect();
        let mut smallest_edge_at: HashMap<usize, f64> = HashMap::new();
        for (a, b, ohms) in &root_edges {
            for root in [*a, *b] {
                let entry = smallest_edge_at.entry(root).or_insert(f64::INFINITY);
                *entry = entry.min(*ohms);
            }
        }
        Topology {
            version: self.topology_version,
            n,
            root_of,
            cluster_index,
            clusters,
            terminals,
            terminal_roots,
            root_edges,
            root_couplings,
            smallest_edge_at,
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
    /// ([`Self::resolve_dirty`]). One code path, two scopes. The terminals
    /// are decided first, each at its own cluster, so every dependent's
    /// pass reads what its boundary holds *now*.
    pub(crate) fn resolve(
        &mut self,
        nets: &mut [Net],
        diagnostics: &mut Diagnostics,
        solver: &dyn ClusterSolver,
    ) {
        let n = nets.len();
        self.ensure_topology(n);
        let topology = self.topology.take().expect("ensure_topology built it");
        let mut out = PassOutput::default();
        let mut states = std::mem::take(&mut self.terminal_states);
        states.clear();
        states.extend(
            (0..topology.terminals.len())
                .map(|position| self.resolve_terminal(&topology, position, solver)),
        );
        let arrivals = self.arrivals(&topology, None);
        for cid in 0..topology.clusters.len() {
            let reads = ClusterReads {
                terminal_states: &states,
                arrivals: arrivals_in(&arrivals, cid),
            };
            self.resolve_cluster(&topology, cid, nets, reads, &mut out, solver);
        }
        self.terminal_states = states;
        self.terminals_resolved = true;
        out.findings.emit(diagnostics);
        self.apply_currents(out.currents);
        self.topology = Some(topology);
        self.dirty.clear();
    }

    /// The nets the next [`Self::resolve_dirty`] will touch, ascending: the
    /// members of every cluster whose drive table changed — or every net,
    /// when the topology changed and the next pass must be a full one.
    ///
    /// Written into `scope`, a list the caller keeps between passes, so the
    /// per-edge path allocates none.
    pub(crate) fn dirty_scope(&self, n: usize, scope: &mut Vec<usize>) {
        scope.clear();
        if !self.topology_is_current(n) || !self.terminals_resolved {
            scope.extend(0..n);
            return;
        }
        let topology = self.topology.as_ref().expect("current");
        scope.extend(
            self.dirty
                .iter()
                .flat_map(|&cid| topology.clusters[cid].nets.iter().copied()),
        );
        scope.sort_unstable();
        scope.dedup();
    }

    /// Resolve only the clusters a drive changed since the last pass (the
    /// scope [`Self::dirty_scope`] announced), reporting their findings.
    /// Every other net keeps its state, which is exactly what the full pass
    /// would have recomputed for it. Falls back to a full pass when the
    /// topology changed underneath. A dirty terminal cluster is decided
    /// before any dirty dependent is resolved — the fan-out marked both.
    pub(crate) fn resolve_dirty(
        &mut self,
        nets: &mut [Net],
        diagnostics: &mut Diagnostics,
        solver: &dyn ClusterSolver,
    ) {
        if !self.topology_is_current(nets.len()) || !self.terminals_resolved {
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
        let mut out = PassOutput::default();
        let mut states = std::mem::take(&mut self.terminal_states);
        for &cid in &dirty {
            if let Some(position) = topology.clusters[cid].terminal {
                states[position] = self.resolve_terminal(&topology, position, solver);
            }
        }
        let arrivals = self.arrivals(&topology, Some(&dirty));
        for &cid in &dirty {
            let reads = ClusterReads {
                terminal_states: &states,
                arrivals: arrivals_in(&arrivals, cid),
            };
            self.resolve_cluster(&topology, cid, nets, reads, &mut out, solver);
        }
        self.terminal_states = states;
        out.findings.emit(diagnostics);
        self.apply_currents(out.currents);
        self.topology = Some(topology);
        // The dirty list back, emptied, at its capacity: a pass marks
        // nothing dirty, so the next drive's push allocates nothing.
        debug_assert!(self.dirty.is_empty(), "a pass marks nothing dirty");
        dirty.clear();
        self.dirty = dirty;
    }

    /// Decide what one terminal holds its root at, from its sources alone
    /// ([`decide_terminal`]).
    fn resolve_terminal(
        &self,
        topology: &Topology,
        position: usize,
        solver: &dyn ClusterSolver,
    ) -> TerminalState {
        let terminal = &topology.terminals[position];
        decide_terminal(
            terminal
                .sources
                .iter()
                .map(|&id| self.terminal_source(id).drive),
            |volts| self.solve_fight(terminal.root, volts, solver),
        )
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
    /// cluster boundary; a terminal on the boundary is read as the one
    /// state its own cluster decided ([`TerminalState`]) — which is what
    /// makes resolving a subset exact. Iteration is over dense, ascending
    /// indices throughout (never a hash walk), so a pass is bit-for-bit
    /// reproducible; see `DETERMINISM.md`.
    fn resolve_cluster(
        &self,
        topology: &Topology,
        cid: usize,
        nets: &mut [Net],
        reads: ClusterReads<'_>,
        out: &mut PassOutput,
        solver: &dyn ClusterSolver,
    ) {
        let ClusterReads {
            terminal_states,
            arrivals,
        } = reads;
        let c = &topology.clusters[cid];
        let root_of = &topology.root_of;
        let k = c.roots.len();
        let columns = c.columns();
        let has_elements = !c.elements.is_empty();
        let pos_of_root = |root: usize| -> usize {
            c.roots
                .iter()
                .position(|&r| r == root)
                .expect("a cluster's sources sit on its own roots")
        };

        // A cluster a periodic drive sits in resolves **twice** — once with
        // every periodic slot at its high port, once at its low port — and
        // each root combines the two outcomes ([`combine_phases`]). Every
        // other cluster resolves once, exactly as before: the second pass
        // is paid only where a clock is (`DESIGN.md` rule 8).
        let periodic = c
            .slots
            .iter()
            .any(|&si| matches!(self.slots[si].drive, Some(Drive::Periodic { .. })));
        // Every slot source in the cluster, in endpoint order, per phase:
        // the SPICE card order the cluster solver stamps (determinism), and
        // the tie-break order of rule 2's ranking. Current injections are
        // collected apart: they reach nothing and rank nowhere; they are
        // stamped into the solve. The low phase's lists stay empty — and
        // unallocated — in a cluster no periodic drive sits in.
        //
        // The per-cluster lists live in the resolver's scratch between
        // passes (`ClusterScratch`): a pass over a cluster with no periodic
        // drive allocates nothing for them (the edge path's cost,
        // `NODES.md` §12 item 5, the review).
        let mut scratch = self.scratch.take();
        let phase_sources = |phase: Option<Phase>,
                             sources: &mut Vec<ClusterSource>,
                             source_slots: &mut Vec<usize>| {
            sources.clear();
            source_slots.clear();
            for &si in &c.slots {
                let slot = &self.slots[si];
                if let Some(port) = phase_port(slot.drive, phase) {
                    sources.push(ClusterSource {
                        node: NetId(root_of[slot.net]),
                        volts: port.volts,
                        impedance: port.impedance,
                    });
                    source_slots.push(si);
                }
            }
        };
        let mut hi_sources = std::mem::take(&mut scratch.sources);
        let mut hi_slots = std::mem::take(&mut scratch.source_slots);
        phase_sources(
            periodic.then_some(Phase::High),
            &mut hi_sources,
            &mut hi_slots,
        );
        let (mut lo_sources, mut lo_slots) = (Vec::new(), Vec::new());
        if periodic {
            phase_sources(Some(Phase::Low), &mut lo_sources, &mut lo_slots);
        }
        let mut cluster_sourced = !hi_sources.is_empty() || !lo_sources.is_empty();
        let mut injections: Vec<ClusterInjection> = Vec::new();
        let mut injection_slots: Vec<usize> = Vec::new();
        for &si in &c.slots {
            let slot = &self.slots[si];
            if let Some(Drive::Current { amps }) = slot.drive {
                injections.push(ClusterInjection {
                    node: NetId(root_of[slot.net]),
                    amps,
                });
                injection_slots.push(si);
            }
        }

        // The terminals, each as the one state its own cluster decided:
        // the cluster's own (when it is a terminal's), then its boundary,
        // then the controls' — in the solver's card order. A voltage is a
        // Dirichlet constant of the solve and an ideal source of the
        // ranking through the path to it; an unmodelled rail sources the
        // cluster and is the fallback presentation of a root nothing
        // numeric reaches; a released one is nothing at all. A control's
        // terminal is a constant for its region test and sources nothing.
        let mut terminals: Vec<ClusterTerminal> = Vec::new();
        let mut terminal_sources: Vec<TerminalSourceColumn> = Vec::new();
        let mut unmodelled_columns: Vec<usize> = Vec::new();
        let mut boundary_nodes: Vec<NetId> = Vec::new();
        let mut own_fight: Option<Volts> = None;
        let mut admit = |position: usize, column: usize, boundary: bool| {
            let state = terminal_states[position];
            let node = NetId(topology.terminals[position].root);
            match state.drive {
                TerminalDrive::Volts(volts) => {
                    terminals.push(ClusterTerminal { node, volts });
                    terminal_sources.push(TerminalSourceColumn { volts, column });
                    if boundary {
                        boundary_nodes.push(node);
                    } else if state.fought {
                        own_fight = Some(volts);
                    }
                    cluster_sourced = true;
                }
                TerminalDrive::Unmodelled => {
                    unmodelled_columns.push(column);
                    cluster_sourced = true;
                }
                TerminalDrive::Released => {}
            }
        };
        if let Some(position) = c.terminal {
            admit(position, 0, false);
        }
        for (j, &position) in c.boundary.iter().enumerate() {
            admit(position, k + j, true);
        }
        for &position in &c.boundary_controls {
            if let TerminalDrive::Volts(volts) = terminal_states[position].drive {
                terminals.push(ClusterTerminal {
                    node: NetId(topology.terminals[position].root),
                    volts,
                });
            }
        }
        let terminal_numeric = !terminal_sources.is_empty();
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

        // An analog sense reads a voltage, a current injection has no
        // projection form (its effect is `I · R` along whatever the node is
        // tied to), an element's region has none either, and a current
        // instrument reads what only a solve has — so any of them asks for
        // the cluster's operating point: every root reached by a numeric
        // source publishes the solved voltage, and rule 2's fights are
        // reported beside it exactly as they are without the reader — two
        // strong drivers disagreeing across less than the pull bar, two
        // terminal sources fighting on one root (`NODES.md` §12 item 5,
        // the rules task, which retired phase 1's operating-point
        // precedence: the reader is handed the operating point by its
        // `Sense`, so a finding no longer costs it the voltage). A periodic
        // cluster escalated this way solves once per phase.
        let injected = injections.iter().any(|i| i.amps != 0.0);
        let any_source = !hi_sources.is_empty() || !lo_sources.is_empty();
        //
        // A root exactly one source reaches is the exception (`DESIGN.md`
        // rule 8, `NODES.md` §10 "Resolution"): no other source, terminal
        // or injection shares its conduction component — a second one
        // would reach it — so no current flows there and the node sits at
        // that source's open-circuit voltage exactly. An analog reader is
        // handed that voltage without a solve; only a root two or more
        // sources reach asks the solver for it. An injection, an
        // instrument and an element still solve the cluster whatever the
        // count: the first two need a current, the last its region.
        let eager = (any_source || terminal_numeric)
            && (injected || !c.current_instruments.is_empty() || has_elements);
        let on_request = eager || ((any_source || terminal_numeric) && !c.analog_senses.is_empty());

        // One phase: every root's rule-2 outcome for one source list, handed
        // to `emit` with the sources that reached it, by position in
        // `c.roots`; the phase's solve, when it ran, is returned. The
        // cluster solve is built at most once per phase, on demand; the
        // terminals enter as constants, linear clusters included
        // (`cluster.rs`, "Terminals are constants").
        let run_phase = |sources: &[ClusterSource],
                         source_slots: &[usize],
                         reaching: &mut Vec<ReachingSource>,
                         emit: &mut dyn FnMut(usize, RootOutcome, &[ReachingSource])|
         -> Option<ClusterSolution> {
            let solve = || -> ClusterSolution {
                self.solve_cluster(
                    c,
                    &boundary_nodes,
                    ClusterInputs {
                        sources: sources.to_vec(),
                        injections: injections.clone(),
                        terminals: terminals.clone(),
                        elements: elements.clone(),
                    },
                    solver,
                )
            };
            let mut solution: Option<ClusterSolution> = eager.then(solve);
            for (pr, &root) in c.roots.iter().enumerate() {
                reaching.clear();
                for (source, &si) in sources.iter().zip(source_slots) {
                    let path = c.dist[pr * columns + pos_of_root(source.node.0)];
                    if !path.is_finite() {
                        continue; // no resistive path: does not reach this root
                    }
                    reaching.push(ReachingSource {
                        slot: Some(si),
                        volts: source.volts,
                        impedance: source.impedance,
                        path,
                    });
                }
                for terminal in &terminal_sources {
                    let path = c.dist[pr * columns + terminal.column];
                    if !path.is_finite() {
                        continue;
                    }
                    reaching.push(ReachingSource {
                        slot: None,
                        volts: terminal.volts,
                        impedance: 0.0,
                        path,
                    });
                }
                // Nothing numeric reaches the root: an unmodelled rail that
                // does presents as up through the path to it (the supply
                // gates read `Pulled(High)` as a rail that is there);
                // otherwise the root floats.
                let unmodelled_or_floating = || {
                    let nearest = unmodelled_columns
                        .iter()
                        .map(|&column| c.dist[pr * columns + column])
                        .filter(|d| d.is_finite())
                        .fold(f64::INFINITY, f64::min);
                    if nearest.is_finite() {
                        NetState::Pulled(Level::High, nearest)
                    } else {
                        NetState::Floating
                    }
                };
                let mut outcome = if let (true, Some(solution)) = (has_elements, solution.as_ref())
                {
                    // An element cluster: the solve decided every root, the
                    // elements' far sides included (no resistive path
                    // reaches those, so the ranking has nothing to say
                    // about them). The ranking's fights among the linear
                    // sources are still reported.
                    let ranked = if reaching.is_empty() {
                        None
                    } else {
                        let mut solved = || match solution.state_of(NetId(root)) {
                            Some(NetState::Analog(v)) => Some(v),
                            _ => None,
                        };
                        Some(project_root(reaching, &mut solved))
                    };
                    // No operating point: every non-terminal root floats
                    // (`NODES.md` §7), an unmodelled rail's path
                    // notwithstanding.
                    let state = match solution.state_of(NetId(root)) {
                        Some(NetState::Analog(v)) => NetState::Analog(v),
                        _ if !solution.converged => NetState::Floating,
                        _ => unmodelled_or_floating(),
                    };
                    // The voltage is the solve's, never the ranking's
                    // winner: the state is the operating point.
                    RootOutcome {
                        state,
                        volts: RootOutcome::quiet(state).volts,
                        ..ranked.unwrap_or_else(|| RootOutcome::quiet(state))
                    }
                } else if reaching.is_empty() {
                    RootOutcome::quiet(unmodelled_or_floating())
                } else if let ([only], false, true) = (reaching.as_slice(), eager, on_request) {
                    // The single-source rule: an analog reader's root one
                    // source reaches is handed its open-circuit voltage,
                    // unsolved.
                    RootOutcome {
                        state: NetState::Analog(only.volts),
                        volts: Some(only.volts),
                        fight: None,
                        ambiguous: None,
                        solved: false,
                    }
                } else if on_request {
                    // Escalated on request — an analog reader of a root two
                    // or more sources reach, an injection or an instrument:
                    // the operating point is published and rule 2's fights
                    // are reported beside it.
                    let solution = &*solution.get_or_insert_with(solve);
                    let state = solution.state_of(NetId(root)).unwrap_or_else(|| {
                        tracing::warn!(net = %nets[root].name, "cluster solver omitted a node; reporting Floating");
                        NetState::Floating
                    });
                    let mut solved = || match solution.state_of(NetId(root)) {
                        Some(NetState::Analog(v)) => Some(v),
                        _ => None,
                    };
                    RootOutcome {
                        state,
                        volts: RootOutcome::quiet(state).volts,
                        ..project_root(reaching, &mut solved)
                    }
                } else {
                    let mut solved = || -> Option<Volts> {
                        let solution = solution.get_or_insert_with(solve);
                        match solution.state_of(NetId(root)) {
                            Some(NetState::Analog(v)) => Some(v),
                            _ => None,
                        }
                    };
                    project_root(reaching, &mut solved)
                };
                // A terminal two of its own sources fought over — a rail
                // against a `net_stuck` — is decided once, here, at the
                // fight's operating point: one finding naming the strong
                // slots that fought it too, if any (a terminal has no pin),
                // and the voltage as `AmbiguousLevel` inside the dead band.
                // The state is `Contention` inside the band and the voltage
                // outside it — or, in a cluster solved on request, the
                // operating point the solve holds the terminal at, the
                // fight reported beside it.
                if let Some(volts) = own_fight {
                    debug_assert_eq!(pr, 0);
                    let in_band = V_IL < volts && volts < V_IH;
                    outcome.fight = Some(outcome.fight.take().unwrap_or_default());
                    outcome.solved = false;
                    outcome.volts = Some(volts);
                    outcome.ambiguous = in_band.then_some(volts);
                    if !on_request {
                        outcome.state = if in_band {
                            NetState::Contention
                        } else {
                            NetState::Analog(volts)
                        };
                    }
                }
                emit(pr, outcome, reaching);
            }
            solution
        };

        // Per root, by position in `c.roots`: the state, and rule 2's
        // findings — the strong sources fighting on it, and the solved
        // voltage that fell inside the dead band. A cluster with no
        // periodic drive takes each root's one outcome as it is emitted; a
        // periodic cluster's two phases are recorded ([`PhaseRoot`]) and
        // combine here.
        let mut reaching = std::mem::take(&mut scratch.reaching);
        let mut root_states = std::mem::take(&mut scratch.root_states);
        let mut root_volts = std::mem::take(&mut scratch.root_volts);
        root_states.clear();
        root_volts.clear();
        let mut root_fights: Vec<(usize, Vec<usize>)> = Vec::new();
        let mut root_ambiguous: Vec<(usize, Volts)> = Vec::new();
        let mut record = |root: usize,
                          (state, fight, ambiguous, volts): (
            NetState,
            Option<Vec<usize>>,
            Option<Volts>,
            NetVolts,
        )| {
            if let Some(fighting) = fight {
                root_fights.push((root, fighting));
            }
            if let Some(volts) = ambiguous {
                root_ambiguous.push((root, volts));
            }
            root_states.push(state);
            root_volts.push(volts);
        };
        let (hi_solution, lo_solution, phase_roots) = if periodic {
            let is_periodic =
                |si: usize| matches!(self.slots[si].drive, Some(Drive::Periodic { .. }));
            let mut run = |sources: &[ClusterSource], slots: &[usize]| {
                let mut roots: Vec<PhaseRoot> = Vec::with_capacity(k);
                let solution =
                    run_phase(sources, slots, &mut reaching, &mut |_, outcome, reached| {
                        roots.push(PhaseRoot::of(
                            outcome,
                            reached,
                            Some(&is_periodic as &dyn Fn(usize) -> bool),
                        ));
                    });
                (solution, roots)
            };
            let (hi_solution, hi_roots) = run(&hi_sources, &hi_slots);
            let (lo_solution, lo_roots) = run(&lo_sources, &lo_slots);
            for (pr, &root) in c.roots.iter().enumerate() {
                record(
                    root,
                    combine_phases(&hi_roots[pr], &lo_roots[pr], |si| {
                        match self.slots[si].drive {
                            Some(Drive::Periodic { segment, .. }) => Some(segment),
                            _ => None,
                        }
                    }),
                );
            }
            (hi_solution, lo_solution, Some((hi_roots, lo_roots)))
        } else {
            let solution = run_phase(
                &hi_sources,
                &hi_slots,
                &mut reaching,
                &mut |pr, outcome, _| {
                    record(
                        c.roots[pr],
                        (
                            outcome.state,
                            outcome.fight,
                            outcome.ambiguous,
                            NetVolts::dc(outcome.volts),
                        ),
                    );
                },
            );
            (solution, None, None)
        };
        // The rates arriving across coupling capacitors, over the states
        // the cluster's own sources decided (the AC rule).
        if !arrivals.is_empty() {
            let overlays = overlay_arrivals(
                arrivals,
                |root| root_states[pos_of_root(root)],
                |root| match &phase_roots {
                    Some((hi, lo)) => {
                        let pr = pos_of_root(root);
                        slot_union([hi[pr].contending.as_slice(), lo[pr].contending.as_slice()])
                    }
                    None => Vec::new(),
                },
                |root| nets[root].name.clone(),
            );
            for (root, state, volts, fight) in overlays.states {
                root_states[pos_of_root(root)] = state;
                root_volts[pos_of_root(root)] = volts;
                if let Some(fighting) = fight {
                    match root_fights.iter_mut().find(|(r, _)| *r == root) {
                        Some((_, existing)) => {
                            *existing = slot_union([existing.as_slice(), fighting.as_slice()]);
                        }
                        None => root_fights.push((root, fighting)),
                    }
                }
            }
            out.findings.coupling.extend(overlays.refused);
        }

        // -- currents -------------------------------------------------------
        // What the solve, when there was one, says flows into every
        // endpoint and through every element of the cluster. A cluster
        // resolved by projection alone clears a stale reading once and is
        // otherwise silent here: the ROM boot's every pass is such a
        // cluster, and it must pay nothing for a table it never fills
        // (`DESIGN.md` rule 8). A cluster a periodic drive sits in has no
        // single operating point — a square wave has two — so it reports
        // no current anywhere (`sil-unified-drive.md`), clearing any stale
        // reading.
        let solution = if periodic { None } else { hi_solution.as_ref() };
        for &si in &c.slots {
            let amps = solution.and_then(|solution| {
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
                    Some(Drive::Periodic { .. }) => None,
                    None => match solution.state_of(NetId(root_of[slot.net])) {
                        Some(NetState::Analog(_)) => Some(0.0),
                        _ => None,
                    },
                }
            });
            if amps.is_some() || self.endpoint_currents.get(si).is_some_and(Option::is_some) {
                out.currents.endpoints.push((si, amps));
            }
        }
        for (position, &ei) in c.elements.iter().enumerate() {
            let amps = solution
                .and_then(|solution| solution.branch_currents.get(position).copied().flatten());
            if amps.is_some() || self.element_currents.get(ei).is_some_and(Option::is_some) {
                out.currents.elements.push((ei, amps));
            }
        }

        // -- state assignment -----------------------------------------------
        for &i in &c.nets {
            let pr = pos_of_root(root_of[i]);
            nets[i].state = root_states[pr];
            nets[i].volts = root_volts[pr];
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
            out.findings.contention.push((
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
                out.findings.contention.push((
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
                    out.findings.floating.push((
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
        let nonconvergent = hi_solution
            .iter()
            .chain(lo_solution.iter())
            .find(|solution| has_elements && !solution.converged);
        let mut reported_power: Vec<usize> = Vec::new();
        for &(pos, net) in &c.power_senses {
            let root = root_of[net];
            let unsourced = !cluster_sourced
                || (has_elements
                    && nonconvergent.is_none()
                    && nets[net].state == NetState::Floating);
            if unsourced && !reported_power.contains(&root) {
                reported_power.push(root);
                out.findings.power.push((
                    pos,
                    Finding::PowerNetUnsourced {
                        net: nets[net].name.clone(),
                    },
                ));
            }
        }
        // A current injected where no Thevenin source reaches — in either
        // phase of a periodic cluster: the node has no return path, stays
        // Floating, and the injection went nowhere.
        for (injection, &si) in injections.iter().zip(&injection_slots) {
            if injection.amps == 0.0 {
                continue;
            }
            let pr = pos_of_root(injection.node.0);
            let reached = hi_sources
                .iter()
                .chain(&lo_sources)
                .any(|s| c.dist[pr * columns + pos_of_root(s.node.0)].is_finite())
                || terminal_sources
                    .iter()
                    .any(|t| c.dist[pr * columns + t.column].is_finite());
            if !reached {
                let slot = &self.slots[si];
                out.findings.injection.push((
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
        if let Some(solution) = nonconvergent {
            let first = c.nets[0];
            out.findings.nonconvergent.push((
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
        // The lists back into the scratch, emptied of nothing but kept at
        // their capacity for the next cluster.
        scratch.sources = hi_sources;
        scratch.source_slots = hi_slots;
        scratch.reaching = reaching;
        scratch.root_states = root_states;
        scratch.root_volts = root_volts;
        self.scratch.replace(scratch);
    }

    /// Escalate one cluster to the [`ClusterSolver`]: its roots as nodes,
    /// then the boundary terminals holding a voltage (nodes of the solve
    /// only as the constants they are, never members whose state this
    /// cluster publishes), its identity-collapsed edges, and `inputs` —
    /// the slot sources in canonical order, the terminals as constants,
    /// the current injections and the elements. An unmodelled or released
    /// boundary terminal stays outside: an edge or element naming it is
    /// dropped by the solver, as a branch to no voltage is. Counted,
    /// because every solve is a cost the fast path did not pay
    /// ([`Self::escalated_solves`]).
    fn solve_cluster(
        &self,
        c: &ClusterTopo,
        boundary_nodes: &[NetId],
        inputs: ClusterInputs,
        solver: &dyn ClusterSolver,
    ) -> ClusterSolution {
        let mut nodes: Vec<NetId> = c.roots.iter().map(|&r| NetId(r)).collect();
        nodes.extend_from_slice(boundary_nodes);
        let resistors: Vec<ClusterResistor> = c
            .edges
            .iter()
            .map(|(a, b, ohms)| ClusterResistor {
                a: NetId(*a),
                b: NetId(*b),
                ohms: *ohms,
            })
            .collect();
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
    /// per-cluster pass is checked against. It shares [`project_root`] and
    /// [`decide_terminal`] — each rule is one function — and nothing else:
    /// no topology cache, no dirty scope, no cluster tables, no fan-out.
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

        // The declared terminals: path barriers, what no edge or element
        // unions through, and each a cluster of its own.
        let terminal_roots = self.terminal_roots(&root_of);
        let is_terminal = |root: usize| terminal_roots.binary_search(&root).is_ok();

        // Conduction clusters: identity merges are 0-ohm, conduction edges
        // between two non-terminal roots connect within a cluster without
        // merging identity, elements are membership edges among their
        // non-terminal nets (ends and control), as `build_topology` has it.
        let mut conduction = Dsu::new(n);
        for (i, &root) in root_of.iter().enumerate() {
            conduction.union(root, i);
        }
        for (a, b, _ohms) in &self.edges {
            let (ra, rb) = (root_of[*a], root_of[*b]);
            if !is_terminal(ra) && !is_terminal(rb) {
                conduction.union(ra, rb);
            }
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

        // What every terminal root holds, decided once from its sources in
        // canonical order. hash-order: keyed access only.
        let mut terminal_state: HashMap<usize, TerminalState> = HashMap::new();
        for &root in &terminal_roots {
            let state = decide_terminal(
                self.terminal_sources()
                    .filter(|source| root_of[source.net] == root)
                    .map(|source| source.drive),
                |volts| self.solve_fight(root, volts, solver),
            );
            terminal_state.insert(root, state);
        }

        // Sources per cluster in canonical order — a dense walk of the slot
        // table, terminal slots skipped — each beside the slot it came from;
        // injections likewise; the elements per cluster; the boundary
        // terminal roots per cluster (the edges' terminal ends and the
        // elements' conducting ends) and the control-only ones.
        //
        // **Determinism (load-bearing):** iterate the DENSE drive table. This
        // `Vec`'s order is the SPICE card order
        // [`crate::cluster::QuasiStaticMna::solve`] stamps, so a hash walk
        // would make the deck (and, for a linear solver, last-bit voltages)
        // depend on a per-process hasher seed. See `DETERMINISM.md`.
        // hash-order: every map below is keyed access only.
        let mut cluster_injections: HashMap<usize, Vec<(ClusterInjection, usize)>> = HashMap::new();
        let mut cluster_elements: HashMap<usize, Vec<(ClusterElement, usize)>> = HashMap::new();
        let mut cluster_boundary: HashMap<usize, Vec<usize>> = HashMap::new();
        let mut cluster_controls: HashMap<usize, Vec<usize>> = HashMap::new();
        for (a, b, _) in &root_edges {
            for (x, y) in [(*a, *b), (*b, *a)] {
                if !is_terminal(x) && is_terminal(y) {
                    let boundary = cluster_boundary.entry(cluster_of[x]).or_default();
                    if !boundary.contains(&y) {
                        boundary.push(y);
                    }
                }
            }
        }
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
                let boundary = cluster_boundary.entry(cluster).or_default();
                if !boundary.contains(&root) {
                    boundary.push(root);
                }
            }
            if let Some(root) = home.terminal_control {
                let controls = cluster_controls.entry(cluster).or_default();
                if !controls.contains(&root) {
                    controls.push(root);
                }
            }
        }
        for boundary in cluster_boundary.values_mut() {
            boundary.sort_unstable();
        }
        for (cluster, controls) in cluster_controls.iter_mut() {
            let boundary = cluster_boundary.get(cluster).cloned().unwrap_or_default();
            controls.retain(|root| !boundary.contains(root));
            controls.sort_unstable();
        }
        // A cluster a periodic drive sits in resolves once per phase (see
        // `resolve_cluster`); every other cluster once.
        // hash-order shape 3: membership only.
        let periodic_clusters: HashSet<usize> = self
            .slots
            .iter()
            .filter(|slot| {
                slot.terminal.is_none() && matches!(slot.drive, Some(Drive::Periodic { .. }))
            })
            .map(|slot| cluster_of[slot.net])
            .collect();
        // The slot sources of every cluster in one phase, in endpoint order
        // (a cluster no periodic drive sits in has the one list either way).
        let sources_in = |phase: Phase| -> HashMap<usize, Vec<(ClusterSource, usize)>> {
            let mut map: HashMap<usize, Vec<(ClusterSource, usize)>> = HashMap::new();
            for (si, slot) in self.slots.iter().enumerate() {
                if slot.terminal.is_some() {
                    continue;
                }
                let cluster = cluster_of[slot.net];
                let phase = periodic_clusters.contains(&cluster).then_some(phase);
                if let Some(port) = phase_port(slot.drive, phase) {
                    map.entry(cluster).or_default().push((
                        ClusterSource {
                            node: NetId(root_of[slot.net]),
                            volts: port.volts,
                            impedance: port.impedance,
                        },
                        si,
                    ));
                }
            }
            map
        };
        let sources_hi = sources_in(Phase::High);
        let sources_lo = sources_in(Phase::Low);
        let slot_sourced =
            |cluster: usize| sources_hi.contains_key(&cluster) || sources_lo.contains_key(&cluster);
        for (si, slot) in self.slots.iter().enumerate() {
            if slot.terminal.is_some() {
                continue;
            }
            if let Some(Drive::Current { amps }) = slot.drive {
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

        // The terminals a cluster reads, each as the state its root
        // holds: the cluster's own root when it is a terminal's, then its
        // boundary, then the controls' — the constants, the ideal sources
        // `(root, volts)` of the ranking, the unmodelled roots, whether
        // anything sources the cluster, the boundary nodes of its solve and
        // its own fight.
        struct ReadTerminals {
            constants: Vec<ClusterTerminal>,
            ideal: Vec<(usize, Volts)>,
            unmodelled: Vec<usize>,
            sourced: bool,
            boundary_nodes: Vec<NetId>,
            own_fight: Option<Volts>,
        }
        let read_terminals = |cluster: usize| -> ReadTerminals {
            let mut read = ReadTerminals {
                constants: Vec::new(),
                ideal: Vec::new(),
                unmodelled: Vec::new(),
                sourced: false,
                boundary_nodes: Vec::new(),
                own_fight: None,
            };
            let own: Vec<usize> = terminal_roots
                .iter()
                .copied()
                .filter(|&root| cluster_of[root] == cluster)
                .collect();
            let boundary = cluster_boundary.get(&cluster).cloned().unwrap_or_default();
            for (root, is_boundary) in own
                .iter()
                .map(|&r| (r, false))
                .chain(boundary.iter().map(|&r| (r, true)))
            {
                let state = terminal_state[&root];
                match state.drive {
                    TerminalDrive::Volts(volts) => {
                        read.constants.push(ClusterTerminal {
                            node: NetId(root),
                            volts,
                        });
                        read.ideal.push((root, volts));
                        read.sourced = true;
                        if is_boundary {
                            read.boundary_nodes.push(NetId(root));
                        } else if state.fought {
                            read.own_fight = Some(volts);
                        }
                    }
                    TerminalDrive::Unmodelled => {
                        read.unmodelled.push(root);
                        read.sourced = true;
                    }
                    TerminalDrive::Released => {}
                }
            }
            for root in cluster_controls.get(&cluster).cloned().unwrap_or_default() {
                if let TerminalDrive::Volts(volts) = terminal_state[&root].drive {
                    read.constants.push(ClusterTerminal {
                        node: NetId(root),
                        volts,
                    });
                }
            }
            read
        };

        let solve_cluster = |cluster: usize,
                             cluster_sources: &HashMap<usize, Vec<(ClusterSource, usize)>>|
         -> ClusterSolution {
            let read = read_terminals(cluster);
            let mut nodes: Vec<NetId> = (0..n)
                .filter(|&i| root_of[i] == i && cluster_of[i] == cluster)
                .map(NetId)
                .collect();
            nodes.extend(read.boundary_nodes.iter().copied());
            // An edge belongs to the cluster of its non-terminal end(s).
            let resistors: Vec<ClusterResistor> = root_edges
                .iter()
                .filter(|(a, b, _)| {
                    (cluster_of[*a] == cluster && !is_terminal(*a))
                        || (cluster_of[*b] == cluster && !is_terminal(*b))
                })
                .map(|(a, b, ohms)| ClusterResistor {
                    a: NetId(*a),
                    b: NetId(*b),
                    ohms: *ohms,
                })
                .collect();
            let inputs = ClusterInputs {
                sources: cluster_sources
                    .get(&cluster)
                    .map(|sources| sources.iter().map(|(s, _)| *s).collect())
                    .unwrap_or_default(),
                injections: cluster_injections
                    .get(&cluster)
                    .map(|injections| injections.iter().map(|(i, _)| *i).collect())
                    .unwrap_or_default(),
                terminals: read.constants,
                elements: cluster_elements
                    .get(&cluster)
                    .map(|elements| elements.iter().map(|(e, _)| *e).collect())
                    .unwrap_or_default(),
            };
            self.escalated_solves.fetch_add(1, Ordering::SeqCst);
            solver.solve(&Cluster { nodes, resistors }, &inputs)
        };
        let has_elements = |cluster: usize| -> bool { cluster_elements.contains_key(&cluster) };
        // An analog sense, an injection, an instrument or an element asks
        // for the operating point; rule 2's fights are reported beside it.
        let injected = |cluster: usize| -> bool {
            cluster_injections
                .get(&cluster)
                .is_some_and(|injections| injections.iter().any(|(i, _)| i.amps != 0.0))
        };
        // An analog reader's root one source reaches is handed that
        // source's open-circuit voltage unsolved (the single-source rule);
        // the rest solve eagerly.
        let sourced = |cluster: usize| -> bool {
            slot_sourced(cluster) || !read_terminals(cluster).ideal.is_empty()
        };
        let eager = |cluster: usize| -> bool {
            sourced(cluster)
                && (injected(cluster)
                    || instrument_clusters.contains(&cluster)
                    || has_elements(cluster))
        };
        let on_request = |cluster: usize| -> bool {
            eager(cluster) || (sourced(cluster) && analog_clusters.contains(&cluster))
        };
        // One phase over every root (or, for the low phase, over the roots
        // of the periodic clusters alone): each root's outcome and every
        // cluster the pass solved.
        // hash-order: `escalated` and the outcome map are keyed access
        // only (`entry`, `get`, index) — the walks that fill and read them
        // are over dense indices.
        let pass = |cluster_sources: &HashMap<usize, Vec<(ClusterSource, usize)>>,
                    periodic_only: bool|
         -> (HashMap<usize, PhaseRoot>, HashMap<usize, ClusterSolution>) {
            let mut escalated: HashMap<usize, ClusterSolution> = HashMap::new();
            let mut outcomes: HashMap<usize, PhaseRoot> = HashMap::new();
            for root in (0..n).filter(|&i| root_of[i] == i) {
                let cluster = cluster_of[root];
                if periodic_only && !periodic_clusters.contains(&cluster) {
                    continue;
                }
                let read = read_terminals(cluster);
                // A terminal's root reaches nothing out of its own cluster:
                // its paths are its dependents' to rank it by, not its own.
                let dist: HashMap<usize, f64> = if is_terminal(root) {
                    HashMap::from([(root, 0.0)])
                } else {
                    min_path_ohms(&root_edges, root, &terminal_roots)
                };
                let mut reaching: Vec<ReachingSource> = cluster_sources
                    .get(&cluster)
                    .map(|sources| {
                        sources
                            .iter()
                            .filter_map(|(source, slot)| {
                                let path = *dist.get(&source.node.0)?;
                                path.is_finite().then_some(ReachingSource {
                                    slot: Some(*slot),
                                    volts: source.volts,
                                    impedance: source.impedance,
                                    path,
                                })
                            })
                            .collect()
                    })
                    .unwrap_or_default();
                for &(terminal_root, volts) in &read.ideal {
                    let Some(&path) = dist.get(&terminal_root) else {
                        continue;
                    };
                    if path.is_finite() {
                        reaching.push(ReachingSource {
                            slot: None,
                            volts,
                            impedance: 0.0,
                            path,
                        });
                    }
                }
                let unmodelled_or_floating = || {
                    let nearest = read
                        .unmodelled
                        .iter()
                        .filter_map(|r| dist.get(r).copied())
                        .filter(|d| d.is_finite())
                        .fold(f64::INFINITY, f64::min);
                    if nearest.is_finite() {
                        NetState::Pulled(Level::High, nearest)
                    } else {
                        NetState::Floating
                    }
                };
                let mut outcome = if has_elements(cluster) && on_request(cluster) {
                    let solution = escalated
                        .entry(cluster)
                        .or_insert_with(|| solve_cluster(cluster, cluster_sources));
                    let ranked = if reaching.is_empty() {
                        None
                    } else {
                        let mut solved = || match solution.state_of(NetId(root)) {
                            Some(NetState::Analog(v)) => Some(v),
                            _ => None,
                        };
                        Some(project_root(&reaching, &mut solved))
                    };
                    let state = match solution.state_of(NetId(root)) {
                        Some(NetState::Analog(v)) => NetState::Analog(v),
                        _ if !solution.converged => NetState::Floating,
                        _ => unmodelled_or_floating(),
                    };
                    // The voltage is the solve's, as `resolve_cluster`
                    // names it: the phases combine on it.
                    RootOutcome {
                        state,
                        volts: RootOutcome::quiet(state).volts,
                        ..ranked.unwrap_or_else(|| RootOutcome::quiet(state))
                    }
                } else if reaching.is_empty() {
                    RootOutcome::quiet(unmodelled_or_floating())
                } else if let ([only], false, true) =
                    (reaching.as_slice(), eager(cluster), on_request(cluster))
                {
                    RootOutcome {
                        state: NetState::Analog(only.volts),
                        volts: Some(only.volts),
                        fight: None,
                        ambiguous: None,
                        solved: false,
                    }
                } else if on_request(cluster) {
                    let solution = escalated
                        .entry(cluster)
                        .or_insert_with(|| solve_cluster(cluster, cluster_sources));
                    let state = solution.state_of(NetId(root)).unwrap_or(NetState::Floating);
                    let mut solved = || match solution.state_of(NetId(root)) {
                        Some(NetState::Analog(v)) => Some(v),
                        _ => None,
                    };
                    RootOutcome {
                        state,
                        volts: RootOutcome::quiet(state).volts,
                        ..project_root(&reaching, &mut solved)
                    }
                } else {
                    let mut solved = || -> Option<Volts> {
                        match escalated
                            .entry(cluster)
                            .or_insert_with(|| solve_cluster(cluster, cluster_sources))
                            .state_of(NetId(root))
                        {
                            Some(NetState::Analog(v)) => Some(v),
                            _ => None,
                        }
                    };
                    project_root(&reaching, &mut solved)
                };
                // A fought terminal's own root, decided once (see
                // `resolve_cluster`).
                if let Some(volts) = read.own_fight {
                    let in_band = V_IL < volts && volts < V_IH;
                    outcome.fight = Some(outcome.fight.take().unwrap_or_default());
                    outcome.solved = false;
                    outcome.volts = Some(volts);
                    outcome.ambiguous = in_band.then_some(volts);
                    if !on_request(cluster) {
                        outcome.state = if in_band {
                            NetState::Contention
                        } else {
                            NetState::Analog(volts)
                        };
                    }
                }
                let is_periodic =
                    |si: usize| matches!(self.slots[si].drive, Some(Drive::Periodic { .. }));
                outcomes.insert(
                    root,
                    PhaseRoot::of(
                        outcome,
                        &reaching,
                        periodic_clusters
                            .contains(&cluster)
                            .then_some(&is_periodic as &dyn Fn(usize) -> bool),
                    ),
                );
            }
            (outcomes, escalated)
        };
        let (hi_roots, mut escalated) = pass(&sources_hi, false);
        let (lo_roots, escalated_lo) = pass(&sources_lo, true);
        // A cluster solved in both phases keeps a non-convergent solve, if
        // either was one (the finding below names it).
        for (cluster, solution) in escalated_lo {
            match escalated.get(&cluster) {
                Some(kept) if !kept.converged => {}
                _ => {
                    escalated.insert(cluster, solution);
                }
            }
        }
        let mut root_state: HashMap<usize, NetState> = HashMap::new();
        let mut root_fights: HashMap<usize, Vec<usize>> = HashMap::new();
        let mut root_ambiguous: HashMap<usize, Volts> = HashMap::new();
        for root in (0..n).filter(|&i| root_of[i] == i) {
            let hi = &hi_roots[&root];
            let (state, fight, ambiguous) = match lo_roots.get(&root) {
                Some(lo) => {
                    let (state, fight, ambiguous, _) =
                        combine_phases(hi, lo, |si| match self.slots[si].drive {
                            Some(Drive::Periodic { segment, .. }) => Some(segment),
                            _ => None,
                        });
                    (state, fight, ambiguous)
                }
                None => (hi.state, hi.fight.clone(), hi.ambiguous),
            };
            if let Some(fighting) = fight {
                root_fights.insert(root, fighting);
            }
            if let Some(volts) = ambiguous {
                root_ambiguous.insert(root, volts);
            }
            root_state.insert(root, state);
        }

        // The rates arriving across coupling capacitors (the AC rule, as
        // `resolve_cluster` applies it per cluster): every periodic slot's
        // reach walked afresh, in slot order.
        let root_couplings: Vec<(usize, usize, usize)> = self
            .couplings
            .iter()
            .enumerate()
            .map(|(ci, c)| (root_of[c.a], root_of[c.b], ci))
            .filter(|(a, b, _)| a != b)
            .collect();
        let smallest_edge_at = |root: usize| -> f64 {
            root_edges
                .iter()
                .filter(|(a, b, _)| *a == root || *b == root)
                .map(|(_, _, ohms)| *ohms)
                .fold(f64::INFINITY, f64::min)
        };
        let mut arrivals: Vec<(usize, Arrival)> = Vec::new();
        for (si, slot) in self.slots.iter().enumerate() {
            let Some(Drive::Periodic { hi, lo, segment }) = slot.drive else {
                continue;
            };
            if slot.terminal.is_some() {
                continue;
            }
            let from = root_of[slot.net];
            let reach = coupled_reach(&root_edges, &root_couplings, from, &terminal_roots);
            // hash-order shape 2: the reached roots are collected and sorted.
            let mut reached: Vec<(usize, Vec<(usize, usize)>)> = reach
                .into_iter()
                .filter(|(root, (ohms, path))| {
                    *ohms < COUPLED_REACH_OHMS
                        && !path.is_empty()
                        && !is_terminal(*root)
                        && cluster_of[*root] != cluster_of[from]
                })
                .map(|(root, (_, path))| (root, path))
                .collect();
            reached.sort_by_key(|(root, _)| *root);
            for (root, path) in reached {
                arrivals.push((
                    cluster_of[root],
                    Arrival {
                        root,
                        slot: si,
                        crossings: path
                            .iter()
                            .map(|&(ci, far_root)| CouplingCrossing {
                                capacitor: self.couplings[ci].reference.clone(),
                                far_root,
                                farads: self.couplings[ci].farads,
                                far_ohms: smallest_edge_at(far_root),
                            })
                            .collect(),
                        hi: level_of_volts(hi.volts),
                        lo: level_of_volts(lo.volts),
                        swing: Arrival::swing_of(hi, lo),
                        segment,
                    },
                ));
            }
        }
        let overlays = overlay_arrivals(
            &arrivals,
            |root| root_state[&root],
            |root| match lo_roots.get(&root) {
                Some(lo) => slot_union([
                    hi_roots[&root].contending.as_slice(),
                    lo.contending.as_slice(),
                ]),
                None => Vec::new(),
            },
            |root| nets[root].name.clone(),
        );
        for (root, state, _, fight) in overlays.states {
            root_state.insert(root, state);
            if let Some(fighting) = fight {
                let merged = match root_fights.get(&root) {
                    Some(existing) => slot_union([existing.as_slice(), fighting.as_slice()]),
                    None => fighting,
                };
                root_fights.insert(root, merged);
            }
        }
        let mut refused = overlays.refused;

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
            let sourced = slot_sourced(cluster) || read_terminals(cluster).sourced;
            let unsourced = !sourced
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
            if amps == 0.0 || slot.terminal.is_some() {
                continue;
            }
            let cluster = cluster_of[slot.net];
            let dist = min_path_ohms(&root_edges, root_of[slot.net], &terminal_roots);
            let reached = [&sources_hi, &sources_lo].iter().any(|phase| {
                phase.get(&cluster).is_some_and(|sources| {
                    sources
                        .iter()
                        .any(|(s, _)| dist.get(&s.node.0).is_some_and(|d| d.is_finite()))
                })
            }) || read_terminals(cluster)
                .ideal
                .iter()
                .any(|(root, _)| dist.get(root).is_some_and(|d| d.is_finite()));
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

        // Rates a coupling capacitor refused, by the far root.
        refused.sort_by_key(|(far_root, _)| *far_root);
        for (_, finding) in refused {
            diagnostics.report(finding);
        }
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

/// How a net moved in one pass, for the sense change gate
/// ([`EngineCore::deliver_senses`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum NetMove {
    /// Its state changed: the engine's report records it, and its senses
    /// are delivered.
    State,
    /// Only the voltage behind an unchanged state moved: its senses are
    /// delivered — what a pin is handed is the voltage — and the report
    /// records nothing.
    Volts,
}

impl NetMove {
    /// How `net` moved from `state` at `volts`, or `None` if it did not.
    pub(crate) fn of(state: &NetState, volts: &NetVolts, net: &Net) -> Option<Self> {
        if !same_state(state, &net.state) {
            Some(Self::State)
        } else if !same_volts(volts, &net.volts) {
            Some(Self::Volts)
        } else {
            None
        }
    }
}

/// NaN-robust voltage equality for the sense change gate: every figure
/// compared by `total_cmp`, as [`same_state`] compares an `Analog` state's.
pub(crate) fn same_volts(a: &NetVolts, b: &NetVolts) -> bool {
    fn same(x: Option<Volts>, y: Option<Volts>) -> bool {
        match (x, y) {
            (None, None) => true,
            (Some(x), Some(y)) => x.to_bits() == y.to_bits() || x.total_cmp(&y).is_eq(),
            _ => false,
        }
    }
    if !same(a.dc, b.dc) {
        return false;
    }
    match (a.phases, b.phases) {
        (None, None) => true,
        (Some((ah, al)), Some((bh, bl))) => same(ah, bh) && same(al, bl),
        _ => false,
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
pub(crate) fn same_state(a: &NetState, b: &NetState) -> bool {
    match (a, b) {
        (NetState::Analog(x), NetState::Analog(y)) => x.total_cmp(y).is_eq(),
        (NetState::Pulled(la, xa), NetState::Pulled(lb, xb)) => {
            la == lb && xa.total_cmp(xb).is_eq()
        }
        // A periodic state is time-varying, so "changed" means **the
        // segment changed** — compared by identity, anchor included, never
        // by the level the clock is at now — else every instant would be a
        // change and the engine would deliver one event per edge, which is
        // what the rate representation exists to avoid
        // (`sil-unified-drive.md`, "The sense change gate"). Every field is
        // an integer or a level, so the derived equality is exact.
        (NetState::Periodic { .. }, NetState::Periodic { .. }) => a == b,
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
    /// The voltage the node is handed (`NetVolts::dc`): the winner's
    /// open-circuit voltage where the ranking projected, the solved one
    /// where the root solved — a fight's operating point included — and
    /// `None` where nothing names one.
    volts: Option<Volts>,
    /// The strong sources on the root, as the slots to name in the
    /// `Contention` finding (a terminal has none), when they fought: a
    /// strong source disagreed and lost, or disagreeing strong sources
    /// solved.
    fight: Option<Vec<usize>>,
    /// The solved voltage, when it fell inside the dead band.
    ambiguous: Option<Volts>,
    /// The contest disagreed and the root was solved (rule 5 below), as
    /// opposed to projected from its strongest source.
    solved: bool,
}

impl RootOutcome {
    /// A state with nothing to report, and the voltage it names (an
    /// `Analog` state's; `None` for any other — the caller that knows the
    /// winner's voltage sets it).
    fn quiet(state: NetState) -> Self {
        Self {
            state,
            volts: match state {
                NetState::Analog(v) => Some(v),
                _ => None,
            },
            fight: None,
            ambiguous: None,
            solved: false,
        }
    }
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
    let quiet = RootOutcome::quiet;
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
            volts: Some(strongest.volts),
            fight: silenced.then(strong_slots),
            ambiguous: None,
            solved: false,
        };
    }
    match solved() {
        None => quiet(NetState::Floating),
        Some(volts) if !contest_is_strong => RootOutcome {
            solved: true,
            ..quiet(NetState::Analog(volts))
        },
        Some(volts) if V_IL < volts && volts < V_IH => RootOutcome {
            state: NetState::Contention,
            volts: Some(volts),
            fight: Some(strong_slots()),
            ambiguous: Some(volts),
            solved: true,
        },
        Some(volts) => RootOutcome {
            state: NetState::Analog(volts),
            volts: Some(volts),
            fight: Some(strong_slots()),
            ambiguous: None,
            solved: true,
        },
    }
}

// ============================================================
// Periodic drives: two phases, one state (rule 2, twice)
// ============================================================

/// The arrivals whose root is in cluster `cid`, from a list sorted by
/// cluster.
fn arrivals_in(arrivals: &[(usize, Arrival)], cid: usize) -> &[(usize, Arrival)] {
    let start = arrivals.partition_point(|(cluster, _)| *cluster < cid);
    let end = arrivals.partition_point(|(cluster, _)| *cluster <= cid);
    &arrivals[start..end]
}

/// What a cluster's pass reads from outside the cluster: every terminal's
/// decided state, and the rates arriving at its roots across coupling
/// capacitors.
#[derive(Clone, Copy)]
struct ClusterReads<'a> {
    terminal_states: &'a [TerminalState],
    arrivals: &'a [(usize, Arrival)],
}

/// What the coupling rule decided for a set of roots
/// ([`overlay_arrivals`]).
struct Overlays {
    /// `(root, state, volts, fight)`: the state each reached root publishes
    /// over its own, the voltage it names (the source's swing; none where
    /// rates fight), and the slots to name when rates fight on it.
    states: Vec<(usize, NetState, NetVolts, Option<Vec<usize>>)>,
    /// Crossings the AC rule refused, keyed by the far root.
    refused: Vec<(usize, Finding)>,
}

/// The AC-coupling rule of phase 2 (`NODES.md` §8), re-expressed for a
/// periodic **drive** (`sil-unified-drive.md` step 4): a periodic slot's
/// rate is carried across the coupling capacitors in its AC reach, and
/// every root it arrives at publishes [`NetState::Periodic`] with the
/// source's segment and phase levels **over** the state its own cluster
/// resolved — a capacitor blocks the DC bias and passes the swing, and the
/// receiver is told the rate and the swing, not a time-average.
///
/// - Each crossing is judged at the segment's rate: `1/(2π·f·C)` must be at
///   most the far node's resistance estimate over
///   [`COUPLING_REACTANCE_RATIO`], else the rate stops at that capacitor
///   with [`Finding::PeriodicNotCoupled`]. A held segment (no rate) crosses
///   unconditionally — a capacitor cannot refuse a stop, and the far side
///   must learn of it.
/// - **A fought far node clamps the crossing**: a root its own sources
///   resolved `Contention` takes no overlay.
/// - **Two rates on one root are contention**, whatever their segments —
///   two arriving across capacitors, or one arriving onto a root a periodic
///   slot of its own cluster drives strongly (`contending_on`) — naming
///   every slot. A periodic *pull* there (a self-biased stage's feedback
///   resistor) yields to the coupled rate, as a pull yields to a driver.
/// - A terminal is never reached ([`Resolver::ensure_reach`]).
fn overlay_arrivals(
    arrivals: &[(usize, Arrival)],
    state_of: impl Fn(usize) -> NetState,
    contending_on: impl Fn(usize) -> Vec<usize>,
    name_of: impl Fn(usize) -> String,
) -> Overlays {
    let mut refused: Vec<(usize, Finding)> = Vec::new();
    // The arrivals that crossed, by root in first-arrival order.
    let mut crossed: Vec<(usize, Vec<&Arrival>)> = Vec::new();
    'arrivals: for (_, arrival) in arrivals {
        if arrival.segment.freq_hz > 0 {
            let hz = f64::from(arrival.segment.freq_hz);
            for crossing in &arrival.crossings {
                let reactance_ohms = 1.0 / (2.0 * std::f64::consts::PI * hz * crossing.farads);
                if reactance_ohms > crossing.far_ohms / COUPLING_REACTANCE_RATIO {
                    refused.push((
                        crossing.far_root,
                        Finding::PeriodicNotCoupled {
                            net: name_of(crossing.far_root),
                            capacitor: crossing.capacitor.clone(),
                            hz: arrival.segment.freq_hz,
                            reactance_ohms,
                            far_ohms: crossing.far_ohms,
                        },
                    ));
                    continue 'arrivals;
                }
            }
        }
        if state_of(arrival.root) == NetState::Contention {
            continue;
        }
        match crossed.iter_mut().find(|(root, _)| *root == arrival.root) {
            Some((_, list)) => list.push(arrival),
            None => crossed.push((arrival.root, vec![arrival])),
        }
    }
    let states = crossed
        .into_iter()
        .map(|(root, list)| {
            let driving = contending_on(root);
            match list.as_slice() {
                [only] if driving.is_empty() => (
                    root,
                    NetState::Periodic {
                        hi: only.hi,
                        lo: only.lo,
                        segment: only.segment,
                    },
                    NetVolts {
                        dc: None,
                        phases: Some(only.swing),
                    },
                    None,
                ),
                _ => {
                    let slots: Vec<usize> = list.iter().map(|a| a.slot).collect();
                    let named = slot_union([slots.as_slice(), driving.as_slice()]);
                    (root, NetState::Contention, NetVolts::default(), Some(named))
                }
            }
        })
        .collect();
    Overlays { states, refused }
}

/// Which port of every periodic drive a resolution pass stands on
/// (`sil-unified-drive.md`, "What resolution has to learn": "solve twice,
/// once per phase" — the existing quasi-static projection and solve, run
/// once per phase, for a cluster a periodic drive sits in only).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Phase {
    High,
    Low,
}

/// The Thevenin port a slot's drive presents in `phase`: a Thevenin drive
/// is its own port in every phase; a periodic drive its high or low port,
/// or nothing where that port is released (a non-finite impedance — the
/// slot normalisation's released, per phase); an injection, or a released
/// slot, no port. `phase` is `None` only in a cluster no periodic drive
/// sits in.
fn phase_port(drive: Option<Drive>, phase: Option<Phase>) -> Option<TheveninDrive> {
    match (drive?, phase) {
        (Drive::Thevenin(port), _) => Some(port),
        (Drive::Periodic { hi, .. }, Some(Phase::High)) => Some(hi),
        (Drive::Periodic { lo, .. }, Some(Phase::Low)) => Some(lo),
        (Drive::Periodic { .. }, None) => {
            debug_assert!(false, "a periodic slot resolves in a phase");
            None
        }
        (Drive::Current { .. }, _) => None,
    }
    .filter(|port| port.impedance.is_finite())
}

/// What one phase decided for one root, with what [`combine_phases`] needs
/// of the ranking beside it.
#[derive(Debug, Clone, PartialEq)]
struct PhaseRoot {
    state: NetState,
    /// The voltage the phase resolved the root to ([`RootOutcome::volts`]).
    volts: Option<Volts>,
    fight: Option<Vec<usize>>,
    ambiguous: Option<Volts>,
    /// The phase was itself a fight: `Contention`, or disagreeing strong
    /// sources that solved.
    fought: bool,
    /// Every strong slot reaching the root in this phase — the names of a
    /// two-clock `Contention`.
    strong: Vec<usize>,
    /// The periodic slots that contend here: strong, and not losing to the
    /// strongest source by the ratio.
    contending: Vec<usize>,
    /// Every periodic slot reaching the root, with its total ohms in this
    /// phase.
    periodic: Vec<(usize, Ohms)>,
    /// A periodic slot is in this phase's contest ([`project_root`]'s steps
    /// 2 and 3: it does not lose to the strongest source by the ratio, and
    /// it is no pull against a strong one) — the phase's state is, in part,
    /// the periodic source's. Where no periodic slot is in either phase's
    /// contest the root is the static sources' ([`combine_phases`] rule 3),
    /// whatever a solve says the losing source's port does to its voltage.
    periodic_decides: bool,
}

impl PhaseRoot {
    /// The phase record of a rule-2 outcome over `reaching`. The ranking
    /// lists [`combine_phases`] reads are gathered only for a root of a
    /// cluster a periodic drive sits in (`is_periodic` given): a cluster
    /// without one resolves once and combines nothing, and pays nothing for
    /// them (`DESIGN.md` rule 8).
    fn of(
        outcome: RootOutcome,
        reaching: &[ReachingSource],
        is_periodic: Option<&dyn Fn(usize) -> bool>,
    ) -> Self {
        // A fight inside the dead band carries its ambiguous level whether
        // the root publishes `Contention` or — solved on request — the
        // operating point beside the finding.
        let fought = outcome.state == NetState::Contention
            || outcome.ambiguous.is_some()
            || (outcome.solved && outcome.fight.is_some());
        let Some(is_periodic) = is_periodic else {
            return Self {
                state: outcome.state,
                volts: outcome.volts,
                fight: outcome.fight,
                ambiguous: outcome.ambiguous,
                fought,
                strong: Vec::new(),
                contending: Vec::new(),
                periodic: Vec::new(),
                periodic_decides: false,
            };
        };
        let bar = reaching
            .iter()
            .map(ReachingSource::total)
            .fold(f64::INFINITY, f64::min);
        let loses =
            |s: &ReachingSource| s.total() > bar && s.total() >= bar * ESCALATION_IMPEDANCE_RATIO;
        let contends = |s: &ReachingSource| !s.is_pull() && !loses(s);
        // `project_root`'s contest, by the same bar: a pull is out of it
        // when the strongest source is not a pull.
        let contest_is_strong = bar < WEAK_DRIVE_OHMS;
        let in_contest = |s: &ReachingSource| !loses(s) && (!contest_is_strong || !s.is_pull());
        Self {
            state: outcome.state,
            volts: outcome.volts,
            fight: outcome.fight,
            ambiguous: outcome.ambiguous,
            fought,
            strong: reaching
                .iter()
                .filter(|s| !s.is_pull())
                .filter_map(|s| s.slot)
                .collect(),
            contending: reaching
                .iter()
                .filter(|s| contends(s))
                .filter_map(|s| s.slot)
                .filter(|&si| is_periodic(si))
                .collect(),
            periodic: reaching
                .iter()
                .filter_map(|s| s.slot.map(|si| (si, s.total())))
                .filter(|&(si, _)| is_periodic(si))
                .collect(),
            periodic_decides: reaching
                .iter()
                .filter(|s| in_contest(s))
                .filter_map(|s| s.slot)
                .any(is_periodic),
        }
    }
}

/// Sorted, deduplicated union of slot lists.
fn slot_union<'a>(lists: impl IntoIterator<Item = &'a [usize]>) -> Vec<usize> {
    let mut slots: Vec<usize> = lists.into_iter().flatten().copied().collect();
    slots.sort_unstable();
    slots.dedup();
    slots
}

/// Combine a root's two phases into the one state it publishes
/// (`sil-unified-drive.md`, "What resolution has to learn"), with the fight
/// to report and the ambiguous level beside it:
///
/// 1. **Two periodic sources contend** — each strong and within
///    [`ESCALATION_IMPEDANCE_RATIO`] of the strongest, in either phase —
///    `Contention`, whatever their segments say: phase is not modelled,
///    and two sources agreeing by construction is a wiring the reference
///    machine does not have (the note's one ambiguous row, decided the safe
///    way, here where the resolver makes it). The fight names every strong
///    slot on the root.
/// 2. **A phase that fought** — `Contention`, or disagreeing strong sources
///    that solved: a periodic source against a comparable static one — is
///    `Contention`, a sustained fight for half of every cycle, with both
///    phases' fights and the ambiguous level kept.
/// 3. **No periodic source decided the root** — none is in either phase's
///    contest ([`PhaseRoot::periodic_decides`]: it lost to a stronger
///    source by the ratio, or is a pull against a strong one) — **or the
///    phases agree on one state at one voltage**: the high phase's state.
///    A losing source decides nothing whether or not the cluster is
///    solved: a solve on request still moves the root by the losing port's
///    millivolts, and a ripple a stronger source holds is not a clock, so
///    the phases combine as the same wiring's do without a reader: to the
///    high phase's state (under a reader, its operating point).
/// 4. **A phase with no level** floats the root: a clock whose line floats
///    for half its cycle — an open-drain clock with no pull-up — is a
///    floating net, not a square wave.
/// 5. **Both phases at one voltage** (a `Driven` high against a `Pulled`
///    high, from one rail): no edge, no clock on this node — the high
///    phase's state.
/// 6. Otherwise the root is [`NetState::Periodic`] with each phase's level
///    — projected by the engine's own rule, [`crate::level_of`], for its
///    report, and possibly one level for both phases: a swing inside one
///    of the report's bands is still a swing, and whether a receiver sees
///    an edge in it is the receiver's projection of the two phase voltages
///    it is handed, never the report's (`NODES.md` §10, "the level is the
///    receiver's") — and the segment of the strongest periodic source that
///    reaches it (ties to the earliest slot). A pull follows the square
///    wave; a periodic pull against a stronger static source loses in both
///    phases (rule 3). A **held** segment (no rate: the source stopped)
///    rests at its low port — the pulse that ended left the line there —
///    so the root names the low phase's voltage as its DC voltage beside
///    the two phases and the segment: a level receiver reads the resting
///    level, a relay forwards the segment, and a consumer folds the final
///    count (`NODES.md` §12 item 5, the review's decisions).
///
/// The voltage it names ([`NetVolts`]) follows the state: the high phase's
/// where the state is the high phase's (rules 3, 5), each phase's where it
/// is periodic (rule 6; a held segment's low phase besides, as its DC
/// voltage), and none where the root has no single operating point —
/// fought for half of every cycle (rules 1, 2) — or floats (rule 4).
fn combine_phases(
    hi: &PhaseRoot,
    lo: &PhaseRoot,
    segment_of: impl Fn(usize) -> Option<PeriodicSchedule>,
) -> (NetState, Option<Vec<usize>>, Option<Volts>, NetVolts) {
    let none = NetVolts::default();
    let high = NetVolts::dc(hi.volts);
    let fight = match (&hi.fight, &lo.fight) {
        (None, None) => None,
        (a, b) => Some(slot_union(
            a.iter().chain(b.iter()).map(|slots| slots.as_slice()),
        )),
    };
    // Rule 1.
    if slot_union([hi.contending.as_slice(), lo.contending.as_slice()]).len() >= 2 {
        let named = slot_union([
            hi.strong.as_slice(),
            lo.strong.as_slice(),
            fight.as_deref().unwrap_or_default(),
        ]);
        return (NetState::Contention, Some(named), None, none);
    }
    // Rule 2.
    if hi.fought || lo.fought {
        return (
            NetState::Contention,
            Some(fight.unwrap_or_default()),
            hi.ambiguous.or(lo.ambiguous),
            none,
        );
    }
    let one_voltage = at_one_voltage(hi.volts, lo.volts);
    // Rule 3.
    let decided = hi.periodic_decides || lo.periodic_decides;
    if !decided || (one_voltage && same_state(&hi.state, &lo.state)) {
        return (hi.state, fight, None, high);
    }
    // Rule 4.
    let (Some(level_hi), Some(level_lo)) = (level_of(hi.state), level_of(lo.state)) else {
        return (NetState::Floating, fight, None, none);
    };
    // Rule 5.
    if one_voltage && level_hi == level_lo {
        return (hi.state, fight, None, high);
    }
    // Rule 6.
    let strongest = hi
        .periodic
        .iter()
        .chain(&lo.periodic)
        .min_by(|a, b| a.1.total_cmp(&b.1).then(a.0.cmp(&b.0)))
        .and_then(|&(si, _)| segment_of(si));
    match strongest {
        Some(segment) => (
            NetState::Periodic {
                hi: level_hi,
                lo: level_lo,
                segment,
            },
            fight,
            None,
            NetVolts {
                // A held segment rests at its low port.
                dc: (segment.freq_hz == 0).then_some(lo.volts).flatten(),
                phases: Some((hi.volts, lo.volts)),
            },
        ),
        None => (hi.state, fight, None, high),
    }
}

/// Whether two phases name one voltage: both none, or the same number
/// (bitwise, `-0.0` folded to `0.0`, as currents compare) — exact, so the
/// combination is deterministic.
fn at_one_voltage(a: Option<Volts>, b: Option<Volts>) -> bool {
    match (a, b) {
        (None, None) => true,
        (Some(a), Some(b)) => (a + 0.0).total_cmp(&(b + 0.0)).is_eq(),
        _ => false,
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
/// cluster boundary; this is that rule in the path matrix, and since phase
/// 4 the cluster split says the same — a terminal is a cluster of its own,
/// so a cluster's edges never lead past one anyway). Without it the
/// module's P59 pad, driven high through the P59 pull-down to ground, would
/// reach the core rail's feedback divider on the other side of ground and
/// rank there as a pull disagreeing with ground — a divider solve on every
/// MOSI edge of the ROM boot. `from` itself is never a barrier: a terminal's
/// own paths out are what its dependents rank it by (the AC reach walks from
/// a source's root; the cluster pass never walks from a terminal).
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

/// One sense subscription as the engine keeps it ([`Command::RegisterSense`]).
struct SenseSub {
    /// The net it reads.
    net: usize,
    /// The net of the reference it is measured against, on another net.
    reference: Option<usize>,
    /// The net of the supply its thresholds scale with, on a third net.
    supply: Option<usize>,
    callback: SenseCallback,
}

impl SenseSub {
    /// The nets beside its own whose moves re-deliver it: its reference,
    /// then its supply (never the same net twice — the registration drops
    /// a supply on the reference's net).
    fn dependencies(&self) -> impl Iterator<Item = usize> {
        self.reference.into_iter().chain(self.supply)
    }

    /// The lowest-numbered of its dependencies that `moved` — the one net
    /// whose walk delivers it, so a pass that moved both delivers it once.
    fn first_moved_dependency(&self, moved: impl Fn(usize) -> bool) -> Option<usize> {
        self.dependencies().filter(|&net| moved(net)).min()
    }
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
    /// The voltage per net, published with `states` ([`EngineLink::volts`]).
    volts: Arc<VoltsTable>,
    /// The currents the last solves produced, published beside `states`
    /// after every pass ([`CurrentTable`]).
    currents: Arc<Mutex<CurrentTable>>,
    /// Current instruments ([`Command::RegisterCurrent`]), in registration
    /// order, each with the value it was last delivered.
    current_subs: Vec<CurrentSub>,
    diagnostics: Arc<Mutex<Diagnostics>>,
    /// The subscriptions on each net (indices into `senses`), in
    /// registration order — dense by net index, so a delivery pass looks
    /// each moved net up without hashing.
    sense_subs: Vec<Vec<usize>>,
    /// Every sense subscription, in registration order: the net it reads,
    /// the nets it depends on beside it (its reference, its supply), and
    /// its callback. `sense_subs` and `dependent_subs` index into it.
    senses: Vec<SenseSub>,
    /// The nets one pass moved, reused pass to pass so a delivery pass
    /// allocates nothing ([`Self::deliver_senses`]).
    moves: Vec<(usize, NetMove)>,
    /// The nets a dirty pass resolves, and what each held before it —
    /// reused pass to pass like `moves`, so the per-edge path allocates
    /// neither.
    scope: Vec<usize>,
    old: Vec<(NetState, NetVolts)>,
    /// The subscriptions that depend on each net beside their own (indices
    /// into `senses`), in registration order: measured against it as their
    /// reference, or scaling their thresholds by it as their supply —
    /// re-delivered when it moves and their own net did not.
    dependent_subs: Vec<Vec<usize>>,
    /// Whether any subscription depends on a net beside its own: a pass
    /// with none skips the dependents' walk.
    any_dependent_subs: bool,
    // hash-order: every map below is **keyed access only** — `get`, `entry`,
    // `insert`, `contains_key`. None is iterated. Sense delivery walks
    // `self.nets` by index and the per-net callbacks are a `Vec` in
    // registration order. Adding an iteration over any of these needs a sort
    // (see the module's review rule).
    wake_subs: HashMap<usize, WakeCallback>,
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
        let mut old = std::mem::take(&mut self.old);
        old.clear();
        old.extend(self.nets.iter().map(|n| (n.state, n.volts)));
        let mut pass = Diagnostics::new();
        self.resolver
            .resolve(&mut self.nets, &mut pass, self.solver.as_ref());

        {
            let mut shared = self.states.lock().unwrap();
            shared.clear();
            shared.extend(self.nets.iter().map(|n| n.state));
            for (i, net) in self.nets.iter().enumerate() {
                self.volts.store(i, net.volts);
            }
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
        let mut moves = std::mem::take(&mut self.moves);
        moves.clear();
        moves.extend((0..self.nets.len()).filter_map(|i| {
            let net = &self.nets[i];
            let moved = match old.get(i) {
                None => Some(NetMove::State),
                Some((state, volts)) => NetMove::of(state, volts, net),
            }?;
            Some((i, moved))
        }));
        self.old = old;
        self.deliver_senses(&moves);
        self.moves = moves;
        if currents_published {
            self.deliver_currents();
        }
    }

    /// Deliver the senses one pass moved, `moves` ascending by net: for
    /// each net whose state changed its [`EngineEvent::NetResolved`]
    /// record, and for each net that moved at all — its state, or only the
    /// voltage behind it (a `Driven(High)` whose source moved from 3.3 V to
    /// 1.8 V) — every subscription on it, in registration order. Then the
    /// subscriptions that depend on a net that moved, whose own net did
    /// not — once each, at the first such net: a reference that moves
    /// moves what the pin is handed, and a supply that moves moves the
    /// thresholds it projects through. The engine's report records only a
    /// state change; a sense delivery is recorded every time.
    fn deliver_senses(&self, moves: &[(usize, NetMove)]) {
        for &(i, moved) in moves {
            let net = &self.nets[i];
            if moved == NetMove::State {
                self.event_log.record(|| EngineEvent::NetResolved {
                    net: NetId(i),
                    state: net.state,
                });
            }
            if let Some(subs) = self.sense_subs.get(i) {
                for &sub in subs {
                    self.deliver_sense(sub);
                }
            }
        }
        if !self.any_dependent_subs {
            return;
        }
        let moved = |net: usize| moves.binary_search_by_key(&net, |&(i, _)| i).is_ok();
        for &(net, _) in moves {
            let Some(subs) = self.dependent_subs.get(net) else {
                continue;
            };
            for &sub in subs {
                let sense = &self.senses[sub];
                if !moved(sense.net) && sense.first_moved_dependency(moved) == Some(net) {
                    self.deliver_sense(sub);
                }
            }
        }
    }

    /// Deliver one subscription its net's current state, recorded.
    fn deliver_sense(&self, sub: usize) {
        let SenseSub {
            net: i,
            reference,
            callback,
            ..
        } = &self.senses[sub];
        let net = &self.nets[*i];
        self.event_log.record(|| EngineEvent::SenseDelivered {
            net: NetId(*i),
            state: net.state,
        });
        let delivery = Delivery {
            state: net.state,
            node: net.volts,
            reference: reference.map(|r| self.nets[r].volts),
        };
        self.deliver_contained(CallbackKind::Sense, &net.name, || {
            callback(&delivery);
        });
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

    /// Publish the topology epoch — at spawn, and on any topology-affecting
    /// change (the topology seam, `Command::RegisterTopologyObserver`). The
    /// record keeps its historical name: it is the second line of every
    /// golden trace.
    fn publish_topology_epoch(&mut self) {
        let epoch = self.topology_epoch;
        self.event_log.record(|| EngineEvent::Reroute { epoch });
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
        let mut scope = std::mem::take(&mut self.scope);
        self.resolver.dirty_scope(self.nets.len(), &mut scope);
        if scope.is_empty() {
            self.scope = scope;
            return;
        }
        let mut old = std::mem::take(&mut self.old);
        old.clear();
        old.extend(
            scope
                .iter()
                .map(|&i| (self.nets[i].state, self.nets[i].volts)),
        );
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
            for &i in &scope {
                self.volts.store(i, self.nets[i].volts);
            }
        }
        let currents_published = self.publish_currents();
        self.merge_findings(&pass);

        // Changed nets in ascending index order, per-net callbacks in
        // registration order — the sequence the full pass records.
        let mut moves = std::mem::take(&mut self.moves);
        moves.clear();
        moves.extend(scope.iter().zip(&old).filter_map(|(&i, (state, volts))| {
            Some((i, NetMove::of(state, volts, &self.nets[i])?))
        }));
        self.scope = scope;
        self.old = old;
        self.deliver_senses(&moves);
        self.moves = moves;
        if currents_published {
            self.deliver_currents();
        }
    }

    /// Fire every wheel entry whose deadline has passed, in `(deadline,
    /// schedule)` order; returns how many fired.
    ///
    /// `now` is the instant the engine last advanced to, so a wake fires at
    /// its scheduled deadline.
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
            Command::RegisterSense {
                net,
                reference,
                supply,
                callback,
            } => {
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
                let reference = reference.filter(|r| r.0 < self.nets.len());
                let delivery = Delivery {
                    state,
                    node: self.nets.get(net.0).map(|n| n.volts).unwrap_or_default(),
                    reference: reference.map(|r| self.nets[r.0].volts),
                };
                // Deliver the current state once at registration, so e.g. a
                // floating ~RESET is reported before any traffic.
                self.event_log
                    .record(|| EngineEvent::SenseDelivered { net, state });
                self.deliver_contained(CallbackKind::Sense, &subscriber, || callback(&delivery));
                if net.0 < self.nets.len() {
                    let sub = self.senses.len();
                    let supply = supply
                        .filter(|s| s.0 < self.nets.len() && *s != net && Some(*s) != reference);
                    let sense = SenseSub {
                        net: net.0,
                        reference: reference.map(|r| r.0),
                        supply: supply.map(|s| s.0),
                        callback,
                    };
                    if self.sense_subs.len() < self.nets.len() {
                        self.sense_subs.resize_with(self.nets.len(), Vec::new);
                        self.dependent_subs.resize_with(self.nets.len(), Vec::new);
                    }
                    self.sense_subs[net.0].push(sub);
                    for dependency in sense.dependencies() {
                        self.dependent_subs[dependency].push(sub);
                        self.any_dependent_subs = true;
                    }
                    self.senses.push(sense);
                }
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
            // then one pass over the clusters they sit in reports what
            // floats under them (`Resolver::declare_reads`).
            if !self.pending_reads.is_empty() {
                let reads = std::mem::take(&mut self.pending_reads);
                self.resolver.declare_reads(&reads);
                self.resolve_and_publish_dirty();
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
    /// resolution pass runs synchronously *before* the thread starts, and the
    /// topology epoch is published right after it, so never-driven nets are
    /// reported by the time this returns — before any traffic. Both therefore
    /// land in `event_log` from the *calling* thread, which is still
    /// single-writer: the engine thread does not exist yet.
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
        let volts = Arc::new(VoltsTable::of(nets.iter().map(|n| n.volts)));
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
            volts: Arc::clone(&volts),
            currents: Arc::clone(&currents),
            current_subs: Vec::new(),
            diagnostics: Arc::clone(&diagnostics),
            sense_subs: Vec::new(),
            senses: Vec::new(),
            moves: Vec::new(),
            scope: Vec::new(),
            old: Vec::new(),
            dependent_subs: Vec::new(),
            any_dependent_subs: false,
            wake_subs: HashMap::new(),
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
        core.publish_topology_epoch();

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
                volts,
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

    /// Topology-observer seam (later slice): observe net-graph topology
    /// changes — what a coupled periodic drive's reach and a cluster's
    /// membership are recomputed from. The observer runs on the engine thread with no engine lock
    /// held; the current epoch is delivered once at registration, and the
    /// engine will notify on every future topology-affecting change (jumper
    /// toggles, fault injection, harness swaps) once live mutation lands.
    #[allow(dead_code)] // the topology-mutation slice consumes this seam
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
                volts: crate::net::NetVolts::default(),
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
            reference: None,
            supply: None,
            callback: Box::new(move |delivery: &Delivery| {
                sink.lock().unwrap().push(delivery.state)
            }),
        });
        log
    }

    /// A `PowerOut` slot's declared idle drive is recorded whole: a
    /// Thevenin idle keeps its impedance on the slot (the I-V port's
    /// record) while the terminal holds its open-circuit voltage, and the
    /// two 0 Ω encodings — released, unmodelled — are what the kinds mean.
    #[rstest]
    fn a_terminal_slots_idle_drive_keeps_its_declared_impedance() {
        behaviour!(Test {
            id: "engine.terminal-idle-drive-keeps-its-impedance",
            covers: Some("board/src/engine.rs#Resolver::add_terminal_endpoint"),
            given: "three power-out slots on three nets, declared idle at 3.3 volts behind 0.1 ohms, released, and unmodelled",
        });
        expect!(
            "impedance-on-the-slot",
            "the declared drive is recorded on its slot whole, impedance included, and the terminal holds the open-circuit voltage",
            "the impedance is recorded for the current port and never solved, so the terminal's voltage is the open-circuit one",
        );
        expect!(
            "released-and-unmodelled-encodings",
            "the released slot records no drive and its terminal is released; the unmodelled slot records the NaN-volt drive and its terminal is unmodelled",
        );
        let mut resolver = Resolver::new(3, Dsu::new(3));
        let declared = Drive::Thevenin(TheveninDrive {
            volts: 3.3,
            impedance: 0.1,
        });
        let e0 = resolver.add_terminal_endpoint(0, PinRef::new("U", "OUT"), Some(declared));
        let e1 = resolver.add_terminal_endpoint(
            1,
            PinRef::new("U", "OUT2"),
            TerminalDrive::Released.idle_slot_drive(),
        );
        let e2 = resolver.add_terminal_endpoint(
            2,
            PinRef::new("U", "OUT3"),
            TerminalDrive::Unmodelled.idle_slot_drive(),
        );
        assert_eq!(resolver.slots[e0.0].drive, Some(declared));
        assert_eq!(
            resolver
                .terminal_source(resolver.slots[e0.0].terminal.unwrap())
                .drive,
            TerminalDrive::Volts(3.3)
        );
        assert_eq!(resolver.slots[e1.0].drive, None);
        assert_eq!(
            resolver
                .terminal_source(resolver.slots[e1.0].terminal.unwrap())
                .drive,
            TerminalDrive::Released
        );
        match resolver.slots[e2.0].drive {
            Some(Drive::Thevenin(t)) => {
                assert!(t.volts.is_nan());
                assert_eq!(t.impedance, 0.0);
            }
            other => panic!("{other:?}"),
        }
        assert_eq!(
            resolver
                .terminal_source(resolver.slots[e2.0].terminal.unwrap())
                .drive,
            TerminalDrive::Unmodelled
        );
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
        let h0 = crate::component::PinHandle::wired(NetId(0), Some(e0), link.clone());
        let h1 = crate::component::PinHandle::wired(NetId(0), Some(e1), link);
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
            let hb = crate::component::PinHandle::wired(NetId(1), Some(e_b), link.clone());
            link.send(Command::RegisterSense {
                net: NetId(0),
                reference: None,
                supply: None,
                callback: Box::new(move |delivery: &Delivery| {
                    let state = delivery.state;
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
            let ha = crate::component::PinHandle::wired(NetId(0), Some(e_a2), link.clone());
            link.send(Command::RegisterSense {
                net: NetId(1),
                reference: None,
                supply: None,
                callback: Box::new(move |delivery: &Delivery| {
                    let state = delivery.state;
                    sink.lock().unwrap().push(state);
                    if state == NetState::Driven(Level::High) {
                        ha.set_drive(Some(high()));
                    }
                }),
            });
            log
        };

        let ha = crate::component::PinHandle::wired(NetId(0), Some(e_a), link);
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
            let pin = crate::component::PinHandle::wired(NetId(1), Some(e1), link.clone());
            link.send(Command::RegisterSense {
                net: NetId(0),
                reference: None,
                supply: None,
                callback: Box::new(move |_: &Delivery| {
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

    /// An analog sense (ADC input) reads its node's voltage; one source
    /// reaching it is its open-circuit voltage, unsolved (`DESIGN.md` rule
    /// 8). The same topology with a digital-only sense keeps the fast-path
    /// `Pulled` projection.
    #[rstest]
    fn analog_sense_escalates_sourced_cluster_but_pull_up_stays_pulled() {
        behaviour!(Test {
            id: "engine.analog-sense-escalates",
            covers: Some("board/src/engine.rs#Resolver::resolve_cluster"),
            given: "a 3.3 volt rail reaching an unloaded input through a 4.7 kilohm pull-up",
        });
        expect!(
            "analog-reads-solved",
            "an analog sense on the input reads the rail's full 3.3 volts exactly, and nothing is solved",
            "one source reaching a node is the node's voltage: no current flows where nothing else is connected",
        );
        expect!(
            "digital-stays-pulled",
            "a digital sense on the same input reads it as pulled high through the 4.7 kilohms",
            "a digital reader wants the pull-up view, which a numeric solve would replace with a bare voltage",
        );
        // 3.3 V rail —4.7 kΩ— AIN (no load: the rail's open-circuit voltage).
        let mut resolver = Resolver::new(2, Dsu::new(2));
        resolver.add_power_source(0, 3.3);
        resolver.add_edge(0, 1, 4_700.0);
        resolver.add_analog_sense(1);
        let mut net_table = nets(2);
        let mut diags = Diagnostics::new();
        resolver.resolve(&mut net_table, &mut diags, &QuasiStaticMna);
        assert_eq!(
            net_table[1].state,
            NetState::Analog(3.3),
            "analog sense must read the rail's open-circuit voltage"
        );
        assert_eq!(resolver.escalated_solves(), 0, "one source solves nothing");

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
            let pin = crate::component::PinHandle::wired(NetId(0), Some(e0), handle.link());
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
            let pin = crate::component::PinHandle::wired(NetId(0), Some(e0), handle.link());
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
            reference: None,
            supply: None,
            callback: Box::new(|_: &Delivery| panic!("component bug")),
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
        let sensed = handle.sense();
        assert_eq!((sensed.volts, sensed.periodic), (None, None));
        assert_eq!(handle.net_report(), NetState::Floating);

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
            /// `PowerOut` pins' nets: terminals a part drives through a
            /// slot ([`Resolver::add_terminal_endpoint`]), unmodelled at
            /// first.
            rails: Vec<usize>,
            /// Coupling capacitors `(a, b, farads)`: the AC paths a clock
            /// crosses, and the fan-out a clock's change dirties.
            couplings: Vec<(usize, usize, f64)>,
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
                // A clock: rail to rail, or a sink that releases high, held
                // or at one of two rates — so two on one root, a clock
                // against a static source, a clock through a pull, and a
                // rate a capacitor passes or refuses all occur.
                2 => (
                    prop_oneof![Just(25.0f64), Just(15_000.0), Just(f64::INFINITY)],
                    prop_oneof![Just(0u32), Just(1_000), Just(20_000_000)],
                )
                    .prop_map(|(hi_ohms, freq_hz)| Some(Drive::Periodic {
                        hi: TheveninDrive {
                            volts: 3.3,
                            impedance: hi_ohms,
                        },
                        lo: TheveninDrive {
                            volts: 0.0,
                            impedance: 25.0,
                        },
                        segment: PeriodicSchedule {
                            emitted: 0,
                            freq_hz,
                            total: None,
                            since_ns: 0,
                        },
                    })),
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
                    prop::collection::vec(0..n, 0..2),
                    prop::collection::vec(
                        (0..n, 0..n, prop_oneof![Just(10e-12f64), Just(100e-9)]),
                        0..3,
                    ),
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
                            rails,
                            couplings,
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
                            rails,
                            couplings,
                        },
                    )
            })
        }

        /// The resolver a spec builds, its slot endpoints, its terminal
        /// sources (the harness's and the faults', in canonical order) and
        /// its `PowerOut` slots.
        struct Built {
            resolver: Resolver,
            ids: Vec<EndpointId>,
            terminals: Vec<TerminalId>,
            rails: Vec<EndpointId>,
        }

        fn build(spec: &Spec) -> Built {
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
            let mut terminals: Vec<TerminalId> = Vec::new();
            for &(net, volts) in &spec.power {
                terminals.push(resolver.add_power_source(net, volts));
            }
            let rails: Vec<EndpointId> = spec
                .rails
                .iter()
                .enumerate()
                .map(|(i, &net)| {
                    resolver.add_terminal_endpoint(
                        net,
                        PinRef::new("U", format!("OUT{i}")),
                        TerminalDrive::Unmodelled.idle_slot_drive(),
                    )
                })
                .collect();
            for &(net, volts) in &spec.stuck {
                terminals.push(resolver.add_stuck_source(net, volts));
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
            for (i, &(a, b, farads)) in spec.couplings.iter().enumerate() {
                resolver.add_coupling(a, b, farads, format!("C{i}"));
            }
            Built {
                resolver,
                ids,
                terminals,
                rails,
            }
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
                given: "a random board of up to nine nets with shorts, resistors, coupling capacitors, drivers of assorted strengths, clocks, injections, rails, faults, senses, diodes, switched channels and regulator outputs, under random drive changes, terminal changes and rail publishes",
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

        /// One random change mid-run: a slot's drive; what a declared
        /// terminal source holds its net at (a harness supply re-set,
        /// released, or a fault's voltage moved); or a `PowerOut` pin's
        /// part driving its rail to a new voltage or releasing it.
        #[derive(Debug, Clone, Copy)]
        enum Op {
            Drive(usize, Option<Drive>),
            Terminal(usize, TerminalDrive),
            Rail(usize, Option<Drive>),
        }

        fn terminal_drive_strategy() -> impl Strategy<Value = TerminalDrive> {
            prop_oneof![
                Just(TerminalDrive::Released),
                Just(TerminalDrive::Unmodelled),
                Just(TerminalDrive::Volts(0.0)),
                Just(TerminalDrive::Volts(3.3)),
                Just(TerminalDrive::Volts(5.0)),
                Just(TerminalDrive::Volts(1.5)),
            ]
        }

        fn op_strategy() -> impl Strategy<Value = Op> {
            prop_oneof![
                4 => (0usize..8, drive_strategy()).prop_map(|(slot, drive)| Op::Drive(slot, drive)),
                1 => (0usize..8, terminal_drive_strategy())
                    .prop_map(|(terminal, drive)| Op::Terminal(terminal, drive)),
                1 => (0usize..8, drive_strategy()).prop_map(|(rail, drive)| Op::Rail(rail, drive)),
            ]
        }

        proptest! {
            #![proptest_config(ProptestConfig { cases: 2000, ..ProptestConfig::default() })]
            fn touched_cluster_cases(
                spec in spec_strategy(),
                ops in prop::collection::vec(op_strategy(), 1..12),
            ) {
                let Built {
                    resolver: mut incremental,
                    ids,
                    terminals,
                    rails,
                } = build(&spec);
                let mut full = build(&spec).resolver; // the OLD algorithm, one global pass
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

                for (step, op) in ops.into_iter().enumerate() {
                    let changed = match op {
                        Op::Drive(slot, drive) => {
                            let endpoint = ids[slot % ids.len()];
                            let _ = full.set_drive(endpoint, drive);
                            incremental.set_drive(endpoint, drive)
                        }
                        Op::Terminal(_, _) if terminals.is_empty() => continue,
                        Op::Terminal(terminal, drive) => {
                            let id = terminals[terminal % terminals.len()];
                            let _ = full.set_terminal(id, drive);
                            incremental.set_terminal(id, drive)
                        }
                        Op::Rail(_, _) if rails.is_empty() => continue,
                        Op::Rail(rail, drive) => {
                            let endpoint = rails[rail % rails.len()];
                            let _ = full.set_drive(endpoint, drive);
                            incremental.set_drive(endpoint, drive)
                        }
                    };

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
            let pin = crate::component::PinHandle::wired(NetId(1), Some(endpoint), handle.link());
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
        /// point on every root, and rule 2's fights are reported beside it;
        /// the two analog goldens (`nominal_analog_cluster`,
        /// `net_stuck_shared_node`) pin the same rule on the wire.
        #[rstest]
        fn an_analog_sense_reads_the_operating_point_of_a_fought_node() {
            behaviour!(Test {
                id: "engine.analog-sense-reads-fought-node",
                covers: Some("board/src/engine.rs#Resolver::resolve_cluster"),
                given: "two pins driving one net to opposite levels at 25 ohms, with an analog input on that net",
            });
            expect!(
                "voltage-delivered",
                "the net publishes the solved mid-rail voltage, 1.65 volts",
                "an analog reader is handed the operating point of its cluster, whatever the fight on it",
            );
            expect!(
                "fight-reported",
                "the fight is reported beside the voltage, naming both pins, with the mid-rail level as ambiguous",
                "a reader asking for the voltage does not hide a fault on the node it reads",
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
            assert!(
                contention_naming(
                    &diags,
                    &net_table[0].name,
                    &[PinRef::new("U1", "1"), PinRef::new("U2", "1")]
                ),
                "{:?}",
                diags.findings()
            );
            assert!(
                diags.findings().iter().any(|f| matches!(
                    f,
                    Finding::AmbiguousLevel { volts, .. } if (volts - 1.65).abs() < 1e-9
                )),
                "{:?}",
                diags.findings()
            );
            assert_eq!(diags.findings().len(), 2, "{:?}", diags.findings());
        }
    }

    /// A periodic drive resolves through rule 2 once per phase
    /// (`sil-unified-drive.md` steps 1–2): the resolution table of "What
    /// resolution has to learn", case by case, on the resolver alone.
    mod periodic {
        use super::*;

        const SEGMENT: PeriodicSchedule = PeriodicSchedule {
            emitted: 0,
            freq_hz: 8_192,
            total: None,
            since_ns: 1_000_000,
        };

        /// A clock swinging 0–3.3 V, each phase behind `hi_ohms` / `lo_ohms`.
        fn clock(hi_ohms: Ohms, lo_ohms: Ohms, segment: PeriodicSchedule) -> Option<Drive> {
            Some(Drive::Periodic {
                hi: TheveninDrive {
                    volts: 3.3,
                    impedance: hi_ohms,
                },
                lo: TheveninDrive {
                    volts: 0.0,
                    impedance: lo_ohms,
                },
                segment,
            })
        }

        fn level(volts: Volts, impedance: Ohms) -> Option<Drive> {
            Some(Drive::Thevenin(TheveninDrive { volts, impedance }))
        }

        /// The `Contention` findings a pass reported, as `(net, pins)`.
        fn fights(diags: &Diagnostics) -> Vec<(String, Vec<PinRef>)> {
            diags
                .findings()
                .iter()
                .filter_map(|f| match f {
                    Finding::Contention { net, drivers } => Some((net.clone(), drivers.clone())),
                    _ => None,
                })
                .collect()
        }

        /// Resolve one net holding `drives`, returning its state, the pass's
        /// findings and the solves it cost.
        fn resolve_one(drives: &[(&str, Option<Drive>)]) -> (NetState, Diagnostics, u64) {
            let mut resolver = Resolver::new(1, Dsu::new(1));
            for (pin, drive) in drives {
                resolver.add_endpoint_with(0, PinRef::new("U1", *pin), *drive);
            }
            let mut net_table = nets(1);
            let mut diags = Diagnostics::new();
            resolver.resolve(&mut net_table, &mut diags, &QuasiStaticMna);
            (net_table[0].state, diags, resolver.escalated_solves())
        }

        #[rstest]
        fn a_clock_against_a_pull_up_is_a_square_wave_carrying_its_segment() {
            behaviour!(Test {
                id: "engine.clock-follows-through-a-pull",
                covers: Some("board/src/engine.rs#combine_phases"),
                given: "a pin clocking a net rail to rail at 25 ohms, 8192 pulses a second, and a 10 kilohm pull-up on the same net",
            });
            expect!(
                "square-wave",
                "the net is a square wave between the two levels, carrying the clock's rate, count and start instant",
                "each phase is ranked like any drive, and a pull sets a level only where nothing stronger reaches",
            );
            expect!("nothing-reported", "nothing is reported");
            expect!("zero-solves", "the pass costs no cluster solve");
            let (state, diags, solves) = resolve_one(&[
                ("STEP", clock(25.0, 25.0, SEGMENT)),
                ("PU", level(3.3, 10_000.0)),
            ]);
            assert_eq!(
                state,
                NetState::Periodic {
                    hi: Level::High,
                    lo: Level::Low,
                    segment: SEGMENT,
                }
            );
            assert!(diags.is_empty(), "{:?}", diags.findings());
            assert_eq!(solves, 0);
        }

        /// A step line fought by a stuck driver: the fight the rate could
        /// not show while it rode a channel beside the net.
        #[rstest]
        #[case::stuck_low_at_equal_strength(0.0, 25.0, Some(1.65))]
        #[case::stuck_high_at_a_quarter_strength(3.3, 100.0, None)]
        fn a_clock_fought_by_a_comparable_static_driver_is_contention(
            #[case] stuck_volts: Volts,
            #[case] stuck_ohms: Ohms,
            #[case] ambiguous: Option<Volts>,
        ) {
            behaviour!(Test {
                id: "engine.fought-clock-is-contention",
                covers: Some("board/src/engine.rs#combine_phases"),
                given: "a pin clocking a net rail to rail at 25 ohms, and a second pin holding the same net at one rail within a factor of ten of the clock's strength",
            });
            expect!(
                "contention",
                "the net is in contention",
                "the two disagree for half of every cycle, and a sustained fight has no clean level to carry the clock on",
            );
            expect!(
                "both-named",
                "exactly one contention finding is reported on the net, naming both pins"
            );
            expect!(
                "one-solve",
                "the pass costs exactly one cluster solve, for the phase in which they disagree"
            );
            let (state, diags, solves) = resolve_one(&[
                ("STEP", clock(25.0, 25.0, SEGMENT)),
                ("STUCK", level(stuck_volts, stuck_ohms)),
            ]);
            assert_eq!(state, NetState::Contention);
            assert_eq!(
                fights(&diags),
                vec![(
                    "N0".to_string(),
                    vec![PinRef::new("U1", "STEP"), PinRef::new("U1", "STUCK")]
                )]
            );
            let reported_ambiguous = diags.findings().iter().find_map(|f| match f {
                Finding::AmbiguousLevel { volts, .. } => Some(*volts),
                _ => None,
            });
            match (ambiguous, reported_ambiguous) {
                (Some(expected), Some(volts)) => assert!((volts - expected).abs() < 1e-9),
                (None, None) => {}
                other => panic!("ambiguous level {other:?}"),
            }
            assert_eq!(solves, 1);
        }

        /// Two clocks on one root are contention whatever their segments
        /// say — phase is not modelled.
        #[rstest]
        #[case::same_segment(SEGMENT)]
        #[case::different_rate(PeriodicSchedule { freq_hz: 16_384, ..SEGMENT })]
        fn two_clocks_on_one_net_are_contention(#[case] other: PeriodicSchedule) {
            behaviour!(Test {
                id: "engine.two-clocks-contend",
                covers: Some("board/src/engine.rs#combine_phases"),
                given: "two pins clocking one net rail to rail at 25 ohms each, at the same rate from the same instant or at different rates",
            });
            expect!(
                "contention",
                "the net is in contention",
                "the phase of a clock is not modelled, so two clocks agreeing by construction cannot be told from two clocks fighting",
            );
            expect!(
                "both-named",
                "exactly one contention finding is reported on the net, naming both clock pins"
            );
            let (state, diags, _) = resolve_one(&[
                ("A", clock(25.0, 25.0, SEGMENT)),
                ("B", clock(25.0, 25.0, other)),
            ]);
            assert_eq!(state, NetState::Contention);
            assert_eq!(
                fights(&diags),
                vec![(
                    "N0".to_string(),
                    vec![PinRef::new("U1", "A"), PinRef::new("U1", "B")]
                )]
            );
        }

        /// An open-drain clock: a sink that releases in its high phase.
        #[rstest]
        #[case::no_pull_up(None, NetState::Floating)]
        #[case::pulled_up(
            Some(4_700.0),
            NetState::Periodic { hi: Level::High, lo: Level::Low, segment: SEGMENT }
        )]
        fn a_clock_that_releases_high_needs_a_pull_up(
            #[case] pull_up: Option<Ohms>,
            #[case] expected: NetState,
        ) {
            behaviour!(Test {
                id: "engine.open-drain-clock-needs-a-pull-up",
                covers: Some("board/src/engine.rs#combine_phases"),
                given: "a pin sinking a net at 25 ohms in its low phase and releasing it in its high phase, with or without a 4.7 kilohm pull-up",
            });
            expect!(
                "floats-without-pull-up",
                "with no pull-up the net floats",
                "for half of every cycle nothing sources the net, so it has no level to carry the clock on",
            );
            expect!(
                "square-wave-with-pull-up",
                "with the pull-up the net is a square wave carrying the clock's segment"
            );
            let mut drives = vec![("SCL", clock(f64::INFINITY, 25.0, SEGMENT))];
            if let Some(ohms) = pull_up {
                drives.push(("PU", level(3.3, ohms)));
            }
            let (state, diags, _) = resolve_one(&drives);
            assert_eq!(state, expected);
            assert!(fights(&diags).is_empty(), "{:?}", diags.findings());
        }

        /// A clock ten times weaker than a static driver loses in the phase
        /// it disagrees in, and the net is the static driver's.
        #[rstest]
        fn a_clock_outvoted_by_a_far_stronger_driver_is_that_drivers_level() {
            behaviour!(Test {
                id: "engine.outvoted-clock-carries-nothing",
                covers: Some("board/src/engine.rs#combine_phases"),
                given: "a pin clocking a net rail to rail at 25 ohms, and a second pin holding the net high at 2 ohms",
            });
            expect!(
                "driven-high",
                "the net is driven high",
                "a source ten times weaker than the strongest loses, so the clock never takes the net low",
            );
            expect!(
                "fight-reported",
                "exactly one contention finding is reported on the net, naming both pins"
            );
            expect!("zero-solves", "the pass costs no cluster solve");
            let (state, diags, solves) = resolve_one(&[
                ("STEP", clock(25.0, 25.0, SEGMENT)),
                ("HOLD", level(3.3, 2.0)),
            ]);
            assert_eq!(state, NetState::Driven(Level::High));
            assert_eq!(
                fights(&diags),
                vec![(
                    "N0".to_string(),
                    vec![PinRef::new("U1", "STEP"), PinRef::new("U1", "HOLD")]
                )]
            );
            assert_eq!(solves, 0);
        }

        /// A clock that decides neither phase decides nothing under a
        /// reader either: the solve moves the net by the losing port's
        /// millivolts, and that ripple is no clock.
        #[rstest]
        #[case::ten_times_weaker_driver(25.0, 2.0)]
        #[case::pull_against_a_driver(10_000.0, 25.0)]
        #[case::pull_inside_the_ratio(1_000.0, 200.0)]
        fn a_losing_clock_under_a_reader_is_the_static_drivers_net(
            #[case] clock_ohms: Ohms,
            #[case] hold_ohms: Ohms,
        ) {
            behaviour!(Test {
                id: "engine.losing-clock-under-a-reader-carries-nothing",
                covers: Some("board/src/engine.rs#combine_phases"),
                given: "a pin reading the voltage of a net one pin clocks rail to rail and a second pin holds high, the holder ten or more times stronger or the clock a pull against it",
            });
            expect!(
                "driven-high-unread",
                "with no pin reading its voltage the net is driven high"
            );
            expect!(
                "steady-voltage",
                "the reading pin is handed one steady voltage and no clock, exactly the one it reads with the clock's pin held at its high port",
                "in its low phase the losing clock still moves the solved net by millivolts, and a ripple the stronger pin holds is no clock for a receiver to count",
            );
            expect!(
                "same-findings",
                "the contention findings are the ones the same wiring reports with no pin reading the net"
            );
            let resolve = |step: Option<Drive>, read: bool| {
                let mut resolver = Resolver::new(1, Dsu::new(1));
                resolver.add_endpoint_with(0, PinRef::new("U1", "STEP"), step);
                resolver.add_endpoint_with(0, PinRef::new("U1", "HOLD"), level(3.3, hold_ohms));
                if read {
                    resolver.add_analog_sense(0);
                }
                let mut net_table = nets(1);
                let mut diags = Diagnostics::new();
                resolver.resolve(&mut net_table, &mut diags, &QuasiStaticMna);
                (net_table[0].state, net_table[0].volts, fights(&diags))
            };
            let step = clock(clock_ohms, clock_ohms, SEGMENT);
            let (unread, _, unread_fights) = resolve(step, false);
            assert_eq!(unread, NetState::Driven(Level::High));
            let (state, volts, read_fights) = resolve(step, true);
            let held_high = resolve(level(3.3, clock_ohms), true);
            assert!(
                !matches!(state, NetState::Periodic { .. }),
                "{state:?} {volts:?}"
            );
            assert_eq!((state, volts), (held_high.0, held_high.1));
            assert_eq!(volts.phases, None);
            assert_eq!(read_fights, unread_fights);
        }

        /// A clock through a series resistor still reaches the far node,
        /// which a pull there follows.
        #[rstest]
        fn a_clock_crosses_a_series_resistor() {
            behaviour!(Test {
                id: "engine.clock-crosses-a-resistor",
                covers: Some("board/src/engine.rs#combine_phases"),
                given: "a pin clocking a net rail to rail at 25 ohms, a 47 ohm resistor from that net to a second one, and a 10 kilohm pull-down on the second",
            });
            expect!(
                "square-wave-beyond",
                "the second net is a square wave carrying the clock's segment",
                "each phase reaches the far node through the resistor like any drive",
            );
            let mut resolver = Resolver::new(2, Dsu::new(2));
            resolver.add_endpoint_with(0, PinRef::new("U1", "STEP"), clock(25.0, 25.0, SEGMENT));
            resolver.add_endpoint_with(1, PinRef::new("R2", "1"), level(0.0, 10_000.0));
            resolver.add_edge(0, 1, 47.0);
            let mut net_table = nets(2);
            let mut diags = Diagnostics::new();
            resolver.resolve(&mut net_table, &mut diags, &QuasiStaticMna);
            assert_eq!(
                net_table[1].state,
                NetState::Periodic {
                    hi: Level::High,
                    lo: Level::Low,
                    segment: SEGMENT,
                }
            );
            assert!(diags.is_empty(), "{:?}", diags.findings());
        }

        /// A swing the report's own projection reads as one level is still
        /// a swing: the net carries the clock and both phases' voltages,
        /// and the receiver decides whether it sees an edge.
        #[rstest]
        fn a_clock_inside_one_report_band_is_still_a_clock() {
            behaviour!(Test {
                id: "engine.low-swing-clock-stays-a-clock",
                covers: Some("board/src/engine.rs#combine_phases"),
                given: "a pin clocking a net between 0 volts and 1.2 volts at 25 ohms, both below the net-level 1.5 volt split, 8192 pulses a second",
            });
            expect!(
                "clock-carried",
                "the net carries the clock's segment, reported low in both phases",
                "the report's levels are the engine's; whether a receiver sees an edge is its own thresholds' call",
            );
            expect!(
                "both-phase-voltages",
                "a sensing pin is handed 1.2 volts for the high phase and 0 volts for the low phase, and no single voltage"
            );
            let mut resolver = Resolver::new(1, Dsu::new(1));
            resolver.add_endpoint_with(
                0,
                PinRef::new("U1", "CLK"),
                Some(Drive::Periodic {
                    hi: TheveninDrive {
                        volts: 1.2,
                        impedance: 25.0,
                    },
                    lo: TheveninDrive {
                        volts: 0.0,
                        impedance: 25.0,
                    },
                    segment: SEGMENT,
                }),
            );
            let mut net_table = nets(1);
            let mut diags = Diagnostics::new();
            resolver.resolve(&mut net_table, &mut diags, &QuasiStaticMna);
            assert_eq!(
                net_table[0].state,
                NetState::Periodic {
                    hi: Level::Low,
                    lo: Level::Low,
                    segment: SEGMENT,
                }
            );
            assert_eq!(
                net_table[0].volts,
                NetVolts {
                    dc: None,
                    phases: Some((Some(1.2), Some(0.0))),
                }
            );
        }

        /// A stopped clock rests at its low port and still carries its
        /// final count.
        #[rstest]
        fn a_held_clock_rests_at_its_low_port() {
            behaviour!(Test {
                id: "engine.held-clock-rests-low",
                covers: Some("board/src/engine.rs#combine_phases"),
                given: "a pin whose clock has stopped after 400 pulses, its high port 3.3 volts and its low port 0 volts at 25 ohms",
            });
            expect!(
                "count-carried",
                "the net still carries the stopped segment and its count of 400"
            );
            expect!(
                "rests-low",
                "a sensing pin is handed the low port's 0 volts as the net's voltage, beside both ports' voltages",
                "the pulse that ended left the line at its low port, so a level receiver reads the line's resting level",
            );
            let held = PeriodicSchedule {
                emitted: 400,
                freq_hz: 0,
                total: Some(400),
                since_ns: 2_000_000,
            };
            let mut resolver = Resolver::new(1, Dsu::new(1));
            resolver.add_endpoint_with(0, PinRef::new("U1", "STEP"), clock(25.0, 25.0, held));
            let mut net_table = nets(1);
            let mut diags = Diagnostics::new();
            resolver.resolve(&mut net_table, &mut diags, &QuasiStaticMna);
            assert_eq!(
                net_table[0].state,
                NetState::Periodic {
                    hi: Level::High,
                    lo: Level::Low,
                    segment: held,
                }
            );
            assert_eq!(
                net_table[0].volts,
                NetVolts {
                    dc: Some(0.0),
                    phases: Some((Some(3.3), Some(0.0))),
                }
            );
        }

        /// A rail is not a clock: a periodic drive on a rail's terminal
        /// releases it, and nothing crosses a capacitor from it.
        #[rstest]
        fn a_rail_driven_periodic_is_released_and_couples_nothing() {
            behaviour!(Test {
                id: "engine.rail-is-not-a-clock",
                covers: Some("board/src/engine.rs#TerminalDrive::from_slot"),
                given: "a regulator output driven with a 0-to-3.3 volt clock at 8192 pulses a second, coupled through 1 microfarad to a node a 10 kilohm resistor pulls to 0 volts",
            });
            expect!(
                "rail-released",
                "the rail holds nothing: its terminal is released",
                "a current into a terminal is not a rail and a rail is not a clock, so both encodings release it",
            );
            expect!(
                "nothing-coupled",
                "the far node is the pull-down's low, and no clock arrives across the capacitor"
            );
            let mut resolver = Resolver::new(2, Dsu::new(2));
            let rail = resolver.add_terminal_endpoint(
                0,
                PinRef::new("U1", "OUT"),
                TerminalDrive::Released.idle_slot_drive(),
            );
            resolver.add_endpoint_with(1, PinRef::new("R1", "1"), level(0.0, 10_000.0));
            resolver.add_coupling(0, 1, 1e-6, "C1".to_string());
            let mut net_table = nets(2);
            let mut diags = Diagnostics::new();
            resolver.resolve(&mut net_table, &mut diags, &QuasiStaticMna);
            resolver.set_drive(rail, clock(0.0, 0.0, SEGMENT));
            let mut diags = Diagnostics::new();
            resolver.resolve(&mut net_table, &mut diags, &QuasiStaticMna);
            assert_eq!(
                resolver
                    .terminal_source(resolver.slots[rail.0].terminal.unwrap())
                    .drive,
                TerminalDrive::Released
            );
            assert!(
                matches!(net_table[1].state, NetState::Pulled(Level::Low, _)),
                "{:?}",
                net_table[1].state
            );
            assert!(
                !diags
                    .findings()
                    .iter()
                    .any(|f| matches!(f, Finding::PeriodicNotCoupled { .. })),
                "{:?}",
                diags.findings()
            );
        }

        /// The sense change gate compares a periodic state by its segment,
        /// anchor included.
        #[rstest]
        fn a_periodic_state_changes_only_when_its_segment_does() {
            behaviour!(Test {
                id: "engine.periodic-change-is-a-new-segment",
                covers: Some("board/src/engine.rs#same_state"),
                given: "a net carrying a clock segment, compared with itself, with the same segment re-anchored one microsecond later, and with the same segment at a new rate",
            });
            expect!(
                "same-segment-unchanged",
                "the same segment is no change, however much time has passed",
                "a clock's state is its schedule, so time passing within a segment delivers nothing",
            );
            expect!(
                "new-anchor-or-rate-changes",
                "a segment with a new start instant or a new rate is a change"
            );
            let state = |segment| NetState::Periodic {
                hi: Level::High,
                lo: Level::Low,
                segment,
            };
            assert!(same_state(&state(SEGMENT), &state(SEGMENT)));
            assert!(!same_state(
                &state(SEGMENT),
                &state(PeriodicSchedule {
                    since_ns: SEGMENT.since_ns + 1,
                    ..SEGMENT
                })
            ));
            assert!(!same_state(
                &state(SEGMENT),
                &state(PeriodicSchedule {
                    freq_hz: 1,
                    ..SEGMENT
                })
            ));
        }
    }
}
