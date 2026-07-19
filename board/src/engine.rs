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
//! wheel keyed to `embsim_core::virtual_clock` **virtual time**. The clock is
//! free-running scaled wall time (no step/pause API), so wakeup timestamps
//! are *sampled*, not deterministic: late wakeups fire immediately, in
//! deadline order, and missed periodic deadlines coalesce (one catch-up fire,
//! then back on period) — time-dependent component state must be computed at
//! read time, never integrated per tick. Idle components cost nothing — the
//! engine parks on its command queue (`recv_timeout`) until the next wheel
//! deadline, or indefinitely when the wheel is empty.
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
//! **Stream routing** (`BOARD_ENGINE.md`, "Stream endpoints (serial over
//! pins)") is engine-owned and **derived from net resolution, never
//! installed beside it**: the shared `Resolver` routes each stream
//! `Producer` to the `Consumer`s reachable through its net and through
//! series passives whose accumulated resistance stays below
//! [`STREAM_COLLAPSE_THRESHOLD`] (the DS2Addon's 47 Ω series resistors
//! collapse into the link). The routing pass runs at build, at engine spawn,
//! and on any topology-affecting change; two producers reachable from each
//! other raise [`Finding::StreamMismatch`] and neither routes. Producer
//! bytes are enqueued (`Command::StreamWrite`), paced at the producer's
//! declared baud against virtual time (10 bits/byte, the embsim 8N1 serial
//! convention — paced streams therefore require `virtual_clock::init`), and
//! delivered from the engine thread with no lock held. Delivery is gated by
//! the live state of the nets the link spans: bytes written into a broken
//! route — or one resolving `Contention`/`Floating` — are dropped with a
//! trace, never queued forever. Paced routes cap their in-flight queue
//! (`STREAM_ROUTE_QUEUE_MAX`) and shed the overflow with a
//! [`Finding::StreamOverrun`].
//!
//! **Failure containment**: component-provided callbacks (sense, wake,
//! stream-byte, topology) are panic-contained — a panic is reported as a
//! [`Finding::CallbackPanic`] and the engine stays alive, so one
//! misbehaving component never silently ends net service for the rest of
//! the system. Requests that need the virtual clock before
//! `virtual_clock::init` has run are dropped with a
//! [`Finding::VirtualClockUninitialized`] instead of panicking the engine
//! thread, and [`EngineHandle::is_alive`] reports engine-thread health.

use std::cmp::Reverse;
use std::collections::{BTreeMap, BinaryHeap, HashMap, HashSet, VecDeque};
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc::{self, Receiver, RecvTimeoutError, Sender};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use embsim_core::virtual_clock;

use crate::cluster::{
    Cluster, ClusterInputs, ClusterResistor, ClusterSolution, ClusterSolver, ClusterSource,
};
use crate::component::StreamRole;
use crate::diagnostics::{CallbackKind, Diagnostics, Finding, SenseKind};
use crate::net::{
    Level, Net, NetId, NetState, Ohms, PinRef, TheveninDrive, Volts, ESCALATION_IMPEDANCE_RATIO,
    STREAM_COLLAPSE_THRESHOLD,
};
use crate::system::StreamDropPolicy;

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

/// Bits clocked per byte for stream baud pacing (8N1: 1 start + 8 data +
/// 1 stop), matching the embsim serial peripheral's pacing convention.
const STREAM_BITS_PER_BYTE: u64 = 10;

/// Maximum queued commands handled per drain batch before control returns
/// to the engine loop's timer check. Enqueuing a command costs one atomic
/// increment plus a channel send — orders of magnitude cheaper than
/// handling one (a `Drive` runs a full resolution pass) — so one busy
/// producer thread can keep the queue non-empty indefinitely. An unbounded
/// drain would therefore starve the timer wheel system-wide
/// (`schedule_at`/`schedule_every` wakes and paced stream delivery would
/// never fire); time-driven behavior is engine-owned (`BOARD_ENGINE.md`),
/// so the wheel must be serviced within a bounded amount of queued work.
const COMMAND_DRAIN_BATCH_MAX: usize = 64;

/// Maximum in-flight (paced, deadline-stamped) bytes per stream route. A
/// real UART has a finite TX path, not an infinite buffer: a producer that
/// sustains writes above its declared baud would otherwise grow the queue —
/// and its virtual delivery horizon — without bound. Overflow bytes are
/// shed with a trace and a [`Finding::StreamOverrun`] naming the producer
/// pin (the natural surface for a producer-vs-declared-baud mismatch). At
/// 115 200 baud this depth is well under a second of backlog.
const STREAM_ROUTE_QUEUE_MAX: usize = 8192;

/// How long the engine waits on a missing drive enqueue sequence before
/// skipping the gap. `set_drive` reserves the global sequence and sends the
/// command as two separate steps, so a thread dying between them leaves a
/// permanent gap that would otherwise hold EVERY later drive hostage in
/// `pending_drives` — no timeout, no symptom beyond the board going quiet.
/// The wait is wall-clock (enqueuing is wall-side) and generous: a live
/// enqueuer covers the reserve→send window in nanoseconds.
const DRIVE_SEQ_STALL_TIMEOUT: Duration = Duration::from_millis(250);

// ============================================================
// Identity
// ============================================================

/// Dense index of one drive-capable pin endpoint, assigned at assembly.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub(crate) struct EndpointId(pub(crate) usize);

/// Dense index of one attached component, assigned at assembly (keys the
/// timer wheel's wakeup delivery).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub(crate) struct ComponentId(pub(crate) usize);

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

/// Stream byte-delivery callback: called from the engine thread once per
/// byte routed to a consumer endpoint; no engine lock held. A callback MAY
/// drive pins or write stream bytes — both enqueue, per the re-entrancy
/// contract.
pub(crate) type StreamCallback = Box<dyn Fn(u8) + Send>;

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
    /// One-shot wakeup at an absolute virtual time (µs).
    ScheduleAt {
        /// Component whose wake handler fires.
        component: ComponentId,
        /// Absolute virtual deadline (µs). Past deadlines fire immediately.
        at_us: u64,
    },
    /// Periodic wakeup every `period_us` of virtual time.
    ScheduleEvery {
        /// Component whose wake handler fires.
        component: ComponentId,
        /// Virtual period (µs); zero is rejected with a warning.
        period_us: u64,
    },
    /// Subscribe to net-graph topology changes (stream-routing seam). The
    /// current epoch is delivered once at registration.
    RegisterTopologyObserver {
        /// Notification callback.
        callback: TopologyCallback,
    },
    /// Bytes written by a stream producer endpoint. Bytes flow on the route
    /// derived at the last routing pass, paced at the producer's declared
    /// baud; bytes written into a broken/missing route are dropped with a
    /// trace, never queued forever. Unlike drives, writes carry no enqueue
    /// sequence: per-producer FIFO order (this channel's order) is the wire
    /// contract, and cross-endpoint byte ordering is not meaningful.
    StreamWrite {
        /// Producer endpoint.
        endpoint: EndpointId,
        /// Payload bytes, in wire order.
        bytes: Vec<u8>,
    },
    /// Subscribe a byte callback to a stream consumer endpoint.
    RegisterStreamConsumer {
        /// Consumer endpoint.
        endpoint: EndpointId,
        /// Delivery callback.
        callback: StreamCallback,
    },
    /// Stop the engine loop; pending drives and timers are discarded.
    Shutdown,
}

/// Cloneable client half of the engine: command sender, the global drive
/// sequence counter, and the engine-published net-state table.
///
/// An **inert** link (`tx == None`) is what the build-time analysis path
/// hands out: senses read the build-resolved snapshot; drives and schedules
/// are traced and dropped.
#[derive(Debug, Clone, Default)]
pub(crate) struct EngineLink {
    /// Command queue into the engine thread; `None` on the inert build path.
    pub(crate) tx: Option<Sender<Command>>,
    /// Global drive enqueue sequence, shared by every clone of this link.
    pub(crate) drive_seq: Arc<AtomicU64>,
    /// Engine-published resolved state per net (build snapshot when inert).
    pub(crate) states: Arc<Mutex<Vec<NetState>>>,
}

impl EngineLink {
    /// Inert link over a fixed state snapshot (the build-time analysis path).
    pub(crate) fn inert(states: Arc<Mutex<Vec<NetState>>>) -> Self {
        Self {
            tx: None,
            drive_seq: Arc::new(AtomicU64::new(0)),
            states,
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

/// One derived producer→consumers serial route (see
/// [`Resolver::route_streams`]).
pub(crate) struct StreamRouteSpec {
    /// Producer endpoint the route originates at.
    pub(crate) producer: EndpointId,
    /// Producer's declared byte pacing rate (0 = unpaced).
    pub(crate) baud_hz: u32,
    /// Consumer endpoints reachable through the collapsed link.
    pub(crate) consumers: Vec<EndpointId>,
    /// Identity roots of every net the collapsed link spans — live delivery
    /// is gated on their resolved state.
    pub(crate) path_roots: Vec<usize>,
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
    /// Scenario `stream_drop` byte-loss policies per endpoint.
    stream_drops: Vec<(EndpointId, StreamDropPolicy)>,
    net_count: usize,
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
            stream_drops: Vec::new(),
            net_count,
        }
    }

    /// Add a conduction edge between two nets.
    pub(crate) fn add_edge(&mut self, a: usize, b: usize, ohms: f64) {
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
        self.slots.push(DriveSlot {
            net,
            pin,
            drive: initial,
        });
        EndpointId(self.slots.len() - 1)
    }

    /// Replace an endpoint's drive contribution (`None` releases to high-Z).
    /// Live path only; the next `resolve` sees the new table.
    pub(crate) fn set_drive(&mut self, endpoint: EndpointId, drive: Option<TheveninDrive>) {
        if let Some(slot) = self.slots.get_mut(endpoint.0) {
            slot.drive = drive;
        } else {
            tracing::warn!(endpoint = endpoint.0, "drive for unknown endpoint dropped");
        }
    }

    /// Add a power-rail source (harness power endpoint or `PowerOut` pin).
    pub(crate) fn add_power_source(&mut self, net: usize, volts: Volts) {
        self.power_sources.push((net, volts));
    }

    /// Add a `net_stuck` fault source.
    pub(crate) fn add_stuck_source(&mut self, net: usize, volts: Volts) {
        self.stuck_sources.push((net, volts));
    }

    /// Register a digital sense pin (floating-sense findings).
    pub(crate) fn add_digital_sense(&mut self, net: usize) {
        self.digital_senses.push(net);
    }

    /// Register an analog sense pin (floating-sense findings).
    pub(crate) fn add_analog_sense(&mut self, net: usize) {
        self.analog_senses.push(net);
    }

    /// Register a power sense pin (`PowerNetUnsourced` findings).
    pub(crate) fn add_power_sense(&mut self, net: usize) {
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

    /// Register a scenario `stream_drop` byte-loss policy on one stream
    /// endpoint (producer side drops before pacing; consumer side drops at
    /// delivery).
    pub(crate) fn add_stream_drop(&mut self, endpoint: EndpointId, policy: StreamDropPolicy) {
        self.stream_drops.push((endpoint, policy));
    }

    /// Run one full resolution pass: assign every net a [`NetState`] from the
    /// current drive table and report findings. Identity-merged nets share
    /// state; conduction clusters share sourced-ness. Clusters where a
    /// competing source sits within [`ESCALATION_IMPEDANCE_RATIO`] of the
    /// strongest driver escalate to `solver`.
    pub(crate) fn resolve(
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
        for (net, _volts) in &self.stuck_sources {
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
        let mut cluster_sources: HashMap<usize, Vec<ClusterSource>> = HashMap::new();
        for slots in net_drivers.values() {
            for &si in slots {
                let drive = drive_of(si);
                let net = self.slots[si].net;
                cluster_sources
                    .entry(cluster_of[net])
                    .or_default()
                    .push(ClusterSource {
                        node: NetId(root_of[net]),
                        volts: drive.volts,
                        impedance: drive.impedance,
                    });
            }
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
            let mut driver_roots: Vec<usize> = net_drivers.keys().copied().collect();
            driver_roots.sort_unstable();
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
        let mut escalated: HashMap<usize, ClusterSolution> = HashMap::new();
        let mut driver_roots: Vec<usize> = net_drivers.keys().copied().collect();
        driver_roots.sort_unstable();
        for root in driver_roots {
            let cluster = cluster_of[root];
            if escalated.contains_key(&cluster) {
                continue;
            }
            let slots = &net_drivers[&root];
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
        let ideal_contended: HashSet<usize> = ideal_root_levels
            .iter()
            .filter(|(_, levels)| levels.len() > 1)
            .map(|(&root, _)| root)
            .collect();
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
                let level = match cluster_power.get(&cluster) {
                    Some(v) if *v < DIGITAL_LEVEL_THRESHOLD_VOLTS => Level::Low,
                    _ => Level::High,
                };
                NetState::Pulled(level, total)
            } else {
                NetState::Floating
            };
            net.state = state;
        }

        // -- findings ---------------------------------------------------------
        // Contention (per identity root, deduped).
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

    /// Derive the serial byte routes from the current net topology
    /// (`BOARD_ENGINE.md`, "Stream endpoints (serial over pins)"). Byte
    /// pipes are derived from and gated by net resolution, never installed
    /// beside it:
    ///
    /// - Each `Producer` routes to every `Consumer` reachable through its
    ///   net and through series conduction whose **accumulated** resistance
    ///   stays below [`STREAM_COLLAPSE_THRESHOLD`] — the DS2Addon's 47 Ω
    ///   series resistors collapse into the link.
    /// - Two producers reachable from each other (the crossed-TX/RX harness)
    ///   raise [`Finding::StreamMismatch`] once per pair, and neither
    ///   producer gets a route (bytes written into it are dropped).
    ///
    /// Runs at `System::build`, at engine spawn, and on any
    /// topology-affecting change, so routes can never outlive the topology
    /// they were derived from.
    pub(crate) fn route_streams(
        &mut self,
        nets: &[Net],
        diagnostics: &mut Diagnostics,
    ) -> Vec<StreamRouteSpec> {
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
        let mut reported_pairs: HashSet<(usize, usize)> = HashSet::new();
        for (pi, producer) in self.streams.iter().enumerate() {
            let StreamRole::Producer { baud_hz } = producer.role else {
                continue;
            };
            let origin = root_of[producer.net];
            let dist = min_path_ohms(&root_edges, origin);
            let reachable = |net: usize| {
                dist.get(&root_of[net])
                    .is_some_and(|&ohms| ohms < STREAM_COLLAPSE_THRESHOLD)
            };

            // Facing producers invalidate the route (crossed TX/RX).
            let facing: Vec<usize> = self
                .streams
                .iter()
                .enumerate()
                .filter(|(oi, other)| {
                    *oi != pi
                        && matches!(other.role, StreamRole::Producer { .. })
                        && reachable(other.net)
                })
                .map(|(oi, _)| oi)
                .collect();
            if !facing.is_empty() {
                for oi in facing {
                    let pair = (pi.min(oi), pi.max(oi));
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

            let consumers: Vec<EndpointId> = self
                .streams
                .iter()
                .filter(|s| matches!(s.role, StreamRole::Consumer { .. }) && reachable(s.net))
                .map(|s| s.endpoint)
                .collect();
            // Deliberately conservative gate: EVERY identity root within
            // the collapse radius of the producer is collected, not just
            // roots on a producer→consumer path — so delivery is also
            // gated on side branches hanging off the link below the
            // threshold. Reconstructing exact shortest paths from the
            // dist map (predecessor tracking, or two-sided distance
            // equality over accumulated f64 sums) buys little fidelity
            // for the added subtlety: over-gating can only drop bytes a
            // real link might have carried; it never leaks bytes onto a
            // broken one.
            let mut path_roots: Vec<usize> = dist
                .iter()
                .filter(|(_, &ohms)| ohms < STREAM_COLLAPSE_THRESHOLD)
                .map(|(&root, _)| root)
                .collect();
            path_roots.sort_unstable();
            routes.push(StreamRouteSpec {
                producer: producer.endpoint,
                baud_hz,
                consumers,
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
    /// Paced byte delivery on one stream producer's route.
    Stream(EndpointId),
}

/// One armed wakeup. Ordered by `(deadline_us, seq)` so simultaneous and
/// late deadlines fire in schedule order.
#[derive(Debug, Clone, Copy, Eq)]
struct TimerEntry {
    deadline_us: u64,
    seq: u64,
    target: TimerTarget,
    /// `Some(period)` re-arms after firing (periodic wakes only); `None`
    /// is one-shot.
    period_us: Option<u64>,
}

impl PartialEq for TimerEntry {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other) == std::cmp::Ordering::Equal
    }
}

impl Ord for TimerEntry {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        (self.deadline_us, self.seq).cmp(&(other.deadline_us, other.seq))
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

/// Live per-producer route state: the derived consumers, the pacing
/// schedule, and the in-flight (deadline-stamped) bytes.
struct LiveRoute {
    /// Producer's declared byte pacing rate (0 = unpaced).
    baud_hz: u32,
    /// Consumer endpoints on the collapsed link.
    consumers: Vec<EndpointId>,
    /// Identity roots of the nets the link spans (delivery gate).
    path_roots: Vec<usize>,
    /// Bytes clocked onto the link, stamped with the virtual deadline at
    /// which their frame finishes.
    queue: VecDeque<(u64, u8)>,
    /// Virtual time at which the line is next free — the pacing slot,
    /// mirroring the embsim serial peripheral's `tx_next_v_us` convention.
    line_next_v_us: u64,
}

/// Byte-loss injection state for one endpoint (`Scenario::stream_drop`).
struct DropState {
    policy: StreamDropPolicy,
    /// Bytes offered so far (the `EveryNth` counter).
    seen: u64,
}

impl DropState {
    /// Count one offered byte; true when the policy says to drop it.
    fn drops_next(&mut self) -> bool {
        self.seen += 1;
        match self.policy {
            StreamDropPolicy::All => true,
            // Degenerate EveryNth(0) keeps everything (warned at spawn).
            StreamDropPolicy::EveryNth(0) => false,
            StreamDropPolicy::EveryNth(n) => self.seen.is_multiple_of(u64::from(n)),
        }
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
    diagnostics: Arc<Mutex<Diagnostics>>,
    sense_subs: HashMap<usize, Vec<SenseCallback>>,
    wake_subs: HashMap<usize, WakeCallback>,
    /// Live stream routes, keyed by producer endpoint index. Rebuilt (and
    /// in-flight bytes discarded) on every routing pass.
    routes: HashMap<usize, LiveRoute>,
    /// Stream byte subscriptions, keyed by consumer endpoint index.
    stream_subs: HashMap<usize, Vec<StreamCallback>>,
    /// `stream_drop` fault state, keyed by endpoint index.
    drop_state: HashMap<usize, DropState>,
    topology_observers: Vec<TopologyCallback>,
    topology_epoch: u64,
    wheel: BinaryHeap<Reverse<TimerEntry>>,
    timer_seq: u64,
    /// Drives received but not yet applicable (a lower enqueue seq is still
    /// in flight); applied strictly in seq order.
    pending_drives: BTreeMap<u64, (EndpointId, Option<TheveninDrive>)>,
    next_drive_seq: u64,
    /// Gap watchdog: the missing seq currently being waited on and when the
    /// wait started (wall clock). `None` when no gap is outstanding.
    drive_stall: Option<(u64, Instant)>,
}

impl EngineCore {
    /// Report one finding straight onto the cumulative live bus, deduped
    /// like [`Self::merge_findings`] (once per distinct occurrence).
    fn report_finding(&self, finding: Finding) {
        let mut cumulative = self.diagnostics.lock().unwrap();
        if !cumulative.contains(&finding) {
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
        for (i, net) in self.nets.iter().enumerate() {
            let changed = old.get(i).is_none_or(|prev| !same_state(prev, &net.state));
            if changed {
                if let Some(subs) = self.sense_subs.get(&i) {
                    for callback in subs {
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
            if !cumulative.contains(finding) {
                cumulative.report(finding.clone());
            }
        }
    }

    /// (Re-)derive the stream routes from the current topology and merge the
    /// routing findings (`StreamMismatch`). Runs at spawn and on any
    /// topology-affecting change; in-flight bytes on stale routes are
    /// discarded — a broken route stops delivery, it never queues forever.
    fn reroute_streams(&mut self) {
        let mut pass = Diagnostics::new();
        let specs = self.resolver.route_streams(&self.nets, &mut pass);
        self.merge_findings(&pass);
        self.routes = specs
            .into_iter()
            .map(|spec| {
                (
                    spec.producer.0,
                    LiveRoute {
                        baud_hz: spec.baud_hz,
                        consumers: spec.consumers,
                        path_roots: spec.path_roots,
                        queue: VecDeque::new(),
                        line_next_v_us: 0,
                    },
                )
            })
            .collect();
    }

    /// Clock producer bytes onto their derived route: apply the
    /// producer-side drop policy, then either deliver immediately (unpaced)
    /// or stamp each byte with its virtual delivery deadline at the
    /// producer's declared baud ([`STREAM_BITS_PER_BYTE`] bits per byte, the
    /// embsim 8N1 pacing convention) and arm the wheel. Bytes written into a
    /// broken or missing route are dropped with a trace.
    fn stream_write(&mut self, endpoint: EndpointId, bytes: Vec<u8>) {
        let bytes: Vec<u8> = match self.drop_state.get_mut(&endpoint.0) {
            Some(state) => bytes.into_iter().filter(|_| !state.drops_next()).collect(),
            None => bytes,
        };
        let Some(route) = self.routes.get(&endpoint.0) else {
            tracing::debug!(
                endpoint = endpoint.0,
                dropped = bytes.len(),
                "stream bytes dropped: producer has no valid route"
            );
            return;
        };
        if bytes.is_empty() {
            return;
        }
        let cost_v_us = if route.baud_hz == 0 {
            0
        } else {
            STREAM_BITS_PER_BYTE.saturating_mul(1_000_000) / u64::from(route.baud_hz)
        };
        if cost_v_us == 0 {
            // Unpaced (baud 0, or faster than a byte per µs): deliver now,
            // in write order, without ever touching the virtual clock.
            let consumers = route.consumers.clone();
            let path_roots = route.path_roots.clone();
            self.deliver_stream_bytes(&consumers, &path_roots, &bytes);
            return;
        }
        // Paced delivery needs the virtual clock — validated loudly here so
        // a missing `virtual_clock::init` cannot panic the engine thread.
        if !self.clock_ready("paced stream write") {
            tracing::debug!(
                endpoint = endpoint.0,
                dropped = bytes.len(),
                "paced stream bytes dropped: virtual clock not initialized"
            );
            return;
        }
        let now = virtual_clock::virtual_us();
        let route = self
            .routes
            .get_mut(&endpoint.0)
            .expect("route existed above; routes are engine-thread-owned");
        // Cap the in-flight queue ([`STREAM_ROUTE_QUEUE_MAX`]): the newest
        // bytes are shed first, like a full TX FIFO, and the mismatch is
        // surfaced below rather than modeling an infinite buffer no UART has.
        let room = STREAM_ROUTE_QUEUE_MAX.saturating_sub(route.queue.len());
        let accepted = bytes.len().min(room);
        let shed = bytes.len() - accepted;
        let start = route.line_next_v_us.max(now);
        let first_deadline = start.saturating_add(cost_v_us);
        let mut deadline = start;
        for &byte in &bytes[..accepted] {
            deadline = deadline.saturating_add(cost_v_us);
            route.queue.push_back((deadline, byte));
        }
        route.line_next_v_us = deadline;
        if accepted > 0 {
            self.arm(first_deadline, TimerTarget::Stream(endpoint), None);
        }
        if shed > 0 {
            let producer = self
                .resolver
                .streams
                .iter()
                .find(|s| s.endpoint == endpoint)
                .map(|s| s.pin.clone())
                .unwrap_or_else(|| PinRef::new("?", "?"));
            tracing::warn!(
                producer = ?producer,
                shed,
                in_flight = STREAM_ROUTE_QUEUE_MAX,
                "paced stream route overflowed: producer sustains more than \
                 its declared baud; overflow bytes shed"
            );
            self.report_finding(Finding::StreamOverrun { producer });
        }
    }

    /// Deliver every queued byte whose deadline has passed on one producer's
    /// route, then re-arm the wheel for the next in-flight byte.
    fn deliver_due_stream(&mut self, endpoint: EndpointId, now: u64) {
        let Some(route) = self.routes.get_mut(&endpoint.0) else {
            return; // route dissolved while the timer was in flight
        };
        let mut due: Vec<u8> = Vec::new();
        while let Some(&(deadline, byte)) = route.queue.front() {
            if deadline > now {
                break;
            }
            route.queue.pop_front();
            due.push(byte);
        }
        let next = route.queue.front().map(|&(deadline, _)| deadline);
        let consumers = route.consumers.clone();
        let path_roots = route.path_roots.clone();
        if let Some(deadline) = next {
            self.arm(deadline, TimerTarget::Stream(endpoint), None);
        }
        self.deliver_stream_bytes(&consumers, &path_roots, &due);
    }

    /// Deliver bytes to a route's consumers, gated by net resolution: a link
    /// whose nets currently resolve `Contention` or `Floating` cannot carry
    /// clean bytes, so the bytes are dropped with a trace (never queued
    /// forever). Callbacks run on the engine thread with no lock held.
    fn deliver_stream_bytes(
        &mut self,
        consumers: &[EndpointId],
        path_roots: &[usize],
        bytes: &[u8],
    ) {
        if bytes.is_empty() {
            return;
        }
        let broken = path_roots.iter().any(|&root| {
            !matches!(
                self.nets.get(root).map(|net| net.state),
                Some(NetState::Driven(_) | NetState::Pulled(_, _) | NetState::Analog(_))
            )
        });
        if broken {
            tracing::debug!(
                dropped = bytes.len(),
                "stream bytes dropped: a net on the route is not signal-capable"
            );
            return;
        }
        // Consumer identities for panic-containment findings, resolved once
        // per delivery batch rather than once per byte.
        let subscribers: Vec<String> = consumers
            .iter()
            .map(|consumer| {
                self.resolver
                    .streams
                    .iter()
                    .find(|s| s.endpoint == *consumer)
                    .map(|s| format!("{}.{}", s.pin.reference, s.pin.pin))
                    .unwrap_or_else(|| format!("stream consumer endpoint {}", consumer.0))
            })
            .collect();
        for &byte in bytes {
            for (ci, consumer) in consumers.iter().enumerate() {
                if let Some(state) = self.drop_state.get_mut(&consumer.0) {
                    if state.drops_next() {
                        continue;
                    }
                }
                if let Some(subs) = self.stream_subs.get(&consumer.0) {
                    for callback in subs {
                        self.deliver_contained(CallbackKind::StreamByte, &subscribers[ci], || {
                            callback(byte);
                        });
                    }
                }
            }
        }
    }

    /// Apply buffered drives strictly in enqueue-seq order, resolving after
    /// each so senses observe the authoritative event order.
    fn apply_ready_drives(&mut self) {
        while let Some((endpoint, drive)) = self.pending_drives.remove(&self.next_drive_seq) {
            self.next_drive_seq += 1;
            self.resolver.set_drive(endpoint, drive);
            self.resolve_and_publish();
        }
    }

    /// Detect and break a wedged drive queue. A reserved-but-never-sent
    /// enqueue seq (the enqueuing thread died between `next_drive_seq`'s
    /// fetch_add and the channel send) would otherwise hold every later
    /// drive from EVERY component hostage in `pending_drives`, silently.
    /// After [`DRIVE_SEQ_STALL_TIMEOUT`] on one missing seq the engine
    /// reports a loud [`Finding::DriveSeqGap`] and skips to the lowest
    /// buffered seq — ordering against a dead enqueuer is moot. The run
    /// loop bounds its park time while a gap is outstanding, so detection
    /// does not depend on further traffic arriving.
    fn check_drive_stall(&mut self) {
        let Some((&lowest, _)) = self.pending_drives.first_key_value() else {
            self.drive_stall = None;
            return;
        };
        let missing = self.next_drive_seq;
        match self.drive_stall {
            Some((seq, since)) if seq == missing => {
                if since.elapsed() >= DRIVE_SEQ_STALL_TIMEOUT {
                    tracing::warn!(
                        missing_seq = missing,
                        resuming_at = lowest,
                        buffered = self.pending_drives.len(),
                        "drive enqueue seq never arrived (enqueuer died between \
                         reserve and send?); skipping the gap"
                    );
                    self.report_finding(Finding::DriveSeqGap { seq: missing });
                    self.drive_stall = None;
                    self.next_drive_seq = lowest;
                    self.apply_ready_drives();
                }
            }
            // New gap (or the gap moved): restart the wall-clock wait.
            _ => self.drive_stall = Some((missing, Instant::now())),
        }
    }

    /// Fire every wheel entry whose deadline has passed, in `(deadline,
    /// schedule)` order. Wake timestamps are sampled from the free-running
    /// virtual clock; missed periodic deadlines coalesce to one catch-up
    /// fire, then re-arm on period. Stream targets deliver their route's
    /// due bytes instead of a wake callback.
    fn fire_due_timers(&mut self) {
        while let Some(&Reverse(head)) = self.wheel.peek() {
            let now = virtual_clock::virtual_us();
            if head.deadline_us > now {
                break;
            }
            self.wheel.pop();
            match head.target {
                TimerTarget::Wake(component) => {
                    if let Some(callback) = self.wake_subs.get(&component.0) {
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
                    if let Some(period) = head.period_us {
                        let mut next = head.deadline_us.saturating_add(period);
                        if next <= now {
                            next = now.saturating_add(period);
                        }
                        self.arm(next, head.target, Some(period));
                    }
                }
                TimerTarget::Stream(endpoint) => self.deliver_due_stream(endpoint, now),
            }
        }
    }

    /// Push a wheel entry.
    fn arm(&mut self, deadline_us: u64, target: TimerTarget, period_us: Option<u64>) {
        let seq = self.timer_seq;
        self.timer_seq += 1;
        self.wheel.push(Reverse(TimerEntry {
            deadline_us,
            seq,
            target,
            period_us,
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
                self.deliver_contained(CallbackKind::Sense, &subscriber, || callback(state));
                self.sense_subs.entry(net.0).or_default().push(callback);
            }
            Command::RegisterWake {
                component,
                callback,
            } => {
                self.wake_subs.insert(component.0, callback);
            }
            Command::ScheduleAt { component, at_us } => {
                // Wheel entries make the run loop read the virtual clock;
                // gate here (and at every other arm entry point) so a
                // missing init fails the request, never the engine thread.
                if self.clock_ready("schedule_at") {
                    self.arm(at_us, TimerTarget::Wake(component), None);
                }
            }
            Command::ScheduleEvery {
                component,
                period_us,
            } => {
                if period_us == 0 {
                    tracing::warn!(component = component.0, "schedule_every(0) ignored");
                } else if self.clock_ready("schedule_every") {
                    let now = virtual_clock::virtual_us();
                    self.arm(
                        now.saturating_add(period_us),
                        TimerTarget::Wake(component),
                        Some(period_us),
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
            Command::StreamWrite { endpoint, bytes } => {
                self.stream_write(endpoint, bytes);
            }
            Command::RegisterStreamConsumer { endpoint, callback } => {
                self.stream_subs
                    .entry(endpoint.0)
                    .or_default()
                    .push(callback);
            }
            Command::Shutdown => return true,
        }
        false
    }

    /// Engine thread body: fire due timers, run the drive-gap watchdog,
    /// park on the queue until the next wheel deadline or stall check (or
    /// indefinitely when neither is pending), then handle commands in
    /// bounded batches ([`COMMAND_DRAIN_BATCH_MAX`]) so due timers keep
    /// firing under sustained command load.
    fn run(mut self, rx: Receiver<Command>) {
        loop {
            self.fire_due_timers();
            self.check_drive_stall();
            let command = match self.next_wall_wait_us() {
                None => match rx.recv() {
                    Ok(c) => c,
                    Err(_) => break, // every handle dropped without Shutdown
                },
                Some(wait_wall) => match rx.recv_timeout(Duration::from_micros(wait_wall)) {
                    Ok(c) => c,
                    Err(RecvTimeoutError::Timeout) => continue,
                    Err(RecvTimeoutError::Disconnected) => break,
                },
            };
            if self.handle(command) {
                break;
            }
            // Bounded drain: a saturated queue must not starve the timer
            // wheel — control returns to the timer check above after at
            // most COMMAND_DRAIN_BATCH_MAX further commands. A disconnect
            // observed here is handled by the blocking receive above.
            let mut shutdown = false;
            let mut drained = 0;
            while drained < COMMAND_DRAIN_BATCH_MAX {
                let Ok(command) = rx.try_recv() else { break };
                drained += 1;
                if self.handle(command) {
                    shutdown = true;
                    break;
                }
            }
            if shutdown {
                break;
            }
        }
    }

    /// Wall-clock park bound (µs) until the engine must wake without a
    /// command: the earlier of the next wheel deadline and the outstanding
    /// drive-gap stall check. `None` parks indefinitely (empty wheel, no
    /// gap). The virtual clock is only read while the wheel is non-empty,
    /// so a timer-free system never requires `virtual_clock::init`.
    fn next_wall_wait_us(&self) -> Option<u64> {
        let wheel_wait = self.wheel.peek().map(|&Reverse(head)| {
            let now = virtual_clock::virtual_us();
            virtual_clock::virtual_to_wall_us(head.deadline_us.saturating_sub(now))
        });
        let stall_wait = self.drive_stall.map(|(_, since)| {
            let remaining = DRIVE_SEQ_STALL_TIMEOUT.saturating_sub(since.elapsed());
            u64::try_from(remaining.as_micros()).unwrap_or(u64::MAX)
        });
        match (wheel_wait, stall_wait) {
            (Some(wheel), Some(stall)) => Some(wheel.min(stall)),
            (wheel, stall) => wheel.or(stall),
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
    join: Option<JoinHandle<()>>,
}

impl EngineHandle {
    /// Start the engine thread over an assembled topology. The initial full
    /// resolution pass **and** the initial stream-routing pass run
    /// synchronously *before* the thread starts, so never-driven nets are
    /// reported and routing findings (`StreamMismatch`) are populated by the
    /// time this returns — before any traffic.
    ///
    /// # Panics
    /// Panics if the OS refuses to spawn the engine thread.
    pub(crate) fn spawn(
        resolver: Resolver,
        nets: Vec<Net>,
        solver: Box<dyn ClusterSolver>,
    ) -> Self {
        let states: Arc<Mutex<Vec<NetState>>> =
            Arc::new(Mutex::new(nets.iter().map(|n| n.state).collect()));
        let diagnostics = Arc::new(Mutex::new(Diagnostics::new()));
        let (tx, rx) = mpsc::channel();

        // Scenario byte-loss state, keyed by endpoint (last policy wins).
        let drop_state: HashMap<usize, DropState> = resolver
            .stream_drops
            .iter()
            .map(|&(endpoint, policy)| {
                if policy == StreamDropPolicy::EveryNth(0) {
                    tracing::warn!(
                        endpoint = endpoint.0,
                        "stream_drop EveryNth(0) never drops a byte"
                    );
                }
                (endpoint.0, DropState { policy, seen: 0 })
            })
            .collect();

        let mut core = EngineCore {
            resolver,
            nets,
            solver,
            states: Arc::clone(&states),
            diagnostics: Arc::clone(&diagnostics),
            sense_subs: HashMap::new(),
            wake_subs: HashMap::new(),
            routes: HashMap::new(),
            stream_subs: HashMap::new(),
            drop_state,
            topology_observers: Vec::new(),
            topology_epoch: 0,
            wheel: BinaryHeap::new(),
            timer_seq: 0,
            pending_drives: BTreeMap::new(),
            next_drive_seq: 0,
            drive_stall: None,
        };
        core.resolve_and_publish();
        // Byte pipes are derived from net resolution, never installed
        // beside it: the routing pass runs against the just-resolved nets.
        core.reroute_streams();

        let join = std::thread::Builder::new()
            .name("embsim-board-net-engine".to_string())
            .spawn(move || core.run(rx))
            .expect("failed to spawn the net-engine thread");

        Self {
            link: EngineLink {
                tx: Some(tx),
                drive_seq: Arc::new(AtomicU64::new(0)),
                states,
            },
            diagnostics,
            join: Some(join),
        }
    }

    /// Cloneable client link for attaching components.
    pub(crate) fn link(&self) -> EngineLink {
        self.link.clone()
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
        let handle = EngineHandle::spawn(resolver, nets(1), Box::new(QuasiStaticMna));
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
        let handle = EngineHandle::spawn(resolver, nets(1), Box::new(QuasiStaticMna));
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
        let handle = EngineHandle::spawn(resolver, nets(2), Box::new(QuasiStaticMna));
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
        let handle = EngineHandle::spawn(resolver, nets(2), Box::new(QuasiStaticMna));
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
        let handle = EngineHandle::spawn(resolver, nets(2), Box::new(QuasiStaticMna));
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
    /// asserted independent of the (evolving) MNA implementation.
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
        handle.link().send(Command::RegisterWake {
            component,
            callback: Box::new(move |now_us| sink.lock().unwrap().push(now_us)),
        });
        log
    }

    fn empty_engine() -> EngineHandle {
        EngineHandle::spawn(
            Resolver::new(0, Dsu::new(0)),
            Vec::new(),
            Box::new(QuasiStaticMna),
        )
    }

    /// `schedule_at` fires exactly once, at-or-after its virtual deadline
    /// (timestamps are sampled from the free-running scaled clock).
    #[rstest]
    fn one_shot_timer_fires_once_at_virtual_deadline() {
        let _g = lock_clock();
        virtual_clock::init(50.0, 1_000_000);
        let handle = empty_engine();
        let log = wake_log(&handle, ComponentId(0));

        let now = virtual_clock::virtual_us();
        let deadline = now + 100_000; // 100 virtual ms = 2 wall ms at 50x
        handle.link().send(Command::ScheduleAt {
            component: ComponentId(0),
            at_us: deadline,
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
        virtual_clock::init(50.0, 1_000_000);
        let handle = empty_engine();
        let log = wake_log(&handle, ComponentId(0));

        handle.link().send(Command::ScheduleEvery {
            component: ComponentId(0),
            period_us: 20_000, // 0.4 wall ms at 50x
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
        virtual_clock::init(1.0, 1_000_000);
        std::thread::sleep(Duration::from_millis(5)); // let virtual time pass both deadlines
        let handle = empty_engine();

        let order: Arc<StdMutex<Vec<u32>>> = Arc::new(StdMutex::new(Vec::new()));
        for (component, tag) in [(ComponentId(0), 0u32), (ComponentId(1), 1u32)] {
            let sink = Arc::clone(&order);
            handle.link().send(Command::RegisterWake {
                component,
                callback: Box::new(move |_| sink.lock().unwrap().push(tag)),
            });
        }
        // Schedule the LATER deadline first; firing order must follow the
        // deadlines, not the schedule order.
        handle.link().send(Command::ScheduleAt {
            component: ComponentId(1),
            at_us: 2_000,
        });
        handle.link().send(Command::ScheduleAt {
            component: ComponentId(0),
            at_us: 1_000,
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
        virtual_clock::init(1.0, 1_000_000);
        let handle = empty_engine();
        let _log = wake_log(&handle, ComponentId(0));
        handle.link().send(Command::ScheduleAt {
            component: ComponentId(0),
            at_us: virtual_clock::virtual_us() + 60_000_000, // one virtual minute out
        });
        std::thread::sleep(Duration::from_millis(10)); // let the engine park on the deadline

        let start = Instant::now();
        drop(handle); // sends Shutdown + joins
        assert!(
            start.elapsed() < Duration::from_secs(5),
            "drop must not wait for the pending timer"
        );
    }

    /// The timer wheel must not starve under sustained command load: a busy
    /// protocol thread enqueuing drives in a tight loop keeps the channel
    /// non-empty (enqueuing is far cheaper than the resolution pass each
    /// `Drive` costs), yet a due wake must still fire — the drain batch is
    /// bounded ([`COMMAND_DRAIN_BATCH_MAX`]).
    #[rstest]
    fn sustained_drive_flood_does_not_starve_the_timer_wheel() {
        let _g = lock_clock();
        virtual_clock::init(1.0, 1_000_000);
        let mut resolver = Resolver::new(1, Dsu::new(1));
        let e0 = resolver.add_endpoint(0, PinRef::new("U1", "1"), None);
        let handle = EngineHandle::spawn(resolver, nets(1), Box::new(QuasiStaticMna));
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

        handle.link().send(Command::ScheduleAt {
            component: ComponentId(0),
            at_us: virtual_clock::virtual_us() + 10_000, // 10 virtual ms
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
    /// later drive forever: after [`DRIVE_SEQ_STALL_TIMEOUT`] the engine
    /// reports [`Finding::DriveSeqGap`] naming the missing seq and applies
    /// the buffered drives.
    #[rstest]
    fn missing_drive_seq_is_skipped_after_a_bounded_wait() {
        let mut resolver = Resolver::new(1, Dsu::new(1));
        let e0 = resolver.add_endpoint(0, PinRef::new("U1", "1"), None);
        let handle = EngineHandle::spawn(resolver, nets(1), Box::new(QuasiStaticMna));
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
        let handle = EngineHandle::spawn(resolver, nets(1), Box::new(QuasiStaticMna));
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

    fn producer(baud_hz: u32) -> StreamRole {
        StreamRole::Producer { baud_hz }
    }

    fn consumer(baud_hz: u32) -> StreamRole {
        StreamRole::Consumer { baud_hz }
    }

    /// Stream routes collapse series passives below the threshold
    /// (accumulated along the path) and stop at/above it.
    #[rstest]
    fn stream_routes_collapse_series_passives_below_threshold() {
        // producer(0) --47Ω-- (1) --47Ω-- consumer(2), plus a consumer
        // behind a 4.7 kΩ edge that must NOT be routed.
        let mut resolver = Resolver::new(4, Dsu::new(4));
        let p = resolver.add_endpoint(0, PinRef::new("U1", "15"), Some(high()));
        let c_near = resolver.add_endpoint(2, PinRef::new("U2", "16"), None);
        let c_far = resolver.add_endpoint(3, PinRef::new("U3", "16"), None);
        resolver.add_edge(0, 1, 47.0);
        resolver.add_edge(1, 2, 47.0);
        resolver.add_edge(0, 3, 4_700.0);
        resolver.add_stream_pin(p, 0, producer(115_200), PinRef::new("U1", "15"));
        resolver.add_stream_pin(c_near, 2, consumer(115_200), PinRef::new("U2", "16"));
        resolver.add_stream_pin(c_far, 3, consumer(115_200), PinRef::new("U3", "16"));

        let net_table = nets(4);
        let mut diags = Diagnostics::new();
        let routes = resolver.route_streams(&net_table, &mut diags);
        assert!(diags.is_empty(), "no mismatch: {:?}", diags.findings());
        assert_eq!(routes.len(), 1);
        assert_eq!(routes[0].producer, p);
        assert_eq!(routes[0].baud_hz, 115_200);
        assert_eq!(routes[0].consumers, vec![c_near]);
    }

    /// Two producers reachable from each other (the crossed-TX/RX harness)
    /// raise StreamMismatch once per pair, and neither producer routes.
    #[rstest]
    fn facing_producers_raise_stream_mismatch_and_do_not_route() {
        let mut resolver = Resolver::new(2, Dsu::new(2));
        let p_a = resolver.add_endpoint(0, PinRef::new("MCU", "1"), Some(high()));
        let p_b = resolver.add_endpoint(1, PinRef::new("U1", "15"), Some(high()));
        resolver.add_edge(0, 1, 47.0);
        resolver.add_stream_pin(p_a, 0, producer(115_200), PinRef::new("MCU", "1"));
        resolver.add_stream_pin(p_b, 1, producer(115_200), PinRef::new("U1", "15"));

        let net_table = nets(2);
        let mut diags = Diagnostics::new();
        let routes = resolver.route_streams(&net_table, &mut diags);
        assert!(routes.is_empty(), "facing producers must not route");
        assert_eq!(
            diags.len(),
            1,
            "one finding per pair: {:?}",
            diags.findings()
        );
        assert!(diags.findings().iter().any(|f| matches!(
            f,
            Finding::StreamMismatch { producers, .. }
                if producers.contains(&PinRef::new("MCU", "1"))
                    && producers.contains(&PinRef::new("U1", "15"))
        )));
    }

    /// The paced in-flight queue is capped at [`STREAM_ROUTE_QUEUE_MAX`]:
    /// a producer writing more than the route absorbs sheds the overflow
    /// with a [`Finding::StreamOverrun`] naming the producer pin — the
    /// surface for a producer-vs-declared-baud mismatch — instead of
    /// growing an infinite TX buffer no UART has.
    #[rstest]
    fn paced_route_queue_overflow_sheds_with_a_finding() {
        let _g = lock_clock();
        virtual_clock::init(1.0, 1_000_000);
        let mut resolver = Resolver::new(2, Dsu::new(2));
        let p = resolver.add_endpoint(0, PinRef::new("U1", "15"), Some(high()));
        let c = resolver.add_endpoint(1, PinRef::new("U2", "16"), None);
        resolver.add_edge(0, 1, 47.0);
        resolver.add_stream_pin(p, 0, producer(300), PinRef::new("U1", "15"));
        resolver.add_stream_pin(c, 1, consumer(300), PinRef::new("U2", "16"));
        let handle = EngineHandle::spawn(resolver, nets(2), Box::new(QuasiStaticMna));

        // One write larger than the cap: the excess sheds, loudly.
        handle.link().send(Command::StreamWrite {
            endpoint: p,
            bytes: vec![0u8; STREAM_ROUTE_QUEUE_MAX + 7],
        });
        assert!(
            wait_for(
                || handle.findings().contains(&Finding::StreamOverrun {
                    producer: PinRef::new("U1", "15"),
                }),
                Duration::from_secs(5)
            ),
            "overflow must surface as StreamOverrun; got {} findings",
            handle.findings().len()
        );
    }
}
