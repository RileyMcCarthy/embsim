//! System assembly: boards + harnesses + scenario overrides + fault algebra.
//!
//! ```rust,ignore
//! let sys = System::new()
//!     .board("EdgeBoard", edge)
//!     .board("DS2Addon", ds2)
//!     .harness(harness)
//!     .scenario(Scenario::default()
//!         .jumper("DS2Addon.JP1", JumperState::Closed)
//!         .pin_detach("DS2Addon.U1.3"))
//!     .build()?;
//! ```
//!
//! **No implicit net-name merging across boards** — two boards both naming a
//! net `GND` share nothing until a harness connects them, grounds included.
//!
//! Two terminal operations share one assembly (and one resolution code path,
//! the crate-internal `engine::Resolver` — build-time analysis and live
//! resolution can never fork semantics):
//!
//! - [`System::build`] — the build-time analysis pass: resolve once, report
//!   findings, validate every component facade, then drop the components.
//! - [`System::start`] — the live path: spawn the single-writer net-engine
//!   thread, attach components with engine-wired I/O handles, and return a
//!   [`SystemHandle`] that owns both (clean engine shutdown on drop).

use std::collections::{HashMap, HashSet};
use std::fmt;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use crate::board::{validate_pin_declarations, Board, BoardError, PartClass};
use crate::cluster::QuasiStaticMna;
use crate::component::{
    clamp_branches, Branch, BuildTopology, Component, ComponentNetIo, DeclaredThresholds, Drive,
    DriveCapability, PinDecl, PinHandle, PinRole, PwlCurve, RegionTest, ResistorAt, SenseFrame,
};
use crate::diagnostics::{Diagnostics, Finding, RailDownReason, SenseKind};
use crate::engine::{
    ComponentId, CurrentTable, Dsu, EndpointId, EngineHandle, EngineLink, NetMove, ReadKind,
    RecordedCallback, Resolver, SenseLog, TerminalDrive, VoltsTable,
};
use crate::event_log::EventLog;
use crate::net::{Amps, Net, NetId, NetState, NetVolts, PinRef, TheveninDrive, Volts};
use crate::registry::{parse_passive_value, JumperState, PassiveKind};

/// How many rounds the build-time fixed point runs before it gives up and
/// reports [`Finding::BuildNotSettled`]. Each round replays the drives
/// components issued in response to the states the previous round delivered.
/// The deepest chain on the reference boards is the EC32MB power tree —
/// J203 → U401 → U402.IN → Common_LDOin → U501..U508 EN/IN, three senses
/// deep — so eight is headroom, not a budget: a system that needs more is
/// oscillating, and the bound is what turns that into a finding.
pub const BUILD_FIXED_POINT_BOUND: usize = 8;

// ============================================================
// Qualified net names
// ============================================================

/// Canonicalize a dotted `Board.NETNAME` reference the way the assembly
/// stored it: the board prefix is passed through verbatim and the net part —
/// everything after the FIRST `.` — goes through
/// [`crate::netlist::normalize_net_name`], so both the overline rewrite
/// (`~{RESET}` → `~RESET`) and the sheet-scoped leaf rewrite
/// (`/Sheet2/~{FOO}` → `/Sheet2/~FOO`) fire.
///
/// Splitting on the first `.` (not the last) is deliberate: board names hold
/// no dots, while KiCad net labels legitimately do.
///
/// Both name-keyed lookups — [`System::named_net`] (which resolves
/// `Scenario::net_stuck` targets) and [`SystemHandle::net_state`] — must use
/// this, or a caller naming a net exactly as the netlist spells it silently
/// fails to find a net that exists.
fn normalize_qualified_net_name(path: &str) -> String {
    match path.split_once('.') {
        Some((board, net)) => {
            format!("{board}.{}", crate::netlist::normalize_net_name(net))
        }
        None => crate::netlist::normalize_net_name(path),
    }
}

// ============================================================
// Harness endpoints
// ============================================================

/// A harness endpoint: `Board.Connector.Pin`, or the bare `Board.Pin` form
/// for bench rigs that aren't a designed PCB (`P2EVAL.P0`).
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct EndpointRef {
    /// System board name.
    pub board: String,
    /// Connector reference (`"J1"`); `None` for bare MCU-pin endpoints.
    pub connector: Option<String>,
    /// Pin identity on the connector (or bare pin name).
    pub pin: String,
}

impl EndpointRef {
    /// Parse a dotted endpoint: `"DS2Addon.J1.3"` or `"P2EVAL.P0"`.
    pub fn parse(s: &str) -> Result<Self, HarnessError> {
        let parts: Vec<&str> = s.split('.').collect();
        match parts.as_slice() {
            [board, pin] if !board.is_empty() && !pin.is_empty() => Ok(Self {
                board: board.to_string(),
                connector: None,
                pin: pin.to_string(),
            }),
            [board, connector, pin]
                if !board.is_empty() && !connector.is_empty() && !pin.is_empty() =>
            {
                Ok(Self {
                    board: board.to_string(),
                    connector: Some(connector.to_string()),
                    pin: pin.to_string(),
                })
            }
            _ => Err(HarnessError::BadEndpoint {
                endpoint: s.to_string(),
            }),
        }
    }
}

/// Electrical kind of a harness connection endpoint.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EndpointKind {
    /// Plain signal interconnect.
    Signal,
    /// A power source endpoint — bench rigs can source domains without a
    /// designed PCB (`from = "P2EVAL.3V3", kind = "power(3.3V)"`).
    Power {
        /// Sourced rail voltage.
        volts: Volts,
    },
}

/// One harness wire: connector-pin ↔ connector-pin.
#[derive(Debug, Clone, PartialEq)]
pub struct HarnessConnection {
    /// One end.
    pub from: EndpointRef,
    /// Other end.
    pub to: EndpointRef,
    /// Signal or power.
    pub kind: EndpointKind,
}

/// An inter-board harness: the only mechanism that merges nets across boards.
/// Deliberately wrong harnesses (swapped pins) are valid fixtures — the
/// `Contention`/`Floating` findings are the assertion targets.
///
/// `Harness::from_toml` is deferred: the `toml` crate is not in the
/// workspace's dependency tree, so harnesses are built via this plain Rust
/// builder API for now. Revisit if/when the workspace adopts a TOML
/// dependency.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Harness {
    connections: Vec<HarnessConnection>,
}

impl Harness {
    /// Empty harness.
    pub fn new() -> Self {
        Self::default()
    }

    /// Add a signal wire between two parsed endpoints.
    pub fn connect(mut self, from: EndpointRef, to: EndpointRef) -> Self {
        self.connections.push(HarnessConnection {
            from,
            to,
            kind: EndpointKind::Signal,
        });
        self
    }

    /// Add a signal wire between two dotted endpoint strings.
    pub fn connect_str(self, from: &str, to: &str) -> Result<Self, HarnessError> {
        Ok(self.connect(EndpointRef::parse(from)?, EndpointRef::parse(to)?))
    }

    /// Add a power wire: `from` sources the connected net at `volts`.
    pub fn power(mut self, from: EndpointRef, to: EndpointRef, volts: Volts) -> Self {
        self.connections.push(HarnessConnection {
            from,
            to,
            kind: EndpointKind::Power { volts },
        });
        self
    }

    /// All wires, in declaration order.
    pub fn connections(&self) -> &[HarnessConnection] {
        &self.connections
    }
}

// ============================================================
// Scenario + fault algebra
// ============================================================

/// Scenario-time DNP override state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum DnpState {
    /// Fit the component regardless of its netlist DNP marking.
    Populated,
    /// Remove the component from the built system.
    Absent,
}

/// One injected fault, defined in terms of graph primitives the netlist
/// actually has.
#[derive(Debug, Clone, PartialEq)]
pub enum Fault {
    /// Remove one node from its net — a lifted pin / cold joint
    /// (`"Board.Ref.Pin"`).
    PinDetach {
        /// Dotted pin endpoint.
        endpoint: String,
    },
    /// Union two nets (solder bridge, crossed probe).
    PinShort {
        /// Dotted pin endpoint.
        a: String,
        /// Dotted pin endpoint.
        b: String,
    },
    /// Add a Thevenin source to a net (stuck-at rail).
    NetStuck {
        /// Dotted net reference (`"Board.NETNAME"`).
        net: String,
        /// Rail voltage of the injected source.
        volts: Volts,
    },
}

/// Scenario overrides: switch and jumper states, DNP/value BOM changes,
/// injected faults.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Scenario {
    jumpers: Vec<(String, JumperState)>,
    switches: Vec<(String, usize, JumperState)>,
    value_overrides: Vec<(String, String)>,
    dnp_overrides: Vec<(String, DnpState)>,
    faults: Vec<Fault>,
}

impl Scenario {
    /// Set a jumper's state (`"DS2Addon.JP1"`) — the one-pole form of
    /// [`Scenario::switch`]: on a jumper it sets the jumper, on a switch it
    /// sets pole 0.
    pub fn jumper(mut self, reference: &str, state: JumperState) -> Self {
        self.jumpers.push((reference.to_string(), state));
        self
    }

    /// Set one pole of a switch (`"EC32MB.S301"`, pole `1`, closed). Poles
    /// are indexed from 0 in the order the registry declared them (so a DIP
    /// switch's printed position *n* is pole `n - 1`). A closed pole is a
    /// build-time identity union of its two pins' nets, exactly the merge
    /// [`Scenario::pin_short`] makes, and honours [`Scenario::pin_detach`]
    /// on either pin; an open pole is nothing. On a jumper, pole 0 is the
    /// jumper itself. A pole the part does not have fails the build with
    /// [`SystemError::UnknownSwitchPole`].
    pub fn switch(mut self, reference: &str, pole: usize, state: JumperState) -> Self {
        self.switches.push((reference.to_string(), pole, state));
        self
    }

    /// Detach one pin from its net (`"DS2Addon.U1.3"`).
    pub fn pin_detach(mut self, endpoint: &str) -> Self {
        self.faults.push(Fault::PinDetach {
            endpoint: endpoint.to_string(),
        });
        self
    }

    /// Short two pins' nets together.
    pub fn pin_short(mut self, a: &str, b: &str) -> Self {
        self.faults.push(Fault::PinShort {
            a: a.to_string(),
            b: b.to_string(),
        });
        self
    }

    /// Stick a net at a rail voltage.
    pub fn net_stuck(mut self, net: &str, volts: Volts) -> Self {
        self.faults.push(Fault::NetStuck {
            net: net.to_string(),
            volts,
        });
        self
    }

    /// Override a component's value (`"Board.R5"`, `"4k7"`).
    pub fn value_override(mut self, reference: &str, value: &str) -> Self {
        self.value_overrides
            .push((reference.to_string(), value.to_string()));
        self
    }

    /// Override a component's DNP state.
    pub fn dnp_override(mut self, reference: &str, state: DnpState) -> Self {
        self.dnp_overrides.push((reference.to_string(), state));
        self
    }

    /// Jumper overrides, in declaration order.
    pub fn jumpers(&self) -> &[(String, JumperState)] {
        &self.jumpers
    }

    /// Switch pole overrides, in declaration order, as `(reference, pole,
    /// state)`.
    pub fn switches(&self) -> &[(String, usize, JumperState)] {
        &self.switches
    }

    /// Value overrides, in declaration order.
    pub fn value_overrides(&self) -> &[(String, String)] {
        &self.value_overrides
    }

    /// DNP overrides, in declaration order.
    pub fn dnp_overrides(&self) -> &[(String, DnpState)] {
        &self.dnp_overrides
    }

    /// Injected faults, in declaration order.
    pub fn faults(&self) -> &[Fault] {
        &self.faults
    }
}

// ============================================================
// System
// ============================================================

/// System builder: named boards + bench components + harnesses + scenario.
#[derive(Debug, Default)]
pub struct System {
    boards: Vec<(String, Board)>,
    bench: Vec<BenchComponent>,
    harnesses: Vec<Harness>,
    scenario: Scenario,
    /// Determinism Oracle 1 (`crate::event_log`); disabled unless
    /// [`System::event_log`] was called.
    event_log: EventLog,
    /// Stepped-clock quiescence timeout override; `None` uses the engine
    /// default. See [`System::quiescence_timeout`].
    quiescence_timeout: Option<Duration>,
    /// Keep virtual time held after `start` (see [`System::hold_time`]).
    hold_time: bool,
}

/// A bench component: a bare [`Component`] added to the system without a
/// board netlist — the "bench rigs that aren't a designed PCB" case from the
/// design doc. Its declared pins become harness-addressable nets named
/// `"{name}.{pin}"` (the bare `P2EVAL.P0` endpoint form).
struct BenchComponent {
    name: String,
    component: Box<dyn Component>,
}

impl fmt::Debug for BenchComponent {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("BenchComponent")
            .field("name", &self.name)
            .finish()
    }
}

/// One registered component prepared for attach: its wiring (per-pin net +
/// drive endpoint) resolved against the merged system net table.
struct PreparedComponent {
    board: String,
    reference: String,
    component: Box<dyn Component>,
    pins: Vec<PreparedPin>,
}

/// One prepared pin: every identity it answers to, its global net, and its
/// drive endpoint (when the pin can drive and is not detached).
struct PreparedPin {
    number: String,
    name: Option<String>,
    net: usize,
    endpoint: Option<EndpointId>,
    /// A bidirectional pin idling released
    /// ([`PinDecl::reads_when_subscribed`]): an input until its owner
    /// drives it, whose sense subscription declares its net read (see
    /// [`crate::PinHandle`]'s field of the same name).
    reads_when_released: bool,
    /// The pin's declared thresholds and the net of the supply pin they
    /// are relative to ([`PinHandle::thresholds`]).
    declared: Option<DeclaredThresholds>,
    /// What the pin's sense is measured against: its declared reference's
    /// net ([`crate::Sense`]).
    frame: SenseFrame,
    /// The elements this pin terminates, as `(element index, sign)` (see
    /// [`crate::PinHandle::sense_current`]).
    branch_terms: Vec<(usize, f64)>,
    /// A `PowerOut` pin: its slot is its terminal's, and its current spans
    /// clusters — no current port, no instrument.
    terminal: bool,
    /// What the pin declares it can do to its net ([`PinDecl::can_source`],
    /// [`PinDecl::can_sink`]), checked against what it publishes.
    capability: DriveCapability,
}

/// One pin whose current the solve accounts for — it has a drive slot or
/// terminates a declared branch — addressable by its `Board.Ref.Pin` path
/// from [`BuiltSystem::pin_current`] / [`SystemHandle::pin_current`].
#[derive(Debug, Clone)]
struct CurrentPort {
    path: String,
    endpoint: Option<EndpointId>,
    branch_terms: Vec<(usize, f64)>,
}

impl CurrentPort {
    /// The current into the port from a published table: the slot's own
    /// current plus the branch currents into the pin — the same sum
    /// [`crate::PinHandle::sense_current`] makes.
    fn read(&self, table: &CurrentTable) -> Option<Amps> {
        let mut total: Option<Amps> = None;
        if let Some(endpoint) = self.endpoint {
            if let Some(Some(amps)) = table.endpoints.get(endpoint.0) {
                total = Some(*amps);
            }
        }
        for (element, sign) in &self.branch_terms {
            if let Some(Some(amps)) = table.elements.get(*element) {
                total = Some(total.unwrap_or(0.0) + sign * amps);
            }
        }
        total
    }
}

/// The elements and current ports a system carries, by path, so a built
/// system and a live one answer [`BuiltSystem::branch_current`] and
/// [`BuiltSystem::pin_current`] the same way.
#[derive(Debug, Clone, Default)]
struct CurrentPaths {
    /// `Board.Reference` of the part declaring each element, by element
    /// index.
    elements: Vec<String>,
    /// Every pin with a current.
    ports: Vec<CurrentPort>,
}

impl CurrentPaths {
    /// The current through the one branch of the part at `path` — a diode,
    /// an LED — from `table`; `None` for a part with no branch or more than
    /// one, or one whose cluster the last pass did not solve.
    fn branch_current(&self, table: &CurrentTable, path: &str) -> Option<Amps> {
        let mut matches = self
            .elements
            .iter()
            .enumerate()
            .filter(|(_, p)| p.as_str() == path)
            .map(|(index, _)| index);
        let index = matches.next()?;
        if matches.next().is_some() {
            return None;
        }
        table.elements.get(index).copied().flatten()
    }

    /// The current into the pin at `path` from `table`.
    fn pin_current(&self, table: &CurrentTable, path: &str) -> Option<Amps> {
        self.ports
            .iter()
            .find(|port| port.path == path)
            .and_then(|port| port.read(table))
    }
}

/// One pin of a part, as the build lints see it.
struct PinLint {
    number: String,
    role: PinRole,
    net: usize,
    /// A signal pin that sinks and cannot source: an open drain, whose
    /// net needs a pull-up ([`Finding::OpenDrainWithoutPullUp`]).
    open_drain: bool,
    /// A pin that can pull its net up: a signal pin that sources. An
    /// input's declared port is its own load, not a pull-up — the
    /// AM26LV32's 12 kΩ to 0.83 V holds an open input inside every logic
    /// band, and no open drain's high comes from it.
    pulls_up: bool,
}

impl PinLint {
    fn of(pin: &PinDecl, net: usize) -> Self {
        Self {
            number: pin.number.to_string(),
            role: pin.role,
            net,
            open_drain: pin.role == PinRole::Signal && pin.can_sink && !pin.can_source,
            pulls_up: pin.role == PinRole::Signal && pin.can_source,
        }
    }
}

/// Whether the build's domain lints read a pin's declared reference: a
/// power pin's — the supply a domain is measured against. A signal pin
/// declares one for its thresholds and its sense; a signal measured
/// against a ground nothing holds is that ground's supply pins' finding,
/// not one more per signal.
fn is_power(pin: &PinDecl) -> bool {
    matches!(pin.role, PinRole::PowerIn | PinRole::PowerOut)
}

/// One part, as the build lints see it: its path, its pins on their global
/// nets, and its power pins' declared references as pin numbers.
struct PartLint {
    path: String,
    pins: Vec<PinLint>,
    /// `(pin number, reference pin number)`.
    references: Vec<(String, String)>,
}

/// What the build lints read after the fixed point (`NODES.md` §8 phase 4,
/// "the build lints"): the fitted two-pin capacitors by the nets they
/// bridge, the mechanical nodes' pads, and every registered part's pins and
/// references.
#[derive(Default)]
struct LintInputs {
    /// Global nets at the two ends of every fitted two-pin capacitor.
    capacitors: Vec<(usize, usize)>,
    /// The harness supplies and `net_stuck` faults, as `(net, volts)`.
    supplies: Vec<(usize, Volts)>,
    /// `(Board.Reference, global net)` of every mechanical node's pad.
    mechanical: Vec<(String, usize)>,
    parts: Vec<PartLint>,
}

/// Output of the shared assembly pass: the merged net table, the populated
/// resolver (one code path for build-time analysis and live resolution),
/// the components ready to attach, the current paths, the build-time
/// topology a component may read at attach, and the lint inputs.
struct Assembly {
    nets: Vec<Net>,
    resolver: Resolver,
    components: Vec<PreparedComponent>,
    paths: CurrentPaths,
    topology: Arc<BuildTopology>,
    lints: LintInputs,
}

impl System {
    /// Empty system.
    pub fn new() -> Self {
        Self::default()
    }

    /// Add a named board.
    pub fn board(mut self, name: &str, board: Board) -> Self {
        self.boards.push((name.to_string(), board));
        self
    }

    /// Add a named **bench component** — a bare [`Component`] with no board
    /// netlist (a bench rig that isn't a designed PCB, e.g. an MCU dev
    /// board or a transducer plugged straight into a harness).
    ///
    /// Each declared pin gets its own global net named `"{name}.{pin}"`
    /// (declared number and alias both resolve), addressable as a bare
    /// harness endpoint (`"P2EVAL.P0"`). Pins get the same electrical
    /// descriptors as netlist-registered pins — drives and senses all
    /// participate in resolution. Endpoints
    /// under the component's name that do not match a declared pin
    /// synthesize a fresh external net, exactly like endpoints on unknown
    /// names (so a bench rig can also source rails the component facade
    /// does not declare, e.g. `"P2EVAL.3V3"` as a power endpoint).
    pub fn component(mut self, name: &str, component: Box<dyn Component>) -> Self {
        self.bench.push(BenchComponent {
            name: name.to_string(),
            component,
        });
        self
    }

    /// Enable the engine **event log** — determinism Oracle 1
    /// (`DETERMINISM.md`, "Proving it: determinism testing"; see
    /// [`crate::event_log`] for the record vocabulary and the normalization
    /// contract). Off by default, and zero cost when off.
    ///
    /// Read the log back from [`SystemHandle::event_log`] after
    /// [`System::start`]. The [`System::build`] analysis path has no engine and
    /// so records nothing.
    pub fn event_log(mut self) -> Self {
        self.event_log = EventLog::enabled();
        self
    }

    /// Override how long the engine waits for every registered
    /// `embsim_core::virtual_clock` actor to park before reporting
    /// [`Finding::QuiescenceTimeout`] and advancing anyway.
    ///
    /// **Stepped clock mode only**; ignored while free-running. The default
    /// ([`crate::engine::STEPPED_QUIESCENCE_TIMEOUT`]) is deliberately
    /// generous — reaching it means a defect, and the finding says the run is
    /// no longer reproducible. Raise it for a consumer whose actors do heavy
    /// work between parks; lower it in a test that means to provoke the
    /// stall.
    pub fn quiescence_timeout(mut self, timeout: Duration) -> Self {
        self.quiescence_timeout = Some(timeout);
        self
    }

    /// Keep virtual time **held** after [`System::start`] returns, until
    /// [`SystemHandle::release_time`] is called.
    ///
    /// With the hold kept, the live engine still applies every attach-time
    /// drive and delivers every sense — the system rests where its parts
    /// left it — but no scheduled wake fires, so the state the handle reads
    /// is the system's **before its first wake**: what a rail with a
    /// soft-start, an oscillator with a start-up time or a gate with a
    /// propagation delay has not yet changed. That is the state
    /// [`System::build`] analyzes, and the two are compared with the hold
    /// in place (`board/tests/build_fixed_point.rs`). Release it to run.
    ///
    /// The hold is the same in both pacing modes: virtual time is only the
    /// counter the engine advances (never wall time), and the release gate
    /// sits before the engine's one advance, so a paced (free-running)
    /// system started held is held exactly as an unpaced one is.
    pub fn hold_time(mut self) -> Self {
        self.hold_time = true;
        self
    }

    /// Add an inter-board harness.
    pub fn harness(mut self, harness: Harness) -> Self {
        self.harnesses.push(harness);
        self
    }

    /// Apply scenario overrides (last call wins).
    pub fn scenario(mut self, scenario: Scenario) -> Self {
        self.scenario = scenario;
        self
    }

    /// Assemble the system: merge harness-connected nets, apply the scenario
    /// (jumpers, BOM overrides, fault algebra), then run the **full
    /// build-time resolution pass** so never-driven nets are reported
    /// `Floating` (and unsourced power nets `PowerNetUnsourced`, …) to their
    /// sensing components immediately, before any traffic. Components are
    /// attached with inert I/O handles for facade validation and dropped —
    /// use [`System::start`] to keep them running against the live engine.
    pub fn build(self) -> Result<BuiltSystem, SystemError> {
        let Assembly {
            mut nets,
            mut resolver,
            components,
            paths,
            topology,
            lints,
        } = self.assemble()?;

        let mut diagnostics = Diagnostics::new();
        resolver.resolve(&mut nets, &mut diagnostics, &QuasiStaticMna);

        // Inert attach: sense() reads this build-resolved snapshot; drives
        // are recorded, sense subscriptions are delivered once and recorded,
        // schedules are traced and dropped.
        let states: Arc<Mutex<Vec<NetState>>> =
            Arc::new(Mutex::new(nets.iter().map(|n| n.state).collect()));
        let volts = Arc::new(VoltsTable::of(nets.iter().map(|n| n.volts)));
        let currents: Arc<Mutex<CurrentTable>> = Arc::new(Mutex::new(resolver.current_table()));
        let recorded_drives = Arc::new(Mutex::new(Vec::new()));
        let recorded_senses = SenseLog::default();
        // The build holds the sense log's one strong reference: the inert
        // link inside every handle the components keep sees it weakly, so
        // the callbacks the log records (which capture those handles) are
        // freed with the log when the build is done (`SenseLog`).
        let link = EngineLink::inert(
            (Arc::clone(&states), Arc::clone(&volts)),
            Arc::clone(&currents),
            Arc::clone(&recorded_drives),
            &recorded_senses,
        );
        let mut attached = Vec::with_capacity(components.len());
        for mut prepared in components {
            let io =
                ComponentNetIo::wired(handle_entries(&prepared.pins, &link), None, link.clone())
                    .with_topology(Arc::clone(&topology));
            prepared
                .component
                .attach(io)
                .map_err(|error| SystemError::Board {
                    name: prepared.board.clone(),
                    error: BoardError::Attach {
                        reference: prepared.reference.clone(),
                        error,
                    },
                })?;
            // Kept alive through the fixed point below: the sense callbacks
            // recorded at attach may hold handles into the component.
            attached.push(prepared.component);
        }

        // The build fixed point. Apply the drives components issued during
        // attach, resolve again, deliver the states that changed to the
        // senses that registered for them, and repeat while the components
        // answer with a different drive — so the published states and
        // findings describe the system as its components actually idle it,
        // the way the live engine would have settled it before its first
        // wake. Without the replay a component that *releases* a
        // push-pull output (an end switch with an open contact, an
        // open-drain output) is analyzed as if it drove its declared
        // idle-high;
        // without the iteration a chain of sense→drive components (the
        // GPIO bridge driving its power-on level, an isolator driving at the
        // rail it senses, a power tree three senses deep) is analyzed one
        // hop short of where it rests.
        //
        // Bounded: a system that is still changing after
        // `BUILD_FIXED_POINT_BOUND` rounds is oscillating, and the snapshot
        // it gets is the last round's, marked `Finding::BuildNotSettled`.
        let mut passes = 0;
        let mut unsettled: Vec<String> = Vec::new();
        // Nets a released bidirectional pad subscribed to are read from
        // that subscription on: each is declared a digital sense once, the
        // way the live engine's `Command::DeclareRead` does, and the
        // declaration is a change the next pass reports (a floating one is
        // `Finding::FloatingSense`). A current instrument's net is declared
        // an instrument the same way, so its cluster solves and the
        // current it reads is the one the live engine would produce — and
        // nothing is reported for it. Subscriptions made from inside a
        // delivered callback are caught on the round after.
        let mut declared_reads: Vec<(usize, ReadKind)> = Vec::new();
        let settled = loop {
            let idle_drives = std::mem::take(&mut *recorded_drives.lock().expect("never poisoned"));
            let mut changed = false;
            for (endpoint, drive) in idle_drives {
                changed |= resolver.set_drive(endpoint, drive);
            }
            let reads: Vec<(usize, ReadKind)> = recorded_senses
                .0
                .lock()
                .expect("never poisoned")
                .iter()
                .filter_map(|sense| match &sense.callback {
                    RecordedCallback::State(_) if sense.reads => {
                        Some((sense.net.0, ReadKind::Digital))
                    }
                    RecordedCallback::State(_) => None,
                    RecordedCallback::Current { .. } => Some((sense.net.0, ReadKind::Instrument)),
                })
                .collect();
            for read in reads {
                if !declared_reads.contains(&read) {
                    declared_reads.push(read);
                    let (net, kind) = read;
                    match kind {
                        ReadKind::Digital => resolver.add_digital_sense(net),
                        ReadKind::Instrument => resolver.add_current_instrument(net),
                    }
                    changed = true;
                }
            }
            if !changed {
                break true;
            }

            let previous: Vec<(NetState, NetVolts)> =
                nets.iter().map(|n| (n.state, n.volts)).collect();
            diagnostics = Diagnostics::new();
            resolver.resolve(&mut nets, &mut diagnostics, &QuasiStaticMna);
            *states.lock().expect("never poisoned") = nets.iter().map(|n| n.state).collect();
            for (i, net) in nets.iter().enumerate() {
                volts.store(i, net.volts);
            }
            *currents.lock().expect("never poisoned") = resolver.current_table();
            // What moved, as the live engine's change gate sees it: a state,
            // or only the voltage behind it (`NetMove`) — a sense is
            // handed the voltage, so both deliver; a net whose state did
            // not change has not moved for the fixed point's bound.
            let moved: Vec<(usize, NetMove)> = (0..nets.len())
                .filter_map(|i| Some((i, NetMove::of(&previous[i].0, &previous[i].1, &nets[i])?)))
                .collect();
            unsettled = moved
                .iter()
                .filter(|(_, how)| *how == NetMove::State)
                .map(|&(i, _)| nets[i].name.clone())
                .collect();
            if passes == BUILD_FIXED_POINT_BOUND {
                // Resolved so the snapshot matches the drive table, but not
                // delivered: the components would only answer again.
                break false;
            }
            passes += 1;

            // Deliver changed states in the order the live engine does: net
            // index ascending, subscribers in registration order; then the
            // subscriptions whose reference or supply moved and whose own
            // net did not (`EngineCore::deliver_senses`); then the current
            // instruments whose reading changed. No lock is held across a
            // callback: it may sense (the states lock) and drive (the drive
            // log); a sense it registers now is appended behind the ones
            // that exist.
            let mut senses =
                std::mem::take(&mut *recorded_senses.0.lock().expect("never poisoned"));
            let delivery = |sense: &crate::engine::RecordedSense| crate::engine::Delivery {
                state: nets[sense.net.0].state,
                node: nets[sense.net.0].volts,
                reference: sense.reference.map(|r| nets[r.0].volts),
            };
            for &(i, _) in &moved {
                for sense in &senses {
                    if let (true, RecordedCallback::State(callback)) =
                        (sense.net.0 == i, &sense.callback)
                    {
                        callback(&delivery(sense));
                    }
                }
            }
            let is_moved = |net: usize| moved.binary_search_by_key(&net, |&(i, _)| i).is_ok();
            for &(net, _) in &moved {
                for sense in &senses {
                    // Its reference or its supply moved, its own net did
                    // not: delivered once, at the lowest such net.
                    let first = sense
                        .reference
                        .into_iter()
                        .chain(sense.supply)
                        .map(|dependency| dependency.0)
                        .filter(|&dependency| is_moved(dependency))
                        .min();
                    if let (Some(first), RecordedCallback::State(callback)) =
                        (first, &sense.callback)
                    {
                        if first == net && !is_moved(sense.net.0) {
                            callback(&delivery(sense));
                        }
                    }
                }
            }
            for sense in &mut senses {
                if let RecordedCallback::Current {
                    handle,
                    callback,
                    last,
                } = &mut sense.callback
                {
                    let now = handle.sense_current();
                    // Bitwise after folding -0.0 to 0.0, as the live engine
                    // compares (`engine.rs` `same_current`).
                    let same = match (*last, now) {
                        (None, None) => true,
                        (Some(a), Some(b)) => (a + 0.0).total_cmp(&(b + 0.0)).is_eq(),
                        _ => false,
                    };
                    if !same {
                        *last = now;
                        callback(now);
                    }
                }
            }
            let mut log = recorded_senses.0.lock().expect("never poisoned");
            let late = std::mem::replace(&mut *log, senses);
            log.extend(late);
        };
        if !settled {
            diagnostics.report(Finding::BuildNotSettled {
                passes,
                nets: unsettled,
            });
        }
        // Build-time analysis: every component validated its facade and is
        // dropped here, after the callbacks that reach into it; System::start
        // keeps them. The sense log is the one strong owner of the recorded
        // callbacks (the links in the handles they capture hold it weakly),
        // so dropping it here frees them and everything they captured.
        drop(recorded_senses);
        drop(link);
        drop(attached);

        let roots = resolver.identity_roots(nets.len());
        // The build lints, over the settled snapshot (`NODES.md` §8 phase
        // 4): a rail that sources nothing, a domain measured against
        // nothing, a supply pin no capacitor decouples, a mechanical pad
        // a pin drives. Build-time analysis only — they read the settled
        // states and the declarations, and the live engine re-derives
        // nothing of them.
        lint_build(&nets, &roots, &mut resolver, &lints, &mut diagnostics);
        let cluster_roots = resolver.cluster_roots(nets.len());
        let escalated_solves = resolver.escalated_solves();
        let currents = resolver.current_table();
        // The build path resolves a fixed number of times, so logging the
        // standing set once here is the whole story (`Diagnostics::report` is
        // silent by design — see its type docs).
        diagnostics.log_all();
        Ok(BuiltSystem {
            nets,
            diagnostics,
            roots,
            cluster_roots,
            escalated_solves,
            currents,
            paths,
        })
    }

    /// Assemble the system and **start the live net engine**: the
    /// single-writer engine thread takes ownership of all net state (initial
    /// resolution pass included, so findings are populated before any
    /// traffic), and every registered component attaches with an I/O handle
    /// whose drives, sense subscriptions, and schedules route to the engine.
    ///
    /// The returned [`SystemHandle`] owns the components and the engine;
    /// dropping it shuts the engine down cleanly (shutdown message + join —
    /// see [`crate::engine`] for why joining cannot deadlock with in-flight
    /// senses).
    pub fn start(self) -> Result<SystemHandle, SystemError> {
        let event_log = self.event_log.clone();
        let quiescence_timeout = self.quiescence_timeout;
        let hold_time = self.hold_time;
        let Assembly {
            nets,
            resolver,
            components,
            paths,
            topology,
            lints: _,
        } = self.assemble()?;

        let net_names: Vec<String> = nets.iter().map(|n| n.name.clone()).collect();
        let engine = EngineHandle::spawn(
            resolver,
            nets,
            Box::new(QuasiStaticMna),
            event_log,
            quiescence_timeout,
        );
        let link = engine.link();

        let mut attached: Vec<(String, Box<dyn Component>)> = Vec::new();
        for (index, mut prepared) in components.into_iter().enumerate() {
            let io = ComponentNetIo::wired(
                handle_entries(&prepared.pins, &link),
                Some(ComponentId(index)),
                link.clone(),
            )
            .with_topology(Arc::clone(&topology));
            if let Err(error) = prepared.component.attach(io) {
                let error = SystemError::Board {
                    name: prepared.board.clone(),
                    error: BoardError::Attach {
                        reference: prepared.reference.clone(),
                        error,
                    },
                };
                // The same drop order SystemHandle documents as
                // load-bearing must hold on this path too: components
                // (including the failing one — it may have registered
                // callbacks or spawned protocol threads before erroring)
                // must never be dropped while the engine thread is still
                // delivering callbacks. Shut the engine down first.
                attached.push((prepared.reference, prepared.component));
                drop(engine);
                drop(attached);
                return Err(error);
            }
            attached.push((prepared.reference, prepared.component));
        }

        // Every component is attached and the engine is live: let components
        // begin execution they own (MCU firmware entries). Runs strictly
        // after the attach loop so the first instruction of any spawned
        // entry observes a fully-wired system (the init-ordering contract in
        // BOARD_ENGINE.md "The MCU as a component").
        for (_, component) in attached.iter_mut() {
            component.start();
        }

        // The system is assembled. Release the engine's hold on virtual time:
        // nothing may advance until every component has registered its
        // schedules, or a second component's period would be anchored at a
        // different instant from run to run. A caller that asked to keep
        // the hold releases it through `SystemHandle::release_time`.
        if !hold_time {
            engine.release_time();
        }

        Ok(SystemHandle {
            engine,
            net_names,
            components: attached,
            paths,
        })
    }

    /// The shared assembly pass: merge harness-connected nets, apply the
    /// scenario, register every electrical descriptor with the resolver, and
    /// prepare registered components for attach. `build` and `start` differ
    /// only in what they do with the result.
    fn assemble(mut self) -> Result<Assembly, SystemError> {
        // -- duplicate-name gate ------------------------------------------
        let mut seen = HashSet::new();
        for (name, _) in &self.boards {
            if !seen.insert(name.clone()) {
                return Err(SystemError::DuplicateBoard { name: name.clone() });
            }
        }
        // Bench components share the boards' namespace: a bare "Name.Pin"
        // endpoint on a *known board* name is an error, so a collision would
        // make the bench pins unreachable from any harness.
        for bench in &self.bench {
            if !seen.insert(bench.name.clone()) {
                return Err(SystemError::DuplicateComponent {
                    name: bench.name.clone(),
                });
            }
        }

        // -- global net table ---------------------------------------------
        // Global net = (board index, board-local NetId), flattened densely.
        // No implicit name merging: only harness wires and pin_short faults
        // union nets across (or within) boards.
        let mut nets: Vec<Net> = Vec::new();
        let mut base_of_board: Vec<usize> = Vec::new();
        for (bname, board) in &self.boards {
            base_of_board.push(nets.len());
            for net in &board.nets {
                let mut qualified = net.clone();
                qualified.id = NetId(nets.len());
                qualified.name = format!("{bname}.{name}", name = net.name);
                nets.push(qualified);
            }
        }

        // -- bench-component pin nets ---------------------------------------
        // Every declared pin of a bench component gets its own global net,
        // pre-seeded into the external-net table (under the pin number AND
        // its alias) so a bare harness endpoint ("P2EVAL.P0") resolves to
        // the live pin instead of synthesizing a disconnected net.
        let mut external_nets: HashMap<String, usize> = HashMap::new();
        // Per bench component, per declared pin: its global net index.
        let mut bench_pin_nets: Vec<Vec<usize>> = Vec::new();
        for bench in &self.bench {
            // No netlist facade to validate, but the same declarations to
            // honour: an idle drive on a pin without a slot, a reference
            // naming no declared pin, thresholds the pin cannot hold are
            // refused here exactly as `Board::from_netlist` refuses them
            // for a netlist part.
            validate_pin_declarations(&bench.name, bench.component.pins()).map_err(|error| {
                SystemError::Board {
                    name: bench.name.clone(),
                    error,
                }
            })?;
            let mut pin_nets = Vec::new();
            for pin in bench.component.pins() {
                let idx = nets.len();
                let name = format!("{}.{}", bench.name, pin.number);
                if external_nets.insert(name.clone(), idx).is_some() {
                    return Err(SystemError::DuplicateComponent { name });
                }
                if let Some(alias) = pin.name {
                    let alias_name = format!("{}.{alias}", bench.name);
                    if external_nets.insert(alias_name.clone(), idx).is_some() {
                        return Err(SystemError::DuplicateComponent { name: alias_name });
                    }
                }
                nets.push(Net {
                    id: NetId(idx),
                    name,
                    nodes: vec![PinRef::new(bench.name.clone(), pin.number)],
                    state: NetState::Floating,
                    volts: crate::net::NetVolts::default(),
                });
                pin_nets.push(idx);
            }
            bench_pin_nets.push(pin_nets);
        }

        let mut dsu = Dsu::new(nets.len());

        // (board name -> index) and ((board, PinRef) -> global net) lookups.
        let board_index: HashMap<String, usize> = self
            .boards
            .iter()
            .enumerate()
            .map(|(i, (n, _))| (n.clone(), i))
            .collect();
        let mut net_of_pin: HashMap<(usize, PinRef), usize> = HashMap::new();
        for (bi, (_, board)) in self.boards.iter().enumerate() {
            for (li, net) in board.nets.iter().enumerate() {
                for node in &net.nodes {
                    net_of_pin.insert((bi, node.clone()), base_of_board[bi] + li);
                }
            }
        }

        // -- scenario: BOM overrides + jumpers ----------------------------
        let mut detached: HashSet<(usize, PinRef)> = HashSet::new();
        {
            // Jumpers and switches share one mechanism: a jumper is a
            // one-pole switch, so `jumper(ref, s)` is `switch(ref, 0, s)`.
            let positions: Vec<(String, usize, JumperState)> = self
                .scenario
                .jumpers()
                .iter()
                .map(|(path, state)| (path.clone(), 0, *state))
                .chain(self.scenario.switches().iter().cloned())
                .collect();
            let dnp_overrides = self.scenario.dnp_overrides().to_vec();
            let value_overrides = self.scenario.value_overrides().to_vec();

            for (path, pole, state) in &positions {
                let (bi, reference) = split_board_ref(path, &board_index).ok_or_else(|| {
                    SystemError::UnknownEndpoint {
                        endpoint: path.clone(),
                    }
                })?;
                let record = self.boards[bi]
                    .1
                    .records
                    .iter_mut()
                    .find(|r| r.reference == reference)
                    .ok_or_else(|| SystemError::UnknownEndpoint {
                        endpoint: path.clone(),
                    })?;
                let poles = match &mut record.class {
                    PartClass::Jumper { state: s } => {
                        if *pole == 0 {
                            *s = *state;
                            continue;
                        }
                        1
                    }
                    PartClass::Switch { poles } => {
                        if let Some(p) = poles.get_mut(*pole) {
                            p.state = *state;
                            continue;
                        }
                        poles.len()
                    }
                    _ => 0,
                };
                return Err(SystemError::UnknownSwitchPole {
                    reference: path.clone(),
                    pole: *pole,
                    poles,
                });
            }
            for (path, dnp) in &dnp_overrides {
                let (bi, reference) = split_board_ref(path, &board_index).ok_or_else(|| {
                    SystemError::UnknownEndpoint {
                        endpoint: path.clone(),
                    }
                })?;
                if let Some(record) = self.boards[bi]
                    .1
                    .records
                    .iter_mut()
                    .find(|r| r.reference == reference)
                {
                    record.fitted = matches!(dnp, DnpState::Populated);
                }
            }
            for (path, value) in &value_overrides {
                let (bi, reference) = split_board_ref(path, &board_index).ok_or_else(|| {
                    SystemError::UnknownEndpoint {
                        endpoint: path.clone(),
                    }
                })?;
                if let Some(record) = self.boards[bi]
                    .1
                    .records
                    .iter_mut()
                    .find(|r| r.reference == reference)
                {
                    if let PartClass::Passive { value: v, .. } = &mut record.class {
                        *v = parse_passive_value(value);
                    }
                }
            }
        }

        // -- external endpoints + power sources ----------------------------
        // sources[global net root (pre-resolution index)] accumulate later;
        // collected as (net index, volts, is_power).
        let mut power_sources: Vec<(usize, Volts)> = Vec::new();
        let mut stuck_sources: Vec<(usize, Volts)> = Vec::new();

        // Bench-rig externals not matching a bench-component pin ("BENCH.3V3")
        // get synthetic nets on demand, added to `external_nets` above.
        let harnesses = std::mem::take(&mut self.harnesses);
        for harness in &harnesses {
            for conn in harness.connections() {
                let a = self.resolve_endpoint(
                    &conn.from,
                    &board_index,
                    &net_of_pin,
                    &mut nets,
                    &mut external_nets,
                )?;
                let b = self.resolve_endpoint(
                    &conn.to,
                    &board_index,
                    &net_of_pin,
                    &mut nets,
                    &mut external_nets,
                )?;
                if dsu.len() < nets.len() {
                    dsu.grow(nets.len());
                }
                dsu.union(a, b);
                if let EndpointKind::Power { volts } = conn.kind {
                    power_sources.push((a, volts));
                }
            }
        }

        // -- scenario: fault algebra ---------------------------------------
        for fault in self.scenario.faults().to_vec() {
            match fault {
                Fault::PinDetach { endpoint } => {
                    let (bi, pin) = split_board_pin(&endpoint, &board_index).ok_or_else(|| {
                        SystemError::UnknownEndpoint {
                            endpoint: endpoint.clone(),
                        }
                    })?;
                    detached.insert((bi, pin));
                }
                Fault::PinShort { a, b } => {
                    let ra = self.pin_net(&a, &board_index, &net_of_pin)?;
                    let rb = self.pin_net(&b, &board_index, &net_of_pin)?;
                    dsu.union(ra, rb);
                }
                Fault::NetStuck { net, volts } => {
                    let idx = self.named_net(&net, &nets)?;
                    stuck_sources.push((idx, volts));
                }
            }
        }

        // -- closed switch poles and inductors: identity unions -------------
        // A closed pole is the same merge a `pin_short` fault makes — its two
        // pins' nets become one electrical node — and it honours a detached
        // pin the same way a passive edge does: a lifted contact conducts
        // nothing. Membership is fixed at build (`NODES.md` §2, rule 3), so
        // this is the whole of a switch's electrical existence. An inductor
        // is the same merge: the DC short it has always been (`NODES.md` §2,
        // the Inductor row), with a closed pole's semantics rather than a
        // 0 Ω conduction edge's — so a regulator's output inductor makes the
        // rail *the terminal's node*, a boundary of its loads' clusters,
        // where a 0 Ω edge from the switch node made the rail a member of
        // every load's cluster (the Edge board's `+3.3V` with its nine LED
        // chains: 19 roots; `NODES.md` §8, the phase-4 record).
        for (bi, (_, board)) in self.boards.iter().enumerate() {
            for record in &board.records {
                if !record.fitted {
                    continue;
                }
                let poles: Vec<(String, String)> = match &record.class {
                    PartClass::Switch { poles } => poles
                        .iter()
                        .filter(|p| p.state == JumperState::Closed)
                        .map(|p| (p.a.clone(), p.b.clone()))
                        .collect(),
                    PartClass::Passive {
                        kind: PassiveKind::Inductor,
                        ..
                    } if record.pins.len() == 2 => {
                        vec![(record.pins[0].clone(), record.pins[1].clone())]
                    }
                    _ => continue,
                };
                for (pin_a, pin_b) in poles {
                    let a = (bi, PinRef::new(record.reference.clone(), pin_a));
                    let b = (bi, PinRef::new(record.reference.clone(), pin_b));
                    if detached.contains(&a) || detached.contains(&b) {
                        continue;
                    }
                    if let (Some(&na), Some(&nb)) = (net_of_pin.get(&a), net_of_pin.get(&b)) {
                        dsu.union(na, nb);
                    }
                }
            }
        }

        // -- electrical descriptors ----------------------------------------
        let mut resolver = Resolver::new(nets.len(), dsu);
        // The identity roots are final here — every harness merge and
        // closed pole is in the DSU — so the build-time topology a part
        // reads at attach (`ComponentNetIo::resistors_at`, `::node`) and
        // the lint inputs are collected against them as the descriptors
        // are registered.
        let root_of = resolver.identity_roots(nets.len());
        let mut resistors: HashMap<usize, Vec<ResistorAt>> = HashMap::new();
        let mut lints = LintInputs::default();
        for (idx, volts) in power_sources {
            resolver.add_power_source(idx, volts);
            lints.supplies.push((idx, volts));
        }
        for (idx, volts) in stuck_sources {
            resolver.add_stuck_source(idx, volts);
            lints.supplies.push((idx, volts));
        }

        let mut endpoints: HashMap<(usize, PinRef), EndpointId> = HashMap::new();
        // The elements each pin terminates, as `(element index, sign)`, and
        // the paths of every element and current port (`CurrentPaths`).
        let mut branch_terms: HashMap<(usize, PinRef), Vec<(usize, f64)>> = HashMap::new();
        let mut paths = CurrentPaths::default();
        // Register one declared branch of a part: its pins resolved to nets
        // through `net_of`, a detached terminal opening the branch and a
        // detached control leaving the channel uncontrolled (always off).
        // Returns the element index, or nothing when a terminal is missing.
        let register_branch = |resolver: &mut Resolver,
                               paths: &mut CurrentPaths,
                               path: &str,
                               a: &str,
                               b: &str,
                               curve: PwlCurve,
                               control: Option<(&str, RegionTest)>,
                               net_of: &dyn Fn(&str) -> Option<usize>|
         -> Option<(usize, String, String)> {
            let (a_net, b_net) = (net_of(a)?, net_of(b)?);
            let control = control.and_then(|(pin, test)| net_of(pin).map(|net| (net, test)));
            let index = resolver.add_element(a_net, b_net, curve, control, path.to_string());
            debug_assert_eq!(index, paths.elements.len());
            paths.elements.push(path.to_string());
            Some((index, a.to_string(), b.to_string()))
        };
        for (bi, (bname, board)) in self.boards.iter().enumerate() {
            for record in &board.records {
                if !record.fitted {
                    continue;
                }
                match &record.class {
                    PartClass::Passive { kind, value } => {
                        let conducts = match kind {
                            PassiveKind::Resistor => value.is_some(),
                            // A DC short: an identity union, made above with
                            // the closed poles — no edge.
                            PassiveKind::Inductor => false,
                            // DC open in the build-time pass.
                            PassiveKind::Capacitor | PassiveKind::Diode | PassiveKind::Led => false,
                        };
                        let ends = (record.pins.len() == 2)
                            .then(|| {
                                let a = (
                                    bi,
                                    PinRef::new(record.reference.clone(), record.pins[0].clone()),
                                );
                                let b = (
                                    bi,
                                    PinRef::new(record.reference.clone(), record.pins[1].clone()),
                                );
                                if detached.contains(&a) || detached.contains(&b) {
                                    return None;
                                }
                                Some((*net_of_pin.get(&a)?, *net_of_pin.get(&b)?))
                            })
                            .flatten();
                        match (kind, ends) {
                            // The decoupling lint's input: every fitted
                            // two-pin capacitor by the nets it bridges,
                            // value or no value.
                            (PassiveKind::Capacitor, Some(ends)) => lints.capacitors.push(ends),
                            // The topology query's input: a resistor with
                            // a value, on both nodes it touches — with the
                            // far end reported as a node (its root), so a
                            // part can compare it with its own pins'.
                            (PassiveKind::Resistor, Some((a, b))) => {
                                if let Some(ohms) = value {
                                    let (ra, rb) = (root_of[a], root_of[b]);
                                    if ra != rb {
                                        let reference = format!("{bname}.{}", record.reference);
                                        for (here, there) in [(ra, rb), (rb, ra)] {
                                            resistors.entry(here).or_default().push(ResistorAt {
                                                reference: reference.clone(),
                                                ohms: *ohms,
                                                far: NetId(there),
                                            });
                                        }
                                    }
                                }
                            }
                            _ => {}
                        }
                        // A capacitor with a parsed value is an AC path for
                        // rate routing — a step clock or an oscillator's
                        // output crosses it — and a DC open everywhere else
                        // (`NODES.md` §2; it is never a conduction edge).
                        if *kind == PassiveKind::Capacitor && record.pins.len() == 2 {
                            if let Some(farads) = value {
                                self.coupling_capacitor(
                                    bi,
                                    record,
                                    *farads,
                                    &net_of_pin,
                                    &detached,
                                    &mut resolver,
                                );
                            }
                        }
                        if conducts && record.pins.len() == 2 {
                            let ohms = match kind {
                                PassiveKind::Resistor => value.unwrap_or(0.0),
                                _ => 0.0,
                            };
                            self.passive_edge(
                                bi,
                                record,
                                ohms,
                                &net_of_pin,
                                &detached,
                                &mut resolver,
                            );
                        }
                    }
                    PartClass::Jumper { state } => {
                        if *state == JumperState::Closed && record.pins.len() >= 2 {
                            self.passive_edge(
                                bi,
                                record,
                                0.0,
                                &net_of_pin,
                                &detached,
                                &mut resolver,
                            );
                        }
                    }
                    PartClass::Registered { pins, branches } => {
                        for pin in pins {
                            let key = (bi, PinRef::new(record.reference.clone(), pin.number));
                            if detached.contains(&key) {
                                continue;
                            }
                            let Some(&net) = net_of_pin.get(&key) else {
                                continue;
                            };
                            if let Some(endpoint) =
                                add_pin_descriptor(&mut resolver, net, pin, &key.1)
                            {
                                endpoints.insert(key, endpoint);
                            }
                        }
                        // The component's declared branches, by the pin
                        // names its facade gives them (number or alias).
                        let path = format!("{bname}.{}", record.reference);
                        let number_of = |id: &str| -> Option<&'static str> {
                            pins.iter()
                                .find(|p| p.number == id || p.name == Some(id))
                                .map(|p| p.number)
                        };
                        let net_of = |id: &str| -> Option<usize> {
                            let key = (bi, PinRef::new(record.reference.clone(), number_of(id)?));
                            (!detached.contains(&key))
                                .then(|| net_of_pin.get(&key).copied())
                                .flatten()
                        };
                        // A pin's declared clamps are diode branches of
                        // the part's own (`clamp_branches`).
                        let clamps: Vec<Branch> = pins.iter().flat_map(clamp_branches).collect();
                        for branch in branches.iter().chain(&clamps) {
                            if let Some((index, a, b)) = register_branch(
                                &mut resolver,
                                &mut paths,
                                &path,
                                branch.a,
                                branch.b,
                                branch.curve,
                                branch.control,
                                &net_of,
                            ) {
                                for (pin, sign) in [(a, 1.0), (b, -1.0)] {
                                    let number = number_of(&pin).expect("validated at build");
                                    let key = (bi, PinRef::new(record.reference.clone(), number));
                                    branch_terms.entry(key).or_default().push((index, sign));
                                }
                            }
                        }
                        // The lints' view of the part: its pins on their
                        // nets and its references by pin number (a detached
                        // pin is on no net and drops out).
                        lints.parts.push(PartLint {
                            path,
                            pins: pins
                                .iter()
                                .filter_map(|pin| {
                                    let key =
                                        (bi, PinRef::new(record.reference.clone(), pin.number));
                                    (!detached.contains(&key))
                                        .then(|| net_of_pin.get(&key).copied())
                                        .flatten()
                                        .map(|net| PinLint::of(pin, net))
                                })
                                .collect(),
                            references: pins
                                .iter()
                                .filter(|p| is_power(p))
                                .filter_map(|p| {
                                    Some((
                                        p.number.to_string(),
                                        number_of(p.reference?)?.to_string(),
                                    ))
                                })
                                .collect(),
                        });
                    }
                    // An element registered by spec: its branches stamp
                    // into the cluster solve like a component's, and each
                    // of its pins is a current port.
                    PartClass::Pwl { spec } => {
                        let path = format!("{bname}.{}", record.reference);
                        let net_of = |id: &str| -> Option<usize> {
                            let key = (bi, PinRef::new(record.reference.clone(), id));
                            (!detached.contains(&key))
                                .then(|| net_of_pin.get(&key).copied())
                                .flatten()
                        };
                        let mut terms: Vec<(String, usize, f64)> = Vec::new();
                        for branch in &spec.branches {
                            if let Some((index, a, b)) = register_branch(
                                &mut resolver,
                                &mut paths,
                                &path,
                                &branch.a,
                                &branch.b,
                                branch.curve,
                                branch
                                    .control
                                    .as_ref()
                                    .map(|(pin, test)| (pin.as_str(), *test)),
                                &net_of,
                            ) {
                                terms.push((a, index, 1.0));
                                terms.push((b, index, -1.0));
                            }
                        }
                        for pin in &spec.pins {
                            let branch_terms: Vec<(usize, f64)> = terms
                                .iter()
                                .filter(|(p, _, _)| p == pin)
                                .map(|(_, index, sign)| (*index, *sign))
                                .collect();
                            if !branch_terms.is_empty() {
                                paths.ports.push(CurrentPort {
                                    path: format!("{path}.{pin}"),
                                    endpoint: None,
                                    branch_terms,
                                });
                            }
                        }
                    }
                    // A boundary's pins are where a harness attaches; a
                    // switch's closed poles were unioned above and its open
                    // ones are nothing; a mechanical part has pads and no
                    // electrical existence; a probe senses through
                    // `BuiltSystem::probe`, which lands with the first board
                    // that carries a test point.
                    PartClass::Mechanical => {
                        for pin in &record.pins {
                            let key = (bi, PinRef::new(record.reference.clone(), pin.clone()));
                            if let Some(&net) = net_of_pin.get(&key) {
                                lints
                                    .mechanical
                                    .push((format!("{bname}.{}", record.reference), net));
                            }
                        }
                    }
                    PartClass::Boundary | PartClass::Switch { .. } | PartClass::Probe => {}
                }
            }
        }

        // Bench-component pins get the same electrical descriptors as
        // netlist-registered pins. There is no netlist facade to validate —
        // the declaration is the truth the nets were synthesized from.
        let mut bench_endpoints: Vec<Vec<Option<EndpointId>>> = Vec::new();
        // Per bench component, per declared pin: the branch terms.
        let mut bench_terms: Vec<Vec<Vec<(usize, f64)>>> = Vec::new();
        for (bench, pin_nets) in self.bench.iter().zip(&bench_pin_nets) {
            let mut eps = Vec::new();
            let pins = bench.component.pins();
            for (pin, &net) in pins.iter().zip(pin_nets) {
                let pin_ref = PinRef::new(bench.name.clone(), pin.number);
                let endpoint = add_pin_descriptor(&mut resolver, net, pin, &pin_ref);
                eps.push(endpoint);
            }
            bench_endpoints.push(eps);
            // The bench component's declared branches. A bench pin is never
            // detached (the fault algebra names board pins), so every
            // declared branch registers.
            let position_of = |id: &str| -> Option<usize> {
                pins.iter()
                    .position(|p| p.number == id || p.name == Some(id))
            };
            let net_of = |id: &str| -> Option<usize> { position_of(id).map(|i| pin_nets[i]) };
            lints.parts.push(PartLint {
                path: bench.name.clone(),
                pins: pins
                    .iter()
                    .zip(pin_nets)
                    .map(|(pin, &net)| PinLint::of(pin, net))
                    .collect(),
                references: pins
                    .iter()
                    .filter(|p| is_power(p))
                    .filter_map(|p| {
                        let named = p.reference?;
                        let reference = pins.iter().find(|q| q.answers_to(named))?;
                        Some((p.number.to_string(), reference.number.to_string()))
                    })
                    .collect(),
            });
            let mut terms: Vec<Vec<(usize, f64)>> = vec![Vec::new(); pins.len()];
            let clamps: Vec<Branch> = pins.iter().flat_map(clamp_branches).collect();
            for branch in bench.component.branches().iter().chain(&clamps) {
                if let Some((index, a, b)) = register_branch(
                    &mut resolver,
                    &mut paths,
                    &bench.name,
                    branch.a,
                    branch.b,
                    branch.curve,
                    branch.control,
                    &net_of,
                ) {
                    for (pin, sign) in [(a, 1.0), (b, -1.0)] {
                        let position = position_of(&pin).expect("validated at build");
                        terms[position].push((index, sign));
                    }
                }
            }
            bench_terms.push(terms);
        }

        // -- prepare registered components for attach ------------------------
        let boards = std::mem::take(&mut self.boards);
        let mut components: Vec<PreparedComponent> = Vec::new();
        for (bi, (bname, board)) in boards.into_iter().enumerate() {
            for (reference, component) in board.components {
                let record = board
                    .records
                    .iter()
                    .find(|r| r.reference == reference)
                    .expect("registered component has a record");
                if !record.fitted {
                    continue;
                }
                let mut prepared_pins: Vec<PreparedPin> = Vec::new();
                if let PartClass::Registered { pins, .. } = &record.class {
                    // The net a declared pin (a supply, a reference) is on,
                    // by the identity a declaration names it with; a
                    // detached pin is on none.
                    let net_named = |id: &str| -> Option<NetId> {
                        let number = pins.iter().find(|p| p.answers_to(id))?.number;
                        let key = (bi, PinRef::new(reference.clone(), number));
                        (!detached.contains(&key))
                            .then(|| net_of_pin.get(&key).copied().map(NetId))
                            .flatten()
                    };
                    for pin in pins {
                        let key = (bi, PinRef::new(reference.clone(), pin.number));
                        if let Some(&net) = net_of_pin.get(&key) {
                            let endpoint = endpoints.get(&key).copied();
                            let terms = branch_terms.get(&key).cloned().unwrap_or_default();
                            let terminal = pin.role == PinRole::PowerOut;
                            // A terminal's current spans clusters: its
                            // slot is no current port (`NODES.md` §2, the
                            // I-V port paragraph).
                            if (endpoint.is_some() && !terminal) || !terms.is_empty() {
                                paths.ports.push(CurrentPort {
                                    path: format!("{bname}.{reference}.{}", pin.number),
                                    endpoint: endpoint.filter(|_| !terminal),
                                    branch_terms: terms.clone(),
                                });
                            }
                            prepared_pins.push(PreparedPin {
                                number: pin.number.to_string(),
                                name: pin.name.map(str::to_string),
                                net,
                                endpoint,
                                reads_when_released: pin.reads_when_subscribed(),
                                declared: declared_thresholds(pin, net_named),
                                frame: sense_frame(pin, net_named),
                                branch_terms: terms,
                                terminal,
                                capability: DriveCapability::of(pin),
                            });
                        }
                    }
                }
                components.push(PreparedComponent {
                    board: bname.clone(),
                    reference,
                    component,
                    pins: prepared_pins,
                });
            }
        }

        // Bench components attach after board components, in add order.
        let bench = std::mem::take(&mut self.bench);
        for (((bench, pin_nets), endpoints), terms) in bench
            .into_iter()
            .zip(bench_pin_nets)
            .zip(bench_endpoints)
            .zip(bench_terms)
        {
            let pins = bench.component.pins();
            // A bench pin is never detached: every declared pin is on its
            // own global net.
            let net_named = |id: &str| -> Option<NetId> {
                let position = pins.iter().position(|p| p.answers_to(id))?;
                Some(NetId(pin_nets[position]))
            };
            let prepared_pins: Vec<PreparedPin> = pins
                .iter()
                .zip(&pin_nets)
                .zip(endpoints)
                .zip(terms)
                .map(|(((pin, &net), endpoint), branch_terms)| {
                    let terminal = pin.role == PinRole::PowerOut;
                    if (endpoint.is_some() && !terminal) || !branch_terms.is_empty() {
                        paths.ports.push(CurrentPort {
                            path: format!("{}.{}", bench.name, pin.number),
                            endpoint: endpoint.filter(|_| !terminal),
                            branch_terms: branch_terms.clone(),
                        });
                    }
                    PreparedPin {
                        number: pin.number.to_string(),
                        name: pin.name.map(str::to_string),
                        net,
                        endpoint,
                        reads_when_released: pin.reads_when_subscribed(),
                        declared: declared_thresholds(pin, net_named),
                        frame: sense_frame(pin, net_named),
                        branch_terms,
                        terminal,
                        capability: DriveCapability::of(pin),
                    }
                })
                .collect();
            components.push(PreparedComponent {
                board: bench.name.clone(),
                reference: bench.name,
                component: bench.component,
                pins: prepared_pins,
            });
        }

        Ok(Assembly {
            nets,
            resolver,
            components,
            paths,
            topology: Arc::new(BuildTopology { root_of, resistors }),
            lints,
        })
    }

    /// Resolve a harness endpoint to a global net index, creating synthetic
    /// external nets for bench-rig endpoints on boards the system does not
    /// contain (`P2EVAL.P0`).
    fn resolve_endpoint(
        &self,
        endpoint: &EndpointRef,
        board_index: &HashMap<String, usize>,
        net_of_pin: &HashMap<(usize, PinRef), usize>,
        nets: &mut Vec<Net>,
        external_nets: &mut HashMap<String, usize>,
    ) -> Result<usize, SystemError> {
        match (board_index.get(&endpoint.board), &endpoint.connector) {
            (Some(&bi), Some(connector)) => net_of_pin
                .get(&(bi, PinRef::new(connector.clone(), endpoint.pin.clone())))
                .copied()
                .ok_or_else(|| SystemError::UnknownEndpoint {
                    endpoint: format!("{}.{}.{}", endpoint.board, connector, endpoint.pin),
                }),
            (Some(_), None) => Err(SystemError::UnknownEndpoint {
                endpoint: format!("{}.{}", endpoint.board, endpoint.pin),
            }),
            (None, _) => {
                // Bench-rig external: synthesize one net per unique name.
                let name = match &endpoint.connector {
                    Some(c) => format!("{}.{}.{}", endpoint.board, c, endpoint.pin),
                    None => format!("{}.{}", endpoint.board, endpoint.pin),
                };
                let idx = *external_nets.entry(name.clone()).or_insert_with(|| {
                    let idx = nets.len();
                    nets.push(Net {
                        id: NetId(idx),
                        name,
                        nodes: Vec::new(),
                        state: NetState::Floating,
                        volts: crate::net::NetVolts::default(),
                    });
                    idx
                });
                Ok(idx)
            }
        }
    }

    /// Global net of a dotted `Board.Ref.Pin` endpoint.
    fn pin_net(
        &self,
        endpoint: &str,
        board_index: &HashMap<String, usize>,
        net_of_pin: &HashMap<(usize, PinRef), usize>,
    ) -> Result<usize, SystemError> {
        let (bi, pin) =
            split_board_pin(endpoint, board_index).ok_or_else(|| SystemError::UnknownEndpoint {
                endpoint: endpoint.to_string(),
            })?;
        net_of_pin
            .get(&(bi, pin))
            .copied()
            .ok_or_else(|| SystemError::UnknownEndpoint {
                endpoint: endpoint.to_string(),
            })
    }

    /// Global net index of a dotted `Board.NETNAME` reference.
    fn named_net(&self, path: &str, nets: &[Net]) -> Result<usize, SystemError> {
        let normalized = normalize_qualified_net_name(path);
        nets.iter()
            .position(|n| n.name == normalized)
            .ok_or_else(|| SystemError::UnknownEndpoint {
                endpoint: path.to_string(),
            })
    }

    /// Add a two-terminal passive/jumper conduction edge between the nets of
    /// a record's pins, respecting detached pins.
    /// Register a two-terminal capacitor as a coupling for rate routing,
    /// honouring a detached pad the way a passive edge does.
    fn coupling_capacitor(
        &self,
        bi: usize,
        record: &crate::board::PartRecord,
        farads: f64,
        net_of_pin: &HashMap<(usize, PinRef), usize>,
        detached: &HashSet<(usize, PinRef)>,
        resolver: &mut Resolver,
    ) {
        let a_key = (
            bi,
            PinRef::new(record.reference.clone(), record.pins[0].clone()),
        );
        let b_key = (
            bi,
            PinRef::new(record.reference.clone(), record.pins[1].clone()),
        );
        if detached.contains(&a_key) || detached.contains(&b_key) {
            return;
        }
        if let (Some(&a), Some(&b)) = (net_of_pin.get(&a_key), net_of_pin.get(&b_key)) {
            resolver.add_coupling(a, b, farads, record.reference.clone());
        }
    }

    fn passive_edge(
        &self,
        bi: usize,
        record: &crate::board::PartRecord,
        ohms: f64,
        net_of_pin: &HashMap<(usize, PinRef), usize>,
        detached: &HashSet<(usize, PinRef)>,
        resolver: &mut Resolver,
    ) {
        let a_key = (
            bi,
            PinRef::new(record.reference.clone(), record.pins[0].clone()),
        );
        let b_key = (
            bi,
            PinRef::new(record.reference.clone(), record.pins[1].clone()),
        );
        if detached.contains(&a_key) || detached.contains(&b_key) {
            return;
        }
        if let (Some(&a), Some(&b)) = (net_of_pin.get(&a_key), net_of_pin.get(&b_key)) {
            resolver.add_edge(a, b, ohms);
        }
    }
}

/// The build lints (`NODES.md` §8 phase 4), over the settled snapshot.
///
/// - [`Finding::RailDown`] for every `PowerOut` pin whose net reaches no
///   source — the terminal is released — unless the pin is another pin's
///   declared reference (an isolated ground is a reference terminal the
///   board or the harness holds, never a rail). The reason is what the
///   build can see: an input of the part (a power-in pin with a declared
///   reference) on an unsourced net first, the output's reference unheld
///   second, any other floating power-in pin — a ground — as an unheld
///   reference third, the part's own gate otherwise.
/// - [`Finding::UnreferencedDomain`] for a power pin whose net a source
///   reaches while its declared reference's net reaches none.
/// - [`Finding::UndecoupledPowerPin`] for a power-in pin with no fitted
///   capacitor between its node and its reference's node.
/// - [`Finding::MechanicalOnDrivenNet`] for a mechanical pad on a node a
///   pin drives.
/// - [`Finding::OpenDrainWithoutPullUp`] for an open-drain pin whose net
///   no pull-up reaches ([`open_drains_without_pull_up`]).
///
/// "Reaches no source" is the settled state `Floating` — the same fact
/// `FloatingSense` and `PowerNetUnsourced` report from.
fn lint_build(
    nets: &[Net],
    root_of: &[usize],
    resolver: &mut Resolver,
    lints: &LintInputs,
    diagnostics: &mut Diagnostics,
) {
    let unsourced = |net: usize| nets[net].state == NetState::Floating;
    let capacitor_pairs: HashSet<(usize, usize)> = lints
        .capacitors
        .iter()
        .map(|&(a, b)| {
            let (ra, rb) = (root_of[a], root_of[b]);
            (ra.min(rb), ra.max(rb))
        })
        .collect();
    for part in &lints.parts {
        let pin = |number: &str| part.pins.iter().find(|p| p.number == number);
        let reference_of = |number: &str| -> Option<&PinLint> {
            part.references
                .iter()
                .find(|(p, _)| p == number)
                .and_then(|(_, r)| pin(r))
        };
        let is_reference = |number: &str| part.references.iter().any(|(_, r)| r == number);
        for p in &part.pins {
            let reference = reference_of(&p.number);
            if let Some(r) = reference {
                if !unsourced(p.net) && unsourced(r.net) {
                    diagnostics.report(Finding::UnreferencedDomain {
                        part: part.path.clone(),
                        pin: p.number.clone(),
                        reference: r.number.clone(),
                    });
                }
                if p.role == PinRole::PowerIn {
                    let (ra, rb) = (root_of[p.net], root_of[r.net]);
                    if ra != rb && !capacitor_pairs.contains(&(ra.min(rb), ra.max(rb))) {
                        diagnostics.report(Finding::UndecoupledPowerPin {
                            part: part.path.clone(),
                            pin: p.number.clone(),
                            reference: r.number.clone(),
                        });
                    }
                }
            }
            if p.role == PinRole::PowerOut && !is_reference(&p.number) && unsourced(p.net) {
                // An input is a power-in pin measured against a declared
                // reference; a power-in pin with none is a ground, and a
                // floating ground is an unheld reference, not a missing
                // supply.
                let reason = if let Some(input) = part.pins.iter().find(|q| {
                    q.role == PinRole::PowerIn
                        && reference_of(&q.number).is_some()
                        && unsourced(q.net)
                }) {
                    RailDownReason::InputUnsourced {
                        pin: input.number.clone(),
                    }
                } else if let Some(r) = reference.filter(|r| unsourced(r.net)) {
                    RailDownReason::ReferenceUnheld {
                        pin: r.number.clone(),
                    }
                } else if let Some(ground) = part
                    .pins
                    .iter()
                    .find(|q| q.role == PinRole::PowerIn && unsourced(q.net))
                {
                    RailDownReason::ReferenceUnheld {
                        pin: ground.number.clone(),
                    }
                } else {
                    RailDownReason::HeldDown
                };
                diagnostics.report(Finding::RailDown {
                    part: part.path.clone(),
                    pin: p.number.clone(),
                    reason,
                });
            }
        }
    }
    for (part, net) in &lints.mechanical {
        let drivers = resolver.driving_pins(root_of, root_of[*net]);
        if !drivers.is_empty() {
            diagnostics.report(Finding::MechanicalOnDrivenNet {
                part: part.clone(),
                net: nets[*net].name.clone(),
                drivers,
            });
        }
    }
    open_drains_without_pull_up(nets, root_of, resolver, lints, diagnostics);
}

/// The pull-up lint (`NODES.md` §10, `can_source`): a structural question,
/// asked of the declarations and the resistive network — never of the
/// settled states, since a rail that has not risen by the build's snapshot
/// (a regulator's soft-start) is still the pull-up the board wires.
///
/// A **pull-up** is anything that can hold a net above ground: a
/// `PowerOut` pin that is not its part's declared reference (a rail, not a
/// ground), a harness supply or `net_stuck` above 0 V (or at an unmodelled
/// voltage), a signal pin that sources, an input port biased above 0 V. It
/// **reaches** an open drain when a resistive path — any resistance, a
/// declared terminal ending the path — joins its node to the open drain's:
/// rule 2's reach. A pull-up is by construction a source at or above
/// `WEAK_DRIVE_OHMS`, so the lint reads `NODES.md` §10's "no other source
/// reaches it through less than `WEAK_DRIVE_OHMS`" as rule 2's "no other
/// source reaches it" (`NODES.md` §12 item 5, the rules task, says why).
/// An open drain whose path reaches no other part's pin — a no-connect, or
/// a net that leaves the board only through a connector, whose pull-up is
/// the far side's — raises nothing.
fn open_drains_without_pull_up(
    nets: &[Net],
    root_of: &[usize],
    resolver: &mut Resolver,
    lints: &LintInputs,
    diagnostics: &mut Diagnostics,
) {
    let mut pull_ups: Vec<usize> = lints
        .supplies
        .iter()
        // Above 0 V, or unmodelled (NaN: a rail presented as up).
        .filter(|(_, volts)| volts.is_nan() || *volts > 0.0)
        .map(|&(net, _)| root_of[net])
        .collect();
    for part in &lints.parts {
        let is_reference = |number: &str| part.references.iter().any(|(_, r)| r == number);
        pull_ups.extend(
            part.pins
                .iter()
                .filter(|p| p.pulls_up || (p.role == PinRole::PowerOut && !is_reference(&p.number)))
                .map(|p| root_of[p.net]),
        );
    }
    pull_ups.sort_unstable();
    pull_ups.dedup();
    // How many part pins sit on each root. hash-order: keyed access only.
    let mut pins_on: HashMap<usize, usize> = HashMap::new();
    for pin in lints.parts.iter().flat_map(|part| &part.pins) {
        *pins_on.entry(root_of[pin.net]).or_default() += 1;
    }
    for part in &lints.parts {
        for pin in part.pins.iter().filter(|p| p.open_drain) {
            let reached = resolver.reached_roots(nets.len(), root_of[pin.net]);
            // The open drain reaches no other part's pin — a no-connect, or
            // a net that only leaves the board through a connector: nothing
            // on the board reads it, and its pull-up, if any, is the far
            // side's.
            let pins: usize = reached
                .iter()
                .map(|r| pins_on.get(r).copied().unwrap_or(0))
                .sum();
            if pins < 2 {
                continue;
            }
            let pulled_up = reached.iter().any(|r| pull_ups.binary_search(r).is_ok());
            if !pulled_up {
                diagnostics.report(Finding::OpenDrainWithoutPullUp {
                    part: part.path.clone(),
                    pin: pin.number.clone(),
                    net: nets[pin.net].name.clone(),
                });
            }
        }
    }
}

/// Split `"Board.Ref"` against known boards.
fn split_board_ref(path: &str, boards: &HashMap<String, usize>) -> Option<(usize, String)> {
    let (board, reference) = path.split_once('.')?;
    Some((*boards.get(board)?, reference.to_string()))
}

/// Split `"Board.Ref.Pin"` against known boards.
fn split_board_pin(path: &str, boards: &HashMap<String, usize>) -> Option<(usize, PinRef)> {
    let mut parts = path.splitn(3, '.');
    let board = parts.next()?;
    let reference = parts.next()?;
    let pin = parts.next()?;
    Some((*boards.get(board)?, PinRef::new(reference, pin)))
}

/// Register one component pin's electrical descriptor with the resolver,
/// returning the drive endpoint for pins that can drive — by its
/// [`PinRole`] and the declarations beside it (`NODES.md` §11):
///
/// - a [`PinRole::Signal`] pin has a drive slot idling at its declared
///   idle drive (released when it declares none), and its net joins the
///   senses when the pin is a sense from its declaration
///   ([`PinDecl::senses_at_build`]) — a digital sense when it declares
///   thresholds, an analog reader, which escalates its cluster, when it
///   declares none;
/// - a [`PinRole::PowerIn`] pin is sensed as a supply and has no slot;
/// - a [`PinRole::PowerOut`] pin's slot is a declared terminal's, holding
///   its idle drive until the part publishes;
/// - a [`PinRole::Passive`] pin is nothing.
fn add_pin_descriptor(
    resolver: &mut Resolver,
    net: usize,
    pin: &PinDecl,
    pin_ref: &PinRef,
) -> Option<EndpointId> {
    // A declared input port is stamped once, a permanent weak source at
    // the pin (`InputPort`); the build refused one on any other role.
    if let Some(port) = pin.input {
        resolver.add_port(
            net,
            pin_ref.clone(),
            TheveninDrive {
                volts: port.v_bias,
                impedance: port.r_in,
            },
        );
    }
    match pin.role {
        PinRole::Signal => {
            match pin.senses_at_build() {
                Some(SenseKind::Digital) => resolver.add_digital_sense(net),
                Some(SenseKind::Analog) => resolver.add_analog_sense(net),
                None => {}
            }
            // Every signal pin gets a slot, a sense's too: the re-entrancy
            // contract allows a sense callback to drive.
            Some(resolver.add_endpoint(net, pin_ref.clone(), pin.idle))
        }
        // Power and passive pins have no drive slot, so there is nothing
        // for a declared idle drive to set — `validate_pin_declarations`
        // refused such a declaration before any pin reached here.
        PinRole::PowerIn => {
            debug_assert!(
                pin.idle.is_none(),
                "{pin_ref:?}: idle drive on a PowerIn pin"
            );
            resolver.add_power_sense(net);
            None
        }
        PinRole::PowerOut => {
            // The pin's net is a declared terminal from build on — its own
            // cluster and a boundary of every cluster around it (`NODES.md`
            // "Three rules the taxonomy rests on", 1) — and the slot is how
            // the part sets what it holds. Until the part publishes it
            // holds its declared idle: released (a rail that is down), a
            // voltage, or the unmodelled rail `PinDecl::power_out` declares
            // by default ("sourced at an unmodelled voltage", NaN in the
            // source table). A declared idle keeps its impedance on the
            // slot — the I-V port's record, never solved — as a published
            // drive does.
            let idle = match pin.idle {
                None => TerminalDrive::Released.idle_slot_drive(),
                Some(drive) => Some(Drive::Thevenin(drive)),
            };
            Some(resolver.add_terminal_endpoint(net, pin_ref.clone(), idle))
        }
        PinRole::Passive => {
            debug_assert!(
                pin.idle.is_none(),
                "{pin_ref:?}: idle drive on a Passive pin"
            );
            None
        }
    }
}

/// A pin's declared thresholds as its handle carries them, with the net of
/// the supply pin it names (`net_named` resolves a declared identity to the
/// net its pin is on, `None` for a detached pin).
fn declared_thresholds(
    pin: &PinDecl,
    net_named: impl Fn(&str) -> Option<NetId>,
) -> Option<DeclaredThresholds> {
    Some(DeclaredThresholds {
        thresholds: pin.thresholds?,
        supply: pin.supply.and_then(&net_named),
        relative: pin.supply.is_some(),
    })
}

/// What a pin's sense is measured against: its declared reference pin's
/// net, the engine's frame when it declares none, and nothing when the
/// reference pin is detached (`net_named` as for [`declared_thresholds`]).
fn sense_frame(pin: &PinDecl, net_named: impl Fn(&str) -> Option<NetId>) -> SenseFrame {
    match pin.reference {
        None => SenseFrame::Absolute,
        Some(reference) => net_named(reference).map_or(SenseFrame::Detached, SenseFrame::Against),
    }
}

/// Build the (identity → handle) entries for one prepared component.
fn handle_entries(pins: &[PreparedPin], link: &EngineLink) -> Vec<(String, PinHandle)> {
    let mut entries = Vec::new();
    for pin in pins {
        let handle = PinHandle::wired(NetId(pin.net), pin.endpoint, link.clone())
            .reading_when_released(pin.reads_when_released)
            .with_thresholds(pin.declared)
            .measured_in(pin.frame)
            .with_branch_terms(pin.branch_terms.clone())
            .on_terminal(pin.terminal)
            .declaring(pin.capability);
        entries.push((pin.number.clone(), handle.clone()));
        if let Some(name) = &pin.name {
            entries.push((name.clone(), handle));
        }
    }
    entries
}

// ============================================================
// Built system + live system handle
// ============================================================

/// A built system: system-wide resolved nets plus the diagnostics collected
/// by the build-time resolution pass.
#[derive(Debug)]
pub struct BuiltSystem {
    nets: Vec<Net>,
    diagnostics: Diagnostics,
    /// Identity root per net index (harness/pin-short merges).
    roots: Vec<usize>,
    /// The identity roots of every conduction cluster, in ascending
    /// cluster-root order.
    cluster_roots: Vec<Vec<NetId>>,
    /// Cluster solves the build-time passes escalated to the solver.
    escalated_solves: u64,
    /// The currents the build's last pass produced.
    currents: CurrentTable,
    /// The elements and current ports, by path.
    paths: CurrentPaths,
}

impl BuiltSystem {
    /// System-wide resolved nets (post harness merge), indexed by
    /// [`crate::net::NetId`].
    pub fn nets(&self) -> &[Net] {
        &self.nets
    }

    /// Whether two nets are the same electrical node — i.e. a harness wire
    /// or `pin_short` merged them.
    ///
    /// A merge does **not** rename nets: each keeps its own board's label
    /// (`EdgeBoard./Sheet2/IEND_U+` stays distinct from `END_U.COM` in
    /// [`BuiltSystem::nets`]), so comparing names — or comparing resolved
    /// [`NetState`]s, which are equal for any two floating nets whether or
    /// not they touch — cannot answer "is this wired?". This can.
    pub fn nets_are_merged(&self, a: NetId, b: NetId) -> bool {
        match (self.roots.get(a.0), self.roots.get(b.0)) {
            (Some(ra), Some(rb)) => ra == rb,
            _ => false,
        }
    }

    /// Index of a net by qualified name (`"Board.NETNAME"`, overline- and
    /// sheet-normalized like the assembly stored it).
    pub fn net_id(&self, name: &str) -> Option<NetId> {
        let normalized = normalize_qualified_net_name(name);
        self.nets
            .iter()
            .position(|n| n.name == normalized)
            .map(NetId)
    }

    /// Whether two nets named as the netlist spells them are the same
    /// electrical node. Returns `false` if either name is unknown — callers
    /// asserting connectivity should check [`BuiltSystem::net_id`] first if
    /// they want a typo to fail loudly instead of reading as "not wired".
    pub fn names_are_merged(&self, a: &str, b: &str) -> bool {
        match (self.net_id(a), self.net_id(b)) {
            (Some(a), Some(b)) => self.nets_are_merged(a, b),
            _ => false,
        }
    }

    /// Findings from the build-time resolution pass (and, later, the live
    /// engine).
    pub fn diagnostics(&self) -> &Diagnostics {
        &self.diagnostics
    }

    /// The census of the system's conduction clusters: one entry per
    /// cluster, the identity roots it holds, in ascending cluster-root
    /// order (roots ascending within). A conduction cluster is what a
    /// resistor joins, ending at a declared terminal, and what nothing else
    /// crosses; a root is one electrical node after harness, `pin_short`,
    /// closed-pole and inductor merges, named by any of the nets merged into
    /// it (see
    /// [`BuiltSystem::nets`]). So an entry's length is the size `m` of the
    /// matrix an escalated solve of that cluster builds, and the longest
    /// entry bounds every solve on the board — the number `DESIGN.md` rule 4
    /// holds to `m ≤ 8`.
    pub fn cluster_roots(&self) -> &[Vec<NetId>] {
        &self.cluster_roots
    }

    /// How many cluster solves the build-time resolution passes escalated to
    /// the solver (see [`SystemHandle::escalated_solves`] for the rule).
    pub fn escalated_solves(&self) -> u64 {
        self.escalated_solves
    }

    /// The current through the one branch of the piecewise-linear element
    /// at `"Board.Reference"` — a diode, an LED — positive from its anode to
    /// its cathode, from the build's last pass. `None` for a part with no
    /// branch or more than one, or one whose cluster no solve reached (a
    /// cluster with an element always solves, so that is a part nothing
    /// sources). An LED is lit when this is at or above its threshold
    /// (`NODES.md` §2).
    pub fn branch_current(&self, part: &str) -> Option<Amps> {
        self.paths.branch_current(&self.currents, part)
    }

    /// The current **into** the pin at `"Board.Reference.Pin"` from its net
    /// — the sum [`crate::PinHandle::sense_current`] makes — from the
    /// build's last pass. `None` where no solve produced one: the pin has
    /// no drive slot and terminates no branch, or its cluster resolved by
    /// projection alone (a current instrument on it,
    /// [`crate::ComponentNetIo::on_branch`], is what escalates it).
    pub fn pin_current(&self, pin: &str) -> Option<Amps> {
        self.paths.pin_current(&self.currents, pin)
    }
}

/// A running live system: the net-engine thread plus the attached
/// components, created by [`System::start`].
///
/// Dropping the handle shuts the engine down first (shutdown message +
/// join — in-flight sense/wake callbacks complete, and joining cannot
/// deadlock because callbacks run with no engine lock held), then drops the
/// components.
pub struct SystemHandle {
    // Field order is load-bearing: the engine joins before components drop.
    engine: EngineHandle,
    net_names: Vec<String>,
    components: Vec<(String, Box<dyn Component>)>,
    /// The elements and current ports, by path.
    paths: CurrentPaths,
}

impl fmt::Debug for SystemHandle {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("SystemHandle")
            .field("engine", &self.engine)
            .field(
                "components",
                &self.components.iter().map(|(r, _)| r).collect::<Vec<_>>(),
            )
            .finish()
    }
}

impl SystemHandle {
    /// Most recently engine-published state of a net, by qualified system
    /// name (`"Board.NETNAME"`, overline-normalized).
    pub fn net_state(&self, name: &str) -> Option<NetState> {
        let normalized = normalize_qualified_net_name(name);
        let index = self.net_names.iter().position(|n| *n == normalized)?;
        self.engine.net_state(NetId(index))
    }

    /// Most recently engine-published state of a net, by id.
    pub fn net_state_of(&self, net: NetId) -> Option<NetState> {
        self.engine.net_state(net)
    }

    /// The current through the one branch of the element at
    /// `"Board.Reference"`, from the engine's last solve of its cluster
    /// (see [`BuiltSystem::branch_current`]).
    pub fn branch_current(&self, part: &str) -> Option<Amps> {
        self.paths.branch_current(&self.engine.currents(), part)
    }

    /// The current into the pin at `"Board.Reference.Pin"` (or `"Bench.Pin"`
    /// for a bench component), from the engine's last solve of its cluster
    /// (see [`BuiltSystem::pin_current`]).
    pub fn pin_current(&self, pin: &str) -> Option<Amps> {
        self.paths.pin_current(&self.engine.currents(), pin)
    }

    /// Snapshot of the cumulative live findings (the initial resolution pass
    /// runs before [`System::start`] returns, so build-time findings are
    /// already present).
    pub fn findings(&self) -> Vec<Finding> {
        self.engine.findings()
    }

    /// True while the net-engine thread is alive and serving commands (see
    /// [`crate::engine::EngineHandle::is_alive`]). Component callbacks are
    /// panic-contained, so `false` means the engine itself failed.
    pub fn engine_is_alive(&self) -> bool {
        self.engine.is_alive()
    }

    /// How many cluster solves the engine has escalated to the solver so
    /// far, the initial resolution pass included. A solve runs only where
    /// sources within a factor of ten disagree or an analog sense asks;
    /// everything else is a projection (`DESIGN.md` rule 8). A run on a
    /// board with neither reads 0, and a test holds it there as the budget
    /// it is (see [`crate::engine::EngineHandle::escalated_solves`]).
    pub fn escalated_solves(&self) -> u64 {
        self.engine.escalated_solves()
    }

    /// Handle to this system's engine event log (determinism Oracle 1 — see
    /// [`System::event_log`] and [`crate::event_log`]). Reads empty unless the
    /// log was enabled on the builder.
    pub fn event_log(&self) -> EventLog {
        self.engine.event_log()
    }

    /// Reference designators of the attached components, in attach order.
    pub fn component_refs(&self) -> impl Iterator<Item = &str> {
        self.components
            .iter()
            .map(|(reference, _)| reference.as_str())
    }

    /// Release the engine's hold on virtual time, for a system started with
    /// [`System::hold_time`]. Idempotent; a system started without the
    /// hold has already released it. Paced or unpaced alike — see
    /// [`System::hold_time`].
    pub fn release_time(&self) {
        self.engine.release_time();
    }

    /// Shut the live system down explicitly (equivalent to dropping it).
    pub fn shutdown(self) {}
}

// ============================================================
// Errors
// ============================================================

/// Harness construction failure.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum HarnessError {
    /// An endpoint string is not `Board.Connector.Pin` / `Board.Pin`.
    BadEndpoint {
        /// The offending string.
        endpoint: String,
    },
}

impl fmt::Display for HarnessError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            HarnessError::BadEndpoint { endpoint } => {
                write!(
                    f,
                    "bad harness endpoint {endpoint:?} (expected Board.Connector.Pin or Board.Pin)"
                )
            }
        }
    }
}

impl std::error::Error for HarnessError {}

/// System assembly failure.
#[derive(Debug)]
pub enum SystemError {
    /// Two boards were added under the same name.
    DuplicateBoard {
        /// The colliding name.
        name: String,
    },
    /// A bench component's name (or one of its pin identities) collides
    /// with a board, another bench component, or another pin.
    DuplicateComponent {
        /// The colliding name (`"P2EVAL"`) or dotted pin (`"P2EVAL.P0"`).
        name: String,
    },
    /// A harness or scenario referenced a board/connector/pin that does not
    /// exist in the system.
    UnknownEndpoint {
        /// The dotted reference that failed to resolve.
        endpoint: String,
    },
    /// A scenario set a switch pole the part does not have (a jumper has one
    /// pole, index 0; a part that is neither a switch nor a jumper has none).
    UnknownSwitchPole {
        /// The dotted part reference (`"EC32MB.S301"`).
        reference: String,
        /// The pole index asked for.
        pole: usize,
        /// How many poles the part has.
        poles: usize,
    },
    /// A harness failed to validate.
    Harness(HarnessError),
    /// A board-level structural failure surfaced during assembly.
    Board {
        /// System board name.
        name: String,
        /// The underlying board error.
        error: BoardError,
    },
}

impl fmt::Display for SystemError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SystemError::DuplicateBoard { name } => write!(f, "duplicate board name {name:?}"),
            SystemError::DuplicateComponent { name } => {
                write!(f, "duplicate bench component name {name:?}")
            }
            SystemError::UnknownEndpoint { endpoint } => {
                write!(f, "unknown endpoint {endpoint:?}")
            }
            SystemError::UnknownSwitchPole {
                reference,
                pole,
                poles,
            } => write!(f, "{reference} has no switch pole {pole} (it has {poles})"),
            SystemError::Harness(e) => write!(f, "harness: {e}"),
            SystemError::Board { name, error } => write!(f, "board {name:?}: {error}"),
        }
    }
}

impl std::error::Error for SystemError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            SystemError::Harness(e) => Some(e),
            SystemError::Board { error, .. } => Some(error),
            _ => None,
        }
    }
}

impl From<HarnessError> for SystemError {
    fn from(e: HarnessError) -> Self {
        SystemError::Harness(e)
    }
}

#[cfg(test)]
mod tests {
    use rstest::rstest;

    use super::*;

    #[rstest]
    fn endpoint_parses_connector_and_bare_forms() {
        assert_eq!(
            EndpointRef::parse("DS2Addon.J1.3").unwrap(),
            EndpointRef {
                board: "DS2Addon".to_string(),
                connector: Some("J1".to_string()),
                pin: "3".to_string(),
            }
        );
        assert_eq!(
            EndpointRef::parse("P2EVAL.P0").unwrap(),
            EndpointRef {
                board: "P2EVAL".to_string(),
                connector: None,
                pin: "P0".to_string()
            }
        );
        assert!(EndpointRef::parse("JustOneSegment").is_err());
        assert!(EndpointRef::parse("A.B.C.D").is_err());
        assert!(EndpointRef::parse("A..C").is_err());
    }

    #[rstest]
    fn harness_builder_accumulates_signal_and_power_wires() {
        let harness = Harness::new()
            .connect_str("EdgeBoard.J3.1", "DS2Addon.J1.2")
            .unwrap()
            .power(
                EndpointRef::parse("P2EVAL.3V3").unwrap(),
                EndpointRef::parse("DS2Addon.J1.1").unwrap(),
                3.3,
            );
        assert_eq!(harness.connections().len(), 2);
        assert_eq!(harness.connections()[0].kind, EndpointKind::Signal);
        assert_eq!(
            harness.connections()[1].kind,
            EndpointKind::Power { volts: 3.3 }
        );
    }

    #[rstest]
    fn scenario_builder_accumulates_fault_algebra() {
        let scenario = Scenario::default()
            .jumper("DS2Addon.JP1", JumperState::Closed)
            .pin_detach("DS2Addon.U1.3")
            .pin_short("DS2Addon.A0", "DS2Addon.A1")
            .net_stuck("DS2Addon.AIN0", 3.3)
            .value_override("DS2Addon.R5", "4k7")
            .dnp_override("DS2Addon.C7", DnpState::Populated);

        assert_eq!(
            scenario.jumpers(),
            &[("DS2Addon.JP1".to_string(), JumperState::Closed)]
        );
        assert_eq!(scenario.faults().len(), 3);
        assert_eq!(
            scenario.faults()[0],
            Fault::PinDetach {
                endpoint: "DS2Addon.U1.3".to_string()
            }
        );
        assert_eq!(scenario.value_overrides().len(), 1);
        assert_eq!(scenario.dnp_overrides().len(), 1);
    }

    /// A dotted reference must canonicalize exactly the way the assembly
    /// stored the net: board prefix verbatim, net part through the net
    /// normalizer. Naming a net the way the netlist spells it — with the
    /// overline braces, or sheet-scoped — has to resolve, or `net_stuck`
    /// silently misses a net that exists.
    #[rstest]
    #[case("DS2Addon.~{RESET}", "DS2Addon.~RESET")]
    #[case("DS2Addon.~RESET", "DS2Addon.~RESET")]
    #[case(
        "EdgeBoard./MaD_Edge_Sheet2/~{IFG_RX}",
        "EdgeBoard./MaD_Edge_Sheet2/~IFG_RX"
    )]
    #[case(
        "EdgeBoard./MaD_Edge_Sheet2/IFG_RX",
        "EdgeBoard./MaD_Edge_Sheet2/IFG_RX"
    )]
    // A net label containing a dot keeps everything after the FIRST dot.
    #[case("Board.A.B", "Board.A.B")]
    #[case("GND", "GND")]
    fn qualified_net_names_canonicalize_like_the_assembly(
        #[case] input: &str,
        #[case] expected: &str,
    ) {
        assert_eq!(normalize_qualified_net_name(input), expected);
    }
}
