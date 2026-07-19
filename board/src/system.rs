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

use crate::board::{Board, BoardError, PartClass};
use crate::cluster::QuasiStaticMna;
use crate::component::{Component, ComponentNetIo, PinDecl, PinHandle, PinKind, StreamRole};
use crate::diagnostics::{Diagnostics, Finding};
use crate::engine::{
    ComponentId, Dsu, EndpointId, EngineHandle, EngineLink, Resolver, DEFAULT_HIGH_LEVEL_VOLTS,
};
use crate::net::{Net, NetId, NetState, PinRef, TheveninDrive, Volts, DEFAULT_PUSH_PULL_IMPEDANCE};
use crate::registry::{parse_passive_value, JumperState, PassiveKind};

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
/// `StreamMismatch`/`Contention`/`Floating` findings are the assertion
/// targets.
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

/// Byte-loss injection policy for a serial route (the supported way to
/// exercise loss-handling code — byte loss is not emergent at pipe
/// granularity).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum StreamDropPolicy {
    /// Drop every byte (broken wire while the route stays valid).
    All,
    /// Drop every Nth byte.
    EveryNth(u32),
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
    /// Byte-loss injection on a serial route.
    StreamDrop {
        /// Dotted stream-endpoint pin.
        endpoint: String,
        /// Drop policy.
        policy: StreamDropPolicy,
    },
}

/// Scenario overrides: jumper states, DNP/value BOM changes, injected faults.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Scenario {
    jumpers: Vec<(String, JumperState)>,
    value_overrides: Vec<(String, String)>,
    dnp_overrides: Vec<(String, DnpState)>,
    faults: Vec<Fault>,
}

impl Scenario {
    /// Set a jumper's state (`"DS2Addon.JP1"`).
    pub fn jumper(mut self, reference: &str, state: JumperState) -> Self {
        self.jumpers.push((reference.to_string(), state));
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

    /// Inject byte loss on a serial route.
    pub fn stream_drop(mut self, endpoint: &str, policy: StreamDropPolicy) -> Self {
        self.faults.push(Fault::StreamDrop {
            endpoint: endpoint.to_string(),
            policy,
        });
        self
    }

    /// Jumper overrides, in declaration order.
    pub fn jumpers(&self) -> &[(String, JumperState)] {
        &self.jumpers
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

/// One prepared pin: every identity it answers to, its global net, its
/// drive endpoint (when the pin can drive and is not detached), and its
/// declared serial-stream role (for the stream I/O surface).
struct PreparedPin {
    number: String,
    name: Option<String>,
    net: usize,
    endpoint: Option<EndpointId>,
    stream: Option<StreamRole>,
}

/// Output of the shared assembly pass: the merged net table, the populated
/// resolver (one code path for build-time analysis and live resolution),
/// and the components ready to attach.
struct Assembly {
    nets: Vec<Net>,
    resolver: Resolver,
    components: Vec<PreparedComponent>,
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
    /// descriptors as netlist-registered pins — drives, senses, and serial
    /// stream roles all participate in resolution and routing. Endpoints
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
        } = self.assemble()?;

        let mut diagnostics = Diagnostics::new();
        resolver.resolve(&mut nets, &mut diagnostics, &QuasiStaticMna);
        // Stream routing runs at build too: byte pipes are derived from and
        // gated by net resolution, and the routing findings
        // (`StreamMismatch`) are build-time analysis output. The derived
        // routes themselves only come alive on the `System::start` path.
        let _ = resolver.route_streams(&nets, &mut diagnostics);

        // Inert attach: sense() reads this build-resolved snapshot; drives
        // and schedules are traced and dropped.
        let states: Arc<Mutex<Vec<NetState>>> =
            Arc::new(Mutex::new(nets.iter().map(|n| n.state).collect()));
        let link = EngineLink::inert(states);
        for mut prepared in components {
            let io =
                ComponentNetIo::wired(handle_entries(&prepared.pins, &link), None, link.clone());
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
            // Build-time analysis slice: the component validated its facade
            // and is dropped here; System::start keeps it.
        }

        Ok(BuiltSystem { nets, diagnostics })
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
        let Assembly {
            nets,
            resolver,
            components,
        } = self.assemble()?;

        let net_names: Vec<String> = nets.iter().map(|n| n.name.clone()).collect();
        let engine = EngineHandle::spawn(resolver, nets, Box::new(QuasiStaticMna));
        let link = engine.link();

        let mut attached: Vec<(String, Box<dyn Component>)> = Vec::new();
        for (index, mut prepared) in components.into_iter().enumerate() {
            let io = ComponentNetIo::wired(
                handle_entries(&prepared.pins, &link),
                Some(ComponentId(index)),
                link.clone(),
            );
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

        Ok(SystemHandle {
            engine,
            net_names,
            components: attached,
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
            let jumpers = self.scenario.jumpers().to_vec();
            let dnp_overrides = self.scenario.dnp_overrides().to_vec();
            let value_overrides = self.scenario.value_overrides().to_vec();

            for (path, state) in &jumpers {
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
                if let PartClass::Jumper { state: s } = &mut record.class {
                    *s = *state;
                }
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
        // Stream drops resolve to endpoints only after the electrical
        // descriptors are registered below; collect them here.
        let mut stream_drops: Vec<(String, StreamDropPolicy)> = Vec::new();
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
                Fault::StreamDrop { endpoint, policy } => {
                    stream_drops.push((endpoint, policy));
                }
            }
        }

        // -- electrical descriptors ----------------------------------------
        let mut resolver = Resolver::new(nets.len(), dsu);
        for (idx, volts) in power_sources {
            resolver.add_power_source(idx, volts);
        }
        for (idx, volts) in stuck_sources {
            resolver.add_stuck_source(idx, volts);
        }

        let mut endpoints: HashMap<(usize, PinRef), EndpointId> = HashMap::new();
        for (bi, (_bname, board)) in self.boards.iter().enumerate() {
            for record in &board.records {
                if !record.fitted {
                    continue;
                }
                match &record.class {
                    PartClass::Passive { kind, value } => {
                        let conducts = match kind {
                            PassiveKind::Resistor => value.is_some(),
                            // DC short (documented simplification).
                            PassiveKind::Inductor => true,
                            // DC open in the build-time pass.
                            PassiveKind::Capacitor | PassiveKind::Diode | PassiveKind::Led => false,
                        };
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
                    PartClass::Registered { pins } => {
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
                                // Serial-capable pins register for stream
                                // routing (their pipes derive from the nets
                                // they drive/sense — never installed beside
                                // them).
                                if let Some(role) = pin.stream {
                                    resolver.add_stream_pin(endpoint, net, role, key.1.clone());
                                }
                                endpoints.insert(key, endpoint);
                            } else if pin.stream.is_some() {
                                tracing::warn!(
                                    reference = %record.reference,
                                    pin = pin.number,
                                    "stream role on a pin without a drive endpoint is ignored"
                                );
                            }
                        }
                    }
                    PartClass::Boundary | PartClass::Stubbed => {}
                }
            }
        }

        // Bench-component pins get the same electrical descriptors as
        // netlist-registered pins. There is no netlist facade to validate —
        // the declaration is the truth the nets were synthesized from.
        let mut bench_endpoints: Vec<Vec<Option<EndpointId>>> = Vec::new();
        for (bench, pin_nets) in self.bench.iter().zip(&bench_pin_nets) {
            let mut eps = Vec::new();
            for (pin, &net) in bench.component.pins().iter().zip(pin_nets) {
                let pin_ref = PinRef::new(bench.name.clone(), pin.number);
                let endpoint = add_pin_descriptor(&mut resolver, net, pin, &pin_ref);
                if let Some(ep) = endpoint {
                    if let Some(role) = pin.stream {
                        resolver.add_stream_pin(ep, net, role, pin_ref);
                    }
                } else if pin.stream.is_some() {
                    tracing::warn!(
                        component = %bench.name,
                        pin = pin.number,
                        "stream role on a pin without a drive endpoint is ignored"
                    );
                }
                eps.push(endpoint);
            }
            bench_endpoints.push(eps);
        }

        // -- stream-drop faults (need the endpoint table) --------------------
        for (path, policy) in stream_drops {
            let (bi, pin) = split_board_pin(&path, &board_index).ok_or_else(|| {
                SystemError::UnknownEndpoint {
                    endpoint: path.clone(),
                }
            })?;
            let endpoint =
                endpoints
                    .get(&(bi, pin))
                    .copied()
                    .ok_or_else(|| SystemError::UnknownEndpoint {
                        endpoint: path.clone(),
                    })?;
            resolver.add_stream_drop(endpoint, policy);
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
                if let PartClass::Registered { pins } = &record.class {
                    for pin in pins {
                        let key = (bi, PinRef::new(reference.clone(), pin.number));
                        if let Some(&net) = net_of_pin.get(&key) {
                            prepared_pins.push(PreparedPin {
                                number: pin.number.to_string(),
                                name: pin.name.map(str::to_string),
                                net,
                                endpoint: endpoints.get(&key).copied(),
                                stream: pin.stream,
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
        for ((bench, pin_nets), endpoints) in
            bench.into_iter().zip(bench_pin_nets).zip(bench_endpoints)
        {
            let prepared_pins: Vec<PreparedPin> = bench
                .component
                .pins()
                .iter()
                .zip(&pin_nets)
                .zip(endpoints)
                .map(|((pin, &net), endpoint)| PreparedPin {
                    number: pin.number.to_string(),
                    name: pin.name.map(str::to_string),
                    net,
                    endpoint,
                    stream: pin.stream,
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
        let normalized = crate::netlist::normalize_pin_name(path);
        nets.iter()
            .position(|n| n.name == normalized)
            .ok_or_else(|| SystemError::UnknownEndpoint {
                endpoint: path.to_string(),
            })
    }

    /// Add a two-terminal passive/jumper conduction edge between the nets of
    /// a record's pins, respecting detached pins.
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
/// returning the drive endpoint for pins that can drive.
fn add_pin_descriptor(
    resolver: &mut Resolver,
    net: usize,
    pin: &PinDecl,
    pin_ref: &PinRef,
) -> Option<EndpointId> {
    match pin.kind {
        PinKind::DigitalIn => {
            resolver.add_digital_sense(net);
            // Sense pins still get a released drive slot: the re-entrancy
            // contract allows a sense callback to drive.
            Some(resolver.add_endpoint(net, pin_ref.clone(), None))
        }
        PinKind::Analog => {
            resolver.add_analog_sense(net);
            Some(resolver.add_endpoint(net, pin_ref.clone(), None))
        }
        PinKind::PowerIn => {
            resolver.add_power_sense(net);
            None
        }
        PinKind::PowerOut => {
            // Component-declared rail voltage arrives with the regulator
            // models (a later slice); presence is what the build-time pass
            // needs. NaN marks "sourced at an unmodeled voltage".
            resolver.add_power_source(net, f64::NAN);
            None
        }
        PinKind::DigitalOut | PinKind::DigitalBidir => {
            // Idle default: stream producers idle Driven(High) per the
            // stream spec; plain outputs idle High as the documented default
            // until the component drives otherwise.
            let impedance = pin.drive_impedance.unwrap_or(DEFAULT_PUSH_PULL_IMPEDANCE);
            Some(resolver.add_endpoint(
                net,
                pin_ref.clone(),
                Some(TheveninDrive {
                    volts: DEFAULT_HIGH_LEVEL_VOLTS,
                    impedance,
                }),
            ))
        }
        PinKind::Passive => None,
    }
}

/// Build the (identity → handle) entries for one prepared component.
fn handle_entries(pins: &[PreparedPin], link: &EngineLink) -> Vec<(String, PinHandle)> {
    let mut entries = Vec::new();
    for pin in pins {
        let handle = PinHandle::wired(NetId(pin.net), pin.endpoint, pin.stream, link.clone());
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
}

impl BuiltSystem {
    /// System-wide resolved nets (post harness merge), indexed by
    /// [`crate::net::NetId`].
    pub fn nets(&self) -> &[Net] {
        &self.nets
    }

    /// Findings from the build-time resolution pass (and, later, the live
    /// engine).
    pub fn diagnostics(&self) -> &Diagnostics {
        &self.diagnostics
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
        let normalized = crate::netlist::normalize_pin_name(name);
        let index = self.net_names.iter().position(|n| *n == normalized)?;
        self.engine.net_state(NetId(index))
    }

    /// Most recently engine-published state of a net, by id.
    pub fn net_state_of(&self, net: NetId) -> Option<NetState> {
        self.engine.net_state(net)
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

    /// Reference designators of the attached components, in attach order.
    pub fn component_refs(&self) -> impl Iterator<Item = &str> {
        self.components
            .iter()
            .map(|(reference, _)| reference.as_str())
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
            .dnp_override("DS2Addon.C7", DnpState::Populated)
            .stream_drop("DS2Addon.U1.2", StreamDropPolicy::EveryNth(3));

        assert_eq!(
            scenario.jumpers(),
            &[("DS2Addon.JP1".to_string(), JumperState::Closed)]
        );
        assert_eq!(scenario.faults().len(), 4);
        assert_eq!(
            scenario.faults()[0],
            Fault::PinDetach {
                endpoint: "DS2Addon.U1.3".to_string()
            }
        );
        assert_eq!(scenario.value_overrides().len(), 1);
        assert_eq!(scenario.dnp_overrides().len(), 1);
    }
}
