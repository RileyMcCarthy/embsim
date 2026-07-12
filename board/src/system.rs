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
//! Slice status: the builder surface, harness/scenario types, and the fault
//! algebra are final; `System::build` (the full build-time resolution pass
//! producing [`crate::diagnostics::Finding`]s) is the system slice.

use std::collections::{HashMap, HashSet};
use std::fmt;

use crate::board::{Board, BoardError, PartClass};
use crate::component::{ComponentNetIo, PinDecl, PinHandle, PinKind};
use crate::diagnostics::{Diagnostics, Finding, SenseKind};
use crate::net::{Level, Net, NetId, NetState, PinRef, Volts, DEFAULT_PUSH_PULL_IMPEDANCE};
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

/// System builder: named boards + harnesses + scenario.
#[derive(Debug, Default)]
pub struct System {
    boards: Vec<(String, Board)>,
    harnesses: Vec<Harness>,
    scenario: Scenario,
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
    /// sensing components immediately, before any traffic.
    pub fn build(mut self) -> Result<BuiltSystem, SystemError> {
        // -- duplicate-name gate ------------------------------------------
        let mut seen = HashSet::new();
        for (name, _) in &self.boards {
            if !seen.insert(name.clone()) {
                return Err(SystemError::DuplicateBoard { name: name.clone() });
            }
        }

        // -- global net table ---------------------------------------------
        // Global net = (board index, board-local NetId), flattened densely.
        // No implicit name merging: only harness wires and pin_short faults
        // union nets across (or within) boards.
        let mut nets: Vec<Net> = Vec::new();
        let mut base_of_board: Vec<usize> = Vec::new();
        for (bi, (bname, board)) in self.boards.iter().enumerate() {
            base_of_board.push(nets.len());
            for net in &board.nets {
                let mut qualified = net.clone();
                qualified.id = NetId(nets.len());
                qualified.name = format!("{bname}.{name}", name = net.name);
                nets.push(qualified);
            }
            let _ = bi;
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

        // Bench-rig externals ("P2EVAL.P0") get synthetic nets on demand.
        let mut external_nets: HashMap<String, usize> = HashMap::new();

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
                Fault::StreamDrop { .. } => {
                    // Stream routing is a later slice; the fault is recorded
                    // in the scenario but has no build-time effect yet.
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
                            add_pin_descriptor(&mut resolver, net, pin, &key.1);
                        }
                    }
                    PartClass::Boundary | PartClass::Stubbed => {}
                }
            }
        }

        // -- resolution pass -----------------------------------------------
        let mut diagnostics = Diagnostics::new();
        resolver.resolve(&mut nets, &mut diagnostics);

        // -- attach registered components (pre-share, final net ids) --------
        let boards = std::mem::take(&mut self.boards);
        for (bi, (bname, board)) in boards.into_iter().enumerate() {
            for (reference, mut component) in board.components {
                let record = board
                    .records
                    .iter()
                    .find(|r| r.reference == reference)
                    .expect("registered component has a record");
                if !record.fitted {
                    continue;
                }
                let mut entries: Vec<(String, PinHandle)> = Vec::new();
                if let PartClass::Registered { pins } = &record.class {
                    for pin in pins {
                        let key = (bi, PinRef::new(reference.clone(), pin.number));
                        if let Some(&net) = net_of_pin.get(&key) {
                            let handle = PinHandle::new(NetId(net));
                            entries.push((pin.number.to_string(), handle));
                            if let Some(name) = pin.name {
                                entries.push((name.to_string(), handle));
                            }
                        }
                    }
                }
                component
                    .attach(ComponentNetIo::from_entries(entries))
                    .map_err(|error| SystemError::Board {
                        name: bname.clone(),
                        error: BoardError::Attach {
                            reference: reference.clone(),
                            error,
                        },
                    })?;
                // Component is now attached; the live engine slice will keep
                // it (Arc) and service its drives/senses. The build-time
                // slice drops it after validation.
            }
        }

        Ok(BuiltSystem { nets, diagnostics })
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

/// Register one component pin's electrical descriptor with the resolver.
fn add_pin_descriptor(resolver: &mut Resolver, net: usize, pin: &PinDecl, pin_ref: &PinRef) {
    match pin.kind {
        PinKind::DigitalIn => resolver.add_digital_sense(net),
        PinKind::Analog => resolver.add_analog_sense(net),
        PinKind::PowerIn => resolver.add_power_sense(net),
        PinKind::PowerOut => {
            // Component-declared rail voltage arrives with the regulator
            // models (a later slice); presence is what the build-time pass
            // needs. NaN marks "sourced at an unmodeled voltage".
            resolver.add_power_source(net, f64::NAN);
        }
        PinKind::DigitalOut | PinKind::DigitalBidir => {
            // Build-time idle: stream producers idle Driven(High) per the
            // stream spec; plain outputs idle High as the documented
            // build-time default (components are not running yet).
            let impedance = pin.drive_impedance.unwrap_or(DEFAULT_PUSH_PULL_IMPEDANCE);
            resolver.add_driver(net, Level::High, impedance, pin_ref.clone());
        }
        PinKind::Passive => {}
    }
}

// ============================================================
// Build-time resolution
// ============================================================

/// Union-find over global net indices.
struct Dsu {
    parent: Vec<usize>,
}

impl Dsu {
    fn new(n: usize) -> Self {
        Self {
            parent: (0..n).collect(),
        }
    }
    fn len(&self) -> usize {
        self.parent.len()
    }
    fn grow(&mut self, n: usize) {
        while self.parent.len() < n {
            self.parent.push(self.parent.len());
        }
    }
    fn find(&mut self, mut x: usize) -> usize {
        while self.parent[x] != x {
            self.parent[x] = self.parent[self.parent[x]];
            x = self.parent[x];
        }
        x
    }
    fn union(&mut self, a: usize, b: usize) {
        let (ra, rb) = (self.find(a), self.find(b));
        if ra != rb {
            self.parent[rb] = ra;
        }
    }
}

/// One push-pull driver contribution.
struct Driver {
    net: usize,
    level: Level,
    #[allow(dead_code)] // impedance projection is the cluster-solver slice
    impedance: f64,
    pin: PinRef,
}

/// Build-time resolution state: descriptors accumulated per global net, then
/// resolved per conduction cluster.
struct Resolver {
    /// Union-find of net *identity* merges (harness wires, pin shorts).
    identity: Dsu,
    /// Conduction edges (resistors, inductors, closed jumpers): (a, b, ohms).
    edges: Vec<(usize, usize, f64)>,
    drivers: Vec<Driver>,
    power_sources: Vec<(usize, Volts)>,
    stuck_sources: Vec<(usize, Volts)>,
    digital_senses: Vec<usize>,
    analog_senses: Vec<usize>,
    power_senses: Vec<usize>,
    net_count: usize,
}

impl Resolver {
    fn new(net_count: usize, identity: Dsu) -> Self {
        Self {
            identity,
            edges: Vec::new(),
            drivers: Vec::new(),
            power_sources: Vec::new(),
            stuck_sources: Vec::new(),
            digital_senses: Vec::new(),
            analog_senses: Vec::new(),
            power_senses: Vec::new(),
            net_count,
        }
    }

    fn add_edge(&mut self, a: usize, b: usize, ohms: f64) {
        self.edges.push((a, b, ohms));
    }
    fn add_driver(&mut self, net: usize, level: Level, impedance: f64, pin: PinRef) {
        self.drivers.push(Driver {
            net,
            level,
            impedance,
            pin,
        });
    }
    fn add_power_source(&mut self, net: usize, volts: Volts) {
        self.power_sources.push((net, volts));
    }
    fn add_stuck_source(&mut self, net: usize, volts: Volts) {
        self.stuck_sources.push((net, volts));
    }
    fn add_digital_sense(&mut self, net: usize) {
        self.digital_senses.push(net);
    }
    fn add_analog_sense(&mut self, net: usize) {
        self.analog_senses.push(net);
    }
    fn add_power_sense(&mut self, net: usize) {
        self.power_senses.push(net);
    }

    /// Run the build-time pass: assign every net a [`NetState`] and report
    /// findings. Identity-merged nets share state; conduction clusters share
    /// sourced-ness.
    fn resolve(&mut self, nets: &mut [Net], diagnostics: &mut Diagnostics) {
        self.identity.grow(self.net_count.max(nets.len()));

        // Conduction clusters: identity merges are 0-ohm, conduction edges
        // connect within a cluster without merging identity.
        let mut conduction = Dsu::new(nets.len());
        for i in 0..nets.len() {
            let root = self.identity.find(i);
            conduction.union(root, i);
        }
        let edges = self.edges.clone();
        for (a, b, _ohms) in &edges {
            let (ra, rb) = (self.identity.find(*a), self.identity.find(*b));
            conduction.union(ra, rb);
        }

        // Sources per conduction cluster.
        let mut cluster_power: HashMap<usize, Volts> = HashMap::new();
        let mut cluster_sourced: HashSet<usize> = HashSet::new();
        let power_sources = self.power_sources.clone();
        let stuck_sources = self.stuck_sources.clone();
        for (net, volts) in &power_sources {
            let c = conduction.find(self.identity.find(*net));
            cluster_power.entry(c).or_insert(*volts);
            cluster_sourced.insert(c);
        }
        for (net, _volts) in &stuck_sources {
            let c = conduction.find(self.identity.find(*net));
            cluster_sourced.insert(c);
        }

        // Drivers: per identity-merged net, detect contention; drivers also
        // source their conduction cluster.
        let mut net_drivers: HashMap<usize, Vec<&Driver>> = HashMap::new();
        for driver in &self.drivers {
            let root = self.identity.find(driver.net);
            net_drivers.entry(root).or_default().push(driver);
            cluster_sourced.insert(conduction.find(root));
        }

        // Direct source levels per identity root (power/stuck beat drivers
        // for state projection).
        let mut direct_volts: HashMap<usize, Volts> = HashMap::new();
        for (net, volts) in &power_sources {
            direct_volts
                .entry(self.identity.find(*net))
                .or_insert(*volts);
        }
        for (net, volts) in &stuck_sources {
            direct_volts
                .entry(self.identity.find(*net))
                .or_insert(*volts);
        }

        // Assign states.
        for i in 0..nets.len() {
            let root = self.identity.find(i);
            let cluster = conduction.find(root);

            let state = if let Some(v) = direct_volts.get(&root) {
                NetState::Analog(*v)
            } else if let Some(drivers) = net_drivers.get(&root) {
                let levels: HashSet<Level> = drivers.iter().map(|d| d.level).collect();
                if levels.len() > 1 {
                    NetState::Contention
                } else {
                    NetState::Driven(drivers[0].level)
                }
            } else if cluster_sourced.contains(&cluster) {
                // Reached only through conduction edges: project as pulled
                // toward the cluster's source. Exact series resistance is the
                // cluster-solver slice; the build-time pass reports the sum
                // of the cluster's edge resistances as an upper bound.
                let total: f64 = edges
                    .iter()
                    .filter(|(a, b, _)| {
                        conduction.find(self.identity.find(*a)) == cluster
                            || conduction.find(self.identity.find(*b)) == cluster
                    })
                    .map(|(_, _, ohms)| ohms)
                    .sum();
                let level = match cluster_power.get(&cluster) {
                    Some(v) if *v < 1.5 => Level::Low,
                    _ => Level::High,
                };
                NetState::Pulled(level, total)
            } else {
                NetState::Floating
            };
            nets[i].state = state;
        }

        // Contention findings (per identity root, deduped).
        let mut reported_contention: HashSet<usize> = HashSet::new();
        for i in 0..nets.len() {
            let root = self.identity.find(i);
            if nets[i].state == NetState::Contention && reported_contention.insert(root) {
                let drivers = net_drivers
                    .get(&root)
                    .map(|ds| ds.iter().map(|d| d.pin.clone()).collect())
                    .unwrap_or_default();
                diagnostics.report(Finding::Contention {
                    net: nets[root.min(i)].name.clone(),
                    drivers,
                });
            }
        }

        // Floating senses (deduped per (identity root, kind)).
        let mut reported: HashSet<(usize, SenseKind)> = HashSet::new();
        let digital = self.digital_senses.clone();
        let analog = self.analog_senses.clone();
        for (senses, kind) in [(digital, SenseKind::Digital), (analog, SenseKind::Analog)] {
            for net in senses {
                let root = self.identity.find(net);
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
        let power = self.power_senses.clone();
        for net in power {
            let root = self.identity.find(net);
            let cluster = conduction.find(root);
            if !cluster_sourced.contains(&cluster) && reported_power.insert(root) {
                diagnostics.report(Finding::PowerNetUnsourced {
                    net: nets[net].name.clone(),
                });
            }
        }
    }
}

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
    use super::*;

    #[test]
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

    #[test]
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

    #[test]
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
