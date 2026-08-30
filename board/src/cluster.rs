//! Analog cluster extraction types + [`ClusterSolver`] seam.
//!
//! Connected subgraphs of `Passive`/`Analog` pins form **clusters**, extracted
//! at build time. Every driver in the net model is a Thevenin source (see
//! `BOARD_ENGINE.md` "Net state model": voltage and impedance). A
//! [`ClusterSolver`] turns that network into node voltages. Digital projection
//! (rails, contention, thresholds) remains the resolver's job; this module
//! only publishes [`NetState::Analog`] or [`NetState::Floating`].
//!
//! Singularity is handled **structurally, before the deck is built**: nodes
//! with no conductive path to any source are reported [`NetState::Floating`]
//! and never sent to ngspice. The solver never invents a voltage and never
//! panics on singular input.
//!
//! Numerical policy (each choice documented at its constant/field):
//! - **Zero-ohm edges** are a hard merge (supernode via union-find), never a
//!   `1/0` resistance card.
//! - **Ideal sources** (0 Ω impedance) are stamped as `V` + series `R` of
//!   [`IDEAL_SOURCE_FLOOR_OHMS`]. Two disagreeing ideal sources on one
//!   supernode then divide instead of forming a SPICE voltage-source loop;
//!   flagging the fight as [`NetState::Contention`] remains the resolver's
//!   projection job.
//! - **Non-finite or negative** resistances and source impedances are guarded:
//!   such edges are open, such sources absent.
//! - **Capacitors** stamp analog-only. They do not join digital conduction
//!   (a decoupling cap must not analog-union the board). Non-positive or
//!   non-finite values are omitted.
//!
//! [`ClusterSolver`] is the analog seam: inject any impl, or use the bundled
//! [`AnalogOff`] / (feature `spice`) ngspice `.op` or windowed `.tran`.

use crate::net::{NetId, NetState, Ohms, Volts};
#[cfg(feature = "spice")]
use embsim_core::virtual_clock;
#[cfg(feature = "spice")]
use std::collections::HashMap;
#[cfg(feature = "spice")]
use std::fmt::Write as _;
#[cfg(feature = "spice")]
use std::sync::Mutex;

// ============================================================
// Solver constants
// ============================================================

/// Impedance floor applied to Thevenin sources when stamping a SPICE `V`+`R`.
///
/// An ideal source (declared impedance 0 Ω) would be a voltage source tied
/// straight to ground. Two disagreeing ideals on one supernode are then a
/// topology error in ngspice; with this floor they resolve to the divided
/// mid-value, and flagging that fight as [`NetState::Contention`] remains the
/// resolver's projection job. The floor keeps solved voltages within a
/// microvolt of ideal for any realistic cluster load (1 A of load current
/// drops 1 µV).
pub const IDEAL_SOURCE_FLOOR_OHMS: Ohms = 1e-6;

// ============================================================
// Cluster topology
// ============================================================

/// One resistive edge inside a cluster (a passive primitive, or a
/// parameterized primitive contributed by a transducer component — e.g. a
/// load-cell bridge leg whose value the consumer's physics plant drives).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ClusterResistor {
    /// First terminal node.
    pub a: NetId,
    /// Second terminal node.
    pub b: NetId,
    /// Edge resistance. Exactly `0.0` is a hard merge (the terminals become
    /// one supernode — never a `1/0` conductance); non-finite or negative
    /// values are guarded as open circuit.
    pub ohms: Ohms,
}

/// A Thevenin source presented to a cluster node (push-pull driver reaching
/// the cluster, power rail, `net_stuck` fault, …). Down power domains present
/// their rail nodes as 0 V sources, not removed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ClusterSource {
    /// Node the source is attached to.
    pub node: NetId,
    /// Open-circuit source voltage.
    pub volts: Volts,
    /// Source impedance. `0.0` is an ideal source, stamped through the
    /// [`IDEAL_SOURCE_FLOOR_OHMS`] series resistor; non-finite or negative
    /// impedance (or non-finite volts) disqualifies the source entirely.
    pub impedance: Ohms,
}

/// One capacitor inside a cluster. Open at DC (`.op`); integrates in `.tran`.
///
/// Does **not** join digital conduction / stream-collapse — only the analog
/// deck. Diodes and vendor `.subckt` instances are a later stamp.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ClusterCapacitor {
    /// First terminal node.
    pub a: NetId,
    /// Second terminal node.
    pub b: NetId,
    /// Capacitance in farads. Non-finite or non-positive values are omitted.
    pub farads: f64,
}

/// Build-time-extracted analog cluster: nodes, resistive edges, capacitors.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Cluster {
    /// Member nodes (nets participating in this cluster).
    pub nodes: Vec<NetId>,
    /// Resistive edges between member nodes.
    pub resistors: Vec<ClusterResistor>,
    /// Capacitors (analog-only; DC-open for digital routing).
    pub capacitors: Vec<ClusterCapacitor>,
}

/// Boundary inputs to a cluster solve — the values that change between
/// recomputations (drives, rail states, transducer primitive values).
#[derive(Debug, Clone, PartialEq, Default)]
pub struct ClusterInputs {
    /// Thevenin sources currently reaching the cluster.
    pub sources: Vec<ClusterSource>,
    /// Analog window in virtual µs for a windowed `.tran` solver. `None`
    /// lets the solver use elapsed virtual time (capped). `Some(0)` is DC
    /// (`.op`). Ignored by [`Spice`] (always `.op`).
    pub window_us: Option<u64>,
}

/// Result of one cluster solve: a state per member node, parallel to
/// [`Cluster::nodes`].
#[derive(Debug, Clone, PartialEq)]
pub struct ClusterSolution {
    /// `(node, solved state)` for every member node.
    pub node_states: Vec<(NetId, NetState)>,
}

impl ClusterSolution {
    /// Solved state of one node, if it belongs to the cluster.
    pub fn state_of(&self, node: NetId) -> Option<NetState> {
        self.node_states
            .iter()
            .find(|(n, _)| *n == node)
            .map(|(_, s)| *s)
    }
}

// ============================================================
// Solver seam
// ============================================================

/// Solves one cluster from its boundary inputs.
///
/// Bundled impls: [`AnalogOff`], ngspice `.op` ([`Spice`]), and windowed
/// `.tran` ([`SpiceTransient`]) when feature `spice` is on. Tests and
/// consumers may install any other impl — the engine only sees this trait.
pub trait ClusterSolver: Send + Sync {
    /// Solve the cluster; must never return garbage — a source-free
    /// cluster solves to [`NetState::Floating`] for all nodes.
    fn solve(&self, cluster: &Cluster, inputs: &ClusterInputs) -> ClusterSolution;

    /// Periodic analog window (virtual µs) the engine should re-resolve on.
    ///
    /// `None` (default) is event-driven analog only (`.op` on escalation).
    /// Windowed `.tran` returns `Some(max_step_us)` so capacitors keep
    /// integrating between digital events. The engine remains the time
    /// authority: it advances at `min(next digital deadline, this window)`.
    fn analog_window_us(&self) -> Option<u64> {
        None
    }
}

/// Analog solver that publishes [`NetState::Floating`] for every node.
///
/// Used when analog is compiled out or the run profile selects
/// [`crate::AnalogBackend::Off`]. Digital resolution is unaffected.
#[derive(Debug, Clone, Copy, Default)]
pub struct AnalogOff;

impl ClusterSolver for AnalogOff {
    fn solve(&self, cluster: &Cluster, _inputs: &ClusterInputs) -> ClusterSolution {
        ClusterSolution {
            node_states: cluster
                .nodes
                .iter()
                .map(|&id| (id, NetState::Floating))
                .collect(),
        }
    }
}

/// Default analog solver: stamp the cluster as a SPICE deck and run ngspice
/// `.op`.
///
/// Requires the `spice` cargo feature (on by default). Capacitors are
/// stamped but open at DC; use [`SpiceTransient`] for windowed `.tran`.
///
/// Pipeline:
/// 1. collapse zero-ohm edges into supernodes (union-find);
/// 2. find the source-reachable supernodes — the complement reports
///    [`NetState::Floating`];
/// 3. stamp `R`/`C` cards and Thevenin `V`+`R` sources (impedance clamped to
///    [`IDEAL_SOURCE_FLOOR_OHMS`]);
/// 4. [`embsim_spice::operating_point`];
/// 5. map supernode voltages back to every member node as
///    [`NetState::Analog`].
#[cfg(feature = "spice")]
#[derive(Debug, Clone, Copy, Default)]
pub struct Spice;

#[cfg(feature = "spice")]
impl ClusterSolver for Spice {
    fn solve(&self, cluster: &Cluster, inputs: &ClusterInputs) -> ClusterSolution {
        spice_solve(cluster, inputs, None, None).0
    }
}

/// Windowed ngspice `.tran` on the same cluster deck as [`Spice`].
///
/// Δt is `min(elapsed virtual µs since the last solve, max_step_us)`, or
/// [`ClusterInputs::window_us`] when a test/engine supplies it. Capacitor
/// voltages from the previous window are restamped as `.ic`. Resistive
/// clusters (no C) fall back to `.op` — `.tran` would be the same DC answer.
///
/// Requires the `spice` cargo feature.
#[cfg(feature = "spice")]
#[derive(Debug)]
pub struct SpiceTransient {
    max_step_us: u64,
    last_us: Mutex<Option<u64>>,
    /// Last analog voltage per cluster node, for `.ic` on the next window.
    ic: Mutex<HashMap<NetId, f64>>,
}

#[cfg(feature = "spice")]
impl SpiceTransient {
    /// `max_step_us` caps a single analog window (0 is treated as 1 µs).
    pub fn new(max_step_us: u64) -> Self {
        Self {
            max_step_us: max_step_us.max(1),
            last_us: Mutex::new(None),
            ic: Mutex::new(HashMap::new()),
        }
    }

    fn window_us(&self, inputs: &ClusterInputs) -> u64 {
        if let Some(w) = inputs.window_us {
            return w.min(self.max_step_us);
        }
        if !virtual_clock::is_initialized() {
            return 0;
        }
        let now = virtual_clock::virtual_us();
        let mut last = self.last_us.lock().unwrap_or_else(|p| p.into_inner());
        let dt = match *last {
            None => 0,
            Some(t) => now.saturating_sub(t).min(self.max_step_us),
        };
        *last = Some(now);
        dt
    }
}

#[cfg(feature = "spice")]
impl ClusterSolver for SpiceTransient {
    fn solve(&self, cluster: &Cluster, inputs: &ClusterInputs) -> ClusterSolution {
        let dt = self.window_us(inputs);
        let mut ic = self.ic.lock().unwrap_or_else(|p| p.into_inner());
        let (solution, next_ic) = spice_solve(cluster, inputs, Some(dt), Some(&ic));
        // Merge per-cluster IC into the process-wide map: one solver serves
        // every escalated island. Drop this cluster's previous entries so a
        // node that went Floating does not keep a stale `.ic`.
        for id in &cluster.nodes {
            ic.remove(id);
        }
        ic.extend(next_ic);
        solution
    }

    fn analog_window_us(&self) -> Option<u64> {
        Some(self.max_step_us)
    }
}

#[cfg(feature = "spice")]
fn floating_all(cluster: &Cluster) -> ClusterSolution {
    ClusterSolution {
        node_states: cluster
            .nodes
            .iter()
            .map(|&id| (id, NetState::Floating))
            .collect(),
    }
}

/// Stamp and run ngspice. `window_us = None` is always `.op` ([`Spice`]).
/// `Some(0)` is `.op`; `Some(dt>0)` is `.tran` when the cluster has C.
///
/// Returns the published solution and the analog voltages to restamp as
/// `.ic` on the next window (empty on a floating/failed solve).
#[cfg(feature = "spice")]
fn spice_solve(
    cluster: &Cluster,
    inputs: &ClusterInputs,
    window_us: Option<u64>,
    ic: Option<&HashMap<NetId, f64>>,
) -> (ClusterSolution, HashMap<NetId, f64>) {
    let n = cluster.nodes.len();

    // Cluster-local dense index per node (first occurrence wins).
    //
    // hash-order: `node_index` is keyed access only (`get`, `entry`, index
    // by dense `i`).
    let mut node_index: HashMap<NetId, usize> = HashMap::with_capacity(n);
    for (i, &id) in cluster.nodes.iter().enumerate() {
        node_index.entry(id).or_insert(i);
    }

    let mut dsu = Dsu::new(n);
    for r in &cluster.resistors {
        let (Some(&a), Some(&b)) = (node_index.get(&r.a), node_index.get(&r.b)) else {
            continue;
        };
        if r.ohms == 0.0 {
            dsu.union(a, b);
        }
    }
    let root_of: Vec<usize> = (0..n).map(|i| dsu.find(i)).collect();

    let mut edges: Vec<(usize, usize, f64)> = Vec::new();
    for r in &cluster.resistors {
        let (Some(&a), Some(&b)) = (node_index.get(&r.a), node_index.get(&r.b)) else {
            continue;
        };
        if !r.ohms.is_finite() || r.ohms <= 0.0 {
            continue;
        }
        let (ra, rb) = (root_of[a], root_of[b]);
        if ra != rb {
            edges.push((ra, rb, r.ohms));
        }
    }

    let mut capacitors: Vec<(usize, usize, f64)> = Vec::new();
    for c in &cluster.capacitors {
        let (Some(&a), Some(&b)) = (node_index.get(&c.a), node_index.get(&c.b)) else {
            continue;
        };
        if !c.farads.is_finite() || c.farads <= 0.0 {
            continue;
        }
        let (ra, rb) = (root_of[a], root_of[b]);
        if ra != rb {
            capacitors.push((ra, rb, c.farads));
        }
    }

    let mut sources: Vec<(usize, f64, f64)> = Vec::new(); // (supernode, V, Z)
    for s in &inputs.sources {
        let Some(&node) = node_index.get(&s.node) else {
            continue;
        };
        if !s.volts.is_finite() || !s.impedance.is_finite() || s.impedance < 0.0 {
            continue;
        }
        sources.push((
            root_of[node],
            s.volts,
            s.impedance.max(IDEAL_SOURCE_FLOOR_OHMS),
        ));
    }

    let mut adjacency: Vec<Vec<usize>> = vec![Vec::new(); n];
    for &(a, b, _) in &edges {
        adjacency[a].push(b);
        adjacency[b].push(a);
    }
    let mut reachable = vec![false; n];
    let mut stack: Vec<usize> = Vec::new();
    for &(root, _, _) in &sources {
        if !reachable[root] {
            reachable[root] = true;
            stack.push(root);
        }
    }
    while let Some(x) = stack.pop() {
        for &y in &adjacency[x] {
            if !reachable[y] {
                reachable[y] = true;
                stack.push(y);
            }
        }
    }

    let prev_ic = ic.cloned().unwrap_or_default();
    if sources.is_empty() {
        return (floating_all(cluster), HashMap::new());
    }

    // `.ic` cards in supernode order — never walk the HashMap into the deck.
    let mut ic_by_root: HashMap<usize, f64> = HashMap::new();
    let mut ic_pairs: Vec<(usize, f64)> = prev_ic
        .iter()
        .filter_map(|(id, v)| {
            let idx = *node_index.get(id)?;
            v.is_finite().then_some((root_of[idx], *v))
        })
        .collect();
    ic_pairs.sort_by_key(|(root, _)| *root);
    for (root, v) in ic_pairs {
        ic_by_root.entry(root).or_insert(v);
    }
    let mut ic_cards: Vec<(usize, f64)> = ic_by_root.into_iter().collect();
    ic_cards.sort_by_key(|(root, _)| *root);

    let tran_us = window_us.filter(|&dt| dt > 0 && !capacitors.is_empty());
    let cards = stamp_deck(
        &edges,
        &sources,
        &capacitors,
        &reachable,
        tran_us.is_some().then_some(ic_cards.as_slice()),
    );
    let refs: Vec<&str> = cards.iter().map(String::as_str).collect();
    let voltages = if let Some(dt) = tran_us {
        let tstop_s = dt as f64 * 1e-6;
        let tstep_s = (tstop_s / 100.0).clamp(1e-12, tstop_s);
        match embsim_spice::transient(&refs, tstop_s, tstep_s) {
            Ok(v) => v,
            Err(_) => return (floating_all(cluster), prev_ic),
        }
    } else {
        match embsim_spice::operating_point(&refs) {
            Ok(v) => v,
            Err(_) => return (floating_all(cluster), prev_ic),
        }
    };

    let mut next_ic = HashMap::new();
    let node_states = cluster
        .nodes
        .iter()
        .map(|&id| {
            let root = root_of[node_index[&id]];
            let state = if reachable[root] {
                match spice_volts(&voltages, root) {
                    Some(v) => {
                        next_ic.insert(id, v);
                        NetState::Analog(v)
                    }
                    None => NetState::Floating,
                }
            } else {
                NetState::Floating
            };
            (id, state)
        })
        .collect();
    (ClusterSolution { node_states }, next_ic)
}

#[cfg(feature = "spice")]
fn stamp_deck(
    edges: &[(usize, usize, f64)],
    sources: &[(usize, f64, f64)],
    capacitors: &[(usize, usize, f64)],
    reachable: &[bool],
    ic: Option<&[(usize, f64)]>,
) -> Vec<String> {
    let mut cards = Vec::new();
    for (i, &(a, b, ohms)) in edges.iter().enumerate() {
        if !reachable[a] || !reachable[b] {
            continue;
        }
        cards.push(format!("R{i} n{a} n{b} {ohms}"));
    }
    for (i, &(a, b, farads)) in capacitors.iter().enumerate() {
        if !reachable[a] || !reachable[b] {
            continue;
        }
        cards.push(format!("C{i} n{a} n{b} {farads}"));
    }
    for (i, &(root, volts, z)) in sources.iter().enumerate() {
        if !reachable[root] {
            continue;
        }
        // Thevenin vs ground: V on an internal node, series R into the net.
        let mut card = String::new();
        let _ = write!(card, "V{i} vs{i} 0 {volts}");
        cards.push(card);
        cards.push(format!("Rs{i} vs{i} n{root} {z}"));
    }
    if let Some(ic) = ic {
        for &(root, volts) in ic {
            if reachable[root] {
                cards.push(format!(".ic v(n{root})={volts}"));
            }
        }
    }
    cards
}

#[cfg(feature = "spice")]
fn spice_volts(map: &HashMap<String, f64>, root: usize) -> Option<Volts> {
    let key = format!("n{root}");
    map.get(&key)
        .copied()
        .or_else(|| map.get(&format!("v({key})")).copied())
}

// ============================================================
// Solver internals
// ============================================================

/// Union-find over cluster-local node indices (zero-ohm supernode merges).
/// Mirrors the resolver's build-time `Dsu` in `system.rs`.
#[cfg(feature = "spice")]
struct Dsu {
    parent: Vec<usize>,
}

#[cfg(feature = "spice")]
impl Dsu {
    fn new(n: usize) -> Self {
        Self {
            parent: (0..n).collect(),
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

#[cfg(test)]
mod off_tests {
    use super::*;

    #[test]
    fn analog_off_floats_every_node() {
        let cluster = Cluster {
            nodes: vec![NetId(0), NetId(1)],
            resistors: vec![],
            ..Default::default()
        };
        let solution = AnalogOff.solve(
            &cluster,
            &ClusterInputs {
                sources: vec![ClusterSource {
                    node: NetId(0),
                    volts: 3.3,
                    impedance: 25.0,
                }],
                ..Default::default()
            },
        );
        assert_eq!(solution.state_of(NetId(0)), Some(NetState::Floating));
        assert_eq!(solution.state_of(NetId(1)), Some(NetState::Floating));
    }
}

#[cfg(all(test, feature = "spice"))]
mod tests {
    use rstest::rstest;

    use super::*;

    fn three_node_cluster() -> Cluster {
        Cluster {
            nodes: vec![NetId(0), NetId(1), NetId(2)],
            resistors: vec![
                ClusterResistor {
                    a: NetId(0),
                    b: NetId(1),
                    ohms: 47.0,
                },
                ClusterResistor {
                    a: NetId(1),
                    b: NetId(2),
                    ohms: 4_700.0,
                },
            ],
            ..Default::default()
        }
    }

    fn sourced(node: NetId, volts: Volts, impedance: Ohms) -> ClusterInputs {
        ClusterInputs {
            sources: vec![ClusterSource {
                node,
                volts,
                impedance,
            }],
            ..Default::default()
        }
    }

    fn analog_volts(solution: &ClusterSolution, node: NetId) -> Volts {
        match solution.state_of(node) {
            Some(NetState::Analog(v)) => v,
            other => panic!("expected Analog at {node:?}, got {other:?}"),
        }
    }

    #[rstest]
    fn source_free_cluster_solves_floating_for_all_nodes() {
        let solution = Spice.solve(&three_node_cluster(), &ClusterInputs::default());
        assert_eq!(solution.node_states.len(), 3);
        for (_, state) in &solution.node_states {
            assert_eq!(*state, NetState::Floating);
        }
        assert_eq!(solution.state_of(NetId(1)), Some(NetState::Floating));
        assert_eq!(solution.state_of(NetId(9)), None);
    }

    #[rstest]
    fn sourced_cluster_is_not_floating() {
        let inputs = sourced(NetId(0), 3.3, 25.0);
        let solution = Spice.solve(&three_node_cluster(), &inputs);
        for (_, state) in &solution.node_states {
            assert_ne!(*state, NetState::Floating);
        }
    }

    #[rstest]
    fn unloaded_chain_sits_at_the_source_open_circuit_voltage() {
        // One source, no return path: no current flows, so every node solves
        // to the source's open-circuit voltage exactly.
        let solution = Spice.solve(&three_node_cluster(), &sourced(NetId(0), 3.3, 25.0));
        for node in [NetId(0), NetId(1), NetId(2)] {
            assert!((analog_volts(&solution, node) - 3.3).abs() < 1e-9);
        }
    }

    #[rstest]
    fn zero_ohm_edge_merges_nodes_into_a_supernode() {
        // 3.3 V ideal at n0; 0 Ω n0–n1 (hard merge); 100 Ω n1–n2;
        // 100 Ω n2–n3; 0 V ideal at n3. Hand check: n0 = n1 = 3.3 V,
        // n2 = 1.65 V (midpoint of two equal legs), n3 = 0 V.
        let cluster = Cluster {
            nodes: vec![NetId(0), NetId(1), NetId(2), NetId(3)],
            resistors: vec![
                ClusterResistor {
                    a: NetId(0),
                    b: NetId(1),
                    ohms: 0.0,
                },
                ClusterResistor {
                    a: NetId(1),
                    b: NetId(2),
                    ohms: 100.0,
                },
                ClusterResistor {
                    a: NetId(2),
                    b: NetId(3),
                    ohms: 100.0,
                },
            ],
            ..Default::default()
        };
        let inputs = ClusterInputs {
            sources: vec![
                ClusterSource {
                    node: NetId(0),
                    volts: 3.3,
                    impedance: 0.0,
                },
                ClusterSource {
                    node: NetId(3),
                    volts: 0.0,
                    impedance: 0.0,
                },
            ],
            ..Default::default()
        };
        let solution = Spice.solve(&cluster, &inputs);
        assert!((analog_volts(&solution, NetId(0)) - 3.3).abs() < 1e-6);
        assert!((analog_volts(&solution, NetId(1)) - 3.3).abs() < 1e-6);
        assert!((analog_volts(&solution, NetId(2)) - 1.65).abs() < 1e-6);
        assert!(analog_volts(&solution, NetId(3)).abs() < 1e-6);
    }

    #[rstest]
    fn non_finite_and_negative_edges_are_open() {
        // n0 sourced; n1 behind an infinite edge, n2 behind a NaN edge,
        // n3 behind a negative edge — all three are open, hence Floating.
        let cluster = Cluster {
            nodes: vec![NetId(0), NetId(1), NetId(2), NetId(3)],
            resistors: vec![
                ClusterResistor {
                    a: NetId(0),
                    b: NetId(1),
                    ohms: f64::INFINITY,
                },
                ClusterResistor {
                    a: NetId(0),
                    b: NetId(2),
                    ohms: f64::NAN,
                },
                ClusterResistor {
                    a: NetId(0),
                    b: NetId(3),
                    ohms: -47.0,
                },
            ],
            ..Default::default()
        };
        let inputs = sourced(NetId(0), 3.3, 25.0);
        let solution = Spice.solve(&cluster, &inputs);
        assert!((analog_volts(&solution, NetId(0)) - 3.3).abs() < 1e-9);
        for node in [NetId(1), NetId(2), NetId(3)] {
            assert_eq!(solution.state_of(node), Some(NetState::Floating));
        }
    }

    #[rstest]
    fn invalid_sources_are_ignored() {
        // Non-finite volts/impedance, negative impedance, and a source on a
        // node outside the cluster all contribute nothing → all Floating.
        let inputs = ClusterInputs {
            sources: vec![
                ClusterSource {
                    node: NetId(0),
                    volts: f64::NAN,
                    impedance: 25.0,
                },
                ClusterSource {
                    node: NetId(0),
                    volts: 3.3,
                    impedance: f64::INFINITY,
                },
                ClusterSource {
                    node: NetId(0),
                    volts: 3.3,
                    impedance: -25.0,
                },
                ClusterSource {
                    node: NetId(42),
                    volts: 3.3,
                    impedance: 25.0,
                },
            ],
            ..Default::default()
        };
        let solution = Spice.solve(&three_node_cluster(), &inputs);
        for (_, state) in &solution.node_states {
            assert_eq!(*state, NetState::Floating);
        }
    }

    #[rstest]
    fn ideal_source_is_clamped_not_divided_by_zero() {
        let cluster = Cluster {
            nodes: vec![NetId(0)],
            resistors: vec![],
            ..Default::default()
        };
        let solution = Spice.solve(&cluster, &sourced(NetId(0), 3.3, 0.0));
        assert!((analog_volts(&solution, NetId(0)) - 3.3).abs() < 1e-9);
    }

    fn rc_cluster() -> Cluster {
        // 1 V — 1 kΩ — n1 — 1 µF — 0 V. τ = 1 ms.
        Cluster {
            nodes: vec![NetId(0), NetId(1), NetId(2)],
            resistors: vec![ClusterResistor {
                a: NetId(0),
                b: NetId(1),
                ohms: 1_000.0,
            }],
            capacitors: vec![ClusterCapacitor {
                a: NetId(1),
                b: NetId(2),
                farads: 1e-6,
            }],
        }
    }

    fn rc_step_inputs(window_us: u64) -> ClusterInputs {
        ClusterInputs {
            sources: vec![
                ClusterSource {
                    node: NetId(0),
                    volts: 1.0,
                    impedance: 0.0,
                },
                ClusterSource {
                    node: NetId(2),
                    volts: 0.0,
                    impedance: 0.0,
                },
            ],
            window_us: Some(window_us),
        }
    }

    /// `.op` treats C as open: the cap top sits at the source OCV.
    #[rstest]
    fn spice_op_treats_capacitor_as_open() {
        let solution = Spice.solve(&rc_cluster(), &rc_step_inputs(0));
        assert!((analog_volts(&solution, NetId(1)) - 1.0).abs() < 1e-6);
        assert!(analog_volts(&solution, NetId(2)).abs() < 1e-6);
    }

    /// Windowed `.tran` from a discharged cap: v(τ) = 1 − e^{−1} ≈ 0.632.
    #[rstest]
    fn spice_transient_rc_step_at_one_tau() {
        let solver = SpiceTransient::new(10_000);
        let solution = solver.solve(&rc_cluster(), &rc_step_inputs(1_000));
        let expected = 1.0 - (-1.0f64).exp();
        assert!(
            (analog_volts(&solution, NetId(1)) - expected).abs() < 1e-3,
            "got {} expected {expected}",
            analog_volts(&solution, NetId(1))
        );
    }

    /// Two half-tau windows with `.ic` continuity equal one full tau.
    #[rstest]
    fn spice_transient_ic_continues_across_windows() {
        let solver = SpiceTransient::new(10_000);
        let cluster = rc_cluster();
        let first = solver.solve(&cluster, &rc_step_inputs(500));
        let expected_half = 1.0 - (-0.5f64).exp();
        assert!((analog_volts(&first, NetId(1)) - expected_half).abs() < 5e-3);
        let second = solver.solve(&cluster, &rc_step_inputs(500));
        let expected_full = 1.0 - (-1.0f64).exp();
        assert!(
            (analog_volts(&second, NetId(1)) - expected_full).abs() < 5e-3,
            "got {} expected {expected_full}",
            analog_volts(&second, NetId(1))
        );
    }

    /// Resistive clusters fall back to `.op` even when a window is set.
    #[rstest]
    fn spice_transient_without_c_matches_op() {
        let solver = SpiceTransient::new(1_000);
        let inputs = ClusterInputs {
            sources: vec![ClusterSource {
                node: NetId(0),
                volts: 3.3,
                impedance: 25.0,
            }],
            window_us: Some(1_000),
        };
        let tran = solver.solve(&three_node_cluster(), &inputs);
        let op = Spice.solve(
            &three_node_cluster(),
            &ClusterInputs {
                sources: inputs.sources.clone(),
                window_us: None,
            },
        );
        for node in [NetId(0), NetId(1), NetId(2)] {
            assert!((analog_volts(&tran, node) - analog_volts(&op, node)).abs() < 1e-9);
        }
    }

    #[rstest]
    fn analog_window_us_is_the_tran_cap() {
        assert_eq!(SpiceTransient::new(1_000).analog_window_us(), Some(1_000));
        assert_eq!(Spice.analog_window_us(), None);
        assert_eq!(AnalogOff.analog_window_us(), None);
    }

    #[rstest]
    fn non_positive_capacitors_are_omitted() {
        let cluster = Cluster {
            nodes: vec![NetId(0), NetId(1)],
            resistors: vec![],
            capacitors: vec![
                ClusterCapacitor {
                    a: NetId(0),
                    b: NetId(1),
                    farads: 0.0,
                },
                ClusterCapacitor {
                    a: NetId(0),
                    b: NetId(1),
                    farads: f64::NAN,
                },
                ClusterCapacitor {
                    a: NetId(0),
                    b: NetId(1),
                    farads: -1e-6,
                },
            ],
        };
        // No conductive path and no valid C: n1 is Floating; n0 is sourced.
        let solution = Spice.solve(&cluster, &sourced(NetId(0), 3.3, 0.0));
        assert!((analog_volts(&solution, NetId(0)) - 3.3).abs() < 1e-9);
        assert_eq!(solution.state_of(NetId(1)), Some(NetState::Floating));
    }
}
