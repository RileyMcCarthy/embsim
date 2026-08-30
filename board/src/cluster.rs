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
//!
//! [`ClusterSolver`] is the analog seam: inject any impl, or use the bundled
//! [`AnalogOff`] / (feature `spice`) ngspice `.op` solver.

use crate::net::{NetId, NetState, Ohms, Volts};
#[cfg(feature = "spice")]
use std::collections::HashMap;
#[cfg(feature = "spice")]
use std::fmt::Write as _;

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

/// Build-time-extracted analog cluster: the node set and its resistive edges.
///
/// TODO(board-engine): capacitors, diodes, and vendor `.subckt` instances
/// stamp as additional SPICE cards; the solver already is ngspice.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Cluster {
    /// Member nodes (nets participating in this cluster).
    pub nodes: Vec<NetId>,
    /// Resistive edges between member nodes.
    pub resistors: Vec<ClusterResistor>,
}

/// Boundary inputs to a cluster solve — the values that change between
/// recomputations (drives, rail states, transducer primitive values).
#[derive(Debug, Clone, PartialEq, Default)]
pub struct ClusterInputs {
    /// Thevenin sources currently reaching the cluster.
    pub sources: Vec<ClusterSource>,
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
/// Bundled impls: [`AnalogOff`], and ngspice `.op` when feature `spice` is on.
/// Tests and consumers may install any other impl — the engine only sees this trait.
pub trait ClusterSolver: Send + Sync {
    /// Solve the cluster; must never return garbage — a source-free
    /// cluster solves to [`NetState::Floating`] for all nodes.
    fn solve(&self, cluster: &Cluster, inputs: &ClusterInputs) -> ClusterSolution;
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
/// Requires the `spice` cargo feature (on by default).
///
/// Pipeline:
/// 1. collapse zero-ohm edges into supernodes (union-find);
/// 2. find the source-reachable supernodes — the complement reports
///    [`NetState::Floating`];
/// 3. stamp `R` cards and Thevenin `V`+`R` sources (impedance clamped to
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

        if sources.is_empty() {
            return floating_all(cluster);
        }

        let cards = stamp_deck(&edges, &sources, &reachable);
        let voltages = match embsim_spice::operating_point(
            &cards.iter().map(String::as_str).collect::<Vec<_>>(),
        ) {
            Ok(v) => v,
            Err(_) => return floating_all(cluster),
        };

        let node_states = cluster
            .nodes
            .iter()
            .map(|&id| {
                let root = root_of[node_index[&id]];
                let state = if reachable[root] {
                    match spice_volts(&voltages, root) {
                        Some(v) => NetState::Analog(v),
                        None => NetState::Floating,
                    }
                } else {
                    NetState::Floating
                };
                (id, state)
            })
            .collect();
        ClusterSolution { node_states }
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

#[cfg(feature = "spice")]
fn stamp_deck(
    edges: &[(usize, usize, f64)],
    sources: &[(usize, f64, f64)],
    reachable: &[bool],
) -> Vec<String> {
    let mut cards = Vec::new();
    for (i, &(a, b, ohms)) in edges.iter().enumerate() {
        if !reachable[a] || !reachable[b] {
            continue;
        }
        cards.push(format!("R{i} n{a} n{b} {ohms}"));
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
        };
        let solution = AnalogOff.solve(
            &cluster,
            &ClusterInputs {
                sources: vec![ClusterSource {
                    node: NetId(0),
                    volts: 3.3,
                    impedance: 25.0,
                }],
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
        let inputs = ClusterInputs {
            sources: vec![ClusterSource {
                node: NetId(0),
                volts: 3.3,
                impedance: 25.0,
            }],
        };
        let solution = Spice.solve(&three_node_cluster(), &inputs);
        for (_, state) in &solution.node_states {
            assert_ne!(*state, NetState::Floating);
        }
    }

    #[rstest]
    fn unloaded_chain_sits_at_the_source_open_circuit_voltage() {
        // One source, no return path: no current flows, so every node solves
        // to the source's open-circuit voltage exactly.
        let inputs = ClusterInputs {
            sources: vec![ClusterSource {
                node: NetId(0),
                volts: 3.3,
                impedance: 25.0,
            }],
        };
        let solution = Spice.solve(&three_node_cluster(), &inputs);
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
        };
        let inputs = ClusterInputs {
            sources: vec![ClusterSource {
                node: NetId(0),
                volts: 3.3,
                impedance: 25.0,
            }],
        };
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
        };
        let inputs = ClusterInputs {
            sources: vec![ClusterSource {
                node: NetId(0),
                volts: 3.3,
                impedance: 0.0,
            }],
        };
        let solution = Spice.solve(&cluster, &inputs);
        assert!((analog_volts(&solution, NetId(0)) - 3.3).abs() < 1e-9);
    }
}
