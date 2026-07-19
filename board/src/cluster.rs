//! Analog cluster extraction types + quasi-static MNA solver.
//!
//! Connected subgraphs of `Passive`/`Analog` pins form **clusters**, extracted
//! at build time and solved by quasi-static modified nodal analysis (MNA):
//! Thevenin sources + resistors → node voltages, recomputed only when a
//! boundary input changes.
//!
//! # Method (provenance)
//!
//! Governing method: quasi-static **modified nodal analysis** (Ho, Ruehli,
//! Brennan 1975, "The Modified Nodal Approach to Network Analysis"), reduced
//! to pure nodal form. Every driver in the net model is a Thevenin source
//! (voltage + impedance — see `BOARD_ENGINE.md` "Net state model"), so each
//! source is **Norton-converted** — a current injection `V/Z` plus a shunt
//! conductance `1/Z` stamped at its node — which eliminates the branch-current
//! unknowns full MNA would otherwise add. The nodal system `G · V = I` is then
//! solved by dense Gaussian elimination with partial pivoting; clusters are
//! small (a handful of nets), so dense elimination is the right tool.
//!
//! Singularity is handled **structurally, before assembly**: the nodes with no
//! conductive path to any source are exactly the MNA-singular ones, so the
//! solver finds the source-reachable supernode set first, assembles and solves
//! only that subgraph, and reports [`NetState::Floating`] for the rest. Every
//! solved block contains at least one Norton shunt conductance on its
//! diagonal, making it strictly diagonally dominant on the sourced rows and
//! nonsingular. The solver never invents a voltage and never panics on
//! singular input.
//!
//! Numerical policy (each choice documented at its constant/field):
//! - **Zero-ohm edges** are a hard merge (supernode via union-find), never a
//!   `1/0` conductance.
//! - **Ideal sources** (0 Ω impedance) are Norton-converted through the
//!   [`IDEAL_SOURCE_FLOOR_OHMS`] clamp rather than node elimination.
//! - **Non-finite or negative** resistances and source impedances are guarded:
//!   such edges are open, such sources absent — never `1/0`, never NaN in the
//!   matrix.
//!
//! Output states stay [`NetState::Analog`] — digital projection (rails,
//! contention, thresholds) remains the resolver's job.
//!
//! The [`ClusterSolver`] trait is the deliberate seam: the default is
//! [`QuasiStaticMna`]; a transient SPICE-backed solver is a possible future
//! implementation and is intentionally NOT part of this design.

use crate::net::{NetId, NetState, Ohms, Volts};
use std::collections::HashMap;

// ============================================================
// Solver constants
// ============================================================

/// Impedance floor applied to Thevenin sources during Norton conversion.
///
/// An ideal source (declared impedance 0 Ω) has no finite Norton equivalent,
/// so the solver clamps every source impedance up to this floor (1 µΩ)
/// instead of eliminating the node. The alternative — Dirichlet node
/// elimination — was rejected because two disagreeing ideal sources on one
/// supernode would leave no consistent constraint; with a finite floor they
/// resolve to the divided mid-value, and flagging that fight as
/// [`NetState::Contention`] remains the resolver's projection job. The floor
/// keeps solved voltages within a microvolt of ideal for any realistic
/// cluster load (1 A of load current drops 1 µV) while keeping the
/// conductance matrix finite and well-pivoted at cluster sizes.
pub const IDEAL_SOURCE_FLOOR_OHMS: Ohms = 1e-6;

/// Pivot magnitude below which elimination reports the matrix singular.
/// Defensive only: reachability filtering guarantees every assembled block is
/// sourced and therefore nonsingular — tripping this returns Floating for the
/// block's nodes, never garbage.
const SINGULAR_PIVOT: f64 = 1e-30;

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
    /// Source impedance. `0.0` is an ideal source, Norton-converted through
    /// the [`IDEAL_SOURCE_FLOOR_OHMS`] clamp; non-finite or negative
    /// impedance (or non-finite volts) disqualifies the source entirely.
    pub impedance: Ohms,
}

/// Build-time-extracted analog cluster: the node set and its resistive edges.
///
/// TODO(board-engine): single-pole RC closed form (time constant annotated on
/// the cluster; senses read the exponential at read time) is a later slice.
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

/// Solves one cluster from its boundary inputs. Deliberate seam for future
/// higher-fidelity solvers; [`QuasiStaticMna`] is the default.
pub trait ClusterSolver: Send + Sync {
    /// Solve the cluster; must never return garbage — a source-free
    /// (MNA-singular) cluster solves to [`NetState::Floating`] for all nodes.
    fn solve(&self, cluster: &Cluster, inputs: &ClusterInputs) -> ClusterSolution;
}

/// Default quasi-static MNA solver.
///
/// Pipeline (see the module docs for the governing method and numerical
/// policy):
/// 1. collapse zero-ohm edges into supernodes (union-find);
/// 2. Norton-convert valid Thevenin sources (impedance clamped to
///    [`IDEAL_SOURCE_FLOOR_OHMS`]);
/// 3. find the source-reachable supernodes — the complement is exactly the
///    MNA-singular set and reports [`NetState::Floating`];
/// 4. stamp conductances and Norton injections into `G · V = I` over the
///    reachable subgraph and solve by Gaussian elimination with partial
///    pivoting;
/// 5. map supernode voltages back to every member node as
///    [`NetState::Analog`].
#[derive(Debug, Clone, Copy, Default)]
pub struct QuasiStaticMna;

impl ClusterSolver for QuasiStaticMna {
    fn solve(&self, cluster: &Cluster, inputs: &ClusterInputs) -> ClusterSolution {
        let n = cluster.nodes.len();

        // Cluster-local dense index per node (first occurrence wins).
        let mut node_index: HashMap<NetId, usize> = HashMap::with_capacity(n);
        for (i, &id) in cluster.nodes.iter().enumerate() {
            node_index.entry(id).or_insert(i);
        }

        // Zero-ohm edges are a hard merge: union the terminals into one
        // supernode rather than stamping a 1/0 conductance. Edges naming a
        // node outside the cluster are ignored (defensive — never panic).
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

        // Conductive edges between distinct supernodes. Non-finite or
        // negative ohms are guarded as open circuit (an infinite resistance
        // conducts nothing; NaN/negative are defect inputs treated the same
        // way rather than poisoning the matrix).
        let mut edges: Vec<(usize, usize, f64)> = Vec::new();
        for r in &cluster.resistors {
            let (Some(&a), Some(&b)) = (node_index.get(&r.a), node_index.get(&r.b)) else {
                continue;
            };
            if !r.ohms.is_finite() || r.ohms <= 0.0 {
                continue; // 0.0 already merged above; the rest are open
            }
            let (ra, rb) = (root_of[a], root_of[b]);
            if ra != rb {
                edges.push((ra, rb, 1.0 / r.ohms));
            }
        }

        // Norton conversion of the valid Thevenin sources:
        // (V, Z) → current injection V/Z with shunt conductance 1/Z, with Z
        // clamped to IDEAL_SOURCE_FLOOR_OHMS so ideal sources stay finite.
        // Invalid sources (non-finite volts/impedance, negative impedance,
        // node outside the cluster) contribute nothing.
        let mut sources: Vec<(usize, f64, f64)> = Vec::new(); // (supernode, G, I)
        for s in &inputs.sources {
            let Some(&node) = node_index.get(&s.node) else {
                continue;
            };
            if !s.volts.is_finite() || !s.impedance.is_finite() || s.impedance < 0.0 {
                continue;
            }
            let g = 1.0 / s.impedance.max(IDEAL_SOURCE_FLOOR_OHMS);
            sources.push((root_of[node], g, s.volts * g));
        }

        // Source reachability over the supernode graph: the unreachable
        // supernodes are exactly the MNA-singular ones — they solve to
        // Floating and are excluded from the matrix.
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

        // Compact matrix index over the reachable supernodes.
        let mut compact: Vec<Option<usize>> = vec![None; n];
        let mut m = 0usize;
        for (slot, &is_reachable) in compact.iter_mut().zip(reachable.iter()) {
            if is_reachable {
                *slot = Some(m);
                m += 1;
            }
        }

        // Assemble G · V = I. Resistor stamp: +g on both diagonals, −g on
        // both off-diagonals. Norton source stamp: +G on the diagonal, +I on
        // the right-hand side.
        let mut matrix = vec![vec![0.0f64; m]; m];
        let mut rhs = vec![0.0f64; m];
        for &(a, b, g) in &edges {
            let (Some(ca), Some(cb)) = (compact[a], compact[b]) else {
                continue; // unreachable block — solved as Floating instead
            };
            matrix[ca][ca] += g;
            matrix[cb][cb] += g;
            matrix[ca][cb] -= g;
            matrix[cb][ca] -= g;
        }
        for &(root, g, i) in &sources {
            let Some(c) = compact[root] else {
                continue; // source roots are always reachable; defensive
            };
            matrix[c][c] += g;
            rhs[c] += i;
        }

        let voltages = solve_dense(matrix, rhs);

        // Map supernode voltages back per member node; unreachable nodes (and
        // the defensive singular-solve fallback) report Floating — a voltage
        // is never invented.
        let node_states = cluster
            .nodes
            .iter()
            .map(|&id| {
                let root = root_of[node_index[&id]];
                let state = match (&voltages, compact[root]) {
                    (Some(v), Some(c)) => NetState::Analog(v[c]),
                    _ => NetState::Floating,
                };
                (id, state)
            })
            .collect();
        ClusterSolution { node_states }
    }
}

// ============================================================
// Solver internals
// ============================================================

/// Union-find over cluster-local node indices (zero-ohm supernode merges).
/// Mirrors the resolver's build-time `Dsu` in `system.rs`.
struct Dsu {
    parent: Vec<usize>,
}

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

/// Dense Gaussian elimination with partial pivoting on `matrix · v = rhs`.
///
/// Returns `None` if a pivot collapses below [`SINGULAR_PIVOT`] or the
/// solution is non-finite — defensive only: reachability filtering guarantees
/// every assembled block carries at least one Norton shunt conductance on its
/// diagonal, which keeps the block nonsingular.
fn solve_dense(mut matrix: Vec<Vec<f64>>, mut rhs: Vec<f64>) -> Option<Vec<Volts>> {
    let n = rhs.len();
    for col in 0..n {
        // Partial pivot: bring the largest |entry| in this column up.
        let pivot_row =
            (col..n).max_by(|&r1, &r2| matrix[r1][col].abs().total_cmp(&matrix[r2][col].abs()))?;
        let pivot_abs = matrix[pivot_row][col].abs();
        if pivot_abs.is_nan() || pivot_abs < SINGULAR_PIVOT {
            return None;
        }
        matrix.swap(col, pivot_row);
        rhs.swap(col, pivot_row);

        // Eliminate the column below the pivot.
        let pivot_vals = matrix[col].clone();
        let pivot_rhs = rhs[col];
        for (row_vals, row_rhs) in matrix.iter_mut().zip(rhs.iter_mut()).skip(col + 1) {
            let factor = row_vals[col] / pivot_vals[col];
            if factor == 0.0 {
                continue;
            }
            for (rv, pv) in row_vals.iter_mut().zip(pivot_vals.iter()).skip(col) {
                *rv -= factor * pv;
            }
            *row_rhs -= factor * pivot_rhs;
        }
    }

    // Back-substitution on the upper triangle.
    let mut v = vec![0.0f64; n];
    for row in (0..n).rev() {
        let mut acc = rhs[row];
        for (coeff, solved) in matrix[row].iter().zip(v.iter()).skip(row + 1) {
            acc -= coeff * solved;
        }
        v[row] = acc / matrix[row][row];
    }
    if v.iter().all(|x| x.is_finite()) {
        Some(v)
    } else {
        None
    }
}

#[cfg(test)]
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
        let solution = QuasiStaticMna.solve(&three_node_cluster(), &ClusterInputs::default());
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
        let solution = QuasiStaticMna.solve(&three_node_cluster(), &inputs);
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
        let solution = QuasiStaticMna.solve(&three_node_cluster(), &inputs);
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
        let solution = QuasiStaticMna.solve(&cluster, &inputs);
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
        let solution = QuasiStaticMna.solve(&cluster, &inputs);
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
        let solution = QuasiStaticMna.solve(&three_node_cluster(), &inputs);
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
        let solution = QuasiStaticMna.solve(&cluster, &inputs);
        assert!((analog_volts(&solution, NetId(0)) - 3.3).abs() < 1e-9);
    }
}
