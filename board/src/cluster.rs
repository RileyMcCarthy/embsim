//! Analog cluster extraction types + quasi-static MNA solver seam.
//!
//! Connected subgraphs of `Passive`/`Analog` pins form **clusters**, extracted
//! at build time and solved by quasi-static modified nodal analysis (MNA):
//! Thevenin sources + resistors → node voltages, recomputed only when a
//! boundary input changes.
//!
//! The [`ClusterSolver`] trait is the deliberate seam: the default is
//! [`QuasiStaticMna`]; a transient SPICE-backed solver is a possible future
//! implementation and is intentionally NOT part of this design.
//!
//! Slice status: [`QuasiStaticMna`] currently only detects **source-free
//! clusters** (all nodes solve to [`NetState::Floating`] — the MNA-singular
//! case). The full nodal solve is a later slice.

use crate::net::{NetId, NetState, Ohms, Volts};

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
    /// Edge resistance.
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
    /// Source impedance.
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
/// Slice status: detects source-free clusters (all nodes
/// [`NetState::Floating`]); the full nodal analysis is a later slice.
#[derive(Debug, Clone, Copy, Default)]
pub struct QuasiStaticMna;

impl ClusterSolver for QuasiStaticMna {
    fn solve(&self, cluster: &Cluster, inputs: &ClusterInputs) -> ClusterSolution {
        if inputs.sources.is_empty() {
            // No source reaches any node: the MNA is singular for the whole
            // cluster — report Floating rather than inventing a voltage.
            return ClusterSolution {
                node_states: cluster
                    .nodes
                    .iter()
                    .map(|&n| (n, NetState::Floating))
                    .collect(),
            };
        }

        // TODO(board-engine): full quasi-static MNA (Thevenin sources +
        // resistors → node voltages, hand-checked to µV in tests). Placeholder:
        // present every node at the lowest-impedance source's open-circuit
        // voltage so sourced clusters are visibly non-floating.
        let strongest = inputs
            .sources
            .iter()
            .min_by(|a, b| a.impedance.total_cmp(&b.impedance))
            .expect("sources is non-empty");
        ClusterSolution {
            node_states: cluster
                .nodes
                .iter()
                .map(|&n| (n, NetState::Analog(strongest.volts)))
                .collect(),
        }
    }
}

#[cfg(test)]
mod tests {
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

    #[test]
    fn source_free_cluster_solves_floating_for_all_nodes() {
        let solution = QuasiStaticMna.solve(&three_node_cluster(), &ClusterInputs::default());
        assert_eq!(solution.node_states.len(), 3);
        for (_, state) in &solution.node_states {
            assert_eq!(*state, NetState::Floating);
        }
        assert_eq!(solution.state_of(NetId(1)), Some(NetState::Floating));
        assert_eq!(solution.state_of(NetId(9)), None);
    }

    #[test]
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
}
