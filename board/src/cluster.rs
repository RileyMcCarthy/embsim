//! Analog cluster extraction types + quasi-static MNA solver.
//!
//! Connected subgraphs of `Passive`/`Analog` pins form **clusters**, extracted
//! at build time and solved by quasi-static modified nodal analysis (MNA):
//! Thevenin sources + resistors + piecewise-linear elements → node voltages,
//! recomputed only when a boundary input changes.
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
//! solved block contains at least one Norton shunt conductance, or one edge to
//! a terminal constant, on its diagonal, making it strictly diagonally
//! dominant on the sourced rows and nonsingular. The solver never invents a
//! voltage and never panics on singular input.
//!
//! # Terminals are constants (the Dirichlet revision)
//!
//! A **declared terminal** — a rail, a harness supply, a `net_stuck` fault
//! — handed over as a [`ClusterTerminal`] is not an unknown of the solve:
//! its supernode is eliminated, and every edge or element that touches it
//! stamps the terminal's voltage onto the right-hand side of the node on
//! the other side. This is Dirichlet elimination, which this module once
//! rejected for *all* ideal sources because two disagreeing ideal sources
//! on one node would leave no consistent constraint. The revision
//! (`NODES.md` "Three rules the taxonomy rests on", 1: "its voltage enters
//! the solve as a constant") keeps that reason intact: a supernode held by
//! **exactly one terminal voltage** is a constant, and a supernode two
//! terminals disagree on — a fight the resolver reports as contention —
//! keeps the Norton stamping at the [`IDEAL_SOURCE_FLOOR_OHMS`] floor, so
//! the fight still solves to its divided mid-value until phase 4 decides it
//! once at the terminal's own cluster. What the constant buys: an LED chain
//! (driver → 220 Ω → anode → LED → ground) is a two-unknown solve, and the
//! ground's voltage is exact rather than a microvolt from it.
//!
//! **Which terminals arrive as constants is the resolver's decision, and
//! today it is: those of a cluster that carries an element.** A linear
//! cluster's terminals are still handed over as the 0 Ω
//! [`ClusterSource`]s they always were. The `net_stuck_shared_node` golden
//! trace records every pass of a stuck node's cluster re-publishing the
//! node a few nanovolts from its constant — the Norton floor's drop under
//! each new drive, quantized to 0 µV in the trace but a fresh state each
//! time — and holding that node exactly at its constant deletes those
//! records (17 against 65). Phase 3's gate is that the goldens pass without
//! re-blessing, so the constant is confined to the clusters phase 3 adds,
//! and the linear clusters change hands in phase 4, which makes every
//! terminal its own cluster and reviews the two analog goldens' diffs.
//!
//! # Piecewise-linear elements
//!
//! A [`ClusterElement`] is a branch between two supernodes with a region
//! per linear stamp ([`crate::PwlCurve`], [`Region`]): **off** is the
//! leakage conductance `1 / `[`GMIN_OHMS`] — never an open, so the far side
//! of an off element stays reachable and the region test always has an
//! operand — and **on** is the curve's linear stamp (a `V_f` source in
//! series with `r_d`; `r_on`; a regulator's `i_reg` as a current source
//! beside its leakage; a transistor's saturated `r_sat`). Two curves add
//! one thing each. A **regulator**'s off region is not a leakage but the
//! ohmic segment from the origin to its knee, `v_reg / i_reg`, so the loop
//! it sits in conducts from the cold start and the knee test has a current
//! to judge. A **transistor**'s collector has a third region, **active**: a
//! current source of `hfe` times the base current, stamped as a
//! transconductance on the base–emitter diode's own stamp (a
//! current-controlled current source, which makes the matrix
//! unsymmetric and nothing else), entered from saturation when the load
//! draws more than the base supports and left for it again when the
//! source would pull the collector under the saturation line — so an
//! under-driven base is a sagging collector, never a clean switch.
//!
//! Regions are chosen by a **cold-started, ordered, bounded flip loop**
//! (`NODES.md` §7): every element starts off, the cluster solves, the
//! elements' region tests are evaluated in declaration order, the *first*
//! element whose test disagrees with its region moves to the region the
//! test names, and the cluster solves again — at most
//! [`PWL_SOLVES_PER_ELEMENT`] × N solves. A pass is therefore a function of
//! the inputs alone (no warm start, so two histories reaching one drive
//! table publish identical states), and on exhaustion the solution is
//! marked [`ClusterSolution::converged`] `= false` with every non-terminal
//! node [`NetState::Floating`] — never `NaN`, never a "last consistent
//! solution". A node that only leakage reaches — one no source or terminal
//! feeds over the resistive edges and the conducting stamps, whatever the
//! count of gigaohms into it — is [`NetState::Floating`] too: the far side
//! of an off diode is not sourced by its gigaohm. The probe is structural
//! (a reach over the final regions, `O(m + edges)`), never an elimination.
//!
//! An element's pin on a **declared terminal** — a gate tied to ground, an
//! LED cathode on it, a polarity FET's drain on the input rail — does not
//! make that terminal a member of the element's cluster: the resolver
//! never unions an element through a terminal (`engine.rs`,
//! `build_topology`; a terminal's voltage is fixed for the life of the
//! system until phase 4's fan-out), and hands the terminal over instead as
//! a *foreign* node of the solve — appended to the cluster's node list
//! with its constant in [`ClusterInputs::terminals`] — so the branch
//! stamps against the constant and the region test reads it. A control on
//! a terminal outside the node list reads the constant of the same id
//! ([`ClusterInputs::terminals`] may name a node the cluster does not
//! hold). An element whose every pin is a terminal sits between constants,
//! changes nothing, and is in no solve.
//!
//! Numerical policy (each choice documented at its constant/field):
//! - **Zero-ohm edges** are a hard merge (supernode via union-find), never a
//!   `1/0` conductance.
//! - **Ideal sources** (0 Ω impedance) are Norton-converted through the
//!   [`IDEAL_SOURCE_FLOOR_OHMS`] clamp rather than node elimination; an
//!   on-element's `r_d`/`r_on`/`r_sat` of 0 Ω takes the same floor.
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

use crate::component::{PwlCurve, RegionTest};
use crate::net::{Amps, NetId, NetState, Ohms, Volts};
use std::collections::HashMap;

// ============================================================
// Solver constants
// ============================================================

/// Impedance floor applied to Thevenin sources during Norton conversion.
///
/// An ideal source (declared impedance 0 Ω) has no finite Norton equivalent,
/// so the solver clamps every source impedance up to this floor (1 µΩ)
/// instead of eliminating the node. Dirichlet elimination is reserved for
/// declared terminals (see the module docs): two disagreeing ideal sources
/// on one supernode would leave no consistent constraint, and with a finite
/// floor they resolve to the divided mid-value, with the fight remaining the
/// resolver's projection job. The floor keeps solved voltages within a
/// microvolt of ideal for any realistic cluster load (1 A of load current
/// drops 1 µV) while keeping the conductance matrix finite and well-pivoted
/// at cluster sizes. An on-element with a 0 Ω segment takes the same floor.
pub const IDEAL_SOURCE_FLOOR_OHMS: Ohms = 1e-6;

/// The leakage resistance of an off piecewise-linear element: 1 GΩ.
///
/// An off diode or channel is stamped as this conductance rather than as an
/// open circuit, for two reasons the plan states (`NODES.md` §2, the Diode /
/// LED row): an open would leave the far side of the element MNA-singular —
/// unreachable, and so [`NetState::Floating`] with no voltage — and the
/// element's own region test would then have no operand to evaluate.
/// Through a gigaohm the far side has a voltage (the near side's, less
/// nothing) and the test can say whether the element should conduct. A node
/// that *only* such leakage reaches still publishes Floating: a gigaohm is
/// what keeps the matrix nonsingular, not a source, and a node no source
/// feeds over a resistor or a conducting stamp is fed by leakage alone,
/// however many leakages meet there (a resistor of a gigaohm is a
/// resistor, and feeds).
pub const GMIN_OHMS: Ohms = 1e9;

/// The flip loop's bound, as solves per element: a cluster with N elements
/// runs at most `2N` linear solves before it is [`Finding::NonConvergent`]
/// (`NODES.md` §7, "bound 2N").
///
/// [`Finding::NonConvergent`]: crate::Finding::NonConvergent
pub const PWL_SOLVES_PER_ELEMENT: usize = 2;

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

/// A declared terminal holding a cluster node at a constant: a rail, a
/// harness supply, a `net_stuck` fault. Enters the solve as a Dirichlet
/// constant (see the module docs) — the node is not an unknown, and its
/// state is [`NetState::Analog`] at `volts` whatever else happens in the
/// cluster. A non-finite `volts` disqualifies the terminal.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ClusterTerminal {
    /// The node the terminal holds.
    pub node: NetId,
    /// The voltage it holds it at.
    pub volts: Volts,
}

/// The region a piecewise-linear element is stamped in — one linear stamp
/// each ([`crate::PwlCurve`]). Every element cold-starts in
/// [`Region::Off`]; the flip loop moves it to the region its test names.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Region {
    /// The cold-start region: the leakage conductance [`GMIN_OHMS`] for a
    /// diode, a channel and a transistor's collector; the ohmic segment
    /// below the knee for a regulator.
    Off,
    /// Conducting: the knee source and `r_d` of a diode, `r_on` of a
    /// channel, the regulation current of a regulator, the saturated
    /// `r_sat` of a transistor's collector. An LED in this region is lit.
    On,
    /// A transistor's collector in its active region: a current source of
    /// `hfe` times the base current — the under-driven switch whose
    /// collector sags instead of closing. Only a [`PwlCurve::Bjt`] reaches
    /// it.
    Active,
}

impl Region {
    /// Whether the element conducts by its curve rather than its leakage —
    /// [`Region::On`] or [`Region::Active`]. A regulator's ohmic segment is
    /// [`Region::Off`] although it conducts.
    pub fn is_on(self) -> bool {
        self != Region::Off
    }
}

/// One piecewise-linear element in a cluster: a branch from `a` to `b`
/// ([`crate::Branch`], with its pins resolved to nodes). The branch current
/// is reported positive from `a` to `b`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ClusterElement {
    /// The anode / drain / collector node.
    pub a: NetId,
    /// The cathode / source / emitter node — the reference of the control
    /// test.
    pub b: NetId,
    /// The curve.
    pub curve: PwlCurve,
    /// The control node and its test on `V(control) − V(b)`, for a
    /// [`PwlCurve::Channel`] or a [`PwlCurve::Bjt`]. A channel with no
    /// control is always off. A control node outside the cluster reads the
    /// constant a [`ClusterTerminal`] of the same id carries, and is off
    /// without one.
    pub control: Option<(NetId, RegionTest)>,
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

/// A current injection into a cluster node ([`crate::Drive::Current`]): a
/// Norton source with **no shunt**, stamped as `rhs += amps` at its node and
/// nowhere else. It contributes no reachability — a node only current
/// sources touch has no open-circuit voltage, is MNA-singular exactly like an
/// unsourced one, and solves to [`NetState::Floating`] with the injection
/// dropped. Positive `amps` flow *into* the node (KCL: `G · V = I` with `I`
/// the injected current), so 1 mA into a node 1 kΩ above a 0 V terminal
/// raises it to 1 V.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ClusterInjection {
    /// Node the current is injected into.
    pub node: NetId,
    /// Current into the node, amperes. Non-finite values are dropped.
    pub amps: Amps,
}

/// Boundary inputs to a cluster solve — the values that change between
/// recomputations (drives, rail states, transducer primitive values) and the
/// elements whose regions the solve chooses. **No regions**: every solve
/// cold-starts with every element off (`NODES.md` §7).
#[derive(Debug, Clone, PartialEq, Default)]
pub struct ClusterInputs {
    /// Thevenin sources currently reaching the cluster.
    pub sources: Vec<ClusterSource>,
    /// Current injections into cluster nodes.
    pub injections: Vec<ClusterInjection>,
    /// Declared terminals holding cluster nodes at constants. A terminal
    /// naming a node **outside** the cluster holds nothing: it is the
    /// constant an element's control pin on a declared terminal reads
    /// (the module docs, "a control pin on a declared terminal").
    pub terminals: Vec<ClusterTerminal>,
    /// Piecewise-linear elements, in declaration order — the order the flip
    /// loop evaluates their region tests in.
    pub elements: Vec<ClusterElement>,
}

/// Result of one cluster solve: a state per member node, parallel to
/// [`Cluster::nodes`], and what the solve decided about the elements.
#[derive(Debug, Clone, PartialEq)]
pub struct ClusterSolution {
    /// `(node, solved state)` for every member node.
    pub node_states: Vec<(NetId, NetState)>,
    /// The region of every element, parallel to [`ClusterInputs::elements`].
    /// All [`Region::Off`] when the solve did not converge.
    pub regions: Vec<Region>,
    /// The current through every element from `a` to `b`, parallel to
    /// [`ClusterInputs::elements`]; `None` when the solve did not converge
    /// or the element names a node outside the cluster.
    pub branch_currents: Vec<Option<Amps>>,
    /// Whether the flip loop found a consistent set of regions within its
    /// bound. Always `true` for a cluster without elements.
    pub converged: bool,
    /// Linear solves the flip loop ran: one for a cluster without elements,
    /// at most [`PWL_SOLVES_PER_ELEMENT`] × N with N elements.
    pub solves: usize,
    /// The size `m` of the matrix built: the source-reachable supernodes
    /// that are not terminal constants.
    pub unknowns: usize,
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
///    [`IDEAL_SOURCE_FLOOR_OHMS`]); hold each supernode exactly one
///    terminal voltage names as a constant, and Norton-stamp the terminals
///    of a supernode two disagree on;
/// 3. find the source-reachable supernodes — over the resistive edges and
///    the elements, whose off region still conducts — the complement is
///    exactly the MNA-singular set and reports [`NetState::Floating`];
/// 4. with every element off, stamp conductances, Norton injections, bare
///    current injections ([`ClusterInjection`], right-hand side only) and
///    the elements' current regions into `G · V = I` over the reachable
///    non-terminal supernodes and solve by Gaussian elimination with
///    partial pivoting; evaluate the elements' region tests in order and
///    flip the first that disagrees; repeat within the bound;
/// 5. map supernode voltages back to every member node as
///    [`NetState::Analog`] — [`NetState::Floating`] for a node only leakage
///    reaches, and for every non-terminal node when the loop did not
///    converge.
#[derive(Debug, Clone, Copy, Default)]
pub struct QuasiStaticMna;

/// Where an element's control voltage comes from: a supernode of the
/// cluster, or the constant of a declared terminal outside it.
#[derive(Debug, Clone, Copy)]
enum ControlNode {
    Local(usize),
    Constant(Volts),
}

/// One element with its nodes resolved to cluster-local supernodes.
#[derive(Debug, Clone, Copy)]
struct ResolvedElement {
    a: usize,
    b: usize,
    curve: PwlCurve,
    control: Option<(ControlNode, RegionTest)>,
    /// For a [`PwlCurve::Bjt`]: the index of its base–emitter diode — the
    /// first [`PwlCurve::Diode`] element from the control node to `b`.
    base: Option<usize>,
}

/// The current-controlled part of a transistor's active-region stamp: the
/// collector current `k · (V_bn − V_en) − i0`, `hfe` times the base diode's
/// own stamp (`k = hfe · g_be`, `i0 = hfe · i_be`), flowing from the
/// collector to the emitter.
#[derive(Debug, Clone, Copy)]
struct Gain {
    bn: usize,
    en: usize,
    k: f64,
    i0: f64,
}

/// One element's stamp in the current region: a conductance `g` between its
/// supernodes and a Norton current `i` flowing from `a` to `b` through the
/// element (the on-diode's `V_f · g`, a regulating regulator's `−i_reg`),
/// so the branch current is `g · (V_a − V_b) − i`, plus the gain term for
/// an active transistor.
#[derive(Debug, Clone, Copy)]
struct ElementStamp {
    a: usize,
    b: usize,
    g: f64,
    i: f64,
    gain: Option<Gain>,
}

impl ElementStamp {
    /// The current from `a` to `b` at the given supernode voltages.
    fn current(&self, voltages: &[Option<Volts>]) -> Option<Amps> {
        let (va, vb) = (voltages[self.a]?, voltages[self.b]?);
        let mut amps = self.g * (va - vb) - self.i;
        if let Some(gain) = self.gain {
            let (vbn, ven) = (voltages[gain.bn]?, voltages[gain.en]?);
            amps += gain.k * (vbn - ven) - gain.i0;
        }
        Some(amps)
    }
}

impl ClusterSolver for QuasiStaticMna {
    fn solve(&self, cluster: &Cluster, inputs: &ClusterInputs) -> ClusterSolution {
        let n = cluster.nodes.len();

        // Cluster-local dense index per node (first occurrence wins).
        //
        // hash-order: `node_index` is keyed access only (`get`, `entry`, index)
        // and is never iterated. Everything downstream — `dsu`, `edges`,
        // `sources`, `reachable`, `compact`, and the returned `node_states` —
        // is built by walking `cluster.nodes` / `cluster.resistors` /
        // `inputs.sources` / `inputs.terminals` / `inputs.elements`, all
        // `Vec`s, so the whole solve is a pure function of those slices *in
        // their given order*. That last clause is why the resolver must hand
        // `inputs.sources` over in a canonical order: the `matrix[c][c] += g`
        // / `rhs[c] += i` accumulation below is where source order becomes
        // float rounding (`DETERMINISM.md`, "One real hash-order defect").
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

        // Terminals: a supernode exactly one voltage holds is a Dirichlet
        // constant; a supernode two terminals disagree on keeps the Norton
        // fight at the floor (the module docs say why). Invalid terminals
        // (non-finite volts, node outside the cluster) contribute nothing.
        //
        // A cluster handed no terminal — every linear cluster today — pays
        // nothing here: the tables stay empty and `constant` reads `None`
        // for every supernode (`DESIGN.md` rule 8).
        let mut terminal_roots: Vec<(usize, Volts)> = Vec::new();
        let mut fought: Vec<bool> = Vec::new();
        let mut dirichlet: Vec<Option<Volts>> = Vec::new();
        if !inputs.terminals.is_empty() {
            let mut held: Vec<Option<Volts>> = vec![None; n];
            fought = vec![false; n];
            for t in &inputs.terminals {
                let Some(&node) = node_index.get(&t.node) else {
                    continue;
                };
                if !t.volts.is_finite() {
                    continue;
                }
                let root = root_of[node];
                terminal_roots.push((root, t.volts));
                match held[root] {
                    None => held[root] = Some(t.volts),
                    // `==`, so 0.0 and -0.0 are one voltage, not a fight
                    // (both are finite here; NaN never reaches this arm).
                    Some(v) if v == t.volts => {}
                    Some(_) => fought[root] = true,
                }
            }
            dirichlet = (0..n)
                .map(|root| if fought[root] { None } else { held[root] })
                .collect();
        }
        let constant = |root: usize| -> Option<Volts> { dirichlet.get(root).copied().flatten() };

        // Norton conversion of the valid Thevenin sources:
        // (V, Z) → current injection V/Z with shunt conductance 1/Z, with Z
        // clamped to IDEAL_SOURCE_FLOOR_OHMS so ideal sources stay finite.
        // Invalid sources (non-finite volts/impedance, negative impedance,
        // node outside the cluster) contribute nothing. The terminals of a
        // fought supernode are stamped here too, as the ideal sources they
        // are, after the slot sources and in their own order.
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
        for &(root, volts) in &terminal_roots {
            if fought[root] {
                let g = 1.0 / IDEAL_SOURCE_FLOOR_OHMS;
                sources.push((root, g, volts * g));
            }
        }

        // Elements, resolved to supernodes; one naming a node outside the
        // cluster is dropped (defensive) but keeps its slot in the outputs.
        // A control node outside the cluster reads the constant of the
        // terminal of the same id, if one was handed over, and is off
        // otherwise (the module docs, "a control pin on a declared
        // terminal").
        let elements: Vec<Option<ResolvedElement>> = inputs
            .elements
            .iter()
            .map(|e| {
                let (&a, &b) = (node_index.get(&e.a)?, node_index.get(&e.b)?);
                let control = e.control.map(|(node, test)| {
                    let control = match node_index.get(&node) {
                        Some(&c) => ControlNode::Local(root_of[c]),
                        None => ControlNode::Constant(
                            inputs
                                .terminals
                                .iter()
                                .find(|t| t.node == node && t.volts.is_finite())
                                .map_or(f64::NAN, |t| t.volts),
                        ),
                    };
                    (control, test)
                });
                Some(ResolvedElement {
                    a: root_of[a],
                    b: root_of[b],
                    curve: e.curve,
                    control,
                    base: None,
                })
            })
            .collect();
        // A transistor's base diode: the first diode from its control
        // node to its emitter, by declaration order.
        let elements: Vec<Option<ResolvedElement>> = elements
            .iter()
            .map(|element| {
                let mut element = (*element)?;
                if let (PwlCurve::Bjt { .. }, Some((ControlNode::Local(base), _))) =
                    (element.curve, element.control)
                {
                    element.base = elements.iter().position(|candidate| {
                        candidate.is_some_and(|c| {
                            matches!(c.curve, PwlCurve::Diode { .. })
                                && c.a == base
                                && c.b == element.b
                        })
                    });
                }
                Some(element)
            })
            .collect();
        let element_count = elements.iter().flatten().count();

        // Source reachability over the supernode graph — the resistive edges
        // and the elements, which conduct in every region: the unreachable
        // supernodes are exactly the MNA-singular ones — they solve to
        // Floating and are excluded from the matrix. Terminal constants are
        // sources of reachability like any other.
        let mut adjacency: Vec<Vec<usize>> = vec![Vec::new(); n];
        for &(a, b, _) in &edges {
            adjacency[a].push(b);
            adjacency[b].push(a);
        }
        for element in elements.iter().flatten() {
            if element.a != element.b {
                adjacency[element.a].push(element.b);
                adjacency[element.b].push(element.a);
            }
        }
        let seeds = sources
            .iter()
            .map(|&(root, _, _)| root)
            .chain(terminal_roots.iter().map(|&(root, _)| root));
        let reachable = reach(&adjacency, seeds);

        // Compact matrix index over the reachable, non-constant supernodes.
        let mut compact: Vec<Option<usize>> = vec![None; n];
        let mut m = 0usize;
        for root in 0..n {
            if reachable[root] && constant(root).is_none() {
                compact[root] = Some(m);
                m += 1;
            }
        }

        // Assemble G · V = I over the unknowns. Resistor stamp: +g on both
        // diagonals, −g on both off-diagonals; against a constant, +g on
        // the near diagonal and +g·V_T on the near right-hand side. Norton
        // source stamp: +G on the diagonal, +I on the right-hand side. A
        // source or injection on a constant is absorbed by the terminal
        // holding it. One stamping for both drivers below: the one-shot
        // linear solve, and the flip loop, which adds the elements' stamps
        // of the current regions on top.
        let stamp_conductance =
            |a: usize, b: usize, g: f64, matrix: &mut [Vec<f64>], rhs: &mut [f64]| {
                match (compact[a], compact[b]) {
                    (Some(ca), Some(cb)) => {
                        matrix[ca][ca] += g;
                        matrix[cb][cb] += g;
                        matrix[ca][cb] -= g;
                        matrix[cb][ca] -= g;
                    }
                    (Some(ca), None) => {
                        if let Some(vb) = constant(b) {
                            matrix[ca][ca] += g;
                            rhs[ca] += g * vb;
                        }
                    }
                    (None, Some(cb)) => {
                        if let Some(va) = constant(a) {
                            matrix[cb][cb] += g;
                            rhs[cb] += g * va;
                        }
                    }
                    (None, None) => {} // both constant, or unreachable
                }
            };
        let assemble_linear = || -> (Vec<Vec<f64>>, Vec<f64>) {
            let mut matrix = vec![vec![0.0f64; m]; m];
            let mut rhs = vec![0.0f64; m];
            for &(a, b, g) in &edges {
                stamp_conductance(a, b, g, &mut matrix, &mut rhs);
            }
            for &(root, g, i) in &sources {
                if let Some(c) = compact[root] {
                    matrix[c][c] += g;
                    rhs[c] += i;
                }
            }
            // Bare current injections: right-hand side only, no shunt, no
            // reachability. An injection into an unreachable supernode has no
            // return path and is dropped — the node stays Floating rather than
            // acquiring an invented voltage. Stamped after the sources, in
            // declaration order (the accumulation order is part of the deck).
            for injection in &inputs.injections {
                let Some(&node) = node_index.get(&injection.node) else {
                    continue;
                };
                if !injection.amps.is_finite() {
                    continue;
                }
                if let Some(c) = compact[root_of[node]] {
                    rhs[c] += injection.amps;
                }
            }
            (matrix, rhs)
        };
        // Map supernode voltages back per member node; unreachable nodes,
        // leakage-only nodes, every non-constant node of a solve that did
        // not converge, and the defensive singular-solve fallback report
        // Floating — a voltage is never invented. A terminal constant is
        // its constant regardless.
        let states_of = |solved: &Option<Vec<Volts>>,
                         converged: bool,
                         leakage_only: &[bool]|
         -> Vec<(NetId, NetState)> {
            cluster
                .nodes
                .iter()
                .map(|&id| {
                    let root = root_of[node_index[&id]];
                    let fed = converged && !leakage_only.get(root).copied().unwrap_or(false);
                    let state = match (constant(root), compact[root], solved) {
                        (Some(v), _, _) => NetState::Analog(v),
                        (None, Some(c), Some(v)) if fed => NetState::Analog(v[c]),
                        _ => NetState::Floating,
                    };
                    (id, state)
                })
                .collect()
        };

        // A cluster without an element: one assembly, one elimination, the
        // states — the shape it had before elements existed, with nothing
        // of the region machinery built or walked (`DESIGN.md` rule 8).
        if element_count == 0 {
            let (matrix, rhs) = assemble_linear();
            let solved = solve_dense(matrix, rhs);
            return ClusterSolution {
                node_states: states_of(&solved, true, &[]),
                regions: Vec::new(),
                branch_currents: Vec::new(),
                converged: true,
                solves: 1,
                unknowns: m,
            };
        }

        // The flip loop: every element off, solve, move the first element
        // whose region test disagrees to the region it names, solve again —
        // bounded.
        let bound = PWL_SOLVES_PER_ELEMENT * element_count;
        let mut regions = vec![Region::Off; elements.len()];
        let mut solves = 0usize;
        let mut converged = false;
        // Supernode voltages between solves — the region tests' and the
        // branch currents' operands.
        let mut voltages: Vec<Option<Volts>> = vec![None; n];
        // The last solve's unknowns and element stamps, for the states,
        // the leakage probe and the branch currents.
        let solved: Option<Vec<Volts>>;
        let last_stamps: Vec<Option<ElementStamp>>;
        loop {
            let (mut matrix, mut rhs) = assemble_linear();
            // The elements in their current regions: a conductance between
            // their supernodes, and a Norton current `i` from `a` to `b` —
            // `+i` on `a`'s right-hand side, `−i` on `b`'s (the branch
            // carries `g·(V_a − V_b) − i`): the on-diode's knee `V_f · g`,
            // the regulating regulator's `−i_reg`. The active transistor's
            // collector adds a transconductance on its base diode's stamp.
            let stamps: Vec<Option<ElementStamp>> = elements
                .iter()
                .zip(&regions)
                .map(|(element, &region)| {
                    let element = (*element)?;
                    let leakage = (1.0 / GMIN_OHMS, 0.0);
                    let (g, i) = match (element.curve, region) {
                        (PwlCurve::Regulator { i_reg, v_reg }, Region::Off) => {
                            (i_reg / v_reg.max(IDEAL_SOURCE_FLOOR_OHMS), 0.0)
                        }
                        (_, Region::Off) => leakage,
                        (PwlCurve::Diode { vf, r_d }, _) => {
                            let g = 1.0 / r_d.max(IDEAL_SOURCE_FLOOR_OHMS);
                            (g, vf * g)
                        }
                        (PwlCurve::Channel { r_on }, _) => {
                            (1.0 / r_on.max(IDEAL_SOURCE_FLOOR_OHMS), 0.0)
                        }
                        (PwlCurve::Regulator { i_reg, .. }, _) => (1.0 / GMIN_OHMS, -i_reg),
                        (PwlCurve::Bjt { r_sat, .. }, Region::On) => {
                            (1.0 / r_sat.max(IDEAL_SOURCE_FLOOR_OHMS), 0.0)
                        }
                        (PwlCurve::Bjt { .. }, Region::Active) => leakage,
                    };
                    Some(ElementStamp {
                        a: element.a,
                        b: element.b,
                        g,
                        i,
                        gain: None,
                    })
                })
                .collect();
            // The gain of every active transistor, from its base diode's
            // stamp of this same solve.
            let stamps: Vec<Option<ElementStamp>> = stamps
                .iter()
                .zip(&elements)
                .zip(&regions)
                .map(|((stamp, element), &region)| {
                    let mut stamp = (*stamp)?;
                    let element = (*element)?;
                    if let (PwlCurve::Bjt { hfe, .. }, Region::Active, Some(base)) =
                        (element.curve, region, element.base)
                    {
                        let diode = stamps[base]?;
                        stamp.gain = Some(Gain {
                            bn: diode.a,
                            en: diode.b,
                            k: hfe * diode.g,
                            i0: hfe * diode.i,
                        });
                    }
                    Some(stamp)
                })
                .collect();
            for stamp in stamps.iter().flatten() {
                if stamp.a == stamp.b {
                    continue; // both ends on one supernode: no branch
                }
                stamp_conductance(stamp.a, stamp.b, stamp.g, &mut matrix, &mut rhs);
                if let Some(ca) = compact[stamp.a] {
                    rhs[ca] += stamp.i;
                }
                if let Some(cb) = compact[stamp.b] {
                    rhs[cb] -= stamp.i;
                }
                // The controlled source: `k · (V_bn − V_en) − i0` leaves the
                // collector and enters the emitter. Each row takes its
                // unknown factors on the left and its constants — a
                // Dirichlet base or emitter, and `i0` — on the right.
                let Some(gain) = stamp.gain else {
                    continue;
                };
                for (row, sign) in [(stamp.a, 1.0), (stamp.b, -1.0)] {
                    let Some(r) = compact[row] else {
                        continue;
                    };
                    match (compact[gain.bn], constant(gain.bn)) {
                        (Some(c), _) => matrix[r][c] += sign * gain.k,
                        (None, Some(v)) => rhs[r] -= sign * gain.k * v,
                        (None, None) => {}
                    }
                    match (compact[gain.en], constant(gain.en)) {
                        (Some(c), _) => matrix[r][c] -= sign * gain.k,
                        (None, Some(v)) => rhs[r] += sign * gain.k * v,
                        (None, None) => {}
                    }
                    rhs[r] += sign * gain.i0;
                }
            }

            // The matrix is consumed: nothing after the elimination reads
            // it (the leakage probe is structural).
            let this = solve_dense(matrix, rhs);
            solves += 1;
            for root in 0..n {
                voltages[root] = match (constant(root), compact[root], &this) {
                    (Some(v), _, _) => Some(v),
                    (None, Some(c), Some(v)) => Some(v[c]),
                    _ => None,
                };
            }

            // Region tests, in declaration order: the region every
            // element's test names at these voltages. The first element
            // whose named region differs from its current one moves to it,
            // and only it. A test on a node with no voltage (an unreached
            // control terminal) evaluates as off.
            let control_minus_b = |element: &ResolvedElement| -> Option<Volts> {
                let (control, _) = element.control?;
                let vc = match control {
                    ControlNode::Local(c) => voltages[c]?,
                    ControlNode::Constant(v) => v,
                };
                (vc.is_finite()).then_some(vc - voltages[element.b]?)
            };
            let wanted: Vec<Region> = elements
                .iter()
                .zip(&regions)
                .map(|(element, &region)| {
                    let Some(element) = *element else {
                        return region;
                    };
                    let (a, b) = (element.a, element.b);
                    let controlled = || {
                        element.control.is_some_and(|(_, test)| {
                            control_minus_b(&element).is_some_and(|v| test.passes(v))
                        })
                    };
                    let on = |conducts: bool| if conducts { Region::On } else { Region::Off };
                    match element.curve {
                        PwlCurve::Diode { vf, .. } => on(match (voltages[a], voltages[b]) {
                            (Some(va), Some(vb)) => va - vb >= vf,
                            _ => false,
                        }),
                        PwlCurve::Regulator { v_reg, .. } => on(match (voltages[a], voltages[b]) {
                            (Some(va), Some(vb)) => va - vb >= v_reg,
                            _ => false,
                        }),
                        PwlCurve::Channel { .. } => on(controlled()),
                        PwlCurve::Bjt { hfe, r_sat } => {
                            // The base current the collector is judged
                            // against: the base diode's own current.
                            let i_b = element
                                .base
                                .and_then(|j| stamps[j])
                                .and_then(|diode| diode.current(&voltages))
                                .unwrap_or(0.0);
                            match (controlled(), voltages[a], voltages[b], region) {
                                (false, ..) => Region::Off,
                                (true, _, _, Region::Off) => Region::On,
                                (true, Some(va), Some(vb), Region::On) => {
                                    // Saturated: the collector current the
                                    // load pushes through r_sat, against the
                                    // most the base supports.
                                    let i_c = (va - vb) / r_sat.max(IDEAL_SOURCE_FLOOR_OHMS);
                                    if i_c > hfe * i_b {
                                        Region::Active
                                    } else {
                                        Region::On
                                    }
                                }
                                (true, Some(va), Some(vb), Region::Active) => {
                                    // Active: the source would pull the
                                    // collector under the saturation line.
                                    let i_c = hfe * i_b;
                                    if va - vb < i_c * r_sat {
                                        Region::On
                                    } else {
                                        Region::Active
                                    }
                                }
                                (true, _, _, region) => region,
                            }
                        }
                    }
                })
                .collect();
            let disagreeing = wanted
                .iter()
                .zip(&regions)
                .position(|(wants, region)| wants != region);
            match disagreeing {
                None => {
                    converged = true;
                    solved = this;
                    last_stamps = stamps;
                    break;
                }
                Some(_) if solves >= bound => {
                    solved = this;
                    last_stamps = stamps;
                    break;
                }
                Some(index) => regions[index] = wanted[index],
            }
        }

        // A node only leakage reaches floats. Structural: from the sources
        // and terminals, over the resistive edges and the stamps that are
        // more than the bare leakage — a conducting region, a regulator's
        // ohmic segment, a current source — the supernodes not reached are
        // fed by gigaohms alone, however many. O(m + edges), never an
        // elimination; only where an element could be the only path (the
        // linear solve returned above, before it).
        let mut leakage_only: Vec<bool> = Vec::new();
        if converged {
            let mut conducting: Vec<Vec<usize>> = vec![Vec::new(); n];
            for &(a, b, _) in &edges {
                conducting[a].push(b);
                conducting[b].push(a);
            }
            for stamp in last_stamps.iter().flatten() {
                let more_than_leakage =
                    stamp.g > 1.0 / GMIN_OHMS || stamp.i != 0.0 || stamp.gain.is_some();
                if more_than_leakage && stamp.a != stamp.b {
                    conducting[stamp.a].push(stamp.b);
                    conducting[stamp.b].push(stamp.a);
                }
            }
            let seeds = sources
                .iter()
                .map(|&(root, _, _)| root)
                .chain(terminal_roots.iter().map(|&(root, _)| root));
            let fed = reach(&conducting, seeds);
            leakage_only = (0..n).map(|root| reachable[root] && !fed[root]).collect();
        }
        let leakage_only_at = |root: usize| leakage_only.get(root).copied().unwrap_or(false);

        // Branch currents from the final regions and voltages; none when
        // the loop did not converge — there is no operating point to take
        // them from.
        let branch_currents: Vec<Option<Amps>> = last_stamps
            .iter()
            .map(|stamp| {
                if !converged {
                    return None;
                }
                let stamp = (*stamp)?;
                match (voltages[stamp.a], voltages[stamp.b]) {
                    (Some(_), Some(_))
                        if !leakage_only_at(stamp.a) && !leakage_only_at(stamp.b) =>
                    {
                        // The stamp's own current, the gain term of an
                        // active transistor included.
                        stamp.current(&voltages)
                    }
                    (Some(_), Some(_)) => Some(0.0),
                    _ => None,
                }
            })
            .collect();
        if !converged {
            regions.iter_mut().for_each(|r| *r = Region::Off);
        }

        ClusterSolution {
            node_states: states_of(&solved, converged, &leakage_only),
            regions,
            branch_currents,
            converged,
            solves,
            unknowns: m,
        }
    }
}

/// The supernodes reached from `seeds` over `adjacency` — a depth-first
/// walk over dense indices, so the visit order is a function of the input
/// order alone.
fn reach(adjacency: &[Vec<usize>], seeds: impl Iterator<Item = usize>) -> Vec<bool> {
    let mut reached = vec![false; adjacency.len()];
    let mut stack: Vec<usize> = Vec::new();
    for root in seeds {
        if !reached[root] {
            reached[root] = true;
            stack.push(root);
        }
    }
    while let Some(x) = stack.pop() {
        for &y in &adjacency[x] {
            if !reached[y] {
                reached[y] = true;
                stack.push(y);
            }
        }
    }
    reached
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
/// every assembled block carries at least one Norton shunt conductance, or
/// one conductance to a terminal constant, on its diagonal, which keeps the
/// block nonsingular.
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
    use vibes_behaviour::{behaviour, expect, Test};

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

    fn resistor(a: usize, b: usize, ohms: f64) -> ClusterResistor {
        ClusterResistor {
            a: NetId(a),
            b: NetId(b),
            ohms,
        }
    }

    fn terminal(node: usize, volts: f64) -> ClusterTerminal {
        ClusterTerminal {
            node: NetId(node),
            volts,
        }
    }

    fn diode(a: usize, b: usize, vf: f64, r_d: f64) -> ClusterElement {
        ClusterElement {
            a: NetId(a),
            b: NetId(b),
            curve: PwlCurve::Diode { vf, r_d },
            control: None,
        }
    }

    fn channel(a: usize, b: usize, control: usize, r_on: f64, test: RegionTest) -> ClusterElement {
        ClusterElement {
            a: NetId(a),
            b: NetId(b),
            curve: PwlCurve::Channel { r_on },
            control: Some((NetId(control), test)),
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
        assert!(solution.converged);
        assert_eq!(solution.solves, 1);
        assert_eq!(solution.unknowns, 0);
    }

    #[rstest]
    fn sourced_cluster_is_not_floating() {
        let inputs = ClusterInputs {
            sources: vec![ClusterSource {
                node: NetId(0),
                volts: 3.3,
                impedance: 25.0,
            }],
            ..Default::default()
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
            ..Default::default()
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
            ..Default::default()
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
            ..Default::default()
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
            ..Default::default()
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
            ..Default::default()
        };
        let solution = QuasiStaticMna.solve(&cluster, &inputs);
        assert!((analog_volts(&solution, NetId(0)) - 3.3).abs() < 1e-9);
    }

    /// A current injection is a right-hand-side stamp: a node held a
    /// resistor away from a terminal rises by `I · R` above it. The terminal
    /// is ideal (0 Ω, clamped to the 1 µΩ floor), so the drop across the
    /// floor is a nanovolt against a 1 V answer.
    #[rstest]
    #[case::one_ma_into_1k(1e-3, 1_000.0, 1.0)]
    #[case::hundred_ua_into_4k7(100e-6, 4_700.0, 0.47)]
    #[case::sink_ten_ua_from_10k(-10e-6, 10_000.0, -0.1)]
    fn a_current_injection_into_a_resistor_to_a_terminal_reads_i_times_r(
        #[case] amps: f64,
        #[case] ohms: f64,
        #[case] expect: f64,
    ) {
        // 0 V terminal at n0 —R— n1 <- I
        let cluster = Cluster {
            nodes: vec![NetId(0), NetId(1)],
            resistors: vec![ClusterResistor {
                a: NetId(0),
                b: NetId(1),
                ohms,
            }],
        };
        let inputs = ClusterInputs {
            sources: vec![ClusterSource {
                node: NetId(0),
                volts: 0.0,
                impedance: 0.0,
            }],
            injections: vec![ClusterInjection {
                node: NetId(1),
                amps,
            }],
            ..Default::default()
        };
        let solution = QuasiStaticMna.solve(&cluster, &inputs);
        assert!((analog_volts(&solution, NetId(1)) - expect).abs() < 1e-6);
        assert!(analog_volts(&solution, NetId(0)).abs() < 1e-6);
    }

    /// An injection reaches nothing on its own: with no Thevenin source in
    /// the cluster every node is MNA-singular and stays Floating, the
    /// current dropped rather than turned into a voltage; a non-finite
    /// injection is dropped the same way even where a source exists.
    #[rstest]
    fn a_current_injection_alone_leaves_its_node_floating() {
        let inputs = ClusterInputs {
            sources: vec![],
            injections: vec![ClusterInjection {
                node: NetId(1),
                amps: 1e-3,
            }],
            ..Default::default()
        };
        let solution = QuasiStaticMna.solve(&three_node_cluster(), &inputs);
        for (_, state) in &solution.node_states {
            assert_eq!(*state, NetState::Floating);
        }

        let inputs = ClusterInputs {
            sources: vec![ClusterSource {
                node: NetId(0),
                volts: 3.3,
                impedance: 25.0,
            }],
            injections: vec![ClusterInjection {
                node: NetId(1),
                amps: f64::NAN,
            }],
            ..Default::default()
        };
        let solution = QuasiStaticMna.solve(&three_node_cluster(), &inputs);
        for node in [NetId(0), NetId(1), NetId(2)] {
            assert!((analog_volts(&solution, node) - 3.3).abs() < 1e-9);
        }
    }

    // --------------------------------------------------------
    // Terminals as constants
    // --------------------------------------------------------

    /// The LED chain of `NODES.md` §2 rule 1: a driver's node, 220 Ω, the
    /// anode, the LED, and ground as a declared terminal. Ground is a
    /// constant, so the matrix holds two unknowns — the driver's node and
    /// the anode — and the chain solves at m = 2.
    #[rstest]
    fn a_terminal_is_a_constant_and_the_led_chain_solves_with_two_unknowns() {
        behaviour!(Test {
            id: "solver.terminal-is-a-constant",
            covers: Some("board/src/cluster.rs#QuasiStaticMna::solve"),
            given: "a 3.3 volt driver's node joined by 220 ohms to an LED's anode, the LED's \
                    cathode on a ground the harness declares at 0 volts",
        });
        expect!(
            "two-unknowns",
            "the matrix the solve builds has two unknowns, the driver's node and the anode",
            "a declared terminal holds its node at a constant and is eliminated from the \
             unknowns; only the nodes between the terminals are solved for"
        );
        expect!(
            "ground-exact",
            "the ground node reads exactly 0 volts",
            "a terminal's voltage enters the solve as the constant it is, with no source \
             impedance to drop across"
        );
        expect!(
            "anode-at-the-knee",
            "the anode sits at the LED's forward voltage",
            "the LED conducts, and the drop across a conducting diode is its knee"
        );
        // n0 driver's node, n1 anode, n2 ground.
        let cluster = Cluster {
            nodes: vec![NetId(0), NetId(1), NetId(2)],
            resistors: vec![resistor(0, 1, 220.0)],
        };
        let inputs = ClusterInputs {
            sources: vec![ClusterSource {
                node: NetId(0),
                volts: 3.3,
                impedance: 25.0,
            }],
            terminals: vec![terminal(2, 0.0)],
            elements: vec![diode(1, 2, 2.0, 0.0)],
            ..Default::default()
        };
        let solution = QuasiStaticMna.solve(&cluster, &inputs);
        assert_eq!(solution.unknowns, 2);
        assert!(solution.converged);
        assert_eq!(solution.state_of(NetId(2)), Some(NetState::Analog(0.0)));
        assert!((analog_volts(&solution, NetId(1)) - 2.0).abs() < 1e-6);
        // I = (3.3 − 2.0) / (25 + 220) through the driver's 25 Ω.
        let expected = 1.3 / 245.0;
        assert!((solution.branch_currents[0].unwrap() - expected).abs() < expected * 1e-6);
        assert_eq!(solution.regions, vec![Region::On]);
    }

    /// Two terminals that disagree on one node — a rail against a short to
    /// ground — are not a constant: they keep the Norton fight at the floor
    /// and divide to the mid-value, the way two ideal sources always have,
    /// so the resolver can report the fight. Two that agree are one
    /// constant.
    #[rstest]
    fn disagreeing_terminals_on_one_node_keep_the_norton_fight() {
        behaviour!(Test {
            id: "solver.fought-terminal-is-not-a-constant",
            covers: Some("board/src/cluster.rs#QuasiStaticMna::solve"),
            given: "a node declared at 3.3 volts by a rail and at 0 volts by an injected short",
        });
        expect!(
            "mid-value",
            "the node solves to the mid-value between the two",
            "no constant can hold two voltages, so the disagreeing terminals are stamped as \
             the ideal sources they are and divide equally"
        );
        expect!(
            "still-an-unknown",
            "the node stays an unknown of the solve",
            "a fought node is decided by the fight between its terminals"
        );
        let cluster = Cluster {
            nodes: vec![NetId(0)],
            resistors: vec![],
        };
        let inputs = ClusterInputs {
            terminals: vec![terminal(0, 3.3), terminal(0, 0.0)],
            ..Default::default()
        };
        let solution = QuasiStaticMna.solve(&cluster, &inputs);
        assert!((analog_volts(&solution, NetId(0)) - 1.65).abs() < 1e-6);
        assert_eq!(solution.unknowns, 1);

        let inputs = ClusterInputs {
            terminals: vec![terminal(0, 3.3), terminal(0, 3.3)],
            ..Default::default()
        };
        let solution = QuasiStaticMna.solve(&cluster, &inputs);
        assert_eq!(solution.state_of(NetId(0)), Some(NetState::Analog(3.3)));
        assert_eq!(solution.unknowns, 0);
    }

    // --------------------------------------------------------
    // Piecewise-linear elements
    // --------------------------------------------------------

    /// A diode from a 3.3 V terminal through 220 Ω to a 0 V terminal: the
    /// cold start finds it off, the first solve puts 3.3 V across it, the
    /// test flips it on, and the second solve is consistent — two solves,
    /// the anode at the knee, the current set by the resistor.
    #[rstest]
    fn a_forward_biased_diode_turns_on_in_two_solves() {
        behaviour!(Test {
            id: "solver.diode-forward-biased",
            covers: Some("board/src/cluster.rs#QuasiStaticMna::solve"),
            given: "a diode with a 0.75 volt knee whose anode is fed from 3.3 volts through \
                    220 ohms and whose cathode is on 0 volts",
        });
        expect!(
            "anode-at-the-knee",
            "the anode reads the knee voltage",
            "a conducting diode drops its forward voltage and the resistor takes the rest"
        );
        expect!(
            "current-by-the-resistor",
            "the diode carries the supply less the knee, divided by the resistor, within one \
             percent",
        );
        expect!(
            "two-solves",
            "the operating point is found in exactly two solves",
            "every element starts off; one solve shows the diode forward-biased past its \
             knee, the flip turns it on, and the next solve agrees with the test"
        );
        expect!("on", "the diode is reported in its on region");
        let cluster = Cluster {
            nodes: vec![NetId(0), NetId(1), NetId(2)],
            resistors: vec![resistor(0, 1, 220.0)],
        };
        let inputs = ClusterInputs {
            terminals: vec![terminal(0, 3.3), terminal(2, 0.0)],
            elements: vec![diode(1, 2, 0.75, 0.0)],
            ..Default::default()
        };
        let solution = QuasiStaticMna.solve(&cluster, &inputs);
        assert!(solution.converged);
        assert_eq!(solution.solves, 2);
        assert_eq!(solution.regions, vec![Region::On]);
        assert!((analog_volts(&solution, NetId(1)) - 0.75).abs() < 1e-6);
        let expected = (3.3 - 0.75) / 220.0;
        let current = solution.branch_currents[0].expect("a converged branch has a current");
        assert!(
            (current - expected).abs() < expected * 0.01,
            "{current} vs {expected}"
        );
    }

    /// Two diodes in series under one resistor: the first flips on into a
    /// far side only the second's leakage loads — the state whose on-segment
    /// margin is the forward current across the 1 µΩ floor, a few ulp of
    /// the node voltage — and holds through the second's flip; three
    /// solves, both on, one current.
    #[rstest]
    fn two_diodes_in_series_turn_on_one_after_the_other() {
        behaviour!(Test {
            id: "solver.diodes-in-series",
            covers: Some("board/src/cluster.rs#QuasiStaticMna::solve"),
            given: "two diodes with 0.75 volt knees in series, fed from 3.3 volts through 220 \
                    ohms, the far cathode on 0 volts",
        });
        expect!(
            "three-solves",
            "the operating point is found in exactly three solves, the diodes turning on one \
             after the other in declaration order",
            "every element starts off and one flips per solve; the first diode, on into a far \
             side only the second's leakage loads, stays on while the second turns on"
        );
        expect!(
            "both-on",
            "both diodes are on, the middle node one knee above the far cathode and the anode \
             two knees above it",
        );
        expect!(
            "one-current",
            "both carry the supply less two knees, divided by the resistor, within one percent",
        );
        let cluster = Cluster {
            nodes: vec![NetId(0), NetId(1), NetId(2), NetId(3)],
            resistors: vec![resistor(0, 1, 220.0)],
        };
        let inputs = ClusterInputs {
            terminals: vec![terminal(0, 3.3), terminal(3, 0.0)],
            elements: vec![diode(1, 2, 0.75, 0.0), diode(2, 3, 0.75, 0.0)],
            ..Default::default()
        };
        let solution = QuasiStaticMna.solve(&cluster, &inputs);
        assert!(solution.converged);
        assert_eq!(solution.solves, 3);
        assert_eq!(solution.regions, vec![Region::On, Region::On]);
        assert!((analog_volts(&solution, NetId(2)) - 0.75).abs() < 1e-6);
        assert!((analog_volts(&solution, NetId(1)) - 1.5).abs() < 1e-6);
        let expected = (3.3 - 1.5) / 220.0;
        for current in &solution.branch_currents {
            let current = current.expect("a converged branch has a current");
            assert!(
                (current - expected).abs() < expected * 0.01,
                "{current} vs {expected}"
            );
        }
    }

    /// The same diode the other way round blocks: one solve, off, the anode
    /// at its source, a leakage current of nanoamps.
    #[rstest]
    fn a_reverse_biased_diode_stays_off_in_one_solve() {
        behaviour!(Test {
            id: "solver.diode-reverse-biased",
            covers: Some("board/src/cluster.rs#QuasiStaticMna::solve"),
            given: "a diode whose cathode is fed from 3.3 volts through 220 ohms and whose anode \
                    is on 0 volts",
        });
        expect!(
            "cathode-at-the-source",
            "the cathode reads the full 3.3 volts",
            "an off diode carries only leakage, so nothing drops across the resistor"
        );
        expect!("one-solve", "the operating point is found in one solve");
        expect!(
            "leakage-only",
            "the diode carries under ten nanoamps",
            "an off element conducts through its gigaohm of leakage and nothing more"
        );
        let cluster = Cluster {
            nodes: vec![NetId(0), NetId(1), NetId(2)],
            resistors: vec![resistor(0, 1, 220.0)],
        };
        let inputs = ClusterInputs {
            terminals: vec![terminal(0, 3.3), terminal(2, 0.0)],
            elements: vec![diode(2, 1, 0.75, 0.0)],
            ..Default::default()
        };
        let solution = QuasiStaticMna.solve(&cluster, &inputs);
        assert!(solution.converged);
        assert_eq!(solution.solves, 1);
        assert_eq!(solution.regions, vec![Region::Off]);
        // Within the leakage drop: 3.3 nA through 220 Ω is under a microvolt.
        assert!((analog_volts(&solution, NetId(1)) - 3.3).abs() < 1e-5);
        let current = solution.branch_currents[0].unwrap();
        assert!(current.abs() < 10e-9, "{current}");
    }

    /// A channel conducts while its control passes the test on
    /// `V(control) − V(b)`, in either direction of the test.
    #[rstest]
    #[case::n_type_on(RegionTest::AtLeast(2.0), 3.3, true)]
    #[case::n_type_off(RegionTest::AtLeast(2.0), 0.0, false)]
    #[case::p_type_on(RegionTest::AtMost(-2.0), 0.0, true)]
    #[case::p_type_off(RegionTest::AtMost(-2.0), 3.3, false)]
    fn a_channel_follows_its_control_test(
        #[case] test: RegionTest,
        #[case] gate_volts: f64,
        #[case] on: bool,
    ) {
        behaviour!(Test {
            id: "solver.channel-follows-its-control",
            covers: Some("board/src/cluster.rs#QuasiStaticMna::solve"),
            given: "a switched channel between a node pulled up through 1 kilohm and a reference \
                    rail, its control held at one rail or the other, its on test declared for a \
                    positive or a negative threshold",
        });
        expect!(
            "on-when-the-test-passes",
            "the channel conducts exactly when the control-to-reference voltage passes the \
             declared test, pulling the node to its reference",
        );
        expect!(
            "off-otherwise",
            "otherwise the node stays at the pull-up's rail",
            "an off channel carries only leakage"
        );
        // n0 3.3 V terminal, n1 the switched node, n2 the reference (b),
        // n3 the control.
        let reference_volts = match test {
            RegionTest::AtLeast(_) => 0.0,
            RegionTest::AtMost(_) => 3.3,
        };
        let cluster = Cluster {
            nodes: vec![NetId(0), NetId(1), NetId(2), NetId(3)],
            resistors: vec![resistor(0, 1, 1_000.0)],
        };
        let inputs = ClusterInputs {
            terminals: vec![
                terminal(0, 3.3),
                terminal(2, reference_volts),
                terminal(3, gate_volts),
            ],
            elements: vec![channel(1, 2, 3, 1.0, test)],
            ..Default::default()
        };
        let solution = QuasiStaticMna.solve(&cluster, &inputs);
        assert!(solution.converged);
        assert_eq!(
            solution.regions,
            vec![if on { Region::On } else { Region::Off }]
        );
        let node = analog_volts(&solution, NetId(1));
        if on {
            // 1 Ω against 1 kΩ: within 0.4 % of the reference.
            assert!(
                (node - reference_volts).abs() < 3.3 / 1_000.0 * 1.001,
                "{node}"
            );
        } else {
            // The off channel leaks 3.3 nA through its gigaohm, 3.3 µV
            // across the 1 kΩ pull-up: a real drop, and the whole of it.
            assert!((node - 3.3).abs() < 1e-5, "{node}");
        }
    }

    /// Two channels whose tests chase each other — each turns on exactly
    /// when the other's node is at the level the other turns off at — have
    /// no consistent set of regions. The loop runs its bound of two solves
    /// per element and stops: not converged, every non-terminal node
    /// floating, no current, no NaN.
    #[rstest]
    fn a_pair_of_elements_that_chase_each_other_is_non_convergent_at_the_bound() {
        behaviour!(Test {
            id: "solver.non-convergence-is-bounded",
            covers: Some("board/src/cluster.rs#QuasiStaticMna::solve"),
            given: "two switched channels, each pulling its own node down from 3.3 volts, the \
                    first switched on by the second's node being high and the second switched \
                    on by the first's node being low",
        });
        expect!(
            "stops-at-the-bound",
            "the solve gives up after exactly four solves, two per element",
            "the flip loop is bounded so a pair that chases each other costs a fixed amount \
             and never spins"
        );
        expect!(
            "not-converged",
            "the solution is reported non-convergent with every element off",
        );
        expect!(
            "nodes-float",
            "both switched nodes float and the terminals keep their constants",
            "a cluster with no consistent operating point publishes no voltage for the nodes \
             the elements decide; a declared terminal is a constant regardless"
        );
        expect!(
            "no-nan",
            "no node carries a non-finite voltage and no branch reports a current",
        );
        // n0 3.3 V, n1 P, n2 Q, n3 0 V ground.
        let cluster = Cluster {
            nodes: vec![NetId(0), NetId(1), NetId(2), NetId(3)],
            resistors: vec![resistor(0, 1, 1_000.0), resistor(0, 2, 1_000.0)],
        };
        let inputs = ClusterInputs {
            terminals: vec![terminal(0, 3.3), terminal(3, 0.0)],
            elements: vec![
                channel(1, 3, 2, 1.0, RegionTest::AtLeast(1.5)),
                channel(2, 3, 1, 1.0, RegionTest::AtMost(1.5)),
            ],
            ..Default::default()
        };
        let solution = QuasiStaticMna.solve(&cluster, &inputs);
        assert!(!solution.converged);
        assert_eq!(solution.solves, PWL_SOLVES_PER_ELEMENT * 2);
        assert_eq!(solution.regions, vec![Region::Off, Region::Off]);
        assert_eq!(solution.state_of(NetId(1)), Some(NetState::Floating));
        assert_eq!(solution.state_of(NetId(2)), Some(NetState::Floating));
        assert_eq!(solution.state_of(NetId(0)), Some(NetState::Analog(3.3)));
        assert_eq!(solution.state_of(NetId(3)), Some(NetState::Analog(0.0)));
        assert_eq!(solution.branch_currents, vec![None, None]);
        for (_, state) in &solution.node_states {
            if let NetState::Analog(v) = state {
                assert!(v.is_finite());
            }
        }
    }

    /// A node the only path to which is an off diode's leakage is fed by
    /// no source: it floats, and says so, rather than reporting the near
    /// side's voltage as its own.
    #[rstest]
    fn a_node_reached_only_through_leakage_floats() {
        behaviour!(Test {
            id: "solver.leakage-only-node-floats",
            covers: Some("board/src/cluster.rs#QuasiStaticMna::solve"),
            given: "a diode whose anode is on a 3.3 volt terminal and whose cathode net has \
                    nothing else on it",
        });
        expect!(
            "cathode-floats",
            "the cathode floats",
            "an off element's gigaohm keeps the matrix solvable but is not a source; a node no \
             source or terminal feeds over a resistor or a conducting element has no voltage"
        );
        expect!("diode-off", "the diode is off");
        let cluster = Cluster {
            nodes: vec![NetId(0), NetId(1)],
            resistors: vec![],
        };
        let inputs = ClusterInputs {
            terminals: vec![terminal(0, 3.3)],
            elements: vec![diode(0, 1, 0.75, 0.0)],
            ..Default::default()
        };
        let solution = QuasiStaticMna.solve(&cluster, &inputs);
        assert!(solution.converged);
        assert_eq!(solution.state_of(NetId(1)), Some(NetState::Floating));
        assert_eq!(solution.regions, vec![Region::Off]);
        assert_eq!(solution.state_of(NetId(0)), Some(NetState::Analog(3.3)));
    }

    /// A channel with no control is a declared-open switch, and a control
    /// on a node nothing reaches evaluates as off.
    #[rstest]
    fn a_channel_with_no_control_or_a_floating_one_is_off() {
        behaviour!(Test {
            id: "solver.uncontrolled-channel-is-off",
            covers: Some("board/src/cluster.rs#QuasiStaticMna::solve"),
            given: "a switched channel pulled up through 1 kilohm to 3.3 volts over a 0 volt \
                    reference, once with no control declared and once with a control on a net \
                    no source reaches",
        });
        expect!(
            "off-both-ways",
            "the channel is off and its node reads the pull-up's rail in both cases",
            "a test with no operand cannot pass, and a channel with no test never conducts"
        );
        let cluster = Cluster {
            nodes: vec![NetId(0), NetId(1), NetId(2), NetId(3)],
            resistors: vec![resistor(0, 1, 1_000.0)],
        };
        for control in [None, Some((NetId(3), RegionTest::AtLeast(0.0)))] {
            let inputs = ClusterInputs {
                terminals: vec![terminal(0, 3.3), terminal(2, 0.0)],
                elements: vec![ClusterElement {
                    a: NetId(1),
                    b: NetId(2),
                    curve: PwlCurve::Channel { r_on: 1.0 },
                    control,
                }],
                ..Default::default()
            };
            let solution = QuasiStaticMna.solve(&cluster, &inputs);
            assert!(solution.converged);
            assert_eq!(solution.regions, vec![Region::Off]);
            // Within the off channel's leakage drop (3.3 nA through 1 kΩ).
            assert!((analog_volts(&solution, NetId(1)) - 3.3).abs() < 1e-5);
        }
    }

    fn regulator(a: usize, b: usize, i_reg: f64, v_reg: f64) -> ClusterElement {
        ClusterElement {
            a: NetId(a),
            b: NetId(b),
            curve: PwlCurve::Regulator { i_reg, v_reg },
            control: None,
        }
    }

    fn bjt(c: usize, e: usize, base: usize, hfe: f64, r_sat: f64, vbe: f64) -> ClusterElement {
        ClusterElement {
            a: NetId(c),
            b: NetId(e),
            curve: PwlCurve::Bjt { hfe, r_sat },
            control: Some((NetId(base), RegionTest::AtLeast(vbe))),
        }
    }

    /// A P-channel polarity FET — body diode drain to source, then the
    /// channel gated by the gate at or below −2 V — starting up into a
    /// 100 Ω load, with its gate on the ground terminal: the diode lifts
    /// the source, the gate falls below threshold, the channel turns on
    /// and shorts the diode off; reversed, nothing conducts.
    #[rstest]
    #[case::forward(12.0, true)]
    #[case::reversed(-12.0, false)]
    fn a_polarity_fet_starts_through_its_body_diode_and_ends_on_its_channel(
        #[case] input_volts: f64,
        #[case] conducts: bool,
    ) {
        behaviour!(Test {
            id: "cluster.polarity-fet-start-up",
            covers: Some("board/src/cluster.rs#QuasiStaticMna::solve"),
            given: "a P-channel FET declared as its body diode from drain to source and then its \
                    channel gated by the gate at or below minus two volts, its gate on the ground \
                    terminal and a 100 ohm load from its source to ground, with the drain first \
                    12 volts above ground and then 12 volts below it",
        });
        expect!(
            "forward-sequence",
            "with the drain above ground the solve converges in exactly four linear solves — \
             the body diode on, the channel on, the body diode off again — and rests with the \
             channel on and the diode off",
            "every element starts off; the diode is the first to disagree, the source it \
             lifts puts the gate below threshold, and the channel it turns on shorts the diode \
             back off"
        );
        expect!(
            "forward-source",
            "with the drain above ground the source sits at the input less the channel's \
             share of the load, and the load current flows through the channel, none through \
             the diode",
        );
        expect!(
            "reversed-blocks",
            "with the drain below ground nothing conducts: the source sits within a microvolt \
             of ground, both branches carry under a microamp, and the solve converges in one \
             linear solve",
            "the gate is above the source and the body diode is reverse-biased, so the cold \
             start is already consistent"
        );
        // n0 the input, n1 the source (the protected rail), n2 ground —
        // also the gate. The load ties the source to ground.
        let cluster = Cluster {
            nodes: vec![NetId(0), NetId(1), NetId(2)],
            resistors: vec![resistor(1, 2, 100.0)],
        };
        let inputs = ClusterInputs {
            terminals: vec![terminal(0, input_volts), terminal(2, 0.0)],
            elements: vec![
                diode(0, 1, 1.3, 0.0),
                channel(0, 1, 2, 0.06, RegionTest::AtMost(-2.0)),
            ],
            ..Default::default()
        };
        let solution = QuasiStaticMna.solve(&cluster, &inputs);
        assert!(solution.converged);
        let source = analog_volts(&solution, NetId(1));
        if conducts {
            assert_eq!(solution.solves, 4);
            assert_eq!(solution.regions, vec![Region::Off, Region::On]);
            let expected = 12.0 * 100.0 / 100.06;
            assert!((source - expected).abs() < 1e-9, "{source}");
            let through_channel = solution.branch_currents[1].unwrap();
            assert!(
                (through_channel - 12.0 / 100.06).abs() < 1e-9,
                "{through_channel}"
            );
            assert!(solution.branch_currents[0].unwrap().abs() < 1e-8);
        } else {
            assert_eq!(solution.solves, 1);
            assert_eq!(solution.regions, vec![Region::Off, Region::Off]);
            // The two branches' leakages (2 nS) pull −12 V across the
            // 100 Ω load: −2.4 µV.
            assert!(source.abs() < 1e-5, "{source}");
            for current in solution.branch_currents.iter().flatten() {
                assert!(current.abs() < 1e-6, "{current}");
            }
        }
    }

    /// A constant-current regulator into a 100 Ω load: below its knee it
    /// is the ohmic segment from the origin to the knee, at or above it a
    /// current source.
    #[rstest]
    #[case::below_the_knee(1.0, false)]
    #[case::at_the_knee(2.0, false)]
    #[case::regulating(24.0, true)]
    fn a_current_regulator_is_ohmic_below_its_knee_and_a_source_above_it(
        #[case] supply_volts: f64,
        #[case] regulating: bool,
    ) {
        behaviour!(Test {
            id: "cluster.regulator-two-regions",
            covers: Some("board/src/cluster.rs#QuasiStaticMna::solve"),
            given: "a two-region current regulator — 10 milliamps at or above a 1.8 volt knee, \
                    the straight segment from the origin to the knee below it — in series with a \
                    100 ohm load between a terminal and ground, the terminal at 1, 2 and 24 volts",
        });
        expect!(
            "ohmic-below-the-knee",
            "while the voltage across the regulator stays under its knee the loop carries the \
             supply divided by the load plus the knee's 180 ohms, from the first solve",
            "the regulator's cold start is its ohmic segment, so a loop below the knee is \
             consistent without a flip"
        );
        expect!(
            "regulates-above-it",
            "with the supply well above the knee the loop carries exactly the regulation \
             current after one flip, whatever the supply",
            "past the knee the branch is a current source whose value the voltage across it \
             does not change"
        );
        let cluster = Cluster {
            nodes: vec![NetId(0), NetId(1), NetId(2)],
            resistors: vec![resistor(1, 2, 100.0)],
        };
        let inputs = ClusterInputs {
            terminals: vec![terminal(0, supply_volts), terminal(2, 0.0)],
            elements: vec![regulator(0, 1, 10e-3, 1.8)],
            ..Default::default()
        };
        let solution = QuasiStaticMna.solve(&cluster, &inputs);
        assert!(solution.converged);
        let current = solution.branch_currents[0].unwrap();
        if regulating {
            assert_eq!(solution.solves, 2);
            assert_eq!(solution.regions, vec![Region::On]);
            // The regulating branch is the source beside its leakage: 23 V
            // across 1 GΩ adds 23 nA.
            assert!((current - 10e-3).abs() < 1e-7, "{current}");
            assert!((analog_volts(&solution, NetId(1)) - 1.0).abs() < 1e-5);
        } else {
            assert_eq!(solution.solves, 1);
            assert_eq!(solution.regions, vec![Region::Off]);
            let expected = supply_volts / (100.0 + 180.0);
            assert!((current - expected).abs() < 1e-9, "{current} vs {expected}");
        }
    }

    /// A transistor switch — base–emitter diode, then the collector gated
    /// by the base — driven through 43 kΩ from 5 V: a light load saturates
    /// it, a heavy one leaves it active at the current the base supports.
    #[rstest]
    #[case::light_load_saturates(1_000.0, Region::On)]
    #[case::heavy_load_sags(220.0, Region::Active)]
    fn a_transistor_saturates_under_a_light_load_and_sags_under_a_heavy_one(
        #[case] load_ohms: f64,
        #[case] collector: Region,
    ) {
        behaviour!(Test {
            id: "cluster.transistor-three-regions",
            covers: Some("board/src/cluster.rs#QuasiStaticMna::solve"),
            given: "a transistor declared as its base-emitter diode at a 0.65 volt knee and its \
                    collector gated by the base, with a minimum gain of 100 and a saturated \
                    resistance of 20 ohms, its base driven through 43 kilohms from 5 volts, its \
                    emitter on ground and its collector loaded from 5 volts through 1 kilohm and \
                    then through 220 ohms",
        });
        expect!(
            "base-at-the-knee",
            "the base sits at the knee and carries the 5 volts less the knee over 43 kilohms",
            "a driven base-emitter junction drops its knee and the base resistor sets the \
             current"
        );
        expect!(
            "light-load-saturates",
            "under the 1 kilohm load the collector saturates: it carries the load current \
             through 20 ohms and sits that drop above the emitter, in three linear solves",
            "the base supports a hundred times its own current, more than the load asks for"
        );
        expect!(
            "heavy-load-sags",
            "under the 220 ohm load the collector is active: it carries exactly a hundred \
             times the base current and sits volts above the emitter, in four linear solves",
            "the load asks for more than the base supports, so the collector current is the \
             base's to give and the load line sets the voltage — an under-driven base reads \
             as a sagging collector"
        );
        // n0 the collector supply, n1 the collector, n2 ground (the
        // emitter), n3 the base, n4 the base drive.
        let cluster = Cluster {
            nodes: vec![NetId(0), NetId(1), NetId(2), NetId(3), NetId(4)],
            resistors: vec![resistor(0, 1, load_ohms), resistor(4, 3, 43_000.0)],
        };
        let inputs = ClusterInputs {
            terminals: vec![terminal(0, 5.0), terminal(2, 0.0), terminal(4, 5.0)],
            elements: vec![diode(3, 2, 0.65, 0.0), bjt(1, 2, 3, 100.0, 20.0, 0.65)],
            ..Default::default()
        };
        let solution = QuasiStaticMna.solve(&cluster, &inputs);
        assert!(solution.converged);
        assert_eq!(solution.regions, vec![Region::On, collector]);
        let base = analog_volts(&solution, NetId(3));
        assert!((base - 0.65).abs() < 1e-5, "{base}");
        let i_b = solution.branch_currents[0].unwrap();
        let expected_i_b = (5.0 - base) / 43_000.0;
        // A vertical diode's current is read off a microohm segment, so
        // the reported figure carries the cancellation of `g·V − I` at the
        // 1e-6 level.
        assert!((i_b - expected_i_b).abs() < expected_i_b * 1e-4, "{i_b}");
        let i_c = solution.branch_currents[1].unwrap();
        let v_c = analog_volts(&solution, NetId(1));
        match collector {
            Region::On => {
                assert_eq!(solution.solves, 3);
                let expected = 5.0 / (load_ohms + 20.0);
                assert!((i_c - expected).abs() < expected * 1e-6, "{i_c}");
                assert!((v_c - i_c * 20.0).abs() < 1e-6, "{v_c}");
                assert!(i_c < 100.0 * i_b);
            }
            Region::Active => {
                assert_eq!(solution.solves, 4);
                assert!(
                    (i_c - 100.0 * i_b).abs() < i_c * 1e-4,
                    "{i_c} vs {}",
                    100.0 * i_b
                );
                assert!((v_c - (5.0 - i_c * load_ohms)).abs() < 1e-6, "{v_c}");
                assert!(v_c > 1.0, "a sagging collector, not a switch: {v_c}");
            }
            Region::Off => unreachable!(),
        }
    }
}
