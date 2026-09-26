//! What one escalated cluster solve costs, at the cluster sizes the plan
//! argues about.
//!
//! `NODES.md` §6 bounds every cluster at `m ≤ 8` under each board's
//! reference harness by making declared terminals cluster boundaries;
//! `board/tests/cluster_census.rs` records each board's largest cluster.
//! With diodes as edges and no boundary the Edge board's +3.3V, +5V, GND
//! and all 21 LED chains collapse into one 47-net cluster (§2, rule 1).
//! This records what the dense elimination in `QuasiStaticMna` costs at the
//! sizes that bound argues about, so the decision rests on a number in the
//! tree (the LU-cache decision is `NODES.md` §8, phase 7).
//!
//! The cluster at each size is representative of the boards, not synthetic:
//! a 3.3 V rail terminal and a ground terminal, kept as ideal 0 Ω sources so
//! that `m` counts them as unknowns: an upper bound on the matrix the engine
//! builds, which hands a terminal over as a constant. Signal nodes are each
//! held by a 10.5 kΩ pull-up to the rail (the EC32MB's `R301`–`R303`),
//! joined in a chain of 220 Ω series resistors (the Edge board's LED
//! resistors `R9`…), every third node driven by a push-pull pad at the
//! engine's default drive impedance ([`DEFAULT_PUSH_PULL_IMPEDANCE`],
//! `BOARD_ENGINE.md` "Net state model") at alternating levels — so the
//! solve is a real divided-voltage one, never a single-source projection.
//!
//! Not a test: the figures are hardware-dependent. Run it, in release, as
//!
//! ```text
//! cargo run -p embsim-board --release --example solve_bench
//! ```
//!
//! and read the table. Elimination is O(m³): the last column is the
//! multiply-add count of a dense Gaussian elimination, m³/3, for scale.

use std::hint::black_box;
use std::time::{Duration, Instant};

use embsim_board::net::DEFAULT_PUSH_PULL_IMPEDANCE;
use embsim_board::{
    Cluster, ClusterInputs, ClusterResistor, ClusterSolver, ClusterSource, NetId, NetState,
    QuasiStaticMna,
};

/// The sizes the plan argues about:
/// - 2, a pull-up against a pad;
/// - 4, a short chain;
/// - 8, rule 4's bound under the reference harnesses (`cluster_census`
///   `RULE_4_LARGEST_CLUSTER_ROOTS`);
/// - 29, the Edge board's largest cluster built bare (`cluster_census`
///   `EDGE`);
/// - 47, the Edge board with no terminal boundaries.
const SIZES: [usize; 5] = [2, 4, 8, 29, 47];

/// Rail the pull-ups reach and the rail terminal sources.
const RAIL_VOLTS: f64 = 3.3;
/// EC32MB `R301`–`R303` (`boards/netlists/p2_ec32mb.net`).
const PULL_UP_OHMS: f64 = 10_500.0;
/// Edge board LED series resistors (`board/tests/fixtures/mad_edge.net`, `220R`).
const SERIES_OHMS: f64 = 220.0;

/// Wall time each timed batch aims for; batches are repeated and the median
/// and the minimum per-solve figure reported.
const BATCH_TARGET: Duration = Duration::from_millis(60);
const BATCHES: usize = 7;

/// One representative cluster of `m` roots with its sources.
fn representative(m: usize) -> (Cluster, ClusterInputs) {
    assert!(m >= 2, "a cluster of one root never solves");
    let rail = 0usize;
    let ground = (m >= 3).then_some(m - 1);
    // Signal nodes: everything that is neither rail nor ground.
    let signals: Vec<usize> = (1..m).filter(|&n| Some(n) != ground).collect();

    let mut resistors = Vec::new();
    for &n in &signals {
        resistors.push(ClusterResistor {
            a: NetId(rail),
            b: NetId(n),
            ohms: PULL_UP_OHMS,
        });
    }
    for pair in signals.windows(2) {
        resistors.push(ClusterResistor {
            a: NetId(pair[0]),
            b: NetId(pair[1]),
            ohms: SERIES_OHMS,
        });
    }
    if let (Some(g), Some(&last)) = (ground, signals.last()) {
        resistors.push(ClusterResistor {
            a: NetId(last),
            b: NetId(g),
            ohms: SERIES_OHMS,
        });
    }

    let mut sources = vec![ClusterSource {
        node: NetId(rail),
        volts: RAIL_VOLTS,
        impedance: 0.0,
    }];
    if let Some(g) = ground {
        sources.push(ClusterSource {
            node: NetId(g),
            volts: 0.0,
            impedance: 0.0,
        });
    }
    // Every third signal node carries a pad, at alternating levels, so the
    // matrix has disagreeing sources within a factor of ten of each other
    // through the chain — the case that escalates.
    for (i, &n) in signals.iter().enumerate().filter(|(i, _)| i % 3 == 0) {
        sources.push(ClusterSource {
            node: NetId(n),
            volts: if (i / 3) % 2 == 0 { 0.0 } else { RAIL_VOLTS },
            impedance: DEFAULT_PUSH_PULL_IMPEDANCE,
        });
    }

    let cluster = Cluster {
        nodes: (0..m).map(NetId).collect(),
        resistors,
    };
    (
        cluster,
        ClusterInputs {
            sources,
            ..Default::default()
        },
    )
}

/// Nanoseconds per solve over one batch sized to `BATCH_TARGET`.
fn time_batch(cluster: &Cluster, inputs: &ClusterInputs) -> f64 {
    // Size the batch from a short calibration run.
    let calibrate = Instant::now();
    let mut n = 0u64;
    while calibrate.elapsed() < Duration::from_millis(5) {
        black_box(QuasiStaticMna.solve(black_box(cluster), black_box(inputs)));
        n += 1;
    }
    let iterations = (n * BATCH_TARGET.as_millis() as u64 / 5).max(1);

    let start = Instant::now();
    for _ in 0..iterations {
        black_box(QuasiStaticMna.solve(black_box(cluster), black_box(inputs)));
    }
    start.elapsed().as_nanos() as f64 / iterations as f64
}

fn main() {
    println!(
        "QuasiStaticMna, one solve of a representative cluster (rail + ground terminals, \
         {PULL_UP_OHMS} Ω pull-ups, {SERIES_OHMS} Ω chain, {DEFAULT_PUSH_PULL_IMPEDANCE} Ω pads on every third node); \
         median and min of {BATCHES} batches of ~{} ms",
        BATCH_TARGET.as_millis()
    );
    println!(
        "{:>4} {:>8} {:>6} {:>14} {:>11} {:>10}",
        "m", "sources", "edges", "ns/solve(med)", "ns/solve(min)", "m^3/3"
    );
    for &m in &SIZES {
        let (cluster, inputs) = representative(m);
        // Every node must come out solved, or the bench would be timing a
        // reachability short-circuit rather than an elimination.
        let solution = QuasiStaticMna.solve(&cluster, &inputs);
        for (node, state) in &solution.node_states {
            assert!(
                matches!(state, NetState::Analog(v) if v.is_finite()),
                "node {node:?} in the m={m} cluster came out {state:?}, not solved"
            );
        }

        let mut samples: Vec<f64> = (0..BATCHES)
            .map(|_| time_batch(&cluster, &inputs))
            .collect();
        samples.sort_by(|a, b| a.total_cmp(b));
        let median = samples[samples.len() / 2];
        let min = samples[0];
        println!(
            "{m:>4} {:>8} {:>6} {median:>14.0} {min:>11.0} {:>10}",
            inputs.sources.len(),
            cluster.resistors.len(),
            m * m * m / 3
        );
    }
}
