//! MNA hand-checked reference circuits (`BOARD_ENGINE.md` "Testing
//! conventions": hand-computed reference circuits asserted to µV).
//!
//! Each test states its closed-form hand calculation next to the assertion;
//! tolerances are 1e-6 V unless a comment says otherwise. Component values
//! come from the MaD reference consumer: the DS2Addon 47 Ω series resistors,
//! the 350 Ω strain-gauge bridge (`SIL/models/src/strain_gauge.rs`), and the
//! 25 Ω default push-pull drive impedance.

use embsim_board::{
    Cluster, ClusterInputs, ClusterResistor, ClusterSolution, ClusterSolver, ClusterSource, NetId,
    NetState, QuasiStaticMna, Volts,
};

/// Unwrap a node's solved analog voltage; panics with the actual state on a
/// projection mismatch so failures read like the hand calculation.
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

fn source(node: usize, volts: f64, impedance: f64) -> ClusterSource {
    ClusterSource {
        node: NetId(node),
        volts,
        impedance,
    }
}

// ============================================================
// (1) Voltage divider — 3.3 V through 47 Ω then 4.7 kΩ to 0 V
// ============================================================

#[test]
fn divider_47r_over_4k7_matches_closed_form() {
    // Hand check: V_mid = 3.3 · 4700 / (47 + 4700) = 3.267326732… V.
    let cluster = Cluster {
        nodes: vec![NetId(0), NetId(1), NetId(2)], // rail, mid, gnd
        resistors: vec![resistor(0, 1, 47.0), resistor(1, 2, 4_700.0)],
    };
    let inputs = ClusterInputs {
        sources: vec![source(0, 3.3, 0.0), source(2, 0.0, 0.0)],
    };
    let solution = QuasiStaticMna.solve(&cluster, &inputs);
    let expected_mid = 3.3 * 4_700.0 / 4_747.0;
    assert!((analog_volts(&solution, NetId(1)) - expected_mid).abs() < 1e-6);
    assert!((analog_volts(&solution, NetId(0)) - 3.3).abs() < 1e-6);
    assert!(analog_volts(&solution, NetId(2)).abs() < 1e-6);
}

// ============================================================
// (2) Wheatstone bridge — MaD strain-gauge values
// ============================================================

#[test]
fn wheatstone_bridge_mad_strain_gauge_matches_bridge_equation() {
    // MaD load-cell values (`SIL/models/src/strain_gauge.rs` provenance):
    // 350 Ω arms, 3.3 V excitation, rated sensitivity S = 1.0 mV/V, so the
    // provenance transfer function V_out = (F/F_fs)·S·V_exc gives 3.3 mV at
    // full scale. A quarter bridge realizes that small-signal output with a
    // fractional arm perturbation δ = 4·S·1e-3 = 0.004 (V_out ≈ V_exc·δ/4).
    //
    // Exact ratiometric bridge equation for one perturbed arm
    // (R2 = R·(1+δ) between SIG+ and GND):
    //   V_sig+ = V_exc·(1+δ)/(2+δ),  V_sig− = V_exc/2,
    //   V_diff = V_exc·δ/(2·(2+δ)) = 3.3·0.004/4.008 = 3.293413173… mV,
    // which reduces to the provenance transfer function to first order in δ.
    let r_arm = 350.0;
    let v_exc = 3.3;
    let delta = 0.004;
    let cluster = Cluster {
        nodes: vec![NetId(0), NetId(1), NetId(2), NetId(3)], // EXC, GND, SIG+, SIG−
        resistors: vec![
            resistor(0, 2, r_arm),                 // EXC — SIG+
            resistor(2, 1, r_arm * (1.0 + delta)), // SIG+ — GND (active gauge)
            resistor(0, 3, r_arm),                 // EXC — SIG−
            resistor(3, 1, r_arm),                 // SIG− — GND
        ],
    };
    let inputs = ClusterInputs {
        sources: vec![source(0, v_exc, 0.0), source(1, 0.0, 0.0)],
    };
    let solution = QuasiStaticMna.solve(&cluster, &inputs);

    let v_sig_p = analog_volts(&solution, NetId(2));
    let v_sig_n = analog_volts(&solution, NetId(3));
    let expected_p = v_exc * (1.0 + delta) / (2.0 + delta); // 1.653293413… V
    assert!((v_sig_p - expected_p).abs() < 1e-6);
    assert!((v_sig_n - v_exc / 2.0).abs() < 1e-6);

    let expected_diff = v_exc * delta / (2.0 * (2.0 + delta)); // 3.293413… mV
    assert!((v_sig_p - v_sig_n - expected_diff).abs() < 1e-6);
    // Tie back to the provenance small-signal transfer function (first order
    // in δ, so a looser 10 µV bound): V_out ≈ S·V_exc = 3.3 mV.
    assert!((v_sig_p - v_sig_n - 3.3e-3).abs() < 1e-5);
}

// ============================================================
// (3) Two stiff sources fighting through a series resistor
// ============================================================

#[test]
fn stiff_sources_fighting_through_series_resistor_sit_between_rails() {
    // The crossed-TX/RX bench case (`BOARD_ENGINE.md` "Stream endpoints"):
    // a 3.3 V/25 Ω push-pull and a 0 V/25 Ω push-pull fight through the
    // DS2Addon's 47 Ω series resistor. Hand check, loop current
    // I = 3.3/(25+47+25) = 3.3/97 A:
    //   V_a = 3.3 − 25·I = 3.3·72/97 = 2.449484536… V
    //   V_b =  0 + 25·I = 3.3·25/97 = 0.850515464… V
    let cluster = Cluster {
        nodes: vec![NetId(0), NetId(1)],
        resistors: vec![resistor(0, 1, 47.0)],
    };
    let inputs = ClusterInputs {
        sources: vec![source(0, 3.3, 25.0), source(1, 0.0, 25.0)],
    };
    let solution = QuasiStaticMna.solve(&cluster, &inputs);

    let v_a = analog_volts(&solution, NetId(0));
    let v_b = analog_volts(&solution, NetId(1));
    assert!((v_a - 3.3 * 72.0 / 97.0).abs() < 1e-6);
    assert!((v_b - 3.3 * 25.0 / 97.0).abs() < 1e-6);
    // Both solved voltages sit strictly between the rails — the mid-rail
    // fight is represented numerically, not just flagged.
    assert!(0.0 < v_b && v_b < v_a && v_a < 3.3);
}

// ============================================================
// (4) Disconnected island — sourced block solves, island floats
// ============================================================

#[test]
fn disconnected_island_reports_floating_only_for_unsourced_nodes() {
    // One cluster, two components: nodes 0–1 are sourced (solve to the
    // open-circuit 3.3 V — no return path, no current); nodes 2–3 have a
    // resistor but no conductive path to any source, so they are exactly
    // the MNA-singular set and must report Floating — never a voltage.
    let cluster = Cluster {
        nodes: vec![NetId(0), NetId(1), NetId(2), NetId(3)],
        resistors: vec![resistor(0, 1, 1_000.0), resistor(2, 3, 1_000.0)],
    };
    let inputs = ClusterInputs {
        sources: vec![source(0, 3.3, 25.0)],
    };
    let solution = QuasiStaticMna.solve(&cluster, &inputs);

    assert!((analog_volts(&solution, NetId(0)) - 3.3).abs() < 1e-6);
    assert!((analog_volts(&solution, NetId(1)) - 3.3).abs() < 1e-6);
    assert_eq!(solution.state_of(NetId(2)), Some(NetState::Floating));
    assert_eq!(solution.state_of(NetId(3)), Some(NetState::Floating));
}
