//! The same declared scenario must settle the same way every time.
//!
//! `edgeboard.rs`'s RS422 receiver case fails about half the time, alone, in a
//! fresh process — the receiver output reads `Floating` instead of
//! `Driven(High)`. That contradicts a guarantee this project has already
//! shipped: `DETERMINISM.md` says T1 holds *fully* for "systems whose only
//! actors are engine-hosted components (board + models + faults + streams,
//! scripted stimulus)", and this system is exactly that — a board, a harness
//! and a declarative scenario, with no firmware anywhere.
//!
//! So this is not a flaky test to be papered over with a longer wait. It is a
//! simulator that reaches two different steady states from one input, and the
//! test that finds it belongs in the suite.
//!
//! Running the scenario N times **inside one process** is what makes the cause
//! legible. Rust seeds its hasher once per process, so:
//!
//! * all N runs agreeing, but differing between processes, means hash order
//!   has leaked into the solve;
//! * runs disagreeing *within* one process means thread scheduling or event
//!   ordering, and the hasher is innocent.

mod machine_parts;

use std::sync::Once;
use std::time::{Duration, Instant};

use embsim_board::{Level, NetState, Scenario, System, SystemHandle};
use machine_parts::{
    bench_rails, edge_board, edge_polarity_fet_conducting, encoder_jumpers_closed,
};

/// The isolator input the P2 reads as P9 — the receiver's channel-1 output.
const OUTPUT: &str = "EdgeBoard.Net-(IC16-INA)";
const A_PLUS: &str = "EdgeBoard./MaD_Edge_Sheet3/A+";

fn ensure_clock() {
    static CLOCK: Once = Once::new();
    CLOCK.call_once(|| embsim_core::virtual_clock::init(1.0, 1_000_000));
}

fn wait_for(mut pred: impl FnMut() -> bool, timeout: Duration) -> bool {
    let start = Instant::now();
    while start.elapsed() < timeout {
        if pred() {
            return true;
        }
        std::thread::sleep(Duration::from_millis(1));
    }
    pred()
}

/// `case_1_forward` from `edgeboard.rs`: A+ driven high, A- pulled to 0 by the
/// closed jumper, so the receiver should decode a logic high.
fn start_forward() -> SystemHandle {
    ensure_clock();
    let scenario = encoder_jumpers_closed(
        edge_polarity_fet_conducting(Scenario::default(), "EdgeBoard"),
        "EdgeBoard",
    )
    .net_stuck(A_PLUS, 3.3);
    System::new()
        .board("EdgeBoard", edge_board())
        .harness(bench_rails("EdgeBoard"))
        .scenario(scenario)
        .event_log()
        .start()
        .expect("the servo-domain system starts")
}

/// One run: settle, then report what the receiver output came to rest at.
fn settle_once() -> (NetState, Vec<String>) {
    let system = start_forward();
    let want = NetState::Driven(Level::High);
    wait_for(
        || system.net_state(OUTPUT) == Some(want),
        Duration::from_secs(5),
    );
    let got = system
        .net_state(OUTPUT)
        .unwrap_or_else(|| panic!("net {OUTPUT} exists"));
    let log = system.event_log().normalized_shape();
    system.shutdown();
    (got, log)
}

/// KNOWN FAILING, roughly one process in ten. Kept as a runnable reproduction
/// rather than deleted, because it is the only thing that catches the residual bug
/// and it catches it *inside one process*, which is what rules out hash order.
///
/// Two distinct causes were found here. The first is fixed: sense callbacks
/// dirty clusters after the pass that delivered them had already resolved, so
/// the incremental resolver never gave them a second lap — see
/// `resolve_and_publish_dirty`. That alone took the `edgeboard` RS422 case
/// from ~50% failure to 15 clean runs in 15.
///
/// The second is still open. The event log shows a `drive_applied seq=9
/// endpoint=90 drive=release` landing at a different point relative to a
/// `sense`, and three extra events in the failing trace — a command drain
/// interleaving with resolution differently between runs. Run this with
/// `--ignored` to reproduce; it prints the first diverging event.
#[ignore = "reproduces an open engine nondeterminism: drive-release ordering vs resolve"]
#[test]
fn the_rs422_receiver_settles_the_same_way_every_time() {
    const RUNS: usize = 12;

    let mut outcomes: Vec<(NetState, Vec<String>)> = Vec::with_capacity(RUNS);
    for _ in 0..RUNS {
        outcomes.push(settle_once());
    }

    let states: Vec<NetState> = outcomes.iter().map(|(s, _)| *s).collect();
    let first = states[0];
    if states.iter().all(|s| *s == first) {
        // Consistent within this process. If CI still sees both answers, the
        // difference is between processes and the hasher seed is the suspect.
        assert_eq!(
            first,
            NetState::Driven(Level::High),
            "every run in this process agreed, but on the WRONG state — so the \
             solve is stable and simply incorrect for this scenario, not racy"
        );
        return;
    }

    // Disagreement inside one process: the hasher seed is fixed for all of
    // these, so it cannot be hash order. Show where the event traces diverge,
    // which is the whole reason the engine keeps a log.
    let good = outcomes
        .iter()
        .find(|(s, _)| *s == NetState::Driven(Level::High));
    let bad = outcomes
        .iter()
        .find(|(s, _)| *s != NetState::Driven(Level::High));
    let mut detail = String::new();
    if let (Some((_, g)), Some((_, b))) = (good, bad) {
        detail.push_str(&format!(
            "\ngood trace {} events, bad trace {} events\n",
            g.len(),
            b.len()
        ));
        for (i, (ge, be)) in g.iter().zip(b.iter()).enumerate() {
            if ge != be {
                detail.push_str(&format!(
                    "first divergence at event {i}:\n  good: {ge}\n  bad:  {be}\n"
                ));
                break;
            }
        }
    }
    panic!(
        "the same declared scenario settled differently within ONE process: {:?}\n\
         Hash order is ruled out (one hasher seed for all {RUNS} runs), so this is \
         event ordering or thread scheduling.{detail}",
        states
    );
}
