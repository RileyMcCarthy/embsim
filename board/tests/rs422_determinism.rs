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
use vibes_behaviour::{behaviour, expect, Test};

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

/// A reproduction for a real but ELUSIVE nondeterminism. `#[ignore]`d because
/// it does not fire reliably, not because it is unimportant.
///
/// What is established:
///
/// * The bug is real. CI has hit it, and it was observed many times across a
///   day — the receiver output resting at `Floating` where the same declared
///   scenario usually gives `Driven(High)`.
/// * It is not hash order. This test runs the scenario twelve times in ONE
///   process, so all twelve share a hasher seed, and it has caught runs
///   disagreeing with each other inside that one process.
/// * The engine event log names the divergence: a `drive_applied seq=..
///   drive=release` landing at a different point relative to a `sense`, with
///   extra events in the failing trace. That is a command drain interleaving
///   with resolution differently between runs.
///
/// What is NOT established, and was wrongly claimed once:
///
/// * A first attempt looped `resolve_and_publish_dirty` to a fixpoint, on the
///   theory that a sense callback's `set_drive` dirtied a cluster after its
///   pass had resolved. That theory is WRONG: `PinHandle::set_drive` enqueues
///   a `Command::Drive` and never touches the resolver inline (see its doc in
///   `component.rs`), so a sense callback cannot mark anything dirty during
///   the pass. Instrumenting the loop proved it never ran a second lap. The
///   apparent "~50% to 15/15" improvement was a confounded measurement: the
///   before-numbers were taken while the machine was saturated by other work.
/// * The failure rate is strongly timing-dependent and has ranged from ~50%
///   to zero across sessions on the same commit. Synthetic CPU load does not
///   reproduce it on demand.
///
/// So whoever picks this up: the mechanism is still open, this test is the
/// sharpest instrument available for it, and the honest first step is a
/// reproduction that fires on demand rather than another plausible story.
///
/// **Phase 0 of `NODES.md` (2026-09-23).** Recipe:
///
/// ```text
/// cargo test -p embsim-board --test rs422_determinism -- --ignored --nocapture
/// ```
///
/// It fired on the first invocation that day: one run of twelve rested
/// `Floating`, in 0.2 s of wall time for all twelve, and the two traces were
/// the same 175 events until event 162, where the good run delivered
/// `sense net=44 state=analog:5000000uv` and the bad run applied
/// `drive_applied seq=26 endpoint=91 drive=release` first — a component's
/// attach-time release reaching the drive table before or after a
/// resolution pass delivers its senses. That is the free-running
/// interleaving `DETERMINISM.md` documents as unfixed at T0, landing on a
/// scenario whose settled state depends on it. The mechanism stays open;
/// what phase 0 decided is the rule in `TESTING.md` (rule 9): until this
/// closes, every new model's proving test runs in stepped mode, where the
/// engine quiesces every actor before it advances. Run this a few times if
/// it does not fire at once — the rate has ranged from one in two to zero
/// across sessions on one commit.
#[ignore = "reproduces an open engine nondeterminism; run with --ignored"]
#[test]
fn the_rs422_receiver_settles_the_same_way_every_time() {
    behaviour!(Test {
        id: "rs422.settles-the-same-way",
        covers: Some("board/src/engine.rs#EngineCore::run_stepped_iteration"),
        given: "one declared RS-422 scenario, its A+ line at 3.3 volts and its A- line \
                at ground, is started and settled repeatedly inside one process",
    });
    expect!(
        "agree-across-runs",
        "every run comes to rest with the receiver output in the same state",
        "a simulation with no firmware in it, a board and scripted stimulus alone, must \
         reach one steady state for one input whatever the thread scheduling"
    );
    expect!(
        "decodes-high",
        "the receiver output settles driven high",
        "a high A+ leg against a grounded A- leg is a differential high"
    );

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
