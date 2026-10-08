//! The same declared scenario must settle the same way every time.
//!
//! A board, a harness and a declarative scenario — no firmware anywhere —
//! started twelve times inside one process: every run must find the RS-422
//! receiver's output, the isolator input the P2 reads as P9, in the one
//! state the differential on its inputs decodes to. `DETERMINISM.md` says T1
//! holds *fully* for "systems whose only actors are engine-hosted components
//! (board + models + faults + streams, scripted stimulus)", and this system
//! is exactly that.
//!
//! # Time
//!
//! Each run re-anchors the clock in stepped mode, starts its system with
//! time held, registers the test's thread as a virtual-clock actor, releases
//! time and parks for [`SETTLE_NS`] of virtual time before it reads — the
//! pattern `isolation_bridge.rs` set (`TESTING.md` rules 5 and 9). The
//! engine advances only while the thread is parked, so the read is of the
//! system at rest, and the run fails if the engine ever stopped waiting for
//! the thread.
//!
//! # What this binary once reproduced
//!
//! It was written, and `#[ignore]`d, as the reproduction of what looked like
//! a simulator reaching two steady states from one input: on the free-running
//! clock one run in twelve read the output `Floating` where the rest read
//! `Driven(High)`, and the run's engine event log diverged from a good run's
//! where a component's attach-time release landed against a sense delivery
//! (phase 0 of `NODES.md`, 2026-09-23; `TESTING.md` rule 9 made stepped mode
//! the rule for proving tests while the mechanism stayed open).
//!
//! The mechanism was a read race, not a second steady state (`NODES.md` §12
//! item 5: the flake record's (A), the same net in `isolation_bridge.rs`,
//! and the performance and stepped-tests record for this binary).
//! `System::start` returns with the attach cascade in flight. The AM26LV32's
//! `1Y` is a push-pull output whose idle is `Driven(High)` — the state the
//! run expects — and on its way there `U25` releases it twice, because the
//! supply's delivery reaches the part before either enable has been read,
//! and `G`'s next reads no level with `Z+` open; `~G`'s, `Z−` at the
//! isolated ground through `JP4`, then drives it high. That start-up
//! release is the model's (the test tree's `Rs422Receiver` then, the
//! catalog's `embsim_models::am26lv32` now: each releases its outputs until
//! it has read its supply and an enable), not the engine's: it
//! happens in every run, stepped or not, and leaves the settled state
//! alone. The free-running run polled until the net read `Driven(High)`,
//! which the idle satisfied at the first glance, before `U25` had run, and
//! then read it again inside those releases. Measured in an instrumented copy of the
//! free-running run under load: every failing read came microseconds after
//! a poll that returned at its first glance, and the same net read
//! `Driven(High)` again 0.2–16 ms of wall time later and for the rest of
//! the run. The diverging logs were the free-running interleaving of the
//! attach thread with the engine's deliveries, read at different wall
//! instants; every run reached the one settled state.
//!
//! That interleaving is still the attach thread's here: `System::start`
//! attaches on the test's thread while the engine delivers, and the thread
//! registers only after it returns, so two runs' logs differ inside the
//! held start-up instant (measured: every run of twelve against the first,
//! 356 to 421 records at the settled instant) while their settled states
//! agree. This binary therefore compares the state at rest, and keeps each
//! run's log to show where two runs part if the states ever disagree.

mod machine_parts;

use std::sync::{Mutex, MutexGuard};

use embsim_board::{Finding, Level, NetState, Scenario, System};
use embsim_core::virtual_clock::{self, ClockMode};
use embsim_models::logic_gate::LVC1G14_T_PD_NS;
use embsim_models::rail::UCC12040_RISE_NS;
use machine_parts::{bench_rails, edge_board, encoder_jumpers_closed};
use vibes_behaviour::{behaviour, expect, Test};

/// The isolator input the P2 reads as P9 — the receiver's channel-1 output.
const OUTPUT: &str = "EdgeBoard.Net-(IC16-INA)";
const A_PLUS: &str = "EdgeBoard./MaD_Edge_Sheet3/A+";

/// One case at a time: the virtual clock is process-global, and every run
/// re-anchors it in stepped mode (`TESTING.md` rule 5).
static SUITE_LOCK: Mutex<()> = Mutex::new(());

fn suite_lock() -> MutexGuard<'static, ()> {
    SUITE_LOCK.lock().unwrap_or_else(|poisoned| {
        SUITE_LOCK.clear_poison();
        poisoned.into_inner()
    })
}

/// The virtual time a run hands the engine before it reads: 1 ms. Longer
/// than every instant a part on the Edge board arms — an indicator
/// inverter's `t_pd` (SN74LVC1G14 §5.6, 4.6 ns max, 5 ns on the wheel:
/// [`LVC1G14_T_PD_NS`]) and the longest start-up any of its parts declares,
/// the UCC12040's `VISO` rise (SNVSBO5B §6.9, 750 µs typ:
/// [`UCC12040_RISE_NS`]) — so the read is the system at rest. The window
/// is the harness's, not a part's (`isolation_bridge.rs` and `edgeboard.rs`
/// derive the same one for the same board), asserted past the longest chain
/// the board arms: the rise, then an indicator's `t_pd` on the side it
/// powers.
const SETTLE_NS: u64 = 1_000_000;
const _: () = assert!(SETTLE_NS > UCC12040_RISE_NS + LVC1G14_T_PD_NS);

/// One run of `case_1_forward` from `edgeboard.rs` — A+ driven high, A-
/// pulled to 0 by the closed jumper, so the receiver should decode a logic
/// high — read at its first settled instant: what the receiver output came
/// to rest at, and the run's engine event log.
fn settle_once() -> (NetState, Vec<String>) {
    virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);
    let scenario = encoder_jumpers_closed(Scenario::default(), "EdgeBoard").net_stuck(A_PLUS, 3.3);
    let system = System::new()
        .board("EdgeBoard", edge_board())
        .harness(bench_rails("EdgeBoard"))
        .scenario(scenario)
        .event_log()
        .hold_time()
        .start()
        .expect("the servo-domain system starts");
    // Time is held until the thread has registered, so the settled instant
    // is the same every run.
    let actor = virtual_clock::register_actor("rs422-determinism-run");
    system.release_time();
    virtual_clock::wait_virtual_ns(SETTLE_NS);

    let got = system
        .net_state(OUTPUT)
        .unwrap_or_else(|| panic!("net {OUTPUT} exists"));
    let log = system.event_log().normalized();
    let stalled: Vec<Finding> = system
        .findings()
        .into_iter()
        .filter(|f| matches!(f, Finding::QuiescenceTimeout { .. }))
        .collect();
    assert!(
        stalled.is_empty(),
        "the engine stopped waiting for the run's thread, so the read may have \
         raced the system: {stalled:?}"
    );
    drop(actor);
    system.shutdown();
    (got, log)
}

#[test]
fn the_rs422_receiver_settles_the_same_way_every_time() {
    behaviour!(Test {
        id: "rs422.settles-the-same-way",
        covers: Some("board/src/engine.rs#EngineCore::run_stepped_iteration"),
        given: "one RS-422 scenario, A+ at 3.3 volts and A- at ground, started twelve \
                times in one process, each run read once settled, on a clock that moves \
                only while the test waits",
    });
    expect!(
        "agree-across-runs",
        "every run finds the receiver output in the same state",
        "a simulation with no firmware in it, a board and scripted stimulus alone, must \
         reach one steady state for one input whatever the thread scheduling"
    );
    expect!(
        "decodes-high",
        "the receiver output is driven high",
        "a high A+ leg against a grounded A- leg is a differential high"
    );

    const RUNS: usize = 12;

    let _suite = suite_lock();
    let mut outcomes: Vec<(NetState, Vec<String>)> = Vec::with_capacity(RUNS);
    for _ in 0..RUNS {
        outcomes.push(settle_once());
    }

    let states: Vec<NetState> = outcomes.iter().map(|(s, _)| *s).collect();
    let first = states[0];
    if states.iter().all(|s| *s == first) {
        assert_eq!(
            first,
            NetState::Driven(Level::High),
            "every run in this process agreed, but on the WRONG state — so the \
             solve is stable and simply incorrect for this scenario, not racy"
        );
        return;
    }

    // Disagreement inside one process, at a settled instant: show where the
    // event traces diverge, which is the whole reason the engine keeps a log.
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
        "the same declared scenario settled differently within ONE process, each \
         run read at a settled instant: {states:?}{detail}"
    );
}
