//! Determinism Oracle 1 in anger: run one case N times, normalize the engine
//! event logs, and compare them (`DETERMINISM.md`, "Proving it: determinism
//! testing" → "Tests").
//!
//! **One counter.** Every case is a discrete-event run: `virtual_us` is the
//! value the engine set. The full projection ([`EventLog::normalized`]),
//! `v_us` included, must be **identical across N runs**, in-process *and*
//! across separate processes, and must match a blessed golden trace.
//!
//! Pacing (`init(speed > 0)`) only sleeps the host after a jump; it must not
//! change virtual timestamps. [`wake_ladder_timestamps_match_paced_and_unpaced`]
//! asserts that.
//!
//! Its own test binary per `TESTING.md` rule 5, and every case here takes
//! [`suite_lock`]: the cases pin the process-global virtual clock *and its
//! mode*, so two of them running concurrently would be measuring each other.
//!
//! Deliberately excluded, and why: firmware, real file descriptors, and the
//! host PTY. `DETERMINISM.md` T1 §4 shows all three are structurally outside
//! any barrier a scheduler can draw, and the cases here are exactly the
//! "scripted inside the emulator process" class the doc says D1 makes exactly
//! repeatable.

use rstest::rstest;
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};

use embsim_board::uart::{UartDecoder, UartFraming};
use embsim_board::{
    AttachError, Board, Component, ComponentNetIo, EventLog, Harness, PartRegistry, PinDecl,
    PinHandle, PinKind, Scenario, SerialLevelBridge, System, SystemHandle, TheveninDrive,
};
use embsim_core::virtual_clock::{self, ClockMode};

/// How many times each case runs in-process. `DETERMINISM.md` specifies N = 5.
const RUNS: usize = 5;

/// How many separate processes the cross-process identity case compares.
const PROCESS_RUNS: usize = 3;

// ============================================================
// Suite serialization + clock mode guards
// ============================================================

/// Every case in this binary re-anchors the process-global virtual clock and
/// switches its mode; they must not overlap.
static SUITE_LOCK: Mutex<()> = Mutex::new(());

fn suite_lock() -> MutexGuard<'static, ()> {
    SUITE_LOCK.lock().unwrap_or_else(|poisoned| {
        SUITE_LOCK.clear_poison();
        poisoned.into_inner()
    })
}

/// Puts the process-global clock in unpaced mode for the lifetime of the guard
/// and restores paced `init(1.0)` on the way out, panic or not.
struct Stepped;

impl Stepped {
    fn enter() -> Self {
        virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);
        Self
    }
}

impl Drop for Stepped {
    fn drop(&mut self) {
        virtual_clock::init(1.0, 1_000_000);
    }
}

fn wait_for(mut pred: impl FnMut() -> bool, timeout: Duration) -> bool {
    let start = Instant::now();
    while start.elapsed() < timeout {
        if pred() {
            return true;
        }
        // A *test-side* wall sleep, deliberately not a `virtual_clock` wait:
        // this thread is not part of the simulation and must not park on a
        // clock the engine is stepping.
        std::thread::sleep(Duration::from_millis(1));
    }
    pred()
}

type PinSlot = Arc<Mutex<Option<PinHandle>>>;
type BridgeSlot = Arc<Mutex<Option<Arc<SerialLevelBridge>>>>;
type ByteLog = Arc<Mutex<Vec<u8>>>;

// ============================================================
// Components
// ============================================================

/// A two-terminal analog driver whose pins the test scripts from the outside —
/// so the stimulus order is the *test thread's* order, not a race between
/// component threads.
///
/// `DETERMINISM.md` is emphatic that `EngineLink::next_drive_seq` is "a racing
/// `fetch_add` across component threads", so enqueue-seq makes the applied
/// order consistent without making it the same order twice. A single-threaded
/// stimulus removes exactly that race and nothing else.
struct ScriptedDriver {
    pins: [PinDecl; 2],
    a: PinSlot,
    b: PinSlot,
}

impl ScriptedDriver {
    fn new(a: PinSlot, b: PinSlot) -> Self {
        Self {
            pins: [analog_pin("A"), analog_pin("B")],
            a,
            b,
        }
    }
}

impl Component for ScriptedDriver {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        *self.a.lock().unwrap() = Some(io.pin("A")?);
        *self.b.lock().unwrap() = Some(io.pin("B")?);
        Ok(())
    }
}

/// An analog sense terminal — present so every drive produces a sense
/// delivery, and so the cluster escalates to the MNA solver (the float path
/// the `cluster_sources` ordering fix protects).
struct Probe {
    pins: [PinDecl; 1],
    seen: Arc<Mutex<usize>>,
}

impl Component for Probe {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let seen = Arc::clone(&self.seen);
        io.on_sense("P", move |_| *seen.lock().unwrap() += 1)?;
        Ok(())
    }
}

const fn analog_pin(name: &'static str) -> PinDecl {
    PinDecl {
        number: name,
        name: None,
        kind: PinKind::Analog,
        stream: None,
        drive_impedance: None,
    }
}

/// A component whose **only** stimulus is the engine's timer wheel: it arms a
/// ladder of `TICKS` one-shot wakeups exactly `PERIOD_US` apart and drives its
/// pin to a fresh voltage on each.
///
/// This is the case that makes the D1 contrast visible. The *deadline* sequence
/// is fixed arithmetic in both modes, so both produce the same records in the
/// same order; what differs is the `v_us` each record is stamped with. In
/// free-running that is sampled wall time (it drifts every run); in stepped it
/// is the instant the engine advanced to, so it is `anchor + k · PERIOD_US`
/// exactly. A one-shot ladder rather than `schedule_every` on purpose: a
/// periodic entry never empties the wheel, and a stepped engine would then
/// advance virtual time forever instead of terminating.
struct WakeLadder {
    pins: [PinDecl; 1],
    ticks: usize,
    period_us: u64,
    fired: Arc<AtomicUsize>,
}

impl Component for WakeLadder {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let pin = io.pin("P")?;
        let fired = Arc::clone(&self.fired);
        let ticks = self.ticks;
        let period = self.period_us;
        // Anchor the ladder at the virtual instant of attach. In stepped mode
        // that is 0 for every run: the engine holds time until the whole system
        // has attached and started (`engine::Command::ReleaseTime`).
        let anchor = virtual_clock::virtual_us();
        let next = Arc::new(AtomicU64::new(anchor + period));
        let rearm = io.clone();
        let schedule = Arc::clone(&next);
        io.on_wake(move |_now_us| {
            let tick = fired.fetch_add(1, Ordering::SeqCst) + 1;
            // A fresh solved voltage per tick, so no drive is swallowed by the
            // sense change gate.
            pin.set_drive(Some(TheveninDrive {
                volts: 0.5 + 0.25 * tick as f64,
                impedance: 17.0 + tick as f64,
            }));
            if tick < ticks {
                rearm.schedule_at(schedule.fetch_add(period, Ordering::SeqCst) + period);
            }
        });
        io.schedule_at(next.load(Ordering::SeqCst));
        Ok(())
    }
}

/// A one-net transmitter→receiver serial link on a *netlist* board.
const LINK_NETLIST: &str = r#"(export (version "E")
  (components
    (comp (ref "MCU")
      (value "LinkTx")
      (libsource (lib "test") (part "LINK_TX") (description "")))
    (comp (ref "SNS")
      (value "LinkRx")
      (libsource (lib "test") (part "LINK_RX") (description ""))))
  (nets
    (net (code "1") (name "LINK") (class "Default")
      (node (ref "MCU") (pin "1") (pinfunction "TX") (pintype "output"))
      (node (ref "SNS") (pin "1") (pinfunction "RX") (pintype "input")))))"#;

/// Framing for the link: 8N1 at 115.2 kbaud, so one bit is 8680 ns.
fn link_framing() -> UartFraming {
    UartFraming::new_8n1(115_200)
}

/// Single-pin transmitter: frames bytes onto its pin as edges.
struct LinkTx {
    pins: [PinDecl; 1],
    bridge: BridgeSlot,
}

impl LinkTx {
    fn new(bridge: BridgeSlot) -> Self {
        Self {
            pins: [PinDecl {
                number: "1",
                name: Some("TX"),
                kind: PinKind::DigitalOut,
                stream: None,
                drive_impedance: None,
            }],
            bridge,
        }
    }
}

impl Component for LinkTx {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let bridge = Arc::new(SerialLevelBridge::new(
            link_framing(),
            io.pin("TX")?,
            io.clone(),
            Arc::new(AtomicBool::new(false)),
        ));
        bridge.idle();
        {
            let bridge = Arc::clone(&bridge);
            io.on_wake_ns(move |now_ns| {
                let _ = bridge.service(now_ns);
            });
        }
        *self.bridge.lock().unwrap() = Some(bridge);
        Ok(())
    }
}

/// Single-pin receiver: deframes the edges back into bytes.
struct LinkRx {
    pins: [PinDecl; 1],
    rx: ByteLog,
}

impl LinkRx {
    fn new(rx: ByteLog) -> Self {
        Self {
            pins: [PinDecl {
                number: "1",
                name: Some("RX"),
                kind: PinKind::DigitalIn,
                stream: None,
                drive_impedance: None,
            }],
            rx,
        }
    }
}

impl Component for LinkRx {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let decoder = Arc::new(Mutex::new(UartDecoder::new(link_framing())));
        {
            let (log, decoder, io_arm) = (Arc::clone(&self.rx), Arc::clone(&decoder), io.clone());
            io.on_sense("RX", move |state| {
                let now = virtual_clock::virtual_ns();
                let mut rx = decoder.lock().unwrap();
                if let Some(level) = embsim_board::level_of(state) {
                    rx.on_level(level, now);
                }
                while let Some(Ok(byte)) = rx.poll(now) {
                    log.lock().unwrap().push(byte);
                }
                if let Some(at) = rx.frame_deadline_ns() {
                    io_arm.schedule_at_ns(at);
                }
            })?;
        }
        {
            let log = Arc::clone(&self.rx);
            io.on_wake_ns(move |now_ns| {
                let mut rx = decoder.lock().unwrap();
                while let Some(Ok(byte)) = rx.poll(now_ns) {
                    log.lock().unwrap().push(byte);
                }
            });
        }
        Ok(())
    }
}

// ============================================================
// Scenario bodies
// ============================================================

/// Build the analog scenario and drive a fixed script through it from this
/// thread. Returns the engine event log after the system has quiesced.
fn analog_scenario(scenario: Scenario) -> EventLog {
    let a: PinSlot = Arc::new(Mutex::new(None));
    let b: PinSlot = Arc::new(Mutex::new(None));
    let seen = Arc::new(Mutex::new(0usize));

    let harness = Harness::new()
        // Both driver terminals and the probe land on ONE node, so every
        // drive changes a solved voltage and the sources accumulate into the
        // same MNA supernode.
        .connect_str("DRV.A", "PROBE.P")
        .expect("endpoints parse")
        .connect_str("DRV.B", "PROBE.P")
        .expect("endpoints parse");

    let system: SystemHandle = System::new()
        .component(
            "DRV",
            Box::new(ScriptedDriver::new(Arc::clone(&a), Arc::clone(&b))),
        )
        .component(
            "PROBE",
            Box::new(Probe {
                pins: [analog_pin("P")],
                seen: Arc::clone(&seen),
            }),
        )
        .harness(harness)
        .scenario(scenario)
        .event_log()
        .start()
        .expect("analog system starts");

    let log = system.event_log();
    let a = a.lock().unwrap().clone().expect("A attached");
    let b = b.lock().unwrap().clone().expect("B attached");

    // A fixed script, issued from ONE thread: 12 drives whose solved node
    // voltage changes every step (so no drive is swallowed by the sense
    // change gate).
    let mut expected_drives = 0usize;
    for step in 0..6 {
        a.set_drive(Some(TheveninDrive {
            volts: 3.3 - 0.11 * step as f64,
            impedance: 23.7 + step as f64,
        }));
        b.set_drive(Some(TheveninDrive {
            volts: 0.19 + 0.07 * step as f64,
            impedance: 31.1 + step as f64,
        }));
        expected_drives += 2;
    }

    // Quiesce on the oracle itself: every scripted drive must have been
    // applied before the log is read, or the comparison would be racing the
    // engine rather than measuring it.
    let applied = |log: &EventLog| {
        log.records()
            .iter()
            .filter(|r| matches!(r.event, embsim_board::EngineEvent::DriveApplied { .. }))
            .count()
    };
    assert!(
        wait_for(|| applied(&log) >= expected_drives, Duration::from_secs(10)),
        "engine applied {} of {expected_drives} scripted drives",
        applied(&log)
    );
    assert!(system.engine_is_alive(), "engine must survive the script");
    system.shutdown();
    log
}

/// Build the one-net serial link, write a fixed payload, and return the event
/// log once every surviving byte has been delivered.
fn stream_scenario(scenario: Scenario, payload: &[u8], expected_bytes: usize) -> EventLog {
    let tx: BridgeSlot = Arc::new(Mutex::new(None));
    let rx: ByteLog = Arc::new(Mutex::new(Vec::new()));

    let mut registry = PartRegistry::new();
    {
        let tx = Arc::clone(&tx);
        registry.register("LINK_TX", move |_decl| {
            Box::new(LinkTx::new(Arc::clone(&tx)))
        });
    }
    {
        let rx = Arc::clone(&rx);
        registry.register("LINK_RX", move |_decl| {
            Box::new(LinkRx::new(Arc::clone(&rx)))
        });
    }
    let board = Board::from_netlist(
        embsim_board::netlist::parse(LINK_NETLIST).expect("link netlist parses"),
        &registry,
    )
    .expect("link board builds");

    let system = System::new()
        .board("Rig", board)
        .scenario(scenario)
        .event_log()
        .start()
        .expect("stream system starts");

    let log = system.event_log();
    tx.lock()
        .unwrap()
        .as_ref()
        .expect("MCU.TX attached")
        .transmit(payload);

    assert!(
        wait_for(
            || rx.lock().unwrap().len() >= expected_bytes,
            Duration::from_secs(10)
        ),
        "SNS received {} of {expected_bytes} expected bytes",
        rx.lock().unwrap().len()
    );
    system.shutdown();
    log
}

/// Ladder period and tick count — 8 wakeups, 1 ms of virtual time apart.
const LADDER_PERIOD_US: u64 = 1_000;
const LADDER_TICKS: usize = 8;

/// Build the wake-ladder scenario and return its log once every tick has fired.
fn wake_ladder_scenario() -> EventLog {
    let fired = Arc::new(AtomicUsize::new(0));
    let seen = Arc::new(Mutex::new(0usize));

    let harness = Harness::new()
        .connect_str("LADDER.P", "PROBE.P")
        .expect("endpoints parse");

    let system = System::new()
        .component(
            "LADDER",
            Box::new(WakeLadder {
                pins: [analog_pin("P")],
                ticks: LADDER_TICKS,
                period_us: LADDER_PERIOD_US,
                fired: Arc::clone(&fired),
            }),
        )
        .component(
            "PROBE",
            Box::new(Probe {
                pins: [analog_pin("P")],
                seen: Arc::clone(&seen),
            }),
        )
        .harness(harness)
        .event_log()
        .start()
        .expect("ladder system starts");

    let log = system.event_log();
    assert!(
        wait_for(
            || fired.load(Ordering::SeqCst) >= LADDER_TICKS,
            Duration::from_secs(10)
        ),
        "ladder fired {} of {LADDER_TICKS} wakeups",
        fired.load(Ordering::SeqCst)
    );
    // The last tick's drive is enqueued from the wake callback, so let the
    // engine apply it before the log is read.
    let drives = |log: &EventLog| {
        log.records()
            .iter()
            .filter(|r| matches!(r.event, embsim_board::EngineEvent::DriveApplied { .. }))
            .count()
    };
    assert!(
        wait_for(|| drives(&log) >= LADDER_TICKS, Duration::from_secs(10)),
        "engine applied {} of {LADDER_TICKS} ladder drives",
        drives(&log)
    );
    assert!(system.engine_is_alive(), "engine must survive the ladder");
    system.shutdown();
    log
}

// ============================================================
// The case matrix
// ============================================================

/// Every case, by the name used for its golden fixture and for the
/// cross-process dump protocol. One dispatch, so a golden, an in-process run,
/// and a subprocess run can never drift apart.
const CASES: &[&str] = &[
    "nominal_analog_cluster",
    "net_stuck_shared_node",
    "serial_levels",
    "wake_ladder",
];

/// Run one named case once and return its engine event log. The clock must
/// already be initialized in the desired mode.
fn run_case(name: &str) -> EventLog {
    match name {
        "nominal_analog_cluster" => analog_scenario(Scenario::default()),
        "net_stuck_shared_node" => analog_scenario(Scenario::default().net_stuck("DRV.A", 0.0)),
        "serial_levels" => {
            // Four bytes, framed onto the net one bit at a time. This is the
            // case that pins the *bit clock*: every edge is a drive, a
            // resolution and a sense, and every bit boundary is a wheel entry.
            let payload = [0x00u8, 0x5A, 0xFF, 0xA5];
            stream_scenario(Scenario::default(), &payload, payload.len())
        }
        "wake_ladder" => wake_ladder_scenario(),
        other => panic!("unknown determinism case {other:?}"),
    }
}

/// Free-running speed scale for a case. 50x keeps the 115.2 kbaud bit clock
/// of a four-byte payload sub-millisecond in wall time; everything else runs
/// at real time.
fn free_running_scale(name: &str) -> f64 {
    match name {
        "serial_levels" => 50.0,
        _ => 1.0,
    }
}

// ============================================================
// Comparison + reporting
// ============================================================

/// Where two normalized logs first disagree, and how long each was.
struct Divergence {
    first_index: Option<usize>,
    len_a: usize,
    len_b: usize,
}

fn diverge(a: &[String], b: &[String]) -> Divergence {
    let first_index = a
        .iter()
        .zip(b.iter())
        .position(|(x, y)| x != y)
        .or_else(|| (a.len() != b.len()).then_some(a.len().min(b.len())));
    Divergence {
        first_index,
        len_a: a.len(),
        len_b: b.len(),
    }
}

/// Assert every run agrees, and panic with the first disagreeing pair of lines
/// when they do not. `projection` names what is being compared so the failure
/// says which promise broke.
fn assert_identical(case: &str, projection: &str, logs: &[Vec<String>]) {
    let baseline = &logs[0];
    assert!(
        !baseline.is_empty(),
        "{case}: the event log recorded nothing — the oracle is not wired up, \
         so every comparison below would be vacuously true"
    );
    for (run, log) in logs.iter().enumerate().skip(1) {
        let d = diverge(baseline, log);
        if let Some(index) = d.first_index {
            panic!(
                "{case}: run {run} diverged from run 0 in the {projection} of engine \
                 events.\n\
                 first difference at record {index} (run 0 has {} records, run {run} has {})\n\
                 run 0:     {}\n\
                 run {run}: {}",
                d.len_a,
                d.len_b,
                baseline.get(index).map_or("<end of log>", String::as_str),
                log.get(index).map_or("<end of log>", String::as_str),
            );
        }
    }
}

/// Report — never assert — how far the *timestamped* projections drift. This is
/// the measured free-running baseline; stepped mode is what drives it to zero.
fn report_timestamp_divergence(case: &str, full: &[Vec<String>], stamps: &[Vec<u64>]) {
    let baseline = &full[0];
    let mut identical = 0usize;
    let mut first_diffs: Vec<usize> = Vec::new();
    for run in full.iter().skip(1) {
        match diverge(baseline, run).first_index {
            None => identical += 1,
            Some(index) => first_diffs.push(index),
        }
    }
    let spans: Vec<u64> = stamps
        .iter()
        .map(|s| s.last().copied().unwrap_or(0))
        .collect();
    let (lo, hi) = (
        spans.iter().copied().min().unwrap_or(0),
        spans.iter().copied().max().unwrap_or(0),
    );
    println!(
        "[free-running] {case}: {} records/run; {}/{} timestamped logs identical to run 0; \
         first divergence at record {:?}; final v_us across runs {lo}..{hi} (spread {} µs)",
        baseline.len(),
        identical,
        full.len() - 1,
        first_diffs,
        hi.saturating_sub(lo),
    );
}

/// One case, `RUNS` times, in **stepped** mode: the full timestamped
/// projection must be identical. Returns the (single, canonical) log.
fn stepped_matrix(case: &str) -> Vec<String> {
    let mut full: Vec<Vec<String>> = Vec::new();
    for _ in 0..RUNS {
        let _stepped = Stepped::enter();
        full.push(run_case(case).normalized());
    }
    assert_identical(case, "FULL timestamped projection", &full);
    println!(
        "[stepped] {case}: {} records/run, {}/{} runs byte-identical including every v_us",
        full[0].len(),
        RUNS,
        RUNS
    );
    full.remove(0)
}

/// One case, `RUNS` times, in **free-running** mode: assert the timestamp-free
/// projection, report the timestamped one.
fn free_running_matrix(case: &str) {
    let scale = free_running_scale(case);
    let mut shapes: Vec<Vec<String>> = Vec::new();
    let mut full: Vec<Vec<String>> = Vec::new();
    let mut stamps: Vec<Vec<u64>> = Vec::new();
    for _ in 0..RUNS {
        virtual_clock::init(scale, 1_000_000);
        let log = run_case(case);
        shapes.push(log.normalized_shape());
        full.push(log.normalized());
        stamps.push(log.records().iter().map(|r| r.v_us).collect());
    }
    assert_identical(case, "ORDER (timestamp-free projection)", &shapes);
    report_timestamp_divergence(case, &full, &stamps);
}

// ============================================================
// N-run identity, both modes
// ============================================================

/// **The D1 assertion.** In stepped mode the whole normalized log — order and
/// every virtual timestamp — is identical across N runs of the same case.
#[rstest]
#[case::nominal("nominal_analog_cluster")]
#[case::net_stuck("net_stuck_shared_node")]
#[case::serial_levels("serial_levels")]
#[case::wake_ladder("wake_ladder")]
fn stepped_logs_are_identical_across_runs(#[case] case: &str) {
    let _suite = suite_lock();
    stepped_matrix(case);
}

/// Paced (`init(speed > 0)`) runs must match on **order**. Virtual timestamps
/// are still the counter, so they should match too; the printout stays as a
/// measurement if a future regression reintroduces wall coupling.
#[rstest]
#[case::nominal("nominal_analog_cluster")]
#[case::net_stuck("net_stuck_shared_node")]
#[case::serial_levels("serial_levels")]
#[case::wake_ladder("wake_ladder")]
fn paced_order_is_reproducible(#[case] case: &str) {
    let _suite = suite_lock();
    free_running_matrix(case);
}

/// One counter: paced (`init(1.0)`) and unpaced (`init(0.0)`) produce the
/// same virtual timestamps. Pacing only sleeps the host after a jump.
#[rstest]
fn wake_ladder_timestamps_match_paced_and_unpaced() {
    let _suite = suite_lock();

    virtual_clock::init(1.0, 1_000_000);
    let paced = run_case("wake_ladder").normalized();
    virtual_clock::init(0.0, 1_000_000);
    let unpaced = run_case("wake_ladder").normalized();
    assert_identical(
        "wake_ladder (paced vs unpaced)",
        "FULL timestamped projection",
        &[paced, unpaced],
    );

    let stepped_a = {
        let _stepped = Stepped::enter();
        run_case("wake_ladder").normalized()
    };
    let stepped_b = {
        let _stepped = Stepped::enter();
        run_case("wake_ladder").normalized()
    };
    assert_identical(
        "wake_ladder (unpaced repeat)",
        "FULL timestamped projection",
        &[stepped_a.clone(), stepped_b],
    );

    // The k-th wake lands at exactly k · period — integers the engine chose.
    let wake_stamps: Vec<u64> = stepped_a
        .iter()
        .filter(|line| line.contains(" wake component="))
        .map(|line| {
            line.split_whitespace()
                .find_map(|token| token.strip_prefix("v="))
                .and_then(|v| v.parse::<u64>().ok())
                .expect("every normalized line carries v=")
        })
        .collect();
    let expected: Vec<u64> = (1..=LADDER_TICKS as u64)
        .map(|k| k * LADDER_PERIOD_US)
        .collect();
    assert_eq!(
        wake_stamps, expected,
        "wakeups must land exactly on their scheduled deadlines"
    );

    println!("[one-counter] wake_ladder wakes at {expected:?} µs paced and unpaced");
}

// ============================================================
// Golden traces
// ============================================================

/// Directory of blessed normalized traces. `DETERMINISM.md` sketched these as
/// `*.jsonl`; the normalized form the D0 event log actually produces is
/// line-oriented plain text with no serializer dependency (`event_log.rs`,
/// "Normalization"), so they are `*.trace`.
fn golden_path(case: &str) -> std::path::PathBuf {
    std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures/traces")
        .join(format!("{case}.trace"))
}

/// Golden-trace regression: a stepped run must reproduce the blessed trace
/// byte for byte. Rewrite them with `EMBSIM_BLESS=1 cargo test -p embsim-board
/// --test determinism`.
///
/// This is what turns "did this engine/model change alter the wire behavior?"
/// into a diff, and it is only possible now: at D0 the timestamps were sampled
/// wall time, so there was nothing stable to bless.
#[rstest]
#[case::nominal("nominal_analog_cluster")]
#[case::net_stuck("net_stuck_shared_node")]
#[case::serial_levels("serial_levels")]
#[case::wake_ladder("wake_ladder")]
fn stepped_logs_match_their_golden_trace(#[case] case: &str) {
    let _suite = suite_lock();
    let actual = {
        let _stepped = Stepped::enter();
        run_case(case).normalized()
    };
    assert!(
        !actual.is_empty(),
        "{case}: refusing to compare an empty log against a golden"
    );
    let path = golden_path(case);

    if std::env::var_os("EMBSIM_BLESS").is_some() {
        std::fs::create_dir_all(path.parent().expect("golden path has a parent"))
            .expect("golden directory is writable");
        std::fs::write(&path, format!("{}\n", actual.join("\n"))).expect("golden is writable");
        println!(
            "[bless] wrote {} records to {}",
            actual.len(),
            path.display()
        );
        return;
    }

    let blessed = std::fs::read_to_string(&path).unwrap_or_else(|error| {
        panic!(
            "{case}: golden trace {} could not be read ({error}); \
             bless it with EMBSIM_BLESS=1",
            path.display()
        )
    });
    let expected: Vec<String> = blessed.lines().map(str::to_string).collect();
    let d = diverge(&expected, &actual);
    if let Some(index) = d.first_index {
        panic!(
            "{case}: the stepped trace no longer matches {}.\n\
             first difference at record {index} (golden has {} records, run has {})\n\
             golden: {}\n\
             actual: {}\n\
             If the change is intended, re-bless with EMBSIM_BLESS=1.",
            path.display(),
            d.len_a,
            d.len_b,
            expected.get(index).map_or("<end of log>", String::as_str),
            actual.get(index).map_or("<end of log>", String::as_str),
        );
    }
}

// ============================================================
// Cross-process identity
// ============================================================

const DUMP_ENV: &str = "EMBSIM_DETERMINISM_DUMP";
const DUMP_BEGIN: &str = "---embsim-determinism-dump-begin---";
const DUMP_END: &str = "---embsim-determinism-dump-end---";

/// Subprocess entry point. Inert unless [`DUMP_ENV`] names a case, so a normal
/// `cargo test` run pays nothing for it.
///
/// This is how the suite gets `DETERMINISM.md`'s "CI must run the binary N
/// times as separate processes" without any CI-side scripting: the test binary
/// re-executes *itself*, filtered to this one case. A fresh process means fresh
/// `HashMap` seeds and a fresh clock, which is the class of nondeterminism
/// in-process repetition cannot reach for long-lived maps.
#[rstest]
fn dump_case_for_subprocess() {
    let Ok(case) = std::env::var(DUMP_ENV) else {
        return;
    };
    let _suite = suite_lock();
    let lines = {
        let _stepped = Stepped::enter();
        run_case(&case).normalized()
    };
    println!("{DUMP_BEGIN}");
    for line in &lines {
        println!("{line}");
    }
    println!("{DUMP_END}");
}

/// Run this test binary again, in a fresh process, and read back the normalized
/// stepped log it dumps.
fn dump_via_subprocess(case: &str) -> Vec<String> {
    let exe = std::env::current_exe().expect("test binary path");
    let output = std::process::Command::new(&exe)
        .args([
            "--exact",
            "dump_case_for_subprocess",
            "--nocapture",
            "--test-threads=1",
        ])
        .env(DUMP_ENV, case)
        .output()
        .unwrap_or_else(|error| panic!("could not re-exec {}: {error}", exe.display()));
    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(
        output.status.success(),
        "{case}: subprocess failed ({}):\n{stdout}\n{}",
        output.status,
        String::from_utf8_lossy(&output.stderr)
    );
    // Split on the markers as substrings, not whole lines: `--nocapture`
    // interleaves the harness's own progress output, so the opening marker
    // shares a line with `test dump_case_for_subprocess ...`.
    let body = stdout
        .split_once(DUMP_BEGIN)
        .and_then(|(_, rest)| rest.split_once(DUMP_END))
        .map(|(body, _)| body)
        .unwrap_or_else(|| panic!("{case}: subprocess printed no dump markers:\n{stdout}"));
    let lines: Vec<String> = body
        .lines()
        .filter(|line| !line.trim().is_empty())
        .map(str::to_string)
        .collect();
    assert!(
        !lines.is_empty(),
        "{case}: subprocess produced no dump between the markers:\n{stdout}"
    );
    lines
}

/// Stepped identity across **separate processes**, not just repeated runs in
/// one. Same assertion, different net: a fresh process re-seeds every
/// `HashMap`, so this is what catches hash order held in a long-lived engine
/// map (`DETERMINISM.md`, "Multi-process identity").
#[rstest]
#[case::nominal("nominal_analog_cluster")]
#[case::serial_levels("serial_levels")]
#[case::wake_ladder("wake_ladder")]
fn stepped_logs_are_identical_across_processes(#[case] case: &str) {
    let logs: Vec<Vec<String>> = (0..PROCESS_RUNS)
        .map(|_| dump_via_subprocess(case))
        .collect();
    assert_identical(
        case,
        "FULL timestamped projection (across processes)",
        &logs,
    );
    println!(
        "[stepped/multi-process] {case}: {} records, {PROCESS_RUNS}/{PROCESS_RUNS} \
         processes byte-identical",
        logs[0].len()
    );
}

// ============================================================
// Anti-vacuity guards
// ============================================================

/// Guard on the guard: [`assert_identical`] must actually reject a perturbed
/// log. Without this, a comparator bug (or a log that silently stopped
/// recording) would let every case above pass while testing nothing.
#[rstest]
#[case::reordered(0)]
#[case::truncated(1)]
#[case::mutated(2)]
fn the_comparison_rejects_a_perturbed_log(#[case] perturbation: usize) {
    let baseline: Vec<String> = (0..8)
        .map(|i| format!("{i} v={i} net_resolved net={i}"))
        .collect();
    let mut perturbed = baseline.clone();
    match perturbation {
        0 => perturbed.swap(2, 5),
        1 => {
            perturbed.truncate(6);
        }
        _ => perturbed[4] = "4 v=4 net_resolved net=99".to_string(),
    }
    let result = std::panic::catch_unwind(|| {
        assert_identical("synthetic", "test", &[baseline.clone(), perturbed.clone()]);
    });
    assert!(
        result.is_err(),
        "perturbation {perturbation} must be detected: {perturbed:?}"
    );
}

/// A one-µs timestamp difference must fail the *full* comparison and pass the
/// *shape* one — the exact distinction the two projections exist to draw, and
/// the reason stepped mode can assert something free-running cannot.
#[rstest]
fn the_full_comparison_catches_a_timestamp_difference_the_shape_one_ignores() {
    let a = vec!["0 v=1000 wake component=0".to_string()];
    let b = vec!["0 v=1001 wake component=0".to_string()];
    assert!(diverge(&a, &b).first_index.is_some());
    let shape_a = vec!["0 wake component=0".to_string()];
    let shape_b = vec!["0 wake component=0".to_string()];
    assert!(diverge(&shape_a, &shape_b).first_index.is_none());
}

/// An empty log is a failure, not a pass — the specific way this suite could
/// go quiet (event log never enabled, or the recording sites removed).
#[rstest]
fn an_empty_log_fails_rather_than_passing_vacuously() {
    let result = std::panic::catch_unwind(|| {
        assert_identical("synthetic", "test", &[Vec::new(), Vec::new()]);
    });
    assert!(result.is_err(), "an empty log must not pass");
}

/// Every case in [`CASES`] is covered by the stepped matrix, the free-running
/// matrix, and a golden — a new case added to the dispatch without wiring it
/// into the `#[case]` lists would otherwise be silently untested.
#[rstest]
fn every_named_case_has_a_golden() {
    for case in CASES {
        let path = golden_path(case);
        assert!(
            path.exists(),
            "case {case:?} has no golden trace at {}; bless it with EMBSIM_BLESS=1",
            path.display()
        );
    }
}

/// A stepped run must not have slept on the wall clock: every wait inside the
/// simulation goes through the virtual chokepoint, and a real sleep is
/// invisible to the quiescence barrier. This is `DETERMINISM.md`'s "wall-sleep
/// tripwire", asserted rather than merely logged.
#[rstest]
fn a_stepped_run_serves_no_wall_sleeps() {
    let _suite = suite_lock();
    let before = virtual_clock::stepped_wall_sleep_count();
    {
        let _stepped = Stepped::enter();
        let _ = run_case("wake_ladder");
        let _ = run_case("serial_levels");
    }
    assert_eq!(
        virtual_clock::stepped_wall_sleep_count(),
        before,
        "a stepped run served a real sleep: some wait escaped virtualization \
         (the error-level log names it)"
    );
}
