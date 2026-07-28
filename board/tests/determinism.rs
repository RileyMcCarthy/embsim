//! Determinism Oracle 1 in anger: run one scenario N times, normalize the
//! engine event logs, and compare them (`DETERMINISM.md`, "Proving it:
//! determinism testing" → "Tests" → N-run identity).
//!
//! **This binary is deliberately OBSERVATIONAL, and free-running.** Phase D0
//! ships no stepped clock, so `DETERMINISM.md`'s T0 baseline still holds: the
//! engine's event order is *consistent* but its virtual timestamps are sampled
//! wall time. The doc is explicit that the baseline must be **measured, not
//! guessed**, so this suite splits the comparison in two:
//!
//! - **Asserted** — the timestamp-free projection
//!   ([`EventLog::normalized_shape`]). Everything T0's "Already determined"
//!   list covers: drive/apply ordering, the net-state sequence, per-net sense
//!   delivery, stream FIFO, finding order. If any of that regresses, this
//!   binary fails.
//! - **Reported** — the full projection ([`EventLog::normalized`]), `v_us`
//!   included. Free-running mode is *expected* to diverge here, so divergence
//!   is printed with numbers (how many records, at which index they first
//!   disagree, the µs spread) instead of failing on wall-clock jitter. Phase D1
//!   turns those numbers into zero; until then they are the baseline.
//!
//! Its own test binary per `TESTING.md` rule 5: the scenarios pin the
//! process-global virtual clock, and the run-to-run comparison must not be
//! perturbed by an unrelated integration case sharing the process.
//!
//! Deliberately excluded, and why: firmware, real file descriptors, and the
//! host PTY. `DETERMINISM.md` T1 §4 shows all three are structurally outside
//! any barrier a scheduler can draw, and the scenarios here are exactly the
//! "scripted inside the emulator process" class the doc says D1 can make
//! exactly repeatable.

use rstest::rstest;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use embsim_board::component::StreamTx;
use embsim_board::{
    AttachError, Board, Component, ComponentNetIo, EventLog, Harness, PartRegistry, PinDecl,
    PinHandle, PinKind, Scenario, StreamDropPolicy, StreamRole, System, SystemHandle,
    TheveninDrive,
};
use embsim_core::virtual_clock;

/// How many times each scenario runs. `DETERMINISM.md` specifies N = 5.
const RUNS: usize = 5;

// ============================================================
// Shared plumbing
// ============================================================

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

type PinSlot = Arc<Mutex<Option<PinHandle>>>;
type TxSlot = Arc<Mutex<Option<StreamTx>>>;
type ByteLog = Arc<Mutex<Vec<u8>>>;

/// A two-terminal analog driver whose pins the test scripts from the outside —
/// so the stimulus order is the *test thread's* order, not a race between
/// component threads.
///
/// This is the crux of an honest D0 baseline. `DETERMINISM.md` is emphatic that
/// `EngineLink::next_drive_seq` is "a racing `fetch_add` across component
/// threads", so enqueue-seq makes the applied order consistent without making
/// it the same order twice. A single-threaded stimulus removes exactly that
/// race and nothing else, which isolates what T0 *does* determine.
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

/// A one-net producer→consumer serial link on a *netlist* board.
///
/// A netlist board rather than bench components on purpose: `stream_drop`
/// endpoints resolve through `split_board_pin`, which only knows board names,
/// so the byte-loss case below needs `Rig.MCU.1` rather than a bare bench
/// endpoint.
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

/// Single-pin stream producer.
struct LinkTx {
    pins: [PinDecl; 1],
    tx: TxSlot,
}

impl LinkTx {
    fn new(baud_hz: u32, tx: TxSlot) -> Self {
        Self {
            pins: [PinDecl {
                number: "1",
                name: Some("TX"),
                kind: PinKind::DigitalOut,
                stream: Some(StreamRole::Producer { baud_hz }),
                drive_impedance: None,
            }],
            tx,
        }
    }
}

impl Component for LinkTx {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        *self.tx.lock().unwrap() = Some(io.stream_tx("TX")?);
        Ok(())
    }
}

/// Single-pin stream consumer.
struct LinkRx {
    pins: [PinDecl; 1],
    rx: ByteLog,
}

impl LinkRx {
    fn new(baud_hz: u32, rx: ByteLog) -> Self {
        Self {
            pins: [PinDecl {
                number: "1",
                name: Some("RX"),
                kind: PinKind::DigitalIn,
                stream: Some(StreamRole::Consumer { baud_hz }),
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
        let log = Arc::clone(&self.rx);
        io.on_byte("RX", move |byte| log.lock().unwrap().push(byte))?;
        Ok(())
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

/// Assert every run agrees on the timestamp-free projection — the T0
/// "Already determined" set — and panic with the first disagreeing pair of
/// lines when they do not.
fn assert_shapes_identical(scenario: &str, shapes: &[Vec<String>]) {
    let baseline = &shapes[0];
    assert!(
        !baseline.is_empty(),
        "{scenario}: the event log recorded nothing — the oracle is not wired up, \
         so every comparison below would be vacuously true"
    );
    for (run, shape) in shapes.iter().enumerate().skip(1) {
        let d = diverge(baseline, shape);
        if let Some(index) = d.first_index {
            panic!(
                "{scenario}: run {run} diverged from run 0 in the ORDER of engine \
                 events, which `DETERMINISM.md` T0 lists as already determined.\n\
                 first difference at record {index} (run 0 has {} records, run {run} has {})\n\
                 run 0:     {}\n\
                 run {run}: {}",
                d.len_a,
                d.len_b,
                baseline.get(index).map_or("<end of log>", String::as_str),
                shape.get(index).map_or("<end of log>", String::as_str),
            );
        }
    }
}

/// Report — never assert — how far the *timestamped* projections drift. This
/// is the measured D0 baseline; Phase D1 is what drives it to zero.
fn report_timestamp_divergence(scenario: &str, full: &[Vec<String>], logs: &[Vec<u64>]) {
    let baseline = &full[0];
    let mut identical = 0usize;
    let mut first_diffs: Vec<usize> = Vec::new();
    for run in full.iter().skip(1) {
        match diverge(baseline, run).first_index {
            None => identical += 1,
            Some(index) => first_diffs.push(index),
        }
    }
    let spans: Vec<u64> = logs
        .iter()
        .map(|stamps| stamps.last().copied().unwrap_or(0))
        .collect();
    let (lo, hi) = (
        spans.iter().copied().min().unwrap_or(0),
        spans.iter().copied().max().unwrap_or(0),
    );
    println!(
        "[baseline] {scenario}: {} records/run; {}/{} timestamped logs identical to run 0; \
         first divergence at record {:?}; final v_us across runs {lo}..{hi} (spread {} µs)",
        baseline.len(),
        identical,
        full.len() - 1,
        first_diffs,
        hi.saturating_sub(lo),
    );
}

/// Run one scenario `RUNS` times, assert the invariant projection, report the
/// timestamped one.
///
/// `scale` re-anchors the process-global virtual clock **before every run**
/// (`virtual_clock::init` restarts virtual time at 0). Without that, run 1's
/// first record already carries a larger `v_us` than run 0's simply because
/// wall time kept moving, and the reported "first divergence" would be a
/// tautological 0 rather than a measurement of where jitter bites.
fn run_matrix(scenario: &str, scale: f64, mut once: impl FnMut() -> EventLog) {
    let mut shapes: Vec<Vec<String>> = Vec::new();
    let mut full: Vec<Vec<String>> = Vec::new();
    let mut stamps: Vec<Vec<u64>> = Vec::new();
    for _ in 0..RUNS {
        virtual_clock::init(scale, 1_000_000);
        let log = once();
        shapes.push(log.normalized_shape());
        full.push(log.normalized());
        stamps.push(log.records().iter().map(|r| r.v_us).collect());
    }
    assert_shapes_identical(scenario, &shapes);
    report_timestamp_divergence(scenario, &full, &stamps);
}

// ============================================================
// Scenario: scripted analog drives over a resistive cluster
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

/// The nominal analog cluster: 5 runs must agree exactly on the order of
/// drives, resolutions, and sense deliveries.
#[rstest]
fn nominal_analog_cluster_order_is_reproducible() {
    run_matrix("nominal analog cluster", 1.0, || {
        analog_scenario(Scenario::default())
    });
}

/// The same script with a `net_stuck` fault on the shared node — the fault
/// algebra path, which reaches contention projection and its findings.
#[rstest]
fn net_stuck_fault_order_is_reproducible() {
    run_matrix("net_stuck on the shared node", 1.0, || {
        analog_scenario(Scenario::default().net_stuck("DRV.A", 0.0))
    });
}

// ============================================================
// Scenario: paced serial stream, with and without byte loss
// ============================================================

/// Build the one-net serial link, write a fixed payload, and return the event
/// log once every surviving byte has been delivered.
fn stream_scenario(scenario: Scenario, payload: &[u8], expected_bytes: usize) -> EventLog {
    let tx: TxSlot = Arc::new(Mutex::new(None));
    let rx: ByteLog = Arc::new(Mutex::new(Vec::new()));

    let mut registry = PartRegistry::new();
    {
        let tx = Arc::clone(&tx);
        registry.register("LINK_TX", move |_decl| {
            Box::new(LinkTx::new(115_200, Arc::clone(&tx)))
        });
    }
    {
        let rx = Arc::clone(&rx);
        registry.register("LINK_RX", move |_decl| {
            Box::new(LinkRx::new(115_200, Arc::clone(&rx)))
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
        .write(payload);

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

/// A paced route: per-producer FIFO is the wire contract
/// (`DETERMINISM.md` T0, "Per-producer stream FIFO"), so the byte order in the
/// log must be identical across runs even though the *pacing deadlines* are
/// sampled from the free-running clock.
#[rstest]
fn paced_stream_byte_order_is_reproducible() {
    // 50x keeps 115.2 kbaud pacing of a 16-byte payload sub-millisecond in
    // wall time (the same convention `bench_component.rs` documents).
    let payload: Vec<u8> = (0..16u8).map(|i| i.wrapping_mul(17)).collect();
    run_matrix("paced stream, 16 bytes", 50.0, || {
        stream_scenario(Scenario::default(), &payload, payload.len())
    });
}

/// The `stream_drop(EveryNth(3))` fault: the drop *pattern* is a counter on
/// the endpoint, not a wall-clock decision, so which bytes survive must be
/// identical run to run.
#[rstest]
fn stream_drop_every_nth_pattern_is_reproducible() {
    let payload: Vec<u8> = (0..15u8).map(|i| i.wrapping_mul(17)).collect();
    // EveryNth(3) drops bytes 3, 6, 9, 12, 15 of 15 → 10 survive.
    let survivors = payload.len() - payload.len() / 3;
    run_matrix("paced stream, stream_drop(EveryNth(3))", 50.0, || {
        stream_scenario(
            Scenario::default().stream_drop("Rig.MCU.1", StreamDropPolicy::EveryNth(3)),
            &payload,
            survivors,
        )
    });
}

// ============================================================
// The comparison must not be vacuous
// ============================================================

/// Guard on the guard: [`assert_shapes_identical`] must actually reject a
/// perturbed log. Without this, a comparator bug (or a log that silently
/// stopped recording) would let every case above pass while testing nothing.
#[rstest]
#[case::reordered(0)]
#[case::truncated(1)]
#[case::mutated(2)]
fn the_order_comparison_rejects_a_perturbed_log(#[case] perturbation: usize) {
    let baseline: Vec<String> = (0..8)
        .map(|i| format!("{i} net_resolved net={i}"))
        .collect();
    let mut perturbed = baseline.clone();
    match perturbation {
        0 => perturbed.swap(2, 5),
        1 => {
            perturbed.truncate(6);
        }
        _ => perturbed[4] = "4 net_resolved net=99".to_string(),
    }
    let result = std::panic::catch_unwind(|| {
        assert_shapes_identical("synthetic", &[baseline.clone(), perturbed.clone()]);
    });
    assert!(
        result.is_err(),
        "perturbation {perturbation} must be detected: {perturbed:?}"
    );
}

/// An empty log is a failure, not a pass — the specific way this suite could
/// go quiet (event log never enabled, or the recording sites removed).
#[rstest]
fn an_empty_log_fails_rather_than_passing_vacuously() {
    let result = std::panic::catch_unwind(|| {
        assert_shapes_identical("synthetic", &[Vec::new(), Vec::new()]);
    });
    assert!(result.is_err(), "an empty log must not pass");
}

/// The negative control `DETERMINISM.md` asks for: in free-running mode the
/// *timestamped* projection is explicitly NOT required to match. Asserting
/// that it does would freeze the clock in the wrong mode and quietly stop the
/// suite from testing anything — so this records the expectation in code, and
/// prints whether today's runs happen to agree.
#[rstest]
fn free_running_timestamps_are_not_required_to_match() {
    virtual_clock::init(1.0, 1_000_000);
    let a = analog_scenario(Scenario::default()).normalized();
    let b = analog_scenario(Scenario::default()).normalized();
    let d = diverge(&a, &b);
    println!(
        "[negative control] free-running timestamped logs: first divergence {:?} \
         ({} vs {} records) — divergence here is expected, not a failure; \
         Phase D1's stepped clock is what makes it None",
        d.first_index, d.len_a, d.len_b
    );
    // The ORDER must still match, in both modes. That part is not optional.
    assert_shapes_identical(
        "negative control",
        &[
            analog_scenario(Scenario::default()).normalized_shape(),
            analog_scenario(Scenario::default()).normalized_shape(),
        ],
    );
}
