//! The node without QEMU: a fake guest on a socketpair with a stopwatch.
//!
//! Proves what the node exists for — the guest runs only while the board's
//! clock advances, a slice a quantum, and bytes cross the net in both
//! directions as levels — with nothing installed, on every CI run.
//!
//! Stepped (`TESTING.md` rule 9), its own binary: the clock stepped, the
//! system started with time held, the case's thread a registered actor
//! that hands the engine virtual time, so the board advances only while the
//! case is parked. Each node's line is on a rail of its own, 3.3 V on `VIO`
//! against 0 V on `GND`, as a project wires a host's real rail. A fake guest
//! writes in wall time, so the slice its bytes enter the line in is wherever
//! the run has reached: what is asserted of bytes is that they arrive, whole
//! and in order, none shed; the wall-time bound on such a wait is sized for
//! a hang ([`HANG`]). What is asserted of time is exact where the engine
//! owns it — the instants slices run at, the virtual time they book — and
//! bounded by a quantum where the host's scheduler owns it.

use std::io::{self, Read, Write};
use std::os::fd::{AsRawFd, RawFd};
use std::os::unix::net::UnixStream;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};

use embsim_board::{EndpointRef, Finding, Harness, System, SystemHandle};
use embsim_core::virtual_clock::{self, Actor, ClockMode};
use embsim_qemu::{Guest, NodeStats, QemuNode};
use rstest::rstest;
use vibes_behaviour::{behaviour, expect, Test};

// ============================================================
// Plumbing
// ============================================================

/// How long, in wall time, a stepped run may take to carry what a case
/// sends before the case calls it hung.
///
/// It is not a performance claim, and no runner's speed may decide it. The
/// wall time a burst takes is the engine's cost per edge times the edges the
/// burst is, plus a quantum of host time per quantum of line time: the
/// 256 KiB burst below is 1.31 s of line time (262 144 bytes × 10 bits at
/// 2 Mbaud), and CI's Ubuntu runner took some 70 s for it in a debug build
/// when the node was an actor. 300 s is four times that, is reached only by
/// a run that has stopped, and costs nothing when the bytes arrive: every
/// wait returns the moment its condition holds.
const HANG: Duration = Duration::from_secs(300);

/// The virtual time a case hands the engine between reads of a far end.
const SETTLE_NS: u64 = 1_000_000;

/// The quantum the metering cases run at: long beside the host's sleep and
/// scheduling granularity, so a slice's overrun stays well inside one.
const QUANTUM: Duration = Duration::from_millis(10);

/// One case at a time: the virtual clock is process-global, and each case
/// re-anchors it stepped.
static SUITE_LOCK: Mutex<()> = Mutex::new(());

fn suite_lock() -> MutexGuard<'static, ()> {
    SUITE_LOCK.lock().unwrap_or_else(|poisoned| {
        SUITE_LOCK.clear_poison();
        poisoned.into_inner()
    })
}

fn ep(endpoint: &str) -> EndpointRef {
    EndpointRef::parse(endpoint).expect("endpoint parses")
}

/// The nodes on one bench, each on its own 3.3 V rail, their lines wired
/// as `wires` says; started with time held, the case's thread registered
/// as an actor, and time released. The clock is re-anchored stepped first.
fn bench(nodes: Vec<(&str, QemuNode)>, wires: &[(&str, &str)]) -> (SystemHandle, Actor) {
    virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);
    let mut harness = Harness::new();
    for (from, to) in wires {
        harness = harness.connect(ep(from), ep(to));
    }
    let mut system = System::new();
    for (name, node) in nodes {
        harness = harness
            .power(
                ep(&format!("BENCH.{name}3V3")),
                ep(&format!("{name}.VIO")),
                3.3,
            )
            .power(
                ep(&format!("BENCH.{name}GND")),
                ep(&format!("{name}.GND")),
                0.0,
            );
        system = system.component(name, Box::new(node));
    }
    let system = system
        .harness(harness)
        .hold_time()
        .start()
        .expect("the bench starts");
    let actor = virtual_clock::register_actor("qemu-loopback-case");
    system.release_time();
    (system, actor)
}

/// The end of a case: no stall, the case's thread out of the clock, the
/// system down.
fn finish(system: SystemHandle, actor: Actor) {
    let stalled: Vec<Finding> = system
        .findings()
        .into_iter()
        .filter(|finding| matches!(finding, Finding::QuiescenceTimeout { .. }))
        .collect();
    assert!(
        stalled.is_empty(),
        "the engine stopped waiting for the case's thread: {stalled:?}"
    );
    drop(actor);
    system.shutdown();
}

/// A stopwatch that only runs between `resume` and `pause`: the fake guest's
/// clock.
#[derive(Default)]
struct Stopwatch {
    running_since: Option<Instant>,
    total: Duration,
    resumes: u64,
}

fn guest_total(clock: &Mutex<Stopwatch>) -> Duration {
    let c = clock.lock().unwrap();
    c.total + c.running_since.map(|s| s.elapsed()).unwrap_or_default()
}

/// A guest that is a socketpair and a stopwatch.
///
/// The near end mirrors [`embsim_qemu::QemuVm`]: detach drops it so the old
/// raw fd is dead (poll sees `POLLNVAL`); attach builds a fresh pair. The
/// test's far end lives in a shared slot so a reconnect can hand back a live
/// handle after the previous one has hung up.
struct FakeGuest {
    port: Option<UnixStream>,
    /// The test's end of the cable. Replaced on each attach.
    far: Arc<Mutex<Option<UnixStream>>>,
    clock: Arc<Mutex<Stopwatch>>,
    dropped: Arc<AtomicBool>,
    /// The pause after this many resumes takes this long to be answered,
    /// the guest running meanwhile, as a loaded host's `stop` does.
    slow_pause: Option<(u64, Duration)>,
    /// Resumes after this many fail, as a QEMU that has exited does.
    dies_after: Option<u64>,
}

impl Drop for FakeGuest {
    fn drop(&mut self) {
        self.dropped.store(true, Ordering::Relaxed);
    }
}

/// What a test keeps of a fake guest once the node holds it.
struct FarEnd {
    far: Arc<Mutex<Option<UnixStream>>>,
    clock: Arc<Mutex<Stopwatch>>,
    dropped: Arc<AtomicBool>,
}

impl FarEnd {
    /// Write `bytes` as the guest's program would, blocking.
    fn write(&self, bytes: &[u8]) {
        let mut slot = self.far.lock().unwrap();
        let far = slot.as_mut().expect("the cable is plugged in");
        far.set_nonblocking(false).unwrap();
        far.write_all(bytes).expect("the guest writes");
        far.set_nonblocking(true).unwrap();
    }

    /// Whatever the guest's port holds now.
    fn read_now(&self) -> Vec<u8> {
        let mut got = Vec::new();
        let mut slot = self.far.lock().unwrap();
        let Some(far) = slot.as_mut() else {
            return got;
        };
        let mut buf = [0u8; 4096];
        loop {
            match far.read(&mut buf) {
                Ok(0) => break,
                Ok(n) => got.extend_from_slice(&buf[..n]),
                Err(e) if e.kind() == io::ErrorKind::WouldBlock => break,
                Err(e) => panic!("reading the guest's port: {e}"),
            }
        }
        got
    }
}

impl FakeGuest {
    fn new() -> (Self, FarEnd) {
        let (near, far) = Self::open_pair();
        let far = Arc::new(Mutex::new(Some(far)));
        let clock = Arc::new(Mutex::new(Stopwatch::default()));
        let dropped = Arc::new(AtomicBool::new(false));
        (
            Self {
                port: Some(near),
                far: Arc::clone(&far),
                clock: Arc::clone(&clock),
                dropped: Arc::clone(&dropped),
                slow_pause: None,
                dies_after: None,
            },
            FarEnd {
                far,
                clock,
                dropped,
            },
        )
    }

    /// The pause that ends the `resume`th slice takes `delay` to answer.
    fn slow_pause_at(mut self, resume: u64, delay: Duration) -> Self {
        self.slow_pause = Some((resume, delay));
        self
    }

    /// Every resume after the first `resumes` fails.
    fn dying_after(mut self, resumes: u64) -> Self {
        self.dies_after = Some(resumes);
        self
    }

    fn open_pair() -> (UnixStream, UnixStream) {
        let (near, far) = UnixStream::pair().expect("socketpair");
        near.set_nonblocking(true).expect("nonblocking");
        far.set_nonblocking(true).expect("nonblocking");
        (near, far)
    }
}

impl Guest for FakeGuest {
    fn resume(&mut self) -> io::Result<()> {
        let mut c = self.clock.lock().unwrap();
        if self.dies_after.is_some_and(|after| c.resumes >= after) {
            return Err(io::Error::new(
                io::ErrorKind::BrokenPipe,
                "the fake guest's process has exited",
            ));
        }
        assert!(c.running_since.is_none(), "resumed twice without a pause");
        c.running_since = Some(Instant::now());
        c.resumes += 1;
        Ok(())
    }

    fn pause(&mut self) -> io::Result<()> {
        let resumes = self.clock.lock().unwrap().resumes;
        if let Some((_, delay)) = self.slow_pause.filter(|(at, _)| *at == resumes) {
            // The guest runs on while the `stop` is not answered.
            std::thread::sleep(delay);
        }
        let mut c = self.clock.lock().unwrap();
        if let Some(since) = c.running_since.take() {
            c.total += since.elapsed();
        }
        Ok(())
    }

    fn explain(&mut self, error: io::Error) -> io::Error {
        io::Error::new(error.kind(), format!("{error}; its log says why"))
    }

    fn serial_fd(&self) -> RawFd {
        // -1 while unplugged, matching QemuVm: poll(2) ignores a negative fd.
        self.port.as_ref().map_or(-1, |s| s.as_raw_fd())
    }

    fn serial_attached(&self) -> bool {
        self.port.is_some()
    }

    fn set_serial_attached(&mut self, attached: bool) -> io::Result<()> {
        match (attached, self.port.is_some()) {
            (true, false) => {
                let (near, far) = Self::open_pair();
                self.port = Some(near);
                *self.far.lock().unwrap() = Some(far);
            }
            (false, true) => {
                // Dropping closes it — the prior raw fd must go dead so a
                // capture-once pump would see POLLNVAL, just as QemuVm does
                // when it drops the chardev stream.
                self.port = None;
            }
            _ => {}
        }
        Ok(())
    }

    fn clock_ns(&mut self) -> Option<u64> {
        Some(guest_total(&self.clock).as_nanos() as u64)
    }
}

/// Hand the engine virtual time until `want` bytes have come out of each
/// far end in `ends`, and return what each read. A wait that hangs fails
/// with what each node's counters say.
fn read_stepped(ends: &[(&FarEnd, usize)], stats: &[&Arc<NodeStats>]) -> Vec<Vec<u8>> {
    let start = Instant::now();
    let mut got: Vec<Vec<u8>> = vec![Vec::new(); ends.len()];
    while ends
        .iter()
        .zip(&got)
        .any(|((_, want), have)| have.len() < *want)
    {
        assert!(
            start.elapsed() < HANG,
            "the bytes never arrived: got {:?} of {:?} at {} ns of virtual time; {stats:?}",
            got.iter().map(Vec::len).collect::<Vec<_>>(),
            ends.iter().map(|(_, want)| *want).collect::<Vec<_>>(),
            virtual_clock::virtual_ns(),
        );
        virtual_clock::wait_virtual_ns(SETTLE_NS);
        for ((end, _), have) in ends.iter().zip(got.iter_mut()) {
            have.extend(end.read_now());
        }
    }
    got
}

// ============================================================
// Metering
// ============================================================

#[rstest]
fn the_guest_runs_only_while_the_boards_clock_advances() {
    behaviour!(Test {
        id: "qemu-node.metered",
        covers: Some("qemu/src/node.rs#QemuNode"),
        given: "a computer on the board metered every 10 milliseconds, its guest a stand-in \
                with a stopwatch that runs only while it is let run, on the stepped clock",
    });
    expect!(
        "frozen-while-held",
        "while the board's clock is held the guest does not run at all, however much host \
         time passes"
    );
    expect!(
        "slice-per-quantum",
        "the guest is run once at each multiple of the quantum after the start, and is owed \
         exactly the board's time since the last",
        "the slices are the node's own wakes, which the engine fires at their instants"
    );
    expect!(
        "lives-the-boards-time",
        "the guest has lived as long as the board's clock has advanced, to within a quantum"
    );
    expect!(
        "books-match-the-guest",
        "what the node books as the guest's life is what the guest's own clock says, to \
         within two milliseconds",
        "each slice is booked from the guest's own clock when it can read one"
    );
    expect!(
        "goes-down-frozen",
        "once the board stops the guest runs no more, and it goes down with the node"
    );

    let _suite = suite_lock();
    let (guest, end) = FakeGuest::new();
    let node = QemuNode::new(Box::new(guest), 115_200).with_quantum(QUANTUM);
    let stats = node.stats();
    let (system, actor) = bench(vec![("PC", node)], &[]);
    let origin = virtual_clock::virtual_ns();
    let quantum_ns = QUANTUM.as_nanos() as u64;

    // The case is awake, so the engine cannot advance: the guest has not
    // run, and does not, however long the host waits.
    std::thread::sleep(Duration::from_millis(100));
    assert_eq!(guest_total(&end.clock), Duration::ZERO, "frozen-while-held");
    assert_eq!(end.clock.lock().unwrap().resumes, 0, "frozen-while-held");

    // Ten quanta. The slice due at the tenth fires after the case parks
    // again (at an instant, a released actor runs before what is due there
    // fires), so nine have run when the case reads, and they booked exactly
    // nine quanta of the board's time.
    virtual_clock::wait_until_ns(origin + 10 * quantum_ns);
    assert_eq!(stats.virtual_ns(), 9 * quantum_ns, "slice-per-quantum");
    assert_eq!(end.clock.lock().unwrap().resumes, stats.slices());
    let lived = guest_total(&end.clock).as_nanos() as u64;
    let board = stats.virtual_ns();
    assert!(
        lived.abs_diff(board) <= quantum_ns,
        "lives-the-boards-time: the guest lived {lived} ns of the board's {board} ns"
    );
    // A host that oversleeps a slice by a whole quantum pays it back by
    // skipping the next, so a slice can be missing; none is extra.
    assert!(
        (8..=9).contains(&stats.slices()),
        "slice-per-quantum: {} slices in nine quanta",
        stats.slices()
    );
    let booked = stats.guest_ns();
    assert!(
        booked.abs_diff(lived) < 2_000_000,
        "books-match-the-guest: the node booked {booked} ns, the guest's stopwatch says {lived} ns"
    );

    finish(system, actor);
    // Once the board stops, so does the guest.
    let after = guest_total(&end.clock);
    std::thread::sleep(Duration::from_millis(50));
    assert_eq!(guest_total(&end.clock), after, "goes-down-frozen");
    assert!(end.dropped.load(Ordering::Relaxed), "goes-down-frozen");
}

#[rstest]
fn a_node_that_boots_its_guest_boots_it_at_the_first_slice() {
    behaviour!(Test {
        id: "qemu-node.boots-at-first-slice",
        covers: Some("qemu/src/node.rs#QemuNode::booting"),
        given: "a computer on the board that makes its guest itself, the making taking a \
                fifth of a second of host time, metered every 10 milliseconds on the stepped \
                clock",
    });
    expect!(
        "not-while-held",
        "a bench started with its clock held and stopped again never boots the guest"
    );
    expect!(
        "at-first-quantum",
        "the guest is booted once, at exactly one quantum after the start, the board's clock \
         held there while it boots"
    );
    expect!(
        "boot-is-nobodys-time",
        "the host time the boot takes is not booked as the guest's life"
    );

    let _suite = suite_lock();
    let booted_at = Arc::new(AtomicU64::new(u64::MAX));
    let boots = Arc::new(AtomicU64::new(0));
    let make =
        |booted_at: &Arc<AtomicU64>, boots: &Arc<AtomicU64>, keep: Arc<Mutex<Option<FarEnd>>>| {
            let (booted_at, boots) = (Arc::clone(booted_at), Arc::clone(boots));
            QemuNode::booting(
                Box::new(move || {
                    booted_at.store(virtual_clock::virtual_ns(), Ordering::SeqCst);
                    boots.fetch_add(1, Ordering::SeqCst);
                    std::thread::sleep(Duration::from_millis(200));
                    let (guest, end) = FakeGuest::new();
                    *keep.lock().unwrap() = Some(end);
                    Ok(Box::new(guest) as Box<dyn Guest>)
                }),
                115_200,
            )
            .with_quantum(QUANTUM)
        };

    // Started held and stopped again, as `embsim check` does: no boot.
    virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);
    let held = System::new()
        .component(
            "PC",
            Box::new(make(&booted_at, &boots, Arc::new(Mutex::new(None)))),
        )
        .hold_time()
        .start()
        .expect("the bench starts");
    held.shutdown();
    assert_eq!(boots.load(Ordering::SeqCst), 0, "not-while-held");

    let end = Arc::new(Mutex::new(None));
    let node = make(&booted_at, &boots, Arc::clone(&end));
    let stats = node.stats();
    let (system, actor) = bench(vec![("PC", node)], &[]);
    let origin = virtual_clock::virtual_ns();
    let quantum_ns = QUANTUM.as_nanos() as u64;
    virtual_clock::wait_until_ns(origin + 5 * quantum_ns);
    assert_eq!(boots.load(Ordering::SeqCst), 1, "at-first-quantum");
    assert_eq!(
        booted_at.load(Ordering::SeqCst),
        origin + quantum_ns,
        "at-first-quantum"
    );
    let (took, at_ns) = stats.booted().expect("the node says it booted");
    assert_eq!(at_ns, origin + quantum_ns, "at-first-quantum");
    assert!(took >= Duration::from_millis(200), "the boot took {took:?}");
    let lived = guest_total(&end.lock().unwrap().as_ref().expect("booted").clock);
    assert!(
        lived < Duration::from_millis(200),
        "boot-is-nobodys-time: the guest lived {lived:?} in four quanta"
    );
    assert!(
        lived.as_nanos() as u64 >= 3 * quantum_ns,
        "the guest lived {lived:?} in four quanta"
    );
    assert_eq!(stats.failure(), None);
    finish(system, actor);
}

#[rstest]
fn a_guest_that_does_not_boot_stops_the_node_saying_why() {
    behaviour!(Test {
        id: "qemu-node.boot-failure",
        covers: Some("qemu/src/node.rs#QemuNode::booting"),
        given: "a computer on the board whose guest cannot be made, on the stepped clock",
    });
    expect!(
        "failure-says-why",
        "the node reports that the guest did not boot, with the reason the boot gave"
    );
    expect!("no-slices", "no slice runs, then or later");

    let _suite = suite_lock();
    let node = QemuNode::booting(
        Box::new(|| Err("no image at /nowhere.qcow2".to_string())),
        115_200,
    )
    .with_quantum(QUANTUM);
    let stats = node.stats();
    let (system, actor) = bench(vec![("PC", node)], &[]);
    virtual_clock::wait_virtual_ns(5 * QUANTUM.as_nanos() as u64);
    assert_eq!(
        stats.failure().as_deref(),
        Some("the guest did not boot: no image at /nowhere.qcow2"),
        "failure-says-why"
    );
    assert_eq!(stats.slices(), 0, "no-slices");
    assert_eq!(stats.booted(), None);
    finish(system, actor);
}

#[rstest]
fn a_late_stop_leaves_the_guest_ahead_which_is_reported_and_paid_back() {
    behaviour!(Test {
        id: "qemu-node.late-stop",
        covers: Some("qemu/src/node.rs#NodeStats::peak_lead_ns"),
        given: "a computer on the board metered every 10 milliseconds whose third slice's \
                freeze is answered 50 milliseconds late, the guest running on meanwhile, run \
                for twenty quanta on the stepped clock",
    });
    expect!(
        "lead-reported",
        "the node reports the guest at least 50 milliseconds ahead of the board at a slice's \
         end, and a slice that ran at least 50 milliseconds past its budget",
        "a slice ends when its freeze takes, so the guest leaves it ahead by what it \
         outlived its budget"
    );
    expect!(
        "paid-back",
        "the guest is not run again until the board has caught up, so by the end it has \
         lived the board's time to within a quantum, in at least four fewer slices"
    );

    let _suite = suite_lock();
    let late = Duration::from_millis(50);
    let (guest, end) = FakeGuest::new();
    let node = QemuNode::new(Box::new(guest.slow_pause_at(3, late)), 115_200).with_quantum(QUANTUM);
    let stats = node.stats();
    let (system, actor) = bench(vec![("PC", node)], &[]);
    let origin = virtual_clock::virtual_ns();
    let quantum_ns = QUANTUM.as_nanos() as u64;
    virtual_clock::wait_until_ns(origin + 20 * quantum_ns);

    let late_ns = late.as_nanos() as u64;
    assert!(
        stats.peak_lead_ns() >= late_ns,
        "lead-reported: the peak lead is {} ns",
        stats.peak_lead_ns()
    );
    assert!(
        stats.peak_overrun_ns() >= late_ns,
        "lead-reported: the longest overrun is {} ns",
        stats.peak_overrun_ns()
    );
    let lived = guest_total(&end.clock).as_nanos() as u64;
    let board = stats.virtual_ns();
    assert_eq!(board, 19 * quantum_ns);
    assert!(
        lived.abs_diff(board) <= quantum_ns,
        "paid-back: the guest lived {lived} ns of the board's {board} ns"
    );
    assert!(
        stats.slices() <= 19 - 4,
        "paid-back: {} slices in nineteen quanta",
        stats.slices()
    );
    assert_eq!(stats.failure(), None);
    finish(system, actor);
}

#[rstest]
fn a_lead_past_the_nodes_bound_stops_it_saying_so() {
    behaviour!(Test {
        id: "qemu-node.max-lead",
        covers: Some("qemu/src/node.rs#QemuNode::with_max_lead"),
        given: "a computer on the board metered every 10 milliseconds and allowed to lead the \
                board by 20 milliseconds, whose third slice's freeze is answered 50 \
                milliseconds late, on the stepped clock",
    });
    expect!(
        "failure-says-so",
        "the node stops with a failure saying how far ahead of the board the guest ended the \
         slice, and the bound it passed"
    );
    expect!("no-more-slices", "no slice runs after the third");

    let _suite = suite_lock();
    let (guest, _end) = FakeGuest::new();
    let node = QemuNode::new(
        Box::new(guest.slow_pause_at(3, Duration::from_millis(50))),
        115_200,
    )
    .with_quantum(QUANTUM)
    .with_max_lead(2 * QUANTUM);
    let stats = node.stats();
    let (system, actor) = bench(vec![("PC", node)], &[]);
    let quantum_ns = QUANTUM.as_nanos() as u64;
    virtual_clock::wait_virtual_ns(10 * quantum_ns);
    let failure = stats.failure().expect("failure-says-so: the node failed");
    assert!(
        failure.starts_with("the guest ended a slice ")
            && failure.contains(" ahead of the board, past the 20.000 ms the node allows"),
        "failure-says-so: {failure}"
    );
    assert_eq!(stats.slices(), 3, "no-more-slices");
    virtual_clock::wait_virtual_ns(10 * quantum_ns);
    assert_eq!(stats.slices(), 3, "no-more-slices");
    finish(system, actor);
}

#[rstest]
fn a_guest_that_dies_mid_run_stops_the_node_saying_why() {
    behaviour!(Test {
        id: "qemu-node.guest-dies",
        covers: Some("qemu/src/node.rs#QemuNode"),
        given: "a computer on the board metered every 10 milliseconds whose guest cannot be \
                run again after its third slice, as when its process has exited, on the stepped \
                clock",
    });
    expect!(
        "failure-says-why",
        "the node stops with a failure that carries the guest's own error and what the guest \
         adds about why",
        "a QEMU guest adds its exit status and the end of its log, which it keeps"
    );
    expect!("no-more-slices", "no slice runs after the third");

    let _suite = suite_lock();
    let (guest, _end) = FakeGuest::new();
    let node = QemuNode::new(Box::new(guest.dying_after(3)), 115_200).with_quantum(QUANTUM);
    let stats = node.stats();
    let (system, actor) = bench(vec![("PC", node)], &[]);
    let quantum_ns = QUANTUM.as_nanos() as u64;
    virtual_clock::wait_virtual_ns(10 * quantum_ns);
    assert_eq!(
        stats.failure().as_deref(),
        Some("the guest failed mid-slice: the fake guest's process has exited; its log says why"),
        "failure-says-why"
    );
    assert_eq!(stats.slices(), 3, "no-more-slices");
    virtual_clock::wait_virtual_ns(10 * quantum_ns);
    assert_eq!(stats.slices(), 3, "no-more-slices");
    finish(system, actor);
}

// ============================================================
// Bytes
// ============================================================

#[rstest]
fn a_guest_on_an_unpowered_line_sends_nothing_and_is_counted() {
    behaviour!(Test {
        id: "qemu-node.unpowered-line",
        covers: Some("qemu/src/node.rs#NodeStats::unpowered"),
        given: "a computer on the board whose line's rail and return are wired to nothing, \
                as a harness of only the transmit and receive wires leaves them, its guest \
                writing twelve bytes, on the stepped clock",
    });
    expect!(
        "counted-unpowered",
        "the node counts all twelve as sent while its line was unpowered, and as shed",
        "a host's line is driven from its own rail, and a host with no rail sends nothing"
    );

    let _suite = suite_lock();
    virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);
    let (guest, end) = FakeGuest::new();
    let node = QemuNode::new(Box::new(guest), 2_000_000).with_quantum(QUANTUM);
    let stats = node.stats();
    let system = System::new()
        .component("PC", Box::new(node))
        .hold_time()
        .start()
        .expect("the bench starts");
    let actor = virtual_clock::register_actor("qemu-loopback-case");
    system.release_time();
    let bytes = b"hello, board";
    end.write(bytes);
    let start = Instant::now();
    while stats.unpowered() < bytes.len() as u64 {
        assert!(
            start.elapsed() < HANG,
            "the bytes were never taken: {stats:?}"
        );
        virtual_clock::wait_virtual_ns(SETTLE_NS);
    }
    assert_eq!(stats.unpowered(), bytes.len() as u64, "counted-unpowered");
    assert_eq!(stats.shed(), bytes.len() as u64, "counted-unpowered");
    assert_eq!(stats.from_guest(), 0, "counted-unpowered");
    finish(system, actor);
}

/// Two nodes as a null-modem cable at `baud`, each one's `TX` to the
/// other's `RX`.
fn null_modem(baud: u32) -> ((SystemHandle, Actor), [FarEnd; 2], [Arc<NodeStats>; 2]) {
    let (guest_a, end_a) = FakeGuest::new();
    let (guest_b, end_b) = FakeGuest::new();
    let node_a = QemuNode::new(Box::new(guest_a), baud);
    let node_b = QemuNode::new(Box::new(guest_b), baud);
    let stats = [node_a.stats(), node_b.stats()];
    let bench = bench(
        vec![("A", node_a), ("B", node_b)],
        &[("A.TX", "B.RX"), ("B.TX", "A.RX")],
    );
    (bench, [end_a, end_b], stats)
}

#[rstest]
fn bytes_cross_the_net_in_both_directions() {
    behaviour!(Test {
        id: "qemu-node.duplex",
        covers: Some("qemu/src/node.rs#QemuNode"),
        given: "two computers on the board wired as a null-modem cable at 2 megabaud on their \
                own 3.3 volt rails, both guests writing at once, on the stepped clock",
    });
    expect!(
        "a-to-b",
        "what the first guest writes to its serial port is read, whole and in order, from \
         the second's",
        "the bytes cross the wire as levels, framed by one node and deframed by the other"
    );
    expect!(
        "b-to-a",
        "what the second guest writes is read from the first's"
    );
    expect!(
        "counted",
        "each node counts every byte its guest sent and every byte it delivered, and sheds none"
    );

    let _suite = suite_lock();
    let ((system, actor), [end_a, end_b], [stats_a, stats_b]) = null_modem(2_000_000);
    let from_a = b"hello from A";
    let from_b = b"and hello back from B";
    end_a.write(from_a);
    end_b.write(from_b);
    let got = read_stepped(
        &[(&end_b, from_a.len()), (&end_a, from_b.len())],
        &[&stats_a, &stats_b],
    );
    assert_eq!(got[0], from_a, "a-to-b");
    assert_eq!(got[1], from_b, "b-to-a");
    assert_eq!(stats_a.from_guest(), from_a.len() as u64, "counted");
    assert_eq!(stats_b.to_guest(), from_a.len() as u64, "counted");
    assert_eq!(stats_b.from_guest(), from_b.len() as u64, "counted");
    assert_eq!(stats_a.to_guest(), from_b.len() as u64, "counted");
    assert_eq!(stats_a.shed() + stats_b.shed(), 0, "counted");
    assert_eq!(stats_a.framing_errors() + stats_b.framing_errors(), 0);
    finish(system, actor);
}

#[rstest]
fn a_burst_longer_than_a_quantum_arrives_intact_and_in_order() {
    behaviour!(Test {
        id: "qemu-node.burst",
        covers: Some("qemu/src/node.rs#QemuNode"),
        given: "two computers on the board at 115200 baud, the first guest writing 2000 bytes \
                at once, about 174 milliseconds of the line's time, on the stepped clock",
    });
    expect!(
        "intact-in-order",
        "the second guest reads all 2000 bytes, in the order they were written, none shed",
        "bytes the board delivers while a guest is frozen wait for its next slice"
    );

    let _suite = suite_lock();
    let ((system, actor), [end_a, end_b], [stats_a, stats_b]) = null_modem(115_200);
    let burst: Vec<u8> = (0..2000u32).map(|i| (i % 251) as u8).collect();
    end_a.write(&burst);
    let got = read_stepped(&[(&end_b, burst.len())], &[&stats_a, &stats_b]);
    assert_eq!(got[0], burst, "intact-in-order");
    assert_eq!(stats_b.shed(), 0, "intact-in-order");
    finish(system, actor);
}

#[rstest]
fn a_guest_that_outruns_the_line_is_never_blocked() {
    behaviour!(Test {
        id: "qemu-node.never-blocks-the-guest",
        covers: Some("qemu/src/node.rs#pump_main"),
        given: "two computers on the board at 2 megabaud, the first guest writing 256 \
                kilobytes in one blocking write, 1.3 seconds of the line's time, on the stepped \
                clock",
    });
    expect!(
        "write-completes",
        "the guest's write completes within five seconds of host time, however far behind \
         the line is",
        "QEMU writes a guest's serial bytes blocking, under its big lock: a port nobody reads \
         stalls the guest and with it the stop that ends its slice"
    );
    expect!(
        "all-arrive",
        "every byte reaches the second guest, in order, none shed"
    );

    let _suite = suite_lock();
    let ((system, actor), [end_a, end_b], [stats_a, stats_b]) = null_modem(2_000_000);
    let burst: Vec<u8> = (0..(256 * 1024u32)).map(|i| (i % 253) as u8).collect();
    // The bound on the write is the claim: the pump drains the socket into
    // its own queue whatever the line or the engine is doing, so the write
    // takes a socket copy's time on any runner. The bound on the read is
    // `HANG`: when the last byte arrives is this runner's engine speed
    // times the 2.6 million bits the burst is, which is no claim here.
    {
        let mut slot = end_a.far.lock().unwrap();
        let far = slot.as_mut().unwrap();
        far.set_nonblocking(false).unwrap();
        far.set_write_timeout(Some(Duration::from_secs(5))).unwrap();
        let t0 = Instant::now();
        far.write_all(&burst)
            .expect("write-completes: the guest's write is never blocked");
        let took = t0.elapsed();
        assert!(
            took < Duration::from_secs(5),
            "write-completes: the guest was held up for {took:?} writing {} bytes",
            burst.len()
        );
        far.set_nonblocking(true).unwrap();
    }
    let got = read_stepped(&[(&end_b, burst.len())], &[&stats_a, &stats_b]);
    assert_eq!(got[0].len(), burst.len(), "all-arrive");
    assert!(got[0] == burst, "all-arrive: bytes reordered or corrupted");
    assert_eq!(stats_b.shed() + stats_a.shed(), 0, "all-arrive");
    finish(system, actor);
}

/// A port that is unplugged carries nothing, and the guest keeps running.
///
/// The node meters the guest against the board's clock, so an unplugged
/// slice still has to resume and pause it -- a guest frozen for the duration
/// of the unplug could not notice the unplug, and the browser inside it
/// would never fire the disconnect event the reconnect path is waiting for.
#[rstest]
fn an_unplugged_port_carries_nothing_and_the_guest_still_runs() {
    behaviour!(Test {
        id: "qemu-node.unplug",
        covers: Some("qemu/src/node.rs#LinkControl"),
        given: "two computers on the board at 2 megabaud, the first one's serial cable pulled \
                while the second keeps writing, then put back, on the stepped clock",
    });
    expect!(
        "still-runs",
        "the unplugged guest goes on being run a slice every quantum"
    );
    expect!(
        "nothing-crosses",
        "nothing the second guest writes while the cable is out reaches the first"
    );
    expect!(
        "back-again",
        "once the cable is back, what the second guest writes reaches the first"
    );
    expect!(
        "not-gone",
        "pulling the cable is not taken for the guest going away"
    );

    let _suite = suite_lock();
    let (guest_a, end_a) = FakeGuest::new();
    let (guest_b, end_b) = FakeGuest::new();
    let node_a = QemuNode::new(Box::new(guest_a), 2_000_000);
    let node_b = QemuNode::new(Box::new(guest_b), 2_000_000);
    let (stats_a, stats_b) = (node_a.stats(), node_b.stats());
    // The cable is A's, taken before the node goes into the system: the
    // handle a harness keeps.
    let cable = node_a.link();
    let (system, actor) = bench(
        vec![("A", node_a), ("B", node_b)],
        &[("A.TX", "B.RX"), ("B.TX", "A.RX")],
    );

    // Plugged: a byte from B reaches A's guest.
    end_b.write(b"plugged");
    let got = read_stepped(&[(&end_a, b"plugged".len())], &[&stats_a, &stats_b]);
    assert_eq!(got[0], b"plugged");

    assert!(cable.is_plugged(), "the port starts plugged in");
    cable.unplug().expect("unplug");
    assert!(!cable.is_plugged(), "the port reports itself unplugged");
    let slices_before = stats_a.slices();
    end_b.write(b"into the void");
    // Twenty milliseconds of the board's time: the void's 13 bytes take 65
    // microseconds of line time, and at least nineteen slices come due.
    for _ in 0..20 {
        virtual_clock::wait_virtual_ns(SETTLE_NS);
    }
    assert!(
        stats_a.slices() >= slices_before + 10,
        "still-runs: the node stopped running an unplugged guest ({slices_before} -> {})",
        stats_a.slices()
    );
    // The far end may have seen end of file from the closed near end; that
    // is still nothing crossing.
    assert!(end_a.read_now().is_empty(), "nothing-crosses");

    cable.plug().expect("replug");
    assert!(cable.is_plugged(), "the port came back");
    end_b.write(b"back again");
    let got = read_stepped(&[(&end_a, b"back again".len())], &[&stats_a, &stats_b]);
    assert_eq!(got[0], b"back again", "back-again");
    assert!(!stats_a.disconnected(), "not-gone");
    finish(system, actor);
}
