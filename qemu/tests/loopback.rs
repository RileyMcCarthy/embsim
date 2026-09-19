//! The node without QEMU: a fake guest on a socketpair with a stopwatch.
//!
//! Proves the two things the node exists for — the guest's clock advances
//! only as far as the board's does, and bytes cross the net in both
//! directions — with nothing installed.

use std::io::{self, Read, Write};
use std::os::fd::{AsRawFd, RawFd};
use std::os::unix::net::UnixStream;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};

use embsim_board::{Harness, System};
use embsim_core::virtual_clock::{self, ClockMode};
use embsim_qemu::{Guest, NodeStats, QemuNode};

// ============================================================
// Plumbing
// ============================================================

/// The virtual clock is process-global: one test at a time.
static SUITE_LOCK: Mutex<()> = Mutex::new(());

fn suite_lock() -> MutexGuard<'static, ()> {
    SUITE_LOCK.lock().unwrap_or_else(|poisoned| {
        SUITE_LOCK.clear_poison();
        poisoned.into_inner()
    })
}

/// Unpaced for the guard's lifetime; the re-init on drop is also what
/// releases the node's parked actor so it can see its shutdown flag.
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
        std::thread::sleep(Duration::from_millis(5));
    }
    pred()
}

/// A stopwatch that only runs between `resume` and `pause`: the fake guest's clock.
#[derive(Default)]
struct Stopwatch {
    running_since: Option<Instant>,
    total: Duration,
    resumes: u64,
}

/// A guest that is a socketpair and a stopwatch.
struct FakeGuest {
    port: UnixStream,
    clock: Arc<Mutex<Stopwatch>>,
    dropped: Arc<AtomicBool>,
    /// Stands in for a pulled cable: `serial_fd` reports -1 while set, which
    /// is exactly what `QemuVm` does once it has closed the chardev.
    detached: Arc<AtomicBool>,
}

impl Drop for FakeGuest {
    fn drop(&mut self) {
        self.dropped.store(true, Ordering::Relaxed);
    }
}

impl FakeGuest {
    /// Returns the guest and the test's end of its serial port.
    fn new() -> (Self, UnixStream, Arc<Mutex<Stopwatch>>) {
        let (guest, far, clock, _) = Self::with_drop_flag();
        (guest, far, clock)
    }

    /// [`new`](Self::new), plus the cable: setting the flag makes the guest
    /// report no descriptor, which is what a pulled USB serial adapter looks
    /// like to the node.
    fn unpluggable() -> (Self, UnixStream, Arc<AtomicBool>) {
        let (guest, far, _, _) = Self::with_drop_flag();
        let cable = Arc::clone(&guest.detached);
        (guest, far, cable)
    }

    /// [`new`](Self::new), plus a flag the guest raises when it is dropped.
    fn with_drop_flag() -> (Self, UnixStream, Arc<Mutex<Stopwatch>>, Arc<AtomicBool>) {
        let (port, far) = UnixStream::pair().expect("socketpair");
        port.set_nonblocking(true).expect("nonblocking");
        far.set_read_timeout(Some(Duration::from_millis(50)))
            .expect("read timeout");
        let clock = Arc::new(Mutex::new(Stopwatch::default()));
        let dropped = Arc::new(AtomicBool::new(false));
        (
            Self {
                port,
                clock: Arc::clone(&clock),
                dropped: Arc::clone(&dropped),
                detached: Arc::new(AtomicBool::new(false)),
            },
            far,
            clock,
            dropped,
        )
    }
}

impl Guest for FakeGuest {
    fn resume(&mut self) -> io::Result<()> {
        let mut c = self.clock.lock().unwrap();
        assert!(c.running_since.is_none(), "resumed twice without a pause");
        c.running_since = Some(Instant::now());
        c.resumes += 1;
        Ok(())
    }

    fn pause(&mut self) -> io::Result<()> {
        let mut c = self.clock.lock().unwrap();
        if let Some(since) = c.running_since.take() {
            c.total += since.elapsed();
        }
        Ok(())
    }

    fn serial_fd(&self) -> RawFd {
        if self.detached.load(Ordering::Relaxed) {
            -1
        } else {
            self.port.as_raw_fd()
        }
    }

    fn serial_attached(&self) -> bool {
        !self.detached.load(Ordering::Relaxed)
    }

    fn set_serial_attached(&mut self, attached: bool) -> io::Result<()> {
        self.detached.store(!attached, Ordering::Relaxed);
        Ok(())
    }

    fn clock_ns(&mut self) -> Option<u64> {
        Some(guest_total(&self.clock).as_nanos() as u64)
    }
}

fn guest_total(clock: &Mutex<Stopwatch>) -> Duration {
    let c = clock.lock().unwrap();
    c.total + c.running_since.map(|s| s.elapsed()).unwrap_or_default()
}

fn read_until(far: &mut UnixStream, expected: usize, timeout: Duration) -> Vec<u8> {
    let mut got = Vec::new();
    let mut buf = [0u8; 256];
    let start = Instant::now();
    while got.len() < expected && start.elapsed() < timeout {
        match far.read(&mut buf) {
            Ok(0) => break,
            Ok(n) => got.extend_from_slice(&buf[..n]),
            Err(e)
                if matches!(
                    e.kind(),
                    io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
                ) => {}
            Err(e) => panic!("read: {e}"),
        }
    }
    got
}

// ============================================================
// Tests
// ============================================================

#[test]
fn the_guest_runs_only_as_far_as_the_board_has_advanced() {
    let _suite = suite_lock();
    let _stepped = Stepped::enter();

    let (guest, _far, clock, dropped) = FakeGuest::with_drop_flag();
    let node = QemuNode::new(Box::new(guest), 115_200).with_slice(Duration::from_millis(10));
    let stats: Arc<NodeStats> = node.stats();
    let system = System::new()
        .component("PC", Box::new(node))
        .start()
        .expect("system starts");

    // Unpaced, the engine's only deadlines are the node's slices, so the
    // board advances 10 ms of virtual time each time the guest has lived
    // 10 ms of wall time: the two clocks run in lockstep at wall speed.
    assert!(
        wait_for(|| stats.slices() >= 20, Duration::from_secs(5)),
        "expected 20 slices, got {}",
        stats.slices()
    );
    let slice_ns = 10_000_000u64;

    // Sampled mid-run: the board never runs ahead of the guest by more than
    // one slice, and the guest never runs ahead of the board at all (it is
    // frozen while the board moves). Bounded skew is the guarantee.
    let skew = stats.skew_ns();
    assert!(
        skew.unsigned_abs() <= slice_ns + 2_000_000,
        "skew {skew} ns exceeds a slice: virtual {} ns, guest {} ns",
        stats.virtual_ns(),
        stats.guest_ns()
    );
    assert_eq!(stats.shed(), 0);
    assert!(!stats.disconnected());

    drop(system);
    // Once the board stops, so does the guest: no slice can run without a
    // virtual deadline being reached.
    let before = guest_total(&clock);
    std::thread::sleep(Duration::from_millis(50));
    assert_eq!(
        guest_total(&clock),
        before,
        "the guest ran with the board stopped"
    );

    // With both clocks final, what the node booked as the guest's life is
    // what the guest's own stopwatch saw, to a resume/pause round trip —
    // not one slice more, not one slice less.
    let guest_lived = stats.guest_ns();
    let stopwatch = before.as_nanos() as u64;
    let disagreement = stopwatch.abs_diff(guest_lived);
    // The books are corrected from the guest's own clock every slice, so
    // only the last slice's stopwatch estimate can be off — by a scheduling
    // hiccup at most.
    let tolerance = 2_000_000;
    assert!(
        disagreement < tolerance,
        "node says the guest lived {guest_lived} ns, the guest's stopwatch says {stopwatch} ns"
    );
    // The guest went down with the node — no clock re-init was needed.
    assert!(
        dropped.load(Ordering::Relaxed),
        "the guest outlived its node"
    );
}

#[test]
fn bytes_cross_the_net_in_both_directions() {
    let _suite = suite_lock();
    let _stepped = Stepped::enter();

    let (guest_a, mut far_a, _) = FakeGuest::new();
    let (guest_b, mut far_b, _) = FakeGuest::new();
    let node_a = QemuNode::new(Box::new(guest_a), 2_000_000);
    let node_b = QemuNode::new(Box::new(guest_b), 2_000_000);
    let (stats_a, stats_b) = (node_a.stats(), node_b.stats());
    let harness = Harness::new()
        .connect_str("A.TX", "B.RX")
        .unwrap()
        .connect_str("B.TX", "A.RX")
        .unwrap();
    let system = System::new()
        .component("A", Box::new(node_a))
        .component("B", Box::new(node_b))
        .harness(harness)
        .start()
        .expect("system starts");

    // Both computers speak at once. Each byte becomes ten edges on its net
    // and is deframed off the other node's RX at 2 Mbaud.
    let from_a = b"hello from A";
    let from_b = b"and hello back from B";
    far_a.write_all(from_a).unwrap();
    far_b.write_all(from_b).unwrap();

    let got_b = read_until(&mut far_b, from_a.len(), Duration::from_secs(5));
    let got_a = read_until(&mut far_a, from_b.len(), Duration::from_secs(5));
    assert_eq!(got_b, from_a, "B did not hear A");
    assert_eq!(got_a, from_b, "A did not hear B");

    assert_eq!(stats_a.from_guest(), from_a.len() as u64);
    assert_eq!(stats_b.to_guest(), from_a.len() as u64);
    assert_eq!(stats_b.from_guest(), from_b.len() as u64);
    assert_eq!(stats_a.to_guest(), from_b.len() as u64);
    assert_eq!(stats_a.shed() + stats_b.shed(), 0);

    drop(system);
}

#[test]
fn a_burst_longer_than_a_slice_arrives_intact_and_in_order() {
    let _suite = suite_lock();
    let _stepped = Stepped::enter();

    let (guest_a, mut far_a, _) = FakeGuest::new();
    let (guest_b, mut far_b, _) = FakeGuest::new();
    // At 115200 baud a byte is ~87 µs; 2000 bytes is ~174 ms of line time,
    // spanning many 10 ms slices, so the receiver's outbound queue has to
    // hold bytes across freezes and hand them over in order.
    let node_a = QemuNode::new(Box::new(guest_a), 115_200);
    let node_b = QemuNode::new(Box::new(guest_b), 115_200);
    let stats_b = node_b.stats();
    let harness = Harness::new()
        .connect_str("A.TX", "B.RX")
        .unwrap()
        .connect_str("B.TX", "A.RX")
        .unwrap();
    let system = System::new()
        .component("A", Box::new(node_a))
        .component("B", Box::new(node_b))
        .harness(harness)
        .start()
        .expect("system starts");

    let burst: Vec<u8> = (0..2000u32).map(|i| (i % 251) as u8).collect();
    far_a.write_all(&burst).unwrap();
    let got = read_until(&mut far_b, burst.len(), Duration::from_secs(10));
    assert_eq!(
        got.len(),
        burst.len(),
        "B got {} of {} bytes",
        got.len(),
        burst.len()
    );
    assert_eq!(got, burst, "bytes reordered or corrupted");
    assert_eq!(stats_b.shed(), 0);

    drop(system);
}

#[test]
fn a_guest_that_outruns_the_line_is_never_blocked() {
    let _suite = suite_lock();
    let _stepped = Stepped::enter();

    let (guest_a, mut far_a, _) = FakeGuest::new();
    let (guest_b, mut far_b, _) = FakeGuest::new();
    let node_a = QemuNode::new(Box::new(guest_a), 2_000_000);
    let node_b = QemuNode::new(Box::new(guest_b), 2_000_000);
    let stats_b = node_b.stats();
    let harness = Harness::new()
        .connect_str("A.TX", "B.RX")
        .unwrap()
        .connect_str("B.TX", "A.RX")
        .unwrap();
    let system = System::new()
        .component("A", Box::new(node_a))
        .component("B", Box::new(node_b))
        .harness(harness)
        .start()
        .expect("system starts");

    // QEMU writes the guest's serial bytes to its socket blocking, under its
    // big lock: a socket the node stops reading stalls the vCPU, and with it
    // the QMP channel the node needs to freeze the guest — a deadlock. So
    // the node must keep draining the socket even when the line is far
    // behind. A blocking write far larger than any socket buffer or the
    // bridge's queue must complete promptly, and the bytes still arrive in
    // order once the line has carried them (256 KiB at 2 Mbaud is ~1.3 s of
    // line time, spanning a thousand slices).
    let burst: Vec<u8> = (0..(256 * 1024u32)).map(|i| (i % 253) as u8).collect();
    far_a
        .set_write_timeout(Some(Duration::from_secs(5)))
        .unwrap();
    let t0 = Instant::now();
    far_a
        .write_all(&burst)
        .expect("the guest's write is never blocked");
    let took = t0.elapsed();
    assert!(
        took < Duration::from_secs(5),
        "the guest was held up for {took:?} writing {} bytes",
        burst.len()
    );
    let got = read_until(&mut far_b, burst.len(), Duration::from_secs(60));
    assert_eq!(
        got.len(),
        burst.len(),
        "B got {} of {} bytes",
        got.len(),
        burst.len()
    );
    assert_eq!(got, burst, "bytes reordered or corrupted");
    assert_eq!(stats_b.shed(), 0);

    drop(system);
}

/// A port that is unplugged carries nothing, and the guest keeps running.
///
/// The node meters the guest against the board's clock, so an unplugged slice
/// still has to resume and pause it -- a guest frozen for the duration of the
/// unplug could not notice the unplug, and the browser inside it would never
/// fire the disconnect event the reconnect path is waiting for.
#[test]
fn an_unplugged_port_carries_nothing_and_the_guest_still_runs() {
    let _suite = suite_lock();
    let _stepped = Stepped::enter();

    let (guest_a, mut far_a, _cable_a) = FakeGuest::unpluggable();
    let (guest_b, mut far_b, _) = FakeGuest::new();
    let node_a = QemuNode::new(Box::new(guest_a), 2_000_000);
    let node_b = QemuNode::new(Box::new(guest_b), 2_000_000);
    let (stats_a, stats_b) = (node_a.stats(), node_b.stats());
    // The public handle, not the guest's own flag: this is the path a test
    // harness takes, so it is the path worth covering.
    let cable_a = node_a.link();
    let harness = Harness::new()
        .connect_str("A.TX", "B.RX")
        .unwrap()
        .connect_str("B.TX", "A.RX")
        .unwrap();
    let _system = System::new()
        .component("A", Box::new(node_a))
        .component("B", Box::new(node_b))
        .harness(harness)
        .start()
        .expect("system starts");

    // Plugged: a byte from B reaches A's guest.
    far_b.write_all(b"plugged").unwrap();
    let slices_before = wait_for_slices(&stats_a, 1);
    let mut buf = [0u8; 32];
    let _ = far_a.read(&mut buf);

    // Pull the cable. B keeps talking into a port that is not there.
    assert!(cable_a.is_plugged(), "the port starts plugged in");
    cable_a.unplug().expect("unplug");
    assert!(!cable_a.is_plugged(), "the port reports itself unplugged");
    far_b.write_all(b"into the void").unwrap();
    let slices_during = wait_for_slices(&stats_a, slices_before + 4);

    // The guest went on being scheduled -- that is the claim, and it has to be
    // "kept going" rather than "twitched once". Removing the unplugged-path
    // guard in run_slice makes drain_outbound write to fd -1, which returns
    // EBADF and kills the pump; a few slices still land before it dies, so
    // `> slices_before` passes and proves nothing. Requiring the node to reach
    // the target is what makes this test fail for that mutation.
    assert!(
        slices_during >= slices_before + 4,
        "the node stopped slicing an unplugged guest ({slices_before} -> {slices_during}, wanted {})",
        slices_before + 4
    );
    // ...and nothing arrived while it was out.
    let mut void = [0u8; 64];
    let got = far_a.read(&mut void).unwrap_or(0);
    assert_eq!(got, 0, "bytes crossed a port that was unplugged");

    // Put it back: a reconnect test needs the port to RETURN, not just vanish.
    cable_a.plug().expect("replug");
    assert!(cable_a.is_plugged(), "the port came back");
    far_b.write_all(b"back again").unwrap();
    let deadline = Instant::now() + Duration::from_secs(10);
    let mut seen = Vec::new();
    while Instant::now() < deadline && seen.len() < b"back again".len() {
        let mut buf = [0u8; 64];
        match far_a.read(&mut buf) {
            Ok(0) => break,
            Ok(n) => seen.extend_from_slice(&buf[..n]),
            Err(_) => continue,
        }
    }
    assert_eq!(
        String::from_utf8_lossy(&seen),
        "back again",
        "nothing crossed after the port was plugged back in"
    );

    assert!(stats_b.slices() > 0, "B never ran");
}

/// Wait until the node has completed at least `want` slices, and report where
/// it got to. Bounded so a stalled node fails the test rather than hanging it.
fn wait_for_slices(stats: &Arc<NodeStats>, want: u64) -> u64 {
    let deadline = Instant::now() + Duration::from_secs(10);
    loop {
        let now = stats.slices();
        if now >= want || Instant::now() > deadline {
            return now;
        }
        std::thread::sleep(Duration::from_millis(20));
    }
}
