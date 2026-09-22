//! A host's serial port, as a component — proved with a null-modem cable.
//!
//! [`HostPty`] puts a PTY on one side and two digital pins on the other. The
//! cheapest honest way to test it is to wire two of them together the way a
//! null-modem cable does — each one's TX to the other's RX — open both PTYs the
//! way host software does, and check a byte written to one comes out of the
//! other.
//!
//! That exercises the whole path in both directions with nothing simulated
//! away: host write → pump thread → UART framing → net edges → deframing →
//! engine thread → host read. A byte that arrives has been a waveform on a net,
//! not a value passed between two halves of the same object.
//!
//! # Why this is the test that catches the interesting failures
//!
//! Each end frames and deframes independently, so the two cannot cancel out a
//! shared bug — a wrong bit order, a missing stop bit, or an idle level that
//! never gets driven breaks the round trip rather than surviving it. The
//! mismatched-baud case below is the same argument from the other side: the
//! path is real enough that getting the rate wrong actually costs you the data.

use std::io::{Read, Write};
use std::os::fd::AsRawFd;
use std::sync::{Mutex, MutexGuard};
use std::time::{Duration, Instant};

use embsim_board::{Harness, HostPty, System};
use embsim_core::virtual_clock;

/// The virtual clock is process-global; these tests take it one at a time.
static CLOCK_LOCK: Mutex<()> = Mutex::new(());

fn lock_clock() -> MutexGuard<'static, ()> {
    CLOCK_LOCK.lock().unwrap_or_else(|poisoned| {
        CLOCK_LOCK.clear_poison();
        poisoned.into_inner()
    })
}

/// A PTY symlink path unique to one test, so tests can run concurrently and a
/// stale link from a killed run cannot be mistaken for this one's.
fn link_path(test: &str, end: &str) -> String {
    let dir = std::env::temp_dir().join("embsim-host-pty-tests");
    let _ = std::fs::create_dir_all(&dir);
    dir.join(format!("tty.{test}.{end}"))
        .to_string_lossy()
        .into_owned()
}

/// Open the PTY the way host software does, non-blocking so a quiet link does
/// not wedge the test.
fn open_host_end(path: &str) -> std::fs::File {
    let file = std::fs::OpenOptions::new()
        .read(true)
        .write(true)
        .open(path)
        .expect("the component's PTY symlink is openable");
    // SAFETY: `file` owns the descriptor for the duration of these calls.
    unsafe {
        let fd = file.as_raw_fd();
        let flags = libc::fcntl(fd, libc::F_GETFL);
        assert!(flags >= 0, "F_GETFL on the PTY");
        assert!(
            libc::fcntl(fd, libc::F_SETFL, flags | libc::O_NONBLOCK) >= 0,
            "O_NONBLOCK on the PTY"
        );
    }
    file
}

/// Read from a non-blocking PTY until `want` bytes have arrived or time runs
/// out. Returns what it got, so a failure can show the partial read.
fn read_until(file: &mut std::fs::File, want: usize, timeout: Duration) -> Vec<u8> {
    let mut got = Vec::new();
    let start = Instant::now();
    while got.len() < want && start.elapsed() < timeout {
        let mut buf = [0u8; 256];
        match file.read(&mut buf) {
            Ok(0) => std::thread::sleep(Duration::from_millis(1)),
            Ok(n) => got.extend_from_slice(&buf[..n]),
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                std::thread::sleep(Duration::from_millis(1))
            }
            Err(e) => panic!("reading the host end: {e}"),
        }
    }
    got
}

/// Two host ports wired as a null-modem cable, each end at its own baud.
struct NullModem {
    a: std::fs::File,
    b: std::fs::File,
    _system: embsim_board::SystemHandle,
}

fn null_modem(test: &str, baud_a: u32, baud_b: u32) -> NullModem {
    virtual_clock::init(0.0, 1_000_000);

    let (path_a, path_b) = (link_path(test, "a"), link_path(test, "b"));
    let port_a = HostPty::open(&path_a, baud_a).expect("a PTY opens");
    let port_b = HostPty::open(&path_b, baud_b).expect("a PTY opens");

    // A null-modem cable crosses the pair: each end's transmit is the other's
    // receive.
    let harness = Harness::new()
        .connect_str("A.TX", "B.RX")
        .expect("endpoints parse")
        .connect_str("B.TX", "A.RX")
        .expect("endpoints parse");

    let system = System::new()
        .component("A", Box::new(port_a))
        .component("B", Box::new(port_b))
        .harness(harness)
        .start()
        .expect("the bench system starts");

    NullModem {
        a: open_host_end(&path_a),
        b: open_host_end(&path_b),
        _system: system,
    }
}

#[test]
fn a_byte_written_to_one_host_port_arrives_at_the_other() {
    let _clock = lock_clock();
    let mut link = null_modem("roundtrip", 115_200, 115_200);

    link.a.write_all(b"Hello").expect("writing the host end");
    link.a.flush().expect("flushing the host end");
    assert_eq!(
        read_until(&mut link.b, 5, Duration::from_secs(10)),
        b"Hello",
        "five bytes crossed a net as edges and came back out a PTY"
    );

    // And the other way, on the cable's other pair — a link that only works in
    // one direction is a wiring mistake this would otherwise hide.
    link.b.write_all(b"World!").expect("writing the host end");
    link.b.flush().expect("flushing the host end");
    assert_eq!(
        read_until(&mut link.a, 6, Duration::from_secs(10)),
        b"World!",
        "the return pair carries too"
    );
}

#[test]
fn every_byte_value_survives_the_wire() {
    // A framing bug rarely breaks every byte — it breaks the ones whose bit
    // pattern happens to expose it. $00 and $FF bracket the stop-bit and
    // idle-level mistakes; the rest catch bit-order and off-by-one errors.
    let _clock = lock_clock();
    let mut link = null_modem("all-values", 115_200, 115_200);

    let payload: Vec<u8> = (0..=255u8).collect();
    link.a.write_all(&payload).expect("writing the host end");
    link.a.flush().expect("flushing the host end");

    let got = read_until(&mut link.b, payload.len(), Duration::from_secs(30));
    assert_eq!(
        got.len(),
        payload.len(),
        "all 256 values arrived (got {} of {})",
        got.len(),
        payload.len()
    );
    assert_eq!(got, payload, "and none of them changed on the way");
}

#[test]
fn a_baud_mismatch_costs_the_data_rather_than_being_defined_away() {
    // The point of carrying serial as levels instead of handing bytes across:
    // a link whose ends disagree about the rate must actually fail. If this
    // ever passes, the bytes are not really crossing a wire.
    let _clock = lock_clock();
    let mut link = null_modem("baud-mismatch", 115_200, 9_600);

    link.a.write_all(b"Hello").expect("writing the host end");
    link.a.flush().expect("flushing the host end");

    let got = read_until(&mut link.b, 5, Duration::from_secs(5));
    assert_ne!(
        got, b"Hello",
        "a receiver sampling twelve times too slowly cannot recover the message"
    );
}
