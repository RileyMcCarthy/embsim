//! The `host-serial` bench kind, live: a host's serial port as a PTY whose
//! bytes are levels on the wire, carrying bytes both ways.
//!
//! Two `host-serial` components, `A` and `B`, wired as a null-modem cable
//! (each one's `TX` to the other's `RX`) on one host rail: 3.3 V on both
//! `VIO` pins, 0 V on both `GND`s. The case opens both PTYs the way host
//! software does, writes to one and reads what arrives at the other, then
//! the other way. A byte that arrives has crossed the nets as ten levels at
//! its baud, framed by one port and deframed by the other, at the rail the
//! project gave the hosts.
//!
//! Stepped (`TESTING.md` rule 9), its own binary: the clock stepped, the
//! system started with time held, the case's thread a registered actor
//! that hands the engine a millisecond of virtual time between reads. A
//! host writes in wall time, so the instant its bytes land is wherever the
//! run has reached — what is asserted is that they arrive, whole and in
//! order; the wall-time bound on the wait is sized for a hang.

use std::io::{Read, Write};
use std::os::fd::AsRawFd;
use std::path::PathBuf;
use std::time::{Duration, Instant};

use embsim_board::{Finding, Project};
use embsim_boards::catalog::CatalogSet;
use embsim_core::virtual_clock::{self, ClockMode};
use rstest::rstest;
use vibes_behaviour::{behaviour, expect, Test};

/// The virtual time the case hands the engine between reads of a host end.
const SETTLE_NS: u64 = 1_000_000;

/// How long, in wall time, a stepped run may take to carry a few bytes
/// before the case calls it hung.
const HANG: Duration = Duration::from_secs(60);

/// A PTY path of this test's own.
fn link(end: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("embsim-host-serial-{}", std::process::id()));
    std::fs::create_dir_all(&dir).expect("the PTY directory can be made");
    dir.join(format!("tty.{end}"))
}

/// Open a PTY the way host software does, non-blocking.
fn open_host_end(path: &PathBuf) -> std::fs::File {
    let file = std::fs::OpenOptions::new()
        .read(true)
        .write(true)
        .open(path)
        .expect("the component's PTY is openable");
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

/// Hand the engine virtual time until `want` bytes have come out of
/// `file`, and return them.
fn read_stepped(file: &mut std::fs::File, want: usize) -> Vec<u8> {
    let start = Instant::now();
    let mut got = Vec::new();
    while got.len() < want {
        assert!(
            start.elapsed() < HANG,
            "the bytes never arrived; got {got:?}"
        );
        virtual_clock::wait_virtual_ns(SETTLE_NS);
        let mut buf = [0u8; 64];
        match file.read(&mut buf) {
            Ok(n) => got.extend_from_slice(&buf[..n]),
            Err(err) if err.kind() == std::io::ErrorKind::WouldBlock => {
                std::thread::sleep(Duration::from_millis(1));
            }
            Err(err) => panic!("reading the host end: {err}"),
        }
    }
    got
}

#[rstest]
fn bytes_written_to_one_host_port_arrive_at_the_other_both_ways() {
    behaviour!(Test {
        id: "host-serial.bytes-both-ways",
        covers: Some("boards/src/catalog.rs#host_serial"),
        given: "two host serial ports in a project, wired as a null-modem cable at 115200 \
                baud on a 3.3 volt host rail, run on the stepped clock",
    });
    expect!(
        "first-to-second",
        "what a host writes to the first port's terminal is read, whole and in order, from \
         the second's",
        "the bytes cross the wire as levels at the rail the project gives the hosts"
    );
    expect!(
        "second-to-first",
        "what a host writes to the second port's terminal is read from the first's"
    );
    let (a, b) = (link("a"), link("b"));
    let project = format!(
        r#"
[[component]]
name = "A"
kind = "host-serial"
[component.options]
baud = 115200
path = {a:?}

[[component]]
name = "B"
kind = "host-serial"
[component.options]
baud = 115200
path = {b:?}

[[wire]]
from = "A.TX"
to = "B.RX"

[[wire]]
from = "B.TX"
to = "A.RX"

[[wire]]
from = "HOSTS.3V3"
to = "A.VIO"
volts = 3.3

[[wire]]
from = "HOSTS.3V3"
to = "B.VIO"

[[wire]]
from = "HOSTS.GND"
to = "A.GND"
volts = 0.0

[[wire]]
from = "HOSTS.GND"
to = "B.GND"
"#
    );

    virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);
    let system = Project::parse(&project)
        .expect("the text is a project")
        .instantiate(&CatalogSet::new())
        .expect("the bench builds")
        .hold_time()
        .start()
        .expect("the bench starts");
    let actor = virtual_clock::register_actor("host-serial-case");
    system.release_time();
    let mut host_a = open_host_end(&a);
    let mut host_b = open_host_end(&b);

    host_a.write_all(b"ping").expect("the first host writes");
    host_a.flush().expect("the first host flushes");
    assert_eq!(read_stepped(&mut host_b, 4), b"ping");

    host_b.write_all(b"pong!").expect("the second host writes");
    host_b.flush().expect("the second host flushes");
    assert_eq!(read_stepped(&mut host_a, 5), b"pong!");

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

#[rstest]
#[case::no_baud("", "options.baud is the link's rate")]
#[case::zero_baud("baud = 0", "options.baud = 0 is not a rate")]
#[case::unknown("baud = 9600\nparity = \"even\"", "unknown option \"parity\"")]
fn a_host_port_that_names_no_rate_is_refused(#[case] options: &str, #[case] says: &str) {
    behaviour!(Test {
        id: "host-serial.refusals",
        covers: Some("boards/src/catalog.rs#host_serial"),
        given: "a host serial port with no baud rate, a rate of zero, or an option the kind \
                does not take",
    });
    expect!(
        "refused-saying-why",
        "the project is refused before it starts, naming the component and what to fix",
        "a link's rate is the project's to name, and the kind invents none"
    );
    let text = format!(
        "[[component]]\nname = \"HOST\"\nkind = \"host-serial\"\n[component.options]\n\
         path = {:?}\n{options}\n",
        link("refused")
    );
    let message = Project::parse(&text)
        .expect("the text is a project")
        .instantiate(&CatalogSet::new())
        .expect_err("the port is refused")
        .to_string();
    assert!(
        message.contains("component HOST (kind \"host-serial\")"),
        "{message}"
    );
    assert!(message.contains(says), "{says:?} missing from:\n{message}");
}
