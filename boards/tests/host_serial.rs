//! The `host-serial` bench kind, live: a host's serial port as a PTY whose
//! bytes are levels on the wire, carrying bytes both ways at the rail the
//! project gives the host.
//!
//! Two `host-serial` components, `A` and `B`, wired as a null-modem cable
//! (each one's `TX` to the other's `RX`) on one host rail: 3.3 V on both
//! `VIO` pins, 0 V on both `GND`s. The case opens both PTYs the way host
//! software does, writes to one and reads what arrives at the other, then
//! the other way. A byte that arrives has crossed the nets as ten levels at
//! its baud, framed by one port and deframed by the other, at the rail the
//! project gave the hosts. A second test writes the first bytes before the
//! system starts, when the engine has not yet read either host's rail: the
//! port reads its host only once it has, so none is shed. A wait that hangs
//! fails with each port's counts, the nets and the findings.
//!
//! Stepped (`TESTING.md` rule 9), its own binary: the clock stepped, the
//! system started with time held, the case's thread a registered actor
//! that hands the engine a millisecond of virtual time between reads. A
//! host writes in wall time, so the instant its bytes land is wherever the
//! run has reached — what is asserted is that they arrive, whole and in
//! order; the wall-time bound on the wait is sized for a hang.
//!
//! One host whose `VIO` a scripted source steps — unsourced, then 1.8 V,
//! then 3.3 V — shows `TX` driving at whatever its rail reads, and nothing
//! while it reads none. The live cases take a suite lock: the virtual clock
//! is process-global, and each case re-anchors it stepped.

use std::io::{Read, Write};
use std::os::fd::AsRawFd;
use std::path::PathBuf;
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};

use embsim_board::{
    AttachError, Component, ComponentNetIo, EndpointRef, Finding, Harness, NetState, PinDecl,
    Project, Reports,
};
use embsim_boards::catalog::CatalogSet;
use embsim_core::virtual_clock::{self, ClockMode};
use rstest::rstest;
use vibes_behaviour::{behaviour, expect, Test};

/// The virtual time the case hands the engine between reads of a host end.
const SETTLE_NS: u64 = 1_000_000;

/// How long, in wall time, a stepped run may take to carry a few bytes
/// before the case calls it hung.
const HANG: Duration = Duration::from_secs(60);

/// One live case at a time: the virtual clock is process-global, and each
/// case re-anchors it in stepped mode (`TESTING.md` rule 9).
static SUITE_LOCK: Mutex<()> = Mutex::new(());

fn suite_lock() -> MutexGuard<'static, ()> {
    SUITE_LOCK.lock().unwrap_or_else(|poisoned| {
        SUITE_LOCK.clear_poison();
        poisoned.into_inner()
    })
}

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
/// `file`, and return them. A wait that hangs fails with `said()`: what the
/// run can say of where the bytes stopped.
fn read_stepped(file: &mut std::fs::File, want: usize, said: &dyn Fn() -> String) -> Vec<u8> {
    let start = Instant::now();
    let mut got = Vec::new();
    while got.len() < want {
        assert!(
            start.elapsed() < HANG,
            "the bytes never arrived; got {got:?} at {} ns of virtual time\n{}",
            virtual_clock::virtual_ns(),
            said()
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
    null_modem(false);
}

#[rstest]
fn bytes_a_host_wrote_before_the_start_arrive_once_it_runs() {
    behaviour!(Test {
        id: "host-serial.written-before-start",
        covers: Some("board/src/host_pty.rs#pump_loop"),
        given: "two host serial ports wired as a null-modem cable at 115200 baud on a 3.3 \
                volt host rail, the first host writing to its terminal once the bench is \
                built and before it starts, when the engine has read neither host's rail, \
                run on the stepped clock",
    });
    expect!(
        "arrives-whole",
        "once the run starts, what the first host wrote is read, whole and in order, from \
         the second port's terminal",
        "a port reads its host once the engine has read the host's rail, so a powered host's \
         bytes go out on the rail it has"
    );
    expect!("none-shed", "both ports report zero bytes shed");
    null_modem(true);
}

/// The two ports as a null-modem cable, `ping` one way and `pong!` the
/// other, the first host writing before the system starts when `early`;
/// with the stepped clock, under the suite lock. Fails when a byte is
/// lost or a port reports one shed.
fn null_modem(early: bool) {
    let tag = if early { "early" } else { "running" };
    let (a, b) = (link(&format!("a-{tag}")), link(&format!("b-{tag}")));
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

    let _suite = suite_lock();
    virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);
    let reports = Reports::new();
    let system = Project::parse(&project)
        .expect("the text is a project")
        .instantiate_with(&CatalogSet::new(), &reports)
        .expect("the bench builds");
    // The PTYs are there once the bench is built.
    let mut host_a = open_host_end(&a);
    let mut host_b = open_host_end(&b);
    let write_ping = |host: &mut std::fs::File| {
        host.write_all(b"ping").expect("the first host writes");
        host.flush().expect("the first host flushes");
    };
    if early {
        write_ping(&mut host_a);
    }
    let system = system.hold_time().start().expect("the bench starts");
    let reports = reports.take();
    let actor = virtual_clock::register_actor("host-serial-case");
    system.release_time();
    // Each port's count of the bytes each way, the nets and the findings:
    // where a byte that never arrived stopped.
    let said = || {
        let mut text: Vec<String> = reports
            .iter()
            .flat_map(|report| {
                let subject = report.subject();
                report
                    .summary()
                    .into_iter()
                    .map(move |line| format!("{subject}: {line}"))
            })
            .collect();
        for net in ["A.TX", "B.RX", "B.TX", "A.RX", "A.VIO", "B.VIO"] {
            text.push(format!("net {net}: {:?}", system.net_state(net)));
        }
        text.push(format!("findings: {:?}", system.findings()));
        text.join("\n")
    };
    if !early {
        write_ping(&mut host_a);
    }
    assert_eq!(read_stepped(&mut host_b, 4, &said), b"ping");

    host_b.write_all(b"pong!").expect("the second host writes");
    host_b.flush().expect("the second host flushes");
    assert_eq!(read_stepped(&mut host_a, 5, &said), b"pong!");
    let told = said();
    assert!(!told.contains("shed"), "{told}");

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

/// The rail steps the scripted source drives onto the host's `VIO`, as
/// (ns after the start, volts): unsourced before the first.
const RAIL_STEPS: [(u64, f64); 2] = [(1_000_000, 1.8), (2_000_000, 3.3)];

/// Every voltage the scope's pin is handed, with its instant.
type Trace = Arc<Mutex<Vec<(u64, Option<f64>)>>>;

/// An analog reader on the host's `TX`, recording what it is handed: the
/// line's voltage itself, which a net's state projects to a level once it
/// is a valid one.
struct Scope {
    pins: [PinDecl; 1],
    trace: Trace,
}

impl Component for Scope {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let trace = Arc::clone(&self.trace);
        io.on_sense("IN", move |sense| {
            trace
                .lock()
                .expect("the trace is never poisoned")
                .push((sense.at_ns, sense.volts));
        })
    }
}

#[rstest]
fn a_host_ports_transmit_line_drives_at_the_rail_its_vio_reads() {
    behaviour!(Test {
        id: "host-serial.tx-follows-vio",
        covers: Some("board/src/host_pty.rs#HostPty::open_on_rail"),
        given: "a host serial port on a 0 volt ground whose I/O rail pin a 1 ohm scripted \
                source leaves unsourced for a millisecond, then steps to 1.8 volts and, at two \
                milliseconds, to 3.3 volts, its idle transmit line read by a scope and \
                nothing else, run on the stepped clock",
    });
    expect!(
        "released-without-rail",
        "while the rail pin reads no voltage the transmit line floats and the scope is \
         handed no voltage",
        "a host's driver is powered from the host's own rail"
    );
    expect!(
        "idles-at-its-rail",
        "the scope is handed 1.8 volts at exactly one millisecond, the instant the rail \
         comes up",
        "an idle line is high, and a high is the host's own rail above its own ground"
    );
    expect!(
        "follows-its-rail",
        "the scope is handed 3.3 volts at exactly two milliseconds, the instant the rail \
         steps, and no other voltage after the build"
    );
    let project = format!(
        r#"
[[component]]
name = "A"
kind = "host-serial"
[component.options]
baud = 115200
path = {path:?}

[[component]]
name = "RAIL"
kind = "scripted-source"
[component.options]
ohms = 1.0
steps = [["1ms", 1.8], ["2ms", 3.3]]

[[wire]]
from = "RAIL.OUT"
to = "A.VIO"

[[wire]]
from = "BENCH.GND"
to = "A.GND"
volts = 0.0
"#,
        path = link("rail")
    );
    let trace: Trace = Arc::new(Mutex::new(Vec::new()));
    let scope = Scope {
        pins: [PinDecl::analog("IN")],
        trace: Arc::clone(&trace),
    };

    let _suite = suite_lock();
    virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);
    let system = Project::parse(&project)
        .expect("the text is a project")
        .instantiate(&CatalogSet::new())
        .expect("the bench builds")
        .component("SCOPE", Box::new(scope))
        .harness(Harness::new().connect(
            EndpointRef::parse("SCOPE.IN").expect("endpoint parses"),
            EndpointRef::parse("A.TX").expect("endpoint parses"),
        ))
        .hold_time()
        .start()
        .expect("the bench starts");
    let actor = virtual_clock::register_actor("host-serial-rail-case");
    system.release_time();
    assert_eq!(
        virtual_clock::virtual_ns(),
        0,
        "the system starts at the clock's 0"
    );

    let [(first_ns, first_volts), (second_ns, second_volts)] = RAIL_STEPS;
    virtual_clock::wait_until_ns(first_ns - 1);
    assert_eq!(
        system.net_state("A.TX"),
        Some(NetState::Floating),
        "1 ns before the rail comes up"
    );
    virtual_clock::wait_until_ns(second_ns + 1_000_000);

    // The build's own delivery reads the line released; after it, the scope
    // is handed the rail's voltage at each of its instants.
    let trace = trace.lock().expect("the trace is never poisoned").clone();
    assert!(
        trace
            .iter()
            .all(|(at_ns, volts)| *at_ns > 0 || volts.is_none()),
        "with no rail the line carries no voltage: {trace:?}"
    );
    let handed: Vec<(u64, Option<f64>)> = trace
        .iter()
        .copied()
        .filter(|(at_ns, _)| *at_ns > 0)
        .collect();
    let volts = |at: usize| handed[at].1.expect("the line reads a voltage");
    assert_eq!(handed.len(), 2, "{handed:?}");
    assert_eq!(handed[0].0, first_ns, "{handed:?}");
    assert!((volts(0) - first_volts).abs() < 1e-9, "{handed:?}");
    assert_eq!(handed[1].0, second_ns, "{handed:?}");
    assert!((volts(1) - second_volts).abs() < 1e-9, "{handed:?}");

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
#[case::a_file("file")]
#[case::a_directory("directory")]
fn a_host_port_never_replaces_what_its_path_holds(#[case] what: &str) {
    behaviour!(Test {
        id: "host-serial.path-guard",
        covers: Some("boards/src/catalog.rs#host_serial"),
        given: "a host serial port whose path names an existing file holding text, or an \
                existing directory",
    });
    expect!(
        "refused-and-left",
        "the project is refused before it starts, naming the component and the path and \
         saying to name a free path, and the file or directory is left as it was",
        "a port's link replaces only a link an earlier run left there"
    );
    let path = link(&format!("taken-{what}"));
    let _ = std::fs::remove_dir_all(&path);
    let _ = std::fs::remove_file(&path);
    if what == "file" {
        std::fs::write(&path, "precious notes").expect("the file is writable");
    } else {
        std::fs::create_dir(&path).expect("the directory can be made");
    }
    let text = format!(
        "[[component]]\nname = \"HOST\"\nkind = \"host-serial\"\n[component.options]\n\
         baud = 115200\npath = {path:?}\n"
    );
    let message = Project::parse(&text)
        .expect("the text is a project")
        .instantiate(&CatalogSet::new())
        .expect_err("the port is refused")
        .to_string();
    assert!(
        message.contains(&format!(
            "component HOST (kind \"host-serial\"): {} exists and is not a PTY link; name a \
             free path",
            path.display()
        )),
        "{message}"
    );
    if what == "file" {
        assert_eq!(
            std::fs::read_to_string(&path).expect("the file is still there"),
            "precious notes"
        );
        let _ = std::fs::remove_file(&path);
    } else {
        assert!(path.is_dir(), "the directory is still there");
        let _ = std::fs::remove_dir(&path);
    }
}
