//! The real `qemu-system-p2` lives exactly as long as its node: killed, it
//! stops the core and says why; dropped with its node, it is gone; and when
//! the process that started it dies, however it dies, it ends itself.
//!
//! The guest is two instructions of PASM2 run from cog 0 in place of the
//! boot ROM — `drvnot #0` and a jump back to it — so the program is always
//! mid-run: every toggle of `P0` is a pad change, a turn, a publish.
//!
//! ```text
//!         org     0
//!         drvnot  #0                    ' FD64005F
//!         jmp     #\0                   ' FD800000
//! ```
//!
//! Needs a `qemu-system-p2` (`embsim qemu install`), so every case is
//! `#[ignore]`d in the workspace's tests; CI's `p2-qemu-boot` job runs them.
//! The stand-in program of `tests/program.rs` proves the node's half of the
//! same claims without QEMU.

use std::io::{BufRead, BufReader};
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

use embsim_board::{EndpointRef, Harness, System};
use embsim_boards::p2::P2Package;
use embsim_core::virtual_clock;
use embsim_p2_qemu::P2Qemu;

/// `drvnot #0` / `jmp #\0`: toggle `P0` for ever.
const TOGGLE: [u32; 2] = [0xFD64_005F, 0xFD80_0000];

/// The variable that makes [`orphan_parent`] start a P2 and wait to be
/// killed.
const PARENT: &str = "EMBSIM_P2_QEMU_ORPHAN_PARENT";

fn toggle_image() -> Vec<u8> {
    TOGGLE.iter().flat_map(|w| w.to_le_bytes()).collect()
}

fn ep(endpoint: &str) -> EndpointRef {
    EndpointRef::parse(endpoint).expect("endpoint parses")
}

/// Whether process `pid` is gone.
fn gone(pid: u32) -> bool {
    // SAFETY: signal 0 delivers nothing; it asks whether the process exists.
    let alive = unsafe { libc::kill(pid as libc::pid_t, 0) } == 0;
    !alive
}

fn wait_for(mut pred: impl FnMut() -> bool, timeout: Duration) -> bool {
    let start = Instant::now();
    while start.elapsed() < timeout {
        if pred() {
            return true;
        }
        std::thread::sleep(Duration::from_millis(2));
    }
    pred()
}

#[test]
#[ignore = "needs qemu-system-p2 (embsim qemu install); CI's p2-qemu-boot job runs it"]
fn a_program_killed_mid_run_stops_its_core_and_says_why() {
    let p2 = P2Qemu::with_boot_rom(&toggle_image(), &[]).expect("qemu-system-p2 starts");
    let handle = p2.handle();
    let pid = handle.pid();

    virtual_clock::init(0.0, 160_000_000);
    let system = System::new()
        .component("P2", Box::new(P2Package::new(p2)))
        .harness(
            Harness::new()
                .power(ep("BENCH.GND"), ep("P2.GND"), 0.0)
                .power(ep("BENCH.VDD"), ep("P2.VDD"), 1.8)
                .power(ep("BENCH.RESN"), ep("P2.RESN"), 3.3)
                .power(ep("BENCH.VIO_0_3"), ep("P2.VIO_0_3"), 3.3),
        )
        .start()
        .expect("the bench starts");

    assert!(
        wait_for(|| handle.publishes() > 1_000, Duration::from_secs(60)),
        "the guest toggles P0: publishes={} failure={:?}",
        handle.publishes(),
        handle.failure()
    );
    // SAFETY: the test's own child, ended the way `kill -9` ends it.
    assert_eq!(unsafe { libc::kill(pid as libc::pid_t, libc::SIGKILL) }, 0);
    assert!(
        wait_for(|| handle.failure().is_some(), Duration::from_secs(5)),
        "the core noticed its program die"
    );
    let failure = handle.failure().expect("a failure");
    assert!(
        failure.contains(&format!(
            "pid {pid}) was killed by signal 9 (SIGKILL) during a run"
        )),
        "{failure}"
    );
    assert!(handle.halted(), "the core runs no further");
    assert!(gone(pid), "the program was reaped");
    // The rest of the system goes on: the engine is alive, and P0 keeps
    // the drive it last published.
    let publishes = handle.publishes();
    std::thread::sleep(Duration::from_millis(50));
    assert_eq!(handle.publishes(), publishes, "no publish after the death");
    assert!(system.engine_is_alive());
    assert!(system.net_state("P2.P0").is_some());
    drop(system);
}

#[test]
#[ignore = "needs qemu-system-p2 (embsim qemu install); CI's p2-qemu-boot job runs it"]
fn a_dropped_core_takes_its_program_with_it() {
    let p2 = P2Qemu::with_boot_rom(&toggle_image(), &[]).expect("qemu-system-p2 starts");
    let pid = p2.handle().pid();
    assert!(!gone(pid));
    drop(p2);
    assert!(gone(pid), "pid {pid} outlived its core");
}

/// The process the next case kills: started by it, it starts a P2, says
/// the program's pid, and waits to be killed. Does nothing unless started
/// so.
#[test]
#[ignore = "the process `the_program_ends_when_the_process_that_started_it_dies` kills; not a test"]
fn orphan_parent() {
    if std::env::var_os(PARENT).is_none() {
        return;
    }
    let p2 = P2Qemu::with_boot_rom(&toggle_image(), &[]).expect("qemu-system-p2 starts");
    println!("qemu-system-p2 pid {}", p2.handle().pid());
    std::thread::sleep(Duration::from_secs(120));
    drop(p2);
}

#[test]
#[ignore = "needs qemu-system-p2 (embsim qemu install); CI's p2-qemu-boot job runs it"]
fn the_program_ends_when_the_process_that_started_it_dies() {
    let mut parent = Command::new(std::env::current_exe().expect("the test binary"))
        .args(["--exact", "orphan_parent", "--ignored", "--nocapture"])
        .env(PARENT, "1")
        .stdout(Stdio::piped())
        .spawn()
        .expect("the parent starts");
    let mut lines = BufReader::new(parent.stdout.take().expect("piped")).lines();
    let pid: u32 = loop {
        let line = lines
            .next()
            .expect("the parent says the pid before it ends")
            .expect("readable");
        if let Some(pid) = line.strip_prefix("qemu-system-p2 pid ") {
            break pid.trim().parse().expect("a pid");
        }
    };
    assert!(!gone(pid), "the program runs while its parent does");
    // The parent dies the way nothing can clean up after: no drop, no
    // atexit, no signal handler.
    parent.kill().expect("killed");
    parent.wait().expect("reaped");
    assert!(
        wait_for(|| gone(pid), Duration::from_secs(5)),
        "qemu-system-p2 (pid {pid}) outlived the process that started it"
    );
}
