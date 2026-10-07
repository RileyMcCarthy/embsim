//! `qemu-system-p2`, the program, as the node meets it: the handshake that
//! refuses the wrong one, a program that dies before or during a run, and
//! a program that will not quit.
//!
//! None of it needs QEMU. The program the node starts here is a stand-in
//! that speaks the node's protocol (`embsim_p2_qemu::protocol`) over the
//! same channels: this test binary itself, started through a two-line shell
//! script as `fake_qemu_system_p2` (an `#[ignore]`d test that does nothing
//! unless the script set its mode), which says hello as it is told, serves
//! runs by answering that every cog reached the horizon, and dies as it is
//! told. What it proves is the node's half — the refusals, the exit status
//! and standard error in the error, the time to notice, the program never
//! outliving its node — and the identity check that ties an installed
//! program to this crate's target. The real program's half is proved
//! against the real program (`tests/lifecycle.rs`, `#[ignore]`d; CI's
//! `p2-qemu-boot` job).

use std::os::fd::{BorrowedFd, FromRawFd};
use std::os::unix::net::UnixStream;
use std::path::PathBuf;
use std::sync::atomic::Ordering;
use std::time::{Duration, Instant};

use embsim_p2_qemu::protocol::{self, shm, Hello, Run, ShmPage, StopHeader, OP_RUN};
use embsim_p2_qemu::{target, Identity, P2Qemu, P2QemuError, Peer, QemuSystemP2, Transport};
use rstest::rstest;
use vibes_behaviour::{behaviour, expect, Test};

/// The variable that makes this binary the stand-in, and says how it acts.
const MODE: &str = "EMBSIM_FAKE_QEMU_MODE";
/// The arguments the node started the stand-in with, one per line.
const ARGS: &str = "EMBSIM_FAKE_QEMU_ARGS";

/// A run request whose horizon is `horizon`.
fn run_to(horizon: u64) -> Run {
    Run {
        op: OP_RUN,
        horizon_clocks: horizon,
        banks_powered: 0xFFFF,
        banks_high: 0xFFFF,
        ..Run::default()
    }
}

/// A `qemu-system-p2` that is this binary, acting as `mode` says.
fn fake(mode: &str) -> QemuSystemP2 {
    use std::os::unix::fs::PermissionsExt;
    let dir = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join("fake-qemu");
    std::fs::create_dir_all(&dir).expect("scratch");
    // One script per call: a test writing a script another is starting
    // would find it busy.
    static NEXT: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
    let script = dir.join(format!(
        "qemu-system-p2-{}-{}-{}",
        mode.replace([':', '='], "-"),
        std::process::id(),
        NEXT.fetch_add(1, Ordering::Relaxed)
    ));
    let me = std::env::current_exe().expect("the test binary");
    std::fs::write(
        &script,
        format!(
            "#!/bin/sh\n{ARGS}=\"$(printf '%s\\n' \"$@\")\"\n{MODE}='{mode}'\n\
             export {ARGS} {MODE}\n\
             exec '{}' --exact fake_qemu_system_p2 --ignored --nocapture --test-threads=1\n",
            me.display()
        ),
    )
    .expect("the script is writable");
    std::fs::set_permissions(&script, std::fs::Permissions::from_mode(0o755)).expect("executable");
    QemuSystemP2::at(script)
}

/// Whether process `pid` is gone.
fn gone(pid: u32) -> bool {
    // SAFETY: signal 0 delivers nothing; it asks whether the process exists.
    let alive = unsafe { libc::kill(pid as libc::pid_t, 0) } == 0;
    !alive
}

/// The stand-in: does nothing unless the script started it.
#[test]
#[ignore = "the stand-in qemu-system-p2 the other tests start; not a test"]
fn fake_qemu_system_p2() {
    let Ok(mode) = std::env::var(MODE) else {
        return;
    };
    let args = std::env::var(ARGS).unwrap_or_default();
    let spec = args
        .lines()
        .find_map(|a| a.strip_prefix("p2,hostipc="))
        .expect("the node names the channel");
    let fields: Vec<&str> = spec.split(':').collect();
    let fd: i32 = fields[1].parse().expect("a channel descriptor");

    if mode == "refuses-hostipc" {
        eprintln!("qemu-system-p2: Property 'p2-machine.hostipc' not found");
        std::process::exit(1);
    }
    let mut hello = Hello {
        magic: protocol::MAGIC,
        protocol: protocol::PROTOCOL,
        pid: std::process::id(),
        target: target::identity().to_string(),
        qemu: target::qemu_pin().version().to_string(),
    };
    match mode.as_str() {
        "old-protocol" => hello.protocol = 999,
        "other-target" => hello.target = "0000000000000000".to_string(),
        "not-the-protocol" => hello.magic = 0xDEAD_BEEF,
        _ => {}
    }
    let dies_after: Option<u32> = mode.strip_prefix("dies-after:").map(|n| n.parse().unwrap());
    let killed_after: Option<u32> = mode
        .strip_prefix("killed-after:")
        .map(|n| n.parse().unwrap());
    let stubborn = mode == "stubborn";

    let reply = |run: &Run| {
        StopHeader {
            reason: protocol::reason::HORIZON,
            now_clocks: run.horizon_clocks,
            any_running: 1,
            slices: 1,
            ..StopHeader::default()
        }
        .encode()
    };
    let mut served = 0u32;
    let mut serve = |run: &Run| -> bool {
        if run.op != OP_RUN {
            return !stubborn;
        }
        if dies_after == Some(served) {
            eprintln!("fake qemu-system-p2: dying after {served} runs, as asked");
            std::process::exit(3);
        }
        if killed_after == Some(served) {
            // SAFETY: the stand-in ends itself the way `kill -9` would. The
            // signal ends the process, not necessarily before this thread
            // returns from the call, so it answers nothing more meanwhile.
            unsafe { libc::kill(libc::getpid(), libc::SIGKILL) };
            loop {
                std::thread::sleep(Duration::from_secs(1));
            }
        }
        served += 1;
        false
    };

    if fields[0] == "shm" {
        // SAFETY: the node handed this descriptor down open.
        let page = ShmPage::map(unsafe { BorrowedFd::borrow_raw(fd) }).expect("the page");
        page.write(shm::HELLO, &hello.encode());
        page.word(shm::READY).store(1, Ordering::SeqCst);
        protocol::futex_wake(page.word(shm::READY));
        let mut seen = 0u32;
        loop {
            let seq = page.word(shm::REQ_SEQ);
            while seq.load(Ordering::SeqCst) == seen {
                protocol::futex_wait(seq, seen, Duration::from_millis(10));
            }
            seen = seq.load(Ordering::SeqCst);
            let mut bytes = [0u8; Run::LEN];
            page.read(shm::REQ, &mut bytes);
            let run = Run::decode(&bytes);
            if serve(&run) {
                std::process::exit(0);
            }
            if run.op != OP_RUN {
                continue;
            }
            page.write(shm::REP, &reply(&run));
            page.word(shm::REP_SEQ).store(seen, Ordering::SeqCst);
            if page.word(shm::REP_SLEEP).load(Ordering::SeqCst) != 0 {
                protocol::futex_wake(page.word(shm::REP_SEQ));
            }
        }
    } else {
        use std::io::{Read, Write};
        // SAFETY: the node handed this descriptor down open, and only this
        // process uses it.
        let mut stream = unsafe { UnixStream::from_raw_fd(fd) };
        stream.write_all(&hello.encode()).expect("hello");
        loop {
            let mut bytes = [0u8; Run::LEN];
            if stream.read_exact(&mut bytes).is_err() {
                if stubborn {
                    std::thread::sleep(Duration::from_secs(60));
                }
                std::process::exit(0);
            }
            let run = Run::decode(&bytes);
            if serve(&run) {
                std::process::exit(0);
            }
            if run.op == OP_RUN {
                stream.write_all(&reply(&run)).expect("reply");
            }
        }
    }
}

/// The digest `stage.sh` writes into a staged tree, for the program to
/// report, is the one the node checks the program's hello against.
#[rstest]
fn the_identity_stage_sh_computes_is_the_one_the_node_checks() {
    behaviour!(Test {
        id: "p2-qemu.target-identity",
        covers: Some("p2-qemu/src/target.rs#identity"),
        given: "the P2 target's sources as this embsim carries them, and the script that stages \
                them into a QEMU tree",
    });
    expect!(
        "one-digest",
        "the script's digest of the sources and embsim's are the same sixteen hex digits",
        "an installed qemu-system-p2 reports the digest the script staged, and the node refuses \
         one whose digest is not its own"
    );
    let script = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("qemu-target/stage.sh");
    let output = std::process::Command::new("sh")
        .arg(&script)
        .arg("--identity")
        .output()
        .expect("sh runs");
    assert!(output.status.success(), "{output:?}");
    assert_eq!(
        String::from_utf8_lossy(&output.stdout).trim(),
        target::identity()
    );
}

#[rstest]
fn a_program_of_another_protocol_is_refused_saying_how_to_install_the_right_one(
    #[values(Transport::Shm { spin_ns: 20_000 }, Transport::Socket)] transport: Transport,
) {
    behaviour!(Test {
        id: "p2-qemu.refuse-other-protocol",
        covers: Some("p2-qemu/src/peer.rs#Peer::start"),
        given: "a qemu-system-p2 whose handshake names a protocol other than the one this embsim \
                speaks",
    });
    expect!(
        "names-both",
        "starting a P2 on it fails, naming its protocol and this embsim's"
    );
    expect!(
        "how-to-install",
        "the error says to install the matching program with `embsim qemu install`, and where \
         it goes"
    );
    let program = fake("old-protocol");
    let err = P2Qemu::start(&program, &[], &[], transport).expect_err("refused");
    let text = err.to_string();
    assert!(matches!(err, P2QemuError::Refused { .. }), "{text}");
    assert!(
        text.contains(&format!(
            "it speaks protocol 999 and this embsim speaks protocol {}",
            protocol::PROTOCOL
        )),
        "{text}"
    );
    assert!(text.contains("`embsim qemu install`"), "{text}");
    assert!(text.contains(target::identity()), "{text}");
}

#[rstest]
fn a_program_built_from_another_target_is_refused_naming_both() {
    behaviour!(Test {
        id: "p2-qemu.refuse-other-target",
        covers: Some("p2-qemu/src/peer.rs#Identity::check"),
        given: "a qemu-system-p2 that speaks this embsim's protocol but was built from other P2 \
                target sources",
    });
    expect!(
        "names-both-targets",
        "starting a P2 on it fails, naming the target it was built from and the one this \
         embsim carries"
    );
    let err =
        P2Qemu::start(&fake("other-target"), &[], &[], Transport::default()).expect_err("refused");
    let text = err.to_string();
    assert!(
        text.contains(&format!(
            "it was built from P2 target 0000000000000000 and this embsim carries target {}",
            target::identity()
        )),
        "{text}"
    );
    let err = P2Qemu::start(&fake("not-the-protocol"), &[], &[], Transport::default())
        .expect_err("refused");
    assert!(
        err.to_string()
            .contains("answered with something that is not embsim's handshake"),
        "{err}"
    );
}

#[rstest]
fn a_program_that_exits_before_its_handshake_is_reported_with_its_status_and_last_words() {
    behaviour!(Test {
        id: "p2-qemu.exit-before-hello",
        covers: Some("p2-qemu/src/peer.rs#Peer::start"),
        given: "a qemu-system-p2 that rejects the channel option and exits before its handshake, \
                as one built without embsim's host-driven mode does",
    });
    expect!(
        "status-and-stderr",
        "starting a P2 on it fails with the program's exit status and the last line it wrote \
         to standard error"
    );
    expect!(
        "how-to-install",
        "the error says the program is not one `embsim qemu install` built"
    );
    let err = P2Qemu::start(&fake("refuses-hostipc"), &[], &[], Transport::default())
        .expect_err("it exited");
    let text = err.to_string();
    assert!(
        text.contains("exited with status 1 before it said hello"),
        "{text}"
    );
    assert!(
        text.contains("Property 'p2-machine.hostipc' not found"),
        "{text}"
    );
    assert!(
        text.contains("is not one `embsim qemu install` built"),
        "{text}"
    );
}

#[rstest]
fn a_program_that_dies_mid_run_is_reported_with_its_status_and_last_words(
    #[values(Transport::Shm { spin_ns: 20_000 }, Transport::Socket)] transport: Transport,
) {
    behaviour!(Test {
        id: "p2-qemu.death-mid-run",
        covers: Some("p2-qemu/src/peer.rs#Peer::turn"),
        given: "a qemu-system-p2 that serves three runs and then exits, and another that is \
                killed after three",
    });
    expect!(
        "runs-served",
        "the runs before the death are answered with the instant the program reached"
    );
    expect!(
        "status-and-stderr",
        "the next run fails with the exit status, or the signal that killed it, and the last \
         line it wrote to standard error"
    );
    expect!(
        "noticed-promptly",
        "the death is noticed within a second, whether the node was spinning or blocked",
        "a blocked wait asks every 100 ms whether the program lives"
    );
    for (mode, status) in [
        ("dies-after:3", "exited with status 3"),
        ("killed-after:3", "was killed by signal 9 (SIGKILL)"),
    ] {
        let mut peer = Peer::start(&fake(mode), &[], &[], transport).expect("it starts");
        for horizon in [100, 200, 300] {
            let stop = peer.turn(&run_to(horizon)).expect("a run is served");
            assert_eq!(stop.now_clocks, horizon);
        }
        let asked = Instant::now();
        let err = peer.turn(&run_to(400)).expect_err("the program died");
        let noticed = asked.elapsed();
        let text = err.to_string();
        assert!(matches!(err, P2QemuError::Died { .. }), "{mode}: {text}");
        assert!(
            text.contains(&format!("{status} during a run")),
            "{mode}: {text}"
        );
        if mode.starts_with("dies") {
            assert!(
                text.contains("fake qemu-system-p2: dying after 3 runs, as asked"),
                "{text}"
            );
        }
        assert!(
            noticed < Duration::from_secs(1),
            "{mode}: noticed after {noticed:?}"
        );
        assert!(gone(peer.pid()), "{mode}: the program was reaped");
    }
}

#[rstest]
fn dropping_the_node_ends_a_program_that_will_not_quit(
    #[values(Transport::Shm { spin_ns: 20_000 }, Transport::Socket)] transport: Transport,
) {
    behaviour!(Test {
        id: "p2-qemu.drop-ends-program",
        covers: Some("p2-qemu/src/peer.rs#Peer::drop"),
        given: "a qemu-system-p2 that ignores the request to quit",
    });
    expect!(
        "ended",
        "when the P2 is dropped its program is gone, killed after a second's grace"
    );
    let peer = Peer::start(&fake("stubborn"), &[], &[], transport).expect("it starts");
    let pid = peer.pid();
    assert!(!gone(pid));
    let dropped = Instant::now();
    drop(peer);
    assert!(gone(pid), "pid {pid} outlived its node");
    assert!(
        dropped.elapsed() >= Duration::from_millis(900),
        "it was given its grace: {:?}",
        dropped.elapsed()
    );
}

/// What `embsim qemu path` prints: what the program says it is, whether or
/// not this embsim can use it.
#[rstest]
fn a_probe_reports_what_the_program_says_it_is() {
    let identity = fake("other-target").probe().expect("it answers");
    assert_eq!(
        identity,
        Identity {
            protocol: protocol::PROTOCOL,
            target: "0000000000000000".to_string(),
            qemu: target::qemu_pin().version().to_string(),
        }
    );
    assert!(identity.check().is_err());
    assert_eq!(
        fake("serve").probe().expect("it answers"),
        Identity::needed()
    );
}
