//! The node against a real QEMU, when one is installed.
//!
//! No disk, no firmware: the vCPU has nothing to run, but the guest's clock
//! runs, QMP answers, and the serial chardev exists — enough to prove the
//! spawn, the freeze/thaw channel and the slice loop against the real
//! thing. Skips, loudly, without QEMU on `PATH` (override the binary with
//! `EMBSIM_QEMU_BIN`).

use std::path::PathBuf;
use std::sync::{Mutex, MutexGuard};
use std::time::{Duration, Instant};

use embsim_board::System;
use embsim_core::virtual_clock::{self, ClockMode};
use embsim_qemu::{QemuNode, QemuSpec, SerialDevice};

fn alive(pid: u32) -> bool {
    // SAFETY: signal 0 checks for existence only; no signal is delivered.
    unsafe { libc::kill(pid as libc::pid_t, 0) == 0 }
}

static SUITE_LOCK: Mutex<()> = Mutex::new(());

fn suite_lock() -> MutexGuard<'static, ()> {
    SUITE_LOCK.lock().unwrap_or_else(|poisoned| {
        SUITE_LOCK.clear_poison();
        poisoned.into_inner()
    })
}

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

/// `EMBSIM_QEMU_BIN`, else the first `qemu-system-aarch64` on `PATH`.
fn qemu_binary() -> Option<PathBuf> {
    if let Ok(bin) = std::env::var("EMBSIM_QEMU_BIN") {
        return Some(PathBuf::from(bin));
    }
    let path = std::env::var_os("PATH")?;
    std::env::split_paths(&path)
        .map(|dir| dir.join("qemu-system-aarch64"))
        .find(|candidate| candidate.is_file())
}

fn wait_for(mut pred: impl FnMut() -> bool, timeout: Duration) -> bool {
    let start = Instant::now();
    while start.elapsed() < timeout {
        if pred() {
            return true;
        }
        std::thread::sleep(Duration::from_millis(10));
    }
    pred()
}

#[test]
fn a_real_qemu_is_frozen_between_slices_and_metered_within_them() {
    let Some(binary) = qemu_binary() else {
        eprintln!(
            "*** SKIPPED: a_real_qemu_is_frozen_between_slices_and_metered_within_them needs \
             qemu-system-aarch64 on PATH (or EMBSIM_QEMU_BIN). *** This test asserted NOTHING."
        );
        return;
    };
    let _suite = suite_lock();
    let _stepped = Stepped::enter();

    // TCG so it runs on any host and in CI. The USB serial is attached even
    // though no guest kernel will enumerate it: that exercises the device
    // arguments QEMU has to accept.
    let vm = QemuSpec::new(binary)
        .args([
            "-M",
            "virt",
            "-accel",
            "tcg",
            "-cpu",
            "cortex-a57",
            "-m",
            "64M",
        ])
        .serial(SerialDevice::UsbFtdi)
        .spawn()
        .expect("QEMU spawns");
    let pid = vm.pid();
    let mut control = vm.control().expect("control QMP connects");
    let born = control.query_status().expect("query-status");
    assert!(!born.running, "a guest is born frozen, got {born:?}");

    let node = QemuNode::new(Box::new(vm), 115_200).with_slice(Duration::from_millis(20));
    let stats = node.stats();
    let system = System::new()
        .component("PC", Box::new(node))
        .start()
        .expect("system starts");

    assert!(
        wait_for(|| stats.slices() >= 5, Duration::from_secs(10)),
        "expected 5 slices, got {}",
        stats.slices()
    );
    // Freeze/thaw over real QMP: the guest lived about as long as the board
    // advanced, and not a slice more.
    let skew = stats.skew_ns();
    assert!(
        skew.unsigned_abs() <= 20_000_000 + 5_000_000,
        "skew {skew} ns exceeds a slice (virtual {} ns, guest {} ns)",
        stats.virtual_ns(),
        stats.guest_ns()
    );
    assert!(!stats.disconnected());

    drop(system);
    // The board's engine is joined only once every actor has parked, and the
    // node parks only after freezing the guest; dropping the node then
    // drops the guest, which quits QEMU. No clock re-init, no orphan.
    assert!(
        wait_for(|| !alive(pid), Duration::from_secs(5)),
        "QEMU (pid {pid}) outlived the node"
    );
    assert!(
        control.query_status().is_err(),
        "the control socket survived QEMU"
    );
}
