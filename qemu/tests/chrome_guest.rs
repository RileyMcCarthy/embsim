//! The Chrome guest under the node, when its image has been built.
//!
//! Boots the image (`guest/chrome/build.sh`), lets it warm up on host time
//! until DevTools answers, then puts it on the board at 1 ms slices and
//! checks the two things phase 2 adds: the agent's clock keeps the books
//! exact (no drift, not just bounded skew), and Chrome stays reachable while
//! it lives on the board's time. Skips, loudly, without the image
//! (`EMBSIM_CHROME_IMAGE` or the default cache path) or without QEMU.

use std::path::PathBuf;
use std::sync::{Mutex, MutexGuard};
use std::time::{Duration, Instant};

use embsim_board::System;
use embsim_core::virtual_clock::{self, ClockMode};
use embsim_qemu::{ChromeGuest, QemuNode};

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

fn image() -> Option<PathBuf> {
    if let Ok(p) = std::env::var("EMBSIM_CHROME_IMAGE") {
        return Some(PathBuf::from(p)).filter(|p| p.is_file());
    }
    let cache = std::env::var_os("EMBSIM_QEMU_CACHE")
        .map(PathBuf::from)
        .or_else(|| {
            std::env::var_os("XDG_CACHE_HOME").map(|c| PathBuf::from(c).join("embsim/qemu"))
        })
        .or_else(|| {
            std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".cache/embsim/qemu"))
        })?;
    let arch = std::env::consts::ARCH;
    Some(cache.join(format!("chrome-debian13-{arch}.qcow2"))).filter(|p| p.is_file())
}

fn qemu_on_path() -> bool {
    let name = format!("qemu-system-{}", std::env::consts::ARCH);
    std::env::var_os("PATH")
        .map(|p| std::env::split_paths(&p).any(|d| d.join(&name).is_file()))
        .unwrap_or(false)
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
fn the_chrome_guest_keeps_exact_time_on_the_board_and_stays_reachable() {
    let Some(image) = image() else {
        eprintln!(
            "*** SKIPPED: the_chrome_guest_keeps_exact_time_on_the_board_and_stays_reachable \
             needs the Chrome guest image (guest/chrome/build.sh, or EMBSIM_CHROME_IMAGE). \
             *** This test asserted NOTHING."
        );
        return;
    };
    if !qemu_on_path() {
        eprintln!(
            "*** SKIPPED: needs qemu-system-{} on PATH. *** This test asserted NOTHING.",
            std::env::consts::ARCH
        );
        return;
    }
    let _suite = suite_lock();
    let _stepped = Stepped::enter();

    let booted = Instant::now();
    let chrome = ChromeGuest::new(&image)
        .spawn()
        .expect("the Chrome guest boots and answers DevTools");
    let devtools = chrome.devtools().clone();
    eprintln!(
        "guest warm in {:?}; DevTools at {}",
        booted.elapsed(),
        devtools.url()
    );
    assert!(
        chrome.vm().has_agent(),
        "the image's agent port should be attached"
    );

    let node = QemuNode::new(Box::new(chrome), 2_000_000).with_slice(Duration::from_millis(1));
    let stats = node.stats();
    let system = System::new()
        .component("PC", Box::new(node))
        .start()
        .expect("system starts");

    // Chrome answers over TCP while metered — every packet needs guest
    // time, and it gets it a millisecond at a time.
    assert!(
        wait_for(|| devtools.is_up(), Duration::from_secs(30)),
        "DevTools did not answer while the guest lived on the board's clock"
    );

    assert!(
        wait_for(|| stats.slices() >= 1000, Duration::from_secs(60)),
        "expected 1000 slices, got {}",
        stats.slices()
    );
    // The agent answered: the books come from the guest's own clock.
    assert!(
        stats.clocked() >= 900,
        "only {} of {} slices were clocked by the agent",
        stats.clocked(),
        stats.slices()
    );
    // Exact accounting: after a thousand slices the skew is still within a
    // slice or two, not the ~1 % drift a stopwatch alone accumulates
    // (which over 1000 ms would be ~10 ms).
    let skew = stats.skew_ns();
    assert!(
        skew.unsigned_abs() <= 3_000_000,
        "skew {skew} ns after {} slices (virtual {} ns, guest {} ns)",
        stats.slices(),
        stats.virtual_ns(),
        stats.guest_ns()
    );
    assert_eq!(stats.shed(), 0);
    assert!(!stats.disconnected());

    drop(system);
}
