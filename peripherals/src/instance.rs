//! Instance — per-MCU peripheral state and thread-identity routing.
//!
//! Historically every peripheral module in this crate kept its state in
//! process-global `static`s, which hard-limited a process to exactly one
//! emulated MCU. [`PeripheralInstance`] owns all of that state instead: one
//! value per emulated MCU, with each peripheral module's logic exposed as
//! methods on a per-instance struct (`serial::Serial`, `gpio::Gpio`, …).
//!
//! ## Routing model
//!
//! The module-level free functions (`serial::transmit_data`, …) — and
//! therefore the platform crates' `#[no_mangle]` HAL trampolines — resolve
//! the *calling thread's* instance and delegate:
//!
//! 1. a thread bound via [`bind_current_thread`] uses its bound instance;
//! 2. firmware threads spawned through `system::start_thread` inherit their
//!    creator's instance (the spawn path binds the new thread before the
//!    thread body runs);
//! 3. any unbound thread falls back to the process-wide [`default`] instance,
//!    which is created lazily and preserves the historical single-MCU
//!    behavior exactly.
//!
//! Bindings are tracked in a process-wide thread-identity map
//! ([`registered`]) plus a thread-local cache for the hot path. A thread's
//! binding can only be changed *by that thread itself* (there is no
//! bind-by-`ThreadId` API), which is what keeps the thread-local cache
//! coherent by construction.
//!
//! ## Limits
//!
//! Instance routing de-globalizes the *Rust-side* peripheral state only. A
//! given firmware image's own C statics still limit that image to one
//! instance per process — two instances of the *same* firmware would share
//! its `.data`/`.bss`. Multiple instances are for multi-MCU systems built
//! from distinct images (or pure-Rust components), per `BOARD_ENGINE.md`.

use std::cell::RefCell;
use std::collections::HashMap;
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{Arc, LazyLock, Mutex, OnceLock};
use std::thread::{self, ThreadId};

use crate::{encoder, filesystem, gpio, lock, pulse_out, serial, system};

/// All peripheral state for one emulated MCU.
///
/// Fields are public so a multi-instance consumer (e.g. an MCU component in a
/// board engine) can drive a specific instance directly:
/// `inst.serial.transmit_data(0, b"…")`. Single-MCU consumers keep using the
/// module-level free functions, which route here via [`current`].
pub struct PeripheralInstance {
    /// FD-bridged serial channel bank.
    pub serial: serial::Serial,
    /// GPIO channel bank.
    pub gpio: gpio::Gpio,
    /// Encoder counter bank.
    pub encoder: encoder::Encoder,
    /// Pulse-output channel bank.
    pub pulse_out: pulse_out::PulseOut,
    /// Hardware-lock pool.
    pub locks: lock::LockPool,
    /// Thread/core registry.
    pub system: system::System,
    /// Filesystem mount configuration.
    pub filesystem: filesystem::Filesystem,
    /// Per-MCU clock frequency override in Hz. `0` = follow the process-wide
    /// `embsim_core::virtual_clock` frequency (the historical behavior).
    clock_freq: AtomicU32,
}

impl PeripheralInstance {
    /// Create a fresh instance with every peripheral in its power-on state
    /// (no channels configured, nothing connected).
    pub fn new() -> Self {
        Self {
            serial: serial::Serial::new(),
            gpio: gpio::Gpio::new(),
            encoder: encoder::Encoder::new(),
            pulse_out: pulse_out::PulseOut::new(),
            locks: lock::LockPool::new(),
            system: system::System::new(),
            filesystem: filesystem::Filesystem::new(),
            clock_freq: AtomicU32::new(0),
        }
    }

    /// Set this instance's clock frequency override in Hz (`0` clears it,
    /// falling back to the process-wide virtual clock's frequency).
    pub fn set_clock_freq(&self, freq: u32) {
        self.clock_freq.store(freq, Ordering::Relaxed);
    }

    /// The clock frequency this instance observes: its own override if set,
    /// otherwise the process-wide `virtual_clock` frequency.
    pub fn effective_clock_freq(&self) -> u32 {
        let own = self.clock_freq.load(Ordering::Relaxed);
        if own != 0 {
            own
        } else {
            embsim_core::virtual_clock::clock_freq()
        }
    }
}

impl Default for PeripheralInstance {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================
// Registry — default singleton + thread-identity map
// ============================================================

/// Lazily-created default instance (the historical process globals).
static DEFAULT: OnceLock<Arc<PeripheralInstance>> = OnceLock::new();

/// Explicit thread bindings (thread id → instance). Only ever mutated by the
/// bound thread itself, via [`bind_current_thread`] / guard drop.
static REGISTRY: LazyLock<Mutex<HashMap<ThreadId, Arc<PeripheralInstance>>>> =
    LazyLock::new(|| Mutex::new(HashMap::new()));

thread_local! {
    /// Per-thread cache of the resolved instance (bound or default fallback).
    /// Coherent because a binding can only be changed by the owning thread.
    static CURRENT: RefCell<Option<Arc<PeripheralInstance>>> = const { RefCell::new(None) };
}

/// The process-wide default instance, created lazily on first use — exactly
/// as the former module globals were. Unbound threads route here.
pub fn default() -> Arc<PeripheralInstance> {
    Arc::clone(DEFAULT.get_or_init(|| Arc::new(PeripheralInstance::new())))
}

/// Resolve the calling thread's instance: its explicit binding if any,
/// otherwise the [`default`] singleton.
pub fn current() -> Arc<PeripheralInstance> {
    // Fast path: thread-local cache (also holds the default fallback).
    // `try_with` so calls during thread teardown degrade to the slow path
    // instead of panicking on a destroyed thread-local.
    if let Ok(Some(inst)) = CURRENT.try_with(|c| c.borrow().clone()) {
        return inst;
    }
    let resolved = registered(thread::current().id()).unwrap_or_else(default);
    let _ = CURRENT.try_with(|c| *c.borrow_mut() = Some(Arc::clone(&resolved)));
    resolved
}

/// Look up the explicit binding for a thread, if any (diagnostics / tests).
pub fn registered(thread: ThreadId) -> Option<Arc<PeripheralInstance>> {
    REGISTRY.lock().unwrap().get(&thread).cloned()
}

/// Bind the calling thread to `inst` until the returned guard drops, at which
/// point the previous binding (or none) is restored. All peripheral free
/// functions called on this thread while the guard lives route to `inst`.
///
/// Guards nest, and nested guards **must be dropped in LIFO order**
/// (innermost first — the natural scope order). Each guard restores the
/// binding that was current when *it* was created, so an out-of-order drop
/// would resurrect a stale binding; the guard's `Drop` detects this and
/// panics instead of silently corrupting the thread's routing.
///
/// `system::start_thread` uses this internally so firmware threads inherit
/// their creator's instance; consumers standing up an MCU component bind the
/// thread that runs the firmware entry point.
#[must_use = "the binding is dropped (and the previous one restored) when the guard drops"]
pub fn bind_current_thread(inst: Arc<PeripheralInstance>) -> InstanceGuard {
    let id = thread::current().id();
    let prev_map = REGISTRY.lock().unwrap().insert(id, Arc::clone(&inst));
    let prev_local = CURRENT
        .try_with(|c| c.borrow_mut().replace(Arc::clone(&inst)))
        .unwrap_or(None);
    InstanceGuard {
        thread: id,
        bound: inst,
        prev_map,
        prev_local,
        _not_send: std::marker::PhantomData,
    }
}

/// RAII binding of a thread to a [`PeripheralInstance`]; restores the
/// previous binding on drop. Created by [`bind_current_thread`].
///
/// Nested guards must be dropped in **LIFO order** (see
/// [`bind_current_thread`]); dropping a guard while a later one is still
/// live panics rather than silently restoring a stale binding.
///
/// `!Send` by construction: the drop handler restores the *creating* thread's
/// thread-local cache, so the guard must be dropped on the thread it bound.
/// This is what makes the "a thread's binding can only be changed by that
/// thread itself" invariant (see module docs) true by the type system rather
/// than by convention.
pub struct InstanceGuard {
    thread: ThreadId,
    /// The instance this guard installed — checked on drop to detect
    /// non-LIFO drops (the current binding must still be this one).
    bound: Arc<PeripheralInstance>,
    prev_map: Option<Arc<PeripheralInstance>>,
    prev_local: Option<Arc<PeripheralInstance>>,
    /// Keeps the guard `!Send`/`!Sync` so it cannot be dropped on another
    /// thread (which would poison that thread's `CURRENT` cache).
    _not_send: std::marker::PhantomData<*mut ()>,
}

impl Drop for InstanceGuard {
    fn drop(&mut self) {
        let mut map = REGISTRY.lock().unwrap();
        // LIFO check: the thread's current binding must still be the one this
        // guard installed. If a later guard is live, restoring `prev` here
        // would drop that guard's binding now and resurrect a stale one when
        // it drops — a silent corruption. Misuse bug: fail loud.
        let is_current = map
            .get(&self.thread)
            .is_some_and(|cur| Arc::ptr_eq(cur, &self.bound));
        if !is_current {
            drop(map); // never panic while holding the registry lock
            if thread::panicking() {
                return; // avoid a double-panic abort; the thread is dying
            }
            panic!(
                "InstanceGuard dropped out of LIFO order: the calling thread's \
                 current binding is not the one this guard installed (a guard \
                 created later is still live); drop nested guards innermost-first"
            );
        }
        match self.prev_map.take() {
            Some(prev) => {
                map.insert(self.thread, prev);
            }
            None => {
                map.remove(&self.thread);
            }
        }
        drop(map);
        let prev_local = self.prev_local.take();
        let _ = CURRENT.try_with(|c| *c.borrow_mut() = prev_local);
    }
}

// ============================================================
// Tests
// ============================================================

#[cfg(test)]
mod tests {
    use super::*;
    use nix::libc;
    use std::os::fd::{BorrowedFd, RawFd};
    use std::sync::atomic::AtomicUsize;

    /// A connected, non-blocking AF_UNIX socket pair: `a` is wired into a
    /// serial channel, `b` is the far end the test observes. Closed on drop.
    struct Pair {
        a: RawFd,
        b: RawFd,
    }

    impl Pair {
        fn new() -> Self {
            let mut fds = [0i32; 2];
            let rc =
                unsafe { libc::socketpair(libc::AF_UNIX, libc::SOCK_STREAM, 0, fds.as_mut_ptr()) };
            assert_eq!(rc, 0, "socketpair failed");
            for &fd in &fds {
                let flags = unsafe { libc::fcntl(fd, libc::F_GETFL) };
                let rc = unsafe { libc::fcntl(fd, libc::F_SETFL, flags | libc::O_NONBLOCK) };
                assert_eq!(rc, 0, "fcntl O_NONBLOCK failed");
            }
            Pair {
                a: fds[0],
                b: fds[1],
            }
        }

        fn write_far(&self, data: &[u8]) {
            let fd = unsafe { BorrowedFd::borrow_raw(self.b) };
            let mut off = 0;
            while off < data.len() {
                off += nix::unistd::write(fd, &data[off..]).expect("write_far");
            }
        }

        /// Non-blocking read of up to `n` bytes; empty vec when nothing waits.
        fn read_far(&self, n: usize) -> Vec<u8> {
            let mut buf = vec![0u8; n];
            match nix::unistd::read(self.b, &mut buf) {
                Ok(read) => {
                    buf.truncate(read);
                    buf
                }
                Err(nix::errno::Errno::EAGAIN) => Vec::new(),
                Err(e) => panic!("read_far: {e}"),
            }
        }
    }

    impl Drop for Pair {
        fn drop(&mut self) {
            unsafe {
                libc::close(self.a);
                libc::close(self.b);
            }
        }
    }

    #[test]
    fn unbound_thread_falls_back_to_the_default_singleton() {
        let _g = crate::test_support::guard();
        crate::test_support::ensure_clock();
        // The libtest worker thread is unbound → current() is the default, and
        // repeated calls return the same instance.
        assert!(Arc::ptr_eq(&current(), &default()));
        assert!(Arc::ptr_eq(&default(), &default()));
    }

    #[test]
    fn two_instances_have_independent_serial_channel_state() {
        let _g = crate::test_support::guard();
        crate::test_support::ensure_clock();
        // Pure-Rust two-MCU scenario: no firmware, no free functions — each
        // instance's serial bank is driven directly and never sees the other's.
        let a = PeripheralInstance::new();
        let b = PeripheralInstance::new();
        a.serial.init(2);
        b.serial.init(1);

        let wire_a = Pair::new();
        a.serial.init_channel_fd(0, wire_a.a);
        // b's channel 0 stays unconnected (fd -1).

        // TX on A reaches A's wire; the same call on B is silently discarded.
        a.serial.transmit_data(0, b"AA");
        assert_eq!(wire_a.read_far(2), b"AA");
        b.serial.transmit_data(0, b"BB");
        assert!(wire_a.read_far(2).is_empty(), "B's TX never crosses to A");

        // RX likewise: bytes on A's wire are invisible to B.
        wire_a.write_far(&[0x5A]);
        assert_eq!(b.serial.receive_byte(0), None);
        assert_eq!(a.serial.receive_byte(0), Some(0x5A));

        // Per-instance config: pacing A's channel leaves B unpaced, and
        // resetting B does not disconnect A.
        a.serial.set_baud(0, 1_000_000);
        b.serial.reset();
        a.serial.transmit_data(0, b"ok");
        assert_eq!(wire_a.read_far(2), b"ok");
    }

    #[test]
    fn bound_thread_routes_free_functions_to_its_instance() {
        let _g = crate::test_support::guard();
        crate::test_support::ensure_clock();
        // The default instance gets its own wired channel; a bound thread's
        // free-function TX must land on the bound instance's wire, not this one.
        crate::serial::init(1);
        let wire_default = Pair::new();
        crate::serial::init_channel_fd(0, wire_default.a);

        let inst = Arc::new(PeripheralInstance::new());
        inst.serial.init(1);
        let wire_inst = Pair::new();
        inst.serial.init_channel_fd(0, wire_inst.a);

        let for_thread = Arc::clone(&inst);
        std::thread::spawn(move || {
            let _bind = bind_current_thread(Arc::clone(&for_thread));
            assert!(Arc::ptr_eq(&current(), &for_thread), "bound identity");
            crate::serial::transmit_data(0, b"xy");
        })
        .join()
        .expect("bound thread");

        assert_eq!(wire_inst.read_far(2), b"xy", "TX routed to bound instance");
        assert!(
            wire_default.read_far(2).is_empty(),
            "default instance untouched"
        );
        // Leave the default serial bank clean for sibling tests.
        crate::serial::reset();
    }

    #[test]
    fn dropping_the_guard_restores_the_previous_binding() {
        let _g = crate::test_support::guard();
        crate::test_support::ensure_clock();
        std::thread::spawn(|| {
            let outer = Arc::new(PeripheralInstance::new());
            let inner = Arc::new(PeripheralInstance::new());
            let id = thread::current().id();

            let g1 = bind_current_thread(Arc::clone(&outer));
            assert!(Arc::ptr_eq(&current(), &outer));
            {
                let _g2 = bind_current_thread(Arc::clone(&inner));
                assert!(Arc::ptr_eq(&current(), &inner), "nested bind wins");
                assert!(Arc::ptr_eq(&registered(id).unwrap(), &inner));
            }
            // Inner guard dropped → outer binding restored (local + map).
            assert!(Arc::ptr_eq(&current(), &outer), "outer restored");
            assert!(Arc::ptr_eq(&registered(id).unwrap(), &outer));
            drop(g1);
            // Fully unbound → default fallback, and the map entry is gone.
            assert!(Arc::ptr_eq(&current(), &default()));
            assert!(registered(id).is_none(), "map entry removed");
        })
        .join()
        .expect("guard thread");
    }

    /// Dropping guards out of LIFO order is a misuse bug: without the check,
    /// the first drop would unbind the still-guarded instance and the second
    /// would resurrect the first's — leaving the thread bound with no live
    /// guard. The drop handler must panic instead.
    #[test]
    #[should_panic(expected = "InstanceGuard dropped out of LIFO order")]
    fn non_lifo_guard_drop_panics() {
        let _g = crate::test_support::guard();
        crate::test_support::ensure_clock();
        let g1 = bind_current_thread(Arc::new(PeripheralInstance::new()));
        let _g2 = bind_current_thread(Arc::new(PeripheralInstance::new()));
        drop(g1); // out of order: `_g2` is still live → panics
    }

    #[test]
    fn threads_spawned_via_system_inherit_the_creators_instance() {
        let _g = crate::test_support::guard();
        crate::test_support::ensure_clock();

        /// Records the identity of the spawned thread's resolved instance.
        static SEEN: AtomicUsize = AtomicUsize::new(0);
        unsafe extern "C" fn record_instance(_arg: *mut std::ffi::c_void) {
            SEEN.store(Arc::as_ptr(&current()) as usize, Ordering::SeqCst);
        }

        let inst = Arc::new(PeripheralInstance::new());
        inst.system.init(4);
        SEEN.store(0, Ordering::SeqCst);

        // A "creator" thread bound to `inst` spawns a firmware thread through
        // the free function; the child must resolve the same instance.
        let creator_inst = Arc::clone(&inst);
        std::thread::spawn(move || {
            let _bind = bind_current_thread(Arc::clone(&creator_inst));
            let id =
                unsafe { crate::system::start_thread(Some(record_instance), std::ptr::null_mut()) };
            assert!(id >= 0, "spawn succeeded");
            // Still bound → join routes to creator_inst's own handle table.
            crate::system::join_all_threads();
        })
        .join()
        .expect("creator thread");

        assert_eq!(
            SEEN.load(Ordering::SeqCst),
            Arc::as_ptr(&inst) as usize,
            "spawned thread inherited its creator's instance"
        );
        // The handle table lives in `inst`, not the default instance.
        inst.system.reset();
    }

    #[test]
    fn clock_freq_override_is_per_instance_with_global_fallback() {
        let _g = crate::test_support::guard();
        crate::test_support::ensure_clock();
        let inst = PeripheralInstance::new();
        // Unset → follows the process-wide virtual clock (pinned at 180 MHz).
        assert_eq!(
            inst.effective_clock_freq(),
            embsim_core::virtual_clock::clock_freq()
        );
        // Override applies to this instance only.
        inst.set_clock_freq(320_000_000);
        assert_eq!(inst.effective_clock_freq(), 320_000_000);
        assert_eq!(
            default().effective_clock_freq(),
            embsim_core::virtual_clock::clock_freq(),
            "default instance unaffected"
        );
        // Clearing restores the fallback.
        inst.set_clock_freq(0);
        assert_eq!(
            inst.effective_clock_freq(),
            embsim_core::virtual_clock::clock_freq()
        );
    }
}
