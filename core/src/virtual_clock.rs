//! Quantum virtual clock — Renode-style synchronized simulation time.
//!
//! Virtual time is a monotonic microsecond **counter**. It does not track the
//! host wall clock. It advances only at a **quantum barrier**:
//!
//! 1. Firmware cores (and any other [`participate`]rs) run.
//! 2. They park on [`wait_us`] / [`wait_until`] (or drop their
//!    [`Participation`]).
//! 3. When nobody is running, the clock jumps to
//!    `min(next_wake, now + quantum_us)` and every waiter is woken.
//!
//! That is the same shape as Renode's time framework (cores consume a quantum,
//! then all executors synchronize) adapted to host-compiled firmware, which
//! cannot count guest cycles. Compute is treated as instantaneous; only waits
//! and scheduled deadlines move time. Idle jumps are capped by the quantum so
//! peripherals (net-engine timer wheel, UART pacing, ADC pumps) stay in lockstep
//! the way Renode's multi-core sync does.
//!
//! [`pause`] freezes the counter. Optional **wall pacing** (the old `--speed`
//! scale) sleeps *after* an advance so an interactive playground still feels
//! real-time; tests should call [`init`] with `speed = 0.0` so jumps are
//! instant and deterministic.
//!
//! Threads that never wait and never park hold the barrier open (time cannot
//! jump). Native SIL cannot preempt host machine code the way an ISS can, so
//! the emulator stops a cog at the next HAL trampoline via [`charge`]: after
//! a quantum of HAL work the thread parks. Firmware stays free to spin
//! (`LOCKTRY`, UART poll) exactly as on silicon. A cog that never hits HAL
//! (`while (1) {}`) still freezes time — that needs an ISS.

use std::cell::Cell;
use std::collections::{BTreeMap, HashMap};
use std::sync::{Condvar, Mutex, MutexGuard};
use std::time::Duration;

/// Default quantum: 1 ms of virtual time, a common Renode slice.
pub const DEFAULT_QUANTUM_US: u64 = 1_000;

thread_local! {
    static PARTICIPANT: Cell<Option<u32>> = const { Cell::new(None) };
    /// HAL-proxy work charged against this thread's quantum. Reset by
    /// [`wait_until`] / [`participate`].
    static SLICE_USED_US: Cell<u64> = const { Cell::new(0) };
}

struct Participant {
    /// `None` = running (holds the barrier). `Some(t)` = parked until `t`.
    parked_until: Option<u64>,
}

struct State {
    now_us: u64,
    freq: u32,
    quantum_us: u64,
    paused: bool,
    /// Pacing: wall_sleep = dt_virtual * denom / numer. `numer == 0` means
    /// unpaced (tests / CI).
    pace_numer: u64,
    pace_denom: u64,
    next_id: u32,
    parts: HashMap<u32, Participant>,
    /// Count of waiters (cores and anonymous) armed at each deadline.
    deadlines: BTreeMap<u64, u32>,
    /// Bumped by [`kick`] so [`wait_until_or_kicked`] can return early.
    epoch: u64,
}

static STATE: Mutex<Option<State>> = Mutex::new(None);
static CV: Condvar = Condvar::new();

fn lock_state() -> MutexGuard<'static, Option<State>> {
    STATE.lock().unwrap_or_else(|p| {
        STATE.clear_poison();
        p.into_inner()
    })
}

fn pace_from_speed(speed: f64) -> (u64, u64) {
    if speed <= 0.0 {
        (0, 1)
    } else {
        ((speed * 1000.0) as u64, 1000)
    }
}

/// Sleep `dt` virtual microseconds of wall time according to the pacing scale.
fn apply_pace(dt: u64, numer: u64, denom: u64) {
    if numer == 0 || dt == 0 {
        return;
    }
    let wall_us = dt.saturating_mul(denom) / numer.max(1);
    if wall_us > 0 {
        std::thread::sleep(Duration::from_micros(wall_us));
    }
}

fn mark_parked(st: &mut State, until: u64) {
    if let Some(id) = PARTICIPANT.get() {
        if let Some(p) = st.parts.get_mut(&id) {
            p.parked_until = Some(until);
        }
    }
}

fn mark_running(st: &mut State) {
    if let Some(id) = PARTICIPANT.get() {
        if let Some(p) = st.parts.get_mut(&id) {
            p.parked_until = None;
        }
    }
}

fn arm_deadline(st: &mut State, t: u64) {
    *st.deadlines.entry(t).or_insert(0) += 1;
}

fn disarm_deadline(st: &mut State, t: u64) {
    if let Some(c) = st.deadlines.get_mut(&t) {
        *c = c.saturating_sub(1);
        if *c == 0 {
            st.deadlines.remove(&t);
        }
    }
}

/// If every participant is parked (or there are none), jump `now` to
/// `min(next deadline, now + quantum)`. Returns the virtual µs advanced, which
/// the caller paces *after* releasing the lock.
fn try_advance(st: &mut State) -> u64 {
    if st.paused {
        return 0;
    }
    if st.parts.values().any(|p| p.parked_until.is_none()) {
        return 0;
    }
    // No armed deadline → nothing to jump to (do not idle-spin quanta).
    let Some((&t, _)) = st.deadlines.iter().next() else {
        return 0;
    };
    let next = t.min(st.now_us.saturating_add(st.quantum_us));
    if next <= st.now_us {
        return 0;
    }
    let dt = next - st.now_us;
    st.now_us = next;
    dt
}

fn commit_after_unlock(dt: u64, numer: u64, denom: u64) {
    apply_pace(dt, numer, denom);
    CV.notify_all();
}

/// Initialize (or re-anchor) the clock.
///
/// `speed > 0` enables wall pacing at that scale (1.0 = real-time feel after
/// each jump; 5.0 = five times faster). `speed <= 0` is **unpaced**: jumps are
/// instant and fully deterministic — use this in tests.
///
/// Re-init resets `now` to 0, clears participants, and unpauses. The simulated
/// clock frequency `freq` is used by [`virtual_cycles`].
pub fn init(speed: f64, freq: u32) {
    init_with_quantum(speed, freq, DEFAULT_QUANTUM_US);
}

/// [`init`] with an explicit quantum (virtual microseconds).
pub fn init_with_quantum(speed: f64, freq: u32, quantum_us: u64) {
    let (pace_numer, pace_denom) = pace_from_speed(speed);
    let mut g = lock_state();
    *g = Some(State {
        now_us: 0,
        freq,
        quantum_us: quantum_us.max(1),
        paused: false,
        pace_numer,
        pace_denom,
        next_id: 1,
        parts: HashMap::new(),
        deadlines: BTreeMap::new(),
        epoch: 0,
    });
    drop(g);
    CV.notify_all();
}

/// Change wall-pacing scale. `scale <= 0` disables pacing.
pub fn set_scale(scale: f64) {
    let (pace_numer, pace_denom) = pace_from_speed(scale);
    if let Some(st) = lock_state().as_mut() {
        st.pace_numer = pace_numer;
        st.pace_denom = pace_denom;
    }
}

/// Set the quantum cap (virtual µs). Minimum 1.
pub fn set_quantum_us(quantum_us: u64) {
    let mut g = lock_state();
    let Some(st) = g.as_mut() else { return };
    st.quantum_us = quantum_us.max(1);
    let dt = try_advance(st);
    let numer = st.pace_numer;
    let denom = st.pace_denom;
    drop(g);
    commit_after_unlock(dt, numer, denom);
}

/// Current quantum in virtual microseconds.
pub fn quantum_us() -> u64 {
    lock_state()
        .as_ref()
        .map(|s| s.quantum_us)
        .unwrap_or(DEFAULT_QUANTUM_US)
}

/// Freeze time. Waiters whose deadline is not yet reached stay parked.
pub fn pause() {
    if let Some(st) = lock_state().as_mut() {
        st.paused = true;
    }
}

/// Resume time and attempt a quantum advance.
pub fn resume() {
    let mut g = lock_state();
    let Some(st) = g.as_mut() else { return };
    st.paused = false;
    let dt = try_advance(st);
    let numer = st.pace_numer;
    let denom = st.pace_denom;
    drop(g);
    commit_after_unlock(dt, numer, denom);
}

/// True when [`pause`] is in effect.
pub fn is_paused() -> bool {
    lock_state().as_ref().is_some_and(|s| s.paused)
}

/// True once [`init`] has run in this process (including after re-init).
pub fn is_initialized() -> bool {
    lock_state().is_some()
}

/// Current virtual microseconds since the last [`init`].
pub fn virtual_us() -> u64 {
    lock_state()
        .as_ref()
        .expect("Virtual clock not initialized")
        .now_us
}

/// Current virtual milliseconds since the last [`init`].
pub fn virtual_ms() -> u64 {
    virtual_us() / 1000
}

/// Simulated clock frequency in Hz.
pub fn clock_freq() -> u32 {
    lock_state()
        .as_ref()
        .map(|s| s.freq)
        .expect("Virtual clock not initialized")
}

/// Virtual cycle count (`now_us * freq / 1_000_000`).
pub fn virtual_cycles() -> u64 {
    let g = lock_state();
    let st = g.as_ref().expect("Virtual clock not initialized");
    (st.now_us as u128 * st.freq as u128 / 1_000_000) as u64
}

/// Convert a virtual duration to a wall-clock sleep for **pacing only**.
/// Unpaced clocks (`speed <= 0`) return 0.
pub fn virtual_to_wall_us(virtual_wait_us: u64) -> u64 {
    let g = lock_state();
    let Some(st) = g.as_ref() else {
        return virtual_wait_us;
    };
    if st.pace_numer == 0 {
        return 0;
    }
    virtual_wait_us.saturating_mul(st.pace_denom) / st.pace_numer.max(1)
}

/// RAII core registration. While held, this thread is a **time participant**:
/// time cannot jump until it parks ([`wait_until`]) or the guard is dropped.
///
/// Firmware threads spawned via `system::start_thread` take one for their
/// lifetime. The emulator entry thread should too.
pub struct Participation {
    id: u32,
}

impl Drop for Participation {
    fn drop(&mut self) {
        PARTICIPANT.set(None);
        SLICE_USED_US.set(0);
        let mut g = lock_state();
        let Some(st) = g.as_mut() else { return };
        st.parts.remove(&self.id);
        let dt = try_advance(st);
        let numer = st.pace_numer;
        let denom = st.pace_denom;
        drop(g);
        commit_after_unlock(dt, numer, denom);
    }
}

/// Register the current thread as a time participant until the guard drops.
///
/// Panics if this thread already participates (nested participate is not
/// supported).
pub fn participate() -> Participation {
    assert!(
        PARTICIPANT.get().is_none(),
        "virtual_clock::participate: this thread is already a participant"
    );
    let mut g = lock_state();
    let st = g
        .as_mut()
        .expect("virtual_clock::init must run before participate");
    let id = st.next_id;
    st.next_id = st.next_id.wrapping_add(1).max(1);
    st.parts.insert(id, Participant { parked_until: None });
    drop(g);
    PARTICIPANT.set(Some(id));
    SLICE_USED_US.set(0);
    Participation { id }
}

/// Park until virtual time reaches `deadline_us` (absolute).
pub fn wait_until(deadline_us: u64) {
    let _ = wait_inner(deadline_us, false);
}

/// Like [`wait_until`], but returns `false` if [`kick`] woke the caller before
/// the deadline (so a net-engine thread can drain commands).
pub fn wait_until_or_kicked(deadline_us: u64) -> bool {
    wait_inner(deadline_us, true)
}

fn wait_inner(deadline_us: u64, return_on_kick: bool) -> bool {
    SLICE_USED_US.set(0);
    let mut g = lock_state();
    let start_epoch = g.as_ref().map(|s| s.epoch).unwrap_or(0);
    arm_deadline(
        g.as_mut().expect("virtual_clock::init must run before wait"),
        deadline_us,
    );
    let reached = loop {
        let st = g.as_mut().expect("virtual_clock::init must run before wait");
        if st.now_us >= deadline_us {
            mark_running(st);
            break true;
        }
        if return_on_kick && st.epoch != start_epoch {
            // Still try to commit a quantum: a flood of kicks must not
            // freeze time (the net engine polls commands AND timers).
            let dt = try_advance(st);
            let numer = st.pace_numer;
            let denom = st.pace_denom;
            let reached_now = st.now_us >= deadline_us;
            mark_running(st);
            if dt > 0 {
                drop(g);
                commit_after_unlock(dt, numer, denom);
                g = lock_state();
            }
            if reached_now {
                break true;
            }
            break false;
        }
        if st.paused {
            mark_parked(st, deadline_us);
            g = CV.wait(g).unwrap_or_else(|p| p.into_inner());
            continue;
        }
        mark_parked(st, deadline_us);
        let dt = try_advance(st);
        let numer = st.pace_numer;
        let denom = st.pace_denom;
        if dt > 0 {
            drop(g);
            commit_after_unlock(dt, numer, denom);
            g = lock_state();
            continue;
        }
        g = CV.wait(g).unwrap_or_else(|p| p.into_inner());
    };
    if let Some(st) = g.as_mut() {
        disarm_deadline(st, deadline_us);
        let dt = try_advance(st);
        let numer = st.pace_numer;
        let denom = st.pace_denom;
        drop(g);
        commit_after_unlock(dt, numer, denom);
    }
    reached
}

/// Park until `virtual_us() + us`. `us == 0` returns immediately.
pub fn wait_us(us: u64) {
    if us == 0 {
        return;
    }
    let deadline = virtual_us().saturating_add(us);
    wait_until(deadline);
}

/// Park until `virtual_ms() + ms` milliseconds.
pub fn wait_ms(ms: u64) {
    wait_us(ms.saturating_mul(1000));
}

/// Account for guest work on this thread (one HAL call ≈ 1 µs of slice).
///
/// Native SIL executes host machine code, so the emulator is not in the
/// instruction stream and cannot stop a cog the way Renode/tlib can. The
/// HAL ABI is the only place it is on-stack. Platform trampolines call this
/// on entry; after one [`quantum_us`] of charged work the thread parks so
/// other participants can run.
///
/// No-op if this thread is not a [`participate`]r (unit tests, host
/// tooling). A real [`wait_us`] resets the slice.
pub fn charge(us: u64) {
    if us == 0 || PARTICIPANT.get().is_none() {
        return;
    }
    let q = quantum_us();
    let used = SLICE_USED_US.get().saturating_add(us);
    if used < q {
        SLICE_USED_US.set(used);
        return;
    }
    SLICE_USED_US.set(0);
    wait_us(used);
}

/// Wake [`wait_until_or_kicked`] callers without advancing time.
///
/// The board net-engine uses this when a command is queued so it can leave a
/// virtual-time park and drain the channel.
pub fn kick() {
    let mut g = lock_state();
    if let Some(st) = g.as_mut() {
        st.epoch = st.epoch.wrapping_add(1);
    }
    drop(g);
    CV.notify_all();
}

/// Test/debug: force the counter forward by `us` (clamped by pause).
///
/// Prefer [`wait_us`] from production code so participants stay in sync.
pub fn advance(us: u64) {
    if us == 0 {
        return;
    }
    let mut g = lock_state();
    let Some(st) = g.as_mut() else { return };
    if st.paused {
        return;
    }
    st.now_us = st.now_us.saturating_add(us);
    let numer = st.pace_numer;
    let denom = st.pace_denom;
    drop(g);
    commit_after_unlock(us, numer, denom);
}

// ============================================================
// Tests
// ============================================================

#[cfg(test)]
mod tests {
    use super::*;
    use rstest::rstest;
    use std::sync::Mutex as StdMutex;
    use std::thread;
    use std::time::Instant;

    static TEST_LOCK: StdMutex<()> = StdMutex::new(());

    fn lock_or_recover() -> std::sync::MutexGuard<'static, ()> {
        TEST_LOCK.lock().unwrap_or_else(|p| {
            TEST_LOCK.clear_poison();
            p.into_inner()
        })
    }

    fn fresh() {
        init(0.0, 1_000_000);
    }

    /// Unpaced `wait_us` jumps the counter by exactly that amount when no
    /// core is running.
    #[rstest]
    fn wait_us_jumps_now_when_idle() {
        let _g = lock_or_recover();
        fresh();
        assert_eq!(virtual_us(), 0);
        wait_us(1_500);
        assert_eq!(virtual_us(), 1_500);
        wait_ms(2);
        assert_eq!(virtual_us(), 3_500);
    }

    /// Quantum caps a single idle jump; `wait_until` still reaches the
    /// deadline by taking several slices.
    #[rstest]
    fn quantum_caps_idle_jump() {
        let _g = lock_or_recover();
        init_with_quantum(0.0, 1_000_000, 1_000);
        wait_until(2_500);
        assert_eq!(virtual_us(), 2_500);
        assert_eq!(quantum_us(), 1_000);
    }

    /// Two threads parking at different deadlines wake in deadline order and
    /// share one `now`. A running participant holds the barrier until both
    /// waiters have parked, so the first waiter cannot solo-jump past the
    /// second's deadline.
    #[rstest]
    fn two_waiters_synchronize_on_the_same_now() {
        let _g = lock_or_recover();
        fresh();
        let _hold = participate();
        let seen = std::sync::Arc::new(StdMutex::new(Vec::new()));
        let a = {
            let seen = std::sync::Arc::clone(&seen);
            thread::spawn(move || {
                wait_until(3_000);
                seen.lock().unwrap().push(('a', virtual_us()));
            })
        };
        let b = {
            let seen = std::sync::Arc::clone(&seen);
            thread::spawn(move || {
                wait_until(1_000);
                seen.lock().unwrap().push(('b', virtual_us()));
            })
        };
        thread::sleep(Duration::from_millis(20));
        drop(_hold);
        a.join().unwrap();
        b.join().unwrap();
        let got = seen.lock().unwrap().clone();
        assert_eq!(got.len(), 2, "{got:?}");
        let b_wake = got.iter().find(|(id, _)| *id == 'b').unwrap().1;
        let a_wake = got.iter().find(|(id, _)| *id == 'a').unwrap().1;
        // `wait_until(t)` guarantees `now >= t` on return; the clock may already
        // have taken the next quantum for remaining waiters.
        assert!(b_wake >= 1_000, "b woke at {b_wake}");
        assert!(a_wake >= 3_000, "a woke at {a_wake}");
        assert!(a_wake >= b_wake);
        assert_eq!(virtual_us(), a_wake.max(b_wake));
    }

    /// A running participant blocks idle jumps until it parks.
    #[rstest]
    fn running_participant_holds_the_barrier() {
        let _g = lock_or_recover();
        fresh();
        let (go_wait, wait_started) = {
            use std::sync::mpsc;
            let (tx_go, rx_go) = mpsc::channel::<()>();
            let (tx_started, rx_started) = mpsc::channel::<()>();
            let waiter = thread::spawn(move || {
                tx_started.send(()).unwrap();
                rx_go.recv().unwrap();
                wait_us(1_000);
            });
            rx_started.recv().unwrap();
            (tx_go, waiter)
        };
        let holder = thread::spawn(|| {
            let _p = participate();
            thread::sleep(Duration::from_millis(20));
            // still running — waiter must not have jumped time yet
            assert_eq!(virtual_us(), 0);
            drop(_p);
        });
        thread::sleep(Duration::from_millis(5));
        go_wait.send(()).unwrap();
        holder.join().unwrap();
        wait_started.join().unwrap();
        assert_eq!(virtual_us(), 1_000);
    }

    /// Pause stops [`wait_until`] from completing; resume lets it finish.
    #[rstest]
    fn pause_blocks_advance_until_resume() {
        let _g = lock_or_recover();
        fresh();
        pause();
        let waiter = thread::spawn(|| {
            wait_us(500);
            virtual_us()
        });
        thread::sleep(Duration::from_millis(20));
        assert_eq!(virtual_us(), 0);
        assert!(is_paused());
        resume();
        let woke_at = waiter.join().unwrap();
        assert_eq!(woke_at, 500);
        assert!(!is_paused());
    }

    /// [`kick`] makes [`wait_until_or_kicked`] return false before the
    /// deadline; time does not jump.
    #[rstest]
    fn kick_wakes_without_advancing() {
        let _g = lock_or_recover();
        fresh();
        let _hold = participate();
        let waiter = thread::spawn(|| wait_until_or_kicked(1_000_000));
        thread::sleep(Duration::from_millis(20));
        kick();
        let reached = waiter.join().unwrap();
        assert!(!reached, "kick must pre-empt the deadline");
        assert_eq!(virtual_us(), 0);
    }

    /// Unpaced `virtual_to_wall_us` is 0; paced 2x halves the wall duration.
    #[rstest]
    fn pacing_map_is_deterministic() {
        let _g = lock_or_recover();
        init(0.0, 1_000_000);
        assert_eq!(virtual_to_wall_us(1000), 0);
        set_scale(2.0);
        assert_eq!(virtual_to_wall_us(1000), 500);
        set_scale(1.0);
        assert_eq!(virtual_to_wall_us(1000), 1000);
        set_scale(0.0);
        assert_eq!(virtual_to_wall_us(1000), 0);
    }

    /// `advance` is a test override that moves `now` even with no waiters.
    #[rstest]
    fn advance_moves_now() {
        let _g = lock_or_recover();
        fresh();
        advance(42);
        assert_eq!(virtual_us(), 42);
        assert_eq!(virtual_ms(), 0);
        advance(1000);
        assert_eq!(virtual_ms(), 1);
    }

    /// Cycles track `now * freq / 1e6` exactly (1 MHz → 1 cycle per µs).
    #[rstest]
    fn cycles_follow_the_counter() {
        let _g = lock_or_recover();
        init(0.0, 1_000_000);
        assert_eq!(virtual_cycles(), 0);
        wait_us(10);
        assert_eq!(virtual_cycles(), 10);
        assert_eq!(clock_freq(), 1_000_000);
    }

    /// Re-init resets `now` to 0.
    #[rstest]
    fn reinit_reanchors() {
        let _g = lock_or_recover();
        fresh();
        wait_us(9_000);
        init(0.0, 180_000_000);
        assert_eq!(virtual_us(), 0);
        assert_eq!(clock_freq(), 180_000_000);
    }

    /// `wait_us(0)` does not hang.
    #[rstest]
    fn zero_wait_is_a_no_op() {
        let _g = lock_or_recover();
        fresh();
        wait_us(0);
        wait_ms(0);
        assert_eq!(virtual_us(), 0);
    }

    /// An unpaced 50 ms virtual wait must not burn 50 ms of wall time.
    #[rstest]
    fn unpaced_wait_is_faster_than_wall() {
        let _g = lock_or_recover();
        fresh();
        let t0 = Instant::now();
        wait_us(50_000);
        assert!(
            t0.elapsed() < Duration::from_millis(20),
            "unpaced 50ms virtual wait took {:?}",
            t0.elapsed()
        );
        assert_eq!(virtual_us(), 50_000);
    }

    /// Host threads that are not participants must not be preempted.
    #[rstest]
    fn charge_is_a_no_op_without_participate() {
        let _g = lock_or_recover();
        fresh();
        for _ in 0..10_000 {
            charge(1);
        }
        assert_eq!(virtual_us(), 0);
    }

    /// A running participant is left alone until its HAL-proxy slice fills.
    #[rstest]
    fn charge_below_quantum_does_not_advance() {
        let _g = lock_or_recover();
        fresh();
        let _p = participate();
        let q = quantum_us();
        for _ in 0..(q - 1) {
            charge(1);
        }
        assert_eq!(virtual_us(), 0);
    }

    /// Exhausting the slice parks and donates that virtual time — the
    /// emulator "stops" the cog at the next HAL entry.
    #[rstest]
    fn charge_exhausting_quantum_parks() {
        let _g = lock_or_recover();
        fresh();
        let _p = participate();
        let q = quantum_us();
        for _ in 0..q {
            charge(1);
        }
        assert_eq!(virtual_us(), q);
    }

    /// A real wait is a sync point: the slice starts over afterwards.
    #[rstest]
    fn wait_resets_the_charge_slice() {
        let _g = lock_or_recover();
        fresh();
        let _p = participate();
        let q = quantum_us();
        for _ in 0..(q - 1) {
            charge(1);
        }
        wait_us(10);
        assert_eq!(virtual_us(), 10);
        for _ in 0..(q - 1) {
            charge(1);
        }
        assert_eq!(virtual_us(), 10);
    }
}
