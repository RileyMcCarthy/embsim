//! Virtual Clock — provides scalable time for the emulator.
//!
//! Two modes, selected at [`init_mode`] ([`ClockMode`]):
//!
//! - **[`ClockMode::FreeRunning`]** (the default, and unchanged): virtual time
//!   is *scaled wall time*. At 1x virtual time == wall time; at 5x virtual time
//!   advances 5x faster (waits are 5x shorter); at 0.5x it advances half as
//!   fast (waits take twice as long).
//! - **[`ClockMode::Stepped`]**: virtual time is a value a *scheduler* sets
//!   with [`advance_to`]. Nothing samples wall time, so every timestamp is an
//!   integer the scheduler chose. This is `DETERMINISM.md` Phase D1.
//!
//! # Waiting
//!
//! Nothing outside this module may call [`std::thread::sleep`] to serve a
//! *simulated* wait. Every wait in the workspace goes through one of three
//! functions here, and one private `park_wall_us` is the only place that
//! actually sleeps:
//!
//! | call | free-running | stepped |
//! |---|---|---|
//! | [`wait_until`] | `sleep(scaled remaining)` | park until the scheduler advances to the deadline |
//! | [`wait_virtual_us`] | `sleep(scaled span)` | park until the scheduler advances to `now + span` |
//! | [`wait_wall_us`] | `sleep(span)` | `sleep(span)` — **and trips the wall-sleep tripwire** |
//!
//! The first two were the migration lever `DETERMINISM.md` Phase D0 installed;
//! D1 swapped their bodies without touching one of the 12 call sites.
//! [`wait_wall_us`] marks the waits that are wall-clock *by nature* — fd-poll
//! retry cadence, a startup warm-up. In stepped mode those are invisible to the
//! barrier below, so each one increments [`stepped_wall_sleep_count`] and logs
//! at error level: they are the grep-able list Phase D2 has to virtualize (see
//! `DETERMINISM.md`, "Wall-clock deadlines inside the simulation").
//!
//! # Stepped mode: actors, quiescence, and what it cannot determine
//!
//! In stepped mode nothing advances virtual time except a call to
//! [`advance_to`], and by convention exactly one component makes that call: the
//! **board engine**, which owns the only ordered event queue and the only timer
//! wheel (`DETERMINISM.md` T1 §3, "The engine is the time authority").
//!
//! A thread that can create simulation work registers with [`register_actor`].
//! Time may only advance when **every registered actor is parked** — the
//! quiescence barrier of `DETERMINISM.md` T1 §4 — which the scheduler waits for
//! with [`await_quiescence`]. `register_actor` binds the calling thread, so
//! [`wait_until`] knows the parking thread's actor identity without the caller
//! passing it around.
//!
//! **What the barrier can and cannot see** (stated at the API, per
//! `DETERMINISM.md` T1 §4, because it is a real limit and not an implementation
//! detail):
//!
//! - **Seen:** a registered actor parked at a virtual deadline. It is accounted
//!   for exactly, and [`advance_to`] releases it *and restores its runnable
//!   accounting* before it returns — so a scheduler can never step past an
//!   actor's wake instant while the actor is still catching up.
//! - **Seen:** an unregistered thread parked at a virtual deadline. Its
//!   deadline joins [`SchedulerState::next_deadline_us`] (so it is always
//!   released eventually), but it does **not** hold time back — it cannot,
//!   since the scheduler has no way to know when such a thread is between
//!   waits.
//! - **Not seen, and not determinable in D1:** a thread blocked on a real file
//!   descriptor, an OS mutex, or a channel. Such a thread is neither
//!   running-with-work nor parked-at-a-deadline, and no barrier here can
//!   classify it. That is why `DETERMINISM.md` scopes D1 to systems whose I/O
//!   is engine-side and defers the byte transports to D2.
//! - **Not seen:** whether an actor *will* register. New actors start runnable,
//!   so registering one while the scheduler is mid-advance is a race the
//!   scheduler cannot arbitrate. Register before the system starts.
//!
//! # Native firmware: preemption at HAL
//!
//! Host-compiled firmware is real machine code. The emulator is not in the
//! instruction stream, so a cog that never waits (`LOCKTRY` spin, UART poll)
//! would stay runnable forever and freeze stepped time. Platform trampolines
//! call [`charge`] on every HAL entry; after [`DEFAULT_QUANTUM_US`] of charged
//! work the cog parks through [`wait_virtual_us`]. Firmware stays free to
//! spin. A cog that never hits HAL still cannot be stopped — that needs an ISS.

use std::cell::Cell;
use std::collections::BTreeMap;
use std::marker::PhantomData;
use std::sync::atomic::{AtomicU32, AtomicU64, AtomicU8, Ordering};
use std::sync::{Condvar, Mutex, MutexGuard};
use std::time::{Duration, Instant};

/// Global virtual clock state.
static SCALE_NUMER: AtomicU64 = AtomicU64::new(1);
static SCALE_DENOM: AtomicU64 = AtomicU64::new(1);

/// Immovable process time origin (set once, the first time `init` runs).
static PROCESS_ORIGIN: std::sync::OnceLock<Instant> = std::sync::OnceLock::new();

/// Real microseconds (from `PROCESS_ORIGIN`) at which the virtual clock was
/// last (re)anchored. Re-anchored on every `init`, so re-initializing the
/// emulator in-process restarts virtual time at 0 without the lock-free hot
/// path ever taking a mutex.
static BOOT_OFFSET_US: AtomicU64 = AtomicU64::new(0);

/// Simulated clock frequency in Hz, supplied per-MCU by `init`.
///
/// Defaults to `0` (uninitialized) rather than any specific part's frequency,
/// so a project that forgets to call `init` gets an obviously-wrong `0` from
/// cycle math instead of silently inheriting another MCU's clock. Platform
/// crates (e.g. `embsim-p2`) own their real frequency.
static CLOCK_FREQ: AtomicU32 = AtomicU32::new(0);

/// Current [`ClockMode`] discriminant: `MODE_FREE_RUNNING` or `MODE_STEPPED`.
/// Free-running is the default so a consumer that never calls [`init_mode`]
/// behaves exactly as it did before Phase D1.
static MODE: AtomicU8 = AtomicU8::new(MODE_FREE_RUNNING);
const MODE_FREE_RUNNING: u8 = 0;
const MODE_STEPPED: u8 = 1;

/// Stepped mode's "now", in virtual µs. Mirrors `Sched::now_us` so
/// [`virtual_us`] stays one relaxed load with no mutex on the hot path.
static NOW_US: AtomicU64 = AtomicU64::new(0);

/// How many real sleeps were served while the clock was stepped — the
/// wall-sleep tripwire (`DETERMINISM.md`, "Proving it" → Tests). Read with
/// [`stepped_wall_sleep_count`].
static STEPPED_WALL_SLEEPS: AtomicU64 = AtomicU64::new(0);

/// How virtual time advances (`DETERMINISM.md` T1 §1).
///
/// A **runtime enum, not a Cargo feature**, per T1 §2: the clock is already a
/// process-global configured by one call, features are additive and would let
/// `--all-features` or two dependents silently pick a winner, and consumers
/// need both modes in one binary (`--deterministic` scenario regressions vs. an
/// interactive playground with a human at the PTY).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ClockMode {
    /// Virtual time is scaled wall time — the historical behavior, and the
    /// default.
    FreeRunning {
        /// Time scale (1.0 = real time, 5.0 = five times faster).
        speed: f64,
    },
    /// Virtual time is a value the scheduler sets with [`advance_to`]; it
    /// advances only when every registered actor is parked.
    Stepped,
}

/// Initialize the virtual clock with the given speed scale and clock frequency,
/// in **free-running** mode. Must be called before any time functions. Calling
/// it again re-anchors virtual time to 0 (an in-process restart) and updates the
/// scale/frequency.
///
/// Exactly `init_mode(ClockMode::FreeRunning { speed }, freq)`.
pub fn init(speed: f64, freq: u32) {
    init_mode(ClockMode::FreeRunning { speed }, freq);
}

/// Initialize the virtual clock in an explicit [`ClockMode`], re-anchoring
/// virtual time to 0 and setting the simulated clock frequency.
///
/// `DETERMINISM.md` T1 §1 specifies the mode as immutable after `init_mode`.
/// That is enforced *for the lifetime of a run* rather than for the lifetime of
/// the process: re-`init` has always meant "in-process restart", and a mode
/// change is only sound at exactly that point. So this call may change the mode
/// — but entering [`ClockMode::Stepped`] with actors still registered is a
/// **panic**, because a leaked actor thread from a previous run is precisely
/// what would make the next run's barrier lie. See "Deviations from the design
/// doc" in `DETERMINISM.md`.
///
/// # Panics
/// Panics when entering stepped mode while any [`Actor`] is still registered,
/// naming them.
pub fn init_mode(mode: ClockMode, freq: u32) {
    if mode == ClockMode::Stepped {
        let leaked = registered_actor_names();
        assert!(
            leaked.is_empty(),
            "cannot enter stepped clock mode with {} actor(s) still registered: {leaked:?} — \
             a leaked actor thread from a previous run makes the quiescence barrier lie \
             (DETERMINISM.md T1 §4)",
            leaked.len()
        );
    }
    let origin = PROCESS_ORIGIN.get_or_init(Instant::now);
    BOOT_OFFSET_US.store(origin.elapsed().as_micros() as u64, Ordering::Relaxed);
    CLOCK_FREQ.store(freq, Ordering::Relaxed);
    {
        // Re-anchor stepped "now" to 0 under the scheduler lock so a concurrent
        // `advance_to` cannot interleave with the reset. Bumping the epoch
        // releases anything parked against the OLD timeline: its deadline is
        // now unreachable (time just went back to 0), so waiting for it would
        // be waiting forever.
        let mut sched = lock_sched();
        sched.now_us = 0;
        sched.waits.clear();
        sched.epoch += 1;
        let mut released = 0usize;
        for state in sched.actors.values_mut() {
            if state.parked {
                state.parked = false;
                released += 1;
            }
        }
        sched.running += released;
        NOW_US.store(0, Ordering::Relaxed);
    }
    ADVANCED.notify_all();
    match mode {
        ClockMode::FreeRunning { speed } => {
            MODE.store(MODE_FREE_RUNNING, Ordering::Relaxed);
            set_scale(speed);
        }
        ClockMode::Stepped => {
            // A stepped clock has no scale; pin the ratio at 1:1 so any stray
            // `virtual_to_wall_us` reader gets an identity mapping rather than
            // whatever the previous run left behind.
            SCALE_NUMER.store(1000, Ordering::Relaxed);
            SCALE_DENOM.store(1000, Ordering::Relaxed);
            MODE.store(MODE_STEPPED, Ordering::Relaxed);
        }
    }
}

/// The current clock mode.
pub fn mode() -> ClockMode {
    if is_stepped() {
        ClockMode::Stepped
    } else {
        let numer = SCALE_NUMER.load(Ordering::Relaxed) as f64;
        let denom = SCALE_DENOM.load(Ordering::Relaxed).max(1) as f64;
        ClockMode::FreeRunning {
            speed: numer / denom,
        }
    }
}

/// True when the clock is stepped — one relaxed load, cheap enough for the
/// engine loop and every [`virtual_us`] call.
pub fn is_stepped() -> bool {
    MODE.load(Ordering::Relaxed) == MODE_STEPPED
}

/// Change the time scale at runtime.
/// Uses integer numerator/denominator to avoid floating point in the hot path.
///
/// **Loud no-op in stepped mode** (`DETERMINISM.md` T1 §1): scaling a clock the
/// scheduler sets by hand is meaningless, and silently accepting it would make
/// the caller believe time behaves differently than it does.
pub fn set_scale(scale: f64) {
    if is_stepped() {
        tracing::warn!(
            scale,
            "set_scale ignored: the clock is stepped, so virtual time is set by \
             advance_to and has no wall-clock ratio to scale"
        );
        return;
    }
    let precision = 1000u64;
    let numer = (scale * precision as f64) as u64;
    let denom = precision;
    SCALE_NUMER.store(numer, Ordering::Relaxed);
    SCALE_DENOM.store(denom, Ordering::Relaxed);
}

/// True once `init` has run in this process. Time functions such as
/// [`virtual_us`] panic before `init`; callers that must stay alive (e.g. a
/// long-running engine thread validating a schedule request) check this
/// first and fail the request loudly instead.
pub fn is_initialized() -> bool {
    PROCESS_ORIGIN.get().is_some()
}

/// Get virtual microseconds elapsed since the last `init`.
///
/// In [`ClockMode::Stepped`] this is one relaxed load of the value the
/// scheduler last set with [`advance_to`] — no wall clock is sampled, which is
/// the whole point.
///
/// # Panics
/// In free-running mode, panics if [`init`] has not run (there is no origin to
/// measure from). Stepped mode has no origin to need.
pub fn virtual_us() -> u64 {
    if is_stepped() {
        return NOW_US.load(Ordering::Relaxed);
    }
    let origin = PROCESS_ORIGIN.get().expect("Virtual clock not initialized");
    let wall_us = (origin.elapsed().as_micros() as u64)
        .saturating_sub(BOOT_OFFSET_US.load(Ordering::Relaxed));
    let numer = SCALE_NUMER.load(Ordering::Relaxed);
    let denom = SCALE_DENOM.load(Ordering::Relaxed);
    wall_us * numer / denom
}

/// Get virtual milliseconds elapsed since boot.
pub fn virtual_ms() -> u64 {
    virtual_us() / 1000
}

/// Convert a virtual wait duration to a wall-clock sleep duration.
/// If speed is 5x, a 1000us virtual wait becomes a 200us real sleep.
pub fn virtual_to_wall_us(virtual_wait_us: u64) -> u64 {
    let numer = SCALE_NUMER.load(Ordering::Relaxed);
    let denom = SCALE_DENOM.load(Ordering::Relaxed);
    virtual_wait_us * denom / numer.max(1)
}

/// Get the simulated clock frequency.
pub fn clock_freq() -> u32 {
    CLOCK_FREQ.load(Ordering::Relaxed)
}

/// HAL-proxy work that exhausts a cog's slice, after which [`charge`] parks.
/// 1 ms, a common Renode quantum.
pub const DEFAULT_QUANTUM_US: u64 = 1_000;

// ============================================================
// Waiting — the single chokepoint between virtual and wall time
// ============================================================

/// Park the caller for `wall_us` of **real** time. The only `thread::sleep`
/// serving a simulated wait in the workspace; a zero wait never sleeps.
///
/// In stepped mode a real sleep is a defect by construction — the scheduler
/// cannot see it — so each one trips the wall-sleep tripwire
/// ([`stepped_wall_sleep_count`]) and logs at error level.
fn park_wall_us(wall_us: u64) {
    if wall_us == 0 {
        return;
    }
    if is_stepped() {
        STEPPED_WALL_SLEEPS.fetch_add(1, Ordering::Relaxed);
        tracing::error!(
            wall_us,
            "wall sleep while the clock is STEPPED: this wait is invisible to the \
             quiescence barrier, so the scheduler may step past it \
             (DETERMINISM.md T1 §4, 'Wall-clock deadlines inside the simulation')"
        );
    }
    std::thread::sleep(Duration::from_micros(wall_us));
}

/// How many real sleeps have been served while the clock was stepped — the
/// wall-sleep tripwire `DETERMINISM.md` asks for under "Proving it". Non-zero
/// means some wait escaped virtualization; the error-level log names the span.
pub fn stepped_wall_sleep_count() -> u64 {
    STEPPED_WALL_SLEEPS.load(Ordering::Relaxed)
}

/// Park the caller for `d_us` of **virtual** time (a relative wait).
///
/// Free-running: the scaled wall equivalent, i.e. exactly
/// `sleep(virtual_to_wall_us(d_us))`. Needs no clock origin, so it is safe
/// before [`init`] — a wait span is pure scale arithmetic, unlike
/// [`wait_until`], which must read the current time.
///
/// Stepped: exactly `wait_until(virtual_us() + d_us)`.
///
/// This is the sibling of [`wait_until`] for the many call sites that know how
/// long to wait but not *when* they started (`HAL_time_waitUs`, a poll
/// cadence, a receive timeout).
pub fn wait_virtual_us(d_us: u64) {
    SLICE_USED_US.set(0);
    if is_stepped() {
        park_until_virtual(NOW_US.load(Ordering::Relaxed).saturating_add(d_us));
        return;
    }
    park_wall_us(virtual_to_wall_us(d_us));
}

/// Park the caller until virtual time reaches the absolute deadline
/// `deadline_v_us`. Returns immediately when the deadline has already passed.
///
/// This is the form `DETERMINISM.md` Phase D1 swapped out: in stepped mode it
/// registers a pending deadline, marks the caller parked (when the caller is a
/// registered [`Actor`]), and blocks until the scheduler advances virtual time
/// to it. Prefer it wherever the call site genuinely holds a deadline (a
/// reserved wire slot, a scheduled edge) rather than a duration — an absolute
/// deadline cannot drift, and it is the only form a discrete-event scheduler
/// can serve.
///
/// **Stepped-mode liveness:** the park is released only by [`advance_to`]. A
/// caller that parks in a process where nothing advances time never wakes. The
/// deadline is published in [`SchedulerState::next_deadline_us`] precisely so a
/// scheduler can guarantee it does.
///
/// # Panics
/// In free-running mode, panics if [`init`] has not run — reading "now"
/// requires the clock origin. Callers that must survive an uninitialized clock
/// check [`is_initialized`] first, or use [`wait_virtual_us`].
pub fn wait_until(deadline_v_us: u64) {
    SLICE_USED_US.set(0);
    if is_stepped() {
        park_until_virtual(deadline_v_us);
        return;
    }
    let now = virtual_us();
    wait_virtual_us(deadline_v_us.saturating_sub(now));
}

/// Park the caller for `d_us` of **real** time, bypassing the virtual clock.
///
/// Reserved for waits that are wall-clock by nature and must stay that way in
/// free-running mode: the retry cadence of a spin on a real file descriptor,
/// and a fixed startup warm-up. These are the sites `DETERMINISM.md` (T1 §4,
/// "Wall-clock deadlines inside the simulation") says must *become* virtual.
/// Phase D1 deliberately left them wall-clock — they belong to the byte
/// transports D2 replaces — and made them **loud instead**: in stepped mode
/// each call trips [`stepped_wall_sleep_count`] and logs at error level, so
/// they surface as a measurement rather than as a mysterious hang. Do not reach
/// for this for anything the simulation's timing depends on; use [`wait_until`]
/// or [`wait_virtual_us`].
pub fn wait_wall_us(d_us: u64) {
    park_wall_us(d_us);
}

/// Account for guest work on this thread (one HAL call ≈ 1 µs of slice).
///
/// Native SIL executes host machine code, so the emulator is not in the
/// instruction stream. Platform trampolines call this on HAL entry; after
/// [`DEFAULT_QUANTUM_US`] of charged work the thread parks via
/// [`wait_virtual_us`] so other actors and the engine can run.
///
/// No-op if this thread is not a registered [`Actor`] (unit tests, host
/// tooling). A real [`wait_virtual_us`] / [`wait_until`] resets the slice.
pub fn charge(us: u64) {
    if us == 0 || THREAD_ACTOR.with(|slot| slot.get()).is_none() {
        return;
    }
    let used = SLICE_USED_US.get().saturating_add(us);
    if used < DEFAULT_QUANTUM_US {
        SLICE_USED_US.set(used);
        return;
    }
    SLICE_USED_US.set(0);
    wait_virtual_us(used);
}

// ============================================================
// Stepped mode: actor registry + the quiescence barrier
// ============================================================

/// One registered actor's accounting.
#[derive(Debug)]
struct ActorState {
    name: String,
    parked: bool,
}

/// Everything the stepped scheduler owns. One mutex; the hot read path
/// ([`virtual_us`]) never takes it.
struct Sched {
    /// Authoritative stepped "now" (µs). [`NOW_US`] mirrors it.
    now_us: u64,
    /// Registered actors by dense id — a `BTreeMap`, so every diagnostic list
    /// this produces is in registration order rather than hash order (the
    /// engine review rule in `DETERMINISM.md` applies to the clock too).
    actors: BTreeMap<u64, ActorState>,
    /// Registered actors that are **not** parked. Time may advance only at 0.
    running: usize,
    /// Pending park deadlines: `(deadline_us, token) -> actor id (if any)`.
    /// The token disambiguates identical deadlines so two parks never collide.
    waits: BTreeMap<(u64, u64), Option<u64>>,
    next_token: u64,
    next_actor_id: u64,
    /// Bumped by every [`init_mode`] (an in-process restart). A park from a
    /// previous epoch releases immediately rather than waiting for a "now" that
    /// was just reset behind it.
    epoch: u64,
}

static SCHED: Mutex<Sched> = Mutex::new(Sched {
    now_us: 0,
    actors: BTreeMap::new(),
    running: 0,
    waits: BTreeMap::new(),
    next_token: 0,
    next_actor_id: 0,
    epoch: 0,
});

/// Notified by [`advance_to`] when "now" moves.
static ADVANCED: Condvar = Condvar::new();

/// Notified when the runnable-actor count reaches zero.
static QUIESCENT: Condvar = Condvar::new();

thread_local! {
    /// Actor id bound to this thread by [`register_actor`], if any.
    static THREAD_ACTOR: Cell<Option<u64>> = const { Cell::new(None) };
    /// HAL-proxy work charged against this thread's quantum. Reset by
    /// [`wait_until`] / [`wait_virtual_us`] / actor drop.
    static SLICE_USED_US: Cell<u64> = const { Cell::new(0) };
}

/// Take the scheduler lock, recovering from poison. A thread that panicked
/// while holding it left the clock readable but not corrupt (every mutation
/// here is a small, self-contained update), and poisoning the whole clock would
/// turn one component's panic into a process-wide hang.
fn lock_sched() -> MutexGuard<'static, Sched> {
    SCHED.lock().unwrap_or_else(|poisoned| {
        SCHED.clear_poison();
        poisoned.into_inner()
    })
}

/// Names of every registered actor, in registration order.
fn registered_actor_names() -> Vec<String> {
    lock_sched()
        .actors
        .values()
        .map(|state| state.name.clone())
        .collect()
}

/// Registration handle for a thread that can create simulation work.
///
/// Held for as long as the thread may do work; dropping it unregisters (so a
/// thread that exits — or unwinds — stops holding the barrier). Binds the
/// calling thread, so [`wait_until`] finds the identity without plumbing.
///
/// Deliberately `!Send`, exactly like `embsim_peripherals::instance`'s binding
/// guard: dropping it on another thread would clear the wrong thread's binding
/// and leave this one accounted as permanently runnable.
#[derive(Debug)]
pub struct Actor {
    id: u64,
    name: String,
    _not_send: PhantomData<*const ()>,
}

impl Actor {
    /// This actor's registered name (diagnostics).
    pub fn name(&self) -> &str {
        &self.name
    }
}

impl Drop for Actor {
    fn drop(&mut self) {
        THREAD_ACTOR.with(|slot| slot.set(None));
        SLICE_USED_US.set(0);
        let quiescent = {
            let mut sched = lock_sched();
            if let Some(state) = sched.actors.remove(&self.id) {
                if !state.parked {
                    sched.running -= 1;
                }
            }
            // Any park this actor still owns is now anonymous, not leaked: its
            // deadline stays visible to the scheduler so nothing waits forever.
            for slot in sched.waits.values_mut() {
                if *slot == Some(self.id) {
                    *slot = None;
                }
            }
            sched.running == 0
        };
        if quiescent {
            QUIESCENT.notify_all();
        }
    }
}

/// Register the calling thread as an actor: a thread that can create
/// simulation work, and that the stepped scheduler must therefore wait for
/// before advancing time (`DETERMINISM.md` T1 §4).
///
/// Registration is free in free-running mode — nothing consults the registry
/// there — so a model may register unconditionally and behave identically in
/// both modes.
///
/// # Panics
/// Panics if this thread is already registered. One thread is one actor; a
/// nested registration means two owners disagree about who parks it.
pub fn register_actor(name: &str) -> Actor {
    // Reject a double registration BEFORE touching the registry: a rejected
    // call must leave no trace, or the panic itself leaks the actor it refused
    // to create and every later `init_mode(Stepped)` fails.
    assert!(
        THREAD_ACTOR.with(|slot| slot.get().is_none()),
        "thread is already registered as a virtual-clock actor; \
         one thread is one actor (attempted to register {name:?})"
    );
    let id = {
        let mut sched = lock_sched();
        let id = sched.next_actor_id;
        sched.next_actor_id += 1;
        sched.actors.insert(
            id,
            ActorState {
                name: name.to_string(),
                parked: false,
            },
        );
        sched.running += 1;
        id
    };
    THREAD_ACTOR.with(|slot| slot.set(Some(id)));
    Actor {
        id,
        name: name.to_string(),
        _not_send: PhantomData,
    }
}

/// Number of actors currently registered (diagnostics and tests).
pub fn registered_actors() -> usize {
    lock_sched().actors.len()
}

/// Park the calling thread until stepped "now" reaches `deadline_v_us`.
fn park_until_virtual(deadline_v_us: u64) {
    let actor = THREAD_ACTOR.with(|slot| slot.get());
    let mut sched = lock_sched();
    if sched.now_us >= deadline_v_us {
        return; // already due: never park, never yield
    }
    let token = sched.next_token;
    sched.next_token += 1;
    let epoch = sched.epoch;
    sched.waits.insert((deadline_v_us, token), actor);
    let mut quiescent = false;
    if let Some(id) = actor {
        if let Some(state) = sched.actors.get_mut(&id) {
            if !state.parked {
                state.parked = true;
                sched.running -= 1;
                quiescent = sched.running == 0;
            }
        }
    }
    if quiescent {
        QUIESCENT.notify_all();
    }
    while sched.now_us < deadline_v_us && sched.epoch == epoch {
        sched = ADVANCED
            .wait(sched)
            .unwrap_or_else(|poisoned| poisoned.into_inner());
    }
    // On the normal path `advance_to` already removed the wait entry and
    // restored this actor's runnable accounting before releasing the lock — see
    // its docs for why that ordering is load-bearing. An epoch bump
    // (`init_mode`, an in-process restart) does the same wholesale, so there is
    // nothing left to clean up either way.
}

/// Failure of [`advance_to`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AdvanceError {
    /// The clock is free-running: nothing may set "now" by hand.
    ///
    /// (`DETERMINISM.md` T1 §1 sketches this as a single `TimeWentBackwards`
    /// error; a one-variant error cannot express "you called this in the wrong
    /// mode", which the implementation has to reject — see "Deviations from the
    /// design doc".)
    NotStepped,
    /// Virtual time may only move forward.
    WentBackwards {
        /// Stepped "now" at the time of the call.
        now_us: u64,
        /// The requested (earlier) time.
        requested_us: u64,
    },
}

impl std::fmt::Display for AdvanceError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            AdvanceError::NotStepped => {
                write!(f, "advance_to requires ClockMode::Stepped")
            }
            AdvanceError::WentBackwards {
                now_us,
                requested_us,
            } => write!(
                f,
                "advance_to({requested_us}) would move virtual time backwards from {now_us}"
            ),
        }
    }
}

impl std::error::Error for AdvanceError {}

/// **Scheduler only.** Advance stepped virtual time to `v_us` and release every
/// wait whose deadline has now passed. Monotonic; going backwards is rejected.
///
/// Releasing is done *here*, under the scheduler lock: every released actor is
/// marked runnable again **before this returns**. That ordering is what makes
/// the barrier sound — otherwise a scheduler could observe `running == 0` in
/// the window between "notified" and "actually rescheduled by the OS" and step
/// straight past the instant it just woke someone for.
pub fn advance_to(v_us: u64) -> Result<(), AdvanceError> {
    if !is_stepped() {
        return Err(AdvanceError::NotStepped);
    }
    let mut sched = lock_sched();
    if v_us < sched.now_us {
        return Err(AdvanceError::WentBackwards {
            now_us: sched.now_us,
            requested_us: v_us,
        });
    }
    sched.now_us = v_us;
    NOW_US.store(v_us, Ordering::Relaxed);

    // Every wait now due, in `(deadline, token)` order. The map is keyed by
    // that pair, so the range is exactly the due set — no scan of the pending
    // tail, and no reliance on `v_us + 1` (which would misbehave at `u64::MAX`).
    let due: Vec<(u64, u64)> = sched
        .waits
        .range(..=(v_us, u64::MAX))
        .map(|(&key, _)| key)
        .collect();
    for key in due {
        let Some(Some(owner)) = sched.waits.remove(&key) else {
            continue; // an unregistered waiter: released, never accounted
        };
        if let Some(state) = sched.actors.get_mut(&owner) {
            if state.parked {
                state.parked = false;
                sched.running += 1;
            }
        }
    }
    drop(sched);
    ADVANCED.notify_all();
    Ok(())
}

/// What the stepped scheduler needs to decide its next move.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SchedulerState {
    /// Stepped "now" (µs).
    pub now_us: u64,
    /// Registered actors that are not parked. Time may advance only at 0.
    pub running: usize,
    /// Earliest pending park deadline across **all** waiters, registered or
    /// not — the scheduler must not advance past it, and must eventually
    /// advance *to* it or the waiter never wakes.
    pub next_deadline_us: Option<u64>,
}

/// Snapshot of the stepped scheduler's state.
pub fn scheduler_state() -> SchedulerState {
    let sched = lock_sched();
    SchedulerState {
        now_us: sched.now_us,
        running: sched.running,
        next_deadline_us: sched.waits.keys().next().map(|&(deadline, _)| deadline),
    }
}

/// Outcome of [`await_quiescence`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Quiescence {
    /// Every registered actor is parked; time may advance.
    Reached {
        /// Earliest pending park deadline, if any.
        next_deadline_us: Option<u64>,
    },
    /// Some actor stayed runnable for the whole timeout. Virtual time cannot
    /// advance without breaking the barrier, so the scheduler must report this
    /// rather than stall forever — and the run's determinism guarantee is void.
    Stalled {
        /// Names of the actors still runnable, in registration order.
        actors: Vec<String>,
    },
}

/// **Scheduler only.** Block until every registered actor is parked, or until
/// `timeout` of **wall** time elapses.
///
/// The timeout is wall-clock on purpose: it is not part of the simulation, it
/// is the escape hatch for a wedged actor. Reaching it is a defect report
/// ([`Quiescence::Stalled`]), and a run that hits it is not reproducible.
pub fn await_quiescence(timeout: Duration) -> Quiescence {
    let deadline = Instant::now() + timeout;
    let mut sched = lock_sched();
    while sched.running > 0 {
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            return Quiescence::Stalled {
                actors: running_actor_names(&sched),
            };
        }
        let (guard, result) = QUIESCENT
            .wait_timeout(sched, remaining)
            .unwrap_or_else(|poisoned| poisoned.into_inner());
        sched = guard;
        if result.timed_out() && sched.running > 0 {
            return Quiescence::Stalled {
                actors: running_actor_names(&sched),
            };
        }
    }
    Quiescence::Reached {
        next_deadline_us: sched.waits.keys().next().map(|&(deadline, _)| deadline),
    }
}

/// Names of the actors currently runnable, in registration order.
fn running_actor_names(sched: &Sched) -> Vec<String> {
    sched
        .actors
        .values()
        .filter(|state| !state.parked)
        .map(|state| state.name.clone())
        .collect()
}

/// Get virtual cycle count (`virtual_us * clock_freq / 1_000_000`).
///
/// Computed in `u128` and divided last so frequencies that are not a whole
/// number of MHz keep full precision (the old `freq / 1_000_000` pre-divide
/// silently truncated sub-MHz and fractional-MHz parts).
pub fn virtual_cycles() -> u64 {
    let us = virtual_us() as u128;
    let freq = CLOCK_FREQ.load(Ordering::Relaxed) as u128;
    (us * freq / 1_000_000) as u64
}

// ============================================================
// Tests
// ============================================================

#[cfg(test)]
mod tests {
    use rstest::rstest;

    use super::*;
    use std::sync::Arc;
    use std::sync::Mutex as StdMutex;

    /// The virtual clock mutates process-global mode / scale / frequency /
    /// boot-offset / scheduler state, so every test that touches it must run
    /// serially. Recover from any panic-induced poisoning exactly like the
    /// `pulse_out` reference suite.
    static TEST_LOCK: StdMutex<()> = StdMutex::new(());

    fn lock_or_recover() -> std::sync::MutexGuard<'static, ()> {
        TEST_LOCK.lock().unwrap_or_else(|p| {
            TEST_LOCK.clear_poison();
            p.into_inner()
        })
    }

    /// `set_scale` then `virtual_to_wall_us` is a deterministic pure mapping:
    /// at 1.0× a virtual wait equals the wall wait; at 2.0× it halves; at 0.5×
    /// it doubles. No real time elapses, so the exact arithmetic is asserted.
    #[rstest]
    #[case::scale_1x(1.0, 1000, 1000)]
    #[case::scale_2x(2.0, 1000, 500)]
    #[case::scale_half(0.5, 1000, 2000)]
    #[case::scale_5x(5.0, 1000, 200)]
    fn virtual_to_wall_is_deterministic_pure_mapping(
        #[case] scale: f64,
        #[case] virtual_us: u64,
        #[case] expected_wall: u64,
    ) {
        let _g = lock_or_recover();
        set_scale(scale);
        assert_eq!(
            virtual_to_wall_us(virtual_us),
            expected_wall,
            "{scale}x: wall for {virtual_us} virtual us"
        );
        set_scale(1.0);
    }

    /// A scale of 0.0 truncates the numerator to 0; `virtual_to_wall_us` must
    /// clamp the divisor via `numer.max(1)` so it never divides by zero. The
    /// result is therefore `wait * denom` (denom == 1000 internally).
    #[rstest]
    fn scale_zero_is_clamped_no_divide_by_zero() {
        let _g = lock_or_recover();
        set_scale(0.0);
        // numer == 0 → clamped to 1 → wait * denom(1000) / 1.
        assert_eq!(virtual_to_wall_us(1), 1000);
        assert_eq!(virtual_to_wall_us(7), 7000);
        // Restore a sane scale for any sibling that races on re-init.
        set_scale(1.0);
    }

    /// `virtual_to_wall_us(0)` is always 0 regardless of scale (no wait → no
    /// sleep), at normal, fast, slow, and clamped-zero scales.
    #[rstest]
    fn zero_wait_maps_to_zero_at_every_scale() {
        let _g = lock_or_recover();
        for s in [1.0, 2.0, 0.5, 0.0, 10.0] {
            set_scale(s);
            assert_eq!(virtual_to_wall_us(0), 0, "zero wait at scale {s}");
        }
        set_scale(1.0);
    }

    /// `init` stores the supplied clock frequency and `clock_freq` returns it.
    #[rstest]
    fn init_sets_clock_freq() {
        let _g = lock_or_recover();
        init(1.0, 180_000_000);
        assert_eq!(clock_freq(), 180_000_000);
        init(1.0, 320_000_000);
        assert_eq!(clock_freq(), 320_000_000);
    }

    /// `init` also applies the speed scale it is given (the scale feeds straight
    /// into the deterministic `virtual_to_wall_us` mapping).
    #[rstest]
    fn init_applies_speed_scale() {
        let _g = lock_or_recover();
        init(2.0, 1_000_000);
        assert_eq!(virtual_to_wall_us(1000), 500, "init(2.0) halves wall wait");
        init(0.5, 1_000_000);
        assert_eq!(
            virtual_to_wall_us(1000),
            2000,
            "init(0.5) doubles wall wait"
        );
        init(1.0, 1_000_000);
    }

    /// `virtual_us` is monotonic non-decreasing across repeated reads — virtual
    /// time never runs backwards (timing magnitude is machine-dependent and not
    /// asserted, only ordering).
    #[rstest]
    fn virtual_us_is_monotonic() {
        let _g = lock_or_recover();
        init(1.0, 180_000_000);
        let mut last = virtual_us();
        for _ in 0..1000 {
            let now = virtual_us();
            assert!(now >= last, "virtual_us went backwards: {now} < {last}");
            last = now;
        }
    }

    /// `virtual_ms` is monotonic and is exactly `virtual_us / 1000` ordering —
    /// ms can never exceed the µs reading divided by 1000.
    #[rstest]
    fn virtual_ms_tracks_virtual_us() {
        let _g = lock_or_recover();
        init(1.0, 180_000_000);
        let mut last_ms = virtual_ms();
        for _ in 0..1000 {
            let us = virtual_us();
            let ms = virtual_ms();
            assert!(ms >= last_ms, "virtual_ms went backwards");
            // ms reading taken after us can only have advanced, so ms*1000 may
            // exceed the earlier us; but ms must never exceed us/1000 + slack.
            assert!(ms <= virtual_us() / 1000, "ms must not lead the us clock");
            last_ms = ms;
            let _ = us;
        }
    }

    /// `virtual_cycles` is 0 whenever the configured frequency is 0 (the
    /// uninitialized-frequency default), no matter how much virtual time has
    /// elapsed.
    #[rstest]
    fn virtual_cycles_zero_when_freq_zero() {
        let _g = lock_or_recover();
        // Re-anchor with an explicit zero frequency.
        init(1.0, 0);
        assert_eq!(clock_freq(), 0);
        assert_eq!(virtual_cycles(), 0, "no freq → no cycles");
        // Even after advancing virtual time it stays zero.
        for _ in 0..500 {
            let _ = virtual_us();
        }
        assert_eq!(virtual_cycles(), 0);
    }

    /// With a non-zero frequency and advancing virtual time, `virtual_cycles`
    /// is monotonic non-decreasing and eventually grows above zero. The exact
    /// cycle count is wall-clock dependent and is deliberately NOT asserted.
    #[rstest]
    fn virtual_cycles_grow_with_time_when_freq_nonzero() {
        let _g = lock_or_recover();
        init(1.0, 180_000_000);
        let mut last = virtual_cycles();
        let mut grew = false;
        for _ in 0..200_000 {
            let now = virtual_cycles();
            assert!(now >= last, "cycles went backwards");
            if now > 0 {
                grew = true;
            }
            last = now;
            if grew {
                break;
            }
        }
        assert!(
            grew,
            "cycles should rise above zero as virtual time advances"
        );
    }

    /// [`wait_virtual_us`] is exactly the old inline
    /// `sleep(virtual_to_wall_us(d))` shape it replaced: a zero span never
    /// sleeps, and a non-zero span parks for at least the scaled wall
    /// equivalent. Only the lower bound is asserted — a host may always
    /// oversleep (`TESTING.md` rule 4).
    #[rstest]
    #[case::zero_at_1x(1.0, 0)]
    #[case::zero_at_10x(10.0, 0)]
    #[case::span_at_1x(1.0, 2_000)]
    #[case::span_at_10x(10.0, 20_000)]
    fn wait_virtual_us_parks_for_the_scaled_wall_equivalent(
        #[case] scale: f64,
        #[case] span_v_us: u64,
    ) {
        let _g = lock_or_recover();
        init(scale, 1_000_000);
        let expected_wall = virtual_to_wall_us(span_v_us);
        let start = Instant::now();
        wait_virtual_us(span_v_us);
        let elapsed_us = start.elapsed().as_micros() as u64;
        assert!(
            elapsed_us + 1_000 >= expected_wall,
            "{scale}x: {span_v_us} virtual µs should park ~{expected_wall} wall µs, \
             parked {elapsed_us}"
        );
        init(1.0, 1_000_000);
    }

    /// [`wait_virtual_us`] needs no clock origin (it is pure scale
    /// arithmetic), which is why the sites that only know a *duration* — the
    /// `HAL_time_wait*` trampolines, a receive timeout — use it rather than
    /// [`wait_until`]. Asserted as a zero-span no-op so the case is cheap
    /// whether or not a sibling test already ran `init` in this process.
    #[rstest]
    fn wait_virtual_us_needs_no_clock_origin() {
        let _g = lock_or_recover();
        wait_virtual_us(0);
    }

    /// [`wait_until`] treats its argument as an absolute deadline: a deadline
    /// already in the past returns immediately (never a negative wait that
    /// wraps into a very long sleep), and a future deadline parks until virtual
    /// time reaches it.
    #[rstest]
    fn wait_until_is_an_absolute_deadline_and_never_wraps() {
        let _g = lock_or_recover();
        init(1.0, 1_000_000);

        // Past deadline: immediate. `saturating_sub` is what makes this safe.
        let start = Instant::now();
        wait_until(0);
        assert!(
            start.elapsed() < std::time::Duration::from_millis(200),
            "a past deadline must return immediately, took {:?}",
            start.elapsed()
        );

        // Future deadline: virtual time has passed it on return.
        let deadline = virtual_us() + 2_000;
        wait_until(deadline);
        assert!(
            virtual_us() >= deadline,
            "wait_until must not return before its deadline"
        );
    }

    /// [`wait_wall_us`] is deliberately unscaled — that is its whole reason to
    /// exist. At 10x a 2 ms *wall* wait still takes ~2 ms, where the same span
    /// through [`wait_virtual_us`] would take ~0.2 ms.
    #[rstest]
    fn wait_wall_us_ignores_the_time_scale() {
        let _g = lock_or_recover();
        init(10.0, 1_000_000);
        let start = Instant::now();
        wait_wall_us(2_000);
        let elapsed_us = start.elapsed().as_micros() as u64;
        assert!(
            elapsed_us + 500 >= 2_000,
            "a wall wait must not be divided by the 10x scale, parked {elapsed_us} µs"
        );
        assert_eq!(
            virtual_to_wall_us(2_000),
            200,
            "the same span as a VIRTUAL wait would have been 200 wall µs"
        );
        init(1.0, 1_000_000);
    }

    // ========================================================
    // Stepped mode (DETERMINISM.md Phase D1)
    // ========================================================

    /// Enter stepped mode for the duration of a test and restore free-running
    /// on the way out, panic or not. Takes the shared clock lock, so stepped
    /// and free-running cases in this binary can never overlap.
    struct SteppedGuard(#[allow(dead_code)] std::sync::MutexGuard<'static, ()>);

    impl SteppedGuard {
        fn enter() -> Self {
            let guard = lock_or_recover();
            init_mode(ClockMode::Stepped, 1_000_000);
            Self(guard)
        }
    }

    impl Drop for SteppedGuard {
        fn drop(&mut self) {
            init(1.0, 1_000_000);
        }
    }

    /// The mode discriminant round-trips, and free-running is the default a
    /// plain [`init`] selects — nothing about an existing consumer changes.
    #[rstest]
    fn init_selects_free_running_and_init_mode_selects_stepped() {
        let _g = lock_or_recover();
        init(2.0, 1_000_000);
        assert!(!is_stepped());
        assert_eq!(mode(), ClockMode::FreeRunning { speed: 2.0 });

        init_mode(ClockMode::Stepped, 1_000_000);
        assert!(is_stepped());
        assert_eq!(mode(), ClockMode::Stepped);

        init(1.0, 1_000_000);
        assert!(!is_stepped());
    }

    /// The defining property of stepped mode: `virtual_us` is *only* what the
    /// scheduler set. Wall time passing changes nothing.
    #[rstest]
    fn stepped_time_moves_only_on_advance_to() {
        let _g = SteppedGuard::enter();
        assert_eq!(virtual_us(), 0);

        // Burn real time; virtual time must not budge.
        let start = Instant::now();
        while start.elapsed() < Duration::from_millis(20) {
            assert_eq!(virtual_us(), 0, "stepped time must not track wall time");
        }

        advance_to(1_000).expect("forward advance");
        assert_eq!(virtual_us(), 1_000);
        assert_eq!(virtual_ms(), 1);
        advance_to(1_000).expect("advancing to the same instant is allowed");
        assert_eq!(virtual_us(), 1_000);
        advance_to(2_500_000).expect("forward advance");
        assert_eq!(virtual_cycles(), 2_500_000, "1 MHz × 2.5 s");
    }

    /// `advance_to` is monotonic, and rejected outright in free-running mode —
    /// only a stepped clock has a "now" anyone may set.
    #[rstest]
    fn advance_to_rejects_backwards_and_free_running() {
        {
            let _g = SteppedGuard::enter();
            advance_to(500).expect("forward");
            assert_eq!(
                advance_to(499),
                Err(AdvanceError::WentBackwards {
                    now_us: 500,
                    requested_us: 499
                })
            );
            assert_eq!(virtual_us(), 500, "a rejected advance changes nothing");
        }
        let _g = lock_or_recover();
        init(1.0, 1_000_000);
        assert_eq!(advance_to(1), Err(AdvanceError::NotStepped));
    }

    /// `set_scale` is a loud no-op while stepped: scaling a clock the scheduler
    /// sets by hand is meaningless (`DETERMINISM.md` T1 §1).
    #[rstest]
    fn set_scale_is_a_no_op_in_stepped_mode() {
        let _g = SteppedGuard::enter();
        advance_to(1_000).expect("forward");
        set_scale(50.0);
        assert_eq!(mode(), ClockMode::Stepped, "mode is unchanged");
        assert_eq!(virtual_us(), 1_000, "time is unchanged");
        assert_eq!(
            virtual_to_wall_us(1_000),
            1_000,
            "the scale ratio stays 1:1 in stepped mode"
        );
    }

    /// A `wait_until` in stepped mode parks until the scheduler advances — it
    /// never returns on wall time — and its deadline is visible to the
    /// scheduler *before* it returns, so nothing can be waited on forever by
    /// accident.
    #[rstest]
    fn stepped_wait_until_parks_until_the_scheduler_advances() {
        let _g = SteppedGuard::enter();
        let woke = Arc::new(AtomicU64::new(0));
        let sink = Arc::clone(&woke);
        let waiter = std::thread::spawn(move || {
            wait_until(5_000);
            sink.store(virtual_us(), Ordering::SeqCst);
        });

        // The waiter's deadline becomes visible; virtual time stays put.
        assert!(
            spin_until(|| scheduler_state().next_deadline_us == Some(5_000)),
            "the park deadline must be published to the scheduler"
        );
        std::thread::sleep(Duration::from_millis(20));
        assert_eq!(woke.load(Ordering::SeqCst), 0, "wall time must not wake it");

        advance_to(4_999).expect("forward");
        std::thread::sleep(Duration::from_millis(5));
        assert_eq!(woke.load(Ordering::SeqCst), 0, "one µs short must not wake");

        advance_to(5_000).expect("forward");
        waiter.join().expect("waiter joins");
        assert_eq!(woke.load(Ordering::SeqCst), 5_000);
        assert_eq!(
            scheduler_state().next_deadline_us,
            None,
            "a released park leaves no pending deadline behind"
        );
    }

    /// A deadline already in the past never parks — the same contract
    /// free-running mode has, so a call site cannot behave differently by mode.
    #[rstest]
    fn stepped_past_deadline_returns_immediately() {
        let _g = SteppedGuard::enter();
        advance_to(10_000).expect("forward");
        let start = Instant::now();
        wait_until(0);
        wait_until(10_000);
        wait_virtual_us(0);
        assert!(
            start.elapsed() < Duration::from_millis(100),
            "past/equal deadlines must return immediately"
        );
    }

    /// The quiescence barrier, end to end: a registered actor holds time back
    /// while it is doing work, `await_quiescence` returns only once it parks,
    /// and `advance_to` restores its runnable accounting **before** it returns
    /// — so a scheduler can never step past the instant it just woke it for.
    #[rstest]
    fn registered_actor_holds_the_barrier_until_it_parks() {
        let _g = SteppedGuard::enter();
        let gate = Arc::new(AtomicU64::new(0));
        let observed = Arc::new(Mutex::new(Vec::<u64>::new()));

        let actor_gate = Arc::clone(&gate);
        let actor_observed = Arc::clone(&observed);
        let actor = std::thread::spawn(move || {
            let _registration = register_actor("test-actor");
            // Stay runnable until the test lets go.
            while actor_gate.load(Ordering::SeqCst) == 0 {
                std::thread::yield_now();
            }
            for _ in 0..3 {
                wait_virtual_us(100);
                actor_observed.lock().unwrap().push(virtual_us());
            }
        });

        assert!(
            spin_until(|| scheduler_state().running == 1),
            "the actor must register as runnable"
        );
        // While it is runnable, quiescence is unreachable and it is named.
        match await_quiescence(Duration::from_millis(50)) {
            Quiescence::Stalled { actors } => assert_eq!(actors, vec!["test-actor".to_string()]),
            other => panic!("a runnable actor must hold the barrier, got {other:?}"),
        }

        gate.store(1, Ordering::SeqCst);
        for step in 1..=3u64 {
            match await_quiescence(Duration::from_secs(5)) {
                Quiescence::Reached { next_deadline_us } => {
                    assert_eq!(next_deadline_us, Some(step * 100));
                }
                other => panic!("step {step}: actor should have parked, got {other:?}"),
            }
            assert_eq!(
                scheduler_state().running,
                0,
                "a parked actor is not runnable"
            );
            advance_to(step * 100).expect("forward");
            assert_eq!(
                scheduler_state().running,
                1,
                "advance_to must restore the woken actor's runnable accounting \
                 BEFORE it returns"
            );
        }
        actor.join().expect("actor joins");
        assert_eq!(*observed.lock().unwrap(), vec![100, 200, 300]);
        assert_eq!(registered_actors(), 0, "dropping the guard unregisters");
    }

    /// An **unregistered** waiter is a documented half-citizen: its deadline is
    /// published (so a scheduler always releases it) but it does not hold the
    /// barrier — the scheduler has no way to know when it is between waits.
    #[rstest]
    fn unregistered_waiters_publish_a_deadline_but_do_not_hold_the_barrier() {
        let _g = SteppedGuard::enter();
        let done = Arc::new(AtomicU64::new(0));
        let sink = Arc::clone(&done);
        let waiter = std::thread::spawn(move || {
            wait_until(300);
            sink.store(1, Ordering::SeqCst);
        });
        assert!(
            spin_until(|| scheduler_state().next_deadline_us == Some(300)),
            "an unregistered park still publishes its deadline"
        );
        assert_eq!(
            scheduler_state().running,
            0,
            "an unregistered waiter is never counted as a runnable actor"
        );
        assert!(matches!(
            await_quiescence(Duration::from_millis(50)),
            Quiescence::Reached {
                next_deadline_us: Some(300)
            }
        ));
        advance_to(300).expect("forward");
        waiter.join().expect("waiter joins");
        assert_eq!(done.load(Ordering::SeqCst), 1);
    }

    /// The wall-sleep tripwire: a `wait_wall_us` in stepped mode is counted and
    /// logged, because the barrier cannot see it (`DETERMINISM.md` T1 §4).
    /// A zero-length wait is not a sleep and must not trip it.
    #[rstest]
    fn wall_sleeps_in_stepped_mode_trip_the_tripwire() {
        let _g = SteppedGuard::enter();
        let before = stepped_wall_sleep_count();
        wait_wall_us(0);
        assert_eq!(
            stepped_wall_sleep_count(),
            before,
            "a zero wait never sleeps, so it never trips"
        );
        wait_wall_us(200);
        assert_eq!(stepped_wall_sleep_count(), before + 1);
        // Virtual waits must NOT trip it — they are the whole point.
        wait_virtual_us(0);
        assert_eq!(stepped_wall_sleep_count(), before + 1);
    }

    /// Entering stepped mode with an actor left over from a previous run is a
    /// panic, not a warning: a leaked actor thread is exactly what would make
    /// the next run's barrier lie.
    #[rstest]
    #[should_panic(expected = "still registered")]
    fn entering_stepped_mode_with_a_leaked_actor_panics() {
        let _g = lock_or_recover();
        init(1.0, 1_000_000);
        let _leaked = register_actor("leaked-from-a-previous-run");
        init_mode(ClockMode::Stepped, 1_000_000);
    }

    /// One thread is one actor: a nested registration means two owners
    /// disagree about who parks it.
    #[rstest]
    #[should_panic(expected = "already registered")]
    fn double_registration_on_one_thread_panics() {
        let _g = lock_or_recover();
        let _first = register_actor("first");
        let _second = register_actor("second");
    }

    /// Re-`init` is an in-process restart: it re-anchors stepped time to 0 and
    /// releases anything parked against the old timeline, rather than leaving
    /// it waiting for a deadline that can no longer arrive.
    #[rstest]
    fn reinit_releases_parks_from_the_previous_epoch() {
        let _g = SteppedGuard::enter();
        let released = Arc::new(AtomicU64::new(0));
        let sink = Arc::clone(&released);
        let waiter = std::thread::spawn(move || {
            wait_until(1_000_000);
            sink.store(1, Ordering::SeqCst);
        });
        assert!(
            spin_until(|| scheduler_state().next_deadline_us == Some(1_000_000)),
            "the park must be registered before the restart"
        );
        // A restart, without ever advancing to that deadline.
        init_mode(ClockMode::Stepped, 1_000_000);
        waiter.join().expect("the stale park must be released");
        assert_eq!(released.load(Ordering::SeqCst), 1);
        assert_eq!(virtual_us(), 0, "restart re-anchors stepped time to 0");
    }

    /// Spin (bounded) until a predicate holds; a test helper only — nothing in
    /// the clock itself waits on wall time in stepped mode.
    fn spin_until(mut pred: impl FnMut() -> bool) -> bool {
        let start = Instant::now();
        while start.elapsed() < Duration::from_secs(5) {
            if pred() {
                return true;
            }
            std::thread::yield_now();
        }
        pred()
    }

    /// Re-`init` re-anchors the boot offset, so virtual time restarts near zero.
    /// We can't assert an exact value (wall time keeps moving), but immediately
    /// after a re-init the reading must be small relative to a coarse ceiling.
    #[rstest]
    fn reinit_reanchors_virtual_time() {
        let _g = lock_or_recover();
        init(1.0, 180_000_000);
        // Burn some virtual time.
        for _ in 0..50_000 {
            let _ = virtual_us();
        }
        let before = virtual_us();
        // Re-init should drop the reading back toward zero.
        init(1.0, 180_000_000);
        let after = virtual_us();
        // The re-anchored reading must be far below the accumulated `before`
        // (or `before` itself was tiny on a very fast machine — either way the
        // post-init value cannot exceed a generous 1-second ceiling).
        assert!(
            after < 1_000_000,
            "re-anchored virtual_us should be small, got {after}"
        );
        let _ = before;
    }

    /// Host threads that are not actors must not be preempted.
    #[rstest]
    fn charge_is_a_no_op_without_an_actor() {
        let _g = SteppedGuard::enter();
        for _ in 0..10_000 {
            charge(1);
        }
        assert_eq!(virtual_us(), 0);
        assert_eq!(scheduler_state().running, 0);
    }

    /// A running actor is left alone until its HAL-proxy slice fills.
    #[rstest]
    fn charge_below_quantum_does_not_park() {
        let _g = SteppedGuard::enter();
        let gate = Arc::new(AtomicU64::new(0));
        let actor_gate = Arc::clone(&gate);
        let actor = std::thread::spawn(move || {
            let _registration = register_actor("below-quantum");
            for _ in 0..(DEFAULT_QUANTUM_US - 1) {
                charge(1);
            }
            while actor_gate.load(Ordering::SeqCst) == 0 {
                std::thread::yield_now();
            }
        });
        assert!(
            spin_until(|| scheduler_state().running == 1),
            "the actor must register as runnable"
        );
        match await_quiescence(Duration::from_millis(50)) {
            Quiescence::Stalled { actors } => {
                assert_eq!(actors, vec!["below-quantum".to_string()])
            }
            other => panic!("a slice still open must hold the barrier, got {other:?}"),
        }
        gate.store(1, Ordering::SeqCst);
        actor.join().expect("actor must exit once released");
    }

    /// Exhausting the slice parks the cog so the engine can advance.
    #[rstest]
    fn charge_exhausting_quantum_parks() {
        let _g = SteppedGuard::enter();
        let actor = std::thread::spawn(|| {
            let _registration = register_actor("spinner");
            for _ in 0..DEFAULT_QUANTUM_US {
                charge(1);
            }
        });
        assert!(
            spin_until(|| {
                let s = scheduler_state();
                s.running == 1 || s.next_deadline_us == Some(DEFAULT_QUANTUM_US)
            }),
            "the spinner must register or park"
        );
        match await_quiescence(Duration::from_secs(5)) {
            Quiescence::Reached { next_deadline_us } => {
                assert_eq!(next_deadline_us, Some(DEFAULT_QUANTUM_US));
            }
            other => panic!("exhausting the slice must park, got {other:?}"),
        }
        advance_to(DEFAULT_QUANTUM_US).expect("engine can step to the park");
        actor.join().expect("the cog resumes after the step");
        assert_eq!(virtual_us(), DEFAULT_QUANTUM_US);
    }
}
