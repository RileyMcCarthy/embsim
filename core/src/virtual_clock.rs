//! Virtual Clock — provides scalable time for the emulator.
//!
//! All timer functions route through this module.
//! At 1x: virtual time == wall time
//! At 5x: virtual time advances 5x faster (waits are 5x shorter)
//! At 0.5x: virtual time advances 0.5x (waits are 2x longer)
//!
//! # Waiting
//!
//! Nothing outside this module may call [`std::thread::sleep`] to serve a
//! *simulated* wait. Every wait in the workspace goes through one of three
//! functions here, and one private `park_wall_us` is the only place that
//! actually sleeps:
//!
//! | call | meaning |
//! |---|---|
//! | [`wait_until`] | park until virtual time reaches an absolute deadline |
//! | [`wait_virtual_us`] | park for a *relative* span of virtual time |
//! | [`wait_wall_us`] | park for real time — deliberately **not** virtual |
//!
//! This is the migration lever for `DETERMINISM.md` Phase D1: a stepped clock
//! replaces the bodies of the first two (register a deadline, mark the caller
//! parked, block until the scheduler advances) without touching a single call
//! site. [`wait_wall_us`] marks the waits that are wall-clock *by nature* —
//! fd-poll retry cadence, a startup warm-up — which D1 must revisit
//! deliberately as a semantic change, not sweep up by accident. See
//! `DETERMINISM.md`, "T1 §5 Migration lever" and "Wall-clock deadlines inside
//! the simulation".

use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};
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

/// Initialize the virtual clock with the given speed scale and clock frequency.
/// Must be called before any time functions. Calling it again re-anchors
/// virtual time to 0 (an in-process restart) and updates the scale/frequency.
pub fn init(speed: f64, freq: u32) {
    let origin = PROCESS_ORIGIN.get_or_init(Instant::now);
    BOOT_OFFSET_US.store(origin.elapsed().as_micros() as u64, Ordering::Relaxed);
    CLOCK_FREQ.store(freq, Ordering::Relaxed);
    set_scale(speed);
}

/// Change the time scale at runtime.
/// Uses integer numerator/denominator to avoid floating point in the hot path.
pub fn set_scale(scale: f64) {
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
pub fn virtual_us() -> u64 {
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

// ============================================================
// Waiting — the single chokepoint between virtual and wall time
// ============================================================

/// Park the caller for `wall_us` of **real** time. The only `thread::sleep`
/// serving a simulated wait in the workspace; a zero wait never sleeps.
fn park_wall_us(wall_us: u64) {
    if wall_us > 0 {
        std::thread::sleep(Duration::from_micros(wall_us));
    }
}

/// Park the caller for `d_us` of **virtual** time (a relative wait).
///
/// Free-running: the scaled wall equivalent, i.e. exactly
/// `sleep(virtual_to_wall_us(d_us))`. Needs no clock origin, so it is safe
/// before [`init`] — a wait span is pure scale arithmetic, unlike
/// [`wait_until`], which must read the current time.
///
/// This is the sibling of [`wait_until`] for the many call sites that know how
/// long to wait but not *when* they started (`HAL_time_waitUs`, a poll
/// cadence, a receive timeout). In `DETERMINISM.md` Phase D1 it becomes
/// `wait_until(virtual_us() + d_us)` against the stepped clock.
pub fn wait_virtual_us(d_us: u64) {
    park_wall_us(virtual_to_wall_us(d_us));
}

/// Park the caller until virtual time reaches the absolute deadline
/// `deadline_v_us`. Returns immediately when the deadline has already passed.
///
/// This is the form `DETERMINISM.md` Phase D1 swaps out: in stepped mode it
/// registers a pending deadline, marks the caller parked, and blocks until the
/// scheduler advances virtual time to it. Prefer it wherever the call site
/// genuinely holds a deadline (a reserved wire slot, a scheduled edge) rather
/// than a duration — an absolute deadline cannot drift, and it is the only
/// form a discrete-event scheduler can serve.
///
/// # Panics
/// Panics if [`init`] has not run — reading "now" requires the clock origin.
/// Callers that must survive an uninitialized clock check
/// [`is_initialized`] first, or use [`wait_virtual_us`].
pub fn wait_until(deadline_v_us: u64) {
    let now = virtual_us();
    wait_virtual_us(deadline_v_us.saturating_sub(now));
}

/// Park the caller for `d_us` of **real** time, bypassing the virtual clock.
///
/// Reserved for waits that are wall-clock by nature and must stay that way in
/// free-running mode: the retry cadence of a spin on a real file descriptor,
/// and a fixed startup warm-up. These are the sites `DETERMINISM.md` (T1 §4,
/// "Wall-clock deadlines inside the simulation") says must *become* virtual —
/// naming them here means D1 can find them, rather than discovering them as a
/// stepped-mode hang. Do not reach for this for anything the simulation's
/// timing depends on; use [`wait_until`] or [`wait_virtual_us`].
pub fn wait_wall_us(d_us: u64) {
    park_wall_us(d_us);
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
    use std::sync::Mutex as StdMutex;

    /// The virtual clock mutates process-global scale / frequency / boot-offset
    /// state, so every test that touches it must run serially. Recover from any
    /// panic-induced poisoning exactly like the `pulse_out` reference suite.
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
}
