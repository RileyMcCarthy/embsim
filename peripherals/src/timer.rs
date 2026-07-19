//! Timer — Timing functions backed by VirtualClock.
//!
//! Virtual *time* is process-wide (free-running scaled wall time in
//! `embsim_core::virtual_clock`), but the *clock frequency* used for cycle
//! math is per-MCU: [`get_clock_freq`] and [`get_cycles`] honor the calling
//! thread's `instance::PeripheralInstance` clock-frequency override, falling
//! back to the virtual clock's process-wide frequency when unset.

use embsim_core::virtual_clock;

/// Get virtual milliseconds elapsed since boot.
pub fn get_ms() -> u32 {
    virtual_clock::virtual_ms() as u32
}

/// Get virtual microseconds elapsed since boot.
pub fn get_us() -> u32 {
    virtual_clock::virtual_us() as u32
}

/// Blocking wait for specified virtual milliseconds.
pub fn wait_ms(ms: u32) {
    let wall_us = virtual_clock::virtual_to_wall_us(ms as u64 * 1000);
    if wall_us > 0 {
        std::thread::sleep(std::time::Duration::from_micros(wall_us));
    }
}

/// Blocking wait for specified virtual microseconds.
pub fn wait_us(us: u32) {
    let wall_us = virtual_clock::virtual_to_wall_us(us as u64);
    if wall_us > 0 {
        std::thread::sleep(std::time::Duration::from_micros(wall_us));
    }
}

/// Get raw virtual cycle counter value (`virtual_us * clock_freq / 1_000_000`,
/// using the calling thread's instance clock frequency).
pub fn get_cycles() -> u32 {
    let freq = crate::instance::current().effective_clock_freq() as u128;
    let us = virtual_clock::virtual_us() as u128;
    (us * freq / 1_000_000) as u32
}

/// Get the simulated clock frequency (the calling thread's instance override,
/// or the process-wide virtual clock frequency when unset).
pub fn get_clock_freq() -> u32 {
    crate::instance::current().effective_clock_freq()
}

// ============================================================
// Tests
// ============================================================

#[cfg(test)]
mod tests {
    use rstest::rstest;

    use super::*;

    #[rstest]
    fn ms_and_us_are_monotonic_non_decreasing() {
        let _g = crate::test_support::guard();
        crate::test_support::ensure_clock();
        // Time only ever moves forward; consecutive reads never go backwards.
        let mut last_ms = get_ms();
        let mut last_us = get_us();
        for _ in 0..50 {
            let ms = get_ms();
            let us = get_us();
            assert!(ms >= last_ms, "ms must be non-decreasing");
            assert!(us >= last_us, "us must be non-decreasing");
            last_ms = ms;
            last_us = us;
        }
    }

    #[rstest]
    fn ms_roughly_tracks_us() {
        let _g = crate::test_support::guard();
        crate::test_support::ensure_clock();
        // ms should be approximately us/1000. We avoid exact wall-clock claims:
        // read us bracketing the ms read and assert ms falls within that window
        // (in ms units), plus a small slack for the time elapsed between reads.
        let us_before = get_us() as u64;
        let ms = get_ms() as u64;
        let us_after = get_us() as u64;
        let lo = us_before / 1000;
        let hi = us_after / 1000 + 2; // +2ms slack
        assert!(
            ms + 1 >= lo && ms <= hi,
            "ms={ms} not within [{lo},{hi}] from us window"
        );
    }

    #[rstest]
    fn clock_freq_is_the_pinned_frequency() {
        let _g = crate::test_support::guard();
        crate::test_support::ensure_clock();
        // ensure_clock pins 180 MHz for the whole crate.
        assert_eq!(get_clock_freq(), 180_000_000);
    }

    #[rstest]
    fn cycles_grow_with_a_positive_frequency() {
        let _g = crate::test_support::guard();
        crate::test_support::ensure_clock();
        // With a non-zero freq, cycles is non-decreasing and eventually advances.
        let first = get_cycles();
        let mut saw_growth = false;
        let mut last = first;
        for _ in 0..1000 {
            let c = get_cycles();
            assert!(c >= last, "cycles must be non-decreasing");
            if c > first {
                saw_growth = true;
                break;
            }
            last = c;
        }
        assert!(
            saw_growth,
            "cycles should advance under a positive clock freq"
        );
    }

    #[rstest]
    fn wait_zero_returns_immediately() {
        let _g = crate::test_support::guard();
        crate::test_support::ensure_clock();
        // A zero-duration wait must not sleep or hang.
        wait_ms(0);
        wait_us(0);
    }

    #[rstest]
    fn wait_small_returns_without_hanging() {
        let _g = crate::test_support::guard();
        crate::test_support::ensure_clock();
        // A tiny wait completes (sub-millisecond at 1x). We assert only that the
        // call returns, never the exact elapsed wall time.
        wait_us(200);
    }
}
