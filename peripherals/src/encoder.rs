//! Encoder — Generic position counter peripheral.
//!
//! State lives in a per-MCU [`Encoder`] bank owned by
//! `instance::PeripheralInstance`. The module-level free functions route to
//! the calling thread's instance (see `crate::instance`), so existing
//! single-MCU consumers are unaffected.

use std::sync::atomic::{AtomicI32, AtomicUsize, Ordering};
use tracing::trace;

/// Maximum encoder channels supported (hard ceiling of the backing array).
pub const MAX_CHANNELS: usize = 16;

/// Encoder counter bank for one MCU instance.
pub struct Encoder {
    /// Configured channel count.
    count: AtomicUsize,
    /// Encoder values (atomic for thread safety).
    values: [AtomicI32; MAX_CHANNELS],
}

impl Encoder {
    /// Create a bank with no channels configured, all counters at zero.
    pub const fn new() -> Self {
        // justification: this `const` is never read as a value; it only seeds
        // the `[INIT; N]` array-repeat initializer for the field below.
        // Array-repeat syntax *requires* a `const`, and no interior mutability
        // is ever observed through the const itself.
        #[allow(clippy::declare_interior_mutable_const)]
        const VALUE_INIT: AtomicI32 = AtomicI32::new(0);
        Self {
            count: AtomicUsize::new(0),
            values: [VALUE_INIT; MAX_CHANNELS],
        }
    }

    /// Configure the encoder peripheral with the number of channels.
    /// Resets all counters, so re-init yields a clean state.
    ///
    /// # Panics
    /// If `count` exceeds [`MAX_CHANNELS`].
    pub fn init(&self, count: usize) {
        assert!(
            count <= MAX_CHANNELS,
            "Encoder count {} exceeds max {}",
            count,
            MAX_CHANNELS
        );
        self.reset();
        self.count.store(count, Ordering::Relaxed);
    }

    /// Clear all encoder values and channel count (used by `init` and teardown).
    pub fn reset(&self) {
        self.count.store(0, Ordering::Relaxed);
        for v in self.values.iter() {
            v.store(0, Ordering::Relaxed);
        }
    }

    /// Start an encoder channel (no-op in emulation).
    pub fn start(&self, channel: usize) {
        trace!("encoder::start(channel={})", channel);
    }

    /// Get the current encoder value.
    pub fn value(&self, channel: usize) -> i32 {
        if channel < self.count.load(Ordering::Relaxed) {
            self.values[channel].load(Ordering::Relaxed)
        } else {
            0
        }
    }

    /// Set the encoder value (used by MCU HAL or wiring layer).
    pub fn set(&self, channel: usize, val: i32) {
        if channel < self.count.load(Ordering::Relaxed) {
            self.values[channel].store(val, Ordering::Relaxed);
            trace!("encoder::set(channel={}, value={})", channel, val);
        }
    }
}

impl Default for Encoder {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================
// Free functions — route to the calling thread's instance
// ============================================================

/// Configure the encoder peripheral with the number of channels.
/// Resets all counters, so re-init yields a clean state.
pub fn init(count: usize) {
    crate::instance::current().encoder.init(count);
}

/// Clear all encoder values and channel count (used by `init` and teardown).
pub fn reset() {
    crate::instance::current().encoder.reset();
}

/// Start an encoder channel (no-op in emulation).
pub fn start(channel: usize) {
    crate::instance::current().encoder.start(channel);
}

/// Get the current encoder value.
pub fn value(channel: usize) -> i32 {
    crate::instance::current().encoder.value(channel)
}

/// Set the encoder value (used by MCU HAL or wiring layer).
pub fn set(channel: usize, val: i32) {
    crate::instance::current().encoder.set(channel, val);
}

// ============================================================
// Wiring API
// ============================================================

/// Set encoder value from external source.
pub fn set_value(channel: usize, val: i32) {
    set(channel, val);
}

// ============================================================
// Tests
// ============================================================

#[cfg(test)]
mod tests {
    use super::*;

    /// Take the crate test lock, pin the clock, and reset the encoder bank.
    fn setup(count: usize) {
        crate::test_support::ensure_clock();
        init(count);
    }

    #[test]
    fn init_at_max_channels_is_allowed() {
        let _g = crate::test_support::guard();
        // Exactly MAX_CHANNELS is the inclusive upper bound.
        setup(MAX_CHANNELS);
        assert_eq!(value(MAX_CHANNELS - 1), 0);
    }

    #[test]
    #[should_panic(expected = "exceeds max")]
    fn init_above_max_channels_panics() {
        let _g = crate::test_support::guard();
        crate::test_support::ensure_clock();
        init(MAX_CHANNELS + 1);
    }

    #[test]
    fn value_and_set_in_range() {
        let _g = crate::test_support::guard();
        setup(4);
        assert_eq!(value(2), 0, "encoders start at zero");
        set(2, -1234);
        assert_eq!(value(2), -1234);
        set(2, i32::MAX);
        assert_eq!(value(2), i32::MAX);
    }

    #[test]
    fn out_of_range_value_is_zero_and_set_is_a_no_op() {
        let _g = crate::test_support::guard();
        setup(2);
        // Channel 9 is unconfigured: read returns 0, write is dropped.
        assert_eq!(value(9), 0);
        set(9, 5000);
        assert_eq!(value(9), 0);
        // In-range channels remain untouched.
        assert_eq!(value(0), 0);
    }

    #[test]
    fn set_value_is_an_alias_for_set() {
        let _g = crate::test_support::guard();
        setup(2);
        set_value(1, 777);
        assert_eq!(value(1), 777);
        // Out-of-range alias is likewise a no-op.
        set_value(50, 1);
        assert_eq!(value(50), 0);
    }

    #[test]
    fn reset_zeroes_all_values_and_count() {
        let _g = crate::test_support::guard();
        setup(3);
        set(0, 10);
        set(1, 20);
        set(2, 30);
        reset();
        // Count is cleared, so every read is out-of-range → 0.
        assert_eq!(value(0), 0);
        assert_eq!(value(1), 0);
        assert_eq!(value(2), 0);
        // Re-init exposes the (already-zeroed) storage.
        init(3);
        assert_eq!(value(0), 0);
        assert_eq!(value(2), 0);
    }

    #[test]
    fn start_is_a_no_op() {
        let _g = crate::test_support::guard();
        setup(1);
        // start() is a documented no-op in emulation; it must not panic or
        // change the value.
        set(0, 99);
        start(0);
        assert_eq!(value(0), 99);
    }
}
