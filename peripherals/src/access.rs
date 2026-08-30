//! Unimplemented / out-of-range HAL access.
//!
//! Native SIL used to swallow unknown channels and negative ids. That hides
//! new firmware paths. Every such access is counted and logged; tests can
//! assert the count, and optionally treat a non-zero count as a failure.

use std::sync::atomic::{AtomicU64, Ordering};

static UNIMPLEMENTED: AtomicU64 = AtomicU64::new(0);

/// Record one illegal HAL/peripheral access. Never panics: production
/// playgrounds must keep running; tests assert [`count`].
pub fn report(kind: &'static str, what: &str) {
    UNIMPLEMENTED.fetch_add(1, Ordering::Relaxed);
    tracing::error!(kind, what, "unimplemented or out-of-range HAL access");
}

/// How many reports since process start or the last [`take_count`].
pub fn count() -> u64 {
    UNIMPLEMENTED.load(Ordering::Relaxed)
}

/// Swap the counter to zero and return the previous value (test helper).
pub fn take_count() -> u64 {
    UNIMPLEMENTED.swap(0, Ordering::Relaxed)
}

#[cfg(test)]
mod tests {
    use super::*;
    use rstest::rstest;

    #[rstest]
    fn report_increments_and_take_clears() {
        let _g = crate::test_support::guard();
        let before = count();
        report("test", "unit");
        assert!(count() > before);
        let _ = take_count();
        assert_eq!(count(), 0);
    }
}
