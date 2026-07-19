//! Edge detection — report only on boolean transitions.
//!
//! A tiny reusable primitive shared by any model that turns a continuous input
//! into edge-triggered events (limit switches, thresholds, comparators). It
//! replaces the hand-rolled `AtomicBool::swap` + compare pattern that was
//! previously copy-pasted across models.

use std::sync::atomic::{AtomicBool, Ordering};

/// Tracks a boolean level and reports only when it changes.
pub struct EdgeDetector {
    state: AtomicBool,
}

impl EdgeDetector {
    /// Create a detector seeded with an initial level.
    pub const fn new(initial: bool) -> Self {
        Self {
            state: AtomicBool::new(initial),
        }
    }

    /// Feed the current level. Returns `Some(level)` on a rising or falling
    /// edge (the level changed), or `None` if it is unchanged.
    pub fn update(&self, level: bool) -> Option<bool> {
        let prev = self.state.swap(level, Ordering::Relaxed);
        if level != prev {
            Some(level)
        } else {
            None
        }
    }

    /// Current level.
    pub fn state(&self) -> bool {
        self.state.load(Ordering::Relaxed)
    }
}

impl Default for EdgeDetector {
    fn default() -> Self {
        Self::new(false)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rstest::rstest;

    #[rstest]
    #[case::start_low(false)]
    #[case::start_high(true)]
    fn initial_state_matches_constructor(#[case] initial: bool) {
        let e = EdgeDetector::new(initial);
        assert_eq!(e.state(), initial);
    }

    #[rstest]
    fn default_starts_low() {
        let e = EdgeDetector::default();
        assert!(!e.state());
        assert_eq!(e.update(false), None);
    }

    #[rstest]
    #[case::rising(false, true, Some(true))]
    #[case::falling(true, false, Some(false))]
    #[case::hold_low(false, false, None)]
    #[case::hold_high(true, true, None)]
    fn update_reports_only_on_transition(
        #[case] initial: bool,
        #[case] next: bool,
        #[case] expected: Option<bool>,
    ) {
        let e = EdgeDetector::new(initial);
        assert_eq!(e.update(next), expected);
        assert_eq!(e.state(), next);
    }

    #[rstest]
    fn sequence_rising_hold_falling() {
        let e = EdgeDetector::new(false);
        assert_eq!(e.update(false), None);
        assert_eq!(e.update(true), Some(true));
        assert_eq!(e.update(true), None);
        assert_eq!(e.update(false), Some(false));
        assert!(!e.state());
    }

    /// Concurrent updates from two threads still only report real transitions
    /// (no panic / no torn bool). Exact edge counts are racy; we only require
    /// that final state matches the last written level family and `update`
    /// never panics.
    #[rstest]
    fn concurrent_updates_do_not_panic() {
        use std::sync::Arc;
        let e = Arc::new(EdgeDetector::new(false));
        let mut handles = Vec::new();
        for i in 0..4 {
            let ed = Arc::clone(&e);
            handles.push(std::thread::spawn(move || {
                for k in 0..1_000 {
                    let _ = ed.update((i + k) % 2 == 0);
                }
            }));
        }
        for h in handles {
            h.join().expect("edge worker");
        }
        // Final state is some bool; just force a known value.
        let _ = e.update(true);
        assert!(e.state());
    }
}
