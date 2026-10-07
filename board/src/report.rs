//! What a run prints about what a catalog built.
//!
//! A core's console, a PTY's path and a carriage's travel are not findings,
//! and no net carries them. Anything a catalog builds hands the run a
//! [`Report`] instead, through the sink a project build carries
//! ([`Reports`]; [`crate::Assignment::reports`] for a part's constructor,
//! [`crate::ComponentRequest::reports`] for a bench component's). The run
//! takes the reports once the system is built ([`Reports::take`]), asks
//! each what is new at every look, and asks each for its state at the end.
//! A report whose subject can fail — a core whose program died — says so
//! ([`Report::failure`]), and the run stops there and exits with it.
//!
//! A look reads state the engine's thread wrote while the run's thread was
//! parked, so on the stepped clock two runs of one project print the same
//! lines at the same instants.

use std::sync::{Arc, Mutex};

/// A virtual instant as milliseconds, exactly to the nanosecond, the way a
/// run prints every instant: `5.500000 ms`.
pub fn instant(ns: u64) -> String {
    format!("{}.{:06} ms", ns / 1_000_000, ns % 1_000_000)
}

/// One thing a catalog built, as a run reports it.
pub trait Report: Send {
    /// What the lines are about, as the run prints it before each: the
    /// part (`"EC32.U100"`) or the bench component (`"HOST"`).
    fn subject(&self) -> String;

    /// What is new since the last look, at `now_ns` of the run's virtual
    /// time: one line each, said without the subject. The first look is
    /// at 0, before virtual time moves.
    fn look(&mut self, now_ns: u64) -> Vec<String>;

    /// The state at the end of the run, one line each.
    fn summary(&self) -> Vec<String>;

    /// Why what this reports has failed, once it has: a part whose model
    /// can no longer run (a CPU core whose program died). A run asks after
    /// every look, stops at the first look that finds one, prints its
    /// summary, and exits with the failure as its error. `None` — the
    /// default — for something that cannot fail.
    fn failure(&self) -> Option<String> {
        None
    }
}

/// The sink the reports of one project build go to: shared by every
/// constructor the build calls, and taken by the run that started it.
#[derive(Clone, Default)]
pub struct Reports {
    inner: Arc<Mutex<Vec<Box<dyn Report>>>>,
}

impl std::fmt::Debug for Reports {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let subjects: Vec<String> = self
            .inner
            .lock()
            .expect("the report sink is never poisoned")
            .iter()
            .map(|report| report.subject())
            .collect();
        f.debug_struct("Reports")
            .field("subjects", &subjects)
            .finish()
    }
}

impl Reports {
    /// An empty sink.
    pub fn new() -> Self {
        Self::default()
    }

    /// Add `report`: a constructor calls this when it builds what the
    /// report is about, never when a kind registers (a survey registers and
    /// builds nothing).
    pub fn add(&self, report: impl Report + 'static) {
        self.inner
            .lock()
            .expect("the report sink is never poisoned")
            .push(Box::new(report));
    }

    /// Every report added so far, in the order they were added, leaving the
    /// sink empty.
    pub fn take(&self) -> Vec<Box<dyn Report>> {
        std::mem::take(
            &mut *self
                .inner
                .lock()
                .expect("the report sink is never poisoned"),
        )
    }

    /// How many reports the sink holds.
    pub fn len(&self) -> usize {
        self.inner
            .lock()
            .expect("the report sink is never poisoned")
            .len()
    }

    /// Whether the sink holds none.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}
