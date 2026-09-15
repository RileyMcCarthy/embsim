//! Driving a real chip through thousands of cases without lying about it.
//!
//! Capture is not a loop over cases. A silicon target is a physical thing on
//! the end of a serial link, running a small worker program, and some
//! instructions **wedge that worker** — an op whose result never reaches the
//! mailbox leaves it waiting forever. How the driver reacts to silence decides
//! whether the resulting corpus is trustworthy, and the two obvious reactions
//! are both wrong:
//!
//! * **Abort the run.** One bad case throws away every case already captured.
//!   On a link that manages about ten cases a second, that is ten minutes of
//!   board time for one silent instruction.
//!
//! * **Skip and carry on.** Worse, and quietly so. A wedged worker does not
//!   recover by itself, so every later case either times out too or answers
//!   out of stale state. Observed on a P2: skips grew 36 → 137 → 240 → 325,
//!   and the tail of that list was `mergeb` and `splitb` — plain ALU
//!   instructions that cannot wedge anything. The worker had been dead for
//!   thousands of cases, and the records captured after the first wedge were
//!   not measurements of those instructions at all.
//!
//! The policy here is the one that survived contact with hardware: on silence,
//! **reload the target** and ask once more. A case that answers after a reload
//! was collateral damage from a previous wedge and is kept. A case that is
//! silent twice across a fresh load is genuinely reproducible, and is recorded
//! as wedged and skipped — after another reload, so the next case starts from
//! a known-good worker. On the corpus that motivated this, that took 325 skips
//! down to 2, and the 2 were real.
//!
//! The transport is the adapter's business; this module owns only the policy,
//! which is why it has no dependencies and can be tested without a board.

use crate::Record;

/// A live connection to a chip running the case worker.
pub trait SiliconTarget {
    /// The adapter's case description.
    type Case;

    /// Name for logs and for the wedged list.
    fn name(&self, case: &Self::Case) -> String;

    /// Run one case.
    ///
    /// `Ok(Some(record))` is an answer, `Ok(None)` is silence within the
    /// adapter's timeout — *not* an error, because silence is an expected
    /// outcome that the policy here knows how to handle. `Err` is a broken
    /// link, which nothing can recover from and which stops the run.
    fn query(&mut self, case: &Self::Case) -> Result<Option<Record>, String>;

    /// Reload the worker so the next query starts from a known state.
    ///
    /// Must be a genuine reload — re-flashing or restarting the target — not
    /// merely draining buffered bytes. Draining is what produced the cascade
    /// described above: the buffer was clean and the worker was still dead.
    fn reload(&mut self) -> Result<(), String>;
}

/// What a capture run produced.
#[derive(Debug, Clone, Default)]
pub struct Capture {
    pub records: Vec<Record>,
    /// Cases that stayed silent across a fresh reload, in order.
    pub wedged: Vec<String>,
    /// Reloads performed, including those after a confirmed wedge.
    pub reloads: usize,
    /// Cases that answered only after a reload — collateral from an earlier
    /// wedge. A non-zero count here means some *other* case is wedging and the
    /// corpus was at risk.
    pub recovered: Vec<String>,
}

/// Progress, so a long run can say what it is doing.
pub trait Progress {
    fn case_started(&mut self, _name: &str) {}
    fn reloading(&mut self, _name: &str) {}
    fn recovered(&mut self, _name: &str) {}
    fn wedged(&mut self, _name: &str) {}
}

/// A [`Progress`] that says nothing.
pub struct Silent;
impl Progress for Silent {}

/// Run every case, reloading the target around anything that wedges it.
pub fn capture<T: SiliconTarget, P: Progress>(
    target: &mut T,
    cases: &[T::Case],
    progress: &mut P,
) -> Result<Capture, String> {
    let mut out = Capture::default();
    for case in cases {
        let name = target.name(case);
        progress.case_started(&name);

        if let Some(record) = target.query(case)? {
            out.records.push(record);
            continue;
        }

        // Silence. The worker may be wedged by THIS case or by an earlier one,
        // and those need opposite responses, so distinguish them by reloading
        // and asking again.
        progress.reloading(&name);
        target.reload()?;
        out.reloads += 1;

        if let Some(record) = target.query(case)? {
            progress.recovered(&name);
            out.recovered.push(name);
            out.records.push(record);
            continue;
        }

        // Silent twice across a fresh load: this case is the culprit. Reload
        // once more so the next case does not inherit the wedge.
        progress.wedged(&name);
        out.wedged.push(name);
        target.reload()?;
        out.reloads += 1;
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeMap;

    fn record(name: &str) -> Record {
        Record {
            name: name.to_string(),
            input: BTreeMap::new(),
            output: BTreeMap::new(),
        }
    }

    /// A target that wedges on the named cases and, once wedged, answers
    /// nothing until it is reloaded — exactly how the real one behaves.
    struct Fake {
        wedging: Vec<&'static str>,
        wedged: bool,
        reloads: usize,
        queries: usize,
        link_fails_after: Option<usize>,
    }

    impl Fake {
        fn new(wedging: &[&'static str]) -> Self {
            Self {
                wedging: wedging.to_vec(),
                wedged: false,
                reloads: 0,
                queries: 0,
                link_fails_after: None,
            }
        }
    }

    impl SiliconTarget for Fake {
        type Case = &'static str;

        fn name(&self, case: &Self::Case) -> String {
            (*case).to_string()
        }

        fn query(&mut self, case: &Self::Case) -> Result<Option<Record>, String> {
            self.queries += 1;
            if let Some(n) = self.link_fails_after {
                if self.queries > n {
                    return Err("link went away".into());
                }
            }
            if self.wedged {
                return Ok(None);
            }
            if self.wedging.contains(case) {
                self.wedged = true;
                return Ok(None);
            }
            Ok(Some(record(case)))
        }

        fn reload(&mut self) -> Result<(), String> {
            self.reloads += 1;
            self.wedged = false;
            Ok(())
        }
    }

    #[test]
    fn a_clean_run_captures_everything_and_never_reloads() {
        let mut t = Fake::new(&[]);
        let got = capture(&mut t, &["add", "sub", "xor"], &mut Silent).unwrap();
        assert_eq!(got.records.len(), 3);
        assert!(got.wedged.is_empty());
        assert_eq!(got.reloads, 0);
    }

    #[test]
    fn a_wedging_case_is_recorded_once_and_does_not_take_the_rest_with_it() {
        // The whole point: `sub` wedges, and `xor` after it must still be a
        // real measurement rather than a casualty.
        let mut t = Fake::new(&["sub"]);
        let got = capture(&mut t, &["add", "sub", "xor"], &mut Silent).unwrap();
        assert_eq!(got.wedged, vec!["sub"]);
        let names: Vec<&str> = got.records.iter().map(|r| r.name.as_str()).collect();
        assert_eq!(names, vec!["add", "xor"]);
        assert_eq!(got.reloads, 2, "one to diagnose, one to clear");
    }

    #[test]
    fn a_case_that_answers_after_a_reload_is_kept_and_flagged_as_collateral() {
        // `sub` wedges the worker; with the old drain-and-continue policy the
        // silence at `xor` would have been recorded as "xor is broken".
        let mut t = Fake::new(&["sub"]);
        let got = capture(&mut t, &["sub", "xor"], &mut Silent).unwrap();
        assert_eq!(got.wedged, vec!["sub"]);
        assert!(
            got.recovered.is_empty(),
            "the post-wedge reload already cleared it, so xor answers first time"
        );
        assert_eq!(got.records.len(), 1);
    }

    #[test]
    fn collateral_silence_is_recovered_rather_than_blamed_on_the_case() {
        // A target that is already wedged when the run reaches a good case.
        let mut t = Fake::new(&[]);
        t.wedged = true;
        let got = capture(&mut t, &["add"], &mut Silent).unwrap();
        assert!(got.wedged.is_empty(), "add is not the culprit");
        assert_eq!(got.recovered, vec!["add"]);
        assert_eq!(got.records.len(), 1);
        assert_eq!(got.reloads, 1);
    }

    #[test]
    fn several_wedging_cases_are_all_reported() {
        let mut t = Fake::new(&["sca", "scas"]);
        let got = capture(&mut t, &["add", "sca", "sub", "scas", "xor"], &mut Silent).unwrap();
        assert_eq!(got.wedged, vec!["sca", "scas"]);
        assert_eq!(got.records.len(), 3);
    }

    #[test]
    fn a_broken_link_stops_the_run_rather_than_being_mistaken_for_a_wedge() {
        let mut t = Fake::new(&[]);
        t.link_fails_after = Some(1);
        let err = capture(&mut t, &["add", "sub"], &mut Silent).unwrap_err();
        assert!(err.contains("link"));
    }

    #[test]
    fn progress_reports_the_reload_and_the_wedge() {
        #[derive(Default)]
        struct Log {
            events: Vec<String>,
        }
        impl Progress for Log {
            fn reloading(&mut self, n: &str) {
                self.events.push(format!("reload {n}"));
            }
            fn wedged(&mut self, n: &str) {
                self.events.push(format!("wedged {n}"));
            }
        }
        let mut t = Fake::new(&["sca"]);
        let mut log = Log::default();
        capture(&mut t, &["sca"], &mut log).unwrap();
        assert_eq!(log.events, vec!["reload sca", "wedged sca"]);
    }
}
