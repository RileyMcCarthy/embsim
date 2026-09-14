//! What the corpus never asked about.
//!
//! Two different holes, and only the first is obvious:
//!
//! 1. **Planned but never captured** — an instruction the sweep intended to
//!    cover has no record. Usually it wedged the target and was skipped, which
//!    is easy to miss in a log of thousands of lines.
//!
//! 2. **Captured but undiscriminated** — every record for an instruction
//!    produced the same output, so the corpus cannot tell a correct
//!    implementation from any wrong one that happens to agree. This is the
//!    dangerous hole, because it reports as *green*. A real case: a P2 `SAL`
//!    sweep whose two destination operands both had bit 0 clear could not see
//!    that the simulator filled with zero instead of with bit 0. Fifty green
//!    records, and the instruction was wrong.
//!
//! A harness that reports only (1) will keep handing out confident greens for
//! instructions nobody has actually tested.

use std::collections::{BTreeMap, BTreeSet};

use crate::Record;

/// Mnemonics that were planned but appear in no record.
pub fn never_captured<'a>(
    planned: impl IntoIterator<Item = &'a str>,
    captured: &BTreeSet<&str>,
) -> Vec<String> {
    let mut missing: Vec<String> = planned
        .into_iter()
        .filter(|m| !captured.contains(m))
        .map(str::to_string)
        .collect();
    missing.sort_unstable();
    missing.dedup();
    missing
}

/// An instruction whose records cannot distinguish a right answer from a wrong
/// one.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Undiscriminated {
    pub mnemonic: String,
    /// How many records the corpus holds for it.
    pub records: usize,
    /// Why it proves nothing.
    pub reason: &'static str,
}

/// Instructions whose every record shares one output.
///
/// `outputs` maps a record to the observable it produced; `mnemonic_of` says
/// which instruction a record belongs to. Both are supplied by the adapter,
/// because only it knows how its records are named and what counts as the
/// observable.
///
/// An instruction with a single record is reported too: one sample can be
/// matched by a great many wrong rules, and calling that "covered" is the
/// mistake this function exists to prevent.
pub fn undiscriminated<R>(
    records: &[R],
    mnemonic_of: impl Fn(&R) -> String,
    output_of: impl Fn(&R) -> String,
) -> Vec<Undiscriminated> {
    let mut by_op: BTreeMap<String, Vec<String>> = BTreeMap::new();
    for r in records {
        by_op.entry(mnemonic_of(r)).or_default().push(output_of(r));
    }
    let mut out = Vec::new();
    for (mnemonic, outputs) in by_op {
        let distinct: BTreeSet<&String> = outputs.iter().collect();
        let reason = if outputs.len() == 1 {
            "only one record — a single sample cannot separate rival rules"
        } else if distinct.len() == 1 {
            "every record produced the same output — any rule that agrees once agrees always"
        } else {
            continue;
        };
        out.push(Undiscriminated {
            mnemonic,
            records: outputs.len(),
            reason,
        });
    }
    out
}

/// What a [`Record`] produced, flattened to a string.
///
/// The default notion of "the observable" for adapters that already speak
/// [`Record`]: every `OUT` field, in key order, so two records compare equal
/// exactly when silicon reported the same thing for both.
pub fn record_output(r: &Record) -> String {
    r.output
        .iter()
        .map(|(k, v)| format!("{k}={v}"))
        .collect::<Vec<_>>()
        .join(" ")
}

#[cfg(test)]
mod tests {
    use super::*;

    struct R {
        op: &'static str,
        out: &'static str,
    }

    fn scan(rs: &[R]) -> Vec<Undiscriminated> {
        undiscriminated(rs, |r| r.op.to_string(), |r| r.out.to_string())
    }

    #[test]
    fn an_instruction_whose_records_all_agree_is_reported() {
        // The SAL shape: several records, one answer, nothing proven.
        let found = scan(&[
            R {
                op: "sal",
                out: "0",
            },
            R {
                op: "sal",
                out: "0",
            },
            R {
                op: "sal",
                out: "0",
            },
        ]);
        assert_eq!(found.len(), 1);
        assert_eq!(found[0].mnemonic, "sal");
        assert_eq!(found[0].records, 3);
        assert!(found[0].reason.contains("same output"));
    }

    #[test]
    fn a_single_record_is_reported_even_though_it_is_not_constant() {
        let found = scan(&[R {
            op: "mul",
            out: "6",
        }]);
        assert_eq!(found.len(), 1);
        assert!(found[0].reason.contains("one record"));
    }

    #[test]
    fn an_instruction_with_differing_outputs_is_not_reported() {
        let found = scan(&[
            R {
                op: "add",
                out: "5",
            },
            R {
                op: "add",
                out: "80000001",
            },
        ]);
        assert!(
            found.is_empty(),
            "differing outputs discriminate: {found:?}"
        );
    }

    #[test]
    fn each_instruction_is_judged_on_its_own_records() {
        let found = scan(&[
            R {
                op: "add",
                out: "5",
            },
            R {
                op: "add",
                out: "6",
            },
            R {
                op: "sal",
                out: "0",
            },
            R {
                op: "sal",
                out: "0",
            },
        ]);
        assert_eq!(found.len(), 1);
        assert_eq!(found[0].mnemonic, "sal");
    }

    #[test]
    fn planned_instructions_with_no_records_are_missing() {
        let captured: BTreeSet<&str> = ["add", "sub"].into_iter().collect();
        assert_eq!(
            never_captured(["add", "sub", "sal"], &captured),
            vec!["sal".to_string()]
        );
        assert!(never_captured(["add"], &captured).is_empty());
    }
}
