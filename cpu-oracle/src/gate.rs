//! A conformance gate that can go green while the debt is still there.
//!
//! The obvious gate — "the simulator must match silicon on every case" — is
//! the one nobody can commit. A first capture against a young ISS finds
//! hundreds of divergences, the test cannot pass, and so it lives in someone's
//! working tree instead of in CI, where it protects nothing. (Observed: a P2
//! corpus sat uncommitted at 544 divergent cases for exactly this reason.)
//!
//! So the gate takes a **baseline** of mnemonics known to diverge, and fails in
//! two directions instead of one:
//!
//! * a mnemonic diverges that is **not** in the baseline — a regression, or a
//!   newly captured instruction nobody has looked at;
//! * a mnemonic is in the baseline but **no longer diverges** — a stale entry,
//!   which matters because a baseline that outlives its bug silently stops the
//!   gate noticing when the bug comes back.
//!
//! The second direction is what keeps the list honest, and it doubles as a
//! to-do list that tells you when you have finished.

use std::collections::BTreeSet;

/// Mnemonics accepted as diverging, parsed from a baseline file.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Baseline {
    entries: BTreeSet<String>,
}

impl Baseline {
    /// Parse a baseline: one mnemonic per line, `#` starts a comment.
    ///
    /// Comments are the point of the format rather than a decoration — the
    /// useful baseline says *why* each entry is there, and in particular which
    /// entries are the corpus's fault rather than the simulator's.
    pub fn parse(text: &str) -> Self {
        let entries = text
            .lines()
            .map(|l| l.split('#').next().unwrap_or("").trim())
            .filter(|l| !l.is_empty())
            .map(str::to_string)
            .collect();
        Self { entries }
    }

    pub fn contains(&self, mnemonic: &str) -> bool {
        self.entries.contains(mnemonic)
    }

    pub fn len(&self) -> usize {
        self.entries.len()
    }

    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    pub fn iter(&self) -> impl Iterator<Item = &str> {
        self.entries.iter().map(String::as_str)
    }
}

/// What the gate found.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Verdict {
    /// Diverging and not baselined: a regression or an unexamined instruction.
    pub regressed: Vec<String>,
    /// Baselined but no longer diverging: the entry should be deleted.
    pub stale: Vec<String>,
    /// Every mnemonic currently diverging.
    pub divergent: Vec<String>,
}

impl Verdict {
    /// Whether the gate passes. Divergence alone does not fail it; unexpected
    /// divergence and stale entries do.
    pub fn is_green(&self) -> bool {
        self.regressed.is_empty() && self.stale.is_empty()
    }

    /// A message for a failing gate, or `None` when it passes.
    ///
    /// Says what to do, not merely what happened: the failure a developer sees
    /// most often is the good one — an instruction they just fixed — and the
    /// gate should tell them to delete the line rather than leave them
    /// wondering what broke.
    pub fn failure(&self, baseline_path: &str) -> Option<String> {
        if self.is_green() {
            return None;
        }
        let mut msg = String::new();
        if !self.regressed.is_empty() {
            msg.push_str(&format!(
                "{} mnemonic(s) diverge from silicon and are NOT in the baseline: {:?}\n\
                 That is either a regression or a newly captured instruction. If it is \
                 deliberate, add it to {baseline_path} with a reason.\n",
                self.regressed.len(),
                self.regressed
            ));
        }
        if !self.stale.is_empty() {
            msg.push_str(&format!(
                "good news, and the baseline is now stale: {:?} match silicon but are \
                 still listed in {baseline_path}. Delete those lines — a baseline that \
                 outlives its bug stops the gate noticing when the bug returns.\n",
                self.stale
            ));
        }
        Some(msg)
    }
}

/// Compare the mnemonics that diverge against the baseline.
pub fn evaluate<'a>(divergent: impl IntoIterator<Item = &'a str>, baseline: &Baseline) -> Verdict {
    let divergent: BTreeSet<&str> = divergent.into_iter().collect();
    Verdict {
        regressed: divergent
            .iter()
            .filter(|m| !baseline.contains(m))
            .map(|m| m.to_string())
            .collect(),
        stale: baseline
            .iter()
            .filter(|m| !divergent.contains(m))
            .map(str::to_string)
            .collect(),
        divergent: divergent.iter().map(|m| m.to_string()).collect(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn baseline() -> Baseline {
        Baseline::parse(
            "# known divergent\n\
             mul   # the multiplier is only partly modelled\n\
             \n\
             rdlong\n",
        )
    }

    #[test]
    fn comments_and_blank_lines_are_not_entries() {
        let b = baseline();
        assert_eq!(b.len(), 2);
        assert!(b.contains("mul"));
        assert!(b.contains("rdlong"));
        assert!(!b.contains("# known divergent"));
    }

    #[test]
    fn divergence_that_is_baselined_is_green() {
        let v = evaluate(["mul", "rdlong"], &baseline());
        assert!(v.is_green());
        assert_eq!(v.divergent.len(), 2);
        assert!(v.failure("baseline.txt").is_none());
    }

    #[test]
    fn a_new_divergence_fails_and_names_itself() {
        let v = evaluate(["mul", "rdlong", "sal"], &baseline());
        assert!(!v.is_green());
        assert_eq!(v.regressed, vec!["sal"]);
        let msg = v.failure("baseline.txt").expect("a failure message");
        assert!(msg.contains("sal"), "the message must name the regression");
        assert!(msg.contains("baseline.txt"));
    }

    #[test]
    fn a_fixed_instruction_fails_the_gate_until_its_line_is_deleted() {
        // The direction that keeps the list honest: `mul` now matches, so the
        // baseline must shrink or it stops protecting `mul`.
        let v = evaluate(["rdlong"], &baseline());
        assert!(!v.is_green());
        assert_eq!(v.stale, vec!["mul"]);
        assert!(v.failure("baseline.txt").unwrap().contains("stale"));
    }

    #[test]
    fn an_empty_baseline_means_every_divergence_is_a_regression() {
        let v = evaluate(["mul"], &Baseline::default());
        assert_eq!(v.regressed, vec!["mul"]);
    }

    #[test]
    fn full_conformance_against_an_empty_baseline_is_green() {
        let v = evaluate(Vec::<&str>::new(), &Baseline::default());
        assert!(v.is_green());
        assert!(v.divergent.is_empty());
    }
}
