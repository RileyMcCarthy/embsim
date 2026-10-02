//! What a binary of the command is made of, for every `check` and `run` to
//! print under its project line and for `--version` (`DESIGN.md` rule 9:
//! a run says what made it).
//!
//! Two kinds of fact. What the binary was compiled from — embsim's
//! version, its git revision and where its sources were, the compiler, the
//! target and the profile — `build.rs` records into the binary itself, so
//! any binary over this crate says it, however it is started. What else it
//! holds — each catalog crate's version, directory and git revision, and a
//! project's own runner crate — the `embsim` tool measures right after it
//! builds a runner, and hands to the runner it starts in
//! [`PROVENANCE_ENV`]; a runner started by hand names its crates and says
//! where their revisions come from. A QEMU core says which program it runs
//! in its own report, at the run's first look.

use std::path::Path;
use std::process::Command;

use crate::CatalogCrate;

/// The environment variable the `embsim` tool hands a runner the facts it
/// measured in, one line each.
pub const PROVENANCE_ENV: &str = "EMBSIM_PROVENANCE";

/// This build's embsim version (`Cargo.toml`).
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// The git revision of the embsim sources this binary was compiled from,
/// `+changes` when they had changes no commit holds; empty when unknown.
pub const GIT_REV: &str = env!("EMBSIM_GIT_REV");

/// Where the embsim sources were when this binary was compiled.
pub const SOURCE_DIR: &str = env!("EMBSIM_SOURCE_DIR");

/// The compiler: `rustc -vV`'s release line, host and LLVM.
pub const RUSTC: &str = env!("EMBSIM_RUSTC");

/// The target triple.
pub const TARGET: &str = env!("EMBSIM_TARGET");

/// The Cargo profile and its optimisation level.
pub const PROFILE: &str = env!("EMBSIM_PROFILE");

/// `0.2.0 (rev 0123456789ab)`: the short version.
pub fn short_version() -> String {
    match short_rev(GIT_REV) {
        Some(rev) => format!("{VERSION} (rev {rev})"),
        None => VERSION.to_string(),
    }
}

/// The first twelve hex digits of `rev`, and its `+changes`.
fn short_rev(rev: &str) -> Option<String> {
    if rev.is_empty() {
        return None;
    }
    let (sha, changes) = match rev.split_once('+') {
        Some((sha, more)) => (sha, format!("+{more}")),
        None => (rev, String::new()),
    };
    Some(format!("{}{changes}", &sha[..sha.len().min(12)]))
}

/// What a binary over `crates` is made of, one fact a line: embsim, the
/// build, and each catalog crate.
pub fn lines(crates: &[CatalogCrate]) -> Vec<String> {
    let rev = match short_rev(GIT_REV) {
        Some(rev) => format!("git rev {rev}"),
        None => "git revision unknown".to_string(),
    };
    let mut lines = vec![
        format!("embsim {VERSION}, {rev}, from {SOURCE_DIR}"),
        format!("built by {RUSTC}, for {TARGET}, profile {PROFILE}"),
    ];
    let measured: Vec<String> = std::env::var(PROVENANCE_ENV)
        .map(|text| text.lines().map(str::to_string).collect())
        .unwrap_or_default();
    if !measured.is_empty() {
        lines.extend(measured);
    } else if !crates.is_empty() {
        let names: Vec<&str> = crates.iter().map(|catalog| catalog.name).collect();
        lines.push(format!(
            "catalog crates {}: their revisions are measured when the `embsim` tool builds and \
             starts this runner",
            names.join(", ")
        ));
    }
    lines
}

/// `--version`'s long text: every line of [`lines`], the first without
/// the `embsim ` clap puts before it.
pub fn long_version(crates: &[CatalogCrate]) -> String {
    let text = lines(crates).join("\n");
    text.strip_prefix("embsim ")
        .map_or_else(|| text.clone(), str::to_string)
}

/// `git rev 0123456789ab` for the directory `dir`, `, with changes no
/// commit holds` when the files under it have some, or that it is in no git
/// repository: what the tool measures of a crate it built.
pub fn git_state(dir: &Path) -> String {
    let git = |args: &[&str]| -> Option<String> {
        let output = Command::new("git")
            .arg("-C")
            .arg(dir)
            .args(args)
            .output()
            .ok()?;
        output
            .status
            .success()
            .then(|| String::from_utf8_lossy(&output.stdout).trim().to_string())
    };
    let Some(rev) = git(&["rev-parse", "HEAD"]).filter(|rev| !rev.is_empty()) else {
        return "in no git repository".to_string();
    };
    let rev = &rev[..rev.len().min(12)];
    // Files of the directory no commit holds: none tracked at all, or
    // changed, added or left untracked beside the tracked ones.
    let tracked = git(&["ls-files", "--", "."]).is_some_and(|text| !text.is_empty());
    if !tracked {
        return format!("in the git repository at rev {rev}, but no commit holds its files");
    }
    let changed = git(&["status", "--porcelain", "--", "."]).is_some_and(|text| !text.is_empty());
    if changed {
        format!("git rev {rev}, with changes no commit holds")
    } else {
        format!("git rev {rev}")
    }
}

#[cfg(test)]
mod tests {
    use rstest::rstest;

    use super::*;

    #[rstest]
    #[case::plain("0123456789abcdef0123", Some("0123456789ab"))]
    #[case::changed("0123456789abcdef+changes", Some("0123456789ab+changes"))]
    #[case::short("abc", Some("abc"))]
    #[case::unknown("", None)]
    fn a_revision_is_shown_by_its_first_twelve_digits(
        #[case] rev: &str,
        #[case] shown: Option<&str>,
    ) {
        assert_eq!(short_rev(rev).as_deref(), shown);
    }

    /// `git` in `dir` with `args`, a commit needing no configuration.
    fn git(dir: &Path, args: &[&str]) -> String {
        let output = Command::new("git")
            .arg("-C")
            .arg(dir)
            .args([
                "-c",
                "user.name=embsim",
                "-c",
                "user.email=embsim@example.invalid",
                "-c",
                "commit.gpgsign=false",
            ])
            .args(args)
            .output()
            .expect("git runs");
        assert!(output.status.success(), "git {args:?}: {output:?}");
        String::from_utf8_lossy(&output.stdout).trim().to_string()
    }

    #[rstest]
    fn a_directorys_state_names_its_revision_and_what_no_commit_holds() {
        let root = std::env::temp_dir().join(format!("embsim-git-state-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&root);
        let repo = root.join("repo");
        std::fs::create_dir_all(repo.join("crate")).expect("writable");
        std::fs::write(repo.join("crate/lib.rs"), "// one\n").expect("writable");
        git(&repo, &["init", "-q"]);
        git(&repo, &["add", "."]);
        git(&repo, &["commit", "-qm", "one"]);
        let rev = git(&repo, &["rev-parse", "HEAD"]);
        let short = &rev[..12];
        assert_eq!(git_state(&repo.join("crate")), format!("git rev {short}"));
        std::fs::write(repo.join("crate/lib.rs"), "// two\n").expect("writable");
        assert_eq!(
            git_state(&repo.join("crate")),
            format!("git rev {short}, with changes no commit holds")
        );
        std::fs::create_dir_all(repo.join("new")).expect("writable");
        std::fs::write(repo.join("new/lib.rs"), "").expect("writable");
        assert_eq!(
            git_state(&repo.join("new")),
            format!("in the git repository at rev {short}, but no commit holds its files")
        );
        let outside = root.join("outside");
        std::fs::create_dir_all(&outside).expect("writable");
        assert_eq!(git_state(&outside), "in no git repository");
        let _ = std::fs::remove_dir_all(&root);
    }
}
