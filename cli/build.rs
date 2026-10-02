//! What this build of the `embsim` command is made of, recorded for every
//! run and for `--version` to print (`src/provenance.rs`, `DESIGN.md` rule
//! 9): the embsim sources' git revision, the compiler, the target and the
//! profile. It runs in whatever binary links `embsim-cli` — the tool, a
//! runner the tool writes, a project's own runner — so it records that
//! binary's build.
//!
//! - `EMBSIM_GIT_REV`: the commit the embsim sources are, from the
//!   `.cargo_vcs_info.json` a published package carries, else from `git`
//!   in the checkout (a path checkout, a submodule, or the checkout Cargo
//!   keeps for a git dependency), `+changes` when the embsim crates have
//!   changes no commit holds; empty when neither says.
//! - `EMBSIM_SOURCE_DIR`: the embsim workspace this crate was compiled in.
//! - `EMBSIM_RUSTC`: `rustc -vV`'s release line, host and LLVM.
//! - `EMBSIM_TARGET`, `EMBSIM_PROFILE`: the target triple, and the Cargo
//!   profile with its optimisation level.

use std::path::{Path, PathBuf};
use std::process::Command;

/// The embsim crates this crate compiles in, by directory under the
/// workspace root: a change in any is a change to what the build is made
/// of.
const CRATES: [&str; 6] = ["board", "boards", "core", "models", "p2-qemu", "cli"];

fn main() {
    let manifest = PathBuf::from(std::env::var_os("CARGO_MANIFEST_DIR").unwrap_or_default());
    let root = manifest
        .parent()
        .map_or_else(|| manifest.clone(), Path::to_path_buf);
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rustc-env=EMBSIM_SOURCE_DIR={}", root.display());
    println!(
        "cargo:rustc-env=EMBSIM_GIT_REV={}",
        git_rev(&manifest, &root)
    );
    println!("cargo:rustc-env=EMBSIM_RUSTC={}", rustc());
    println!(
        "cargo:rustc-env=EMBSIM_TARGET={}",
        std::env::var("TARGET").unwrap_or_default()
    );
    println!("cargo:rustc-env=EMBSIM_PROFILE={}", profile());
}

/// The commit the sources are, as described above.
fn git_rev(manifest: &Path, root: &Path) -> String {
    // A published package: Cargo wrote the commit it was packaged from.
    if let Ok(text) = std::fs::read_to_string(manifest.join(".cargo_vcs_info.json")) {
        if let Some(sha) = json_string(&text, "sha1") {
            return sha;
        }
    }
    let git = |args: &[&str]| -> Option<String> {
        let output = Command::new("git")
            .arg("-C")
            .arg(root)
            .args(args)
            .output()
            .ok()?;
        output
            .status
            .success()
            .then(|| String::from_utf8_lossy(&output.stdout).trim().to_string())
    };
    let Some(rev) = git(&["rev-parse", "HEAD"]).filter(|rev| !rev.is_empty()) else {
        return String::new();
    };
    // Run again when HEAD moves, the branch it names moves, or a crate's
    // sources change (a directory is scanned whole).
    for path in ["HEAD", "packed-refs"] {
        watch_git_path(&git(&["rev-parse", "--git-path", path]), root);
    }
    if let Some(branch) = git(&["symbolic-ref", "-q", "HEAD"]) {
        watch_git_path(&git(&["rev-parse", "--git-path", &branch]), root);
    }
    for dir in CRATES {
        let path = root.join(dir);
        if path.exists() {
            println!("cargo:rerun-if-changed={}", path.display());
        }
    }
    let mut status = vec!["status", "--porcelain", "--untracked-files=no", "--"];
    status.extend(CRATES);
    let changed = git(&status).is_some_and(|text| !text.is_empty());
    if changed {
        format!("{rev}+changes")
    } else {
        rev
    }
}

/// Watch the file `path` names, relative to `root` when it is relative, if
/// it is there.
fn watch_git_path(path: &Option<String>, root: &Path) {
    let Some(path) = path else { return };
    let path = Path::new(path);
    let path = if path.is_absolute() {
        path.to_path_buf()
    } else {
        root.join(path)
    };
    if path.exists() {
        println!("cargo:rerun-if-changed={}", path.display());
    }
}

/// The string value of `"key": "…"` in a small JSON text.
fn json_string(text: &str, key: &str) -> Option<String> {
    let (_, rest) = text.split_once(&format!("\"{key}\""))?;
    let (_, rest) = rest.split_once('"')?;
    let (value, _) = rest.split_once('"')?;
    Some(value.to_string())
}

/// `rustc 1.96.1 (31fca3adb 2026-06-26), host aarch64-apple-darwin, LLVM
/// 20.1.8`, from `$RUSTC -vV`.
fn rustc() -> String {
    let rustc = std::env::var_os("RUSTC").unwrap_or_else(|| "rustc".into());
    let Ok(output) = Command::new(rustc).arg("-vV").output() else {
        return String::new();
    };
    let text = String::from_utf8_lossy(&output.stdout);
    let mut lines = text.lines();
    let mut said = vec![lines.next().unwrap_or_default().to_string()];
    for line in lines {
        if let Some(host) = line.strip_prefix("host: ") {
            said.push(format!("host {host}"));
        } else if let Some(llvm) = line.strip_prefix("LLVM version: ") {
            said.push(format!("LLVM {llvm}"));
        }
    }
    said.join(", ")
}

/// `release (opt-level 3)`: the Cargo profile this crate is built in, the
/// directory its `OUT_DIR` sits under (`<target>/[<triple>/]<profile>/build
/// /<package>/out`), `dev` for `debug`.
fn profile() -> String {
    let out = PathBuf::from(std::env::var_os("OUT_DIR").unwrap_or_default());
    let name = out
        .ancestors()
        .nth(3)
        .and_then(Path::file_name)
        .map(|name| name.to_string_lossy().into_owned())
        .unwrap_or_default();
    let name = if name == "debug" {
        "dev".to_string()
    } else {
        name
    };
    let opt = std::env::var("OPT_LEVEL").unwrap_or_default();
    format!("{name} (opt-level {opt})")
}
