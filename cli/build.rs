//! Link QEMU into the `embsim` binary and this crate's tests when
//! `embsim-p2-qemu` linked it, and set `cfg(qemu_linked)` for the tests that
//! boot the P2. When it is linked, record the QEMU tree it came from
//! (`EMBSIM_QEMU_TREE`): a runner the tool builds for a project's catalog
//! crates links the same one (`src/runner.rs`).
//!
//! `embsim-p2-qemu` links QEMU by passing its objects as link arguments,
//! which reach that package's own binaries only; it hands the same
//! arguments to a direct dependent through its `links = "qemu-p2"`
//! metadata, a file of one argument a line. Without QEMU it hands nothing,
//! and neither is set.

use std::env;
use std::fs;

fn main() {
    println!("cargo::rustc-check-cfg=cfg(qemu_linked)");
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-env-changed=DEP_QEMU_P2_LINKED");
    println!("cargo:rerun-if-env-changed=DEP_QEMU_P2_LINK_ARGS_FILE");
    println!("cargo:rerun-if-env-changed=EMBSIM_QEMU_P2_BUILD");
    if env::var_os("DEP_QEMU_P2_LINKED").is_none() {
        return;
    }
    if let Some(tree) = env::var_os("EMBSIM_QEMU_P2_BUILD") {
        println!(
            "cargo:rustc-env=EMBSIM_QEMU_TREE={}",
            tree.to_string_lossy()
        );
    }
    let file = env::var_os("DEP_QEMU_P2_LINK_ARGS_FILE")
        .expect("embsim-p2-qemu says QEMU is linked and names its link arguments");
    println!("cargo:rerun-if-changed={}", file.to_string_lossy());
    let args = fs::read_to_string(&file)
        .unwrap_or_else(|e| panic!("cannot read {}: {e}", file.to_string_lossy()));
    // The binary, and the tests: every test here links embsim-p2-qemu,
    // which defines the one symbol QEMU's objects take from their host
    // (`qemu_main`, `embsim-p2-qemu/src/ffi.rs`).
    for arg in args.lines().filter(|arg| !arg.is_empty()) {
        println!("cargo:rustc-link-arg-bins={arg}");
        println!("cargo:rustc-link-arg-tests={arg}");
    }
    println!("cargo:rustc-cfg=qemu_linked");
}
