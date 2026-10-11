//! Proves a killed parent takes the process group it started.
//!
//! The fork has to happen before any other thread exists, so the proof is
//! the `parent_death_probe` binary rather than a fork inside this harness.
//! `CARGO_BIN_EXE_*` is set for integration tests only.

#![cfg(target_os = "linux")]

use std::process::Command;

use vibes_behaviour::{behaviour, expect, Test};

#[test]
fn a_killed_parent_ends_the_process_group_it_started() {
    behaviour!(Test {
        id: "chrome-cdp.parent-death",
        covers: Some("cdp/src/chrome.rs#arm_parent_death"),
        given: "a process the node started in its own group",
    });
    expect!(
        "dies-with-parent",
        "killing its parent ends that process and its group, and its profile is removed"
    );
    let status = Command::new(env!("CARGO_BIN_EXE_parent_death_probe"))
        .status()
        .expect("the parent-death probe runs");
    assert!(status.success(), "dies-with-parent: {status}");
}
