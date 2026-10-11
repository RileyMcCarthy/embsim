//! The example project, run the way its runner runs it: the command over
//! the catalogs embsim ships and this crate's ([`embsim_cli::run_with_crates`]),
//! in this process — how a project tests its own catalog without building
//! a runner.
//!
//! `project.toml` powers a P2-EC32MB from its fingers; the crate's core
//! toggles `P0` every half millisecond from the processor's start, `P0`
//! drives the crate's EX-BUF1 buffer on the crate's board, and the buffer's
//! output reaches the crate's edge counter. The run is stepped inside the
//! command (`TESTING.md` rule 9): it ends at exactly the instant asked for,
//! and every instant printed is the same on every machine. Its own binary:
//! the virtual clock is one per process.

use std::process::ExitCode;

use custom_project_catalog::buffer::T_PD_NS;
use embsim_cli::CatalogCrate;
use rstest::rstest;
use vibes_behaviour::{behaviour, expect, Test};

/// The project file, beside this crate.
const PROJECT: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../project.toml");

/// The P2-EC32MB's START instant from its fingers: the bucks' 2.5 ms
/// soft-start, then the datasheet's 3 ms restart delay (`cli/tests/cli.rs`,
/// `run_boots_the_p2_off_the_modules_flash`).
const START_NS: u64 = 5_500_000;

/// The blinker's period, as `project.toml` gives it.
const PERIOD_NS: u64 = 1_000_000;

/// The command as this crate's runner, with `args`.
fn runner(args: &[&str]) -> (ExitCode, String, String) {
    let crates = [CatalogCrate::new(
        custom_project_catalog::NAME,
        env!("CARGO_MANIFEST_DIR"),
        custom_project_catalog::register,
    )];
    let (mut out, mut err) = (Vec::new(), Vec::new());
    let code = embsim_cli::run_with_crates(
        &crates,
        std::iter::once("embsim").chain(args.iter().copied()),
        &mut out,
        &mut err,
    );
    (
        code,
        String::from_utf8_lossy(&out).into_owned(),
        String::from_utf8_lossy(&err).into_owned(),
    )
}

/// An instant as the run prints it.
fn ms(ns: u64) -> String {
    embsim_board::report::instant(ns)
}

#[rstest]
fn the_blinking_pad_reaches_the_counter_through_the_buffer_once_a_period() {
    behaviour!(Test {
        id: "example.custom-project-run",
        covers: Some("examples/custom-project/catalog/src/lib.rs#register"),
        given: "the example project with kinds of its own — a P2-EC32MB powered from its \
                fingers running the example core that toggles pad P0 with a 1 millisecond \
                period, P0 wired to the input of the example buffer on the example board, the \
                buffer's output wired to the example edge counter — checked, then run for 10 \
                milliseconds as its runner runs it",
    });
    expect!(
        "checks",
        "check builds the system, listing the example's catalog beside the two embsim ships"
    );
    expect!(
        "first-edge",
        "the counter sees its first rising edge the buffer's 12 nanosecond propagation delay \
         after the processor leaves reset at 5.5 milliseconds",
        "the core drives its pad high at the start, and the buffer's output follows its input \
         after the delay its datasheet gives"
    );
    expect!(
        "an-edge-a-period",
        "a rising edge follows every millisecond after the first: five in the run, the last \
         at 9.500012 milliseconds"
    );
    expect!(
        "core-flips",
        "the core flips its pad every half millisecond from its start, eight times by the \
         end, leaving it high"
    );

    let (code, out, err) = runner(&["check", PROJECT]);
    assert_eq!(code, ExitCode::SUCCESS, "{err}\n{out}");
    assert!(
        out.contains("catalogs: embsim-boards, embsim-p2-qemu, embsim-cdp, custom-project-catalog"),
        "{out}"
    );

    let (code, out, err) = runner(&["run", PROJECT, "--for", "10ms", "--net", "BUF.OUT"]);
    assert_eq!(code, ExitCode::SUCCESS, "{err}\n{out}");
    let first = START_NS + T_PD_NS;
    let edges: Vec<String> = (0..5)
        .map(|n| {
            format!(
                "COUNTER: rising edge {} on IN at {}",
                n + 1,
                ms(first + n * PERIOD_NS)
            )
        })
        .collect();
    for line in &edges {
        assert!(out.contains(line.as_str()), "{line:?} missing from:\n{out}");
    }
    assert!(!out.contains("rising edge 6"), "{out}");
    for line in [
        format!(
            "COUNTER: 5 rising edges on IN, the first at {} and the last at {}, every {}",
            ms(first),
            ms(first + 4 * PERIOD_NS),
            ms(PERIOD_NS)
        ),
        format!(
            "EC32.U100: blinker: started at {}; P0 flipped 8 times, and drives it high",
            ms(START_NS)
        ),
        "net BUF.OUT: Driven(High)".to_string(),
        "ran 10.000000 ms of virtual time".to_string(),
    ] {
        assert!(out.contains(&line), "{line:?} missing from:\n{out}");
    }
}
