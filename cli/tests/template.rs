//! The catalog crate `embsim new --catalog` starts, run in process: its
//! four example kinds — a board, a part, a P2 core and a bench component —
//! named by one project, checked and run through the command over a set
//! the crate's registration function joined, as the runner the `embsim`
//! tool builds would run it.
//!
//! The crate is `cli/catalog-template` (package `yourproject-catalog`);
//! `embsim new --catalog DIR` copies its library with the project's name in
//! place of `yourproject`, so what holds here holds for every crate the
//! command starts. The run is stepped inside the command (`TESTING.md`
//! rule 9). Its own binary: the virtual clock is one per process.

use std::path::{Path, PathBuf};
use std::process::ExitCode;

use embsim_cli::CatalogCrate;
use rstest::rstest;
use vibes_behaviour::{behaviour, expect, Test};

/// The project naming the template's four kinds, the module powered from
/// its carrier fingers.
const PROJECT: &str = r#"
[[board]]
name = "BRD"
kind = "yourproject-board"

[[board.model]]
value = "YOURPROJECT-SENSOR"
kind = "yourproject-sensor"

[[board]]
name = "EC32"
kind = "p2-ec32mb"

[[board.model]]
value = "P2X8C4M64P"
kind = "p2"
[board.model.options]
core = "yourproject-core"
pin = 0

[[component]]
name = "SRC"
kind = "yourproject-source"
[component.options]
volts = 3.3
ohms = 100.0
at = "1ms"

[[wire]]
from = "SRC.OUT"
to = "BRD.J1.1"

[[wire]]
from = "BENCH.GND"
to = "BRD.J1.2"
volts = 0.0

[[wire]]
from = "CARRIER.5V"
to = "EC32.J203.41"
volts = 5.0

[[wire]]
from = "CARRIER.5Vb"
to = "EC32.J203.42"
volts = 5.0

[[wire]]
from = "CARRIER.GND"
to = "EC32.J203.43"
volts = 0.0
"#;

/// The template crate's directory.
const TEMPLATE_DIR: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/catalog-template");

/// The project file, naming the template crate in its `[catalog]` as a
/// project of its own would, in a directory of this test's own.
fn project_file() -> PathBuf {
    let dir = Path::new(env!("CARGO_TARGET_TMPDIR")).join("template");
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).expect("the scratch directory can be made");
    let path = dir.join("rig.toml");
    let catalog = format!("[catalog]\ncrates = [{TEMPLATE_DIR:?}]\n");
    std::fs::write(&path, format!("{catalog}{PROJECT}")).expect("the project is writable");
    path
}

/// The command as the template crate's runner, with `args`: its exit
/// status, what it printed and its errors.
fn runner(args: &[&str]) -> (ExitCode, String, String) {
    let crates = [CatalogCrate::new(
        "yourproject-catalog",
        TEMPLATE_DIR,
        yourproject_catalog::register,
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

#[rstest]
fn the_started_catalogs_four_kinds_check_and_run_in_one_project() {
    behaviour!(Test {
        id: "cli.template-kinds-run",
        covers: Some("cli/catalog-template/src/lib.rs#register"),
        given: "a project naming the four example kinds of the catalog crate `embsim new \
                --catalog` starts — its board with its sensor part, its processor core in a \
                P2-EC32MB powered from its fingers, and its bench source wired to the board's \
                connector at 3.3 volts behind 100 ohms — checked and run for 10 milliseconds \
                as the crate's runner runs it",
    });
    expect!(
        "checks",
        "check builds the project, listing the started crate's catalog beside the two embsim \
         ships"
    );
    expect!(
        "sensor-reads-source",
        "the sensor on the board reads the 3.3 volts the bench source drives onto the \
         connector"
    );
    expect!(
        "source-acts-at-its-instant",
        "the bench source drives from the instant its options name, a millisecond after the \
         system starts, and the sensor is first handed its volts at that instant",
        "a bench component acts over time on a wake it arms when the system starts"
    );
    expect!(
        "core-drives-its-pad",
        "the core drives the pad its options name high from the processor's start, and the \
         pad's net reads driven high at the end",
        "a core drives its pads through the package, at its bank's supply"
    );
    let project = project_file();
    let project = project.to_str().expect("the path is text");

    let (code, out, err) = runner(&["check", project]);
    assert_eq!(code, ExitCode::SUCCESS, "{err}\n{out}");
    assert!(
        out.contains("catalogs: embsim-boards, embsim-p2-qemu, yourproject-catalog"),
        "{out}"
    );

    let (code, out, err) = runner(&["run", project, "--for", "10ms", "--net", "EC32.P2_IO0"]);
    assert_eq!(code, ExitCode::SUCCESS, "{err}\n{out}");
    for line in [
        "BRD.U1: pin 1 read 3.3 V from 1.000000 ms",
        "EC32.U100: core \"yourproject-core\": started at 5.500000 ms",
        "EC32.U100: drove P0 high",
        "SRC: drove OUT at 3.3 V behind 100 Ω from 1.000000 ms",
        "net EC32.P2_IO0: Driven(High)",
    ] {
        assert!(out.contains(line), "{line:?} missing from:\n{out}");
    }
}
