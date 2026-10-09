//! A project naming a bench component kind its catalog builds as an
//! [`Assembly`] (`PROJECTS.md` §10, "A plant: an `Assembly`"): the
//! command checks it and runs it, through [`embsim_cli::run`] in this
//! process.
//!
//! The catalog is the test's own, one kind: `assembly-test-axis`, embsim's
//! step/direction drive and quadrature encoder on one shaft, as one
//! assembly — the drive's inputs measured against `DRIVE_GND` and the
//! encoder's outputs against `ENC_GND`, returns the assembly declares — and
//! a report of where the encoder stands. The project steps it six times
//! with the standard catalog's `scripted-source`, holds its direction and
//! enable high and its returns at 0 V from the bench, and reads its
//! encoder's outputs with `--net`.
//!
//! The run is stepped inside the library (`TESTING.md` rule 9). Its own
//! binary: the virtual clock is one per process.

use std::path::{Path, PathBuf};
use std::process::ExitCode;

use embsim_board::{
    Assembly, Catalog, Component, ComponentRequest, KindInfo, ProjectError, Report,
};
use embsim_boards::catalog::CatalogSet;
use embsim_models::machine::quadrature_encoder::{self, EncoderInput};
use embsim_models::machine::stepper_motor;
use embsim_models::machine::{MotorShaft, QuadratureEncoder, StepperMotor};
use rstest::rstest;
use vibes_behaviour::{behaviour, expect, Test};

/// Where the axis's encoder stands, said at the end of the run.
struct AxisReport {
    subject: String,
    shaft: MotorShaft,
    encoder: EncoderInput,
}

impl Report for AxisReport {
    fn subject(&self) -> String {
        self.subject.clone()
    }

    fn look(&mut self, _now_ns: u64) -> Vec<String> {
        Vec::new()
    }

    fn summary(&self) -> Vec<String> {
        vec![format!(
            "encoder at {} counts, {} steps commanded",
            self.encoder.count(),
            self.shaft.commanded_steps()
        )]
    }
}

/// The test's catalog: one bench component kind, an assembly.
struct AxisCatalog;

impl Catalog for AxisCatalog {
    fn name(&self) -> &str {
        "assembly-test-catalog"
    }

    fn component_kinds(&self) -> Vec<KindInfo> {
        vec![KindInfo::new(
            "assembly-test-axis",
            "a step/direction drive turning a quadrature encoder, as one assembly",
        )
        .requires(
            "steps_per_mm",
            "8192.0",
            "steps of the drive, and counts of the encoder, per millimetre",
        )]
    }

    fn component(&self, request: ComponentRequest<'_>) -> Result<Box<dyn Component>, ProjectError> {
        let context = request.context();
        let ComponentRequest {
            spec,
            mut options,
            reports,
            ..
        } = request;
        let steps_per_mm = options
            .number("steps_per_mm")?
            .ok_or_else(|| options.error("options.steps_per_mm is the axis's steps per mm"))?;
        options.finish()?;
        let refuse =
            |err: &dyn std::fmt::Display| ProjectError::message(format!("{context}: {err}"));
        // The test's drive carries no load, so the carriage comes to rest
        // where the steps put it.
        let drive = StepperMotor::new(stepper_motor::Config {
            load_loss: 0.0,
            ..stepper_motor::Config::new(steps_per_mm)
        })
        .map_err(|err| refuse(&err))?;
        let encoder = QuadratureEncoder::new(quadrature_encoder::Config::new(steps_per_mm))
            .map_err(|err| refuse(&err))?;
        let (shaft, input) = (drive.shaft(), encoder.input());
        // The link between the two members: the assembly's code.
        {
            let input = input.clone();
            shaft.on_position_change(move |mm| input.set_position_mm(mm));
        }
        let axis = Assembly::new()
            .member(
                "DRIVE",
                Box::new(drive),
                &[("STEP", "STEP"), ("DIR", "DIR"), ("ENA", "ENA")],
            )
            .and_then(|axis| {
                axis.member(
                    "ENCODER",
                    Box::new(encoder),
                    &[("A", "ENC_A"), ("B", "ENC_B")],
                )
            })
            .and_then(|axis| axis.reference("DRIVE_GND", &["STEP", "DIR", "ENA"]))
            .and_then(|axis| axis.reference("ENC_GND", &["ENC_A", "ENC_B"]))
            .map_err(|err| refuse(&err))?;
        reports.add(AxisReport {
            subject: spec.name.clone(),
            shaft,
            encoder: input,
        });
        Ok(Box::new(axis))
    }
}

/// The project: the axis, six steps a millisecond apart on its `STEP`, its
/// direction and enable held high and its returns at 0 V by the bench.
const PROJECT: &str = r#"
[[component]]
name = "AXIS"
kind = "assembly-test-axis"
[component.options]
steps_per_mm = 8192.0

[[component]]
name = "STEPS"
kind = "scripted-source"
[component.options]
ohms = 25.0
steps = [
  ["0ms", 0.0],
  ["1ms", 3.3], ["1.5ms", 0.0],
  ["2ms", 3.3], ["2.5ms", 0.0],
  ["3ms", 3.3], ["3.5ms", 0.0],
  ["4ms", 3.3], ["4.5ms", 0.0],
  ["5ms", 3.3], ["5.5ms", 0.0],
  ["6ms", 3.3], ["6.5ms", 0.0],
]

[[wire]]
from = "STEPS.OUT"
to = "AXIS.STEP"

[[wire]]
from = "BENCH.DIR"
to = "AXIS.DIR"
volts = 3.3

[[wire]]
from = "BENCH.ENA"
to = "AXIS.ENA"
volts = 3.3

[[wire]]
from = "BENCH.DRIVE_GND"
to = "AXIS.DRIVE_GND"
volts = 0.0

[[wire]]
from = "BENCH.ENC_GND"
to = "AXIS.ENC_GND"
volts = 0.0
"#;

/// The project file, in a directory of this test's own.
fn project_file() -> PathBuf {
    let dir = Path::new(env!("CARGO_TARGET_TMPDIR")).join("assembly");
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).expect("the scratch directory can be made");
    let path = dir.join("axis.toml");
    std::fs::write(&path, PROJECT).expect("the project is writable");
    path
}

/// The command over `set` with `args`, in this process: its exit status,
/// what it printed and its errors.
fn embsim(set: &CatalogSet, args: &[&str]) -> (ExitCode, String, String) {
    let (mut out, mut err) = (Vec::new(), Vec::new());
    let code = embsim_cli::run(
        set,
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
fn a_project_naming_a_catalogs_assembly_kind_checks_and_runs() {
    behaviour!(Test {
        id: "cli.assembly-kind-checks-and-runs",
        covers: Some("board/src/assembly.rs#Assembly"),
        given: "a project naming a test catalog's bench component kind, a drive turning an \
                encoder built as one assembly, stepped six times a millisecond apart, checked \
                and then run for 300 milliseconds",
    });
    expect!(
        "checks",
        "check builds the project, each wire landing on an assembly pin: the drive's three \
         inputs and the two returns the assembly declares",
        "the engine sees the assembly as one bench component whose pins are its members' \
         pins, renamed, and the returns it declares"
    );
    expect!(
        "runs-to-the-steps",
        "the run ends with the encoder at the six counts the six steps commanded, its \
         outputs at that count's quadrature state, both high",
        "the drive's shaft turns the encoder inside the one component, on the engine's time"
    );
    let mut set = embsim_cli::shipped();
    set.add(AxisCatalog).expect("the catalog joins");
    let project = project_file();
    let project = project.to_str().expect("the path is text");

    let (code, out, err) = embsim(&set, &["check", project]);
    assert_eq!(code, ExitCode::SUCCESS, "{err}\n{out}");
    assert!(
        out.contains("catalogs: embsim-boards, embsim-p2-qemu, embsim-qemu, assembly-test-catalog"),
        "{out}"
    );
    assert!(out.contains("ok:"), "{out}");

    let (code, out, err) = embsim(
        &set,
        &[
            "run",
            project,
            "--for",
            "300ms",
            "--net",
            "AXIS.ENC_A",
            "--net",
            "AXIS.ENC_B",
        ],
    );
    assert_eq!(code, ExitCode::SUCCESS, "{err}\n{out}");
    for line in [
        "AXIS: encoder at 6 counts, 6 steps commanded",
        "net AXIS.ENC_A: Driven(High)",
        "net AXIS.ENC_B: Driven(High)",
        "ran 300.000000 ms of virtual time",
    ] {
        assert!(out.contains(line), "{line:?} missing from:\n{out}");
    }
}
