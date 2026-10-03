//! A run that a part's failure ends: `embsim run` stops at the first look
//! at which a report says its part failed — a P2 core whose program died,
//! here a bench component of the test's own that fails at an instant its
//! option names — prints its summary, and exits non-zero with the reason.
//!
//! The failing part is a bench component, `fail-test-source`: one pin
//! driving 3.3 V behind 100 Ω onto the header board's connector, and a
//! report that says the part failed once a look reaches `fails-at`. The
//! command runs in this process through [`embsim_cli::run`] over the
//! shipped set and the test's catalog; its own binary, since the virtual
//! clock is one per process. The QEMU core's half — its report says
//! failed when its program dies — is `run_stops_when_its_qemu_core_dies`
//! in `cli.rs`, against an installed `qemu-system-p2`.

use std::path::{Path, PathBuf};
use std::process::ExitCode;

use embsim_board::{
    parse_duration, Catalog, Component, ComponentNetIo, ComponentRequest, KindInfo, PinDecl,
    ProjectError, Report, TheveninDrive,
};
use rstest::rstest;
use vibes_behaviour::{behaviour, expect, Test};

/// The source's impedance and voltage, the test's own.
const SOURCE_OHMS: f64 = 100.0;
const SOURCE_VOLTS: f64 = 3.3;

/// The bench component: one pin at its volts from attach.
struct Source {
    pins: [PinDecl; 1],
}

impl Component for Source {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, _io: ComponentNetIo) -> Result<(), embsim_board::AttachError> {
        Ok(())
    }
}

/// What the component reports: that it failed, from the first look at or
/// after `fails_at_ns`.
struct FailingReport {
    subject: String,
    fails_at_ns: u64,
    failed: bool,
}

impl Report for FailingReport {
    fn subject(&self) -> String {
        self.subject.clone()
    }

    fn look(&mut self, now_ns: u64) -> Vec<String> {
        if !self.failed && now_ns >= self.fails_at_ns {
            self.failed = true;
            return vec!["the test's part failed, as asked\nits second line".to_string()];
        }
        Vec::new()
    }

    fn summary(&self) -> Vec<String> {
        vec![if self.failed { "failed" } else { "running" }.to_string()]
    }

    fn failure(&self) -> Option<String> {
        self.failed
            .then(|| "the test's part failed, as asked\nits second line".to_string())
    }
}

/// The test's catalog: the one component kind.
struct FailCatalog;

impl Catalog for FailCatalog {
    fn name(&self) -> &str {
        "fail-test-catalog"
    }

    fn component_kinds(&self) -> Vec<KindInfo> {
        vec![
            KindInfo::new("fail-test-source", "a source whose report fails").requires(
                "fails-at",
                "2ms",
                "the virtual instant its report says it failed",
            ),
        ]
    }

    fn component(&self, request: ComponentRequest<'_>) -> Result<Box<dyn Component>, ProjectError> {
        let ComponentRequest {
            spec,
            mut options,
            reports,
            ..
        } = request;
        let at = options
            .string("fails-at")?
            .ok_or_else(|| options.error("options.fails-at is when the part fails"))?;
        let fails_at_ns = parse_duration(&at).map_err(|why| options.error(why))?;
        options.finish()?;
        reports.add(FailingReport {
            subject: spec.name.clone(),
            fails_at_ns,
            failed: false,
        });
        Ok(Box::new(Source {
            pins: [PinDecl::analog_source("OUT").with_idle(Some(TheveninDrive {
                volts: SOURCE_VOLTS,
                impedance: SOURCE_OHMS,
            }))],
        }))
    }
}

/// The header board, and the failing source on its connector's first pin.
fn project_file() -> PathBuf {
    let dir = Path::new(env!("CARGO_TARGET_TMPDIR")).join("failure");
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).expect("the scratch directory can be made");
    let netlist = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../boards/projects/header.net")
        .canonicalize()
        .expect("the header netlist is there");
    let path = dir.join("fails.toml");
    std::fs::write(
        &path,
        format!(
            r#"
[[board]]
name = "HDR"
kind = "netlist"
netlist = {netlist:?}

[[component]]
name = "SRC"
kind = "fail-test-source"
[component.options]
fails-at = "2ms"

[[wire]]
from = "SRC.OUT"
to = "HDR.J1.1"

[[wire]]
from = "BENCH.GND"
to = "HDR.J1.2"
volts = 0.0
"#,
            netlist = netlist.display().to_string()
        ),
    )
    .expect("the project is writable");
    path
}

#[rstest]
fn a_part_that_fails_stops_the_run_and_the_command_exits_non_zero() {
    behaviour!(Test {
        id: "cli.run-stops-on-failure",
        covers: Some("cli/src/live.rs#run"),
        given: "a project whose bench component reports that it failed 2 milliseconds in, run \
                for 10 milliseconds",
    });
    expect!(
        "stops-there",
        "the run stops at the first look at or after the failure, 2 milliseconds in, and says \
         which part failed",
        "nothing after a part fails is the system the project describes"
    );
    expect!(
        "summary",
        "it still prints how far it ran and what each part reports at the end"
    );
    expect!(
        "exits-non-zero",
        "the command exits non-zero, and its error names the part, the instant and the first \
         line of why",
        "a script or CI job that runs a project sees the failure in the exit status"
    );
    let mut set = embsim_cli::shipped();
    set.add(FailCatalog).expect("the catalog joins");
    let project = project_file();
    let (mut out, mut err) = (Vec::new(), Vec::new());
    let code = embsim_cli::run(
        &set,
        [
            "embsim",
            "run",
            project.to_str().expect("text"),
            "--for",
            "10ms",
        ],
        &mut out,
        &mut err,
    );
    let out = String::from_utf8_lossy(&out);
    let err = String::from_utf8_lossy(&err);
    assert_eq!(code, ExitCode::FAILURE, "{err}\n{out}");
    for line in [
        "2.000000 ms] SRC: the test's part failed, as asked",
        "stopped at 2.000000 ms of virtual time: SRC failed",
        "ran 2.000000 ms of virtual time",
        "SRC: failed",
    ] {
        assert!(out.contains(line), "{line:?} missing from:\n{out}");
    }
    assert!(!out.contains("ran 10.000000 ms"), "{out}");
    assert_eq!(
        err.trim_end(),
        "error: SRC failed at 2.000000 ms of virtual time, and the run stopped there: the \
         test's part failed, as asked"
    );
}
