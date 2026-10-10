//! The `embsim` command as a library, over a set a project's own catalog
//! joins: a board kind, a part kind, a P2 core and a bench component, all
//! defined here, and a project that names the four, checked and run through
//! [`embsim_cli::run`] in this process.
//!
//! The catalog's kinds:
//!
//! - `lib-test-strip`, a board kind: a bundled netlist of a header drawn
//!   with a symbol of its own library (the board brings the entry that
//!   makes it a connector) and a lamp, `U1`;
//! - `lib-test-lamp`, a part kind for the lamp: an analog reader across its
//!   two pins that records what it is handed;
//! - `lib-test-core`, a P2 core: it records when it attached and the
//!   virtual instant it was started at;
//! - `lib-test-source`, a bench component: one pin, driving the volts its
//!   option names behind 100 Ω, and a report saying so.
//!
//! The run is stepped inside the library (`TESTING.md` rule 9), so it ends
//! at exactly the virtual instant asked for. Its own binary: the virtual
//! clock is one per process.

use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::sync::{Arc, Mutex};

use embsim_board::{
    netlist, Assignment, AttachError, BoardSpec, Catalog, CatalogBoard, Component, ComponentNetIo,
    ComponentRequest, KindGuide, KindInfo, ModelFacade, ModelSpec, Named, PartOptions,
    PartRegistry, PinDecl, ProjectError, Report, TheveninDrive,
};
use embsim_boards::catalog::CatalogSet;
use embsim_boards::p2::{CoreCatalog, CoreCtor, P2Core, P2Pads};
use embsim_core::virtual_clock;
use rstest::rstest;
use vibes_behaviour::{behaviour, expect, Test};

/// The strip: a header `J1` in a symbol of the strip's own library, and
/// the lamp `U1` across its two pins.
const STRIP: &str = r#"(export (version "E")
  (components
    (comp (ref "J1") (value "Strip")
      (libsource (lib "Strip") (part "Strip_Header")))
    (comp (ref "U1") (value "LIBTEST-LAMP")
      (libsource (lib "Strip") (part "Lamp"))))
  (nets
    (net (code "1") (name "SIG")
      (node (ref "J1") (pin "1"))
      (node (ref "U1") (pin "1")))
    (net (code "2") (name "GND")
      (node (ref "J1") (pin "2"))
      (node (ref "U1") (pin "2")))))"#;

/// The lamp's pins: what it reads across, and its return.
const LAMP_PINS: [PinDecl; 2] = [
    PinDecl::analog("1").with_reference("2"),
    PinDecl::power_in("2"),
];

/// What the test's kinds saw: the lamp's readings, whether the core
/// attached, and the instant it started.
#[derive(Default)]
struct Seen {
    lamp: Vec<Option<f64>>,
    core_attached: bool,
    core_started_ns: Option<u64>,
}

type Shared = Arc<Mutex<Seen>>;

/// The lamp: an analog reader across its pins.
struct Lamp {
    seen: Shared,
}

impl Component for Lamp {
    fn pins(&self) -> &[PinDecl] {
        &LAMP_PINS
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let seen = Arc::clone(&self.seen);
        io.on_sense("1", move |sense| {
            seen.lock().unwrap().lamp.push(sense.volts);
        })
    }
}

/// The core: it records when it attached and when it started.
struct TestCore {
    seen: Shared,
}

impl P2Core for TestCore {
    fn attach(&mut self, _pads: P2Pads) -> Result<(), AttachError> {
        self.seen.lock().unwrap().core_attached = true;
        Ok(())
    }

    fn start(&mut self) {
        self.seen.lock().unwrap().core_started_ns = Some(virtual_clock::virtual_ns());
    }
}

/// The bench source: one pin at its volts behind 100 Ω, from attach.
struct Source {
    pins: [PinDecl; 1],
}

/// The source's impedance, the test's own.
const SOURCE_OHMS: f64 = 100.0;

/// What the source says: what it drives, at the first look.
struct SourceReport {
    subject: String,
    volts: f64,
    said: bool,
}

impl Report for SourceReport {
    fn subject(&self) -> String {
        self.subject.clone()
    }

    fn look(&mut self, _now_ns: u64) -> Vec<String> {
        if std::mem::replace(&mut self.said, true) {
            return Vec::new();
        }
        vec![format!(
            "driving OUT at {} V behind {SOURCE_OHMS} Ω",
            self.volts
        )]
    }

    fn summary(&self) -> Vec<String> {
        vec![format!("drove OUT at {} V", self.volts)]
    }
}

impl Component for Source {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, _io: ComponentNetIo) -> Result<(), AttachError> {
        Ok(())
    }
}

/// The test's catalog: a board kind, a part kind and a component kind.
struct LibCatalog {
    seen: Shared,
}

impl Catalog for LibCatalog {
    fn name(&self) -> &str {
        "lib-test-catalog"
    }

    fn board_kinds(&self) -> Vec<KindInfo> {
        vec![KindInfo::new(
            "lib-test-strip",
            "a strip whose header is a connector",
        )]
    }

    fn board(&self, spec: &BoardSpec) -> Result<CatalogBoard, ProjectError> {
        let netlist = netlist::parse(STRIP)
            .map_err(|err| ProjectError::message(format!("board {}: {err}", spec.name)))?;
        Ok(CatalogBoard::from_base(netlist)
            .with_model(ModelSpec::by_part("Strip_Header", "boundary")))
    }

    fn part_kinds(&self) -> Vec<KindGuide> {
        vec![KindGuide::new(
            "lib-test-lamp",
            "a lamp that reads the voltage across it",
            Named::family(["LIBTEST-LAMP"]),
        )]
    }

    fn register_part(
        &self,
        registry: &mut PartRegistry,
        assignment: &Assignment<'_>,
        options: PartOptions,
    ) -> Result<(), ProjectError> {
        options.finish()?;
        let seen = Arc::clone(&self.seen);
        registry.register_model(
            assignment.key,
            ModelFacade::of("lib-test-lamp", &LAMP_PINS),
            move |_| {
                Box::new(Lamp {
                    seen: Arc::clone(&seen),
                })
            },
        );
        Ok(())
    }

    fn component_kinds(&self) -> Vec<KindInfo> {
        vec![
            KindInfo::new("lib-test-source", "one pin driving a voltage").requires(
                "volts",
                "3.3",
                "what the source drives",
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
        let volts = options
            .number("volts")?
            .ok_or_else(|| options.error("options.volts is what the source drives"))?;
        options.finish()?;
        reports.add(SourceReport {
            subject: spec.name.clone(),
            volts,
            said: false,
        });
        Ok(Box::new(Source {
            pins: [PinDecl::analog_source("OUT").with_idle(Some(TheveninDrive {
                volts,
                impedance: SOURCE_OHMS,
            }))],
        }))
    }
}

/// The test's core catalog: one core kind.
struct LibCores {
    seen: Shared,
}

impl CoreCatalog for LibCores {
    fn name(&self) -> &str {
        "lib-test-catalog"
    }

    fn core_kinds(&self) -> Vec<KindInfo> {
        vec![KindInfo::new(
            "lib-test-core",
            "a core that records when it starts",
        )]
    }

    fn seat(
        &self,
        _core: &str,
        _assignment: &Assignment<'_>,
        options: PartOptions,
    ) -> Result<CoreCtor, ProjectError> {
        options.finish()?;
        let seen = Arc::clone(&self.seen);
        Ok(Box::new(move |_| {
            Ok(Box::new(TestCore {
                seen: Arc::clone(&seen),
            }) as Box<dyn P2Core>)
        }))
    }
}

/// The project naming all four kinds, the module powered from its carrier
/// fingers as `boards/projects/ec32-carrier.toml` powers it.
const PROJECT: &str = r#"
[[board]]
name = "STRIP"
kind = "lib-test-strip"

[[board.model]]
value = "LIBTEST-LAMP"
kind = "lib-test-lamp"

[[board]]
name = "EC32"
kind = "p2-ec32mb"

[[board.model]]
value = "P2X8C4M64P"
kind = "p2"
[board.model.options]
core = "lib-test-core"

[[component]]
name = "SRC"
kind = "lib-test-source"
[component.options]
volts = 3.3

[[wire]]
from = "SRC.OUT"
to = "STRIP.J1.1"

[[wire]]
from = "BENCH.GND"
to = "STRIP.J1.2"
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

[[wire]]
from = "CARRIER.GNDb"
to = "EC32.J203.44"
volts = 0.0

[[wire]]
from = "CARRIER.GNDc"
to = "EC32.J203.45"
volts = 0.0
"#;

/// The project file, in a directory of this test's own.
fn project_file() -> PathBuf {
    let dir = Path::new(env!("CARGO_TARGET_TMPDIR")).join("library");
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).expect("the scratch directory can be made");
    let path = dir.join("lib.toml");
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
fn a_project_naming_a_catalogs_four_kinds_checks_and_runs_through_the_library() {
    behaviour!(Test {
        id: "cli.library-runs-a-projects-own-kinds",
        covers: Some("cli/src/lib.rs#run"),
        given: "a project naming a board, a part model, a processor core and a bench \
                component that a test's own catalog adds, checked and run for 10 milliseconds \
                through the command's library",
    });
    expect!(
        "checks",
        "check builds the project and lists the catalog beside the two embsim ships",
        "a binary of the project's own is the command over a set its catalogs joined"
    );
    expect!(
        "core-at-start",
        "in a P2-EC32MB powered from its fingers, the added core is attached and starts at \
         5.5 milliseconds, when the processor leaves reset",
        "the processor kind holds every core behind the same reset and restart delay, \
         whichever catalog provides it"
    );
    expect!(
        "part-reads-component",
        "the added part on the added board reads the 3.3 volts the added bench component \
         drives onto the board's connector"
    );
    expect!(
        "reports",
        "the run prints what the bench component and the processor report, each under its \
         name"
    );
    let seen: Shared = Arc::default();
    let mut set = embsim_cli::shipped();
    set.add(LibCatalog {
        seen: Arc::clone(&seen),
    })
    .expect("the catalog joins");
    set.add_cores(LibCores {
        seen: Arc::clone(&seen),
    })
    .expect("the core joins");
    let project = project_file();
    let project = project.to_str().expect("the path is text");

    let (code, out, err) = embsim(&set, &["check", project]);
    assert_eq!(code, ExitCode::SUCCESS, "{err}\n{out}");
    assert!(
        out.contains("catalogs: embsim-boards, embsim-p2-qemu, embsim-cdp, lib-test-catalog"),
        "{out}"
    );
    assert!(out.contains("ok:"), "{out}");
    assert_eq!(
        seen.lock().unwrap().core_started_ns,
        None,
        "check runs nothing"
    );

    let (code, out, err) = embsim(
        &set,
        &["run", project, "--for", "10ms", "--net", "STRIP.SIG"],
    );
    assert_eq!(code, ExitCode::SUCCESS, "{err}\n{out}");
    {
        let seen = seen.lock().unwrap();
        assert!(seen.core_attached);
        assert_eq!(seen.core_started_ns, Some(5_500_000), "{out}");
        let last = seen.lamp.last().copied().flatten().expect("the lamp read");
        assert!((last - 3.3).abs() < 1e-9, "the lamp read {last} V");
    }
    for line in [
        "0.000000 ms] SRC: driving OUT at 3.3 V behind 100 Ω",
        "EC32.U100: the core started at 5.500000 ms",
        "EC32.U100: core \"lib-test-core\": started at 5.500000 ms",
        "SRC: drove OUT at 3.3 V",
        "net STRIP.SIG: Analog(3.3)",
        "ran 10.000000 ms of virtual time",
    ] {
        assert!(out.contains(line), "{line:?} missing from:\n{out}");
    }
}
