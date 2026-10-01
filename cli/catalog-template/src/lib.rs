//! The yourproject catalog: the kinds this project adds to embsim
//! (`PROJECTS.md` §10, "Extending embsim from a project").
//!
//! `embsim new --catalog` started this crate with one example of each sort
//! of kind a catalog can add, so a project file can name all four at once:
//!
//! | Kind | Sort | In the project file |
//! |---|---|---|
//! | `yourproject-board` | a board: a netlist this crate bundles | `[[board]] kind = "yourproject-board"` |
//! | `yourproject-sensor` | a part model | `[[board.model]] value = "YOURPROJECT-SENSOR"`, `kind = "yourproject-sensor"` |
//! | `yourproject-core` | what runs inside the P2's package | `[board.model.options] core = "yourproject-core"`, `pin = 0` |
//! | `yourproject-source` | a bench component | `[[component]] kind = "yourproject-source"`, `volts`, `ohms` |
//!
//! Keep the ones the project needs, rename them, and write the rest. Three
//! rules bind them as they bind embsim's own (`DESIGN.md`):
//!
//! - **Every number has its source.** A model's figures come from its
//!   part's datasheet, each with its citation beside it; a figure the
//!   bench sets (a supply's volts) is an option the project file gives.
//!   None of these examples holds a figure of its own for that reason.
//! - **One interface.** A model talks to the board through its declared
//!   pins: drives it publishes, senses it is handed, wakes it schedules.
//!   A mechanism (a shaft, a carriage) stays inside one component, which
//!   presents electrical pins.
//! - **Nothing starts at registration.** `embsim survey` and `new`
//!   register every kind and build nothing; a thread starts, or a core
//!   runs, when the board or the bench is built.
//!
//! [`register`] is what the runner the `embsim` tool builds calls: keep its
//! name and signature. A finished catalog of this shape, with a test that
//! runs its project, is `examples/custom-project` in the embsim repository.

use std::sync::{Arc, Mutex};

use embsim_board::{
    netlist, Assignment, AttachError, BoardSpec, Catalog, CatalogBoard, Component, ComponentNetIo,
    ComponentRequest, Drive, KindGuide, ModelFacade, Named, PartOptions, PartRegistry, PinDecl,
    PinHandle, ProjectError, Report, Reports, TheveninDrive,
};
use embsim_boards::catalog::CatalogSet;
use embsim_boards::p2::{
    CoreCatalog, CoreCtor, CoreKind, P2Core, P2Pads, PadDrive, NATIVE_PAD_MODE, NUM_PADS,
};

/// Add this crate's kinds to `set`: called once by the runner, after the
/// catalogs embsim ships are in it and before the project is read. Starts
/// nothing.
pub fn register(set: &mut CatalogSet) -> Result<(), ProjectError> {
    // Board, part and bench component kinds: one `Catalog`.
    set.add(Kinds)?;
    // P2 cores: one `CoreCatalog`, under the same name.
    set.add_cores(Cores)?;
    Ok(())
}

/// The catalog's name, as `embsim check` lists it and an error naming two
/// catalogs prints it: the crate's name.
const CATALOG: &str = "yourproject-catalog";

/// The kinds. A kind is lowercase letters, digits and hyphens, and starts
/// with the project's name so no kind embsim ships ever meets it.
const BOARD: &str = "yourproject-board";
const SENSOR: &str = "yourproject-sensor";
const CORE: &str = "yourproject-core";
const SOURCE: &str = "yourproject-source";

// ============================================================
// A board kind: a netlist the crate bundles
// ============================================================

/// The board's netlist, as `kicad-cli sch export netlist` writes one (a
/// real board bundles its export with `include_str!`): a two-pin header
/// `J1` and the sensor `U1` across it.
const BOARD_NETLIST: &str = r#"(export (version "E")
  (components
    (comp (ref "J1") (value "Conn_01x02")
      (libsource (lib "Connector") (part "Conn_01x02")))
    (comp (ref "U1") (value "YOURPROJECT-SENSOR")
      (libsource (lib "yourproject") (part "Sensor"))))
  (nets
    (net (code "1") (name "IN")
      (node (ref "J1") (pin "1") (pinfunction "Pin_1"))
      (node (ref "U1") (pin "1")))
    (net (code "2") (name "GND")
      (node (ref "J1") (pin "2") (pinfunction "Pin_2"))
      (node (ref "U1") (pin "2")))))"#;

/// Board, part and bench component kinds.
struct Kinds;

impl Catalog for Kinds {
    fn name(&self) -> &str {
        CATALOG
    }

    fn board_kinds(&self) -> Vec<String> {
        vec![BOARD.to_string()]
    }

    fn board(&self, spec: &BoardSpec) -> Result<CatalogBoard, ProjectError> {
        let netlist = netlist::parse(BOARD_NETLIST)
            .map_err(|err| ProjectError::message(format!("board {}: {err}", spec.name)))?;
        // The connector and anything embsim places by part number come
        // from the base registrations; `U1` is left for the project's
        // `[[board.model]]`. A board may also bring entries for its own
        // parts: `.with_model(ModelSpec::by_value("YOURPROJECT-SENSOR",
        // SENSOR))`.
        Ok(CatalogBoard::from_base(netlist))
    }

    fn part_kinds(&self) -> Vec<KindGuide> {
        // What the kind is, and what a part must be for it to seat there:
        // the project checks every part an entry reaches against `is`
        // before `register_part` runs.
        vec![KindGuide::new(
            SENSOR,
            "a sensor that reads the voltage across its two pins",
            Named::Family(&["YOURPROJECT-SENSOR"]),
        )]
    }

    fn register_part(
        &self,
        registry: &mut PartRegistry,
        assignment: &Assignment<'_>,
        options: PartOptions,
    ) -> Result<(), ProjectError> {
        // `register_part` is called only for this catalog's part kinds.
        // This one takes no options; `finish` refuses any it is given,
        // naming the ones it takes.
        options.finish()?;
        let board = assignment.board.to_string();
        let reports = assignment.reports.clone();
        // The facade is the model's pin table: the survey checks it against
        // the netlist without building anything. The constructor runs once
        // per part, when the board is built.
        registry.register_model(
            assignment.key,
            ModelFacade::of(SENSOR, &SENSOR_PINS),
            move |decl| {
                let reading = Arc::new(Mutex::new(None));
                reports.add(SensorReport {
                    subject: format!("{board}.{}", decl.reference),
                    reading: Arc::clone(&reading),
                });
                Box::new(Sensor { reading })
            },
        );
        Ok(())
    }

    fn component_kinds(&self) -> Vec<String> {
        vec![SOURCE.to_string()]
    }

    fn component(&self, request: ComponentRequest<'_>) -> Result<Box<dyn Component>, ProjectError> {
        source(request)
    }
}

// ============================================================
// A part kind: the sensor's model
// ============================================================

/// The sensor's pins: what it reads (`1`, against `2`), and its return.
const SENSOR_PINS: [PinDecl; 2] = [
    PinDecl::analog("1").with_reference("2"),
    PinDecl::power_in("2"),
];

/// The sensor: an analog reader across its pins. A real part's model
/// declares its datasheet's thresholds, supply and outputs here.
struct Sensor {
    /// The voltage last handed to pin 1, `None` while no source reaches it.
    reading: Arc<Mutex<Option<Option<f64>>>>,
}

impl Component for Sensor {
    fn pins(&self) -> &[PinDecl] {
        &SENSOR_PINS
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let reading = Arc::clone(&self.reading);
        // Delivered once at registration and on every change of the net,
        // on the engine thread.
        io.on_sense("1", move |sense| {
            *reading.lock().expect("the reading is never poisoned") = Some(sense.volts);
        })
    }
}

/// What a sensor says at the end of a run.
struct SensorReport {
    subject: String,
    reading: Arc<Mutex<Option<Option<f64>>>>,
}

impl Report for SensorReport {
    fn subject(&self) -> String {
        self.subject.clone()
    }

    fn look(&mut self, _now_ns: u64) -> Vec<String> {
        Vec::new()
    }

    fn summary(&self) -> Vec<String> {
        let line = match *self.reading.lock().expect("the reading is never poisoned") {
            Some(Some(volts)) => format!("pin 1 read {volts} V"),
            Some(None) => "pin 1 read no voltage: no source reaches it".to_string(),
            None => "pin 1 was never read".to_string(),
        };
        vec![line]
    }
}

// ============================================================
// A P2 core kind: what runs inside the processor
// ============================================================

/// The core kinds. The `p2` part kind is the package — its pins, its reset
/// and restart delay, its bank supplies — and asks the set for the core
/// its `core` option names, handing it the entry's other options.
struct Cores;

impl CoreCatalog for Cores {
    fn name(&self) -> &str {
        CATALOG
    }

    fn core_kinds(&self) -> Vec<CoreKind> {
        vec![CoreKind {
            name: CORE,
            summary: "a core that drives one pad high from the moment it starts",
        }]
    }

    fn seat(
        &self,
        _core: &str,
        assignment: &Assignment<'_>,
        mut options: PartOptions,
    ) -> Result<CoreCtor, ProjectError> {
        let pin = options
            .integer("pin")?
            .ok_or_else(|| options.error("options.pin is the pad the core drives, 0 to 63"))?;
        let pin = u8::try_from(pin)
            .ok()
            .filter(|pin| usize::from(*pin) < NUM_PADS)
            .ok_or_else(|| options.error(format!("options.pin = {pin} is not a pad, 0 to 63")))?;
        options.finish()?;
        let reports = assignment.reports.clone();
        let board = assignment.board.to_string();
        // Called once per part when the board is built; a survey never
        // calls it.
        Ok(Box::new(move |decl| {
            let driven = Arc::new(Mutex::new(false));
            reports.add(CoreReport {
                subject: format!("{board}.{}", decl.reference),
                pin,
                driven: Arc::clone(&driven),
            });
            Ok(Box::new(PadHigh {
                pin,
                pad: None,
                driven,
            }) as Box<dyn P2Core>)
        }))
    }
}

/// The core: at the START instant it drives its pad high, as a program's
/// `DRVH` would with the pad in its reset mode (fast both ways).
struct PadHigh {
    pin: u8,
    /// The pad's handle and the package's bank supplies, from attach.
    pad: Option<(PinHandle, P2Pads)>,
    driven: Arc<Mutex<bool>>,
}

impl P2Core for PadHigh {
    fn attach(&mut self, pads: P2Pads) -> Result<(), AttachError> {
        // A core keeps what it needs from its pads: here one pad's handle.
        // `pads.on_pad_sense` reads a pad, `pads.on_wake_ns` and
        // `pads.schedule_at_ns` give it time through the package's START
        // gate.
        self.pad = Some((pads.pad(self.pin)?, pads));
        Ok(())
    }

    fn start(&mut self) {
        let Some((pad, pads)) = &self.pad else {
            return;
        };
        // The drive a pad presents is the package's: its bank's supply
        // behind the mode's impedance (P2 datasheet), or nothing when the
        // bank has no supply.
        match pads
            .bank_supplies()
            .pad_drive(self.pin, NATIVE_PAD_MODE, true, true)
        {
            PadDrive::Thevenin(drive) => {
                pad.drive(Drive::Thevenin(drive));
                *self.driven.lock().expect("never poisoned") = true;
            }
            PadDrive::Released | PadDrive::CurrentSource(_) => pad.release(),
        }
    }
}

/// What the core says at the end of a run.
struct CoreReport {
    subject: String,
    pin: u8,
    driven: Arc<Mutex<bool>>,
}

impl Report for CoreReport {
    fn subject(&self) -> String {
        self.subject.clone()
    }

    fn look(&mut self, _now_ns: u64) -> Vec<String> {
        Vec::new()
    }

    fn summary(&self) -> Vec<String> {
        let driven = *self.driven.lock().expect("never poisoned");
        vec![if driven {
            format!("drove P{} high", self.pin)
        } else {
            format!(
                "did not drive P{}: it did not start, or its bank had no supply",
                self.pin
            )
        }]
    }
}

// ============================================================
// A bench component kind
// ============================================================

/// `yourproject-source`: one pin, `OUT`, driving `volts` behind `ohms`
/// from the moment the system starts. Both are the bench's to say, so both
/// are required: the kind invents neither.
fn source(request: ComponentRequest<'_>) -> Result<Box<dyn Component>, ProjectError> {
    let ComponentRequest {
        spec,
        mut options,
        reports,
        ..
    } = request;
    let volts = options
        .number("volts")?
        .ok_or_else(|| options.error("options.volts is what the source drives"))?;
    let ohms = options
        .number("ohms")?
        .filter(|ohms| *ohms > 0.0)
        .ok_or_else(|| options.error("options.ohms, more than 0, is the source's impedance"))?;
    options.finish()?;
    add_source_report(reports, &spec.name, volts, ohms);
    Ok(Box::new(Source {
        pins: [PinDecl::analog_source("OUT").with_idle(Some(TheveninDrive {
            volts,
            impedance: ohms,
        }))],
    }))
}

/// The source: its one pin idles at the drive the file named.
struct Source {
    pins: [PinDecl; 1],
}

impl Component for Source {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, _io: ComponentNetIo) -> Result<(), AttachError> {
        Ok(())
    }
}

/// Say what the source drives, once, at the end of the run.
fn add_source_report(reports: &Reports, name: &str, volts: f64, ohms: f64) {
    reports.add(SourceReport {
        subject: name.to_string(),
        line: format!("drove OUT at {volts} V behind {ohms} Ω"),
    });
}

/// What the source says.
struct SourceReport {
    subject: String,
    line: String,
}

impl Report for SourceReport {
    fn subject(&self) -> String {
        self.subject.clone()
    }

    fn look(&mut self, _now_ns: u64) -> Vec<String> {
        Vec::new()
    }

    fn summary(&self) -> Vec<String> {
        vec![self.line.clone()]
    }
}
