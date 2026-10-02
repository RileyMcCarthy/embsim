//! QEMU as a core the P2 package can hold, for a project.
//!
//! [`QemuCores`] is a [`CoreCatalog`] of one core kind, `qemu`: the `p2`
//! part kind seats a [`P2Qemu`] inside its [`P2Package`] (the package and
//! its START gate are the standard catalog's, `embsim_boards::p2`), and
//! the chip boots its ROM — [`BOOT_ROM`], or the file its `rom` option
//! names, relative to the project file — the way `tests/rom_boot_ec32mb.rs`
//! boots it, off whatever the board's nets give it. [`register`] adds it to
//! a set, as a project's own catalog crate adds its kinds:
//!
//! ```
//! use embsim_boards::catalog::CatalogSet;
//!
//! let mut set = CatalogSet::new();
//! embsim_p2_qemu::catalog::register(&mut set).expect("one core, spelled as a kind is");
//! let cores: Vec<&str> = set.core_kinds().iter().map(|kind| kind.name).collect();
//! assert_eq!(cores, ["held-in-reset", "qemu"]);
//! ```
//!
//! ```toml
//! [[board.model]]
//! value = "P2X8C4M64P"
//! kind = "p2"
//! [board.model.options]
//! core = "qemu"               # or "held-in-reset"
//! rom = "rom.bin"             # optional; the chip's own ROM otherwise
//! ```
//!
//! Seating finds the `qemu-system-p2` to run ([`QemuSystemP2::find`]) and
//! refuses the entry, saying how to install one, when there is none. The
//! chip boots when the board is built — seating only checks — so a survey
//! of the project boots nothing; each part the key reaches boots a program
//! of its own. The core reports its console, per pad, its yields, and why
//! it stopped if its program died ([`Report`]).

use embsim_board::{Assignment, PartOptions, ProjectError, Report};
use embsim_boards::catalog::CatalogSet;
use embsim_boards::p2::{CoreCatalog, CoreCtor, CoreKind, P2Core};

use crate::{P2Qemu, P2QemuHandle, QemuSystemP2, Transport, BOOT_ROM};

#[cfg(doc)]
use embsim_boards::p2::P2Package;

/// The catalog's name, as an error naming two catalogs prints it.
pub const NAME: &str = "embsim-p2-qemu";

/// The P2's smart pins, the ones a console can be on.
const P2_PADS: u8 = 64;

/// Add QEMU's core to `set`.
pub fn register(set: &mut CatalogSet) -> Result<(), ProjectError> {
    set.add_cores(QemuCores::default())
}

/// The `qemu` core kind (module docs).
#[derive(Debug, Default, Clone)]
pub struct QemuCores {
    /// The program every seat runs; found when a part is seated if `None`.
    program: Option<QemuSystemP2>,
}

impl QemuCores {
    /// The kind, running `program` for every part it seats instead of the
    /// one [`QemuSystemP2::find`] finds.
    pub fn with_program(program: QemuSystemP2) -> Self {
        Self {
            program: Some(program),
        }
    }
}

impl CoreCatalog for QemuCores {
    fn name(&self) -> &str {
        NAME
    }

    fn core_kinds(&self) -> Vec<CoreKind> {
        vec![CoreKind {
            name: "qemu",
            summary: "the chip booting its ROM on QEMU",
        }]
    }

    fn seat(
        &self,
        _core: &str,
        assignment: &Assignment<'_>,
        mut options: PartOptions,
    ) -> Result<CoreCtor, ProjectError> {
        let rom = options.string("rom")?;
        options.finish()?;
        let program = match &self.program {
            Some(program) => program.clone(),
            None => QemuSystemP2::find().map_err(|err| assignment.error(err))?,
        };
        let transport = Transport::from_env().map_err(|err| assignment.error(err))?;
        let rom = match &rom {
            None => BOOT_ROM.to_vec(),
            Some(file) => {
                let path = assignment.dir.join(file);
                std::fs::read(&path).map_err(|err| {
                    assignment.error(format!("cannot read boot ROM {}: {err}", path.display()))
                })?
            }
        };
        let reports = assignment.reports.clone();
        let board = assignment.board.to_string();
        Ok(Box::new(move |decl| {
            // The chip boots here, when the board is built: a survey never
            // builds, so it never boots one.
            let p2 = P2Qemu::start(&program, &rom, &[], transport)
                .map_err(|err| format!("QEMU did not boot: {err}"))?;
            reports.add(QemuReport {
                subject: format!("{board}.{}", decl.reference),
                core: p2.handle(),
                printed: vec![0; usize::from(P2_PADS)],
                halted: false,
                failed: false,
            });
            Ok(Box::new(p2) as Box<dyn P2Core>)
        }))
    }
}

/// What a QEMU core says in a run: what its guest wrote to each pad's
/// console as it writes it, when every cog stops, and at the end its yields,
/// whether it runs, and every console.
struct QemuReport {
    subject: String,
    core: P2QemuHandle,
    /// Console characters printed so far, per pad.
    printed: Vec<usize>,
    halted: bool,
    /// Whether the run has been told the program failed.
    failed: bool,
}

impl Report for QemuReport {
    fn subject(&self) -> String {
        self.subject.clone()
    }

    fn look(&mut self, _now_ns: u64) -> Vec<String> {
        let mut lines = Vec::new();
        for pad in 0..P2_PADS {
            let text = self.core.console(pad);
            let printed = &mut self.printed[usize::from(pad)];
            let count = text.chars().count();
            if count > *printed {
                let new: String = text.chars().skip(*printed).collect();
                lines.push(format!("P{pad} {new:?}"));
                *printed = count;
            }
        }
        if !self.failed {
            if let Some(failure) = self.core.failure() {
                self.failed = true;
                self.halted = true;
                lines.push(format!("QEMU stopped: {failure}"));
            }
        }
        if !self.halted && self.core.halted() {
            self.halted = true;
            lines.push("every cog has stopped".to_string());
        }
        lines
    }

    fn summary(&self) -> Vec<String> {
        let consoles: Vec<String> = (0..P2_PADS)
            .filter_map(|pad| {
                let text = self.core.console(pad);
                (!text.is_empty()).then(|| format!("P{pad} {text:?}"))
            })
            .collect();
        let state = match self.core.failure() {
            // The whole of it was printed when it happened; its first line
            // says what.
            Some(failure) => format!("stopped: {}", failure.lines().next().unwrap_or_default()),
            None if self.core.halted() => "halted".to_string(),
            None => "running".to_string(),
        };
        vec![format!(
            "QEMU: {} pad yields; {state}; console {}",
            self.core.yields(),
            if consoles.is_empty() {
                "empty".to_string()
            } else {
                consoles.join(", ")
            }
        )]
    }
}

#[cfg(test)]
mod tests {
    use std::path::Path;

    use embsim_board::{Catalog, ComponentDecl, KeyField, ParsedNetlist, PartRegistry, Reports};

    use super::*;

    fn decl(reference: &str) -> ComponentDecl {
        ComponentDecl {
            reference: reference.to_string(),
            value: "P2X8C4M64P".to_string(),
            footprint: String::new(),
            lib: String::new(),
            part: String::new(),
            sheetpath: "/".to_string(),
            dnp: false,
            mpn: None,
        }
    }

    /// The `p2` kind through a set holding QEMU's core, as a project
    /// registers it, the core running `program` (which seating never
    /// starts).
    fn register(core: &str, extra: &str, parts: &[&ComponentDecl]) -> Result<(), ProjectError> {
        let mut set = CatalogSet::new();
        set.add_cores(QemuCores::with_program(QemuSystemP2::at(
            "/nonexistent/qemu-system-p2",
        )))
        .expect("QEMU's core joins the set");
        register_in(&set, core, extra, parts)
    }

    fn register_in(
        set: &CatalogSet,
        core: &str,
        extra: &str,
        parts: &[&ComponentDecl],
    ) -> Result<(), ProjectError> {
        let netlist = ParsedNetlist {
            version: "E".to_string(),
            components: Vec::new(),
            nets: Vec::new(),
        };
        let reports = Reports::new();
        let assignment = Assignment {
            board: "EC32",
            by: KeyField::Value,
            key: "P2X8C4M64P",
            kind: "p2",
            parts,
            dir: Path::new("."),
            netlist: &netlist,
            reports: &reports,
        };
        let table: toml::Table =
            toml::from_str(&format!("core = {core:?}\n{extra}")).expect("the options parse");
        set.register_part(
            &mut PartRegistry::new(),
            &assignment,
            PartOptions::new(assignment.context(), table),
        )
    }

    #[test]
    fn held_in_reset_is_the_standard_catalogs_and_takes_no_rom() {
        let u100 = decl("U100");
        register("held-in-reset", "", &[&u100]).expect("the standard seat");
        let err = register("held-in-reset", "rom = \"rom.bin\"", &[&u100])
            .expect_err("a ROM is for a core that runs");
        assert!(err.to_string().contains("unknown option \"rom\""), "{err}");
    }

    #[test]
    fn an_unknown_core_names_both_the_set_seats() {
        let u100 = decl("U100");
        let err = register("p2core", "", &[&u100]).expect_err("not a core here");
        assert!(
            err.to_string()
                .contains("it offers \"held-in-reset\", \"qemu\""),
            "{err}"
        );
    }

    /// A core is seated only in a part the board names a P2, whatever runs
    /// inside it: the project checks the part before the set seats a core.
    #[test]
    fn a_core_in_a_part_that_is_not_a_p2_is_refused() {
        let mut set = CatalogSet::new();
        super::register(&mut set).expect("QEMU's core joins the set");
        for core in ["held-in-reset", "qemu"] {
            let project = embsim_board::Project::parse(&format!(
                "[[board]]\nname = \"EC32\"\nkind = \"netlist\"\n\
                 netlist = \"../boards/netlists/p2_ec32mb.net\"\n[[board.model]]\n\
                 mpn = \"218-4LPSTJR\"\nkind = \"p2\"\n[board.model.options]\n\
                 core = {core:?}\n"
            ))
            .expect("the text is a project")
            .relative_to(env!("CARGO_MANIFEST_DIR"));
            let err = project
                .survey(&set, "EC32")
                .expect_err("the option switch is no P2");
            assert!(
                err.to_string()
                    .contains("S301 is not the part this kind says it is"),
                "{err}"
            );
        }
    }

    /// Seating checks and starts nothing: a key may reach two P2s, each
    /// booting a program of its own when the board is built, and a ROM
    /// file that is not there is refused before anything is built.
    #[test]
    fn a_qemu_core_seats_on_every_part_its_key_reaches_and_starts_nothing() {
        let u100 = decl("U100");
        let u200 = decl("U200");
        register("qemu", "", &[&u100, &u200]).expect("two parts, two programs at build");
        let err = register("qemu", "rom = \"no-such-rom.bin\"", &[&u100])
            .expect_err("a ROM that is not there");
        assert!(err.to_string().contains("cannot read boot ROM"), "{err}");
    }

    /// The set's own `qemu` finds its program when a part is seated, and
    /// with none to find refuses the entry saying how to install one.
    #[test]
    fn a_qemu_core_with_no_program_to_find_is_refused_saying_how_to_install_one() {
        let set = {
            let mut set = CatalogSet::new();
            super::register(&mut set).expect("QEMU's core joins the set");
            set
        };
        let u100 = decl("U100");
        match (
            QemuSystemP2::find(),
            register_in(&set, "qemu", "", &[&u100]),
        ) {
            (Ok(_), outcome) => outcome.expect("a program was found"),
            (Err(_), outcome) => {
                let err = outcome.expect_err("nothing to run");
                assert!(err.to_string().contains("`embsim qemu install`"), "{err}");
            }
        }
    }
}
