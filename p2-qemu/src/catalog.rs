//! The standard catalog, with QEMU as a core the P2 package can hold.
//!
//! [`QemuCatalog`] is [`StandardCatalog`] with one more value for the `p2`
//! part kind's `core` option. `core = "qemu"` seats a [`P2Qemu`] inside the
//! [`P2Package`]: the chip boots its ROM — [`BOOT_ROM`], or the file its
//! `rom` option names, relative to the project file — the way
//! `tests/rom_boot_ec32mb.rs` boots it, off whatever the board's nets give
//! it. Every other kind, option and board is the standard catalog's.
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
//! QEMU is one machine per process, so a `core = "qemu"` key may reach one
//! part, and a build with no QEMU linked ([`crate::linked`]) refuses the
//! entry with [`P2QemuError::Unavailable`]'s message. The chip boots when
//! the board is built — the registration only checks — so a survey of the
//! project boots nothing. [`QemuCatalog::seats`] hands out a view of each
//! core it seated, and of its package, for a program to report from.

use std::sync::{Arc, Mutex};

use embsim_board::{
    Assignment, AttachError, BoardSpec, Catalog, CatalogBoard, Component, ComponentNetIo,
    ComponentSpec, ModelFacade, PartOptions, PartRegistry, PinDecl, ProjectError,
};
use embsim_boards::catalog::{KindGuide, StandardCatalog};
use embsim_boards::p2::{P2Package, P2PackageHandle};

use crate::{p2x8c4m64p_pins, P2Qemu, P2QemuError, P2QemuHandle, BOOT_ROM};

/// The values the `p2` kind's `core` option takes here.
const CORES: [&str; 2] = ["held-in-reset", "qemu"];

/// A QEMU core the catalog seated, with the views of it that outlive the
/// system it runs in.
#[derive(Clone)]
pub struct QemuSeat {
    /// The board it sits on, as the project names it.
    pub board: String,
    /// The part it is (`"U100"`).
    pub reference: String,
    /// The core: its console, its yields, whether it halted.
    pub core: P2QemuHandle,
    /// The package around it: when the START gate opened, the crystal, the
    /// reset inputs.
    pub package: P2PackageHandle,
}

impl std::fmt::Debug for QemuSeat {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("QemuSeat")
            .field("board", &self.board)
            .field("reference", &self.reference)
            .field("package", &self.package)
            .finish()
    }
}

/// [`StandardCatalog`], with `core = "qemu"` for the `p2` kind (module docs).
#[derive(Debug, Default, Clone)]
pub struct QemuCatalog {
    seats: Arc<Mutex<Vec<QemuSeat>>>,
}

impl QemuCatalog {
    /// A catalog that has seated nothing yet.
    pub fn new() -> Self {
        Self::default()
    }

    /// Every QEMU core seated so far, in the order the boards were built.
    pub fn seats(&self) -> Vec<QemuSeat> {
        self.seats.lock().expect("seats never poisoned").clone()
    }

    /// The standard catalog's guide, with the `p2` kind's `core` saying
    /// what this catalog seats.
    pub fn guide() -> Vec<KindGuide> {
        let mut guide = StandardCatalog::guide();
        for kind in guide.iter_mut().filter(|kind| kind.name == "p2") {
            for option in kind.required.iter_mut().filter(|o| o.name == "core") {
                option.means = "what runs inside the package: \"held-in-reset\", the chip before \
                                it runs, or \"qemu\", the chip booting its ROM";
            }
        }
        guide
    }

    fn register_p2(
        &self,
        registry: &mut PartRegistry,
        assignment: &Assignment<'_>,
        mut options: PartOptions,
    ) -> Result<(), ProjectError> {
        // The part has to be a P2, whatever runs in it.
        StandardCatalog::check_parts_are_the_kind(assignment)?;
        let core = options.choice("core", &CORES)?.ok_or_else(|| {
            assignment.error(
                "options.core says what runs inside the package: \"held-in-reset\", the chip \
                 before it runs, or \"qemu\", the chip booting its ROM on QEMU",
            )
        })?;
        if core == "held-in-reset" {
            options.finish()?;
            let mut table = toml::Table::new();
            table.insert("core".to_string(), toml::Value::String(core.to_string()));
            return StandardCatalog.register_part(
                registry,
                assignment,
                PartOptions::new(assignment.context(), table),
            );
        }
        let rom = options.string("rom")?;
        options.finish()?;
        if !crate::linked() {
            return Err(assignment.error(P2QemuError::Unavailable));
        }
        if let [first, second, ..] = assignment.parts {
            return Err(assignment.error(format!(
                "QEMU is one machine per process, and this key reaches {} and {}; give one \
                 processor core = \"qemu\" by a key only it has",
                first.reference, second.reference
            )));
        }
        let rom = match &rom {
            None => BOOT_ROM.to_vec(),
            Some(file) => {
                let path = assignment.dir.join(file);
                std::fs::read(&path).map_err(|err| {
                    assignment.error(format!("cannot read boot ROM {}: {err}", path.display()))
                })?
            }
        };
        let seats = Arc::clone(&self.seats);
        let board = assignment.board.to_string();
        registry.register_model(
            assignment.key,
            ModelFacade::of("p2, core = \"qemu\"", &p2x8c4m64p_pins()),
            move |decl| -> Box<dyn Component> {
                // The chip boots here, when the board is built: a survey
                // never builds, so it never boots one.
                match P2Qemu::with_boot_rom(&rom, &[]) {
                    Ok(p2) => {
                        let core = p2.handle();
                        let package = P2Package::new(p2);
                        seats.lock().expect("seats never poisoned").push(QemuSeat {
                            board: board.clone(),
                            reference: decl.reference.clone(),
                            core,
                            package: package.handle(),
                        });
                        Box::new(package)
                    }
                    Err(err) => Box::new(Unbooted {
                        pins: p2x8c4m64p_pins(),
                        message: format!("{}: QEMU did not boot: {err}", decl.reference),
                    }),
                }
            },
        );
        Ok(())
    }
}

impl Catalog for QemuCatalog {
    fn board_kinds(&self) -> Vec<String> {
        StandardCatalog.board_kinds()
    }

    fn board(&self, spec: &BoardSpec) -> Result<CatalogBoard, ProjectError> {
        StandardCatalog.board(spec)
    }

    fn base_registry(&self) -> PartRegistry {
        StandardCatalog::base_registry()
    }

    fn part_kinds(&self) -> Vec<String> {
        StandardCatalog.part_kinds()
    }

    fn register_part(
        &self,
        registry: &mut PartRegistry,
        assignment: &Assignment<'_>,
        options: PartOptions,
    ) -> Result<(), ProjectError> {
        if assignment.kind == "p2" {
            self.register_p2(registry, assignment, options)
        } else {
            StandardCatalog.register_part(registry, assignment, options)
        }
    }

    fn component_kinds(&self) -> Vec<String> {
        StandardCatalog.component_kinds()
    }

    fn component(&self, spec: &ComponentSpec) -> Result<Box<dyn Component>, ProjectError> {
        StandardCatalog.component(spec)
    }
}

/// The package's pins around a core QEMU could not boot: it refuses to
/// attach with the reason, so the system does not start.
struct Unbooted {
    pins: Vec<PinDecl>,
    message: String,
}

impl Component for Unbooted {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, _io: ComponentNetIo) -> Result<(), AttachError> {
        Err(AttachError::Failed {
            message: self.message.clone(),
        })
    }
}

#[cfg(test)]
mod tests {
    use std::path::Path;

    use embsim_board::{ComponentDecl, KeyField, ParsedNetlist};

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

    fn register(core: &str, extra: &str, parts: &[&ComponentDecl]) -> Result<(), ProjectError> {
        let netlist = ParsedNetlist {
            version: "E".to_string(),
            components: Vec::new(),
            nets: Vec::new(),
        };
        let assignment = Assignment {
            board: "EC32",
            by: KeyField::Value,
            key: "P2X8C4M64P",
            kind: "p2",
            parts,
            dir: Path::new("."),
            netlist: &netlist,
        };
        let table: toml::Table =
            toml::from_str(&format!("core = {core:?}\n{extra}")).expect("the options parse");
        QemuCatalog::new().register_part(
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
    fn an_unknown_core_names_both_this_catalog_seats() {
        let u100 = decl("U100");
        let err = register("p2core", "", &[&u100]).expect_err("not a core here");
        assert!(
            err.to_string()
                .contains("it offers \"held-in-reset\", \"qemu\""),
            "{err}"
        );
    }

    /// A core is seated only in a part the board names a P2, whatever runs
    /// inside it.
    #[test]
    fn a_core_in_a_part_that_is_not_a_p2_is_refused() {
        let mut other = decl("U7");
        other.value = "ATMEGA328P".to_string();
        for core in CORES {
            let err = register(core, "", &[&other]).expect_err("U7 is no P2");
            assert!(
                err.to_string()
                    .contains("U7 is not the part this kind says it is"),
                "{err}"
            );
        }
    }

    /// Without QEMU the entry is refused with the stub's own message; with
    /// it, a key reaching two parts is refused, since QEMU is one machine
    /// per process. Neither boots anything.
    #[test]
    fn a_qemu_core_is_refused_where_it_cannot_boot() {
        let u100 = decl("U100");
        let u200 = decl("U200");
        if crate::linked() {
            let err = register("qemu", "", &[&u100, &u200]).expect_err("two parts, one QEMU");
            assert!(err.to_string().contains("reaches U100 and U200"), "{err}");
            register("qemu", "", &[&u100]).expect("one part boots at build");
        } else {
            let err = register("qemu", "", &[&u100]).expect_err("no QEMU in this build");
            assert!(
                err.to_string()
                    .contains(&P2QemuError::Unavailable.to_string()),
                "{err}"
            );
        }
    }
}
