//! The board kinds embsim ships, as a [`Catalog`] a project file can name.
//!
//! A kind is a module this crate already knows how to build. The project
//! says which one, and the options that module takes. `p2-ec32mb` is the
//! Parallax module: its regulators, flash, and socket are the module's, and
//! `core` says what sits in `U100`.

use embsim_board::{
    netlist, Board, BoardSpec, BoardSurvey, Catalog, Component, ComponentSpec, ProjectError,
};

use crate::ec32mb::Ec32mb;
use crate::p2::P2Package;

/// Kinds this crate ships. A project that names another kind fails here.
#[derive(Debug, Default)]
pub struct StandardCatalog;

impl Catalog for StandardCatalog {
    fn board(&self, spec: &BoardSpec) -> Result<Board, ProjectError> {
        match spec.kind.as_str() {
            "p2-ec32mb" => ec32(spec),
            other => Err(ProjectError::message(format!(
                "board {name}: unknown kind {other:?}",
                name = spec.name
            ))),
        }
    }

    fn survey(&self, spec: &BoardSpec) -> Result<BoardSurvey, ProjectError> {
        match spec.kind.as_str() {
            "p2-ec32mb" => {
                // The same module `board` builds, so the checklist and the
                // build cannot disagree about which parts have a model.
                let registry = seated(spec)?.registry();
                let parsed = netlist::parse(crate::ec32mb::NETLIST)
                    .expect("the bundled EC32 netlist parses");
                Ok(BoardSurvey::of(&parsed, &registry))
            }
            "netlist" => Err(ProjectError::message(
                "kind \"netlist\" is surveyed from the file's netlist path".to_string(),
            )),
            other => Err(ProjectError::message(format!(
                "board {name}: unknown kind {other:?}",
                name = spec.name
            ))),
        }
    }

    fn component(&self, spec: &ComponentSpec) -> Result<Box<dyn Component>, ProjectError> {
        Err(ProjectError::message(format!(
            "component {name}: unknown kind {kind:?}",
            name = spec.name,
            kind = spec.kind
        )))
    }
}

fn seated(spec: &BoardSpec) -> Result<Ec32mb, ProjectError> {
    let core = spec.core.as_deref().unwrap_or("held-in-reset");
    match core {
        "held-in-reset" => Ok(Ec32mb::new().with_p2(|_decl| Box::new(P2Package::held_in_reset()))),
        other => Err(ProjectError::message(format!(
            "board {name}: p2-ec32mb core {other:?} is not a core this catalog seats",
            name = spec.name
        ))),
    }
}

fn ec32(spec: &BoardSpec) -> Result<Board, ProjectError> {
    seated(spec)?
        .build()
        .map_err(|err| ProjectError::message(format!("board {name}: {err}", name = spec.name)))
}
