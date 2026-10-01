//! The example project's catalog: the four kinds `examples/custom-project`
//! adds to embsim, one of each sort (`PROJECTS.md` §10).
//!
//! | Kind | Sort | What it is |
//! |---|---|---|
//! | `example-buffer-board` | board | a small board bundled with the crate: the EX-BUF1 buffer behind a four-pin header ([`board`]) |
//! | `example-ex-buf1` | part | the EX-BUF1 single Schmitt-trigger buffer, from its stand-in datasheet ([`buffer`]) |
//! | `example-blinker` | P2 core | a core that toggles one pad on a schedule ([`blinker`]) |
//! | `example-edge-counter` | bench component | an instrument that counts rising edges on its input ([`counter`]) |
//!
//! **The part is an example.** No EX-BUF1 exists: `datasheets/EX-BUF1.md`
//! is a stand-in datasheet written for this example, so the model can cite
//! a section for every figure it uses as a real part's model cites its
//! vendor's. A project's real model cites its real part.
//!
//! [`register`] is the crate's registration function: the runner the
//! `embsim` tool builds for `project.toml` calls it, and so does the
//! project's own test (`tests/project.rs`).

use embsim_board::ProjectError;
use embsim_boards::catalog::CatalogSet;

pub mod blinker;
pub mod board;
pub mod buffer;
pub mod counter;

/// The catalog's name: the crate's.
pub const NAME: &str = "custom-project-catalog";

/// Add the example's kinds to `set`: the board, part and bench component
/// kinds as one catalog ([`board::ExampleCatalog`]), the core as a core
/// catalog ([`blinker::BlinkerCores`]). Starts nothing.
pub fn register(set: &mut CatalogSet) -> Result<(), ProjectError> {
    set.add(board::ExampleCatalog)?;
    set.add_cores(blinker::BlinkerCores)?;
    Ok(())
}
