//! The catalog: the example's board, part and bench component kinds.
//!
//! - `example-buffer-board`: the netlist `netlists/buffer-board.net`,
//!   bundled. Its header `J1`, its resistors and anything else embsim
//!   places by part number take the base registrations; the buffer `U1` is
//!   left to the project's `[[board.model]]`, so the board needs the
//!   `example-ex-buf1` model before it builds.
//! - `example-ex-buf1`: the EX-BUF1's model ([`crate::buffer`]), seated
//!   only on a part whose part name, number or value names `EX-BUF1`.
//! - `example-edge-counter`: the bench instrument ([`crate::counter`]).

use embsim_board::{
    netlist, Assignment, BoardSpec, Catalog, CatalogBoard, Component, ComponentRequest, KindGuide,
    ModelFacade, Named, PartOptions, PartRegistry, ProjectError,
};

use crate::buffer::{Buffer, BUFFER_PINS};
use crate::counter;

/// The board kind.
pub const BOARD: &str = "example-buffer-board";
/// The part kind.
pub const BUFFER: &str = "example-ex-buf1";
/// The bench component kind.
pub const COUNTER: &str = "example-edge-counter";

/// The board's netlist, bundled with the crate as a real board's KiCad
/// export would be.
const BUFFER_BOARD: &str = include_str!("../netlists/buffer-board.net");

/// The example's board, part and bench component kinds.
#[derive(Debug, Clone, Copy, Default)]
pub struct ExampleCatalog;

impl Catalog for ExampleCatalog {
    fn name(&self) -> &str {
        crate::NAME
    }

    fn board_kinds(&self) -> Vec<String> {
        vec![BOARD.to_string()]
    }

    fn board(&self, spec: &BoardSpec) -> Result<CatalogBoard, ProjectError> {
        let netlist = netlist::parse(BUFFER_BOARD)
            .map_err(|err| ProjectError::message(format!("board {}: {err}", spec.name)))?;
        Ok(CatalogBoard::from_base(netlist))
    }

    fn part_kinds(&self) -> Vec<KindGuide> {
        vec![KindGuide::new(
            BUFFER,
            "the EX-BUF1 single Schmitt-trigger buffer (an example part, from its stand-in \
             datasheet)",
            Named::Family(&["EX-BUF1"]),
        )]
    }

    fn register_part(
        &self,
        registry: &mut PartRegistry,
        assignment: &Assignment<'_>,
        options: PartOptions,
    ) -> Result<(), ProjectError> {
        // The model's every number is the datasheet's: the kind takes no
        // options.
        options.finish()?;
        registry.register_model(
            assignment.key,
            ModelFacade::of(BUFFER, &BUFFER_PINS),
            |_| Box::new(Buffer::new()),
        );
        Ok(())
    }

    fn component_kinds(&self) -> Vec<String> {
        vec![COUNTER.to_string()]
    }

    fn component(&self, request: ComponentRequest<'_>) -> Result<Box<dyn Component>, ProjectError> {
        counter::build(request)
    }
}
