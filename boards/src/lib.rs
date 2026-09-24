//! embsim-boards — off-the-shelf boards and modules, ready to populate.
//!
//! A netlist on its own is a description; a board is that description with
//! something behind every component. This crate supplies both for hardware you
//! can buy, so a test can say "a Parallax P2-EC32MB" and mean the same circuit
//! on every machine that runs it.
//!
//! Each board leaves its **processor** to the consumer. That is the useful
//! split: the CPU is usually what is under test, and everything around it —
//! the memories, the regulators, the card socket, the shared buses that make a
//! real module awkward — is the fixture that makes the test worth running.
//!
//! - [`ec32mb`] — the Parallax P2-EC32MB Propeller 2 module
//! - [`stub`] — pin facades for the parts of a board that are real but not
//!   yet modelled, and the rule for choosing their pin directions (retiring:
//!   `DESIGN.md` rule 1 admits no stub tier, and `NODES.md` §8 replaces each
//!   facade with a model)

pub mod ec32mb;
pub mod stub;
