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
//! - [`p2`] — the Propeller 2 package as a node: the 86-pin facade around
//!   a core (QEMU, an instruction-set simulator, the native firmware) or
//!   around no core at all, held in reset
//!
//! There is no stub tier (`DESIGN.md` rule 1): every part on a board here
//! is a node whose class has behaviour, and a part nobody has modelled is
//! a build error naming it.

pub mod ec32mb;
pub mod p2;
