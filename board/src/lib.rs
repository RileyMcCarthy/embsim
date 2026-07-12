//! embsim-board — component-centric board/system simulation engine.
//!
//! Turns embsim from a firmware-centric emulator into a system simulator:
//! boards are ingested from EDA netlists (`kicad-cli sch export netlist`),
//! every component — including the MCU — is a [`Component`] with named pins,
//! and an engine resolves the nets between them. Consumers stop writing
//! wiring code and start writing *system descriptions*.
//!
//! Authoritative design: `BOARD_ENGINE.md` at the workspace root. This crate
//! currently implements the **build-time analysis slice** (netlist → board →
//! system → resolution pass → findings). The live net-engine thread, timer
//! wheel, full MNA solver, and stream byte pipes are specified but not yet
//! implemented; their seams (types/traits) are defined here so consumers and
//! follow-up slices build against stable signatures.
//!
//! Module map (mirrors the design doc's crate layout):
//! - [`netlist`] — KiCad s-expression netlist parser → [`ComponentDecl`]/[`NetDecl`] graph
//! - [`component`] — [`Component`] trait, [`PinDecl`], [`PinKind`], [`StreamRole`], [`ComponentNetIo`]
//! - [`registry`] — [`PartRegistry`]: identity → constructor; auto-classification tiers
//! - [`net`] — net state model ([`NetState`]) and shared net/pin identity types
//! - [`cluster`] — analog cluster types + [`ClusterSolver`] trait ([`QuasiStaticMna`] default)
//! - [`board`] — [`Board::from_netlist`]: netlist + registry → components + nets
//! - [`system`] — [`System`]: boards + harnesses + scenario overrides + fault algebra
//! - [`diagnostics`] — structured [`Finding`]s on a [`Diagnostics`] collector, mirrored to `tracing`

pub mod board;
pub mod cluster;
pub mod component;
pub mod diagnostics;
pub mod net;
pub mod netlist;
pub mod registry;
pub mod system;

pub use board::{Board, BoardError};
pub use cluster::{
    Cluster, ClusterInputs, ClusterResistor, ClusterSolution, ClusterSolver, ClusterSource,
    QuasiStaticMna,
};
pub use component::{
    AttachError, Component, ComponentNetIo, PinDecl, PinHandle, PinKind, StreamRole,
};
pub use diagnostics::{Diagnostics, Finding, PinMismatchDirection, SenseKind};
pub use net::{Level, Net, NetId, NetState, Ohms, PinRef, TheveninDrive, Volts};
pub use netlist::{ComponentDecl, NetDecl, NetlistError, NodeDecl, ParsedNetlist};
pub use registry::{Classification, JumperState, PartRegistry, PassiveKind, RegistryError};
pub use system::{
    BuiltSystem, DnpState, EndpointKind, EndpointRef, Fault, Harness, HarnessConnection,
    HarnessError, Scenario, StreamDropPolicy, System, SystemError,
};
