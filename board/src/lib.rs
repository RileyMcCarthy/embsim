//! embsim-board — component-centric board/system simulation engine.
//!
//! Turns embsim from a firmware-centric emulator into a system simulator:
//! boards are ingested from EDA netlists (`kicad-cli sch export netlist`),
//! every component — including the MCU — is a [`Component`] with named pins,
//! and an engine resolves the nets between them. Consumers stop writing
//! wiring code and start writing *system descriptions*.
//!
//! Authoritative design: `BOARD_ENGINE.md` at the workspace root. This crate
//! implements the **build-time analysis slice** (netlist → board → system →
//! resolution pass → findings) and the **live net-engine slice**
//! (`System::start`): the single-writer engine thread with its drive queue
//! and timer wheel, the quasi-static MNA cluster solver ([`QuasiStaticMna`]),
//! and stream byte pipes (routing, baud pacing, drop policies) derived from
//! net resolution. Both paths drive one shared resolution code path. Still
//! deferred to later slices: live topology mutation and its epoch
//! notification (the seam is registered), dead-band [`Finding::AmbiguousLevel`]
//! projection, and transducer primitives.
//!
//! Module map (mirrors the design doc's crate layout):
//! - [`netlist`] — KiCad s-expression netlist parser → [`ComponentDecl`]/[`NetDecl`] graph
//! - [`component`] — [`Component`] trait, [`PinDecl`], [`PinKind`], [`StreamRole`], [`ComponentNetIo`]
//! - [`registry`] — [`PartRegistry`]: identity → constructor; auto-classification tiers
//! - [`engine`] — the live single-writer net engine: drive queue, resolution, timer wheel, stream routing
//! - [`net`] — net state model ([`NetState`]) and shared net/pin identity types
//! - [`cluster`] — analog cluster types + [`ClusterSolver`] trait ([`QuasiStaticMna`] default)
//! - [`board`] — [`Board::from_netlist`]: netlist + registry → components + nets
//! - [`system`] — [`System`]: boards + harnesses + scenario overrides + fault algebra
//! - [`diagnostics`] — structured [`Finding`]s on a [`Diagnostics`] collector, mirrored to `tracing`
//! - [`mcu`] — [`McuComponent`]: the MCU as a component (force-path slice: serial
//!   channels bridged to stream pins; the firmware-entry inversion is deferred)

pub mod board;
pub mod cluster;
pub mod component;
pub mod diagnostics;
pub mod engine;
pub mod mcu;
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
pub use diagnostics::{CallbackKind, Diagnostics, Finding, PinMismatchDirection, SenseKind};
pub use engine::EngineHandle;
pub use mcu::{McuBuildError, McuBuilder, McuComponent};
pub use net::{Level, Net, NetId, NetState, Ohms, PinRef, TheveninDrive, Volts};
pub use netlist::{ComponentDecl, NetDecl, NetlistError, NodeDecl, ParsedNetlist};
pub use registry::{Classification, JumperState, PartRegistry, PassiveKind, RegistryError};
pub use system::{
    BuiltSystem, DnpState, EndpointKind, EndpointRef, Fault, Harness, HarnessConnection,
    HarnessError, Scenario, StreamDropPolicy, System, SystemError, SystemHandle,
};
