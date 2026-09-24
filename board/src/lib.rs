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
//! stream byte pipes (routing, baud pacing, drop policies) derived from
//! net resolution, and rate-carried pulse trains ([`PulseTrain`]) on the same
//! derived routes. Both paths drive one shared resolution code path, whose
//! projection is source-strength ranking (`NODES.md` rule 2): sources
//! reaching a node ranked by total ohms, a pull never contending, a ten
//! times weaker source losing with a [`Finding::Contention`], comparable
//! sources solved and projected through the [`net::V_IL`]/[`net::V_IH`]
//! dead band ([`Finding::AmbiguousLevel`]). Still deferred to later slices:
//! live topology mutation and its epoch notification (the seam is
//! registered), and transducer primitives.
//!
//! Module map (mirrors the design doc's crate layout):
//! - [`netlist`] — KiCad s-expression netlist parser → [`ComponentDecl`]/[`NetDecl`] graph
//! - [`component`] — [`Component`] trait, [`PinDecl`], [`PinKind`], [`StreamRole`], [`PulseTrain`], [`ComponentNetIo`]
//! - [`registry`] — [`PartRegistry`]: identity → constructor; auto-classification tiers
//! - [`engine`] — the live single-writer net engine: drive queue, resolution, timer wheel, stream routing
//! - [`net`] — net state model ([`NetState`]) and shared net/pin identity types
//! - [`cluster`] — analog cluster types + [`ClusterSolver`] trait ([`QuasiStaticMna`] default)
//! - [`board`] — [`Board::from_netlist`]: netlist + registry → components + nets
//! - [`system`] — [`System`]: boards + harnesses + scenario overrides + fault algebra
//! - [`diagnostics`] — structured [`Finding`]s on a [`Diagnostics`] collector, mirrored to `tracing`
//! - [`event_log`] — opt-in [`EventLog`]: the engine's totally-ordered event transcript
//!   (determinism Oracle 1, `DETERMINISM.md`) with its normalization contract
//! - [`mcu`] — [`McuComponent`]: the MCU as a component — serial, GPIO,
//!   pulse-out and encoder channels bridged to physical pins
//! - [`uart`] — asynchronous serial framing, so a byte-oriented peripheral
//!   can put its bits on a net instead of bypassing it
//! - [`serial_levels`] — [`SerialLevelBridge`]: that codec wired to a pin, with
//!   the bit clock and frame deadlines a live UART needs

pub mod board;
pub mod cluster;
pub mod component;
pub mod diagnostics;
pub mod engine;
pub mod event_log;
pub mod host_pty;
pub mod mcu;
pub mod net;
pub mod netlist;
pub mod registry;
pub mod serial_levels;
pub mod system;
pub mod uart;

pub use board::{Board, BoardError, PartClass};
pub use cluster::{
    Cluster, ClusterElement, ClusterInjection, ClusterInputs, ClusterResistor, ClusterSolution,
    ClusterSolver, ClusterSource, ClusterTerminal, QuasiStaticMna, GMIN_OHMS,
    PWL_SOLVES_PER_ELEMENT,
};
pub use component::{
    AttachError, Branch, Component, ComponentNetIo, Drive, IdleDrive, PinDecl, PinHandle, PinKind,
    PulseDirection, PulseSegment, PulseTrain, PulseTx, PwlCurve, RegionTest, StreamRole,
};
pub use diagnostics::{CallbackKind, Diagnostics, Finding, PinMismatchDirection, SenseKind};
pub use engine::{ComponentId, EndpointId, EngineHandle};
pub use event_log::{EngineEvent, EngineEventRecord, EventLog};
pub use host_pty::HostPty;
pub use mcu::{McuBuildError, McuBuilder, McuComponent};
pub use net::{
    digital_drive, level_of, Amps, Level, Net, NetId, NetState, Ohms, PinRef, TheveninDrive, Volts,
    COUPLING_REACTANCE_RATIO, ESCALATION_IMPEDANCE_RATIO, V_IH, V_IL, WEAK_DRIVE_OHMS,
};
pub use netlist::{ComponentDecl, NetDecl, NetlistError, NodeDecl, ParsedNetlist};
pub use registry::{
    reference_designator_class, Classification, JumperState, PartRegistry, PassiveKind, PwlBranch,
    PwlSpec, RegistryError, SwitchPole,
};
pub use serial_levels::SerialLevelBridge;
pub use system::{
    BuiltSystem, DnpState, EndpointKind, EndpointRef, Fault, Harness, HarnessConnection,
    HarnessError, Scenario, System, SystemError, SystemHandle, BUILD_FIXED_POINT_BOUND,
};
pub use uart::{FramingError, UartDecoder, UartEncoder, UartFraming};
