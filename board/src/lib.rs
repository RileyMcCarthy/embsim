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
//! UARTs framed onto the nets as levels ([`SerialLevelBridge`]), and step
//! clocks as periodic drives ([`Drive::Periodic`]) resolved like every other
//! drive. Both paths drive one shared resolution code path, whose
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
//! - [`component`] — [`Component`] trait, [`PinDecl`], [`PinRole`], [`Drive`], [`ComponentNetIo`]
//! - [`assembly`] — [`Assembly`]: several components as one node behind one
//!   set of pins, their wakes through the assembly, the links between them
//!   its own code (a plant made of models)
//! - [`registry`] — [`PartRegistry`]: identity → constructor; auto-classification tiers
//! - [`engine`] — the live single-writer net engine: drive queue, resolution, timer wheel
//! - [`net`] — net state model ([`NetState`]) and shared net/pin identity types
//! - [`cluster`] — analog cluster types + [`ClusterSolver`] trait ([`QuasiStaticMna`] default)
//! - [`board`] — [`Board::from_netlist`]: netlist + registry → components + nets
//! - [`system`] — [`System`]: boards + harnesses + scenario overrides + fault algebra
//! - [`survey`] — [`BoardSurvey`]: what a netlist still asks of a project — the
//!   parts with no model, the pin tables that miss, the connectors a wire may use
//! - [`project`] — [`Project`]: a system written down as a TOML file of boards,
//!   part models, wires and a scenario, built through a [`Catalog`]
//! - [`kind`] — [`KindGuide`]: what a part kind is, and the check that it
//!   seats only on a part that is what it says ([`Named`])
//! - [`report`] — [`Report`]: what a run prints about what a catalog built
//! - [`host_pty`] — [`HostPty`]: the host's end of a serial link, a PTY whose
//!   bytes are levels on two pins
//! - [`scripted_source`] — [`ScriptedSource`]: a pin driven through a list of
//!   steps at their instants
//! - [`diagnostics`] — structured [`Finding`]s on a [`Diagnostics`] collector, mirrored to `tracing`
//! - [`event_log`] — opt-in [`EventLog`]: the engine's totally-ordered event transcript
//!   (determinism Oracle 1, `DETERMINISM.md`) with its normalization contract
//! - [`uart`] — asynchronous serial framing, so a byte-oriented peripheral
//!   can put its bits on a net instead of bypassing it
//! - [`serial_levels`] — [`SerialLevelBridge`]: that codec wired to a pin, with
//!   the bit clock and frame deadlines a live UART needs

pub mod assembly;
pub mod board;
pub mod cluster;
pub mod component;
pub mod diagnostics;
pub mod engine;
pub mod event_log;
pub mod host_pty;
pub mod kind;
mod limits;
pub mod net;
pub mod netlist;
pub mod project;
pub mod registry;
pub mod report;
pub mod scripted_source;
pub mod serial_levels;
pub mod survey;
pub mod system;
pub mod uart;

pub use assembly::{Assembly, AssemblyError};
pub use board::{Board, BoardError, PartClass};
pub use cluster::{
    Cluster, ClusterElement, ClusterInjection, ClusterInputs, ClusterResistor, ClusterSolution,
    ClusterSolver, ClusterSource, ClusterTerminal, QuasiStaticMna, GMIN_OHMS,
    PWL_SOLVES_PER_ELEMENT,
};
pub use component::{
    jesd8c01_lvcmos_thresholds, AttachError, Branch, Clamp, ClampRail, Component, ComponentNetIo,
    DeadBand, DigitalReceiver, Drive, InputPort, PeriodicSchedule, PeriodicSense, PinDecl,
    PinHandle, PinLimits, PinRole, PwlCurve, RegionTest, ResistorAt, Sense, Thresholds, WakeGate,
    WakeHandler,
};
pub use diagnostics::{
    CallbackKind, Diagnostics, Finding, PinMismatchDirection, RailDownReason, SenseKind,
};
pub use engine::{ComponentId, EndpointId, EngineHandle};
pub use event_log::{EngineEvent, EngineEventRecord, EventLog};
pub use host_pty::{HostPty, HostPtyCounters};
pub use kind::{
    fitting_option_table, is_connector, is_switch, kinds_without_a_model, Fit, KindGuide, KindInfo,
    Named, OptionValues, PinTable, RequiredOption,
};
pub use net::{
    digital_drive, level_of, Amps, Level, Net, NetId, NetState, Ohms, PinRef, TheveninDrive, Volts,
    COUPLING_REACTANCE_RATIO, ESCALATION_IMPEDANCE_RATIO, V_IH, V_IL, WEAK_DRIVE_OHMS,
};
pub use netlist::{ComponentDecl, NetDecl, NetlistError, NodeDecl, ParsedNetlist};
pub use project::{
    parse_duration, state_dir, version_meets, Assignment, BoardSpec, Catalog, CatalogBoard,
    CatalogTable, ComponentRequest, ComponentSpec, ContactState, JumperSpec, KeyField, MateSpec,
    ModelSpec, PartOptions, PinShortSpec, Project, ProjectError, ProjectHead, SwitchSpec, WireSpec,
    EMBSIM_VERSION,
};
pub use registry::{
    reference_designator_class, Classification, Classified, JumperState, ModelFacade, PartRegistry,
    PassiveKind, PwlBranch, PwlSpec, RegistryError, SwitchPole,
};
pub use report::{Report, Reports};
pub use scripted_source::{ScriptedSource, Step};
pub use serial_levels::SerialLevelBridge;
pub use survey::{
    BoardSurvey, ConnectorReport, FacadeMismatch, PinSite, PinTableGroup, SurveyedPart,
    UnmodelledPart,
};
pub use system::{
    BuiltSystem, DnpState, EndpointKind, EndpointRef, Fault, Harness, HarnessConnection,
    HarnessError, Scenario, System, SystemError, SystemHandle, BUILD_FIXED_POINT_BOUND,
};
pub use uart::{FramingError, UartDecoder, UartEncoder, UartFraming};
