//! Model: incremental quadrature encoder — a position input turned into
//! Gray-coded A/B (and optionally index Z) pin transitions.
//!
//! This is the mirror image of [`crate::machine::stepper_motor`]: the drive
//! *consumes* logic edges, the encoder *produces* them. Feed it a position and
//! it walks its output pins through the quadrature sequence the MCU's encoder
//! peripheral counts:
//!
//! ```text
//!   set_position_mm(x) ──► count = round(x · counts_per_mm)
//!                          │
//!                          └─► one A/B transition per count, in order:
//!                                A ┐   ┌───┐   ┌───┐
//!                                  └───┘   └───┘        (A leads B)
//!                                B ─┐   ┌───┐   ┌──
//!                                   └───┘   └───┘
//!                              count  0  1  2  3  0 …
//! ```
//!
//! # Provenance (mechanism model — no datasheet)
//!
//! ## Governing behavior
//!
//! An incremental quadrature encoder emits two square waves 90° out of phase.
//! Their 2-bit code is **Gray**: exactly one channel changes per count, which
//! is what makes ×4 decoding unambiguous and what lets a counter recover
//! direction from the order of the changes. The sequence for increasing count
//! with A leading B is
//!
//! ```text
//!   count mod 4 :  0      1      2      3
//!   (A, B)      : (0,0)  (1,0)  (1,1)  (0,1)
//! ```
//!
//! and decreasing count walks it backwards. One count is one *state*
//! transition, so `counts_per_mm` is already the ×4-decoded resolution — the
//! same number the firmware's steps/mm constant uses.
//!
//! The optional index (`Z`) channel asserts for
//! [`IndexConfig::width_counts`] counts once every
//! [`IndexConfig::counts_per_revolution`] counts, the once-per-revolution
//! marker a homing routine can latch on.
//!
//! ## Parameter sources
//!
//! `counts_per_mm`, `counts_per_revolution`, and the channel order are rig
//! geometry from the consuming system description, not part constants. For the
//! reference consumer they must match the firmware's own scale (MaD:
//! 4 microsteps × 2048 steps/rev = 8192 counts/mm, `STEPS_PER_MM` in
//! `SIL/MaDSim/src/wiring.rs`).
//!
//! ## The output stage
//!
//! What drives each output is the encoder's output stage
//! ([`Config::output`], [`OutputStage`]; `NODES.md` §20). Its two numbers
//! are the configuration's, [`Config::high_volts`] and
//! [`Config::drive_impedance_ohms`], whose defaults are embsim's stated bench
//! defaults (3.3 V and 25 Ω, [`DEFAULT_HIGH_VOLTS`] and
//! `embsim_board::net::DEFAULT_PUSH_PULL_IMPEDANCE`): no encoder datasheet
//! is behind them, and a machine whose encoder names its own sets them.
//!
//! - [`OutputStage::PushPull`] (the default): one output per channel, `A`,
//!   `B` and with an index `Z`, driven to `high_volts` or 0 V behind
//!   `drive_impedance_ohms`. This is the logic-level side of a receiver: a
//!   system description wires `ENCODER.A` to whatever net carries the
//!   encoder's `A` at logic level.
//! - [`OutputStage::Rs422`] ([`Config::with_complements`]): an RS-422 line
//!   driver, each channel a pair, `A` and `A-`, `B` and `B-`, and `Z` and
//!   `Z-` with an index, the `-` leg driven as the inverse of its leg in the
//!   same publish, each push-pull at the same rail and impedance. A
//!   differential receiver reads the pair as a valid difference either way.
//! - [`OutputStage::NpnOpenCollector`]: an NPN open collector per channel,
//!   as MaD's machine's encoder presents them (a Taiss 600 P/R optical
//!   encoder: "NPN open collector output", A and B, no complements, no
//!   index, a 5–24 V supply). While its phase is low the transistor sinks
//!   the output to the encoder's ground, 0 V behind `drive_impedance_ohms`;
//!   while it is high the transistor is off and the output is released, so
//!   its high is whatever the board pulls it to. The stage has no pull-up
//!   of its own (none is named by the Taiss listing, which says the outputs
//!   must not be tied straight to the supply): with none on the board, a
//!   released output floats, and the build's
//!   `embsim_board::Finding::OpenDrainWithoutPullUp` names it.
//!
//! ## The encoder through a differential receiver
//!
//! MaD's Edge carrier receives the encoder on `J20` with an AM26LV32 quad
//! differential line receiver (`U25`, `embsim_models::am26lv32`) before it
//! reaches the P2 — see `Hardware/EdgeBoard/KiCad/MaD_Edge_Sheet3.kicad_sch`
//! (the AM26LS31 driver `U24` on the same sheet handles the outbound
//! step/direction pairs). The receiver reads each pair's difference against
//! ±200 mV (TI SLLS202H §6.5). The line driver's pairs read as the encoder's
//! levels. A single-ended output — push-pull or open collector — on `A+`
//! with `A-` at the encoder's ground gives the pair no difference for its
//! low: Table 8-1's "?" with a common-mode voltage applied, which the
//! receiver's model reads as its fail-safe high (§8.4.1) and the engine
//! reports as `embsim_board::Finding::PairInBand`. On today's Edge board an
//! open-collector encoder's `P9` and `P10` therefore stay high at every
//! count; a pull-up on each `+` to the encoder's supply and each `-` held
//! between the output's two levels would read it.
//!
//! ## Not modeled
//!
//! - **The driver behind an output** beyond the stage: the levels are
//!   `0 V` and [`Config::high_volts`] behind one impedance; no common-mode
//!   offset, no single-ended-leg fault, no saturation voltage or sink
//!   current limit on the open collector, no leakage while it is off.
//! - **The encoder's supply.** The model has no supply pin: its outputs
//!   drive and sink whatever the board does to the encoder's power.
//! - **Quadrature error**: no phase error between channels, no duty-cycle
//!   error, no jitter, no line-count tolerance. Every transition is exactly one
//!   count wide.
//! - **Pacing.** The walk between two positions is emitted as fast as the
//!   engine drains its drive queue, not at the velocity implied by the motion.
//!   Consumers that count edges see every one, in the right order, with the
//!   right direction; consumers that *time* edges to recover velocity would
//!   read the queue's rate, not the machine's.
//! - **Teleporting position.** A jump larger than
//!   [`Config::max_counts_per_update`] snaps to the target state instead of
//!   walking, and counts a
//!   [`EncoderInput::snapped_updates`] event. Real shafts cannot teleport; the
//!   cap exists because a system description can (a homing routine calling
//!   `set_position` re-anchors the coordinate frame), and flooding the engine's
//!   drive queue with a million transitions is not a better failure.
//! - **Index alignment to a mechanical home**: the index window is purely
//!   `count mod counts_per_revolution`, phased on count 0.

use std::fmt;
use std::sync::{Arc, Mutex};

use embsim_board::net::DEFAULT_PUSH_PULL_IMPEDANCE;
use embsim_board::{
    AttachError, Component, ComponentNetIo, Drive, Level, Ohms, PinDecl, PinHandle, TheveninDrive,
    Volts,
};
use embsim_core::event::Observers;

use super::{invert, require_positive, MachineConfigError, DEFAULT_HIGH_VOLTS};

// ============================================================
// Configuration
// ============================================================

/// Default ceiling on transitions emitted by one position update.
///
/// One full revolution of a 2048-line encoder is 8192 ×4 counts, so this
/// absorbs a whole turn of continuous motion while still catching a
/// coordinate-frame re-anchor.
pub const DEFAULT_MAX_COUNTS_PER_UPDATE: u32 = 8_192;

/// Which channel leads for increasing count — the direction convention.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChannelOrder {
    /// A leads B by 90° as the count increases (the common convention).
    ALeadsB,
    /// B leads A as the count increases (a reversed shaft or swapped pair).
    BLeadsA,
}

/// What drives each of the encoder's outputs (see the module docs, "The
/// output stage").
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum OutputStage {
    /// One push-pull output per channel: [`Config::high_volts`] or 0 V
    /// behind [`Config::drive_impedance_ohms`].
    #[default]
    PushPull,
    /// An RS-422 line driver: each channel a pair, its `-` leg driven to
    /// the inverse of its leg in the same publish, each push-pull at the
    /// same rail and impedance.
    Rs422,
    /// An NPN open collector per channel: sinking to 0 V behind
    /// [`Config::drive_impedance_ohms`] while the phase is low, released
    /// while it is high. No pull-up of its own.
    NpnOpenCollector,
}

impl OutputStage {
    /// Every stage, in the order [`Self::NAMES`] names them.
    pub const ALL: [Self; 3] = [Self::PushPull, Self::Rs422, Self::NpnOpenCollector];

    /// The stages' names, as a kind's option takes them
    /// (`embsim_board::PartOptions::choice`): `push-pull`, `rs422`,
    /// `npn-open-collector`.
    pub const NAMES: [&'static str; 3] = ["push-pull", "rs422", "npn-open-collector"];

    /// The stage's name ([`Self::NAMES`]).
    pub const fn name(self) -> &'static str {
        match self {
            Self::PushPull => Self::NAMES[0],
            Self::Rs422 => Self::NAMES[1],
            Self::NpnOpenCollector => Self::NAMES[2],
        }
    }

    /// The stage a name names ([`Self::NAMES`]), or `None`.
    pub fn from_name(name: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|stage| stage.name() == name)
    }

    /// Whether each channel has a `-` leg.
    pub const fn has_complements(self) -> bool {
        matches!(self, Self::Rs422)
    }

    /// The drive one output presents for `level`, from `rail` behind
    /// `impedance`: a push-pull port either way, or for the open collector
    /// a sink to 0 V while low and released (`None`) while high.
    pub const fn port(self, level: Level, rail: Volts, impedance: Ohms) -> Option<TheveninDrive> {
        match (self, level) {
            (Self::NpnOpenCollector, Level::High) => None,
            (_, Level::High) => Some(TheveninDrive {
                volts: rail,
                impedance,
            }),
            (_, Level::Low) => Some(TheveninDrive {
                volts: 0.0,
                impedance,
            }),
        }
    }
}

/// Optional once-per-revolution index (`Z`) channel. Declaring it adds a `Z`
/// pin to the facade; omitting it means the component has no `Z` pin at all.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IndexConfig {
    /// Counts in one shaft revolution (×4-decoded).
    pub counts_per_revolution: u32,
    /// Width of the index window in counts, starting at count 0 of each
    /// revolution.
    pub width_counts: u32,
    /// Level the index channel asserts to.
    pub active_level: Level,
}

/// Quadrature encoder configuration. Build with [`Config::new`] and override
/// the fields a particular machine needs.
#[derive(Debug, Clone)]
pub struct Config {
    /// ×4-decoded counts per millimetre of carriage travel. Rig geometry; must
    /// match the firmware's own scale.
    pub counts_per_mm: f64,
    /// Direction convention.
    pub order: ChannelOrder,
    /// Index channel, or `None` for a plain A/B encoder.
    pub index: Option<IndexConfig>,
    /// Open-circuit voltage a push-pull output drives for a logic high
    /// ([`DEFAULT_HIGH_VOLTS`]); the open collector drives no high.
    pub high_volts: Volts,
    /// Thevenin impedance of the outputs: a push-pull driver's at both
    /// levels, the open collector's sink while it is low
    /// (`embsim_board::net::DEFAULT_PUSH_PULL_IMPEDANCE`).
    pub drive_impedance_ohms: Ohms,
    /// Transitions one position update may emit before snapping
    /// ([`DEFAULT_MAX_COUNTS_PER_UPDATE`]).
    pub max_counts_per_update: u32,
    /// What drives each output ([`OutputStage`], push-pull by default).
    pub output: OutputStage,
}

impl Config {
    /// Configuration at the given `counts_per_mm`, A leading B, no index
    /// channel, 3.3 V push-pull outputs.
    pub fn new(counts_per_mm: f64) -> Self {
        Self {
            counts_per_mm,
            order: ChannelOrder::ALeadsB,
            index: None,
            high_volts: DEFAULT_HIGH_VOLTS,
            drive_impedance_ohms: DEFAULT_PUSH_PULL_IMPEDANCE,
            max_counts_per_update: DEFAULT_MAX_COUNTS_PER_UPDATE,
            output: OutputStage::PushPull,
        }
    }

    /// Drive the outputs through `stage`.
    pub fn with_output(mut self, stage: OutputStage) -> Self {
        self.output = stage;
        self
    }

    /// Add an index channel (and its `Z` pin).
    pub fn with_index(mut self, index: IndexConfig) -> Self {
        self.index = Some(index);
        self
    }

    /// Make every channel a complementary pair, as an RS-422 encoder's line
    /// driver presents it ([`OutputStage::Rs422`]): `A-`, `B-` (and `Z-`
    /// with an index) are declared after the single-ended pins, each driven
    /// to the inverse of its leg's level in the same publish, so the pair's
    /// differential is `±high_volts` once attached.
    pub fn with_complements(self) -> Self {
        self.with_output(OutputStage::Rs422)
    }

    /// Reject a configuration that cannot describe an encoder.
    fn validate(&self) -> Result<(), MachineConfigError> {
        require_positive("counts_per_mm", self.counts_per_mm)?;
        require_positive("high_volts", self.high_volts)?;
        require_positive("drive_impedance_ohms", self.drive_impedance_ohms)?;
        if self.max_counts_per_update == 0 {
            return Err(MachineConfigError::Zero {
                field: "max_counts_per_update",
            });
        }
        if let Some(index) = &self.index {
            if index.counts_per_revolution == 0 {
                return Err(MachineConfigError::Zero {
                    field: "counts_per_revolution",
                });
            }
            if index.width_counts == 0 {
                return Err(MachineConfigError::Zero {
                    field: "width_counts",
                });
            }
        }
        Ok(())
    }
}

// ============================================================
// Pin facade
// ============================================================

/// One declared output channel through `stage`: push-pull, idling high
/// behind the configured impedance until the encoder drives it, or an open
/// collector, which sinks, cannot source and rests released.
fn output(number: &'static str, stage: OutputStage, impedance: Ohms) -> PinDecl {
    match stage {
        OutputStage::NpnOpenCollector => PinDecl::digital_out(number).sink_only(),
        OutputStage::PushPull | OutputStage::Rs422 => {
            PinDecl::digital_out(number).with_impedance(impedance)
        }
    }
}

// ============================================================
// Wire state
// ============================================================

/// The encoder's count and the levels currently driven onto its pins.
#[derive(Debug)]
struct Wire {
    /// ×4-decoded count — the encoder's own position truth.
    count: i64,
    /// Output pin handles, `None` until attach.
    a: Option<PinHandle>,
    b: Option<PinHandle>,
    z: Option<PinHandle>,
    /// The complementary legs, `None` until attach or for a stage without
    /// them ([`OutputStage::has_complements`]).
    a_n: Option<PinHandle>,
    b_n: Option<PinHandle>,
    z_n: Option<PinHandle>,
    /// Last levels actually driven, so unchanged channels enqueue nothing.
    driven: Option<(Level, Level, Level)>,
    /// Position updates that exceeded [`Config::max_counts_per_update`] and
    /// snapped instead of walking.
    snapped: u64,
}

/// Shared wire state + observers behind the component and every
/// [`EncoderInput`].
struct EncoderCore {
    config: Config,
    wire: Mutex<Wire>,
    on_count: Observers<i64>,
}

impl fmt::Debug for EncoderCore {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("EncoderCore")
            .field("config", &self.config)
            .field("wire", &self.wire)
            .field("observers", &self.on_count.len())
            .finish()
    }
}

impl EncoderCore {
    /// Gray-coded `(A, B)` for a count, honoring the channel order.
    fn phase(&self, count: i64) -> (Level, Level) {
        let (a, b) = match count.rem_euclid(4) {
            0 => (Level::Low, Level::Low),
            1 => (Level::High, Level::Low),
            2 => (Level::High, Level::High),
            _ => (Level::Low, Level::High),
        };
        match self.config.order {
            ChannelOrder::ALeadsB => (a, b),
            ChannelOrder::BLeadsA => (b, a),
        }
    }

    /// Index channel level for a count (inactive when no index is configured).
    fn index_level(&self, count: i64) -> Level {
        let Some(index) = &self.config.index else {
            return Level::Low;
        };
        let revolution = i64::from(index.counts_per_revolution);
        if count.rem_euclid(revolution) < i64::from(index.width_counts) {
            index.active_level
        } else {
            invert(index.active_level)
        }
    }

    /// Drive one pin to a level through the output stage: a Thevenin port
    /// at the configured rail and impedance, or the open collector released
    /// for a high ([`OutputStage::port`]).
    fn drive(&self, handle: &Option<PinHandle>, level: Level) {
        let Some(handle) = handle else {
            return;
        };
        let config = &self.config;
        match config
            .output
            .port(level, config.high_volts, config.drive_impedance_ohms)
        {
            Some(port) => handle.drive(Drive::Thevenin(port)),
            None => handle.release(),
        }
    }

    /// Drive a channel: its leg to `level` and, with complements
    /// ([`OutputStage::Rs422`]), its complement to the inverse, together.
    fn drive_pair(&self, leg: &Option<PinHandle>, complement: &Option<PinHandle>, level: Level) {
        self.drive(leg, level);
        self.drive(complement, invert(level));
    }

    /// Publish the current count's levels, enqueueing only the channels that
    /// actually changed. `force` re-drives everything (used at attach, where
    /// the engine has assigned each output its declared idle).
    fn publish(&self, wire: &mut Wire, force: bool) {
        let (a, b) = self.phase(wire.count);
        let z = self.index_level(wire.count);
        let previous = wire.driven;
        if force || previous.is_none_or(|(pa, _, _)| pa != a) {
            self.drive_pair(&wire.a, &wire.a_n, a);
        }
        if force || previous.is_none_or(|(_, pb, _)| pb != b) {
            self.drive_pair(&wire.b, &wire.b_n, b);
        }
        if wire.z.is_some() && (force || previous.is_none_or(|(_, _, pz)| pz != z)) {
            self.drive_pair(&wire.z, &wire.z_n, z);
        }
        wire.driven = Some((a, b, z));
    }

    /// Move to `target`, walking one count at a time so every intermediate
    /// Gray transition reaches the net (a counter on the other side must see
    /// each edge), or snapping when the jump exceeds the configured cap.
    fn set_count(&self, target: i64) {
        let count = {
            let mut wire = self.wire.lock().unwrap();
            if target == wire.count {
                return;
            }
            let delta = target - wire.count;
            let cap = i64::from(self.config.max_counts_per_update);
            if delta.abs() > cap {
                tracing::debug!(
                    from = wire.count,
                    to = target,
                    cap,
                    "quadrature_encoder: position jump exceeds the walk cap; snapping"
                );
                wire.snapped += 1;
                wire.count = target;
                self.publish(&mut wire, false);
            } else {
                let step = delta.signum();
                for _ in 0..delta.abs() {
                    wire.count += step;
                    self.publish(&mut wire, false);
                }
            }
            wire.count
        };
        // Observers run with the wire lock released, so a subscriber may read
        // the encoder back without deadlocking.
        self.on_count.emit(count);
    }
}

// ============================================================
// Input handle
// ============================================================

/// Drive-side handle to a [`QuadratureEncoder`].
///
/// Cloned out of the component *before* it is handed to
/// [`embsim_board::System`], then used to feed it position — typically wired
/// straight to a motor's observer:
///
/// ```rust,ignore
/// let input = encoder.input();
/// motor.shaft().on_position_change(move |mm| input.set_position_mm(mm));
/// ```
#[derive(Clone, Debug)]
pub struct EncoderInput {
    core: Arc<EncoderCore>,
}

impl EncoderInput {
    /// Move the encoder to the count implied by a carriage position in
    /// millimetres (`round(mm · counts_per_mm)`).
    ///
    /// A non-finite position is ignored with a trace: the shaft has no
    /// defensible count, and inventing one would corrupt the count the
    /// firmware closes its loop on.
    pub fn set_position_mm(&self, position_mm: f64) {
        let counts = position_mm * self.core.config.counts_per_mm;
        if !counts.is_finite() {
            tracing::warn!(
                position_mm,
                "quadrature_encoder: non-finite position ignored"
            );
            return;
        }
        self.set_position_counts(counts.round() as i64);
    }

    /// Move the encoder directly to a ×4-decoded count.
    pub fn set_position_counts(&self, count: i64) {
        self.core.set_count(count);
    }

    /// The encoder's current count.
    pub fn count(&self) -> i64 {
        self.core.wire.lock().unwrap().count
    }

    /// Current carriage position implied by the count, in millimetres.
    pub fn position_mm(&self) -> f64 {
        self.count() as f64 / self.core.config.counts_per_mm
    }

    /// Levels currently driven as `(A, B, Z)` — `Z` is meaningless without a
    /// configured index channel. `None` before the component has attached.
    pub fn driven_levels(&self) -> Option<(Level, Level, Level)> {
        self.core.wire.lock().unwrap().driven
    }

    /// How many position updates snapped instead of walking (see the module
    /// docs' "teleporting position" note). A non-zero count in a test means the
    /// system description slewed position discontinuously.
    pub fn snapped_updates(&self) -> u64 {
        self.core.wire.lock().unwrap().snapped
    }

    /// Subscribe to the encoder's count after each position update. Multiple
    /// subscribers are appended, never overwritten.
    pub fn on_count_change(&self, callback: impl Fn(i64) + Send + 'static) {
        self.core.on_count.subscribe(callback);
    }
}

// ============================================================
// Component
// ============================================================

/// An incremental quadrature encoder as a live board-engine component.
///
/// Its pins are outputs: `A`, `B`, and — only when an [`IndexConfig`] is
/// configured — `Z`; with [`Config::with_complements`], `A-`, `B-` (and
/// `Z-`) after them. Each is driven through the configured
/// [`OutputStage`]. Attach drives the initial phase, so the count and the
/// nets agree before any traffic.
#[derive(Debug)]
pub struct QuadratureEncoder {
    core: Arc<EncoderCore>,
    pins: Vec<PinDecl>,
}

impl QuadratureEncoder {
    /// Create an encoder from a validated configuration.
    pub fn new(config: Config) -> Result<Self, MachineConfigError> {
        config.validate()?;
        let (stage, impedance) = (config.output, config.drive_impedance_ohms);
        let channel = |number| output(number, stage, impedance);
        let mut pins = vec![channel("A"), channel("B")];
        if config.index.is_some() {
            pins.push(channel("Z"));
        }
        if stage.has_complements() {
            pins.push(channel("A-"));
            pins.push(channel("B-"));
            if config.index.is_some() {
                pins.push(channel("Z-"));
            }
        }
        tracing::info!(
            counts_per_mm = config.counts_per_mm,
            index = config.index.is_some(),
            output = stage.name(),
            "quadrature_encoder: init"
        );
        Ok(Self {
            core: Arc::new(EncoderCore {
                config,
                wire: Mutex::new(Wire {
                    count: 0,
                    a: None,
                    b: None,
                    z: None,
                    a_n: None,
                    b_n: None,
                    z_n: None,
                    driven: None,
                    snapped: 0,
                }),
                on_count: Observers::new(),
            }),
            pins,
        })
    }

    /// A handle for feeding this encoder position.
    pub fn input(&self) -> EncoderInput {
        EncoderInput {
            core: Arc::clone(&self.core),
        }
    }

    /// The validated configuration in force.
    pub fn config(&self) -> &Config {
        &self.core.config
    }
}

impl Component for QuadratureEncoder {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let mut wire = self.core.wire.lock().unwrap();
        wire.a = Some(io.pin("A")?);
        wire.b = Some(io.pin("B")?);
        if self.core.config.index.is_some() {
            wire.z = Some(io.pin("Z")?);
        }
        if self.core.config.output.has_complements() {
            wire.a_n = Some(io.pin("A-")?);
            wire.b_n = Some(io.pin("B-")?);
            if self.core.config.index.is_some() {
                wire.z_n = Some(io.pin("Z-")?);
            }
        }
        // Force the initial phase: a push-pull channel idles high from
        // assembly and an open collector released, neither of which is the
        // encoder's count-0 state, so all channels are re-driven here
        // rather than diffed.
        self.core.publish(&mut wire, true);
        Ok(())
    }
}

// ============================================================
// Tests
// ============================================================
//
// The Gray-code walk is exercised here against the wire state directly (no
// engine, so no pin handles: `publish` is a no-op on the nets but keeps the
// count and `driven` bookkeeping). Live edge ordering as observed by a peer
// component is covered by `models/tests/machine_live_system.rs`.

#[cfg(test)]
mod tests {
    use rstest::rstest;

    use super::*;

    fn encoder(config: Config) -> QuadratureEncoder {
        QuadratureEncoder::new(config).expect("valid config")
    }

    /// Walk the whole 4-state cycle and collect the `(A, B)` pairs, count 0
    /// first.
    fn cycle(order: ChannelOrder) -> Vec<(Level, Level)> {
        let enc = encoder(Config {
            order,
            ..Config::new(1.0)
        });
        (0..4).map(|count| enc.core.phase(count)).collect()
    }

    // ========================================================
    // Gray code
    // ========================================================

    /// The quadrature sequence for increasing count, A leading B.
    #[rstest]
    fn a_leads_b_walks_the_standard_gray_sequence() {
        assert_eq!(
            cycle(ChannelOrder::ALeadsB),
            vec![
                (Level::Low, Level::Low),
                (Level::High, Level::Low),
                (Level::High, Level::High),
                (Level::Low, Level::High),
            ]
        );
    }

    /// The reversed convention is the same sequence with the channels swapped,
    /// so B changes first.
    #[rstest]
    fn b_leads_a_swaps_the_channels() {
        assert_eq!(
            cycle(ChannelOrder::BLeadsA),
            vec![
                (Level::Low, Level::Low),
                (Level::Low, Level::High),
                (Level::High, Level::High),
                (Level::High, Level::Low),
            ]
        );
    }

    /// The defining Gray property: exactly one channel changes per count, in
    /// both directions and across the wrap. This is what makes ×4 decoding
    /// unambiguous.
    #[rstest]
    #[case::a_leads(ChannelOrder::ALeadsB)]
    #[case::b_leads(ChannelOrder::BLeadsA)]
    fn exactly_one_channel_changes_per_count(#[case] order: ChannelOrder) {
        let enc = encoder(Config {
            order,
            ..Config::new(1.0)
        });
        for count in -9i64..=9 {
            let (a0, b0) = enc.core.phase(count);
            let (a1, b1) = enc.core.phase(count + 1);
            let changes = u8::from(a0 != a1) + u8::from(b0 != b1);
            assert_eq!(
                changes,
                1,
                "count {count} -> {} changed {changes} channels",
                count + 1
            );
        }
    }

    /// Negative counts walk the same cycle (`rem_euclid`, not `%`), so
    /// travelling backwards through zero does not glitch the code.
    #[rstest]
    fn negative_counts_stay_on_the_cycle() {
        let enc = encoder(Config::new(1.0));
        assert_eq!(enc.core.phase(-4), enc.core.phase(0));
        assert_eq!(enc.core.phase(-1), enc.core.phase(3));
        assert_eq!(enc.core.phase(-3), enc.core.phase(1));
    }

    // ========================================================
    // Position → count
    // ========================================================

    /// Position in millimetres scales by `counts_per_mm` and rounds to the
    /// nearest count.
    #[rstest]
    #[case::zero(0.0, 0)]
    #[case::one_mm(1.0, 8_192)]
    #[case::half_mm(0.5, 4_096)]
    #[case::negative(-0.25, -2_048)]
    #[case::rounds_up(0.000_061_1, 1)]
    fn position_scales_and_rounds(#[case] mm: f64, #[case] expect: i64) {
        let enc = encoder(Config {
            max_counts_per_update: u32::MAX,
            ..Config::new(8_192.0)
        });
        let input = enc.input();
        input.set_position_mm(mm);
        assert_eq!(input.count(), expect);
        assert!((input.position_mm() - expect as f64 / 8_192.0).abs() < f64::EPSILON);
    }

    /// A non-finite position is refused rather than corrupting the count.
    #[rstest]
    #[case::nan(f64::NAN)]
    #[case::infinite(f64::INFINITY)]
    fn non_finite_position_is_ignored(#[case] mm: f64) {
        let enc = encoder(Config::new(8_192.0));
        let input = enc.input();
        input.set_position_counts(17);
        input.set_position_mm(mm);
        assert_eq!(input.count(), 17, "the previous count must survive");
    }

    /// A walk emits one transition per count and lands on the target's phase.
    #[rstest]
    #[case::forward(7)]
    #[case::backward(-7)]
    #[case::one(1)]
    fn walking_lands_on_the_target_phase(#[case] target: i64) {
        let enc = encoder(Config::new(1.0));
        let input = enc.input();
        input.set_position_counts(target);
        assert_eq!(input.count(), target);
        let (a, b) = enc.core.phase(target);
        assert_eq!(input.driven_levels(), Some((a, b, Level::Low)));
        assert_eq!(input.snapped_updates(), 0, "no snap within the cap");
    }

    /// A jump beyond the cap snaps to the target phase and is counted, rather
    /// than flooding the engine's drive queue.
    #[rstest]
    fn oversized_jumps_snap_and_are_counted() {
        let enc = encoder(Config {
            max_counts_per_update: 10,
            ..Config::new(1.0)
        });
        let input = enc.input();
        input.set_position_counts(10); // exactly at the cap: still walks
        assert_eq!(input.snapped_updates(), 0);
        input.set_position_counts(1_000); // 990 counts: snaps
        assert_eq!(input.count(), 1_000);
        assert_eq!(input.snapped_updates(), 1);
        let (a, b) = enc.core.phase(1_000);
        assert_eq!(input.driven_levels(), Some((a, b, Level::Low)));
    }

    /// Setting the position it already holds is a no-op — no transitions, no
    /// observer traffic.
    #[rstest]
    fn idempotent_update_publishes_nothing() {
        let enc = encoder(Config::new(1.0));
        let input = enc.input();
        let seen = Arc::new(Mutex::new(Vec::new()));
        {
            let sink = Arc::clone(&seen);
            input.on_count_change(move |c| sink.lock().unwrap().push(c));
        }
        input.set_position_counts(5);
        input.set_position_counts(5);
        assert_eq!(*seen.lock().unwrap(), vec![5]);
    }

    // ========================================================
    // Index channel
    // ========================================================

    /// The index asserts for `width_counts` counts once per revolution, phased
    /// on count 0, and wraps in both directions.
    #[rstest]
    #[case::at_zero(0, true)]
    #[case::inside_window(1, true)]
    #[case::just_past_window(2, false)]
    #[case::mid_revolution(50, false)]
    #[case::next_revolution(100, true)]
    #[case::previous_revolution(-100, true)]
    #[case::just_before_zero(-1, false)]
    fn index_window_marks_each_revolution(#[case] count: i64, #[case] active: bool) {
        let enc = encoder(Config::new(1.0).with_index(IndexConfig {
            counts_per_revolution: 100,
            width_counts: 2,
            active_level: Level::High,
        }));
        let expect = if active { Level::High } else { Level::Low };
        assert_eq!(enc.core.index_level(count), expect);
    }

    /// The index polarity is configuration, so an active-low index inverts
    /// both states.
    #[rstest]
    fn index_polarity_is_configurable() {
        let enc = encoder(Config::new(1.0).with_index(IndexConfig {
            counts_per_revolution: 100,
            width_counts: 1,
            active_level: Level::Low,
        }));
        assert_eq!(enc.core.index_level(0), Level::Low);
        assert_eq!(enc.core.index_level(1), Level::High);
    }

    /// Without an index there is no `Z` pin at all; with one there is exactly
    /// one more.
    #[rstest]
    fn index_declaration_controls_the_facade() {
        let plain = encoder(Config::new(1.0));
        assert_eq!(
            plain.pins().iter().map(|p| p.number).collect::<Vec<_>>(),
            vec!["A", "B"]
        );
        let indexed = encoder(Config::new(1.0).with_index(IndexConfig {
            counts_per_revolution: 4,
            width_counts: 1,
            active_level: Level::High,
        }));
        assert_eq!(
            indexed.pins().iter().map(|p| p.number).collect::<Vec<_>>(),
            vec!["A", "B", "Z"]
        );
    }

    /// Complements add one `-` pin per channel after the single-ended ones,
    /// the index's included.
    #[rstest]
    fn complements_declare_a_minus_leg_per_channel() {
        let pair = encoder(Config::new(1.0).with_complements());
        assert_eq!(
            pair.pins().iter().map(|p| p.number).collect::<Vec<_>>(),
            vec!["A", "B", "A-", "B-"]
        );
        let indexed = encoder(
            Config::new(1.0)
                .with_index(IndexConfig {
                    counts_per_revolution: 4,
                    width_counts: 1,
                    active_level: Level::High,
                })
                .with_complements(),
        );
        assert_eq!(
            indexed.pins().iter().map(|p| p.number).collect::<Vec<_>>(),
            vec!["A", "B", "Z", "A-", "B-", "Z-"]
        );
    }

    /// Every declared channel is a push-pull output at the configured
    /// impedance and carries no stream role.
    #[rstest]
    fn declared_channels_are_push_pull_outputs() {
        let enc = encoder(Config {
            drive_impedance_ohms: 47.0,
            ..Config::new(1.0)
        });
        for decl in enc.pins() {
            assert!(decl.drives());
            assert_eq!(decl.idle.map(|idle| idle.impedance), Some(47.0));
        }
    }

    /// The open collector declares the same channels as the push-pull
    /// stage, each a sink that cannot source and rests released; it has no
    /// `-` legs.
    #[rstest]
    fn open_collector_channels_sink_and_rest_released() {
        let enc = encoder(
            Config::new(1.0)
                .with_index(IndexConfig {
                    counts_per_revolution: 4,
                    width_counts: 1,
                    active_level: Level::High,
                })
                .with_output(OutputStage::NpnOpenCollector),
        );
        assert_eq!(
            enc.pins().iter().map(|p| p.number).collect::<Vec<_>>(),
            vec!["A", "B", "Z"]
        );
        for decl in enc.pins() {
            assert!(decl.can_sink && !decl.can_source, "{}", decl.number);
            assert_eq!(decl.idle, None, "{}", decl.number);
        }
    }

    /// Each stage's port for each level: push-pull and the line driver at
    /// the rail or 0 V behind the impedance; the open collector a sink at
    /// 0 V behind it, and released for a high.
    #[rstest]
    #[case::push_pull(OutputStage::PushPull, Some(5.0))]
    #[case::rs422(OutputStage::Rs422, Some(5.0))]
    #[case::open_collector(OutputStage::NpnOpenCollector, None)]
    fn each_stage_presents_its_port_at_both_levels(
        #[case] stage: OutputStage,
        #[case] high: Option<Volts>,
    ) {
        let low = stage.port(Level::Low, 5.0, 47.0);
        assert_eq!(
            low,
            Some(TheveninDrive {
                volts: 0.0,
                impedance: 47.0
            })
        );
        let expected = high.map(|volts| TheveninDrive {
            volts,
            impedance: 47.0,
        });
        assert_eq!(stage.port(Level::High, 5.0, 47.0), expected);
    }

    /// A stage's name, as a kind's option takes it, names that stage, and
    /// `with_complements` is the line driver.
    #[rstest]
    fn stages_answer_to_their_names() {
        for stage in OutputStage::ALL {
            assert_eq!(OutputStage::from_name(stage.name()), Some(stage));
        }
        assert_eq!(OutputStage::from_name("open-drain"), None);
        assert_eq!(OutputStage::default(), OutputStage::PushPull);
        assert_eq!(
            Config::new(1.0).with_complements().output,
            OutputStage::Rs422
        );
    }

    // ========================================================
    // Configuration
    // ========================================================

    #[rstest]
    #[case::zero_scale(Config { counts_per_mm: 0.0, ..Config::new(1.0) }, "counts_per_mm")]
    #[case::zero_rail(Config { high_volts: 0.0, ..Config::new(1.0) }, "high_volts")]
    #[case::zero_impedance(
        Config { drive_impedance_ohms: 0.0, ..Config::new(1.0) },
        "drive_impedance_ohms"
    )]
    #[case::zero_cap(Config { max_counts_per_update: 0, ..Config::new(1.0) }, "max_counts_per_update")]
    #[case::zero_revolution(
        Config::new(1.0).with_index(IndexConfig {
            counts_per_revolution: 0,
            width_counts: 1,
            active_level: Level::High,
        }),
        "counts_per_revolution"
    )]
    #[case::zero_width(
        Config::new(1.0).with_index(IndexConfig {
            counts_per_revolution: 4,
            width_counts: 0,
            active_level: Level::High,
        }),
        "width_counts"
    )]
    fn invalid_config_is_rejected_loudly(#[case] config: Config, #[case] field: &str) {
        let error = QuadratureEncoder::new(config).expect_err("must reject");
        assert!(
            error.to_string().contains(field),
            "the error must name {field}: {error}"
        );
    }

    /// The input handle shares the component's wire state, not a copy.
    #[rstest]
    fn input_shares_the_wire_state() {
        let enc = encoder(Config::new(1.0));
        let a = enc.input();
        let b = enc.input();
        a.set_position_counts(3);
        assert_eq!(b.count(), 3);
        assert!(format!("{enc:?}").contains("EncoderCore"));
    }
}
