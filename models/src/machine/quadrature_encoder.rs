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
//! ## Electrical reality of the reference machine
//!
//! A real machine encoder does **not** present logic-level A/B. MaD's encoder
//! drives RS-422 differential pairs `A±`, `B±`, `Z±`, which the EdgeBoard
//! receives with an AM26LV32 quad differential line receiver (U25) before they
//! reach the P2 as the single-ended `ENC_A`, `ENC_B`, `ENC_Z` — see
//! `Hardware/EdgeBoard/KiCad/MaD_Edge_Sheet3.kicad_sch` (the complementary
//! AM26LS31 driver U24 on the same sheet handles the outbound step/direction
//! pairs). **This component models the logic-level pair on the MCU side of
//! that receiver**: each differential pair collapses to one logical channel,
//! so a system description wires `ENCODER.A` to whatever net carries `ENC_A`.
//! Modeling the pair itself would mean modeling the receiver, which buys
//! nothing until a fault scenario wants to break one leg of a pair.
//!
//! ## Not modeled
//!
//! - **The differential pair and its receiver** (above): no common-mode range,
//!   no failsafe bias, no single-ended-leg fault.
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
    AttachError, Component, ComponentNetIo, Level, Ohms, PinDecl, PinHandle, PinKind,
    TheveninDrive, Volts,
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
    /// Open-circuit voltage driven for a logic high
    /// ([`DEFAULT_HIGH_VOLTS`]).
    pub high_volts: Volts,
    /// Thevenin source impedance of the output drivers
    /// (`embsim_board::net::DEFAULT_PUSH_PULL_IMPEDANCE`).
    pub drive_impedance_ohms: Ohms,
    /// Transitions one position update may emit before snapping
    /// ([`DEFAULT_MAX_COUNTS_PER_UPDATE`]).
    pub max_counts_per_update: u32,
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
        }
    }

    /// Add an index channel (and its `Z` pin).
    pub fn with_index(mut self, index: IndexConfig) -> Self {
        self.index = Some(index);
        self
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

/// One declared output channel.
fn output(number: &'static str, impedance: Ohms) -> PinDecl {
    PinDecl {
        number,
        name: None,
        kind: PinKind::DigitalOut,
        stream: None,
        drive_impedance: Some(impedance),
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

    /// Drive one pin to a level (a push-pull Thevenin source at the configured
    /// rail and impedance).
    fn drive(&self, handle: &Option<PinHandle>, level: Level) {
        let Some(handle) = handle else {
            return;
        };
        handle.set_drive(Some(TheveninDrive {
            volts: match level {
                Level::High => self.config.high_volts,
                Level::Low => 0.0,
            },
            impedance: self.config.drive_impedance_ohms,
        }));
    }

    /// Publish the current count's levels, enqueueing only the channels that
    /// actually changed. `force` re-drives everything (used at attach, where
    /// the engine has assigned each output its default idle drive).
    fn publish(&self, wire: &mut Wire, force: bool) {
        let (a, b) = self.phase(wire.count);
        let z = self.index_level(wire.count);
        let previous = wire.driven;
        if force || previous.is_none_or(|(pa, _, _)| pa != a) {
            self.drive(&wire.a, a);
        }
        if force || previous.is_none_or(|(_, pb, _)| pb != b) {
            self.drive(&wire.b, b);
        }
        if wire.z.is_some() && (force || previous.is_none_or(|(_, _, pz)| pz != z)) {
            self.drive(&wire.z, z);
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
/// configured — `Z`. Attach drives the initial phase, so the count and the
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
        let impedance = config.drive_impedance_ohms;
        let mut pins = vec![output("A", impedance), output("B", impedance)];
        if config.index.is_some() {
            pins.push(output("Z", impedance));
        }
        tracing::info!(
            counts_per_mm = config.counts_per_mm,
            index = config.index.is_some(),
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
        // Force the initial phase: the engine gives every `DigitalOut` an
        // idle-high drive at assembly, which is not the encoder's count-0
        // state, so all channels are re-driven here rather than diffed.
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

    /// Every declared channel is a push-pull output at the configured
    /// impedance and carries no stream role.
    #[rstest]
    fn declared_channels_are_push_pull_outputs() {
        let enc = encoder(Config {
            drive_impedance_ohms: 47.0,
            ..Config::new(1.0)
        });
        for decl in enc.pins() {
            assert_eq!(decl.kind, PinKind::DigitalOut);
            assert_eq!(decl.drive_impedance, Some(47.0));
            assert_eq!(decl.stream, None);
        }
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
