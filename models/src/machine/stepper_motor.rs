//! Model: step/direction motor drive — the drive's **input side** plus the
//! carriage it moves.
//!
//! The component's boundary is the three logic inputs every step/direction
//! stepper or servo drive exposes — `STEP`, `DIR`, `ENA` — and its output is
//! a carriage position an encoder or an end switch can follow:
//!
//! ```text
//!   STEP ──┐                         ┌── position_counts() / position_mm()
//!   DIR  ──┤ commanded rate ──lag──► ├── velocity_counts_per_s()
//!   ENA  ──┘                         └── MotorShaft::on_position_change(mm)
//! ```
//!
//! # Two ways the STEP input arrives
//!
//! The drive is told a *rate*, not a position, and it can learn that rate two
//! ways. Both are live at once; whichever the system description wires up wins.
//!
//! 1. **Edges** — the classic path. `STEP` is a plain digital input and the
//!    plant reconstructs the rate from transition timing (below). Exact, and
//!    the only option for a source that really does toggle a pin, but it costs
//!    the engine one drive + resolution + sense per step.
//! 2. **A periodic drive** — a source that drives `STEP` with
//!    [`embsim_board::Drive::Periodic`] (an [`embsim_board::McuComponent`]
//!    with a bridged pulse-out channel) puts a square wave on the net, and
//!    the drive's `STEP` sense is handed its segment
//!    ([`embsim_board::PeriodicSense`]) — rate,
//!    accumulated count, ceiling and anchor — **once per rate change**. The
//!    plant folds the exact pulse count out of each segment at read time, so
//!    a 100 kHz train costs no more engine traffic than a 1 Hz one and the
//!    step count still matches the firmware's own view bit for bit. This is
//!    the path that lets a consumer stop hand-wiring the carriage; the
//!    fidelity it trades away is listed on [`embsim_board::Drive::Periodic`].
//!    The wire carries no direction: the sign is the drive's own `DIR`
//!    input, latched at the instant the `DIR` net changes.
//!
//! A rate-carried train **suspends the edge machinery**: it supplies its own
//! rate, so the stall window ([`DEFAULT_STALL_INTERVALS`]) — which exists only
//! to notice a *measured* train going quiet — does not apply to it. A finite
//! train instead stops commanding exactly when its last pulse goes out. A
//! `STEP` net that stops carrying a segment (the source released it, or a
//! fight took it) ends the train at the instant the drive sees it end.
//!
//! # Provenance (physics model — no datasheet)
//!
//! ## Governing equations
//!
//! On the edge path the plant reconstructs the rate from **STEP edge timing**:
//!
//! ```text
//!   f_step[k] = 1e9 / (t[k] − t[k−1])         steps/s, from the last two
//!                                            active STEP edges (t in ns)
//!   v_cmd     = ±f_step[k]                    sign from the DIR level
//!                                            latched at edge k
//! ```
//!
//! On the rate-carried path `v_cmd = ±freq_hz` straight from the segment, with
//! the sign from the latched `DIR` level. Everything downstream — the lag,
//! the load, the observers — is identical.
//!
//! Motor plus carriage is a first-order velocity lag with a viscous load —
//! `dv/dt = (v_cmd − v)/τ` — solved **exactly** over each segment of constant
//! `v_cmd`:
//!
//! ```text
//!   v(t₀+Δ) = v_cmd + (v(t₀) − v_cmd)·e^(−Δ/τ)
//!   ∫v dt   = v_cmd·Δ + (v(t₀) − v_cmd)·τ·(1 − e^(−Δ/τ))
//!   x(t₀+Δ) = x(t₀) + (1 − k_load)·∫v dt
//! ```
//!
//! `k_load` is a fractional speed loss proportional to velocity (sample load
//! and friction): the open-loop pulse rate over-delivers against true motion,
//! which is exactly the "slip under load" a closed encoder loop exists to
//! catch. It vanishes at rest, so the carriage still settles cleanly with no
//! stiction dead zone.
//!
//! ## Parameter sources
//!
//! - `τ` = [`DEFAULT_TAU_S`] (20 ms) and `k_load` = [`DEFAULT_LOAD_LOSS`]
//!   (0.15) come from the reference consumer's hand-wired plant: MaD's
//!   `SIL/MaDSim/src/wiring.rs`, `struct ServoPlant` with `SERVO_TAU_S =
//!   0.020` and `SERVO_LOAD_LOSS = 0.15`. They were chosen there as the
//!   headline plant-fidelity knobs — large enough that the firmware's
//!   encoder-as-truth velocity servo has real dynamics to close against.
//!   Both are `Config` fields here, because they describe *a* machine, not
//!   all machines.
//! - `steps_per_mm` has no default: it is rig geometry from the system
//!   description (MaD: 4 microsteps × 2048 steps/rev = 8192 counts/mm, which
//!   must match the firmware's own constant).
//! - [`DEFAULT_STALL_INTERVALS`] is derived, not measured — see its docs.
//!
//! ## Deliberate deviation from the cited plant
//!
//! `ServoPlant` advances with the Euler update `v += (v_cmd − v)·min(Δ/τ, 1)`
//! on every pulse-out progress tick. That is the first-order Taylor expansion
//! of the exponential above, and its answer depends on the tick cadence. The
//! board engine has no broadcast tick and `embsim_core::virtual_clock` is
//! free-running *sampled* scaled wall time (`BOARD_ENGINE.md`, "Execution
//! model"), so this model uses the closed form and evaluates it at **read
//! time**: the position you read does not depend on how often anyone looked.
//!
//! ## Electrical reality of the reference machine
//!
//! On MaD's EdgeBoard these signals leave the PCB differentially: `STEP` and
//! `DIR` are driven onto RS-422 pairs `SC_PUL±` / `SC_DIR±` by an AM26LS31
//! quad line driver (U24), while `ENA` leaves single-ended as `SC_ENA` through
//! an open-collector NPN (Q1, 2N3904) sinking the drive's opto input — see
//! `Hardware/EdgeBoard/KiCad/MaD_Edge_Sheet3.kicad_sch`. This component models
//! the **logic-level** inputs: each differential pair collapses to one logical
//! channel, and the open-collector `ENA` is why its active level is
//! configurable ([`Config::enable_active_low`]) rather than assumed.
//!
//! ## Not modeled
//!
//! - **Torque, current limit, stall, lost steps.** The carriage follows the
//!   commanded rate through the lag whatever the load; a real drive would
//!   fault or lose position. Force/extension physics belongs to the
//!   consumer's own models.
//! - **Position is unbounded.** There is no mechanical travel limit here —
//!   end of travel is [`crate::machine::end_switch`].
//! - **Holding torque and braking.** Dropping `ENA` sets `v_cmd = 0` and the
//!   carriage decays to rest through the same lag (an unbraked servo coasts).
//!   It neither locks in place nor free-wheels forever.
//! - **STEP pulse width, and DIR setup/hold.** `DIR` is sampled at the active
//!   `STEP` edge, so a `DIR` change closer than a real drive's setup window
//!   still takes effect on that step; a runt `STEP` pulse is a full step.
//! - **Microstep sub-division within one step, cogging, resonance,
//!   backlash, lead-screw error, thermal drift.**
//! - **A rate that is measured, not commanded.** The plant cannot know a
//!   train has ended (silence is not an event), so it expires the commanded
//!   rate after [`Config::stall_intervals`] of quiet; see
//!   [`DEFAULT_STALL_INTERVALS`].

use std::fmt;
use std::sync::{Arc, Mutex};

use embsim_board::{
    jesd8c01_lvcmos_thresholds, AttachError, Component, ComponentNetIo, DeadBand, DigitalReceiver,
    Level, PeriodicSchedule, PinDecl, Sense,
};
use embsim_core::event::Observers;
use embsim_core::virtual_clock;

use super::{require_fraction, require_positive, MachineConfigError};

// ============================================================
// Reference parameters
// ============================================================

/// Motor + carriage velocity time constant (s) — the headline
/// plant-fidelity knob. Larger is more sluggish, so the closed loop must work
/// harder. Source: `SERVO_TAU_S` in MaD's `SIL/MaDSim/src/wiring.rs`.
pub const DEFAULT_TAU_S: f64 = 0.020;

/// Viscous load: fractional speed loss proportional to velocity. Source:
/// `SERVO_LOAD_LOSS` in MaD's `SIL/MaDSim/src/wiring.rs`.
pub const DEFAULT_LOAD_LOSS: f64 = 0.15;

/// Quiet intervals after the last STEP edge before the commanded rate expires.
///
/// A step train that stops sends no "stopped" event, so the plant infers the
/// end of the train from silence lasting this many of the most recently
/// measured edge-to-edge intervals.
///
/// The value 2.0 is **derived, not tuned**. For a uniform train of `N` edges
/// spaced `d` apart, the reconstructed travel in the lossless quasi-static
/// limit (`k_load = 0`, `τ ≪ d`) is
///
/// ```text
///   (N − 2) steps  from the N−1 inter-edge segments (the first edge only
///                  establishes phase — one edge cannot measure a rate)
/// + stall_intervals steps  from the tail, where the last edge's rate persists
/// ```
///
/// so `stall_intervals = 2.0` makes a train of `N` pulses reconstruct to
/// exactly `N` steps of travel. Anything else biases every move.
pub const DEFAULT_STALL_INTERVALS: f64 = 2.0;

/// Ceiling (µs) on the stall window, so a very slow train does not keep the
/// carriage creeping for seconds after it stops.
pub const DEFAULT_MAX_STALL_US: u64 = 250_000;

/// Nanoseconds in a microsecond: the plant runs on the engine's
/// nanoseconds, its configuration's durations are microseconds.
const NS_PER_US: u64 = 1_000;

/// Default cadence (µs of virtual time) at which the component samples its own
/// plant and emits [`MotorShaft::on_position_change`]. 1 kHz is fine enough
/// for a downstream encoder at any practical step rate while costing the
/// engine one timer-wheel entry.
pub const DEFAULT_OBSERVE_INTERVAL_US: u64 = 1_000;

// ============================================================
// Configuration
// ============================================================

/// Which STEP transition the drive counts as a step.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StepEdge {
    /// Low → high (the common convention).
    Rising,
    /// High → low.
    Falling,
}

/// Step/direction drive configuration. Build with [`Config::new`] and
/// override the fields a particular machine needs.
#[derive(Debug, Clone)]
pub struct Config {
    /// Steps (encoder counts) per millimetre of carriage travel. Rig
    /// geometry; must match the firmware's own constant.
    pub steps_per_mm: f64,
    /// Velocity time constant `τ` in seconds ([`DEFAULT_TAU_S`]).
    pub tau_s: f64,
    /// Viscous load `k_load` in `[0, 1)` ([`DEFAULT_LOAD_LOSS`]).
    pub load_loss: f64,
    /// STEP transition counted as a step.
    pub step_edge: StepEdge,
    /// The DIR level that means *forward* (increasing position). MaD's
    /// firmware convention is `SERVO_DIR` active → reverse, so a system
    /// description mapping an active-high `SERVO_DIR` sets this to
    /// [`Level::Low`].
    pub dir_forward_level: Level,
    /// `true` when the drive is enabled by pulling ENA **low** (the
    /// open-collector convention). `false` for an active-high enable.
    pub enable_active_low: bool,
    /// Quiet intervals before the commanded rate expires
    /// ([`DEFAULT_STALL_INTERVALS`]).
    pub stall_intervals: f64,
    /// Ceiling (µs) on the stall window ([`DEFAULT_MAX_STALL_US`]).
    pub max_stall_us: u64,
    /// Virtual-time cadence for [`MotorShaft::on_position_change`], or `None`
    /// for a purely pull-based plant that arms no timer at all.
    pub observe_interval_us: Option<u64>,
}

impl Config {
    /// Configuration for a rig with the given `steps_per_mm`, with every
    /// other field at its documented default: `τ` = [`DEFAULT_TAU_S`],
    /// `k_load` = [`DEFAULT_LOAD_LOSS`], rising STEP edges, DIR high =
    /// forward, active-high ENA, and a 1 kHz observation cadence.
    pub fn new(steps_per_mm: f64) -> Self {
        Self {
            steps_per_mm,
            tau_s: DEFAULT_TAU_S,
            load_loss: DEFAULT_LOAD_LOSS,
            step_edge: StepEdge::Rising,
            dir_forward_level: Level::High,
            enable_active_low: false,
            stall_intervals: DEFAULT_STALL_INTERVALS,
            max_stall_us: DEFAULT_MAX_STALL_US,
            observe_interval_us: Some(DEFAULT_OBSERVE_INTERVAL_US),
        }
    }

    /// Reject a configuration that cannot describe a machine.
    fn validate(&self) -> Result<(), MachineConfigError> {
        require_positive("steps_per_mm", self.steps_per_mm)?;
        require_positive("tau_s", self.tau_s)?;
        require_fraction("load_loss", self.load_loss)?;
        require_positive("stall_intervals", self.stall_intervals)?;
        if self.max_stall_us == 0 {
            return Err(MachineConfigError::Zero {
                field: "max_stall_us",
            });
        }
        Ok(())
    }

    /// The ENA level that enables the drive.
    fn enable_level(&self) -> Level {
        if self.enable_active_low {
            Level::Low
        } else {
            Level::High
        }
    }

    /// The STEP level the counted edge arrives at.
    fn step_active_level(&self) -> Level {
        match self.step_edge {
            StepEdge::Rising => Level::High,
            StepEdge::Falling => Level::Low,
        }
    }
}

// ============================================================
// Pin facade
// ============================================================

/// One declared logic input, reading through the JEDEC JESD8C.01 3.3 V
/// LVCMOS pair ([`jesd8c01_lvcmos_thresholds`]): the drive is a mechanism
/// model with no datasheet for its receivers, so the pair the engine's own
/// dead band projects a net through is declared for them, stated, with the
/// standard's no-level answer between its figures — each input then holds
/// what it last latched, as for a floating one. Each input projects its own
/// sense through that declaration ([`embsim_board::DigitalReceiver`]).
const fn input(number: &'static str) -> PinDecl {
    PinDecl::digital_in(number, jesd8c01_lvcmos_thresholds(DeadBand::Unknown))
}

/// The drive's logic inputs: `STEP`, `DIR`, `ENA` — all sensed, never driven
/// (the drive presents high-impedance opto/receiver inputs; the board on the
/// other side of the harness owns the drive strength).
///
/// `STEP` is a plain sense: a periodic drive on its net arrives as a
/// [`embsim_board::PeriodicSense`] and, when its phases cross `STEP`'s
/// thresholds ([`embsim_board::PeriodicSense::rate`]), reaches the plant
/// without a per-step edge; a clock whose phases do not cross them is the
/// level they settle to, or none, and a net that toggles levels runs the
/// edge path.
pub const STEPPER_PINS: [PinDecl; 3] = [input("STEP"), input("DIR"), input("ENA")];

// ============================================================
// Plant
// ============================================================

/// Everything the closed form needs, advanced lazily to a sampled virtual
/// time. All fields are in step (encoder count) units.
#[derive(Debug)]
struct Plant {
    /// Virtual time (ns) the closed form has been advanced to — the
    /// engine's own unit, so a train's pulses fold at the instant their
    /// schedule and a `DIR` change say, to the nanosecond.
    now_ns: u64,
    /// Lagged carriage velocity (steps/s) — the plant truth.
    vel: f64,
    /// Carriage position (steps, fractional) — the plant truth.
    pos: f64,
    /// Commanded rate (steps/s), piecewise constant between STEP edges.
    cmd: f64,
    /// Virtual time (ns) of the most recent counted STEP edge.
    last_edge_ns: Option<u64>,
    /// Most recent measured edge-to-edge interval (ns); `None` until two
    /// edges of one train have been seen.
    interval_ns: Option<u64>,
    /// Counted STEP edges, signed by direction — the raw *commanded* step
    /// count, independent of the plant's lag and load.
    steps: i64,
    /// The segment currently presented on `STEP`, **exactly as it was
    /// published** — never re-anchored. `None` on the edge path.
    ///
    /// Keeping the publisher's anchor is what makes the fold lossless: every
    /// reading derives from the one `emitted_at_ns` the source anchored, so the
    /// integer truncation happens once instead of once per read. Re-basing on
    /// every read would trail the true count by up to a pulse each time (see
    /// [`embsim_board::PeriodicSchedule::rebased_at_ns`]).
    train: Option<PeriodicSchedule>,
    /// Pulses of the current train already folded into `train_steps`,
    /// measured from that train's own baseline. Reset when a new segment
    /// replaces it.
    train_folded: u64,
    /// Signed pulses folded out of rate-carried trains — the same quantity
    /// `steps` is for edges, kept apart so a defect in one path cannot look
    /// like the other.
    train_steps: i64,
    /// How late the current rate-carried train was delivered relative to its
    /// publisher anchor (`delivery − since_ns`). Zero when the engine handed
    /// it over on time. Shifted onto [`MotorCore::train_end`] so the plant
    /// integrates `rate × duration` even when free-running delivery slipped —
    /// otherwise pulse count and distance disagree by exactly `lag × rate`
    /// (see issue #43).
    train_delivery_lag_ns: u64,
    /// Last position (rounded to whole steps) published to observers.
    emitted: i64,
    /// Last projected STEP level, for edge detection.
    step_level: Option<Level>,
    /// Last latched direction: `true` = forward. Held across a floating or
    /// fought-over DIR — the drive latches the last level it actually saw.
    forward: bool,
    /// Whether the drive is currently enabled.
    enabled: bool,
}

impl Plant {
    fn new() -> Self {
        Self {
            now_ns: 0,
            vel: 0.0,
            pos: 0.0,
            cmd: 0.0,
            last_edge_ns: None,
            interval_ns: None,
            steps: 0,
            train: None,
            train_folded: 0,
            train_steps: 0,
            train_delivery_lag_ns: 0,
            emitted: 0,
            step_level: None,
            // Before DIR ever presents a level the drive assumes forward;
            // documented rather than guessed at read time.
            forward: true,
            // A floating ENA is not an enable (safe machine).
            enabled: false,
        }
    }
}

/// Shared plant + observers behind the component and every [`MotorShaft`].
struct MotorCore {
    config: Config,
    plant: Mutex<Plant>,
    on_position_mm: Observers<f64>,
}

impl fmt::Debug for MotorCore {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // `Observers` holds boxed closures and is not `Debug`; the plant and
        // the configuration are what a diagnostic dump wants anyway.
        f.debug_struct("MotorCore")
            .field("config", &self.config)
            .field("plant", &self.plant)
            .field("observers", &self.on_position_mm.len())
            .finish()
    }
}

impl MotorCore {
    /// Virtual time now, or `None` while `virtual_clock::init` has not run —
    /// in which case the plant simply does not advance (a loud trace, never a
    /// panic, mirroring the engine's own clock gate).
    fn sample_now(&self, context: &str) -> Option<u64> {
        if virtual_clock::is_initialized() {
            Some(virtual_clock::virtual_ns())
        } else {
            tracing::debug!(context, "stepper_motor: virtual clock not initialized");
            None
        }
    }

    /// Virtual time (ns) at which the current commanded rate expires, if a
    /// measured train is running.
    fn stall_deadline(&self, plant: &Plant) -> Option<u64> {
        let edge = plant.last_edge_ns?;
        let interval = plant.interval_ns?;
        let window = (interval as f64 * self.config.stall_intervals).round() as u64;
        let ceiling = self.config.max_stall_us.saturating_mul(NS_PER_US);
        Some(edge.saturating_add(window.clamp(1, ceiling)))
    }

    /// Sign a rate-carried train's pulses carry: the `DIR` level latched
    /// last — the wire carries no direction.
    fn train_sign(plant: &Plant) -> i64 {
        if plant.forward {
            1
        } else {
            -1
        }
    }

    /// Commanded rate (steps/s) implied by the current rate-carried train, or
    /// `0` while the drive is disabled.
    fn train_rate(plant: &Plant, train: &PeriodicSchedule) -> f64 {
        if !plant.enabled {
            return 0.0;
        }
        Self::train_sign(plant) as f64 * f64::from(train.freq_hz)
    }

    /// Virtual time (ns) at which the current rate-carried finite train stops
    /// commanding — the publisher's last-pulse instant, shifted by any
    /// free-running delivery lag so travel equals `rate × duration` even when
    /// the train arrived late (issue #43).
    fn train_end(&self, plant: &Plant) -> Option<u64> {
        Some(
            plant
                .train?
                .completes_at()?
                .saturating_add(plant.train_delivery_lag_ns),
        )
    }

    /// Advance the closed form to `to_ns`, splitting wherever the commanded
    /// rate changes on its own so each segment sees a constant `cmd`.
    ///
    /// Two such splits exist and they belong to the two input paths: the
    /// edge path's stall expiry, and a rate-carried finite train's completion.
    /// They are mutually exclusive — a train suspends the edge machinery — so
    /// the two branches never interleave.
    fn advance(&self, plant: &mut Plant, to_ns: u64) {
        if to_ns <= plant.now_ns {
            return;
        }
        if plant.train.is_some() {
            match self.train_end(plant) {
                // Already finished on an earlier advance.
                Some(end) if end <= plant.now_ns => plant.cmd = 0.0,
                // Finishes inside this advance: integrate up to it, then coast.
                Some(end) if end < to_ns => {
                    self.fold_and_integrate(plant, end);
                    plant.cmd = 0.0;
                }
                _ => {}
            }
            self.fold_and_integrate(plant, to_ns);
            return;
        }
        match self.stall_deadline(plant) {
            // Already expired on an earlier advance.
            Some(stall) if stall <= plant.now_ns => plant.cmd = 0.0,
            // Expires inside this advance: integrate up to it, then coast.
            Some(stall) if stall < to_ns => {
                self.integrate(plant, stall);
                plant.cmd = 0.0;
            }
            _ => {}
        }
        self.integrate(plant, to_ns);
    }

    /// Fold the rate-carried train's pulses up to `to_ns` into `train_steps`.
    ///
    /// Exact and idempotent by construction: `train_folded` records how much
    /// of *this* segment has already been counted, and every reading comes
    /// from the publisher's own anchor, so folding at every advance — which is
    /// every read — can neither double-count a pulse nor miss one. The sign is
    /// taken at fold time, which is why callers advance the plant *before*
    /// latching a new direction.
    fn fold(&self, plant: &mut Plant, to_ns: u64) {
        let Some(train) = plant.train else {
            return;
        };
        let total = train.emitted_at_ns(to_ns).saturating_sub(train.emitted);
        let pulses = total.saturating_sub(plant.train_folded);
        // A disabled drive ignores the train, but the pulses still went out:
        // consuming them here is what stops them arriving late on re-enable.
        if plant.enabled && pulses > 0 {
            let sign = Self::train_sign(plant);
            plant.train_steps += sign * i64::try_from(pulses).unwrap_or(i64::MAX);
        }
        plant.train_folded = total;
    }

    /// [`Self::fold`] the train, then integrate the plant over the same span.
    fn fold_and_integrate(&self, plant: &mut Plant, to_ns: u64) {
        self.fold(plant, to_ns);
        self.integrate(plant, to_ns);
    }

    /// [`Self::receive_train`] with no delivery instant: the unit tests
    /// inject a segment's instants directly.
    #[cfg(test)]
    fn set_train(&self, train: PeriodicSchedule) {
        self.receive_train(train, None);
    }

    /// Present a new rate-carried segment on `STEP`, delivered at
    /// `delivered_ns`.
    ///
    /// The outgoing segment is folded up to the new segment's start instant
    /// first, so the pulses on each side of a rate change keep their own rate
    /// — which is what makes a retarget exact. The same segment handed over
    /// again (the net's levels changed and its schedule did not) is no change
    /// at all: it is still the segment being folded.
    ///
    /// The plant never moves past the instant it was handed the segment. A
    /// segment anchored later than that — a producer that
    /// published ahead of virtual time, against `NODES.md` §11 contract
    /// line 2 — is traced, and the plant folds only to the delivery; the
    /// segment's own arithmetic reads its baseline until its anchor, so the
    /// count stays the producer's.
    fn receive_train(&self, train: PeriodicSchedule, delivered_ns: Option<u64>) {
        let mut plant = self.plant.lock().unwrap();
        if plant.train == Some(train) {
            return;
        }
        let anchor = match delivered_ns {
            Some(now) if train.since_ns > now => {
                tracing::warn!(
                    since_ns = train.since_ns,
                    delivered_ns = now,
                    "stepper_motor: a step segment anchored after the instant it was delivered"
                );
                now
            }
            _ => train.since_ns,
        };
        // The engine may deliver later than the source acted (free-running);
        // never rewind the plant to match. Record the lag so `train_end`
        // finishes `lag` later than the source said — conserving the
        // pulse↔distance invariant instead of silently truncating travel.
        let at = anchor.max(plant.now_ns);
        let delivery_lag_ns = at.saturating_sub(train.since_ns);
        self.advance(&mut plant, at);
        // `advance` is a no-op when `at == now_ns`, so fold explicitly. It is
        // idempotent, so doing both is safe.
        self.fold(&mut plant, at);
        plant.cmd = Self::train_rate(&plant, &train);
        plant.train = Some(train);
        plant.train_folded = 0;
        plant.train_delivery_lag_ns = delivery_lag_ns;
        // A rate-carried train supplies its own rate, so the edge path's
        // phase measurement (and its stall window) must not also apply.
        plant.last_edge_ns = None;
        plant.interval_ns = None;
    }

    /// The `STEP` net stopped carrying a segment at `now_ns` — its source
    /// released it, or a fight took it: fold the train up to that instant
    /// and stop commanding. Nothing to do on the edge path.
    fn end_train(&self, now_ns: u64) {
        let mut plant = self.plant.lock().unwrap();
        if plant.train.is_none() {
            return;
        }
        self.advance(&mut plant, now_ns);
        self.fold(&mut plant, now_ns);
        plant.train = None;
        plant.train_folded = 0;
        plant.train_delivery_lag_ns = 0;
        plant.cmd = 0.0;
    }

    /// One closed-form segment at the current `plant.cmd`.
    fn integrate(&self, plant: &mut Plant, to_ns: u64) {
        let dt = to_ns.saturating_sub(plant.now_ns) as f64 / 1_000_000_000.0;
        plant.now_ns = to_ns;
        if dt <= 0.0 {
            return;
        }
        let decay = (-dt / self.config.tau_s).exp();
        let v0 = plant.vel;
        let cmd = plant.cmd;
        plant.vel = cmd + (v0 - cmd) * decay;
        let travel = cmd * dt + (v0 - cmd) * self.config.tau_s * (1.0 - decay);
        plant.pos += travel * (1.0 - self.config.load_loss);
    }

    /// Handle one counted STEP edge at `now_ns`.
    fn step_edge(&self, now_ns: u64) {
        let mut plant = self.plant.lock().unwrap();
        // A disabled drive ignores the step train entirely; the carriage
        // still coasts, so time must still advance.
        if !plant.enabled {
            self.advance(&mut plant, now_ns);
            return;
        }
        // A gap longer than the stall window ends the train: the next edge
        // restarts phase measurement rather than reading the gap as an
        // absurdly slow rate (one step of dead time, by construction).
        let fresh = self
            .stall_deadline(&plant)
            .is_some_and(|stall| now_ns >= stall);
        self.advance(&mut plant, now_ns);

        plant.interval_ns = if fresh {
            None
        } else {
            plant
                .last_edge_ns
                .map(|previous| now_ns.saturating_sub(previous))
                .filter(|&interval| interval > 0)
        };
        plant.last_edge_ns = Some(now_ns);
        plant.steps += if plant.forward { 1 } else { -1 };
        let sign = if plant.forward { 1.0 } else { -1.0 };
        plant.cmd = match plant.interval_ns {
            Some(interval) => sign * 1_000_000_000.0 / interval as f64,
            // One edge cannot measure a rate.
            None => 0.0,
        };
    }

    /// Latch a new DIR level at `now_ns`. A `None` projection (floating /
    /// contended DIR) holds the last latched direction.
    ///
    /// The plant is advanced first: on the rate-carried path that folds
    /// everything emitted so far under the *old* direction, so a reversal
    /// mid-train splits exactly at the instant DIR changed rather than
    /// re-signing pulses that already went out.
    fn set_dir(&self, now_ns: u64, level: Option<Level>) {
        let mut plant = self.plant.lock().unwrap();
        self.advance(&mut plant, now_ns);
        let Some(level) = level else {
            tracing::debug!("stepper_motor: DIR has no level; holding last latched direction");
            return;
        };
        plant.forward = level == self.config.dir_forward_level;
        if let Some(train) = plant.train {
            plant.cmd = Self::train_rate(&plant, &train);
        }
    }

    /// Apply a new ENA level at `now_ns`. Disabling ends the train: the
    /// commanded rate drops to zero and the carriage decays through the lag.
    fn set_ena(&self, now_ns: u64, level: Option<Level>) {
        let mut plant = self.plant.lock().unwrap();
        self.advance(&mut plant, now_ns);
        let enabled = level == Some(self.config.enable_level());
        if plant.enabled && !enabled {
            plant.cmd = 0.0;
            plant.last_edge_ns = None;
            plant.interval_ns = None;
        }
        plant.enabled = enabled;
        // A rate-carried train keeps running at its source whether or not the
        // drive is listening, so enabling mid-train picks it up at its current
        // rate (and disabling drops it) without waiting for the next segment.
        if let Some(train) = plant.train {
            plant.cmd = Self::train_rate(&plant, &train);
        }
    }

    /// Detect a counted STEP edge from a new net state.
    fn set_step(&self, now_ns: u64, level: Option<Level>) {
        let previous = {
            let mut plant = self.plant.lock().unwrap();
            let previous = plant.step_level;
            plant.step_level = level;
            previous
        };
        let Some(level) = level else {
            // A floating or fought-over STEP net is not an edge; the next
            // real level re-seeds edge detection.
            tracing::debug!("stepper_motor: STEP has no level; no edge counted");
            return;
        };
        let active = self.config.step_active_level();
        // The first delivery only seeds the detector — `on_sense` reports the
        // current state at registration, which is not a transition.
        if previous.is_some_and(|prev| prev != level) && level == active {
            self.step_edge(now_ns);
        }
    }

    /// Sample the plant at `now_ns` and publish a position change when the
    /// whole-step position moved. The observer runs with the plant lock
    /// released, so a subscriber may read the shaft back without deadlocking.
    fn observe(&self, now_ns: u64) {
        let emit = {
            let mut plant = self.plant.lock().unwrap();
            self.advance(&mut plant, now_ns);
            let rounded = plant.pos.round() as i64;
            (rounded != plant.emitted).then(|| {
                plant.emitted = rounded;
                plant.pos / self.config.steps_per_mm
            })
        };
        if let Some(position_mm) = emit {
            self.on_position_mm.emit(position_mm);
        }
    }

    /// Advance to sampled virtual time and read the plant.
    fn read<T>(&self, context: &str, project: impl FnOnce(&Plant) -> T) -> T {
        let mut plant = self.plant.lock().unwrap();
        if let Some(now_ns) = self.sample_now(context) {
            self.advance(&mut plant, now_ns);
        }
        project(&plant)
    }
}

// ============================================================
// Shaft handle
// ============================================================

/// Read-side handle to the carriage a [`StepperMotor`] moves.
///
/// Cloned out of the component *before* it is handed to
/// [`embsim_board::System`], then used by physics chains and tests to read
/// position or subscribe to it. Cheap to clone and thread-safe, like
/// [`embsim_board::PinHandle`].
///
/// Every read advances the plant's closed form to sampled virtual time, so
/// two reads a millisecond apart differ even with no STEP traffic in between
/// (the carriage is still decaying through its lag). Reads before
/// `virtual_clock::init` do not advance and are traced.
#[derive(Clone, Debug)]
pub struct MotorShaft {
    core: Arc<MotorCore>,
}

impl MotorShaft {
    /// Carriage position in steps (encoder counts), fractional — the plant
    /// truth, lag and load included. This is deliberately *not* the commanded
    /// step count; see [`MotorShaft::commanded_steps`].
    pub fn position_counts(&self) -> f64 {
        self.core.read("position_counts", |plant| plant.pos)
    }

    /// Carriage position in millimetres (`position_counts / steps_per_mm`).
    pub fn position_mm(&self) -> f64 {
        let steps_per_mm = self.core.config.steps_per_mm;
        self.core
            .read("position_mm", |plant| plant.pos / steps_per_mm)
    }

    /// Lagged carriage velocity in steps/s (signed).
    pub fn velocity_counts_per_s(&self) -> f64 {
        self.core.read("velocity_counts_per_s", |plant| plant.vel)
    }

    /// Commanded step count with no lag and no load loss — counted STEP edges
    /// signed by the direction latched at each edge, **plus** the pulses
    /// folded out of any rate-carried train. Use it when a test needs to know
    /// what the firmware *asked* for; [`MotorShaft::position_counts`] is what
    /// the machine *did*.
    ///
    /// On the rate-carried path this is exact: the fold uses the same integer
    /// arithmetic the pulse-out peripheral hands the firmware.
    pub fn commanded_steps(&self) -> i64 {
        self.core
            .read("commanded_steps", |plant| plant.steps + plant.train_steps)
    }

    /// The segment currently presented on `STEP`, exactly as the source
    /// published it; `None` on the edge path. A test asserting *what was
    /// published* (rather than what the plant integrated to) reads it here.
    pub fn train(&self) -> Option<PeriodicSchedule> {
        self.core.read("train", |plant| plant.train)
    }

    /// Whether the drive is currently enabled. A floating or fought-over ENA
    /// reads `false` — the engine never invents a level, and the safe reading
    /// of "no enable signal" is "not enabled".
    pub fn enabled(&self) -> bool {
        self.core.plant.lock().unwrap().enabled
    }

    /// The direction currently latched: `true` = forward (increasing
    /// position). Held across a floating or fought-over DIR.
    pub fn forward(&self) -> bool {
        self.core.plant.lock().unwrap().forward
    }

    /// Subscribe to carriage position in millimetres, published whenever the
    /// whole-step position changes at the configured
    /// [`Config::observe_interval_us`] cadence. Multiple subscribers are
    /// appended, never overwritten.
    ///
    /// This is the seam a [`crate::machine::quadrature_encoder`] or an
    /// [`crate::machine::end_switch`] attaches to. With
    /// `observe_interval_us: None` nothing is ever published — the shaft is
    /// pull-only.
    pub fn on_position_change(&self, callback: impl Fn(f64) + Send + 'static) {
        self.core.on_position_mm.subscribe(callback);
    }
}

// ============================================================
// Component
// ============================================================

/// A step/direction motor drive as a live board-engine component.
///
/// Build it, clone its [`MotorShaft`] out, then move the component into the
/// system:
///
/// ```rust,ignore
/// let motor = StepperMotor::new(Config::new(8_192.0))?;
/// let shaft = motor.shaft();
/// let system = System::new()
///     .component("MOTOR", Box::new(motor))
///     .harness(Harness::new().connect_str("P2.P6", "MOTOR.STEP")?)
///     .start()?;
/// shaft.position_mm();
/// ```
#[derive(Debug)]
pub struct StepperMotor {
    core: Arc<MotorCore>,
}

impl StepperMotor {
    /// Create a drive from a validated configuration.
    pub fn new(config: Config) -> Result<Self, MachineConfigError> {
        config.validate()?;
        tracing::info!(
            steps_per_mm = config.steps_per_mm,
            tau_s = config.tau_s,
            load_loss = config.load_loss,
            "stepper_motor: init"
        );
        Ok(Self {
            core: Arc::new(MotorCore {
                config,
                plant: Mutex::new(Plant::new()),
                on_position_mm: Observers::new(),
            }),
        })
    }

    /// A handle to the carriage this drive moves.
    pub fn shaft(&self) -> MotorShaft {
        MotorShaft {
            core: Arc::clone(&self.core),
        }
    }

    /// The validated configuration in force.
    pub fn config(&self) -> &Config {
        &self.core.config
    }
}

impl Component for StepperMotor {
    fn pins(&self) -> &[PinDecl] {
        &STEPPER_PINS
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        // Registration order is load-bearing: `on_sense` delivers the current
        // state once at registration, so ENA and DIR must settle before the
        // first STEP delivery can be read as an edge.
        {
            let core = Arc::clone(&self.core);
            let ena = DigitalReceiver::new(io.pin("ENA")?);
            io.on_sense("ENA", move |sense| {
                // No clock yet means no elapsed time to account for, so 0 is
                // a no-op advance rather than a wrong one.
                let now_ns = core.sample_now("ENA").unwrap_or(0);
                core.set_ena(now_ns, read_level("ENA", &ena, &sense));
            })?;
        }
        {
            let core = Arc::clone(&self.core);
            let dir = DigitalReceiver::new(io.pin("DIR")?);
            io.on_sense("DIR", move |sense| {
                let now_ns = core.sample_now("DIR").unwrap_or(0);
                core.set_dir(now_ns, read_level("DIR", &dir, &sense));
            })?;
        }
        // Registered last, so ENA and DIR have settled before a segment can
        // be folded against them (the engine delivers the net's current
        // state, a periodic one included, once at registration).
        {
            let core = Arc::clone(&self.core);
            let pin = io.pin("STEP")?;
            let step = DigitalReceiver::new(pin.clone());
            io.on_sense("STEP", move |sense| {
                // A clock is a train to fold when its phases cross STEP's
                // own thresholds ([`embsim_board::PeriodicSense::rate`]); a
                // held segment is the stop that folds a running train's
                // final count. A running clock that does not cross them is
                // no train: it is the level its phases settle to, or none,
                // read on the level path below.
                if let Some(clock) = sense.periodic {
                    let counts = clock.segment.freq_hz == 0
                        || pin.thresholds().is_some_and(|t| clock.rate(&t).is_some());
                    if counts {
                        core.receive_train(clock.segment, Some(sense.at_ns));
                        return;
                    }
                }
                let now_ns = core.sample_now("STEP");
                core.end_train(now_ns.unwrap_or(0));
                let level = read_level("STEP", &step, &sense);
                match now_ns {
                    Some(now_ns) => core.set_step(now_ns, level),
                    // No clock means no edge timing; still record the level so
                    // edge detection is seeded once the clock exists.
                    None => core.plant.lock().unwrap().step_level = level,
                }
            })?;
        }

        if let Some(period_us) = self.core.config.observe_interval_us {
            let core = Arc::clone(&self.core);
            io.on_wake_ns(move |now_ns| core.observe(now_ns));
            io.schedule_every(period_us);
        }
        Ok(())
    }
}

/// Project a delivered sense through the input's declared thresholds,
/// tracing the senses it reads no level from.
fn read_level(pin: &'static str, receiver: &DigitalReceiver, sense: &Sense) -> Option<Level> {
    let level = receiver.read(sense);
    if level.is_none() {
        tracing::trace!(pin, ?sense, "stepper_motor: input has no logic level");
    }
    level
}

// ============================================================
// Tests
// ============================================================
//
// The plant is driven through injected timestamps here, so every case is
// exact and clock-independent; the live wiring (real senses through a running
// engine) is covered by `models/tests/machine_live_system.rs`.

#[cfg(test)]
mod tests {
    use rstest::rstest;

    use embsim_board::PeriodicSchedule;

    use super::*;

    /// One step per count and no load: the lossless limit in which the
    /// `stall_intervals` derivation is exact.
    fn lossless() -> Config {
        Config {
            load_loss: 0.0,
            ..Config::new(1.0)
        }
    }

    /// A lag far shorter than any step interval used below.
    fn quasi_static() -> Config {
        Config {
            tau_s: 0.000_1,
            ..lossless()
        }
    }

    /// A test instant written in microseconds, as the plant's nanoseconds.
    const fn us(t: u64) -> u64 {
        t * NS_PER_US
    }

    fn core(config: Config) -> MotorCore {
        MotorCore {
            config,
            plant: Mutex::new(Plant::new()),
            on_position_mm: Observers::new(),
        }
    }

    /// Enable the drive at t = 0 (a harness normally does this through the ENA
    /// sense at attach).
    fn enabled_core(config: Config) -> MotorCore {
        let core = core(config);
        core.set_ena(0, Some(Level::High));
        core
    }

    /// Feed `edges` counted STEP edges spaced `interval_us` apart, the first at
    /// `interval_us`. Returns the virtual time (ns) of the last edge.
    fn run_train(core: &MotorCore, edges: u32, interval_us: u64) -> u64 {
        let mut t = 0;
        for _ in 0..edges {
            t += us(interval_us);
            core.step_edge(t);
        }
        t
    }

    /// Advance from `from_ns` past the stall window *and* 20 τ of velocity
    /// decay, so the run provably ends at rest.
    fn settle(core: &MotorCore, from_ns: u64, interval_us: u64) {
        let window = us(interval_us) as f64 * (core.config.stall_intervals + 1.0);
        let decay = core.config.tau_s * 20.0 * 1_000_000_000.0;
        let mut plant = core.plant.lock().unwrap();
        core.advance(&mut plant, from_ns + (window + decay) as u64);
    }

    fn pos(core: &MotorCore) -> f64 {
        core.plant.lock().unwrap().pos
    }

    fn vel(core: &MotorCore) -> f64 {
        core.plant.lock().unwrap().vel
    }

    fn steps(core: &MotorCore) -> i64 {
        core.plant.lock().unwrap().steps
    }

    // ========================================================
    // Configuration
    // ========================================================

    #[rstest]
    fn config_defaults_match_the_cited_plant() {
        let config = Config::new(8_192.0);
        assert!((config.tau_s - DEFAULT_TAU_S).abs() < f64::EPSILON);
        assert!((config.load_loss - DEFAULT_LOAD_LOSS).abs() < f64::EPSILON);
        assert!((config.stall_intervals - DEFAULT_STALL_INTERVALS).abs() < f64::EPSILON);
        assert_eq!(config.step_edge, StepEdge::Rising);
        assert_eq!(config.dir_forward_level, Level::High);
        assert!(!config.enable_active_low);
    }

    #[rstest]
    #[case::zero_scale(Config { steps_per_mm: 0.0, ..Config::new(1.0) }, "steps_per_mm")]
    #[case::nan_scale(Config { steps_per_mm: f64::NAN, ..Config::new(1.0) }, "steps_per_mm")]
    #[case::zero_tau(Config { tau_s: 0.0, ..Config::new(1.0) }, "tau_s")]
    #[case::full_loss(Config { load_loss: 1.0, ..Config::new(1.0) }, "load_loss")]
    #[case::negative_loss(Config { load_loss: -0.1, ..Config::new(1.0) }, "load_loss")]
    #[case::zero_stall(Config { stall_intervals: 0.0, ..Config::new(1.0) }, "stall_intervals")]
    #[case::zero_max_stall(Config { max_stall_us: 0, ..Config::new(1.0) }, "max_stall_us")]
    fn invalid_config_is_rejected_loudly(#[case] config: Config, #[case] field: &str) {
        let error = StepperMotor::new(config).expect_err("must reject");
        assert!(
            error.to_string().contains(field),
            "the error must name {field}: {error}"
        );
    }

    #[rstest]
    fn enable_level_follows_polarity() {
        assert_eq!(Config::new(1.0).enable_level(), Level::High);
        assert_eq!(
            Config {
                enable_active_low: true,
                ..Config::new(1.0)
            }
            .enable_level(),
            Level::Low
        );
    }

    // ========================================================
    // Step count → position
    // ========================================================

    /// The [`DEFAULT_STALL_INTERVALS`] derivation as an exact contract.
    ///
    /// Integrating the closed form gives `∫v dt = ∫v_cmd dt − τ·Δv`, and a run
    /// that starts and ends at rest has `Δv = 0`. So total travel is exactly
    /// `∫v_cmd dt` — **independent of τ** — which for a uniform train of N
    /// edges spaced d apart is `(1/d)·((N + stall_intervals)·d − 2d) = N` steps
    /// at `stall_intervals = 2`. The τ cases below prove the independence.
    #[rstest]
    #[case::ten_at_1ms(10, 1_000, 0.000_1)]
    #[case::ten_at_200us(10, 200, 0.000_1)]
    #[case::hundred_at_500us(100, 500, 0.000_1)]
    #[case::two_pulses(2, 1_000, 0.000_1)]
    #[case::reference_tau(10, 1_000, DEFAULT_TAU_S)]
    #[case::sluggish_tau(10, 1_000, 0.200)]
    fn step_train_reconstructs_to_the_commanded_step_count(
        #[case] edges: u32,
        #[case] interval_us: u64,
        #[case] tau_s: f64,
    ) {
        let core = enabled_core(Config {
            tau_s,
            ..lossless()
        });
        let last = run_train(&core, edges, interval_us);
        settle(&core, last, interval_us);

        assert_eq!(steps(&core), i64::from(edges), "every edge is counted");
        assert!(
            (pos(&core) - f64::from(edges)).abs() < 0.01,
            "{edges} pulses must reconstruct to {edges} steps (τ = {tau_s}), got {}",
            pos(&core)
        );
        assert!(
            vel(&core).abs() < 0.01,
            "and the carriage must be at rest, v = {}",
            vel(&core)
        );
    }

    /// The viscous load is a fractional loss on travel, exactly as the cited
    /// plant defines it: the same train delivers `(1 − k_load)` of the
    /// commanded steps, and the commanded count itself is untouched.
    #[rstest]
    #[case::none(0.0)]
    #[case::reference(DEFAULT_LOAD_LOSS)]
    #[case::heavy(0.5)]
    fn load_loss_scales_delivered_travel(#[case] load_loss: f64) {
        let core = enabled_core(Config {
            load_loss,
            ..quasi_static()
        });
        let last = run_train(&core, 20, 1_000);
        settle(&core, last, 1_000);

        let expected = 20.0 * (1.0 - load_loss);
        assert!(
            (pos(&core) - expected).abs() < 0.01,
            "expected {expected} steps at load_loss {load_loss}, got {}",
            pos(&core)
        );
        assert_eq!(steps(&core), 20, "commanded steps ignore the load");
    }

    /// A single edge cannot measure a rate, so it commands nothing — but it is
    /// still counted.
    #[rstest]
    fn one_edge_counts_but_commands_no_rate() {
        let core = enabled_core(quasi_static());
        core.step_edge(us(1_000));
        assert_eq!(steps(&core), 1);
        assert!((core.plant.lock().unwrap().cmd - 0.0).abs() < f64::EPSILON);
        {
            let mut plant = core.plant.lock().unwrap();
            core.advance(&mut plant, us(100_000));
        }
        assert!(
            pos(&core).abs() < f64::EPSILON,
            "no measured rate means no travel, got {}",
            pos(&core)
        );
    }

    // ========================================================
    // Direction
    // ========================================================

    /// Reversing DIR reverses the commanded rate on the very next edge, and the
    /// carriage returns to where it started.
    #[rstest]
    fn direction_reversal_turns_the_carriage_around() {
        let core = enabled_core(quasi_static());
        let last = run_train(&core, 20, 1_000);
        settle(&core, last, 1_000);
        let forward_pos = pos(&core);
        assert!(
            (forward_pos - 20.0).abs() < 0.01,
            "20 forward steps, got {forward_pos}"
        );
        assert!(core.plant.lock().unwrap().forward);

        // Bind `now` first: the guard from a `lock()` inside the argument list
        // would live to the end of the statement and self-deadlock `set_dir`.
        let now_ns = core.plant.lock().unwrap().now_ns;
        core.set_dir(now_ns, Some(Level::Low));
        assert!(!core.plant.lock().unwrap().forward, "DIR low = reverse");

        // Resume the train from wherever the previous one settled.
        let start = core.plant.lock().unwrap().now_ns;
        let mut t = start;
        for _ in 0..20 {
            t += us(1_000);
            core.step_edge(t);
        }
        settle(&core, t, 1_000);

        assert_eq!(
            steps(&core),
            0,
            "20 forward + 20 reverse = 0 commanded steps"
        );
        assert!(
            pos(&core).abs() < 0.02,
            "the carriage must come back to the origin, got {}",
            pos(&core)
        );
        assert!(
            vel(&core).abs() < 0.01,
            "and be at rest, v = {}",
            vel(&core)
        );
    }

    /// The direction convention is configuration, not a hard-coded polarity:
    /// with `dir_forward_level: Low` (MaD's `SERVO_DIR` active = reverse) a DIR
    /// high steps backwards.
    #[rstest]
    #[case::high_is_forward(Level::High, Level::High, 1)]
    #[case::high_is_forward_reversed(Level::High, Level::Low, -1)]
    #[case::low_is_forward(Level::Low, Level::Low, 1)]
    #[case::low_is_forward_reversed(Level::Low, Level::High, -1)]
    fn dir_convention_is_configurable(
        #[case] forward_level: Level,
        #[case] dir: Level,
        #[case] expect_sign: i64,
    ) {
        let core = enabled_core(Config {
            dir_forward_level: forward_level,
            ..quasi_static()
        });
        core.set_dir(0, Some(dir));
        core.step_edge(us(1_000));
        assert_eq!(steps(&core), expect_sign);
    }

    /// A DIR the engine gives no level for (floating / contended) holds the last
    /// latched direction rather than guessing.
    #[rstest]
    fn floating_dir_holds_the_last_latched_direction() {
        let core = enabled_core(quasi_static());
        core.set_dir(0, Some(Level::Low));
        core.set_dir(0, None);
        assert!(!core.plant.lock().unwrap().forward);
        core.step_edge(us(1_000));
        assert_eq!(steps(&core), -1);
    }

    // ========================================================
    // Enable
    // ========================================================

    /// A drive that was never enabled ignores the whole train — a floating ENA
    /// is not an enable.
    #[rstest]
    fn disabled_drive_ignores_step_edges() {
        let core = core(quasi_static());
        assert!(
            !core.plant.lock().unwrap().enabled,
            "floating ENA must read as disabled"
        );
        let last = run_train(&core, 20, 1_000);
        settle(&core, last, 1_000);
        assert_eq!(steps(&core), 0);
        assert!(pos(&core).abs() < f64::EPSILON);
    }

    /// Dropping ENA mid-train ends the train: the carriage decays to rest
    /// through the same lag instead of stopping dead or running on forever.
    #[rstest]
    fn disabling_mid_train_coasts_to_rest() {
        let core = enabled_core(Config {
            tau_s: 0.010,
            ..lossless()
        });
        let last = run_train(&core, 20, 1_000);
        assert!(
            vel(&core) > 100.0,
            "the carriage is moving, v = {}",
            vel(&core)
        );

        core.set_ena(last, Some(Level::Low));
        let at_disable = pos(&core);
        {
            let mut plant = core.plant.lock().unwrap();
            core.advance(&mut plant, last + us(500_000));
        }
        assert!(!core.plant.lock().unwrap().enabled);
        assert!(
            vel(&core).abs() < 0.01,
            "coasted to rest, v = {}",
            vel(&core)
        );
        assert!(
            pos(&core) > at_disable,
            "and drifted forward while decaying: {at_disable} -> {}",
            pos(&core)
        );
    }

    /// Enable polarity is configurable, and a level-less ENA never enables.
    #[rstest]
    #[case::active_high_high(false, Some(Level::High), true)]
    #[case::active_high_low(false, Some(Level::Low), false)]
    #[case::active_high_floating(false, None, false)]
    #[case::active_low_low(true, Some(Level::Low), true)]
    #[case::active_low_high(true, Some(Level::High), false)]
    #[case::active_low_floating(true, None, false)]
    fn enable_polarity_and_floating(
        #[case] active_low: bool,
        #[case] level: Option<Level>,
        #[case] expect: bool,
    ) {
        let core = core(Config {
            enable_active_low: active_low,
            ..quasi_static()
        });
        core.set_ena(0, level);
        assert_eq!(core.plant.lock().unwrap().enabled, expect);
    }

    // ========================================================
    // Edge detection
    // ========================================================

    /// The first sense delivery only seeds the detector (`on_sense` reports the
    /// current state at registration, which is not a transition), and only the
    /// configured edge counts: a full high → low → high cycle is exactly one
    /// step either way.
    #[rstest]
    #[case::rising(StepEdge::Rising)]
    #[case::falling(StepEdge::Falling)]
    fn only_the_configured_edge_counts(#[case] step_edge: StepEdge) {
        let core = enabled_core(Config {
            step_edge,
            ..quasi_static()
        });
        core.set_step(0, Some(Level::High));
        core.set_step(us(1_000), Some(Level::Low));
        core.set_step(us(2_000), Some(Level::High));
        assert_eq!(steps(&core), 1);
    }

    /// A STEP net with no level is not an edge, and the level coming back
    /// through an unknown state re-seeds the detector rather than counting a
    /// phantom step.
    #[rstest]
    fn levelless_step_net_counts_nothing() {
        let core = enabled_core(quasi_static());
        core.set_step(0, Some(Level::Low));
        core.set_step(us(1_000), None);
        core.set_step(us(2_000), Some(Level::High));
        assert_eq!(steps(&core), 0);
        core.set_step(us(3_000), Some(Level::Low));
        core.set_step(us(4_000), Some(Level::High));
        assert_eq!(steps(&core), 1, "the re-seeded detector counts normally");
    }

    // ========================================================
    // Stall window
    // ========================================================

    /// A gap longer than the stall window ends the train: the next edge
    /// restarts phase measurement instead of reading the gap as an absurdly
    /// slow rate.
    #[rstest]
    fn a_long_gap_restarts_phase_measurement() {
        let core = enabled_core(quasi_static());
        core.step_edge(us(1_000));
        core.step_edge(us(2_000));
        assert_eq!(core.plant.lock().unwrap().interval_ns, Some(us(1_000)));

        core.step_edge(us(52_000)); // 50 ms of silence ≫ 2 × 1 ms
        let plant = core.plant.lock().unwrap();
        assert_eq!(
            plant.interval_ns, None,
            "the gap must not be read as a ~20 Hz step rate"
        );
        assert!((plant.cmd - 0.0).abs() < f64::EPSILON);
    }

    /// The stall window is capped, so a very slow train does not keep the
    /// carriage creeping for seconds after it stops.
    #[rstest]
    fn stall_window_is_capped() {
        let core = enabled_core(Config {
            max_stall_us: 5_000,
            ..quasi_static()
        });
        core.step_edge(us(1_000));
        core.step_edge(us(1_000_000)); // a 999 ms interval
        let deadline = {
            let plant = core.plant.lock().unwrap();
            core.stall_deadline(&plant).expect("a train is running")
        };
        assert_eq!(
            deadline,
            us(1_005_000),
            "the window must clamp to max_stall_us, not 2 × 999 ms"
        );
    }

    // ========================================================
    // Read-time evaluation
    // ========================================================

    /// Position is a closed form of elapsed virtual time, so reading it at any
    /// cadence gives the same answer — the property a per-tick integrator
    /// cannot offer against the default free-running clock.
    #[rstest]
    fn read_cadence_does_not_change_the_answer() {
        let config = Config {
            tau_s: 0.010,
            ..lossless()
        };
        let coarse = enabled_core(config.clone());
        let fine = enabled_core(config);
        let last = run_train(&coarse, 10, 1_000);
        assert_eq!(last, run_train(&fine, 10, 1_000));

        // One 40 ms advance versus forty 1 ms advances from the same state.
        {
            let mut plant = coarse.plant.lock().unwrap();
            coarse.advance(&mut plant, last + us(40_000));
        }
        for step in 1..=40 {
            let mut plant = fine.plant.lock().unwrap();
            fine.advance(&mut plant, last + us(step * 1_000));
        }
        assert!(
            (pos(&coarse) - pos(&fine)).abs() < 1e-9,
            "closed form must be cadence-independent: {} vs {}",
            pos(&coarse),
            pos(&fine)
        );
        assert!((vel(&coarse) - vel(&fine)).abs() < 1e-9);
    }

    /// The observer publishes millimetres (so a downstream encoder applies its
    /// own scale) and only on a whole-step change.
    #[rstest]
    fn observer_publishes_millimetres_on_whole_step_changes() {
        let core = enabled_core(Config {
            steps_per_mm: 10.0,
            ..quasi_static()
        });
        let seen: Arc<Mutex<Vec<f64>>> = Arc::new(Mutex::new(Vec::new()));
        {
            let sink = Arc::clone(&seen);
            core.on_position_mm
                .subscribe(move |mm| sink.lock().unwrap().push(mm));
        }

        let last = run_train(&core, 20, 1_000);
        settle(&core, last, 1_000);
        let settled = core.plant.lock().unwrap().now_ns;
        core.observe(settled);
        core.observe(settled); // nothing moved: no second publication

        let seen = seen.lock().unwrap();
        assert_eq!(seen.len(), 1, "one publication per whole-step change");
        assert!(
            (seen[0] - 2.0).abs() < 0.01,
            "20 steps at 10 steps/mm is 2 mm, got {}",
            seen[0]
        );
    }

    // ========================================================
    // Facade + handle
    // ========================================================

    /// The declared facade is exactly the three logic inputs, all sensed — the
    /// drive presents high-impedance inputs and never sources the net.
    #[rstest]
    #[case::step("STEP")]
    #[case::dir("DIR")]
    #[case::ena("ENA")]
    fn pin_facade_is_three_sensed_inputs(#[case] number: &str) {
        let motor = StepperMotor::new(Config::new(8_192.0)).expect("builds");
        let pins = motor.pins();
        assert_eq!(pins.len(), 3);
        assert_eq!(
            pins.iter().map(|p| p.number).collect::<Vec<_>>(),
            ["STEP", "DIR", "ENA"]
        );
        let decl = pins
            .iter()
            .find(|p| p.number == number)
            .expect("declared pin");
        assert_eq!(
            decl.senses_at_build(),
            Some(embsim_board::SenseKind::Digital)
        );
        assert_eq!(decl.idle, None);
    }

    /// The shaft is a handle onto the same plant, not a copy of it.
    #[rstest]
    fn shaft_shares_the_plant() {
        let motor = StepperMotor::new(quasi_static()).expect("builds");
        let shaft = motor.shaft();
        motor.core.set_ena(0, Some(Level::High));
        motor.core.step_edge(us(1_000));
        motor.core.step_edge(us(2_000));
        assert_eq!(shaft.commanded_steps(), 2);
        assert!(shaft.enabled());
        assert!(shaft.forward());
        assert!(format!("{motor:?}").contains("MotorCore"));
    }

    // ========================================================
    // The rate-carried path
    // ========================================================

    /// A constant-rate segment anchored at `since_us`.
    fn train(freq_hz: u32, since_us: u64, total: Option<u64>) -> PeriodicSchedule {
        PeriodicSchedule {
            emitted: 0,
            freq_hz,
            total,
            since_ns: us(since_us),
        }
    }

    /// Advance the plant to `to_us` without any input — what a read does.
    fn read_at(core: &MotorCore, to_us: u64) {
        let mut plant = core.plant.lock().unwrap();
        core.advance(&mut plant, us(to_us));
    }

    fn train_steps(core: &MotorCore) -> i64 {
        core.plant.lock().unwrap().train_steps
    }

    /// One rate-carried segment yields the exact pulse count at every instant,
    /// with **no** per-step traffic: the whole train below is one `set_train`.
    #[rstest]
    #[case::one_second(1_000_000, 8_192)]
    #[case::quarter_second(250_000, 2_048)]
    #[case::sub_pulse(100, 0)]
    fn a_rate_carried_segment_folds_the_exact_pulse_count(
        #[case] elapsed_us: u64,
        #[case] expect: i64,
    ) {
        let core = enabled_core(quasi_static());
        core.set_train(train(8_192, 0, None));
        read_at(&core, elapsed_us);
        assert_eq!(train_steps(&core), expect);
    }

    /// **The exactness claim.** Reading the plant at arbitrary, uneven instants
    /// must not change the count: folding is anchored to the publisher's own
    /// segment, so a hundred reads total exactly what one read would.
    #[rstest]
    fn folding_is_independent_of_how_often_the_plant_is_read() {
        let one_read = {
            let core = enabled_core(quasi_static());
            core.set_train(train(8_192, 0, None));
            read_at(&core, 1_000_000);
            train_steps(&core)
        };

        let many_reads = {
            let core = enabled_core(quasi_static());
            core.set_train(train(8_192, 0, None));
            // Deliberately uneven, prime-ish spacing: a per-read re-anchor
            // would truncate at each of these and lose pulses.
            let mut t = 0;
            for step in 1..=100u64 {
                t += 7 * step + 3;
                read_at(&core, t.min(1_000_000));
            }
            read_at(&core, 1_000_000);
            train_steps(&core)
        };

        assert_eq!(
            many_reads, one_read,
            "100 reads folded {many_reads} pulses where 1 read folded {one_read}"
        );
        assert_eq!(one_read, 8_192);
    }

    /// A finite train stops commanding exactly when its last pulse goes out —
    /// the plant needs no stall window to notice, because the segment says so.
    #[rstest]
    fn a_finite_train_completes_at_its_own_ceiling() {
        let core = enabled_core(quasi_static());
        core.set_train(train(1_000, 0, Some(10)));
        read_at(&core, 5_000);
        assert_eq!(train_steps(&core), 5, "half way in");

        read_at(&core, 10_000);
        assert_eq!(train_steps(&core), 10);
        read_at(&core, 10_000_000);
        assert_eq!(
            train_steps(&core),
            10,
            "the ceiling holds however long the plant runs on"
        );
        assert_eq!(
            core.plant.lock().unwrap().cmd,
            0.0,
            "a completed train commands nothing"
        );
    }

    /// A direction change mid-train splits the count at the instant `DIR`
    /// changed: pulses before keep their sign, pulses after take the new one,
    /// all folded from the one segment the source anchored.
    #[rstest]
    fn a_mid_train_reversal_signs_each_side_separately() {
        let core = enabled_core(quasi_static());
        core.set_train(train(1_000, 0, None));
        read_at(&core, 6_000);
        assert_eq!(train_steps(&core), 6);

        core.set_dir(us(6_000), Some(Level::Low));

        read_at(&core, 10_000);
        assert_eq!(
            train_steps(&core),
            2,
            "6 forward then 4 reverse is +2, not ±10"
        );
    }

    /// A rate change at an instant between two microseconds folds each side
    /// exactly: the outgoing train is counted up to its successor's own
    /// anchor, to the nanosecond, so the fold totals the source's count.
    #[rstest]
    fn a_rate_change_between_microseconds_folds_each_side_exactly() {
        let core = enabled_core(quasi_static());
        let first = PeriodicSchedule {
            emitted: 0,
            freq_hz: 20_000_000,
            total: None,
            since_ns: 0,
        };
        core.set_train(first);
        // A retarget at 1 000 500 ns banks the 20 010 pulses the first rate
        // put out by then, as the pulse-out peripheral does.
        let second = PeriodicSchedule {
            emitted: first.emitted_at_ns(1_000_500),
            freq_hz: 1_000_000,
            total: None,
            since_ns: 1_000_500,
        };
        assert_eq!(second.emitted, 20_010);
        core.set_train(second);
        {
            let mut plant = core.plant.lock().unwrap();
            core.advance(&mut plant, 2_000_500);
        }
        assert_eq!(
            train_steps(&core),
            i64::try_from(second.emitted_at_ns(2_000_500)).unwrap(),
            "every pulse of both rates, none lost to a microsecond floor"
        );
        assert_eq!(train_steps(&core), 21_010);
    }

    /// A disabled drive ignores the train it is being handed, and enabling
    /// mid-train picks it up at its current rate rather than waiting for the
    /// next segment — the source keeps running either way.
    #[rstest]
    fn a_disabled_drive_ignores_a_running_train_until_it_is_enabled() {
        let core = core(quasi_static());
        core.set_train(train(1_000, 0, None));
        read_at(&core, 5_000);
        assert_eq!(train_steps(&core), 0, "a disabled drive counts nothing");

        core.set_ena(us(5_000), Some(Level::High));
        read_at(&core, 9_000);
        assert_eq!(
            train_steps(&core),
            4,
            "only the pulses since the enable are counted"
        );

        core.set_ena(us(9_000), Some(Level::Low));
        read_at(&core, 20_000);
        assert_eq!(train_steps(&core), 4, "disabling stops counting again");
        assert_eq!(core.plant.lock().unwrap().cmd, 0.0);
    }

    /// A rate-carried train supplies its own rate, so the edge path's stall
    /// window — which exists only to notice a *measured* train going quiet —
    /// must not expire it. A minute of silence at a constant rate is a minute
    /// of travel, not a stall.
    #[rstest]
    fn a_rate_carried_train_is_not_subject_to_the_stall_window() {
        let core = enabled_core(quasi_static());
        // Seed the edge path first, so its stall state exists to be cleared.
        core.step_edge(us(1_000));
        core.step_edge(us(2_000));
        core.set_train(train(1_000, 2_000, None));
        {
            let plant = core.plant.lock().unwrap();
            assert!(plant.last_edge_ns.is_none(), "phase measurement cleared");
            assert!(plant.interval_ns.is_none(), "stall window cleared");
        }

        read_at(&core, 60_002_000);
        assert_eq!(train_steps(&core), 60_000, "60 s at 1 kHz");
        assert!(
            core.plant.lock().unwrap().cmd > 0.0,
            "the train is still commanding after a minute of no events"
        );
    }

    /// `commanded_steps` is the sum of both paths, and the shaft exposes the
    /// train that produced the rate-carried half.
    #[rstest]
    fn commanded_steps_sums_both_input_paths() {
        let motor = StepperMotor::new(quasi_static()).expect("builds");
        let shaft = motor.shaft();
        motor.core.set_ena(0, Some(Level::High));
        motor.core.step_edge(us(1_000));
        motor.core.step_edge(us(2_000));

        let published = train(1_000, 2_000, Some(3));
        motor.core.set_train(published);
        {
            let mut plant = motor.core.plant.lock().unwrap();
            motor.core.advance(&mut plant, us(5_000));
        }

        assert_eq!(shaft.commanded_steps(), 5, "2 edges + 3 rate-carried");
        assert_eq!(
            shaft.train(),
            Some(published),
            "the shaft reports the train as it was published, un-anchored"
        );
    }

    /// A late-delivered finite train must still travel `rate × duration`.
    ///
    /// Before the delivery-lag shift on `train_end`, integrating only from
    /// delivery to the publisher's `completes_at` lost exactly `lag × rate`
    /// of distance while `commanded_steps` (the fold) still saw every pulse —
    /// the plant's two views of the same motion disagreed (issue #43).
    #[rstest]
    fn a_late_delivered_finite_train_conserves_distance() {
        let core = enabled_core(quasi_static());
        // Pretend the plant already free-ran past the publisher's anchor.
        {
            let mut plant = core.plant.lock().unwrap();
            plant.now_ns = us(200); // 200 µs late relative to its anchor at 0
        }
        // 100 pulses at 20 kHz → 5 ms of commanded travel.
        core.set_train(train(20_000, 0, Some(100)));
        assert_eq!(
            core.plant.lock().unwrap().train_delivery_lag_ns,
            us(200),
            "delivery lag must be recorded"
        );

        // Settle well past the shifted end (completes_at 5_000 + lag 200).
        read_at(&core, 5_200);
        // Coast through the lag so velocity decays to rest.
        read_at(&core, 5_200 + 100_000);

        assert_eq!(train_steps(&core), 100, "fold still sees every pulse");
        assert!(
            (pos(&core) - 100.0).abs() < 0.01,
            "late delivery must not truncate travel; expected 100 steps, got {}",
            pos(&core)
        );
        assert!(
            vel(&core).abs() < 0.01,
            "and the carriage must be at rest, v = {}",
            vel(&core)
        );
    }
}
