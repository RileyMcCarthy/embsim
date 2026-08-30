//! Model: limit / end-of-travel switch — a **dry contact** across `COM` and
//! `NO`, with real-switch actuation hysteresis and optional contact bounce.
//!
//! [`crate::limit_switch`] is the pure-mechanism predecessor: an ideal position
//! comparator that emits `bool` transitions and leaves electrical polarity to
//! the consumer. Its provenance header lists actuation hysteresis and contact
//! bounce as **not modeled**. Both are modeled here, because a board-engine
//! component has somewhere to put them: it does not emit a boolean, it opens
//! and closes a contact on a real net.
//!
//! ```text
//!            board / harness                     this component
//!   3V3 ─┬─[ pull-up ]──┬── SENSE ──── NO ──┐
//!        │              │                   │  closed → drive NO to COM's level
//!   MCU sense pin ──────┘                   │  open   → release NO (high-Z)
//!                            GND ──── COM ──┘
//! ```
//!
//! # Provenance (mechanism model — no datasheet)
//!
//! ## Electrical behavior
//!
//! A mechanical switch is not a driver: it is a **short with a resistance**.
//! So the contact is modeled exactly that way, and the idle level is decided
//! by whatever the board provides, not by this component:
//!
//! - **Closed**: `NO` is driven to the level `COM` presents, through
//!   [`Config::contact_resistance_ohms`] of Thevenin source impedance. When the
//!   engine has a numeric solve for `COM` ([`NetState::Analog`]) that exact node
//!   voltage is reproduced; when it only has a digital projection the nominal
//!   [`Config::high_volts`] (or 0 V) stands in.
//! - **Open**: `NO` releases to high impedance and contributes nothing, so an
//!   external pull-up (or pull-down, or nothing at all) decides the idle level.
//!   A sense net with no pull-up therefore resolves
//!   [`NetState::Floating`] and raises
//!   [`embsim_board::Finding::FloatingSense`] — the physically honest result,
//!   and a real finding about the board rather than an invented level.
//! - **`COM` with no level of its own** (floating, or fought over): a closed
//!   contact conducts, but there is nothing to conduct. `NO` stays released and
//!   the situation is traced. The engine never invents a value for an unsourced
//!   node and neither does the switch.
//!
//! Only the normally-open (`NO`) terminal is declared. A normally-closed switch
//! is the same component with the actuation sense inverted in the system
//! description (swap [`Config::operate_mm`] and the sense), which keeps one
//! contact per component and one clear meaning for "closed".
//!
//! ## Actuation hysteresis
//!
//! A real end switch **operates** and **releases** at different positions — the
//! differential travel is a published parameter of every microswitch, and it is
//! what stops a carriage parked exactly on the threshold from chattering. The
//! model is a two-threshold comparator:
//!
//! ```text
//!   ActuationSense::Increasing :  close when x ≥ operate_mm
//!                                 open  when x <  release_mm   (release ≤ operate)
//!   ActuationSense::Decreasing :  close when x ≤ operate_mm
//!                                 open  when x >  release_mm   (release ≥ operate)
//! ```
//!
//! `release_mm == operate_mm` degenerates to the ideal comparator
//! [`crate::limit_switch`] implements, and is allowed; a `release_mm` on the
//! *actuated* side of `operate_mm` is rejected at construction
//! ([`crate::machine::MachineConfigError::InvertedHysteresis`]) because the
//! contact could never reopen.
//!
//! ## Contact bounce
//!
//! Metal contacts bounce for a few milliseconds on each actuation. Configured
//! with [`BounceConfig`], the contact takes the new state immediately and then
//! emits a burst of `transitions` further changes spread evenly across
//! `window_us` of virtual time, served by the engine's timer wheel, always
//! **ending on the settled state** (an odd `transitions` count lands on the
//! settled state rather than mid-bounce — a real switch does not stop halfway
//! through a bounce). Default is off, because most tests want a clean edge and
//! debounce testing should be opt-in.
//!
//! ## Parameter sources
//!
//! Thresholds, differential travel, and bounce parameters are rig and part
//! properties from the consuming system description, not constants here. For
//! the reference machine the switches appear as `END_U` / `END_L` on MaD's
//! EdgeBoard, arriving as two-wire loops (`IEND_U±`, `IEND_L±`) into isolated
//! inputs — see `Hardware/EdgeBoard/KiCad/MaD_Edge_Sheet2.kicad_sch`. That
//! topology is exactly why the honest model is "close a contact and let the
//! board bias it": the machine-side switch supplies no level of its own.
//!
//! ## Not modeled
//!
//! - **Repeatability tolerance and wear**: operate and release points are
//!   exact, not distributions, and do not drift.
//! - **Contact bounce on *release* differing from bounce on *operate***: one
//!   [`BounceConfig`] serves both edges.
//! - **Arc / wetting current, contact resistance drift, temperature.**
//!   [`Config::contact_resistance_ohms`] is a single constant.
//! - **Lever geometry and over-travel**: actuation is a function of carriage
//!   position only, and there is no travel limit past the switch.
//! - **The isolated two-wire input** on the reference board: the isolator's own
//!   current threshold and propagation delay live on the board side of the
//!   harness, not here.
//! - **Bounce without a virtual clock**: bursts need
//!   `embsim_core::virtual_clock`, so before `init` the contact settles
//!   immediately (traced), exactly as if bounce were disabled.

use std::collections::VecDeque;
use std::fmt;
use std::sync::{Arc, Mutex};

use embsim_board::{
    AttachError, Component, ComponentNetIo, Level, NetState, Ohms, PinDecl, PinHandle, PinKind,
    TheveninDrive, Volts,
};
use embsim_core::event::Observers;
use embsim_core::virtual_clock;

use super::{
    digital_level, require_positive, MachineConfigError, DEFAULT_HIGH_VOLTS,
    DEFAULT_INPUT_THRESHOLD_VOLTS,
};

// ============================================================
// Configuration
// ============================================================

/// Default closed-contact resistance (Ω). Small enough to dominate any
/// practical pull-up (so the closed contact projects a clean
/// [`NetState::Driven`] rather than escalating to a divided voltage), and
/// non-zero because a real contact is not a perfect short.
pub const DEFAULT_CONTACT_RESISTANCE_OHMS: Ohms = 0.1;

/// Which direction of carriage travel actuates the switch.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ActuationSense {
    /// The contact closes as position **increases** past
    /// [`Config::operate_mm`] — a maximum / far end stop.
    Increasing,
    /// The contact closes as position **decreases** past
    /// [`Config::operate_mm`] — a minimum / home end stop.
    Decreasing,
}

impl ActuationSense {
    /// Name used in [`MachineConfigError::InvertedHysteresis`].
    fn label(self) -> &'static str {
        match self {
            ActuationSense::Increasing => "Increasing",
            ActuationSense::Decreasing => "Decreasing",
        }
    }
}

/// Contact-bounce burst shape. Off by default.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BounceConfig {
    /// Contact changes emitted *after* the immediate actuation edge, spread
    /// evenly across `window_us`. An odd count still lands on the settled
    /// state. Zero degenerates to a clean edge.
    pub transitions: u32,
    /// Virtual-time span (µs) of the burst.
    pub window_us: u64,
}

/// End-switch configuration. Build with [`Config::new`] and override the
/// fields a particular switch needs.
#[derive(Debug, Clone)]
pub struct Config {
    /// Carriage position (mm) at which the contact closes.
    pub operate_mm: f64,
    /// Carriage position (mm) at which a closed contact reopens. Must be on
    /// the un-actuated side of (or equal to) [`Config::operate_mm`] for the
    /// configured [`Config::sense`].
    pub release_mm: f64,
    /// Direction of travel that actuates the switch.
    pub sense: ActuationSense,
    /// Closed-contact Thevenin source impedance
    /// ([`DEFAULT_CONTACT_RESISTANCE_OHMS`]).
    pub contact_resistance_ohms: Ohms,
    /// Contact-bounce burst, or `None` for a clean edge.
    pub bounce: Option<BounceConfig>,
    /// Logic threshold used to read `COM` ([`DEFAULT_INPUT_THRESHOLD_VOLTS`]).
    pub input_threshold_volts: Volts,
    /// Voltage driven for a `COM` known only as a digital high
    /// ([`DEFAULT_HIGH_VOLTS`]).
    pub high_volts: Volts,
}

impl Config {
    /// A switch with no differential travel (`release_mm == operate_mm`) and
    /// no bounce — the ideal comparator, which every field can then relax
    /// toward the real part.
    pub fn new(operate_mm: f64, sense: ActuationSense) -> Self {
        Self {
            operate_mm,
            release_mm: operate_mm,
            sense,
            contact_resistance_ohms: DEFAULT_CONTACT_RESISTANCE_OHMS,
            bounce: None,
            input_threshold_volts: DEFAULT_INPUT_THRESHOLD_VOLTS,
            high_volts: DEFAULT_HIGH_VOLTS,
        }
    }

    /// Set the release point, giving the switch differential travel.
    pub fn with_release(mut self, release_mm: f64) -> Self {
        self.release_mm = release_mm;
        self
    }

    /// Give the switch a contact-bounce burst.
    pub fn with_bounce(mut self, bounce: BounceConfig) -> Self {
        self.bounce = Some(bounce);
        self
    }

    /// Reject a configuration that cannot describe a switch.
    fn validate(&self) -> Result<(), MachineConfigError> {
        require_positive("contact_resistance_ohms", self.contact_resistance_ohms)?;
        require_positive("input_threshold_volts", self.input_threshold_volts)?;
        require_positive("high_volts", self.high_volts)?;
        if !self.operate_mm.is_finite() {
            return Err(MachineConfigError::NotPositive {
                field: "operate_mm",
                value: self.operate_mm,
            });
        }
        if !self.release_mm.is_finite() {
            return Err(MachineConfigError::NotPositive {
                field: "release_mm",
                value: self.release_mm,
            });
        }
        // A release point on the actuated side of the operate point would keep
        // the contact closed forever.
        let inverted = match self.sense {
            ActuationSense::Increasing => self.release_mm > self.operate_mm,
            ActuationSense::Decreasing => self.release_mm < self.operate_mm,
        };
        if inverted {
            return Err(MachineConfigError::InvertedHysteresis {
                sense: self.sense.label(),
                operate_mm: self.operate_mm,
                release_mm: self.release_mm,
            });
        }
        if let Some(bounce) = &self.bounce {
            if bounce.transitions > 0 && bounce.window_us == 0 {
                return Err(MachineConfigError::Zero { field: "window_us" });
            }
        }
        Ok(())
    }

    /// Contact state for a carriage position, given the current state — the
    /// two-threshold comparator from the module docs.
    fn actuated(&self, position_mm: f64, closed: bool) -> bool {
        match (self.sense, closed) {
            (ActuationSense::Increasing, false) => position_mm >= self.operate_mm,
            (ActuationSense::Increasing, true) => position_mm >= self.release_mm,
            (ActuationSense::Decreasing, false) => position_mm <= self.operate_mm,
            (ActuationSense::Decreasing, true) => position_mm <= self.release_mm,
        }
    }
}

// ============================================================
// Pin facade
// ============================================================

/// `COM` senses whatever the board biases the common terminal to; `NO` is the
/// normally-open terminal the contact drives onto the sense net.
///
/// `NO` is declared [`PinKind::DigitalOut`] because it is the only kind that
/// can source a net at all — but a switch **idles open**, and the engine
/// assigns every `DigitalOut` an idle-high drive at assembly (the UART-TX
/// convention). [`EndSwitch::attach`] therefore releases `NO` as its first
/// action. On the live path that release lands in the first resolution pass
/// after attach, before any harness traffic; the
/// [`embsim_board::System::build`] analysis snapshot, which resolves *before*
/// attach, still shows the pin idle-high. Read build-time findings for this
/// net with that in mind.
pub const END_SWITCH_PINS: [PinDecl; 2] = [
    PinDecl {
        number: "COM",
        name: None,
        kind: PinKind::DigitalIn,
        stream: None,
        drive_impedance: None,
    },
    PinDecl {
        number: "NO",
        name: None,
        kind: PinKind::DigitalOut,
        stream: None,
        // The contact resistance is applied per drive (it is configuration,
        // not a `&'static` constant), so the declaration carries no default.
        drive_impedance: None,
    },
];

// ============================================================
// Contact state
// ============================================================

/// Everything the contact needs: its state, the position driving it, the level
/// `COM` offers, and any bounce burst still in flight.
#[derive(Debug)]
struct Contact {
    /// Settled contact state (what the position says), ignoring bounce.
    closed: bool,
    /// Contact state currently driven onto the net (differs from `closed`
    /// only mid-bounce).
    driven: bool,
    /// Last fed carriage position (mm).
    position_mm: Option<f64>,
    /// Last engine-published state of the `COM` net.
    com: NetState,
    /// `NO` pin handle, `None` until attach.
    no: Option<PinHandle>,
    /// Engine I/O handle, kept for `schedule_at` during a bounce burst.
    io: Option<ComponentNetIo>,
    /// Bounce burst still to emit: `(virtual deadline µs, contact state)`.
    burst: VecDeque<(u64, bool)>,
}

/// Shared contact state + observers behind the component and every
/// [`EndSwitchActuator`].
struct SwitchCore {
    config: Config,
    contact: Mutex<Contact>,
    on_change: Observers<bool>,
}

impl fmt::Debug for SwitchCore {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("SwitchCore")
            .field("config", &self.config)
            .field("contact", &self.contact)
            .field("observers", &self.on_change.len())
            .finish()
    }
}

impl SwitchCore {
    /// The voltage a closed contact reproduces from a `COM` state, or `None`
    /// when `COM` offers no level at all.
    ///
    /// A closed contact is a short, not a level shifter: a numeric solve is
    /// reproduced exactly, and only a `COM` known solely as a digital
    /// projection falls back to the nominal rail.
    fn contact_volts(&self, com: NetState) -> Option<Volts> {
        let level = digital_level(com, self.config.input_threshold_volts)?;
        Some(match (level, com) {
            (_, NetState::Analog(volts)) => volts,
            (Level::High, _) => self.config.high_volts,
            (Level::Low, _) => 0.0,
        })
    }

    /// Drive (or release) `NO` for a contact state. Closed reproduces `COM`'s
    /// level through the contact resistance; open contributes nothing.
    fn apply(&self, contact: &mut Contact, closed: bool) {
        contact.driven = closed;
        let Some(no) = &contact.no else {
            return;
        };
        let drive = closed
            .then(|| self.contact_volts(contact.com))
            .flatten()
            .map(|volts| TheveninDrive {
                volts,
                impedance: self.config.contact_resistance_ohms,
            });
        if closed && drive.is_none() {
            // A closed contact conducts, but an unbiased COM has nothing to
            // conduct. Release rather than invent a level.
            tracing::debug!(
                com = ?contact.com,
                "end_switch: closed contact with no level on COM; NO stays released"
            );
        }
        no.set_drive(drive);
    }

    /// Build the bounce burst for a settled state: `transitions` alternating
    /// changes spread across the window, always ending on `settled`.
    fn burst_for(&self, now_us: u64, settled: bool) -> VecDeque<(u64, bool)> {
        let Some(bounce) = &self.config.bounce else {
            return VecDeque::new();
        };
        if bounce.transitions == 0 || bounce.window_us == 0 {
            return VecDeque::new();
        }
        let count = bounce.transitions;
        let mut burst = VecDeque::with_capacity(count as usize);
        for index in 1..=count {
            let deadline = now_us + bounce.window_us * u64::from(index) / u64::from(count);
            // Alternate away from and back to the settled state; the final
            // entry is forced to `settled` so an odd count still lands there.
            let state = if index == count {
                settled
            } else if index % 2 == 1 {
                !settled
            } else {
                settled
            };
            burst.push_back((deadline, state));
        }
        burst
    }

    /// Arm the engine for the next queued bounce deadline.
    fn arm(&self, contact: &Contact) {
        let Some((deadline, _)) = contact.burst.front() else {
            return;
        };
        if let Some(io) = &contact.io {
            io.schedule_at(*deadline);
        }
    }

    /// Feed a carriage position; actuate (with hysteresis) if it crosses.
    fn set_position(&self, position_mm: f64) {
        if !position_mm.is_finite() {
            tracing::warn!(position_mm, "end_switch: non-finite position ignored");
            return;
        }
        let settled = {
            let mut contact = self.contact.lock().unwrap();
            contact.position_mm = Some(position_mm);
            let next = self.config.actuated(position_mm, contact.closed);
            if next == contact.closed {
                return;
            }
            contact.closed = next;

            let now_us = virtual_clock::is_initialized().then(virtual_clock::virtual_us);
            match now_us {
                Some(now_us) if self.config.bounce.is_some() => {
                    contact.burst = self.burst_for(now_us, next);
                    if contact.burst.is_empty() {
                        self.apply(&mut contact, next);
                    } else {
                        // The burst's own first entry carries the settled or
                        // bounced state; drive the leading edge now so the
                        // actuation is visible immediately.
                        self.apply(&mut contact, next);
                        self.arm(&contact);
                    }
                }
                _ => {
                    if self.config.bounce.is_some() {
                        tracing::debug!(
                            "end_switch: virtual clock not initialized; contact settles \
                             immediately instead of bouncing"
                        );
                    }
                    contact.burst.clear();
                    self.apply(&mut contact, next);
                }
            }
            next
        };
        // Observers see one publication per actuation — the settled state, not
        // each bounce edge (the chatter is visible electrically, on the net).
        self.on_change.emit(settled);
    }

    /// Timer-wheel delivery: apply every bounce entry now due, then re-arm.
    fn on_wake(&self, now_us: u64) {
        let mut contact = self.contact.lock().unwrap();
        while contact
            .burst
            .front()
            .is_some_and(|&(deadline, _)| deadline <= now_us)
        {
            let (_, state) = contact.burst.pop_front().expect("front matched above");
            self.apply(&mut contact, state);
        }
        self.arm(&contact);
    }

    /// A new `COM` state: record it, and re-apply if the contact is currently
    /// conducting (a `COM` that comes up late must reach the sense net).
    fn set_com(&self, state: NetState) {
        let mut contact = self.contact.lock().unwrap();
        contact.com = state;
        if contact.driven {
            self.apply(&mut contact, true);
        }
    }
}

// ============================================================
// Actuator handle
// ============================================================

/// Actuation-side handle to an [`EndSwitch`].
///
/// Cloned out of the component *before* it is handed to
/// [`embsim_board::System`], then fed carriage position — typically wired
/// straight to a motor's observer:
///
/// ```rust,ignore
/// let upper = switch.actuator();
/// motor.shaft().on_position_change(move |mm| upper.set_position_mm(mm));
/// ```
#[derive(Clone, Debug)]
pub struct EndSwitchActuator {
    core: Arc<SwitchCore>,
}

impl EndSwitchActuator {
    /// Feed the carriage position (mm) the switch is watching.
    pub fn set_position_mm(&self, position_mm: f64) {
        self.core.set_position(position_mm);
    }

    /// The settled contact state: `true` = closed (actuated). Mid-bounce this
    /// is the state the contact is *settling to*; use
    /// [`EndSwitchActuator::driving_closed`] for what is on the net right now.
    pub fn is_closed(&self) -> bool {
        self.core.contact.lock().unwrap().closed
    }

    /// The contact state the switch is currently presenting — differs from
    /// [`EndSwitchActuator::is_closed`] only while a bounce burst is in
    /// flight. Whether a closed contact actually reaches the net still depends
    /// on `COM` having a level (see the module docs).
    pub fn driving_closed(&self) -> bool {
        self.core.contact.lock().unwrap().driven
    }

    /// Bounce transitions still queued for the timer wheel.
    pub fn pending_bounce(&self) -> usize {
        self.core.contact.lock().unwrap().burst.len()
    }

    /// Last fed carriage position (mm), or `None` before the first update.
    pub fn position_mm(&self) -> Option<f64> {
        self.core.contact.lock().unwrap().position_mm
    }

    /// Subscribe to settled contact transitions. One publication per
    /// actuation — bounce chatter is electrical, not an event. Multiple
    /// subscribers are appended, never overwritten.
    pub fn on_change(&self, callback: impl Fn(bool) + Send + 'static) {
        self.core.on_change.subscribe(callback);
    }
}

// ============================================================
// Component
// ============================================================

/// A limit / end-of-travel switch as a live board-engine component: a dry
/// contact across `COM` and `NO`, actuated by carriage position with
/// hysteresis and optional bounce.
#[derive(Debug)]
pub struct EndSwitch {
    core: Arc<SwitchCore>,
}

impl EndSwitch {
    /// Create a switch from a validated configuration.
    pub fn new(config: Config) -> Result<Self, MachineConfigError> {
        config.validate()?;
        tracing::info!(
            operate_mm = config.operate_mm,
            release_mm = config.release_mm,
            sense = ?config.sense,
            bounce = config.bounce.is_some(),
            "end_switch: init"
        );
        Ok(Self {
            core: Arc::new(SwitchCore {
                config,
                contact: Mutex::new(Contact {
                    closed: false,
                    driven: false,
                    position_mm: None,
                    com: NetState::Floating,
                    no: None,
                    io: None,
                    burst: VecDeque::new(),
                }),
                on_change: Observers::new(),
            }),
        })
    }

    /// A handle for actuating this switch.
    pub fn actuator(&self) -> EndSwitchActuator {
        EndSwitchActuator {
            core: Arc::clone(&self.core),
        }
    }

    /// The validated configuration in force.
    pub fn config(&self) -> &Config {
        &self.core.config
    }
}

impl Component for EndSwitch {
    fn pins(&self) -> &[PinDecl] {
        &END_SWITCH_PINS
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        {
            let mut contact = self.core.contact.lock().unwrap();
            contact.no = Some(io.pin("NO")?);
            contact.io = Some(io.clone());
            // A switch idles OPEN. The engine gave this `DigitalOut` an
            // idle-high drive at assembly, so releasing it is the first thing
            // that must happen — see the `END_SWITCH_PINS` docs.
            let closed = contact.closed;
            self.core.apply(&mut contact, closed);
        }
        {
            let core = Arc::clone(&self.core);
            io.on_sense("COM", move |state| core.set_com(state))?;
        }
        {
            let core = Arc::clone(&self.core);
            io.on_wake(move |now_us| core.on_wake(now_us));
        }
        Ok(())
    }
}

// ============================================================
// Tests
// ============================================================
//
// The comparator, the bounce schedule, and the contact's drive decision are
// exercised here without an engine (the `NO` handle is absent, so drives are
// bookkeeping only). What the *net* does — open = floating or pull-up-decided,
// closed = driven to COM's level — is covered against a live system in
// `models/tests/machine_live_system.rs`.

#[cfg(test)]
mod tests {
    use rstest::rstest;

    use super::*;

    fn switch(config: Config) -> EndSwitch {
        EndSwitch::new(config).expect("valid config")
    }

    /// Records every settled transition.
    fn recorder(actuator: &EndSwitchActuator) -> Arc<Mutex<Vec<bool>>> {
        let log: Arc<Mutex<Vec<bool>>> = Arc::new(Mutex::new(Vec::new()));
        let sink = Arc::clone(&log);
        actuator.on_change(move |closed| sink.lock().unwrap().push(closed));
        log
    }

    // ========================================================
    // Hysteresis
    // ========================================================

    /// An increasing-sense switch closes at the operate point and holds closed
    /// until the position falls below the release point.
    #[rstest]
    fn increasing_sense_operates_and_releases_at_two_points() {
        let sw = switch(Config::new(10.0, ActuationSense::Increasing).with_release(9.5));
        let actuator = sw.actuator();
        let log = recorder(&actuator);

        actuator.set_position_mm(9.99);
        assert!(!actuator.is_closed(), "below the operate point");
        actuator.set_position_mm(10.0);
        assert!(actuator.is_closed(), "at the operate point");
        actuator.set_position_mm(9.6);
        assert!(actuator.is_closed(), "still above the release point");
        actuator.set_position_mm(9.4);
        assert!(!actuator.is_closed(), "below the release point");

        assert_eq!(*log.lock().unwrap(), vec![true, false]);
    }

    /// A decreasing-sense switch (a home end stop) mirrors it.
    #[rstest]
    fn decreasing_sense_operates_and_releases_at_two_points() {
        let sw = switch(Config::new(-10.0, ActuationSense::Decreasing).with_release(-9.5));
        let actuator = sw.actuator();

        actuator.set_position_mm(-9.99);
        assert!(!actuator.is_closed());
        actuator.set_position_mm(-10.0);
        assert!(actuator.is_closed());
        actuator.set_position_mm(-9.6);
        assert!(actuator.is_closed(), "still below the release point");
        actuator.set_position_mm(-9.4);
        assert!(!actuator.is_closed());
    }

    /// The reason hysteresis exists: a carriage parked on the threshold and
    /// wobbling inside the differential travel actuates **once**, where the
    /// ideal comparator would chatter on every wobble.
    #[rstest]
    fn hysteresis_suppresses_chatter_at_the_threshold() {
        let wobble = [9.98, 10.02, 9.9, 10.05, 9.7, 10.01, 9.6];

        let real = switch(Config::new(10.0, ActuationSense::Increasing).with_release(9.5));
        let real_actuator = real.actuator();
        let real_log = recorder(&real_actuator);
        for position in wobble {
            real_actuator.set_position_mm(position);
        }
        assert_eq!(
            *real_log.lock().unwrap(),
            vec![true],
            "differential travel must absorb the wobble"
        );
        assert!(real_actuator.is_closed());

        // Same wobble, zero differential travel: the ideal comparator toggles
        // on every crossing. This is the behavior `crate::limit_switch` has.
        let ideal = switch(Config::new(10.0, ActuationSense::Increasing));
        let ideal_actuator = ideal.actuator();
        let ideal_log = recorder(&ideal_actuator);
        for position in wobble {
            ideal_actuator.set_position_mm(position);
        }
        assert_eq!(
            *ideal_log.lock().unwrap(),
            vec![true, false, true, false, true, false],
            "zero differential travel is the chattering comparator"
        );
    }

    /// Repeating a position never re-publishes, and a non-finite position is
    /// refused.
    #[rstest]
    fn repeated_and_non_finite_positions_are_quiet() {
        let sw = switch(Config::new(10.0, ActuationSense::Increasing));
        let actuator = sw.actuator();
        let log = recorder(&actuator);
        actuator.set_position_mm(11.0);
        actuator.set_position_mm(11.0);
        actuator.set_position_mm(12.0);
        actuator.set_position_mm(f64::NAN);
        assert_eq!(*log.lock().unwrap(), vec![true]);
        assert_eq!(
            actuator.position_mm(),
            Some(12.0),
            "NaN must not be latched"
        );
    }

    /// The comparator predicate, as a matrix over sense × state × position.
    #[rstest]
    #[case::inc_open_below(ActuationSense::Increasing, false, 9.0, false)]
    #[case::inc_open_at(ActuationSense::Increasing, false, 10.0, true)]
    #[case::inc_closed_in_band(ActuationSense::Increasing, true, 9.7, true)]
    #[case::inc_closed_below_release(ActuationSense::Increasing, true, 9.4, false)]
    #[case::dec_open_above(ActuationSense::Decreasing, false, 11.0, false)]
    #[case::dec_open_at(ActuationSense::Decreasing, false, 10.0, true)]
    #[case::dec_closed_in_band(ActuationSense::Decreasing, true, 10.3, true)]
    #[case::dec_closed_above_release(ActuationSense::Decreasing, true, 10.6, false)]
    fn comparator_matrix(
        #[case] sense: ActuationSense,
        #[case] closed: bool,
        #[case] position_mm: f64,
        #[case] expect: bool,
    ) {
        let release = match sense {
            ActuationSense::Increasing => 9.5,
            ActuationSense::Decreasing => 10.5,
        };
        let config = Config::new(10.0, sense).with_release(release);
        assert_eq!(config.actuated(position_mm, closed), expect);
    }

    // ========================================================
    // Bounce
    // ========================================================

    /// A bounce burst is `transitions` entries spread evenly across the
    /// window, alternating away from and back to the settled state, and always
    /// ending on it.
    #[rstest]
    #[case::even(4, true)]
    #[case::odd(5, true)]
    #[case::even_open(4, false)]
    #[case::odd_open(3, false)]
    #[case::single(1, true)]
    fn bounce_burst_alternates_and_lands_settled(#[case] transitions: u32, #[case] settled: bool) {
        let sw = switch(
            Config::new(10.0, ActuationSense::Increasing).with_bounce(BounceConfig {
                transitions,
                window_us: 1_000,
            }),
        );
        let burst = sw.core.burst_for(0, settled);
        assert_eq!(burst.len() as u32, transitions);
        assert_eq!(
            burst.back().expect("non-empty").1,
            settled,
            "the burst must end on the settled state, never mid-bounce"
        );
        // Deadlines are strictly increasing and fit inside the window.
        let mut previous = 0;
        for &(deadline, _) in &burst {
            assert!(deadline > previous, "deadlines must advance");
            assert!(deadline <= 1_000, "and stay inside the window");
            previous = deadline;
        }
        // Every entry but the last alternates.
        for (index, &(_, state)) in burst.iter().enumerate().take(burst.len() - 1) {
            let expect = if index % 2 == 0 { !settled } else { settled };
            assert_eq!(state, expect, "entry {index} must alternate");
        }
    }

    /// Bounce disabled (the default) produces no burst at all.
    #[rstest]
    fn no_bounce_config_means_no_burst() {
        let sw = switch(Config::new(10.0, ActuationSense::Increasing));
        assert!(sw.core.burst_for(0, true).is_empty());
    }

    /// A zero-transition burst is legal and degenerates to a clean edge.
    #[rstest]
    fn zero_transition_bounce_is_a_clean_edge() {
        let sw = switch(
            Config::new(10.0, ActuationSense::Increasing).with_bounce(BounceConfig {
                transitions: 0,
                window_us: 1_000,
            }),
        );
        assert!(sw.core.burst_for(0, true).is_empty());
    }

    /// The wake handler drains every due entry in order and re-arms for the
    /// rest, so the contact walks the burst rather than jumping to the end.
    #[rstest]
    fn wake_drains_due_bounce_entries_in_order() {
        let sw = switch(
            Config::new(10.0, ActuationSense::Increasing).with_bounce(BounceConfig {
                transitions: 4,
                window_us: 400,
            }),
        );
        let actuator = sw.actuator();
        {
            let mut contact = sw.core.contact.lock().unwrap();
            contact.closed = true;
            contact.burst = sw.core.burst_for(0, true);
        }
        assert_eq!(actuator.pending_bounce(), 4);

        // Deadlines are 100/200/300/400: false, true, false, true.
        sw.core.on_wake(100);
        assert!(!actuator.driving_closed(), "first bounce opens the contact");
        assert_eq!(actuator.pending_bounce(), 3);

        sw.core.on_wake(350); // two entries due at once
        assert!(
            !actuator.driving_closed(),
            "drained in order, ending at 300"
        );
        assert_eq!(actuator.pending_bounce(), 1);

        sw.core.on_wake(400);
        assert!(actuator.driving_closed(), "the burst settles closed");
        assert_eq!(actuator.pending_bounce(), 0);
        assert!(actuator.is_closed());
    }

    // ========================================================
    // Contact drive decision
    // ========================================================

    /// The voltage a closed contact reproduces: an exact numeric solve when
    /// the engine has one, the nominal rail when it only has a level, and
    /// nothing at all when `COM` has no level.
    #[rstest]
    #[case::analog(NetState::Analog(2.5), Some(2.5))]
    #[case::analog_ground(NetState::Analog(0.0), Some(0.0))]
    #[case::driven_high(NetState::Driven(Level::High), Some(DEFAULT_HIGH_VOLTS))]
    #[case::driven_low(NetState::Driven(Level::Low), Some(0.0))]
    #[case::pulled_high(NetState::Pulled(Level::High, 10_000.0), Some(DEFAULT_HIGH_VOLTS))]
    #[case::floating(NetState::Floating, None)]
    #[case::contention(NetState::Contention, None)]
    fn closed_contact_reproduces_com(#[case] com: NetState, #[case] expect: Option<Volts>) {
        let sw = switch(Config::new(10.0, ActuationSense::Increasing));
        assert_eq!(sw.core.contact_volts(com), expect);
        sw.core.set_com(com);
        assert_eq!(sw.core.contact.lock().unwrap().com, com);
    }

    /// The nominal rail used for a level-only `COM` is configuration, so a 5 V
    /// sense loop reproduces 5 V rather than 3.3 V.
    #[rstest]
    fn nominal_rail_for_a_level_only_com_is_configurable() {
        let sw = switch(Config {
            high_volts: 5.0,
            ..Config::new(10.0, ActuationSense::Increasing)
        });
        assert_eq!(
            sw.core.contact_volts(NetState::Driven(Level::High)),
            Some(5.0)
        );
    }

    /// A `COM` that comes up after the contact closed still reaches the sense
    /// net: `set_com` re-applies while the contact is conducting.
    #[rstest]
    fn late_com_reapplies_a_conducting_contact() {
        let sw = switch(Config::new(10.0, ActuationSense::Increasing));
        let actuator = sw.actuator();
        actuator.set_position_mm(11.0);
        assert!(actuator.driving_closed());
        sw.core.set_com(NetState::Analog(0.0));
        assert!(
            actuator.driving_closed(),
            "the contact stays closed across a COM update"
        );
    }

    // ========================================================
    // Configuration + facade
    // ========================================================

    #[rstest]
    #[case::inverted_increasing(
        Config::new(10.0, ActuationSense::Increasing).with_release(11.0),
        "Increasing"
    )]
    #[case::inverted_decreasing(
        Config::new(10.0, ActuationSense::Decreasing).with_release(9.0),
        "Decreasing"
    )]
    #[case::zero_contact(
        Config { contact_resistance_ohms: 0.0, ..Config::new(10.0, ActuationSense::Increasing) },
        "contact_resistance_ohms"
    )]
    #[case::nan_operate(Config::new(f64::NAN, ActuationSense::Increasing), "operate_mm")]
    #[case::zero_bounce_window(
        Config::new(10.0, ActuationSense::Increasing).with_bounce(BounceConfig {
            transitions: 3,
            window_us: 0,
        }),
        "window_us"
    )]
    fn invalid_config_is_rejected_loudly(#[case] config: Config, #[case] needle: &str) {
        let error = EndSwitch::new(config).expect_err("must reject");
        assert!(
            error.to_string().contains(needle),
            "the error must mention {needle}: {error}"
        );
    }

    /// Zero differential travel is legal — it is the ideal comparator.
    #[rstest]
    fn zero_differential_travel_is_allowed() {
        assert!(EndSwitch::new(Config::new(10.0, ActuationSense::Increasing)).is_ok());
        assert!(EndSwitch::new(Config::new(10.0, ActuationSense::Decreasing)).is_ok());
    }

    /// The facade is one sensed common terminal and one driven normally-open
    /// terminal, and the switch starts open.
    #[rstest]
    fn facade_is_com_in_and_no_out() {
        let sw = switch(Config::new(10.0, ActuationSense::Increasing));
        assert_eq!(sw.pins().len(), 2);
        assert_eq!(sw.pins()[0].number, "COM");
        assert_eq!(sw.pins()[0].kind, PinKind::DigitalIn);
        assert_eq!(sw.pins()[1].number, "NO");
        assert_eq!(sw.pins()[1].kind, PinKind::DigitalOut);
        assert!(!sw.actuator().is_closed(), "a switch starts open");
        assert!(!sw.actuator().driving_closed());
        assert_eq!(sw.actuator().position_mm(), None);
        assert!(format!("{sw:?}").contains("SwitchCore"));
    }
}
