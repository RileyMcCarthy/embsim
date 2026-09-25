//! Component trait, pin declarations, and the per-component net I/O handle.
//!
//! Every part on a board — including the MCU — is a [`Component`] with
//! declared pins. At build time the board validates each component's
//! [`PinDecl`] facade against the netlist in BOTH directions and calls
//! [`Component::attach`] with a [`ComponentNetIo`] so the component can grab
//! typed pin handles **before it is shared** (pre-`Arc`, no interior
//! mutability needed) and fail loudly on facade mismatch.
//!
//! Handles come in two flavors, decided by which `System` path attached the
//! component: `System::start` wires them to the live net engine
//! ([`crate::engine`]) so drives/schedules/sense subscriptions route to the
//! engine thread; `System::build` (the build-time analysis pass) hands out
//! inert handles whose `sense` reads the build-resolved snapshot and whose
//! drives/schedules are traced and dropped.
//!
//! A pin is declared once, as a [`PinRole`] and the static facts beside it
//! on [`PinDecl`] (`NODES.md` §11): its idle drive, its input port and
//! clamps, its capacitance, its thresholds, the reference and supply pins
//! they are measured against, and whether it can source or sink. There are
//! no pin kinds: what a pin reads and drives follows from those
//! declarations ([`PinDecl::senses_at_build`],
//! [`PinDecl::reads_when_subscribed`]).
//!
//! What a sensing pin is handed is a [`Sense`] (`NODES.md` §10, §11): the
//! node's voltage against the pin's declared reference, `None` when no
//! source reaches it, the square wave when it carries one, and the instant.
//! The level is the receiver's own projection through its declared
//! [`Thresholds`] — its hysteresis chosen by the level it last read, its
//! [`DeadBand`] policy for the band between ([`Sense::level`],
//! [`PinHandle::level`], [`DigitalReceiver`]). The engine's projection,
//! [`NetState`], is the engine's report — `BuiltSystem::net_state`, the
//! event log, the goldens, an instrument's
//! [`ComponentNetIo::on_net_report`] — never what a node reads.
//!
//! A step clock is not a second surface: it is the third encoding of the one
//! per-instant message, [`Drive::Periodic`] — two Thevenin phases and the
//! integer schedule that alternates them — published through
//! [`PinHandle::drive`] like every other drive and handed to a sensing pin
//! as a [`PeriodicSense`]. See [`Drive::Periodic`] for why a rate and not
//! edges; a UART is edges, framed onto the net by
//! [`crate::SerialLevelBridge`].

use std::collections::HashMap;
use std::fmt;
use std::sync::{Arc, Weak};

use crate::diagnostics::SenseKind;
use crate::engine::{Command, ComponentId, Delivery, EndpointId, EngineLink, ReadKind};
use crate::net::{Amps, Level, NetId, NetState, NetVolts, Ohms, TheveninDrive, Volts};

pub use crate::net::PeriodicSchedule;

// ============================================================
// Pin declarations
// ============================================================

/// What a declared pin **is** to the solve (`NODES.md` §11, `PinRole`) —
/// the one fact the role decides; everything else a pin is electrically is
/// a declaration beside it on [`PinDecl`] (its idle drive, its input port,
/// its clamps, its capacitance, its thresholds, its reference and supply
/// pins, whether it can source or sink).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PinRole {
    /// A signal pin. It has a drive slot — a sense callback may drive, and
    /// a driver publishes through it — and how it reads its net is its
    /// declarations': a pin that can neither source nor sink is a **sense**
    /// ([`PinDecl::senses_at_build`]) — a digital one through its
    /// [`PinDecl::thresholds`], an analog reader, which asks for the solved
    /// voltage, when it declares none; a pin that drives **and** declares
    /// thresholds is bidirectional, and reads its net once it subscribes
    /// while it idles released ([`PinDecl::reads_when_subscribed`]).
    Signal,
    /// Consumes a power domain: the net is sensed as a supply; no drive
    /// slot.
    PowerIn,
    /// Generates a power domain: the net is a **declared terminal** — a
    /// cluster of its own and a boundary of every cluster around it
    /// (`NODES.md` "Three rules the taxonomy rests on", 1) — held at what the
    /// pin's slot publishes, from its [`PinDecl::idle`] until the part
    /// drives it.
    PowerOut,
    /// A terminal that contributes nothing electrical of its own: a passive
    /// primitive's pad, a no-connect, a pin a model reads only at attach
    /// (a feedback divider, a select strap). No slot, no sense.
    Passive,
}

/// The one per-instant message a pin publishes onto its net (`NODES.md`
/// §10/§11): what the pin *is* electrically at the instant it published,
/// sequenced through the engine's drive queue like every other drive. Three
/// encodings, one command (`DESIGN.md` rule 2).
///
/// A released pin (high-Z) is the absence of a drive — [`PinHandle::release`]
/// — and a [`Drive::Thevenin`] with a non-finite impedance is normalised to
/// exactly that at the drive slot, so it is never ranked against anything.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Drive {
    /// A linear port: open-circuit voltage behind a source impedance. Push-
    /// pull pads, sinks (open-drain low = `{0 V, R_on}`, high = released),
    /// pull modes, rails, straps and stuck-at faults are all this.
    Thevenin(TheveninDrive),
    /// A Norton injection with no shunt: `amps` **into** the net, stamped
    /// straight onto the right-hand side of the cluster solve. It carries no
    /// open-circuit voltage and reaches nothing by itself — a node no
    /// Thevenin source reaches stays [`NetState::Floating`] whatever is
    /// injected into it, with a [`crate::Finding::CurrentIntoFloatingNode`].
    /// An instrument (a current regulator, a pad's current-source pull
    /// mode, a load), never the normal path.
    Current {
        /// Current into the net, amperes.
        amps: Amps,
    },
    /// A square wave: the Thevenin port the pin presents in its high phase,
    /// the one it presents in its low phase, and the integer schedule that
    /// alternates them — a declared compression of a Thevenin edge train,
    /// published **once per rate change** (`start` / retarget / `stop`) and
    /// never per edge (`NODES.md` §10 row 3, `sil-unified-drive.md`). Each
    /// phase resolves through rule 2 like any Thevenin drive: a pull-up
    /// follows the square wave; a comparable static source, or a second
    /// periodic source, is [`NetState::Contention`] — a sustained fight for
    /// half of every cycle. The net publishes [`NetState::Periodic`], a
    /// sensing pin is handed a [`PeriodicSense`] — each phase's voltage and
    /// the segment — and a consumer integrates the segment itself.
    ///
    /// # Why a rate and not edges
    ///
    /// A step clock is the one digital signal whose *information* is its
    /// frequency and whose edge count runs far ahead of anything else on the
    /// board. On the reference machine — 8192 steps/mm — a single mm/s of
    /// carriage speed is 8192 edges/s, each of which would be a drive
    /// command, a resolution pass over the STEP cluster, and a sense delivery
    /// through the single-writer engine; 820 000 edges a second was measured
    /// and refused (`DESIGN.md` §6), for a signal whose consumer only ever
    /// reconstructs `frequency × time` from them. So the drive carries the
    /// *segment*, and the count stays **exact**: [`PeriodicSchedule::emitted_at_ns`]
    /// is the same integer arithmetic the pulse-out peripheral hands the
    /// firmware, so an encoder fed from it cannot drift from the firmware's
    /// own view.
    ///
    /// ```
    /// use embsim_board::{Drive, PeriodicSchedule, TheveninDrive};
    ///
    /// // 8192 steps/s, unbounded, from t = 1 ms with 0 emitted, swinging
    /// // 0–3.3 V through 25 Ω.
    /// let drive = Drive::Periodic {
    ///     hi: TheveninDrive { volts: 3.3, impedance: 25.0 },
    ///     lo: TheveninDrive { volts: 0.0, impedance: 25.0 },
    ///     segment: PeriodicSchedule { emitted: 0, freq_hz: 8_192, total: None, since_ns: 1_000_000 },
    /// };
    /// let Drive::Periodic { segment, .. } = drive else { unreachable!() };
    /// // One second later, exactly 8192 pulses have gone out.
    /// assert_eq!(segment.emitted_at_ns(1_001_000_000), 8_192);
    /// ```
    ///
    /// # How to fold a sequence of segments
    ///
    /// A segment is superseded, never continued: when the next one arrives,
    /// the outgoing segment is folded **up to its successor's
    /// [`PeriodicSchedule::since_ns`]**, and the successor's own baseline takes
    /// over from there. Within one segment, evaluate against the anchor the
    /// source published — do not re-base per read
    /// ([`PeriodicSchedule::rebased_at_ns`] explains why: it costs the source's
    /// pulse phase); `embsim_models::machine::stepper_motor` is the reference
    /// consumer. A segment whose rate is zero is a held clock: it still
    /// carries the final count, so a consumer that joins late folds exactly.
    ///
    /// # Fidelity limits
    ///
    /// - **There are no edges.** A receiver that counts level transitions
    ///   sees none (a periodic [`Sense`] names no single voltage, and
    ///   [`Sense::level`] reads none);
    ///   pulse width, duty cycle, rise time and jitter are not modelled, and
    ///   neither is DIR setup/hold against an individual step edge. A wire
    ///   carries no direction: a step/direction drive reads its DIR input
    ///   at the instant the DIR net changes.
    /// - **Counts are exact at the peripheral's own truncation.**
    ///   `emitted_at_ns` floors `elapsed_ns × freq / 1_000_000_000` exactly as
    ///   `embsim_peripherals::pulse_out::PulseOut::run` does, so consumer
    ///   and firmware agree bit for bit — both share that truncation, at the
    ///   engine's own nanosecond.
    /// - **Phase is not modelled.** Two periodic sources that contend on one
    ///   root are contention whatever their segments say — two sources
    ///   agreeing by construction is a wiring the reference machine does not
    ///   have (`sil-unified-drive.md`, "What resolution has to learn").
    /// - **A phase released at the slot** (a non-finite impedance: an
    ///   open-drain clock's high, a clipped-sine output no datasheet gives a
    ///   DC port for) sources nothing in that phase.
    Periodic {
        /// The port presented in the high phase.
        hi: TheveninDrive,
        /// The port presented in the low phase.
        lo: TheveninDrive,
        /// Rate, accumulated count, ceiling and start instant — the half a
        /// board-agnostic peripheral owns.
        segment: PeriodicSchedule,
    },
}

/// A receiver's input switching thresholds (`NODES.md` §10, "Per pin,
/// declared once": `V_IL`/`V_IH` plus hysteresis, declared relative to the
/// pin's supply so a brown-out scales them). One struct in two forms, the
/// form chosen by the declaration beside it:
///
/// - **relative** — the pin names a [`PinDecl::supply`]: each figure is a
///   fraction of the supply's span, `V(supply) − V(reference)`
///   ([`Thresholds::scaled`]) — the form a datasheet gives as `0.7 × VCC`,
///   and the form a brown-out scales;
/// - **absolute** — the pin names no supply: each figure is volts against
///   the pin's reference — the form a datasheet gives as `2.0 V` over a
///   supply range, or a standard gives for an interface
///   ([`jesd8c01_lvcmos_thresholds`]).
///
/// The build refuses a relative declaration whose figures are not fractions
/// of the supply ([`crate::BoardError::InvalidDeclaration`]), so an absolute
/// pair handed to a pin that names a supply is never read as a multiple of
/// it. Every figure is a datasheet's number, cited where it is declared:
/// there is no crate default, and no constructor supplies one (`DESIGN.md`
/// rule 6).
///
/// The receiver projects its own [`Sense`] through them
/// ([`Sense::level`], [`Thresholds::project`]) — `V_IL`/`V_IH`, the
/// hysteresis chosen by the last level it read, and its declared
/// [`DeadBand`] policy for a voltage the datasheet guarantees neither level
/// at; [`PinHandle::thresholds`] reports them in volts at the instant, and
/// [`PinHandle::level`] projects through them.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Thresholds {
    /// The highest input read as low: `V_IL` max, or a Schmitt input's
    /// negative-going threshold `V_T−` min.
    pub v_il: f64,
    /// The lowest input read as high: `V_IH` min, or a Schmitt input's
    /// positive-going threshold `V_T+` max.
    pub v_ih: f64,
    /// How far the switching point moves with the last level read — a
    /// Schmitt input's `ΔV_T` — and 0 for an input whose datasheet names
    /// none.
    pub hysteresis: f64,
    /// What the receiver reads strictly inside its dead band — a voltage
    /// its datasheet guarantees neither level at. Declared per receiver,
    /// with no crate default: [`Self::new`] requires it.
    pub dead_band: DeadBand,
}

/// A receiver's policy for a voltage strictly inside its dead band — above
/// `V_IL` and below `V_IH` once its hysteresis has moved them (`NODES.md`
/// §10, "Delivered to a sensing pin"). Declared on every [`Thresholds`];
/// there is no crate default (`DESIGN.md` rule 6).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum DeadBand {
    /// The input keeps the level it last read: a Schmitt trigger between
    /// its thresholds — the SN74LVC1G14's `V_T−`..`V_T+` window, a
    /// regulator enable between its falling and rising thresholds — holds
    /// its state, which is what its hysteresis is for.
    HoldLast,
    /// The input reads no level: a plain CMOS input between `V_IL` max and
    /// `V_IH` min, where its datasheet guarantees neither. The part's own
    /// answer for an input with no level applies — what it does with a
    /// floating or fought net.
    Unknown,
}

impl Thresholds {
    /// Thresholds from their three figures (see the type for the two forms)
    /// and the receiver's dead-band policy.
    pub const fn new(v_il: f64, v_ih: f64, hysteresis: f64, dead_band: DeadBand) -> Self {
        Self {
            v_il,
            v_ih,
            hysteresis,
            dead_band,
        }
    }

    /// A relative declaration at a supply span: every figure times `span`
    /// volts — `0.3/0.7 × VIO` at a 1.8 V bank is 0.54 V / 1.26 V. The
    /// policy is the receiver's, whatever its supply.
    pub fn scaled(self, span: Volts) -> Self {
        Self {
            v_il: self.v_il * span,
            v_ih: self.v_ih * span,
            hysteresis: self.hysteresis * span,
            dead_band: self.dead_band,
        }
    }

    /// The receiver's projection of `volts` (against its reference, in the
    /// same form as these thresholds) to a level, chosen by the level it
    /// `last` read (`NODES.md` §10: one compare per delivery).
    ///
    /// At or below `V_IL` the input reads low and at or above `V_IH` high,
    /// whatever it read last — the two figures the datasheet guarantees.
    /// Between them the hysteresis applies: an input that last read high is
    /// still high down to `V_IH − ΔV_T` (its falling threshold `V_T−` sits
    /// at least `ΔV_T` below its rising one, which is at most `V_IH`), and
    /// one that last read low is still low up to `V_IL + ΔV_T`. What is
    /// left is the dead band, and the receiver's [`DeadBand`] answers it:
    /// the last level, or none. With no last level the dead band is the
    /// whole `V_IL`..`V_IH` span. A non-finite voltage reads no level.
    pub fn project(&self, volts: Volts, last: Option<Level>) -> Option<Level> {
        if !volts.is_finite() {
            return None;
        }
        if volts <= self.v_il {
            return Some(Level::Low);
        }
        if volts >= self.v_ih {
            return Some(Level::High);
        }
        match last {
            Some(Level::High) if volts >= self.v_ih - self.hysteresis => {
                return Some(Level::High);
            }
            Some(Level::Low) if volts <= self.v_il + self.hysteresis => {
                return Some(Level::Low);
            }
            _ => {}
        }
        match self.dead_band {
            DeadBand::HoldLast => last,
            DeadBand::Unknown => None,
        }
    }

    /// Whether every figure is a fraction of a supply — the one form a pin
    /// that names a supply may declare — with `v_il` at or below `v_ih`.
    fn are_fractions(&self) -> bool {
        let unit = 0.0..=1.0;
        unit.contains(&self.v_il)
            && unit.contains(&self.v_ih)
            && unit.contains(&self.hysteresis)
            && self.v_il <= self.v_ih
    }

    /// Whether the figures are volts a receiver could hold: finite, `v_il`
    /// at or below `v_ih`, a hysteresis that is not negative.
    fn are_volts(&self) -> bool {
        self.v_il.is_finite()
            && self.v_ih.is_finite()
            && self.hysteresis.is_finite()
            && self.hysteresis >= 0.0
            && self.v_il <= self.v_ih
    }
}

/// The 3.3 V LVCMOS input pair, **absolute**: JEDEC JESD8C.01 (Interface
/// Standard for Nominal 3 V/3.3 V Supply Digital Integrated Circuits), DC
/// input specifications — `V_IL` max 0.8 V ([`crate::net::V_IL`]), `V_IH`
/// min 2.0 V ([`crate::net::V_IH`]); the standard names no hysteresis. The
/// pair the engine's own dead band is, for a caller whose part names no
/// threshold of its own — a bench fixture, a pin whose datasheet gives none
/// — to pass with the reason at the call site, and the receiver's own
/// [`DeadBand`] policy beside it: the standard names none, so the pair
/// carries none until its caller declares it. Never a constructor's
/// default.
pub const fn jesd8c01_lvcmos_thresholds(dead_band: DeadBand) -> Thresholds {
    Thresholds::new(crate::net::V_IL, crate::net::V_IH, 0.0, dead_band)
}

/// What a sensing pin is handed (`NODES.md` §10, "Delivered to a sensing
/// pin"; §11 `Sense`): the voltage of the node its net is on, **against the
/// pin's declared [`PinDecl::reference`]** — or in the engine's frame, the
/// one 0 V every published voltage is in, when the pin declares none — the
/// square wave, when the node carries one, and the instant the engine
/// delivered it. The digital level is the receiver's own projection
/// ([`Sense::level`]); the engine's projection, [`NetState`], is its report,
/// not what a node sees.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sense {
    /// The node's voltage against the pin's reference. `None` when no
    /// source reaches the node — a fact a receiver branches on (an open
    /// enable on the ISO67xx means *enabled*), not an ambiguous voltage —
    /// and likewise when no voltage can be named for it: a node only an
    /// unmodelled rail reaches (the rail names none), a node with two
    /// operating points (a clock fought for half of every cycle, two rates
    /// meeting), a running periodic node (see [`Self::periodic`] — a
    /// **held** one, its source stopped, names the voltage it rests at, its
    /// low phase's, beside the phases and the segment that carries its
    /// final count), and a pin whose
    /// reference names no voltage itself (it floats, or its pin is
    /// detached) — there is no ground to measure against, and none is
    /// implied (`DESIGN.md` rule 6). A fought node with one operating point
    /// is handed that voltage, the one the
    /// [`crate::Finding::AmbiguousLevel`] beside it names: contention and
    /// floating stay distinguishable as findings, and a reader asking for
    /// a voltage gets the one the fight settled at.
    pub volts: Option<Volts>,
    /// The square wave the node carries, when it carries one: each phase's
    /// voltage against the pin's reference and the integer segment that
    /// alternates them. A node that consumes a clock integrates the segment
    /// itself (`NODES.md` §11, contract line 6).
    pub periodic: Option<PeriodicSense>,
    /// The virtual instant, nanoseconds, the engine delivered this at — 0 on
    /// a build before the virtual clock exists.
    pub at_ns: u64,
}

/// A periodic node as a sensing pin is handed it ([`Sense::periodic`]).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PeriodicSense {
    /// The node's voltage in the drive's high phase, against the pin's
    /// reference (`None` where that phase names none: it floats).
    pub hi: Option<Volts>,
    /// The node's voltage in the drive's low phase.
    pub lo: Option<Volts>,
    /// Rate, count, ceiling and anchor — compared by identity, so a
    /// segment is delivered once and never again as time passes.
    pub segment: PeriodicSchedule,
}

impl PeriodicSense {
    /// Each phase through the receiver's thresholds, `(high phase, low
    /// phase)`. A phase is steady for half a cycle and has no last level
    /// of its own the receiver could hold, so each projects with none: the
    /// guaranteed thresholds, and the dead band per the receiver's policy
    /// with nothing to hold.
    pub fn levels(&self, thresholds: &Thresholds) -> (Option<Level>, Option<Level>) {
        let project = |volts: Option<Volts>| volts.and_then(|v| thresholds.project(v, None));
        (project(self.hi), project(self.lo))
    }
}

impl Sense {
    /// The receiver's own projection (`NODES.md` §11): its thresholds, its
    /// hysteresis, its last level, its dead-band policy
    /// ([`Thresholds::project`]). `None` where the node names no voltage —
    /// a floating node reads no level, and nothing invents one. A running
    /// periodic node is projected phase by phase, each from the same last
    /// level: where both phases project to one level the receiver sees no
    /// edge in the swing and reads that level (a 0 V / 1.2 V clock at a
    /// receiver whose `V_IL` is above 1.2 V is a steady low to it); where
    /// they project to two, or a phase to none, the node has no single
    /// level and reads `None` — the receiver consumes the segment, or
    /// reads nothing (`NODES.md` §10, "the level is the receiver's").
    pub fn level(&self, thresholds: &Thresholds, last: Option<Level>) -> Option<Level> {
        match (self.volts, &self.periodic) {
            (Some(volts), _) => thresholds.project(volts, last),
            (None, Some(periodic)) => {
                let project =
                    |volts: Option<Volts>| volts.and_then(|v| thresholds.project(v, last));
                let (hi, lo) = (project(periodic.hi), project(periodic.lo));
                if hi == lo {
                    hi
                } else {
                    None
                }
            }
            (None, None) => None,
        }
    }

    /// What a pin measured in `frame` is handed for a net resolved to
    /// `state` at `node`, when the pin's reference is at `reference` —
    /// the one conversion the live engine and the build path share
    /// (`on_sense`'s two-code-paths rule).
    pub(crate) fn measured(
        state: NetState,
        node: NetVolts,
        frame: SenseFrame,
        reference: Option<NetVolts>,
        at_ns: u64,
    ) -> Self {
        let offset = match frame {
            SenseFrame::Absolute => 0.0,
            SenseFrame::Against(_) => match reference {
                Some(NetVolts {
                    dc: Some(volts), ..
                }) => volts,
                _ => {
                    return Self {
                        volts: None,
                        periodic: None,
                        at_ns,
                    }
                }
            },
            SenseFrame::Detached => {
                return Self {
                    volts: None,
                    periodic: None,
                    at_ns,
                }
            }
        };
        let volts = node.dc.map(|v| v - offset);
        let periodic = match state {
            NetState::Periodic { segment, .. } => {
                let (hi, lo) = node.phases.unwrap_or((None, None));
                Some(PeriodicSense {
                    hi: hi.map(|v| v - offset),
                    lo: lo.map(|v| v - offset),
                    segment,
                })
            }
            _ => None,
        };
        Self {
            volts,
            periodic,
            at_ns,
        }
    }
}

/// What a pin's [`Sense`] is measured against: the engine's frame, when the
/// pin declares no [`PinDecl::reference`]; the net its reference pin is on;
/// or nothing, when the reference pin is on no net (detached by a
/// scenario).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub(crate) enum SenseFrame {
    /// The pin declares no reference.
    #[default]
    Absolute,
    /// The pin's reference pin is on this net.
    Against(NetId),
    /// The pin's reference pin is on no net.
    Detached,
}

/// A sense pin's own DC port (`NODES.md` §10, IBIS `Input`): `r_in` to
/// `v_bias`, measured against the pin's reference — the mechanism of an
/// open-input failsafe (the AM26LV32's inputs), a regulator's quiescent
/// load. Without one a node read only by high-impedance inputs has no load
/// at all.
///
/// Stamped at build as a permanent Thevenin source at the pin — `v_bias`
/// behind `r_in` — that no drive ever changes (`NODES.md` §10: declared
/// once, stamped by the solver, never republished). Rule 2 ranks it like
/// any source, and the build refuses one below
/// [`crate::net::WEAK_DRIVE_OHMS`]: a pull that never contends, so the node
/// sits at its bias when nothing else reaches it and a driver on the node
/// wins outright. `v_bias` is stamped
/// in the engine's frame — exact while the part's reference sits at 0 V,
/// as every board's ground does (a part that drives at a voltage it senses
/// makes the same assumption). Only a [`PinRole::Signal`] or
/// [`PinRole::PowerIn`] pin declares one; the build refuses it elsewhere,
/// and refuses a bias that is not finite or a resistance that is not
/// finite and positive. A port's current is the pin's own load: it is not
/// summed into the pin's reported current.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InputPort {
    /// The voltage the input rests at, open, against the pin's reference.
    pub v_bias: Volts,
    /// The resistance from the pin to that bias.
    pub r_in: Ohms,
}

/// The declared pin a [`Clamp`] shunts to.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ClampRail {
    /// The pin's [`PinDecl::supply`] (IBIS `[POWER Clamp]`): conducts from
    /// the pin into the supply once the pin rises `vf` above it.
    Supply,
    /// The pin's [`PinDecl::reference`] (IBIS `[GND Clamp]`): conducts from
    /// the reference into the pin once the pin falls `vf` below it.
    Reference,
}

/// An always-on clamp diode between a pin and its supply or reference pin
/// (`NODES.md` §10; IBIS `[POWER Clamp]`/`[GND Clamp]`): on, `vf` in series
/// with `r_d`. A clamp is the pin's protection, never its drive — a pad's
/// strength is its published Thevenin, and a pad that needs a nonlinear
/// driver curve is a piecewise-linear element part, not a pin table — so
/// the two never disagree. A clamp names its rail by role; the build
/// refuses one whose pin declares no such rail
/// ([`crate::BoardError::InvalidDeclaration`]).
///
/// Stamped at build as a [`PwlCurve::Diode`] branch — anode the pin and
/// cathode its supply pin for [`ClampRail::Supply`], anode the reference
/// pin and cathode the pin for [`ClampRail::Reference`] — registered as the
/// part's own branches are, so the region loop decides it and its current
/// is summed into the pin's. A clamp to a rail on a declared terminal is an
/// element ending on a terminal: a boundary, no union. A clamp to a pin on
/// a net that is not a terminal joins that net's cluster to the pin's, as
/// any element does. A cluster a clamp sits in is an element cluster and is
/// solved whenever it resolves — the cost is the declaring part's alone. No
/// part on the three boards declares one: none of their datasheets gives a
/// clamp curve.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Clamp {
    /// The rail the clamp shunts to.
    pub to: ClampRail,
    /// Forward voltage at the knee.
    pub vf: Volts,
    /// Dynamic resistance of the on segment.
    pub r_d: Ohms,
}

/// The idle a push-pull output presents until its component drives it:
/// high, at the crate's logic rail behind the push-pull impedance —
/// [`crate::net::digital_drive`]`(High)`, as a constant a constructor can
/// hold.
const PUSH_PULL_HIGH: TheveninDrive = TheveninDrive {
    volts: crate::net::LOGIC_HIGH_VOLTS,
    impedance: crate::net::DEFAULT_PUSH_PULL_IMPEDANCE,
};

/// The idle of a `PowerOut` pin whose declaration names none: a rail at a
/// voltage no model declares (`f64::NAN` behind 0 Ω — the source table's
/// "unmodelled"), which presents as up through the path to it and ranks
/// nowhere. Every regulator model declares its output released instead;
/// this is what a bench part that stands in for a rail it does not model
/// holds (`BOARD_ENGINE.md`, the resolution rules).
const UNMODELLED_RAIL: TheveninDrive = TheveninDrive {
    volts: f64::NAN,
    impedance: 0.0,
};

/// One declared pin of a [`Component`] (`NODES.md` §11): its identity, its
/// [`PinRole`], and the static electrical facts the solver stamps and the
/// engine never asks for again — IBIS's decomposition. The set returned by
/// [`Component::pins`] must cover the component's netlist pins exactly —
/// build validates both directions — and every pin a declaration names
/// (a reference, a supply) must be one of them.
///
/// Build a declaration from one of the eight constructors —
/// [`digital_in`](Self::digital_in), [`digital_out`](Self::digital_out),
/// [`digital_io`](Self::digital_io), [`analog`](Self::analog),
/// [`analog_source`](Self::analog_source), [`power_in`](Self::power_in),
/// [`power_out`](Self::power_out), [`passive`](Self::passive) — and the
/// `with_*` builders; a constructor that needs thresholds takes them.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PinDecl {
    /// Netlist pin number (`"3"`).
    pub number: &'static str,
    /// Alias (`"RX"`) — matches KiCad `pinfunction` when present.
    pub name: Option<&'static str>,
    /// What the pin is to the solve.
    pub role: PinRole,
    /// The drive the pin presents from attach until its component drives
    /// it; `None` is released. Honoured on every pin with a slot — a
    /// [`PinRole::Signal`] pin and a [`PinRole::PowerOut`] pin, whose slot
    /// is its terminal — and refused at build on a power-in or passive pin,
    /// which has none ([`crate::BoardError::IdleOnSlotlessPin`]).
    pub idle: Option<TheveninDrive>,
    /// The pin's own DC input port (see [`InputPort`]; stamped as a
    /// permanent weak source at the pin).
    pub input: Option<InputPort>,
    /// Always-on shunts to the pin's supply or reference (see [`Clamp`];
    /// stamped as diode branches).
    pub clamps: &'static [Clamp],
    /// The pin's capacitance to its reference, picofarads — the RC pole the
    /// engine will arm. Declared now, **armed in phase 6** (`NODES.md` §12,
    /// the capacitors phase): the engine makes no use of it yet.
    pub capacitance_pf: Option<f64>,
    /// The receiver's switching thresholds (see [`Thresholds`] for the
    /// relative and absolute forms). A [`PinRole::Signal`] sense that
    /// declares none is an analog reader.
    pub thresholds: Option<Thresholds>,
    /// The pin its voltages are measured against — a regulator's output
    /// against its own ground pin, a supply pin against the part's ground.
    /// The build lints read it ([`crate::Finding::UnreferencedDomain`],
    /// [`crate::Finding::UndecoupledPowerPin`], the reason in
    /// [`crate::Finding::RailDown`]); [`PinHandle::thresholds`] measures a
    /// supply span against it.
    pub reference: Option<&'static str>,
    /// The pin whose voltage the pin's relative thresholds (and its power
    /// clamp) are declared against — a pad's bank supply.
    pub supply: Option<&'static str>,
    /// Whether the pin can source current onto its net — drive it high. A
    /// pin that can sink and not source is open-drain.
    pub can_source: bool,
    /// Whether the pin can sink current from its net — drive it low. A pin
    /// that can do neither is an input: a sense.
    pub can_sink: bool,
}

impl PinDecl {
    /// A pin of `role` with nothing else declared.
    const fn bare(number: &'static str, role: PinRole) -> Self {
        Self {
            number,
            name: None,
            role,
            idle: None,
            input: None,
            clamps: &[],
            capacitance_pf: None,
            thresholds: None,
            reference: None,
            supply: None,
            can_source: false,
            can_sink: false,
        }
    }

    /// A digital input: a [`PinRole::Signal`] pin that neither sources nor
    /// sinks and reads its net through `thresholds` — the datasheet's, or
    /// [`jesd8c01_lvcmos_thresholds`] with the reason where it names none.
    pub const fn digital_in(number: &'static str, thresholds: Thresholds) -> Self {
        let mut pin = Self::bare(number, PinRole::Signal);
        pin.thresholds = Some(thresholds);
        pin
    }

    /// A push-pull output: sources and sinks, and idles driven high at the
    /// crate's logic rail behind the push-pull impedance until the component
    /// drives it — unless [`with_idle`](Self::with_idle) says otherwise. An
    /// output that also declares thresholds
    /// ([`with_thresholds`](Self::with_thresholds)) is bidirectional.
    pub const fn digital_out(number: &'static str) -> Self {
        let mut pin = Self::bare(number, PinRole::Signal);
        pin.idle = Some(PUSH_PULL_HIGH);
        pin.can_source = true;
        pin.can_sink = true;
        pin
    }

    /// An analog reader: a [`PinRole::Signal`] sense with no thresholds,
    /// which asks for the solved voltage — a differential receiver input,
    /// an ADC input. Its net's cluster is solved whenever it resolves. A
    /// reader only: it neither sources nor sinks, so a drive published
    /// through it contradicts its declaration (traced) — a pin that
    /// publishes a linear source is [`analog_source`](Self::analog_source).
    pub const fn analog(number: &'static str) -> Self {
        Self::bare(number, PinRole::Signal)
    }

    /// A linear source: a [`PinRole::Signal`] pin that publishes a Thevenin
    /// port at any voltage — sources and sinks — and idles released until
    /// its component drives it, with no thresholds: a bench supply's or a
    /// bridge excitation's output, a resistor pull a bench part stands in
    /// for. It is no sense (nothing reads its net because of it), so it
    /// never asks for a solve; [`with_idle`](Self::with_idle) declares a
    /// static pull in place of an attach-time publish.
    pub const fn analog_source(number: &'static str) -> Self {
        let mut pin = Self::bare(number, PinRole::Signal);
        pin.can_source = true;
        pin.can_sink = true;
        pin
    }

    /// A bidirectional digital pad: sources and sinks, reads its net
    /// through `thresholds`, and idles released — an input until its
    /// component drives it ([`reads_when_subscribed`](Self::reads_when_subscribed)).
    pub const fn digital_io(number: &'static str, thresholds: Thresholds) -> Self {
        let mut pin = Self::analog_source(number);
        pin.thresholds = Some(thresholds);
        pin
    }

    /// A rail the part consumes.
    pub const fn power_in(number: &'static str) -> Self {
        Self::bare(number, PinRole::PowerIn)
    }

    /// A rail the part generates: a declared terminal. It idles at a
    /// voltage no model declares (see [`PinRole::PowerOut`]) unless
    /// [`with_idle`](Self::with_idle) names what it holds — released for a
    /// rail that is down until its part publishes, which is what every
    /// regulator model declares.
    pub const fn power_out(number: &'static str) -> Self {
        let mut pin = Self::bare(number, PinRole::PowerOut);
        pin.idle = Some(UNMODELLED_RAIL);
        pin.can_source = true;
        pin
    }

    /// A terminal that contributes nothing electrical of its own.
    pub const fn passive(number: &'static str) -> Self {
        Self::bare(number, PinRole::Passive)
    }

    /// The same pin with an alias (`"RX"`).
    pub const fn with_name(mut self, name: &'static str) -> Self {
        self.name = Some(name);
        self
    }

    /// The same pin idling at `idle` (`None`: released).
    pub const fn with_idle(mut self, idle: Option<TheveninDrive>) -> Self {
        self.idle = idle;
        self
    }

    /// The same pin idling behind `ohms`: the impedance of a declared idle
    /// drive. A pin that idles released has no impedance to set.
    pub const fn with_impedance(mut self, ohms: Ohms) -> Self {
        if let Some(idle) = &mut self.idle {
            idle.impedance = ohms;
        }
        self
    }

    /// The same pin reading its net through `thresholds`.
    pub const fn with_thresholds(mut self, thresholds: Thresholds) -> Self {
        self.thresholds = Some(thresholds);
        self
    }

    /// The same pin measured against the pin `reference` (number or alias).
    pub const fn with_reference(mut self, reference: &'static str) -> Self {
        self.reference = Some(reference);
        self
    }

    /// The same pin with its relative thresholds declared against the pin
    /// `supply` (number or alias).
    pub const fn with_supply(mut self, supply: &'static str) -> Self {
        self.supply = Some(supply);
        self
    }

    /// The same pin with its own DC input port.
    pub const fn with_input(mut self, input: InputPort) -> Self {
        self.input = Some(input);
        self
    }

    /// The same pin with always-on clamps.
    pub const fn with_clamps(mut self, clamps: &'static [Clamp]) -> Self {
        self.clamps = clamps;
        self
    }

    /// The same pin with a capacitance to its reference, picofarads
    /// (declared; armed in phase 6).
    pub const fn with_capacitance_pf(mut self, picofarads: f64) -> Self {
        self.capacitance_pf = Some(picofarads);
        self
    }

    /// The same pin as an open-drain (open-collector) output: it sinks and
    /// cannot source, and rests released.
    pub const fn sink_only(mut self) -> Self {
        self.can_source = false;
        self.can_sink = true;
        self.idle = None;
        self
    }

    /// Whether the pin can publish a drive onto its net at all — it sources
    /// or sinks. A signal pin that can do neither is a sense.
    pub const fn drives(&self) -> bool {
        self.can_source || self.can_sink
    }

    /// The sense the pin declares **at build**, which makes its net read
    /// from the first pass: a [`PinRole::Signal`] pin that neither sources
    /// nor sinks is a digital sense when it declares thresholds and an
    /// analog reader — its cluster solved — when it declares none. `None`
    /// for every other pin (a power-in pin is sensed as a supply, by its
    /// role).
    pub const fn senses_at_build(&self) -> Option<SenseKind> {
        match self.role {
            PinRole::Signal if !self.drives() => Some(if self.thresholds.is_some() {
                SenseKind::Digital
            } else {
                SenseKind::Analog
            }),
            PinRole::Signal | PinRole::PowerIn | PinRole::PowerOut | PinRole::Passive => None,
        }
    }

    /// Whether a sense subscription through the pin is the declaration that
    /// it **reads** its net (a digital read, reported floating like a
    /// sense's): a bidirectional pin — one that drives and declares
    /// thresholds — idling released, an input until its owner drives it. A
    /// pad nothing subscribes to reads nothing and floats without a finding.
    pub const fn reads_when_subscribed(&self) -> bool {
        matches!(self.role, PinRole::Signal)
            && self.drives()
            && self.thresholds.is_some()
            && self.idle.is_none()
    }

    /// Whether the pin answers to `id` — its number or its alias.
    pub fn answers_to(&self, id: &str) -> bool {
        self.number == id || self.name == Some(id)
    }
}

/// Validate one component's declarations against themselves: every pin a
/// declaration names (a reference, a supply) is a declared pin and not the
/// pin itself; thresholds are fractions on a pin that names a supply and
/// volts on one that names none; a clamp's rail is declared; an idle drive
/// sits on a pin with a slot. The first violation, in declaration order.
pub(crate) fn validate_declarations(pins: &[PinDecl]) -> Result<(), DeclarationError> {
    for pin in pins {
        if matches!(pin.role, PinRole::PowerIn | PinRole::Passive) && pin.idle.is_some() {
            return Err(DeclarationError::IdleOnSlotless {
                pin: pin.number.to_string(),
            });
        }
        for named in [pin.reference, pin.supply].into_iter().flatten() {
            if !pins.iter().any(|p| p.answers_to(named)) {
                return Err(DeclarationError::UndeclaredPin {
                    pin: named.to_string(),
                });
            }
            if pin.answers_to(named) {
                return Err(DeclarationError::UndeclaredPin {
                    pin: format!("{named} (its own reference)"),
                });
            }
        }
        if let Some(thresholds) = pin.thresholds {
            let valid = if pin.supply.is_some() {
                thresholds.are_fractions()
            } else {
                thresholds.are_volts()
            };
            if !valid {
                return Err(DeclarationError::Invalid {
                    pin: pin.number.to_string(),
                    reason: if pin.supply.is_some() {
                        "thresholds relative to a supply must be fractions of it (0..=1, v_il <= v_ih)"
                    } else {
                        "absolute thresholds must be finite volts with v_il <= v_ih and a \
                         hysteresis that is not negative"
                    },
                });
            }
        }
        if let Some(port) = pin.input {
            if !matches!(pin.role, PinRole::Signal | PinRole::PowerIn) {
                return Err(DeclarationError::Invalid {
                    pin: pin.number.to_string(),
                    reason: "an input port on a pin that is neither a signal nor a power input",
                });
            }
            if !port.v_bias.is_finite() || !port.r_in.is_finite() || port.r_in <= 0.0 {
                return Err(DeclarationError::Invalid {
                    pin: pin.number.to_string(),
                    reason: "an input port needs a finite bias and a finite, positive resistance",
                });
            }
            // A port is the pin's own load: rule 2 must rank it as a pull,
            // which never contends. One stronger than the weak-drive
            // boundary would rank as a driver, fight the net's real drivers
            // and be named in their `Contention` — a load is not a source.
            if port.r_in < crate::net::WEAK_DRIVE_OHMS {
                return Err(DeclarationError::Invalid {
                    pin: pin.number.to_string(),
                    reason: "an input port's resistance must be at least the weak-drive boundary \
                             (1 kOhm): a load that ranks as a driver would contend",
                });
            }
        }
        for clamp in pin.clamps {
            let declared = match clamp.to {
                ClampRail::Supply => pin.supply.is_some(),
                ClampRail::Reference => pin.reference.is_some(),
            };
            if !declared {
                return Err(DeclarationError::Invalid {
                    pin: pin.number.to_string(),
                    reason: "a clamp to a rail the pin does not declare",
                });
            }
            let finite = |x: f64| x.is_finite() && x >= 0.0;
            if !finite(clamp.vf) || !finite(clamp.r_d) {
                return Err(DeclarationError::Invalid {
                    pin: pin.number.to_string(),
                    reason: "a clamp needs a finite knee and a finite dynamic resistance, \
                             neither negative",
                });
            }
        }
    }
    Ok(())
}

/// The branches a pin's declared [`Clamp`]s stamp: a
/// [`PwlCurve::Diode`] from the pin to its supply pin, or from its
/// reference pin to the pin, named by the identities the declaration
/// names them with ([`validate_declarations`] has refused a clamp to a
/// rail the pin does not declare).
pub(crate) fn clamp_branches(pin: &PinDecl) -> impl Iterator<Item = Branch> + '_ {
    pin.clamps.iter().filter_map(move |clamp| {
        let curve = PwlCurve::Diode {
            vf: clamp.vf,
            r_d: clamp.r_d,
        };
        let (a, b) = match clamp.to {
            ClampRail::Supply => (pin.number, pin.supply?),
            ClampRail::Reference => (pin.reference?, pin.number),
        };
        Some(Branch {
            a,
            b,
            curve,
            control: None,
        })
    })
}

/// Why [`validate_declarations`] refused a pin table; the board build turns
/// it into a [`crate::BoardError`] naming the part.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum DeclarationError {
    /// An idle drive on a power-in or passive pin.
    IdleOnSlotless {
        /// The pin carrying it.
        pin: String,
    },
    /// A reference or supply naming a pin the facade does not declare (or
    /// the pin itself).
    UndeclaredPin {
        /// The identity that names nothing.
        pin: String,
    },
    /// A declaration the engine could not honour as written.
    Invalid {
        /// The pin carrying it.
        pin: String,
        /// What is wrong with it.
        reason: &'static str,
    },
}

// ============================================================
// Nonlinear branches (piecewise-linear elements)
// ============================================================

/// The piecewise-linear curve of a [`Branch`]: a few regions, each one
/// linear stamp in the cluster solve (`NODES.md` §2, the Diode / LED, FET /
/// BJT and CCR rows) — **off** and **on** for a diode, a channel and a
/// regulator, plus **active** for a transistor's collector
/// ([`crate::cluster::Region`]). Off is the leakage conductance
/// [`crate::cluster::GMIN_OHMS`] (the regulator's ohmic segment excepted),
/// never an open — so the far side of an off element stays reachable and
/// its region test has an operand. The region is chosen by the engine's
/// bounded, ordered flip loop, cold-started on every solve
/// ([`crate::cluster::QuasiStaticMna`]); a node never chooses its own
/// region.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PwlCurve {
    /// A diode, an LED, a body diode: on when the forward drop from
    /// [`Branch::a`] (anode) to [`Branch::b`] (cathode) reaches `vf`, and
    /// then a `vf` source in series with `r_d` — the branch carries
    /// `(V(a) − V(b) − vf) / r_d`. Off below the knee.
    Diode {
        /// Forward voltage at the knee, volts — a datasheet number.
        vf: Volts,
        /// Dynamic resistance of the on segment, ohms. Zero is a vertical
        /// segment (the drop is `vf` at any current), stamped at the ideal
        /// floor ([`crate::cluster::IDEAL_SOURCE_FLOOR_OHMS`]).
        r_d: Ohms,
    },
    /// A switched channel — a FET's drain–source: `r_on` between
    /// [`Branch::a`] and [`Branch::b`] while the branch's [`Branch::control`]
    /// test passes, the leakage conductance otherwise. A channel with no
    /// control is a declared-open switch: always off.
    Channel {
        /// On-state resistance, ohms (`R_DS(on)`).
        r_on: Ohms,
    },
    /// A two-terminal constant-current regulator (the NSI50010 family): the
    /// datasheet's I–V curve as two regions. Below the knee — `V(a) − V(b)`
    /// under `v_reg` — the branch is the ohmic segment from the origin to
    /// the knee, a resistor of `v_reg / i_reg`; at or above it the branch
    /// carries `i_reg` from `a` to `b` whatever the voltage across it (a
    /// current source with the leakage conductance in parallel). Reverse
    /// bias conducts through the ohmic segment, a stated simplification of
    /// a part whose reverse rating is a few hundred millivolts. Its cold
    /// start is the ohmic segment, not a leakage.
    Regulator {
        /// Regulation current, amperes, from `a` to `b`.
        i_reg: Amps,
        /// The knee: the anode–cathode voltage at which regulation begins
        /// (the datasheet's overhead voltage).
        v_reg: Volts,
    },
    /// A bipolar transistor's collector–emitter branch, `a` the collector
    /// and `b` the emitter, whose [`Branch::control`] is the base with the
    /// base–emitter knee as its test ([`RegionTest::AtLeast`]`(v_be)`); the
    /// base–emitter junction itself is a [`PwlCurve::Diode`] branch from
    /// the base to the emitter, declared **before** this one. Three
    /// regions ([`crate::cluster::Region`]): off (leakage) while the base
    /// test fails; **on** — saturated, `r_sat` between collector and
    /// emitter — while the base current supports the collector current the
    /// load draws, `I_C ≤ hfe · I_B`; **active** — a current source of
    /// `hfe · I_B` from collector to emitter — when it does not, so an
    /// under-driven base reads as a sagging collector rather than a closed
    /// switch. A collector branch whose base diode is not declared carries
    /// nothing: its gain has no base current to multiply.
    Bjt {
        /// The minimum DC current gain the load current is judged against
        /// (`h_FE` min at the load's collector current).
        hfe: f64,
        /// Saturated collector–emitter resistance, ohms (`V_CE(sat) / I_C`).
        r_sat: Ohms,
    },
}

/// The test a [`Branch`]'s control pin applies to decide the on region: it
/// compares `V(control) − V(b)` — the control terminal against the
/// branch's own `b` terminal (a FET's gate against its source, a
/// transistor's base against its emitter) — with a threshold. A control
/// terminal no source reaches has no voltage, and the test evaluates as
/// **off**; a control terminal on a declared terminal (a rail, a harness
/// supply, a stuck net) reads that terminal's constant and joins no cluster
/// (`NODES.md` §8 phase 3, the parts record).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RegionTest {
    /// On while `V(control) − V(b)` is at or above the threshold — an
    /// N-channel's positive `V_GS(th)`.
    AtLeast(Volts),
    /// On while `V(control) − V(b)` is at or below the threshold — a
    /// P-channel's negative `V_GS(th)`.
    AtMost(Volts),
}

impl RegionTest {
    /// Whether a control-to-`b` voltage puts the branch in its on region.
    pub fn passes(self, control_minus_b: Volts) -> bool {
        match self {
            RegionTest::AtLeast(threshold) => control_minus_b >= threshold,
            RegionTest::AtMost(threshold) => control_minus_b <= threshold,
        }
    }
}

/// A nonlinear element declared by a [`Component`]: a branch between two of
/// its pins with a piecewise-linear curve, and an optional control pin
/// (`NODES.md` §11). Never a per-pin curve to a reference — a diode is a
/// branch between two pins, a channel a branch decided by a third. Pins are
/// named as the component's [`PinDecl`]s name them (number or alias); a
/// branch naming a pin the facade does not declare fails the board build.
///
/// The current through a branch is reported positive from `a` to `b`
/// ([`PinHandle::sense_current`], [`ComponentNetIo::on_branch`],
/// `BuiltSystem::branch_current`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Branch {
    /// The anode / drain side.
    pub a: &'static str,
    /// The cathode / source side — the reference of the control test.
    pub b: &'static str,
    /// The two-region curve.
    pub curve: PwlCurve,
    /// The control pin and the test on `V(control) − V(b)`, for a
    /// [`PwlCurve::Channel`] or the base of a [`PwlCurve::Bjt`]; a diode's
    /// and a regulator's test is its own forward drop.
    pub control: Option<(&'static str, RegionTest)>,
}

// ============================================================
// Component trait
// ============================================================

/// A part on a board: declares its pin facade and receives its net I/O
/// handle at build time.
///
/// Concurrency contract: sense callbacks and scheduled wakeups are all
/// delivered from the engine thread, so they never race each other; they MAY
/// race the component's own protocol threads, which remains the component's
/// responsibility.
pub trait Component: Send + Sync {
    /// Declared pins. Must cover the component's netlist pins exactly —
    /// build validates BOTH directions (declared-but-absent and
    /// present-but-undeclared netlist pins are hard errors).
    fn pins(&self) -> &[PinDecl];

    /// The component's nonlinear elements: piecewise-linear branches
    /// between two of its declared pins ([`Branch`]). The engine stamps
    /// them into the cluster solve and chooses their regions; the
    /// component never publishes a drive for them. Default: none.
    fn branches(&self) -> &[Branch] {
        &[]
    }

    /// Runs once at build, BEFORE the component is shared (pre-`Arc`), so
    /// components store typed pin handles without interior mutability and
    /// fail loudly on facade mismatch.
    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError>;

    /// Runs once on the live path ([`crate::System::start`]) after EVERY
    /// component in the system has attached — the init-ordering point where
    /// a component may begin execution it owns (an MCU spawning its firmware
    /// entry thread). By this point all bridged I/O is wired and the engine
    /// is delivering, so code started here observes a fully-connected
    /// system from its first instruction. Build-time analysis
    /// ([`crate::System::build`]) never calls it. Default: nothing.
    fn start(&mut self) {}
}

/// One resistor touching a pin's node, as [`ComponentNetIo::resistors_at`]
/// reports it: the build-time topology a part reads **once at attach** —
/// the feedback divider a buck's output voltage is set by, the strap a
/// select pin is tied through — and never again (`NODES.md` §2, the
/// Regulator row; §5, `ComponentNetIo::resistors_at`).
#[derive(Debug, Clone, PartialEq)]
pub struct ResistorAt {
    /// The resistor, as `Board.Reference`.
    pub reference: String,
    /// Its value, ohms.
    pub ohms: Ohms,
    /// The node at its far end: the identity root of the net its other pin
    /// is on, comparable with [`ComponentNetIo::node`].
    pub far: NetId,
}

/// The build-time topology a component may read at attach: the identity
/// root of every net (harness merges and closed poles applied) and the
/// resistors touching each root. Fixed at build (`NODES.md` "Three rules
/// the taxonomy rests on", 3), so a value read at attach holds for the
/// system's life.
#[derive(Debug, Default)]
pub(crate) struct BuildTopology {
    /// Identity root of every global net.
    pub(crate) root_of: Vec<usize>,
    /// The resistors touching each identity root, in declaration order.
    pub(crate) resistors: HashMap<usize, Vec<ResistorAt>>,
}

// ============================================================
// Net I/O handle
// ============================================================

/// Handle to one attached pin's net.
///
/// Cloneable and thread-safe: components hand clones to their protocol
/// threads and callbacks. Equality compares pin identity (net + endpoint),
/// not engine wiring.
#[derive(Debug, Clone)]
pub struct PinHandle {
    net: NetId,
    endpoint: Option<EndpointId>,
    /// The pin is bidirectional and idles released
    /// ([`PinDecl::reads_when_subscribed`]): a pad that is an input until
    /// its owner drives it, so a sense subscription through it declares the
    /// net **read** (`Command::DeclareRead`) and a floating one becomes a
    /// reported finding. A sense pin is a sense from its declaration; this
    /// is the bidirectional pad's equivalent, made at the subscription that
    /// reads it.
    reads_when_released: bool,
    /// The pin's declared thresholds, with the net of the supply they are
    /// relative to ([`PinHandle::thresholds`]); `None` for a pin that
    /// declares none. Shared, so a handle stays a few words wide.
    declared: Option<Arc<DeclaredThresholds>>,
    /// What the pin's [`Sense`] is measured against: its declared
    /// reference's net ([`PinDecl::reference`]).
    frame: SenseFrame,
    /// The elements this pin terminates, as `(element index, sign)`: `+1`
    /// where the pin is the branch's `a` (the branch current flows from the
    /// net into the pin), `−1` where it is `b`. Summed with the pin's own
    /// drive current by [`PinHandle::sense_current`].
    branch_terms: Vec<(usize, f64)>,
    /// A `PowerOut` pin: its slot is its terminal's — what it publishes is
    /// what the rail holds — and its current spans clusters, so it is no
    /// instrument.
    terminal: bool,
    /// What the pin declared it can do to its net: a publish that exceeds
    /// it is traced ([`Self::drive`]).
    capability: DriveCapability,
    link: EngineLink,
}

/// A pin's declared drive capability ([`PinDecl::can_source`],
/// [`PinDecl::can_sink`]) as its handle carries it, so a publish can be
/// checked against the declaration (`NODES.md` §11, contract line 3:
/// the sourcing capability is a declaration, and a drive that contradicts
/// it is the model's bug to hear about).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub(crate) struct DriveCapability {
    pub(crate) can_source: bool,
    pub(crate) can_sink: bool,
    /// The capability was declared — a handle built from a [`PinDecl`].
    /// An identity-only or test handle declares nothing and is not
    /// checked.
    pub(crate) declared: bool,
}

impl DriveCapability {
    /// The capability `pin` declares.
    pub(crate) const fn of(pin: &PinDecl) -> Self {
        Self {
            can_source: pin.can_source,
            can_sink: pin.can_sink,
            declared: true,
        }
    }

    /// Why `drive` contradicts the declaration, when it does beyond doubt:
    /// any drive from a pin declared to neither source nor sink (an input),
    /// and a current injection in a direction the pin cannot push —
    /// sourcing (positive amps) from a pin that cannot source, sinking from
    /// one that cannot sink. Whether a Thevenin or a periodic port sources
    /// or sinks depends on the voltage its node settles at, which only the
    /// solve knows; the build's `OpenDrainWithoutPullUp` lint is where a
    /// sink-only declaration is checked against its net.
    pub(crate) fn contradicted_by(&self, drive: &Drive) -> Option<&'static str> {
        if !self.declared {
            return None;
        }
        if !self.can_source && !self.can_sink {
            return Some("a pin declared to neither source nor sink published a drive");
        }
        match *drive {
            Drive::Current { amps } if amps > 0.0 && !self.can_source => {
                Some("a pin declared unable to source injected current into its net")
            }
            Drive::Current { amps } if amps < 0.0 && !self.can_sink => {
                Some("a pin declared unable to sink drew current from its net")
            }
            _ => None,
        }
    }
}

/// A pin's [`PinDecl::thresholds`] as its handle carries them: the
/// declaration, and the net of the [`PinDecl::supply`] pin it names (a
/// detached one is on no net). The reference is the handle's
/// [`SenseFrame`], shared with its sense.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct DeclaredThresholds {
    pub(crate) thresholds: Thresholds,
    pub(crate) supply: Option<NetId>,
    /// The pin names a supply: its thresholds are relative (see
    /// [`Thresholds`]), whether or not the supply pin is on a net.
    pub(crate) relative: bool,
}

impl PartialEq for PinHandle {
    fn eq(&self, other: &Self) -> bool {
        self.net == other.net && self.endpoint == other.endpoint
    }
}

impl Eq for PinHandle {}

impl PinHandle {
    /// Create an identity-only handle bound to a resolved net (board-build
    /// internal use; carries no engine wiring).
    pub fn new(net: NetId) -> Self {
        Self {
            net,
            endpoint: None,
            reads_when_released: false,
            declared: None,
            frame: SenseFrame::Absolute,
            branch_terms: Vec::new(),
            terminal: false,
            capability: DriveCapability::default(),
            link: EngineLink::default(),
        }
    }

    /// Create a wired handle (system-build internal use). `endpoint` is
    /// `None` for pins that cannot drive (power/passive/detached pins).
    pub(crate) fn wired(net: NetId, endpoint: Option<EndpointId>, link: EngineLink) -> Self {
        Self {
            net,
            endpoint,
            reads_when_released: false,
            declared: None,
            frame: SenseFrame::Absolute,
            branch_terms: Vec::new(),
            terminal: false,
            capability: DriveCapability::default(),
            link,
        }
    }

    /// Mark the handle as a `PowerOut` pin's (system-build internal use;
    /// see the field).
    pub(crate) fn on_terminal(mut self, terminal: bool) -> Self {
        self.terminal = terminal;
        self
    }

    /// Record what the pin declared it can do to its net (system-build
    /// internal use; see the field).
    pub(crate) fn declaring(mut self, capability: DriveCapability) -> Self {
        self.capability = capability;
        self
    }

    /// Mark the handle as a released bidirectional pad's: a sense
    /// subscription through it declares the net read (system-build internal
    /// use; see the field).
    pub(crate) fn reading_when_released(mut self, reads: bool) -> Self {
        self.reads_when_released = reads;
        self
    }

    /// Record the pin's declared thresholds and the nets they are read
    /// against (system-build internal use; see the field).
    pub(crate) fn with_thresholds(mut self, declared: Option<DeclaredThresholds>) -> Self {
        self.declared = declared.map(Arc::new);
        self
    }

    /// Record what the pin's sense is measured against (system-build
    /// internal use; see the field).
    pub(crate) fn measured_in(mut self, frame: SenseFrame) -> Self {
        self.frame = frame;
        self
    }

    /// The net of the pin's declared reference, when it declares one on a
    /// net — the net whose moves re-deliver the pin's sense.
    pub(crate) fn reference_net(&self) -> Option<NetId> {
        match self.frame {
            SenseFrame::Against(net) if net != self.net => Some(net),
            _ => None,
        }
    }

    /// The net of the supply the pin's declared thresholds are relative
    /// to, when it names one on a net other than its own and its
    /// reference's — the net whose moves re-deliver the pin's sense, so a
    /// receiver re-projects the voltage it holds through the thresholds
    /// the moved supply scales ([`Self::thresholds`]).
    pub(crate) fn supply_net(&self) -> Option<NetId> {
        let declared = self.declared.as_deref().filter(|d| d.relative)?;
        declared
            .supply
            .filter(|&net| net != self.net && Some(net) != self.reference_net())
    }

    /// Record the elements this pin terminates (system-build internal use;
    /// see the field).
    pub(crate) fn with_branch_terms(mut self, terms: Vec<(usize, f64)>) -> Self {
        self.branch_terms = terms;
        self
    }

    /// Whether the engine can report a current into this pin at all: it
    /// has a drive slot, or it terminates a declared branch. A power-in or
    /// passive pin on no branch carries nothing the solve accounts for,
    /// and a terminal's (a `PowerOut` pin's) current spans clusters.
    pub(crate) fn carries_current(&self) -> bool {
        (self.endpoint.is_some() && !self.terminal) || !self.branch_terms.is_empty()
    }

    /// The net this pin is attached to.
    pub fn net(&self) -> NetId {
        self.net
    }

    /// The current flowing **into** this pin from its net, from the last
    /// solve of the pin's cluster — an instrument, not the normal path
    /// (`NODES.md` §2: every pin is also an I-V port). For a pin with a drive
    /// slot it is the current the pin's own Thevenin source sinks,
    /// `(V(net) − V_oc) / Z` (a sink holding a pulled-up line low reads
    /// positive); for a current drive, the negative of the injection; for a
    /// pin terminating a declared [`Branch`], the branch current into it
    /// (positive at `a`, negative at `b`), added to the above.
    ///
    /// `None` when no solve has produced one: the cluster resolved by
    /// projection alone (only an escalated cluster has node voltages —
    /// subscribe through [`ComponentNetIo::on_branch`] to escalate it), the
    /// net floats, or the pin has no slot and no branch. On the build path
    /// this reads the build snapshot.
    pub fn sense_current(&self) -> Option<Amps> {
        let table = self.link.currents.lock().unwrap();
        let mut total: Option<Amps> = None;
        if let Some(endpoint) = self.endpoint {
            if let Some(Some(amps)) = table.endpoints.get(endpoint.0) {
                total = Some(*amps);
            }
        }
        for (element, sign) in &self.branch_terms {
            if let Some(Some(amps)) = table.elements.get(*element) {
                total = Some(total.unwrap_or(0.0) + sign * amps);
            }
        }
        total
    }

    /// The pin's declared [`PinDecl::thresholds`] **in volts** against its
    /// reference, at this instant: a relative declaration (the pin names a
    /// [`PinDecl::supply`]) scaled by the supply's span — the voltage the
    /// supply pin's net names, less the reference pin's when the pin
    /// declares one ([`Thresholds::scaled`]) — so a bank that browns out to
    /// 1.8 V moves a `0.3/0.7 × VIO` receiver to 0.54 V / 1.26 V; an
    /// absolute one as declared.
    ///
    /// `None` when the pin declares no thresholds, or when a relative
    /// declaration has no span to scale by: the supply is on no net, or the
    /// supply or the reference names no voltage — floating, only an
    /// unmodelled rail behind it, a clock, a reference pin that is
    /// detached. Nothing is invented for it (`DESIGN.md` rule 6). Reads
    /// the live engine's most recent publication, or the build snapshot on
    /// the analysis path, as [`Self::sense`] does — consistent on the
    /// engine thread only, as it is.
    pub fn thresholds(&self) -> Option<Thresholds> {
        let declared = self.declared.as_deref()?;
        if !declared.relative {
            return Some(declared.thresholds);
        }
        let table = &self.link.volts;
        let supply = match declared.supply {
            Some(net) => table.dc(net.0)?,
            None => return None,
        };
        let reference = match self.frame {
            SenseFrame::Absolute => 0.0,
            SenseFrame::Against(net) => table.dc(net.0)?,
            SenseFrame::Detached => return None,
        };
        if !(supply.is_finite() && reference.is_finite()) {
            return None;
        }
        Some(declared.thresholds.scaled(supply - reference))
    }

    /// The receiver's projection of a `sense` delivered through this pin:
    /// [`Sense::level`] through the pin's declared thresholds at this
    /// instant ([`Self::thresholds`] — a relative declaration scaled by its
    /// supply now), chosen by the `last` level the receiver read. `None`
    /// when the pin declares no thresholds (it is handed volts only), when
    /// a relative declaration has no supply to scale by, or when the sense
    /// names no voltage.
    pub fn level(&self, sense: &Sense, last: Option<Level>) -> Option<Level> {
        sense.level(&self.thresholds()?, last)
    }

    /// What the pin senses now — the [`Sense`] a subscription through it
    /// would be handed at this instant: the live engine's most recent
    /// publication, or the build-time snapshot on the analysis path. An
    /// identity-only handle (no table behind it) senses nothing.
    ///
    /// **A poll, not a delivery**: `at_ns` is the caller's virtual now (0
    /// on a build), not an instant the engine delivered at, and a model's
    /// timing reads its `on_sense` deliveries. Consistent on the engine
    /// thread — inside a callback, where every model reads; a caller on
    /// another thread may pair a state from one pass with voltages from
    /// the next (the state table and the voltages are published apart,
    /// the voltages as separate words), so a protocol thread reads what
    /// its callbacks recorded instead.
    pub fn sense(&self) -> Sense {
        self.measure(&self.published())
    }

    /// An **instrument's** read of the engine's own report of the attached
    /// net ([`NetState`]), as [`ComponentNetIo::on_net_report`] delivers
    /// it: what a bench master or probe asserts. A model reads
    /// [`Self::sense`]. Identity-only handles report
    /// [`NetState::Floating`].
    pub fn net_report(&self) -> NetState {
        self.net_state()
    }

    /// The engine's report of the attached net ([`NetState`]): what the
    /// build path hands a sense subscription to convert.
    pub(crate) fn net_state(&self) -> NetState {
        self.link
            .states
            .lock()
            .unwrap()
            .get(self.net.0)
            .copied()
            .unwrap_or(NetState::Floating)
    }

    /// The attached net's resolution as last published — its state, its
    /// voltage and its reference's — read from the tables the engine (or
    /// the build) published: what [`Self::sense`] and a registration on
    /// the build path measure.
    pub(crate) fn published(&self) -> Delivery {
        let table = &self.link.volts;
        Delivery {
            state: self.net_state(),
            node: table.load(self.net.0),
            reference: self.reference_net().map(|net| table.load(net.0)),
        }
    }

    /// What this pin is handed for a delivery: the net's voltage and its
    /// reference's as the pass resolved them, measured in the pin's frame,
    /// at the virtual instant now — the one conversion on the live and the
    /// build path alike.
    pub(crate) fn measure(&self, delivery: &Delivery) -> Sense {
        // A build has no instant: the virtual clock another system (or an
        // earlier test) initialised is not this build's, so an inert link
        // hands 0, as `Sense::at_ns` says.
        let at_ns = if self.link.tx.is_some() && embsim_core::virtual_clock::is_initialized() {
            embsim_core::virtual_clock::virtual_ns()
        } else {
            0
        };
        // A reference on the pin's own net is the node itself.
        let reference = match self.frame {
            SenseFrame::Against(net) if net == self.net => Some(delivery.node),
            SenseFrame::Against(_) => delivery.reference,
            SenseFrame::Absolute | SenseFrame::Detached => None,
        };
        Sense::measured(delivery.state, delivery.node, self.frame, reference, at_ns)
    }

    /// Publish a [`Drive`] on this pin. Drives are enqueued, never applied
    /// inline — the engine thread serializes them by enqueue sequence and
    /// resolves each in a later iteration, so calling this from a sense
    /// callback is safe by construction.
    ///
    /// On the inert build-time path the drive is recorded for the build's
    /// fixed point; for a pin without a drive slot it is traced and dropped.
    pub fn drive(&self, drive: Drive) {
        self.publish(Some(drive));
    }

    /// Release this pin to high-Z: the absence of a drive, sequenced like one.
    pub fn release(&self) {
        self.publish(None);
    }

    /// [`Self::drive`] for the Thevenin encoding, `None` releasing — a thin
    /// alias kept until the last caller moves to [`Self::drive`]: the
    /// in-tree models' and MaD's, due at the phase after the MaD pin bump
    /// (`NODES.md` §12 item 5, the review), when it is deleted.
    pub fn set_drive(&self, drive: Option<TheveninDrive>) {
        self.publish(drive.map(Drive::Thevenin));
    }

    fn publish(&self, drive: Option<Drive>) {
        if let Some(reason) = drive
            .as_ref()
            .and_then(|drive| self.capability.contradicted_by(drive))
        {
            // Published all the same: the engine resolves what the node
            // publishes, and the trace names the declaration it broke.
            tracing::warn!(net = self.net.0, ?drive, "{reason}");
        }
        let Some(endpoint) = self.endpoint else {
            tracing::debug!(
                net = self.net.0,
                "drive on a pin without a drive slot dropped"
            );
            return;
        };
        let seq = self.link.next_drive_seq();
        self.link.send(Command::Drive {
            seq,
            endpoint,
            drive,
        });
    }
}

/// A digital receiver's memory: the pin it reads through and the level it
/// last read — what a model keeps per sensing pin so its projection is
/// chosen by its last level ([`PinHandle::level`], `NODES.md` §11). One
/// per pin, captured by that pin's sense callback.
#[derive(Debug)]
pub struct DigitalReceiver {
    pin: PinHandle,
    /// The level last read: 0 none, 1 low, 2 high. An atomic, not a lock:
    /// a receiver reads once per delivery, on the engine thread.
    last: std::sync::atomic::AtomicU8,
}

impl DigitalReceiver {
    /// A receiver reading through `pin`, that has read nothing yet.
    pub fn new(pin: PinHandle) -> Self {
        Self {
            pin,
            last: std::sync::atomic::AtomicU8::new(0),
        }
    }

    /// Project `sense` through the pin's declared thresholds, chosen by the
    /// level last read, and remember the result as the new last level —
    /// `None` included: an input that read no level (floating, or inside a
    /// dead band it has no memory for) has no level to hold next time.
    pub fn read(&self, sense: &Sense) -> Option<Level> {
        let level = self.pin.level(sense, self.last());
        let code = match level {
            None => 0,
            Some(Level::Low) => 1,
            Some(Level::High) => 2,
        };
        self.last.store(code, std::sync::atomic::Ordering::Relaxed);
        level
    }

    /// The level last read.
    pub fn last(&self) -> Option<Level> {
        match self.last.load(std::sync::atomic::Ordering::Relaxed) {
            1 => Some(Level::Low),
            2 => Some(Level::High),
            _ => None,
        }
    }
}

/// A wake handler, as a [`WakeGate`] is handed it by
/// [`ComponentNetIo::on_wake_ns`].
pub type WakeHandler = Box<dyn Fn(u64) + Send>;

/// The time of a node another node **hosts**: the hosted node's wake
/// handler and its schedules, routed through its host instead of straight
/// to the engine ([`ComponentNetIo::with_wake_gate`]).
///
/// A package around a core is the case (`embsim-boards`' `P2Package`):
/// whether the chip may run — its reset, its core supply, the datasheet's
/// restart delay — is the package's to decide, whichever core runs inside
/// it, so the package takes the core's wakes and lands them when the chip
/// can run. The core still reaches the engine through the one interface
/// (`NODES.md` §11): the gate sits between its wake requests and the
/// engine's timer wheel and changes nothing else — pins, senses and drives
/// are the host's handle table as it is.
pub trait WakeGate: Send + Sync {
    /// The hosted node registered its wake handler (one per node; a later
    /// registration replaces an earlier one, as on the engine).
    fn on_wake_ns(&self, handler: WakeHandler);
    /// The hosted node asked to be woken at `at_ns`.
    fn schedule_at_ns(&self, at_ns: u64);
    /// The hosted node asked to be woken every `period_ns`.
    fn schedule_every_ns(&self, period_ns: u64);
}

/// A [`WakeGate`] as a handle table carries it.
#[derive(Clone)]
struct GatedWakes(Arc<dyn WakeGate>);

impl fmt::Debug for GatedWakes {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("GatedWakes")
    }
}

/// Per-component net I/O passed to [`Component::attach`]: typed pin-handle
/// lookup, sense subscription, and engine-owned scheduling.
#[derive(Debug, Clone, Default)]
pub struct ComponentNetIo {
    /// Keyed by BOTH the netlist pin number and the declared pin name (when
    /// present), so `io.pin("3")` and `io.pin("RX")` resolve identically.
    pins: HashMap<String, PinHandle>,
    component: Option<ComponentId>,
    link: EngineLink,
    /// The build-time topology behind [`Self::resistors_at`] and
    /// [`Self::node`]; `None` on a handle table built without a system
    /// (tests), where both answer with an error naming the fact.
    topology: Option<Arc<BuildTopology>>,
    /// The host a hosted node's wakes go through
    /// ([`Self::with_wake_gate`]); `None` for a node the engine wakes
    /// directly.
    wake_gate: Option<GatedWakes>,
}

impl ComponentNetIo {
    /// Build an inert handle table (board-build internal use; tests). Insert
    /// each handle under every identity it answers to (pin number, declared
    /// name).
    pub fn from_entries(entries: impl IntoIterator<Item = (String, PinHandle)>) -> Self {
        Self {
            pins: entries.into_iter().collect(),
            component: None,
            link: EngineLink::default(),
            topology: None,
            wake_gate: None,
        }
    }

    /// Build a wired handle table (system-build internal use).
    pub(crate) fn wired(
        entries: impl IntoIterator<Item = (String, PinHandle)>,
        component: Option<ComponentId>,
        link: EngineLink,
    ) -> Self {
        Self {
            pins: entries.into_iter().collect(),
            component,
            link,
            topology: None,
            wake_gate: None,
        }
    }

    /// The same handle table for a node this one **hosts**: its wake
    /// handler and its schedules ([`Self::on_wake_ns`],
    /// [`Self::schedule_at_ns`], [`Self::schedule_every_ns`] and their
    /// microsecond forms) go to `gate`, which decides when they reach the
    /// engine; pins, senses and drives are this table's. What a package
    /// hands the core inside it, so the chip's START gate holds any core
    /// the same way (see [`WakeGate`]).
    pub fn with_wake_gate(mut self, gate: Arc<dyn WakeGate>) -> Self {
        self.wake_gate = Some(GatedWakes(gate));
        self
    }

    /// The same handle table with the build-time topology behind it
    /// (system-build internal use).
    pub(crate) fn with_topology(mut self, topology: Arc<BuildTopology>) -> Self {
        self.topology = Some(topology);
        self
    }

    /// The **node** a pin is on: the identity root of its net, with every
    /// harness merge and closed pole applied — what a resistor's far end is
    /// reported as by [`Self::resistors_at`], so a part can tell which of
    /// the resistors on its feedback pin returns to its own ground pin.
    pub fn node(&self, id: &str) -> Result<NetId, AttachError> {
        let handle = self.pin(id)?;
        let topology = self.topology.as_ref().ok_or_else(|| AttachError::Failed {
            message: format!("pin {id:?}: no build topology behind this handle table"),
        })?;
        let net = handle.net().0;
        Ok(NetId(topology.root_of.get(net).copied().unwrap_or(net)))
    }

    /// The resistors touching the node a pin is on — a **build-time
    /// topology query**, read once at attach and never again (`NODES.md`
    /// §5): the divider a regulator's output voltage is set by, the strap a
    /// select pin is tied through. Each entry names the resistor, its ohms
    /// and the node at its far end ([`ResistorAt::far`], comparable with
    /// [`Self::node`]). Only resistors with a parsed value and both pads
    /// fitted are reported; a resistor with both ends on this node is not.
    /// Fails on a handle table with no build behind it.
    pub fn resistors_at(&self, id: &str) -> Result<Vec<ResistorAt>, AttachError> {
        let node = self.node(id)?;
        let topology = self
            .topology
            .as_ref()
            .expect("node() succeeded, so the topology is present");
        Ok(topology.resistors.get(&node.0).cloned().unwrap_or_default())
    }

    /// Look up a pin handle by declared name or netlist pin number.
    pub fn pin(&self, id: &str) -> Result<PinHandle, AttachError> {
        self.pins
            .get(id)
            .cloned()
            .ok_or_else(|| AttachError::UnknownPin {
                pin: id.to_string(),
            })
    }

    /// Subscribe to what the pin senses (`NODES.md` §11, `on_sense`): the
    /// callback is handed a [`Sense`] — the node's voltage against the
    /// pin's declared reference, the square wave when the node carries one,
    /// and the instant — and projects it itself, through its own thresholds
    /// ([`PinHandle::level`], [`Sense::level`]). It runs on the engine
    /// thread with **no engine lock held**; the current sense is delivered
    /// once at registration (so a floating net is reported before any
    /// traffic), then on every change — of the net's state, of the voltage
    /// behind it, of the reference it is measured against, or of the supply
    /// its declared thresholds are relative to (the same [`Sense`] again,
    /// so a [`DigitalReceiver`] re-projects it through the thresholds the
    /// supply now scales, from its last level). A callback
    /// MAY drive a pin — the drive is enqueued and resolved in a later
    /// engine iteration.
    ///
    /// The build path and the live path hand the same [`Sense`] for the
    /// same resolution: both convert the engine's resolved net through the
    /// one [`PinHandle`] measurement, against the table published with it
    /// (the two-code-paths rule — a component's floating-detection must
    /// behave identically under `System::build` and `System::start`).
    ///
    /// Through a released bidirectional pad
    /// ([`PinDecl::reads_when_subscribed`]) the subscription is also the
    /// declaration that the pad **reads** its net: the net joins the
    /// digital senses, and a floating one is reported as
    /// [`crate::Finding::FloatingSense`] — on the build path and the live
    /// path alike. A pad nothing subscribes to is read by nothing and
    /// floats without a finding.
    pub fn on_sense(
        &self,
        id: &str,
        callback: impl Fn(Sense) + Send + 'static,
    ) -> Result<(), AttachError> {
        let handle = self.pin(id)?;
        let measure = handle.clone();
        self.subscribe(
            &handle,
            Box::new(move |delivery| callback(measure.measure(delivery))),
        );
        Ok(())
    }

    /// An **instrument's** subscription to the engine's own report of the
    /// net behind a pin, [`NetState`], delivered when and as
    /// [`Self::on_sense`] delivers: what a bench probe records to assert
    /// the engine's projection — `Driven`, `Pulled`, `Contention`, a
    /// periodic state's engine levels — the way `BuiltSystem::net_state`,
    /// the event log and the goldens read it.
    ///
    /// A model never reads it. What a node sees is its [`Sense`], projected
    /// through its own thresholds; the engine's report is the engine's, its
    /// levels the net-level JESD8C.01 pair, and a part that branched on it
    /// would be reading a projection that is not its own (`DESIGN.md` rule
    /// 2 — one delivery). Like [`PinHandle::sense_current`], it is an
    /// instrument, not the normal path.
    pub fn on_net_report(
        &self,
        id: &str,
        callback: impl Fn(NetState) + Send + 'static,
    ) -> Result<(), AttachError> {
        let handle = self.pin(id)?;
        self.subscribe(&handle, Box::new(move |delivery| callback(delivery.state)));
        Ok(())
    }

    /// Register a sense subscription through `handle`, on the live engine
    /// or — on the inert build path — synchronously against the build
    /// snapshot and recorded for the build's fixed point.
    fn subscribe(&self, handle: &PinHandle, callback: crate::engine::SenseCallback) {
        if self.link.tx.is_none() {
            // Inert build-path link: the engine that would deliver the
            // once-at-registration state does not exist, so honor the same
            // contract synchronously against the build-resolved snapshot —
            // a component's floating-detection must behave identically on
            // `System::build` and `System::start` (the two-code-paths
            // divergence the shared resolver exists to prevent). The
            // callback is then recorded so the build's fixed point can
            // deliver the states its replayed attach drives change, the
            // way the live engine would (`System::build`).
            callback(&handle.published());
            // The build holds the log's one strong reference; a dead weak
            // (a handle used after the build returned) records nothing.
            if let Some(log) = self.link.recorded_senses.as_ref().and_then(Weak::upgrade) {
                log.lock()
                    .expect("sense log never poisoned")
                    .push(crate::engine::RecordedSense {
                        net: handle.net(),
                        reference: handle.reference_net(),
                        supply: handle.supply_net(),
                        reads: handle.reads_when_released,
                        callback: crate::engine::RecordedCallback::State(callback),
                    });
            }
            return;
        }
        if handle.reads_when_released {
            self.link.send(Command::DeclareRead {
                net: handle.net(),
                kind: ReadKind::Digital,
            });
        }
        self.link.send(Command::RegisterSense {
            net: handle.net(),
            reference: handle.reference_net(),
            supply: handle.supply_net(),
            callback,
        });
    }

    /// Subscribe to the current into a pin ([`PinHandle::sense_current`]):
    /// the bench instrument of `NODES.md` §2, delivered like a sense — once
    /// at registration, then whenever a solve changes it — on the engine
    /// thread with no lock held.
    ///
    /// Subscribing **escalates the pin's cluster**, and nothing else: only
    /// a solved cluster has node voltages, so the net joins the current
    /// instruments and every pass over that cluster from now on solves it;
    /// an instrument on an open loop is not a floating input (no
    /// [`crate::Finding::FloatingSense`]), and rule 2's fight findings are
    /// still reported in its cluster. Refused, at attach,
    /// on a pin the engine cannot account a current for — a power or
    /// passive pin that terminates no declared [`Branch`], a terminal whose
    /// current spans clusters.
    pub fn on_branch(
        &self,
        id: &str,
        callback: impl Fn(Option<Amps>) + Send + 'static,
    ) -> Result<(), AttachError> {
        let handle = self.pin(id)?;
        if !handle.carries_current() {
            return Err(AttachError::Failed {
                message: format!(
                    "pin {id:?} carries no current the solve accounts for: it has no drive \
                     slot and terminates no declared branch"
                ),
            });
        }
        if self.link.tx.is_none() {
            // Inert build path: the same once-at-registration delivery,
            // synchronously against the snapshot, then recorded so the
            // build's fixed point can escalate the cluster and deliver the
            // current its solve produces (`System::build`).
            let last = handle.sense_current();
            callback(last);
            if let Some(log) = self.link.recorded_senses.as_ref().and_then(Weak::upgrade) {
                log.lock()
                    .expect("sense log never poisoned")
                    .push(crate::engine::RecordedSense {
                        net: handle.net(),
                        reference: None,
                        supply: None,
                        reads: false,
                        callback: crate::engine::RecordedCallback::Current {
                            handle,
                            callback: Box::new(callback),
                            last,
                        },
                    });
            }
            return Ok(());
        }
        self.link.send(Command::DeclareRead {
            net: handle.net(),
            kind: ReadKind::Instrument,
        });
        self.link.send(Command::RegisterCurrent {
            handle,
            callback: Box::new(callback),
        });
        Ok(())
    }

    /// Register this component's wakeup handler for
    /// [`schedule_at`](Self::schedule_at) /
    /// [`schedule_every`](Self::schedule_every) deliveries (last
    /// registration wins). The callback runs on the engine thread with the
    /// current virtual time (µs) and no engine lock held.
    ///
    /// A component whose time comes from here is **deterministic for free** in
    /// stepped clock mode: it is not a separate actor at all, so nothing about
    /// it can race the engine (`DETERMINISM.md` T1 §4). Prefer a wakeup over a
    /// thread with its own poll loop wherever the work is non-blocking —
    /// `embsim_models::ads122u04_component` is the reference conversion.
    pub fn on_wake(&self, callback: impl Fn(u64) + Send + 'static) {
        self.on_wake_ns(move |ns| callback(ns / 1_000));
    }

    /// [`on_wake`](Self::on_wake) with the timestamp in **nanoseconds**.
    ///
    /// One wake handler per component either way — registering one replaces
    /// the other. Take this form when the component's own events are closer
    /// together than a microsecond (a UART bit at 2 Mbaud is 500 ns), and the
    /// microsecond form everywhere else.
    pub fn on_wake_ns(&self, callback: impl Fn(u64) + Send + 'static) {
        if let Some(GatedWakes(gate)) = &self.wake_gate {
            gate.on_wake_ns(Box::new(callback));
            return;
        }
        let Some(component) = self.component else {
            tracing::debug!("on_wake on an inert io handle dropped");
            return;
        };
        // Control plane, so it is ordered ahead of the schedules that
        // depend on it: a wake armed before its handler is registered fires
        // into nothing.
        self.link.send_control(Command::RegisterWake {
            component,
            callback: Box::new(callback),
        });
    }

    /// Request a one-shot wakeup at the given absolute virtual time (µs),
    /// served by the engine thread's timer wheel. A deadline already in the
    /// past fires immediately, in deadline order. Requires
    /// `virtual_clock::init`.
    ///
    /// Free-running: the delivered timestamp is *sampled* from the scaled
    /// clock, so a wake lands at or after its deadline by an unspecified
    /// margin. Stepped: the engine advances virtual time **to** the deadline,
    /// so the delivered timestamp is exactly `at_us`.
    pub fn schedule_at(&self, at_us: u64) {
        self.schedule_at_ns(at_us.saturating_mul(1_000));
    }

    /// [`schedule_at`](Self::schedule_at) with a **nanosecond** deadline.
    pub fn schedule_at_ns(&self, at_ns: u64) {
        if let Some(GatedWakes(gate)) = &self.wake_gate {
            gate.schedule_at_ns(at_ns);
            return;
        }
        let Some(component) = self.component else {
            tracing::debug!("schedule_at on an inert io handle dropped");
            return;
        };
        self.link
            .send_control(Command::ScheduleAt { component, at_ns });
    }

    /// Request a periodic wakeup every `period_us` of virtual time. Missed
    /// deadlines coalesce (one catch-up fire, then back on period) — compute
    /// time-dependent state at read time, never per tick. Requires
    /// `virtual_clock::init`.
    ///
    /// The period is anchored at the virtual instant the engine *handles* this
    /// request. In stepped mode that instant is pinned for the whole system:
    /// virtual time is held until every component has attached and started, so
    /// two components' periods are anchored together, run after run.
    /// Free-running coalescing is unreachable in stepped mode — the engine
    /// never advances past a deadline it has not fired.
    pub fn schedule_every(&self, period_us: u64) {
        self.schedule_every_ns(period_us.saturating_mul(1_000));
    }

    /// [`schedule_every`](Self::schedule_every) with a **nanosecond** period.
    pub fn schedule_every_ns(&self, period_ns: u64) {
        if let Some(GatedWakes(gate)) = &self.wake_gate {
            gate.schedule_every_ns(period_ns);
            return;
        }
        let Some(component) = self.component else {
            tracing::debug!("schedule_every on an inert io handle dropped");
            return;
        };
        self.link.send_control(Command::ScheduleEvery {
            component,
            period_ns,
        });
    }
}

// ============================================================
// Errors
// ============================================================

/// Failure inside [`Component::attach`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AttachError {
    /// The component asked [`ComponentNetIo::pin`] for an identity the build
    /// did not wire (facade mismatch — fails the build loudly).
    UnknownPin {
        /// The identity that failed to resolve (name or number).
        pin: String,
    },
    /// Component-specific attach failure.
    Failed {
        /// Human-readable cause.
        message: String,
    },
}

impl fmt::Display for AttachError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            AttachError::UnknownPin { pin } => {
                write!(f, "attach: no net handle for pin {pin:?} (facade mismatch)")
            }
            AttachError::Failed { message } => write!(f, "attach failed: {message}"),
        }
    }
}

impl std::error::Error for AttachError {}

#[cfg(test)]
mod tests {
    use rstest::rstest;

    use super::*;

    #[rstest]
    fn pin_lookup_resolves_by_number_and_name() {
        let handle = PinHandle::new(NetId(7));
        let io = ComponentNetIo::from_entries([
            ("3".to_string(), handle.clone()),
            ("RX".to_string(), handle),
        ]);
        assert_eq!(io.pin("3").unwrap().net(), NetId(7));
        assert_eq!(io.pin("RX").unwrap().net(), NetId(7));
    }

    /// Build/live parity: the inert build-path link has no engine to defer
    /// to, so `on_sense` delivers the build-resolved snapshot synchronously,
    /// exactly once — the same once-at-registration contract the live path
    /// honors. A component doing floating-detection in its sense callback
    /// must behave identically under `System::build` and `System::start`.
    #[rstest]
    fn on_sense_inert_link_delivers_the_snapshot_once_synchronously() {
        use crate::engine::EngineLink;
        use crate::net::Level;
        use std::sync::{Arc, Mutex};

        let states = Arc::new(Mutex::new(vec![NetState::Driven(Level::High)]));
        let volts = Arc::new(crate::engine::VoltsTable::of([NetVolts::dc(Some(3.3))]));
        let link = EngineLink::inert(
            (states, volts),
            Arc::new(Mutex::new(crate::engine::CurrentTable::default())),
            Arc::new(Mutex::new(Vec::new())),
            &crate::engine::SenseLog::default(),
        );
        let handle = PinHandle::wired(NetId(0), None, link.clone());
        let io = ComponentNetIo::wired([("1".to_string(), handle)], None, link);

        let log: Arc<Mutex<Vec<Sense>>> = Arc::new(Mutex::new(Vec::new()));
        let sink = Arc::clone(&log);
        io.on_sense("1", move |sense| sink.lock().unwrap().push(sense))
            .unwrap();
        let delivered: Vec<(Option<Volts>, bool)> = log
            .lock()
            .unwrap()
            .iter()
            .map(|sense| (sense.volts, sense.periodic.is_some()))
            .collect();
        assert_eq!(delivered, vec![(Some(3.3), false)]);
    }

    /// What a signal pin reads follows from its declarations alone: an
    /// input with thresholds is a digital sense, one without an analog
    /// reader; an output — push-pull or open-drain — reads nothing; an
    /// output that declares thresholds and rests released is a
    /// bidirectional pad that reads once it subscribes.
    #[rstest]
    #[case::digital_input(
        PinDecl::digital_in("1", jesd8c01_lvcmos_thresholds(DeadBand::Unknown)),
        Some(SenseKind::Digital),
        false
    )]
    #[case::analog_input(PinDecl::analog("1"), Some(SenseKind::Analog), false)]
    #[case::push_pull(PinDecl::digital_out("1"), None, false)]
    #[case::open_drain(PinDecl::digital_out("1").sink_only(), None, false)]
    #[case::bidirectional(PinDecl::digital_out("1").with_idle(None).with_thresholds(jesd8c01_lvcmos_thresholds(DeadBand::Unknown)), None, true)]
    #[case::driven_bidirectional(PinDecl::digital_out("1").with_thresholds(jesd8c01_lvcmos_thresholds(DeadBand::Unknown)), None, false)]
    #[case::supply(PinDecl::power_in("1"), None, false)]
    fn a_pins_declarations_decide_what_it_reads(
        #[case] pin: PinDecl,
        #[case] at_build: Option<SenseKind>,
        #[case] when_subscribed: bool,
    ) {
        use vibes_behaviour::{behaviour, expect, Test};
        behaviour!(Test {
            id: "pin.declarations-decide-the-read",
            covers: Some("board/src/component.rs#PinDecl::senses_at_build"),
            given: "a pin declared as a digital input, an analog input, a push-pull output, an \
                    open-drain output, a released bidirectional pad, a driven bidirectional \
                    pad, or a supply",
        });
        expect!(
            "read-at-build",
            "a pin that drives nothing reads its net from the build, through its thresholds if \
             it has them and as a voltage if not",
            "there are no pin kinds: an input is a pin that drives nothing, and its thresholds \
             say how it reads"
        );
        expect!(
            "read-on-subscribe",
            "only a pad that drives, declares thresholds and rests released reads its net \
             once it subscribes",
            "a bidirectional pad is an input until its owner drives it"
        );
        assert_eq!(pin.senses_at_build(), at_build);
        assert_eq!(pin.reads_when_subscribed(), when_subscribed);
    }

    /// A relative declaration scales every figure, hysteresis included.
    #[rstest]
    fn relative_thresholds_scale_every_figure_by_the_supply() {
        use vibes_behaviour::{behaviour, expect, Test};
        behaviour!(Test {
            id: "pin.thresholds-scale-every-figure",
            covers: Some("board/src/component.rs#Thresholds::scaled"),
            given: "thresholds of 0.3 and 0.7 of the supply with a hysteresis of 0.1 of it, at \
                    a 2 volt supply",
        });
        expect!(
            "every-figure-scaled",
            "they read 0.6 and 1.4 volts with 0.2 volts of hysteresis"
        );
        let scaled = Thresholds::new(0.3, 0.7, 0.1, DeadBand::Unknown).scaled(2.0);
        assert!((scaled.v_il - 0.6).abs() < 1e-12, "{scaled:?}");
        assert!((scaled.v_ih - 1.4).abs() < 1e-12, "{scaled:?}");
        assert!((scaled.hysteresis - 0.2).abs() < 1e-12, "{scaled:?}");
    }

    /// A Schmitt receiver's figures, the SN74LVC1G14's at 3 V (§5.5):
    /// `V_T−` min 0.84 V, `V_T+` max 1.87 V, `ΔV_T` min 0.56 V.
    const SCHMITT: (f64, f64, f64) = (0.84, 1.87, 0.56);

    /// The receiver's projection, case by case: the guaranteed figures
    /// whatever it read last, the hysteresis chosen by its last level, the
    /// declared policy for what is left.
    #[rstest]
    #[case::at_v_il(0.84, Some(Level::High), DeadBand::HoldLast, Some(Level::Low))]
    #[case::at_v_ih(1.87, Some(Level::Low), DeadBand::Unknown, Some(Level::High))]
    #[case::high_held_by_hysteresis(1.35, Some(Level::High), DeadBand::Unknown, Some(Level::High))]
    #[case::low_held_by_hysteresis(1.35, Some(Level::Low), DeadBand::Unknown, Some(Level::Low))]
    #[case::band_hold_last(1.2, Some(Level::High), DeadBand::HoldLast, Some(Level::High))]
    #[case::band_unknown(1.2, Some(Level::High), DeadBand::Unknown, None)]
    #[case::band_with_nothing_to_hold(1.5, None, DeadBand::HoldLast, None)]
    #[case::no_voltage(f64::NAN, Some(Level::High), DeadBand::HoldLast, None)]
    fn a_receiver_projects_through_its_own_thresholds(
        #[case] volts: f64,
        #[case] last: Option<Level>,
        #[case] dead_band: DeadBand,
        #[case] expected: Option<Level>,
    ) {
        use vibes_behaviour::{behaviour, expect, Test};
        behaviour!(Test {
            id: "sense.receiver-projection-rule",
            covers: Some("board/src/component.rs#Thresholds::project"),
            given: "a receiver with a low threshold of 0.84 volts, a high threshold of 1.87 \
                    volts and 0.56 volts of hysteresis, handed a voltage with the level it \
                    read last",
        });
        expect!(
            "guaranteed-figures",
            "at or below the low threshold it reads low and at or above the high one high, \
             whatever it read last"
        );
        expect!(
            "hysteresis-by-last-level",
            "after a high it reads high down to the high threshold less the hysteresis, and \
             after a low, low up to the low threshold plus it"
        );
        expect!(
            "declared-policy-decides-the-rest",
            "between those, a receiver declared to hold reads its last level, and one \
             declared unknown reads none"
        );
        let (v_il, v_ih, hysteresis) = SCHMITT;
        let thresholds = Thresholds::new(v_il, v_ih, hysteresis, dead_band);
        assert_eq!(thresholds.project(volts, last), expected);
        let sensed = Sense {
            volts: volts.is_finite().then_some(volts),
            periodic: None,
            at_ns: 0,
        };
        assert_eq!(sensed.level(&thresholds, last), expected);
    }

    /// A publish is checked against what the pin declared it can do.
    #[rstest]
    #[case::input_drives(PinDecl::analog("1"), Drive::Thevenin(TheveninDrive { volts: 1.0, impedance: 100.0 }), true)]
    #[case::source_drives(PinDecl::analog_source("1"), Drive::Thevenin(TheveninDrive { volts: 1.0, impedance: 100.0 }), false)]
    #[case::open_drain_sinks(PinDecl::digital_out("1").sink_only(), Drive::Current { amps: -0.001 }, false)]
    #[case::open_drain_sources(PinDecl::digital_out("1").sink_only(), Drive::Current { amps: 0.001 }, true)]
    #[case::open_drain_thevenin(PinDecl::digital_out("1").sink_only(), Drive::Thevenin(TheveninDrive { volts: 0.0, impedance: 30.0 }), false)]
    fn a_drive_beyond_the_declared_capability_is_named(
        #[case] pin: PinDecl,
        #[case] drive: Drive,
        #[case] contradicts: bool,
    ) {
        use vibes_behaviour::{behaviour, expect, Test};
        behaviour!(Test {
            id: "pin.drive-checked-against-declaration",
            covers: Some("board/src/component.rs#DriveCapability"),
            given: "an analog reader, a linear source and an output that can only pull low, \
                    each publishing a voltage behind a resistance or a current into or out of \
                    its net",
        });
        expect!(
            "input-drive-named",
            "any drive from a pin declared to neither source nor sink is named as contradicting \
             its declaration"
        );
        expect!(
            "current-direction-checked",
            "a current pushed into the net by a pin that cannot source is named; one drawn out \
             by a pin that can sink is accepted",
            "whether a voltage behind a resistance sources or sinks depends on where its node \
             settles, which only the solve knows; a current's direction is the drive's own"
        );
        expect!(
            "declared-drives-accepted",
            "a linear source's voltage, and a pull-low output sinking to 0 volts, are accepted"
        );
        assert_eq!(
            DriveCapability::of(&pin).contradicted_by(&drive).is_some(),
            contradicts
        );
    }

    /// The linear-source constructor declares a pin that drives and reads
    /// nothing.
    #[rstest]
    fn a_linear_source_is_no_sense() {
        use vibes_behaviour::{behaviour, expect, Test};
        behaviour!(Test {
            id: "pin.linear-source-declaration",
            covers: Some("board/src/component.rs#PinDecl::analog_source"),
            given: "a pin declared as a linear source, such as a bench supply's output",
        });
        expect!(
            "drives-both-ways",
            "it may source and sink, and rests released until its part drives it"
        );
        expect!(
            "no-sense",
            "its net is not read because of it, so it never asks for a solved voltage"
        );
        let pin = PinDecl::analog_source("S+");
        assert!(pin.can_source && pin.can_sink);
        assert_eq!(pin.idle, None);
        assert_eq!(pin.senses_at_build(), None);
        assert_eq!(
            PinDecl::analog("S+").senses_at_build(),
            Some(SenseKind::Analog)
        );
    }

    /// A clock is projected phase by phase through the receiver's own
    /// thresholds: two levels are a clock it reads no single level from,
    /// one level is no edge it can see.
    #[rstest]
    #[case::swing_across_the_thresholds(3.3, 0.0, None)]
    #[case::swing_below_v_il(1.2, 0.0, Some(Level::Low))]
    #[case::swing_above_v_ih(3.3, 2.5, Some(Level::High))]
    #[case::a_phase_in_the_band(1.5, 0.0, None)]
    fn a_receiver_projects_a_clock_phase_by_phase(
        #[case] hi: f64,
        #[case] lo: f64,
        #[case] expected: Option<Level>,
    ) {
        use vibes_behaviour::{behaviour, expect, Test};
        behaviour!(Test {
            id: "sense.clock-projected-per-phase",
            covers: Some("board/src/component.rs#Sense::level"),
            given: "a receiver with a low threshold of 1.3 volts and a high threshold of 2 volts, \
                    handed a running square wave whose two phases sit at various voltages",
        });
        expect!(
            "one-level-no-edge",
            "where both phases read one level through its thresholds it reads that level",
            "a swing the receiver's thresholds see no edge in is a steady level to it",
        );
        expect!(
            "two-levels-no-level",
            "where the phases read two levels, or one reads none, it reads no level"
        );
        let thresholds = Thresholds::new(1.3, 2.0, 0.0, DeadBand::Unknown);
        let sensed = Sense {
            volts: None,
            periodic: Some(PeriodicSense {
                hi: Some(hi),
                lo: Some(lo),
                segment: PeriodicSchedule {
                    emitted: 0,
                    freq_hz: 1_000,
                    total: None,
                    since_ns: 0,
                },
            }),
            at_ns: 0,
        };
        assert_eq!(sensed.level(&thresholds, None), expected);
    }

    /// What a pin is handed is its net's voltage against its reference:
    /// the engine's frame with none declared, less the reference's voltage
    /// with one, and nothing when the reference names none.
    #[rstest]
    #[case::absolute(SenseFrame::Absolute, None, Some(3.3))]
    #[case::against_a_reference(SenseFrame::Against(NetId(1)), Some(1.0), Some(2.3))]
    #[case::against_a_floating_reference(SenseFrame::Against(NetId(1)), None, None)]
    #[case::against_a_detached_reference(SenseFrame::Detached, None, None)]
    fn a_sense_is_measured_against_the_pins_reference(
        #[case] frame: SenseFrame,
        #[case] reference: Option<Volts>,
        #[case] expected: Option<Volts>,
    ) {
        use vibes_behaviour::{behaviour, expect, Test};
        behaviour!(Test {
            id: "sense.measured-against-reference",
            covers: Some("board/src/component.rs#Sense"),
            given: "a pin on a 3.3 volt net or a 3.3-to-0 volt square wave, against no reference, \
                    a 1 volt one, a floating one or a detached one",
        });
        expect!(
            "volts-against-the-reference",
            "it is handed the net's voltage less its reference's, the engine's own voltage with \
             no reference, and none when its reference names none"
        );
        expect!(
            "phases-against-the-reference",
            "a square wave's two phases are measured against the same reference"
        );
        let clock = PeriodicSchedule {
            emitted: 0,
            freq_hz: 1_000,
            total: None,
            since_ns: 0,
        };
        let reference = Some(NetVolts::dc(reference));
        let dc = Sense::measured(
            NetState::Analog(3.3),
            NetVolts::dc(Some(3.3)),
            frame,
            reference,
            7,
        );
        assert_eq!(dc.volts.map(|v| (v * 1e9).round() / 1e9), expected);
        assert_eq!((dc.periodic, dc.at_ns), (None, 7));
        let square = Sense::measured(
            NetState::Periodic {
                hi: Level::High,
                lo: Level::Low,
                segment: clock,
            },
            NetVolts {
                dc: None,
                phases: Some((Some(3.3), Some(0.0))),
            },
            frame,
            reference,
            7,
        );
        assert_eq!(square.volts, None);
        let offset = expected.map(|v| 3.3 - v);
        assert_eq!(
            square.periodic.map(|p| (p.hi, p.lo, p.segment)),
            offset.map(|offset| (Some(3.3 - offset), Some(0.0 - offset), clock)),
        );
    }

    #[rstest]
    fn pin_lookup_fails_loudly_on_facade_mismatch() {
        let io = ComponentNetIo::default();
        assert_eq!(
            io.pin("TX"),
            Err(AttachError::UnknownPin {
                pin: "TX".to_string()
            })
        );
        assert_eq!(
            io.on_sense("TX", |_| {}),
            Err(AttachError::UnknownPin {
                pin: "TX".to_string()
            })
        );
    }
}
