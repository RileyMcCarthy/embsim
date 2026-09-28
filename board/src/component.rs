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
//! Pulse-capable pins get a **pulse I/O surface**
//! ([`ComponentNetIo::pulse_tx`] / [`ComponentNetIo::on_pulse`]), derived from
//! and gated by net resolution, carrying a *rate* ([`PulseTrain`]) — see
//! [`StreamRole::PulseSource`] for why a step clock is not modeled as edges,
//! and why a UART now is.

use std::collections::HashMap;
use std::fmt;
use std::sync::{Arc, Weak};

use crate::engine::{Command, ComponentId, EndpointId, EngineLink, ReadKind};
use crate::net::{Amps, NetId, NetState, Ohms, TheveninDrive, Volts};

pub use embsim_peripherals::pulse_out::PulseSegment;

// ============================================================
// Pin declarations
// ============================================================

/// Electrical role of a declared pin.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PinKind {
    /// Senses net level; contributes no drive.
    DigitalIn,
    /// Push-pull Thevenin driver (default 25 Ω).
    DigitalOut,
    /// Driver with runtime direction (GPIO).
    DigitalBidir,
    /// Participates in cluster solve (high-Z sense, source, or parameterized
    /// primitive — see the transducer-component rules in `BOARD_ENGINE.md`).
    Analog,
    /// Consumes a power domain.
    PowerIn,
    /// Sources a power domain at a declared voltage.
    PowerOut,
    /// Terminal of a passive primitive (R/C/L/jumper).
    Passive,
}

/// Channel role of a pin: a **pulse train** (a step clock), carried as a rate.
/// The pin's [`PinKind`] stays digital; the channel is derived from and gated
/// by net resolution, never installed beside it.
///
/// There used to be `Producer`/`Consumer` variants here, carrying UART bytes.
/// They are gone: a byte is not a thing a net can hold, and routing one past
/// the resolution meant a UART could not experience contention, a fighting
/// driver, or a floating line. Bytes are now framed onto the net as levels by
/// [`crate::SerialLevelBridge`], and the only signal left that is *not* carried
/// as its own waveform is the one for which a rate is exactly lossless.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum StreamRole {
    /// Emits a pulse train onto the net **as a rate**, not as edges (a
    /// step-clock output; see [`PulseTrain`]).
    ///
    /// # Why a rate and not edges
    ///
    /// A step clock is the one digital signal whose *information* is its
    /// frequency and whose edge count runs far ahead of anything else on the
    /// board. On the reference machine — 8192 steps/mm — a single mm/s of
    /// carriage speed is 8192 edges/s, each of which would be a drive command,
    /// a resolution pass over the STEP cluster, and a sense delivery through
    /// the single-writer engine; a realistic 50 mm/s traverse is over 400 k
    /// engine events per second, for a signal whose consumer only ever
    /// reconstructs `frequency × time` from them.
    ///
    /// So the wire carries the *segment*: one event per **rate change**
    /// (`start` / retarget / `stop`), and consumers integrate at read time —
    /// the same read-time discipline `DETERMINISM.md` mandates and
    /// `embsim_models::machine::stepper_motor` already uses for its plant. The
    /// engine cost of a move becomes a small constant instead of a function of
    /// speed, and the count stays **exact**: [`PulseTrain::emitted_at`] is the
    /// same integer arithmetic the pulse-out peripheral hands the firmware, so
    /// an encoder fed from it cannot drift from the firmware's own view.
    ///
    /// Fidelity limits are stated on [`PulseTrain`].
    PulseSource,
    /// Observes a routed pulse train (a step/direction drive's STEP input).
    ///
    /// The sink is delivered a [`PulseTrain`] at registration (when a routed
    /// source already has one) and on every subsequent rate change — never per
    /// pulse. Between deliveries the sink integrates the train itself.
    PulseSink,
}

// ============================================================
// Pulse trains
// ============================================================

/// Direction a pulse train advances a downstream counter.
///
/// A pulse-out peripheral has no direction of its own (a step clock is one
/// wire); a source that *does* know its direction — an MCU whose pulse channel
/// declares a direction GPIO — stamps it here so the train is self-describing
/// for a sink that has no DIR pin of its own.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum PulseDirection {
    /// Pulses increase the count.
    #[default]
    Forward,
    /// Pulses decrease the count.
    Reverse,
}

impl PulseDirection {
    /// `+1` forward, `-1` reverse.
    pub fn sign(self) -> i64 {
        match self {
            PulseDirection::Forward => 1,
            PulseDirection::Reverse => -1,
        }
    }

    /// The direction implied by a sign: negative is [`Self::Reverse`].
    pub fn from_sign(sign: i64) -> Self {
        if sign < 0 {
            PulseDirection::Reverse
        } else {
            PulseDirection::Forward
        }
    }
}

/// One constant-rate segment of a pulse train as it appears on a net:
/// frequency, direction, and accumulated count.
///
/// This is the whole state of the channel from
/// [`PulseSegment::since_us`] onward, which is what makes one event per rate
/// change sufficient. A consumer keeps the latest train and evaluates it at
/// **read time**:
///
/// ```
/// use embsim_board::{PulseDirection, PulseSegment, PulseTrain};
///
/// // 8192 steps/s, unbounded, starting at t = 1 000 µs with 0 emitted.
/// let train = PulseTrain {
///     pulses: PulseSegment { emitted: 0, freq_hz: 8_192, total: None, since_us: 1_000 },
///     direction: PulseDirection::Reverse,
/// };
/// // One second later, exactly 8192 pulses have gone out, counting down.
/// assert_eq!(train.emitted_at(1_001_000), 8_192);
/// assert_eq!(train.delta_at(1_001_000), -8_192);
/// ```
///
/// # How to fold a sequence of segments
///
/// A segment is superseded, never continued: when the next one arrives, the
/// outgoing segment is folded **up to its successor's
/// [`PulseSegment::since_us`]**, and the successor's own baseline takes over
/// from there. Folding a superseded segment to *now* instead would keep an
/// unbounded train integrating forever and double-count every pulse its
/// successor already carries.
///
/// Within one segment, evaluate against the anchor the source published — do
/// not re-base per read. [`PulseSegment::rebased_at`] explains why (it costs
/// the source's pulse phase);
/// `embsim_models::machine::stepper_motor` is the reference consumer.
///
/// # Fidelity limits
///
/// - **There are no edges.** The source pin's resolved [`NetState`] holds its
///   idle level for the whole train — it does not toggle. A consumer that
///   counts `NetState` transitions sees nothing; it must declare
///   [`StreamRole::PulseSink`]. Pulse width, duty cycle, rise time and jitter
///   are therefore not modeled, and neither is DIR setup/hold against an
///   individual step edge (direction applies to a whole segment).
/// - **Counts are exact at the peripheral's own truncation.** `emitted_at`
///   floors `elapsed_us × freq / 1_000_000` exactly as
///   `embsim_peripherals::pulse_out::PulseOut::run` does, so consumer and
///   firmware agree bit for bit — but both share that truncation, so a
///   sub-microsecond instant is not resolvable.
/// - **A direction split costs up to one pulse of phase.** Re-signing a train
///   mid-flight re-anchors it at the change instant, which discards the
///   source's pulse phase (integer microseconds cannot carry it). The count
///   handed over at the split is exact; from there the re-anchored segment can
///   trail the peripheral by at most one pulse, once per reversal. A rate
///   change costs nothing extra — the peripheral re-anchors there anyway, so
///   both sides truncate identically.
/// - **Delivery is gated at rate-change granularity.** The engine checks that
///   the route's nets are signal-capable when it delivers a train; a net that
///   falls into `Contention`/`Floating` *mid-train* does not interrupt a train
///   already in flight — the next rate change is what notices. Scenarios that
///   break a step net should assert on the resulting finding rather than on
///   the carriage stopping.
/// - **One train per source.** A source publishes its whole channel state each
///   time; there is no per-pulse ordering against other traffic on the net.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PulseTrain {
    /// Rate, accumulated count, ceiling and start instant — the same segment
    /// vocabulary the pulse-out peripheral publishes.
    pub pulses: PulseSegment,
    /// Which way these pulses move a downstream counter.
    pub direction: PulseDirection,
}

impl PulseTrain {
    /// A held channel: no rate, nothing emitted, forward.
    pub const IDLE: Self = Self {
        pulses: PulseSegment::IDLE,
        direction: PulseDirection::Forward,
    };

    /// Cumulative (unsigned) pulses emitted by this train at virtual time
    /// `now_us`.
    pub fn emitted_at(&self, now_us: u64) -> u64 {
        self.pulses.emitted_at(now_us)
    }

    /// **Signed** pulses emitted since this segment began — the amount a
    /// consumer folds into its own position when it re-bases or replaces the
    /// train.
    pub fn delta_at(&self, now_us: u64) -> i64 {
        let delta = self.emitted_at(now_us).saturating_sub(self.pulses.emitted);
        self.direction.sign() * i64::try_from(delta).unwrap_or(i64::MAX)
    }

    /// Virtual time (µs) at which a finite train emits its last pulse, or
    /// `None` for an unbounded or held train.
    pub fn completes_at(&self) -> Option<u64> {
        self.pulses.completes_at()
    }

    /// The same train re-anchored at `at_us` — identical rate, ceiling and
    /// direction, with the count advanced to that instant.
    ///
    /// Idempotent and exact, so folding `delta_at(t)` into an accumulator and
    /// then re-basing at `t` never double-counts a pulse.
    pub fn rebased_at(&self, at_us: u64) -> Self {
        Self {
            pulses: self.pulses.rebased_at(at_us),
            direction: self.direction,
        }
    }

    /// Signed rate in pulses/second (negative when reversing).
    pub fn signed_rate(&self) -> f64 {
        self.direction.sign() as f64 * f64::from(self.pulses.freq_hz)
    }
}

/// The one per-instant message a pin publishes onto its net (`NODES.md`
/// §10/§11): what the pin *is* electrically at the instant it published,
/// sequenced through the engine's drive queue like every other drive. Two
/// encodings exist in this phase; the pulse train ([`PulseTrain`]) folds in
/// as the third when the pulse channel retires.
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
}

/// The drive a pin presents from attach until its component drives it — a
/// static fact declared once on [`PinDecl`], so a model whose output rests
/// released (an open-drain sink, an output nothing drives until the model
/// does) no longer has to release it in `attach`.
///
/// This is the transitional shape of `NODES.md` §10's `idle:
/// Option<Thevenin>`: [`IdleDrive::KindDefault`] keeps the engine's
/// documented per-kind default for every declaration that does not set an
/// idle drive, so no existing pin table moves; a declaration that sets one
/// is honoured on every pin that has a drive slot (digital in/out/bidir and
/// analog). Power and passive pins have no slot and no idle drive.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum IdleDrive {
    /// The kind's documented default: push-pull digital
    /// ([`PinKind::DigitalOut`], [`PinKind::DigitalBidir`]) idles driven
    /// high at its declared impedance; every other kind idles released.
    #[default]
    KindDefault,
    /// Released (high-Z) at attach.
    Released,
    /// A static Thevenin drive at attach.
    Thevenin(TheveninDrive),
}

/// One declared pin of a [`Component`]. The set returned by
/// [`Component::pins`] must cover the component's netlist pins exactly —
/// build validates both directions.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PinDecl {
    /// Netlist pin number (`"3"`).
    pub number: &'static str,
    /// Alias (`"RX"`) — matches KiCad `pinfunction` when present.
    pub name: Option<&'static str>,
    /// Electrical role.
    pub kind: PinKind,
    /// Serial endpoint role, if any.
    pub stream: Option<StreamRole>,
    /// Thevenin source impedance; default per kind
    /// ([`crate::net::DEFAULT_PUSH_PULL_IMPEDANCE`] for push-pull digital).
    pub drive_impedance: Option<Ohms>,
    /// The drive the pin presents from attach until the component drives it
    /// (see [`IdleDrive`]).
    pub idle: IdleDrive,
}

impl PinDecl {
    /// A pin of the given kind with no alias, no stream role, the kind's
    /// default impedance and the kind's default idle drive.
    pub const fn new(number: &'static str, kind: PinKind) -> Self {
        Self {
            number,
            name: None,
            kind,
            stream: None,
            drive_impedance: None,
            idle: IdleDrive::KindDefault,
        }
    }

    /// A pin the component senses and never drives.
    pub const fn digital_in(number: &'static str) -> Self {
        Self::new(number, PinKind::DigitalIn)
    }

    /// A push-pull output pin (idles driven high until the component drives
    /// it, unless [`PinDecl::with_idle`] says otherwise).
    pub const fn digital_out(number: &'static str) -> Self {
        Self::new(number, PinKind::DigitalOut)
    }

    /// A pin whose *voltage* the component needs (participates in the
    /// cluster solve) — a differential receiver input, an ADC input.
    pub const fn analog(number: &'static str) -> Self {
        Self::new(number, PinKind::Analog)
    }

    /// A rail the part consumes.
    pub const fn power_in(number: &'static str) -> Self {
        Self::new(number, PinKind::PowerIn)
    }

    /// A rail the part generates.
    pub const fn power_out(number: &'static str) -> Self {
        Self::new(number, PinKind::PowerOut)
    }

    /// A terminal that contributes nothing electrical of its own.
    pub const fn passive(number: &'static str) -> Self {
        Self::new(number, PinKind::Passive)
    }

    /// The same pin with an alias (`"RX"`).
    pub const fn with_name(mut self, name: &'static str) -> Self {
        self.name = Some(name);
        self
    }

    /// The same pin with a pulse-train role.
    pub const fn with_stream(mut self, stream: StreamRole) -> Self {
        self.stream = Some(stream);
        self
    }

    /// The same pin with a declared Thevenin source impedance.
    pub const fn with_impedance(mut self, ohms: Ohms) -> Self {
        self.drive_impedance = Some(ohms);
        self
    }

    /// The same pin with a declared idle drive.
    pub const fn with_idle(mut self, idle: IdleDrive) -> Self {
        self.idle = idle;
        self
    }
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

/// A pin's declared **reference**: the pin its voltages are measured
/// against (`NODES.md` §11, `PinDecl::reference`). Declared on the
/// component beside its branches until phase 5 rebuilds `PinDecl` around
/// `PinRole`, when it moves onto the pin — a regulator's output against its
/// own ground pin, an isolator's side supply against that side's ground, a
/// supervisor's supply against its `VSS`. The build reads it for two lints
/// and nothing else: a supply pin whose reference net no source reaches
/// while its own does ([`crate::Finding::UnreferencedDomain`]), and a
/// power-in pin with no capacitor to its reference
/// ([`crate::Finding::UndecoupledPowerPin`]); a rail released because its
/// reference is unheld names the reference in
/// [`crate::Finding::RailDown`]. A reference naming a pin the facade does
/// not declare fails the board build, as a branch does.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PinReference {
    /// The pin, as its [`PinDecl`] names it (number or alias).
    pub pin: &'static str,
    /// The pin it is measured against.
    pub reference: &'static str,
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

    /// The pins each of the component's pins is measured against
    /// ([`PinReference`]): a static declaration the build lints read.
    /// Default: none.
    fn references(&self) -> &[PinReference] {
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
    stream: Option<StreamRole>,
    /// The pin is a `DigitalBidir` declared `IdleDrive::Released`: a pad
    /// that is an input until its owner drives it, so a sense subscription
    /// through it declares the net **read** (`Command::DeclareRead`) and a
    /// floating one becomes a reported finding. A `DigitalIn` is a sense
    /// from its declaration; this is the bidirectional pad's equivalent,
    /// made at the subscription that reads it.
    reads_when_released: bool,
    /// The elements this pin terminates, as `(element index, sign)`: `+1`
    /// where the pin is the branch's `a` (the branch current flows from the
    /// net into the pin), `−1` where it is `b`. Summed with the pin's own
    /// drive current by [`PinHandle::sense_current`].
    branch_terms: Vec<(usize, f64)>,
    /// A `PowerOut` pin: its slot is its terminal's — what it publishes is
    /// what the rail holds — and its current spans clusters, so it is no
    /// instrument.
    terminal: bool,
    link: EngineLink,
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
            stream: None,
            reads_when_released: false,
            branch_terms: Vec::new(),
            terminal: false,
            link: EngineLink::default(),
        }
    }

    /// Create a wired handle (system-build internal use). `endpoint` is
    /// `None` for pins that cannot drive (power/passive/detached pins);
    /// `stream` carries the pin's declared serial role for the stream I/O
    /// surface.
    pub(crate) fn wired(
        net: NetId,
        endpoint: Option<EndpointId>,
        stream: Option<StreamRole>,
        link: EngineLink,
    ) -> Self {
        Self {
            net,
            endpoint,
            stream,
            reads_when_released: false,
            branch_terms: Vec::new(),
            terminal: false,
            link,
        }
    }

    /// Mark the handle as a `PowerOut` pin's (system-build internal use;
    /// see the field).
    pub(crate) fn on_terminal(mut self, terminal: bool) -> Self {
        self.terminal = terminal;
        self
    }

    /// Mark the handle as a released bidirectional pad's: a sense
    /// subscription through it declares the net read (system-build internal
    /// use; see the field).
    pub(crate) fn reading_when_released(mut self, reads: bool) -> Self {
        self.reads_when_released = reads;
        self
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

    /// Read the current resolved state of the attached net: the live
    /// engine's most recent publication, or the build-time snapshot on the
    /// analysis path. Identity-only handles (no state table) report
    /// [`NetState::Floating`].
    pub fn sense(&self) -> NetState {
        self.link
            .states
            .lock()
            .unwrap()
            .get(self.net.0)
            .copied()
            .unwrap_or(NetState::Floating)
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

    /// [`Self::drive`] for the Thevenin encoding, `None` releasing —
    /// the form every existing model publishes in; a thin alias kept until
    /// the last caller moves to [`Self::drive`].
    pub fn set_drive(&self, drive: Option<TheveninDrive>) {
        self.publish(drive.map(Drive::Thevenin));
    }

    fn publish(&self, drive: Option<Drive>) {
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

// ============================================================
// Pulse write handle
// ============================================================

/// Write half of a [`StreamRole::PulseSource`] pin (a step clock), obtained
/// via [`ComponentNetIo::pulse_tx`].
///
/// [`PulseTx::set_train`] publishes a whole constant-rate segment; call it
/// **only when the rate, direction or ceiling changes** — that is the entire
/// point of the representation (see [`StreamRole::PulseSource`]). Publishing
/// is non-blocking: the train is enqueued to the engine thread and delivered
/// to every routed [`StreamRole::PulseSink`] with no lock held.
///
/// Cloneable and thread-safe, exactly like [`PinHandle`].
#[derive(Debug, Clone)]
pub struct PulseTx {
    endpoint: Option<EndpointId>,
    link: EngineLink,
}

impl PulseTx {
    /// Publish a new constant-rate segment on this pin.
    ///
    /// A train written into a pin with no drive endpoint (detached), or on the
    /// inert build-time path, is traced and dropped.
    pub fn set_train(&self, train: PulseTrain) {
        let Some(endpoint) = self.endpoint else {
            tracing::debug!("pulse train on a pin without a drive endpoint dropped");
            return;
        };
        self.link.send(Command::PulseUpdate { endpoint, train });
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
        }
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

    /// Subscribe to state changes of the net behind a pin. The callback runs
    /// on the engine thread with **no engine lock held**; the current state
    /// is delivered once at registration (so a floating net is reported
    /// before any traffic), then on every change. A callback MAY drive a
    /// pin — the drive is enqueued and resolved in a later engine iteration.
    ///
    /// Through a released bidirectional pad (`DigitalBidir` declared
    /// `IdleDrive::Released`) the subscription is also the declaration that
    /// the pad **reads** its net: the net joins the digital senses, and a
    /// floating one is reported as [`crate::Finding::FloatingSense`] — on
    /// the build path and the live path alike. A pad nothing subscribes to
    /// is read by nothing and floats without a finding.
    pub fn on_sense(
        &self,
        id: &str,
        callback: impl Fn(NetState) + Send + 'static,
    ) -> Result<(), AttachError> {
        let handle = self.pin(id)?;
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
            callback(handle.sense());
            // The build holds the log's one strong reference; a dead weak
            // (a handle used after the build returned) records nothing.
            if let Some(log) = self.link.recorded_senses.as_ref().and_then(Weak::upgrade) {
                log.lock()
                    .expect("sense log never poisoned")
                    .push(crate::engine::RecordedSense {
                        net: handle.net(),
                        reads: handle.reads_when_released,
                        callback: crate::engine::RecordedCallback::State(Box::new(callback)),
                    });
            }
            return Ok(());
        }
        if handle.reads_when_released {
            self.link.send(Command::DeclareRead {
                net: handle.net(),
                kind: ReadKind::Digital,
            });
        }
        self.link.send(Command::RegisterSense {
            net: handle.net(),
            callback: Box::new(callback),
        });
        Ok(())
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

    /// Write half of a [`StreamRole::PulseSource`] pin (a step clock). Fails
    /// loudly when the pin was not declared a pulse source — a component
    /// asking to pulse on a non-source pin is a facade bug, caught at attach.
    pub fn pulse_tx(&self, id: &str) -> Result<PulseTx, AttachError> {
        let pin = self.pin(id)?;
        match pin.stream {
            Some(StreamRole::PulseSource) => Ok(PulseTx {
                endpoint: pin.endpoint,
                link: pin.link,
            }),
            _ => Err(AttachError::Failed {
                message: format!("pin {id:?} is not a pulse source"),
            }),
        }
    }

    /// Subscribe to the pulse train routed to a [`StreamRole::PulseSink`] pin
    /// (a step/direction drive's STEP input). The callback runs on the engine
    /// thread with **no engine lock held**, once per *rate change* — never per
    /// pulse; between deliveries the subscriber integrates the train itself
    /// (see [`PulseTrain`]).
    ///
    /// A routed source that already has a train delivers it once at
    /// registration, mirroring [`ComponentNetIo::on_sense`]. Fails loudly when
    /// the pin was not declared a pulse sink. A detached sink pin registers
    /// nothing (its route never forms), which is not an attach failure.
    pub fn on_pulse(
        &self,
        id: &str,
        callback: impl Fn(PulseTrain) + Send + 'static,
    ) -> Result<(), AttachError> {
        let pin = self.pin(id)?;
        match pin.stream {
            Some(StreamRole::PulseSink) => {
                let Some(endpoint) = pin.endpoint else {
                    tracing::debug!(
                        pin = id,
                        "on_pulse on a pin without a drive endpoint dropped"
                    );
                    return Ok(());
                };
                self.link.send(Command::RegisterPulseSink {
                    endpoint,
                    callback: Box::new(callback),
                });
                Ok(())
            }
            _ => Err(AttachError::Failed {
                message: format!("pin {id:?} is not a pulse sink"),
            }),
        }
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
        let link = EngineLink::inert(
            states,
            Arc::new(Mutex::new(crate::engine::CurrentTable::default())),
            Arc::new(Mutex::new(Vec::new())),
            &crate::engine::SenseLog::default(),
        );
        let handle = PinHandle::wired(NetId(0), None, None, link.clone());
        let io = ComponentNetIo::wired([("1".to_string(), handle)], None, link);

        let log: Arc<Mutex<Vec<NetState>>> = Arc::new(Mutex::new(Vec::new()));
        let sink = Arc::clone(&log);
        io.on_sense("1", move |state| sink.lock().unwrap().push(state))
            .unwrap();
        assert_eq!(*log.lock().unwrap(), vec![NetState::Driven(Level::High)]);
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
