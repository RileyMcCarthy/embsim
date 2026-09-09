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
//! Serial-capable pins additionally get a **stream I/O surface**
//! ([`ComponentNetIo::stream_tx`] / [`ComponentNetIo::on_byte`]) whose byte
//! pipes are derived from and gated by net resolution — see the stream
//! section of [`crate::engine`].
//!
//! Pulse-capable pins get the analogous **pulse I/O surface**
//! ([`ComponentNetIo::pulse_tx`] / [`ComponentNetIo::on_pulse`]), routed the
//! same way but carrying a *rate* ([`PulseTrain`]) rather than bytes — see
//! [`StreamRole::PulseSource`] for why a step clock is not modeled as edges.

use std::collections::HashMap;
use std::fmt;

use crate::engine::{Command, ComponentId, EndpointId, EngineLink};
use crate::net::{NetId, NetState, Ohms, TheveninDrive};

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

/// Channel role of a pin: a **byte stream** (UART) or a **pulse train** (a
/// step clock). The pin's [`PinKind`] stays digital in both cases; the
/// channel is derived from and gated by net resolution, never installed
/// beside it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum StreamRole {
    /// Transmits bytes onto the net (UART TX; idles `Driven(High)`).
    Producer {
        /// Byte pacing rate.
        baud_hz: u32,
    },
    /// Receives bytes routed from a reachable producer (UART RX).
    Consumer {
        /// Byte pacing rate.
        baud_hz: u32,
    },
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
            link,
        }
    }

    /// The net this pin is attached to.
    pub fn net(&self) -> NetId {
        self.net
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

    /// Enqueue a new drive for this pin (`None` releases to high-Z). Drives
    /// are enqueued, never applied inline — the engine thread serializes
    /// them by enqueue sequence and resolves each in a later iteration, so
    /// calling this from a sense callback is safe by construction.
    ///
    /// On the inert build-time path (or for a pin without a drive slot) the
    /// drive is traced and dropped.
    pub fn set_drive(&self, drive: Option<TheveninDrive>) {
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
// Stream write handle
// ============================================================

/// Write half of a stream producer pin (UART TX), obtained via
/// [`ComponentNetIo::stream_tx`].
///
/// Bytes written here flow on the route **derived from net resolution** —
/// through the producer's net and any collapsed series passives — paced at
/// the producer's declared baud against virtual time. Writes never block:
/// bytes are enqueued to the engine thread, and bytes written into a broken
/// route (no routed consumer topology, an inert build-time handle, or a
/// link whose nets resolve `Contention`/`Floating`) are dropped with a
/// trace, never queued forever.
///
/// Cloneable and thread-safe: components hand clones to their protocol
/// threads, exactly like [`PinHandle`].
#[derive(Debug, Clone)]
pub struct StreamTx {
    endpoint: Option<EndpointId>,
    link: EngineLink,
}

impl StreamTx {
    /// Enqueue bytes onto the producer's derived route, in wire order.
    pub fn write(&self, bytes: &[u8]) {
        let Some(endpoint) = self.endpoint else {
            tracing::debug!("stream write on a pin without a drive endpoint dropped");
            return;
        };
        if bytes.is_empty() {
            return;
        }
        self.link.send(Command::StreamWrite {
            endpoint,
            bytes: bytes.to_vec(),
        });
    }
}

/// Write half of a [`StreamRole::PulseSource`] pin (a step clock), obtained
/// via [`ComponentNetIo::pulse_tx`].
///
/// [`PulseTx::set_train`] publishes a whole constant-rate segment; call it
/// **only when the rate, direction or ceiling changes** — that is the entire
/// point of the representation (see [`StreamRole::PulseSource`]). Publishing
/// is non-blocking: the train is enqueued to the engine thread and delivered
/// to every routed [`StreamRole::PulseSink`] with no lock held.
///
/// Cloneable and thread-safe, exactly like [`StreamTx`].
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
        }
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
            // divergence the shared resolver exists to prevent). Later
            // deliveries never happen on this path: the snapshot is final.
            callback(handle.sense());
            return Ok(());
        }
        self.link.send(Command::RegisterSense {
            net: handle.net(),
            callback: Box::new(callback),
        });
        Ok(())
    }

    /// Write half of a stream producer pin (UART TX). Fails loudly when the
    /// pin was not declared [`StreamRole::Producer`] — a component asking to
    /// transmit on a non-producer pin is a facade bug, caught at attach.
    pub fn stream_tx(&self, id: &str) -> Result<StreamTx, AttachError> {
        let pin = self.pin(id)?;
        match pin.stream {
            Some(StreamRole::Producer { .. }) => Ok(StreamTx {
                endpoint: pin.endpoint,
                link: pin.link,
            }),
            _ => Err(AttachError::Failed {
                message: format!("pin {id:?} is not a stream producer"),
            }),
        }
    }

    /// Subscribe to bytes routed to a stream consumer pin (UART RX). The
    /// callback runs on the engine thread with **no engine lock held**, one
    /// call per delivered byte, paced at the routed producer's declared
    /// baud. Fails loudly when the pin was not declared
    /// [`StreamRole::Consumer`]. A detached consumer pin registers nothing
    /// (its route never forms), which is not an attach failure.
    pub fn on_byte(
        &self,
        id: &str,
        callback: impl Fn(u8) + Send + 'static,
    ) -> Result<(), AttachError> {
        let pin = self.pin(id)?;
        match pin.stream {
            Some(StreamRole::Consumer { .. }) => {
                let Some(endpoint) = pin.endpoint else {
                    tracing::debug!(
                        pin = id,
                        "on_byte on a pin without a drive endpoint dropped"
                    );
                    return Ok(());
                };
                self.link.send(Command::RegisterStreamConsumer {
                    endpoint,
                    callback: Box::new(callback),
                });
                Ok(())
            }
            _ => Err(AttachError::Failed {
                message: format!("pin {id:?} is not a stream consumer"),
            }),
        }
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
        self.link.send(Command::RegisterWake {
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
        self.link.send(Command::ScheduleAt { component, at_ns });
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
        self.link.send(Command::ScheduleEvery {
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
        let link = EngineLink::inert(states, Arc::new(Mutex::new(Vec::new())));
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
