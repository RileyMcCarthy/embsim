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

use std::collections::HashMap;
use std::fmt;

use crate::engine::{Command, ComponentId, EndpointId, EngineLink};
use crate::net::{NetId, NetState, Ohms, TheveninDrive};

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

/// Serial-stream role of a pin. The pin's [`PinKind`] stays digital; byte
/// pipes are derived from and gated by net resolution, never installed
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

    /// Register this component's wakeup handler for
    /// [`schedule_at`](Self::schedule_at) /
    /// [`schedule_every`](Self::schedule_every) deliveries (last
    /// registration wins). The callback runs on the engine thread with the
    /// sampled virtual time (µs) and no engine lock held.
    pub fn on_wake(&self, callback: impl Fn(u64) + Send + 'static) {
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
    /// served by the engine thread's timer wheel. Timestamps are sampled
    /// from the free-running scaled clock — a deadline already in the past
    /// fires immediately, in deadline order. Requires `virtual_clock::init`.
    pub fn schedule_at(&self, at_us: u64) {
        let Some(component) = self.component else {
            tracing::debug!("schedule_at on an inert io handle dropped");
            return;
        };
        self.link.send(Command::ScheduleAt { component, at_us });
    }

    /// Request a periodic wakeup every `period_us` of virtual time. Missed
    /// deadlines coalesce (one catch-up fire, then back on period) — compute
    /// time-dependent state at read time, never per tick. Requires
    /// `virtual_clock::init`.
    pub fn schedule_every(&self, period_us: u64) {
        let Some(component) = self.component else {
            tracing::debug!("schedule_every on an inert io handle dropped");
            return;
        };
        self.link.send(Command::ScheduleEvery {
            component,
            period_us,
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
    use super::*;

    #[test]
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
    #[test]
    fn on_sense_inert_link_delivers_the_snapshot_once_synchronously() {
        use crate::engine::EngineLink;
        use crate::net::Level;
        use std::sync::{Arc, Mutex};

        let states = Arc::new(Mutex::new(vec![NetState::Driven(Level::High)]));
        let link = EngineLink::inert(states);
        let handle = PinHandle::wired(NetId(0), None, None, link.clone());
        let io = ComponentNetIo::wired([("1".to_string(), handle)], None, link);

        let log: Arc<Mutex<Vec<NetState>>> = Arc::new(Mutex::new(Vec::new()));
        let sink = Arc::clone(&log);
        io.on_sense("1", move |state| sink.lock().unwrap().push(state))
            .unwrap();
        assert_eq!(*log.lock().unwrap(), vec![NetState::Driven(Level::High)]);
    }

    #[test]
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
