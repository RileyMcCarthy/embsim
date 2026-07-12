//! Component trait, pin declarations, and the per-component net I/O handle.
//!
//! Every part on a board — including the MCU — is a [`Component`] with
//! declared pins. At build time the board validates each component's
//! [`PinDecl`] facade against the netlist in BOTH directions and calls
//! [`Component::attach`] with a [`ComponentNetIo`] so the component can grab
//! typed pin handles **before it is shared** (pre-`Arc`, no interior
//! mutability needed) and fail loudly on facade mismatch.

use std::collections::HashMap;
use std::fmt;

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
/// This slice carries net identity only; `sense`/`drive` are thin
/// placeholders until the live net engine lands.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PinHandle {
    net: NetId,
}

impl PinHandle {
    /// Create a handle bound to a resolved net (board-build internal use).
    pub fn new(net: NetId) -> Self {
        Self { net }
    }

    /// The net this pin is attached to.
    pub fn net(&self) -> NetId {
        self.net
    }

    /// Read the current resolved state of the attached net.
    ///
    /// TODO(board-engine): served by the live net engine; not available in
    /// the build-time analysis slice.
    pub fn sense(&self) -> NetState {
        todo!("live net engine: sense delivery is a later slice")
    }

    /// Enqueue a new drive for this pin (`None` releases to high-Z). Drives
    /// are enqueued, never applied inline — the engine thread serializes and
    /// resolves them in a later iteration.
    ///
    /// TODO(board-engine): served by the live net engine (MPSC drive queue);
    /// not available in the build-time analysis slice.
    pub fn set_drive(&self, _drive: Option<TheveninDrive>) {
        todo!("live net engine: drive queue is a later slice")
    }
}

/// Per-component net I/O passed to [`Component::attach`]: typed pin-handle
/// lookup plus (later) engine-owned scheduling.
#[derive(Debug, Clone, Default)]
pub struct ComponentNetIo {
    /// Keyed by BOTH the netlist pin number and the declared pin name (when
    /// present), so `io.pin("3")` and `io.pin("RX")` resolve identically.
    pins: HashMap<String, PinHandle>,
}

impl ComponentNetIo {
    /// Build the handle table (board-build internal use). Insert each handle
    /// under every identity it answers to (pin number, declared name).
    pub fn from_entries(entries: impl IntoIterator<Item = (String, PinHandle)>) -> Self {
        Self {
            pins: entries.into_iter().collect(),
        }
    }

    /// Look up a pin handle by declared name or netlist pin number.
    pub fn pin(&self, id: &str) -> Result<PinHandle, AttachError> {
        self.pins
            .get(id)
            .copied()
            .ok_or_else(|| AttachError::UnknownPin {
                pin: id.to_string(),
            })
    }

    /// Request a one-shot wakeup at the given virtual time (µs).
    ///
    /// TODO(board-engine): served by the engine thread's timer wheel; not
    /// available in the build-time analysis slice.
    pub fn schedule_at(&self, _at_us: u64) {
        todo!("live net engine: timer wheel is a later slice")
    }

    /// Request a periodic wakeup every `period_us` of virtual time.
    ///
    /// TODO(board-engine): served by the engine thread's timer wheel; not
    /// available in the build-time analysis slice.
    pub fn schedule_every(&self, _period_us: u64) {
        todo!("live net engine: timer wheel is a later slice")
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
        let io =
            ComponentNetIo::from_entries([("3".to_string(), handle), ("RX".to_string(), handle)]);
        assert_eq!(io.pin("3").unwrap().net(), NetId(7));
        assert_eq!(io.pin("RX").unwrap().net(), NetId(7));
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
    }
}
