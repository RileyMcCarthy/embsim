//! The serial byte↔level bridge: a UART that actually puts its bits on a net.
//!
//! [`crate::mcu::McuComponent`] normally hands a firmware TX byte to a
//! [`crate::StreamRole::Producer`] pin, which routes it as a *byte*. The net
//! decides who is connected, but the payload never becomes a level — so it
//! cannot experience contention, cannot be corrupted by a fighting driver, and
//! cannot notice the line was floating. This module is the other path: the
//! byte becomes ten timed edges on a plain [`crate::PinKind::DigitalOut`], and
//! the peer reads it back off a plain [`crate::PinKind::DigitalIn`].
//!
//! # Why it needs nanoseconds
//!
//! At 2 Mbaud a bit is 500 ns, so eight of them fit inside one microsecond.
//! Everything here schedules on
//! [`ComponentNetIo::schedule_at_ns`] and stamps with
//! [`virtual_clock::virtual_ns`]; the microsecond forms would collapse a whole
//! frame to a single instant.
//!
//! # Why the receiver arms a timer
//!
//! The engine delivers a sense only when the resolved state *changes*, so the
//! receiver cannot be handed one notification per bit — `0xFF` has no
//! transition between its start bit and its stop bit. [`UartDecoder`] recovers
//! bit counts from the interval between transitions, and the missing tail is
//! closed by a wake armed at [`UartDecoder::frame_deadline_ns`].
//!
//! # What is deliberately not modeled
//!
//! The transmitter drives the line and never reads it back, so it does not
//! detect a driver fighting it mid-byte. The net still reports the contention
//! and the *receiver* sees the frame break — which is the fidelity that was
//! missing. TX-side collision detection is a real UART feature, but not one
//! this firmware uses.

use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

use embsim_core::virtual_clock;

use crate::component::{ComponentNetIo, PinHandle};
use crate::mcu::{level_of, output_drive};
use crate::net::{Level, NetState};
use crate::uart::{FramingError, UartDecoder, UartEncoder, UartFraming};

/// How many bytes may queue for transmission before the newest are shed.
///
/// A real UART has a small FIFO and a line that clocks at one fixed rate, so a
/// producer that outruns the wire loses data on hardware too. An unbounded
/// queue would turn an overrun into unbounded memory and a growing lie about
/// when a byte reached the peer. Matches the byte path's
/// `STREAM_ROUTE_QUEUE_MAX` policy: shed the newest, like a full TX FIFO.
const TX_QUEUE_MAX: usize = 4096;

/// The transmitter's mutable state, shared between the pump thread (which
/// enqueues bytes) and the engine thread (which clocks them out).
#[derive(Debug, Default)]
struct TxState {
    /// Bytes accepted but not yet framed.
    pending: VecDeque<u8>,
    /// Levels of the frame being clocked out, oldest first.
    bits: VecDeque<Level>,
    /// Virtual instant the next bit is driven at; `None` when the line is idle.
    next_edge_ns: Option<u64>,
}

/// One serial channel carried as levels rather than as bytes.
#[derive(Debug)]
pub(crate) struct SerialLevelBridge {
    framing: UartFraming,
    encoder: UartEncoder,
    tx_pin: PinHandle,
    tx: Mutex<TxState>,
    rx: Mutex<UartDecoder>,
    /// Scheduling handle; both directions arm wakes through it.
    io: ComponentNetIo,
    shutdown: Arc<AtomicBool>,
}

impl SerialLevelBridge {
    pub(crate) fn new(
        framing: UartFraming,
        tx_pin: PinHandle,
        io: ComponentNetIo,
        shutdown: Arc<AtomicBool>,
    ) -> Self {
        Self {
            framing,
            encoder: UartEncoder::new(framing),
            tx_pin,
            tx: Mutex::new(TxState::default()),
            rx: Mutex::new(UartDecoder::new(framing)),
            io,
            shutdown,
        }
    }

    /// Hold the line at its idle level.
    ///
    /// Called once at attach: an asynchronous line that has never transmitted
    /// still drives, and a peer that saw it floating would have no reference
    /// against which the first start bit is a falling edge.
    pub(crate) fn idle(&self) {
        self.tx_pin
            .set_drive(Some(output_drive(self.encoder.idle_level())));
    }

    /// Accept bytes for transmission (pump thread). Returns how many were
    /// shed because the queue was full.
    pub(crate) fn transmit(&self, bytes: &[u8]) -> usize {
        if self.shutdown.load(Ordering::Relaxed) {
            return bytes.len();
        }
        let (shed, kick_at) = {
            let mut tx = self.tx.lock().expect("tx state never poisoned");
            let room = TX_QUEUE_MAX.saturating_sub(tx.pending.len());
            let accepted = bytes.len().min(room);
            tx.pending.extend(&bytes[..accepted]);
            // An idle line starts clocking now; a busy one is already armed and
            // picks these up when it drains the frame it is already sending.
            let kick_at = if tx.next_edge_ns.is_none() && accepted > 0 {
                let now = virtual_clock::virtual_ns();
                tx.next_edge_ns = Some(now);
                Some(now)
            } else {
                None
            };
            (bytes.len() - accepted, kick_at)
        };
        if let Some(at) = kick_at {
            self.io.schedule_at_ns(at);
        }
        shed
    }

    /// Feed one observed net state to the receiver and arm the frame deadline.
    ///
    /// A state with no logic level ([`NetState::Floating`] /
    /// [`NetState::Contention`]) is deliberately *not* forced to a bit: the
    /// decoder holds its last level, the frame in flight still completes on
    /// its deadline, and it fails its stop-bit check if the line never
    /// recovered. That is what the far end of a contended wire really sees.
    pub(crate) fn receive_sense(&self, state: NetState) -> Vec<Result<u8, FramingError>> {
        let now = virtual_clock::virtual_ns();
        let mut rx = self.rx.lock().expect("rx state never poisoned");
        if let Some(level) = level_of(state) {
            rx.on_level(level, now);
        }
        let out = drain(&mut rx, now);
        let deadline = rx.frame_deadline_ns();
        drop(rx);
        if let Some(at) = deadline {
            self.io.schedule_at_ns(at);
        }
        out
    }

    /// Service both directions at `now_ns` (engine thread, on wake). Returns
    /// the bytes decoded and the next instant this bridge needs a wake at.
    pub(crate) fn service(&self, now_ns: u64) -> (Vec<Result<u8, FramingError>>, Option<u64>) {
        let tx_next = self.service_tx(now_ns);
        let mut rx = self.rx.lock().expect("rx state never poisoned");
        let bytes = drain(&mut rx, now_ns);
        let rx_next = rx.frame_deadline_ns();
        drop(rx);
        let next = match (tx_next, rx_next) {
            (Some(a), Some(b)) => Some(a.min(b)),
            (a, b) => a.or(b),
        };
        (bytes, next)
    }

    /// Drive whatever bit is due at `now_ns`; returns the next edge instant.
    fn service_tx(&self, now_ns: u64) -> Option<u64> {
        let mut tx = self.tx.lock().expect("tx state never poisoned");
        let due = tx.next_edge_ns?;
        if due > now_ns {
            return Some(due); // this bit is not due yet
        }
        if tx.bits.is_empty() {
            let Some(byte) = tx.pending.pop_front() else {
                // Nothing left to send: park at idle and stop arming.
                tx.next_edge_ns = None;
                drop(tx);
                self.idle();
                return None;
            };
            tx.bits = self
                .encoder
                .encode(byte)
                .into_iter()
                .map(|(level, _)| level)
                .collect();
        }
        let level = tx.bits.pop_front().expect("filled above");
        // Advance on the frame's own grid, not from `now`: a wake delivered
        // late must not stretch the bit period, or the byte arrives as a
        // different byte at the far end.
        let next = due + self.framing.bit_period_ns;
        tx.next_edge_ns = Some(next);
        drop(tx);
        self.tx_pin.set_drive(Some(output_drive(level)));
        Some(next)
    }
}

fn drain(rx: &mut UartDecoder, now_ns: u64) -> Vec<Result<u8, FramingError>> {
    let mut out = Vec::new();
    while let Some(frame) = rx.poll(now_ns) {
        out.push(frame);
    }
    out
}
