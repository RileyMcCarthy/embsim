//! The serial byte↔level bridge: a UART that actually puts its bits on a net.
//!
//! A [`crate::StreamRole::Producer`] pin routes a UART byte as a *byte*. The
//! net decides who is connected, but the payload never becomes a level — so it
//! cannot experience contention, cannot be corrupted by a fighting driver, and
//! cannot notice the line was floating. This module is the other path: the
//! byte becomes ten timed edges on a plain [`crate::PinKind::DigitalOut`], and
//! the peer reads it back off a plain [`crate::PinKind::DigitalIn`].
//!
//! **One implementation, shared.** [`crate::mcu::McuComponent`] uses it for a
//! bridged firmware channel; `embsim_models::ads122u04_component` uses it for
//! the chip's UART; a test probe uses it to be the other end of either. Bit
//! order and framing are exactly the details that are cheapest to get subtly
//! wrong in a second copy, so there is only one.
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
//! # Output enable
//!
//! [`SerialLevelBridge::set_output_enabled`] releases the TX pin to high-Z and
//! drops whatever was queued. That is what an unpowered or held-in-reset part
//! actually does — it stops driving, rather than politely discarding bytes
//! behind a still-driven line — and it is expressible only because the payload
//! is on the net now.
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
use crate::net::{digital_drive, level_of, Level, NetState};
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
    /// Virtual instant the next bit is driven at; `None` until the engine
    /// anchors the frame (see [`SerialLevelBridge::transmit`]).
    next_edge_ns: Option<u64>,
}

impl TxState {
    /// Nothing queued, in flight, or armed.
    fn is_idle(&self) -> bool {
        self.next_edge_ns.is_none() && self.bits.is_empty() && self.pending.is_empty()
    }
}

/// One serial channel carried as levels rather than as bytes.
///
/// The owner supplies the time: call [`Self::receive_sense`] from the RX pin's
/// sense callback, [`Self::transmit`] whenever it has bytes to send, and
/// [`Self::service`] from its wake handler at whatever instant the bridge last
/// asked for.
#[derive(Debug)]
pub struct SerialLevelBridge {
    framing: UartFraming,
    encoder: UartEncoder,
    tx_pin: PinHandle,
    tx: Mutex<TxState>,
    rx: Mutex<UartDecoder>,
    /// Scheduling handle; both directions arm wakes through it.
    io: ComponentNetIo,
    /// Whether the output driver is powered. Cleared, the pin goes high-Z.
    output_enabled: AtomicBool,
    shutdown: Arc<AtomicBool>,
}

impl SerialLevelBridge {
    /// Build a bridge over `tx_pin`, framing at `framing`.
    ///
    /// `shutdown` is the owner's own teardown flag: once set the bridge accepts
    /// nothing further, so a callback that outlives its component is inert
    /// rather than driving a dead engine.
    pub fn new(
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
            output_enabled: AtomicBool::new(true),
            shutdown,
        }
    }

    /// The framing this bridge encodes and decodes with.
    pub fn framing(&self) -> UartFraming {
        self.framing
    }

    /// Hold the line at its idle level.
    ///
    /// Call once the owner is ready to be seen: an asynchronous line that has
    /// never transmitted still drives, and a peer that saw it floating would
    /// have no reference against which the first start bit is a falling edge.
    /// A bridge whose output is disabled stays high-Z instead.
    pub fn idle(&self) {
        if self.output_enabled.load(Ordering::Relaxed) {
            self.tx_pin
                .set_drive(Some(digital_drive(self.encoder.idle_level())));
        } else {
            self.tx_pin.set_drive(None);
        }
    }

    /// Power the output driver on or off.
    ///
    /// Disabling releases the pin to high-Z and **drops** the queue and any
    /// frame in flight: a part that loses power mid-byte does not resume that
    /// byte when it comes back, and its output stops driving rather than
    /// continuing to hold the line while quietly discarding data. Enabling
    /// parks the line back at idle.
    pub fn set_output_enabled(&self, enabled: bool) {
        if self.output_enabled.swap(enabled, Ordering::Relaxed) == enabled {
            return;
        }
        if !enabled {
            let mut tx = self.tx.lock().expect("tx state never poisoned");
            tx.pending.clear();
            tx.bits.clear();
            tx.next_edge_ns = None;
        }
        self.idle();
    }

    /// Accept bytes for transmission. Returns how many were shed — because the
    /// queue was full, or because the output driver is off.
    ///
    /// Safe to call from any thread, and deliberately does **not** stamp the
    /// first bit with the clock it reads here. A caller on another thread —
    /// the MCU's pump, a test body — reads a virtual instant that the engine
    /// may have run far past by the time the request crosses the channel, and
    /// anchoring the frame there makes every bit of it "late": the whole byte
    /// then clocks out at one instant and arrives as garbage. The engine
    /// anchors the frame when it actually starts it; this only asks it to.
    pub fn transmit(&self, bytes: &[u8]) -> usize {
        if self.shutdown.load(Ordering::Relaxed) || !self.output_enabled.load(Ordering::Relaxed) {
            return bytes.len();
        }
        let (shed, kick) = {
            let mut tx = self.tx.lock().expect("tx state never poisoned");
            let idle = tx.is_idle();
            let room = TX_QUEUE_MAX.saturating_sub(tx.pending.len());
            let accepted = bytes.len().min(room);
            tx.pending.extend(&bytes[..accepted]);
            // A busy line is already armed and picks these up when it drains
            // the frame it is sending; an idle one needs a wake to get going.
            (bytes.len() - accepted, idle && accepted > 0)
        };
        if kick {
            self.io.schedule_at_ns(virtual_clock::virtual_ns());
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
    pub fn receive_sense(&self, state: NetState) -> Vec<Result<u8, FramingError>> {
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
    pub fn service(&self, now_ns: u64) -> (Vec<Result<u8, FramingError>>, Option<u64>) {
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
        if !self.output_enabled.load(Ordering::Relaxed) {
            return None;
        }
        let mut tx = self.tx.lock().expect("tx state never poisoned");
        let due = match tx.next_edge_ns {
            Some(due) if due > now_ns => return Some(due), // this bit is not due yet
            Some(due) => due,
            // Anchor a new frame on the engine's clock. Every later bit is
            // `due + bit_period` and is armed on the wheel, which the engine
            // cannot advance past — so the grid is exact from here on.
            None if !tx.pending.is_empty() => now_ns,
            None => return None,
        };
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
        self.tx_pin.set_drive(Some(digital_drive(level)));
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
