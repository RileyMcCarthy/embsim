//! Asynchronous serial framing, so bytes can travel as levels on a net.
//!
//! One shared implementation, so every byte-oriented peripheral frames
//! identically rather than growing its own — and so a byte on a wire is a
//! waveform that can experience contention, a fighting driver, or a floating
//! line, rather than a payload routed past the net that carries it.
//!
//! ```text
//!   idle   start  d0    d1    d2 …            stop   idle
//!   ─────┐       ┌─────┐     ┌───────────────┬────────────
//!        └───────┘     └─────┘
//!        ^ falling edge opens the frame
//! ```
//!
//! # Why the decoder is edge-driven
//!
//! The engine only delivers a sense when the resolved state *changes*, so a
//! decoder cannot be handed one notification per bit: `0xFF` has no transition
//! at all between its start bit and its stop bit. The decoder therefore recovers
//! bit counts from the *interval* between transitions, and needs
//! [`UartDecoder::poll`] to close a frame whose tail is silent.
//!
//! Completed frames are queued rather than returned from `on_level`, so
//! back-to-back bytes decode correctly no matter how often the owner polls.

use std::collections::VecDeque;

use crate::net::Level;

/// Framing parameters for one link. [`UartFraming::new_8n1`] gives 8N1 LSB
/// first — what every UART on the reference machine uses.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UartFraming {
    /// One bit time, in nanoseconds of virtual time.
    pub bit_period_ns: u64,
    /// Data bits per frame.
    pub data_bits: u8,
    /// Stop bits per frame.
    pub stop_bits: u8,
    /// Least-significant bit first (true for standard asynchronous serial).
    pub lsb_first: bool,
}

impl UartFraming {
    /// 8N1 at `baud_hz`, LSB first.
    ///
    /// Panics on a zero baud rate: a link with no rate has no framing, and
    /// substituting one would hide a misconfigured peripheral rather than
    /// report it.
    pub fn new_8n1(baud_hz: u32) -> Self {
        assert!(baud_hz > 0, "a UART link needs a non-zero baud rate");
        Self {
            bit_period_ns: 1_000_000_000 / baud_hz as u64,
            data_bits: 8,
            stop_bits: 1,
            lsb_first: true,
        }
    }

    /// Bits in one frame: start + data + stop.
    pub fn frame_bits(&self) -> u32 {
        1 + self.data_bits as u32 + self.stop_bits as u32
    }

    /// How long a whole frame occupies.
    pub fn frame_ns(&self) -> u64 {
        self.frame_bits() as u64 * self.bit_period_ns
    }

    /// When a receiver may consider the frame complete and start looking for
    /// the next start bit: the **midpoint of the stop bit**.
    ///
    /// Not the end of the frame. A receiver that waits for the full nominal
    /// frame before accepting another start bit cannot talk to a sender that
    /// is even marginally fast — the next start bit arrives *before* the
    /// deadline, gets absorbed into the frame in progress, and the stream
    /// desynchronizes permanently with no way back.
    ///
    /// This is not a hypothetical. A P2 smart pin programmed for 115'200 baud
    /// at `clkfreq = 160 MHz` actually clocks 115'273 — 0.06 % fast, a rate
    /// any real UART accepts without noticing. Against a full-frame deadline
    /// its second byte was swallowed by its first, and the ADS122U04 model
    /// read the protocol's `0x55` sync byte as register data.
    ///
    /// Sampling mid-stop-bit is what real hardware does, and it buys the
    /// standard half-bit of tolerance in both directions.
    pub fn stop_sampled_ns(&self) -> u64 {
        self.frame_ns() - self.bit_period_ns / 2
    }
}

/// A frame that did not decode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FramingError {
    /// A stop bit was low — the line was still asserted when the frame should
    /// have returned to idle. On real hardware this is what a baud mismatch or
    /// a break condition looks like.
    BadStopBit,
}

/// Turns a byte into the levels a line carries.
///
/// Emits `(level, hold_ns)` pairs, one per bit. Adjacent pairs with the same
/// level are deliberately *not* merged: a caller driving a net is deduped by
/// the resolver anyway, and one entry per bit keeps the encoder trivially
/// checkable against the decoder.
#[derive(Debug, Clone, Copy)]
pub struct UartEncoder {
    framing: UartFraming,
}

impl UartEncoder {
    pub fn new(framing: UartFraming) -> Self {
        Self { framing }
    }

    /// The level an asynchronous line rests at between frames.
    pub fn idle_level(&self) -> Level {
        Level::High
    }

    /// One frame's worth of levels, in order.
    pub fn encode(&self, byte: u8) -> Vec<(Level, u64)> {
        let period = self.framing.bit_period_ns;
        let mut out = Vec::with_capacity(self.framing.frame_bits() as usize);
        out.push((Level::Low, period)); // start bit

        for i in 0..self.framing.data_bits {
            let shift = if self.framing.lsb_first {
                i
            } else {
                self.framing.data_bits - 1 - i
            };
            let set = (byte >> shift) & 1 != 0;
            out.push((if set { Level::High } else { Level::Low }, period));
        }

        for _ in 0..self.framing.stop_bits {
            out.push((Level::High, period));
        }
        out
    }
}

/// Reassembles bytes from level transitions.
///
/// Feed every observed transition to [`Self::on_level`] and call [`Self::poll`]
/// as virtual time advances; `poll` is what closes a frame whose final bits
/// carry no transition, and drains frames already completed by an edge.
#[derive(Debug, Clone)]
pub struct UartDecoder {
    framing: UartFraming,
    /// Level the line is currently held at.
    level: Level,
    /// When the in-progress frame's start bit began.
    frame_start_ns: Option<u64>,
    /// Bits recovered so far this frame, oldest first, start bit excluded.
    bits: Vec<bool>,
    /// When the current run of `level` began, for bit accounting.
    since_ns: u64,
    /// Frames finished but not yet handed back.
    done: VecDeque<Result<u8, FramingError>>,
}

impl UartDecoder {
    pub fn new(framing: UartFraming) -> Self {
        Self {
            framing,
            level: Level::High,
            frame_start_ns: None,
            bits: Vec::new(),
            since_ns: 0,
            done: VecDeque::new(),
        }
    }

    /// The level the decoder holds the line at: the last level it was fed,
    /// idle high before any — what a receiver projecting its next sense
    /// holds as its last level.
    pub fn level(&self) -> Level {
        self.level
    }

    /// Record a level transition observed at `at_ns` of virtual time.
    ///
    /// A falling edge on an idle line opens a frame. A transition arriving
    /// after the current frame's last bit closes that frame first, so
    /// back-to-back bytes do not depend on the caller's poll cadence.
    pub fn on_level(&mut self, level: Level, at_ns: u64) {
        if level == self.level {
            return;
        }
        if let Some(start) = self.frame_start_ns {
            // Mid-stop-bit, not end-of-frame: see `stop_sampled_ns`.
            if at_ns >= start + self.framing.stop_sampled_ns() {
                self.close_frame(start);
            } else {
                self.absorb_until(at_ns);
                self.level = level;
                self.since_ns = at_ns;
                return;
            }
        }
        // Line is idle, so only a falling edge means anything.
        self.level = level;
        self.since_ns = at_ns;
        if level == Level::Low {
            self.frame_start_ns = Some(at_ns);
            self.bits.clear();
            // The start bit is accounted for here; `bits` holds data and stop.
            self.since_ns = at_ns + self.framing.bit_period_ns;
        }
    }

    /// The instant the in-progress frame completes, if one is open.
    ///
    /// An owner that only hears about *transitions* has to arm a timer for
    /// this, or a frame whose tail is silent never closes — `0xFF` ends with
    /// its stop bit at the same level as its last data bit, so nothing more
    /// arrives to prompt a [`Self::poll`].
    pub fn frame_deadline_ns(&self) -> Option<u64> {
        self.frame_start_ns
            .map(|start| start + self.framing.frame_ns())
    }

    /// Whether a decoded frame is waiting, without advancing time.
    pub fn has_pending(&self) -> bool {
        !self.done.is_empty()
    }

    /// Advance virtual time and take the next decoded frame, if any.
    pub fn poll(&mut self, now_ns: u64) -> Option<Result<u8, FramingError>> {
        if let Some(start) = self.frame_start_ns {
            // The idle-tail case: no transition is coming, so wait out the
            // whole frame rather than closing half a bit early.
            if now_ns.saturating_sub(start) >= self.framing.frame_ns() {
                self.close_frame(start);
            }
        }
        self.done.pop_front()
    }

    /// Finish the frame that began at `start` and queue its result.
    fn close_frame(&mut self, start: u64) {
        self.absorb_until(start + self.framing.frame_ns());
        let outcome = self.assemble();
        self.done.push_back(outcome);
        self.frame_start_ns = None;
        self.bits.clear();
        // `level` is deliberately left alone: after a break the line really is
        // still low, and the decoder must wait for it to return to idle before
        // it can see another start bit.
    }

    /// Extend the current run of `self.level` up to `t`, recording whole bits.
    ///
    /// Bit counts come from the interval between transitions rather than from
    /// one notification per bit, because the engine only reports changes.
    fn absorb_until(&mut self, t: u64) {
        let period = self.framing.bit_period_ns.max(1);
        let held = t.saturating_sub(self.since_ns);
        // Round to nearest, so a transition landing a fraction early or late
        // still resolves to the bit count the sender intended.
        let count = (held + period / 2) / period;
        let want = self.framing.frame_bits() as u64 - 1; // start bit is implicit
        let high = self.level == Level::High;
        for _ in 0..count {
            if self.bits.len() as u64 >= want {
                break;
            }
            self.bits.push(high);
        }
        self.since_ns = t;
    }

    /// Assemble the recovered bits into a byte.
    fn assemble(&self) -> Result<u8, FramingError> {
        let data_bits = self.framing.data_bits as usize;
        let mut byte = 0u8;
        for (i, &set) in self.bits.iter().take(data_bits).enumerate() {
            if set {
                let shift = if self.framing.lsb_first {
                    i
                } else {
                    data_bits - 1 - i
                };
                byte |= 1 << shift;
            }
        }
        // Every stop bit must be high. A low one is a real framing error, and
        // reporting it is the whole point of putting bits on the net.
        let stops_ok = self
            .bits
            .iter()
            .skip(data_bits)
            .take(self.framing.stop_bits as usize)
            .all(|&set| set);

        if stops_ok {
            Ok(byte)
        } else {
            // The owner drops the frame, and the driver waiting for that byte
            // sees a *timeout* rather than a corruption — so without the bits
            // here there is nothing to diagnose from. This only fires on a
            // frame that is already lost.
            tracing::warn!(
                bits = ?self.bits,
                want_bits = self.framing.frame_bits() - 1,
                bit_period_ns = self.framing.bit_period_ns,
                "uart: stop bit low, frame dropped"
            );
            Err(FramingError::BadStopBit)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use vibes_behaviour::{behaviour, expect, Test};

    /// Play an encoded byte into a decoder, returning the time it ended at.
    fn play(dec: &mut UartDecoder, framing: UartFraming, byte: u8, from: u64) -> u64 {
        let mut t = from;
        for (level, hold) in UartEncoder::new(framing).encode(byte) {
            dec.on_level(level, t);
            t += hold;
        }
        t
    }

    /// A sender a fraction fast, talking to a receiver at the nominal rate,
    /// with no gap between bytes.
    ///
    /// This is the real case: a P2 smart pin asked for 115'200 baud at
    /// `clkfreq = 160 MHz` clocks **115'273**. Integer bit periods make the
    /// sender's frame 86'750 ns against the receiver's 86'800 — so the second
    /// byte's start bit lands 50 ns *before* a full-frame deadline would
    /// expire. A receiver that waits that long swallows it, and never
    /// resynchronizes: the ADS122U04 model read the `0x55` sync byte as
    /// register data and the force gauge never came up.
    #[test]
    fn a_marginally_fast_sender_still_frames_back_to_back_bytes() {
        behaviour!(Test {
            id: "uart.fast-sender-burst",
            covers: Some("board/src/uart.rs#UartDecoder::on_level"),
            given: "a sender a fraction fast streams bytes back to back, with no idle gap, into a receiver at the nominal rate",
        });
        expect!(
            "all-bytes-decode",
            "every byte of the burst decodes, in the order sent",
            "a sender's clock is never exactly nominal; a real receiver accepts the next start bit from the midpoint of the stop bit, which grants half a bit of tolerance either way",
        );
        let fast = UartFraming::new_8n1(115_273);
        let nominal = UartFraming::new_8n1(115_200);
        assert!(
            fast.frame_ns() < nominal.frame_ns(),
            "the premise: the sender's frame is shorter ({} < {})",
            fast.frame_ns(),
            nominal.frame_ns()
        );

        let mut dec = UartDecoder::new(nominal);
        let mut t = 1_000_000;
        // Back to back, exactly as a burst arrives — no idle between them.
        for byte in [0x55u8, 0x40, 0x0E] {
            t = play(&mut dec, fast, byte, t);
        }
        let mut got = Vec::new();
        while let Some(frame) = dec.poll(t + nominal.frame_ns()) {
            got.push(frame.expect("every byte must frame cleanly"));
        }
        assert_eq!(
            got,
            vec![0x55, 0x40, 0x0E],
            "a 0.06% fast sender is well inside any real UART's tolerance"
        );
    }

    /// The tolerance is symmetric: a marginally *slow* sender must also frame.
    #[test]
    fn a_marginally_slow_sender_still_frames_back_to_back_bytes() {
        behaviour!(Test {
            id: "uart.slow-sender-burst",
            covers: Some("board/src/uart.rs#UartDecoder::on_level"),
            given: "a sender a fraction slow streams bytes back to back, with no idle gap, into a receiver at the nominal rate",
        });
        expect!(
            "all-bytes-decode",
            "every byte of the burst decodes, in the order sent",
            "the receiver's half-bit tolerance holds in both directions",
        );
        let slow = UartFraming::new_8n1(115_000);
        let nominal = UartFraming::new_8n1(115_200);
        let mut dec = UartDecoder::new(nominal);
        let mut t = 1_000_000;
        for byte in [0xA5u8, 0x3C] {
            t = play(&mut dec, slow, byte, t);
        }
        let mut got = Vec::new();
        while let Some(frame) = dec.poll(t + nominal.frame_ns()) {
            got.push(frame.expect("every byte must frame cleanly"));
        }
        assert_eq!(got, vec![0xA5, 0x3C]);
    }

    fn round_trip(framing: UartFraming, byte: u8) -> Option<Result<u8, FramingError>> {
        let mut dec = UartDecoder::new(framing);
        // Start well after zero, as virtual time would.
        let end = play(&mut dec, framing, byte, 1_000_000);
        dec.poll(end)
    }

    #[test]
    fn every_byte_round_trips() {
        behaviour!(Test {
            id: "uart.every-byte-round-trips",
            covers: Some("board/src/uart.rs#UartDecoder::poll"),
            given: "each of the 256 byte values, sent alone on an otherwise idle line",
        });
        expect!(
            "value-survives",
            "the receiver reads back the value that was sent"
        );
        let framing = UartFraming::new_8n1(2_000_000);
        for byte in 0u8..=255 {
            assert_eq!(
                round_trip(framing, byte),
                Some(Ok(byte)),
                "byte {byte:#04X} did not survive the round trip"
            );
        }
    }

    /// The cases an edge-driven decoder gets wrong: `0x00` has no transition
    /// between the start bit and the data, `0xFF` none between the data and
    /// the stop bit.
    #[test]
    fn transition_free_frames_still_decode() {
        behaviour!(Test {
            id: "uart.transition-free-frames",
            covers: Some("board/src/uart.rs#UartDecoder::absorb_until"),
            given: "a byte with no transition among its data bits: all zeros, or all ones",
        });
        expect!(
            "all-zeros",
            "the all-zeros byte decodes as zero",
            "its start bit and eight data bits are one unbroken low run; the receiver only hears about level changes, so bit counts come from how long a level is held",
        );
        expect!(
            "all-ones",
            "the all-ones byte decodes as all ones",
            "its data, stop bit and the idle after are one unbroken high run, so no edge ever marks the frame's end; the receiver closes it on the clock",
        );
        let framing = UartFraming::new_8n1(115_200);
        assert_eq!(round_trip(framing, 0x00), Some(Ok(0x00)));
        assert_eq!(round_trip(framing, 0xFF), Some(Ok(0xFF)));
    }

    #[test]
    fn msb_first_is_the_mirror_of_lsb_first() {
        behaviour!(Test {
            id: "uart.msb-first",
            covers: Some("board/src/uart.rs#UartEncoder::encode"),
            given: "a link configured most-significant bit first, carrying a byte whose bit pattern is not symmetric",
        });
        expect!(
            "round-trips",
            "the receiver reads back the byte that was sent"
        );
        expect!(
            "wire-differs",
            "the levels put on the line differ from those the same byte produces least-significant bit first",
        );
        let lsb = UartFraming::new_8n1(115_200);
        let msb = UartFraming {
            lsb_first: false,
            ..lsb
        };
        assert_eq!(round_trip(msb, 0xA4), Some(Ok(0xA4)));
        // 0xA4 is not bit-symmetric, so the two orders differ on the wire.
        assert_ne!(
            UartEncoder::new(lsb).encode(0xA4),
            UartEncoder::new(msb).encode(0xA4)
        );
    }

    /// Known-good waveform, written out by hand rather than produced by the
    /// encoder — a round trip against itself would agree with itself even if
    /// the bit order were backwards.
    #[test]
    fn a_hand_written_waveform_decodes() {
        behaviour!(Test {
            id: "uart.hand-written-waveform",
            covers: Some("board/src/uart.rs#UartDecoder::on_level"),
            given: "a hand-written waveform for a known byte, least-significant bit first, that is played onto the line bit by bit",
        });
        expect!(
            "decodes",
            "the receiver reads the byte the waveform encodes",
            "sender and receiver would agree with each other even with the bit order reversed, so the order is checked against a reference neither of them produced",
        );
        use Level::{High as H, Low as L};
        let framing = UartFraming::new_8n1(1_000_000); // 1 µs per bit
        let mut dec = UartDecoder::new(framing);
        // 0x31 = 0b0011_0001, LSB first on the wire: 1 0 0 0 1 1 0 0
        let wire = [L, H, L, L, L, H, H, L, L, H];
        let mut t = 0u64;
        for level in wire {
            dec.on_level(level, t);
            t += 1_000;
        }
        assert_eq!(dec.poll(t), Some(Ok(0x31)));
    }

    #[test]
    fn a_low_stop_bit_is_a_framing_error() {
        behaviour!(Test {
            id: "uart.low-stop-bit",
            covers: Some("board/src/uart.rs#UartDecoder::poll"),
            given: "the line falls and stays low past where the stop bit should be",
        });
        expect!("framing-error", "the frame is reported as a framing error");
        expect!(
            "recovers",
            "a byte sent after the line returns to idle decodes cleanly",
            "a break must not poison every frame that follows it",
        );
        let framing = UartFraming::new_8n1(115_200);
        let mut dec = UartDecoder::new(framing);
        // A break: the line falls and simply stays down past the stop bit.
        dec.on_level(Level::Low, 1_000);
        let after_break = 1_000 + framing.frame_ns();
        assert_eq!(dec.poll(after_break), Some(Err(FramingError::BadStopBit)));

        // The line must return to idle before another frame can start.
        dec.on_level(Level::High, after_break);
        let idle_until = after_break + framing.bit_period_ns;
        let end = play(&mut dec, framing, 0x5A, idle_until);
        assert_eq!(dec.poll(end), Some(Ok(0x5A)));
    }

    /// Back-to-back frames with no idle gap and no poll in between: the edge
    /// that opens the second frame has to close the first.
    #[test]
    fn back_to_back_frames_decode_without_an_intervening_poll() {
        behaviour!(Test {
            id: "uart.back-to-back-without-a-read",
            covers: Some("board/src/uart.rs#UartDecoder::on_level"),
            given: "a stream of bytes with no idle gap between them, and the receiver is read only once the last has ended",
        });
        expect!(
            "all-in-order",
            "every byte comes out, in the order sent",
            "the edge that opens a frame closes the one before it, so decoding cannot depend on how often the receiver is read",
        );
        let framing = UartFraming::new_8n1(2_000_000);
        let mut dec = UartDecoder::new(framing);
        let mut t = 500_000u64;
        for byte in [0x55, 0xAA, 0x00, 0xFF, 0x01] {
            t = play(&mut dec, framing, byte, t);
        }
        let mut got = Vec::new();
        while let Some(frame) = dec.poll(t) {
            got.push(frame);
        }
        assert_eq!(got, vec![Ok(0x55), Ok(0xAA), Ok(0x00), Ok(0xFF), Ok(0x01)]);
    }

    #[test]
    fn the_frame_deadline_is_how_a_silent_tail_gets_closed() {
        behaviour!(Test {
            id: "uart.frame-completion-time",
            covers: Some("board/src/uart.rs#UartDecoder::frame_deadline_ns"),
            given: "an all-ones byte arriving on an idle line, its only edge the start bit",
        });
        expect!(
            "none-while-idle",
            "before the start bit, the receiver names no completion time"
        );
        expect!(
            "one-frame-after-start",
            "once the start bit falls, the receiver names the frame's completion time as one full frame after it",
            "a listener that only hears about edges has to arm a timer for this, since a frame whose tail is silent has no edge to close it",
        );
        expect!(
            "unchanged-by-edges",
            "edges inside the frame leave that time unchanged"
        );
        expect!(
            "closes-at-that-time",
            "read at that instant, the byte comes out and no completion time remains",
        );
        let framing = UartFraming::new_8n1(115_200);
        let mut dec = UartDecoder::new(framing);
        assert_eq!(dec.frame_deadline_ns(), None, "no frame, no deadline");

        dec.on_level(Level::Low, 7_000);
        assert_eq!(dec.frame_deadline_ns(), Some(7_000 + framing.frame_ns()));

        // 0xFF: the start bit is the only transition in the whole frame, so
        // the deadline is the only thing that can close it.
        dec.on_level(Level::High, 7_000 + framing.bit_period_ns);
        assert_eq!(dec.frame_deadline_ns(), Some(7_000 + framing.frame_ns()));
        assert_eq!(dec.poll(dec.frame_deadline_ns().unwrap()), Some(Ok(0xFF)));
        assert_eq!(dec.frame_deadline_ns(), None);
    }

    #[test]
    fn a_frame_is_not_reported_before_it_has_elapsed() {
        behaviour!(Test {
            id: "uart.frame-held-to-full-length",
            covers: Some("board/src/uart.rs#UartDecoder::poll"),
            given: "a start bit has fallen and the receiver is asked for a byte a moment before the frame's nominal end",
        });
        expect!(
            "nothing-yet",
            "no byte is reported",
            "the half-bit tolerance is for a frame the next start bit closes; a frame the clock closes runs its full nominal length",
        );
        let framing = UartFraming::new_8n1(115_200);
        let mut dec = UartDecoder::new(framing);
        dec.on_level(Level::Low, 0);
        assert_eq!(dec.poll(framing.frame_ns() - 1), None);
    }

    #[test]
    fn baud_sets_the_bit_period() {
        behaviour!(Test {
            id: "uart.baud-sets-framing",
            covers: Some("board/src/uart.rs#UartFraming::new_8n1"),
            given: "a link configured 8N1 at a stated baud rate",
        });
        expect!(
            "bit-period",
            "one bit lasts one second divided by the baud rate"
        );
        expect!(
            "ten-bits-per-frame",
            "a frame is ten bits: one start, eight data, one stop"
        );
        expect!("frame-time", "a frame lasts ten bit periods");
        assert_eq!(UartFraming::new_8n1(1_000_000).bit_period_ns, 1_000);
        assert_eq!(UartFraming::new_8n1(2_000_000).frame_bits(), 10);
        assert_eq!(UartFraming::new_8n1(2_000_000).frame_ns(), 5_000);
    }
}
