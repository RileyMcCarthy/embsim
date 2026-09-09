//! Asynchronous serial framing, so bytes can travel as levels on a net.
//!
//! Today a UART byte crosses a [`crate::StreamRole::Producer`] pin as a byte:
//! the net decides who is connected, but the payload never becomes a level, so
//! it cannot experience contention, a fighting driver, or a floating line. This
//! is the codec that lets those bytes become levels instead — one shared
//! implementation, so every byte-oriented peripheral frames identically rather
//! than growing its own.
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
            if at_ns >= start + self.framing.frame_ns() {
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

    /// Advance virtual time and take the next decoded frame, if any.
    pub fn poll(&mut self, now_ns: u64) -> Option<Result<u8, FramingError>> {
        if let Some(start) = self.frame_start_ns {
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
            Err(FramingError::BadStopBit)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Play an encoded byte into a decoder, returning the time it ended at.
    fn play(dec: &mut UartDecoder, framing: UartFraming, byte: u8, from: u64) -> u64 {
        let mut t = from;
        for (level, hold) in UartEncoder::new(framing).encode(byte) {
            dec.on_level(level, t);
            t += hold;
        }
        t
    }

    fn round_trip(framing: UartFraming, byte: u8) -> Option<Result<u8, FramingError>> {
        let mut dec = UartDecoder::new(framing);
        // Start well after zero, as virtual time would.
        let end = play(&mut dec, framing, byte, 1_000_000);
        dec.poll(end)
    }

    #[test]
    fn every_byte_round_trips() {
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
        let framing = UartFraming::new_8n1(115_200);
        assert_eq!(round_trip(framing, 0x00), Some(Ok(0x00)));
        assert_eq!(round_trip(framing, 0xFF), Some(Ok(0xFF)));
    }

    #[test]
    fn msb_first_is_the_mirror_of_lsb_first() {
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
    fn a_frame_is_not_reported_before_it_has_elapsed() {
        let framing = UartFraming::new_8n1(115_200);
        let mut dec = UartDecoder::new(framing);
        dec.on_level(Level::Low, 0);
        assert_eq!(dec.poll(framing.frame_ns() - 1), None);
    }

    #[test]
    fn baud_sets_the_bit_period() {
        assert_eq!(UartFraming::new_8n1(1_000_000).bit_period_ns, 1_000);
        assert_eq!(UartFraming::new_8n1(2_000_000).frame_bits(), 10);
        assert_eq!(UartFraming::new_8n1(2_000_000).frame_ns(), 5_000);
    }
}
