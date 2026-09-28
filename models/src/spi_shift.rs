//! A byte-wide serial shift register, MSB first — the bit-level engine every
//! SPI-mode device model here is built on ([`crate::spi_flash`], the serial
//! NOR flash; [`crate::psram`], the QSPI PSRAM in its SPI mode), so a second
//! device is a command state machine and nothing else.
//!
//! The engine samples data-in on the **rising** clock edge and presents the
//! data-out bit **for that pulse before advancing** — the rule
//! [`crate::spi_flash`]'s module docs settle for a bit-banging master that
//! reads the line after driving the clock, and one a peripheral-clocked
//! master sampling mid-pulse sees identically. A repeated clock level is not
//! an edge, so a caller may forward every sense of the clock net without
//! tracking edges itself.
//!
//! What the engine does not know: which byte comes next. A device answers
//! [`RisingEdge::finished_out`] by loading the next out byte with
//! [`ByteShifter::load_out`], from whatever phase its command decoder is in
//! after consuming [`RisingEdge::byte_in`] on the same edge — the same order
//! the flash model always kept.

/// Byte-wide MSB-first shift register with the clock level it last saw.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ByteShifter {
    clk_high: bool,
    /// Bits shifted in this byte, MSB first.
    in_bits: u8,
    in_count: u32,
    /// The byte being shifted out and how far through it we are.
    out_byte: u8,
    out_count: u32,
    /// The bit currently presented on data-out.
    dout: bool,
}

impl Default for ByteShifter {
    fn default() -> Self {
        Self::new()
    }
}

/// What one rising clock edge completed.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct RisingEdge {
    /// The eighth input bit landed: a whole byte, MSB first.
    pub byte_in: Option<u8>,
    /// The out byte's last bit was presented on this edge, and the byte
    /// itself; the device loads the next with [`ByteShifter::load_out`].
    pub finished_out: Option<u8>,
}

impl ByteShifter {
    /// An idle register: clock low, nothing shifted, data-out presenting a
    /// one (the level a released line idles at on its pull-up).
    pub const fn new() -> Self {
        Self {
            clk_high: false,
            in_bits: 0,
            in_count: 0,
            out_byte: 0xFF,
            out_count: 0,
            dout: true,
        }
    }

    /// Start a transaction: counters to zero, `first_out` loaded and its MSB
    /// presented at once — what a device does on select, so the first bit
    /// is on the line before the first clock.
    pub fn begin(&mut self, first_out: u8) {
        self.in_bits = 0;
        self.in_count = 0;
        self.out_count = 0;
        self.out_byte = first_out;
        self.dout = self.out_bit();
    }

    /// The bit currently presented on data-out.
    pub const fn dout(&self) -> bool {
        self.dout
    }

    /// The clock level last seen.
    pub const fn clock_is_high(&self) -> bool {
        self.clk_high
    }

    /// Record the clock level without shifting — for a device that is not
    /// selected, so a level that changes while deselected is not an edge
    /// the moment it is selected.
    pub fn track_clock(&mut self, high: bool) {
        self.clk_high = high;
    }

    /// Move the clock to `high` with `din` on the input line. `None` for a
    /// repeated level or a falling edge (which carries nothing); on a rising
    /// edge the input bit is shifted in, then the current output bit is
    /// presented and the position advances.
    pub fn clock(&mut self, high: bool, din: bool) -> Option<RisingEdge> {
        if self.clk_high == high {
            return None;
        }
        self.clk_high = high;
        if !high {
            return None;
        }
        let mut edge = RisingEdge::default();
        // Input: sample and assemble the byte.
        self.in_bits = (self.in_bits << 1) | u8::from(din);
        self.in_count += 1;
        if self.in_count == 8 {
            self.in_count = 0;
            edge.byte_in = Some(self.in_bits);
            self.in_bits = 0;
        }
        // Output: present the current bit NOW, then step. Stepping first
        // reads back one bit late.
        self.dout = self.out_bit();
        self.out_count += 1;
        if self.out_count == 8 {
            self.out_count = 0;
            edge.finished_out = Some(self.out_byte);
        }
        Some(edge)
    }

    /// Load the byte to shift out next; its MSB is presented on the next
    /// rising edge. Call after [`RisingEdge::finished_out`].
    pub fn load_out(&mut self, byte: u8) {
        self.out_byte = byte;
    }

    /// The bit at the current output position of the out byte.
    fn out_bit(&self) -> bool {
        self.out_byte >> (7 - self.out_count.min(7)) & 1 != 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pulse(shift: &mut ByteShifter, din: bool) -> RisingEdge {
        let edge = shift.clock(true, din).expect("a rising edge");
        assert!(
            shift.clock(false, din).is_none(),
            "falling edges carry nothing"
        );
        edge
    }

    #[test]
    fn eight_rising_edges_assemble_a_byte_msb_first() {
        let mut shift = ByteShifter::new();
        shift.begin(0x00);
        let mut completed = None;
        for i in (0..8).rev() {
            let edge = pulse(&mut shift, (0xA5 >> i) & 1 != 0);
            if edge.byte_in.is_some() {
                completed = edge.byte_in;
            }
        }
        assert_eq!(completed, Some(0xA5));
    }

    #[test]
    fn the_out_byte_is_presented_a_bit_per_pulse_msb_first_from_select() {
        let mut shift = ByteShifter::new();
        shift.begin(0x81);
        assert!(
            shift.dout(),
            "the MSB is on the line before the first clock"
        );
        let mut seen = 0u8;
        let mut finished = None;
        for _ in 0..8 {
            let edge = pulse(&mut shift, true);
            seen = (seen << 1) | u8::from(shift.dout());
            if edge.finished_out.is_some() {
                finished = edge.finished_out;
            }
        }
        assert_eq!(seen, 0x81, "pulse then sample reads the byte back");
        assert_eq!(
            finished,
            Some(0x81),
            "and the eighth edge says the byte is done"
        );
    }

    #[test]
    fn a_repeated_level_is_not_an_edge_and_a_tracked_level_shifts_nothing() {
        let mut a = ByteShifter::new();
        let mut b = ByteShifter::new();
        a.begin(0xFF);
        b.begin(0xFF);
        for i in (0..8).rev() {
            let bit = (0x9F >> i) & 1 != 0;
            let ea = a.clock(true, bit);
            let eb = b.clock(true, bit);
            assert!(
                b.clock(true, bit).is_none(),
                "the same level twice is one edge"
            );
            assert_eq!(ea, eb);
            a.clock(false, bit);
            b.clock(false, bit);
            b.clock(false, bit);
        }
        assert_eq!(a, b);

        let mut c = ByteShifter::new();
        c.track_clock(true);
        assert!(c.clock_is_high());
        assert!(
            c.clock(true, true).is_none(),
            "a level that changed while deselected is not an edge once selected"
        );
    }
}
