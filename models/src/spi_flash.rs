//! A serial NOR flash, device side — generic, bit-level, bus-agnostic.
//!
//! The model is a shift register with a command state machine and a backing
//! image. It knows nothing about which MCU is driving it, which pins it sits
//! on, or whether those pins are bit-banged by software or clocked by a
//! peripheral: the whole surface is [`SpiNorFlash::set_selected`],
//! [`SpiNorFlash::clock`] and [`SpiNorFlash::miso`]. Anything that can produce
//! a chip select, a clock edge and a data bit can talk to it, which is what
//! makes it reusable across boards and MCUs.
//!
//! Mounting it on a netlist — deciding that CS is *this* pin and CLK is *that*
//! one — is the board adapter's job, not this module's.
//!
//! # Bit presentation, and why it is spelled out
//!
//! The output bit is **stable on MISO throughout the clock pulse**: presented
//! at select, and again on each rising edge before the position advances. MOSI
//! is sampled on the rising edge.
//!
//! That is not a stylistic choice. A bit-banging master reads the data line
//! after driving the clock, and on a Propeller 2 the pin input registers a beat
//! late, so the bit it reads after a pulse is the one the device presented
//! *for* that pulse. Presenting on the falling edge instead reads back one bit
//! late, and a boot ROM's checksum never matches. A peripheral-clocked master
//! samples mid-pulse and sees the same bit, so this presentation serves both.
//!
//! # Provenance
//!
//! **This model is not datasheet-derived, and says so rather than implying
//! otherwise.** Its command set and behaviour come from the two programs that
//! exercise it — the Propeller 2 boot ROM's `try_spi` (read path) and loadp2's
//! `flash_loader` stub (program path) — run as real machine code against it
//! until they behave as they do on hardware. It was ported from
//! `MaD/SIL/p2core/src/flash.rs`, where that validation lives
//! (`p2core/tests/flash_program.rs` drives the loader stub through it).
//!
//! The command opcodes are the JEDEC-standard SPI NOR set common to Winbond
//! W25Q, Macronix MX25 and Micron N25Q parts; the default JEDEC ID reports a
//! W25Q128, which is what a Parallax P2 Edge module carries. A part whose
//! behaviour differs in these commands would need its own model.
//!
//! What a datasheet would be needed to claim, and what is therefore **not**
//! modelled: program and erase *timing* (WIP always reads 0, so a poll exits
//! at once), power-on reset delay, SFDP, dual/quad I/O modes, block protection
//! and status register 2/3, suspend/resume, and OTP regions.
//!
//! NOR semantics are modelled where they bite: a program may only clear bits
//! (the byte is AND-ed into place), so writing without erasing first gives the
//! wrong answer here exactly as it would on the part. Erase sets `$FF`. The
//! write-enable latch clears after each program or erase, as on silicon.

/// Default manufacturer/type/capacity triple for `$9F`: Winbond W25Q128.
///
/// A loader generally only needs a non-`$FF` first byte to believe a device
/// answered; the rest is reported for completeness.
pub const JEDEC_ID_W25Q128: [u8; 3] = [0xEF, 0x40, 0x18];

/// Bytes in a page program before it wraps. 256 on every part in this family.
pub const DEFAULT_PAGE_SIZE: u32 = 256;

/// How many served bytes and commands to retain for diagnostics. A boot reads
/// the whole image, and keeping all of it would be a leak in a long run.
const DIAG_LIMIT: usize = 4096;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
enum Phase {
    #[default]
    Command,
    /// Collecting the 24-bit address of a `$03` read.
    Address {
        have: u32,
    },
    Reading,
    Status,
    /// Reading the JEDEC ID triple.
    Jedec,
    /// Collecting the 24-bit address of a page program.
    ProgramAddress {
        have: u32,
    },
    /// Address taken; every further byte is programmed.
    Programming,
    /// Collecting the 24-bit address of an erase of `span` bytes.
    EraseAddress {
        span: u32,
        have: u32,
    },
}

/// A serial NOR flash chip.
#[derive(Debug, Default)]
pub struct SpiNorFlash {
    image: Vec<u8>,
    jedec_id: [u8; 3],
    page_size: u32,

    selected: bool,
    clk_high: bool,
    /// Bits shifted in this byte, MSB first.
    in_bits: u8,
    in_count: u32,
    /// The byte being shifted out and how far through it we are.
    out_byte: u8,
    out_count: u32,
    /// The bit currently held on MISO, updated as the clock moves.
    miso_bit: bool,
    phase: Phase,
    addr_acc: u32,
    read_addr: u32,
    /// Write-enable latch. Set by `$06`, cleared by `$04` and by completing a
    /// program or erase — so a loader that forgets `$06` writes nothing, here
    /// as on the part.
    wel: bool,
    /// Where the next programmed byte lands, and the base of its page.
    prog_addr: u32,
    prog_page: u32,
    /// How far through the JEDEC triple a `$9F` read has got.
    jedec_idx: u32,

    /// Starting addresses of `$03` reads served, oldest first.
    pub reads: Vec<u32>,
    /// Output bytes fully shifted out, oldest first.
    pub served: Vec<u8>,
    /// Every command opcode decoded, in order. The cheapest honest answer to
    /// "what does this loader actually issue?" — read it, do not guess from
    /// byte frequencies in the binary.
    pub commands: Vec<u8>,
    /// `(address, len)` of each program and erase applied, oldest first.
    pub writes: Vec<(u32, u32)>,
    pub erases: Vec<(u32, u32)>,
}

impl SpiNorFlash {
    /// A blank part of `capacity` bytes: erased, so every byte reads `$FF`.
    pub fn blank(capacity: usize) -> Self {
        Self::with_image(vec![0xFF; capacity])
    }

    /// A part preloaded with `image`. Its length is the capacity; reads beyond
    /// it return `$FF`, as an address past the end of a real part's array
    /// would after erase.
    pub fn with_image(image: Vec<u8>) -> Self {
        let mut flash = Self {
            image,
            jedec_id: JEDEC_ID_W25Q128,
            page_size: DEFAULT_PAGE_SIZE,
            ..Self::default()
        };
        flash.miso_bit = true;
        flash
    }

    /// Report a different manufacturer/type/capacity triple for `$9F`.
    pub fn with_jedec_id(mut self, id: [u8; 3]) -> Self {
        self.jedec_id = id;
        self
    }

    /// Use a page size other than [`DEFAULT_PAGE_SIZE`]. Must be a power of
    /// two: a page program wraps by masking, as the part does.
    pub fn with_page_size(mut self, bytes: u32) -> Self {
        debug_assert!(bytes.is_power_of_two(), "page size must be a power of two");
        self.page_size = bytes;
        self
    }

    /// The backing image, as programming and erase have left it.
    pub fn image_bytes(&self) -> Vec<u8> {
        self.image.clone()
    }

    /// Capacity in bytes.
    pub fn capacity(&self) -> usize {
        self.image.len()
    }

    /// Whether the part holds an image at all. A zero-capacity flash answers
    /// every read with `$FF`, which is how a master concludes that no device
    /// is fitted.
    pub fn present(&self) -> bool {
        !self.image.is_empty()
    }

    /// The bit currently on the data-out line.
    pub fn miso(&self) -> bool {
        if !self.selected {
            return true; // a released line idles high on its pull-up
        }
        self.miso_bit
    }

    /// Chip select changed. `selected` is the ASSERTED sense — the caller
    /// inverts an active-low ~CS before calling, because whether the pin is
    /// active low is a property of the wiring, not of the chip's state
    /// machine.
    pub fn set_selected(&mut self, selected: bool) {
        if self.selected == selected {
            return;
        }
        self.selected = selected;
        if !selected
            && matches!(
                self.phase,
                Phase::Programming | Phase::ProgramAddress { .. } | Phase::EraseAddress { .. }
            )
        {
            // A program or erase commits when the device is deselected; the
            // latch drops with it, so the next write needs its own `$06`.
            self.wel = false;
        }
        self.in_count = 0;
        self.in_bits = 0;
        self.out_count = 0;
        self.jedec_idx = 0;
        self.phase = Phase::Command;
        self.out_byte = self.next_out();
        self.miso_bit = self.out_bit();
    }

    /// Move the clock to `high`, sampling `mosi`. Returns after updating the
    /// outgoing bit, so a master that reads the data line immediately after
    /// the edge sees the fresh value.
    ///
    /// Idempotent in the level: calling twice with the same level is not two
    /// edges. A caller may therefore forward every sense of the clock net
    /// without tracking edges itself.
    pub fn clock(&mut self, high: bool, mosi: bool) {
        if self.clk_high == high || !self.selected {
            self.clk_high = high;
            return;
        }
        self.clk_high = high;
        if !high {
            return; // the falling edge carries nothing
        }
        // Input: sample MOSI and assemble the byte.
        self.in_bits = (self.in_bits << 1) | u8::from(mosi);
        self.in_count += 1;
        if self.in_count == 8 {
            self.in_count = 0;
            let byte = self.in_bits;
            self.in_bits = 0;
            self.consume(byte);
        }
        // Output: present the current bit NOW, then step -- see the module
        // note on bit presentation. Stepping first reads back one bit late.
        self.miso_bit = self.out_bit();
        self.out_count += 1;
        if self.out_count == 8 {
            if self.served.len() < DIAG_LIMIT {
                self.served.push(self.out_byte);
            }
            self.out_count = 0;
            self.out_byte = self.next_out();
        }
    }

    /// The byte to shift out next, by phase.
    fn next_out(&mut self) -> u8 {
        match self.phase {
            // bit0 WIP, bit1 WEL. Programming is instantaneous here, so WIP is
            // always clear and a master's "wait while busy" spin exits at once.
            Phase::Status => u8::from(self.wel) << 1,
            Phase::Jedec => {
                let b = self
                    .jedec_id
                    .get(self.jedec_idx as usize)
                    .copied()
                    .unwrap_or(0);
                self.jedec_idx = self.jedec_idx.saturating_add(1);
                b
            }
            Phase::Reading => {
                let b = self
                    .image
                    .get(self.read_addr as usize)
                    .copied()
                    .unwrap_or(0xFF);
                self.read_addr = self.read_addr.wrapping_add(1);
                b
            }
            _ => 0xFF,
        }
    }

    /// The bit at the current output position of `out_byte`.
    fn out_bit(&self) -> bool {
        self.out_byte >> (7 - self.out_count.min(7)) & 1 != 0
    }

    /// Program one byte. NOR flash can only pull bits to 0, so this AND-s
    /// rather than assigns: programming over un-erased data gives the same
    /// wrong answer here that it gives on the part.
    fn program_byte(&mut self, addr: u32, byte: u8) {
        if let Some(cell) = self.image.get_mut(addr as usize) {
            *cell &= byte;
        }
    }

    /// Erase `span` bytes around `addr`, or the whole part when `span` is 0.
    fn erase(&mut self, addr: u32, span: u32) {
        let (start, end) = if span == 0 {
            (0usize, self.image.len())
        } else {
            let base = (addr & !(span - 1)) as usize;
            (base, base + span as usize)
        };
        let end = end.min(self.image.len());
        if start < end {
            self.image[start..end].fill(0xFF);
            self.erases.push((start as u32, (end - start) as u32));
        }
        self.wel = false;
    }

    fn consume(&mut self, byte: u8) {
        self.phase = match self.phase {
            Phase::Command => {
                if self.commands.len() < DIAG_LIMIT {
                    self.commands.push(byte);
                }
                match byte {
                    0x03 => Phase::Address { have: 0 },
                    0x05 => Phase::Status,
                    0x9F => {
                        self.jedec_idx = 0;
                        Phase::Jedec
                    }
                    0x06 => {
                        self.wel = true;
                        Phase::Command
                    }
                    0x04 => {
                        self.wel = false;
                        Phase::Command
                    }
                    0x02 => Phase::ProgramAddress { have: 0 },
                    0x20 => Phase::EraseAddress {
                        span: 4 * 1024,
                        have: 0,
                    },
                    0x52 => Phase::EraseAddress {
                        span: 32 * 1024,
                        have: 0,
                    },
                    0xD8 => Phase::EraseAddress {
                        span: 64 * 1024,
                        have: 0,
                    },
                    // Chip erase takes no address: act at once.
                    0x60 | 0xC7 => {
                        if self.wel {
                            self.erase(0, 0);
                        }
                        Phase::Command
                    }
                    // `$66`/`$99` reset and anything else: accepted as a no-op
                    // rather than ignored as unknown. They need no state to be
                    // correct, and a master that issues them is not in error.
                    _ => Phase::Command,
                }
            }
            Phase::Address { have } => {
                self.addr_acc = (self.addr_acc << 8) | u32::from(byte);
                if have == 2 {
                    let addr = self.addr_acc & 0x00FF_FFFF;
                    self.addr_acc = 0;
                    self.read_addr = addr;
                    if self.reads.len() < DIAG_LIMIT {
                        self.reads.push(addr);
                    }
                    Phase::Reading
                } else {
                    Phase::Address { have: have + 1 }
                }
            }
            Phase::ProgramAddress { have } => {
                self.addr_acc = (self.addr_acc << 8) | u32::from(byte);
                if have == 2 {
                    let addr = self.addr_acc & 0x00FF_FFFF;
                    self.addr_acc = 0;
                    self.prog_addr = addr;
                    self.prog_page = addr & !(self.page_size - 1);
                    self.writes.push((addr, 0));
                    Phase::Programming
                } else {
                    Phase::ProgramAddress { have: have + 1 }
                }
            }
            Phase::Programming => {
                if self.wel {
                    let addr = self.prog_addr;
                    self.program_byte(addr, byte);
                    if let Some(last) = self.writes.last_mut() {
                        last.1 += 1;
                    }
                    // A page program wraps within its own page rather than
                    // running on into the next one.
                    let next = addr.wrapping_add(1);
                    self.prog_addr = if next & !(self.page_size - 1) != self.prog_page {
                        self.prog_page
                    } else {
                        next
                    };
                }
                Phase::Programming
            }
            Phase::EraseAddress { span, have } => {
                self.addr_acc = (self.addr_acc << 8) | u32::from(byte);
                if have == 2 {
                    let addr = self.addr_acc & 0x00FF_FFFF;
                    self.addr_acc = 0;
                    if self.wel {
                        self.erase(addr, span);
                    }
                    Phase::Command
                } else {
                    Phase::EraseAddress {
                        span,
                        have: have + 1,
                    }
                }
            }
            other => other,
        };
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Shift a byte in, MSB first.
    fn send(flash: &mut SpiNorFlash, byte: u8) {
        for i in (0..8).rev() {
            let bit = (byte >> i) & 1 != 0;
            flash.clock(true, bit);
            flash.clock(false, bit);
        }
    }

    /// Read a byte out, MSB first — pulse then sample, as a bit-banging master
    /// does.
    fn recv(flash: &mut SpiNorFlash) -> u8 {
        let mut b = 0u8;
        for _ in 0..8 {
            flash.clock(true, true);
            flash.clock(false, true);
            b = (b << 1) | u8::from(flash.miso());
        }
        b
    }

    /// One complete transaction: select, send bytes, deselect.
    fn tx(flash: &mut SpiNorFlash, bytes: &[u8]) {
        flash.set_selected(true);
        for &b in bytes {
            send(flash, b);
        }
        flash.set_selected(false);
    }

    #[test]
    fn a_read_command_streams_from_the_addressed_offset() {
        let mut image = vec![0u8; 0x500];
        image[0x400] = 0xDE;
        image[0x401] = 0xAD;
        let mut flash = SpiNorFlash::with_image(image);
        flash.set_selected(true);
        send(&mut flash, 0x03);
        send(&mut flash, 0x00);
        send(&mut flash, 0x04);
        send(&mut flash, 0x00);
        assert_eq!(recv(&mut flash), 0xDE, "first byte at $400");
        assert_eq!(recv(&mut flash), 0xAD, "then $401");
        assert_eq!(flash.reads, vec![0x400]);
    }

    #[test]
    fn a_read_past_the_end_of_the_array_gives_erased_bytes() {
        let mut flash = SpiNorFlash::with_image(vec![0x11; 4]);
        flash.set_selected(true);
        send(&mut flash, 0x03);
        send(&mut flash, 0x00);
        send(&mut flash, 0x00);
        send(&mut flash, 0x03);
        assert_eq!(recv(&mut flash), 0x11, "the last real byte");
        assert_eq!(recv(&mut flash), 0xFF, "then past the end");
    }

    #[test]
    fn status_reads_zero_so_a_master_finds_an_idle_writable_device() {
        let mut flash = SpiNorFlash::with_image(vec![1, 2, 3, 4]);
        flash.set_selected(true);
        send(&mut flash, 0x05);
        assert_eq!(recv(&mut flash), 0x00, "idle and writable");
    }

    #[test]
    fn a_deselected_part_releases_the_line_high() {
        let mut flash = SpiNorFlash::with_image(vec![0x00; 4]);
        assert!(flash.miso(), "idles high before any transaction");
        flash.set_selected(true);
        send(&mut flash, 0x03);
        send(&mut flash, 0x00);
        send(&mut flash, 0x00);
        send(&mut flash, 0x00);
        flash.set_selected(false);
        assert!(flash.miso(), "and again once released");
    }

    #[test]
    fn a_page_program_needs_the_write_enable_latch() {
        let mut flash = SpiNorFlash::blank(512);
        // No `$06` first: the part ignores the data, and so does this.
        tx(&mut flash, &[0x02, 0x00, 0x00, 0x00, 0xAA]);
        assert_eq!(flash.image_bytes()[0], 0xFF, "unlatched write is discarded");

        tx(&mut flash, &[0x06]);
        tx(&mut flash, &[0x02, 0x00, 0x00, 0x00, 0xAA]);
        assert_eq!(flash.image_bytes()[0], 0xAA, "latched write lands");
    }

    #[test]
    fn the_latch_clears_after_a_program_so_the_next_write_needs_its_own_enable() {
        let mut flash = SpiNorFlash::blank(512);
        tx(&mut flash, &[0x06]);
        tx(&mut flash, &[0x02, 0x00, 0x00, 0x00, 0x0F]);
        // Second write, no new `$06`.
        tx(&mut flash, &[0x02, 0x00, 0x00, 0x01, 0x0F]);
        let img = flash.image_bytes();
        assert_eq!(img[0], 0x0F, "first write was enabled");
        assert_eq!(img[1], 0xFF, "second was not");
    }

    #[test]
    fn programming_only_clears_bits_so_an_unerased_write_is_wrong_here_too() {
        let mut flash = SpiNorFlash::with_image(vec![0x0F; 4]);
        tx(&mut flash, &[0x06]);
        tx(&mut flash, &[0x02, 0x00, 0x00, 0x00, 0xF0]);
        assert_eq!(
            flash.image_bytes()[0],
            0x00,
            "NOR programming ANDs: $0F & $F0 = $00, not $F0"
        );
    }

    #[test]
    fn a_page_program_wraps_inside_its_own_page() {
        let mut flash = SpiNorFlash::blank(1024);
        tx(&mut flash, &[0x06]);
        // Start two bytes below the page end and write four.
        let mut frame = vec![0x02, 0x00, 0x00, 0xFE];
        frame.extend_from_slice(&[0x11, 0x22, 0x33, 0x44]);
        tx(&mut flash, &frame);
        let img = flash.image_bytes();
        assert_eq!((img[0xFE], img[0xFF]), (0x11, 0x22), "the tail of the page");
        assert_eq!(
            img[0x100], 0xFF,
            "the next page is untouched — the write wrapped"
        );
        assert_eq!(
            (img[0x00], img[0x01]),
            (0x33, 0x44),
            "it wrapped to the base"
        );
    }

    #[test]
    fn the_page_size_is_configurable_and_the_wrap_follows_it() {
        let mut flash = SpiNorFlash::blank(1024).with_page_size(16);
        tx(&mut flash, &[0x06]);
        tx(&mut flash, &[0x02, 0x00, 0x00, 0x0E, 0x11, 0x22, 0x33]);
        let img = flash.image_bytes();
        assert_eq!((img[0x0E], img[0x0F]), (0x11, 0x22), "the tail of page 0");
        assert_eq!(img[0x10], 0xFF, "page 1 untouched");
        assert_eq!(img[0x00], 0x33, "wrapped to the base of a 16-byte page");
    }

    #[test]
    fn a_sector_erase_clears_its_own_4k_and_nothing_else() {
        let mut flash = SpiNorFlash::with_image(vec![0x00; 16 * 1024]);
        tx(&mut flash, &[0x06]);
        // An address inside the second sector, not its base.
        tx(&mut flash, &[0x20, 0x00, 0x11, 0x22]);
        let img = flash.image_bytes();
        assert_eq!(img[0x0FFF], 0x00, "sector 0 untouched");
        assert_eq!(img[0x1000], 0xFF, "sector 1 erased from its base");
        assert_eq!(img[0x1FFF], 0xFF, "to its end");
        assert_eq!(img[0x2000], 0x00, "sector 2 untouched");
        assert_eq!(flash.erases, vec![(0x1000, 4096)]);
    }

    #[test]
    fn a_chip_erase_takes_no_address_and_clears_everything() {
        let mut flash = SpiNorFlash::with_image(vec![0x00; 8 * 1024]);
        tx(&mut flash, &[0x06]);
        tx(&mut flash, &[0xC7]);
        assert!(
            flash.image_bytes().iter().all(|&b| b == 0xFF),
            "the whole array"
        );
    }

    #[test]
    fn the_jedec_id_answers_so_a_loader_can_see_a_device() {
        let mut flash = SpiNorFlash::blank(16);
        flash.set_selected(true);
        send(&mut flash, 0x9F);
        assert_eq!(
            [recv(&mut flash), recv(&mut flash), recv(&mut flash)],
            JEDEC_ID_W25Q128
        );
    }

    #[test]
    fn the_jedec_id_is_configurable_for_a_different_part() {
        let mut flash = SpiNorFlash::blank(16).with_jedec_id([0xC2, 0x20, 0x18]);
        flash.set_selected(true);
        send(&mut flash, 0x9F);
        assert_eq!(
            [recv(&mut flash), recv(&mut flash), recv(&mut flash)],
            [0xC2, 0x20, 0x18],
            "a Macronix MX25L128 answers as itself"
        );
    }

    #[test]
    fn every_opcode_is_logged_so_a_loader_can_be_observed_rather_than_guessed() {
        let mut flash = SpiNorFlash::blank(512);
        tx(&mut flash, &[0x06]);
        tx(&mut flash, &[0x02, 0x00, 0x00, 0x00, 0xAA]);
        tx(&mut flash, &[0x05]);
        assert_eq!(flash.commands, vec![0x06, 0x02, 0x05]);
    }

    #[test]
    fn repeating_a_clock_level_is_not_a_second_edge() {
        let mut a = SpiNorFlash::blank(16);
        let mut b = SpiNorFlash::blank(16);
        a.set_selected(true);
        b.set_selected(true);
        // `a` sees each level once; `b` sees every level twice, as a component
        // forwarding every net sense would.
        for i in (0..8).rev() {
            let bit = (0x9F >> i) & 1 != 0;
            a.clock(true, bit);
            a.clock(false, bit);
            b.clock(true, bit);
            b.clock(true, bit);
            b.clock(false, bit);
            b.clock(false, bit);
        }
        assert_eq!(a.commands, vec![0x9F]);
        assert_eq!(b.commands, a.commands, "duplicate levels changed nothing");
    }
}
