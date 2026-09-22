//! An SD card, device side, in SPI mode — generic and host-agnostic.
//!
//! Anything that can assert a chip select and exchange bytes can talk to this:
//! a bit-banged GPIO master, a hardware SPI peripheral, a DMA engine. The card
//! knows nothing about who is clocking it.
//!
//! Written against permissive references only — never a GPL model:
//!
//! * the **SD Physical Layer Simplified Specification**, for the SPI-mode
//!   command set and response formats;
//! * ChaN's `sdmm.cc` ("no restriction on use"), the reference SPI-mode driver
//!   that ships with several embedded toolchains, which pins down exactly which
//!   commands a real host sends and what each expects back;
//! * differential testing against real driver traffic — run a filesystem stack
//!   against it and see what it sends.
//!
//! # Scope
//!
//! Enough of the SPI-mode protocol for a FAT driver to initialise a card and
//! read and write 512-byte blocks. The card reports itself as SDHC/SDv2
//! (block-addressed), which is the simplest case: `CMD17`/`CMD24` take a block
//! index rather than a byte offset, so no capacity arithmetic is needed.
//!
//! Storage is a flat image — [`SdCard::blocks`] is just the bytes of a disk.
//! **No filesystem logic lives here**, deliberately: the filesystem is normally
//! the code under test, and keeping it out leaves this model dependency-free.
//!
//! # Not modelled
//!
//! Card-detect and write-protect switches, CRC checking (the card accepts any
//! CRC, as a real card does in SPI mode until `CMD59` turns checking on), the
//! `CMD59` / erase / lock-unlock command groups, multi-voltage negotiation
//! beyond the `CMD8` echo, and any notion of programming time — a write
//! completes in one busy byte rather than milliseconds. A host that depends on
//! any of those gets an optimistic answer here.
//!
//! # Byte-level, with an exact bit-level escape hatch
//!
//! [`SdCard::xfer`] moves a whole byte each way at once. On a real bus the two
//! directions are **simultaneous**, so a bit-level adapter needs the outgoing
//! byte at the *start* of an exchange — that is what [`SdCard::peek_miso`] is
//! for, and in SPI mode it is exact rather than an approximation: a response is
//! always queued by an earlier command and never depends on the byte arriving
//! now. [`crate::sd_card_component`] is built on exactly that.

use std::collections::VecDeque;

use tracing::debug;

/// Bytes per block. Fixed at 512 for SDHC.
pub const BLOCK_LEN: usize = 512;

/// R1 response bits. Bit 7 is always 0, which is how the host finds the
/// response byte in a stream of `$FF` idles.
const R1_READY: u8 = 0x00;
const R1_IDLE: u8 = 0x01;
const R1_ILLEGAL: u8 = 0x04;

/// Data-block start token for single-block read/write.
const TOKEN_START: u8 = 0xFE;
/// Data-response token: bits 3:0 = `%0101` means "data accepted".
const TOKEN_ACCEPTED: u8 = 0x05;
/// Start token for each block of a `CMD25` multi-block write.
const TOKEN_MULTI_START: u8 = 0xFC;
/// Stop-transmission token, ending a `CMD25` write.
const TOKEN_STOP_TRAN: u8 = 0xFD;

/// A command frame's first byte is `%01xxxxxx`; everything else on MOSI while
/// the card waits for one is an idle.
const FRAME_START_MASK: u8 = 0xC0;
const FRAME_START_BITS: u8 = 0x40;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Phase {
    /// Waiting for the 6-byte command frame to arrive.
    Command,
    /// Streaming a queued response out to the host.
    Responding,
    /// Receiving a data block the host is writing (token + payload + CRC).
    ReceivingBlock { addr: u32, got: usize },
    /// A `CMD18` read is running: the next block goes out when the host clocks
    /// for it, and a command frame instead (`CMD12`) ends the stream.
    MultiRead { addr: u32 },
}

/// A card that speaks SPI-mode SD over one-byte exchanges.
#[derive(Debug)]
pub struct SdCard {
    /// The disk image. Length should be a multiple of [`BLOCK_LEN`].
    pub blocks: Vec<u8>,
    /// True once `ACMD41` has completed initialisation.
    pub initialised: bool,
    /// Chip select, as the adapter has resolved it. Commands only run when
    /// asserted. Active-low wiring is the adapter's business, not the card's.
    pub selected: bool,
    /// Set when the host sends `CMD55`, so the next command is an ACMD.
    app_cmd: bool,
    /// A `CMD24` whose R1 response is still draining: the block it is waiting
    /// for begins once that response is out. See [`SdCard::after_responding`].
    pending_write: Option<u32>,
    /// Next block of a `CMD25` multi-block write, while one is running.
    multi_write: Option<u32>,
    /// Next block of a `CMD18` multi-block read, while one is running.
    multi_read: Option<u32>,
    phase: Phase,
    /// Bytes queued to return to the host, oldest first.
    out: VecDeque<u8>,
    /// Command frame being assembled.
    frame: Vec<u8>,
    /// Payload being received during a block write.
    incoming: Vec<u8>,
    /// Every command seen, in order, with bit 7 set on an ACMD — the cheapest
    /// way to see how far a host's initialisation sequence got.
    pub commands: Vec<u8>,
    /// Block addresses read, in order — what the host's filesystem asked for.
    ///
    /// Diagnostic, and the cheapest way to see where a mount gives up: a FAT
    /// driver reads the boot sector, then the FAT, then the root. A mount that
    /// stops after one read did not like what it found in that sector.
    pub reads: Vec<u32>,
    /// Block addresses written, in order.
    pub writes: Vec<u32>,
    /// Optional log of `(mosi, miso)` byte exchanges, for bring-up. Capped at
    /// [`Self::TRACE_LIMIT`] exchanges so leaving it on cannot grow unbounded.
    pub trace: Option<Vec<(u8, u8)>>,
}

impl SdCard {
    /// How many byte exchanges [`Self::trace`] retains before it stops
    /// recording.
    pub const TRACE_LIMIT: usize = 4096;

    /// A card holding `bytes` of zeroed image.
    pub fn blank(bytes: usize) -> Self {
        Self::with_image(vec![0u8; bytes])
    }

    /// A card backed by an existing disk image.
    pub fn with_image(blocks: Vec<u8>) -> Self {
        Self {
            blocks,
            initialised: false,
            // Deselected at power-up, which is what a real card is: CS idles
            // high on its pull-up, and the mandatory power-up clock burst is
            // specified to run with the card NOT selected. The host asserts CS
            // when it is ready to talk.
            //
            // Powering up selected would be an accommodation for one driver
            // that happens to assert CS first, and it would put this model out
            // of step with an adapter — [`crate::sd_card_component`] starts its
            // wire deselected, and the two would disagree until the first edge.
            selected: false,
            app_cmd: false,
            pending_write: None,
            multi_write: None,
            multi_read: None,
            phase: Phase::Command,
            out: VecDeque::new(),
            frame: Vec::new(),
            incoming: Vec::new(),
            commands: Vec::new(),
            reads: Vec::new(),
            writes: Vec::new(),
            trace: None,
        }
    }

    /// Record every byte exchange, up to [`Self::TRACE_LIMIT`].
    #[must_use]
    pub fn with_trace(mut self) -> Self {
        self.trace = Some(Vec::new());
        self
    }

    /// Capacity of the backing image in bytes.
    pub fn capacity(&self) -> usize {
        self.blocks.len()
    }

    /// Whole blocks the image holds. A read or write past this is refused.
    pub fn block_count(&self) -> usize {
        self.blocks.len() / BLOCK_LEN
    }

    /// Drive chip select, abandoning anything in flight when it is released.
    ///
    /// Releasing CS ends the current operation on a real card. Modelling that
    /// matters most when the *host* gives up mid-transfer: a driver that sends
    /// `CMD24` and then times out waiting for the card to be ready returns
    /// without ever sending the data token — leaving the card waiting for a
    /// block that will never arrive. Deselect is the only thing that frees it.
    /// Without this the card stays in [`Phase::ReceivingBlock`] forever,
    /// silently swallowing every later command frame as if it were payload,
    /// and the bus goes dead for the rest of the run.
    ///
    /// This has to be a method rather than a write to [`SdCard::selected`]: an
    /// adapter stops exchanging bytes altogether while the card is deselected,
    /// so a card that only noticed the release during an exchange would never
    /// notice it at all.
    pub fn set_selected(&mut self, selected: bool) {
        if self.selected == selected {
            return;
        }
        self.selected = selected;
        if !selected {
            self.phase = Phase::Command;
            self.pending_write = None;
            self.multi_write = None;
            self.multi_read = None;
            self.incoming.clear();
            self.frame.clear();
            self.out.clear();
        }
    }

    /// What the card would put on MISO for the next byte, without consuming
    /// anything.
    ///
    /// See the module docs: this is what makes a bit-level adapter exact
    /// rather than approximate.
    pub fn peek_miso(&self) -> u8 {
        if !self.selected {
            return 0xFF;
        }
        match self.phase {
            Phase::Responding => self.out.front().copied().unwrap_or(0xFF),
            // Command and block-receive phases hold the line idle-high while
            // they take bytes in, exactly as `xfer_inner` returns.
            _ => 0xFF,
        }
    }

    /// Exchange one byte: the host shifts `mosi` in, the card shifts one out.
    ///
    /// SPI is symmetric, so every host byte produces exactly one card byte —
    /// `$FF` whenever the card has nothing to say, which is also how it signals
    /// "busy" and how the host clocks responses out.
    pub fn xfer(&mut self, mosi: u8) -> u8 {
        let miso = self.xfer_inner(mosi);
        if let Some(log) = self.trace.as_mut() {
            if log.len() < Self::TRACE_LIMIT {
                log.push((mosi, miso));
            }
        }
        miso
    }

    fn xfer_inner(&mut self, mosi: u8) -> u8 {
        if !self.selected {
            let _ = mosi;
            return 0xFF;
        }
        match self.phase {
            Phase::Responding => {
                if let Some(b) = self.out.pop_front() {
                    if self.out.is_empty() {
                        self.phase = self.after_responding();
                    }
                    return b;
                }
                self.phase = self.after_responding();
                0xFF
            }
            Phase::ReceivingBlock { addr, got } => self.receive_block_byte(mosi, addr, got),
            Phase::MultiRead { addr } => self.multi_read_byte(mosi, addr),
            Phase::Command => {
                self.collect_command(mosi);
                // Anything queued by the command becomes readable next byte.
                if !self.out.is_empty() {
                    self.phase = Phase::Responding;
                }
                0xFF
            }
        }
    }

    /// Where the card goes once a queued response has drained.
    ///
    /// `CMD24` answers R1 *and then* waits for the block the host is about to
    /// send. Returning to [`Phase::Command`] unconditionally lost that: the
    /// 512 payload bytes were fed to the command collector, which latched on
    /// the first byte matching `%01xxxxxx` and manufactured commands out of
    /// file data (a directory entry's `"BIN "` decoded as `CMD2`), while the
    /// host read `$FF` where it wanted the `$05` data-accepted token and failed
    /// every write.
    fn after_responding(&mut self) -> Phase {
        if let Some(addr) = self.pending_write.take() {
            return Phase::ReceivingBlock { addr, got: 0 };
        }
        // A multi-block transfer stays open across each block's response: the
        // host keeps clocking until it sends the stop token (write) or CMD12
        // (read).
        if let Some(addr) = self.multi_write {
            return Phase::ReceivingBlock { addr, got: 0 };
        }
        if let Some(addr) = self.multi_read {
            return Phase::MultiRead { addr };
        }
        Phase::Command
    }

    /// Queue one block of data with its start token and dummy CRC.
    fn queue_block(&mut self, addr: u32, token: u8) {
        let off = addr as usize * BLOCK_LEN;
        if off + BLOCK_LEN > self.blocks.len() {
            debug!(
                block = addr,
                offset = off,
                image_len = self.blocks.len(),
                recent_commands = ?self.commands.iter().rev().take(12).collect::<Vec<_>>(),
                "SD card: read past the end of the image"
            );
            self.out.push_back(0x08); // out-of-range error token
            return;
        }
        self.out.push_back(token);
        for i in 0..BLOCK_LEN {
            self.out.push_back(self.blocks[off + i]);
        }
        self.out.push_back(0xFF); // CRC hi
        self.out.push_back(0xFF); // CRC lo
    }

    /// Between blocks of a `CMD18` read.
    ///
    /// A typical driver reads exactly one block at a time and then either
    /// clocks on for the next or sends `CMD12` to stop, so the card need not
    /// stream ahead: an idle byte asks for another block, a command frame ends
    /// the read.
    fn multi_read_byte(&mut self, mosi: u8, addr: u32) -> u8 {
        if mosi & FRAME_START_MASK == FRAME_START_BITS {
            self.multi_read = None;
            self.phase = Phase::Command;
            self.collect_command(mosi);
            if !self.out.is_empty() {
                self.phase = Phase::Responding;
            }
            return 0xFF;
        }
        self.reads.push(addr);
        self.queue_block(addr, TOKEN_START);
        self.multi_read = Some(addr.wrapping_add(1));
        self.phase = Phase::Responding;
        0xFF
    }

    fn receive_block_byte(&mut self, mosi: u8, addr: u32, got: usize) -> u8 {
        if got == 0 {
            // The host ends a multi-block write with the stop token in place
            // of another block.
            if self.multi_write.is_some() && mosi == TOKEN_STOP_TRAN {
                self.multi_write = None;
                self.incoming.clear();
                self.out.push_back(0xFF); // one busy byte, then ready
                self.phase = Phase::Responding;
                return 0xFF;
            }
            // Skip idles until a start token arrives: $FE for a single block,
            // $FC for each block of a multi-block write.
            let started =
                mosi == TOKEN_START || (self.multi_write.is_some() && mosi == TOKEN_MULTI_START);
            if !started {
                return 0xFF;
            }
            self.incoming.clear();
            self.phase = Phase::ReceivingBlock { addr, got: 1 };
            return 0xFF;
        }
        if self.incoming.len() < BLOCK_LEN {
            self.incoming.push(mosi);
            return 0xFF;
        }
        // Two CRC bytes follow the payload; consume them, then acknowledge.
        self.incoming.push(mosi);
        if self.incoming.len() >= BLOCK_LEN + 2 {
            let off = addr as usize * BLOCK_LEN;
            if off + BLOCK_LEN > self.blocks.len() {
                debug!(
                    block = addr,
                    offset = off,
                    image_len = self.blocks.len(),
                    "SD card: write past the end of the image, discarded"
                );
            } else {
                self.blocks[off..off + BLOCK_LEN].copy_from_slice(&self.incoming[..BLOCK_LEN]);
            }
            self.incoming.clear();
            if self.multi_write.is_some() {
                self.writes.push(addr);
                self.multi_write = Some(addr.wrapping_add(1));
            }
            // Data-accepted token, then one busy byte before ready.
            self.out.push_back(TOKEN_ACCEPTED);
            self.out.push_back(0xFF);
            self.phase = Phase::Responding;
        }
        0xFF
    }

    fn collect_command(&mut self, mosi: u8) {
        if self.frame.is_empty() {
            if mosi & FRAME_START_MASK != FRAME_START_BITS {
                return;
            }
            self.frame.push(mosi);
            return;
        }
        self.frame.push(mosi);
        if self.frame.len() < 6 {
            return;
        }
        let cmd = self.frame[0] & 0x3F;
        let arg = u32::from_be_bytes([self.frame[1], self.frame[2], self.frame[3], self.frame[4]]);
        // frame[5] is the CRC, which a card ignores in SPI mode unless CMD59
        // has turned checking on. CMD59 is not modelled.
        self.frame.clear();
        self.run(cmd, arg);
    }

    fn run(&mut self, cmd: u8, arg: u32) {
        let app = std::mem::take(&mut self.app_cmd);
        self.commands.push(if app { cmd | 0x80 } else { cmd });

        match (app, cmd) {
            // ACMD41 -- initialise. Report ready immediately; the host polls
            // until it sees 0, and there is nothing to wait for here.
            (true, 41) => {
                self.initialised = true;
                self.out.push_back(R1_READY);
            }
            // ACMD other -- accept and report ready.
            (true, _) => self.out.push_back(R1_READY),

            // CMD0 GO_IDLE_STATE
            (false, 0) => self.out.push_back(R1_IDLE),

            // CMD8 SEND_IF_COND -- an SDv2 card echoes the check pattern in an
            // R7 (R1 + 4 bytes). Answering this is what makes the host treat
            // the card as SDv2/SDHC and use block addressing.
            (false, 8) => {
                self.out.push_back(R1_IDLE);
                self.out.push_back(0x00);
                self.out.push_back(0x00);
                self.out.push_back(0x01); // voltage accepted
                self.out.push_back((arg & 0xFF) as u8); // echo-back pattern
            }

            // CMD58 READ_OCR -- R3. CCS set marks a block-addressed (SDHC) card.
            (false, 58) => {
                self.out.push_back(R1_READY);
                self.out.push_back(0xC0); // ready + CCS
                self.out.push_back(0xFF);
                self.out.push_back(0x80);
                self.out.push_back(0x00);
            }

            // CMD55 APP_CMD -- the next command is an ACMD.
            (false, 55) => {
                self.app_cmd = true;
                self.out.push_back(R1_READY);
            }

            // CMD16 SET_BLOCKLEN -- fixed at 512, so just acknowledge.
            (false, 16) => self.out.push_back(R1_READY),

            // CMD17 READ_SINGLE_BLOCK -- R1, then a start token and 512 bytes.
            (false, 17) => {
                self.out.push_back(R1_READY);
                self.reads.push(arg);
                self.queue_block(arg, TOKEN_START);
            }

            // CMD18 READ_MULTIPLE_BLOCK -- R1, then one block each time the
            // host clocks for another, until it sends CMD12. A FAT driver uses
            // this whenever it asks for more than one sector, which is most of
            // a file read.
            (false, 18) => {
                self.out.push_back(R1_READY);
                self.multi_read = Some(arg);
            }

            // CMD25 WRITE_MULTIPLE_BLOCK -- R1, then a block per $FC token
            // until the $FD stop token, for a write of more than one sector.
            (false, 25) => {
                self.out.push_back(R1_READY);
                self.multi_write = Some(arg);
            }

            // CMD24 WRITE_BLOCK -- R1, then the host sends the data block.
            (false, 24) => {
                self.out.push_back(R1_READY);
                self.writes.push(arg);
                // The block phase must survive the R1 draining, so it is
                // recorded rather than set here: `xfer_inner`'s Command arm
                // overwrites `phase` with `Responding` the moment `out` is
                // non-empty. See `after_responding`.
                self.pending_write = Some(arg);
            }

            // CMD9/CMD10 SEND_CSD/CID -- R1 then a 16-byte register block.
            // Zeroed: no host under test has yet read either for anything but
            // a presence check, and inventing plausible register contents
            // would be a fiction this model cannot back up.
            (false, 9) | (false, 10) => {
                self.out.push_back(R1_READY);
                self.out.push_back(TOKEN_START);
                for _ in 0..16 {
                    self.out.push_back(0x00);
                }
                self.out.push_back(0xFF);
                self.out.push_back(0xFF);
            }

            // CMD12 STOP_TRANSMISSION -- ends a CMD18 stream.
            (false, 12) => {
                self.multi_read = None;
                self.multi_write = None;
                self.out.push_back(R1_READY);
            }
            // CMD13 SEND_STATUS -- R2, which is R1 plus a second status byte.
            (false, 13) => {
                self.out.push_back(R1_READY);
                self.out.push_back(0x00);
            }

            // Anything else: report it as unsupported rather than pretending.
            _ => {
                debug!(app, cmd, arg, "SD card: unsupported command");
                self.out.push_back(R1_ILLEGAL)
            }
        }
    }
}
