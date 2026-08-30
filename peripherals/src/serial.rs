//! Serial — UART byte FIFOs with an optional host-FD backend.
//!
//! Firmware HAL talks only to per-channel **RX/TX FIFOs**. A PTY, socketpair,
//! or test harness is a **backend** that fills RX and drains TX (via an
//! attached FD or [`Serial::write_host_rx`] / [`Serial::take_host_tx`]).
//! Baud pacing is virtual-time.
//!
//! State lives in a per-MCU [`Serial`] bank owned by
//! `instance::PeripheralInstance`. The module-level free functions route to
//! the calling thread's instance (see `crate::instance`), so existing
//! single-MCU consumers are unaffected.

use std::collections::VecDeque;
use std::os::fd::{BorrowedFd, RawFd};
use std::sync::atomic::{AtomicI32, AtomicU32, AtomicU64, AtomicUsize, Ordering};
use std::sync::Mutex;
use tracing::{debug, trace};

use crate::access;

/// Maximum serial channels supported (hard ceiling of the backing array).
pub const MAX_CHANNELS: usize = 16;

/// How long [`Serial::receive_data_timeout`] parks between RX FIFO polls.
const RX_POLL_INTERVAL_US: u64 = 100;

/// Per-channel UART FIFOs (firmware-facing).
struct SerialFifos {
    tx: [VecDeque<u8>; MAX_CHANNELS],
    rx: [VecDeque<u8>; MAX_CHANNELS],
}

/// UART channel bank for one MCU instance.
pub struct Serial {
    /// Bits clocked per byte on the wire, for baud pacing. Defaults to 10
    /// (8N1: 1 start + 8 data + 1 stop). Configure via [`Serial::set_frame_bits`]
    /// for other UART frames (e.g. 11 for 8E1 / 8N2, 9 for 7N1).
    bits_per_byte: AtomicU64,
    /// Configured channel count.
    count: AtomicUsize,
    /// Optional host-FD backend per channel. -1 = FIFO-only (tests / inject).
    fds: [AtomicI32; MAX_CHANNELS],
    /// Configured baud per channel. 0 = unpaced (instant TX/RX, default).
    baud: [AtomicU32; MAX_CHANNELS],
    /// Next virtual-microsecond at which the TX line is free for this channel.
    /// Advanced atomically per chunk to model serialized UART transmission.
    tx_next_v_us: [AtomicU64; MAX_CHANNELS],
    /// Next virtual-microsecond at which the firmware may consume the next
    /// byte from the RX line. Independent of TX so the link is full-duplex.
    rx_next_v_us: [AtomicU64; MAX_CHANNELS],
    fifos: Mutex<SerialFifos>,
}

impl Serial {
    /// Create a bank with no channels configured and nothing connected.
    pub fn new() -> Self {
        // justification: these `const`s are never read as values; they only
        // seed the `[INIT; N]` array-repeat initializers for the fields below.
        // Array-repeat syntax *requires* a `const`, and no interior mutability
        // is ever observed through the consts themselves.
        #[allow(clippy::declare_interior_mutable_const)]
        const FD_INIT: AtomicI32 = AtomicI32::new(-1);
        #[allow(clippy::declare_interior_mutable_const)]
        const U32_INIT: AtomicU32 = AtomicU32::new(0);
        #[allow(clippy::declare_interior_mutable_const)]
        const U64_INIT: AtomicU64 = AtomicU64::new(0);
        Self {
            bits_per_byte: AtomicU64::new(10),
            count: AtomicUsize::new(0),
            fds: [FD_INIT; MAX_CHANNELS],
            baud: [U32_INIT; MAX_CHANNELS],
            tx_next_v_us: [U64_INIT; MAX_CHANNELS],
            rx_next_v_us: [U64_INIT; MAX_CHANNELS],
            fifos: Mutex::new(SerialFifos {
                tx: std::array::from_fn(|_| VecDeque::new()),
                rx: std::array::from_fn(|_| VecDeque::new()),
            }),
        }
    }

    /// Set the number of bits clocked per byte (used by baud pacing). Default 10.
    pub fn set_frame_bits(&self, bits: u64) {
        self.bits_per_byte.store(bits.max(1), Ordering::Relaxed);
    }

    /// Configure the serial peripheral with the number of channels.
    /// Resets FDs, baud, and pacing schedules, so re-init yields a clean state.
    ///
    /// # Panics
    /// If `count` exceeds [`MAX_CHANNELS`].
    pub fn init(&self, count: usize) {
        assert!(
            count <= MAX_CHANNELS,
            "Serial count {} exceeds max {}",
            count,
            MAX_CHANNELS
        );
        // Sizing and wiring commute: a component (or runtime) may install
        // channel FDs before OR after the firmware sizes the bank — in the
        // owned-execution MCU flow the firmware's own HAL init runs inside
        // the entry, strictly after attach installed the bridges, and must
        // not sever them. Pacing state is still cleared (a re-init restarts
        // the wire clock); full disconnection is `reset()`'s job (teardown).
        for ch in 0..MAX_CHANNELS {
            self.baud[ch].store(0, Ordering::Relaxed);
            self.tx_next_v_us[ch].store(0, Ordering::Relaxed);
            self.rx_next_v_us[ch].store(0, Ordering::Relaxed);
        }
        {
            let mut fifos = self.fifos.lock().unwrap();
            for q in fifos.tx.iter_mut() {
                q.clear();
            }
            for q in fifos.rx.iter_mut() {
                q.clear();
            }
        }
        self.count.store(count, Ordering::Relaxed);
    }

    /// Disconnect all channels and clear baud/pacing/FIFOs (used by `init` and
    /// teardown). Does not close FDs — the owner of the FD (e.g. the PTY) does that.
    pub fn reset(&self) {
        self.count.store(0, Ordering::Relaxed);
        for ch in 0..MAX_CHANNELS {
            self.fds[ch].store(-1, Ordering::Relaxed);
            self.baud[ch].store(0, Ordering::Relaxed);
            self.tx_next_v_us[ch].store(0, Ordering::Relaxed);
            self.rx_next_v_us[ch].store(0, Ordering::Relaxed);
        }
        let mut fifos = self.fifos.lock().unwrap();
        for q in fifos.tx.iter_mut() {
            q.clear();
        }
        for q in fifos.rx.iter_mut() {
            q.clear();
        }
    }

    fn known(&self, channel: usize) -> bool {
        channel < self.count.load(Ordering::Relaxed)
    }

    /// Configured channel count.
    pub fn count(&self) -> usize {
        self.count.load(Ordering::Relaxed)
    }

    /// Host backend: bytes the firmware will `receive`.
    pub fn write_host_rx(&self, channel: usize, data: &[u8]) {
        if !self.known(channel) {
            access::report("serial", &format!("write_host_rx channel {channel}"));
            return;
        }
        self.fifos.lock().unwrap().rx[channel].extend(data.iter().copied());
    }

    /// Host backend: drain bytes the firmware `transmit`ted.
    pub fn take_host_tx(&self, channel: usize, max: usize) -> Vec<u8> {
        if !self.known(channel) {
            access::report("serial", &format!("take_host_tx channel {channel}"));
            return Vec::new();
        }
        let mut tx = self.fifos.lock().unwrap();
        let n = max.min(tx.tx[channel].len());
        tx.tx[channel].drain(..n).collect()
    }

    /// Firmware-facing RX FIFO depth (inspect).
    pub fn rx_len(&self, channel: usize) -> usize {
        if !self.known(channel) {
            return 0;
        }
        self.fifos.lock().unwrap().rx[channel].len()
    }

    /// Firmware-facing TX FIFO depth (inspect).
    pub fn tx_len(&self, channel: usize) -> usize {
        if !self.known(channel) {
            return 0;
        }
        self.fifos.lock().unwrap().tx[channel].len()
    }

    /// Attached backend FD, or -1 if FIFO-only.
    pub fn backend_fd(&self, channel: usize) -> i32 {
        if !self.known(channel) {
            return -1;
        }
        self.fds[channel].load(Ordering::Relaxed)
    }

    pub fn baud(&self, channel: usize) -> u32 {
        if !self.known(channel) {
            return 0;
        }
        self.baud[channel].load(Ordering::Relaxed)
    }

    /// Pull available bytes from the optional FD backend into the RX FIFO.
    fn pull_backend(&self, channel: usize) {
        let fd = self.fds[channel].load(Ordering::Relaxed);
        if fd < 0 {
            return;
        }
        let borrowed = unsafe { BorrowedFd::borrow_raw(fd) };
        let mut buf = [0u8; 256];
        match nix::unistd::read(borrowed, &mut buf) {
            Ok(n) if n > 0 => {
                self.fifos.lock().unwrap().rx[channel].extend(buf[..n].iter().copied());
            }
            _ => {}
        }
    }

    /// Push TX FIFO bytes to the optional FD backend.
    fn flush_backend(&self, channel: usize) {
        let fd = self.fds[channel].load(Ordering::Relaxed);
        if fd < 0 {
            return;
        }
        let pending: Vec<u8> = {
            let mut fifos = self.fifos.lock().unwrap();
            fifos.tx[channel].drain(..).collect()
        };
        if pending.is_empty() {
            return;
        }
        let borrowed = unsafe { BorrowedFd::borrow_raw(fd) };
        let mut written = 0;
        while written < pending.len() {
            match nix::unistd::write(borrowed, &pending[written..]) {
                Ok(n) => written += n,
                Err(nix::errno::Errno::EAGAIN) => {
                    // Put the rest back; a later transmit/flush will retry.
                    self.fifos.lock().unwrap().tx[channel]
                        .extend(pending[written..].iter().copied());
                    break;
                }
                Err(e) => {
                    trace!(
                        "serial flush_backend(channel={}): write error: {}",
                        channel,
                        e
                    );
                    break;
                }
            }
        }
    }

    /// Initialize a serial channel with a file descriptor.
    pub fn init_channel_fd(&self, channel: usize, fd: RawFd) {
        if channel < MAX_CHANNELS {
            self.fds[channel].store(fd, Ordering::Relaxed);
            debug!("Serial channel {} initialized with fd={}", channel, fd);
        }
    }

    /// Configure deterministic baud-rate pacing for a channel (full-duplex).
    ///
    /// `baud == 0` disables pacing (instant TX/RX, default). Any positive value
    /// enables both directions: every `transmit_data` and every successful read
    /// reserves a slot on its respective TX or RX schedule and blocks (in wall
    /// time) for the virtual duration a real UART would spend clocking those
    /// bytes (`bytes * 10 / baud` seconds at 8N1).
    ///
    /// All scheduling decisions read `embsim_core::virtual_clock`, so timing is
    /// reproducible across runs and scales correctly with `--speed`.
    ///
    /// Calling `set_baud` resets both TX and RX schedules for the channel.
    pub fn set_baud(&self, channel: usize, baud: u32) {
        if channel >= MAX_CHANNELS {
            return;
        }
        self.baud[channel].store(baud, Ordering::Relaxed);
        self.tx_next_v_us[channel].store(0, Ordering::Relaxed);
        self.rx_next_v_us[channel].store(0, Ordering::Relaxed);
        if baud == 0 {
            debug!("Serial channel {} baud pacing disabled", channel);
        } else {
            debug!(
                "Serial channel {} baud pacing enabled at {} bps ({} us/byte, full-duplex)",
                channel,
                baud,
                self.bits_per_byte.load(Ordering::Relaxed) * 1_000_000 / baud as u64
            );
        }
    }

    /// Reserve a slot of `n` bytes on the given direction's schedule and block
    /// (in wall time) for the equivalent virtual duration. No-op when `baud` is 0.
    fn pace_bytes(&self, slot: &AtomicU64, baud: u32, n: usize) {
        if n == 0 || baud == 0 {
            return;
        }

        let bits_per_byte = self.bits_per_byte.load(Ordering::Relaxed);
        let cost_v_us = (n as u64).saturating_mul(bits_per_byte * 1_000_000) / baud as u64;
        if cost_v_us == 0 {
            return;
        }

        let now_v = embsim_core::virtual_clock::virtual_us();

        let mut current = slot.load(Ordering::Relaxed);
        let end_v = loop {
            let start_v = current.max(now_v);
            let end_v = start_v.saturating_add(cost_v_us);
            match slot.compare_exchange_weak(current, end_v, Ordering::Relaxed, Ordering::Relaxed) {
                Ok(_) => break end_v,
                Err(actual) => current = actual,
            }
        };

        // The reserved slot end is an absolute virtual deadline, so this is
        // the `wait_until` form (`DETERMINISM.md` T1 §5). Re-reading "now"
        // inside `wait_until` can only shorten the sleep by however long the
        // CAS above took, which lands the caller *closer* to `end_v` than the
        // old `end_v - now_v` span did.
        embsim_core::virtual_clock::wait_until(end_v);
    }

    /// Reserve a TX slot of `n` bytes; sleeps to model the firmware blocking
    /// while the UART clocks the bytes out.
    fn pace_tx(&self, channel: usize, n: usize) {
        if channel >= MAX_CHANNELS {
            return;
        }
        let baud = self.baud[channel].load(Ordering::Relaxed);
        self.pace_bytes(&self.tx_next_v_us[channel], baud, n);
    }

    /// Reserve an RX slot of `n` bytes; called after a successful read so the
    /// firmware can never consume bytes faster than the wire could deliver them.
    fn pace_rx(&self, channel: usize, n: usize) {
        if channel >= MAX_CHANNELS {
            return;
        }
        let baud = self.baud[channel].load(Ordering::Relaxed);
        self.pace_bytes(&self.rx_next_v_us[channel], baud, n);
    }

    /// Start a serial channel (no-op in emulation, channels are FD-based).
    pub fn start(&self, channel: usize) {
        trace!("serial::start(channel={})", channel);
    }

    /// Stop a serial channel (no-op in emulation).
    pub fn stop(&self, channel: usize) {
        trace!("serial::stop(channel={})", channel);
    }

    /// Transmit data on a serial channel (UART TX FIFO, then optional FD backend).
    pub fn transmit_data(&self, channel: usize, data: &[u8]) {
        if data.is_empty() {
            return;
        }
        if !self.known(channel) {
            access::report("serial", &format!("transmit channel {channel}"));
            return;
        }

        self.pace_tx(channel, data.len());
        self.fifos.lock().unwrap().tx[channel].extend(data.iter().copied());
        self.flush_backend(channel);
        trace!(
            "serial::transmit_data(channel={}, len={})",
            channel,
            data.len()
        );
    }

    /// Receive data with a timeout (virtual microseconds).
    /// Returns true if all `buf.len()` bytes were received before timeout.
    pub fn receive_data_timeout(&self, channel: usize, buf: &mut [u8], timeout_us: u64) -> bool {
        if buf.is_empty() {
            embsim_core::virtual_clock::wait_virtual_us(timeout_us);
            return false;
        }
        if !self.known(channel) {
            access::report("serial", &format!("receive_timeout channel {channel}"));
            embsim_core::virtual_clock::wait_virtual_us(timeout_us);
            return false;
        }

        let deadline_v_us = embsim_core::virtual_clock::virtual_us().saturating_add(timeout_us);
        let mut total_read = 0;

        while total_read < buf.len() {
            self.pull_backend(channel);
            let got = {
                let mut fifos = self.fifos.lock().unwrap();
                let mut n = 0;
                while total_read + n < buf.len() {
                    match fifos.rx[channel].pop_front() {
                        Some(b) => {
                            buf[total_read + n] = b;
                            n += 1;
                        }
                        None => break,
                    }
                }
                n
            };
            if got > 0 {
                total_read += got;
                self.pace_rx(channel, got);
            }
            if total_read >= buf.len() {
                return true;
            }
            let now = embsim_core::virtual_clock::virtual_us();
            if now >= deadline_v_us {
                break;
            }
            let step = RX_POLL_INTERVAL_US.min(deadline_v_us.saturating_sub(now));
            embsim_core::virtual_clock::wait_virtual_us(step);
        }

        total_read == buf.len()
    }

    /// Receive up to `buf.len()` bytes in a single non-blocking read.
    ///
    /// Returns the number of bytes read (0 if none were available). Like
    /// [`Serial::receive_byte`], a successful read is paced to the configured
    /// baud so the firmware can never consume bytes faster than the wire would
    /// deliver them.
    pub fn receive_bytes(&self, channel: usize, buf: &mut [u8]) -> usize {
        if buf.is_empty() {
            return 0;
        }
        if !self.known(channel) {
            access::report("serial", &format!("receive_bytes channel {channel}"));
            return 0;
        }
        self.pull_backend(channel);
        let mut n = 0;
        {
            let mut fifos = self.fifos.lock().unwrap();
            while n < buf.len() {
                match fifos.rx[channel].pop_front() {
                    Some(b) => {
                        buf[n] = b;
                        n += 1;
                    }
                    None => break,
                }
            }
        }
        if n > 0 {
            self.pace_rx(channel, n);
        }
        n
    }

    /// Receive a single byte (non-blocking).
    /// Returns Some(byte) if a byte was available, None otherwise.
    pub fn receive_byte(&self, channel: usize) -> Option<u8> {
        if !self.known(channel) {
            access::report("serial", &format!("receive_byte channel {channel}"));
            return None;
        }
        self.pull_backend(channel);
        let byte = self.fifos.lock().unwrap().rx[channel].pop_front();
        if byte.is_some() {
            self.pace_rx(channel, 1);
        }
        byte
    }
}

impl Default for Serial {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================
// Free functions — route to the calling thread's instance
// ============================================================

/// Set the number of bits clocked per byte (used by baud pacing). Default 10.
pub fn set_frame_bits(bits: u64) {
    crate::instance::current().serial.set_frame_bits(bits);
}

/// Configure the serial peripheral with the number of channels.
/// Resets FDs, baud, and pacing schedules, so re-init yields a clean state.
pub fn init(count: usize) {
    crate::instance::current().serial.init(count);
}

/// Disconnect all channels and clear baud/pacing state (used by `init` and
/// teardown). Does not close FDs — the owner of the FD (e.g. the PTY) does that.
pub fn reset() {
    crate::instance::current().serial.reset();
}

/// Initialize a serial channel with a file descriptor.
pub fn init_channel_fd(channel: usize, fd: RawFd) {
    crate::instance::current()
        .serial
        .init_channel_fd(channel, fd);
}

/// Host backend: inject bytes the firmware will receive (no FD required).
pub fn write_host_rx(channel: usize, data: &[u8]) {
    crate::instance::current()
        .serial
        .write_host_rx(channel, data);
}

/// Host backend: drain bytes the firmware transmitted (no FD required).
pub fn take_host_tx(channel: usize, max: usize) -> Vec<u8> {
    crate::instance::current().serial.take_host_tx(channel, max)
}

/// Configure deterministic baud-rate pacing for a channel (full-duplex).
/// See [`Serial::set_baud`].
pub fn set_baud(channel: usize, baud: u32) {
    crate::instance::current().serial.set_baud(channel, baud);
}

/// Start a serial channel (no-op in emulation, channels are FD-based).
pub fn start(channel: usize) {
    crate::instance::current().serial.start(channel);
}

/// Stop a serial channel (no-op in emulation).
pub fn stop(channel: usize) {
    crate::instance::current().serial.stop(channel);
}

/// Transmit data on a serial channel.
pub fn transmit_data(channel: usize, data: &[u8]) {
    crate::instance::current()
        .serial
        .transmit_data(channel, data);
}

/// Receive data with a timeout (virtual microseconds).
/// Returns true if all `len` bytes were received before timeout.
pub fn receive_data_timeout(channel: usize, buf: &mut [u8], timeout_us: u64) -> bool {
    crate::instance::current()
        .serial
        .receive_data_timeout(channel, buf, timeout_us)
}

/// Receive up to `buf.len()` bytes in a single non-blocking read.
/// See [`Serial::receive_bytes`].
pub fn receive_bytes(channel: usize, buf: &mut [u8]) -> usize {
    crate::instance::current()
        .serial
        .receive_bytes(channel, buf)
}

/// Receive a single byte (non-blocking).
/// Returns Some(byte) if a byte was available, None otherwise.
pub fn receive_byte(channel: usize) -> Option<u8> {
    crate::instance::current().serial.receive_byte(channel)
}

// ============================================================
// Tests
// ============================================================

#[cfg(test)]
mod tests {
    use rstest::rstest;

    use super::*;
    use nix::libc;

    /// A connected pair of file descriptors. The first (`a`) is wired into a
    /// serial channel; the second (`b`) is the "other end of the wire" the test
    /// reads from / writes to. Both are closed on drop.
    struct Pair {
        a: RawFd,
        b: RawFd,
    }

    impl Pair {
        /// Build a connected AF_UNIX stream socket pair. BOTH ends are made
        /// non-blocking so a read on an empty buffer returns EAGAIN instead of
        /// hanging — a blocking read here would also stall the crate-wide test
        /// lock and cascade-hang every other test.
        fn new() -> Self {
            let mut fds = [0i32; 2];
            let rc =
                unsafe { libc::socketpair(libc::AF_UNIX, libc::SOCK_STREAM, 0, fds.as_mut_ptr()) };
            assert_eq!(rc, 0, "socketpair failed");
            for &fd in &fds {
                let flags = unsafe { libc::fcntl(fd, libc::F_GETFL) };
                let rc = unsafe { libc::fcntl(fd, libc::F_SETFL, flags | libc::O_NONBLOCK) };
                assert_eq!(rc, 0, "fcntl O_NONBLOCK failed");
            }
            Pair {
                a: fds[0],
                b: fds[1],
            }
        }

        /// Write raw bytes to the far end so the channel can read them.
        fn write_far(&self, data: &[u8]) {
            let fd = unsafe { BorrowedFd::borrow_raw(self.b) };
            let mut off = 0;
            while off < data.len() {
                off += nix::unistd::write(fd, &data[off..]).expect("write_far");
            }
        }

        /// Read up to `n` bytes from the far end (what the channel transmitted).
        /// Non-blocking: returns an empty vec when nothing is buffered, so a test
        /// asserting "nothing was sent" can never hang on an empty pipe. (Bytes
        /// written to an AF_UNIX stream are available to the peer synchronously
        /// once the writer's `write` returns, so the data cases never race.)
        fn read_far(&self, n: usize) -> Vec<u8> {
            let mut buf = vec![0u8; n];
            let fd = unsafe { BorrowedFd::borrow_raw(self.b) };
            match nix::unistd::read(fd, &mut buf) {
                Ok(read) => {
                    buf.truncate(read);
                    buf
                }
                Err(nix::errno::Errno::EAGAIN) => Vec::new(),
                Err(e) => panic!("read_far: {e}"),
            }
        }
    }

    impl Drop for Pair {
        fn drop(&mut self) {
            unsafe {
                libc::close(self.a);
                libc::close(self.b);
            }
        }
    }

    /// Take the crate test lock, pin the clock, and reset the serial bank.
    fn setup(count: usize) {
        crate::test_support::ensure_clock();
        init(count);
    }

    #[rstest]
    fn init_at_max_channels_is_allowed() {
        let _g = crate::test_support::guard();
        setup(MAX_CHANNELS);
        // After init, every channel is disconnected (fd -1).
        assert!(receive_byte(MAX_CHANNELS - 1).is_none());
    }

    #[rstest]
    #[should_panic(expected = "exceeds max")]
    fn init_above_max_channels_panics() {
        let _g = crate::test_support::guard();
        crate::test_support::ensure_clock();
        init(MAX_CHANNELS + 1);
    }

    #[rstest]
    fn reset_sets_all_fds_to_minus_one_and_clears_pacing() {
        let _g = crate::test_support::guard();
        let pair = Pair::new();
        setup(2);
        init_channel_fd(0, pair.a);
        set_baud(0, 9600);
        reset();
        // After reset the channel is unknown (count 0) and disconnected, so a
        // transmit is a no-op and a receive returns None.
        transmit_data(0, b"x");
        assert!(receive_byte(0).is_none());
    }

    #[rstest]
    fn init_channel_fd_stores_the_fd_and_enables_io() {
        let _g = crate::test_support::guard();
        let pair = Pair::new();
        setup(1);
        init_channel_fd(0, pair.a);
        // With a real fd wired in, a transmit reaches the far end.
        transmit_data(0, b"hi");
        assert_eq!(pair.read_far(2), b"hi");
    }

    #[rstest]
    fn transmit_to_unconnected_channel_is_a_no_op() {
        let _g = crate::test_support::guard();
        setup(1);
        // Channel 0 configured but fd still -1: transmit silently discards.
        transmit_data(0, b"data");
        // Nothing to assert beyond "did not panic / did not block".
    }

    #[rstest]
    fn transmit_empty_data_is_a_no_op() {
        let _g = crate::test_support::guard();
        let pair = Pair::new();
        setup(1);
        init_channel_fd(0, pair.a);
        transmit_data(0, b"");
        // Far end received nothing.
        let got = pair.read_far(1);
        assert!(got.is_empty(), "empty transmit writes nothing");
    }

    #[rstest]
    fn transmit_out_of_range_channel_is_a_no_op() {
        let _g = crate::test_support::guard();
        setup(1);
        // Channel 5 is past the configured count: no panic, nothing sent.
        transmit_data(5, b"data");
    }

    #[rstest]
    fn transmit_then_read_round_trips_bytes() {
        let _g = crate::test_support::guard();
        let pair = Pair::new();
        setup(1);
        init_channel_fd(0, pair.a);
        transmit_data(0, b"hi");
        assert_eq!(pair.read_far(2), b"hi");
    }

    #[rstest]
    fn fifo_host_backend_round_trips_without_an_fd() {
        let _g = crate::test_support::guard();
        setup(1);
        write_host_rx(0, b"ab");
        assert_eq!(crate::instance::current().serial.rx_len(0), 2);
        assert_eq!(receive_byte(0), Some(b'a'));
        assert_eq!(receive_byte(0), Some(b'b'));
        assert_eq!(receive_byte(0), None);

        transmit_data(0, b"xy");
        assert_eq!(take_host_tx(0, 8), b"xy");
        assert_eq!(take_host_tx(0, 8), b"");
    }

    #[rstest]
    fn host_tester_byte_is_visible_by_a_virtual_deadline() {
        let _g = crate::test_support::guard();
        crate::test_support::ensure_clock();
        crate::access::take_count();
        setup(1);
        let t_inject = embsim_core::virtual_clock::virtual_us() + 1_000;
        std::thread::spawn(move || {
            embsim_core::virtual_clock::wait_until(t_inject);
            write_host_rx(0, b"K");
        });
        let mut buf = [0u8; 1];
        assert!(
            receive_data_timeout(0, &mut buf, 5_000),
            "byte must arrive by the virtual deadline"
        );
        assert_eq!(buf[0], b'K');
    }

    #[rstest]
    fn receive_byte_returns_available_then_none_when_empty() {
        let _g = crate::test_support::guard();
        let pair = Pair::new();
        setup(1);
        init_channel_fd(0, pair.a);
        pair.write_far(&[0x5A]);
        assert_eq!(receive_byte(0), Some(0x5A));
        // Non-blocking fd with nothing left → None.
        assert_eq!(receive_byte(0), None);
    }

    #[rstest]
    fn receive_byte_on_unconnected_or_out_of_range_is_none() {
        let _g = crate::test_support::guard();
        setup(1);
        // Configured but disconnected (fd -1).
        assert_eq!(receive_byte(0), None);
        // Out-of-range channel.
        assert_eq!(receive_byte(9), None);
    }

    #[rstest]
    fn receive_bytes_burst_drains_then_zero_when_empty() {
        let _g = crate::test_support::guard();
        let pair = Pair::new();
        setup(1);
        init_channel_fd(0, pair.a);
        pair.write_far(b"hello");
        let mut buf = [0u8; 8];
        // One call drains all available bytes (the firmware's burst receive).
        assert_eq!(receive_bytes(0, &mut buf), 5);
        assert_eq!(&buf[..5], b"hello");
        // Nothing left → 0.
        assert_eq!(receive_bytes(0, &mut buf), 0);
    }

    #[rstest]
    fn receive_bytes_clamps_to_buffer_len() {
        let _g = crate::test_support::guard();
        let pair = Pair::new();
        setup(1);
        init_channel_fd(0, pair.a);
        pair.write_far(b"abcdef");
        // Buffer smaller than what's waiting: read at most buf.len() this call.
        let mut small = [0u8; 4];
        assert_eq!(receive_bytes(0, &mut small), 4);
        assert_eq!(&small, b"abcd");
        // The remainder is still readable on the next call.
        let mut rest = [0u8; 4];
        assert_eq!(receive_bytes(0, &mut rest), 2);
        assert_eq!(&rest[..2], b"ef");
    }

    #[rstest]
    fn receive_bytes_unconnected_out_of_range_or_empty_buf_is_zero() {
        let _g = crate::test_support::guard();
        setup(1);
        // Configured but disconnected (fd -1).
        let mut buf = [0u8; 4];
        assert_eq!(receive_bytes(0, &mut buf), 0);
        // Out-of-range channel.
        assert_eq!(receive_bytes(9, &mut buf), 0);
        // Empty output buffer with a connected fd → 0 (and reads nothing).
        let pair = Pair::new();
        init_channel_fd(0, pair.a);
        let mut empty: [u8; 0] = [];
        assert_eq!(receive_bytes(0, &mut empty), 0);
    }

    #[rstest]
    fn receive_data_timeout_fills_buffer_when_bytes_ready() {
        let _g = crate::test_support::guard();
        let pair = Pair::new();
        setup(1);
        init_channel_fd(0, pair.a);
        pair.write_far(b"abcd");
        let mut buf = [0u8; 4];
        // All 4 bytes ready → returns true and fills the buffer.
        assert!(receive_data_timeout(0, &mut buf, 200));
        assert_eq!(&buf, b"abcd");
    }

    #[rstest]
    fn receive_data_timeout_returns_false_when_short() {
        let _g = crate::test_support::guard();
        let pair = Pair::new();
        setup(1);
        init_channel_fd(0, pair.a);
        // Only 2 of 4 requested bytes available, tiny timeout → false.
        pair.write_far(b"ab");
        let mut buf = [0u8; 4];
        assert!(!receive_data_timeout(0, &mut buf, 200));
        // The bytes that did arrive landed at the front of the buffer.
        assert_eq!(&buf[..2], b"ab");
    }

    /// A blocking receive must PARK the virtual clock while it waits, not spin
    /// on wall time.
    ///
    /// The caller is a registered actor (firmware cogs are), and virtual time
    /// only advances once every actor is parked. A wall-clock wait therefore
    /// stops the clock for its whole duration — so a peer that answers from its
    /// own virtual deadline can never run, and the read waits for a reply that
    /// the waiting itself prevents. This models exactly that: the peer writes
    /// only after `wait_virtual_us`, which is what every device-model protocol
    /// thread in the workspace does between poll iterations.
    ///
    /// This is a deadlock by construction, and it hid for a long time because
    /// it resolved by race — where the peer happened to still be mid-iteration
    /// when the request landed, it answered before parking. On a host that
    /// scheduled the caller first it failed every time: every ADS122U04
    /// register read-back timed out on macOS while passing on Linux CI, from
    /// identical code.
    #[rstest]
    fn receive_data_timeout_lets_a_parked_peer_answer() {
        let _g = crate::test_support::guard();
        let pair = Pair::new();
        setup(1);
        init_channel_fd(0, pair.a);

        // This is what makes the wait load-bearing: while this thread counts as
        // running, the scheduler may not advance.
        let reader_actor = embsim_core::virtual_clock::register_actor("reader-cog");

        let stop = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
        let peer_stop = std::sync::Arc::clone(&stop);
        let far = pair.b;
        let peer = std::thread::spawn(move || {
            let _actor = embsim_core::virtual_clock::register_actor("device-model");
            embsim_core::virtual_clock::wait_virtual_us(250);
            let fd = unsafe { BorrowedFd::borrow_raw(far) };
            let _ = nix::unistd::write(fd, b"R");
            // Keep parking rather than returning. The idle jump is performed by
            // a thread on its way *into* a park, so an actor that exits while it
            // is the last one running leaves nobody to advance the clock to the
            // reader's next deadline. Device-model threads loop forever; this
            // mimics that until the test releases it.
            while !peer_stop.load(std::sync::atomic::Ordering::Relaxed) {
                embsim_core::virtual_clock::wait_virtual_us(1_000);
            }
        });

        let mut buf = [0u8; 1];
        let got = receive_data_timeout(0, &mut buf, 5_000);

        // Teardown has to terminate on the regression path too, or the suite
        // hangs in join() instead of reporting the assertion. Three steps, in
        // this order: ask the peer to stop; stop counting as a running actor so
        // its future parks can self-advance (park_until_virtual idle-jumps when
        // it is the last one running); then bump the clock epoch, which releases
        // a peer that is *already* parked and waiting.
        stop.store(true, std::sync::atomic::Ordering::Relaxed);
        drop(reader_actor);
        embsim_core::virtual_clock::init(1.0, 180_000_000);
        peer.join().expect("peer thread");

        assert!(
            got,
            "receive timed out: the peer could not run while the reader waited"
        );
        assert_eq!(&buf, b"R");
    }

    #[rstest]
    fn receive_data_timeout_empty_buf_or_unknown_channel_is_false() {
        let _g = crate::test_support::guard();
        let pair = Pair::new();
        setup(1);
        init_channel_fd(0, pair.a);
        // Empty buffer → false immediately.
        let mut empty: [u8; 0] = [];
        assert!(!receive_data_timeout(0, &mut empty, 100));
        // Unknown (out-of-range) channel → false.
        let mut buf = [0u8; 1];
        assert!(!receive_data_timeout(9, &mut buf, 100));
        // Connected-but-disconnected channel (fd -1) → false.
        init(2); // channel 1 configured but fd -1
        init_channel_fd(0, pair.a);
        let mut buf2 = [0u8; 1];
        assert!(!receive_data_timeout(1, &mut buf2, 100));
    }

    #[rstest]
    fn set_frame_bits_clamps_to_at_least_one() {
        let _g = crate::test_support::guard();
        let pair = Pair::new();
        setup(1);
        init_channel_fd(0, pair.a);
        // 0 bits would divide-by-zero in pacing; it clamps to 1. Bytes still flow.
        set_frame_bits(0);
        set_baud(0, 1_000_000);
        transmit_data(0, b"z");
        assert_eq!(pair.read_far(1), b"z");
        // Restore the conventional 8N1 framing for subsequent tests.
        set_frame_bits(10);
    }

    #[rstest]
    fn paced_baud_still_delivers_bytes_correctly() {
        let _g = crate::test_support::guard();
        let pair = Pair::new();
        setup(1);
        init_channel_fd(0, pair.a);
        // Enable pacing. We do NOT assert timing — only that the bytes arrive
        // intact through the paced path. Use a fast baud so any sleep is sub-ms.
        set_baud(0, 1_000_000);
        transmit_data(0, b"hi");
        assert_eq!(pair.read_far(2), b"hi");
        // Receive path is also paced and must still deliver the byte.
        pair.write_far(&[0x42]);
        assert_eq!(receive_byte(0), Some(0x42));
    }

    #[rstest]
    fn set_baud_zero_is_unpaced_and_resets_schedules() {
        let _g = crate::test_support::guard();
        let pair = Pair::new();
        setup(1);
        init_channel_fd(0, pair.a);
        // Turn pacing on then back off; either way bytes deliver correctly.
        set_baud(0, 1_000_000);
        set_baud(0, 0); // back to unpaced — also resets TX/RX schedules
        transmit_data(0, b"ok");
        assert_eq!(pair.read_far(2), b"ok");
    }

    #[rstest]
    fn set_baud_out_of_range_channel_is_a_no_op() {
        let _g = crate::test_support::guard();
        setup(1);
        // Channel index past the array ceiling: silently ignored, no panic.
        set_baud(MAX_CHANNELS, 9600);
    }

    #[rstest]
    fn start_and_stop_are_no_ops() {
        let _g = crate::test_support::guard();
        let pair = Pair::new();
        setup(1);
        init_channel_fd(0, pair.a);
        // No-ops: must not disturb the channel or panic.
        start(0);
        stop(0);
        transmit_data(0, b"x");
        assert_eq!(pair.read_far(1), b"x");
    }

    /// Paced TX of `N` bytes at baud `B` with `F` frame bits must block for
    /// approximately `N * F * 1e6 / B` virtual microseconds (scale 1.0 ⇒ wall).
    ///
    /// We assert a lower bound of 50% of the theoretical cost (scheduler slack
    /// can only make sleeps shorter under load, never invent free time) and a
    /// generous upper bound so CI hosts with jitter still pass.
    #[rstest]
    #[case::ten_kbaud_2b(10_000, 10, 2, 2_000)]
    #[case::twenty_kbaud_5b(20_000, 10, 5, 2_500)]
    #[case::frame_11(10_000, 11, 2, 2_200)]
    fn paced_tx_blocks_for_expected_virtual_us(
        #[case] baud: u32,
        #[case] frame_bits: u64,
        #[case] nbytes: usize,
        #[case] expected_v_us: u64,
    ) {
        let _g = crate::test_support::guard();
        let pair = Pair::new();
        setup(1);
        init_channel_fd(0, pair.a);
        set_frame_bits(frame_bits);
        set_baud(0, baud);

        let payload = vec![0xA5u8; nbytes];
        let t0 = std::time::Instant::now();
        transmit_data(0, &payload);
        let wall_us = t0.elapsed().as_micros() as u64;

        assert_eq!(pair.read_far(nbytes), payload.as_slice());
        assert!(
            wall_us >= expected_v_us / 2,
            "paced TX too fast: wall={wall_us}us expected≥{}us (baud={baud} F={frame_bits} N={nbytes})",
            expected_v_us / 2
        );
        assert!(
            wall_us <= expected_v_us.saturating_mul(8).saturating_add(20_000),
            "paced TX too slow: wall={wall_us}us expected≤{}us",
            expected_v_us.saturating_mul(8).saturating_add(20_000)
        );
    }

    /// RX pacing is independent of TX (full duplex): a paced receive after a
    /// paced transmit still delivers the byte and incurs its own schedule cost.
    #[rstest]
    fn paced_rx_is_independent_full_duplex() {
        let _g = crate::test_support::guard();
        let pair = Pair::new();
        setup(1);
        init_channel_fd(0, pair.a);
        set_frame_bits(10);
        set_baud(0, 50_000); // 200 us/byte — measurable but snappy

        // TX half: one byte.
        let t_tx = std::time::Instant::now();
        transmit_data(0, b"T");
        let tx_us = t_tx.elapsed().as_micros() as u64;
        assert_eq!(pair.read_far(1), b"T");
        assert!(tx_us >= 100, "TX half should sleep ~200us, got {tx_us}");

        // RX half: one byte from the far end.
        pair.write_far(b"R");
        let t_rx = std::time::Instant::now();
        assert_eq!(receive_byte(0), Some(b'R'));
        let rx_us = t_rx.elapsed().as_micros() as u64;
        assert!(rx_us >= 100, "RX half should sleep ~200us, got {rx_us}");
    }

    /// Frame-bits / baud matrix: bytes always land intact under pacing.
    #[rstest]
    #[case::eight_n1(10, 100_000)]
    #[case::eight_e1(11, 100_000)]
    #[case::seven_n1(9, 100_000)]
    fn paced_bytes_round_trip_for_frame_and_baud(#[case] frame_bits: u64, #[case] baud: u32) {
        let _g = crate::test_support::guard();
        let pair = Pair::new();
        setup(1);
        init_channel_fd(0, pair.a);
        set_frame_bits(frame_bits);
        set_baud(0, baud);
        transmit_data(0, b"xy");
        assert_eq!(pair.read_far(2), b"xy");
        pair.write_far(b"z");
        assert_eq!(receive_byte(0), Some(b'z'));
    }
}
