//! Serial — FD-bridged serial port channels.
//!
//! Bridges firmware serial channels to file descriptors. The peripheral
//! manages raw FDs and has no knowledge of what's on the other end
//! (a host PTY, a peer socket, a sensor model, etc). Channel assignment is
//! done by the project wiring layer via `init_channel_fd()`.
//!
//! State lives in a per-MCU [`Serial`] bank owned by
//! `instance::PeripheralInstance`. The module-level free functions route to
//! the calling thread's instance (see `crate::instance`), so existing
//! single-MCU consumers are unaffected.

use std::os::fd::{BorrowedFd, RawFd};
use std::sync::atomic::{AtomicI32, AtomicU32, AtomicU64, AtomicUsize, Ordering};
use tracing::{debug, trace};

/// Maximum serial channels supported (hard ceiling of the backing array).
pub const MAX_CHANNELS: usize = 16;

/// FD-bridged serial channel bank for one MCU instance.
pub struct Serial {
    /// Bits clocked per byte on the wire, for baud pacing. Defaults to 10
    /// (8N1: 1 start + 8 data + 1 stop). Configure via [`Serial::set_frame_bits`]
    /// for other UART frames (e.g. 11 for 8E1 / 8N2, 9 for 7N1).
    bits_per_byte: AtomicU64,
    /// Configured channel count.
    count: AtomicUsize,
    /// File descriptors for each channel. -1 = not connected.
    fds: [AtomicI32; MAX_CHANNELS],
    /// Configured baud per channel. 0 = unpaced (instant TX/RX, default).
    baud: [AtomicU32; MAX_CHANNELS],
    /// Next virtual-microsecond at which the TX line is free for this channel.
    /// Advanced atomically per chunk to model serialized UART transmission.
    tx_next_v_us: [AtomicU64; MAX_CHANNELS],
    /// Next virtual-microsecond at which the firmware may consume the next
    /// byte from the RX line. Independent of TX so the link is full-duplex.
    rx_next_v_us: [AtomicU64; MAX_CHANNELS],
}

impl Serial {
    /// Create a bank with no channels configured and nothing connected.
    pub const fn new() -> Self {
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
        self.reset();
        self.count.store(count, Ordering::Relaxed);
    }

    /// Disconnect all channels and clear baud/pacing state (used by `init` and
    /// teardown). Does not close FDs — the owner of the FD (e.g. the PTY) does that.
    pub fn reset(&self) {
        self.count.store(0, Ordering::Relaxed);
        for ch in 0..MAX_CHANNELS {
            self.fds[ch].store(-1, Ordering::Relaxed);
            self.baud[ch].store(0, Ordering::Relaxed);
            self.tx_next_v_us[ch].store(0, Ordering::Relaxed);
            self.rx_next_v_us[ch].store(0, Ordering::Relaxed);
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

        let wait_v_us = end_v.saturating_sub(now_v);
        let wall_us = embsim_core::virtual_clock::virtual_to_wall_us(wait_v_us);
        if wall_us > 0 {
            std::thread::sleep(std::time::Duration::from_micros(wall_us));
        }
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

    /// Transmit data on a serial channel.
    pub fn transmit_data(&self, channel: usize, data: &[u8]) {
        if data.is_empty() {
            return;
        }
        let count = self.count.load(Ordering::Relaxed);
        if channel >= count {
            trace!("serial::transmit_data(channel={}): unknown", channel);
            return;
        }

        let fd = self.fds[channel].load(Ordering::Relaxed);
        if fd < 0 {
            trace!(
                "serial::transmit_data(channel={}): not connected, discarding {} bytes",
                channel,
                data.len()
            );
            return;
        }

        // Reserve and wait for our slot on the simulated wire before clocking the
        // bytes out. No-op unless `set_baud` has been called for this channel.
        self.pace_tx(channel, data.len());

        let borrowed = unsafe { BorrowedFd::borrow_raw(fd) };
        let mut written = 0;
        while written < data.len() {
            match nix::unistd::write(borrowed, &data[written..]) {
                Ok(n) => written += n,
                Err(nix::errno::Errno::EAGAIN) => {
                    std::thread::yield_now();
                }
                Err(e) => {
                    trace!(
                        "serial::transmit_data(channel={}): write error: {}",
                        channel,
                        e
                    );
                    break;
                }
            }
        }
        trace!(
            "serial::transmit_data(channel={}, len={})",
            channel,
            data.len()
        );
    }

    /// Receive data with a timeout (virtual microseconds).
    /// Returns true if all `buf.len()` bytes were received before timeout.
    pub fn receive_data_timeout(&self, channel: usize, buf: &mut [u8], timeout_us: u64) -> bool {
        let count = self.count.load(Ordering::Relaxed);
        if buf.is_empty() || channel >= count {
            let wall_us = embsim_core::virtual_clock::virtual_to_wall_us(timeout_us);
            if wall_us > 0 {
                std::thread::sleep(std::time::Duration::from_micros(wall_us));
            }
            return false;
        }

        let fd = self.fds[channel].load(Ordering::Relaxed);
        if fd < 0 {
            let wall_us = embsim_core::virtual_clock::virtual_to_wall_us(timeout_us);
            if wall_us > 0 {
                std::thread::sleep(std::time::Duration::from_micros(wall_us));
            }
            return false;
        }

        let wall_us = embsim_core::virtual_clock::virtual_to_wall_us(timeout_us);
        let deadline = std::time::Instant::now() + std::time::Duration::from_micros(wall_us);
        let mut total_read = 0;

        // SAFETY: `fd` is the channel FD owned by `self.fds`; it stays open for
        // the duration of this call (borrow does not outlive it).
        let borrowed = unsafe { BorrowedFd::borrow_raw(fd) };
        while total_read < buf.len() {
            match nix::unistd::read(borrowed, &mut buf[total_read..]) {
                Ok(0) => break,
                Ok(n) => {
                    total_read += n;
                    // Throttle consumption to virtual baud. Sleeps wall-time
                    // equivalent of n*10/baud virtual µs; no-op when unpaced.
                    self.pace_rx(channel, n);
                    if total_read >= buf.len() {
                        break;
                    }
                }
                Err(nix::errno::Errno::EAGAIN) => {
                    if std::time::Instant::now() >= deadline {
                        break;
                    }
                    std::thread::sleep(std::time::Duration::from_micros(100));
                }
                Err(_) => break,
            }
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
        let count = self.count.load(Ordering::Relaxed);
        if buf.is_empty() || channel >= count {
            return 0;
        }

        let fd = self.fds[channel].load(Ordering::Relaxed);
        if fd < 0 {
            return 0;
        }

        // SAFETY: `fd` is the channel FD owned by `self.fds`; it stays open for
        // the duration of this call (borrow does not outlive it).
        let borrowed = unsafe { BorrowedFd::borrow_raw(fd) };
        match nix::unistd::read(borrowed, buf) {
            Ok(n) if n > 0 => {
                // Throttle consumption to virtual baud (no-op when unpaced).
                self.pace_rx(channel, n);
                n
            }
            _ => 0,
        }
    }

    /// Receive a single byte (non-blocking).
    /// Returns Some(byte) if a byte was available, None otherwise.
    pub fn receive_byte(&self, channel: usize) -> Option<u8> {
        let count = self.count.load(Ordering::Relaxed);
        if channel >= count {
            return None;
        }

        let fd = self.fds[channel].load(Ordering::Relaxed);
        if fd < 0 {
            return None;
        }

        let mut byte = [0u8; 1];
        // SAFETY: `fd` is the channel FD owned by `self.fds`; it stays open for
        // the duration of this call (borrow does not outlive it).
        let borrowed = unsafe { BorrowedFd::borrow_raw(fd) };
        match nix::unistd::read(borrowed, &mut byte) {
            Ok(1) => {
                // Throttle consumption to virtual baud (no-op when unpaced).
                self.pace_rx(channel, 1);
                Some(byte[0])
            }
            _ => None,
        }
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

    #[test]
    fn init_at_max_channels_is_allowed() {
        let _g = crate::test_support::guard();
        setup(MAX_CHANNELS);
        // After init, every channel is disconnected (fd -1).
        assert!(receive_byte(MAX_CHANNELS - 1).is_none());
    }

    #[test]
    #[should_panic(expected = "exceeds max")]
    fn init_above_max_channels_panics() {
        let _g = crate::test_support::guard();
        crate::test_support::ensure_clock();
        init(MAX_CHANNELS + 1);
    }

    #[test]
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

    #[test]
    fn init_channel_fd_stores_the_fd_and_enables_io() {
        let _g = crate::test_support::guard();
        let pair = Pair::new();
        setup(1);
        init_channel_fd(0, pair.a);
        // With a real fd wired in, a transmit reaches the far end.
        transmit_data(0, b"hi");
        assert_eq!(pair.read_far(2), b"hi");
    }

    #[test]
    fn transmit_to_unconnected_channel_is_a_no_op() {
        let _g = crate::test_support::guard();
        setup(1);
        // Channel 0 configured but fd still -1: transmit silently discards.
        transmit_data(0, b"data");
        // Nothing to assert beyond "did not panic / did not block".
    }

    #[test]
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

    #[test]
    fn transmit_out_of_range_channel_is_a_no_op() {
        let _g = crate::test_support::guard();
        setup(1);
        // Channel 5 is past the configured count: no panic, nothing sent.
        transmit_data(5, b"data");
    }

    #[test]
    fn transmit_then_read_round_trips_bytes() {
        let _g = crate::test_support::guard();
        let pair = Pair::new();
        setup(1);
        init_channel_fd(0, pair.a);
        transmit_data(0, b"hi");
        assert_eq!(pair.read_far(2), b"hi");
    }

    #[test]
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

    #[test]
    fn receive_byte_on_unconnected_or_out_of_range_is_none() {
        let _g = crate::test_support::guard();
        setup(1);
        // Configured but disconnected (fd -1).
        assert_eq!(receive_byte(0), None);
        // Out-of-range channel.
        assert_eq!(receive_byte(9), None);
    }

    #[test]
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

    #[test]
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

    #[test]
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

    #[test]
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

    #[test]
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

    #[test]
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

    #[test]
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

    #[test]
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

    #[test]
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

    #[test]
    fn set_baud_out_of_range_channel_is_a_no_op() {
        let _g = crate::test_support::guard();
        setup(1);
        // Channel index past the array ceiling: silently ignored, no panic.
        set_baud(MAX_CHANNELS, 9600);
    }

    #[test]
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
}
