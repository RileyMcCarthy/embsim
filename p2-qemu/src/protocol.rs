//! The RUN/STOP protocol between the node and `qemu-system-p2`'s host-driven
//! mode, mirrored field for field from `qemu-target/target-p2/hostipc.c`,
//! which says what each message is for.
//!
//! The node and the program take turns: the node sends a [`Run`] (run the
//! cogs until this cog clock, with these net levels), the program answers
//! with a [`StopHeader`] and its tail (why it stopped, at what instant, what
//! the guest now drives). Before the first turn the program sends a
//! [`Hello`]: the protocol it speaks and the target it was built from.
//!
//! Both processes run on one host, so the structures travel in its native
//! byte order. They cross either a [`ShmPage`] — a shared page with a word
//! each side spins on, then blocks on with a futex — or a Unix socket pair,
//! the fallback. [`PROTOCOL`] changes whenever a layout here does.
//!
//! Public so another program can speak it: the tests' stand-in for
//! `qemu-system-p2` does (`tests/peer.rs`).

use std::io;
use std::os::fd::{AsRawFd, BorrowedFd, OwnedFd};
use std::ptr::NonNull;
use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};
use std::time::Duration;

/// The first word of the shared page and of every hello: `"PI2P"`.
pub const MAGIC: u32 = 0x5032_4950;

/// The protocol this crate speaks. A program speaking another is refused.
pub const PROTOCOL: u32 = 1;

/// A request asking for a run.
pub const OP_RUN: u32 = 1;
/// A request asking the program to exit.
pub const OP_QUIT: u32 = 2;

/// Why a run stopped: bits of [`StopHeader::reason`].
pub mod reason {
    /// A pad's drive changed: publish it at the pending instant.
    pub const YIELD: u32 = 1;
    /// Every running cog reached the horizon.
    pub const HORIZON: u32 = 2;
    /// The `HUBSET` clock word changed.
    pub const CLOCK: u32 = 4;
    /// No cog could run.
    pub const NOSTEP: u32 = 8;
    /// A running cog retired nothing for a thousand slices.
    pub const STALL: u32 = 16;
    /// The `WYPIN` tap is nearly full: come back for the rest.
    pub const CONSOLE: u32 = 32;
}

/// Offsets into the shared page. Each handoff word sits on a 128-byte line
/// of its own.
pub mod shm {
    /// `u32`: [`super::MAGIC`], written by the node before the program
    /// starts.
    pub const MAGIC: usize = 0;
    /// The program's [`super::Hello`].
    pub const HELLO: usize = 64;
    /// `u32`: 1 once the hello is in place.
    pub const READY: usize = 256;
    /// `u32`: the node bumps it once per request.
    pub const REQ_SEQ: usize = 384;
    /// `u32`: the program is blocked on `REQ_SEQ`.
    pub const REQ_SLEEP: usize = 512;
    /// `u32`: the request sequence number a reply answers.
    pub const REP_SEQ: usize = 640;
    /// `u32`: the node is blocked on `REP_SEQ`.
    pub const REP_SLEEP: usize = 768;
    /// The request, a [`super::Run`].
    pub const REQ: usize = 1024;
    /// The reply, a [`super::StopHeader`] and its tail.
    pub const REP: usize = 2048;
    /// The page.
    pub const SIZE: usize = 16384;
}

/// The longest reply: a header, every pad's mode word, and the rest
/// `WYPIN` bytes.
pub const REPLY_CAPACITY: usize = shm::SIZE - shm::REP;

fn u32_at(b: &[u8], at: usize) -> u32 {
    u32::from_ne_bytes(b[at..at + 4].try_into().expect("four bytes"))
}

fn u64_at(b: &[u8], at: usize) -> u64 {
    u64::from_ne_bytes(b[at..at + 8].try_into().expect("eight bytes"))
}

fn put_u32(b: &mut [u8], at: usize, v: u32) {
    b[at..at + 4].copy_from_slice(&v.to_ne_bytes());
}

fn put_u64(b: &mut [u8], at: usize, v: u64) {
    b[at..at + 8].copy_from_slice(&v.to_ne_bytes());
}

fn text_at(b: &[u8], at: usize, len: usize) -> String {
    let field = &b[at..at + len];
    let end = field.iter().position(|&c| c == 0).unwrap_or(len);
    String::from_utf8_lossy(&field[..end]).into_owned()
}

fn put_text(b: &mut [u8], at: usize, len: usize, text: &str) {
    let bytes = text.as_bytes();
    let n = bytes.len().min(len - 1);
    b[at..at + n].copy_from_slice(&bytes[..n]);
}

/// What the program says before the first turn. The magic and the protocol
/// are the first two words in every protocol, so a program of any version
/// can be told apart and named.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Hello {
    /// [`MAGIC`].
    pub magic: u32,
    /// The protocol the program speaks.
    pub protocol: u32,
    /// Its process id.
    pub pid: u32,
    /// The identity of the target sources it was built from.
    pub target: String,
    /// The QEMU version it is (`QEMU_VERSION`).
    pub qemu: String,
}

impl Hello {
    /// Its size on the wire.
    pub const LEN: usize = 128;

    /// The bytes.
    pub fn encode(&self) -> [u8; Self::LEN] {
        let mut b = [0u8; Self::LEN];
        put_u32(&mut b, 0, self.magic);
        put_u32(&mut b, 4, self.protocol);
        put_u32(&mut b, 8, self.pid);
        put_text(&mut b, 16, 32, &self.target);
        put_text(&mut b, 48, 32, &self.qemu);
        b
    }

    /// From the bytes.
    pub fn decode(b: &[u8; Self::LEN]) -> Self {
        Self {
            magic: u32_at(b, 0),
            protocol: u32_at(b, 4),
            pid: u32_at(b, 8),
            target: text_at(b, 16, 32),
            qemu: text_at(b, 48, 32),
        }
    }
}

/// A request: run, or quit.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Run {
    /// [`OP_RUN`] or [`OP_QUIT`].
    pub op: u32,
    /// The cog a pass continues from (after a clock change).
    pub start_cog: u32,
    /// Run every running cog until its clock reaches this.
    pub horizon_clocks: u64,
    /// What each net presents, last known: `P0..P31`, `P32..P63`.
    pub in_ext: [u32; 2],
    /// The pads whose published drive is strong: these read their own
    /// `OUT` bit, every other pad its net.
    pub strong: [u32; 2],
    /// Bit `b`: bank `b`'s supply names a voltage.
    pub banks_powered: u32,
    /// Bit `b`: and that voltage is not 0 V.
    pub banks_high: u32,
}

impl Run {
    /// Its size on the wire.
    pub const LEN: usize = 64;

    /// The bytes.
    pub fn encode(&self) -> [u8; Self::LEN] {
        let mut b = [0u8; Self::LEN];
        put_u32(&mut b, 0, self.op);
        put_u32(&mut b, 4, self.start_cog);
        put_u64(&mut b, 8, self.horizon_clocks);
        put_u32(&mut b, 16, self.in_ext[0]);
        put_u32(&mut b, 20, self.in_ext[1]);
        put_u32(&mut b, 24, self.strong[0]);
        put_u32(&mut b, 28, self.strong[1]);
        put_u32(&mut b, 32, self.banks_powered);
        put_u32(&mut b, 36, self.banks_high);
        b
    }

    /// From the bytes.
    pub fn decode(b: &[u8; Self::LEN]) -> Self {
        Self {
            op: u32_at(b, 0),
            start_cog: u32_at(b, 4),
            horizon_clocks: u64_at(b, 8),
            in_ext: [u32_at(b, 16), u32_at(b, 20)],
            strong: [u32_at(b, 24), u32_at(b, 28)],
            banks_powered: u32_at(b, 32),
            banks_high: u32_at(b, 36),
        }
    }
}

/// A reply's fixed part. The tail follows it: [`StopHeader::n_mode`]
/// `(u32 pin, u32 mode word)` pairs, then [`StopHeader::n_console`]
/// `(u8 pin, u8 byte)` pairs.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct StopHeader {
    /// Why the run stopped ([`reason`]).
    pub reason: u32,
    /// Mode words that changed.
    pub n_mode: u32,
    /// `WYPIN` bytes, in order.
    pub n_console: u32,
    /// The `HUBSET` clock word.
    pub clock_mode: u32,
    /// The cog clock it was written at.
    pub clock_mode_at: u64,
    /// With [`reason::YIELD`]: the clock of the pad change.
    pub pending_at_clocks: u64,
    /// The machine's clock: the least-advanced running cog.
    pub now_clocks: u64,
    /// With [`reason::YIELD`]: the pads to publish.
    pub dirty: u64,
    /// `DIR`, ORed over the cogs.
    pub dir: [u32; 2],
    /// `OUT`, ORed over the cogs.
    pub out: [u32; 2],
    /// Slices run.
    pub slices: u32,
    /// Whether any cog runs.
    pub any_running: u32,
    /// The cog that ran last, for a pass to continue after.
    pub last_cog: u32,
    /// What the program spent in the run, by its own clock, in ns.
    pub run_ns: u32,
}

impl StopHeader {
    /// Its size on the wire.
    pub const LEN: usize = 80;

    /// The tail's length, in bytes.
    pub fn tail_len(&self) -> usize {
        self.n_mode as usize * 8 + self.n_console as usize * 2
    }

    /// The bytes.
    pub fn encode(&self) -> [u8; Self::LEN] {
        let mut b = [0u8; Self::LEN];
        put_u32(&mut b, 0, self.reason);
        put_u32(&mut b, 4, self.n_mode);
        put_u32(&mut b, 8, self.n_console);
        put_u32(&mut b, 12, self.clock_mode);
        put_u64(&mut b, 16, self.clock_mode_at);
        put_u64(&mut b, 24, self.pending_at_clocks);
        put_u64(&mut b, 32, self.now_clocks);
        put_u64(&mut b, 40, self.dirty);
        put_u32(&mut b, 48, self.dir[0]);
        put_u32(&mut b, 52, self.dir[1]);
        put_u32(&mut b, 56, self.out[0]);
        put_u32(&mut b, 60, self.out[1]);
        put_u32(&mut b, 64, self.slices);
        put_u32(&mut b, 68, self.any_running);
        put_u32(&mut b, 72, self.last_cog);
        put_u32(&mut b, 76, self.run_ns);
        b
    }

    /// From the bytes.
    pub fn decode(b: &[u8]) -> Self {
        Self {
            reason: u32_at(b, 0),
            n_mode: u32_at(b, 4),
            n_console: u32_at(b, 8),
            clock_mode: u32_at(b, 12),
            clock_mode_at: u64_at(b, 16),
            pending_at_clocks: u64_at(b, 24),
            now_clocks: u64_at(b, 32),
            dirty: u64_at(b, 40),
            dir: [u32_at(b, 48), u32_at(b, 52)],
            out: [u32_at(b, 56), u32_at(b, 60)],
            slices: u32_at(b, 64),
            any_running: u32_at(b, 68),
            last_cog: u32_at(b, 72),
            run_ns: u32_at(b, 76),
        }
    }

    /// The changed mode words in `tail`, as (pin, word).
    pub fn modes<'a>(&self, tail: &'a [u8]) -> impl Iterator<Item = (usize, u32)> + 'a {
        tail[..self.n_mode as usize * 8]
            .as_chunks::<8>()
            .0
            .iter()
            .map(|pair| (u32_at(pair, 0) as usize & 63, u32_at(pair, 4)))
    }

    /// The `WYPIN` bytes in `tail`, as (pin, byte).
    pub fn console<'a>(&self, tail: &'a [u8]) -> impl Iterator<Item = (u8, u8)> + 'a {
        let start = self.n_mode as usize * 8;
        tail[start..start + self.n_console as usize * 2]
            .as_chunks::<2>()
            .0
            .iter()
            .map(|&[pin, byte]| (pin, byte))
    }
}

// ---- the shared page ---------------------------------------------------------

/// A process-shared page, [`shm::SIZE`] bytes: the channel the node and the
/// program hand requests and replies across.
///
/// It is a file with no name: created, unlinked at once and sized, so only
/// the descriptor reaches it and nothing is left on disk whichever process
/// dies first.
pub struct ShmPage {
    base: NonNull<u8>,
}

// SAFETY: the page is plain shared memory; every access goes through atomics
// or through copies the protocol orders with them.
unsafe impl Send for ShmPage {}

impl ShmPage {
    /// A new page with [`MAGIC`] in place, and the descriptor to hand the
    /// program.
    pub fn create() -> io::Result<(Self, OwnedFd)> {
        let fd = anonymous_file("shm", &[])?;
        let file = std::fs::File::from(fd);
        file.set_len(shm::SIZE as u64)?;
        let fd = OwnedFd::from(file);
        let page = Self::map(fd_ref(&fd))?;
        page.word(shm::MAGIC).store(MAGIC, Ordering::SeqCst);
        Ok((page, fd))
    }

    /// The page behind `fd`, mapped shared: what the program does with the
    /// descriptor it is handed.
    pub fn map(fd: BorrowedFd<'_>) -> io::Result<Self> {
        // SAFETY: a fresh shared mapping of a descriptor we were given; the
        // kernel checks it.
        let base = unsafe {
            libc::mmap(
                std::ptr::null_mut(),
                shm::SIZE,
                libc::PROT_READ | libc::PROT_WRITE,
                libc::MAP_SHARED,
                fd.as_raw_fd(),
                0,
            )
        };
        if base == libc::MAP_FAILED {
            return Err(io::Error::last_os_error());
        }
        Ok(Self {
            base: NonNull::new(base.cast::<u8>()).expect("mmap never maps page 0"),
        })
    }

    /// The handoff word at `offset`.
    pub fn word(&self, offset: usize) -> &AtomicU32 {
        assert!(offset.is_multiple_of(4) && offset + 4 <= shm::SIZE);
        // SAFETY: inside the mapping, aligned, alive as long as `self`.
        unsafe { &*self.base.as_ptr().add(offset).cast::<AtomicU32>() }
    }

    /// Copy `bytes` into the page at `offset`. Ordered by the store of the
    /// sequence word that follows it.
    pub fn write(&self, offset: usize, bytes: &[u8]) {
        assert!(offset + bytes.len() <= shm::SIZE);
        // SAFETY: inside the mapping; the other side reads it only after
        // the sequence word says so.
        unsafe {
            std::ptr::copy_nonoverlapping(
                bytes.as_ptr(),
                self.base.as_ptr().add(offset),
                bytes.len(),
            )
        };
    }

    /// Copy the page at `offset` into `out`. Ordered by the load of the
    /// sequence word before it.
    pub fn read(&self, offset: usize, out: &mut [u8]) {
        assert!(offset + out.len() <= shm::SIZE);
        // SAFETY: inside the mapping; the other side wrote it before the
        // sequence word was stored.
        unsafe {
            std::ptr::copy_nonoverlapping(
                self.base.as_ptr().add(offset),
                out.as_mut_ptr(),
                out.len(),
            )
        };
    }
}

impl Drop for ShmPage {
    fn drop(&mut self) {
        // SAFETY: our own mapping, of exactly this size.
        unsafe { libc::munmap(self.base.as_ptr().cast(), shm::SIZE) };
    }
}

fn fd_ref(fd: &OwnedFd) -> BorrowedFd<'_> {
    use std::os::fd::AsFd;
    fd.as_fd()
}

/// A file with no name holding `bytes`, its offset at 0: created in the
/// temporary directory and unlinked at once, so only the descriptor reaches
/// it. The boot ROM travels to the program this way, as `/dev/fd/N`.
pub fn anonymous_file(what: &str, bytes: &[u8]) -> io::Result<OwnedFd> {
    use std::io::{Seek, SeekFrom, Write};

    static NEXT: AtomicU64 = AtomicU64::new(0);
    let dir = std::env::temp_dir();
    loop {
        let path = dir.join(format!(
            "embsim-p2-qemu-{}-{}.{what}",
            std::process::id(),
            NEXT.fetch_add(1, Ordering::Relaxed)
        ));
        let mut file = match std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .create_new(true)
            .open(&path)
        {
            Ok(file) => file,
            Err(e) if e.kind() == io::ErrorKind::AlreadyExists => continue,
            Err(e) => return Err(e),
        };
        std::fs::remove_file(&path)?;
        file.write_all(bytes)?;
        file.seek(SeekFrom::Start(0))?;
        return Ok(OwnedFd::from(file));
    }
}

// ---- the futex both sides block on ------------------------------------------

#[cfg(target_os = "macos")]
mod futex {
    use std::sync::atomic::AtomicU32;
    use std::time::Duration;

    const UL_COMPARE_AND_WAIT_SHARED: u32 = 3;
    const ULF_NO_ERRNO: u32 = 0x0100_0000;

    extern "C" {
        fn __ulock_wait(op: u32, addr: *mut libc::c_void, value: u64, timeout_us: u32) -> i32;
        fn __ulock_wake(op: u32, addr: *mut libc::c_void, wake_value: u64) -> i32;
    }

    pub fn wait(word: &AtomicU32, expected: u32, timeout: Duration) {
        // A zero timeout is "forever" to __ulock_wait: at least 1 us.
        let us = u32::try_from(timeout.as_micros())
            .unwrap_or(u32::MAX)
            .max(1);
        // SAFETY: a word in a shared mapping; the kernel compares it with
        // `expected` before it sleeps.
        unsafe {
            __ulock_wait(
                UL_COMPARE_AND_WAIT_SHARED | ULF_NO_ERRNO,
                word.as_ptr().cast(),
                u64::from(expected),
                us,
            )
        };
    }

    pub fn wake(word: &AtomicU32) {
        // SAFETY: as above.
        unsafe {
            __ulock_wake(
                UL_COMPARE_AND_WAIT_SHARED | ULF_NO_ERRNO,
                word.as_ptr().cast(),
                0,
            )
        };
    }
}

#[cfg(target_os = "linux")]
mod futex {
    use std::sync::atomic::AtomicU32;
    use std::time::Duration;

    // Shared futexes: no FUTEX_PRIVATE_FLAG, the word is in a page two
    // processes map.
    pub fn wait(word: &AtomicU32, expected: u32, timeout: Duration) {
        let ts = libc::timespec {
            tv_sec: timeout.as_secs() as libc::time_t,
            tv_nsec: timeout.subsec_nanos() as libc::c_long,
        };
        // SAFETY: a word in a shared mapping; the kernel compares it with
        // `expected` before it sleeps.
        unsafe {
            libc::syscall(
                libc::SYS_futex,
                word.as_ptr(),
                libc::FUTEX_WAIT,
                expected,
                &ts as *const libc::timespec,
                std::ptr::null::<u32>(),
                0,
            )
        };
    }

    pub fn wake(word: &AtomicU32) {
        // SAFETY: as above.
        unsafe {
            libc::syscall(
                libc::SYS_futex,
                word.as_ptr(),
                libc::FUTEX_WAKE,
                1,
                std::ptr::null::<libc::timespec>(),
                std::ptr::null::<u32>(),
                0,
            )
        };
    }
}

#[cfg(not(any(target_os = "macos", target_os = "linux")))]
mod futex {
    use std::sync::atomic::AtomicU32;
    use std::time::Duration;

    // No shared futex here: the page still works, polling.
    pub fn wait(_word: &AtomicU32, _expected: u32, timeout: Duration) {
        std::thread::sleep(timeout.min(Duration::from_micros(50)));
    }

    pub fn wake(_word: &AtomicU32) {}
}

/// Block until `word` may no longer hold `expected`, or `timeout` passes:
/// a shared futex (`__ulock` on macOS, `futex(2)` on Linux). It may return
/// early; the caller re-reads the word.
pub fn futex_wait(word: &AtomicU32, expected: u32, timeout: Duration) {
    futex::wait(word, expected, timeout);
}

/// Wake the process blocked on `word`.
pub fn futex_wake(word: &AtomicU32) {
    futex::wake(word);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_message_round_trips_at_its_wire_size() {
        let hello = Hello {
            magic: MAGIC,
            protocol: PROTOCOL,
            pid: 42,
            target: "0123456789abcdef".into(),
            qemu: "10.1.0".into(),
        };
        assert_eq!(Hello::decode(&hello.encode()), hello);
        let run = Run {
            op: OP_RUN,
            start_cog: 3,
            horizon_clocks: 1 << 40,
            in_ext: [1, 2],
            strong: [3, 4],
            banks_powered: 0xFFFF,
            banks_high: 0x7FFF,
        };
        assert_eq!(Run::decode(&run.encode()), run);
        let stop = StopHeader {
            reason: reason::YIELD | reason::CLOCK,
            n_mode: 1,
            n_console: 2,
            clock_mode: 0x0100_07FB,
            clock_mode_at: 5,
            pending_at_clocks: 6,
            now_clocks: 7,
            dirty: 1 << 63,
            dir: [8, 9],
            out: [10, 11],
            slices: 12,
            any_running: 1,
            last_cog: 7,
            run_ns: 13,
        };
        assert_eq!(StopHeader::decode(&stop.encode()), stop);
        let mut tail = Vec::new();
        tail.extend_from_slice(&62u32.to_ne_bytes());
        tail.extend_from_slice(&0x7Cu32.to_ne_bytes());
        tail.extend_from_slice(&[62, b'H', 62, b'i']);
        assert_eq!(stop.tail_len(), tail.len());
        assert_eq!(stop.modes(&tail).collect::<Vec<_>>(), vec![(62, 0x7C)]);
        assert_eq!(
            stop.console(&tail).collect::<Vec<_>>(),
            vec![(62, b'H'), (62, b'i')]
        );
    }

    #[test]
    fn a_shared_page_carries_its_magic_and_is_seen_through_a_second_mapping() {
        let (page, fd) = ShmPage::create().expect("a shared page");
        let other = ShmPage::map(fd_ref(&fd)).expect("mapped again");
        assert_eq!(other.word(shm::MAGIC).load(Ordering::SeqCst), MAGIC);
        page.write(shm::REQ, &[1, 2, 3, 4]);
        let mut back = [0u8; 4];
        other.read(shm::REQ, &mut back);
        assert_eq!(back, [1, 2, 3, 4]);
        // A wait on a word that already moved returns at once.
        futex_wait(other.word(shm::REQ_SEQ), 1, Duration::from_secs(5));
    }
}
