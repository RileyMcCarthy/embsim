//! The QEMU Propeller 2 target as the core of an embsim P2 package.
//!
//! A `P2X8C4M64P` that boots the way silicon does: the real Parallax ROM in
//! the top of hub, and everything else arriving over pins that are **nets** —
//! a flash on the board answers the ROM's bit-banged SPI, and the P2 sees it
//! the way it sees any other component. The node knows nothing about flashes,
//! cards or UARTs. It drives pads and senses pads.
//!
//! # The package around it
//!
//! [`P2Qemu`] is a [`P2Core`]: it goes inside
//! [`embsim_boards::p2::P2Package`], which declares the 86 package pins,
//! hands the core its 64 pads, and delivers the package-level facts —
//! the **crystal**, which is the rate the board puts on `XI` (a TCXO
//! through its buffer on the P2-EC32MB), the `RESN`/`VDD` state, and the
//! sixteen bank supplies the pads drive high at. A board fills its
//! processor slot with `P2Package::new(P2Qemu::…)`.
//!
//! # When the guest starts
//!
//! The package's **START gate** decides: the core is started — and its
//! first wake delivered — the datasheet's 3 ms restart delay after `RESN`
//! reads released with `VDD` inside its window (`embsim_boards::p2`). The
//! START instant is where the guest's clock begins: [`P2Core::start`]
//! anchors clock segment 0 at the virtual instant it runs, so a guest
//! started 5.5 ms in (the P2-EC32MB from its carrier's 5 V: its bucks'
//! soft-start elapsed at 2.5 ms, which releases the reset, and the restart
//! delay after it) stamps its first instruction there, and every edge
//! after it at its own instant from there.
//!
//! # Where the CPU runs
//!
//! In `qemu-system-p2`, a program of its own: QEMU is not linked into
//! embsim. The program is found when a node starts ([`QemuSystemP2::find`]:
//! `EMBSIM_QEMU_SYSTEM_P2`, then `PATH`, then where `embsim qemu install`
//! puts it), runs in host-driven mode, and takes turns with the node in
//! lockstep: the node's wake sends one RUN — run the cogs until this
//! instant, with these net levels — and the program answers with one STOP —
//! a pad changed at this instant, or the instant was reached ([`protocol`]).
//! The guest-facing half of the pin bus lives in the program, next to the
//! CPU (`qemu-target/target-p2/hostipc.c`), so not one pin operation crosses
//! between the two; the electrical model — what a pad presents, the bank
//! supplies, the nets, the clock — stays here. Over a shared page the two
//! spin before they block, and a turn costs 0.24–0.39 us more than a call
//! into a linked QEMU did; the ROM boot takes the same 29–34 ms.
//!
//! Each node starts its own program, so a board may carry two P2s, and a
//! test binary may hold as many as it likes. The program never outlives the
//! node: it dies with it, and with embsim's process however that ends
//! ([`peer`]). A program that dies or stops answering is reported — its
//! exit status and the last of its standard error — by
//! [`P2QemuHandle::failure`], and the core runs no further.
//!
//! The program must be the one this crate speaks to: its handshake names its
//! protocol, the target sources it was built from and its QEMU, and any
//! other is refused with how to install the matching one
//! ([`target::identity`]).
//!
//! # How an edge gets its instant
//!
//! **The guest leads; the engine follows it to each edge.** A wake runs the
//! cogs forward from their own clocks. The moment one changes a pad, the
//! program stops that cog after the instruction, records the cog's clock as
//! the edge's instant, and the wake ends by arming itself at that instant.
//! The engine advances there, the next wake **publishes** the drive — so it
//! is stamped at the guest's own instant, never the wake's — and arms again
//! one nanosecond on, so the engine resolves the net and delivers any
//! response (a flash presenting its next bit) before the guest reads
//! anything back. Two wakes per edge, each edge at its true instant, and a
//! device on the net sees every transition — the rules
//! `docs/dev/sil-unified-drive.md` sets out, R1 through R5.
//!
//! # What a pad is, and what it reads
//!
//! A pad the guest drives is a Thevenin source at the strength its `WRPIN`
//! word configured (`embsim_boards::p2::pad_drive`): fast at
//! [`embsim_boards::p2::P2_FAST_OHMS`], the 1.5 k / 15 k / 150 kΩ modes at
//! those resistances, float released — and high at its **bank's supply**,
//! the voltage the package senses on the pad's `VIO_a_b` pin
//! ([`BankSupplies::pad_drive`]); a pad in a bank whose supply names no
//! voltage presents nothing, and the package reports the bank once. A
//! `WRPIN` on a driven pad is a pad change like a `DIR` write — it yields,
//! and the next wake republishes the pad at the new strength. A bank whose
//! supply moves has its driven pads republished at the node's next wake.
//! The current-source modes are not mapped: such a pad presents nothing,
//! and the node says so once.
//!
//! `IN` reflects the **net** for every pad whose published drive is
//! released or a pull (at or above [`WEAK_DRIVE_OHMS`]) — a pad pulling a
//! line high through 15 kΩ reads what the line resolved to, which is what
//! makes an I2C slave's clock stretch and its ACK visible to the master
//! driving through the pull mode. A pad driven fast keeps reading its own
//! `OUT` bit, which is what p2core does and what keeps the two engines'
//! state traces identical instruction for instruction.

#![warn(missing_docs)]

use std::collections::VecDeque;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use embsim_board::{AttachError, Level, PinHandle, TheveninDrive, WEAK_DRIVE_OHMS};
use embsim_boards::p2::{self, BankSupplies, P2Core, P2Pads, P2ResetState, PadDrive, NUM_BANKS};
use embsim_core::virtual_clock;

pub use embsim_boards::p2::{p2x8c4m64p_pins, pin_name};

pub mod catalog;
pub mod flashimage;
pub mod install;
pub mod peer;
pub mod protocol;
pub mod target;

pub use peer::{Found, Identity, Peer, QemuSystemP2, Transport};

use protocol::{reason, Run, StopHeader, OP_RUN};

/// Parallax's boot ROM, the program the chip carries into the top of hub
/// at reset, trimmed to its boot path — the flash and serial loaders
/// (`rom/README.md`; MIT, Copyright (c) 2019 Parallax Inc.,
/// `rom/LICENSE-PARALLAX`). What [`P2Qemu::with_boot_rom`] takes when a
/// program has no other ROM to give it.
pub const BOOT_ROM: &[u8] = include_bytes!("../rom/rom_booter_v33k.bin");

/// embsim's stage-1 flash loader (`rom/stage1.spin2`): the kilobyte the ROM
/// loads off the flash, which [`flashimage::boot_flash`] puts in front of a
/// program.
pub const STAGE1: &[u8] = include_bytes!("../rom/stage1.bin");

/// How far past its own clock one wake may run the guest, in nanoseconds.
/// Only bounds the work per wake; the engine is re-armed at wherever the
/// guest got to.
pub const SLICE_NS: u64 = 100_000;

const NUM_COGS: u32 = 8;
const NUM_PINS: usize = p2::NUM_PADS;
/// Pads per bank: bank `b` is `P(4b)..P(4b+3)`.
const PADS_PER_BANK: usize = NUM_PINS / NUM_BANKS;

/// The internal RC oscillator the chip comes up on, nominal. Silicon runs
/// RCFAST anywhere in 20–24 MHz; 20 MHz is Parallax's stated figure.
pub const RCFAST_HZ: u64 = 20_000_000;
/// The slow internal oscillator (`%01`), nominal.
pub const RCSLOW_HZ: u64 = 20_000;

/// The system clock a HUBSET clock word selects, given the crystal on `XI`
/// — the rate the board delivers there, or `None` when nothing does.
///
/// `%0000_000E_DDDD_DDMM_MMMM_MMMM_PPPP_CC_SS`: `SS` picks RCFAST, RCSLOW,
/// the crystal, or the PLL; the PLL runs the crystal through `/(D+1)`,
/// `*(M+1)`, and a final divider `PPPP` — `%1111` is the VCO direct,
/// anything else `/((P+1)*2)`. A word that derives its clock from the
/// crystal when there is none yields `None`: a chip whose clock source is
/// absent has no clock, and the node stalls the guest until one arrives
/// rather than inventing a frequency. What the guest records in hub `$14`
/// is NOT used: the boot ROM overwrites that long with its base64 table,
/// and the clock is a fact of the hardware the guest set, not a value it
/// stored.
///
/// `SS` is read with the fields the datasheet's `%SS` notes name
/// (P2X8C4M64P Datasheet, System Clock, p. 18): `XI` (`%10`) needs
/// "CC != %00" and the PLL (`%11`) "CC != %00 and E=1" — in `%CC` = `%00`
/// "XI status" is "ignored", and `%E` is "PLL off/on". A word that selects
/// either without them selects a source that never runs, and yields `None`
/// whatever the crystal: the clock selector waits for a positive edge on
/// the new source before switching over to it (PLL Example, p. 19), so the
/// chip has no clock, and no rate reaching an ignored `XI` gives it one
/// (`source_runs`).
pub fn clock_hz(mode: u32, crystal_hz: Option<u64>) -> Option<u64> {
    if !source_runs(mode) {
        return None;
    }
    match mode & 0b11 {
        0b00 => Some(RCFAST_HZ),
        0b01 => Some(RCSLOW_HZ),
        0b10 => crystal_hz,
        _ => {
            let crystal_hz = crystal_hz?;
            let d = u64::from((mode >> 18) & 0x3F) + 1;
            let m = u64::from((mode >> 8) & 0x3FF) + 1;
            let p = (mode >> 4) & 0xF;
            let pdiv = if p == 0xF { 1 } else { (u64::from(p) + 1) * 2 };
            Some((crystal_hz * m / d / pdiv).max(1))
        }
    }
}

/// Whether a HUBSET clock word's source is the crystal (directly or through
/// the PLL).
const fn derives_from_crystal(mode: u32) -> bool {
    matches!(mode & 0b11, 0b10 | 0b11)
}

/// Whether the source a HUBSET clock word selects runs at all, by the
/// datasheet's `%SS` notes (System Clock, p. 18): RCFAST and RCSLOW always;
/// `XI` only with its input on, `%CC` ≠ `%00`; the PLL only with `XI` on
/// and `%E` set.
const fn source_runs(mode: u32) -> bool {
    let xi_on = (mode >> 2) & 0b11 != 0;
    let pll_on = (mode >> 24) & 1 == 1;
    match mode & 0b11 {
        0b10 => xi_on,
        0b11 => xi_on && pll_on,
        _ => true,
    }
}

// ============================================================
// Errors
// ============================================================

/// Why a node could not start, or stopped.
#[derive(Debug)]
pub enum P2QemuError {
    /// No `qemu-system-p2` to run, or a setting naming one that is wrong:
    /// the text says where it looked and how to install it.
    NotFound(String),
    /// The program could not be started.
    Start {
        /// The program.
        program: PathBuf,
        /// Why.
        error: std::io::Error,
    },
    /// The program is not the one this crate speaks to: another protocol,
    /// another target, another QEMU. The text says which, and how to
    /// install the matching one.
    Refused {
        /// The program.
        program: PathBuf,
        /// What differs, and the fix.
        why: String,
    },
    /// The program exited.
    Died {
        /// The program.
        program: PathBuf,
        /// Its process id.
        pid: u32,
        /// Its exit status, in words.
        status: String,
        /// When: before its hello, or during a run.
        during: String,
        /// The last lines it wrote to standard error.
        stderr: String,
    },
    /// The program lives but did not answer.
    Unresponsive {
        /// The program.
        program: PathBuf,
        /// Its process id.
        pid: u32,
        /// How long it was given.
        waited: Duration,
        /// When.
        during: String,
    },
    /// Setting up the channel or the ROM failed.
    Io(std::io::Error),
}

impl std::fmt::Display for P2QemuError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            P2QemuError::NotFound(text) => write!(f, "{text}"),
            P2QemuError::Start { program, error } => write!(
                f,
                "cannot start {}: {error}. {}",
                program.display(),
                peer::install_advice()
            ),
            P2QemuError::Refused { why, .. } => write!(f, "{why}"),
            P2QemuError::Died {
                program,
                pid,
                status,
                during,
                stderr,
            } => {
                write!(
                    f,
                    "qemu-system-p2 ({}, pid {pid}) {status} {during}",
                    program.display()
                )?;
                if stderr.is_empty() {
                    write!(f, "; it wrote nothing to standard error")?;
                } else {
                    write!(f, "; the last it wrote to standard error:\n{stderr}")?;
                }
                if during.starts_with("before") {
                    write!(
                        f,
                        "\nA qemu-system-p2 that does not know the `hostipc` machine property \
                         is not one `embsim qemu install` built. {}",
                        peer::install_advice()
                    )?;
                }
                Ok(())
            }
            P2QemuError::Unresponsive {
                program,
                pid,
                waited,
                during,
            } => write!(
                f,
                "qemu-system-p2 ({}, pid {pid}) did not answer {during} in {} s and was killed",
                program.display(),
                waited.as_secs()
            ),
            P2QemuError::Io(e) => write!(f, "setting up qemu-system-p2's channel: {e}"),
        }
    }
}

impl std::error::Error for P2QemuError {}

impl From<std::io::Error> for P2QemuError {
    fn from(e: std::io::Error) -> Self {
        P2QemuError::Io(e)
    }
}

// ============================================================
// Shared, cross-thread state
// ============================================================

/// What a [`P2QemuHandle`] can see from any thread.
#[derive(Default)]
struct Shared {
    /// Every `WYPIN` the guest issued, in order, as (pin, low byte). An
    /// instrumentation tap on the instruction stream, not the wire: it
    /// records the write whether or not the pin was configured to transmit,
    /// because a boot payload's whole observable is often one such write to
    /// the debug pin — and the P2's own boot chain issues it without ever
    /// configuring the pin. A serial peer on the net is where the bytes go
    /// once a transmit bridge exists.
    console: Mutex<Vec<(u8, u8)>>,
    /// Net transitions sensed since the last wake, in delivery order.
    edges: Mutex<VecDeque<(u8, bool)>>,
    /// The crystal, as the package last delivered it from `XI`: the rate
    /// in hertz, 0 while nothing reaches the pin.
    crystal_hz: AtomicU64,
    /// The reset inputs, as the package last delivered them. Information:
    /// the START gate that acts on them is the package's, and a rail
    /// dropping after the start reaches the core as [`P2Core::reset`].
    reset: Mutex<P2ResetState>,
    /// The package held the core ([`P2Core::reset`]): a brownout without a
    /// reset. No slice runs from here, and the pads keep what they last
    /// published.
    held: AtomicBool,
    /// The guest selected a clock derived from the crystal while none
    /// reached `XI`, and is not running until one does.
    stalled: AtomicBool,
    /// Times the guest stopped on a pad change.
    yields: AtomicU64,
    /// Drives published to nets.
    publishes: AtomicU64,
    /// Slices run.
    slices: AtomicU64,
    /// Turns taken with the program: RUN/STOP pairs.
    turns: AtomicU64,
    /// Wall time spent in turns, nanoseconds.
    turn_ns: AtomicU64,
    /// Of that, what the program spent running the guest, by its clock.
    run_ns: AtomicU64,
    /// Every cog has stopped.
    halted: AtomicBool,
    /// The node is being torn down; wakes do nothing.
    shutdown: AtomicBool,
    /// The program's process id.
    pid: AtomicU32,
    /// Why the core stopped, if the program died or stopped answering.
    failure: Mutex<Option<String>>,
}

/// A view of the node that outlives handing it to a `System`.
#[derive(Clone)]
pub struct P2QemuHandle {
    shared: Arc<Shared>,
}

impl P2QemuHandle {
    /// The low byte of every `WYPIN` the guest issued on `pin`, as text,
    /// lossy. An instruction-stream tap, recorded whether or not the pin
    /// was configured to transmit — see [`P2Qemu`] for why.
    pub fn console(&self, pin: u8) -> String {
        let bytes: Vec<u8> = self
            .shared
            .console
            .lock()
            .expect("console never poisoned")
            .iter()
            .filter(|(p, _)| *p == pin)
            .map(|(_, b)| *b)
            .collect();
        String::from_utf8_lossy(&bytes).into_owned()
    }

    /// Times the guest stopped on a pad change so the net could resolve.
    pub fn yields(&self) -> u64 {
        self.shared.yields.load(Ordering::Relaxed)
    }

    /// Drives the node has published to nets.
    pub fn publishes(&self) -> u64 {
        self.shared.publishes.load(Ordering::Relaxed)
    }

    /// Slices run so far.
    pub fn slices(&self) -> u64 {
        self.shared.slices.load(Ordering::Relaxed)
    }

    /// Turns taken with `qemu-system-p2`: one RUN and one STOP each.
    pub fn turns(&self) -> u64 {
        self.shared.turns.load(Ordering::Relaxed)
    }

    /// Wall time the node spent in turns, and of that what the program
    /// spent running the guest by its own clock, in nanoseconds: the
    /// difference is the channel's.
    pub fn turn_ns(&self) -> (u64, u64) {
        (
            self.shared.turn_ns.load(Ordering::Relaxed),
            self.shared.run_ns.load(Ordering::Relaxed),
        )
    }

    /// The process id of the node's `qemu-system-p2`.
    pub fn pid(&self) -> u32 {
        self.shared.pid.load(Ordering::Relaxed)
    }

    /// Whether every cog has stopped (or the core stopped on a failure).
    pub fn halted(&self) -> bool {
        self.shared.halted.load(Ordering::Relaxed)
    }

    /// Why the core stopped, when `qemu-system-p2` died or stopped
    /// answering: its exit status and the last it wrote to standard error.
    pub fn failure(&self) -> Option<String> {
        self.shared
            .failure
            .lock()
            .expect("failure never poisoned")
            .clone()
    }

    /// The crystal the PLL multiplies: the rate the board delivers on `XI`,
    /// or `None` while nothing reaches the pin.
    pub fn crystal_hz(&self) -> Option<u64> {
        let hz = self.shared.crystal_hz.load(Ordering::Relaxed);
        (hz != 0).then_some(hz)
    }

    /// The reset inputs (`RESN`, `VDD`), as the package last delivered them.
    pub fn reset(&self) -> P2ResetState {
        *self
            .shared
            .reset
            .lock()
            .expect("reset state never poisoned")
    }

    /// Whether the guest is stalled on a crystal-derived clock with no
    /// crystal on `XI`.
    pub fn stalled(&self) -> bool {
        self.shared.stalled.load(Ordering::Relaxed)
    }

    /// Whether the package held the core — `VDD` left its window while the
    /// guest ran with `RESN` not asserted ([`P2Core::reset`]).
    pub fn held(&self) -> bool {
        self.shared.held.load(Ordering::Relaxed)
    }
}

// ============================================================
// The electrical side
// ============================================================

/// Said once per process: a pad was configured in a current-source drive
/// mode, which the node does not map.
static CURRENT_SOURCE_LOGGED: AtomicBool = AtomicBool::new(false);

/// What a bank's supply was last seen as: its voltage's bits, or none.
const NO_SUPPLY: u64 = u64::MAX;

/// One constant-frequency stretch of the guest's clock.
#[derive(Debug, Clone, Copy)]
struct ClockSegment {
    from_clocks: u64,
    from_ns: u64,
    hz: u64,
}

/// The electrical model and the guest as the last STOP left it: what each
/// pad presents and was told, the bank supplies, what the nets read, and the
/// guest's clock in nanoseconds. Everything the program does not own.
struct Pads {
    // ---- the guest, as the last STOP reported it ------------------------
    dir: [u32; 2],
    out: [u32; 2],
    mode: [u32; NUM_PINS],
    /// The machine's clock: the least-advanced running cog.
    now_clocks: u64,
    any_running: bool,
    reply_clock_mode: u32,
    reply_clock_mode_at: u64,

    // ---- the outside, as the next RUN carries it ------------------------
    /// What the nets present, last known. A pin whose net is floating or in
    /// contention keeps its last level: an unresolvable net is not a logic
    /// value, and inventing one would hide the fault.
    in_ext: [u32; 2],
    /// Pads whose **published** drive is a strong source (under
    /// [`WEAK_DRIVE_OHMS`]): these read their own `OUT` bit; every other pad
    /// reads its net. Recomputed at each publish.
    strong: [u32; 2],

    // ---- what each net was told -----------------------------------------
    /// `None` never, `Some(None)` released, `Some(Some(drive))` a source.
    published: [Option<Option<TheveninDrive>>; NUM_PINS],
    /// Pins the guest changed and the next publish puts on their nets.
    dirty: u64,
    /// The instant of the pending pad change: the driving cog's clock at the
    /// instruction. `Some` between the yield and the publish.
    pending_at_ns: Option<u64>,
    handles: Vec<Option<PinHandle>>,
    /// The bank supplies, as the package senses them: what a pad drives
    /// high at. Unpowered until the package hands the core its table.
    banks: BankSupplies,
    /// Each bank's supply as the pads were last published against it.
    bank_bits: [u64; NUM_BANKS],
    /// Banks a pad was driven in while unpowered, reported once each.
    unpowered_reported: u16,

    // ---- time -----------------------------------------------------------
    /// The crystal on `XI`, as last drained from the package's delivery.
    crystal_hz: Option<u64>,
    /// The crystal the current clock segment was derived from.
    clocked_crystal_hz: Option<u64>,
    /// The HUBSET clock word last seen, to notice a change.
    clock_mode: u32,
    /// The guest selected a crystal-derived clock with no crystal: no
    /// clock, no instructions, until a rate reaches `XI`.
    stalled: bool,
    /// The clocks→nanoseconds mapping, piecewise: each segment starts at a
    /// cog clock count and the nanoseconds it corresponds to, at a rate. A
    /// clock change mid-run adds a segment; earlier instants keep theirs.
    clock_segments: Vec<ClockSegment>,
    shared: Arc<Shared>,
}

impl Pads {
    fn new(shared: Arc<Shared>) -> Self {
        Self {
            dir: [0; 2],
            out: [0; 2],
            mode: [0; NUM_PINS],
            now_clocks: 0,
            any_running: true,
            reply_clock_mode: 0,
            reply_clock_mode_at: 0,
            in_ext: [0; 2],
            strong: [0; 2],
            published: [None; NUM_PINS],
            dirty: 0,
            pending_at_ns: None,
            handles: (0..NUM_PINS).map(|_| None).collect(),
            banks: BankSupplies::unpowered(),
            bank_bits: [NO_SUPPLY; NUM_BANKS],
            unpowered_reported: 0,
            crystal_hz: None,
            clocked_crystal_hz: None,
            clock_mode: 0,
            stalled: false,
            clock_segments: vec![ClockSegment {
                from_clocks: 0,
                from_ns: 0,
                hz: RCFAST_HZ,
            }],
            shared,
        }
    }

    /// The drive the guest presents on `pin`: its `DIR`/`OUT` bits through
    /// the strength its `WRPIN` word configured, high at the pad's bank
    /// supply, or `None` when the pad is released — `DIR` clear, the float
    /// mode, or a bank with no supply. A current-source mode is not mapped
    /// and presents nothing.
    fn pad_drive(&self, pin: usize) -> Option<TheveninDrive> {
        let bit = 1u32 << (pin & 31);
        let half = pin >> 5;
        let dir = self.dir[half] & bit != 0;
        let out = self.out[half] & bit != 0;
        match self.banks.pad_drive(pin as u8, self.mode[pin], dir, out) {
            PadDrive::Released => None,
            PadDrive::Thevenin(drive) => Some(drive),
            PadDrive::CurrentSource(mode) => {
                if !CURRENT_SOURCE_LOGGED.swap(true, Ordering::Relaxed) {
                    tracing::warn!(
                        pin,
                        ?mode,
                        "p2-qemu: a pad was configured in a current-source drive mode, which \
                         is not mapped (no caller, and a resistor would be an invention); \
                         the pad presents nothing to its net"
                    );
                }
                None
            }
        }
    }

    fn set_input_level(&mut self, pin: u8, level: bool) {
        let p = usize::from(pin & 63);
        let bit = 1u32 << (p & 31);
        if level {
            self.in_ext[p >> 5] |= bit;
        } else {
            self.in_ext[p >> 5] &= !bit;
        }
    }

    fn drain_edges(&mut self) {
        let shared = Arc::clone(&self.shared);
        let mut queue = shared.edges.lock().expect("edge queue never poisoned");
        for (pin, level) in queue.drain(..) {
            self.set_input_level(pin, level);
        }
    }

    /// Take the crystal the package last delivered on `XI`.
    fn drain_crystal(&mut self) {
        let hz = self.shared.crystal_hz.load(Ordering::Relaxed);
        self.crystal_hz = (hz != 0).then_some(hz);
    }

    // ---- the banks --------------------------------------------------------

    /// What the next RUN says of the banks: which supplies name a voltage,
    /// and which of those are not 0 V — what the program's drive key needs
    /// to change exactly when [`Self::pad_drive`] does.
    fn bank_masks(&self) -> (u32, u32) {
        let mut powered = 0;
        let mut high = 0;
        for (bank, &bits) in self.bank_bits.iter().enumerate() {
            if bits != NO_SUPPLY {
                powered |= 1 << bank;
                if f64::from_bits(bits) != 0.0 {
                    high |= 1 << bank;
                }
            }
        }
        (powered, high)
    }

    /// Republish the pads of every bank whose supply moved since they were
    /// published: a pad driven high follows its supply. Run at a wake with
    /// no pad change pending, before the guest runs on.
    fn follow_supplies(&mut self) {
        let mut moved = 0u16;
        for bank in 0..NUM_BANKS {
            let bits = self.banks.volts(bank).map_or(NO_SUPPLY, f64::to_bits);
            if bits != self.bank_bits[bank] {
                self.bank_bits[bank] = bits;
                moved |= 1 << bank;
            }
        }
        if moved == 0 {
            return;
        }
        let mut published = 0u64;
        let mut changed = false;
        for pin in 0..NUM_PINS {
            if moved & (1 << (pin / PADS_PER_BANK)) == 0 {
                continue;
            }
            let want = self.pad_drive(pin);
            if self.published[pin] != Some(want) {
                if let Some(handle) = self.handles[pin].as_ref() {
                    handle.set_drive(want);
                    published += 1;
                }
                self.published[pin] = Some(want);
                changed = true;
            }
        }
        if changed {
            self.refresh_strong();
            self.shared
                .publishes
                .fetch_add(published, Ordering::Relaxed);
        }
    }

    /// A pad the guest drives in a bank whose supply names no voltage
    /// presents nothing, and the package reports the bank once
    /// ([`BankSupplies::pad_drive`]). The program never asks to publish
    /// such a pad — released to released is no change — so the node asks
    /// the package on its behalf, once per bank.
    fn report_unpowered_drives(&mut self) {
        let driven = u64::from(self.dir[0]) | (u64::from(self.dir[1]) << 32);
        if driven == 0 {
            return;
        }
        for bank in 0..NUM_BANKS {
            if self.bank_bits[bank] != NO_SUPPLY || self.unpowered_reported & (1 << bank) != 0 {
                continue;
            }
            let pins = 0b1111u64 << (bank * PADS_PER_BANK);
            if driven & pins == 0 {
                continue;
            }
            let pin = (driven & pins).trailing_zeros() as usize;
            self.unpowered_reported |= 1 << bank;
            let _ = self.pad_drive(pin);
        }
    }

    // ---- time -------------------------------------------------------------

    /// Notice a HUBSET clock change, or the crystal arriving or changing
    /// under a clock derived from it, and start a new segment: at the
    /// instant the guest made the change, or — for a crystal that arrived
    /// while the guest was stalled — at `now`, the wake that found it.
    /// Cheap when nothing changed: two compares.
    ///
    /// A changed word is reported to the package first
    /// ([`P2Pads::set_clock_mode`]): its `%CC` field is `XI`'s mode, which
    /// decides the crystal the package reads on `XI`, and the package
    /// answers through `on_crystal` before the call returns — so the
    /// crystal is taken again before the word is decoded against it.
    fn poll_clock_mode(&mut self, now: u64, pads: &P2Pads) {
        let mode = self.reply_clock_mode;
        let mode_changed = mode != self.clock_mode;
        if mode_changed {
            pads.set_clock_mode(mode);
            self.drain_crystal();
        }
        let crystal_changed = self.crystal_hz != self.clocked_crystal_hz;
        if !mode_changed && !crystal_changed {
            return;
        }
        self.clocked_crystal_hz = self.crystal_hz;
        if !mode_changed && !derives_from_crystal(mode) {
            // RCFAST/RCSLOW: the crystal is not the clock.
            return;
        }
        let was_stalled = self.stalled;
        let at = if mode_changed {
            self.reply_clock_mode_at
        } else {
            self.now_clocks
        };
        self.clock_mode = mode;
        match clock_hz(mode, self.crystal_hz) {
            Some(hz) => {
                let from_ns = if was_stalled {
                    now
                } else {
                    self.clocks_to_ns(at)
                };
                tracing::info!(
                    mode = format_args!("{mode:#010x}"),
                    hz,
                    crystal_hz = self.crystal_hz,
                    at,
                    from_ns,
                    "p2-qemu: clock set"
                );
                self.stalled = false;
                self.clock_segments.push(ClockSegment {
                    from_clocks: at,
                    from_ns,
                    hz,
                });
            }
            None if !source_runs(mode) => {
                tracing::warn!(
                    mode = format_args!("{mode:#010x}"),
                    "p2-qemu: HUBSET selected XI with its input off (%CC = %00) or the PLL \
                     with it or %E off; the source never runs, so the guest has no clock and \
                     stalls for good"
                );
                self.stalled = true;
            }
            None => {
                tracing::warn!(
                    mode = format_args!("{mode:#010x}"),
                    "p2-qemu: HUBSET selected a clock derived from the crystal while no rate \
                     reaches XI; the guest has no clock and stalls until one arrives"
                );
                self.stalled = true;
            }
        }
    }

    fn clocks_to_ns(&self, clocks: u64) -> u64 {
        let segment = self
            .clock_segments
            .iter()
            .rev()
            .find(|s| s.from_clocks <= clocks)
            .or(self.clock_segments.first())
            .copied()
            .expect("at least the reset segment");
        let run = u128::from(clocks.saturating_sub(segment.from_clocks)) * 1_000_000_000u128
            / u128::from(segment.hz);
        u64::try_from(run)
            .unwrap_or(u64::MAX)
            .saturating_add(segment.from_ns)
    }

    /// The least cog clock whose instant is at or past `h`: the horizon in
    /// the units the program counts in. `clocks_to_ns` never decreases (a
    /// new segment starts where the old one reached, or later after a
    /// stall), so `c >= this` is exactly `clocks_to_ns(c) >= h`.
    fn ns_to_clocks_ceil(&self, h: u64) -> u64 {
        let s = *self.clock_segments.last().expect("the reset segment");
        let guess = if h <= s.from_ns {
            s.from_clocks
        } else {
            let num = u128::from(h - s.from_ns) * u128::from(s.hz);
            s.from_clocks
                .saturating_add(u64::try_from(num.div_ceil(1_000_000_000)).unwrap_or(u64::MAX))
        };
        if self.clocks_to_ns(guess) >= h && (guess == 0 || self.clocks_to_ns(guess - 1) < h) {
            return guess;
        }
        // An earlier segment: bisect on the mapping itself.
        let (mut lo, mut hi) = (0u64, guess.max(1));
        while self.clocks_to_ns(hi) < h {
            hi = hi.saturating_mul(2);
        }
        while lo < hi {
            let mid = lo + (hi - lo) / 2;
            if self.clocks_to_ns(mid) >= h {
                hi = mid;
            } else {
                lo = mid + 1;
            }
        }
        lo
    }

    /// The machine's "now": the least-advanced running cog, exactly as
    /// p2core's `system_clocks`. Taking the maximum instead lets one cog's
    /// `waitx` drag every other cog's time forward.
    fn machine_now_ns(&self) -> u64 {
        self.clocks_to_ns(self.now_clocks)
    }

    // ---- publishing -------------------------------------------------------

    /// Put every pending pad change on its net. Called from the wake that
    /// fires at the change's own instant, so the engine stamps it right.
    fn publish_pending(&mut self) {
        let mut published = 0u64;
        for pin in 0..NUM_PINS {
            if self.dirty & (1u64 << pin) == 0 {
                continue;
            }
            let want = self.pad_drive(pin);
            if let Some(handle) = self.handles[pin].as_ref() {
                handle.set_drive(want);
                published += 1;
            }
            self.published[pin] = Some(want);
        }
        self.dirty = 0;
        self.pending_at_ns = None;
        self.refresh_strong();
        self.shared
            .publishes
            .fetch_add(published, Ordering::Relaxed);
    }

    /// The pads whose published drive is a strong source.
    fn refresh_strong(&mut self) {
        self.strong = [0; 2];
        for pin in 0..NUM_PINS {
            if let Some(Some(drive)) = self.published[pin] {
                if drive.impedance < WEAK_DRIVE_OHMS {
                    self.strong[pin >> 5] |= 1u32 << (pin & 31);
                }
            }
        }
    }

    /// Fold a STOP into the mirrors: the guest's `DIR`/`OUT`, its changed
    /// mode words and console bytes, its clock, and a pad change to publish.
    fn take_stop(&mut self, stop: &StopHeader, tail: &[u8]) {
        self.dir = stop.dir;
        self.out = stop.out;
        for (pin, word) in stop.modes(tail) {
            self.mode[pin] = word;
        }
        if stop.n_console > 0 {
            self.shared
                .console
                .lock()
                .expect("console never poisoned")
                .extend(stop.console(tail));
        }
        self.now_clocks = stop.now_clocks;
        self.any_running = stop.any_running != 0;
        self.reply_clock_mode = stop.clock_mode;
        self.reply_clock_mode_at = stop.clock_mode_at;
        self.shared
            .slices
            .fetch_add(u64::from(stop.slices), Ordering::Relaxed);
        if stop.reason & reason::YIELD != 0 {
            // Stamped with the segment table as it stood when the guest
            // made the change — before this STOP's clock change.
            self.pending_at_ns = Some(self.clocks_to_ns(stop.pending_at_clocks));
            self.dirty = stop.dirty;
        }
        self.report_unpowered_drives();
    }
}

/// A node with its program: the pads, and the turns.
struct Node {
    pads: Pads,
    peer: Peer,
}

impl Node {
    /// One turn: run the guest from `start_cog` until `horizon_ns`, and fold
    /// the STOP in.
    fn run(&mut self, start_cog: u32, horizon_ns: u64) -> Result<StopHeader, P2QemuError> {
        let (banks_powered, banks_high) = self.pads.bank_masks();
        let run = Run {
            op: OP_RUN,
            start_cog,
            horizon_clocks: self.pads.ns_to_clocks_ceil(horizon_ns),
            in_ext: self.pads.in_ext,
            strong: self.pads.strong,
            banks_powered,
            banks_high,
        };
        let started = Instant::now();
        let stop = self.peer.turn(&run)?;
        let shared = &self.pads.shared;
        shared.turns.fetch_add(1, Ordering::Relaxed);
        shared.turn_ns.fetch_add(
            u64::try_from(started.elapsed().as_nanos()).unwrap_or(u64::MAX),
            Ordering::Relaxed,
        );
        shared
            .run_ns
            .fetch_add(u64::from(stop.run_ns), Ordering::Relaxed);
        self.pads.take_stop(&stop, self.peer.tail());
        Ok(stop)
    }
}

/// One wake: publish a pending pad change at its instant, or run the guest
/// forward until its next one.
fn wake(node: &mut Node, shared: &Arc<Shared>, arm: &P2Pads, now: u64) -> Result<(), P2QemuError> {
    // Replay every transition the nets resolved since the last run before
    // the guest can read a pin, and take the crystal as it stands.
    node.pads.drain_edges();
    node.pads.drain_crystal();
    node.pads.poll_clock_mode(now, arm);
    if node.pads.stalled {
        // No clock, no instructions. The crystal's arrival re-arms.
        shared.stalled.store(true, Ordering::Relaxed);
        return Ok(());
    }
    shared.stalled.store(false, Ordering::Relaxed);

    // A pad change waiting for its own instant.
    if let Some(at) = node.pads.pending_at_ns {
        if at > now {
            arm.schedule_at_ns(at);
            return Ok(());
        }
        // Stamped `now` — which is `at`, or later only if the engine could
        // not stop exactly there. Then one nanosecond on, so the engine
        // resolves the drive and delivers any response before the guest
        // resumes.
        node.pads.publish_pending();
        arm.schedule_at_ns(now.saturating_add(1));
        return Ok(());
    }

    node.pads.follow_supplies();

    // THE GUEST LEADS. Run it forward from its own clock, round-robin over
    // the running cogs as p2core does, until one changes a pad or the
    // horizon is reached. A clock change ends a turn early: the horizon is
    // read again in the new clock's counts and the pass goes on from the
    // next cog.
    let horizon = node.pads.machine_now_ns().saturating_add(SLICE_NS);
    let mut start_cog = 0u32;
    loop {
        let stop = node.run(start_cog, horizon)?;
        let yielded = stop.reason & reason::YIELD != 0;
        if yielded {
            shared.yields.fetch_add(1, Ordering::Relaxed);
        }
        node.pads.poll_clock_mode(now, arm);
        if node.pads.stalled {
            // No clock, no instructions. The crystal's arrival re-arms, and
            // the pending pad change (if the turn made one) is published at
            // that instant by the branch above.
            shared.stalled.store(true, Ordering::Relaxed);
            return Ok(());
        }
        if yielded {
            let at = node.pads.pending_at_ns.unwrap_or(now);
            if at <= now {
                // The change happened at or before the engine's now (a cog
                // that lagged its peers): publish it here and let the
                // engine resolve before the guest goes on.
                node.pads.publish_pending();
                arm.schedule_at_ns(now.saturating_add(1));
            } else {
                arm.schedule_at_ns(at);
            }
            return Ok(());
        }
        if stop.reason & (reason::CLOCK | reason::CONSOLE) != 0 {
            start_cog = (stop.last_cog + 1) % NUM_COGS;
            continue;
        }
        if stop.reason & reason::STALL != 0 {
            tracing::warn!(
                cog = stop.last_cog,
                "p2-qemu: a running cog retired nothing for 1000 slices; giving the engine a \
                 turn"
            );
        }
        break;
    }

    if node.pads.any_running {
        // Strictly forward, always: a guest that did not advance must still
        // let time move, or the engine spins on one instant.
        arm.schedule_at_ns(node.pads.machine_now_ns().max(now.saturating_add(1)));
    } else if !shared.halted.swap(true, Ordering::Relaxed) {
        tracing::info!("p2-qemu: every cog has stopped");
    }
    Ok(())
}

// ============================================================
// The core
// ============================================================

/// A Propeller 2, booting from its ROM, on QEMU: the core inside a
/// [`embsim_boards::p2::P2Package`], with QEMU in a `qemu-system-p2` of its
/// own.
pub struct P2Qemu {
    shared: Arc<Shared>,
    /// The node, shared with the wake closure once attached; taken out (and
    /// the program with it) when the core is dropped or its program fails.
    state: Arc<Mutex<Option<Node>>>,
}

impl std::fmt::Debug for P2Qemu {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("P2Qemu")
            .field("pid", &self.shared.pid.load(Ordering::Relaxed))
            .finish()
    }
}

impl P2Qemu {
    /// Start a `qemu-system-p2` ([`QemuSystemP2::find`]) with `rom` in the
    /// top 16 KB of hub, cog 0 seeded from it, and nothing else in memory:
    /// everything further arrives over the pins. The channel is
    /// [`Transport::from_env`]'s.
    ///
    /// `extra_args` go to QEMU's command line after the node's own (`-d cpu
    /// -D trace.txt` for a state trace to diff against p2core, say).
    pub fn with_boot_rom(rom: &[u8], extra_args: &[&str]) -> Result<Self, P2QemuError> {
        let program = QemuSystemP2::find()?;
        Self::start(&program, rom, extra_args, Transport::from_env()?)
    }

    /// [`Self::with_boot_rom`] with the program and the channel given.
    pub fn start(
        program: &QemuSystemP2,
        rom: &[u8],
        extra_args: &[&str],
        transport: Transport,
    ) -> Result<Self, P2QemuError> {
        let peer = Peer::start(program, rom, extra_args, transport)?;
        let shared = Arc::new(Shared::default());
        shared.pid.store(peer.pid(), Ordering::Relaxed);
        let pads = Pads::new(Arc::clone(&shared));
        Ok(Self {
            shared,
            state: Arc::new(Mutex::new(Some(Node { pads, peer }))),
        })
    }

    /// A view that outlives handing this core to a package and a `System`.
    pub fn handle(&self) -> P2QemuHandle {
        P2QemuHandle {
            shared: Arc::clone(&self.shared),
        }
    }
}

impl Drop for P2Qemu {
    fn drop(&mut self) {
        self.shared.shutdown.store(true, Ordering::Relaxed);
        // The program goes with the core.
        let node = self.state.lock().map(|mut state| state.take());
        drop(node);
    }
}

impl P2Core for P2Qemu {
    fn attach(&mut self, pads: P2Pads) -> Result<(), AttachError> {
        {
            let mut state = self.state.lock().expect("state never poisoned");
            let node = state
                .as_mut()
                .expect("a core is attached once, before it is dropped");
            // The bank supplies: what each pad drives high at.
            node.pads.banks = pads.bank_supplies();
            // Every pad senses its net. The package declares every pad
            // released at attach — a chip out of reset floats every pin —
            // so that is what each net has been told. Floating and
            // contention hold the last level rather than inventing one.
            for pin in 0..NUM_PINS as u8 {
                let handle = pads.pad(pin)?;
                node.pads.published[usize::from(pin)] = Some(None);
                node.pads.handles[usize::from(pin)] = Some(handle);
                let shared = Arc::clone(&self.shared);
                pads.on_pad_sense(pin, move |level| {
                    if shared.shutdown.load(Ordering::Relaxed) {
                        return;
                    }
                    let Some(level) = level else {
                        return;
                    };
                    shared
                        .edges
                        .lock()
                        .expect("edge queue never poisoned")
                        .push_back((pin, level == Level::High));
                })?;
            }
        }

        // The crystal is whatever rate the board delivers on XI. A guest
        // stalled on a crystal-derived clock is woken by its arrival.
        {
            let shared = Arc::clone(&self.shared);
            let arm = pads.clone();
            pads.on_crystal(move |hz| {
                shared.crystal_hz.store(hz.unwrap_or(0), Ordering::Relaxed);
                if shared.stalled.load(Ordering::Relaxed) {
                    arm.schedule_at_ns(virtual_clock::virtual_ns().saturating_add(1));
                }
            });
        }
        // The reset inputs, as information: the package's START gate is
        // what holds the guest on them.
        {
            let shared = Arc::clone(&self.shared);
            pads.on_reset(move |state| {
                *shared.reset.lock().expect("reset state never poisoned") = state;
            });
        }

        let shared = Arc::clone(&self.shared);
        let state = Arc::clone(&self.state);
        let arm = pads.clone();
        pads.on_wake_ns(move |now| {
            if shared.shutdown.load(Ordering::Relaxed) || shared.held.load(Ordering::Relaxed) {
                // Torn down, or held by the package: no clock, no
                // instructions — the stall path, for good.
                return;
            }
            let mut guard = state.lock().expect("state never poisoned");
            let Some(node) = guard.as_mut() else {
                return;
            };
            if let Err(error) = wake(node, &shared, &arm, now) {
                // The program is gone or wedged: the core stops, says why,
                // and the program is reaped with the node.
                let text = error.to_string();
                tracing::error!(error = %text, "p2-qemu: the core stops");
                *shared.failure.lock().expect("failure never poisoned") = Some(text);
                shared.halted.store(true, Ordering::Relaxed);
                shared.held.store(true, Ordering::Relaxed);
                drop(guard.take());
            }
        });
        // The first wake, at once — which the package holds until the START
        // gate opens, so it lands at the START instant (or one nanosecond
        // in, for a bench whose supplies are up from the build). Without it
        // the engine's first look at the guest would be whenever something
        // else scheduled, and the guest's first edge would be stamped there.
        pads.schedule_at_ns(1);
        Ok(())
    }

    /// The START instant: the guest's clock counts from here. Clock
    /// segment 0 — cog clock 0, RCFAST — is anchored at the current virtual
    /// nanosecond, so the first instruction the guest retires is stamped at
    /// the instant the chip could run, never at zero.
    fn start(&mut self) {
        let now = virtual_clock::virtual_ns();
        if let Some(node) = self.state.lock().expect("state never poisoned").as_mut() {
            let segment = node
                .pads
                .clock_segments
                .first_mut()
                .expect("at least the reset segment");
            segment.from_ns = now;
            tracing::info!(
                start_ns = now,
                hz = segment.hz,
                "p2-qemu: START; the guest's clock counts from here"
            );
        }
    }

    /// Held by the package — a brownout without a reset: the guest runs no
    /// further slice, the way a stalled guest runs none, and its pads keep
    /// the drives they last published.
    fn reset(&mut self) {
        self.shared.held.store(true, Ordering::Relaxed);
        tracing::info!("p2-qemu: held by the package; the guest runs no further");
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embsim_boards::p2::{P2_FAST_OHMS, P_HIGH_15K, P_HIGH_1MA, P_HIGH_FLOAT, P_LOW_FAST};

    /// The bench's bank supplies: every `VIO_a_b` at 3.3 V.
    const BENCH_VIO_VOLTS: f64 = 3.3;

    /// Pads with every bank at [`BENCH_VIO_VOLTS`], taken as published,
    /// and nothing published yet: a core once the package has handed it
    /// its table.
    fn bench_pads() -> Pads {
        let mut pads = Pads::new(Arc::new(Shared::default()));
        pads.banks = BankSupplies::held_at(BENCH_VIO_VOLTS);
        pads.published = [Some(None); NUM_PINS];
        pads.follow_supplies();
        pads
    }

    /// What the program reports when the guest set `dir`/`out` on P0..P31
    /// and that is a pad change: the STOP's mirrors and its dirty pads.
    fn guest_drives(pads: &mut Pads, dir: u32, out: u32) {
        pads.dir[0] = dir;
        pads.out[0] = out;
        pads.dirty = (0..NUM_PINS)
            .filter(|&pin| pads.published[pin] != Some(pads.pad_drive(pin)))
            .fold(0, |mask, pin| mask | 1 << pin);
        pads.publish_pending();
    }

    #[test]
    fn a_published_fast_pad_is_strong_and_a_pull_is_not() {
        let mut pads = bench_pads();
        pads.mode[0] = P_HIGH_15K;
        guest_drives(&mut pads, 0b11, 0b11);
        assert_eq!(
            pads.published[0],
            Some(Some(TheveninDrive {
                volts: BENCH_VIO_VOLTS,
                impedance: 15_000.0
            }))
        );
        assert_eq!(
            pads.published[1],
            Some(Some(TheveninDrive {
                volts: BENCH_VIO_VOLTS,
                impedance: P2_FAST_OHMS
            }))
        );
        assert_eq!(pads.strong[0], 0b10, "only the fast pad reads its own OUT");
    }

    #[test]
    fn a_mode_change_on_a_driven_pad_is_published_at_the_new_strength() {
        let mut pads = bench_pads();
        guest_drives(&mut pads, 0b1, 0b1);
        assert_eq!(pads.published[0].unwrap().unwrap().impedance, P2_FAST_OHMS);
        pads.mode[0] = P_HIGH_FLOAT | P_LOW_FAST;
        guest_drives(&mut pads, 0b1, 0b1);
        assert_eq!(pads.published[0], Some(None), "float while OUT is high");
        guest_drives(&mut pads, 0b1, 0b0);
        assert_eq!(
            pads.published[0],
            Some(Some(TheveninDrive {
                volts: 0.0,
                impedance: P2_FAST_OHMS
            }))
        );
        assert_eq!(pads.strong[0], 0b1);
    }

    #[test]
    fn a_current_source_mode_presents_nothing_to_the_net() {
        let mut pads = bench_pads();
        pads.mode[5] = P_HIGH_1MA;
        pads.dir[0] = 1 << 5;
        pads.out[0] = 1 << 5;
        assert_eq!(pads.pad_drive(5), None);
    }

    /// A pad's high is its bank's supply, and a supply that moves takes the
    /// pads driven in its bank with it at the next wake; the banks the next
    /// RUN reports follow too.
    #[test]
    fn a_supply_that_moves_republishes_the_pads_driven_in_its_bank() {
        let mut pads = bench_pads();
        guest_drives(&mut pads, (1 << 4) | (1 << 8), (1 << 4) | (1 << 8));
        assert_eq!(pads.bank_masks(), (0xFFFF, 0xFFFF));

        // A table where every supply reads 1.8 V.
        pads.banks = BankSupplies::held_at(1.8);
        pads.follow_supplies();
        assert_eq!(
            pads.published[4],
            Some(Some(TheveninDrive {
                volts: 1.8,
                impedance: P2_FAST_OHMS
            }))
        );
        assert_eq!(pads.published[8].unwrap().unwrap().volts, 1.8);
        assert_eq!(
            pads.published[5],
            Some(None),
            "an undriven pad stays released"
        );

        // Every supply gone: both pads release.
        pads.banks = BankSupplies::unpowered();
        pads.follow_supplies();
        assert_eq!(pads.published[4], Some(None));
        assert_eq!(pads.published[8], Some(None));
        assert_eq!(pads.bank_masks(), (0, 0));
        assert_eq!(pads.strong[0], 0);

        // A supply at 0 V is powered, and drives no voltage high.
        pads.banks = BankSupplies::held_at(0.0);
        pads.follow_supplies();
        assert_eq!(pads.bank_masks(), (0xFFFF, 0));
        assert_eq!(pads.published[4].unwrap().unwrap().volts, 0.0);
    }

    #[test]
    fn a_pad_driven_in_an_unpowered_bank_is_reported_once() {
        let mut pads = Pads::new(Arc::new(Shared::default()));
        let banks = BankSupplies::unpowered();
        pads.banks = banks.clone();
        pads.dir[0] = 1 << 9;
        pads.report_unpowered_drives();
        pads.report_unpowered_drives();
        assert_eq!(banks.unpowered_banks_driven(), vec![2]);
        assert_eq!(pads.unpowered_reported, 1 << 2);
    }

    /// `%CC` = `%10`, the 15 pF crystal mode: `XI`'s input on.
    const CC_CRYSTAL_15PF: u32 = 0b10 << 2;

    /// HUBSET's word selects the oscillator or multiplies the crystal —
    /// and the crystal is the rate delivered on `XI`, so a PLL word yields
    /// 160 MHz from a delivered 20 MHz and nothing from no crystal.
    #[test]
    fn the_clock_word_selects_the_oscillator_or_multiplies_the_delivered_crystal() {
        let crystal = Some(20_000_000);
        assert_eq!(clock_hz(0, crystal), Some(RCFAST_HZ));
        assert_eq!(clock_hz(0b01, crystal), Some(RCSLOW_HZ));
        let xi = CC_CRYSTAL_15PF | 0b10;
        assert_eq!(clock_hz(xi, crystal), Some(20_000_000));
        // flexspin's 160 MHz from a 20 MHz crystal, `$010007FB`: D=0, M=7,
        // P=%1111, %CC=%10, PLL on.
        let pll = (1 << 24) | (7 << 8) | (0xF << 4) | CC_CRYSTAL_15PF | 0b11;
        assert_eq!(pll, 0x0100_07FB);
        assert_eq!(clock_hz(pll, crystal), Some(160_000_000));
        // P=%0000 divides the VCO by two.
        let halved = (1 << 24) | (15 << 8) | CC_CRYSTAL_15PF | 0b11;
        assert_eq!(clock_hz(halved, crystal), Some(160_000_000));
        // The datasheet's own example (PLL Example, p. 19): a 20 MHz crystal
        // divided by 40 and multiplied by 297, the VCO direct — 148.5 MHz.
        assert_eq!(clock_hz(0x019D_28FB, crystal), Some(148_500_000));

        // No crystal: the internal oscillators still run, nothing derived
        // from XI does.
        assert_eq!(clock_hz(0, None), Some(RCFAST_HZ));
        assert_eq!(clock_hz(xi, None), None);
        assert_eq!(clock_hz(pll, None), None);
        assert!(derives_from_crystal(pll) && derives_from_crystal(xi));
        assert!(!derives_from_crystal(0) && !derives_from_crystal(0b01));
    }

    /// The `%SS` notes (datasheet, System Clock, p. 18): `XI` needs
    /// `%CC` ≠ `%00`, the PLL that and `%E` — a word that selects either
    /// without them has no clock, whatever rate reaches the pin; the
    /// internal oscillators run in any `%CC`.
    #[test]
    fn a_source_the_word_leaves_off_gives_no_clock() {
        let crystal = Some(20_000_000);
        assert_eq!(clock_hz(0b10, crystal), None);
        assert_eq!(clock_hz(0x0100_07F3, crystal), None);
        assert_eq!(clock_hz(0x0000_07FB, crystal), None);
        assert_eq!(clock_hz(0x0100_07F8, crystal), Some(RCFAST_HZ));
        assert_eq!(clock_hz(0b11_01, crystal), Some(RCSLOW_HZ));
        for word in [0b10, 0x0100_07F3, 0x0000_07FB] {
            assert!(!source_runs(word), "{word:#010x}");
        }
        for word in [0, 0b01, 0x0100_07F8, 0x0100_07FB, 0x019D_28FB, 0b01_10] {
            assert!(source_runs(word), "{word:#010x}");
        }
    }

    /// The package delivers the rate on XI; the node takes it at its next
    /// wake and the PLL arithmetic runs on it.
    #[test]
    fn a_delivered_rate_on_xi_is_the_crystal_the_pll_multiplies() {
        let shared = Arc::new(Shared::default());
        let mut pads = Pads::new(Arc::clone(&shared));
        pads.drain_crystal();
        assert_eq!(pads.crystal_hz, None);
        shared.crystal_hz.store(20_000_000, Ordering::Relaxed);
        pads.drain_crystal();
        assert_eq!(pads.crystal_hz, Some(20_000_000));
        shared.crystal_hz.store(0, Ordering::Relaxed);
        pads.drain_crystal();
        assert_eq!(pads.crystal_hz, None);
    }

    #[test]
    fn a_clock_change_keeps_earlier_instants_and_rescales_later_ones() {
        let mut pads = Pads::new(Arc::new(Shared::default()));
        // 1000 clocks of RCFAST at 20 MHz is 50 us.
        assert_eq!(pads.clocks_to_ns(1000), 50_000);
        pads.clock_segments.push(ClockSegment {
            from_clocks: 1000,
            from_ns: 50_000,
            hz: 160_000_000,
        });
        assert_eq!(pads.clocks_to_ns(500), 25_000);
        assert_eq!(pads.clocks_to_ns(1000), 50_000);
        assert_eq!(pads.clocks_to_ns(1160), 51_000);
    }

    /// The horizon a RUN carries, in clocks, is the least clock at or past
    /// the horizon in nanoseconds: the program's `>=` on clocks is the
    /// node's `>=` on instants.
    #[test]
    fn the_horizon_in_clocks_is_the_least_clock_at_or_past_it() {
        let mut pads = Pads::new(Arc::new(Shared::default()));
        pads.clock_segments[0].from_ns = 5_500_000;
        for h in [
            0,
            5_500_000u64,
            5_500_001,
            5_500_049,
            5_500_050,
            5_600_000,
            5_600_013,
        ] {
            let c = pads.ns_to_clocks_ceil(h);
            assert!(pads.clocks_to_ns(c) >= h, "h={h} c={c}");
            assert!(c == 0 || pads.clocks_to_ns(c - 1) < h, "h={h} c={c}");
        }
        // Across a clock change, and after a stall that moved the anchor on.
        pads.clock_segments.push(ClockSegment {
            from_clocks: 2_000,
            from_ns: 5_600_000,
            hz: 160_000_000,
        });
        pads.clock_segments.push(ClockSegment {
            from_clocks: 3_000,
            from_ns: 7_000_000,
            hz: 160_000_000,
        });
        for h in [
            5_550_000u64,
            5_600_000,
            5_600_007,
            6_000_000,
            7_000_000,
            7_000_001,
        ] {
            let c = pads.ns_to_clocks_ceil(h);
            assert!(pads.clocks_to_ns(c) >= h, "h={h} c={c}");
            assert!(c == 0 || pads.clocks_to_ns(c - 1) < h, "h={h} c={c}");
        }
    }

    #[test]
    fn the_facade_is_the_ec32mb_u100_slot() {
        let pins = p2x8c4m64p_pins();
        assert_eq!(pins.len(), 86);
        assert_eq!(pins[0].number, "P0");
        assert_eq!(pins[63].number, "P63");
        assert!(pins.iter().any(|p| p.number == "VIO_56_59"));
        assert!(pins.iter().any(|p| p.number == "RESN"));
    }
}
