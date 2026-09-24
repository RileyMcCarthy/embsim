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
//! hands the core its 64 pads, and delivers the two package-level facts —
//! the **crystal**, which is the rate the board puts on `XI` (a TCXO
//! through its buffer on the P2-EC32MB), and the `RESN`/`VDD` state. A
//! board fills its processor slot with `P2Package::new(P2Qemu::…)`.
//!
//! # Where the CPU runs
//!
//! On the engine thread, called from its own wake — the same shape `p2iss`
//! runs `p2core` in. QEMU is linked in as a library, its vCPU thread parks at
//! start-up, and each wake runs the cogs for a bounded slice
//! (`hostdrive.c`). A pin edge is therefore a function call, not a
//! cross-thread hand-off: spike 1d measured a slice at 172–254 ns against
//! 10 900 ns for a park/wake, and a boot spends ~16 600 edges on a kilobyte.
//!
//! # How an edge gets its instant
//!
//! **The guest leads; the engine follows it to each edge.** A wake runs the
//! cogs forward from their own clocks. The moment one changes a pad, the bus
//! stops that cog after the instruction (`p2_pinbus_yield`), records the
//! cog's clock as the edge's instant, and the wake ends by arming itself at
//! that instant. The engine advances there, the next wake **publishes** the
//! drive — so it is stamped at the guest's own instant, never the wake's —
//! and arms again one nanosecond on, so the engine resolves the net and
//! delivers any response (a flash presenting its next bit) before the guest
//! reads anything back. Two wakes per edge, each edge at its true instant,
//! and a device on the net sees every transition — the rules
//! `docs/dev/sil-unified-drive.md` sets out, R1 through R5.
//!
//! # What a pad is, and what it reads
//!
//! A pad the guest drives is a Thevenin source at the strength its `WRPIN`
//! word configured (`embsim_boards::p2::pad_drive`): fast at
//! [`embsim_boards::p2::P2_FAST_OHMS`], the 1.5 k / 15 k / 150 kΩ modes at
//! those resistances, float released. A `WRPIN` on a driven pad is a pad
//! change like a `DIR` write — it yields, and the next wake republishes the
//! pad at the new strength. The current-source modes are not mapped: such a
//! pad presents nothing, and the node says so once.
//!
//! `IN` reflects the **net** for every pad whose published drive is
//! released or a pull (at or above [`WEAK_DRIVE_OHMS`]) — a pad pulling a
//! line high through 15 kΩ reads what the line resolved to, which is what
//! makes an I2C slave's clock stretch and its ACK visible to the master
//! driving through the pull mode. A pad driven fast keeps reading its own
//! `OUT` bit, which is what p2core does and what keeps the two engines'
//! state traces identical instruction for instruction.
//!
//! # One machine per process
//!
//! QEMU's init is process-global and not repeatable. The first
//! [`P2Qemu::with_boot_rom`] boots it; a second one in the same process is
//! refused. Put each system that needs a P2 in its own test binary.

#![warn(missing_docs)]

use std::collections::VecDeque;
use std::ffi::CString;
use std::os::raw::{c_char, c_int, c_uint, c_void};
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

use embsim_board::{level_of, AttachError, Level, PinHandle, TheveninDrive, WEAK_DRIVE_OHMS};
use embsim_boards::p2::{self, P2Core, P2Pads, P2ResetState, PadDrive, LOGIC_HIGH_VOLTS};
use embsim_core::virtual_clock;

pub use embsim_boards::p2::{p2x8c4m64p_pins, pin_name};

mod ffi;
pub mod flashimage;

/// How far past its own clock one wake may run the guest, in nanoseconds.
/// Only bounds the work per wake; the engine is re-armed at wherever the
/// guest got to.
pub const SLICE_NS: u64 = 100_000;

/// Instructions one cog runs before the next cog gets a turn. Round-robin
/// TCG's own quantum, and what spike 0c measured the firmware tolerates.
///
/// This is an icount budget, and in hub space one instruction is one unit.
/// In cog space the unit is one interpreter RUN of up to this many
/// instructions (the whole run is one translation block), so a cog-space
/// slice takes a budget of one and still retires up to the quantum.
pub const COG_QUANTUM: i64 = 48;

/// Where hub space begins in the unified PC: below it a cog runs interpreted.
const HUB_EXEC_BASE: u32 = 0x400;

const NUM_COGS: usize = 8;
const NUM_PINS: usize = p2::NUM_PADS;

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
pub fn clock_hz(mode: u32, crystal_hz: Option<u64>) -> Option<u64> {
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

/// Cog register addresses the bus is told about.
const REG_DIRA: c_uint = 0x1FA;
const REG_DIRB: c_uint = 0x1FB;
const REG_OUTA: c_uint = 0x1FC;
const REG_OUTB: c_uint = 0x1FD;

/// Smart-pin mode field values the bus interprets. The field (`%SSSSS`) is
/// bits 5..1 of the WRPIN word, above the `%0` in bit 0 — the target's own
/// transition-mode test is `(cfg & 0x3F) == 0x0A` for `%00101`.
const SMART_ASYNC_TX: u32 = 0b11110;
const SMART_ASYNC_RX: u32 = 0b11111;
const SMART_SYNC_RX: u32 = 0b11101;

/// The `%SSSSS` smart-pin mode of a WRPIN word; zero is plain GPIO.
const fn smart_mode(cfg: u32) -> u32 {
    (cfg >> 1) & 0x1F
}
/// Pin-configuration field (`%MMMMMMMMMMMMM`, bits 20:8) values `$10`..`$17`
/// in the high byte select the ADC modes (`P_ADC_GIO` .. `P_ADC_100X`).
const PIN_CFG_ADC_MASK: u32 = 0x00F8_0000;
const PIN_CFG_ADC: u32 = 0x0010_0000;

// ============================================================
// Errors
// ============================================================

/// Why a node could not be created.
#[derive(Debug)]
pub enum P2QemuError {
    /// This build carries no QEMU: `EMBSIM_QEMU_P2_BUILD` was unset when the
    /// crate compiled.
    Unavailable,
    /// QEMU's init is process-global; a second machine cannot be booted.
    AlreadyBooted,
    /// The ROM image could not be staged for `-bios`.
    Io(std::io::Error),
}

impl std::fmt::Display for P2QemuError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            P2QemuError::Unavailable => write!(
                f,
                "embsim-p2-qemu was built without a QEMU tree; set EMBSIM_QEMU_P2_BUILD to a \
                 configured QEMU build with the p2 target and rebuild"
            ),
            P2QemuError::AlreadyBooted => {
                write!(
                    f,
                    "QEMU is already booted in this process; one P2 node per process"
                )
            }
            P2QemuError::Io(e) => write!(f, "staging the boot ROM: {e}"),
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
    /// The reset inputs, as the package last delivered them. Recorded, not
    /// yet acted on: the START gate is `NODES.md` §8 phase 4's, with the
    /// rails that make `VDD` read a voltage.
    reset: Mutex<P2ResetState>,
    /// The guest selected a clock derived from the crystal while none
    /// reached `XI`, and is not running until one does.
    stalled: AtomicBool,
    /// Times the guest stopped on a pad change.
    yields: AtomicU64,
    /// Drives published to nets.
    publishes: AtomicU64,
    /// Slices run.
    slices: AtomicU64,
    /// Every cog has stopped.
    halted: AtomicBool,
    /// The node is being torn down; wakes do nothing.
    shutdown: AtomicBool,
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

    /// Whether every cog has stopped.
    pub fn halted(&self) -> bool {
        self.shared.halted.load(Ordering::Relaxed)
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
}

// ============================================================
// The bus: the electrical side of the seam
// ============================================================

/// Said once per process: a pad was configured in a current-source drive
/// mode, which the node does not map.
static CURRENT_SOURCE_LOGGED: AtomicBool = AtomicBool::new(false);

/// The pin model behind QEMU's `P2PinBusOps`.
///
/// Reached from C through a raw pointer while a slice runs, and from the wake
/// closure between slices. Both happen on the engine thread and never at the
/// same time, which is the whole aliasing argument: a callback runs only
/// inside `p2host_slice`, which the wake calls with no borrow of the bus held.
struct Bus {
    /// `DIRx`/`OUTx` are PER-COG registers and the pad sees the OR across all
    /// eight. Mirroring them globally lets one cog's write erase another's —
    /// p2core saw a console print on P62 reset the SD card's shifter that way.
    dir_cog: [[u32; 2]; NUM_COGS],
    out_cog: [[u32; 2]; NUM_COGS],
    dir: [u32; 2],
    out: [u32; 2],
    /// What the nets present, last known. A pin whose net is floating or in
    /// contention keeps its last level: an unresolvable net is not a logic
    /// value, and inventing one would hide the fault.
    in_ext: [u32; 2],
    /// Pads whose **published** drive is a strong source (under
    /// [`WEAK_DRIVE_OHMS`]): these read their own `OUT` bit; every other pad
    /// reads its net. Recomputed at each publish.
    strong: [u32; 2],

    mode: [u32; NUM_PINS],
    x: [u32; NUM_PINS],
    in_flag: [bool; NUM_PINS],

    /// The drive each net was last told: `None` never, `Some(None)`
    /// released, `Some(Some(drive))` a Thevenin source.
    published: [Option<Option<TheveninDrive>>; NUM_PINS],
    /// Pins whose wanted drive differs from what was published.
    dirty: u64,
    /// The instant of the pending pad change: the driving cog's clock at the
    /// instruction. `Some` between the yield and the publish.
    pending_at_ns: Option<u64>,

    handles: Vec<Option<PinHandle>>,
    /// The crystal on `XI`, as last drained from the package's delivery.
    crystal_hz: Option<u64>,
    /// The crystal the current clock segment was derived from, to notice a
    /// change that matters.
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

/// One constant-frequency stretch of the guest's clock.
#[derive(Debug, Clone, Copy)]
struct ClockSegment {
    from_clocks: u64,
    from_ns: u64,
    hz: u64,
}

impl Bus {
    fn new(shared: Arc<Shared>) -> Self {
        Self {
            dir_cog: [[0; 2]; NUM_COGS],
            out_cog: [[0; 2]; NUM_COGS],
            dir: [0; 2],
            out: [0; 2],
            in_ext: [0; 2],
            strong: [0; 2],
            mode: [0; NUM_PINS],
            x: [0; NUM_PINS],
            in_flag: [false; NUM_PINS],
            published: [None; NUM_PINS],
            dirty: 0,
            pending_at_ns: None,
            handles: (0..NUM_PINS).map(|_| None).collect(),
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

    // ---- what a bank reads --------------------------------------------------

    /// What the guest drives where its published drive is strong, and what
    /// the outside presents everywhere else — a released pad and a pad
    /// pulling through a resistive mode both read their net. This one line
    /// is why the ROM can use P61 as both a strap and a chip select, why it
    /// can float P58 to read the flash, and why a pad pulling a line high
    /// sees the sink that is holding it low.
    fn sensed(&self, bank: usize) -> u32 {
        let strong = self.strong[bank];
        (strong & self.out[bank]) | (!strong & self.in_ext[bank])
    }

    fn pad_level(&self, pin: usize) -> bool {
        (self.sensed(pin >> 5) >> (pin & 31)) & 1 != 0
    }

    /// The drive the guest presents on `pin`: its `DIR`/`OUT` bits through
    /// the strength its `WRPIN` word configured, or `None` when the pad is
    /// released. A current-source mode is not mapped and presents nothing.
    fn pad_drive(&self, pin: usize) -> Option<TheveninDrive> {
        let bit = 1u32 << (pin & 31);
        let bank = pin >> 5;
        let dir = self.dir[bank] & bit != 0;
        let out = self.out[bank] & bit != 0;
        match p2::pad_drive(self.mode[pin], dir, out, LOGIC_HIGH_VOLTS) {
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

    // ---- time -----------------------------------------------------------------

    /// Notice a HUBSET clock change, or the crystal arriving or changing
    /// under a clock derived from it, and start a new segment: at the
    /// instant the guest made the change, or — for a crystal that arrived
    /// while the guest was stalled — at `now`, the wake that found it.
    /// Cheap when nothing changed: two loads and two compares.
    fn poll_clock_mode(&mut self, now: u64) {
        // SAFETY: plain reads of two globals the target owns.
        let mode = unsafe { ffi::p2host_clock_mode() };
        let mode_changed = mode != self.clock_mode;
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
            unsafe { ffi::p2host_clock_mode_at() }
        } else {
            self.machine_now_clocks()
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

    fn cog_clocks(cog: usize) -> u64 {
        // SAFETY: cog < NUM_COGS; the machine exists for the node's lifetime.
        unsafe { ffi::p2host_cog_clocks(cog as c_uint) }
    }

    fn cog_ns(&self, cog: usize) -> u64 {
        self.clocks_to_ns(Self::cog_clocks(cog))
    }

    fn cog_running(cog: usize) -> bool {
        // SAFETY: as above.
        unsafe { ffi::p2host_cog_running(cog as c_uint) }
    }

    /// The machine's "now" in clocks: the least-advanced running cog, the
    /// same rule as [`Self::machine_now_ns`].
    fn machine_now_clocks(&self) -> u64 {
        let running = (0..NUM_COGS)
            .filter(|&c| Self::cog_running(c))
            .map(Self::cog_clocks)
            .min();
        running.unwrap_or_else(|| (0..NUM_COGS).map(Self::cog_clocks).max().unwrap_or(0))
    }

    /// The machine's "now": the least-advanced running cog, exactly as
    /// p2core's `system_clocks`. Taking the maximum instead lets one cog's
    /// `waitx` drag every other cog's time forward.
    fn machine_now_ns(&self) -> u64 {
        let running = (0..NUM_COGS)
            .filter(|&c| Self::cog_running(c))
            .map(|c| self.cog_ns(c))
            .min();
        running.unwrap_or_else(|| (0..NUM_COGS).map(|c| self.cog_ns(c)).max().unwrap_or(0))
    }

    // ---- pad changes ----------------------------------------------------------

    /// Recompute the OR-reduced DIR/OUT and note every pad whose drive —
    /// level and strength — differs from what its net was last told.
    /// Returns whether a change is newly pending: something is dirty and no
    /// instant has been taken for it yet.
    fn mark_changes(&mut self) -> bool {
        self.dir = [0; 2];
        self.out = [0; 2];
        for c in 0..NUM_COGS {
            self.dir[0] |= self.dir_cog[c][0];
            self.dir[1] |= self.dir_cog[c][1];
            self.out[0] |= self.out_cog[c][0];
            self.out[1] |= self.out_cog[c][1];
        }
        let mut changed = 0u64;
        for pin in 0..NUM_PINS {
            let want = self.pad_drive(pin);
            if self.published[pin] != Some(want) {
                changed |= 1u64 << pin;
            }
        }
        self.dirty = changed;
        self.dirty != 0 && self.pending_at_ns.is_none()
    }

    /// [`Self::mark_changes`] from a bus callback: the first change in an
    /// instruction sets the pending instant to the executing cog's clock
    /// and stops the cog; later ones in the same instruction (DRVH
    /// publishes OUT then DIR) share it.
    fn recompute_and_mark(&mut self, cog: usize) {
        if self.mark_changes() {
            self.pending_at_ns = Some(self.cog_ns(cog));
            // SAFETY: called from a bus callback, on the thread running the
            // slice; stops the executing cog after this instruction.
            unsafe { ffi::p2host_request_yield() };
        }
    }

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

    // ---- the vtable, in Rust --------------------------------------------------

    fn dir_out_changed(&mut self, cog: c_uint, reg: c_uint, value: u32) {
        let c = (cog as usize) & (NUM_COGS - 1);
        match reg {
            REG_DIRA => self.dir_cog[c][0] = value,
            REG_DIRB => self.dir_cog[c][1] = value,
            REG_OUTA => self.out_cog[c][0] = value,
            REG_OUTB => self.out_cog[c][1] = value,
            _ => return,
        }
        self.recompute_and_mark(c);
    }

    /// A mode write. On a pad the guest is driving, a new drive strength is
    /// a pad change like a `DIR` write: it takes the executing cog's
    /// instant and stops the cog, and the wake republishes the pad.
    fn wrpin(&mut self, pin: c_uint, cfg: u32) {
        let p = (pin as usize) & 63;
        // AKPIN assembles as `WRPIN #1,S` and never arrives as a distinct op:
        // a cfg of 1 is an acknowledge, not a mode write. Treating it as a
        // mode would wipe the pin's configuration.
        if cfg == 1 {
            self.in_flag[p] = false;
            return;
        }
        self.mode[p] = cfg;
        self.in_flag[p] = cfg != 0;
        if self.dir[p >> 5] & (1u32 << (p & 31)) != 0 {
            // SAFETY: inside a bus callback, where the executing cog is
            // defined (`hostdrive.c`).
            let cog = unsafe { ffi::p2host_current_cog() } as usize;
            self.recompute_and_mark(cog & (NUM_COGS - 1));
        }
    }

    fn wxpin(&mut self, pin: c_uint, x: u32) {
        let p = (pin as usize) & 63;
        self.x[p] = x;
        self.in_flag[p] = true;
    }

    fn wypin(&mut self, pin: c_uint, y: u32) {
        let p = (pin as usize) & 63;
        // Recorded regardless of mode: a boot chain's whole observable is
        // often one byte written here, and it means the program loaded off
        // the flash really ran. See `Shared::console`.
        let byte = (y & 0xFF) as u8;
        tracing::debug!(
            pin = p,
            byte,
            transmitter = smart_mode(self.mode[p]) == SMART_ASYNC_TX,
            "p2-qemu: WYPIN"
        );
        self.shared
            .console
            .lock()
            .expect("console never poisoned")
            .push((p as u8, byte));
        self.in_flag[p] = true;
    }

    fn pin_cfg(&self, pin: c_uint) -> u32 {
        self.mode[(pin as usize) & 63]
    }

    fn rdpin(&mut self, pin: c_uint) -> (u32, bool) {
        let p = (pin as usize) & 63;
        self.in_flag[p] = false;
        // $FF reads as an idle, pulled-high line; C reports BUSY and nothing
        // here ever is.
        (0xFF, false)
    }

    fn testp(&self, pin: c_uint) -> bool {
        let p = (pin as usize) & 63;
        let mode = self.mode[p];
        // In an ADC mode IN carries the sigma-delta bit stream, not the pad's
        // logic level. The boot ROM's very first act is to seed its RNG by
        // sampling the RX pin in ADC-calibration mode 1550 times; the stream
        // is not modelled, and the reference (p2core) reads it as zeros, so a
        // bus that reported the pad level — high, behind the board's pull-up
        // — diverges on the fifth instruction of the boot. Deterministic and
        // matching is what a differential harness needs from it.
        if mode & PIN_CFG_ADC_MASK == PIN_CFG_ADC {
            return false;
        }
        // A pin with no smart-pin mode is plain GPIO and TESTP reads its
        // LEVEL, not an IN flag. That is the path the boot ROM takes to read
        // the flash — it floats P58 and samples it — and the path a GPIO
        // driver's `_pinr()` compiles to.
        if smart_mode(mode) == 0 {
            return self.pad_level(p);
        }
        // A receiver reports "a byte is waiting". No serial peer exists on
        // the net yet, so nothing is ever waiting.
        if matches!(smart_mode(mode), SMART_ASYNC_RX | SMART_SYNC_RX) {
            return false;
        }
        // Any other configured pin completes its operation at once, so its
        // IN flag reads set whether or not a WXPIN/WYPIN has raised it.
        let _ = self.in_flag[p];
        true
    }

    fn akpin(&mut self, pin: c_uint) {
        self.in_flag[(pin as usize) & 63] = false;
    }
}

// The C entry points: each recovers the bus from the opaque pointer QEMU was
// handed at install and forwards.

unsafe extern "C" fn cb_ina(o: *mut c_void) -> u32 {
    (*o.cast::<Bus>()).sensed(0)
}
unsafe extern "C" fn cb_inb(o: *mut c_void) -> u32 {
    (*o.cast::<Bus>()).sensed(1)
}
unsafe extern "C" fn cb_dir_out_changed(o: *mut c_void, cog: c_uint, reg: c_uint, value: u32) {
    (*o.cast::<Bus>()).dir_out_changed(cog, reg, value);
}
unsafe extern "C" fn cb_wrpin(o: *mut c_void, pin: c_uint, cfg: u32) {
    (*o.cast::<Bus>()).wrpin(pin, cfg);
}
unsafe extern "C" fn cb_wxpin(o: *mut c_void, pin: c_uint, x: u32) {
    (*o.cast::<Bus>()).wxpin(pin, x);
}
unsafe extern "C" fn cb_wypin(o: *mut c_void, pin: c_uint, y: u32) {
    (*o.cast::<Bus>()).wypin(pin, y);
}
unsafe extern "C" fn cb_pin_cfg(o: *mut c_void, pin: c_uint) -> u32 {
    (*o.cast::<Bus>()).pin_cfg(pin)
}
unsafe extern "C" fn cb_rdpin(o: *mut c_void, pin: c_uint, busy: *mut bool) -> u32 {
    let (value, is_busy) = (*o.cast::<Bus>()).rdpin(pin);
    if !busy.is_null() {
        *busy = is_busy;
    }
    value
}
unsafe extern "C" fn cb_testp(o: *mut c_void, pin: c_uint) -> bool {
    (*o.cast::<Bus>()).testp(pin)
}
unsafe extern "C" fn cb_akpin(o: *mut c_void, pin: c_uint) {
    (*o.cast::<Bus>()).akpin(pin);
}

static BUS_OPS: ffi::P2PinBusOps = ffi::P2PinBusOps {
    ina: cb_ina,
    inb: cb_inb,
    dir_out_changed: cb_dir_out_changed,
    wrpin: cb_wrpin,
    wxpin: cb_wxpin,
    wypin: cb_wypin,
    pin_cfg: cb_pin_cfg,
    rdpin: cb_rdpin,
    testp: cb_testp,
    akpin: cb_akpin,
};

/// The bus, as the one pointer both QEMU and the wake closure hold.
///
/// Leaked on purpose: QEMU keeps the pointer in its vtable for the life of
/// the process, and only the wake ever runs the machine, so nothing can reach
/// the bus after the node is gone — but nothing can free it safely either.
#[derive(Clone, Copy)]
struct BusPtr(*mut Bus);

impl BusPtr {
    /// The pointer, through a method so a closure captures the whole
    /// (`Send`) wrapper rather than its raw field.
    fn get(self) -> *mut Bus {
        self.0
    }
}

// SAFETY: the pointer is only ever dereferenced on the engine thread — from
// the wake closure, and from QEMU callbacks that run inside a slice the wake
// called. Construction and attach touch it before any wake exists.
unsafe impl Send for BusPtr {}
unsafe impl Sync for BusPtr {}

// ============================================================
// The core
// ============================================================

static BOOTED: AtomicBool = AtomicBool::new(false);

thread_local! {
    /// Whether THIS thread has registered with RCU and TCG.
    static THREAD_ATTACHED: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

/// A Propeller 2, booting from its ROM, on QEMU: the core inside a
/// [`embsim_boards::p2::P2Package`].
pub struct P2Qemu {
    shared: Arc<Shared>,
    bus: BusPtr,
    /// Where the ROM was staged for `-bios`. Removed on drop.
    rom_path: PathBuf,
}

impl std::fmt::Debug for P2Qemu {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("P2Qemu")
            .field("rom_path", &self.rom_path)
            .finish()
    }
}

impl P2Qemu {
    /// Boot QEMU with `rom` in the top 16 KB of hub, cog 0 seeded from it, and
    /// nothing else in memory: everything further arrives over the pins.
    ///
    /// `extra_args` go to QEMU's command line after the node's own (`-d cpu
    /// -D trace.txt` for a state trace to diff against p2core, say).
    pub fn with_boot_rom(rom: &[u8], extra_args: &[&str]) -> Result<Self, P2QemuError> {
        if !ffi::linked() {
            return Err(P2QemuError::Unavailable);
        }
        if BOOTED.swap(true, Ordering::SeqCst) {
            return Err(P2QemuError::AlreadyBooted);
        }

        let rom_path =
            std::env::temp_dir().join(format!("embsim-p2-qemu-{}-rom.bin", std::process::id()));
        std::fs::write(&rom_path, rom)?;

        let shared = Arc::new(Shared::default());
        let bus = Box::into_raw(Box::new(Bus::new(Arc::clone(&shared))));

        let mut args: Vec<String> = [
            "embsim-p2-qemu",
            "-M",
            "p2",
            "-accel",
            "tcg",
            // One instruction is one nanosecond of QEMU's own virtual clock,
            // and — what matters here — the slice budget is an instruction
            // count that is honoured exactly (spike 1d).
            "-icount",
            "shift=0,sleep=off",
            "-display",
            "none",
            "-monitor",
            "none",
            "-serial",
            "none",
            "-parallel",
            "none",
            "-bios",
        ]
        .iter()
        .map(|s| s.to_string())
        .collect();
        args.push(rom_path.to_string_lossy().into_owned());
        args.extend(extra_args.iter().map(|s| s.to_string()));

        let cstrings: Vec<CString> = args
            .iter()
            .map(|a| CString::new(a.as_str()).expect("no NUL in a QEMU argument"))
            .collect();
        let mut argv: Vec<*mut c_char> =
            cstrings.iter().map(|c| c.as_ptr() as *mut c_char).collect();
        argv.push(std::ptr::null_mut());

        tracing::info!(?args, "p2-qemu: booting QEMU as a library");
        // SAFETY: argv outlives the call and is NULL-terminated; QEMU copies
        // what it keeps. The vtable and its opaque outlive the process.
        unsafe {
            ffi::p2host_boot((argv.len() - 1) as c_int, argv.as_mut_ptr());
            ffi::p2host_install_bus(&BUS_OPS, bus.cast::<c_void>());
        }

        Ok(Self {
            shared,
            bus: BusPtr(bus),
            rom_path,
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
        let _ = std::fs::remove_file(&self.rom_path);
    }
}

impl P2Core for P2Qemu {
    fn attach(&mut self, pads: P2Pads) -> Result<(), AttachError> {
        let ptr = self.bus;
        // SAFETY: attach runs before any wake exists; nothing else holds the bus.
        let bus = unsafe { &mut *ptr.get() };

        // Every pad senses its net. The package declares every pad released
        // at attach — a chip out of reset floats every pin — so that is what
        // each net has been told. Floating and contention hold the last
        // level rather than inventing one.
        for pin in 0..NUM_PINS as u8 {
            let handle = pads.pad(pin)?;
            bus.published[usize::from(pin)] = Some(None);
            bus.handles[usize::from(pin)] = Some(handle);
            let shared = Arc::clone(&self.shared);
            pads.on_pad_sense(pin, move |state| {
                if shared.shutdown.load(Ordering::Relaxed) {
                    return;
                }
                let Some(level) = level_of(state) else {
                    return;
                };
                shared
                    .edges
                    .lock()
                    .expect("edge queue never poisoned")
                    .push_back((pin, level == Level::High));
            })?;
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
        // The reset inputs: recorded for the START gate (phase 4).
        {
            let shared = Arc::clone(&self.shared);
            pads.on_reset(move |state| {
                *shared.reset.lock().expect("reset state never poisoned") = state;
            });
        }

        let shared = Arc::clone(&self.shared);
        let arm = pads.clone();
        pads.on_wake_ns(move |now| {
            if shared.shutdown.load(Ordering::Relaxed) {
                return;
            }
            THREAD_ATTACHED.with(|attached| {
                if !attached.get() {
                    // SAFETY: once per thread, before its first slice.
                    unsafe { ffi::p2host_attach_thread() };
                    attached.set(true);
                }
            });
            // SAFETY: the engine thread, between slices; the only borrow.
            let bus = unsafe { &mut *ptr.get() };
            wake(bus, &shared, &arm, now);
        });
        // The first wake, at once. Without it the engine's first look at the
        // guest would be whenever something else scheduled, and the guest's
        // first edge would be stamped there.
        pads.schedule_at_ns(1);
        Ok(())
    }
}

/// One wake: publish a pending pad change at its instant, or run the guest
/// forward until its next one.
fn wake(bus: &mut Bus, shared: &Arc<Shared>, arm: &P2Pads, now: u64) {
    // Replay every transition the nets resolved since the last slice before
    // the guest can read a pin, and take the crystal as it stands.
    bus.drain_edges();
    bus.drain_crystal();
    bus.poll_clock_mode(now);
    if bus.stalled {
        // No clock, no instructions. The crystal's arrival re-arms.
        shared.stalled.store(true, Ordering::Relaxed);
        return;
    }
    shared.stalled.store(false, Ordering::Relaxed);

    // A pad change waiting for its own instant.
    if let Some(at) = bus.pending_at_ns {
        if at > now {
            arm.schedule_at_ns(at);
            return;
        }
        // Stamped `now` — which is `at`, or later only if the engine could not
        // stop exactly there. Then one nanosecond on, so the engine resolves
        // the drive and delivers any response before the guest resumes.
        bus.publish_pending();
        arm.schedule_at_ns(now.saturating_add(1));
        return;
    }

    // THE GUEST LEADS. Run it forward from its own clock, round-robin over
    // the running cogs as p2core does, until one changes a pad or the
    // horizon is reached.
    let horizon = bus.machine_now_ns().saturating_add(SLICE_NS);
    let mut stalls = 0u32;
    'run: loop {
        if bus.machine_now_ns() >= horizon {
            break;
        }
        let mut stepped = false;
        for cog in 0..NUM_COGS {
            if !Bus::cog_running(cog) || bus.cog_ns(cog) >= horizon {
                continue;
            }
            // SAFETY: cog < NUM_COGS, on the attached engine thread.
            let before = unsafe { ffi::p2host_cog_clocks(cog as c_uint) };
            let budget = if unsafe { ffi::p2host_cog_pc(cog as c_uint) } < HUB_EXEC_BASE {
                1
            } else {
                COG_QUANTUM
            };
            let result = unsafe { ffi::p2host_slice(cog as c_uint, budget) };
            shared.slices.fetch_add(1, Ordering::Relaxed);
            stepped = true;
            // The slice's yield is taken with the slice, whatever else it
            // did: a guest that selects a crystal-derived clock and changes
            // a pad in the same slice stalls with that change pending, and
            // the flag is the change's — consumed here, so it cannot fire
            // as a phantom pad change on the slice after the clock arrives.
            // SAFETY: as above.
            let yielded = unsafe { ffi::p2host_take_yield() };
            if yielded {
                shared.yields.fetch_add(1, Ordering::Relaxed);
            }
            bus.poll_clock_mode(now);
            if bus.stalled {
                // No clock, no instructions. The crystal's arrival re-arms,
                // and the pending pad change (if the slice made one) is
                // published at that instant by the branch above.
                shared.stalled.store(true, Ordering::Relaxed);
                return;
            }
            if yielded {
                let at = bus.pending_at_ns.unwrap_or(now);
                if at <= now {
                    // The change happened at or before the engine's now (a
                    // cog that lagged its peers): publish it here and let
                    // the engine resolve before the guest goes on.
                    bus.publish_pending();
                    arm.schedule_at_ns(now.saturating_add(1));
                } else {
                    arm.schedule_at_ns(at);
                }
                return;
            }
            // SAFETY: as above.
            let after = unsafe { ffi::p2host_cog_clocks(cog as c_uint) };
            if after == before {
                stalls += 1;
                if stalls > 1_000 {
                    tracing::warn!(
                        cog,
                        result,
                        "p2-qemu: a running cog retired nothing for 1000 slices; giving \
                         the engine a turn"
                    );
                    break 'run;
                }
            } else {
                stalls = 0;
            }
        }
        if !stepped {
            break;
        }
    }

    if (0..NUM_COGS).any(Bus::cog_running) {
        // Strictly forward, always: a guest that did not advance must still
        // let time move, or the engine spins on one instant.
        arm.schedule_at_ns(bus.machine_now_ns().max(now.saturating_add(1)));
    } else if !shared.halted.swap(true, Ordering::Relaxed) {
        tracing::info!("p2-qemu: every cog has stopped");
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embsim_boards::p2::{P2_FAST_OHMS, P_HIGH_15K, P_HIGH_1MA, P_HIGH_FLOAT, P_LOW_FAST};

    /// A bus with pads `dir`/`out` set by cog 0 and every change published
    /// (to no net: the handles are empty), so `sensed` reads what a guest
    /// would after the wake that publishes.
    fn bus_driving(dir: u32, out: u32) -> Bus {
        let mut bus = Bus::new(Arc::new(Shared::default()));
        bus.published = [Some(None); NUM_PINS];
        bus.dir_cog[0][0] = dir;
        bus.out_cog[0][0] = out;
        assert!(bus.mark_changes() || dir == 0);
        bus.publish_pending();
        bus
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

    #[test]
    fn a_bank_reads_the_guest_where_driven_fast_and_the_net_elsewhere() {
        let mut bus = bus_driving(0b0011, 0b0001);
        bus.in_ext = [0b1100, 0];
        assert_eq!(bus.sensed(0), 0b1101);
        assert_eq!(
            bus.pad_drive(0),
            Some(TheveninDrive {
                volts: LOGIC_HIGH_VOLTS,
                impedance: P2_FAST_OHMS
            })
        );
        assert_eq!(
            bus.pad_drive(1),
            Some(TheveninDrive {
                volts: 0.0,
                impedance: P2_FAST_OHMS
            })
        );
        assert_eq!(bus.pad_drive(2), None);
    }

    /// The `sensed()` rule: a pad whose published drive is a pull reads its
    /// NET, so a master driving SCL high through 15 kΩ sees a slave holding
    /// it low, and a released pad reads its net as it always did. Only a
    /// fast pad reads its own `OUT` bit.
    #[test]
    fn a_pulling_pad_reads_its_net_and_a_fast_pad_its_own_out_bit() {
        let mut bus = Bus::new(Arc::new(Shared::default()));
        bus.published = [Some(None); NUM_PINS];
        bus.mode[0] = P_HIGH_15K;
        bus.dir_cog[0][0] = 0b11;
        bus.out_cog[0][0] = 0b11;
        assert!(bus.mark_changes());
        bus.publish_pending();
        assert_eq!(
            bus.published[0],
            Some(Some(TheveninDrive {
                volts: LOGIC_HIGH_VOLTS,
                impedance: 15_000.0
            }))
        );
        assert_eq!(bus.strong[0], 0b10, "only the fast pad is strong");

        // The net resolved low (a sink on it): the pulling pad reads 0.
        bus.set_input_level(0, false);
        bus.set_input_level(1, false);
        assert!(!bus.testp(0), "the pull-up reads the sink holding the line");
        assert!(bus.testp(1), "the fast pad reads its own OUT bit");
        // The sink lets go and the net rises: the pull-up reads 1.
        bus.set_input_level(0, true);
        assert!(bus.testp(0));
    }

    /// A `WRPIN` on a driven pad is a pad change: the drive the net is told
    /// changes strength, so the pad is dirty again and republishes.
    #[test]
    fn a_mode_change_on_a_driven_pad_republishes_it_at_the_new_strength() {
        let mut bus = bus_driving(0b1, 0b1);
        assert_eq!(bus.dirty, 0);
        assert_eq!(bus.published[0].unwrap().unwrap().impedance, P2_FAST_OHMS);

        bus.mode[0] = P_HIGH_15K;
        assert!(bus.mark_changes(), "a strength change is a pad change");
        assert_eq!(bus.dirty, 0b1);
        bus.publish_pending();
        assert_eq!(bus.published[0].unwrap().unwrap().impedance, 15_000.0);
        assert_eq!(bus.strong[0], 0);

        // Float while OUT = 1: released; fast while OUT = 0: a sink.
        bus.mode[0] = P_HIGH_FLOAT | P_LOW_FAST;
        assert!(bus.mark_changes());
        bus.publish_pending();
        assert_eq!(bus.published[0], Some(None));
        bus.out_cog[0][0] = 0;
        assert!(bus.mark_changes());
        bus.publish_pending();
        assert_eq!(
            bus.published[0],
            Some(Some(TheveninDrive {
                volts: 0.0,
                impedance: P2_FAST_OHMS
            }))
        );
        assert_eq!(bus.strong[0], 0b1);

        // The same word again changes nothing.
        assert!(!bus.mark_changes());
    }

    #[test]
    fn a_current_source_mode_presents_nothing_to_the_net() {
        let mut bus = Bus::new(Arc::new(Shared::default()));
        bus.published = [Some(None); NUM_PINS];
        bus.mode[5] = P_HIGH_1MA;
        bus.dir_cog[0][0] = 1 << 5;
        bus.out_cog[0][0] = 1 << 5;
        assert!(!bus.mark_changes(), "released to released is no change");
        assert_eq!(bus.pad_drive(5), None);
    }

    #[test]
    fn testp_reads_the_level_of_an_unconfigured_pin_and_no_byte_on_a_receiver() {
        let mut bus = Bus::new(Arc::new(Shared::default()));
        bus.in_ext = [1 << 5, 0];
        assert!(bus.testp(5));
        assert!(!bus.testp(6));
        // The ROM's RNG seed: ADC-calibration mode reads the bit stream, not
        // the pad, even with the pad high behind a pull-up.
        bus.in_ext[1] |= 1 << 31;
        bus.wrpin(63, 0x0010_0000);
        assert!(!bus.testp(63));
        bus.wrpin(63, 0);
        assert!(bus.testp(63));
        bus.wrpin(53, SMART_ASYNC_RX << 1);
        assert!(!bus.testp(53));
        bus.wrpin(62, SMART_ASYNC_TX << 1);
        assert!(bus.testp(62));
    }

    /// HUBSET's word selects the oscillator or multiplies the crystal —
    /// and the crystal is the rate delivered on `XI`, so a PLL word yields
    /// 160 MHz from a delivered 20 MHz and nothing from no crystal.
    #[test]
    fn the_clock_word_selects_the_oscillator_or_multiplies_the_delivered_crystal() {
        let crystal = Some(20_000_000);
        assert_eq!(clock_hz(0, crystal), Some(RCFAST_HZ));
        assert_eq!(clock_hz(0b01, crystal), Some(RCSLOW_HZ));
        assert_eq!(clock_hz(0b10, crystal), Some(20_000_000));
        // flexspin's 160 MHz from a 20 MHz crystal: D=0, M=7, P=%1111, PLL on.
        let pll = (1 << 24) | (7 << 8) | (0xF << 4) | 0b11;
        assert_eq!(clock_hz(pll, crystal), Some(160_000_000));
        // P=%0000 divides the VCO by two.
        let halved = (1 << 24) | (15 << 8) | 0b11;
        assert_eq!(clock_hz(halved, crystal), Some(160_000_000));

        // No crystal: the internal oscillators still run, nothing derived
        // from XI does.
        assert_eq!(clock_hz(0, None), Some(RCFAST_HZ));
        assert_eq!(clock_hz(0b10, None), None);
        assert_eq!(clock_hz(pll, None), None);
        assert!(derives_from_crystal(pll) && derives_from_crystal(0b10));
        assert!(!derives_from_crystal(0) && !derives_from_crystal(0b01));
    }

    /// The package delivers the rate on XI; the bus takes it at its next
    /// wake and the PLL arithmetic runs on it.
    #[test]
    fn a_delivered_rate_on_xi_is_the_crystal_the_pll_multiplies() {
        let shared = Arc::new(Shared::default());
        let mut bus = Bus::new(Arc::clone(&shared));
        bus.drain_crystal();
        assert_eq!(bus.crystal_hz, None);
        shared.crystal_hz.store(20_000_000, Ordering::Relaxed);
        bus.drain_crystal();
        assert_eq!(bus.crystal_hz, Some(20_000_000));
        let pll = (1 << 24) | (7 << 8) | (0xF << 4) | 0b11;
        assert_eq!(clock_hz(pll, bus.crystal_hz), Some(160_000_000));
        shared.crystal_hz.store(0, Ordering::Relaxed);
        bus.drain_crystal();
        assert_eq!(bus.crystal_hz, None);
    }

    #[test]
    fn a_clock_change_keeps_earlier_instants_and_rescales_later_ones() {
        let mut bus = Bus::new(Arc::new(Shared::default()));
        // 1000 clocks of RCFAST at 20 MHz is 50 us.
        assert_eq!(bus.clocks_to_ns(1000), 50_000);
        bus.clock_segments.push(ClockSegment {
            from_clocks: 1000,
            from_ns: 50_000,
            hz: 160_000_000,
        });
        assert_eq!(bus.clocks_to_ns(500), 25_000);
        assert_eq!(bus.clocks_to_ns(1000), 50_000);
        assert_eq!(bus.clocks_to_ns(1160), 51_000);
    }

    #[test]
    fn wrpin_one_is_an_acknowledge_not_a_mode() {
        let mut bus = Bus::new(Arc::new(Shared::default()));
        bus.wrpin(58, 0x2C);
        bus.wxpin(58, 7);
        assert!(bus.in_flag[58]);
        bus.wrpin(58, 1);
        assert!(!bus.in_flag[58]);
        assert_eq!(bus.pin_cfg(58), 0x2C);
    }
}
