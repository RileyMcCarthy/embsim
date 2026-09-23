//! The QEMU Propeller 2 target as an embsim board component.
//!
//! A `P2X8C4M64P` that boots the way silicon does: the real Parallax ROM in
//! the top of hub, and everything else arriving over pins that are **nets** —
//! a flash on the board answers the ROM's bit-banged SPI, and the P2 sees it
//! the way it sees any other component. The node knows nothing about flashes,
//! cards or UARTs. It drives pads and senses pads.
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

use embsim_board::{
    digital_drive, level_of, AttachError, Component, ComponentNetIo, Level, PinDecl, PinHandle,
    PinKind,
};

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
const NUM_PINS: usize = 64;

/// The internal RC oscillator the chip comes up on, nominal. Silicon runs
/// RCFAST anywhere in 20–24 MHz; 20 MHz is Parallax's stated figure.
pub const RCFAST_HZ: u64 = 20_000_000;
/// The slow internal oscillator (`%01`), nominal.
pub const RCSLOW_HZ: u64 = 20_000;
/// The crystal a P2-EC32MB and a P2 Eval carry, and the default here.
pub const DEFAULT_CRYSTAL_HZ: u64 = 20_000_000;

/// The system clock a HUBSET clock word selects.
///
/// `%0000_000E_DDDD_DDMM_MMMM_MMMM_PPPP_CC_SS`: `SS` picks RCFAST, RCSLOW,
/// the crystal, or the PLL; the PLL runs the crystal through `/(D+1)`,
/// `*(M+1)`, and a final divider `PPPP` — `%1111` is the VCO direct,
/// anything else `/((P+1)*2)`. What the guest records in hub `$14` is NOT
/// used: the boot ROM overwrites that long with its base64 table, and the
/// clock is a fact of the hardware the guest set, not a value it stored.
pub fn clock_hz(mode: u32, crystal_hz: u64) -> u64 {
    match mode & 0b11 {
        0b00 => RCFAST_HZ,
        0b01 => RCSLOW_HZ,
        0b10 => crystal_hz,
        _ => {
            let d = u64::from((mode >> 18) & 0x3F) + 1;
            let m = u64::from((mode >> 8) & 0x3FF) + 1;
            let p = (mode >> 4) & 0xF;
            let pdiv = if p == 0xF { 1 } else { (u64::from(p) + 1) * 2 };
            (crystal_hz * m / d / pdiv).max(1)
        }
    }
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

/// `P0`..`P63`, as the netlists name them.
static P_NAMES: [&str; NUM_PINS] = [
    "P0", "P1", "P2", "P3", "P4", "P5", "P6", "P7", "P8", "P9", "P10", "P11", "P12", "P13", "P14",
    "P15", "P16", "P17", "P18", "P19", "P20", "P21", "P22", "P23", "P24", "P25", "P26", "P27",
    "P28", "P29", "P30", "P31", "P32", "P33", "P34", "P35", "P36", "P37", "P38", "P39", "P40",
    "P41", "P42", "P43", "P44", "P45", "P46", "P47", "P48", "P49", "P50", "P51", "P52", "P53",
    "P54", "P55", "P56", "P57", "P58", "P59", "P60", "P61", "P62", "P63",
];

/// The name of I/O pin `pin` on the facade.
pub fn pin_name(pin: u8) -> &'static str {
    P_NAMES[usize::from(pin & 63)]
}

/// The package's other pins, as the P2-EC32MB netlist normalises them: the
/// rails, the reset and test inputs, and the crystal pair.
const PACKAGE_PINS: [(&str, PinKind); 22] = [
    ("GND", PinKind::PowerIn),
    ("VDD", PinKind::PowerIn),
    ("RESN", PinKind::DigitalIn),
    ("TEST", PinKind::DigitalIn),
    ("XI", PinKind::DigitalIn),
    // XO drives the crystal; nothing on a board reads it as logic.
    ("XO", PinKind::Passive),
    ("VIO_0_3", PinKind::PowerIn),
    ("VIO_4_7", PinKind::PowerIn),
    ("VIO_8_11", PinKind::PowerIn),
    ("VIO_12_15", PinKind::PowerIn),
    ("VIO_16_19", PinKind::PowerIn),
    ("VIO_20_23", PinKind::PowerIn),
    ("VIO_24_27", PinKind::PowerIn),
    ("VIO_28_31", PinKind::PowerIn),
    ("VIO_32_35", PinKind::PowerIn),
    ("VIO_36_39", PinKind::PowerIn),
    ("VIO_40_43", PinKind::PowerIn),
    ("VIO_44_47", PinKind::PowerIn),
    ("VIO_48_51", PinKind::PowerIn),
    ("VIO_52_55", PinKind::PowerIn),
    ("VIO_56_59", PinKind::PowerIn),
    ("VIO_60_63", PinKind::PowerIn),
];

/// The chip's facade: 64 bidirectional I/O pins plus the 22 package pins.
/// This is what the P2-EC32MB's `U100` slot expects, in both directions.
pub fn p2x8c4m64p_pins() -> Vec<PinDecl> {
    let mut pins = Vec::with_capacity(NUM_PINS + PACKAGE_PINS.len());
    for name in P_NAMES {
        pins.push(PinDecl {
            number: name,
            name: None,
            kind: PinKind::DigitalBidir,
            stream: None,
            drive_impedance: None,
        });
    }
    for (name, kind) in PACKAGE_PINS {
        pins.push(PinDecl {
            number: name,
            name: None,
            kind,
            stream: None,
            drive_impedance: None,
        });
    }
    pins
}

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
}

// ============================================================
// The bus: the electrical side of the seam
// ============================================================

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

    mode: [u32; NUM_PINS],
    x: [u32; NUM_PINS],
    in_flag: [bool; NUM_PINS],

    /// The drive each net was last told: `None` never, `Some(None)` released,
    /// `Some(Some(level))` driven.
    published: [Option<Option<bool>>; NUM_PINS],
    /// Pins whose wanted drive differs from what was published.
    dirty: u64,
    /// The instant of the pending pad change: the driving cog's clock at the
    /// instruction. `Some` between the yield and the publish.
    pending_at_ns: Option<u64>,

    handles: Vec<Option<PinHandle>>,
    /// The crystal on the board's XI pin, for the PLL arithmetic.
    crystal_hz: u64,
    /// The HUBSET clock word last seen, to notice a change.
    clock_mode: u32,
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
            mode: [0; NUM_PINS],
            x: [0; NUM_PINS],
            in_flag: [false; NUM_PINS],
            published: [None; NUM_PINS],
            dirty: 0,
            pending_at_ns: None,
            handles: (0..NUM_PINS).map(|_| None).collect(),
            crystal_hz: DEFAULT_CRYSTAL_HZ,
            clock_mode: 0,
            clock_segments: vec![ClockSegment {
                from_clocks: 0,
                from_ns: 0,
                hz: RCFAST_HZ,
            }],
            shared,
        }
    }

    // ---- what a bank reads --------------------------------------------------

    /// What the guest drives where DIR is set, and what the outside presents
    /// everywhere else. This one line is why the ROM can use P61 as both a
    /// strap and a chip select, and why it can float P58 to read the flash.
    fn sensed(&self, bank: usize) -> u32 {
        let driven = self.dir[bank];
        (driven & self.out[bank]) | (!driven & self.in_ext[bank])
    }

    fn pad_level(&self, pin: usize) -> bool {
        (self.sensed(pin >> 5) >> (pin & 31)) & 1 != 0
    }

    /// The level the guest drives on `pin`, or `None` when it has released it.
    fn output_level(&self, pin: usize) -> Option<bool> {
        let bit = 1u32 << (pin & 31);
        let bank = pin >> 5;
        if self.dir[bank] & bit != 0 {
            Some(self.out[bank] & bit != 0)
        } else {
            None
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

    // ---- time -----------------------------------------------------------------

    /// Notice a HUBSET clock change and start a new segment at the instant
    /// the guest made it. Cheap when nothing changed: one load and a compare.
    fn poll_clock_mode(&mut self) {
        // SAFETY: plain reads of two globals the target owns.
        let mode = unsafe { ffi::p2host_clock_mode() };
        if mode == self.clock_mode {
            return;
        }
        let at = unsafe { ffi::p2host_clock_mode_at() };
        let hz = clock_hz(mode, self.crystal_hz);
        let from_ns = self.clocks_to_ns(at);
        tracing::info!(
            mode = format_args!("{mode:#010x}"),
            hz,
            at,
            "p2-qemu: clock set"
        );
        self.clock_mode = mode;
        self.clock_segments.push(ClockSegment {
            from_clocks: at,
            from_ns,
            hz,
        });
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

    fn cog_ns(&self, cog: usize) -> u64 {
        // SAFETY: cog < NUM_COGS; the machine exists for the node's lifetime.
        self.clocks_to_ns(unsafe { ffi::p2host_cog_clocks(cog as c_uint) })
    }

    fn cog_running(cog: usize) -> bool {
        // SAFETY: as above.
        unsafe { ffi::p2host_cog_running(cog as c_uint) }
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

    /// Recompute the OR-reduced DIR/OUT and note every pad whose drive changed.
    /// The first change in an instruction sets the pending instant and stops
    /// the cog; later ones in the same instruction (DRVH publishes OUT then
    /// DIR) share it.
    fn recompute_and_mark(&mut self, cog: usize) {
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
            let want = self.output_level(pin);
            if self.published[pin] != Some(want) {
                changed |= 1u64 << pin;
            }
        }
        if changed != self.dirty {
            self.dirty = changed;
        }
        if self.dirty != 0 && self.pending_at_ns.is_none() {
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
            let want = self.output_level(pin);
            if let Some(handle) = self.handles[pin].as_ref() {
                handle.set_drive(
                    want.map(|high| digital_drive(if high { Level::High } else { Level::Low })),
                );
                published += 1;
            }
            self.published[pin] = Some(want);
        }
        self.dirty = 0;
        self.pending_at_ns = None;
        self.shared
            .publishes
            .fetch_add(published, Ordering::Relaxed);
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
// The component
// ============================================================

static BOOTED: AtomicBool = AtomicBool::new(false);

thread_local! {
    /// Whether THIS thread has registered with RCU and TCG.
    static THREAD_ATTACHED: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

/// A Propeller 2, booting from its ROM, on QEMU.
pub struct P2Qemu {
    pins: Vec<PinDecl>,
    shared: Arc<Shared>,
    bus: BusPtr,
    /// Where the ROM was staged for `-bios`. Removed on drop.
    rom_path: PathBuf,
}

impl std::fmt::Debug for P2Qemu {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("P2Qemu")
            .field("pins", &self.pins.len())
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
            pins: p2x8c4m64p_pins(),
            shared,
            bus: BusPtr(bus),
            rom_path,
        })
    }

    /// The crystal on `XI`, for the PLL the guest programs with HUBSET.
    /// Defaults to the 20 MHz a P2-EC32MB carries.
    #[must_use]
    pub fn with_crystal_hz(self, hz: u64) -> Self {
        // SAFETY: before attach, nothing else holds the bus.
        unsafe { (*self.bus.get()).crystal_hz = hz.max(1) };
        self
    }

    /// A view that outlives handing this component to a `System`.
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

impl Component for P2Qemu {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let ptr = self.bus;
        // SAFETY: attach runs before any wake exists; nothing else holds the bus.
        let bus = unsafe { &mut *ptr.get() };

        // Every I/O pin senses its net, and starts RELEASED. The engine's idle
        // default for a bidirectional pin is driven high, which would make
        // the ROM's P61 strap read the P2's own drive rather than the board's
        // pull-up, and contend with the flash on P58, until the guest's first
        // DIR write happened to publish every pad at once. A chip out of
        // reset floats every pin; say so before anything can sample one.
        // Floating and contention hold the last level rather than inventing
        // one.
        for pin in 0..NUM_PINS as u8 {
            let handle = io.pin(pin_name(pin))?;
            handle.set_drive(None);
            bus.published[usize::from(pin)] = Some(None);
            bus.handles[usize::from(pin)] = Some(handle);
            let shared = Arc::clone(&self.shared);
            io.on_sense(pin_name(pin), move |state| {
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

        let shared = Arc::clone(&self.shared);
        let arm = io.clone();
        io.on_wake_ns(move |now| {
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
        io.schedule_at_ns(1);
        Ok(())
    }
}

/// One wake: publish a pending pad change at its instant, or run the guest
/// forward until its next one.
fn wake(bus: &mut Bus, shared: &Arc<Shared>, arm: &ComponentNetIo, now: u64) {
    // Replay every transition the nets resolved since the last slice before
    // the guest can read a pin.
    bus.drain_edges();
    bus.poll_clock_mode();

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
            bus.poll_clock_mode();
            // SAFETY: as above.
            if unsafe { ffi::p2host_take_yield() } {
                shared.yields.fetch_add(1, Ordering::Relaxed);
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
    fn a_bank_reads_the_guest_where_driven_and_the_net_elsewhere() {
        let mut bus = Bus::new(Arc::new(Shared::default()));
        bus.dir = [0b0011, 0];
        bus.out = [0b0001, 0];
        bus.in_ext = [0b1100, 0];
        assert_eq!(bus.sensed(0), 0b1101);
        assert_eq!(bus.output_level(0), Some(true));
        assert_eq!(bus.output_level(1), Some(false));
        assert_eq!(bus.output_level(2), None);
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

    #[test]
    fn the_clock_word_selects_the_oscillator_or_the_pll() {
        assert_eq!(clock_hz(0, 20_000_000), RCFAST_HZ);
        assert_eq!(clock_hz(0b01, 20_000_000), RCSLOW_HZ);
        assert_eq!(clock_hz(0b10, 20_000_000), 20_000_000);
        // flexspin's 160 MHz from a 20 MHz crystal: D=0, M=7, P=%1111, PLL on.
        let mode = (1 << 24) | (7 << 8) | (0xF << 4) | 0b11;
        assert_eq!(clock_hz(mode, 20_000_000), 160_000_000);
        // P=%0000 divides the VCO by two.
        let mode = (1 << 24) | (15 << 8) | 0b11;
        assert_eq!(clock_hz(mode, 20_000_000), 160_000_000);
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
