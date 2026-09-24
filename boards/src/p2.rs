//! The Propeller 2 **package** as a node: [`P2Package`], the 86-pin
//! `P2X8C4M64P` facade around a *core* that runs the chip.
//!
//! The package is what a board sees: 64 I/O pads, the core and bank
//! supplies, reset and test, and the crystal pair. It declares those pins
//! once, delivers what the board puts on them to the core, and puts on the
//! nets what the core drives. The core is whatever executes instructions —
//! QEMU's target (`embsim-p2-qemu`), an instruction-set simulator, or the
//! native firmware image behind [`McuComponent`] — and it never sees a net:
//! it is handed [`P2Pads`], which is exactly the surface `NODES.md` §11
//! gives a node, narrowed to the pads.
//!
//! A P2 with **no** core is a real state too: every pad released, the
//! rails and `RESN` sensed, `XI` accepting the board's rate. That is what
//! any P2 is before it runs, and [`P2Package::held_in_reset`] is it — the
//! node that fills a module's processor slot for tests about the board.
//!
//! # What the package declares (`NODES.md` §2, "MCU node (P2)")
//!
//! | Pins | Kind | Idle |
//! |---|---|---|
//! | `P0`..`P63` | `DigitalBidir` | **released** — a pad out of reset floats |
//! | `VDD`, `GND`, the 16 `VIO_a_b` | `PowerIn` | — |
//! | `RESN`, `TEST` | `DigitalIn` (sensed) | — |
//! | `XI` | `DigitalIn` + [`StreamRole::PulseSink`]: the rate delivered here **is the crystal** | — |
//! | `XO` | `DigitalOut`, released | the crystal driver, unused with an external clock |
//!
//! # Pad drive strength (the `WRPIN` pin-configuration field)
//!
//! A pad is a Thevenin source at the strength the guest configured. The
//! `WRPIN` word's pin-configuration field `%PPPPPPPPPPPPP` (bits 20:8)
//! is, in its logic modes, `%0000_CIO_HHH_LLL`: `HHH` (bits 13:11) is the
//! drive while `OUT` = 1 and `LLL` (bits 10:8) the drive while `OUT` = 0,
//! each one of
//!
//! | field | mode | here |
//! |---|---|---|
//! | `%000` | fast | [`P2_FAST_OHMS`] (a named placeholder, see its TODO) |
//! | `%001` | 1.5 kΩ | [`P2_PULL_1K5_OHMS`] |
//! | `%010` | 15 kΩ | [`P2_PULL_15K_OHMS`] |
//! | `%011` | 150 kΩ | [`P2_PULL_150K_OHMS`] |
//! | `%100` | 1 mA | not mapped ([`PadDrive::CurrentSource`]) |
//! | `%101` | 100 µA | not mapped |
//! | `%110` | 10 µA | not mapped |
//! | `%111` | float | released |
//!
//! Source: Parallax, *Propeller 2 (P2X8C4M64P) Silicon Documentation*,
//! rev. v35, "Smart Pins" → "Pin Configuration Modes" — the
//! `%0000_CIO_HHH_LLL` logic-mode word and its `HHH`/`LLL` drive table
//! (fast, 1.5k, 15k, 150k, 1mA, 100uA, 10uA, float). The three resistive
//! modes are the resistances the document names; the current-source modes
//! are **not** mapped to a resistor (a V/I approximation would be an
//! invention, `DESIGN.md` rule 6) and reach a core as
//! [`PadDrive::CurrentSource`], an unimplemented path the core logs once.
//! `Drive::Current` is the encoding they take when a caller exists.
//!
//! # What this package does not do yet
//!
//! - **The START gate** (`NODES.md` §8 phase 4): the package delivers the
//!   `RESN`/`VDD` state ([`P2ResetState`]) to the core, and the core
//!   records it; holding the core's first wake until the rails and reset
//!   read up lands with the rail models, since today no rail on a bare
//!   module carries a voltage.
//! - **The bank supply as the pad's high level**: a pad drives high at
//!   [`LOGIC_HIGH_VOLTS`], the nominal 3.3 V every push-pull output in the
//!   crate drives at, until the rails are modelled and the package can
//!   hand each pad its own bank's `VIO`.

use std::sync::{Arc, Mutex};

use embsim_board::{
    level_of, AttachError, Component, ComponentNetIo, IdleDrive, Level, McuComponent, NetState,
    Ohms, PinDecl, PinHandle, PinKind, PulseTrain, StreamRole, TheveninDrive, Volts,
};

/// Rail a pad drives high at until the bank supplies are modelled rails:
/// the nominal 3.3 V of [`embsim_board::net::LOGIC_HIGH_VOLTS`].
pub const LOGIC_HIGH_VOLTS: Volts = embsim_board::net::LOGIC_HIGH_VOLTS;

/// I/O pads on the package.
pub const NUM_PADS: usize = 64;

/// Pins on the package: the 64 pads and the 22 others.
pub const NUM_PINS: usize = NUM_PADS + PACKAGE_PINS.len();

// ============================================================
// Pad drive strengths
// ============================================================

/// A pad in **fast** drive mode (`%000`), as a Thevenin impedance.
///
/// TODO(provenance): a placeholder, the crate's default push-pull
/// impedance. The value to measure is in the Parallax *P2X8C4M64P Data
/// Sheet*, "DC Characteristics" → I/O pin output drive (`V_OH`/`V_OL` at
/// the rated `I_OH`/`I_OL`), which gives the fast driver's effective
/// source resistance at the 3.3 V bank supply. Every projection on the
/// three reference boards ranks a fast pad against pulls of 10 kΩ and
/// more, so the placeholder decides nothing today.
pub const P2_FAST_OHMS: Ohms = embsim_board::net::DEFAULT_PUSH_PULL_IMPEDANCE;

/// The `%001` drive mode: 1.5 kΩ (Silicon Doc, pin configuration table).
pub const P2_PULL_1K5_OHMS: Ohms = 1_500.0;
/// The `%010` drive mode: 15 kΩ (Silicon Doc, pin configuration table).
pub const P2_PULL_15K_OHMS: Ohms = 15_000.0;
/// The `%011` drive mode: 150 kΩ (Silicon Doc, pin configuration table).
pub const P2_PULL_150K_OHMS: Ohms = 150_000.0;

/// Bit position of the `HHH` field (the drive while `OUT` = 1) in a
/// `WRPIN` word: bits 13:11.
pub const P_HIGH_SHIFT: u32 = 11;
/// Bit position of the `LLL` field (the drive while `OUT` = 0): bits 10:8.
pub const P_LOW_SHIFT: u32 = 8;
/// The pin-configuration field's mode selector, bits 20:17: `%0000` is a
/// logic mode (`%0000_CIO_HHH_LLL`); anything else is a DAC, ADC or
/// comparator mode whose low bits mean something else.
const P_MODE_SELECT_MASK: u32 = 0x001E_0000;

/// `WRPIN` pin-configuration constants for the eight `OUT` = 1 drive
/// modes, as the Spin2 compiler names them (`P_HIGH_*`).
pub const P_HIGH_FAST: u32 = 0b000 << P_HIGH_SHIFT;
/// See [`P_HIGH_FAST`].
pub const P_HIGH_1K5: u32 = 0b001 << P_HIGH_SHIFT;
/// See [`P_HIGH_FAST`].
pub const P_HIGH_15K: u32 = 0b010 << P_HIGH_SHIFT;
/// See [`P_HIGH_FAST`].
pub const P_HIGH_150K: u32 = 0b011 << P_HIGH_SHIFT;
/// See [`P_HIGH_FAST`].
pub const P_HIGH_1MA: u32 = 0b100 << P_HIGH_SHIFT;
/// See [`P_HIGH_FAST`].
pub const P_HIGH_100UA: u32 = 0b101 << P_HIGH_SHIFT;
/// See [`P_HIGH_FAST`].
pub const P_HIGH_10UA: u32 = 0b110 << P_HIGH_SHIFT;
/// See [`P_HIGH_FAST`].
pub const P_HIGH_FLOAT: u32 = 0b111 << P_HIGH_SHIFT;

/// `WRPIN` pin-configuration constants for the eight `OUT` = 0 drive
/// modes (`P_LOW_*`).
pub const P_LOW_FAST: u32 = 0b000 << P_LOW_SHIFT;
/// See [`P_LOW_FAST`].
pub const P_LOW_1K5: u32 = 0b001 << P_LOW_SHIFT;
/// See [`P_LOW_FAST`].
pub const P_LOW_15K: u32 = 0b010 << P_LOW_SHIFT;
/// See [`P_LOW_FAST`].
pub const P_LOW_150K: u32 = 0b011 << P_LOW_SHIFT;
/// See [`P_LOW_FAST`].
pub const P_LOW_1MA: u32 = 0b100 << P_LOW_SHIFT;
/// See [`P_LOW_FAST`].
pub const P_LOW_100UA: u32 = 0b101 << P_LOW_SHIFT;
/// See [`P_LOW_FAST`].
pub const P_LOW_10UA: u32 = 0b110 << P_LOW_SHIFT;
/// See [`P_LOW_FAST`].
pub const P_LOW_FLOAT: u32 = 0b111 << P_LOW_SHIFT;

/// One of the eight drive modes a `HHH`/`LLL` field selects.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PadMode {
    /// `%000`: the fast push-pull driver.
    Fast,
    /// `%001`: 1.5 kΩ.
    Ohms1k5,
    /// `%010`: 15 kΩ.
    Ohms15k,
    /// `%011`: 150 kΩ.
    Ohms150k,
    /// `%100`: a 1 mA current source (not mapped).
    Current1mA,
    /// `%101`: a 100 µA current source (not mapped).
    Current100uA,
    /// `%110`: a 10 µA current source (not mapped).
    Current10uA,
    /// `%111`: the pad floats.
    Float,
}

impl PadMode {
    /// The mode a three-bit field selects.
    pub const fn from_field(bits: u32) -> Self {
        match bits & 0b111 {
            0b000 => PadMode::Fast,
            0b001 => PadMode::Ohms1k5,
            0b010 => PadMode::Ohms15k,
            0b011 => PadMode::Ohms150k,
            0b100 => PadMode::Current1mA,
            0b101 => PadMode::Current100uA,
            0b110 => PadMode::Current10uA,
            _ => PadMode::Float,
        }
    }

    /// The mode's Thevenin impedance, for the modes that have one.
    pub const fn ohms(self) -> Option<Ohms> {
        match self {
            PadMode::Fast => Some(P2_FAST_OHMS),
            PadMode::Ohms1k5 => Some(P2_PULL_1K5_OHMS),
            PadMode::Ohms15k => Some(P2_PULL_15K_OHMS),
            PadMode::Ohms150k => Some(P2_PULL_150K_OHMS),
            PadMode::Current1mA | PadMode::Current100uA | PadMode::Current10uA | PadMode::Float => {
                None
            }
        }
    }

    /// Whether the mode is one of the current sources the package does not
    /// map.
    pub const fn is_current_source(self) -> bool {
        matches!(
            self,
            PadMode::Current1mA | PadMode::Current100uA | PadMode::Current10uA
        )
    }
}

/// The `(OUT = 1, OUT = 0)` drive modes a `WRPIN` word configures.
///
/// A word whose mode selector (bits 20:17) is not `%0000` is a DAC, ADC or
/// comparator configuration whose low bits are not drive modes; a pad the
/// guest drives in one of those reads as **fast** both ways — the drive a
/// pad had before any of this was decoded, and the smallest claim about a
/// mode the package does not model.
pub const fn pad_modes(cfg: u32) -> (PadMode, PadMode) {
    if cfg & P_MODE_SELECT_MASK != 0 {
        return (PadMode::Fast, PadMode::Fast);
    }
    (
        PadMode::from_field(cfg >> P_HIGH_SHIFT),
        PadMode::from_field(cfg >> P_LOW_SHIFT),
    )
}

/// What a pad the guest is driving presents to its net.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PadDrive {
    /// High-impedance: `DIR` clear, or the float mode.
    Released,
    /// A Thevenin source: the level through the mode's impedance.
    Thevenin(TheveninDrive),
    /// One of the current-source modes — not mapped. A core treats it as
    /// released and says so once.
    CurrentSource(PadMode),
}

/// The drive a pad with `DIR` set presents, from its `WRPIN` word and its
/// `OUT` bit: high at `high_volts` through the `HHH` mode's impedance, low
/// at 0 V through the `LLL` mode's. A pad with `DIR` clear is
/// [`PadDrive::Released`] whatever the word says.
pub fn pad_drive(cfg: u32, dir: bool, out: bool, high_volts: Volts) -> PadDrive {
    if !dir {
        return PadDrive::Released;
    }
    let (high, low) = pad_modes(cfg);
    let mode = if out { high } else { low };
    match mode.ohms() {
        Some(impedance) => PadDrive::Thevenin(TheveninDrive {
            volts: if out { high_volts } else { 0.0 },
            impedance,
        }),
        None if mode.is_current_source() => PadDrive::CurrentSource(mode),
        None => PadDrive::Released,
    }
}

// ============================================================
// The facade
// ============================================================

/// `P0`..`P63`, as the netlists name them.
static PAD_NAMES: [&str; NUM_PADS] = [
    "P0", "P1", "P2", "P3", "P4", "P5", "P6", "P7", "P8", "P9", "P10", "P11", "P12", "P13", "P14",
    "P15", "P16", "P17", "P18", "P19", "P20", "P21", "P22", "P23", "P24", "P25", "P26", "P27",
    "P28", "P29", "P30", "P31", "P32", "P33", "P34", "P35", "P36", "P37", "P38", "P39", "P40",
    "P41", "P42", "P43", "P44", "P45", "P46", "P47", "P48", "P49", "P50", "P51", "P52", "P53",
    "P54", "P55", "P56", "P57", "P58", "P59", "P60", "P61", "P62", "P63",
];

/// The name of pad `pin` (taken modulo 64) on the facade.
pub fn pin_name(pin: u8) -> &'static str {
    PAD_NAMES[usize::from(pin & 63)]
}

/// The package's other pins, as the P2-EC32MB netlist normalises them: the
/// rails, the reset and test inputs, and the crystal pair.
const PACKAGE_PINS: [(&str, PinKind); 22] = [
    ("GND", PinKind::PowerIn),
    ("VDD", PinKind::PowerIn),
    ("RESN", PinKind::DigitalIn),
    ("TEST", PinKind::DigitalIn),
    ("XI", PinKind::DigitalIn),
    ("XO", PinKind::DigitalOut),
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

/// The chip's facade: 64 bidirectional pads that idle released, and the 22
/// package pins — the supplies as `PowerIn`, `RESN` and `TEST` sensed, `XI`
/// a pulse sink whose delivered rate is the crystal, `XO` a released
/// output. This is what the P2-EC32MB's `U100` slot expects, in both
/// directions.
pub fn p2x8c4m64p_pins() -> Vec<PinDecl> {
    let mut pins = Vec::with_capacity(NUM_PINS);
    for name in PAD_NAMES {
        pins.push(PinDecl::new(name, PinKind::DigitalBidir).with_idle(IdleDrive::Released));
    }
    for (name, kind) in PACKAGE_PINS {
        let pin = PinDecl::new(name, kind);
        pins.push(match name {
            "XI" => pin.with_stream(StreamRole::PulseSink),
            "XO" => pin.with_idle(IdleDrive::Released),
            _ => pin,
        });
    }
    pins
}

// ============================================================
// What the package delivers to its core
// ============================================================

/// The reset inputs as the package reads them, projected through the
/// engine's own level rule ([`level_of`]): `Some(High)` on `RESN` is a
/// released reset, `Some(Low)` a held one, `None` nothing reaching the
/// pin; `VDD` likewise, `Some(High)` being a core rail a source holds up.
///
/// No voltage window is applied here — the P2 datasheet's `VDD` operating
/// range is the START gate's to cite (`NODES.md` §8 phase 4), and until
/// then the package invents no threshold of its own.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct P2ResetState {
    /// `RESN` (active low).
    pub resn: Option<Level>,
    /// `VDD`, the core supply.
    pub vdd: Option<Level>,
}

impl P2ResetState {
    /// Whether the chip is out of reset: `RESN` released **and** `VDD` up.
    pub fn out_of_reset(&self) -> bool {
        self.resn == Some(Level::High) && self.vdd == Some(Level::High)
    }
}

/// The crystal a delivered train on `XI` is: its rate, or `None` for a
/// held train (no clock reaches the pin).
pub fn crystal_of(train: &PulseTrain) -> Option<u64> {
    (train.pulses.freq_hz > 0).then(|| u64::from(train.pulses.freq_hz))
}

type CrystalCallback = Box<dyn Fn(Option<u64>) + Send + Sync>;
type ResetCallback = Box<dyn Fn(P2ResetState) + Send + Sync>;

/// What a core subscribed to through [`P2Pads`], collected during its
/// attach and frozen when the package installs its own senses.
#[derive(Default)]
struct Subscribers {
    crystal: Vec<CrystalCallback>,
    reset: Vec<ResetCallback>,
}

/// The package's last-delivered facts, readable through
/// [`P2PackageHandle`].
#[derive(Default)]
struct PackageState {
    crystal_hz: Option<u64>,
    reset: P2ResetState,
}

/// A core's whole surface: its 64 pads, the package's two facts (the
/// crystal on `XI`, the reset inputs), and the engine's wake scheduling.
/// Handed to [`P2Core::attach`] once, by the package; a core keeps the
/// handles it needs and subscribes to what it wants delivered. (The native
/// core is the exception that takes the underlying net I/O whole — see
/// [`McuComponent`]'s `P2Core` impl.)
///
/// A pad sense is also the declaration that the core **reads** that pad:
/// every pad is a released bidirectional pin, an input until driven, and a
/// pad the core subscribes to whose net floats is reported as
/// [`embsim_board::Finding::FloatingSense`] — a pad nothing reads floats
/// without one. A core subscribes to what it samples: the QEMU core to all
/// 64 (a guest may `testp` any of them), the native core to the pads its
/// HAL tables bridge, a core held in reset to none.
///
/// Everything here goes through the one interface a node has
/// (`NODES.md` §11): pad handles publish drives, pad senses deliver the
/// resolved net, and the two package facts are the package's own senses
/// projected once and fanned out. A core never reaches a net it has no
/// pad on.
#[derive(Clone)]
pub struct P2Pads {
    io: ComponentNetIo,
    subscribers: Arc<Mutex<Subscribers>>,
}

impl std::fmt::Debug for P2Pads {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("P2Pads").finish_non_exhaustive()
    }
}

impl P2Pads {
    /// The handle a core drives pad `pin` (`0..64`) through.
    pub fn pad(&self, pin: u8) -> Result<PinHandle, AttachError> {
        self.io.pin(pin_name(pin))
    }

    /// Subscribe to the resolved state of pad `pin`'s net: delivered once
    /// at registration and on every change, on the engine thread.
    pub fn on_pad_sense(
        &self,
        pin: u8,
        callback: impl Fn(NetState) + Send + 'static,
    ) -> Result<(), AttachError> {
        self.io.on_sense(pin_name(pin), callback)
    }

    /// Subscribe to the crystal: the rate delivered on `XI`, `Some(hz)`
    /// when a train with a rate reaches the pin and `None` when it is held.
    /// Delivered on the engine thread whenever the train changes.
    pub fn on_crystal(&self, callback: impl Fn(Option<u64>) + Send + Sync + 'static) {
        self.subscribers
            .lock()
            .expect("subscriber list never poisoned")
            .crystal
            .push(Box::new(callback));
    }

    /// Subscribe to the reset inputs (`RESN` and `VDD`, projected):
    /// delivered on the engine thread whenever either changes.
    pub fn on_reset(&self, callback: impl Fn(P2ResetState) + Send + Sync + 'static) {
        self.subscribers
            .lock()
            .expect("subscriber list never poisoned")
            .reset
            .push(Box::new(callback));
    }

    /// The core's wake handler (one per package; see
    /// [`ComponentNetIo::on_wake_ns`]).
    pub fn on_wake_ns(&self, callback: impl Fn(u64) + Send + 'static) {
        self.io.on_wake_ns(callback);
    }

    /// Arm a one-shot wake at an absolute virtual nanosecond.
    pub fn schedule_at_ns(&self, at_ns: u64) {
        self.io.schedule_at_ns(at_ns);
    }
}

/// What runs inside a [`P2Package`].
///
/// The trait is deliberately small: a core is attached once to its
/// [`P2Pads`] — pad handles, pad senses, the crystal and reset
/// subscriptions, wake scheduling — and started once, after every
/// component in the system has attached. The package owns the pin
/// declarations and the package-level senses; the core owns execution.
pub trait P2Core: Send + Sync {
    /// Take the pads. Runs at build, before the package is shared.
    fn attach(&mut self, pads: P2Pads) -> Result<(), AttachError>;

    /// Begin execution the core owns ([`Component::start`]): after every
    /// component has attached, on the live path only.
    fn start(&mut self) {}
}

/// A package with no core: the state any P2 is in before it runs. Every
/// pad released, the rails and `RESN` sensed, `XI` accepting the rate.
#[derive(Debug, Default, Clone, Copy)]
pub struct HeldInReset;

impl P2Core for HeldInReset {
    fn attach(&mut self, _pads: P2Pads) -> Result<(), AttachError> {
        Ok(())
    }
}

/// The native firmware image is a core too: [`McuComponent`] already
/// speaks the node interface, so the package hands it the pads' net I/O
/// and it bridges the channels its HAL tables name, exactly as it would
/// on a board of its own. The package still declares every pin and still
/// runs its own `XI`/`RESN`/`VDD` senses beside it.
///
/// What it is handed is the package's **whole** net I/O — the handle table
/// carries `XI`, `XO`, `RESN`, `VDD` and the `VIO` pins beside the 64 pads
/// — and the narrowing to the pads is by what the core names, not by a
/// filter: its HAL tables name pads (`P0`, `P2`, …) and nothing else. The
/// interface is the same one every node has, so nothing is reachable that
/// a node could not reach; the package-level facts still arrive through
/// the package's own senses, as for any core.
impl P2Core for McuComponent {
    fn attach(&mut self, pads: P2Pads) -> Result<(), AttachError> {
        Component::attach(self, pads.io)
    }

    fn start(&mut self) {
        Component::start(self);
    }
}

/// A view of a package's delivered facts that outlives handing it to a
/// `System`.
#[derive(Clone)]
pub struct P2PackageHandle {
    state: Arc<Mutex<PackageState>>,
}

impl std::fmt::Debug for P2PackageHandle {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("P2PackageHandle")
            .field("crystal_hz", &self.crystal_hz())
            .field("reset", &self.reset())
            .finish()
    }
}

impl P2PackageHandle {
    /// The crystal, as last delivered on `XI`: `Some(hz)` while a rate
    /// reaches the pin.
    pub fn crystal_hz(&self) -> Option<u64> {
        self.state
            .lock()
            .expect("package state never poisoned")
            .crystal_hz
    }

    /// The reset inputs, as last read.
    pub fn reset(&self) -> P2ResetState {
        self.state
            .lock()
            .expect("package state never poisoned")
            .reset
    }
}

/// The `P2X8C4M64P` package around a core.
pub struct P2Package<C> {
    pins: Vec<PinDecl>,
    core: C,
    state: Arc<Mutex<PackageState>>,
}

impl<C: std::fmt::Debug> std::fmt::Debug for P2Package<C> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("P2Package")
            .field("pins", &self.pins.len())
            .field("core", &self.core)
            .finish()
    }
}

impl P2Package<HeldInReset> {
    /// A package with no core: every pad released, rails and `RESN` sensed,
    /// `XI` accepting the rate — the state any P2 is in before it runs.
    pub fn held_in_reset() -> Self {
        Self::new(HeldInReset)
    }
}

impl P2Package<McuComponent> {
    /// The package around the native firmware image.
    pub fn native(mcu: McuComponent) -> Self {
        Self::new(mcu)
    }
}

impl<C: P2Core> P2Package<C> {
    /// The package around `core`.
    pub fn new(core: C) -> Self {
        Self {
            pins: p2x8c4m64p_pins(),
            core,
            state: Arc::new(Mutex::new(PackageState::default())),
        }
    }

    /// The core inside.
    pub fn core(&self) -> &C {
        &self.core
    }

    /// A view of the package's delivered facts that outlives handing it to
    /// a `System`.
    pub fn handle(&self) -> P2PackageHandle {
        P2PackageHandle {
            state: Arc::clone(&self.state),
        }
    }
}

impl<C: P2Core> Component for P2Package<C> {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let subscribers = Arc::new(Mutex::new(Subscribers::default()));
        self.core.attach(P2Pads {
            io: io.clone(),
            subscribers: Arc::clone(&subscribers),
        })?;
        // The core has subscribed to what it wants; freeze the lists so
        // delivery never holds a lock across a callback.
        let Subscribers { crystal, reset } =
            std::mem::take(&mut *subscribers.lock().expect("subscriber list never poisoned"));
        let crystal: Arc<[CrystalCallback]> = crystal.into();
        let reset: Arc<[ResetCallback]> = reset.into();

        // XI: the rate delivered here is the crystal. One projection, then
        // every subscriber.
        {
            let state = Arc::clone(&self.state);
            io.on_pulse("XI", move |train| {
                let hz = crystal_of(&train);
                state
                    .lock()
                    .expect("package state never poisoned")
                    .crystal_hz = hz;
                for callback in crystal.iter() {
                    callback(hz);
                }
            })?;
        }

        // RESN and VDD: each sense updates its half and delivers the pair.
        for (pin, is_resn) in [("RESN", true), ("VDD", false)] {
            let state = Arc::clone(&self.state);
            let reset = Arc::clone(&reset);
            io.on_sense(pin, move |sensed| {
                let level = level_of(sensed);
                let snapshot = {
                    let mut state = state.lock().expect("package state never poisoned");
                    if is_resn {
                        state.reset.resn = level;
                    } else {
                        state.reset.vdd = level;
                    }
                    state.reset
                };
                for callback in reset.iter() {
                    callback(snapshot);
                }
            })?;
        }
        Ok(())
    }

    fn start(&mut self) {
        self.core.start();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embsim_board::{PulseDirection, PulseSegment};
    use rstest::rstest;

    #[test]
    fn the_facade_is_the_ec32mb_u100_slot() {
        let pins = p2x8c4m64p_pins();
        assert_eq!(pins.len(), 86);
        assert_eq!(NUM_PINS, 86);
        assert_eq!(pins[0].number, "P0");
        assert_eq!(pins[63].number, "P63");
        for pad in &pins[..NUM_PADS] {
            assert_eq!(pad.kind, PinKind::DigitalBidir, "{}", pad.number);
            assert_eq!(pad.idle, IdleDrive::Released, "{}", pad.number);
            assert_eq!(pad.stream, None, "{}", pad.number);
        }
        let pin = |name: &str| pins.iter().find(|p| p.number == name).copied().unwrap();
        assert_eq!(pin("XI").kind, PinKind::DigitalIn);
        assert_eq!(pin("XI").stream, Some(StreamRole::PulseSink));
        assert_eq!(pin("XO").kind, PinKind::DigitalOut);
        assert_eq!(pin("XO").idle, IdleDrive::Released);
        assert_eq!(pin("RESN").kind, PinKind::DigitalIn);
        assert_eq!(pin("TEST").kind, PinKind::DigitalIn);
        for rail in ["VDD", "GND", "VIO_0_3", "VIO_56_59", "VIO_60_63"] {
            assert_eq!(pin(rail).kind, PinKind::PowerIn, "{rail}");
        }
        assert_eq!(pin_name(0), "P0");
        assert_eq!(pin_name(63), "P63");
        assert_eq!(pin_name(64), "P0", "taken modulo 64");
    }

    /// Bits 13:11 select the drive while `OUT` = 1 and bits 10:8 while
    /// `OUT` = 0; the three resistive modes are their resistances, fast is
    /// the placeholder, float is released, and a current source is the
    /// unmapped variant.
    #[rstest]
    #[case::fast_high(P_HIGH_FAST | P_LOW_FAST, true, PadDrive::Thevenin(TheveninDrive { volts: 3.3, impedance: P2_FAST_OHMS }))]
    #[case::fast_low(P_HIGH_FAST | P_LOW_FAST, false, PadDrive::Thevenin(TheveninDrive { volts: 0.0, impedance: P2_FAST_OHMS }))]
    #[case::pull_up_15k(P_HIGH_15K, true, PadDrive::Thevenin(TheveninDrive { volts: 3.3, impedance: 15_000.0 }))]
    #[case::pull_up_1k5(P_HIGH_1K5, true, PadDrive::Thevenin(TheveninDrive { volts: 3.3, impedance: 1_500.0 }))]
    #[case::pull_up_150k(P_HIGH_150K, true, PadDrive::Thevenin(TheveninDrive { volts: 3.3, impedance: 150_000.0 }))]
    #[case::pull_down_15k(P_LOW_15K, false, PadDrive::Thevenin(TheveninDrive { volts: 0.0, impedance: 15_000.0 }))]
    #[case::open_drain_high(P_HIGH_FLOAT | P_LOW_FAST, true, PadDrive::Released)]
    #[case::open_drain_low(P_HIGH_FLOAT | P_LOW_FAST, false, PadDrive::Thevenin(TheveninDrive { volts: 0.0, impedance: P2_FAST_OHMS }))]
    #[case::high_mode_does_not_apply_low(P_HIGH_15K | P_LOW_FAST, false, PadDrive::Thevenin(TheveninDrive { volts: 0.0, impedance: P2_FAST_OHMS }))]
    #[case::current_1ma(P_HIGH_1MA, true, PadDrive::CurrentSource(PadMode::Current1mA))]
    #[case::current_100ua(P_HIGH_100UA, true, PadDrive::CurrentSource(PadMode::Current100uA))]
    #[case::current_10ua_low(P_LOW_10UA, false, PadDrive::CurrentSource(PadMode::Current10uA))]
    #[case::adc_mode_word_is_fast(0x0010_0000 | P_HIGH_15K, true, PadDrive::Thevenin(TheveninDrive { volts: 3.3, impedance: P2_FAST_OHMS }))]
    fn a_wrpin_word_is_the_pads_thevenin(
        #[case] cfg: u32,
        #[case] out: bool,
        #[case] expected: PadDrive,
    ) {
        assert_eq!(pad_drive(cfg, true, out, 3.3), expected);
    }

    #[rstest]
    #[case::fast(P_HIGH_FAST)]
    #[case::pull(P_HIGH_15K)]
    #[case::current(P_HIGH_1MA)]
    fn a_pad_with_dir_clear_is_released_whatever_its_word(#[case] cfg: u32) {
        assert_eq!(pad_drive(cfg, false, true, 3.3), PadDrive::Released);
        assert_eq!(pad_drive(cfg, false, false, 3.3), PadDrive::Released);
    }

    #[test]
    fn the_spin2_constants_land_in_their_fields() {
        assert_eq!(P_HIGH_15K, 0x1000);
        assert_eq!(P_HIGH_FLOAT, 0x3800);
        assert_eq!(P_LOW_15K, 0x200);
        assert_eq!(P_LOW_FLOAT, 0x700);
        assert_eq!(
            pad_modes(P_HIGH_15K | P_LOW_FLOAT),
            (PadMode::Ohms15k, PadMode::Float)
        );
    }

    #[test]
    fn a_train_with_a_rate_is_the_crystal_and_a_held_one_is_none() {
        let train = PulseTrain {
            pulses: PulseSegment {
                emitted: 0,
                freq_hz: 20_000_000,
                total: None,
                since_us: 1_000,
            },
            direction: PulseDirection::Forward,
        };
        assert_eq!(crystal_of(&train), Some(20_000_000));
        assert_eq!(crystal_of(&PulseTrain::IDLE), None);
    }

    #[test]
    fn out_of_reset_needs_resn_released_and_vdd_up() {
        let both = P2ResetState {
            resn: Some(Level::High),
            vdd: Some(Level::High),
        };
        assert!(both.out_of_reset());
        assert!(!P2ResetState {
            resn: Some(Level::Low),
            ..both
        }
        .out_of_reset());
        assert!(!P2ResetState { vdd: None, ..both }.out_of_reset());
        assert!(!P2ResetState::default().out_of_reset());
    }
}
