//! Model: the **EX-BUF1**, a single Schmitt-trigger buffer. **An example
//! part**: no EX-BUF1 exists, and its datasheet is a stand-in written for
//! this example.
//!
//! The input `A` is read through the part's thresholds against its `GND`
//! pin; the output `Y` follows it after the part's propagation delay, as a
//! wake the model arms at the instant the input changed plus `t_pd`, and
//! drives the part's own supply above its ground behind the datasheet's
//! output impedance (`DESIGN.md` rule 3: the model publishes at its own
//! instant, never in the pass that handed it the input).
//!
//! # Datasheet provenance
//!
//! *EX-BUF1 single Schmitt-trigger buffer, stand-in datasheet, revision A,
//! 2026-10-01*: `datasheets/EX-BUF1.md` in this crate.
//!
//! - §1 "Pin functions": 1 `A`, 2 `GND`, 3 `Y`, 4 `VCC`; `Y = A`.
//!   [`BUFFER_PINS`].
//! - §2 "Recommended operating conditions": `VCC` 3.0 V to 3.6 V
//!   ([`VCC_MIN_VOLTS`], [`VCC_MAX_VOLTS`]); outside it the behaviour is not
//!   specified, so the model releases `Y`.
//! - §3 "Electrical characteristics": `V_T+` max 2.0 V and `V_T−` min
//!   0.9 V, the input holding its last level between them ([`THRESHOLDS`]);
//!   `V_OH` min `VCC − 0.4 V` at −8 mA and `V_OL` max 0.32 V at 8 mA, the
//!   output impedances `0.4 V / 8 mA` and `0.32 V / 8 mA` ([`R_OH_OHMS`],
//!   [`R_OL_OHMS`]).
//! - §4 "Switching characteristics": `t_pd` max 12 ns ([`T_PD_NS`]), the
//!   bound, as embsim's gates take theirs.
//! - §5 "Application notes": the output of an open input is not specified,
//!   so `Y` is released while `A` reads no level.
//!
//! # Deliberate simplifications
//!
//! - **One impedance per level**, the worst case at 8 mA; the output's
//!   current-voltage curve is not modelled.
//! - **Input capacitance and rise-time limits** are not modelled.

use std::collections::VecDeque;
use std::sync::{Arc, Mutex};

use embsim_board::{
    AttachError, Component, ComponentNetIo, DeadBand, DigitalReceiver, Drive, Level, Ohms, PinDecl,
    PinHandle, Sense, TheveninDrive, Thresholds, Volts,
};
use embsim_core::virtual_clock;

/// `V_T−` min, the input reads low at or below it (§3).
pub const V_T_MINUS_VOLTS: Volts = 0.9;
/// `V_T+` max, the input reads high at or above it (§3).
pub const V_T_PLUS_VOLTS: Volts = 2.0;
/// The input's thresholds, against `GND`: low at or below `V_T−`, high at
/// or above `V_T+`, and between them the level it last read (§3).
pub const THRESHOLDS: Thresholds =
    Thresholds::new(V_T_MINUS_VOLTS, V_T_PLUS_VOLTS, 0.0, DeadBand::HoldLast);
/// The high-level output impedance: `(VCC − V_OH min) / I_OH`, 0.4 V at
/// 8 mA (§3).
pub const R_OH_OHMS: Ohms = 0.4 / 0.008;
/// The low-level output impedance: `V_OL max / I_OL`, 0.32 V at 8 mA (§3).
pub const R_OL_OHMS: Ohms = 0.32 / 0.008;
/// `t_pd` max, `A` to `Y` (§4).
pub const T_PD_NS: u64 = 12;
/// The lowest recommended supply (§2).
pub const VCC_MIN_VOLTS: Volts = 3.0;
/// The highest recommended supply (§2).
pub const VCC_MAX_VOLTS: Volts = 3.6;

/// The pins (§1): `A` read against `GND`, `Y` released until the model
/// drives it, `VCC` measured against `GND`, and `GND` itself, read in the
/// engine's frame so `Y` drives above wherever it sits.
pub const BUFFER_PINS: [PinDecl; 4] = [
    PinDecl::digital_in("1", THRESHOLDS)
        .with_name("A")
        .with_reference("2"),
    PinDecl::power_in("2").with_name("GND"),
    PinDecl::digital_out("3")
        .with_name("Y")
        .with_idle(None)
        .with_reference("2"),
    PinDecl::power_in("4").with_name("VCC").with_reference("2"),
];

/// What the model knows, on the engine thread.
#[derive(Default)]
struct State {
    io: Option<ComponentNetIo>,
    y: Option<PinHandle>,
    /// `A`'s level, through [`THRESHOLDS`].
    level: Option<Level>,
    /// `VCC` against `GND`.
    vcc: Option<Volts>,
    /// `GND` in the engine's frame.
    gnd: Option<Volts>,
    /// The drive last asked for, so an unchanged input arms nothing.
    requested: Option<Option<TheveninDrive>>,
    /// Drives asked for and not yet due, in instant order.
    pending: VecDeque<(u64, Option<TheveninDrive>)>,
}

impl State {
    /// What `Y` presents for what the model knows now.
    fn drive(&self) -> Option<TheveninDrive> {
        let vcc = self
            .vcc
            .filter(|vcc| (VCC_MIN_VOLTS..=VCC_MAX_VOLTS).contains(vcc))?;
        let gnd = self.gnd?;
        match self.level? {
            Level::High => Some(TheveninDrive {
                volts: gnd + vcc,
                impedance: R_OH_OHMS,
            }),
            Level::Low => Some(TheveninDrive {
                volts: gnd,
                impedance: R_OL_OHMS,
            }),
        }
    }

    /// Arm `Y`'s next drive `t_pd` after now, when it changes.
    fn refresh(&mut self) {
        let drive = self.drive();
        if self.requested == Some(drive) {
            return;
        }
        self.requested = Some(drive);
        let due = virtual_clock::virtual_ns().saturating_add(T_PD_NS);
        self.pending.push_back((due, drive));
        if let Some(io) = &self.io {
            io.schedule_at_ns(due);
        }
    }

    /// A wake: present the latest drive that is due.
    fn wake(&mut self, now_ns: u64) {
        let mut latest = None;
        while self.pending.front().is_some_and(|(due, _)| *due <= now_ns) {
            latest = self.pending.pop_front().map(|(_, drive)| drive);
        }
        if let (Some(drive), Some(y)) = (latest, &self.y) {
            match drive {
                Some(drive) => y.drive(Drive::Thevenin(drive)),
                None => y.release(),
            }
        }
    }
}

/// One EX-BUF1.
pub struct Buffer {
    state: Arc<Mutex<State>>,
}

impl Buffer {
    /// A buffer that has read nothing yet: `Y` released.
    pub fn new() -> Self {
        Self {
            state: Arc::default(),
        }
    }
}

impl Default for Buffer {
    fn default() -> Self {
        Self::new()
    }
}

impl Component for Buffer {
    fn pins(&self) -> &[PinDecl] {
        &BUFFER_PINS
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        {
            let mut state = self.state.lock().expect("never poisoned");
            state.io = Some(io.clone());
            state.y = Some(io.pin("Y")?);
        }
        let state = Arc::clone(&self.state);
        io.on_wake_ns(move |now_ns| state.lock().expect("never poisoned").wake(now_ns));
        // The supply and ground first, so an input delivered before them
        // does not drive from a part with no supply.
        let state = Arc::clone(&self.state);
        io.on_sense("GND", move |sense: Sense| {
            let mut state = state.lock().expect("never poisoned");
            state.gnd = sense.volts;
            state.refresh();
        })?;
        let state = Arc::clone(&self.state);
        io.on_sense("VCC", move |sense: Sense| {
            let mut state = state.lock().expect("never poisoned");
            state.vcc = sense.volts;
            state.refresh();
        })?;
        let receiver = DigitalReceiver::new(io.pin("A")?);
        let state = Arc::clone(&self.state);
        io.on_sense("A", move |sense: Sense| {
            let mut state = state.lock().expect("never poisoned");
            state.level = receiver.read(&sense);
            state.refresh();
        })
    }
}
