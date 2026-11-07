//! Model: the **reset stage of a Parallax Prop Plug** (#32201), the USB
//! adapter a Propeller's programming header takes: its serial chip's DTR
//! output coupled through a series capacitor into an NPN transistor's
//! base, the emitter on the plug's `VSS`, the collector the plug's `RES`
//! pin. A rising edge on DTR pushes charge into the base and switches the
//! transistor on for a short pulse, and `RES` sinks the target's reset
//! line low; between pulses the transistor is off and `RES` is released.
//! The plug has no pull-up: the target's own holds its reset high.
//!
//! # Provenance
//!
//! - **Topology** — Parallax, *Prop Plug (#32200/#32201) product guide*
//!   v1.3, p. 5 (the Rev A schematic, FT232RQ): DTR (pin 31) → 10 nF in
//!   series → the NPN's base, 33 kΩ from the base to VSS, emitter to VSS,
//!   collector straight to the header's RES pin; p. 3, the header: VSS,
//!   RES, TX, RX. The Rev E schematic (units shipped from 2021-01-28) was
//!   not reachable when this was written; its stage is taken as Rev A's.
//! - **Pulse width** — Parallax's Prop Plug product page (Rev E), archived
//!   2025-01-17: "Reset pulse width: ~20 µs (typical)".
//!   [`PROP_PLUG_RESET_PULSE_NS`]. Rev A's capacitor and base resistor
//!   bound it at 10 nF × 33 kΩ = 330 µs; the base-emitter junction clamps
//!   the base near 0.65 V, so the pulse the collector sinks is much
//!   shorter, as the page's typical says.
//! - **The sink** — the schematic names no transistor, so the saturated
//!   collector is the crate's strong default
//!   ([`embsim_board::net::DEFAULT_PUSH_PULL_IMPEDANCE`]), as a bench
//!   driver no datasheet describes is ([`embsim_board::HostRailLine`]'s
//!   `TX`). Against a reset pull-up of kilohms any saturated transistor
//!   projects the same low.
//!
//! # Which DTR state resets
//!
//! The stage fires on a **rising** edge of its input; a falling edge
//! reverse-biases the base and does nothing. Which DTR state that is
//! belongs to the adapter's chip, not to this stage: an FTDI's DTR output
//! is active low from the factory (`DTR#`), so a rising edge is DTR
//! deasserted, and its EEPROM can invert it, so a rising edge is DTR
//! asserted. The host kinds carry that as `invert_dtr`.
//!
//! # Deliberate simplifications
//!
//! - **A closed form, not the RC transient** (`DESIGN.md` §5: no
//!   exponential device law, no capacitor as a cluster edge): each rising
//!   edge sinks `RES` for [`PROP_PLUG_RESET_PULSE_NS`] from its instant and
//!   releases it; a rising edge inside a pulse starts the pulse again.
//! - **The input reads through the JESD8C.01 LVCMOS pair** of the 3.3 V
//!   rail the adapter's chip drives it at; the transistor's base threshold
//!   is not followed.
//! - **The level the input first reads is no edge**: the adapter is taken
//!   as powered and settled when the system starts. Plugging a Prop Plug in
//!   — its chip powering up with DTR deasserted — is an edge this does not
//!   model.
//! - **`VSS` names the voltage the collector sinks to**; a `VSS` that names
//!   none (nothing holds the plug's ground) sinks nothing (`DESIGN.md`
//!   rule 6).

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

use embsim_board::net::DEFAULT_PUSH_PULL_IMPEDANCE;
use embsim_board::{
    jesd8c01_lvcmos_thresholds, AttachError, Component, ComponentNetIo, DeadBand, DigitalReceiver,
    Level, PinDecl, PinHandle, TheveninDrive, Volts,
};

/// How long `RES` sinks after a rising edge on DTR: 20 µs, the Rev E
/// product page's typical ("Reset pulse width: ~20 µs (typical)").
pub const PROP_PLUG_RESET_PULSE_NS: u64 = 20_000;

/// The stage's pins: `DTR`, the input from the adapter's chip; `RES`, the
/// open collector on the plug's header; `VSS`, the plug's ground, which
/// both are read and driven against.
pub const PROP_PLUG_RESET_PINS: [PinDecl; 3] = [
    PinDecl::digital_in("DTR", jesd8c01_lvcmos_thresholds(DeadBand::Unknown)).with_reference("VSS"),
    PinDecl::digital_out("RES")
        .sink_only()
        .with_idle(None)
        .with_reference("VSS"),
    PinDecl::power_in("VSS"),
];

#[derive(Debug, Default)]
struct State {
    /// The level the input last read, once it has read one.
    dtr: Option<Level>,
    /// The voltage `VSS` names in the engine's frame.
    vss: Option<Volts>,
    /// The instant the pulse under way ends, while one is.
    until_ns: Option<u64>,
    res: Option<PinHandle>,
}

impl State {
    /// `RES` as the stage drives it now: sinking to `VSS` while a pulse is
    /// under way, released otherwise.
    fn publish(&self) {
        let Some(res) = self.res.as_ref() else {
            return;
        };
        let drive = match (self.until_ns, self.vss) {
            (Some(_), Some(volts)) => Some(TheveninDrive {
                volts,
                impedance: DEFAULT_PUSH_PULL_IMPEDANCE,
            }),
            _ => None,
        };
        res.set_drive(drive);
    }
}

/// What a [`PropPlugReset`] has done, readable after it is moved into a
/// system.
#[derive(Debug, Clone, Default)]
pub struct PropPlugResetHandle {
    pulses: Arc<AtomicU64>,
    last_pulse_ns: Arc<Mutex<Option<u64>>>,
}

impl PropPlugResetHandle {
    /// Pulses started on `RES`: one a rising edge on DTR.
    pub fn pulses(&self) -> u64 {
        self.pulses.load(Ordering::Relaxed)
    }

    /// The instant the last pulse started.
    pub fn last_pulse_ns(&self) -> Option<u64> {
        *self
            .last_pulse_ns
            .lock()
            .expect("the pulse record is never poisoned")
    }
}

/// The Prop Plug's reset stage as a board component (module docs).
#[derive(Debug, Default)]
pub struct PropPlugReset {
    handle: PropPlugResetHandle,
}

impl PropPlugReset {
    /// A stage at rest: `RES` released.
    pub fn new() -> Self {
        Self::default()
    }

    /// A view that outlives handing the stage to a system.
    pub fn handle(&self) -> PropPlugResetHandle {
        self.handle.clone()
    }
}

impl Component for PropPlugReset {
    fn pins(&self) -> &[PinDecl] {
        &PROP_PLUG_RESET_PINS
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let state = Arc::new(Mutex::new(State {
            res: Some(io.pin("RES")?),
            ..State::default()
        }));
        {
            let state = Arc::clone(&state);
            io.on_sense("VSS", move |sense| {
                let mut state = state.lock().expect("the stage is never poisoned");
                state.vss = sense.volts;
                state.publish();
            })?;
        }
        {
            let state = Arc::clone(&state);
            let handle = self.handle.clone();
            let arm = io.clone();
            let receiver = DigitalReceiver::new(io.pin("DTR")?);
            io.on_sense("DTR", move |sense| {
                let Some(level) = receiver.read(&sense) else {
                    // A floating or undecided input moves no charge: the
                    // last level holds.
                    return;
                };
                let mut state = state.lock().expect("the stage is never poisoned");
                let rising = state.dtr == Some(Level::Low) && level == Level::High;
                state.dtr = Some(level);
                if !rising {
                    return;
                }
                let until = sense.at_ns.saturating_add(PROP_PLUG_RESET_PULSE_NS);
                state.until_ns = Some(until);
                state.publish();
                handle.pulses.fetch_add(1, Ordering::Relaxed);
                *handle
                    .last_pulse_ns
                    .lock()
                    .expect("the pulse record is never poisoned") = Some(sense.at_ns);
                arm.schedule_at_ns(until);
            })?;
        }
        io.on_wake_ns(move |now| {
            let mut state = state.lock().expect("the stage is never poisoned");
            // A wake a later edge has outlived is not this pulse's end.
            if state.until_ns == Some(now) {
                state.until_ns = None;
                state.publish();
            }
        });
        Ok(())
    }
}
