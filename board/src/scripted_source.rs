//! A pin driven through a list of steps: the smallest stimulus that lets a
//! system do something over time — a button pressed, a rail browned out, a
//! sensor's output swept.
//!
//! A [`ScriptedSource`] has one pin, `OUT`, a linear source
//! ([`PinDecl::analog_source`]). Each [`Step`] is an instant, counted from
//! the instant the system starts, and the volts the pin drives from then
//! on, behind the source's impedance. Before its first instant the pin is
//! released; after its last it holds.
//!
//! Each step is one [`Drive::Thevenin`] published at its own instant, on a
//! wake the source armed for it (`DESIGN.md` rules 3 and 5: published at
//! its own instant, time entering only as scheduled instants). The source
//! keeps no thread: on the stepped clock it is deterministic for free.
//!
//! Every number is the scenario's (`DESIGN.md` rule 6): the impedance and
//! every step's instant and volts are given, and the source invents none —
//! it has no default impedance. Volts are in the engine's frame, as a
//! project wire's `volts` are.

use std::sync::{Arc, Mutex};

use embsim_core::virtual_clock;

use crate::component::{AttachError, Component, ComponentNetIo, Drive, PinDecl};
use crate::net::{Ohms, TheveninDrive, Volts};

/// One step of a [`ScriptedSource`]: from `at_ns` after the system starts,
/// the pin drives `volts`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Step {
    /// The instant the step lands, nanoseconds after the system starts.
    pub at_ns: u64,
    /// What the pin drives from that instant, behind the source's
    /// impedance.
    pub volts: Volts,
}

/// Where the source is in its script.
#[derive(Debug, Default)]
struct Cursor {
    /// The virtual instant the system started at: steps count from it.
    origin_ns: Option<u64>,
    /// The next step not yet driven.
    next: usize,
}

/// One pin, `OUT`, driven through a list of [`Step`]s (module docs).
pub struct ScriptedSource {
    pins: [PinDecl; 1],
    ohms: Ohms,
    steps: Arc<[Step]>,
    cursor: Arc<Mutex<Cursor>>,
    io: Option<ComponentNetIo>,
}

impl std::fmt::Debug for ScriptedSource {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ScriptedSource")
            .field("ohms", &self.ohms)
            .field("steps", &self.steps)
            .finish()
    }
}

impl ScriptedSource {
    /// A source of impedance `ohms` driven through `steps`. Refused, with
    /// the reason, unless `ohms` is a positive finite number, every step's
    /// volts are finite, there is at least one step, and the instants
    /// increase strictly.
    pub fn new(ohms: Ohms, steps: Vec<Step>) -> Result<Self, String> {
        if !(ohms.is_finite() && ohms > 0.0) {
            return Err(format!(
                "ohms = {ohms} is not a source impedance; give one more than 0 Ω (an ideal \
                 constant supply is a wire with volts)"
            ));
        }
        if steps.is_empty() {
            return Err("steps is empty; give at least one [\"instant\", volts]".to_string());
        }
        for (index, step) in steps.iter().enumerate() {
            if !step.volts.is_finite() {
                return Err(format!("step {index}: {} is not a voltage", step.volts));
            }
            if index > 0 && step.at_ns <= steps[index - 1].at_ns {
                return Err(format!(
                    "step {index} lands at {} ns, not after step {} at {} ns; the instants \
                     increase strictly",
                    step.at_ns,
                    index - 1,
                    steps[index - 1].at_ns
                ));
            }
        }
        Ok(Self {
            pins: [PinDecl::analog_source("OUT")],
            ohms,
            steps: steps.into(),
            cursor: Arc::new(Mutex::new(Cursor::default())),
            io: None,
        })
    }

    /// The steps, in order.
    pub fn steps(&self) -> &[Step] {
        &self.steps
    }

    /// The source's impedance.
    pub fn ohms(&self) -> Ohms {
        self.ohms
    }
}

impl Component for ScriptedSource {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let pin = io.pin("OUT")?;
        let (steps, cursor, ohms) = (Arc::clone(&self.steps), Arc::clone(&self.cursor), self.ohms);
        let wakes = io.clone();
        io.on_wake_ns(move |now_ns| {
            let mut cursor = cursor
                .lock()
                .expect("the script's cursor is never poisoned");
            let Some(origin) = cursor.origin_ns else {
                return;
            };
            let elapsed = now_ns.saturating_sub(origin);
            // Every step due by now; the last of them is what the pin drives.
            let mut due = None;
            while let Some(step) = steps.get(cursor.next).filter(|step| step.at_ns <= elapsed) {
                due = Some(*step);
                cursor.next += 1;
            }
            if let Some(step) = due {
                pin.drive(Drive::Thevenin(TheveninDrive {
                    volts: step.volts,
                    impedance: ohms,
                }));
            }
            if let Some(next) = steps.get(cursor.next) {
                wakes.schedule_at_ns(origin.saturating_add(next.at_ns));
            }
        });
        self.io = Some(io);
        Ok(())
    }

    /// The system started: the steps count from now. Time is held until
    /// every component has started, so the instant is the same on every run.
    fn start(&mut self) {
        let origin = virtual_clock::virtual_ns();
        self.cursor
            .lock()
            .expect("the script's cursor is never poisoned")
            .origin_ns = Some(origin);
        if let (Some(io), Some(first)) = (&self.io, self.steps.first()) {
            io.schedule_at_ns(origin.saturating_add(first.at_ns));
        }
    }
}
