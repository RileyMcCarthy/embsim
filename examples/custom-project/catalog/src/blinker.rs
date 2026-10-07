//! `example-blinker`: a P2 core that toggles one pad on a schedule.
//!
//! What a program of three instructions would do — `DRVH` the pad, wait
//! half a period, `DRVL` it, wait, and again — said as a core: at the
//! package's START instant it drives the pad high, and every half period
//! after that it flips it, each flip a wake it armed for itself through the
//! package (`P2Pads::schedule_at_ns`). The pad drives through the package's
//! own decode ([`BankSupplies::pad_drive`]): its bank's supply behind the
//! fast mode's impedance, the mode a pad is in until a program sets
//! another, or nothing while the bank has no supply.
//!
//! The core is held to the same gate as every core: it is not started
//! before the package's reset and restart delay let it, and once the
//! package holds it after a brownout it is woken no more.
//!
//! Options (`[board.model.options]` beside `core = "example-blinker"`):
//!
//! - `pin`, required: the pad, 0 to 63;
//! - `period`, required: the time between two rising edges, written as
//!   `run --for` takes a time (`"1ms"`), an even number of nanoseconds
//!   so each half is a whole one.

use std::sync::{Arc, Mutex};

use embsim_board::report::instant;
use embsim_board::{
    Assignment, AttachError, Drive, KindInfo, PartOptions, PinHandle, ProjectError, Report,
};
use embsim_boards::p2::{
    BankSupplies, CoreCatalog, CoreCtor, P2Core, P2Pads, PadDrive, NATIVE_PAD_MODE, NUM_PADS,
};
use embsim_core::virtual_clock;

/// The core kind.
pub const BLINKER: &str = "example-blinker";

/// The core catalog: one kind.
#[derive(Debug, Clone, Copy, Default)]
pub struct BlinkerCores;

impl CoreCatalog for BlinkerCores {
    fn name(&self) -> &str {
        crate::NAME
    }

    fn core_kinds(&self) -> Vec<KindInfo> {
        vec![KindInfo::new(
            BLINKER,
            "a core that toggles one pad every half period from its start",
        )
        .requires("pin", "0", "the pad the core toggles, 0 to 63")
        .requires("period", "\"1ms\"", "the time between two rising edges")]
    }

    fn seat(
        &self,
        _core: &str,
        assignment: &Assignment<'_>,
        mut options: PartOptions,
    ) -> Result<CoreCtor, ProjectError> {
        let pin = options
            .integer("pin")?
            .ok_or_else(|| options.error("options.pin is the pad the core toggles, 0 to 63"))?;
        let pin = u8::try_from(pin)
            .ok()
            .filter(|pin| usize::from(*pin) < NUM_PADS)
            .ok_or_else(|| options.error(format!("options.pin = {pin} is not a pad, 0 to 63")))?;
        let period = options.duration("period")?.ok_or_else(|| {
            options.error("options.period is the time between two rising edges (\"1ms\")")
        })?;
        if period == 0 || period % 2 != 0 {
            return Err(options.error(format!(
                "options.period is {period} ns; it is more than 0 and an even number of \
                 nanoseconds, so each half is a whole one"
            )));
        }
        options.finish()?;
        let reports = assignment.reports.clone();
        let board = assignment.board.to_string();
        Ok(Box::new(move |decl| {
            let state = Arc::new(Mutex::new(Blink::default()));
            reports.add(BlinkReport {
                subject: format!("{board}.{}", decl.reference),
                pin,
                half_ns: period / 2,
                state: Arc::clone(&state),
                said_start: false,
            });
            Ok(Box::new(Blinker {
                pin,
                half_ns: period / 2,
                state,
            }) as Box<dyn P2Core>)
        }))
    }
}

/// What the core has done, shared with its report.
#[derive(Default)]
struct Blink {
    /// The pad's handle, the bank supplies and the package's pads (for its
    /// wakes), from attach.
    attached: Option<(PinHandle, BankSupplies, P2Pads)>,
    /// The level the pad drives now.
    high: bool,
    /// The START instant.
    started_at_ns: Option<u64>,
    /// How many times the pad changed level after the first drive.
    toggles: u64,
    /// The instant of the next flip: the one wake this core armed. The
    /// package delivers every wake on its net I/O to the core once it is
    /// started, so a wake at any other instant is not this core's.
    next_at_ns: Option<u64>,
}

impl Blink {
    /// Drive the pad at `high` and arm the next flip `half_ns` after `now`.
    fn drive(&mut self, pin: u8, high: bool, now_ns: u64, half_ns: u64) {
        let Some((pad, banks, pads)) = &self.attached else {
            return;
        };
        match banks.pad_drive(pin, NATIVE_PAD_MODE, true, high) {
            PadDrive::Thevenin(drive) => pad.drive(Drive::Thevenin(drive)),
            PadDrive::Released | PadDrive::CurrentSource(_) => pad.release(),
        }
        let next = now_ns + half_ns;
        pads.schedule_at_ns(next);
        self.next_at_ns = Some(next);
        self.high = high;
    }
}

/// The core.
struct Blinker {
    pin: u8,
    half_ns: u64,
    state: Arc<Mutex<Blink>>,
}

impl P2Core for Blinker {
    fn attach(&mut self, pads: P2Pads) -> Result<(), AttachError> {
        let pad = pads.pad(self.pin)?;
        let (pin, half_ns) = (self.pin, self.half_ns);
        let state = Arc::clone(&self.state);
        // Delivered only once the core is started, at each instant it armed.
        pads.on_wake_ns(move |now_ns| {
            let mut blink = state.lock().expect("never poisoned");
            if blink.next_at_ns != Some(now_ns) {
                return;
            }
            let high = !blink.high;
            blink.drive(pin, high, now_ns, half_ns);
            blink.toggles += 1;
        });
        self.state.lock().expect("never poisoned").attached =
            Some((pad, pads.bank_supplies(), pads));
        Ok(())
    }

    fn start(&mut self) {
        // The START instant: the current virtual nanosecond.
        let now_ns = virtual_clock::virtual_ns();
        let mut blink = self.state.lock().expect("never poisoned");
        blink.started_at_ns = Some(now_ns);
        blink.drive(self.pin, true, now_ns, self.half_ns);
    }
}

/// What the core says: when it started, and at the end how often it
/// flipped its pad.
struct BlinkReport {
    subject: String,
    pin: u8,
    half_ns: u64,
    state: Arc<Mutex<Blink>>,
    said_start: bool,
}

impl Report for BlinkReport {
    fn subject(&self) -> String {
        self.subject.clone()
    }

    fn look(&mut self, _now_ns: u64) -> Vec<String> {
        let started = self.state.lock().expect("never poisoned").started_at_ns;
        match started {
            Some(at) if !self.said_start => {
                self.said_start = true;
                vec![format!(
                    "blinker: P{} high at {}, flipping every {}",
                    self.pin,
                    instant(at),
                    instant(self.half_ns)
                )]
            }
            _ => Vec::new(),
        }
    }

    fn summary(&self) -> Vec<String> {
        let blink = self.state.lock().expect("never poisoned");
        vec![match blink.started_at_ns {
            Some(at) => format!(
                "blinker: started at {}; P{} flipped {} times, and drives it {}",
                instant(at),
                self.pin,
                blink.toggles,
                if blink.high { "high" } else { "low" }
            ),
            None => format!("blinker: never started; P{} was never driven", self.pin),
        }]
    }
}
