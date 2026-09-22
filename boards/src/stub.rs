//! Pin facades with no behaviour, for the parts of a board that are real but
//! not exercised.
//!
//! A board is only buildable when EVERY component declares a facade the netlist
//! agrees with, in both directions — so the parts nothing drives still have to
//! be declared. A stub is the honest way to do that: it says what pins the part
//! has and what direction each one faces, and nothing else. Promoting one to a
//! model later is a registry line, not a restructuring.
//!
//! # Choosing pin kinds for a stub
//!
//! Declare what the part does on the BOARD, not what its datasheet permits. An
//! output nothing reads is declared [`dig_in`], because a stub that claimed to
//! drive a net would make that net contend with whatever really drives it, and
//! the engine would be right to complain. [`passive`] is for pins whose
//! direction is genuinely topology-only.

use embsim_board::{AttachError, Component, ComponentNetIo, PartRegistry, PinDecl, PinKind};

/// A pin the component senses and never drives.
pub const fn dig_in(number: &'static str) -> PinDecl {
    PinDecl {
        number,
        name: None,
        kind: PinKind::DigitalIn,
        stream: None,
        drive_impedance: None,
    }
}

/// A push-pull output pin (idles `Driven(High)` until the component drives).
pub const fn dig_out(number: &'static str) -> PinDecl {
    PinDecl {
        number,
        name: None,
        kind: PinKind::DigitalOut,
        stream: None,
        drive_impedance: None,
    }
}

/// A pin whose *voltage* the component needs (participates in the cluster
/// solve) — a differential receiver input, an ADC input.
pub const fn analog(number: &'static str) -> PinDecl {
    PinDecl {
        number,
        name: None,
        kind: PinKind::Analog,
        stream: None,
        drive_impedance: None,
    }
}

/// A rail the part consumes.
pub const fn pwr_in(number: &'static str) -> PinDecl {
    PinDecl {
        number,
        name: None,
        kind: PinKind::PowerIn,
        stream: None,
        drive_impedance: None,
    }
}

/// A rail the part generates (regulator/DC-DC output, isolated-domain
/// reference). Registers the net as sourced at an unmodeled voltage — enough
/// to clear [`embsim_board::Finding::PowerNetUnsourced`], not enough for a
/// component that gates on a rail *voltage*; see [`bench_rails`].
pub const fn pwr_out(number: &'static str) -> PinDecl {
    PinDecl {
        number,
        name: None,
        kind: PinKind::PowerOut,
        stream: None,
        drive_impedance: None,
    }
}

/// A terminal that contributes nothing electrical.
pub const fn passive(number: &'static str) -> PinDecl {
    PinDecl {
        number,
        name: None,
        kind: PinKind::Passive,
        stream: None,
        drive_impedance: None,
    }
}

/// A pin the schematic marks no-connect. Spelled distinctly from [`passive`]
/// so the tables read as documentation: the facade must still declare the pin
/// (the netlist has a node for it, on an `unconnected-(…)` stub net), and
/// declaring it passive keeps a deliberately dangling pad out of the findings.
pub const fn nc(number: &'static str) -> PinDecl {
    passive(number)
}

// ============================================================
// Topology-only stub
// ============================================================

/// A part with a real pin facade and no behavior. See the module docs for how
/// its pin kinds are chosen and why.
#[derive(Debug)]
pub struct StubPart {
    pins: &'static [PinDecl],
}

impl StubPart {
    /// A stub declaring `pins`.
    pub const fn new(pins: &'static [PinDecl]) -> Self {
        Self { pins }
    }
}

impl Component for StubPart {
    fn pins(&self) -> &[PinDecl] {
        self.pins
    }

    fn attach(&mut self, _io: ComponentNetIo) -> Result<(), AttachError> {
        Ok(())
    }
}

/// Register `part` as a [`StubPart`] declaring `pins`.
pub fn register_stub(registry: &mut PartRegistry, part: &str, pins: &'static [PinDecl]) {
    registry.register(part, move |_decl| Box::new(StubPart::new(pins)));
}
