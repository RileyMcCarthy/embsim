//! An [`Assembly`]: one component made of several, behind one set of pins.
//!
//! A plant is one bench component (`PROJECTS.md` §10, "What a project can
//! add"): its mechanism — a motor's shaft, a carriage, a sample, a load
//! cell — is inside it, in Rust, and its outside is electrical pins and the
//! reference each is measured against. Most of that mechanism is models
//! embsim already has (`embsim_models::machine`'s step/direction drive,
//! quadrature encoder and end switch), each a [`Component`] of its own. An
//! assembly hosts them, and a project's own, as **one** node:
//!
//! - **Its pins are the union its members expose.** Each member's declared
//!   pins are renamed onto the assembly's ([`Assembly::member`]) with every
//!   declaration kept — role, idle, thresholds, ports, clamps, the pins it
//!   is measured against — and its branches with them. Every pin a member
//!   declares is one of the assembly's: a link between two members is not a
//!   net inside the node, which the engine could not resolve (`DESIGN.md`
//!   rule 4), but the assembly's own code (below). A pin the members do not
//!   measure against anything may be given a return the assembly declares
//!   itself ([`Assembly::reference`]): a drive's input return, an encoder's
//!   ground.
//! - **Its members reach the engine through the one interface.** A member
//!   is attached with a handle table whose pins are the assembly's handles
//!   under the names the member declared, so its drives, its senses and its
//!   branches are the assembly's, on the assembly's nets (`DESIGN.md` rule
//!   2): no second channel, and no member sees another (rule 4).
//! - **Its members' time is the assembly's.** The engine holds one wake
//!   handler for the assembly. A member's wake handler and its schedules go
//!   through a [`WakeGate`] the assembly holds for it, as a package holds
//!   its core's (`embsim-boards`' `P2Package`): the assembly keeps each
//!   member's instants, arms each on the engine as its own, and when the
//!   engine wakes it hands the instant to exactly the members that asked
//!   for it, in the order they were added. A member is woken once an
//!   instant, as the engine wakes a node once an instant; a member asking
//!   again at the instant it is handed is asked again, as on the engine.
//!   A periodic request is the engine's rule too: anchored at the instant
//!   it is asked for, a missed period coalesced into one wake and the next
//!   counted from it.
//! - **The links between members are the assembly's code.** A shaft
//!   turning an encoder, a carriage opening a switch, a sample straining a
//!   load cell: the author connects its members' handles before handing the
//!   assembly to the system (a motor's position observer calling an
//!   encoder's input). Each link runs where the member that emits runs — in
//!   that member's sense callback or wake, on the engine's thread, at that
//!   instant — so it is the engine's time and no other clock. A link that
//!   needs instants of its own is a member with no pins that asks for them.
//!
//! What a link hands the next member is what the emitting member samples,
//! when it samples it. The drive model's position observer runs on a
//! periodic wake (`embsim_models::machine::stepper_motor`, its observation
//! cadence), so an encoder fed from it moves at those instants: its count
//! trails the carriage by at most one observation, and a switch fed from it
//! trips within one observation of the carriage crossing its point. The
//! carriage itself is the drive's closed form, read exact at every
//! observation: a different cadence moves when an edge lands, never how far
//! the carriage went, so the count at rest is the steps the drive folded
//! (`NODES.md` §17).

use std::collections::BTreeSet;
use std::fmt;
use std::sync::{Arc, Mutex};

use embsim_core::virtual_clock;

use crate::component::{
    AttachError, Branch, Component, ComponentNetIo, PinDecl, WakeGate, WakeHandler,
};

/// A member's wake handler as the assembly holds it: called with no lock of
/// the assembly's held, so the handler may ask for its next wake.
type SharedHandler = Arc<Mutex<WakeHandler>>;

/// One periodic request of a member: the instant it next falls due, and
/// its period.
#[derive(Debug)]
struct Period {
    member: usize,
    next_ns: u64,
    period_ns: u64,
}

/// Every member's instants: what the assembly arms on the engine as its
/// own, and hands back to the member that asked.
#[derive(Default)]
struct Agenda {
    /// Each member's wake handler, by the order the members were added.
    handlers: Vec<Option<SharedHandler>>,
    /// One-shot requests, `(instant, member)`: ordered by the instant, then
    /// by the order the members were added.
    oneshots: BTreeSet<(u64, usize)>,
    /// Periodic requests, in the order they were made.
    periods: Vec<Period>,
}

/// The time of one member, as the assembly hosts it: the [`WakeGate`]
/// behind the member's handle table.
struct MemberWakes {
    member: usize,
    agenda: Arc<Mutex<Agenda>>,
    /// The assembly's own handle table: where the instants go.
    io: ComponentNetIo,
}

impl MemberWakes {
    fn agenda(&self) -> std::sync::MutexGuard<'_, Agenda> {
        self.agenda
            .lock()
            .expect("the assembly's agenda is never poisoned")
    }
}

impl WakeGate for MemberWakes {
    fn on_wake_ns(&self, handler: WakeHandler) {
        // No slot once the assembly is dropped (`Drop`): nothing wakes it.
        if let Some(slot) = self.agenda().handlers.get_mut(self.member) {
            *slot = Some(Arc::new(Mutex::new(handler)));
        }
    }

    fn schedule_at_ns(&self, at_ns: u64) {
        self.agenda().oneshots.insert((at_ns, self.member));
        self.io.schedule_at_ns(at_ns);
    }

    fn schedule_every_ns(&self, period_ns: u64) {
        if period_ns == 0 {
            tracing::warn!(member = self.member, "assembly: schedule_every(0) ignored");
            return;
        }
        // Anchored at the instant it is asked for, which is the instant the
        // engine would anchor it at: asked on the engine's thread, or while
        // the system's time is held before it starts.
        let next_ns = virtual_clock::virtual_ns().saturating_add(period_ns);
        self.agenda().periods.push(Period {
            member: self.member,
            next_ns,
            period_ns,
        });
        self.io.schedule_at_ns(next_ns);
    }
}

/// One member: the component, its name, and the assembly pin each of its
/// declared pins is.
struct Member {
    name: String,
    component: Box<dyn Component>,
    /// Per declared pin, in the member's order: its number, its alias, and
    /// the assembly pin it is.
    pins: Vec<(&'static str, Option<&'static str>, &'static str)>,
}

/// Several components as one, behind one set of pins (module docs).
///
/// ```
/// use embsim_board::{Assembly, PinDecl, Component, ComponentNetIo, AttachError};
///
/// /// A part with one output, `OUT`.
/// struct Part([PinDecl; 1]);
///
/// impl Component for Part {
///     fn pins(&self) -> &[PinDecl] {
///         &self.0
///     }
///     fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
///         io.pin("OUT").map(|_| ())
///     }
/// }
///
/// let assembly = Assembly::new()
///     .member("LEFT", Box::new(Part([PinDecl::digital_out("OUT")])), &[("OUT", "L")])?
///     .member("RIGHT", Box::new(Part([PinDecl::digital_out("OUT")])), &[("OUT", "R")])?
///     .reference("GND", &["L", "R"])?;
/// let pins: Vec<&str> = assembly.pins().iter().map(|pin| pin.number).collect();
/// assert_eq!(pins, ["L", "R", "GND"]);
/// assert_eq!(assembly.pins()[0].reference, Some("GND"));
/// # Ok::<(), embsim_board::AssemblyError>(())
/// ```
pub struct Assembly {
    pins: Vec<PinDecl>,
    branches: Vec<Branch>,
    members: Vec<Member>,
    agenda: Arc<Mutex<Agenda>>,
}

impl fmt::Debug for Assembly {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Assembly")
            .field(
                "members",
                &self
                    .members
                    .iter()
                    .map(|m| m.name.as_str())
                    .collect::<Vec<_>>(),
            )
            .field(
                "pins",
                &self.pins.iter().map(|p| p.number).collect::<Vec<_>>(),
            )
            .finish()
    }
}

impl Default for Assembly {
    fn default() -> Self {
        Self::new()
    }
}

impl Assembly {
    /// An assembly with no members and no pins.
    pub fn new() -> Self {
        Self {
            pins: Vec::new(),
            branches: Vec::new(),
            members: Vec::new(),
            agenda: Arc::default(),
        }
    }

    /// Add `component` as the member `name`, each of its pins renamed onto
    /// the assembly's: `pins` pairs a pin the member declares (its number
    /// or its alias) with the assembly pin it is. Every pin the member
    /// declares is named once, and every assembly pin is one member's.
    ///
    /// The member's declarations are kept as it made them, the pins they
    /// name (a reference, a supply, a branch's ends) renamed with them.
    /// Members attach, start and are woken at a shared instant in the order
    /// they were added.
    ///
    /// Refused: a name that is empty or another member's; a pin the member
    /// does not declare, or one named twice; a pin the member declares and
    /// `pins` leaves out (a link between members is the assembly's code,
    /// not a pin); an assembly pin that is empty, holds a `.` (an endpoint
    /// is `Name.Pin`) or is already the assembly's; a declaration of the
    /// member's naming a pin the member does not declare.
    pub fn member(
        mut self,
        name: &str,
        component: Box<dyn Component>,
        pins: &[(&'static str, &'static str)],
    ) -> Result<Self, AssemblyError> {
        if name.is_empty() || self.members.iter().any(|m| m.name == name) {
            return Err(AssemblyError::MemberName {
                member: name.to_string(),
            });
        }
        let declared = component.pins();
        let mut outer: Vec<Option<&'static str>> = vec![None; declared.len()];
        for &(inner, assembly_pin) in pins {
            let index = declared
                .iter()
                .position(|pin| pin.answers_to(inner))
                .ok_or_else(|| AssemblyError::UnknownPin {
                    member: name.to_string(),
                    pin: inner.to_string(),
                })?;
            if outer[index].is_some() {
                return Err(AssemblyError::PinNamedTwice {
                    member: name.to_string(),
                    pin: inner.to_string(),
                });
            }
            self.check_new_pin(assembly_pin)?;
            if outer.contains(&Some(assembly_pin)) {
                return Err(AssemblyError::PinName {
                    pin: assembly_pin.to_string(),
                    reason: "it is already one of the assembly's pins",
                });
            }
            outer[index] = Some(assembly_pin);
        }
        let outer: Vec<&'static str> = declared
            .iter()
            .zip(&outer)
            .map(|(pin, assembly_pin)| {
                assembly_pin.ok_or_else(|| AssemblyError::PinNotExposed {
                    member: name.to_string(),
                    pin: pin.number.to_string(),
                })
            })
            .collect::<Result<_, _>>()?;
        // A declaration names a pin of the member's by number or alias;
        // renamed, it names the assembly pin that pin is — never another
        // member's pin of the same spelling.
        let rename = |id: &'static str| -> Result<&'static str, AssemblyError> {
            declared
                .iter()
                .position(|pin| pin.answers_to(id))
                .map(|index| outer[index])
                .ok_or_else(|| AssemblyError::UndeclaredPin {
                    member: name.to_string(),
                    pin: id.to_string(),
                })
        };
        let mut renamed = Vec::with_capacity(declared.len());
        for (pin, &assembly_pin) in declared.iter().zip(&outer) {
            let mut pin = *pin;
            pin.number = assembly_pin;
            pin.name = None;
            pin.reference = pin.reference.map(rename).transpose()?;
            pin.supply = pin.supply.map(rename).transpose()?;
            renamed.push(pin);
        }
        let mut branches = Vec::with_capacity(component.branches().len());
        for branch in component.branches() {
            let mut branch = *branch;
            branch.a = rename(branch.a)?;
            branch.b = rename(branch.b)?;
            branch.control = branch
                .control
                .map(|(pin, test)| rename(pin).map(|pin| (pin, test)))
                .transpose()?;
            branches.push(branch);
        }
        let identities = declared
            .iter()
            .zip(&outer)
            .map(|(pin, &assembly_pin)| (pin.number, pin.name, assembly_pin))
            .collect();
        self.pins.extend(renamed);
        self.branches.extend(branches);
        self.members.push(Member {
            name: name.to_string(),
            component,
            pins: identities,
        });
        self.agenda
            .lock()
            .expect("the assembly's agenda is never poisoned")
            .handlers
            .push(None);
        Ok(self)
    }

    /// Measure each of `pins` against `reference`: the return a member's
    /// inputs are read across, the ground its outputs are driven against —
    /// what the board joins it to, so no ground is common inside the
    /// assembly that the board does not join. `reference` is one of the
    /// assembly's pins, or a return the assembly declares itself, a power
    /// input no member hosts ([`PinDecl::power_in`]). A sense through each
    /// pin is then measured against it, as any part's against its declared
    /// reference; a drive's volts stay in the engine's frame, as every
    /// part's do (`DESIGN.md` §2).
    ///
    /// Refused: a pin that is not the assembly's, that is `reference`
    /// itself, or that its member already measures against another pin (a
    /// member's own declaration stands); a new `reference` that is not a
    /// pin name.
    pub fn reference(
        mut self,
        reference: &'static str,
        pins: &[&'static str],
    ) -> Result<Self, AssemblyError> {
        if !self.pins.iter().any(|pin| pin.number == reference) {
            self.check_new_pin(reference)?;
            self.pins.push(PinDecl::power_in(reference));
        }
        for &measured in pins {
            let reason = if measured == reference {
                Some("a pin is not measured against itself")
            } else {
                match self.pins.iter().find(|pin| pin.number == measured) {
                    None => Some("it is not one of the assembly's pins"),
                    Some(pin) if pin.reference.is_some() => {
                        Some("its member already measures it against another pin")
                    }
                    Some(_) => None,
                }
            };
            if let Some(reason) = reason {
                return Err(AssemblyError::Reference {
                    pin: measured.to_string(),
                    reference: reference.to_string(),
                    reason,
                });
            }
            if let Some(pin) = self.pins.iter_mut().find(|pin| pin.number == measured) {
                pin.reference = Some(reference);
            }
        }
        Ok(self)
    }

    /// The members' names, in the order they were added.
    pub fn members(&self) -> impl Iterator<Item = &str> {
        self.members.iter().map(|member| member.name.as_str())
    }

    /// Refuse `pin` as a new assembly pin.
    fn check_new_pin(&self, pin: &str) -> Result<(), AssemblyError> {
        let reason = if pin.is_empty() {
            Some("a pin needs a name")
        } else if pin.contains('.') {
            Some("an endpoint is Name.Pin, so a pin's name holds no '.'")
        } else if self.pins.iter().any(|declared| declared.number == pin) {
            Some("it is already one of the assembly's pins")
        } else {
            None
        };
        match reason {
            Some(reason) => Err(AssemblyError::PinName {
                pin: pin.to_string(),
                reason,
            }),
            None => Ok(()),
        }
    }

    /// The engine woke the assembly at `now`: hand the instant to every
    /// member it is due for, in the order they were added, each once, and
    /// arm the next instant of each periodic request that fell due.
    fn dispatch(agenda: &Mutex<Agenda>, io: &ComponentNetIo, now: u64) {
        let (due, rearm) = {
            let mut agenda = agenda
                .lock()
                .expect("the assembly's agenda is never poisoned");
            let mut members = BTreeSet::new();
            while let Some(&(at_ns, member)) = agenda.oneshots.first() {
                if at_ns > now {
                    break;
                }
                agenda.oneshots.pop_first();
                members.insert(member);
            }
            let mut rearm = Vec::new();
            for period in agenda.periods.iter_mut().filter(|p| p.next_ns <= now) {
                members.insert(period.member);
                // The engine's rule: the next period from the instant it
                // fell due, or from now when that is already past.
                let mut next = period.next_ns.saturating_add(period.period_ns);
                if next <= now {
                    next = now.saturating_add(period.period_ns);
                }
                period.next_ns = next;
                rearm.push(next);
            }
            let due: Vec<SharedHandler> = members
                .into_iter()
                .filter_map(|member| agenda.handlers.get(member).cloned().flatten())
                .collect();
            (due, rearm)
        };
        for at_ns in rearm {
            io.schedule_at_ns(at_ns);
        }
        for handler in due {
            (handler.lock().expect("a member's wake is never poisoned"))(now);
        }
    }
}

impl Component for Assembly {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn branches(&self) -> &[Branch] {
        &self.branches
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        // The assembly's one wake handler, registered before any member can
        // ask for an instant.
        {
            let agenda = Arc::clone(&self.agenda);
            let rearm = io.clone();
            io.on_wake_ns(move |now| Self::dispatch(&agenda, &rearm, now));
        }
        for (index, member) in self.members.iter_mut().enumerate() {
            let mut entries = Vec::with_capacity(member.pins.len() * 2);
            for &(number, alias, assembly_pin) in &member.pins {
                let handle = io.pin(assembly_pin)?;
                if let Some(alias) = alias {
                    entries.push((alias.to_string(), handle.clone()));
                }
                entries.push((number.to_string(), handle));
            }
            let gate = Arc::new(MemberWakes {
                member: index,
                agenda: Arc::clone(&self.agenda),
                io: io.clone(),
            });
            let name = &member.name;
            member
                .component
                .attach(io.for_member(entries, gate))
                .map_err(|error| match error {
                    AttachError::UnknownPin { pin } => AttachError::UnknownPin {
                        pin: format!("{name}.{pin}"),
                    },
                    AttachError::Failed { message } => AttachError::Failed {
                        message: format!("member {name}: {message}"),
                    },
                })?;
        }
        Ok(())
    }

    fn start(&mut self) {
        for member in &mut self.members {
            member.component.start();
        }
    }
}

impl Drop for Assembly {
    /// A member's wake handler may hold the member's own handle table,
    /// whose gate holds the agenda the handler is in: the handlers go with
    /// the assembly, so nothing a member's wake holds outlives it.
    fn drop(&mut self) {
        if let Ok(mut agenda) = self.agenda.lock() {
            agenda.handlers.clear();
            agenda.oneshots.clear();
            agenda.periods.clear();
        }
    }
}

/// Why an [`Assembly`] refused a member or a reference.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AssemblyError {
    /// A member's name is empty or another member's.
    MemberName {
        /// The name.
        member: String,
    },
    /// The pin map names a pin the member does not declare.
    UnknownPin {
        /// The member.
        member: String,
        /// The pin, as the map names it.
        pin: String,
    },
    /// The pin map names one of the member's pins twice.
    PinNamedTwice {
        /// The member.
        member: String,
        /// The pin, as the map names it the second time.
        pin: String,
    },
    /// The member declares a pin the map leaves out.
    PinNotExposed {
        /// The member.
        member: String,
        /// The pin's number.
        pin: String,
    },
    /// An assembly pin's name is not one a pin can have.
    PinName {
        /// The name.
        pin: String,
        /// Why not.
        reason: &'static str,
    },
    /// A declaration of the member's names a pin the member does not
    /// declare.
    UndeclaredPin {
        /// The member.
        member: String,
        /// The pin the declaration names.
        pin: String,
    },
    /// A pin cannot be measured against the reference asked for.
    Reference {
        /// The pin.
        pin: String,
        /// The reference.
        reference: String,
        /// Why not.
        reason: &'static str,
    },
}

impl fmt::Display for AssemblyError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            AssemblyError::MemberName { member } => write!(
                f,
                "member {member:?}: each member has a name of its own, and no other member's"
            ),
            AssemblyError::UnknownPin { member, pin } => write!(
                f,
                "member {member}: it declares no pin {pin:?}; name each of its pins by its \
                 number or alias"
            ),
            AssemblyError::PinNamedTwice { member, pin } => write!(
                f,
                "member {member}: pin {pin:?} is named twice; each of its pins is one \
                 assembly pin"
            ),
            AssemblyError::PinNotExposed { member, pin } => write!(
                f,
                "member {member}: its pin {pin:?} is none of the assembly's; every pin a member \
                 declares is one of the assembly's, and a link between members is the \
                 assembly's code, not a pin (DESIGN.md rule 4)"
            ),
            AssemblyError::PinName { pin, reason } => {
                write!(f, "assembly pin {pin:?}: {reason}")
            }
            AssemblyError::UndeclaredPin { member, pin } => write!(
                f,
                "member {member}: a declaration of its pins names {pin:?}, which it does not \
                 declare"
            ),
            AssemblyError::Reference {
                pin,
                reference,
                reason,
            } => write!(f, "pin {pin:?} against {reference:?}: {reason}"),
        }
    }
}

impl std::error::Error for AssemblyError {}

#[cfg(test)]
mod tests {
    use rstest::rstest;
    use vibes_behaviour::{behaviour, expect, Test};

    use super::*;
    use crate::component::{jesd8c01_lvcmos_thresholds, DeadBand, PwlCurve};

    /// A member that declares what it is given and does nothing.
    struct Declares {
        pins: Vec<PinDecl>,
        branches: Vec<Branch>,
    }

    impl Declares {
        fn new(pins: &[PinDecl]) -> Box<Self> {
            Box::new(Self {
                pins: pins.to_vec(),
                branches: Vec::new(),
            })
        }
    }

    impl Component for Declares {
        fn pins(&self) -> &[PinDecl] {
            &self.pins
        }

        fn branches(&self) -> &[Branch] {
            &self.branches
        }

        fn attach(&mut self, _io: ComponentNetIo) -> Result<(), AttachError> {
            Ok(())
        }
    }

    const INPUT: PinDecl = PinDecl::digital_in("1", jesd8c01_lvcmos_thresholds(DeadBand::Unknown))
        .with_name("IN")
        .with_reference("2");
    const RETURN: PinDecl = PinDecl::power_in("2").with_name("GND");

    #[rstest]
    fn an_assemblys_pins_are_its_members_pins_renamed() {
        behaviour!(Test {
            id: "assembly.pins-are-the-members-renamed",
            covers: Some("board/src/assembly.rs#Assembly::member"),
            given: "an assembly of two members, one with an input measured against its own \
                    ground and a diode between the two, the other with a single output, each \
                    pin given an assembly name",
        });
        expect!(
            "union-in-order",
            "the assembly's pins are every member's pins under their assembly names, member \
             by member in the order they were added"
        );
        expect!(
            "declarations-kept",
            "each pin keeps its member's declaration: its role, its thresholds and the drive \
             it rests at"
        );
        expect!(
            "named-pins-follow",
            "a pin a member's declaration names, the ground an input is measured against or \
             a diode's ends, is renamed to that pin's assembly name",
            "a declaration names the member's own pin, never another member's pin that \
             happens to share its number"
        );
        let diode = Branch {
            a: "IN",
            b: "GND",
            curve: PwlCurve::Diode { vf: 0.7, r_d: 1.0 },
            control: None,
        };
        let first = Box::new(Declares {
            pins: vec![INPUT, RETURN],
            branches: vec![diode],
        });
        let assembly = Assembly::new()
            .member("SENSOR", first, &[("IN", "SENSE"), ("2", "SENSOR_GND")])
            .expect("the sensor joins")
            .member(
                "DRIVER",
                Declares::new(&[PinDecl::digital_out("1")]),
                &[("1", "OUT")],
            )
            .expect("the driver joins");
        let numbers: Vec<&str> = assembly.pins().iter().map(|pin| pin.number).collect();
        assert_eq!(numbers, ["SENSE", "SENSOR_GND", "OUT"]);
        let sense = assembly.pins()[0];
        assert_eq!(sense.role, INPUT.role);
        assert_eq!(sense.thresholds, INPUT.thresholds);
        assert_eq!(sense.name, None);
        assert_eq!(assembly.pins()[2].idle, PinDecl::digital_out("1").idle);
        assert_eq!(sense.reference, Some("SENSOR_GND"));
        assert_eq!(
            assembly.branches(),
            [Branch {
                a: "SENSE",
                b: "SENSOR_GND",
                ..diode
            }]
        );
    }

    #[rstest]
    #[case::empty_member_name("", &[("1", "OUT")], "each member has a name of its own")]
    #[case::member_named_twice("FIRST", &[("1", "B")], "each member has a name of its own")]
    #[case::unknown_pin("SECOND", &[("9", "B")], "it declares no pin \"9\"")]
    #[case::pin_named_twice("SECOND", &[("1", "B"), ("1", "C")], "is named twice")]
    #[case::pin_left_out("SECOND", &[], "a link between members is the assembly's code")]
    #[case::pin_name_taken("SECOND", &[("1", "A")], "already one of the assembly's pins")]
    #[case::dotted_pin_name("SECOND", &[("1", "B.1")], "holds no '.'")]
    #[case::empty_pin_name("SECOND", &[("1", "")], "a pin needs a name")]
    fn an_assembly_refuses_a_member_whose_pins_do_not_map_one_to_one(
        #[case] name: &str,
        #[case] pins: &[(&'static str, &'static str)],
        #[case] says: &str,
    ) {
        behaviour!(Test {
            id: "assembly.refuses-a-member",
            covers: Some("board/src/assembly.rs#Assembly::member"),
            given: "an assembly with one member, asked to add a second with no name or the \
                    first one's, or with its single pin unnamed, named twice, named as a pin \
                    it does not have, or given an assembly name that is empty, holds a dot \
                    or is already taken",
        });
        expect!(
            "refused-saying-why",
            "the second member is refused with a message naming what is wrong",
            "every pin of every member is exactly one assembly pin, so the engine sees each \
             of the members' pins on a net of its own"
        );
        let assembly = Assembly::new()
            .member(
                "FIRST",
                Declares::new(&[PinDecl::digital_out("1")]),
                &[("1", "A")],
            )
            .expect("the first member joins");
        let error = assembly
            .member(name, Declares::new(&[PinDecl::digital_out("1")]), pins)
            .expect_err("the second member is refused");
        assert!(error.to_string().contains(says), "{error}");
    }

    #[rstest]
    fn an_assembly_declares_a_return_its_members_pins_are_measured_against() {
        behaviour!(Test {
            id: "assembly.reference",
            covers: Some("board/src/assembly.rs#Assembly::reference"),
            given: "an assembly of a member with two inputs measured against nothing, given a \
                    return of the assembly's own",
        });
        expect!(
            "return-is-a-pin",
            "the return is one more pin of the assembly, a power input no member hosts"
        );
        expect!(
            "measured-against-it",
            "each input it is given for is measured against it"
        );
        expect!(
            "member-reference-stands",
            "a pin its member already measures against a pin of its own is refused, as is a \
             pin the assembly does not have",
            "a member's own declaration of what a pin is measured against is the member's \
             model, and stands"
        );
        let inputs = [
            PinDecl::digital_in("STEP", jesd8c01_lvcmos_thresholds(DeadBand::Unknown)),
            PinDecl::digital_in("DIR", jesd8c01_lvcmos_thresholds(DeadBand::Unknown)),
        ];
        let assembly = Assembly::new()
            .member(
                "DRIVE",
                Declares::new(&inputs),
                &[("STEP", "STEP"), ("DIR", "DIR")],
            )
            .expect("the drive joins")
            .reference("DRIVE_GND", &["STEP", "DIR"])
            .expect("the return is declared");
        assert_eq!(
            assembly.pins()[2],
            PinDecl::power_in("DRIVE_GND"),
            "the return"
        );
        assert!(assembly.pins()[..2]
            .iter()
            .all(|pin| pin.reference == Some("DRIVE_GND")));

        let sensor = Assembly::new()
            .member(
                "SENSOR",
                Declares::new(&[INPUT, RETURN]),
                &[("1", "S"), ("2", "S_GND")],
            )
            .expect("the sensor joins");
        let error = sensor
            .reference("OTHER", &["S"])
            .expect_err("the member's own reference stands");
        assert!(error.to_string().contains("already measures it"), "{error}");
        let error = Assembly::new()
            .reference("GND", &["NOWHERE"])
            .expect_err("a pin the assembly lacks");
        assert!(
            error.to_string().contains("not one of the assembly's pins"),
            "{error}"
        );
    }
}
