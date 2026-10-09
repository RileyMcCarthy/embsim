//! Structured findings on a diagnostics bus, mirrored to `tracing`.
//!
//! Findings are the engine's way of reporting electrical/topological problems
//! without panicking: tests assert that a specific [`Finding`] fired, trace
//! tooling can consume the same bus later. The [`Diagnostics`] collector is
//! Vec-based; every reported finding is also emitted as a `tracing` warning.

use std::fmt;

use crate::net::{Ohms, PinRef, Volts};

// ============================================================
// Findings
// ============================================================

/// Which sense domain observed a floating net.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SenseKind {
    /// A digital sense pin (e.g. a floating `~RESET`).
    Digital,
    /// An analog sense pin (e.g. a floating ADC input).
    Analog,
}

/// Which engine-thread delivery a contained callback panic escaped from.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CallbackKind {
    /// A net-sense delivery.
    Sense,
    /// A timer-wheel wakeup delivery.
    Wake,
    /// A topology-epoch notification.
    Topology,
}

/// Direction of a pin-facade mismatch between a registered component's
/// declared pins and the netlist.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PinMismatchDirection {
    /// The component declares the pin but the netlist has no such node.
    DeclaredButAbsent,
    /// The netlist has the node but the component does not declare it.
    PresentButUndeclared,
}

/// Why a rail is down at build ([`Finding::RailDown`]): what the build can
/// see of the part from the outside — its supply pins and its declared
/// reference — with the part's own gate (its threshold, its enable, its
/// soft-start) the third case, named as such.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RailDownReason {
    /// A power-in pin of the part is on a net no source reaches: the rail
    /// has nothing to regulate from.
    InputUnsourced {
        /// The unsourced power-in pin.
        pin: String,
    },
    /// The output's declared reference pin is on a net no source reaches:
    /// the rail has nothing to measure its voltage against, so it publishes
    /// none (`NODES.md` §2, the Regulator row: an unheld reference is a
    /// floating output).
    ReferenceUnheld {
        /// The reference pin.
        pin: String,
    },
    /// The part's supply and reference are sourced and it holds the output
    /// released anyway: its input is below its threshold, its enable is
    /// off, or its soft-start has not elapsed — the build snapshot is the
    /// state before the first wake, and a rail with a soft-start is down in
    /// it (`NODES.md` §5). The part's own monitor says which.
    HeldDown,
}

/// One structured diagnostic finding. A finding, never a panic.
#[derive(Debug, Clone, PartialEq)]
pub enum Finding {
    /// Strong sources (under [`crate::net::WEAK_DRIVE_OHMS`] in total)
    /// fighting on one net: one lost to a source ten times stronger, or
    /// comparable ones solved to a divided voltage. Names every strong pin
    /// on the net; an ideal source (a rail, a `net_stuck`) has none to name.
    Contention {
        /// Net name.
        net: String,
        /// The fighting driver pins.
        drivers: Vec<PinRef>,
    },
    /// A sensing pin observes a net no source reaches. The sensing component
    /// chooses datasheet behavior — the engine never invents a value silently.
    FloatingSense {
        /// Net name.
        net: String,
        /// Digital or analog sense domain.
        kind: SenseKind,
    },
    /// Disagreeing sources of comparable strength solved to a node voltage
    /// strictly inside the [`crate::net::V_IL`]/[`crate::net::V_IH`] dead
    /// band: neither a valid low nor a valid high, so the net projects
    /// [`crate::NetState::Contention`] and this names the voltage it
    /// actually sits at. Reported beside the fight's `Contention` finding.
    AmbiguousLevel {
        /// Net name.
        net: String,
        /// The solved node voltage that fell inside the dead band.
        volts: Volts,
    },
    /// A [`crate::Drive::Current`] injected into a net no Thevenin source
    /// reaches. A current source has no open-circuit voltage and no return
    /// path here, so the net stays [`crate::NetState::Floating`] and the
    /// injection goes nowhere — a modelling error, never an invented
    /// voltage.
    CurrentIntoFloatingNode {
        /// Net name.
        net: String,
        /// The injecting pin.
        pin: PinRef,
    },
    /// A periodic drive's rate reached a coupling capacitor whose reactance
    /// at that rate is not small against the far node's resistance
    /// (`1/(2π·f·C) > R_far /` [`crate::net::COUPLING_REACTANCE_RATIO`]), so
    /// the rate does not cross: it stops at the capacitor and the node
    /// beyond it keeps the state its own sources give it. Raised by the pass
    /// that resolves the far node while the rate is on the source's pin, at
    /// build or live. Reported once per distinct occurrence like every
    /// finding (the bus dedups on equality): a segment re-published at one
    /// rate raises it once, and a rate that changes is a new verdict with
    /// its own reactance.
    PeriodicNotCoupled {
        /// The net on the far side of the capacitor.
        net: String,
        /// The capacitor's reference designator.
        capacitor: String,
        /// The rate.
        hz: u32,
        /// The capacitor's reactance at that rate.
        reactance_ohms: Ohms,
        /// The far node's resistance estimate the reactance was judged
        /// against (the smallest resistor touching the node; `+∞` for none).
        far_ohms: Ohms,
    },
    /// A cluster's piecewise-linear elements found no consistent set of
    /// regions: the flip loop — every element off, then the first element
    /// whose region test disagrees with its state flipped, one per solve,
    /// in declaration order — ran its bound of
    /// [`crate::cluster::PWL_SOLVES_PER_ELEMENT`] solves per element and a
    /// test still disagreed. Two elements whose tests chase each other (an
    /// inverting loop with no rest state) do this. The cluster then has no
    /// operating point: every node of it publishes
    /// [`crate::NetState::Floating`] (a terminal keeps its constant) — never
    /// `NaN`, never the last set of regions tried — and this names the
    /// elements and the solve count (`NODES.md` §7).
    NonConvergent {
        /// The cluster, named by its lowest-indexed net.
        cluster: String,
        /// The elements in the cluster, in declaration order, as
        /// `Board.Reference`.
        elements: Vec<String>,
        /// Linear solves the loop ran before giving up: the bound.
        solves: usize,
    },
    /// A power net with no `PowerOut` source anywhere (board or harness);
    /// presents as down (0 V into cluster solves).
    PowerNetUnsourced {
        /// Net name.
        net: String,
    },
    /// A netlist component could not be classified (no auto tier match, no
    /// registry entry, pin-count violation, …).
    ClassificationError {
        /// Component reference designator.
        reference: String,
        /// Libsource part name (rescue-normalized).
        part: String,
        /// Human-readable cause.
        message: String,
    },
    /// A component-provided callback (sense, wake, stream-byte, or
    /// topology delivery) panicked on the engine thread. The panic is
    /// contained — the engine stays alive and net service continues for
    /// every other component — but the panicking component's own state is
    /// suspect. Reported once per (kind, subscriber).
    CallbackPanic {
        /// Which delivery panicked.
        kind: CallbackKind,
        /// Identity of the failing subscriber (net name for senses,
        /// component index for wakes, consumer pin for stream bytes).
        subscriber: String,
    },
    /// The engine needed `embsim_core::virtual_clock` (a `schedule_at` /
    /// `schedule_every` request or a paced stream write) before
    /// `virtual_clock::init` ran. The request is dropped loudly instead of
    /// panicking the engine thread into a silent zombie.
    VirtualClockUninitialized {
        /// What needed the clock.
        context: String,
    },
    /// A reserved drive enqueue sequence number never arrived (the
    /// enqueuing thread died between reserving the seq and sending the
    /// command). After a bounded wait the engine skips the gap — ordering
    /// against a dead enqueuer is moot — so later drives from every other
    /// component are not wedged forever.
    DriveSeqGap {
        /// The first missing sequence number.
        seq: u64,
    },
    /// **Stepped clock mode only.** A registered
    /// `embsim_core::virtual_clock` actor never parked at a virtual deadline,
    /// so the engine could not reach the quiescence barrier
    /// (`DETERMINISM.md` T1 §4) within its timeout.
    ///
    /// The engine advances anyway rather than hanging — but **this finding
    /// voids the run's determinism guarantee**, and is the marker a
    /// golden-trace comparison should fail on rather than mysteriously
    /// diverge. Common causes: an actor blocked on a real file descriptor or a
    /// host mutex (neither is visible to the barrier — that is Phase D2's
    /// transport work), or an actor spinning without ever calling a
    /// `virtual_clock` wait.
    QuiescenceTimeout {
        /// Names of the actors still runnable, in registration order.
        actors: Vec<String>,
    },
    /// Pin-facade mismatch between a registered component and the netlist
    /// (both directions are hard build errors; the finding carries the
    /// specifics).
    UnconnectedRegistryPin {
        /// Component reference designator.
        reference: String,
        /// Pin identity (number, or declared name when the number is absent).
        pin: String,
        /// Which side declared the pin the other lacks.
        direction: PinMismatchDirection,
    },
    /// A rail — a `PowerOut` pin's net — that sources nothing at build: the
    /// terminal is released, by the part or because nothing holds it.
    /// Raised by the build for every such pin that is not itself another
    /// pin's declared reference (an isolated ground is held by the board or
    /// the harness, never a rail), with the reason the build can see
    /// ([`RailDownReason`]).
    RailDown {
        /// The part, as `Board.Reference`.
        part: String,
        /// The `PowerOut` pin.
        pin: String,
        /// Why.
        reason: RailDownReason,
    },
    /// A pin whose net a source reaches while the net of its declared
    /// reference pin ([`crate::PinDecl::reference`]) reaches none: the domain is
    /// live and its voltages are measured against nothing. An isolator
    /// whose secondary ground is unwired, an isolated supply whose
    /// return nothing ties down.
    UnreferencedDomain {
        /// The part, as `Board.Reference`.
        part: String,
        /// The live pin.
        pin: String,
        /// Its reference pin, on a net no source reaches.
        reference: String,
    },
    /// A power-in pin with no capacitor between its node and its declared
    /// reference pin's node: a supply pin the layout does not decouple. Two
    /// caps on the same two nodes are one; a cap to any other node is none.
    UndecoupledPowerPin {
        /// The part, as `Board.Reference`.
        part: String,
        /// The power-in pin.
        pin: String,
        /// Its reference pin.
        reference: String,
    },
    /// An open-drain pin — a signal pin that sinks and cannot source
    /// ([`crate::PinDecl::can_source`]) — on a net no pull-up reaches: no
    /// resistive path, a declared terminal ending it, leads from the net
    /// to a rail, a supply above 0 V, a pin that sources, or an input port
    /// biased above 0 V. Released, the pin leaves its net floating: a
    /// supervisor's reset, a regulator's power-good or an opto's collector
    /// that nothing pulls up reads no level at all. An open drain whose
    /// path reaches no other part's pin — a no-connect, or a net that
    /// leaves the board only through a connector — raises nothing: nothing
    /// on the board reads it, and its pull-up is the far side's. Raised by
    /// the build (`NODES.md` §10).
    OpenDrainWithoutPullUp {
        /// The part, as `Board.Reference`.
        part: String,
        /// The open-drain pin.
        pin: String,
        /// Its net.
        net: String,
    },
    /// A mechanical node's pad — a mounting hole, a fiducial, a layout node
    /// — shares a net with a pin that drives it: a driver is loaded by a
    /// pad the schematic meant to be ground or nothing. A pad on a declared
    /// terminal (a ground the harness holds) or on no net raises nothing.
    MechanicalOnDrivenNet {
        /// The mechanical part, as `Board.Reference`.
        part: String,
        /// The net.
        net: String,
        /// The pins driving it.
        drivers: Vec<PinRef>,
    },
    /// A pin sits above the recommended operating range its part declares
    /// for it ([`crate::PinLimits`]): the part runs, and the finding says
    /// what is no longer guaranteed. The part declares the limits; the
    /// engine checks them against the solved net — at build over the
    /// settled snapshot, and live once each time the pin's voltage leaves
    /// the range upward (the voltage named is the first one above it).
    PinAboveRecommended {
        /// The part, as `Board.Reference`.
        part: String,
        /// The pin, by number.
        pin: String,
        /// The pin's voltage against its declared reference.
        volts: Volts,
        /// The recommended range's minimum.
        min: Volts,
        /// The recommended range's maximum.
        max: Volts,
        /// The absolute maximum rating, where the part declares one.
        absolute_max: Option<Volts>,
        /// What the part says running above the range costs, in plain
        /// words; empty for nothing more.
        note: String,
    },
    /// The build-time fixed point did not settle within its bound: after
    /// `passes` rounds of replaying the drives components issued in response
    /// to the states they were delivered, some component was still changing
    /// its drive. The build snapshot then describes the last pass, not a
    /// rest state, and cannot be relied on to equal the live system's
    /// pre-wake state (`System::build`). Names the nets whose states were
    /// still moving on the last pass.
    BuildNotSettled {
        /// Rounds of the fixed point that ran (the bound).
        passes: usize,
        /// Nets whose state changed on the last round, by name.
        nets: Vec<String>,
    },
}

// ============================================================
// Plain words
// ============================================================

/// A list of pins as `Reference.Pin`, comma-separated, in the order held.
fn pins(pins: &[PinRef]) -> String {
    pins.iter()
        .map(|pin| format!("{}.{}", pin.reference, pin.pin))
        .collect::<Vec<_>>()
        .join(", ")
}

impl fmt::Display for SenseKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::Digital => "digital",
            Self::Analog => "analog",
        })
    }
}

impl fmt::Display for CallbackKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::Sense => "sense",
            Self::Wake => "wake",
            Self::Topology => "topology",
        })
    }
}

/// One line in plain words: what on the board the finding is about — the
/// net, the pins as `Reference.Pin`, the part — and what is wrong with it.
/// Deterministic: lists print in the order the finding holds them and
/// numbers in Rust's shortest round-trip form, so two runs that make the
/// same finding print the same line. `Debug` stays the structural form
/// (traces, test failures).
impl fmt::Display for Finding {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Contention { net, drivers } if drivers.is_empty() => {
                write!(f, "contention on {net}: ideal sources fight to hold it")
            }
            Self::Contention { net, drivers } => {
                write!(
                    f,
                    "contention on {net}: {} fight to drive it",
                    pins(drivers)
                )
            }
            Self::FloatingSense { net, kind } => {
                let article = match kind {
                    SenseKind::Digital => "a",
                    SenseKind::Analog => "an",
                };
                write!(
                    f,
                    "no source reaches {net}, which {article} {kind} input reads"
                )
            }
            Self::AmbiguousLevel { net, volts } => write!(
                f,
                "{net} sits at {volts} V, neither a valid low nor a valid high"
            ),
            Self::CurrentIntoFloatingNode { net, pin } => write!(
                f,
                "{}.{} injects a current into {net}, which no voltage source reaches, so the \
                 current goes nowhere",
                pin.reference, pin.pin
            ),
            Self::PeriodicNotCoupled {
                net,
                capacitor,
                hz,
                reactance_ohms,
                far_ohms,
            } => {
                write!(
                    f,
                    "a {hz} Hz signal does not cross {capacitor} to {net}: the capacitor's \
                     {reactance_ohms} Ω at that rate is not small against "
                )?;
                if far_ohms.is_finite() {
                    write!(f, "the {far_ohms} Ω at the far node")
                } else {
                    write!(f, "the far node, which no resistor touches")
                }
            }
            Self::NonConvergent {
                cluster,
                elements,
                solves,
            } => write!(
                f,
                "the cluster at {cluster} has no operating point: its elements ({}) found no \
                 consistent regions in {solves} solves, so its nets float",
                elements.join(", ")
            ),
            Self::PowerNetUnsourced { net } => {
                write!(f, "power net {net} has no source")
            }
            Self::ClassificationError {
                reference,
                part,
                message,
            } => write!(f, "{reference} ({part}) cannot be classified: {message}"),
            Self::CallbackPanic { kind, subscriber } => write!(
                f,
                "a {kind} callback for {subscriber} panicked on the engine thread: the engine \
                 carries on, but that part's own state is suspect"
            ),
            Self::VirtualClockUninitialized { context } => write!(
                f,
                "{context} needed the virtual clock before it was started, and the request was \
                 dropped"
            ),
            Self::DriveSeqGap { seq } => write!(
                f,
                "drive {seq} never arrived (the thread that sent it died), and the engine \
                 skipped it"
            ),
            Self::QuiescenceTimeout { actors } if actors.is_empty() => write!(
                f,
                "the engine advanced without waiting for a part: this run is not reproducible"
            ),
            Self::QuiescenceTimeout { actors } => write!(
                f,
                "the engine advanced without waiting for {}: this run is not reproducible",
                actors.join(", ")
            ),
            Self::UnconnectedRegistryPin {
                reference,
                pin,
                direction: PinMismatchDirection::DeclaredButAbsent,
            } => write!(
                f,
                "{reference}'s model declares pin {pin}, which the netlist does not have"
            ),
            Self::UnconnectedRegistryPin {
                reference,
                pin,
                direction: PinMismatchDirection::PresentButUndeclared,
            } => write!(
                f,
                "the netlist gives {reference} a pin {pin} its model does not declare"
            ),
            Self::RailDown { part, pin, reason } => {
                write!(f, "rail {part}.{pin} is down at build: ")?;
                match reason {
                    RailDownReason::InputUnsourced { pin: input } => {
                        write!(f, "its supply pin {part}.{input} has no source")
                    }
                    RailDownReason::ReferenceUnheld { pin: reference } => write!(
                        f,
                        "its reference {part}.{reference} is on a net no source reaches"
                    ),
                    RailDownReason::HeldDown => write!(
                        f,
                        "the part holds it off (its input below threshold, its enable off, or \
                         its soft-start not done)"
                    ),
                }
            }
            Self::UnreferencedDomain {
                part,
                pin,
                reference,
            } => write!(
                f,
                "{part}.{pin} is live, but its reference {part}.{reference} is on a net no \
                 source reaches"
            ),
            Self::UndecoupledPowerPin {
                part,
                pin,
                reference,
            } => write!(
                f,
                "supply pin {part}.{pin} has no capacitor to its reference {part}.{reference}"
            ),
            Self::OpenDrainWithoutPullUp { part, pin, net } => write!(
                f,
                "open-drain pin {part}.{pin} has no pull-up on {net}: released, it leaves the \
                 net floating"
            ),
            Self::MechanicalOnDrivenNet { part, net, drivers } => write!(
                f,
                "mechanical pad {part} is on {net}, which {} drive{}",
                pins(drivers),
                if drivers.len() == 1 { "s" } else { "" }
            ),
            Self::PinAboveRecommended {
                part,
                pin,
                volts,
                min,
                max,
                absolute_max,
                note,
            } => {
                write!(
                    f,
                    "pin {part}.{pin} is at {volts} V, above its recommended {min} V to {max} V"
                )?;
                if let Some(abs) = absolute_max {
                    let side = if volts > abs { "beyond" } else { "within" };
                    write!(f, ", {side} its {abs} V absolute maximum")?;
                }
                if note.is_empty() {
                    Ok(())
                } else {
                    write!(f, ": {note}")
                }
            }
            Self::BuildNotSettled { passes, nets } if nets.is_empty() => {
                write!(f, "the build did not settle in {passes} passes")
            }
            Self::BuildNotSettled { passes, nets } => write!(
                f,
                "the build did not settle in {passes} passes: {} still changing",
                nets.join(", ")
            ),
        }
    }
}

// ============================================================
// Collector
// ============================================================

/// Finding collector with set semantics: a finding is *standing*, not an
/// event, so reporting one twice records it once.
///
/// **Reporting does not log.** Net resolution runs on every drive — hundreds
/// of times a second on a live machine — and re-reports every finding that
/// still holds each pass, so logging inside `report` turned standing facts
/// into a firehose: a measured 18 405 lines/s, in which one true and
/// permanent `FloatingSense` on an unconnected crystal pin appeared 139 262
/// times and starved the simulation it was describing. Logging therefore
/// belongs where novelty is known — the engine's cumulative merge and
/// `System::build` — and each of those reports a finding exactly once, when
/// it first appears.
#[derive(Debug, Default)]
pub struct Diagnostics {
    findings: Vec<Finding>,
}

impl Diagnostics {
    /// Empty collector.
    pub const fn new() -> Self {
        Self {
            findings: Vec::new(),
        }
    }

    /// Record a finding, returning `true` when it was not already present.
    ///
    /// Deliberately silent — see the type docs. Callers that know a finding
    /// is new (the engine's merge, `System::build`) log it via
    /// [`Diagnostics::log`].
    pub fn report(&mut self, finding: Finding) -> bool {
        if self.findings.contains(&finding) {
            return false;
        }
        self.findings.push(finding);
        true
    }

    /// Mirror one finding to `tracing`. Call only for a finding that has just
    /// appeared, never per resolution pass.
    pub fn log(finding: &Finding) {
        tracing::warn!(finding = ?finding, "board diagnostic finding");
    }

    /// Log every finding held, once. Used by the one-shot build path.
    pub fn log_all(&self) {
        for finding in &self.findings {
            Self::log(finding);
        }
    }

    /// All findings, in report order.
    pub fn findings(&self) -> &[Finding] {
        &self.findings
    }

    /// True when no findings were reported.
    pub fn is_empty(&self) -> bool {
        self.findings.is_empty()
    }

    /// Number of reported findings.
    pub fn len(&self) -> usize {
        self.findings.len()
    }

    /// True when an identical finding was reported (test assertion helper).
    pub fn contains(&self, finding: &Finding) -> bool {
        self.findings.iter().any(|f| f == finding)
    }
}

#[cfg(test)]
mod tests {
    use rstest::rstest;
    use vibes_behaviour::{behaviour, expect, Test};

    use super::*;

    /// A finding is standing, not an event: resolution re-reports it on every
    /// pass, so the collector must record it once and say so. The boolean is
    /// the novelty test the engine logs on — if `report` ever returns `true`
    /// for a repeat, the 18 405 lines/s firehose comes back.
    #[rstest]
    fn reporting_the_same_finding_twice_records_it_once() {
        let mut diags = Diagnostics::new();
        let floating = Finding::FloatingSense {
            net: "EC32MB.XTAL_XO".to_string(),
            kind: SenseKind::Digital,
        };

        assert!(diags.report(floating.clone()), "first report is new");
        for _ in 0..1_000 {
            assert!(
                !diags.report(floating.clone()),
                "a standing finding is never new again"
            );
        }

        assert_eq!(diags.len(), 1, "one standing fact, one record");
        assert_eq!(diags.findings(), &[floating]);
    }

    /// Distinct findings stay distinct — deduplication must key on the whole
    /// finding, not merely its kind, or a second floating net would be hidden
    /// by the first.
    #[rstest]
    fn deduplication_does_not_collapse_distinct_findings() {
        let mut diags = Diagnostics::new();
        let a = Finding::FloatingSense {
            net: "EC32MB.XTAL_XO".to_string(),
            kind: SenseKind::Digital,
        };
        let b = Finding::FloatingSense {
            net: "EC32MB.XTAL_XI".to_string(),
            kind: SenseKind::Digital,
        };

        assert!(diags.report(a.clone()));
        assert!(diags.report(b.clone()), "a different net is a new finding");
        assert!(!diags.report(a));
        assert_eq!(diags.len(), 2);
    }

    #[rstest]
    fn collector_records_in_order_and_answers_contains() {
        let mut diags = Diagnostics::new();
        assert!(diags.is_empty());

        let unsourced = Finding::PowerNetUnsourced {
            net: "AVDD".to_string(),
        };
        let floating = Finding::FloatingSense {
            net: "~RESET".to_string(),
            kind: SenseKind::Digital,
        };
        diags.report(unsourced.clone());
        diags.report(floating.clone());

        assert_eq!(diags.len(), 2);
        assert_eq!(diags.findings(), &[unsourced.clone(), floating.clone()]);
        assert!(diags.contains(&floating));
        assert!(!diags.contains(&Finding::PowerNetUnsourced {
            net: "DVDD".to_string()
        }));
    }

    #[rstest]
    fn findings_carry_asserted_fields() {
        let finding = Finding::UnconnectedRegistryPin {
            reference: "U1".to_string(),
            pin: "3".to_string(),
            direction: PinMismatchDirection::DeclaredButAbsent,
        };
        match finding {
            Finding::UnconnectedRegistryPin {
                reference,
                pin,
                direction,
            } => {
                assert_eq!(reference, "U1");
                assert_eq!(pin, "3");
                assert_eq!(direction, PinMismatchDirection::DeclaredButAbsent);
            }
            other => panic!("unexpected finding {other:?}"),
        }
    }

    // --------------------------------------------------------
    // Plain words
    // --------------------------------------------------------

    fn pin(reference: &str, pin: &str) -> PinRef {
        PinRef::new(reference, pin)
    }

    /// Every variant, and the variants a variant's text branches on, once:
    /// the line `embsim check` and `embsim run` print for it.
    #[rstest]
    #[case::contention(
        Finding::Contention {
            net: "EC32.P2_IO0".into(),
            drivers: vec![pin("EC32.U1", "3"), pin("EC32.U2", "5")],
        },
        "contention on EC32.P2_IO0: EC32.U1.3, EC32.U2.5 fight to drive it"
    )]
    #[case::contention_ideal(
        Finding::Contention { net: "BENCH.3V3".into(), drivers: vec![] },
        "contention on BENCH.3V3: ideal sources fight to hold it"
    )]
    #[case::floating_digital(
        Finding::FloatingSense { net: "DS2.~RESET".into(), kind: SenseKind::Digital },
        "no source reaches DS2.~RESET, which a digital input reads"
    )]
    #[case::floating_analog(
        Finding::FloatingSense { net: "DS2.AIN0".into(), kind: SenseKind::Analog },
        "no source reaches DS2.AIN0, which an analog input reads"
    )]
    #[case::ambiguous_level(
        Finding::AmbiguousLevel { net: "DRV.A".into(), volts: 1.65 },
        "DRV.A sits at 1.65 V, neither a valid low nor a valid high"
    )]
    #[case::current_into_floating_node(
        Finding::CurrentIntoFloatingNode { net: "LC.SENSE".into(), pin: pin("LC.U4", "2") },
        "LC.U4.2 injects a current into LC.SENSE, which no voltage source reaches, so the \
         current goes nowhere"
    )]
    #[case::periodic_not_coupled(
        Finding::PeriodicNotCoupled {
            net: "EDGE.STEP_B".into(),
            capacitor: "EDGE.C7".into(),
            hz: 1000,
            reactance_ohms: 1591.5,
            far_ohms: 100.0,
        },
        "a 1000 Hz signal does not cross EDGE.C7 to EDGE.STEP_B: the capacitor's 1591.5 Ω at \
         that rate is not small against the 100 Ω at the far node"
    )]
    #[case::periodic_not_coupled_no_resistor(
        Finding::PeriodicNotCoupled {
            net: "EDGE.STEP_B".into(),
            capacitor: "EDGE.C7".into(),
            hz: 10,
            reactance_ohms: 159154.9,
            far_ohms: f64::INFINITY,
        },
        "a 10 Hz signal does not cross EDGE.C7 to EDGE.STEP_B: the capacitor's 159154.9 Ω at \
         that rate is not small against the far node, which no resistor touches"
    )]
    #[case::non_convergent(
        Finding::NonConvergent {
            cluster: "OSC.N1".into(),
            elements: vec!["OSC.Q1".into(), "OSC.Q2".into()],
            solves: 8,
        },
        "the cluster at OSC.N1 has no operating point: its elements (OSC.Q1, OSC.Q2) found no \
         consistent regions in 8 solves, so its nets float"
    )]
    #[case::power_net_unsourced(
        Finding::PowerNetUnsourced { net: "EC32.Common_VDD".into() },
        "power net EC32.Common_VDD has no source"
    )]
    #[case::classification_error(
        Finding::ClassificationError {
            reference: "U9".into(),
            part: "XYZ123".into(),
            message: "no model for the part".into(),
        },
        "U9 (XYZ123) cannot be classified: no model for the part"
    )]
    #[case::callback_panic(
        Finding::CallbackPanic { kind: CallbackKind::Wake, subscriber: "component 4".into() },
        "a wake callback for component 4 panicked on the engine thread: the engine carries on, \
         but that part's own state is suspect"
    )]
    #[case::virtual_clock_uninitialized(
        Finding::VirtualClockUninitialized { context: "schedule_at".into() },
        "schedule_at needed the virtual clock before it was started, and the request was dropped"
    )]
    #[case::drive_seq_gap(
        Finding::DriveSeqGap { seq: 42 },
        "drive 42 never arrived (the thread that sent it died), and the engine skipped it"
    )]
    #[case::quiescence_timeout(
        Finding::QuiescenceTimeout { actors: vec!["cog 0".into(), "uart pump".into()] },
        "the engine advanced without waiting for cog 0, uart pump: this run is not reproducible"
    )]
    #[case::quiescence_timeout_unnamed(
        Finding::QuiescenceTimeout { actors: vec![] },
        "the engine advanced without waiting for a part: this run is not reproducible"
    )]
    #[case::declared_but_absent(
        Finding::UnconnectedRegistryPin {
            reference: "U1".into(),
            pin: "3".into(),
            direction: PinMismatchDirection::DeclaredButAbsent,
        },
        "U1's model declares pin 3, which the netlist does not have"
    )]
    #[case::present_but_undeclared(
        Finding::UnconnectedRegistryPin {
            reference: "U1".into(),
            pin: "9".into(),
            direction: PinMismatchDirection::PresentButUndeclared,
        },
        "the netlist gives U1 a pin 9 its model does not declare"
    )]
    #[case::rail_down_input(
        Finding::RailDown {
            part: "EC32.U5".into(),
            pin: "OUT".into(),
            reason: RailDownReason::InputUnsourced { pin: "VIN".into() },
        },
        "rail EC32.U5.OUT is down at build: its supply pin EC32.U5.VIN has no source"
    )]
    #[case::rail_down_reference(
        Finding::RailDown {
            part: "ISO.U3".into(),
            pin: "VOUT".into(),
            reason: RailDownReason::ReferenceUnheld { pin: "GND2".into() },
        },
        "rail ISO.U3.VOUT is down at build: its reference ISO.U3.GND2 is on a net no source \
         reaches"
    )]
    #[case::rail_down_held(
        Finding::RailDown {
            part: "EC32.U5".into(),
            pin: "OUT".into(),
            reason: RailDownReason::HeldDown,
        },
        "rail EC32.U5.OUT is down at build: the part holds it off (its input below threshold, \
         its enable off, or its soft-start not done)"
    )]
    #[case::unreferenced_domain(
        Finding::UnreferencedDomain {
            part: "ISO.U2".into(),
            pin: "VOA".into(),
            reference: "GND2".into(),
        },
        "ISO.U2.VOA is live, but its reference ISO.U2.GND2 is on a net no source reaches"
    )]
    #[case::undecoupled_power_pin(
        Finding::UndecoupledPowerPin {
            part: "DS2.U1".into(),
            pin: "DVDD".into(),
            reference: "DGND".into(),
        },
        "supply pin DS2.U1.DVDD has no capacitor to its reference DS2.U1.DGND"
    )]
    #[case::open_drain_without_pull_up(
        Finding::OpenDrainWithoutPullUp {
            part: "DS2.U3".into(),
            pin: "RESET".into(),
            net: "DS2.~RESET".into(),
        },
        "open-drain pin DS2.U3.RESET has no pull-up on DS2.~RESET: released, it leaves the net \
         floating"
    )]
    #[case::mechanical_one_driver(
        Finding::MechanicalOnDrivenNet {
            part: "EC32.H1".into(),
            net: "EC32.P2_IO7".into(),
            drivers: vec![pin("EC32.U1", "12")],
        },
        "mechanical pad EC32.H1 is on EC32.P2_IO7, which EC32.U1.12 drives"
    )]
    #[case::mechanical_drivers(
        Finding::MechanicalOnDrivenNet {
            part: "EC32.H1".into(),
            net: "EC32.P2_IO7".into(),
            drivers: vec![pin("EC32.U1", "12"), pin("EC32.U7", "1")],
        },
        "mechanical pad EC32.H1 is on EC32.P2_IO7, which EC32.U1.12, EC32.U7.1 drive"
    )]
    #[case::build_not_settled(
        Finding::BuildNotSettled { passes: 64, nets: vec!["OSC.A".into(), "OSC.B".into()] },
        "the build did not settle in 64 passes: OSC.A, OSC.B still changing"
    )]
    #[case::build_not_settled_unnamed(
        Finding::BuildNotSettled { passes: 64, nets: vec![] },
        "the build did not settle in 64 passes"
    )]
    #[case::pin_above_recommended(
        Finding::PinAboveRecommended {
            part: "EDGE.U25".into(),
            pin: "16".into(),
            volts: 5.0,
            min: 3.0,
            max: 3.6,
            absolute_max: Some(6.0),
            note: "its open-input bias is not characterised above 3.6 V".into(),
        },
        "pin EDGE.U25.16 is at 5 V, above its recommended 3 V to 3.6 V, within its 6 V absolute \
         maximum: its open-input bias is not characterised above 3.6 V"
    )]
    #[case::pin_beyond_absolute_max(
        Finding::PinAboveRecommended {
            part: "EDGE.U25".into(),
            pin: "16".into(),
            volts: 6.5,
            min: 3.0,
            max: 3.6,
            absolute_max: Some(6.0),
            note: String::new(),
        },
        "pin EDGE.U25.16 is at 6.5 V, above its recommended 3 V to 3.6 V, beyond its 6 V absolute \
         maximum"
    )]
    #[case::pin_above_recommended_bare(
        Finding::PinAboveRecommended {
            part: "B.U1".into(),
            pin: "VCC".into(),
            volts: 3.7,
            min: 3.0,
            max: 3.6,
            absolute_max: None,
            note: String::new(),
        },
        "pin B.U1.VCC is at 3.7 V, above its recommended 3 V to 3.6 V"
    )]
    fn a_finding_prints_in_plain_words(#[case] finding: Finding, #[case] line: &str) {
        behaviour!(Test {
            id: "diagnostics.finding-in-plain-words",
            covers: Some("board/src/diagnostics.rs#Finding"),
            given: "one finding of each kind, and each variation its text depends on",
        });
        expect!(
            "one-line-of-names",
            "the finding prints as one line naming the net, the pins as Reference.Pin or the \
             part it is about, and what is wrong with it",
            "the line is what embsim check and embsim run print to a terminal or a CI log"
        );
        expect!(
            "same-every-time",
            "printing the same finding twice gives the same line, its names in the order the \
             finding holds them",
            "two runs of a project print the same report"
        );
        let text = finding.to_string();
        assert_eq!(text, line);
        assert!(!text.contains('\n'), "{text}");
        assert_eq!(finding.to_string(), text);
    }
}
