//! Structured findings on a diagnostics bus, mirrored to `tracing`.
//!
//! Findings are the engine's way of reporting electrical/topological problems
//! without panicking: tests assert that a specific [`Finding`] fired, trace
//! tooling can consume the same bus later. The [`Diagnostics`] collector is
//! Vec-based; every reported finding is also emitted as a `tracing` warning.

use crate::net::{PinRef, Volts};

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
    /// A stream byte delivery.
    StreamByte,
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

/// One structured diagnostic finding. A finding, never a panic.
#[derive(Debug, Clone, PartialEq)]
pub enum Finding {
    /// ≥ 2 push-pull sources fighting on one net (directly or through
    /// collapsed low-value series resistance).
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
    /// A solved voltage inside a digital sense's `V_IL`/`V_IH` dead band.
    AmbiguousLevel {
        /// Net name.
        net: String,
        /// The solved node voltage that fell inside the dead band.
        volts: Volts,
    },
    /// A power net with no `PowerOut` source anywhere (board or harness);
    /// presents as down (0 V into cluster solves).
    PowerNetUnsourced {
        /// Net name.
        net: String,
    },
    /// A serial route with two `Producer`s facing each other (the crossed
    /// TX/RX harness).
    StreamMismatch {
        /// Name of the net carrying the invalid route.
        net: String,
        /// The producer pins facing each other.
        producers: Vec<PinRef>,
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
    /// A paced stream route's in-flight queue overflowed: the producer
    /// sustained writes above its declared baud for longer than the queue
    /// absorbs (a producer-vs-declared-baud mismatch — exactly the class
    /// of disagreement the framework exists to surface). Overflow bytes
    /// are shed; the trace carries the counts.
    StreamOverrun {
        /// The producer pin whose route overflowed.
        producer: PinRef,
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
}

// ============================================================
// Collector
// ============================================================

/// Vec-based finding collector; every report is mirrored to `tracing::warn!`.
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

    /// Record a finding and mirror it to `tracing`.
    pub fn report(&mut self, finding: Finding) {
        tracing::warn!(finding = ?finding, "board diagnostic finding");
        self.findings.push(finding);
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

    use super::*;

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
}
