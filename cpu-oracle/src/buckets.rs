//! Which instructions a one-instruction probe can actually observe.
//!
//! The single most useful idea in a silicon-vs-ISS harness is not the diffing;
//! it is admitting that **most instructions cannot be checked one at a time**.
//! A trampoline that patches an encoding, runs it, and dumps the destination
//! register answers beautifully for `ADD` and not at all for `WAITX`, whose
//! answer is a number of clocks; nor for `DRVH`, whose answer is a level on a
//! wire; nor for a CORDIC op, whose answer arrives in a different register
//! several instructions later.
//!
//! Getting this classification wrong is not a missing test — it is a *hang*.
//! An op whose result never reaches the mailbox leaves the worker waiting, and
//! a capture that treats that as a transient error rather than a wedged target
//! silently degrades: every later case fails or answers out of stale state.
//! (Observed: a P2 sweep that classified the pixel-blend and scale ops as
//! one-instruction-safe produced 325 "skips" whose tail was plain ALU ops,
//! because the worker had been dead for thousands of cases.)
//!
//! So each ISA maps its opcodes onto [`Observability`], and the buckets that
//! are not [`Observability::OneInstruction`] each need their own small program
//! — a timing program, an I/O program — whose console output is the golden.

use core::fmt;

/// How an instruction's effect can be observed from outside.
///
/// The variants are deliberately about the *shape of the observation*, not
/// about any one architecture: every CPU has instructions whose result is a
/// register, a duration, a pin, a coprocessor queue, or a change of control
/// flow, and each of those needs a different harness.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Observability {
    /// The whole effect lands in the destination (and flags) by the next
    /// instruction, so a one-instruction trampoline can capture it.
    OneInstruction,
    /// Modifies the instruction that follows it, so it belongs in the
    /// trampoline's prefix slot rather than its subject slot.
    Prefix,
    /// The answer is a number of cycles.
    Timing,
    /// The answer is a pin level, or depends on one.
    Io,
    /// The answer moves through a FIFO, streamer or DMA engine.
    Streaming,
    /// The answer lands in a coprocessor queue (CORDIC, FPU) and is collected
    /// by a later instruction.
    Coprocessor,
    /// Changes the program counter, so it would derail the trampoline.
    ControlFlow,
    /// Reads or writes the hardware call stack.
    Stack,
    /// Interrupts, self-modifying code, mode changes.
    System,
    /// Locks, mailboxes, anything whose meaning needs a second core.
    Concurrency,
}

impl Observability {
    /// Whether a one-instruction trampoline can capture this.
    ///
    /// Everything else needs a dedicated program; [`Self::needs_program`] says
    /// so in words, which is what a coverage report should print.
    pub fn one_instruction(self) -> bool {
        self == Self::OneInstruction
    }

    /// Why this cannot go through the one-instruction probe, or `None` if it
    /// can.
    pub fn needs_program(self) -> Option<&'static str> {
        Some(match self {
            Self::OneInstruction => return None,
            Self::Prefix => "prefix — modifies the next instruction",
            Self::Timing => "timing — the result is a cycle count",
            Self::Io => "i/o — the result is a pin level",
            Self::Streaming => "streaming — the result moves through a FIFO",
            Self::Coprocessor => "coprocessor — the result lands in a queue, not D",
            Self::ControlFlow => "control flow — would derail the trampoline",
            Self::Stack => "stack",
            Self::System => "system / interrupt / self-modifying",
            Self::Concurrency => "concurrency — needs a second core",
        })
    }

    pub fn name(self) -> &'static str {
        match self {
            Self::OneInstruction => "one-instruction",
            Self::Prefix => "prefix",
            Self::Timing => "timing",
            Self::Io => "io",
            Self::Streaming => "streaming",
            Self::Coprocessor => "coprocessor",
            Self::ControlFlow => "control-flow",
            Self::Stack => "stack",
            Self::System => "system",
            Self::Concurrency => "concurrency",
        }
    }
}

impl fmt::Display for Observability {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_the_one_instruction_bucket_is_probe_safe() {
        assert!(Observability::OneInstruction.one_instruction());
        for other in [
            Observability::Prefix,
            Observability::Timing,
            Observability::Io,
            Observability::Streaming,
            Observability::Coprocessor,
            Observability::ControlFlow,
            Observability::Stack,
            Observability::System,
            Observability::Concurrency,
        ] {
            assert!(!other.one_instruction(), "{other} must not be probe-safe");
        }
    }

    #[test]
    fn every_excluded_bucket_can_say_why() {
        assert_eq!(Observability::OneInstruction.needs_program(), None);
        for other in [
            Observability::Prefix,
            Observability::Timing,
            Observability::Io,
            Observability::Coprocessor,
        ] {
            let why = other.needs_program().expect("a reason");
            assert!(!why.is_empty(), "{other} needs a printable reason");
        }
    }
}
