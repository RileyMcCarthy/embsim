//! A computer on the board.
//!
//! Firmware talks to a host — a laptop running the control app, a Raspberry
//! Pi, a test rig. On a real bench that host has its own clock, and the
//! simulation cannot slow it down: when the board runs at a hundredth of real
//! time, every timeout the host holds against the board fires a hundred times
//! too early. This crate puts the host *inside* the simulation as a virtual
//! machine whose clock only advances when the board's does.
//!
//! # The model
//!
//! [`QemuNode`] is a board [`Component`](embsim_board::Component) with two
//! pins, `TX` and `RX`, named from the computer's point of view. On the far
//! side of those pins sits a [`Guest`]: something that can be frozen and
//! thawed and owns a serial port. [`QemuVm`] is the production guest — a QEMU
//! process paused and resumed over QMP, its serial port an emulated USB
//! adapter whose bytes cross a unix socket.
//!
//! Time is metered in slices. The node registers as a virtual-clock actor and
//! parks until the next slice boundary; the engine advances the board that
//! far; the node wakes, lets the guest run for the same span of *wall* time
//! (a hardware-virtualised guest runs at 1×), freezes it again and parks.
//! Because the engine will not advance while a registered actor is awake
//! ([`embsim_core::virtual_clock`]'s quiescence barrier), the guest can never
//! get ahead of the board, and because the guest is frozen whenever the node
//! is parked, the board can never get ahead of the guest by more than one
//! slice. A closed loop carries the guest's owed time forward so the two
//! clocks stay within a millisecond over a run, not just per slice.
//!
//! Bytes the board sends while the guest is frozen wait in a queue and are
//! handed over when the guest next runs; bytes the guest sends during its
//! slice enter the net at the slice's virtual instant, in order, at the
//! line's baud. The wire sees the host's traffic quantised to the slice —
//! which is the resolution the host itself has once it is a computer rather
//! than a peripheral.
//!
//! # What this is not
//!
//! A guest is host I/O. Its scheduling, its network stack and its browser are
//! real programs on a real OS, and no two runs of it are bit-identical, so a
//! run with a `QemuNode` in it is excluded from the stepped-mode determinism
//! guarantee the same way a run with a host PTY is (see `DETERMINISM.md`).
//! What the node guarantees is bounded skew between the two clocks — enough
//! for a host's timeouts to mean what they say.

#![forbid(unsafe_op_in_unsafe_fn)]

mod chrome;
mod guest;
mod node;
pub mod qmp;
mod vm;

pub use chrome::{
    Accel, ChromeError, ChromeGuest, ChromeVm, DevTools, DEFAULT_WARMUP_TIMEOUT,
    GUEST_DEVTOOLS_PORT,
};
pub use guest::Guest;
pub use node::{NodeStats, QemuNode, DEFAULT_SLICE, MAX_SLICE};
pub use qmp::{Qmp, QmpError, RunState, MAX_RETAINED_EVENTS};
pub use vm::{
    QemuSpec, QemuVm, SerialDevice, SpawnError, AGENT_PORT_NAME, DEFAULT_STARTUP_TIMEOUT,
};
