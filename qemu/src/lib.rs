//! A host computer on the board.
//!
//! Firmware talks to a host — a laptop running the control app, a Raspberry
//! Pi, a test rig. On a bench that host has its own clock, and the
//! simulation cannot slow it down: when the board runs at a hundredth of real
//! time, every timeout the host holds against the board fires a hundred times
//! too early. This crate puts the host *inside* the simulation as a virtual
//! machine whose clock advances only when the board's does.
//!
//! # The model
//!
//! [`QemuNode`] is a board [`Component`](embsim_board::Component) with a
//! host's serial pins — `TX`, `RX`, and the `VIO` and `GND` of the rail its
//! line signals at ([`embsim_board::HostRailLine`], the line `host-serial`
//! has) — named from the computer's side. On the far side of those pins
//! sits a [`Guest`]: something that can be frozen and thawed and owns a
//! serial port. [`QemuVm`] is the production guest — a process of the
//! host's own system QEMU (`qemu-system-aarch64` under HVF on a Mac,
//! `qemu-system-x86_64` under KVM on Linux), paused and resumed over QMP, its
//! serial port an emulated USB adapter whose bytes cross a unix socket.
//! [`ChromeGuest`] launches the image `guest/chrome/build.sh` builds: a
//! headless Chromium a harness drives over DevTools, its real Web Serial on
//! that adapter.
//!
//! Time is metered in slices, through the node's own wakes. Every quantum of
//! virtual time the node is woken on the engine's thread, lets the guest run
//! for that long in host time (a hardware-virtualised guest runs at 1×),
//! freezes it again over QMP, and asks to be woken a quantum on. The engine
//! cannot advance the board while the wake runs, and the guest is frozen
//! whenever it does, so the guest's clock advances only while the board's
//! does and lags it by at most a quantum. A closed loop carries the guest's
//! owed time forward so the two clocks stay together over a run, not just
//! per slice, and a guest that can read its own clock books every slice
//! from it.
//!
//! Bytes the board sends while the guest is frozen wait in a queue and are
//! handed over when the guest next runs; bytes the guest sends during its
//! slice enter the net at the slice's virtual instant, in order, at the
//! line's baud, as levels through the same [`SerialLevelBridge`] every UART
//! in embsim uses. The wire sees the host's traffic quantised to the
//! quantum — the resolution the host itself has once it is a computer rather
//! than a peripheral.
//!
//! # In a project
//!
//! [`catalog::register`] adds two bench component kinds to a set, and the
//! `embsim` command's set has them: `qemu-vm`, any image on any machine the
//! host's QEMU runs, and `chrome-vm`, the Chrome guest with its DevTools
//! port forwarded to the host (`PROJECTS.md` §5).
//!
//! # What this is not
//!
//! A guest is host I/O. Its scheduling, its network stack and its browser are
//! real programs on a real OS, and no two runs of it are bit-identical, so a
//! run with a `QemuNode` in it is excluded from the stepped-mode determinism
//! guarantee the same way a run with a host PTY is (`DETERMINISM.md`). The
//! instants the node acts at are the board's — every slice is at a multiple
//! of the quantum after the start, and every byte enters the line at one —
//! but what the guest sends, and in which slice, is the guest's. What the
//! node guarantees is bounded skew between the two clocks — enough for a
//! host's timeouts to mean what they say.
//!
//! [`SerialLevelBridge`]: embsim_board::SerialLevelBridge

#![forbid(unsafe_op_in_unsafe_fn)]

pub mod catalog;
mod chrome;
mod guest;
mod node;
pub mod qmp;
mod vm;

pub use chrome::{
    default_image, free_port, Accel, ChromeError, ChromeGuest, ChromeVm, DevTools,
    DEFAULT_WARMUP_TIMEOUT, GUEST_DEVTOOLS_PORT,
};
pub use guest::Guest;
pub use node::{Boot, LinkControl, NodeStats, QemuNode, DEFAULT_QUANTUM, MAX_QUANTUM};
pub use qmp::{Qmp, QmpError, RunState, MAX_RETAINED_EVENTS};
pub use vm::{
    QemuSpec, QemuVm, SerialDevice, SpawnError, AGENT_PORT_NAME, DEFAULT_STARTUP_TIMEOUT,
};
