//! The host's Chrome on the board.
//!
//! A web app that talks to a board over Web Serial — a machine's control
//! app, a flasher, a dashboard — holds timeouts against the board in its
//! own clock. When the board runs at a hundredth of real time, every one of
//! them fires a hundred times too early, and a run measures the host's
//! speed, not the machine's. This crate puts the browser *inside* the
//! simulation: the host's own Chrome, every page's clock and every
//! dedicated worker's metered by the board's over the Chrome DevTools
//! Protocol, its Web Serial port the far end of a host's serial line.
//!
//! # The model
//!
//! [`CdpNode`] is a board [`Component`](embsim_board::Component) with a
//! host's serial pins — `TX`, `RX`, and the `VIO` and `GND` of the rail its
//! line signals at ([`embsim_board::HostRailLine`], the line `host-serial`
//! has) — named from the host's side. On the far side of those pins is
//! Chrome, launched or attached to at the node's first slice:
//!
//! - **Held from birth.** Every page and every dedicated worker attaches
//!   paused (`Target.setAutoAttach` with `waitForDebuggerOnStart`). A page's
//!   virtual time is paused, the node's binding and its Web Serial shim are
//!   installed before its first script, and its workers attach the same
//!   way; a dedicated worker follows the page's clock (`advance` with a
//!   budget that never runs out). Service workers and shared workers run
//!   unmetered.
//! - **Metered by slices.** Every quantum of board time the node is woken
//!   on the engine's thread and grants each page what it is owed — the
//!   board's time since the page was first booked, less what the page's own
//!   clock says it lived — and waits for the budget to expire. JavaScript
//!   runs in no virtual time; timers, `Date.now()`, `performance.now()` and
//!   a WASM module's imported clock advance only in grants. A page whose
//!   clock got ahead (Chrome moves it outside a budget at a worker's birth
//!   and at storage calls) is paid back by skipping its grants.
//! - **The port is the node's.** The shim ([`SHIM`]) replaces
//!   `navigator.serial` with a port whose far end is this line, by Chrome's
//!   own rules: the messages, a `close()` refused while a stream is
//!   locked, streams released a slice after their flush, a disconnect that
//!   errors both streams and a re-insert that hands out a new `SerialPort`.
//!   Its bytes and its requests cross the same DevTools connection that
//!   meters the clock, through a binding the page calls and one
//!   `Runtime.evaluate` a slice, so they land at the board's instants.
//! - **The drain barrier.** A dedicated worker that owns the port's
//!   transferred stream is told the board's bytes by a message, which
//!   Chrome's clock does not wait for. So after handing bytes over, the
//!   node waits until the consumer has called `read()` again — a probe in
//!   each realm reports read calls — before the next grant.
//!
//! # In a project
//!
//! [`catalog::register`] adds the `chrome-cdp` bench component kind to a
//! set, and the `embsim` command's set has it (`PROJECTS.md` §5).
//!
//! # What this is not
//!
//! Chrome is host I/O. What a page sends, and in which slice, is the
//! page's, and Chrome's own scheduling decides some of it, so a run with a
//! `CdpNode` is excluded from the stepped-mode determinism guarantee as a
//! run with a host PTY is (`DETERMINISM.md`). Animation frames, resize and
//! intersection observers barely run under virtual time; the OS's serial
//! driver, Chrome's real Web Serial and a USB adapter are not in the byte
//! path. A fresh browser context per scenario is the harness's job.

#![forbid(unsafe_op_in_unsafe_fn)]

mod base64;
mod browser;
pub mod catalog;
mod chrome;
pub mod devtools;
mod node;

pub use browser::Browse;
pub use chrome::{find_chrome, free_port, LaunchSpec};
pub use node::{
    CdpNode, LinkControl, LinkOp, NodeStats, Settings, UsbIds, DEFAULT_QUANTUM,
    DEFAULT_STUCK_AFTER, DRAIN_BOUND, DRAIN_STRIKES, MAX_QUANTUM,
};

/// The Web Serial shim each page's documents get, before their own scripts;
/// `__EMBSIM_CONFIG__` stands for its configuration (the port's USB ids).
pub const SHIM: &str = include_str!("shim.js");

/// The read probe each dedicated worker gets.
pub const PROBE: &str = include_str!("probe.js");
