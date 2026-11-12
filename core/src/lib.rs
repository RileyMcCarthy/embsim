//! embsim-core — Core infrastructure for embedded MCU simulation.
//!
//! Provides MCU-agnostic primitives shared by the board engine and its nodes:
//! - `virtual_clock` — one-counter virtual time (optional wall pacing)
//! - `serial_pty` — PTY pair creation for a host's end of a serial line
//! - `event` — multi-subscriber callback primitive for model/peripheral events

pub mod event;
pub mod serial_pty;
pub mod virtual_clock;
