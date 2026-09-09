//! embsim-core — Core infrastructure for embedded MCU simulation.
//!
//! Provides MCU-agnostic primitives shared by all platform crates:
//! - `virtual_clock` — one-counter virtual time (optional wall pacing)
//! - `profile` — named run backends (live / sil / fullsim)
//! - `serial_pty` — PTY pair creation for host ↔ firmware serial communication
//! - `event` — multi-subscriber callback primitive for model/peripheral events

pub mod event;
pub mod profile;
pub mod serial_pty;
pub mod virtual_clock;
