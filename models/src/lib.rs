//! embsim-models — Reusable, generic hardware component models.
//!
//! Generic device/IC-level models and shared primitives with NO knowledge of
//! any specific machine, MCU driver, or HAL:
//! - [`ads122u04`] — TI ADS122U04 UART ADC IC protocol model
//! - [`ads122u04_component`] — that model as a live `embsim-board` component
//!   (pin facade, power/reset gate, stream pump)
//! - [`limit_switch`] — position-threshold limit switch
//! - [`edge`] — edge-detection primitive shared by threshold models
//!
//! Models communicate through [`embsim_core::event::Observers`]: each accepts
//! input via setter functions and emits output to any number of subscribers
//! when state changes. The project wiring layer connects these chains together.
//!
//! Project-specific physics (e.g. a tensile tester's gantry/sample/strain
//! gauge) lives in the consumer's own models crate, wired to these primitives.

pub mod ads122u04;
pub mod ads122u04_component;
pub mod edge;
pub mod limit_switch;
