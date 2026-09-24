//! embsim-models — Reusable, generic hardware component models.
//!
//! Generic device/IC-level models and shared primitives with NO knowledge of
//! any specific machine, MCU driver, or HAL:
//! - [`ads122u04`] — TI ADS122U04 UART ADC IC protocol model
//! - [`ads122u04_component`] — that model as a live `embsim-board` component
//!   (pin facade, power/reset gate, stream pump)
//! - [`isolation`] — the parts between an MCU pin and the machine: the TI
//!   ISO67xx digital isolator family, a dual optocoupler, a constant-current
//!   LED driver, and a small NPN switch
//! - [`limit_switch`] — position-threshold limit switch
//! - [`sd_card`] — an SD card, device side, in SPI mode: a byte-level protocol
//!   model that knows nothing about who is clocking it
//! - [`sd_card_component`] — that model as a live `embsim-board` component
//!   (microSD or by-function pin facade, active-low CS, DO released when
//!   deselected)
//! - [`spi_flash`] — a serial NOR flash, bit-level and bus-agnostic: anything
//!   that can produce a chip select, a clock edge and a data bit can talk to
//!   it, whether bit-banged or peripheral-clocked
//! - [`spi_flash_component`] — that model as a live `embsim-board` component
//!   (SOIC-8 pin facade, active-low ~CS, DO driven from the sense callbacks)
//! - [`edge`] — edge-detection primitive shared by threshold models
//! - [`fat16`] — a FAT16 card image built in memory, so a guest filesystem has
//!   something to mount on [`sd_card`]
//! - [`logic_gate`] — single-input CMOS gates (the NXP 74LVC2G04 dual
//!   inverter, the TI SN74LVC1G14 Schmitt inverter): thresholds with
//!   hysteresis, the datasheet output impedance, `t_pd` as a scheduled
//!   instant, and a rate mode that relays a routed clock
//! - [`oscillator`] — a clock oscillator (the EPSON TG2520SMN TCXO) whose
//!   output is a rate published once at its start-up instant
//! - [`psram`] — the AP Memory APS6404L QSPI PSRAM in its SPI mode, on the
//!   same shift engine as the flash
//! - [`spi_shift`] — the byte-wide shift register every SPI-mode device
//!   model here is built on
//! - [`machine`] — the **physical world** as harness-attached `embsim-board`
//!   components: a step/direction motor drive, a quadrature encoder, and an
//!   end-of-travel switch, each with a real pin facade
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
pub mod fat16;
pub mod isolation;
pub mod limit_switch;
pub mod logic_gate;
pub mod machine;
pub mod oscillator;
pub mod psram;
pub mod sd_card;
pub mod sd_card_component;
pub mod spi_flash;
pub mod spi_flash_component;
pub mod spi_shift;
