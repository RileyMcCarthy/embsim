//! Firmware memory introspection — extract C enums, struct layouts, and variable
//! addresses from DWARF debug info embedded in firmware object files.
//!
//! # Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────┐
//! │  libfirmware.a (built with -g)          │
//! │  ├── app_control.o  ─── DWARF ───┐      │
//! │  ├── dev_stepper.o  ─── DWARF ───┤      │
//! │  └── ...                         │      │
//! └──────────────────────────────────┼──────┘
//!                                    ▼
//! ┌─────────────────────────────────────────┐
//! │  FirmwareInfo (parsed at startup)       │
//! │  ├── enums:   HashMap<name, variants>   │
//! │  ├── structs: HashMap<name, fields>     │
//! │  └── variables: HashMap<name, type>     │
//! └──────────────────┬──────────────────────┘
//!                    │
//!        ┌───────────┴───────────┐
//!        ▼                       ▼
//!   Enum lookup             Variable read
//!   fw.enum_channel(        fw.read_field::<T>(
//!     "HAL_GPIO_SERVO_ENA"    "dev_cogManager_data",
//!   ) → 0usize               "channels[0].state"
//!                           ) → i32
//! ```
//!
//! # Usage
//!
//! ```rust,ignore
//! use embsim_memory_inspect::FirmwareInfo;
//!
//! // Parse all .o files from the firmware archive
//! let fw = FirmwareInfo::from_archive("path/to/libfirmware.a").unwrap();
//!
//! // Look up enum channel index by variant name (panics if not found)
//! let servo_ena = fw.enum_channel("HAL_GPIO_SERVO_ENA");
//!
//! // Look up enum variant count
//! let gpio_count = fw.channel_count("HAL_GPIO_channel_E");
//!
//! // Get all variants of an enum type
//! let variants = fw.enum_variants("HAL_GPIO_channel_E");
//!
//! // After firmware is loaded (linked into binary), read variables:
//! // (resolve symbol address from the running binary's symbol table)
//! let state: i32 = unsafe { fw.read_field("dev_cogManager_data", "channels[0].state") }.unwrap();
//! ```
//!
//! A third path reads the **initialized values** of global data symbols
//! straight from the archive — no running binary required. This is how a
//! consumer's HAL wiring/config tables (pin maps, bauds) are read for the
//! board engine's pin facade (see [`ArchiveValueReader`]):
//!
//! ```rust,ignore
//! use embsim_memory_inspect::ArchiveValueReader;
//!
//! let values = ArchiveValueReader::from_archive("path/to/libfirmware.a".as_ref()).unwrap();
//! let cfg = values.read_value(&fw, "HAL_serial_channelConfig").unwrap();
//! let rx = cfg.index(0).unwrap().field("rx").unwrap().as_i64(); // e.g. Some(0)
//! ```
//!
//! The [`hal_tables`] module layers consumer-shaped decoding on top of that
//! path: the four HAL wiring tables (serial, GPIO, encoder, pulse-out)
//! decode into plain Rust structs, with the table symbol names as
//! parameters (defaults document the reference consumer's names).

pub mod hal_tables;

mod archive_values;
mod dwarf_parser;
mod runtime;
mod types;

pub use archive_values::{ArchiveValueReader, Value, ValueReadError};
pub use runtime::SymbolResolver;
pub use types::{
    EnumInfo, FieldInfo, FirmwareInfo, MemInspectError, ParseOptions, StructInfo, TypeInfo,
    VariableInfo,
};
