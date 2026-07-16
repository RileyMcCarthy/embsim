//! HAL config-table decoding — the consumer-shaped layer above
//! [`ArchiveValueReader`]: read a firmware's HAL wiring/config tables from
//! its static library and hand back plain Rust structs.
//!
//! This is the board-engine pin facade's read path (`BOARD_ENGINE.md`,
//! "The MCU as a component", point 3): the consumer's Phase-0 data-only HAL
//! config tables are decoded from `libfirmware.a` so an MCU component's
//! pinout and baud derive from the same data that runs on hardware — the
//! emulator stops inventing its own defaults.
//!
//! # Symbol and field names are the consumer's
//!
//! The table symbol names and the field names inside each entry are defined
//! by the **consumer's** HAL config structs, not by this crate. The
//! `DEFAULT_*_SYMBOL` constants document the reference consumer's (MaD's)
//! symbol names; every read function takes the symbol as a parameter so
//! another firmware can point the helpers at its own tables. The field names
//! each decoder expects (also the reference consumer's):
//!
//! | Table | Fields read | Notes |
//! |---|---|---|
//! | serial | `rx`, `tx`, `baud` | `rx`/`tx` decode as [`Value::Enum`] of the consumer's pin enum (`HW_pin_E` in MaD); the numeric enumerator value is the physical pin index |
//! | GPIO | `pin`, `activeLow` | |
//! | encoder | `pinA`, `pinB` | other fields (presets, limits) are ignored |
//! | pulse-out | `pin` | other fields (step timing) are ignored |
//!
//! Extra fields in a table entry are ignored, so consumers may carry
//! additional per-channel configuration without breaking the decode.
//!
//! # Usage
//! ```rust,ignore
//! use embsim_memory_inspect::{
//!     hal_tables, ArchiveValueReader, FirmwareInfo,
//! };
//! use std::path::Path;
//!
//! let path = Path::new("libfirmware.a");
//! let fw = FirmwareInfo::from_archive(path).unwrap();
//! let values = ArchiveValueReader::from_archive(path).unwrap();
//!
//! let serial =
//!     hal_tables::read_serial_table(&values, &fw, hal_tables::DEFAULT_SERIAL_TABLE_SYMBOL)
//!         .unwrap();
//! assert_eq!(serial[0].baud, 115_200);
//! ```

use crate::archive_values::{ArchiveValueReader, Value, ValueReadError};
use crate::types::FirmwareInfo;
use std::fmt;

// ============================================================
// Default symbol names (the reference consumer's)
// ============================================================

/// Default serial-table symbol: MaD's `HAL_serial_channelConfig[]`.
pub const DEFAULT_SERIAL_TABLE_SYMBOL: &str = "HAL_serial_channelConfig";

/// Default GPIO-table symbol: MaD's `HAL_GPIO_channelConfig[]`.
pub const DEFAULT_GPIO_TABLE_SYMBOL: &str = "HAL_GPIO_channelConfig";

/// Default encoder-table symbol: MaD's `HAL_encoder_config[]`.
pub const DEFAULT_ENCODER_TABLE_SYMBOL: &str = "HAL_encoder_config";

/// Default pulse-out-table symbol: MaD's `HAL_pulseOut_channelConfig[]`.
pub const DEFAULT_PULSE_OUT_TABLE_SYMBOL: &str = "HAL_pulseOut_channelConfig";

// ============================================================
// Decoded table entries
// ============================================================

/// One serial channel's wiring: physical RX/TX pins and configured baud.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SerialChannelConfig {
    /// Physical RX pin index (the numeric value of the consumer's pin enum).
    pub rx_pin: u32,
    /// Physical TX pin index.
    pub tx_pin: u32,
    /// Configured baud rate in bits per second.
    pub baud: u32,
}

/// One GPIO channel's wiring: physical pin and polarity.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GpioChannelConfig {
    /// Physical pin index.
    pub pin: u32,
    /// `true` when the channel's active state drives the pin low.
    pub active_low: bool,
}

/// One quadrature-encoder channel's wiring: the A/B phase pins.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EncoderConfig {
    /// Physical A-phase pin index.
    pub pin_a: u32,
    /// Physical B-phase pin index.
    pub pin_b: u32,
}

/// One pulse-output channel's wiring: the step pin.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PulseOutConfig {
    /// Physical pin index.
    pub pin: u32,
}

// ============================================================
// Errors
// ============================================================

/// Errors raised while decoding a HAL config table.
#[derive(Debug)]
pub enum HalTableError {
    /// The underlying archive value read failed (symbol missing, no DWARF,
    /// undecodable bytes — see [`ValueReadError`]).
    Read(ValueReadError),
    /// The symbol decoded, but its shape does not match the expected table
    /// layout (not an array, a missing/mistyped field, an out-of-range
    /// value).
    Shape {
        /// The table symbol being decoded.
        symbol: String,
        /// What did not match.
        detail: String,
    },
}

impl fmt::Display for HalTableError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            HalTableError::Read(e) => write!(f, "{e}"),
            HalTableError::Shape { symbol, detail } => {
                write!(f, "HAL table '{symbol}' has an unexpected shape: {detail}")
            }
        }
    }
}

impl std::error::Error for HalTableError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            HalTableError::Read(e) => Some(e),
            HalTableError::Shape { .. } => None,
        }
    }
}

impl From<ValueReadError> for HalTableError {
    fn from(e: ValueReadError) -> Self {
        HalTableError::Read(e)
    }
}

fn shape_err(symbol: &str, detail: impl Into<String>) -> HalTableError {
    HalTableError::Shape {
        symbol: symbol.to_string(),
        detail: detail.into(),
    }
}

// ============================================================
// Read functions
// ============================================================

/// Read and decode a serial wiring table (fields `rx`, `tx`, `baud`) from
/// the archive. `symbol` is the consumer's table name —
/// [`DEFAULT_SERIAL_TABLE_SYMBOL`] for the reference consumer.
pub fn read_serial_table(
    values: &ArchiveValueReader,
    fw: &FirmwareInfo,
    symbol: &str,
) -> Result<Vec<SerialChannelConfig>, HalTableError> {
    decode_serial_table(&values.read_value(fw, symbol)?, symbol)
}

/// Read and decode a GPIO wiring table (fields `pin`, `activeLow`).
/// `symbol` defaults to [`DEFAULT_GPIO_TABLE_SYMBOL`] by convention.
pub fn read_gpio_table(
    values: &ArchiveValueReader,
    fw: &FirmwareInfo,
    symbol: &str,
) -> Result<Vec<GpioChannelConfig>, HalTableError> {
    decode_gpio_table(&values.read_value(fw, symbol)?, symbol)
}

/// Read and decode an encoder wiring table (fields `pinA`, `pinB`).
/// `symbol` defaults to [`DEFAULT_ENCODER_TABLE_SYMBOL`] by convention.
pub fn read_encoder_table(
    values: &ArchiveValueReader,
    fw: &FirmwareInfo,
    symbol: &str,
) -> Result<Vec<EncoderConfig>, HalTableError> {
    decode_encoder_table(&values.read_value(fw, symbol)?, symbol)
}

/// Read and decode a pulse-out wiring table (field `pin`).
/// `symbol` defaults to [`DEFAULT_PULSE_OUT_TABLE_SYMBOL`] by convention.
pub fn read_pulse_out_table(
    values: &ArchiveValueReader,
    fw: &FirmwareInfo,
    symbol: &str,
) -> Result<Vec<PulseOutConfig>, HalTableError> {
    decode_pulse_out_table(&values.read_value(fw, symbol)?, symbol)
}

// ============================================================
// Decoding (from an already-read Value)
// ============================================================

fn decode_serial_table(
    table: &Value,
    symbol: &str,
) -> Result<Vec<SerialChannelConfig>, HalTableError> {
    entries(table, symbol)?
        .iter()
        .enumerate()
        .map(|(index, entry)| {
            Ok(SerialChannelConfig {
                rx_pin: field_u32(entry, symbol, index, "rx")?,
                tx_pin: field_u32(entry, symbol, index, "tx")?,
                baud: field_u32(entry, symbol, index, "baud")?,
            })
        })
        .collect()
}

fn decode_gpio_table(table: &Value, symbol: &str) -> Result<Vec<GpioChannelConfig>, HalTableError> {
    entries(table, symbol)?
        .iter()
        .enumerate()
        .map(|(index, entry)| {
            Ok(GpioChannelConfig {
                pin: field_u32(entry, symbol, index, "pin")?,
                active_low: field_bool(entry, symbol, index, "activeLow")?,
            })
        })
        .collect()
}

fn decode_encoder_table(table: &Value, symbol: &str) -> Result<Vec<EncoderConfig>, HalTableError> {
    entries(table, symbol)?
        .iter()
        .enumerate()
        .map(|(index, entry)| {
            Ok(EncoderConfig {
                pin_a: field_u32(entry, symbol, index, "pinA")?,
                pin_b: field_u32(entry, symbol, index, "pinB")?,
            })
        })
        .collect()
}

fn decode_pulse_out_table(
    table: &Value,
    symbol: &str,
) -> Result<Vec<PulseOutConfig>, HalTableError> {
    entries(table, symbol)?
        .iter()
        .enumerate()
        .map(|(index, entry)| {
            Ok(PulseOutConfig {
                pin: field_u32(entry, symbol, index, "pin")?,
            })
        })
        .collect()
}

/// The table's entries. A HAL config table is `const T name[COUNT]`, so it
/// must decode as an array (a 1-element table is still `Value::Array`).
fn entries<'v>(table: &'v Value, symbol: &str) -> Result<&'v [Value], HalTableError> {
    table
        .elements()
        .ok_or_else(|| shape_err(symbol, "table does not decode as an array"))
}

/// A named integer-shaped field of one table entry, as `u32`. Pin fields
/// decode as [`Value::Enum`] of the consumer's pin enum — the numeric
/// enumerator value is used.
fn field_u32(entry: &Value, symbol: &str, index: usize, field: &str) -> Result<u32, HalTableError> {
    let value = entry
        .field(field)
        .ok_or_else(|| shape_err(symbol, format!("entry [{index}] has no field '{field}'")))?;
    let raw = value.as_i64().ok_or_else(|| {
        shape_err(
            symbol,
            format!("entry [{index}].{field} is not integer-shaped: {value:?}"),
        )
    })?;
    u32::try_from(raw).map_err(|_| {
        shape_err(
            symbol,
            format!("entry [{index}].{field} = {raw} does not fit u32"),
        )
    })
}

/// A named bool-shaped field of one table entry.
fn field_bool(
    entry: &Value,
    symbol: &str,
    index: usize,
    field: &str,
) -> Result<bool, HalTableError> {
    let value = entry
        .field(field)
        .ok_or_else(|| shape_err(symbol, format!("entry [{index}] has no field '{field}'")))?;
    value.as_bool().ok_or_else(|| {
        shape_err(
            symbol,
            format!("entry [{index}].{field} is not bool-shaped: {value:?}"),
        )
    })
}

// ============================================================
// Tests
// ============================================================

#[cfg(test)]
mod tests {
    use super::*;

    /// A struct-shaped entry from (field, value) pairs.
    fn entry(fields: Vec<(&str, Value)>) -> Value {
        Value::Struct {
            type_name: "cfg_S".to_string(),
            fields: fields
                .into_iter()
                .map(|(name, value)| (name.to_string(), value))
                .collect(),
        }
    }

    /// A pin field as the consumer's enum shape.
    fn pin(value: i64) -> Value {
        Value::Enum {
            type_name: "HW_pin_E".to_string(),
            value,
        }
    }

    /// The MaD-shaped serial table decodes entry-by-entry, taking the pin
    /// enums' numeric values and ignoring extra fields (`type`, `LSB`).
    #[test]
    fn serial_table_decodes_pins_and_baud() {
        let table = Value::Array(vec![
            entry(vec![
                ("rx", pin(0)),
                ("tx", pin(2)),
                ("baud", Value::Int(115_200)),
                ("type", pin(0)),
                ("LSB", Value::Bool(false)),
            ]),
            entry(vec![
                ("rx", pin(53)),
                ("tx", pin(55)),
                ("baud", Value::Int(2_000_000)),
            ]),
        ]);
        let decoded = decode_serial_table(&table, "tbl").expect("decodes");
        assert_eq!(
            decoded,
            vec![
                SerialChannelConfig {
                    rx_pin: 0,
                    tx_pin: 2,
                    baud: 115_200
                },
                SerialChannelConfig {
                    rx_pin: 53,
                    tx_pin: 55,
                    baud: 2_000_000
                },
            ]
        );
    }

    /// The GPIO table decodes `pin` + `activeLow`; `activeLow` accepts both
    /// `Bool` and 0/1 integer shapes (compilers differ on `bool` DWARF).
    #[test]
    fn gpio_table_decodes_pin_and_polarity() {
        let table = Value::Array(vec![
            entry(vec![("pin", pin(6)), ("activeLow", Value::Bool(false))]),
            entry(vec![("pin", pin(16)), ("activeLow", Value::UInt(1))]),
        ]);
        let decoded = decode_gpio_table(&table, "tbl").expect("decodes");
        assert_eq!(
            decoded,
            vec![
                GpioChannelConfig {
                    pin: 6,
                    active_low: false
                },
                GpioChannelConfig {
                    pin: 16,
                    active_low: true
                },
            ]
        );
    }

    /// The encoder table decodes the A/B phase pins, ignoring the preset and
    /// limit fields the consumer carries alongside them.
    #[test]
    fn encoder_table_decodes_phase_pins() {
        let table = Value::Array(vec![entry(vec![
            ("preset", Value::Int(0)),
            ("lo", Value::Int(-1_000_000)),
            ("hi", Value::Int(1_000_000)),
            ("pinA", pin(9)),
            ("pinB", pin(10)),
        ])]);
        let decoded = decode_encoder_table(&table, "tbl").expect("decodes");
        assert_eq!(
            decoded,
            vec![EncoderConfig {
                pin_a: 9,
                pin_b: 10
            }]
        );
    }

    /// The pulse-out table decodes the step pin, ignoring timing fields.
    #[test]
    fn pulse_out_table_decodes_pin() {
        let table = Value::Array(vec![entry(vec![
            ("maxHardwareClockCyclePerStep", Value::Int(131_070)),
            ("pin", pin(8)),
        ])]);
        let decoded = decode_pulse_out_table(&table, "tbl").expect("decodes");
        assert_eq!(decoded, vec![PulseOutConfig { pin: 8 }]);
    }

    /// A non-array value (a lone struct, a scalar) is a `Shape` error naming
    /// the symbol — never a panic.
    #[test]
    fn non_array_table_is_a_shape_error() {
        let lone = entry(vec![("rx", pin(0))]);
        let err = decode_serial_table(&lone, "tbl").unwrap_err();
        match &err {
            HalTableError::Shape { symbol, detail } => {
                assert_eq!(symbol, "tbl");
                assert!(detail.contains("array"), "got: {detail}");
            }
            other => panic!("expected Shape, got {other:?}"),
        }
    }

    /// Missing fields, non-integer fields, and values that do not fit `u32`
    /// each produce a `Shape` error naming the entry index and field.
    #[test]
    fn field_error_paths_name_index_and_field() {
        // Missing field.
        let table = Value::Array(vec![entry(vec![("rx", pin(0)), ("tx", pin(2))])]);
        let err = decode_serial_table(&table, "tbl").unwrap_err();
        assert!(
            err.to_string().contains("[0]") && err.to_string().contains("baud"),
            "got: {err}"
        );

        // Non-integer field.
        let table = Value::Array(vec![entry(vec![
            ("rx", Value::Float(1.0)),
            ("tx", pin(2)),
            ("baud", Value::Int(9600)),
        ])]);
        let err = decode_serial_table(&table, "tbl").unwrap_err();
        assert!(err.to_string().contains("rx"), "got: {err}");

        // Negative pin does not fit u32.
        let table = Value::Array(vec![entry(vec![
            ("pin", pin(-1)),
            ("activeLow", Value::Bool(false)),
        ])]);
        let err = decode_gpio_table(&table, "tbl").unwrap_err();
        assert!(err.to_string().contains("-1"), "got: {err}");

        // Non-bool polarity.
        let table = Value::Array(vec![entry(vec![
            ("pin", pin(3)),
            ("activeLow", Value::UInt(2)),
        ])]);
        let err = decode_gpio_table(&table, "tbl").unwrap_err();
        assert!(err.to_string().contains("activeLow"), "got: {err}");
    }

    /// `HalTableError` Display renders both variants; `Read` preserves the
    /// underlying error's message and source.
    #[test]
    fn error_display_and_source() {
        let read: HalTableError = ValueReadError::SymbolNotFound {
            symbol: "sym".to_string(),
        }
        .into();
        assert!(read.to_string().contains("sym"));
        assert!(std::error::Error::source(&read).is_some());

        let shape = shape_err("sym", "boom");
        assert!(shape.to_string().contains("sym") && shape.to_string().contains("boom"));
        assert!(std::error::Error::source(&shape).is_none());
    }
}
