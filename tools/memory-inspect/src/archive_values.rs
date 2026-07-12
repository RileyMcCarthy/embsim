//! Archive value reading — decode the *initialized values* of global data
//! symbols straight from a firmware static library (`.a`), using the DWARF
//! layout already parsed by [`FirmwareInfo`].
//!
//! This is the pin-facade enabler from the board-engine design ("The MCU as a
//! component", point 3 in `BOARD_ENGINE.md`): a consumer's HAL wiring/config
//! tables (e.g. `HAL_serial_channelConfig[]`) are data-only globals compiled
//! into the firmware archive. This module locates the symbol's defining
//! object member, reads the initializer bytes from that member's data/rodata
//! section, and decodes them into a structured [`Value`] using the same DWARF
//! struct layouts consumers already parse for enum/type lookups. Unlike
//! [`SymbolResolver`](crate::SymbolResolver), nothing has to be *running* —
//! the values come from the archive file itself, so a CI check can assert a
//! config table is present and non-empty before the emulator ever boots.
//!
//! Both Mach-O and ELF relocatable members are handled (the same formats the
//! DWARF parser reads), and multi-byte values are decoded with the object's
//! endianness.
//!
//! Deliberate limitations:
//! - **Pointer-valued fields are not resolved.** Their initializers live in
//!   relocations, not section bytes; they decode to [`Value::Opaque`].
//! - **Zero-initialized globals** (`.bss`) decode as zeros, which is exactly
//!   their C-semantics initial value.
//! - **Tentative (common) definitions are not supported.** A global compiled
//!   with `-fcommon` lands in a common block, which the symbol indexer does
//!   not treat as a definition — the symbol is simply not indexed
//!   ([`read_value`](ArchiveValueReader::read_value) reports
//!   [`ValueReadError::SymbolNotFound`]). Symbols must be real definitions;
//!   `-fno-common` (the default since GCC 10 / Clang 11) guarantees this by
//!   turning every such global into an ordinary `.bss` definition.
//!
//! # Usage
//! ```rust,ignore
//! use embsim_memory_inspect::{ArchiveValueReader, FirmwareInfo};
//! use std::path::Path;
//!
//! let path = Path::new("libfirmware.a");
//! let fw = FirmwareInfo::from_archive(path).unwrap();
//! let values = ArchiveValueReader::from_archive(path).unwrap();
//!
//! // const HAL_serial_channelConfig_S HAL_serial_channelConfig[2] = {...};
//! let cfg = values.read_value(&fw, "HAL_serial_channelConfig").unwrap();
//! let rx = cfg.index(0).unwrap().field("rx").unwrap().as_i64().unwrap();
//! let baud = cfg.index(0).unwrap().field("baud").unwrap().as_i64().unwrap();
//! ```

use crate::types::{FirmwareInfo, MemInspectError, TypeInfo};
use object::read::archive::ArchiveFile;
use object::{Object, ObjectSection, ObjectSymbol, SymbolSection};
use std::collections::HashMap;
use std::path::Path;
use tracing::{debug, info};

// ============================================================
// Value model
// ============================================================

/// A decoded initialized value, mirroring the DWARF type shape.
///
/// Aggregates recurse: an array of structs decodes to
/// `Value::Array(vec![Value::Struct { .. }, ..])` with per-field values in
/// declaration order. Use the accessors ([`as_i64`](Value::as_i64),
/// [`field`](Value::field), [`index`](Value::index), …) to walk a value
/// without matching on every variant.
#[derive(Debug, Clone, PartialEq)]
pub enum Value {
    /// Signed integer (signed C base types, sign-extended).
    Int(i64),
    /// Unsigned integer (unsigned C base types).
    UInt(u64),
    /// C `bool` / `_Bool` (detected by base-type name).
    Bool(bool),
    /// IEEE-754 `float` / `double`, widened to `f64`.
    Float(f64),
    /// Enum-typed value with its DWARF type name (e.g. `"HW_pin_E"`).
    Enum {
        /// The enum's typedef name.
        type_name: String,
        /// The decoded enumerator value.
        value: i64,
    },
    /// Struct or union: fields in declaration order. (Union members all
    /// decode from the same bytes — every view is present.)
    Struct {
        /// The struct/union's typedef name.
        type_name: String,
        /// `(field name, decoded value)` in declaration order.
        fields: Vec<(String, Value)>,
    },
    /// Fixed-size array of decoded elements.
    Array(Vec<Value>),
    /// Pointer / unsupported type — the raw size is known but the value is
    /// not decodable from section bytes (pointer initializers live in
    /// relocations).
    Opaque {
        /// The type's byte size per DWARF.
        byte_size: usize,
    },
}

impl Value {
    /// The value as a signed integer, if it is integer-shaped
    /// (`Int` / `UInt` / `Bool` / `Enum`).
    pub fn as_i64(&self) -> Option<i64> {
        match self {
            Value::Int(v) => Some(*v),
            Value::UInt(v) => i64::try_from(*v).ok(),
            Value::Bool(b) => Some(i64::from(*b)),
            Value::Enum { value, .. } => Some(*value),
            _ => None,
        }
    }

    /// The value as a bool: `Bool` directly, or an integer that is exactly
    /// 0 or 1 (a `bool` field whose base type the compiler named unusually).
    pub fn as_bool(&self) -> Option<bool> {
        match self {
            Value::Bool(b) => Some(*b),
            Value::Int(0) | Value::UInt(0) => Some(false),
            Value::Int(1) | Value::UInt(1) => Some(true),
            _ => None,
        }
    }

    /// The value as an `f64` (floats directly, integer shapes widened).
    pub fn as_f64(&self) -> Option<f64> {
        match self {
            Value::Float(v) => Some(*v),
            Value::Int(v) => Some(*v as f64),
            Value::UInt(v) => Some(*v as f64),
            Value::Bool(b) => Some(f64::from(u8::from(*b))),
            Value::Enum { value, .. } => Some(*value as f64),
            _ => None,
        }
    }

    /// A struct/union field by name, or `None` if this is not a struct or
    /// has no such field.
    pub fn field(&self, name: &str) -> Option<&Value> {
        match self {
            Value::Struct { fields, .. } => fields.iter().find(|(n, _)| n == name).map(|(_, v)| v),
            _ => None,
        }
    }

    /// The array elements, or `None` if this is not an array.
    pub fn elements(&self) -> Option<&[Value]> {
        match self {
            Value::Array(elems) => Some(elems),
            _ => None,
        }
    }

    /// Array element by index, or `None` if this is not an array or the
    /// index is out of bounds.
    pub fn index(&self, i: usize) -> Option<&Value> {
        self.elements()?.get(i)
    }
}

// ============================================================
// Errors
// ============================================================

/// Errors raised while reading an initialized value from the archive.
#[derive(Debug)]
pub enum ValueReadError {
    /// No object member in the archive defines the symbol.
    SymbolNotFound {
        /// The requested symbol name.
        symbol: String,
    },
    /// The archive's DWARF has no variable entry (type layout) for the symbol.
    MissingTypeInfo {
        /// The requested symbol name.
        symbol: String,
    },
    /// The defining member/section bytes could not be read.
    Section {
        /// The requested symbol name.
        symbol: String,
        /// What went wrong.
        detail: String,
    },
    /// The initializer bytes could not be decoded against the DWARF layout.
    Decode {
        /// The requested symbol name.
        symbol: String,
        /// What went wrong.
        detail: String,
    },
}

impl std::fmt::Display for ValueReadError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ValueReadError::SymbolNotFound { symbol } => {
                write!(
                    f,
                    "symbol '{symbol}' is not defined by any object member in the archive"
                )
            }
            ValueReadError::MissingTypeInfo { symbol } => write!(
                f,
                "no DWARF variable entry for symbol '{symbol}' (was the firmware built with -g?)"
            ),
            ValueReadError::Section { symbol, detail } => {
                write!(f, "cannot read initializer bytes for '{symbol}': {detail}")
            }
            ValueReadError::Decode { symbol, detail } => {
                write!(
                    f,
                    "cannot decode '{symbol}' against its DWARF layout: {detail}"
                )
            }
        }
    }
}

impl std::error::Error for ValueReadError {}

fn section_err(symbol: &str, detail: impl Into<String>) -> ValueReadError {
    ValueReadError::Section {
        symbol: symbol.to_string(),
        detail: detail.into(),
    }
}

fn decode_err(symbol: &str, detail: impl Into<String>) -> ValueReadError {
    ValueReadError::Decode {
        symbol: symbol.to_string(),
        detail: detail.into(),
    }
}

// ============================================================
// ArchiveValueReader
// ============================================================

/// Reads the initialized values of global data symbols from a firmware
/// static library archive, decoding them with DWARF layout info from
/// [`FirmwareInfo`] (typically parsed from the same archive).
pub struct ArchiveValueReader {
    /// Full archive bytes; members are re-parsed lazily per read.
    data: Vec<u8>,
    /// Defined symbol name (Mach-O leading underscore stripped) → file range
    /// `(offset, size)` of the defining object member. First definition wins,
    /// except a strong (non-weak) definition displaces a weak one indexed
    /// from an earlier member — mirroring linker symbol resolution.
    symbols: HashMap<String, (u64, u64)>,
}

impl ArchiveValueReader {
    /// Index every object member's defined symbols from a static library
    /// archive. Non-object members (e.g. the archive symbol table) are
    /// skipped, matching [`FirmwareInfo::from_archive`].
    pub fn from_archive(archive_path: &Path) -> Result<Self, MemInspectError> {
        let data = std::fs::read(archive_path).map_err(MemInspectError::Io)?;

        let archive =
            ArchiveFile::parse(&*data).map_err(|e| MemInspectError::Archive(e.to_string()))?;

        // Symbol → (member range, definition-is-weak). First definition wins,
        // except a strong definition displaces a previously indexed weak one
        // (linker semantics — a weak def in an early member must not shadow
        // the real one in a later member).
        let mut indexed: HashMap<String, ((u64, u64), bool)> = HashMap::new();
        for member in archive.members() {
            let member = member.map_err(|e| MemInspectError::Archive(e.to_string()))?;
            let range = member.file_range();
            let member_data = member
                .data(&*data)
                .map_err(|e| MemInspectError::Archive(e.to_string()))?;
            let Ok(obj) = object::File::parse(member_data) else {
                continue; // non-object member (symbol table, string table, ...)
            };
            for symbol in obj.symbols() {
                if !symbol.is_definition() {
                    continue;
                }
                let Ok(name) = symbol.name() else { continue };
                let clean = cleaned_name(obj.format(), name);
                let weak = symbol.is_weak();
                match indexed.entry(clean.to_string()) {
                    std::collections::hash_map::Entry::Vacant(e) => {
                        e.insert((range, weak));
                    }
                    std::collections::hash_map::Entry::Occupied(mut e) => {
                        if e.get().1 && !weak {
                            e.insert((range, weak));
                        }
                    }
                }
            }
        }
        let symbols = indexed
            .into_iter()
            .map(|(name, (range, _weak))| (name, range))
            .collect::<HashMap<_, _>>();

        info!(
            "Archive value reader: {} defined symbols indexed",
            symbols.len()
        );

        Ok(Self { data, symbols })
    }

    /// `true` if some object member defines this symbol — the "config table
    /// present" CI probe. (Any defined symbol counts, including functions;
    /// [`read_value`](Self::read_value) additionally requires a DWARF
    /// variable entry.)
    pub fn has_symbol(&self, name: &str) -> bool {
        self.symbols.contains_key(name)
    }

    /// Read and decode the initialized value of a global data symbol.
    ///
    /// The symbol's type layout comes from `fw` (parse it from the same
    /// archive), its bytes from the defining member's section at the symbol's
    /// offset. Symbols in zero-initialized storage (`.bss`) decode as zeros.
    pub fn read_value(&self, fw: &FirmwareInfo, symbol: &str) -> Result<Value, ValueReadError> {
        let &(offset, size) =
            self.symbols
                .get(symbol)
                .ok_or_else(|| ValueReadError::SymbolNotFound {
                    symbol: symbol.to_string(),
                })?;
        let var_info = fw
            .variables
            .get(symbol)
            .ok_or_else(|| ValueReadError::MissingTypeInfo {
                symbol: symbol.to_string(),
            })?;
        let byte_size = var_info.type_info.byte_size();
        if byte_size == 0 {
            return Err(decode_err(symbol, "DWARF reports a zero-sized type"));
        }

        let start = offset as usize;
        let end = start
            .checked_add(size as usize)
            .ok_or_else(|| section_err(symbol, "member range overflows"))?;
        let member_data = self
            .data
            .get(start..end)
            .ok_or_else(|| section_err(symbol, "member range exceeds the archive"))?;
        let obj = object::File::parse(member_data)
            .map_err(|e| section_err(symbol, format!("member is not a parseable object: {e}")))?;

        let bytes = initializer_bytes(&obj, symbol, byte_size)
            .map_err(|detail| section_err(symbol, detail))?;

        decode_value(
            fw,
            &var_info.type_info,
            &bytes,
            obj.is_little_endian(),
            symbol,
        )
    }
}

/// Symbol-table names on Mach-O carry a leading underscore the C source
/// doesn't have; ELF names match the source directly.
fn cleaned_name(format: object::BinaryFormat, name: &str) -> &str {
    if format == object::BinaryFormat::MachO {
        name.strip_prefix('_').unwrap_or(name)
    } else {
        name
    }
}

/// Extract `len` initializer bytes for `symbol` from its defining section in
/// one object member. Zero-initialized storage (`.bss`) yields zeros. Errors
/// are plain strings; the caller wraps them with context.
fn initializer_bytes(obj: &object::File, symbol: &str, len: usize) -> Result<Vec<u8>, String> {
    let sym = obj
        .symbols()
        .find(|s| {
            s.is_definition()
                && s.name()
                    .map(|n| cleaned_name(obj.format(), n) == symbol)
                    .unwrap_or(false)
        })
        .ok_or_else(|| "symbol not defined in the indexed member".to_string())?;

    // ELF symbols carry a size; when it disagrees with DWARF, trust DWARF for
    // the read length but leave a trace for diagnosis.
    if sym.size() > 0 && sym.size() != len as u64 {
        debug!(
            "symbol '{}' size mismatch: symtab says {} bytes, DWARF says {}",
            symbol,
            sym.size(),
            len
        );
    }

    match sym.section() {
        // Unreachable via `read_value`: `is_definition()` is false for common
        // symbols, so they are never indexed (see the module docs). Kept as
        // an explicit error rather than a dead "decode as zeros" path.
        SymbolSection::Common => {
            Err("tentative (common) definition; build with -fno-common".to_string())
        }
        SymbolSection::Section(index) => {
            let section = obj
                .section_by_index(index)
                .map_err(|e| format!("bad section index for symbol: {e}"))?;
            // .bss and friends occupy no file bytes; their initial value is zero.
            if section.kind().is_bss() {
                return Ok(vec![0u8; len]);
            }
            let data = section
                .data()
                .map_err(|e| format!("cannot read section data: {e}"))?;
            let start = sym
                .address()
                .checked_sub(section.address())
                .ok_or_else(|| "symbol address below its section base".to_string())?
                as usize;
            let end = start
                .checked_add(len)
                .ok_or_else(|| "symbol extent overflows".to_string())?;
            data.get(start..end).map(<[u8]>::to_vec).ok_or_else(|| {
                format!(
                    "initializer [{start}..{end}] exceeds section '{}' ({} bytes)",
                    section.name().unwrap_or("?"),
                    data.len()
                )
            })
        }
        other => Err(format!("symbol has no data section ({other:?})")),
    }
}

// ============================================================
// Decoding
// ============================================================

/// Recursively decode `bytes` against a DWARF [`TypeInfo`]. `bytes` must be
/// at least `ti.byte_size()` long; aggregate recursion slices exactly.
fn decode_value(
    fw: &FirmwareInfo,
    ti: &TypeInfo,
    bytes: &[u8],
    little_endian: bool,
    symbol: &str,
) -> Result<Value, ValueReadError> {
    match ti {
        TypeInfo::Base {
            name,
            byte_size,
            signed,
        } => {
            let raw = read_uint(bytes, *byte_size, little_endian).ok_or_else(|| {
                decode_err(symbol, format!("unsupported base type size {byte_size}"))
            })?;
            // C99 `bool` is spelled `_Bool` in DWARF; C++ spells it `bool`.
            if name == "_Bool" || name == "bool" {
                Ok(Value::Bool(raw != 0))
            } else if *signed {
                Ok(Value::Int(sign_extend(raw, *byte_size * 8)))
            } else {
                Ok(Value::UInt(raw))
            }
        }
        TypeInfo::Float { byte_size: 4, .. } => {
            let raw = read_uint(bytes, 4, little_endian)
                .ok_or_else(|| decode_err(symbol, "truncated f32"))?;
            Ok(Value::Float(f64::from(f32::from_bits(raw as u32))))
        }
        TypeInfo::Float { byte_size: 8, .. } => {
            let raw = read_uint(bytes, 8, little_endian)
                .ok_or_else(|| decode_err(symbol, "truncated f64"))?;
            Ok(Value::Float(f64::from_bits(raw)))
        }
        TypeInfo::Float { byte_size, .. } => Err(decode_err(
            symbol,
            format!("unsupported float size {byte_size}"),
        )),
        TypeInfo::Enum {
            type_name,
            byte_size,
        } => {
            let raw = read_uint(bytes, *byte_size, little_endian)
                .ok_or_else(|| decode_err(symbol, format!("unsupported enum size {byte_size}")))?;
            Ok(Value::Enum {
                type_name: type_name.clone(),
                value: sign_extend(raw, *byte_size * 8),
            })
        }
        TypeInfo::Bitfield {
            bit_offset,
            bit_size,
            storage_size,
            signed,
        } => {
            if *bit_size == 0 || *bit_size > 64 {
                return Err(decode_err(
                    symbol,
                    format!("invalid bitfield width {bit_size}"),
                ));
            }
            let storage = (*storage_size).min(8);
            // The bits must lie inside the storage unit actually read. An
            // out-of-range offset (e.g. a DWARF5 `DW_AT_data_bit_offset` the
            // parser could not normalize to the member's byte offset) would
            // otherwise wrap the shift and extract the wrong bits — fail
            // loud instead of returning wrong data.
            let available_bits = storage as u64 * 8;
            if bit_offset
                .checked_add(*bit_size)
                .is_none_or(|end| end > available_bits)
            {
                return Err(decode_err(
                    symbol,
                    format!(
                        "bitfield bits [{bit_offset}, {bit_offset}+{bit_size}) exceed the \
                         {storage}-byte storage unit read for it (unnormalized \
                         DW_AT_data_bit_offset?)"
                    ),
                ));
            }
            let raw = read_uint(bytes, storage, little_endian)
                .ok_or_else(|| decode_err(symbol, "truncated bitfield storage"))?;
            let mask = if *bit_size == 64 {
                u64::MAX
            } else {
                (1u64 << bit_size) - 1
            };
            // In range by the check above: bit_offset + bit_size <= 64 and
            // bit_size >= 1, so the shift amount is < 64.
            let field = (raw >> bit_offset) & mask;
            if *signed {
                Ok(Value::Int(sign_extend(field, *bit_size as usize)))
            } else {
                Ok(Value::UInt(field))
            }
        }
        TypeInfo::Struct { type_name, .. } | TypeInfo::Union { type_name, .. } => {
            let info = fw.structs.get(type_name).ok_or_else(|| {
                decode_err(
                    symbol,
                    format!("no DWARF layout for struct/union '{type_name}'"),
                )
            })?;
            let mut fields = Vec::with_capacity(info.fields.len());
            for f in &info.fields {
                let fsize = f.type_info.byte_size();
                let end = f.offset.checked_add(fsize).ok_or_else(|| {
                    decode_err(symbol, format!("field '{}' extent overflows", f.name))
                })?;
                let slice = bytes.get(f.offset..end).ok_or_else(|| {
                    decode_err(
                        symbol,
                        format!(
                            "field '{}.{}' [{}, {end}) exceeds the {}-byte value",
                            type_name,
                            f.name,
                            f.offset,
                            bytes.len()
                        ),
                    )
                })?;
                let value =
                    decode_value(fw, &f.type_info, slice, little_endian, symbol).map_err(|e| {
                        match e {
                            // Name the failing field so nested errors (e.g. a
                            // bitfield range violation) identify their source.
                            ValueReadError::Decode { symbol, detail } => ValueReadError::Decode {
                                symbol,
                                detail: format!("field '{}.{}': {detail}", type_name, f.name),
                            },
                            other => other,
                        }
                    })?;
                fields.push((f.name.clone(), value));
            }
            Ok(Value::Struct {
                type_name: type_name.clone(),
                fields,
            })
        }
        TypeInfo::Array {
            element_type,
            count,
        } => {
            let elem_size = element_type.byte_size();
            if elem_size == 0 && *count > 0 {
                return Err(decode_err(symbol, "array of zero-sized elements"));
            }
            let mut elems = Vec::with_capacity(*count);
            for i in 0..*count {
                let start = i * elem_size;
                let slice = bytes.get(start..start + elem_size).ok_or_else(|| {
                    decode_err(symbol, format!("array element {i} exceeds the value bytes"))
                })?;
                elems.push(decode_value(
                    fw,
                    element_type,
                    slice,
                    little_endian,
                    symbol,
                )?);
            }
            Ok(Value::Array(elems))
        }
        TypeInfo::Pointer { byte_size } | TypeInfo::Unknown { byte_size } => {
            // Pointer initializers are relocations, not section bytes — the
            // value here would be meaningless. Surface the shape, not garbage.
            Ok(Value::Opaque {
                byte_size: *byte_size,
            })
        }
    }
}

/// Assemble an unsigned integer of `size` bytes (1..=8) from `bytes` with the
/// given endianness. `None` if the size is unsupported or `bytes` is short.
fn read_uint(bytes: &[u8], size: usize, little_endian: bool) -> Option<u64> {
    if size == 0 || size > 8 || bytes.len() < size {
        return None;
    }
    let mut v: u64 = 0;
    if little_endian {
        for i in (0..size).rev() {
            v = (v << 8) | u64::from(bytes[i]);
        }
    } else {
        for &b in &bytes[..size] {
            v = (v << 8) | u64::from(b);
        }
    }
    Some(v)
}

/// Sign-extend the low `bits` of `raw` to an `i64`.
fn sign_extend(raw: u64, bits: usize) -> i64 {
    if bits == 0 || bits >= 64 {
        return raw as i64;
    }
    let shift = 64 - bits;
    ((raw << shift) as i64) >> shift
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{FieldInfo, StructInfo};

    // ── read_uint / sign_extend ──

    /// `read_uint` assembles both endiannesses and rejects bad sizes.
    #[test]
    fn read_uint_endianness_and_bounds() {
        let bytes = [0x01, 0x02, 0x03, 0x04];
        assert_eq!(read_uint(&bytes, 4, true), Some(0x0403_0201));
        assert_eq!(read_uint(&bytes, 4, false), Some(0x0102_0304));
        assert_eq!(read_uint(&bytes, 2, true), Some(0x0201));
        assert_eq!(read_uint(&bytes, 1, false), Some(0x01));
        // Odd (but <= 8) sizes work too.
        assert_eq!(read_uint(&bytes, 3, true), Some(0x03_0201));
        // Unsupported / short.
        assert_eq!(read_uint(&bytes, 0, true), None);
        assert_eq!(read_uint(&bytes, 9, true), None);
        assert_eq!(read_uint(&bytes, 5, true), None);
    }

    /// `sign_extend` extends negative values and passes positives through.
    #[test]
    fn sign_extend_cases() {
        assert_eq!(sign_extend(0xFF, 8), -1);
        assert_eq!(sign_extend(0x7F, 8), 127);
        assert_eq!(sign_extend(0xFFFF_FFFF, 32), -1);
        assert_eq!(sign_extend(0x8000_0000, 32), i64::from(i32::MIN));
        assert_eq!(sign_extend(42, 32), 42);
        assert_eq!(sign_extend(u64::MAX, 64), -1);
        // 3-bit field holding 0b111 = -1.
        assert_eq!(sign_extend(0b111, 3), -1);
    }

    // ── decode_value against a hand-built layout ──

    /// A firmware model mirroring the HAL serial config shape:
    /// `struct { enum rx; enum tx; int32_t baud; enum type; bool lsb; }`.
    fn config_fw() -> FirmwareInfo {
        let mut fw = FirmwareInfo::new();
        fw.structs.insert(
            "cfg_S".to_string(),
            StructInfo {
                name: "cfg_S".to_string(),
                byte_size: 20,
                fields: vec![
                    FieldInfo {
                        name: "rx".into(),
                        offset: 0,
                        type_info: TypeInfo::Enum {
                            type_name: "pin_E".into(),
                            byte_size: 4,
                        },
                    },
                    FieldInfo {
                        name: "tx".into(),
                        offset: 4,
                        type_info: TypeInfo::Enum {
                            type_name: "pin_E".into(),
                            byte_size: 4,
                        },
                    },
                    FieldInfo {
                        name: "baud".into(),
                        offset: 8,
                        type_info: TypeInfo::Base {
                            name: "int".into(),
                            byte_size: 4,
                            signed: true,
                        },
                    },
                    FieldInfo {
                        name: "type".into(),
                        offset: 12,
                        type_info: TypeInfo::Enum {
                            type_name: "type_E".into(),
                            byte_size: 4,
                        },
                    },
                    FieldInfo {
                        name: "lsb".into(),
                        offset: 16,
                        type_info: TypeInfo::Base {
                            name: "_Bool".into(),
                            byte_size: 1,
                            signed: false,
                        },
                    },
                ],
            },
        );
        fw
    }

    /// Little-endian bytes for one `cfg_S` value.
    fn cfg_bytes(rx: i32, tx: i32, baud: i32, ty: i32, lsb: bool) -> Vec<u8> {
        let mut bytes = Vec::with_capacity(20);
        bytes.extend_from_slice(&rx.to_le_bytes());
        bytes.extend_from_slice(&tx.to_le_bytes());
        bytes.extend_from_slice(&baud.to_le_bytes());
        bytes.extend_from_slice(&ty.to_le_bytes());
        bytes.push(u8::from(lsb));
        bytes.extend_from_slice(&[0, 0, 0]); // struct tail padding
        bytes
    }

    /// An array of config structs decodes element-by-element, field-by-field.
    #[test]
    fn decodes_array_of_config_structs() {
        let fw = config_fw();
        let ti = TypeInfo::Array {
            element_type: Box::new(TypeInfo::Struct {
                type_name: "cfg_S".into(),
                byte_size: 20,
            }),
            count: 2,
        };
        let mut bytes = cfg_bytes(0, 2, 115_200, 0, false);
        bytes.extend(cfg_bytes(53, 55, 2_000_000, 1, true));

        let v = decode_value(&fw, &ti, &bytes, true, "cfg").unwrap();
        let elems = v.elements().expect("array");
        assert_eq!(elems.len(), 2);

        let e0 = &elems[0];
        assert_eq!(e0.field("rx").unwrap().as_i64(), Some(0));
        assert_eq!(e0.field("tx").unwrap().as_i64(), Some(2));
        assert_eq!(e0.field("baud").unwrap(), &Value::Int(115_200));
        assert_eq!(e0.field("type").unwrap().as_i64(), Some(0));
        assert_eq!(e0.field("lsb").unwrap(), &Value::Bool(false));

        let e1 = v.index(1).expect("second element");
        assert_eq!(e1.field("rx").unwrap().as_i64(), Some(53));
        assert_eq!(e1.field("tx").unwrap().as_i64(), Some(55));
        assert_eq!(e1.field("baud").unwrap().as_i64(), Some(2_000_000));
        assert_eq!(e1.field("type").unwrap().as_i64(), Some(1));
        assert_eq!(e1.field("lsb").unwrap().as_bool(), Some(true));

        // Enum fields carry their DWARF type name.
        match e1.field("rx").unwrap() {
            Value::Enum { type_name, value } => {
                assert_eq!(type_name, "pin_E");
                assert_eq!(*value, 53);
            }
            other => panic!("rx should be an Enum, got {other:?}"),
        }
    }

    /// Scalar shapes decode to the matching `Value` variant (both endiannesses
    /// for a multi-byte int).
    #[test]
    fn decodes_scalars() {
        let fw = FirmwareInfo::new();

        let i32_ti = TypeInfo::Base {
            name: "int".into(),
            byte_size: 4,
            signed: true,
        };
        assert_eq!(
            decode_value(&fw, &i32_ti, &(-42i32).to_le_bytes(), true, "v").unwrap(),
            Value::Int(-42)
        );
        assert_eq!(
            decode_value(&fw, &i32_ti, &(-42i32).to_be_bytes(), false, "v").unwrap(),
            Value::Int(-42)
        );

        let u8_ti = TypeInfo::Base {
            name: "unsigned char".into(),
            byte_size: 1,
            signed: false,
        };
        assert_eq!(
            decode_value(&fw, &u8_ti, &[200], true, "v").unwrap(),
            Value::UInt(200)
        );

        let bool_ti = TypeInfo::Base {
            name: "_Bool".into(),
            byte_size: 1,
            signed: false,
        };
        assert_eq!(
            decode_value(&fw, &bool_ti, &[1], true, "v").unwrap(),
            Value::Bool(true)
        );
        assert_eq!(
            decode_value(&fw, &bool_ti, &[0], true, "v").unwrap(),
            Value::Bool(false)
        );

        let f32_ti = TypeInfo::Float {
            name: "float".into(),
            byte_size: 4,
        };
        assert_eq!(
            decode_value(&fw, &f32_ti, &1.5f32.to_le_bytes(), true, "v").unwrap(),
            Value::Float(1.5)
        );

        let f64_ti = TypeInfo::Float {
            name: "double".into(),
            byte_size: 8,
        };
        assert_eq!(
            decode_value(&fw, &f64_ti, &(-2.25f64).to_le_bytes(), true, "v").unwrap(),
            Value::Float(-2.25)
        );
    }

    /// Bitfields extract the right bits; signed ones sign-extend.
    #[test]
    fn decodes_bitfields() {
        let fw = FirmwareInfo::new();

        let unsigned = TypeInfo::Bitfield {
            bit_offset: 4,
            bit_size: 3,
            storage_size: 1,
            signed: false,
        };
        assert_eq!(
            decode_value(&fw, &unsigned, &[0b0101_0000], true, "v").unwrap(),
            Value::UInt(5)
        );

        let signed = TypeInfo::Bitfield {
            bit_offset: 0,
            bit_size: 3,
            storage_size: 1,
            signed: true,
        };
        assert_eq!(
            decode_value(&fw, &signed, &[0b0000_0111], true, "v").unwrap(),
            Value::Int(-1)
        );
    }

    /// A bitfield whose bits do not fit the storage unit read for it (e.g. an
    /// absolute DWARF5 `DW_AT_data_bit_offset` the parser could not normalize
    /// to the member's byte offset) is a `Decode` error — never a silently
    /// wrapped shift returning wrong bits.
    #[test]
    fn out_of_range_bitfield_is_a_decode_error() {
        let fw = FirmwareInfo::new();

        // Absolute struct-level bit offset 72 against a 4-byte storage unit:
        // the old modulo-64 shift would have "decoded" bits 8..11.
        let unnormalized = TypeInfo::Bitfield {
            bit_offset: 72,
            bit_size: 3,
            storage_size: 4,
            signed: false,
        };
        let err = decode_value(&fw, &unnormalized, &[0xFF; 4], true, "v").unwrap_err();
        match &err {
            ValueReadError::Decode { detail, .. } => {
                assert!(detail.contains("72"), "offset named in: {detail}");
            }
            other => panic!("expected Decode, got {other:?}"),
        }

        // Boundary: the topmost bits of the storage unit still decode…
        let last_bits = TypeInfo::Bitfield {
            bit_offset: 29,
            bit_size: 3,
            storage_size: 4,
            signed: false,
        };
        assert_eq!(
            decode_value(&fw, &last_bits, &[0, 0, 0, 0b1110_0000], true, "v").unwrap(),
            Value::UInt(7)
        );
        // …but one bit past the end errors.
        let one_past = TypeInfo::Bitfield {
            bit_offset: 30,
            bit_size: 3,
            storage_size: 4,
            signed: false,
        };
        assert!(decode_value(&fw, &one_past, &[0xFF; 4], true, "v").is_err());

        // Overflow of bit_offset + bit_size must not wrap into "fits".
        let overflow = TypeInfo::Bitfield {
            bit_offset: u64::MAX,
            bit_size: 3,
            storage_size: 4,
            signed: false,
        };
        assert!(decode_value(&fw, &overflow, &[0xFF; 4], true, "v").is_err());
    }

    /// Pointers and unknown types decode to `Opaque`, never garbage integers.
    #[test]
    fn pointers_and_unknowns_are_opaque() {
        let fw = FirmwareInfo::new();
        let bytes = [0u8; 8];
        assert_eq!(
            decode_value(&fw, &TypeInfo::Pointer { byte_size: 8 }, &bytes, true, "v").unwrap(),
            Value::Opaque { byte_size: 8 }
        );
        assert_eq!(
            decode_value(&fw, &TypeInfo::Unknown { byte_size: 8 }, &bytes, true, "v").unwrap(),
            Value::Opaque { byte_size: 8 }
        );
    }

    /// A struct type with no layout in `FirmwareInfo`, or bytes too short for
    /// a field, produce `Decode` errors — not panics or wrong values.
    #[test]
    fn decode_error_paths() {
        let fw = config_fw();

        let missing = TypeInfo::Struct {
            type_name: "nope_S".into(),
            byte_size: 4,
        };
        let err = decode_value(&fw, &missing, &[0u8; 4], true, "v").unwrap_err();
        assert!(matches!(err, ValueReadError::Decode { .. }), "got {err:?}");

        let cfg = TypeInfo::Struct {
            type_name: "cfg_S".into(),
            byte_size: 20,
        };
        let err = decode_value(&fw, &cfg, &[0u8; 8], true, "v").unwrap_err();
        assert!(matches!(err, ValueReadError::Decode { .. }), "got {err:?}");
    }

    // ── Value accessors ──

    /// The accessors return `Some` for matching shapes and `None` otherwise.
    #[test]
    fn value_accessors() {
        assert_eq!(Value::Int(-3).as_i64(), Some(-3));
        assert_eq!(Value::UInt(7).as_i64(), Some(7));
        assert_eq!(Value::UInt(u64::MAX).as_i64(), None); // doesn't fit
        assert_eq!(Value::Bool(true).as_i64(), Some(1));
        assert_eq!(
            Value::Enum {
                type_name: "E".into(),
                value: 9
            }
            .as_i64(),
            Some(9)
        );
        assert_eq!(Value::Float(1.0).as_i64(), None);

        assert_eq!(Value::Bool(false).as_bool(), Some(false));
        assert_eq!(Value::UInt(1).as_bool(), Some(true));
        assert_eq!(Value::Int(0).as_bool(), Some(false));
        assert_eq!(Value::UInt(2).as_bool(), None);

        assert_eq!(Value::Float(2.5).as_f64(), Some(2.5));
        assert_eq!(Value::Int(-2).as_f64(), Some(-2.0));

        let s = Value::Struct {
            type_name: "S".into(),
            fields: vec![("a".into(), Value::Int(1))],
        };
        assert_eq!(s.field("a"), Some(&Value::Int(1)));
        assert_eq!(s.field("b"), None);
        assert_eq!(Value::Int(0).field("a"), None);

        let a = Value::Array(vec![Value::Int(1), Value::Int(2)]);
        assert_eq!(a.elements().map(<[Value]>::len), Some(2));
        assert_eq!(a.index(1), Some(&Value::Int(2)));
        assert_eq!(a.index(2), None);
        assert_eq!(Value::Int(0).elements(), None);
    }

    /// `ValueReadError` Display renders every variant with the symbol name.
    #[test]
    fn error_display() {
        let cases: Vec<ValueReadError> = vec![
            ValueReadError::SymbolNotFound {
                symbol: "sym".into(),
            },
            ValueReadError::MissingTypeInfo {
                symbol: "sym".into(),
            },
            section_err("sym", "boom"),
            decode_err("sym", "boom"),
        ];
        for err in cases {
            let s = err.to_string();
            assert!(s.contains("sym"), "missing symbol name in: {s}");
        }
    }
}
