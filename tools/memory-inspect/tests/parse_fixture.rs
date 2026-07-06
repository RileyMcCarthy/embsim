//! End-to-end DWARF parsing coverage for [`FirmwareInfo::from_archive`].
//!
//! Compiles a small C fixture *with debug info* into a static `ar` archive at
//! test time, then parses it back and asserts the recovered enum/struct/field
//! layout matches the source. The fixture deliberately exercises every shape the
//! parser handles: an enum with a `_COUNT` variant, a nested struct, an array of
//! structs, `float`/`double`, bitfields, a union, typedefs, an enum-typed field,
//! and top-level variables.
//!
//! If no C compiler (`clang`/`cc`/`gcc`) and `ar` are available, the tests
//! print a skip message and pass rather than failing on a toolchain that can't
//! build the fixture. `clang` is preferred: the parser targets clang-emitted
//! DWARF (its real use case), and gcc's DWARF encodes some shapes differently.
//!
//! One test additionally cross-compiles the fixture to an **ELF x86_64
//! relocatable object** (clang `--target=`, no linker needed) so the
//! relocation-applying read path is exercised on every host, not just Linux.

use embsim_memory_inspect::{FirmwareInfo, ParseOptions, TypeInfo};
use std::path::{Path, PathBuf};
use std::process::Command;

/// The C fixture. Covers: enum (with `_COUNT`), nested struct, array of structs,
/// float + double, bitfields, union, typedefs, enum-typed field, top-level vars.
const FIXTURE_C: &str = r#"
#include <stdint.h>
typedef enum { HAL_GPIO_A=0, HAL_GPIO_B=1, HAL_GPIO_channel_COUNT=2 } HAL_GPIO_channel_E;
typedef struct { uint8_t running:1; uint8_t mode:3; int8_t trim:4; } flags_S;
typedef union { uint32_t raw; float as_f; } conv_U;
typedef struct { int32_t state; float position_mm; double load_n; flags_S flags; conv_U conv; } channel_S;
typedef struct { channel_S channels[3]; uint32_t count; HAL_GPIO_channel_E sel; } data_S;
data_S g_data;
HAL_GPIO_channel_E g_sel = HAL_GPIO_B;
"#;

/// Returns the first available C compiler binary name, or `None`. `clang`
/// first — the parser targets clang-emitted DWARF, and on Linux `cc` is gcc.
fn find_compiler() -> Option<&'static str> {
    for cc in ["clang", "cc", "gcc"] {
        if Command::new(cc).arg("--version").output().is_ok() {
            return Some(cc);
        }
    }
    None
}

/// Create a unique temp dir under the OS temp dir, namespaced by `tag` + pid +
/// a nonce so parallel test binaries don't collide. Returns `None` (after
/// printing a skip message) if the dir can't be created.
fn make_temp_dir(tag: &str) -> Option<PathBuf> {
    let nonce = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_nanos())
        .unwrap_or(0);
    let dir = std::env::temp_dir().join(format!(
        "embsim_dwarf_{}_{}_{}",
        tag,
        std::process::id(),
        nonce
    ));
    if std::fs::create_dir_all(&dir).is_err() {
        eprintln!("SKIP: could not create temp dir {}", dir.display());
        return None;
    }
    Some(dir)
}

/// Archive `o_path` into `a_path`, trying `ar` then `llvm-ar` (some system
/// `ar`s refuse foreign-format members). Prints a skip message and returns
/// `false` if neither produces the archive.
fn archive_object(a_path: &Path, o_path: &Path) -> bool {
    let (Some(a), Some(o)) = (a_path.to_str(), o_path.to_str()) else {
        eprintln!("SKIP: non-UTF-8 temp path");
        return false;
    };
    for ar in ["ar", "llvm-ar"] {
        let _ = std::fs::remove_file(a_path);
        let archived = Command::new(ar)
            .args(["rcs", a, o])
            .output()
            .map(|out| out.status.success() && a_path.exists())
            .unwrap_or(false);
        if archived {
            return true;
        }
    }
    eprintln!(
        "SKIP: failed to archive {} with ar or llvm-ar",
        o_path.display()
    );
    false
}

/// Write [`FIXTURE_C`] into a fresh temp dir (namespaced by `tag`), compile it
/// with `cc` trying each flag set in `flag_sets` in order (`-c`/`-o` appended),
/// and archive the object. Returns the archive path plus the dir (kept alive so
/// it isn't cleaned up while the archive is in use), or `None` (after printing
/// a skip message) if any step fails.
fn build_archive_with(cc: &str, flag_sets: &[&[&str]], tag: &str) -> Option<(PathBuf, PathBuf)> {
    let dir = make_temp_dir(tag)?;
    let c_path = dir.join("fixture.c");
    let o_path = dir.join("fixture.o");
    let a_path = dir.join("libfixture.a");

    if std::fs::write(&c_path, FIXTURE_C).is_err() {
        eprintln!("SKIP: could not write fixture.c");
        return None;
    }

    let c = c_path.to_str()?;
    let o = o_path.to_str()?;
    let compiled = flag_sets.iter().any(|flags| {
        let mut args: Vec<&str> = flags.to_vec();
        args.extend_from_slice(&["-c", c, "-o", o]);
        Command::new(cc)
            .args(&args)
            .output()
            .map(|out| out.status.success() && o_path.exists())
            .unwrap_or(false)
    });
    if !compiled {
        eprintln!("SKIP: failed to compile fixture.c with {}", cc);
        return None;
    }

    if !archive_object(&a_path, &o_path) {
        return None;
    }

    Some((a_path, dir))
}

/// Build the fixture archive for the **host** target. Compiles with DWARF
/// debug info pinned to DWARF v4 for the widest gimli support; if the compiler
/// rejects the flag, falls back to plain `-g`.
fn build_fixture_archive() -> Option<(PathBuf, PathBuf)> {
    let cc = match find_compiler() {
        Some(c) => c,
        None => {
            eprintln!("SKIP: no C compiler (cc/clang/gcc) available");
            return None;
        }
    };
    build_archive_with(cc, &[&["-g", "-gdwarf-4"], &["-g"]], "fixture")
}

/// Build the fixture archive as an **ELF x86_64 relocatable object**,
/// regardless of host OS. clang can emit ELF objects for a foreign target
/// without a linker or sysroot (`-c` only; the fixture needs no libc headers
/// beyond the compiler-provided `<stdint.h>`). Requires clang specifically —
/// gcc can't cross-target like this.
fn build_elf_fixture_archive() -> Option<(PathBuf, PathBuf)> {
    if Command::new("clang").arg("--version").output().is_err() {
        eprintln!("SKIP: clang not available for the ELF cross-compile");
        return None;
    }
    build_archive_with(
        "clang",
        &[&["--target=x86_64-unknown-linux-gnu", "-g", "-gdwarf-4"]],
        "elf",
    )
}

/// Parse the fixture archive, or `None` if it could not be built.
fn parse_fixture() -> Option<(FirmwareInfo, PathBuf)> {
    let (a_path, dir) = build_fixture_archive()?;
    let fw = FirmwareInfo::from_archive(&a_path)
        .unwrap_or_else(|e| panic!("from_archive failed on a valid fixture archive: {e}"));
    Some((fw, dir))
}

/// The enum is recovered with its variants, values, and `_COUNT` convention.
#[test]
fn enum_variants_values_and_count() {
    let (fw, _dir) = match parse_fixture() {
        Some(v) => v,
        None => return,
    };

    // enum_channel resolves a variant to its integer value.
    assert_eq!(fw.enum_channel("HAL_GPIO_B"), 1);
    assert_eq!(fw.enum_channel("HAL_GPIO_A"), 0);

    // channel_count follows the `_COUNT` suffix convention.
    assert_eq!(fw.channel_count("HAL_GPIO_channel_E"), 2);

    // All three enumerators are recovered (A, B, _COUNT).
    assert_eq!(fw.enum_variants("HAL_GPIO_channel_E").len(), 3);

    // has_* probes: present → true, absent → false.
    assert!(fw.has_enum_type("HAL_GPIO_channel_E"));
    assert!(!fw.has_enum_type("Nonexistent_E"));
    assert!(fw.has_enum_variant("HAL_GPIO_B"));
    assert!(!fw.has_enum_variant("HAL_GPIO_NOPE"));

    // try_* equivalents.
    assert_eq!(fw.try_enum_channel("HAL_GPIO_B"), Some(1));
    assert_eq!(fw.try_enum_channel("HAL_GPIO_NOPE"), None);
    assert_eq!(fw.try_channel_count("HAL_GPIO_channel_E"), Some(2));
}

/// Struct layouts are recovered, including the nested struct and primitive
/// field types (signed int, float, double).
#[test]
fn struct_layout_and_primitive_field_types() {
    let (fw, _dir) = match parse_fixture() {
        Some(v) => v,
        None => return,
    };

    // Both the top-level and the nested struct are present with non-zero size.
    let data = fw.struct_info("data_S");
    assert!(data.byte_size > 0, "data_S must have a non-zero size");
    let channel = fw.struct_info("channel_S");
    assert!(channel.byte_size > 0, "channel_S must have a non-zero size");
    assert!(fw.try_struct_info("channel_S").is_some());
    assert!(fw.try_struct_info("Nonexistent_S").is_none());

    // int32_t state → signed 4-byte base type.
    match fw.field_type("channel_S", "state") {
        TypeInfo::Base {
            signed, byte_size, ..
        } => {
            assert!(*signed, "int32_t must be signed");
            assert_eq!(*byte_size, 4);
        }
        other => panic!("state should be a signed 4-byte Base, got {other:?}"),
    }

    // float position_mm → 4-byte Float.
    match fw.field_type("channel_S", "position_mm") {
        TypeInfo::Float { byte_size, .. } => assert_eq!(*byte_size, 4),
        other => panic!("position_mm should be Float{{4}}, got {other:?}"),
    }

    // double load_n → 8-byte Float.
    match fw.field_type("channel_S", "load_n") {
        TypeInfo::Float { byte_size, .. } => assert_eq!(*byte_size, 8),
        other => panic!("load_n should be Float{{8}}, got {other:?}"),
    }
}

/// Bitfield members are recovered as [`TypeInfo::Bitfield`].
#[test]
fn bitfield_field_is_recovered() {
    let (fw, _dir) = match parse_fixture() {
        Some(v) => v,
        None => return,
    };

    // Descend into the nested flags_S struct to its `running` bitfield member.
    match fw.field_type("channel_S", "flags.running") {
        TypeInfo::Bitfield {
            bit_size,
            storage_size,
            ..
        } => {
            assert_eq!(*bit_size, 1, "running is a 1-bit field");
            assert!(*storage_size >= 1);
        }
        other => panic!("flags.running should be a Bitfield, got {other:?}"),
    }

    // flags_S itself is stored among structs, reachable as a struct field.
    match fw.field_type("channel_S", "flags") {
        TypeInfo::Struct { type_name, .. } => assert_eq!(type_name, "flags_S"),
        other => panic!("flags should be a Struct field, got {other:?}"),
    }
}

/// A union is stored among structs and all its members sit at offset 0.
#[test]
fn union_members_share_offset_zero() {
    let (fw, _dir) = match parse_fixture() {
        Some(v) => v,
        None => return,
    };

    // conv_U is stored alongside structs.
    assert!(
        fw.try_struct_info("conv_U").is_some(),
        "union conv_U should be stored among structs"
    );

    // Both union members resolve to the same offset within channel_S.
    let raw = fw.field_offset("channel_S", "conv.raw");
    let as_f = fw.field_offset("channel_S", "conv.as_f");
    assert_eq!(
        raw, as_f,
        "union members must share offset 0 within the union"
    );

    // conv itself is a Union field.
    match fw.field_type("channel_S", "conv") {
        TypeInfo::Union { type_name, .. } => assert_eq!(type_name, "conv_U"),
        other => panic!("conv should be a Union field, got {other:?}"),
    }
}

/// Array element offsets advance by exactly the element's byte size, and
/// out-of-bounds indices resolve to `None`.
#[test]
fn array_element_offsets_are_relative_and_bounded() {
    let (fw, _dir) = match parse_fixture() {
        Some(v) => v,
        None => return,
    };

    let elem_size = fw.struct_info("channel_S").byte_size;
    let base = fw.field_offset("data_S", "channels[0].state");
    let one = fw.field_offset("data_S", "channels[1].state");
    let two = fw.field_offset("data_S", "channels[2].state");

    // Assert the *relationship*, never alignment-dependent absolute offsets.
    assert_eq!(one, base + elem_size, "channels[1] is one element past [0]");
    assert_eq!(
        two,
        base + 2 * elem_size,
        "channels[2] is two elements past [0]"
    );

    // Out-of-bounds index (array has 3 elements) → None.
    assert_eq!(fw.try_field_offset("data_S", "channels[99].state"), None);

    // A valid in-bounds index resolves.
    assert!(fw.try_field_offset("data_S", "channels[2].state").is_some());
}

/// Top-level variables are recovered, including their resolved types.
#[test]
fn top_level_variables_are_recovered() {
    let (fw, _dir) = match parse_fixture() {
        Some(v) => v,
        None => return,
    };

    assert!(
        fw.variables.contains_key("g_data"),
        "g_data should be a variable"
    );
    assert!(
        fw.variables.contains_key("g_sel"),
        "g_sel should be a variable"
    );

    // g_data's type is the data_S struct.
    match &fw.variables.get("g_data").unwrap().type_info {
        TypeInfo::Struct { type_name, .. } => assert_eq!(type_name, "data_S"),
        other => panic!("g_data should be a Struct, got {other:?}"),
    }

    // g_sel's type resolves through the typedef to the enum.
    match &fw.variables.get("g_sel").unwrap().type_info {
        TypeInfo::Enum { type_name, .. } => assert_eq!(type_name, "HAL_GPIO_channel_E"),
        other => panic!("g_sel should be an Enum, got {other:?}"),
    }
}

/// Parsing junk bytes (not an `ar` archive) yields an `Err`, not a panic.
#[test]
fn non_archive_bytes_yield_err() {
    let dir = std::env::temp_dir().join(format!(
        "embsim_dwarf_junk_{}_{}",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_nanos())
            .unwrap_or(0)
    ));
    std::fs::create_dir_all(&dir).expect("create temp dir");
    let junk = dir.join("not_an_archive.a");
    std::fs::write(
        &junk,
        b"this is definitely not a valid ar archive\x00\x01\x02",
    )
    .expect("write junk file");

    let result = FirmwareInfo::from_archive(&junk);
    assert!(result.is_err(), "junk bytes must not parse as an archive");

    let _ = std::fs::remove_dir_all(&dir);
}

/// Reading a nonexistent path yields an `Err` (the IO branch of the error type).
#[test]
fn missing_file_yields_err() {
    let missing = Path::new("/this/path/should/not/exist/embsim_xyz.a");
    let result = FirmwareInfo::from_archive(missing);
    assert!(result.is_err(), "a missing file must produce an Err");
}

/// `from_archive_with` honors a custom `count_suffix` while still parsing the
/// full archive, and the default `_COUNT` suffix is overridden.
#[test]
fn from_archive_with_custom_options() {
    let (a_path, dir) = match build_fixture_archive() {
        Some(v) => v,
        None => return,
    };

    // A non-`_COUNT` suffix that no variant matches → channel_count finds nothing.
    let opts = ParseOptions {
        pointer_size: 4,
        count_suffix: "_TOTAL".to_string(),
    };
    let fw = FirmwareInfo::from_archive_with(&a_path, &opts)
        .expect("from_archive_with should parse a valid archive");

    // Structs/enums still parsed regardless of the suffix override.
    assert!(fw.has_enum_type("HAL_GPIO_channel_E"));
    assert!(fw.try_struct_info("channel_S").is_some());

    // The custom suffix is recorded and used; no variant ends with `_TOTAL`.
    assert_eq!(fw.count_suffix, "_TOTAL");
    assert_eq!(fw.try_channel_count("HAL_GPIO_channel_E"), None);

    // And the default-suffix parse still finds the `_COUNT` variant.
    let default =
        FirmwareInfo::from_archive_with(&a_path, &ParseOptions::default()).expect("default parse");
    assert_eq!(default.try_channel_count("HAL_GPIO_channel_E"), Some(2));

    let _ = std::fs::remove_dir_all(&dir);
}

/// ELF relocatable objects store DWARF cross-section references (e.g.
/// `DW_FORM_strp` offsets into `.debug_str`) as relocations over zeroed
/// section bytes — this is what Linux CI's toolchain produces for
/// `libfirmware.a`. Without applying relocations, every name resolves to
/// `.debug_str` offset 0 and all lookups fail. Mach-O resolves these offsets
/// at assembly time, so host-target coverage on macOS can't catch it; this
/// test cross-compiles the same fixture to an ELF x86_64 object on any host.
#[test]
fn elf_relocatable_archive_resolves_names_via_relocations() {
    let (a_path, dir) = match build_elf_fixture_archive() {
        Some(v) => v,
        None => return,
    };
    let fw = FirmwareInfo::from_archive(&a_path)
        .unwrap_or_else(|e| panic!("from_archive failed on a valid ELF fixture archive: {e}"));

    // Enum variant names come from `.debug_str` via relocated strp offsets.
    assert_eq!(fw.try_enum_channel("HAL_GPIO_A"), Some(0));
    assert_eq!(fw.try_enum_channel("HAL_GPIO_B"), Some(1));
    assert!(fw.has_enum_type("HAL_GPIO_channel_E"));
    assert_eq!(fw.try_channel_count("HAL_GPIO_channel_E"), Some(2));

    // Struct layout: offsets are deterministic for the pinned x86_64 SysV ABI.
    let channel = fw.struct_info("channel_S");
    assert!(channel.byte_size > 0, "channel_S must have a non-zero size");
    assert_eq!(fw.field_offset("channel_S", "state"), 0);
    assert_eq!(fw.field_offset("channel_S", "position_mm"), 4);
    assert_eq!(fw.field_offset("channel_S", "load_n"), 8);

    // Variables are recovered with resolved struct types.
    assert!(
        fw.variables.contains_key("g_data"),
        "g_data should be a variable"
    );
    match &fw.variables.get("g_data").unwrap().type_info {
        TypeInfo::Struct { type_name, .. } => assert_eq!(type_name, "data_S"),
        other => panic!("g_data should be a Struct, got {other:?}"),
    }

    let _ = std::fs::remove_dir_all(&dir);
}
