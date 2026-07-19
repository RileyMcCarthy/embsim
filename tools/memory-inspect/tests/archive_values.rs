//! End-to-end coverage for the archive value-read path ([`ArchiveValueReader`]):
//! compiles a small C fixture with initialized, config-table-shaped globals
//! into a static `ar` archive at test time, then reads the values back and
//! asserts them field-by-field. Follows the toolchain-skip conventions of
//! `parse_fixture.rs` (no C compiler / archiver → print SKIP and pass).
//!
//! One test additionally cross-compiles the same fixture to an **ELF x86_64
//! relocatable object** (clang `--target=`, archive bytes written directly by
//! the test) so the ELF leg of the reader is exercised on every host, not
//! just Linux — same rationale and skip discrimination as the relocation test
//! in `parse_fixture.rs`.
//!
//! The `#[ignore]`d test at the bottom reads the *real* MaD firmware archive
//! (the reference consumer's `libfirmware.a`); see its doc comment for why it
//! cannot run in this repo's CI.

use embsim_memory_inspect::{hal_tables, ArchiveValueReader, FirmwareInfo, Value, ValueReadError};
use rstest::rstest;
use std::path::{Path, PathBuf};
use std::process::Command;

/// The C fixture: an initialized array of config structs shaped like a HAL
/// serial wiring table (enum pins, int baud, enum type, bool flag), plus
/// scalar globals of each shape, a zero-initialized array (`.bss`), and a
/// function (defined symbol with *no* DWARF variable entry).
const FIXTURE_C: &str = r#"
#include <stdint.h>
#include <stdbool.h>
typedef enum { PIN_P0 = 0, PIN_P2 = 2, PIN_P53 = 53, PIN_P55 = 55, PIN_COUNT = 64 } pin_E;
typedef enum { SER_TYPE_HW = 0, SER_TYPE_BUILTIN = 1, SER_TYPE_COUNT = 2 } ser_type_E;
typedef struct { pin_E rx; pin_E tx; int32_t baud; ser_type_E type; bool lsb; } ser_cfg_S;
const ser_cfg_S ser_config[2] = {
    {PIN_P0, PIN_P2, 115200, SER_TYPE_HW, false},
    {PIN_P53, PIN_P55, 2000000, SER_TYPE_BUILTIN, true},
};
const int32_t answer = -42;
const uint8_t small_u8 = 200;
const bool truthy = true;
const float ratio = 1.5f;
int32_t zeroed[3];
int fixture_fn(int x) { return x + 1; }
"#;

/// A second C fixture shaped exactly like the reference consumer's Phase-0
/// HAL config tables (MaD's `src/HAL/Config/*.c`): the four wiring tables
/// under their default symbol names, with the extra per-channel fields the
/// consumer carries (`type`/`LSB`, presets, step timing) that the
/// `hal_tables` decoders must ignore.
const HAL_TABLES_C: &str = r#"
#include <stdint.h>
#include <stdbool.h>
typedef enum {
    HW_PIN_P0 = 0, HW_PIN_P2 = 2, HW_PIN_P6 = 6, HW_PIN_P8 = 8, HW_PIN_P9 = 9,
    HW_PIN_P10 = 10, HW_PIN_P16 = 16, HW_PIN_P53 = 53, HW_PIN_P55 = 55,
    HW_PIN_COUNT = 64
} HW_pin_E;
typedef enum { HAL_SERIAL_TYPE_HW = 0, HAL_SERIAL_TYPE_COUNT = 1 } HAL_serial_type_E;
typedef struct {
    HW_pin_E rx;
    HW_pin_E tx;
    int32_t baud;
    HAL_serial_type_E type;
    bool LSB;
} HAL_serial_channelConfig_S;
const HAL_serial_channelConfig_S HAL_serial_channelConfig[2] = {
    {HW_PIN_P0, HW_PIN_P2, 115200, HAL_SERIAL_TYPE_HW, false},
    {HW_PIN_P53, HW_PIN_P55, 2000000, HAL_SERIAL_TYPE_HW, false},
};
typedef struct {
    HW_pin_E pin;
    bool activeLow;
} HAL_GPIO_channelConfig_S;
const HAL_GPIO_channelConfig_S HAL_GPIO_channelConfig[2] = {
    {HW_PIN_P6, false},
    {HW_PIN_P16, true},
};
typedef struct {
    int32_t preset;
    int32_t lo;
    int32_t hi;
    HW_pin_E pinA;
    HW_pin_E pinB;
} HAL_encoder_config_S;
const HAL_encoder_config_S HAL_encoder_config[1] = {
    {0, -1000000, 1000000, HW_PIN_P9, HW_PIN_P10},
};
typedef struct {
    int32_t maxHardwareClockCyclePerStep;
    HW_pin_E pin;
} HAL_pulseOut_channelConfig_S;
const HAL_pulseOut_channelConfig_S HAL_pulseOut_channelConfig[1] = {
    {131070, HW_PIN_P8},
};
"#;

// ============================================================
// Fixture build helpers (same conventions as parse_fixture.rs)
// ============================================================

/// Returns the first available C compiler binary name, or `None`. `clang`
/// first — the parser targets clang-emitted DWARF.
fn find_compiler() -> Option<&'static str> {
    ["clang", "cc", "gcc"]
        .into_iter()
        .find(|cc| Command::new(cc).arg("--version").output().is_ok())
}

/// Create a unique temp dir namespaced by `tag` + pid + a nonce + a
/// process-wide sequence number (tests run in parallel threads; a wall-clock
/// nonce alone can collide within one process), or `None` (after printing a
/// skip message) if it can't be created.
fn make_temp_dir(tag: &str) -> Option<PathBuf> {
    static DIR_SEQ: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
    let nonce = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_nanos())
        .unwrap_or(0);
    let seq = DIR_SEQ.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    let dir = std::env::temp_dir().join(format!(
        "embsim_values_{}_{}_{}_{}",
        tag,
        std::process::id(),
        nonce,
        seq
    ));
    if std::fs::create_dir_all(&dir).is_err() {
        eprintln!("SKIP: could not create temp dir {}", dir.display());
        return None;
    }
    Some(dir)
}

/// Compile `source` with `cc` and `flags` (`-c`/`-o` appended) into
/// `dir/<name>.o`. Returns the object path, or `None` (after printing a skip
/// message) if any step fails.
fn compile_object(
    cc: &str,
    dir: &Path,
    name: &str,
    source: &str,
    flags: &[&str],
) -> Option<PathBuf> {
    let c_path = dir.join(format!("{name}.c"));
    let o_path = dir.join(format!("{name}.o"));

    if std::fs::write(&c_path, source).is_err() {
        eprintln!("SKIP: could not write {name}.c");
        return None;
    }

    let mut args: Vec<&str> = flags.to_vec();
    let c = c_path.to_str()?;
    let o = o_path.to_str()?;
    // The reader's documented contract requires real definitions (tentative
    // definitions become unindexable common symbols). Modern compilers make
    // -fno-common the default, but the fixture pins it so the tests don't
    // depend on the host toolchain's default (older Apple clang differs).
    args.extend_from_slice(&["-fno-common", "-c", c, "-o", o]);
    let compiled = Command::new(cc)
        .args(&args)
        .output()
        .map(|out| out.status.success() && o_path.exists())
        .unwrap_or(false);
    if !compiled {
        eprintln!("SKIP: failed to compile {name}.c with {cc}");
        return None;
    }

    Some(o_path)
}

/// Write [`FIXTURE_C`] into a fresh temp dir and compile it with `cc` and
/// `flags`. Returns the object path plus the dir, or `None` (after printing
/// a skip message) if any step fails.
fn compile_fixture_object(cc: &str, flags: &[&str], tag: &str) -> Option<(PathBuf, PathBuf)> {
    let dir = make_temp_dir(tag)?;
    let o_path = compile_object(cc, &dir, "fixture", FIXTURE_C, flags)?;
    Some((o_path, dir))
}

/// Archive `o_paths` (in order) into `a_path` with the host archiver
/// (`ar`/`llvm-ar`).
fn archive_objects(a_path: &Path, o_paths: &[&Path]) -> bool {
    let Some(a) = a_path.to_str() else {
        eprintln!("SKIP: non-UTF-8 temp path");
        return false;
    };
    let Some(objects) = o_paths
        .iter()
        .map(|o| o.to_str())
        .collect::<Option<Vec<_>>>()
    else {
        eprintln!("SKIP: non-UTF-8 temp path");
        return false;
    };
    for ar in ["ar", "llvm-ar"] {
        let _ = std::fs::remove_file(a_path);
        let archived = Command::new(ar)
            .args(["rcs", a])
            .args(&objects)
            .output()
            .map(|out| out.status.success() && a_path.exists())
            .unwrap_or(false);
        if archived {
            return true;
        }
    }
    eprintln!("SKIP: failed to archive {objects:?} with ar or llvm-ar");
    false
}

/// Write a minimal single-member `ar` archive byte-for-byte (no external
/// archiver) — same format notes as `parse_fixture.rs`.
fn write_ar_archive(a_path: &Path, member_name: &str, member_data: &[u8]) -> std::io::Result<()> {
    assert!(
        member_name.len() <= 15,
        "member name must fit the 16-byte field"
    );
    let header = format!(
        "{:<16}{:<12}{:<6}{:<6}{:<8}{:<10}\x60\n",
        format!("{member_name}/"),
        0,
        0,
        0,
        "100644",
        member_data.len(),
    );
    assert_eq!(
        header.len(),
        60,
        "ar member header must be exactly 60 bytes"
    );

    let mut bytes = Vec::with_capacity(8 + 60 + member_data.len() + 1);
    bytes.extend_from_slice(b"!<arch>\n");
    bytes.extend_from_slice(header.as_bytes());
    bytes.extend_from_slice(member_data);
    if !member_data.len().is_multiple_of(2) {
        bytes.push(b'\n');
    }
    std::fs::write(a_path, bytes)
}

/// Build the fixture archive for the **host** target with DWARF v4 debug
/// info (plain `-g` fallback).
fn build_host_fixture_archive() -> Option<(PathBuf, PathBuf)> {
    let cc = match find_compiler() {
        Some(c) => c,
        None => {
            eprintln!("SKIP: no C compiler (cc/clang/gcc) available");
            return None;
        }
    };
    for flags in [&["-g", "-gdwarf-4"][..], &["-g"][..]] {
        if let Some((o_path, dir)) = compile_fixture_object(cc, flags, "host") {
            let a_path = dir.join("libfixture.a");
            if archive_objects(&a_path, &[&o_path]) {
                return Some((a_path, dir));
            }
            return None;
        }
    }
    None
}

/// Build the fixture archive as an **ELF x86_64 relocatable object** on any
/// host (clang cross-target, no linker needed; archive bytes written
/// directly so host-archiver variability can't corrupt the fixture).
fn build_elf_fixture_archive() -> Option<(PathBuf, PathBuf)> {
    if Command::new("clang").arg("--version").output().is_err() {
        eprintln!("SKIP: clang not available for the ELF cross-compile");
        return None;
    }
    let (o_path, dir) = compile_fixture_object(
        "clang",
        &["--target=x86_64-unknown-linux-gnu", "-g", "-gdwarf-4"],
        "elf",
    )?;
    let a_path = dir.join("libfixture.a");
    let object_bytes = match std::fs::read(&o_path) {
        Ok(bytes) => bytes,
        Err(e) => {
            eprintln!("SKIP: could not read back {}: {e}", o_path.display());
            return None;
        }
    };
    if let Err(e) = write_ar_archive(&a_path, "fixture.o", &object_bytes) {
        eprintln!("SKIP: could not write {}: {e}", a_path.display());
        return None;
    }
    Some((a_path, dir))
}

// ============================================================
// Shared assertions
// ============================================================

/// Assert every initialized value in the fixture decodes correctly from
/// `a_path`. Factored so the host and ELF legs check identical behavior.
fn assert_fixture_values(a_path: &Path) {
    let fw = FirmwareInfo::from_archive(a_path)
        .unwrap_or_else(|e| panic!("from_archive failed on a valid fixture archive: {e}"));
    let values = ArchiveValueReader::from_archive(a_path)
        .unwrap_or_else(|e| panic!("ArchiveValueReader failed on a valid fixture archive: {e}"));

    // The CI-probe path: the table symbol is present in the archive.
    assert!(
        values.has_symbol("ser_config"),
        "ser_config must be indexed"
    );
    assert!(
        !values.has_symbol("definitely_not_a_symbol"),
        "absent symbols are not indexed"
    );

    // const ser_cfg_S ser_config[2] — the consumer shape: array of structs
    // with enum / int / bool fields.
    let cfg = values
        .read_value(&fw, "ser_config")
        .expect("read ser_config");
    let elems = cfg.elements().expect("ser_config decodes as an array");
    assert_eq!(elems.len(), 2, "ser_config has 2 elements");

    let e0 = &elems[0];
    assert_eq!(e0.field("rx").unwrap().as_i64(), Some(0));
    assert_eq!(e0.field("tx").unwrap().as_i64(), Some(2));
    assert_eq!(e0.field("baud").unwrap(), &Value::Int(115_200));
    assert_eq!(e0.field("type").unwrap().as_i64(), Some(0));
    assert_eq!(e0.field("lsb").unwrap().as_bool(), Some(false));

    let e1 = &elems[1];
    assert_eq!(e1.field("rx").unwrap().as_i64(), Some(53));
    assert_eq!(e1.field("tx").unwrap().as_i64(), Some(55));
    assert_eq!(e1.field("baud").unwrap().as_i64(), Some(2_000_000));
    assert_eq!(e1.field("type").unwrap().as_i64(), Some(1));
    assert_eq!(e1.field("lsb").unwrap().as_bool(), Some(true));

    // Enum fields carry their DWARF type name (the pin-facade key).
    match e1.field("rx").unwrap() {
        Value::Enum { type_name, value } => {
            assert_eq!(type_name, "pin_E");
            assert_eq!(*value, 53);
        }
        other => panic!("rx should decode as an Enum, got {other:?}"),
    }

    // Scalar globals of each shape.
    assert_eq!(
        values.read_value(&fw, "answer").expect("answer"),
        Value::Int(-42)
    );
    assert_eq!(
        values.read_value(&fw, "small_u8").expect("small_u8"),
        Value::UInt(200)
    );
    assert_eq!(
        values.read_value(&fw, "truthy").expect("truthy"),
        Value::Bool(true)
    );
    assert_eq!(
        values.read_value(&fw, "ratio").expect("ratio"),
        Value::Float(1.5)
    );

    // Zero-initialized array (.bss under the default -fno-common) reads as
    // zeros, not an error.
    let zeroed = values.read_value(&fw, "zeroed").expect("zeroed");
    let elems = zeroed.elements().expect("zeroed decodes as an array");
    assert_eq!(elems.len(), 3);
    for (i, elem) in elems.iter().enumerate() {
        assert_eq!(elem.as_i64(), Some(0), "zeroed[{i}] must decode as 0");
    }
}

// ============================================================
// Tests
// ============================================================

/// Host-target archive: every fixture value decodes correctly.
#[rstest]
fn host_archive_reads_initialized_values() {
    let (a_path, dir) = match build_host_fixture_archive() {
        Some(v) => v,
        None => return,
    };
    assert_fixture_values(&a_path);
    let _ = std::fs::remove_dir_all(&dir);
}

/// The `hal_tables` helpers decode the four HAL-shaped config tables from a
/// compiled archive under their default (reference-consumer) symbol names,
/// ignoring the extra per-channel fields the consumer carries.
#[rstest]
fn hal_tables_decode_from_a_compiled_fixture_archive() {
    let cc = match find_compiler() {
        Some(c) => c,
        None => {
            eprintln!("SKIP: no C compiler (cc/clang/gcc) available");
            return;
        }
    };
    let Some(dir) = make_temp_dir("hal") else {
        return;
    };
    let Some(o_path) = compile_object(cc, &dir, "hal", HAL_TABLES_C, &["-g", "-gdwarf-4"]) else {
        return;
    };
    let a_path = dir.join("libhal.a");
    if !archive_objects(&a_path, &[&o_path]) {
        return;
    }

    let fw = FirmwareInfo::from_archive(&a_path).expect("parse HAL fixture archive");
    let values = ArchiveValueReader::from_archive(&a_path).expect("index HAL fixture archive");

    let serial =
        hal_tables::read_serial_table(&values, &fw, hal_tables::DEFAULT_SERIAL_TABLE_SYMBOL)
            .expect("serial table decodes");
    assert_eq!(
        serial,
        vec![
            hal_tables::SerialChannelConfig {
                rx_pin: 0,
                tx_pin: 2,
                baud: 115_200
            },
            hal_tables::SerialChannelConfig {
                rx_pin: 53,
                tx_pin: 55,
                baud: 2_000_000
            },
        ]
    );

    let gpio = hal_tables::read_gpio_table(&values, &fw, hal_tables::DEFAULT_GPIO_TABLE_SYMBOL)
        .expect("GPIO table decodes");
    assert_eq!(
        gpio,
        vec![
            hal_tables::GpioChannelConfig {
                pin: 6,
                active_low: false
            },
            hal_tables::GpioChannelConfig {
                pin: 16,
                active_low: true
            },
        ]
    );

    let encoder =
        hal_tables::read_encoder_table(&values, &fw, hal_tables::DEFAULT_ENCODER_TABLE_SYMBOL)
            .expect("encoder table decodes");
    assert_eq!(
        encoder,
        vec![hal_tables::EncoderConfig {
            pin_a: 9,
            pin_b: 10
        }]
    );

    let pulse =
        hal_tables::read_pulse_out_table(&values, &fw, hal_tables::DEFAULT_PULSE_OUT_TABLE_SYMBOL)
            .expect("pulse-out table decodes");
    assert_eq!(pulse, vec![hal_tables::PulseOutConfig { pin: 8 }]);

    // A missing table symbol surfaces as the underlying read error.
    let err = hal_tables::read_serial_table(&values, &fw, "HAL_no_such_table").unwrap_err();
    assert!(
        matches!(
            err,
            hal_tables::HalTableError::Read(ValueReadError::SymbolNotFound { .. })
        ),
        "got {err:?}"
    );

    let _ = std::fs::remove_dir_all(&dir);
}

/// Missing symbols and symbols without DWARF variable entries produce the
/// matching typed errors, not panics.
#[rstest]
fn error_variants_for_missing_symbol_and_type_info() {
    let (a_path, dir) = match build_host_fixture_archive() {
        Some(v) => v,
        None => return,
    };
    let fw = FirmwareInfo::from_archive(&a_path).expect("parse fixture");
    let values = ArchiveValueReader::from_archive(&a_path).expect("index fixture");

    // Nothing defines this symbol.
    let err = values.read_value(&fw, "no_such_symbol").unwrap_err();
    assert!(
        matches!(err, ValueReadError::SymbolNotFound { .. }),
        "got {err:?}"
    );

    // `fixture_fn` is a defined symbol (a function) but has no DWARF
    // *variable* entry — the type-info lookup must fail cleanly.
    assert!(
        values.has_symbol("fixture_fn"),
        "functions are still indexed symbols"
    );
    let err = values.read_value(&fw, "fixture_fn").unwrap_err();
    assert!(
        matches!(err, ValueReadError::MissingTypeInfo { .. }),
        "got {err:?}"
    );

    let _ = std::fs::remove_dir_all(&dir);
}

/// Two members define the same symbol, one weak and one strong: the strong
/// (non-weak) definition must win regardless of member order — a weak
/// definition in an earlier member must not shadow the real one (linker
/// symbol-resolution semantics).
#[rstest]
fn strong_definition_wins_over_weak_regardless_of_member_order() {
    let cc = match find_compiler() {
        Some(c) => c,
        None => {
            eprintln!("SKIP: no C compiler (cc/clang/gcc) available");
            return;
        }
    };
    let weak_src = "#include <stdint.h>\n__attribute__((weak)) int32_t dual_def = 11;\n";
    let strong_src = "#include <stdint.h>\nint32_t dual_def = 22;\n";

    for (tag, weak_first) in [("weak_first", true), ("strong_first", false)] {
        let Some(dir) = make_temp_dir(tag) else {
            return;
        };
        let flags = &["-g", "-gdwarf-4"];
        let (Some(weak_o), Some(strong_o)) = (
            compile_object(cc, &dir, "weak", weak_src, flags),
            compile_object(cc, &dir, "strong", strong_src, flags),
        ) else {
            return;
        };

        let a_path = dir.join("libdual.a");
        let members: [&Path; 2] = if weak_first {
            [&weak_o, &strong_o]
        } else {
            [&strong_o, &weak_o]
        };
        if !archive_objects(&a_path, &members) {
            return;
        }

        let fw = FirmwareInfo::from_archive(&a_path).expect("parse dual archive");
        let values = ArchiveValueReader::from_archive(&a_path).expect("index dual archive");
        assert_eq!(
            values.read_value(&fw, "dual_def").expect("read dual_def"),
            Value::Int(22),
            "strong definition must win with the {tag} member order"
        );
        let _ = std::fs::remove_dir_all(&dir);
    }
}

/// ELF relocatable member: the value-read path works on ELF objects too
/// (section lookup, no Mach-O underscore stripping, endianness plumbed).
/// Skip discrimination mirrors `parse_fixture.rs`: an *empty* DWARF parse on
/// a non-Linux host is a toolchain limitation (loud skip); on Linux — the
/// platform this leg exists for — it fails strictly.
#[rstest]
fn elf_archive_reads_initialized_values() {
    let (a_path, dir) = match build_elf_fixture_archive() {
        Some(v) => v,
        None => return,
    };
    let fw = FirmwareInfo::from_archive(&a_path)
        .unwrap_or_else(|e| panic!("from_archive failed on a valid ELF fixture archive: {e}"));
    if fw.enums.is_empty() && fw.structs.is_empty() && fw.variables.is_empty() {
        if cfg!(target_os = "linux") {
            panic!("ELF fixture archive parsed completely empty on Linux — failing strictly");
        }
        eprintln!(
            "SKIP: ELF cross-compiled fixture parsed completely empty on a non-Linux host \
             (host toolchain limitation; see parse_fixture.rs for the discrimination rationale)"
        );
        let _ = std::fs::remove_dir_all(&dir);
        return;
    }
    assert_fixture_values(&a_path);
    let _ = std::fs::remove_dir_all(&dir);
}

// ============================================================
// Real-firmware integration (ignored by default)
// ============================================================

/// Locate the MaD `libfirmware.a`: `EMBSIM_MAD_FIRMWARE_ARCHIVE` override,
/// else the well-known build path relative to this crate inside a MaD
/// checkout (embsim lives at `<MaD>/SIL/embsim`).
fn mad_archive_path() -> PathBuf {
    if let Ok(p) = std::env::var("EMBSIM_MAD_FIRMWARE_ARCHIVE") {
        return PathBuf::from(p);
    }
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../../../Firmware/MaDCore/.pio/build/native_emulator/libfirmware.a")
}

/// Reads the real MaD HAL wiring/config tables from `libfirmware.a` —
/// the board-engine pin facade's exact consumer use.
///
/// `#[ignore]` by default because the archive lives **outside this repo**:
/// embsim is consumed as a submodule at `<MaD>/SIL/embsim`, and the archive
/// is a build artifact of the consuming checkout
/// (`Firmware/MaDCore/.pio/build/native_emulator/libfirmware.a`, built with
/// `pio run -e native_emulator` and `-DENABLE_DEBUG_SERIAL=1`). This repo's
/// own CI has neither the path nor the toolchain to produce it. Run manually
/// from a MaD checkout:
///
/// ```text
/// cargo test -p embsim-memory-inspect --test archive_values -- --ignored --nocapture
/// ```
///
/// Point `EMBSIM_MAD_FIRMWARE_ARCHIVE` at the archive to run from elsewhere.
#[rstest]
#[ignore = "reads the consuming MaD repo's libfirmware.a (cross-repo build artifact)"]
fn reads_mad_hal_config_tables() {
    let path = mad_archive_path();
    assert!(
        path.exists(),
        "MaD firmware archive not found at {} — build it with \
         `pio run -e native_emulator` in Firmware/MaDCore, or set \
         EMBSIM_MAD_FIRMWARE_ARCHIVE",
        path.display()
    );

    let fw = FirmwareInfo::from_archive(&path).expect("parse MaD archive DWARF");
    let values = ArchiveValueReader::from_archive(&path).expect("index MaD archive symbols");

    // ── HAL_serial_channelConfig[HAL_SERIAL_CHANNEL_COUNT] ──
    // Debug-serial build truth: FORCE_GAUGE {rx 0, tx 2, 115200, HW},
    // MAIN {rx 53 (RPI_RX), tx 55 (RPI_TX), 2000000, HW}.
    let serial = values
        .read_value(&fw, "HAL_serial_channelConfig")
        .expect("serial table");
    println!("HAL_serial_channelConfig = {serial:#?}");
    let elems = serial.elements().expect("serial table is an array");
    assert_eq!(elems.len(), fw.channel_count("HAL_serial_channel_E"));
    assert_eq!(elems.len(), 2);

    let fg = &elems[0];
    assert_eq!(fg.field("rx").unwrap().as_i64(), Some(0));
    assert_eq!(fg.field("tx").unwrap().as_i64(), Some(2));
    assert_eq!(fg.field("baud").unwrap().as_i64(), Some(115_200));
    assert_eq!(fg.field("type").unwrap().as_i64(), Some(0));
    assert_eq!(fg.field("LSB").unwrap().as_bool(), Some(false));

    let main = &elems[1];
    assert_eq!(main.field("rx").unwrap().as_i64(), Some(53));
    assert_eq!(main.field("tx").unwrap().as_i64(), Some(55));
    assert_eq!(main.field("baud").unwrap().as_i64(), Some(2_000_000));
    assert_eq!(main.field("type").unwrap().as_i64(), Some(0));

    // Pin fields carry the HW_pin_E enum type — that's the facade key.
    match main.field("rx").unwrap() {
        Value::Enum { type_name, .. } => assert_eq!(type_name, "HW_pin_E"),
        other => panic!("rx should decode as an Enum, got {other:?}"),
    }

    // ── HAL_GPIO_channelConfig[HAL_GPIO_COUNT] ──
    let gpio = values
        .read_value(&fw, "HAL_GPIO_channelConfig")
        .expect("GPIO table");
    println!("HAL_GPIO_channelConfig = {gpio:#?}");
    let elems = gpio.elements().expect("GPIO table is an array");
    assert_eq!(elems.len(), fw.channel_count("HAL_GPIO_channel_E"));
    // Spot checks against HW_pins.h + HAL_GPIO_config.c.
    let servo_ena = &elems[fw.enum_channel("HAL_GPIO_SERVO_ENA")];
    assert_eq!(servo_ena.field("pin").unwrap().as_i64(), Some(6));
    assert_eq!(servo_ena.field("activeLow").unwrap().as_bool(), Some(false));
    let esd_upper = &elems[fw.enum_channel("HAL_GPIO_ESD_UPPER")];
    assert_eq!(esd_upper.field("pin").unwrap().as_i64(), Some(16));
    assert_eq!(esd_upper.field("activeLow").unwrap().as_bool(), Some(true));

    // ── HAL_encoder_config[HAL_ENCODER_CHANNEL_COUNT] ──
    let enc = values
        .read_value(&fw, "HAL_encoder_config")
        .expect("encoder table");
    println!("HAL_encoder_config = {enc:#?}");
    let elems = enc.elements().expect("encoder table is an array");
    assert_eq!(elems.len(), fw.channel_count("HAL_encoder_channel_E"));
    let servo = &elems[0];
    assert_eq!(servo.field("preset").unwrap().as_i64(), Some(0));
    assert_eq!(servo.field("lo").unwrap().as_i64(), Some(-1_000_000));
    assert_eq!(servo.field("hi").unwrap().as_i64(), Some(1_000_000));
    assert_eq!(servo.field("pinA").unwrap().as_i64(), Some(9));
    assert_eq!(servo.field("pinB").unwrap().as_i64(), Some(10));

    // ── HAL_pulseOut_channelConfig[HAL_PULSE_OUT_CHANNEL_COUNT] ──
    let pulse = values
        .read_value(&fw, "HAL_pulseOut_channelConfig")
        .expect("pulse-out table");
    println!("HAL_pulseOut_channelConfig = {pulse:#?}");
    let elems = pulse.elements().expect("pulse-out table is an array");
    assert_eq!(elems.len(), fw.channel_count("HAL_pulseOut_channel_E"));
    let servo = &elems[0];
    assert_eq!(
        servo
            .field("maxHardwareClockCyclePerStep")
            .unwrap()
            .as_i64(),
        Some(65_535 * 2)
    );
    assert_eq!(servo.field("pin").unwrap().as_i64(), Some(8));

    // ── hal_tables helpers over the same archive ──
    // The consumer-shaped decode layer must agree with the raw reads above:
    // this is exactly what an MCU component's pin facade consumes.
    let serial =
        hal_tables::read_serial_table(&values, &fw, hal_tables::DEFAULT_SERIAL_TABLE_SYMBOL)
            .expect("serial table decodes via hal_tables");
    println!("hal_tables serial = {serial:?}");
    assert_eq!(
        serial[0],
        hal_tables::SerialChannelConfig {
            rx_pin: 0,
            tx_pin: 2,
            baud: 115_200
        },
        "FORCE_GAUGE channel"
    );
    assert_eq!(
        serial[1],
        hal_tables::SerialChannelConfig {
            rx_pin: 53,
            tx_pin: 55,
            baud: 2_000_000
        },
        "MAIN channel"
    );

    let gpio = hal_tables::read_gpio_table(&values, &fw, hal_tables::DEFAULT_GPIO_TABLE_SYMBOL)
        .expect("GPIO table decodes via hal_tables");
    assert_eq!(gpio.len(), fw.channel_count("HAL_GPIO_channel_E"));
    assert_eq!(
        gpio[fw.enum_channel("HAL_GPIO_SERVO_ENA")],
        hal_tables::GpioChannelConfig {
            pin: 6,
            active_low: false
        }
    );
    assert_eq!(
        gpio[fw.enum_channel("HAL_GPIO_ESD_UPPER")],
        hal_tables::GpioChannelConfig {
            pin: 16,
            active_low: true
        }
    );

    let encoder =
        hal_tables::read_encoder_table(&values, &fw, hal_tables::DEFAULT_ENCODER_TABLE_SYMBOL)
            .expect("encoder table decodes via hal_tables");
    assert_eq!(
        encoder[0],
        hal_tables::EncoderConfig {
            pin_a: 9,
            pin_b: 10
        }
    );

    let pulse =
        hal_tables::read_pulse_out_table(&values, &fw, hal_tables::DEFAULT_PULSE_OUT_TABLE_SYMBOL)
            .expect("pulse-out table decodes via hal_tables");
    assert_eq!(pulse[0], hal_tables::PulseOutConfig { pin: 8 });
}
