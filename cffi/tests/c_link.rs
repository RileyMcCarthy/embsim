//! Does real C compile, link and run against the static library?
//!
//! `c_abi.rs` proves the shim BEHAVES; this proves it is reachable at all. They
//! are different failures: a missing symbol, a `#[no_mangle]` that got mangled,
//! a header that disagrees with the Rust signature, or a platform link
//! dependency the consumer has to know about. None of those show up from Rust,
//! and all of them show up as a wall of linker output in the C host — which for
//! us is a QEMU target build, where the feedback loop is minutes long.
//!
//! So this compiles `embsim.h` against `libembsim_cffi.a` with the system C
//! compiler and runs the result. It is skipped, loudly, where there is no `cc`.

use std::path::{Path, PathBuf};
use std::process::Command;

/// The C program: the P2 boot ROM's own opening move — select the part, issue
/// `$03 READ DATA` at address 0, and clock four bytes back.
const PROGRAM: &str = r#"
#include "embsim.h"
#include <stdio.h>
#include <string.h>

static void send(EmbsimSpiFlash *f, uint8_t byte) {
    for (int i = 7; i >= 0; i--) {
        bool bit = (byte >> i) & 1;
        embsim_spi_flash_clock(f, false, bit);
        embsim_spi_flash_clock(f, true, bit);
    }
}

/* Sample AFTER the rising edge: the part takes MOSI and presents its own next
 * bit on that same edge. */
static uint8_t recv(EmbsimSpiFlash *f) {
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        embsim_spi_flash_clock(f, false, true);
        embsim_spi_flash_clock(f, true, true);
        byte = (uint8_t)((byte << 1) | (embsim_spi_flash_miso(f) ? 1 : 0));
    }
    return byte;
}

int main(void) {
    uint8_t image[4] = { 0xDE, 0xAD, 0xBE, 0xEF };
    EmbsimSpiFlash *f = embsim_spi_flash_with_image(4096, image, sizeof image);
    if (!f) { fprintf(stderr, "construct failed\n"); return 1; }
    if (!embsim_spi_flash_present(f)) { fprintf(stderr, "not present\n"); return 1; }

    embsim_spi_flash_set_selected(f, true);
    send(f, 0x03); send(f, 0x00); send(f, 0x00); send(f, 0x00);
    uint8_t got[4];
    for (int i = 0; i < 4; i++) got[i] = recv(f);
    embsim_spi_flash_set_selected(f, false);

    if (memcmp(got, image, 4) != 0) {
        fprintf(stderr, "got %02X %02X %02X %02X\n", got[0], got[1], got[2], got[3]);
        return 1;
    }
    uint32_t where = 0xFFFFFFFF;
    if (embsim_spi_flash_reads(f, &where, 1) != 1 || where != 0) {
        fprintf(stderr, "reads wrong: %u\n", where);
        return 1;
    }
    embsim_spi_flash_free(f);
    printf("OK\n");
    return 0;
}
"#;

/// The directory cargo put this test binary's artifacts in — `target/<profile>`.
fn artifact_dir() -> PathBuf {
    let exe = std::env::current_exe().expect("the test binary has a path");
    // .../target/<profile>/deps/<name>-<hash>
    exe.parent()
        .and_then(Path::parent)
        .expect("target/<profile>")
        .to_path_buf()
}

#[test]
fn a_c_program_links_against_the_static_library_and_runs() {
    let cc = std::env::var("CC").unwrap_or_else(|_| "cc".to_string());
    if Command::new(&cc).arg("--version").output().is_err() {
        eprintln!("\n*** SKIPPED: no `{cc}` to compile with. This test asserted NOTHING.\n");
        return;
    }

    let lib = artifact_dir().join("libembsim_cffi.a");
    assert!(
        lib.exists(),
        "the static library must be built before this test can link it: {}\n\
         (cargo builds it for this package, so a missing file here means the \
         crate-type no longer includes `staticlib`)",
        lib.display()
    );

    let dir = artifact_dir().join("embsim-cffi-c-link");
    std::fs::create_dir_all(&dir).expect("scratch dir");
    let src = dir.join("main.c");
    let bin = dir.join("main");
    std::fs::write(&src, PROGRAM).expect("write the C program");

    let include = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("include");
    let mut cmd = Command::new(&cc);
    cmd.arg("-std=c11")
        .arg("-Wall")
        .arg("-Werror")
        .arg("-I")
        .arg(&include)
        .arg(&src)
        .arg(&lib)
        .arg("-o")
        .arg(&bin);
    // A Rust staticlib carries the std runtime's own dependencies; these are
    // what a C host has to add, and naming them here is how a QEMU build knows
    // what to put in its link line.
    if cfg!(target_os = "macos") {
        cmd.args(["-framework", "CoreFoundation", "-framework", "Security"]);
    } else {
        cmd.args(["-lpthread", "-ldl", "-lm"]);
    }

    let out = cmd.output().expect("run the compiler");
    assert!(
        out.status.success(),
        "the C program must compile and link:\n{}\n{}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    );

    let run = Command::new(&bin).output().expect("run the C program");
    assert!(
        run.status.success(),
        "the C program must run clean:\n{}\n{}",
        String::from_utf8_lossy(&run.stdout),
        String::from_utf8_lossy(&run.stderr)
    );
    assert_eq!(
        String::from_utf8_lossy(&run.stdout).trim(),
        "OK",
        "and read back what it wrote"
    );
}
