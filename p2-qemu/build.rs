//! Link the QEMU Propeller 2 target into this crate, or compile a stub.
//!
//! QEMU emits no `libqemu-<target>.a`. The emulator is linked from a raw list
//! of several hundred object files plus a few archives, and the way to put it
//! inside a foreign binary is to replay that list — minus the one object that
//! defines `main()`. This build script scrapes the list out of a configured
//! QEMU build tree's `build.ninja`.
//!
//! The tree is named by `EMBSIM_QEMU_P2_BUILD`. It must be staged from this
//! crate's `qemu-target/` and configured for the node
//! (`qemu-target/README.md`); both are checked. When it is unset the crate
//! compiles to a stub that reports the node unavailable, so a workspace still
//! builds on a machine with no QEMU — a CI job that only runs the models must
//! not need a ten-minute QEMU build first.

use std::env;
use std::fs;
use std::path::{Path, PathBuf};

fn main() {
    println!("cargo:rerun-if-env-changed=EMBSIM_QEMU_P2_BUILD");
    println!("cargo:rerun-if-changed=hostdrive.c");
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo::rustc-check-cfg=cfg(qemu_linked)");

    let Some(build_dir) = env::var_os("EMBSIM_QEMU_P2_BUILD") else {
        println!(
            "cargo:warning=EMBSIM_QEMU_P2_BUILD is unset; embsim-p2-qemu builds as a stub \
             (the node is unavailable). Point it at a configured QEMU build tree with \
             target/p2 to link the real thing."
        );
        return;
    };
    let build_dir = PathBuf::from(build_dir);
    let ninja_path = build_dir.join("build.ninja");
    let ninja = fs::read_to_string(&ninja_path).unwrap_or_else(|e| {
        panic!(
            "EMBSIM_QEMU_P2_BUILD={}: cannot read build.ninja: {e}",
            build_dir.display()
        )
    });
    println!("cargo:rerun-if-changed={}", ninja_path.display());

    let link = LinkLine::scrape(&ninja, &build_dir);

    // Cargo does not fingerprint `rustc-link-arg` inputs, and ninja rebuilds
    // an object without touching build.ninja, so without these a QEMU rebuilt
    // after a target edit would leave the old one linked. A rerun rebuilds
    // the crate and relinks its test binaries.
    for input in link.objects.iter().chain(&link.archives) {
        println!("cargo:rerun-if-changed={}", input.display());
    }
    check_staged(&ninja, &build_dir);

    // The C shim: the few functions Rust calls into QEMU with, compiled against
    // QEMU's own headers with QEMU's own flags. QEMU's headers are not
    // -Wextra clean (sign-compare, unused-parameter in inline helpers), and
    // QEMU compiles them with its own -W set, which this does not replay.
    let mut cc = cc::Build::new();
    cc.file("hostdrive.c").warnings(false);
    for flag in &link.cflags {
        cc.flag(flag);
    }
    cc.compile("p2hostdrive");

    let mut args: Vec<String> = Vec::new();
    args.extend(link.objects.iter().map(|obj| obj.display().to_string()));
    args.extend(
        link.archives
            .iter()
            .map(|archive| archive.display().to_string()),
    );
    args.extend(link.libs.iter().cloned());
    for framework in &link.frameworks {
        args.push("-framework".to_string());
        args.push(framework.clone());
    }
    for arg in &args {
        println!("cargo:rustc-link-arg={arg}");
    }
    println!("cargo:rustc-cfg=qemu_linked");

    // A link argument reaches this package's own binaries only. A crate
    // that links this one into a binary of its own — the `embsim` command —
    // replays the same arguments from its build script: `links =
    // "qemu-p2"` hands it this file as `DEP_QEMU_P2_LINK_ARGS_FILE`, one
    // argument a line, and `DEP_QEMU_P2_LINKED`.
    let out_dir = PathBuf::from(env::var_os("OUT_DIR").expect("cargo sets OUT_DIR"));
    let args_file = out_dir.join("qemu-link-args.txt");
    fs::write(&args_file, args.join("\n"))
        .unwrap_or_else(|e| panic!("cannot write {}: {e}", args_file.display()));
    println!("cargo::metadata=link_args_file={}", args_file.display());
    println!("cargo::metadata=linked=true");
}

/// The tree must hold the target this crate carries, and QEMU must carry
/// `host-thread.patch`. Neither fails anywhere else in time: a tree that was
/// not re-staged after a `qemu-target/` edit links and tests the old target,
/// and a tree without the patch fails only at run time, on the
/// `tcg_register_thread` assert the patch exists to avoid.
///
/// One direction only: every file in `qemu-target/target-p2` and `hw-p2` must
/// be in the tree byte for byte. Files only the tree has (the build's own
/// outputs) are not this crate's to judge.
fn check_staged(ninja: &str, build_dir: &Path) {
    let manifest =
        PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").expect("cargo sets CARGO_MANIFEST_DIR"));
    let ours = manifest.join("qemu-target");
    println!("cargo:rerun-if-changed={}", ours.display());

    // The source tree is wherever the target's own objects are compiled from:
    // `build libqemu-p2-softmmu.a.p/target_p2_op_helper.c.o: c_COMPILER
    // ../target/p2/op_helper.c`, three levels below the source root.
    let key = "build libqemu-p2-softmmu.a.p/target_p2_op_helper.c.o: c_COMPILER ";
    let op_helper = ninja
        .lines()
        .find_map(|l| l.strip_prefix(key))
        .and_then(|rest| rest.split_whitespace().next())
        .expect("build.ninja has no target_p2_op_helper.c.o rule to find the source tree by");
    let src = build_dir
        .join(op_helper)
        .ancestors()
        .nth(3)
        .map(Path::to_path_buf)
        .expect("op_helper.c sits three levels below the QEMU source root");
    let src = fs::canonicalize(&src).unwrap_or(src);
    let restage = format!(
        "re-stage with p2-qemu/qemu-target/stage.sh {} (qemu-target/README.md), then run \
         ninja -C {}",
        src.display(),
        build_dir.display()
    );

    for (dir, staged) in [("target-p2", "target/p2"), ("hw-p2", "hw/p2")] {
        let dir = ours.join(dir);
        let mut names: Vec<_> = fs::read_dir(&dir)
            .unwrap_or_else(|e| panic!("cannot list {}: {e}", dir.display()))
            .filter_map(Result::ok)
            .filter(|e| e.file_type().is_ok_and(|t| t.is_file()))
            .map(|e| e.file_name())
            .collect();
        names.sort();
        for name in names {
            let mine = dir.join(&name);
            let theirs = src.join(staged).join(&name);
            let want =
                fs::read(&mine).unwrap_or_else(|e| panic!("cannot read {}: {e}", mine.display()));
            let verdict = match fs::read(&theirs) {
                Ok(got) if got == want => continue,
                Ok(_) => "differs from",
                Err(_) => "is missing, the tree's copy of",
            };
            panic!(
                "{} {verdict} p2-qemu/qemu-target/{}/{}: {restage}",
                theirs.display(),
                dir.file_name().unwrap_or_default().to_string_lossy(),
                name.to_string_lossy()
            );
        }
    }

    // The symbol host-thread.patch introduces.
    let rr = src.join("accel/tcg/tcg-accel-ops-rr.c");
    if !fs::read_to_string(&rr).is_ok_and(|text| text.contains("rr_host_driven")) {
        panic!(
            "host-thread.patch is not applied to {}: {restage}",
            src.display()
        );
    }
}

/// Everything the emulator's own link line says, minus `main()`.
struct LinkLine {
    objects: Vec<PathBuf>,
    archives: Vec<PathBuf>,
    libs: Vec<String>,
    frameworks: Vec<String>,
    cflags: Vec<String>,
}

impl LinkLine {
    fn scrape(ninja: &str, build_dir: &Path) -> Self {
        // --- the emulator's link line ---------------------------------------
        //
        // macOS links `qemu-system-p2-unsigned` and code-signs it into
        // `qemu-system-p2`; Linux links `qemu-system-p2` directly. And when
        // the command line is long — Linux's is — meson switches the rule
        // to `c_LINKER_RSP`, which passes a response file; the inputs are
        // still listed on the `build` line, because ninja needs them as
        // dependencies. So match the rule by prefix.
        let start = ["qemu-system-p2-unsigned", "qemu-system-p2"]
            .iter()
            .map(|target| format!("build {target}: c_LINKER"))
            .find_map(|key| ninja.find(&key))
            .unwrap_or_else(|| {
                let seen: Vec<&str> = ninja
                    .lines()
                    .filter(|l| l.starts_with("build qemu-system-p2"))
                    .map(|l| &l[..l.len().min(120)])
                    .collect();
                panic!(
                    "build.ninja has no `build qemu-system-p2[-unsigned]: c_LINKER*` rule: is \
                     the tree configured for p2-softmmu? build lines seen: {seen:?}"
                )
            });
        let block = &ninja[start..];
        let block_end = block[10..].find("\nbuild ").map_or(block.len(), |i| i + 10);
        let block = &block[..block_end];
        let first_line = block.lines().next().unwrap_or("");

        let absify = |p: &str| -> PathBuf {
            let path = Path::new(p);
            if path.is_absolute() {
                path.to_path_buf()
            } else {
                build_dir.join(path)
            }
        };

        // One object stays behind: system_main.c.o is the ONLY definition of
        // main(); it must not come along. The UI backends still reference the
        // `qemu_main` pointer it owns, which the Rust side defines instead.
        let objects: Vec<PathBuf> = first_line
            .split_whitespace()
            .filter(|t| t.ends_with(".o") && !t.ends_with("system_main.c.o"))
            .map(absify)
            .collect();
        // A build configured before the standalone flash bus was removed still
        // has flashbus.c.o on the link line. That object is gone from this
        // tree; refuse the stale build rather than fail at link with a
        // missing file.
        if let Some(flashbus) = objects
            .iter()
            .find(|o| o.to_string_lossy().ends_with("_flashbus.c.o"))
        {
            panic!(
                "{} is in the link line: this QEMU build still has the removed \
                 standalone flash bus. Delete the build directory and reconfigure \
                 (qemu-target/README.md).",
                flashbus.display()
            );
        }
        // Archives are read from the whole line, implicit inputs (after `|`)
        // included, on purpose: meson lists libqemuutil.a and the
        // --extra-ldflags archives only there.
        let archives: Vec<PathBuf> = first_line
            .split_whitespace()
            .filter(|t| t.ends_with(".a"))
            .map(absify)
            .collect();

        let link_args = block
            .lines()
            .find_map(|l| l.trim_start().strip_prefix("LINK_ARGS = "))
            .unwrap_or("");
        let args: Vec<&str> = link_args.split_whitespace().collect();
        let libs: Vec<String> = args
            .iter()
            .filter(|a| a.starts_with("-l") || a.ends_with(".dylib") || a.ends_with(".so"))
            .map(|a| a.to_string())
            .collect();
        let mut frameworks: Vec<String> = args
            .windows(2)
            .filter(|w| w[0] == "-framework")
            .map(|w| w[1].to_string())
            .collect();
        frameworks.sort();
        frameworks.dedup();

        // --- compile flags for the shim, from a real system object ------------
        let cflags = Self::scrape_cflags(ninja, build_dir);

        assert!(
            objects.len() > 100,
            "scraped only {} objects; the link line did not parse",
            objects.len()
        );
        // Informational: on the build script's own output (`cargo build -vv`),
        // not a warning on every linked build.
        eprintln!(
            "embsim-p2-qemu: linking {} objects, {} archives, {} libs, {} frameworks from {}",
            objects.len(),
            archives.len(),
            libs.len(),
            frameworks.len(),
            build_dir.display()
        );

        Self {
            objects,
            archives,
            libs,
            frameworks,
            cflags,
        }
    }

    /// The flags QEMU compiles the P2 target's own objects with, made absolute.
    ///
    /// A TARGET object's flags, not a system one's: the shim reads
    /// `CPUP2State` (a cog's clock, its PC), and `target/p2/cpu.h` only
    /// compiles under the per-target defines (`COMPILING_PER_TARGET`,
    /// `CONFIG_TARGET`, the `-Itarget/p2` include).
    ///
    /// Two things that are not obvious: `-iquote` takes a SEPARATE argument,
    /// and QEMU's `-I` paths are relative to the build directory. Get either
    /// wrong and the shim cannot find `qemu/osdep.h`.
    fn scrape_cflags(ninja: &str, build_dir: &Path) -> Vec<String> {
        let key = "build libqemu-p2-softmmu.a.p/target_p2_op_helper.c.o:";
        let start = ninja
            .find(key)
            .expect("build.ninja has no target_p2_op_helper.c.o rule to borrow cflags from");
        let block = &ninja[start..];
        let block_end = block[10..].find("\nbuild ").map_or(block.len(), |i| i + 10);
        let args_line = block[..block_end]
            .lines()
            .find_map(|l| l.trim_start().strip_prefix("ARGS = "))
            .unwrap_or("");
        let args: Vec<&str> = args_line.split_whitespace().collect();

        let absify = |p: &str| -> String {
            let path = Path::new(p);
            if path.is_absolute() {
                p.to_string()
            } else {
                build_dir.join(path).to_string_lossy().into_owned()
            }
        };

        let mut out = Vec::new();
        let mut i = 0;
        while i < args.len() {
            let a = args[i];
            if a == "-iquote" {
                if let Some(next) = args.get(i + 1) {
                    out.push(format!("-iquote{}", absify(next)));
                }
                i += 2;
                continue;
            }
            // ninja single-quotes a define whose value carries double quotes
            // (`'-DCONFIG_TARGET="p2-softmmu-config-target.h"'`); the shell
            // would strip those, so strip them here.
            let a = a.trim_matches('\'');
            if let Some(rest) = a.strip_prefix("-I") {
                out.push(format!("-I{}", absify(rest)));
            } else if a.starts_with("-D")
                || a.starts_with("-std")
                || matches!(a, "-fno-strict-aliasing" | "-fno-common" | "-fwrapv")
            {
                out.push(a.to_string());
            }
            i += 1;
        }
        out
    }
}
