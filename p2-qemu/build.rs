//! Link the QEMU Propeller 2 target into this crate, or compile a stub.
//!
//! QEMU emits no `libqemu-<target>.a`. The emulator is linked from a raw list
//! of several hundred object files plus a few archives, and the way to put it
//! inside a foreign binary is to replay that list — minus the one object that
//! defines `main()`. This build script scrapes the list out of a configured
//! QEMU build tree's `build.ninja`, the way spike 1c did by hand.
//!
//! The tree is named by `EMBSIM_QEMU_P2_BUILD`. When it is unset the crate
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

    // The C shim: the few functions Rust calls into QEMU with, compiled against
    // QEMU's own headers with QEMU's own flags.
    let mut cc = cc::Build::new();
    cc.file("hostdrive.c").warnings(false);
    for flag in &link.cflags {
        cc.flag(flag);
    }
    cc.compile("p2hostdrive");

    for obj in &link.objects {
        println!("cargo:rustc-link-arg={}", obj.display());
    }
    for archive in &link.archives {
        println!("cargo:rustc-link-arg={}", archive.display());
    }
    for lib in &link.libs {
        println!("cargo:rustc-link-arg={lib}");
    }
    for framework in &link.frameworks {
        println!("cargo:rustc-link-arg=-framework");
        println!("cargo:rustc-link-arg={framework}");
    }
    println!("cargo:rustc-cfg=qemu_linked");
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

        // Two objects and one archive stay behind.
        //
        // system_main.c.o is the ONLY definition of main(); it must not come
        // along. The UI backends still reference the `qemu_main` pointer it
        // owns, which the Rust side defines instead.
        //
        // flashbus.c.o is the standalone emulator's flash bus, and it pulls
        // in libembsim_cffi.a -- a Rust staticlib carrying its own copy of
        // std. Two Rust runtimes in one binary is a duplicate-symbol wall
        // (`rust_eh_personality`, the allocator shims). The node has no use
        // for either: its flash is a component on the board. The shim
        // defines the three `p2_flashbus_*` symbols the board still calls.
        let excluded_object =
            |t: &str| t.ends_with("system_main.c.o") || t.ends_with("target_p2_flashbus.c.o");
        let objects: Vec<PathBuf> = first_line
            .split_whitespace()
            .filter(|t| t.ends_with(".o") && !excluded_object(t))
            .map(absify)
            .collect();
        let archives: Vec<PathBuf> = first_line
            .split_whitespace()
            .filter(|t| t.ends_with(".a") && !t.ends_with("libembsim_cffi.a"))
            .map(absify)
            .collect();

        let link_args = block
            .lines()
            .find_map(|l| l.trim_start().strip_prefix("LINK_ARGS = "))
            .unwrap_or("");
        let args: Vec<&str> = link_args.split_whitespace().collect();
        let libs: Vec<String> = args
            .iter()
            .filter(|a| {
                (a.starts_with("-l") || a.ends_with(".dylib") || a.ends_with(".so"))
                    && !a.ends_with("libembsim_cffi.a")
            })
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
        println!(
            "cargo:warning=embsim-p2-qemu: linking {} objects, {} archives, {} libs, {} frameworks from {}",
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
    /// Two things that are not obvious, both from spike 1c: `-iquote` takes a
    /// SEPARATE argument, and QEMU's `-I` paths are relative to the build
    /// directory. Get either wrong and the shim cannot find `qemu/osdep.h`.
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
