//! The QEMU target this crate speaks to, carried as data.
//!
//! `qemu-target/` holds the P2 target and board, the two patches that
//! register them with QEMU and let a host thread run the cogs, `stage.sh`,
//! which puts all of it into a QEMU source tree, and `QEMU_PIN`, the QEMU
//! release it is staged into. Every file is embedded here ([`FILES`]), so
//! `embsim qemu install` builds `qemu-system-p2` from exactly the target this
//! build of embsim speaks to, wherever embsim was installed from.
//!
//! The target's [`identity`] is what the built program reports in its
//! handshake and what [`crate::QemuSystemP2`] checks it against: a program
//! staged from other sources is refused before it runs anything. It is the
//! first 16 hex digits of the SHA-256 of a listing of the target's files,
//! one line each in path order, as `sha256sum` prints a file's digest
//! (`"<digest>  <path>"`): every file but the documentation (`README.md`),
//! the licence notices (`LICENSE-*`) and dotfiles. `stage.sh` computes the
//! same digest with `sha256sum` (or `shasum -a 256`) and writes it into the
//! staged tree for the program to report; `stage.sh --identity` prints it.

use std::io;
use std::path::Path;
use std::sync::OnceLock;

use sha2::{Digest, Sha256};

/// One file of `qemu-target/`, as this crate carries it.
#[derive(Debug, Clone, Copy)]
pub struct TargetFile {
    /// The path under `qemu-target/`, `/`-separated.
    pub path: &'static str,
    /// Its bytes.
    pub bytes: &'static [u8],
}

macro_rules! target_files {
    ($($path:literal),* $(,)?) => {
        &[$(TargetFile {
            path: $path,
            bytes: include_bytes!(concat!("../qemu-target/", $path)),
        }),*]
    };
}

/// Every file of `qemu-target/` but its `README.md`, in path order.
/// `tests::every_file_of_the_target_is_carried` holds this list to the
/// directory.
pub static FILES: &[TargetFile] = target_files![
    "LICENSE-PNut-TS",
    "QEMU_PIN",
    "host-thread.patch",
    "hw-p2/Kconfig",
    "hw-p2/meson.build",
    "hw-p2/p2_board.c",
    "p2-softmmu-devices.mak",
    "p2-softmmu.mak",
    "register-p2.patch",
    "stage.sh",
    "target-p2/Kconfig",
    "target-p2/cpu-param.h",
    "target-p2/cpu-qom.h",
    "target-p2/cpu.c",
    "target-p2/cpu.h",
    "target-p2/gen_stubs.py",
    "target-p2/helper.h",
    "target-p2/hostipc.c",
    "target-p2/hostipc.h",
    "target-p2/insn.decode",
    "target-p2/interp.c",
    "target-p2/meson.build",
    "target-p2/op_helper.c",
    "target-p2/pinbus.c",
    "target-p2/pinbus.h",
    "target-p2/translate.c",
];

/// Whether a file of `qemu-target/` is part of the target's identity:
/// everything the build reads, so not the documentation, the licence
/// notices or dotfiles. `stage.sh` applies the same rule.
pub fn in_identity(path: &str) -> bool {
    let name = path.rsplit('/').next().unwrap_or(path);
    name != "README.md" && !name.starts_with("LICENSE-") && !name.starts_with('.')
}

fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}

/// The identity of `files`, as [`identity`] computes it for [`FILES`].
pub fn identity_of(files: &[TargetFile]) -> String {
    let mut listed: Vec<&TargetFile> = files.iter().filter(|f| in_identity(f.path)).collect();
    listed.sort_by(|a, b| a.path.as_bytes().cmp(b.path.as_bytes()));
    let mut listing = String::new();
    for file in listed {
        listing.push_str(&hex(&Sha256::digest(file.bytes)));
        listing.push_str("  ");
        listing.push_str(file.path);
        listing.push('\n');
    }
    let digest = hex(&Sha256::digest(listing.as_bytes()));
    digest[..16].to_string()
}

/// The identity of the target this crate carries: 16 hex digits.
pub fn identity() -> &'static str {
    static IDENTITY: OnceLock<String> = OnceLock::new();
    IDENTITY.get_or_init(|| identity_of(FILES))
}

/// The QEMU release the target is staged into (`QEMU_PIN`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QemuPin {
    /// The tag, `v10.1.0`.
    pub tag: &'static str,
    /// The commit the tag must resolve to: a tag can be moved, a commit
    /// cannot.
    pub commit: &'static str,
}

impl QemuPin {
    /// The version QEMU reports itself as (`QEMU_VERSION`): the tag without
    /// its `v`.
    pub fn version(&self) -> &'static str {
        self.tag.strip_prefix('v').unwrap_or(self.tag)
    }
}

/// The pin, read from `QEMU_PIN`'s `tag` and `commit` lines.
pub fn qemu_pin() -> QemuPin {
    let text: &'static str = std::str::from_utf8(file("QEMU_PIN").bytes).expect("QEMU_PIN is text");
    let field = |key: &str| -> &'static str {
        text.lines()
            .filter_map(|line| line.trim().split_once(' '))
            .find(|(k, _)| *k == key)
            .map(|(_, value)| value.trim())
            .unwrap_or_else(|| panic!("QEMU_PIN has no `{key}` line"))
    };
    QemuPin {
        tag: field("tag"),
        commit: field("commit"),
    }
}

/// The carried file at `path`.
pub fn file(path: &str) -> &'static TargetFile {
    FILES
        .iter()
        .find(|f| f.path == path)
        .unwrap_or_else(|| panic!("qemu-target/{path} is not carried"))
}

/// Write every carried file under `dir`, `stage.sh` executable: a copy of
/// `qemu-target/` that `stage.sh` can stage from.
pub fn write_to(dir: &Path) -> io::Result<()> {
    for file in FILES {
        let path = dir.join(file.path);
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent)?;
        }
        std::fs::write(&path, file.bytes)?;
        if file.path.ends_with(".sh") {
            use std::os::unix::fs::PermissionsExt;
            std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o755))?;
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::path::PathBuf;

    fn target_dir() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("qemu-target")
    }

    fn walk(dir: &Path, prefix: &str, out: &mut Vec<String>) {
        for entry in std::fs::read_dir(dir).expect("qemu-target is readable") {
            let entry = entry.expect("a directory entry");
            let name = entry.file_name().to_string_lossy().into_owned();
            if name.starts_with('.') {
                continue;
            }
            let path = format!("{prefix}{name}");
            if entry.file_type().expect("a file type").is_dir() {
                walk(&entry.path(), &format!("{path}/"), out);
            } else {
                out.push(path);
            }
        }
    }

    #[test]
    fn every_file_of_the_target_is_carried() {
        let mut on_disk = Vec::new();
        walk(&target_dir(), "", &mut on_disk);
        on_disk.retain(|path| path != "README.md");
        on_disk.sort();
        let carried: Vec<String> = FILES.iter().map(|f| f.path.to_string()).collect();
        assert_eq!(
            carried, on_disk,
            "src/target.rs's FILES must list every file of qemu-target/ but README.md"
        );
        for file in FILES {
            assert_eq!(
                std::fs::read(target_dir().join(file.path)).expect("readable"),
                file.bytes,
                "{}",
                file.path
            );
        }
    }

    #[test]
    fn the_pin_is_a_tag_and_the_commit_it_resolves_to() {
        let pin = qemu_pin();
        assert!(pin.tag.starts_with('v'), "{pin:?}");
        assert_eq!(pin.commit.len(), 40, "{pin:?}");
        assert!(pin.commit.bytes().all(|b| b.is_ascii_hexdigit()), "{pin:?}");
        assert_eq!(pin.version(), &pin.tag[1..]);
    }

    #[test]
    fn the_identity_ignores_documentation_and_notices() {
        assert!(in_identity("target-p2/hostipc.c"));
        assert!(in_identity("QEMU_PIN"));
        assert!(!in_identity("README.md"));
        assert!(!in_identity("LICENSE-PNut-TS"));
        assert!(!in_identity("target-p2/.DS_Store"));
        let id = identity();
        assert_eq!(id.len(), 16);
        assert!(id.bytes().all(|b| b.is_ascii_hexdigit()));
    }
}
