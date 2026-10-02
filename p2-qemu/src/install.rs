//! `embsim qemu install`: build `qemu-system-p2` from the target this crate
//! carries and put it where the node looks.
//!
//! The steps, each a command the log beside the build keeps:
//!
//! 1. **Fetch** QEMU at the pinned tag ([`crate::target::qemu_pin`]) into
//!    the build directory, shallow, and check the tag resolves to the pinned
//!    commit — a tag can be moved, a commit cannot.
//! 2. **Stage** the target: the carried `qemu-target/` is written out and
//!    its `stage.sh` copies the target and board in and applies the two
//!    patches. The identity `stage.sh` writes for the program to report must
//!    be the one this crate computes ([`crate::target::identity`]).
//! 3. **Configure** the one target, and nothing QEMU builds by default
//!    ([`CONFIGURE`]): a binary of about 8 MB that needs only the system's
//!    libraries and glib.
//! 4. **Build** `qemu-system-p2` with ninja.
//! 5. **Install** it into `~/.embsim/qemu/<identity>/` (or `--prefix`), with
//!    QEMU's licence texts, a `NOTICE` saying what it is and how it was
//!    built, and the target sources it was built from under `source/`.
//! 6. **Check** it: start it and take its hello, which must name this
//!    crate's protocol, target and QEMU.
//!
//! The installed program is QEMU, a GPL-2.0 work, and is distributed under
//! that licence with its corresponding source; embsim runs it as a separate
//! program over a small fixed protocol and links none of it.

use std::io::Write;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};

use crate::peer::{self, QemuSystemP2};
use crate::target;

/// Where QEMU is fetched from unless `--qemu-git` says otherwise.
pub const QEMU_GIT: &str = "https://gitlab.com/qemu-project/qemu.git";

/// The configure line: the P2 target and none of QEMU's default features
/// (no UI, no networking back-ends, no tools), so the binary needs only
/// glib beyond the system's libraries; no containers (configure otherwise
/// probes docker, which hangs on a Mac without it); no docs; warnings not
/// fatal on a compiler newer than QEMU's.
pub const CONFIGURE: [&str; 5] = [
    "--target-list=p2-softmmu",
    "--without-default-features",
    "--disable-containers",
    "--disable-docs",
    "--disable-werror",
];

/// What `embsim qemu install` was asked.
#[derive(Debug, Clone, Default)]
pub struct InstallOptions {
    /// Install into this directory instead of [`peer::install_dir`].
    pub prefix: Option<PathBuf>,
    /// Build here instead of a directory under the system's temporary one.
    /// A build directory is reused: an interrupted install picks up where
    /// it stopped.
    pub build_dir: Option<PathBuf>,
    /// Fetch QEMU from here instead of [`QEMU_GIT`] (a mirror, a local
    /// clone); the pinned commit is checked whatever the source.
    pub qemu_git: Option<String>,
    /// Build jobs (ninja's own default without it).
    pub jobs: Option<usize>,
    /// Keep the build directory after a successful install.
    pub keep_build: bool,
    /// Build and install even when a matching program is installed.
    pub force: bool,
    /// Say what would be done, and do nothing.
    pub dry_run: bool,
}

/// What an install would do, worked out from its options.
#[derive(Debug, Clone)]
pub struct Plan {
    /// The directory the program goes into.
    pub dir: PathBuf,
    /// The build directory.
    pub build: PathBuf,
    /// Where QEMU comes from.
    pub git: String,
}

impl Plan {
    /// The plan for `options`, or why there is none.
    pub fn new(options: &InstallOptions) -> Result<Self, String> {
        let dir = match &options.prefix {
            Some(dir) => absolute(dir)?,
            None => peer::install_dir().ok_or_else(|| {
                "HOME is not set, so there is no ~/.embsim/qemu to install into; name a \
                 directory with --prefix"
                    .to_string()
            })?,
        };
        let build = match &options.build_dir {
            Some(dir) => absolute(dir)?,
            None => std::env::temp_dir().join(format!("embsim-qemu-{}", target::identity())),
        };
        Ok(Self {
            dir,
            build,
            git: options
                .qemu_git
                .clone()
                .unwrap_or_else(|| QEMU_GIT.to_string()),
        })
    }

    /// The program's path once installed.
    pub fn program(&self) -> PathBuf {
        self.dir.join(QemuSystemP2::NAME)
    }

    fn source(&self) -> PathBuf {
        self.build.join("qemu")
    }

    fn log(&self) -> PathBuf {
        self.build.join("install.log")
    }

    /// The plan, as `--dry-run` prints it.
    pub fn describe(&self, out: &mut dyn Write) {
        let pin = target::qemu_pin();
        let _ = writeln!(out, "target {}", target::identity());
        let _ = writeln!(out, "qemu {} {} from {}", pin.tag, pin.commit, self.git);
        let _ = writeln!(out, "build {}", self.build.display());
        let _ = writeln!(out, "configure {}", CONFIGURE.join(" "));
        let _ = writeln!(out, "install {}", self.program().display());
    }
}

fn absolute(path: &Path) -> Result<PathBuf, String> {
    if path.is_absolute() {
        return Ok(path.to_path_buf());
    }
    std::env::current_dir()
        .map(|dir| dir.join(path))
        .map_err(|err| format!("{}: no current directory: {err}", path.display()))
}

/// The tools a build needs, and the one that is missing, with how to get
/// them all.
fn check_tools() -> Result<(), String> {
    let probes: [(&str, &[&str]); 5] = [
        ("git", &["--version"]),
        ("cc", &["--version"]),
        ("ninja", &["--version"]),
        ("python3", &["--version"]),
        ("pkg-config", &["--exists", "glib-2.0"]),
    ];
    let mut missing = Vec::new();
    for (tool, args) in probes {
        let ok = Command::new(tool)
            .args(args)
            .stdin(Stdio::null())
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .status()
            .is_ok_and(|status| status.success());
        if !ok {
            missing.push(if tool == "pkg-config" {
                "glib's development files (pkg-config glib-2.0)".to_string()
            } else {
                tool.to_string()
            });
        }
    }
    if missing.is_empty() {
        return Ok(());
    }
    Err(format!(
        "building qemu-system-p2 needs {} and this machine has not got it. On Debian or \
         Ubuntu: sudo apt-get install git build-essential ninja-build pkg-config \
         libglib2.0-dev python3-venv flex bison. On macOS: xcode-select --install, then \
         brew install ninja pkgconf glib",
        missing.join(", ")
    ))
}

/// Run `command`, its output appended to the plan's log; on failure, the
/// log's tail and where the whole of it is.
fn run_logged(plan: &Plan, what: &str, command: &mut Command) -> Result<(), String> {
    let log = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(plan.log())
        .map_err(|err| format!("cannot write {}: {err}", plan.log().display()))?;
    let mut header = log
        .try_clone()
        .map_err(|err| format!("cannot write {}: {err}", plan.log().display()))?;
    let _ = writeln!(header, "\n==> {what}: {command:?}");
    let status = command
        .stdin(Stdio::null())
        .stdout(log.try_clone().map_err(|err| err.to_string())?)
        .stderr(log)
        .status()
        .map_err(|err| format!("{what}: cannot start {:?}: {err}", command.get_program()))?;
    if status.success() {
        return Ok(());
    }
    let text = std::fs::read_to_string(plan.log()).unwrap_or_default();
    let lines: Vec<&str> = text.lines().collect();
    let tail = lines[lines.len().saturating_sub(30)..].join("\n");
    Err(format!(
        "{what} failed ({status}); the end of {}:\n{tail}",
        plan.log().display()
    ))
}

/// The output of `git` with `args` in `dir`, trimmed.
fn git_output(dir: &Path, args: &[&str]) -> Option<String> {
    let output = Command::new("git")
        .arg("-C")
        .arg(dir)
        .args(args)
        .stderr(Stdio::null())
        .output()
        .ok()?;
    output
        .status
        .success()
        .then(|| String::from_utf8_lossy(&output.stdout).trim().to_string())
}

/// Install `qemu-system-p2` as `options` say, saying what it does on `out`.
/// Returns the installed program's path.
pub fn install(options: &InstallOptions, out: &mut dyn Write) -> Result<PathBuf, String> {
    let plan = Plan::new(options)?;
    let pin = target::qemu_pin();
    if options.dry_run {
        plan.describe(out);
        return Ok(plan.program());
    }
    if !options.force {
        let existing = QemuSystemP2::at(plan.program());
        if plan.program().exists() {
            if let Ok(identity) = existing.probe() {
                if identity.check().is_ok() {
                    let _ = writeln!(
                        out,
                        "qemu-system-p2 for target {} is installed: {}",
                        target::identity(),
                        plan.program().display()
                    );
                    return Ok(plan.program());
                }
            }
        }
    }
    check_tools()?;
    std::fs::create_dir_all(&plan.build)
        .map_err(|err| format!("cannot make {}: {err}", plan.build.display()))?;
    let _ = writeln!(
        out,
        "embsim qemu install: QEMU {} with the P2 target {}, built in {} (log: {})",
        pin.tag,
        target::identity(),
        plan.build.display(),
        plan.log().display()
    );

    // 1. Fetch.
    let src = plan.source();
    if git_output(&src, &["rev-parse", "HEAD"]).as_deref() != Some(pin.commit) {
        let _ = writeln!(out, "  fetching QEMU {} from {}", pin.tag, plan.git);
        let _ = std::fs::remove_dir_all(&src);
        std::fs::create_dir_all(&src)
            .map_err(|err| format!("cannot make {}: {err}", src.display()))?;
        run_logged(
            &plan,
            "git init",
            Command::new("git").arg("init").arg("-q").arg(&src),
        )?;
        run_logged(
            &plan,
            &format!("fetching QEMU {} from {}", pin.tag, plan.git),
            Command::new("git").arg("-C").arg(&src).args([
                "fetch",
                "-q",
                "--depth",
                "1",
                &plan.git,
                &format!("refs/tags/{}", pin.tag),
            ]),
        )?;
        let fetched = git_output(&src, &["rev-parse", "FETCH_HEAD^{commit}"]).unwrap_or_default();
        if fetched != pin.commit {
            return Err(format!(
                "QEMU's tag {} at {} is commit {fetched}, and this embsim's target is staged \
                 into {}: the tag moved. Fetch from a source that has the pinned commit \
                 (--qemu-git)",
                pin.tag, plan.git, pin.commit
            ));
        }
    } else {
        let _ = writeln!(out, "  QEMU {} is already fetched", pin.tag);
    }
    // A tree an earlier install staged: its patched files back as fetched,
    // so the patches apply afresh.
    run_logged(
        &plan,
        "checking out the pinned commit",
        Command::new("git")
            .arg("-C")
            .arg(&src)
            .args(["checkout", "-q", "-f", pin.commit]),
    )?;

    // 2. Stage.
    let _ = writeln!(out, "  staging the P2 target");
    let staged = plan.build.join("qemu-target");
    let _ = std::fs::remove_dir_all(&staged);
    target::write_to(&staged)
        .map_err(|err| format!("cannot write the target to {}: {err}", staged.display()))?;
    run_logged(
        &plan,
        "staging the P2 target",
        Command::new("sh").arg(staged.join("stage.sh")).arg(&src),
    )?;
    let header = std::fs::read_to_string(src.join("target/p2/hostipc-identity.h"))
        .map_err(|err| format!("stage.sh wrote no identity header: {err}"))?;
    if !header.contains(&format!("\"{}\"", target::identity())) {
        return Err(format!(
            "stage.sh computed the target's identity as {}, and this embsim computes {}: the \
             two digests of the same files disagree, which is a bug in one of them \
             (src/target.rs, qemu-target/stage.sh)",
            header.trim(),
            target::identity()
        ));
    }

    // 3. Configure, once per build directory: ninja reconfigures itself
    // when a meson file it read changes.
    let build = src.join("build-p2");
    if !build.join("build.ninja").exists() {
        let _ = writeln!(out, "  configuring: {}", CONFIGURE.join(" "));
        std::fs::create_dir_all(&build)
            .map_err(|err| format!("cannot make {}: {err}", build.display()))?;
        run_logged(
            &plan,
            "configuring QEMU",
            Command::new(src.join("configure"))
                .args(CONFIGURE)
                .current_dir(&build),
        )?;
    }

    // 4. Build.
    let _ = writeln!(
        out,
        "  building qemu-system-p2 (a few minutes the first time)"
    );
    let mut ninja = Command::new("ninja");
    ninja.arg("-C").arg(&build);
    if let Some(jobs) = options.jobs {
        ninja.arg("-j").arg(jobs.to_string());
    }
    ninja.arg(QemuSystemP2::NAME);
    run_logged(&plan, "building qemu-system-p2", &mut ninja)?;

    // 5. Install.
    std::fs::create_dir_all(&plan.dir)
        .map_err(|err| format!("cannot make {}: {err}", plan.dir.display()))?;
    let temporary = plan
        .dir
        .join(format!(".{}.{}", QemuSystemP2::NAME, std::process::id()));
    std::fs::copy(build.join(QemuSystemP2::NAME), &temporary).map_err(|err| {
        format!(
            "cannot copy qemu-system-p2 into {}: {err}",
            plan.dir.display()
        )
    })?;
    std::fs::rename(&temporary, plan.program())
        .map_err(|err| format!("cannot install {}: {err}", plan.program().display()))?;
    for licence in ["COPYING", "COPYING.LIB"] {
        let _ = std::fs::copy(src.join(licence), plan.dir.join(licence));
    }
    let source = plan.dir.join("source");
    let _ = std::fs::remove_dir_all(&source);
    target::write_to(&source).map_err(|err| format!("cannot write {}: {err}", source.display()))?;
    std::fs::write(plan.dir.join("NOTICE"), notice(&plan))
        .map_err(|err| format!("cannot write the NOTICE: {err}"))?;

    // 6. Check.
    let installed = QemuSystemP2::at(plan.program());
    let identity = installed
        .probe()
        .map_err(|err| format!("the installed qemu-system-p2 does not start: {err}"))?;
    identity
        .check()
        .map_err(|why| format!("the installed qemu-system-p2 is not the one built: {why}"))?;
    if !options.keep_build {
        let _ = std::fs::remove_dir_all(&plan.build);
    }
    let _ = writeln!(out, "installed {}", plan.program().display());
    let _ = writeln!(out, "  {identity}");
    let _ = writeln!(
        out,
        "  qemu-system-p2 is QEMU, a GPL-2.0 program, with its licence and source beside it \
         ({}); embsim runs it as a separate program and links none of it",
        plan.dir.join("NOTICE").display()
    );
    Ok(plan.program())
}

/// The `NOTICE` beside an installed program.
fn notice(plan: &Plan) -> String {
    let pin = target::qemu_pin();
    format!(
        "qemu-system-p2\n\
         ==============\n\
         \n\
         QEMU {tag} (commit {commit}, {git}) with embsim's Propeller 2 target, target\n\
         identity {identity}, built by `embsim qemu install`:\n\
         \n\
         \x20   ../configure {configure}\n\
         \x20   ninja qemu-system-p2\n\
         \n\
         Licence. QEMU as a whole is licensed under the GNU General Public License,\n\
         version 2 (COPYING), and so is this program. The P2 target and board in\n\
         source/ are LGPL-2.1-or-later (COPYING.LIB); its two QEMU patches are\n\
         GPL-2.0-or-later, host-thread.patch's hunks to MIT-licensed QEMU files MIT;\n\
         target-p2/insn.decode is MIT (source/LICENSE-PNut-TS).\n\
         \n\
         Corresponding source. QEMU at the commit above, with source/ staged into it\n\
         by `sh source/stage.sh <qemu>`, configured and built as above.\n\
         \n\
         embsim starts this program and talks to it over a small fixed protocol (a\n\
         shared page or a socket pair); it links none of it, and nothing of embsim is\n\
         in it.\n",
        tag = pin.tag,
        commit = pin.commit,
        git = plan.git,
        identity = target::identity(),
        configure = CONFIGURE.join(" "),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_prefix_is_where_the_program_goes_and_the_plan_names_the_pin() {
        let options = InstallOptions {
            prefix: Some(PathBuf::from("/opt/p2")),
            build_dir: Some(PathBuf::from("/tmp/b")),
            ..InstallOptions::default()
        };
        let plan = Plan::new(&options).expect("a plan");
        assert_eq!(plan.program(), Path::new("/opt/p2/qemu-system-p2"));
        let mut text = Vec::new();
        plan.describe(&mut text);
        let text = String::from_utf8(text).expect("text");
        let pin = target::qemu_pin();
        assert!(
            text.contains(&format!("qemu {} {} from {QEMU_GIT}", pin.tag, pin.commit)),
            "{text}"
        );
        assert!(
            text.contains(&format!("target {}", target::identity())),
            "{text}"
        );
        assert!(text.contains("install /opt/p2/qemu-system-p2"), "{text}");
    }
}
