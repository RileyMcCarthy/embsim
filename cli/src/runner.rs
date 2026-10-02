//! The runner: a project's own catalog crates built into the command.
//!
//! A project whose file names catalog crates (`[catalog] crates = [...]`)
//! runs in a **runner**, a small binary crate the `embsim` tool writes into
//! `.embsim/runner-<id>/` beside the project: its dependencies are
//! `embsim-cli` from one embsim checkout and each catalog crate by path,
//! and its `main` is [`crate::runner_main`] over them. The tool builds it
//! with Cargo — incremental, so a build with nothing changed is Cargo's own
//! no-op check — and `exec`s it with the same arguments. There is no plugin
//! interface: Cargo compiles the project's crates and embsim into one
//! binary, against one copy of embsim (`PROJECTS.md` §10, `NODES.md` §13).
//!
//! What the tool decides, in order:
//!
//! 1. **The crates**: each `[catalog] crates` entry, relative to the project
//!    file, is a directory with a `Cargo.toml` whose package has a library.
//! 2. **The embsim checkout**: `[catalog] embsim` when the project names
//!    one, else the checkout this tool was built from
//!    ([`crate::source_dir`]).
//! 3. **The runner's directory**: `.embsim/runner-<id>/`, `<id>` a hash of
//!    the crates' canonical directories and the checkout's, so projects
//!    naming the same crates share a runner. `.embsim/` carries a
//!    `.gitignore` that keeps it out of version control.
//! 4. **One embsim**: `cargo metadata --no-deps` on each crate gives its
//!    embsim dependencies. Every one must be in the checkout, or the tool
//!    refuses before it builds, naming both directories: two copies would be
//!    two virtual clocks, and a part on one would wait on time nobody
//!    advances. The runner reaches the checkout by the path the crates use
//!    (a symlinked checkout is spelled as they spell it), since Cargo takes
//!    two spellings of one directory for two packages.
//! 5. **The files**: `Cargo.toml`, `main.rs`, `build.rs` (QEMU's link
//!    arguments, as `cli/build.rs` passes them), each rewritten only when
//!    its content would change, so Cargo sees nothing new.
//! 6. **Where it builds**: the target directory of the Cargo workspace the
//!    first crate is a member of (`CARGO_TARGET_DIR` included, as Cargo
//!    itself reads it), so what that workspace built is reused; for a crate
//!    in no workspace — a package that is its own root, as `embsim new
//!    --catalog` starts one — `.embsim/target` (or `CARGO_TARGET_DIR`), so
//!    no build lands in the crate's source tree.
//! 7. **The lock file**: the runner's `Cargo.lock` is seeded from the
//!    workspace's lock file, with every package of embsim's own lock file
//!    whose name the workspace's does not lock, and seeded again whenever
//!    that seed changes. The profile is `release`, unless
//!    `EMBSIM_RUNNER_PROFILE` names another.
//! 8. **After the build**: every `embsim_*` library the build reports must
//!    come from one place; a build that fails on two copies of an embsim
//!    package is said to, in place of the hint about what a catalog crate
//!    exports.

use std::collections::{BTreeMap, BTreeSet};
use std::convert::Infallible;
use std::ffi::{OsStr, OsString};
use std::io::{BufRead, BufReader, Write};
use std::os::unix::process::CommandExt;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};

use embsim_board::{state_dir, CatalogTable};

use crate::checklist::toml_string;
use crate::CatalogCrate;

/// The profile a runner builds in, unless `EMBSIM_RUNNER_PROFILE` names
/// another: a run is the ISS's or a model's hot loop, and the build is paid
/// once.
const DEFAULT_PROFILE: &str = "release";

/// The prefix of every embsim library's crate name: the libraries a runner
/// must link exactly one copy of.
const EMBSIM_LIBRARY: &str = "embsim_";

/// The directory under the target directory a profile's binaries land in.
fn profile_dir(profile: &str) -> &str {
    match profile {
        "dev" | "test" => "debug",
        "bench" => "release",
        other => other,
    }
}

/// One catalog crate, as the runner depends on it.
#[derive(Debug, Clone)]
struct CrateDep {
    /// The package name, the dependency's key.
    package: String,
    /// The library's crate name, as Rust code names it.
    library: String,
    /// The directory, canonical.
    dir: PathBuf,
}

/// Everything the tool decided about one project's runner.
#[derive(Debug)]
struct Plan {
    /// The project file, as the command line gave it.
    project: PathBuf,
    crates: Vec<CrateDep>,
    /// The embsim checkout, canonical.
    embsim: PathBuf,
    /// The same checkout as the runner's manifest spells it: the path the
    /// catalog crates' embsim dependencies use, else [`Self::embsim`].
    embsim_path: PathBuf,
    /// `[catalog] embsim`, as the file gives it.
    named_embsim: Option<String>,
    /// `.embsim/runner-<id>/`, absolute: Cargo runs in it.
    dir: PathBuf,
    /// The same directory as the project's path reaches it, for messages.
    shown: PathBuf,
    /// `embsim-runner-<id>`: the package and its binary.
    package: String,
}

impl Plan {
    /// The crates' package names, comma-separated.
    fn crate_names(&self) -> String {
        self.crates
            .iter()
            .map(|dep| dep.package.as_str())
            .collect::<Vec<_>>()
            .join(", ")
    }

    /// Where the checkout came from, for a message.
    fn embsim_source(&self) -> String {
        match &self.named_embsim {
            Some(path) => format!("[catalog] embsim = {path:?}"),
            None => "the checkout this embsim was built from".to_string(),
        }
    }

    /// The project file's directory, as the command line reaches it.
    fn project_dir(&self) -> &Path {
        self.project
            .parent()
            .filter(|dir| !dir.as_os_str().is_empty())
            .unwrap_or(Path::new("."))
    }
}

/// Write, build and `exec` the runner for `project`, whose `[catalog]` is
/// `catalog`, with the command line `args`. Returns only when it could not
/// hand over, with the reason; Cargo's own output goes to standard error as
/// it comes.
pub fn hand_over(
    project: &Path,
    catalog: &CatalogTable,
    rebuild: bool,
    args: &[OsString],
    err: &mut dyn Write,
) -> Result<Infallible, String> {
    let mut plan = plan(project, catalog)?;
    if rebuild {
        match std::fs::remove_dir_all(&plan.dir) {
            Ok(()) => {}
            Err(error) if error.kind() == std::io::ErrorKind::NotFound => {}
            Err(error) => {
                return Err(format!(
                    "--rebuild: cannot remove {}: {error}",
                    plan.dir.display()
                ))
            }
        }
    }
    let cargo = Cargo::find();
    let metadata = match cargo.crate_metadata(&plan) {
        Ok(metadata) => metadata,
        Err(missing) => {
            // Written either way, so the runner can be built by hand.
            write_runner(&plan)?;
            return Err(missing);
        }
    };
    plan.embsim_path = one_embsim(&plan, &metadata)?;
    write_runner(&plan)?;
    let workspace = workspace_of(&plan, &metadata);
    seed_lock(&plan, workspace.as_ref())?;
    let target = workspace.as_ref().map_or_else(
        || {
            std::env::var_os("CARGO_TARGET_DIR").map_or_else(
                || plan.dir.parent().unwrap_or(&plan.dir).join("target"),
                PathBuf::from,
            )
        },
        |workspace| workspace.target.clone(),
    );
    let profile = std::env::var("EMBSIM_RUNNER_PROFILE")
        .ok()
        .filter(|profile| !profile.is_empty())
        .unwrap_or_else(|| DEFAULT_PROFILE.to_string());
    let _ = writeln!(
        err,
        "embsim: building the runner for {} ({}, embsim at {}) in {}",
        plan.project.display(),
        plan.crate_names(),
        plan.embsim_path.display(),
        plan.shown.display()
    );
    let _ = err.flush();
    if rebuild {
        cargo.clean(&plan, &target, &profile)?;
    }
    // A runner built before is brought up to date quietly: Cargo's errors
    // still show, its progress and the warnings it replays from earlier
    // builds do not. The first build, and a rebuild, show it all.
    let quiet = !rebuild
        && target
            .join(profile_dir(&profile))
            .join(&plan.package)
            .exists();
    let built = cargo.build(&plan, &target, &profile, quiet)?;
    let executable = match built {
        Build::Built { executable, copies } => {
            if let Some(copies) = two_copies(&copies) {
                return Err(format!(
                    "the runner for {} links two copies of embsim: {copies}. Each copy has its \
                     own virtual clock, and a part on one waits on time nobody advances. Point \
                     every catalog crate's embsim dependencies at {}, or name the checkout they \
                     use with [catalog] embsim",
                    plan.project.display(),
                    plan.embsim_path.display()
                ));
            }
            executable
        }
        Build::Failed { copies } => {
            let collision = two_copies(&copies).or_else(|| cargo.collision(&plan));
            if let Some(copies) = collision {
                return Err(format!(
                    "the runner for {} did not build: it met two copies of embsim ({copies}); \
                     Cargo's errors are above. Each copy would have its own virtual clock. \
                     Point the embsim dependencies of every catalog crate, and of the crates \
                     they depend on, at {}, each spelled by that path, or name the checkout \
                     they use with [catalog] embsim (PROJECTS.md §10)",
                    plan.project.display(),
                    plan.embsim_path.display()
                ));
            }
            return Err(format!(
                "the runner for {} did not build (catalog crates {}; embsim at {}); Cargo's \
                 errors are above. A catalog crate is a library with `pub fn register(set: &mut \
                 CatalogSet) -> Result<(), ProjectError>` at its root (PROJECTS.md §10)",
                plan.project.display(),
                plan.crate_names(),
                plan.embsim_path.display()
            ));
        }
    };
    let error = Command::new(&executable)
        .arg0("embsim")
        .args(args.iter().skip(1))
        .exec();
    Err(format!(
        "cannot start the runner {}: {error}",
        executable.display()
    ))
}

/// The crates, the checkout and the runner's directory for `project`.
fn plan(project: &Path, catalog: &CatalogTable) -> Result<Plan, String> {
    let project_dir = project
        .parent()
        .filter(|dir| !dir.as_os_str().is_empty())
        .unwrap_or(Path::new("."));
    let crates = catalog
        .crates
        .iter()
        .map(|path| crate_dep(project, project_dir, path))
        .collect::<Result<Vec<_>, _>>()?;
    let mut packages = BTreeSet::new();
    for dep in &crates {
        if dep.package == "embsim-cli" || dep.package == "embsim-p2-qemu" {
            return Err(format!(
                "{}: [catalog] crates: {} is embsim's own crate, which every runner already \
                 holds; a catalog crate is one of the project's",
                project.display(),
                dep.dir.display()
            ));
        }
        if !packages.insert(dep.package.as_str()) {
            return Err(format!(
                "{}: [catalog] crates names two crates whose package is {:?}; a runner depends \
                 on each by its package name",
                project.display(),
                dep.package
            ));
        }
    }
    let embsim = embsim_checkout(project, project_dir, catalog.embsim.as_deref())?;
    let id = runner_id(&crates, &embsim);
    let cannot = |error: std::io::Error| {
        format!(
            "cannot make {}: {error}",
            project_dir.join(".embsim").display()
        )
    };
    let state = state_dir(project_dir)
        .and_then(|state| state.canonicalize())
        .map_err(cannot)?;
    let runner = format!("runner-{id}");
    Ok(Plan {
        project: project.to_path_buf(),
        crates,
        embsim_path: embsim.clone(),
        embsim,
        named_embsim: catalog.embsim.clone(),
        dir: state.join(&runner),
        shown: project_dir.join(".embsim").join(runner),
        package: format!("embsim-runner-{id}"),
    })
}

/// The catalog crate `path` names, relative to the project's directory.
fn crate_dep(project: &Path, project_dir: &Path, path: &str) -> Result<CrateDep, String> {
    let at = project_dir.join(path);
    let fail = |why: String| {
        format!(
            "{}: [catalog] crates: {path:?} ({}) {why}",
            project.display(),
            at.display()
        )
    };
    let dir = at.canonicalize().map_err(|error| {
        fail(format!(
            "is not there ({error}); a catalog crate is a directory holding a Cargo.toml, \
             relative to the project file (`embsim new --catalog DIR` starts one)"
        ))
    })?;
    let manifest = dir.join("Cargo.toml");
    let text = std::fs::read_to_string(&manifest).map_err(|error| {
        fail(format!(
            "holds no readable Cargo.toml ({error}); a catalog crate is a Cargo package"
        ))
    })?;
    let table: toml::Table = toml::from_str(&text)
        .map_err(|error| fail(format!("has a Cargo.toml that does not parse: {error}")))?;
    let package = table
        .get("package")
        .and_then(|package| package.get("name"))
        .and_then(toml::Value::as_str)
        .ok_or_else(|| {
            fail(
                "has a Cargo.toml with no [package] name: name the catalog crate's own \
                 directory, not a workspace's"
                    .to_string(),
            )
        })?
        .to_string();
    let lib = table.get("lib");
    if lib.is_none() && !dir.join("src/lib.rs").exists() {
        return Err(fail(format!(
            "is the package {package:?}, which has no library: a catalog crate is a library \
             whose root holds `pub fn register`"
        )));
    }
    let library = lib
        .and_then(|lib| lib.get("name"))
        .and_then(toml::Value::as_str)
        .map_or_else(|| package.replace('-', "_"), str::to_string);
    Ok(CrateDep {
        package,
        library,
        dir,
    })
}

/// Whether `dir` is an embsim checkout: its `cli/Cargo.toml` is the
/// `embsim-cli` package, and its `p2-qemu` is there.
pub(crate) fn is_checkout(dir: &Path) -> Result<(), String> {
    let manifest = dir.join("cli/Cargo.toml");
    let text = std::fs::read_to_string(&manifest)
        .map_err(|error| format!("{} cannot be read: {error}", manifest.display()))?;
    let table: toml::Table = toml::from_str(&text)
        .map_err(|error| format!("{} does not parse: {error}", manifest.display()))?;
    let name = table
        .get("package")
        .and_then(|package| package.get("name"))
        .and_then(toml::Value::as_str);
    if name != Some("embsim-cli") {
        return Err(format!(
            "{} is not the embsim-cli package",
            manifest.display()
        ));
    }
    if !dir.join("p2-qemu/Cargo.toml").exists() {
        return Err(format!("{} has no p2-qemu crate", dir.display()));
    }
    Ok(())
}

/// The embsim checkout the runner builds against: `[catalog] embsim`, else
/// the one this tool was built from.
fn embsim_checkout(
    project: &Path,
    project_dir: &Path,
    named: Option<&str>,
) -> Result<PathBuf, String> {
    match named {
        Some(path) => {
            let at = project_dir.join(path);
            let dir = at.canonicalize().map_err(|error| {
                format!(
                    "{}: [catalog] embsim = {path:?} ({}) is not there: {error}",
                    project.display(),
                    at.display()
                )
            })?;
            is_checkout(&dir).map_err(|why| {
                format!(
                    "{}: [catalog] embsim = {path:?} is not an embsim checkout's root: {why}",
                    project.display()
                )
            })?;
            Ok(dir)
        }
        None => {
            let built_from = crate::source_dir();
            let dir = built_from
                .canonicalize()
                .map_err(|error| error.to_string())
                .and_then(|dir| is_checkout(&dir).map(|()| dir));
            dir.map_err(|why| {
                format!(
                    "{}: names catalog crates, and a runner builds them against an embsim \
                     checkout. This embsim was built from {}, which is not one now ({why}); \
                     name one in the project with [catalog] embsim = \"path/to/embsim\" \
                     (PROJECTS.md §10)",
                    project.display(),
                    built_from.display()
                )
            })
        }
    }
}

/// A runner's id: eight hex digits of a 64-bit FNV-1a hash of the crates'
/// directories and the checkout's, in order — stable across runs and
/// machines' hash seeds.
fn runner_id(crates: &[CrateDep], embsim: &Path) -> String {
    let mut hash: u64 = 0xcbf2_9ce4_8422_2325;
    let mut feed = |bytes: &[u8]| {
        for byte in bytes {
            hash ^= u64::from(*byte);
            hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
        }
    };
    for dep in crates {
        feed(dep.dir.as_os_str().as_encoded_bytes());
        feed(b"\n");
    }
    feed(b"embsim=");
    feed(embsim.as_os_str().as_encoded_bytes());
    // Fold the high half in, so the eight digits shown depend on all 64 bits.
    format!("{:08x}", (hash ^ (hash >> 32)) & 0xffff_ffff)
}

/// A path as a TOML string.
fn toml_path(path: &Path) -> String {
    toml_string(&path.to_string_lossy())
}

/// The runner's `Cargo.toml`.
fn manifest(plan: &Plan) -> String {
    let mut text = format!(
        "# The runner `embsim` builds for a project's catalog crates (PROJECTS.md §10,\n\
         # \"The runner\"): the embsim command over the catalogs embsim ships and these\n\
         # crates'. embsim writes this file again whenever it would change; edits are\n\
         # lost.\n\
         [package]\n\
         name = \"{package}\"\n\
         version = \"0.0.0\"\n\
         edition = \"2021\"\n\
         publish = false\n\
         build = \"build.rs\"\n\
         \n\
         [[bin]]\n\
         name = \"{package}\"\n\
         path = \"main.rs\"\n\
         \n\
         [dependencies]\n\
         embsim-cli = {{ path = {cli} }}\n\
         # Named so build.rs is handed QEMU's link arguments when it links QEMU.\n\
         embsim-p2-qemu = {{ path = {qemu} }}\n",
        package = plan.package,
        cli = toml_path(&plan.embsim_path.join("cli")),
        qemu = toml_path(&plan.embsim_path.join("p2-qemu")),
    );
    for dep in &plan.crates {
        text.push_str(&format!(
            "{} = {{ path = {} }}\n",
            dep.package,
            toml_path(&dep.dir)
        ));
    }
    text.push_str("\n# A workspace of its own, whatever directory it sits in.\n[workspace]\n");
    text
}

/// The runner's `main.rs`.
fn main_rs(plan: &Plan) -> String {
    let mut text = String::from(
        "//! The runner `embsim` builds for a project's catalog crates: the embsim\n\
         //! command over the catalogs embsim ships and these crates' (PROJECTS.md\n\
         //! §10, \"The runner\"). embsim writes this file again whenever it would\n\
         //! change; edits are lost.\n\
         \n\
         fn main() -> std::process::ExitCode {\n    embsim_cli::runner_main(&[\n",
    );
    for dep in &plan.crates {
        text.push_str(&format!(
            "        embsim_cli::CatalogCrate {{\n            name: {:?},\n            dir: \
             {:?},\n            register: {}::register,\n        }},\n",
            dep.package,
            dep.dir.to_string_lossy(),
            dep.library
        ));
    }
    text.push_str("    ])\n}\n");
    text
}

/// The runner's `build.rs`: QEMU's link arguments for the runner's binary,
/// as `cli/build.rs` passes them to the `embsim` binary's.
const BUILD_RS: &str = r#"//! Link QEMU into the runner when embsim-p2-qemu linked it, as embsim's own
//! `cli/build.rs` does for the `embsim` binary: embsim-p2-qemu hands a
//! direct dependent its link arguments through its `links = "qemu-p2"`
//! metadata. Written by `embsim`; edits are lost.

use std::env;
use std::fs;

fn main() {
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-env-changed=DEP_QEMU_P2_LINKED");
    println!("cargo:rerun-if-env-changed=DEP_QEMU_P2_LINK_ARGS_FILE");
    if env::var_os("DEP_QEMU_P2_LINKED").is_none() {
        return;
    }
    let file = env::var_os("DEP_QEMU_P2_LINK_ARGS_FILE")
        .expect("embsim-p2-qemu says QEMU is linked and names its link arguments");
    println!("cargo:rerun-if-changed={}", file.to_string_lossy());
    let args = fs::read_to_string(&file)
        .unwrap_or_else(|e| panic!("cannot read {}: {e}", file.to_string_lossy()));
    for arg in args.lines().filter(|arg| !arg.is_empty()) {
        println!("cargo:rustc-link-arg-bins={arg}");
    }
}
"#;

/// Write `content` to `path` unless it already holds exactly that, so an
/// unchanged runner gives Cargo nothing new to look at. Whether it wrote.
fn write_if_changed(path: &Path, content: &str) -> Result<bool, String> {
    if std::fs::read_to_string(path).is_ok_and(|now| now == content) {
        return Ok(false);
    }
    std::fs::write(path, content)
        .map(|()| true)
        .map_err(|error| format!("cannot write {}: {error}", path.display()))
}

/// The runner's three files.
fn write_runner(plan: &Plan) -> Result<(), String> {
    std::fs::create_dir_all(&plan.dir)
        .map_err(|error| format!("cannot make {}: {error}", plan.dir.display()))?;
    write_if_changed(&plan.dir.join("Cargo.toml"), &manifest(plan))?;
    write_if_changed(&plan.dir.join("main.rs"), &main_rs(plan))?;
    write_if_changed(&plan.dir.join("build.rs"), BUILD_RS)?;
    Ok(())
}

/// The file beside the runner's `Cargo.lock` holding the seed it was last
/// seeded from: a seed that differs from it seeds the lock again.
const LOCK_SEED: &str = "Cargo.lock.seed";

/// Seed the runner's `Cargo.lock` ([`lock_seed`]) when it has none, or when
/// the seed changed since it was seeded: a lock file of the workspace or of
/// embsim that moved. Between seeds the lock is Cargo's, as it resolved it.
fn seed_lock(plan: &Plan, workspace: Option<&Workspace>) -> Result<(), String> {
    let Some(seed) = lock_seed(plan, workspace)? else {
        return Ok(());
    };
    let lock = plan.dir.join("Cargo.lock");
    let changed = write_if_changed(&plan.dir.join(LOCK_SEED), &seed)?;
    if changed || !lock.exists() {
        std::fs::write(&lock, &seed)
            .map_err(|error| format!("cannot write {}: {error}", lock.display()))?;
    }
    Ok(())
}

/// The lock file a runner starts from: the catalog workspace's
/// `Cargo.lock`, and every package of embsim's own `Cargo.lock` whose name
/// the workspace's does not lock. So a dependency the workspace builds
/// keeps the version the workspace builds it at, one only embsim has takes
/// the version embsim was tested at, and Cargo resolves afresh only what
/// neither names (or a locked version embsim's requirement does not meet).
/// A name the workspace locks is taken whole from it, never a second
/// version beside it, so the workspace's own entries stay unambiguous.
/// `None` when neither file is there.
fn lock_seed(plan: &Plan, workspace: Option<&Workspace>) -> Result<Option<String>, String> {
    let read = |path: PathBuf| -> Result<Option<(PathBuf, String)>, String> {
        match std::fs::read_to_string(&path) {
            Ok(text) => Ok(Some((path, text))),
            Err(error) if error.kind() == std::io::ErrorKind::NotFound => Ok(None),
            Err(error) => Err(format!("cannot read {}: {error}", path.display())),
        }
    };
    let own = read(plan.embsim.join("Cargo.lock"))?;
    let theirs = match workspace {
        Some(workspace) => read(workspace.root.join("Cargo.lock"))?,
        None => None,
    };
    match (theirs, own) {
        (None, None) => Ok(None),
        (Some((_, text)), None) | (None, Some((_, text))) => Ok(Some(text)),
        (Some((base, text)), Some((extra, _))) if canonical(&base) == canonical(&extra) => {
            Ok(Some(text))
        }
        (Some(base), Some(extra)) => merge_locks(&base, &extra).map(Some),
    }
}

/// The lock file `base`, with every package of `extra` whose name `base`
/// does not lock appended (each a path and its text).
fn merge_locks(base: &(PathBuf, String), extra: &(PathBuf, String)) -> Result<String, String> {
    let parse = |(path, text): &(PathBuf, String)| {
        toml::from_str::<toml::Table>(text)
            .map_err(|error| format!("{} does not parse: {error}", path.display()))
    };
    let mut merged = parse(base)?;
    let added = parse(extra)?;
    let name = |package: &toml::Value| {
        package
            .get("name")
            .and_then(toml::Value::as_str)
            .map(str::to_string)
    };
    let toml::Value::Array(packages) = merged
        .entry("package")
        .or_insert_with(|| toml::Value::Array(Vec::new()))
    else {
        return Err(format!("{}: package is not a list", base.0.display()));
    };
    let locked: BTreeSet<String> = packages.iter().filter_map(name).collect();
    if let Some(toml::Value::Array(more)) = added.get("package") {
        packages.extend(
            more.iter()
                .filter(|package| name(package).is_some_and(|name| !locked.contains(&name)))
                .cloned(),
        );
    }
    toml::to_string(&merged)
        .map_err(|error| format!("the runner's lock seed does not write: {error}"))
}

/// The Cargo workspace a catalog crate belongs to.
#[derive(Debug)]
struct Workspace {
    root: PathBuf,
    target: PathBuf,
}

/// What Cargo says of one catalog crate (`cargo metadata --no-deps`).
#[derive(Debug)]
struct CrateMetadata {
    /// The root of the workspace Cargo puts the crate in: its own
    /// directory when it is in none.
    workspace_root: PathBuf,
    /// That workspace's target directory.
    target: PathBuf,
    /// The crate's embsim dependencies that are built into the runner
    /// (normal and build), by package name, at the path Cargo resolved.
    embsim_deps: Vec<(String, PathBuf)>,
}

impl CrateMetadata {
    /// Read `metadata` for the package whose manifest is `manifest`.
    fn read(metadata: &serde_json::Value, manifest: &Path) -> Option<Self> {
        let path = |value: &serde_json::Value| value.as_str().map(PathBuf::from);
        let workspace_root = path(metadata.get("workspace_root")?)?;
        let target = path(metadata.get("target_directory")?)?;
        let wanted = canonical(manifest);
        let package = metadata
            .get("packages")?
            .as_array()?
            .iter()
            .find(|package| {
                package
                    .get("manifest_path")
                    .and_then(path)
                    .is_some_and(|found| canonical(&found) == wanted)
            })?;
        let embsim_deps = package
            .get("dependencies")
            .and_then(serde_json::Value::as_array)
            .map(|deps| {
                deps.iter()
                    .filter(|dep| {
                        // Dev-dependencies are not built into the runner.
                        dep.get("kind")
                            .and_then(serde_json::Value::as_str)
                            .is_none_or(|kind| kind == "build")
                    })
                    .filter_map(|dep| {
                        let name = dep.get("name")?.as_str()?;
                        let at = dep.get("path").and_then(path)?;
                        name.starts_with("embsim-").then(|| (name.to_string(), at))
                    })
                    .collect()
            })
            .unwrap_or_default();
        Some(Self {
            workspace_root,
            target,
            embsim_deps,
        })
    }
}

/// The workspace the first catalog crate is a member of, if any. A package
/// that is its own workspace root without a `[workspace]` table of its own
/// — a crate `embsim new --catalog` starts outside any workspace — is in
/// none: Cargo would name its own directory, and its `target/` there.
fn workspace_of(plan: &Plan, metadata: &[Option<CrateMetadata>]) -> Option<Workspace> {
    let (dep, metadata) = plan.crates.first().zip(metadata.first())?;
    let metadata = metadata.as_ref()?;
    if canonical(&metadata.workspace_root) == dep.dir && !declares_workspace(&dep.dir) {
        return None;
    }
    Some(Workspace {
        root: metadata.workspace_root.clone(),
        target: metadata.target.clone(),
    })
}

/// Whether the manifest in `dir` has a `[workspace]` table.
fn declares_workspace(dir: &Path) -> bool {
    std::fs::read_to_string(dir.join("Cargo.toml"))
        .ok()
        .and_then(|text| toml::from_str::<toml::Table>(&text).ok())
        .is_some_and(|table| table.contains_key("workspace"))
}

/// The embsim checkout as the runner's manifest spells it: the path the
/// catalog crates' embsim dependencies reach it by, so Cargo, which takes
/// two spellings of one directory (through a symlink) for two packages,
/// sees one. Refused, naming both directories, when a crate's embsim
/// dependency is not in the checkout, or when the crates spell it two
/// ways. A crate Cargo could not read is left to the build's own check.
fn one_embsim(plan: &Plan, metadata: &[Option<CrateMetadata>]) -> Result<PathBuf, String> {
    let mut spellings: BTreeMap<PathBuf, String> = BTreeMap::new();
    for (dep, metadata) in plan.crates.iter().zip(metadata) {
        let Some(metadata) = metadata else { continue };
        for (name, at) in &metadata.embsim_deps {
            let real = canonical(at);
            let Ok(inside) = real.strip_prefix(&plan.embsim) else {
                return Err(another_copy(plan, dep, name, &real));
            };
            let mut root = at.clone();
            for _ in inside.components() {
                root.pop();
            }
            if canonical(&root) != plan.embsim {
                // A link inside the checkout: spell it canonically.
                root.clone_from(&plan.embsim);
            }
            spellings.entry(root).or_insert_with(|| dep.package.clone());
        }
    }
    if spellings.len() > 1 {
        let ways: Vec<String> = spellings
            .iter()
            .map(|(root, package)| format!("{} ({package})", root.display()))
            .collect();
        return Err(format!(
            "{}: the catalog crates reach the embsim checkout {} by {} paths, {}. Cargo takes \
             each path for a copy of embsim of its own, and a runner holds one: spell every \
             catalog crate's embsim dependencies by one of them",
            plan.project.display(),
            plan.embsim.display(),
            spellings.len(),
            ways.join(" and ")
        ));
    }
    Ok(spellings
        .into_keys()
        .next()
        .unwrap_or_else(|| plan.embsim.clone()))
}

/// The refusal for a catalog crate whose embsim dependency `name`, at
/// `real` once links are followed, is outside the runner's checkout.
fn another_copy(plan: &Plan, dep: &CrateDep, name: &str, real: &Path) -> String {
    let mut message = format!(
        "{}: catalog crate {} takes {name} from {}, which is not in the embsim checkout the \
         runner builds against, {} ({}). A runner holds one copy of embsim: two would be two \
         virtual clocks, and a part on one would wait on time nobody advances. Point the \
         crate's embsim dependencies at {}",
        plan.project.display(),
        dep.package,
        real.display(),
        plan.embsim.display(),
        plan.embsim_source(),
        plan.embsim.display()
    );
    let other = real.ancestors().find(|dir| is_checkout(dir).is_ok());
    if let Some(other) = other {
        let path = crate::checklist::relative_path(other, plan.project_dir())
            .unwrap_or_else(|_| other.to_path_buf());
        message.push_str(&format!(
            ", or build against the checkout it uses: [catalog] embsim = {}",
            toml_string(&path.to_string_lossy())
        ));
    }
    message.push_str(" (PROJECTS.md §10)");
    message
}

/// What a runner build gave.
enum Build {
    Built { executable: PathBuf, copies: Copies },
    Failed { copies: Copies },
}

/// Each embsim library the build reported, and the directories it came
/// from.
type Copies = BTreeMap<String, BTreeSet<PathBuf>>;

/// The libraries that came from more than one place, as a sentence; `None`
/// when every one came from one.
fn two_copies(copies: &Copies) -> Option<String> {
    let doubled: Vec<String> = copies
        .iter()
        .filter(|(_, dirs)| dirs.len() > 1)
        .map(|(library, dirs)| {
            format!(
                "{library} from {}",
                dirs.iter()
                    .map(|dir| dir.display().to_string())
                    .collect::<Vec<_>>()
                    .join(" and from ")
            )
        })
        .collect();
    (!doubled.is_empty()).then(|| doubled.join("; "))
}

/// The `cargo` the runner is built with: `$CARGO` (what Cargo sets for a
/// program it runs), else `cargo` on the `PATH`.
struct Cargo {
    program: OsString,
    from_env: bool,
}

impl Cargo {
    fn find() -> Self {
        match std::env::var_os("CARGO").filter(|cargo| !cargo.is_empty()) {
            Some(program) => Self {
                program,
                from_env: true,
            },
            None => Self {
                program: OsString::from("cargo"),
                from_env: false,
            },
        }
    }

    /// `cargo` with `args`, run in `dir`.
    fn command(&self, dir: &Path, args: &[&OsStr]) -> Command {
        let mut command = Command::new(&self.program);
        command.args(args).current_dir(dir);
        // The tool links QEMU into a runner when it was linked into the
        // tool, unless the environment names a tree itself.
        if std::env::var_os("EMBSIM_QEMU_P2_BUILD").is_none() {
            if let Some(tree) = option_env!("EMBSIM_QEMU_TREE") {
                command.env("EMBSIM_QEMU_P2_BUILD", tree);
            }
        }
        command
    }

    /// Why `error`, from starting Cargo, means there is no runner.
    fn missing(&self, plan: &Plan, error: &std::io::Error) -> String {
        let which = if self.from_env {
            format!(
                "$CARGO names {}, which does not start ({error})",
                self.program.to_string_lossy()
            )
        } else {
            format!("no `cargo` on the PATH starts ({error})")
        };
        format!(
            "{} names catalog crates ({}), which embsim builds into a runner with Cargo, and \
             {which}. Install Rust's toolchain (https://rustup.rs), or run the project from a \
             binary of its own over embsim_cli::main_with (PROJECTS.md §10)",
            plan.project.display(),
            plan.crate_names()
        )
    }

    /// What `cargo metadata --no-deps` says of each catalog crate, in
    /// order: `None` for a crate Cargo cannot read on its own (one in a
    /// directory a workspace covers without listing it). An error only when
    /// Cargo does not start.
    fn crate_metadata(&self, plan: &Plan) -> Result<Vec<Option<CrateMetadata>>, String> {
        let mut all = Vec::with_capacity(plan.crates.len());
        for dep in &plan.crates {
            let manifest = dep.dir.join("Cargo.toml");
            let output = self
                .command(
                    &dep.dir,
                    &[
                        OsStr::new("metadata"),
                        OsStr::new("--no-deps"),
                        OsStr::new("--format-version"),
                        OsStr::new("1"),
                        OsStr::new("--manifest-path"),
                        manifest.as_os_str(),
                    ],
                )
                .stderr(Stdio::null())
                .output()
                .map_err(|error| self.missing(plan, &error))?;
            all.push(
                output
                    .status
                    .success()
                    .then(|| serde_json::from_slice(&output.stdout).ok())
                    .flatten()
                    .and_then(|metadata| CrateMetadata::read(&metadata, &manifest)),
            );
        }
        Ok(all)
    }

    /// The line of Cargo's error that says the runner's dependency graph
    /// holds two packages of one embsim name, when it does: read off a
    /// `cargo metadata` of the runner (which resolves as the build did),
    /// after a build failed.
    fn collision(&self, plan: &Plan) -> Option<String> {
        let manifest = plan.dir.join("Cargo.toml");
        let output = self
            .command(
                &plan.dir,
                &[
                    OsStr::new("metadata"),
                    OsStr::new("--format-version"),
                    OsStr::new("1"),
                    OsStr::new("--manifest-path"),
                    manifest.as_os_str(),
                ],
            )
            .stdout(Stdio::null())
            .output()
            .ok()?;
        if output.status.success() {
            return None;
        }
        String::from_utf8_lossy(&output.stderr)
            .lines()
            .find(|line| line.contains("package collision") && line.contains("embsim-"))
            .map(|line| line.trim().trim_start_matches("error: ").to_string())
    }

    /// `cargo clean` of the runner and the catalog crates, for `--rebuild`.
    fn clean(&self, plan: &Plan, target: &Path, profile: &str) -> Result<(), String> {
        let manifest = plan.dir.join("Cargo.toml");
        let mut args: Vec<&OsStr> = vec![
            OsStr::new("clean"),
            OsStr::new("--manifest-path"),
            manifest.as_os_str(),
            OsStr::new("--target-dir"),
            target.as_os_str(),
            OsStr::new("--profile"),
            OsStr::new(profile),
            OsStr::new("-p"),
            OsStr::new(&plan.package),
        ];
        for dep in &plan.crates {
            args.push(OsStr::new("-p"));
            args.push(OsStr::new(&dep.package));
        }
        let status = self
            .command(&plan.dir, &args)
            .status()
            .map_err(|error| self.missing(plan, &error))?;
        if status.success() {
            Ok(())
        } else {
            Err(format!(
                "--rebuild: `cargo clean` of the runner in {} failed ({status}); its errors are \
                 above",
                plan.dir.display()
            ))
        }
    }

    /// `cargo build` of the runner: its errors and progress to standard
    /// error (only its errors when `quiet`), its report read for the binary
    /// and the embsim libraries.
    fn build(
        &self,
        plan: &Plan,
        target: &Path,
        profile: &str,
        quiet: bool,
    ) -> Result<Build, String> {
        let manifest = plan.dir.join("Cargo.toml");
        let mut args = vec![
            OsStr::new("build"),
            OsStr::new("--manifest-path"),
            manifest.as_os_str(),
            OsStr::new("--target-dir"),
            target.as_os_str(),
            OsStr::new("--profile"),
            OsStr::new(profile),
            OsStr::new("--message-format"),
            OsStr::new("json-render-diagnostics"),
        ];
        if quiet {
            args.push(OsStr::new("--quiet"));
        }
        let mut child = self
            .command(&plan.dir, &args)
            .stdout(Stdio::piped())
            .spawn()
            .map_err(|error| self.missing(plan, &error))?;
        let mut executable = None;
        let mut copies = Copies::new();
        if let Some(stdout) = child.stdout.take() {
            for line in BufReader::new(stdout).lines() {
                let Ok(line) = line else { break };
                let Ok(message) = serde_json::from_str::<serde_json::Value>(&line) else {
                    continue;
                };
                if message.get("reason").and_then(|v| v.as_str()) != Some("compiler-artifact") {
                    continue;
                }
                let target = &message["target"];
                let name = target.get("name").and_then(|v| v.as_str()).unwrap_or("");
                let is_lib = target
                    .get("kind")
                    .and_then(|v| v.as_array())
                    .is_some_and(|kinds| {
                        kinds
                            .iter()
                            .any(|kind| kind.as_str().is_some_and(|kind| kind.contains("lib")))
                    });
                if is_lib && name.starts_with(EMBSIM_LIBRARY) {
                    if let Some(dir) = message
                        .get("manifest_path")
                        .and_then(|v| v.as_str())
                        .and_then(|path| Path::new(path).parent())
                    {
                        copies
                            .entry(name.to_string())
                            .or_default()
                            .insert(dir.to_path_buf());
                    }
                }
                if name == plan.package {
                    if let Some(path) = message.get("executable").and_then(|v| v.as_str()) {
                        executable = Some(PathBuf::from(path));
                    }
                }
            }
        }
        let status = child
            .wait()
            .map_err(|error| format!("cargo did not finish: {error}"))?;
        match (status.success(), executable) {
            (true, Some(executable)) => Ok(Build::Built { executable, copies }),
            (true, None) => Err(format!(
                "cargo built the runner in {} and named no binary for {}",
                plan.dir.display(),
                plan.package
            )),
            (false, _) => Ok(Build::Failed { copies }),
        }
    }
}

/// Refuse to run `project` in a runner built with `crates` unless its
/// `[catalog]` names exactly those crates, in that order, and — when it
/// names one — this runner's embsim checkout.
pub fn check_runner_fits(crates: &[CatalogCrate], project: &Path) -> Result<(), String> {
    let catalog = CatalogTable::of_project(project).map_err(|error| error.to_string())?;
    let held: Vec<PathBuf> = crates
        .iter()
        .map(|catalog| canonical(Path::new(catalog.dir)))
        .collect();
    let project_dir = project
        .parent()
        .filter(|dir| !dir.as_os_str().is_empty())
        .unwrap_or(Path::new("."));
    let named: Vec<PathBuf> = catalog
        .as_ref()
        .map(|catalog| {
            catalog
                .crates
                .iter()
                .map(|path| canonical(&project_dir.join(path)))
                .collect()
        })
        .unwrap_or_default();
    let list = |dirs: &[PathBuf]| {
        if dirs.is_empty() {
            "none".to_string()
        } else {
            dirs.iter()
                .map(|dir| dir.display().to_string())
                .collect::<Vec<_>>()
                .join(", ")
        }
    };
    if named != held {
        return Err(format!(
            "{}: this runner holds the catalog crates {}, and the project names {}: run the \
             project with `embsim`, which builds the runner its [catalog] names (PROJECTS.md \
             §10)",
            project.display(),
            list(&held),
            list(&named)
        ));
    }
    if let Some(path) = catalog
        .as_ref()
        .and_then(|catalog| catalog.embsim.as_deref())
    {
        let wanted = canonical(&project_dir.join(path));
        let own = canonical(&crate::source_dir());
        if wanted != own {
            return Err(format!(
                "{}: this runner was built from embsim at {}, and the project names [catalog] \
                 embsim = {path:?} ({}): run the project with `embsim`, which builds the runner \
                 its [catalog] names",
                project.display(),
                own.display(),
                wanted.display()
            ));
        }
    }
    Ok(())
}

/// `path` made canonical where it exists, as written where it does not.
fn canonical(path: &Path) -> PathBuf {
    path.canonicalize().unwrap_or_else(|_| path.to_path_buf())
}

#[cfg(test)]
mod tests {
    use rstest::rstest;

    use super::*;

    fn dep(dir: &str) -> CrateDep {
        CrateDep {
            package: "p".to_string(),
            library: "p".to_string(),
            dir: PathBuf::from(dir),
        }
    }

    #[rstest]
    fn a_runner_id_is_eight_hex_digits_that_follow_the_crates_and_the_checkout() {
        let embsim = Path::new("/e");
        let one = runner_id(&[dep("/a")], embsim);
        assert_eq!(one.len(), 8);
        assert!(one.chars().all(|c| c.is_ascii_hexdigit()), "{one}");
        assert_eq!(one, runner_id(&[dep("/a")], embsim));
        assert_ne!(one, runner_id(&[dep("/b")], embsim));
        assert_ne!(one, runner_id(&[dep("/a")], Path::new("/f")));
        assert_ne!(
            runner_id(&[dep("/a"), dep("/b")], embsim),
            runner_id(&[dep("/b"), dep("/a")], embsim)
        );
    }

    #[rstest]
    fn a_lock_seed_keeps_the_workspaces_versions_and_adds_what_only_embsim_locks() {
        let lock = |packages: &[(&str, &str)]| {
            let mut text = String::from("version = 4\n");
            for (name, version) in packages {
                text.push_str(&format!(
                    "\n[[package]]\nname = \"{name}\"\nversion = \"{version}\"\nsource = \
                     \"registry+https://github.com/rust-lang/crates.io-index\"\n"
                ));
            }
            text
        };
        let base = (
            PathBuf::from("/w/Cargo.lock"),
            lock(&[("clap", "4.5.57"), ("libc", "0.2.170")]),
        );
        let extra = (
            PathBuf::from("/e/Cargo.lock"),
            lock(&[
                ("clap", "4.6.7"),
                ("serde_json", "1.0.140"),
                ("toml", "0.8.23"),
            ]),
        );
        let merged: toml::Table = toml::from_str(&merge_locks(&base, &extra).unwrap()).unwrap();
        assert_eq!(merged["version"].as_integer(), Some(4));
        let packages: Vec<(String, String)> = merged["package"]
            .as_array()
            .unwrap()
            .iter()
            .map(|package| {
                (
                    package["name"].as_str().unwrap().to_string(),
                    package["version"].as_str().unwrap().to_string(),
                )
            })
            .collect();
        let packages: Vec<(&str, &str)> = packages
            .iter()
            .map(|(name, version)| (name.as_str(), version.as_str()))
            .collect();
        assert_eq!(
            packages,
            [
                ("clap", "4.5.57"),
                ("libc", "0.2.170"),
                ("serde_json", "1.0.140"),
                ("toml", "0.8.23")
            ]
        );
    }

    #[rstest]
    fn two_copies_names_each_library_from_more_than_one_place() {
        let mut copies = Copies::new();
        copies
            .entry("embsim_core".to_string())
            .or_default()
            .extend([PathBuf::from("/x/core"), PathBuf::from("/y/core")]);
        copies
            .entry("embsim_board".to_string())
            .or_default()
            .insert(PathBuf::from("/x/board"));
        assert_eq!(
            two_copies(&copies).as_deref(),
            Some("embsim_core from /x/core and from /y/core")
        );
        copies.remove("embsim_core");
        assert_eq!(two_copies(&copies), None);
    }
}
