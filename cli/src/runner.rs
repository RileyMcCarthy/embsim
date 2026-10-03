//! The runner: a project's own catalog crates built into the command.
//!
//! A project whose file names catalog crates (`[catalog] crates = [...]`)
//! runs in a **runner**: the embsim command over the catalogs embsim ships
//! and the crates', one binary Cargo compiles against one copy of embsim
//! ([`crate::runner_main`]; `PROJECTS.md` §10, `NODES.md` §13). There is no
//! plugin interface. The runner has one of two owners:
//!
//! - **The project** (`[catalog] runner = "DIR"`): a binary crate in the
//!   project's own Cargo workspace. The tool builds it with the
//!   workspace's lock file, `--locked` once there is one, in the
//!   workspace's target directory and profile settings, and `exec`s it.
//!   Which embsim it holds is the crate's own dependency.
//! - **The tool**, for a project without one: a crate it writes into
//!   `.embsim/runner-<id>/` beside the project. Its embsim is the one the
//!   first catalog crate's `embsim-boards` dependency names — a path, a git
//!   source at a revision, or a release — and every crate's embsim
//!   dependencies must name the same. Its lock file is kept beside the
//!   project as `embsim.lock`: written by the first build, and once it is
//!   there every build is `--locked` against it, so every machine builds
//!   the same runner.
//!
//! What the tool decides for a runner of its own, in order:
//!
//! 1. **The crates**: each `[catalog] crates` entry, relative to the project
//!    file, is a directory with a `Cargo.toml` whose package has a library.
//! 2. **The runner's directory**: `.embsim/runner-<id>/`, `<id>` a hash of
//!    the crates' package names, which is also what the lock file holds of
//!    the runner, so a committed `embsim.lock` fits on any machine.
//!    `.embsim/` carries a `.gitignore` that keeps it out of version
//!    control.
//! 3. **One embsim**: `cargo metadata --no-deps` on each crate gives its
//!    embsim dependencies. Every one must come from where the first
//!    crate's `embsim-boards` does — one checkout, spelled one way (Cargo
//!    takes two spellings of a directory for two packages), or one git
//!    revision, or the release registry — or the tool refuses before it
//!    builds, naming both: two copies would be two virtual clocks, and a
//!    part on one would wait on time nobody advances. `links =
//!    "embsim-core"` makes Cargo refuse any second copy the check cannot
//!    see, when it resolves.
//! 4. **The files**: `Cargo.toml` (`embsim-cli` from that same source, and
//!    each crate by path) and `main.rs`, each rewritten only when its
//!    content would change, so Cargo sees nothing new.
//! 5. **Where it builds**: the target directory of the Cargo workspace the
//!    first crate is a member of (`CARGO_TARGET_DIR` included, as Cargo
//!    itself reads it), so what that workspace built is reused; for a crate
//!    in no workspace — a package that is its own root, as `embsim new
//!    --catalog` starts one — `.embsim/target` (or `CARGO_TARGET_DIR`), so
//!    no build lands in the crate's source tree.
//! 6. **The lock file**: `embsim.lock` beside the project, copied in and
//!    built `--locked`; without one, the runner's lock is seeded from the
//!    workspace's lock file, with every package of embsim's own lock file
//!    (for a checkout) whose name the workspace's does not lock, built, and
//!    kept as `embsim.lock`. The profile is `release`, unless
//!    `EMBSIM_RUNNER_PROFILE` names another.
//! 7. **After the build**: every `embsim_*` library the build reports must
//!    come from one place; a build Cargo refused for two copies is said to
//!    be that, and one refused for a lock file that no longer fits says
//!    how to lock again.
//!
//! Either way the tool measures what it built — each catalog crate's
//! version, directory and git revision, and the project's runner crate's —
//! and hands it to the runner to print with its own build facts
//! ([`crate::provenance`]).

use std::collections::{BTreeMap, BTreeSet};
use std::convert::Infallible;
use std::ffi::{OsStr, OsString};
use std::io::{BufRead, BufReader, Write};
use std::os::unix::process::CommandExt;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};

use embsim_board::{state_dir, CatalogTable, ProjectHead};

use crate::checklist::{relative_path, toml_string};
use crate::provenance::{git_state, PROVENANCE_ENV};
use crate::CatalogCrate;

/// The profile a runner builds in, unless `EMBSIM_RUNNER_PROFILE` names
/// another: a run is the ISS's or a model's hot loop, and the build is paid
/// once.
const DEFAULT_PROFILE: &str = "release";

/// The prefix of every embsim library's crate name: the libraries a runner
/// must link exactly one copy of.
const EMBSIM_LIBRARY: &str = "embsim_";

/// The lock file of a runner the tool writes, kept beside the project.
pub const LOCK_FILE: &str = "embsim.lock";

/// The embsim crate a catalog crate's embsim is read from: every catalog
/// crate depends on it, for the `CatalogSet` its registration function
/// takes.
const BOARDS: &str = "embsim-boards";

/// The embsim crate a runner depends on, for `runner_main`.
const CLI: &str = "embsim-cli";

/// embsim's own crates a catalog crate or a runner may depend on, each a
/// directory of an embsim checkout's root named after it: every one a
/// runner links must come from one embsim.
const EMBSIM_CRATES: [&str; 6] = [
    "embsim-core",
    "embsim-board",
    "embsim-boards",
    "embsim-models",
    "embsim-p2-qemu",
    "embsim-cli",
];

/// The directory under the target directory a profile's binaries land in.
fn profile_dir(profile: &str) -> &str {
    match profile {
        "dev" | "test" => "debug",
        "bench" => "release",
        other => other,
    }
}

/// The profile runners build in.
fn runner_profile() -> String {
    std::env::var("EMBSIM_RUNNER_PROFILE")
        .ok()
        .filter(|profile| !profile.is_empty())
        .unwrap_or_else(|| DEFAULT_PROFILE.to_string())
}

// ============================================================
// Where embsim comes from
// ============================================================

/// Where a catalog crate's embsim comes from, as its manifest says.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum EmbsimSource {
    /// An embsim checkout by path: its root, spelled as the crate spells
    /// it (each embsim crate is a directory of the root).
    Path(PathBuf),
    /// A git repository: its URL, and the `rev`, `branch` or `tag` the
    /// dependency names, if it names one.
    Git {
        url: String,
        reference: Option<(String, String)>,
    },
    /// A release from crates.io, at a version requirement (`^0.2`).
    Registry { req: String },
}

impl EmbsimSource {
    /// The source of one dependency in `cargo metadata`'s report: its
    /// `path`, else its `source`.
    fn of_dependency(dep: &serde_json::Value) -> Result<Self, String> {
        if let Some(path) = dep.get("path").and_then(serde_json::Value::as_str) {
            let path = Path::new(path);
            return Ok(Self::Path(
                path.parent()
                    .map_or_else(|| path.to_path_buf(), Path::to_path_buf),
            ));
        }
        let source = dep
            .get("source")
            .and_then(serde_json::Value::as_str)
            .unwrap_or_default();
        if let Some(git) = source.strip_prefix("git+") {
            return Ok(Self::git(git));
        }
        if source.starts_with("registry+") {
            if let Some(registry) = dep.get("registry").and_then(serde_json::Value::as_str) {
                return Err(format!(
                    "the registry {registry}, which a runner the tool writes cannot name; take \
                     embsim from crates.io, a git source or a path, or give the project a runner \
                     of its own ([catalog] runner)"
                ));
            }
            let req = dep
                .get("req")
                .and_then(serde_json::Value::as_str)
                .unwrap_or("*");
            return Ok(Self::Registry {
                req: req.to_string(),
            });
        }
        Err(format!("a source this embsim does not know ({source:?})"))
    }

    /// `URL?rev=…`, as Cargo writes a git source.
    fn git(text: &str) -> Self {
        let (url, query) = text.split_once('?').unwrap_or((text, ""));
        let reference = query
            .split('&')
            .filter_map(|pair| pair.split_once('='))
            .find(|(key, _)| matches!(*key, "rev" | "branch" | "tag"))
            .map(|(key, value)| (key.to_string(), value.to_string()));
        Self::Git {
            url: url.to_string(),
            reference,
        }
    }

    /// The source a manifest's dependency table `value` names, `dir` the
    /// directory a path in it is relative to.
    fn of_manifest(value: &toml::Value, dir: &Path) -> Option<Self> {
        if let Some(version) = value.as_str() {
            return Some(Self::Registry {
                req: version.to_string(),
            });
        }
        let table = value.as_table()?;
        if let Some(path) = table.get("path").and_then(toml::Value::as_str) {
            // As Cargo spells a path dependency: `..` taken lexically, links
            // kept.
            let path = normalize(&dir.join(path));
            return Some(Self::Path(
                path.parent()
                    .map_or_else(|| path.clone(), Path::to_path_buf),
            ));
        }
        if let Some(url) = table.get("git").and_then(toml::Value::as_str) {
            let reference = ["rev", "branch", "tag"].iter().find_map(|key| {
                table
                    .get(*key)
                    .and_then(toml::Value::as_str)
                    .map(|value| ((*key).to_string(), value.to_string()))
            });
            return Some(Self::Git {
                url: url.to_string(),
                reference,
            });
        }
        table
            .get("version")
            .and_then(toml::Value::as_str)
            .map(|req| Self::Registry {
                req: req.to_string(),
            })
    }

    /// What a message calls it: `at /home/me/embsim`, `from git
    /// https://github.com/… rev 1a2b3c`, `from crates.io ^0.2`.
    pub(crate) fn describe(&self) -> String {
        match self {
            Self::Path(root) => format!("at {}", root.display()),
            Self::Git { url, reference } => match reference {
                Some((key, value)) => format!("from git {url} {key} {value}"),
                None => format!("from git {url}, its default branch"),
            },
            Self::Registry { req } => format!("from crates.io {req}"),
        }
    }

    /// Whether `other` is the same copy: one checkout, however spelled;
    /// one repository at one revision; any release (which version one
    /// graph takes is Cargo's to resolve, and `links` refuses two).
    pub(crate) fn same_copy(&self, other: &Self) -> bool {
        match (self, other) {
            (Self::Path(a), Self::Path(b)) => canonical(a) == canonical(b),
            (
                Self::Git { url, reference },
                Self::Git {
                    url: other_url,
                    reference: other_reference,
                },
            ) => same_url(url, other_url) && reference == other_reference,
            (Self::Registry { .. }, Self::Registry { .. }) => true,
            _ => false,
        }
    }

    /// The dependency on the embsim crate in directory `crate_dir` of the
    /// workspace (`board`, `cli`), as a manifest in `from` writes it: a
    /// path relative to `from` when `relative`, else whole; the git source
    /// with `version` checked against it; the release requirement.
    pub(crate) fn dependency(
        &self,
        crate_dir: &str,
        from: &Path,
        relative: bool,
        version: Option<&str>,
    ) -> Result<String, String> {
        Ok(match self {
            Self::Path(root) => {
                let path = root.join(crate_dir);
                let path = if relative {
                    relative_path(&path, from)?
                } else {
                    path
                };
                format!("{{ path = {} }}", toml_string(&path.to_string_lossy()))
            }
            Self::Git { url, reference } => {
                let mut table = format!("{{ git = {}", toml_string(url));
                if let Some((key, value)) = reference {
                    table.push_str(&format!(", {key} = {}", toml_string(value)));
                }
                if let Some(version) = version {
                    table.push_str(&format!(", version = {}", toml_string(version)));
                }
                table.push_str(" }");
                table
            }
            Self::Registry { req } => toml_string(req),
        })
    }
}

/// Two git URLs for one repository: equal but for a trailing `/` or
/// `.git`, and case in the host.
fn same_url(a: &str, b: &str) -> bool {
    let norm = |url: &str| {
        url.trim_end_matches('/')
            .trim_end_matches(".git")
            .to_ascii_lowercase()
    };
    norm(a) == norm(b)
}

/// Whether `dir` is an embsim checkout: its `cli/Cargo.toml` is the
/// `embsim-cli` package.
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
    if name == Some("embsim-cli") {
        Ok(())
    } else {
        Err(format!(
            "{} is not the embsim-cli package",
            manifest.display()
        ))
    }
}

/// The embsim the catalog crate in `dir` depends on, read from its
/// manifest — `embsim-boards` under its own name or renamed, from the
/// workspace's table when it says `workspace = true` — for when Cargo is
/// not there to say.
pub(crate) fn declared_source(dir: &Path) -> Option<EmbsimSource> {
    declared_source_of(dir, BOARDS)
}

/// The source the manifest in `dir` gives its dependency on the embsim
/// crate `package` ([`declared_source`]; `embsim-cli` for a runner crate).
fn declared_source_of(dir: &Path, package: &str) -> Option<EmbsimSource> {
    let read = |path: &Path| -> Option<toml::Table> {
        toml::from_str(&std::fs::read_to_string(path).ok()?).ok()
    };
    let manifest = read(&dir.join("Cargo.toml"))?;
    let dependencies = manifest.get("dependencies")?.as_table()?;
    let (_, value) = dependencies.iter().find(|(key, value)| {
        key.as_str() == package
            || value.get("package").and_then(toml::Value::as_str) == Some(package)
    })?;
    if value.get("workspace").and_then(toml::Value::as_bool) == Some(true) {
        let root = dir.ancestors().skip(1).find(|ancestor| {
            read(&ancestor.join("Cargo.toml")).is_some_and(|table| table.contains_key("workspace"))
        })?;
        let workspace = read(&root.join("Cargo.toml"))?;
        let value = workspace
            .get("workspace")?
            .get("dependencies")?
            .get(package)?
            .clone();
        return EmbsimSource::of_manifest(&value, root);
    }
    EmbsimSource::of_manifest(value, dir)
}

// ============================================================
// The plan
// ============================================================

/// One catalog crate, as the runner depends on it.
#[derive(Debug, Clone)]
struct CrateDep {
    /// The package name, the dependency's key.
    package: String,
    /// The library's crate name, as Rust code names it.
    library: String,
    /// The directory, canonical.
    dir: PathBuf,
    /// The version its manifest gives, when it gives one itself.
    version: Option<String>,
}

/// Everything the tool decided about one project's runner.
#[derive(Debug)]
struct Plan {
    /// The project file, as the command line gave it.
    project: PathBuf,
    crates: Vec<CrateDep>,
    /// The embsim the runner builds against, once the crates are read.
    source: Option<EmbsimSource>,
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
        crate_names(&self.crates)
    }

    /// The project file's directory, as the command line reaches it.
    fn project_dir(&self) -> &Path {
        project_dir(&self.project)
    }

    /// Where the runner's embsim comes from, for a message.
    fn source_said(&self) -> String {
        self.source
            .as_ref()
            .map_or_else(|| "unknown".to_string(), EmbsimSource::describe)
    }
}

/// The package names of `crates`, comma-separated.
fn crate_names(crates: &[CrateDep]) -> String {
    crates
        .iter()
        .map(|dep| dep.package.as_str())
        .collect::<Vec<_>>()
        .join(", ")
}

/// The directory of the project file `project`, as the command line
/// reaches it.
fn project_dir(project: &Path) -> &Path {
    project
        .parent()
        .filter(|dir| !dir.as_os_str().is_empty())
        .unwrap_or(Path::new("."))
}

/// Write or find, build and `exec` the runner for `project`, whose
/// `[catalog]` is `catalog`, with the command line `args`. Returns only
/// when it could not hand over, with the reason; Cargo's own output goes to
/// standard error as it comes.
pub fn hand_over(
    project: &Path,
    catalog: &CatalogTable,
    rebuild: bool,
    args: &[OsString],
    err: &mut dyn Write,
) -> Result<Infallible, String> {
    match catalog.runner.as_deref() {
        Some(runner) => hand_over_own(project, catalog, runner, rebuild, args, err),
        None => hand_over_written(project, catalog, rebuild, args, err),
    }
}

/// [`hand_over`] for a runner the tool writes.
fn hand_over_written(
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
    let metadata = match cargo.crate_metadata(&plan.crates, &plan.project) {
        Ok(metadata) => metadata,
        Err(missing) => {
            // Written either way, where the crate's manifest says which
            // embsim, so the runner can be built by hand.
            plan.source = declared_source(&plan.crates[0].dir);
            if plan.source.is_some() {
                write_runner(&plan)?;
            }
            return Err(missing);
        }
    };
    plan.source = Some(one_embsim(&plan, &metadata)?);
    write_runner(&plan)?;
    let workspace = workspace_of(&plan, &metadata);
    let locked = lock(&plan, workspace.as_ref())?;
    let target = workspace.as_ref().map_or_else(
        || {
            std::env::var_os("CARGO_TARGET_DIR").map_or_else(
                || plan.dir.parent().unwrap_or(&plan.dir).join("target"),
                PathBuf::from,
            )
        },
        |workspace| workspace.target.clone(),
    );
    let profile = runner_profile();
    let _ = writeln!(
        err,
        "embsim: building the runner for {} ({}, embsim {}) in {}",
        plan.project.display(),
        plan.crate_names(),
        plan.source_said(),
        plan.shown.display()
    );
    let _ = err.flush();
    let manifest = plan.dir.join("Cargo.toml");
    if rebuild {
        let mut packages = vec![plan.package.as_str()];
        packages.extend(plan.crates.iter().map(|dep| dep.package.as_str()));
        cargo.clean(&manifest, Some(&target), &profile, &packages)?;
    }
    // A runner built before is brought up to date quietly: Cargo's errors
    // still show, its progress and the warnings it replays from earlier
    // builds do not. The first build, and a rebuild, show it all.
    let quiet = !rebuild
        && target
            .join(profile_dir(&profile))
            .join(&plan.package)
            .exists();
    let build = Build {
        manifest: &manifest,
        target: Some(&target),
        profile: &profile,
        quiet,
        locked,
        package: &plan.package,
        bin: &plan.package,
    };
    let executable = match cargo.build(&build)? {
        Built::Done { executable, copies } => {
            if let Some(copies) = two_copies(&copies) {
                return Err(format!(
                    "the runner for {} links two copies of embsim: {copies}. Each copy has its \
                     own virtual clock, and a part on one waits on time nobody advances. Point \
                     every catalog crate's embsim dependencies, and those of the crates they \
                     depend on, at embsim {} (PROJECTS.md §10)",
                    plan.project.display(),
                    plan.source_said()
                ));
            }
            executable
        }
        Built::Failed { copies } => {
            return Err(build_failure(&cargo, &plan, &manifest, locked, &copies));
        }
    };
    if !locked {
        save_lock(&plan, err)?;
    }
    let mut provenance: Vec<String> = Vec::new();
    for (dep, metadata) in plan.crates.iter().zip(&metadata) {
        let version = metadata
            .as_ref()
            .map(|metadata| metadata.version.as_str())
            .or(dep.version.as_deref());
        provenance.push(crate_provenance(
            "catalog crate",
            &dep.package,
            version,
            &dep.dir,
        ));
    }
    Err(exec(&executable, args, &provenance))
}

/// Why the runner `plan` did not build: two copies of embsim, a lock file
/// that no longer fits, or the crates.
fn build_failure(
    cargo: &Cargo,
    plan: &Plan,
    manifest: &Path,
    locked: bool,
    copies: &Copies,
) -> String {
    let diagnosis = cargo.diagnose(manifest);
    if let Some(unfetched) = &diagnosis.unfetched {
        return format!(
            "the runner for {} did not build: Cargo could not fetch embsim {} ({unfetched}); \
             Cargo's errors are above. {}",
            plan.project.display(),
            plan.source_said(),
            fetchable_embsim()
        );
    }
    let collision = two_copies(copies).or(diagnosis.collision);
    if let Some(copies) = collision {
        return format!(
            "the runner for {} did not build: it met two copies of embsim ({copies}); Cargo's \
             errors are above. Each copy would have its own virtual clock. Point the embsim \
             dependencies of every catalog crate, and of the crates they depend on, at embsim {}, \
             each spelled the same way (PROJECTS.md §10)",
            plan.project.display(),
            plan.source_said()
        );
    }
    if locked && cargo.lock_does_not_fit(manifest) {
        let saved = plan.project_dir().join(LOCK_FILE);
        return format!(
            "the runner for {} did not build: {} does not lock what the runner now depends on \
             (Cargo's errors are above), and the runner builds --locked against it. Remove it \
             and run again: Cargo locks what changed, and the tool keeps the new lock file \
             there for you to commit (PROJECTS.md §10)",
            plan.project.display(),
            saved.display()
        );
    }
    format!(
        "the runner for {} did not build (catalog crates {}; embsim {}); Cargo's errors are \
         above. A catalog crate is a library with `pub fn register(set: &mut CatalogSet) -> \
         Result<(), ProjectError>` at its root (PROJECTS.md §10)",
        plan.project.display(),
        plan.crate_names(),
        plan.source_said()
    )
}

/// What to do about an embsim Cargo could not fetch.
fn fetchable_embsim() -> String {
    format!(
        "Point the catalog crates' embsim dependencies at a source every machine can fetch — a \
         published release tag (`{{ git = \"{}\", tag = \"v{}\" }}`) or a commit a remote \
         holds — or at an embsim checkout by path; `embsim new --catalog DIR --embsim \
         PATH|URL@REF` writes either (PROJECTS.md §10)",
        env!("CARGO_PKG_REPOSITORY"),
        crate::provenance::VERSION
    )
}

/// `catalog crate rig-catalog 0.1.0: /home/me/rig/sim/catalog, git rev
/// 0123456789ab`: one line the tool hands a runner about a crate it built.
fn crate_provenance(what: &str, package: &str, version: Option<&str>, dir: &Path) -> String {
    let version = version
        .map(|version| format!(" {version}"))
        .unwrap_or_default();
    format!(
        "{what} {package}{version}: {}, {}",
        dir.display(),
        git_state(dir)
    )
}

/// `exec` the runner `executable` with the command line `args` (its
/// `argv[0]` `embsim`, so usage reads as the tool's) and what the tool
/// measured of it in [`PROVENANCE_ENV`]. Returns only the reason it could
/// not.
fn exec(executable: &Path, args: &[OsString], provenance: &[String]) -> String {
    let error = Command::new(executable)
        .arg0("embsim")
        .args(args.iter().skip(1))
        .env(PROVENANCE_ENV, provenance.join("\n"))
        .exec();
    format!("cannot start the runner {}: {error}", executable.display())
}

/// The crates and the runner's directory for `project`.
fn plan(project: &Path, catalog: &CatalogTable) -> Result<Plan, String> {
    let crates = crate_deps(project, catalog)?;
    let id = runner_id(&crates);
    let dir = project_dir(project);
    let state = state_dir(dir)
        .and_then(|state| state.canonicalize())
        .map_err(|error| format!("cannot make {}: {error}", dir.join(".embsim").display()))?;
    let runner = format!("runner-{id}");
    Ok(Plan {
        project: project.to_path_buf(),
        crates,
        source: None,
        dir: state.join(&runner),
        shown: dir.join(".embsim").join(runner),
        package: format!("embsim-runner-{id}"),
    })
}

/// Each crate `[catalog] crates` names, read and checked: no embsim crate,
/// no package twice.
fn crate_deps(project: &Path, catalog: &CatalogTable) -> Result<Vec<CrateDep>, String> {
    let dir = project_dir(project);
    let crates = catalog
        .crates
        .iter()
        .map(|path| crate_dep(project, dir, path))
        .collect::<Result<Vec<_>, _>>()?;
    let mut packages = BTreeSet::new();
    for dep in &crates {
        if EMBSIM_CRATES.contains(&dep.package.as_str()) {
            return Err(format!(
                "{}: [catalog] crates: {} is embsim's own crate {}, which every runner already \
                 holds; a catalog crate is one of the project's",
                project.display(),
                dep.dir.display(),
                dep.package
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
    Ok(crates)
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
    let version = table
        .get("package")
        .and_then(|package| package.get("version"))
        .and_then(toml::Value::as_str)
        .map(str::to_string);
    Ok(CrateDep {
        package,
        library,
        dir,
        version,
    })
}

/// A runner's id: eight hex digits of a 64-bit FNV-1a hash of the crates'
/// package names, in order — stable across runs, machines and where the
/// project sits, so the lock file that names the runner fits on any
/// machine.
fn runner_id(crates: &[CrateDep]) -> String {
    let mut hash: u64 = 0xcbf2_9ce4_8422_2325;
    let mut feed = |bytes: &[u8]| {
        for byte in bytes {
            hash ^= u64::from(*byte);
            hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
        }
    };
    for dep in crates {
        feed(dep.package.as_bytes());
        feed(b"\n");
    }
    // Fold the high half in, so the eight digits shown depend on all 64 bits.
    format!("{:08x}", (hash ^ (hash >> 32)) & 0xffff_ffff)
}

/// A path as a TOML string.
fn toml_path(path: &Path) -> String {
    toml_string(&path.to_string_lossy())
}

/// The runner's `Cargo.toml`.
fn manifest(plan: &Plan) -> Result<String, String> {
    let source = plan
        .source
        .as_ref()
        .ok_or_else(|| "the runner's embsim is not known yet".to_string())?;
    if let EmbsimSource::Path(root) = source {
        is_checkout(root).map_err(|why| {
            format!(
                "{}: the catalog crates take embsim from {}, whose embsim-cli the runner needs: \
                 {why}. Point their embsim dependencies at an embsim checkout",
                plan.project.display(),
                root.display()
            )
        })?;
    }
    let cli = source.dependency("cli", &plan.dir, false, None)?;
    let mut text = format!(
        "# The runner `embsim` builds for a project's catalog crates (PROJECTS.md §10,\n\
         # \"The runner\"): the embsim command over the catalogs embsim ships and these\n\
         # crates', against the embsim the crates depend on. embsim writes this file\n\
         # again whenever it would change; edits are lost.\n\
         [package]\n\
         name = \"{package}\"\n\
         version = \"0.0.0\"\n\
         edition = \"2021\"\n\
         publish = false\n\
         \n\
         [[bin]]\n\
         name = \"{package}\"\n\
         path = \"main.rs\"\n\
         \n\
         [dependencies]\n\
         embsim-cli = {cli}\n",
        package = plan.package,
    );
    for dep in &plan.crates {
        text.push_str(&format!(
            "{} = {{ path = {} }}\n",
            dep.package,
            toml_path(&dep.dir)
        ));
    }
    text.push_str("\n# A workspace of its own, whatever directory it sits in.\n[workspace]\n");
    Ok(text)
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
            "        embsim_cli::CatalogCrate::new(\n            {:?},\n            {:?},\n            \
             {}::register,\n        ),\n",
            dep.package,
            dep.dir.to_string_lossy(),
            dep.library
        ));
    }
    text.push_str("    ])\n}\n");
    text
}

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

/// The runner's two files. A `build.rs` an older embsim wrote (it linked
/// QEMU) is removed: Cargo would run any `build.rs` beside the manifest.
fn write_runner(plan: &Plan) -> Result<(), String> {
    let manifest = manifest(plan)?;
    std::fs::create_dir_all(&plan.dir)
        .map_err(|error| format!("cannot make {}: {error}", plan.dir.display()))?;
    write_if_changed(&plan.dir.join("Cargo.toml"), &manifest)?;
    write_if_changed(&plan.dir.join("main.rs"), &main_rs(plan))?;
    for stale in ["build.rs", "Cargo.lock.seed"] {
        let stale = plan.dir.join(stale);
        if stale.exists() {
            std::fs::remove_file(&stale)
                .map_err(|error| format!("cannot remove {}: {error}", stale.display()))?;
        }
    }
    Ok(())
}

// ============================================================
// The lock file
// ============================================================

/// The runner whose lock `text` is: the name of its one package with no
/// source that is a runner.
fn locked_runner(text: &str) -> Option<String> {
    let table: toml::Table = toml::from_str(text).ok()?;
    table
        .get("package")?
        .as_array()?
        .iter()
        .filter(|package| package.get("source").is_none())
        .filter_map(|package| package.get("name").and_then(toml::Value::as_str))
        .find(|name| name.starts_with("embsim-runner-"))
        .map(str::to_string)
}

/// Put the runner's lock in place: `embsim.lock` beside the project when it
/// is there, to build `--locked` against (`true`); else the lock Cargo left
/// the last time, or a seed ([`lock_seed`]) when there is none (`false`).
fn lock(plan: &Plan, workspace: Option<&Workspace>) -> Result<bool, String> {
    let saved = plan.project_dir().join(LOCK_FILE);
    let lock = plan.dir.join("Cargo.lock");
    match std::fs::read_to_string(&saved) {
        Ok(text) => {
            if let Some(other) = locked_runner(&text).filter(|name| *name != plan.package) {
                return Err(format!(
                    "{}: {} is the lock of another runner ({other}, for catalog crates other \
                     than {}): the projects in one directory that name catalog crates share \
                     its {LOCK_FILE}, so they name the same crates. Give this project a \
                     directory of its own, or remove the file to lock this one's runner",
                    plan.project.display(),
                    saved.display(),
                    plan.crate_names()
                ));
            }
            write_if_changed(&lock, &text)?;
            Ok(true)
        }
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => {
            if !lock.exists() {
                if let Some(seed) = lock_seed(plan, workspace)? {
                    std::fs::write(&lock, seed)
                        .map_err(|error| format!("cannot write {}: {error}", lock.display()))?;
                }
            }
            Ok(false)
        }
        Err(error) => Err(format!("cannot read {}: {error}", saved.display())),
    }
}

/// Keep the lock Cargo resolved for the runner as `embsim.lock` beside the
/// project, and say so.
fn save_lock(plan: &Plan, err: &mut dyn Write) -> Result<(), String> {
    let saved = plan.project_dir().join(LOCK_FILE);
    let text = std::fs::read_to_string(plan.dir.join("Cargo.lock"))
        .map_err(|error| format!("cannot read the runner's Cargo.lock: {error}"))?;
    std::fs::write(&saved, text)
        .map_err(|error| format!("cannot write {}: {error}", saved.display()))?;
    let _ = writeln!(
        err,
        "embsim: wrote {}: the versions this runner was built from. Commit it: from now on the \
         runner builds --locked against it, the same on every machine",
        saved.display()
    );
    Ok(())
}

/// The lock file a runner starts from: the catalog workspace's
/// `Cargo.lock`, and every package of embsim's own `Cargo.lock` (an embsim
/// checkout's) whose name the workspace's does not lock. So a dependency
/// the workspace builds keeps the version the workspace builds it at, one
/// only embsim has takes the version embsim was tested at, and Cargo
/// resolves afresh only what neither names (or a locked version embsim's
/// requirement does not meet). A name the workspace locks is taken whole
/// from it, never a second version beside it, so the workspace's own
/// entries stay unambiguous. `None` when neither file is there.
fn lock_seed(plan: &Plan, workspace: Option<&Workspace>) -> Result<Option<String>, String> {
    let read = |path: PathBuf| -> Result<Option<(PathBuf, String)>, String> {
        match std::fs::read_to_string(&path) {
            Ok(text) => Ok(Some((path, text))),
            Err(error) if error.kind() == std::io::ErrorKind::NotFound => Ok(None),
            Err(error) => Err(format!("cannot read {}: {error}", path.display())),
        }
    };
    let own = match &plan.source {
        Some(EmbsimSource::Path(root)) => read(root.join("Cargo.lock"))?,
        _ => None,
    };
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

// ============================================================
// What Cargo says of the crates
// ============================================================

/// The Cargo workspace a catalog crate belongs to.
#[derive(Debug)]
struct Workspace {
    root: PathBuf,
    target: PathBuf,
}

/// One embsim dependency of a catalog crate.
#[derive(Debug)]
struct EmbsimDep {
    /// The package (`embsim-board`).
    name: String,
    source: Result<EmbsimSource, String>,
}

/// What Cargo says of one catalog crate (`cargo metadata --no-deps`).
#[derive(Debug)]
struct CrateMetadata {
    /// The root of the workspace Cargo puts the crate in: its own
    /// directory when it is in none.
    workspace_root: PathBuf,
    /// That workspace's target directory.
    target: PathBuf,
    /// The crate's version.
    version: String,
    /// The crate's embsim dependencies that are built into the runner
    /// (normal and build), with where each comes from.
    embsim_deps: Vec<EmbsimDep>,
}

impl CrateMetadata {
    /// Read `metadata` for the package whose manifest is `manifest`.
    fn read(metadata: &serde_json::Value, manifest: &Path) -> Option<Self> {
        let path = |value: &serde_json::Value| value.as_str().map(PathBuf::from);
        let workspace_root = path(metadata.get("workspace_root")?)?;
        let target = path(metadata.get("target_directory")?)?;
        let package = package_at(metadata, manifest)?;
        let version = package
            .get("version")
            .and_then(serde_json::Value::as_str)
            .unwrap_or_default()
            .to_string();
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
                        EMBSIM_CRATES.contains(&name).then(|| EmbsimDep {
                            name: name.to_string(),
                            source: EmbsimSource::of_dependency(dep),
                        })
                    })
                    .collect()
            })
            .unwrap_or_default();
        Some(Self {
            workspace_root,
            target,
            version,
            embsim_deps,
        })
    }
}

/// The package of `metadata` whose manifest is `manifest`.
fn package_at<'m>(
    metadata: &'m serde_json::Value,
    manifest: &Path,
) -> Option<&'m serde_json::Value> {
    let wanted = canonical(manifest);
    metadata
        .get("packages")?
        .as_array()?
        .iter()
        .find(|package| {
            package
                .get("manifest_path")
                .and_then(serde_json::Value::as_str)
                .is_some_and(|found| canonical(Path::new(found)) == wanted)
        })
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

/// The embsim the runner builds against: where the first catalog crate's
/// `embsim-boards` comes from, a checkout spelled as the crates spell it.
/// Refused, naming both, when any crate's embsim dependency comes from
/// elsewhere, or the crates spell one checkout two ways. A crate Cargo
/// could not read is left to the build's own check.
fn one_embsim(plan: &Plan, metadata: &[Option<CrateMetadata>]) -> Result<EmbsimSource, String> {
    let first = &plan.crates[0];
    let source = match metadata.first().and_then(Option::as_ref) {
        Some(metadata) => metadata
            .embsim_deps
            .iter()
            .find(|dep| dep.name == BOARDS)
            .map(|dep| dep.source.clone())
            .ok_or_else(|| no_boards(plan, first))?,
        None => declared_source(&first.dir).ok_or_else(|| no_boards(plan, first)),
    }
    .map_err(|why| {
        format!(
            "{}: catalog crate {} takes {BOARDS} from {why}",
            plan.project.display(),
            first.package
        )
    })?;
    let mut spellings: BTreeMap<PathBuf, String> = BTreeMap::new();
    for (dep, metadata) in plan.crates.iter().zip(metadata) {
        let Some(metadata) = metadata else { continue };
        for embsim in &metadata.embsim_deps {
            let theirs = embsim.source.as_ref().map_err(|why| {
                format!(
                    "{}: catalog crate {} takes {} from {why}",
                    plan.project.display(),
                    dep.package,
                    embsim.name
                )
            })?;
            if !theirs.same_copy(&source) {
                return Err(another_copy(
                    plan,
                    first,
                    &source,
                    dep,
                    &embsim.name,
                    theirs,
                ));
            }
            if let EmbsimSource::Path(root) = theirs {
                spellings
                    .entry(root.clone())
                    .or_insert_with(|| dep.package.clone());
            }
        }
    }
    if spellings.len() > 1 {
        let ways: Vec<String> = spellings
            .iter()
            .map(|(root, package)| format!("{} ({package})", root.display()))
            .collect();
        return Err(format!(
            "{}: the catalog crates reach one embsim checkout by {} paths, {}. Cargo takes each \
             path for a copy of embsim of its own, and a runner holds one: spell every catalog \
             crate's embsim dependencies by one of them",
            plan.project.display(),
            spellings.len(),
            ways.join(" and ")
        ));
    }
    Ok(match (source, spellings.into_keys().next()) {
        (EmbsimSource::Path(_), Some(spelled)) => EmbsimSource::Path(spelled),
        (source, _) => source,
    })
}

/// The refusal for a first catalog crate that does not depend on
/// `embsim-boards`.
fn no_boards(plan: &Plan, dep: &CrateDep) -> String {
    format!(
        "{}: catalog crate {} does not depend on {BOARDS}: its registration function takes \
         embsim_boards::catalog::CatalogSet, and the runner takes embsim from where the first \
         catalog crate's {BOARDS} comes from (PROJECTS.md §10)",
        plan.project.display(),
        dep.package
    )
}

/// The refusal for catalog crate `dep`, whose embsim dependency `name`
/// comes from `theirs`, when `first`'s `embsim-boards` comes from `source`.
fn another_copy(
    plan: &Plan,
    first: &CrateDep,
    source: &EmbsimSource,
    dep: &CrateDep,
    name: &str,
    theirs: &EmbsimSource,
) -> String {
    let said = |source: &EmbsimSource| match source {
        EmbsimSource::Path(root) => format!("at {}", canonical(root).display()),
        other => other.describe(),
    };
    format!(
        "{}: catalog crate {} takes {name} {}, and the runner builds against the embsim {} \
         takes {BOARDS} {}. A runner holds one copy of embsim: two would be two virtual clocks, \
         and a part on one would wait on time nobody advances. Point every catalog crate's \
         embsim dependencies at one embsim (PROJECTS.md §10)",
        plan.project.display(),
        dep.package,
        said(theirs),
        first.package,
        said(source)
    )
}

// ============================================================
// A runner the project owns
// ============================================================

/// What Cargo says of a project's own runner crate.
#[derive(Debug)]
struct OwnRunner {
    /// Its package name and version.
    package: String,
    version: String,
    /// Its one binary.
    bin: String,
    /// The root of its workspace, where the lock file is.
    workspace_root: PathBuf,
    /// The workspace's target directory.
    target: PathBuf,
}

/// [`hand_over`] for the project's own runner crate, `runner` relative to
/// the project file.
fn hand_over_own(
    project: &Path,
    catalog: &CatalogTable,
    runner: &str,
    rebuild: bool,
    args: &[OsString],
    err: &mut dyn Write,
) -> Result<Infallible, String> {
    let crates = crate_deps(project, catalog)?;
    let at = project_dir(project).join(runner);
    let dir = at.canonicalize().map_err(|error| {
        format!(
            "{}: [catalog] runner = {runner:?} ({}) is not there ({error}); it is the project's \
             runner crate, relative to the project file (`embsim new --catalog DIR \
             --own-runner` starts one)",
            project.display(),
            at.display()
        )
    })?;
    let manifest = dir.join("Cargo.toml");
    let cargo = Cargo::find();
    let own = cargo.own_runner(project, runner, &manifest, &crates)?;
    let lock = own.workspace_root.join("Cargo.lock");
    let locked = lock.exists();
    let profile = runner_profile();
    let _ = writeln!(
        err,
        "embsim: building the project's runner {} for {} ({}) in {}",
        own.package,
        project.display(),
        crate_names(&crates),
        at.display()
    );
    let _ = err.flush();
    if rebuild {
        let mut packages = vec![own.package.as_str()];
        packages.extend(crates.iter().map(|dep| dep.package.as_str()));
        cargo.clean(&manifest, None, &profile, &packages)?;
    }
    let quiet = !rebuild
        && own
            .target
            .join(profile_dir(&profile))
            .join(&own.bin)
            .exists();
    let build = Build {
        manifest: &manifest,
        target: None,
        profile: &profile,
        quiet,
        locked,
        package: &own.package,
        bin: &own.bin,
    };
    let executable = match cargo.build(&build)? {
        Built::Done { executable, copies } => {
            if let Some(copies) = two_copies(&copies) {
                return Err(format!(
                    "the project's runner {} links two copies of embsim: {copies}. Each copy has \
                     its own virtual clock, and a part on one waits on time nobody advances. \
                     Point every embsim dependency in its workspace at one embsim \
                     (PROJECTS.md §10)",
                    own.package
                ));
            }
            executable
        }
        Built::Failed { copies } => {
            let diagnosis = cargo.diagnose(&manifest);
            let embsim = declared_source_of(&dir, CLI)
                .map_or_else(|| "unknown".to_string(), |source| source.describe());
            if let Some(unfetched) = &diagnosis.unfetched {
                return Err(format!(
                    "the project's runner {} did not build: Cargo could not fetch embsim {embsim} \
                     ({unfetched}); Cargo's errors are above. {}",
                    own.package,
                    fetchable_embsim()
                ));
            }
            let collision = two_copies(&copies).or(diagnosis.collision);
            return Err(match collision {
                Some(copies) => format!(
                    "the project's runner {} did not build: it met two copies of embsim \
                     ({copies}); Cargo's errors are above. Point every embsim dependency in \
                     its workspace — the runner's, the catalog crates', and those of the crates \
                     they depend on — at one embsim (PROJECTS.md §10)",
                    own.package
                ),
                None if locked && cargo.lock_does_not_fit(&manifest) => format!(
                    "the project's runner {} did not build: {} does not lock what it now \
                     depends on (Cargo's errors are above), and embsim builds it --locked so \
                     every machine builds the same runner. Update the lock with Cargo (`cargo \
                     update --workspace` in {}, or a `cargo build` of the runner) and commit it",
                    own.package,
                    lock.display(),
                    own.workspace_root.display()
                ),
                None => format!(
                    "the project's runner {} did not build (embsim {embsim}); Cargo's errors \
                     are above. Its main is embsim_cli::runner_main over the project's catalog \
                     crates, each a library with `pub fn register(set: &mut CatalogSet) -> \
                     Result<(), ProjectError>` at its root (PROJECTS.md §10)",
                    own.package
                ),
            });
        }
    };
    if !locked {
        let _ = writeln!(
            err,
            "embsim: Cargo wrote {}: commit it. Once it is there the runner builds --locked \
             against it, the same on every machine",
            lock.display()
        );
    }
    let mut provenance = vec![crate_provenance(
        "runner crate",
        &own.package,
        Some(&own.version),
        &dir,
    )];
    for dep in &crates {
        let version = cargo
            .crate_metadata(std::slice::from_ref(dep), project)
            .ok()
            .and_then(|mut all| all.pop().flatten())
            .map(|metadata| metadata.version)
            .or_else(|| dep.version.clone());
        provenance.push(crate_provenance(
            "catalog crate",
            &dep.package,
            version.as_deref(),
            &dep.dir,
        ));
    }
    Err(exec(&executable, args, &provenance))
}

// ============================================================
// Cargo
// ============================================================

/// What Cargo's resolution of a runner that did not build says.
#[derive(Debug, Default, PartialEq, Eq)]
struct Diagnosis {
    /// The graph holds two copies of embsim: two packages of one embsim
    /// name ("package collision in the lockfile"), or a second claim on
    /// `links = "embsim-core"`; Cargo's lines that say so.
    collision: Option<String>,
    /// Cargo could not get an embsim crate from its source (a revision no
    /// remote has, a repository it cannot reach): the crate, and the last
    /// cause Cargo gives.
    unfetched: Option<String>,
}

impl Diagnosis {
    /// Read Cargo's error `text`.
    fn of_cargo_errors(text: &str) -> Self {
        let lines: Vec<&str> = text.lines().map(str::trim).collect();
        Self {
            collision: Self::collision(&lines),
            unfetched: Self::unfetched(&lines),
        }
    }

    fn collision(lines: &[&str]) -> Option<String> {
        if let Some(line) = lines
            .iter()
            .find(|line| line.contains("package collision") && line.contains("embsim-"))
        {
            return Some(line.trim_start_matches("error: ").to_string());
        }
        let at = lines
            .iter()
            .position(|line| line.contains("links to the native library `embsim-core`"))?;
        let mut said = vec![lines[at].trim_end_matches(':').to_string()];
        if let Some(first) = lines[at + 1..]
            .iter()
            .find(|line| line.starts_with("package `embsim-core"))
        {
            said.push(format!("the first is {first}"));
        }
        if let Some(second) = lines
            .iter()
            .find(|line| line.starts_with("... required by"))
        {
            said.push(format!("the second {}", second.trim_start_matches("... ")));
        }
        Some(said.join("; "))
    }

    /// `error: failed to get `embsim-board` as a dependency of …`, then its
    /// causes, the last of which says what went wrong.
    fn unfetched(lines: &[&str]) -> Option<String> {
        let at = lines
            .iter()
            .position(|line| line.starts_with("error: failed to get `embsim-"))?;
        let package = lines[at]
            .trim_start_matches("error: failed to get `")
            .split('`')
            .next()
            .unwrap_or_default();
        let cause = lines[at + 1..]
            .iter()
            .rev()
            .find(|line| !line.is_empty() && !line.starts_with("Caused by"))
            .copied()
            .unwrap_or_default();
        Some(format!("{package}: {cause}"))
    }
}

/// A runner build, as [`Cargo::build`] runs it.
struct Build<'a> {
    manifest: &'a Path,
    /// The target directory; `None` for Cargo's own.
    target: Option<&'a Path>,
    profile: &'a str,
    quiet: bool,
    /// `--locked`.
    locked: bool,
    /// The package whose binary is the runner, and the binary.
    package: &'a str,
    bin: &'a str,
}

/// What a runner build gave.
enum Built {
    Done { executable: PathBuf, copies: Copies },
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
        command
    }

    /// Why `error`, from starting Cargo, means there is no runner for
    /// `project`, whose catalog crates are `crates`.
    fn missing(&self, project: &Path, crates: &str, error: &std::io::Error) -> String {
        let which = if self.from_env {
            format!(
                "$CARGO names {}, which does not start ({error})",
                self.program.to_string_lossy()
            )
        } else {
            format!("no `cargo` on the PATH starts ({error})")
        };
        format!(
            "{} names catalog crates ({crates}), which embsim builds into a runner with Cargo, \
             and {which}. Install Rust's toolchain (https://rustup.rs), or run the project from \
             a binary of its own over embsim_cli::main_with (PROJECTS.md §10)",
            project.display(),
        )
    }

    /// `cargo metadata --no-deps` of the manifest `manifest`, run in `dir`:
    /// `None` when Cargo cannot read it, an error only when Cargo does not
    /// start.
    fn metadata_no_deps(
        &self,
        dir: &Path,
        manifest: &Path,
        project: &Path,
        crates: &str,
    ) -> Result<Option<serde_json::Value>, String> {
        let output = self
            .command(
                dir,
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
            .map_err(|error| self.missing(project, crates, &error))?;
        Ok(output
            .status
            .success()
            .then(|| serde_json::from_slice(&output.stdout).ok())
            .flatten())
    }

    /// What `cargo metadata --no-deps` says of each catalog crate, in
    /// order: `None` for a crate Cargo cannot read on its own (one in a
    /// directory a workspace covers without listing it). An error only when
    /// Cargo does not start.
    fn crate_metadata(
        &self,
        crates: &[CrateDep],
        project: &Path,
    ) -> Result<Vec<Option<CrateMetadata>>, String> {
        let names = crate_names(crates);
        let mut all = Vec::with_capacity(crates.len());
        for dep in crates {
            let manifest = dep.dir.join("Cargo.toml");
            let metadata = self.metadata_no_deps(&dep.dir, &manifest, project, &names)?;
            all.push(metadata.and_then(|metadata| CrateMetadata::read(&metadata, &manifest)));
        }
        Ok(all)
    }

    /// What Cargo says of the project's own runner crate: refused, saying
    /// what a runner crate is, when it is not one.
    fn own_runner(
        &self,
        project: &Path,
        runner: &str,
        manifest: &Path,
        crates: &[CrateDep],
    ) -> Result<OwnRunner, String> {
        let dir = manifest.parent().unwrap_or(manifest);
        let refuse = |why: String| {
            format!(
                "{}: [catalog] runner = {runner:?}: {why}; the project's runner is a binary \
                 crate whose main is embsim_cli::runner_main over its catalog crates \
                 (PROJECTS.md §10)",
                project.display()
            )
        };
        let metadata = self
            .metadata_no_deps(dir, manifest, project, &crate_names(crates))?
            .ok_or_else(|| {
                refuse(format!(
                    "Cargo cannot read {} (`cargo metadata --manifest-path {}` says why)",
                    manifest.display(),
                    manifest.display()
                ))
            })?;
        let package = package_at(&metadata, manifest)
            .ok_or_else(|| refuse(format!("{} is not a package", manifest.display())))?;
        let text = |value: Option<&serde_json::Value>| {
            value
                .and_then(serde_json::Value::as_str)
                .unwrap_or_default()
                .to_string()
        };
        let name = text(package.get("name"));
        let bins: Vec<String> = package
            .get("targets")
            .and_then(serde_json::Value::as_array)
            .map(|targets| {
                targets
                    .iter()
                    .filter(|target| {
                        target
                            .get("kind")
                            .and_then(serde_json::Value::as_array)
                            .is_some_and(|kinds| kinds.iter().any(|kind| kind == "bin"))
                    })
                    .map(|target| text(target.get("name")))
                    .collect()
            })
            .unwrap_or_default();
        let bin = match bins.as_slice() {
            [one] => one.clone(),
            [] => return Err(refuse(format!("the package {name} has no binary"))),
            several => several
                .iter()
                .find(|bin| **bin == name)
                .cloned()
                .ok_or_else(|| {
                    refuse(format!(
                        "the package {name} has {} binaries, {}, and none is named {name}",
                        several.len(),
                        several.join(", ")
                    ))
                })?,
        };
        Ok(OwnRunner {
            version: text(package.get("version")),
            package: name,
            bin,
            workspace_root: PathBuf::from(text(metadata.get("workspace_root"))),
            target: PathBuf::from(text(metadata.get("target_directory"))),
        })
    }

    /// Why the build of `manifest` failed, when Cargo's resolution says:
    /// read off a `cargo metadata` of the runner, which resolves as the
    /// build did, after a build failed — once, its network retries off, so
    /// a source Cargo cannot reach fails at once the second time.
    fn diagnose(&self, manifest: &Path) -> Diagnosis {
        let output = self
            .command(
                manifest.parent().unwrap_or(manifest),
                &[
                    OsStr::new("metadata"),
                    OsStr::new("--format-version"),
                    OsStr::new("1"),
                    OsStr::new("--manifest-path"),
                    manifest.as_os_str(),
                ],
            )
            .env("CARGO_NET_RETRY", "0")
            .stdout(Stdio::null())
            .output();
        match output {
            Ok(output) if !output.status.success() => {
                Diagnosis::of_cargo_errors(&String::from_utf8_lossy(&output.stderr))
            }
            _ => Diagnosis::default(),
        }
    }

    /// Whether the lock file of the build of `manifest` no longer locks
    /// what it depends on: what `cargo metadata --locked` refuses.
    fn lock_does_not_fit(&self, manifest: &Path) -> bool {
        let output = self
            .command(
                manifest.parent().unwrap_or(manifest),
                &[
                    OsStr::new("metadata"),
                    OsStr::new("--locked"),
                    OsStr::new("--format-version"),
                    OsStr::new("1"),
                    OsStr::new("--manifest-path"),
                    manifest.as_os_str(),
                ],
            )
            .stdout(Stdio::null())
            .output();
        output.is_ok_and(|output| {
            !output.status.success() && String::from_utf8_lossy(&output.stderr).contains("--locked")
        })
    }

    /// `cargo clean` of `packages` in the build of `manifest`, for
    /// `--rebuild`.
    fn clean(
        &self,
        manifest: &Path,
        target: Option<&Path>,
        profile: &str,
        packages: &[&str],
    ) -> Result<(), String> {
        let mut args: Vec<&OsStr> = vec![
            OsStr::new("clean"),
            OsStr::new("--manifest-path"),
            manifest.as_os_str(),
            OsStr::new("--profile"),
            OsStr::new(profile),
        ];
        if let Some(target) = target {
            args.push(OsStr::new("--target-dir"));
            args.push(target.as_os_str());
        }
        for package in packages {
            args.push(OsStr::new("-p"));
            args.push(OsStr::new(package));
        }
        let dir = manifest.parent().unwrap_or(manifest);
        let status = self
            .command(dir, &args)
            .status()
            .map_err(|error| format!("--rebuild: cannot start Cargo: {error}"))?;
        if status.success() {
            Ok(())
        } else {
            Err(format!(
                "--rebuild: `cargo clean` of the runner in {} failed ({status}); its errors are \
                 above",
                dir.display()
            ))
        }
    }

    /// `cargo build` of a runner: its errors and progress to standard error
    /// (only its errors when quiet), its report read for the binary and the
    /// embsim libraries.
    fn build(&self, build: &Build<'_>) -> Result<Built, String> {
        let mut args = vec![
            OsStr::new("build"),
            OsStr::new("--manifest-path"),
            build.manifest.as_os_str(),
            OsStr::new("--profile"),
            OsStr::new(build.profile),
            OsStr::new("--bin"),
            OsStr::new(build.bin),
            OsStr::new("--message-format"),
            OsStr::new("json-render-diagnostics"),
        ];
        if let Some(target) = build.target {
            args.push(OsStr::new("--target-dir"));
            args.push(target.as_os_str());
        }
        if build.locked {
            args.push(OsStr::new("--locked"));
        }
        if build.quiet {
            args.push(OsStr::new("--quiet"));
        }
        let dir = build.manifest.parent().unwrap_or(build.manifest);
        let mut child = self
            .command(dir, &args)
            .stdout(Stdio::piped())
            .spawn()
            .map_err(|error| format!("cannot start Cargo: {error}"))?;
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
                if name == build.bin {
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
            (true, Some(executable)) => Ok(Built::Done { executable, copies }),
            (true, None) => Err(format!(
                "cargo built {} and named no binary {} of {}",
                build.manifest.display(),
                build.bin,
                build.package
            )),
            (false, _) => Ok(Built::Failed { copies }),
        }
    }
}

/// Refuse to run `project` in a runner built with `crates` unless its
/// `[catalog]` names exactly those crates, in that order.
pub fn check_runner_fits(crates: &[CatalogCrate], project: &Path) -> Result<(), String> {
    let head = ProjectHead::of_project(project).map_err(|error| error.to_string())?;
    let held: Vec<PathBuf> = crates
        .iter()
        .map(|catalog| canonical(Path::new(catalog.dir)))
        .collect();
    let dir = project_dir(project);
    let named: Vec<PathBuf> = head
        .catalog
        .as_ref()
        .map(|catalog| {
            catalog
                .crates
                .iter()
                .map(|path| canonical(&dir.join(path)))
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
             project with `embsim`, which builds or finds the runner its [catalog] names \
             (PROJECTS.md §10)",
            project.display(),
            list(&held),
            list(&named)
        ));
    }
    Ok(())
}

/// `path` with its `.` dropped and each `..` taking the directory before
/// it, as Cargo reads a path dependency: symlinks left as written.
fn normalize(path: &Path) -> PathBuf {
    use std::path::Component;
    let mut out = PathBuf::new();
    for part in path.components() {
        match part {
            Component::CurDir => {}
            Component::ParentDir => {
                if !out.pop() {
                    out.push("..");
                }
            }
            other => out.push(other.as_os_str()),
        }
    }
    out
}

/// `path` made canonical where it exists, as written where it does not.
fn canonical(path: &Path) -> PathBuf {
    path.canonicalize().unwrap_or_else(|_| path.to_path_buf())
}

#[cfg(test)]
mod tests {
    use rstest::rstest;

    use super::*;

    fn dep(package: &str) -> CrateDep {
        CrateDep {
            package: package.to_string(),
            library: package.replace('-', "_"),
            dir: PathBuf::from("/nowhere").join(package),
            version: None,
        }
    }

    #[rstest]
    fn a_runner_id_is_eight_hex_digits_that_follow_the_crates_names_only() {
        let one = runner_id(&[dep("a")]);
        assert_eq!(one.len(), 8);
        assert!(one.chars().all(|c| c.is_ascii_hexdigit()), "{one}");
        assert_eq!(one, runner_id(&[dep("a")]));
        assert_ne!(one, runner_id(&[dep("b")]));
        let mut moved = dep("a");
        moved.dir = PathBuf::from("/elsewhere/a");
        assert_eq!(
            one,
            runner_id(&[moved]),
            "where the crate is does not count"
        );
        assert_ne!(
            runner_id(&[dep("a"), dep("b")]),
            runner_id(&[dep("b"), dep("a")])
        );
    }

    #[rstest]
    #[case::rev("https://github.com/x/embsim?rev=abc", Some(("rev", "abc")))]
    #[case::branch("https://github.com/x/embsim?branch=main", Some(("branch", "main")))]
    #[case::tag("https://github.com/x/embsim?tag=v0.2.0", Some(("tag", "v0.2.0")))]
    #[case::default_branch("https://github.com/x/embsim", None)]
    fn a_git_source_keeps_the_revision_it_names(
        #[case] text: &str,
        #[case] reference: Option<(&str, &str)>,
    ) {
        let EmbsimSource::Git {
            url,
            reference: found,
        } = EmbsimSource::git(text)
        else {
            panic!("a git source");
        };
        assert_eq!(url, "https://github.com/x/embsim");
        assert_eq!(
            found,
            reference.map(|(key, value)| (key.to_string(), value.to_string()))
        );
    }

    #[rstest]
    fn one_repository_at_one_revision_is_one_copy() {
        let at = |url: &str, rev: &str| EmbsimSource::Git {
            url: url.to_string(),
            reference: Some(("rev".to_string(), rev.to_string())),
        };
        assert!(at("https://github.com/x/embsim", "a")
            .same_copy(&at("https://github.com/x/embsim.git/", "a")));
        assert!(!at("https://github.com/x/embsim", "a")
            .same_copy(&at("https://github.com/x/embsim", "b")));
        assert!(!at("https://github.com/x/embsim", "a")
            .same_copy(&EmbsimSource::Path(PathBuf::from("/e"))));
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
    fn a_lock_names_the_runner_it_locks() {
        let text = "version = 4\n\n[[package]]\nname = \"embsim-runner-0123abcd\"\nversion = \
                    \"0.0.0\"\n\n[[package]]\nname = \"embsim-cli\"\nversion = \"0.2.0\"\nsource = \
                    \"git+https://github.com/x/embsim?rev=a#a\"\n";
        assert_eq!(
            locked_runner(text).as_deref(),
            Some("embsim-runner-0123abcd")
        );
        assert_eq!(locked_runner("version = 4\n"), None);
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
