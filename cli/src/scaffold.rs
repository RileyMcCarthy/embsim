//! `embsim new --catalog DIR [--own-runner [RUNNER]]`: a catalog crate, and
//! optionally the project's own runner crate, started for a project.
//!
//! The crate is the one `cli/catalog-template` keeps compiling in this
//! workspace: its `src/lib.rs`, with the project's name in place of
//! `yourproject` (kinds `<name>-board`, `<name>-sensor`, `<name>-core`,
//! `<name>-source`, package `<name>-catalog`), and a manifest whose embsim
//! dependencies are where the runner will take embsim from: the crate's
//! dependency is the version the project builds against (`PROJECTS.md`
//! §10, "Which embsim the runner builds against"). That is, in order:
//!
//! 1. the embsim the project's first catalog crate already depends on, for
//!    a crate that joins a project (`--add-to`) naming one;
//! 2. the checkout this embsim was built from, by a relative path, when it
//!    sits inside the project's repository (a submodule, as in MaD);
//! 3. embsim's repository at the revision this embsim was built from,
//!    with its release as the version (`{ git, rev, version = "0.2" }`), or
//!    at its release tag when the revision is not known.
//!
//! The project names the crate in its `[catalog]`: the starter project
//! `embsim new` writes, or, with `--add-to`, an existing file, edited in
//! place with its comments kept. `--own-runner` also starts a binary crate
//! whose main is the command over the catalog crate, and names it in
//! `[catalog] runner`.

use std::io::Write;
use std::path::{Component as PathPart, Path, PathBuf};
use std::process::Command;

use embsim_board::CatalogTable;

use crate::checklist::{relative_path, toml_string};
use crate::provenance;
use crate::runner::{declared_source, is_checkout, EmbsimSource};

/// The template's library, compiled in this workspace as
/// `yourproject-catalog`.
const TEMPLATE_LIB: &str = include_str!("../catalog-template/src/lib.rs");

/// The word the template uses for the project's name.
const TEMPLATE_NAME: &str = "yourproject";

/// embsim's repository, where a crate outside it takes embsim from.
const REPOSITORY: &str = env!("CARGO_PKG_REPOSITORY");

/// What `write_crate` made.
#[derive(Debug)]
pub struct Scaffold {
    /// The crate's directory, as the command line gave it.
    dir: PathBuf,
    /// The package's name.
    package: String,
    /// The library's crate name.
    library: String,
    /// The project's name, in front of every kind.
    prefix: String,
    /// Where its embsim comes from.
    source: EmbsimSource,
}

impl Scaffold {
    /// Say what was written.
    pub fn say(&self, out: &mut dyn Write) {
        let _ = writeln!(
            out,
            "wrote {} and {}: catalog crate {}",
            self.dir.join("Cargo.toml").display(),
            self.dir.join("src/lib.rs").display(),
            self.package
        );
        let _ = writeln!(
            out,
            "  kinds {p}-board (a board), {p}-sensor (a part), {p}-core (a P2 core), {p}-source \
             (a bench component): one commented example of each to keep, rename or replace",
            p = self.prefix
        );
        let _ = writeln!(
            out,
            "  its embsim, and so the runner's: {}",
            self.source.describe()
        );
    }
}

/// What `write_runner_crate` made.
#[derive(Debug)]
pub struct RunnerScaffold {
    /// The crate's directory, as the command line gave it.
    dir: PathBuf,
    /// The package's name, its binary's too.
    package: String,
}

impl RunnerScaffold {
    /// Say what was written.
    pub fn say(&self, out: &mut dyn Write) {
        let _ = writeln!(
            out,
            "wrote {} and {}: the project's runner {}, the embsim command over the catalog crate",
            self.dir.join("Cargo.toml").display(),
            self.dir.join("src/main.rs").display(),
            self.package
        );
    }
}

/// Lowercase letters, digits and single hyphens from `text`, trimmed of
/// hyphens: the shape a kind is spelled in.
fn kind_word(text: &str) -> String {
    let mut word = String::new();
    for c in text.chars() {
        let c = c.to_ascii_lowercase();
        if c.is_ascii_lowercase() || c.is_ascii_digit() {
            word.push(c);
        } else if matches!(c, '-' | '_' | ' ' | '.') && !word.ends_with('-') && !word.is_empty() {
            word.push('-');
        }
    }
    word.trim_end_matches('-').to_string()
}

/// The project's name a crate in `dir` takes: the directory's name, less a
/// `catalog` word, else its parent's (`sim/catalog` is `sim`, `mad-catalog`
/// is `mad`). A name that does not start with a letter gets `project-` in
/// front.
fn prefix_of(dir: &Path) -> String {
    let names: Vec<String> = dir
        .components()
        .filter_map(|part| match part {
            PathPart::Normal(name) => Some(kind_word(&name.to_string_lossy())),
            _ => None,
        })
        .collect();
    let mut found = String::new();
    for name in names.iter().rev() {
        let stem = name
            .strip_suffix("-catalog")
            .unwrap_or(if name == "catalog" { "" } else { name });
        if !stem.is_empty() {
            found = stem.to_string();
            break;
        }
    }
    if found.is_empty() {
        found = "project".to_string();
    }
    if found.starts_with(|c: char| c.is_ascii_lowercase()) {
        found
    } else {
        format!("project-{found}")
    }
}

/// Where the project's own runner goes when `--own-runner` names no
/// directory: beside the catalog crate, `catalog` in its name made
/// `runner` (`sim/catalog` → `sim/runner`, `mad-catalog` → `mad-runner`),
/// else `-runner` after it.
pub fn default_runner_dir(catalog: &Path) -> PathBuf {
    let name = catalog
        .file_name()
        .map(|name| name.to_string_lossy().into_owned())
        .unwrap_or_default();
    let runner = match name.strip_suffix("catalog") {
        Some(stem) => format!("{stem}runner"),
        None => format!("{name}-runner"),
    };
    catalog.with_file_name(runner)
}

/// Refuse a directory that already holds something: a crate is started in
/// an empty or new directory, never over files.
pub fn check_free(dir: &Path) -> Result<(), String> {
    match std::fs::read_dir(dir) {
        Ok(mut entries) => {
            if entries.next().is_some() {
                Err(format!(
                    "{}: the directory is not empty; a crate starts in an empty or new \
                     directory",
                    dir.display()
                ))
            } else {
                Ok(())
            }
        }
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => Ok(()),
        Err(error) => Err(format!("{}: {error}", dir.display())),
    }
}

/// `major.minor` of this embsim's version: the release a dependency on it
/// asks for.
fn release() -> String {
    provenance::VERSION
        .splitn(3, '.')
        .take(2)
        .collect::<Vec<_>>()
        .join(".")
}

/// The root of the git repository `dir` is in — the nearest directory of
/// it that exists asked — else the current directory: what a project is,
/// for whether embsim sits inside it.
fn project_root(dir: &Path) -> Option<PathBuf> {
    let dir = std::env::current_dir().ok()?.join(dir);
    let existing = dir.ancestors().find(|ancestor| ancestor.is_dir())?;
    let output = Command::new("git")
        .arg("-C")
        .arg(existing)
        .args(["rev-parse", "--show-toplevel"])
        .output()
        .ok();
    let root = output
        .filter(|output| output.status.success())
        .map(|output| PathBuf::from(String::from_utf8_lossy(&output.stdout).trim()))
        .filter(|root| !root.as_os_str().is_empty())
        .unwrap_or(std::env::current_dir().ok()?);
    root.canonicalize().ok()
}

/// Where a crate started in `dir` takes embsim from (module docs): `joins`,
/// the embsim of the project it joins, else this embsim's checkout when it
/// sits inside the project, else this embsim's repository and revision.
pub fn embsim_for(dir: &Path, joins: Option<EmbsimSource>) -> EmbsimSource {
    if let Some(source) = joins {
        return source;
    }
    let checkout = Path::new(provenance::SOURCE_DIR);
    if let (Ok(()), Ok(checkout), Some(root)) = (
        is_checkout(checkout),
        checkout.canonicalize(),
        project_root(dir),
    ) {
        if checkout.starts_with(&root) {
            return EmbsimSource::Path(checkout);
        }
    }
    let sha = provenance::GIT_REV
        .split('+')
        .next()
        .unwrap_or_default()
        .to_string();
    let reference = if sha.is_empty() {
        ("tag".to_string(), format!("v{}", provenance::VERSION))
    } else {
        ("rev".to_string(), sha)
    };
    EmbsimSource::Git {
        url: REPOSITORY.to_string(),
        reference: Some(reference),
    }
}

/// The embsim the first catalog crate of the project at `project` (whose
/// `[catalog]` is `catalog`) depends on, read from its manifest.
fn joined_embsim(project: &Path, catalog: Option<&CatalogTable>) -> Option<EmbsimSource> {
    let first = catalog?.crates.first()?;
    let dir = project_dir(project).join(first);
    match declared_source(&dir)? {
        EmbsimSource::Path(root) => Some(EmbsimSource::Path(root.canonicalize().ok()?)),
        other => Some(other),
    }
}

/// The directory of the project file `project`.
fn project_dir(project: &Path) -> PathBuf {
    project
        .parent()
        .filter(|dir| !dir.as_os_str().is_empty())
        .map_or_else(|| PathBuf::from("."), Path::to_path_buf)
}

/// The dependency lines a manifest in `dir` gives the embsim crates
/// `crates` (directory, package) to take them from `source`.
fn embsim_dependencies(
    source: &EmbsimSource,
    dir: &Path,
    crates: &[(&str, &str)],
) -> Result<String, String> {
    let release = release();
    let mut lines = String::new();
    for (crate_dir, package) in crates {
        lines.push_str(&format!(
            "{package} = {}\n",
            source.dependency(crate_dir, dir, true, Some(&release))?
        ));
    }
    Ok(lines)
}

/// Write the crate into `dir` (which [`check_free`] accepted), its embsim
/// dependencies on `source`.
pub fn write_crate(dir: &Path, source: &EmbsimSource) -> Result<Scaffold, String> {
    check_free(dir).map_err(|why| format!("--catalog {why}"))?;
    let prefix = prefix_of(dir);
    let package = format!("{prefix}-catalog");
    let src = dir.join("src");
    std::fs::create_dir_all(&src)
        .map_err(|error| format!("cannot make {}: {error}", src.display()))?;
    let board = embsim_dependencies(
        source,
        dir,
        &[("board", "embsim-board"), ("boards", "embsim-boards")],
    )?;
    let core = embsim_dependencies(source, dir, &[("core", "embsim-core")])?;
    let manifest = format!(
        "# {package}: the kinds this project adds to embsim (PROJECTS.md §10). Started by\n\
         # `embsim new --catalog`; a project names it in its [catalog] crates, and the\n\
         # `embsim` tool builds it into the runner that runs the project.\n\
         [package]\n\
         name = \"{package}\"\n\
         version = \"0.1.0\"\n\
         edition = \"2021\"\n\
         publish = false\n\
         \n\
         [dependencies]\n\
         # The embsim this project builds against: the runner takes embsim from where\n\
         # this crate's embsim-boards comes from, and every catalog crate's embsim\n\
         # dependencies come from there too — one copy of embsim in the runner.\n\
         {board}\
         # The virtual clock: the instant a component starts at, which its wakes\n\
         # count from.\n\
         {core}",
    );
    let lib = TEMPLATE_LIB.replace(TEMPLATE_NAME, &prefix).replace(
        &TEMPLATE_NAME.to_ascii_uppercase(),
        &prefix.to_ascii_uppercase(),
    );
    for (path, text) in [
        (dir.join("Cargo.toml"), manifest),
        (src.join("lib.rs"), lib),
    ] {
        std::fs::write(&path, text)
            .map_err(|error| format!("cannot write {}: {error}", path.display()))?;
    }
    Ok(Scaffold {
        dir: dir.to_path_buf(),
        library: package.replace('-', "_"),
        package,
        prefix,
        source: source.clone(),
    })
}

/// Write the project's own runner into `dir`: a binary crate over the
/// catalog crate `catalog` made, against the same embsim. In no Cargo
/// workspace, it is a workspace root of its own.
pub fn write_runner_crate(dir: &Path, catalog: &Scaffold) -> Result<RunnerScaffold, String> {
    check_free(dir).map_err(|why| format!("--own-runner {why}"))?;
    let package = format!("{}-runner", catalog.prefix);
    let src = dir.join("src");
    std::fs::create_dir_all(&src)
        .map_err(|error| format!("cannot make {}: {error}", src.display()))?;
    let cli = embsim_dependencies(&catalog.source, dir, &[("cli", "embsim-cli")])?;
    let to_catalog = relative_path(&catalog.dir, dir)?;
    let to_catalog = to_catalog.to_string_lossy();
    let workspace = if enclosing_workspace(dir).is_some() {
        String::new()
    } else {
        "\n# A Cargo workspace of its own: its Cargo.lock goes beside this file.\n[workspace]\n"
            .to_string()
    };
    let manifest = format!(
        "# {package}: this project's runner, the embsim command over its catalog crates\n\
         # (PROJECTS.md §10, \"A runner the project owns\"). The project names it in\n\
         # [catalog] runner; the `embsim` tool builds it with the workspace's Cargo.lock\n\
         # (--locked once there is one) and runs the project through it.\n\
         [package]\n\
         name = \"{package}\"\n\
         version = \"0.1.0\"\n\
         edition = \"2021\"\n\
         publish = false\n\
         \n\
         [[bin]]\n\
         name = \"{package}\"\n\
         path = \"src/main.rs\"\n\
         \n\
         [dependencies]\n\
         # The same embsim as the catalog crates': one copy of embsim in the runner.\n\
         {cli}\
         {catalog_package} = {{ path = {catalog_path} }}\n\
         {workspace}",
        catalog_package = catalog.package,
        catalog_path = toml_string(&to_catalog),
    );
    let main = format!(
        "//! {package}: the embsim command over this project's catalog crates, which\n\
         //! `embsim check` and `embsim run` build and run for a project whose [catalog]\n\
         //! runner names this crate (PROJECTS.md §10). A catalog crate the project adds\n\
         //! joins the list below and the dependencies, as it joins [catalog] crates.\n\
         \n\
         use std::process::ExitCode;\n\
         \n\
         fn main() -> ExitCode {{\n    \
             embsim_cli::runner_main(&[embsim_cli::CatalogCrate::new(\n        \
                 {name:?},\n        \
                 concat!(env!(\"CARGO_MANIFEST_DIR\"), {dir:?}),\n        \
                 {library}::register,\n    \
             )])\n\
         }}\n",
        name = catalog.package,
        dir = format!("/{to_catalog}"),
        library = catalog.library,
    );
    for (path, text) in [
        (dir.join("Cargo.toml"), manifest),
        (src.join("main.rs"), main),
    ] {
        std::fs::write(&path, text)
            .map_err(|error| format!("cannot write {}: {error}", path.display()))?;
    }
    Ok(RunnerScaffold {
        dir: dir.to_path_buf(),
        package,
    })
}

/// The `[catalog]` table naming `crates`, and the project's own runner
/// `runner` when given, as a project file writes it.
fn catalog_table(crates: &[String], runner: Option<&str>) -> String {
    let crates: Vec<String> = crates.iter().map(|path| toml_string(path)).collect();
    let mut table = format!(
        "# The project's own kinds: catalog crates the `embsim` tool builds into the\n\
         # runner that runs this project (PROJECTS.md §10).\n\
         [catalog]\n\
         crates = [{}]\n",
        crates.join(", ")
    );
    if let Some(runner) = runner {
        table.push_str(&format!(
            "# The project's own runner crate, which holds them.\nrunner = {}\n",
            toml_string(runner)
        ));
    }
    table
}

/// `text`, a project with no `[catalog]`, with one naming the crate `path`
/// before its first table.
#[cfg(test)]
fn with_catalog_table(text: &str, path: &str) -> String {
    with_catalog(text, &[path.to_string()], None)
}

/// `text`, a project with no `[catalog]`, with one naming `crates` (and
/// the runner crate `runner`) before its first table: after its leading
/// comment and any key of the file's own (`requires-embsim`), before the
/// comment that heads that table.
pub fn with_catalog(text: &str, crates: &[String], runner: Option<&str>) -> String {
    let lines: Vec<&str> = text.split_inclusive('\n').collect();
    let first_table = lines
        .iter()
        .position(|line| line.trim_start().starts_with('['))
        .unwrap_or(lines.len());
    // The comment lines right above the table are the table's.
    let mut at = first_table;
    while at > 0 && lines[at - 1].trim_start().starts_with('#') {
        at -= 1;
    }
    // A file that is one comment and then its tables: the comment is the
    // file's, and the table goes after it.
    if at == 0 {
        at = first_table;
    }
    let head: String = lines[..at].concat();
    let rest: String = lines[at..].concat();
    let mut out = head;
    if !out.is_empty() && !out.ends_with("\n\n") {
        if !out.ends_with('\n') {
            out.push('\n');
        }
        out.push('\n');
    }
    out.push_str(&catalog_table(crates, runner));
    if !rest.is_empty() {
        out.push('\n');
        out.push_str(rest.trim_start_matches('\n'));
    }
    out
}

/// Add the crate `path` to the `[catalog] crates` of the project `text`
/// (and, given, make `runner` its runner), keeping its comments and layout.
fn add_crate(text: &str, path: &str, runner: Option<&str>) -> Result<String, String> {
    let catalog = CatalogTable::of_project_text(text).map_err(|error| error.to_string())?;
    let Some(catalog) = catalog else {
        return Ok(with_catalog(text, &[path.to_string()], runner));
    };
    if catalog.crates.iter().any(|named| named == path) {
        return Err(format!("its [catalog] crates already names {path:?}"));
    }
    if let (Some(named), Some(_)) = (&catalog.runner, runner) {
        return Err(format!(
            "its [catalog] already names its runner, {named:?}; add the crate to that runner's \
             main and dependencies, or start the crate without --own-runner"
        ));
    }
    let mut document: toml_edit::DocumentMut = text
        .parse()
        .map_err(|error| format!("does not parse: {error}"))?;
    let table = document
        .get_mut("catalog")
        .and_then(toml_edit::Item::as_table_like_mut)
        .ok_or_else(|| "its [catalog] is not a table".to_string())?;
    table
        .get_mut("crates")
        .and_then(toml_edit::Item::as_array_mut)
        .ok_or_else(|| "its [catalog] crates is not an array".to_string())?
        .push(path);
    if let Some(runner) = runner {
        table.insert("runner", toml_edit::value(runner));
    }
    Ok(document.to_string())
}

/// `embsim new --catalog DIR [--own-runner [RUNNER]] [--add-to PROJECT]`.
pub fn new_catalog(
    dir: &Path,
    own_runner: Option<&Path>,
    add_to: Option<&Path>,
    out: &mut dyn Write,
) -> Result<(), String> {
    check_free(dir).map_err(|why| format!("--catalog {why}"))?;
    let runner_dir = own_runner.map(|path| {
        if path.as_os_str().is_empty() {
            default_runner_dir(dir)
        } else {
            path.to_path_buf()
        }
    });
    if let Some(runner_dir) = &runner_dir {
        check_free(runner_dir).map_err(|why| format!("--own-runner {why}"))?;
    }
    let project = match add_to {
        Some(project) => {
            let text = std::fs::read_to_string(project)
                .map_err(|error| format!("--add-to {}: {error}", project.display()))?;
            let catalog = CatalogTable::of_project_text(&text)
                .map_err(|error| format!("--add-to {}: {error}", project.display()))?;
            Some((project, text, catalog))
        }
        None => None,
    };
    // A crate that joins a project takes the embsim its crates take.
    let joins = project
        .as_ref()
        .and_then(|(project, _, catalog)| joined_embsim(project, catalog.as_ref()));
    let source = embsim_for(dir, joins);
    let scaffold = write_crate(dir, &source)?;
    scaffold.say(out);
    let runner = match &runner_dir {
        Some(runner_dir) => {
            let runner = write_runner_crate(runner_dir, &scaffold)?;
            runner.say(out);
            Some(runner)
        }
        None => None,
    };
    match project {
        Some((project, text, catalog)) => {
            let from = project_dir(project);
            let path = relative_path(dir, &from)?;
            let path = path.to_string_lossy();
            let runner_path = match &runner {
                Some(runner) => Some(
                    relative_path(&runner.dir, &from)?
                        .to_string_lossy()
                        .into_owned(),
                ),
                None => None,
            };
            let edited = add_crate(&text, &path, runner_path.as_deref())
                .map_err(|error| format!("--add-to {}: {error}", project.display()))?;
            std::fs::write(project, edited)
                .map_err(|error| format!("cannot write {}: {error}", project.display()))?;
            let _ = writeln!(
                out,
                "added {path:?} to the [catalog] crates of {}; `embsim check {}` builds the \
                 runner that holds it",
                project.display(),
                project.display()
            );
            if let Some(runner_path) = &runner_path {
                let _ = writeln!(out, "  and named its runner: runner = {runner_path:?}");
            } else if let Some(named) = catalog.and_then(|catalog| catalog.runner) {
                let _ = writeln!(
                    out,
                    "note: {} names its own runner, {named:?}: add {} to that crate's \
                     dependencies and to the list its main hands embsim_cli::runner_main",
                    project.display(),
                    scaffold.package
                );
            }
        }
        None => {
            let _ = writeln!(
                out,
                "  a project names it in its [catalog]: crates = [\"<this directory, relative to \
                 the project file>\"]{}; or `embsim new --catalog DIR --add-to PROJECT` does",
                if runner.is_some() {
                    ", runner = \"<the runner's directory>\""
                } else {
                    ""
                }
            );
        }
    }
    let crates: Vec<&Path> = std::iter::once(dir)
        .chain(runner.as_ref().map(|runner| runner.dir.as_path()))
        .collect();
    if let Some(workspace) = enclosing_workspace(dir) {
        let _ = writeln!(
            out,
            "note: {} sit{} inside the Cargo workspace {}; add {} to that workspace's members \
             to build and test {} there{}",
            crates
                .iter()
                .map(|dir| dir.display().to_string())
                .collect::<Vec<_>>()
                .join(" and "),
            if crates.len() == 1 { "s" } else { "" },
            workspace.display(),
            if crates.len() == 1 { "it" } else { "them" },
            if crates.len() == 1 { "it" } else { "them" },
            if runner.is_some() {
                ""
            } else {
                " (the runner builds it either way)"
            }
        );
    }
    Ok(())
}

/// The manifest of the nearest Cargo workspace above `dir`, if any.
pub fn enclosing_workspace(dir: &Path) -> Option<PathBuf> {
    let dir = std::env::current_dir().ok()?.join(dir);
    let existing = dir.ancestors().find(|ancestor| ancestor.is_dir())?;
    let existing = existing.canonicalize().ok()?;
    existing.ancestors().skip(1).find_map(|ancestor| {
        let manifest = ancestor.join("Cargo.toml");
        let text = std::fs::read_to_string(&manifest).ok()?;
        let table: toml::Table = toml::from_str(&text).ok()?;
        table.contains_key("workspace").then_some(manifest)
    })
}

#[cfg(test)]
mod tests {
    use rstest::rstest;

    use super::*;

    #[rstest]
    #[case::catalog_under_a_project("sim/catalog", "sim")]
    #[case::named_catalog("tools/mad-catalog", "mad")]
    #[case::plain("rig", "rig")]
    #[case::underscores("My_Rig.Sim", "my-rig-sim")]
    #[case::bare_catalog("catalog", "project")]
    #[case::digit_first("2024-rig", "project-2024-rig")]
    fn a_crate_takes_its_projects_name_from_its_directory(#[case] dir: &str, #[case] prefix: &str) {
        assert_eq!(prefix_of(Path::new(dir)), prefix);
    }

    #[rstest]
    #[case::under_a_project("sim/catalog", "sim/runner")]
    #[case::named("SIL/mad-catalog", "SIL/mad-runner")]
    #[case::other("rig", "rig-runner")]
    fn a_runner_goes_beside_its_catalog_crate(#[case] catalog: &str, #[case] runner: &str) {
        assert_eq!(default_runner_dir(Path::new(catalog)), Path::new(runner));
    }

    #[rstest]
    fn a_catalog_table_goes_after_the_leading_comment() {
        let text = "# A project.\n# Two lines.\n\n[[board]]\nname = \"B\"\n";
        let with = with_catalog_table(text, "sim/catalog");
        assert!(
            with.starts_with("# A project.\n# Two lines.\n\n# The project's own kinds"),
            "{with}"
        );
        assert!(
            with.contains("[catalog]\ncrates = [\"sim/catalog\"]\n\n[[board]]"),
            "{with}"
        );
        assert_eq!(
            CatalogTable::of_project_text(&with)
                .unwrap()
                .unwrap()
                .crates,
            ["sim/catalog"]
        );
    }

    #[rstest]
    fn a_catalog_table_goes_after_the_files_own_keys() {
        let text = "# A project.\n\n# Its release.\nrequires-embsim = \"0.2\"\n\n# The board.\n\
                    [[board]]\nname = \"B\"\n";
        let with = with_catalog(text, &["sim/catalog".to_string()], Some("sim/runner"));
        assert!(
            with.contains("requires-embsim = \"0.2\"\n\n# The project's own kinds"),
            "{with}"
        );
        assert!(
            with.contains("runner = \"sim/runner\"\n\n# The board.\n[[board]]"),
            "{with}"
        );
        let head = embsim_board::ProjectHead::of_project_text(&with).unwrap();
        assert_eq!(head.requires_embsim.as_deref(), Some("0.2"));
        let catalog = head.catalog.unwrap();
        assert_eq!(catalog.crates, ["sim/catalog"]);
        assert_eq!(catalog.runner.as_deref(), Some("sim/runner"));
    }

    #[rstest]
    fn a_crate_joins_an_existing_catalog_table_keeping_its_comments() {
        let text = "[catalog]\n# ours\ncrates = [\"a\"] # first\n\n[[board]]\nname = \"B\"\n";
        let added = add_crate(text, "b", None).unwrap();
        assert!(added.contains("# ours"), "{added}");
        assert!(added.contains("# first"), "{added}");
        assert_eq!(
            CatalogTable::of_project_text(&added)
                .unwrap()
                .unwrap()
                .crates,
            ["a", "b"]
        );
        assert!(add_crate(&added, "a", None)
            .unwrap_err()
            .contains("already names"));
        let with_runner = add_crate(text, "b", Some("runner")).unwrap();
        assert_eq!(
            CatalogTable::of_project_text(&with_runner)
                .unwrap()
                .unwrap()
                .runner
                .as_deref(),
            Some("runner")
        );
    }
}
