//! `embsim new --catalog DIR`: a catalog crate started for a project.
//!
//! The crate is the one `cli/catalog-template` keeps compiling in this
//! workspace: its `src/lib.rs`, with the project's name in place of
//! `yourproject` (kinds `<name>-board`, `<name>-sensor`, `<name>-core`,
//! `<name>-source`, package `<name>-catalog`), and a manifest whose paths
//! reach the embsim checkout the runner would build against. The project
//! names it in its `[catalog]`: the starter project `embsim new` writes,
//! or, with `--add-to`, an existing file, edited in place with its
//! comments kept.

use std::io::Write;
use std::path::{Component as PathPart, Path, PathBuf};

use embsim_board::CatalogTable;

use crate::checklist::{relative_path, toml_string};
use crate::runner::is_checkout;

/// The template's library, compiled in this workspace as
/// `yourproject-catalog`.
const TEMPLATE_LIB: &str = include_str!("../catalog-template/src/lib.rs");

/// The word the template uses for the project's name.
const TEMPLATE_NAME: &str = "yourproject";

/// What `write_crate` made.
#[derive(Debug)]
pub struct Scaffold {
    /// The crate's directory, as the command line gave it.
    dir: PathBuf,
    /// The package's name.
    package: String,
    /// The project's name, in front of every kind.
    prefix: String,
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

/// Refuse a directory that already holds something: a crate is started in
/// an empty or new directory, never over files.
pub fn check_free(dir: &Path) -> Result<(), String> {
    match std::fs::read_dir(dir) {
        Ok(mut entries) => {
            if entries.next().is_some() {
                Err(format!(
                    "--catalog {}: the directory is not empty; a catalog crate starts in an \
                     empty or new directory",
                    dir.display()
                ))
            } else {
                Ok(())
            }
        }
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => Ok(()),
        Err(error) => Err(format!("--catalog {}: {error}", dir.display())),
    }
}

/// `path` as the crate's manifest reaches it from `dir`: relative when the
/// two share a directory below the root, whole otherwise.
fn reach(path: &Path, dir: &Path) -> Result<String, String> {
    Ok(relative_path(path, dir)?.to_string_lossy().into_owned())
}

/// Write the crate into `dir` (which [`check_free`] accepted), its embsim
/// dependencies on the checkout `embsim`.
pub fn write_crate(dir: &Path, embsim: &Path) -> Result<Scaffold, String> {
    check_free(dir)?;
    is_checkout(embsim).map_err(|why| {
        format!(
            "--catalog: a catalog crate depends on an embsim checkout, and this embsim was \
             built from {}, which is not one now ({why})",
            embsim.display()
        )
    })?;
    let prefix = prefix_of(dir);
    let package = format!("{prefix}-catalog");
    let src = dir.join("src");
    std::fs::create_dir_all(&src)
        .map_err(|error| format!("cannot make {}: {error}", src.display()))?;
    let board = reach(&embsim.join("board"), dir)?;
    let boards = reach(&embsim.join("boards"), dir)?;
    let core = reach(&embsim.join("core"), dir)?;
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
         # The embsim checkout the runner builds against: one copy of embsim in the\n\
         # runner, so these paths and the project's [catalog] embsim (or the checkout\n\
         # the `embsim` tool was built from) must be the same directory.\n\
         embsim-board = {{ path = {board} }}\n\
         embsim-boards = {{ path = {boards} }}\n\
         # The virtual clock: the instant a component starts at, which its wakes\n\
         # count from.\n\
         embsim-core = {{ path = {core} }}\n",
        board = toml_string(&board),
        boards = toml_string(&boards),
        core = toml_string(&core),
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
        package,
        prefix,
    })
}

/// The `[catalog]` table naming `crates`, and the checkout `embsim` when
/// given, as a project file writes it.
fn catalog_table(crates: &[String], embsim: Option<&str>) -> String {
    let crates: Vec<String> = crates.iter().map(|path| toml_string(path)).collect();
    let mut table = format!(
        "# The project's own kinds: catalog crates the `embsim` tool builds into the\n\
         # runner that runs this project (PROJECTS.md §10).\n\
         [catalog]\n\
         crates = [{}]\n",
        crates.join(", ")
    );
    if let Some(embsim) = embsim {
        table.push_str(&format!("embsim = {}\n", toml_string(embsim)));
    }
    table
}

/// `text`, a project with no `[catalog]`, with one naming the crate `path`
/// after its leading comment.
pub fn with_catalog_table(text: &str, path: &str) -> String {
    with_catalog(text, &[path.to_string()], None)
}

/// `text`, a project with no `[catalog]`, with one naming `crates` (and
/// the checkout `embsim`) after its leading comment.
pub fn with_catalog(text: &str, crates: &[String], embsim: Option<&str>) -> String {
    let mut head = 0;
    for line in text.split_inclusive('\n') {
        let trimmed = line.trim();
        if trimmed.is_empty() || trimmed.starts_with('#') {
            head += line.len();
        } else {
            break;
        }
    }
    let (comment, rest) = text.split_at(head);
    let mut out = comment.to_string();
    if !out.is_empty() && !out.ends_with("\n\n") {
        if !out.ends_with('\n') {
            out.push('\n');
        }
        out.push('\n');
    }
    out.push_str(&catalog_table(crates, embsim));
    if !rest.is_empty() {
        out.push('\n');
        out.push_str(rest);
    }
    out
}

/// Add the crate `path` to the `[catalog] crates` of the project `text`,
/// keeping its comments and layout.
fn add_crate(text: &str, path: &str) -> Result<String, String> {
    let catalog = CatalogTable::of_project_text(text).map_err(|error| error.to_string())?;
    let Some(catalog) = catalog else {
        return Ok(with_catalog_table(text, path));
    };
    if catalog.crates.iter().any(|named| named == path) {
        return Err(format!("its [catalog] crates already names {path:?}"));
    }
    let mut document: toml_edit::DocumentMut = text
        .parse()
        .map_err(|error| format!("does not parse: {error}"))?;
    let crates = document
        .get_mut("catalog")
        .and_then(|catalog| catalog.get_mut("crates"))
        .and_then(toml_edit::Item::as_array_mut)
        .ok_or_else(|| "its [catalog] crates is not an array".to_string())?;
    crates.push(path);
    Ok(document.to_string())
}

/// `embsim new --catalog DIR [--add-to PROJECT]`.
pub fn new_catalog(dir: &Path, add_to: Option<&Path>, out: &mut dyn Write) -> Result<(), String> {
    check_free(dir)?;
    // The project's own embsim checkout, when it names one, is the one the
    // crate depends on.
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
    let project_dir = |project: &Path| {
        project
            .parent()
            .filter(|dir| !dir.as_os_str().is_empty())
            .map_or_else(|| PathBuf::from("."), Path::to_path_buf)
    };
    let embsim = project
        .as_ref()
        .and_then(|(project, _, catalog)| {
            catalog
                .as_ref()
                .and_then(|catalog| catalog.embsim.as_deref())
                .map(|path| project_dir(project).join(path))
        })
        .unwrap_or_else(crate::source_dir);
    let embsim = embsim.canonicalize().unwrap_or(embsim);
    let scaffold = write_crate(dir, &embsim)?;
    scaffold.say(out);
    match project {
        Some((project, text, _)) => {
            let path = relative_path(dir, &project_dir(project))?;
            let path = path.to_string_lossy();
            let edited = add_crate(&text, &path)
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
        }
        None => {
            let _ = writeln!(
                out,
                "  a project names it in its [catalog]: crates = [\"<this directory, relative to \
                 the project file>\"]; or `embsim new --catalog DIR --add-to PROJECT` does"
            );
        }
    }
    if let Some(workspace) = enclosing_workspace(dir) {
        let _ = writeln!(
            out,
            "note: {} sits inside the Cargo workspace {}; add it to that workspace's members to \
             build and test it there (the runner builds it either way)",
            dir.display(),
            workspace.display()
        );
    }
    Ok(())
}

/// The manifest of the nearest Cargo workspace above `dir`, if any.
fn enclosing_workspace(dir: &Path) -> Option<PathBuf> {
    let dir = dir.canonicalize().ok()?;
    dir.ancestors().skip(1).find_map(|ancestor| {
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
    fn a_crate_joins_an_existing_catalog_table_keeping_its_comments() {
        let text = "[catalog]\n# ours\ncrates = [\"a\"] # first\n\n[[board]]\nname = \"B\"\n";
        let added = add_crate(text, "b").unwrap();
        assert!(added.contains("# ours"), "{added}");
        assert!(added.contains("# first"), "{added}");
        assert_eq!(
            CatalogTable::of_project_text(&added)
                .unwrap()
                .unwrap()
                .crates,
            ["a", "b"]
        );
        assert!(add_crate(&added, "a")
            .unwrap_err()
            .contains("already names"));
    }
}
