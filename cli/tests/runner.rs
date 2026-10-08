//! A project's own catalog crates and the runner: `embsim new --catalog`,
//! the `[catalog]` table, and the `embsim` tool writing, building and
//! handing a project to the runner that holds its crates.
//!
//! Two speeds. The cases that need no build run the binary as a user runs
//! it, a process a case: the crate `new --catalog` starts, the refusals the
//! tool makes before Cargo, the runner's files written when no Cargo
//! starts, and a runner refusing a project its crates are not. The cases
//! that build a runner with Cargo — `examples/custom-project` checked and
//! run through the real binary and its runner, a rebuild, the started
//! crate built and run, a crate that does not compile — take a build of
//! embsim in the `release` profile the first time, so they are
//! `#[ignore]`d here and run by CI's `project-runner` job:
//!
//! ```text
//! cargo test -p embsim-cli --test runner -- --ignored
//! ```
//!
//! Like the QEMU boot in `cli.rs`, those declare no behaviour: the ledger's
//! suite does not build runners, so a claim there would never be seen to
//! hold. What they prove is said in each case's comment.

use std::path::{Path, PathBuf};
use std::process::{Command, ExitCode, Output};

use embsim_cli::CatalogCrate;
use rstest::rstest;
use vibes_behaviour::{behaviour, expect, Test};

/// The workspace root.
fn workspace() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("the CLI crate sits in the workspace")
        .to_path_buf()
}

/// The template crate `new --catalog` copies (`cli/catalog-template`).
const TEMPLATE_DIR: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/catalog-template");

/// A directory of the test's own inside the embsim checkout, emptied: the
/// checkout is then inside the crate's git repository, as embsim is in a
/// project that keeps it as a submodule, whichever target directory the
/// tests build in.
fn scratch(test: &str) -> PathBuf {
    let dir = workspace().join("target").join("runner-tests").join(test);
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).expect("the scratch directory can be made");
    dir
}

/// `embsim` with `args`, run in `dir`, with `env` set.
fn embsim_in(dir: &Path, args: &[&str], env: &[(&str, &str)]) -> Output {
    let mut command = Command::new(env!("CARGO_BIN_EXE_embsim"));
    command.args(args).current_dir(dir);
    for (key, value) in env {
        command.env(key, value);
    }
    command.output().expect("the embsim binary runs")
}

fn stdout(output: &Output) -> String {
    plain(&String::from_utf8_lossy(&output.stdout))
}

fn stderr(output: &Output) -> String {
    plain(&String::from_utf8_lossy(&output.stderr))
}

/// `text` without terminal colour escapes (`ESC [ ... letter`): Cargo and
/// rustc colour their messages when the environment asks for it, as CI's
/// CARGO_TERM_COLOR=always does, and the assertions compare the words.
fn plain(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    let mut chars = text.chars().peekable();
    while let Some(c) = chars.next() {
        if c == '\u{1b}' && chars.peek() == Some(&'[') {
            chars.next();
            for c in chars.by_ref() {
                if c.is_ascii_alphabetic() {
                    break;
                }
            }
        } else {
            out.push(c);
        }
    }
    out
}

fn read(path: &Path) -> String {
    std::fs::read_to_string(path).unwrap_or_else(|err| panic!("{}: {err}", path.display()))
}

fn assert_says(text: &str, needles: &[&str]) {
    for needle in needles {
        assert!(text.contains(needle), "{needle:?} missing from:\n{text}");
    }
}

/// The one runner directory under `dir/.embsim`.
fn runner_dir(dir: &Path) -> PathBuf {
    let runners: Vec<PathBuf> = std::fs::read_dir(dir.join(".embsim"))
        .expect("the tool made .embsim")
        .map(|entry| entry.expect("an entry").path())
        .filter(|path| {
            path.file_name()
                .is_some_and(|name| name.to_string_lossy().starts_with("runner-"))
        })
        .collect();
    assert_eq!(runners.len(), 1, "{runners:?}");
    runners.into_iter().next().expect("one runner")
}

// ============================================================
// embsim new --catalog
// ============================================================

#[rstest]
fn new_catalog_starts_a_crate_and_names_it_in_the_project() {
    behaviour!(Test {
        id: "cli.new-catalog-scaffold",
        covers: Some("cli/src/scaffold.rs#new_catalog"),
        given: "`embsim new --catalog` asked for a crate in a directory named rig/catalog and \
                to add it to an existing project file that has a comment of its own and no \
                catalog crates",
    });
    expect!(
        "crate-written",
        "the directory gets a Cargo package named rig-catalog, and a library that is the \
         started catalog with every kind named rig-board, rig-sensor, rig-core and rig-source",
        "a kind starts with its project's name, so no kind embsim ships ever meets it"
    );
    expect!(
        "embsim-by-path",
        "the package's embsim dependencies reach, by paths relative to it, the embsim checkout \
         the command was built from, which sits in the same repository as the crate",
        "embsim inside a project, as a submodule is, is the project's embsim"
    );
    expect!(
        "project-names-it",
        "the project file now names the crate among its catalog crates, relative to the file, \
         and keeps its comment and its boards"
    );
    let dir = scratch("new_catalog");
    let project = dir.join("rig.toml");
    std::fs::write(
        &project,
        "# The rig.\n[[board]] # the header\nname = \"HDR\"\nkind = \"netlist\"\nnetlist = \
         \"header.net\"\n",
    )
    .expect("the project is writable");
    let output = embsim_in(
        &dir,
        &["new", "--catalog", "rig/catalog", "--add-to", "rig.toml"],
        &[],
    );
    assert!(output.status.success(), "{}", stderr(&output));
    assert_says(
        &stdout(&output),
        &[
            "catalog crate rig-catalog",
            "added \"rig/catalog\" to the [catalog] crates of rig.toml",
        ],
    );

    let manifest = read(&dir.join("rig/catalog/Cargo.toml"));
    let table: toml::Table = toml::from_str(&manifest).expect("the manifest parses");
    assert_eq!(table["package"]["name"].as_str(), Some("rig-catalog"));
    let crate_dir = dir.join("rig/catalog");
    for dependency in ["embsim-board", "embsim-boards", "embsim-core"] {
        let path = table["dependencies"][dependency]["path"]
            .as_str()
            .expect("a path dependency");
        let reached = crate_dir
            .join(path)
            .canonicalize()
            .expect("the path is there");
        let checkout = workspace().join(dependency.trim_start_matches("embsim-"));
        assert_eq!(reached, checkout.canonicalize().unwrap(), "{dependency}");
    }
    let lib = read(&crate_dir.join("src/lib.rs"));
    let template = read(&Path::new(TEMPLATE_DIR).join("src/lib.rs"));
    assert_eq!(
        lib,
        template
            .replace("yourproject", "rig")
            .replace("YOURPROJECT", "RIG")
    );
    assert!(lib.contains("const CORE: &str = \"rig-core\";"), "{lib}");

    let text = read(&project);
    assert!(text.starts_with("# The rig.\n"), "{text}");
    assert!(text.contains("[[board]] # the header"), "{text}");
    let catalog = embsim_board::CatalogTable::of_project_text(&text)
        .expect("the project parses")
        .expect("it has a [catalog]");
    assert_eq!(catalog.crates, ["rig/catalog"]);
}

#[rstest]
fn new_with_a_netlist_and_a_catalog_writes_a_project_that_names_the_crate() {
    behaviour!(Test {
        id: "cli.new-project-with-catalog",
        covers: Some("cli/src/checklist.rs#new_project"),
        given: "`embsim new` for the header board's netlist, writing the project to a file, \
                with a catalog crate asked for in a directory beside it",
    });
    expect!(
        "both-written",
        "the starter project and the crate are both written, and the project's catalog \
         crates name the crate relative to the project file"
    );
    expect!(
        "refuses-a-full-directory",
        "asked again for the same crate directory, the command refuses before writing \
         anything, saying the directory is not empty",
        "a crate starts in an empty or new directory, never over files"
    );
    let dir = scratch("new_project_catalog");
    std::fs::copy(
        workspace().join("boards/projects/header.net"),
        dir.join("header.net"),
    )
    .expect("the netlist copies");
    let output = embsim_in(
        &dir,
        &[
            "new",
            "header.net",
            "-o",
            "hdr.toml",
            "--catalog",
            "sim/catalog",
        ],
        &[],
    );
    assert!(output.status.success(), "{}", stderr(&output));
    assert!(dir.join("sim/catalog/src/lib.rs").exists());
    let text = read(&dir.join("hdr.toml"));
    let project = embsim_board::Project::parse(&text).expect("the project parses");
    let catalog = project.catalog().expect("it names a catalog crate");
    assert_eq!(catalog.crates, ["sim/catalog"]);

    let again = embsim_in(
        &dir,
        &[
            "new",
            "header.net",
            "-o",
            "other.toml",
            "--catalog",
            "sim/catalog",
        ],
        &[],
    );
    assert!(!again.status.success());
    assert_says(
        &stderr(&again),
        &["--catalog sim/catalog: the directory is not empty"],
    );
    assert!(!dir.join("other.toml").exists());
}

#[rstest]
#[case::release_tag("tag")]
#[case::commit("rev")]
#[case::checkout("path")]
fn a_crate_started_with_embsim_named_takes_embsim_from_there(#[case] form: &str) {
    behaviour!(Test {
        id: "cli.new-catalog-embsim-named",
        covers: Some("cli/src/scaffold.rs#parse_embsim"),
        given: "`embsim new --catalog` asked for a crate outside every git repository, with \
                --embsim naming embsim's repository at a release tag, the same repository at a \
                commit, or an embsim checkout's directory",
    });
    expect!(
        "named-source",
        "each embsim dependency of the crate is the source named: the repository at that tag, \
         or at that commit as its revision, each with the command's release as the version \
         Cargo checks; or the checkout, by a path relative to the crate",
        "a crate's embsim dependency is the embsim the project builds against, and whoever \
         starts the crate may say which"
    );
    expect!(
        "said",
        "the command says where the crate's embsim, and so its runner's, comes from"
    );
    let dir = outside(&format!("new_named_{form}"));
    let checkout = workspace().canonicalize().unwrap();
    let named = match form {
        "tag" => format!(
            "https://github.com/RileyMcCarthy/embsim@v{}",
            env!("CARGO_PKG_VERSION")
        ),
        "rev" => "https://github.com/RileyMcCarthy/embsim@0123abcd".to_string(),
        _ => checkout.display().to_string(),
    };
    let output = embsim_in(
        &dir,
        &["new", "--catalog", "rig/catalog", "--embsim", &named],
        &[],
    );
    assert!(output.status.success(), "{}", stderr(&output));
    let crate_dir = dir.join("rig/catalog");
    let table: toml::Table =
        toml::from_str(&read(&crate_dir.join("Cargo.toml"))).expect("the manifest parses");
    for dependency in ["embsim-board", "embsim-boards", "embsim-core"] {
        let dep = &table["dependencies"][dependency];
        if form == "path" {
            let path = dep["path"].as_str().expect("a path dependency");
            assert_eq!(
                crate_dir.join(path).canonicalize().unwrap(),
                checkout.join(dependency.trim_start_matches("embsim-"))
            );
            continue;
        }
        assert_eq!(
            dep["git"].as_str(),
            Some("https://github.com/RileyMcCarthy/embsim"),
            "{dependency}"
        );
        assert_eq!(dep["version"].as_str(), Some(release().as_str()));
        let expected = if form == "tag" {
            format!("v{}", env!("CARGO_PKG_VERSION"))
        } else {
            "0123abcd".to_string()
        };
        assert_eq!(dep[form].as_str(), Some(expected.as_str()), "{dep}");
    }
    let said = if form == "path" {
        format!("its embsim, and so the runner's: at {}", checkout.display())
    } else {
        "its embsim, and so the runner's: from git https://github.com/RileyMcCarthy/embsim"
            .to_string()
    };
    assert_says(&stdout(&output), &[&said]);
    assert!(!stdout(&output).contains("note:"), "{}", stdout(&output));
}

#[rstest]
fn embsim_that_is_not_a_source_or_not_the_projects_is_refused_before_anything_is_written() {
    behaviour!(Test {
        id: "cli.new-catalog-embsim-refused",
        covers: Some("cli/src/scaffold.rs#embsim_for"),
        given: "`embsim new --catalog` with --embsim naming a repository with no tag or commit, \
                then a directory that is not an embsim checkout, then a source other than the \
                one the catalog crates of the project it joins take",
    });
    expect!(
        "names-what-it-takes",
        "each is refused naming the value, and saying what --embsim takes or which embsim the \
         project's crates take",
        "every machine must build the same embsim, and a runner holds one copy of it"
    );
    expect!(
        "nothing-written",
        "no crate is written and the project file is unchanged"
    );
    let dir = outside("new_named_refused");
    git_crate(&dir.join("first"), "rig-first", "0123abcd");
    std::fs::write(dir.join("p.toml"), "[catalog]\ncrates = [\"first\"]\n").expect("writable");
    for (args, says) in [
        (
            vec!["--embsim", "https://github.com/RileyMcCarthy/embsim"],
            "--embsim https://github.com/RileyMcCarthy/embsim: name a tag or a revision after \
             the repository",
        ),
        (
            vec!["--embsim", "first"],
            "--embsim first: neither a git repository at a tag or revision",
        ),
        (
            vec![
                "--embsim",
                "https://github.com/RileyMcCarthy/embsim@v0.2.0",
                "--add-to",
                "p.toml",
            ],
            "--embsim names embsim from git https://github.com/RileyMcCarthy/embsim tag v0.2.0, \
             and the project's catalog crates take embsim from git \
             https://example.invalid/embsim.git rev 0123abcd",
        ),
    ] {
        let mut line = vec!["new", "--catalog", "second"];
        line.extend(args);
        let output = embsim_in(&dir, &line, &[]);
        assert!(!output.status.success(), "{line:?}");
        assert_says(&stderr(&output), &[says]);
        assert!(!dir.join("second").exists(), "{line:?}");
    }
    assert_eq!(
        read(&dir.join("p.toml")),
        "[catalog]\ncrates = [\"first\"]\n"
    );
}

#[rstest]
fn a_crate_that_joins_a_project_takes_the_embsim_its_crates_take() {
    behaviour!(Test {
        id: "cli.new-catalog-joins",
        covers: Some("cli/src/scaffold.rs#new_catalog"),
        given: "`embsim new --catalog --add-to` for a project whose catalog crate takes embsim \
                from a git repository at a revision",
    });
    expect!(
        "same-embsim",
        "the new crate's embsim dependencies are that repository at that revision",
        "a runner holds one copy of embsim, so every catalog crate of a project takes the same"
    );
    let dir = outside("new_joins");
    git_crate(&dir.join("first"), "rig-first", "0123abcd");
    std::fs::write(dir.join("p.toml"), "[catalog]\ncrates = [\"first\"]\n").expect("writable");
    let output = embsim_in(
        &dir,
        &["new", "--catalog", "second", "--add-to", "p.toml"],
        &[],
    );
    assert!(output.status.success(), "{}", stderr(&output));
    let table: toml::Table =
        toml::from_str(&read(&dir.join("second/Cargo.toml"))).expect("the manifest parses");
    let dep = &table["dependencies"]["embsim-boards"];
    assert_eq!(dep["git"].as_str(), Some(OTHER_REPOSITORY));
    assert_eq!(dep["rev"].as_str(), Some("0123abcd"));
    let catalog = embsim_board::CatalogTable::of_project(&dir.join("p.toml"))
        .expect("the project reads")
        .expect("it has a [catalog]");
    assert_eq!(catalog.crates, ["first", "second"]);
}

#[rstest]
fn new_catalog_with_its_own_runner_starts_both_crates() {
    behaviour!(Test {
        id: "cli.new-own-runner",
        covers: Some("cli/src/scaffold.rs#write_runner_crate"),
        given: "`embsim new --catalog sim/catalog --own-runner --add-to` a project, in a \
                directory of a Cargo workspace",
    });
    expect!(
        "runner-crate",
        "a binary crate is written beside the catalog crate, in sim/runner: it depends on the \
         embsim command from the catalog crate's embsim and on the catalog crate by path, and \
         its main runs the command over the catalog crate, found from the runner's own \
         directory",
        "the project's own runner is the same command a runner the tool writes is"
    );
    expect!(
        "named-as-runner",
        "the project names the catalog crate among its crates and the binary crate as its \
         runner, both relative to the file"
    );
    expect!(
        "workspace-note",
        "the command says the two crates sit inside the workspace and are to be added to its \
         members"
    );
    let dir = scratch("own_runner_new");
    std::fs::write(dir.join("Cargo.toml"), "[workspace]\nmembers = []\n").expect("writable");
    std::fs::write(dir.join("rig.toml"), "# The rig.\n").expect("writable");
    let output = embsim_in(
        &dir,
        &[
            "new",
            "--catalog",
            "sim/catalog",
            "--own-runner",
            "--add-to",
            "rig.toml",
        ],
        &[],
    );
    assert!(output.status.success(), "{}", stderr(&output));
    let runner = dir.join("sim/runner");
    let table: toml::Table =
        toml::from_str(&read(&runner.join("Cargo.toml"))).expect("the manifest parses");
    assert_eq!(table["package"]["name"].as_str(), Some("sim-runner"));
    assert_eq!(table["bin"][0]["name"].as_str(), Some("sim-runner"));
    assert!(
        !table.contains_key("workspace"),
        "a member of the workspace above it"
    );
    let cli = table["dependencies"]["embsim-cli"]["path"]
        .as_str()
        .expect("embsim by path");
    assert_eq!(
        runner.join(cli).canonicalize().unwrap(),
        workspace().join("cli").canonicalize().unwrap()
    );
    assert_eq!(
        table["dependencies"]["sim-catalog"]["path"].as_str(),
        Some("../catalog")
    );
    let main = read(&runner.join("src/main.rs"));
    assert_says(
        &main,
        &[
            "embsim_cli::runner_main(&[embsim_cli::CatalogCrate::new(",
            "\"sim-catalog\"",
            "concat!(env!(\"CARGO_MANIFEST_DIR\"), \"/../catalog\")",
            "sim_catalog::register",
        ],
    );
    let catalog = embsim_board::CatalogTable::of_project(&dir.join("rig.toml"))
        .expect("the project reads")
        .expect("it has a [catalog]");
    assert_eq!(catalog.crates, ["sim/catalog"]);
    assert_eq!(catalog.runner.as_deref(), Some("sim/runner"));
    assert_says(
        &stdout(&output),
        &[
            "the project's runner sim-runner",
            "sim/catalog and sim/runner sit inside the Cargo workspace",
            "add them to that workspace's members",
        ],
    );
}

/// `git` in `dir` with `args`, a commit needing no configuration.
fn git(dir: &Path, args: &[&str]) -> String {
    let output = Command::new("git")
        .arg("-C")
        .arg(dir)
        .args([
            "-c",
            "user.name=embsim",
            "-c",
            "user.email=embsim@example.invalid",
            "-c",
            "commit.gpgsign=false",
        ])
        .args(args)
        .output()
        .expect("git runs");
    assert!(output.status.success(), "git {args:?}: {output:?}");
    String::from_utf8_lossy(&output.stdout).trim().to_string()
}

#[rstest]
fn a_runner_crate_of_its_own_keeps_its_builds_out_of_the_projects_commits() {
    behaviour!(Test {
        id: "cli.own-runner-gitignore",
        covers: Some("cli/src/scaffold.rs#write_runner_crate"),
        given: "`embsim new --catalog sim/catalog --own-runner` in a git repository no Cargo \
                workspace covers, everything it wrote committed, and then the files a build of \
                the runner leaves in its target directory",
    });
    expect!(
        "ignores-target",
        "the runner crate, a Cargo workspace of its own, gets a .gitignore that keeps its \
         target directory out of git, and the command says where its builds and its lock file \
         go",
        "Cargo builds a workspace of its own in a target directory beside its manifest"
    );
    expect!(
        "nothing-uncommitted",
        "after a build, git finds nothing in the runner's directory that the commit does not \
         hold, which is what the runner's provenance line reads",
        "a run names the runner crate's revision, with changes only when its sources have them"
    );
    let dir = outside("own_runner_alone");
    git(&dir, &["init", "-q"]);
    std::fs::write(dir.join("rig.toml"), "# The rig.\n").expect("writable");
    let checkout = workspace().canonicalize().unwrap();
    let output = embsim_in(
        &dir,
        &[
            "new",
            "--catalog",
            "sim/catalog",
            "--own-runner",
            "--add-to",
            "rig.toml",
            "--embsim",
            checkout.to_str().expect("text"),
        ],
        &[],
    );
    assert!(output.status.success(), "{}", stderr(&output));
    let runner = dir.join("sim/runner");
    assert_eq!(read(&runner.join(".gitignore")), "/target\n");
    let table: toml::Table =
        toml::from_str(&read(&runner.join("Cargo.toml"))).expect("the manifest parses");
    assert!(table.contains_key("workspace"), "a workspace of its own");
    assert_says(
        &stdout(&output),
        &[&format!(
            "in no Cargo workspace, it is one of its own: Cargo builds it in {} ({} keeps that \
             out of git) and writes its Cargo.lock beside it, to commit",
            Path::new("sim/runner/target").display(),
            Path::new("sim/runner/.gitignore").display()
        )],
    );
    git(&dir, &["add", "-A"]);
    git(&dir, &["commit", "-qm", "the rig"]);
    // What `cargo build` leaves in a workspace of its own.
    std::fs::create_dir_all(runner.join("target/release")).expect("writable");
    std::fs::write(runner.join("target/release/sim-runner"), "built").expect("writable");
    std::fs::write(runner.join("target/CACHEDIR.TAG"), "").expect("writable");
    assert_eq!(git(&runner, &["status", "--porcelain", "--", "."]), "");
}

// ============================================================
// The tool, before Cargo
// ============================================================

#[rstest]
fn a_catalog_crate_that_is_not_there_is_refused_naming_it() {
    behaviour!(Test {
        id: "cli.runner-missing-crate",
        covers: Some("cli/src/runner.rs#hand_over"),
        given: "a project whose catalog crates name a directory that does not exist, checked \
                with the `embsim` tool",
    });
    expect!(
        "names-the-crate",
        "the tool refuses it before any build, naming the path as the file gives it and where \
         it reached, and how a crate is started"
    );
    let dir = scratch("missing_crate");
    std::fs::write(
        dir.join("p.toml"),
        "[catalog]\ncrates = [\"sim/nowhere\"]\n",
    )
    .expect("the project is writable");
    let output = embsim_in(&dir, &["check", "p.toml"], &[]);
    assert!(!output.status.success());
    assert_says(
        &stderr(&output),
        &[
            "p.toml: [catalog] crates: \"sim/nowhere\" (./sim/nowhere) is not there",
            "`embsim new --catalog DIR` starts one",
        ],
    );
    assert!(!dir.join(".embsim").exists(), "nothing is written");
}

#[rstest]
fn without_cargo_the_runner_is_written_and_the_error_says_how_to_get_cargo() {
    behaviour!(Test {
        id: "cli.runner-without-cargo",
        covers: Some("cli/src/runner.rs#hand_over"),
        given: "a project naming one catalog crate, checked with the `embsim` tool where the \
                Cargo it would build with does not start",
    });
    expect!(
        "runner-written",
        "the runner's manifest is written beside the project, in a directory of its own: \
         one binary depending on the embsim command from the embsim checkout the crate's own \
         embsim dependency names, and on the crate by its path, as a workspace of its own",
        "Cargo compiles the project's crates and embsim into one binary, against one copy of \
         embsim"
    );
    expect!(
        "main-registers-the-crate",
        "the runner's main runs the command over the catalogs embsim ships and the crate's \
         registration function"
    );
    expect!(
        "kept-out-of-git",
        "the directory embsim keeps beside the project ignores everything in it"
    );
    expect!(
        "says-cargo",
        "the check fails saying the project's catalog crates need Cargo, naming the one that \
         did not start, where to install Rust, and that a binary of the project's own needs \
         no runner"
    );
    let dir = scratch("no_cargo");
    std::fs::write(
        dir.join("p.toml"),
        format!("[catalog]\ncrates = [{TEMPLATE_DIR:?}]\n"),
    )
    .expect("the project is writable");
    let output = embsim_in(
        &dir,
        &["check", "p.toml"],
        &[("CARGO", "/nonexistent/embsim-test/cargo")],
    );
    assert!(!output.status.success());
    assert_says(
        &stderr(&output),
        &[
            "p.toml names catalog crates (yourproject-catalog), which embsim builds into a \
             runner with Cargo",
            "$CARGO names /nonexistent/embsim-test/cargo, which does not start",
            "https://rustup.rs",
            "embsim_cli::main_with",
        ],
    );

    let runner = runner_dir(&dir);
    let manifest = read(&runner.join("Cargo.toml"));
    let table: toml::Table = toml::from_str(&manifest).expect("the manifest parses");
    let package = table["package"]["name"].as_str().expect("a name");
    assert!(package.starts_with("embsim-runner-"), "{package}");
    assert_eq!(table["bin"][0]["name"].as_str(), Some(package));
    assert!(table.contains_key("workspace"), "{manifest}");
    let path = |name: &str| {
        PathBuf::from(
            table["dependencies"][name]["path"]
                .as_str()
                .unwrap_or_else(|| panic!("{name} is a path dependency")),
        )
    };
    assert_eq!(
        path("embsim-cli"),
        workspace().canonicalize().unwrap().join("cli")
    );
    assert_eq!(
        path("yourproject-catalog"),
        Path::new(TEMPLATE_DIR).canonicalize().unwrap()
    );
    let main = read(&runner.join("main.rs"));
    assert_says(
        &main,
        &[
            "embsim_cli::runner_main(&[",
            "embsim_cli::CatalogCrate::new(",
            "\"yourproject-catalog\"",
            "yourproject_catalog::register",
        ],
    );
    assert_eq!(
        read(&dir.join(".embsim/.gitignore")).lines().last(),
        Some("*")
    );
}

#[rstest]
fn a_runner_refuses_a_project_that_names_other_catalog_crates() {
    behaviour!(Test {
        id: "cli.runner-refuses-other-crates",
        covers: Some("cli/src/runner.rs#check_runner_fits"),
        given: "a runner holding the started catalog crate, asked to check a project whose \
                catalog crates name a different directory",
    });
    expect!(
        "refused",
        "the runner refuses the project, naming the crates it holds and the ones the project \
         names, and says to run it with the `embsim` tool",
        "the tool builds one runner for each set of crates a project names"
    );
    let dir = scratch("other_crates");
    let other = dir.join("other");
    std::fs::create_dir_all(&other).expect("the directory can be made");
    let project = dir.join("p.toml");
    std::fs::write(&project, "[catalog]\ncrates = [\"other\"]\n").expect("writable");
    let crates = [CatalogCrate::new(
        "yourproject-catalog",
        TEMPLATE_DIR,
        yourproject_catalog::register,
    )];
    let (mut out, mut err) = (Vec::new(), Vec::new());
    let code = embsim_cli::run_with_crates(
        &crates,
        ["embsim", "check", project.to_str().expect("text")],
        &mut out,
        &mut err,
    );
    assert_eq!(code, ExitCode::FAILURE);
    let err = String::from_utf8_lossy(&err);
    assert_says(
        &err,
        &[
            &format!(
                "this runner holds the catalog crates {}",
                Path::new(TEMPLATE_DIR).canonicalize().unwrap().display()
            ),
            &format!(
                "and the project names {}",
                other.canonicalize().unwrap().display()
            ),
            "run the project with `embsim`",
        ],
    );
}

// ============================================================
// survey and new with --project: a project's own kinds
// ============================================================

/// A board netlist naming the started crate's sensor: a two-pin header
/// `J1` and `U1`, valued as the crate's sensor kind places it.
const SENSOR_BOARD: &str = r#"(export (version "E")
  (components
    (comp (ref "J1") (value "Conn_01x02")
      (libsource (lib "Connector") (part "Conn_01x02")))
    (comp (ref "U1") (value "YOURPROJECT-SENSOR")
      (libsource (lib "rig") (part "Sensor"))))
  (nets
    (net (code "1") (name "IN")
      (node (ref "J1") (pin "1") (pinfunction "Pin_1"))
      (node (ref "U1") (pin "1")))
    (net (code "2") (name "GND")
      (node (ref "J1") (pin "2") (pinfunction "Pin_2"))
      (node (ref "U1") (pin "2")))))"#;

/// The command as the started crate's runner, with `args`.
fn template_runner(args: &[&str]) -> (ExitCode, String, String) {
    let crates = [CatalogCrate::new(
        "yourproject-catalog",
        TEMPLATE_DIR,
        yourproject_catalog::register,
    )];
    let (mut out, mut err) = (Vec::new(), Vec::new());
    let code = embsim_cli::run_with_crates(
        &crates,
        std::iter::once("embsim").chain(args.iter().copied()),
        &mut out,
        &mut err,
    );
    (
        code,
        String::from_utf8_lossy(&out).into_owned(),
        String::from_utf8_lossy(&err).into_owned(),
    )
}

#[rstest]
fn survey_with_a_project_names_the_projects_own_kinds() {
    behaviour!(Test {
        id: "cli.survey-with-project",
        covers: Some("cli/src/lib.rs#Command::project"),
        given: "a project naming the started catalog crate, and a board netlist whose sensor \
                part only that crate has a kind for, each surveyed through the crate's runner \
                with --project naming the project, as is the crate's own board kind",
    });
    expect!(
        "candidates-include-the-crates",
        "the netlist's checklist offers the crate's sensor kind for the sensor part",
        "with --project a survey runs over the catalogs the project's runner holds"
    );
    expect!(
        "kind-surveyed",
        "the crate's board kind is surveyed, its sensor the one part left to the project"
    );
    let dir = scratch("survey_project");
    std::fs::write(
        dir.join("rig.toml"),
        format!("[catalog]\ncrates = [{TEMPLATE_DIR:?}]\n"),
    )
    .expect("writable");
    std::fs::write(dir.join("board.net"), SENSOR_BOARD).expect("writable");
    let project = dir.join("rig.toml");
    let project = project.to_str().expect("text");
    let netlist = dir.join("board.net");

    let (code, out, err) = template_runner(&[
        "survey",
        "--project",
        project,
        netlist.to_str().expect("text"),
    ]);
    assert_eq!(code, ExitCode::SUCCESS, "{err}");
    assert_says(&out, &["could be: yourproject-sensor"]);

    let (code, out, err) = template_runner(&[
        "survey",
        "--project",
        project,
        "--kind",
        "yourproject-board",
    ]);
    assert_eq!(code, ExitCode::SUCCESS, "{err}");
    assert_says(
        &out,
        &[
            "kind \"yourproject-board\"",
            "1 need a model",
            "could be: yourproject-sensor",
        ],
    );
}

#[rstest]
fn a_kind_only_a_project_adds_is_surveyed_through_the_project() {
    behaviour!(Test {
        id: "cli.survey-kind-hint",
        covers: Some("cli/src/checklist.rs#survey_kind"),
        given: "the `embsim` tool asked to survey a board kind that only a project's catalog \
                crate adds, with no project named",
    });
    expect!(
        "says-project",
        "the survey is refused as an unknown kind, listing the kinds the tool ships and \
         saying to survey the kind with --project naming the project"
    );
    let dir = scratch("survey_kind_hint");
    let tool = embsim_in(&dir, &["survey", "--kind", "yourproject-board"], &[]);
    assert!(!tool.status.success());
    assert_says(
        &stderr(&tool),
        &[
            "unknown kind \"yourproject-board\"; the board kinds are \"netlist\", \"p2-ec32mb\"",
            "embsim survey --project FILE --kind yourproject-board",
        ],
    );
}

#[rstest]
fn the_tool_hands_a_survey_with_a_project_to_its_runner() {
    behaviour!(Test {
        id: "cli.survey-project-hand-over",
        covers: Some("cli/src/lib.rs#tool_main"),
        given: "a survey given --project naming a project whose catalog crate is not there, \
                run by the `embsim` tool",
    });
    expect!(
        "refused-as-check-is",
        "the tool refuses it before any build, naming the crate path, as it refuses a check \
         of that project",
        "a survey with --project runs in the project's runner, so the runner's crates must be \
         there"
    );
    let dir = scratch("survey_hand_over");
    std::fs::write(
        dir.join("p.toml"),
        "[catalog]\ncrates = [\"sim/nowhere\"]\n",
    )
    .expect("writable");
    let output = embsim_in(
        &dir,
        &["survey", "--project", "p.toml", "--kind", "p2-ec32mb"],
        &[],
    );
    assert!(!output.status.success());
    assert_says(
        &stderr(&output),
        &["p.toml: [catalog] crates: \"sim/nowhere\" (./sim/nowhere) is not there"],
    );
}

#[rstest]
fn new_with_a_project_offers_its_kinds_and_carries_its_catalog() {
    behaviour!(Test {
        id: "cli.new-with-project",
        covers: Some("cli/src/checklist.rs#new_project"),
        given: "`embsim new` for a board netlist whose sensor part only the started catalog \
                crate has a kind for, through the crate's runner with --project naming a \
                project that names the crate, writing the starter project into a directory \
                below the project's",
    });
    expect!(
        "stub-offers-the-kind",
        "the sensor's stub in the starter project offers the crate's sensor kind"
    );
    expect!(
        "catalog-carried",
        "the starter project names the same catalog crate by a path that reaches it from the \
         starter project's own directory",
        "a starter project runs through the same runner as the project it was started from"
    );
    let dir = scratch("new_project");
    // The project names its crate relative to itself, through a link in
    // its own directory, so the path only reaches the crate from there.
    std::os::unix::fs::symlink(TEMPLATE_DIR, dir.join("catalog")).expect("the link can be made");
    std::fs::write(dir.join("rig.toml"), "[catalog]\ncrates = [\"catalog\"]\n").expect("writable");
    std::fs::write(dir.join("board.net"), SENSOR_BOARD).expect("writable");
    std::fs::create_dir_all(dir.join("boards")).expect("the directory can be made");
    let starter = dir.join("boards/board.toml");
    let (code, out, err) = template_runner(&[
        "new",
        "--project",
        dir.join("rig.toml").to_str().expect("text"),
        dir.join("board.net").to_str().expect("text"),
        "-o",
        starter.to_str().expect("text"),
    ]);
    assert_eq!(code, ExitCode::SUCCESS, "{err}\n{out}");
    let text = read(&starter);
    assert_says(&text, &["yourproject-sensor"]);
    let catalog = embsim_board::CatalogTable::of_project_text(&text)
        .expect("the project parses")
        .expect("it has a [catalog]");
    assert_eq!(catalog.crates.len(), 1, "{text}");
    assert_eq!(
        dir.join("boards")
            .join(&catalog.crates[0])
            .canonicalize()
            .unwrap(),
        Path::new(TEMPLATE_DIR).canonicalize().unwrap()
    );
}

// ============================================================
// The tool and Cargo, with a stand-in Cargo that builds nothing
// ============================================================
//
// `$CARGO` names a script that writes each command line it is given to a
// log and hands `cargo metadata` (a read of the manifests and the lock file,
// no build) to the real Cargo; anything else, a build included, it fails.
// So these cases see every decision the tool makes before it builds — the
// target directory, the embsim the runner's manifest names, the lock file,
// the refusals — and what it says of a build that failed, without paying
// for a build of embsim. A case whose crates take embsim from git hands on
// only `cargo metadata --no-deps`, which reads manifests and fetches
// nothing.

/// A directory of the test's own outside every Cargo workspace and git
/// repository (embsim's target directory is inside both), emptied.
fn outside(test: &str) -> PathBuf {
    let dir = std::env::temp_dir()
        .join(format!("embsim-runner-{}", std::process::id()))
        .join(test);
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).expect("the directory can be made");
    dir.canonicalize().expect("the directory is there")
}

/// The stand-in Cargo in `dir`, and the log it writes: `cargo metadata`
/// goes to the real Cargo, all of it, or only `--no-deps` when `resolve`
/// is false.
fn logging_cargo_with(dir: &Path, resolve: bool) -> (PathBuf, PathBuf) {
    use std::os::unix::fs::PermissionsExt;
    let script = dir.join("cargo-log.sh");
    let log = dir.join("cargo.log");
    let pattern = if resolve {
        "metadata*"
    } else {
        "metadata\\ --no-deps*"
    };
    std::fs::write(
        &script,
        format!(
            "#!/bin/sh\nprintf '%s\\n' \"$*\" >> {log:?}\ncase \"$*\" in\n  {pattern}) exec \
             {cargo:?} \"$@\" ;;\nesac\nexit 101\n",
            cargo = env!("CARGO"),
        ),
    )
    .expect("the script is writable");
    std::fs::set_permissions(&script, std::fs::Permissions::from_mode(0o755))
        .expect("the script is executable");
    (script, log)
}

/// [`logging_cargo_with`], every `cargo metadata` to the real Cargo.
fn logging_cargo(dir: &Path) -> (PathBuf, PathBuf) {
    logging_cargo_with(dir, true)
}

/// `embsim` with `args` in `dir`, Cargo the stand-in `cargo`, and no
/// `CARGO_TARGET_DIR`: what the tool decides on its own.
fn embsim_logged(dir: &Path, cargo: &Path, args: &[&str]) -> Output {
    Command::new(env!("CARGO_BIN_EXE_embsim"))
        .args(args)
        .current_dir(dir)
        .env("CARGO", cargo)
        .env_remove("CARGO_TARGET_DIR")
        .env_remove("EMBSIM_RUNNER_PROFILE")
        .output()
        .expect("the embsim binary runs")
}

/// The build line in `log`, split into words.
fn build_line(log: &Path) -> Vec<String> {
    let text = read(log);
    let build = text
        .lines()
        .find(|line| line.starts_with("build "))
        .unwrap_or_else(|| panic!("no build in:\n{text}"));
    build.split(' ').map(str::to_string).collect()
}

/// The `--target-dir` the build in `log` was given.
fn build_target(log: &Path) -> PathBuf {
    let words = build_line(log);
    let at = words
        .iter()
        .position(|word| word == "--target-dir")
        .expect("the build names a target directory");
    PathBuf::from(&words[at + 1])
}

/// A stand-in embsim checkout in `dir`: its `embsim-cli`, and `embsim-board`
/// and `embsim-boards` over its own `embsim-core`, which claims `links =
/// "embsim-core"` as embsim's does — a second copy as Cargo sees one.
fn other_checkout(dir: &Path) -> PathBuf {
    for (crate_dir, package, deps) in [
        ("cli", "embsim-cli", ""),
        ("core", "embsim-core", ""),
        (
            "board",
            "embsim-board",
            "embsim-core = { path = \"../core\" }\n",
        ),
        (
            "boards",
            "embsim-boards",
            "embsim-board = { path = \"../board\" }\n",
        ),
    ] {
        let at = dir.join(crate_dir);
        std::fs::create_dir_all(at.join("src")).expect("the directory can be made");
        let links = if package == "embsim-core" {
            std::fs::write(at.join("build.rs"), "fn main() {}\n").expect("writable");
            "links = \"embsim-core\"\n"
        } else {
            ""
        };
        std::fs::write(
            at.join("Cargo.toml"),
            format!(
                "[package]\nname = \"{package}\"\nversion = \"0.1.0\"\nedition = \
                 \"2021\"\n{links}\n[dependencies]\n{deps}"
            ),
        )
        .expect("the manifest is writable");
        std::fs::write(at.join("src/lib.rs"), "").expect("the library is writable");
    }
    dir.to_path_buf()
}

/// A catalog crate `rig-catalog` in `dir` whose embsim dependency,
/// `embsim-boards`, is in the checkout `root`.
fn crate_on(dir: &Path, root: &Path) {
    library_on(
        dir,
        "rig-catalog",
        &[("embsim-boards", &root.join("boards"))],
    );
}

/// A library package `name` in `dir` depending on each `(package, path)`.
fn library_on(dir: &Path, name: &str, deps: &[(&str, &Path)]) {
    std::fs::create_dir_all(dir.join("src")).expect("the directory can be made");
    let deps: String = deps
        .iter()
        .map(|(package, path)| {
            format!(
                "{package} = {{ path = {:?} }}\n",
                path.display().to_string()
            )
        })
        .collect();
    std::fs::write(
        dir.join("Cargo.toml"),
        format!(
            "[package]\nname = \"{name}\"\nversion = \"0.1.0\"\nedition = \"2021\"\n\n\
             [dependencies]\n{deps}"
        ),
    )
    .expect("the manifest is writable");
    std::fs::write(dir.join("src/lib.rs"), "").expect("the library is writable");
}

/// A repository the git-sourced crates name; nothing fetches it.
const OTHER_REPOSITORY: &str = "https://example.invalid/embsim.git";

/// A catalog crate `name` in `dir` whose embsim dependencies are
/// [`OTHER_REPOSITORY`] at the revision `rev`.
fn git_crate(dir: &Path, name: &str, rev: &str) {
    git_crate_at(dir, name, OTHER_REPOSITORY, rev);
}

/// A catalog crate `name` in `dir` whose embsim dependencies are the
/// repository `url` at the revision `rev`.
fn git_crate_at(dir: &Path, name: &str, url: &str, rev: &str) {
    std::fs::create_dir_all(dir.join("src")).expect("the directory can be made");
    let dep = |package: &str| format!("{package} = {{ git = {url:?}, rev = {rev:?} }}\n");
    std::fs::write(
        dir.join("Cargo.toml"),
        format!(
            "[package]\nname = \"{name}\"\nversion = \"0.1.0\"\nedition = \"2021\"\n\n\
             [dependencies]\n{}{}",
            dep("embsim-boards"),
            dep("embsim-core")
        ),
    )
    .expect("the manifest is writable");
    std::fs::write(dir.join("src/lib.rs"), "").expect("the library is writable");
}

/// This embsim's release, `major.minor`.
fn release() -> String {
    let parts: Vec<&str> = env!("CARGO_PKG_VERSION").split('.').collect();
    format!("{}.{}", parts[0], parts[1])
}

#[rstest]
fn a_crate_in_no_workspace_builds_its_runner_beside_the_project() {
    behaviour!(Test {
        id: "cli.runner-lone-crate-target",
        covers: Some("cli/src/runner.rs#workspace_of"),
        given: "a project and a catalog crate beside it that is a Cargo package of its own, in \
                a directory no Cargo workspace covers, checked with the `embsim` tool and no \
                target directory set in the environment",
    });
    expect!(
        "builds-in-dot-embsim",
        "the runner is built in the target directory under the project's .embsim directory, \
         outside the catalog crate",
        "a crate that is its own Cargo root is in no workspace, and builds stay out of the \
         project's source tree"
    );
    let dir = outside("lone_crate");
    let (cargo, log) = logging_cargo(&dir);
    crate_on(
        &dir.join("sim/catalog"),
        &workspace().canonicalize().unwrap(),
    );
    std::fs::write(
        dir.join("rig.toml"),
        "[catalog]\ncrates = [\"sim/catalog\"]\n",
    )
    .expect("writable");
    // What made the runner build inside the crate: Cargo reads a lone
    // package as a workspace of its own, rooted at its directory.
    let metadata = Command::new(env!("CARGO"))
        .args(["metadata", "--no-deps", "--format-version", "1"])
        .current_dir(dir.join("sim/catalog"))
        .output()
        .expect("cargo runs");
    assert!(metadata.status.success(), "{}", stderr(&metadata));
    let metadata: serde_json::Value =
        serde_json::from_slice(&metadata.stdout).expect("cargo metadata is JSON");
    assert_eq!(
        metadata["workspace_root"].as_str().map(PathBuf::from),
        Some(dir.join("sim/catalog"))
    );
    let check = embsim_logged(&dir, &cargo, &["check", "rig.toml"]);
    assert!(!check.status.success(), "the stand-in Cargo builds nothing");
    assert_says(&read(&log), &["metadata --no-deps"]);
    assert_eq!(build_target(&log), dir.join(".embsim/target"));
    assert!(!dir.join("sim/catalog/target").exists());
}

#[rstest]
fn an_embsim_cargo_cannot_fetch_is_named_with_where_to_point_the_crates() {
    behaviour!(Test {
        id: "cli.runner-unfetched",
        covers: Some("cli/src/runner.rs#build_failure"),
        given: "a project whose catalog crate takes embsim from a git repository at a commit \
                the repository does not have, as a commit never pushed is, checked with the \
                `embsim` tool",
    });
    expect!(
        "names-the-source",
        "the runner does not build, and the tool says Cargo could not fetch embsim, naming the \
         repository, the commit and Cargo's own reason",
        "Cargo's retries and its chain of causes end in the line that says what went wrong"
    );
    expect!(
        "says-the-fix",
        "the error says to point the crates' embsim dependencies at a published release tag, \
         a commit a remote holds, or a checkout by path, and that `embsim new --catalog \
         --embsim` writes either"
    );
    let dir = outside("unfetched");
    let (cargo, _) = logging_cargo(&dir);
    let repository = dir.join("embsim.git");
    std::fs::create_dir_all(&repository).expect("writable");
    git(&repository, &["init", "-q"]);
    git(
        &repository,
        &["commit", "-q", "--allow-empty", "-m", "empty"],
    );
    let url = format!("file://{}", repository.display());
    let rev = "0123456789abcdef0123456789abcdef01234567";
    git_crate_at(&dir.join("sim/catalog"), "rig-catalog", &url, rev);
    std::fs::write(
        dir.join("rig.toml"),
        "[catalog]\ncrates = [\"sim/catalog\"]\n",
    )
    .expect("writable");
    let check = embsim_logged(&dir, &cargo, &["check", "rig.toml"]);
    assert!(!check.status.success());
    assert_says(
        &stderr(&check),
        &[
            &format!(
                "error: the runner for rig.toml did not build: Cargo could not fetch embsim from \
                 git {url} rev {rev} (embsim-"
            ),
            &format!("revspec '{rev}' not found"),
            "Cargo's errors are above. Point the catalog crates' embsim dependencies at a \
             source every machine can fetch — a published release tag",
            "or a commit a remote holds — or at an embsim checkout by path; `embsim new \
             --catalog DIR --embsim PATH|URL@REF` writes either",
        ],
    );
}

#[rstest]
fn a_runner_takes_embsim_from_where_its_crates_do() {
    behaviour!(Test {
        id: "cli.runner-embsim-from-crates",
        covers: Some("cli/src/runner.rs#one_embsim"),
        given: "a project whose one catalog crate takes embsim from a git repository at a \
                revision, checked with the `embsim` tool",
    });
    expect!(
        "same-source",
        "the runner's manifest takes the embsim command from that repository at that revision",
        "the project's own Cargo files say which embsim it builds against, wherever the tool \
         itself was installed from"
    );
    expect!(
        "said",
        "the line the tool prints before it builds names that repository and revision"
    );
    let dir = outside("git_source");
    let (cargo, _) = logging_cargo_with(&dir, false);
    git_crate(&dir.join("rig"), "rig-catalog", "0123abcd");
    std::fs::write(dir.join("p.toml"), "[catalog]\ncrates = [\"rig\"]\n").expect("writable");
    let check = embsim_logged(&dir, &cargo, &["check", "p.toml"]);
    assert!(!check.status.success(), "the stand-in Cargo builds nothing");
    let table: toml::Table =
        toml::from_str(&read(&runner_dir(&dir).join("Cargo.toml"))).expect("the manifest parses");
    let cli = &table["dependencies"]["embsim-cli"];
    assert_eq!(cli["git"].as_str(), Some(OTHER_REPOSITORY));
    assert_eq!(cli["rev"].as_str(), Some("0123abcd"));
    assert_says(
        &stderr(&check),
        &[&format!(
            "(rig-catalog, embsim from git {OTHER_REPOSITORY} rev 0123abcd)"
        )],
    );
}

#[rstest]
#[case::two_crates(false)]
#[case::one_crate_two_checkouts(true)]
fn a_crate_on_another_embsim_is_refused_before_any_build(#[case] one_crate: bool) {
    behaviour!(Test {
        id: "cli.runner-two-copies",
        covers: Some("cli/src/runner.rs#one_embsim"),
        given: "a project whose catalog crates take embsim from two checkouts: a second crate's \
                from another checkout than the first's, or the first crate's own embsim-core \
                from another checkout than its embsim-boards",
    });
    expect!(
        "refused-naming-both",
        "the tool refuses the project before it builds anything, naming the crate, the \
         embsim crate it takes from the other checkout and that checkout, and the checkout \
         the first crate's embsim-boards is in",
        "two copies of embsim in one runner would be two virtual clocks"
    );
    expect!(
        "says-the-fix",
        "the refusal says to point every catalog crate's embsim dependencies at one embsim"
    );
    let dir = outside(if one_crate {
        "copies_one"
    } else {
        "copies_two"
    });
    let (cargo, log) = logging_cargo(&dir);
    let copy = other_checkout(&dir.join("embsim-copy"));
    let real = workspace().canonicalize().unwrap();
    let (crates, culprit, name) = if one_crate {
        library_on(
            &dir.join("rig"),
            "rig-catalog",
            &[
                ("embsim-boards", &real.join("boards")),
                ("embsim-core", &copy.join("core")),
            ],
        );
        ("\"rig\"", "rig-catalog", "embsim-core")
    } else {
        crate_on(&dir.join("rig"), &real);
        library_on(
            &dir.join("more"),
            "more-catalog",
            &[("embsim-boards", &copy.join("boards"))],
        );
        ("\"rig\", \"more\"", "more-catalog", "embsim-boards")
    };
    std::fs::write(
        dir.join("p.toml"),
        format!("[catalog]\ncrates = [{crates}]\n"),
    )
    .expect("writable");
    let check = embsim_logged(&dir, &cargo, &["check", "p.toml"]);
    assert!(!check.status.success());
    assert_says(
        &stderr(&check),
        &[
            &format!(
                "catalog crate {culprit} takes {name} at {}, and the runner builds against \
                 the embsim rig-catalog takes embsim-boards at {}",
                copy.display(),
                real.display()
            ),
            "two would be two virtual clocks",
            "Point every catalog crate's embsim dependencies at one embsim",
        ],
    );
    assert!(
        !read(&log).lines().any(|line| line.starts_with("build ")),
        "nothing is built"
    );
}

#[rstest]
fn a_dependency_on_another_embsim_checkout_is_named_as_two_copies() {
    behaviour!(Test {
        id: "cli.runner-collision",
        covers: Some("cli/src/runner.rs#Cargo::collision"),
        given: "a catalog crate whose own embsim dependency is embsim's checkout, and which \
                depends on a crate of the project's taking embsim from another checkout, \
                checked with the `embsim` tool, the build failing",
    });
    expect!(
        "named-two-copies",
        "the error gives two copies of embsim as the one reason the build failed, quotes \
         Cargo's refusal of a second package claiming embsim-core's links name and the first \
         one's directory, and says to point every crate's embsim dependencies, the catalog \
         crates' and those they depend on, at the runner's embsim",
        "a second copy reached through a crate the catalog crate depends on is refused by \
         Cargo's resolver before anything compiles, and the tool names it"
    );
    let dir = outside("collision");
    let (cargo, log) = logging_cargo(&dir);
    let copy = other_checkout(&dir.join("embsim-copy"));
    let real = workspace().canonicalize().unwrap();
    library_on(
        &dir.join("helper"),
        "rig-helper",
        &[("embsim-board", &copy.join("board"))],
    );
    crate_on(&dir.join("rig"), &real);
    let manifest = dir.join("rig/Cargo.toml");
    let text = read(&manifest) + &format!("rig-helper = {{ path = {:?} }}\n", "../helper");
    std::fs::write(&manifest, text).expect("the manifest is writable");
    std::fs::write(dir.join("p.toml"), "[catalog]\ncrates = [\"rig\"]\n").expect("writable");

    let check = embsim_logged(&dir, &cargo, &["check", "p.toml"]);
    assert!(!check.status.success());
    assert!(
        read(&log).lines().any(|line| line.starts_with("build ")),
        "the check before the build reads only the catalog crate's own dependencies"
    );
    let said = stderr(&check);
    assert_says(
        &said,
        &[
            "the runner for p.toml did not build: it met two copies of embsim (package \
             `embsim-core` links to the native library `embsim-core`",
            "the first is package `embsim-core v",
            &format!(
                "Point the embsim dependencies of every catalog crate, and of the crates they \
                 depend on, at embsim at {}",
                real.display()
            ),
        ],
    );
    assert!(!said.contains("A catalog crate is a library"), "{said}");
}

#[rstest]
fn two_copies_of_embsim_do_not_resolve() {
    behaviour!(Test {
        id: "cli.one-embsim-links",
        covers: Some("core/Cargo.toml"),
        given: "a Cargo workspace whose one package depends on embsim's virtual-clock crate \
                and whose other depends on a second package of that name claiming the same \
                links name, at another version and directory",
    });
    expect!(
        "refused-at-resolution",
        "Cargo refuses the graph when it resolves it, before anything compiles, naming the \
         links name embsim-core that only one package may claim",
        "the virtual clock is process-global: a second copy of embsim would be a second clock"
    );
    let dir = outside("links");
    let core_manifest = read(&workspace().join("core/Cargo.toml"));
    let core: toml::Table = toml::from_str(&core_manifest).expect("the manifest parses");
    let links = core["package"]["links"]
        .as_str()
        .expect("embsim-core claims a links name");
    let copy = dir.join("copy");
    std::fs::create_dir_all(copy.join("src")).expect("the directory can be made");
    std::fs::write(
        copy.join("Cargo.toml"),
        format!(
            "[package]\nname = \"embsim-core\"\nversion = \"0.0.1\"\nedition = \"2021\"\n\
             links = {links:?}\n"
        ),
    )
    .expect("writable");
    std::fs::write(copy.join("build.rs"), "fn main() {}\n").expect("writable");
    std::fs::write(copy.join("src/lib.rs"), "").expect("writable");
    let real = workspace().canonicalize().unwrap();
    library_on(&dir.join("a"), "a", &[("embsim-core", &real.join("core"))]);
    library_on(&dir.join("b"), "b", &[("embsim-core", &copy)]);
    std::fs::write(
        dir.join("Cargo.toml"),
        "[workspace]\nmembers = [\"a\", \"b\"]\nresolver = \"2\"\n",
    )
    .expect("writable");
    let resolved = Command::new(env!("CARGO"))
        .args(["metadata", "--format-version", "1", "--offline"])
        .current_dir(&dir)
        .output()
        .expect("cargo runs");
    assert!(!resolved.status.success(), "two copies resolved");
    assert_says(
        &stderr(&resolved),
        &["links to the native library `embsim-core`"],
    );
}

#[rstest]
fn a_runner_spells_the_checkout_as_its_crates_do() {
    behaviour!(Test {
        id: "cli.runner-spelling",
        covers: Some("cli/src/runner.rs#one_embsim"),
        given: "a catalog crate whose embsim dependency reaches an embsim checkout through a \
                symlink, checked with the `embsim` tool",
    });
    expect!(
        "spelled-alike",
        "the runner's manifest takes the embsim command from the checkout by the same \
         symlinked path the crate uses",
        "Cargo takes two spellings of one directory for two copies of a package"
    );
    let dir = outside("spelling");
    let (cargo, _) = logging_cargo(&dir);
    let link = dir.join("embsim-link");
    std::os::unix::fs::symlink(workspace().canonicalize().unwrap(), &link)
        .expect("the link can be made");
    crate_on(&dir.join("rig"), &link);
    std::fs::write(dir.join("p.toml"), "[catalog]\ncrates = [\"rig\"]\n").expect("writable");
    let check = embsim_logged(&dir, &cargo, &["check", "p.toml"]);
    assert!(!check.status.success(), "the stand-in Cargo builds nothing");
    let manifest = read(&runner_dir(&dir).join("Cargo.toml"));
    let table: toml::Table = toml::from_str(&manifest).expect("the manifest parses");
    assert_eq!(
        table["dependencies"]["embsim-cli"]["path"].as_str(),
        Some(link.join("cli").to_str().expect("text")),
        "{manifest}"
    );
    assert_says(
        &stderr(&check),
        &[&format!("embsim at {})", link.display())],
    );
}

#[rstest]
#[case::kept(true)]
#[case::first(false)]
fn a_runners_lock_file_is_kept_beside_the_project(#[case] kept: bool) {
    behaviour!(Test {
        id: "cli.runner-lock",
        covers: Some("cli/src/runner.rs#lock"),
        given: "a project naming one catalog crate, checked with the `embsim` tool, with or \
                without the runner's embsim.lock beside it",
    });
    expect!(
        "locked-when-kept",
        "with embsim.lock there, the runner's lock is that file and the build is --locked; \
         without it, the build may lock afresh",
        "a lock kept beside the project, which can be committed, builds the same runner on \
         every machine"
    );
    let dir = outside(if kept { "lock_kept" } else { "lock_first" });
    let (cargo, log) = logging_cargo_with(&dir, false);
    crate_on(&dir.join("rig"), &workspace().canonicalize().unwrap());
    std::fs::write(dir.join("p.toml"), "[catalog]\ncrates = [\"rig\"]\n").expect("writable");
    // The runner is named after its crates alone, so the lock names the
    // same runner on every machine: one built here says which.
    let first = embsim_logged(&dir, &cargo, &["check", "p.toml"]);
    assert!(!first.status.success(), "the stand-in Cargo builds nothing");
    let package = runner_dir(&dir)
        .file_name()
        .expect("a name")
        .to_string_lossy()
        .replace("runner-", "embsim-runner-");
    let saved =
        format!("version = 4\n\n[[package]]\nname = \"{package}\"\nversion = \"0.0.0\"\n# kept\n");
    if kept {
        std::fs::write(dir.join("embsim.lock"), &saved).expect("writable");
    }
    std::fs::remove_file(&log).expect("the log is there");
    let check = embsim_logged(&dir, &cargo, &["check", "p.toml"]);
    assert!(!check.status.success(), "the stand-in Cargo builds nothing");
    let locked = build_line(&log).iter().any(|word| word == "--locked");
    assert_eq!(locked, kept, "{}", read(&log));
    if kept {
        assert_eq!(read(&runner_dir(&dir).join("Cargo.lock")), saved);
    }
}

#[rstest]
fn another_runners_lock_file_is_refused() {
    behaviour!(Test {
        id: "cli.runner-lock-other",
        covers: Some("cli/src/runner.rs#lock"),
        given: "a project beside an embsim.lock that locks the runner of other catalog crates",
    });
    expect!(
        "refused",
        "the tool refuses the project before it builds, naming the file and the runner it \
         locks, and saying the projects in one directory share it"
    );
    let dir = outside("lock_other");
    let (cargo, log) = logging_cargo_with(&dir, false);
    crate_on(&dir.join("rig"), &workspace().canonicalize().unwrap());
    std::fs::write(dir.join("p.toml"), "[catalog]\ncrates = [\"rig\"]\n").expect("writable");
    std::fs::write(
        dir.join("embsim.lock"),
        "version = 4\n\n[[package]]\nname = \"embsim-runner-00000000\"\nversion = \"0.0.0\"\n",
    )
    .expect("writable");
    let check = embsim_logged(&dir, &cargo, &["check", "p.toml"]);
    assert!(!check.status.success());
    assert_says(
        &stderr(&check),
        &[
            "embsim.lock is the lock of another runner (embsim-runner-00000000",
            "the projects in one directory that name catalog crates share its embsim.lock",
        ],
    );
    assert!(
        !read(&log).lines().any(|line| line.starts_with("build ")),
        "nothing is built"
    );
}

/// The project's own runner crate `name` in `dir`: a binary over the
/// catalog crate at `catalog`, the embsim command from the checkout `root`.
fn runner_on(dir: &Path, name: &str, catalog: (&str, &Path), root: &Path) {
    std::fs::create_dir_all(dir.join("src")).expect("the directory can be made");
    std::fs::write(
        dir.join("Cargo.toml"),
        format!(
            "[package]\nname = \"{name}\"\nversion = \"0.1.0\"\nedition = \"2021\"\n\n\
             [dependencies]\nembsim-cli = {{ path = {:?} }}\n{} = {{ path = {:?} }}\n",
            root.join("cli").display().to_string(),
            catalog.0,
            catalog.1.display().to_string()
        ),
    )
    .expect("the manifest is writable");
    std::fs::write(dir.join("src/main.rs"), "fn main() {}\n").expect("writable");
}

#[rstest]
#[case::locked(true)]
#[case::no_lock_yet(false)]
fn the_projects_own_runner_is_built_in_its_workspace(#[case] lock: bool) {
    behaviour!(Test {
        id: "cli.own-runner-build",
        covers: Some("cli/src/runner.rs#hand_over_own"),
        given: "a project whose catalog table names its own runner crate, a member of the \
                project's Cargo workspace with its catalog crate, checked with the `embsim` \
                tool, with or without the workspace's Cargo.lock",
    });
    expect!(
        "builds-the-crate",
        "the tool builds the runner crate's one binary by its own manifest, in the target \
         directory the workspace's Cargo settles, and writes no runner of its own",
        "a project that owns its runner owns its lock file, target directory and profiles"
    );
    expect!(
        "locked-with-the-workspace-lock",
        "the build is --locked when the workspace has a Cargo.lock, and may lock afresh when \
         it has none"
    );
    let dir = outside(if lock { "own_locked" } else { "own_unlocked" });
    let (cargo, log) = logging_cargo_with(&dir, false);
    let real = workspace().canonicalize().unwrap();
    crate_on(&dir.join("sim/catalog"), &real);
    runner_on(
        &dir.join("sim/runner"),
        "sim-runner",
        ("rig-catalog", &dir.join("sim/catalog")),
        &real,
    );
    std::fs::write(
        dir.join("Cargo.toml"),
        "[workspace]\nmembers = [\"sim/catalog\", \"sim/runner\"]\nresolver = \"2\"\n",
    )
    .expect("writable");
    if lock {
        std::fs::copy(real.join("Cargo.lock"), dir.join("Cargo.lock")).expect("the lock copies");
    }
    std::fs::write(
        dir.join("p.toml"),
        "[catalog]\ncrates = [\"sim/catalog\"]\nrunner = \"sim/runner\"\n",
    )
    .expect("writable");
    let check = embsim_logged(&dir, &cargo, &["check", "p.toml"]);
    assert!(!check.status.success(), "the stand-in Cargo builds nothing");
    assert_says(
        &stderr(&check),
        &["embsim: building the project's runner sim-runner for p.toml (rig-catalog)"],
    );
    let words = build_line(&log);
    let manifest = dir.join("sim/runner/Cargo.toml");
    assert!(
        words.windows(2).any(|pair| pair[0] == "--manifest-path"
            && Path::new(&pair[1]).canonicalize().ok() == manifest.canonicalize().ok()),
        "{words:?}"
    );
    assert!(
        words.windows(2).any(|pair| pair == ["--bin", "sim-runner"]),
        "{words:?}"
    );
    assert!(
        !words.iter().any(|word| word == "--target-dir"),
        "{words:?}"
    );
    assert_eq!(
        words.iter().any(|word| word == "--locked"),
        lock,
        "{words:?}"
    );
    assert!(
        !dir.join(".embsim").exists(),
        "the tool writes no runner of its own"
    );
}

#[rstest]
#[case::not_there(
    "not_there",
    "runner = \"sim/nowhere\"",
    "[catalog] runner = \"sim/nowhere\""
)]
#[case::no_binary(
    "no_binary",
    "runner = \"sim/catalog\"",
    "the package rig-catalog has no binary"
)]
fn a_runner_that_is_not_one_is_refused_saying_what_it_is(
    #[case] case: &str,
    #[case] line: &str,
    #[case] says: &str,
) {
    behaviour!(Test {
        id: "cli.own-runner-refused",
        covers: Some("cli/src/runner.rs#Cargo::own_runner"),
        given: "a project whose catalog table names as its runner a directory that is not \
                there, or a crate with no binary",
    });
    expect!(
        "refused",
        "the tool refuses it before any build, naming the runner as the file gives it and \
         saying a runner is a binary crate over the catalog crates"
    );
    // Each case its own directory: the cases run in parallel, and `outside`
    // empties the directory it is given.
    let dir = outside(&format!("own_refused_{case}"));
    let (cargo, log) = logging_cargo_with(&dir, false);
    crate_on(
        &dir.join("sim/catalog"),
        &workspace().canonicalize().unwrap(),
    );
    std::fs::write(
        dir.join("p.toml"),
        format!("[catalog]\ncrates = [\"sim/catalog\"]\n{line}\n"),
    )
    .expect("writable");
    let check = embsim_logged(&dir, &cargo, &["check", "p.toml"]);
    assert!(!check.status.success());
    assert_says(&stderr(&check), &[says, "runner"]);
    assert!(
        !std::fs::read_to_string(&log)
            .unwrap_or_default()
            .lines()
            .any(|line| line.starts_with("build ")),
        "nothing is built"
    );
}

#[rstest]
#[case::check(&["check", "p.toml", "--a-flag-only-a-newer-embsim-has"])]
#[case::survey(&["survey", "--project", "p.toml", "--a-flag-only-a-newer-embsim-has"])]
fn a_command_line_this_tool_cannot_parse_is_handed_to_the_projects_runner(#[case] args: &[&str]) {
    behaviour!(Test {
        id: "cli.hand-over-unparsed",
        covers: Some("cli/src/lib.rs#tool_main"),
        given: "the `embsim` tool given a flag it does not know, for a project with catalog \
                crates named after `check` or with --project",
    });
    expect!(
        "handed-over",
        "the tool goes on to the project's runner, which here refuses the missing crate the \
         project names",
        "a newer flag is the runner's embsim's to read: a project written for a newer embsim \
         reaches its runner through an older tool"
    );
    let dir = outside(&format!("unparsed_{}", args[0]));
    std::fs::write(
        dir.join("p.toml"),
        "[catalog]\ncrates = [\"sim/nowhere\"]\n",
    )
    .expect("writable");
    let output = embsim_in(&dir, args, &[]);
    assert!(!output.status.success());
    let said = stderr(&output);
    assert_says(&said, &["[catalog] crates: \"sim/nowhere\""]);
    assert!(!said.contains("unexpected argument"), "{said}");
}

#[rstest]
fn a_project_for_a_newer_embsim_says_so_when_its_line_does_not_parse() {
    behaviour!(Test {
        id: "cli.unparsed-newer-project",
        covers: Some("cli/src/lib.rs#tool_main"),
        given: "the `embsim` tool given a flag it does not know, for a project without catalog \
                crates written for a later embsim release",
    });
    expect!(
        "says-the-release",
        "the tool refuses it saying the project is written for that release and this is \
         another"
    );
    let dir = outside("unparsed_newer");
    std::fs::write(dir.join("p.toml"), "requires-embsim = \"99.0\"\n").expect("writable");
    let output = embsim_in(&dir, &["check", "p.toml", "--new-flag"], &[]);
    assert!(!output.status.success());
    assert_says(
        &stderr(&output),
        &[
            "this project is written for embsim 99.0",
            &format!("this is embsim {}", env!("CARGO_PKG_VERSION")),
        ],
    );
}

// ============================================================
// Built with Cargo: ignored here, run by CI's project-runner job
// ============================================================

/// The workspace's target directory, where these builds reuse what the
/// workspace built: Cargo's per-target scratch space is inside it.
fn target_dir() -> PathBuf {
    std::env::var_os("CARGO_TARGET_DIR").map_or_else(
        || {
            Path::new(env!("CARGO_TARGET_TMPDIR"))
                .parent()
                .expect("the scratch space sits in the target directory")
                .to_path_buf()
        },
        PathBuf::from,
    )
}

/// What `examples/custom-project` prints when it runs for 10 ms: the
/// blinker's pad high at the module's START instant (5.5 ms), each rising
/// edge reaching the counter the buffer's 12 ns later, one a millisecond.
const EXAMPLE_RUN: [&str; 9] = [
    "catalogs: embsim-boards, embsim-p2-qemu, embsim-qemu, custom-project-catalog",
    "EC32.U100: blinker: P0 high at 5.500000 ms, flipping every 0.500000 ms",
    "COUNTER: rising edge 1 on IN at 5.500012 ms",
    "COUNTER: rising edge 5 on IN at 9.500012 ms",
    "EC32.U100: core \"example-blinker\": started at 5.500000 ms",
    "EC32.U100: blinker: started at 5.500000 ms; P0 flipped 8 times, and drives it high",
    "COUNTER: 5 rising edges on IN, the first at 5.500012 ms and the last at 9.500012 ms, \
     every 1.000000 ms",
    "net BUF.OUT: Driven(High)",
    "ran 10.000000 ms of virtual time",
];

/// What a project started with `embsim new header.net --catalog
/// sim/catalog` adds to name the started crate's four kinds: its board, its
/// part on that board, and its source wired to the board's connector.
const STARTED_KINDS: &str = r#"
[[board]]
name = "BRD"
kind = "sim-board"

[[board.model]]
value = "SIM-SENSOR"
kind = "sim-sensor"

[[component]]
name = "SRC"
kind = "sim-source"
[component.options]
volts = 2.5
ohms = 100.0
at = "0.5ms"

[[wire]]
from = "SRC.OUT"
to = "BRD.J1.1"

[[wire]]
from = "BENCH.GND"
to = "BRD.J1.2"
volts = 0.0
"#;

/// The example's project file, and the directory it is in.
fn example() -> (PathBuf, PathBuf) {
    let dir = workspace().join("examples/custom-project");
    (dir.join("project.toml"), dir)
}

#[rstest]
#[ignore = "builds a runner with Cargo; CI's project-runner job runs it (--ignored)"]
fn the_example_project_checks_and_runs_through_the_tool_and_its_runner() {
    // examples/custom-project names its catalog crate in [catalog]. The real
    // `embsim` binary writes the runner beside it, builds it in the
    // workspace's target directory (the crate is a workspace member),
    // --locked against the embsim.lock the example commits, and execs it
    // for `check` and for `run`: the run prints what the crate's core, part
    // and instrument did, and what the runner is made of. A second build
    // with nothing changed is quiet; `--rebuild` compiles the crate again
    // and keeps the lock.
    let (project, dir) = example();
    let project = project.to_str().expect("text");
    let lock = dir.join("embsim.lock");
    let committed = std::fs::read_to_string(&lock).ok();
    let check = embsim_in(&dir, &["check", project], &[]);
    assert!(
        check.status.success(),
        "{}\n{}",
        stderr(&check),
        stdout(&check)
    );
    assert_says(
        &stderr(&check),
        &[
            "embsim: building the runner for",
            "(custom-project-catalog, embsim at",
        ],
    );
    assert_says(
        &stdout(&check),
        &[
            EXAMPLE_RUN[0],
            "catalog crate custom-project-catalog 0.1.0: ",
            ", git rev ",
            "ok: ",
        ],
    );
    match &committed {
        // Built --locked against it: the file is as it was.
        Some(text) => assert_eq!(&read(&lock), text),
        None => assert_says(&stderr(&check), &["embsim: wrote", "embsim.lock"]),
    }
    assert_eq!(
        read(&dir.join(".embsim/.gitignore")).lines().last(),
        Some("*")
    );

    let run = embsim_in(
        &dir,
        &["run", project, "--for", "10ms", "--net", "BUF.OUT"],
        &[],
    );
    assert!(run.status.success(), "{}\n{}", stderr(&run), stdout(&run));
    assert_says(&stdout(&run), &EXAMPLE_RUN);
    assert!(
        !stderr(&run).contains("Compiling"),
        "a runner built before is checked quietly:\n{}",
        stderr(&run)
    );

    let kept = read(&lock);
    let rebuilt = embsim_in(&dir, &["check", "--rebuild", project], &[]);
    assert!(rebuilt.status.success(), "{}", stderr(&rebuilt));
    assert_says(&stderr(&rebuilt), &["Compiling custom-project-catalog"]);
    assert_eq!(read(&lock), kept, "--rebuild keeps the lock");
}

#[rstest]
#[ignore = "builds a runner with Cargo; CI's project-runner job runs it (--ignored)"]
fn a_started_crate_builds_into_a_runner_and_runs_its_kinds() {
    // `embsim new <netlist> --catalog` in a directory of its own: the
    // copied crate, outside any workspace, compiles into a runner and its
    // four kinds run in one project, as `template.rs` runs the template.
    let dir = scratch("started_crate");
    std::fs::copy(
        workspace().join("boards/projects/header.net"),
        dir.join("header.net"),
    )
    .expect("the netlist copies");
    let new = embsim_in(
        &dir,
        &[
            "new",
            "header.net",
            "-o",
            "rig.toml",
            "--catalog",
            "sim/catalog",
        ],
        &[],
    );
    assert!(new.status.success(), "{}", stderr(&new));
    let mut text = read(&dir.join("rig.toml"));
    text.push_str(STARTED_KINDS);
    std::fs::write(dir.join("rig.toml"), text).expect("writable");
    let target = target_dir();
    let target = target.to_str().expect("text");
    let run = embsim_in(
        &dir,
        &["run", "rig.toml", "--for", "1ms"],
        &[("CARGO_TARGET_DIR", target)],
    );
    assert!(run.status.success(), "{}\n{}", stderr(&run), stdout(&run));
    assert_says(
        &stdout(&run),
        &[
            "catalogs: embsim-boards, embsim-p2-qemu, embsim-qemu, sim-catalog",
            "BRD.U1: pin 1 read 2.5 V from 0.500000 ms",
            "SRC: drove OUT at 2.5 V behind 100 Ω from 0.500000 ms",
        ],
    );
    // The first build kept its lock beside the project; the next builds
    // --locked against it, and says nothing of it.
    assert_says(&stderr(&run), &["embsim: wrote", "embsim.lock"]);
    let lock = read(&dir.join("embsim.lock"));
    let again = embsim_in(
        &dir,
        &["check", "rig.toml"],
        &[("CARGO_TARGET_DIR", target)],
    );
    assert!(again.status.success(), "{}", stderr(&again));
    assert!(
        !stderr(&again).contains("embsim: wrote"),
        "{}",
        stderr(&again)
    );
    assert_eq!(read(&dir.join("embsim.lock")), lock);
}

#[rstest]
#[ignore = "builds a runner with Cargo; CI's project-runner job runs it (--ignored)"]
fn the_projects_own_runner_builds_in_its_workspace_and_runs() {
    // A Cargo workspace of the project's own, as MaD's SIL is: `embsim new
    // <netlist> --catalog sim/catalog --own-runner` writes the catalog crate
    // and the runner crate, members of it, and names the runner in the
    // project. The tool builds the runner by its own manifest, writes the
    // workspace's Cargo.lock the first time and says to commit it, runs
    // the project's kinds through it, and says what the runner is made of.
    let dir = scratch("own_runner_built");
    std::fs::write(
        dir.join("Cargo.toml"),
        "[workspace]\nmembers = [\"sim/catalog\", \"sim/runner\"]\nresolver = \"2\"\n",
    )
    .expect("writable");
    std::fs::copy(
        workspace().join("boards/projects/header.net"),
        dir.join("header.net"),
    )
    .expect("the netlist copies");
    let new = embsim_in(
        &dir,
        &[
            "new",
            "header.net",
            "-o",
            "rig.toml",
            "--catalog",
            "sim/catalog",
            "--own-runner",
        ],
        &[],
    );
    assert!(new.status.success(), "{}", stderr(&new));
    let mut text = read(&dir.join("rig.toml"));
    text.push_str(STARTED_KINDS);
    std::fs::write(dir.join("rig.toml"), text).expect("writable");
    let target = target_dir();
    let target = target.to_str().expect("text");
    let run = embsim_in(
        &dir,
        &["run", "rig.toml", "--for", "1ms"],
        &[("CARGO_TARGET_DIR", target)],
    );
    assert!(run.status.success(), "{}\n{}", stderr(&run), stdout(&run));
    assert_says(
        &stderr(&run),
        &[
            "embsim: building the project's runner sim-runner for rig.toml (sim-catalog)",
            "Cargo.lock: commit it",
        ],
    );
    assert_says(
        &stdout(&run),
        &[
            "catalogs: embsim-boards, embsim-p2-qemu, embsim-qemu, sim-catalog",
            "runner crate sim-runner 0.1.0: ",
            "catalog crate sim-catalog 0.1.0: ",
            "BRD.U1: pin 1 read 2.5 V from 0.500000 ms",
        ],
    );
    assert!(dir.join("Cargo.lock").exists());
    assert!(
        !dir.join(".embsim").exists(),
        "the tool writes no runner of its own"
    );
    let again = embsim_in(
        &dir,
        &["check", "rig.toml"],
        &[("CARGO_TARGET_DIR", target)],
    );
    assert!(again.status.success(), "{}", stderr(&again));
    assert!(!stderr(&again).contains("commit it"), "{}", stderr(&again));
}

#[rstest]
#[ignore = "builds a runner with Cargo; CI's project-runner job runs it (--ignored)"]
fn a_runner_crate_of_its_own_names_the_commit_that_holds_it() {
    // A project in a git repository of its own and in no Cargo workspace,
    // its runner crate a workspace of its own (`--own-runner`), all of it
    // committed, built once, its new Cargo.lock committed, and a build's
    // files left in the runner's target directory: the runner's line names
    // the commit and no changes, because the crate's .gitignore keeps its
    // target directory out of git.
    let dir = outside("own_runner_committed");
    git(&dir, &["init", "-q"]);
    std::fs::copy(
        workspace().join("boards/projects/header.net"),
        dir.join("header.net"),
    )
    .expect("the netlist copies");
    let checkout = workspace().canonicalize().unwrap();
    let new = embsim_in(
        &dir,
        &[
            "new",
            "header.net",
            "-o",
            "rig.toml",
            "--catalog",
            "rig/catalog",
            "--own-runner",
            "--embsim",
            checkout.to_str().expect("text"),
        ],
        &[],
    );
    assert!(new.status.success(), "{}", stderr(&new));
    let mut text = read(&dir.join("rig.toml"));
    // The started crate names its kinds after the catalog (rig-…).
    text.push_str(
        &STARTED_KINDS
            .replace("sim-", "rig-")
            .replace("SIM-", "RIG-"),
    );
    std::fs::write(dir.join("rig.toml"), text).expect("writable");
    git(&dir, &["add", "-A"]);
    git(&dir, &["commit", "-qm", "the rig"]);
    let target = target_dir();
    let target = target.to_str().expect("text");
    let first = embsim_in(
        &dir,
        &["check", "rig.toml"],
        &[("CARGO_TARGET_DIR", target)],
    );
    assert!(first.status.success(), "{}", stderr(&first));
    assert_says(&stderr(&first), &["rig/runner/Cargo.lock: commit it"]);
    git(&dir, &["add", "-A"]);
    git(&dir, &["commit", "-qm", "the runner's lock"]);
    let rev = git(&dir, &["rev-parse", "HEAD"]);
    let runner = dir.join("rig/runner");
    std::fs::create_dir_all(runner.join("target/release")).expect("writable");
    std::fs::write(runner.join("target/release/rig-runner"), "built").expect("writable");
    let again = embsim_in(
        &dir,
        &["check", "rig.toml"],
        &[("CARGO_TARGET_DIR", target)],
    );
    assert!(again.status.success(), "{}", stderr(&again));
    let out = stdout(&again);
    let line = out
        .lines()
        .find(|line| line.contains("runner crate rig-runner 0.1.0: "))
        .unwrap_or_else(|| panic!("no runner line in:\n{out}"));
    assert!(
        line.ends_with(&format!("{}, git rev {}", runner.display(), &rev[..12])),
        "{line}"
    );
}

#[rstest]
#[ignore = "builds a runner with Cargo; CI's project-runner job runs it (--ignored)"]
fn a_catalog_crate_that_does_not_compile_shows_the_compilers_errors() {
    // The started crate with a registration function of the wrong shape:
    // the build fails, rustc's error is on standard error as Cargo renders
    // it, and the tool's own line says which runner did not build and what
    // a catalog crate exports.
    let dir = scratch("broken_crate");
    std::fs::write(dir.join("p.toml"), "[catalog]\ncrates = [\"broken\"]\n").expect("writable");
    let new = embsim_in(&dir, &["new", "--catalog", "broken"], &[]);
    assert!(new.status.success(), "{}", stderr(&new));
    let lib = dir.join("broken/src/lib.rs");
    let text = read(&lib).replace(
        "pub fn register(set: &mut CatalogSet) -> Result<(), ProjectError> {",
        "pub fn register(set: &mut CatalogSet, _extra: u8) -> Result<(), ProjectError> {",
    );
    std::fs::write(&lib, text).expect("writable");
    let target = target_dir();
    let check = embsim_in(
        &dir,
        &["check", "p.toml"],
        &[("CARGO_TARGET_DIR", target.to_str().expect("text"))],
    );
    assert!(!check.status.success());
    assert_says(
        &stderr(&check),
        &[
            "error[E0308]: mismatched types",
            "the runner for p.toml did not build (catalog crates broken-catalog; embsim at",
            "`pub fn register(set: &mut CatalogSet) -> Result<(), ProjectError>`",
        ],
    );
}
