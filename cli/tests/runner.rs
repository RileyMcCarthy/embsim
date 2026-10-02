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

/// A directory of the test's own, emptied.
fn scratch(test: &str) -> PathBuf {
    let dir = Path::new(env!("CARGO_TARGET_TMPDIR"))
        .join("runner")
        .join(test);
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
    String::from_utf8_lossy(&output.stdout).into_owned()
}

fn stderr(output: &Output) -> String {
    String::from_utf8_lossy(&output.stderr).into_owned()
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
        "the directory gets a Cargo package named rig-catalog whose embsim dependencies are \
         the checkout the command was built from, and a library that is the started catalog \
         with every kind named rig-board, rig-sensor, rig-core and rig-source",
        "a kind starts with its project's name, so no kind embsim ships ever meets it"
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
         one binary depending on the embsim command from the checkout the tool was built \
         from and on the crate by its path, as a workspace of its own",
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
            "name: \"yourproject-catalog\"",
            "register: yourproject_catalog::register",
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
    let crates = [CatalogCrate {
        name: "yourproject-catalog",
        dir: TEMPLATE_DIR,
        register: yourproject_catalog::register,
    }];
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
    let crates = [CatalogCrate {
        name: "yourproject-catalog",
        dir: TEMPLATE_DIR,
        register: yourproject_catalog::register,
    }];
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
// target directory, the checkout the runner's manifest spells, the
// refusals — and what it says of a build that failed, without paying for a
// build of embsim.

/// A directory of the test's own outside every Cargo workspace (embsim's
/// target directory is inside embsim's), emptied.
fn outside(test: &str) -> PathBuf {
    let dir = std::env::temp_dir()
        .join(format!("embsim-runner-{}", std::process::id()))
        .join(test);
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).expect("the directory can be made");
    dir.canonicalize().expect("the directory is there")
}

/// The stand-in Cargo in `dir`, and the log it writes.
fn logging_cargo(dir: &Path) -> (PathBuf, PathBuf) {
    use std::os::unix::fs::PermissionsExt;
    let script = dir.join("cargo-log.sh");
    let log = dir.join("cargo.log");
    std::fs::write(
        &script,
        format!(
            "#!/bin/sh\nprintf '%s\\n' \"$*\" >> {log:?}\ncase \"$1\" in\n  metadata) exec \
             {cargo:?} \"$@\" ;;\nesac\nexit 101\n",
            cargo = env!("CARGO"),
        ),
    )
    .expect("the script is writable");
    std::fs::set_permissions(&script, std::fs::Permissions::from_mode(0o755))
        .expect("the script is executable");
    (script, log)
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

/// The `--target-dir` the build in `log` was given.
fn build_target(log: &Path) -> PathBuf {
    let text = read(log);
    let build = text
        .lines()
        .find(|line| line.starts_with("build "))
        .unwrap_or_else(|| panic!("no build in:\n{text}"));
    let words: Vec<&str> = build.split(' ').collect();
    let at = words
        .iter()
        .position(|word| *word == "--target-dir")
        .expect("the build names a target directory");
    PathBuf::from(words[at + 1])
}

/// A stand-in embsim checkout in `dir`: what the tool reads to know a
/// checkout (`cli` is `embsim-cli`, `p2-qemu` is there), and an
/// `embsim-board` package a crate can depend on.
fn other_checkout(dir: &Path) -> PathBuf {
    for (crate_dir, package) in [
        ("cli", "embsim-cli"),
        ("p2-qemu", "embsim-p2-qemu"),
        ("board", "embsim-board"),
    ] {
        let at = dir.join(crate_dir);
        std::fs::create_dir_all(at.join("src")).expect("the directory can be made");
        std::fs::write(
            at.join("Cargo.toml"),
            format!("[package]\nname = \"{package}\"\nversion = \"0.1.0\"\nedition = \"2021\"\n"),
        )
        .expect("the manifest is writable");
        std::fs::write(at.join("src/lib.rs"), "").expect("the library is writable");
    }
    dir.to_path_buf()
}

/// A catalog crate `rig-catalog` in `dir` whose one embsim dependency,
/// `embsim-board`, is at `board`.
fn crate_on(dir: &Path, board: &Path) {
    library_on(dir, "rig-catalog", board);
}

/// A library package `name` in `dir` depending on `embsim-board` at `board`.
fn library_on(dir: &Path, name: &str, board: &Path) {
    std::fs::create_dir_all(dir.join("src")).expect("the directory can be made");
    std::fs::write(
        dir.join("Cargo.toml"),
        format!(
            "[package]\nname = \"{name}\"\nversion = \"0.1.0\"\nedition = \"2021\"\n\n\
             [dependencies]\nembsim-board = {{ path = {:?} }}\n",
            board.display().to_string()
        ),
    )
    .expect("the manifest is writable");
    std::fs::write(dir.join("src/lib.rs"), "").expect("the library is writable");
}

#[rstest]
fn a_started_crate_in_no_workspace_builds_its_runner_beside_the_project() {
    behaviour!(Test {
        id: "cli.runner-lone-crate-target",
        covers: Some("cli/src/runner.rs#workspace_of"),
        given: "a project and the catalog crate `embsim new --catalog` started beside it, in a \
                directory no Cargo workspace covers, checked with the `embsim` tool and no \
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
    std::fs::write(dir.join("rig.toml"), "# The rig.\n").expect("writable");
    let new = embsim_logged(
        &dir,
        &cargo,
        &["new", "--catalog", "sim/catalog", "--add-to", "rig.toml"],
    );
    assert!(new.status.success(), "{}", stderr(&new));
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
#[case::the_tools_own(false)]
#[case::named_in_the_project(true)]
fn a_crate_on_another_embsim_checkout_is_refused_before_any_build(#[case] named: bool) {
    behaviour!(Test {
        id: "cli.runner-two-copies",
        covers: Some("cli/src/runner.rs#one_embsim"),
        given: "a project whose catalog crate depends on embsim-board from one embsim \
                checkout while the runner would build against another: the checkout the tool \
                was built from, or one the project names with [catalog] embsim",
    });
    expect!(
        "refused-naming-both",
        "the tool refuses the project before it builds anything, naming the crate, the \
         directory its embsim dependency is in, and the checkout the runner builds against",
        "two copies of embsim in one runner would be two virtual clocks"
    );
    expect!(
        "says-the-fix",
        "the refusal says to point the crate's embsim dependencies at the runner's checkout, \
         or, when the crate's is a checkout, to name it with [catalog] embsim"
    );
    let dir = outside(if named { "copies_named" } else { "copies_own" });
    let (cargo, log) = logging_cargo(&dir);
    let copy = other_checkout(&dir.join("embsim-copy"));
    let real = workspace().canonicalize().unwrap();
    let (board, embsim_line, fix) = if named {
        (
            real.join("board"),
            "embsim = \"embsim-copy\"\n",
            format!(
                "Point the crate's embsim dependencies at {}",
                copy.display()
            ),
        )
    } else {
        (
            copy.join("board"),
            "",
            "[catalog] embsim = \"embsim-copy\"".to_string(),
        )
    };
    crate_on(&dir.join("rig"), &board);
    std::fs::write(
        dir.join("p.toml"),
        format!("[catalog]\ncrates = [\"rig\"]\n{embsim_line}"),
    )
    .expect("writable");
    let check = embsim_logged(&dir, &cargo, &["check", "p.toml"]);
    assert!(!check.status.success());
    let runner_checkout = if named { &copy } else { &real };
    assert_says(
        &stderr(&check),
        &[
            &format!(
                "catalog crate rig-catalog takes embsim-board from {}, which is not in the \
                 embsim checkout the runner builds against, {}",
                board.display(),
                runner_checkout.display()
            ),
            "two would be two virtual clocks",
            &fix,
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
        given: "a catalog crate whose own embsim dependency is in the checkout the tool was \
                built from, and which depends on a crate of the project's taking embsim-board \
                from another checkout, checked with the `embsim` tool, the build failing",
    });
    expect!(
        "named-two-copies",
        "the error gives two copies of embsim as the one reason the build failed, quotes \
         Cargo's package collision naming both embsim-board directories, and says to point \
         every crate's embsim dependencies, the catalog crates' and those they depend on, at \
         the checkout",
        "a second copy reached through a crate the catalog crate depends on is refused by \
         Cargo's resolver before anything compiles, and the tool names it"
    );
    let dir = outside("collision");
    let (cargo, log) = logging_cargo(&dir);
    let copy = other_checkout(&dir.join("embsim-copy"));
    let real = workspace().canonicalize().unwrap();
    library_on(&dir.join("helper"), "rig-helper", &copy.join("board"));
    crate_on(&dir.join("rig"), &real.join("board"));
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
             collision in the lockfile: packages embsim-board v0.1.0 (",
            &format!("{})", real.join("board").display()),
            &format!("{})", copy.join("board").display()),
            &format!(
                "Point the embsim dependencies of every catalog crate, and of the crates they \
                 depend on, at {}",
                real.display()
            ),
        ],
    );
    assert!(!said.contains("A catalog crate is a library"), "{said}");
}

#[rstest]
fn a_runner_spells_the_checkout_as_its_crates_do() {
    behaviour!(Test {
        id: "cli.runner-spelling",
        covers: Some("cli/src/runner.rs#one_embsim"),
        given: "a catalog crate whose embsim dependency reaches the checkout the tool was \
                built from through a symlink, checked with the `embsim` tool",
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
    crate_on(&dir.join("rig"), &link.join("board"));
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
    "catalogs: embsim-boards, embsim-p2-qemu, custom-project-catalog",
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
    // workspace's target directory (the crate is a workspace member) and
    // execs it for `check` and for `run`: the run prints what the crate's
    // core, part and instrument did. A second build with nothing changed
    // is quiet; `--rebuild` compiles the crate again.
    let (project, dir) = example();
    let project = project.to_str().expect("text");
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
    assert_says(&stdout(&check), &[EXAMPLE_RUN[0], "ok: "]);
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

    let rebuilt = embsim_in(&dir, &["check", "--rebuild", project], &[]);
    assert!(rebuilt.status.success(), "{}", stderr(&rebuilt));
    assert_says(&stderr(&rebuilt), &["Compiling custom-project-catalog"]);
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
    text.push_str(
        r#"
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
"#,
    );
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
            "catalogs: embsim-boards, embsim-p2-qemu, sim-catalog",
            "BRD.U1: pin 1 read 2.5 V from 0.500000 ms",
            "SRC: drove OUT at 2.5 V behind 100 Ω from 0.500000 ms",
        ],
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
