//! The `embsim` command, as a library: a system simulated from its boards'
//! netlists, written down as a project file, over a set of catalogs.
//!
//! A project names boards (a KiCad netlist, or a board a catalog ships),
//! the model each part takes, the bench components, the wires between
//! connectors, and the scenario (`embsim_board::Project`). Four subcommands
//! take a netlist to a running system:
//!
//! - `embsim survey <netlist>` — the checklist: what the catalogs populate
//!   by themselves, what needs a model and which kinds could be it, and
//!   every connector pin a wire may use. `embsim survey --kind p2-ec32mb`
//!   surveys a board a catalog ships the same way. With `--project FILE`
//!   the project's own kinds are among them (below).
//! - `embsim new <netlist>` — a starter project answering that checklist as
//!   far as the catalogs can, with a commented stub for every part left.
//! - `embsim check <project>` — loads, surveys and builds the system with
//!   time held, and prints what the build found; any refusal is an error
//!   with the text that says what to fix.
//! - `embsim run <project>` — starts the system and runs it in virtual
//!   time, until `--for` elapses or the run is interrupted (Ctrl-C), then
//!   prints its summary.
//!
//! And one for the P2's QEMU core, which runs in a program of its own:
//! `embsim qemu install` builds that `qemu-system-p2` from the target this
//! embsim carries and installs it where the core looks; `embsim qemu path`
//! says which one a run would start and whether it is the right one
//! (`embsim_p2_qemu`, "Where the CPU runs").
//!
//! Every kind a project names comes from a [`CatalogSet`]. A project with
//! kinds of its own — a model, a board, an instruction-set simulator as the
//! P2's core, a bench part — writes them in a catalog crate and names it in
//! its file (`[catalog] crates = ["sim/catalog"]`). The `embsim` binary is
//! [`tool_main`]: a project without `[catalog]` runs in it, over
//! [`shipped`] (the standard catalog, and QEMU as a P2 core); a project
//! with `[catalog]` is handed to a **runner** — the same command over a set
//! the project's catalogs joined ([`runner_main`]) — which the tool builds
//! with Cargo and `exec`s: the project's own runner crate when the file
//! names one (`[catalog] runner`), else one the tool writes beside the
//! project, against the embsim the catalog crates themselves depend on
//! (`PROJECTS.md` §10, "The runner"). `embsim new --catalog DIR` starts a
//! catalog crate, and `--own-runner` a runner crate beside it. `survey` and
//! `new` take `--project FILE` to run in that project's runner the same
//! way, so a part's candidates, `--kind`, and the starter project's
//! `[catalog]` are the project's.
//!
//! The tool reads two things of a project before anything else, and
//! nothing else before it chooses who runs it: its `[catalog]` and the
//! embsim release it is written for, `requires-embsim` (`ProjectHead`),
//! each read leniently. So a project written for a newer embsim, even one
//! whose command line this tool cannot parse, still reaches its runner,
//! and a project without catalog crates that a newer embsim wrote says so.
//!
//! Every `check` and `run` prints what its binary is made of under the
//! project line — embsim's version and git revision and where its sources
//! were, the compiler, target and profile, each catalog crate's version and
//! revision — and `--version` says the same.
//!
//! A project's own binary is the same command over a set its catalogs
//! joined, in ten lines, for a project that would rather own its binary
//! than have the tool build one:
//!
//! ```no_run
//! use std::process::ExitCode;
//!
//! use embsim_board::ProjectError;
//! use embsim_boards::catalog::CatalogSet;
//!
//! /// The project's catalog crate's registration function (here a stand-in
//! /// that adds nothing): its catalogs and cores join the set.
//! fn register(_set: &mut CatalogSet) -> Result<(), ProjectError> {
//!     Ok(())
//! }
//!
//! fn main() -> ExitCode {
//!     let mut set = embsim_cli::shipped();
//!     match register(&mut set) {
//!         Ok(()) => embsim_cli::main_with(set),
//!         Err(err) => {
//!             eprintln!("error: {err}");
//!             ExitCode::FAILURE
//!         }
//!     }
//! }
//! ```
//!
//! [`run`] is the same command with its arguments and its output handed in,
//! for a test or a program that runs it in place. Every binary over
//! [`main_with`] or [`runner_main`] runs each project itself; only the
//! `embsim` tool hands one over.
//!
//! Every board is built by the one pipeline (`DESIGN.md` rule 1): the
//! project's netlist and registry through `Board::from_netlist`, surveyed
//! first with the same registry, and every kind — whichever catalog it
//! comes from — seated only on a part that is what it says.

use std::ffi::{OsStr, OsString};
use std::io::Write;
use std::path::{Path, PathBuf};
use std::process::ExitCode;

use clap::{ArgGroup, CommandFactory, FromArgMatches, Parser, Subcommand};

use embsim_board::{CatalogTable, ProjectError, ProjectHead};
pub use embsim_boards::catalog::CatalogSet;

mod checklist;
mod live;
mod provenance;
mod qemu;
mod runner;
mod scaffold;
mod signals;

pub use provenance::PROVENANCE_ENV;
pub use runner::LOCK_FILE;

/// Simulate boards from their netlists: survey a netlist, start a project
/// for it, check the project, run it.
#[derive(Debug, Parser)]
#[command(name = "embsim", version)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Debug, Subcommand)]
enum Command {
    /// The checklist for a netlist, or for a board kind a catalog ships:
    /// the parts the catalogs populate, the parts that need a model and the
    /// kinds that could be them, and every connector pin with its name and
    /// net.
    #[command(group(ArgGroup::new("board").required(true).args(["netlist", "kind"])))]
    Survey {
        /// A KiCad netlist export (`kicad-cli sch export netlist`).
        netlist: Option<PathBuf>,
        /// A board kind a catalog ships (`p2-ec32mb`), surveyed with the
        /// registry a project builds it with.
        #[arg(long)]
        kind: Option<String>,
        /// Survey with the kinds of this project's `[catalog]` crates among
        /// the catalogs': the survey runs in the project's runner, as
        /// `check` does.
        #[arg(long, value_name = "PROJECT")]
        project: Option<PathBuf>,
    },
    /// Write a starter project for a netlist: the board, the pin tables the
    /// catalogs can choose, a commented stub for every part that needs a
    /// model, and the connectors' pins to wire. With `--catalog`, start a
    /// catalog crate for the project's own kinds as well, or alone.
    New {
        /// A KiCad netlist export. Without it, `--catalog` starts a catalog
        /// crate alone.
        #[arg(required_unless_present = "catalog")]
        netlist: Option<PathBuf>,
        /// The board's name in the project, the first word of every
        /// endpoint on it. Defaults to the netlist's file name.
        #[arg(long)]
        name: Option<String>,
        /// Where to write the project; its netlist path is relative to this
        /// file. Without it the project goes to standard output, its
        /// netlist path relative to the current directory.
        #[arg(short, long, value_name = "PROJECT")]
        output: Option<PathBuf>,
        /// Replace the output file if it exists.
        #[arg(long)]
        force: bool,
        /// Start a catalog crate in this directory — a `Cargo.toml` and a
        /// `src/lib.rs` with the registration function and one commented
        /// example of each sort of kind — and name it in the project's
        /// `[catalog]`: the starter project's, or `--add-to`'s.
        #[arg(long, value_name = "DIR")]
        catalog: Option<PathBuf>,
        /// With `--catalog`: start the project's own runner as well, a
        /// binary crate in `DIR` (by default beside the catalog crate,
        /// `sim/catalog` → `sim/runner`) whose main is the command over the
        /// crate, and name it in the project's `[catalog] runner`. The tool
        /// then builds that crate, with the workspace's lock file, instead
        /// of writing a runner of its own.
        #[arg(long = "own-runner", value_name = "DIR", requires = "catalog", num_args = 0..=1)]
        own_runner: Option<Option<PathBuf>>,
        /// With `--catalog` and no netlist: the existing project whose
        /// `[catalog] crates` the new crate joins (the file is edited in
        /// place, its comments kept).
        #[arg(
            long = "add-to",
            value_name = "PROJECT",
            requires = "catalog",
            conflicts_with_all = ["netlist", "output", "name", "force"]
        )]
        add_to: Option<PathBuf>,
        /// Start the project with the kinds of this project's `[catalog]`
        /// crates among the catalogs': `new` runs in the project's runner,
        /// and the starter project names the same `[catalog]`, its paths
        /// rewritten to reach the crates from where it is written.
        #[arg(
            long,
            value_name = "PROJECT",
            requires = "netlist",
            conflicts_with_all = ["catalog", "add_to"]
        )]
        project: Option<PathBuf>,
    },
    /// Load a project, survey its boards and build its system with time
    /// held; print what the build found. Exits non-zero, with the reason,
    /// when anything is refused.
    Check {
        /// The project file.
        project: PathBuf,
        /// For a project with `[catalog]`: write the runner afresh and
        /// rebuild it and the project's catalog crates, for a change Cargo
        /// cannot see. A project without `[catalog]` has nothing to
        /// rebuild.
        #[arg(long)]
        rebuild: bool,
    },
    /// Start a project's system and run it in virtual time, printing
    /// findings and what its parts report as the run reaches them, and a
    /// summary when it ends.
    Run {
        /// The project file.
        project: PathBuf,
        /// How long to run, in virtual time: `20ms`, `1.5s`, `250us`,
        /// `100ns`. Without it the run lasts until it is interrupted
        /// (Ctrl-C); either way an interrupt ends it with its summary.
        #[arg(long = "for", value_name = "DURATION", value_parser = embsim_board::parse_duration)]
        duration: Option<u64>,
        /// A net to read when the run ends, as `Board.Net` (repeat for
        /// more).
        #[arg(long = "net", value_name = "BOARD.NET")]
        nets: Vec<String>,
        /// Where the project's `host-serial` PTY is reached: `PATH` for its
        /// one host, `NAME=PATH` for the component `NAME` (repeat for
        /// more). Relative to the current directory.
        #[arg(long = "pty", value_name = "[NAME=]PATH")]
        ptys: Vec<String>,
        /// As `check --rebuild`.
        #[arg(long)]
        rebuild: bool,
    },
    /// The P2's QEMU core runs in a program of its own, `qemu-system-p2`:
    /// install it, or say which one a run would start.
    Qemu {
        #[command(subcommand)]
        command: QemuCommand,
    },
}

/// `embsim qemu …`.
#[derive(Debug, Subcommand)]
enum QemuCommand {
    /// Build `qemu-system-p2` from the P2 target this embsim carries —
    /// fetching QEMU at its pinned release, staging the target, a minimal
    /// configure, ninja — and install it into `~/.embsim/qemu/<target>/`,
    /// where the core looks. Needs git, a C compiler, ninja, pkg-config,
    /// glib and python3. The program is QEMU, GPL-2.0, installed with its
    /// licence and source; embsim runs it and links none of it.
    Install {
        /// Install into this directory instead (then point
        /// `EMBSIM_QEMU_SYSTEM_P2` at the program, or put it on `PATH`).
        #[arg(long, value_name = "DIR")]
        prefix: Option<PathBuf>,
        /// Build here instead of under the system's temporary directory. An
        /// install that stopped picks up where it was.
        #[arg(long, value_name = "DIR")]
        build_dir: Option<PathBuf>,
        /// Keep the build directory afterwards.
        #[arg(long)]
        keep_build: bool,
        /// Fetch QEMU from this repository (a mirror, a local clone); the
        /// pinned commit is checked whatever the source.
        #[arg(long, value_name = "URL")]
        qemu_git: Option<String>,
        /// Build jobs (ninja's default without it).
        #[arg(long, short)]
        jobs: Option<usize>,
        /// Build and install even when the matching program is installed.
        #[arg(long)]
        force: bool,
        /// Say what it would do — the target, the QEMU release, the build
        /// and install directories, the configure line — and do nothing.
        #[arg(long)]
        dry_run: bool,
    },
    /// Which `qemu-system-p2` a run would start (`EMBSIM_QEMU_SYSTEM_P2`,
    /// then `PATH`, then the installed one), what it says it is, and
    /// whether that is what this embsim needs.
    Path,
}

impl Command {
    /// The project a `check` or `run` names, or a `survey` or `new` takes
    /// with `--project`: the file whose `[catalog]` decides which binary
    /// runs the command.
    fn project(&self) -> Option<&Path> {
        match self {
            Self::Check { project, .. } | Self::Run { project, .. } => Some(project),
            Self::Survey { project, .. } | Self::New { project, .. } => project.as_deref(),
            Self::Qemu { .. } => None,
        }
    }

    /// Whether `--rebuild` was given.
    fn rebuild(&self) -> bool {
        match self {
            Self::Check { rebuild, .. } | Self::Run { rebuild, .. } => *rebuild,
            Self::Survey { .. } | Self::New { .. } | Self::Qemu { .. } => false,
        }
    }
}

/// The catalogs the `embsim` command ships: the standard catalog
/// (`embsim_boards::catalog::StandardCatalog`, its `held-in-reset` core
/// with it) and QEMU as a P2 core (`embsim_p2_qemu::catalog`). A project's
/// own catalogs join this set.
pub fn shipped() -> CatalogSet {
    let mut set = CatalogSet::new();
    embsim_p2_qemu::catalog::register(&mut set).expect(
        "QEMU's core is spelled as a kind is, and the set holds no other catalog by its name",
    );
    set
}

/// The `embsim` binary's `main`: the command over [`shipped`], except that
/// a `check` or `run` of a project with `[catalog]`, or a `survey` or `new`
/// with `--project` naming one, is handed to the project's runner — its
/// own runner crate, or one the tool writes beside it — built with Cargo
/// and `exec`ed with the same arguments (`PROJECTS.md` §10, "The runner").
///
/// Before it hands over it reads only the project's head, leniently
/// (`ProjectHead`): its `[catalog]` and `requires-embsim`. A command line
/// clap cannot parse — a flag only a newer embsim has — is still handed
/// over when it names a project with `[catalog]` (the project after
/// `check` or `run`, or `--project`), for the runner's embsim to parse; a
/// project without one that requires another embsim says so.
pub fn tool_main() -> ExitCode {
    let args: Vec<OsString> = std::env::args_os().collect();
    let mut out = std::io::stdout();
    let mut err = std::io::stderr();
    let cli = match parse(&args, &[]) {
        Ok(cli) => cli,
        Err(usage) => {
            if usage.use_stderr() {
                if let Some((project, rebuild)) = scan_project(&args) {
                    if let Ok(head) = ProjectHead::of_project(&project) {
                        if let Some(catalog) = head.catalog {
                            return hand_over(&project, &catalog, rebuild, &args, &mut err);
                        }
                        if let Err(message) = head.check_version() {
                            let _ = writeln!(err, "error: {}: {message}", project.display());
                            return ExitCode::FAILURE;
                        }
                    }
                }
            }
            return usage_exit(&usage, &mut out, &mut err);
        }
    };
    if let Some(project) = cli.command.project() {
        let head = match ProjectHead::of_project(project) {
            Ok(head) => head,
            Err(message) => {
                let _ = writeln!(err, "error: {message}");
                return ExitCode::FAILURE;
            }
        };
        if let Some(catalog) = head.catalog {
            return hand_over(project, &catalog, cli.command.rebuild(), &args, &mut err);
        }
        if let Err(message) = head.check_version() {
            let _ = writeln!(err, "error: {}: {message}", project.display());
            return ExitCode::FAILURE;
        }
    }
    execute(&shipped(), &[], cli.command, &mut out, &mut err)
}

/// Hand `project` to its runner; returns only when that failed.
fn hand_over(
    project: &Path,
    catalog: &CatalogTable,
    rebuild: bool,
    args: &[OsString],
    err: &mut dyn Write,
) -> ExitCode {
    let outcome = runner::hand_over(project, catalog, rebuild, args, err);
    // `hand_over` returns only when it could not hand over.
    let Err(message) = outcome;
    let _ = writeln!(err, "error: {message}");
    ExitCode::FAILURE
}

/// The project a command line names, read without clap: the first argument
/// after `check` or `run` that names a file, or `--project`'s value after
/// `survey` or `new`; and whether `--rebuild` is among the arguments. What
/// the tool hands over on when clap refuses the line, for the runner's
/// embsim to parse.
fn scan_project(args: &[OsString]) -> Option<(PathBuf, bool)> {
    let words: Vec<&OsStr> = args.iter().skip(1).map(OsString::as_os_str).collect();
    let at = words
        .iter()
        .position(|word| matches!(word.to_str(), Some("check" | "run" | "survey" | "new")))?;
    let rebuild = words[at..].iter().any(|word| *word == "--rebuild");
    let rest = &words[at + 1..];
    let project = match words[at].to_str() {
        Some("check" | "run") => rest
            .iter()
            .filter(|word| !word.to_string_lossy().starts_with('-'))
            .map(PathBuf::from)
            .find(|path| path.is_file()),
        _ => rest.iter().enumerate().find_map(|(index, word)| {
            let word = word.to_string_lossy();
            match word.strip_prefix("--project") {
                Some("") => rest.get(index + 1).map(PathBuf::from),
                Some(value) => value.strip_prefix('=').map(PathBuf::from),
                None => None,
            }
        }),
    }?;
    Some((project, rebuild))
}

/// A project's catalog crate's registration function: it adds the crate's
/// catalogs and P2 cores to the set (`CatalogSet::add`,
/// `CatalogSet::add_cores`), and starts nothing.
pub type Register = fn(&mut CatalogSet) -> Result<(), ProjectError>;

/// One catalog crate a runner was built with: what a runner's main lists,
/// one per crate in the project's `[catalog]`. Non-exhaustive: build one
/// with [`Self::new`].
#[derive(Debug, Clone, Copy)]
#[non_exhaustive]
pub struct CatalogCrate {
    /// The crate's package name, as an error names it.
    pub name: &'static str,
    /// The crate's directory, absolute: what the runner compares a
    /// project's `[catalog] crates` with.
    pub dir: &'static str,
    /// The crate's registration function (`pub fn register` at its root).
    pub register: Register,
}

impl CatalogCrate {
    /// The crate `name` in the directory `dir` (absolute, or reached from
    /// the runner crate's own: `concat!(env!("CARGO_MANIFEST_DIR"),
    /// "/../catalog")`), registering with `register`.
    pub const fn new(name: &'static str, dir: &'static str, register: Register) -> Self {
        Self {
            name,
            dir,
            register,
        }
    }
}

/// A runner's `main`: the command over [`shipped`] and `crates`, each
/// crate's registration function called in order, with the process's own
/// arguments and output. A `check` or `run` of a project (or a `survey` or
/// `new` with `--project`) whose `[catalog]` names other crates is
/// refused, naming both: a runner runs only the projects whose crates it
/// holds. What it prints of what it is made of names the crates, and the
/// `embsim` tool that started it adds each one's version and revision.
pub fn runner_main(crates: &[CatalogCrate]) -> ExitCode {
    run_with_crates(
        crates,
        std::env::args_os(),
        &mut std::io::stdout(),
        &mut std::io::stderr(),
    )
}

/// [`runner_main`] with its arguments and its output handed in, for a test.
pub fn run_with_crates<I, T>(
    crates: &[CatalogCrate],
    args: I,
    out: &mut dyn Write,
    err: &mut dyn Write,
) -> ExitCode
where
    I: IntoIterator<Item = T>,
    T: Into<OsString> + Clone,
{
    let mut set = shipped();
    for catalog in crates {
        if let Err(message) = (catalog.register)(&mut set) {
            let _ = writeln!(
                err,
                "error: catalog crate {} ({}): {message}",
                catalog.name, catalog.dir
            );
            return ExitCode::FAILURE;
        }
    }
    let args: Vec<OsString> = args.into_iter().map(Into::into).collect();
    let cli = match parse(&args, crates) {
        Ok(cli) => cli,
        Err(usage) => return usage_exit(&usage, out, err),
    };
    if let Some(project) = cli.command.project() {
        if let Err(message) = runner::check_runner_fits(crates, project) {
            let _ = writeln!(err, "error: {message}");
            return ExitCode::FAILURE;
        }
    }
    execute(&set, crates, cli.command, out, err)
}

/// The `embsim` command over `set`, with the process's own arguments and
/// standard output: a binary of a project's own, with its catalogs in
/// `set`. It runs every project itself, whatever its `[catalog]` says. A
/// usage error prints clap's message and exits 2.
pub fn main_with(set: CatalogSet) -> ExitCode {
    run(
        &set,
        std::env::args_os(),
        &mut std::io::stdout(),
        &mut std::io::stderr(),
    )
}

/// The `embsim` command over `set` with `args` — the program's name first,
/// as `std::env::args_os` gives them — writing what it prints to `out` and
/// its errors to `err`. A usage error writes clap's message and returns its
/// code (2); `--help` and `--version` write theirs to `out` and return 0.
pub fn run<I, T>(set: &CatalogSet, args: I, out: &mut dyn Write, err: &mut dyn Write) -> ExitCode
where
    I: IntoIterator<Item = T>,
    T: Into<OsString> + Clone,
{
    let args: Vec<OsString> = args.into_iter().map(Into::into).collect();
    match parse(&args, &[]) {
        Ok(cli) => execute(set, &[], cli.command, out, err),
        Err(usage) => usage_exit(&usage, out, err),
    }
}

/// The command line `args` for a binary over `crates`, its `--version`
/// saying what the binary is made of.
fn parse(args: &[OsString], crates: &[CatalogCrate]) -> Result<Cli, clap::Error> {
    Cli::command()
        .version(provenance::short_version())
        .long_version(provenance::long_version(crates))
        .try_get_matches_from(args)
        .and_then(|matches| Cli::from_arg_matches(&matches))
}

/// Write clap's message where clap would write it, and return its exit
/// code: 0 for `--help` and `--version`, 2 for a usage error.
fn usage_exit(usage: &clap::Error, out: &mut dyn Write, err: &mut dyn Write) -> ExitCode {
    let text = usage.render().to_string();
    let _ = if usage.use_stderr() {
        write!(err, "{text}")
    } else {
        write!(out, "{text}")
    };
    ExitCode::from(u8::try_from(usage.exit_code()).unwrap_or(2))
}

fn execute(
    set: &CatalogSet,
    crates: &[CatalogCrate],
    command: Command,
    out: &mut dyn Write,
    err: &mut dyn Write,
) -> ExitCode {
    let outcome = match command {
        // `--project` chose the binary, and so the set; the survey reads
        // nothing else of the file.
        Command::Survey {
            netlist,
            kind,
            project,
        } => match (netlist, kind) {
            (Some(netlist), None) => checklist::survey(set, &netlist, out),
            (None, Some(kind)) => checklist::survey_kind(set, &kind, project.is_some(), out),
            _ => unreachable!("clap requires exactly one of the netlist and --kind"),
        },
        Command::New {
            netlist,
            name,
            output,
            force,
            catalog,
            own_runner,
            add_to,
            project,
        } => {
            // An empty path is the default directory, beside the crate.
            let own_runner = own_runner.map(Option::unwrap_or_default);
            match (netlist, catalog) {
                (Some(netlist), catalog) => checklist::new_project(
                    set,
                    &netlist,
                    &checklist::NewOptions {
                        name: name.as_deref(),
                        output: output.as_deref(),
                        force,
                        catalog: catalog.as_deref(),
                        own_runner: own_runner.as_deref(),
                        project: project.as_deref(),
                    },
                    out,
                ),
                (None, Some(catalog)) => {
                    scaffold::new_catalog(&catalog, own_runner.as_deref(), add_to.as_deref(), out)
                }
                (None, None) => unreachable!("clap requires the netlist unless --catalog is given"),
            }
        }
        Command::Check { project, .. } => {
            live::check(set, &project, &provenance::lines(crates), out)
        }
        Command::Run {
            project,
            duration,
            nets,
            ptys,
            ..
        } => live::run(
            set,
            &project,
            &live::RunOptions {
                duration,
                nets,
                ptys,
                provenance: provenance::lines(crates),
            },
            out,
        ),
        Command::Qemu { command } => match command {
            QemuCommand::Install {
                prefix,
                build_dir,
                keep_build,
                qemu_git,
                jobs,
                force,
                dry_run,
            } => qemu::install(
                &embsim_p2_qemu::install::InstallOptions {
                    prefix,
                    build_dir,
                    qemu_git,
                    jobs,
                    keep_build,
                    force,
                    dry_run,
                },
                out,
            ),
            QemuCommand::Path => qemu::path(out),
        },
    };
    let _ = out.flush();
    match outcome {
        Ok(()) => ExitCode::SUCCESS,
        Err(message) => {
            let _ = writeln!(err, "error: {message}");
            ExitCode::FAILURE
        }
    }
}
