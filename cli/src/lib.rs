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
//!   surveys a board a catalog ships the same way.
//! - `embsim new <netlist>` — a starter project answering that checklist as
//!   far as the catalogs can, with a commented stub for every part left.
//! - `embsim check <project>` — loads, surveys and builds the system with
//!   time held, and prints what the build found; any refusal is an error
//!   with the text that says what to fix.
//! - `embsim run <project>` — starts the system and runs it in virtual
//!   time, until `--for` elapses or the run is interrupted (Ctrl-C), then
//!   prints its summary.
//!
//! Every kind a project names comes from a [`CatalogSet`]. A project with
//! kinds of its own — a model, a board, an instruction-set simulator as the
//! P2's core, a bench part — writes them in a catalog crate and names it in
//! its file (`[catalog] crates = ["sim/catalog"]`). The `embsim` binary is
//! [`tool_main`]: a project without `[catalog]` runs in it, over
//! [`shipped`] (the standard catalog, and QEMU as a P2 core); a project
//! with `[catalog]` is handed to a **runner**, a crate the tool writes
//! beside the project, builds with Cargo and `exec`s — the same command
//! over a set the project's catalogs joined ([`runner_main`];
//! `PROJECTS.md` §10, "The runner"). `embsim new --catalog DIR` starts a
//! catalog crate.
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

use std::ffi::OsString;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::process::ExitCode;

use clap::{ArgGroup, Parser, Subcommand};

use embsim_board::{CatalogTable, ProjectError};
pub use embsim_boards::catalog::CatalogSet;

mod checklist;
mod live;
mod runner;
mod scaffold;
mod signals;

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
}

impl Command {
    /// The project a `check` or `run` names: the file whose `[catalog]`
    /// decides which binary runs it.
    fn project(&self) -> Option<&Path> {
        match self {
            Self::Check { project, .. } | Self::Run { project, .. } => Some(project),
            Self::Survey { .. } | Self::New { .. } => None,
        }
    }

    /// Whether `--rebuild` was given.
    fn rebuild(&self) -> bool {
        match self {
            Self::Check { rebuild, .. } | Self::Run { rebuild, .. } => *rebuild,
            Self::Survey { .. } | Self::New { .. } => false,
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

/// The embsim checkout this crate was built from: its workspace root. A
/// runner the `embsim` tool builds takes embsim from here unless the
/// project names another (`[catalog] embsim`). For `cargo install --path
/// cli` it is that checkout; for `cargo install --git` it is the checkout
/// Cargo keeps under `$CARGO_HOME/git/checkouts`, which lasts until Cargo's
/// cache is cleaned. It may be gone when a runner is wanted, and the tool
/// then says to name one.
pub fn source_dir() -> PathBuf {
    let cli = Path::new(env!("CARGO_MANIFEST_DIR"));
    cli.parent().unwrap_or(cli).to_path_buf()
}

/// The `embsim` binary's `main`: the command over [`shipped`], except that
/// a `check` or `run` of a project with `[catalog]` is handed to the
/// project's runner — written beside the project, built with Cargo and
/// `exec`ed with the same arguments (`PROJECTS.md` §10, "The runner").
/// Before it hands over it reads only the project's `[catalog]` table.
pub fn tool_main() -> ExitCode {
    let args: Vec<OsString> = std::env::args_os().collect();
    let cli = Cli::parse_from(&args);
    let mut err = std::io::stderr();
    if let Some(project) = cli.command.project() {
        match CatalogTable::of_project(project) {
            Ok(Some(catalog)) => {
                let rebuild = cli.command.rebuild();
                let outcome = runner::hand_over(project, &catalog, rebuild, &args, &mut err);
                // `hand_over` returns only when it could not hand over.
                let Err(message) = outcome;
                let _ = writeln!(err, "error: {message}");
                return ExitCode::FAILURE;
            }
            Ok(None) => {}
            Err(message) => {
                let _ = writeln!(err, "error: {message}");
                return ExitCode::FAILURE;
            }
        }
    }
    execute(&shipped(), cli.command, &mut std::io::stdout(), &mut err)
}

/// A project's catalog crate's registration function: it adds the crate's
/// catalogs and P2 cores to the set (`CatalogSet::add`,
/// `CatalogSet::add_cores`), and starts nothing.
pub type Register = fn(&mut CatalogSet) -> Result<(), ProjectError>;

/// One catalog crate a runner was built with: what the runner main the
/// `embsim` tool writes lists, one per crate in the project's `[catalog]`.
#[derive(Debug, Clone, Copy)]
pub struct CatalogCrate {
    /// The crate's package name, as an error names it.
    pub name: &'static str,
    /// The crate's directory, absolute: what the runner compares a
    /// project's `[catalog] crates` with.
    pub dir: &'static str,
    /// The crate's registration function (`pub fn register` at its root).
    pub register: Register,
}

/// A runner's `main`: the command over [`shipped`] and `crates`, each
/// crate's registration function called in order, with the process's own
/// arguments and output. A `check` or `run` of a project whose `[catalog]`
/// names other crates, or another embsim checkout, is refused, naming
/// both: the `embsim` tool builds one runner per set of crates, and runs a
/// project only through its own.
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
    let cli = match parse(args, out, err) {
        Ok(cli) => cli,
        Err(code) => return code,
    };
    if let Some(project) = cli.command.project() {
        if let Err(message) = runner::check_runner_fits(crates, project) {
            let _ = writeln!(err, "error: {message}");
            return ExitCode::FAILURE;
        }
    }
    execute(&set, cli.command, out, err)
}

/// The `embsim` command over `set`, with the process's own arguments and
/// standard output: a binary of a project's own, with its catalogs in
/// `set`. It runs every project itself, whatever its `[catalog]` says. A
/// usage error prints clap's message and exits 2.
pub fn main_with(set: CatalogSet) -> ExitCode {
    let cli = Cli::parse();
    execute(
        &set,
        cli.command,
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
    match parse(args, out, err) {
        Ok(cli) => execute(set, cli.command, out, err),
        Err(code) => code,
    }
}

/// The command line `args`, or clap's message written where clap would
/// write it and its exit code.
fn parse<I, T>(args: I, out: &mut dyn Write, err: &mut dyn Write) -> Result<Cli, ExitCode>
where
    I: IntoIterator<Item = T>,
    T: Into<OsString> + Clone,
{
    Cli::try_parse_from(args).map_err(|usage| {
        let text = usage.render().to_string();
        let _ = if usage.use_stderr() {
            write!(err, "{text}")
        } else {
            write!(out, "{text}")
        };
        ExitCode::from(u8::try_from(usage.exit_code()).unwrap_or(2))
    })
}

fn execute(
    set: &CatalogSet,
    command: Command,
    out: &mut dyn Write,
    err: &mut dyn Write,
) -> ExitCode {
    let outcome = match command {
        Command::Survey { netlist, kind } => match (netlist, kind) {
            (Some(netlist), None) => checklist::survey(set, &netlist, out),
            (None, Some(kind)) => checklist::survey_kind(set, &kind, out),
            _ => unreachable!("clap requires exactly one of the netlist and --kind"),
        },
        Command::New {
            netlist,
            name,
            output,
            force,
            catalog,
            add_to,
        } => match (netlist, catalog) {
            (Some(netlist), catalog) => checklist::new_project(
                set,
                &netlist,
                &checklist::NewOptions {
                    name: name.as_deref(),
                    output: output.as_deref(),
                    force,
                    catalog: catalog.as_deref(),
                },
                out,
            ),
            (None, Some(catalog)) => scaffold::new_catalog(&catalog, add_to.as_deref(), out),
            (None, None) => unreachable!("clap requires the netlist unless --catalog is given"),
        },
        Command::Check { project, .. } => live::check(set, &project, out),
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
            },
            out,
        ),
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
