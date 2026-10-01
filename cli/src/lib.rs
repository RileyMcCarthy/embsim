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
//! Every kind a project names comes from a [`CatalogSet`]. The `embsim`
//! binary is [`main_with`] over [`shipped`]: the standard catalog, and QEMU
//! as a P2 core. A project with kinds of its own — a model, a board, an
//! instruction-set simulator as the P2's core, a bench part — is the same
//! command over a set its own catalogs join: a binary of ten lines.
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
//! for a test or a program that runs it in place.
//!
//! Every board is built by the one pipeline (`DESIGN.md` rule 1): the
//! project's netlist and registry through `Board::from_netlist`, surveyed
//! first with the same registry, and every kind — whichever catalog it
//! comes from — seated only on a part that is what it says.

use std::ffi::OsString;
use std::io::Write;
use std::path::PathBuf;
use std::process::ExitCode;

use clap::{ArgGroup, Parser, Subcommand};

pub use embsim_boards::catalog::CatalogSet;

mod checklist;
mod live;
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
    /// model, and the connectors' pins to wire.
    New {
        /// A KiCad netlist export.
        netlist: PathBuf,
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
    },
    /// Load a project, survey its boards and build its system with time
    /// held; print what the build found. Exits non-zero, with the reason,
    /// when anything is refused.
    Check {
        /// The project file.
        project: PathBuf,
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
    },
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

/// The `embsim` command over `set`, with the process's own arguments and
/// standard output: what the `embsim` binary's `main` is, with
/// [`shipped`]. A usage error prints clap's message and exits 2.
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
    match Cli::try_parse_from(args) {
        Ok(cli) => execute(set, cli.command, out, err),
        Err(usage) => {
            let text = usage.render().to_string();
            let _ = if usage.use_stderr() {
                write!(err, "{text}")
            } else {
                write!(out, "{text}")
            };
            ExitCode::from(u8::try_from(usage.exit_code()).unwrap_or(2))
        }
    }
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
        } => checklist::new_project(
            set,
            &netlist,
            name.as_deref(),
            output.as_deref(),
            force,
            out,
        ),
        Command::Check { project } => live::check(set, &project, out),
        Command::Run {
            project,
            duration,
            nets,
            ptys,
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
