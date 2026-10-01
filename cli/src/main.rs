//! `embsim` — a system simulated from its boards' netlists, written down as
//! a project file.
//!
//! A project names boards (a KiCad netlist, or a board the catalog ships),
//! the model each part takes, the wires between connectors, and the
//! scenario (`embsim_board::Project`). Four subcommands take a netlist to a
//! running system:
//!
//! - `embsim survey <netlist>` — the checklist: what the catalog populates
//!   by itself, what needs a model and which catalog kinds could be it, and
//!   every connector pin a wire may use. `embsim survey --kind p2-ec32mb`
//!   surveys a board the catalog ships the same way.
//! - `embsim new <netlist>` — a starter project answering that checklist as
//!   far as the catalog can, with a commented stub for every part left.
//! - `embsim check <project>` — loads, surveys and builds the system with
//!   time held, and prints what the build found; any refusal is an error
//!   with the text that says what to fix.
//! - `embsim run <project>` — starts the system and runs it in virtual
//!   time. A P2 whose `core` is `"qemu"` boots its ROM off the board's
//!   flash (`embsim_p2_qemu::catalog`).
//!
//! Every board is built by the one pipeline (`DESIGN.md` rule 1): the
//! project's netlist and registry through `Board::from_netlist`, surveyed
//! first with the same registry. The catalog is
//! [`embsim_p2_qemu::catalog::QemuCatalog`], the standard catalog with QEMU
//! as a core.

use std::path::PathBuf;
use std::process::ExitCode;

use clap::{ArgGroup, Parser, Subcommand};

mod checklist;
mod live;

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
    /// The checklist for a netlist, or for a board kind the catalog ships:
    /// the parts the catalog populates, the parts that need a model and the
    /// kinds that could be them, and every connector pin with its name and
    /// net.
    #[command(group(ArgGroup::new("board").required(true).args(["netlist", "kind"])))]
    Survey {
        /// A KiCad netlist export (`kicad-cli sch export netlist`).
        netlist: Option<PathBuf>,
        /// A board kind the catalog ships (`p2-ec32mb`), surveyed with the
        /// registry a project builds it with.
        #[arg(long)]
        kind: Option<String>,
    },
    /// Write a starter project for a netlist: the board, the pin tables the
    /// catalog can choose, a commented stub for every part that needs a
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
    /// findings and a P2's console output as the run reaches them.
    Run {
        /// The project file.
        project: PathBuf,
        /// How long to run, in virtual time: `20ms`, `1.5s`, `250us`,
        /// `100ns`. Without it the run lasts until it is interrupted.
        #[arg(long = "for", value_name = "DURATION", value_parser = live::parse_duration)]
        duration: Option<u64>,
        /// A net to read when the run ends, as `Board.Net` (repeat for
        /// more).
        #[arg(long = "net", value_name = "BOARD.NET")]
        nets: Vec<String>,
    },
}

fn main() -> ExitCode {
    let cli = Cli::parse();
    let outcome = match cli.command {
        Command::Survey { netlist, kind } => match (netlist, kind) {
            (Some(netlist), None) => checklist::survey(&netlist),
            (None, Some(kind)) => checklist::survey_kind(&kind),
            _ => unreachable!("clap requires exactly one of the netlist and --kind"),
        },
        Command::New {
            netlist,
            name,
            output,
            force,
        } => checklist::new_project(&netlist, name.as_deref(), output.as_deref(), force),
        Command::Check { project } => live::check(&project),
        Command::Run {
            project,
            duration,
            nets,
        } => live::run(&project, duration, &nets),
    };
    match outcome {
        Ok(()) => ExitCode::SUCCESS,
        Err(message) => {
            eprintln!("error: {message}");
            ExitCode::FAILURE
        }
    }
}
