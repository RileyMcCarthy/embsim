//! `embsim` — a system simulated from its boards' netlists, written down as
//! a project file: the command over the catalogs embsim ships
//! ([`embsim_cli::shipped`]). The command itself is the library
//! (`embsim_cli`), so a project with kinds of its own runs the same command
//! over a set its catalogs join.

use std::process::ExitCode;

fn main() -> ExitCode {
    embsim_cli::main_with(embsim_cli::shipped())
}
