//! `embsim` — a system simulated from its boards' netlists, written down as
//! a project file. The command itself is the library (`embsim_cli`); this
//! binary is the tool ([`embsim_cli::tool_main`]): it runs a project on the
//! catalogs embsim ships ([`embsim_cli::shipped`]), and hands a project
//! that names catalog crates of its own (`[catalog]`) to the runner it
//! builds for them.

use std::process::ExitCode;

fn main() -> ExitCode {
    embsim_cli::tool_main()
}
