//! The example project's own binary: the `embsim` command over the
//! catalogs embsim ships and this crate's, for a project that would rather
//! own its binary than have the `embsim` tool build a runner for it
//! (`PROJECTS.md` §10, "The catalog crate").
//!
//! ```bash
//! cargo run -p custom-project-catalog --example own_binary -- run project.toml --for 10ms
//! ```
//!
//! It runs every project itself, whatever its `[catalog]` names: only the
//! `embsim` tool hands a project to a runner.

use std::process::ExitCode;

fn main() -> ExitCode {
    let mut set = embsim_cli::shipped();
    match custom_project_catalog::register(&mut set) {
        Ok(()) => embsim_cli::main_with(set),
        Err(err) => {
            eprintln!("error: {err}");
            ExitCode::FAILURE
        }
    }
}
