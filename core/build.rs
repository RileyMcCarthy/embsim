//! The build script `links = "embsim-core"` needs (Cargo.toml): it claims
//! the name, so Cargo refuses a dependency graph that holds two copies of
//! embsim when it resolves it. It builds nothing.

fn main() {
    // Nothing here depends on any file: run once.
    println!("cargo:rerun-if-changed=build.rs");
}
