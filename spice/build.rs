//! Link libngspice via pkg-config (`ngspice.pc`).
//!
//! Ubuntu: `libngspice0-dev`. macOS: `brew install libngspice`.

fn main() {
    println!("cargo:rerun-if-env-changed=PKG_CONFIG_PATH");
    pkg_config::Config::new()
        .atleast_version("36")
        .probe("ngspice")
        .unwrap_or_else(|err| {
            panic!(
                "libngspice not found via pkg-config ({err}). Install the shared library:\n\
                 - Ubuntu/Debian: sudo apt-get install -y libngspice0-dev pkg-config\n\
                 - macOS:         brew install libngspice pkg-config"
            );
        });
}
