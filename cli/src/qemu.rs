//! `embsim qemu install` and `embsim qemu path`: the P2's QEMU core runs in
//! a `qemu-system-p2` of its own, which these install and find
//! (`embsim_p2_qemu::install`, `embsim_p2_qemu::peer`).

use std::io::Write;

use embsim_p2_qemu::install::{self, InstallOptions};
use embsim_p2_qemu::peer::install_advice;
use embsim_p2_qemu::{target, Identity, QemuSystemP2};

/// `embsim qemu install`.
pub fn install(options: &InstallOptions, out: &mut dyn Write) -> Result<(), String> {
    install::install(options, out).map(|_| ())
}

/// `embsim qemu path`: the program a run would start, where it was found,
/// what it says it is, and whether that is what this embsim needs. Fails
/// when there is none, or it is not the one.
pub fn path(out: &mut dyn Write) -> Result<(), String> {
    let needed = Identity::needed();
    let _ = writeln!(out, "this embsim needs: {needed}");
    let program = QemuSystemP2::find().map_err(|err| err.to_string())?;
    let _ = writeln!(
        out,
        "qemu-system-p2: {} ({})",
        program.path().display(),
        program.found().describe()
    );
    let identity = program.probe().map_err(|err| err.to_string())?;
    let _ = writeln!(out, "it says: {identity}");
    match identity.check() {
        Ok(()) => {
            let _ = writeln!(
                out,
                "ok: the one this embsim needs (target {})",
                target::identity()
            );
            Ok(())
        }
        Err(why) => Err(format!(
            "{} is not the qemu-system-p2 this embsim needs: {why}. {}",
            program.path().display(),
            install_advice()
        )),
    }
}
