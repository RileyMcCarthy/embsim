//! `embsim qemu install` and `embsim qemu path`: the P2's QEMU core runs in
//! a `qemu-system-p2` of its own, which these install and find
//! (`embsim_p2_qemu::install`, `embsim_p2_qemu::peer`). And `embsim
//! flash-image`: a program laid out as the boot flash the P2's ROM boots
//! (`embsim_p2_qemu::flashimage`).

use std::io::Write;
use std::path::Path;

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

/// The P2X8C4M64P's hub RAM, which stage-1 loads the program into from $0:
/// 512 KB (Parallax, "Propeller 2 P2X8C4M64P Datasheet", "Hub RAM").
const HUB_BYTES: usize = 512 * 1024;

/// `embsim flash-image PROGRAM -o IMAGE`: embsim's stage-1 loader
/// ([`embsim_p2_qemu::STAGE1`]) in the first kilobyte, balanced so its 256
/// longs sum to `"Prop"`, then the program's length and the program at
/// `$400` ([`embsim_p2_qemu::flashimage::boot_flash`]), written to
/// `output`.
pub fn flash_image(program: &Path, output: &Path, out: &mut dyn Write) -> Result<(), String> {
    let bytes = std::fs::read(program)
        .map_err(|err| format!("cannot read the program {}: {err}", program.display()))?;
    if bytes.is_empty() {
        return Err(format!(
            "the program {} is empty; it is the P2 binary the compiler wrote, which stage-1 \
             copies into hub RAM from $0",
            program.display()
        ));
    }
    if bytes.len() > HUB_BYTES {
        return Err(format!(
            "the program {} is {} bytes, and stage-1 copies it into the P2's hub RAM, which \
             holds {HUB_BYTES}",
            program.display(),
            bytes.len()
        ));
    }
    let image = embsim_p2_qemu::flashimage::boot_flash(embsim_p2_qemu::STAGE1, &bytes)
        .map_err(|err| format!("embsim's stage-1 does not fit its kilobyte: {err}"))?;
    std::fs::write(output, &image)
        .map_err(|err| format!("cannot write {}: {err}", output.display()))?;
    let _ = writeln!(
        out,
        "wrote {}: {} bytes, a flash image the P2's boot ROM boots",
        output.display(),
        image.len()
    );
    let _ = writeln!(
        out,
        "  $000  embsim's stage-1 loader ({} bytes), its first kilobyte summing to \"Prop\"",
        embsim_p2_qemu::STAGE1.len()
    );
    let _ = writeln!(out, "  $400  the program's length, {} bytes", bytes.len());
    let _ = writeln!(
        out,
        "  $404  {}, which stage-1 copies into hub RAM from $0 and runs",
        program.display()
    );
    let _ = writeln!(
        out,
        "  a w25q128jv part's `image` option names it, relative to the project file"
    );
    Ok(())
}
