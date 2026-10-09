//! The `qemu-vm` and `chrome-vm` bench kinds as a project names them: what
//! building a project checks, and that building one boots nothing.
//!
//! Nothing here launches QEMU. A project whose `qemu` is a file that is not
//! QEMU at all — this test's own executable — builds, and is started with
//! time held and stopped again, as `embsim check` does: the VM boots at the
//! node's first slice, which a held clock never reaches, so the stand-in is
//! never run. The cases that boot real QEMU are `tests/real_qemu.rs`.

use std::path::PathBuf;
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{Mutex, MutexGuard};

use embsim_board::{Project, Reports};
use embsim_boards::catalog::CatalogSet;
use embsim_core::virtual_clock::{self, ClockMode};
use rstest::rstest;
use vibes_behaviour::{behaviour, expect, Test};

static SUITE_LOCK: Mutex<()> = Mutex::new(());

fn suite_lock() -> MutexGuard<'static, ()> {
    SUITE_LOCK.lock().unwrap_or_else(|poisoned| {
        SUITE_LOCK.clear_poison();
        poisoned.into_inner()
    })
}

/// The set the `embsim` command ships, as far as these kinds go.
fn set() -> CatalogSet {
    let mut set = CatalogSet::new();
    embsim_qemu::catalog::register(&mut set).expect("the VM kinds join a set");
    set
}

/// A file that exists and is not QEMU: what `qemu` names when the case
/// must never launch it.
fn stand_in() -> PathBuf {
    std::env::current_exe().expect("the test binary has a path")
}

/// A scratch directory of the case's own, holding a stand-in image,
/// removed when the case ends.
struct Scratch(PathBuf);

impl Drop for Scratch {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

fn scratch() -> Scratch {
    static NEXT: AtomicU32 = AtomicU32::new(0);
    let dir = std::env::temp_dir().join(format!(
        "embsim-qemu-catalog-{}-{}",
        std::process::id(),
        NEXT.fetch_add(1, Ordering::Relaxed)
    ));
    std::fs::create_dir_all(&dir).expect("the scratch directory can be made");
    std::fs::write(dir.join("disk.qcow2"), b"not a disk").expect("the stand-in image");
    Scratch(dir)
}

/// A project of one VM component named `PC`, of `kind`, with `options`.
fn project(kind: &str, options: &str) -> String {
    format!(
        "[[component]]\nname = \"PC\"\nkind = \"{kind}\"\n[component.options]\n{options}\n\n\
         [[wire]]\nfrom = \"BENCH.3V3\"\nto = \"PC.VIO\"\nvolts = 3.3\n\n\
         [[wire]]\nfrom = \"BENCH.GND\"\nto = \"PC.GND\"\nvolts = 0.0\n"
    )
}

#[rstest]
#[case::qemu_vm("qemu-vm", "image = \"disk.qcow2\"\nfirmware = \"disk.qcow2\"")]
#[case::chrome_vm(
    "chrome-vm",
    "image = \"disk.qcow2\"\nfirmware = \"disk.qcow2\"\ndevtools_port = 9333"
)]
fn a_vm_in_a_project_is_built_and_checked_without_booting(#[case] kind: &str, #[case] extra: &str) {
    behaviour!(Test {
        id: "qemu-vm.checked-without-booting",
        covers: Some("qemu/src/catalog.rs#VmCatalog"),
        given: "a project whose one bench component is a virtual machine, built and started \
                with its clock held, then stopped, as embsim check does",
    });
    expect!(
        "builds",
        "the project builds, its VM on the four pins a host's serial line has"
    );
    expect!(
        "boots-nothing",
        "the VM is never launched, and nothing reports it failed",
        "the VM boots at its first slice, a quantum after the run starts"
    );
    expect!(
        "says-what-it-runs",
        "its first report line names what it runs, the line's rate and the metering quantum"
    );

    let _suite = suite_lock();
    let dir = scratch();
    let text = project(
        kind,
        &format!("baud = 115200\nqemu = {:?}\n{extra}", stand_in()),
    );
    virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);
    let reports = Reports::new();
    let system = Project::parse(&text)
        .expect("the text is a project")
        .relative_to(&dir.0)
        .instantiate_with(&set(), &reports)
        .expect("builds: the project builds");
    let handle = system
        .hold_time()
        .start()
        .expect("builds: the system starts");
    let refs: Vec<&str> = handle.component_refs().collect();
    assert_eq!(refs, ["PC"], "builds");
    for net in ["PC.TX", "PC.RX", "PC.VIO", "PC.GND"] {
        assert!(handle.net_state(net).is_some(), "builds: no net {net}");
    }
    let mut reports = reports.take();
    assert_eq!(reports.len(), 1);
    let first = reports[0].look(0);
    assert!(
        first
            .iter()
            .any(|line| line.contains("115200 baud 8N1")
                && line.contains("metered every 1.000000 ms")),
        "says-what-it-runs: {first:?}"
    );
    handle.shutdown();
    assert_eq!(reports[0].failure(), None, "boots-nothing");
    let summary = reports[0].summary().join("\n");
    assert!(summary.contains("0 slices"), "boots-nothing: {summary}");
}

#[rstest]
#[case::no_baud("qemu-vm", "", "options.baud is the serial line's rate")]
#[case::zero_baud("qemu-vm", "baud = 0", "options.baud = 0 is not a rate")]
#[case::zero_quantum(
    "qemu-vm",
    "baud = 9600\nquantum = \"0ms\"",
    "options.quantum is how much virtual time passes between two slices"
)]
#[case::long_quantum(
    "qemu-vm",
    "baud = 9600\nquantum = \"2s\"",
    "options.quantum is at most 1000.000000 ms"
)]
#[case::zero_max_lead(
    "qemu-vm",
    "baud = 9600\nmax_lead = \"0ms\"",
    "options.max_lead is how far the guest may end a slice ahead of the board"
)]
#[case::no_image(
    "qemu-vm",
    "baud = 9600\nimage = \"missing.qcow2\"",
    "options.image: no file at"
)]
#[case::bad_serial(
    "qemu-vm",
    "baud = 9600\nserial = \"bluetooth\"",
    "options.serial = \"bluetooth\" is not one this kind offers"
)]
#[case::bad_args(
    "qemu-vm",
    "baud = 9600\nargs = [\"-m\", 4]",
    "options.args is a list of QEMU's arguments"
)]
#[case::unknown(
    "qemu-vm",
    "baud = 9600\nparity = \"even\"",
    "unknown option \"parity\""
)]
#[case::chrome_no_image(
    "chrome-vm",
    "baud = 9600\nimage = \"missing.qcow2\"",
    "options.image: no file at"
)]
#[case::chrome_bad_port(
    "chrome-vm",
    "baud = 9600\nimage = \"disk.qcow2\"\nfirmware = \"disk.qcow2\"\ndevtools_port = 70000",
    "options.devtools_port = 70000 is not a TCP port"
)]
fn a_vm_entry_that_cannot_run_is_refused(
    #[case] kind: &str,
    #[case] options: &str,
    #[case] says: &str,
) {
    behaviour!(Test {
        id: "qemu-vm.refusals",
        covers: Some("qemu/src/catalog.rs#VmCatalog"),
        given: "a VM entry missing its baud rate, or with a zero or over-long quantum, a zero \
                bound on how far the guest may lead the board, a missing image, or an option, \
                serial device or DevTools port the kind does not take",
    });
    expect!(
        "refused-saying-why",
        "the project is refused before anything is launched, naming the component and what \
         to fix"
    );
    let _suite = suite_lock();
    let dir = scratch();
    let text = project(kind, &format!("qemu = {:?}\n{options}", stand_in()));
    let message = Project::parse(&text)
        .expect("the text is a project")
        .relative_to(&dir.0)
        .instantiate(&set())
        .expect_err("the entry is refused")
        .to_string();
    assert!(
        message.contains(&format!("component PC (kind \"{kind}\")")),
        "refused-saying-why: {message}"
    );
    assert!(message.contains(says), "{says:?} missing from:\n{message}");
}

#[rstest]
fn a_vm_with_no_qemu_is_refused_saying_where_qemu_comes_from() {
    behaviour!(Test {
        id: "qemu-vm.no-qemu",
        covers: Some("qemu/src/catalog.rs#VmCatalog"),
        given: "a virtual machine entry whose QEMU binary is not there",
    });
    expect!(
        "names-the-host-qemu",
        "the project is refused, saying the VM runs on the host's own system QEMU and where \
         that comes from"
    );
    let _suite = suite_lock();
    let text = project(
        "qemu-vm",
        "baud = 9600\nqemu = \"/nonexistent/qemu-system-aarch64\"",
    );
    let message = Project::parse(&text)
        .expect("the text is a project")
        .instantiate(&set())
        .expect_err("the entry is refused")
        .to_string();
    assert!(
        message.contains("no /nonexistent/qemu-system-aarch64 on PATH or at that path")
            && message.contains("the host's own system QEMU"),
        "names-the-host-qemu: {message}"
    );
}
