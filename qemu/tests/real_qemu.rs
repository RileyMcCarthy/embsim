//! The node against the host's real QEMU, its guest a counter.
//!
//! No disk and no OS: the guest is six aarch64 instructions
//! ([`COUNTER_GUEST`]) that store the CPU's virtual counter to memory in a
//! loop, so the guest's own clock can be read over QMP at any moment —
//! running or frozen — with nothing installed in it. Under HVF the guest
//! reads the host's counter less the offset QEMU re-bases at every resume;
//! under TCG, QEMU's virtual clock; either way the clock stops while QEMU
//! is stopped. That makes three things measurable against the real thing:
//! that the guest runs only while the board's clock advances, how close
//! the node keeps the two clocks over hundreds of slices, and the floor a
//! stop and a cont put under a slice ([`DEFAULT_QUANTUM`]'s figures).
//!
//! `#[ignore]`d: they need `qemu-system-aarch64` (`EMBSIM_QEMU_BIN`
//! overrides it), which the workspace's tests do not. CI's `qemu-vm` job
//! runs them on Linux under TCG; on a Mac they run under HVF:
//!
//! ```text
//! cargo test -p embsim-qemu --test real_qemu -- --ignored --nocapture
//! ```
//!
//! They declare no behaviours: the ledger's suite does not run them.

use std::path::PathBuf;
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};

use embsim_board::{EndpointRef, Harness, Project, Reports, System, SystemHandle};
use embsim_boards::catalog::CatalogSet;
use embsim_core::virtual_clock::{self, Actor, ClockMode};
use embsim_qemu::{Accel, Guest, QemuNode, QemuSpec, QemuVm, Qmp, SerialDevice, DEFAULT_QUANTUM};

/// Where the counter guest is loaded: above the device tree QEMU's `virt`
/// places at the bottom of RAM (`0x4000_0000`, one megabyte).
const LOAD_AT: u64 = 0x4020_0000;

/// Where it stores the counter, and the counter's frequency beside it.
const SLOT_AT: u64 = LOAD_AT + 0x100;

/// The counter guest, aarch64 (Arm ARM, A64 encodings):
///
/// ```text
///         adr  x1, slot              // 0x10000801: slot = LOAD_AT + 0x100
///         mrs  x2, cntfrq_el0        // 0xd53be002
///         str  x2, [x1, #8]          // 0xf9000422: the frequency, once
/// loop:   mrs  x0, cntvct_el0        // 0xd53be040
///         str  x0, [x1]              // 0xf9000020: the counter, always
///         b    loop                  // 0x17fffffe
/// ```
const COUNTER_GUEST: [u32; 6] = [
    0x1000_0801,
    0xd53b_e002,
    0xf900_0422,
    0xd53b_e040,
    0xf900_0020,
    0x17ff_fffe,
];

static SUITE_LOCK: Mutex<()> = Mutex::new(());

fn suite_lock() -> MutexGuard<'static, ()> {
    SUITE_LOCK.lock().unwrap_or_else(|poisoned| {
        SUITE_LOCK.clear_poison();
        poisoned.into_inner()
    })
}

/// `EMBSIM_QEMU_BIN`, else the first `qemu-system-aarch64` on `PATH`.
fn qemu_binary() -> PathBuf {
    if let Ok(bin) = std::env::var("EMBSIM_QEMU_BIN") {
        return PathBuf::from(bin);
    }
    std::env::var_os("PATH")
        .and_then(|path| {
            std::env::split_paths(&path)
                .map(|dir| dir.join("qemu-system-aarch64"))
                .find(|candidate| candidate.is_file())
        })
        .expect(
            "needs qemu-system-aarch64 on PATH (Homebrew's qemu; Debian's qemu-system-arm) or \
             EMBSIM_QEMU_BIN",
        )
}

/// The counter guest written where QEMU can load it, in a directory of the
/// case's own that goes when the case ends.
struct CounterImage(PathBuf);

impl CounterImage {
    fn write() -> Self {
        static NEXT: AtomicU32 = AtomicU32::new(0);
        let dir = std::env::temp_dir().join(format!(
            "embsim-qemu-real-{}-{}",
            std::process::id(),
            NEXT.fetch_add(1, Ordering::Relaxed)
        ));
        std::fs::create_dir_all(&dir).expect("a scratch directory");
        let bytes: Vec<u8> = COUNTER_GUEST.iter().flat_map(|w| w.to_le_bytes()).collect();
        std::fs::write(dir.join("counter.bin"), bytes).expect("the guest is written");
        Self(dir)
    }

    fn path(&self) -> PathBuf {
        self.0.join("counter.bin")
    }
}

impl Drop for CounterImage {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

/// QEMU's arguments for the counter guest on this host: `virt` with its
/// GIC in QEMU (`qemu/src/catalog.rs` says why), the accelerator the host
/// has for an aarch64 guest, 64 MiB, and the loader that starts CPU 0 at
/// the guest.
fn counter_args(image: &std::path::Path) -> Vec<String> {
    let mut args = vec!["-M".to_string(), "virt,kernel-irqchip=off".to_string()];
    args.extend(Accel::Auto.args("aarch64"));
    args.extend([
        "-m".to_string(),
        "64M".to_string(),
        "-device".to_string(),
        format!(
            "loader,file={},addr={LOAD_AT:#x},cpu-num=0",
            image.display()
        ),
    ]);
    args
}

/// The counter's value and frequency, read from guest memory over QMP.
fn read_counter(qmp: &mut Qmp) -> (u64, u64) {
    let text = qmp
        .hmp(&format!("xp /2gx {SLOT_AT:#x}"))
        .expect("the monitor reads guest memory");
    let words: Vec<u64> = text
        .split_whitespace()
        .filter_map(|word| word.strip_prefix("0x"))
        .filter(|word| word.len() == 16)
        .filter_map(|word| u64::from_str_radix(word, 16).ok())
        .collect();
    match words.as_slice() {
        [counter, frequency] => (*counter, *frequency),
        _ => panic!("unexpected monitor output: {text:?}"),
    }
}

fn ticks_to_ns(ticks: u64, frequency: u64) -> u64 {
    (u128::from(ticks) * 1_000_000_000 / u128::from(frequency)) as u64
}

/// The counter guest as a [`Guest`] whose own clock is its counter, read
/// over the VM's one control connection, which the case shares (QEMU
/// serves one client per QMP socket).
struct CounterGuest {
    vm: QemuVm,
    monitor: Arc<Mutex<Qmp>>,
}

impl CounterGuest {
    fn spawn(image: &CounterImage) -> Self {
        let vm = QemuSpec::new(qemu_binary())
            .args(counter_args(&image.path()))
            .serial(SerialDevice::UsbFtdi)
            .spawn()
            .expect("QEMU spawns");
        let monitor = Arc::new(Mutex::new(vm.control().expect("the control QMP connects")));
        Self { vm, monitor }
    }
}

impl Guest for CounterGuest {
    fn resume(&mut self) -> std::io::Result<()> {
        self.vm.resume()
    }

    fn pause(&mut self) -> std::io::Result<()> {
        self.vm.pause()
    }

    fn serial_fd(&self) -> std::os::fd::RawFd {
        self.vm.serial_fd()
    }

    fn clock_ns(&mut self) -> Option<u64> {
        let (counter, frequency) = read_counter(&mut self.monitor.lock().unwrap());
        // Zero until the guest's first instructions have run.
        (frequency > 0).then(|| ticks_to_ns(counter, frequency))
    }
}

fn ep(endpoint: &str) -> EndpointRef {
    EndpointRef::parse(endpoint).expect("endpoint parses")
}

/// One node on a 3.3 V rail, started with time held, the case's thread an
/// actor, time released.
fn bench(node: QemuNode) -> (SystemHandle, Actor) {
    virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);
    let system = System::new()
        .component("PC", Box::new(node))
        .harness(
            Harness::new()
                .power(ep("BENCH.3V3"), ep("PC.VIO"), 3.3)
                .power(ep("BENCH.GND"), ep("PC.GND"), 0.0),
        )
        .hold_time()
        .start()
        .expect("the bench starts");
    let actor = virtual_clock::register_actor("qemu-real-case");
    system.release_time();
    (system, actor)
}

fn alive(pid: u32) -> bool {
    // SAFETY: signal 0 checks for existence only; no signal is delivered.
    unsafe { libc::kill(pid as libc::pid_t, 0) == 0 }
}

fn wait_for(mut pred: impl FnMut() -> bool, timeout: Duration) -> bool {
    let start = Instant::now();
    while start.elapsed() < timeout {
        if pred() {
            return true;
        }
        std::thread::sleep(Duration::from_millis(10));
    }
    pred()
}

#[test]
#[ignore = "needs qemu-system-aarch64; CI's qemu-vm job runs it"]
fn a_real_qemu_runs_only_while_the_boards_clock_advances() {
    let _suite = suite_lock();
    let image = CounterImage::write();
    let guest = CounterGuest::spawn(&image);
    let pid = guest.vm.pid();
    let shared = Arc::clone(&guest.monitor);
    let monitor = || shared.lock().unwrap();
    assert!(
        !monitor().query_status().expect("query-status").running,
        "a guest is born frozen"
    );
    let node = QemuNode::new(Box::new(guest), 115_200);
    let stats = node.stats();
    let (system, actor) = bench(node);
    let origin = virtual_clock::virtual_ns();
    let quantum_ns = DEFAULT_QUANTUM.as_nanos() as u64;

    // Held: the case is awake, so the board cannot advance and the guest
    // does not run, however long the host waits.
    std::thread::sleep(Duration::from_millis(300));
    assert_eq!(
        read_counter(&mut monitor()).0,
        0,
        "the guest ran with the board held"
    );

    // Two hundred quanta of the board's time. The slice due at the last
    // instant fires after the case parks again, so 199 have run.
    let wall = Instant::now();
    let quanta = 200;
    virtual_clock::wait_until_ns(origin + quanta * quantum_ns);
    let took = wall.elapsed();
    let (counter, frequency) = read_counter(&mut monitor());
    let lived = ticks_to_ns(counter, frequency);
    let board = stats.virtual_ns();
    assert_eq!(board, (quanta - 1) * quantum_ns);
    // Frozen again: the guest's counter stands while the case reads.
    std::thread::sleep(Duration::from_millis(100));
    assert_eq!(
        read_counter(&mut monitor()).0,
        counter,
        "the guest ran between slices"
    );

    eprintln!(
        "{} slices of {} ms in {:.3} s of host time ({:.3} ms a slice): the guest lived {:.3} ms \
         of the board's {:.3} ms by its own counter, the node booked {:.3} ms, {} slices \
         booked from the counter",
        stats.slices(),
        quantum_ns as f64 / 1e6,
        took.as_secs_f64(),
        took.as_secs_f64() * 1e3 / stats.slices() as f64,
        lived as f64 / 1e6,
        board as f64 / 1e6,
        stats.guest_ns() as f64 / 1e6,
        stats.clocked(),
    );
    // Each slice booked from the guest's own counter: the books are the
    // guest's clock, and the guest's clock is the board's, to a quantum.
    assert!(
        stats.clocked() + 1 >= stats.slices(),
        "{} of {} slices clocked",
        stats.clocked(),
        stats.slices()
    );
    assert!(
        lived.abs_diff(board) <= quantum_ns,
        "the guest lived {lived} ns of the board's {board} ns"
    );
    assert!(stats.failure().is_none(), "{:?}", stats.failure());

    drop(actor);
    system.shutdown();
    // The engine is joined, the node dropped and with it the guest, which
    // quits QEMU: no orphan.
    assert!(
        wait_for(|| !alive(pid), Duration::from_secs(5)),
        "QEMU (pid {pid}) outlived the node"
    );
}

/// Percentiles of `samples` in milliseconds: tenth, median, ninetieth.
fn spread(mut samples: Vec<f64>) -> (f64, f64, f64) {
    samples.sort_by(f64::total_cmp);
    let at = |q: f64| samples[((samples.len() - 1) as f64 * q).round() as usize];
    (at(0.1), at(0.5), at(0.9))
}

#[test]
#[ignore = "needs qemu-system-aarch64; CI's qemu-vm job runs it"]
fn the_metering_floor_is_below_the_default_quantum() {
    let _suite = suite_lock();
    // The node's own channel (`Guest::resume`, `Guest::pause`) for the
    // stops and conts, the control channel for reading the counter.
    let image = CounterImage::write();
    let mut node = QemuSpec::new(qemu_binary())
        .args(counter_args(&image.path()))
        .spawn()
        .expect("QEMU spawns");
    let mut monitor = node.control().expect("the monitor's QMP");
    // Let the guest store its frequency.
    node.resume().expect("cont");
    std::thread::sleep(Duration::from_millis(100));
    node.pause().expect("stop");
    let frequency = read_counter(&mut monitor).1;
    assert!(frequency > 0, "the guest never ran");

    // A stop and a cont with no window between them, a hundred and fifty
    // times: what the pair costs the host, and how long it lets the guest
    // live.
    let mut previous = read_counter(&mut monitor).0;
    let (mut lives, mut pairs) = (Vec::new(), Vec::new());
    for _ in 0..150 {
        let t = Instant::now();
        node.resume().expect("cont");
        node.pause().expect("stop");
        pairs.push(t.elapsed().as_secs_f64() * 1e3);
        let counter = read_counter(&mut monitor).0;
        lives.push(ticks_to_ns(counter - previous, frequency) as f64 / 1e6);
        previous = counter;
    }
    let (life_p10, floor, life_p90) = spread(lives);
    let (_, pair, pair_p90) = spread(pairs);
    eprintln!(
        "floor: a stop and cont back to back cost {pair:.3} ms of host time (ninetieth \
         percentile {pair_p90:.3} ms) and let the guest live {floor:.3} ms ({life_p10:.3}-\
         {life_p90:.3} ms, tenth to ninetieth percentile)"
    );
    // A window of `w`: how long the guest lives for it.
    for window_ms in [0.25, 0.5, 1.0, 2.0, 5.0] {
        let window = Duration::from_secs_f64(window_ms / 1e3);
        let mut lives = Vec::new();
        let mut previous = read_counter(&mut monitor).0;
        for _ in 0..60 {
            let t = Instant::now();
            node.resume().expect("cont");
            while t.elapsed() < window {
                std::hint::spin_loop();
            }
            node.pause().expect("stop");
            let counter = read_counter(&mut monitor).0;
            lives.push(ticks_to_ns(counter - previous, frequency) as f64 / 1e6);
            previous = counter;
        }
        let (p10, median, p90) = spread(lives);
        eprintln!(
            "window {window_ms:.2} ms: the guest lives {median:.3} ms ({p10:.3}-{p90:.3} ms)"
        );
    }
    // Frozen is frozen: half a second of host time moves nothing.
    let before = read_counter(&mut monitor).0;
    std::thread::sleep(Duration::from_millis(500));
    assert_eq!(
        read_counter(&mut monitor).0,
        before,
        "a stopped guest's clock moved"
    );

    let quantum_ms = DEFAULT_QUANTUM.as_secs_f64() * 1e3;
    assert!(
        floor < quantum_ms,
        "a slice of nothing lets the guest live {floor:.3} ms, longer than the default \
         quantum of {quantum_ms} ms: the default would not meter this host"
    );
}

#[test]
#[ignore = "needs qemu-system-aarch64; CI's qemu-vm job runs it"]
fn a_qemu_vm_from_a_project_boots_at_its_first_slice_and_is_metered() {
    let _suite = suite_lock();
    let counter = CounterImage::write();
    let image = counter.path();
    let text = format!(
        r#"
[[component]]
name = "PC"
kind = "qemu-vm"
[component.options]
baud = 115200
qemu = {qemu:?}
memory = "64M"
args = ["-device", "loader,file={image},addr={LOAD_AT:#x},cpu-num=0"]

[[wire]]
from = "BENCH.3V3"
to = "PC.VIO"
volts = 3.3

[[wire]]
from = "BENCH.GND"
to = "PC.GND"
volts = 0.0
"#,
        qemu = qemu_binary(),
        image = image.display(),
    );
    virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);
    let mut set = CatalogSet::new();
    embsim_qemu::catalog::register(&mut set).expect("the VM kinds join");
    let reports = Reports::new();
    let system = Project::parse(&text)
        .expect("the text is a project")
        .instantiate_with(&set, &reports)
        .expect("the project builds")
        .hold_time()
        .start()
        .expect("the bench starts");
    let mut reports = reports.take();
    let actor = virtual_clock::register_actor("qemu-vm-project-case");
    system.release_time();
    let mut said: Vec<String> = reports[0].look(0);
    virtual_clock::wait_virtual_ns(50 * DEFAULT_QUANTUM.as_nanos() as u64);
    said.extend(reports[0].look(virtual_clock::virtual_ns()));
    drop(actor);
    system.shutdown();
    let summary = reports[0].summary();
    eprintln!("{}\n{}", said.join("\n"), summary.join("\n"));
    assert_eq!(reports[0].failure(), None);
    assert!(
        said.iter()
            .any(|line| line.contains("the board's clock held at 1.000000 ms")),
        "{said:?}"
    );
    assert!(
        summary[0].starts_with("49 slices") || summary[0].starts_with("48 slices"),
        "{summary:?}"
    );
}
