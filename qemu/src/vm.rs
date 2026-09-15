//! QEMU as a [`Guest`]: a process born frozen, thawed and re-frozen over QMP.

use std::ffi::OsString;
use std::fmt;
use std::fs::{self, File};
use std::io::{self, BufRead, BufReader, Write};
use std::os::fd::{AsRawFd, RawFd};
use std::os::unix::net::UnixStream;
use std::path::{Path, PathBuf};
use std::process::{Child, Command, ExitStatus, Stdio};
use std::sync::atomic::{AtomicU64, Ordering};
use std::thread;
use std::time::{Duration, Instant};

use crate::guest::Guest;
use crate::qmp::{Qmp, QmpError};

/// File names inside the VM's working directory. Sockets are relative
/// because QEMU resolves chardev paths against its cwd, and a unix socket
/// path is capped at 104 bytes on macOS — a temp dir plus a name can blow
/// that.
const QMP_SOCKET: &str = "qmp.sock";
const CONTROL_SOCKET: &str = "ctl.sock";
const SERIAL_SOCKET: &str = "serial.sock";
const AGENT_SOCKET: &str = "agent.sock";
const OVERLAY_DISK: &str = "disk.qcow2";
const LOG_FILE: &str = "qemu.log";

/// The virtio-serial port name the in-guest agent listens on
/// (`/dev/virtio-ports/embsim.agent` on Linux).
pub const AGENT_PORT_NAME: &str = "embsim.agent";

/// How long [`QemuSpec::spawn`] waits for QEMU to open its QMP socket.
pub const DEFAULT_STARTUP_TIMEOUT: Duration = Duration::from_secs(10);

/// Bound on the node's own `stop`/`cont` round trips. The node holds virtual
/// time still for a slice plus two of these, and the engine's quiescence
/// barrier gives up on it after `System::quiescence_timeout` (5 s by
/// default) — a guest that cannot acknowledge a freeze inside a second is
/// not one the board can wait for.
const NODE_QMP_TIMEOUT: Duration = Duration::from_secs(1);

/// How long [`QemuVm::shutdown`] gives a `quit` before it kills the process.
const QUIT_GRACE: Duration = Duration::from_secs(3);

/// Bound on one agent clock query. The agent answers in tens of
/// microseconds; a guest that takes longer is not running (or has no agent).
const AGENT_TIMEOUT: Duration = Duration::from_millis(5);

/// Consecutive unanswered queries after which the agent is given up on, so a
/// guest without one costs nothing after its first few slices.
const AGENT_MISSES: u32 = 10;

/// Attempts at one `stop`/`cont` before the node gives up on the guest. A
/// round trip past [`NODE_QMP_TIMEOUT`] is a host hiccup, not a dead guest:
/// the command still lands, its late reply is skipped by id, and the retry
/// is idempotent (`stop` on a stopped guest, `cont` on a running one). A
/// loaded host has been seen to starve QEMU's main loop for several seconds
/// (measured: three 1 s timeouts in a row while another VM booted), so the
/// budget is twenty seconds — inside the 30 s quiescence budget a consumer
/// gives the engine when a computer is on the board.
const QMP_ATTEMPTS: u32 = 20;

/// How often [`QemuVm::run_until`] re-checks its predicate.
const WARMUP_POLL: Duration = Duration::from_millis(100);

/// Which emulated device carries the node's serial bytes into the guest.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SerialDevice {
    /// An FTDI FT232 on an xHCI controller. The guest sees a USB serial
    /// adapter (`/dev/ttyUSB0`, a COM port), and connecting to the chardev
    /// socket is plugging it in: QEMU attaches the device only while a
    /// client holds the socket, so the guest observes real USB attach and
    /// detach events. Needs a guest kernel with xHCI and `ftdi_sio`.
    UsbFtdi,
    /// The machine's first built-in UART (`-serial`), for guests without a
    /// USB stack. Always present; no plug/unplug semantics.
    Uart,
}

/// Why [`QemuSpec::spawn`] failed.
#[derive(Debug)]
pub enum SpawnError {
    /// The process could not be started or its working directory prepared.
    Io(io::Error),
    /// QEMU started but QMP never came up, or misbehaved when it did.
    Qmp(QmpError),
    /// QEMU exited before opening QMP; its output is in `log`.
    Exited {
        /// The exit status.
        status: ExitStatus,
        /// Where QEMU's stdout/stderr went.
        log: PathBuf,
    },
    /// QMP did not appear within the startup timeout; QEMU was killed.
    Timeout {
        /// Where QEMU's stdout/stderr went.
        log: PathBuf,
    },
}

impl fmt::Display for SpawnError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io(e) => write!(f, "spawning QEMU: {e}"),
            Self::Qmp(e) => write!(f, "QEMU's QMP: {e}"),
            Self::Exited { status, log } => {
                write!(
                    f,
                    "QEMU exited ({status}) before QMP came up; see {}",
                    log.display()
                )
            }
            Self::Timeout { log } => {
                write!(f, "QEMU did not open QMP in time; see {}", log.display())
            }
        }
    }
}

impl std::error::Error for SpawnError {}

impl From<io::Error> for SpawnError {
    fn from(e: io::Error) -> Self {
        Self::Io(e)
    }
}

/// How to launch QEMU.
///
/// The caller supplies the machine — binary, accelerator, memory, disk,
/// network — and the spec adds what the node needs: a frozen start (`-S`),
/// two QMP sockets (one for the node, one for the caller), the serial
/// chardev and its device, and no display or monitor.
///
/// ```no_run
/// # use embsim_qemu::QemuSpec;
/// let vm = QemuSpec::new("qemu-system-aarch64")
///     .args(["-M", "virt", "-accel", "hvf", "-cpu", "host", "-m", "4G"])
///     .overlay_of("guest.qcow2")
///     .agent(true)
///     .spawn()?;
/// # Ok::<(), embsim_qemu::SpawnError>(())
/// ```
#[derive(Debug, Clone)]
pub struct QemuSpec {
    binary: PathBuf,
    args: Vec<OsString>,
    serial: SerialDevice,
    agent: bool,
    overlay: Option<PathBuf>,
    startup_timeout: Duration,
}

impl QemuSpec {
    /// A spec for the given `qemu-system-*` binary (a name on `PATH` or a path).
    pub fn new(binary: impl Into<PathBuf>) -> Self {
        Self {
            binary: binary.into(),
            args: Vec::new(),
            serial: SerialDevice::UsbFtdi,
            agent: false,
            overlay: None,
            startup_timeout: DEFAULT_STARTUP_TIMEOUT,
        }
    }

    /// Append one command-line argument.
    pub fn arg(mut self, arg: impl Into<OsString>) -> Self {
        self.args.push(arg.into());
        self
    }

    /// Append several command-line arguments.
    pub fn args<I, S>(mut self, args: I) -> Self
    where
        I: IntoIterator<Item = S>,
        S: Into<OsString>,
    {
        self.args.extend(args.into_iter().map(Into::into));
        self
    }

    /// Which device carries the serial bytes (default: [`SerialDevice::UsbFtdi`]).
    pub fn serial(mut self, device: SerialDevice) -> Self {
        self.serial = device;
        self
    }

    /// Attach a virtio-serial port named [`AGENT_PORT_NAME`] for the in-guest
    /// agent that answers [`Guest::clock_ns`] (default: off).
    ///
    /// A guest without the agent still works — the node falls back to its
    /// stopwatch after a few unanswered queries — so this is safe to leave
    /// on for any Linux guest.
    pub fn agent(mut self, enabled: bool) -> Self {
        self.agent = enabled;
        self
    }

    /// Boot from a throw-away copy-on-write overlay of `base`, so the image
    /// on disk is never written to. The overlay lives in the VM's working
    /// directory and goes with it; needs `qemu-img` next to the binary or on
    /// `PATH`. Adds the disk as the first virtio block device.
    pub fn overlay_of(mut self, base: impl Into<PathBuf>) -> Self {
        self.overlay = Some(base.into());
        self
    }

    /// How long to wait for QMP after launch (default: [`DEFAULT_STARTUP_TIMEOUT`]).
    pub fn startup_timeout(mut self, timeout: Duration) -> Self {
        self.startup_timeout = timeout;
        self
    }

    /// The binary this spec launches.
    pub fn binary(&self) -> &Path {
        &self.binary
    }

    /// `qemu-img` from the same installation as the binary, else from `PATH`.
    fn qemu_img(&self) -> PathBuf {
        match self.binary.parent() {
            Some(dir) if !dir.as_os_str().is_empty() && dir.join("qemu-img").is_file() => {
                dir.join("qemu-img")
            }
            _ => PathBuf::from("qemu-img"),
        }
    }

    /// Launch QEMU frozen, wait for QMP, and plug the serial port in.
    ///
    /// The guest has executed nothing when this returns: `-S` holds the
    /// vCPUs until the node's first slice (or [`QemuVm::run_until`]).
    pub fn spawn(self) -> Result<QemuVm, SpawnError> {
        static COUNTER: AtomicU64 = AtomicU64::new(0);
        let workdir = std::env::temp_dir().join(format!(
            "embsim-qemu-{}-{}",
            std::process::id(),
            COUNTER.fetch_add(1, Ordering::Relaxed)
        ));
        fs::create_dir_all(&workdir)?;
        let log_path = workdir.join(LOG_FILE);
        let log = File::create(&log_path)?;

        if let Some(base) = &self.overlay {
            let base = fs::canonicalize(base)?;
            let output = Command::new(self.qemu_img())
                .current_dir(&workdir)
                .args(["create", "-q", "-f", "qcow2", "-F", "qcow2", "-b"])
                .arg(&base)
                .arg(OVERLAY_DISK)
                .output()?;
            if !output.status.success() {
                let _ = fs::remove_dir_all(&workdir);
                return Err(SpawnError::Io(io::Error::other(format!(
                    "qemu-img could not create an overlay of {}: {}",
                    base.display(),
                    String::from_utf8_lossy(&output.stderr).trim()
                ))));
            }
        }

        let mut command = Command::new(&self.binary);
        command
            .current_dir(&workdir)
            .args(&self.args)
            .args(["-S", "-display", "none", "-monitor", "none"])
            .args(["-qmp", &format!("unix:{QMP_SOCKET},server=on,wait=off")])
            .args(["-qmp", &format!("unix:{CONTROL_SOCKET},server=on,wait=off")])
            .args([
                "-chardev",
                &format!("socket,id=embsim-serial,path={SERIAL_SOCKET},server=on,wait=off"),
            ]);
        if self.overlay.is_some() {
            command.args([
                "-drive",
                &format!("file={OVERLAY_DISK},if=virtio,format=qcow2"),
            ]);
        }
        match self.serial {
            SerialDevice::UsbFtdi => command.args([
                "-device",
                "qemu-xhci,id=embsim-xhci",
                "-device",
                "usb-serial,chardev=embsim-serial,bus=embsim-xhci.0",
            ]),
            SerialDevice::Uart => command.args(["-serial", "chardev:embsim-serial"]),
        };
        if self.agent {
            command
                .args(["-device", "virtio-serial-pci,id=embsim-vser"])
                .args([
                    "-chardev",
                    &format!("socket,id=embsim-agent,path={AGENT_SOCKET},server=on,wait=off"),
                ])
                .args([
                    "-device",
                    &format!(
                        "virtserialport,chardev=embsim-agent,name={AGENT_PORT_NAME},bus=embsim-vser.0"
                    ),
                ]);
        }
        command
            .stdin(Stdio::null())
            .stdout(Stdio::from(log.try_clone()?))
            .stderr(Stdio::from(log));
        let mut child = command.spawn()?;
        tracing::info!(pid = child.id(), workdir = %workdir.display(), "QEMU launched (frozen)");

        // Wait for QEMU to open QMP. This is process warm-up on the host, not
        // a simulated wait, so a plain sleep between attempts is the honest
        // tool (the virtual clock has nothing to do with it).
        let deadline = Instant::now() + self.startup_timeout;
        let qmp = loop {
            if let Some(status) = child.try_wait()? {
                return Err(SpawnError::Exited {
                    status,
                    log: log_path,
                });
            }
            match Qmp::connect_with_timeout(&workdir.join(QMP_SOCKET), NODE_QMP_TIMEOUT) {
                Ok(qmp) => break qmp,
                Err(QmpError::Io(e))
                    if matches!(
                        e.kind(),
                        io::ErrorKind::NotFound | io::ErrorKind::ConnectionRefused
                    ) =>
                {
                    if Instant::now() >= deadline {
                        let _ = child.kill();
                        let _ = child.wait();
                        return Err(SpawnError::Timeout { log: log_path });
                    }
                    thread::sleep(Duration::from_millis(20));
                }
                Err(e) => {
                    let _ = child.kill();
                    let _ = child.wait();
                    return Err(SpawnError::Qmp(e));
                }
            }
        };

        // Plug the serial port in. For the USB device this is the moment the
        // guest will see an attach once it runs.
        let connect = |name: &str| -> io::Result<UnixStream> {
            let s = UnixStream::connect(workdir.join(name))?;
            s.set_nonblocking(true)?;
            Ok(s)
        };
        let serial = match connect(SERIAL_SOCKET) {
            Ok(serial) => serial,
            Err(e) => {
                let _ = child.kill();
                let _ = child.wait();
                return Err(SpawnError::Io(e));
            }
        };
        let agent = if self.agent {
            match AgentLink::connect(&workdir.join(AGENT_SOCKET)) {
                Ok(link) => Some(link),
                Err(e) => {
                    let _ = child.kill();
                    let _ = child.wait();
                    return Err(SpawnError::Io(e));
                }
            }
        } else {
            None
        };

        Ok(QemuVm {
            child,
            workdir,
            qmp,
            serial,
            agent,
        })
    }
}

/// The host end of the in-guest agent's virtio-serial port.
///
/// One query is one line: `t <seq>`; the answer is `<seq> <monotonic ns>`.
/// The sequence number lets a reply that arrives after its query timed out
/// be recognised and discarded instead of being taken for the next answer.
struct AgentLink {
    writer: UnixStream,
    reader: BufReader<UnixStream>,
    seq: u64,
    misses: u32,
    line: String,
}

impl AgentLink {
    fn connect(path: &Path) -> io::Result<Self> {
        let writer = UnixStream::connect(path)?;
        writer.set_read_timeout(Some(AGENT_TIMEOUT))?;
        writer.set_write_timeout(Some(AGENT_TIMEOUT))?;
        let reader = BufReader::new(writer.try_clone()?);
        Ok(Self {
            writer,
            reader,
            seq: 0,
            misses: 0,
            line: String::new(),
        })
    }

    /// Ask the guest for its clock. `None` on timeout or garbage.
    fn clock_ns(&mut self) -> Option<u64> {
        self.seq += 1;
        let seq = self.seq;
        if self
            .writer
            .write_all(format!("t {seq}\n").as_bytes())
            .is_err()
        {
            return None;
        }
        loop {
            self.line.clear();
            match self.reader.read_line(&mut self.line) {
                Ok(0) | Err(_) => return None,
                Ok(_) => {}
            }
            let mut parts = self.line.split_whitespace();
            let (Some(got_seq), Some(ns)) = (parts.next(), parts.next()) else {
                return None;
            };
            if got_seq.parse::<u64>().ok() != Some(seq) {
                continue; // a late answer to an earlier query
            }
            return ns.parse().ok();
        }
    }
}

/// A running QEMU process, frozen unless a node is running it.
///
/// Dropping it quits QEMU (killing it if `quit` is not honoured promptly)
/// and removes the working directory.
pub struct QemuVm {
    child: Child,
    workdir: PathBuf,
    qmp: Qmp,
    serial: UnixStream,
    agent: Option<AgentLink>,
}

impl fmt::Debug for QemuVm {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("QemuVm")
            .field("pid", &self.child.id())
            .field("workdir", &self.workdir)
            .field("agent", &self.agent.is_some())
            .finish()
    }
}

impl QemuVm {
    /// A second, independent QMP connection for the caller's own use —
    /// port forwards, device hot-plug, `query-*` — so it never contends with
    /// the node's stop/cont channel.
    pub fn control(&self) -> Result<Qmp, QmpError> {
        Qmp::connect(&self.workdir.join(CONTROL_SOCKET))
    }

    /// The QEMU process id.
    pub fn pid(&self) -> u32 {
        self.child.id()
    }

    /// The directory holding the sockets and the log.
    pub fn workdir(&self) -> &Path {
        &self.workdir
    }

    /// Where QEMU's stdout and stderr go.
    pub fn log_path(&self) -> PathBuf {
        self.workdir.join(LOG_FILE)
    }

    /// Whether the in-guest agent is attached and still answering.
    pub fn has_agent(&self) -> bool {
        self.agent.is_some()
    }

    /// Let the guest run on host time until `ready` returns true, then
    /// freeze it again. Returns how long it ran.
    ///
    /// This is how a guest is brought to a useful state — an OS booted, a
    /// browser listening — before the board's clock takes over: those tens of
    /// seconds are nobody's simulated time, and metering them through the
    /// board would cost thousands of slices for nothing. Fails with
    /// [`io::ErrorKind::TimedOut`] (guest frozen again) if `ready` never
    /// holds, or with the QEMU error if the process dies.
    pub fn run_until(
        &mut self,
        mut ready: impl FnMut() -> bool,
        timeout: Duration,
    ) -> io::Result<Duration> {
        self.qmp.cont()?;
        let start = Instant::now();
        let outcome = loop {
            if let Some(status) = self.child.try_wait()? {
                break Err(io::Error::other(format!(
                    "QEMU exited ({status}) during warm-up"
                )));
            }
            if ready() {
                break Ok(start.elapsed());
            }
            if start.elapsed() >= timeout {
                break Err(io::Error::new(
                    io::ErrorKind::TimedOut,
                    format!("guest not ready after {timeout:?}"),
                ));
            }
            // Host-time warm-up, not a simulated wait.
            thread::sleep(WARMUP_POLL);
        };
        self.qmp.stop()?;
        outcome
    }

    /// Ask QEMU to quit and wait for it, killing it after a grace period.
    pub fn shutdown(&mut self) {
        if self.child.try_wait().ok().flatten().is_some() {
            return;
        }
        let _ = self.qmp.quit();
        let deadline = Instant::now() + QUIT_GRACE;
        while Instant::now() < deadline {
            if self.child.try_wait().ok().flatten().is_some() {
                return;
            }
            // Host process teardown, not a simulated wait.
            thread::sleep(Duration::from_millis(20));
        }
        tracing::warn!(pid = self.child.id(), "QEMU ignored quit; killing it");
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}

impl QemuVm {
    /// Run a freeze/thaw command, retrying a timed-out round trip.
    fn run_state_command(&mut self, what: &str) -> io::Result<()> {
        let mut attempt = 1;
        loop {
            let result = match what {
                "stop" => self.qmp.stop(),
                _ => self.qmp.cont(),
            };
            match result {
                Ok(()) => return Ok(()),
                Err(QmpError::Io(e))
                    if matches!(
                        e.kind(),
                        io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
                    ) && attempt < QMP_ATTEMPTS =>
                {
                    tracing::warn!(what, attempt, "QMP round trip timed out; retrying");
                    attempt += 1;
                }
                Err(e) => return Err(io::Error::from(e)),
            }
        }
    }
}

impl Guest for QemuVm {
    fn resume(&mut self) -> io::Result<()> {
        self.run_state_command("cont")
    }

    fn pause(&mut self) -> io::Result<()> {
        self.run_state_command("stop")
    }

    fn serial_fd(&self) -> RawFd {
        self.serial.as_raw_fd()
    }

    fn clock_ns(&mut self) -> Option<u64> {
        let link = self.agent.as_mut()?;
        match link.clock_ns() {
            Some(ns) => {
                link.misses = 0;
                Some(ns)
            }
            None => {
                link.misses += 1;
                if link.misses >= AGENT_MISSES {
                    tracing::warn!(
                        "qemu guest: no agent answered {AGENT_MISSES} clock queries; \
                         falling back to the stopwatch for the rest of the run"
                    );
                    self.agent = None;
                }
                None
            }
        }
    }
}

impl Drop for QemuVm {
    fn drop(&mut self) {
        self.shutdown();
        let _ = fs::remove_dir_all(&self.workdir);
    }
}
