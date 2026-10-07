//! `qemu-system-p2`, the program: finding it, starting it, the handshake,
//! one turn at a time, and making sure it never outlives the node.
//!
//! # Finding it
//!
//! [`QemuSystemP2::find`] looks in three places, in order: the file
//! `EMBSIM_QEMU_SYSTEM_P2` names; `qemu-system-p2` on `PATH`; and where
//! `embsim qemu install` puts it, `~/.embsim/qemu/<identity>/` — the
//! [`crate::target::identity`] of the target this crate carries, so two
//! embsims whose targets differ keep two programs side by side.
//!
//! # Starting it
//!
//! The program is started with the node's own QEMU arguments and the
//! machine property `hostipc=<shm|sock>:<channel-fd>:<watch-fd>:<spin-ns>`
//! (`qemu-target/target-p2/hostipc.h`). Three descriptors are handed down
//! and nothing reaches the file system: the channel (a [`ShmPage`] or one
//! end of a socket pair), the read end of a pipe only this process holds
//! the write end of, and the boot ROM, an unlinked file read as
//! `/dev/fd/N`. The program goes into a process group of its own, so a
//! terminal's ^C reaches embsim, whose run ends cleanly, and not it.
//!
//! # Lifetime
//!
//! The program never outlives the node: dropping the [`Peer`] asks it to
//! quit and kills it if it has not within a second, and if embsim itself
//! dies — however it dies — the pipe's write end closes and the program's
//! watch thread ends it at once.
//!
//! # A program that dies
//!
//! A turn waits on the program with a bounded block: every 100 ms it asks
//! the kernel whether the program still lives, and a program that exited is
//! reported with its exit status and the last of what it wrote to standard
//! error ([`P2QemuError::Died`]); on the socket the closed connection says
//! so at once. A program that lives but has not answered a turn in
//! [`TURN_TIMEOUT`] is reported as unresponsive and killed.

use std::collections::VecDeque;
use std::ffi::OsString;
use std::io::{self, Read, Write};
use std::os::fd::{AsRawFd, OwnedFd};
use std::os::unix::net::UnixStream;
use std::os::unix::process::{CommandExt, ExitStatusExt};
use std::path::{Path, PathBuf};
use std::process::{Child, Command, ExitStatus, Stdio};
use std::sync::atomic::Ordering;
use std::sync::{Arc, Condvar, Mutex};
use std::time::{Duration, Instant};

use crate::protocol::{self, shm, Hello, Run, ShmPage, StopHeader, OP_QUIT};
use crate::{target, P2QemuError};

/// The variable naming the `qemu-system-p2` to run, ahead of `PATH`.
pub const PROGRAM_VAR: &str = "EMBSIM_QEMU_SYSTEM_P2";

/// How long the program may take to start and say hello. The first start
/// of a newly built binary on macOS is a security scan of it: hundreds of
/// milliseconds, seconds on a loaded machine.
pub const START_TIMEOUT: Duration = Duration::from_secs(30);

/// How long one turn may take before the program is called unresponsive.
/// A turn runs at most the node's slice of virtual time (`SLICE_NS`), which
/// is milliseconds of the program's own time even with every instruction
/// traced.
pub const TURN_TIMEOUT: Duration = Duration::from_secs(30);

/// How often a blocked wait asks whether the program still lives.
const LIVENESS: Duration = Duration::from_millis(100);

/// How long a program asked to quit is given before it is killed.
const QUIT_GRACE: Duration = Duration::from_secs(1);

/// How much of the program's standard error an error quotes.
const TAIL_BYTES: usize = 4096;

/// What the program's own arguments always are: the P2 machine under
/// `-icount`, so one instruction is one unit of its budget, honoured
/// exactly, and nothing attached to the host.
const QEMU_ARGS: [&str; 12] = [
    "-accel",
    "tcg",
    "-icount",
    "shift=0,sleep=off",
    "-display",
    "none",
    "-monitor",
    "none",
    "-serial",
    "none",
    "-parallel",
    "none",
];

// ============================================================
// Finding it
// ============================================================

/// Where a [`QemuSystemP2`] was found.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Found {
    /// Named by the caller.
    Given,
    /// Named by `EMBSIM_QEMU_SYSTEM_P2`.
    Variable,
    /// On `PATH`.
    OnPath,
    /// Where `embsim qemu install` puts it.
    Installed,
}

impl Found {
    /// Where, in words: `"on PATH"`.
    pub fn describe(self) -> &'static str {
        match self {
            Found::Given => "as given",
            Found::Variable => "from EMBSIM_QEMU_SYSTEM_P2",
            Found::OnPath => "on PATH",
            Found::Installed => "installed by `embsim qemu install`",
        }
    }
}

/// A `qemu-system-p2` the node can start.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct QemuSystemP2 {
    path: PathBuf,
    found: Found,
}

/// Where `embsim qemu install` puts the program for the target this crate
/// carries: `~/.embsim/qemu/<identity>/`. `None` without a home directory.
pub fn install_dir() -> Option<PathBuf> {
    let home = std::env::var_os("HOME").filter(|h| !h.is_empty())?;
    Some(install_dir_in(Path::new(&home)))
}

fn install_dir_in(home: &Path) -> PathBuf {
    home.join(".embsim").join("qemu").join(target::identity())
}

/// How to get the program, said the same way by every error that lacks it.
pub fn install_advice() -> String {
    let pin = target::qemu_pin();
    format!(
        "`embsim qemu install` builds it — QEMU {} with the P2 target this embsim carries \
         (identity {}) — and installs it where this embsim looks ({}); it needs git, a C \
         compiler, ninja, pkg-config, glib and python3",
        pin.tag,
        target::identity(),
        install_dir().map_or_else(
            || "~/.embsim/qemu/<identity>/".to_string(),
            |dir| dir.display().to_string()
        ),
    )
}

fn runnable(path: &Path) -> bool {
    use std::os::unix::fs::PermissionsExt;
    std::fs::metadata(path).is_ok_and(|m| m.is_file() && m.permissions().mode() & 0o111 != 0)
}

impl QemuSystemP2 {
    /// The program's file name.
    pub const NAME: &'static str = "qemu-system-p2";

    /// The program at `path`, as given.
    pub fn at(path: impl Into<PathBuf>) -> Self {
        Self {
            path: path.into(),
            found: Found::Given,
        }
    }

    /// Find the program: `EMBSIM_QEMU_SYSTEM_P2`, then `PATH`, then
    /// [`install_dir`]. An error says where it looked and how to install it.
    pub fn find() -> Result<Self, P2QemuError> {
        Self::find_in(
            std::env::var_os(PROGRAM_VAR),
            std::env::var_os("PATH"),
            install_dir(),
        )
    }

    /// [`Self::find`] over the variable, the `PATH` and the install
    /// directory given.
    pub fn find_in(
        variable: Option<OsString>,
        path: Option<OsString>,
        installed: Option<PathBuf>,
    ) -> Result<Self, P2QemuError> {
        if let Some(named) = variable.filter(|v| !v.is_empty()) {
            let named = PathBuf::from(named);
            if runnable(&named) {
                return Ok(Self {
                    path: named,
                    found: Found::Variable,
                });
            }
            return Err(P2QemuError::NotFound(format!(
                "{PROGRAM_VAR} names {}, which is not a program this process can run; point it \
                 at a qemu-system-p2, or unset it to use the one on PATH or the installed one. {}",
                named.display(),
                install_advice()
            )));
        }
        if let Some(path) = &path {
            for dir in std::env::split_paths(path) {
                let candidate = dir.join(Self::NAME);
                if runnable(&candidate) {
                    return Ok(Self {
                        path: candidate,
                        found: Found::OnPath,
                    });
                }
            }
        }
        let looked_installed = match &installed {
            Some(dir) => {
                let candidate = dir.join(Self::NAME);
                if runnable(&candidate) {
                    return Ok(Self {
                        path: candidate,
                        found: Found::Installed,
                    });
                }
                format!("none at {}", candidate.display())
            }
            None => "no home directory to look in for an installed one".to_string(),
        };
        Err(P2QemuError::NotFound(format!(
            "no qemu-system-p2: {PROGRAM_VAR} is unset, none is on PATH, and {looked_installed}. \
             {}",
            install_advice()
        )))
    }

    /// The program's path.
    pub fn path(&self) -> &Path {
        &self.path
    }

    /// Where it was found.
    pub fn found(&self) -> Found {
        self.found
    }

    /// Start the program, take its hello, and stop it: what it says it is,
    /// whether or not that is what this crate needs (`embsim qemu path`).
    pub fn probe(&self) -> Result<Identity, P2QemuError> {
        let (_peer, identity) = Peer::start_unchecked(self, &[], &[], Transport::default())?;
        Ok(identity)
    }

    /// The fix for a program that is not the one this crate needs, by where
    /// it was found.
    fn wrong_program_advice(&self) -> String {
        let instead = match self.found {
            Found::Given => "start the matching one".to_string(),
            Found::Variable => format!("point {PROGRAM_VAR} at the matching one"),
            Found::OnPath => format!(
                "this one comes first on PATH: set {PROGRAM_VAR} to the matching one, or take {} \
                 off PATH",
                self.path.parent().unwrap_or(&self.path).display()
            ),
            Found::Installed => "install it again".to_string(),
        };
        format!("{instead}. {}", install_advice())
    }
}

/// What a program says it is in its hello.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Identity {
    /// The protocol it speaks.
    pub protocol: u32,
    /// The identity of the target sources it was built from.
    pub target: String,
    /// The QEMU it is, `10.1.0`.
    pub qemu: String,
}

impl Identity {
    /// What this crate needs: its protocol, its target, its QEMU.
    pub fn needed() -> Self {
        Self {
            protocol: protocol::PROTOCOL,
            target: target::identity().to_string(),
            qemu: target::qemu_pin().version().to_string(),
        }
    }

    /// Whether `self` is what this crate needs, and if not, what differs.
    pub fn check(&self) -> Result<(), String> {
        let needed = Self::needed();
        if self.protocol != needed.protocol {
            return Err(format!(
                "it speaks protocol {} and this embsim speaks protocol {}",
                self.protocol, needed.protocol
            ));
        }
        let mut differs = Vec::new();
        if self.target != needed.target {
            differs.push(format!(
                "it was built from P2 target {} and this embsim carries target {}",
                self.target, needed.target
            ));
        }
        if self.qemu != needed.qemu {
            differs.push(format!(
                "it is QEMU {} and this embsim's target is built into QEMU {}",
                self.qemu, needed.qemu
            ));
        }
        if differs.is_empty() {
            Ok(())
        } else {
            Err(differs.join("; "))
        }
    }
}

impl std::fmt::Display for Identity {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "protocol {}, P2 target {}, QEMU {}",
            self.protocol, self.target, self.qemu
        )
    }
}

// ============================================================
// The channel
// ============================================================

/// How the node and the program hand turns across.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Transport {
    /// A shared page: each side spins `spin_ns` for the other's turn, then
    /// blocks on a futex. A turn costs a fraction of a microsecond while
    /// both sides' turns are shorter than the spin; the cost is a second
    /// core spinning while they are.
    Shm {
        /// How long either side spins before it blocks.
        spin_ns: u64,
    },
    /// A Unix socket pair, blocking: a turn costs a wake-up each way
    /// (about 5 us). The fallback.
    Socket,
}

impl Default for Transport {
    fn default() -> Self {
        if cfg!(any(target_os = "macos", target_os = "linux")) {
            Transport::Shm { spin_ns: 20_000 }
        } else {
            Transport::Socket
        }
    }
}

impl Transport {
    /// The variable choosing the transport: `shm` (the default) or
    /// `socket`.
    pub const VAR: &'static str = "EMBSIM_P2_QEMU_TRANSPORT";
    /// The variable setting the shared page's spin, in nanoseconds.
    pub const SPIN_VAR: &'static str = "EMBSIM_P2_QEMU_SPIN_NS";

    /// The transport the environment chooses, [`Transport::default`]
    /// without one.
    pub fn from_env() -> Result<Self, P2QemuError> {
        let spin_ns = match std::env::var(Self::SPIN_VAR) {
            Ok(text) => text.trim().parse::<u64>().map_err(|_| {
                P2QemuError::NotFound(format!(
                    "{}={text:?} is not a whole number of nanoseconds",
                    Self::SPIN_VAR
                ))
            })?,
            Err(_) => 20_000,
        };
        match std::env::var(Self::VAR).as_deref() {
            Err(_) | Ok("") => Ok(match Self::default() {
                Transport::Shm { .. } => Transport::Shm { spin_ns },
                other => other,
            }),
            Ok("shm") => Ok(Transport::Shm { spin_ns }),
            Ok("socket") => Ok(Transport::Socket),
            Ok(other) => Err(P2QemuError::NotFound(format!(
                "{}={other:?}: the transports are `shm` and `socket`",
                Self::VAR
            ))),
        }
    }
}

enum Channel {
    Shm {
        page: ShmPage,
        seq: u32,
        spin: Duration,
    },
    Socket {
        stream: UnixStream,
    },
}

// ============================================================
// Standard error, kept
// ============================================================

/// The program's standard error: passed through to this process's, and the
/// last few kilobytes kept for an error to quote.
#[derive(Default)]
struct Stderr {
    state: Mutex<(VecDeque<u8>, bool)>,
    closed: Condvar,
}

impl Stderr {
    fn start(mut pipe: std::process::ChildStderr) -> Arc<Self> {
        let me = Arc::new(Self::default());
        let mine = Arc::clone(&me);
        let spawned = std::thread::Builder::new()
            .name("qemu-system-p2 stderr".to_string())
            .spawn(move || {
                let mut chunk = [0u8; 4096];
                loop {
                    let n = match pipe.read(&mut chunk) {
                        Ok(0) | Err(_) => break,
                        Ok(n) => n,
                    };
                    let _ = io::stderr().write_all(&chunk[..n]);
                    let mut state = mine.state.lock().expect("stderr tail never poisoned");
                    state.0.extend(&chunk[..n]);
                    let excess = state.0.len().saturating_sub(TAIL_BYTES);
                    state.0.drain(..excess);
                }
                mine.state.lock().expect("stderr tail never poisoned").1 = true;
                mine.closed.notify_all();
            });
        if spawned.is_err() {
            me.state.lock().expect("stderr tail never poisoned").1 = true;
        }
        me
    }

    /// The last lines the program wrote, once its standard error closed
    /// (or a moment has passed): the tail an error quotes.
    fn tail(&self) -> String {
        let deadline = Instant::now() + Duration::from_millis(500);
        let mut state = self.state.lock().expect("stderr tail never poisoned");
        while !state.1 {
            let left = deadline.saturating_duration_since(Instant::now());
            if left.is_zero() {
                break;
            }
            state = self
                .closed
                .wait_timeout(state, left)
                .expect("stderr tail never poisoned")
                .0;
        }
        let bytes: Vec<u8> = state.0.iter().copied().collect();
        let text = String::from_utf8_lossy(&bytes);
        let lines: Vec<&str> = text.lines().filter(|l| !l.trim().is_empty()).collect();
        lines[lines.len().saturating_sub(12)..].join("\n")
    }
}

/// An exit status in words, its signal named.
fn status_text(status: ExitStatus) -> String {
    if let Some(code) = status.code() {
        return format!("exited with status {code}");
    }
    match status.signal() {
        Some(signal) => {
            let name = match signal {
                libc::SIGHUP => " (SIGHUP)",
                libc::SIGINT => " (SIGINT)",
                libc::SIGQUIT => " (SIGQUIT)",
                libc::SIGILL => " (SIGILL)",
                libc::SIGTRAP => " (SIGTRAP)",
                libc::SIGABRT => " (SIGABRT)",
                libc::SIGBUS => " (SIGBUS)",
                libc::SIGFPE => " (SIGFPE)",
                libc::SIGKILL => " (SIGKILL)",
                libc::SIGSEGV => " (SIGSEGV)",
                libc::SIGPIPE => " (SIGPIPE)",
                libc::SIGTERM => " (SIGTERM)",
                _ => "",
            };
            format!("was killed by signal {signal}{name}")
        }
        None => format!("ended ({status})"),
    }
}

// ============================================================
// The peer
// ============================================================

/// A running `qemu-system-p2` in host-driven mode, and the channel to it.
pub struct Peer {
    program: QemuSystemP2,
    child: Child,
    pid: u32,
    channel: Channel,
    /// The write end of the watch pipe: when it closes, the program exits.
    _watch: OwnedFd,
    stderr: Arc<Stderr>,
    /// The last reply's tail.
    tail: Vec<u8>,
    /// Set once the program is known to be gone.
    gone: bool,
}

impl std::fmt::Debug for Peer {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Peer")
            .field("program", &self.program.path)
            .field("pid", &self.pid)
            .finish()
    }
}

/// A pipe, both ends close-on-exec from the start where the system can
/// (`pipe2` on Linux), so no other child started meanwhile inherits the
/// write end and keeps the program's watch from ever reading end of file.
fn pipe() -> io::Result<(OwnedFd, OwnedFd)> {
    let (read, write) = io::pipe()?;
    Ok((OwnedFd::from(read), OwnedFd::from(write)))
}

impl Peer {
    /// Start `program` with `rom` as its boot ROM and `extra_args` after
    /// the node's own, and return once its hello says it is the program
    /// this crate needs.
    pub fn start(
        program: &QemuSystemP2,
        rom: &[u8],
        extra_args: &[&str],
        transport: Transport,
    ) -> Result<Self, P2QemuError> {
        let (peer, identity) = Self::start_unchecked(program, rom, extra_args, transport)?;
        if let Err(why) = identity.check() {
            return Err(P2QemuError::Refused {
                program: program.path.clone(),
                why: format!(
                    "{} ({}) is not the qemu-system-p2 this embsim needs: {why}; {}",
                    program.path.display(),
                    program.found.describe(),
                    program.wrong_program_advice()
                ),
            });
        }
        Ok(peer)
    }

    fn start_unchecked(
        program: &QemuSystemP2,
        rom: &[u8],
        extra_args: &[&str],
        transport: Transport,
    ) -> Result<(Self, Identity), P2QemuError> {
        let rom_fd = protocol::anonymous_file("rom", rom)?;
        let (watch_read, watch_write) = pipe()?;
        let (channel, theirs, kind, spin_ns) = match transport {
            Transport::Shm { spin_ns } => {
                let (page, fd) = ShmPage::create()?;
                (
                    Channel::Shm {
                        page,
                        seq: 0,
                        spin: Duration::from_nanos(spin_ns),
                    },
                    fd,
                    "shm",
                    spin_ns,
                )
            }
            Transport::Socket => {
                let (ours, theirs) = UnixStream::pair()?;
                (
                    Channel::Socket { stream: ours },
                    OwnedFd::from(theirs),
                    "sock",
                    0,
                )
            }
        };
        let handed = [
            theirs.as_raw_fd(),
            watch_read.as_raw_fd(),
            rom_fd.as_raw_fd(),
        ];

        let mut command = Command::new(&program.path);
        command
            .arg("-M")
            .arg(format!(
                "p2,hostipc={kind}:{}:{}:{spin_ns}",
                handed[0], handed[1]
            ))
            .args(QEMU_ARGS)
            .arg("-bios")
            .arg(format!("/dev/fd/{}", handed[2]))
            .args(extra_args)
            .stdin(Stdio::null())
            .stdout(Stdio::null())
            .stderr(Stdio::piped());
        // SAFETY: only async-signal-safe calls, between fork and exec: a
        // process group of its own, and the three descriptors kept open
        // across the exec.
        unsafe {
            command.pre_exec(move || {
                libc::setpgid(0, 0);
                for fd in handed {
                    let flags = libc::fcntl(fd, libc::F_GETFD);
                    if flags < 0 || libc::fcntl(fd, libc::F_SETFD, flags & !libc::FD_CLOEXEC) < 0 {
                        return Err(io::Error::last_os_error());
                    }
                }
                Ok(())
            })
        };
        let mut child = command.spawn().map_err(|error| P2QemuError::Start {
            program: program.path.clone(),
            error,
        })?;
        // The program's copies are its own now.
        drop((theirs, watch_read, rom_fd));
        let stderr = Stderr::start(child.stderr.take().expect("stderr is piped"));
        let pid = child.id();
        let mut peer = Self {
            program: program.clone(),
            child,
            pid,
            channel,
            _watch: watch_write,
            stderr,
            tail: Vec::with_capacity(4096),
            gone: false,
        };
        let hello = peer.hello()?;
        if hello.magic != protocol::MAGIC {
            return Err(P2QemuError::Refused {
                program: program.path.clone(),
                why: format!(
                    "{} ({}) answered with something that is not embsim's handshake (magic \
                     {:#010x}); {}",
                    program.path.display(),
                    program.found.describe(),
                    hello.magic,
                    program.wrong_program_advice()
                ),
            });
        }
        let identity = Identity {
            protocol: hello.protocol,
            target: hello.target,
            qemu: hello.qemu,
        };
        tracing::info!(
            program = %program.path.display(),
            pid,
            %identity,
            "p2-qemu: qemu-system-p2 answered"
        );
        Ok((peer, identity))
    }

    /// The program's process id.
    pub fn pid(&self) -> u32 {
        self.pid
    }

    /// The program.
    pub fn program(&self) -> &QemuSystemP2 {
        &self.program
    }

    /// Whether the program has exited; reaps it if so.
    fn exited(&mut self) -> Option<ExitStatus> {
        match self.child.try_wait() {
            Ok(Some(status)) => {
                self.gone = true;
                Some(status)
            }
            _ => None,
        }
    }

    /// The program is gone or going: wait for its status (killing it if
    /// it lingers) and say what happened `during`.
    fn died(&mut self, during: &str) -> P2QemuError {
        let deadline = Instant::now() + QUIT_GRACE;
        let status = loop {
            if let Some(status) = self.exited() {
                break Some(status);
            }
            if Instant::now() >= deadline {
                let _ = self.child.kill();
                break self.child.wait().ok();
            }
            std::thread::sleep(Duration::from_millis(2));
        };
        self.gone = true;
        P2QemuError::Died {
            program: self.program.path.clone(),
            pid: self.pid,
            status: status.map_or_else(|| "ended".to_string(), status_text),
            during: during.to_string(),
            stderr: self.stderr.tail(),
        }
    }

    fn unresponsive(&mut self, during: &str, waited: Duration) -> P2QemuError {
        let _ = self.child.kill();
        let _ = self.child.wait();
        self.gone = true;
        P2QemuError::Unresponsive {
            program: self.program.path.clone(),
            pid: self.pid,
            waited,
            during: during.to_string(),
        }
    }

    fn hello(&mut self) -> Result<Hello, P2QemuError> {
        const DURING: &str = "before it said hello";
        let started = Instant::now();
        let mut bytes = [0u8; Hello::LEN];
        match &mut self.channel {
            Channel::Shm { page, .. } => loop {
                if page.word(shm::READY).load(Ordering::Acquire) != 0 {
                    page.read(shm::HELLO, &mut bytes);
                    break;
                }
                protocol::futex_wait(page.word(shm::READY), 0, Duration::from_millis(10));
                if page.word(shm::READY).load(Ordering::Acquire) != 0 {
                    continue;
                }
                if self.child.try_wait().ok().flatten().is_some() {
                    return Err(self.died(DURING));
                }
                if started.elapsed() > START_TIMEOUT {
                    return Err(self.unresponsive(DURING, START_TIMEOUT));
                }
            },
            Channel::Socket { stream } => {
                stream.set_read_timeout(Some(LIVENESS))?;
                let mut got = 0;
                while got < bytes.len() {
                    match stream.read(&mut bytes[got..]) {
                        Ok(0) => return Err(self.died(DURING)),
                        Ok(n) => got += n,
                        Err(e)
                            if matches!(
                                e.kind(),
                                io::ErrorKind::WouldBlock
                                    | io::ErrorKind::TimedOut
                                    | io::ErrorKind::Interrupted
                            ) =>
                        {
                            if self.child.try_wait().ok().flatten().is_some() {
                                return Err(self.died(DURING));
                            }
                            if started.elapsed() > START_TIMEOUT {
                                return Err(self.unresponsive(DURING, START_TIMEOUT));
                            }
                        }
                        Err(_) => return Err(self.died(DURING)),
                    }
                }
            }
        }
        Ok(Hello::decode(&bytes))
    }

    /// One turn: send `run`, wait for the reply. The header is returned and
    /// its tail is [`Self::tail`] until the next turn.
    pub fn turn(&mut self, run: &Run) -> Result<StopHeader, P2QemuError> {
        const DURING: &str = "during a run";
        if self.gone {
            return Err(self.died(DURING));
        }
        let started = Instant::now();
        let request = run.encode();
        let mut header = [0u8; StopHeader::LEN];
        match &mut self.channel {
            Channel::Shm { page, seq, spin } => {
                page.write(shm::REQ, &request);
                *seq = seq.wrapping_add(1);
                let want = *seq;
                let req_seq = page.word(shm::REQ_SEQ);
                req_seq.store(want, Ordering::SeqCst);
                if page.word(shm::REQ_SLEEP).load(Ordering::SeqCst) != 0 {
                    protocol::futex_wake(req_seq);
                }
                let rep_seq = page.word(shm::REP_SEQ);
                // Spin first: the program's turn is usually well under the
                // window. Check the clock only every 64 rounds.
                let spin_until = started + *spin;
                let mut rounds = 0u32;
                while rep_seq.load(Ordering::Acquire) != want {
                    rounds = rounds.wrapping_add(1);
                    if !spin.is_zero() && (rounds & 63 != 0 || Instant::now() < spin_until) {
                        std::hint::spin_loop();
                        continue;
                    }
                    let sleeping = page.word(shm::REP_SLEEP);
                    sleeping.store(1, Ordering::SeqCst);
                    while rep_seq.load(Ordering::SeqCst) != want {
                        protocol::futex_wait(rep_seq, want.wrapping_sub(1), LIVENESS);
                        if rep_seq.load(Ordering::SeqCst) == want {
                            break;
                        }
                        if self.child.try_wait().ok().flatten().is_some() {
                            sleeping.store(0, Ordering::SeqCst);
                            return Err(self.died(DURING));
                        }
                        if started.elapsed() > TURN_TIMEOUT {
                            sleeping.store(0, Ordering::SeqCst);
                            return Err(self.unresponsive(DURING, TURN_TIMEOUT));
                        }
                    }
                    sleeping.store(0, Ordering::SeqCst);
                }
                page.read(shm::REP, &mut header);
                let stop = StopHeader::decode(&header);
                let len = stop
                    .tail_len()
                    .min(protocol::REPLY_CAPACITY - StopHeader::LEN);
                self.tail.resize(len, 0);
                page.read(shm::REP + StopHeader::LEN, &mut self.tail);
                Ok(stop)
            }
            Channel::Socket { stream } => {
                match socket_turn(stream, &request, &mut self.tail, started) {
                    Ok(stop) => Ok(stop),
                    Err(failure) => Err(self.failed(failure, DURING)),
                }
            }
        }
    }

    fn failed(&mut self, failure: ReadFailure, during: &str) -> P2QemuError {
        match failure {
            ReadFailure::Closed => self.died(during),
            ReadFailure::Slow => {
                if self.child.try_wait().ok().flatten().is_some() {
                    self.died(during)
                } else {
                    self.unresponsive(during, TURN_TIMEOUT)
                }
            }
        }
    }

    /// The tail of the last reply: its mode words and `WYPIN` bytes.
    pub fn tail(&self) -> &[u8] {
        &self.tail
    }

    /// Ask the program to exit, give it a moment, then make sure.
    fn stop(&mut self) {
        if self.gone {
            return;
        }
        let quit = Run {
            op: OP_QUIT,
            ..Run::default()
        }
        .encode();
        match &mut self.channel {
            Channel::Shm { page, seq, .. } => {
                page.write(shm::REQ, &quit);
                *seq = seq.wrapping_add(1);
                page.word(shm::REQ_SEQ).store(*seq, Ordering::SeqCst);
                protocol::futex_wake(page.word(shm::REQ_SEQ));
            }
            Channel::Socket { stream } => {
                let _ = stream.write_all(&quit);
            }
        }
        let deadline = Instant::now() + QUIT_GRACE;
        while Instant::now() < deadline {
            if self.exited().is_some() {
                return;
            }
            std::thread::sleep(Duration::from_millis(1));
        }
        let _ = self.child.kill();
        let _ = self.child.wait();
        self.gone = true;
    }
}

impl Drop for Peer {
    fn drop(&mut self) {
        self.stop();
    }
}

/// One turn over the socket: the request out, the header and its tail in.
fn socket_turn(
    stream: &mut UnixStream,
    request: &[u8],
    tail: &mut Vec<u8>,
    started: Instant,
) -> Result<StopHeader, ReadFailure> {
    stream.write_all(request).map_err(|_| ReadFailure::Closed)?;
    stream
        .set_read_timeout(Some(LIVENESS))
        .map_err(|_| ReadFailure::Closed)?;
    let mut header = [0u8; StopHeader::LEN];
    read_full(stream, &mut header, started)?;
    let stop = StopHeader::decode(&header);
    tail.resize(stop.tail_len(), 0);
    read_full(stream, tail, started)?;
    Ok(stop)
}

enum ReadFailure {
    /// End of file: the program closed its end.
    Closed,
    /// Nothing for [`TURN_TIMEOUT`].
    Slow,
}

/// Fill `buf` from `stream`, whose reads time out every [`LIVENESS`].
fn read_full(stream: &mut UnixStream, buf: &mut [u8], started: Instant) -> Result<(), ReadFailure> {
    let mut got = 0;
    while got < buf.len() {
        match stream.read(&mut buf[got..]) {
            Ok(0) => return Err(ReadFailure::Closed),
            Ok(n) => got += n,
            Err(e)
                if matches!(
                    e.kind(),
                    io::ErrorKind::WouldBlock
                        | io::ErrorKind::TimedOut
                        | io::ErrorKind::Interrupted
                ) =>
            {
                if started.elapsed() > TURN_TIMEOUT {
                    return Err(ReadFailure::Slow);
                }
            }
            Err(_) => return Err(ReadFailure::Closed),
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scratch(name: &str) -> PathBuf {
        let dir =
            std::env::temp_dir().join(format!("embsim-p2-qemu-find-{}-{name}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).expect("scratch");
        dir
    }

    fn program_in(dir: &Path) -> PathBuf {
        use std::os::unix::fs::PermissionsExt;
        let path = dir.join(QemuSystemP2::NAME);
        std::fs::write(&path, "#!/bin/sh\n").expect("writable");
        std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o755))
            .expect("executable");
        path
    }

    #[test]
    fn the_variable_comes_before_path_and_path_before_the_install_directory() {
        let root = scratch("order");
        let (named, on_path, installed) = (root.join("named"), root.join("bin"), root.join("inst"));
        for dir in [&named, &on_path, &installed] {
            std::fs::create_dir_all(dir).expect("dir");
        }
        let named_program = program_in(&named);
        let path = std::env::join_paths([root.join("empty"), on_path.clone()]).expect("a PATH");

        let found = QemuSystemP2::find_in(
            Some(named_program.clone().into()),
            Some(path.clone()),
            Some(installed.clone()),
        )
        .expect("the named one");
        assert_eq!(
            (found.path(), found.found()),
            (named_program.as_path(), Found::Variable)
        );

        let path_program = program_in(&on_path);
        let found = QemuSystemP2::find_in(None, Some(path.clone()), Some(installed.clone()))
            .expect("the one on PATH");
        assert_eq!(
            (found.path(), found.found()),
            (path_program.as_path(), Found::OnPath)
        );

        let installed_program = program_in(&installed);
        let found = QemuSystemP2::find_in(Some(OsString::new()), None, Some(installed.clone()))
            .expect("the installed one");
        assert_eq!(
            (found.path(), found.found()),
            (installed_program.as_path(), Found::Installed)
        );
        let _ = std::fs::remove_dir_all(&root);
    }

    #[test]
    fn nothing_found_says_where_it_looked_and_how_to_install() {
        let root = scratch("none");
        let err = QemuSystemP2::find_in(None, Some(root.clone().into()), Some(root.join("inst")))
            .expect_err("nothing there");
        let text = err.to_string();
        assert!(text.contains("EMBSIM_QEMU_SYSTEM_P2 is unset"), "{text}");
        assert!(text.contains("none is on PATH"), "{text}");
        assert!(
            text.contains(&format!(
                "none at {}",
                root.join("inst/qemu-system-p2").display()
            )),
            "{text}"
        );
        assert!(text.contains("`embsim qemu install`"), "{text}");

        let err = QemuSystemP2::find_in(Some(root.join("missing").into()), None, None)
            .expect_err("a variable naming nothing");
        assert!(
            err.to_string().contains("EMBSIM_QEMU_SYSTEM_P2 names"),
            "{err}"
        );
        let _ = std::fs::remove_dir_all(&root);
    }

    #[test]
    fn the_install_directory_is_keyed_by_the_target() {
        let dir = install_dir_in(Path::new("/home/me"));
        assert_eq!(
            dir,
            Path::new("/home/me/.embsim/qemu").join(target::identity())
        );
    }

    #[test]
    fn an_identity_differing_in_any_part_is_refused_naming_it() {
        let mut theirs = Identity::needed();
        assert_eq!(theirs.check(), Ok(()));
        theirs.protocol += 1;
        assert!(theirs.check().unwrap_err().contains("speaks protocol"));
        let mut theirs = Identity::needed();
        theirs.target = "0000000000000000".into();
        theirs.qemu = "9.2.0".into();
        let why = theirs.check().unwrap_err();
        assert!(
            why.contains("built from P2 target 0000000000000000"),
            "{why}"
        );
        assert!(why.contains("it is QEMU 9.2.0"), "{why}");
    }

    #[test]
    fn a_status_names_its_signal() {
        assert_eq!(
            status_text(ExitStatus::from_raw(libc::SIGKILL)),
            "was killed by signal 9 (SIGKILL)"
        );
        assert_eq!(
            status_text(ExitStatus::from_raw(3 << 8)),
            "exited with status 3"
        );
    }
}
