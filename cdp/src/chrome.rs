//! Finding, launching and stopping the host's Chrome.

use std::io;
use std::net::TcpListener;
use std::os::unix::process::CommandExt;
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant};

/// Where Chrome usually is, in the order looked at: the macOS application,
/// then the names Linux packages install on `PATH`.
const MACOS_CHROME: &str = "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome";
const PATH_NAMES: [&str; 4] = [
    "google-chrome",
    "google-chrome-stable",
    "chromium",
    "chromium-browser",
];

/// How long a launched Chrome has to answer on its DevTools port.
pub const LAUNCH_TIMEOUT: Duration = Duration::from_secs(30);

/// How long a stopped Chrome has to exit before its process group is killed.
const QUIT_GRACE: Duration = Duration::from_secs(2);

/// The host's Chrome: `CHROME` if set, the macOS application, or the first
/// of `google-chrome`, `google-chrome-stable`, `chromium`,
/// `chromium-browser` on `PATH`.
pub fn find_chrome() -> Option<PathBuf> {
    if let Some(path) = std::env::var_os("CHROME").map(PathBuf::from) {
        return path.is_file().then_some(path);
    }
    let mac = Path::new(MACOS_CHROME);
    if mac.is_file() {
        return Some(mac.to_path_buf());
    }
    PATH_NAMES.iter().find_map(|name| on_path(name))
}

/// `name` resolved on `PATH`, if it is there.
pub fn on_path(name: &str) -> Option<PathBuf> {
    let path = std::env::var_os("PATH")?;
    std::env::split_paths(&path)
        .map(|dir| dir.join(name))
        .find(|candidate| candidate.is_file())
}

/// What a launch starts.
#[derive(Debug, Clone)]
pub struct LaunchSpec {
    /// The Chrome binary.
    pub binary: PathBuf,
    /// The DevTools port it listens on; 0 to let Chrome pick a free one.
    pub port: u16,
    /// `--headless=new`.
    pub headless: bool,
}

/// A Chrome this process started, in a process group of its own with a
/// throw-away profile; dropping it stops the group and removes the profile.
#[derive(Debug)]
pub struct ChromeProcess {
    child: Child,
    profile: PathBuf,
    /// The port its DevTools listen on, once read.
    port: u16,
    /// Its browser target's WebSocket path (`/devtools/browser/…`).
    ws_path: String,
}

/// Each launch's profile gets a number of its own within the process.
static PROFILES: AtomicU64 = AtomicU64::new(0);

impl ChromeProcess {
    /// Start Chrome as `spec` says, on `about:blank`, in a fresh profile.
    ///
    /// The arguments keep Chrome from doing work of its own a run did not
    /// ask for (first-run pages, background networking, throttling a
    /// background renderer's timers, the macOS keychain), the ones
    /// Playwright passes for the same reason.
    pub fn launch(spec: &LaunchSpec) -> io::Result<Self> {
        if spec.port != 0 && TcpListener::bind(("127.0.0.1", spec.port)).is_err() {
            return Err(io::Error::new(
                io::ErrorKind::AddrInUse,
                format!(
                    "port {} on 127.0.0.1 is taken (by another debuggable Chrome?); leave \
                     devtools_port out and Chrome picks a free one",
                    spec.port
                ),
            ));
        }
        let profile = std::env::temp_dir().join(format!(
            "embsim-cdp-{}-{}",
            std::process::id(),
            PROFILES.fetch_add(1, Ordering::Relaxed)
        ));
        std::fs::create_dir_all(&profile)?;
        let mut command = Command::new(&spec.binary);
        command
            .arg(format!("--remote-debugging-port={}", spec.port))
            .arg(format!("--user-data-dir={}", profile.display()))
            .args([
                "--no-first-run",
                "--no-default-browser-check",
                "--disable-background-networking",
                "--disable-background-timer-throttling",
                "--disable-backgrounding-occluded-windows",
                "--disable-renderer-backgrounding",
                "--disable-component-update",
                "--disable-component-extensions-with-background-pages",
                "--disable-default-apps",
                "--disable-sync",
                "--disable-features=Translate,MediaRouter,OptimizationHints",
                "--password-store=basic",
                "--use-mock-keychain",
            ]);
        if spec.headless {
            command.arg("--headless=new");
        }
        command
            .arg("about:blank")
            .stdin(Stdio::null())
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            // A group of its own, so stopping it stops its renderers too.
            .process_group(0);
        let child = command.spawn().map_err(|e| {
            let _ = std::fs::remove_dir_all(&profile);
            io::Error::new(
                e.kind(),
                format!("could not start {}: {e}", spec.binary.display()),
            )
        })?;
        let mut process = Self {
            child,
            profile,
            port: 0,
            ws_path: String::new(),
        };
        process.await_devtools(spec)?;
        Ok(process)
    }

    /// Wait until Chrome has opened DevTools, and read where: Chrome writes
    /// the port it listens on and its browser target's path to
    /// `DevToolsActivePort` in its profile, so the node reaches this Chrome
    /// and no other, whatever else listens nearby.
    fn await_devtools(&mut self, spec: &LaunchSpec) -> io::Result<()> {
        let file = self.profile.join("DevToolsActivePort");
        let deadline = Instant::now() + LAUNCH_TIMEOUT;
        loop {
            if let Ok(text) = std::fs::read_to_string(&file) {
                let mut lines = text.lines();
                if let (Some(port), Some(path)) = (lines.next(), lines.next()) {
                    if let Ok(port) = port.trim().parse::<u16>() {
                        self.port = port;
                        self.ws_path = path.trim().to_string();
                        return Ok(());
                    }
                }
            }
            if self.leader_exited() {
                return Err(io::Error::other(format!(
                    "{} exited before it opened DevTools",
                    spec.binary.display()
                )));
            }
            if Instant::now() >= deadline {
                return Err(io::Error::new(
                    io::ErrorKind::TimedOut,
                    format!(
                        "{} did not open DevTools within {:.0} s{}",
                        spec.binary.display(),
                        LAUNCH_TIMEOUT.as_secs_f64(),
                        if spec.port == 0 {
                            String::new()
                        } else {
                            format!(" (is port {} taken?)", spec.port)
                        }
                    ),
                ));
            }
            std::thread::sleep(Duration::from_millis(20));
        }
    }

    /// The DevTools HTTP endpoint, `http://127.0.0.1:PORT`.
    pub fn endpoint(&self) -> String {
        format!("http://127.0.0.1:{}", self.port)
    }

    /// The browser target's WebSocket, `ws://127.0.0.1:PORT/devtools/browser/…`.
    pub fn ws_url(&self) -> String {
        format!("ws://127.0.0.1:{}{}", self.port, self.ws_path)
    }

    /// Whether the browser process has exited, leaving it unreaped.
    pub fn leader_exited(&self) -> bool {
        // SAFETY: `info` is a valid, zeroed `siginfo_t` for waitid(2) to
        // fill; WNOWAIT leaves the child to be reaped by `Child::wait`.
        unsafe {
            let mut info: libc::siginfo_t = std::mem::zeroed();
            let rc = libc::waitid(
                libc::P_PID,
                self.child.id() as libc::id_t,
                &mut info,
                libc::WEXITED | libc::WNOHANG | libc::WNOWAIT,
            );
            rc == 0 && info.si_pid() != 0
        }
    }
}

impl Drop for ChromeProcess {
    fn drop(&mut self) {
        let group = -(self.child.id() as i32);
        // SAFETY: kill(2) with a negative pid signals the process group this
        // launch made (`process_group(0)`); no memory is touched.
        unsafe {
            libc::kill(group, libc::SIGTERM);
        }
        // Wait for the leader to exit without reaping it: until it is
        // reaped its id cannot name another group, so the SIGKILL below
        // reaches only what this launch started (renderers that outlive
        // the browser process among them).
        let deadline = Instant::now() + QUIT_GRACE;
        while Instant::now() < deadline && !self.leader_exited() {
            std::thread::sleep(Duration::from_millis(20));
        }
        // SAFETY: as above; the group is the one this launch made, and its
        // leader is not reaped yet.
        unsafe {
            libc::kill(group, libc::SIGKILL);
        }
        let _ = self.child.wait();
        let _ = std::fs::remove_dir_all(&self.profile);
    }
}
