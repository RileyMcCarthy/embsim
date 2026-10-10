//! Finding, launching and stopping the host's Chrome.

use std::io::{self, BufRead, BufReader};
use std::net::TcpListener;
use std::os::unix::process::CommandExt;
use std::path::{Path, PathBuf};
use std::process::{Child, ChildStderr, Command, Stdio};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex, PoisonError};
use std::time::{Duration, Instant};

use crate::devtools::ask_browser_ws_url;

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

/// What Chrome prints to its stderr once its DevTools server listens, before
/// the browser's WebSocket URL; Puppeteer reads the same line.
const LISTENING: &str = "DevTools listening on ";

/// What Chrome logs to its stderr when its DevTools server could not listen.
const NO_SERVER: &str = "Cannot start http server for devtools";

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
    /// What it has said on its stderr about its DevTools server.
    heard: Arc<Mutex<Heard>>,
}

/// What a launched Chrome has said on its stderr about its DevTools server.
#[derive(Debug, Default, Clone, PartialEq, Eq)]
struct Heard {
    /// The browser's WebSocket URL, from its "DevTools listening on" line.
    listening: Option<String>,
    /// Whether it said its DevTools server could not listen.
    no_server: bool,
}

impl Heard {
    /// Take in one line of Chrome's stderr.
    fn hear(&mut self, line: &str) {
        if let Some(ws) = line.trim().strip_prefix(LISTENING) {
            self.listening.get_or_insert_with(|| ws.trim().to_string());
        } else if line.contains(NO_SERVER) {
            self.no_server = true;
        }
    }
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
            // Read for where its DevTools listen (`await_devtools`).
            .stderr(Stdio::piped())
            // A group of its own, so stopping it stops its renderers too.
            .process_group(0);
        let mut child = command.spawn().map_err(|e| {
            let _ = std::fs::remove_dir_all(&profile);
            io::Error::new(
                e.kind(),
                format!("could not start {}: {e}", spec.binary.display()),
            )
        })?;
        let stderr = child.stderr.take();
        let mut process = Self {
            child,
            profile,
            port: 0,
            ws_path: String::new(),
            heard: Arc::default(),
        };
        if let Some(stderr) = stderr {
            listen(stderr, Arc::clone(&process.heard))?;
        }
        process.await_devtools(spec)?;
        Ok(process)
    }

    /// Wait until Chrome has opened DevTools, and read where.
    ///
    /// On a port Chrome picks (`port` 0), Chrome writes the port and its
    /// browser target's path to `DevToolsActivePort` in its profile, so the
    /// node reaches this Chrome and no other, whatever else listens nearby.
    /// On a port the spec names it writes no such file (`NODES.md` §19,
    /// evidence E16). There the node takes the browser Chrome says on its
    /// stderr it listens as, once `/json/version` on that port names the
    /// same one; when Chrome says it listens anywhere else, or could not
    /// listen, another program took the port after [`ChromeProcess::launch`]
    /// found it free, and the launch fails saying so.
    fn await_devtools(&mut self, spec: &LaunchSpec) -> io::Result<()> {
        let file = self.profile.join("DevToolsActivePort");
        let endpoint = format!("http://127.0.0.1:{}", spec.port);
        let deadline = Instant::now() + LAUNCH_TIMEOUT;
        let mut asked: Option<io::Error> = None;
        loop {
            let heard = self.heard();
            let found = if spec.port == 0 {
                read_active_port(&file)
            } else {
                match on_named_port(spec.port, &heard)? {
                    Some(path) => match ask_browser_ws_url(&endpoint) {
                        Ok(ws) if ws_parts(&ws).map(|(_, p)| p) == Some(path.as_str()) => {
                            Some((spec.port, path))
                        }
                        Ok(ws) => {
                            asked = Some(io::Error::new(
                                io::ErrorKind::InvalidData,
                                format!("it names another browser, {ws}"),
                            ));
                            None
                        }
                        Err(e) => {
                            asked = Some(e);
                            None
                        }
                    },
                    None => None,
                }
            };
            if let Some((port, path)) = found {
                self.port = port;
                self.ws_path = path;
                return Ok(());
            }
            if self.leader_exited() {
                return Err(io::Error::other(format!(
                    "{} exited before it opened DevTools",
                    spec.binary.display()
                )));
            }
            if Instant::now() >= deadline {
                let binary = spec.binary.display();
                let within = format!("within {:.0} s", LAUNCH_TIMEOUT.as_secs_f64());
                return Err(io::Error::new(
                    io::ErrorKind::TimedOut,
                    match (spec.port, &heard.listening, &asked) {
                        (0, _, _) => format!(
                            "{binary} did not write DevToolsActivePort to its profile {within}"
                        ),
                        (_, Some(ws), asked) => format!(
                            "{binary} said its DevTools listen at {ws}, but {endpoint}/json/version \
                             did not answer as that browser {within}{}",
                            asked.as_ref().map(|e| format!(": {e}")).unwrap_or_default()
                        ),
                        _ => format!(
                            "{binary} did not say where its DevTools listen {within} (no \
                             \"DevTools listening on\" line on its stderr)"
                        ),
                    },
                ));
            }
            std::thread::sleep(Duration::from_millis(20));
        }
    }

    /// What Chrome has said on its stderr so far.
    fn heard(&self) -> Heard {
        self.heard
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .clone()
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

/// Read Chrome's stderr into `heard` on a thread of its own until Chrome
/// and everything it started have closed it, so a full pipe never stalls
/// Chrome.
fn listen(stderr: ChildStderr, heard: Arc<Mutex<Heard>>) -> io::Result<()> {
    std::thread::Builder::new()
        .name("chrome-stderr".to_string())
        .spawn(move || {
            let mut reader = BufReader::new(stderr);
            let mut line = Vec::new();
            while matches!(reader.read_until(b'\n', &mut line), Ok(n) if n > 0) {
                heard
                    .lock()
                    .unwrap_or_else(PoisonError::into_inner)
                    .hear(&String::from_utf8_lossy(&line));
                line.clear();
            }
        })
        .map(drop)
}

/// For a launch on the named `port`, the browser target's path once Chrome
/// has said it listens on 127.0.0.1 at that port, or nothing while it has
/// said nothing. Chrome that listens anywhere else, or could not listen,
/// found the port taken: an error naming the port and what to do.
fn on_named_port(port: u16, heard: &Heard) -> io::Result<Option<String>> {
    let taken = |what: String| {
        io::Error::new(
            io::ErrorKind::AddrInUse,
            format!(
                "port {port} on 127.0.0.1 is taken (it was free just before Chrome started), and \
                 Chrome {what}; name another devtools_port, or leave it out and Chrome picks a \
                 free one"
            ),
        )
    };
    match &heard.listening {
        Some(ws) => match ws_parts(ws) {
            Some((at, path)) if at == format!("127.0.0.1:{port}") => Ok(Some(path.to_string())),
            _ => Err(taken(format!("listens for DevTools at {ws} instead"))),
        },
        None if heard.no_server => Err(taken("could not start its DevTools server".to_string())),
        None => Ok(None),
    }
}

/// The port and the browser target's path a `DevToolsActivePort` file
/// holds, once Chrome has written both lines.
fn read_active_port(file: &Path) -> Option<(u16, String)> {
    let text = std::fs::read_to_string(file).ok()?;
    let mut lines = text.lines();
    let port = lines.next()?.trim().parse::<u16>().ok()?;
    let path = lines.next()?.trim();
    (!path.is_empty()).then(|| (port, path.to_string()))
}

/// A browser's DevTools WebSocket URL in two: where it listens and the
/// browser target's path (`ws://127.0.0.1:9222/devtools/browser/…` →
/// `127.0.0.1:9222` and `/devtools/browser/…`).
fn ws_parts(ws: &str) -> Option<(&str, &str)> {
    let rest = ws.strip_prefix("ws://")?;
    let slash = rest.find('/')?;
    let (at, path) = rest.split_at(slash);
    (path.len() > 1).then_some((at, path))
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

#[cfg(test)]
mod tests {
    use super::*;
    use rstest::rstest;
    use vibes_behaviour::{behaviour, expect, Test};

    /// What Chrome's stderr carried, line by line.
    fn heard(lines: &[&str]) -> Heard {
        let mut heard = Heard::default();
        for line in lines {
            heard.hear(line);
        }
        heard
    }

    #[test]
    fn chrome_saying_it_listens_on_the_named_port_is_the_browser_reached() {
        behaviour!(Test {
            id: "chrome-cdp.named-port-said",
            covers: Some("cdp/src/chrome.rs#on_named_port"),
            given: "Chrome launched on a named DevTools port, logging that it listens on \
                    127.0.0.1 there",
        });
        expect!(
            "browser-taken",
            "the node takes the browser that log line names, whatever Chrome logged before it"
        );
        // Chrome 153's stderr on port 60428 (NODES.md §19, evidence E16).
        let said = heard(&[
            "[52210:107446156:1009/202310.133967:ERROR:ui/display/mac/cv_display_link_mac.mm:195] \
             CVDisplayLinkCreateWithCGDisplay failed. CVReturn: -6670\n",
            "\n",
            "DevTools listening on \
             ws://127.0.0.1:60428/devtools/browser/1335b82b-2e77-43e4-a058-345d6bb2041e\n",
        ]);
        assert_eq!(
            on_named_port(60428, &said).expect("the port is Chrome's"),
            Some("/devtools/browser/1335b82b-2e77-43e4-a058-345d6bb2041e".to_string()),
            "browser-taken"
        );
    }

    // Chrome 153's stderr when another program held 127.0.0.1 at the port,
    // and when it held the IPv6 loopback there too (NODES.md §19, E16).
    #[rstest]
    #[case::elsewhere(
        60454,
        &[
            "\n",
            "DevTools listening on \
             ws://[::1]:60454/devtools/browser/9f8c901e-cbc5-4cf1-8e92-33a22f414624\n",
        ],
        "Chrome listens for DevTools at ws://[::1]:60454/devtools/browser/"
    )]
    #[case::nowhere(
        60505,
        &[
            "[52203:107446206:1009/202309.780367:ERROR:net/socket/socket_posix.cc:175] bind() \
             failed: Address already in use (48)\n",
            "[52203:107446206:1009/202309.780430:ERROR:content/browser/devtools/\
             devtools_http_handler.cc:311] Cannot start http server for devtools.\n",
        ],
        "Chrome could not start its DevTools server"
    )]
    fn a_named_port_another_program_holds_fails_the_launch_saying_so(
        #[case] port: u16,
        #[case] lines: &[&str],
        #[case] says: &str,
    ) {
        behaviour!(Test {
            id: "chrome-cdp.named-port-taken",
            covers: Some("cdp/src/chrome.rs#on_named_port"),
            given: "Chrome launched on a named DevTools port another program holds on \
                    127.0.0.1, logging that it listens on the IPv6 loopback there, or cannot \
                    listen at all",
        });
        expect!(
            "fails-saying-taken",
            "the launch fails, saying the port is taken and what Chrome said"
        );
        expect!(
            "names-the-way-out",
            "the failure says to name another port, or to leave it out so Chrome picks a free one"
        );
        let error = on_named_port(port, &heard(lines)).expect_err("the launch fails");
        let message = error.to_string();
        assert_eq!(error.kind(), io::ErrorKind::AddrInUse, "fails-saying-taken");
        assert!(
            message.contains(&format!("port {port} on 127.0.0.1 is taken"))
                && message.contains(says),
            "fails-saying-taken: {message}"
        );
        assert!(
            message.contains(
                "name another devtools_port, or leave it out and Chrome picks a free one"
            ),
            "names-the-way-out: {message}"
        );
    }
}
