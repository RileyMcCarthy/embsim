//! The Chrome guest under the node, when its image has been built.
//!
//! Boots the image (`guest/chrome/build.sh`), lets it warm up on host time
//! until DevTools answers, then puts it on the board at the default quantum
//! and checks the three things the guest adds: Chrome stays reachable while
//! it lives on the board's time, the agent's clock keeps the books exact
//! (no drift, not just bounded skew), and the guest's clock is the board's,
//! to a quantum, over a thousand slices. A second case builds the same
//! guest from a project's `chrome-vm` entry, as `embsim run` does, and a
//! third has a page in it use Web Serial — the browser's own, on the
//! emulated FTDI the guest's kernel enumerates — to trade bytes with a
//! `host-serial` port over the board's wires.
//!
//! `#[ignore]`d: they need the image (`EMBSIM_CHROME_IMAGE`, or where
//! `build.sh` writes it, `embsim_qemu::default_image`) and
//! `qemu-system-<host arch>`, neither of which a CI runner has. The recipe,
//! once the image is built (`qemu/guest/chrome/README.md`):
//!
//! ```text
//! cargo test -p embsim-qemu --test chrome_guest -- --ignored --nocapture
//! ```
//!
//! They declare no behaviours: the ledger's suite does not run them.

use std::path::PathBuf;
use std::sync::{Mutex, MutexGuard};
use std::time::{Duration, Instant};

use embsim_board::{EndpointRef, Harness, Project, Reports, System};
use embsim_boards::catalog::CatalogSet;
use embsim_core::virtual_clock::{self, ClockMode};
use embsim_qemu::{ChromeGuest, QemuNode, DEFAULT_QUANTUM};

static SUITE_LOCK: Mutex<()> = Mutex::new(());

fn suite_lock() -> MutexGuard<'static, ()> {
    SUITE_LOCK.lock().unwrap_or_else(|poisoned| {
        SUITE_LOCK.clear_poison();
        poisoned.into_inner()
    })
}

/// The image: `EMBSIM_CHROME_IMAGE`, else where `build.sh` writes it.
fn image() -> PathBuf {
    std::env::var_os("EMBSIM_CHROME_IMAGE")
        .map(PathBuf::from)
        .or_else(embsim_qemu::default_image)
        .filter(|image| image.is_file())
        .expect(
            "needs the Chrome guest image: qemu/guest/chrome/build.sh builds it, or \
             EMBSIM_CHROME_IMAGE names one",
        )
}

fn ep(endpoint: &str) -> EndpointRef {
    EndpointRef::parse(endpoint).expect("endpoint parses")
}

/// Hand the engine virtual time a millisecond at a time until `done`, or
/// fail after `hang` of host time.
fn step_until(mut done: impl FnMut() -> bool, hang: Duration, what: &str) {
    let start = Instant::now();
    while !done() {
        assert!(start.elapsed() < hang, "{what} after {hang:?}");
        virtual_clock::wait_virtual_ns(1_000_000);
    }
}

#[test]
#[ignore = "needs the Chrome guest image (qemu/guest/chrome/build.sh) and QEMU"]
fn the_chrome_guest_keeps_exact_time_on_the_board_and_stays_reachable() {
    let _suite = suite_lock();
    let booted = Instant::now();
    let chrome = ChromeGuest::new(image())
        .spawn()
        .expect("the Chrome guest boots and answers DevTools");
    let devtools = chrome.devtools().clone();
    eprintln!(
        "guest warm in {:?}; DevTools at {}",
        booted.elapsed(),
        devtools.url()
    );
    assert!(
        chrome.vm().has_agent(),
        "the image's agent port should be attached"
    );

    virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);
    let node = QemuNode::new(Box::new(chrome), 2_000_000);
    let stats = node.stats();
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
    let actor = virtual_clock::register_actor("chrome-guest-case");
    system.release_time();

    // Chrome answers over TCP while metered: every packet needs guest time,
    // and it gets it a quantum at a time. The probe runs on a thread of its
    // own, so the board advances while it waits.
    let probe = {
        let devtools = devtools.clone();
        std::thread::spawn(move || {
            let start = Instant::now();
            while start.elapsed() < Duration::from_secs(60) {
                if devtools.is_up() {
                    return true;
                }
            }
            false
        })
    };
    step_until(
        || probe.is_finished(),
        Duration::from_secs(90),
        "the DevTools probe never finished",
    );
    assert!(
        probe.join().unwrap(),
        "DevTools did not answer while the guest lived on the board's clock"
    );

    let wall = Instant::now();
    step_until(
        || stats.slices() >= 1000,
        Duration::from_secs(120),
        "fewer than 1000 slices",
    );
    eprintln!(
        "{} slices in {:.3} s of host time: the guest lived {:.3} ms of the board's {:.3} ms, {} slices booked by its agent, {} µs apart",
        stats.slices(),
        wall.elapsed().as_secs_f64(),
        stats.guest_ns() as f64 / 1e6,
        stats.virtual_ns() as f64 / 1e6,
        stats.clocked(),
        stats.skew_ns() / 1_000
    );
    // The agent answered: the books come from the guest's own clock.
    assert!(
        stats.clocked() >= 900,
        "only {} of {} slices were clocked by the agent",
        stats.clocked(),
        stats.slices()
    );
    // Exact accounting: after a thousand slices the skew is still within a
    // quantum or two, not the ~1 % drift a stopwatch alone accumulates.
    let skew = stats.skew_ns();
    let quantum = DEFAULT_QUANTUM.as_nanos() as i64;
    assert!(
        skew.abs() <= 3 * quantum,
        "skew {skew} ns after {} slices (virtual {} ns, guest {} ns)",
        stats.slices(),
        stats.virtual_ns(),
        stats.guest_ns()
    );
    assert_eq!(stats.shed(), 0);
    assert!(!stats.disconnected());
    drop(actor);
    system.shutdown();
}

#[test]
#[ignore = "needs the Chrome guest image (qemu/guest/chrome/build.sh) and QEMU"]
fn a_chrome_vm_from_a_project_boots_at_its_first_slice_and_answers_devtools() {
    let _suite = suite_lock();
    let port = embsim_qemu::free_port().expect("a free port");
    let text = format!(
        r#"
[[component]]
name = "PC"
kind = "chrome-vm"
[component.options]
baud = 2000000
image = {image:?}
devtools_port = {port}

[[wire]]
from = "BENCH.3V3"
to = "PC.VIO"
volts = 3.3

[[wire]]
from = "BENCH.GND"
to = "PC.GND"
volts = 0.0
"#,
        image = image(),
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
    let actor = virtual_clock::register_actor("chrome-vm-project-case");
    system.release_time();
    let mut said = reports[0].look(0);
    // The first slice boots the guest, the board held at one quantum.
    virtual_clock::wait_virtual_ns(2 * DEFAULT_QUANTUM.as_nanos() as u64);
    said.extend(reports[0].look(virtual_clock::virtual_ns()));
    assert_eq!(reports[0].failure(), None);
    let url = format!("http://127.0.0.1:{port}");
    assert!(
        said.iter()
            .any(|line| line == &format!("DevTools at {url}")),
        "{said:?}"
    );
    let devtools = embsim_qemu::DevTools { port };
    let probe = std::thread::spawn(move || {
        let start = Instant::now();
        while start.elapsed() < Duration::from_secs(60) {
            if devtools.is_up() {
                return true;
            }
        }
        false
    });
    step_until(
        || probe.is_finished(),
        Duration::from_secs(90),
        "the DevTools probe never finished",
    );
    assert!(probe.join().unwrap(), "DevTools did not answer at {url}");
    drop(actor);
    system.shutdown();
    eprintln!("{}\n{}", said.join("\n"), reports[0].summary().join("\n"));
}

// ============================================================
// The page's Web Serial, through the board
// ============================================================

/// The dev-server ports the image's managed policy grants Web Serial to
/// (`guest/chrome/user-data`): the page is served on the first free one.
const POLICY_PORTS: [u16; 6] = [8000, 8080, 3000, 5173, 4173, 5174];

/// Serve one page on the host's loopback, which the guest reaches at
/// `10.0.2.2`, on a port the policy names. Returns the port; the server
/// thread lives as long as the test process.
fn serve_page() -> u16 {
    use std::io::{Read, Write};
    let (listener, port) = POLICY_PORTS
        .iter()
        .find_map(|port| {
            std::net::TcpListener::bind(("127.0.0.1", *port))
                .ok()
                .map(|listener| (listener, *port))
        })
        .expect("one of the policy's ports is free on the host");
    std::thread::spawn(move || {
        for stream in listener.incoming() {
            let Ok(mut stream) = stream else { continue };
            let mut request = [0u8; 2048];
            let _ = stream.read(&mut request);
            let body = "<!doctype html><title>embsim</title><p>the board is on the serial port</p>";
            let _ = write!(
                stream,
                "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Length: {}\r\n\
                 Connection: close\r\n\r\n{body}",
                body.len()
            );
        }
    });
    port
}

/// Just enough of the DevTools protocol to open a page and evaluate in it:
/// an HTTP request for the target, a WebSocket to it, text frames of JSON.
struct Cdp {
    socket: std::net::TcpStream,
    id: u64,
}

impl Cdp {
    /// Open a new page at `url` in the guest's Chrome, DevTools at `port`.
    fn open(port: u16, url: &str) -> Self {
        use std::io::{Read, Write};
        let mut http = std::net::TcpStream::connect(("127.0.0.1", port)).expect("DevTools");
        http.set_read_timeout(Some(Duration::from_secs(60)))
            .unwrap();
        write!(
            http,
            "PUT /json/new?{url} HTTP/1.1\r\nHost: 127.0.0.1:{port}\r\nConnection: close\r\n\r\n"
        )
        .unwrap();
        // Chrome may keep the connection open: read the head, then as much
        // body as it says it sent.
        let mut reply = Vec::new();
        let mut chunk = [0u8; 4096];
        let body_at = loop {
            if let Some(at) = reply.windows(4).position(|w| w == b"\r\n\r\n") {
                let head = String::from_utf8_lossy(&reply[..at]).to_ascii_lowercase();
                let length: usize = head
                    .lines()
                    .find_map(|line| line.strip_prefix("content-length:"))
                    .and_then(|n| n.trim().parse().ok())
                    .expect("DevTools says how long its answer is");
                if reply.len() >= at + 4 + length {
                    break at + 4;
                }
            }
            let n = http.read(&mut chunk).expect("DevTools answers /json/new");
            assert!(
                n > 0,
                "DevTools closed: {}",
                String::from_utf8_lossy(&reply)
            );
            reply.extend_from_slice(&chunk[..n]);
        };
        let text = String::from_utf8_lossy(&reply[body_at..]);
        let json = text.as_ref();
        let target: serde_json::Value = serde_json::from_str(json).expect("the target's JSON");
        let ws = target["webSocketDebuggerUrl"]
            .as_str()
            .expect("a debugger URL")
            .to_string();
        let path = &ws[ws.find("/devtools").expect("a DevTools path")..];

        let mut socket = std::net::TcpStream::connect(("127.0.0.1", port)).expect("DevTools");
        socket
            .set_read_timeout(Some(Duration::from_secs(120)))
            .unwrap();
        write!(
            socket,
            "GET {path} HTTP/1.1\r\nHost: 127.0.0.1:{port}\r\nUpgrade: websocket\r\n\
             Connection: Upgrade\r\nSec-WebSocket-Key: ZW1ic2ltLWNocm9tZS12bQ==\r\n\
             Sec-WebSocket-Version: 13\r\n\r\n"
        )
        .unwrap();
        let mut head = Vec::new();
        let mut byte = [0u8; 1];
        while !head.ends_with(b"\r\n\r\n") {
            socket
                .read_exact(&mut byte)
                .expect("the WebSocket handshake");
            head.push(byte[0]);
        }
        let head = String::from_utf8_lossy(&head);
        assert!(head.contains(" 101 "), "no WebSocket: {head}");
        Self { socket, id: 0 }
    }

    fn send(&mut self, text: &str) {
        use std::io::Write;
        let data = text.as_bytes();
        let mask = [0x5a, 0x17, 0xc3, 0x09];
        let mut frame = vec![0x81];
        match data.len() {
            n if n < 126 => frame.push(0x80 | n as u8),
            n if n <= 0xffff => {
                frame.push(0x80 | 126);
                frame.extend_from_slice(&(n as u16).to_be_bytes());
            }
            n => {
                frame.push(0x80 | 127);
                frame.extend_from_slice(&(n as u64).to_be_bytes());
            }
        }
        frame.extend_from_slice(&mask);
        frame.extend(data.iter().enumerate().map(|(i, b)| b ^ mask[i % 4]));
        self.socket.write_all(&frame).expect("a frame to DevTools");
    }

    fn receive(&mut self) -> serde_json::Value {
        use std::io::Read;
        let mut head = [0u8; 2];
        self.socket
            .read_exact(&mut head)
            .expect("a frame from DevTools");
        let len = match head[1] & 0x7f {
            126 => {
                let mut n = [0u8; 2];
                self.socket.read_exact(&mut n).unwrap();
                u64::from(u16::from_be_bytes(n))
            }
            127 => {
                let mut n = [0u8; 8];
                self.socket.read_exact(&mut n).unwrap();
                u64::from_be_bytes(n)
            }
            n => u64::from(n),
        };
        let mut body = vec![0u8; len as usize];
        self.socket.read_exact(&mut body).unwrap();
        serde_json::from_slice(&body).expect("DevTools sends JSON")
    }

    /// Evaluate `expression` in the page, awaiting a promise it returns,
    /// and return its value.
    fn evaluate(&mut self, expression: &str) -> serde_json::Value {
        self.id += 1;
        let request = serde_json::json!({
            "id": self.id,
            "method": "Runtime.evaluate",
            "params": { "expression": expression, "awaitPromise": true, "returnByValue": true },
        });
        self.send(&request.to_string());
        loop {
            let message = self.receive();
            if message["id"] == self.id {
                return message["result"]["result"]["value"].clone();
            }
        }
    }
}

#[test]
#[ignore = "needs the Chrome guest image (qemu/guest/chrome/build.sh) and QEMU"]
fn the_pages_web_serial_reaches_a_host_port_on_the_board_and_back() {
    use std::io::{Read, Write};
    let _suite = suite_lock();
    let page_port = serve_page();
    let devtools_port = embsim_qemu::free_port().expect("a free port");
    let dir = std::env::temp_dir().join(format!("embsim-chrome-serial-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let pty = dir.join("host.pty");
    // The directory goes when the case ends; the host port removes its link.
    struct Gone(PathBuf);
    impl Drop for Gone {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }
    let _gone = Gone(dir.clone());
    // The guest's line to a host serial port on the board, as a null-modem
    // cable at 115 200 baud, both on one 3.3 V rail.
    let text = format!(
        r#"
[[component]]
name = "PC"
kind = "chrome-vm"
[component.options]
baud = 115200
image = {image:?}
devtools_port = {devtools_port}

[[component]]
name = "HOST"
kind = "host-serial"
[component.options]
baud = 115200
path = {pty:?}

[[wire]]
from = "PC.TX"
to = "HOST.RX"

[[wire]]
from = "HOST.TX"
to = "PC.RX"

[[wire]]
from = "BENCH.3V3"
to = "PC.VIO"
volts = 3.3

[[wire]]
from = "BENCH.3V3"
to = "HOST.VIO"

[[wire]]
from = "BENCH.GND"
to = "PC.GND"
volts = 0.0

[[wire]]
from = "BENCH.GND"
to = "HOST.GND"
"#,
        image = image(),
    );
    virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);
    let set = {
        let mut set = CatalogSet::new();
        embsim_qemu::catalog::register(&mut set).expect("the VM kinds join");
        set
    };
    let system = Project::parse(&text)
        .expect("the text is a project")
        .instantiate(&set)
        .expect("the project builds")
        .hold_time()
        .start()
        .expect("the bench starts");
    let mut host = std::fs::OpenOptions::new()
        .read(true)
        .write(true)
        .open(&pty)
        .expect("the host port's PTY");
    // SAFETY: `host` owns the descriptor for the duration of these calls.
    unsafe {
        let fd = std::os::fd::AsRawFd::as_raw_fd(&host);
        let flags = libc::fcntl(fd, libc::F_GETFL);
        assert!(libc::fcntl(fd, libc::F_SETFL, flags | libc::O_NONBLOCK) >= 0);
    }
    let actor = virtual_clock::register_actor("chrome-web-serial-case");
    system.release_time();

    // The page's side, on a thread of its own: everything it does needs the
    // guest to run, so the case's thread hands the board its time.
    let page = std::thread::spawn(move || {
        let devtools = embsim_qemu::DevTools {
            port: devtools_port,
        };
        let start = Instant::now();
        while !devtools.is_up() {
            assert!(start.elapsed() < Duration::from_secs(300), "no DevTools");
        }
        let mut cdp = Cdp::open(devtools_port, &format!("http://10.0.2.2:{page_port}/"));
        let start = Instant::now();
        while cdp.evaluate("document.readyState") != "complete" {
            assert!(
                start.elapsed() < Duration::from_secs(120),
                "the page never loaded"
            );
        }
        cdp.evaluate(
            "(async () => {
               const ports = await navigator.serial.getPorts();
               const port = ports.find(p => p.getInfo().usbVendorId === 0x0403);
               if (!port) return 'no FTDI port: ' + JSON.stringify(ports.map(p => p.getInfo()));
               await port.open({ baudRate: 115200 });
               const writer = port.writable.getWriter();
               await writer.write(new TextEncoder().encode('hello from chrome'));
               writer.releaseLock();
               const reader = port.readable.getReader();
               let got = '';
               while (!got.includes('board')) {
                 const { value } = await reader.read();
                 got += new TextDecoder().decode(value);
               }
               reader.releaseLock();
               return 'read ' + got;
             })()",
        )
    });

    let start = Instant::now();
    let mut heard = Vec::new();
    let mut answered = false;
    while !page.is_finished() {
        assert!(
            start.elapsed() < Duration::from_secs(600),
            "the round trip never finished; the host heard {:?}",
            String::from_utf8_lossy(&heard)
        );
        virtual_clock::wait_virtual_ns(1_000_000);
        let mut buf = [0u8; 256];
        match host.read(&mut buf) {
            Ok(n) => heard.extend_from_slice(&buf[..n]),
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {}
            Err(e) => panic!("reading the host port: {e}"),
        }
        if !answered && heard.ends_with(b"hello from chrome") {
            host.write_all(b"hello from the board").unwrap();
            answered = true;
        }
    }
    let said = page.join().expect("the page's side finished");
    assert_eq!(
        String::from_utf8_lossy(&heard),
        "hello from chrome",
        "the host port heard what the page wrote to its serial port"
    );
    assert_eq!(
        said, "read hello from the board",
        "the page read the host's answer"
    );
    eprintln!(
        "the page's Web Serial and the host port traded 17 and 20 bytes over the board's \
         wires in {:.3} s of host time, at {} of virtual time",
        start.elapsed().as_secs_f64(),
        embsim_board::report::instant(virtual_clock::virtual_ns())
    );
    drop(actor);
    system.shutdown();
}
