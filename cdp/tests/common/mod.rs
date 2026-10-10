//! What the node's tests share: a board-side peer on the line, the bench,
//! a static server for the real-Chrome pages, and a DevTools client that
//! reads a page the node holds, as a harness's `connectOverCDP` would.
#![allow(dead_code)] // each test binary uses its own part of this

use std::io::{BufRead, BufReader, Write};
use std::net::{TcpListener, TcpStream};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use embsim_board::uart::{FramingError, UartFraming};
use embsim_board::{
    AttachError, Component, ComponentNetIo, EndpointRef, Finding, Harness, HostRailLine, PinDecl,
    System, SystemHandle, HOST_RAIL_PINS,
};
use embsim_cdp::devtools::{browser_ws_url, DevTools};
use embsim_cdp::CdpNode;
use embsim_core::virtual_clock::{self, Actor, ClockMode};
use serde_json::{json, Value};

/// One live case at a time: the virtual clock is process-global, and each
/// case re-anchors it stepped (`TESTING.md` rule 9).
static SUITE_LOCK: Mutex<()> = Mutex::new(());

pub fn suite_lock() -> MutexGuard<'static, ()> {
    SUITE_LOCK.lock().unwrap_or_else(|poisoned| {
        SUITE_LOCK.clear_poison();
        poisoned.into_inner()
    })
}

pub fn ep(endpoint: &str) -> EndpointRef {
    EndpointRef::parse(endpoint).expect("endpoint parses")
}

/// What a peer heard and how it is told to speak.
#[derive(Default)]
pub struct PeerShared {
    line: Mutex<Option<HostRailLine>>,
    /// Every byte heard, with the instant its frame ended.
    heard: Mutex<Vec<(u64, u8)>>,
    pub framing_errors: AtomicU64,
    /// Bytes the line took to send.
    pub sent: AtomicU64,
    /// Bytes shed (no rail yet, or a full queue).
    pub shed: AtomicU64,
}

/// The test's end of a peer.
#[derive(Clone)]
pub struct PeerHandle(pub Arc<PeerShared>);

impl PeerHandle {
    /// Send `bytes` from the board's side: framed from the instant the
    /// engine next runs the peer.
    pub fn send(&self, bytes: &[u8]) {
        let line = self.0.line.lock().unwrap();
        let line = line.as_ref().expect("the peer is attached");
        assert!(line.rail_known(), "the peer's rail has not been read yet");
        let shed = line.bridge().transmit(bytes);
        self.0.shed.fetch_add(shed as u64, Ordering::Relaxed);
        self.0
            .sent
            .fetch_add((bytes.len() - shed) as u64, Ordering::Relaxed);
    }

    /// Whether the peer's line can take bytes (its rail has been read).
    pub fn ready(&self) -> bool {
        self.0
            .line
            .lock()
            .unwrap()
            .as_ref()
            .is_some_and(HostRailLine::rail_known)
    }

    /// The bytes heard so far.
    pub fn heard(&self) -> Vec<u8> {
        self.0
            .heard
            .lock()
            .unwrap()
            .iter()
            .map(|&(_, b)| b)
            .collect()
    }

    /// The bytes heard so far, each with the instant its frame ended.
    pub fn heard_at(&self) -> Vec<(u64, u8)> {
        self.0.heard.lock().unwrap().clone()
    }
}

/// The board's side of the line: a serial port on the host-rail pins
/// that records what it hears and, if asked, echoes it.
pub struct Peer {
    framing: UartFraming,
    echo: bool,
    shared: Arc<PeerShared>,
    shutdown: Arc<AtomicBool>,
}

impl Peer {
    pub fn new(baud: u32, echo: bool) -> (Self, PeerHandle) {
        let shared = Arc::new(PeerShared::default());
        (
            Self {
                framing: UartFraming::new_8n1(baud),
                echo,
                shared: Arc::clone(&shared),
                shutdown: Arc::new(AtomicBool::new(false)),
            },
            PeerHandle(shared),
        )
    }
}

impl Drop for Peer {
    fn drop(&mut self) {
        self.shutdown.store(true, Ordering::Relaxed);
    }
}

fn record(
    shared: &PeerShared,
    line: &HostRailLine,
    echo: bool,
    at: u64,
    frames: Vec<Result<u8, FramingError>>,
) {
    let mut bytes = Vec::new();
    for frame in frames {
        match frame {
            Ok(byte) => bytes.push(byte),
            Err(_) => {
                shared.framing_errors.fetch_add(1, Ordering::Relaxed);
            }
        }
    }
    if bytes.is_empty() {
        return;
    }
    shared
        .heard
        .lock()
        .unwrap()
        .extend(bytes.iter().map(|&b| (at, b)));
    if echo {
        let shed = line.bridge().transmit(&bytes);
        shared.shed.fetch_add(shed as u64, Ordering::Relaxed);
        shared
            .sent
            .fetch_add((bytes.len() - shed) as u64, Ordering::Relaxed);
    }
}

impl Component for Peer {
    fn pins(&self) -> &[PinDecl] {
        &HOST_RAIL_PINS
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let line = HostRailLine::attach(&io, self.framing, Arc::clone(&self.shutdown))?;
        *self.shared.line.lock().unwrap() = Some(line.clone());
        {
            let (shared, line, echo) = (Arc::clone(&self.shared), line.clone(), self.echo);
            let rx = io.pin("RX")?;
            io.on_sense("RX", move |sense| {
                let at = sense.at_ns;
                record(
                    &shared,
                    &line,
                    echo,
                    at,
                    line.bridge().receive_sense(&rx, &sense),
                );
            })?;
        }
        {
            let (shared, echo) = (Arc::clone(&self.shared), self.echo);
            io.on_wake_ns(move |now| {
                record(&shared, &line, echo, now, line.bridge().service(now));
            });
        }
        Ok(())
    }
}

/// The node and its peer on one bench, each line on a 3.3 V rail of its
/// own, each one's `TX` to the other's `RX`; started with time held, the
/// case's thread a registered actor, time released. The clock is
/// re-anchored stepped first.
pub fn bench(node: CdpNode, peer: Box<dyn Component>) -> (SystemHandle, Actor) {
    virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);
    let mut harness = Harness::new()
        .connect(ep("PC.TX"), ep("PEER.RX"))
        .connect(ep("PEER.TX"), ep("PC.RX"));
    for name in ["PC", "PEER"] {
        harness = harness
            .power(
                ep(&format!("BENCH.{name}3V3")),
                ep(&format!("{name}.VIO")),
                3.3,
            )
            .power(
                ep(&format!("BENCH.{name}GND")),
                ep(&format!("{name}.GND")),
                0.0,
            );
    }
    let system = System::new()
        .component("PC", Box::new(node))
        .component("PEER", peer)
        .harness(harness)
        .hold_time()
        .start()
        .expect("the bench starts");
    let actor = virtual_clock::register_actor("cdp-case");
    system.release_time();
    (system, actor)
}

/// The end of a case: no stall, the case's thread out of the clock, the
/// system down.
pub fn finish(system: SystemHandle, actor: Actor) {
    let stalled: Vec<Finding> = system
        .findings()
        .into_iter()
        .filter(|finding| matches!(finding, Finding::QuiescenceTimeout { .. }))
        .collect();
    assert!(
        stalled.is_empty(),
        "the engine stopped waiting for the case's thread: {stalled:?}"
    );
    drop(actor);
    system.shutdown();
}

// ============================================================
// The real-Chrome side
// ============================================================

/// A static file server for a directory of pages, cross-origin isolated
/// (so `performance.now()` resolves to 5 µs), on a free loopback port.
pub struct Server {
    pub url: String,
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

impl Server {
    pub fn start(dir: &Path) -> Self {
        let listener = TcpListener::bind("127.0.0.1:0").expect("a loopback port");
        listener.set_nonblocking(true).expect("non-blocking");
        let url = format!(
            "http://127.0.0.1:{}/",
            listener.local_addr().unwrap().port()
        );
        let stop = Arc::new(AtomicBool::new(false));
        let dir = dir.to_path_buf();
        let thread = {
            let stop = Arc::clone(&stop);
            std::thread::spawn(move || {
                while !stop.load(Ordering::Relaxed) {
                    match listener.accept() {
                        Ok((stream, _)) => {
                            let dir = dir.clone();
                            std::thread::spawn(move || serve(stream, &dir));
                        }
                        Err(_) => std::thread::sleep(Duration::from_millis(2)),
                    }
                }
            })
        };
        Self {
            url,
            stop,
            thread: Some(thread),
        }
    }
}

impl Drop for Server {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

fn serve(stream: TcpStream, dir: &Path) {
    stream.set_nonblocking(false).ok();
    let mut reader = BufReader::new(stream.try_clone().expect("clone"));
    let mut line = String::new();
    if reader.read_line(&mut line).is_err() {
        return;
    }
    loop {
        let mut header = String::new();
        if reader.read_line(&mut header).is_err() || header.trim().is_empty() {
            break;
        }
    }
    let path = line.split_whitespace().nth(1).unwrap_or("/");
    let path = path.split('?').next().unwrap_or("/");
    let file: PathBuf = dir.join(if path == "/" {
        "index.html"
    } else {
        path.trim_start_matches('/')
    });
    let mut stream = stream;
    match std::fs::read(&file) {
        Ok(body) => {
            let kind = match file.extension().and_then(|e| e.to_str()) {
                Some("html") => "text/html",
                Some("js") | Some("mjs") => "text/javascript",
                Some("wasm") => "application/wasm",
                _ => "application/octet-stream",
            };
            let _ = write!(
                stream,
                "HTTP/1.1 200 OK\r\nContent-Type: {kind}\r\nContent-Length: {}\r\n\
                 Cross-Origin-Opener-Policy: same-origin\r\n\
                 Cross-Origin-Embedder-Policy: require-corp\r\n\
                 Cross-Origin-Resource-Policy: same-origin\r\n\
                 Cache-Control: no-store\r\nConnection: close\r\n\r\n",
                body.len()
            );
            let _ = stream.write_all(&body);
        }
        Err(_) => {
            let _ = write!(
                stream,
                "HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\nConnection: close\r\n\r\n"
            );
        }
    }
}

/// A DevTools client on one page the node holds: the harness's view.
pub struct PageClient {
    devtools: DevTools,
    session: String,
}

impl PageClient {
    /// Attach to the page whose URL starts with `url`, at the browser
    /// whose DevTools HTTP endpoint is `endpoint`.
    pub fn attach(endpoint: &str, url: &str) -> Self {
        let ws = browser_ws_url(endpoint, Duration::from_secs(30)).expect("DevTools answers");
        let mut devtools = DevTools::connect(&ws, Duration::from_secs(30)).expect("connects");
        let deadline = Instant::now() + Duration::from_secs(30);
        let target = loop {
            let targets = devtools
                .call(
                    None,
                    "Target.getTargets",
                    json!({}),
                    Duration::from_secs(10),
                )
                .expect("targets");
            let found = targets["targetInfos"].as_array().and_then(|list| {
                list.iter()
                    .find(|t| {
                        t["type"] == "page" && t["url"].as_str().unwrap_or("").starts_with(url)
                    })
                    .and_then(|t| t["targetId"].as_str().map(str::to_string))
            });
            if let Some(found) = found {
                break found;
            }
            assert!(Instant::now() < deadline, "no page at {url}: {targets}");
            std::thread::sleep(Duration::from_millis(50));
        };
        let attached = devtools
            .call(
                None,
                "Target.attachToTarget",
                json!({ "targetId": target, "flatten": true }),
                Duration::from_secs(10),
            )
            .expect("attaches");
        let session = attached["sessionId"]
            .as_str()
            .expect("a session")
            .to_string();
        Self { devtools, session }
    }

    /// Evaluate `expression` in the page's main world; its value.
    pub fn eval(&mut self, expression: &str) -> Value {
        let answer = self
            .devtools
            .call(
                Some(&self.session),
                "Runtime.evaluate",
                json!({ "expression": expression, "returnByValue": true }),
                Duration::from_secs(30),
            )
            .expect("evaluates");
        assert!(
            answer.get("exceptionDetails").is_none(),
            "{expression} threw: {answer}"
        );
        answer["result"]["value"].clone()
    }
}

/// Standard base64, as the page's `btoa` writes it.
pub fn b64(bytes: &[u8]) -> String {
    const A: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut out = String::new();
    for c in bytes.chunks(3) {
        let n = (u32::from(c[0]) << 16)
            | (u32::from(c.get(1).copied().unwrap_or(0)) << 8)
            | u32::from(c.get(2).copied().unwrap_or(0));
        out.push(A[(n >> 18) as usize & 63] as char);
        out.push(A[(n >> 12) as usize & 63] as char);
        out.push(if c.len() > 1 {
            A[(n >> 6) as usize & 63] as char
        } else {
            '='
        });
        out.push(if c.len() > 2 {
            A[n as usize & 63] as char
        } else {
            '='
        });
    }
    out
}

/// Standard base64 decoded.
pub fn unb64(text: &str) -> Vec<u8> {
    let mut out = Vec::new();
    let (mut acc, mut bits) = (0u32, 0);
    for c in text.bytes() {
        let v = match c {
            b'A'..=b'Z' => c - b'A',
            b'a'..=b'z' => c - b'a' + 26,
            b'0'..=b'9' => c - b'0' + 52,
            b'+' => 62,
            b'/' => 63,
            _ => continue,
        };
        acc = (acc << 6) | u32::from(v);
        bits += 6;
        if bits >= 8 {
            bits -= 8;
            out.push((acc >> bits) as u8);
            acc &= (1 << bits) - 1;
        }
    }
    out
}
