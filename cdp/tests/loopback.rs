//! The node without Chrome: a stand-in DevTools endpoint, stepped.
//!
//! Proves what the node exists for — a page's clock advances only while
//! the board's does, a grant a quantum, owed time booked from the page's own
//! clock, a lead paid back, a stuck grant stopping the run, bytes crossing
//! the line both ways as levels, the cable pulled and put back, a port
//! opened at the wrong rate, the drain barrier holding the next grant — with
//! nothing installed, on every CI run.
//!
//! The stand-in ([`Fake`]) is a WebSocket server speaking the DevTools
//! messages the node sends, in flatten mode. It keeps a page clock that
//! advances by exactly what each grant says (plus an overrun when a case
//! asks for one), and plays a small app on it: timed steps — ask for the
//! port, open it, write, pull the cable — sent through the node's binding as
//! the shim would, and the node's answers read from each slice's evaluate.
//! It can also stand for a dedicated worker that owns the port's stream,
//! acknowledging each delivery after a delay of host time. It runs none of
//! the shim's JavaScript: `real_chrome.rs` holds the shim to Chrome.
//!
//! Stepped (`TESTING.md` rule 9): the case's thread is the clock's actor.
//! What is asserted is exact where virtual time owns it — the budgets
//! granted, the instants bytes enter the line, the page's clock against the
//! board's — and host time is never asserted; the wall-time bounds are
//! sized for a hang.

mod common;

use std::collections::HashMap;
use std::io::ErrorKind;
use std::net::{TcpListener, TcpStream};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use common::{b64, bench, ep, finish, suite_lock, unb64, Peer, PeerHandle};
use embsim_board::{Project, Reports};
use embsim_boards::catalog::CatalogSet;
use embsim_cdp::{Browse, CdpNode, NodeStats, Settings, UsbIds};
use embsim_core::virtual_clock::{self, ClockMode};
use rstest::rstest;
use serde_json::{json, Value};
use tungstenite::{Message, WebSocket};
use vibes_behaviour::{behaviour, expect, Test};

/// One millisecond of virtual time.
const MS: u64 = 1_000_000;

/// The page's clock when the stand-in starts it, in milliseconds.
const ORIGIN_MS: f64 = 1_000_000.0;

/// How long, in wall time, a case may wait before it is hung.
const HANG: Duration = Duration::from_secs(120);

// ============================================================
// The stand-in
// ============================================================

/// A step of the stand-in's app, at a page time after its clock's origin.
#[derive(Debug, Clone)]
enum Act {
    /// `navigator.serial.getPorts()`.
    GetPorts,
    /// `port.open()` at a rate, on the port the last `getPorts` gave.
    Open(u32),
    /// `port.open()` on the port of a given generation.
    OpenGen(u64, u32),
    /// A write on the open port.
    Write(Vec<u8>),
    /// `__embsim.link(op)`.
    Link(&'static str),
}

/// What a case asks of the stand-in.
#[derive(Debug, Default)]
struct Script {
    acts: Vec<(f64, Act)>,
    /// At this grant (counting from 1) the clock runs this many ms past it.
    overrun: Option<(usize, f64)>,
    /// From this grant on, budgets never expire.
    stuck_from: Option<usize>,
    /// A dedicated worker owns the port's stream and acknowledges each
    /// delivery after this delay of host time (`None`: it never does).
    worker: Option<Option<Duration>>,
}

/// What the stand-in saw.
#[derive(Debug, Default)]
struct Seen {
    connected: bool,
    grants: Vec<f64>,
    slices: u64,
    /// The page clock at each slice, as the stand-in answered it.
    clocks: Vec<f64>,
    /// Bytes handed to the page while its port was open.
    rx: Vec<u8>,
    /// Every answer to a request, in order, with the step it answered.
    replies: Vec<(String, Value)>,
    /// The device's generation and whether it was plugged, at each change.
    states: Vec<(u64, bool)>,
    opened: bool,
    lost: u64,
    deliveries: u64,
    acks: u64,
    /// Messages the node sent while a delivery waited for its ack.
    early: u64,
}

/// A stand-in DevTools endpoint on a free loopback port.
struct Fake {
    url: String,
    seen: Arc<Mutex<Seen>>,
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

impl Fake {
    fn start(script: Script) -> Self {
        let listener = TcpListener::bind("127.0.0.1:0").expect("a loopback port");
        listener.set_nonblocking(true).expect("non-blocking");
        let url = format!(
            "ws://127.0.0.1:{}/devtools/browser/fake",
            listener.local_addr().unwrap().port()
        );
        let script = Arc::new(Mutex::new(script));
        let seen = Arc::new(Mutex::new(Seen::default()));
        let stop = Arc::new(AtomicBool::new(false));
        let thread = {
            let (script, seen, stop) = (Arc::clone(&script), Arc::clone(&seen), Arc::clone(&stop));
            std::thread::spawn(move || {
                let stream = loop {
                    if stop.load(Ordering::Relaxed) {
                        return;
                    }
                    match listener.accept() {
                        Ok((stream, _)) => break stream,
                        Err(_) => std::thread::sleep(Duration::from_millis(2)),
                    }
                };
                stream.set_nonblocking(false).unwrap();
                stream.set_nodelay(true).unwrap();
                stream
                    .set_read_timeout(Some(Duration::from_millis(50)))
                    .unwrap();
                let ws = tungstenite::accept(stream).expect("the handshake");
                seen.lock().unwrap().connected = true;
                let mut page = FakePage {
                    ws,
                    script,
                    seen,
                    clock: ORIGIN_MS,
                    acts_done: 0,
                    gen: None,
                    open: false,
                    plugged: None,
                    next_id: 1,
                    pending: HashMap::new(),
                    worker_reads: 0,
                };
                page.serve(&stop);
            })
        };
        Self {
            url,
            seen,
            stop,
            thread: Some(thread),
        }
    }

    fn seen(&self) -> std::sync::MutexGuard<'_, Seen> {
        self.seen.lock().unwrap()
    }
}

impl Drop for Fake {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

const PAGE: &str = "P1";
const WORKER: &str = "W1";
const DOC: &str = "fake-doc";
const ORIGIN: &str = "http://fake.test";

struct FakePage {
    ws: WebSocket<TcpStream>,
    script: Arc<Mutex<Script>>,
    seen: Arc<Mutex<Seen>>,
    clock: f64,
    acts_done: usize,
    gen: Option<u64>,
    open: bool,
    plugged: Option<(u64, bool)>,
    next_id: u64,
    pending: HashMap<u64, Act>,
    worker_reads: u64,
}

impl FakePage {
    fn serve(&mut self, stop: &AtomicBool) {
        while !stop.load(Ordering::Relaxed) {
            let message = match self.ws.read() {
                Ok(Message::Text(text)) => text.to_string(),
                Ok(Message::Close(_)) => return,
                Ok(_) => continue,
                Err(tungstenite::Error::Io(e))
                    if matches!(e.kind(), ErrorKind::WouldBlock | ErrorKind::TimedOut) =>
                {
                    continue
                }
                Err(_) => return,
            };
            let m: Value = serde_json::from_str(&message).expect("the node sends JSON");
            self.command(&m);
        }
    }

    fn send(&mut self, value: Value) {
        let _ = self.ws.send(Message::text(value.to_string()));
    }

    fn reply(&mut self, m: &Value, result: Value) {
        let mut answer = json!({ "id": m["id"], "result": result });
        if let Some(session) = m.get("sessionId") {
            answer["sessionId"] = session.clone();
        }
        self.send(answer);
    }

    fn event(&mut self, session: Option<&str>, method: &str, params: Value) {
        let mut event = json!({ "method": method, "params": params });
        if let Some(session) = session {
            event["sessionId"] = json!(session);
        }
        self.send(event);
    }

    /// Something the app says through the node's binding.
    fn binding(&mut self, payload: Value) {
        self.event(
            Some(PAGE),
            "Runtime.bindingCalled",
            json!({ "name": "__embsimTx", "payload": payload.to_string(), "executionContextId": 1 }),
        );
    }

    fn command(&mut self, m: &Value) {
        let method = m["method"].as_str().unwrap_or_default().to_string();
        let session = m["sessionId"].as_str().map(str::to_string);
        match (session.as_deref(), method.as_str()) {
            (None, "Browser.getVersion") => self.reply(m, json!({ "product": "FakeChrome/1.0" })),
            // Chrome sends the attach events for the targets it already has
            // before it answers (measured, Chrome 153).
            (None, "Target.setAutoAttach") => {
                self.event(
                    None,
                    "Target.attachedToTarget",
                    json!({
                        "sessionId": PAGE,
                        "targetInfo": { "targetId": "T1", "type": "page", "url": "about:blank" },
                        "waitingForDebugger": true,
                    }),
                );
                self.reply(m, json!({}));
            }
            (Some(PAGE), "Target.setAutoAttach") => {
                if self.script.lock().unwrap().worker.is_some() {
                    self.event(
                        Some(PAGE),
                        "Target.attachedToTarget",
                        json!({
                            "sessionId": WORKER,
                            "targetInfo": { "targetId": "T2", "type": "worker", "url": "w.js" },
                            "waitingForDebugger": true,
                        }),
                    );
                }
                self.reply(m, json!({}));
            }
            (Some(PAGE), "Runtime.runIfWaitingForDebugger") => {
                self.reply(m, json!({}));
                self.binding(json!({ "k": "hello", "doc": DOC, "origin": ORIGIN }));
            }
            (Some(PAGE), "Emulation.setVirtualTimePolicy") => {
                match m["params"]["budget"].as_f64() {
                    Some(budget) => self.grant(m, budget),
                    None => self.reply(m, json!({})),
                }
            }
            (Some(PAGE), "Runtime.evaluate") => self.slice(m),
            (Some(WORKER), "Runtime.evaluate") => {
                self.reply(m, json!({ "result": { "type": "number", "value": 1 } }))
            }
            _ => self.reply(m, json!({})),
        }
    }

    fn grant(&mut self, m: &Value, budget: f64) {
        let n = {
            let mut seen = self.seen.lock().unwrap();
            seen.grants.push(budget);
            seen.grants.len()
        };
        self.reply(m, json!({ "virtualTimeTicksBase": 0 }));
        let (stuck, extra) = {
            let script = self.script.lock().unwrap();
            (
                script.stuck_from.is_some_and(|from| n >= from),
                script
                    .overrun
                    .filter(|(at, _)| *at == n)
                    .map_or(0.0, |(_, ms)| ms),
            )
        };
        if stuck {
            return;
        }
        let to = self.clock + budget + extra;
        // The app's steps whose time has come, as the page lives the grant.
        loop {
            let next = {
                let script = self.script.lock().unwrap();
                script.acts.get(self.acts_done).cloned()
            };
            let Some((at, act)) = next else { break };
            if ORIGIN_MS + at > to {
                break;
            }
            self.acts_done += 1;
            self.act(act);
        }
        self.clock = to;
        self.event(Some(PAGE), "Emulation.virtualTimeBudgetExpired", json!({}));
    }

    fn request(&mut self, act: Act, op: &str, extra: Value) {
        let id = self.next_id;
        self.next_id += 1;
        self.pending.insert(id, act);
        let mut payload = json!({ "k": "req", "doc": DOC, "id": id, "op": op, "origin": ORIGIN });
        for (key, value) in extra.as_object().unwrap() {
            payload[key] = value.clone();
        }
        self.binding(payload);
    }

    fn act(&mut self, act: Act) {
        match act.clone() {
            Act::GetPorts => self.request(act, "getPorts", json!({})),
            Act::Open(baud) => {
                let gen = self.gen.unwrap_or(u64::MAX);
                self.request(act, "open", open_options(gen, baud));
            }
            Act::OpenGen(gen, baud) => self.request(act, "open", open_options(gen, baud)),
            Act::Write(bytes) => {
                self.binding(json!({ "k": "tx", "doc": DOC, "b": b64(&bytes) }));
            }
            Act::Link(op) => self.binding(json!({ "k": "link", "doc": DOC, "op": op })),
        }
    }

    fn slice(&mut self, m: &Value) {
        let expression = m["params"]["expression"].as_str().unwrap_or_default();
        let start =
            expression.find("const a = ").expect("the slice's argument") + "const a = ".len();
        let end = expression
            .find("; return")
            .expect("the slice's argument's end");
        let arg: Value =
            serde_json::from_str(&expression[start..end]).expect("the argument is JSON");
        let state = (
            arg["gen"].as_u64().unwrap(),
            arg["plugged"].as_bool().unwrap(),
        );
        {
            let mut seen = self.seen.lock().unwrap();
            seen.slices += 1;
            seen.clocks.push(self.clock);
            if self.plugged != Some(state) {
                seen.states.push(state);
            }
            if let Some((gen, true)) = self.plugged {
                if !state.1 || state.0 != gen {
                    seen.lost += 1;
                    self.open = false;
                }
            }
        }
        self.plugged = Some(state);
        let mut opened = false;
        for reply in arg["replies"].as_array().cloned().unwrap_or_default() {
            let act = self.pending.remove(&reply["id"].as_u64().unwrap());
            let name = match &act {
                Some(Act::GetPorts) => "getPorts",
                Some(Act::Open(_)) | Some(Act::OpenGen(..)) => "open",
                _ => "other",
            };
            if reply["ok"] == true {
                match act {
                    Some(Act::GetPorts) => {
                        self.gen = reply["v"]
                            .as_array()
                            .and_then(|v| v.first())
                            .and_then(Value::as_u64)
                    }
                    Some(Act::Open(_)) | Some(Act::OpenGen(..)) => {
                        self.open = true;
                        opened = true;
                    }
                    _ => {}
                }
            }
            self.seen
                .lock()
                .unwrap()
                .replies
                .push((name.to_string(), reply));
        }
        let mut reads = 0;
        if let (Some(rx), true) = (arg["rx"].as_str(), self.open) {
            let bytes = unb64(rx);
            reads = bytes.len().div_ceil(255) as u64;
            let mut seen = self.seen.lock().unwrap();
            seen.rx.extend(bytes);
            seen.deliveries += 1;
        }
        let worker = self.script.lock().unwrap().worker;
        let answer = json!({
            "t": self.clock,
            "doc": DOC,
            "reading": self.open,
            "locked": self.open && worker.is_some(),
            "reads": reads,
            "pageReads": 0,
        });
        self.reply(
            m,
            json!({ "result": { "type": "object", "value": answer } }),
        );
        if opened {
            self.seen.lock().unwrap().opened = true;
            self.binding(json!({ "k": "reading", "doc": DOC }));
        }
        if reads > 0 {
            if let Some(delay) = worker {
                self.worker_ack(reads, delay);
            }
        }
    }

    /// The worker reads what it was handed: after `delay` of host time it
    /// comes back for more. Anything the node sends meanwhile is early.
    fn worker_ack(&mut self, reads: u64, delay: Option<Duration>) {
        let until = Instant::now() + delay.unwrap_or(Duration::from_millis(100));
        let stream = self.ws.get_ref();
        stream.set_nonblocking(true).unwrap();
        let mut early = false;
        while Instant::now() < until {
            let mut byte = [0u8; 1];
            if matches!(stream.peek(&mut byte), Ok(n) if n > 0) {
                early = true;
                break;
            }
            std::thread::sleep(Duration::from_micros(200));
        }
        stream.set_nonblocking(false).unwrap();
        let mut seen = self.seen.lock().unwrap();
        if early && delay.is_some() {
            seen.early += 1;
        }
        if delay.is_none() {
            return;
        }
        seen.acks += 1;
        drop(seen);
        self.worker_reads += reads;
        let n = self.worker_reads;
        self.event(
            Some(WORKER),
            "Runtime.bindingCalled",
            json!({ "name": "__embsimDrain", "payload": n.to_string(), "executionContextId": 2 }),
        );
    }
}

fn open_options(gen: u64, baud: u32) -> Value {
    json!({
        "gen": gen, "baudRate": baud, "dataBits": 8, "stopBits": 1, "parity": "none",
        "bufferSize": 255, "flowControl": "none",
    })
}

// ============================================================
// Plumbing
// ============================================================

fn node(fake: &Fake, baud: u32) -> CdpNode {
    let mut settings = Settings::new(Browse::Attach(fake.url.clone()));
    settings.usb = UsbIds {
        vendor: Some(0x0403),
        product: Some(0x6001),
    };
    settings.granted = true;
    CdpNode::new(settings, baud)
}

fn node_with(fake: &Fake, baud: u32, set: impl FnOnce(&mut Settings)) -> CdpNode {
    let mut settings = Settings::new(Browse::Attach(fake.url.clone()));
    settings.granted = true;
    set(&mut settings);
    CdpNode::new(settings, baud)
}

/// Hand the board time until `done`, or the case is hung.
fn until(stats: &NodeStats, what: &str, mut done: impl FnMut() -> bool) {
    let started = Instant::now();
    while !done() {
        assert!(
            started.elapsed() < HANG,
            "{what} never happened by {} ns of virtual time: {stats:?}",
            virtual_clock::virtual_ns()
        );
        virtual_clock::wait_virtual_ns(MS);
    }
}

/// A peer on the board's side that does not echo.
fn peer(baud: u32) -> (Box<Peer>, PeerHandle) {
    let (peer, handle) = Peer::new(baud, false);
    (Box::new(peer), handle)
}

// ============================================================
// Metering
// ============================================================

#[test]
fn nothing_starts_while_the_board_is_held() {
    behaviour!(Test {
        id: "chrome-cdp.nothing-while-held",
        covers: Some("cdp/src/node.rs#CdpNode"),
        given: "a host's Chrome on the board, set to attach to a DevTools endpoint, in a \
                bench started with its clock held and stopped again, as embsim check does",
    });
    expect!(
        "no-connection",
        "nothing connects to the endpoint: the browser is reached at the first slice, a \
         quantum after the board's clock starts"
    );

    let _suite = suite_lock();
    let fake = Fake::start(Script::default());
    virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);
    let system = embsim_board::System::new()
        .component("PC", Box::new(node(&fake, 115_200)))
        .hold_time()
        .start()
        .expect("the bench starts");
    std::thread::sleep(Duration::from_millis(100));
    system.shutdown();
    assert!(!fake.seen().connected, "no-connection");
}

#[test]
fn the_page_lives_the_boards_time_a_quantum_at_a_time() {
    behaviour!(Test {
        id: "chrome-cdp.metered",
        covers: Some("cdp/src/node.rs#CdpNode"),
        given: "a host's Chrome on the board metered every millisecond, its page's clock \
                advancing by exactly what it is granted, on the stepped clock",
    });
    expect!(
        "frozen-while-held",
        "while the case holds the board's clock, the page is granted nothing, however much \
         host time passes"
    );
    expect!(
        "one-quantum-a-slice",
        "once its clock is first read, the page is granted exactly one quantum at each slice"
    );
    expect!(
        "level-with-the-board",
        "the page's clock, as the node books it, has lived exactly the board's time since it \
         was first read"
    );

    let _suite = suite_lock();
    let fake = Fake::start(Script::default());
    let node = node(&fake, 115_200);
    let stats = node.stats();
    let (peer, _) = peer(115_200);
    let (system, actor) = bench(node, peer);
    let origin = virtual_clock::virtual_ns();

    std::thread::sleep(Duration::from_millis(100));
    assert!(fake.seen().grants.is_empty(), "frozen-while-held");

    virtual_clock::wait_until_ns(origin + 20 * MS);
    let seen = fake.seen();
    assert!(
        seen.grants.len() >= 15,
        "one-quantum-a-slice: {:?}",
        seen.grants
    );
    assert!(
        seen.grants.iter().all(|&budget| budget == 1.0),
        "one-quantum-a-slice: {:?}",
        seen.grants
    );
    assert_eq!(
        seen.grants.len() as u64 + 1,
        stats.slices(),
        "one-quantum-a-slice"
    );
    drop(seen);
    assert_eq!(
        stats.board_ns(),
        stats.lived_ns(),
        "level-with-the-board: {stats:?}"
    );
    assert_eq!(
        stats.granted_ns(),
        stats.board_ns(),
        "level-with-the-board: {stats:?}"
    );
    assert_eq!(stats.peak_lead_ns(), 0, "level-with-the-board");
    assert_eq!(stats.failure(), None);
    finish(system, actor);
}

#[test]
fn a_page_ahead_of_the_board_is_paid_back_by_skipped_grants() {
    behaviour!(Test {
        id: "chrome-cdp.lead-paid-back",
        covers: Some("cdp/src/node.rs#NodeStats::peak_lead_ns"),
        given: "a page metered every millisecond whose clock runs 5 ms past its third grant, \
                as Chrome's does at a worker's birth or a storage call",
    });
    expect!(
        "lead-reported",
        "the node reports the page 5 ms ahead of the board, and a clock that passed its \
         budget by 5 ms"
    );
    expect!(
        "paid-back",
        "the page is granted nothing for the next five slices, then a quantum a slice again, \
         and ends level with the board"
    );

    let _suite = suite_lock();
    let fake = Fake::start(Script {
        overrun: Some((3, 5.0)),
        ..Script::default()
    });
    let node = node(&fake, 115_200);
    let stats = node.stats();
    let (peer, _) = peer(115_200);
    let (system, actor) = bench(node, peer);
    let origin = virtual_clock::virtual_ns();
    virtual_clock::wait_until_ns(origin + 30 * MS);

    assert_eq!(stats.peak_lead_ns(), 5 * MS, "lead-reported: {stats:?}");
    assert_eq!(stats.peak_overrun_ns(), 5 * MS, "lead-reported: {stats:?}");
    assert_eq!(stats.skipped(), 5, "paid-back: {stats:?}");
    let seen = fake.seen();
    assert!(
        seen.grants.iter().all(|&b| b == 1.0),
        "paid-back: {:?}",
        seen.grants
    );
    assert_eq!(
        seen.grants.len() as u64 + 1 + 5,
        stats.slices(),
        "paid-back"
    );
    drop(seen);
    assert_eq!(stats.lead_ns(), 0, "paid-back: {stats:?}");
    assert_eq!(stats.failure(), None);
    finish(system, actor);
}

#[test]
fn a_lead_past_max_lead_stops_the_run_saying_so() {
    behaviour!(Test {
        id: "chrome-cdp.max-lead",
        covers: Some("cdp/src/node.rs#Settings::max_lead"),
        given: "a page allowed to lead the board by 2 ms whose clock runs 5 ms past its third \
                grant",
    });
    expect!(
        "failure-says-how-far",
        "the node stops with a failure saying how far ahead of the board the page's clock \
         ran and the bound it passed"
    );
    expect!("no-more-grants", "the page is granted nothing after that");

    let _suite = suite_lock();
    let fake = Fake::start(Script {
        overrun: Some((3, 5.0)),
        ..Script::default()
    });
    let node = node_with(&fake, 115_200, |s| {
        s.max_lead = Some(Duration::from_millis(2))
    });
    let stats = node.stats();
    let (peer, _) = peer(115_200);
    let (system, actor) = bench(node, peer);
    virtual_clock::wait_virtual_ns(15 * MS);
    let failure = stats
        .failure()
        .expect("failure-says-how-far: the node failed");
    assert!(
        failure.starts_with(
            "a page's clock ran 5.000 ms ahead of the board, past the 2.000 ms the node allows"
        ),
        "failure-says-how-far: {failure}"
    );
    assert_eq!(fake.seen().grants.len(), 3, "no-more-grants");
    virtual_clock::wait_virtual_ns(10 * MS);
    assert_eq!(fake.seen().grants.len(), 3, "no-more-grants");
    finish(system, actor);
}

#[test]
fn a_grant_that_never_expires_stops_the_run_saying_why() {
    behaviour!(Test {
        id: "chrome-cdp.stuck-grant",
        covers: Some("cdp/src/browser.rs#Browser::grant"),
        given: "a page whose budgets stop expiring from its third grant, the node allowed to \
                wait 300 ms of host time for one",
    });
    expect!(
        "failure-says-why",
        "the node stops, saying the grant stuck, its budget, and that a fetch that never \
         completes or a task that never ends holds a budget"
    );
    expect!("no-more-grants", "no grant is sent after the stuck one");

    let _suite = suite_lock();
    let fake = Fake::start(Script {
        stuck_from: Some(3),
        ..Script::default()
    });
    let node = node_with(&fake, 115_200, |s| {
        s.stuck_after = Duration::from_millis(300)
    });
    let stats = node.stats();
    let (peer, _) = peer(115_200);
    let (system, actor) = bench(node, peer);
    virtual_clock::wait_virtual_ns(10 * MS);
    let failure = stats.failure().expect("failure-says-why: the node failed");
    assert!(
        failure.starts_with("a grant stuck: the page at http://fake.test was granted 1.000 ms")
            && failure.contains("a fetch that never completes or a task that never ends"),
        "failure-says-why: {failure}"
    );
    assert_eq!(stats.stuck(), 1, "failure-says-why");
    assert_eq!(fake.seen().grants.len(), 3, "no-more-grants");
    virtual_clock::wait_virtual_ns(10 * MS);
    assert_eq!(fake.seen().grants.len(), 3, "no-more-grants");
    finish(system, actor);
}

// ============================================================
// Bytes
// ============================================================

/// Open the port: ask for it at 1 ms of page time, open it at 3 ms.
fn opening(baud: u32) -> Vec<(f64, Act)> {
    vec![(1.0, Act::GetPorts), (3.0, Act::Open(baud))]
}

#[test]
fn bytes_cross_both_ways_as_levels_at_slice_instants() {
    behaviour!(Test {
        id: "chrome-cdp.duplex",
        covers: Some("cdp/src/node.rs#CdpNode"),
        given: "a page that opens its port at 115200 baud and writes twelve bytes, and a peer \
                on the board's side of the line that writes eleven",
    });
    expect!(
        "page-to-board",
        "the peer hears the page's twelve bytes, in order, with no framing error"
    );
    expect!(
        "at-a-slice",
        "the page's first byte starts on the wire at a slice, a whole number of quanta after \
         the start"
    );
    expect!(
        "board-to-page",
        "the page is handed the peer's eleven bytes, in order, and the node counts each way"
    );

    let _suite = suite_lock();
    let mut acts = opening(115_200);
    acts.push((10.0, Act::Write(b"hello, board".to_vec())));
    let fake = Fake::start(Script {
        acts,
        ..Script::default()
    });
    let node = node(&fake, 115_200);
    let stats = node.stats();
    let (peer, heard) = peer(115_200);
    let (system, actor) = bench(node, peer);
    let origin = virtual_clock::virtual_ns();
    until(&stats, "the open", || fake.seen().opened);
    heard.send(b"hello, page");
    until(&stats, "both ways", || {
        heard.heard().len() >= 12 && fake.seen().rx.len() >= 11
    });
    virtual_clock::wait_virtual_ns(5 * MS);

    assert_eq!(heard.heard(), b"hello, board", "page-to-board");
    assert_eq!(
        heard.0.framing_errors.load(Ordering::Relaxed),
        0,
        "page-to-board"
    );
    let frame_ns = 10 * 1_000_000_000 / 115_200;
    let (first, _) = heard.heard_at()[0];
    let into_quantum = (first - origin) % MS;
    assert!(
        into_quantum <= frame_ns + 1,
        "at-a-slice: the first frame ended {into_quantum} ns into a quantum, a frame is \
         {frame_ns} ns"
    );
    assert_eq!(fake.seen().rx, b"hello, page", "board-to-page");
    assert_eq!(stats.from_page(), 12, "board-to-page: {stats:?}");
    assert_eq!(stats.to_page(), 11, "board-to-page: {stats:?}");
    assert_eq!(stats.shed(), 0, "board-to-page: {stats:?}");
    assert_eq!(stats.failure(), None);
    finish(system, actor);
}

#[test]
fn a_burst_each_way_arrives_whole_and_in_order() {
    behaviour!(Test {
        id: "chrome-cdp.burst",
        covers: Some("cdp/src/node.rs#CdpNode"),
        given: "a page that writes 4096 bytes in one write at 115200 baud while a peer on the \
                board's side writes 4096 of its own",
    });
    expect!(
        "page-burst-whole",
        "the peer hears all 4096 of the page's bytes, in order, none shed"
    );
    expect!(
        "board-burst-whole",
        "the page is handed all 4096 of the peer's bytes, in order"
    );

    let _suite = suite_lock();
    let ours: Vec<u8> = (0..4096u32).map(|i| (i * 13 + 5) as u8).collect();
    let theirs: Vec<u8> = (0..4096u32).map(|i| (i * 7 + 1) as u8).collect();
    let mut acts = opening(115_200);
    acts.push((10.0, Act::Write(ours.clone())));
    let fake = Fake::start(Script {
        acts,
        ..Script::default()
    });
    let node = node(&fake, 115_200);
    let stats = node.stats();
    let (peer, heard) = peer(115_200);
    let (system, actor) = bench(node, peer);
    until(&stats, "the open", || fake.seen().opened);
    heard.send(&theirs);
    until(&stats, "both bursts", || {
        heard.heard().len() >= ours.len() && fake.seen().rx.len() >= theirs.len()
    });
    assert_eq!(heard.heard(), ours, "page-burst-whole");
    assert_eq!(stats.shed(), 0, "page-burst-whole: {stats:?}");
    assert_eq!(fake.seen().rx, theirs, "board-burst-whole");
    assert_eq!(stats.failure(), None);
    finish(system, actor);
}

#[test]
fn a_pulled_cable_carries_nothing_and_a_replugged_one_is_a_new_port() {
    behaviour!(Test {
        id: "chrome-cdp.unplug-replug",
        covers: Some("cdp/src/node.rs#LinkControl"),
        given: "a page with its port open that pulls the cable, writes, puts the cable back, \
                tries its old port, then opens the port it is given; the board writes while \
                the cable is out and once it is back",
    });
    expect!(
        "page-told",
        "the page is told the device is unplugged at the next slice, and its open port is \
         gone"
    );
    expect!(
        "nothing-crosses",
        "while the cable is out, the page's write reaches no wire and the board's bytes reach \
         no page: both are counted"
    );
    expect!(
        "new-generation",
        "once the cable is back, the old port fails to open and the new one opens and carries \
         bytes"
    );

    let _suite = suite_lock();
    let mut acts = opening(115_200);
    acts.extend([
        (10.0, Act::Link("unplug")),
        (12.0, Act::Write(b"lost".to_vec())),
        (20.0, Act::Link("plug")),
        (22.0, Act::OpenGen(0, 115_200)),
        (24.0, Act::GetPorts),
        (27.0, Act::Open(115_200)),
    ]);
    let fake = Fake::start(Script {
        acts,
        ..Script::default()
    });
    let node = node(&fake, 115_200);
    let stats = node.stats();
    let (peer, heard) = peer(115_200);
    let (system, actor) = bench(node, peer);
    until(&stats, "the unplug", || stats.unplugs() == 1);
    virtual_clock::wait_virtual_ns(MS);
    assert_eq!(fake.seen().states.last(), Some(&(0, false)), "page-told");
    assert_eq!(fake.seen().lost, 1, "page-told");
    heard.send(b"gone");
    until(&stats, "the plug", || stats.plugs() == 1);
    until(&stats, "the new port's open", || {
        fake.seen()
            .replies
            .iter()
            .filter(|(name, r)| name == "open" && r["ok"] == true)
            .count()
            == 2
    });
    heard.send(b"back");
    until(&stats, "the bytes after", || {
        fake.seen().rx.ends_with(b"back")
    });

    assert_eq!(stats.unheard(), 4, "nothing-crosses: {stats:?}");
    assert_eq!(stats.shed(), 4, "nothing-crosses: {stats:?}");
    assert!(
        !fake.seen().rx.windows(4).any(|w| w == b"gone"),
        "nothing-crosses"
    );
    assert!(
        heard.heard().is_empty(),
        "nothing-crosses: {:?}",
        heard.heard()
    );
    let seen = fake.seen();
    assert_eq!(
        seen.states,
        [(0, true), (0, false), (1, true)],
        "new-generation"
    );
    let opens: Vec<&Value> = seen
        .replies
        .iter()
        .filter(|(name, _)| name == "open")
        .map(|(_, r)| r)
        .collect();
    assert_eq!(opens.len(), 3, "new-generation: {opens:?}");
    assert_eq!(opens[1]["ok"], false, "new-generation: {opens:?}");
    assert_eq!(opens[1]["name"], "NetworkError", "new-generation");
    assert_eq!(
        opens[1]["message"], "Failed to open serial port.",
        "new-generation"
    );
    assert_eq!(opens[2]["ok"], true, "new-generation: {opens:?}");
    drop(seen);
    assert_eq!(stats.failure(), None);
    finish(system, actor);
}

#[test]
fn the_cable_pulled_by_the_harness_acts_at_the_next_slice() {
    behaviour!(Test {
        id: "chrome-cdp.link-control",
        covers: Some("cdp/src/node.rs#LinkControl"),
        given: "a page with its port open, the cable pulled and put back through the node's \
                link handle by the test's own thread",
    });
    expect!(
        "unplugged-next-slice",
        "the page is told the device is unplugged at the next slice"
    );
    expect!(
        "replugged-next-slice",
        "the page is told of a new port at the slice after the cable is put back"
    );

    let _suite = suite_lock();
    let fake = Fake::start(Script {
        acts: opening(115_200),
        ..Script::default()
    });
    let node = node(&fake, 115_200);
    let stats = node.stats();
    let link = node.link();
    let (peer, _) = peer(115_200);
    let (system, actor) = bench(node, peer);
    until(&stats, "the open", || fake.seen().opened);
    link.unplug();
    virtual_clock::wait_virtual_ns(2 * MS);
    assert_eq!(
        fake.seen().states.last(),
        Some(&(0, false)),
        "at-the-next-slice"
    );
    assert_eq!(fake.seen().lost, 1, "at-the-next-slice");
    link.plug();
    virtual_clock::wait_virtual_ns(2 * MS);
    assert_eq!(
        fake.seen().states,
        [(0, true), (0, false), (1, true)],
        "at-the-next-slice"
    );
    assert_eq!(
        (stats.unplugs(), stats.plugs()),
        (1, 1),
        "at-the-next-slice"
    );
    assert_eq!(stats.failure(), None);
    finish(system, actor);
}

#[test]
fn a_port_opened_at_the_wrong_rate_is_counted_and_its_bytes_shed() {
    behaviour!(Test {
        id: "chrome-cdp.wrong-baud",
        covers: Some("cdp/src/node.rs#NodeStats::mismatched_opens"),
        given: "a page that opens its port at 9600 baud on a line that runs at 115200, writes \
                three bytes, and is written three by the board's side",
    });
    expect!(
        "open-succeeds",
        "the open succeeds, as a real port opens at whatever rate it is asked"
    );
    expect!(
        "counted-and-shed",
        "the node counts one open at the wrong rate and sheds the six bytes: the board hears \
         none and the page is handed none"
    );

    let _suite = suite_lock();
    let mut acts = opening(9_600);
    acts.push((10.0, Act::Write(b"abc".to_vec())));
    let fake = Fake::start(Script {
        acts,
        ..Script::default()
    });
    let node = node(&fake, 115_200);
    let stats = node.stats();
    let (peer, heard) = peer(115_200);
    let (system, actor) = bench(node, peer);
    until(&stats, "the open", || fake.seen().opened);
    heard.send(b"xyz");
    until(&stats, "the page's write", || stats.mismatched_bytes() >= 6);
    virtual_clock::wait_virtual_ns(5 * MS);
    assert_eq!(stats.mismatched_opens(), 1, "counted-and-shed: {stats:?}");
    assert_eq!(stats.mismatched_bytes(), 6, "counted-and-shed: {stats:?}");
    assert_eq!(stats.shed(), 6, "counted-and-shed: {stats:?}");
    assert!(heard.heard().is_empty(), "counted-and-shed");
    assert!(fake.seen().rx.is_empty(), "counted-and-shed");
    assert_eq!(stats.failure(), None);
    finish(system, actor);
}

// ============================================================
// The drain barrier
// ============================================================

#[test]
fn the_next_grant_waits_until_the_worker_has_read_what_it_was_handed() {
    behaviour!(Test {
        id: "chrome-cdp.drain-barrier",
        covers: Some("cdp/src/browser.rs#Browser::drain"),
        given: "a page whose port's stream is owned by a dedicated worker that comes back for \
                more 30 ms of host time after each delivery, the board's side writing five \
                times",
    });
    expect!(
        "waits-for-the-worker",
        "after each delivery the node sends nothing until the worker has come back for more"
    );
    expect!(
        "every-delivery-waited",
        "the node counts a wait for every delivery and none that ran out"
    );

    let _suite = suite_lock();
    let fake = Fake::start(Script {
        acts: opening(115_200),
        worker: Some(Some(Duration::from_millis(30))),
        ..Script::default()
    });
    let node = node(&fake, 115_200);
    let stats = node.stats();
    let (peer, heard) = peer(115_200);
    let (system, actor) = bench(node, peer);
    until(&stats, "the open", || fake.seen().opened);
    for round in 0..5u8 {
        heard.send(&[round; 3]);
        until(&stats, "a delivery", || {
            fake.seen().deliveries > u64::from(round)
        });
        virtual_clock::wait_virtual_ns(2 * MS);
    }
    let seen = fake.seen();
    assert!(seen.deliveries >= 5, "{seen:?}");
    assert_eq!(seen.early, 0, "waits-for-the-worker: {seen:?}");
    assert_eq!(seen.acks, seen.deliveries, "waits-for-the-worker");
    assert_eq!(
        stats.drain_waits(),
        seen.deliveries,
        "every-delivery-waited: {stats:?}"
    );
    drop(seen);
    assert_eq!(
        stats.drain_timeouts(),
        0,
        "every-delivery-waited: {stats:?}"
    );
    assert_eq!(stats.failure(), None);
    finish(system, actor);
}

#[test]
fn a_consumer_that_never_comes_back_turns_the_barrier_off_after_three_waits() {
    behaviour!(Test {
        id: "chrome-cdp.drain-barrier-off",
        covers: Some("cdp/src/node.rs#DRAIN_STRIKES"),
        given: "a page whose port's stream is held by a consumer that never reports reading \
                (a pipe the probe cannot see), the board's side writing five times",
    });
    expect!(
        "three-waits-run-out",
        "the first three deliveries each wait the barrier's bound and run out"
    );
    expect!(
        "then-off",
        "the barrier is then off for that page: later deliveries go straight on, and the bytes \
         keep arriving"
    );

    let _suite = suite_lock();
    let fake = Fake::start(Script {
        acts: opening(115_200),
        worker: Some(None),
        ..Script::default()
    });
    let node = node(&fake, 115_200);
    let stats = node.stats();
    let (peer, heard) = peer(115_200);
    let (system, actor) = bench(node, peer);
    until(&stats, "the open", || fake.seen().opened);
    for round in 0..5u8 {
        heard.send(&[round; 3]);
        until(&stats, "a delivery", || {
            fake.seen().deliveries > u64::from(round)
        });
        virtual_clock::wait_virtual_ns(2 * MS);
    }
    assert_eq!(stats.drain_timeouts(), 3, "three-waits-run-out: {stats:?}");
    assert_eq!(stats.drain_waits(), 3, "then-off: {stats:?}");
    assert_eq!(stats.drain_off(), 1, "then-off: {stats:?}");
    assert_eq!(fake.seen().rx.len(), 15, "then-off");
    assert_eq!(stats.failure(), None);
    finish(system, actor);
}

// ============================================================
// The kind
// ============================================================

fn set() -> CatalogSet {
    let mut set = CatalogSet::new();
    embsim_cdp::catalog::register(&mut set).expect("the kind registers");
    set
}

#[rstest]
#[case::no_baud("", "options.baud is the serial line's rate")]
#[case::zero_baud("baud = 0", "options.baud = 0 is not a rate")]
#[case::zero_quantum(
    "baud = 9600\nquantum = \"0ms\"",
    "options.quantum is how much virtual time"
)]
#[case::long_quantum(
    "baud = 9600\nquantum = \"2s\"",
    "options.quantum is at most 1000.000000 ms"
)]
#[case::zero_lead("baud = 9600\nmax_lead = \"0ms\"", "options.max_lead is how far")]
#[case::bad_usb(
    "baud = 9600\nusb_vendor_id = 70000",
    "options.usb_vendor_id = 70000 is not a 16-bit USB id"
)]
#[case::both(
    "baud = 9600\nattach = \"http://127.0.0.1:9222\"\nheadless = false",
    "options.headless says how to launch Chrome"
)]
#[case::bad_attach("baud = 9600\nattach = \"127.0.0.1:9222\"", "is a DevTools endpoint")]
#[case::no_chrome("baud = 9600\nchrome = \"./no-such-chrome\"", "no Chrome at ")]
#[case::bad_port(
    "baud = 9600\nattach = \"http://x\"\ndevtools_port = 0",
    "options.devtools_port says how to launch Chrome"
)]
#[case::not_a_bool("baud = 9600\ngranted = \"yes\"", "options.granted is true or false")]
#[case::unknown("baud = 9600\nparity = \"even\"", "unknown option \"parity\"")]
fn a_chrome_cdp_entry_that_cannot_be_built_is_refused_saying_why(
    #[case] options: &str,
    #[case] says: &str,
) {
    behaviour!(Test {
        id: "chrome-cdp.refusals",
        covers: Some("cdp/src/catalog.rs#chrome_cdp"),
        given: "a chrome-cdp component whose options cannot be built: a missing or zero rate, \
                a zero or over-long quantum, a zero lead bound, a 17-bit USB id, a launch \
                option beside an attach, a bad endpoint, no Chrome at the path, or an unknown \
                option",
    });
    expect!(
        "refused-saying-why",
        "the project is refused before anything starts, naming the component and what to fix"
    );
    let text = format!(
        "[[component]]\nname = \"PC\"\nkind = \"chrome-cdp\"\n[component.options]\n{options}\n"
    );
    let message = Project::parse(&text)
        .expect("the text is a project")
        .instantiate(&set())
        .expect_err("the entry is refused")
        .to_string();
    assert!(
        message.contains("component PC (kind \"chrome-cdp\")") || message.contains("PC"),
        "{message}"
    );
    assert!(message.contains(says), "refused-saying-why: {message}");
}

#[test]
fn a_project_names_the_host_chrome_and_its_report_says_how_it_was_metered() {
    behaviour!(Test {
        id: "chrome-cdp.project",
        covers: Some("cdp/src/catalog.rs#CdpReport"),
        given: "a project that wires a chrome-cdp component, attached to a DevTools endpoint, \
                to a host-serial port on one 3.3 V rail, run for 20 ms of board time",
    });
    expect!(
        "first-look",
        "its first look says which browser it reaches, the line's rate, the quantum, and that \
         the browser is reached at the first slice"
    );
    expect!(
        "reached",
        "a later look says which browser answered and where its DevTools are"
    );
    expect!(
        "summary",
        "its summary says how many slices ran, what the page lived of the board's time, its \
         peak lead, and the bytes each way"
    );

    let _suite = suite_lock();
    let fake = Fake::start(Script::default());
    let pty = std::env::temp_dir().join(format!("embsim-cdp-project-{}.pty", std::process::id()));
    let text = format!(
        r#"[[component]]
name = "PC"
kind = "chrome-cdp"
[component.options]
baud = 115200
attach = "{url}"
usb_vendor_id = 0x0403
usb_product_id = 0x6001

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
        url = fake.url
    );
    virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);
    let reports = Reports::new();
    let system = Project::parse(&text)
        .expect("the text is a project")
        .instantiate_with(&set(), &reports)
        .expect("the bench builds");
    let system = system.hold_time().start().expect("the bench starts");
    let mut reports = reports.take();
    let report = reports
        .iter_mut()
        .find(|report| report.subject() == "PC")
        .expect("the component reports");
    let first = report.look(0);
    let actor = virtual_clock::register_actor("cdp-project-case");
    system.release_time();
    virtual_clock::wait_virtual_ns(20 * MS);
    let later = report.look(20 * MS);
    let summary = report.summary();
    assert_eq!(
        first,
        [format!(
            "the Chrome at {}, 115200 baud 8N1, metered every 1.000000 ms of virtual time; \
             Chrome is reached at the first slice",
            fake.url
        )],
        "first-look"
    );
    assert_eq!(later.len(), 1, "reached: {later:?}");
    assert!(
        later[0].starts_with("FakeChrome/1.0 reached in ")
            && later[0].contains("the board's clock held at ")
            && later[0].ends_with(&format!(
                "DevTools at {}; its pages live only while the board's clock advances",
                fake.url
            )),
        "reached: {later:?}"
    );
    assert!(
        summary[0].contains(" slices (")
            && summary[0].contains("the page lived ")
            && summary[0].contains(" of the board's "),
        "summary: {summary:?}"
    );
    assert!(
        summary[1].starts_with("at most 0.000 ms ahead of the board"),
        "summary: {summary:?}"
    );
    assert!(
        summary
            .iter()
            .any(|line| line == "0 bytes from the page, 0 to it, 0 framing errors"),
        "summary: {summary:?}"
    );
    assert_eq!(report.failure(), None);
    drop(actor);
    system.shutdown();
    let _ = ep("PC.TX");
}
