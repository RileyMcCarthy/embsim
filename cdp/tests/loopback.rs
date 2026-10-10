//! The node without Chrome: a stand-in DevTools endpoint, stepped.
//!
//! Proves what the node exists for — a page's clock advances only while
//! the board's does, a grant a quantum, owed time booked from what the page
//! was granted, a lead paid back, a new document level with the board, a
//! stuck grant and a page that does not return stopping the run, a page
//! that closes or crashes mid-grant let go at once, pages that share one
//! clock granted once, a hidden page poked into answering, bytes crossing
//! the line both ways as levels, the port's requests, the cable pulled and
//! put back, a port opened at the wrong rate, the drain barrier holding
//! the next grant — with nothing installed, on every CI run.
//!
//! The stand-in ([`Fake`]) is a WebSocket server speaking the DevTools
//! messages the node sends, in flatten mode. It keeps page clocks that
//! advance by exactly what each grant says (plus what a case asks for), and
//! plays a small app on its first page: timed steps — ask for the port,
//! open it, write, pull the cable — sent through the node's binding as the
//! shim would, and the node's answers read from each slice's evaluate. It
//! can also stand for a dedicated worker that owns the port's stream,
//! acknowledging each delivery after a delay of host time, for a second page
//! that shares the first's clock, and for a hidden page that holds back what
//! it says until it next hears from the node, as Chrome's do. It runs none
//! of the shim's JavaScript: `real_chrome.rs` holds the shim to Chrome.
//!
//! Stepped (`TESTING.md` rule 9): the case's thread is the clock's actor.
//! What is asserted is exact where virtual time owns it — the budgets
//! granted, the instants bytes enter the line, the page's clock against the
//! board's — and host time is never asserted; the wall-time bounds are
//! sized for a hang.

mod common;

use std::collections::{BTreeMap, HashMap};
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

/// The first page's clock (its document's `performance.now()`) when the
/// stand-in starts it, in milliseconds.
const ORIGIN_MS: f64 = 1_000.0;

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
    /// `port.open()` at a rate with these options besides.
    OpenWith(u32, Value),
    /// A write on the open port.
    Write(Vec<u8>),
    /// `__embsim.link(op)`.
    Link(&'static str),
    /// Any other request the shim makes of the port: its op and fields.
    Req(&'static str, Value),
}

/// What befalls the first page at one of its grants (counting from 1).
#[derive(Debug, Clone)]
enum At {
    /// Its clock runs this many ms past the budget.
    Overrun(f64),
    /// Its target closes: Chrome answers nothing more and says it detached.
    Detach,
    /// Its renderer crashes.
    Crash,
    /// A dialog of a kind with a message opens: the budget expires, and the
    /// page's main thread answers nothing more.
    Dialog(&'static str, &'static str),
    /// A navigation into a new renderer: a new document, whose clock reads
    /// this many ms when the grant ends.
    NewDocument(f64),
    /// A second page is made with a URL, which ran before it was held.
    LatePage,
}

/// What a case asks of the stand-in.
#[derive(Debug, Default)]
struct Script {
    acts: Vec<(f64, Act)>,
    at: Vec<(usize, At)>,
    /// From this grant on, budgets never expire.
    stuck_from: Option<usize>,
    /// An open fetch holds every `pauseIfNetworkFetchesPending` budget.
    /// A later `pause` budget expires, and the fetch is then done.
    hold_fetches: bool,
    /// A dedicated worker owns the port's stream (it was transferred) and
    /// acknowledges each delivery after this delay of host time (`None`: it
    /// never does).
    worker: Option<Option<Duration>>,
    /// Added to each reading of the first page's clock, in turn: the fuzz
    /// Chrome puts below a clock's resolution.
    fuzz: Vec<f64>,
    /// Whether the first page is cross-origin isolated (its clock read to
    /// 5 µs) as it says.
    isolated: bool,
    /// A second page, at boot, whose main thread is the first's: one clock.
    shared: bool,
    /// The first page is hidden, a background tab.
    hidden: bool,
    /// The first page's document ran its scripts before the shim.
    late: bool,
    /// A navigation the node asks for fails with this.
    navigate_error: Option<&'static str>,
}

impl Script {
    fn at(&self, grant: usize) -> Option<At> {
        self.at
            .iter()
            .find(|(n, _)| *n == grant)
            .map(|(_, at)| at.clone())
    }
}

/// What the stand-in saw.
#[derive(Debug, Default)]
struct Seen {
    connected: bool,
    /// The first page's grants.
    grants: Vec<f64>,
    /// Every grant: the page it went to and its budget.
    grants_to: Vec<(String, f64)>,
    slices: u64,
    /// The first page's clock at each slice, as the stand-in answered it.
    clocks: Vec<f64>,
    /// Bytes handed to the page while its port was open.
    rx: Vec<u8>,
    /// Every answer to a request, in order, with the step it answered.
    replies: Vec<(String, Value)>,
    /// The board's instant each answer was handed over at, in order.
    replied_at: Vec<u64>,
    /// The device's generation and whether it was plugged, at each change.
    states: Vec<(u64, bool)>,
    opened: bool,
    lost: u64,
    deliveries: u64,
    acks: u64,
    /// Messages the node sent while a delivery waited for its ack.
    early: u64,
    /// Pokes: evaluates of no effect.
    pokes: u64,
    /// Virtual-time policies the node sent, in order.
    policies: Vec<String>,
    /// The navigation the node asked for.
    navigated: Option<String>,
    /// Messages for a page after it closed.
    after_close: u64,
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
                let mut fake = FakeBrowser {
                    ws,
                    script,
                    seen,
                    clocks: Vec::new(),
                    tabs: BTreeMap::new(),
                    fuzz_at: 0,
                    acts_done: 0,
                    gen: None,
                    open: false,
                    plugged: None,
                    next_id: 1,
                    pending: HashMap::new(),
                    worker_reads: 0,
                    fetch_released: false,
                };
                fake.serve(&stop);
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

/// The first page's session, the one the app plays in.
const PAGE: &str = "P1";
/// A second page's session.
const SECOND: &str = "P2";
/// The page the node makes for its `url` in a browser it attached to.
const MADE: &str = "P3";
const WORKER: &str = "W1";
const DOC: &str = "fake-doc";
const ORIGIN: &str = "http://fake.test";
const HREF: &str = "http://fake.test/app.html";

/// One page of the stand-in browser.
#[derive(Debug)]
struct Tab {
    target: String,
    doc: String,
    href: String,
    /// Its clock: an index into the browser's clocks.
    clock: usize,
    hidden: bool,
    late: bool,
    /// What a hidden page holds back until it next hears from the node.
    held: Vec<Value>,
    /// A dialog is open: its main thread answers no evaluate.
    blocked: bool,
    grants: usize,
}

struct FakeBrowser {
    ws: WebSocket<TcpStream>,
    script: Arc<Mutex<Script>>,
    seen: Arc<Mutex<Seen>>,
    /// The page clocks, ms: one per renderer main thread.
    clocks: Vec<f64>,
    tabs: BTreeMap<String, Tab>,
    fuzz_at: usize,
    acts_done: usize,
    gen: Option<u64>,
    open: bool,
    plugged: Option<(u64, bool)>,
    next_id: u64,
    pending: HashMap<u64, Act>,
    worker_reads: u64,
    /// A `pause` budget has ended the held fetch.
    fetch_released: bool,
}

impl FakeBrowser {
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

    fn send_now(&mut self, value: Value) {
        let _ = self.ws.send(Message::text(value.to_string()));
    }

    /// Say `value` on `session`'s behalf: held back while its page is
    /// hidden.
    fn say(&mut self, session: Option<&str>, value: Value) {
        if let Some(tab) = session.and_then(|s| self.tabs.get_mut(s)) {
            if tab.hidden {
                tab.held.push(value);
                return;
            }
        }
        self.send_now(value);
    }

    fn reply(&mut self, m: &Value, result: Value) {
        let mut answer = json!({ "id": m["id"], "result": result });
        let session = m
            .get("sessionId")
            .and_then(Value::as_str)
            .map(str::to_string);
        if let Some(session) = &session {
            answer["sessionId"] = json!(session);
        }
        self.say(session.as_deref(), answer);
    }

    fn event(&mut self, session: Option<&str>, method: &str, params: Value) {
        let mut event = json!({ "method": method, "params": params });
        if let Some(session) = session {
            event["sessionId"] = json!(session);
        }
        self.say(session, event);
    }

    /// Something a page's shim says through the node's binding.
    fn binding(&mut self, session: &str, payload: Value) {
        self.event(
            Some(session),
            "Runtime.bindingCalled",
            json!({ "name": "__embsimTx", "payload": payload.to_string(), "executionContextId": 1 }),
        );
    }

    /// A page attached, waiting for the node.
    fn attach(&mut self, session: &str, tab: Tab, under: Option<&str>) {
        let info = json!({ "targetId": tab.target, "type": "page", "url": "about:blank" });
        self.tabs.insert(session.to_string(), tab);
        self.event(
            under,
            "Target.attachedToTarget",
            json!({ "sessionId": session, "targetInfo": info, "waitingForDebugger": true }),
        );
    }

    fn tab(&self, target: &str, doc: &str, href: &str, clock: usize) -> Tab {
        Tab {
            target: target.to_string(),
            doc: doc.to_string(),
            href: href.to_string(),
            clock,
            hidden: false,
            late: false,
            held: Vec::new(),
            blocked: false,
            grants: 0,
        }
    }

    fn command(&mut self, m: &Value) {
        let method = m["method"].as_str().unwrap_or_default().to_string();
        let session = m["sessionId"].as_str().map(str::to_string);
        // A hidden page says what it held back once it hears from the node.
        if let Some(tab) = session.as_deref().and_then(|s| self.tabs.get_mut(s)) {
            let held = std::mem::take(&mut tab.held);
            for value in held {
                self.send_now(value);
            }
        } else if session.as_deref().is_some_and(|s| s != WORKER) {
            self.seen.lock().unwrap().after_close += 1;
            return;
        }
        match (session.as_deref(), method.as_str()) {
            (None, "Browser.getVersion") => self.reply(m, json!({ "product": "FakeChrome/1.0" })),
            // Chrome sends the attach events for the targets it already has
            // before it answers (measured, Chrome 153).
            (None, "Target.setAutoAttach") => {
                let (shared, hidden, late) = {
                    let script = self.script.lock().unwrap();
                    (script.shared, script.hidden, script.late)
                };
                self.clocks.push(ORIGIN_MS);
                let mut first = self.tab("T1", DOC, HREF, 0);
                first.hidden = hidden;
                first.late = late;
                self.attach(PAGE, first, None);
                if shared {
                    let second = self.tab("T2", "fake-doc-2", "http://fake.test/opened.html", 0);
                    self.attach(SECOND, second, None);
                }
                self.reply(m, json!({}));
            }
            (None, "Target.createTarget") => {
                self.clocks.push(ORIGIN_MS);
                let clock = self.clocks.len() - 1;
                let made = self.tab("T3", "fake-doc-3", "about:blank", clock);
                self.reply(m, json!({ "targetId": "T3" }));
                self.attach(MADE, made, None);
            }
            (Some(page), "Target.setAutoAttach") if page == PAGE => {
                if self.script.lock().unwrap().worker.is_some() {
                    self.event(
                        Some(PAGE),
                        "Target.attachedToTarget",
                        json!({
                            "sessionId": WORKER,
                            "targetInfo": { "targetId": "TW", "type": "worker", "url": "w.js" },
                            "waitingForDebugger": true,
                        }),
                    );
                }
                self.reply(m, json!({}));
            }
            (Some(WORKER), "Runtime.evaluate") => {
                self.reply(m, json!({ "result": { "type": "number", "value": 1 } }))
            }
            (Some(WORKER), _) => self.reply(m, json!({})),
            (Some(page), "Runtime.runIfWaitingForDebugger") => {
                self.reply(m, json!({}));
                let tab = &self.tabs[page];
                let hello = json!({
                    "k": "hello", "doc": tab.doc, "origin": ORIGIN, "href": tab.href,
                    "late": tab.late,
                });
                let page = page.to_string();
                self.binding(&page, hello);
            }
            (Some(page), "Emulation.setVirtualTimePolicy") => {
                let page = page.to_string();
                match m["params"]["budget"].as_f64() {
                    Some(budget) => self.grant(&page, m, budget),
                    None => {
                        let base = self.base(&page);
                        self.reply(m, json!({ "virtualTimeTicksBase": base }));
                    }
                }
            }
            (Some(page), "Runtime.evaluate") => {
                let page = page.to_string();
                if self.tabs[&page].blocked {
                    return;
                }
                if m["params"]["expression"] == "0" {
                    self.seen.lock().unwrap().pokes += 1;
                    self.reply(m, json!({ "result": { "type": "number", "value": 0 } }));
                } else {
                    self.slice(&page, m);
                }
            }
            (Some(_), "Page.navigate") => {
                let url = m["params"]["url"].as_str().unwrap_or_default().to_string();
                self.seen.lock().unwrap().navigated = Some(url);
                let error = self.script.lock().unwrap().navigate_error;
                match error {
                    Some(text) => self.reply(m, json!({ "frameId": "F", "errorText": text })),
                    None => self.reply(m, json!({ "frameId": "F", "loaderId": "L" })),
                }
            }
            _ => self.reply(m, json!({})),
        }
    }

    /// A page's `virtualTimeTicksBase`: one per clock.
    fn base(&self, page: &str) -> f64 {
        5_000.0 + self.tabs[page].clock as f64
    }

    fn grant(&mut self, page: &str, m: &Value, budget: f64) {
        let policy = m["params"]["policy"].as_str().unwrap_or("").to_string();
        self.seen.lock().unwrap().policies.push(policy.clone());
        if policy == "pause" {
            self.fetch_released = true;
        }
        let n = {
            let tab = self.tabs.get_mut(page).expect("a page");
            tab.grants += 1;
            tab.grants
        };
        {
            let mut seen = self.seen.lock().unwrap();
            seen.grants_to.push((page.to_string(), budget));
            if page == PAGE {
                seen.grants.push(budget);
            }
        }
        let (stuck, at) = {
            let script = self.script.lock().unwrap();
            let first = page == PAGE;
            let held = script.hold_fetches && !self.fetch_released && policy != "pause";
            (
                first && (held || script.stuck_from.is_some_and(|from| n >= from)),
                if first { script.at(n) } else { None },
            )
        };
        match at {
            Some(At::Detach) => {
                let tab = self.tabs.remove(page).expect("a page");
                self.event(
                    None,
                    "Target.detachedFromTarget",
                    json!({ "sessionId": page, "targetId": tab.target }),
                );
                return;
            }
            Some(At::Crash) => {
                self.event(Some(page), "Inspector.targetCrashed", json!({}));
                return;
            }
            _ => {}
        }
        let base = self.base(page);
        self.reply(m, json!({ "virtualTimeTicksBase": base }));
        if stuck {
            return;
        }
        let clock = self.tabs[page].clock;
        let extra = match at {
            Some(At::Overrun(ms)) => ms,
            _ => 0.0,
        };
        let to = self.clocks[clock] + budget + extra;
        // The app's steps whose time has come, as the page lives the grant.
        if page == PAGE {
            loop {
                let next = {
                    let script = self.script.lock().unwrap();
                    script.acts.get(self.acts_done).cloned()
                };
                let Some((when, act)) = next else { break };
                if ORIGIN_MS + when > to {
                    break;
                }
                self.acts_done += 1;
                self.act(act);
            }
        }
        self.clocks[clock] = to;
        match at {
            Some(At::NewDocument(reads)) => {
                // A new renderer: a clock of its own, and a new document.
                self.clocks.push(reads);
                let tab = self.tabs.get_mut(page).expect("a page");
                tab.clock = self.clocks.len() - 1;
                tab.doc = "fake-doc-next".to_string();
                let hello = json!({
                    "k": "hello", "doc": "fake-doc-next", "origin": ORIGIN, "href": HREF,
                    "late": false,
                });
                self.binding(page, hello);
            }
            Some(At::Dialog(kind, message)) => {
                self.event(
                    Some(page),
                    "Page.javascriptDialogOpening",
                    json!({ "url": HREF, "message": message, "type": kind, "hasBrowserHandler": false }),
                );
                self.tabs.get_mut(page).expect("a page").blocked = true;
            }
            Some(At::LatePage) => {
                self.clocks.push(ORIGIN_MS);
                let clock = self.clocks.len() - 1;
                let mut late = self.tab("T2", "fake-doc-2", "http://fake.test/made.html", clock);
                late.late = true;
                self.attach(SECOND, late, None);
            }
            _ => {}
        }
        self.event(Some(page), "Emulation.virtualTimeBudgetExpired", json!({}));
    }

    fn request(&mut self, act: Act, op: &str, extra: Value) {
        let id = self.next_id;
        self.next_id += 1;
        self.pending.insert(id, act);
        let mut payload = json!({ "k": "req", "doc": DOC, "id": id, "op": op, "origin": ORIGIN });
        for (key, value) in extra.as_object().unwrap() {
            payload[key] = value.clone();
        }
        self.binding(PAGE, payload);
    }

    fn act(&mut self, act: Act) {
        match act.clone() {
            Act::GetPorts => self.request(act, "getPorts", json!({})),
            Act::Open(baud) => {
                let gen = self.gen.unwrap_or(u64::MAX);
                self.request(act, "open", open_options(gen, baud));
            }
            Act::OpenGen(gen, baud) => self.request(act, "open", open_options(gen, baud)),
            Act::OpenWith(baud, extra) => {
                let gen = self.gen.unwrap_or(u64::MAX);
                let mut options = open_options(gen, baud);
                for (key, value) in extra.as_object().unwrap() {
                    options[key] = value.clone();
                }
                self.request(act, "open", options);
            }
            Act::Write(bytes) => {
                self.binding(PAGE, json!({ "k": "tx", "doc": DOC, "b": b64(&bytes) }));
            }
            Act::Link(op) => self.binding(PAGE, json!({ "k": "link", "doc": DOC, "op": op })),
            Act::Req(op, extra) => self.request(act, op, extra),
        }
    }

    fn slice(&mut self, page: &str, m: &Value) {
        let expression = m["params"]["expression"].as_str().unwrap_or_default();
        let start =
            expression.find("const a = ").expect("the slice's argument") + "const a = ".len();
        let end = expression
            .find("; return")
            .expect("the slice's argument's end");
        let arg: Value =
            serde_json::from_str(&expression[start..end]).expect("the argument is JSON");
        let (clock, doc, hidden) = {
            let tab = &self.tabs[page];
            (self.clocks[tab.clock], tab.doc.clone(), tab.hidden)
        };
        let (isolated, worker) = {
            let script = self.script.lock().unwrap();
            (script.isolated, script.worker)
        };
        if page != PAGE {
            let answer = json!({ "t": clock, "doc": doc, "iso": isolated, "hidden": hidden });
            self.reply(
                m,
                json!({ "result": { "type": "object", "value": answer } }),
            );
            return;
        }
        let state = (
            arg["gen"].as_u64().unwrap(),
            arg["plugged"].as_bool().unwrap(),
        );
        {
            let mut seen = self.seen.lock().unwrap();
            seen.slices += 1;
            seen.clocks.push(clock);
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
                Some(Act::Open(_)) | Some(Act::OpenGen(..)) | Some(Act::OpenWith(..)) => "open",
                Some(Act::Req(op, _)) => op,
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
                    Some(Act::Req("requestPort", _)) => self.gen = reply["v"].as_u64(),
                    Some(Act::Open(_)) | Some(Act::OpenGen(..)) | Some(Act::OpenWith(..)) => {
                        self.open = true;
                        opened = true;
                    }
                    Some(Act::Req("forget", _)) if reply["v"]["closed"] == true => {
                        self.open = false;
                    }
                    _ => {}
                }
            }
            let mut seen = self.seen.lock().unwrap();
            seen.replies.push((name.to_string(), reply));
            seen.replied_at.push(virtual_clock::virtual_ns());
        }
        let mut reads = 0;
        if let (Some(rx), true) = (arg["rx"].as_str(), self.open) {
            let bytes = unb64(rx);
            reads = bytes.len().div_ceil(255) as u64;
            let mut seen = self.seen.lock().unwrap();
            seen.rx.extend(bytes);
            seen.deliveries += 1;
        }
        let fuzz = {
            let script = self.script.lock().unwrap();
            match script.fuzz.as_slice() {
                [] => 0.0,
                fuzz => fuzz[self.fuzz_at % fuzz.len()],
            }
        };
        self.fuzz_at += 1;
        let answer = json!({
            "t": clock + fuzz,
            "doc": doc,
            "iso": isolated,
            "hidden": hidden,
            "reading": self.open,
            "transferred": self.open && worker.is_some(),
            "reads": reads,
        });
        self.reply(
            m,
            json!({ "result": { "type": "object", "value": answer } }),
        );
        if opened {
            self.seen.lock().unwrap().opened = true;
            self.binding(PAGE, json!({ "k": "reading", "doc": DOC }));
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
        let until = Instant::now() + delay.unwrap_or(Duration::from_millis(20));
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
    node_with(fake, baud, |settings| {
        settings.usb = UsbIds {
            vendor: Some(0x0403),
            product: Some(0x6001),
        };
    })
}

fn node_with(fake: &Fake, baud: u32, set: impl FnOnce(&mut Settings)) -> CdpNode {
    let mut settings = Settings::new(Browse::Attach(fake.url.clone()));
    settings.granted = true;
    settings.drain_bound = HANG;
    set(&mut settings);
    CdpNode::new(settings, baud).expect("the settings run")
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

/// The fake and a node attached to it on the bench, run until `done`.
fn run_until(
    script: Script,
    set: impl FnOnce(&mut Settings),
    what: &str,
    done: impl Fn(&Fake, &NodeStats) -> bool,
) -> (Fake, Arc<NodeStats>) {
    let fake = Fake::start(script);
    let node = node_with(&fake, 115_200, set);
    let stats = node.stats();
    let (peer, _) = peer(115_200);
    let (system, actor) = bench(node, peer);
    until(&stats, what, || done(&fake, &stats));
    finish(system, actor);
    (fake, stats)
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

#[rstest]
#[case::isolated(true, vec![0.0, 0.004_883, -0.005_127, 0.0, -0.004_883, 0.005_127])]
#[case::not_isolated(false, vec![0.0, 0.098, -0.099, 0.05, -0.1, 0.1])]
fn a_clock_read_to_its_resolution_is_booked_as_its_grants(
    #[case] isolated: bool,
    #[case] fuzz: Vec<f64>,
) {
    behaviour!(Test {
        id: "chrome-cdp.booked-from-grants",
        covers: Some("cdp/src/node.rs#Meter::book"),
        given: "a page metered every millisecond whose clock reads with a fuzz below its \
                resolution, as Chrome clamps one (5 µs when cross-origin isolated, 100 µs \
                otherwise)",
    });
    expect!(
        "whole-quanta",
        "every grant is exactly one quantum: the books take what the page was granted"
    );
    expect!(
        "no-lead",
        "the node reports no lead and no clock past its budget"
    );

    let _suite = suite_lock();
    let (fake, stats) = run_until(
        Script {
            fuzz,
            isolated,
            ..Script::default()
        },
        |_| {},
        "40 slices",
        |_, stats| stats.slices() >= 40,
    );
    let grants = fake.seen().grants.clone();
    assert!(
        grants.iter().all(|&budget| budget == 1.0),
        "whole-quanta: {grants:?}"
    );
    assert_eq!(stats.peak_lead_ns(), 0, "no-lead: {stats:?}");
    assert_eq!(stats.peak_overrun_ns(), 0, "no-lead: {stats:?}");
    assert_eq!(stats.failure(), None);
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
        at: vec![(3, At::Overrun(5.0))],
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
        at: vec![(3, At::Overrun(5.0))],
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
fn a_new_document_starts_level_with_the_board() {
    behaviour!(Test {
        id: "chrome-cdp.new-document",
        covers: Some("cdp/src/node.rs#Meter::book"),
        given: "a page allowed to lead by 2 ms that navigates at its third grant into a new \
                renderer, whose document's clock first reads 150 ms",
    });
    expect!(
        "no-lead",
        "the new document is booked level with the board: no lead, no clock past its \
         budget, and the run goes on"
    );
    expect!(
        "quanta-after",
        "the page is granted one quantum a slice before and after the navigation"
    );
    expect!(
        "counted",
        "the node counts the books set level with the board once"
    );

    let _suite = suite_lock();
    let (fake, stats) = run_until(
        Script {
            at: vec![(3, At::NewDocument(150.0))],
            ..Script::default()
        },
        |s| s.max_lead = Some(Duration::from_millis(2)),
        "20 slices",
        |_, stats| stats.slices() >= 20 || stats.failure().is_some(),
    );
    assert_eq!(stats.failure(), None, "no-lead");
    assert_eq!(stats.peak_lead_ns(), 0, "no-lead: {stats:?}");
    assert_eq!(stats.peak_overrun_ns(), 0, "no-lead: {stats:?}");
    let grants = fake.seen().grants.clone();
    assert!(
        grants.iter().all(|&budget| budget == 1.0),
        "quanta-after: {grants:?}"
    );
    assert_eq!(stats.reanchored(), 1, "counted: {stats:?}");
}

#[test]
fn a_grant_that_never_expires_stops_the_run_saying_why() {
    behaviour!(Test {
        id: "chrome-cdp.stuck-grant",
        covers: Some("cdp/src/browser.rs#Browser::grant"),
        given: "a page whose budgets stop expiring from its third grant, the node allowed to \
                wait 2 s of host time for one",
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
    let node = node_with(&fake, 115_200, |s| s.stuck_after = Duration::from_secs(2));
    let stats = node.stats();
    let (peer, _) = peer(115_200);
    let (system, actor) = bench(node, peer);
    until(&stats, "the failure", || stats.failure().is_some());
    let failure = stats.failure().expect("failure-says-why: the node failed");
    assert!(
        failure.starts_with(&format!(
            "a grant stuck: the page at {HREF} was granted 1.000 ms"
        )) && failure.contains("a fetch that never completes, or a task that never ends"),
        "failure-says-why: {failure}"
    );
    assert_eq!(stats.stuck(), 1, "failure-says-why");
    assert_eq!(fake.seen().grants.len(), 3, "no-more-grants");
    virtual_clock::wait_virtual_ns(10 * MS);
    assert_eq!(fake.seen().grants.len(), 3, "no-more-grants");
    finish(system, actor);
}

#[test]
fn a_fetch_that_holds_a_budget_lets_that_budget_end() {
    behaviour!(Test {
        id: "chrome-cdp.fetch-hold",
        covers: Some("cdp/src/browser.rs#Browser::grant"),
        given: "a page whose open network fetch holds every virtual-time budget until the \
                budget is allowed to end",
    });
    expect!(
        "budget-ends",
        "the held budget ends and the page is granted again"
    );
    expect!(
        "pause-policy",
        "the held budget is granted again under the policy that lets it end while the fetch \
         is still open"
    );

    let _suite = suite_lock();
    let (fake, stats) = run_until(
        Script {
            hold_fetches: true,
            ..Script::default()
        },
        |s| s.stuck_after = Duration::from_secs(5),
        "8 slices",
        |_, stats| stats.slices() >= 8 || stats.failure().is_some(),
    );
    assert_eq!(stats.failure(), None, "budget-ends: {stats:?}");
    assert!(stats.slices() >= 8, "budget-ends: {stats:?}");
    let policies = fake.seen().policies.clone();
    assert!(
        policies.iter().any(|policy| policy == "pause"),
        "pause-policy: {policies:?}"
    );
}

#[test]
fn a_page_that_closes_during_its_grant_is_let_go_at_once() {
    behaviour!(Test {
        id: "chrome-cdp.closed-mid-grant",
        covers: Some("cdp/src/devtools.rs#DevTools::session_gone"),
        given: "a page that closes during its third grant, so its budget is never answered \
                and Chrome says it detached, as a harness closing a scenario's browser \
                context does",
    });
    expect!(
        "run-goes-on",
        "the node carries on with no failure, the board's clock running on"
    );
    expect!(
        "nothing-sent-after",
        "nothing more is sent for the closed page"
    );

    let _suite = suite_lock();
    let fake = Fake::start(Script {
        at: vec![(3, At::Detach)],
        ..Script::default()
    });
    let node = node_with(&fake, 115_200, |s| s.stuck_after = HANG);
    let stats = node.stats();
    let (peer, _) = peer(115_200);
    let (system, actor) = bench(node, peer);
    until(&stats, "the third grant", || fake.seen().grants.len() >= 3);
    let at = stats.slices();
    until(&stats, "20 slices past the close", || {
        stats.slices() >= at + 20 || stats.failure().is_some()
    });
    assert_eq!(stats.failure(), None, "run-goes-on");
    assert_eq!(fake.seen().grants.len(), 3, "nothing-sent-after");
    assert_eq!(fake.seen().after_close, 0, "nothing-sent-after");
    finish(system, actor);
}

#[test]
fn a_page_that_crashes_stops_the_run_at_once() {
    behaviour!(Test {
        id: "chrome-cdp.crash",
        covers: Some("cdp/src/node.rs#Meter::run_slice"),
        given: "a page whose renderer crashes during its third grant, the node allowed to \
                wait two minutes of host time for a grant",
    });
    expect!(
        "failure-names-the-crash",
        "the node stops at that slice, saying the page crashed and naming it"
    );
    expect!("no-more-grants", "no grant is sent after the crash");

    let _suite = suite_lock();
    let fake = Fake::start(Script {
        at: vec![(3, At::Crash)],
        ..Script::default()
    });
    let node = node_with(&fake, 115_200, |s| s.stuck_after = HANG);
    let stats = node.stats();
    let (peer, _) = peer(115_200);
    let (system, actor) = bench(node, peer);
    until(&stats, "the failure", || stats.failure().is_some());
    let failure = stats.failure().unwrap();
    assert_eq!(
        failure,
        format!("a page crashed at {HREF}: its renderer process went away mid-run"),
        "failure-names-the-crash"
    );
    assert_eq!(fake.seen().grants.len(), 3, "no-more-grants");
    finish(system, actor);
}

#[test]
fn a_dialog_nobody_answers_stops_the_run_naming_it() {
    behaviour!(Test {
        id: "chrome-cdp.dialog",
        covers: Some("cdp/src/node.rs#Meter::page_slice"),
        given: "a page that opens a confirm dialog asking \"sure?\" during its third grant, \
                which nothing answers, the node allowed to wait 2 s of host time for a page",
    });
    expect!(
        "failure-names-the-dialog",
        "the node stops, saying the page's main thread did not return, and naming the \
         dialog's kind and message"
    );

    let _suite = suite_lock();
    let (_fake, stats) = run_until(
        Script {
            at: vec![(3, At::Dialog("confirm", "sure?"))],
            ..Script::default()
        },
        |s| s.stuck_after = Duration::from_secs(2),
        "the failure",
        |_, stats| stats.failure().is_some(),
    );
    let failure = stats.failure().unwrap();
    assert!(
        failure.starts_with(&format!(
            "the main thread of the page at {HREF} did not return"
        )) && failure.contains("it shows a confirm dialog (\"sure?\") that nobody answered"),
        "failure-names-the-dialog: {failure}"
    );
}

#[test]
fn pages_that_share_a_clock_are_granted_once_between_them() {
    behaviour!(Test {
        id: "chrome-cdp.shared-clock",
        covers: Some("cdp/src/node.rs#Meter::budgets"),
        given: "two pages whose main thread is one renderer's, so a grant to either advances \
                both, as a page and a window it opened are",
    });
    expect!(
        "one-grant-a-slice",
        "each slice grants one quantum between the two pages, to each in turn"
    );
    expect!(
        "level",
        "both pages stay level with the board: no lead, no grant skipped"
    );

    let _suite = suite_lock();
    let (fake, stats) = run_until(
        Script {
            shared: true,
            ..Script::default()
        },
        |_| {},
        "20 slices",
        |_, stats| stats.slices() >= 20 || stats.failure().is_some(),
    );
    let grants = fake.seen().grants_to.clone();
    assert!(grants.len() >= 15, "one-grant-a-slice: {grants:?}");
    assert!(
        grants.iter().all(|(_, budget)| *budget == 1.0),
        "one-grant-a-slice: {grants:?}"
    );
    assert!(
        grants.windows(2).all(|pair| pair[0].0 != pair[1].0),
        "one-grant-a-slice: {grants:?}"
    );
    assert_eq!(
        grants.len() as u64,
        stats.slices() - 1,
        "one-grant-a-slice: {stats:?}"
    );
    assert_eq!(stats.peak_lead_ns(), 0, "level: {stats:?}");
    assert_eq!(stats.skipped(), 0, "level: {stats:?}");
    assert_eq!(stats.failure(), None);
}

#[test]
fn a_hidden_page_says_what_it_held_back_once_poked() {
    behaviour!(Test {
        id: "chrome-cdp.hidden-page",
        covers: Some("cdp/src/browser.rs#Browser::await_answer"),
        given: "a hidden page (a background tab) that holds back its budget's expiry and its \
                answers until it next hears from the node, as Chrome's do",
    });
    expect!(
        "runs-on",
        "the page is granted a quantum at each slice and the run goes on with no failure"
    );
    expect!(
        "poked",
        "the node pokes the page with commands of no effect"
    );

    let _suite = suite_lock();
    let (fake, stats) = run_until(
        Script {
            hidden: true,
            ..Script::default()
        },
        |s| s.stuck_after = HANG,
        "20 slices",
        |_, stats| stats.slices() >= 20 || stats.failure().is_some(),
    );
    assert_eq!(stats.failure(), None, "runs-on");
    let grants = fake.seen().grants.clone();
    assert!(
        grants.len() >= 15 && grants.iter().all(|&b| b == 1.0),
        "runs-on: {grants:?}"
    );
    assert!(fake.seen().pokes > 0, "poked");
}

#[test]
fn a_page_born_with_a_url_after_the_node_reached_chrome_stops_the_run() {
    behaviour!(Test {
        id: "chrome-cdp.page-born-late",
        covers: Some("cdp/src/node.rs#Meter::take"),
        given: "a second page made during the run whose document ran its scripts before the \
                shim was installed in it, as a page Chrome makes with a URL does",
    });
    expect!(
        "failure-says-how-to-make-one",
        "the node stops, naming the page and saying to make it at about:blank and navigate \
         it"
    );

    let _suite = suite_lock();
    let (_fake, stats) = run_until(
        Script {
            at: vec![(3, At::LatePage)],
            ..Script::default()
        },
        |_| {},
        "the failure",
        |_, stats| stats.failure().is_some(),
    );
    let failure = stats.failure().unwrap();
    assert!(
        failure.starts_with(
            "a page at http://fake.test/made.html ran its own scripts before the node held it"
        ) && failure.contains("make the page at about:blank and navigate it"),
        "failure-says-how-to-make-one: {failure}"
    );
}

#[test]
fn a_page_open_before_the_node_attached_is_reported_and_metered() {
    behaviour!(Test {
        id: "chrome-cdp.page-open-at-attach",
        covers: Some("cdp/src/catalog.rs#CdpReport"),
        given: "a browser the node attaches to with a page already open, whose document ran \
                its scripts before the shim was installed in it",
    });
    expect!(
        "metered",
        "the page is granted a quantum at each slice and the run goes on with no failure"
    );
    expect!(
        "counted",
        "the node counts one document that ran before the shim"
    );

    let _suite = suite_lock();
    let (fake, stats) = run_until(
        Script {
            late: true,
            ..Script::default()
        },
        |_| {},
        "10 slices",
        |_, stats| stats.slices() >= 10 || stats.failure().is_some(),
    );
    assert_eq!(stats.failure(), None, "metered");
    assert!(fake.seen().grants.len() >= 8, "metered");
    assert_eq!(stats.late_at_attach(), 1, "counted: {stats:?}");
}

#[test]
fn a_url_that_does_not_open_stops_the_run_saying_why() {
    behaviour!(Test {
        id: "chrome-cdp.url-fails",
        covers: Some("cdp/src/browser.rs#Browser::navigation_failure"),
        given: "a node attached to a browser, set to open a page whose navigation Chrome \
                refuses because nothing listens at its address",
    });
    expect!(
        "new-window",
        "the node opens the page in a page of its own, made at about:blank"
    );
    expect!(
        "failure-says-why",
        "the node stops, naming the URL and Chrome's reason"
    );

    let _suite = suite_lock();
    let url = "http://127.0.0.1:9/app.html";
    let (fake, stats) = run_until(
        Script {
            navigate_error: Some("net::ERR_CONNECTION_REFUSED"),
            ..Script::default()
        },
        |s| s.url = Some(url.to_string()),
        "the failure",
        |_, stats| stats.failure().is_some(),
    );
    assert_eq!(fake.seen().navigated.as_deref(), Some(url), "new-window");
    assert_eq!(
        stats.failure().unwrap(),
        format!("the page at {url} did not open: net::ERR_CONNECTION_REFUSED"),
        "failure-says-why"
    );
}

#[test]
fn a_devtools_endpoint_named_by_localhost_is_reached() {
    behaviour!(Test {
        id: "chrome-cdp.localhost",
        covers: Some("cdp/src/devtools.rs#DevTools::connect"),
        given: "a node attached to ws://localhost:PORT/… where the browser listens on \
                127.0.0.1 only, as Chrome's DevTools does, while localhost may resolve to ::1 \
                first",
    });
    expect!(
        "reached",
        "the node reaches the browser and meters its page"
    );

    let _suite = suite_lock();
    let fake = Fake::start(Script::default());
    let by_name = fake.url.replace("127.0.0.1", "localhost");
    let mut settings = Settings::new(Browse::Attach(by_name));
    settings.granted = true;
    let node = CdpNode::new(settings, 115_200).expect("the settings run");
    let stats = node.stats();
    let (peer, _) = peer(115_200);
    let (system, actor) = bench(node, peer);
    until(&stats, "five grants", || {
        fake.seen().grants.len() >= 5 || stats.failure().is_some()
    });
    assert_eq!(stats.failure(), None, "reached");
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
    expect!(
        "no-barrier",
        "the page reads its port on its own main thread, so no slice waits for a consumer"
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
    assert_eq!(stats.drain_waits(), 0, "no-barrier: {stats:?}");
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
fn a_drain_is_answered_once_the_last_byte_has_left_the_line() {
    behaviour!(Test {
        id: "chrome-cdp.drain",
        covers: Some("cdp/src/node.rs#Meter::request"),
        given: "a page that writes 100 bytes at 115200 baud and at once closes its writer, \
                which waits until they are sent",
    });
    expect!(
        "after-the-last-byte",
        "the close completes at the first slice after the peer heard the last of the 100 bytes"
    );

    let _suite = suite_lock();
    let mut acts = opening(115_200);
    acts.push((10.0, Act::Write(vec![0x5a; 100])));
    acts.push((10.0, Act::Req("drain", json!({}))));
    let fake = Fake::start(Script {
        acts,
        ..Script::default()
    });
    let node = node(&fake, 115_200);
    let stats = node.stats();
    let (peer, heard) = peer(115_200);
    let (system, actor) = bench(node, peer);
    until(&stats, "the drain's answer", || {
        fake.seen().replies.iter().any(|(name, _)| name == "drain")
    });
    let answered_at = {
        let seen = fake.seen();
        let at = seen
            .replies
            .iter()
            .position(|(name, _)| name == "drain")
            .unwrap();
        seen.replied_at[at]
    };
    assert_eq!(heard.heard().len(), 100, "after-the-last-byte");
    let (last, _) = *heard.heard_at().last().unwrap();
    let frame_ns = 10 * 1_000_000_000 / 115_200;
    assert!(
        last <= answered_at + frame_ns && answered_at <= last + MS,
        "after-the-last-byte: the last byte heard at {last} ns, the drain answered at \
         {answered_at} ns"
    );
    assert_eq!(stats.failure(), None);
    finish(system, actor);
}

#[test]
fn an_aborted_write_discards_what_has_not_left() {
    behaviour!(Test {
        id: "chrome-cdp.abort",
        covers: Some("cdp/src/node.rs#Meter::request"),
        given: "a page that writes 6000 bytes at 9600 baud in one write, more than the line's \
                own queue of 4096 holds, and abandons the write a millisecond later",
    });
    expect!(
        "discarded",
        "the bytes still waiting for the line's queue are thrown away and counted; the peer \
         hears only what the queue had taken"
    );

    let _suite = suite_lock();
    let mut acts = opening(9_600);
    acts.push((10.0, Act::Write(vec![0x33; 6000])));
    acts.push((11.0, Act::Req("flush", json!({ "dir": "tx" }))));
    let fake = Fake::start(Script {
        acts,
        ..Script::default()
    });
    let node = node(&fake, 9_600);
    let stats = node.stats();
    let (peer, heard) = peer(9_600);
    let (system, actor) = bench(node, peer);
    until(&stats, "the flush's answer", || {
        fake.seen().replies.iter().any(|(name, _)| name == "flush")
    });
    virtual_clock::wait_virtual_ns(200 * MS);
    let heard = heard.heard().len() as u64;
    assert!(
        heard <= stats.from_page(),
        "discarded: the peer heard {heard}"
    );
    assert_eq!(
        stats.from_page() + stats.flushed(),
        6000,
        "discarded: {stats:?}"
    );
    assert!(stats.flushed() > 0, "discarded: {stats:?}");
    assert_eq!(stats.failure(), None);
    finish(system, actor);
}

#[rstest]
#[case::policy(true, false)]
#[case::by_request(false, true)]
fn forget_gives_up_the_port_unless_the_policy_grants_it(
    #[case] policy: bool,
    #[case] closes: bool,
) {
    behaviour!(Test {
        id: "chrome-cdp.forget",
        covers: Some("cdp/src/node.rs#Meter::request"),
        given: "a page with its port open that gives the port up and then lists its ports, \
                the port granted by the scenario's policy or by the page's own request",
    });
    expect!(
        "by-request-closed",
        "a port the page asked for is closed, and the page's next list of ports is empty"
    );
    expect!(
        "policy-kept",
        "a port the policy grants stays open and listed: a policy grant cannot be given up"
    );

    let _suite = suite_lock();
    let acts = vec![
        (1.0, Act::Req("requestPort", json!({ "filters": [] }))),
        (3.0, Act::Open(115_200)),
        (10.0, Act::Req("forget", json!({ "gen": 0 }))),
        (12.0, Act::GetPorts),
    ];
    let (fake, stats) = run_until(
        Script {
            acts,
            ..Script::default()
        },
        |s| s.granted = policy,
        "the getPorts after",
        |fake, _| {
            fake.seen()
                .replies
                .iter()
                .any(|(name, _)| name == "getPorts")
        },
    );
    let seen = fake.seen();
    let forget = &seen
        .replies
        .iter()
        .find(|(name, _)| name == "forget")
        .expect("forget answered")
        .1;
    let listed = &seen
        .replies
        .iter()
        .find(|(name, _)| name == "getPorts")
        .expect("getPorts answered")
        .1["v"];
    let expectation = if closes {
        "by-request-closed"
    } else {
        "policy-kept"
    };
    assert_eq!(forget["v"]["closed"], closes, "{expectation}: {forget}");
    assert_eq!(
        listed.as_array().map(Vec::len),
        Some(if closes { 0 } else { 1 }),
        "{expectation}: {listed}"
    );
    assert_eq!(stats.failure(), None);
}

#[rstest]
#[case::no_filters(json!([]), true)]
#[case::the_vendor(json!([{ "usbVendorId": 0x0403 }]), true)]
#[case::vendor_and_product(json!([{ "usbVendorId": 0x0403, "usbProductId": 0x6001 }]), true)]
#[case::another_product(json!([{ "usbVendorId": 0x0403, "usbProductId": 0x6015 }]), false)]
#[case::another_vendor(json!([{ "usbVendorId": 0x10c4 }]), false)]
#[case::bluetooth(json!([{ "usbVendorId": null, "usbProductId": null, "bluetooth": true }]), false)]
#[case::one_of_two(json!([{ "usbVendorId": 0x10c4 }, { "usbVendorId": 0x0403 }]), true)]
fn request_port_offers_the_port_to_a_filter_that_matches_it(
    #[case] filters: Value,
    #[case] offered: bool,
) {
    behaviour!(Test {
        id: "chrome-cdp.request-port-filters",
        covers: Some("cdp/src/node.rs#Meter::request"),
        given: "a page that asks for a port with filters naming USB ids (Web Serial's port \
                chooser), the port an FTDI FT232R (vendor 0x0403, product 0x6001)",
    });
    expect!(
        "matched",
        "the port is offered when no filter is given or a filter names its vendor (and its \
         product, if it names one)"
    );
    expect!(
        "refused",
        "otherwise the request is refused with NotFoundError: no port was selected"
    );

    let _suite = suite_lock();
    let acts = vec![(1.0, Act::Req("requestPort", json!({ "filters": filters })))];
    let fake = Fake::start(Script {
        acts,
        ..Script::default()
    });
    let mut settings = Settings::new(Browse::Attach(fake.url.clone()));
    settings.usb = UsbIds {
        vendor: Some(0x0403),
        product: Some(0x6001),
    };
    let node = CdpNode::new(settings, 115_200).expect("the settings run");
    let stats = node.stats();
    let (peer, _) = peer(115_200);
    let (system, actor) = bench(node, peer);
    until(&stats, "the answer", || {
        fake.seen()
            .replies
            .iter()
            .any(|(name, _)| name == "requestPort")
    });
    let reply = fake
        .seen()
        .replies
        .iter()
        .find(|(name, _)| name == "requestPort")
        .unwrap()
        .1
        .clone();
    if offered {
        assert_eq!(reply["ok"], true, "matched: {reply}");
    } else {
        assert_eq!(reply["ok"], false, "refused: {reply}");
        assert_eq!(reply["name"], "NotFoundError", "refused: {reply}");
        assert_eq!(
            reply["message"], "No port selected by the user.",
            "refused: {reply}"
        );
    }
    finish(system, actor);
}

#[test]
fn modem_control_no_pin_carries_is_counted_and_reported() {
    behaviour!(Test {
        id: "chrome-cdp.modem-control",
        covers: Some("cdp/src/node.rs#NodeStats::signals"),
        given: "a page that opens its port with hardware flow control, asserts DTR and RTS, \
                then drops them, on a line with no modem-control pins",
    });
    expect!(
        "signals-counted",
        "the node counts one setSignals call that asserted a signal, and the call that only \
         dropped them is answered and not counted"
    );
    expect!(
        "flow-control-counted",
        "the node counts one open that asked for hardware flow control, and the port opens"
    );
    expect!(
        "reported",
        "the report says once that nothing on the board saw DTR and RTS, and that nothing \
         paces the bytes"
    );

    let _suite = suite_lock();
    let acts = vec![
        (1.0, Act::GetPorts),
        (
            3.0,
            Act::OpenWith(115_200, json!({ "flowControl": "hardware" })),
        ),
        (
            6.0,
            Act::Req(
                "setSignals",
                json!({ "signals": { "dataTerminalReady": true, "requestToSend": true } }),
            ),
        ),
        (
            8.0,
            Act::Req(
                "setSignals",
                json!({ "signals": { "dataTerminalReady": false, "requestToSend": false } }),
            ),
        ),
    ];
    let fake = Fake::start(Script {
        acts,
        ..Script::default()
    });
    let text = project_text(&fake.url, "");
    virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);
    let reports = Reports::new();
    let system = Project::parse(&text)
        .expect("the text is a project")
        .instantiate_with(&set(), &reports)
        .expect("the bench builds")
        .hold_time()
        .start()
        .expect("the bench starts");
    let mut reports = reports.take();
    let actor = virtual_clock::register_actor("cdp-signals-case");
    system.release_time();
    let started = Instant::now();
    while fake
        .seen()
        .replies
        .iter()
        .filter(|(name, _)| name == "setSignals")
        .count()
        < 2
    {
        assert!(started.elapsed() < HANG, "the signals were never answered");
        virtual_clock::wait_virtual_ns(MS);
    }
    let report = reports
        .iter_mut()
        .find(|report| report.subject() == "PC")
        .expect("the component reports");
    let looks = report.look(0);
    let summary = report.summary();
    assert!(fake.seen().opened, "flow-control-counted");
    assert!(
        summary.iter().any(|line| line
            == "1 setSignals call asserted DTR, RTS or a break, which no pin of the line carries"),
        "signals-counted: {summary:?}"
    );
    assert!(
        summary.iter().any(|line| line
            == "1 open asked for hardware flow control, which no pin of the line carries"),
        "flow-control-counted: {summary:?}"
    );
    assert!(
        looks.iter().any(|line| line
            == "a page asserted DTR and RTS (setSignals): the line has no modem-control pins, \
                so nothing on the board saw it")
            && looks.iter().any(|line| line
                == "a page opened the port with hardware flow control: the line has no RTS or \
                    CTS pin, so nothing paces its bytes"),
        "reported: {looks:?}"
    );
    drop(actor);
    system.shutdown();
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
        "unplugged-next-slice"
    );
    assert_eq!(fake.seen().lost, 1, "unplugged-next-slice");
    link.plug();
    virtual_clock::wait_virtual_ns(2 * MS);
    assert_eq!(
        fake.seen().states,
        [(0, true), (0, false), (1, true)],
        "replugged-next-slice"
    );
    assert_eq!(
        (stats.unplugs(), stats.plugs()),
        (1, 1),
        "replugged-next-slice"
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
        given: "a page whose port's stream was transferred to a dedicated worker that comes \
                back for more 30 ms of host time after each delivery, the board's side \
                writing five times",
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
        given: "a page whose port's stream was transferred to a worker that never reports \
                reading (it pipes the stream, which the probe cannot see), the board's side \
                writing five times",
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
    let node = node_with(&fake, 115_200, |s| {
        s.drain_bound = Duration::from_millis(50)
    });
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
#[case::port_with_attach(
    "baud = 9600\nattach = \"http://x\"\ndevtools_port = 9222",
    "options.devtools_port says how to launch Chrome"
)]
#[case::bad_attach("baud = 9600\nattach = \"127.0.0.1:9222\"", "is a DevTools endpoint")]
#[case::no_chrome("baud = 9600\nchrome = \"./no-such-chrome\"", "no Chrome at ")]
#[case::big_port(
    "baud = 9600\ndevtools_port = 70000",
    "options.devtools_port = 70000 is not a TCP port (1 to 65535)"
)]
#[case::zero_port(
    "baud = 9600\ndevtools_port = 0",
    "options.devtools_port = 0 is not a TCP port (1 to 65535)"
)]
#[case::no_page(
    "baud = 9600\nurl = \"no-such-page.html\"",
    "options.url = \"no-such-page.html\" is a page to open"
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
        given: "a chrome-cdp component with an option that cannot be built: a bad rate, \
                quantum, lead bound, USB id, port, endpoint or page, a launch option beside an \
                attach, no Chrome, or a mistyped or unknown option",
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
        message.contains("component PC (kind \"chrome-cdp\")"),
        "refused-saying-why: {message}"
    );
    assert!(message.contains(says), "refused-saying-why: {message}");
}

/// A project of a chrome-cdp attached to `url` and a host-serial on one
/// 3.3 V rail, `extra` added to the chrome-cdp's options.
fn project_text(url: &str, extra: &str) -> String {
    static PROJECTS: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
    let pty = std::env::temp_dir().join(format!(
        "embsim-cdp-project-{}-{}.pty",
        std::process::id(),
        PROJECTS.fetch_add(1, Ordering::Relaxed)
    ));
    format!(
        r#"[[component]]
name = "PC"
kind = "chrome-cdp"
[component.options]
baud = 115200
attach = "{url}"
usb_vendor_id = 0x0403
usb_product_id = 0x6001
granted = true
{extra}

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
"#
    )
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
    let text = project_text(&fake.url, "");
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
