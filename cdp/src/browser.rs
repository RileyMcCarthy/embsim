//! The browser the node holds: its pages and dedicated workers, each held
//! from birth, and the three things a slice asks of it — a grant of virtual
//! time, the page's side of the slice, and the drain barrier.

use std::collections::BTreeMap;
use std::time::{Duration, Instant};

use serde_json::{json, Value};

use crate::chrome::{ChromeProcess, LaunchSpec, LAUNCH_TIMEOUT};
use crate::devtools::{browser_ws_url, CdpError, DevTools, Event};

/// The page-side binding the shim sends through.
pub(crate) const PAGE_BINDING: &str = "__embsimTx";
/// The worker-side binding the read probe reports through.
pub(crate) const WORKER_BINDING: &str = "__embsimDrain";

/// The read probe each dedicated worker gets.
const PROBE: &str = include_str!("probe.js");

/// A worker's budget: so long it never expires in a run. A worker given
/// `advance` with no budget, or with `pauseIfNetworkFetchesPending`, froze
/// the shared clock in some boots (hc_antiCdp, 0 of 20 stuck with this one).
const WORKER_BUDGET_MS: f64 = 1e9;

/// How often, and how far apart, the probe is tried in a worker whose
/// global scope is not ready yet (hc_antiCdp: every 2 ms took within 4).
const PROBE_TRIES: u32 = 200;
const PROBE_RETRY: Duration = Duration::from_millis(2);

/// How long one setup or evaluate command may go unanswered before the
/// browser is called unresponsive.
const COMMAND_TIMEOUT: Duration = Duration::from_secs(30);

/// How the node reaches its browser.
#[derive(Debug, Clone)]
pub enum Browse {
    /// Start the host's Chrome.
    Launch(LaunchSpec),
    /// Attach to a browser already running: its DevTools HTTP endpoint
    /// (`http://127.0.0.1:9222`) or its WebSocket (`ws://…`).
    Attach(String),
}

/// One page the node holds.
#[derive(Debug)]
pub(crate) struct PageTarget {
    pub session: String,
    /// The shim's document, once it said hello.
    pub doc: Option<String>,
    pub origin: Option<String>,
    /// Read calls on the port's readers in this document's realm.
    pub reads: u64,
    /// What the page's binding said since the node last looked, oldest
    /// first.
    pub inbox: Vec<Value>,
    pub expired: bool,
    pub crashed: bool,
}

/// One dedicated worker the node holds.
#[derive(Debug)]
pub(crate) struct WorkerTarget {
    /// The page whose worker it is.
    pub page: String,
    /// Read calls in the worker, as its probe reported them.
    pub reads: u64,
    /// Whether its read probe took.
    pub probed: bool,
}

/// The node's browser.
#[derive(Debug)]
pub(crate) struct Browser {
    pub devtools: DevTools,
    pub process: Option<ChromeProcess>,
    pub endpoint: String,
    pub version: String,
    pub pages: BTreeMap<String, PageTarget>,
    pub workers: BTreeMap<String, WorkerTarget>,
    /// Pages that went away since the node last looked.
    pub gone: Vec<String>,
    /// Counts for the report.
    pub pages_seen: u64,
    pub workers_seen: u64,
    pub workers_unmetered: u64,
    pub workers_unprobed: u64,
    shim: String,
}

/// What a grant came to.
pub(crate) enum Granted {
    /// Every page's budget expired.
    Expired,
    /// A page's budget did not expire within the bound.
    Stuck { session: String, budget_ms: f64 },
}

/// What the drain barrier came to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Drained {
    /// The consumer came back for more.
    Met,
    /// The consumer went away (cancelled its read, the page went).
    Released,
    /// The bound passed first.
    TimedOut,
}

impl Browser {
    /// Start or reach the browser and hold everything it has: every page
    /// and its dedicated workers paused from birth, the shim installed in
    /// every document. `shim` is the shim's source with its configuration.
    pub fn boot(browse: &Browse, shim: String) -> Result<Self, String> {
        let (process, endpoint, ws) = match browse {
            Browse::Launch(spec) => {
                let process = ChromeProcess::launch(spec).map_err(|e| e.to_string())?;
                let endpoint = process.endpoint();
                let ws = browser_ws_url(&endpoint, LAUNCH_TIMEOUT)
                    .map_err(|e| format!("Chrome started ({}), but {e}", spec.binary.display()))?;
                (Some(process), endpoint, ws)
            }
            Browse::Attach(endpoint) if endpoint.starts_with("ws://") => {
                (None, endpoint.clone(), endpoint.clone())
            }
            Browse::Attach(endpoint) => {
                let ws =
                    browser_ws_url(endpoint, Duration::from_secs(5)).map_err(|e| e.to_string())?;
                (None, endpoint.clone(), ws)
            }
        };
        let devtools = DevTools::connect(&ws, COMMAND_TIMEOUT)
            .map_err(|e| format!("cannot reach Chrome's DevTools at {ws}: {e}"))?;
        let mut browser = Self {
            devtools,
            process,
            endpoint,
            version: String::new(),
            pages: BTreeMap::new(),
            workers: BTreeMap::new(),
            gone: Vec::new(),
            pages_seen: 0,
            workers_seen: 0,
            workers_unmetered: 0,
            workers_unprobed: 0,
            shim,
        };
        let version = browser
            .devtools
            .call(None, "Browser.getVersion", json!({}), COMMAND_TIMEOUT)
            .map_err(|e| e.to_string())?;
        browser.version = version
            .get("product")
            .and_then(Value::as_str)
            .unwrap_or("Chrome")
            .to_string();
        // Every page, and every page made from now on, attaches paused
        // until the node has set it up.
        browser
            .devtools
            .call(
                None,
                "Target.setAutoAttach",
                json!({ "autoAttach": true, "waitForDebuggerOnStart": true, "flatten": true }),
                COMMAND_TIMEOUT,
            )
            .map_err(|e| e.to_string())?;
        browser.pump(Instant::now()).map_err(|e| e.to_string())?;
        Ok(browser)
    }

    /// Navigate the first page the node holds to `url`, once it is set up,
    /// without waiting for the navigation: it lives on the board's time.
    pub fn open(&mut self, url: &str) -> Result<(), String> {
        let session = self
            .pages
            .keys()
            .next()
            .cloned()
            .ok_or_else(|| "the browser has no page to open the url in".to_string())?;
        self.devtools
            .send_and_forget(Some(&session), "Page.navigate", json!({ "url": url }))
            .map_err(|e| e.to_string())
    }

    /// Handle every event already read, then read until `deadline` while
    /// the socket holds more.
    pub fn pump(&mut self, deadline: Instant) -> Result<(), CdpError> {
        loop {
            while let Some(event) = self.devtools.pop_event() {
                self.handle(event)?;
            }
            if !self.devtools.read_message(deadline)? {
                return Ok(());
            }
        }
    }

    /// Handle what is already read and whatever the socket holds now.
    pub fn drain_socket(&mut self) -> Result<(), CdpError> {
        self.pump(Instant::now())
    }

    fn handle(&mut self, event: Event) -> Result<(), CdpError> {
        match event.method.as_str() {
            "Target.attachedToTarget" => {
                let session = str_of(&event.params, "sessionId");
                let info = &event.params["targetInfo"];
                let kind = str_of(info, "type");
                let waiting = event.params["waitingForDebugger"]
                    .as_bool()
                    .unwrap_or(false);
                match kind.as_str() {
                    "page" => self.setup_page(session, waiting)?,
                    "worker" => {
                        let page = event.session.clone().unwrap_or_default();
                        self.setup_worker(session, page, waiting)?;
                    }
                    // Service workers, shared workers and out-of-process
                    // frames run unmetered (PROJECTS.md, `chrome-cdp`).
                    _ => {
                        if waiting {
                            self.devtools.send_and_forget(
                                Some(&session),
                                "Runtime.runIfWaitingForDebugger",
                                json!({}),
                            )?;
                        }
                    }
                }
            }
            "Target.detachedFromTarget" => {
                let session = str_of(&event.params, "sessionId");
                if self.pages.remove(&session).is_some() {
                    self.workers.retain(|_, w| w.page != session);
                    self.gone.push(session);
                } else {
                    self.workers.remove(&session);
                }
            }
            "Runtime.bindingCalled" => {
                let session = event.session.unwrap_or_default();
                let name = str_of(&event.params, "name");
                let payload = str_of(&event.params, "payload");
                if name == PAGE_BINDING {
                    if let Some(page) = self.pages.get_mut(&session) {
                        if let Ok(message) = serde_json::from_str::<Value>(&payload) {
                            match message["k"].as_str() {
                                Some("read") => {
                                    if page.doc.as_deref() == message["doc"].as_str() {
                                        page.reads =
                                            page.reads.max(message["n"].as_u64().unwrap_or(0));
                                    }
                                }
                                Some("hello") => {
                                    page.doc = message["doc"].as_str().map(str::to_string);
                                    page.origin = message["origin"].as_str().map(str::to_string);
                                    page.reads = 0;
                                    page.inbox.push(message);
                                }
                                _ => page.inbox.push(message),
                            }
                        }
                    }
                } else if name == WORKER_BINDING {
                    if let Some(worker) = self.workers.get_mut(&session) {
                        worker.reads = worker.reads.max(payload.trim().parse().unwrap_or(0));
                    }
                }
            }
            "Emulation.virtualTimeBudgetExpired" => {
                if let Some(page) = event.session.and_then(|s| self.pages.get_mut(&s)) {
                    page.expired = true;
                }
            }
            "Inspector.targetCrashed" => {
                if let Some(page) = event.session.and_then(|s| self.pages.get_mut(&s)) {
                    page.crashed = true;
                }
            }
            _ => {}
        }
        Ok(())
    }

    /// Hold a page: its clock paused, the binding and the shim installed
    /// before its next document's scripts, its workers attached paused;
    /// then let it go on.
    fn setup_page(&mut self, session: String, waiting: bool) -> Result<(), CdpError> {
        let s = Some(session.as_str());
        let deadline = Instant::now() + COMMAND_TIMEOUT;
        let ids = [
            self.devtools.send(s, "Runtime.enable", json!({}))?,
            self.devtools.send(s, "Page.enable", json!({}))?,
            self.devtools.send(
                s,
                "Emulation.setVirtualTimePolicy",
                json!({ "policy": "pause" }),
            )?,
            self.devtools
                .send(s, "Runtime.addBinding", json!({ "name": PAGE_BINDING }))?,
            self.devtools.send(
                s,
                "Page.addScriptToEvaluateOnNewDocument",
                json!({ "source": self.shim, "runImmediately": true }),
            )?,
            self.devtools.send(
                s,
                "Target.setAutoAttach",
                json!({ "autoAttach": true, "waitForDebuggerOnStart": true, "flatten": true }),
            )?,
        ];
        self.pages.insert(
            session.clone(),
            PageTarget {
                session: session.clone(),
                doc: None,
                origin: None,
                reads: 0,
                inbox: Vec::new(),
                expired: false,
                crashed: false,
            },
        );
        self.pages_seen += 1;
        for id in ids {
            match self.devtools.wait(id, deadline) {
                Ok(_) => {}
                // A page that closed while it was being set up.
                Err(CdpError::Protocol { .. }) => {}
                Err(e) => return Err(e),
            }
        }
        if waiting {
            self.devtools
                .send_and_forget(s, "Runtime.runIfWaitingForDebugger", json!({}))?;
        }
        Ok(())
    }

    /// Hold a dedicated worker: on the page's clock (`advance` with a
    /// budget that never runs out, so the page's grants drive it), its read
    /// probe in, then let it go on.
    fn setup_worker(
        &mut self,
        session: String,
        page: String,
        waiting: bool,
    ) -> Result<(), CdpError> {
        let s = Some(session.as_str());
        let deadline = Instant::now() + COMMAND_TIMEOUT;
        let policy = self.devtools.send(
            s,
            "Emulation.setVirtualTimePolicy",
            json!({ "policy": "advance", "budget": WORKER_BUDGET_MS }),
        )?;
        let binding =
            self.devtools
                .send(s, "Runtime.addBinding", json!({ "name": WORKER_BINDING }))?;
        let metered = self.devtools.wait(policy, deadline).is_ok();
        let _ = self.devtools.wait(binding, deadline);
        self.workers_seen += 1;
        if !metered {
            self.workers_unmetered += 1;
            tracing::error!(
                "chrome-cdp: Chrome refused a virtual-time policy on a dedicated worker; its \
                 timers run on host time"
            );
        }
        self.workers.insert(
            session.clone(),
            WorkerTarget {
                page,
                reads: 0,
                probed: false,
            },
        );
        let mut probed = self.probe(&session, 8)?;
        if waiting {
            self.devtools
                .call(
                    s,
                    "Runtime.runIfWaitingForDebugger",
                    json!({}),
                    COMMAND_TIMEOUT,
                )
                .or_else(ignore_gone)?;
        }
        if !probed {
            probed = self.probe(&session, PROBE_TRIES)?;
        }
        if let Some(worker) = self.workers.get_mut(&session) {
            worker.probed = probed;
        }
        if !probed {
            self.workers_unprobed += 1;
        }
        Ok(())
    }

    /// Try the read probe in a worker up to `tries` times.
    fn probe(&mut self, session: &str, tries: u32) -> Result<bool, CdpError> {
        for _ in 0..tries {
            let answer = self
                .devtools
                .call(
                    Some(session),
                    "Runtime.evaluate",
                    json!({ "expression": PROBE, "returnByValue": true, "silent": true }),
                    COMMAND_TIMEOUT,
                )
                .or_else(ignore_gone)?;
            if answer["result"]["value"].as_i64() == Some(1) {
                return Ok(true);
            }
            if !self.workers.contains_key(session) {
                return Ok(false);
            }
            std::thread::sleep(PROBE_RETRY);
        }
        Ok(false)
    }

    /// Grant each page its budget, in milliseconds of virtual time, and
    /// wait until every one has expired, or `stuck` passes.
    pub fn grant(
        &mut self,
        budgets: &[(String, f64)],
        stuck: Duration,
    ) -> Result<Granted, CdpError> {
        let mut ids = Vec::with_capacity(budgets.len());
        for (session, budget) in budgets {
            if let Some(page) = self.pages.get_mut(session) {
                page.expired = false;
            }
            ids.push(self.devtools.send(
                Some(session),
                "Emulation.setVirtualTimePolicy",
                json!({ "policy": "pauseIfNetworkFetchesPending", "budget": budget }),
            )?);
        }
        let deadline = Instant::now() + stuck;
        loop {
            while let Some(event) = self.devtools.pop_event() {
                self.handle(event)?;
            }
            let pending = budgets.iter().find(|(session, _)| {
                self.pages
                    .get(session)
                    .is_some_and(|page| !page.expired && !page.crashed)
            });
            let answered = ids.iter().all(|id| self.devtools.answered(*id));
            match pending {
                None if answered => break,
                _ => {}
            }
            if !self.devtools.read_message(deadline)? {
                if let Some((session, budget_ms)) = pending {
                    return Ok(Granted::Stuck {
                        session: session.clone(),
                        budget_ms: *budget_ms,
                    });
                }
                break;
            }
        }
        for id in ids {
            match self.devtools.wait(id, deadline) {
                Ok(_) | Err(CdpError::Protocol { .. }) => {}
                Err(e) => return Err(e),
            }
        }
        Ok(Granted::Expired)
    }

    /// The page's side of a slice: `__embsim.slice(arg)` evaluated in its
    /// main world, the page's clock held. `None` while the page has no
    /// document to evaluate in (between two).
    pub fn slice(&mut self, session: &str, arg: &Value) -> Result<Option<Value>, CdpError> {
        let expression = format!(
            "(() => {{ const a = {arg}; return globalThis.__embsim ? globalThis.__embsim.slice(a) \
             : {{ t: performance.timeOrigin + performance.now() }}; }})()"
        );
        let id = self.devtools.send(
            Some(session),
            "Runtime.evaluate",
            json!({ "expression": expression, "returnByValue": true, "silent": true }),
        )?;
        match self.devtools.wait(id, Instant::now() + COMMAND_TIMEOUT) {
            Ok(answer) => {
                if answer.get("exceptionDetails").is_some() {
                    return Ok(None);
                }
                Ok(answer["result"].get("value").cloned())
            }
            Err(CdpError::Protocol { .. }) => Ok(None),
            Err(e) => Err(e),
        }
    }

    /// Read calls the page's consumers have made: its own realm's on the
    /// port's readers, and every one of its dedicated workers'.
    pub fn reads(&self, session: &str) -> u64 {
        let own = self.pages.get(session).map_or(0, |page| page.reads);
        own + self
            .workers
            .values()
            .filter(|worker| worker.page == session)
            .map(|worker| worker.reads)
            .sum::<u64>()
    }

    /// Wait until the page's consumers have made `target` read calls in
    /// all — the bytes just handed over taken and the reader back for more
    /// — or the reader went away, or `bound` passed.
    pub fn drain(
        &mut self,
        session: &str,
        doc: &str,
        target: u64,
        bound: Duration,
    ) -> Result<Drained, CdpError> {
        let deadline = Instant::now() + bound;
        loop {
            while let Some(event) = self.devtools.pop_event() {
                self.handle(event)?;
            }
            let Some(page) = self.pages.get(session) else {
                return Ok(Drained::Released);
            };
            if page.crashed || page.doc.as_deref() != Some(doc) {
                return Ok(Drained::Released);
            }
            // The consumer cancelled its read: the port flushes.
            let cancelled = page
                .inbox
                .iter()
                .any(|m| m["k"] == "req" && m["op"] == "flush" && m["dir"] == "rx");
            if cancelled {
                return Ok(Drained::Released);
            }
            if self.reads(session) >= target {
                return Ok(Drained::Met);
            }
            if !self.devtools.read_message(deadline)? {
                return Ok(Drained::TimedOut);
            }
        }
    }
}

/// A command refused because its target went away is not an error here.
fn ignore_gone(e: CdpError) -> Result<Value, CdpError> {
    match e {
        CdpError::Protocol { .. } => Ok(Value::Null),
        other => Err(other),
    }
}

fn str_of(value: &Value, key: &str) -> String {
    value
        .get(key)
        .and_then(Value::as_str)
        .unwrap_or_default()
        .to_string()
}
