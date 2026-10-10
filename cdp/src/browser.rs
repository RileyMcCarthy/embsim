//! The browser the node holds: its pages and dedicated workers, each held
//! from birth, and the three things a slice asks of it — a grant of virtual
//! time, the page's side of the slice, and the drain barrier.

use std::collections::BTreeMap;
use std::time::{Duration, Instant};

use serde_json::{json, Value};

use crate::chrome::{ChromeProcess, LaunchSpec};
use crate::devtools::{browser_ws_url, CdpError, DevTools, Event};

/// The page-side binding the shim sends through.
pub(crate) const PAGE_BINDING: &str = "__embsimTx";
/// The worker-side binding the read probe reports through.
pub(crate) const WORKER_BINDING: &str = "__embsimDrain";

/// The read probe each dedicated worker gets.
const PROBE: &str = include_str!("probe.js");

/// A worker's budget: so long it never expires in a run. A worker given
/// `advance` with no budget, or with `pauseIfNetworkFetchesPending`, froze
/// the shared clock in some boots (`NODES.md` §19, evidence E4).
const WORKER_BUDGET_MS: f64 = 1e9;

/// How often, and how far apart, the probe is tried in a worker whose
/// global scope is not ready yet: up to [`PROBE_BEFORE_RELEASE`] times
/// before the worker is released, then up to [`PROBE_TRIES`] more after
/// (`NODES.md` §19, evidence E4: every 2 ms took within 4 tries).
const PROBE_BEFORE_RELEASE: u32 = 8;
const PROBE_TRIES: u32 = 200;
const PROBE_RETRY: Duration = Duration::from_millis(2);

/// How long one setup command may go unanswered before the browser is
/// called unresponsive.
const COMMAND_TIMEOUT: Duration = Duration::from_secs(30);

/// How long the node waits for a page's answer or its budget's expiry
/// before it pokes the page with a command of no effect, and how often it
/// pokes again: every [`POKE_HIDDEN`] for a page that was hidden at its
/// last slice, every [`POKE_EVERY`] for any other. Chrome holds back what a
/// hidden page (a background tab) says over DevTools — an expiry, an
/// evaluate's answer — until something else reaches the page, sometimes for
/// good; a visible page's comes within 5 ms of host time (`NODES.md` §19,
/// evidence E11).
const POKE_EVERY: Duration = Duration::from_millis(20);
const POKE_HIDDEN: Duration = Duration::from_millis(2);

/// How the node reaches its browser.
#[derive(Debug, Clone)]
pub enum Browse {
    /// Start the host's Chrome.
    Launch(LaunchSpec),
    /// Attach to a browser already running: its DevTools HTTP endpoint
    /// (`http://127.0.0.1:9222`) or its WebSocket (`ws://…`).
    Attach(String),
}

/// A JavaScript dialog a page has open.
#[derive(Debug, Clone)]
pub(crate) struct Dialog {
    /// `alert`, `confirm`, `prompt` or `beforeunload`.
    pub kind: String,
    pub message: String,
}

/// One page the node holds.
#[derive(Debug)]
pub(crate) struct PageTarget {
    pub session: String,
    pub target: String,
    /// The order it was attached in.
    pub seq: u64,
    /// Attached while the node reached the browser: a page already open.
    pub at_boot: bool,
    /// The shim's document, once it said hello.
    pub doc: Option<String>,
    pub origin: Option<String>,
    pub href: Option<String>,
    /// What the page's binding said since the node last looked, oldest
    /// first.
    pub inbox: Vec<Value>,
    pub expired: bool,
    pub crashed: bool,
    /// The virtual clock the page lives on: Chrome's `virtualTimeTicksBase`
    /// (in µs), which every page whose main thread it shares reports.
    pub clock: Option<i64>,
    /// A dialog the page has open.
    pub dialog: Option<Dialog>,
    /// Whether the page was hidden (a background tab) at its last slice.
    pub hidden: bool,
}

impl PageTarget {
    /// `" at http://…"`, the page as an error names it.
    pub fn at(&self) -> String {
        match self.href.as_deref().or(self.origin.as_deref()) {
            Some(href) => format!(" at {href}"),
            None => String::new(),
        }
    }
}

/// One dedicated worker the node holds.
#[derive(Debug)]
pub(crate) struct WorkerTarget {
    /// The page whose worker it is (a nested worker's page is its outer
    /// worker's).
    pub page: String,
    /// Read calls on streams the worker was sent, as its probe reported
    /// them.
    pub reads: u64,
    /// Whether its read probe took.
    pub probed: bool,
    /// Whether it asked for its own `navigator.serial`.
    pub serial: bool,
}

/// The page `url` opens in, and the navigation's answer.
#[derive(Debug)]
struct Navigation {
    url: String,
    /// The page target to navigate, once it is held (attach mode makes one).
    target: Option<String>,
    /// The `Page.navigate` command, once sent.
    id: Option<u64>,
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
    pub workers_serial: u64,
    shim: String,
    booting: bool,
    navigation: Option<Navigation>,
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

/// Chrome's `virtualTimeTicksBase` in an answer, in µs.
fn clock_of(answer: &Value) -> Option<i64> {
    answer["virtualTimeTicksBase"]
        .as_f64()
        .map(|ms| (ms * 1e3).round() as i64)
}

impl Browser {
    /// Start or reach the browser and hold everything it has: every page
    /// and its dedicated workers paused from birth, the shim installed in
    /// every document. `shim` is the shim's source with its configuration.
    pub fn boot(browse: &Browse, shim: String) -> Result<Self, String> {
        let (process, endpoint, ws) = match browse {
            Browse::Launch(spec) => {
                let process = ChromeProcess::launch(spec).map_err(|e| e.to_string())?;
                let (endpoint, ws) = (process.endpoint(), process.ws_url());
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
            workers_serial: 0,
            shim,
            booting: true,
            navigation: None,
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

    /// The node has reached the browser: pages attached from now on were
    /// born in the run.
    pub fn booted(&mut self) {
        self.booting = false;
    }

    /// Open `url` in a page the node holds, without waiting for it: it
    /// loads on the board's time. A launched Chrome's own first tab takes
    /// it; in a browser the node attached to, a new window at about:blank
    /// is made for it. [`Self::navigation_failure`] reads how it went.
    pub fn open(&mut self, url: &str, launched: bool) -> Result<(), String> {
        let target = if launched {
            self.pages
                .values()
                .min_by_key(|page| page.seq)
                .map(|page| page.target.clone())
                .ok_or_else(|| "the browser has no page to open the url in".to_string())?
        } else {
            let made = self
                .devtools
                .call(
                    None,
                    "Target.createTarget",
                    json!({ "url": "about:blank", "newWindow": true }),
                    COMMAND_TIMEOUT,
                )
                .map_err(|e| format!("Chrome could not make a page for {url}: {e}"))?;
            made["targetId"].as_str().unwrap_or_default().to_string()
        };
        self.navigation = Some(Navigation {
            url: url.to_string(),
            target: Some(target),
            id: None,
        });
        let deadline = Instant::now() + Duration::from_secs(5);
        while self.navigation.as_ref().is_some_and(|n| n.id.is_none()) {
            self.navigate_when_held().map_err(|e| e.to_string())?;
            if self.navigation.as_ref().is_some_and(|n| n.id.is_some()) {
                break;
            }
            if Instant::now() >= deadline {
                return Err(format!("the page made for {url} never attached"));
            }
            self.pump(Instant::now() + Duration::from_millis(20))
                .map_err(|e| e.to_string())?;
        }
        Ok(())
    }

    /// Send the navigation `url` asked for once its page is held.
    fn navigate_when_held(&mut self) -> Result<(), CdpError> {
        let Some(navigation) = self.navigation.as_ref() else {
            return Ok(());
        };
        if navigation.id.is_some() {
            return Ok(());
        }
        let Some(session) = self
            .pages
            .values()
            .find(|page| Some(&page.target) == navigation.target.as_ref())
            .map(|page| page.session.clone())
        else {
            return Ok(());
        };
        let url = navigation.url.clone();
        let id = self
            .devtools
            .send(Some(&session), "Page.navigate", json!({ "url": url }))?;
        if let Some(navigation) = self.navigation.as_mut() {
            navigation.id = Some(id);
        }
        Ok(())
    }

    /// Why the navigation `url` asked for failed, once Chrome says it did.
    pub fn navigation_failure(&mut self) -> Option<String> {
        let id = self.navigation.as_ref()?.id?;
        let url = self.navigation.as_ref()?.url.clone();
        let answer = self.devtools.try_take(id)?;
        self.navigation = None;
        match answer {
            Ok(value) => value["errorText"]
                .as_str()
                .filter(|text| !text.is_empty())
                .map(|text| format!("the page at {url} did not open: {text}")),
            Err(e) if e.is_target_gone() => None,
            Err(e) => Some(format!("the page at {url} did not open: {e}")),
        }
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
                let target = str_of(info, "targetId");
                let waiting = event.params["waitingForDebugger"]
                    .as_bool()
                    .unwrap_or(false);
                let parent = event.session.clone().unwrap_or_default();
                // A worker's page: the page that attached it, or, for a
                // worker a worker started, that worker's page.
                let page = if self.pages.contains_key(&parent) {
                    Some(parent)
                } else {
                    self.workers.get(&parent).map(|w| w.page.clone())
                };
                match (kind.as_str(), page) {
                    ("page", _) => self.setup_page(session, target, waiting)?,
                    ("worker", Some(page)) => self.setup_worker(session, page, waiting)?,
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
                self.navigate_when_held()?;
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
                            if message["k"] == "hello" {
                                page.doc = message["doc"].as_str().map(str::to_string);
                                page.origin = message["origin"].as_str().map(str::to_string);
                                page.href = message["href"].as_str().map(str::to_string);
                            }
                            page.inbox.push(message);
                        }
                    }
                } else if name == WORKER_BINDING {
                    if let Some(worker) = self.workers.get_mut(&session) {
                        if payload == "serial" {
                            if !std::mem::replace(&mut worker.serial, true) {
                                self.workers_serial += 1;
                            }
                        } else {
                            worker.reads = worker.reads.max(payload.trim().parse().unwrap_or(0));
                        }
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
            // Browser-session form. Some Chrome builds emit this and not
            // the session-scoped Inspector event when a renderer dies.
            "Target.targetCrashed" => {
                let target = str_of(&event.params, "targetId");
                let session = self
                    .pages
                    .values()
                    .find(|page| page.target == target)
                    .map(|page| page.session.clone());
                if let Some(session) = session {
                    self.devtools.session_gone(&session);
                    if let Some(page) = self.pages.get_mut(&session) {
                        page.crashed = true;
                    }
                }
            }
            "Page.javascriptDialogOpening" => {
                if let Some(page) = event.session.and_then(|s| self.pages.get_mut(&s)) {
                    page.dialog = Some(Dialog {
                        kind: str_of(&event.params, "type"),
                        message: str_of(&event.params, "message"),
                    });
                }
            }
            "Page.javascriptDialogClosed" => {
                if let Some(page) = event.session.and_then(|s| self.pages.get_mut(&s)) {
                    page.dialog = None;
                }
            }
            _ => {}
        }
        Ok(())
    }

    /// Hold a page: its clock paused, the binding and the shim installed
    /// before its next document's scripts, its workers attached paused;
    /// then let it go on.
    fn setup_page(
        &mut self,
        session: String,
        target: String,
        waiting: bool,
    ) -> Result<(), CdpError> {
        let s = Some(session.as_str());
        let deadline = Instant::now() + COMMAND_TIMEOUT;
        let commands = [
            ("Runtime.enable", json!({})),
            ("Page.enable", json!({})),
            // Without this, Chrome does not deliver Inspector.targetCrashed.
            // A renderer that dies mid-grant then holds the budget until
            // stuck_after, and the run reports a stuck grant.
            ("Inspector.enable", json!({})),
            (
                "Emulation.setVirtualTimePolicy",
                json!({ "policy": "pause" }),
            ),
            ("Runtime.addBinding", json!({ "name": PAGE_BINDING })),
            (
                "Page.addScriptToEvaluateOnNewDocument",
                json!({ "source": self.shim, "runImmediately": true }),
            ),
            (
                "Target.setAutoAttach",
                json!({ "autoAttach": true, "waitForDebuggerOnStart": true, "flatten": true }),
            ),
        ];
        let mut ids = Vec::with_capacity(commands.len());
        for (method, params) in commands {
            ids.push((method, self.devtools.send(s, method, params)?));
        }
        // Released before the answers are awaited: a page whose renderer
        // does not exist yet (a window opened with `noopener`) answers
        // nothing until it runs, and Chrome delivers the commands above to
        // its renderer, in order, before the page's first script.
        if waiting {
            self.devtools
                .send_and_forget(s, "Runtime.runIfWaitingForDebugger", json!({}))?;
        }
        self.pages.insert(
            session.clone(),
            PageTarget {
                session: session.clone(),
                target,
                seq: self.pages_seen,
                at_boot: self.booting,
                doc: None,
                origin: None,
                href: None,
                inbox: Vec::new(),
                expired: false,
                crashed: false,
                clock: None,
                dialog: None,
                hidden: false,
            },
        );
        self.pages_seen += 1;
        for (method, id) in ids {
            match self.await_answer(id, &session, deadline) {
                Ok(answer) => {
                    if method == "Emulation.setVirtualTimePolicy" {
                        if let Some(page) = self.pages.get_mut(&session) {
                            page.clock = clock_of(&answer);
                        }
                    }
                }
                // A page that closed while it was being set up.
                Err(e) if e.is_target_gone() => {}
                Err(e) => return Err(e),
            }
        }
        Ok(())
    }

    /// Hold a dedicated worker: on the page's clock (`advance` with a
    /// budget that never runs out, so the page's grants drive it), the
    /// workers it starts attached paused, its read probe in, then let it
    /// go on.
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
        let nested = self.devtools.send(
            s,
            "Target.setAutoAttach",
            json!({ "autoAttach": true, "waitForDebuggerOnStart": true, "flatten": true }),
        )?;
        let metered = self.devtools.wait(policy, deadline).is_ok();
        let _ = self.devtools.wait(binding, deadline);
        let _ = self.devtools.wait(nested, deadline);
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
                serial: false,
            },
        );
        let mut probed = self.probe(&session, PROBE_BEFORE_RELEASE)?;
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
    /// wait until every one has expired, or `stuck` passes. A page still
    /// waiting after [`POKE_EVERY`] is poked, and again after each.
    pub fn grant(
        &mut self,
        budgets: &[(String, f64)],
        stuck: Duration,
    ) -> Result<Granted, CdpError> {
        let mut sent = Vec::with_capacity(budgets.len());
        for (session, budget) in budgets {
            if let Some(page) = self.pages.get_mut(session) {
                page.expired = false;
            }
            let id = self.devtools.send(
                Some(session),
                "Emulation.setVirtualTimePolicy",
                json!({ "policy": "pauseIfNetworkFetchesPending", "budget": budget }),
            )?;
            sent.push((id, session.clone()));
        }
        let deadline = Instant::now() + stuck;
        let interval = budgets
            .iter()
            .map(|(session, _)| self.poke_interval(session))
            .min()
            .unwrap_or(POKE_EVERY);
        let mut poke_at = Instant::now() + interval;
        loop {
            while let Some(event) = self.devtools.pop_event() {
                self.handle(event)?;
            }
            let pending: Vec<&(String, f64)> = budgets
                .iter()
                .filter(|(session, _)| {
                    self.pages
                        .get(session)
                        .is_some_and(|page| !page.expired && !page.crashed)
                })
                .collect();
            let answered = sent.iter().all(|(id, _)| self.devtools.answered(*id));
            if pending.is_empty() && answered {
                break;
            }
            let now = Instant::now();
            if now >= deadline {
                if let Some((session, budget_ms)) = pending.first() {
                    return Ok(Granted::Stuck {
                        session: session.clone(),
                        budget_ms: *budget_ms,
                    });
                }
                break;
            }
            if now >= poke_at {
                let sessions: Vec<String> = pending.iter().map(|(s, _)| s.clone()).collect();
                for session in sessions {
                    self.poke(&session)?;
                }
                poke_at = now + interval;
            }
            self.devtools.read_message(deadline.min(poke_at))?;
        }
        for (id, session) in sent {
            match self.devtools.wait(id, deadline) {
                Ok(answer) => {
                    if let Some(page) = self.pages.get_mut(&session) {
                        page.clock = clock_of(&answer).or(page.clock);
                    }
                }
                Err(e) if e.is_target_gone() => {}
                Err(e) => return Err(e),
            }
        }
        Ok(Granted::Expired)
    }

    /// The page's side of a slice: `__embsim.slice(arg)` evaluated in its
    /// main world, the page's clock held, waited for up to `bound`. `None`
    /// while the page has no document to evaluate in (between two).
    pub fn slice(
        &mut self,
        session: &str,
        arg: &Value,
        bound: Duration,
    ) -> Result<Option<Value>, CdpError> {
        let expression = format!(
            "(() => {{ const a = {arg}; return globalThis.__embsim ? globalThis.__embsim.slice(a) \
             : {{ t: performance.now(), o: performance.timeOrigin }}; }})()"
        );
        let id = self.devtools.send(
            Some(session),
            "Runtime.evaluate",
            json!({ "expression": expression, "returnByValue": true, "silent": true }),
        )?;
        match self.await_answer(id, session, Instant::now() + bound) {
            Ok(answer) => {
                if answer.get("exceptionDetails").is_some() {
                    return Ok(None);
                }
                Ok(answer["result"].get("value").cloned())
            }
            Err(e) if e.is_target_gone() => Ok(None),
            Err(e) => Err(e),
        }
    }

    /// Read calls the page's dedicated workers have made on streams they
    /// were sent: where a transferred port stream is read.
    pub fn reads(&self, session: &str) -> u64 {
        self.workers
            .values()
            .filter(|worker| worker.page == session)
            .map(|worker| worker.reads)
            .sum()
    }

    /// Wait until the page's workers have made `target` read calls in all
    /// — the bytes just handed over taken and the reader back for more —
    /// or the reader went away, or `bound` passed.
    pub fn drain(
        &mut self,
        session: &str,
        doc: &str,
        target: u64,
        bound: Duration,
    ) -> Result<Drained, CdpError> {
        let deadline = Instant::now() + bound;
        // A hidden page's workers are poked, as the page is; a visible
        // page's say what they read unprompted.
        let hidden = self.pages.get(session).is_some_and(|page| page.hidden);
        let mut poke_at = Instant::now() + POKE_HIDDEN;
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
            let now = Instant::now();
            if now >= deadline {
                return Ok(Drained::TimedOut);
            }
            if hidden && now >= poke_at {
                let workers: Vec<String> = self
                    .workers
                    .iter()
                    .filter(|(_, worker)| worker.page == session)
                    .map(|(id, _)| id.clone())
                    .collect();
                for worker in workers {
                    self.poke(&worker)?;
                }
                poke_at = now + POKE_HIDDEN;
            }
            let until = if hidden {
                deadline.min(poke_at)
            } else {
                deadline
            };
            self.devtools.read_message(until)?;
        }
    }
}

impl Browser {
    /// How long after its last word a page is poked.
    fn poke_interval(&self, session: &str) -> Duration {
        if self.pages.get(session).is_some_and(|page| page.hidden) {
            POKE_HIDDEN
        } else {
            POKE_EVERY
        }
    }

    /// A command of no effect to `session`, whose arrival lets Chrome send
    /// what a hidden page holds back.
    fn poke(&mut self, session: &str) -> Result<(), CdpError> {
        self.devtools.send_and_forget(
            Some(session),
            "Runtime.evaluate",
            json!({ "expression": "0", "silent": true }),
        )
    }

    /// Wait for command `id`'s answer from `session` until `deadline`,
    /// poking the session while it is quiet.
    fn await_answer(
        &mut self,
        id: u64,
        session: &str,
        deadline: Instant,
    ) -> Result<Value, CdpError> {
        let interval = self.poke_interval(session);
        let mut poke_at = Instant::now() + interval;
        loop {
            if let Some(answer) = self.devtools.try_take(id) {
                return answer;
            }
            let now = Instant::now();
            if now >= deadline {
                return self.devtools.wait(id, deadline);
            }
            if now >= poke_at {
                self.poke(session)?;
                poke_at = now + interval;
            }
            self.devtools.read_message(deadline.min(poke_at))?;
        }
    }
}

/// A command refused because its target went away is not an error here.
fn ignore_gone(e: CdpError) -> Result<Value, CdpError> {
    if e.is_target_gone() {
        Ok(Value::Null)
    } else {
        Err(e)
    }
}

fn str_of(value: &Value, key: &str) -> String {
    value
        .get(key)
        .and_then(Value::as_str)
        .unwrap_or_default()
        .to_string()
}
