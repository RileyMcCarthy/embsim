//! The node against the host's real Chrome.
//!
//! `#[ignore]`d: each case launches Chrome (the system's, or `CHROME`), so
//! the test job does not run them; CI's `chrome-cdp` job does, with the
//! runner's `google-chrome`. Pages are served from `tests/pages` by a
//! static server in the test, cross-origin isolated, so `performance.now()`
//! resolves to 5 µs. The board's side of the line is a peer that records
//! what it hears (and echoes it, or plays a board's protocol); the case's
//! thread is the stepped clock's actor and reads the page between steps
//! through a DevTools connection of its own, as a harness's
//! `connectOverCDP` would, and makes contexts and pages through another,
//! as a harness does a scenario's.
//!
//! What is asserted is what the page and the board did in virtual time:
//! ticks counted, clocks read, bytes compared, failures worded. Host time
//! is measured and printed, never asserted; the wall-time bound on a wait
//! is sized for a hang.

mod common;

use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::time::{Duration, Instant};

use common::{bench, finish, suite_lock, Driver, PageClient, Peer, Server};
use embsim_cdp::{find_chrome, Browse, CdpNode, LaunchSpec, NodeStats, Settings, UsbIds};
use embsim_core::virtual_clock;
use serde_json::{json, Value};

/// How long, in wall time, a case may wait for the page before it is hung.
const HANG: Duration = Duration::from_secs(180);

/// One millisecond of board time.
const MS: u64 = 1_000_000;

fn pages() -> std::path::PathBuf {
    std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/pages")
}

/// A node that launches the host's Chrome on `url`, headless, on a DevTools
/// port Chrome picks, granted the port (as Chrome's
/// `SerialAllowUsbDevicesForUrls` policy grants it), its adapter an FTDI's
/// ids, then as `set` says.
fn node_with(url: &str, baud: u32, set: impl FnOnce(&mut Settings)) -> CdpNode {
    let binary = find_chrome().expect("a Chrome on this host (or CHROME names one)");
    let mut settings = Settings::new(Browse::Launch(LaunchSpec {
        binary,
        port: 0,
        headless: true,
    }));
    settings.url = Some(url.to_string());
    settings.granted = true;
    settings.usb = UsbIds {
        vendor: Some(0x0403),
        product: Some(0x6001),
    };
    set(&mut settings);
    CdpNode::new(settings, baud).expect("the settings run")
}

fn node(url: &str, baud: u32) -> CdpNode {
    node_with(url, baud, |_| {})
}

/// Hand the board time a millisecond at a time until `done`, or the case
/// is hung (or the node failed, when `failing` is false).
fn wait_for(stats: &NodeStats, what: &str, failing: bool, mut done: impl FnMut() -> bool) {
    let started = Instant::now();
    while !done() {
        assert!(
            started.elapsed() < HANG && (failing || stats.failure().is_none()),
            "{what} never happened by {} ns of board time: {stats:?}",
            virtual_clock::virtual_ns()
        );
        virtual_clock::wait_virtual_ns(MS);
    }
}

/// Hand the board time a millisecond at a time until `done` holds of the
/// page, or the case is hung.
fn until(
    page: &mut PageClient,
    stats: &NodeStats,
    what: &str,
    mut done: impl FnMut(&mut PageClient) -> bool,
) {
    wait_for(stats, what, false, || done(page));
}

/// Run the board until the node has reached Chrome: its DevTools endpoint.
fn booted(stats: &NodeStats) -> String {
    wait_for(stats, "Chrome", false, || stats.booted().is_some());
    stats
        .devtools_endpoint()
        .expect("the endpoint, once booted")
}

/// Run the board until the node has reached Chrome, and attach the case's
/// own DevTools client to the page at `url`, once it is ready.
fn page_up(url: &str, stats: &NodeStats) -> PageClient {
    let endpoint = booted(stats);
    let mut page = PageClient::attach(&endpoint, url);
    until(&mut page, stats, "the page's ready flag", |page| {
        page.try_eval("window.ready === true") == Ok(Value::Bool(true))
    });
    page
}

/// Start `name` in the page and run the board until it settles.
fn run(page: &mut PageClient, stats: &NodeStats, name: &str, call: &str) -> Value {
    page.eval(&format!("window.run({name:?}, () => {call})"));
    until(page, stats, name, |page| {
        page.try_eval(&format!("window.out[{name:?}].done")) == Ok(Value::Bool(true))
    });
    let out = page.eval(&format!("window.out[{name:?}]"));
    assert!(out.get("error").is_none(), "{name} failed: {out}");
    out["value"].clone()
}

fn f64s(value: &Value) -> Vec<f64> {
    value
        .as_array()
        .expect("an array")
        .iter()
        .map(|v| v.as_f64().expect("a number"))
        .collect()
}

/// How many of a timer's `ticks` fall in `span` ms of its clock from the
/// first tick after `from`, the window's edges half a period from any
/// tick: a timer every `period` ms that lives the board's time ticks
/// exactly `span / period` times in it, whatever its phase.
fn ticks_in(ticks: &[f64], from: f64, span: f64, period: f64) -> usize {
    let first = ticks
        .iter()
        .copied()
        .find(|&t| t > from)
        .expect("a tick after the window opens");
    let start = first - period / 2.0;
    ticks
        .iter()
        .filter(|&&t| t > start && t <= start + span)
        .count()
}

fn host_line(stats: &NodeStats) -> String {
    let ms = |p: f64| {
        stats
            .host_per_slice(p)
            .map_or(f64::NAN, |d| d.as_secs_f64() * 1e3)
    };
    format!(
        "host ms per slice: median {:.2}, p90 {:.2}, p99 {:.2}; peak lead {:.3} ms; peak \
         overrun {:.3} ms; {} slices, {} skipped",
        ms(0.5),
        ms(0.9),
        ms(0.99),
        stats.peak_lead_ns() as f64 / 1e6,
        stats.peak_overrun_ns() as f64 / 1e6,
        stats.slices(),
        stats.skipped()
    )
}

// ============================================================
// Clocks
// ============================================================

/// Given the host's Chrome metered every millisecond by the node, its page and
/// a dedicated worker each running a 4 ms timer stamped with its clock, run for
/// 400 milliseconds of board time once the page is up:
/// - the worker's timer ticks exactly 100 times in 400 ms of its clock, 4.000
///   ms apart to within 10 µs (`worker-ticks-100`) — a dedicated worker shares
///   the page's clock only once its own session is given a virtual-time
///   policy, so its timers advance in the page's grants
/// - the page's own 4 ms timer ticks exactly 100 times in 400 ms of its clock
///   (`page-ticks-100`)
/// - the page's performance.now() advances by the board's 400 ms to within 10
///   µs, and Date.now() by 400 ms to within 1 ms (`clocks-advance-by-the-board`)
///
/// Not a declared behaviour: the case is `#[ignore]`d where the ledger is
/// collected (TESTING.md).
#[test]
#[ignore = "launches the host's Chrome; CI's chrome-cdp job runs it"]
fn a_workers_four_millisecond_timer_ticks_once_every_four_grants() {
    let _suite = suite_lock();
    let server = Server::start(&pages());
    let url = format!("{}clocks.html", server.url);
    let node = node(&url, 115_200);
    let stats = node.stats();
    let (peer, _peer) = Peer::new(115_200, false);
    let (system, actor) = bench(node, Box::new(peer));
    let mut page = page_up(&url, &stats);
    // The timers start as the page says it is ready; their first period can
    // be short, so the window opens once they are ticking steadily.
    virtual_clock::wait_virtual_ns(20 * MS);

    let before = page.eval("window.clocks()");
    let from = before["now"].as_f64().unwrap();
    virtual_clock::wait_virtual_ns(400 * MS);
    let at = page.eval("window.clocks()");
    let to = at["now"].as_f64().unwrap();
    // A tick due at the window's far edge lands by the next slice, and a
    // worker's message by the one after.
    virtual_clock::wait_virtual_ns(8 * MS);
    let after = page.eval("window.clocks()");
    eprintln!("canary: {}", host_line(&stats));

    let worker = f64s(&after["worker"]);
    let main = f64s(&after["main"]);
    assert_eq!(
        ticks_in(&worker, from, 400.0, 4.0),
        100,
        "worker-ticks-100: {worker:?}"
    );
    for pair in worker.windows(2).filter(|pair| pair[0] > from) {
        assert!(
            (pair[1] - pair[0] - 4.0).abs() <= 0.01,
            "worker-ticks-100: ticks {pair:?} are not 4 ms apart"
        );
    }
    assert_eq!(
        ticks_in(&main, from, 400.0, 4.0),
        100,
        "page-ticks-100: {main:?}"
    );
    assert!(
        (to - from - 400.0).abs() <= 0.01,
        "clocks-advance-by-the-board: performance.now() moved {} ms",
        to - from
    );
    let date = at["date"].as_f64().unwrap() - before["date"].as_f64().unwrap();
    assert!(
        (date - 400.0).abs() <= 1.0,
        "clocks-advance-by-the-board: Date.now() moved {date} ms"
    );
    assert_eq!(stats.failure(), None);
    finish(system, actor);
}

/// Given a page held by the node, and a harness of its own (a second DevTools
/// client, as Playwright's connectOverCDP is) that makes a fresh browser
/// context, a page in it at about:blank, and navigates that page into a
/// cross-origin-isolated page with a dedicated worker; the node allowed a lead
/// of 20 ms; the context closed from the harness's own thread while the board
/// runs:
/// - the shim is in before the page's first script (`held-from-birth`)
/// - while the board is held for 300 ms of host time, neither the page's timer
///   nor its worker's ticks (`frozen-while-held`)
/// - over 100 ms of board time each ticks every 4 ms of its clock
///   (`metered`)
/// - the navigation into the isolated page is booked level with the board: no
///   lead, and the run goes on (`navigation-level`)
/// - once the context is closed the run goes on, with no failure, the board's
///   clock running (`close-let-go`)
///
/// Not a declared behaviour: the case is `#[ignore]`d where the ledger is
/// collected (TESTING.md).
#[test]
#[ignore = "launches the host's Chrome; CI's chrome-cdp job runs it"]
fn a_scenarios_page_in_a_fresh_context_is_held_from_birth_and_let_go_when_closed() {
    let _suite = suite_lock();
    let server = Server::start(&pages());
    let url = format!("{}ticks.html?main", server.url);
    let node = node_with(&url, 115_200, |s| {
        s.max_lead = Some(Duration::from_millis(20))
    });
    let stats = node.stats();
    let (peer, _peer) = Peer::new(115_200, false);
    let (system, actor) = bench(node, Box::new(peer));
    let _main = page_up(&url, &stats);
    let endpoint = stats.devtools_endpoint().unwrap();

    let mut harness = Driver::connect(&endpoint);
    let context = harness.new_context();
    let (_target, session) =
        harness.new_page(json!({ "url": "about:blank", "browserContextId": context }));
    // The page waits for the node, which holds it at its next slice; the
    // case's thread lets the board run before it asks anything of it.
    virtual_clock::wait_virtual_ns(5 * MS);
    harness.navigate(
        &session,
        &format!("{}ticks.html?worker&scenario", server.url),
    );
    wait_for(&stats, "the scenario's page", false, || {
        harness.try_eval(&session, "window.ready === true") == Ok(Value::Bool(true))
            && harness
                .try_eval(&session, "window.worker.ticks.length > 2")
                .is_ok_and(|v| v == Value::Bool(true))
    });
    assert_eq!(
        harness.eval(&session, "window.firstSawShim"),
        Value::Bool(true),
        "held-from-birth"
    );

    let count = |harness: &mut Driver| {
        let v = harness.eval(
            &session,
            "[window.ticks.length, window.worker.ticks.length]",
        );
        (v[0].as_u64().unwrap(), v[1].as_u64().unwrap())
    };
    let held = count(&mut harness);
    std::thread::sleep(Duration::from_millis(300));
    assert_eq!(count(&mut harness), held, "frozen-while-held");

    let from = harness
        .eval(&session, "performance.now()")
        .as_f64()
        .unwrap();
    let worker_from = harness
        .eval(&session, "performance.timeOrigin + performance.now()")
        .as_f64()
        .unwrap();
    virtual_clock::wait_virtual_ns(110 * MS);
    let ticks = f64s(&harness.eval(&session, "window.ticks"));
    let worker = f64s(&harness.eval(&session, "window.worker.ticks"));
    assert_eq!(ticks_in(&ticks, from, 100.0, 4.0), 25, "metered: {ticks:?}");
    assert_eq!(
        ticks_in(&worker, worker_from, 100.0, 4.0),
        25,
        "metered: {worker:?}"
    );
    assert_eq!(stats.failure(), None, "navigation-level");
    assert!(stats.peak_lead_ns() < MS, "navigation-level: {stats:?}");

    // The harness's teardown, from a thread of its own while the board
    // runs, as a test runner closes a scenario's context.
    let closer = {
        let endpoint = endpoint.clone();
        std::thread::spawn(move || {
            let mut harness = Driver::connect(&endpoint);
            std::thread::sleep(Duration::from_millis(100));
            harness.dispose_context(&context);
        })
    };
    let slices = stats.slices();
    let board = virtual_clock::virtual_ns();
    wait_for(&stats, "300 ms past the close", false, || {
        virtual_clock::virtual_ns() >= board + 300 * MS && closer.is_finished()
    });
    closer.join().expect("the context closes");
    eprintln!("scenario: {}", host_line(&stats));
    assert_eq!(stats.failure(), None, "close-let-go");
    assert!(stats.slices() >= slices + 300, "close-let-go: {stats:?}");
    finish(system, actor);
}

/// Given a page held by the node; a harness that opens a second tab in the
/// same browser context (so the first is hidden, a background tab) and
/// navigates it to a page of its own; and the first page opening a popup
/// window on a user's gesture, which shares its renderer and so its clock:
/// - each of the three pages' 4 ms timers ticks exactly 50 times in 200 ms of
///   its clock (`all-metered`)
/// - no grant sticks and no page leads the board (`nothing-stuck`)
/// - the node grants the page and its popup once between them
///   (`shared-once`)
///
/// Not a declared behaviour: the case is `#[ignore]`d where the ledger is
/// collected (TESTING.md).
#[test]
#[ignore = "launches the host's Chrome; CI's chrome-cdp job runs it"]
fn a_hidden_tab_and_a_popup_are_metered_with_the_page() {
    let _suite = suite_lock();
    let server = Server::start(&pages());
    let url = format!("{}ticks.html?first", server.url);
    let node = node_with(&url, 115_200, |s| {
        s.max_lead = Some(Duration::from_millis(20))
    });
    let stats = node.stats();
    let (peer, _peer) = Peer::new(115_200, false);
    let (system, actor) = bench(node, Box::new(peer));
    let mut first = page_up(&url, &stats);
    let endpoint = stats.devtools_endpoint().unwrap();

    let mut harness = Driver::connect(&endpoint);
    let (_target, second) = harness.new_page(json!({ "url": "about:blank" }));
    virtual_clock::wait_virtual_ns(5 * MS);
    harness.navigate(&second, &format!("{}ticks.html?second", server.url));
    wait_for(&stats, "the second tab", false, || {
        harness.try_eval(&second, "window.ready === true") == Ok(Value::Bool(true))
    });
    first.eval_gesture(
        "setTimeout(() => { window.child = window.open('ticks.html?popup', '_blank', \
         'popup,width=400,height=300'); }, 0), true",
    );
    until(&mut first, &stats, "the popup", |page| {
        page.try_eval("!!(window.child && window.child.ready)") == Ok(Value::Bool(true))
    });
    eprintln!(
        "visibility: first {}, second {}",
        first.eval("document.visibilityState"),
        harness.eval(&second, "document.visibilityState")
    );
    // The popup's timer starts as it loads; its first period can be short.
    virtual_clock::wait_virtual_ns(20 * MS);

    let read = |first: &mut PageClient, harness: &mut Driver| {
        (
            first.eval("performance.now()").as_f64().unwrap(),
            harness.eval(&second, "performance.now()").as_f64().unwrap(),
            first
                .eval("window.child.performance.now()")
                .as_f64()
                .unwrap(),
        )
    };
    let (f, s, p) = read(&mut first, &mut harness);
    virtual_clock::wait_virtual_ns(206 * MS);
    let ticks = (
        f64s(&first.eval("window.ticks")),
        f64s(&harness.eval(&second, "window.ticks")),
        f64s(&first.eval("window.child.ticks")),
    );
    eprintln!("hidden tab and popup: {}", host_line(&stats));
    assert_eq!(stats.failure(), None, "nothing-stuck");
    assert_eq!(stats.stuck(), 0, "nothing-stuck");
    assert!(stats.peak_lead_ns() < MS, "nothing-stuck: {stats:?}");
    assert_eq!(ticks_in(&ticks.0, f, 200.0, 4.0), 50, "all-metered: first");
    assert_eq!(ticks_in(&ticks.1, s, 200.0, 4.0), 50, "all-metered: second");
    assert_eq!(ticks_in(&ticks.2, p, 200.0, 4.0), 50, "all-metered: popup");
    assert!(stats.shared() > 0, "shared-once: {stats:?}");
    finish(system, actor);
}

/// Given a page that opens a window without an opener (`noopener`, as a link
/// with target=_blank does), whose renderer does not exist until Chrome lets
/// it run:
/// - the node holds the new page, its shim in before its first script
///   (`held-from-birth`)
/// - the run goes on with nothing stuck, the new page's 4 ms timer ticking
///   every 4 ms of its clock (`metered`)
///
/// Not a declared behaviour: the case is `#[ignore]`d where the ledger is
/// collected (TESTING.md).
#[test]
#[ignore = "launches the host's Chrome; CI's chrome-cdp job runs it"]
fn a_window_opened_without_an_opener_is_held_from_birth() {
    let _suite = suite_lock();
    let server = Server::start(&pages());
    let url = format!("{}ticks.html?opener", server.url);
    let node = node(&url, 115_200);
    let stats = node.stats();
    let (peer, _peer) = Peer::new(115_200, false);
    let (system, actor) = bench(node, Box::new(peer));
    let mut page = page_up(&url, &stats);
    page.eval_gesture(
        "setTimeout(() => { window.open('ticks.html?noopener', '_blank', 'noopener'); }, 0), true",
    );
    wait_for(&stats, "the new page held", false, || {
        stats.pages_held() >= 2
    });
    let endpoint = stats.devtools_endpoint().unwrap();
    let opened = format!("{}ticks.html?noopener", server.url);
    let mut harness = Driver::connect(&endpoint);
    let target = loop {
        let targets = harness
            .call(None, "Target.getTargets", json!({}))
            .expect("the targets");
        let found = targets["targetInfos"].as_array().and_then(|list| {
            list.iter()
                .find(|t| t["type"] == "page" && t["url"] == opened.as_str())
                .and_then(|t| t["targetId"].as_str().map(str::to_string))
        });
        if let Some(found) = found {
            break found;
        }
        assert!(stats.failure().is_none(), "{stats:?}");
        virtual_clock::wait_virtual_ns(MS);
    };
    let attached = harness
        .call(
            None,
            "Target.attachToTarget",
            json!({ "targetId": target, "flatten": true }),
        )
        .expect("attaches");
    let session = attached["sessionId"].as_str().unwrap().to_string();
    wait_for(&stats, "the new page's ready flag", false, || {
        harness.try_eval(&session, "window.ready === true") == Ok(Value::Bool(true))
    });
    assert_eq!(
        harness.eval(&session, "window.firstSawShim"),
        Value::Bool(true),
        "held-from-birth"
    );
    virtual_clock::wait_virtual_ns(20 * MS);
    let from = harness
        .eval(&session, "performance.now()")
        .as_f64()
        .unwrap();
    virtual_clock::wait_virtual_ns(106 * MS);
    let ticks = f64s(&harness.eval(&session, "window.ticks"));
    assert_eq!(ticks_in(&ticks, from, 100.0, 4.0), 25, "metered: {ticks:?}");
    assert_eq!(stats.failure(), None, "metered");
    finish(system, actor);
}

/// Given a harness that makes a page with a URL (Target.createTarget with a
/// url, as /json/new does), which Chrome does not hold:
/// - the run stops, naming the page and saying to make it at about:blank and
///   navigate it (`refused`)
///
/// Not a declared behaviour: the case is `#[ignore]`d where the ledger is
/// collected (TESTING.md).
#[test]
#[ignore = "launches the host's Chrome; CI's chrome-cdp job runs it"]
fn a_page_made_with_a_url_stops_the_run_saying_how_to_make_one() {
    let _suite = suite_lock();
    let server = Server::start(&pages());
    let url = format!("{}ticks.html?main", server.url);
    let node = node(&url, 115_200);
    let stats = node.stats();
    let (peer, _peer) = Peer::new(115_200, false);
    let (system, actor) = bench(node, Box::new(peer));
    let _main = page_up(&url, &stats);
    let mut harness = Driver::connect(&stats.devtools_endpoint().unwrap());
    let context = harness.new_context();
    let born = format!("{}ticks.html?born", server.url);
    let _ = harness.new_page(json!({ "url": born, "browserContextId": context }));
    // Made while the board is held: the page loads and runs its scripts on
    // host time before the node's next slice can set it up.
    std::thread::sleep(Duration::from_secs(1));
    wait_for(&stats, "the failure", true, || stats.failure().is_some());
    let failure = stats.failure().unwrap();
    assert!(
        failure.starts_with(&format!(
            "a page at {born} ran its own scripts before the node held it"
        )) && failure.contains("make the page at about:blank and navigate it"),
        "refused: {failure}"
    );
    finish(system, actor);
}

/// Given a page whose dedicated worker starts a worker of its own, which runs
/// a 4 ms timer and asks for its own navigator.serial:
/// - the node holds both workers (`both-held`)
/// - while the board is held for 300 ms of host time, the inner worker's timer
///   does not tick (`frozen-while-held`)
/// - over 100 ms of board time it ticks every 4 ms of its clock (`metered`)
/// - the report counts one worker that asked for its own navigator.serial,
///   which is Chrome's (`serial-reported`)
///
/// Not a declared behaviour: the case is `#[ignore]`d where the ledger is
/// collected (TESTING.md).
#[test]
#[ignore = "launches the host's Chrome; CI's chrome-cdp job runs it"]
fn a_worker_a_worker_starts_is_held_and_metered() {
    let _suite = suite_lock();
    let server = Server::start(&pages());
    let url = format!("{}ticks.html?nested", server.url);
    let node = node(&url, 115_200);
    let stats = node.stats();
    let (peer, _peer) = Peer::new(115_200, false);
    let (system, actor) = bench(node, Box::new(peer));
    let mut page = page_up(&url, &stats);
    until(&mut page, &stats, "the inner worker's ticks", |page| {
        page.try_eval("window.worker.ticks.length > 2") == Ok(Value::Bool(true))
    });
    virtual_clock::wait_virtual_ns(20 * MS);
    assert_eq!(stats.workers_held(), 2, "both-held: {stats:?}");
    let held = page.eval("window.worker.ticks.length");
    std::thread::sleep(Duration::from_millis(300));
    assert_eq!(
        page.eval("window.worker.ticks.length"),
        held,
        "frozen-while-held"
    );
    let from = page
        .eval("performance.timeOrigin + performance.now()")
        .as_f64()
        .unwrap();
    virtual_clock::wait_virtual_ns(112 * MS);
    let ticks = f64s(&page.eval("window.worker.ticks"));
    assert_eq!(ticks_in(&ticks, from, 100.0, 4.0), 25, "metered: {ticks:?}");
    assert_eq!(
        page.eval("window.worker.serial"),
        Value::String("object".into())
    );
    assert_eq!(stats.workers_with_serial(), 1, "serial-reported: {stats:?}");
    assert_eq!(stats.failure(), None);
    finish(system, actor);
}

/// Given a page whose renderer crashes (Page.crash) a few slices in, the node
/// allowed to wait 30 s of host time for a grant:
/// - the run stops saying the page crashed, naming it (`crash-named`)
///
/// Not a declared behaviour: the case is `#[ignore]`d where the ledger is
/// collected (TESTING.md).
#[test]
#[ignore = "launches the host's Chrome; CI's chrome-cdp job runs it"]
fn a_page_that_crashes_stops_the_run_naming_it() {
    let _suite = suite_lock();
    let server = Server::start(&pages());
    let url = format!("{}ticks.html?main", server.url);
    let node = node(&url, 115_200);
    let stats = node.stats();
    let (peer, _peer) = Peer::new(115_200, false);
    let (system, actor) = bench(node, Box::new(peer));
    let mut page = page_up(&url, &stats);
    virtual_clock::wait_virtual_ns(5 * MS);
    page.send("Page.crash", json!({}));
    let wall = Instant::now();
    wait_for(&stats, "the failure", true, || stats.failure().is_some());
    eprintln!(
        "crash: the run stopped {:.3} s of host time after Page.crash",
        wall.elapsed().as_secs_f64()
    );
    assert_eq!(
        stats.failure().unwrap(),
        format!("a page crashed at {url}: its renderer process went away mid-run"),
        "crash-named"
    );
    finish(system, actor);
}

/// Given the browser itself crashing (Browser.crash) a few slices in:
/// - the run stops saying Chrome exited mid-run (`exit-named`)
///
/// Not a declared behaviour: the case is `#[ignore]`d where the ledger is
/// collected (TESTING.md).
#[test]
#[ignore = "launches the host's Chrome; CI's chrome-cdp job runs it"]
fn a_browser_that_exits_stops_the_run_saying_so() {
    let _suite = suite_lock();
    let server = Server::start(&pages());
    let url = format!("{}ticks.html?main", server.url);
    let node = node(&url, 115_200);
    let stats = node.stats();
    let (peer, _peer) = Peer::new(115_200, false);
    let (system, actor) = bench(node, Box::new(peer));
    let mut page = page_up(&url, &stats);
    virtual_clock::wait_virtual_ns(5 * MS);
    let _ = page.browser_call("Browser.crash", json!({}));
    wait_for(&stats, "the failure", true, || stats.failure().is_some());
    let failure = stats.failure().unwrap();
    assert!(
        failure.starts_with("Chrome exited mid-run"),
        "exit-named: {failure}"
    );
    finish(system, actor);
}

/// Given a page that opens a confirm dialog asking "sure?" from a timer, which
/// nothing answers, the node allowed to wait 3 s of host time:
/// - the run stops saying the page's main thread did not return, naming the
///   dialog (`dialog-named`)
///
/// Not a declared behaviour: the case is `#[ignore]`d where the ledger is
/// collected (TESTING.md).
#[test]
#[ignore = "launches the host's Chrome; CI's chrome-cdp job runs it"]
fn a_confirm_dialog_in_chrome_nobody_answers_stops_the_run_naming_it() {
    let _suite = suite_lock();
    let server = Server::start(&pages());
    let url = format!("{}ticks.html?main", server.url);
    let node = node_with(&url, 115_200, |s| s.stuck_after = Duration::from_secs(3));
    let stats = node.stats();
    let (peer, _peer) = Peer::new(115_200, false);
    let (system, actor) = bench(node, Box::new(peer));
    let mut page = page_up(&url, &stats);
    page.eval("setTimeout(() => { window.answer = confirm('sure?'); }, 1), true");
    wait_for(&stats, "the failure", true, || stats.failure().is_some());
    let failure = stats.failure().unwrap();
    eprintln!("dialog: {failure}");
    assert!(
        failure.contains("it shows a confirm dialog (\"sure?\") that nobody answered"),
        "dialog-named: {failure}"
    );
    finish(system, actor);
}

// ============================================================
// The DevTools port
// ============================================================

/// Given the node launching Chrome on a DevTools port the project names (a
/// free one, as MaD's cosim files name 9222), where Chrome writes no
/// DevToolsActivePort file to its profile:
/// - the node reaches Chrome on that port: its DevTools endpoint is
///   127.0.0.1 at the named port (`on-the-named-port`)
/// - a harness attached there finds the page held from birth and metered,
///   its 4 ms timer ticking 25 times in 100 ms of board time (`metered`)
///
/// Not a declared behaviour: the case is `#[ignore]`d where the ledger is
/// collected (TESTING.md).
#[test]
#[ignore = "launches the host's Chrome; CI's chrome-cdp job runs it"]
fn a_devtools_port_the_project_names_is_where_the_node_reaches_chrome() {
    let _suite = suite_lock();
    let server = Server::start(&pages());
    let url = format!("{}ticks.html?main", server.url);
    // Free now; the node checks it again just before it launches Chrome.
    let port = std::net::TcpListener::bind(("127.0.0.1", 0))
        .and_then(|listener| listener.local_addr())
        .expect("a free port")
        .port();
    let node = node_with(&url, 115_200, |s| {
        if let Browse::Launch(spec) = &mut s.browse {
            spec.port = port;
        }
    });
    let stats = node.stats();
    let (peer, _peer) = Peer::new(115_200, false);
    let (system, actor) = bench(node, Box::new(peer));
    let wall = Instant::now();
    let mut page = page_up(&url, &stats);
    eprintln!(
        "named port {port}: the page up {:.2} s of host time after the bench started",
        wall.elapsed().as_secs_f64()
    );
    assert_eq!(
        stats.devtools_endpoint(),
        Some(format!("http://127.0.0.1:{port}")),
        "on-the-named-port"
    );
    assert_eq!(
        page.eval("window.firstSawShim"),
        Value::Bool(true),
        "metered: the shim before the page's first script"
    );
    // The timer starts as the page loads; its first period can be short.
    virtual_clock::wait_virtual_ns(20 * MS);
    let from = page.eval("performance.now()").as_f64().unwrap();
    virtual_clock::wait_virtual_ns(106 * MS);
    let ticks = f64s(&page.eval("window.ticks"));
    assert_eq!(ticks_in(&ticks, from, 100.0, 4.0), 25, "metered: {ticks:?}");
    assert_eq!(stats.failure(), None);
    finish(system, actor);
}

// ============================================================
// The port
// ============================================================

/// Given a page that opens its granted port on the main thread at 2 Mbaud and
/// transfers both streams to a dedicated worker, the worker writing a 4096-byte
/// pattern, and a peer on the board's side of the line that echoes every byte
/// it hears:
/// - the peer hears exactly the 4096 bytes, in order, with no framing error
///   (`board-hears-the-pattern`)
/// - the worker reads exactly the 4096 bytes back, in order, through the
///   transferred readable, and the node counts 4096 bytes each way with none
///   shed (`page-reads-them-back`)
/// - the port's getInfo() reports the project's adapter, vendor 0x0403 and
///   product 0x6001 (`port-info`)
///
/// Not a declared behaviour: the case is `#[ignore]`d where the ledger is
/// collected (TESTING.md).
#[test]
#[ignore = "launches the host's Chrome; CI's chrome-cdp job runs it"]
fn bytes_round_trip_through_the_shim_and_a_worker_that_owns_the_streams() {
    let _suite = suite_lock();
    let server = Server::start(&pages());
    let url = format!("{}serial.html", server.url);
    let node = node(&url, 2_000_000);
    let stats = node.stats();
    let (peer, heard) = Peer::new(2_000_000, true);
    let (system, actor) = bench(node, Box::new(peer));
    let mut page = page_up(&url, &stats);

    run(&mut page, &stats, "connect", "window.t.connect('echo')");
    let info = run(
        &mut page,
        &stats,
        "info",
        "navigator.serial.getPorts().then(([p]) => p.getInfo())",
    );
    assert_eq!(info["usbVendorId"], 0x0403, "port-info: {info}");
    assert_eq!(info["usbProductId"], 0x6001, "port-info: {info}");
    let pattern: Vec<u8> = (0..4096u32).map(|i| ((i * 7 + 3) & 0xff) as u8).collect();
    until(&mut page, &stats, "the echo", |_| {
        heard.heard().len() >= pattern.len() && stats.to_page() >= pattern.len() as u64
    });
    let worker = run(&mut page, &stats, "final", "window.t.stats()");
    eprintln!("round trip: {}", host_line(&stats));
    assert_eq!(heard.heard(), pattern, "board-hears-the-pattern");
    assert_eq!(
        heard.0.framing_errors.load(Ordering::Relaxed),
        0,
        "board-hears-the-pattern"
    );
    assert_eq!(worker["echoed"], 4096, "page-reads-them-back: {worker}");
    assert_eq!(worker["echoOk"], true, "page-reads-them-back: {worker}");
    assert_eq!(stats.from_page(), 4096, "page-reads-them-back: {stats:?}");
    assert_eq!(stats.to_page(), 4096, "page-reads-them-back: {stats:?}");
    assert_eq!(stats.shed(), 0, "page-reads-them-back: {stats:?}");
    assert_eq!(stats.failure(), None);
    finish(system, actor);
}

/// Given a page that opens its port at 2 Mbaud (a 255-byte buffer) and writes
/// 64 KiB in one write, a peer on the board's side recording what it hears:
/// - the peer hears all 65536 bytes, in order, none shed (`arrives-whole`)
/// - the write resolves only as the line takes the bytes: no sooner, by the
///   page's clock, than the line needs for all but two of the writer's windows
///   of them (`paced-by-the-line`)
///
/// Not a declared behaviour: the case is `#[ignore]`d where the ledger is
/// collected (TESTING.md).
#[test]
#[ignore = "launches the host's Chrome; CI's chrome-cdp job runs it"]
fn a_write_larger_than_the_writers_window_arrives_whole() {
    let _suite = suite_lock();
    let server = Server::start(&pages());
    let url = format!("{}serial.html?baud=2000000", server.url);
    let node = node(&url, 2_000_000);
    let stats = node.stats();
    let (peer, heard) = Peer::new(2_000_000, false);
    let (system, actor) = bench(node, Box::new(peer));
    let mut page = page_up(&url, &stats);
    let n = 65_536usize;
    let out = run(
        &mut page,
        &stats,
        "write",
        &format!("window.t.bigWrite({n})"),
    );
    wait_for(&stats, "the last byte", false, || heard.heard().len() >= n);
    let expected: Vec<u8> = (0..n).map(|i| ((i * 31 + 7) & 0xff) as u8).collect();
    assert_eq!(heard.heard(), expected, "arrives-whole");
    assert_eq!(stats.shed(), 0, "arrives-whole: {stats:?}");
    // 2 Mbaud 8N1 carries 200 bytes a millisecond; the window is two
    // quanta of them, 400 bytes.
    let took = out["resolved"].as_f64().unwrap() - out["began"].as_f64().unwrap();
    let floor = (n as f64 - 2.0 * 400.0) / 200.0;
    eprintln!(
        "big write: resolved after {took:.3} ms of page time; {}",
        host_line(&stats)
    );
    assert!(took >= floor, "paced-by-the-line: resolved after {took} ms");
    finish(system, actor);
}

/// Given a page that opens its port, takes a reader on its readable, and calls
/// close():
/// - close() rejects with a TypeError saying it cannot cancel a locked stream,
///   as Chrome's does (serial_port.cc, readable_stream.cc) (`close-refused`)
/// - the port stays open: open() then rejects with InvalidStateError, the port
///   is already open (`still-open`)
/// - once the reader releases its lock, close() resolves and the port has no
///   readable (`closes-once-released`)
/// - requestPort() with no user gesture rejects with a SecurityError
///   (`gesture-required`)
///
/// Not a declared behaviour: the case is `#[ignore]`d where the ledger is
/// collected (TESTING.md).
#[test]
#[ignore = "launches the host's Chrome; CI's chrome-cdp job runs it"]
fn a_close_while_a_stream_is_locked_is_refused_and_the_port_stays_open() {
    let _suite = suite_lock();
    let server = Server::start(&pages());
    let url = format!("{}serial.html", server.url);
    let node = node(&url, 115_200);
    let stats = node.stats();
    let (peer, _heard) = Peer::new(115_200, false);
    let (system, actor) = bench(node, Box::new(peer));
    let mut page = page_up(&url, &stats);

    let out = run(&mut page, &stats, "close", "window.t.closeWhileLocked()");
    assert_eq!(out["close"]["name"], "TypeError", "close-refused: {out}");
    assert!(
        out["close"]["message"]
            .as_str()
            .unwrap_or("")
            .ends_with("Cannot cancel a locked stream"),
        "close-refused: {out}"
    );
    assert_eq!(
        out["reopen"]["name"], "InvalidStateError",
        "still-open: {out}"
    );
    assert!(
        out["reopen"]["message"]
            .as_str()
            .unwrap_or("")
            .ends_with("The port is already open."),
        "still-open: {out}"
    );
    assert_eq!(out["closeAfter"], "resolved", "closes-once-released: {out}");
    assert_eq!(out["readable"], Value::Null, "closes-once-released: {out}");
    let gesture = run(
        &mut page,
        &stats,
        "gesture",
        "window.t.requestWithoutGesture()",
    );
    assert_eq!(
        gesture["name"], "SecurityError",
        "gesture-required: {gesture}"
    );
    assert_eq!(stats.failure(), None);
    finish(system, actor);
}

/// Given a page with its port open and a read pending, the cable pulled from
/// the page with __embsim.link('unplug') and put back with
/// __embsim.link('plug'):
/// - the pending read rejects with a NetworkError, the device has been lost
///   (`read-rejects`)
/// - the page sees disconnect for its port, then connect for a new one
///   (`events`)
/// - getPorts() then hands out a different SerialPort object, which opens; the
///   old one says it is not connected and its open() rejects with a
///   NetworkError (`new-port`)
///
/// Not a declared behaviour: the case is `#[ignore]`d where the ledger is
/// collected (TESTING.md).
#[test]
#[ignore = "launches the host's Chrome; CI's chrome-cdp job runs it"]
fn a_replug_hands_out_a_new_port_and_the_old_one_is_dead() {
    let _suite = suite_lock();
    let server = Server::start(&pages());
    let url = format!("{}serial.html", server.url);
    let node = node(&url, 115_200);
    let stats = node.stats();
    let (peer, _heard) = Peer::new(115_200, false);
    let (system, actor) = bench(node, Box::new(peer));
    let mut page = page_up(&url, &stats);

    let a = run(&mut page, &stats, "open", "window.t.openAndRead()");
    page.eval("window.__embsim.link('unplug'), true");
    until(&mut page, &stats, "the unplug", |_| stats.unplugs() == 1);
    virtual_clock::wait_virtual_ns(5 * MS);
    page.eval("window.__embsim.link('plug'), true");
    until(&mut page, &stats, "the plug", |_| stats.plugs() == 1);
    let out = run(&mut page, &stats, "after", "window.t.afterReplug()");
    assert_eq!(out["read"]["name"], "NetworkError", "read-rejects: {out}");
    assert_eq!(
        out["read"]["message"], "The device has been lost.",
        "read-rejects: {out}"
    );
    let events: Vec<(String, u64)> = out["events"]
        .as_array()
        .unwrap()
        .iter()
        .map(|e| {
            (
                e["type"].as_str().unwrap().to_string(),
                e["port"].as_u64().unwrap(),
            )
        })
        .collect();
    let b = out["b"]
        .as_u64()
        .expect("new-port: a port after the replug");
    assert_eq!(
        events,
        [
            ("disconnect".to_string(), a.as_u64().unwrap()),
            ("connect".to_string(), b)
        ],
        "events: {out}"
    );
    assert_eq!(out["same"], false, "new-port: {out}");
    assert_eq!(out["aConnected"], false, "new-port: {out}");
    assert_eq!(out["reopenA"]["name"], "NetworkError", "new-port: {out}");
    assert_eq!(out["openB"], "resolved", "new-port: {out}");
    assert_eq!(stats.failure(), None);
    finish(system, actor);
}

/// Given a page with its port open and its reader taken, nothing read yet, the
/// board's side sending three bytes, then the cable pulled:
/// - the reader still reads the three bytes the port's pipe held, in order,
///   and then rejects with a NetworkError, the device has been lost, as
///   Chrome's SignalErrorOnClose does (`pipe-read-first`)
///
/// Not a declared behaviour: the case is `#[ignore]`d where the ledger is
/// collected (TESTING.md).
#[test]
#[ignore = "launches the host's Chrome; CI's chrome-cdp job runs it"]
fn bytes_in_the_pipe_are_read_before_an_unplug_errors_the_stream() {
    let _suite = suite_lock();
    let server = Server::start(&pages());
    let url = format!("{}serial.html?baud=115200", server.url);
    let node = node(&url, 115_200);
    let stats = node.stats();
    let (peer, heard) = Peer::new(115_200, false);
    let (system, actor) = bench(node, Box::new(peer));
    let mut page = page_up(&url, &stats);
    run(&mut page, &stats, "hold", "window.t.openAndHold()");
    heard.send(b"abc");
    until(&mut page, &stats, "the bytes handed over", |_| {
        stats.to_page() >= 3
    });
    page.eval("window.__embsim.link('unplug'), true");
    until(&mut page, &stats, "the unplug", |_| stats.unplugs() == 1);
    virtual_clock::wait_virtual_ns(3 * MS);
    let out = run(&mut page, &stats, "read", "window.t.readAll()");
    assert_eq!(out["bytes"], json!([97, 98, 99]), "pipe-read-first: {out}");
    assert_eq!(
        out["error"]["name"], "NetworkError",
        "pipe-read-first: {out}"
    );
    assert_eq!(
        out["error"]["message"], "The device has been lost.",
        "pipe-read-first: {out}"
    );
    finish(system, actor);
}

/// Given a page the port is granted to (by the scenario's policy) that never
/// asked for it, the cable pulled and put back:
/// - navigator.serial fires disconnect, then connect, as Chrome does for any
///   port the origin may use (`events-unasked`)
///
/// Not a declared behaviour: the case is `#[ignore]`d where the ledger is
/// collected (TESTING.md).
#[test]
#[ignore = "launches the host's Chrome; CI's chrome-cdp job runs it"]
fn a_page_that_never_asked_for_the_port_hears_it_go_and_come_back() {
    let _suite = suite_lock();
    let server = Server::start(&pages());
    let url = format!("{}ticks.html?main", server.url);
    let node = node(&url, 115_200);
    let stats = node.stats();
    let (peer, _heard) = Peer::new(115_200, false);
    let (system, actor) = bench(node, Box::new(peer));
    let mut page = page_up(&url, &stats);
    page.eval("window.__embsim.link('unplug'), true");
    until(&mut page, &stats, "the unplug", |_| stats.unplugs() == 1);
    virtual_clock::wait_virtual_ns(3 * MS);
    page.eval("window.__embsim.link('plug'), true");
    until(&mut page, &stats, "the plug", |_| stats.plugs() == 1);
    virtual_clock::wait_virtual_ns(3 * MS);
    assert_eq!(
        page.eval("window.events"),
        json!(["disconnect", "connect"]),
        "events-unasked"
    );
    finish(system, actor);
}

// ============================================================
// The example project
// ============================================================

/// Given `examples/chrome-ping` — a page that writes "ping" every 100 ms of
/// its own clock from the moment its port opens, its line wired to a
/// host-serial on one 3.3 V rail — loaded as a project file and built with the
/// kind, run for one second of board time:
/// - the page sent nine pings, 45 bytes, onto the line (`nine-pings`)
/// - the page lived exactly the board's time from its first slice
///   (`level-with-the-board`)
///
/// Not a declared behaviour: the case is `#[ignore]`d where the ledger is
/// collected (TESTING.md).
#[test]
#[ignore = "launches the host's Chrome; CI's chrome-cdp job runs it"]
fn the_example_project_hears_nine_pings_in_a_second_of_board_time() {
    let _suite = suite_lock();
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples/chrome-ping");
    let mut project =
        embsim_board::Project::load(dir.join("project.toml")).expect("the example loads");
    let pty = std::env::temp_dir().join(format!("embsim-cdp-ping-{}.pty", std::process::id()));
    project
        .set_component_option("HOST", "path", pty.to_string_lossy().to_string())
        .expect("the example has a HOST");
    let mut set = embsim_boards::catalog::CatalogSet::new();
    embsim_cdp::catalog::register(&mut set).expect("the kind registers");
    virtual_clock::init_mode(virtual_clock::ClockMode::Stepped, 1_000_000);
    let reports = embsim_board::Reports::new();
    let system = project
        .instantiate_with(&set, &reports)
        .expect("the example builds")
        .hold_time()
        .start()
        .expect("the example starts");
    let mut reports = reports.take();
    let actor = virtual_clock::register_actor("cdp-example-case");
    system.release_time();
    let wall = Instant::now();
    virtual_clock::wait_virtual_ns(1_000 * MS);
    let report = reports
        .iter_mut()
        .find(|report| report.subject() == "PC")
        .expect("the component reports");
    let summary = report.summary();
    eprintln!(
        "example: 1 s of board time in {:.3} s of host time; {summary:#?}",
        wall.elapsed().as_secs_f64()
    );
    assert!(
        summary
            .iter()
            .any(|line| line == "45 bytes from the page, 0 to it, 0 framing errors"),
        "nine-pings: {summary:?}"
    );
    let lived = summary[0]
        .split("the page lived ")
        .nth(1)
        .expect("level-with-the-board: the summary says what the page lived");
    let (page, board) = lived
        .split_once(" of the board's ")
        .expect("level-with-the-board");
    assert_eq!(page, board, "level-with-the-board: {}", summary[0]);
    assert_eq!(report.failure(), None);
    drop(actor);
    system.shutdown();
}

// ============================================================
// The proof: a page shaped like MaD Control against a board
// ============================================================

/// `acc * 31 + byte`, wrapping: the WASM module's `mix`.
fn mix(acc: u32, byte: u8) -> u32 {
    acc.wrapping_mul(31).wrapping_add(u32::from(byte))
}

/// What the stand-in board did.
#[derive(Default)]
struct BoardLog {
    samples: u64,
    requests: u64,
    replies: u64,
    /// The mix of every byte sent, in order.
    sum: u32,
    sent: u64,
    bulk_left: usize,
    bulk_sum: u32,
    bulk_sent: u64,
    streaming: bool,
    heard: Vec<u8>,
    due: std::collections::BTreeMap<u64, Vec<u8>>,
    next_sample_ns: u64,
    refill_at_ns: u64,
    io: Option<embsim_board::ComponentNetIo>,
}

/// A stand-in for MaD's firmware on the board's side of the line, in the
/// framing the test page's worker parses: a sample frame every 10 ms of
/// board time, and a reply 2 ms after each request it hears. Then, asked,
/// a 256 KiB bulk stream as fast as the line carries it.
struct Board {
    framing: embsim_board::uart::UartFraming,
    log: Arc<std::sync::Mutex<BoardLog>>,
    shutdown: Arc<std::sync::atomic::AtomicBool>,
    io: Option<embsim_board::ComponentNetIo>,
}

const SAMPLE_EVERY_NS: u64 = 10 * MS;
const REPLY_AFTER_NS: u64 = 2 * MS;

impl Board {
    fn send(line: &embsim_board::HostRailLine, log: &mut BoardLog, frame: &[u8], bulk: bool) {
        let shed = line.bridge().transmit(frame);
        assert_eq!(shed, 0, "the board's line shed bytes");
        for &b in frame {
            if bulk {
                log.bulk_sum = mix(log.bulk_sum, b);
            } else {
                log.sum = mix(log.sum, b);
            }
        }
        if bulk {
            log.bulk_sent += frame.len() as u64;
        } else {
            log.sent += frame.len() as u64;
        }
    }

    fn wake(
        line: &embsim_board::HostRailLine,
        io: &embsim_board::ComponentNetIo,
        log: &std::sync::Mutex<BoardLog>,
        now: u64,
        frames: Vec<Result<u8, embsim_board::uart::FramingError>>,
    ) {
        let mut log = log.lock().unwrap();
        for byte in frames.into_iter().flatten() {
            log.heard.push(byte);
        }
        // Requests: 0x55 0x00 seq.
        while let Some(at) = log.heard.iter().position(|&b| b == 0x55) {
            if log.heard.len() < at + 3 {
                break;
            }
            let frame: Vec<u8> = log.heard.drain(..at + 3).skip(at).collect();
            if frame[1] == 0x00 {
                log.requests += 1;
                let reply = vec![0x55, 0x01, frame[2], 1, 2, 3, 4, 5];
                log.due
                    .entry(now + REPLY_AFTER_NS)
                    .or_default()
                    .extend(reply);
                io.schedule_at_ns(now + REPLY_AFTER_NS);
            }
        }
        if !line.rail_known() {
            return;
        }
        let ready: Vec<u64> = log.due.range(..=now).map(|(&at, _)| at).collect();
        for at in ready {
            let bytes = log.due.remove(&at).unwrap();
            log.replies += (bytes.len() / 8) as u64;
            Self::send(line, &mut log, &bytes, false);
        }
        if log.streaming && now >= log.next_sample_ns {
            let seq = (log.samples & 0xff) as u8;
            let mut frame = vec![0x55, 0x02, seq];
            frame.extend((0..16u8).map(|k| seq.wrapping_mul(13).wrapping_add(k)));
            let x = frame.iter().fold(0u8, |x, &b| x ^ b);
            frame.push(x);
            log.samples += 1;
            Self::send(line, &mut log, &frame, false);
            log.next_sample_ns = now + SAMPLE_EVERY_NS;
            io.schedule_at_ns(log.next_sample_ns);
        }
        if log.bulk_left > 0 {
            let room = line.bridge().tx_room().min(log.bulk_left);
            if room > 0 {
                let start = 262_144 - log.bulk_left;
                let chunk: Vec<u8> = (start..start + room)
                    .map(|i| (i as u32).wrapping_mul(2_654_435_761).to_le_bytes()[3])
                    .collect();
                log.bulk_left -= room;
                Self::send(line, &mut log, &chunk, true);
            }
            // One refill a tenth of a millisecond while bytes are left:
            // never one a bit, which would arm a wake at every edge.
            if log.bulk_left > 0 && log.refill_at_ns <= now {
                log.refill_at_ns = now + 100_000;
                io.schedule_at_ns(log.refill_at_ns);
            }
        }
    }
}

impl embsim_board::Component for Board {
    fn pins(&self) -> &[embsim_board::PinDecl] {
        &embsim_board::HOST_RAIL_PINS
    }

    fn attach(
        &mut self,
        io: embsim_board::ComponentNetIo,
    ) -> Result<(), embsim_board::AttachError> {
        let line =
            embsim_board::HostRailLine::attach(&io, self.framing, Arc::clone(&self.shutdown))?;
        {
            let (line, io2, log) = (line.clone(), io.clone(), Arc::clone(&self.log));
            let rx = io.pin("RX")?;
            io.on_sense("RX", move |sense| {
                let at = sense.at_ns;
                Board::wake(
                    &line,
                    &io2,
                    &log,
                    at,
                    line.bridge().receive_sense(&rx, &sense),
                );
            })?;
        }
        {
            let (io2, log) = (io.clone(), Arc::clone(&self.log));
            io.on_wake_ns(move |now| {
                Board::wake(&line, &io2, &log, now, line.bridge().service(now))
            });
        }
        self.log.lock().unwrap().io = Some(io.clone());
        self.io = Some(io);
        Ok(())
    }
}

/// Given a page shaped like MaD Control — the port opened on the main thread at
/// 2 Mbaud, both streams transferred to a dedicated worker that owns them, a
/// WASM module there checksumming the board's bytes, Comlink-style calls
/// between the two — against a stand-in board that streams a 20-byte sample
/// every 10 ms of board time and answers each 3-byte request 2 ms after it
/// hears it, the worker asking every 25 ms of its own time with a 200 ms
/// timeout; two seconds of that, then 256 KiB from the board as fast as the
/// line carries it:
/// - every sample the board sent arrives whole: the worker counts the board's
///   samples, none corrupt, and its checksum over every byte equals the board's
///   (`samples-exact`)
/// - every request the worker sent is heard and answered, no timeout fires, and
///   each answer arrives 2 ms of the worker's time after its request plus at
///   most 3 quanta (`requests-answered-in-time`)
/// - the 256 KiB arrive whole and in order: the worker's count and checksum
///   equal the board's (`bulk-exact`)
/// - the page's clock ends within a quantum of the board's, and no grant stuck
///   (`level-with-the-board`)
///
/// Not a declared behaviour: the case is `#[ignore]`d where the ledger is
/// collected (TESTING.md).
#[test]
#[ignore = "launches the host's Chrome; CI's chrome-cdp job runs it"]
fn a_page_shaped_like_mad_control_trades_with_a_board_on_the_boards_time() {
    let _suite = suite_lock();
    let server = Server::start(&pages());
    let url = format!("{}serial.html", server.url);
    let node = node(&url, 2_000_000);
    let stats = node.stats();
    let log = Arc::new(std::sync::Mutex::new(BoardLog::default()));
    let board = Board {
        framing: embsim_board::uart::UartFraming::new_8n1(2_000_000),
        log: Arc::clone(&log),
        shutdown: Arc::new(std::sync::atomic::AtomicBool::new(false)),
        io: None,
    };
    let (system, actor) = bench(node, Box::new(board));
    let mut page = page_up(&url, &stats);
    run(&mut page, &stats, "connect", "window.t.connect('mad')");

    // Two seconds of samples and requests.
    let wall = Instant::now();
    let slices_before = stats.slices();
    {
        let mut log = log.lock().unwrap();
        log.streaming = true;
        log.next_sample_ns = virtual_clock::virtual_ns() + MS;
        let io = log.io.clone().expect("the board is attached");
        io.schedule_at_ns(log.next_sample_ns);
    }
    virtual_clock::wait_virtual_ns(2_000 * MS);
    // Quiet: the board stops sampling, the worker stops asking, and what
    // is in flight lands (an unanswered request times out in 200 ms).
    log.lock().unwrap().streaming = false;
    run(&mut page, &stats, "quiet", "window.t.quiet()");
    let mad_wall = wall.elapsed();
    let mad_slices = stats.slices() - slices_before;
    virtual_clock::wait_virtual_ns(300 * MS);
    let worker = run(&mut page, &stats, "stats", "window.t.stats()");
    let (samples, requests, replies, sum, sent) = {
        let board = log.lock().unwrap();
        (
            board.samples,
            board.requests,
            board.replies,
            board.sum,
            board.sent,
        )
    };
    eprintln!(
        "mad shape: {mad_slices} slices in {:.3} s of host time ({:.2} ms a slice overall); {}",
        mad_wall.as_secs_f64(),
        mad_wall.as_secs_f64() * 1e3 / mad_slices as f64,
        host_line(&stats)
    );
    eprintln!(
        "mad shape: the board sent {samples} samples and {sent} bytes, heard {requests} \
         requests, sent {replies} replies; the worker: {worker}"
    );
    assert_eq!(samples, 200, "samples-exact: the board sent {samples}");
    assert_eq!(worker["samples"], samples, "samples-exact: {worker}");
    assert_eq!(worker["bad"], 0, "samples-exact: {worker}");
    assert_eq!(worker["bytes"], sent, "samples-exact: {worker}");
    assert_eq!(worker["sum"], sum, "samples-exact: {worker}");
    assert!(
        requests >= 79,
        "requests-answered-in-time: the board heard {requests}"
    );
    assert_eq!(
        worker["requests"], requests,
        "requests-answered-in-time: {worker}"
    );
    assert_eq!(replies, requests, "requests-answered-in-time");
    assert_eq!(
        worker["replies"], requests,
        "requests-answered-in-time: {worker}"
    );
    assert_eq!(worker["timeouts"], 0, "requests-answered-in-time: {worker}");
    let max = worker["latency"]["max"].as_f64().unwrap();
    let min_ms = REPLY_AFTER_NS as f64 / 1e6;
    assert!(
        max <= min_ms + 3.0 + 0.01 && worker["latency"]["p50"].as_f64().unwrap() >= min_ms,
        "requests-answered-in-time: {worker}"
    );

    // 256 KiB from the board as fast as the line carries it.
    run(&mut page, &stats, "bulk", "window.t.bulk()");
    let wall = Instant::now();
    let slices_before = stats.slices();
    let to_page_before = stats.to_page();
    let io = {
        let mut board = log.lock().unwrap();
        board.bulk_left = 262_144;
        board.io.clone().expect("the board is attached")
    };
    io.schedule_at_ns(virtual_clock::virtual_ns());
    until(&mut page, &stats, "the bulk stream", |_| {
        stats.to_page() >= to_page_before + 262_144
    });
    let bulk_wall = wall.elapsed();
    let bulk_slices = stats.slices() - slices_before;
    virtual_clock::wait_virtual_ns(5 * MS);
    let worker = run(&mut page, &stats, "bulkstats", "window.t.stats()");
    let bulk_sum = log.lock().unwrap().bulk_sum;
    eprintln!(
        "mad shape, 256 KiB: {bulk_slices} slices ({:.3} s of board time) in {:.3} s of host \
         time ({:.2} ms a slice overall); {}; the worker read {} bytes",
        bulk_slices as f64 / 1e3,
        bulk_wall.as_secs_f64(),
        bulk_wall.as_secs_f64() * 1e3 / bulk_slices as f64,
        host_line(&stats),
        worker["bulkBytes"]
    );
    assert_eq!(worker["bulkBytes"], 262_144, "bulk-exact: {worker}");
    assert_eq!(worker["bulkSum"], bulk_sum, "bulk-exact: {worker}");
    assert!(
        stats.lead_ns().unsigned_abs() <= MS,
        "level-with-the-board: {stats:?}"
    );
    assert_eq!(stats.failure(), None);
    finish(system, actor);
}
