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
//! `connectOverCDP` would.
//!
//! What is asserted is what the page and the board did in virtual time:
//! ticks counted, clocks read, bytes compared. Host time is measured and
//! printed, never asserted; the wall-time bound on a wait is sized for a
//! hang.

mod common;

use std::sync::atomic::Ordering;
use std::time::{Duration, Instant};

use common::{bench, finish, suite_lock, PageClient, Peer, Server};
use embsim_cdp::{
    find_chrome, free_port, Browse, CdpNode, LaunchSpec, NodeStats, Settings, UsbIds,
};
use embsim_core::virtual_clock;
use serde_json::Value;

/// How long, in wall time, a case may wait for the page before it is hung.
const HANG: Duration = Duration::from_secs(180);

/// One millisecond of board time.
const MS: u64 = 1_000_000;

fn pages() -> std::path::PathBuf {
    std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/pages")
}

/// A node that launches the host's Chrome on `url`, headless, granted the
/// port (as Chrome's `SerialAllowUsbDevicesForUrls` policy grants it), its
/// adapter an FTDI's ids.
fn node(url: &str, baud: u32) -> (CdpNode, String) {
    let binary = find_chrome().expect("a Chrome on this host (or CHROME names one)");
    let port = free_port().expect("a free port");
    let mut settings = Settings::new(Browse::Launch(LaunchSpec {
        binary,
        port,
        headless: true,
    }));
    settings.url = Some(url.to_string());
    settings.granted = true;
    settings.usb = UsbIds {
        vendor: Some(0x0403),
        product: Some(0x6001),
    };
    (
        CdpNode::new(settings, baud),
        format!("http://127.0.0.1:{port}"),
    )
}

/// Hand the board time a millisecond at a time until `done` holds of the
/// page, or the case is hung.
fn until(
    page: &mut PageClient,
    stats: &NodeStats,
    what: &str,
    mut done: impl FnMut(&mut PageClient) -> bool,
) {
    let started = Instant::now();
    while !done(page) {
        assert!(
            started.elapsed() < HANG && stats.failure().is_none(),
            "{what} never happened by {} ns of board time: {stats:?}",
            virtual_clock::virtual_ns()
        );
        virtual_clock::wait_virtual_ns(MS);
    }
}

/// Run the board until the node has reached Chrome, and attach the case's
/// own DevTools client to the page.
fn page_up(endpoint: &str, url: &str, stats: &NodeStats) -> PageClient {
    let started = Instant::now();
    while stats.booted().is_none() {
        assert!(
            started.elapsed() < HANG && stats.failure().is_none(),
            "Chrome never came up: {stats:?}"
        );
        virtual_clock::wait_virtual_ns(MS);
    }
    let mut page = PageClient::attach(endpoint, url);
    until(&mut page, stats, "the page's ready flag", |page| {
        page.eval("window.ready === true") == Value::Bool(true)
    });
    page
}

/// Start `name` in the page and run the board until it settles.
fn run(page: &mut PageClient, stats: &NodeStats, name: &str, call: &str) -> Value {
    page.eval(&format!("window.run({name:?}, () => {call})"));
    until(page, stats, name, |page| {
        page.eval(&format!("window.out[{name:?}].done")) == Value::Bool(true)
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

/// Given the host's Chrome metered every millisecond by the node, its page and
/// a dedicated worker each running a 4 ms timer stamped with performance.now()
/// and with a WASM module's imported clock, run for 400 milliseconds of board
/// time once the page is up:
/// - the worker's timer ticks exactly 100 times in the 400 milliseconds, 4.000
///   ms apart to within 10 µs (`worker-ticks-100`)
///   — a dedicated worker shares the page's clock only once its own session is
///   given a virtual-time policy, so its timers advance in the page's grants
/// - the page's own 4 ms timer ticks exactly 100 times in the same span
///   (`page-ticks-100`)
/// - the page's performance.now() advances by the board's 400 ms to within 10
///   µs, Date.now() by 400 ms to within 1 ms, and the WASM clock reads what
///   performance.now() reads, in the page and in the worker, at every tick
///   (`clocks-advance-by-the-board`)
///
/// Not a declared behaviour: the case is `#[ignore]`d where the ledger is
/// collected (TESTING.md).
#[test]
#[ignore = "launches the host's Chrome; CI's chrome-cdp job runs it"]
fn a_workers_four_millisecond_timer_ticks_once_every_four_grants() {
    let _suite = suite_lock();
    let server = Server::start(&pages());
    let url = format!("{}clocks.html", server.url);
    let (node, endpoint) = node(&url, 115_200);
    let stats = node.stats();
    let (peer, _peer) = Peer::new(115_200, false);
    let (system, actor) = bench(node, Box::new(peer));
    let mut page = page_up(&endpoint, &url, &stats);

    let before = page.eval("window.clocks()");
    let from = before["now"].as_f64().unwrap();
    virtual_clock::wait_virtual_ns(400 * MS);
    let after = page.eval("window.clocks()");
    let to = after["now"].as_f64().unwrap();
    eprintln!("canary: {}", host_line(&stats));

    let within = |ticks: &[f64]| -> Vec<f64> {
        ticks
            .iter()
            .copied()
            .filter(|&t| t > from && t <= to)
            .collect()
    };
    let worker = within(&f64s(&after["worker"]));
    let main = within(&f64s(&after["main"]));
    assert_eq!(worker.len(), 100, "worker-ticks-100: {worker:?}");
    for pair in worker.windows(2) {
        assert!(
            (pair[1] - pair[0] - 4.0).abs() <= 0.01,
            "worker-ticks-100: ticks {pair:?} are not 4 ms apart"
        );
    }
    assert_eq!(main.len(), 100, "page-ticks-100: {main:?}");
    assert!(
        (to - from - 400.0).abs() <= 0.01,
        "clocks-advance-by-the-board: performance.now() moved {} ms",
        to - from
    );
    let date = after["date"].as_f64().unwrap() - before["date"].as_f64().unwrap();
    assert!(
        (date - 400.0).abs() <= 1.0,
        "clocks-advance-by-the-board: Date.now() moved {date} ms"
    );
    for (now, wasm) in [("main", "mainWasm"), ("worker", "workerWasm")] {
        let (nows, wasms) = (f64s(&after[now]), f64s(&after[wasm]));
        assert_eq!(nows.len(), wasms.len());
        for (n, w) in nows.iter().zip(&wasms) {
            assert!(
                (n - w).abs() <= 0.01,
                "clocks-advance-by-the-board: {now} read {n}, its WASM clock {w}"
            );
        }
    }
    assert!(
        (after["wasm"].as_f64().unwrap() - to).abs() <= 0.01,
        "clocks-advance-by-the-board"
    );
    assert_eq!(stats.failure(), None);
    finish(system, actor);
}

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
    let (node, endpoint) = node(&url, 2_000_000);
    let stats = node.stats();
    let (peer, heard) = Peer::new(2_000_000, true);
    let (system, actor) = bench(node, Box::new(peer));
    let mut page = page_up(&endpoint, &url, &stats);

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
    let (node, endpoint) = node(&url, 115_200);
    let stats = node.stats();
    let (peer, _heard) = Peer::new(115_200, false);
    let (system, actor) = bench(node, Box::new(peer));
    let mut page = page_up(&endpoint, &url, &stats);

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
    let (node, endpoint) = node(&url, 115_200);
    let stats = node.stats();
    let (peer, _heard) = Peer::new(115_200, false);
    let (system, actor) = bench(node, Box::new(peer));
    let mut page = page_up(&endpoint, &url, &stats);

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
    log: std::sync::Arc<std::sync::Mutex<BoardLog>>,
    shutdown: std::sync::Arc<std::sync::atomic::AtomicBool>,
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
        let line = embsim_board::HostRailLine::attach(
            &io,
            self.framing,
            std::sync::Arc::clone(&self.shutdown),
        )?;
        {
            let (line, io2, log) = (line.clone(), io.clone(), std::sync::Arc::clone(&self.log));
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
            let (io2, log) = (io.clone(), std::sync::Arc::clone(&self.log));
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
    let (node, endpoint) = node(&url, 2_000_000);
    let stats = node.stats();
    let log = std::sync::Arc::new(std::sync::Mutex::new(BoardLog::default()));
    let board = Board {
        framing: embsim_board::uart::UartFraming::new_8n1(2_000_000),
        log: std::sync::Arc::clone(&log),
        shutdown: std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false)),
        io: None,
    };
    let (system, actor) = bench(node, Box::new(board));
    let mut page = page_up(&endpoint, &url, &stats);
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
