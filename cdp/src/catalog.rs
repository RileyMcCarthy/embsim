//! The host's Chrome as a bench component kind a project can name.
//!
//! [`CdpCatalog`] is a [`Catalog`] of one bench component kind,
//! `chrome-cdp`, and [`register`] adds it to a set, as the `embsim`
//! command's set has it:
//!
//! ```
//! use embsim_board::Catalog;
//! use embsim_boards::catalog::CatalogSet;
//!
//! let mut set = CatalogSet::new();
//! embsim_cdp::catalog::register(&mut set).expect("one kind, spelled as kinds are");
//! assert!(set
//!     .component_kinds()
//!     .iter()
//!     .any(|kind| kind.name == "chrome-cdp"));
//! ```
//!
//! It is a [`CdpNode`]: the pins of a host's serial line at its own rail
//! (`TX`, `RX`, `VIO`, `GND`, as `host-serial` has them), Web Serial in
//! every page of the host's Chrome on the far side, every page's clock
//! metered by the board's a quantum at a time. Building the project checks
//! what can be checked without starting anything — the Chrome binary — and
//! Chrome is launched (or attached to) at the node's first slice, a quantum
//! after the run starts, with the board's clock held there while it does;
//! so `embsim check` starts nothing. A grant that sticks, a lead past
//! `max_lead`, or a Chrome that goes away stops the run
//! ([`Report::failure`]).
//!
//! ```toml
//! [[component]]
//! name = "PC"
//! kind = "chrome-cdp"
//! [component.options]
//! baud = 2000000
//! usb_vendor_id = 0x0403
//! usb_product_id = 0x6001
//! devtools_port = 9222
//! ```

use std::path::PathBuf;
use std::sync::Arc;
use std::time::Duration;

use embsim_board::report::instant;
use embsim_board::{Catalog, Component, ComponentRequest, KindInfo, ProjectError, Report};
use embsim_boards::catalog::CatalogSet;

use crate::chrome::{find_chrome, free_port, on_path, LaunchSpec};
use crate::node::span;
use crate::{
    Browse, CdpNode, NodeStats, Settings, UsbIds, DEFAULT_QUANTUM, DEFAULT_STUCK_AFTER, MAX_QUANTUM,
};

/// The catalog's name, as an error naming two catalogs prints it.
pub const NAME: &str = "embsim-cdp";

/// The kind.
pub const KIND: &str = "chrome-cdp";

/// Add the `chrome-cdp` kind to `set`.
pub fn register(set: &mut CatalogSet) -> Result<(), ProjectError> {
    set.add(CdpCatalog)
}

/// The `chrome-cdp` bench component kind (module docs).
#[derive(Debug, Default, Clone, Copy)]
pub struct CdpCatalog;

impl Catalog for CdpCatalog {
    fn name(&self) -> &str {
        NAME
    }

    fn component_kinds(&self) -> Vec<KindInfo> {
        vec![KindInfo::new(
            KIND,
            "the host's Chrome, its pages' Web Serial on the wire and their clocks metered by \
             the board's over DevTools",
        )
        .requires("baud", "2000000", "the serial line's rate, framed 8N1")]
    }

    fn component(&self, request: ComponentRequest<'_>) -> Result<Box<dyn Component>, ProjectError> {
        if request.spec.kind != KIND {
            return Err(ProjectError::message(format!(
                "component {}: unknown kind {:?}; the kind of {NAME} is \"{KIND}\"",
                request.spec.name, request.spec.kind
            )));
        }
        chrome_cdp(request)
    }
}

/// `chrome-cdp`: read the options, check the binary, build the node.
fn chrome_cdp(request: ComponentRequest<'_>) -> Result<Box<dyn Component>, ProjectError> {
    let ComponentRequest {
        spec,
        mut options,
        dir,
        reports,
        ..
    } = request;
    let error = |message: String| {
        ProjectError::message(format!(
            "component {} (kind {:?}): {message}",
            spec.name, spec.kind
        ))
    };
    let baud = options.integer("baud")?.ok_or_else(|| {
        error(
            "options.baud is the serial line's rate, framed 8N1; a host names its rate, and the \
             kind invents none (baud = 2000000)"
                .to_string(),
        )
    })?;
    let baud = u32::try_from(baud)
        .ok()
        .filter(|baud| *baud > 0)
        .ok_or_else(|| error(format!("options.baud = {baud} is not a rate")))?;
    let quantum_ns = options
        .duration("quantum")?
        .unwrap_or(DEFAULT_QUANTUM.as_nanos() as u64);
    if quantum_ns == 0 {
        return Err(error(
            "options.quantum is how much virtual time passes between two slices of the page; it \
             is more than 0"
                .to_string(),
        ));
    }
    if quantum_ns > MAX_QUANTUM.as_nanos() as u64 {
        return Err(error(format!(
            "options.quantum is at most {}: the page lags the board by up to a quantum, and each \
             slice holds the board for one",
            instant(MAX_QUANTUM.as_nanos() as u64)
        )));
    }
    let max_lead = options.duration("max_lead")?;
    if max_lead == Some(0) {
        return Err(error(
            "options.max_lead is how far a page's clock may run ahead of the board before the \
             run stops; Chrome moves a clock outside its budget now and then, so it is more than 0"
                .to_string(),
        ));
    }
    let stuck_after = options
        .duration("stuck_after")?
        .unwrap_or(DEFAULT_STUCK_AFTER.as_nanos() as u64);
    if stuck_after == 0 {
        return Err(error(
            "options.stuck_after is how long a grant may hold, in host time, before the run fails \
             as stuck; it is more than 0"
                .to_string(),
        ));
    }
    let usb_id = |options: &mut embsim_board::PartOptions, name: &'static str| {
        options.integer(name)?.map_or(Ok(None), |id| {
            u16::try_from(id)
                .map(Some)
                .map_err(|_| error(format!("options.{name} = {id} is not a 16-bit USB id")))
        })
    };
    let usb = UsbIds {
        vendor: usb_id(&mut options, "usb_vendor_id")?,
        product: usb_id(&mut options, "usb_product_id")?,
    };
    let granted = boolean(&mut options, "granted")?.unwrap_or(false);
    let url = options.string("url")?;
    let attach = options.string("attach")?;
    let chrome = options.string("chrome")?;
    let port = options.integer("devtools_port")?;
    let headless = boolean(&mut options, "headless")?;
    options.finish()?;

    let browse = match attach {
        Some(endpoint) => {
            for (given, name) in [
                (chrome.is_some(), "chrome"),
                (port.is_some(), "devtools_port"),
                (headless.is_some(), "headless"),
            ] {
                if given {
                    return Err(error(format!(
                        "options.{name} says how to launch Chrome, and options.attach says to \
                         attach to one already running; name one or the other"
                    )));
                }
            }
            if !(endpoint.starts_with("http://") || endpoint.starts_with("ws://")) {
                return Err(error(format!(
                    "options.attach = {endpoint:?} is a DevTools endpoint: http://HOST:PORT, or \
                     the browser's ws:// URL"
                )));
            }
            Browse::Attach(endpoint)
        }
        None => {
            let binary = match chrome {
                Some(name) if name.contains('/') => dir.join(name),
                Some(name) => on_path(&name).unwrap_or_else(|| PathBuf::from(name)),
                None => find_chrome().unwrap_or_default(),
            };
            if !binary.is_file() {
                return Err(error(format!(
                    "no Chrome {}: the kind launches the host's Chrome (Google Chrome on macOS, \
                     google-chrome or chromium on Linux), or options.chrome names one, or \
                     options.attach a browser already running",
                    if binary.as_os_str().is_empty() {
                        "found in the usual places".to_string()
                    } else {
                        format!("at {}", binary.display())
                    }
                )));
            }
            let port = match port {
                None => free_port()
                    .map_err(|e| error(format!("cannot find a free port for DevTools: {e}")))?,
                Some(port) => u16::try_from(port)
                    .ok()
                    .filter(|port| *port > 0)
                    .ok_or_else(|| {
                        error(format!("options.devtools_port = {port} is not a TCP port"))
                    })?,
            };
            Browse::Launch(LaunchSpec {
                binary,
                port,
                headless: headless.unwrap_or(true),
            })
        }
    };
    let what = match &browse {
        Browse::Launch(spec) => format!(
            "Chrome {}{}, DevTools at http://127.0.0.1:{}",
            spec.binary.display(),
            if spec.headless { " (headless)" } else { "" },
            spec.port
        ),
        Browse::Attach(endpoint) => format!("the Chrome at {endpoint}"),
    };
    let settings = Settings {
        browse,
        quantum: Duration::from_nanos(quantum_ns),
        max_lead: max_lead.map(Duration::from_nanos),
        stuck_after: Duration::from_nanos(stuck_after),
        url,
        usb,
        granted,
    };
    let node = CdpNode::new(settings, baud);
    reports.add(CdpReport {
        subject: spec.name.clone(),
        what,
        baud,
        quantum_ns,
        stats: node.stats(),
        said: Said::default(),
    });
    Ok(Box::new(node))
}

/// A `true`/`false` option.
fn boolean(
    options: &mut embsim_board::PartOptions,
    name: &'static str,
) -> Result<Option<bool>, ProjectError> {
    match options.value(name) {
        None => Ok(None),
        Some(toml::Value::Boolean(value)) => Ok(Some(value)),
        Some(other) => Err(options.error(format!(
            "options.{name} is true or false; {other} is a {}",
            other.type_str()
        ))),
    }
}

/// What a report has said so far.
#[derive(Debug, Default)]
struct Said {
    start: bool,
    boot: bool,
    mismatch: bool,
    drain_off: bool,
    unmetered: bool,
}

/// What a `chrome-cdp` says in a run: what it launches at the first look,
/// when Chrome is up, and at the end how the pages were metered and what
/// crossed the line.
struct CdpReport {
    subject: String,
    what: String,
    baud: u32,
    quantum_ns: u64,
    stats: Arc<NodeStats>,
    said: Said,
}

impl Report for CdpReport {
    fn subject(&self) -> String {
        self.subject.clone()
    }

    fn look(&mut self, _now_ns: u64) -> Vec<String> {
        let mut lines = Vec::new();
        if !std::mem::replace(&mut self.said.start, true) {
            lines.push(format!(
                "{}, {} baud 8N1, metered every {} of virtual time; Chrome is reached at the \
                 first slice",
                self.what,
                self.baud,
                instant(self.quantum_ns)
            ));
        }
        if let Some((took, at_ns)) = self.stats.booted() {
            if !std::mem::replace(&mut self.said.boot, true) {
                let said = self.stats.said();
                lines.push(format!(
                    "{} reached in {:.3} s of host time, the board's clock held at {}; DevTools \
                     at {}; its pages live only while the board's clock advances",
                    said.version,
                    took.as_secs_f64(),
                    instant(at_ns),
                    said.endpoint
                ));
            }
        }
        let said = self.stats.said();
        if let Some(mismatch) = said.mismatch {
            if !std::mem::replace(&mut self.said.mismatch, true) {
                lines.push(mismatch);
            }
        }
        if self.stats.drain_off() > 0 && !std::mem::replace(&mut self.said.drain_off, true) {
            lines.push(
                "a page's consumer did not come back for more after its bytes in several waits \
                 in a row: the drain barrier is off for it, so a busy consumer may take the \
                 board's bytes a slice late"
                    .to_string(),
            );
        }
        if said.workers_unmetered > 0 && !std::mem::replace(&mut self.said.unmetered, true) {
            lines.push(
                "Chrome refused a virtual-time policy on a dedicated worker: its timers run on \
                 host time"
                    .to_string(),
            );
        }
        lines
    }

    fn summary(&self) -> Vec<String> {
        let stats = &self.stats;
        let said = stats.said();
        let mut lines = vec![format!(
            "{} slices ({} booked from the page's clock, {} grants skipped to pay a lead back): \
             the page lived {} of the board's {}",
            stats.slices(),
            stats.clocked(),
            stats.skipped(),
            instant(stats.lived_ns()),
            instant(stats.board_ns()),
        )];
        lines.push(format!(
            "at most {} ahead of the board at a slice; a page's clock passed its budget by at \
             most {} in one slice",
            span(stats.peak_lead_ns()),
            span(stats.peak_overrun_ns()),
        ));
        if let (Some(median), Some(p90), Some(p99)) = (
            stats.host_per_slice(0.5),
            stats.host_per_slice(0.9),
            stats.host_per_slice(0.99),
        ) {
            lines.push(format!(
                "host time per slice: median {:.2} ms, 90th percentile {:.2} ms, 99th {:.2} ms",
                median.as_secs_f64() * 1e3,
                p90.as_secs_f64() * 1e3,
                p99.as_secs_f64() * 1e3,
            ));
        }
        lines.push(format!(
            "{} page{} and {} dedicated worker{} held; {} drain waits, {} ran out",
            said.pages,
            if said.pages == 1 { "" } else { "s" },
            said.workers,
            if said.workers == 1 { "" } else { "s" },
            stats.drain_waits(),
            stats.drain_timeouts(),
        ));
        lines.push(format!(
            "{} bytes from the page, {} to it, {} framing errors",
            stats.from_page(),
            stats.to_page(),
            stats.framing_errors()
        ));
        if stats.from_page() > 0 {
            lines.push(
                "the page sent during the run: its bytes entered the line in the slice they \
                 reached the node in, so this run is reproducible in what the page sent, and in \
                 when to within a slice"
                    .to_string(),
            );
        }
        if stats.unheard() > 0 {
            lines.push(format!(
                "{} bytes the board sent were heard by no page: none had the port open, or the \
                 cable was out",
                stats.unheard()
            ));
        }
        if stats.flushed() > 0 {
            lines.push(format!(
                "{} bytes discarded by flushes the page asked for: a cancelled read, an aborted \
                 write, a close",
                stats.flushed()
            ));
        }
        if stats.mismatched_opens() > 0 {
            lines.push(format!(
                "{} open{} at a rate or framing other than the line's; {} bytes shed for it",
                stats.mismatched_opens(),
                if stats.mismatched_opens() == 1 {
                    ""
                } else {
                    "s"
                },
                stats.mismatched_bytes()
            ));
        }
        if stats.shed() > 0 {
            lines.push(format!(
                "{} bytes were shed: a port opened at the wrong rate, a line with no rail (VIO \
                 read no voltage: {}), a page writing to a port it does not hold, or a queue past \
                 a megabyte",
                stats.shed(),
                stats.unpowered()
            ));
        }
        if stats.reanchored() > 0 {
            lines.push(format!(
                "a page's clock jumped {} time{} (a navigation to a new renderer process): the \
                 books were re-anchored there",
                stats.reanchored(),
                if stats.reanchored() == 1 { "" } else { "s" }
            ));
        }
        if let Some(failure) = stats.failure() {
            lines.push(format!("failed: {failure}"));
        }
        lines
    }

    fn failure(&self) -> Option<String> {
        self.stats.failure()
    }
}
