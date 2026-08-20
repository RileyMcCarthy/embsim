//! Opt-in, append-only engine event log — determinism **Oracle 1**
//! (`DETERMINISM.md`, "Oracle 1 (new, and the real one): the engine event
//! log").
//!
//! Every record is written from the **engine thread**, so the log is totally
//! ordered by construction — the same single-writer property that makes the
//! engine's event order *consistent* (`BOARD_ENGINE.md`, "Execution model")
//! makes this log a faithful transcript of it. The log records precisely the
//! set of things `DETERMINISM.md`'s T1 claims to determine, which is what makes
//! it the right oracle rather than the trace store.
//!
//! Off by default and **zero cost when off**: every recording site passes a
//! closure, so a disabled log neither allocates nor formats.
//!
//! ```no_run
//! use embsim_board::{EventLog, System};
//!
//! let system = System::new().event_log().start().expect("system starts");
//! let log: EventLog = system.event_log();
//! // … run the scenario …
//! let canonical: Vec<String> = log.normalized();
//! ```
//!
//! # Normalization — this *is* the contract
//!
//! `DETERMINISM.md`'s "Trace normalization spec" defines what two runs must
//! agree on. [`EngineEventRecord::normalized`] implements it, exactly:
//!
//! - **No wall-clock fields.** By design: a record carries only its append
//!   sequence and a *virtual* timestamp. There is no `Instant` anywhere in
//!   this module.
//! - **Virtual timestamps kept exactly.** `v_us` is never rounded or bucketed.
//!   In free-running mode it is sampled wall time and so *will* differ run to
//!   run — that is a fact to be measured, not smoothed over
//!   ([`EventLog::normalized_shape`] is the projection that drops it).
//! - **Floats quantized:** voltages to 1 µV, resistances to 1 mΩ — matching
//!   `BOARD_ENGINE.md`'s MNA hand-check tolerance. Unnecessary *within* one
//!   host and one binary (identical inputs in identical order give
//!   bit-identical IEEE-754 results); required the moment goldens are shared
//!   across architectures.
//! - **Identity canonicalized** to the dense ids that already exist
//!   ([`ComponentId`], [`EndpointId`], net index).
//! - **Host paths elided** from [`Finding`] payload strings (SD paths, PTY
//!   symlinks), conservatively: only tokens under a known host root are
//!   replaced, so KiCad hierarchical net names (`/Sheet1/NET`) survive.
//!
//! The normalized form is one canonical line per record. It is a stable text
//! encoding, deliberately not JSON (this crate has no serializer dependency) —
//! comparable, sortable, diffable, and directly writable as the
//! `board/tests/fixtures/traces/*` goldens of Phase D1.

use std::sync::{Arc, Mutex};

use embsim_core::virtual_clock;

use crate::diagnostics::Finding;
use crate::engine::{ComponentId, EndpointId};
use crate::net::{Level, NetId, NetState, TheveninDrive};

// ============================================================
// Constants
// ============================================================

/// Voltage quantum for normalization: 1 µV, per the trace normalization spec.
const VOLT_QUANTUM_PER_V: f64 = 1_000_000.0;

/// Resistance quantum for normalization: 1 mΩ, per the trace normalization
/// spec.
const OHM_QUANTUM_PER_OHM: f64 = 1_000.0;

/// Host filesystem roots whose paths are elided from finding payloads. A
/// token is only replaced when it *starts* with one of these, so relative
/// names and KiCad hierarchical net names (`/Sheet1/NET`) are untouched.
const HOST_PATH_ROOTS: &[&str] = &[
    "/tmp/",
    "/private/",
    "/var/",
    "/dev/",
    "/Users/",
    "/home/",
    "/Volumes/",
];

// ============================================================
// Records
// ============================================================

/// One engine event, in the vocabulary `DETERMINISM.md` Oracle 1 specifies.
#[derive(Debug, Clone, PartialEq)]
pub enum EngineEvent {
    /// A buffered drive reached the drive table, in enqueue-seq order.
    DriveApplied {
        /// The drive's global enqueue sequence — the authoritative event order.
        seq: u64,
        /// Endpoint whose contribution changed.
        endpoint: EndpointId,
        /// New Thevenin contribution, or release to high-Z.
        drive: Option<TheveninDrive>,
    },
    /// A net's resolved state changed on a resolution pass.
    NetResolved {
        /// Net index.
        net: NetId,
        /// Newly published state.
        state: NetState,
    },
    /// A sense callback was delivered for a net (at registration, or on a
    /// state change).
    SenseDelivered {
        /// Net index.
        net: NetId,
        /// State handed to the callback.
        state: NetState,
    },
    /// A timer-wheel wakeup fired for a component.
    Wake {
        /// Component whose wake handler ran.
        component: ComponentId,
    },
    /// One byte crossed a derived stream route to one consumer.
    StreamByte {
        /// Producer endpoint the byte was written on.
        producer: EndpointId,
        /// Consumer endpoint it was delivered to.
        consumer: EndpointId,
        /// The byte.
        byte: u8,
    },
    /// A stream-routing pass ran, publishing a topology epoch.
    Reroute {
        /// Topology epoch after the pass.
        epoch: u64,
    },
    /// A finding landed on the cumulative live bus (deduped there, so deduped
    /// here).
    Finding(Finding),
}

/// One log entry: append sequence, virtual timestamp, event.
///
/// No wall-clock field exists, by design (see the module docs).
#[derive(Debug, Clone, PartialEq)]
pub struct EngineEventRecord {
    /// Append index — the total order the engine thread wrote records in.
    pub seq: u64,
    /// Virtual microseconds at the write. `0` before `virtual_clock::init`:
    /// a log write must never panic the engine thread, and an uninitialized
    /// clock has no origin to read.
    pub v_us: u64,
    /// What happened.
    pub event: EngineEvent,
}

impl EngineEventRecord {
    /// The canonical normalized line for this record, per the trace
    /// normalization spec in the module docs. Includes `v_us`.
    pub fn normalized(&self) -> String {
        format!("{} v={} {}", self.seq, self.v_us, self.event_form())
    }

    /// The normalized line **without** `v_us` — the run-to-run invariant that
    /// free-running mode can actually be held to (`DETERMINISM.md`: virtual
    /// timestamps are sampled wall time until the stepped clock of Phase D1,
    /// but the drive/apply order and net-state sequence are already
    /// determined). The append sequence is kept: it is the position in the
    /// order, not a timestamp.
    pub fn normalized_shape(&self) -> String {
        format!("{} {}", self.seq, self.event_form())
    }

    /// Canonical encoding of the event payload, shared by both projections.
    fn event_form(&self) -> String {
        match &self.event {
            EngineEvent::DriveApplied {
                seq,
                endpoint,
                drive,
            } => format!(
                "drive_applied seq={seq} endpoint={} drive={}",
                endpoint.0,
                drive_form(drive.as_ref())
            ),
            EngineEvent::NetResolved { net, state } => {
                format!("net_resolved net={} state={}", net.0, state_form(state))
            }
            EngineEvent::SenseDelivered { net, state } => {
                format!("sense net={} state={}", net.0, state_form(state))
            }
            EngineEvent::Wake { component } => format!("wake component={}", component.0),
            EngineEvent::StreamByte {
                producer,
                consumer,
                byte,
            } => format!(
                "stream_byte producer={} consumer={} byte={byte:#04x}",
                producer.0, consumer.0
            ),
            EngineEvent::Reroute { epoch } => format!("reroute epoch={epoch}"),
            EngineEvent::Finding(finding) => {
                format!("finding {}", elide_host_paths(&format!("{finding:?}")))
            }
        }
    }
}

// ============================================================
// Normalization helpers
// ============================================================

/// Quantize a voltage to whole microvolts. `NaN`/infinity are passed through
/// as fixed tokens rather than formatted, so a defect value cannot make two
/// logs compare unequal for a reason that is not an ordering difference.
fn quantize(value: f64, per_unit: f64) -> String {
    if value.is_nan() {
        return "nan".to_string();
    }
    if value.is_infinite() {
        return if value.is_sign_negative() {
            "-inf".to_string()
        } else {
            "inf".to_string()
        };
    }
    // Integer quanta compare exactly, unlike a re-divided float.
    format!("{}", (value * per_unit).round() as i64)
}

/// Canonical encoding of a resolved net state, floats quantized.
fn state_form(state: &NetState) -> String {
    match state {
        NetState::Floating => "floating".to_string(),
        NetState::Driven(level) => format!("driven:{}", level_form(*level)),
        NetState::Pulled(level, ohms) => format!(
            "pulled:{}:{}mohm",
            level_form(*level),
            quantize(*ohms, OHM_QUANTUM_PER_OHM)
        ),
        NetState::Analog(volts) => format!("analog:{}uv", quantize(*volts, VOLT_QUANTUM_PER_V)),
        NetState::Contention => "contention".to_string(),
    }
}

/// Canonical encoding of a digital level.
fn level_form(level: Level) -> &'static str {
    match level {
        Level::Low => "low",
        Level::High => "high",
    }
}

/// Canonical encoding of a drive contribution, floats quantized.
fn drive_form(drive: Option<&TheveninDrive>) -> String {
    match drive {
        None => "release".to_string(),
        Some(drive) => format!(
            "{}uv@{}mohm",
            quantize(drive.volts, VOLT_QUANTUM_PER_V),
            quantize(drive.impedance, OHM_QUANTUM_PER_OHM)
        ),
    }
}

/// Replace host filesystem paths in a finding payload with `<path>`.
///
/// Conservative by construction: a candidate must begin with one of
/// [`HOST_PATH_ROOTS`], so `/Sheet1/NET` (a KiCad hierarchical net name, and
/// load-bearing identity) is never touched. Tokens are delimited by
/// whitespace and by the punctuation that `Debug` output wraps strings in.
fn elide_host_paths(text: &str) -> String {
    let is_boundary = |c: char| c.is_whitespace() || matches!(c, '"' | '\'' | ',' | '(' | ')');
    let mut out = String::with_capacity(text.len());
    let mut rest = text;
    while !rest.is_empty() {
        let token_end = rest.find(is_boundary).unwrap_or(rest.len());
        if token_end == 0 {
            // A boundary character: copy it and continue.
            let mut chars = rest.chars();
            let c = chars.next().expect("rest is non-empty");
            out.push(c);
            rest = chars.as_str();
            continue;
        }
        let (token, tail) = rest.split_at(token_end);
        if HOST_PATH_ROOTS.iter().any(|root| token.starts_with(root)) {
            out.push_str("<path>");
        } else {
            out.push_str(token);
        }
        rest = tail;
    }
    out
}

// ============================================================
// The log
// ============================================================

/// Cloneable handle to an engine event log. Disabled by default; clones share
/// one buffer, so a handle taken from `SystemHandle` reads what the engine
/// thread is writing.
#[derive(Debug, Clone, Default)]
pub struct EventLog {
    /// `None` when disabled — the whole recording path collapses to one
    /// `Option` check.
    inner: Option<Arc<Mutex<Vec<EngineEventRecord>>>>,
}

impl EventLog {
    /// A live log. Recording is on.
    pub fn enabled() -> Self {
        Self {
            inner: Some(Arc::new(Mutex::new(Vec::new()))),
        }
    }

    /// An inert log. Recording is off and costs one `Option` check per site.
    pub fn disabled() -> Self {
        Self::default()
    }

    /// True when this log records.
    pub fn is_enabled(&self) -> bool {
        self.inner.is_some()
    }

    /// Append one event, stamped with the current append sequence and virtual
    /// time. The event is only *built* when the log is enabled, so a disabled
    /// log neither allocates nor clones.
    ///
    /// Called from the engine thread (and, for the pre-spawn resolution and
    /// routing passes, from the thread that is about to become it) — which is
    /// what makes the resulting order total.
    pub(crate) fn record(&self, event: impl FnOnce() -> EngineEvent) {
        let Some(inner) = &self.inner else {
            return;
        };
        let event = event();
        // Reading virtual time must never panic the engine thread: before
        // `virtual_clock::init` there is no origin, and a timer-free system
        // legitimately never initializes one (`EngineCore::next_wall_wait_us`).
        let v_us = if virtual_clock::is_initialized() {
            virtual_clock::virtual_us()
        } else {
            0
        };
        let mut records = inner.lock().expect("event log mutex never poisoned");
        let seq = records.len() as u64;
        records.push(EngineEventRecord { seq, v_us, event });
    }

    /// Snapshot of every record, in append order.
    pub fn records(&self) -> Vec<EngineEventRecord> {
        match &self.inner {
            Some(inner) => inner.lock().expect("event log never poisoned").clone(),
            None => Vec::new(),
        }
    }

    /// Number of records appended so far.
    pub fn len(&self) -> usize {
        match &self.inner {
            Some(inner) => inner.lock().expect("event log never poisoned").len(),
            None => 0,
        }
    }

    /// True when nothing has been recorded (always true when disabled).
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Every record as its canonical normalized line, `v_us` included.
    pub fn normalized(&self) -> Vec<String> {
        self.records().iter().map(|r| r.normalized()).collect()
    }

    /// Every record as its canonical normalized line **without** `v_us` — the
    /// projection free-running mode can be held to run-to-run.
    pub fn normalized_shape(&self) -> Vec<String> {
        self.records()
            .iter()
            .map(|r| r.normalized_shape())
            .collect()
    }
}

// ============================================================
// Tests
// ============================================================

#[cfg(test)]
mod tests {
    use rstest::rstest;

    use super::*;
    use crate::diagnostics::SenseKind;
    use crate::net::PinRef;

    fn record(event: EngineEvent) -> EngineEventRecord {
        EngineEventRecord {
            seq: 7,
            v_us: 1234,
            event,
        }
    }

    /// A disabled log is inert: nothing accumulates and the recording closure
    /// is never even called (the "zero cost when off" claim, asserted rather
    /// than assumed).
    #[rstest]
    fn disabled_log_records_nothing_and_never_builds_an_event() {
        let log = EventLog::disabled();
        assert!(!log.is_enabled());
        for _ in 0..10 {
            log.record(|| panic!("a disabled log must not build the event"));
        }
        assert!(log.is_empty());
        assert!(log.records().is_empty());
        assert!(log.normalized().is_empty());
    }

    /// An enabled log stamps records with a dense append sequence, in order,
    /// and clones share the buffer.
    #[rstest]
    fn enabled_log_appends_in_order_and_clones_share_the_buffer() {
        let log = EventLog::enabled();
        let mirror = log.clone();
        for i in 0..5u64 {
            log.record(|| EngineEvent::Reroute { epoch: i });
        }
        let records = mirror.records();
        assert_eq!(records.len(), 5);
        assert_eq!(mirror.len(), 5);
        let seqs: Vec<u64> = records.iter().map(|r| r.seq).collect();
        assert_eq!(seqs, vec![0, 1, 2, 3, 4], "append seq must be dense");
        let epochs: Vec<u64> = records
            .iter()
            .map(|r| match r.event {
                EngineEvent::Reroute { epoch } => epoch,
                ref other => panic!("unexpected event {other:?}"),
            })
            .collect();
        assert_eq!(
            epochs,
            vec![0, 1, 2, 3, 4],
            "append order must be write order"
        );
    }

    /// Voltages normalize to whole microvolts and resistances to whole
    /// milliohms, so two hosts that disagree in the last mantissa bits still
    /// compare equal (the spec's cross-architecture clause).
    #[rstest]
    #[case::analog(
        NetState::Analog(1.234_567_891),
        "net_resolved net=3 state=analog:1234568uv"
    )]
    #[case::analog_sub_quantum(
        NetState::Analog(1.234_567_499),
        "net_resolved net=3 state=analog:1234567uv"
    )]
    #[case::pulled(
        NetState::Pulled(Level::High, 4_700.000_000_1),
        "net_resolved net=3 state=pulled:high:4700000mohm"
    )]
    #[case::driven(NetState::Driven(Level::Low), "net_resolved net=3 state=driven:low")]
    #[case::floating(NetState::Floating, "net_resolved net=3 state=floating")]
    #[case::contention(NetState::Contention, "net_resolved net=3 state=contention")]
    fn net_states_quantize_to_the_spec_quanta(#[case] state: NetState, #[case] expected: &str) {
        let line = record(EngineEvent::NetResolved {
            net: NetId(3),
            state,
        })
        .normalized();
        assert_eq!(line, format!("7 v=1234 {expected}"));
    }

    /// Two voltages within one quantum of each other normalize to the same
    /// token — the property that makes a golden trace portable.
    #[rstest]
    fn sub_quantum_float_differences_normalize_away() {
        let a = record(EngineEvent::NetResolved {
            net: NetId(0),
            state: NetState::Analog(2.894_382_877_392_857),
        });
        let b = record(EngineEvent::NetResolved {
            net: NetId(0),
            // The 4-ULP spread the pre-fix `cluster_sources` order produced.
            state: NetState::Analog(2.894_382_877_392_858),
        });
        assert_ne!(a, b, "the raw records really do differ");
        assert_eq!(
            a.normalized(),
            b.normalized(),
            "a sub-microvolt difference must normalize away"
        );
    }

    /// `normalized_shape` drops `v_us` and keeps everything else, so two runs
    /// that agree on order but not on sampled wall time still compare equal.
    #[rstest]
    fn shape_projection_drops_only_the_virtual_timestamp() {
        let event = EngineEvent::Wake {
            component: ComponentId(2),
        };
        let early = EngineEventRecord {
            seq: 4,
            v_us: 10,
            event: event.clone(),
        };
        let late = EngineEventRecord {
            seq: 4,
            v_us: 999_999,
            event,
        };
        assert_ne!(early.normalized(), late.normalized());
        assert_eq!(early.normalized_shape(), late.normalized_shape());
        assert_eq!(early.normalized_shape(), "4 wake component=2");
    }

    /// Host paths are elided from finding payloads; KiCad hierarchical net
    /// names, which look superficially similar, are not.
    #[rstest]
    #[case::tmp_sd("/tmp/sd/DATA.CSV", true)]
    #[case::pty("/dev/ttys004", true)]
    #[case::home("/Users/someone/embsim", true)]
    #[case::hierarchical_net("/Sheet1/RESET", false)]
    #[case::relative("sd/DATA.CSV", false)]
    fn host_paths_are_elided_but_net_names_survive(#[case] token: &str, #[case] elided: bool) {
        let payload = format!("value at {token} end");
        let out = elide_host_paths(&payload);
        if elided {
            assert_eq!(out, "value at <path> end", "{token} should be elided");
        } else {
            assert_eq!(out, payload, "{token} must survive verbatim");
        }
    }

    /// Elision reaches inside `Debug`-quoted strings (which is how findings
    /// carry paths) without eating the quotes.
    #[rstest]
    fn elision_handles_debug_quoted_paths() {
        let line = record(EngineEvent::Finding(Finding::ClassificationError {
            reference: "U1".to_string(),
            part: "ADS122U04".to_string(),
            message: "could not read /tmp/embsim-xyz/board.net".to_string(),
        }))
        .normalized();
        assert!(
            line.contains("<path>") && !line.contains("/tmp/"),
            "host path must be elided: {line}"
        );
        assert!(
            line.contains("\"U1\"") && line.contains("\"ADS122U04\""),
            "identity fields must survive: {line}"
        );
    }

    /// Every event variant has a distinct, stable canonical form — a log
    /// comparison is only meaningful if two different events cannot normalize
    /// to the same line.
    #[rstest]
    fn every_event_variant_normalizes_distinctly() {
        let drive = TheveninDrive {
            volts: 3.3,
            impedance: 25.0,
        };
        let events = vec![
            EngineEvent::DriveApplied {
                seq: 1,
                endpoint: EndpointId(0),
                drive: Some(drive),
            },
            EngineEvent::DriveApplied {
                seq: 1,
                endpoint: EndpointId(0),
                drive: None,
            },
            EngineEvent::NetResolved {
                net: NetId(0),
                state: NetState::Floating,
            },
            EngineEvent::SenseDelivered {
                net: NetId(0),
                state: NetState::Floating,
            },
            EngineEvent::Wake {
                component: ComponentId(0),
            },
            EngineEvent::StreamByte {
                producer: EndpointId(0),
                consumer: EndpointId(1),
                byte: 0x5a,
            },
            EngineEvent::Reroute { epoch: 0 },
            EngineEvent::Finding(Finding::FloatingSense {
                net: "N0".to_string(),
                kind: SenseKind::Digital,
            }),
            EngineEvent::Finding(Finding::StreamOverrun {
                producer: PinRef::new("U1", "1"),
            }),
        ];
        let forms: Vec<String> = events
            .into_iter()
            .map(|e| record(e).normalized_shape())
            .collect();
        let mut unique = forms.clone();
        unique.sort();
        unique.dedup();
        assert_eq!(
            unique.len(),
            forms.len(),
            "event forms must be distinguishable: {forms:?}"
        );
        // Spot-check the exact encodings so the golden format is pinned, not
        // merely internally consistent.
        assert_eq!(
            forms[0],
            "7 drive_applied seq=1 endpoint=0 drive=3300000uv@25000mohm"
        );
        assert_eq!(forms[1], "7 drive_applied seq=1 endpoint=0 drive=release");
        assert_eq!(forms[5], "7 stream_byte producer=0 consumer=1 byte=0x5a");
    }

    /// Non-finite floats encode as fixed tokens rather than platform-dependent
    /// float formatting, so a defect value cannot masquerade as an ordering
    /// difference.
    #[rstest]
    #[case::nan(f64::NAN, "nan")]
    #[case::inf(f64::INFINITY, "inf")]
    #[case::neg_inf(f64::NEG_INFINITY, "-inf")]
    fn non_finite_floats_encode_as_tokens(#[case] volts: f64, #[case] expected: &str) {
        let line = record(EngineEvent::NetResolved {
            net: NetId(0),
            state: NetState::Analog(volts),
        })
        .normalized_shape();
        assert_eq!(
            line,
            format!("7 net_resolved net=0 state=analog:{expected}uv")
        );
    }
}
