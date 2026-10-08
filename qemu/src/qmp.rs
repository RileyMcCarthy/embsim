//! A small client for QMP, the QEMU Machine Protocol.
//!
//! QMP is JSON over a socket, one object per line: a greeting on connect, a
//! capabilities handshake, then `{"execute": ..}` answered by `{"return": ..}`
//! or `{"error": ..}`, with `{"event": ..}` objects arriving unasked at any
//! point in between. This is just enough of it to freeze and thaw a guest —
//! `stop`, `cont`, `query-status`, `quit` — plus the human-monitor escape
//! hatch for the odd thing QMP has no command for (a live `hostfwd_add`).

use std::fmt;
use std::io::{self, BufRead, BufReader, Write};
use std::os::unix::net::UnixStream;
use std::path::Path;
use std::time::Duration;

use serde_json::{json, Value};

/// How many unasked-for events a connection keeps for
/// [`take_events`](Qmp::take_events). QEMU sends `STOP` and `RESUME` for
/// every freeze and thaw — two per slice, forever — so a connection nobody
/// drains must not grow with the run; the newest are kept.
pub const MAX_RETAINED_EVENTS: usize = 64;

/// Bound on a single round trip, connect included. A guest that takes longer
/// than this to acknowledge `stop` is not a guest we can meter.
pub const DEFAULT_TIMEOUT: Duration = Duration::from_secs(5);

/// Why a QMP exchange failed.
#[derive(Debug)]
pub enum QmpError {
    /// The socket failed, closed, or timed out.
    Io(io::Error),
    /// QEMU sent something that is not QMP.
    Protocol(String),
    /// QEMU understood the command and refused it.
    Command {
        /// QEMU's error class (`GenericError`, `CommandNotFound`, ...).
        class: String,
        /// QEMU's description.
        desc: String,
    },
}

impl fmt::Display for QmpError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io(e) => write!(f, "QMP socket: {e}"),
            Self::Protocol(m) => write!(f, "QMP protocol: {m}"),
            Self::Command { class, desc } => write!(f, "QMP command failed ({class}): {desc}"),
        }
    }
}

impl std::error::Error for QmpError {}

impl From<io::Error> for QmpError {
    fn from(e: io::Error) -> Self {
        Self::Io(e)
    }
}

impl From<QmpError> for io::Error {
    fn from(e: QmpError) -> Self {
        match e {
            QmpError::Io(e) => e,
            other => io::Error::other(other),
        }
    }
}

/// The guest's run state as QEMU reports it (`query-status`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RunState {
    /// Whether the vCPUs are executing.
    pub running: bool,
    /// QEMU's status word (`running`, `paused`, `prelaunch`, `shutdown`, ...).
    pub status: String,
}

/// One QMP connection.
///
/// Commands are synchronous: [`execute`](Self::execute) writes one request
/// and reads until its reply, setting aside any events that arrive first
/// ([`take_events`](Self::take_events)).
pub struct Qmp {
    writer: UnixStream,
    reader: BufReader<UnixStream>,
    events: Vec<Value>,
    /// Id of the last command sent. Every command carries one, so a reply
    /// that arrives after its command timed out is recognised and skipped
    /// instead of being taken for the answer to the next one.
    last_id: u64,
}

impl fmt::Debug for Qmp {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Qmp")
            .field("pending_events", &self.events.len())
            .finish()
    }
}

impl Qmp {
    /// Connect to a QMP unix socket and complete the capabilities handshake.
    pub fn connect(path: &Path) -> Result<Self, QmpError> {
        Self::connect_with_timeout(path, DEFAULT_TIMEOUT)
    }

    /// [`connect`](Self::connect) with an explicit per-operation timeout.
    pub fn connect_with_timeout(path: &Path, timeout: Duration) -> Result<Self, QmpError> {
        let stream = UnixStream::connect(path)?;
        Self::from_stream(stream, timeout)
    }

    /// Speak QMP over an already-connected stream (a test double, a socketpair).
    ///
    /// Reads the greeting and negotiates capabilities before returning.
    pub fn from_stream(stream: UnixStream, timeout: Duration) -> Result<Self, QmpError> {
        stream.set_read_timeout(Some(timeout))?;
        stream.set_write_timeout(Some(timeout))?;
        let reader = BufReader::new(stream.try_clone()?);
        let mut qmp = Self {
            writer: stream,
            reader,
            events: Vec::new(),
            last_id: 0,
        };
        let greeting = qmp.read_object()?;
        if greeting.get("QMP").is_none() {
            return Err(QmpError::Protocol(format!(
                "expected a QMP greeting, got {greeting}"
            )));
        }
        qmp.execute("qmp_capabilities", None)?;
        Ok(qmp)
    }

    /// Execute one command and return its `return` value.
    ///
    /// A reply to an *earlier* command (one whose wait timed out) is skipped;
    /// a reply without an id (a test double) is accepted as is.
    pub fn execute(&mut self, command: &str, arguments: Option<Value>) -> Result<Value, QmpError> {
        self.last_id += 1;
        let id = self.last_id;
        let mut request = json!({ "execute": command, "id": id });
        if let Some(arguments) = arguments {
            request["arguments"] = arguments;
        }
        let mut line = serde_json::to_string(&request)
            .map_err(|e| QmpError::Protocol(format!("encoding {command}: {e}")))?;
        line.push('\n');
        self.writer.write_all(line.as_bytes())?;
        loop {
            let reply = self.read_object()?;
            if reply.get("event").is_some() {
                if self.events.len() >= MAX_RETAINED_EVENTS {
                    self.events.remove(0);
                }
                self.events.push(reply);
                continue;
            }
            if let Some(got) = reply.get("id").and_then(Value::as_u64) {
                if got != id {
                    tracing::debug!(command, expected = id, got, "QMP: skipping a late reply");
                    continue;
                }
            }
            if let Some(value) = reply.get("return") {
                return Ok(value.clone());
            }
            if let Some(error) = reply.get("error") {
                let field = |k: &str| {
                    error
                        .get(k)
                        .and_then(Value::as_str)
                        .unwrap_or("")
                        .to_string()
                };
                return Err(QmpError::Command {
                    class: field("class"),
                    desc: field("desc"),
                });
            }
            return Err(QmpError::Protocol(format!(
                "unexpected QMP object: {reply}"
            )));
        }
    }

    /// Freeze the guest: vCPUs stop, and so does the guest's clock.
    pub fn stop(&mut self) -> Result<(), QmpError> {
        self.execute("stop", None).map(drop)
    }

    /// Thaw the guest.
    pub fn cont(&mut self) -> Result<(), QmpError> {
        self.execute("cont", None).map(drop)
    }

    /// Ask QEMU for its run state.
    pub fn query_status(&mut self) -> Result<RunState, QmpError> {
        let v = self.execute("query-status", None)?;
        Ok(RunState {
            running: v.get("running").and_then(Value::as_bool).unwrap_or(false),
            status: v
                .get("status")
                .and_then(Value::as_str)
                .unwrap_or("")
                .to_string(),
        })
    }

    /// Ask QEMU to exit. QEMU acknowledges and closes the socket; a socket
    /// that closes before the acknowledgement arrives is still a success.
    pub fn quit(&mut self) -> Result<(), QmpError> {
        match self.execute("quit", None) {
            Ok(_) => Ok(()),
            Err(QmpError::Io(e)) if e.kind() == io::ErrorKind::UnexpectedEof => Ok(()),
            Err(e) => Err(e),
        }
    }

    /// Run a line of the human monitor (HMP) and return its text output —
    /// for the few operations QMP has no command for.
    pub fn hmp(&mut self, command_line: &str) -> Result<String, QmpError> {
        let v = self.execute(
            "human-monitor-command",
            Some(json!({ "command-line": command_line })),
        )?;
        Ok(v.as_str().unwrap_or("").to_string())
    }

    /// Events QEMU sent since the last call (`STOP`, `RESUME`, `SHUTDOWN`, ...),
    /// the newest [`MAX_RETAINED_EVENTS`] of them.
    pub fn take_events(&mut self) -> Vec<Value> {
        std::mem::take(&mut self.events)
    }

    fn read_object(&mut self) -> Result<Value, QmpError> {
        let mut line = String::new();
        loop {
            line.clear();
            let n = self.reader.read_line(&mut line)?;
            if n == 0 {
                return Err(QmpError::Io(io::Error::new(
                    io::ErrorKind::UnexpectedEof,
                    "QMP socket closed",
                )));
            }
            if line.trim().is_empty() {
                continue;
            }
            return serde_json::from_str(&line)
                .map_err(|e| QmpError::Protocol(format!("{e}: {}", line.trim())));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Read;
    use std::thread;

    /// A QEMU that only knows how to greet, negotiate, and answer `stop`.
    fn fake_qemu(
        mut sock: UnixStream,
        replies: &'static [&'static str],
    ) -> thread::JoinHandle<Vec<String>> {
        thread::spawn(move || {
            sock.write_all(b"{\"QMP\": {\"version\": {}, \"capabilities\": []}}\n")
                .unwrap();
            let mut seen = Vec::new();
            let mut reader = BufReader::new(sock.try_clone().unwrap());
            let mut line = String::new();
            for reply in replies {
                line.clear();
                reader.read_line(&mut line).unwrap();
                seen.push(line.trim().to_string());
                sock.write_all(reply.as_bytes()).unwrap();
            }
            // Drain until the client hangs up so the writer never sees EPIPE.
            let mut rest = Vec::new();
            let _ = sock.read_to_end(&mut rest);
            seen
        })
    }

    #[test]
    fn handshake_then_stop_skips_events_and_returns_the_reply() {
        let (client, server) = UnixStream::pair().unwrap();
        let fake = fake_qemu(
            server,
            &[
                "{\"return\": {}}\n",
                "{\"event\": \"STOP\", \"timestamp\": {}}\n{\"return\": {}}\n",
            ],
        );
        let mut qmp = Qmp::from_stream(client, Duration::from_secs(2)).unwrap();
        qmp.stop().unwrap();
        assert_eq!(
            qmp.take_events().len(),
            1,
            "the STOP event was set aside, not lost"
        );
        drop(qmp);
        let seen = fake.join().unwrap();
        assert_eq!(seen[0], "{\"execute\":\"qmp_capabilities\",\"id\":1}");
        assert_eq!(seen[1], "{\"execute\":\"stop\",\"id\":2}");
    }

    #[test]
    fn a_command_error_is_reported_with_qemus_class_and_description() {
        let (client, server) = UnixStream::pair().unwrap();
        let _fake = fake_qemu(
            server,
            &[
                "{\"return\": {}}\n",
                "{\"error\": {\"class\": \"CommandNotFound\", \"desc\": \"The command nope has not been found\"}}\n",
            ],
        );
        let mut qmp = Qmp::from_stream(client, Duration::from_secs(2)).unwrap();
        match qmp.execute("nope", None) {
            Err(QmpError::Command { class, desc }) => {
                assert_eq!(class, "CommandNotFound");
                assert!(desc.contains("nope"));
            }
            other => panic!("expected a command error, got {other:?}"),
        }
    }

    #[test]
    fn a_late_reply_to_an_earlier_command_is_skipped() {
        let (client, server) = UnixStream::pair().unwrap();
        let _fake = fake_qemu(
            server,
            &[
                "{\"return\": {}, \"id\": 1}\n",
                // A reply tagged with the capabilities command's id arrives
                // first (as it would after a timeout); the real one follows.
                "{\"return\": {\"stale\": true}, \"id\": 1}\n{\"return\": {\"fresh\": true}, \"id\": 2}\n",
            ],
        );
        let mut qmp = Qmp::from_stream(client, Duration::from_secs(2)).unwrap();
        let v = qmp.execute("stop", None).unwrap();
        assert_eq!(v, serde_json::json!({ "fresh": true }));
    }

    #[test]
    fn a_non_qmp_greeting_is_a_protocol_error() {
        let (client, mut server) = UnixStream::pair().unwrap();
        server.write_all(b"hello\n").unwrap();
        match Qmp::from_stream(client, Duration::from_secs(2)) {
            Err(QmpError::Protocol(_)) => {}
            other => panic!("expected a protocol error, got {other:?}"),
        }
    }
}
