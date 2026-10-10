//! A small synchronous client for the Chrome DevTools Protocol.
//!
//! One WebSocket to the browser target, in flatten mode: every command and
//! event names its session, and a page's or a worker's commands go over the
//! same socket with that page's or worker's session id. Commands are numbered;
//! a reply is matched to its command by number, and an event that arrives
//! while a reply is awaited is kept, in order, for [`DevTools::next_event`].
//! Nothing here runs a thread: the node reads the socket only inside its own
//! slices, on the engine's thread, so what Chrome says between slices waits
//! in the socket until the next one.
//!
//! Also used by tests and harnesses that drive a page the node holds, as
//! Playwright's `connectOverCDP` would: [`browser_ws_url`] finds the
//! browser's socket from its DevTools HTTP endpoint.

use std::collections::{HashMap, VecDeque};
use std::fmt;
use std::io::{self, Read, Write};
use std::net::{TcpStream, ToSocketAddrs};
use std::time::{Duration, Instant};

use serde_json::{json, Value};
use tungstenite::{Message, WebSocket};

/// Why a DevTools exchange failed.
#[derive(Debug)]
pub enum CdpError {
    /// The socket failed, or the browser closed it.
    Io(io::Error),
    /// The browser answered the command with an error.
    Protocol {
        /// The command's method.
        method: String,
        /// The browser's message.
        message: String,
    },
    /// No answer before the deadline.
    Timeout {
        /// What was awaited.
        what: String,
        /// How long it was awaited, in host time.
        after: Duration,
    },
    /// The socket closed.
    Closed,
}

impl fmt::Display for CdpError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            CdpError::Io(e) => write!(f, "the DevTools socket failed: {e}"),
            CdpError::Protocol { method, message } => {
                write!(f, "Chrome refused {method}: {message}")
            }
            CdpError::Timeout { what, after } => write!(
                f,
                "{what} got no answer in {:.3} s of host time",
                after.as_secs_f64()
            ),
            CdpError::Closed => write!(f, "Chrome closed its DevTools socket"),
        }
    }
}

impl std::error::Error for CdpError {}

impl From<io::Error> for CdpError {
    fn from(e: io::Error) -> Self {
        CdpError::Io(e)
    }
}

fn ws_error(e: tungstenite::Error) -> CdpError {
    match e {
        tungstenite::Error::Io(e) => CdpError::Io(e),
        tungstenite::Error::ConnectionClosed | tungstenite::Error::AlreadyClosed => {
            CdpError::Closed
        }
        other => CdpError::Io(io::Error::other(other.to_string())),
    }
}

/// An event the browser sent: its session (`None` for the browser's own),
/// its method and its parameters.
#[derive(Debug, Clone)]
pub struct Event {
    /// The session the event belongs to; `None` for the browser target.
    pub session: Option<String>,
    /// `Domain.event`.
    pub method: String,
    /// The event's parameters.
    pub params: Value,
}

/// A command's answer, kept until it is asked for.
type Answer = Result<Value, (String, String)>;

/// One DevTools connection.
pub struct DevTools {
    socket: WebSocket<TcpStream>,
    next_id: u64,
    /// Events read while a reply was awaited, oldest first.
    events: VecDeque<Event>,
    /// Answers read before they were asked for, by command number.
    answers: HashMap<u64, Answer>,
    /// Commands whose answer nobody will ask for: dropped on arrival.
    forgotten: std::collections::HashSet<u64>,
    /// The method of each command still unanswered, for its error.
    methods: HashMap<u64, String>,
}

impl fmt::Debug for DevTools {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("DevTools")
            .field("next_id", &self.next_id)
            .field("queued_events", &self.events.len())
            .finish_non_exhaustive()
    }
}

impl DevTools {
    /// Connect to a DevTools WebSocket (`ws://127.0.0.1:9222/devtools/browser/…`).
    pub fn connect(ws_url: &str, timeout: Duration) -> Result<Self, CdpError> {
        let rest = ws_url.strip_prefix("ws://").ok_or_else(|| {
            CdpError::Io(io::Error::new(
                io::ErrorKind::InvalidInput,
                format!("{ws_url} is not a ws:// URL"),
            ))
        })?;
        let host = rest.split('/').next().unwrap_or(rest);
        let addr = host.to_socket_addrs()?.next().ok_or_else(|| {
            io::Error::new(io::ErrorKind::NotFound, format!("no address for {host}"))
        })?;
        let stream = TcpStream::connect_timeout(&addr, timeout)?;
        stream.set_nodelay(true)?;
        stream.set_read_timeout(Some(timeout))?;
        stream.set_write_timeout(Some(timeout))?;
        let (socket, _response) = tungstenite::client(ws_url, stream).map_err(|e| {
            CdpError::Io(io::Error::other(format!(
                "the WebSocket handshake failed: {e}"
            )))
        })?;
        Ok(Self {
            socket,
            next_id: 1,
            events: VecDeque::new(),
            answers: HashMap::new(),
            forgotten: Default::default(),
            methods: HashMap::new(),
        })
    }

    /// Send a command and return its number; [`Self::wait`] takes its answer.
    pub fn send(
        &mut self,
        session: Option<&str>,
        method: &str,
        params: Value,
    ) -> Result<u64, CdpError> {
        let id = self.next_id;
        self.next_id += 1;
        let mut message = json!({ "id": id, "method": method, "params": params });
        if let Some(session) = session {
            message["sessionId"] = Value::String(session.to_string());
        }
        self.methods.insert(id, method.to_string());
        self.socket
            .send(Message::text(message.to_string()))
            .map_err(ws_error)?;
        Ok(id)
    }

    /// Send a command whose answer nobody will wait for (a navigation that
    /// answers only once the page has lived some time).
    pub fn send_and_forget(
        &mut self,
        session: Option<&str>,
        method: &str,
        params: Value,
    ) -> Result<(), CdpError> {
        let id = self.send(session, method, params)?;
        if self.answers.remove(&id).is_none() {
            self.forgotten.insert(id);
        }
        Ok(())
    }

    /// Wait for command `id`'s answer until `deadline`, keeping the events
    /// that arrive meanwhile.
    pub fn wait(&mut self, id: u64, deadline: Instant) -> Result<Value, CdpError> {
        let started = Instant::now();
        loop {
            if let Some(answer) = self.answers.remove(&id) {
                let method = self.methods.remove(&id).unwrap_or_default();
                return answer.map_err(|(_, message)| CdpError::Protocol { method, message });
            }
            if !self.read_one(deadline)? {
                return Err(CdpError::Timeout {
                    what: format!(
                        "{} (command {id})",
                        self.methods
                            .get(&id)
                            .map(String::as_str)
                            .unwrap_or("a command")
                    ),
                    after: started.elapsed(),
                });
            }
        }
    }

    /// Send a command and wait up to `timeout` for its answer.
    pub fn call(
        &mut self,
        session: Option<&str>,
        method: &str,
        params: Value,
        timeout: Duration,
    ) -> Result<Value, CdpError> {
        let id = self.send(session, method, params)?;
        self.wait(id, Instant::now() + timeout)
    }

    /// Whether command `id` has been answered yet (its answer stays for
    /// [`Self::wait`]).
    pub fn answered(&self, id: u64) -> bool {
        self.answers.contains_key(&id)
    }

    /// The next event, read until `deadline`; `None` once it passes with
    /// none.
    pub fn next_event(&mut self, deadline: Instant) -> Result<Option<Event>, CdpError> {
        loop {
            if let Some(event) = self.events.pop_front() {
                return Ok(Some(event));
            }
            if !self.read_one(deadline)? {
                return Ok(None);
            }
        }
    }

    /// The next event already read or waiting in the socket, without
    /// blocking for one.
    pub fn poll_event(&mut self) -> Result<Option<Event>, CdpError> {
        self.next_event(Instant::now())
    }

    /// The oldest event already read, without reading the socket.
    pub fn pop_event(&mut self) -> Option<Event> {
        self.events.pop_front()
    }

    /// Read one message — an answer or an event — from the socket, waiting
    /// until `deadline` for it. `false` when the deadline passed first.
    pub fn read_message(&mut self, deadline: Instant) -> Result<bool, CdpError> {
        self.read_one(deadline)
    }

    /// Read one message into the queues. `false` when `deadline` passed
    /// with nothing read.
    fn read_one(&mut self, deadline: Instant) -> Result<bool, CdpError> {
        loop {
            let now = Instant::now();
            // A deadline already passed still takes what the socket holds:
            // the shortest timeout the socket accepts.
            let timeout = deadline
                .saturating_duration_since(now)
                .max(Duration::from_micros(1));
            self.socket.get_mut().set_read_timeout(Some(timeout))?;
            match self.socket.read() {
                Ok(Message::Text(text)) => {
                    self.take(text.as_str())?;
                    return Ok(true);
                }
                Ok(Message::Binary(bytes)) => {
                    let text = String::from_utf8_lossy(&bytes).into_owned();
                    self.take(&text)?;
                    return Ok(true);
                }
                Ok(Message::Close(_)) => return Err(CdpError::Closed),
                Ok(_) => continue,
                Err(tungstenite::Error::Io(e))
                    if matches!(
                        e.kind(),
                        io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
                    ) =>
                {
                    if Instant::now() >= deadline {
                        return Ok(false);
                    }
                }
                Err(tungstenite::Error::Io(e)) if e.kind() == io::ErrorKind::Interrupted => {}
                Err(e) => return Err(ws_error(e)),
            }
        }
    }

    /// File one message: an answer by its number, or an event.
    fn take(&mut self, text: &str) -> Result<(), CdpError> {
        let message: Value = serde_json::from_str(text).map_err(|e| {
            CdpError::Io(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("Chrome sent something that is not JSON ({e}): {text:.200}"),
            ))
        })?;
        if let Some(id) = message.get("id").and_then(Value::as_u64) {
            if self.forgotten.remove(&id) {
                self.methods.remove(&id);
                return Ok(());
            }
            let answer = match message.get("error") {
                Some(error) => Err((
                    error.get("code").map(|c| c.to_string()).unwrap_or_default(),
                    error
                        .get("message")
                        .and_then(Value::as_str)
                        .unwrap_or("an error with no message")
                        .to_string(),
                )),
                None => Ok(message.get("result").cloned().unwrap_or(Value::Null)),
            };
            self.answers.insert(id, answer);
            return Ok(());
        }
        let method = message
            .get("method")
            .and_then(Value::as_str)
            .unwrap_or_default()
            .to_string();
        self.events.push_back(Event {
            session: message
                .get("sessionId")
                .and_then(Value::as_str)
                .map(str::to_string),
            method,
            params: message.get("params").cloned().unwrap_or(Value::Null),
        });
        Ok(())
    }
}

/// The browser's DevTools WebSocket URL, read from its HTTP endpoint
/// (`http://127.0.0.1:9222`, or `127.0.0.1:9222`) at `/json/version`,
/// retried until `timeout` while nothing answers there yet.
pub fn browser_ws_url(endpoint: &str, timeout: Duration) -> io::Result<String> {
    let host = endpoint
        .trim_start_matches("http://")
        .trim_end_matches('/')
        .to_string();
    let deadline = Instant::now() + timeout;
    let mut last: io::Error;
    loop {
        match get_json_version(&host) {
            Ok(body) => {
                let version: Value = serde_json::from_str(&body).map_err(|e| {
                    io::Error::new(
                        io::ErrorKind::InvalidData,
                        format!("{endpoint}/json/version is not JSON ({e}): {body:.200}"),
                    )
                })?;
                return version
                    .get("webSocketDebuggerUrl")
                    .and_then(Value::as_str)
                    .map(str::to_string)
                    .ok_or_else(|| {
                        io::Error::new(
                            io::ErrorKind::InvalidData,
                            format!("{endpoint}/json/version names no webSocketDebuggerUrl"),
                        )
                    });
            }
            Err(e) => last = e,
        }
        if Instant::now() >= deadline {
            return Err(io::Error::new(
                last.kind(),
                format!(
                    "nothing answered DevTools at {endpoint} within {:.1} s: {last}",
                    timeout.as_secs_f64()
                ),
            ));
        }
        std::thread::sleep(Duration::from_millis(20));
    }
}

/// `GET /json/version` over a plain socket, the body returned.
fn get_json_version(host: &str) -> io::Result<String> {
    let addr = host
        .to_socket_addrs()?
        .next()
        .ok_or_else(|| io::Error::new(io::ErrorKind::NotFound, format!("no address for {host}")))?;
    let mut stream = TcpStream::connect_timeout(&addr, Duration::from_millis(500))?;
    stream.set_read_timeout(Some(Duration::from_secs(2)))?;
    write!(
        stream,
        "GET /json/version HTTP/1.1\r\nHost: {host}\r\nConnection: close\r\n\r\n"
    )?;
    // Chrome keeps the connection open whatever the request says, so the
    // body is read to its Content-Length, not to the end of the stream.
    let mut response = Vec::new();
    let mut chunk = [0u8; 4096];
    let (head_len, body_len) = loop {
        let n = stream.read(&mut chunk)?;
        if n == 0 {
            return Err(io::Error::new(
                io::ErrorKind::UnexpectedEof,
                "the DevTools endpoint closed before it answered",
            ));
        }
        response.extend_from_slice(&chunk[..n]);
        if let Some(end) = response.windows(4).position(|w| w == b"\r\n\r\n") {
            let head = String::from_utf8_lossy(&response[..end]).into_owned();
            if !head.starts_with("HTTP/1.1 200") && !head.starts_with("HTTP/1.0 200") {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    format!(
                        "/json/version answered {:.80}",
                        head.lines().next().unwrap_or("")
                    ),
                ));
            }
            let length = head
                .lines()
                .find_map(|line| {
                    let (name, value) = line.split_once(':')?;
                    name.trim()
                        .eq_ignore_ascii_case("content-length")
                        .then(|| value.trim().parse::<usize>().ok())
                        .flatten()
                })
                .ok_or_else(|| {
                    io::Error::new(
                        io::ErrorKind::InvalidData,
                        "an answer with no Content-Length",
                    )
                })?;
            break (end + 4, length);
        }
    };
    while response.len() < head_len + body_len {
        let n = stream.read(&mut chunk)?;
        if n == 0 {
            break;
        }
        response.extend_from_slice(&chunk[..n]);
    }
    let end = response.len().min(head_len + body_len);
    Ok(String::from_utf8_lossy(&response[head_len..end]).into_owned())
}
