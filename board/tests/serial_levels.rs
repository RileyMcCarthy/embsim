//! Serial carried as **levels**, end to end
//! (`docs/dev/sil-lossless-net-transport.md`, step 3).
//!
//! The byte path routes a UART byte as a byte: the net decides who is
//! connected, but the payload never becomes a level, so it cannot be corrupted
//! by a fighting driver. These tests wire an [`McuComponent`] built with
//! [`McuBuilder::serial_on_levels`] to a peer that speaks only edges, and check
//! the three things the byte path could not show:
//!
//! 1. a firmware byte appears on the wire as a real waveform, at the table baud;
//! 2. a waveform a peer drives is readable as a byte on the firmware side;
//! 3. put a second driver on that wire and the byte **breaks** — the receiver
//!    reports bad framing instead of quietly receiving it.
//!
//! The peer frames and deframes with the public [`embsim_board::uart`] codec
//! but is otherwise an independent implementation: it does not share the MCU's
//! bridge, so a bug there cannot cancel itself out.

use rstest::rstest;
use std::collections::VecDeque;
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};

use embsim_board::event_log::EngineEvent;
use embsim_board::mcu::SerialChannelConfig;
use embsim_board::uart::{FramingError, UartDecoder, UartEncoder, UartFraming};
use embsim_board::{
    AttachError, Board, Component, ComponentNetIo, Harness, Level, McuComponent, NetState,
    PartRegistry, PinDecl, PinHandle, PinKind, StreamRole, System, SystemHandle, TheveninDrive,
};
use embsim_core::virtual_clock;
use embsim_peripherals::serial;

mod uart_probe;
use uart_probe::{ProbeHandle, UartProbe};

/// The reference consumer's force-gauge channel: RX on P0, TX on P2,
/// 115.2 kbaud.
const FG: SerialChannelConfig = SerialChannelConfig {
    rx_pin: 0,
    tx_pin: 2,
    baud: 115_200,
};

/// Push-pull rails, matching the engine's own digital defaults.
const HIGH: TheveninDrive = TheveninDrive {
    volts: 3.3,
    impedance: 25.0,
};
const LOW: TheveninDrive = TheveninDrive {
    volts: 0.0,
    impedance: 25.0,
};

/// The process-default peripheral bank and the virtual clock are global, so
/// the live tests in this file run one at a time.
static CLOCK_LOCK: Mutex<()> = Mutex::new(());

fn lock_clock() -> MutexGuard<'static, ()> {
    CLOCK_LOCK.lock().unwrap_or_else(|poisoned| {
        CLOCK_LOCK.clear_poison();
        poisoned.into_inner()
    })
}

fn wait_for(mut pred: impl FnMut() -> bool, timeout: Duration) -> bool {
    let start = Instant::now();
    while start.elapsed() < timeout {
        if pred() {
            return true;
        }
        std::thread::sleep(Duration::from_millis(1));
    }
    pred()
}

/// The logic level a resolved net presents, if it presents one. A contended or
/// floating net has none — the engine never invents one, and neither does a
/// receiver.
fn level_of(state: NetState) -> Option<Level> {
    match state {
        NetState::Driven(level) | NetState::Pulled(level, _) => Some(level),
        NetState::Analog(volts) => Some(if volts >= 1.5 {
            Level::High
        } else {
            Level::Low
        }),
        NetState::Floating | NetState::Contention => None,
    }
}

// ============================================================
// Netlists
// ============================================================

const MCU_NETLIST: &str = r#"(export (version "E")
  (components
    (comp (ref "U1")
      (value "P2")
      (libsource (lib "test") (part "MCU_P2") (description "")))
    (comp (ref "J1")
      (value "Conn_01x02")
      (libsource (lib "Connector_Generic") (part "Conn_01x02") (description ""))))
  (nets
    (net (code "1") (name "MCU_TX") (class "Default")
      (node (ref "U1") (pin "P2") (pintype "output"))
      (node (ref "J1") (pin "1") (pintype "passive")))
    (net (code "2") (name "MCU_RX") (class "Default")
      (node (ref "U1") (pin "P0") (pintype "input"))
      (node (ref "J1") (pin "2") (pintype "passive")))))"#;

const PEER_NETLIST: &str = r#"(export (version "E")
  (components
    (comp (ref "U1")
      (value "Peer")
      (libsource (lib "test") (part "PEER_UART") (description "")))
    (comp (ref "J1")
      (value "Conn_01x02")
      (libsource (lib "Connector_Generic") (part "Conn_01x02") (description ""))))
  (nets
    (net (code "1") (name "PEER_TX") (class "Default")
      (node (ref "U1") (pin "1") (pinfunction "TX") (pintype "output"))
      (node (ref "J1") (pin "1") (pintype "passive")))
    (net (code "2") (name "PEER_RX") (class "Default")
      (node (ref "U1") (pin "2") (pinfunction "RX") (pintype "input"))
      (node (ref "J1") (pin "2") (pintype "passive")))))"#;

/// The same peer board with a second driver hung off the wire the MCU
/// transmits on — a stuck-low bus fault, or a peer that thinks it owns the
/// line.
const CONTENDED_PEER_NETLIST: &str = r#"(export (version "E")
  (components
    (comp (ref "U1")
      (value "Peer")
      (libsource (lib "test") (part "PEER_UART") (description "")))
    (comp (ref "U2")
      (value "Contender")
      (libsource (lib "test") (part "STUCK_LOW") (description "")))
    (comp (ref "J1")
      (value "Conn_01x02")
      (libsource (lib "Connector_Generic") (part "Conn_01x02") (description ""))))
  (nets
    (net (code "1") (name "PEER_TX") (class "Default")
      (node (ref "U1") (pin "1") (pinfunction "TX") (pintype "output"))
      (node (ref "J1") (pin "1") (pintype "passive")))
    (net (code "2") (name "PEER_RX") (class "Default")
      (node (ref "U1") (pin "2") (pinfunction "RX") (pintype "input"))
      (node (ref "U2") (pin "1") (pintype "output"))
      (node (ref "J1") (pin "2") (pintype "passive")))))"#;

// ============================================================
// A peer that speaks only edges
// ============================================================

/// What the peer observed and what it wants to say, shared with the test.
#[derive(Debug, Default)]
struct PeerState {
    /// Every transition the peer saw on its RX pin, as `(level, ns)`.
    seen: Vec<(Level, u64)>,
    /// Frames the peer decoded, good and bad.
    frames: Vec<Result<u8, FramingError>>,
    /// Bytes queued for transmission.
    outbox: VecDeque<u8>,
    /// Levels of the frame being clocked out.
    bits: VecDeque<Level>,
    /// Instant the next TX bit is driven at.
    next_edge_ns: Option<u64>,
    /// Filled at attach, so the test can kick a transmission.
    io: Option<ComponentNetIo>,
}

#[derive(Debug, Clone)]
struct Peer(Arc<Mutex<PeerState>>);

impl Peer {
    fn new() -> Self {
        Self(Arc::new(Mutex::new(PeerState::default())))
    }

    fn lock(&self) -> MutexGuard<'_, PeerState> {
        self.0.lock().expect("peer state never poisoned")
    }

    /// Queue bytes and wake the peer to start clocking them out.
    fn send(&self, bytes: &[u8]) {
        let io = {
            let mut state = self.lock();
            state.outbox.extend(bytes);
            if state.next_edge_ns.is_none() {
                state.next_edge_ns = Some(virtual_clock::virtual_ns());
            }
            state.io.clone()
        };
        if let Some(io) = io {
            io.schedule_at_ns(virtual_clock::virtual_ns());
        }
    }

    fn decoded_bytes(&self) -> Vec<u8> {
        self.lock()
            .frames
            .iter()
            .filter_map(|f| f.as_ref().ok().copied())
            .collect()
    }
}

/// A UART with no stream role at all: two plain digital pins and a codec.
struct PeerUart {
    pins: [PinDecl; 2],
    framing: UartFraming,
    state: Peer,
}

impl PeerUart {
    fn new(framing: UartFraming, state: Peer) -> Self {
        Self {
            pins: [
                PinDecl {
                    number: "1",
                    name: Some("TX"),
                    kind: PinKind::DigitalOut,
                    stream: None,
                    drive_impedance: None,
                },
                PinDecl {
                    number: "2",
                    name: Some("RX"),
                    kind: PinKind::DigitalIn,
                    stream: None,
                    drive_impedance: None,
                },
            ],
            framing,
            state,
        }
    }
}

impl Component for PeerUart {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let tx_pin: PinHandle = io.pin("TX")?;
        let framing = self.framing;
        let encoder = UartEncoder::new(framing);
        let decoder = Arc::new(Mutex::new(UartDecoder::new(framing)));

        tx_pin.set_drive(Some(HIGH)); // an idle asynchronous line still drives
        self.state.lock().io = Some(io.clone());

        // Receive: every transition, stamped, then whatever it completes.
        {
            let state = self.state.clone();
            let decoder = Arc::clone(&decoder);
            let io = io.clone();
            io.clone().on_sense("RX", move |net_state| {
                let now = virtual_clock::virtual_ns();
                let mut rx = decoder.lock().expect("decoder never poisoned");
                if let Some(level) = level_of(net_state) {
                    state.lock().seen.push((level, now));
                    rx.on_level(level, now);
                }
                while let Some(frame) = rx.poll(now) {
                    state.lock().frames.push(frame);
                }
                if let Some(at) = rx.frame_deadline_ns() {
                    io.schedule_at_ns(at);
                }
            })?;
        }

        // Transmit, and close a receive frame whose tail carried no edge.
        {
            let state = self.state.clone();
            let decoder = Arc::clone(&decoder);
            let io = io.clone();
            io.clone().on_wake_ns(move |now| {
                let (level, next) = {
                    let mut peer = state.lock();
                    match peer.next_edge_ns {
                        // Not this bit yet: leave the line where it is.
                        Some(due) if due > now => (None, Some(due)),
                        Some(due) => {
                            if peer.bits.is_empty() {
                                if let Some(byte) = peer.outbox.pop_front() {
                                    peer.bits =
                                        encoder.encode(byte).into_iter().map(|(l, _)| l).collect();
                                }
                            }
                            match peer.bits.pop_front() {
                                Some(level) => {
                                    let next = due + framing.bit_period_ns;
                                    peer.next_edge_ns = Some(next);
                                    (Some(level), Some(next))
                                }
                                // Nothing left to send: park at idle.
                                None => {
                                    peer.next_edge_ns = None;
                                    (Some(Level::High), None)
                                }
                            }
                        }
                        None => (None, None),
                    }
                };
                if let Some(level) = level {
                    tx_pin.set_drive(Some(if level == Level::High { HIGH } else { LOW }));
                }

                let mut rx = decoder.lock().expect("decoder never poisoned");
                while let Some(frame) = rx.poll(now) {
                    state.lock().frames.push(frame);
                }
                let next = match (next, rx.frame_deadline_ns()) {
                    (Some(a), Some(b)) => Some(a.min(b)),
                    (a, b) => a.or(b),
                };
                if let Some(at) = next {
                    io.schedule_at_ns(at);
                }
            });
        }
        Ok(())
    }
}

/// A second driver holding the wire low: a bus fault, in one component.
struct StuckLow {
    pins: [PinDecl; 1],
}

impl StuckLow {
    fn new() -> Self {
        Self {
            pins: [PinDecl {
                number: "1",
                name: None,
                kind: PinKind::DigitalOut,
                stream: None,
                drive_impedance: None,
            }],
        }
    }
}

impl Component for StuckLow {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        io.pin("1")?.set_drive(Some(LOW));
        Ok(())
    }
}

// ============================================================
// Fixture assembly
// ============================================================

fn start_system(peer: &Peer, peer_netlist: &str) -> SystemHandle {
    let mut registry = PartRegistry::new();
    registry.register("MCU_P2", |_decl| {
        Box::new(
            McuComponent::builder("p2")
                .serial_table(vec![FG])
                .bridge_serial(0)
                .serial_on_levels()
                .build()
                .expect("MCU builds from the FG table"),
        )
    });
    {
        let peer = peer.clone();
        registry.register("PEER_UART", move |_decl| {
            Box::new(PeerUart::new(UartFraming::new_8n1(FG.baud), peer.clone()))
        });
    }
    registry.register("STUCK_LOW", |_decl| Box::new(StuckLow::new()));

    let mcu_board = Board::from_netlist(
        embsim_board::netlist::parse(MCU_NETLIST).expect("MCU netlist parses"),
        &registry,
    )
    .expect("MCU board builds");
    let peer_board = Board::from_netlist(
        embsim_board::netlist::parse(peer_netlist).expect("peer netlist parses"),
        &registry,
    )
    .expect("peer board builds");

    System::new()
        .board("McuBoard", mcu_board)
        .board("PeerBoard", peer_board)
        .harness(
            Harness::new()
                .connect_str("McuBoard.J1.1", "PeerBoard.J1.2")
                .expect("endpoints parse")
                .connect_str("PeerBoard.J1.1", "McuBoard.J1.2")
                .expect("endpoints parse"),
        )
        .start()
        .expect("live system starts")
}

/// Two [`UartProbe`]s wired TX↔RX through a harness, each on its own board.
///
/// Deliberately the *shared* bridge on both ends: this is a cost test, and
/// what it measures is how many wheel entries one duplex exchange creates.
fn start_probe_pair(a: &ProbeHandle, b: &ProbeHandle) -> SystemHandle {
    let framing = UartFraming::new_8n1(FG.baud);
    let mut registry = PartRegistry::new();
    {
        let a = a.clone();
        registry.register("PEER_UART", move |_decl| {
            Box::new(UartProbe::new(
                "1",
                Some("TX"),
                "2",
                Some("RX"),
                framing,
                a.clone(),
            ))
        });
    }
    let board_a = Board::from_netlist(
        embsim_board::netlist::parse(PEER_NETLIST).expect("peer netlist parses"),
        &registry,
    )
    .expect("board A builds");

    let mut registry_b = PartRegistry::new();
    {
        let b = b.clone();
        registry_b.register("PEER_UART", move |_decl| {
            Box::new(UartProbe::new(
                "1",
                Some("TX"),
                "2",
                Some("RX"),
                framing,
                b.clone(),
            ))
        });
    }
    let board_b = Board::from_netlist(
        embsim_board::netlist::parse(PEER_NETLIST).expect("peer netlist parses"),
        &registry_b,
    )
    .expect("board B builds");

    System::new()
        .board("A", board_a)
        .board("B", board_b)
        .harness(
            Harness::new()
                .connect_str("A.J1.1", "B.J1.2")
                .expect("endpoints parse")
                .connect_str("B.J1.1", "A.J1.2")
                .expect("endpoints parse"),
        )
        .event_log()
        .start()
        .expect("probe pair starts")
}

// ============================================================
// Tests
// ============================================================

/// On levels a serial pin carries no stream role at all: the framing lives in
/// the component, not in the route.
#[rstest]
fn a_level_carried_channel_declares_plain_digital_pins() {
    let mcu = McuComponent::builder("p2")
        .serial_table(vec![FG])
        .bridge_serial(0)
        .serial_on_levels()
        .build()
        .expect("builds");

    let pins = mcu.pins();
    let tx = pins.iter().find(|p| p.number == "P2").expect("P2 declared");
    assert_eq!(tx.kind, PinKind::DigitalOut);
    assert_eq!(tx.stream, None, "TX carries edges, not a byte route");

    let rx = pins.iter().find(|p| p.number == "P0").expect("P0 declared");
    assert_eq!(rx.kind, PinKind::DigitalIn);
    assert_eq!(rx.stream, None, "RX reads edges, not routed bytes");

    // The byte path is unchanged when the flag is off.
    let bytes = McuComponent::builder("p2")
        .serial_table(vec![FG])
        .bridge_serial(0)
        .build()
        .expect("builds");
    assert_eq!(
        bytes
            .pins()
            .iter()
            .find(|p| p.number == "P2")
            .expect("P2")
            .stream,
        Some(StreamRole::Producer { baud_hz: 115_200 })
    );
}

/// A firmware byte becomes a real waveform: the peer sees the edges, at the
/// table's bit period, and decodes the byte back.
#[rstest]
fn a_firmware_byte_crosses_the_net_as_edges() {
    let _g = lock_clock();
    virtual_clock::init(0.0, 1_000_000);
    serial::init(1);

    let peer = Peer::new();
    let system = start_system(&peer, PEER_NETLIST);

    // 0x5A alternates, so its waveform has an edge at almost every bit — the
    // byte that shows the most of the wire.
    serial::transmit_data(0, &[0x5A]);
    assert!(
        wait_for(|| peer.decoded_bytes() == [0x5A], Duration::from_secs(5)),
        "the peer must decode the firmware's byte; got {:?}",
        peer.lock().frames
    );

    // Every transition after the start bit lands on the bit grid the table
    // baud defines. This is the assertion the byte path cannot make: there
    // were no edges to check.
    let bit = UartFraming::new_8n1(FG.baud).bit_period_ns;
    let seen = peer.lock().seen.clone();
    // [0] is the line coming up to idle at attach; the rest are the frame.
    let frame = &seen[1..];

    // 0x5A on the wire, LSB first: start L, then 0 1 0 1 1 0 1 0, then stop H.
    // Eight of those nine boundaries are a change of level -- only d3→d4 is
    // not -- so a correct waveform is exactly eight transitions. Counting them
    // is what proves the bits are really there and not a byte in disguise.
    assert_eq!(
        frame.len(),
        8,
        "0x5A should put eight transitions on the wire; got {seen:?}"
    );
    assert_eq!(frame[0].0, Level::Low, "a frame opens with a start bit");

    let start = frame[0].1;
    for &(_, at) in &frame[1..] {
        assert_eq!(
            (at - start) % bit,
            0,
            "edge at {at} is off the {bit} ns bit grid anchored at {start}: {seen:?}"
        );
    }
    assert_eq!(
        frame[frame.len() - 1].1 - start,
        9 * bit,
        "the last edge is the stop bit, nine bit times after the start bit"
    );

    drop(system);
    serial::reset();
}

/// A waveform a peer drives is readable as a byte on the firmware side.
#[rstest]
fn a_peer_waveform_is_readable_as_a_firmware_byte() {
    let _g = lock_clock();
    virtual_clock::init(0.0, 1_000_000);
    serial::init(1);

    let peer = Peer::new();
    let system = start_system(&peer, PEER_NETLIST);

    // 0x00 and 0xFF are the frames with no data-bit transitions at all, so a
    // receiver that counted notifications instead of intervals would lose them.
    peer.send(&[0x00, 0x41, 0xFF]);

    let mut got: Vec<u8> = Vec::new();
    assert!(
        wait_for(
            || {
                while let Some(byte) = serial::receive_byte(0) {
                    got.push(byte);
                }
                got == [0x00, 0x41, 0xFF]
            },
            Duration::from_secs(5)
        ),
        "peer edges must arrive as firmware bytes, in wire order; got {got:?}"
    );

    drop(system);
    serial::reset();
}

/// Duplex traffic must cost a **linear** number of engine wakes.
///
/// Both directions arm the wheel and both re-arm from the same handler, and
/// [`UartDecoder::frame_deadline_ns`] is the same absolute instant for every
/// transition inside a frame — so an un-deduplicated bridge pushes one wheel
/// entry per *edge* received, and while the other direction is busy none of
/// them retire: each duplicate fires, computes the same next instant, and
/// re-arms. Measured before the fix, a duplex burst cost 4x the wakes for 2x
/// the payload, and the engine stopped making progress well before a
/// kilobyte.
///
/// Half-duplex bursts with idle gaps — which is every other test here — hide
/// it completely, because an idle direction's `service` returns nothing to
/// arm and the duplicates drain away.
#[rstest]
fn duplex_traffic_costs_a_linear_number_of_wakes() {
    let _g = lock_clock();
    virtual_clock::init(0.0, 1_000_000);

    let wakes = |payload: usize| -> usize {
        let (a, b) = (ProbeHandle::new(), ProbeHandle::new());
        let system = start_probe_pair(&a, &b);
        let bytes: Vec<u8> = (0..payload).map(|i| i as u8).collect();
        a.send(&bytes);
        b.send(&bytes);
        assert!(
            wait_for(
                || a.received().len() >= payload && b.received().len() >= payload,
                Duration::from_secs(20)
            ),
            "both directions must complete; a={} b={}",
            a.received().len(),
            b.received().len()
        );
        let count = system
            .event_log()
            .records()
            .iter()
            .filter(|r| matches!(r.event, EngineEvent::Wake { .. }))
            .count();
        drop(system);
        count
    };

    let small = wakes(8);
    let large = wakes(16);
    assert!(
        large < small * 3,
        "doubling a duplex payload must not multiply the wake count: \
         8 bytes cost {small} wakes, 16 bytes cost {large} \
         (quadratic growth is ~4x, linear is ~2x)"
    );
}

/// The point of the whole exercise: a second driver on the wire **breaks the
/// byte**. On the byte path this was unrepresentable — the payload never
/// touched a level, so nothing could fight it.
#[rstest]
fn a_second_driver_on_the_wire_breaks_the_byte() {
    let _g = lock_clock();
    virtual_clock::init(0.0, 1_000_000);
    serial::init(1);

    let peer = Peer::new();
    let system = start_system(&peer, CONTENDED_PEER_NETLIST);

    serial::transmit_data(0, &[0x5A]);
    assert!(
        wait_for(|| !peer.lock().frames.is_empty(), Duration::from_secs(5)),
        "the frame must resolve one way or the other"
    );

    let frames = peer.lock().frames.clone();
    assert!(
        frames.iter().all(|f| *f != Ok(0x5A)),
        "a contended wire must not deliver the byte intact; got {frames:?}"
    );
    assert!(
        frames.contains(&Err(FramingError::BadStopBit)),
        "the receiver should report bad framing, not silence; got {frames:?}"
    );

    drop(system);
    serial::reset();
}
