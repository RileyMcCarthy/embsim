//! McuComponent end-to-end (`BOARD_ENGINE.md`, "The MCU as a component",
//! force-path slice): a two-board system in pure Rust — an [`McuComponent`]
//! with one bridged serial channel, harness-wired to a peer component with
//! mirrored Producer/Consumer stream pins.
//!
//! The test stands in for both sides of the real deployment:
//! - it plays the **runtime** by sizing the default peripheral serial bank
//!   (`serial::init`) before `System::start`, exactly as `Emulator::run`
//!   does before project wiring;
//! - it plays the **firmware** by moving bytes through the peripheral free
//!   functions (`serial::transmit_data` / `serial::receive_byte`) — the same
//!   calls the HAL trampolines make — so the whole bridged path
//!   firmware-side FD ⇄ pump ⇄ stream pins ⇄ nets ⇄ peer is exercised
//!   without any consumer firmware.
//!
//! Only one test may touch the process-default peripheral instance and the
//! process-global virtual clock; the fixture-shape tests stay pure.

use rstest::rstest;
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};

use embsim_board::mcu::SerialChannelConfig;
use embsim_board::uart::UartFraming;
use embsim_board::{Board, Harness, McuComponent, PartRegistry, PinKind, System};
use embsim_core::virtual_clock;
use embsim_peripherals::serial;

mod uart_probe;
use uart_probe::{ProbeHandle, UartProbe};

// ============================================================
// Fixture: MCU board + peer board + straight harness
// ============================================================

/// The reference consumer's force-gauge channel: RX on P0, TX on P2,
/// 115.2 kbaud — the same truth the cross-repo HAL-table test asserts.
const FG: SerialChannelConfig = SerialChannelConfig {
    rx_pin: 0,
    tx_pin: 2,
    baud: 115_200,
};

/// MCU board: the MCU's bridged UART broken out to a two-pin connector.
/// The netlist references the MCU's physical pins by their "P{n}" names.
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

/// Peer board: a UART device with mirrored stream roles behind its own
/// connector.
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

/// The process-default peripheral bank and the virtual clock are global, so
/// the live tests in this file run one at a time. (The header's "only one
/// test" note predates there being three of them; the lock is what makes that
/// true in practice.)
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

// ============================================================
// The end-to-end bridge test
// ============================================================

/// Firmware-side bytes cross the bridge to the peer and back, and dropping
/// the system joins the engine and the pump threads cleanly.
#[rstest]
fn bridged_serial_channel_roundtrips_to_a_peer_board() {
    let _g = lock_clock();
    // 50x scale keeps the 115.2 kbaud bit clock sub-millisecond in wall time.
    virtual_clock::init(50.0, 1_000_000);

    // The runtime's role: size the default instance's serial bank before
    // wiring (Emulator::run step 2). Channel 0 is the bridged FG channel.
    serial::init(1);

    let peer = ProbeHandle::new();

    let mut registry = PartRegistry::new();
    registry.register("MCU_P2", |_decl| {
        Box::new(
            McuComponent::builder("p2")
                .serial_table(vec![FG])
                .bridge_serial(0)
                .build()
                .expect("MCU builds from the FG table"),
        )
    });
    {
        let peer = peer.clone();
        registry.register("PEER_UART", move |_decl| {
            Box::new(UartProbe::new(
                "1",
                Some("TX"),
                "2",
                Some("RX"),
                UartFraming::new_8n1(FG.baud),
                peer.clone(),
            ))
        });
    }

    let mcu_board = Board::from_netlist(
        embsim_board::netlist::parse(MCU_NETLIST).expect("MCU netlist parses"),
        &registry,
    )
    .expect("MCU board builds");
    let peer_board = Board::from_netlist(
        embsim_board::netlist::parse(PEER_NETLIST).expect("peer netlist parses"),
        &registry,
    )
    .expect("peer board builds");

    // Straight harness: MCU TX → peer RX, peer TX → MCU RX.
    let harness = Harness::new()
        .connect_str("McuBoard.J1.1", "PeerBoard.J1.2")
        .expect("endpoints parse")
        .connect_str("PeerBoard.J1.1", "McuBoard.J1.2")
        .expect("endpoints parse");

    let system = System::new()
        .board("McuBoard", mcu_board)
        .board("PeerBoard", peer_board)
        .harness(harness)
        .start()
        .expect("live system starts");

    // Firmware → peer: transmit through the peripheral bank exactly as the
    // HAL trampoline would; the pump frames it onto the P2 TX pin, the edges
    // cross the harness, and the peer's decoder puts the byte back together.
    serial::transmit_data(0, &[0x55, 0x08, 0x02]);
    assert!(
        wait_for(
            || peer.received() == [0x55, 0x08, 0x02],
            Duration::from_secs(5)
        ),
        "firmware TX bytes must reach the peer; got {:?}",
        peer.frames()
    );

    // Peer → firmware: edges the peer drives arrive readable on the firmware
    // side of the bridged channel, in wire order.
    peer.send(b"OK");
    let mut got: Vec<u8> = Vec::new();
    assert!(
        wait_for(
            || {
                while let Some(byte) = serial::receive_byte(0) {
                    got.push(byte);
                }
                got == b"OK"
            },
            Duration::from_secs(5)
        ),
        "peer bytes must be readable on the firmware side; got {got:?}"
    );

    // Clean shutdown: the engine joins first (SystemHandle drop order),
    // then each pump thread — bounded, no detached-thread leak.
    let start = Instant::now();
    drop(system);
    assert!(
        start.elapsed() < Duration::from_secs(5),
        "shutdown must join the engine and the pumps promptly"
    );

    // The MCU disconnected its channel on drop: the bank reads nothing and
    // a late firmware-style transmit is a silent no-op, never a panic.
    assert_eq!(serial::receive_byte(0), None);
    serial::transmit_data(0, b"late");
    serial::reset();
}

// ============================================================
// Pin-table correctness from a sample config
// ============================================================

/// The FG channel's pin table: P0 is the MCU's RX pin and P2 its TX pin, both
/// plain digital — the UART is framed onto the net, so there is no byte route
/// to declare.
#[rstest]
fn fg_channel_pin_table_matches_the_hal_config() {
    let mcu = McuComponent::builder("p2")
        .serial_table(vec![FG])
        .bridge_serial(0)
        .build()
        .expect("builds");

    let pins = embsim_board::Component::pins(&mcu);
    assert_eq!(pins.len(), 2, "one bridged channel declares two pins");

    let tx = pins.iter().find(|p| p.number == "P2").expect("P2 declared");
    assert_eq!(tx.kind, PinKind::DigitalOut);
    assert_eq!(tx.stream, None, "TX clocks out edges, not a byte route");

    let rx = pins.iter().find(|p| p.number == "P0").expect("P0 declared");
    assert_eq!(rx.kind, PinKind::DigitalIn);
    assert_eq!(rx.stream, None, "RX reads edges, not routed bytes");
}

// ============================================================
// Owned-execution mode (the entry inversion)
// ============================================================

/// With an entry, the system spawns the firmware on a thread bound to the
/// component's OWN peripheral instance: HAL free functions called by the
/// entry route there (not the process default), the bank sizing the
/// firmware performs inside the entry does not sever the attach-installed
/// bridge (init/wiring commute), and the entry's bytes cross the netlist
/// to the peer. SystemHandle drop must not hang on the detached entry
/// thread.
#[rstest]
fn entry_runs_on_the_component_instance_and_reaches_the_peer() {
    use std::sync::atomic::{AtomicBool, Ordering};

    let _g = lock_clock();
    virtual_clock::init(50.0, 1_000_000);

    let peer = ProbeHandle::new();

    // Set by the entry thread; read by the test.
    static ROUTED_OFF_DEFAULT: AtomicBool = AtomicBool::new(false);
    static STOP: AtomicBool = AtomicBool::new(false);

    let mut registry = PartRegistry::new();
    registry.register("MCU_P2", |_decl| {
        Box::new(
            McuComponent::builder("p2-owned")
                .serial_table(vec![FG])
                .bridge_serial(0)
                .entry(|| {
                    // The inversion's core claim: this thread's instance is
                    // NOT the process default.
                    let mine = embsim_peripherals::instance::current();
                    let default = embsim_peripherals::instance::default();
                    ROUTED_OFF_DEFAULT.store(!Arc::ptr_eq(&mine, &default), Ordering::Relaxed);

                    // Firmware-style boot: size the bank INSIDE the entry —
                    // strictly after attach installed the bridge FD, which
                    // must survive (sizing and wiring commute).
                    serial::init(1);
                    serial::transmit_data(0, b"BOOT");

                    // A firmware main loop that never returns until the
                    // test releases it (the thread is detached by design).
                    while !STOP.load(Ordering::Relaxed) {
                        std::thread::sleep(Duration::from_millis(5));
                    }
                })
                .build()
                .expect("MCU builds with an entry"),
        )
    });
    {
        let peer = peer.clone();
        registry.register("PEER_UART", move |_decl| {
            Box::new(UartProbe::new(
                "1",
                Some("TX"),
                "2",
                Some("RX"),
                UartFraming::new_8n1(FG.baud),
                peer.clone(),
            ))
        });
    }

    let mcu_board = Board::from_netlist(
        embsim_board::netlist::parse(MCU_NETLIST).expect("MCU netlist parses"),
        &registry,
    )
    .expect("MCU board builds");
    let peer_board = Board::from_netlist(
        embsim_board::netlist::parse(PEER_NETLIST).expect("peer netlist parses"),
        &registry,
    )
    .expect("peer board builds");

    let harness = Harness::new()
        .connect_str("McuBoard.J1.1", "PeerBoard.J1.2")
        .expect("endpoints parse")
        .connect_str("PeerBoard.J1.1", "McuBoard.J1.2")
        .expect("endpoints parse");

    let system = System::new()
        .board("McuBoard", mcu_board)
        .board("PeerBoard", peer_board)
        .harness(harness)
        .start()
        .expect("live system starts");

    // The entry's boot bytes cross the bridge: proof its HAL calls landed
    // on the attached (component-owned) instance with the bridge intact.
    assert!(
        wait_for(|| peer.received() == b"BOOT", Duration::from_secs(5)),
        "entry bytes must reach the peer; got {:?}",
        peer.frames()
    );
    assert!(
        ROUTED_OFF_DEFAULT.load(Ordering::Relaxed),
        "the entry thread must be bound to the component's own instance, \
         not the process default"
    );

    // Dropping the system joins the engine and pumps; the still-running
    // entry thread is detached and must not block shutdown.
    let start = Instant::now();
    drop(system);
    assert!(
        start.elapsed() < Duration::from_secs(5),
        "shutdown must not wait on the detached entry thread"
    );
    STOP.store(true, Ordering::Relaxed);
}
