//! Bench components ([`System::component`]): bare components in a system
//! without a board netlist, addressed by bare `Name.Pin` harness endpoints —
//! the "bench rigs that aren't a designed PCB" case (`BOARD_ENGINE.md`,
//! "Boards, harnesses, systems, scenarios"; the reference consumer's P2-EVAL
//! rig is the motivating shape).
//!
//! Covered here:
//! - serial stream routing between two bench components over bare endpoints
//!   (the MCU-on-a-bench path), including alias-name endpoints;
//! - a live analog Thevenin drive on a bench pin sensed across the harness
//!   (the load-cell/transducer path);
//! - name-collision validation (bench vs board, bench vs bench).

use rstest::rstest;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use embsim_board::component::StreamTx;
use embsim_board::{
    AttachError, Board, Component, ComponentNetIo, Finding, Harness, NetState, PartRegistry,
    PinDecl, PinHandle, PinKind, StreamRole, System, SystemError, TheveninDrive,
};
use embsim_core::virtual_clock;

// ============================================================
// Shared plumbing
// ============================================================

type TxSlot = Arc<Mutex<Option<StreamTx>>>;
type ByteLog = Arc<Mutex<Vec<u8>>>;

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
// Fixture: a bench UART (stream pins with alias names)
// ============================================================

/// Bench UART probe: pin numbers "1"/"2" with "TX"/"RX" aliases, so bare
/// endpoints resolve by either identity.
struct BenchUart {
    pins: [PinDecl; 2],
    tx: TxSlot,
    rx: ByteLog,
}

impl BenchUart {
    fn new(baud_hz: u32, tx: TxSlot, rx: ByteLog) -> Self {
        Self {
            pins: [
                PinDecl {
                    number: "1",
                    name: Some("TX"),
                    kind: PinKind::DigitalOut,
                    stream: Some(StreamRole::Producer { baud_hz }),
                    drive_impedance: None,
                },
                PinDecl {
                    number: "2",
                    name: Some("RX"),
                    kind: PinKind::DigitalIn,
                    stream: Some(StreamRole::Consumer { baud_hz }),
                    drive_impedance: None,
                },
            ],
            tx,
            rx,
        }
    }
}

impl Component for BenchUart {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        *self.tx.lock().unwrap() = Some(io.stream_tx("TX")?);
        let log = Arc::clone(&self.rx);
        io.on_byte("RX", move |byte| log.lock().unwrap().push(byte))?;
        Ok(())
    }
}

// ============================================================
// Fixture: analog source + analog probe
// ============================================================

type PinSlot = Arc<Mutex<Option<PinHandle>>>;
type StateSlot = Arc<Mutex<Option<NetState>>>;

/// One live-drivable analog terminal (the transducer/load-cell shape).
struct AnalogSource {
    pins: [PinDecl; 1],
    handle: PinSlot,
}

impl Component for AnalogSource {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let pin = io.pin("S+")?;
        // A transducer presents its quiescent output immediately.
        pin.set_drive(Some(TheveninDrive {
            volts: 1.778,
            impedance: 350.0,
        }));
        *self.handle.lock().unwrap() = Some(pin);
        Ok(())
    }
}

/// One analog sense recording the last delivered net state.
struct AnalogProbe {
    pins: [PinDecl; 1],
    state: StateSlot,
}

impl Component for AnalogProbe {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let slot = Arc::clone(&self.state);
        io.on_sense("A", move |state| *slot.lock().unwrap() = Some(state))?;
        Ok(())
    }
}

const fn analog_pin(number: &'static str) -> PinDecl {
    PinDecl {
        number,
        name: None,
        kind: PinKind::Analog,
        stream: None,
        drive_impedance: None,
    }
}

// ============================================================
// Serial routing over bare endpoints
// ============================================================

/// Two bench UARTs cross-wired by bare (alias) endpoints roundtrip bytes in
/// both directions — the bench-MCU serial path with no board netlist at all.
#[rstest]
fn bench_uarts_roundtrip_over_bare_endpoints() {
    // Stream pacing samples the free-running virtual clock; 50x scale keeps
    // the 115.2 kbaud pacing sub-millisecond in wall time. This is the only
    // test in this binary that touches the process-global clock.
    virtual_clock::init(50.0, 1_000_000);

    let host_tx: TxSlot = Arc::new(Mutex::new(None));
    let host_rx: ByteLog = Arc::new(Mutex::new(Vec::new()));
    let dev_tx: TxSlot = Arc::new(Mutex::new(None));
    let dev_rx: ByteLog = Arc::new(Mutex::new(Vec::new()));

    let harness = Harness::new()
        .connect_str("HOST.TX", "DEV.RX")
        .expect("endpoints parse")
        .connect_str("DEV.TX", "HOST.RX")
        .expect("endpoints parse");

    let system = System::new()
        .component(
            "HOST",
            Box::new(BenchUart::new(
                115_200,
                Arc::clone(&host_tx),
                Arc::clone(&host_rx),
            )),
        )
        .component(
            "DEV",
            Box::new(BenchUart::new(
                115_200,
                Arc::clone(&dev_tx),
                Arc::clone(&dev_rx),
            )),
        )
        .harness(harness)
        .start()
        .expect("bench system starts");

    assert!(
        !system
            .findings()
            .iter()
            .any(|f| matches!(f, Finding::StreamMismatch { .. })),
        "straight bare-endpoint harness must route cleanly; got {:?}",
        system.findings()
    );

    let host = host_tx.lock().unwrap().clone().expect("host attached");
    host.write(&[0x55, 0x10]);
    assert!(
        wait_for(
            || dev_rx.lock().unwrap().as_slice() == [0x55, 0x10],
            Duration::from_secs(5)
        ),
        "host bytes must reach the device; got {:?}",
        dev_rx.lock().unwrap()
    );

    let dev = dev_tx.lock().unwrap().clone().expect("dev attached");
    dev.write(b"OK");
    assert!(
        wait_for(
            || host_rx.lock().unwrap().as_slice() == b"OK",
            Duration::from_secs(5)
        ),
        "device bytes must reach the host; got {:?}",
        host_rx.lock().unwrap()
    );

    system.shutdown();
}

// ============================================================
// Live analog drive across the harness
// ============================================================

/// A Thevenin drive set on a bench pin resolves numerically on the sensing
/// side of the harness, and a later `set_drive` propagates — the transducer
/// (load-cell) mechanism with no board netlist.
#[rstest]
fn live_analog_drive_is_sensed_across_the_harness() {
    let handle: PinSlot = Arc::new(Mutex::new(None));
    let state: StateSlot = Arc::new(Mutex::new(None));

    let system = System::new()
        .component(
            "CELL",
            Box::new(AnalogSource {
                pins: [analog_pin("S+")],
                handle: Arc::clone(&handle),
            }),
        )
        .component(
            "METER",
            Box::new(AnalogProbe {
                pins: [analog_pin("A")],
                state: Arc::clone(&state),
            }),
        )
        .harness(
            Harness::new()
                .connect_str("CELL.S+", "METER.A")
                .expect("endpoints parse"),
        )
        .start()
        .expect("bench system starts");

    let solved = |expect: f64| {
        let state = Arc::clone(&state);
        move || {
            matches!(
                *state.lock().unwrap(),
                Some(NetState::Analog(v)) if (v - expect).abs() < 1e-6
            )
        }
    };
    assert!(
        wait_for(solved(1.778), Duration::from_secs(5)),
        "attach-time quiescent drive must solve on the probe; got {:?}",
        state.lock().unwrap()
    );

    handle
        .lock()
        .unwrap()
        .clone()
        .expect("source attached")
        .set_drive(Some(TheveninDrive {
            volts: 1.522,
            impedance: 350.0,
        }));
    assert!(
        wait_for(solved(1.522), Duration::from_secs(5)),
        "a live drive update must re-solve on the probe; got {:?}",
        state.lock().unwrap()
    );

    system.shutdown();
}

// ============================================================
// Name-collision validation
// ============================================================

/// A minimal board (one connector) to collide names against.
const CONN_NETLIST: &str = r#"(export (version "E")
  (components
    (comp (ref "J1")
      (value "Conn_01x02")
      (libsource (lib "Connector_Generic") (part "Conn_01x02") (description ""))))
  (nets
    (net (code "1") (name "A") (class "Default")
      (node (ref "J1") (pin "1") (pintype "passive")))
    (net (code "2") (name "B") (class "Default")
      (node (ref "J1") (pin "2") (pintype "passive")))))"#;

/// Bench names must be unique against boards and other bench components.
#[rstest]
fn bench_name_collisions_fail_loudly() {
    let registry = PartRegistry::new();
    let board = Board::from_netlist(
        embsim_board::netlist::parse(CONN_NETLIST).expect("netlist parses"),
        &registry,
    )
    .expect("board builds");

    let probe = |state: &StateSlot| {
        Box::new(AnalogProbe {
            pins: [analog_pin("A")],
            state: Arc::clone(state),
        })
    };

    let state: StateSlot = Arc::new(Mutex::new(None));
    let err = System::new()
        .board("RIG", board)
        .component("RIG", probe(&state))
        .build()
        .unwrap_err();
    assert!(
        matches!(err, SystemError::DuplicateComponent { ref name } if name == "RIG"),
        "bench name colliding with a board must fail; got {err:?}"
    );

    let err = System::new()
        .component("CELL", probe(&state))
        .component("CELL", probe(&state))
        .build()
        .unwrap_err();
    assert!(
        matches!(err, SystemError::DuplicateComponent { ref name } if name == "CELL"),
        "two bench components under one name must fail; got {err:?}"
    );
}
