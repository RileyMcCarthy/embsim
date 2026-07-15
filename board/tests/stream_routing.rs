//! Live stream routing (`BOARD_ENGINE.md`, "Stream endpoints (serial over
//! pins)"): byte pipes are derived from and gated by net resolution, never
//! installed beside it.
//!
//! The DS2Addon tests run against `tests/fixtures/ds2_addon.net` — the real
//! `kicad-cli sch export netlist` of the board debugged on the bench — so the
//! 47 Ω series-resistor collapse and the crossed-TX/RX harness regression
//! hold the engine to the same truth the hardware exhibited.

use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};

use embsim_board::component::StreamTx;
use embsim_board::{
    AttachError, Board, Component, ComponentNetIo, EndpointRef, Finding, Harness, Level, NetState,
    PartRegistry, PinDecl, PinHandle, PinKind, PinRef, Scenario, StreamDropPolicy, StreamRole,
    System, SystemHandle, TheveninDrive,
};
use embsim_core::virtual_clock;

// ============================================================
// Shared probe plumbing
// ============================================================

type TxSlot = Arc<Mutex<Option<StreamTx>>>;
type PinSlot = Arc<Mutex<Option<PinHandle>>>;
type ByteLog = Arc<Mutex<Vec<u8>>>;

/// Paced tests re-anchor the process-global virtual clock; serialize them
/// (poison-recovering, like the engine's own timer suite).
static CLOCK_LOCK: Mutex<()> = Mutex::new(());

fn lock_clock() -> MutexGuard<'static, ()> {
    CLOCK_LOCK.lock().unwrap_or_else(|p| {
        CLOCK_LOCK.clear_poison();
        p.into_inner()
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

fn ep(s: &str) -> EndpointRef {
    EndpointRef::parse(s).expect("endpoint parses")
}

const HIGH: TheveninDrive = TheveninDrive {
    volts: 3.3,
    impedance: 25.0,
};
const LOW: TheveninDrive = TheveninDrive {
    volts: 0.0,
    impedance: 25.0,
};

// ============================================================
// Fake MCU probe board (bench rig side of the harness)
// ============================================================

/// Hand-written minimal KiCad netlist: an MCU with a UART (TX producer, RX
/// consumer) broken out to a two-pin connector for harness attachment.
const RIG_NETLIST: &str = r#"(export (version "E")
  (components
    (comp (ref "MCU")
      (value "FakeMcu")
      (libsource (lib "test") (part "FAKE_MCU") (description "")))
    (comp (ref "J1")
      (value "Conn_01x02")
      (libsource (lib "Connector_Generic") (part "Conn_01x02") (description ""))))
  (nets
    (net (code "1") (name "TX") (class "Default")
      (node (ref "MCU") (pin "1") (pinfunction "TX") (pintype "output"))
      (node (ref "J1") (pin "1") (pintype "passive")))
    (net (code "2") (name "RX") (class "Default")
      (node (ref "MCU") (pin "2") (pinfunction "RX") (pintype "input"))
      (node (ref "J1") (pin "2") (pintype "passive")))))"#;

/// Fake MCU UART: shares its stream-write half, its TX pin handle (so tests
/// can drive the line like a firmware bit-bang would), and its RX byte log.
struct FakeMcu {
    pins: [PinDecl; 2],
    tx: TxSlot,
    tx_pin: PinSlot,
    rx: ByteLog,
}

impl FakeMcu {
    fn new(baud_hz: u32, tx: TxSlot, tx_pin: PinSlot, rx: ByteLog) -> Self {
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
            tx_pin,
            rx,
        }
    }
}

impl Component for FakeMcu {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        *self.tx.lock().unwrap() = Some(io.stream_tx("TX")?);
        *self.tx_pin.lock().unwrap() = Some(io.pin("TX")?);
        let log = Arc::clone(&self.rx);
        io.on_byte("RX", move |byte| log.lock().unwrap().push(byte))?;
        Ok(())
    }
}

// ============================================================
// ADS122U04 pin facade (TSSOP-16, TI SBAS752B pin table p.3)
// ============================================================

const NONE: Option<StreamRole> = None;

const fn pin(
    number: &'static str,
    name: Option<&'static str>,
    kind: PinKind,
    stream: Option<StreamRole>,
) -> PinDecl {
    PinDecl {
        number,
        name,
        kind,
        stream,
        drive_impedance: None,
    }
}

/// TSSOP-16 (PW) pinout per SBAS752B; TX/RX declare their UART stream roles
/// at the chip's 115.2 kbaud default.
const ADS122U04_PINS: [PinDecl; 16] = [
    pin("1", Some("GPIO1"), PinKind::DigitalIn, NONE),
    pin("2", Some("GPIO0"), PinKind::DigitalIn, NONE),
    pin("3", Some("~RESET"), PinKind::DigitalIn, NONE),
    pin("4", Some("DGND"), PinKind::PowerIn, NONE),
    pin("5", Some("AVSS"), PinKind::PowerIn, NONE),
    pin("6", Some("AIN3"), PinKind::Analog, NONE),
    pin("7", Some("AIN2"), PinKind::Analog, NONE),
    pin("8", Some("REFN"), PinKind::Analog, NONE),
    pin("9", Some("REFP"), PinKind::Analog, NONE),
    pin("10", Some("AIN1"), PinKind::Analog, NONE),
    pin("11", Some("AIN0"), PinKind::Analog, NONE),
    pin("12", Some("AVDD"), PinKind::PowerIn, NONE),
    pin("13", Some("DVDD"), PinKind::PowerIn, NONE),
    pin("14", Some("DRDY"), PinKind::DigitalIn, NONE),
    pin(
        "15",
        Some("TX"),
        PinKind::DigitalOut,
        Some(StreamRole::Producer { baud_hz: 115_200 }),
    ),
    pin(
        "16",
        Some("RX"),
        PinKind::DigitalIn,
        Some(StreamRole::Consumer { baud_hz: 115_200 }),
    ),
];

/// Stream-capturing ADS122U04 facade (the protocol model lives in
/// `embsim-models`; routing is what this facade exercises).
struct Ads122u04Facade {
    tx: TxSlot,
    rx: ByteLog,
}

impl Component for Ads122u04Facade {
    fn pins(&self) -> &[PinDecl] {
        &ADS122U04_PINS
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        *self.tx.lock().unwrap() = Some(io.stream_tx("TX")?);
        let log = Arc::clone(&self.rx);
        io.on_byte("RX", move |byte| log.lock().unwrap().push(byte))?;
        Ok(())
    }
}

// ============================================================
// System builders
// ============================================================

/// Probe bundle for one UART end.
struct Probe {
    tx: TxSlot,
    tx_pin: PinSlot,
    rx: ByteLog,
}

impl Probe {
    fn new() -> Self {
        Self {
            tx: Arc::new(Mutex::new(None)),
            tx_pin: Arc::new(Mutex::new(None)),
            rx: Arc::new(Mutex::new(Vec::new())),
        }
    }

    fn tx(&self) -> StreamTx {
        self.tx.lock().unwrap().clone().expect("component attached")
    }

    fn tx_pin(&self) -> PinHandle {
        self.tx_pin
            .lock()
            .unwrap()
            .clone()
            .expect("component attached")
    }

    fn rx(&self) -> Vec<u8> {
        self.rx.lock().unwrap().clone()
    }
}

/// Rig + DS2Addon boards, wired to the given probes.
fn serial_boards(mcu: &Probe, ads: &Probe) -> (Board, Board) {
    let mut registry = PartRegistry::new();
    {
        let (tx, tx_pin, rx) = (
            Arc::clone(&mcu.tx),
            Arc::clone(&mcu.tx_pin),
            Arc::clone(&mcu.rx),
        );
        registry.register("FAKE_MCU", move |_decl| {
            Box::new(FakeMcu::new(
                115_200,
                Arc::clone(&tx),
                Arc::clone(&tx_pin),
                Arc::clone(&rx),
            ))
        });
    }
    {
        let (tx, rx) = (Arc::clone(&ads.tx), Arc::clone(&ads.rx));
        registry.register("ADS122U04", move |_decl| {
            Box::new(Ads122u04Facade {
                tx: Arc::clone(&tx),
                rx: Arc::clone(&rx),
            })
        });
    }
    let rig = Board::from_netlist(
        embsim_board::netlist::parse(RIG_NETLIST).expect("rig netlist parses"),
        &registry,
    )
    .expect("rig board builds");
    let ds2 = Board::from_netlist(
        embsim_board::netlist::parse(include_str!("fixtures/ds2_addon.net"))
            .expect("fixture parses"),
        &registry,
    )
    .expect("ds2 board builds");
    (rig, ds2)
}

/// Bench harness with power straps and the serial wires. Straight wiring
/// puts MCU TX on DS2 J1.3 (→ R3 → U1 RX) and MCU RX on DS2 J1.4 (← R4 ←
/// U1 TX); `crossed` swaps them — the bench bug that produced TX-facing-TX.
fn bench_harness(crossed: bool) -> Harness {
    let (tx_to, rx_to) = if crossed {
        ("DS2Addon.J1.4", "DS2Addon.J1.3")
    } else {
        ("DS2Addon.J1.3", "DS2Addon.J1.4")
    };
    Harness::new()
        .power(ep("BENCH.3V3"), ep("DS2Addon.J1.1"), 3.3)
        .power(ep("BENCH.GND"), ep("DS2Addon.J1.2"), 0.0)
        .power(ep("BENCH.3V3A"), ep("DS2Addon.J2.1"), 3.3)
        .power(ep("BENCH.GNDA"), ep("DS2Addon.J2.2"), 0.0)
        .connect_str("Rig.J1.1", tx_to)
        .expect("endpoints parse")
        .connect_str("Rig.J1.2", rx_to)
        .expect("endpoints parse")
}

fn stream_mismatch_with(findings: &[Finding], a: &PinRef, b: &PinRef) -> bool {
    findings.iter().any(|f| matches!(
        f,
        Finding::StreamMismatch { producers, .. } if producers.contains(a) && producers.contains(b)
    ))
}

// ============================================================
// Minimal single-net link board (pacing / drop / gating tests)
// ============================================================

/// One producer pin and one consumer pin sharing the `LINK` net.
const LINK_NETLIST: &str = r#"(export (version "E")
  (components
    (comp (ref "MCU")
      (value "FakeTx")
      (libsource (lib "test") (part "FAKE_TX") (description "")))
    (comp (ref "SNS")
      (value "FakeRx")
      (libsource (lib "test") (part "FAKE_RX") (description ""))))
  (nets
    (net (code "1") (name "LINK") (class "Default")
      (node (ref "MCU") (pin "1") (pinfunction "TX") (pintype "output"))
      (node (ref "SNS") (pin "1") (pinfunction "RX") (pintype "input")))))"#;

/// Single-pin stream producer.
struct FakeTx {
    pins: [PinDecl; 1],
    tx: TxSlot,
    tx_pin: PinSlot,
}

impl FakeTx {
    fn new(baud_hz: u32, tx: TxSlot, tx_pin: PinSlot) -> Self {
        Self {
            pins: [PinDecl {
                number: "1",
                name: Some("TX"),
                kind: PinKind::DigitalOut,
                stream: Some(StreamRole::Producer { baud_hz }),
                drive_impedance: None,
            }],
            tx,
            tx_pin,
        }
    }
}

impl Component for FakeTx {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        *self.tx.lock().unwrap() = Some(io.stream_tx("TX")?);
        *self.tx_pin.lock().unwrap() = Some(io.pin("TX")?);
        Ok(())
    }
}

/// Single-pin stream consumer.
struct FakeRx {
    pins: [PinDecl; 1],
    rx: ByteLog,
}

impl FakeRx {
    fn new(baud_hz: u32, rx: ByteLog) -> Self {
        Self {
            pins: [PinDecl {
                number: "1",
                name: Some("RX"),
                kind: PinKind::DigitalIn,
                stream: Some(StreamRole::Consumer { baud_hz }),
                drive_impedance: None,
            }],
            rx,
        }
    }
}

impl Component for FakeRx {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let log = Arc::clone(&self.rx);
        io.on_byte("RX", move |byte| log.lock().unwrap().push(byte))?;
        Ok(())
    }
}

/// Start the LINK board live with the given producer baud and scenario.
fn link_system(baud_hz: u32, scenario: Scenario, probe: &Probe) -> SystemHandle {
    let mut registry = PartRegistry::new();
    {
        let (tx, tx_pin) = (Arc::clone(&probe.tx), Arc::clone(&probe.tx_pin));
        registry.register("FAKE_TX", move |_decl| {
            Box::new(FakeTx::new(baud_hz, Arc::clone(&tx), Arc::clone(&tx_pin)))
        });
    }
    {
        let rx = Arc::clone(&probe.rx);
        registry.register("FAKE_RX", move |_decl| {
            Box::new(FakeRx::new(baud_hz, Arc::clone(&rx)))
        });
    }
    let board = Board::from_netlist(
        embsim_board::netlist::parse(LINK_NETLIST).expect("link netlist parses"),
        &registry,
    )
    .expect("link board builds");
    System::new()
        .board("Rig", board)
        .scenario(scenario)
        .start()
        .expect("live system starts")
}

// ============================================================
// DS2Addon regression: 47 Ω series resistors collapse into the link
// ============================================================

/// The real board routes its UART through 47 Ω series resistors (R3 into
/// U1 RX, R4 out of U1 TX). Both directions must deliver bytes through the
/// collapsed link — against the actual `kicad-cli` netlist export.
#[test]
fn ds2_47_ohm_series_resistors_collapse_into_the_link() {
    let _g = lock_clock();
    virtual_clock::init(50.0, 1_000_000); // 115.2 kbaud pacing samples the clock

    let mcu = Probe::new();
    let ads = Probe::new();
    let (rig, ds2) = serial_boards(&mcu, &ads);
    let system = System::new()
        .board("Rig", rig)
        .board("DS2Addon", ds2)
        .harness(bench_harness(false))
        .start()
        .expect("live system starts");

    // A correctly-wired harness must not raise a stream mismatch.
    assert!(
        !system
            .findings()
            .iter()
            .any(|f| matches!(f, Finding::StreamMismatch { .. })),
        "straight harness must route cleanly; got {:?}",
        system.findings()
    );

    // MCU TX → harness → DS2 J1.3 → R3 (47 Ω) → U1 RX.
    mcu.tx().write(&[0x55, 0x08, 0x02]);
    assert!(
        wait_for(|| ads.rx() == [0x55, 0x08, 0x02], Duration::from_secs(5)),
        "bytes must reach the ADS RX through the 47 ohm series resistor; got {:?}",
        ads.rx()
    );

    // U1 TX → R4 (47 Ω) → DS2 J1.4 → harness → MCU RX.
    ads.tx().write(b"OK");
    assert!(
        wait_for(|| mcu.rx() == b"OK", Duration::from_secs(5)),
        "bytes must reach the MCU RX through the 47 ohm series resistor; got {:?}",
        mcu.rx()
    );
}

// ============================================================
// Crossed-harness regression: StreamMismatch + net rules
// ============================================================

/// The bench crossed-harness bug: TX wired to TX, RX to RX. Routing time
/// raises `StreamMismatch` (at build AND at start), no bytes ever flow, and
/// the underlying nets resolve per the net rules — the TX↔TX pair goes to
/// `Contention` as soon as the two producers actually disagree, while the
/// RX↔RX net floats with its silent consumers.
#[test]
fn crossed_tx_rx_harness_raises_stream_mismatch_and_contention() {
    let mcu_tx_pin = PinRef::new("MCU", "1");
    let ads_tx_pin = PinRef::new("U1", "15");

    // Build-time analysis path reports the mismatch before anything runs.
    let mcu = Probe::new();
    let ads = Probe::new();
    let (rig, ds2) = serial_boards(&mcu, &ads);
    let built = System::new()
        .board("Rig", rig)
        .board("DS2Addon", ds2)
        .harness(bench_harness(true))
        .build()
        .expect("system builds");
    assert!(
        stream_mismatch_with(built.diagnostics().findings(), &mcu_tx_pin, &ads_tx_pin),
        "build must report the facing producers; got {:?}",
        built.diagnostics().findings()
    );

    // Live path: mismatch present at start (routing time), before traffic.
    let mcu = Probe::new();
    let ads = Probe::new();
    let (rig, ds2) = serial_boards(&mcu, &ads);
    let system = System::new()
        .board("Rig", rig)
        .board("DS2Addon", ds2)
        .harness(bench_harness(true))
        .start()
        .expect("live system starts");
    assert!(
        stream_mismatch_with(&system.findings(), &mcu_tx_pin, &ads_tx_pin),
        "start must report the facing producers; got {:?}",
        system.findings()
    );

    // Bytes written into the invalid route are dropped, not queued.
    mcu.tx().write(b"\x00\x01");
    std::thread::sleep(Duration::from_millis(50));
    assert!(ads.rx().is_empty(), "no route may deliver: {:?}", ads.rx());
    assert!(mcu.rx().is_empty(), "no route may deliver: {:?}", mcu.rx());

    // Both TX producers idle high and agree; the moment one transmits a
    // low level the pair fights through the collapsed 47 Ω — Contention,
    // exactly like the bench line fighting under traffic.
    mcu.tx_pin().set_drive(Some(LOW));
    assert!(
        wait_for(
            || system.net_state("Rig.TX") == Some(NetState::Contention),
            Duration::from_secs(5)
        ),
        "TX-to-TX net must resolve Contention; got {:?}",
        system.net_state("Rig.TX")
    );
    assert_eq!(
        system.net_state("DS2Addon.Net-(U1-TX)"),
        Some(NetState::Contention),
        "the coupled producer net contends too"
    );
    assert!(
        system.findings().iter().any(|f| matches!(
            f,
            Finding::Contention { drivers, .. }
                if drivers.contains(&mcu_tx_pin) && drivers.contains(&ads_tx_pin)
        )),
        "the contention finding must name both producers; got {:?}",
        system.findings()
    );

    // The RX↔RX net has no producer at all: floating, silent consumers.
    assert_eq!(system.net_state("Rig.RX"), Some(NetState::Floating));
}

// ============================================================
// Baud pacing against virtual time
// ============================================================

/// Bytes flow at the producer's declared baud, paced against the virtual
/// clock (10 bits/byte, 8N1): at 50 baud a byte takes 200 virtual ms, so
/// nothing arrives instantly and three bytes take ≥ 600 virtual ms
/// (≥ 300 wall ms at 2× scale), in wire order.
#[test]
fn producer_baud_paces_bytes_against_virtual_time() {
    let _g = lock_clock();
    virtual_clock::init(2.0, 1_000_000);

    let probe = Probe::new();
    let system = link_system(50, Scenario::default(), &probe);

    let started = Instant::now();
    probe.tx().write(b"ABC");
    assert!(
        probe.rx().is_empty(),
        "paced bytes must not arrive instantly"
    );
    assert!(
        wait_for(|| probe.rx().len() == 3, Duration::from_secs(10)),
        "all paced bytes must arrive; got {:?}",
        probe.rx()
    );
    assert!(
        started.elapsed() >= Duration::from_millis(250),
        "three bytes at 50 baud must take >= ~300 wall ms at 2x scale; took {:?}",
        started.elapsed()
    );
    assert_eq!(probe.rx(), b"ABC".to_vec(), "wire order is preserved");
    drop(system);
}

// ============================================================
// Scenario stream_drop applies to live pipes
// ============================================================

/// `Scenario::stream_drop` byte-loss injection actually applies to the live
/// pipe: `EveryNth` on the producer thins the stream, `All` on the consumer
/// silences it.
#[test]
fn stream_drop_policies_apply_to_live_pipes() {
    // EveryNth(2) on the producer pin: every second byte written is lost.
    let probe = Probe::new();
    let system = link_system(
        0, // unpaced: delivery order is still wire order
        Scenario::default().stream_drop("Rig.MCU.1", StreamDropPolicy::EveryNth(2)),
        &probe,
    );
    probe.tx().write(b"ABCDEF");
    assert!(
        wait_for(|| probe.rx() == b"ACE", Duration::from_secs(5)),
        "EveryNth(2) must drop B, D, F; got {:?}",
        probe.rx()
    );
    std::thread::sleep(Duration::from_millis(30));
    assert_eq!(probe.rx(), b"ACE".to_vec(), "dropped bytes never reappear");
    drop(system);

    // All on the consumer pin: the route stays valid but delivers nothing.
    let probe = Probe::new();
    let system = link_system(
        0,
        Scenario::default().stream_drop("Rig.SNS.1", StreamDropPolicy::All),
        &probe,
    );
    probe.tx().write(b"ABCDEF");
    std::thread::sleep(Duration::from_millis(50));
    assert!(
        probe.rx().is_empty(),
        "All must silence the consumer; got {:?}",
        probe.rx()
    );
    drop(system);
}

// ============================================================
// Broken / degraded routes drop bytes instead of queueing
// ============================================================

/// A detached producer pin never forms a route: writes are dropped with a
/// trace, never delivered, never queued — and attach still succeeds.
#[test]
fn detached_producer_pin_breaks_the_route() {
    let probe = Probe::new();
    let system = link_system(0, Scenario::default().pin_detach("Rig.MCU.1"), &probe);
    probe.tx().write(b"lost");
    std::thread::sleep(Duration::from_millis(50));
    assert!(
        probe.rx().is_empty(),
        "a detached producer must deliver nothing; got {:?}",
        probe.rx()
    );
    drop(system);
}

/// Byte pipes are gated by net resolution: when the producer releases its
/// drive the link floats and bytes written meanwhile are dropped (not
/// queued); re-driving the line restores delivery.
#[test]
fn floating_link_gates_delivery_until_redriven() {
    let probe = Probe::new();
    let system = link_system(0, Scenario::default(), &probe);

    probe.tx().write(b"hi");
    assert!(
        wait_for(|| probe.rx() == b"hi", Duration::from_secs(5)),
        "healthy link must deliver; got {:?}",
        probe.rx()
    );

    // Release the producer's drive: the net floats, the route is degraded.
    probe.tx_pin().set_drive(None);
    assert!(
        wait_for(
            || system.net_state("Rig.LINK") == Some(NetState::Floating),
            Duration::from_secs(5)
        ),
        "released line must float; got {:?}",
        system.net_state("Rig.LINK")
    );
    probe.tx().write(b"xx");
    std::thread::sleep(Duration::from_millis(50));
    assert_eq!(
        probe.rx(),
        b"hi".to_vec(),
        "bytes written into a floating link are dropped, not queued"
    );

    // Re-drive the idle-high line: delivery resumes for new bytes only.
    probe.tx_pin().set_drive(Some(HIGH));
    assert!(
        wait_for(
            || system.net_state("Rig.LINK") == Some(NetState::Driven(Level::High)),
            Duration::from_secs(5)
        ),
        "re-driven line must go back to idle-high"
    );
    probe.tx().write(b"!");
    assert!(
        wait_for(|| probe.rx() == b"hi!", Duration::from_secs(5)),
        "delivery must resume on the re-driven link; got {:?}",
        probe.rx()
    );
    drop(system);
}
