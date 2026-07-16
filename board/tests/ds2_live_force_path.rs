//! Live force path against the real DS2Addon netlist: the ADS122U04
//! **protocol model** mounted as a live board-engine component
//! (`embsim_models::ads122u04_component`), driven end-to-end — analog bridge
//! sources through the closed JP1/JP2 jumpers into the AIN senses, an RDATA
//! exchange through the 47 Ω-collapsed UART link, and the power/reset gate.
//!
//! Two scenarios bracket the July 2026 bench truth:
//! - with the reset bodge (`pin_short` U1.3 → U1.13, the bench fix), the
//!   chip answers RDATA with the conversion for the driven differential;
//! - without it, the stock netlist's one-pin `~RESET` net leaves the chip
//!   **silent** — perfect commands in, nothing out — with the engine's
//!   `FloatingSense` finding naming the cause. The bench bug, live.

use rstest::rstest;
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};

use embsim_board::component::StreamTx;
use embsim_board::{
    AttachError, Board, Component, ComponentNetIo, EndpointRef, Finding, Harness, JumperState,
    NetState, PartRegistry, PinDecl, PinKind, Scenario, SenseKind, StreamRole, System,
    SystemHandle,
};
use embsim_core::virtual_clock;
use embsim_models::ads122u04::Config;
use embsim_models::ads122u04_component::{Ads122u04Component, ADS122U04_BAUD_HZ};

// ============================================================
// Shared plumbing
// ============================================================

type TxSlot = Arc<Mutex<Option<StreamTx>>>;
type ByteLog = Arc<Mutex<Vec<u8>>>;

/// Paced tests re-anchor the process-global virtual clock; serialize them
/// (poison-recovering, like the stream-routing suite).
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

// ============================================================
// ADC transfer-function fixture (mirrors the registered Config)
// ============================================================

/// Reference voltage handed to the registered model (mV).
const VREF_MV: f64 = 2_048.0;
/// PGA gain handed to the registered model.
const GAIN: f64 = 1.0;

/// Expected output code for a differential input, reusing the model's
/// `voltage_to_adc` math bit for bit (SBAS752B §8.5.2 Eq. 8:
/// `code = VIN · Gain · 2^23 / VREF`, truncated toward zero, clipped at
/// 7FFFFFh / 800000h; zero offset 0 in this fixture).
fn expected_code(diff_mv: f64) -> i32 {
    let code = (diff_mv * GAIN * 8_388_608.0) / VREF_MV;
    (code as i64).clamp(-0x80_0000, 0x7F_FFFF) as i32
}

/// Decode a 3-byte little-endian, 24-bit two's-complement conversion
/// (SBAS752B §8.5.3.4 NOTE: least significant byte first).
fn code_from_le3(bytes: &[u8]) -> i32 {
    let raw = u32::from(bytes[0]) | (u32::from(bytes[1]) << 8) | (u32::from(bytes[2]) << 16);
    ((raw << 8) as i32) >> 8
}

// ============================================================
// Bench host board (the MCU side of the harness)
// ============================================================

/// Hand-written minimal KiCad netlist: a host UART broken out to a two-pin
/// connector for harness attachment (the firmware stand-in).
const HOST_NETLIST: &str = r#"(export (version "E")
  (components
    (comp (ref "MCU")
      (value "FakeHost")
      (libsource (lib "test") (part "FAKE_HOST") (description "")))
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

/// Host UART: shares its stream-write half and logs every received byte.
struct FakeHost {
    pins: [PinDecl; 2],
    tx: TxSlot,
    rx: ByteLog,
}

impl FakeHost {
    fn new(tx: TxSlot, rx: ByteLog) -> Self {
        Self {
            pins: [
                PinDecl {
                    number: "1",
                    name: Some("TX"),
                    kind: PinKind::DigitalOut,
                    stream: Some(StreamRole::Producer {
                        baud_hz: ADS122U04_BAUD_HZ,
                    }),
                    drive_impedance: None,
                },
                PinDecl {
                    number: "2",
                    name: Some("RX"),
                    kind: PinKind::DigitalIn,
                    stream: Some(StreamRole::Consumer {
                        baud_hz: ADS122U04_BAUD_HZ,
                    }),
                    drive_impedance: None,
                },
            ],
            tx,
            rx,
        }
    }
}

impl Component for FakeHost {
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

/// Probe bundle for the host UART end.
struct HostProbe {
    tx: TxSlot,
    rx: ByteLog,
}

impl HostProbe {
    fn new() -> Self {
        Self {
            tx: Arc::new(Mutex::new(None)),
            rx: Arc::new(Mutex::new(Vec::new())),
        }
    }

    fn tx(&self) -> StreamTx {
        self.tx.lock().unwrap().clone().expect("host attached")
    }

    fn rx(&self) -> Vec<u8> {
        self.rx.lock().unwrap().clone()
    }
}

// ============================================================
// System builder: host + DS2Addon + powered bench harness
// ============================================================

/// Start the live system: the real DS2Addon netlist with the LIVE
/// ADS122U04 component, the powered bench harness (both supply straps),
/// two bridge sources at 1.65 V ± `diff_volts`/2 into J2.3/J2.4, the input
/// jumpers closed, and — when `reset_bodge` — the bench fix (`pin_short`
/// U1.3 → U1.13) that ties `~RESET` to the DVDD rail.
fn ds2_live_system(reset_bodge: bool, diff_volts: f64, host: &HostProbe) -> SystemHandle {
    let mut registry = PartRegistry::new();
    {
        let (tx, rx) = (Arc::clone(&host.tx), Arc::clone(&host.rx));
        registry.register("FAKE_HOST", move |_decl| {
            Box::new(FakeHost::new(Arc::clone(&tx), Arc::clone(&rx)))
        });
    }
    registry.register("ADS122U04", |_decl| {
        Box::new(Ads122u04Component::new(Config {
            vref_mv: VREF_MV,
            gain: GAIN,
            zero_offset: 0,
        }))
    });

    let host_board = Board::from_netlist(
        embsim_board::netlist::parse(HOST_NETLIST).expect("host netlist parses"),
        &registry,
    )
    .expect("host board builds");
    let ds2_board = Board::from_netlist(
        embsim_board::netlist::parse(include_str!("fixtures/ds2_addon.net"))
            .expect("fixture parses"),
        &registry,
    )
    .expect("ds2 board builds");

    // Bench harness: power straps as on the real rig, the straight serial
    // wires (host TX → J1.3 → R3 → U1 RX; U1 TX → R4 → J1.4 → host RX),
    // and two Thevenin bridge sources at the analog terminals.
    let harness = Harness::new()
        .power(ep("BENCH.3V3"), ep("DS2Addon.J1.1"), 3.3)
        .power(ep("BENCH.GND"), ep("DS2Addon.J1.2"), 0.0)
        .power(ep("BENCH.3V3A"), ep("DS2Addon.J2.1"), 3.3)
        .power(ep("BENCH.GNDA"), ep("DS2Addon.J2.2"), 0.0)
        .power(ep("BENCH.A0"), ep("DS2Addon.J2.3"), 1.65 + diff_volts / 2.0)
        .power(ep("BENCH.A1"), ep("DS2Addon.J2.4"), 1.65 - diff_volts / 2.0)
        .connect_str("Host.J1.1", "DS2Addon.J1.3")
        .expect("endpoints parse")
        .connect_str("Host.J1.2", "DS2Addon.J1.4")
        .expect("endpoints parse");

    let mut scenario = Scenario::default()
        .jumper("DS2Addon.JP1", JumperState::Closed)
        .jumper("DS2Addon.JP2", JumperState::Closed);
    if reset_bodge {
        scenario = scenario.pin_short("DS2Addon.U1.3", "DS2Addon.U1.13");
    }

    System::new()
        .board("Host", host_board)
        .board("DS2Addon", ds2_board)
        .harness(harness)
        .scenario(scenario)
        .start()
        .expect("live system starts")
}

/// Solved voltage of a named net (panics with the actual state otherwise).
fn analog_volts(system: &SystemHandle, net: &str) -> f64 {
    match system.net_state(net) {
        Some(NetState::Analog(v)) => v,
        other => panic!("{net} must solve numerically, got {other:?}"),
    }
}

// ============================================================
// (a) RDATA round trip with the reset bodge
// ============================================================

/// With the bodge in place the chip is alive: SYNC + RDATA (0x55 0x10,
/// SBAS752B §8.5.3.4) into the ADS RX stream endpoint answers with the
/// 3-byte conversion for the MNA-solved bridge differential — the whole
/// force path (bridge sources → jumpers → AIN senses → set_voltage →
/// protocol model → TX stream) live, no hand wiring.
#[rstest]
fn rdata_returns_the_conversion_for_the_driven_bridge_differential() {
    let _g = lock_clock();
    virtual_clock::init(50.0, 1_000_000); // 115.2 kbaud pacing samples the clock

    let host = HostProbe::new();
    // 256 mV differential: 1.778 V / 1.522 V at the bridge terminals.
    let system = ds2_live_system(true, 0.256, &host);

    // Straight harness: no stream mismatch, engine healthy.
    assert!(
        !system
            .findings()
            .iter()
            .any(|f| matches!(f, Finding::StreamMismatch { .. })),
        "straight harness must route cleanly; got {:?}",
        system.findings()
    );
    // The bodge sources ~RESET: the bench finding must NOT fire.
    assert!(
        !system.findings().contains(&Finding::FloatingSense {
            net: "DS2Addon.~RESET".to_string(),
            kind: SenseKind::Digital,
        }),
        "bodged reset must not float; got {:?}",
        system.findings()
    );

    // The AIN nets solve through the closed jumpers to the driven terminals.
    let v0 = analog_volts(&system, "DS2Addon.AIN0");
    let v1 = analog_volts(&system, "DS2Addon.AIN1");
    assert!(
        (v0 - 1.778).abs() < 1e-6,
        "AIN0 hand check 1.778 V, got {v0}"
    );
    assert!(
        (v1 - 1.522).abs() < 1e-6,
        "AIN1 hand check 1.522 V, got {v1}"
    );

    // SYNC + RDATA into the ADS RX endpoint (via the host TX producer).
    host.tx().write(&[0x55, 0x10]);
    assert!(
        wait_for(|| host.rx().len() >= 3, Duration::from_secs(10)),
        "RDATA must answer with a 3-byte conversion; got {:?}",
        host.rx()
    );
    let rx = host.rx();
    assert_eq!(rx.len(), 3, "exactly one conversion frame; got {rx:?}");

    // The returned code matches the model transfer function applied to the
    // differential the component computed from the same solved voltages —
    // V(AIN0) − V(AIN1) in mV, fed straight into set_voltage.
    let diff_mv = (v0 - v1) * 1_000.0;
    let expected = expected_code(diff_mv);
    let got = code_from_le3(&rx);
    assert_eq!(
        got, expected,
        "conversion must encode the solved differential ({diff_mv} mV)"
    );
    // Human sanity: 256 mV at gain 1 / VREF 2048 mV ≈ 2^20.
    assert!(
        (i64::from(got) - 0x10_0000).abs() <= 1,
        "256 mV differential must read ~0x100000, got {got:#x}"
    );

    system.shutdown();
}

// ============================================================
// (b) Without the reset bodge: the silent chip, live
// ============================================================

/// The stock netlist leaves `~RESET` on a one-pin net. Without the bodge the
/// engine reports the floating sense at start, and the LIVE chip exhibits
/// the exact bench symptom: the command stream is delivered, the chip never
/// answers. (Two boards were condemned on the bench before a multimeter
/// found this; here it is one assertion.)
#[rstest]
fn without_the_reset_bodge_the_chip_stays_silent() {
    let _g = lock_clock();
    virtual_clock::init(50.0, 1_000_000);

    let host = HostProbe::new();
    let system = ds2_live_system(false, 0.256, &host);

    // The engine names the cause before any traffic.
    assert!(
        system.findings().contains(&Finding::FloatingSense {
            net: "DS2Addon.~RESET".to_string(),
            kind: SenseKind::Digital,
        }),
        "the one-pin ~RESET net must surface as a floating sense; got {:?}",
        system.findings()
    );

    // Perfect SYNC + RDATA in — and nothing out. 400 wall ms at 50× is
    // 20 virtual seconds: any response would long since have arrived.
    host.tx().write(&[0x55, 0x10]);
    std::thread::sleep(Duration::from_millis(400));
    assert!(
        host.rx().is_empty(),
        "a chip held in reset must never answer; got {:?}",
        host.rx()
    );
    assert!(
        system.engine_is_alive(),
        "engine must stay alive throughout"
    );

    system.shutdown();
}
