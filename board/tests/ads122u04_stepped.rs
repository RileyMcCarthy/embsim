//! The reference model component under a **stepped clock**
//! (`DETERMINISM.md` T1 §4, "Model protocol threads").
//!
//! `ds2_live_force_path.rs` proves the ADS122U04 force path works against the
//! real netlist in free-running mode. This binary proves the same component
//! works when *nothing samples wall time*: its output drain is an engine
//! wakeup rather than a pump thread, and the protocol model's own thread is a
//! registered `virtual_clock` actor that parks on the barrier. If either had
//! been left on wall time, this test would hang rather than fail — which is
//! precisely why it is worth having.
//!
//! **What this does NOT claim.** The chip↔model byte path is a real
//! `socketpair`, so *whether the model has written a byte yet* when the engine
//! drains is still an OS decision. This asserts the round trip completes and
//! carries the right value; it deliberately does not assert an identical event
//! log. Making that hold is `DETERMINISM.md` Phase D2's in-process transport.
//!
//! Its own test binary per `TESTING.md` rule 5, and necessarily so: creating an
//! [`Ads122u04Component`] starts a protocol thread that lives for the rest of
//! the process, and `virtual_clock::init_mode(Stepped)` refuses to run with a
//! leftover actor registered. Stepped mode must therefore be entered before any
//! such component exists, which only a dedicated binary can guarantee.

use rstest::rstest;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use embsim_board::component::StreamTx;
use embsim_board::{
    AttachError, Component, ComponentNetIo, EndpointRef, Harness, PinDecl, PinKind, StreamRole,
    System,
};
use embsim_core::virtual_clock::{self, ClockMode};
use embsim_models::ads122u04::Config;
use embsim_models::ads122u04_component::{Ads122u04Component, ADS122U04_BAUD_HZ};

/// Bridge terminal voltages: a 256 mV differential into AIN0/AIN1.
const AIN0_VOLTS: f64 = 1.778;
const AIN1_VOLTS: f64 = 1.522;

/// Model configuration, matching the DS2Addon force path.
const VREF_MV: f64 = 2048.0;
const GAIN: f64 = 1.0;

fn ep(s: &str) -> EndpointRef {
    EndpointRef::parse(s).expect("endpoint parses")
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

type TxSlot = Arc<Mutex<Option<StreamTx>>>;
type ByteLog = Arc<Mutex<Vec<u8>>>;

/// The "firmware" side of the UART: a TX producer and an RX consumer.
struct HostProbe {
    pins: [PinDecl; 2],
    tx: TxSlot,
    rx: ByteLog,
}

impl HostProbe {
    fn new(tx: TxSlot, rx: ByteLog) -> Self {
        Self {
            pins: [
                PinDecl {
                    number: "TX",
                    name: None,
                    kind: PinKind::DigitalOut,
                    stream: Some(StreamRole::Producer {
                        baud_hz: ADS122U04_BAUD_HZ,
                    }),
                    drive_impedance: None,
                },
                PinDecl {
                    number: "RX",
                    name: None,
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

impl Component for HostProbe {
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

/// Expected 24-bit code for a differential, per SBAS752B §8.5.2.
fn expected_code(diff_mv: f64) -> i32 {
    ((diff_mv * GAIN * 8_388_608.0) / VREF_MV) as i32
}

/// Decode a little-endian 3-byte two's-complement conversion word.
fn code_from_le3(bytes: &[u8]) -> i32 {
    let raw = u32::from(bytes[0]) | (u32::from(bytes[1]) << 8) | (u32::from(bytes[2]) << 16);
    if raw & 0x80_0000 != 0 {
        (raw | 0xFF00_0000) as i32
    } else {
        raw as i32
    }
}

/// SYNC + RDATA (SBAS752B §8.5.3.4) answers with the conversion for the solved
/// differential — with virtual time advanced entirely by the engine.
///
/// Every wait in this system is a virtual one: the ADS component's output
/// drain is a `schedule_every` wheel entry, the model's protocol thread parks
/// through `wait_virtual_us` as a registered actor, and the 115.2 kbaud link is
/// paced against the same stepped clock. Nothing here can make progress on wall
/// time, so completion is itself the assertion.
#[rstest]
fn rdata_round_trip_completes_under_a_stepped_clock() {
    // Stepped mode first, before any ADS component (and therefore any actor)
    // exists — see the module docs.
    virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);

    let tx: TxSlot = Arc::new(Mutex::new(None));
    let rx: ByteLog = Arc::new(Mutex::new(Vec::new()));

    let harness = Harness::new()
        // UART: host TX → ADC RX, ADC TX → host RX.
        .connect_str("HOST.TX", "ADC.RX")
        .expect("endpoints parse")
        .connect_str("ADC.TX", "HOST.RX")
        .expect("endpoints parse")
        // Supplies and the reset strap: both rails up and ~RESET high is the
        // chip's power-on envelope (SBAS752B).
        .power(ep("BENCH.3V3"), ep("ADC.AVDD"), 3.3)
        .power(ep("BENCH.3V3D"), ep("ADC.DVDD"), 3.3)
        .power(ep("BENCH.RESET"), ep("ADC.~RESET"), 3.3)
        // The bridge terminals.
        .power(ep("BENCH.AIN0"), ep("ADC.AIN0"), AIN0_VOLTS)
        .power(ep("BENCH.AIN1"), ep("ADC.AIN1"), AIN1_VOLTS);

    let system = System::new()
        .component(
            "ADC",
            Box::new(Ads122u04Component::new(Config {
                vref_mv: VREF_MV,
                gain: GAIN,
                zero_offset: 0,
            })),
        )
        .component(
            "HOST",
            Box::new(HostProbe::new(Arc::clone(&tx), Arc::clone(&rx))),
        )
        .harness(harness)
        .start()
        .expect("stepped ADS system starts");

    tx.lock()
        .unwrap()
        .as_ref()
        .expect("HOST.TX attached")
        .write(&[0x55, 0x10]);

    assert!(
        wait_for(|| rx.lock().unwrap().len() >= 3, Duration::from_secs(20)),
        "RDATA must answer with a 3-byte conversion under a stepped clock; got {:?}. \
         A hang here means some wait in this path is still on wall time — the engine \
         advances virtual time only when every registered actor is parked, so a \
         non-virtual wait stalls the whole system rather than merely drifting.",
        rx.lock().unwrap()
    );

    let frame = rx.lock().unwrap()[..3].to_vec();
    let expected = expected_code((AIN0_VOLTS - AIN1_VOLTS) * 1_000.0);
    assert_eq!(
        code_from_le3(&frame),
        expected,
        "the conversion must encode the solved differential"
    );
    // Human sanity: 256 mV at gain 1 / VREF 2048 mV ≈ 2^20.
    assert!(
        (i64::from(code_from_le3(&frame)) - 0x10_0000).abs() <= 1,
        "256 mV differential must read ~0x100000, got {:#x}",
        code_from_le3(&frame)
    );

    // Virtual time really did advance, and only through the engine.
    assert!(
        virtual_clock::virtual_us() > 0,
        "the engine must have advanced virtual time to pace the link"
    );
    assert!(
        system.engine_is_alive(),
        "the engine must survive a stepped round trip"
    );
    system.shutdown();
}
