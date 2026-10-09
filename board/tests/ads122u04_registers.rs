//! The TI ADS122U04 converter (`embsim_models::ads122u04`) as the standard
//! catalog places it, configured over its own serial pins: the register
//! writes a host sends set every conversion's gain, reference and input,
//! against TI SBAS752B; and the add-on's force path, configured by the exact
//! bytes MaD's firmware sends, reading the code the firmware expects.
//!
//! The bench cases place the part by its ordering code (`ADS122U04IPW`)
//! with the base registry, so what is tested is what a project gets. The
//! bench supplies `DVDD` and `AVDD` against their grounds, holds the inputs
//! and the reference pins, and drives `~RESET` through a source the case can
//! move; a level-speaking UART probe is the host on `RX` and `TX`. The
//! force-path case builds `boards/projects/ds2-addon.toml` with the standard
//! catalog and adds the bridge, the host and the bench's two fixes.
//!
//! Stepped (`TESTING.md` rule 9): a suite lock, the clock re-anchored
//! stepped, the system started with time held, the case's thread a
//! registered actor, every exchange followed by one virtual settle longer
//! than the exchange, no `QuiescenceTimeout` at the end. The converter's
//! protocol thread is a clock actor that ends when the system drops the
//! part; each case waits for it before the next re-anchors the clock.

use std::collections::BTreeMap;
use std::path::PathBuf;
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};

use embsim_board::{
    netlist, AttachError, Board, Component, ComponentNetIo, EndpointRef, Finding, Harness,
    JumperState, NetState, PinDecl, PinHandle, Project, Scenario, System, SystemHandle,
    TheveninDrive, Volts,
};
use embsim_boards::catalog::StandardCatalog;
use embsim_core::virtual_clock::{self, Actor, ClockMode};
use embsim_models::ads122u04_component::{ads122u04_framing, PUMP_POLL_VIRTUAL_US};
use rstest::rstest;
use vibes_behaviour::{behaviour, expect, Test};

mod uart_probe;
use uart_probe::{ProbeHandle, UartProbe};

static SUITE_LOCK: Mutex<()> = Mutex::new(());

fn suite_lock() -> MutexGuard<'static, ()> {
    SUITE_LOCK.lock().unwrap_or_else(|poisoned| {
        SUITE_LOCK.clear_poison();
        poisoned.into_inner()
    })
}

/// The virtual time each exchange is handed before it is read: 3.1 ms. The
/// longest exchange, the five configuration writes, is 15 bytes at
/// 115.2 kbaud (1.3 ms), and an answer adds at most two of the converter's
/// 250 µs polls and three bytes (0.8 ms); and it is off the polls, so no
/// armed instant falls on a deadline.
const SETTLE_NS: u64 = 3_100_000;
const _: () = assert!(!SETTLE_NS.is_multiple_of(PUMP_POLL_VIRTUAL_US * 1_000));

/// How long, in wall time, a case waits for the converter's protocol thread
/// to end after the system shuts down: sized for a hang, not a speed
/// (`TESTING.md` rule 3). The thread ends at its next poll.
const THREAD_END_HANG: Duration = Duration::from_secs(20);

// ---- SBAS752B, as the datasheet prints it ------------------------------

/// The synchronization word before every command (SBAS752B §8.5.1.4).
const SYNC: u8 = 0x55;
/// RESET, START/SYNC and RDATA (SBAS752B §8.5.3 Table 15).
const RESET: u8 = 0x06;
const START: u8 = 0x08;
const RDATA: u8 = 0x10;

/// WREG `0100 rrrx` and its data byte (SBAS752B §8.5.3.6).
fn wreg(register: u8, value: u8) -> [u8; 3] {
    [SYNC, 0x40 | (register << 1), value]
}

/// RREG `0010 rrrx` (SBAS752B §8.5.3.5).
fn rreg(register: u8) -> [u8; 2] {
    [SYNC, 0x20 | (register << 1)]
}

/// Configuration register 0: `MUX[3:0]` 7:4, `GAIN[2:0]` 3:1, `PGA_BYPASS`
/// 0 (SBAS752B §8.6.2.1 Table 18).
const fn config0(mux: u8, gain: u8, pga_bypass: bool) -> u8 {
    (mux << 4) | (gain << 1) | pga_bypass as u8
}

/// Configuration register 1: `DR[2:0]` 7:5, `MODE` 4, `CM` 3, `VREF[1:0]`
/// 2:1, `TS` 0 (SBAS752B §8.6.2.2 Table 19).
const fn config1(dr: u8, turbo: bool, continuous: bool, vref: u8) -> u8 {
    (dr << 5) | ((turbo as u8) << 4) | ((continuous as u8) << 3) | (vref << 1)
}

/// `VREF[1:0]`: the internal 2.048 V, `REFP` − `REFN`, the analog supply
/// (SBAS752B Table 19).
const VREF_INTERNAL: u8 = 0b00;
const VREF_EXTERNAL: u8 = 0b01;
const VREF_ANALOG_SUPPLY: u8 = 0b10;

/// The internal reference, 2.048 V (SBAS752B §8.3.3).
const INTERNAL_REFERENCE: Volts = 2.048;

/// `code = VIN · Gain · 2^23 / VREF`, truncated toward zero (SBAS752B
/// §8.5.2 Equation 8), inside full scale.
fn code(vin: Volts, gain: f64, vref: Volts) -> i32 {
    let code = (vin * gain * 8_388_608.0) / vref;
    assert!(
        code.abs() < 8_388_607.0,
        "{vin} V at gain {gain} over {vref} V clips"
    );
    code as i32
}

/// A 3-byte, least-significant-first, 24-bit two's-complement conversion
/// (SBAS752B §8.5.3.4 NOTE).
fn decode(bytes: &[u8]) -> i32 {
    let raw = u32::from(bytes[0]) | (u32::from(bytes[1]) << 8) | (u32::from(bytes[2]) << 16);
    ((raw << 8) as i32) >> 8
}

// ---- The bench ---------------------------------------------------------

/// Each pin of the TSSOP-16 table (SBAS752B p.3) and the net the bench puts
/// it on: its name, with `~RESET` spelled `NRESET`.
const PINS: [(&str, &str); 16] = [
    ("1", "GPIO1"),
    ("2", "GPIO0"),
    ("3", "NRESET"),
    ("4", "DGND"),
    ("5", "AVSS"),
    ("6", "AIN3"),
    ("7", "AIN2"),
    ("8", "REFN"),
    ("9", "REFP"),
    ("10", "AIN1"),
    ("11", "AIN0"),
    ("12", "AVDD"),
    ("13", "DVDD"),
    ("14", "DRDY"),
    ("15", "TX"),
    ("16", "RX"),
];

/// The converter as `U1`, by its ordering code, every pin on its own net.
fn netlist_text() -> String {
    let mut nets: BTreeMap<&str, &str> = BTreeMap::new();
    for (pin, name) in PINS {
        nets.insert(name, pin);
    }
    let nets: String = nets
        .iter()
        .enumerate()
        .map(|(code, (net, pin))| {
            format!(
                "    (net (code \"{}\") (name \"{net}\") (node (ref \"U1\") (pin \"{pin}\")))\n",
                code + 1
            )
        })
        .collect();
    format!(
        "(export (version \"E\")\n  (components\n    (comp (ref \"U1\") (value \"ADS122U04IPW\") \
         (libsource (lib \"Analog_ADC\") (part \"ADS122U04IPW\"))))\n  (nets\n{nets}))\n"
    )
}

/// The bench's digital supply, and its high on `~RESET`.
const DVDD: Volts = 3.3;
/// The bench's analog supply, unless a case moves it.
const AVDD: Volts = 3.3;

/// The two inputs, 8 mV apart around mid-supply: 8 mV against 2.048 V is
/// 2^15 codes at gain 1, and the difference of these two in binary floating
/// point gives a code whose every power-of-two multiple is exact.
const AIN0_VOLTS: Volts = 1.654;
const AIN1_VOLTS: Volts = 1.646;

/// The reference pins, 2 V apart (both inside `AVSS` − 0.1 V to `AVDD` +
/// 0.1 V, SBAS752B §6.3).
const REFP_VOLTS: Volts = 2.5;
const REFN_VOLTS: Volts = 0.5;

/// The output impedance of the bench source on `~RESET`: a bench pulse
/// generator's 50 Ω, a number this bench names.
const RESET_SOURCE_OHMS: f64 = 50.0;

/// A bench source the case can move: one pin, a Thevenin drive.
struct Lever {
    pins: [PinDecl; 1],
    volts: Volts,
    handle: Arc<Mutex<Option<PinHandle>>>,
}

impl Component for Lever {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let pin = io.pin("OUT")?;
        pin.set_drive(Some(TheveninDrive {
            volts: self.volts,
            impedance: RESET_SOURCE_OHMS,
        }));
        *self.handle.lock().unwrap() = Some(pin);
        Ok(())
    }
}

/// A running system with the converter and its host, read between settles.
struct Bench {
    system: SystemHandle,
    host: ProbeHandle,
    reset_pin: Arc<Mutex<Option<PinHandle>>>,
    actor: Option<Actor>,
    seen: usize,
    actors_before: usize,
    _lock: MutexGuard<'static, ()>,
}

impl Bench {
    /// The catalog's converter on the bench, its analog supply at `avdd`.
    fn catalog_part(avdd: Volts) -> Self {
        let lock = suite_lock();
        let actors_before = virtual_clock::registered_actors();
        virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);
        let parsed = netlist::parse(&netlist_text()).expect("the bench netlist parses");
        let board = Board::from_netlist(parsed, &StandardCatalog::base_registry())
            .expect("the catalog places the converter by its number");
        let ep = |endpoint: &str| EndpointRef::parse(endpoint).expect("endpoint parses");
        let harness = Harness::new()
            .power(ep("BENCH.DGND"), ep("B.U1.4"), 0.0)
            .power(ep("BENCH.AVSS"), ep("B.U1.5"), 0.0)
            .power(ep("BENCH.DVDD"), ep("B.U1.13"), DVDD)
            .power(ep("BENCH.AVDD"), ep("B.U1.12"), avdd)
            .power(ep("BENCH.AIN0"), ep("B.U1.11"), AIN0_VOLTS)
            .power(ep("BENCH.AIN1"), ep("B.U1.10"), AIN1_VOLTS)
            .power(ep("BENCH.REFP"), ep("B.U1.9"), REFP_VOLTS)
            .power(ep("BENCH.REFN"), ep("B.U1.8"), REFN_VOLTS)
            .connect_str("RST.OUT", "B.U1.3")
            .and_then(|harness| harness.connect_str("HOST.TX", "B.U1.16"))
            .and_then(|harness| harness.connect_str("B.U1.15", "HOST.RX"))
            .expect("endpoints parse");
        let host = ProbeHandle::new();
        let reset_pin = Arc::new(Mutex::new(None));
        let system = System::new()
            .board("B", board)
            .component("HOST", Box::new(probe(&host)))
            .component(
                "RST",
                Box::new(Lever {
                    pins: [PinDecl::analog_source("OUT")],
                    volts: DVDD,
                    handle: Arc::clone(&reset_pin),
                }),
            )
            .harness(harness);
        Self::start(system, host, reset_pin, actors_before, lock)
    }

    /// The force-gauge add-on from its project file, its bridge `diff`
    /// volts apart around 1.65 V, the host on its digital connector, and
    /// the bench's two fixes: the input jumpers closed and `~RESET` tied to
    /// the digital supply (`board/tests/ds2_live_force_path.rs`).
    fn ds2_project(diff: Volts) -> Self {
        let lock = suite_lock();
        let actors_before = virtual_clock::registered_actors();
        virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);
        let path: PathBuf = [
            env!("CARGO_MANIFEST_DIR"),
            "..",
            "boards",
            "projects",
            "ds2-addon.toml",
        ]
        .iter()
        .collect();
        let ep = |endpoint: &str| EndpointRef::parse(endpoint).expect("endpoint parses");
        let harness = Harness::new()
            .power(ep("BENCH.A0"), ep("DS2Addon.J2.3"), 1.65 + diff / 2.0)
            .power(ep("BENCH.A1"), ep("DS2Addon.J2.4"), 1.65 - diff / 2.0)
            .connect_str("HOST.TX", "DS2Addon.J1.3")
            .and_then(|harness| harness.connect_str("DS2Addon.J1.4", "HOST.RX"))
            .expect("endpoints parse");
        // The project names no switch, jumper or short, so this scenario is
        // the whole of it.
        let scenario = Scenario::default()
            .jumper("DS2Addon.JP1", JumperState::Closed)
            .jumper("DS2Addon.JP2", JumperState::Closed)
            .pin_short("DS2Addon.U1.3", "DS2Addon.U1.13");
        let host = ProbeHandle::new();
        let system = Project::load(&path)
            .expect("the project loads")
            .instantiate(&StandardCatalog)
            .expect("the project builds with the standard catalog")
            .component("HOST", Box::new(probe(&host)))
            .harness(harness)
            .scenario(scenario);
        Self::start(
            system,
            host,
            Arc::new(Mutex::new(None)),
            actors_before,
            lock,
        )
    }

    fn start(
        system: System,
        host: ProbeHandle,
        reset_pin: Arc<Mutex<Option<PinHandle>>>,
        actors_before: usize,
        lock: MutexGuard<'static, ()>,
    ) -> Self {
        let system = system.hold_time().start().expect("the bench starts");
        let actor = virtual_clock::register_actor("ads122u04-registers-case");
        system.release_time();
        virtual_clock::wait_virtual_ns(SETTLE_NS);
        Self {
            system,
            host,
            reset_pin,
            actor: Some(actor),
            seen: 0,
            actors_before,
            _lock: lock,
        }
    }

    /// Send `bytes`, settle, and return the `answer` bytes the converter
    /// sent back — exactly that many.
    fn exchange(&mut self, bytes: &[u8], answer: usize) -> Vec<u8> {
        self.host.send(bytes);
        virtual_clock::wait_virtual_ns(SETTLE_NS);
        let received = self.host.received();
        let new = received[self.seen..].to_vec();
        self.seen = received.len();
        assert_eq!(
            new.len(),
            answer,
            "{bytes:02x?} is answered by {answer} bytes, got {new:02x?}; frames {:?}",
            self.host.frames()
        );
        new
    }

    /// One conversion, by RDATA.
    fn rdata(&mut self) -> i32 {
        decode(&self.exchange(&[SYNC, RDATA], 3))
    }

    /// The five configuration registers, by RREG.
    fn registers(&mut self) -> [u8; 5] {
        let mut out = [0u8; 5];
        for (register, value) in (0u8..).zip(out.iter_mut()) {
            *value = self.exchange(&rreg(register), 1)[0];
        }
        out
    }

    /// Move the source on `~RESET` to `volts`, and settle.
    fn drive_reset_pin(&self, volts: Volts) {
        self.reset_pin
            .lock()
            .unwrap()
            .as_ref()
            .expect("the reset source has attached")
            .set_drive(Some(TheveninDrive {
                volts,
                impedance: RESET_SOURCE_OHMS,
            }));
        virtual_clock::wait_virtual_ns(SETTLE_NS);
    }

    /// A net's solved voltage.
    fn volts(&self, net: &str) -> Volts {
        match self.system.net_state(net) {
            Some(NetState::Analog(volts)) => volts,
            other => panic!("{net} must solve numerically, got {other:?}"),
        }
    }

    /// Assert the engine never stopped waiting for the case, shut the system
    /// down, and wait for the converter's protocol thread to end with it.
    fn finish(mut self) {
        drop(self.actor.take());
        let stalled: Vec<Finding> = self
            .system
            .findings()
            .into_iter()
            .filter(|finding| matches!(finding, Finding::QuiescenceTimeout { .. }))
            .collect();
        assert!(
            stalled.is_empty(),
            "the engine stopped waiting: {stalled:?}"
        );
        self.system.shutdown();
        let start = Instant::now();
        while virtual_clock::registered_actors() > self.actors_before {
            assert!(
                start.elapsed() < THREAD_END_HANG,
                "the converter's protocol thread outlived its system: {:?}",
                virtual_clock::registered_actor_names()
            );
            std::thread::sleep(Duration::from_millis(1));
        }
    }
}

/// The host: a UART probe at the converter's framing.
fn probe(handle: &ProbeHandle) -> UartProbe {
    UartProbe::new("TX", None, "RX", None, ads122u04_framing(), handle.clone())
}

/// What the configured cases write before they reset the part: gain 128 and
/// the analog supply as the reference, so a conversion that still had them
/// would read 2^7 · 2.048 / 3.3 times the reset one.
const CONFIGURED: [(u8, u8); 2] = [
    (0, config0(0b0000, 0b111, false)),
    (1, config1(0b110, false, true, VREF_ANALOG_SUPPLY)),
];

// ---- Cases -------------------------------------------------------------

/// How a case brings the part to its reset state.
#[derive(Debug, Clone, Copy)]
enum ResetBy {
    PowerUp,
    Command,
    Pin,
}

#[rstest]
#[case::as_powered_up(ResetBy::PowerUp)]
#[case::by_the_reset_command(ResetBy::Command)]
#[case::by_the_reset_pin(ResetBy::Pin)]
fn the_ads122u04_converts_at_its_reset_defaults(#[case] reset_by: ResetBy) {
    behaviour!(Test {
        id: "ads122u04.reset-defaults",
        covers: Some("models/src/ads122u04.rs#Ads122u04"),
        given: "an ADS122U04 from the catalog with inputs 8 millivolts apart, as it powers up \
                or after its host configures it and resets it by command or by its reset pin",
    });
    expect!(
        "registers-zero",
        "each of its five configuration registers reads back zero",
        "SBAS752B section 8.6.1: after power-up or any reset every register is at its \
         default, and every default is 0"
    );
    expect!(
        "gain-one-internal-reference",
        "a conversion is the input difference at gain 1 against the internal 2.048 volt \
         reference",
        "SBAS752B Tables 18 and 19: the zero settings are the first input pair, gain 1 and \
         the internal reference; the host that configures it first writes gain 128 and the \
         analog supply as the reference, which any reset undoes"
    );
    let mut bench = Bench::catalog_part(AVDD);
    if !matches!(reset_by, ResetBy::PowerUp) {
        for (register, value) in CONFIGURED {
            bench.exchange(&wreg(register, value), 0);
        }
        let configured = bench.rdata();
        assert_eq!(
            configured,
            code(AIN0_VOLTS - AIN1_VOLTS, 128.0, AVDD),
            "the writes took before the reset"
        );
        match reset_by {
            ResetBy::Command => {
                bench.exchange(&[SYNC, RESET], 0);
            }
            ResetBy::Pin => {
                bench.drive_reset_pin(0.0);
                bench.drive_reset_pin(DVDD);
            }
            ResetBy::PowerUp => unreachable!(),
        }
    }
    assert_eq!(bench.registers(), [0u8; 5]);
    let reset_code = bench.rdata();
    assert_eq!(
        reset_code,
        code(AIN0_VOLTS - AIN1_VOLTS, 1.0, INTERNAL_REFERENCE)
    );
    assert_eq!(reset_code, 1 << 15, "8 mV over 2.048 V is 2^15 codes");
    bench.finish();
}

#[rstest]
fn a_gain_write_scales_the_ads122u04s_code_by_exactly_the_gain() {
    behaviour!(Test {
        id: "ads122u04.gain-write",
        covers: Some("models/src/ads122u04.rs#Ads122u04"),
        given: "an ADS122U04 from the catalog with inputs 8 millivolts apart, its host \
                writing each of the eight gain settings in turn, the amplifier bypass bit clear \
                and then set",
    });
    expect!(
        "gain-ratio",
        "each conversion is exactly the gain written, 1 to 128, times the conversion at \
         gain 1",
        "SBAS752B Table 18: the gain bits select 2 to the power of their value, and a code \
         is proportional to the gain (section 8.5.2, Equation 8)"
    );
    expect!(
        "bypass-keeps-gain",
        "setting the amplifier bypass bit leaves every gain's conversion unchanged",
        "SBAS752B Table 18 and section 8.3.2: gains 1, 2 and 4 come from the \
         switched-capacitor stage with or without the amplifier, and the amplifier stays on \
         at 8 to 128 whatever the bit says"
    );
    let mut bench = Bench::catalog_part(AVDD);
    let at_gain_one = bench.rdata();
    assert_eq!(at_gain_one, 1 << 15, "8 mV over 2.048 V is 2^15 codes");
    for gain in 0u8..8 {
        bench.exchange(&wreg(0, config0(0b0000, gain, false)), 0);
        assert_eq!(bench.rdata(), at_gain_one << gain, "GAIN {gain:#05b}");
    }
    for gain in 0u8..8 {
        bench.exchange(&wreg(0, config0(0b0000, gain, true)), 0);
        assert_eq!(
            bench.rdata(),
            at_gain_one << gain,
            "GAIN {gain:#05b}, PGA_BYPASS set"
        );
    }
    bench.finish();
}

#[rstest]
#[case::analog_supply_at_3v3(3.3)]
#[case::analog_supply_at_5v(5.0)]
fn the_ads122u04_converts_against_the_reference_its_host_selects(#[case] avdd: Volts) {
    behaviour!(Test {
        id: "ads122u04.reference-select",
        covers: Some("models/src/ads122u04.rs#Ads122u04"),
        given: "an ADS122U04 from the catalog with inputs 8 millivolts apart, reference pins 2 \
                volts apart and a 3.3 or 5 volt analog supply, its host selecting each \
                reference in turn",
    });
    expect!(
        "internal",
        "with the internal reference selected a conversion is the input difference over \
         2.048 volts, whatever the analog supply"
    );
    expect!(
        "reference-pins",
        "with the reference pins selected a conversion is the input difference over the \
         voltage between them"
    );
    expect!(
        "analog-supply",
        "with the analog supply selected a conversion is the input difference over the \
         analog supply's voltage as the converter's own supply pins sense it",
        "SBAS752B Table 19 and section 8.3.3: the supply used as the reference is the \
         difference between the analog supply and analog ground pins"
    );
    let mut bench = Bench::catalog_part(avdd);
    let vin = AIN0_VOLTS - AIN1_VOLTS;
    // The reference reads the supply pins as the engine solved them.
    let sensed = bench.volts("B.AVDD") - bench.volts("B.AVSS");
    assert_eq!(sensed, avdd);
    for (vref, volts) in [
        (VREF_INTERNAL, INTERNAL_REFERENCE),
        (VREF_EXTERNAL, REFP_VOLTS - REFN_VOLTS),
        (VREF_ANALOG_SUPPLY, sensed),
    ] {
        bench.exchange(&wreg(1, config1(0, false, false, vref)), 0);
        assert_eq!(
            bench.rdata(),
            code(vin, 1.0, volts),
            "VREF {vref:#04b} against {volts} V"
        );
    }
    bench.finish();
}

/// MaD's firmware (`Firmware/MaDCore/src/IO/IO_ADS122U04.c`), as
/// `IO_ADS122U04_start` and `IO_ADS122U04_receiveConversion` run it on the
/// force-gauge channel: its `IO_ADS122U04_channelConfig` register values,
/// built there from the same fields.
const MAD_CONFIG: [u8; 5] = [
    // AINP = AIN0, AINN = AIN1; gain 128; the PGA on.
    config0(0b0000, 0b111, false),
    // 1000 SPS, normal mode, continuous; the analog supply as reference;
    // the temperature sensor off.
    config1(0b110, false, true, VREF_ANALOG_SUPPLY),
    // DRDY, the data counter, CRC, burn-out sources and IDACs off.
    0x00,
    // Both IDACs unrouted; manual data read mode.
    0x00,
    // Every GPIO an input, GPIO2 its data bit.
    0x00,
];

/// What the firmware's read-back compares, register by register: all of
/// each but the read-only `DRDY` flag (register 2, bit 7) and the GPIO data
/// bits that follow their pins (register 4, bits 2:0).
const MAD_VERIFY_MASK: [u8; 5] = [0xFF, 0xFF, 0x7F, 0xFF, 0xF8];

/// The bridge output the force path is proved at: 1 mV, which at gain 128
/// against 3.3 V is about 2^18.3 codes, well inside full scale (±25.8 mV).
const BRIDGE_VOLTS: Volts = 0.001;

#[rstest]
fn the_firmware_configures_the_force_path_to_its_expected_code() {
    behaviour!(Test {
        id: "ads122u04.mad-force-path",
        covers: Some("models/src/ads122u04.rs#Ads122u04"),
        given: "the force-gauge add-on from its project file, its bridge 1 millivolt apart on \
                a 3.3 volt analog side, set up by the bytes MaD's firmware sends at start-up, \
                then read once",
    });
    expect!(
        "read-back",
        "each register reads back what the firmware wrote, under the firmware's own \
         comparison masks",
        "the firmware sends the reset command, writes gain 128, the analog supply as the \
         reference, 1000 samples a second and continuous conversion, reads each register \
         back, starts the converter and asks for one conversion"
    );
    expect!(
        "force-code",
        "the data read is the code for the input difference at gain 128 against the \
         add-on's analog supply as its converter senses it",
        "the firmware runs the bridge ratiometric: the bridge's excitation is the analog \
         supply, which is also the converter's reference"
    );
    expect!(
        "firmware-signal",
        "the firmware's conversion of that code to nanovolts per volt is the bridge output \
         over its 3.3 volt excitation, within one nanovolt per volt"
    );
    let mut bench = Bench::ds2_project(BRIDGE_VOLTS);
    // IO_ADS122U04_start: reset, then the firmware's 100 ms wait.
    bench.host.send(&[SYNC, RESET]);
    virtual_clock::wait_virtual_ns(100_000_000);
    let writes: Vec<u8> = (0u8..)
        .zip(MAD_CONFIG)
        .flat_map(|(register, value)| wreg(register, value))
        .collect();
    bench.exchange(&writes, 0);
    let read_back = bench.registers();
    for (register, ((read, written), mask)) in read_back
        .into_iter()
        .zip(MAD_CONFIG)
        .zip(MAD_VERIFY_MASK)
        .enumerate()
    {
        assert_eq!(
            read & mask,
            written & mask,
            "register {register} read back {read:#04x}"
        );
    }
    bench.exchange(&[SYNC, START], 0);
    // IO_ADS122U04_receiveConversion.
    let counts = bench.rdata();

    let vin = bench.volts("DS2Addon.AIN0") - bench.volts("DS2Addon.AIN1");
    let analog_supply = bench.volts("DS2Addon.VDDA") - bench.volts("DS2Addon.VSS");
    assert!(
        (vin - BRIDGE_VOLTS).abs() < 1e-9,
        "the bridge reaches the inputs: {vin} V"
    );
    assert_eq!(analog_supply, 3.3);
    assert_eq!(counts, code(vin, 128.0, analog_supply));

    // signal[nV/V] = counts · 1e9 / (gain · 2^23), the firmware's integer
    // arithmetic, its gain read back from its own register 0.
    let gain = 1i64 << ((MAD_CONFIG[0] >> 1) & 0x7);
    let signal_nvv = i64::from(counts) * 1_000_000_000 / (gain << 23);
    let ideal_nvv = BRIDGE_VOLTS / 3.3 * 1e9;
    assert!(
        (signal_nvv as f64 - ideal_nvv).abs() <= 1.0,
        "the firmware reads {signal_nvv} nV/V for an ideal {ideal_nvv}"
    );
    bench.finish();
}
