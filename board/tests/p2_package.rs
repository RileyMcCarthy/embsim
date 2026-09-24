//! The P2 package as a node, on a bench: what `P2Package` declares and
//! delivers with no core inside it — the state any P2 is in before it runs
//! (`NODES.md` §2 "MCU node (P2)", §8 phase 2).
//!
//! Three facts a board puts on the package and the package hands to its
//! core: the rate on `XI` is the crystal; `RESN` and `VDD` are the reset
//! inputs; and every pad is a released bidirectional pin that a bench
//! driver can take without a fight. Each is asserted from outside — the
//! package's handle and the nets — never from inside a core.
//!
//! Every case runs in stepped mode (`TESTING.md` rule 9), in its own
//! binary (rule 5): the cases pin the process-global clock and its mode.

use std::sync::{Mutex, MutexGuard};
use std::time::{Duration, Instant};

use embsim_board::{
    AttachError, Component, ComponentNetIo, Finding, Harness, IdleDrive, Level, NetState, PinDecl,
    PinKind, PulseDirection, PulseSegment, PulseTrain, PulseTx, StreamRole, System, SystemHandle,
};
use embsim_boards::p2::{pin_name, P2Package, P2ResetState, NUM_PADS};
use embsim_core::virtual_clock::{self, ClockMode};
use rstest::rstest;
use vibes_behaviour::{behaviour, expect, Test};

// ============================================================
// Plumbing
// ============================================================

static SUITE_LOCK: Mutex<()> = Mutex::new(());

fn suite_lock() -> MutexGuard<'static, ()> {
    SUITE_LOCK.lock().unwrap_or_else(|poisoned| {
        SUITE_LOCK.clear_poison();
        poisoned.into_inner()
    })
}

fn stepped() {
    virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);
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

const SETTLE: Duration = Duration::from_secs(5);

/// The rate a bench oscillator puts on `XI`: the P2-EC32MB's 20 MHz.
const CRYSTAL_HZ: u32 = 20_000_000;

fn state(system: &SystemHandle, net: &str) -> NetState {
    system
        .net_state(net)
        .unwrap_or_else(|| panic!("net {net} exists"))
}

/// A bench clock: one push-pull pulse source (a clock buffer's output,
/// resting at a level, unlike a TCXO's capacitor-coupled clipped sine)
/// that publishes a 20 MHz train when the system starts, or a held train.
struct BenchClock {
    pins: [PinDecl; 1],
    tx: Option<PulseTx>,
    hz: u32,
}

impl BenchClock {
    fn new(hz: u32) -> Self {
        Self {
            pins: [PinDecl::digital_out("OUT").with_stream(StreamRole::PulseSource)],
            tx: None,
            hz,
        }
    }
}

impl Component for BenchClock {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        self.tx = Some(io.pulse_tx("OUT")?);
        Ok(())
    }

    fn start(&mut self) {
        let train = if self.hz == 0 {
            PulseTrain::IDLE
        } else {
            PulseTrain {
                pulses: PulseSegment {
                    emitted: 0,
                    freq_hz: self.hz,
                    total: None,
                    since_us: 0,
                },
                direction: PulseDirection::Forward,
            }
        };
        self.tx.as_ref().expect("attached").set_train(train);
    }
}

/// A bench pin resting driven at `level` from attach — a sink or a source
/// on one of the package's pads.
struct BenchDriver {
    pins: [PinDecl; 1],
}

impl BenchDriver {
    fn holding(level: Level) -> Self {
        Self {
            pins: [PinDecl::digital_out("A")
                .with_idle(IdleDrive::Thevenin(embsim_board::digital_drive(level)))],
        }
    }
}

impl Component for BenchDriver {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, _io: ComponentNetIo) -> Result<(), AttachError> {
        Ok(())
    }
}

// ============================================================
// The crystal
// ============================================================

/// A bench clock on `XI`: the package reports its rate as the crystal, and
/// a held train as no crystal.
#[rstest]
#[case::twenty_megahertz(CRYSTAL_HZ, Some(20_000_000))]
#[case::held(0, None)]
fn the_rate_on_xi_is_the_crystal_the_package_reports(
    #[case] hz: u32,
    #[case] expected: Option<u64>,
) {
    behaviour!(Test {
        id: "p2-package.xi-rate-is-the-crystal",
        covers: Some("boards/src/p2.rs#P2Package"),
        given: "a P2 package with no core, its XI pin wired to a bench clock that publishes \
                either a 20 megahertz train or a held train when the system starts",
    });
    expect!(
        "crystal-is-the-delivered-rate",
        "the package reports the train's rate as the crystal, and no crystal for a held \
         train",
        "the crystal a P2 multiplies is whatever rate the board puts on XI; a package \
         invents none of its own"
    );

    let _lock = suite_lock();
    stepped();
    let package = P2Package::held_in_reset();
    let handle = package.handle();
    let system = System::new()
        .component("P2", Box::new(package))
        .component("CLK", Box::new(BenchClock::new(hz)))
        .harness(
            Harness::new()
                .connect_str("CLK.OUT", "P2.XI")
                .expect("endpoints parse"),
        )
        .start()
        .expect("the bench starts");

    if expected.is_some() {
        assert!(
            wait_for(|| handle.crystal_hz() == expected, SETTLE),
            "the rate on XI reaches the package; got {:?}",
            handle.crystal_hz()
        );
    } else {
        std::thread::sleep(Duration::from_millis(50));
    }
    assert_eq!(handle.crystal_hz(), expected);
    drop(system);
}

// ============================================================
// Reset
// ============================================================

/// The reset inputs as the package projects them: `RESN` released and
/// `VDD` up is out of reset; a held `RESN` or an unsourced `VDD` is not.
#[rstest]
#[case::released_and_powered(Some(3.3), Some(1.8), Some(Level::High), Some(Level::High), true)]
#[case::reset_held(Some(0.0), Some(1.8), Some(Level::Low), Some(Level::High), false)]
#[case::core_unpowered(Some(3.3), None, Some(Level::High), None, false)]
#[case::nothing_connected(None, None, None, None, false)]
fn the_reset_state_is_resn_and_vdd_as_the_package_reads_them(
    #[case] resn_volts: Option<f64>,
    #[case] vdd_volts: Option<f64>,
    #[case] resn: Option<Level>,
    #[case] vdd: Option<Level>,
    #[case] out_of_reset: bool,
) {
    behaviour!(Test {
        id: "p2-package.reset-inputs",
        covers: Some("boards/src/p2.rs#P2ResetState"),
        given: "a P2 package with no core, its RESN and VDD pins each either held at a bench \
                voltage or left with nothing on them",
    });
    expect!(
        "resn-and-vdd-projected",
        "the package reports RESN and VDD each as high, low, or nothing reaching the pin, \
         by the engine's own level rule",
        "the package applies no threshold of its own; the datasheet's supply window is the \
         start gate's to cite"
    );
    expect!(
        "out-of-reset-needs-both",
        "the package is out of reset only with RESN released and VDD up",
        "a chip whose reset is held or whose core rail is absent does not run"
    );

    let _lock = suite_lock();
    stepped();
    let package = P2Package::held_in_reset();
    let handle = package.handle();
    let mut harness = Harness::new();
    if let Some(volts) = resn_volts {
        harness = harness.power(
            embsim_board::EndpointRef::parse("BENCH.RESN").unwrap(),
            embsim_board::EndpointRef::parse("P2.RESN").unwrap(),
            volts,
        );
    }
    if let Some(volts) = vdd_volts {
        harness = harness.power(
            embsim_board::EndpointRef::parse("BENCH.VDD").unwrap(),
            embsim_board::EndpointRef::parse("P2.VDD").unwrap(),
            volts,
        );
    }
    let system = System::new()
        .component("P2", Box::new(package))
        .harness(harness)
        .start()
        .expect("the bench starts");

    let expected = P2ResetState { resn, vdd };
    assert!(
        wait_for(|| handle.reset() == expected, SETTLE),
        "the package reads {expected:?}; got {:?}",
        handle.reset()
    );
    assert_eq!(handle.reset().out_of_reset(), out_of_reset);
    drop(system);
}

// ============================================================
// The pads
// ============================================================

/// Every pad of a package in reset is released: its net floats, and a
/// bench driver on one takes it without contention.
#[rstest]
fn every_pad_of_a_package_in_reset_is_released() {
    behaviour!(Test {
        id: "p2-package.pads-released-in-reset",
        covers: Some("boards/src/p2.rs#P2Package::held_in_reset"),
        given: "a P2 package with no core, sixty-three of its pads on nets of their own and \
                one pad wired to a bench pin holding a low level",
    });
    expect!(
        "unwired-pads-float",
        "every pad on a net of its own reads floating",
        "a chip out of reset floats every pin, so a package with no core presents nothing \
         on any pad"
    );
    expect!(
        "a-bench-driver-owns-the-pad",
        "the pad the bench pin holds reads driven low, and no contention is reported \
         anywhere",
        "a released pad is the absence of a drive, and the only source on that net is the \
         bench pin"
    );

    let _lock = suite_lock();
    stepped();
    let held = 5u8;
    let system = System::new()
        .component("P2", Box::new(P2Package::held_in_reset()))
        .component("SINK", Box::new(BenchDriver::holding(Level::Low)))
        .harness(
            Harness::new()
                .connect_str("SINK.A", &format!("P2.{}", pin_name(held)))
                .expect("endpoints parse"),
        )
        .start()
        .expect("the bench starts");
    std::thread::sleep(Duration::from_millis(50));

    for pin in 0..NUM_PADS as u8 {
        let net = format!("P2.{}", pin_name(pin));
        let expected = if pin == held {
            NetState::Driven(Level::Low)
        } else {
            NetState::Floating
        };
        assert_eq!(state(&system, &net), expected, "{net}");
    }
    let fights: Vec<Finding> = system
        .findings()
        .into_iter()
        .filter(|f| matches!(f, Finding::Contention { .. }))
        .collect();
    assert_eq!(fights, Vec::<Finding>::new());
    drop(system);
}

/// The one static fact a pad carries: it idles released, so a package
/// declares no drive at attach and a bench source on a pad is the pad's
/// only source. Asserted on the declaration so a change to it is a change
/// to this test.
#[rstest]
fn the_package_declares_every_pad_released() {
    behaviour!(Test {
        id: "p2-package.pad-declaration",
        covers: Some("boards/src/p2.rs#p2x8c4m64p_pins"),
        given: "the P2 package's pin declarations",
    });
    expect!(
        "sixty-four-released-bidirectional-pads",
        "all sixty-four pads are bidirectional pins declared to idle released, with no \
         stream role",
        "a P2 pad is high-impedance out of reset, and what it drives is decided by the core \
         at run time from its WRPIN word"
    );

    let package = P2Package::held_in_reset();
    let pads = &package.pins()[..NUM_PADS];
    assert_eq!(pads.len(), 64);
    for pad in pads {
        assert_eq!(pad.kind, PinKind::DigitalBidir, "{}", pad.number);
        assert_eq!(pad.idle, IdleDrive::Released, "{}", pad.number);
        assert_eq!(pad.stream, None, "{}", pad.number);
    }
}
