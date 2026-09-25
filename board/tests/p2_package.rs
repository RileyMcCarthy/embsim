//! The P2 package as a node, on a bench: what `P2Package` declares and
//! delivers with no core inside it — the state any P2 is in before it runs
//! (`NODES.md` §2 "MCU node (P2)", §8 phase 2).
//!
//! Three facts a board puts on the package and the package hands to its
//! core: the rate of the square wave on `XI` is the crystal; `RESN` and
//! `VDD` are the reset
//! inputs; and every pad is a released bidirectional pin that a bench
//! driver can take without a fight. Each is asserted from outside — the
//! package's handle and the nets — never from inside a core.
//!
//! And two things the package does with them (`NODES.md` §8 phase 4, §12
//! item 5): the **START gate** — a core is started, and its first wake
//! delivered, the datasheet's 3 ms restart delay after the instant `RESN`
//! reads released with `VDD` inside the datasheet's 1.7–1.9 V window, and
//! is held with the reason readable while either does not hold — and
//! **pads at their bank's supply** — a pad driven high is a source at the
//! voltage its `VIO_a_b` pin reads, and a pad in a bank whose supply names
//! no voltage presents nothing, the bank named once.
//!
//! Every case runs in stepped mode (`TESTING.md` rule 9), in its own
//! binary (rule 5): the cases pin the process-global clock and its mode.

use std::sync::{Mutex, MutexGuard};
use std::time::{Duration, Instant};

use std::sync::{Arc, Mutex as StdMutex};

use embsim_board::{
    AttachError, Component, ComponentNetIo, Drive, EndpointRef, Finding, Harness, Level, NetState,
    PeriodicSchedule, PinDecl, PinHandle, System, SystemHandle, TheveninDrive,
};
use embsim_boards::p2::{
    bank_of, bank_pin_name, pin_name, P2Core, P2Package, P2Pads, P2ResetState, PadDrive,
    StartState, NUM_PADS, P2_FAST_OHMS, P2_FAST_SINK_POINTS, P2_FAST_SOURCE_POINTS,
    P2_RESTART_DELAY_NS, P2_VDD_MAX_VOLTS, P2_VDD_MIN_VOLTS, P_HIGH_FAST, P_LOW_FAST,
};
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

/// The bench's return: `GND` held at 0 V. Ground is not implicit
/// (`DESIGN.md` rule 6), and every package pin's sense is its voltage
/// against `GND` — a bench that held none would hand the package nothing.
fn grounded(harness: Harness) -> Harness {
    harness.power(ep("BENCH.GND"), ep("P2.GND"), 0.0)
}

fn ep(endpoint: &str) -> EndpointRef {
    EndpointRef::parse(endpoint).expect("endpoint parses")
}

/// A bench supply that comes up at an instant: a `PowerOut` pin, released
/// from the build, held at `volts` from the wake at `at_ns` on.
struct BenchRail {
    pins: [PinDecl; 1],
    volts: f64,
    at_ns: u64,
}

impl BenchRail {
    fn rising_to(volts: f64, at_ns: u64) -> Self {
        Self {
            pins: [PinDecl::power_out("V").with_idle(None)],
            volts,
            at_ns,
        }
    }
}

impl Component for BenchRail {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let out = io.pin("V")?;
        let volts = self.volts;
        io.on_wake_ns(move |_| {
            out.set_drive(Some(TheveninDrive {
                volts,
                impedance: 0.1,
            }));
        });
        io.schedule_at_ns(self.at_ns);
        Ok(())
    }
}

/// A bench tick: a component with one released pad that asks for a wake
/// at `at_ns` and records that it fired. The engine fires wakes in instant
/// order and never advances past an undelivered publish, so once the tick
/// has fired every instant up to `at_ns` has been processed — the way a
/// stepped case proves "nothing happened by t" without a wall-clock sleep.
struct Tick {
    pins: [PinDecl; 1],
    at_ns: u64,
    fired: Arc<StdMutex<Option<u64>>>,
}

impl Tick {
    fn at(at_ns: u64) -> Self {
        Self {
            pins: [PinDecl::digital_out("T").with_idle(None)],
            at_ns,
            fired: Arc::default(),
        }
    }
}

impl Component for Tick {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let fired = Arc::clone(&self.fired);
        io.on_wake_ns(move |now| {
            *fired.lock().unwrap() = Some(now);
        });
        io.schedule_at_ns(self.at_ns);
        Ok(())
    }
}

/// A bench tick that reads something at its instant, on the engine thread
/// inside its own wake: the value as it stood at exactly `at_ns`, whatever
/// the engine does after.
struct ReadAt<T> {
    pins: [PinDecl; 1],
    at_ns: u64,
    read: Arc<dyn Fn() -> T + Send + Sync>,
    got: Arc<StdMutex<Option<(u64, T)>>>,
}

impl<T: Send + 'static> ReadAt<T> {
    fn new(at_ns: u64, read: impl Fn() -> T + Send + Sync + 'static) -> Self {
        Self {
            pins: [PinDecl::digital_out("T").with_idle(None)],
            at_ns,
            read: Arc::new(read),
            got: Arc::default(),
        }
    }
}

impl<T: Send + 'static> Component for ReadAt<T> {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let read = Arc::clone(&self.read);
        let got = Arc::clone(&self.got);
        io.on_wake_ns(move |now| {
            *got.lock().unwrap() = Some((now, read()));
        });
        io.schedule_at_ns(self.at_ns);
        Ok(())
    }
}

/// An analog bench probe on one net: the voltage the net sits at.
struct Probe {
    pins: [PinDecl; 1],
}

impl Probe {
    fn new() -> Self {
        Self {
            pins: [PinDecl::analog("A")],
        }
    }
}

impl Component for Probe {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, _io: ComponentNetIo) -> Result<(), AttachError> {
        Ok(())
    }
}

/// A core that records what the package does with it: the virtual instant
/// its `start` ran, and the instant of every wake it was delivered. It
/// asks for a wake at 1 ns the moment it is attached, as a core that runs
/// from its first instant does.
#[derive(Default)]
struct Recorder {
    started_at_ns: Arc<StdMutex<Option<u64>>>,
    woke_at_ns: Arc<StdMutex<Vec<u64>>>,
}

impl P2Core for Recorder {
    fn attach(&mut self, pads: P2Pads) -> Result<(), AttachError> {
        let woke = Arc::clone(&self.woke_at_ns);
        pads.on_wake_ns(move |now| woke.lock().unwrap().push(now));
        pads.schedule_at_ns(1);
        Ok(())
    }

    fn start(&mut self) {
        *self.started_at_ns.lock().unwrap() = Some(virtual_clock::virtual_ns());
    }
}

/// A core that drives one pad high, fast, the moment it is started, the
/// way a guest's `drvh` does: through the bank table the package hands it.
struct PadDriver {
    pin: u8,
    pads: Option<P2Pads>,
    handle: Option<PinHandle>,
}

impl PadDriver {
    fn on(pin: u8) -> Self {
        Self {
            pin,
            pads: None,
            handle: None,
        }
    }
}

impl P2Core for PadDriver {
    fn attach(&mut self, pads: P2Pads) -> Result<(), AttachError> {
        self.handle = Some(pads.pad(self.pin)?);
        self.pads = Some(pads);
        Ok(())
    }

    fn start(&mut self) {
        let banks = self.pads.as_ref().expect("attached").bank_supplies();
        let drive = match banks.pad_drive(self.pin, P_HIGH_FAST | P_LOW_FAST, true, true) {
            PadDrive::Thevenin(drive) => Some(drive),
            PadDrive::Released | PadDrive::CurrentSource(_) => None,
        };
        self.handle.as_ref().expect("attached").set_drive(drive);
    }
}

/// A bench clock: one push-pull output (a clock buffer's, swinging rail to
/// rail at its own 25 Ω, unlike a TCXO's capacitor-coupled clipped sine)
/// that drives a 20 MHz square wave when the system starts, or a held one.
struct BenchClock {
    pins: [PinDecl; 1],
    out: Option<PinHandle>,
    hz: u32,
}

impl BenchClock {
    fn new(hz: u32) -> Self {
        Self {
            pins: [PinDecl::digital_out("OUT")],
            out: None,
            hz,
        }
    }
}

impl Component for BenchClock {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        self.out = Some(io.pin("OUT")?);
        Ok(())
    }

    fn start(&mut self) {
        let segment = if self.hz == 0 {
            PeriodicSchedule::IDLE
        } else {
            PeriodicSchedule {
                emitted: 0,
                freq_hz: self.hz,
                total: None,
                since_ns: 0,
            }
        };
        self.out.as_ref().expect("attached").drive(Drive::Periodic {
            hi: TheveninDrive {
                volts: 3.3,
                impedance: 25.0,
            },
            lo: TheveninDrive {
                volts: 0.0,
                impedance: 25.0,
            },
            segment,
        });
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
            pins: [PinDecl::digital_out("A").with_idle(Some(embsim_board::digital_drive(level)))],
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

/// A bench logic line that steps through a schedule: driven at the first
/// level from the build, then at each `(instant, level)` from its wake at
/// that instant — a reset button, a supervisor's output.
struct BenchLine {
    pins: [PinDecl; 1],
    steps: Vec<(u64, Level)>,
}

impl BenchLine {
    fn stepping(first: Level, steps: &[(u64, Level)]) -> Self {
        Self {
            pins: [PinDecl::digital_out("A").with_idle(Some(embsim_board::digital_drive(first)))],
            steps: steps.to_vec(),
        }
    }
}

impl Component for BenchLine {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let out = io.pin("A")?;
        let steps = self.steps.clone();
        io.on_wake_ns(move |now| {
            if let Some(&(_, level)) = steps.iter().find(|(at, _)| *at == now) {
                out.set_drive(Some(embsim_board::digital_drive(level)));
            }
        });
        for &(at_ns, _) in &self.steps {
            io.schedule_at_ns(at_ns);
        }
        Ok(())
    }
}

// ============================================================
// The crystal
// ============================================================

/// A bench clock on `XI`: the package reports its rate as the crystal, and
/// a held clock as no crystal.
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
        given: "a P2 package with no core, its XI pin wired to a bench clock that drives \
                either a 20 megahertz square wave or a held one when the system starts",
    });
    expect!(
        "crystal-is-the-delivered-rate",
        "the package reports the clock's rate as the crystal, and no crystal for a held \
         clock",
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
        .harness(grounded(
            Harness::new()
                .connect_str("CLK.OUT", "P2.XI")
                .expect("endpoints parse"),
        ))
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

/// The reset inputs as the package reads them: `RESN` released and
/// `VDD` at a voltage inside its window is out of reset; a held `RESN`,
/// an unsourced `VDD`, or a `VDD` outside the window is not.
#[rstest]
#[case::released_and_powered(Some(3.3), Some(1.8), Some(Level::High), true)]
#[case::reset_held(Some(0.0), Some(1.8), Some(Level::Low), false)]
#[case::core_unpowered(Some(3.3), None, Some(Level::High), false)]
#[case::core_under_its_window(Some(3.3), Some(1.2), Some(Level::High), false)]
#[case::core_over_its_window(Some(3.3), Some(3.3), Some(Level::High), false)]
#[case::nothing_connected(None, None, None, false)]
fn the_reset_state_is_resn_and_vdd_as_the_package_reads_them(
    #[case] resn_volts: Option<f64>,
    #[case] vdd_volts: Option<f64>,
    #[case] resn: Option<Level>,
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
        "the package reports RESN as high, low, or nothing reaching the pin, and VDD as the \
         voltage it is handed against GND",
        "RESN's level is the package's own projection through its declared thresholds; VDD \
         declares none, and its voltage is what the START gate's window is read against"
    );
    expect!(
        "out-of-reset-needs-both",
        "the package is out of reset only with RESN released and VDD inside the datasheet's \
         1.7 to 1.9 volt window",
        "a chip whose reset is held or whose core rail is absent, under or over its window \
         does not run"
    );

    let _lock = suite_lock();
    stepped();
    let package = P2Package::held_in_reset();
    let handle = package.handle();
    let mut harness = grounded(Harness::new());
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

    let expected = P2ResetState { resn, vdd_volts };
    assert!(
        wait_for(|| handle.reset() == expected, SETTLE),
        "the package reads {expected:?}; got {:?}",
        handle.reset()
    );
    assert_eq!(handle.reset().out_of_reset(), out_of_reset);
    if out_of_reset {
        assert!(
            wait_for(|| handle.started_at_ns().is_some(), SETTLE),
            "{:?}",
            handle.start_state()
        );
    }
    assert_eq!(
        handle.start_state(),
        if out_of_reset {
            StartState::Started {
                at_ns: P2_RESTART_DELAY_NS,
            }
        } else {
            StartState::Held { reset: expected }
        },
        "a core with no run of its own is still gated: started the restart delay after the \
         build when the inputs allow it, held with the inputs as the reason when they do not"
    );
    drop(system);
}

// ============================================================
// The START gate
// ============================================================

/// A core is started, and its first wake delivered, at the instant `VDD`
/// enters its window with `RESN` released — and is held, its wake with it,
/// while `VDD` reads under or over the window.
#[rstest]
#[case::inside_the_window(1.8, true)]
#[case::at_the_lower_bound(P2_VDD_MIN_VOLTS, true)]
#[case::at_the_upper_bound(P2_VDD_MAX_VOLTS, true)]
#[case::under_the_window(1.2, false)]
#[case::over_the_window(2.5, false)]
fn the_core_is_started_when_vdd_enters_its_window_and_held_outside_it(
    #[case] vdd_volts: f64,
    #[case] runs: bool,
) {
    behaviour!(Test {
        id: "p2-package.start-gate-vdd-window",
        covers: Some("boards/src/p2.rs#P2Package::follow_reset"),
        given: "a P2 package around a core asking for its first wake, RESN released from the \
                bench, and a bench VDD rail rising to a set voltage at two milliseconds",
    });
    expect!(
        "held-before-the-rail",
        "with time held the core has not been started and reports itself held with VDD \
         reaching no voltage",
        "the datasheet gives the core supply a 1.7 to 1.9 volt window, and a rail that is \
         down is outside it"
    );
    expect!(
        "started-at-the-instant",
        "a rail inside the window starts the core exactly three milliseconds after it came \
         up, and the core's first wake lands at that instant",
        "the chip restarts 3 ms after its reset releases, and a wake asked for before it could \
         run is held and lands at the start, never earlier"
    );
    expect!(
        "held-outside-the-window",
        "a rail outside the window leaves the core unstarted and unwoken past the instant one \
         inside it would have started it, its voltage the reason",
        "the proof is made at a bench wake the engine has fired, so the clock is known to \
         have reached that instant"
    );

    let _lock = suite_lock();
    stepped();
    let core = Recorder::default();
    let started_at = Arc::clone(&core.started_at_ns);
    let woke_at = Arc::clone(&core.woke_at_ns);
    let package = P2Package::new(core);
    let handle = package.handle();
    const RAIL_AT_NS: u64 = 2_000_000;
    // The instant the negative cases are proved at: a millisecond past
    // the end of the restart delay a rail inside the window would have
    // started, with nothing else scheduled between.
    const PROOF_AT_NS: u64 = RAIL_AT_NS + P2_RESTART_DELAY_NS + 1_000_000;
    let tick = Tick::at(PROOF_AT_NS);
    let ticked = Arc::clone(&tick.fired);
    let system = System::new()
        .component("P2", Box::new(package))
        .component(
            "RAIL",
            Box::new(BenchRail::rising_to(vdd_volts, RAIL_AT_NS)),
        )
        .component("TICK", Box::new(tick))
        .harness(grounded(
            Harness::new()
                .power(ep("BENCH.RESN"), ep("P2.RESN"), 3.3)
                .connect_str("RAIL.V", "P2.VDD")
                .expect("endpoints parse"),
        ))
        .hold_time()
        .start()
        .expect("the bench starts");

    assert!(
        wait_for(|| handle.reset().resn == Some(Level::High), SETTLE),
        "{:?}",
        handle.reset()
    );
    assert_eq!(
        handle.start_state(),
        StartState::Held {
            reset: P2ResetState {
                resn: Some(Level::High),
                vdd_volts: None,
            }
        }
    );
    assert_eq!(*started_at.lock().unwrap(), None);
    assert_eq!(*woke_at.lock().unwrap(), Vec::<u64>::new());

    system.release_time();
    assert!(
        wait_for(|| handle.reset().vdd_volts == Some(vdd_volts), SETTLE),
        "the rail came up: {:?}",
        handle.reset()
    );
    if runs {
        assert!(
            wait_for(|| !woke_at.lock().unwrap().is_empty(), SETTLE),
            "the core's held wake lands at the start"
        );
        let start_ns = RAIL_AT_NS + P2_RESTART_DELAY_NS;
        assert_eq!(
            handle.start_state(),
            StartState::Started { at_ns: start_ns }
        );
        assert_eq!(*started_at.lock().unwrap(), Some(start_ns));
        assert_eq!(*woke_at.lock().unwrap(), vec![start_ns]);
    } else {
        // No START by a millisecond past the delay a released reset would
        // have counted out: the clock is known to have reached the tick,
        // and no release was ever counted.
        assert!(
            wait_for(|| ticked.lock().unwrap().is_some(), SETTLE),
            "the bench tick at {PROOF_AT_NS} ns fires"
        );
        assert_eq!(*ticked.lock().unwrap(), Some(PROOF_AT_NS));
        assert_eq!(
            handle.start_state(),
            StartState::Held {
                reset: P2ResetState {
                    resn: Some(Level::High),
                    vdd_volts: Some(vdd_volts),
                }
            },
            "held at {PROOF_AT_NS} ns"
        );
        assert_eq!(*started_at.lock().unwrap(), None);
        assert_eq!(*woke_at.lock().unwrap(), Vec::<u64>::new());
    }
    drop(system);
}

/// A held reset holds the core whatever the rail does.
#[rstest]
fn the_core_is_held_while_resn_is_low_with_vdd_in_its_window() {
    behaviour!(Test {
        id: "p2-package.start-gate-resn",
        covers: Some("boards/src/p2.rs#P2Package::follow_reset"),
        given: "a P2 package around a core that asks for a wake at its first nanosecond, VDD \
                held at 1.8 volts and RESN held low from the bench",
    });
    expect!(
        "held-by-reset",
        "a millisecond in the core is not started and its wake not delivered, and the package \
         reports itself held with reset low",
        "reset low holds every cog disabled whatever the supplies read; the proof is made at \
         a bench wake the engine has fired"
    );

    let _lock = suite_lock();
    stepped();
    let core = Recorder::default();
    let started_at = Arc::clone(&core.started_at_ns);
    let woke_at = Arc::clone(&core.woke_at_ns);
    let package = P2Package::new(core);
    let handle = package.handle();
    // The instant the proof is made at: a millisecond in, with the
    // supplies up from the build and nothing scheduled but the tick.
    const PROOF_AT_NS: u64 = 1_000_000;
    let tick = Tick::at(PROOF_AT_NS);
    let ticked = Arc::clone(&tick.fired);
    let system = System::new()
        .component("P2", Box::new(package))
        .component("TICK", Box::new(tick))
        .harness(grounded(
            Harness::new()
                .power(ep("BENCH.RESN"), ep("P2.RESN"), 0.0)
                .power(ep("BENCH.VDD"), ep("P2.VDD"), 1.8),
        ))
        .start()
        .expect("the bench starts");
    assert!(
        wait_for(|| handle.reset().vdd_volts == Some(1.8), SETTLE),
        "{:?}",
        handle.reset()
    );
    assert!(
        wait_for(|| ticked.lock().unwrap().is_some(), SETTLE),
        "the bench tick at {PROOF_AT_NS} ns fires"
    );
    assert_eq!(*ticked.lock().unwrap(), Some(PROOF_AT_NS));
    assert_eq!(
        handle.start_state(),
        StartState::Held {
            reset: P2ResetState {
                resn: Some(Level::Low),
                vdd_volts: Some(1.8),
            }
        },
        "held at {PROOF_AT_NS} ns"
    );
    assert_eq!(*started_at.lock().unwrap(), None);
    assert_eq!(*woke_at.lock().unwrap(), Vec::<u64>::new());
    drop(system);
}

/// The chip restarts the datasheet's 3 ms after `RESN` rises: a core is
/// started there, its first wake landing with it, and reads as restarting
/// in between.
#[rstest]
fn the_core_starts_three_milliseconds_after_resn_rises() {
    behaviour!(Test {
        id: "p2-package.restart-delay",
        covers: Some("boards/src/p2.rs#P2Package::try_begin"),
        given: "a P2 package around a core asking for its first wake, VDD held at 1.8 volts \
                from the bench, and RESN driven low from the build and released at two \
                milliseconds",
    });
    expect!(
        "restarting-inside-the-delay",
        "a millisecond after the release the core is unstarted and the package reports the \
         release instant and the start it leads to",
        "the datasheet's RESN row: the Propeller restarts 3 ms after RESN goes from low to \
         high"
    );
    expect!(
        "started-three-milliseconds-on",
        "the core is started exactly three milliseconds after the release, and its first wake \
         lands at that instant"
    );

    let _lock = suite_lock();
    stepped();
    let core = Recorder::default();
    let started_at = Arc::clone(&core.started_at_ns);
    let woke_at = Arc::clone(&core.woke_at_ns);
    let package = P2Package::new(core);
    let handle = package.handle();
    const RISE_AT_NS: u64 = 2_000_000;
    const PROOF_AT_NS: u64 = RISE_AT_NS + 1_000_000;
    let reader = {
        let handle = handle.clone();
        let started_at = Arc::clone(&started_at);
        ReadAt::new(PROOF_AT_NS, move || {
            (handle.start_state(), *started_at.lock().unwrap())
        })
    };
    let read = Arc::clone(&reader.got);
    let system = System::new()
        .component("P2", Box::new(package))
        .component(
            "RST",
            Box::new(BenchLine::stepping(
                Level::Low,
                &[(RISE_AT_NS, Level::High)],
            )),
        )
        .component("READ", Box::new(reader))
        .harness(grounded(
            Harness::new()
                .power(ep("BENCH.VDD"), ep("P2.VDD"), 1.8)
                .connect_str("RST.A", "P2.RESN")
                .expect("endpoints parse"),
        ))
        .start()
        .expect("the bench starts");

    let start_ns = RISE_AT_NS + P2_RESTART_DELAY_NS;
    assert!(
        wait_for(|| !woke_at.lock().unwrap().is_empty(), SETTLE),
        "the core's held wake lands at the start; {:?}",
        handle.start_state()
    );
    assert_eq!(
        *read.lock().unwrap(),
        Some((
            PROOF_AT_NS,
            (
                StartState::Restarting {
                    released_at_ns: RISE_AT_NS,
                    starts_at_ns: start_ns,
                },
                None
            )
        )),
        "a millisecond into the delay"
    );
    assert_eq!(
        handle.start_state(),
        StartState::Started { at_ns: start_ns }
    );
    assert_eq!(*started_at.lock().unwrap(), Some(start_ns));
    assert_eq!(*woke_at.lock().unwrap(), vec![start_ns]);
    drop(system);
}

/// A release shorter than the restart delay starts nothing: the delay is
/// counted again from the next release.
#[rstest]
fn a_reset_release_shorter_than_the_restart_delay_starts_nothing() {
    behaviour!(Test {
        id: "p2-package.restart-glitch",
        covers: Some("boards/src/p2.rs#P2Package::follow_reset"),
        given: "a P2 core asking for its first wake, VDD at 1.8 volts, and RESN low from the \
                build, released at one millisecond, low again at two, and released for good at \
                six",
    });
    expect!(
        "glitch-starts-nothing",
        "at 4.5 milliseconds, past the end of a delay counted from the first release, the \
         core is unstarted, unwoken, and reported held with RESN low",
        "a reset that re-asserts before the restart delay is out cancels the restart"
    );
    expect!(
        "counted-from-the-last-release",
        "the core is started three milliseconds after the final release, at nine milliseconds"
    );

    let _lock = suite_lock();
    stepped();
    let core = Recorder::default();
    let started_at = Arc::clone(&core.started_at_ns);
    let woke_at = Arc::clone(&core.woke_at_ns);
    let package = P2Package::new(core);
    let handle = package.handle();
    const PROOF_AT_NS: u64 = 4_500_000;
    const FINAL_RISE_NS: u64 = 6_000_000;
    let reader = {
        let handle = handle.clone();
        let started_at = Arc::clone(&started_at);
        let woke_at = Arc::clone(&woke_at);
        ReadAt::new(PROOF_AT_NS, move || {
            (
                handle.start_state(),
                *started_at.lock().unwrap(),
                woke_at.lock().unwrap().len(),
            )
        })
    };
    let read = Arc::clone(&reader.got);
    let system = System::new()
        .component("P2", Box::new(package))
        .component(
            "RST",
            Box::new(BenchLine::stepping(
                Level::Low,
                &[
                    (1_000_000, Level::High),
                    (2_000_000, Level::Low),
                    (FINAL_RISE_NS, Level::High),
                ],
            )),
        )
        .component("READ", Box::new(reader))
        .harness(grounded(
            Harness::new()
                .power(ep("BENCH.VDD"), ep("P2.VDD"), 1.8)
                .connect_str("RST.A", "P2.RESN")
                .expect("endpoints parse"),
        ))
        .start()
        .expect("the bench starts");

    assert!(
        wait_for(|| !woke_at.lock().unwrap().is_empty(), SETTLE),
        "the core starts after the final release; {:?}",
        handle.start_state()
    );
    assert_eq!(
        *read.lock().unwrap(),
        Some((
            PROOF_AT_NS,
            (
                StartState::Held {
                    reset: P2ResetState {
                        resn: Some(Level::Low),
                        vdd_volts: Some(1.8),
                    }
                },
                None,
                0
            )
        )),
        "held at {PROOF_AT_NS} ns, past the end of a delay counted from the first release"
    );
    let start_ns = FINAL_RISE_NS + P2_RESTART_DELAY_NS;
    assert_eq!(start_ns, 9_000_000);
    assert_eq!(
        handle.start_state(),
        StartState::Started { at_ns: start_ns }
    );
    assert_eq!(*started_at.lock().unwrap(), Some(start_ns));
    assert_eq!(*woke_at.lock().unwrap(), vec![start_ns]);
    drop(system);
}

/// A bench supply that steps: a `PowerOut` pin held at the first voltage
/// from the build, then at each `(instant, volts)` from its wake there.
struct BenchSupply {
    pins: [PinDecl; 1],
    steps: Vec<(u64, f64)>,
}

impl BenchSupply {
    fn stepping(first: f64, steps: &[(u64, f64)]) -> Self {
        Self {
            pins: [PinDecl::power_out("V").with_idle(Some(TheveninDrive {
                volts: first,
                impedance: 0.1,
            }))],
            steps: steps.to_vec(),
        }
    }
}

impl Component for BenchSupply {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let out = io.pin("V")?;
        let steps = self.steps.clone();
        io.on_wake_ns(move |now| {
            if let Some(&(_, volts)) = steps.iter().find(|(at, _)| *at == now) {
                out.set_drive(Some(TheveninDrive {
                    volts,
                    impedance: 0.1,
                }));
            }
        });
        for &(at_ns, _) in &self.steps {
            io.schedule_at_ns(at_ns);
        }
        Ok(())
    }
}

/// A core that runs: woken every millisecond from its start until
/// `until_ns`, recording each wake and the instant the package held it.
struct Ticking {
    until_ns: u64,
    woke_at_ns: Arc<StdMutex<Vec<u64>>>,
    held_at_ns: Arc<StdMutex<Option<u64>>>,
}

impl Ticking {
    fn until(until_ns: u64) -> Self {
        Self {
            until_ns,
            woke_at_ns: Arc::default(),
            held_at_ns: Arc::default(),
        }
    }
}

impl P2Core for Ticking {
    fn attach(&mut self, pads: P2Pads) -> Result<(), AttachError> {
        let woke = Arc::clone(&self.woke_at_ns);
        let until = self.until_ns;
        let arm = pads.clone();
        pads.on_wake_ns(move |now| {
            woke.lock().unwrap().push(now);
            if now < until {
                arm.schedule_at_ns(now + 1_000_000);
            }
        });
        pads.schedule_at_ns(1);
        Ok(())
    }

    fn reset(&mut self) {
        *self.held_at_ns.lock().unwrap() = Some(virtual_clock::virtual_ns());
    }
}

/// `VDD` leaving its window while the core runs and `RESN` is not asserted
/// is a brownout without a reset: the package reports it and holds the
/// core. A `RESN` asserted first is a reset, and holds nothing.
#[rstest]
#[case::resn_released(false)]
#[case::resn_asserted_first(true)]
fn a_core_rail_that_leaves_its_window_while_running_holds_the_core(
    #[case] resn_asserted_first: bool,
) {
    behaviour!(Test {
        id: "p2-package.brownout-without-reset",
        covers: Some("boards/src/p2.rs#P2Package::watch_brownout"),
        given: "a running P2 core woken every millisecond whose VDD falls from 1.8 to 1.2 \
                volts at six and a half milliseconds, RESN either released throughout or pulled \
                low just before",
    });
    expect!(
        "reported-with-reset-released",
        "with RESN released the package reports a brownout without a reset at the instant \
         VDD left its window, with the voltage it read",
        "the datasheet's core supply window is 1.7 to 1.9 volts, and a chip run outside it \
         without its reset asserted is what a reset supervisor exists to prevent"
    );
    expect!(
        "core-held",
        "with RESN released the core is held at that instant and woken no more after it"
    );
    expect!(
        "reset-first-holds-nothing",
        "with RESN pulled low first the package reports no brownout and the core keeps being \
         woken"
    );

    let _lock = suite_lock();
    stepped();
    const MS: u64 = 1_000_000;
    let core = Ticking::until(10 * MS);
    let woke_at = Arc::clone(&core.woke_at_ns);
    let held_at = Arc::clone(&core.held_at_ns);
    let package = P2Package::new(core);
    let handle = package.handle();
    let late = {
        let woke_at = Arc::clone(&woke_at);
        ReadAt::new(8 * MS, move || woke_at.lock().unwrap().clone())
    };
    let late_read = Arc::clone(&late.got);
    let resn_steps: &[(u64, Level)] = if resn_asserted_first {
        &[(6_200_000, Level::Low)]
    } else {
        &[]
    };
    let system = System::new()
        .component("P2", Box::new(package))
        .component(
            "VDD",
            Box::new(BenchSupply::stepping(1.8, &[(6_500_000, 1.2)])),
        )
        .component(
            "RST",
            Box::new(BenchLine::stepping(Level::High, resn_steps)),
        )
        .component("LATE", Box::new(late))
        .harness(grounded(
            Harness::new()
                .connect_str("VDD.V", "P2.VDD")
                .expect("endpoints parse")
                .connect_str("RST.A", "P2.RESN")
                .expect("endpoints parse"),
        ))
        .start()
        .expect("the bench starts");

    assert!(
        wait_for(|| late_read.lock().unwrap().is_some(), SETTLE),
        "the bench tick at 8 ms fires; {:?}",
        handle.start_state()
    );
    let (at, woke_by_8ms) = late_read.lock().unwrap().clone().unwrap();
    assert_eq!(at, 8 * MS);
    if resn_asserted_first {
        assert_eq!(
            handle.start_state(),
            StartState::Started {
                at_ns: P2_RESTART_DELAY_NS
            }
        );
        assert_eq!(*held_at.lock().unwrap(), None);
        // Woken past the drop (the 8 ms wake shares the reading's instant
        // and may land after it).
        assert!(
            woke_by_8ms.starts_with(&[3 * MS, 4 * MS, 5 * MS, 6 * MS, 7 * MS]),
            "{woke_by_8ms:?}"
        );
    } else {
        assert_eq!(
            handle.start_state(),
            StartState::BrownoutWithoutReset {
                started_at_ns: P2_RESTART_DELAY_NS,
                at_ns: 6_500_000,
                reset: P2ResetState {
                    resn: Some(Level::High),
                    vdd_volts: Some(1.2),
                },
            }
        );
        assert_eq!(handle.started_at_ns(), Some(P2_RESTART_DELAY_NS));
        assert_eq!(*held_at.lock().unwrap(), Some(6_500_000));
        assert_eq!(woke_by_8ms, vec![3 * MS, 4 * MS, 5 * MS, 6 * MS]);
    }
    drop(system);
}

// ============================================================
// Pads at their bank's supply
// ============================================================

/// A pad driven high is a source at the voltage its bank's supply pin
/// reads; a pad in a bank whose supply pin reaches no voltage presents
/// nothing, and the package names the bank once.
#[rstest]
fn a_pad_drives_high_at_its_banks_supply_and_nothing_in_an_unpowered_bank() {
    behaviour!(Test {
        id: "p2-package.pad-at-bank-supply",
        covers: Some("boards/src/p2.rs#BankSupplies::pad_drive"),
        given: "a P2 package out of reset around a core driving one pad high at start, with \
                that pad's bank supply pin at 1.8 volts from the bench or left unconnected",
    });
    expect!(
        "high-is-the-bank-voltage",
        "the pad in the 1.8 volt bank sits at 1.8 volts",
        "a pad's driver is powered from its bank's supply pin, whatever that pin reads"
    );
    expect!(
        "unpowered-bank-drives-nothing",
        "the pad in the unsupplied bank floats and the package names that bank's supply \
         pin as driven without a supply",
        "a driver with no supply can neither source nor sink, and a nominal voltage in its \
         place would be invented"
    );

    let _lock = suite_lock();
    let powered: u8 = 4;
    let unpowered: u8 = 8;
    for (pin, volts) in [(powered, Some(1.8)), (unpowered, None)] {
        // Each bench from virtual zero: the first one's clock ran to its
        // START instant.
        stepped();
        let package = P2Package::new(PadDriver::on(pin));
        let handle = package.handle();
        let mut harness = grounded(Harness::new())
            .power(ep("BENCH.RESN"), ep("P2.RESN"), 3.3)
            .power(ep("BENCH.VDD"), ep("P2.VDD"), 1.8)
            .connect_str("PROBE.A", &format!("P2.{}", pin_name(pin)))
            .expect("endpoints parse");
        if let Some(volts) = volts {
            harness = harness.power(
                ep("BENCH.VIO"),
                ep(&format!("P2.{}", bank_pin_name(bank_of(pin)))),
                volts,
            );
        }
        let system = System::new()
            .component("P2", Box::new(package))
            .component("PROBE", Box::new(Probe::new()))
            .harness(harness)
            .start()
            .expect("the bench starts");
        let net = format!("P2.{}", pin_name(pin));
        match volts {
            Some(volts) => {
                assert!(
                    wait_for(
                        || matches!(state(&system, &net), NetState::Analog(v) if (v - volts).abs() < 1e-9),
                        SETTLE
                    ),
                    "{net} sits at the bank's {volts} V; got {:?}",
                    state(&system, &net)
                );
                assert_eq!(handle.bank_volts(bank_of(pin)), Some(volts));
                assert_eq!(handle.unpowered_banks_driven(), Vec::<usize>::new());
            }
            None => {
                assert!(
                    wait_for(|| !handle.unpowered_banks_driven().is_empty(), SETTLE),
                    "the bank is reported"
                );
                assert_eq!(state(&system, &net), NetState::Floating, "{net}");
                assert_eq!(handle.bank_volts(bank_of(pin)), None);
                assert_eq!(handle.unpowered_banks_driven(), vec![bank_of(pin)]);
                assert_eq!(bank_pin_name(bank_of(pin)), "VIO_8_11");
            }
        }
        assert_eq!(
            handle.start_state(),
            StartState::Started {
                at_ns: P2_RESTART_DELAY_NS
            }
        );
        let _ = P2_FAST_OHMS;
        drop(system);
    }
}

/// The fast drive mode's strength is fitted to the datasheet's output
/// table, and the fit is what a fast pad presents.
#[rstest]
fn the_fast_pad_strength_is_fitted_to_the_datasheet_output_table() {
    behaviour!(Test {
        id: "p2-package.fast-pad-strength",
        covers: Some("boards/src/p2.rs#P2_FAST_OHMS"),
        given: "the P2 datasheet's typical output voltages for the fast driver at 1, 10 and \
                30 milliamps, sourcing below the bank supply and sinking above ground",
    });
    expect!(
        "least-squares-fit",
        "a fast pad's source impedance is the single resistance that best fits all six \
         figures, 17.99 ohms",
        "a pad drives to its rails, so its impedance is the one figure left to fit, and the \
         fit weights the larger currents where the figure decides anything"
    );
    expect!(
        "inside-the-rated-rows",
        "the fitted impedance lies between the smallest and largest voltage-per-current \
         ratio of the 10 and 30 milliamp rows",
        "those rows span 16 to 19.3 ohms"
    );
    expect!(
        "a-fast-pad-presents-it",
        "a pad driven high in fast mode in a 3.3 volt bank presents 3.3 volts through that \
         impedance"
    );

    let points: Vec<(f64, f64)> = P2_FAST_SOURCE_POINTS
        .iter()
        .chain(&P2_FAST_SINK_POINTS)
        .copied()
        .collect();
    let volt_amps: f64 = points.iter().map(|(i, v)| v * i).sum();
    let amps_squared: f64 = points.iter().map(|(i, _)| i * i).sum();
    assert!((P2_FAST_OHMS - volt_amps / amps_squared).abs() < 1e-12);
    assert!(
        (P2_FAST_OHMS - 17.99).abs() < 0.005,
        "the fit is 17.99 ohms; got {P2_FAST_OHMS}"
    );

    let ratios: Vec<f64> = P2_FAST_SOURCE_POINTS[1..]
        .iter()
        .chain(&P2_FAST_SINK_POINTS[1..])
        .map(|(i, v)| v / i)
        .collect();
    let smallest = ratios.iter().copied().fold(f64::MAX, f64::min);
    let largest = ratios.iter().copied().fold(f64::MIN, f64::max);
    assert!((smallest - 16.0).abs() < 1e-9 && (largest - 0.580 / 0.030).abs() < 1e-9);
    assert!(smallest <= P2_FAST_OHMS && P2_FAST_OHMS <= largest);

    let banks = embsim_boards::p2::BankSupplies::held_at(3.3);
    assert_eq!(
        banks.pad_drive(0, P_HIGH_FAST | P_LOW_FAST, true, true),
        PadDrive::Thevenin(TheveninDrive {
            volts: 3.3,
            impedance: P2_FAST_OHMS,
        })
    );
}

/// An analog probe that keeps its pin, so a bench tick can read the
/// engine's report of the net at an instant ([`ReadAt`]).
struct HeldProbe {
    pins: [PinDecl; 1],
    pin: Arc<StdMutex<Option<PinHandle>>>,
}

impl HeldProbe {
    fn new() -> Self {
        Self {
            pins: [PinDecl::analog("A")],
            pin: Arc::default(),
        }
    }
}

impl Component for HeldProbe {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        *self.pin.lock().unwrap() = Some(io.pin("A")?);
        Ok(())
    }
}

/// The native firmware image inside the package drives its pads through
/// the package's bank supplies, like any core: nothing before the START
/// instant, then high at its bank's `VIO_a_b` — and nothing in a bank
/// whose supply names no voltage.
#[rstest]
#[case::a_1v8_bank(Some(1.8))]
#[case::a_3v3_bank(Some(3.3))]
#[case::an_unsupplied_bank(None)]
fn the_native_cores_pads_drive_at_their_banks_supply_from_the_start(#[case] vio: Option<f64>) {
    use embsim_board::mcu::{GpioChannelConfig, GpioDirection};
    use embsim_board::McuComponent;

    behaviour!(Test {
        id: "p2-package.native-core-pad-at-bank-supply",
        covers: Some("boards/src/p2.rs#P2Core for McuComponent"),
        given: "the native firmware in a P2 package setting a bridged output high once it \
                runs, supplies up from the build, that pin's bank at 1.8 volts, 3.3 volts or \
                unsupplied",
    });
    expect!(
        "floats-before-start",
        "a millisecond in — before the datasheet's restart delay is out — the pin's net \
         floats",
        "a chip in reset floats every pad, whatever core runs inside it"
    );
    expect!(
        "high-is-the-bank-voltage",
        "once the firmware runs, a pin in a supplied bank sits at exactly that bank's voltage",
        "the native core's pads drive through the package's bank supplies, as the QEMU core's \
         do"
    );
    expect!(
        "unsupplied-bank-drives-nothing",
        "a pin in the unsupplied bank floats and the package names that bank as driven \
         without a supply"
    );

    let _lock = suite_lock();
    stepped();
    const PAD: u8 = 4;
    let mcu = McuComponent::builder("native")
        .gpio_table(vec![GpioChannelConfig {
            pin: u32::from(PAD),
            active_low: false,
        }])
        .bridge_gpio(0, GpioDirection::Output)
        .entry(|| embsim_peripherals::gpio::set_active(0, true))
        .build()
        .expect("the channel is in the table and inside P63");
    // The firmware's bank sizing, done before the bridge is wired (sizing
    // a GPIO bank clears its callbacks).
    mcu.instance()
        .expect("an entry gives the MCU its own instance")
        .gpio
        .init(1, None);
    let package = P2Package::native(mcu);
    let handle = package.handle();
    let probe = HeldProbe::new();
    let probe_pin = Arc::clone(&probe.pin);
    let early = ReadAt::new(1_000_000, move || {
        probe_pin
            .lock()
            .unwrap()
            .as_ref()
            .map(PinHandle::net_report)
    });
    let early_read = Arc::clone(&early.got);
    let mut harness = grounded(Harness::new())
        .power(ep("BENCH.RESN"), ep("P2.RESN"), 3.3)
        .power(ep("BENCH.VDD"), ep("P2.VDD"), 1.8)
        .connect_str("PROBE.A", &format!("P2.{}", pin_name(PAD)))
        .expect("endpoints parse");
    if let Some(volts) = vio {
        harness = harness.power(
            ep("BENCH.VIO"),
            ep(&format!("P2.{}", bank_pin_name(bank_of(PAD)))),
            volts,
        );
    }
    let system = System::new()
        .component("P2", Box::new(package))
        .component("PROBE", Box::new(probe))
        .component("EARLY", Box::new(early))
        .harness(harness)
        .start()
        .expect("the bench starts");
    let net = format!("P2.{}", pin_name(PAD));
    assert!(
        wait_for(|| handle.started_at_ns().is_some(), SETTLE),
        "{:?}",
        handle.start_state()
    );
    assert_eq!(handle.started_at_ns(), Some(P2_RESTART_DELAY_NS));
    assert_eq!(
        *early_read.lock().unwrap(),
        Some((1_000_000, Some(NetState::Floating))),
        "{net} floats before the START instant"
    );
    match vio {
        Some(volts) => {
            assert!(
                wait_for(
                    || matches!(state(&system, &net), NetState::Analog(v) if (v - volts).abs() < 1e-9),
                    SETTLE
                ),
                "{net} sits at the bank's {volts} V; got {:?}",
                state(&system, &net)
            );
            assert_eq!(handle.unpowered_banks_driven(), Vec::<usize>::new());
        }
        None => {
            assert!(
                wait_for(|| !handle.unpowered_banks_driven().is_empty(), SETTLE),
                "the bank is reported"
            );
            assert_eq!(state(&system, &net), NetState::Floating, "{net}");
            assert_eq!(handle.unpowered_banks_driven(), vec![bank_of(PAD)]);
        }
    }
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
        assert!(pad.reads_when_subscribed(), "{}", pad.number);
        assert_eq!(pad.idle, None, "{}", pad.number);
    }
}
