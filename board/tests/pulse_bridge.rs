//! Peripheral **pin** bridging end to end (`BOARD_ENGINE.md`, "The MCU as a
//! component"): an [`McuComponent`] whose pulse-out, GPIO and encoder channels
//! reach real pins, harness-wired to peer components that observe those pins.
//!
//! The test stands in for both sides of the deployment, exactly as
//! `mcu_component.rs` does for the serial slice:
//!
//! - it plays the **runtime** by sizing the default peripheral banks
//!   (`pulse_out::init` / `gpio::init` / `encoder::init`) before
//!   `System::start`, as `Emulator::run` does before project wiring;
//! - it plays the **firmware** by calling the peripheral free functions the
//!   HAL trampolines call (`pulse_out::start_velocity`, `gpio::set_active`,
//!   `encoder::value`, …), so the whole path peripheral bank ⇄ bridge ⇄ pins
//!   ⇄ nets ⇄ peer is exercised with no firmware linked.
//!
//! The four claims under test, one per section below:
//!
//! 1. a step train crosses to the drive **as a rate** — a periodic drive on
//!    the STEP net — and the pulse count the peer reconstructs is exactly the
//!    firmware's;
//! 2. a mid-train direction reversal is the DIR net changing and nothing
//!    else: the STEP segment is not re-published, and the drive's own
//!    counter, folding it at the instants its DIR receiver changed, splits
//!    the count exactly — no pulse re-signed, none lost, none of phase;
//! 3. GPIO bridges **both** ways at the channel's own `active_low` polarity —
//!    firmware writes drive the net, and an external drive senses back (the
//!    endstop path);
//! 4. a quadrature pin pair produces the counts firmware reads, incrementing
//!    the firmware's own counter rather than overwriting it.
//!
//! Plus the load-bearing budget: [`a_realistic_step_rate_costs_a_bounded_number_of_engine_events`]
//! pins an explicit engine-event ceiling that per-edge stepping could not meet.
//!
//! This binary owns the process-default peripheral instance and the
//! process-global virtual clock, so its cases take one suite lock
//! (`TESTING.md` rule 5) and reset the banks between them.

use rstest::rstest;
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};

use embsim_board::mcu::{
    EncoderChannelConfig, GpioChannelConfig, GpioDirection, PulseOutChannelConfig,
};
use embsim_board::{
    jesd8c01_lvcmos_thresholds, AttachError, Component, ComponentNetIo, DeadBand, DigitalReceiver,
    Drive, EngineEvent, Harness, Level, McuComponent, NetState, PeriodicSchedule, PinDecl,
    PinHandle, System, SystemHandle,
};
use embsim_core::virtual_clock;
use embsim_peripherals::{encoder, gpio, pulse_out};

// ============================================================
// Channel map — the "HAL tables" this fake firmware was built with
// ============================================================

/// GPIO channel 0: the drive's enable, open-collector like the reference
/// machine's `SC_ENA` (active drives the pin **low**).
const ENA_CHANNEL: usize = 0;
/// GPIO channel 1: the step train's direction, active-high.
const DIR_CHANNEL: usize = 1;
/// GPIO channel 2: an endstop the firmware *reads*, active-low (a closed
/// switch pulls the line down).
const ESTOP_CHANNEL: usize = 2;

const GPIO_TABLE: [GpioChannelConfig; 3] = [
    GpioChannelConfig {
        pin: 6,
        active_low: true,
    },
    GpioChannelConfig {
        pin: 7,
        active_low: false,
    },
    GpioChannelConfig {
        pin: 16,
        active_low: true,
    },
];

/// Pulse-out channel 0: the step clock on P8.
const PULSE_TABLE: [PulseOutChannelConfig; 1] = [PulseOutChannelConfig { pin: 8 }];

/// Encoder channel 0: the quadrature pair on P20/P21.
const ENCODER_TABLE: [EncoderChannelConfig; 1] = [EncoderChannelConfig {
    pin_a: 20,
    pin_b: 21,
}];

/// The reference machine's resolution: 4 microsteps × 2048 steps/rev.
const STEPS_PER_MM: u64 = 8_192;

// ============================================================
// Suite plumbing
// ============================================================

/// The default peripheral banks and the virtual clock are process-global, so
/// no two cases here may overlap.
static SUITE_LOCK: Mutex<()> = Mutex::new(());

fn suite_lock() -> MutexGuard<'static, ()> {
    SUITE_LOCK.lock().unwrap_or_else(|poisoned| {
        SUITE_LOCK.clear_poison();
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
// Peer: a step/direction drive that observes the pins
// ============================================================

/// Every segment the STEP net carried, in the order the drive was handed
/// them.
type TrainLog = Arc<Mutex<Vec<PeriodicSchedule>>>;
/// Every state the drive's DIR and ENA inputs were handed, with the virtual
/// instant (µs) of the handing.
type SenseLog = Arc<Mutex<Vec<(&'static str, NetState, u64)>>>;

/// What the drive's own step counter holds: every pulse it was handed,
/// signed by the direction it read on `DIR` when the pulse went out, and
/// unsigned.
#[derive(Debug, Default)]
struct StepCount {
    /// The segment `STEP` carries now, and how much of it is counted.
    segment: Option<PeriodicSchedule>,
    counted: u64,
    /// `DIR` read high: reverse on this machine.
    reverse: bool,
    signed: i64,
    unsigned: u64,
}

impl StepCount {
    /// Count the current segment's pulses up to `at_ns` under the direction
    /// in force.
    fn fold(&mut self, at_ns: u64) {
        let Some(segment) = self.segment else {
            return;
        };
        let now = segment.emitted_at_ns(at_ns);
        let pulses = now.saturating_sub(self.counted);
        self.counted = now.max(self.counted);
        self.unsigned += pulses;
        let pulses = i64::try_from(pulses).expect("a test's pulses fit");
        self.signed += if self.reverse { -pulses } else { pulses };
    }
}

/// The drive at the far end of the harness: `STEP`, `DIR` and `ENA` are plain
/// sensed inputs, and a periodic drive on the STEP net hands `STEP` its
/// segment. It records what arrived, so the assertions are about the
/// *wire*, and it keeps a step counter the way a drive does — folding each
/// segment it was handed at the instants its own `DIR` receiver changed
/// (`NODES.md` §11, contract line 6: a node that consumes a clock
/// integrates the segment itself).
struct StepDrive {
    pins: [PinDecl; 3],
    trains: TrainLog,
    senses: SenseLog,
    count: Arc<Mutex<StepCount>>,
}

impl StepDrive {
    fn new(trains: TrainLog, senses: SenseLog, count: Arc<Mutex<StepCount>>) -> Self {
        Self {
            pins: [input("STEP"), input("DIR"), input("ENA")],
            trains,
            senses,
            count,
        }
    }
}

impl Component for StepDrive {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        // The counter: DIR first, so a segment is folded against the
        // direction already read.
        let count = Arc::clone(&self.count);
        let dir = DigitalReceiver::new(io.pin("DIR")?);
        io.on_sense("DIR", move |sense| {
            let reverse = dir.read(&sense) == Some(Level::High);
            let mut count = count.lock().unwrap();
            count.fold(sense.at_ns);
            count.reverse = reverse;
        })?;
        let count = Arc::clone(&self.count);
        io.on_sense("STEP", move |sense| {
            let mut count = count.lock().unwrap();
            let Some(clock) = sense.periodic else {
                count.fold(sense.at_ns);
                count.segment = None;
                return;
            };
            // The outgoing segment up to where the new one picks up.
            count.fold(clock.segment.since_ns);
            count.segment = Some(clock.segment);
            count.counted = clock.segment.emitted;
        })?;
        let trains = Arc::clone(&self.trains);
        io.on_net_report("STEP", move |state| {
            if let NetState::Periodic { segment, .. } = state {
                trains.lock().unwrap().push(segment);
            }
        })?;
        for pin in ["DIR", "ENA"] {
            let senses = Arc::clone(&self.senses);
            io.on_net_report(pin, move |state| {
                senses
                    .lock()
                    .unwrap()
                    .push((pin, state, virtual_clock::virtual_ns()))
            })?;
        }
        Ok(())
    }
}

// ============================================================
// Peer: a switch and an encoder the test drives onto MCU inputs
// ============================================================

/// A component that is nothing but drivable output pins, shared out at attach
/// — the endstop contact and the encoder's phase pair.
struct Driver {
    pins: Vec<PinDecl>,
    handles: Arc<Mutex<Vec<(&'static str, PinHandle)>>>,
}

impl Driver {
    fn new(numbers: &[&'static str], handles: Arc<Mutex<Vec<(&'static str, PinHandle)>>>) -> Self {
        Self {
            pins: numbers.iter().map(|n| output(n)).collect(),
            handles,
        }
    }
}

impl Component for Driver {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let mut handles = self.handles.lock().unwrap();
        for pin in &self.pins {
            handles.push((pin.number, io.pin(pin.number)?));
        }
        Ok(())
    }
}

const fn input(number: &'static str) -> PinDecl {
    PinDecl::digital_in(number, jesd8c01_lvcmos_thresholds(DeadBand::Unknown))
}

const fn output(number: &'static str) -> PinDecl {
    PinDecl::digital_out(number)
}

const HIGH: embsim_board::TheveninDrive = embsim_board::TheveninDrive {
    volts: 3.3,
    impedance: 25.0,
};
const LOW: embsim_board::TheveninDrive = embsim_board::TheveninDrive {
    volts: 0.0,
    impedance: 25.0,
};

// ============================================================
// The rig
// ============================================================

/// Everything a case needs to play firmware on one side and read the pins on
/// the other.
struct Rig {
    system: SystemHandle,
    trains: TrainLog,
    senses: SenseLog,
    /// The drive's own step counter.
    count: Arc<Mutex<StepCount>>,
    /// `SW.OUT`, `ENC.A`, `ENC.B` — the pins the *test* drives into the MCU.
    external: Arc<Mutex<Vec<(&'static str, PinHandle)>>>,
}

impl Rig {
    /// The pin handle for one externally-driven net.
    fn external(&self, number: &str) -> PinHandle {
        self.external
            .lock()
            .unwrap()
            .iter()
            .find(|(name, _)| *name == number)
            .map(|(_, handle)| handle.clone())
            .unwrap_or_else(|| panic!("{number} attached"))
    }

    /// Every segment the drive has seen so far.
    fn trains(&self) -> Vec<PeriodicSchedule> {
        self.trains.lock().unwrap().clone()
    }

    /// Block until the drive has seen `n` segments.
    fn await_trains(&self, n: usize) -> Vec<PeriodicSchedule> {
        assert!(
            wait_for(
                || self.trains.lock().unwrap().len() >= n,
                Duration::from_secs(5)
            ),
            "drive saw {} of {n} expected trains",
            self.trains.lock().unwrap().len()
        );
        self.trains()
    }

    /// The most recent state the drive sensed on one of its logic inputs.
    fn last_sense(&self, pin: &str) -> Option<NetState> {
        self.senses
            .lock()
            .unwrap()
            .iter()
            .rev()
            .find(|(name, _, _)| *name == pin)
            .map(|(_, state, _)| *state)
    }

    /// Block until the drive's `pin` input has been handed `state`, and
    /// return the virtual instant it was.
    fn await_sense(&self, pin: &str, state: NetState) -> u64 {
        assert!(
            wait_for(
                || self.last_sense(pin) == Some(state),
                Duration::from_secs(5)
            ),
            "drive's {pin} saw {:?}, expected {state:?}",
            self.last_sense(pin)
        );
        self.senses
            .lock()
            .unwrap()
            .iter()
            .rev()
            .find(|(name, _, _)| *name == pin)
            .map(|(_, _, at)| *at)
            .expect("just seen")
    }
}

/// Build the rig: an MCU with all three motion channel kinds bridged, a drive
/// watching `STEP`/`DIR`/`ENA`, a switch on the endstop input, and an encoder
/// on the quadrature pair.
///
/// `event_log` turns on Oracle 1 for the event-budget case.
fn rig(event_log: bool) -> Rig {
    // The runtime's role: size the default banks before wiring.
    gpio::init(GPIO_TABLE.len(), None);
    pulse_out::init(PULSE_TABLE.len());
    encoder::init(ENCODER_TABLE.len());

    let trains: TrainLog = Arc::new(Mutex::new(Vec::new()));
    let senses: SenseLog = Arc::new(Mutex::new(Vec::new()));
    let count = Arc::new(Mutex::new(StepCount::default()));
    let external = Arc::new(Mutex::new(Vec::new()));

    let mcu = McuComponent::builder("p2")
        .gpio_table(GPIO_TABLE.to_vec())
        .bridge_gpio(ENA_CHANNEL, GpioDirection::Output)
        .bridge_gpio(DIR_CHANNEL, GpioDirection::Output)
        .bridge_gpio(ESTOP_CHANNEL, GpioDirection::Input)
        .pulse_out_table(PULSE_TABLE.to_vec())
        .bridge_pulse_out(0)
        .encoder_table(ENCODER_TABLE.to_vec())
        .bridge_encoder(0)
        .build()
        .expect("MCU builds from the channel tables");

    let harness = Harness::new()
        .connect_str("MCU.P8", "DRIVE.STEP")
        .expect("endpoints parse")
        .connect_str("MCU.P7", "DRIVE.DIR")
        .expect("endpoints parse")
        .connect_str("MCU.P6", "DRIVE.ENA")
        .expect("endpoints parse")
        .connect_str("MCU.P16", "SW.OUT")
        .expect("endpoints parse")
        .connect_str("MCU.P20", "ENC.A")
        .expect("endpoints parse")
        .connect_str("MCU.P21", "ENC.B")
        .expect("endpoints parse");

    let mut builder = System::new()
        .component("MCU", Box::new(mcu))
        .component(
            "DRIVE",
            Box::new(StepDrive::new(
                Arc::clone(&trains),
                Arc::clone(&senses),
                Arc::clone(&count),
            )),
        )
        .component("SW", Box::new(Driver::new(&["OUT"], Arc::clone(&external))))
        .component(
            "ENC",
            Box::new(Driver::new(&["A", "B"], Arc::clone(&external))),
        )
        .harness(harness);
    if event_log {
        builder = builder.event_log();
    }
    let system = builder.start().expect("live system starts");

    Rig {
        system,
        trains,
        senses,
        count,
        external,
    }
}

/// Tear the rig down and leave the process banks clean for the next case.
fn teardown(rig: Rig) {
    // The engine joins before the components drop (`SystemHandle`'s documented
    // order), so no callback can race the bank reset below.
    drop(rig);
    pulse_out::reset();
    gpio::reset();
    encoder::reset();
}

/// `(A, B)` phases in the order that counts up.
const PHASES: [(bool, bool); 4] = [(false, false), (true, false), (true, true), (false, true)];

/// Drive the encoder pair to phase `index` in one move (which may be a
/// two-state jump the decoder refuses to count — callers re-zero afterwards).
fn set_phase(rig: &Rig, index: usize) {
    let (want_a, want_b) = PHASES[index];
    rig.external("A")
        .set_drive(Some(if want_a { HIGH } else { LOW }));
    rig.external("B")
        .set_drive(Some(if want_b { HIGH } else { LOW }));
}

/// Walk the encoder phase pair through `steps` Gray transitions, forward when
/// `forward`, starting from phase index `from`. Returns the phase index left
/// on the pins.
fn walk_encoder(rig: &Rig, from: usize, steps: usize, forward: bool) -> usize {
    let mut index = from;
    for _ in 0..steps {
        index = if forward {
            (index + 1) % 4
        } else {
            (index + 3) % 4
        };
        // One channel changes per transition, which is the Gray property the
        // decoder relies on; drive both and let the unchanged one be a no-op.
        set_phase(rig, index);
    }
    index
}

/// Block until the encoder pair actually presents `phase` on the nets **and**
/// the decoded count has then held still for a stretch.
///
/// Both halves matter. Waiting only for a stable count would pass immediately
/// while the phase drive is still in flight, and the transitions would then
/// land *after* the counter was re-based — the count-is-quiet-so-it-must-be-
/// done trap. Waiting only for the net state would not prove the bridge's
/// sense callback had run. It is a contract rather than a sleep, and it fails
/// loudly instead of silently proceeding.
fn settle_encoder(rig: &Rig, phase: usize) {
    let (want_a, want_b) = PHASES[phase];
    let level = |high: bool| NetState::Driven(if high { Level::High } else { Level::Low });
    assert!(
        wait_for(
            || rig.system.net_state("ENC.A") == Some(level(want_a))
                && rig.system.net_state("ENC.B") == Some(level(want_b)),
            Duration::from_secs(5)
        ),
        "the encoder pair never reached phase {phase}; A = {:?} B = {:?}",
        rig.system.net_state("ENC.A"),
        rig.system.net_state("ENC.B")
    );
    const STABLE_POLLS: usize = 5;
    let deadline = Instant::now() + Duration::from_secs(5);
    let (mut last, mut stable) = (encoder::value(0), 0);
    while Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(2));
        let now = encoder::value(0);
        if now == last {
            stable += 1;
            if stable >= STABLE_POLLS {
                return;
            }
        } else {
            (last, stable) = (now, 0);
        }
    }
    panic!("encoder count never settled (last {last})");
}

// ============================================================
// 1. The step train crosses as a rate, and the count is exact
// ============================================================

/// A finite train started through `HAL_pulseOut_start` reaches the drive as
/// **one** segment on the STEP net carrying frequency, ceiling and baseline —
/// and the count the drive reconstructs from it at any instant is
/// bit-identical to the peripheral's own integration, which is what
/// `HAL_pulseOut_run` hands the firmware.
#[rstest]
fn a_step_train_reaches_the_drive_as_a_rate_with_an_exact_count() {
    let _suite = suite_lock();
    virtual_clock::init(1.0, 1_000_000);
    let rig = rig(false);

    // Attach establishes the channel on the net before any firmware runs.
    let attached = rig.await_trains(1);
    assert_eq!(
        attached[0],
        PeriodicSchedule::IDLE,
        "an unstarted channel presents a held segment, not nothing"
    );

    // Firmware: enable the drive, then 4000 steps at 20 kHz.
    gpio::set_active(ENA_CHANNEL, true);
    pulse_out::start(0, 4_000, 20_000);

    let trains = rig.await_trains(2);
    assert_eq!(
        trains.len(),
        2,
        "one event for the start, not one per pulse"
    );
    let train = trains[1];
    assert_eq!(train.freq_hz, 20_000);
    assert_eq!(train.total, Some(4_000));
    assert_eq!(train.emitted, 0, "a fresh train starts from zero");
    assert_eq!(
        rig.system.net_state("MCU.P8"),
        Some(NetState::Periodic {
            hi: Level::High,
            lo: Level::Low,
            segment: train,
        }),
        "the STEP net is the square wave, between the pad's own levels"
    );

    // Read-time integration: exact at every probe, clamped at the ceiling.
    let t0 = train.since_ns;
    assert_eq!(train.emitted_at_ns(t0), 0);
    assert_eq!(
        train.emitted_at_ns(t0 + 100_000_000),
        2_000,
        "half of 4000 at 20 kHz"
    );
    assert_eq!(train.emitted_at_ns(t0 + 200_000_000), 4_000);
    assert_eq!(
        train.emitted_at_ns(t0 + 10_000_000_000),
        4_000,
        "the ceiling holds forever after"
    );
    assert_eq!(
        train.completes_at(),
        Some(t0 + 200_000_000),
        "the drive knows when the last pulse goes out without being told"
    );

    // …and it agrees with the peripheral's own view, the one the firmware
    // reads through `HAL_pulseOut_run`. Both are evaluated at **one** sampled
    // instant: reading each at its own `now` would compare two different
    // moments of a live train and differ by a pulse whenever they straddle a
    // period boundary.
    let now = virtual_clock::virtual_ns();
    assert_eq!(
        train.emitted_at_ns(now),
        pulse_out::segment(0).emitted_at_ns(now),
        "the wire and the firmware must never disagree about the count"
    );

    // Stopping freezes the train at its exact final count.
    pulse_out::stop(0);
    let stopped = rig.await_trains(3)[2];
    assert_eq!(stopped.freq_hz, 0);
    assert_eq!(stopped.total, Some(stopped.emitted));
    assert_eq!(
        stopped.emitted_at_ns(u64::MAX),
        stopped.emitted,
        "a stopped train emits nothing more, however long you wait"
    );

    teardown(rig);
}

// ============================================================
// 2. Direction reversal, including mid-train
// ============================================================

/// Flipping the direction GPIO **mid-train** changes the DIR net and nothing
/// else: the STEP segment is not re-published, because the wire carries no
/// direction. The drive's own counter, folding the one segment at the
/// instants its DIR receiver changed, splits the count exactly — the pulses
/// on each side keep their own sign, and the three legs add up to the
/// firmware's own count with not a pulse of phase lost. This is the case an
/// edge-free representation has to get right or it is not usable.
#[rstest]
fn a_mid_train_reversal_splits_the_count_at_the_instant_dir_changed() {
    let _suite = suite_lock();
    virtual_clock::init(0.0, 1_000_000);
    let rig = rig(false);
    rig.await_trains(1);
    let released = rig.await_sense("DIR", NetState::Driven(Level::Low));

    gpio::set_active(ENA_CHANNEL, true);
    pulse_out::start_velocity(0, STEPS_PER_MM as u32); // 1 mm/s
    let train = rig.await_trains(2)[1];
    assert_eq!(train.total, None, "a velocity train is unbounded");

    // Jump virtual time so pulses accumulate, then reverse.
    virtual_clock::wait_virtual_us(5_000);
    gpio::set_active(DIR_CHANNEL, true);
    let reversed = rig.await_sense("DIR", NetState::Driven(Level::High));

    // Reverse again: back to forward.
    virtual_clock::wait_virtual_us(5_000);
    gpio::set_active(DIR_CHANNEL, false);
    let restored = rig.await_sense("DIR", NetState::Driven(Level::Low));
    assert!(
        released <= train.since_ns && train.since_ns < reversed && reversed < restored,
        "DIR low at {released}, clock from {}, reversed at {reversed}, restored at {restored}",
        train.since_ns
    );

    pulse_out::stop(0);
    let trains = rig.await_trains(3);
    assert_eq!(
        trains.len(),
        3,
        "idle, start and stop — a direction change publishes no segment: {trains:?}"
    );
    let stopped = trains[2];

    // Fold the one running segment at the instants DIR changed: every pulse
    // is counted once, from the anchor the peripheral published.
    let at = |t: u64| train.emitted_at_ns(t) - train.emitted;
    let forward = at(reversed);
    let reverse = at(restored) - at(reversed);
    let forward_again = at(stopped.since_ns) - at(restored);
    assert!(
        forward > 0 && reverse > 0,
        "the reversal must land mid-train (forward {forward}, reverse {reverse})"
    );
    assert_eq!(
        forward + reverse + forward_again,
        pulse_out::emitted(0),
        "the three legs are exactly the firmware's own count — no pulse of phase lost"
    );
    assert_eq!(
        stopped.emitted,
        train.emitted_at_ns(stopped.since_ns),
        "the stop picks up exactly where the running segment was"
    );
    let signed = forward as i64 - reverse as i64 + forward_again as i64;
    assert!(
        signed.unsigned_abs() < forward + reverse + forward_again,
        "a there-and-back move nets out below its travelled distance"
    );

    // The drive's own counter — the segments it was handed, folded at the
    // instants its DIR receiver changed — holds the same split: the net
    // travel, and every pulse the firmware emitted, once.
    let count = rig.count.lock().unwrap();
    assert_eq!(
        count.signed, signed,
        "the drive counts forward, back and forward again at the instants it read DIR change"
    );
    assert_eq!(
        count.unsigned,
        pulse_out::emitted(0),
        "the drive counted exactly the firmware's pulses — none lost, none twice"
    );
    drop(count);

    teardown(rig);
}

// ============================================================
// 3. GPIO, both directions, at the channel's own polarity
// ============================================================

/// A firmware GPIO write drives the pin at the channel's declared polarity —
/// `ENA` here is open-collector like the reference machine's, so *active* is a
/// **low** pin.
#[rstest]
#[case::inactive_is_high(false, Level::High)]
#[case::active_is_low(true, Level::Low)]
fn a_firmware_gpio_write_drives_the_net_at_the_channel_polarity(
    #[case] active: bool,
    #[case] expect: Level,
) {
    let _suite = suite_lock();
    virtual_clock::init(1.0, 1_000_000);
    let rig = rig(false);

    gpio::set_active(ENA_CHANNEL, active);
    assert!(
        wait_for(
            || rig.last_sense("ENA") == Some(NetState::Driven(expect)),
            Duration::from_secs(5)
        ),
        "peer saw {:?} on ENA, expected {expect:?}",
        rig.last_sense("ENA")
    );

    teardown(rig);
}

/// The other direction, which is what an endstop needs: an external component
/// drives the pin and the firmware reads it back through
/// `HAL_GPIO_getActive`, at the channel's polarity — the `ESTOP` channel is
/// active-low, so a closed switch pulling the line down reads *active*.
#[rstest]
fn an_external_drive_senses_back_into_the_firmware_gpio_bank() {
    let _suite = suite_lock();
    virtual_clock::init(1.0, 1_000_000);
    let rig = rig(false);
    let switch = rig.external("OUT");

    switch.set_drive(Some(HIGH));
    assert!(
        wait_for(|| !gpio::get_active(ESTOP_CHANNEL), Duration::from_secs(5)),
        "an open (high) endstop must read inactive"
    );

    switch.set_drive(Some(LOW));
    assert!(
        wait_for(|| gpio::get_active(ESTOP_CHANNEL), Duration::from_secs(5)),
        "a closed (low) endstop must read ACTIVE on an active-low channel"
    );

    // Releasing the net entirely leaves it floating: the engine refuses to
    // project a level, so the bridge holds the last value rather than
    // inventing a released state.
    switch.set_drive(None);
    std::thread::sleep(Duration::from_millis(20));
    assert!(
        gpio::get_active(ESTOP_CHANNEL),
        "a floating input holds its last value; the bridge invents nothing"
    );

    teardown(rig);
}

// ============================================================
// 4. Encoder counts arrive from a real quadrature pair
// ============================================================

/// A quadrature pair walked by a peer component lands in the firmware's
/// encoder bank as ×4 counts, signed by the walk direction — and a firmware
/// write to the counter (homing) re-bases it instead of being overwritten.
#[rstest]
fn a_quadrature_pin_pair_produces_the_counts_firmware_reads() {
    let _suite = suite_lock();
    virtual_clock::init(1.0, 1_000_000);
    let rig = rig(false);

    // Park the pair at a known phase and let the decoder catch up, then zero
    // the counter: the walk below is asserted in absolute counts, so it must
    // start from a written-down state rather than the pins' idle default.
    let mut phase = 0;
    set_phase(&rig, phase);
    settle_encoder(&rig, phase);
    encoder::set(0, 0);

    // One full cycle forward is four counts.
    phase = walk_encoder(&rig, phase, 4, true);
    assert!(
        wait_for(|| encoder::value(0) == 4, Duration::from_secs(5)),
        "a forward Gray cycle is 4 counts; bank reads {}",
        encoder::value(0)
    );

    // …and reversing the walk unwinds them.
    phase = walk_encoder(&rig, phase, 6, false);
    assert!(
        wait_for(|| encoder::value(0) == -2, Duration::from_secs(5)),
        "6 reverse counts from +4 is -2; bank reads {}",
        encoder::value(0)
    );

    // The firmware owns the register: homing re-bases it, and the next
    // transition continues from there rather than snapping back.
    encoder::set(0, 100_000);
    walk_encoder(&rig, phase, 1, true);
    assert!(
        wait_for(|| encoder::value(0) == 100_001, Duration::from_secs(5)),
        "a homing write must survive the next edge; bank reads {}",
        encoder::value(0)
    );

    teardown(rig);
}

// ============================================================
// The budget: engine events must not scale with the step rate
// ============================================================

/// The ceiling this whole representation exists to meet.
///
/// One drive + resolution + sense per STEP edge is the alternative; at the
/// reference machine's 8192 steps/mm a single mm/s already costs 8192 of them
/// a second. The number below is the *whole* engine event count for a
/// four-segment motion profile — every drive, resolution, sense, reroute and
/// pulse update the rig produces, start-up included — and it is a constant:
/// it does not move when the profile runs longer or faster.
///
/// Measured on the reference host: **31 events for 65 536 pulses** while the
/// train rode a pulse channel beside the net (5 pulse updates; the phase-4
/// tree), **46 events** for the same 65 536 since it is a periodic drive
/// on the STEP net, and **52** since the drive keeps its own step counter
/// (`STEP` and `DIR` subscriptions beside the recorders: six more sense
/// deliveries, `NODES.md` §12 item 5, the review) —
/// each rate change a drive, a resolution and a sense like any other drive,
/// `sil-unified-drive.md`'s "a rate change gets more expensive", measured
/// here; the same count on every repeat while the pulse count moved by tens
/// of thousands — the number really is independent of the rate. The headroom
/// below is for incidental engine bookkeeping, not for per-pulse traffic: a
/// regression that reintroduced edges would miss this ceiling by three
/// orders of magnitude.
const ENGINE_EVENT_CEILING: usize = 64;

/// A realistic step rate costs a bounded number of engine events, and the
/// bound is independent of how many pulses actually went out.
#[rstest]
fn a_realistic_step_rate_costs_a_bounded_number_of_engine_events() {
    let _suite = suite_lock();
    virtual_clock::init(0.0, 1_000_000);
    let rig = rig(true);
    let log = rig.system.event_log();
    rig.await_trains(1);

    // A four-segment profile at 1 mm/s, 2 mm/s, 1 mm/s, stop.
    // Each segment lasts 1 virtual second so the pulse count is realistic
    // while the engine-event count stays a handful of rate changes.
    gpio::set_active(ENA_CHANNEL, true);
    pulse_out::start_velocity(0, STEPS_PER_MM as u32);
    virtual_clock::wait_virtual_us(1_000_000);
    pulse_out::set_frequency(0, 2 * STEPS_PER_MM as u32);
    virtual_clock::wait_virtual_us(1_000_000);
    pulse_out::set_frequency(0, STEPS_PER_MM as u32);
    virtual_clock::wait_virtual_us(1_000_000);
    pulse_out::stop(0);

    // IDLE + start + two retargets + stop.
    let trains = rig.await_trains(5);
    let now = virtual_clock::virtual_ns();
    let pulses: u64 = trains
        .iter()
        .map(|train| train.emitted_at_ns(now) - train.emitted)
        .sum();

    let records = log.records();
    let pulse_updates = records
        .iter()
        .filter(|r| {
            matches!(
                r.event,
                EngineEvent::DriveApplied {
                    drive: Some(Drive::Periodic { .. }),
                    ..
                }
            )
        })
        .count();

    assert!(
        pulses >= STEPS_PER_MM,
        "the profile must actually deliver a realistic number of steps; got {pulses}"
    );
    assert_eq!(
        pulse_updates, 5,
        "one periodic drive per rate change and no more"
    );
    assert!(
        records.len() <= ENGINE_EVENT_CEILING,
        "engine events must not scale with the step rate: {} events for {pulses} pulses \
         (ceiling {ENGINE_EVENT_CEILING})",
        records.len()
    );
    println!(
        "[budget] {pulses} pulses delivered in {} engine events ({pulse_updates} periodic \
         drives); one engine event per STEP edge would have been at least {pulses}",
        records.len()
    );

    teardown(rig);
}

// ============================================================
// Opt-in: an unbridged MCU is unchanged
// ============================================================

/// Every bridge is opt-in per channel: an MCU that bridges nothing declares no
/// pins, installs no callbacks, and leaves the peripheral banks exactly as it
/// found them — which is what keeps existing consumers unaffected until they
/// opt in.
#[rstest]
fn an_unbridged_mcu_touches_nothing() {
    let _suite = suite_lock();
    virtual_clock::init(1.0, 1_000_000);
    gpio::init(GPIO_TABLE.len(), None);
    pulse_out::init(PULSE_TABLE.len());

    let mcu = McuComponent::builder("p2")
        .gpio_table(GPIO_TABLE.to_vec())
        .pulse_out_table(PULSE_TABLE.to_vec())
        .encoder_table(ENCODER_TABLE.to_vec())
        .build()
        .expect("a table without a bridge is legal");
    assert!(mcu.pins().is_empty(), "declaring a table declares no pins");

    let system = System::new()
        .component("MCU", Box::new(mcu))
        .start()
        .expect("live system starts");

    // No pulse bridge means no rate-change subscriber: the peripheral's own
    // behavior is untouched.
    pulse_out::start(0, 10, 1_000);
    assert_eq!(pulse_out::segment(0).freq_hz, 1_000);
    gpio::set_active(ENA_CHANNEL, true);
    assert!(gpio::get_active(ENA_CHANNEL));

    system.shutdown();
    pulse_out::reset();
    gpio::reset();
}
