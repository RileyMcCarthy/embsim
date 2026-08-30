//! The carriage seam, closed: firmware pulse-out → STEP pin → a real
//! [`StepperMotor`] plant → its shaft → a real [`QuadratureEncoder`] → the A/B
//! pins → back into the firmware's encoder bank.
//!
//! This is the loop a consumer has had to hand-wire in Rust — subscribe to
//! `pulse_out::on_progress`, read `pulse_out::frequency`, integrate a plant by
//! hand, and write `encoder::set` — because `McuComponent` bridged serial only
//! and its motion channels had no pins. Every hop below is now a *system
//! description* instead: two channel tables, three harness lines, and the
//! components' own physics.
//!
//! What each assertion is really for:
//!
//! - **The commanded count crosses the wire exactly.** `commanded_steps()` on
//!   the far side of the harness must equal the pulse total the firmware asked
//!   for — not approximately, exactly. Encoder feedback is a closed loop, so
//!   an off-by-a-few step count is a silently wrong machine.
//! - **The encoder walks, it does not teleport.** `snapped_updates() == 0`
//!   proves the counts the firmware reads came from a real Gray-code walk on
//!   real pins, decoded by the bridge — not from a model writing the bank.
//! - **Direction agrees on both paths.** The drive latches its own `DIR` pin
//!   while the train carries a direction of its own; a machine whose two
//!   descriptions of "reverse" disagree is the classic inverted-axis bug.
//!
//! Its own binary: it owns the process-default peripheral banks and the
//! process-global virtual clock (`TESTING.md` rule 5).
//!
//! # Why the stimulus is engine-hosted, and the clock stepped
//!
//! This case used to play its own stimulus from the test thread —
//! `pulse_out::start`, `gpio::set_active` — and then poll the results under
//! wall-clock `wait_for` deadlines. That is not merely the "wall flakiness"
//! `TESTING.md` rule 4 warns about; on this seam it is **wrong answers**, and
//! it made the macOS CI leg red:
//!
//! `pulse_out::start` stamps the new segment's `since_us` with `virtual_us()`
//! *at the moment the caller runs*. From the test thread that instant is a
//! wall-determined sample, and the engine keeps advancing virtual time while
//! the command is in flight. [`StepperMotor`] deliberately refuses to rewind
//! its plant to meet a late-delivered train (`stepper_motor::set_train`:
//! "never rewind the plant to match"), so it anchors the segment at
//! `max(since_us, plant.now_us)`. When an observe wake lands in between, the
//! pulses in `[since_us, plant.now_us]` are still *counted* by
//! `commanded_steps()` — the pulse segment says they went out — but their
//! **travel is never integrated into the position**. Encoder feedback then
//! settles a few counts short, permanently, and the wall-clock guard spends
//! its whole timeout watching a value that will never arrive. The window is
//! exactly one observe interval, so at [`STEP_HZ`] the miss is a clean
//! `200 µs × 20 kHz = 4` counts — which is what CI reported (`bank reads 4`).
//!
//! Both halves of the fix matter, and neither alone is sufficient:
//!
//! - **The stimulus is engine-hosted.** A `Script` component re-arms itself
//!   with `schedule_at` and makes every `pulse_out::*` / `gpio::*` /
//!   `encoder::*` call from `on_wake`, so each one is stamped at an exact
//!   virtual instant the plant has not already passed. `DETERMINISM.md` T1 §4's
//!   "take your time from the engine", which `pulse_bridge_stepped.rs` states
//!   for the same bridges. A longer timeout could not have fixed this, and
//!   stepped mode alone would only have made the race *tighter*.
//! - **The observations are engine-hosted too.** Everything that is only true
//!   mid-sequence — the count after the forward leg, the latched direction —
//!   is captured on the engine thread at the instant it is meant, and the test
//!   thread asserts the captured snapshot afterwards. So no assertion depends
//!   on the test thread winning a race, and extra virtual time cannot spoil a
//!   fact already recorded.
//! - **The clock is stepped**, so the legs cost no wall time and each leg's
//!   completion is a virtual-time fact: [`LEG_US`] for the train plus
//!   [`SETTLE_US`] = 20 τ of first-order lag, after which the plant is at rest
//!   at its commanded position to ~1e-8 of a count.
//!
//! The one remaining wall-clock wait is the test thread parking until the
//! script reports done. It is a liveness guard, not a correctness deadline:
//! every asserted value was recorded at a fixed virtual instant before it.

use rstest::rstest;
use std::sync::atomic::{AtomicU64, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};

use embsim_board::mcu::{
    EncoderChannelConfig, GpioChannelConfig, GpioDirection, PulseOutChannelConfig,
};
use embsim_board::{
    AttachError, Component, ComponentNetIo, Harness, Level, McuComponent, NetState, PinDecl,
    PinKind, PulseDirection, System,
};
use embsim_core::virtual_clock::{self, ClockMode};
use embsim_models::machine::{
    quadrature_encoder, stepper_motor, EncoderInput, MotorShaft, QuadratureEncoder, StepperMotor,
};
use embsim_peripherals::{encoder, gpio, pulse_out};

/// GPIO channel 0: the drive's enable, open-collector — *active* pulls the pin
/// low, and the drive is enabled by a low `ENA`.
const ENA_CHANNEL: usize = 0;
/// GPIO channel 1: direction, active-high — and on this machine *active means
/// reverse*, which is stated twice below and asserted to agree.
const DIR_CHANNEL: usize = 1;

const GPIO_TABLE: [GpioChannelConfig; 2] = [
    GpioChannelConfig {
        pin: 6,
        active_low: true,
    },
    GpioChannelConfig {
        pin: 7,
        active_low: false,
    },
];
const PULSE_TABLE: [PulseOutChannelConfig; 1] = [PulseOutChannelConfig { pin: 8 }];
const ENCODER_TABLE: [EncoderChannelConfig; 1] = [EncoderChannelConfig {
    pin_a: 20,
    pin_b: 21,
}];

/// Steps in each leg of the move. Small enough that the encoder's Gray walk
/// (one engine transition per count — the encoder is *not* rate-carried) stays
/// cheap, large enough that a lost or doubled step is unmistakable.
const MOVE_STEPS: u32 = 100;
/// Step rate of each leg.
const STEP_HZ: u32 = 20_000;

/// The drive's first-order lag. Well under the step interval, so the plant
/// tracks the train rather than filtering it.
const TAU_US: u64 = 500;
/// Observation cadence — fine enough that the encoder walks every count.
const OBSERVE_INTERVAL_US: u64 = 200;

/// Virtual time one leg's pulse train occupies: `MOVE_STEPS / STEP_HZ`.
const LEG_US: u64 = MOVE_STEPS as u64 * 1_000_000 / STEP_HZ as u64;
/// Virtual time to let the plant come to rest after the last pulse. 20 τ of
/// first-order decay leaves `e^-20` ≈ 2e-9 of a count outstanding, so the
/// rounded encoder count is settled with room to spare.
const SETTLE_US: u64 = 20 * TAU_US;
/// Virtual-time spacing between script steps that only need to be ordered.
const STEP_PERIOD_US: u64 = 1_000;

// ============================================================
// Suite serialization
// ============================================================

/// Clock *mode* and the default peripheral banks are both process-global, so
/// the cases here must not overlap (`TESTING.md` rule 5).
static SUITE_LOCK: Mutex<()> = Mutex::new(());

fn suite_lock() -> MutexGuard<'static, ()> {
    SUITE_LOCK.lock().unwrap_or_else(|poisoned| {
        SUITE_LOCK.clear_poison();
        poisoned.into_inner()
    })
}

/// Puts the process clock in stepped mode for the guard's lifetime and
/// restores free-running on the way out, panic or not.
struct Stepped;

impl Stepped {
    fn enter() -> Self {
        virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);
        Self
    }
}

impl Drop for Stepped {
    fn drop(&mut self) {
        virtual_clock::init(1.0, 1_000_000);
    }
}

fn wait_for(mut pred: impl FnMut() -> bool, timeout: Duration) -> bool {
    let start = Instant::now();
    while start.elapsed() < timeout {
        if pred() {
            return true;
        }
        // A *test-side* wall sleep, deliberately not a `virtual_clock` wait:
        // this thread is not part of the simulation and must not park on a
        // clock the engine is stepping.
        std::thread::sleep(Duration::from_millis(1));
    }
    pred()
}

// ============================================================
// What the engine thread records
// ============================================================

/// Every fact the assertions need, each recorded on the engine thread at the
/// exact virtual instant it is meant. `None` is a failure: it means the script
/// never reached that step, so a missing capture can never pass vacuously.
#[derive(Debug, Default)]
struct Capture {
    /// Virtual instant of each capturing step, for the `--nocapture` report.
    at_us: Vec<(&'static str, u64)>,
    /// The encoder's own count-0 phase, sensed on the nets at attach.
    enc_a: Option<NetState>,
    enc_b: Option<NetState>,
    /// The bank's attach-time decode, read twice with no motion in between:
    /// the virtual-time form of "the count settled before the datum is set".
    bank_first: Option<i32>,
    bank_second: Option<i32>,
    /// The drive's enable, read back after `ENA` was pulled active (low).
    enabled: Option<bool>,
    // --- forward leg, after the train has run and the plant has settled ---
    forward_commanded: Option<i64>,
    forward_emitted: Option<u64>,
    forward_bank: Option<i32>,
    forward_snapped: Option<u64>,
    // --- the reversal, latched but not yet driven ---
    reverse_latched: Option<bool>,
    reverse_train_direction: Option<Option<PulseDirection>>,
    // --- reverse leg, after the train has run and the plant has settled ---
    final_commanded: Option<i64>,
    final_bank: Option<i32>,
    final_snapped: Option<u64>,
}

// ============================================================
// The scripted firmware (engine-hosted)
// ============================================================

/// Number of script steps; the run is complete when this many have fired.
const SCRIPT_STEPS: usize = 9;

/// Virtual-time gap before each script step, measured from the step before it
/// (the first from the anchor). The two long gaps are the legs: a full pulse
/// train plus 20 τ of settling, which is what makes each leg's completion a
/// virtual-time fact instead of a wall-clock race.
const GAPS_US: [u64; SCRIPT_STEPS] = [
    STEP_PERIOD_US,     // 0 — probe the encoder's attach phase and the bank
    STEP_PERIOD_US,     // 1 — re-read the bank, then set the datum
    STEP_PERIOD_US,     // 2 — enable the drive
    STEP_PERIOD_US,     // 3 — read the enable back
    STEP_PERIOD_US,     // 4 — start the forward train
    LEG_US + SETTLE_US, // 5 — the forward leg has run and come to rest
    STEP_PERIOD_US,     // 6 — latch reverse on DIR
    STEP_PERIOD_US,     // 7 — read the reversal back, start the reverse train
    LEG_US + SETTLE_US, // 8 — the reverse leg has run and come to rest
];

/// A fake firmware that runs **on the engine thread**: it makes the same
/// peripheral free-function calls a HAL trampoline would, at exact virtual
/// instants, and records the observations that are only true mid-sequence.
///
/// Its two pins are a high-impedance probe on the encoder's A/B nets, so the
/// attach-time phase is read where `system.net_state` would read it but at a
/// virtual instant rather than whenever the test thread got scheduled.
struct Script {
    pins: [PinDecl; 2],
    shaft: MotorShaft,
    encoder_input: EncoderInput,
    capture: Arc<Mutex<Capture>>,
    done: Arc<AtomicUsize>,
}

impl Component for Script {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let probe_a = io.pin("EA")?;
        let probe_b = io.pin("EB")?;
        let shaft = self.shaft.clone();
        let encoder_input = self.encoder_input.clone();
        let capture = Arc::clone(&self.capture);
        let done = Arc::clone(&self.done);
        let rearm = io.clone();
        // Anchor on the virtual instant of attach — 0 for every run, because
        // the engine holds time until the whole system has attached and
        // started (`engine::Command::ReleaseTime`).
        let anchor = virtual_clock::virtual_us();
        let step = Arc::new(AtomicUsize::new(0));
        let cursor = Arc::new(AtomicU64::new(anchor));
        let schedule = Arc::clone(&step);
        let next_us = Arc::clone(&cursor);

        io.on_wake(move |now_us| {
            let index = schedule.load(Ordering::SeqCst);
            {
                let mut capture = capture.lock().unwrap();
                match index {
                    // The encoder drove its own count-0 phase at attach while
                    // the pins idled high; the bridge counted the transitions
                    // between the two. Read the phase off the nets, and the
                    // offset out of the bank.
                    0 => {
                        capture.at_us.push(("attach probe", now_us));
                        capture.enc_a = Some(probe_a.sense());
                        capture.enc_b = Some(probe_b.sense());
                        capture.bank_first = Some(encoder::value(0));
                    }
                    // Nothing has moved since, so the bank must read the same
                    // — then that offset becomes the datum.
                    1 => {
                        capture.at_us.push(("datum", now_us));
                        capture.bank_second = Some(encoder::value(0));
                        encoder::set(0, 0);
                    }
                    2 => gpio::set_active(ENA_CHANNEL, true),
                    3 => {
                        capture.at_us.push(("enabled", now_us));
                        capture.enabled = Some(shaft.enabled());
                    }
                    4 => pulse_out::start(0, MOVE_STEPS, STEP_HZ),
                    5 => {
                        capture.at_us.push(("forward settled", now_us));
                        capture.forward_commanded = Some(shaft.commanded_steps());
                        capture.forward_emitted = Some(pulse_out::emitted(0));
                        capture.forward_bank = Some(encoder::value(0));
                        capture.forward_snapped = Some(encoder_input.snapped_updates());
                    }
                    6 => gpio::set_active(DIR_CHANNEL, true),
                    7 => {
                        capture.at_us.push(("reverse latched", now_us));
                        capture.reverse_latched = Some(!shaft.forward());
                        capture.reverse_train_direction =
                            Some(shaft.train().map(|train| train.direction));
                        pulse_out::start(0, MOVE_STEPS, STEP_HZ);
                    }
                    _ => {
                        capture.at_us.push(("reverse settled", now_us));
                        capture.final_commanded = Some(shaft.commanded_steps());
                        capture.final_bank = Some(encoder::value(0));
                        capture.final_snapped = Some(encoder_input.snapped_updates());
                    }
                }
            }
            let next = schedule.fetch_add(1, Ordering::SeqCst) + 1;
            done.store(next, Ordering::SeqCst);
            // Stop re-arming at the end of the script. The motor's own
            // `schedule_every` keeps the wheel turning, so virtual time does
            // not stop here as it does in `pulse_bridge_stepped.rs` — which is
            // exactly why every fact above was captured rather than polled.
            if let Some(gap) = GAPS_US.get(next) {
                rearm.schedule_at(next_us.fetch_add(*gap, Ordering::SeqCst) + gap);
            }
        });
        io.schedule_at(cursor.fetch_add(GAPS_US[0], Ordering::SeqCst) + GAPS_US[0]);
        Ok(())
    }
}

/// A high-impedance probe pin: it senses a net without loading it.
fn probe(number: &'static str) -> PinDecl {
    PinDecl {
        number,
        name: None,
        kind: PinKind::DigitalIn,
        stream: None,
        drive_impedance: None,
    }
}

/// The whole axis, wired by description alone.
#[rstest]
fn the_carriage_seam_closes_through_real_pins() {
    let _suite = suite_lock();
    let _stepped = Stepped::enter();

    gpio::init(GPIO_TABLE.len(), None);
    pulse_out::init(PULSE_TABLE.len());
    encoder::init(ENCODER_TABLE.len());

    let mcu = McuComponent::builder("p2")
        .gpio_table(GPIO_TABLE.to_vec())
        .bridge_gpio(ENA_CHANNEL, GpioDirection::Output)
        .bridge_gpio(DIR_CHANNEL, GpioDirection::Output)
        .pulse_out_table(PULSE_TABLE.to_vec())
        .bridge_pulse_out_with_direction(0, DIR_CHANNEL, PulseDirection::Reverse)
        .encoder_table(ENCODER_TABLE.to_vec())
        .bridge_encoder(0)
        .build()
        .expect("MCU builds from the channel tables");

    // One count per step and no load, so the assertions are about the loop
    // rather than about the plant's own parameters (which are unit-tested in
    // `embsim-models`). A lag well under the step interval, and an observation
    // cadence fine enough that the encoder walks every count.
    let motor = StepperMotor::new(stepper_motor::Config {
        tau_s: TAU_US as f64 / 1_000_000.0,
        load_loss: 0.0,
        // The machine's convention, stated on the drive: DIR high = reverse.
        dir_forward_level: Level::Low,
        // …and its open-collector enable: ENA low = enabled.
        enable_active_low: true,
        observe_interval_us: Some(OBSERVE_INTERVAL_US),
        ..stepper_motor::Config::new(1.0)
    })
    .expect("valid motor config");
    let shaft = motor.shaft();

    let encoder_model =
        QuadratureEncoder::new(quadrature_encoder::Config::new(1.0)).expect("valid encoder config");
    let input = encoder_model.input();
    {
        // The mechanical seam: the motor publishes millimetres, the encoder
        // applies its own counts/mm. Nothing here knows about the firmware.
        let input = input.clone();
        shaft.on_position_change(move |mm| input.set_position_mm(mm));
    }

    let capture = Arc::new(Mutex::new(Capture::default()));
    let done = Arc::new(AtomicUsize::new(0));
    let script = Script {
        pins: [probe("EA"), probe("EB")],
        shaft: shaft.clone(),
        encoder_input: input.clone(),
        capture: Arc::clone(&capture),
        done: Arc::clone(&done),
    };

    let harness = Harness::new()
        .connect_str("MCU.P8", "MOTOR.STEP")
        .expect("endpoint")
        .connect_str("MCU.P7", "MOTOR.DIR")
        .expect("endpoint")
        .connect_str("MCU.P6", "MOTOR.ENA")
        .expect("endpoint")
        .connect_str("ENC.A", "MCU.P20")
        .expect("endpoint")
        .connect_str("ENC.B", "MCU.P21")
        .expect("endpoint")
        // The script's own probe on the two encoder nets.
        .connect_str("ENC.A", "SCRIPT.EA")
        .expect("endpoint")
        .connect_str("ENC.B", "SCRIPT.EB")
        .expect("endpoint");

    let system = System::new()
        .component("MCU", Box::new(mcu))
        .component("MOTOR", Box::new(motor))
        .component("ENC", Box::new(encoder_model))
        .component("SCRIPT", Box::new(script))
        .harness(harness)
        .start()
        .expect("live system starts");

    // Liveness only: every value asserted below was recorded on the engine
    // thread at a fixed virtual instant, so this deadline cannot decide any
    // of them — it can only fail the run if the engine stopped entirely.
    assert!(
        wait_for(
            || done.load(Ordering::SeqCst) >= SCRIPT_STEPS,
            Duration::from_secs(30)
        ),
        "the script ran {} of {SCRIPT_STEPS} steps",
        done.load(Ordering::SeqCst)
    );
    assert!(system.engine_is_alive(), "engine must survive the script");

    let capture = capture.lock().unwrap();

    // --- the encoder's datum ---------------------------------------------
    // A quadrature counter has no absolute meaning at boot: the encoder drives
    // its own count-0 phase at attach while the pins idle high, so the bridge
    // faithfully counts the transitions between the two. That offset is the
    // physically correct answer, and homing is what turns it into a datum.
    assert_eq!(
        (capture.enc_a, capture.enc_b),
        (
            Some(NetState::Driven(Level::Low)),
            Some(NetState::Driven(Level::Low))
        ),
        "the encoder must present its count-0 phase at attach"
    );
    assert_eq!(
        capture.bank_first, capture.bank_second,
        "nothing moved between the two reads, so the bridge's attach-time \
         decode must already have settled before the datum was set"
    );

    // --- enable -----------------------------------------------------------
    assert_eq!(
        capture.enabled,
        Some(true),
        "an active (low) ENA must enable the drive"
    );

    // --- forward leg ------------------------------------------------------
    assert_eq!(
        capture.forward_commanded,
        Some(i64::from(MOVE_STEPS)),
        "the drive must reconstruct exactly {MOVE_STEPS} commanded steps"
    );
    assert_eq!(
        capture
            .forward_commanded
            .map(|steps| u64::try_from(steps).expect("forward travel is positive")),
        capture.forward_emitted,
        "and agree with the firmware's own emitted count"
    );
    assert_eq!(
        capture.forward_bank,
        Some(MOVE_STEPS as i32),
        "the firmware's encoder bank must reach {MOVE_STEPS} counts"
    );
    assert_eq!(
        capture.forward_snapped,
        Some(0),
        "every count must arrive as a real Gray transition on the pins"
    );

    // --- reverse leg ------------------------------------------------------
    assert_eq!(
        capture.reverse_latched,
        Some(true),
        "the drive latches reverse from its own DIR pin"
    );
    assert_eq!(
        capture.reverse_train_direction,
        Some(Some(PulseDirection::Reverse)),
        "…and the train carries the same direction — the two descriptions of \
         'reverse' must agree, or the axis is silently inverted"
    );
    assert_eq!(
        capture.final_commanded,
        Some(0),
        "{MOVE_STEPS} forward then {MOVE_STEPS} reverse is zero commanded steps"
    );
    assert_eq!(
        capture.final_bank,
        Some(0),
        "…and the carriage is back at its datum"
    );
    assert_eq!(capture.final_snapped, Some(0));

    println!(
        "[carriage seam] stepped; leg = {LEG_US} µs train + {SETTLE_US} µs settle \
         (20 τ). engine-instant captures: {:?}",
        capture.at_us
    );

    drop(capture);
    system.shutdown();
    pulse_out::reset();
    gpio::reset();
    encoder::reset();
}
