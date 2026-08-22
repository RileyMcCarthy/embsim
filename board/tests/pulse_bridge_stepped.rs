//! Determinism Oracle 1 over the **peripheral pin bridges**: the same scripted
//! motion scenario, run N times under `ClockMode::Stepped`, must produce a
//! byte-identical engine event log — order *and* every virtual timestamp
//! (`DETERMINISM.md`, "Proving it: determinism testing").
//!
//! This is the case `determinism.rs` cannot host. Its cases are pure board
//! components; these bridges route through the **process-default peripheral
//! banks**, which are global state that binary deliberately never touches
//! (`TESTING.md` rule 5). So this is its own binary with its own suite lock,
//! exactly as `ads122u04_stepped.rs` is.
//!
//! What makes it deterministic, and what the assertion is really testing:
//!
//! - The stimulus is **engine-hosted**. A `Firmware` component re-arms itself
//!   with `schedule_at` and makes its HAL-style calls from `on_wake`, so every
//!   `pulse_out::*` / `gpio::*` call happens on the engine thread at an exact
//!   virtual instant. A test-thread script would sample virtual time at a
//!   wall-determined moment and stamp it into the segment's `since_us` —
//!   `DETERMINISM.md` T1 §4's "take your time from the engine", applied to a
//!   peripheral bridge.
//! - The script **stops re-arming**, so the wheel empties, virtual time stops,
//!   and the log is bounded. A `schedule_every` would keep firing until
//!   shutdown and make the log length a race.
//! - The rate-carried representation is what makes this affordable: the whole
//!   run's step traffic is a handful of records, so a golden comparison is
//!   over engine *decisions*, not over a hundred thousand edges.

use rstest::rstest;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};

use embsim_board::mcu::{
    EncoderChannelConfig, GpioChannelConfig, GpioDirection, PulseOutChannelConfig,
};
use embsim_board::{
    AttachError, Component, ComponentNetIo, EventLog, Harness, McuComponent, PinDecl, PinKind,
    PulseDirection, StreamRole, System, TheveninDrive,
};
use embsim_core::virtual_clock::{self, ClockMode};
use embsim_peripherals::{encoder, gpio, pulse_out};

/// How many times the scenario runs. `DETERMINISM.md` specifies N = 5.
const RUNS: usize = 5;

/// Virtual-time spacing between script steps.
const STEP_PERIOD_US: u64 = 1_000;

const ENA_CHANNEL: usize = 0;
const DIR_CHANNEL: usize = 1;
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
const PULSE_TABLE: [PulseOutChannelConfig; 1] = [PulseOutChannelConfig { pin: 8 }];
const ENCODER_TABLE: [EncoderChannelConfig; 1] = [EncoderChannelConfig {
    pin_a: 20,
    pin_b: 21,
}];

/// The reference machine's resolution.
const STEPS_PER_MM: u32 = 8_192;

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
// The scripted firmware (engine-hosted)
// ============================================================

/// `(A, B)` phases in the order that counts up.
const PHASES: [(bool, bool); 4] = [(false, false), (true, false), (true, true), (false, true)];

const HIGH: TheveninDrive = TheveninDrive {
    volts: 3.3,
    impedance: 25.0,
};
const LOW: TheveninDrive = TheveninDrive {
    volts: 0.0,
    impedance: 25.0,
};

/// Number of script steps; the run is complete when this many have fired.
const SCRIPT_STEPS: usize = 12;

/// A fake firmware that runs **on the engine thread**: it makes the same
/// peripheral free-function calls a HAL trampoline would, and drives the
/// endstop contact and the encoder phases through its own pins.
struct Firmware {
    pins: [PinDecl; 3],
    done: Arc<AtomicUsize>,
}

impl Component for Firmware {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let switch = io.pin("SW")?;
        let phase_a = io.pin("A")?;
        let phase_b = io.pin("B")?;
        let done = Arc::clone(&self.done);
        let rearm = io.clone();
        // Anchor on the virtual instant of attach — 0 for every run, because
        // the engine holds time until the whole system has attached and
        // started (`engine::Command::ReleaseTime`).
        let anchor = virtual_clock::virtual_us();
        let step = Arc::new(AtomicUsize::new(0));
        let schedule = Arc::clone(&step);

        io.on_wake(move |_now_us| {
            let index = schedule.load(Ordering::SeqCst);
            match index {
                // Enable the drive, then commit to 1 mm/s.
                0 => gpio::set_active(ENA_CHANNEL, true),
                1 => pulse_out::start_velocity(0, STEPS_PER_MM),
                // Reverse mid-train: the segment is re-anchored and re-signed.
                2 => gpio::set_active(DIR_CHANNEL, true),
                // Retarget to 2 mm/s, still reversing.
                3 => pulse_out::set_frequency(0, 2 * STEPS_PER_MM),
                // The endstop closes: an external drive sensed back into the
                // firmware's GPIO bank.
                4 => switch.set_drive(Some(LOW)),
                // Four quadrature transitions, walking forward from the pins'
                // idle phase (both high = index 2) so every move is a single
                // Gray transition the decoder counts.
                5..=8 => {
                    let (a, b) = PHASES[(index - 5 + 3) % 4];
                    phase_a.set_drive(Some(if a { HIGH } else { LOW }));
                    phase_b.set_drive(Some(if b { HIGH } else { LOW }));
                }
                // Back to forward, then stop.
                9 => gpio::set_active(DIR_CHANNEL, false),
                10 => pulse_out::stop(0),
                _ => gpio::set_active(ENA_CHANNEL, false),
            }
            let next = schedule.fetch_add(1, Ordering::SeqCst) + 1;
            done.store(next, Ordering::SeqCst);
            // Stop re-arming at the end of the script: the wheel empties,
            // virtual time stops, and the log is bounded.
            if next < SCRIPT_STEPS {
                rearm.schedule_at(anchor + (next as u64 + 1) * STEP_PERIOD_US);
            }
        });
        io.schedule_at(anchor + STEP_PERIOD_US);
        Ok(())
    }
}

fn output(number: &'static str) -> PinDecl {
    PinDecl {
        number,
        name: None,
        kind: PinKind::DigitalOut,
        stream: None,
        drive_impedance: None,
    }
}

/// The drive at the far end: a pulse sink plus two sensed logic inputs, so the
/// event log carries the trains *and* the DIR/ENA transitions.
struct StepDrive {
    pins: [PinDecl; 3],
}

impl Component for StepDrive {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        io.on_pulse("STEP", |_train| {})?;
        for pin in ["DIR", "ENA"] {
            io.on_sense(pin, |_state| {})?;
        }
        Ok(())
    }
}

fn input(number: &'static str, stream: Option<StreamRole>) -> PinDecl {
    PinDecl {
        number,
        name: None,
        kind: PinKind::DigitalIn,
        stream,
        drive_impedance: None,
    }
}

// ============================================================
// One run
// ============================================================

/// Run the scripted scenario once against a freshly-anchored clock and return
/// its engine event log. The caller owns the clock mode.
fn run_scenario() -> EventLog {
    gpio::init(GPIO_TABLE.len(), None);
    pulse_out::init(PULSE_TABLE.len());
    encoder::init(ENCODER_TABLE.len());

    let done = Arc::new(AtomicUsize::new(0));

    let mcu = McuComponent::builder("p2")
        .gpio_table(GPIO_TABLE.to_vec())
        .bridge_gpio(ENA_CHANNEL, GpioDirection::Output)
        .bridge_gpio(DIR_CHANNEL, GpioDirection::Output)
        .bridge_gpio(ESTOP_CHANNEL, GpioDirection::Input)
        .pulse_out_table(PULSE_TABLE.to_vec())
        .bridge_pulse_out_with_direction(0, DIR_CHANNEL, PulseDirection::Reverse)
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
        .connect_str("MCU.P16", "FW.SW")
        .expect("endpoints parse")
        .connect_str("MCU.P20", "FW.A")
        .expect("endpoints parse")
        .connect_str("MCU.P21", "FW.B")
        .expect("endpoints parse");

    let system = System::new()
        .component("MCU", Box::new(mcu))
        .component(
            "DRIVE",
            Box::new(StepDrive {
                pins: [
                    input("STEP", Some(StreamRole::PulseSink)),
                    input("DIR", None),
                    input("ENA", None),
                ],
            }),
        )
        .component(
            "FW",
            Box::new(Firmware {
                pins: [output("SW"), output("A"), output("B")],
                done: Arc::clone(&done),
            }),
        )
        .harness(harness)
        .event_log()
        .start()
        .expect("live system starts");

    let log = system.event_log();
    assert!(
        wait_for(
            || done.load(Ordering::SeqCst) >= SCRIPT_STEPS,
            Duration::from_secs(10)
        ),
        "the script ran {} of {SCRIPT_STEPS} steps",
        done.load(Ordering::SeqCst)
    );
    assert!(system.engine_is_alive(), "engine must survive the script");
    system.shutdown();

    pulse_out::reset();
    gpio::reset();
    encoder::reset();
    log
}

// ============================================================
// The assertion
// ============================================================

/// The D1 promise, extended to the pin bridges: in stepped mode the whole
/// normalized log — order and every virtual timestamp — is identical across N
/// runs of the scripted motion scenario.
#[rstest]
fn stepped_bridge_logs_are_identical_across_runs() {
    let _suite = suite_lock();

    let mut logs: Vec<Vec<String>> = Vec::new();
    for _ in 0..RUNS {
        let _stepped = Stepped::enter();
        logs.push(run_scenario().normalized());
    }

    let baseline = &logs[0];
    assert!(
        !baseline.is_empty(),
        "the event log recorded nothing — every comparison below would be \
         vacuously true"
    );
    // The bridges must actually be in the log, or this asserts the determinism
    // of a system that did nothing.
    for expect in ["pulse ", "freq=8192hz", "dir=rev", "freq=16384hz"] {
        assert!(
            baseline.iter().any(|record| record.contains(expect)),
            "no record contains {expect:?}; the pulse bridge did not run.\n{}",
            baseline.join("\n")
        );
    }

    for (run, log) in logs.iter().enumerate().skip(1) {
        if let Some(index) = log
            .iter()
            .zip(baseline)
            .position(|(a, b)| a != b)
            .or_else(|| (log.len() != baseline.len()).then_some(log.len().min(baseline.len())))
        {
            panic!(
                "run {run} diverged from run 0 in the FULL timestamped projection.\n\
                 first difference at record {index} (run 0 has {}, run {run} has {})\n\
                 run 0:     {}\n\
                 run {run}: {}",
                baseline.len(),
                log.len(),
                baseline.get(index).map_or("<end of log>", String::as_str),
                log.get(index).map_or("<end of log>", String::as_str),
            );
        }
    }

    println!(
        "[stepped] pulse/gpio/encoder bridges: {} records/run, {RUNS}/{RUNS} runs \
         byte-identical including every v_us",
        baseline.len()
    );
}
