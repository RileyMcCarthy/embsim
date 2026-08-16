//! Stepped-clock engine mechanics: the barrier, the time-release, and the two
//! things that go wrong (`DETERMINISM.md` T1 §3/§4).
//!
//! `determinism.rs` proves the *outcome* — N runs, byte-identical logs. This
//! binary proves the *mechanism*, one property per case, so a regression says
//! which rule broke rather than "the trace changed":
//!
//! - a registered actor's drives land at the virtual instants it parked for,
//!   and the engine never runs ahead of it;
//! - virtual time does not advance until the whole system has attached and
//!   started ([`embsim_board::engine`]'s `ReleaseTime`), which is what makes a
//!   *multi*-component schedule reproducible;
//! - an actor that never parks is reported as
//!   [`Finding::QuiescenceTimeout`] rather than hanging the engine.
//!
//! Its own test binary per `TESTING.md` rule 5: these cases pin the
//! process-global clock **and its mode**.

use rstest::rstest;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};

use embsim_board::{
    AttachError, Component, ComponentNetIo, EventLog, Finding, Harness, PinDecl, PinHandle,
    PinKind, System, TheveninDrive,
};
use embsim_core::virtual_clock::{self, ClockMode};

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

/// Stepped mode for the lifetime of the guard; free-running afterwards.
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
        std::thread::sleep(Duration::from_millis(1));
    }
    pred()
}

const fn analog_pin(name: &'static str) -> PinDecl {
    PinDecl {
        number: name,
        name: None,
        kind: PinKind::Analog,
        stream: None,
        drive_impedance: None,
    }
}

type PinSlot = Arc<Mutex<Option<PinHandle>>>;

/// Publishes its pin handle so a test thread can drive it from outside.
struct Terminal {
    pins: [PinDecl; 1],
    handle: PinSlot,
}

impl Component for Terminal {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        *self.handle.lock().unwrap() = Some(io.pin("P")?);
        Ok(())
    }
}

/// Records the virtual instant of its own `attach`, after a deliberate wall
/// delay — the probe for the time-release barrier.
struct SlowAttach {
    pins: [PinDecl; 1],
    delay: Duration,
    attached_at_v_us: Arc<AtomicU64>,
}

impl Component for SlowAttach {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, _io: ComponentNetIo) -> Result<(), AttachError> {
        // A *wall* sleep on the assembling thread, deliberately not a virtual
        // wait: it models a component whose attach does real work (opening a
        // device, reading a fixture) while the engine is already live.
        std::thread::sleep(self.delay);
        self.attached_at_v_us
            .store(virtual_clock::virtual_us(), Ordering::SeqCst);
        Ok(())
    }
}

/// Schedules one wakeup at `at_us`, so the wheel is non-empty from attach.
struct OneShot {
    pins: [PinDecl; 1],
    at_us: u64,
    fired: Arc<AtomicBool>,
}

impl Component for OneShot {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let fired = Arc::clone(&self.fired);
        io.on_wake(move |_| fired.store(true, Ordering::SeqCst));
        io.schedule_at(self.at_us);
        Ok(())
    }
}

/// Extract the `v_us` stamps of every `drive_applied` record in a normalized
/// log.
fn drive_stamps(log: &EventLog) -> Vec<u64> {
    log.records()
        .iter()
        .filter(|r| matches!(r.event, embsim_board::EngineEvent::DriveApplied { .. }))
        .map(|r| r.v_us)
        .collect()
}

// ============================================================
// The barrier: an actor cannot run ahead of virtual time
// ============================================================

/// A registered actor as the stimulus source — the multi-threaded case the
/// scripted cases in `determinism.rs` deliberately avoid.
///
/// The actor parks at 1 ms, 2 ms, 3 ms … and drives on each wake. The engine
/// must advance to exactly those instants and no further before the actor has
/// run, so every drive is applied at its own deadline. Two runs, byte-equal.
///
/// This is the property the whole actor registry exists for: without it the
/// actor's drives would land at whatever virtual instant the engine had
/// wandered to by the time the OS scheduled the thread.
#[rstest]
fn a_registered_actor_drives_at_exactly_the_instants_it_parked_for() {
    let _suite = suite_lock();
    const PERIOD_US: u64 = 1_000;
    const STEPS: u64 = 6;

    let run = || -> Vec<u64> {
        let _stepped = Stepped::enter();
        let slot: PinSlot = Arc::new(Mutex::new(None));
        let system = System::new()
            .component(
                "T",
                Box::new(Terminal {
                    pins: [analog_pin("P")],
                    handle: Arc::clone(&slot),
                }),
            )
            .event_log()
            .start()
            .expect("system starts");
        let log = system.event_log();
        let pin = slot.lock().unwrap().clone().expect("pin attached");

        let done = Arc::new(AtomicBool::new(false));
        let finished = Arc::clone(&done);
        let actor = std::thread::spawn(move || {
            let _registration = virtual_clock::register_actor("test-stimulus");
            for step in 1..=STEPS {
                virtual_clock::wait_virtual_us(PERIOD_US);
                pin.set_drive(Some(TheveninDrive {
                    volts: 0.4 * step as f64,
                    impedance: 20.0 + step as f64,
                }));
            }
            finished.store(true, Ordering::SeqCst);
            // The registration guard drops here, on this thread, releasing the
            // barrier for good.
        });

        assert!(
            wait_for(
                || done.load(Ordering::SeqCst) && drive_stamps(&log).len() >= STEPS as usize,
                Duration::from_secs(10)
            ),
            "actor produced {} of {STEPS} drives",
            drive_stamps(&log).len()
        );
        actor.join().expect("actor joins");
        system.shutdown();
        drive_stamps(&log)
    };

    let expected: Vec<u64> = (1..=STEPS).map(|k| k * PERIOD_US).collect();
    let first = run();
    assert_eq!(
        first, expected,
        "each of the actor's drives must be applied at the virtual instant it parked for"
    );
    assert_eq!(run(), expected, "and identically on a second run");
}

// ============================================================
// The time-release barrier
// ============================================================

/// Virtual time is held until **every** component has attached and started.
///
/// The first component arms a wakeup 1 µs out, so without the barrier the
/// engine would happily advance while the *second* component is still
/// attaching — and that component's schedules would be anchored at a different
/// instant from run to run. The second component's attach takes a visible
/// wall delay and records the virtual time it saw; with the barrier in force
/// that reading is 0, run after run.
#[rstest]
fn virtual_time_is_held_until_every_component_has_attached() {
    let _suite = suite_lock();
    let _stepped = Stepped::enter();

    let fired = Arc::new(AtomicBool::new(false));
    let attached_at = Arc::new(AtomicU64::new(u64::MAX));
    let system = System::new()
        .component(
            "EARLY",
            Box::new(OneShot {
                pins: [analog_pin("P")],
                at_us: 1,
                fired: Arc::clone(&fired),
            }),
        )
        .component(
            "SLOW",
            Box::new(SlowAttach {
                pins: [analog_pin("P")],
                delay: Duration::from_millis(50),
                attached_at_v_us: Arc::clone(&attached_at),
            }),
        )
        .harness(
            Harness::new()
                .connect_str("EARLY.P", "SLOW.P")
                .expect("endpoints parse"),
        )
        .event_log()
        .start()
        .expect("system starts");

    assert_eq!(
        attached_at.load(Ordering::SeqCst),
        0,
        "virtual time must still be 0 when the last component attaches, even though \
         an earlier component already armed a wakeup and 50 ms of WALL time passed"
    );

    // And once released, the held wakeup fires normally.
    assert!(
        wait_for(|| fired.load(Ordering::SeqCst), Duration::from_secs(5)),
        "the wakeup armed before the release must still fire after it"
    );
    system.shutdown();
}

// ============================================================
// The wedge: an actor that never parks
// ============================================================

/// An actor that never parks cannot be waited out and cannot be stepped over.
/// The engine reports [`Finding::QuiescenceTimeout`] naming it and carries on
/// with an explicitly degraded guarantee — it must not hang, and it must not
/// pretend the run is still reproducible.
#[rstest]
fn an_actor_that_never_parks_is_reported_not_hung() {
    let _suite = suite_lock();
    let _stepped = Stepped::enter();

    let stop = Arc::new(AtomicBool::new(false));
    let spinning = Arc::clone(&stop);
    let spinner = std::thread::spawn(move || {
        let _registration = virtual_clock::register_actor("never-parks");
        while !spinning.load(Ordering::SeqCst) {
            std::thread::sleep(Duration::from_millis(1));
        }
    });
    // Let it register before the engine starts looking.
    assert!(
        wait_for(
            || virtual_clock::scheduler_state().running >= 1,
            Duration::from_secs(5)
        ),
        "the spinning actor must register"
    );

    let fired = Arc::new(AtomicBool::new(false));
    let system = System::new()
        .component(
            "EARLY",
            Box::new(OneShot {
                pins: [analog_pin("P")],
                at_us: 1_000,
                fired: Arc::clone(&fired),
            }),
        )
        // A short timeout so the case costs milliseconds, not the generous
        // production default.
        .quiescence_timeout(Duration::from_millis(50))
        .event_log()
        .start()
        .expect("system starts");

    let reported = |sys: &embsim_board::SystemHandle| {
        sys.findings().iter().any(|finding| {
            matches!(finding, Finding::QuiescenceTimeout { actors }
                if actors.iter().any(|a| a == "never-parks"))
        })
    };
    assert!(
        wait_for(|| reported(&system), Duration::from_secs(10)),
        "the engine must name the actor that never parked; findings: {:?}",
        system.findings()
    );
    assert!(
        system.engine_is_alive(),
        "the engine must report the wedge, not die of it"
    );
    // Degraded, not stopped: time advances anyway, so the held wakeup fires.
    assert!(
        wait_for(|| fired.load(Ordering::SeqCst), Duration::from_secs(10)),
        "the engine must keep advancing after reporting the stall"
    );

    stop.store(true, Ordering::SeqCst);
    spinner.join().expect("spinner joins");
    system.shutdown();
}
