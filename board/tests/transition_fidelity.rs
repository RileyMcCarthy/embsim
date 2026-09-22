//! Can a voltage level carry a COUNT?
//!
//! `p2iss/src/pulse.rs:40` says no, and measured it: driving a step train edge
//! by edge lost all but ~1.5% of the commanded distance at 20 mm/s, because a
//! service arriving late "collapses the edges it missed into one `set_drive` of
//! the final level — an even number of them is no change at all".
//!
//! The mechanism is in the PRODUCER, not the engine. `pulse.rs:241`:
//!
//! ```text
//! let step = state.advance(now_ns);   // works out N transitions were due
//! if step.fired > 0 {
//!     let level = state.level;        // ...the FINAL level after all N
//!     self.handle.set_drive(...);     // and issues ONE drive
//! }
//! ```
//!
//! It catches up by computing the END STATE instead of emitting each
//! transition, so an even N lands back on the starting level and the consumer
//! is told nothing. The engine never saw the other transitions to lose them.
//!
//! These two tests separate the claims. One emits every transition; the other
//! catches up the way `pulse.rs` does. Same net, same count, same consumer.
//!
//! The answer matters because it decides whether the rate channel is
//! irreducible (levels cannot carry counts) or a COST decision (they can, at a
//! price) — and `edgerate.rs` prices it at ~50% of a simulated second for a
//! step train at full traverse.

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use embsim_board::{
    digital_drive, level_of, AttachError, Component, ComponentNetIo, Harness, Level, PinDecl,
    PinHandle, PinKind, System,
};
use embsim_core::virtual_clock::{self, ClockMode};

/// One half-period of the reference machine's step clock at full traverse:
/// 8192 steps/mm, 50 mm/s, two transitions per step (`HAL_pulseOut.c:66`).
const HALF_PERIOD_NS: u64 = 1220;

const TRANSITIONS: u64 = 20_000;

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

#[derive(Debug, Default)]
struct Counts {
    driven: AtomicU64,
    /// What the consumer was actually told about. The only number that counts.
    sensed: AtomicU64,
    done: AtomicBool,
}

/// How the producer places its transitions in time.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Placement {
    /// Each transition at its own future instant — one per wake, re-armed.
    OwnInstant,
    /// The producer CATCHES UP: it works out how many transitions were due,
    /// walks its own level that many times, and drives the final level once.
    /// This is `pulse.rs::service` exactly.
    CatchUp,
}

struct Producer {
    pins: [PinDecl; 1],
    counts: Arc<Counts>,
    pin: Arc<Mutex<Option<PinHandle>>>,
    placement: Placement,
}

impl Component for Producer {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        *self.pin.lock().unwrap() = Some(io.pin("OUT")?);
        let (counts, pin, placement) = (
            Arc::clone(&self.counts),
            Arc::clone(&self.pin),
            self.placement,
        );
        let arm = io.clone();
        io.on_wake_ns(move |now_ns| {
            let guard = pin.lock().unwrap();
            let Some(p) = guard.as_ref() else { return };
            match placement {
                Placement::OwnInstant => {
                    let n = counts.driven.load(Ordering::Relaxed);
                    if n >= TRANSITIONS {
                        counts.done.store(true, Ordering::Release);
                        return;
                    }
                    let level = if n % 2 == 0 { Level::High } else { Level::Low };
                    p.set_drive(Some(digital_drive(level)));
                    counts.driven.fetch_add(1, Ordering::Relaxed);
                    arm.schedule_at_ns(now_ns + HALF_PERIOD_NS);
                }
                Placement::CatchUp => {
                    // Serviced periodically, catching up in bulk. Each wake owes
                    // BATCH transitions; the producer walks its own level that
                    // many times and drives the result ONCE — which is what
                    // `pulse.rs::service` does with `step.fired`.
                    const BATCH: u64 = 64;
                    let n = counts.driven.load(Ordering::Relaxed);
                    if n >= TRANSITIONS {
                        counts.done.store(true, Ordering::Release);
                        return;
                    }
                    let mut level = n % 2 == 0;
                    for _ in 0..BATCH {
                        level = !level;
                    }
                    p.set_drive(Some(digital_drive(if level {
                        Level::High
                    } else {
                        Level::Low
                    })));
                    counts.driven.fetch_add(BATCH, Ordering::Relaxed);
                    arm.schedule_at_ns(now_ns + HALF_PERIOD_NS * BATCH);
                }
            }
        });
        io.schedule_at_ns(HALF_PERIOD_NS);
        Ok(())
    }
}

/// Counts the transitions it is told about. A pure state machine, no thread.
struct Consumer {
    pins: [PinDecl; 1],
    counts: Arc<Counts>,
}

impl Component for Consumer {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let counts = Arc::clone(&self.counts);
        let last: Mutex<Option<Level>> = Mutex::new(None);
        io.on_sense("IN", move |state| {
            let Some(level) = level_of(state) else { return };
            let mut last = last.lock().unwrap();
            if *last != Some(level) {
                *last = Some(level);
                counts.sensed.fetch_add(1, Ordering::Relaxed);
            }
        })?;
        Ok(())
    }
}

fn decl(number: &'static str, kind: PinKind) -> PinDecl {
    PinDecl {
        number,
        name: None,
        kind,
        stream: None,
        drive_impedance: None,
    }
}

fn run(placement: Placement) -> (u64, u64) {
    let counts = Arc::new(Counts::default());
    let harness = Harness::new()
        .connect_str("P.OUT", "C.IN")
        .expect("endpoints parse");
    let _system = System::new()
        .component(
            "P",
            Box::new(Producer {
                pins: [decl("OUT", PinKind::DigitalOut)],
                counts: Arc::clone(&counts),
                pin: Arc::new(Mutex::new(None)),
                placement,
            }),
        )
        .component(
            "C",
            Box::new(Consumer {
                pins: [decl("IN", PinKind::DigitalIn)],
                counts: Arc::clone(&counts),
            }),
        )
        .harness(harness)
        .start()
        .expect("system starts");

    let deadline = Instant::now() + Duration::from_secs(120);
    while !counts.done.load(Ordering::Acquire) && Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(2));
    }
    // Let anything still in flight settle before reading the count.
    std::thread::sleep(Duration::from_millis(200));
    (
        counts.driven.load(Ordering::Relaxed),
        counts.sensed.load(Ordering::Relaxed),
    )
}

/// A transition placed at its own instant is never lost — even at the step
/// clock's full-traverse rate, which is the signal the rate channel exists for.
#[test]
fn transitions_at_their_own_instants_are_never_lost() {
    let _stepped = Stepped::enter();
    let (driven, sensed) = run(Placement::OwnInstant);
    eprintln!("  own-instant : driven {driven}, sensed {sensed}");
    assert_eq!(driven, TRANSITIONS, "the producer emitted the whole train");
    assert_eq!(
        sensed, driven,
        "every transition placed at its own instant reached the consumer — so a \
         level DOES carry a count, provided nothing is folded"
    );
}

/// And the control: a producer that CATCHES UP loses almost everything, which
/// is `pulse.rs`'s measured 1.5%. Without this the test above proves nothing —
/// it would pass on a consumer that simply counted everything it was told.
///
/// Note where the loss is. The engine never saw the missing transitions; the
/// producer collapsed them before driving. So the fix for a count-bearing
/// signal is not a different carrier, it is a producer that emits every
/// transition — which costs one wheel deadline each, ~50% of a simulated
/// second for a step train at full traverse (`examples/edgerate.rs`).
#[test]
fn a_producer_that_catches_up_in_bulk_loses_almost_everything() {
    let _stepped = Stepped::enter();
    let (driven, sensed) = run(Placement::CatchUp);
    eprintln!(
        "  catch-up    : driven {driven}, sensed {sensed}  ({:.2}% survived)",
        100.0 * sensed as f64 / driven as f64
    );
    // `>=` not `==`: the last batch overshoots, which is fine — what matters is
    // that the producer owed at least the whole train.
    assert!(
        driven >= TRANSITIONS,
        "the producer owed the whole train, got {driven}"
    );
    assert!(
        sensed < driven / 10,
        "a bulk catch-up must lose most of the train: it drives the FINAL level \
         once per batch, and an even batch is no change at all. Got {sensed} of \
         {driven}. If this no longer reproduces, the positive test above is \
         measuring nothing."
    );
}
