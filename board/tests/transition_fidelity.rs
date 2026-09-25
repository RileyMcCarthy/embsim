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
    digital_drive, jesd8c01_lvcmos_thresholds, AttachError, Component, ComponentNetIo, DeadBand,
    DigitalReceiver, Harness, Level, PinDecl, PinHandle, System,
};
use embsim_core::virtual_clock::{self, ClockMode};

/// One half-period of the reference machine's step clock at full traverse:
/// 8192 steps/mm, 50 mm/s, two transitions per step (`HAL_pulseOut.c:66`).
const HALF_PERIOD_NS: u64 = 1220;

const TRANSITIONS: u64 = 20_000;

/// The virtual clock is process-global, so these tests take it one at a time.
/// Without this they pass under `--test-threads=1` and fail in parallel, which
/// is the worst way for a test to be wrong.
static CLOCK_LOCK: Mutex<()> = Mutex::new(());

struct Stepped(#[allow(dead_code)] std::sync::MutexGuard<'static, ()>);

impl Stepped {
    fn enter() -> Self {
        let guard = CLOCK_LOCK.lock().unwrap_or_else(|poisoned| {
            CLOCK_LOCK.clear_poison();
            poisoned.into_inner()
        });
        virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);
        Self(guard)
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
        let receiver = DigitalReceiver::new(io.pin("IN")?);
        io.on_sense("IN", move |sense| {
            let Some(level) = receiver.read(&sense) else {
                return;
            };
            let mut last = last.lock().unwrap();
            if *last != Some(level) {
                *last = Some(level);
                counts.sensed.fetch_add(1, Ordering::Relaxed);
            }
        })?;
        Ok(())
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
                pins: [PinDecl::digital_out("OUT")],
                counts: Arc::clone(&counts),
                pin: Arc::new(Mutex::new(None)),
                placement,
            }),
        )
        .component(
            "C",
            Box::new(Consumer {
                pins: [PinDecl::digital_in(
                    "IN",
                    jesd8c01_lvcmos_thresholds(DeadBand::Unknown),
                )],
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

// ============================================================
// The observability limit: what a transition's INSTANT is worth
// ============================================================
//
// The tests above count transitions. These ask the other question: does a
// consumer learn *when* each one happened?
//
// It matters because the two failures look nothing alike. A consumer that
// counts (a step counter, an encoder) needs only order. A consumer that
// MEASURES (a UART deframer recovering bit periods, a setup/hold check, a
// timeout, a waveform view) needs the instants to be distinct and true. The ISS
// stamps every edge in a 100 us slice identically today
// (`p2iss/src/lib.rs:1259` re-arms at `virtual_clock::virtual_ns()`), so this
// is the property that decides whether that is a latent bug or a live one.

/// What the consumer observed: one virtual timestamp per transition.
#[derive(Debug, Default)]
struct Stamps {
    at_ns: Mutex<Vec<u64>>,
    done: AtomicBool,
}

struct StampingConsumer {
    pins: [PinDecl; 1],
    stamps: Arc<Stamps>,
}

impl Component for StampingConsumer {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let stamps = Arc::clone(&self.stamps);
        let last: Mutex<Option<Level>> = Mutex::new(None);
        let receiver = DigitalReceiver::new(io.pin("IN")?);
        io.on_sense("IN", move |sense| {
            let Some(level) = receiver.read(&sense) else {
                return;
            };
            let mut last = last.lock().unwrap();
            if *last != Some(level) {
                *last = Some(level);
                stamps
                    .at_ns
                    .lock()
                    .unwrap()
                    .push(virtual_clock::virtual_ns());
            }
        })?;
        Ok(())
    }
}

/// Emits `n` transitions `apart_ns` apart. `apart_ns == 0` puts them all at one
/// instant, inside a single wake.
struct SpacedProducer {
    pins: [PinDecl; 1],
    pin: Arc<Mutex<Option<PinHandle>>>,
    stamps: Arc<Stamps>,
    n: u64,
    apart_ns: u64,
}

impl Component for SpacedProducer {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        *self.pin.lock().unwrap() = Some(io.pin("OUT")?);
        let (pin, stamps, n, apart) = (
            Arc::clone(&self.pin),
            Arc::clone(&self.stamps),
            self.n,
            self.apart_ns,
        );
        let arm = io.clone();
        let emitted = Mutex::new(0u64);
        io.on_wake_ns(move |now_ns| {
            let guard = pin.lock().unwrap();
            let Some(p) = guard.as_ref() else { return };
            let mut done_count = emitted.lock().unwrap();
            if *done_count >= n {
                stamps.done.store(true, Ordering::Release);
                return;
            }
            if apart == 0 {
                for i in 0..n {
                    let level = i % 2 == 0;
                    p.set_drive(Some(digital_drive(if level {
                        Level::High
                    } else {
                        Level::Low
                    })));
                }
                *done_count = n;
                stamps.done.store(true, Ordering::Release);
            } else {
                let level = (*done_count).is_multiple_of(2);
                p.set_drive(Some(digital_drive(if level {
                    Level::High
                } else {
                    Level::Low
                })));
                *done_count += 1;
                arm.schedule_at_ns(now_ns + apart);
            }
        });
        io.schedule_at_ns(apart.max(1));
        Ok(())
    }
}

/// Run `n` transitions `apart_ns` apart; return (observed, distinct instants).
fn stamped_run(n: u64, apart_ns: u64) -> (usize, usize) {
    let stamps = Arc::new(Stamps::default());
    let harness = Harness::new()
        .connect_str("P.OUT", "C.IN")
        .expect("endpoints parse");
    let _system = System::new()
        .component(
            "P",
            Box::new(SpacedProducer {
                pins: [PinDecl::digital_out("OUT")],
                pin: Arc::new(Mutex::new(None)),
                stamps: Arc::clone(&stamps),
                n,
                apart_ns,
            }),
        )
        .component(
            "C",
            Box::new(StampingConsumer {
                pins: [PinDecl::digital_in(
                    "IN",
                    jesd8c01_lvcmos_thresholds(DeadBand::Unknown),
                )],
                stamps: Arc::clone(&stamps),
            }),
        )
        .harness(harness)
        .start()
        .expect("system starts");

    let deadline = Instant::now() + Duration::from_secs(60);
    while !stamps.done.load(Ordering::Acquire) && Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(2));
    }
    std::thread::sleep(Duration::from_millis(200));

    let at = stamps.at_ns.lock().unwrap().clone();
    let mut distinct = at.clone();
    distinct.sort_unstable();
    distinct.dedup();
    (at.len(), distinct.len())
}

/// Transitions crowded into one instant are all DELIVERED but share a stamp.
///
/// This is the shape that matters: nothing is lost, so a counting consumer is
/// fine and a bit-banged SPI works — which is exactly why the ISS's identical
/// stamping has gone unnoticed. A consumer that measures an interval sees zero
/// for every one of them.
#[test]
fn transitions_at_one_instant_are_delivered_but_share_a_timestamp() {
    let _stepped = Stepped::enter();
    let (observed, distinct) = stamped_run(500, 0);
    eprintln!("  same instant : observed {observed}, distinct instants {distinct}");
    assert_eq!(
        observed, 500,
        "every transition is still delivered, in order"
    );
    // A handful, not one: the wake that emits them sits at its own instant and
    // the engine may advance once while draining. The point is the ratio —
    // hundreds of transitions collapsing onto a couple of timestamps.
    assert!(
        distinct <= 2,
        "they collapse onto one or two timestamps, so any consumer measuring an \
         interval reads zero for almost all of them. Got {distinct} distinct \
         instants for {observed} transitions. This is the ISS's current \
         behaviour for a whole 100 us slice."
    );
}

/// One nanosecond of separation is enough to make them distinguishable, so the
/// clock's resolution — not the engine — is the limit on representable
/// frequency.
#[test]
fn one_nanosecond_of_separation_is_enough_to_distinguish_transitions() {
    let _stepped = Stepped::enter();
    let (observed, distinct) = stamped_run(500, 1);
    eprintln!("  1 ns apart   : observed {observed}, distinct instants {distinct}");
    assert_eq!(observed, 500);
    assert_eq!(
        distinct, observed,
        "a 1 ns gap gives every transition its own instant — the ceiling on \
         representable signal frequency is the clock's nanosecond resolution \
         (1 GHz), not anything the engine imposes"
    );
}
