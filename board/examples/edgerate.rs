//! How many edges per second can the engine actually RESOLVE and DELIVER?
//!
//! `edgecost.rs` measured 0.02 µs to *enqueue* a drive. That is not the cost of
//! an edge: a drive is only an edge once the engine has resolved the net and
//! delivered the sense to the peer, and a node that drives twice without a
//! resolution in between has produced ONE transition, not two. So the number
//! that decides whether an interface can be carried as edges is this one:
//! **resolved, delivered transitions per second.**
//!
//! # What this does NOT decide
//!
//! This example was written to test whether the step clock could be carried as
//! edges rather than as a rate, and it answered the wrong question. Cost is not
//! why `StreamRole::PulseTrain` exists. **Correctness is**, and it was measured
//! before this was written — `p2iss/src/pulse.rs:40`:
//!
//! > driving it edge by edge makes every step a wheel deadline, and any service
//! > that arrives late collapses the edges it missed into one `set_drive` of the
//! > final level — an even number of them is no change at all, so a consumer
//! > that counts transitions loses them silently. At 20 mm/s that lost all but
//! > ~1.5% of the commanded distance.
//!
//! The general rule, which the node contract needs: **a net carries a LEVEL, and
//! a level cannot carry a COUNT.** Any signal whose information is the *number*
//! of transitions rather than their presence is lossy on a level-resolved net
//! the moment a consumer can sample late. No amount of engine throughput fixes
//! that, because the loss happens in the resolution, not in the queue.
//!
//! So what remains useful here is the raw figure: how many resolved, delivered
//! transitions per second the engine sustains. That bounds every signal whose
//! information IS its edges — serial, SPI, chip selects — and it is worth
//! keeping current.
//!
//! The driver here is an engine-hosted node re-arming at the next instant, which
//! is the fully-synced shape: no actor, no park, one edge per wake.

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use embsim_board::{
    digital_drive, level_of, AttachError, Component, ComponentNetIo, Harness, IdleDrive, Level,
    PinDecl, PinHandle, PinKind, System,
};
use embsim_core::virtual_clock::{self, ClockMode};

/// Virtual nanoseconds between edges: 1220 ns, so ~820 000 transitions/s.
///
/// That is the reference machine at full traverse — 8192 steps/mm, 50 mm/s, and
/// **two transitions per step** (`HAL_pulseOut.c:66`, `_wypin(pulses * 2U)`).
/// An earlier version of this file called it 410 kHz and then divided the
/// result by 410 000, halving the price it reported.
const HALF_PERIOD_NS: u64 = 1220;

const EDGES: u64 = 40_000;

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
    /// Transitions the PEER actually saw. The only number that counts an edge.
    sensed: AtomicU64,
    done: AtomicBool,
}

/// Drives one edge per wake and re-arms at the next half-period — the
/// fully-synced shape: on the engine thread, no actor, no park.
struct Driver {
    pins: [PinDecl; 1],
    counts: Arc<Counts>,
    pin: Arc<Mutex<Option<PinHandle>>>,
}

impl Component for Driver {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        *self.pin.lock().unwrap() = Some(io.pin("OUT")?);
        let (counts, pin) = (Arc::clone(&self.counts), Arc::clone(&self.pin));
        let arm = io.clone();
        io.on_wake_ns(move |now_ns| {
            let n = counts.driven.load(Ordering::Relaxed);
            if n >= EDGES {
                counts.done.store(true, Ordering::Release);
                return;
            }
            let level = if n % 2 == 0 { Level::High } else { Level::Low };
            if let Some(p) = pin.lock().unwrap().as_ref() {
                p.set_drive(Some(digital_drive(level)));
            }
            counts.driven.fetch_add(1, Ordering::Relaxed);
            // Re-arm at the NEXT instant, which is what lets time advance and
            // makes each drive its own resolved edge.
            arm.schedule_at_ns(now_ns + HALF_PERIOD_NS);
        });
        io.schedule_at_ns(HALF_PERIOD_NS);
        Ok(())
    }
}

/// Counts transitions it is told about. A pure state machine: no thread.
struct Listener {
    pins: [PinDecl; 1],
    counts: Arc<Counts>,
}

impl Component for Listener {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let counts = Arc::clone(&self.counts);
        io.on_sense("IN", move |state| {
            if level_of(state).is_some() {
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
        idle: IdleDrive::KindDefault,
    }
}

fn main() {
    let _stepped = Stepped::enter();
    let counts = Arc::new(Counts::default());

    let harness = Harness::new()
        .connect_str("DRV.OUT", "LSN.IN")
        .expect("endpoints parse");

    let t0 = Instant::now();
    let _system = System::new()
        .component(
            "DRV",
            Box::new(Driver {
                pins: [decl("OUT", PinKind::DigitalOut)],
                counts: Arc::clone(&counts),
                pin: Arc::new(Mutex::new(None)),
            }),
        )
        .component(
            "LSN",
            Box::new(Listener {
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
    let wall = t0.elapsed();

    let driven = counts.driven.load(Ordering::Relaxed);
    let sensed = counts.sensed.load(Ordering::Relaxed);
    let per_edge_us = wall.as_secs_f64() * 1e6 / driven.max(1) as f64;
    let rate = driven as f64 / wall.as_secs_f64();

    println!();
    println!("driven          : {driven}");
    println!("sensed by peer  : {sensed}");
    println!("wall            : {wall:.3?}");
    println!("per edge        : {per_edge_us:.2} us   (resolve + deliver, no park)");
    println!("engine can do   : {:.0} edges/s", rate);
    println!();
    // 8192 steps/mm x 50 mm/s x 2 transitions/step.
    const STEP_TRANSITIONS_PER_S: f64 = 819_200.0;
    println!("a step train at 50 mm/s is {STEP_TRANSITIONS_PER_S:.0} transitions/s:");
    println!(
        "  cost per simulated second : {:.2} s  ({:.0}%)",
        STEP_TRANSITIONS_PER_S / rate,
        100.0 * STEP_TRANSITIONS_PER_S / rate
    );
    println!();
    println!("  ...but cost is NOT why the step clock is a rate. A level cannot");
    println!("  carry a count: a late consumer folds missed toggles into one");
    println!("  drive, and an even number of them is no change at all. Measured");
    println!("  at 1.5% of commanded distance surviving, at 20 mm/s.");
    println!("  See p2iss/src/pulse.rs:40. Throughput does not fix it.");
}
