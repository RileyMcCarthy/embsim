//! What does one edge cost when it goes through the net?
//!
//! Serial transport currently bypasses the net: `StreamRole::ByteSink` hands
//! whole bytes across, so no level ever reaches the resolver. Moving every
//! peripheral onto levels — one mechanism, digital as a projection — means each
//! bit becomes a drive, a resolve and a sense delivery. This measures that
//! round trip on the cheap path (one push-pull driver, no escalation to the
//! cluster solver) so the decision rests on a number rather than a guess.
//!
//! Reported as a test rather than asserted tightly: the useful output is the
//! nanoseconds-per-edge line, and a hard threshold would be a flaky test on
//! shared CI. It only fails if an edge somehow costs more than a millisecond,
//! which would mean the fast path is not being taken at all.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use embsim_board::{
    AttachError, Component, ComponentNetIo, Harness, PinDecl, PinHandle, PinKind, System,
    TheveninDrive,
};

/// Push-pull rails, at the resolver's default drive impedance.
const HIGH: TheveninDrive = TheveninDrive {
    volts: 3.3,
    impedance: 25.0,
};
const LOW: TheveninDrive = TheveninDrive {
    volts: 0.0,
    impedance: 25.0,
};

type Handle = Arc<Mutex<Option<PinHandle>>>;

const fn digital_pin(number: &'static str, kind: PinKind) -> PinDecl {
    PinDecl {
        number,
        name: None,
        kind,
        stream: None,
        drive_impedance: None,
    }
}

/// Drives one pin; the handle is published so the test can toggle it.
struct Driver {
    pins: [PinDecl; 1],
    handle: Handle,
}

impl Component for Driver {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }
    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let pin = io.pin("D")?;
        pin.set_drive(Some(LOW));
        *self.handle.lock().unwrap() = Some(pin);
        Ok(())
    }
}

/// Counts sense deliveries — one per resolved state change.
struct Probe {
    pins: [PinDecl; 1],
    seen: Arc<AtomicU64>,
}

impl Component for Probe {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }
    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let seen = Arc::clone(&self.seen);
        io.on_sense("S", move |_state| {
            seen.fetch_add(1, Ordering::Relaxed);
        })?;
        Ok(())
    }
}

#[test]
fn cost_of_one_edge_through_the_net() {
    const EDGES: u64 = 200_000;

    let handle: Handle = Arc::new(Mutex::new(None));
    let seen = Arc::new(AtomicU64::new(0));

    let _system = System::new()
        .component(
            "DRV",
            Box::new(Driver {
                pins: [digital_pin("D", PinKind::DigitalOut)],
                handle: Arc::clone(&handle),
            }),
        )
        .component(
            "PRB",
            Box::new(Probe {
                pins: [digital_pin("S", PinKind::DigitalIn)],
                seen: Arc::clone(&seen),
            }),
        )
        .harness(
            Harness::new()
                .connect_str("DRV.D", "PRB.S")
                .expect("endpoints parse"),
        )
        .start()
        .expect("system starts");

    let pin = handle.lock().unwrap().clone().expect("driver attached");

    // Let attach-time resolution settle so it is not counted.
    std::thread::sleep(Duration::from_millis(50));
    let base = seen.load(Ordering::Relaxed);

    let t0 = Instant::now();
    for i in 0..EDGES {
        // Alternate so every drive is a real state change: an unchanged state
        // is deduped by the resolver and would not exercise the delivery path.
        pin.set_drive(Some(if i & 1 == 0 { HIGH } else { LOW }));
    }
    // Wait for the resolver to drain, so the timing covers the whole round
    // trip rather than just the enqueue.
    let enqueue = t0.elapsed();
    let deadline = Instant::now() + Duration::from_secs(60);
    while seen.load(Ordering::Relaxed) - base < EDGES && Instant::now() < deadline {
        std::thread::yield_now();
    }
    let elapsed = t0.elapsed();
    let delivered = seen.load(Ordering::Relaxed) - base;

    let ns = elapsed.as_secs_f64() / delivered.max(1) as f64 * 1e9;
    println!("edges driven:    {EDGES}");
    println!("senses delivered:{delivered}");
    println!("wall:            {:.3} s", elapsed.as_secs_f64());
    println!(
        "  enqueue only:  {:.0} ns/edge",
        enqueue.as_secs_f64() / EDGES as f64 * 1e9
    );
    println!("per edge:        {ns:.0} ns  (drive -> resolve -> sense)");

    assert!(
        delivered >= EDGES,
        "resolver did not drain: {delivered} of {EDGES} delivered"
    );
    assert!(
        ns < 1_000_000.0,
        "an edge costing {ns:.0} ns means the fast path is not being taken"
    );
}
