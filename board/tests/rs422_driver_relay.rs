//! The AM26LS31 (`embsim_models::am26ls31`, Edge `U24`) relays a crossing
//! step clock as a differential `Drive::Periodic` — issue #80 / `NODES.md`
//! §12 item 5's open harness gap.
//!
//! A square wave on channel-1 `A` whose phases settle to two levels through
//! [`AM26LS31_INPUT_THRESHOLDS`] reaches `Y`/`Z` as a
//! complementary pair around that segment. A non-crossing wave stays on the
//! single-level path (released here: neither phase is a defensible level),
//! and an unchanged pair is not republished.

mod machine_parts;

use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};

use embsim_board::{
    AttachError, Component, ComponentNetIo, Drive, EndpointRef, Harness, Level, NetState,
    PeriodicSchedule, PinDecl, PinHandle, System, TheveninDrive,
};
use embsim_core::virtual_clock::{self, ClockMode};
use embsim_models::am26ls31::{Am26ls31, AM26LS31_INPUT_THRESHOLDS};
use machine_parts::SERVO_RAIL_VOLTS;
use rstest::rstest;

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

const SETTLE: Duration = Duration::from_secs(5);

fn ep(endpoint: &str) -> EndpointRef {
    EndpointRef::parse(endpoint).expect("endpoint parses")
}

/// Stimulus pin the test drives onto the driver's `1A`.
struct Driver {
    pins: [PinDecl; 1],
    handle: Arc<Mutex<Option<PinHandle>>>,
}

impl Component for Driver {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        *self.handle.lock().unwrap() = Some(io.pin("Q")?);
        Ok(())
    }
}

type Stamped = Arc<Mutex<Vec<(u64, NetState)>>>;

/// Sense probes on `1A`, `1Y`, and `1Z`.
struct Probe {
    pins: [PinDecl; 3],
    a: Stamped,
    y: Stamped,
    z: Stamped,
}

impl Component for Probe {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        for (pin, log) in [
            ("A", Arc::clone(&self.a)),
            ("Y", Arc::clone(&self.y)),
            ("Z", Arc::clone(&self.z)),
        ] {
            io.on_net_report(pin, move |state| {
                log.lock()
                    .unwrap()
                    .push((virtual_clock::virtual_ns(), state));
            })?;
        }
        Ok(())
    }
}

/// Active-high / active-low enable straps, as the EdgeBoard hard-wires them.
struct Enables {
    pins: [PinDecl; 2],
}

impl Component for Enables {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        io.pin("G")?.set_drive(Some(TheveninDrive {
            volts: SERVO_RAIL_VOLTS,
            impedance: 25.0,
        }));
        io.pin("NG")?.set_drive(Some(TheveninDrive {
            volts: 0.0,
            impedance: 25.0,
        }));
        Ok(())
    }
}

struct Bench {
    q: PinHandle,
    a: Stamped,
    y: Stamped,
    z: Stamped,
    _system: embsim_board::SystemHandle,
}

fn last(log: &Stamped) -> Option<(u64, NetState)> {
    log.lock().unwrap().last().copied()
}

fn bench() -> Bench {
    virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);

    let q_handle = Arc::new(Mutex::new(None));
    let a = Arc::new(Mutex::new(Vec::new()));
    let y = Arc::new(Mutex::new(Vec::new()));
    let z = Arc::new(Mutex::new(Vec::new()));

    let system = System::new()
        .component(
            "DRV",
            Box::new(Driver {
                pins: [PinDecl::digital_out("Q")],
                handle: Arc::clone(&q_handle),
            }),
        )
        .component("U24", Box::new(Am26ls31::new()))
        .component(
            "PROBE",
            Box::new(Probe {
                pins: [
                    PinDecl::digital_in("A", AM26LS31_INPUT_THRESHOLDS),
                    PinDecl::digital_in("Y", AM26LS31_INPUT_THRESHOLDS),
                    PinDecl::digital_in("Z", AM26LS31_INPUT_THRESHOLDS),
                ],
                a: Arc::clone(&a),
                y: Arc::clone(&y),
                z: Arc::clone(&z),
            }),
        )
        .component(
            "EN",
            Box::new(Enables {
                pins: [PinDecl::digital_out("G"), PinDecl::digital_out("NG")],
            }),
        )
        .harness(
            Harness::new()
                .power(ep("BENCH.VCC"), ep("U24.16"), SERVO_RAIL_VOLTS)
                .power(ep("BENCH.GND"), ep("U24.8"), 0.0)
                .connect(ep("DRV.Q"), ep("U24.1"))
                .connect(ep("PROBE.A"), ep("U24.1"))
                .connect(ep("PROBE.Y"), ep("U24.2"))
                .connect(ep("PROBE.Z"), ep("U24.3"))
                .connect(ep("EN.G"), ep("U24.4"))
                .connect(ep("EN.NG"), ep("U24.12")),
        )
        .start()
        .expect("bench starts");

    let q = q_handle
        .lock()
        .unwrap()
        .clone()
        .expect("driver pin shared at attach");
    Bench {
        q,
        a,
        y,
        z,
        _system: system,
    }
}

fn square_wave(high_volts: f64, freq_hz: u32, since_ns: u64) -> Drive {
    Drive::Periodic {
        hi: TheveninDrive {
            volts: high_volts,
            impedance: 25.0,
        },
        lo: TheveninDrive {
            volts: 0.0,
            impedance: 25.0,
        },
        segment: PeriodicSchedule {
            emitted: 0,
            freq_hz,
            total: None,
            since_ns,
        },
    }
}

/// A rail-to-rail clock on `1A` reaches `1Y`/`1Z` as a complementary
/// periodic pair around the same segment; a 0 V / 1.2 V wave (high phase
/// inside the AM26LS31's band) is no clock and no level, so the pair stays
/// released.
#[rstest]
fn a_crossing_clock_on_u24_a_reaches_the_differential_pair() {
    let _lock = suite_lock();
    let b = bench();

    // Idle high first: the level path, Y high / Z low.
    b.q.set_drive(Some(TheveninDrive {
        volts: SERVO_RAIL_VOLTS,
        impedance: 25.0,
    }));
    assert!(
        wait_for(
            || {
                matches!(last(&b.y), Some((_, NetState::Driven(Level::High))))
                    && matches!(last(&b.z), Some((_, NetState::Driven(Level::Low))))
            },
            SETTLE
        ),
        "level path: Y high / Z low; y={:?} z={:?}",
        b.y.lock().unwrap(),
        b.z.lock().unwrap()
    );

    // Non-crossing wave: 1.2 V sits between V_IL max 0.8 V and V_IH min 2.0 V.
    let low_swing = square_wave(1.2, 1_000_000, virtual_clock::virtual_ns());
    b.q.drive(low_swing);
    assert!(
        wait_for(
            || {
                matches!(last(&b.y), Some((_, NetState::Floating)))
                    && matches!(last(&b.z), Some((_, NetState::Floating)))
            },
            SETTLE
        ),
        "non-crossing wave releases the pair: y={:?} z={:?} a={:?}",
        b.y.lock().unwrap(),
        b.z.lock().unwrap(),
        b.a.lock().unwrap()
    );
    assert!(
        matches!(last(&b.a), Some((_, NetState::Periodic { .. }))),
        "input still carries the wave: {:?}",
        last(&b.a)
    );

    // Crossing clock: rail-to-rail at 1 MHz — must land on Y/Z as Periodic.
    let full = square_wave(SERVO_RAIL_VOLTS, 1_000_000, virtual_clock::virtual_ns());
    let Drive::Periodic { segment, .. } = full else {
        unreachable!()
    };
    b.q.drive(full);
    assert!(
        wait_for(
            || {
                matches!(
                    last(&b.y),
                    Some((_, NetState::Periodic { segment: s, .. })) if s == segment
                ) && matches!(
                    last(&b.z),
                    Some((_, NetState::Periodic { segment: s, .. })) if s == segment
                )
            },
            SETTLE
        ),
        "crossing clock must reach Y and Z: y={:?} z={:?} a={:?}",
        b.y.lock().unwrap(),
        b.z.lock().unwrap(),
        b.a.lock().unwrap()
    );
    let Some((
        _,
        NetState::Periodic {
            hi: y_hi, lo: y_lo, ..
        },
    )) = last(&b.y)
    else {
        panic!("Y must be periodic");
    };
    let Some((
        _,
        NetState::Periodic {
            hi: z_hi, lo: z_lo, ..
        },
    )) = last(&b.z)
    else {
        panic!("Z must be periodic");
    };
    assert_eq!(
        (y_hi, y_lo),
        (Level::High, Level::Low),
        "Y follows A: high phase high, low phase low"
    );
    assert_eq!(
        (z_hi, z_lo),
        (Level::Low, Level::High),
        "Z is A's complement in each phase"
    );

    // Same segment at a still-crossing swing: levels stay High/Low, so the
    // published pair is unchanged and must not republish (isolation_bridge
    // event budgets rely on that).
    let y_len = b.y.lock().unwrap().len();
    let z_len = b.z.lock().unwrap().len();
    let a_len = b.a.lock().unwrap().len();
    b.q.drive(Drive::Periodic {
        hi: TheveninDrive {
            volts: 4.5,
            impedance: 25.0,
        },
        lo: TheveninDrive {
            volts: 0.5,
            impedance: 25.0,
        },
        segment,
    });
    assert!(
        wait_for(|| b.a.lock().unwrap().len() > a_len, SETTLE),
        "input must be re-delivered under the new swing: a={:?}",
        b.a.lock().unwrap()
    );
    virtual_clock::wait_virtual_us(1_000);
    assert_eq!(
        b.y.lock().unwrap().len(),
        y_len,
        "unchanged Y pair must not republish"
    );
    assert_eq!(
        b.z.lock().unwrap().len(),
        z_len,
        "unchanged Z pair must not republish"
    );

    drop(b._system);
}
