//! Live-engine end-to-end smoke: the two-component board from the design
//! doc's testing conventions — a fake MCU pin driver + a fake sensor —
//! exercising `System::start` attach / schedule / drive / sense /
//! shutdown through the public API only, without any consumer firmware.

use rstest::rstest;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use embsim_board::{
    AttachError, Board, Component, ComponentNetIo, Level, NetState, PartRegistry, PinDecl,
    PinHandle, PinKind, StreamRole, System, TheveninDrive,
};

// ============================================================
// Fixture: one net, a driver pin and a sensor pin
// ============================================================

/// Hand-written minimal KiCad s-expression netlist: U1.1 (driver) and
/// U2.1 (sensor) share the `SIG` net.
const NETLIST: &str = r#"(export (version "E")
  (components
    (comp (ref "U1")
      (value "FakeDriver")
      (libsource (lib "test") (part "FAKE_DRIVER") (description "")))
    (comp (ref "U2")
      (value "FakeSensor")
      (libsource (lib "test") (part "FAKE_SENSOR") (description ""))))
  (nets
    (net (code "1") (name "SIG") (class "Default")
      (node (ref "U1") (pin "1") (pintype "output"))
      (node (ref "U2") (pin "1") (pintype "input")))))"#;

const NONE: Option<StreamRole> = None;

const DRIVER_PINS: [PinDecl; 1] = [PinDecl {
    number: "1",
    name: Some("OUT"),
    kind: PinKind::DigitalOut,
    stream: NONE,
    drive_impedance: None,
}];

const SENSOR_PINS: [PinDecl; 1] = [PinDecl {
    number: "1",
    name: Some("IN"),
    kind: PinKind::DigitalIn,
    stream: NONE,
    drive_impedance: None,
}];

const LOW: TheveninDrive = TheveninDrive {
    volts: 0.0,
    impedance: 25.0,
};
const HIGH: TheveninDrive = TheveninDrive {
    volts: 3.3,
    impedance: 25.0,
};

/// Fake MCU pin driver: shares its pin handle with the test (a stand-in for
/// a component's own protocol thread) and schedules an immediate wakeup
/// whose handler drives the pin low.
struct FakeDriver {
    handle: Arc<Mutex<Option<PinHandle>>>,
}

impl Component for FakeDriver {
    fn pins(&self) -> &[PinDecl] {
        &DRIVER_PINS
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let pin = io.pin("OUT")?;
        let wake_pin = pin.clone();
        io.on_wake(move |_now_us| wake_pin.set_drive(Some(LOW)));
        io.schedule_at(0); // already due: fires immediately, in order
        *self.handle.lock().unwrap() = Some(pin);
        Ok(())
    }
}

/// Fake sensor: logs every sense delivery for the shared net.
struct FakeSensor {
    seen: Arc<Mutex<Vec<NetState>>>,
}

impl Component for FakeSensor {
    fn pins(&self) -> &[PinDecl] {
        &SENSOR_PINS
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let seen = Arc::clone(&self.seen);
        io.on_sense("IN", move |state| seen.lock().unwrap().push(state))?;
        Ok(())
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

// ============================================================
// The smoke test
// ============================================================

#[rstest]
fn live_system_routes_schedule_drive_and_sense_through_the_engine() {
    // The timer wheel samples the free-running virtual clock.
    embsim_core::virtual_clock::init(1.0, 1_000_000);

    let driver_handle: Arc<Mutex<Option<PinHandle>>> = Arc::new(Mutex::new(None));
    let sensor_seen: Arc<Mutex<Vec<NetState>>> = Arc::new(Mutex::new(Vec::new()));

    let mut registry = PartRegistry::new();
    {
        let handle = Arc::clone(&driver_handle);
        registry.register("FAKE_DRIVER", move |_decl| {
            Box::new(FakeDriver {
                handle: Arc::clone(&handle),
            })
        });
    }
    {
        let seen = Arc::clone(&sensor_seen);
        registry.register("FAKE_SENSOR", move |_decl| {
            Box::new(FakeSensor {
                seen: Arc::clone(&seen),
            })
        });
    }

    let parsed = embsim_board::netlist::parse(NETLIST).expect("fixture parses");
    let board = Board::from_netlist(parsed, &registry).expect("board builds");
    let system = System::new()
        .board("Rig", board)
        .start()
        .expect("live system starts");
    assert_eq!(system.component_refs().collect::<Vec<_>>(), ["U1", "U2"]);

    // The driver's scheduled wake drives SIG low; the sensor observes it.
    assert!(
        wait_for(
            || sensor_seen
                .lock()
                .unwrap()
                .contains(&NetState::Driven(Level::Low)),
            Duration::from_secs(5)
        ),
        "sensor must observe the wake-driven Low; saw {:?}",
        sensor_seen.lock().unwrap()
    );
    assert_eq!(
        system.net_state("Rig.SIG"),
        Some(NetState::Driven(Level::Low))
    );

    // Drive from the test thread (the component's "protocol thread"): the
    // shared pin handle enqueues to the same engine.
    let pin = driver_handle
        .lock()
        .unwrap()
        .clone()
        .expect("driver attached");
    pin.set_drive(Some(HIGH));
    assert!(
        wait_for(
            || system.net_state("Rig.SIG") == Some(NetState::Driven(Level::High)),
            Duration::from_secs(5)
        ),
        "cross-thread drive must resolve"
    );
    assert_eq!(pin.sense(), NetState::Driven(Level::High));
    assert!(
        sensor_seen
            .lock()
            .unwrap()
            .contains(&NetState::Driven(Level::High)),
        "sensor must observe the transition back to High"
    );

    // Clean shutdown: drop joins the engine thread promptly.
    let start = Instant::now();
    system.shutdown();
    assert!(
        start.elapsed() < Duration::from_secs(5),
        "shutdown must join cleanly"
    );

    // The shared handle survives shutdown; late drives are traced and
    // dropped, never a panic or a hang.
    pin.set_drive(None);
}
