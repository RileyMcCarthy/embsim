//! Virtual-clock initialization guard: this binary deliberately NEVER calls
//! `embsim_core::virtual_clock::init` (the clock is process-global and
//! init-once, so the uninitialized path is only testable in its own test
//! binary — do not add clock-using tests to this file).
//!
//! A component requesting a schedule before the clock exists must fail that
//! request loudly (`Finding::VirtualClockUninitialized`) while the engine
//! thread stays alive and keeps resolving drives — never the old failure
//! mode of `virtual_us()` panicking the engine into a silent zombie.

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use embsim_board::{
    AttachError, Board, Component, ComponentNetIo, Finding, Level, NetState, PartRegistry, PinDecl,
    PinHandle, PinKind, StreamRole, System, TheveninDrive,
};

/// U1.1 (driver) and U2.1 (sensor) share the `SIG` net.
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

/// Driver that asks for a periodic wake at attach — before any test code
/// could have initialized the clock — and shares its pin handle.
struct FakeDriver {
    handle: Arc<Mutex<Option<PinHandle>>>,
}

impl Component for FakeDriver {
    fn pins(&self) -> &[PinDecl] {
        &DRIVER_PINS
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        io.on_wake(|_now_us| {});
        io.schedule_every(1_000); // needs the (uninitialized) clock
        *self.handle.lock().unwrap() = Some(io.pin("OUT")?);
        Ok(())
    }
}

/// Sensor facade (pure sense; keeps the fixture two-sided).
struct FakeSensor;

impl Component for FakeSensor {
    fn pins(&self) -> &[PinDecl] {
        &SENSOR_PINS
    }

    fn attach(&mut self, _io: ComponentNetIo) -> Result<(), AttachError> {
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

#[test]
fn uninitialized_clock_fails_the_schedule_loudly_and_engine_survives() {
    let driver_handle: Arc<Mutex<Option<PinHandle>>> = Arc::new(Mutex::new(None));

    let mut registry = PartRegistry::new();
    {
        let handle = Arc::clone(&driver_handle);
        registry.register("FAKE_DRIVER", move |_decl| {
            Box::new(FakeDriver {
                handle: Arc::clone(&handle),
            })
        });
    }
    registry.register("FAKE_SENSOR", |_decl| Box::new(FakeSensor));

    let parsed = embsim_board::netlist::parse(NETLIST).expect("fixture parses");
    let board = Board::from_netlist(parsed, &registry).expect("board builds");
    let system = System::new()
        .board("Rig", board)
        .start()
        .expect("live system starts");

    // The schedule request was dropped loudly, not panicked on.
    assert!(
        wait_for(
            || system
                .findings()
                .contains(&Finding::VirtualClockUninitialized {
                    context: "schedule_every".to_string(),
                }),
            Duration::from_secs(5)
        ),
        "the dropped schedule must surface as a finding; got {:?}",
        system.findings()
    );

    // The engine survived and still serves drives.
    assert!(system.engine_is_alive(), "engine must stay alive");
    let pin = driver_handle
        .lock()
        .unwrap()
        .clone()
        .expect("driver attached");
    pin.set_drive(Some(LOW));
    assert!(
        wait_for(
            || system.net_state("Rig.SIG") == Some(NetState::Driven(Level::Low)),
            Duration::from_secs(5)
        ),
        "drives must keep resolving after the dropped schedule"
    );
    assert!(system.engine_is_alive());
}
