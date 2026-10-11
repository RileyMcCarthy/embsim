//! Components drop while the engine is still serving commands.
//!
//! A component's `Drop` drives its pin and waits for its own sense. The
//! drive is resolved only while the engine thread is alive, so the sense
//! arriving is the drop order.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use embsim_board::{
    AttachError, Component, ComponentNetIo, PinDecl, PinHandle, System, TheveninDrive,
};
use vibes_behaviour::{behaviour, expect, Test};

struct LastDrive {
    pins: [PinDecl; 1],
    pin: Option<PinHandle>,
    saw_high: Arc<AtomicBool>,
}

impl Component for LastDrive {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let saw_high = Arc::clone(&self.saw_high);
        io.on_sense("OUT", move |sense| {
            if sense.volts.is_some_and(|volts| volts > 3.0) {
                saw_high.store(true, Ordering::SeqCst);
            }
        })?;
        self.pin = Some(io.pin("OUT")?);
        Ok(())
    }
}

impl Drop for LastDrive {
    fn drop(&mut self) {
        let Some(pin) = self.pin.take() else {
            return;
        };
        pin.set_drive(Some(TheveninDrive {
            volts: 3.3,
            impedance: 25.0,
        }));
        let started = Instant::now();
        while !self.saw_high.load(Ordering::SeqCst) && started.elapsed() < Duration::from_secs(1) {
            std::thread::sleep(Duration::from_millis(1));
        }
    }
}

#[test]
fn a_component_can_drive_while_it_drops() {
    behaviour!(Test {
        id: "engine.components-drop-before-join",
        covers: Some("board/src/system.rs#Drop for SystemHandle"),
        given: "a live system whose component drives its pin to 3.3 volts from Drop",
    });
    expect!(
        "sense-while-engine-alive",
        "the pin's sense reports that 3.3 volts before Drop returns",
        "the engine is still serving commands while components drop, so the last drive is resolved and delivered",
    );
    let saw_high = Arc::new(AtomicBool::new(false));
    let system = System::new()
        .component(
            "DRV",
            Box::new(LastDrive {
                pins: [PinDecl::digital_out("OUT").with_idle(Some(TheveninDrive {
                    volts: 0.0,
                    impedance: 25.0,
                }))],
                pin: None,
                saw_high: Arc::clone(&saw_high),
            }),
        )
        .start()
        .expect("the bench starts");
    drop(system);
    assert!(
        saw_high.load(Ordering::SeqCst),
        "the drop-time drive must be sensed while the engine is alive"
    );
}
