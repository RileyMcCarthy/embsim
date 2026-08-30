//! A UART probe that speaks **levels**, for tests that need to be the other
//! end of a component's serial pins.
//!
//! Shared because the alternative is a copy per test binary, and a copy is
//! where bit order quietly diverges. It is built on the same
//! [`SerialLevelBridge`] the components under test use — deliberately: these
//! tests are about the *component*, not the codec, and a probe that reframed
//! the wire its own way would be testing two things at once.
//!
//! The codec itself is checked against something it did not produce, in
//! `board/src/uart.rs` (a hand-written waveform) and `board/tests/serial_
//! levels.rs` (an independently written peer). Those are the tests that would
//! catch a codec that agrees with itself while being wrong.

#![allow(dead_code)] // each test binary uses a different part of this

use std::sync::atomic::AtomicBool;
use std::sync::{Arc, Mutex, MutexGuard};

use embsim_board::uart::{FramingError, UartFraming};
use embsim_board::{AttachError, Component, ComponentNetIo, PinDecl, PinKind, SerialLevelBridge};

/// What the probe saw and what it can say, shared with the test body.
#[derive(Debug, Default)]
struct ProbeState {
    /// Every frame the probe decoded, good and bad, in wire order.
    frames: Vec<Result<u8, FramingError>>,
    /// Live once the probe has attached.
    bridge: Option<Arc<SerialLevelBridge>>,
}

/// Handle onto a [`UartProbe`]: clone it into the registry and keep one for
/// the test body.
#[derive(Debug, Clone, Default)]
pub struct ProbeHandle(Arc<Mutex<ProbeState>>);

impl ProbeHandle {
    pub fn new() -> Self {
        Self::default()
    }

    fn lock(&self) -> MutexGuard<'_, ProbeState> {
        self.0.lock().expect("probe state never poisoned")
    }

    /// Queue bytes onto the wire. Panics if the probe has not attached — a
    /// test that writes before `System::start` has a fixture bug, and silently
    /// dropping the bytes would surface as an unexplained timeout.
    pub fn send(&self, bytes: &[u8]) {
        let bridge = self
            .lock()
            .bridge
            .clone()
            .expect("probe must be attached before it can transmit");
        bridge.transmit(bytes);
    }

    /// Bytes decoded so far, framing errors skipped.
    pub fn received(&self) -> Vec<u8> {
        self.lock()
            .frames
            .iter()
            .filter_map(|frame| frame.as_ref().ok().copied())
            .collect()
    }

    /// Every frame, including the ones that did not decode.
    pub fn frames(&self) -> Vec<Result<u8, FramingError>> {
        self.lock().frames.clone()
    }
}

/// A two-pin UART with no stream role at all.
pub struct UartProbe {
    pins: [PinDecl; 2],
    framing: UartFraming,
    handle: ProbeHandle,
    shutdown: Arc<AtomicBool>,
}

impl UartProbe {
    /// Build a probe whose TX and RX pins carry the given identities.
    ///
    /// `tx` and `rx` are the pin *numbers* as the netlist or harness names
    /// them; `tx_name` / `rx_name` are the optional pin functions.
    pub fn new(
        tx: &'static str,
        tx_name: Option<&'static str>,
        rx: &'static str,
        rx_name: Option<&'static str>,
        framing: UartFraming,
        handle: ProbeHandle,
    ) -> Self {
        Self {
            pins: [
                PinDecl {
                    number: tx,
                    name: tx_name,
                    kind: PinKind::DigitalOut,
                    stream: None,
                    drive_impedance: None,
                },
                PinDecl {
                    number: rx,
                    name: rx_name,
                    kind: PinKind::DigitalIn,
                    stream: None,
                    drive_impedance: None,
                },
            ],
            framing,
            handle,
            shutdown: Arc::new(AtomicBool::new(false)),
        }
    }
}

impl Component for UartProbe {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let tx_id = self.pins[0].name.unwrap_or(self.pins[0].number);
        let rx_id = self.pins[1].name.unwrap_or(self.pins[1].number);

        let bridge = Arc::new(SerialLevelBridge::new(
            self.framing,
            io.pin(tx_id)?,
            io.clone(),
            Arc::clone(&self.shutdown),
        ));
        // An idle asynchronous line still drives: without it the peer has no
        // reference against which the first start bit is a falling edge.
        bridge.idle();
        self.handle.lock().bridge = Some(Arc::clone(&bridge));

        {
            let handle = self.handle.clone();
            let bridge = Arc::clone(&bridge);
            io.on_sense(rx_id, move |state| {
                let frames = bridge.receive_sense(state);
                handle.lock().frames.extend(frames);
            })?;
        }
        {
            let handle = self.handle.clone();
            let io_wake = io.clone();
            io.on_wake_ns(move |now_ns| {
                let (frames, next) = bridge.service(now_ns);
                handle.lock().frames.extend(frames);
                if let Some(at) = next {
                    io_wake.schedule_at_ns(at);
                }
            });
        }
        Ok(())
    }
}

impl Drop for UartProbe {
    fn drop(&mut self) {
        self.shutdown
            .store(true, std::sync::atomic::Ordering::Relaxed);
    }
}
