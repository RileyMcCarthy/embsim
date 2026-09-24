//! Where the per-edge cost actually goes.
//!
//! `edge_throughput.rs` says a kilobyte crosses the nets in ~190 ms. This says
//! WHY, by taking the same loop apart:
//!
//! ```text
//!   drive only, no park        0.02 us/edge
//!   park only, no drive       10.90 us/edge
//!   drive + park, flash peer   9.54 us/edge
//! ```
//!
//! An edge is twenty nanoseconds. Everything else is the actor park/wake round
//! trip on the stepped clock — two thread context switches — and it costs five
//! hundred times what the edge does.
//!
//! That is the number to argue with before optimising anything: the net engine
//! and the device model are not the cost, SYNCHRONISING WITH VIRTUAL TIME is,
//! and a node only has to do that when it needs to OBSERVE something. A node
//! that merely drives can fire and forget.
//!
//! Keep this runnable. If a change to the clock or the engine moves these
//! numbers, that is the thing worth knowing.
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use embsim_board::{
    digital_drive, AttachError, Component, ComponentNetIo, Harness, IdleDrive, Level, PinDecl,
    PinHandle, PinKind, System,
};
use embsim_core::virtual_clock::{self, ClockMode};
use embsim_models::spi_flash::SpiNorFlash;
use embsim_models::spi_flash_component::{SpiNorFlashComponent, SPI_FLASH_PINS_SPI_ONLY};

const N: u64 = 20_000;

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

#[derive(Default)]
struct Slot {
    clk: Option<PinHandle>,
    di: Option<PinHandle>,
}

#[derive(Debug, Default)]
struct Out {
    ns: AtomicU64,
    done: AtomicBool,
}

/// `mode`: 0 = park only, 1 = drive only (no park), 2 = drive + park.
struct Probe {
    pins: [PinDecl; 4],
    slot: Arc<Mutex<Slot>>,
    out: Arc<Out>,
    mode: u8,
}

impl Component for Probe {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }
    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let mut s = self.slot.lock().unwrap();
        s.clk = Some(io.pin("CLK")?);
        s.di = Some(io.pin("MOSI")?);
        Ok(())
    }
    fn start(&mut self) {
        let (slot, out, mode) = (Arc::clone(&self.slot), Arc::clone(&self.out), self.mode);
        std::thread::spawn(move || {
            let _a = virtual_clock::register_actor("probe");
            let clk = slot.lock().unwrap().clk.clone().unwrap();
            let t = Instant::now();
            for i in 0..N {
                let lvl = if i % 2 == 0 { Level::High } else { Level::Low };
                if mode != 0 {
                    clk.set_drive(Some(digital_drive(lvl)));
                }
                if mode != 1 {
                    virtual_clock::wait_virtual_ns(50);
                }
            }
            out.ns
                .store(t.elapsed().as_nanos() as u64, Ordering::Release);
            out.done.store(true, Ordering::Release);
        });
    }
}

fn run(label: &str, mode: u8, with_flash: bool) {
    let _s = Stepped::enter();
    let decl = |number, name, kind| PinDecl {
        number,
        name: Some(name),
        kind,
        stream: None,
        drive_impedance: None,
        idle: IdleDrive::KindDefault,
    };
    let slot = Arc::new(Mutex::new(Slot::default()));
    let out = Arc::new(Out::default());
    let probe = Probe {
        pins: [
            decl("1", "CS", PinKind::DigitalOut),
            decl("2", "CLK", PinKind::DigitalOut),
            decl("3", "MOSI", PinKind::DigitalOut),
            decl("4", "MISO", PinKind::DigitalIn),
        ],
        slot: Arc::clone(&slot),
        out: Arc::clone(&out),
        mode,
    };
    let mut h = Harness::new();
    let mut sys = System::new().component("P", Box::new(probe));
    if with_flash {
        h = h
            .connect_str("P.CS", "F.CS")
            .unwrap()
            .connect_str("P.CLK", "F.CLK")
            .unwrap()
            .connect_str("P.MOSI", "F.MOSI")
            .unwrap()
            .connect_str("P.MISO", "F.MISO")
            .unwrap();
        sys = sys.component(
            "F",
            Box::new(
                SpiNorFlashComponent::new(SpiNorFlash::blank(4096))
                    .with_pins(&SPI_FLASH_PINS_SPI_ONLY),
            ),
        );
    } else {
        // Nets still exist, just nothing on the far end.
        h = h.connect_str("P.CS", "P.MISO").unwrap();
    }
    let _sys = sys.harness(h).start().expect("start");
    let t0 = Instant::now();
    while !out.done.load(Ordering::Acquire) && t0.elapsed() < Duration::from_secs(60) {
        std::thread::sleep(Duration::from_millis(1));
    }
    let ns = out.ns.load(Ordering::Acquire);
    println!("{label:<34} {:>8.2} us/edge", ns as f64 / N as f64 / 1000.0);
}

fn main() {
    run("park only, no drive", 0, false);
    run("drive only, no park", 1, false);
    run("drive + park, no peer", 2, false);
    run("drive + park, flash peer", 2, true);
}
