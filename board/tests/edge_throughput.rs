//! Can two nodes bit-bang a bus at each other over nets, fast, with the clock
//! stepping on events rather than on the wall?
//!
//! This is the load-bearing question for using the net engine as the ONLY
//! interface between nodes. A CPU bit-banging an SPI flash spends about 16 600
//! edges loading one kilobyte, and every one of them is a drive, an engine
//! resolution and a delivered sense. If that round trip costs a wall-clock
//! sleep, or a thread hop measured in tens of microseconds, then a boot that
//! takes 3 ms on a direct call takes minutes on nets and the design is dead.
//!
//! The earlier flash tests DID sleep — 300 µs per edge — and that is what made
//! this look impossible. They sleep because the driving thread is the TEST
//! thread, which is deliberately not part of the simulation and so cannot park
//! on a clock the engine is stepping. It has nothing to do with the engine's
//! own cost.
//!
//! So this measures the real thing: the master is an engine-hosted node whose
//! thread is a registered actor, exactly as an MCU's firmware thread is, and it
//! parks on virtual time between edges. Two assertions, and the second matters
//! as much as the first:
//!
//! 1. the whole exchange finishes in a time that makes a real boot affordable;
//! 2. `stepped_wall_sleep_count()` is ZERO — nothing escaped virtualization.
//!
//! A fast run with a non-zero sleep count would mean the clock quietly fell
//! back to the wall, which is the failure this design must not have.

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use embsim_board::{
    digital_drive, jesd8c01_lvcmos_thresholds, level_of, AttachError, Component, ComponentNetIo,
    DeadBand, Harness, Level, PinDecl, PinHandle, System,
};
use embsim_core::virtual_clock::{self, ClockMode};
use embsim_models::spi_flash::SpiNorFlash;
use embsim_models::spi_flash_component::{SpiNorFlashComponent, SPI_FLASH_PINS_SPI_ONLY};

/// Half a bit period, in virtual nanoseconds. 50 ns is a 10 MHz bus — the rate
/// a P2 bit-bangs its boot flash at, so the virtual time this consumes is
/// physically meaningful rather than an arbitrary tick.
const HALF_PERIOD_NS: u64 = 50;

/// Puts the process clock in stepped mode for the guard's lifetime.
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
struct MasterPins {
    cs: Option<PinHandle>,
    clk: Option<PinHandle>,
    di: Option<PinHandle>,
    dout: Option<PinHandle>,
}

/// What the master read back, and how much work it did.
#[derive(Debug, Default)]
struct Report {
    bytes: Mutex<Vec<u8>>,
    edges: AtomicU64,
    done: AtomicBool,
}

/// A bit-banging SPI master as an ENGINE-HOSTED node.
///
/// The distinction from the earlier flash tests is the whole point: its work
/// runs on a thread it owns, registered as a virtual-clock actor, so the
/// scheduler will not advance time while it is runnable and it can park on
/// virtual time instead of sleeping on the wall.
struct BitBangMaster {
    pins: [PinDecl; 4],
    handles: Arc<Mutex<MasterPins>>,
    report: Arc<Report>,
    /// How many bytes to read out of the part after the command frame.
    payload: usize,
}

impl BitBangMaster {
    fn new(handles: Arc<Mutex<MasterPins>>, report: Arc<Report>, payload: usize) -> Self {
        Self {
            pins: [
                PinDecl::digital_out("1").with_name("CS"),
                PinDecl::digital_out("2").with_name("CLK"),
                PinDecl::digital_out("3").with_name("DI"),
                PinDecl::digital_in("4", jesd8c01_lvcmos_thresholds(DeadBand::Unknown))
                    .with_name("DO"),
            ],
            handles,
            report,
            payload,
        }
    }
}

/// Drive a level and let virtual time carry the engine's resolution.
///
/// No wall sleep and no polling: parking is what lets the scheduler reach
/// quiescence, advance, and deliver. The cost is a condvar round trip, not a
/// thread-scheduler timeout.
fn edge(pin: &PinHandle, level: Level, report: &Report) {
    pin.set_drive(Some(digital_drive(level)));
    virtual_clock::wait_virtual_ns(HALF_PERIOD_NS);
    report.edges.fetch_add(1, Ordering::Relaxed);
}

impl Component for BitBangMaster {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let mut slot = self.handles.lock().expect("master pins");
        slot.cs = Some(io.pin("CS")?);
        slot.clk = Some(io.pin("CLK")?);
        slot.di = Some(io.pin("DI")?);
        slot.dout = Some(io.pin("DO")?);
        Ok(())
    }

    fn start(&mut self) {
        let handles = Arc::clone(&self.handles);
        let report = Arc::clone(&self.report);
        let payload = self.payload;
        std::thread::Builder::new()
            .name("spi-master".into())
            .spawn(move || {
                // The node's thread is an ACTOR: the scheduler will not advance
                // time while it is runnable, which is what keeps the exchange
                // deterministic as well as fast.
                let _actor = virtual_clock::register_actor("spi-master");
                let pins = handles.lock().expect("master pins");
                let (cs, clk, di, dout) = (
                    pins.cs.clone().unwrap(),
                    pins.clk.clone().unwrap(),
                    pins.di.clone().unwrap(),
                    pins.dout.clone().unwrap(),
                );
                drop(pins);

                let send = |byte: u8| {
                    for i in (0..8).rev() {
                        let bit = (byte >> i) & 1 != 0;
                        edge(&di, if bit { Level::High } else { Level::Low }, &report);
                        edge(&clk, Level::Low, &report);
                        edge(&clk, Level::High, &report);
                    }
                };
                let recv = || -> u8 {
                    let mut byte = 0u8;
                    for _ in 0..8 {
                        edge(&clk, Level::Low, &report);
                        edge(&clk, Level::High, &report);
                        // The part presents its bit on the rising edge, after
                        // taking MOSI — so sample here, not before.
                        byte = (byte << 1)
                            | u8::from(level_of(dout.net_report()) == Some(Level::High));
                    }
                    byte
                };

                // $03 READ DATA from address 0, then stream the payload.
                edge(&clk, Level::Low, &report);
                edge(&cs, Level::High, &report);
                edge(&cs, Level::Low, &report);
                send(0x03);
                send(0x00);
                send(0x00);
                send(0x00);
                let got: Vec<u8> = (0..payload).map(|_| recv()).collect();
                edge(&cs, Level::High, &report);

                *report.bytes.lock().expect("bytes") = got;
                report.done.store(true, Ordering::Release);
            })
            .expect("spawn the master node");
    }
}

/// Wall-clock wait, from OUTSIDE the simulation. This thread is not an actor
/// and must not park on a clock the engine is stepping.
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

/// One kilobyte over nets, edge by edge — the boot ROM's actual workload.
#[test]
fn a_kilobyte_crosses_the_nets_edge_by_edge_without_touching_the_wall() {
    let _stepped = Stepped::enter();
    let sleeps_before = virtual_clock::stepped_wall_sleep_count();

    // A recognisable image: a shifted byte stream would not reproduce it.
    let image: Vec<u8> = (0..1024).map(|i| (i * 7 % 251) as u8).collect();
    let handles = Arc::new(Mutex::new(MasterPins::default()));
    let report = Arc::new(Report::default());

    let harness = Harness::new()
        .connect_str("MASTER.CS", "FLASH.CS")
        .expect("endpoints parse")
        .connect_str("MASTER.CLK", "FLASH.CLK")
        .expect("endpoints parse")
        .connect_str("MASTER.DI", "FLASH.MOSI")
        .expect("endpoints parse")
        .connect_str("MASTER.DO", "FLASH.MISO")
        .expect("endpoints parse");

    let started = Instant::now();
    let _system = System::new()
        .component(
            "MASTER",
            Box::new(BitBangMaster::new(
                Arc::clone(&handles),
                Arc::clone(&report),
                image.len(),
            )),
        )
        .component(
            "FLASH",
            Box::new(
                SpiNorFlashComponent::new(SpiNorFlash::with_image(image.clone()))
                    .with_pins(&SPI_FLASH_PINS_SPI_ONLY),
            ),
        )
        .harness(harness)
        .start()
        .expect("the bench system starts");

    assert!(
        wait_for(
            || report.done.load(Ordering::Acquire),
            Duration::from_secs(120)
        ),
        "the exchange must finish; got {} edges",
        report.edges.load(Ordering::Relaxed)
    );
    let wall = started.elapsed();

    let edges = report.edges.load(Ordering::Relaxed);
    let got = report.bytes.lock().expect("bytes").clone();
    eprintln!(
        "  {edges} edges in {wall:.3?}  ({:.2} us/edge, {:.0} edges/s)",
        wall.as_secs_f64() * 1e6 / edges as f64,
        edges as f64 / wall.as_secs_f64()
    );

    assert_eq!(got, image, "every byte crossed the nets intact");

    // The tripwire. A fast run that quietly slept on the wall would mean the
    // clock fell back out of stepped mode, which is the one failure that would
    // make the speed number meaningless.
    assert_eq!(
        virtual_clock::stepped_wall_sleep_count(),
        sleeps_before,
        "no wait may escape virtualization while the clock is stepped"
    );
}
