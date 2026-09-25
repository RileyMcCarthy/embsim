//! The crystal is the rate the board delivers on `XI`, and `HUBSET`'s PLL
//! word multiplies it — observed as the spacing of the guest's own edges.
//!
//! A bench clock puts a 20 MHz train on the package's `XI` five
//! milliseconds in. The guest (fifteen instructions of PASM2, assembled
//! below and checked against flexspin's listing) starts at three — the
//! datasheet's restart delay after its supplies, up from the build — waits
//! one millisecond on RCFAST, then does what a flexspin program does in
//! its first
//! instructions: `HUBSET` the PLL word with the source still RCFAST, wait,
//! `HUBSET` again selecting the PLL — `20 MHz × 8 = 160 MHz`. Then four
//! back-to-back pad writes, and a byte to the debug pin.
//!
//! What the scope on `P0` proves:
//!
//! - **No crystal, no clock.** At four milliseconds the guest selects a
//!   clock derived from `XI` while nothing reaches the pin, and the node
//!   stalls it rather than inventing a frequency: the first edge lands
//!   after the clock arrived at 5 ms, never before.
//! - **The PLL multiplies the delivered rate.** Each pad write is one
//!   two-clock instruction; at 160 MHz that is 12.5 ns, so the four edges
//!   are 12 or 13 ns apart on the integer grid — not the 100 ns of RCFAST.
//!
//! And the node reports the crystal it was handed: 20 MHz, from the pin.
//!
//! The bench supplies `VDD` at 1.8 V and `RESN` released, so the
//! package's reset releases at the build and the guest's clock counts from
//! the START instant the restart delay later, 3 ms, and the `VIO_0_3` bank
//! at 3.3 V for the pad it writes.

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use embsim_board::{
    jesd8c01_lvcmos_thresholds, AttachError, Component, ComponentNetIo, DeadBand, DigitalReceiver,
    Drive, EndpointRef, Harness, Level, PeriodicSchedule, PinDecl, System, TheveninDrive,
};
use embsim_boards::p2::{P2Package, P2_RESTART_DELAY_NS};
use embsim_core::virtual_clock;
use embsim_p2_qemu::{P2Qemu, P2QemuError};

/// The P2's debug transmit pin, where the guest writes its byte.
const DEBUG_TX: u8 = 62;
/// When the bench clock starts: five milliseconds in, after the guest —
/// started at three, the restart delay after the build — has selected the
/// PLL at four.
const CLOCK_AT_NS: u64 = 5_000_000;
/// The rate on `XI`.
const CRYSTAL_HZ: u32 = 20_000_000;
/// One two-clock instruction at 160 MHz, on the nanosecond grid.
const PLL_INSTRUCTION_NS: [u64; 2] = [12, 13];

/// The guest, as flexspin 6.0.5 assembles it (`-2 -l`); the last jump
/// re-targeted by hand at itself (word 14):
///
/// ```text
///         org     0
///         waitx   ##20000               ' FF800027 FD64401F  1 ms on RCFAST
///         hubset  ##$010007F0           ' FF808003 FD67E000  PLL word, source RCFAST
///         waitx   ##100                 ' FF800000 FD64C81F
///         hubset  ##$010007F3           ' FF808003 FD67E600  select the PLL
///         drvh    #0                    ' FD640059
///         drvl    #0                    ' FD640058
///         drvh    #0                    ' FD640059
///         drvl    #0                    ' FD640058
///         mov     pa, #"K"              ' F607EC4B
///         wypin   pa, #62               ' FC27EC3E
///         jmp     #\14                  ' FD80000E  (to itself)
/// ```
///
/// `$010007F3` is `%0000_0001_000000_0000000111_1111_00_11`: PLL enabled
/// (bit 24), `D` = 0, `M` = 7, `P` = `%1111` (VCO direct), source `%11`
/// (the PLL) — `20 MHz / 1 × 8 / 1 = 160 MHz`, the word flexspin emits for
/// a 20 MHz crystal.
const PROGRAM: [u32; 15] = [
    0xFF80_0027,
    0xFD64_401F,
    0xFF80_8003,
    0xFD67_E000,
    0xFF80_0000,
    0xFD64_C81F,
    0xFF80_8003,
    0xFD67_E600,
    0xFD64_0059,
    0xFD64_0058,
    0xFD64_0059,
    0xFD64_0058,
    0xF607_EC4B,
    0xFC27_EC3E,
    0xFD80_000E,
];

/// A bench clock: a push-pull output that drives its square wave — rail to
/// rail at 25 Ω — from a wake at [`CLOCK_AT_NS`].
struct BenchClock {
    pins: [PinDecl; 1],
}

impl BenchClock {
    fn new() -> Self {
        Self {
            pins: [PinDecl::digital_out("OUT")],
        }
    }
}

impl Component for BenchClock {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let out = io.pin("OUT")?;
        io.on_wake_ns(move |now| {
            out.drive(Drive::Periodic {
                hi: TheveninDrive {
                    volts: 3.3,
                    impedance: 25.0,
                },
                lo: TheveninDrive {
                    volts: 0.0,
                    impedance: 25.0,
                },
                segment: PeriodicSchedule {
                    emitted: 0,
                    freq_hz: CRYSTAL_HZ,
                    total: None,
                    since_ns: now,
                },
            });
        });
        io.schedule_at_ns(CLOCK_AT_NS);
        Ok(())
    }
}

/// A scope on one net: the virtual instant of every level change it sees.
struct Scope {
    pins: [PinDecl; 1],
    instants: Arc<Mutex<Vec<u64>>>,
}

impl Scope {
    fn new() -> (Self, Arc<Mutex<Vec<u64>>>) {
        let instants = Arc::new(Mutex::new(Vec::new()));
        let scope = Self {
            pins: [PinDecl::digital_in(
                "A",
                jesd8c01_lvcmos_thresholds(DeadBand::Unknown),
            )],
            instants: Arc::clone(&instants),
        };
        (scope, instants)
    }
}

impl Component for Scope {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }
    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let instants = Arc::clone(&self.instants);
        let last = Mutex::new(None::<Level>);
        let receiver = DigitalReceiver::new(io.pin("A")?);
        io.on_sense("A", move |sense| {
            let Some(level) = receiver.read(&sense) else {
                return;
            };
            let mut last = last.lock().unwrap();
            if *last != Some(level) {
                *last = Some(level);
                instants.lock().unwrap().push(sense.at_ns);
            }
        })
    }
}

fn ep(endpoint: &str) -> EndpointRef {
    EndpointRef::parse(endpoint).expect("endpoint parses")
}

/// The bench supplies: the core rail inside its window and reset
/// released (the START gate), and the bank the guest writes its pad in — each
/// measured against the ground the bench holds at 0 V, which is not
/// implicit (`DESIGN.md` rule 6): a pin's sense is its voltage against its
/// reference pin, `GND`.
fn supplies(harness: Harness) -> Harness {
    harness
        .power(ep("BENCH.GND"), ep("P2.GND"), 0.0)
        .power(ep("BENCH.VDD"), ep("P2.VDD"), 1.8)
        .power(ep("BENCH.RESN"), ep("P2.RESN"), 3.3)
        .power(ep("BENCH.VIO_0_3"), ep("P2.VIO_0_3"), 3.3)
}

fn wait_for(mut pred: impl FnMut() -> bool, timeout: Duration) -> bool {
    let start = Instant::now();
    while start.elapsed() < timeout {
        if pred() {
            return true;
        }
        std::thread::sleep(Duration::from_millis(2));
    }
    pred()
}

#[test]
fn hubset_multiplies_the_rate_delivered_on_xi_and_stalls_without_one() {
    let image: Vec<u8> = PROGRAM.iter().flat_map(|w| w.to_le_bytes()).collect();
    let p2 = match P2Qemu::with_boot_rom(&image, &[]) {
        Ok(p2) => p2,
        Err(P2QemuError::Unavailable) => {
            eprintln!("\n*** SKIPPED: built without a QEMU tree (EMBSIM_QEMU_P2_BUILD). Asserted NOTHING.\n");
            return;
        }
        Err(e) => panic!("{e}"),
    };
    let handle = p2.handle();
    let package = P2Package::new(p2);
    let package_handle = package.handle();

    virtual_clock::init(0.0, 160_000_000);

    let (scope, instants) = Scope::new();
    let system = System::new()
        .component("P2", Box::new(package))
        .component("CLK", Box::new(BenchClock::new()))
        .component("SCOPE", Box::new(scope))
        .harness(supplies(
            Harness::new()
                .connect_str("CLK.OUT", "P2.XI")
                .expect("XI is a bench endpoint")
                .connect_str("SCOPE.A", "P2.P0")
                .expect("the pad is a bench endpoint"),
        ))
        .start()
        .expect("the bench starts");

    wait_for(
        || handle.console(DEBUG_TX).contains('K') || handle.halted(),
        Duration::from_secs(60),
    );
    let edges = instants.lock().unwrap().clone();
    assert_eq!(
        handle.console(DEBUG_TX),
        "K",
        "the guest ran to its byte after the clock arrived; edges={edges:?} stalled={} \
         crystal={:?} halted={} yields={} slices={}",
        handle.stalled(),
        handle.crystal_hz(),
        handle.halted(),
        handle.yields(),
        handle.slices(),
    );

    // The reset released at the build — the supplies were up before the
    // first wake — so the guest's clock counts from the end of the
    // datasheet's restart delay.
    assert_eq!(package_handle.started_at_ns(), Some(P2_RESTART_DELAY_NS));

    // The crystal is what the pin carried: the node and the package agree.
    assert_eq!(handle.crystal_hz(), Some(u64::from(CRYSTAL_HZ)));
    assert_eq!(package_handle.crystal_hz(), Some(u64::from(CRYSTAL_HZ)));
    assert!(!handle.stalled(), "running again since the clock arrived");

    // No crystal, no clock: the guest selected the PLL at ~4 ms and its
    // first pad write landed only after the rate reached XI at 5 ms.
    assert_eq!(edges.len(), 4, "four pad writes, four edges: {edges:?}");
    assert!(
        edges[0] >= CLOCK_AT_NS,
        "the guest did not run before its clock existed; first edge at {} ns, clock at \
         {CLOCK_AT_NS}",
        edges[0]
    );
    assert!(
        edges[0] < CLOCK_AT_NS + 1_000_000,
        "and resumed at once when it did: first edge at {} ns",
        edges[0]
    );

    // The PLL multiplied it: one two-clock instruction per edge at 160 MHz.
    let gaps: Vec<u64> = edges.windows(2).map(|w| w[1] - w[0]).collect();
    assert!(
        gaps.iter().all(|g| PLL_INSTRUCTION_NS.contains(g)),
        "edges one 160 MHz instruction apart (12–13 ns), never RCFAST's 100 ns: {gaps:?}"
    );
    // Each pad write stopped the guest exactly once. The `drvh` that follows
    // the PLL-selecting `HUBSET` in the same slice is the case that matters:
    // the guest stalls on that HUBSET with the pad change already yielded,
    // and the yield is the change's, consumed with it — a stall leaves no
    // flag behind to fire as a phantom pad change when the clock arrives.
    assert_eq!(
        handle.yields(),
        4,
        "four pad writes, four yields: a stall consumes the yield of the pad change that \
         shared its slice; publishes={}",
        handle.publishes()
    );
    assert_eq!(system.escalated_solves(), 0, "projections only");
    drop(system);
}
