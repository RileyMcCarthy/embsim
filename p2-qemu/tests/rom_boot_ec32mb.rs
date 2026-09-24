//! The chip boots off the module's own flash, as silicon does, with every
//! part a node on the board's real nets.
//!
//! The P2 is QEMU's target; the flash is `embsim_models`' generic part; the
//! board is the P2-EC32MB from its vendor netlist. Nothing in the P2 knows
//! there is a flash — it bit-bangs four pins the ROM chose, and a device on
//! those nets answers. Two facts of the board have to be true for that to
//! happen, and all are said as scenario, not wiring: DIP switch `S301`
//! position 2 (labelled FLASH) is closed, joining `P61` to the flash's `~CS`;
//! the `VIO_56_63` rail is up, so `R301` pulls that select high — which is
//! the strap the ROM samples to decide the flash is worth trying; and `S301`
//! position 4 (the P59 pull-down, `R303`) is closed, which is the module's
//! "boot from flash without waiting for a serial loader" setting. The ROM
//! really samples that last one: after a valid load it drives P59 high,
//! floats it, waits, and reads it back — low means boot now, high means try
//! serial first. With the switch open the node reads the level P59 was left
//! at, high, and the ROM goes looking for a loader instead. That was the
//! first divergence from p2core this test found, and it was the board's.
//!
//! The P2 is the QEMU core inside the P2 **package** (`embsim_boards::p2`):
//! the package declares the 86 pins the netlist gives `U100`, and the
//! crystal the core's PLL would multiply is the rate the board delivers on
//! `XI` — the module's TCXO through its buffer, not a number handed to the
//! node. The boot itself runs on RCFAST and never selects it, and on this
//! bench the TCXO's own rail is still a facade (see the end of the test).
//!
//! What is asserted is what the boot DID: the flash served reads at `0` and
//! `$400` (stage-1, then the application), and the application's one byte
//! reached the debug pin. And that every edge was the P2's own: the flash saw
//! real clock edges over the net, delivered one instant at a time.

use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use embsim_board::{
    level_of, AttachError, Component, ComponentNetIo, Harness, IdleDrive, JumperState, Level,
    NetState, PinDecl, PinKind, Scenario, System,
};
use embsim_boards::ec32mb::{Ec32mb, FLASH_SELECT_POLE, FLASH_SELECT_SWITCH, P59_PULL_DOWN_POLE};
use embsim_boards::p2::P2Package;
use embsim_core::virtual_clock;
use embsim_p2_qemu::{flashimage, P2Qemu, P2QemuError};

/// The P2's debug transmit pin, where the payload writes its byte.
const DEBUG_TX: u8 = 62;

/// The ROM runs on RCFAST — it never sets the PLL — so two clocks per
/// instruction at 20 MHz: the closest two edges the ROM's SPI loop produces
/// (`drvh`/`drvl` back to back) are one instruction, 100 ns, apart.
const ROM_INSTRUCTION_NS: u64 = 100;

/// A scope on one net: the virtual instant of every level change it sees.
///
/// This is the interface's own claim under test. A CPU bit-banging a bus
/// must put each edge on the net at the instant it happened, not at the end
/// of a slice — collapsed edges are what a device cannot count, and edges
/// stamped early or late are what an interval measurement gets wrong. The
/// flash's clock is the densest thing on the board: two ROM instructions
/// apart, 12.5 ns at 160 MHz.
struct Scope {
    pins: [PinDecl; 1],
    instants: Arc<Mutex<Vec<u64>>>,
}

impl Scope {
    fn new() -> (Self, Arc<Mutex<Vec<u64>>>) {
        let instants = Arc::new(Mutex::new(Vec::new()));
        let scope = Self {
            pins: [PinDecl {
                number: "A",
                name: None,
                kind: PinKind::DigitalIn,
                stream: None,
                drive_impedance: None,
                idle: IdleDrive::KindDefault,
            }],
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
        io.on_sense("A", move |state| {
            let Some(level) = level_of(state) else {
                return;
            };
            let mut last = last.lock().unwrap();
            if *last != Some(level) {
                *last = Some(level);
                instants.lock().unwrap().push(virtual_clock::virtual_ns());
            }
        })
    }
}

fn rom(name: &str) -> Vec<u8> {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("rom")
        .join(name);
    std::fs::read(&path).unwrap_or_else(|e| panic!("{}: {e}", path.display()))
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
fn the_rom_boots_off_the_modules_flash_over_the_nets() {
    // mov pa,#"B" / wypin pa,#62 / jmp #$
    let payload: Vec<u8> = [0xF607EC42u32, 0xFC27EC3E, 0xFD9FFFFC]
        .iter()
        .flat_map(|w| w.to_le_bytes())
        .collect();
    let image = flashimage::boot_flash(&rom("stage1.bin"), &payload).expect("flash image");

    // `EMBSIM_P2_QEMU_TRACE=<file>` writes QEMU's per-instruction state trace
    // there, the same `-d cpu` log `p2core/tools/romtest.sh` diffs against
    // p2core's, so a divergence can be placed at the instruction. The trace
    // is unbounded and ~450 bytes an instruction: a payload that spins and
    // never reaches the byte fills a disk in minutes. The wait below is cut
    // short when tracing for exactly that reason.
    let trace = std::env::var("EMBSIM_P2_QEMU_TRACE").ok();
    let extra: Vec<&str> = match trace.as_deref() {
        Some(path) => vec!["-d", "cpu", "-D", path],
        None => Vec::new(),
    };
    let p2 = match P2Qemu::with_boot_rom(&rom("rom_booter_v33k.bin"), &extra) {
        Ok(p2) => p2,
        Err(P2QemuError::Unavailable) => {
            eprintln!("\n*** SKIPPED: built without a QEMU tree (EMBSIM_QEMU_P2_BUILD). Asserted NOTHING.\n");
            return;
        }
        Err(e) => panic!("{e}"),
    };
    let handle = p2.handle();
    // The core goes inside the package: the package is what the board sees.
    let package = P2Package::new(p2);
    let package_handle = package.handle();

    virtual_clock::init(0.0, 160_000_000);

    let board = Ec32mb::new().with_flash_image(image);
    let flash = board.flash_view().expect("a programmed flash has a view");
    // The slot is filled once; a netlist with two U100s would be a different
    // board, and QEMU is one machine per process anyway.
    let slot = std::sync::Mutex::new(Some(package));
    let board = board
        .with_p2(move |_decl| -> Box<dyn Component> {
            Box::new(slot.lock().unwrap().take().expect("one P2 per board"))
        })
        .build()
        .expect("the module builds");

    let (scope, clock_instants) = Scope::new();
    let started = Instant::now();
    let system = System::new()
        .board("EC32", board)
        .component("SCOPE", Box::new(scope))
        .harness(
            Harness::new()
                .connect_str("EC32.U100.P60", "SCOPE.A")
                .expect("the flash clock net exists"),
        )
        .scenario(
            Scenario::default()
                // S301 position 2 (FLASH) closed: P61 reaches the flash's
                // ~CS. A closed pole is a build-time identity union of its
                // two nets — the same merge a `pin_short` makes, said as
                // the switch position it is.
                .switch(
                    &format!("EC32.{FLASH_SELECT_SWITCH}"),
                    FLASH_SELECT_POLE,
                    JumperState::Closed,
                )
                // S301 position 4 closed: R303 pulls P59 down, so the ROM
                // boots the program it loaded instead of waiting for a
                // serial loader.
                .switch(
                    &format!("EC32.{FLASH_SELECT_SWITCH}"),
                    P59_PULL_DOWN_POLE,
                    JumperState::Closed,
                )
                // The bench supplies the rails: ground is 0 V and the I/O
                // rail is up, so R301 pulls the select high (the boot strap)
                // and R303 pulls P59 down. Neither is implicit — the engine
                // has no idea of ground, and without this GND is just another
                // node the pull-ups reach, which reads high.
                .net_stuck("EC32.GND", 0.0)
                .net_stuck("EC32.VIO_56_63", 3.3),
        )
        .start()
        .expect("the system starts");

    // A chip whose every cog has stopped will not print later.
    let patience = if trace.is_some() { 20 } else { 180 };
    wait_for(
        || handle.console(DEBUG_TX).contains('B') || handle.halted(),
        Duration::from_secs(patience),
    );
    let wall = started.elapsed();
    if trace.is_some() {
        // The p2core differential compares 60 000 states and the byte
        // arrives after ~44 000; let the payload spin a little longer so
        // the trace carries them (bounded: a few seconds of traced spin).
        std::thread::sleep(Duration::from_secs(3));
    }
    let nets: Vec<String> = [
        "EC32.P2_IO58",
        "EC32.P2_IO59",
        "EC32.P2_IO60",
        "EC32.P2_IO61",
        "EC32.SPI_CS",
        "EC32.Net-(S301-4_OFF)",
        "EC32.GND",
        "EC32.VIO_56_63",
    ]
    .iter()
    .map(|n| format!("{n}={:?}", system.net_state(n)))
    .collect();
    assert!(
        handle.console(DEBUG_TX).contains('B'),
        "the payload the flash served must reach the debug pin; console={:?} yields={} \
         publishes={} slices={} reads={:?} commands={:?} halted={} nets={nets:?}",
        handle.console(DEBUG_TX),
        handle.yields(),
        handle.publishes(),
        handle.slices(),
        flash.reads(),
        flash.commands(),
        handle.halted(),
    );

    // WHERE it looked, which is a sharper claim than that it finished: the
    // ROM takes stage-1 from 0, stage-1 takes the application from $400.
    let reads = flash.reads();
    assert!(
        reads.starts_with(&[0, 1024]),
        "the boot reads flash at 0 then $400; got {reads:?}"
    );
    // Every clock edge crossed the net as its own event.
    assert!(
        handle.yields() > 10_000,
        "a kilobyte off the flash is thousands of edges, each a yield; saw {}",
        handle.yields()
    );

    // And each at the guest's own instant. The scope saw every clock edge
    // (a kilobyte is 8 192 data clocks plus the command's), never two at one
    // instant, never a pair closer than the one instruction the ROM puts
    // between them, and typically within a few — not a slice apart, and not
    // the 1 ns apart a fallback "strictly forward" re-arm would leave.
    let instants = clock_instants.lock().unwrap().clone();
    let mut gaps: Vec<u64> = instants.windows(2).map(|w| w[1] - w[0]).collect();
    gaps.sort_unstable();
    let ones = gaps.iter().filter(|&&g| g <= 1).count();
    let median = gaps.get(gaps.len() / 2).copied().unwrap_or(0);
    eprintln!(
        "scope: {} edges, first instants {:?}, min gap {:?}, median gap {median}, yields={} \
         publishes={}",
        instants.len(),
        &instants[..instants.len().min(24)],
        gaps.first(),
        handle.yields(),
        handle.publishes(),
    );
    assert!(
        instants.len() > 16_000,
        "the scope must see every clock edge; saw {}",
        instants.len()
    );
    assert_eq!(
        ones,
        0,
        "no two edges may share an instant or sit 1 ns apart; {ones} did (min gap {:?})",
        gaps.first()
    );
    assert!(
        gaps.first().is_some_and(|&g| g >= ROM_INSTRUCTION_NS),
        "the closest edges are one ROM instruction apart ({ROM_INSTRUCTION_NS} ns); min gap {:?}",
        gaps.first()
    );
    assert!(
        (ROM_INSTRUCTION_NS..=4 * ROM_INSTRUCTION_NS).contains(&median),
        "the typical gap is the ROM's own clock loop, not a slice; median {median} ns"
    );

    // The boot's cost, as the baseline `NODES.md` §8 phase 0 records and
    // every later phase is measured against: edges the flash clock carried,
    // yields and publishes the P2 made, wall time from `start` to the byte —
    // and how many of those edges escalated a cluster to the solver. On this
    // board every SPI edge is a 25 Ω pad against a 10.5 kΩ pull-up to a
    // terminal, so nothing disagrees within a factor of ten and no analog
    // sense asks: the whole boot is projections (`DESIGN.md` rule 8), and
    // the count is held at 0 as the budget it is. A phase that changes it
    // says so.
    let escalated = system.escalated_solves();
    eprintln!(
        "baseline: edges={} yields={} publishes={} escalated_solves={escalated} wall={:.3}s ({:.2} us/edge)",
        instants.len(),
        handle.yields(),
        handle.publishes(),
        wall.as_secs_f64(),
        wall.as_secs_f64() * 1e6 / instants.len() as f64,
    );
    assert_eq!(
        escalated, 0,
        "the ROM boot is projections only: no cluster escalated to the solver"
    );

    // The crystal is the rate the board delivers on XI, and on this bench
    // none arrives: the module's TCXO runs from `Common_VDD`, the 1.8 V
    // core rail, and `U402`, the buck that sources it, is a pin facade
    // until the rails land (`NODES.md` §8 phase 4) — so the TCXO sees its
    // supply pulled low through the feedback divider by the stuck ground
    // and never starts. Holding `Common_VDD` from the bench instead was
    // measured and refused: a second real terminal across that divider
    // inside the ground cluster re-solves it on every P59 edge (30
    // escalated solves against the 0 this boot is held to). The ROM runs
    // on RCFAST and never selects the crystal, so the boot is the same
    // either way; `crystal_pll.rs` proves the XI → HUBSET → PLL path on a
    // bench where the clock reaches the pin. What is asserted here is the
    // CAUSE — the core rail as the divider pulls it — and beside it the
    // consequence, that the package told the core the truth: nothing on
    // XI. TODO(phase 4): when `U402` is a rail the cause assertion fails
    // first; move the crystal assertion onto the module's own TCXO,
    // `Some(20_000_000)` on both handles, and delete the cause.
    let clock_nets: Vec<String> = [
        "EC32.Common_VDD",
        "EC32.Net-(X100-OUT)",
        "EC32.Net-(U101-2A)",
        "EC32.XTAL_XI",
    ]
    .iter()
    .map(|n| format!("{n}={:?}", system.net_state(n)))
    .collect();
    assert_eq!(
        system.net_state("EC32.Common_VDD"),
        Some(NetState::Pulled(Level::Low, 23_800.0)),
        "the TCXO's rail reads the stuck ground through U402's feedback divider (R401 13.3 kΩ + \
         R403 10.5 kΩ = 23.8 kΩ) while the buck is a facade — the cause of the silent XI; clock \
         chain={clock_nets:?}"
    );
    assert_eq!(
        handle.crystal_hz(),
        None,
        "no rate reaches XI while the TCXO's rail is a facade; clock chain={clock_nets:?}"
    );
    assert_eq!(package_handle.crystal_hz(), None);
    assert!(!handle.stalled(), "the boot runs on RCFAST");
    drop(system);
}
