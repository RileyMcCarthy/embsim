//! The chip boots off the module's own flash, as silicon does, with every
//! part a node on the board's real nets.
//!
//! The P2 is QEMU's target; the flash is `embsim_models`' generic part; the
//! board is the P2-EC32MB from its vendor netlist, powered the way a
//! carrier powers it — 5 V and 0 V on its `J203` fingers, nothing stuck.
//! Nothing in the P2 knows there is a flash — it bit-bangs four pins the
//! ROM chose, and a device on those nets answers. Two facts of the board
//! have to be true for that to happen, both said as scenario, not wiring:
//! DIP switch `S301` position 2 (labelled FLASH) is closed, joining `P61`
//! to the flash's `~CS`, so that `R301` pulls the select high from the
//! `VIO_56_63` rail once the module's LDO raises it — which is the strap
//! the ROM samples to decide the flash is worth trying; and `S301`
//! position 4 (the P59 pull-down, `R303`) is closed, which is the module's
//! "boot from flash without waiting for a serial loader" setting. The ROM
//! really samples that last one: after a valid load it drives P59 high,
//! floats it, waits, and reads it back — low means boot now, high means try
//! serial first. With the switch open the node reads the level P59 was left
//! at, high, and the ROM goes looking for a loader instead. That was the
//! first divergence from p2core this test found, and it was the board's.
//!
//! The P2 is the QEMU core inside the P2 **package** (`embsim_boards::p2`):
//! the package declares the 86 pins the netlist gives `U100`, holds the
//! core until the chip can run — its **START gate**: the datasheet's 3 ms
//! restart delay after `RESN` reads released with `VDD` inside its window,
//! which on this module happens the instant the bucks' 2.5 ms soft-start
//! elapses and `U402` raises the core rail past the detector's threshold,
//! so the core starts at 5.5 ms — and the crystal the core's PLL
//! would multiply is the rate the board delivers on `XI`: the module's
//! TCXO, up from the same rail, through its buffer, not a number handed to
//! the node. The boot itself runs on RCFAST and never selects it.
//!
//! What is asserted is what the boot DID: the flash served reads at `0` and
//! `$400` (stage-1, then the application), and the application's one byte
//! reached the debug pin. And that every edge was the P2's own: the flash saw
//! real clock edges over the net, delivered one instant at a time, none
//! before the START instant.

use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use embsim_board::{
    jesd8c01_lvcmos_thresholds, AttachError, Component, ComponentNetIo, DeadBand, DigitalReceiver,
    EndpointRef, Harness, JumperState, Level, NetState, PinDecl, Scenario, System,
};
use embsim_boards::ec32mb::{Ec32mb, FLASH_SELECT_POLE, FLASH_SELECT_SWITCH, P59_PULL_DOWN_POLE};
use embsim_boards::p2::{P2Package, P2_RESTART_DELAY_NS};
use embsim_core::virtual_clock;
use embsim_p2_qemu::{flashimage, P2Qemu, P2QemuError};

/// The P2's debug transmit pin, where the payload writes its byte.
const DEBUG_TX: u8 = 62;

/// The ROM runs on RCFAST — it never sets the PLL — so two clocks per
/// instruction at 20 MHz: the closest two edges the ROM's SPI loop produces
/// (`drvh`/`drvl` back to back) are one instruction, 100 ns, apart.
const ROM_INSTRUCTION_NS: u64 = 100;

/// The instant the chip's reset releases on this module: the AP62301
/// bucks' soft-start, 2.5 ms after the carrier's 5 V arrives (Diodes
/// DS41958 Rev. 4-2, `t_SS`; `embsim_models::rail::AP62301_SOFT_START_NS`).
/// `U402` steps the core rail to 1.813 V there — inside the P2's 1.7–1.9 V
/// window — the LDOs step the bank rails with it, and the STM1061 releases
/// `RESN` at the same instant (its supply steps from nothing past its
/// release threshold, no crossing to delay).
const RESET_RELEASE_NS: u64 = 2_500_000;

/// The START instant: the datasheet's restart delay after the reset
/// releases ("Propeller restarts 3 ms after RESn transitions from low to
/// high", P2X8C4M64P Datasheet, Pin Descriptions, p. 6) — 5.5 ms.
const START_NS: u64 = RESET_RELEASE_NS + P2_RESTART_DELAY_NS;

/// `U402`'s setpoint from its divider, `R401` 13.3 kΩ over `R403` 10.5 kΩ
/// at `V_FB` = 0.800 V (DS41958 Eq. 8): 1.8133 V — the P2's `VDD`.
const CORE_RAIL_VOLTS: f64 = 0.8 * (1.0 + 13.3 / 10.5);

/// The module's TCXO, `X100` (Epson TG2520SMN 20.0000M), up from the core
/// rail: the rate the package reads on `XI`.
const TCXO_HZ: u64 = 20_000_000;

/// Cluster solves the run escalates to the solver, accounted for, all
/// before the first edge: the polarity FET `U401`'s element cluster
/// (`VIN_Edge`/`VIN_Edge_Protected`, the 5 V finger sourcing through it)
/// once, in the start pass at t = 0, and the two bucks' feedback dividers
/// once each when their rails rise at [`RESET_RELEASE_NS`]: two comparable sources
/// (the rail through `R_top`, ground through `R_bot`) that rule 2 solves.
/// The QEMU core's 64 pad-read declarations resolve only the pads'
/// clusters (`Resolver::declare_reads`, the sense task, `NODES.md` §12
/// item 5): until then each drain batch of them was a full pass that
/// re-solved the FET cluster, two batches for five in all, three — a sixth
/// solve — when the attaching thread's commands split three ways. None per
/// edge: the boot's 16 901 edges are projections, as they were with the
/// rails stuck from the bench (0 then, since nothing sourced the FET).
const ESCALATED_SOLVES: u64 = 3;

fn ep(endpoint: &str) -> EndpointRef {
    EndpointRef::parse(endpoint).expect("endpoint parses")
}

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
                .expect("the flash clock net exists")
                // The module powered the way a carrier powers it: 5 V into
                // the two `5V` fingers and 0 V into the three `GND` fingers
                // of `J203` (`p2_ec32mb.net`: fingers 41/42 `5V`, 43/44/45
                // `GND`). Everything else is the board's own power tree —
                // the polarity FET, the two bucks, the eight LDOs, the
                // detector — so the ground the pull-downs return to, the
                // bank rail `R301` pulls the boot strap to, the core rail
                // the START gate reads and the TCXO's supply are the parts'
                // outputs, nothing stuck. Ground is not implicit: the bench
                // return is a declared harness terminal.
                .power(ep("CARRIER.5V"), ep("EC32.J203.41"), 5.0)
                .power(ep("CARRIER.5Vb"), ep("EC32.J203.42"), 5.0)
                .power(ep("CARRIER.GND"), ep("EC32.J203.43"), 0.0)
                .power(ep("CARRIER.GNDb"), ep("EC32.J203.44"), 0.0)
                .power(ep("CARRIER.GNDc"), ep("EC32.J203.45"), 0.0),
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
                ),
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
        "EC32.Common_VDD",
        "EC32.P2_RESN",
    ]
    .iter()
    .map(|n| format!("{n}={:?}", system.net_state(n)))
    .collect();
    assert!(
        handle.console(DEBUG_TX).contains('B'),
        "the payload the flash served must reach the debug pin; console={:?} yields={} \
         publishes={} slices={} reads={:?} commands={:?} halted={} start={:?} nets={nets:?}",
        handle.console(DEBUG_TX),
        handle.yields(),
        handle.publishes(),
        handle.slices(),
        flash.reads(),
        flash.commands(),
        handle.halted(),
        package_handle.start_state(),
    );

    // The START gate: the core ran from the datasheet's restart delay
    // after the instant the module released its reset — the bucks'
    // soft-start elapsed, the core rail inside the P2's window, the
    // detector's `RESN` released — and not one edge before.
    assert_eq!(START_NS, 5_500_000);
    assert_eq!(
        package_handle.started_at_ns(),
        Some(START_NS),
        "the package starts the core 3 ms after the bucks' soft-start instant; nets={nets:?}"
    );
    assert!(
        package_handle.reset().out_of_reset(),
        "RESN released and VDD inside its window: {:?}",
        package_handle.reset()
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
    assert!(
        instants.first().is_some_and(|&first| first >= START_NS),
        "no edge before the START instant {START_NS}; first at {:?}",
        instants.first()
    );
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
    // yields and publishes the P2 made, wall time from `start` to the byte,
    // the START instant — and how many cluster solves the run escalated to
    // the solver. On this board every SPI edge is a fast pad (17.99 Ω, the
    // datasheet fit `P2_FAST_OHMS`) against a 10.5 kΩ pull-up to a
    // terminal, so nothing disagrees within a factor
    // of ten and no analog sense asks: the whole boot is projections
    // (`DESIGN.md` rule 8). The three solves the count carries are the
    // power tree's before the first edge (`ESCALATED_SOLVES`) — none per
    // edge; the count is held exactly, as the budget it is. A phase that
    // changes it says so.
    let escalated = system.escalated_solves();
    eprintln!(
        "baseline: edges={} yields={} publishes={} escalated_solves={escalated} start_ns={:?} \
         wall={:.3}s ({:.2} us/edge)",
        instants.len(),
        handle.yields(),
        handle.publishes(),
        package_handle.started_at_ns(),
        wall.as_secs_f64(),
        wall.as_secs_f64() * 1e6 / instants.len() as f64,
    );
    assert_eq!(
        escalated, ESCALATED_SOLVES,
        "the ROM boot is projections only: the power tree's solves before the first edge and \
         none per edge"
    );

    // The crystal is the rate the board delivers on XI, and on this module
    // it is the TCXO's: `X100` runs from `Common_VDD`, the core rail
    // `U402` raises at the reset release, and its 20 MHz reaches `XI`
    // through the buffer and the coupling capacitor once its own start-up
    // elapses. The ROM runs on RCFAST and never selects the crystal, so
    // the boot is the same either way; `crystal_pll.rs` proves the XI →
    // HUBSET → PLL path. What is asserted here is the package telling the
    // core the truth about the board: the core rail at the divider's
    // setpoint, and 20 MHz on XI, on both handles.
    let clock_nets: Vec<String> = [
        "EC32.Common_VDD",
        "EC32.Net-(X100-OUT)",
        "EC32.Net-(U101-2A)",
        "EC32.XTAL_XI",
    ]
    .iter()
    .map(|n| format!("{n}={:?}", system.net_state(n)))
    .collect();
    match system.net_state("EC32.Common_VDD") {
        Some(NetState::Analog(v)) if (v - CORE_RAIL_VOLTS).abs() < 1e-3 => {}
        other => panic!(
            "the core rail is U402's terminal at its divider's 1.813 V: {other:?}; clock \
             chain={clock_nets:?}"
        ),
    }
    assert_eq!(
        handle.crystal_hz(),
        Some(TCXO_HZ),
        "the module's TCXO reaches XI; clock chain={clock_nets:?}"
    );
    assert_eq!(package_handle.crystal_hz(), Some(TCXO_HZ));
    assert!(!handle.stalled(), "the boot runs on RCFAST");
    // Every pad the ROM drove sat in a bank the module's LDOs supply.
    assert_eq!(
        package_handle.unpowered_banks_driven(),
        Vec::<usize>::new(),
        "no pad was driven in a bank without its supply"
    );
    drop(system);
}
