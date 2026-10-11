// Shared across three integration binaries (`ec32mb_module`, `edgeboard`,
// `machine_system`), each of which uses a different subset — Cargo compiles a
// private copy of this module into every one of them, so anything the *other*
// binaries use is dead code here.
#![allow(dead_code)]

//! Part library and board/harness builders for the netlist-grounded machine
//! tests: the P2-EC32MB module, the MaD EdgeBoard, and the harnesses that
//! plug them into each other and into the machine.
//!
//! `BOARD_ENGINE.md` puts consumer-side specifics — "part registry entries,
//! harness files, plant models" — in the *consuming* repo. This module is
//! their transitional home: the reference consumer's parts, expressed against
//! the two committed netlist fixtures, so the engine has something real to be
//! tested against before MaD grows its own registry.
//!
//! # Netlist sources
//!
//! | Fixture | Board | Notes |
//! |---|---|---|
//! | `embsim_boards::ec32mb::NETLIST` | Parallax P2-EC32MB Rev B module | transcribed from the vendor PDF; **no `libsource`** |
//! | `fixtures/mad_edge.net` | MaD EdgeBoard (3 sheets) | `kicad-cli sch export netlist` |
//! | `fixtures/ds2_addon.net` | MaD DS2 force-gauge add-on | `kicad-cli sch export netlist` |
//!
//! # Every part a node
//!
//! `DESIGN.md` rule 1 admits no stub tier, and since `NODES.md` §8 phase 4
//! none is needed here: every registered part on the three boards is a
//! model or an element by specification, and a part with nothing behind it
//! is a build error naming it. The engine takes electrical descriptors
//! from the component facade and never from the schematic (see the
//! `netlist` module docs), which is what lets a model *correct* a symbol:
//! the EdgeBoard's `XL1509` symbol draws all eight pins as `input`,
//! including VIN, OUT and the four grounds, and the rail model declares
//! what the part is.
//!
//! # Modeled parts (real behavior)
//!
//! Every part with behavior comes from `embsim-models`: the AM26LS31 line
//! driver `U24`, the servo step/direction pairs
//! (`embsim_models::am26ls31`); the AM26LV32 line receiver `U25`, the
//! encoder A/B/ZI pairs (`embsim_models::am26lv32`); every ISO67xx on the
//! Edge board, including `ISO6731DWR` (IC5, the force-gauge isolator), from
//! `embsim_models::isolation::iso67xx` (`IC1`, `IC2`, `IC5`, `IC14` with
//! its STEP channel carrying a rate, `IC15`, `IC16`); the 21 SN74LVC1G14
//! LED drivers and the five optocouplers (`U4` a 6N137, `U5`–`U8` VO2631s)
//! on the Edge board; the TCXO, the two 74LVC2G04 inverters, the four
//! PSRAMs and the boot flash `U301` (blank, the 16 MiB part `embsim-boards`
//! ships the module with; `w25q128jv.rs` re-registers it with each case's
//! own image) on the module. The
//! nonlinear parts are elements registered by specification from
//! `embsim_models::pwl_library`, keyed on the manufacturer part number the
//! netlists carry: on the Edge board the polarity FET `U3` with its body
//! diode, the transistor `Q1`, the eight current regulators `IC6`–`IC13`,
//! the Schottky diodes `D1`/`D2` and the 21 indicator LEDs; on the module
//! the polarity FET `U401` and the white LEDs `D601`/`D602`. The power
//! parts are rails (`embsim_models::rail`) and a detector
//! (`embsim_models::supervisor`): on the module the bucks `U402`/`U403`
//! (one registry key, each reading its own feedback divider at attach),
//! the LDOs `U501`–`U508` and the detector `U404`; on the Edge board the
//! bucks `U1`/`U2` (their version from the value) and the isolated DC/DCs
//! `IC3`/`IC4` (their setpoint from the `SEL` strap). Everything else is
//! an auto-classified primitive.

use std::sync::{Mutex, MutexGuard};

use embsim_board::registry::normalize_part;
use embsim_board::{
    Board, ComponentDecl, EndpointRef, Harness, JumperState, PartRegistry, Scenario, SwitchPole,
    Volts,
};
use embsim_boards::ec32mb::{FLASH_CAPACITY, FLASH_PART};
use embsim_boards::p2::P2Package;
use embsim_models::am26ls31::Am26ls31;
use embsim_models::am26lv32::Am26lv32;
use embsim_models::isolation::{iso67xx, Iso67xx};
use embsim_models::logic_gate::{self, LogicGate, LVC1G14_PINS_SOT23, LVC2G04_PINS_BY_FUNCTION};
use embsim_models::opto::Opto;
use embsim_models::oscillator::{self, Oscillator};
use embsim_models::psram::{Psram, PsramComponent};
use embsim_models::pwl_library;
use embsim_models::rail::{
    self, Rail, AP62301_PINS_BY_FUNCTION, NCP114_PINS_BY_FUNCTION, UCC12040_PINS_SOIC16,
    XL1509_PINS_SOP8,
};
use embsim_models::spi_flash::SpiNorFlash;
use embsim_models::spi_flash_component::{SpiNorFlashComponent, SPI_FLASH_PINS_BY_FUNCTION};
use embsim_models::supervisor::{self, VoltageDetector, STM1061_PINS_BY_FUNCTION};

// ============================================================
// P2-EC32MB module: the P2 behind its full package facade
// ============================================================

/// The P2 package with no core: every pad released. Tests that fill the
/// module's processor slot without running a CPU use this.
pub fn p2_edge_module(_name: &str) -> P2Package<embsim_boards::p2::HeldInReset> {
    P2Package::held_in_reset()
}

/// One system at a time inside a test binary. Tests that build the module
/// take this lock (poison-recovering, like the paced-stream suites).
pub fn lock_module_instance() -> MutexGuard<'static, ()> {
    static LOCK: Mutex<()> = Mutex::new(());
    LOCK.lock().unwrap_or_else(|poisoned| {
        LOCK.clear_poison();
        poisoned.into_inner()
    })
}

// ============================================================
// P2-EC32MB module: classification
// ============================================================
//
// Classification, part by part. The module's netlist carries no `libsource`,
// so the registry runs with `classify_unnamed_by_reference(true)` and keys the
// other parts on their `value` field:
//
//   auto (reference prefix)  85 passives (C×66, R×15, L×2, D×2 LEDs) and the
//                            two J-prefixed pad/socket symbols the fallback
//                            can name — J203 (the 80-finger card edge) and
//                            J301 (microSD socket) — as boundaries.
//   real model               U100, the P2 (see `P2EdgeModule`); U301, the
//                            boot flash (`embsim_models::spi_flash`, blank);
//                            X100, the TCXO (`embsim_models::oscillator`);
//                            U101 and U601, the dual inverters
//                            (`embsim_models::logic_gate`); U302–U305, the
//                            PSRAMs (`embsim_models::psram`); U402/U403,
//                            the bucks, and U501–U508, the LDOs
//                            (`embsim_models::rail`); U404, the brownout
//                            detector (`embsim_models::supervisor`).
//   switch (by value)        S301, the four-way DIP switch, four poles by
//                            `<position>_ON`/`<position>_OFF`; J101, the
//                            oscillator-option solder link, one pole. Every
//                            pole open by default; a scenario closes one.
//   mechanical (by value)    J701/J702 (mounting holes, tied to GND), PCB
//                            (the raw board, no nodes) and NC_Net (a layout
//                            node): pads and nothing electrical.
//   element (by MPN)         U401, the Si3417DV polarity FET: its channel
//                            and body diode as piecewise-linear branches;
//                            D601/D602, the IN-S63AS5UW white LEDs — from
//                            `embsim_models::pwl_library`, keyed on the
//                            `MPN` field the transcription carries.
//
// The power tree is real, and it has a clock: from the carrier's 5 V on
// `J203` the bucks rise 2.5 ms later (the AP62301's soft-start) and the
// LDOs step with them, so a build snapshot — the state before the first
// wake — has every module rail down and says so (`Finding::RailDown`).
// `board/tests/power_tree.rs` starts the module and steps past it.

/// `DIP Switch 4 way` — the module's option switch (S301): four poles, one
/// per printed position *n*, between the netlist's `<n>_ON` and `<n>_OFF`
/// pin ids (the pairing the pin-function labels state: `"FLASH (ON side)"` /
/// `"FLASH (OFF side)"` on position 2). All open by default, as shipped;
/// position *n* is pole `n - 1` to `Scenario::switch`. Closing pole 1
/// (FLASH) ties `P2_IO61` to the flash `~CS`; pole 2 is the P59 pull-up
/// (`R302`), pole 3 the P59 pull-down (`R303`).
pub fn dip_switch_poles() -> Vec<SwitchPole> {
    (1..=4)
        .map(|n| SwitchPole::open(format!("{n}_ON"), format!("{n}_OFF")))
        .collect()
}

/// `Solder Link Pads` — J101, the oscillator-option link between `P2_IO32`
/// and the TCXO's `NC/GND` option pad: one pole, open as shipped.
pub fn solder_link_poles() -> Vec<SwitchPole> {
    vec![SwitchPole::open("1", "2")]
}

/// Part registry for the P2-EC32MB netlist.
pub fn ec32mb_registry() -> PartRegistry {
    let mut registry = PartRegistry::new();
    // The netlist was transcribed from the vendor PDF and has no libsource, so
    // the auto tier keys on reference-designator prefixes and the registry on
    // the `value` field.
    registry.classify_unnamed_by_reference(true);

    registry.register("P2X8C4M64P", |_decl| Box::new(p2_edge_module("p2")));
    registry.register_switch("DIP Switch 4 way", dip_switch_poles());
    registry.register_switch("Solder Link Pads", solder_link_poles());
    registry.register_mechanical("Mounting Hole Vss");
    registry.register_mechanical("PCB for P2 EC Module");
    registry.register_mechanical("Layout node");
    // The TCXO at the frequency its value names, the two dual inverters,
    // the four PSRAMs — the same models `embsim-boards` ships the module
    // with.
    registry.register("TG2520SMN 20.0000M-ECGNNM3", |decl: &ComponentDecl| {
        let config = oscillator::Config::from_value(&decl.value)
            .unwrap_or_else(|| panic!("{}: the TCXO value names no frequency", decl.reference));
        Box::new(Oscillator::new(config))
    });
    registry.register("74LVC2G04GW,125", |_decl| {
        Box::new(
            LogicGate::new(logic_gate::Config::lvc2g04(), &LVC2G04_PINS_BY_FUNCTION)
                .expect("the datasheet configuration is valid"),
        )
    });
    registry.register("PSRAM 64Mbit", |_decl| {
        Box::new(PsramComponent::new(Psram::new()))
    });
    // The boot flash, live and blank — the same part `embsim-boards`
    // registers, so the two registries agree about `U301`; a test that
    // wants an image re-registers the key with its own (`w25q128jv.rs`).
    registry.register(FLASH_PART, |_decl| {
        Box::new(
            SpiNorFlashComponent::new(SpiNorFlash::blank(FLASH_CAPACITY))
                .with_pins(&SPI_FLASH_PINS_BY_FUNCTION),
        )
    });
    // The elements by specification: `U401` (a Si3417DV by its `MPN`
    // field) and the white LEDs `D601`/`D602`, from the element library —
    // as `embsim-boards` registers them.
    pwl_library::register(&mut registry);
    // The power tree — the same models `embsim-boards` registers: two
    // bucks from one key, each reading its own feedback divider at attach;
    // eight LDOs at the voltage their value names; the detector.
    registry.register("DCDC 3A SOT563", |_decl| {
        Box::new(
            Rail::new(rail::Config::ap62301(), &AP62301_PINS_BY_FUNCTION)
                .expect("the AP62301 table carries every role"),
        )
    });
    registry.register("LDO 300mA, 3.3V", |decl: &ComponentDecl| {
        let config = rail::Config::ncp114_from_value(&decl.value)
            .unwrap_or_else(|| panic!("{}: the LDO value names no voltage", decl.reference));
        Box::new(
            Rail::new(config, &NCP114_PINS_BY_FUNCTION)
                .expect("the NCP114 table carries every role"),
        )
    });
    registry.register("Voltage Detector 1.6V", |_decl| {
        Box::new(VoltageDetector::new(
            supervisor::Config::stm1061n16(),
            &STM1061_PINS_BY_FUNCTION,
        ))
    });
    registry
}

/// Build the P2-EC32MB module as a [`Board`].
pub fn ec32mb_board() -> Board {
    let parsed = embsim_board::netlist::parse(embsim_boards::ec32mb::NETLIST)
        .expect("the EC32MB netlist parses");
    Board::from_netlist(parsed, &ec32mb_registry()).expect("the EC32MB module builds")
}

/// The P2-EC32MB as `embsim-boards` ships it — its own registry, a blank
/// boot flash, an empty card socket — with the processor slot filled by a
/// P2 package held in reset ([`P2Package::held_in_reset`]): every pad
/// released, the rails and `RESN` sensed, `XI` accepting the rate, and no
/// core. It touches no process-global peripheral bank, unlike
/// [`P2EdgeModule`], so a test that wants the module as shipped and nothing
/// running in it can use this without the module-instance lock.
pub fn shipped_ec32mb_board() -> Board {
    embsim_boards::ec32mb::Ec32mb::new()
        .with_p2(|_decl| Box::new(P2Package::held_in_reset()))
        .build()
        .expect("the module builds")
}

// ============================================================
// MaD EdgeBoard: classification
// ============================================================
//
// Classification, part by part (this netlist is a real KiCad export, so every
// tier keys on the libsource part name as usual):
//
//   auto        R_Small ×32, C_Small ×29, LED ×21, D_Schottky_Small ×2 (SS36),
//               L_Small ×2 — the 86 passives; Conn_01x0n / Screw_Terminal ×23
//               plus the declared P2_EDGE_MODULE_SOCKET — the 24 boundaries;
//               Jumper_2_Open ×4 + Jumper_3_Open ×1 — the stateful shorts;
//               SW_Push ×1 (SW1, the reset button) — a one-pole switch, open;
//               MountingHole_Pad ×4 — mechanical nodes.
//               (86 + 24 + 5 + 1 + 4 + 48 registered = the netlist's 168.)
//   real model  AM26LS31CD (U24, `embsim_models::am26ls31`) and AM26LV32xD
//               (U25, `embsim_models::am26lv32`), the encoder/servo RS-422
//               pair; ISO6731DWR (IC5, the force-gauge isolator),
//               ISO6742DWR (IC1, IC2), ISO6741DWR (IC14), ISO6721BDR (IC15)
//               and ISO6740FDWR (IC16), each an
//               `embsim_models::isolation::iso67xx` configured from its part
//               name; SN74LVC1G14DBV
//               ×21 (U9–U34), the Schmitt inverters driving the front-panel
//               LEDs (`embsim_models::logic_gate`); 6N137 (U4) and VO2631
//               ×4 (U5–U8), the optocouplers (`embsim_models::opto`);
//               XL1509 (U1 5 V, U2 3.3 V), the bucks behind the polarity
//               FET, and UCC12040DVER (IC3, IC4), the isolated DC/DCs
//               (`embsim_models::rail`).
//   element     the elements by specification (`embsim_models::pwl_library`).

/// `UCC12040DVER` — TI isolated 500 mW DC/DC module (IC3: the isolated I/O
/// domain `5V_IO`/`GND_IO`; IC4: the force-gauge domain `IFG_5V`/`IFG_GND`).
/// Both tie `SEL` to `VISO`: the 5.0 V setpoint (SNVSBO5B Table 5-1). The
/// rail model declares `VISO` **and** its `GNDS` return as terminals — an
/// isolated DC/DC generates a whole domain, its ground included — and the
/// isolated ground is held by whatever the board or the harness ties it
/// to: nothing on the Edge board ties either, so on the board alone both
/// isolated rails stay down with their reference named
/// (`Finding::RailDown`), and a harness that declares the domain's return
/// ([`force_domain_rails`] does for the force gauge's, over the cable) is
/// what lets the rail come up.
pub const UCC12040_PART: &str = "UCC12040DVER";

/// `XL1509` — 2 A step-down converter (U1: +5 V, U2: +3.3 V), the version
/// read from the value (`XL1509-5V`, `XL1509-3.3V`).
///
/// The rail model **corrects the schematic symbol**, which draws all eight
/// pins as `input` — including `VIN`, the four grounds, and `OUT`.
/// Electrical descriptors come from the component, never from the netlist
/// (see the `netlist` module docs), and `OUT` as a `PowerOut` terminal is
/// what makes the board's rails reachable through the output inductors.
pub const XL1509_PART: &str = "XL1509";

/// Baud the EdgeBoard's isolated force-gauge UART runs at — the ADS122U04's
/// fixed 115.2 kbaud.
pub const FORCE_GAUGE_BAUD_HZ: u32 = 115_200;

/// Rail voltage of the isolated servo domain (`SC_5V`), from connector J21.
pub const SERVO_RAIL_VOLTS: Volts = 5.0;

/// Rail voltage of the P2's I/O domain.
pub const LOGIC_RAIL_VOLTS: Volts = 3.3;

/// Part registry for `fixtures/mad_edge.net`.
pub fn edge_registry() -> PartRegistry {
    let mut registry = edge_registry_without_socket();
    // The board's own edge-socket symbol is where the P2 module plugs in.
    registry.register_boundary("P2_EDGE_MODULE_SOCKET");
    registry
}

/// [`edge_registry`] without the `P2_EDGE_MODULE_SOCKET` boundary declaration,
/// so a test can show that the declaration is what classifies the board's
/// project-library socket symbol.
pub fn edge_registry_without_socket() -> PartRegistry {
    let mut registry = PartRegistry::new();

    // Modeled parts.
    registry.register("AM26LS31CD", |_decl| Box::new(Am26ls31::new()));
    registry.register("AM26LV32xD", |_decl| Box::new(Am26lv32::new()));

    // The ISO67xx isolators, configured straight from their part names —
    // `ISO6740FDWR` picks up its fail-safe-low default from the suffix.
    // `IC14`'s STEP channel carries the servo step clock as a clock, like
    // any channel handed one. `ISO6731DWR` is IC5, the force-gauge isolator.
    for part in [
        "ISO6742DWR",
        "ISO6741DWR",
        "ISO6740FDWR",
        "ISO6721BDR",
        "ISO6731DWR",
    ] {
        registry.register(part, move |decl: &ComponentDecl| {
            let name = normalize_part(decl);
            let config = iso67xx::Config::from_part_name(&name)
                .unwrap_or_else(|| panic!("{name} is an ISO67xx"));
            Box::new(Iso67xx::new(config).expect("a valid isolator configuration"))
        });
    }
    // The 21 Schmitt inverters driving the front-panel LEDs.
    registry.register("SN74LVC1G14DBV", |_decl| {
        Box::new(
            LogicGate::new(logic_gate::Config::lvc1g14(), &LVC1G14_PINS_SOT23)
                .expect("the datasheet configuration is valid"),
        )
    });

    // The optocouplers: `U4` (a Lite-On 6N137, the charge-pump drive) and
    // `U5`–`U8` (Vishay VO2631, the eight isolated digital-input loops),
    // each an LED branch the engine solves and a sink that releases.
    registry.register("6N137", |_decl| Box::new(Opto::lite_on_6n137()));
    registry.register("VO2631", |_decl| Box::new(Opto::vo2631()));
    // The elements by specification (`NODES.md` §8 phase 3): the polarity
    // FET `U3` with its body diode, the transistor `Q1`, the eight
    // current regulators `IC6`–`IC13`, the two Schottky diodes `D1`/`D2`
    // and the 21 indicator LEDs, every one keyed on the manufacturer part
    // number the export carries.
    pwl_library::register(&mut registry);

    // The power tree: the two bucks at the version their value names, the
    // two isolated DC/DCs at the setpoint their `SEL` strap selects.
    registry.register(XL1509_PART, |decl: &ComponentDecl| {
        let config = rail::Config::xl1509_from_value(&decl.value).unwrap_or_else(|| {
            panic!(
                "{}: the value {:?} names no fixed version",
                decl.reference, decl.value
            )
        });
        Box::new(Rail::new(config, &XL1509_PINS_SOP8).expect("the XL1509 table carries every role"))
    });
    registry.register(UCC12040_PART, |_decl| {
        Box::new(
            Rail::new(rail::Config::ucc12040(), &UCC12040_PINS_SOIC16)
                .expect("the UCC12040 table carries every role"),
        )
    });
    registry
}

/// Build the MaD EdgeBoard as a [`Board`].
pub fn edge_board() -> Board {
    let parsed = embsim_board::netlist::parse(include_str!("../fixtures/mad_edge.net"))
        .expect("the EdgeBoard fixture parses");
    Board::from_netlist(parsed, &edge_registry()).expect("the EdgeBoard builds")
}

/// Build the DS2 force-gauge add-on as a [`Board`], with the live ADS122U04
/// component (`embsim-models`) as its `U1`.
pub fn ds2_board() -> Board {
    use embsim_models::ads122u04::Config;
    let mut registry = PartRegistry::new();
    registry.register("ADS122U04", |_decl| {
        Box::new(embsim_models::ads122u04_component::Ads122u04Component::new(
            Config::default(),
        ))
    });
    let parsed = embsim_board::netlist::parse(include_str!("../fixtures/ds2_addon.net"))
        .expect("the DS2Addon fixture parses");
    Board::from_netlist(parsed, &registry).expect("the DS2Addon builds")
}

// ============================================================
// Harnesses
// ============================================================

/// Parse a dotted harness endpoint.
pub fn ep(endpoint: &str) -> EndpointRef {
    EndpointRef::parse(endpoint).expect("endpoint parses")
}

/// The J203 edge fingers the module's netlist declares a node for, in finger
/// order: 3..=54 (the P0..P37 block, the 5 V and GND fingers, the reset
/// finger, the first two I/O-bank supplies, and P58..P63) plus 57, 58, 68, 78,
/// 79 and 80.
///
/// The 22 fingers left out — 1, 2, 55, 56, 59..67 and 69..77 — are the ones
/// the harness never wires. Fingers 1 and 2 are the vendor's `NC` pads: they
/// *are* declared in the module netlist (on its `NC_Net`, which
/// `ec32mb_module.rs` asserts), just reserved and not to be connected. The
/// other 20 are the P40..P57 signals — including P56/P57, the PSRAM CLK and
/// CE — and the V40/V48 bank supplies, which the module consumes internally
/// for its four PSRAMs (Rev B product guide: P40-P57 are routed to the
/// on-module 32 MB RAM). A socket pin with nothing behind it is the correct
/// model of a finger that carries nothing.
pub fn edge_fingers() -> impl Iterator<Item = u32> {
    (3..=54).chain([57, 58, 68, 78, 79, 80])
}

/// The module-to-EdgeBoard interconnect: every declared J203 finger wired to
/// the J3 socket pin of the same number.
///
/// **Numbering.** Both netlists number the 80-way card edge by *finger*
/// number — the module's `J203` because the transcription took the printed
/// edge labels as pin functions and the finger numbers as pin ids, the
/// EdgeBoard's `J3` because its socket symbol is drawn the same way. So the
/// mapping is the identity, and the harness can say `J203.46 ↔ J3.46`
/// directly. That is a property of these two netlists, not of card-edge
/// connectors in general: had either side numbered its symbol 1..40 per row,
/// or counted from the other end, this function would carry the translation
/// table instead. The tests assert the correspondence rather than trusting it —
/// `ec32mb_module.rs` checks the module side finger by finger against the
/// vendor product guide, and `machine_system.rs` checks that every declared
/// finger resolves to one node across the socket *and* that both sides label
/// the free P0..P37 block with the same pin numbers.
pub fn module_socket_harness(module: &str, edge: &str) -> Harness {
    let mut harness = Harness::new();
    for finger in edge_fingers() {
        harness = harness.connect(
            ep(&format!("{module}.J203.{finger}")),
            ep(&format!("{edge}.J3.{finger}")),
        );
    }
    harness
}

/// The force-gauge cable: EdgeBoard `J9` (isolated force domain) to the DS2
/// add-on's `J1`.
///
/// | EdgeBoard J9 | net | DS2 J1 | net |
/// |---|---|---|---|
/// | 1 | `IFG_5V` | 1 | `+3V3` |
/// | 5 | `IFG_GND` | 2 | `GND` |
/// | 4 | `IFG_TX` | 3 | ADC RX (through R3) |
/// | 2 | `IFG_RX` | 4 | ADC TX (through R4) |
/// | 3 | `IFG_INT` | 5 | ADC `~DRDY` (through R5) |
///
/// The supply label disagrees across the connector — the EdgeBoard calls the
/// isolated rail `IFG_5V`, the add-on calls the same wire `+3V3` — because the
/// UCC12040 is strapped for the ADC's 3.3 V domain and the EdgeBoard net kept
/// the family name. The wire is one net either way; the *voltage* comes from
/// whatever sources it (see [`bench_rails`]), not from either label.
pub fn force_gauge_harness(edge: &str, ds2: &str) -> Harness {
    Harness::new()
        .connect(ep(&format!("{edge}.J9.1")), ep(&format!("{ds2}.J1.1")))
        .connect(ep(&format!("{edge}.J9.5")), ep(&format!("{ds2}.J1.2")))
        .connect(ep(&format!("{edge}.J9.4")), ep(&format!("{ds2}.J1.3")))
        .connect(ep(&format!("{edge}.J9.2")), ep(&format!("{ds2}.J1.4")))
        .connect(ep(&format!("{edge}.J9.3")), ep(&format!("{ds2}.J1.5")))
}

/// The machine cables: the servo/stepper drive on `J21`, the encoder on `J20`,
/// and the two end-of-travel switches on `J16` / `J15`.
///
/// Three mapping decisions are worth stating, because the netlist and the
/// silkscreen do not agree and the netlist wins:
///
/// 1. **The motor takes the `+` leg of the differential pair.** `SC_PUL±` and
///    `SC_DIR±` leave J21 as RS-422 pairs driven by `U24` ([`Am26ls31`]); a real
///    stepper driver's own receiver turns each pair back into one logic
///    signal. `embsim-models`' `StepperMotor` stands in for driver *and*
///    motor, so it reads `SC_PUL+` / `SC_DIR+` and the complementary legs go
///    unread — the pair is still generated and asserted, it is simply
///    terminated by a model that only needs half of it.
/// 2. **The encoder drives the `+` leg, with `−` grounded by the board's own
///    jumpers.** `QuadratureEncoder` has single-ended `A`/`B` outputs, so
///    they land on `A+`/`B+`; JP2/JP3 (`A_GND`/`B_GND`) tie `A−`/`B−` to the
///    isolated ground, which is exactly what those jumpers are on the board
///    for. Closing JP4 (`Z_GND`) additionally asserts the receiver's
///    active-low enable — see [`encoder_jumpers_closed`]. All three are
///    scenario state, so [`encoder_jumpers_closed`] carries them.
/// 3. **The end-switch connector labels are crossed with their nets.** J14 is
///    silkscreened `ENDUpper` but wired to `IDOOR±`, while J16 is
///    silkscreened `Door` and wired to `IEND_U±`. The isolator/opto chain
///    follows the *nets* (`IEND_U−` → U6 → `P19`, `IEND_L−` → U7 → `P20`,
///    `IDOOR−` → U7 → `P21`), and the firmware reads those pins, so this
///    harness follows the nets too: the upper end switch plugs into J16 and
///    the lower into J15. Worth fixing on the board; worth *knowing* now.
pub fn machine_harness(edge: &str) -> Harness {
    Harness::new()
        // Servo/stepper drive.
        .connect(ep(&format!("{edge}.J21.2")), ep("MOTOR.STEP"))
        .connect(ep(&format!("{edge}.J21.5")), ep("MOTOR.DIR"))
        .connect(ep(&format!("{edge}.J21.7")), ep("MOTOR.ENA"))
        // Encoder.
        .connect(ep("ENC.A"), ep(&format!("{edge}.J20.1")))
        .connect(ep("ENC.B"), ep(&format!("{edge}.J20.3")))
        // End of travel: upper on J16 (net IEND_U), lower on J15 (net IEND_L).
        .connect(ep(&format!("{edge}.J16.2")), ep("END_U.COM"))
        .connect(ep(&format!("{edge}.J16.1")), ep("END_U.NO"))
        .connect(ep(&format!("{edge}.J15.2")), ep("END_L.COM"))
        .connect(ep(&format!("{edge}.J15.1")), ep("END_L.NO"))
}

/// Bench supply straps for the EdgeBoard, through the board's own connector
/// pins — the rig a bring-up bench actually builds: the main input on the
/// screw terminal and the isolated servo domain on its connector. Nothing
/// else: `+5V` and `+3.3V` come from the board's own bucks `U1`/`U2`
/// behind the polarity FET `U3`, the instant the 12 V arrives (the XL1509
/// names no soft-start), so a strap on either would be a second declared
/// source on a rail the board generates.
///
/// | Endpoint | Net | Volts | Domain |
/// |---|---|---|---|
/// | `J2.1` | `Net-(J2-Pin_1)` | 12.0 | main input, ahead of the polarity FET |
/// | `J2.2` | `GND` | 0.0 | primary ground |
/// | `J21.1` | `SC_5V` | 5.0 | isolated servo domain |
/// | `J21.8` | `EN_GND` | 0.0 | isolated servo/encoder ground |
///
/// The isolated I/O domain (`5V_IO`/`GND_IO`, from `IC3`) is not strapped
/// and its return is tied to nothing on the board, so on the board alone
/// that rail stays down with its reference named — `DESIGN.md` rule 6, no
/// implicit ground: a test that needs the domain declares its return
/// through a connector pin (`J10.2`, `J5.3`–`J8.3`).
pub fn bench_rails(edge: &str) -> Harness {
    Harness::new()
        .power(ep("BENCH.12V"), ep(&format!("{edge}.J2.1")), 12.0)
        .power(ep("BENCH.GND"), ep(&format!("{edge}.J2.2")), 0.0)
        .power(
            ep("BENCH.SERVO5V"),
            ep(&format!("{edge}.J21.1")),
            SERVO_RAIL_VOLTS,
        )
        .power(ep("BENCH.SERVOGND"), ep(&format!("{edge}.J21.8")), 0.0)
}

// ============================================================
// Scenario fragments
// ============================================================

/// Close the encoder's ground/enable jumpers: JP2 (`A_GND`), JP3 (`B_GND`) and
/// JP4 (`Z_GND`).
///
/// These three are what let a single-ended encoder drive an RS-422 receiver —
/// they tie the `−` leg of each pair to the isolated ground so the receiver
/// sees a real differential, and JP4 additionally asserts the receiver's
/// active-low enable, which the board wires to `Z−`. See [`machine_harness`]
/// item 2.
///
/// A board note worth stating, because it looks like a mistake and is not:
/// the Edge board wires the encoder's index pair to the receiver `U25`'s
/// *enable* pins — `Z+` to `G` (pin 4) and `Z−` to `G̅` (pin 12) — rather
/// than to a receiver channel. Closing JP4 ties `Z−` to the isolated ground,
/// which asserts `G̅` low and enables all four channels unconditionally, so
/// that jumper is load-bearing for the encoder path. Channel 4's inputs are
/// no-connects on this board: they rest at the part's own open-input bias,
/// and the fail-safe holds `4Y` high (`embsim_models::am26lv32`).
pub fn encoder_jumpers_closed(scenario: Scenario, edge: &str) -> Scenario {
    scenario
        .jumper(&format!("{edge}.JP2"), JumperState::Closed)
        .jumper(&format!("{edge}.JP3"), JumperState::Closed)
        .jumper(&format!("{edge}.JP4"), JumperState::Closed)
}

/// Isolated force-domain straps for the DS2 add-on **on its own**, applied
/// on its connectors (`J1.1`/`J1.2`, `J2.1`/`J2.2`): the add-on has no
/// regulator, so on the bench its `+3V3` and analog supply are declared
/// here. Mirrors the DS2 bench rig in `board/tests/ds2_regressions.rs`.
/// With the add-on on the Edge board's cable the digital rail is the
/// board's `IC4` (5.0 V, `SEL` to `VISO`), so the assembled machine uses
/// [`force_domain_ground`] instead — a 3.3 V strap there would be a second
/// declared source on a 5 V rail.
pub fn force_domain_rails(ds2: &str) -> Harness {
    Harness::new()
        .power(
            ep("BENCH.IFG3V3"),
            ep(&format!("{ds2}.J1.1")),
            LOGIC_RAIL_VOLTS,
        )
        .power(ep("BENCH.IFGGND"), ep(&format!("{ds2}.J1.2")), 0.0)
        .power(
            ep("BENCH.VDDA"),
            ep(&format!("{ds2}.J2.1")),
            LOGIC_RAIL_VOLTS,
        )
        .power(ep("BENCH.AGND"), ep(&format!("{ds2}.J2.2")), 0.0)
}

/// The isolated force domain's **references** for the assembled machine:
/// the domain's return, tied down over the cable on the add-on's `J1.2`
/// (`IFG_GND`), and the add-on's analog supply and return on `J2`, which
/// nothing on either board generates. The domain's 5 V is the Edge board's
/// `IC4`, 750 µs after its input arrives (the UCC12040's rise time), so in
/// a build snapshot it is down and said so (`Finding::RailDown`). No
/// ground is implicit (`DESIGN.md` rule 6): an isolated domain's reference
/// is a harness terminal, as the primary bench return is.
pub fn force_domain_ground(ds2: &str) -> Harness {
    Harness::new()
        .power(ep("BENCH.IFGGND"), ep(&format!("{ds2}.J1.2")), 0.0)
        .power(
            ep("BENCH.VDDA"),
            ep(&format!("{ds2}.J2.1")),
            LOGIC_RAIL_VOLTS,
        )
        .power(ep("BENCH.AGND"), ep(&format!("{ds2}.J2.2")), 0.0)
}
