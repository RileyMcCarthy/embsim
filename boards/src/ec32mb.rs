//! The **Parallax P2-EC32MB** — a Propeller 2 module you can buy, as a board.
//!
//! An off-the-shelf module is the most useful thing a simulator can ship: it is
//! the same part on every desk, so a test written against it means the same
//! thing everywhere, and a CPU model driving it is being driven by real
//! topology rather than by a harness someone invented.
//!
//! What this crate supplies is the module minus its processor. `U100` is a
//! **slot** the consumer fills with a [`crate::p2::P2Package`] — the P2's
//! 86-pin package around a core: QEMU's target, an instruction-set
//! simulator, the native firmware behind `McuComponent`, or no core at all
//! ([`crate::p2::P2Package::held_in_reset`], the state any P2 is in before
//! it runs) — because the CPU is the thing under test and the board is the
//! fixture. Everything around it is here:
//! the boot flash and the card socket as live models, the TCXO as an
//! oscillator publishing its 20 MHz as a rate, the two dual inverters as
//! gates (the oscillator buffer relaying that rate to `XI`, the LED buffer
//! sinking the cathodes), the four PSRAMs as memories answering the SPI
//! command set, the DIP switch and the oscillator-option solder link as
//! switches with poles, the mounting holes and the BOM-only lines as
//! mechanical nodes, the polarity FET and the two white LEDs as elements
//! by specification, the two bucks and the eight LDOs as rails — each
//! output a declared terminal, published at its setpoint from the instant
//! its input allows plus its soft-start, the bucks' setpoints read from
//! their feedback dividers at attach — the brownout detector as a
//! comparator holding `RESN` while the core rail is under 1.6 V, and all
//! 114 components classified and validated against the vendor netlist —
//! every one a node, none a facade.
//!
//! # The power tree, and what it means for a test
//!
//! From the carrier's 5 V and 0 V on `J203`, the polarity FET `U401`
//! passes the input, the bucks `U402` (1.813 V, `Common_VDD`) and `U403`
//! (3.649 V, `Common_LDOin`) rise **2.5 ms** later (the AP62301's
//! soft-start), and the eight LDOs step to 3.3 V on the `VIO_*` bank rails
//! at that same instant. A build snapshot is the state before the first
//! wake, so every module rail is down in it and reported as such
//! ([`embsim_board::Finding::RailDown`]); a test that wants the rails up
//! starts the system and steps past the soft-start
//! (`board/tests/power_tree.rs`).
//!
//! ```no_run
//! # use embsim_boards::ec32mb::Ec32mb;
//! # use embsim_boards::p2::P2Package;
//! let board = Ec32mb::new()
//!     .with_flash_image(std::fs::read("firmware.bin").unwrap())
//!     .with_p2(|_decl| Box::new(P2Package::native(my_cpu())))
//!     .build()
//!     .expect("the module builds");
//! # fn my_cpu() -> embsim_board::McuComponent { unimplemented!() }
//! ```
//!
//! # The SPI bus is SHARED, and that is the interesting part
//!
//! Four P2 pins serve both the flash and the card, and not in the pairing you
//! would guess. Taken from the netlist's own nets and pin functions:
//!
//! | P2 pin | net | flash `U301` | socket `J301` |
//! |---|---|---|---|
//! | P58 | `P2_IO58` / "P58/FLASH_MISO" | `DO(IO1)` direct | `DAT0/MISO` **through R304, 240 Ω** |
//! | P59 | `P2_IO59` / "P59/FLASH_MOSI" | `DI(IO0)` | `CMD/MOSI` |
//! | P60 | `P2_IO60` / "P60/FLASH_CLK" | **`CLK`** | **`CD/DAT3/CS`** |
//! | P61 | `P2_IO61` / "P61/FLASH_CS" | — (see `S301`) | **`CLK`** |
//!
//! Read the last two rows twice: **P60 is the flash's clock and the card's chip
//! select at the same time**, and P61 is the card's clock. A driver that
//! toggles the flash clock is deselecting and reselecting the card on every
//! edge. This is why a board model matters — a harness that wired each device
//! to four private pins would be a different circuit, and would never reproduce
//! it.
//!
//! The flash's own `~CS` sits on the `SPI_CS` net with a pull-up (`R301` to
//! `VIO_56_63`) and one side of `S301` switch 2, labelled "FLASH" in the
//! netlist: closing it ties `P2_IO61` to the flash `~CS`, and leaving it open
//! lets the pull-up hold the flash deselected so only the card is on the bus.
//! [`FLASH_SELECT_SWITCH`] names the switch and [`FLASH_SELECT_POLE`] the
//! pole: `Scenario::switch("EC32.S301", FLASH_SELECT_POLE, JumperState::Closed)`.
//!
//! # Sources
//!
//! `netlists/p2_ec32mb.net`, transcribed from `P2-EC32MB-RevB-SCHEMATIC.pdf`
//! and carrying the vendor provenance notes in its header. It has no
//! `(libsource …)` entries, so the registry classifies by reference-designator
//! prefix and keys parts on their `value` field.

use embsim_board::{
    netlist, Board, BoardError, Component, ComponentDecl, PartRegistry, SwitchPole,
};
use embsim_models::logic_gate::{self, LogicGate, LVC2G04_PINS_BY_FUNCTION};
use embsim_models::oscillator::{self, Oscillator};
use embsim_models::psram::{Psram, PsramComponent};
use embsim_models::pwl_library;
use embsim_models::rail::{self, Rail, AP62301_PINS_BY_FUNCTION, NCP114_PINS_BY_FUNCTION};
use embsim_models::sd_card::SdCard;
use embsim_models::sd_card_component::{SdCardComponent, SD_CARD_PINS_BY_FUNCTION};
use embsim_models::spi_flash::SpiNorFlash;
use embsim_models::spi_flash_component::{
    FlashView, SpiNorFlashComponent, SPI_FLASH_PINS_BY_FUNCTION,
};
use embsim_models::supervisor::{self, VoltageDetector, STM1061_PINS_BY_FUNCTION};

/// The vendor netlist this board is built from.
pub const NETLIST: &str = include_str!("../netlists/p2_ec32mb.net");

/// Registry key for the processor slot — the `value` of `U100`.
pub const P2_PART: &str = "P2X8C4M64P";
/// Registry key for the boot flash, `U301` (Winbond `W25Q128JVSIM`).
pub const FLASH_PART: &str = "SPI Flash 16MB (128Mb)";
/// Registry key for the card socket, `J301` (Molex `473092651`).
pub const SOCKET_PART: &str = "MicroSD Socket";

/// 128 M-bit = 16 MiB, the density the netlist states for `U301`.
pub const FLASH_CAPACITY: usize = 16 * 1024 * 1024;

/// `S301` — the module's four-way option switch. Position 2 (both sides
/// labelled "FLASH" in the netlist) closed ties `P2_IO61` to the flash `~CS`;
/// open, `R301` holds the flash deselected. See [`dip_switch_poles`] for the
/// pole each position is.
pub const FLASH_SELECT_SWITCH: &str = "S301";

/// The pole index of `S301` position 2, FLASH (positions are printed 1–4,
/// poles are indexed from 0 in [`dip_switch_poles`] order).
pub const FLASH_SELECT_POLE: usize = 1;
/// The pole index of `S301` position 3, the P59 pull-up (`R302`).
pub const P59_PULL_UP_POLE: usize = 2;
/// The pole index of `S301` position 4, the P59 pull-down (`R303`) — the
/// module's "boot from flash without waiting for a serial loader" setting.
pub const P59_PULL_DOWN_POLE: usize = 3;

/// Registry key for the DIP switch, `S301` (CTS `218-4LPSTJR`) — its `value`.
pub const DIP_SWITCH_PART: &str = "DIP Switch 4 way";
/// Registry key for the oscillator-option solder link, `J101` — its `value`.
pub const SOLDER_LINK_PART: &str = "Solder Link Pads";
/// The `value` of the mounting holes `J701`/`J702` (tied to `GND`).
pub const MOUNTING_HOLE_PART: &str = "Mounting Hole Vss";
/// The `value` of `PCB`, the raw board (a BOM line with no nodes).
pub const RAW_PCB_PART: &str = "PCB for P2 EC Module";
/// The `value` of `NC_Net`, the layout node terminating the `NC_Net` net.
pub const LAYOUT_NODE_PART: &str = "Layout node";

/// Registry key for the dual inverters `U101` (the oscillator buffer) and
/// `U601` (the LED buffer) — their `value`, NXP `74LVC2G04GW,125`.
pub const INVERTER_PART: &str = "74LVC2G04GW,125";
/// Registry key for the TCXO `X100` — its `value`, EPSON
/// `TG2520SMN 20.0000M-ECGNNM3`, which also names its frequency.
pub const TCXO_PART: &str = "TG2520SMN 20.0000M-ECGNNM3";
/// Registry key for the four PSRAMs `U302`–`U305` — their `value`.
pub const PSRAM_PART: &str = "PSRAM 64Mbit";

/// The TCXO's frequency, as its value names it: 20 MHz.
pub const TCXO_HZ: u32 = 20_000_000;

/// Registry key for the two bucks `U402` (the core rail, `Common_VDD`) and
/// `U403` (the LDO input rail, `Common_LDOin`) — their `value`, Diodes
/// `AP62301Z6-7`. One key, two setpoints: each part reads its own feedback
/// divider at attach.
pub const BUCK_PART: &str = "DCDC 3A SOT563";
/// Registry key for the brownout detector `U404` — its `value`, STMicro
/// `STM1061N16WX6F`.
pub const BROWNOUT_DETECTOR_PART: &str = "Voltage Detector 1.6V";
/// Registry key for the eight bank LDOs `U501`–`U508` — their `value`,
/// onsemi `NCP114AMX330TCG`, which also names their 3.3 V.
pub const LDO_PART: &str = "LDO 300mA, 3.3V";

/// `DIP Switch 4 way` — the module's option switch (S301): four poles, one
/// per printed position, each between the netlist's `<position>_ON` and
/// `<position>_OFF` pin ids. The pairing is a registry declaration — the
/// netlist carries none — and is what the pin-function labels say
/// (`"FLASH (ON side)"` / `"FLASH (OFF side)"` on position 2). Every pole
/// is open by default, the state a module ships in; a scenario closes one
/// by index (`Scenario::switch("EC32.S301", pole, JumperState::Closed)`),
/// and a closed pole joins its two nets into one node. Position *n* is pole
/// `n - 1`: [`FLASH_SELECT_POLE`], [`P59_PULL_UP_POLE`] and
/// [`P59_PULL_DOWN_POLE`] name the three the boot depends on.
///
/// | pole | position | ON side ↔ OFF side | function (vendor labels) |
/// |---|---|---|---|
/// | 0 | 1 | `1_ON` ↔ `1_OFF` | LED ENABLE |
/// | 1 | 2 | `2_ON` ↔ `2_OFF` | FLASH: `P2_IO61` to the flash `~CS` |
/// | 2 | 3 | `3_ON` ↔ `3_OFF` | P59 pull-up (`R302`) |
/// | 3 | 4 | `4_ON` ↔ `4_OFF` | P59 pull-down (`R303`) |
pub fn dip_switch_poles() -> Vec<SwitchPole> {
    (1..=4)
        .map(|position| SwitchPole::open(format!("{position}_ON"), format!("{position}_OFF")))
        .collect()
}

/// `Solder Link Pads` — `J101`, the oscillator-option link between `P2_IO32`
/// and the TCXO's `NC/GND` option pad: one pole across its two pads, open
/// (the vendor ships it unbridged).
pub fn solder_link_poles() -> Vec<SwitchPole> {
    vec![SwitchPole::open("1", "2")]
}

/// The class of every part that is not a live model. [`Ec32mb::registry`]
/// layers the live models and the processor slot on top.
fn class_registry() -> PartRegistry {
    let mut registry = PartRegistry::new();
    // The netlist was transcribed from the vendor PDF and has no libsource, so
    // the auto tier keys on reference-designator prefixes and the registry on
    // the `value` field.
    registry.classify_unnamed_by_reference(true);

    // Switches, by pole.
    registry.register_switch(DIP_SWITCH_PART, dip_switch_poles());
    registry.register_switch(SOLDER_LINK_PART, solder_link_poles());

    // Mechanical parts: pads (or no pads at all), nothing electrical.
    registry.register_mechanical(MOUNTING_HOLE_PART);
    registry.register_mechanical(RAW_PCB_PART);
    registry.register_mechanical(LAYOUT_NODE_PART);

    // Models. The TCXO publishes the rate its value names, one event, at
    // its datasheet start-up instant; the oscillator buffer `U101` relays
    // it across the AC-coupling capacitor `C132` to `XI` and rests its
    // self-biased stage mid-rail; the LED buffer `U601` sinks the cathodes
    // of `D601`/`D602` from `P38`/`P39`; the PSRAMs answer the SPI command
    // set from an 8 MiB array each.
    registry.register(TCXO_PART, |decl| {
        let config = oscillator::Config::from_value(&decl.value)
            .unwrap_or_else(|| oscillator::Config::tg2520smn(TCXO_HZ));
        Box::new(Oscillator::new(config))
    });
    registry.register(INVERTER_PART, |_decl| {
        Box::new(
            LogicGate::new(logic_gate::Config::lvc2g04(), &LVC2G04_PINS_BY_FUNCTION)
                .expect("the 74LVC2G04 configuration is the datasheet's"),
        )
    });
    registry.register(PSRAM_PART, |_decl| {
        Box::new(PsramComponent::new(Psram::new()))
    });

    // The elements registered by specification (`NODES.md` §8 phase 3):
    // the polarity FET `U401` — a Si3417DV by its `MPN` field, its channel
    // and body diode — and the two white LEDs `D601`/`D602`, keyed on
    // their number, all from the element library.
    pwl_library::register(&mut registry);

    // The power tree (`NODES.md` §8 phase 4): the two bucks from one key,
    // each reading its feedback divider at attach; the eight LDOs at the
    // voltage their value names; the detector at its 1.6 V threshold.
    registry.register(BUCK_PART, |_decl| {
        Box::new(
            Rail::new(rail::Config::ap62301(), &AP62301_PINS_BY_FUNCTION)
                .expect("the AP62301 table carries every role"),
        )
    });
    registry.register(LDO_PART, |decl| {
        let config = rail::Config::ncp114_from_value(&decl.value)
            .unwrap_or_else(|| panic!("{}: the LDO value names no voltage", decl.reference));
        Box::new(
            Rail::new(config, &NCP114_PINS_BY_FUNCTION)
                .expect("the NCP114 table carries every role"),
        )
    });
    registry.register(BROWNOUT_DETECTOR_PART, |_decl| {
        Box::new(VoltageDetector::new(
            supervisor::Config::stm1061n16(),
            &STM1061_PINS_BY_FUNCTION,
        ))
    });
    registry
}

// ============================================================
// The board
// ============================================================

/// Constructor for the processor the module is built around.
type P2Ctor = Box<dyn Fn(&ComponentDecl) -> Box<dyn Component> + Send + Sync>;

/// A P2-EC32MB module, with its processor slot to fill.
#[derive(Default)]
pub struct Ec32mb {
    /// The boot flash, built up front so a [`FlashView`] can be handed out
    /// before the part moves into the board. Taken exactly once, at build.
    flash: Option<std::sync::Mutex<Option<SpiNorFlashComponent>>>,
    flash_view: Option<FlashView>,
    sd: Option<SdCard>,
    p2: Option<P2Ctor>,
}

impl std::fmt::Debug for Ec32mb {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Ec32mb")
            .field("flash", &self.flash.as_ref().map(|_| "programmed"))
            .field("sd", &self.sd.as_ref().map(SdCard::capacity))
            .field("p2", &self.p2.as_ref().map(|_| "supplied"))
            .finish()
    }
}

impl Ec32mb {
    /// A module with a blank boot flash, no card in the socket, and the
    /// processor slot empty.
    pub fn new() -> Self {
        Self::default()
    }

    /// Program the boot flash before the module comes up — what a loader would
    /// have left behind.
    ///
    /// The part is its real 16 MiB whatever the image's size: a boot image is
    /// a few kilobytes, and a ROM that reads past it must see the `$FF` a
    /// blank part gives rather than running off the end of a short array.
    #[must_use]
    pub fn with_flash_image(mut self, image: Vec<u8>) -> Self {
        let mut array = vec![0xFFu8; FLASH_CAPACITY];
        let end = image.len().min(FLASH_CAPACITY);
        array[..end].copy_from_slice(&image[..end]);
        let component = SpiNorFlashComponent::new(SpiNorFlash::with_image(array))
            .with_pins(&SPI_FLASH_PINS_BY_FUNCTION);
        self.flash_view = Some(component.view());
        self.flash = Some(std::sync::Mutex::new(Some(component)));
        self
    }

    /// A view of the programmed boot flash — what it served, what it was
    /// told — that stays valid after the board is built and running. `None`
    /// until [`with_flash_image`](Self::with_flash_image).
    pub fn flash_view(&self) -> Option<FlashView> {
        self.flash_view.clone()
    }

    /// Put a card in the socket. Without this the socket is empty, and a driver
    /// reading it gets what an empty socket gives: nothing driving MISO.
    #[must_use]
    pub fn with_card(mut self, card: SdCard) -> Self {
        self.sd = Some(card);
        self
    }

    /// Fill the processor slot — with a [`crate::p2::P2Package`] around the
    /// core under test, or [`crate::p2::P2Package::held_in_reset`] for a
    /// test about the board.
    ///
    /// Left empty, `U100` classifies as an unregistered part and the board
    /// refuses to build — deliberately, because a module whose processor
    /// silently did not exist would look like a working board that never runs.
    #[must_use]
    pub fn with_p2(
        mut self,
        ctor: impl Fn(&ComponentDecl) -> Box<dyn Component> + Send + Sync + 'static,
    ) -> Self {
        self.p2 = Some(Box::new(ctor));
        self
    }

    /// The part registry this configuration produces, for a consumer that wants
    /// to add or replace entries before building.
    pub fn registry(self) -> PartRegistry {
        let mut registry = class_registry();

        // A programmed part was built in `with_flash_image` so its view could
        // be handed out; a blank one is built here. `U301` appears once in
        // the netlist, so the slot is taken exactly once.
        let slot = self.flash.unwrap_or_else(|| std::sync::Mutex::new(None));
        registry.register(FLASH_PART, move |_decl| {
            let programmed = slot.lock().expect("flash slot never poisoned").take();
            Box::new(programmed.unwrap_or_else(|| {
                SpiNorFlashComponent::new(SpiNorFlash::blank(FLASH_CAPACITY))
                    .with_pins(&SPI_FLASH_PINS_BY_FUNCTION)
            }))
        });

        // An empty socket stays a board boundary: there is no card to model, and
        // pretending otherwise would drive MISO for a slot with nothing in it.
        if let Some(card) = self.sd {
            let blocks = card.blocks;
            registry.register(SOCKET_PART, move |_decl| {
                Box::new(
                    SdCardComponent::new(SdCard::with_image(blocks.clone()))
                        .with_pins(&SD_CARD_PINS_BY_FUNCTION),
                )
            });
        }

        if let Some(ctor) = self.p2 {
            registry.register(P2_PART, move |decl| ctor(decl));
        }
        registry
    }

    /// Build the module.
    pub fn build(self) -> Result<Board, BoardError> {
        let parsed = netlist::parse(NETLIST).expect("the bundled netlist parses");
        Board::from_netlist(parsed, &self.registry())
    }
}
