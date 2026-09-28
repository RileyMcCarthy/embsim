//! The **Parallax P2-EC32MB** — a Propeller 2 module you can buy, as a board.
//!
//! An off-the-shelf module is the most useful thing a simulator can ship: it is
//! the same part on every desk, so a test written against it means the same
//! thing everywhere, and a CPU model driving it is being driven by real
//! topology rather than by a harness someone invented.
//!
//! What this crate supplies is the module minus its processor. `U100` is a
//! **slot** the consumer fills — with an instruction-set simulator, a stub, or
//! anything else implementing [`Component`] — because the CPU is the thing
//! under test and the board is the fixture. Everything around it is here:
//! the boot flash and the card socket as live models, the PSRAMs, regulators,
//! oscillator and DIP switch as pin facades, and all 168 components validated
//! against the vendor netlist.
//!
//! ```no_run
//! # use embsim_boards::ec32mb::Ec32mb;
//! let board = Ec32mb::new()
//!     .with_flash_image(std::fs::read("firmware.bin").unwrap())
//!     .with_p2(|_decl| Box::new(my_cpu()))
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
//! [`FLASH_SELECT_SWITCH`] names it.
//!
//! # Sources
//!
//! `netlists/p2_ec32mb.net`, transcribed from `P2-EC32MB-RevB-SCHEMATIC.pdf`
//! and carrying the vendor provenance notes in its header. It has no
//! `(libsource …)` entries, so the registry classifies by reference-designator
//! prefix and keys parts on their `value` field.

use embsim_board::{netlist, Board, BoardError, Component, ComponentDecl, PartRegistry, PinDecl};
use embsim_models::sd_card::SdCard;
use embsim_models::sd_card_component::{SdCardComponent, SD_CARD_PINS_BY_FUNCTION};
use embsim_models::spi_flash::SpiNorFlash;
use embsim_models::spi_flash_component::{
    FlashView, SpiNorFlashComponent, SPI_FLASH_PINS_BY_FUNCTION,
};

use crate::stub::{dig_in, passive, pwr_in, pwr_out, register_stub};

/// The vendor netlist this board is built from.
pub const NETLIST: &str = include_str!("../netlists/p2_ec32mb.net");

/// Reference designators with no electrical existence: `PCB` is the raw board
/// (no nodes at all) and `NC_Net` is a layout artefact with one. They are the
/// documented escape rather than fake components.
pub const STUB_REFS: [&str; 2] = ["PCB", "NC_Net"];

/// Registry key for the processor slot — the `value` of `U100`.
pub const P2_PART: &str = "P2X8C4M64P";
/// Registry key for the boot flash, `U301` (Winbond `W25Q128JVSIM`).
pub const FLASH_PART: &str = "SPI Flash 16MB (128Mb)";
/// Registry key for the card socket, `J301` (Molex `473092651`).
pub const SOCKET_PART: &str = "MicroSD Socket";

/// 128 M-bit = 16 MiB, the density the netlist states for `U301`.
pub const FLASH_CAPACITY: usize = 16 * 1024 * 1024;

/// `S301` position 2 — the netlist labels both its sides "FLASH". Closed, it
/// ties `P2_IO61` to the flash `~CS`; open, `R301` holds the flash deselected.
pub const FLASH_SELECT_SWITCH: &str = "S301";

/// `74LVC2G04GW,125` — NXP dual inverter (U101 oscillator buffer, U601 LED
/// buffer). Outputs are senses per the module docs' stub rule.
pub const INVERTER_2G04_PINS: [PinDecl; 6] = [
    pwr_in("GND"),
    dig_in("1A"),
    dig_in("1Y"),
    dig_in("2A"),
    dig_in("2Y"),
    pwr_in("VCC"),
];

/// `TG2520SMN 20.0000M-ECGNNM3` — EPSON 20 MHz TCXO (X100). `NC_GND` is the
/// vendor's option pad, brought to the `J101` solder link.
pub const TCXO_PINS: [PinDecl; 4] = [
    pwr_in("VCC"),
    pwr_in("GND"),
    dig_in("OUT"),
    passive("NC_GND"),
];

/// `SPI Flash 16MB (128Mb)` — Winbond W25Q128JV (U301).
pub const SPI_FLASH_STUB_PINS: [PinDecl; 8] = [
    pwr_in("VSS"),
    pwr_in("VCC"),
    dig_in("CLK"),
    dig_in("CSn"),
    dig_in("DI_IO0"),
    dig_in("DO_IO1"),
    dig_in("HOLDn"),
    dig_in("WPn"),
];

/// `PSRAM 64Mbit` — AP Memory APS6404L (U302..U305). `NC_EP` is the exposed
/// pad, tied per the vendor's asymmetric routing note.
pub const PSRAM_PINS: [PinDecl; 9] = [
    pwr_in("VSS"),
    pwr_in("VDD"),
    passive("NC_EP"),
    dig_in("SCLK"),
    dig_in("CEn"),
    dig_in("SI_SIO0"),
    dig_in("SO_SIO1"),
    dig_in("SIO2"),
    dig_in("SIO3"),
];

/// `P Mosfet 30V 8A` — Vishay SI3417DV reverse-polarity pass FET (U401). Its
/// conducting channel is not modeled; the system description expresses it with
/// a `pin_short` (a `pin_short` in the scenario).
pub const POLARITY_FET_PINS: [PinDecl; 3] = [dig_in("G"), passive("D"), passive("S")];

/// `DCDC 3A SOT563` — Diodes AP62301Z buck (U402, U403). `SW` is declared
/// `PinKind::PowerOut`: it is the switching node the output inductor
/// integrates into a rail, and marking it a source is what makes the module's
/// power tree reachable.
pub const BUCK_PINS: [PinDecl; 5] = [
    pwr_in("VIN"),
    pwr_in("GND"),
    passive("BST"),
    dig_in("FB"),
    pwr_out("SW"),
];

/// `Voltage Detector 1.6V` — STMicro STM1061 brownout detector (U404). `~OUT`
/// is open-drain and unmodeled, so it is a sense.
pub const BROWNOUT_PINS: [PinDecl; 3] = [pwr_in("VCC"), pwr_in("VSS"), dig_in("OUT")];

/// `LDO 300mA, 3.3V` — OnSemi NCP114 (U501..U508), one per P2 I/O bank pair.
pub const LDO_PINS: [PinDecl; 5] = [
    dig_in("EN"),
    pwr_in("IN"),
    pwr_in("GND"),
    pwr_in("GND_P"),
    pwr_out("OUT"),
];

/// `DIP Switch 4 way` — the module's option switch (S301), pin ids
/// `<position>_<OFF|ON>`.
///
/// All eight terminals are passive: a four-gang switch is not a two-terminal
/// jumper, so the auto tier cannot classify it and the engine has no
/// scenario primitive for a ganged position yet. The consequence is visible
/// and asserted — with every gang open, the `P59` pull-up/pull-down and the
/// flash chip-select strap do not conduct.
pub const DIP_SWITCH_PINS: [PinDecl; 8] = [
    passive("1_ON"),
    passive("1_OFF"),
    passive("2_ON"),
    passive("2_OFF"),
    passive("3_ON"),
    passive("3_OFF"),
    passive("4_ON"),
    passive("4_OFF"),
];

/// Reference designators the module netlist declares that have no electrical
/// existence: `PCB` is the raw board (no nodes at all) and `NC_Net` a layout
/// node. Passed to [`Board::from_netlist_with_stubs`].
pub const EC32MB_STUB_REFS: [&str; 2] = ["PCB", "NC_Net"];

/// Every part that is a pin facade and nothing more. [`Ec32mb::registry`]
/// layers the live models and the processor slot on top.
fn stub_registry() -> PartRegistry {
    let mut registry = PartRegistry::new();
    // The netlist was transcribed from the vendor PDF and has no libsource, so
    // the auto tier keys on reference-designator prefixes and the registry on
    // the `value` field.
    registry.classify_unnamed_by_reference(true);

    register_stub(&mut registry, "74LVC2G04GW,125", &INVERTER_2G04_PINS);
    register_stub(&mut registry, "TG2520SMN 20.0000M-ECGNNM3", &TCXO_PINS);
    register_stub(
        &mut registry,
        "SPI Flash 16MB (128Mb)",
        &SPI_FLASH_STUB_PINS,
    );
    register_stub(&mut registry, "PSRAM 64Mbit", &PSRAM_PINS);
    register_stub(&mut registry, "P Mosfet 30V 8A", &POLARITY_FET_PINS);
    register_stub(&mut registry, "DCDC 3A SOT563", &BUCK_PINS);
    register_stub(&mut registry, "Voltage Detector 1.6V", &BROWNOUT_PINS);
    register_stub(&mut registry, "LDO 300mA, 3.3V", &LDO_PINS);
    register_stub(&mut registry, "DIP Switch 4 way", &DIP_SWITCH_PINS);
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

    /// Fill the processor slot.
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
        let mut registry = stub_registry();

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
        Board::from_netlist_with_stubs(parsed, &self.registry(), &STUB_REFS)
    }
}
