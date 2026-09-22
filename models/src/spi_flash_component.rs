//! Board-engine adapter: [`crate::spi_flash::SpiNorFlash`] as a live
//! [`embsim_board::Component`].
//!
//! The device model stays a pure bit-level state machine; this is the seam
//! that mounts it on a netlist, so a *system description* decides which pins
//! it sits on rather than the model hardcoding them:
//!
//! ```text
//!  net engine                      adapter                    device model
//!  ──────────                      ───────                    ────────────
//!  ~CS  on_sense ──active low──► set_selected(!high) ──► phase reset
//!  DI   on_sense ─────────────► remembered, sampled at the next clock edge
//!  CLK  on_sense ─────────────► clock(high, di) ──────► shift / consume
//!  DO   ◄── digital_drive(miso()) ◄── after every ~CS or CLK sense
//! ```
//!
//! # Why the data line is driven from the sense callbacks
//!
//! The engine enqueues drives and resolves them in a later iteration, so a
//! device cannot answer "inline" within the master's read. It does not need
//! to: the master drives the clock, the engine resolves that and delivers this
//! callback, the callback enqueues the answering DO level, and the engine
//! resolves *that* before the master's next sense — provided the master yields
//! to the engine between the two. A bit-banging master must therefore yield at
//! each pin drive (see `PinBus::take_net_yield` on the Propeller 2 ISS); a
//! peripheral-clocked master gets it for free, because the transfer is a
//! scheduled event rather than a spin.
//!
//! That requirement is a property of the engine's single-writer contract, not
//! of any particular chip, and it applies to every bit-banged device on a net.
//!
//! # Chip select is active low HERE, not in the model
//!
//! The device model takes an asserted/not-asserted boolean. Inverting ~CS is
//! the adapter's job because active-low is a property of the part's wiring, so
//! a variant strapped the other way needs no change to the state machine.
//!
//! # Pin facade
//!
//! The 8-pin SOIC 208-mil package of the W25Q128JV (§3.3, p.5): `/CS`,
//! `DO (IO1)`, `/WP (IO2)`, `GND`, `DI (IO0)`, `CLK`, `/HOLD or /RESET (IO3)`,
//! `VCC`. All eight are declared because netlist validation checks both
//! directions — declared-but-absent and present-but-undeclared are equally
//! hard errors.
//!
//! **Which identifiers those are depends on the netlist, not on the part.**
//! `validate_facade` matches `PinDecl::number` verbatim against the netlist's
//! `(pin "…")`, so a KiCad export that numbers pins needs
//! [`SPI_FLASH_PINS_SOIC8`] while a netlist transcribed with functional names
//! — as the Parallax P2-EC32MB one is — needs
//! [`SPI_FLASH_PINS_BY_FUNCTION`]. Both carry the same `name` aliases, and
//! [`ComponentNetIo`] keys handles under number *and* name, so the attach code
//! below is identical either way.
//!
//! `~WP` and `~HOLD` are declared and **not modelled**: block protection and
//! the hold function are outside [`SpiNorFlash`]'s scope (see its
//! simplifications). A system that strapped either active would get no answer
//! on hardware and a normal answer here, which is the honest limit of this
//! adapter.

use std::sync::{Arc, Mutex};

use embsim_board::{
    digital_drive, level_of, AttachError, Component, ComponentNetIo, Level, PinDecl, PinHandle,
    PinKind,
};
use tracing::trace;

use crate::spi_flash::SpiNorFlash;

const fn pin(number: &'static str, name: &'static str, kind: PinKind) -> PinDecl {
    PinDecl {
        number,
        name: Some(name),
        kind,
        stream: None,
        drive_impedance: None,
    }
}

/// The SOIC-8 facade with datasheet pin NUMBERS as identifiers (§3.3, p.5) —
/// for a netlist that numbers its pins, as a KiCad export does.
pub const SPI_FLASH_PINS_SOIC8: [PinDecl; 8] = [
    pin("1", "~CS", PinKind::DigitalIn),
    pin("2", "DO", PinKind::DigitalOut),
    pin("3", "~WP", PinKind::DigitalIn),
    pin("4", "GND", PinKind::PowerIn),
    pin("5", "DI", PinKind::DigitalIn),
    pin("6", "CLK", PinKind::DigitalIn),
    pin("7", "~HOLD", PinKind::DigitalIn),
    pin("8", "VCC", PinKind::PowerIn),
];

/// The same facade keyed by FUNCTION, which is how a netlist transcribed from
/// a schematic names its pins — the Parallax P2-EC32MB module's `U301` among
/// them.
pub const SPI_FLASH_PINS_BY_FUNCTION: [PinDecl; 8] = [
    pin("CSn", "~CS", PinKind::DigitalIn),
    pin("DO_IO1", "DO", PinKind::DigitalOut),
    pin("WPn", "~WP", PinKind::DigitalIn),
    pin("VSS", "GND", PinKind::PowerIn),
    pin("DI_IO0", "DI", PinKind::DigitalIn),
    pin("CLK", "CLK", PinKind::DigitalIn),
    pin("HOLDn", "~HOLD", PinKind::DigitalIn),
    pin("VCC", "VCC", PinKind::PowerIn),
];

/// Shared between the sense callbacks, which the engine delivers serially from
/// one thread — so this mutex is never contended by the engine with itself,
/// only with a consumer reading the image out.
#[derive(Debug)]
struct Shared {
    flash: SpiNorFlash,
    /// The last level seen on DI. The device samples it on the clock's rising
    /// edge; the master sets it up beforehand, so the engine delivers this
    /// sense first.
    di: bool,
}

/// A serial NOR flash on a board.
pub struct SpiNorFlashComponent {
    shared: Arc<Mutex<Shared>>,
    pins: &'static [PinDecl],
}

impl std::fmt::Debug for SpiNorFlashComponent {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SpiNorFlashComponent")
            .finish_non_exhaustive()
    }
}

impl SpiNorFlashComponent {
    /// Mount `flash` as a board component, with the by-function facade a
    /// transcribed netlist uses. Call [`Self::with_pins`] for a numbered one.
    pub fn new(flash: SpiNorFlash) -> Self {
        Self {
            shared: Arc::new(Mutex::new(Shared {
                flash,
                // An undriven data line idles high on its pull-up, and a
                // master that clocks before driving DI shifts in ones.
                di: true,
            })),
            pins: &SPI_FLASH_PINS_BY_FUNCTION,
        }
    }

    /// Declare a different pin facade — [`SPI_FLASH_PINS_SOIC8`] for a netlist
    /// that identifies pins by number.
    pub fn with_pins(mut self, pins: &'static [PinDecl]) -> Self {
        self.pins = pins;
        self
    }

    /// A blank part of `capacity` bytes.
    pub fn blank(capacity: usize) -> Self {
        Self::new(SpiNorFlash::blank(capacity))
    }

    /// The backing image as programming and erase have left it — how a test
    /// checks what a loader actually wrote.
    pub fn image_bytes(&self) -> Vec<u8> {
        self.shared.lock().expect("flash mutex").flash.image_bytes()
    }

    /// Every command opcode the master has issued, in order.
    pub fn commands(&self) -> Vec<u8> {
        self.shared
            .lock()
            .expect("flash mutex")
            .flash
            .commands
            .clone()
    }

    /// Starting addresses of the reads served, oldest first.
    pub fn reads(&self) -> Vec<u32> {
        self.shared.lock().expect("flash mutex").flash.reads.clone()
    }
}

/// Publish the device's current data-out level, or release the line when the
/// part is not selected.
fn publish_do(shared: &Mutex<Shared>, data_out: &PinHandle) {
    let guard = shared.lock().expect("flash mutex");
    let drive = if guard.flash.present() {
        Some(digital_drive(if guard.flash.miso() {
            Level::High
        } else {
            Level::Low
        }))
    } else {
        // No array fitted: never drive, so a pull-up decides and a master
        // reads all-ones and concludes there is no device.
        None
    };
    drop(guard);
    data_out.set_drive(drive);
}

impl Component for SpiNorFlashComponent {
    fn pins(&self) -> &[PinDecl] {
        self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let data_out = io.pin("DO")?;

        // DI: remembered only. The device samples it at the clock edge, so a
        // level change on its own moves nothing.
        {
            let shared = Arc::clone(&self.shared);
            io.on_sense("DI", move |state| {
                if let Some(level) = level_of(state) {
                    shared.lock().expect("flash mutex").di = level == Level::High;
                }
                // A floating DI keeps its last value rather than guessing: the
                // master is mid-transfer and about to drive it again.
            })?;
        }

        // ~CS: active low. A deselect resets the command phase, which is also
        // what commits a program or erase.
        {
            let shared = Arc::clone(&self.shared);
            let data_out = data_out.clone();
            io.on_sense("~CS", move |state| {
                let Some(level) = level_of(state) else {
                    // Floating chip select is not a state the part can act on;
                    // hold, and let the engine's own diagnostics report it.
                    trace!(?state, "SPI flash: ~CS has no level; holding selection");
                    return;
                };
                shared
                    .lock()
                    .expect("flash mutex")
                    .flash
                    .set_selected(level == Level::Low);
                publish_do(&shared, &data_out);
            })?;
        }

        // CLK: the edge that moves everything. `clock` is idempotent in the
        // level, so forwarding every sense is safe even if the engine
        // republishes one.
        {
            let shared = Arc::clone(&self.shared);
            let data_out = data_out.clone();
            io.on_sense("CLK", move |state| {
                let Some(level) = level_of(state) else {
                    trace!(?state, "SPI flash: CLK has no level; no edge");
                    return;
                };
                {
                    let mut guard = shared.lock().expect("flash mutex");
                    let di = guard.di;
                    guard.flash.clock(level == Level::High, di);
                }
                publish_do(&shared, &data_out);
            })?;
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The adapter's own logic, exercised without an engine: the callbacks are
    /// plain closures over shared state, so the transforms they apply — ~CS
    /// inversion, DI latching, clock idempotence — can be checked directly.
    /// Whether the engine delivers them in the right ORDER is a system-level
    /// question and belongs to a board test.
    fn drive(shared: &Mutex<Shared>, cs_low: bool, clk_high: bool, di_high: bool) {
        let mut guard = shared.lock().expect("flash mutex");
        guard.di = di_high;
        guard.flash.set_selected(cs_low);
        guard.flash.clock(clk_high, di_high);
    }

    fn send(shared: &Mutex<Shared>, byte: u8) {
        for i in (0..8).rev() {
            let bit = (byte >> i) & 1 != 0;
            drive(shared, true, true, bit);
            drive(shared, true, false, bit);
        }
    }

    #[test]
    fn chip_select_is_inverted_by_the_adapter_not_the_model() {
        let component = SpiNorFlashComponent::blank(16);
        let shared = Arc::clone(&component.shared);
        // ~CS high means NOT selected: clocks move nothing.
        {
            let mut guard = shared.lock().expect("flash mutex");
            guard.flash.set_selected(false);
        }
        for _ in 0..8 {
            let mut guard = shared.lock().expect("flash mutex");
            guard.flash.clock(true, true);
            guard.flash.clock(false, true);
        }
        assert!(
            component.commands().is_empty(),
            "a deselected part decodes nothing"
        );

        send(&shared, 0x9F);
        assert_eq!(component.commands(), vec![0x9F], "~CS low selects it");
    }

    #[test]
    fn a_command_shifted_in_through_the_adapter_reaches_the_model() {
        let component = SpiNorFlashComponent::new(SpiNorFlash::blank(512));
        let shared = Arc::clone(&component.shared);
        send(&shared, 0x06); // write enable
        send(&shared, 0x02); // page program
        for byte in [0x00, 0x00, 0x00, 0x5A] {
            send(&shared, byte);
        }
        {
            let mut guard = shared.lock().expect("flash mutex");
            guard.flash.set_selected(false);
        }
        assert_eq!(component.commands(), vec![0x06, 0x02]);
        assert_eq!(component.image_bytes()[0], 0x5A, "the byte was programmed");
    }

    #[test]
    fn the_facade_declares_every_package_pin() {
        let component = SpiNorFlashComponent::blank(16);
        // The validator matches on `number`, so THAT is what has to line up
        // with the netlist. The P2-EC32MB transcription names U301's pins by
        // function; a KiCad export would number them.
        let ids: Vec<_> = component.pins().iter().map(|p| p.number).collect();
        assert_eq!(
            ids,
            ["CSn", "DO_IO1", "WPn", "VSS", "DI_IO0", "CLK", "HOLDn", "VCC"],
            "the default facade is the one the EC32MB netlist uses"
        );
        let numbered = SpiNorFlashComponent::blank(16).with_pins(&SPI_FLASH_PINS_SOIC8);
        let ids: Vec<_> = numbered.pins().iter().map(|p| p.number).collect();
        assert_eq!(ids, ["1", "2", "3", "4", "5", "6", "7", "8"]);
        let names: Vec<_> = numbered.pins().iter().filter_map(|p| p.name).collect();
        assert_eq!(
            names,
            ["~CS", "DO", "~WP", "GND", "DI", "CLK", "~HOLD", "VCC"],
            "the names are the same either way, so attach code does not change"
        );
    }
}
