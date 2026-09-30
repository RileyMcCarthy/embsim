//! The kinds embsim ships, as a [`Catalog`] a project file can name.
//!
//! # Board kinds
//!
//! | kind | what it is |
//! |---|---|
//! | `netlist` | any board, from its KiCad netlist export (`netlist = "board.net"`) |
//! | `p2-ec32mb` | the Parallax P2-EC32MB module ([`crate::ec32mb`]), its processor slot `U100` left for a `[[board.model]]` |
//!
//! # Part kinds
//!
//! A `[[board.model]]` names one of these for every part its key reaches.
//! Each kind is one model in `embsim-models` (or this crate) with the
//! numbers its datasheet gives; the options choose between what the model
//! already offers — a pin table, a device ID — and nothing else. Where a
//! model reads data from the board (a feedback divider, a select strap),
//! it reads it from the netlist at attach, as it does on any board.
//!
//! | kind | model | options |
//! |---|---|---|
//! | `p2` | the Propeller 2 package ([`crate::p2::P2Package`]) | `core` = `"held-in-reset"` (required) |
//! | `tg2520smn` | EPSON TCXO; frequency from the part's value or number | `pins` = `"numbered"`, `"by-function"` |
//! | `74lvc2g04` | NXP dual inverter | `pins` = `"sot363"`, `"by-function"` |
//! | `sn74lvc1g14` | TI Schmitt inverter | `pins` = `"sot23"` |
//! | `aps6404l` | AP Memory PSRAM | `pins` = `"sop8"`, `"by-function"` |
//! | `w25q128jv` | Winbond serial NOR flash, blank or holding an image | `pins` = `"soic8"`, `"by-function"`, `"spi-only"`; `id` = `"im"`, `"iq"`; `image` = a file the part holds from address 0, relative to the project file, the rest erased |
//! | `sd-card` | a card in an SD socket | `image` = card image path (required); `pins` = `"microsd"`, `"by-function"`, `"spi-only"` |
//! | `ap62301` | Diodes buck; setpoint from its feedback divider | `pins` = `"sot563"`, `"by-function"` |
//! | `ncp114` | onsemi LDO; setpoint from the part's value or number | `pins` = `"udfn4"`, `"by-function"` |
//! | `xl1509` | XLSEMI buck; version from the part's value or number | `pins` = `"sop8"` |
//! | `ucc12040` | TI isolated DC/DC; setpoint from its `SEL` strap | `pins` = `"soic16"` |
//! | `stm1061` | ST voltage detector, from its ordering code | `pins` = `"sot23"`, `"by-function"` |
//! | `6n137` | Lite-On optocoupler | — |
//! | `vo2631` | Vishay dual optocoupler | — |
//! | `iso67xx` | TI digital isolator, the member the key names | — |
//! | `ads122u04` | TI 24-bit ADC, as it comes out of reset | `pins` = `"tssop16"` |
//! | `switch` | a switch whose poles pair the part's pins, each open | `poles` = `[["1", "2"], …]` (required) |
//! | `mechanical` | a part with pads and nothing electrical | — |
//! | `boundary` | a connector, by its symbol's part name | — |
//!
//! The first `pins` value is the default: the datasheet's numbered table,
//! which is how an EDA export names pins. `by-function` is the table a
//! netlist transcribed from a schematic uses.
//!
//! # The base registry
//!
//! A `kind = "netlist"` board starts from [`StandardCatalog::base_registry`]:
//! the reference-designator fallback for a netlist with no libsource, the
//! element library ([`embsim_models::pwl_library`]), and every model above
//! under the manufacturer part numbers its datasheet and provenance name,
//! with its default pin table. Two kinds are never placed by number: the
//! processor, whose core is the thing under test, and a card socket, whose
//! card is. A board kind's own registrations (the P2-EC32MB's, keyed on its
//! netlist's values) stay with that board.
//!
//! # The guide
//!
//! [`StandardCatalog::guide`] describes every part kind for someone choosing
//! one: what the model is, the part numbers it is for, the pin tables it can
//! declare (the ones its `pins` option picks, read off the models it
//! registers) and the options it cannot go without. [`KindGuide::fit`] says
//! how a kind could be a part's model — by part number, by a table with
//! exactly the part's pins, or by pin count — which is how `embsim survey`
//! and `embsim new` name the kinds that could be a part the survey lists.

use std::collections::BTreeSet;
use std::path::Path;

use embsim_board::registry::ComponentCtor;
use embsim_board::{
    netlist, Assignment, BoardSpec, Catalog, CatalogBoard, Component, ComponentDecl, ComponentSpec,
    KeyField, ModelFacade, PartOptions, PartRegistry, PinDecl, ProjectError, SwitchPole,
};
use embsim_models::ads122u04::Config as AdcConfig;
use embsim_models::ads122u04_component::{Ads122u04Component, ADS122U04_PINS};
use embsim_models::isolation::iso67xx::{self, Iso67xx};
use embsim_models::logic_gate::{
    self, GatePin, LogicGate, LVC1G14_PINS_SOT23, LVC2G04_PINS_BY_FUNCTION, LVC2G04_PINS_SOT363,
};
use embsim_models::opto::Opto;
use embsim_models::oscillator::{self, Oscillator, TCXO_PINS_BY_FUNCTION, TCXO_PINS_NUMBERED};
use embsim_models::psram::{Psram, PsramComponent, PSRAM_PINS_BY_FUNCTION, PSRAM_PINS_SOP8};
use embsim_models::pwl_library;
use embsim_models::rail::{
    self, Rail, RailPin, AP62301_PINS_BY_FUNCTION, AP62301_PINS_SOT563, NCP114_PINS_BY_FUNCTION,
    NCP114_PINS_UDFN4, UCC12040_PINS_SOIC16, XL1509_PINS_SOP8,
};
use embsim_models::sd_card::SdCard;
use embsim_models::sd_card_component::{
    SdCardComponent, SD_CARD_PINS_BY_FUNCTION, SD_CARD_PINS_MICROSD, SD_CARD_PINS_SPI_ONLY,
};
use embsim_models::spi_flash::{
    SpiNorFlash, JEDEC_ID_W25Q128JV_IM, JEDEC_ID_W25Q128JV_IQ, W25Q128JV_CAPACITY_BYTES,
};
use embsim_models::spi_flash_component::{
    SpiNorFlashComponent, SPI_FLASH_PINS_BY_FUNCTION, SPI_FLASH_PINS_SOIC8, SPI_FLASH_PINS_SPI_ONLY,
};
use embsim_models::supervisor::{
    self, DetectorPin, VoltageDetector, STM1061_PINS_BY_FUNCTION, STM1061_PINS_SOT23,
};

use crate::ec32mb::{self, Ec32mb};
use crate::p2::{p2x8c4m64p_pins, P2Package};

/// The board kinds, part kinds and base registry this crate ships.
#[derive(Debug, Default, Clone, Copy)]
pub struct StandardCatalog;

/// The catalog board kinds.
const BOARD_KINDS: [&str; 1] = ["p2-ec32mb"];

/// One part kind: its name, what it is, and how it registers.
struct PartKind {
    name: &'static str,
    /// The model, in the module docs' words.
    summary: &'static str,
    /// Part numbers the kind is for that the base registry does not place
    /// by number (the processor): the guide names them beside the ones it
    /// does ([`known_parts`]).
    unplaced: &'static [&'static str],
    register: fn(&mut PartRegistry, &Assignment<'_>, PartOptions) -> Result<(), ProjectError>,
}

/// Every part kind, in the order the module docs list them.
const PART_KINDS: &[PartKind] = &[
    PartKind {
        name: "p2",
        summary: "the Propeller 2 package",
        unplaced: &["P2X8C4M64P"],
        register: p2_kind,
    },
    PartKind {
        name: "tg2520smn",
        summary: "EPSON TCXO; frequency from the part's value or number",
        unplaced: &[],
        register: tg2520smn_kind,
    },
    PartKind {
        name: "74lvc2g04",
        summary: "NXP dual inverter",
        unplaced: &[],
        register: lvc2g04_kind,
    },
    PartKind {
        name: "sn74lvc1g14",
        summary: "TI Schmitt inverter",
        unplaced: &[],
        register: lvc1g14_kind,
    },
    PartKind {
        name: "aps6404l",
        summary: "AP Memory PSRAM",
        unplaced: &[],
        register: aps6404l_kind,
    },
    PartKind {
        name: "w25q128jv",
        summary: "Winbond serial NOR flash, blank or holding an image",
        unplaced: &[],
        register: w25q128jv_kind,
    },
    PartKind {
        name: "sd-card",
        summary: "a card in an SD socket",
        unplaced: &[],
        register: sd_card_kind,
    },
    PartKind {
        name: "ap62301",
        summary: "Diodes buck; setpoint from its feedback divider",
        unplaced: &[],
        register: ap62301_kind,
    },
    PartKind {
        name: "ncp114",
        summary: "onsemi LDO; setpoint from the part's value or number",
        unplaced: &[],
        register: ncp114_kind,
    },
    PartKind {
        name: "xl1509",
        summary: "XLSEMI buck; version from the part's value or number",
        unplaced: &[],
        register: xl1509_kind,
    },
    PartKind {
        name: "ucc12040",
        summary: "TI isolated DC/DC; setpoint from its SEL strap",
        unplaced: &[],
        register: ucc12040_kind,
    },
    PartKind {
        name: "stm1061",
        summary: "ST voltage detector, from its ordering code",
        unplaced: &[],
        register: stm1061_kind,
    },
    PartKind {
        name: "6n137",
        summary: "Lite-On optocoupler",
        unplaced: &[],
        register: opto_6n137_kind,
    },
    PartKind {
        name: "vo2631",
        summary: "Vishay dual optocoupler",
        unplaced: &[],
        register: vo2631_kind,
    },
    PartKind {
        name: "iso67xx",
        summary: "TI digital isolator, the member the key names",
        unplaced: &[],
        register: iso67xx_kind,
    },
    PartKind {
        name: "ads122u04",
        summary: "TI 24-bit ADC, as it comes out of reset",
        unplaced: &[],
        register: ads122u04_kind,
    },
    PartKind {
        name: "switch",
        summary: "a switch whose poles pair the part's pins, each open",
        unplaced: &[],
        register: switch_kind,
    },
    PartKind {
        name: "mechanical",
        summary: "a part with pads and nothing electrical",
        unplaced: &[],
        register: mechanical_kind,
    },
    PartKind {
        name: "boundary",
        summary: "a connector, by its symbol's part name",
        unplaced: &[],
        register: boundary_kind,
    },
];

impl Catalog for StandardCatalog {
    fn board_kinds(&self) -> Vec<String> {
        BOARD_KINDS.iter().map(|kind| (*kind).to_string()).collect()
    }

    fn board(&self, spec: &BoardSpec) -> Result<CatalogBoard, ProjectError> {
        match spec.kind.as_str() {
            "p2-ec32mb" => Ok(CatalogBoard {
                netlist: netlist::parse(ec32mb::NETLIST).expect("the bundled EC32 netlist parses"),
                // The module as `Ec32mb` builds it, the processor slot left
                // for the project: `U100` is the part its survey names.
                registry: Ec32mb::new().registry(),
            }),
            other => Err(ProjectError::message(format!(
                "board {}: unknown kind {other:?}",
                spec.name
            ))),
        }
    }

    fn base_registry(&self) -> PartRegistry {
        StandardCatalog::base_registry()
    }

    fn part_kinds(&self) -> Vec<String> {
        PART_KINDS
            .iter()
            .map(|kind| kind.name.to_string())
            .collect()
    }

    fn register_part(
        &self,
        registry: &mut PartRegistry,
        assignment: &Assignment<'_>,
        options: PartOptions,
    ) -> Result<(), ProjectError> {
        let kind = PART_KINDS
            .iter()
            .find(|kind| kind.name == assignment.kind)
            .ok_or_else(|| assignment.error("not a part kind this catalog ships"))?;
        (kind.register)(registry, assignment, options)
    }

    fn component_kinds(&self) -> Vec<String> {
        Vec::new()
    }

    fn component(&self, spec: &ComponentSpec) -> Result<Box<dyn Component>, ProjectError> {
        Err(ProjectError::message(format!(
            "component {}: unknown kind {:?}; this catalog has no component kinds",
            spec.name, spec.kind
        )))
    }
}

impl StandardCatalog {
    /// The registry a `kind = "netlist"` board starts from (module docs,
    /// "The base registry").
    pub fn base_registry() -> PartRegistry {
        let mut registry = PartRegistry::new();
        // A netlist transcribed from a schematic carries no libsource; its
        // passives and connectors classify by their reference designator.
        registry.classify_unnamed_by_reference(true);
        pwl_library::register(&mut registry);
        for KnownPart { number, model, .. } in known_parts() {
            model.register(&mut registry, number);
        }
        registry
    }
}

// ============================================================
// Models
// ============================================================

/// A model ready to register: the facade every component it builds
/// declares, and its constructor.
struct Model {
    facade: ModelFacade,
    ctor: ComponentCtor,
}

impl Model {
    /// A model whose components declare the pin table `pins`.
    fn with_table(
        name: String,
        pins: &[PinDecl],
        ctor: impl Fn(&ComponentDecl) -> Box<dyn Component> + Send + Sync + 'static,
    ) -> Self {
        Self {
            facade: ModelFacade::of(name, pins),
            ctor: Box::new(ctor),
        }
    }

    /// A model whose facade is read off one component it builds — for a
    /// model that derives its pins from a configuration, and whose
    /// construction starts nothing.
    fn built(
        name: String,
        ctor: impl Fn(&ComponentDecl) -> Box<dyn Component> + Send + Sync + 'static,
    ) -> Self {
        let prototype = ctor(&prototype_decl());
        Self {
            facade: ModelFacade::of(name, prototype.pins()),
            ctor: Box::new(ctor),
        }
    }

    fn register(self, registry: &mut PartRegistry, key: &str) {
        let Model { facade, ctor } = self;
        registry.register_model(key, facade, move |decl| ctor(decl));
    }
}

/// The declaration a facade prototype is built from: none of the models
/// [`Model::built`] serves reads it.
fn prototype_decl() -> ComponentDecl {
    ComponentDecl {
        reference: String::new(),
        value: String::new(),
        footprint: String::new(),
        lib: String::new(),
        part: String::new(),
        sheetpath: "/".to_string(),
        dnp: false,
        mpn: None,
    }
}

/// The name a facade carries: the kind, and the pin table when the kind
/// offers a choice.
fn named(kind: &str, table: &str) -> String {
    format!("{kind}, pins = {table:?}")
}

fn p2_model() -> Model {
    Model::with_table(
        "p2, core = \"held-in-reset\"".to_string(),
        &p2x8c4m64p_pins(),
        |_| Box::new(P2Package::held_in_reset()),
    )
}

fn tcxo_model(config: oscillator::Config, table: (&'static str, &'static [PinDecl])) -> Model {
    Model::with_table(named("tg2520smn", table.0), table.1, move |_| {
        Box::new(Oscillator::new(config.clone()).with_pins(table.1))
    })
}

fn gate_model(
    kind: &'static str,
    config: logic_gate::Config,
    table: (&'static str, &'static [GatePin]),
) -> Model {
    Model::built(named(kind, table.0), move |_| {
        Box::new(
            LogicGate::new(config.clone(), table.1)
                .expect("a gate configuration from its datasheet is valid"),
        )
    })
}

fn psram_model(table: (&'static str, &'static [PinDecl])) -> Model {
    Model::with_table(named("aps6404l", table.0), table.1, move |_| {
        Box::new(PsramComponent::new(Psram::new()).with_pins(table.1))
    })
}

fn flash_model(id: (&'static str, [u8; 3]), table: (&'static str, &'static [PinDecl])) -> Model {
    Model::with_table(
        format!("w25q128jv, pins = {:?}, id = {:?}", table.0, id.0),
        table.1,
        move |_| {
            Box::new(
                SpiNorFlashComponent::new(
                    SpiNorFlash::blank(W25Q128JV_CAPACITY_BYTES).with_jedec_id(id.1),
                )
                .with_pins(table.1),
            )
        },
    )
}

fn rail_model(
    kind: &'static str,
    config: rail::Config,
    table: (&'static str, &'static [RailPin]),
) -> Model {
    Model::built(named(kind, table.0), move |_| {
        Box::new(Rail::new(config, table.1).expect("a rail's datasheet table carries every role"))
    })
}

fn detector_model(
    config: supervisor::Config,
    table: (&'static str, &'static [DetectorPin]),
) -> Model {
    Model::built(named("stm1061", table.0), move |_| {
        Box::new(VoltageDetector::new(config, table.1))
    })
}

fn iso_model(config: iso67xx::Config) -> Model {
    Model::built("iso67xx".to_string(), move |_| {
        Box::new(Iso67xx::new(config.clone()).expect("a family member's configuration is valid"))
    })
}

fn opto_6n137_model() -> Model {
    Model::built("6n137".to_string(), |_| Box::new(Opto::lite_on_6n137()))
}

fn vo2631_model() -> Model {
    Model::built("vo2631".to_string(), |_| Box::new(Opto::vo2631()))
}

fn adc_model() -> Model {
    Model::with_table(named("ads122u04", "tssop16"), &ADS122U04_PINS, |_| {
        Box::new(Ads122u04Component::new(AdcConfig::at_reset()))
    })
}

// ============================================================
// Pin tables
// ============================================================

const TCXO_TABLES: [(&str, &[PinDecl]); 2] = [
    ("numbered", &TCXO_PINS_NUMBERED),
    ("by-function", &TCXO_PINS_BY_FUNCTION),
];
const LVC2G04_TABLES: [(&str, &[GatePin]); 2] = [
    ("sot363", &LVC2G04_PINS_SOT363),
    ("by-function", &LVC2G04_PINS_BY_FUNCTION),
];
const LVC1G14_TABLES: [(&str, &[GatePin]); 1] = [("sot23", &LVC1G14_PINS_SOT23)];
const PSRAM_TABLES: [(&str, &[PinDecl]); 2] = [
    ("sop8", &PSRAM_PINS_SOP8),
    ("by-function", &PSRAM_PINS_BY_FUNCTION),
];
const FLASH_TABLES: [(&str, &[PinDecl]); 3] = [
    ("soic8", &SPI_FLASH_PINS_SOIC8),
    ("by-function", &SPI_FLASH_PINS_BY_FUNCTION),
    ("spi-only", &SPI_FLASH_PINS_SPI_ONLY),
];
const FLASH_IDS: [(&str, [u8; 3]); 2] =
    [("im", JEDEC_ID_W25Q128JV_IM), ("iq", JEDEC_ID_W25Q128JV_IQ)];
const SD_TABLES: [(&str, &[PinDecl]); 3] = [
    ("microsd", &SD_CARD_PINS_MICROSD),
    ("by-function", &SD_CARD_PINS_BY_FUNCTION),
    ("spi-only", &SD_CARD_PINS_SPI_ONLY),
];
const AP62301_TABLES: [(&str, &[RailPin]); 2] = [
    ("sot563", &AP62301_PINS_SOT563),
    ("by-function", &AP62301_PINS_BY_FUNCTION),
];
const NCP114_TABLES: [(&str, &[RailPin]); 2] = [
    ("udfn4", &NCP114_PINS_UDFN4),
    ("by-function", &NCP114_PINS_BY_FUNCTION),
];
const XL1509_TABLES: [(&str, &[RailPin]); 1] = [("sop8", &XL1509_PINS_SOP8)];
const UCC12040_TABLES: [(&str, &[RailPin]); 1] = [("soic16", &UCC12040_PINS_SOIC16)];
const STM1061_TABLES: [(&str, &[DetectorPin]); 2] = [
    ("sot23", &STM1061_PINS_SOT23),
    ("by-function", &STM1061_PINS_BY_FUNCTION),
];

/// Take option `name` as one of `tables`' names; the first is the default.
fn choose<T: Copy>(
    options: &mut PartOptions,
    name: &'static str,
    tables: &[(&'static str, T)],
) -> Result<(&'static str, T), ProjectError> {
    let names: Vec<&'static str> = tables.iter().map(|(name, _)| *name).collect();
    let chosen = options.choice(name, &names)?.unwrap_or(names[0]);
    Ok(*tables
        .iter()
        .find(|(name, _)| *name == chosen)
        .expect("the choice is one of the names"))
}

// ============================================================
// The base registry's numbers
// ============================================================

/// One model the base registry places by number.
struct KnownPart {
    /// The ordering code the model's datasheet provenance names.
    number: &'static str,
    /// The part kind the model is ([`PART_KINDS`]).
    kind: &'static str,
    model: Model,
}

/// Every model the base registry places by number, with the number: the
/// ordering codes each model's datasheet provenance names.
fn known_parts() -> Vec<KnownPart> {
    let known = |number: &'static str, kind: &'static str, model: Model| KnownPart {
        number,
        kind,
        model,
    };
    let tcxo = |number: &'static str| {
        let config =
            oscillator::Config::from_value(number).expect("the ordering code names its frequency");
        known(number, "tg2520smn", tcxo_model(config, TCXO_TABLES[0]))
    };
    let flash = |number: &'static str, id: usize| {
        known(
            number,
            "w25q128jv",
            flash_model(FLASH_IDS[id], FLASH_TABLES[0]),
        )
    };
    let xl1509 = |number: &'static str| {
        let config =
            rail::Config::xl1509_from_value(number).expect("the ordering code names its version");
        known(
            number,
            "xl1509",
            rail_model("xl1509", config, XL1509_TABLES[0]),
        )
    };
    let iso = |number: &'static str| {
        let config =
            iso67xx::Config::from_part_name(number).expect("the number names a family member");
        known(number, "iso67xx", iso_model(config))
    };
    let lvc1g14 = |number: &'static str| {
        known(
            number,
            "sn74lvc1g14",
            gate_model(
                "sn74lvc1g14",
                logic_gate::Config::lvc1g14(),
                LVC1G14_TABLES[0],
            ),
        )
    };
    let ucc12040 = |number: &'static str| {
        known(
            number,
            "ucc12040",
            rail_model("ucc12040", rail::Config::ucc12040(), UCC12040_TABLES[0]),
        )
    };
    let adc = |number: &'static str| known(number, "ads122u04", adc_model());
    vec![
        // EPSON TG2520SMN (oscillator.rs provenance: the ordering key).
        tcxo("TG2520SMN 20.0000M-ECGNNM3"),
        // NXP 74LVC2G04 in the GW (SOT363) package.
        known(
            "74LVC2G04GW,125",
            "74lvc2g04",
            gate_model(
                "74lvc2g04",
                logic_gate::Config::lvc2g04(),
                LVC2G04_TABLES[0],
            ),
        ),
        // TI SN74LVC1G14 in the DBV (SOT-23-5) package, reeled.
        lvc1g14("SN74LVC1G14DBVR"),
        lvc1g14("SN74LVC1G14DBVT"),
        // AP Memory APS6404L-3SQR in SOP-8.
        known("APS6404L-3SQR-ZR", "aps6404l", psram_model(PSRAM_TABLES[0])),
        // Winbond W25Q128JV: the IM option (and its reeled BOM spelling on
        // the P2-EC32MB) and the IQ option (spi_flash.rs, the JEDEC IDs).
        flash("W25Q128JVSIM", 0),
        flash("W25Q128JVSIM TR", 0),
        flash("W25Q128JVSIQ", 1),
        // Diodes AP62301 in SOT563.
        known(
            "AP62301Z6-7",
            "ap62301",
            rail_model("ap62301", rail::Config::ap62301(), AP62301_TABLES[0]),
        ),
        // onsemi NCP114, Version A, 3.3 V, UDFN4.
        known(
            "NCP114AMX330TCG",
            "ncp114",
            rail_model(
                "ncp114",
                rail::Config::ncp114_from_part_number("NCP114AMX330TCG")
                    .expect("the model cites this ordering code"),
                NCP114_TABLES[0],
            ),
        ),
        // XLSEMI XL1509 fixed versions (rail.rs: the ordering-code spelling).
        xl1509("XL1509-3.3E1"),
        xl1509("XL1509-5.0E1"),
        xl1509("XL1509-12E1"),
        // TI UCC12040 in the DVE SOIC-16 package.
        ucc12040("UCC12040DVE"),
        ucc12040("UCC12040DVER"),
        // ST STM1061N16 (supervisor.rs: Table 8 Ordering Information).
        known(
            "STM1061N16WX6F",
            "stm1061",
            detector_model(supervisor::Config::stm1061n16(), STM1061_TABLES[0]),
        ),
        // The optocouplers, by the numbers their makers sell them under.
        known("6N137", "6n137", opto_6n137_model()),
        known("VO2631", "vo2631", vo2631_model()),
        // TI ISO67xx family members, each configured from its number.
        iso("ISO6721BDR"),
        iso("ISO6731DWR"),
        iso("ISO6740DWR"),
        iso("ISO6740FDWR"),
        iso("ISO6741DWR"),
        iso("ISO6742DWR"),
        // TI ADS122U04 in TSSOP-16.
        adc("ADS122U04IPW"),
        adc("ADS122U04IPWR"),
    ]
}

// ============================================================
// The part kinds
// ============================================================

/// One configuration for every part an entry matches, derived from each
/// part by `derive`; parts that derive different ones are refused, since
/// one key takes one model.
fn one_config<C: PartialEq + Clone>(
    assignment: &Assignment<'_>,
    derive: impl Fn(&ComponentDecl) -> Option<C>,
    cannot: impl Fn(&ComponentDecl) -> String,
) -> Result<C, ProjectError> {
    let mut found: Option<(C, &str)> = None;
    for decl in assignment.parts {
        let config = derive(decl)
            .ok_or_else(|| assignment.error(format!("{}: {}", decl.reference, cannot(decl))))?;
        match &found {
            None => found = Some((config, decl.reference.as_str())),
            Some((first, first_ref)) if *first != config => {
                return Err(assignment.error(format!(
                    "{first_ref} and {} take different configurations of this model; give each \
                     its own [[board.model]] by a key that tells them apart",
                    decl.reference
                )));
            }
            Some(_) => {}
        }
    }
    found
        .map(|(config, _)| config)
        .ok_or_else(|| assignment.error("matches no part"))
}

/// A part's fields, for a message: `value "…" and mpn "…"`.
fn fields(decl: &ComponentDecl) -> String {
    match &decl.mpn {
        Some(mpn) => format!("value {:?} and mpn {mpn:?}", decl.value),
        None => format!("value {:?} (no mpn)", decl.value),
    }
}

/// The part's value, then its manufacturer part number, through `parse`.
fn value_then_mpn<C>(decl: &ComponentDecl, parse: impl Fn(&str) -> Option<C>) -> Option<C> {
    parse(&decl.value).or_else(|| decl.mpn.as_deref().and_then(&parse))
}

fn p2_kind(
    registry: &mut PartRegistry,
    assignment: &Assignment<'_>,
    mut options: PartOptions,
) -> Result<(), ProjectError> {
    options.choice("core", &["held-in-reset"])?.ok_or_else(|| {
        assignment.error(
            "options.core says what runs inside the package; this catalog seats \
                 \"held-in-reset\", the chip before it runs (a core under test, QEMU or an \
                 instruction-set simulator, is seated by the program that owns it)",
        )
    })?;
    options.finish()?;
    p2_model().register(registry, assignment.key);
    Ok(())
}

fn tg2520smn_kind(
    registry: &mut PartRegistry,
    assignment: &Assignment<'_>,
    mut options: PartOptions,
) -> Result<(), ProjectError> {
    let table = choose(&mut options, "pins", &TCXO_TABLES)?;
    options.finish()?;
    let config = one_config(
        assignment,
        |decl| value_then_mpn(decl, oscillator::Config::from_value),
        |decl| {
            format!(
                "the TCXO's frequency is read from its value or mpn (\"TG2520SMN \
                 20.0000M-ECGNNM3\" names 20 MHz), and its {} name none",
                fields(decl)
            )
        },
    )?;
    tcxo_model(config, table).register(registry, assignment.key);
    Ok(())
}

fn lvc2g04_kind(
    registry: &mut PartRegistry,
    assignment: &Assignment<'_>,
    mut options: PartOptions,
) -> Result<(), ProjectError> {
    let table = choose(&mut options, "pins", &LVC2G04_TABLES)?;
    options.finish()?;
    gate_model("74lvc2g04", logic_gate::Config::lvc2g04(), table)
        .register(registry, assignment.key);
    Ok(())
}

fn lvc1g14_kind(
    registry: &mut PartRegistry,
    assignment: &Assignment<'_>,
    mut options: PartOptions,
) -> Result<(), ProjectError> {
    let table = choose(&mut options, "pins", &LVC1G14_TABLES)?;
    options.finish()?;
    gate_model("sn74lvc1g14", logic_gate::Config::lvc1g14(), table)
        .register(registry, assignment.key);
    Ok(())
}

fn aps6404l_kind(
    registry: &mut PartRegistry,
    assignment: &Assignment<'_>,
    mut options: PartOptions,
) -> Result<(), ProjectError> {
    let table = choose(&mut options, "pins", &PSRAM_TABLES)?;
    options.finish()?;
    psram_model(table).register(registry, assignment.key);
    Ok(())
}

fn w25q128jv_kind(
    registry: &mut PartRegistry,
    assignment: &Assignment<'_>,
    mut options: PartOptions,
) -> Result<(), ProjectError> {
    let table = choose(&mut options, "pins", &FLASH_TABLES)?;
    let id = choose(&mut options, "id", &FLASH_IDS)?;
    let image = options.string("image")?;
    options.finish()?;
    let Some(image) = image else {
        flash_model(id, table).register(registry, assignment.key);
        return Ok(());
    };
    let path = assignment.dir.join(&image);
    let bytes = read_image(&path).map_err(|err| {
        assignment.error(format!("cannot read flash image {}: {err}", path.display()))
    })?;
    if bytes.len() > W25Q128JV_CAPACITY_BYTES {
        return Err(assignment.error(format!(
            "flash image {} is {} bytes; the part holds {W25Q128JV_CAPACITY_BYTES}",
            path.display(),
            bytes.len()
        )));
    }
    // The part is its full 16 MiB whatever the image's length: the rest
    // reads erased, `$FF`, as `SpiNorFlash::blank` leaves every byte — what a
    // programmer that wrote only the image would have left.
    let mut array = vec![0xFF; W25Q128JV_CAPACITY_BYTES];
    array[..bytes.len()].copy_from_slice(&bytes);
    Model::with_table(
        format!(
            "w25q128jv, pins = {:?}, id = {:?}, image = {image:?}",
            table.0, id.0
        ),
        table.1,
        move |_| {
            Box::new(
                SpiNorFlashComponent::new(
                    SpiNorFlash::with_image(array.clone()).with_jedec_id(id.1),
                )
                .with_pins(table.1),
            )
        },
    )
    .register(registry, assignment.key);
    Ok(())
}

fn sd_card_kind(
    registry: &mut PartRegistry,
    assignment: &Assignment<'_>,
    mut options: PartOptions,
) -> Result<(), ProjectError> {
    let table = choose(&mut options, "pins", &SD_TABLES)?;
    let image = options.string("image")?.ok_or_else(|| {
        assignment.error(
            "options.image is the card in the socket: a card image file, relative to the \
             project file",
        )
    })?;
    options.finish()?;
    let path = assignment.dir.join(&image);
    let blocks = read_image(&path).map_err(|err| {
        assignment.error(format!("cannot read card image {}: {err}", path.display()))
    })?;
    Model::with_table(named("sd-card", table.0), table.1, move |_| {
        Box::new(SdCardComponent::new(SdCard::with_image(blocks.clone())).with_pins(table.1))
    })
    .register(registry, assignment.key);
    Ok(())
}

fn read_image(path: &Path) -> std::io::Result<Vec<u8>> {
    std::fs::read(path)
}

fn ap62301_kind(
    registry: &mut PartRegistry,
    assignment: &Assignment<'_>,
    mut options: PartOptions,
) -> Result<(), ProjectError> {
    let table = choose(&mut options, "pins", &AP62301_TABLES)?;
    options.finish()?;
    rail_model("ap62301", rail::Config::ap62301(), table).register(registry, assignment.key);
    Ok(())
}

fn ncp114_kind(
    registry: &mut PartRegistry,
    assignment: &Assignment<'_>,
    mut options: PartOptions,
) -> Result<(), ProjectError> {
    let table = choose(&mut options, "pins", &NCP114_TABLES)?;
    options.finish()?;
    let config = one_config(
        assignment,
        |decl| {
            rail::Config::ncp114_from_value(&decl.value).or_else(|| {
                decl.mpn
                    .as_deref()
                    .and_then(rail::Config::ncp114_from_part_number)
            })
        },
        |decl| {
            format!(
                "the LDO's output is read from its value (\"LDO 300mA, 3.3V\" names 3.3 V) or \
                 from an ordering code the model cites (NCP114AMX330TCG), and its {} name \
                 neither",
                fields(decl)
            )
        },
    )?;
    rail_model("ncp114", config, table).register(registry, assignment.key);
    Ok(())
}

fn xl1509_kind(
    registry: &mut PartRegistry,
    assignment: &Assignment<'_>,
    mut options: PartOptions,
) -> Result<(), ProjectError> {
    let table = choose(&mut options, "pins", &XL1509_TABLES)?;
    options.finish()?;
    let config = one_config(
        assignment,
        |decl| value_then_mpn(decl, rail::Config::xl1509_from_value),
        |decl| {
            format!(
                "the buck's fixed version is read from its value or mpn (\"XL1509-5V\", \
                 \"XL1509-3.3E1\"), and its {} name none",
                fields(decl)
            )
        },
    )?;
    rail_model("xl1509", config, table).register(registry, assignment.key);
    Ok(())
}

fn ucc12040_kind(
    registry: &mut PartRegistry,
    assignment: &Assignment<'_>,
    mut options: PartOptions,
) -> Result<(), ProjectError> {
    let table = choose(&mut options, "pins", &UCC12040_TABLES)?;
    options.finish()?;
    rail_model("ucc12040", rail::Config::ucc12040(), table).register(registry, assignment.key);
    Ok(())
}

fn stm1061_kind(
    registry: &mut PartRegistry,
    assignment: &Assignment<'_>,
    mut options: PartOptions,
) -> Result<(), ProjectError> {
    let table = choose(&mut options, "pins", &STM1061_TABLES)?;
    options.finish()?;
    let config = one_config(
        assignment,
        |decl| {
            [
                decl.mpn.as_deref(),
                Some(decl.part.as_str()),
                Some(decl.value.as_str()),
            ]
            .into_iter()
            .flatten()
            .any(|field| field.trim().starts_with("STM1061N16"))
            .then(supervisor::Config::stm1061n16)
        },
        |decl| {
            format!(
                "the detector's threshold is read from its ordering code, and the model cites \
                 the STM1061N16's; its {} name no STM1061N16…",
                fields(decl)
            )
        },
    )?;
    detector_model(config, table).register(registry, assignment.key);
    Ok(())
}

fn opto_6n137_kind(
    registry: &mut PartRegistry,
    assignment: &Assignment<'_>,
    options: PartOptions,
) -> Result<(), ProjectError> {
    options.finish()?;
    opto_6n137_model().register(registry, assignment.key);
    Ok(())
}

fn vo2631_kind(
    registry: &mut PartRegistry,
    assignment: &Assignment<'_>,
    options: PartOptions,
) -> Result<(), ProjectError> {
    options.finish()?;
    vo2631_model().register(registry, assignment.key);
    Ok(())
}

fn iso67xx_kind(
    registry: &mut PartRegistry,
    assignment: &Assignment<'_>,
    options: PartOptions,
) -> Result<(), ProjectError> {
    options.finish()?;
    let config = iso67xx::Config::from_part_name(assignment.key).ok_or_else(|| {
        assignment.error(
            "the isolator's family member is read from the part number it is assigned by \
             (ISO6720, ISO6721, ISO6721R, ISO6731, ISO6740, ISO6741 or ISO6742, an F after \
             the digits for the fail-safe-low option, then the package: \"ISO6741DWR\")",
        )
    })?;
    iso_model(config).register(registry, assignment.key);
    Ok(())
}

fn ads122u04_kind(
    registry: &mut PartRegistry,
    assignment: &Assignment<'_>,
    mut options: PartOptions,
) -> Result<(), ProjectError> {
    choose(&mut options, "pins", &[("tssop16", ())])?;
    options.finish()?;
    adc_model().register(registry, assignment.key);
    Ok(())
}

fn switch_kind(
    registry: &mut PartRegistry,
    assignment: &Assignment<'_>,
    mut options: PartOptions,
) -> Result<(), ProjectError> {
    let poles = options
        .pairs("poles")?
        .filter(|poles| !poles.is_empty())
        .ok_or_else(|| {
            assignment.error(
                "options.poles pairs the part's pins into poles, each open until a [[switch]] \
                 closes it: poles = [[\"1\", \"2\"]]; poles are numbered from 0 in this order",
            )
        })?;
    options.finish()?;
    registry.register_switch(
        assignment.key,
        poles
            .into_iter()
            .map(|(a, b)| SwitchPole::open(a, b))
            .collect(),
    );
    Ok(())
}

fn mechanical_kind(
    registry: &mut PartRegistry,
    assignment: &Assignment<'_>,
    options: PartOptions,
) -> Result<(), ProjectError> {
    options.finish()?;
    registry.register_mechanical(assignment.key);
    Ok(())
}

fn boundary_kind(
    registry: &mut PartRegistry,
    assignment: &Assignment<'_>,
    options: PartOptions,
) -> Result<(), ProjectError> {
    options.finish()?;
    if assignment.by != KeyField::Part {
        return Err(assignment.error(
            "a connector is declared by its symbol's part name (part = \"…\"); a part with no \
             part name is a connector by its reference designator (J…, P…) already",
        ));
    }
    registry.register_boundary(assignment.key);
    Ok(())
}

// ============================================================
// The guide
// ============================================================

/// A pin table a part kind's model can declare.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PinTable {
    /// What names the table: its `pins` option value (`"soic8"`) when
    /// [`Self::option`] is set, otherwise the part number whose model
    /// declares it (`"ISO6741DWR"`, `"P2X8C4M64P"`).
    pub name: &'static str,
    /// Whether `pins = name` among the kind's options picks this table.
    pub option: bool,
    /// The pin identities the table declares, in declaration order.
    pub pins: Vec<String>,
}

/// An option a part kind cannot be registered without.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RequiredOption {
    /// The option's name (`"core"`).
    pub name: &'static str,
    /// A value of the shape the option takes, as TOML (`"\"held-in-reset\""`).
    pub example: &'static str,
    /// What the option says, in a phrase.
    pub means: &'static str,
}

/// A part kind as someone choosing one reads it: what the model is, the
/// part numbers it is for, the pin tables it can declare, and the options
/// it needs.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct KindGuide {
    /// The kind's name, as a `[[board.model]]` gives it.
    pub name: &'static str,
    /// The model, in a phrase.
    pub summary: &'static str,
    /// The part numbers the kind is for: every one the base registry
    /// places it by, and the processor's, which it never places.
    pub numbers: Vec<&'static str>,
    /// The pin tables its model can declare. Empty for a kind that takes
    /// the part's own pins, whatever they are (`switch`, `mechanical`,
    /// `boundary`).
    pub tables: Vec<PinTable>,
    /// The options it cannot be registered without.
    pub required: Vec<RequiredOption>,
}

/// How a part kind could be a part's model, strongest first.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Fit {
    /// One of the part's keys and a number the kind is for name the same
    /// part: one is the other, or the other with an ordering suffix
    /// (`ADS122U04` and `ADS122U04IPW`), compared on letters and digits.
    Number(&'static str),
    /// The pin table declares exactly the part's pins.
    Pins(&'static str),
    /// The pin table declares as many pins as the part has, by other names.
    PinCount(&'static str),
}

/// The fewest letters and digits a key must have to be compared with a
/// part number: short enough for `6N137`, long enough that a value such
/// as `10k` or `P2` names no part.
const MIN_NUMBER_STEM: usize = 5;

/// Letters and digits, upper-cased: `"W25Q128JVSIM TR"` is `"W25Q128JVSIMTR"`.
fn number_stem(text: &str) -> String {
    text.chars()
        .filter(char::is_ascii_alphanumeric)
        .map(|c| c.to_ascii_uppercase())
        .collect()
}

impl KindGuide {
    /// The strongest way this kind could be the model of a part whose
    /// keys (part name, manufacturer part number, value) and pins are
    /// these ([`Fit`]); `None` when it fits by none of them.
    pub fn fit(&self, keys: &[&str], pins: &[&str]) -> Option<Fit> {
        let keys: Vec<String> = keys
            .iter()
            .map(|key| number_stem(key))
            .filter(|key| key.len() >= MIN_NUMBER_STEM)
            .collect();
        for number in &self.numbers {
            let stem = number_stem(number);
            if keys
                .iter()
                .any(|key| stem.starts_with(key.as_str()) || key.starts_with(stem.as_str()))
            {
                return Some(Fit::Number(number));
            }
        }
        if let Some(table) = self.table_with(pins) {
            return Some(Fit::Pins(table.name));
        }
        self.tables
            .iter()
            .find(|table| table.pins.len() == pins.len())
            .map(|table| Fit::PinCount(table.name))
    }

    /// The table that declares exactly `pins`, compared as sets.
    pub fn table_with(&self, pins: &[&str]) -> Option<&PinTable> {
        let wanted: BTreeSet<&str> = pins.iter().copied().collect();
        self.tables.iter().find(|table| {
            table.pins.len() == wanted.len()
                && table.pins.iter().all(|pin| wanted.contains(pin.as_str()))
        })
    }
}

impl StandardCatalog {
    /// Every part kind, in the module docs' order, as someone choosing one
    /// reads it ([`KindGuide`]). The tables are the ones each kind's
    /// `register` offers; `every_option_table_is_the_one_its_kind_registers`
    /// holds the two together.
    pub fn guide() -> Vec<KindGuide> {
        let known = known_parts();
        PART_KINDS
            .iter()
            .map(|kind| {
                let mut numbers: Vec<&'static str> = kind.unplaced.to_vec();
                numbers.extend(
                    known
                        .iter()
                        .filter(|part| part.kind == kind.name)
                        .map(|part| part.number),
                );
                KindGuide {
                    name: kind.name,
                    summary: kind.summary,
                    numbers,
                    tables: kind_tables(kind.name, &known),
                    required: required_options(kind.name),
                }
            })
            .collect()
    }
}

/// A `pins` option's tables, from what a model built with each declares.
fn option_tables<T: Copy>(
    tables: &[(&'static str, T)],
    facade: impl Fn((&'static str, T)) -> ModelFacade,
) -> Vec<PinTable> {
    tables
        .iter()
        .map(|&table| PinTable {
            name: table.0,
            option: true,
            pins: facade(table).pins,
        })
        .collect()
}

/// The tables of the kind `name`, as its `register` function builds them.
fn kind_tables(name: &str, known: &[KnownPart]) -> Vec<PinTable> {
    let fixed = |name: &'static str, model: Model| {
        vec![PinTable {
            name,
            option: false,
            pins: model.facade.pins,
        }]
    };
    let decls = |tables: &[(&'static str, &'static [PinDecl])]| {
        option_tables(tables, |(name, pins)| ModelFacade::of(name, pins))
    };
    match name {
        "p2" => fixed("P2X8C4M64P", p2_model()),
        "tg2520smn" => decls(&TCXO_TABLES),
        "74lvc2g04" => option_tables(&LVC2G04_TABLES, |table| {
            gate_model("74lvc2g04", logic_gate::Config::lvc2g04(), table).facade
        }),
        "sn74lvc1g14" => option_tables(&LVC1G14_TABLES, |table| {
            gate_model("sn74lvc1g14", logic_gate::Config::lvc1g14(), table).facade
        }),
        "aps6404l" => decls(&PSRAM_TABLES),
        "w25q128jv" => decls(&FLASH_TABLES),
        "sd-card" => decls(&SD_TABLES),
        "ap62301" => option_tables(&AP62301_TABLES, |table| {
            rail_model("ap62301", rail::Config::ap62301(), table).facade
        }),
        "ncp114" => option_tables(&NCP114_TABLES, |table| {
            let config = rail::Config::ncp114_from_part_number("NCP114AMX330TCG")
                .expect("the model cites this ordering code");
            rail_model("ncp114", config, table).facade
        }),
        "xl1509" => option_tables(&XL1509_TABLES, |table| {
            let config = rail::Config::xl1509_from_value("XL1509-3.3E1")
                .expect("the ordering code names its version");
            rail_model("xl1509", config, table).facade
        }),
        "ucc12040" => option_tables(&UCC12040_TABLES, |table| {
            rail_model("ucc12040", rail::Config::ucc12040(), table).facade
        }),
        "stm1061" => option_tables(&STM1061_TABLES, |table| {
            detector_model(supervisor::Config::stm1061n16(), table).facade
        }),
        "6n137" => fixed("6N137", opto_6n137_model()),
        "vo2631" => fixed("VO2631", vo2631_model()),
        // One member's pins per number the family is placed by.
        "iso67xx" => known
            .iter()
            .filter(|part| part.kind == "iso67xx")
            .map(|part| PinTable {
                name: part.number,
                option: false,
                pins: part.model.facade.pins.clone(),
            })
            .collect(),
        "ads122u04" => vec![PinTable {
            name: "tssop16",
            option: true,
            pins: ModelFacade::of("", &ADS122U04_PINS).pins,
        }],
        _ => Vec::new(),
    }
}

/// The options the kind `name` refuses to register without.
fn required_options(name: &str) -> Vec<RequiredOption> {
    match name {
        "p2" => vec![RequiredOption {
            name: "core",
            example: "\"held-in-reset\"",
            means: "what runs inside the package; \"held-in-reset\" is the chip before it runs",
        }],
        "sd-card" => vec![RequiredOption {
            name: "image",
            example: "\"card.img\"",
            means: "the card in the socket: a card image file, relative to the project file",
        }],
        "switch" => vec![RequiredOption {
            name: "poles",
            example: "[[\"1\", \"2\"]]",
            means: "the part's pins paired into poles, each open until a [[switch]] closes it",
        }],
        _ => Vec::new(),
    }
}

#[cfg(test)]
mod tests {
    use rstest::rstest;
    use vibes_behaviour::{behaviour, expect, Test};

    use super::*;

    /// Every model the catalog places by number states the facade its
    /// components declare: the survey's pin check is the build's.
    #[rstest]
    fn every_known_part_states_the_pins_its_component_declares() {
        for KnownPart { number, model, .. } in known_parts() {
            if number.starts_with("ADS122U04") {
                // Construction starts the protocol thread; the facade is the
                // component's own static table.
                assert_eq!(model.facade.pins, ModelFacade::of("", &ADS122U04_PINS).pins);
                continue;
            }
            let component = (model.ctor)(&prototype_decl());
            let pins: Vec<String> = component
                .pins()
                .iter()
                .map(|pin| pin.number.to_string())
                .collect();
            assert_eq!(model.facade.pins, pins, "{number}");
        }
    }

    fn decl(number: &str) -> ComponentDecl {
        ComponentDecl {
            reference: "U1".to_string(),
            value: number.to_string(),
            mpn: Some(number.to_string()),
            ..prototype_decl()
        }
    }

    /// Register `kind` under `key` with `options`, for the part `decl`.
    fn register(
        registry: &mut PartRegistry,
        kind: &str,
        key: &str,
        decl: &ComponentDecl,
        options: &str,
        dir: &Path,
    ) -> Result<(), ProjectError> {
        let parts = [decl];
        let assignment = Assignment {
            board: "B",
            by: KeyField::Mpn,
            key,
            kind,
            parts: &parts,
            dir,
        };
        let table: toml::Table = toml::from_str(options).expect("the options parse");
        StandardCatalog.register_part(
            registry,
            &assignment,
            PartOptions::new(assignment.context(), table),
        )
    }

    /// A card image for the `sd-card` kind, which cannot register without
    /// one.
    fn card_image() -> std::path::PathBuf {
        let path = std::env::temp_dir().join(format!(
            "embsim-catalog-guide-card-{}.img",
            std::process::id()
        ));
        std::fs::write(&path, vec![0u8; 512]).expect("the temp dir is writable");
        path
    }

    #[rstest]
    fn every_option_table_is_the_one_its_kind_registers() {
        behaviour!(Test {
            id: "catalog.guide-tables",
            covers: Some("boards/src/catalog.rs#StandardCatalog::guide"),
            given: "every pin table the catalog's guide lists as a choice for a part kind, \
                    each registered for a part through that kind's own option",
        });
        expect!(
            "same-pins",
            "the model registered declares exactly the pins the guide lists for that table, \
             in the same order",
            "a starter project picks a table because the guide says it has the netlist's \
             pins, so the guide and the registration must be one list"
        );
        let image = card_image();
        let dir = image
            .parent()
            .expect("a temp file has a directory")
            .to_path_buf();
        let mut checked = 0;
        for kind in StandardCatalog::guide() {
            let key = kind.numbers.first().copied().unwrap_or("PART-1");
            let part = decl(key);
            for table in kind.tables.iter().filter(|table| table.option) {
                let mut options = format!("pins = {:?}\n", table.name);
                if kind.name == "sd-card" {
                    let file = image.file_name().expect("a file").to_string_lossy();
                    options.push_str(&format!("image = {file:?}\n"));
                }
                let mut registry = PartRegistry::new();
                register(&mut registry, kind.name, key, &part, &options, &dir)
                    .unwrap_or_else(|err| panic!("{} {}: {err}", kind.name, table.name));
                let facade = registry
                    .facade(&part)
                    .unwrap_or_else(|| panic!("{} states its pins", kind.name));
                assert_eq!(facade.pins, table.pins, "{} {}", kind.name, table.name);
                checked += 1;
            }
        }
        let _ = std::fs::remove_file(&image);
        // Two tables for each of six kinds, three for the flash and the
        // card, one for each of four.
        assert_eq!(checked, 22);
    }

    /// The P2's 86 pins as the P2-EC32MB's netlist gives them.
    fn p2_pins() -> Vec<String> {
        p2x8c4m64p_pins()
            .iter()
            .map(|pin| pin.number.to_string())
            .collect()
    }

    #[rstest]
    #[case::ordering_suffix("ads122u04", &["ADS122U04"], Some(Fit::Number("ADS122U04IPW")))]
    #[case::reel_suffix("w25q128jv", &["W25Q128JVSIQ TR"], Some(Fit::Number("W25Q128JVSIQ")))]
    #[case::processor("p2", &["P2X8C4M64P"], Some(Fit::Number("P2X8C4M64P")))]
    #[case::too_short_to_name_a_part("ads122u04", &["ADS"], None)]
    fn a_part_number_fits_the_kind_it_shares_a_stem_with(
        #[case] kind: &str,
        #[case] keys: &[&str],
        #[case] fit: Option<Fit>,
    ) {
        behaviour!(Test {
            id: "catalog.fit-by-number",
            covers: Some("boards/src/catalog.rs#KindGuide::fit"),
            given: "a part whose part name, number or value is one of a kind's part numbers \
                    with or without its ordering suffix, or a stem too short to name a part",
        });
        expect!(
            "stem-shared",
            "the kind fits by that part number when one is the other plus a suffix, on letters \
             and digits; a stem under five characters fits nothing",
            "an ordering code adds package, reel and temperature letters to the part's number"
        );
        let guide = StandardCatalog::guide();
        let kind = guide
            .iter()
            .find(|guide| guide.name == kind)
            .expect("a kind the catalog ships");
        assert_eq!(kind.fit(keys, &[]), fit);
    }

    #[rstest]
    fn a_part_with_no_number_fits_by_its_pins_then_by_their_count() {
        behaviour!(Test {
            id: "catalog.fit-by-pins",
            covers: Some("boards/src/catalog.rs#KindGuide::fit"),
            given: "parts whose keys name no part number: one with the processor package's 86 \
                    pins, one with eight pins named as no table names them",
        });
        expect!(
            "exact-pins",
            "the processor's pins fit the P2 kind by its pin table"
        );
        expect!(
            "pin-count",
            "the eight pins fit the PSRAM kind only by count, through its eight-pin table"
        );
        let guide = StandardCatalog::guide();
        let find = |name: &str| {
            guide
                .iter()
                .find(|kind| kind.name == name)
                .expect("a kind the catalog ships")
        };
        let pins = p2_pins();
        let pins: Vec<&str> = pins.iter().map(String::as_str).collect();
        assert_eq!(
            find("p2").fit(&["Propeller"], &pins),
            Some(Fit::Pins("P2X8C4M64P"))
        );
        let switch = [
            "1_ON", "1_OFF", "2_ON", "2_OFF", "3_ON", "3_OFF", "4_ON", "4_OFF",
        ];
        assert_eq!(
            find("aps6404l").fit(&["DIP Switch"], &switch),
            Some(Fit::PinCount("sop8"))
        );
        assert_eq!(find("p2").fit(&["DIP Switch"], &switch), None);
    }

    #[rstest]
    #[case::too_big(W25Q128JV_CAPACITY_BYTES + 1, "the part holds 16777216")]
    fn a_flash_image_larger_than_the_part_is_refused(#[case] len: usize, #[case] says: &str) {
        let path = std::env::temp_dir().join(format!(
            "embsim-catalog-flash-{}-{len}.bin",
            std::process::id()
        ));
        std::fs::write(&path, vec![0xA5u8; len]).expect("the temp dir is writable");
        let part = decl("W25Q128JVSIM");
        let file = path
            .file_name()
            .expect("a file")
            .to_string_lossy()
            .into_owned();
        let err = register(
            &mut PartRegistry::new(),
            "w25q128jv",
            "W25Q128JVSIM",
            &part,
            &format!("image = {file:?}\n"),
            path.parent().expect("a directory"),
        )
        .expect_err("the image does not fit");
        let _ = std::fs::remove_file(&path);
        assert!(err.to_string().contains(says), "{err}");
        assert!(
            err.to_string().contains(&format!("is {len} bytes")),
            "{err}"
        );
    }

    #[rstest]
    fn a_flash_image_is_read_relative_to_the_project_and_named_in_the_model() {
        let path = std::env::temp_dir().join(format!(
            "embsim-catalog-flash-{}-small.bin",
            std::process::id()
        ));
        std::fs::write(&path, [1u8, 2, 3]).expect("the temp dir is writable");
        let part = decl("W25Q128JVSIM");
        let file = path
            .file_name()
            .expect("a file")
            .to_string_lossy()
            .into_owned();
        let mut registry = PartRegistry::new();
        register(
            &mut registry,
            "w25q128jv",
            "W25Q128JVSIM",
            &part,
            &format!("image = {file:?}\npins = \"by-function\"\n"),
            path.parent().expect("a directory"),
        )
        .expect("the image fits");
        let _ = std::fs::remove_file(&path);
        let facade = registry.facade(&part).expect("the flash states its pins");
        assert!(
            facade.model.contains(&format!("image = {file:?}")),
            "{}",
            facade.model
        );
        assert_eq!(
            facade.pins,
            ModelFacade::of("", &SPI_FLASH_PINS_BY_FUNCTION).pins
        );

        let err = register(
            &mut PartRegistry::new(),
            "w25q128jv",
            "W25Q128JVSIM",
            &part,
            "image = \"no-such-image.bin\"\n",
            Path::new("/nonexistent-embsim-dir"),
        )
        .expect_err("no such file");
        assert!(
            err.to_string()
                .contains("cannot read flash image /nonexistent-embsim-dir/no-such-image.bin"),
            "{err}"
        );
    }
}
