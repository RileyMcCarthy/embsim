//! What a part kind is: the guide a catalog gives for each of its part
//! kinds, and the check every part an entry reaches passes before a kind
//! registers on it.
//!
//! A kind says what a part is (`DESIGN.md` rule 1), so every part kind
//! states what a part has to be for it to seat there ([`Named`]): a model's
//! kind seats only on a part one of whose keys names its part family; a
//! connector kind only on a connector, by its reference designator or its
//! symbol; a switch kind only on a switch or jumper, by its designator, its
//! symbol or its name; and a kind with nothing electrical only on a part
//! whose pins sit on one net at most. [`crate::Project`] checks it for every
//! part a `[[board.model]]` reaches, whichever catalog the kind comes from
//! ([`KindGuide::check`]), before the catalog registers anything. A part
//! none of them is needs a model.
//!
//! The guide also says, for someone choosing a kind, what the model is, the
//! part numbers it is for, the pin tables it can declare and the options it
//! cannot go without; [`KindGuide::fit`] says how a kind is a part's model —
//! by part number, or by the part family its keys name — which is how
//! `embsim survey` and `embsim new` name the kind a part the survey lists
//! is. Pins alone name no kind.

use std::borrow::Cow;
use std::collections::BTreeSet;

use crate::netlist::ComponentDecl;
use crate::project::{Assignment, ProjectError};
use crate::registry::normalize_part;

// ============================================================
// What a part is
// ============================================================

/// What a part has to be for a kind to seat there: a kind says what the
/// part is (`DESIGN.md` rule 1), so a kind never seats on a part that is
/// something else, whatever its pins. A part none of them fits is a part
/// that needs a model (`PROJECTS.md` §7).
///
/// Non-exhaustive: a way for a part to say what it is may be added without
/// breaking a catalog that matches on these.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub enum Named {
    /// One of the part's part name, manufacturer part number or value names
    /// one of these part families, compared on letters and digits: the part
    /// is a member of the family the model's datasheet describes
    /// ([`Self::family`] makes one).
    Family(Vec<Cow<'static, str>>),
    /// A connector, by its reference designator or its symbol
    /// ([`is_connector`]).
    Connector,
    /// A switch or a jumper, by its reference designator or its symbol
    /// ([`is_switch`]).
    Switch,
    /// A part with nothing electrical: its pins sit on one net at most, so
    /// it joins nothing. A part whose pins join two nets carries current
    /// between them, and that is behaviour a model has to say.
    OneNet,
}

/// The reference designators a connector is drawn with: `J` (a jack or a
/// connector), `P` (a plug), `CN`.
pub const CONNECTOR_DESIGNATORS: [&str; 3] = ["J", "P", "CN"];

/// The reference designators a switch or jumper is drawn with: `S`, `SW`,
/// `JP`, `SJ` (a solder jumper).
pub const SWITCH_DESIGNATORS: [&str; 4] = ["S", "SW", "JP", "SJ"];

/// Words a part's own name says it is a switch or jumper with, in any case:
/// a netlist transcribed from a schematic has no symbol, and its value is
/// the part's name (the P2-EC32MB's `J101`, "Solder Link Pads").
pub const SWITCH_WORDS: [&str; 3] = ["switch", "jumper", "solder link"];

/// A reference designator's class letters: the letters before its first
/// digit, upper-cased (`"SW"` for `SW3`, `"U"` for `U24`).
fn designator(reference: &str) -> String {
    reference
        .chars()
        .take_while(char::is_ascii_alphabetic)
        .map(|c| c.to_ascii_uppercase())
        .collect()
}

/// Whether a part is a connector by what the board says of it: a connector
/// designator ([`CONNECTOR_DESIGNATORS`]) or a `Conn…` symbol.
pub fn is_connector(reference: &str, part: &str) -> bool {
    CONNECTOR_DESIGNATORS.contains(&designator(reference).as_str()) || part.starts_with("Conn")
}

/// Whether a part is a switch or jumper by what the board says of it: a
/// switch designator ([`SWITCH_DESIGNATORS`]), a `SW_…` symbol, or a symbol
/// name or value that says so ([`SWITCH_WORDS`]).
pub fn is_switch(reference: &str, part: &str, value: &str) -> bool {
    let says = |text: &str| {
        let text = text.to_lowercase();
        SWITCH_WORDS.iter().any(|word| text.contains(word))
    };
    SWITCH_DESIGNATORS.contains(&designator(reference).as_str())
        || part.starts_with("SW_")
        || says(part)
        || says(value)
}

/// Whether one of `keys` names one of `families`: a key's letters and
/// digits contain the family's ([`number_stem`]). A key shorter than
/// [`MIN_NUMBER_STEM`] names no part.
fn names_a_family<'f>(keys: &[&str], families: &'f [Cow<'static, str>]) -> Option<&'f str> {
    let keys: Vec<String> = keys
        .iter()
        .map(|key| number_stem(key))
        .filter(|key| key.len() >= MIN_NUMBER_STEM)
        .collect();
    families.iter().map(Cow::as_ref).find(|family| {
        let family = number_stem(family);
        keys.iter().any(|key| key.contains(family.as_str()))
    })
}

impl Named {
    /// A part of one of `families`: `Named::family(["EX-BUF1"])`.
    pub fn family<I, S>(families: I) -> Self
    where
        I: IntoIterator<Item = S>,
        S: Into<Cow<'static, str>>,
    {
        Named::Family(families.into_iter().map(Into::into).collect())
    }

    /// What the kind is for, as the `PROJECTS.md` table and an error say
    /// it.
    pub fn describe(&self) -> String {
        match self {
            Named::Family(families) => {
                let names: Vec<&str> = families.iter().map(Cow::as_ref).collect();
                let listed = match names.as_slice() {
                    [one] => (*one).to_string(),
                    [rest @ .., last] => format!("{} or {last}", rest.join(", ")),
                    [] => String::new(),
                };
                format!("a part whose part name, mpn or value contains {listed}")
            }
            Named::Connector => "a connector: designator J, P or CN, or a Conn… symbol".to_string(),
            Named::Switch => {
                "a switch or jumper: designator S, SW, JP or SJ, a SW_… symbol, or a name that \
                 says switch, jumper or solder link"
                    .to_string()
            }
            Named::OneNet => "a part whose pads sit on one net at most".to_string(),
        }
    }
}

/// `part name "X", mpn "Y" and value "Z"`: the keys a part carries.
fn keys_phrase(part: &str, decl: &ComponentDecl) -> String {
    let mut keys = Vec::new();
    if !part.is_empty() {
        keys.push(format!("part name {part:?}"));
    }
    if let Some(mpn) = &decl.mpn {
        keys.push(format!("mpn {mpn:?}"));
    }
    keys.push(format!("value {:?}", decl.value));
    match keys.as_slice() {
        [one] => one.clone(),
        [rest @ .., last] => format!("{} and {last}", rest.join(", ")),
        [] => String::new(),
    }
}

/// `U1 (designator U, symbol "ADS122U04")`: what the board says a part is.
fn symbol_phrase(part: &str, decl: &ComponentDecl) -> String {
    let designator = designator(&decl.reference);
    if part.is_empty() {
        format!(
            "{} (designator {designator}, no symbol name)",
            decl.reference
        )
    } else {
        format!(
            "{} (designator {designator}, symbol {part:?})",
            decl.reference
        )
    }
}

/// The kinds that need no model a part may take, each with what makes it
/// one: by its reference designator, its symbol's part name and the number
/// of nets its pins sit on — the checks those kinds make when an entry
/// names them ([`KindGuide::check`]).
pub fn kinds_without_a_model(
    reference: &str,
    part: &str,
    value: &str,
    nets: usize,
) -> Vec<(&'static str, String)> {
    let mut kinds = Vec::new();
    let designator = designator(reference);
    if is_switch(reference, part, value) {
        let why = if SWITCH_DESIGNATORS.contains(&designator.as_str()) {
            format!("its designator {designator}")
        } else if part.is_empty() {
            format!("its value {value:?}")
        } else {
            format!("its symbol {part:?}")
        };
        kinds.push(("switch", why));
    }
    if is_connector(reference, part) {
        let why = if CONNECTOR_DESIGNATORS.contains(&designator.as_str()) {
            format!("its designator {designator}")
        } else {
            format!("its symbol {part:?}")
        };
        kinds.push(("boundary", why));
    }
    if nets <= 1 {
        kinds.push((
            "mechanical",
            if nets == 0 {
                "it has no pins on a net".to_string()
            } else {
                "its pins sit on one net".to_string()
            },
        ));
    }
    kinds
}

// ============================================================
// The guide
// ============================================================

/// A pin table a part kind's model can declare.
///
/// Non-exhaustive, as every struct a catalog fills in: build one with
/// [`Self::option()`] or [`Self::fixed()`], so a field added later breaks
/// no catalog.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub struct PinTable {
    /// What names the table: its `pins` option value (`"soic8"`) when
    /// `option` is set, otherwise the part number whose model declares it
    /// (`"ISO6741DWR"`, `"P2X8C4M64P"`).
    pub name: Cow<'static, str>,
    /// Whether `pins = name` among the kind's options picks this table.
    pub option: bool,
    /// The pin identities the table declares, in declaration order.
    pub pins: Vec<String>,
}

impl PinTable {
    /// A table the kind's `pins` option picks by `name`.
    pub fn option(name: impl Into<Cow<'static, str>>, pins: Vec<String>) -> Self {
        Self {
            name: name.into(),
            option: true,
            pins,
        }
    }

    /// The one table the model of the part number `number` declares.
    pub fn fixed(number: impl Into<Cow<'static, str>>, pins: Vec<String>) -> Self {
        Self {
            name: number.into(),
            option: false,
            pins,
        }
    }
}

/// What values an option takes, beyond what its [`RequiredOption::means`]
/// says.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
#[non_exhaustive]
pub enum OptionValues {
    /// What `means` says, and no more.
    #[default]
    Described,
    /// One of the core kinds of the set the kind is in (the `p2` package's
    /// `core`): a set describing the kind names each core it holds after
    /// `means`, so a kind needs no list of cores of its own.
    CoreKind,
}

/// An option a kind cannot be built without.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub struct RequiredOption {
    /// The option's name (`"core"`).
    pub name: Cow<'static, str>,
    /// A value of the shape the option takes, as TOML (`"\"held-in-reset\""`).
    pub example: Cow<'static, str>,
    /// What the option says, in a phrase. A set of catalogs describing an
    /// option of [`OptionValues::CoreKind`] adds the cores it holds.
    pub means: Cow<'static, str>,
    /// What values it takes.
    pub values: OptionValues,
}

impl RequiredOption {
    /// The option `name`, with an `example` value as TOML writes it and
    /// what it `means`.
    pub fn new(
        name: impl Into<Cow<'static, str>>,
        example: impl Into<Cow<'static, str>>,
        means: impl Into<Cow<'static, str>>,
    ) -> Self {
        Self {
            name: name.into(),
            example: example.into(),
            means: means.into(),
            values: OptionValues::Described,
        }
    }

    /// The same option, taking one of the set's core kinds
    /// ([`OptionValues::CoreKind`]).
    #[must_use]
    pub fn one_of_the_core_kinds(mut self) -> Self {
        self.values = OptionValues::CoreKind;
        self
    }
}

/// What a kind is, as someone choosing one reads it: its name, what it is
/// in a phrase, and the options it cannot be built without. Every sort of
/// kind describes itself so — a board kind ([`crate::Catalog::board_kinds`]),
/// a part kind (the [`KindGuide::info`] of its guide), a bench component
/// kind ([`crate::Catalog::component_kinds`]) and a P2 core kind
/// (`embsim_boards::p2::CoreCatalog::core_kinds`).
///
/// Names are `Cow`: `&'static str` for kinds written in code, owned for a
/// kind a catalog makes when it runs (one per file it reads, say).
/// Non-exhaustive: build one with [`Self::new`].
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub struct KindInfo {
    /// The kind's name, as a project file gives it.
    pub name: Cow<'static, str>,
    /// What it is, in a phrase.
    pub summary: Cow<'static, str>,
    /// The options it cannot be built without.
    pub required: Vec<RequiredOption>,
}

impl KindInfo {
    /// The kind `name`, which is `summary`, with no required option.
    pub fn new(name: impl Into<Cow<'static, str>>, summary: impl Into<Cow<'static, str>>) -> Self {
        Self {
            name: name.into(),
            summary: summary.into(),
            required: Vec::new(),
        }
    }

    /// The same kind with an option it cannot be built without.
    #[must_use]
    pub fn requires(
        self,
        name: impl Into<Cow<'static, str>>,
        example: impl Into<Cow<'static, str>>,
        means: impl Into<Cow<'static, str>>,
    ) -> Self {
        self.requires_option(RequiredOption::new(name, example, means))
    }

    /// The same kind with `option` among those it cannot be built without.
    #[must_use]
    pub fn requires_option(mut self, option: RequiredOption) -> Self {
        self.required.push(option);
        self
    }
}

/// A part kind as someone choosing one reads it: what every kind says of
/// itself ([`KindInfo`]), and for a part the part numbers it is for, the
/// pin tables it can declare, and what a part has to be for it to seat
/// there. Non-exhaustive: build one with [`Self::new`] and the builders.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub struct KindGuide {
    /// Its name, what it is, and the options it needs.
    pub info: KindInfo,
    /// The part numbers the kind is for: every one the base registry
    /// places it by, and the processor's, which it never places.
    pub numbers: Vec<Cow<'static, str>>,
    /// The pin tables its model can declare. Empty for a kind that takes
    /// the part's own pins, whatever they are (`switch`, `mechanical`,
    /// `boundary`).
    pub tables: Vec<PinTable>,
    /// What a part has to be for the kind to seat there. The project
    /// checks it for every part an entry reaches ([`Self::check`]).
    pub is: Named,
}

/// How a part kind is a part's model, strongest first. Pins alone say
/// nothing: an EDA export numbers every package's pins from 1, so two parts
/// with as many pins share a table whatever they are.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum Fit<'g> {
    /// One of the part's keys and a number the kind is for name the same
    /// part: one is the other, or the other with an ordering suffix
    /// (`ADS122U04` and `ADS122U04IPW`), compared on letters and digits.
    Number(&'g str),
    /// One of the part's keys names the part family the kind's model is
    /// for (`TG2520SMN 26.0000M-ECGNNM3` names `TG2520SMN`): the kind seats
    /// there ([`Named::Family`]), and the model reads what it needs from the
    /// part or refuses it.
    Family(&'g str),
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
    /// A guide with nothing but the kind's name, what it is, and what it
    /// seats on: no part numbers, no pin tables, no required options — the
    /// shape of a kind a project's own catalog adds, filled in with the
    /// builders.
    pub fn new(
        name: impl Into<Cow<'static, str>>,
        summary: impl Into<Cow<'static, str>>,
        is: Named,
    ) -> Self {
        Self {
            info: KindInfo::new(name, summary),
            numbers: Vec::new(),
            tables: Vec::new(),
            is,
        }
    }

    /// The kind's name.
    pub fn name(&self) -> &str {
        &self.info.name
    }

    /// The same guide with an option it cannot be registered without.
    #[must_use]
    pub fn requires(
        mut self,
        name: impl Into<Cow<'static, str>>,
        example: impl Into<Cow<'static, str>>,
        means: impl Into<Cow<'static, str>>,
    ) -> Self {
        self.info = self.info.requires(name, example, means);
        self
    }

    /// The same guide with `option` among those it cannot be registered
    /// without.
    #[must_use]
    pub fn requires_option(mut self, option: RequiredOption) -> Self {
        self.info = self.info.requires_option(option);
        self
    }

    /// The same guide for one more part number.
    #[must_use]
    pub fn number(mut self, number: impl Into<Cow<'static, str>>) -> Self {
        self.numbers.push(number.into());
        self
    }

    /// The same guide with one more pin table.
    #[must_use]
    pub fn table(mut self, table: PinTable) -> Self {
        self.tables.push(table);
        self
    }

    /// The strongest way this kind is the model of a part whose keys (part
    /// name, manufacturer part number, value) are these ([`Fit`]); `None`
    /// when it is not.
    pub fn fit(&self, keys: &[&str]) -> Option<Fit<'_>> {
        let stems: Vec<String> = keys
            .iter()
            .map(|key| number_stem(key))
            .filter(|key| key.len() >= MIN_NUMBER_STEM)
            .collect();
        for number in &self.numbers {
            let stem = number_stem(number);
            if stems
                .iter()
                .any(|key| stem.starts_with(key.as_str()) || key.starts_with(stem.as_str()))
            {
                return Some(Fit::Number(number));
            }
        }
        match &self.is {
            Named::Family(families) => names_a_family(keys, families).map(Fit::Family),
            _ => None,
        }
    }

    /// The table that declares exactly `pins`, compared as sets.
    pub fn table_with(&self, pins: &[&str]) -> Option<&PinTable> {
        let wanted: BTreeSet<&str> = pins.iter().copied().collect();
        self.tables.iter().find(|table| {
            table.pins.len() == wanted.len()
                && table.pins.iter().all(|pin| wanted.contains(pin.as_str()))
        })
    }

    /// Refuse the entry unless every part it reaches is what this kind
    /// says ([`Self::is`]). [`crate::Project`] checks it for every
    /// `[[board.model]]` before the catalog registers the kind, so no
    /// catalog's kind seats on a part it is not.
    pub fn check(&self, assignment: &Assignment<'_>) -> Result<(), ProjectError> {
        for decl in assignment.parts {
            let part = normalize_part(decl);
            let why = match &self.is {
                Named::Family(families) => {
                    let keys = [
                        part.as_str(),
                        decl.mpn.as_deref().unwrap_or(""),
                        &decl.value,
                    ];
                    if names_a_family(&keys, families).is_some() {
                        continue;
                    }
                    format!(
                        "kind {:?} is for {}, and {}'s {} do not",
                        self.name(),
                        self.is.describe(),
                        decl.reference,
                        keys_phrase(&part, decl)
                    )
                }
                Named::Connector => {
                    if is_connector(&decl.reference, &part) {
                        continue;
                    }
                    format!(
                        "kind {:?} is for {}, and {} is neither",
                        self.name(),
                        Named::Connector.describe(),
                        symbol_phrase(&part, decl)
                    )
                }
                Named::Switch => {
                    if is_switch(&decl.reference, &part, &decl.value) {
                        continue;
                    }
                    format!(
                        "kind {:?} is for {}, and {} is neither",
                        self.name(),
                        Named::Switch.describe(),
                        symbol_phrase(&part, decl)
                    )
                }
                Named::OneNet => {
                    let nets = assignment.nets_of(&decl.reference);
                    if nets.len() <= 1 {
                        continue;
                    }
                    let mut listed: Vec<&str> = nets.into_iter().collect();
                    let more = listed.len().saturating_sub(4);
                    listed.truncate(4);
                    format!(
                        "kind {:?} is for {}, and {}'s pins join {} nets ({}{}): a part whose \
                         pins join nets carries current between them",
                        self.name(),
                        Named::OneNet.describe(),
                        decl.reference,
                        listed.len() + more,
                        listed.join(", "),
                        if more > 0 {
                            format!(" and {more} more")
                        } else {
                            String::new()
                        }
                    )
                }
            };
            return Err(assignment.error(format!(
                "{} is not the part this kind says it is: {why}. A kind says what a part is \
                 (DESIGN.md rule 1); a part no kind is for needs a model (PROJECTS.md §7)",
                decl.reference
            )));
        }
        Ok(())
    }
}
