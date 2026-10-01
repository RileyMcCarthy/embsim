//! A system, written down.
//!
//! embsim runs one [`System`]: boards, bench components, the harness wires
//! between them, and a scenario. A project is that list in a TOML file. The
//! file names **kinds**; a [`Catalog`] turns each kind into a netlist and a
//! part registry, a part model, or a bench component. The file never holds
//! the behaviour of a part — every number a model uses stays in the model,
//! with its citation — only which model sits where.
//!
//! ```toml
//! [[board]]
//! name = "ADDON"
//! kind = "netlist"
//! netlist = "addon.net"          # relative to this file
//!
//! [[board.model]]                # the part the survey said needs a model
//! part = "ADS122U04"             # exactly one of part, mpn, value
//! kind = "ads122u04"
//!
//! [[board]]
//! name = "EC32"
//! kind = "p2-ec32mb"             # a board the catalog ships
//!
//! [[board.model]]
//! mpn = "P2X8C4M64P"
//! kind = "p2"
//! [board.model.options]          # validated by the kind; unknown refused
//! core = "held-in-reset"
//!
//! [[wire]]                       # a supply: `from` is a source of its own
//! from = "BENCH.5V"
//! to = "EC32.J203.41"
//! volts = 5.0
//!
//! [[wire]]                       # board to board, connector to connector
//! from = "ADDON.J1.3"
//! to = "EC32.J203.12"
//!
//! [[mate]]                       # every pin of a to b's pin of its number
//! a = "EC32.J203"
//! b = "CARRIER.J3"
//!
//! [[switch]]
//! part = "EC32.S301"
//! pole = 1
//! state = "closed"
//! ```
//!
//! # One pipeline
//!
//! Every board is built by [`Board::from_netlist`] with a [`PartRegistry`] —
//! the constructor every board in embsim goes through. A `kind = "netlist"`
//! board reads its netlist from the path, relative to the project file, and
//! starts from the catalog's base registrations ([`Catalog::register_base`]);
//! a catalog board kind brings its own netlist, and its own registry or the
//! base one, and may bring entries for its own parts ([`Catalog::board`]).
//! Each `[[board.model]]` is checked against what its kind says a part is
//! ([`KindGuide::check`], `DESIGN.md` rule 1, for every catalog's kinds),
//! then registers a part kind into that registry
//! ([`Catalog::register_part`]), under the part name, the
//! manufacturer part number or the value it names — the registry's own
//! keys, looked up in the registry's order: part name, then manufacturer
//! part number, then value. A reference designator is not a key: a model
//! belongs to a kind of part, and every part of that kind on the board is
//! the same model.
//!
//! # The survey
//!
//! Before a board is built it is surveyed with the registry it will build
//! with ([`BoardSurvey::of`]): every part classified, every pin table a
//! model states compared with the netlist. A board whose survey names a
//! part with no model, a part whose pins the model does not declare, or a
//! part the registry refuses, is refused with the survey as the error —
//! the checklist of what is left to assign. A `[[board.model]]` that matches
//! no part, or that another registry entry or the part's own symbol comes
//! before, is refused naming the part.
//!
//! # Wires
//!
//! A harness attaches to a board at its boundary: a wire's board end is a
//! connector pin, `Board.Connector.Pin`, and an error lists the connector's
//! pins, or the board's connectors. The other end is a connector pin on the
//! same or another board, a bench component's pin (`Name.Pin`), or — for a
//! wire with `volts` — a source of its own the harness creates
//! (`BENCH.5V`), one voltage per source. A `[[mate]]` joins two connectors
//! at once: each pin of `a` to `b`'s pin of the same number, or the pairs a
//! cable's `map` names. A `[[pin_short]]` is a scenario fault, a bodge
//! wire, and may join any two part pins on the boards.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt;
use std::path::{Path, PathBuf};

use serde::Deserialize;

use crate::kind::KindGuide;
use crate::netlist::{self, ComponentDecl, ParsedNetlist};
use crate::registry::{normalize_part, Classification};
use crate::report::Reports;
use crate::survey::{BoardSurvey, ConnectorReport};
use crate::{Board, Component, EndpointRef, Harness, JumperState, PartRegistry, Scenario, System};

// ============================================================
// The catalog
// ============================================================

/// A catalog board kind's source: its netlist, the registry it builds with,
/// and the `[[board.model]]` entries it brings for its own parts.
#[derive(Debug)]
pub struct CatalogBoard {
    /// The netlist.
    pub netlist: ParsedNetlist,
    /// The registry the board builds with, before any `[[board.model]]` is
    /// registered: one the catalog builds itself (the P2-EC32MB's, which
    /// places every part of the module but the processor), or `None` to
    /// start from the catalog's base registrations
    /// ([`Catalog::register_base`]) as a `kind = "netlist"` board does.
    pub registry: Option<PartRegistry>,
    /// The board's own `[[board.model]]` entries, registered through the
    /// catalog's part kinds before the project's, with every check an entry
    /// in a project file gets. A project entry with the same key replaces
    /// the board's.
    pub models: Vec<ModelSpec>,
}

impl CatalogBoard {
    /// A board from `netlist` that starts from the catalog's base
    /// registrations and brings no entries of its own.
    pub fn from_base(netlist: ParsedNetlist) -> Self {
        Self {
            netlist,
            registry: None,
            models: Vec::new(),
        }
    }

    /// The same board with `model` among its own entries.
    #[must_use]
    pub fn with_model(mut self, model: ModelSpec) -> Self {
        self.models.push(model);
        self
    }
}

/// Turns a kind named in a project into what it is: a board's netlist and
/// registry, a part model registered into a registry, a bench component.
///
/// A catalog provides only what it says: every method but [`Self::name`]
/// has a default that provides nothing, so a catalog writes the methods for
/// the kinds it has. Catalogs compose in a set
/// (`embsim_boards::catalog::CatalogSet`), which is itself a catalog: it
/// answers each kind from whichever catalog provides it.
pub trait Catalog {
    /// The catalog's name, as an error that names two catalogs prints it:
    /// its crate's name (`"embsim-boards"`, `"custom-project-catalog"`).
    fn name(&self) -> &str;

    /// The board kinds this catalog provides, besides `"netlist"`, which
    /// every project can name and no catalog provides.
    fn board_kinds(&self) -> Vec<String> {
        Vec::new()
    }

    /// The netlist, registry and own entries of the catalog board `spec`
    /// names (a kind in [`Self::board_kinds`]).
    fn board(&self, spec: &BoardSpec) -> Result<CatalogBoard, ProjectError> {
        Err(ProjectError::message(format!(
            "board {}: catalog {} has no board kind {:?}",
            spec.name,
            self.name(),
            spec.kind
        )))
    }

    /// Register every model the catalog places by a key a netlist carries
    /// by itself — a manufacturer part number — into `registry`, so a
    /// `kind = "netlist"` board's survey names only the parts no catalog
    /// can place. The project turns the registry's reference-designator
    /// fallback on for every board that starts from these registrations.
    fn register_base(&self, _registry: &mut PartRegistry) {}

    /// The part kinds a `[[board.model]]` may name, each as someone
    /// choosing one reads it ([`KindGuide`]). The project checks every part
    /// an entry reaches against the kind's [`KindGuide::is`] before it calls
    /// [`Self::register_part`].
    fn part_kinds(&self) -> Vec<KindGuide> {
        Vec::new()
    }

    /// Register the part kind `assignment` names into `registry` under its
    /// key, taking its options from `options` and refusing any it does not
    /// know ([`PartOptions::finish`]). Every part the key reaches is what
    /// the kind says it is: the project checked. A kind whose model reads
    /// board data from a part (a frequency in its value, a divider at
    /// attach) may check [`Assignment::parts`] here, so a part it cannot
    /// configure is an error naming the part, before anything is built.
    /// Starts nothing: a survey registers every entry and builds nothing.
    fn register_part(
        &self,
        _registry: &mut PartRegistry,
        assignment: &Assignment<'_>,
        _options: PartOptions,
    ) -> Result<(), ProjectError> {
        Err(assignment.error(format!(
            "catalog {} has no part kind {:?}",
            self.name(),
            assignment.kind
        )))
    }

    /// The bench component kinds a `[[component]]` may name.
    fn component_kinds(&self) -> Vec<String> {
        Vec::new()
    }

    /// Build the bench component `request` names (a kind in
    /// [`Self::component_kinds`]), taking its options from
    /// [`ComponentRequest::options`].
    fn component(&self, request: ComponentRequest<'_>) -> Result<Box<dyn Component>, ProjectError> {
        Err(ProjectError::message(format!(
            "component {}: catalog {} has no component kind {:?}",
            request.spec.name,
            self.name(),
            request.spec.kind
        )))
    }

    /// The catalogs that each provide the kind `kind` — a board, part,
    /// component or core kind — when two or more do; empty otherwise. A
    /// single catalog provides each of its kinds once; a set that holds two
    /// catalogs naming one kind answers with both, and a project that names
    /// that kind is refused, naming them.
    fn kind_clash(&self, _kind: &str) -> Vec<String> {
        Vec::new()
    }

    /// The catalogs whose base registrations each place parts by `key`,
    /// when two or more do; empty otherwise. A board that starts from the
    /// base registrations and carries a part the key reaches is refused,
    /// naming them, unless an entry assigns that key a kind itself.
    fn base_key_clash(&self, _key: &str) -> Vec<String> {
        Vec::new()
    }
}

/// Which of a part's registry keys a `[[board.model]]` names.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum KeyField {
    /// The libsource part name (rescue-normalized).
    Part,
    /// The manufacturer part number the export carries.
    Mpn,
    /// The value field.
    Value,
}

impl KeyField {
    /// This field of `decl`, if it has one.
    pub fn of(self, decl: &ComponentDecl) -> Option<String> {
        match self {
            KeyField::Part => Some(normalize_part(decl)).filter(|part| !part.is_empty()),
            KeyField::Mpn => decl.mpn.clone(),
            KeyField::Value => Some(decl.value.clone()),
        }
    }
}

impl fmt::Display for KeyField {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            KeyField::Part => "part",
            KeyField::Mpn => "mpn",
            KeyField::Value => "value",
        })
    }
}

/// One `[[board.model]]` as a catalog registers it.
#[derive(Debug)]
pub struct Assignment<'a> {
    /// The board's name in the project.
    pub board: &'a str,
    /// The field the entry matches parts by.
    pub by: KeyField,
    /// The key: the registry key the kind is registered under.
    pub key: &'a str,
    /// The part kind.
    pub kind: &'a str,
    /// Every part the key reaches on the board — whose part name, number or
    /// value is the key — so a kind that reads data from a part can check
    /// each one it will build.
    pub parts: &'a [&'a ComponentDecl],
    /// The project file's directory: a path an option names is relative to
    /// it.
    pub dir: &'a Path,
    /// The board's netlist, for a kind that checks what a part is by the
    /// nets its pins join ([`Self::nets_of`]).
    pub netlist: &'a ParsedNetlist,
    /// Where what the kind's constructor builds reports to the run
    /// ([`Report`](crate::Report)): a constructor clones it and adds its
    /// report when it builds the part, never when the kind registers.
    pub reports: &'a Reports,
}

impl Assignment<'_> {
    /// The distinct nets the pins of the part `reference` sit on, by name.
    pub fn nets_of(&self, reference: &str) -> BTreeSet<&str> {
        self.netlist
            .nets
            .iter()
            .filter(|net| net.nodes.iter().any(|node| node.reference == reference))
            .map(|net| net.name.as_str())
            .collect()
    }

    /// The entry, as an error names it: `board EC32: [[board.model]] mpn =
    /// "AP62301Z6-7" (kind "ap62301")`.
    pub fn context(&self) -> String {
        format!(
            "board {}: [[board.model]] {} = {:?} (kind {:?})",
            self.board, self.by, self.key, self.kind
        )
    }

    /// An error about this entry.
    pub fn error(&self, message: impl fmt::Display) -> ProjectError {
        ProjectError::message(format!("{}: {message}", self.context()))
    }
}

/// A kind's options, taken one by one; [`Self::finish`] refuses what is
/// left, naming what the kind accepts.
#[derive(Debug)]
pub struct PartOptions {
    context: String,
    table: toml::Table,
    accepted: Vec<&'static str>,
}

impl PartOptions {
    /// The options `table`, under `context` for its errors.
    pub fn new(context: impl Into<String>, table: toml::Table) -> Self {
        Self {
            context: context.into(),
            table,
            accepted: Vec::new(),
        }
    }

    /// An error about these options, with the entry in front of it.
    pub fn error(&self, message: impl fmt::Display) -> ProjectError {
        ProjectError::message(format!("{}: {message}", self.context))
    }

    /// Take the option `name` as it is written, if it is given: for a
    /// shape no other `take` reads, which the kind reads itself (and names
    /// in its errors through [`Self::error`]).
    pub fn value(&mut self, name: &'static str) -> Option<toml::Value> {
        self.accepted.push(name);
        self.table.remove(name)
    }

    /// Take the number option `name` (`3.3`, or a whole number), if it is
    /// given.
    pub fn number(&mut self, name: &'static str) -> Result<Option<f64>, ProjectError> {
        match self.value(name) {
            None => Ok(None),
            Some(toml::Value::Float(number)) => Ok(Some(number)),
            Some(toml::Value::Integer(number)) => Ok(Some(number as f64)),
            Some(other) => Err(self.error(format!(
                "options.{name} is a number; {other} is a {}",
                other.type_str()
            ))),
        }
    }

    /// Take the whole-number option `name`, if it is given.
    pub fn integer(&mut self, name: &'static str) -> Result<Option<i64>, ProjectError> {
        match self.value(name) {
            None => Ok(None),
            Some(toml::Value::Integer(number)) => Ok(Some(number)),
            Some(other) => Err(self.error(format!(
                "options.{name} is a whole number; {other} is a {}",
                other.type_str()
            ))),
        }
    }

    /// Take the option `name`, a time written as `embsim run --for` takes
    /// it (`"1.5ms"`, [`parse_duration`]), in nanoseconds, if it is given.
    pub fn duration(&mut self, name: &'static str) -> Result<Option<u64>, ProjectError> {
        let Some(text) = self.string(name)? else {
            return Ok(None);
        };
        parse_duration(&text)
            .map(Some)
            .map_err(|why| self.error(format!("options.{name}: {why}")))
    }

    /// Take the string option `name`, if it is given.
    pub fn string(&mut self, name: &'static str) -> Result<Option<String>, ProjectError> {
        self.accepted.push(name);
        match self.table.remove(name) {
            None => Ok(None),
            Some(toml::Value::String(text)) => Ok(Some(text)),
            Some(other) => Err(self.error(format!(
                "options.{name} is a string; {other} is a {}",
                other.type_str()
            ))),
        }
    }

    /// Take the option `name`, which must be one of `choices`; `None` when
    /// it is not given.
    pub fn choice(
        &mut self,
        name: &'static str,
        choices: &[&'static str],
    ) -> Result<Option<&'static str>, ProjectError> {
        let Some(text) = self.string(name)? else {
            return Ok(None);
        };
        choices
            .iter()
            .copied()
            .find(|choice| *choice == text)
            .map(Some)
            .ok_or_else(|| {
                self.error(format!(
                    "options.{name} = {text:?} is not one this kind offers; it offers {}",
                    quoted_list(choices)
                ))
            })
    }

    /// Take the option `name`, a list of two-string lists (`[["1", "2"],
    /// ["3", "4"]]`), if it is given.
    pub fn pairs(
        &mut self,
        name: &'static str,
    ) -> Result<Option<Vec<(String, String)>>, ProjectError> {
        self.accepted.push(name);
        let Some(value) = self.table.remove(name) else {
            return Ok(None);
        };
        let shape = || {
            self.error(format!(
                "options.{name} is a list of two-pin lists, such as [[\"1\", \"2\"], [\"3\", \
                 \"4\"]]"
            ))
        };
        let toml::Value::Array(items) = value else {
            return Err(shape());
        };
        let mut pairs = Vec::with_capacity(items.len());
        for item in items {
            match item {
                toml::Value::Array(pair) => match pair.as_slice() {
                    [toml::Value::String(a), toml::Value::String(b)] => {
                        pairs.push((a.clone(), b.clone()));
                    }
                    _ => return Err(shape()),
                },
                _ => return Err(shape()),
            }
        }
        Ok(Some(pairs))
    }

    /// Refuse every option no `take` asked for, naming the ones the kind
    /// accepts.
    pub fn finish(self) -> Result<(), ProjectError> {
        if self.table.is_empty() {
            return Ok(());
        }
        let unknown: Vec<&str> = self.table.keys().map(String::as_str).collect();
        let accepted = if self.accepted.is_empty() {
            "this kind takes no options".to_string()
        } else {
            format!("this kind takes {}", quoted_list(&self.accepted))
        };
        Err(self.error(format!(
            "unknown option{} {}; {accepted}",
            if unknown.len() == 1 { "" } else { "s" },
            quoted_list(&unknown)
        )))
    }
}

/// Parse a duration of virtual time: a number and a unit, `ns`, `us` (or
/// `µs`), `ms` or `s` (`20ms`, `1.5 s`, `250us`), to whole nanoseconds —
/// how `embsim run --for` and every option that is a time write one.
pub fn parse_duration(text: &str) -> Result<u64, String> {
    let text = text.trim();
    let split = text
        .find(|c: char| !(c.is_ascii_digit() || c == '.'))
        .ok_or_else(|| format!("{text:?} has no unit; give one of ns, us, ms, s (20ms)"))?;
    let (number, unit) = text.split_at(split);
    if number.is_empty() {
        return Err(format!(
            "{text:?} does not start with a number; give a number and a unit (20ms)"
        ));
    }
    let per_unit: f64 = match unit.trim() {
        "ns" => 1.0,
        "us" | "µs" => 1e3,
        "ms" => 1e6,
        "s" => 1e9,
        other => {
            return Err(format!(
                "{other:?} is not a unit of time; give one of ns, us, ms, s (20ms)"
            ))
        }
    };
    let value: f64 = number
        .parse()
        .map_err(|_| format!("{number:?} is not a number of {}", unit.trim()))?;
    let ns = (value * per_unit).round();
    if !ns.is_finite() || ns > u64::MAX as f64 {
        return Err(format!("{text:?} is longer than a run can be"));
    }
    Ok(ns as u64)
}

// ============================================================
// The file
// ============================================================

/// Why a project could not be read or built. The message says what to fix.
#[derive(Debug)]
pub struct ProjectError {
    message: String,
}

impl ProjectError {
    /// An error whose whole content is `message`.
    pub fn message(message: impl Into<String>) -> Self {
        Self {
            message: message.into(),
        }
    }
}

impl fmt::Display for ProjectError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.message)
    }
}

impl std::error::Error for ProjectError {}

/// One board: a name in the system, a kind, and the models its parts take.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BoardSpec {
    /// System name. Endpoints are `{name}.{Ref}.{Pin}`.
    pub name: String,
    /// `"netlist"`, or a catalog board kind (`"p2-ec32mb"`).
    pub kind: String,
    /// The netlist, for `kind = "netlist"`: a path relative to the project
    /// file's directory.
    #[serde(default)]
    pub netlist: Option<String>,
    /// The models the board's parts take, `[[board.model]]`.
    #[serde(default)]
    pub model: Vec<ModelSpec>,
}

/// One `[[board.model]]`: the part kind for every part whose part name,
/// manufacturer part number or value — exactly one of them — is the key.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ModelSpec {
    /// Match by the libsource part name.
    #[serde(default)]
    pub part: Option<String>,
    /// Match by the manufacturer part number.
    #[serde(default)]
    pub mpn: Option<String>,
    /// Match by the value field.
    #[serde(default)]
    pub value: Option<String>,
    /// The part kind (`"ads122u04"`).
    pub kind: String,
    /// The kind's options, `[board.model.options]`.
    #[serde(default)]
    pub options: toml::Table,
}

impl ModelSpec {
    fn keyed(by: KeyField, key: &str, kind: &str) -> Self {
        let key = Some(key.to_string());
        Self {
            part: key.clone().filter(|_| by == KeyField::Part),
            mpn: key.clone().filter(|_| by == KeyField::Mpn),
            value: key.filter(|_| by == KeyField::Value),
            kind: kind.to_string(),
            options: toml::Table::new(),
        }
    }

    /// An entry giving every part whose symbol's part name is `key` the
    /// kind `kind`: a board kind's own entry ([`CatalogBoard::models`]).
    pub fn by_part(key: &str, kind: &str) -> Self {
        Self::keyed(KeyField::Part, key, kind)
    }

    /// An entry by manufacturer part number.
    pub fn by_mpn(key: &str, kind: &str) -> Self {
        Self::keyed(KeyField::Mpn, key, kind)
    }

    /// An entry by value.
    pub fn by_value(key: &str, kind: &str) -> Self {
        Self::keyed(KeyField::Value, key, kind)
    }

    /// The same entry with the option `name` set to `value`.
    #[must_use]
    pub fn option(mut self, name: &str, value: impl Into<toml::Value>) -> Self {
        self.options.insert(name.to_string(), value.into());
        self
    }

    /// The one field this entry matches by, and its key.
    pub fn key(&self) -> Option<(KeyField, &str)> {
        match (&self.part, &self.mpn, &self.value) {
            (Some(key), None, None) => Some((KeyField::Part, key)),
            (None, Some(key), None) => Some((KeyField::Mpn, key)),
            (None, None, Some(key)) => Some((KeyField::Value, key)),
            _ => None,
        }
    }
}

/// One bench component: a name in the system, a catalog kind and its
/// options.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ComponentSpec {
    /// System name. Endpoints are `{name}.{Pin}`.
    pub name: String,
    /// Catalog kind.
    pub kind: String,
    /// The kind's options, `[component.options]`.
    #[serde(default)]
    pub options: toml::Table,
}

/// One `[[component]]` as a catalog builds it.
#[derive(Debug)]
pub struct ComponentRequest<'a> {
    /// The entry: its name and kind.
    pub spec: &'a ComponentSpec,
    /// Its options, taken one by one; [`PartOptions::finish`] refuses the
    /// rest.
    pub options: PartOptions,
    /// The project file's directory: a path an option names is relative to
    /// it.
    pub dir: &'a Path,
    /// Where what the component has to say goes ([`Report`](crate::Report)).
    pub reports: &'a Reports,
}

impl ComponentRequest<'_> {
    /// The entry, as an error names it: `component HOST (kind
    /// "host-serial")`.
    pub fn context(&self) -> String {
        component_context(self.spec)
    }

    /// An error about this entry.
    pub fn error(&self, message: impl fmt::Display) -> ProjectError {
        ProjectError::message(format!("{}: {message}", self.context()))
    }
}

fn component_context(spec: &ComponentSpec) -> String {
    format!("component {} (kind {:?})", spec.name, spec.kind)
}

/// One harness wire. `volts` makes it a supply on `from`.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct WireSpec {
    /// Dotted endpoint.
    pub from: String,
    /// Dotted endpoint.
    pub to: String,
    /// Source voltage. Absent, the wire only joins the two endpoints.
    #[serde(default)]
    pub volts: Option<f64>,
}

/// Two connectors mated: a module seated in its socket, a cable between two
/// headers. Each pin of `a` is joined to a pin of `b`.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MateSpec {
    /// The connector that plugs in, `Board.Connector`: every one of its
    /// pins is joined, unless `map` names the ones that are.
    pub a: String,
    /// The connector it mates with, `Board.Connector`. Without `map` it has
    /// a pin of each number `a` has; it may have more, which stay open (a
    /// socket wider than the card seated in it).
    pub b: String,
    /// The pins a cable joins when it does not join them by number, as
    /// `[["a pin", "b pin"], …]`: only these pairs are joined.
    #[serde(default)]
    pub map: Option<Vec<[String; 2]>>,
}

/// Open or closed, for a switch pole or a jumper.
#[derive(Debug, Clone, Copy, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum ContactState {
    /// Terminals disconnected.
    Open,
    /// Terminals shorted.
    Closed,
}

impl From<ContactState> for JumperState {
    fn from(state: ContactState) -> Self {
        match state {
            ContactState::Open => JumperState::Open,
            ContactState::Closed => JumperState::Closed,
        }
    }
}

/// One switch pole (`"EC32.S301"`, pole `1`, closed).
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SwitchSpec {
    /// Dotted reference, board included.
    pub part: String,
    /// Pole index, from 0, in the order the part declared its poles.
    pub pole: usize,
    /// Open or closed.
    pub state: ContactState,
}

/// One jumper (`"DS2Addon.JP1"`, closed).
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct JumperSpec {
    /// Dotted reference, board included.
    pub part: String,
    /// Open or closed.
    pub state: ContactState,
}

/// Two part pins whose nets become one — a scenario fault, a bodge wire.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PinShortSpec {
    /// Dotted pin, `Board.Ref.Pin`.
    pub a: String,
    /// Dotted pin, `Board.Ref.Pin`.
    pub b: String,
}

/// `[catalog]`: the project's own catalog crates, which the `embsim` tool
/// builds into a runner and runs the project through (`PROJECTS.md` §10,
/// "The runner"). A project without it runs on the catalogs embsim ships.
///
/// The table says where the crates are, and nothing about what they hold:
/// the kinds they add are named in the rest of the file like any other.
#[derive(Debug, Clone, Default, PartialEq, Eq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CatalogTable {
    /// The catalog crates, each a directory holding a `Cargo.toml`,
    /// relative to the project file. Their registration functions run in
    /// this order, after the catalogs embsim ships.
    pub crates: Vec<String>,
    /// The embsim checkout (its workspace root, relative to the project
    /// file) the runner builds embsim from; by default the checkout the
    /// `embsim` tool was built from.
    #[serde(default)]
    pub embsim: Option<String>,
}

impl CatalogTable {
    /// The `[catalog]` table of the project text `text`, reading nothing
    /// else of it: what the `embsim` tool reads before it hands a project
    /// to its runner, so a project the runner's embsim reads is never
    /// refused by an older tool. `None` when the file has no `[catalog]`.
    pub fn of_project_text(text: &str) -> Result<Option<Self>, ProjectError> {
        /// The file, every table but `[catalog]` passed over unread.
        #[derive(Deserialize)]
        struct CatalogOnly {
            #[serde(default)]
            catalog: Option<CatalogTable>,
        }
        let file: CatalogOnly = toml::from_str(text)
            .map_err(|err| ProjectError::message(format!("project does not parse: {err}")))?;
        match file.catalog {
            Some(catalog) => {
                catalog.check()?;
                Ok(Some(catalog))
            }
            None => Ok(None),
        }
    }

    /// The `[catalog]` table of the project file at `path`
    /// ([`Self::of_project_text`]).
    pub fn of_project(path: &Path) -> Result<Option<Self>, ProjectError> {
        let text = std::fs::read_to_string(path).map_err(|err| {
            ProjectError::message(format!("cannot read project {}: {err}", path.display()))
        })?;
        Self::of_project_text(&text)
            .map_err(|err| ProjectError::message(format!("{}: {err}", path.display())))
    }

    /// Refuse a table that names no crate, a crate twice, or an empty path.
    fn check(&self) -> Result<(), ProjectError> {
        if self.crates.is_empty() {
            return Err(ProjectError::message(
                "[catalog] crates is empty: name the project's catalog crates (crates = \
                 [\"sim/catalog\"]), or take the table out to run on the catalogs embsim ships",
            ));
        }
        let mut seen = BTreeSet::new();
        for path in &self.crates {
            if path.trim().is_empty() {
                return Err(ProjectError::message(
                    "[catalog] crates names an empty path; each is a crate's directory, \
                     relative to the project file",
                ));
            }
            if !seen.insert(path.as_str()) {
                return Err(ProjectError::message(format!(
                    "[catalog] crates names {path:?} twice; a crate registers its kinds once"
                )));
            }
        }
        if self
            .embsim
            .as_deref()
            .is_some_and(|path| path.trim().is_empty())
        {
            return Err(ProjectError::message(
                "[catalog] embsim is empty: name an embsim checkout's root, relative to the \
                 project file, or leave the key out to build against the one embsim was built \
                 from",
            ));
        }
        Ok(())
    }
}

/// The directory beside a project file where embsim keeps what it makes
/// for the project and nobody edits — a `host-serial`'s PTY, the runner the
/// `embsim` tool builds — made if it is missing, with a `.gitignore` that
/// keeps all of it out of version control.
pub fn state_dir(project_dir: &Path) -> std::io::Result<PathBuf> {
    let dir = project_dir.join(".embsim");
    std::fs::create_dir_all(&dir)?;
    let ignore = dir.join(".gitignore");
    if !ignore.exists() {
        std::fs::write(
            &ignore,
            "# Written by embsim: what it makes for the project beside this directory\n\
             # (PTYs, the runner it builds). Nothing here is source.\n*\n",
        )?;
    }
    Ok(dir)
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct ProjectFile {
    /// `[catalog]`: the project's own catalog crates.
    #[serde(default)]
    catalog: Option<CatalogTable>,
    #[serde(default)]
    board: Vec<BoardSpec>,
    #[serde(default)]
    component: Vec<ComponentSpec>,
    #[serde(default)]
    wire: Vec<WireSpec>,
    #[serde(default)]
    mate: Vec<MateSpec>,
    #[serde(default)]
    switch: Vec<SwitchSpec>,
    #[serde(default)]
    jumper: Vec<JumperSpec>,
    #[serde(default)]
    pin_short: Vec<PinShortSpec>,
}

// ============================================================
// The project
// ============================================================

/// A parsed project. [`Self::instantiate`] asks a [`Catalog`] for every kind
/// and returns the [`System`], not yet started.
#[derive(Debug, Clone)]
pub struct Project {
    file: ProjectFile,
    dir: PathBuf,
}

/// One board, prepared: its netlist, the registry it builds with, and the
/// survey of the two.
struct Prepared {
    netlist: ParsedNetlist,
    registry: PartRegistry,
    survey: BoardSurvey,
}

impl Project {
    /// Parse `text` as a project whose paths are relative to the current
    /// directory. Kinds are not resolved yet.
    pub fn parse(text: &str) -> Result<Self, ProjectError> {
        let file: ProjectFile = toml::from_str(text)
            .map_err(|err| ProjectError::message(format!("project does not parse: {err}")))?;
        if let Some(catalog) = &file.catalog {
            catalog.check()?;
        }
        let project = Self {
            file,
            dir: PathBuf::from("."),
        };
        project.check_names()?;
        Ok(project)
    }

    /// Read and parse the project file at `path`. Its paths are relative
    /// to the file's own directory.
    pub fn load(path: impl AsRef<Path>) -> Result<Self, ProjectError> {
        let path = path.as_ref();
        let text = std::fs::read_to_string(path).map_err(|err| {
            ProjectError::message(format!("cannot read project {}: {err}", path.display()))
        })?;
        let mut project = Self::parse(&text)
            .map_err(|err| ProjectError::message(format!("{}: {err}", path.display())))?;
        project.dir = path
            .parent()
            .map(Path::to_path_buf)
            .unwrap_or_else(|| PathBuf::from("."));
        Ok(project)
    }

    /// The project with its paths relative to `dir` — for a project parsed
    /// from text that lives somewhere else.
    #[must_use]
    pub fn relative_to(mut self, dir: impl Into<PathBuf>) -> Self {
        self.dir = dir.into();
        self
    }

    /// The directory the project's paths are relative to.
    pub fn dir(&self) -> &Path {
        &self.dir
    }

    /// The project's `[catalog]` table, when it names catalog crates of its
    /// own. Building a system does not read it: the kinds those crates add
    /// reach the project through the catalog it is built with.
    pub fn catalog(&self) -> Option<&CatalogTable> {
        self.file.catalog.as_ref()
    }

    /// The boards, in file order.
    pub fn boards(&self) -> &[BoardSpec] {
        &self.file.board
    }

    /// The bench components, in file order.
    pub fn components(&self) -> &[ComponentSpec] {
        &self.file.component
    }

    /// The wires, in file order.
    pub fn wires(&self) -> &[WireSpec] {
        &self.file.wire
    }

    /// The mated connector pairs, in file order.
    pub fn mates(&self) -> &[MateSpec] {
        &self.file.mate
    }

    /// The bench component named `name`, as the file gives it.
    pub fn component_spec(&self, name: &str) -> Option<&ComponentSpec> {
        self.file.component.iter().find(|spec| spec.name == name)
    }

    /// Set the option `key` of the bench component `name` to `value`, as if
    /// the file had said it: what `embsim run --pty` does to a
    /// `host-serial`'s `path`. The kind still checks it when the system is
    /// built.
    pub fn set_component_option(
        &mut self,
        name: &str,
        key: &str,
        value: impl Into<toml::Value>,
    ) -> Result<(), ProjectError> {
        let names: Vec<&str> = self
            .file
            .component
            .iter()
            .map(|spec| spec.name.as_str())
            .collect();
        let none = || {
            ProjectError::message(format!(
                "no component {name:?} in this project; its components: {}",
                if names.is_empty() {
                    "none".to_string()
                } else {
                    names.join(", ")
                }
            ))
        };
        let index = self
            .file
            .component
            .iter()
            .position(|spec| spec.name == name)
            .ok_or_else(none)?;
        self.file.component[index]
            .options
            .insert(key.to_string(), value.into());
        Ok(())
    }

    /// Build the [`System`] this project describes, its paths relative to
    /// [`Self::dir`]. The caller starts it; a test that wants the
    /// attach-time circuit and no later wake calls [`System::hold_time`]
    /// before [`System::start`]. What the build's constructors report goes
    /// nowhere; [`Self::instantiate_with`] keeps it.
    pub fn instantiate(&self, catalog: &dyn Catalog) -> Result<System, ProjectError> {
        self.instantiate_with(catalog, &Reports::new())
    }

    /// [`Self::instantiate`], every report the build's constructors make
    /// added to `reports` ([`crate::Report`]): what a run prints about what
    /// a catalog built.
    pub fn instantiate_with(
        &self,
        catalog: &dyn Catalog,
        reports: &Reports,
    ) -> Result<System, ProjectError> {
        self.build_system(catalog, &self.dir, reports)
    }

    /// [`Self::instantiate`] with `base` as the directory the project's
    /// paths are relative to — for a project parsed from text embedded
    /// somewhere else.
    pub fn instantiate_in(
        &self,
        catalog: &dyn Catalog,
        base: &Path,
    ) -> Result<System, ProjectError> {
        self.build_system(catalog, base, &Reports::new())
    }

    fn build_system(
        &self,
        catalog: &dyn Catalog,
        base: &Path,
        reports: &Reports,
    ) -> Result<System, ProjectError> {
        let mut system = System::new();
        let mut surveys: BTreeMap<String, BoardSurvey> = BTreeMap::new();
        for spec in &self.file.board {
            let (board, survey) = build(spec, catalog, base, reports)?;
            surveys.insert(spec.name.clone(), survey);
            system = system.board(&spec.name, board);
        }

        let known_components = catalog.component_kinds();
        let mut bench_pins: BTreeMap<String, Vec<String>> = BTreeMap::new();
        for spec in &self.file.component {
            if !known_components.contains(&spec.kind) {
                return Err(ProjectError::message(format!(
                    "component {}: unknown kind {:?}; {}",
                    spec.name,
                    spec.kind,
                    kinds_sentence("component", &known_components)
                )));
            }
            refuse_clash(catalog, &spec.kind, &component_context(spec))?;
            let component = catalog.component(ComponentRequest {
                spec,
                options: PartOptions::new(component_context(spec), spec.options.clone()),
                dir: base,
                reports,
            })?;
            let pins = component
                .pins()
                .iter()
                .flat_map(|pin| std::iter::once(pin.number).chain(pin.name))
                .map(str::to_string)
                .collect();
            bench_pins.insert(spec.name.clone(), pins);
            system = system.component(&spec.name, component);
        }

        // Every source a wire's volts make is its own: one endpoint, one
        // voltage. A second `volts` on the same `from` would be a second
        // source fighting the first through whatever joins them.
        let mut sources: BTreeMap<String, &WireSpec> = BTreeMap::new();
        for wire in self.file.wire.iter().filter(|wire| wire.volts.is_some()) {
            let Ok(from) = EndpointRef::parse(&wire.from) else {
                continue;
            };
            let name = endpoint_text(&from);
            if let Some(first) = sources.get(&name) {
                return Err(ProjectError::message(format!(
                    "[[wire]] {} to {}: {name} is already a source, at {} V, made by [[wire]] \
                     {} to {}; join it with a wire that has no volts, or give the second \
                     source a name of its own",
                    wire.from,
                    wire.to,
                    first.volts.unwrap_or_default(),
                    first.from,
                    first.to
                )));
            }
            sources.insert(name, wire);
        }
        // The supplies the harness creates: a name of its own a wire with
        // volts starts at. Any other wire may join one.
        let supplies: BTreeSet<String> = sources
            .keys()
            .filter(|name| {
                EndpointRef::parse(name).is_ok_and(|from| {
                    !surveys.contains_key(&from.board) && !bench_pins.contains_key(&from.board)
                })
            })
            .cloned()
            .collect();
        let places = Places {
            boards: &surveys,
            bench: &bench_pins,
            supplies: &supplies,
        };
        let mut harness = Harness::new();
        for wire in &self.file.wire {
            let what = format!("[[wire]] {} to {}", wire.from, wire.to);
            if let Some(volts) = wire.volts {
                if !volts.is_finite() {
                    return Err(ProjectError::message(format!(
                        "{what}: volts = {volts} is not a voltage"
                    )));
                }
            }
            let from = places.wire_end(&what, &wire.from, wire.volts.is_some())?;
            let to = places.wire_end(&what, &wire.to, false)?;
            if from == to {
                return Err(ProjectError::message(format!(
                    "{what}: a wire joins two different endpoints"
                )));
            }
            harness = match wire.volts {
                Some(volts) => harness.power(from, to, volts),
                None => harness.connect(from, to),
            };
        }
        for mate in &self.file.mate {
            for (a, b) in places.mate(mate)? {
                harness = harness.connect(a, b);
            }
        }
        if !harness.connections().is_empty() {
            system = system.harness(harness);
        }

        let mut scenario = Scenario::default();
        for switch in &self.file.switch {
            places.switch_pole(switch)?;
            scenario = scenario.switch(&switch.part, switch.pole, switch.state.into());
        }
        for jumper in &self.file.jumper {
            places.jumper(jumper)?;
            scenario = scenario.jumper(&jumper.part, jumper.state.into());
        }
        for short in &self.file.pin_short {
            let what = format!("[[pin_short]] {} to {}", short.a, short.b);
            places.part_pin(&what, &short.a)?;
            places.part_pin(&what, &short.b)?;
            scenario = scenario.pin_short(&short.a, &short.b);
        }
        Ok(system.scenario(scenario))
    }

    /// The survey of the board `name` as this project configures it — its
    /// netlist and the registry with every `[[board.model]]` registered —
    /// whether or not it is ready to build: the checklist of what is left.
    pub fn survey(&self, catalog: &dyn Catalog, name: &str) -> Result<BoardSurvey, ProjectError> {
        let spec = self.board_spec(name)?;
        prepare(spec, catalog, &self.dir, &Reports::new()).map(|prepared| prepared.survey)
    }

    /// The board `name`, built as [`Self::instantiate`] builds it.
    pub fn build_board(&self, catalog: &dyn Catalog, name: &str) -> Result<Board, ProjectError> {
        let spec = self.board_spec(name)?;
        build(spec, catalog, &self.dir, &Reports::new()).map(|(board, _)| board)
    }

    fn board_spec(&self, name: &str) -> Result<&BoardSpec, ProjectError> {
        self.file
            .board
            .iter()
            .find(|spec| spec.name == name)
            .ok_or_else(|| {
                ProjectError::message(format!(
                    "no board {name:?} in this project; its boards: {}",
                    self.file
                        .board
                        .iter()
                        .map(|spec| spec.name.as_str())
                        .collect::<Vec<_>>()
                        .join(", ")
                ))
            })
    }

    /// Board and component names are endpoint prefixes: each one a word
    /// with no dot, used once.
    fn check_names(&self) -> Result<(), ProjectError> {
        let mut seen = BTreeSet::new();
        let names = self
            .file
            .board
            .iter()
            .map(|spec| ("board", spec.name.as_str()))
            .chain(
                self.file
                    .component
                    .iter()
                    .map(|spec| ("component", spec.name.as_str())),
            );
        for (what, name) in names {
            if name.is_empty() || name.contains('.') || name.contains(char::is_whitespace) {
                return Err(ProjectError::message(format!(
                    "{what} name {name:?}: a name is the first word of every endpoint on it, \
                     so it is not empty and has no dot or space"
                )));
            }
            if !seen.insert(name) {
                return Err(ProjectError::message(format!(
                    "{what} name {name:?} is used twice; every board and component has its own"
                )));
            }
        }
        Ok(())
    }
}

/// Prepare and build one board, refusing it unless its survey is clean.
fn build(
    spec: &BoardSpec,
    catalog: &dyn Catalog,
    base: &Path,
    reports: &Reports,
) -> Result<(Board, BoardSurvey), ProjectError> {
    let Prepared {
        netlist,
        registry,
        survey,
    } = prepare(spec, catalog, base, reports)?;
    if !survey.compliant() {
        return Err(ProjectError::message(format!(
            "board {} is not ready to build:\n{survey}{}",
            spec.name,
            not_ready_hint(&survey, catalog)
        )));
    }
    let board = Board::from_netlist(netlist, &registry)
        .map_err(|err| ProjectError::message(format!("board {}: {err}", spec.name)))?;
    Ok((board, survey))
}

/// What to do about a survey that is not clean.
fn not_ready_hint(survey: &BoardSurvey, catalog: &dyn Catalog) -> String {
    let mut hint = String::new();
    if !survey.needs_model.is_empty() {
        hint.push_str(
            "give each part that needs a model a [[board.model]] with its part, mpn or value \
             and the kind it is; a kind seats only on a part that is what the kind says, and a \
             part no kind is for needs a model written for it (PROJECTS.md §7)",
        );
        let kinds: Vec<String> = catalog
            .part_kinds()
            .iter()
            .map(|kind| kind.name.to_string())
            .collect();
        hint.push_str(&format!("; {}\n", kinds_sentence("part", &kinds)));
    }
    if !survey.mismatched.is_empty() {
        hint.push_str(
            "a model that declares other pins than the netlist takes another pin table: give \
             the part a [[board.model]] whose options.pins picks the table with the netlist's \
             pins\n",
        );
    }
    hint
}

/// `"a and b"`, `"a, b and c"`.
fn and_list(items: &[String]) -> String {
    match items {
        [] => String::new(),
        [one] => one.clone(),
        [rest @ .., last] => format!("{} and {last}", rest.join(", ")),
    }
}

/// Refuse the kind `kind` when two or more of the catalogs behind
/// `catalog` provide it ([`Catalog::kind_clash`]), naming them.
fn refuse_clash(catalog: &dyn Catalog, kind: &str, what: &str) -> Result<(), ProjectError> {
    let providers = catalog.kind_clash(kind);
    if providers.len() < 2 {
        return Ok(());
    }
    Err(ProjectError::message(format!(
        "{what}: kind {kind:?} is provided by {} catalogs, {}; a kind means one thing, so a \
         catalog a project adds gives each of its kinds a name no other catalog has, its \
         project's prefix in front (PROJECTS.md §10)",
        providers.len(),
        and_list(&providers)
    )))
}

/// A registry that starts from the catalog's base registrations, the
/// reference-designator fallback on: what a `kind = "netlist"` board, and a
/// board kind that names no registry of its own, build with.
fn base_registry(catalog: &dyn Catalog) -> PartRegistry {
    let mut registry = PartRegistry::new();
    // A netlist transcribed from a schematic carries no libsource; its
    // passives and connectors classify by their reference designator.
    registry.classify_unnamed_by_reference(true);
    catalog.register_base(&mut registry);
    registry
}

/// Where a `[[board.model]]` entry comes from: the project file, or the
/// board kind's own ([`CatalogBoard::models`]).
#[derive(Clone, Copy, PartialEq, Eq)]
enum Origin {
    Project,
    BoardKind,
}

/// Read the board's netlist, register its models, and survey it.
fn prepare(
    spec: &BoardSpec,
    catalog: &dyn Catalog,
    base: &Path,
    reports: &Reports,
) -> Result<Prepared, ProjectError> {
    let (netlist, mut registry, from_base, board_models) = if spec.kind == "netlist" {
        let relative = spec.netlist.as_deref().ok_or_else(|| {
            ProjectError::message(format!(
                "board {}: kind \"netlist\" reads a netlist file; give its path, netlist = \
                 \"board.net\", relative to the project file",
                spec.name
            ))
        })?;
        let path = base.join(relative);
        let text = std::fs::read_to_string(&path).map_err(|err| {
            ProjectError::message(format!(
                "board {}: cannot read netlist {}: {err}",
                spec.name,
                path.display()
            ))
        })?;
        let parsed = netlist::parse(&text).map_err(|err| {
            ProjectError::message(format!(
                "board {}: netlist {}: {err}",
                spec.name,
                path.display()
            ))
        })?;
        (parsed, base_registry(catalog), true, Vec::new())
    } else {
        let kinds = catalog.board_kinds();
        if !kinds.contains(&spec.kind) {
            let mut all = vec!["netlist".to_string()];
            all.extend(kinds);
            return Err(ProjectError::message(format!(
                "board {}: unknown kind {:?}; {}",
                spec.name,
                spec.kind,
                kinds_sentence("board", &all)
            )));
        }
        refuse_clash(catalog, &spec.kind, &format!("board {}", spec.name))?;
        if spec.netlist.is_some() {
            return Err(ProjectError::message(format!(
                "board {}: kind {:?} brings its own netlist; netlist = … is for kind = \
                 \"netlist\"",
                spec.name, spec.kind
            )));
        }
        let CatalogBoard {
            netlist,
            registry,
            models,
        } = catalog.board(spec)?;
        match registry {
            Some(registry) => (netlist, registry, false, models),
            None => (netlist, base_registry(catalog), true, models),
        }
    };

    // The board kind's own entries first, each one a project entry with the
    // same key does not replace; then the project's.
    let project_keys: BTreeSet<&str> = spec
        .model
        .iter()
        .filter_map(|model| model.key().map(|(_, key)| key))
        .collect();
    let entries: Vec<(&ModelSpec, Origin)> = board_models
        .iter()
        .filter(|model| {
            model
                .key()
                .is_none_or(|(_, key)| !project_keys.contains(key))
        })
        .map(|model| (model, Origin::BoardKind))
        .chain(spec.model.iter().map(|model| (model, Origin::Project)))
        .collect();
    let from_kind = |origin: Origin, err: ProjectError| match origin {
        Origin::Project => err,
        Origin::BoardKind => ProjectError::message(format!(
            "{err} (an entry board kind {:?} brings for its own parts)",
            spec.kind
        )),
    };

    let part_kinds = catalog.part_kinds();
    // The registry has one namespace of keys, whatever field an entry names:
    // a second entry under the same string would replace the first.
    let mut assigned: BTreeMap<String, KeyField> = BTreeMap::new();
    let mut checks: Vec<(KeyField, String, Vec<usize>)> = Vec::new();
    for (model, origin) in entries {
        let (by, key) = model.key().ok_or_else(|| {
            from_kind(
                origin,
                ProjectError::message(format!(
                    "board {}: a [[board.model]] (kind {:?}) matches parts by exactly one of \
                     part, mpn or value",
                    spec.name, model.kind
                )),
            )
        })?;
        if let Some(first) = assigned.get(key) {
            let clash = if *first == by {
                format!("two [[board.model]] entries have {by} = {key:?}")
            } else {
                format!("[[board.model]] {first} = {key:?} and {by} = {key:?} are one registry key")
            };
            return Err(from_kind(
                origin,
                ProjectError::message(format!(
                    "board {}: {clash}, and a key takes one model; keep one",
                    spec.name
                )),
            ));
        }
        assigned.insert(key.to_string(), by);
        let matched: Vec<usize> = netlist
            .components
            .iter()
            .enumerate()
            .filter(|(_, decl)| by.of(decl).as_deref() == Some(key))
            .map(|(index, _)| index)
            .collect();
        if matched.is_empty() {
            return Err(from_kind(
                origin,
                ProjectError::message(format!(
                    "board {}: [[board.model]] {by} = {key:?} matches no part on the board{}",
                    spec.name,
                    no_match_hint(&netlist, by, key)
                )),
            ));
        }
        let Some(guide) = part_kinds.iter().find(|kind| kind.name == model.kind) else {
            let names: Vec<String> = part_kinds
                .iter()
                .map(|kind| kind.name.to_string())
                .collect();
            return Err(from_kind(
                origin,
                ProjectError::message(format!(
                    "board {}: [[board.model]] {by} = {key:?}: unknown kind {:?}; {}",
                    spec.name,
                    model.kind,
                    kinds_sentence("part", &names)
                )),
            ));
        };
        let reached: Vec<&ComponentDecl> = netlist
            .components
            .iter()
            .filter(|decl| {
                [KeyField::Part, KeyField::Mpn, KeyField::Value]
                    .iter()
                    .any(|field| field.of(decl).as_deref() == Some(key))
            })
            .collect();
        let assignment = Assignment {
            board: &spec.name,
            by,
            key,
            kind: &model.kind,
            parts: &reached,
            dir: base,
            netlist: &netlist,
            reports,
        };
        refuse_clash(catalog, &model.kind, &assignment.context())
            .map_err(|err| from_kind(origin, err))?;
        // A kind says what a part is (DESIGN.md rule 1): checked here, for
        // every catalog's kinds, before the catalog registers anything.
        guide
            .check(&assignment)
            .map_err(|err| from_kind(origin, err))?;
        let options = PartOptions::new(assignment.context(), model.options.clone());
        catalog
            .register_part(&mut registry, &assignment, options)
            .map_err(|err| from_kind(origin, err))?;
        checks.push((by, key.to_string(), matched));
    }

    // Every entry reaches the parts it names: no other entry, and no class
    // the part's own symbol gives it, comes first.
    let pin_counts = pin_counts(&netlist);
    for (by, key, matched) in &checks {
        for &index in matched {
            let decl = &netlist.components[index];
            let pins = pin_counts
                .get(decl.reference.as_str())
                .copied()
                .unwrap_or(0);
            let Ok(classified) = registry.classify_with_key(decl, pins) else {
                // Refused by the registry: the survey names why.
                continue;
            };
            if classified.key.as_deref() == Some(key.as_str()) {
                continue;
            }
            let why = match &classified.key {
                Some(other) => {
                    let field = [KeyField::Part, KeyField::Mpn, KeyField::Value]
                        .into_iter()
                        .find(|field| field.of(decl).as_deref() == Some(other.as_str()))
                        .map_or_else(|| "key".to_string(), |field| field.to_string());
                    format!(
                        "the registry reaches it first by its {field} {other:?}; assign by \
                         {field} = {other:?} instead"
                    )
                }
                None => format!(
                    "its symbol makes it {} by itself, and a model cannot change that",
                    class_phrase(&classified.class)
                ),
            };
            return Err(ProjectError::message(format!(
                "board {}: [[board.model]] {by} = {key:?} does not reach {}: {why}",
                spec.name, decl.reference
            )));
        }
    }

    // A part the base registrations place by a key two catalogs place is
    // ambiguous, unless an entry gave that key a kind itself.
    if from_base {
        for decl in &netlist.components {
            let pins = pin_counts
                .get(decl.reference.as_str())
                .copied()
                .unwrap_or(0);
            let Ok(classified) = registry.classify_with_key(decl, pins) else {
                continue;
            };
            let Some(key) = classified.key else {
                continue;
            };
            if assigned.contains_key(&key) {
                continue;
            }
            let providers = catalog.base_key_clash(&key);
            if providers.len() >= 2 {
                return Err(ProjectError::message(format!(
                    "board {}: {} is placed by {key:?}, and {} catalogs place parts by that key, \
                     {}; give the key one kind with a [[board.model]] of its own, or leave one \
                     catalog out",
                    spec.name,
                    decl.reference,
                    providers.len(),
                    and_list(&providers)
                )));
            }
        }
    }

    let survey = BoardSurvey::of(&netlist, &registry);
    Ok(Prepared {
        netlist,
        registry,
        survey,
    })
}

/// Netlist pins per reference, as the build counts them.
fn pin_counts(netlist: &ParsedNetlist) -> BTreeMap<&str, usize> {
    let mut counts = BTreeMap::new();
    for net in &netlist.nets {
        for node in &net.nodes {
            *counts.entry(node.reference.as_str()).or_insert(0) += 1;
        }
    }
    counts
}

/// A hint for a key that matches no part: the part whose other field is the
/// key, or the fields this field's parts carry.
fn no_match_hint(netlist: &ParsedNetlist, by: KeyField, key: &str) -> String {
    for decl in &netlist.components {
        for field in [KeyField::Part, KeyField::Mpn, KeyField::Value] {
            if field != by && field.of(decl).as_deref() == Some(key) {
                return format!(
                    "; {} has {field} {key:?} — match it by {field} = {key:?}",
                    decl.reference
                );
            }
        }
    }
    let mut keys: Vec<String> = netlist
        .components
        .iter()
        .filter_map(|decl| by.of(decl))
        .collect::<BTreeSet<_>>()
        .into_iter()
        .collect();
    if keys.is_empty() {
        return format!("; no part on it carries a {by}");
    }
    let more = keys.len().saturating_sub(12);
    keys.truncate(12);
    let listed = keys
        .iter()
        .map(|key| format!("{key:?}"))
        .collect::<Vec<_>>()
        .join(", ");
    if more > 0 {
        format!("; the {by}s it has include {listed} and {more} more")
    } else {
        format!("; the {by}s it has are {listed}")
    }
}

/// "a resistor", "a connector" — what a class makes a part, for a sentence.
fn class_phrase(class: &Classification) -> &'static str {
    match class {
        Classification::Passive { .. } => "a passive (a resistor, capacitor or inductor)",
        Classification::Boundary => "a connector",
        Classification::Jumper { .. } => "a jumper",
        Classification::Switch { .. } => "a switch",
        Classification::Pwl { .. } => "an element",
        Classification::Probe => "a test point",
        Classification::Mechanical => "a mechanical part",
        Classification::Registered => "a model",
    }
}

/// `"the part kinds are \"a\", \"b\""`, or that there are none.
fn kinds_sentence(what: &str, kinds: &[String]) -> String {
    if kinds.is_empty() {
        format!("this catalog has no {what} kinds")
    } else {
        let names: Vec<&str> = kinds.iter().map(String::as_str).collect();
        format!("the {what} kinds are {}", quoted_list(&names))
    }
}

fn quoted_list(items: &[&str]) -> String {
    items
        .iter()
        .map(|item| format!("{item:?}"))
        .collect::<Vec<_>>()
        .join(", ")
}

// ============================================================
// Where a wire may land
// ============================================================

/// The boards' surveys and the bench components' pins, for checking the
/// endpoints a project names.
struct Places<'a> {
    boards: &'a BTreeMap<String, BoardSurvey>,
    bench: &'a BTreeMap<String, Vec<String>>,
    /// The supplies wires with volts create, as `Name.Pin`.
    supplies: &'a BTreeSet<String>,
}

/// An endpoint as the project spells it.
fn endpoint_text(endpoint: &EndpointRef) -> String {
    match &endpoint.connector {
        Some(connector) => format!("{}.{connector}.{}", endpoint.board, endpoint.pin),
        None => format!("{}.{}", endpoint.board, endpoint.pin),
    }
}

impl Places<'_> {
    fn endpoint(what: &str, text: &str) -> Result<EndpointRef, ProjectError> {
        EndpointRef::parse(text).map_err(|_| {
            ProjectError::message(format!(
                "{what}: {text:?} is not an endpoint; a board's is Board.Connector.Pin, a bench \
                 component's Name.Pin"
            ))
        })
    }

    /// Check one end of a wire: a connector pin on a board, a bench
    /// component's pin, a supply another wire's volts create, or — `source`,
    /// the `from` of a wire with volts — a supply of its own.
    fn wire_end(&self, what: &str, text: &str, source: bool) -> Result<EndpointRef, ProjectError> {
        let endpoint = Self::endpoint(what, text)?;
        if let Some(survey) = self.boards.get(&endpoint.board) {
            let board = &endpoint.board;
            let Some(connector) = &endpoint.connector else {
                return Err(ProjectError::message(format!(
                    "{what}: {text} names a board without its connector; a wire lands on a \
                     connector pin, {board}.Connector.Pin; {}",
                    connectors_of(board, survey)
                )));
            };
            let Some(report) = survey.connector(connector) else {
                let why = if survey.has_part(connector) {
                    format!("{connector} is not a connector, and a wire lands on a connector pin")
                } else {
                    format!("{board} has no part {connector}")
                };
                return Err(ProjectError::message(format!(
                    "{what}: {why}; {}",
                    connectors_of(board, survey)
                )));
            };
            if !survey.has_pin(connector, &endpoint.pin) {
                return Err(ProjectError::message(format!(
                    "{what}: {connector} has no pin {:?}; its pins are {}",
                    endpoint.pin,
                    report.pin_list()
                )));
            }
            return Ok(endpoint);
        }
        if let Some(pins) = self.bench.get(&endpoint.board) {
            if endpoint.connector.is_some() || !pins.contains(&endpoint.pin) {
                return Err(ProjectError::message(format!(
                    "{what}: {text}: bench component {} has pins {}, named Name.Pin",
                    endpoint.board,
                    pins.join(", ")
                )));
            }
            return Ok(endpoint);
        }
        if source || self.supplies.contains(&endpoint_text(&endpoint)) {
            return Ok(endpoint);
        }
        Err(ProjectError::message(format!(
            "{what}: {text}: {} is not a board or bench component in this project (boards: \
             {}; components: {}); a name of its own is a supply, created by the from of a \
             wire with volts",
            endpoint.board,
            names(self.boards.keys()),
            names(self.bench.keys())
        )))
    }

    /// The connector `text` names, `Board.Connector`: its board's name, its
    /// reference and its report.
    fn connector<'s>(
        &'s self,
        what: &str,
        text: &str,
    ) -> Result<(&'s str, &'s ConnectorReport), ProjectError> {
        let Some((board, reference)) = text.split_once('.') else {
            return Err(ProjectError::message(format!(
                "{what}: {text:?} names a connector as Board.Connector"
            )));
        };
        let Some((name, survey)) = self.boards.get_key_value(board) else {
            return Err(ProjectError::message(format!(
                "{what}: {board} is not a board in this project; its boards: {}",
                names(self.boards.keys())
            )));
        };
        match survey.connector(reference) {
            Some(report) => Ok((name.as_str(), report)),
            None => {
                let why = if survey.has_part(reference) {
                    format!("{reference} is not a connector, and a mate joins two connectors")
                } else {
                    format!("{board} has no part {reference}")
                };
                Err(ProjectError::message(format!(
                    "{what}: {why}; {}",
                    connectors_of(board, survey)
                )))
            }
        }
    }

    /// The pin pairs a mate joins: each pin of `a` with `b`'s pin of the
    /// same number, or the pairs its `map` names.
    fn mate(&self, mate: &MateSpec) -> Result<Vec<(EndpointRef, EndpointRef)>, ProjectError> {
        let what = format!("[[mate]] {} to {}", mate.a, mate.b);
        let (a_board, a) = self.connector(&what, &mate.a)?;
        let (b_board, b) = self.connector(&what, &mate.b)?;
        if a_board == b_board && a.reference == b.reference {
            return Err(ProjectError::message(format!(
                "{what}: a mate joins two different connectors"
            )));
        }
        let has = |report: &ConnectorReport, pin: &str| report.pins.iter().any(|s| s.pin == pin);
        let pairs: Vec<(String, String)> = match &mate.map {
            None => {
                let missing: Vec<&str> = a
                    .pins
                    .iter()
                    .map(|site| site.pin.as_str())
                    .filter(|pin| !has(b, pin))
                    .collect();
                if !missing.is_empty() {
                    return Err(ProjectError::message(format!(
                        "{what}: {} has no pin {} that {} has; without a map each pin of a joins \
                         b's pin of the same number, so a is the side with fewer pins (the \
                         module's fingers, the cable's plug), and a cable that joins other \
                         numbers says which with map = [[\"a pin\", \"b pin\"], …]",
                        mate.b,
                        missing.join(", "),
                        mate.a
                    )));
                }
                a.pins
                    .iter()
                    .map(|site| (site.pin.clone(), site.pin.clone()))
                    .collect()
            }
            Some(map) => {
                if map.is_empty() {
                    return Err(ProjectError::message(format!(
                        "{what}: map = [] joins no pins; leave map out to join every pin of a \
                         by number"
                    )));
                }
                let mut seen_a = BTreeSet::new();
                let mut seen_b = BTreeSet::new();
                for [pin_a, pin_b] in map {
                    for (side, report, pin, seen) in [
                        (&mate.a, a, pin_a, &mut seen_a),
                        (&mate.b, b, pin_b, &mut seen_b),
                    ] {
                        if !has(report, pin) {
                            return Err(ProjectError::message(format!(
                                "{what}: map names pin {pin:?} of {side}, which has pins {}",
                                report.pin_list()
                            )));
                        }
                        if !seen.insert(pin.as_str()) {
                            return Err(ProjectError::message(format!(
                                "{what}: map names pin {pin:?} of {side} twice; a pin has one \
                                 mate"
                            )));
                        }
                    }
                }
                map.iter()
                    .map(|[pin_a, pin_b]| (pin_a.clone(), pin_b.clone()))
                    .collect()
            }
        };
        let end = |board: &str, report: &ConnectorReport, pin: String| EndpointRef {
            board: board.to_string(),
            connector: Some(report.reference.clone()),
            pin,
        };
        Ok(pairs
            .into_iter()
            .map(|(pin_a, pin_b)| (end(a_board, a, pin_a), end(b_board, b, pin_b)))
            .collect())
    }

    fn board_part<'s>(
        &'s self,
        what: &str,
        dotted: &str,
    ) -> Result<(&'s str, &'s BoardSurvey, String), ProjectError> {
        let Some((board, reference)) = dotted.split_once('.') else {
            return Err(ProjectError::message(format!(
                "{what}: {dotted:?} names a part as Board.Ref"
            )));
        };
        let Some((name, survey)) = self.boards.get_key_value(board) else {
            return Err(ProjectError::message(format!(
                "{what}: {board} is not a board in this project; its boards: {}",
                names(self.boards.keys())
            )));
        };
        Ok((name.as_str(), survey, reference.to_string()))
    }

    fn switch_pole(&self, switch: &SwitchSpec) -> Result<(), ProjectError> {
        let what = format!("[[switch]] {} pole {}", switch.part, switch.pole);
        let (board, survey, reference) = self.board_part(&what, &switch.part)?;
        match survey.class_of(&reference) {
            Some(Classification::Switch { poles }) if switch.pole < poles.len() => Ok(()),
            Some(Classification::Switch { poles }) => Err(ProjectError::message(format!(
                "{what}: {reference} has {} pole{}, numbered from 0",
                poles.len(),
                if poles.len() == 1 { "" } else { "s" }
            ))),
            _ => Err(ProjectError::message(format!(
                "{what}: {reference} is not a switch on {board}; its switches: {}",
                parts_of(survey, |class| matches!(
                    class,
                    Classification::Switch { .. }
                ))
            ))),
        }
    }

    fn jumper(&self, jumper: &JumperSpec) -> Result<(), ProjectError> {
        let what = format!("[[jumper]] {}", jumper.part);
        let (board, survey, reference) = self.board_part(&what, &jumper.part)?;
        match survey.class_of(&reference) {
            Some(Classification::Jumper { .. }) => Ok(()),
            _ => Err(ProjectError::message(format!(
                "{what}: {reference} is not a jumper on {board}; its jumpers: {}",
                parts_of(survey, |class| matches!(
                    class,
                    Classification::Jumper { .. }
                ))
            ))),
        }
    }

    fn part_pin(&self, what: &str, dotted: &str) -> Result<(), ProjectError> {
        let (board, survey, rest) = self.board_part(what, dotted)?;
        let Some((reference, pin)) = rest.split_once('.') else {
            return Err(ProjectError::message(format!(
                "{what}: {dotted:?} names a pin as Board.Ref.Pin"
            )));
        };
        match survey.pins_of(reference) {
            None => Err(ProjectError::message(format!(
                "{what}: {board} has no part {reference}"
            ))),
            Some(pins) if pins.iter().any(|site| site.pin == pin) => Ok(()),
            Some(pins) => Err(ProjectError::message(format!(
                "{what}: {reference} has no pin {pin:?}; its pins are {}",
                pins.iter()
                    .map(|site| site.pin.as_str())
                    .collect::<Vec<_>>()
                    .join(", ")
            ))),
        }
    }
}

/// `"its connectors are J1, J2"`, or that it has none.
fn connectors_of(board: &str, survey: &BoardSurvey) -> String {
    if survey.connectors.is_empty() {
        format!("{board} has no connectors")
    } else {
        format!("{board}'s connectors are {}", survey.connector_list())
    }
}

/// The references of a survey's parts whose class `pick` accepts.
fn parts_of(survey: &BoardSurvey, pick: impl Fn(&Classification) -> bool) -> String {
    let found: Vec<&str> = survey
        .classified()
        .filter(|(_, class)| pick(class))
        .map(|(reference, _)| reference)
        .collect();
    if found.is_empty() {
        "none".to_string()
    } else {
        found.join(", ")
    }
}

fn names<'a>(keys: impl Iterator<Item = &'a String>) -> String {
    let all: Vec<&str> = keys.map(String::as_str).collect();
    if all.is_empty() {
        "none".to_string()
    } else {
        all.join(", ")
    }
}

#[cfg(test)]
mod tests {
    use rstest::rstest;

    use super::*;

    #[rstest]
    fn a_power_wire_keeps_its_voltage() {
        let project = Project::parse(
            r#"
            [[wire]]
            from = "CARRIER.5V"
            to = "EC32.J203.41"
            volts = 5.0

            [[switch]]
            part = "EC32.S301"
            pole = 1
            state = "closed"
            "#,
        )
        .expect("the text is a project");
        assert_eq!(project.wires()[0].volts, Some(5.0));
        assert_eq!(project.wires()[0].to, "EC32.J203.41");
    }

    #[rstest]
    #[case::wire_field("[[wire]]\nfrom = \"A.1\"\nto = \"B.2\"\ncolour = \"red\"\n", "colour")]
    #[case::board_field("[[board]]\nname = \"A\"\nkind = \"netlist\"\ncore = \"x\"\n", "core")]
    #[case::model_field(
        "[[board]]\nname = \"A\"\nkind = \"netlist\"\n[[board.model]]\nvalue = \"X\"\nkind = \"y\"\nref = \"U1\"\n",
        "ref"
    )]
    #[case::table("[[bord]]\nname = \"A\"\n", "bord")]
    fn an_unknown_field_is_refused_naming_it(#[case] text: &str, #[case] field: &str) {
        let err = Project::parse(text).expect_err("the field is not part of the format");
        let message = err.to_string();
        assert!(message.contains("project does not parse"), "{message}");
        assert!(message.contains(field), "{message}");
    }

    #[rstest]
    fn board_models_attach_to_their_board() {
        let project = Project::parse(
            r#"
            [[board]]
            name = "A"
            kind = "netlist"
            netlist = "a.net"

            [[board.model]]
            value = "ADS122U04"
            kind = "ads122u04"
            [board.model.options]
            pins = "tssop16"

            [[board.model]]
            mpn = "X-1"
            kind = "mechanical"

            [[board]]
            name = "B"
            kind = "p2-ec32mb"
            "#,
        )
        .expect("the text is a project");
        let boards = project.boards();
        assert_eq!(boards.len(), 2);
        assert_eq!(boards[0].model.len(), 2);
        assert_eq!(
            boards[0].model[0].key(),
            Some((KeyField::Value, "ADS122U04"))
        );
        assert_eq!(
            boards[0].model[0].options.get("pins"),
            Some(&toml::Value::String("tssop16".to_string()))
        );
        assert_eq!(boards[0].model[1].key(), Some((KeyField::Mpn, "X-1")));
        assert!(boards[1].model.is_empty());
    }

    #[rstest]
    #[case::dotted("[[board]]\nname = \"A.B\"\nkind = \"netlist\"\n")]
    #[case::twice(
        "[[board]]\nname = \"A\"\nkind = \"netlist\"\n[[component]]\nname = \"A\"\nkind = \"x\"\n"
    )]
    fn a_name_that_cannot_prefix_an_endpoint_is_refused(#[case] text: &str) {
        let err = Project::parse(text).expect_err("the name cannot be an endpoint prefix");
        assert!(err.to_string().contains("name \"A"), "{err}");
    }

    #[rstest]
    fn options_are_taken_and_the_rest_refused_naming_what_the_kind_takes() {
        let table: toml::Table = toml::from_str("pins = \"soic8\"\nspeed = 3\n").unwrap();
        let mut options = PartOptions::new("ctx", table);
        assert_eq!(
            options.choice("pins", &["soic8", "by-function"]).unwrap(),
            Some("soic8")
        );
        let err = options.finish().expect_err("speed is not an option");
        assert_eq!(
            err.to_string(),
            "ctx: unknown option \"speed\"; this kind takes \"pins\""
        );

        let table: toml::Table = toml::from_str("pins = \"dip8\"\n").unwrap();
        let mut options = PartOptions::new("ctx", table);
        let err = options
            .choice("pins", &["soic8", "by-function"])
            .expect_err("dip8 is not offered");
        assert_eq!(
            err.to_string(),
            "ctx: options.pins = \"dip8\" is not one this kind offers; it offers \"soic8\", \
             \"by-function\""
        );

        let table: toml::Table = toml::from_str("poles = [[\"1\", \"2\"], [\"3\"]]\n").unwrap();
        let mut options = PartOptions::new("ctx", table);
        assert!(options.pairs("poles").is_err());
    }
}
