//! A system, written down.
//!
//! embsim runs one [`System`]: boards, bench components, harness wires, and a
//! scenario. A project is that list in a file. The file names kinds. A
//! [`Catalog`] turns a kind into the board or component it names. The file
//! does not contain the behavior of a part.
//!
//! ```toml
//! [[board]]
//! name = "EC32"
//! kind = "p2-ec32mb"
//! core = "held-in-reset"
//!
//! [[wire]]
//! from = "CARRIER.5V"
//! to = "EC32.J203.41"
//! volts = 5.0
//!
//! [[switch]]
//! part = "EC32.S301"
//! pole = 1
//! state = "closed"
//! ```
//!
//! A wire with `volts` is a power source ([`Harness::power`]). A wire without
//! it joins two endpoints ([`Harness::connect`]). A power source's own
//! endpoint, `CARRIER.5V` above, does not need a component: the harness
//! creates that net.

use std::collections::{HashMap, HashSet};
use std::path::Path;

use serde::Deserialize;

use crate::netlist;
use crate::survey::BoardSurvey;
use crate::{Board, Component, EndpointRef, Harness, JumperState, PartRegistry, Scenario, System};

/// Turns a kind named in a project into the board or component it is.
pub trait Catalog {
    /// Build the board `spec` names.
    fn board(&self, spec: &BoardSpec) -> Result<Board, ProjectError>;

    /// The checklist for `spec`: which parts still need a model, and which
    /// connectors a wire may use. `kind = "netlist"` is surveyed by the loader
    /// from the file itself, and a catalog answers that kind with an error.
    fn survey(&self, spec: &BoardSpec) -> Result<BoardSurvey, ProjectError>;

    /// Build the bench component `spec` names.
    fn component(&self, spec: &ComponentSpec) -> Result<Box<dyn Component>, ProjectError>;
}

/// Why a project could not be read or built.
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

impl std::fmt::Display for ProjectError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.message)
    }
}

impl std::error::Error for ProjectError {}

/// One board: a name in the system, a catalog kind, and the fields that kind reads.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BoardSpec {
    /// System name. Endpoints are `{name}.Ref.Pin`.
    pub name: String,
    /// Catalog kind (`"p2-ec32mb"`, `"netlist"`).
    pub kind: String,
    /// Processor the kind seats, when it has a slot (`"held-in-reset"`).
    #[serde(default)]
    pub core: Option<String>,
    /// Netlist path for `kind = "netlist"`, relative to the directory passed
    /// to [`Project::instantiate_in`].
    #[serde(default)]
    pub netlist: Option<String>,
}

/// One bench component: a name in the system and a catalog kind.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ComponentSpec {
    /// System name. Endpoints are `{name}.Pin`.
    pub name: String,
    /// Catalog kind.
    pub kind: String,
}

/// One harness wire. `volts` makes it a power source on `from`.
#[derive(Debug, Deserialize)]
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
#[derive(Debug, Deserialize)]
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
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct JumperSpec {
    /// Dotted reference, board included.
    pub part: String,
    /// Open or closed.
    pub state: ContactState,
}

/// Two pins whose nets become one.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PinShortSpec {
    /// Dotted pin endpoint.
    pub a: String,
    /// Dotted pin endpoint.
    pub b: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ProjectFile {
    #[serde(default)]
    board: Vec<BoardSpec>,
    #[serde(default)]
    component: Vec<ComponentSpec>,
    #[serde(default)]
    wire: Vec<WireSpec>,
    #[serde(default)]
    switch: Vec<SwitchSpec>,
    #[serde(default)]
    jumper: Vec<JumperSpec>,
    #[serde(default)]
    pin_short: Vec<PinShortSpec>,
}

/// A parsed project. [`Self::instantiate`] asks `catalog` for every kind and
/// returns the [`System`], not yet started.
#[derive(Debug)]
pub struct Project {
    file: ProjectFile,
}

impl Project {
    /// Parse `text` as a project. Kinds are not resolved yet.
    pub fn parse(text: &str) -> Result<Self, ProjectError> {
        let file: ProjectFile = toml::from_str(text)
            .map_err(|err| ProjectError::message(format!("project does not parse: {err}")))?;
        Ok(Self { file })
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

    /// Build the [`System`] this project describes.
    ///
    /// Paths in the file are relative to `.`. [`Self::instantiate_in`] takes
    /// the directory a `netlist` path is relative to.
    ///
    /// The caller starts it. A test that wants the attach-time circuit and
    /// no later wake calls [`System::hold_time`] before [`System::start`].
    ///
    /// A board whose survey still names a part with no model, or a wire that
    /// names a pin the netlist does not have, is refused. The error is the
    /// survey: the parts left to model, and the connectors a wire may use.
    pub fn instantiate(&self, catalog: &dyn Catalog) -> Result<System, ProjectError> {
        self.instantiate_in(catalog, Path::new("."))
    }

    /// [`Self::instantiate`] with `base` as the directory netlist paths are
    /// relative to.
    pub fn instantiate_in(
        &self,
        catalog: &dyn Catalog,
        base: &Path,
    ) -> Result<System, ProjectError> {
        let mut surveys: HashMap<String, BoardSurvey> = HashMap::new();
        let mut system = System::new();
        for board in &self.file.board {
            let (built, survey) = prepare_board(board, catalog, base)?;
            if !survey.compliant() {
                return Err(ProjectError::message(format!(
                    "board {} does not comply:\n{survey}",
                    board.name
                )));
            }
            surveys.insert(board.name.clone(), survey);
            system = system.board(&board.name, built);
        }

        let mut bench_pins: HashMap<String, HashSet<String>> = HashMap::new();
        for component in &self.file.component {
            let built = catalog.component(component)?;
            let pins = built
                .pins()
                .iter()
                .flat_map(|pin| {
                    let mut names = vec![pin.number.to_string()];
                    if let Some(name) = pin.name {
                        names.push(name.to_string());
                    }
                    names
                })
                .collect();
            bench_pins.insert(component.name.clone(), pins);
            system = system.component(&component.name, built);
        }

        let mut harness = Harness::new();
        for wire in &self.file.wire {
            let from = parse_endpoint(&wire.from)?;
            let to = parse_endpoint(&wire.to)?;
            require_endpoint(&from, &surveys, &bench_pins, wire.volts.is_some())?;
            require_endpoint(&to, &surveys, &bench_pins, false)?;
            harness = match wire.volts {
                Some(volts) => harness.power(from, to, volts),
                None => harness.connect(from, to),
            };
        }
        if !self.file.wire.is_empty() {
            system = system.harness(harness);
        }

        for switch in &self.file.switch {
            require_part(&switch.part, &surveys)?;
        }
        for jumper in &self.file.jumper {
            require_part(&jumper.part, &surveys)?;
        }
        for short in &self.file.pin_short {
            require_endpoint(&parse_endpoint(&short.a)?, &surveys, &bench_pins, false)?;
            require_endpoint(&parse_endpoint(&short.b)?, &surveys, &bench_pins, false)?;
        }

        let mut scenario = Scenario::default();
        for switch in &self.file.switch {
            scenario = scenario.switch(&switch.part, switch.pole, switch.state.into());
        }
        for jumper in &self.file.jumper {
            scenario = scenario.jumper(&jumper.part, jumper.state.into());
        }
        for short in &self.file.pin_short {
            scenario = scenario.pin_short(&short.a, &short.b);
        }
        system = system.scenario(scenario);
        Ok(system)
    }
}

fn prepare_board(
    spec: &BoardSpec,
    catalog: &dyn Catalog,
    base: &Path,
) -> Result<(Board, BoardSurvey), ProjectError> {
    if spec.kind == "netlist" {
        let relative = spec.netlist.as_deref().ok_or_else(|| {
            ProjectError::message(format!(
                "board {}: kind \"netlist\" needs a netlist path",
                spec.name
            ))
        })?;
        let path = base.join(relative);
        let text = std::fs::read_to_string(&path).map_err(|err| {
            ProjectError::message(format!("board {}: cannot read {}: {err}", spec.name, path.display()))
        })?;
        let parsed = netlist::parse(&text).map_err(|err| {
            ProjectError::message(format!("board {}: netlist {}: {err}", spec.name, path.display()))
        })?;
        let mut registry = PartRegistry::new();
        registry.classify_unnamed_by_reference(true);
        let survey = BoardSurvey::of(&parsed, &registry);
        if !survey.compliant() {
            return Err(ProjectError::message(format!(
                "board {} does not comply:\n{survey}",
                spec.name
            )));
        }
        let board = Board::from_netlist(parsed, &registry)
            .map_err(|err| ProjectError::message(format!("board {}: {err}", spec.name)))?;
        return Ok((board, survey));
    }
    let survey = catalog.survey(spec)?;
    if !survey.compliant() {
        return Err(ProjectError::message(format!(
            "board {} does not comply:\n{survey}",
            spec.name
        )));
    }
    let board = catalog.board(spec)?;
    Ok((board, survey))
}

fn parse_endpoint(text: &str) -> Result<EndpointRef, ProjectError> {
    EndpointRef::parse(text)
        .map_err(|err| ProjectError::message(format!("endpoint {text}: {err}")))
}

/// `allow_fresh` is the `from` end of a power wire: a name that is neither a
/// board nor a bench component is a source the harness creates (`CARRIER.5V`).
fn require_endpoint(
    endpoint: &EndpointRef,
    boards: &HashMap<String, BoardSurvey>,
    bench_pins: &HashMap<String, HashSet<String>>,
    allow_fresh: bool,
) -> Result<(), ProjectError> {
    if let Some(survey) = boards.get(&endpoint.board) {
        let Some(connector) = &endpoint.connector else {
            return Err(ProjectError::message(format!(
                "{}.{} is a board pin written without its connector; use {}.<connector>.{}",
                endpoint.board, endpoint.pin, endpoint.board, endpoint.pin
            )));
        };
        if !survey.has_pin(connector, &endpoint.pin) {
            return Err(ProjectError::message(format!(
                "{board}.{connector}.{pin} is not a pin on {connector}",
                board = endpoint.board,
                pin = endpoint.pin
            )));
        }
        return Ok(());
    }
    if let Some(pins) = bench_pins.get(&endpoint.board) {
        if endpoint.connector.is_some() {
            return Err(ProjectError::message(format!(
                "{}.{} is a bench component; its pins have no connector in the name",
                endpoint.board, endpoint.pin
            )));
        }
        if !pins.contains(&endpoint.pin) {
            return Err(ProjectError::message(format!(
                "{}.{} is not a pin of {}",
                endpoint.board, endpoint.pin, endpoint.board
            )));
        }
        return Ok(());
    }
    if allow_fresh {
        return Ok(());
    }
    Err(ProjectError::message(format!(
        "{}.{} is not a board connector pin or a bench component pin",
        endpoint.board, endpoint.pin
    )))
}

fn require_part(dotted: &str, boards: &HashMap<String, BoardSurvey>) -> Result<(), ProjectError> {
    let endpoint = parse_endpoint(dotted)?;
    let Some(survey) = boards.get(&endpoint.board) else {
        return Err(ProjectError::message(format!(
            "{dotted}: {} is not a board in this project",
            endpoint.board
        )));
    };
    let reference = endpoint.connector.as_deref().unwrap_or(&endpoint.pin);
    if survey.has_part(reference) {
        Ok(())
    } else {
        Err(ProjectError::message(format!(
            "{dotted}: {reference} is not a part on {}",
            endpoint.board
        )))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
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

    #[test]
    fn an_unknown_field_is_refused() {
        let err = Project::parse(
            r#"
            [[wire]]
            from = "A.1"
            to = "B.2"
            colour = "red"
            "#,
        )
        .expect_err("colour is not a wire field");
        assert!(err.to_string().contains("project does not parse"), "{err}");
    }
}
