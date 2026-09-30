//! What a netlist still asks of a project.
//!
//! [`BoardSurvey::of`] classifies every part through the registry the board
//! will build with (`DESIGN.md` rule 1), and checks what the build checks
//! without building anything:
//!
//! - a part the registry has no class for is [`BoardSurvey::needs_model`],
//!   named by reference, part name, value and manufacturer part number — the
//!   keys a model can be assigned by;
//! - a part whose registered model states its pins ([`ModelFacade`]), a
//!   switch's poles and an element's pins are compared with the netlist's
//!   pins in both directions, and a disagreement is
//!   [`BoardSurvey::mismatched`], with both lists — the usual cause is a pin
//!   table for another package, or for a netlist that names pins by
//!   function;
//! - a class the part cannot satisfy (a resistor with three pins) is
//!   [`BoardSurvey::refused`];
//! - a connector is listed with its pins in [`BoardSurvey::connectors`],
//!   because those pins are the endpoints a harness wire may join to another
//!   board, a bench component or a supply.
//!
//! A survey with anything in the first three lists is not a board yet: the
//! build would refuse it. A part whose model was registered without a
//! facade ([`PartRegistry::register`]) is checked by the build alone, on the
//! component itself.

use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::fmt;

use crate::netlist::{NetDecl, NodeDecl, ParsedNetlist};
use crate::registry::{normalize_part, Classification, ModelFacade, PartRegistry, RegistryError};

/// One pin of a part, as the netlist draws it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PinSite {
    /// Netlist pin id (`"41"`).
    pub pin: String,
    /// The symbol's pin name, when the export carries one (`"5V"`).
    pub pinfunction: Option<String>,
    /// The net this pin sits on.
    pub net: String,
}

/// A part the registry could not classify. The project has to give it a model.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UnmodelledPart {
    /// Reference designator (`"U100"`).
    pub reference: String,
    /// Normalized part name. Empty when the export has no libsource.
    pub part: String,
    /// Value field. On a transcribed netlist this is the part's name.
    pub value: String,
    /// Manufacturer part number, when the export carries one.
    pub mpn: Option<String>,
    /// Pins the netlist attaches, in pin order.
    pub pins: Vec<PinSite>,
}

/// A part whose model, switch poles or element pins name other pins than
/// the netlist gives the part.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FacadeMismatch {
    /// Reference designator.
    pub reference: String,
    /// Value field.
    pub value: String,
    /// Manufacturer part number, when the export carries one.
    pub mpn: Option<String>,
    /// What classified the part: the model and its pin table as the
    /// registrant named them ([`ModelFacade::model`]), `"a switch"` or
    /// `"an element"`.
    pub model: String,
    /// The pins it declares, in declaration order.
    pub declared: Vec<String>,
    /// The pins the netlist gives the part, in pin order.
    pub netlist: Vec<String>,
}

/// A connector: a boundary the harness may wire, and the pins it offers.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ConnectorReport {
    /// Reference designator (`"J203"`).
    pub reference: String,
    /// Value field.
    pub value: String,
    /// Pins, in pin order.
    pub pins: Vec<PinSite>,
}

impl ConnectorReport {
    /// The connector's pin ids, comma-separated, in pin order — the list an
    /// error names when a wire misses them.
    pub fn pin_list(&self) -> String {
        self.pins
            .iter()
            .map(|site| site.pin.as_str())
            .collect::<Vec<_>>()
            .join(", ")
    }
}

/// One part as the survey found it.
#[derive(Debug, Clone, PartialEq)]
struct SurveyedPart {
    /// Its class; `None` when the registry refused it.
    class: Option<Classification>,
    /// Its pins, in pin order.
    pins: Vec<PinSite>,
}

/// The checklist for one netlist.
#[derive(Debug, Clone, PartialEq)]
pub struct BoardSurvey {
    /// Components in the netlist, including ones with no pins.
    pub part_count: usize,
    /// Parts the registry classified, connectors included.
    pub modelled: usize,
    /// Parts with no class, in reference order. The board will not build
    /// while this is non-empty.
    pub needs_model: Vec<UnmodelledPart>,
    /// Parts whose declared pins disagree with the netlist's, in reference
    /// order. The board will not build while this is non-empty.
    pub mismatched: Vec<FacadeMismatch>,
    /// A class the part cannot satisfy, such as a resistor with three pins.
    pub refused: Vec<String>,
    /// Connectors, in reference order.
    pub connectors: Vec<ConnectorReport>,
    /// Every part, by reference.
    parts: BTreeMap<String, SurveyedPart>,
}

impl BoardSurvey {
    /// Survey `netlist` with `registry`: every part classified, every
    /// stated facade checked against the netlist's pins.
    ///
    /// Pin counts and facade checks are the ones
    /// [`crate::Board::from_netlist`] makes, so a part this survey accepts
    /// is a part that build accepts, except where a model registered with
    /// no facade builds a component whose own pins disagree.
    pub fn of(netlist: &ParsedNetlist, registry: &PartRegistry) -> Self {
        let mut pins_by_ref: HashMap<&str, Vec<&NodeDecl>> = HashMap::new();
        let mut net_of: HashMap<(&str, &str), &NetDecl> = HashMap::new();
        for net in &netlist.nets {
            for node in &net.nodes {
                pins_by_ref
                    .entry(node.reference.as_str())
                    .or_default()
                    .push(node);
                net_of
                    .entry((node.reference.as_str(), node.pin.as_str()))
                    .or_insert(net);
            }
        }

        let mut needs_model = Vec::new();
        let mut mismatched = Vec::new();
        let mut refused = Vec::new();
        let mut connectors = Vec::new();
        let mut parts = BTreeMap::new();
        let mut modelled = 0usize;

        for decl in &netlist.components {
            let nodes = pins_by_ref
                .get(decl.reference.as_str())
                .map(Vec::as_slice)
                .unwrap_or(&[]);
            let sites = pin_sites(nodes, &net_of);
            let netlist_pins: Vec<&str> = nodes.iter().map(|node| node.pin.as_str()).collect();

            let class = match registry.classify(decl, nodes.len()) {
                Ok(class) => {
                    modelled += 1;
                    let declared: Option<(String, Vec<String>)> = match &class {
                        Classification::Registered => registry
                            .facade(decl)
                            .map(|ModelFacade { model, pins }| (model.clone(), pins.clone())),
                        Classification::Switch { poles } => Some((
                            "a switch".to_string(),
                            poles
                                .iter()
                                .flat_map(|pole| [pole.a.clone(), pole.b.clone()])
                                .collect(),
                        )),
                        Classification::Pwl { spec } => {
                            Some(("an element".to_string(), spec.pins.clone()))
                        }
                        _ => None,
                    };
                    if let Some((model, declared)) = declared {
                        if !same_pins(&declared, &netlist_pins) {
                            mismatched.push(FacadeMismatch {
                                reference: decl.reference.clone(),
                                value: decl.value.clone(),
                                mpn: decl.mpn.clone(),
                                model,
                                declared,
                                netlist: sites.iter().map(|site| site.pin.clone()).collect(),
                            });
                        }
                    }
                    if class == Classification::Boundary {
                        connectors.push(ConnectorReport {
                            reference: decl.reference.clone(),
                            value: decl.value.clone(),
                            pins: sites.clone(),
                        });
                    }
                    Some(class)
                }
                Err(RegistryError::UnknownPart { .. }) => {
                    needs_model.push(UnmodelledPart {
                        reference: decl.reference.clone(),
                        part: normalize_part(decl),
                        value: decl.value.clone(),
                        mpn: decl.mpn.clone(),
                        pins: sites.clone(),
                    });
                    None
                }
                Err(err) => {
                    refused.push(err.to_string());
                    None
                }
            };
            parts.insert(decl.reference.clone(), SurveyedPart { class, pins: sites });
        }

        needs_model.sort_by(|a, b| natural_cmp(&a.reference, &b.reference));
        mismatched.sort_by(|a, b| natural_cmp(&a.reference, &b.reference));
        connectors.sort_by(|a, b| natural_cmp(&a.reference, &b.reference));
        refused.sort();
        Self {
            part_count: netlist.components.len(),
            modelled,
            needs_model,
            mismatched,
            refused,
            connectors,
            parts,
        }
    }

    /// Survey with only the reference-designator rules (`R` is a resistor,
    /// `J` is a connector) — the list a transcribed netlist starts from,
    /// before any model is assigned.
    pub fn bare(netlist: &ParsedNetlist) -> Self {
        let mut registry = PartRegistry::new();
        registry.classify_unnamed_by_reference(true);
        Self::of(netlist, &registry)
    }

    /// True when every part has a class the build can honour, with the pins
    /// the netlist gives it.
    pub fn compliant(&self) -> bool {
        self.needs_model.is_empty() && self.mismatched.is_empty() && self.refused.is_empty()
    }

    /// Whether `reference` is a part in the netlist.
    pub fn has_part(&self, reference: &str) -> bool {
        self.parts.contains_key(reference)
    }

    /// Whether `reference` has pin `pin`.
    pub fn has_pin(&self, reference: &str, pin: &str) -> bool {
        self.parts
            .get(reference)
            .is_some_and(|part| part.pins.iter().any(|site| site.pin == pin))
    }

    /// The class the registry gave `reference`; `None` for a part it could
    /// not classify, or one the netlist does not have.
    pub fn class_of(&self, reference: &str) -> Option<&Classification> {
        self.parts
            .get(reference)
            .and_then(|part| part.class.as_ref())
    }

    /// Every part the registry classified, with its class, in reference
    /// order (`J2` before `J10`).
    pub fn classified(&self) -> impl Iterator<Item = (&str, &Classification)> {
        let mut parts: Vec<(&str, &Classification)> = self
            .parts
            .iter()
            .filter_map(|(reference, part)| {
                part.class.as_ref().map(|class| (reference.as_str(), class))
            })
            .collect();
        parts.sort_by(|a, b| natural_cmp(a.0, b.0));
        parts.into_iter()
    }

    /// The pins of `reference`, in pin order.
    pub fn pins_of(&self, reference: &str) -> Option<&[PinSite]> {
        self.parts.get(reference).map(|part| part.pins.as_slice())
    }

    /// The connector `reference`, if the survey classified it as one.
    pub fn connector(&self, reference: &str) -> Option<&ConnectorReport> {
        self.connectors
            .iter()
            .find(|conn| conn.reference == reference)
    }

    /// The connectors' references, comma-separated, in reference order.
    pub fn connector_list(&self) -> String {
        self.connectors
            .iter()
            .map(|conn| conn.reference.as_str())
            .collect::<Vec<_>>()
            .join(", ")
    }
}

impl fmt::Display for BoardSurvey {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(
            f,
            "{} parts: {} classified, {} need a model, {} with pins the netlist does not \
             have, {} refused, {} connectors",
            self.part_count,
            self.modelled,
            self.needs_model.len(),
            self.mismatched.len(),
            self.refused.len(),
            self.connectors.len()
        )?;
        if !self.needs_model.is_empty() {
            writeln!(f, "needs a model:")?;
        }
        for part in &self.needs_model {
            write!(f, "  {}", part.reference)?;
            if !part.part.is_empty() {
                write!(f, "  part {:?}", part.part)?;
            }
            write!(f, "  value {:?}", part.value)?;
            if let Some(mpn) = &part.mpn {
                write!(f, "  mpn {mpn:?}")?;
            }
            writeln!(f, "  ({} pins)", part.pins.len())?;
        }
        if !self.mismatched.is_empty() {
            writeln!(f, "declares other pins than the netlist gives it:")?;
        }
        for part in &self.mismatched {
            write!(f, "  {}  value {:?}", part.reference, part.value)?;
            if let Some(mpn) = &part.mpn {
                write!(f, "  mpn {mpn:?}")?;
            }
            writeln!(
                f,
                ": {} declares {}; the netlist has {}",
                part.model,
                part.declared.join(", "),
                part.netlist.join(", ")
            )?;
        }
        if !self.refused.is_empty() {
            writeln!(f, "refused:")?;
        }
        for err in &self.refused {
            writeln!(f, "  {err}")?;
        }
        if !self.connectors.is_empty() {
            writeln!(f, "connectors:")?;
        }
        for conn in &self.connectors {
            writeln!(
                f,
                "  {}  value {:?}  pins {}",
                conn.reference,
                conn.value,
                conn.pin_list()
            )?;
        }
        Ok(())
    }
}

/// Whether `declared` and `netlist` name the same pins, as sets — the
/// build's two-way facade check.
fn same_pins(declared: &[String], netlist: &[&str]) -> bool {
    let declared: BTreeSet<&str> = declared.iter().map(String::as_str).collect();
    let netlist: BTreeSet<&str> = netlist.iter().copied().collect();
    declared == netlist
}

/// Unique pins in pin order (`2` before `10`). A pin named on one net is one
/// pin.
fn pin_sites(nodes: &[&NodeDecl], net_of: &HashMap<(&str, &str), &NetDecl>) -> Vec<PinSite> {
    let mut seen = BTreeSet::new();
    let mut sites = Vec::new();
    for node in nodes {
        if !seen.insert(node.pin.as_str()) {
            continue;
        }
        let net = net_of
            .get(&(node.reference.as_str(), node.pin.as_str()))
            .map(|net| net.name.clone())
            .unwrap_or_default();
        sites.push(PinSite {
            pin: node.pin.clone(),
            pinfunction: node.pinfunction.clone(),
            net,
        });
    }
    sites.sort_by(|a, b| natural_cmp(&a.pin, &b.pin));
    sites
}

/// Order two identifiers the way a reader counts them: runs of digits by
/// value (`J2` before `J10`, pin `9` before `10`), everything else by
/// character.
pub(crate) fn natural_cmp(a: &str, b: &str) -> std::cmp::Ordering {
    let (mut a, mut b) = (a, b);
    loop {
        match (a.is_empty(), b.is_empty()) {
            (true, true) => return std::cmp::Ordering::Equal,
            (true, false) => return std::cmp::Ordering::Less,
            (false, true) => return std::cmp::Ordering::Greater,
            (false, false) => {}
        }
        let a_digits = a.len() - a.trim_start_matches(|c: char| c.is_ascii_digit()).len();
        let b_digits = b.len() - b.trim_start_matches(|c: char| c.is_ascii_digit()).len();
        if a_digits > 0 && b_digits > 0 {
            let (a_run, a_rest) = a.split_at(a_digits);
            let (b_run, b_rest) = b.split_at(b_digits);
            let a_trim = a_run.trim_start_matches('0');
            let b_trim = b_run.trim_start_matches('0');
            let order = a_trim
                .len()
                .cmp(&b_trim.len())
                .then_with(|| a_trim.cmp(b_trim))
                .then_with(|| a_run.len().cmp(&b_run.len()));
            if order.is_ne() {
                return order;
            }
            a = a_rest;
            b = b_rest;
            continue;
        }
        let mut a_chars = a.chars();
        let mut b_chars = b.chars();
        let (x, y) = (a_chars.next(), b_chars.next());
        match x.cmp(&y) {
            std::cmp::Ordering::Equal => {
                a = a_chars.as_str();
                b = b_chars.as_str();
            }
            order => return order,
        }
    }
}

#[cfg(test)]
mod tests {
    use rstest::rstest;
    use vibes_behaviour::{behaviour, expect, Test};

    use super::*;
    use crate::component::{AttachError, Component, ComponentNetIo, PinDecl};
    use crate::netlist::{ComponentDecl, NetDecl, NodeDecl, ParsedNetlist};
    use crate::registry::SwitchPole;

    fn decl(reference: &str, value: &str, part: &str) -> ComponentDecl {
        ComponentDecl {
            reference: reference.to_string(),
            value: value.to_string(),
            footprint: String::new(),
            lib: String::new(),
            part: part.to_string(),
            sheetpath: "/".to_string(),
            dnp: false,
            mpn: None,
        }
    }

    fn node(reference: &str, pin: &str, pinfunction: &str) -> NodeDecl {
        NodeDecl {
            reference: reference.to_string(),
            pin: pin.to_string(),
            pinfunction: Some(pinfunction.to_string()),
            pintype: None,
        }
    }

    /// A resistor, a two-pin header, an IC nobody has modelled, and a
    /// two-pin gate package, on two nets.
    fn netlist() -> ParsedNetlist {
        ParsedNetlist {
            version: "E".to_string(),
            components: vec![
                decl("R1", "10k", "R"),
                decl("J1", "Header", "Conn_01x02"),
                decl("U1", "MCU", "CustomMCU"),
                decl("U2", "Gate", "GATE"),
            ],
            nets: vec![
                NetDecl {
                    code: "1".to_string(),
                    name: "SIG".to_string(),
                    nodes: vec![
                        node("R1", "1", "1"),
                        node("J1", "1", "Pin_1"),
                        node("U2", "A", "A"),
                    ],
                },
                NetDecl {
                    code: "2".to_string(),
                    name: "GND".to_string(),
                    nodes: vec![
                        node("R1", "2", "2"),
                        node("J1", "2", "Pin_2"),
                        node("U1", "VSS", "VSS"),
                        node("U2", "Y", "Y"),
                    ],
                },
            ],
        }
    }

    struct Null;

    impl Component for Null {
        fn pins(&self) -> &[PinDecl] {
            &[]
        }
        fn attach(&mut self, _io: ComponentNetIo) -> Result<(), AttachError> {
            Ok(())
        }
    }

    /// A resistor the engine already understands, a connector whose pins a
    /// wire may use, and an IC that stops the build until it has a model.
    #[rstest]
    fn the_survey_separates_connectors_from_parts_that_need_a_model() {
        behaviour!(Test {
            id: "survey.needs-model-and-connectors",
            covers: Some("board/src/survey.rs#BoardSurvey::of"),
            given: "a netlist with a resistor, a two-pin header and an integrated circuit no \
                    model is registered for",
        });
        expect!(
            "ic-needs-model",
            "the integrated circuit, and only it, is listed as needing a model, named by its \
             reference and its value",
            "a part nothing classifies stops the board building, and the value is a key a \
             model can be assigned by"
        );
        expect!(
            "header-is-a-connector",
            "the header is listed as a connector with both of its pins",
            "a connector's pins are where a harness may wire the board to anything else"
        );
        expect!("not-compliant", "the board is reported not ready to build");
        let mut registry = PartRegistry::new();
        registry.register("GATE", |_| Box::new(Null));
        let survey = BoardSurvey::of(&netlist(), &registry);
        assert!(!survey.compliant());
        assert_eq!(survey.needs_model.len(), 1);
        assert_eq!(survey.needs_model[0].reference, "U1");
        assert_eq!(survey.needs_model[0].value, "MCU");
        let header = survey.connector("J1").expect("J1 is a connector");
        assert_eq!(header.pin_list(), "1, 2");
        assert!(survey.has_pin("J1", "1"));
        assert!(!survey.has_pin("J1", "9"));
        assert!(survey.has_part("R1"));
        assert!(survey.connector("R1").is_none());
    }

    /// A model registered with the pins it declares is checked against the
    /// netlist's pins before anything is built; one registered without is
    /// left to the build.
    #[rstest]
    fn a_stated_facade_that_names_other_pins_is_listed_with_both_pin_lists() {
        behaviour!(Test {
            id: "survey.facade-checked-before-build",
            covers: Some("board/src/survey.rs#BoardSurvey::of"),
            given: "a gate on a netlist that names its pins A and Y, its model registered \
                    stating which pins it has: first pins numbered 1 and 2, then pins A and Y",
        });
        expect!(
            "listed-with-both",
            "stating pins 1 and 2, the gate is listed with the model's pin list beside the \
             netlist's",
            "the build would refuse the part on the same comparison, and the two lists are \
             what a reader needs to pick the pin table that matches"
        );
        expect!(
            "matching-table-passes",
            "stating pins A and Y, nothing is listed for the gate and the board is ready to \
             build"
        );
        let mut registry = PartRegistry::new();
        registry.register("CustomMCU", |_| Box::new(Null));
        registry.register_model(
            "GATE",
            ModelFacade {
                model: "gate, pins = \"numbered\"".to_string(),
                pins: vec!["1".to_string(), "2".to_string()],
            },
            |_| Box::new(Null),
        );
        let survey = BoardSurvey::of(&netlist(), &registry);
        assert_eq!(
            survey.mismatched,
            vec![FacadeMismatch {
                reference: "U2".to_string(),
                value: "Gate".to_string(),
                mpn: None,
                model: "gate, pins = \"numbered\"".to_string(),
                declared: vec!["1".to_string(), "2".to_string()],
                netlist: vec!["A".to_string(), "Y".to_string()],
            }]
        );
        assert!(!survey.compliant());
        assert!(
            survey.to_string().contains(
                "U2  value \"Gate\": gate, pins = \"numbered\" declares 1, 2; the netlist has A, Y"
            ),
            "{survey}"
        );

        registry.register_model(
            "GATE",
            ModelFacade {
                model: "gate, pins = \"by-function\"".to_string(),
                pins: vec!["A".to_string(), "Y".to_string()],
            },
            |_| Box::new(Null),
        );
        let survey = BoardSurvey::of(&netlist(), &registry);
        assert!(survey.mismatched.is_empty(), "{survey}");
        assert!(survey.compliant(), "{survey}");
    }

    /// A switch whose poles name a pin the part does not have is listed the
    /// same way.
    #[rstest]
    fn switch_poles_are_checked_against_the_netlist() {
        let mut registry = PartRegistry::new();
        registry.register("CustomMCU", |_| Box::new(Null));
        registry.register_switch("GATE", vec![SwitchPole::open("A", "B")]);
        let survey = BoardSurvey::of(&netlist(), &registry);
        assert_eq!(survey.mismatched.len(), 1);
        assert_eq!(survey.mismatched[0].model, "a switch");
        assert_eq!(survey.mismatched[0].netlist, vec!["A", "Y"]);
    }

    #[rstest]
    #[case::numbers("2", "10", std::cmp::Ordering::Less)]
    #[case::references("J9", "J10", std::cmp::Ordering::Less)]
    #[case::letters("A", "B", std::cmp::Ordering::Less)]
    #[case::equal("U100", "U100", std::cmp::Ordering::Equal)]
    #[case::prefix("P", "P1", std::cmp::Ordering::Less)]
    fn identifiers_sort_the_way_a_reader_counts(
        #[case] a: &str,
        #[case] b: &str,
        #[case] expected: std::cmp::Ordering,
    ) {
        assert_eq!(natural_cmp(a, b), expected);
    }
}
