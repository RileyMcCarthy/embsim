//! What a netlist still asks of a project.
//!
//! [`BoardSurvey::of`] classifies every part through the same registry the
//! board will build with (`DESIGN.md` rule 1). A part the
//! registry already understands is populated: a resistor, a connector, a
//! jumper, a part with a model. A part it does not is [`BoardSurvey::needs_model`],
//! named by reference, value, and manufacturer part number. A connector is
//! also listed with its pins, because those pins are the endpoints a harness
//! wire may join to another board or to a bench node.
//!
//! A survey with a non-empty [`BoardSurvey::needs_model`] or
//! [`BoardSurvey::refused`] is not a board yet. The project assigns a model
//! to each named part, or the build refuses with this list.

use std::collections::{BTreeMap, BTreeSet, HashMap};

use crate::netlist::{NetDecl, NodeDecl, ParsedNetlist};
use crate::registry::{Classification, PartRegistry, RegistryError};

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
    /// Pins the netlist attaches, in export order.
    pub pins: Vec<PinSite>,
}

/// A connector: a boundary the harness may wire, and the pins it offers.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ConnectorReport {
    /// Reference designator (`"J203"`).
    pub reference: String,
    /// Value field.
    pub value: String,
    /// Pins, in export order.
    pub pins: Vec<PinSite>,
}

/// The checklist for one netlist.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BoardSurvey {
    /// Components in the netlist, including ones with no pins.
    pub part_count: usize,
    /// Parts the registry classified, connectors included.
    pub modelled: usize,
    /// Parts with no class. The board will not build while this is non-empty.
    pub needs_model: Vec<UnmodelledPart>,
    /// A class the part cannot satisfy, such as a resistor with one pin.
    pub refused: Vec<String>,
    /// Connectors, in reference order.
    pub connectors: Vec<ConnectorReport>,
    /// `reference → pins` for every part that has a node.
    pins: BTreeMap<String, Vec<PinSite>>,
    /// Every reference designator.
    parts: BTreeSet<String>,
}

impl BoardSurvey {
    /// Classify `netlist` with `registry`.
    ///
    /// Pin counts match [`crate::Board::from_netlist`], so a part this survey
    /// accepts is a part that build will accept.
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
        let mut refused = Vec::new();
        let mut connectors = Vec::new();
        let mut pins = BTreeMap::new();
        let mut parts = BTreeSet::new();
        let mut modelled = 0usize;

        for decl in &netlist.components {
            parts.insert(decl.reference.clone());
            let nodes = pins_by_ref
                .get(decl.reference.as_str())
                .map(Vec::as_slice)
                .unwrap_or(&[]);
            let sites = pin_sites(nodes, &net_of);
            if !sites.is_empty() {
                pins.insert(decl.reference.clone(), sites.clone());
            }

            match registry.classify(decl, nodes.len()) {
                Ok(Classification::Boundary) => {
                    modelled += 1;
                    connectors.push(ConnectorReport {
                        reference: decl.reference.clone(),
                        value: decl.value.clone(),
                        pins: sites,
                    });
                }
                Ok(_) => modelled += 1,
                Err(RegistryError::UnknownPart {
                    reference,
                    part,
                    value,
                    mpn,
                }) => needs_model.push(UnmodelledPart {
                    reference,
                    part,
                    value,
                    mpn,
                    pins: sites,
                }),
                Err(err) => refused.push(err.to_string()),
            }
        }

        needs_model.sort_by(|a, b| a.reference.cmp(&b.reference));
        connectors.sort_by(|a, b| a.reference.cmp(&b.reference));
        refused.sort();
        Self {
            part_count: netlist.components.len(),
            modelled,
            needs_model,
            refused,
            connectors,
            pins,
            parts,
        }
    }

    /// Classify with only the reference-designator rules (`R` is a resistor,
    /// `J` is a connector). This is the list a new project starts from, before
    /// any model is assigned.
    pub fn bare(netlist: &ParsedNetlist) -> Self {
        let mut registry = PartRegistry::new();
        registry.classify_unnamed_by_reference(true);
        Self::of(netlist, &registry)
    }

    /// True when every part has a class the build can honour.
    pub fn compliant(&self) -> bool {
        self.needs_model.is_empty() && self.refused.is_empty()
    }

    /// Whether `reference` is a part in the netlist.
    pub fn has_part(&self, reference: &str) -> bool {
        self.parts.contains(reference)
    }

    /// Whether `reference` has pin `pin`.
    pub fn has_pin(&self, reference: &str, pin: &str) -> bool {
        self.pins
            .get(reference)
            .is_some_and(|pins| pins.iter().any(|site| site.pin == pin))
    }

    /// The connector `reference`, if the survey classified it as one.
    pub fn connector(&self, reference: &str) -> Option<&ConnectorReport> {
        self.connectors.iter().find(|conn| conn.reference == reference)
    }
}

impl std::fmt::Display for BoardSurvey {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(
            f,
            "{} parts, {} modelled, {} need a model, {} connectors",
            self.part_count,
            self.modelled,
            self.needs_model.len(),
            self.connectors.len()
        )?;
        for part in &self.needs_model {
            write!(
                f,
                "  {} needs a model  value={:?} part={:?}",
                part.reference, part.value, part.part
            )?;
            if let Some(mpn) = &part.mpn {
                write!(f, " mpn={mpn:?}")?;
            }
            writeln!(f, "  ({} pins)", part.pins.len())?;
        }
        for err in &self.refused {
            writeln!(f, "  refused: {err}")?;
        }
        for conn in &self.connectors {
            writeln!(
                f,
                "  connector {}  value={:?}  {} pins",
                conn.reference,
                conn.value,
                conn.pins.len()
            )?;
        }
        Ok(())
    }
}

/// Unique pins in export order. A pin named on one net is one pin.
fn pin_sites(nodes: &[&NodeDecl], net_of: &HashMap<(&str, &str), &NetDecl>) -> Vec<PinSite> {
    let mut seen = BTreeSet::new();
    let mut sites = Vec::new();
    for node in nodes {
        if !seen.insert(node.pin.clone()) {
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
    sites
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::netlist::{ComponentDecl, NetDecl, NodeDecl, ParsedNetlist};

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

    /// A resistor the engine already understands, a connector whose pins a
    /// wire may use, and an IC that stops the build until it has a model.
    #[test]
    fn the_survey_separates_connectors_from_parts_that_need_a_model() {
        let netlist = ParsedNetlist {
            version: "E".to_string(),
            components: vec![
                decl("R1", "10k", "R"),
                decl("J1", "Header", "Conn_01x02"),
                decl("U1", "MCU", "CustomMCU"),
            ],
            nets: vec![
                NetDecl {
                    code: "1".to_string(),
                    name: "SIG".to_string(),
                    nodes: vec![node("R1", "1", "1"), node("J1", "1", "Pin_1")],
                },
                NetDecl {
                    code: "2".to_string(),
                    name: "GND".to_string(),
                    nodes: vec![
                        node("R1", "2", "2"),
                        node("J1", "2", "Pin_2"),
                        node("U1", "VSS", "VSS"),
                    ],
                },
            ],
        };
        let survey = BoardSurvey::of(&netlist, &PartRegistry::new());
        assert!(!survey.compliant());
        assert_eq!(survey.needs_model.len(), 1);
        assert_eq!(survey.needs_model[0].reference, "U1");
        assert_eq!(survey.needs_model[0].value, "MCU");
        let header = survey.connector("J1").expect("J1 is a connector");
        assert_eq!(header.pins.len(), 2);
        assert!(survey.has_pin("J1", "1"));
        assert!(!survey.has_pin("J1", "9"));
        assert!(survey.has_part("R1"));
        assert!(survey.needs_model.iter().all(|part| part.reference != "R1"));
        assert!(survey.connector("R1").is_none());
    }
}
