//! Board construction: `Board::from_netlist(netlist, registry)` → nodes + nets.
//!
//! Building a board classifies every netlist component (see
//! [`crate::registry`]) into a node class, instantiates registered
//! [`Component`]s, validates each declared pin facade — a component's pins,
//! a switch's poles, a piecewise-linear element's pins — against the netlist
//! in both directions, and resolves net membership. **Every netlist part
//! gets a record**: there is no stub list and no ignored tier (`DESIGN.md`
//! rule 1), so a part the registry cannot classify is a build error naming
//! the reference, the part and the value.

use std::collections::{HashMap, HashSet};
use std::fmt;

use crate::component::{AttachError, Branch, Component, IdleDrive, PinDecl, PinKind};
use crate::net::{Net, NetId, NetState, PinRef};
use crate::netlist::{normalize_net_name, NetlistError, ParsedNetlist};
use crate::registry::{
    Classification, JumperState, PartRegistry, PassiveKind, PwlSpec, RegistryError, SwitchPole,
};

// ============================================================
// Board
// ============================================================

/// The node class resolved for one netlist component — what the system build
/// needs to know about it, and what [`Board::nodes`] reports. The classes are
/// exactly the taxonomy rows of `NODES.md` §2.
#[derive(Debug, Clone, PartialEq)]
pub enum PartClass {
    /// Two-terminal passive. Only resistors (known value), inductors
    /// (DC short), and closed jumpers conduct in the build-time DC pass;
    /// capacitors, diodes, and LEDs are DC-open (documented simplification —
    /// full behavior is the cluster-solver slice).
    Passive {
        /// Primitive kind.
        kind: PassiveKind,
        /// Parsed value in base SI units, when numeric.
        value: Option<f64>,
    },
    /// Stateful short (scenario-overridable).
    Jumper {
        /// Current state (default from the symbol name).
        state: JumperState,
    },
    /// A switch: poles by pin id, each open or closed. A closed pole is a
    /// build-time identity union of its two nets; an open pole is nothing.
    Switch {
        /// The poles, in declaration order (the index `Scenario::switch`
        /// addresses).
        poles: Vec<SwitchPole>,
    },
    /// A piecewise-linear element registered by specification
    /// ([`PwlSpec`]): its pins, validated against the netlist both ways,
    /// and the branches between them, which the system build stamps into
    /// the cluster solve.
    Pwl {
        /// The element's specification.
        spec: PwlSpec,
    },
    /// Connector — harness attachment boundary.
    Boundary,
    /// A test point: a one-pin probe node.
    Probe,
    /// A mechanical part: pads recorded, nothing electrical.
    Mechanical,
    /// Consumer-registered component; the pin facade snapshot drives
    /// electrical descriptors.
    Registered {
        /// Declared pins (validated against the netlist both directions).
        pins: Vec<PinDecl>,
        /// Declared nonlinear branches, each naming declared pins only.
        branches: Vec<Branch>,
    },
}

/// One fitted-or-absent part the system build needs to reason about.
#[derive(Debug, Clone)]
pub(crate) struct PartRecord {
    /// Reference designator.
    pub(crate) reference: String,
    /// Node class.
    pub(crate) class: PartClass,
    /// False when DNP (`value == "X"` or the netlist `dnp` property);
    /// scenario `dnp_override` can flip this at system build.
    pub(crate) fitted: bool,
    /// Netlist pins of this component, in netlist order.
    pub(crate) pins: Vec<String>,
}

/// A built board: instantiated components + resolved nets, ready to be added
/// to a [`crate::system::System`].
pub struct Board {
    pub(crate) components: Vec<(String, Box<dyn Component>)>,
    pub(crate) records: Vec<PartRecord>,
    pub(crate) nets: Vec<Net>,
}

impl fmt::Debug for Board {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Board")
            .field(
                "components",
                &self.components.iter().map(|(r, _)| r).collect::<Vec<_>>(),
            )
            .field("nets", &self.nets)
            .finish()
    }
}

impl Board {
    /// Build a board from a parsed netlist and the consumer's part registry —
    /// the one constructor.
    ///
    /// DNP components (`value == "X"` or the KiCad `dnp` property) are absent
    /// from the built board but keep their record. A component with no
    /// classification and no registry entry fails construction with
    /// [`RegistryError::UnknownPart`], naming the reference, the part and the
    /// value; a declared facade that disagrees with the netlist fails with
    /// [`BoardError::PinFacadeMismatch`].
    pub fn from_netlist(
        netlist: ParsedNetlist,
        registry: &PartRegistry,
    ) -> Result<Board, BoardError> {
        // Netlist pins per reference (for pin counts and facade validation).
        let mut pins_by_ref: HashMap<&str, Vec<&str>> = HashMap::new();
        for net in &netlist.nets {
            for node in &net.nodes {
                pins_by_ref
                    .entry(node.reference.as_str())
                    .or_default()
                    .push(node.pin.as_str());
            }
        }

        let mut components: Vec<(String, Box<dyn Component>)> = Vec::new();
        let mut records = Vec::new();

        for decl in &netlist.components {
            let netlist_pins = pins_by_ref
                .get(decl.reference.as_str())
                .cloned()
                .unwrap_or_default();
            let fitted = !(decl.dnp || decl.value == "X");

            let class = match registry.classify(decl, netlist_pins.len())? {
                Classification::Boundary => PartClass::Boundary,
                Classification::Probe => PartClass::Probe,
                Classification::Mechanical => PartClass::Mechanical,
                Classification::Jumper { default } => PartClass::Jumper { state: default },
                Classification::Passive { kind, value } => PartClass::Passive { kind, value },
                Classification::Switch { poles } => {
                    let declared: Vec<&str> = poles
                        .iter()
                        .flat_map(|pole| [pole.a.as_str(), pole.b.as_str()])
                        .collect();
                    validate_pins(&decl.reference, &declared, &netlist_pins)?;
                    PartClass::Switch { poles }
                }
                Classification::Pwl { spec } => {
                    let declared: Vec<&str> = spec.pins.iter().map(String::as_str).collect();
                    validate_pins(&decl.reference, &declared, &netlist_pins)?;
                    let branch_pins = spec.branches.iter().flat_map(|branch| {
                        [branch.a.as_str(), branch.b.as_str()]
                            .into_iter()
                            .chain(branch.control.as_ref().map(|(pin, _)| pin.as_str()))
                    });
                    validate_branch_pins(&decl.reference, branch_pins, |pin| {
                        declared.contains(&pin)
                    })?;
                    PartClass::Pwl { spec }
                }
                Classification::Registered => {
                    let component = registry
                        .construct(decl)
                        .expect("classify() returned Registered, so construct() must succeed");
                    let pins = component.pins().to_vec();
                    validate_facade(&decl.reference, &pins, &netlist_pins)?;
                    let branches = component.branches().to_vec();
                    let branch_pins = branches.iter().flat_map(|branch| {
                        [branch.a, branch.b]
                            .into_iter()
                            .chain(branch.control.map(|(pin, _)| pin))
                    });
                    validate_branch_pins(&decl.reference, branch_pins, |pin| {
                        pins.iter().any(|p| p.number == pin || p.name == Some(pin))
                    })?;
                    // attach() runs at system build, once final (merged) net
                    // ids exist — still pre-share.
                    components.push((decl.reference.clone(), component));
                    PartClass::Registered { pins, branches }
                }
            };

            records.push(PartRecord {
                reference: decl.reference.clone(),
                class,
                fitted,
                pins: netlist_pins.iter().map(|p| p.to_string()).collect(),
            });
        }

        // Board-local nets, one per netlist net in export order. Names get
        // overline-normalized so consumers and findings agree on one spelling
        // (`~{RESET}` -> `~RESET`); the sheet path of a hierarchical local
        // label is kept verbatim, so `/Sheet2/SIGNAL` and `/Sheet3/SIGNAL`
        // stay two distinguishable nets (see `normalize_net_name`).
        let nets = netlist
            .nets
            .iter()
            .enumerate()
            .map(|(i, decl)| Net {
                id: NetId(i),
                name: normalize_net_name(&decl.name),
                nodes: decl
                    .nodes
                    .iter()
                    .map(|n| PinRef::new(n.reference.clone(), n.pin.clone()))
                    .collect(),
                state: NetState::Floating,
            })
            .collect();

        Ok(Board {
            components,
            records,
            nets,
        })
    }

    /// Resolved nets, indexed by [`crate::net::NetId`].
    pub fn nets(&self) -> &[Net] {
        &self.nets
    }

    /// Reference designators of the instantiated (non-DNP) components — the
    /// [`PartClass::Registered`] nodes — in netlist order.
    pub fn component_refs(&self) -> impl Iterator<Item = &str> {
        self.components
            .iter()
            .map(|(reference, _)| reference.as_str())
    }

    /// Every netlist part with the node class it was given, in netlist
    /// order — the census of the one pipeline (`DESIGN.md` rule 1): a part is
    /// a node whose class has behaviour, or the board did not build.
    ///
    /// DNP parts are listed too (their class is what the registry said of
    /// the symbol); they are absent from the built system unless a scenario
    /// populates them.
    pub fn nodes(&self) -> impl Iterator<Item = (&str, &PartClass)> {
        self.records
            .iter()
            .map(|record| (record.reference.as_str(), &record.class))
    }

    /// The node class of one reference, or `None` when the netlist has no
    /// such part.
    pub fn node_class(&self, reference: &str) -> Option<&PartClass> {
        self.records
            .iter()
            .find(|record| record.reference == reference)
            .map(|record| &record.class)
    }
}

/// Validate a registered component's declared pin facade against the netlist
/// in BOTH directions — declared-but-absent and present-but-undeclared pins
/// are hard build errors — and every declaration the engine would otherwise
/// have to drop ([`validate_idle_drives`]).
fn validate_facade(
    reference: &str,
    declared: &[PinDecl],
    netlist_pins: &[&str],
) -> Result<(), BoardError> {
    validate_idle_drives(reference, declared)?;
    let declared: Vec<&str> = declared.iter().map(|p| p.number).collect();
    validate_pins(reference, &declared, netlist_pins)
}

/// A declared idle drive is honoured on every pin with a drive slot, and a
/// power or passive pin has none ([`IdleDrive`]) — so an idle drive declared
/// on one is a static fact the engine cannot keep, refused here rather than
/// dropped without a word. Shared by the netlist facade check and the bench
/// component path in `System::assemble`, which has no netlist facade to
/// validate but the same declarations to honour.
pub(crate) fn validate_idle_drives(
    reference: &str,
    declared: &[PinDecl],
) -> Result<(), BoardError> {
    for pin in declared {
        let slotless = matches!(
            pin.kind,
            PinKind::PowerIn | PinKind::PowerOut | PinKind::Passive
        );
        if slotless && !matches!(pin.idle, IdleDrive::KindDefault) {
            return Err(BoardError::IdleOnSlotlessPin {
                reference: reference.to_string(),
                pin: pin.number.to_string(),
            });
        }
    }
    Ok(())
}

/// Validate a set of declared pin ids (a switch's pole pins, a
/// piecewise-linear element's pins) against the netlist in BOTH directions,
/// like [`validate_facade`]. The two scans walk the declaration and the
/// netlist in their own order, so the pin an error names is the first
/// mismatch as written, the same one on every run.
fn validate_pins(
    reference: &str,
    declared: &[&str],
    netlist_pins: &[&str],
) -> Result<(), BoardError> {
    let declared_set: HashSet<&str> = declared.iter().copied().collect();
    let netlist_set: HashSet<&str> = netlist_pins.iter().copied().collect();

    for pin in declared {
        if !netlist_set.contains(pin) {
            return Err(BoardError::PinFacadeMismatch {
                reference: reference.to_string(),
                pin: (*pin).to_string(),
            });
        }
    }
    for pin in netlist_pins {
        if !declared_set.contains(pin) {
            return Err(BoardError::PinFacadeMismatch {
                reference: reference.to_string(),
                pin: (*pin).to_string(),
            });
        }
    }
    Ok(())
}

/// Every pin a declared branch names — its two terminals and its control —
/// must be a pin the part declares (`is_declared`), or the branch would
/// stamp onto nothing: the first that is not is a facade mismatch naming
/// the pin, in declaration order.
fn validate_branch_pins<'a>(
    reference: &str,
    branch_pins: impl IntoIterator<Item = &'a str>,
    is_declared: impl Fn(&str) -> bool,
) -> Result<(), BoardError> {
    for pin in branch_pins {
        if !is_declared(pin) {
            return Err(BoardError::PinFacadeMismatch {
                reference: reference.to_string(),
                pin: pin.to_string(),
            });
        }
    }
    Ok(())
}

// ============================================================
// Errors
// ============================================================

/// Board construction failure. Electrical findings go to
/// [`crate::diagnostics::Diagnostics`]; these are the structural hard errors.
#[derive(Debug)]
pub enum BoardError {
    /// The netlist failed to parse.
    Netlist(NetlistError),
    /// A component failed classification (unknown part, pin-count violation).
    Classification(RegistryError),
    /// A declared pin facade — a registered component's pins, a switch's
    /// pole pins, a piecewise-linear element's pins — does not match the
    /// netlist (either direction — declared-but-absent or
    /// present-but-undeclared), or a declared branch names a pin the facade
    /// does not declare.
    PinFacadeMismatch {
        /// Component reference designator.
        reference: String,
        /// The mismatched pin identity.
        pin: String,
    },
    /// A declared idle drive on a pin that has no drive slot — a power or
    /// passive pin ([`IdleDrive`]). The engine could only drop the
    /// declaration, so the build refuses it and names the pin.
    IdleOnSlotlessPin {
        /// Component reference designator.
        reference: String,
        /// The pin carrying the declaration.
        pin: String,
    },
    /// A component's `attach()` failed.
    Attach {
        /// Component reference designator.
        reference: String,
        /// The underlying attach failure.
        error: AttachError,
    },
}

impl fmt::Display for BoardError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            BoardError::Netlist(e) => write!(f, "netlist: {e}"),
            BoardError::Classification(e) => write!(f, "classification: {e}"),
            BoardError::PinFacadeMismatch { reference, pin } => {
                write!(f, "{reference}: pin facade mismatch on pin {pin:?}")
            }
            BoardError::IdleOnSlotlessPin { reference, pin } => write!(
                f,
                "{reference}: pin {pin:?} declares an idle drive but has no drive slot \
                 (power and passive pins idle at nothing)"
            ),
            BoardError::Attach { reference, error } => write!(f, "{reference}: {error}"),
        }
    }
}

impl std::error::Error for BoardError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            BoardError::Netlist(e) => Some(e),
            BoardError::Classification(e) => Some(e),
            BoardError::Attach { error, .. } => Some(error),
            BoardError::PinFacadeMismatch { .. } | BoardError::IdleOnSlotlessPin { .. } => None,
        }
    }
}

impl From<NetlistError> for BoardError {
    fn from(e: NetlistError) -> Self {
        BoardError::Netlist(e)
    }
}

impl From<RegistryError> for BoardError {
    fn from(e: RegistryError) -> Self {
        BoardError::Classification(e)
    }
}

#[cfg(test)]
mod tests {
    use rstest::rstest;

    use super::*;
    use crate::netlist::parse;
    use crate::registry::PwlSpec;

    /// A hand-written netlist with one of each declared class: a switch
    /// `S1` whose pins pair off `1_ON`/`1_OFF` and `2_ON`/`2_OFF`, a diode
    /// `D1` on pins `A`/`K`, a test point `TP1`, a mounting hole `H1`, and a
    /// part `U9` nothing classifies.
    const NETLIST: &str = r#"(export (version "E")
  (components
    (comp (ref "S1") (value "DIP2"))
    (comp (ref "D1") (value "SS36") (libsource (lib "Diode") (part "SS36") (description "")))
    (comp (ref "TP1") (value "TP") (libsource (lib "Connector") (part "TestPoint") (description "")))
    (comp (ref "H1") (value "MountingHole_Pad") (libsource (lib "Mechanical") (part "MountingHole_Pad") (description "")))
    (comp (ref "U9") (value "Frobulator 9000")))
  (nets
    (net (code "1") (name "A") (class "Default")
      (node (ref "S1") (pin "1_ON") (pintype "passive"))
      (node (ref "D1") (pin "A") (pintype "passive"))
      (node (ref "TP1") (pin "1") (pintype "passive")))
    (net (code "2") (name "B") (class "Default")
      (node (ref "S1") (pin "1_OFF") (pintype "passive"))
      (node (ref "D1") (pin "K") (pintype "passive"))
      (node (ref "H1") (pin "1") (pintype "passive")))
    (net (code "3") (name "C") (class "Default")
      (node (ref "S1") (pin "2_ON") (pintype "passive"))
      (node (ref "U9") (pin "1") (pintype "passive")))
    (net (code "4") (name "D") (class "Default")
      (node (ref "S1") (pin "2_OFF") (pintype "passive"))
      (node (ref "U9") (pin "2") (pintype "passive")))))"#;

    fn registry() -> PartRegistry {
        let mut registry = PartRegistry::new();
        registry.register_switch(
            "DIP2",
            vec![
                SwitchPole::open("1_ON", "1_OFF"),
                SwitchPole::closed("2_ON", "2_OFF"),
            ],
        );
        registry.register_pwl("SS36", diode_spec());
        registry.register_mechanical("Frobulator 9000");
        registry
    }

    /// The fixture's diode: anode `A`, cathode `K`, a 0.75 V knee.
    fn diode_spec() -> PwlSpec {
        PwlSpec::diode("A", "K", 0.75, 0.0)
    }

    /// Every netlist part is a node of the class the registry gave it.
    #[rstest]
    fn every_netlist_part_is_a_node_with_its_class() {
        let board = Board::from_netlist(parse(NETLIST).unwrap(), &registry()).unwrap();
        let nodes: Vec<(&str, &PartClass)> = board.nodes().collect();
        assert_eq!(nodes.len(), 5, "one record per netlist part: {nodes:?}");
        assert_eq!(
            board.node_class("S1"),
            Some(&PartClass::Switch {
                poles: vec![
                    SwitchPole::open("1_ON", "1_OFF"),
                    SwitchPole::closed("2_ON", "2_OFF"),
                ]
            })
        );
        assert_eq!(
            board.node_class("D1"),
            Some(&PartClass::Pwl { spec: diode_spec() })
        );
        assert_eq!(board.node_class("TP1"), Some(&PartClass::Probe));
        assert_eq!(board.node_class("H1"), Some(&PartClass::Mechanical));
        assert_eq!(board.node_class("U9"), Some(&PartClass::Mechanical));
        assert_eq!(board.node_class("U99"), None);
        // None of them is a component.
        assert_eq!(board.component_refs().count(), 0);
    }

    /// A part the registry cannot classify is a build error naming the
    /// reference, the part and the value.
    #[rstest]
    fn an_unclassifiable_part_fails_the_build_naming_reference_part_and_value() {
        // The registry above without the entry that covers U9.
        let mut registry = PartRegistry::new();
        registry.register_switch(
            "DIP2",
            vec![
                SwitchPole::open("1_ON", "1_OFF"),
                SwitchPole::closed("2_ON", "2_OFF"),
            ],
        );
        registry.register_pwl("SS36", diode_spec());
        let error =
            Board::from_netlist(parse(NETLIST).unwrap(), &registry).expect_err("U9 has no class");
        match &error {
            BoardError::Classification(RegistryError::UnknownPart {
                reference,
                part,
                value,
                mpn,
            }) => {
                assert_eq!(reference, "U9");
                assert_eq!(part, "");
                assert_eq!(value, "Frobulator 9000");
                assert_eq!(mpn, &None);
            }
            other => panic!("expected UnknownPart, got {other:?}"),
        }
        let rendered = error.to_string();
        assert!(
            rendered.contains("U9") && rendered.contains("Frobulator 9000"),
            "{rendered}"
        );
    }

    /// A switch's poles must name exactly the part's netlist pins, in both
    /// directions, like a component's facade; the pin the error names is
    /// the first mismatch in declaration order, then in netlist order.
    #[rstest]
    #[case::pole_names_a_pin_the_part_lacks(vec![SwitchPole::open("1_ON", "1_OFF"), SwitchPole::open("2_ON", "9_OFF")], "9_OFF")]
    #[case::a_pin_no_pole_names(vec![SwitchPole::open("1_ON", "1_OFF")], "2_ON")]
    fn switch_poles_are_validated_against_the_netlist_both_ways(
        #[case] poles: Vec<SwitchPole>,
        #[case] named: &str,
    ) {
        let mut registry = registry();
        registry.register_switch("DIP2", poles);
        let error = Board::from_netlist(parse(NETLIST).unwrap(), &registry)
            .expect_err("the poles disagree with the netlist");
        match &error {
            BoardError::PinFacadeMismatch { reference, pin } => {
                assert_eq!(reference, "S1");
                assert_eq!(pin, named, "the first mismatch as written is the one named");
            }
            other => panic!("expected PinFacadeMismatch, got {other:?}"),
        }
    }

    /// A piecewise-linear element's pins are validated the same way.
    #[rstest]
    fn pwl_pins_are_validated_against_the_netlist_both_ways() {
        let mut registry = registry();
        registry.register_pwl("SS36", PwlSpec::new(["A"]));
        let error = Board::from_netlist(parse(NETLIST).unwrap(), &registry)
            .expect_err("K is a pin no declaration names");
        assert!(
            matches!(&error, BoardError::PinFacadeMismatch { reference, pin } if reference == "D1" && pin == "K"),
            "{error:?}"
        );
    }

    /// A branch names the pins it runs between, and a control pin; each
    /// must be a pin the element declares, or the build refuses it naming
    /// the pin.
    #[rstest]
    fn a_branch_naming_an_undeclared_pin_is_a_facade_mismatch() {
        let mut registry = registry();
        registry.register_pwl(
            "SS36",
            PwlSpec::new(["A", "K"]).with_branch(
                "A",
                "G",
                crate::PwlCurve::Diode { vf: 0.75, r_d: 0.0 },
            ),
        );
        let error = Board::from_netlist(parse(NETLIST).unwrap(), &registry)
            .expect_err("G is a pin the element does not declare");
        assert!(
            matches!(&error, BoardError::PinFacadeMismatch { reference, pin } if reference == "D1" && pin == "G"),
            "{error:?}"
        );
    }
}
