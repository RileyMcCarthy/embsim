//! Board construction: `Board::from_netlist(netlist, registry)` → components + nets.
//!
//! Building a board classifies every netlist component (see
//! [`crate::registry`]), instantiates registered [`Component`]s, validates
//! each pin facade against the netlist in both directions, resolves net
//! membership, and calls [`Component::attach`] pre-`Arc`.
//!
//! Slice status: the [`Board`] shape and error surface are final; the build
//! body is the board-build slice.

use std::collections::{HashMap, HashSet};
use std::fmt;

use crate::component::{AttachError, Component, PinDecl};
use crate::net::{Net, NetId, NetState, PinRef};
use crate::netlist::{normalize_pin_name, NetlistError, ParsedNetlist};
use crate::registry::{Classification, JumperState, PartRegistry, PassiveKind, RegistryError};

// ============================================================
// Board
// ============================================================

/// Electrical class resolved for one fitted netlist component (build-time
/// slice: what the resolution pass needs to know about it).
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum PartClass {
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
    /// Connector — harness attachment boundary.
    Boundary,
    /// Consumer-registered component; the pin facade snapshot drives
    /// electrical descriptors.
    Registered {
        /// Declared pins (validated against the netlist both directions).
        pins: Vec<PinDecl>,
    },
    /// Explicitly stubbed by the board's stub list — electrically absent.
    Stubbed,
}

/// One fitted-or-absent part the system build needs to reason about.
#[derive(Debug, Clone)]
pub(crate) struct PartRecord {
    /// Reference designator.
    pub(crate) reference: String,
    /// Electrical class.
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
    /// Build a board from a parsed netlist and the consumer's part registry.
    ///
    /// DNP components (`value == "X"` or the KiCad `dnp` property) are absent
    /// from the built board. A component with no classification and no
    /// registry entry fails construction — use [`Board::from_netlist_with_stubs`]
    /// to stub references explicitly.
    ///
    /// TODO(board-engine): board-build slice — classification, component
    /// instantiation, facade validation (both directions), net resolution,
    /// pre-`Arc` `attach()`.
    pub fn from_netlist(
        netlist: ParsedNetlist,
        registry: &PartRegistry,
    ) -> Result<Board, BoardError> {
        Self::from_netlist_with_stubs(netlist, registry, &[])
    }

    /// [`Board::from_netlist`] with a per-board explicit stub list — the only
    /// escape from the unclassified-component hard error.
    pub fn from_netlist_with_stubs(
        netlist: ParsedNetlist,
        registry: &PartRegistry,
        stub_refs: &[&str],
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

            let class = match registry.classify(decl, netlist_pins.len()) {
                Ok(Classification::Ignored) => continue,
                Ok(Classification::Boundary) => PartClass::Boundary,
                Ok(Classification::Jumper { default, .. }) => PartClass::Jumper { state: default },
                Ok(Classification::Passive { kind, value }) => PartClass::Passive { kind, value },
                Ok(Classification::Registered) => {
                    let mut component = registry
                        .construct(decl)
                        .expect("classify() returned Registered, so construct() must succeed");
                    let pins = component.pins().to_vec();
                    validate_facade(&decl.reference, &pins, &netlist_pins)?;
                    let pins_snapshot = pins.clone();
                    // attach() runs at system build, once final (merged) net
                    // ids exist — still pre-share.
                    let _ = &mut component;
                    components.push((decl.reference.clone(), component));
                    PartClass::Registered {
                        pins: pins_snapshot,
                    }
                }
                Ok(Classification::Stubbed) => PartClass::Stubbed,
                Err(RegistryError::UnknownPart { .. })
                    if stub_refs.contains(&decl.reference.as_str()) =>
                {
                    PartClass::Stubbed
                }
                Err(e) => return Err(BoardError::Classification(e)),
            };

            records.push(PartRecord {
                reference: decl.reference.clone(),
                class,
                fitted,
                pins: netlist_pins.iter().map(|p| p.to_string()).collect(),
            });
        }

        // Board-local nets. Net names get overline-normalized so consumers
        // and findings agree on one spelling (`~{RESET}` -> `~RESET`).
        let nets = netlist
            .nets
            .iter()
            .enumerate()
            .map(|(i, decl)| Net {
                id: NetId(i),
                name: normalize_pin_name(&decl.name),
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

    /// Reference designators of the instantiated (non-DNP, non-ignored)
    /// components, in netlist order.
    pub fn component_refs(&self) -> impl Iterator<Item = &str> {
        self.components
            .iter()
            .map(|(reference, _)| reference.as_str())
    }
}

/// Validate a registered component's declared pin facade against the netlist
/// in BOTH directions — declared-but-absent and present-but-undeclared pins
/// are hard build errors.
fn validate_facade(
    reference: &str,
    declared: &[PinDecl],
    netlist_pins: &[&str],
) -> Result<(), BoardError> {
    let declared_numbers: HashSet<&str> = declared.iter().map(|p| p.number).collect();
    let netlist_set: HashSet<&str> = netlist_pins.iter().copied().collect();

    for pin in &declared_numbers {
        if !netlist_set.contains(pin) {
            return Err(BoardError::PinFacadeMismatch {
                reference: reference.to_string(),
                pin: (*pin).to_string(),
            });
        }
    }
    for pin in &netlist_set {
        if !declared_numbers.contains(pin) {
            return Err(BoardError::PinFacadeMismatch {
                reference: reference.to_string(),
                pin: (*pin).to_string(),
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
    /// A registered component's declared pins do not match the netlist
    /// (either direction — declared-but-absent or present-but-undeclared).
    PinFacadeMismatch {
        /// Component reference designator.
        reference: String,
        /// The mismatched pin identity.
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
            BoardError::PinFacadeMismatch { .. } => None,
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
