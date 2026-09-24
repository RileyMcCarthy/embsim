//! PartRegistry: component identity → class; auto-classification tiers.
//!
//! There is **one pipeline**: netlist part → registry class → node
//! (`DESIGN.md` rule 1). A part is a node whose class has behaviour, or the
//! board refuses to build naming the part and its value. Classification is
//! three-tier and keyed primarily on the libsource **part** name (the lib
//! name is best-effort only; rescue mangling like
//! `DS2_Addon-rescue::Jumper_NO_Small-Device` is normalized before matching):
//!
//! 1. **auto** — passive primitives (`R*`/`C*`/`L*`/`LED`/`D_*`, pin-count
//!    validated), connectors/screw terminals (board boundary pins), jumpers
//!    (stateful shorts), two-pin `SW_*` switches (one open pole), test points
//!    (probe nodes) and mechanical parts (mounting holes, logos, fiducials —
//!    nodes with pads and nothing electrical). A board-specific connector
//!    symbol whose part name matches no prefix joins this tier through
//!    [`PartRegistry::register_boundary`].
//! 2. **registry** — anything else, keyed by part name, falling back to
//!    `value`: a consumer-registered [`Component`] constructor
//!    ([`PartRegistry::register`]), a switch with declared poles
//!    ([`PartRegistry::register_switch`]), a piecewise-linear element
//!    ([`PartRegistry::register_pwl`]) or a mechanical part
//!    ([`PartRegistry::register_mechanical`]).
//! 3. **error** — no registry match: [`RegistryError::UnknownPart`], naming
//!    the reference, the part and the value. System construction fails.
//!    There is no stub tier and no allow-list.
//!
//! # Netlists with no `libsource`
//!
//! Every tier keys on the libsource **part** name, which an EDA export always
//! carries — but a netlist transcribed from a vendor PDF has no symbol library
//! to name, so every `part` is empty and the whole board lands in tier 3.
//! [`PartRegistry::classify_unnamed_by_reference`] opts such a board into a
//! narrow fallback: when (and only when) a component's part name is empty, the
//! auto tier matches against a class name derived from its **reference
//! designator prefix** ([`reference_designator_class`]). See that function for
//! the prefix table and why the fallback is opt-in.

use std::collections::{HashMap, HashSet};
use std::fmt;

use crate::component::{Component, PwlCurve, RegionTest};
use crate::net::{Ohms, Volts};
use crate::netlist::ComponentDecl;

// ============================================================
// Classification
// ============================================================

/// Class of an auto-classified passive primitive.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PassiveKind {
    /// `R*` parts.
    Resistor,
    /// `C*` parts.
    Capacitor,
    /// `L*` parts.
    Inductor,
    /// `D_*` parts.
    Diode,
    /// `LED` parts.
    Led,
}

/// State of a jumper's stateful short, and of one switch pole
/// (scenario-overridable).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum JumperState {
    /// Terminals disconnected (`_NO`/`_Open` default).
    Open,
    /// Terminals shorted (`_NC`/`_Bridged` default).
    Closed,
}

/// One pole of a switch: two pin ids of the part, and the pole's state.
///
/// A closed pole is a **build-time identity union** of the two pins' nets
/// (the same merge a `pin_short` makes, honouring detached pins); an open
/// pole contributes nothing. Poles are addressed by their index in the
/// declaration order, from `Scenario::switch(reference, pole, state)`.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct SwitchPole {
    /// One side of the pole, by netlist pin id.
    pub a: String,
    /// The other side, by netlist pin id.
    pub b: String,
    /// The pole's state (the default at build; a scenario can change it).
    pub state: JumperState,
}

impl SwitchPole {
    /// A pole between pins `a` and `b`, open by default.
    pub fn open(a: impl Into<String>, b: impl Into<String>) -> Self {
        Self {
            a: a.into(),
            b: b.into(),
            state: JumperState::Open,
        }
    }

    /// A pole between pins `a` and `b`, closed by default.
    pub fn closed(a: impl Into<String>, b: impl Into<String>) -> Self {
        Self {
            a: a.into(),
            b: b.into(),
            state: JumperState::Closed,
        }
    }
}

/// One branch of a [`PwlSpec`]: a [`crate::Branch`] with its pins as the
/// netlist names them (owned, since a registry entry is built at run time
/// from a library), between two of the spec's pins with an optional control
/// pin. The current is reported positive from `a` to `b`.
#[derive(Debug, Clone, PartialEq)]
pub struct PwlBranch {
    /// The anode / drain pin.
    pub a: String,
    /// The cathode / source pin — the reference of the control test.
    pub b: String,
    /// The two-region curve.
    pub curve: PwlCurve,
    /// The control pin and its test on `V(control) − V(b)`, for a channel.
    pub control: Option<(String, RegionTest)>,
}

/// The specification of a piecewise-linear element registered **by spec**
/// — a netlist part with no Rust model behind it: a diode, an LED, a FET or
/// BJT channel. It carries the pins the part declares (validated against the
/// netlist both ways, like a component's facade) and the branches between
/// them; the engine stamps the branches and chooses their regions exactly as
/// it does a [`crate::Component::branches`] declaration. Every number in a
/// spec cites a datasheet — `embsim_models::pwl_library` is the library of
/// such entries, keyed on the netlist's manufacturer part number.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct PwlSpec {
    /// The pins the element declares, by netlist pin id, in declaration
    /// order.
    pub pins: Vec<String>,
    /// The branches, in declaration order — the order the flip loop
    /// evaluates their region tests in.
    pub branches: Vec<PwlBranch>,
}

impl PwlSpec {
    /// A spec declaring `pins` and no branch yet (see [`PwlSpec::with_branch`]).
    pub fn new(pins: impl IntoIterator<Item = impl Into<String>>) -> Self {
        Self {
            pins: pins.into_iter().map(Into::into).collect(),
            branches: Vec::new(),
        }
    }

    /// The spec with one more branch from `a` to `b`, no control.
    pub fn with_branch(
        mut self,
        a: impl Into<String>,
        b: impl Into<String>,
        curve: PwlCurve,
    ) -> Self {
        self.branches.push(PwlBranch {
            a: a.into(),
            b: b.into(),
            curve,
            control: None,
        });
        self
    }

    /// The spec with one more branch from `a` to `b`, switched by
    /// `control` through `test` (a [`PwlCurve::Channel`]).
    pub fn with_controlled_branch(
        mut self,
        a: impl Into<String>,
        b: impl Into<String>,
        curve: PwlCurve,
        control: impl Into<String>,
        test: RegionTest,
    ) -> Self {
        self.branches.push(PwlBranch {
            a: a.into(),
            b: b.into(),
            curve,
            control: Some((control.into(), test)),
        });
        self
    }

    /// A two-pin diode: one [`PwlCurve::Diode`] branch from `anode` to
    /// `cathode`.
    pub fn diode(
        anode: impl Into<String>,
        cathode: impl Into<String>,
        vf: Volts,
        r_d: Ohms,
    ) -> Self {
        let (anode, cathode) = (anode.into(), cathode.into());
        Self::new([anode.clone(), cathode.clone()]).with_branch(
            anode,
            cathode,
            PwlCurve::Diode { vf, r_d },
        )
    }
}

/// Result of classifying one netlist component. Every variant is a node
/// class with behaviour (or, for the types phase 1 declares ahead of their
/// behaviour, a class whose behaviour is scheduled by `NODES.md`); there is
/// no "absent" or "stubbed" outcome.
#[derive(Debug, Clone, PartialEq)]
pub enum Classification {
    /// Auto tier: a passive primitive with its parsed base-SI value
    /// (Ω / F / H), when the value field parses.
    Passive {
        /// Primitive class.
        kind: PassiveKind,
        /// Parsed value in base SI units (`"4k7"` → `4700.0`); `None` when
        /// the value field is not numeric.
        value: Option<f64>,
    },
    /// Auto tier: a connector / screw terminal — board boundary pins that
    /// harnesses attach to.
    Boundary,
    /// Auto tier: a two-pad jumper (a stateful short). A three-pad
    /// `Jumper_3_*` / `SolderJumper_3_*` symbol is a two-pole
    /// [`Classification::Switch`] instead.
    Jumper {
        /// Default state from the part name.
        default: JumperState,
    },
    /// A switch: poles by pin id, each open or closed at build. Auto tier for
    /// a two-pin `SW_*` symbol (one open pole across pins `1` and `2`, the
    /// KiCad `Switch` library's two-terminal pinout); registry tier for
    /// anything declared through [`PartRegistry::register_switch`].
    Switch {
        /// The poles, in declaration order.
        poles: Vec<SwitchPole>,
    },
    /// Registry tier: a piecewise-linear element declared through
    /// [`PartRegistry::register_pwl`]. Its pins are validated against the
    /// netlist in both directions, like a registered component's facade.
    Pwl {
        /// The element's specification: its pins and branches.
        spec: PwlSpec,
    },
    /// Auto tier: a test point — a one-pin probe node that senses and never
    /// drives. A `TestPoint*` symbol on no net is [`Classification::Mechanical`]
    /// (a pad); one with more pins is whatever the consumer registers it as,
    /// and a pin-count error when nothing is registered.
    Probe,
    /// A mechanical part — a mounting hole, a logo, a fiducial, a board
    /// outline, a layout node: pads recorded, nothing electrical. Auto tier
    /// by part-name prefix, or declared through
    /// [`PartRegistry::register_mechanical`].
    Mechanical,
    /// Registry tier: a consumer-registered [`Component`] constructor exists
    /// for this part.
    Registered,
}

// ============================================================
// Registry
// ============================================================

/// Constructor for a consumer-registered component.
pub type ComponentCtor = Box<dyn Fn(&ComponentDecl) -> Box<dyn Component> + Send + Sync>;

/// One registry entry: what a part keyed by name (or value) is.
enum RegistryEntry {
    /// A model: a component constructor.
    Component(ComponentCtor),
    /// A switch with declared poles.
    Switch(Vec<SwitchPole>),
    /// A piecewise-linear element.
    Pwl(PwlSpec),
    /// A mechanical part.
    Mechanical,
}

impl RegistryEntry {
    /// The classification this entry produces.
    fn classification(&self) -> Classification {
        match self {
            RegistryEntry::Component(_) => Classification::Registered,
            RegistryEntry::Switch(poles) => Classification::Switch {
                poles: poles.clone(),
            },
            RegistryEntry::Pwl(spec) => Classification::Pwl { spec: spec.clone() },
            RegistryEntry::Mechanical => Classification::Mechanical,
        }
    }

    /// A one-word label for `Debug`.
    fn kind(&self) -> &'static str {
        match self {
            RegistryEntry::Component(_) => "component",
            RegistryEntry::Switch(_) => "switch",
            RegistryEntry::Pwl(_) => "pwl",
            RegistryEntry::Mechanical => "mechanical",
        }
    }
}

/// Consumer part registry: maps part identity to a class — a component
/// constructor, a switch's poles, a piecewise-linear element, a mechanical
/// part. Lookup keys on the rescue-normalized part name, then on the
/// netlist's manufacturer part number ([`ComponentDecl::mpn`]), then on the
/// component `value`; one table, so a later registration of the same key
/// replaces the earlier one whatever its kind.
#[derive(Default)]
pub struct PartRegistry {
    entries: HashMap<String, RegistryEntry>,
    boundaries: HashSet<String>,
    reference_fallback: bool,
}

impl fmt::Debug for PartRegistry {
    /// hash-order: these two are the only unordered iterations left in the
    /// crate, and they are **not** on an engine path — `Debug` output for a
    /// human, reached by no resolution, routing, or classification decision
    /// (lookups all go through `get`). Sorting them would only be cosmetic, so
    /// the exemption is recorded here rather than paid for. See the review rule
    /// in `crate::engine`'s module docs.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("PartRegistry")
            .field(
                "entries",
                &self
                    .entries
                    .iter()
                    .map(|(key, entry)| (key, entry.kind()))
                    .collect::<Vec<_>>(),
            )
            .field("boundaries", &self.boundaries.iter().collect::<Vec<_>>())
            .field("reference_fallback", &self.reference_fallback)
            .finish()
    }
}

impl PartRegistry {
    /// Empty registry.
    pub fn new() -> Self {
        Self::default()
    }

    /// Register a component constructor for a part name (or, on a netlist
    /// with no libsource, a value).
    pub fn register(
        &mut self,
        part: impl Into<String>,
        ctor: impl Fn(&ComponentDecl) -> Box<dyn Component> + Send + Sync + 'static,
    ) {
        self.entries
            .insert(part.into(), RegistryEntry::Component(Box::new(ctor)));
    }

    /// Declare a part name to be a **switch** with the given poles, each a
    /// pair of the part's netlist pin ids with its default state.
    ///
    /// A DIP switch, a slide switch, a solder link: parts whose pins pair
    /// off into contacts the netlist does not describe. The pairing is a
    /// registry declaration; the board build validates every pole's pins
    /// against the netlist in both directions (a pin no pole names, or a
    /// pole naming a pin the part lacks, is a hard error). Poles are
    /// addressed by their index here, in order, from `Scenario::switch`; a
    /// closed pole is an identity union of its two nets at build.
    ///
    /// Two-pin `SW_*` symbols need no declaration — the auto tier gives them
    /// one open pole across pins `1` and `2`.
    pub fn register_switch(&mut self, part: impl Into<String>, poles: Vec<SwitchPole>) {
        self.entries
            .insert(part.into(), RegistryEntry::Switch(poles));
    }

    /// Declare a part name — or a manufacturer part number, or a value — to
    /// be a **piecewise-linear element** with the given specification: its
    /// pins (validated against the netlist in both directions) and the
    /// branches between them ([`PwlSpec`]). A diode, an LED, a FET or BJT
    /// channel with no Rust model behind it. The key is looked up like any
    /// other: part name first, then the netlist's manufacturer part number,
    /// then the value — so a library entry keyed `LTST-C190KGKT` classifies
    /// the LEDs whose value is the bare `LED` once the auto tier yields to
    /// it, and one keyed `SS36` classifies by value.
    pub fn register_pwl(&mut self, key: impl Into<String>, spec: PwlSpec) {
        self.entries.insert(key.into(), RegistryEntry::Pwl(spec));
    }

    /// Declare a part name — or, on a netlist with no libsource, a value —
    /// to be a **mechanical** part: a node with pads and nothing electrical.
    ///
    /// The auto tier already recognizes `MountingHole*`, `Logo*` and
    /// `Fiducial*` symbols. A transcribed netlist's BOM-only lines (a raw
    /// PCB with no nodes, a layout node, a mounting hole drawn as a pad
    /// symbol) carry no part name and match no prefix; declaring their
    /// values here makes them the explicit mechanical nodes they are, so the
    /// board builds with every part classified and none excused.
    pub fn register_mechanical(&mut self, part: impl Into<String>) {
        self.entries.insert(part.into(), RegistryEntry::Mechanical);
    }

    /// Declare a part name to be a **board boundary** — a connector or socket
    /// whose pins are harness attachment points and which contributes nothing
    /// electrical of its own.
    ///
    /// The auto tier already recognizes KiCad's standard `Conn*` /
    /// `Screw_Terminal*` symbols. Boards that draw their own connector symbol
    /// (`P2_EDGE_MODULE_SOCKET`, `Edge Socket Pads`, a shrouded-header
    /// footprint from a project library) match no prefix and would otherwise
    /// fail to classify, where the truth is "this is where the harness plugs
    /// in". Declaring the part name here classifies every instance as
    /// [`Classification::Boundary`], exactly like a `Conn_01x05`.
    ///
    /// Boundary declarations win over the registry: a part that is both
    /// registered and declared boundary classifies as a boundary (the
    /// declaration is the more specific statement about the board).
    pub fn register_boundary(&mut self, part: impl Into<String>) {
        self.boundaries.insert(part.into());
    }

    /// True when `part` (already rescue-normalized) is a declared boundary.
    pub fn is_boundary(&self, part: &str) -> bool {
        self.boundaries.contains(part)
    }

    /// Opt this registry into classifying components whose libsource **part
    /// name is empty** from their reference-designator prefix
    /// ([`reference_designator_class`]).
    ///
    /// Off by default, and deliberately so. In an EDA export every component
    /// carries a `(libsource (part …))`, so an empty part name there means the
    /// export is damaged or the fixture is truncated — silently classifying
    /// `C7` as a capacitor because of its *name* would paper over that. The
    /// fallback exists for netlists that never had a symbol library to name:
    /// a whole-module netlist transcribed from a vendor PDF (this crate's
    /// `p2_ec32mb.net` fixture), or a hand-written harness description.
    /// Components with a non-empty part name are unaffected either way.
    pub fn classify_unnamed_by_reference(&mut self, enabled: bool) {
        self.reference_fallback = enabled;
    }

    /// True when a registry entry of any kind exists for the (already
    /// normalized) part name or value.
    pub fn has_part(&self, part: &str) -> bool {
        self.entries.contains_key(part)
    }

    /// The registry entry for a declaration, keyed by normalized part name,
    /// then by the netlist's manufacturer part number, then by `value`.
    fn entry(&self, decl: &ComponentDecl) -> Option<&RegistryEntry> {
        let part = normalize_part(decl);
        self.entries
            .get(part.as_str())
            .or_else(|| decl.mpn.as_deref().and_then(|mpn| self.entries.get(mpn)))
            .or_else(|| self.entries.get(decl.value.as_str()))
    }

    /// Construct the registered component for a declaration, keyed by
    /// normalized part name, manufacturer part number, then `value`. `None`
    /// when no component constructor matches (an entry of another kind, or
    /// none).
    pub fn construct(&self, decl: &ComponentDecl) -> Option<Box<dyn Component>> {
        match self.entry(decl) {
            Some(RegistryEntry::Component(ctor)) => Some(ctor(decl)),
            _ => None,
        }
    }

    /// Classify one netlist component through the three tiers. `pin_count`
    /// is the component's node count from the netlist — a 2-terminal passive
    /// class with a different pin count is a hard classification error, and
    /// so is a test point with more than one pin.
    pub fn classify(
        &self,
        decl: &ComponentDecl,
        pin_count: usize,
    ) -> Result<Classification, RegistryError> {
        let part = normalize_part(decl);

        // The name the AUTO tiers match on. Normally the libsource part name;
        // for a netlist that carries no libsource at all, and only when the
        // consumer opted in, a class name synthesized from the reference
        // designator prefix (see `classify_unnamed_by_reference`). Tier 2 and
        // the error tier always report the real part name, so a synthetic
        // class never leaks into a registry key or a diagnostic.
        let synthetic_class = (part.is_empty() && self.reference_fallback)
            .then(|| reference_designator_class(&decl.reference))
            .flatten();
        let auto: &str = synthetic_class.unwrap_or(part.as_str());

        // An EXPLICIT registration beats a SYNTHESIZED class. The reference
        // designator is a guess made only because the netlist carried no part
        // name; an entry the consumer registered for this component's `value`
        // is a statement of intent, and a guess must not override one.
        //
        // Without this a component can be unmountable for a reason nothing
        // reports: `J301` on the P2-EC32MB fixture is a microSD socket with a
        // card behind it, and a `J` prefix classified it as a board boundary
        // before the registry was ever consulted — so registering a live card
        // silently did nothing, and the board built without it. The same
        // rule is what lets `J101` (a solder link) be the one-pole switch it
        // is, and `J701`/`J702` (mounting holes) mechanical.
        //
        // Narrow on purpose: a class from a REAL libsource part name (a
        // `Conn_01x04` symbol, say) still wins, because there the netlist is
        // telling us what the part is rather than us inferring it.
        if synthetic_class.is_some() {
            if let Some(entry) = self.entries.get(decl.value.as_str()) {
                return Ok(entry.classification());
            }
        }

        // Tier 1a: mechanical parts and test points — nodes with pads.
        if starts_with_any(auto, &["MountingHole", "Logo", "Fiducial"]) {
            return Ok(Classification::Mechanical);
        }
        if auto.starts_with("TestPoint") {
            match pin_count {
                // A probe: one pad, sensed, never driven.
                1 => return Ok(Classification::Probe),
                // A test point on no net is a pad with nothing electrical —
                // the export still carries the symbol, and the board still
                // builds.
                0 => return Ok(Classification::Mechanical),
                // A multi-pin `TestPoint_*` symbol (a `TestPoint_2Pole`, a
                // probe header) is not a probe; what it is, the consumer
                // says by registering it, and only an unregistered one is
                // the pin-count error.
                _ => {
                    if let Some(entry) = self.entry(decl) {
                        return Ok(entry.classification());
                    }
                    return Err(RegistryError::BadPinCount {
                        reference: decl.reference.clone(),
                        part,
                        expected: 1,
                        found: pin_count,
                    });
                }
            }
        }

        // Tier 1b: connectors / screw terminals — board boundary pins. The
        // consumer's explicit declarations join this tier for project-library
        // connector symbols that match no prefix.
        if starts_with_any(auto, &["Conn", "Screw_Terminal"]) || self.is_boundary(&part) {
            return Ok(Classification::Boundary);
        }

        // Tier 1c: jumpers — stateful shorts, default state from the name.
        if auto.starts_with("Jumper") || auto.starts_with("SolderJumper") {
            // A three-pad jumper is two poles from the common pad: the
            // KiCad `Jumper_3_*` / `SolderJumper_3_*` pinout is 1 = A,
            // 2 = C (the common), 3 = B, so pole 0 is 1–2 and pole 1 is
            // 2–3, each open unless the name bridges it (`_Bridged12`,
            // `_Bridged23`, `_Bridged123`). A jumper with one closed pole
            // per throw is what the symbol draws; one short across all
            // three pads is what a single closed edge used to make of it.
            // An explicit registration pairs the pads otherwise.
            if auto.starts_with("Jumper_3") || auto.starts_with("SolderJumper_3") {
                if let Some(entry) = self.entry(decl) {
                    return Ok(entry.classification());
                }
                let bridged = |pole: &str| auto.contains("_Bridged123") || auto.contains(pole);
                let pole = |a: &str, b: &str, closed: bool| {
                    if closed {
                        SwitchPole::closed(a, b)
                    } else {
                        SwitchPole::open(a, b)
                    }
                };
                return Ok(Classification::Switch {
                    poles: vec![
                        pole("1", "2", bridged("_Bridged12")),
                        pole("2", "3", bridged("_Bridged23")),
                    ],
                });
            }
            let default = if auto.contains("_NC") || auto.contains("_Bridged") {
                JumperState::Closed
            } else {
                // `_NO` / `_Open` / unmarked jumpers default open.
                JumperState::Open
            };
            return Ok(Classification::Jumper { default });
        }

        // Tier 1d: two-pin switches — one open pole across pins 1 and 2, the
        // pinout of every two-terminal symbol in the KiCad `Switch` library
        // (`SW_Push`, `SW_SPST`, `SW_DIP_x01`, …). A `SW_*` symbol with more
        // pins pairs them off in a way the name does not say, so it is a
        // `register_switch` declaration like any other multi-pole part.
        //
        // The pole pairing here is synthesized from a library convention,
        // and an explicit registration beats a synthesized class (the rule
        // above): a project-library `SW_*` symbol whose pins are not `1`/`2`
        // can only be built through `register_switch`, and a model
        // registered for a switch part name must not be overridden by the
        // guess.
        if auto.starts_with("SW_") && pin_count == 2 {
            if let Some(entry) = self.entry(decl) {
                return Ok(entry.classification());
            }
            return Ok(Classification::Switch {
                poles: vec![SwitchPole::open("1", "2")],
            });
        }

        // Tier 1e: passive primitives — the part-name class is anchored so
        // e.g. "RJ45" never classifies as a resistor.
        //
        // A diode or LED symbol says nothing about the purchasable part's
        // knee: the part is the registration — by part name, by the
        // manufacturer part number the export carries, or by value — that
        // carries its datasheet's forward drop (`NODES.md` §2, the Diode /
        // LED row: "no resolvable Vf = build error naming the part, like
        // any other unmodelled part"). A diode with none is an unknown
        // part, and the error names the number the export gave it.
        if let Some(kind) = passive_kind(auto) {
            if matches!(kind, PassiveKind::Diode | PassiveKind::Led) {
                return match self.entry(decl) {
                    Some(entry) => Ok(entry.classification()),
                    None => Err(RegistryError::UnknownPart {
                        reference: decl.reference.clone(),
                        part,
                        value: decl.value.clone(),
                        mpn: decl.mpn.clone(),
                    }),
                };
            }
            if pin_count != 2 {
                return Err(RegistryError::BadPinCount {
                    reference: decl.reference.clone(),
                    part,
                    expected: 2,
                    found: pin_count,
                });
            }
            return Ok(Classification::Passive {
                kind,
                value: parse_passive_value(&decl.value),
            });
        }

        // Tier 2: consumer registry, keyed on normalized part name, then
        // the manufacturer part number, then the value field.
        if let Some(entry) = self.entry(decl) {
            return Ok(entry.classification());
        }

        // Tier 3: hard error, naming what could not be classified. There is
        // no escape: a part is a node whose class has behaviour, or the board
        // does not build.
        Err(RegistryError::UnknownPart {
            reference: decl.reference.clone(),
            part,
            value: decl.value.clone(),
            mpn: decl.mpn.clone(),
        })
    }
}

/// True when `part` starts with any of the given prefixes.
fn starts_with_any(part: &str, prefixes: &[&str]) -> bool {
    prefixes.iter().any(|p| part.starts_with(p))
}

// ============================================================
// Reference-designator classification (no-libsource fallback)
// ============================================================

/// The auto-tier class name a reference designator implies, for netlists that
/// carry no libsource part name to key on (see
/// [`PartRegistry::classify_unnamed_by_reference`]).
///
/// The **whole leading alphabetic run** of the reference must equal a table
/// entry — `R7` and `R` match `R`, while `RN7` (a resistor network) and
/// `PCB` match nothing and fall through to the registry. Matching the whole
/// run rather than a prefix is what keeps a multi-letter designator class from
/// being silently mistaken for a single-letter one, which is the only way this
/// fallback could quietly misclassify a part.
///
/// | Reference prefix | Class | Result |
/// |---|---|---|
/// | `R` | `R` | resistor (value parsed; conducts) |
/// | `C` | `C` | capacitor (DC-open in the build pass) |
/// | `L` | `L` | inductor (DC short) |
/// | `D` | `D` | diode — LEDs share the `D` prefix and are DC-open either way |
/// | `J`, `P` | `Conn` | board boundary (connector, socket, pad pair) |
/// | `TP` | `TestPoint` | probe node |
/// | `H`, `MK` | `MountingHole` | mechanical node |
///
/// Everything else — `U`, `IC`, `Q`, `X`, `S`, `SW`, `FB`, … — returns `None`
/// so active silicon still has to be registered by the consumer. That is the
/// point: the fallback classifies the parts whose behavior the engine already
/// knows, and refuses to guess at the ones it does not.
pub fn reference_designator_class(reference: &str) -> Option<&'static str> {
    let prefix: String = reference
        .chars()
        .take_while(|c| c.is_ascii_alphabetic())
        .flat_map(char::to_uppercase)
        .collect();
    match prefix.as_str() {
        "R" => Some("R"),
        "C" => Some("C"),
        "L" => Some("L"),
        "D" => Some("D"),
        "J" | "P" => Some("Conn"),
        "TP" => Some("TestPoint"),
        "H" | "MK" => Some("MountingHole"),
        _ => None,
    }
}

/// Passive-primitive class from an anchored part-name pattern: the class
/// letter(s) must be the whole name or be followed by `_` (so `R_Small` and
/// `R` classify, `RJ45` does not).
fn passive_kind(part: &str) -> Option<PassiveKind> {
    let anchored = |prefix: &str| {
        part == prefix
            || part
                .strip_prefix(prefix)
                .is_some_and(|rest| rest.starts_with('_'))
    };
    if anchored("LED") {
        return Some(PassiveKind::Led);
    }
    if anchored("R") {
        return Some(PassiveKind::Resistor);
    }
    if anchored("C") || anchored("C_Polarized") {
        return Some(PassiveKind::Capacitor);
    }
    if anchored("L") {
        return Some(PassiveKind::Inductor);
    }
    if anchored("D") {
        return Some(PassiveKind::Diode);
    }
    None
}

/// Rescue-normalize a component's part name. When a symbol was rescued,
/// KiCad renames it `<origPart>-<origLib>` inside a `<sheet>-rescue` library
/// (`DS2_Addon-rescue :: Jumper_NO_Small-Device` → `Jumper_NO_Small`), so the
/// original-lib suffix is stripped only when the declaring lib is a rescue
/// lib — a bare part name containing `-` is otherwise left alone.
pub fn normalize_part(decl: &ComponentDecl) -> String {
    if decl.lib.ends_with("-rescue") {
        if let Some(idx) = decl.part.rfind('-') {
            return decl.part[..idx].to_string();
        }
    }
    decl.part.clone()
}

/// Backwards-compatible part-name normalization when no lib context is
/// available (identity — rescue stripping needs the lib name; prefer
/// [`normalize_part`]).
pub fn normalize_part_name(part: &str) -> String {
    part.to_string()
}

/// Parse a passive value field into base SI units (Ω / F / H).
///
/// **First-token parsing.** The electrical value is the first token of the
/// field in every convention this crate has met — `"4.7uF 6.3V"`,
/// `"22uF 25V"`, `"47uH/3A"`, `"3.3uH 6.6A 28.6mOhm"`, `"1nF (1000pF)"` —
/// and everything after whitespace, a slash, a comma or an opening
/// parenthesis is a rating, a tolerance or an alias, which a DC model does
/// not read. So the field is cut at the first of those and the token parsed.
///
/// Grammar of the token (case significant for multipliers): digits with
/// either a decimal point or an embedded multiplier letter acting as one
/// (`4k7` = 4.7 k, `2R2` = 2.2), an optional multiplier (`p n u µ m k K M G`,
/// plus `R` = ×1 for resistors), and an optional unit (`R Ω Ohm F H`) which is
/// ignored. Returns `None` for non-numeric fields (`"X"`, `"ADS122U04"`,
/// `"White"`, `"DIP Switch 4 way"`).
pub fn parse_passive_value(value: &str) -> Option<f64> {
    let token = value
        .trim()
        .split(|c: char| c.is_whitespace() || matches!(c, '/' | ',' | '('))
        .next()?;
    let v = token.trim_end_matches('Ω');
    let v = v
        .strip_suffix("Ohms")
        .or_else(|| v.strip_suffix("Ohm"))
        .or_else(|| v.strip_suffix("ohms"))
        .or_else(|| v.strip_suffix("ohm"))
        .unwrap_or(v);
    // Strip a trailing unit letter (F/H); R is handled as a multiplier below.
    let v = v.strip_suffix(['F', 'H']).unwrap_or(v);
    if v.is_empty() {
        return None;
    }

    let multiplier = |c: char| -> Option<f64> {
        match c {
            'p' => Some(1e-12),
            'n' => Some(1e-9),
            'u' | 'µ' => Some(1e-6),
            'm' => Some(1e-3),
            'k' | 'K' => Some(1e3),
            'M' => Some(1e6),
            'G' => Some(1e9),
            'R' => Some(1.0),
            _ => None,
        }
    };

    // Split at the first non-digit, non-dot character: that's the multiplier
    // (possibly embedded: "4k7"), anything after it must be digits (the
    // fractional part).
    let chars: Vec<char> = v.chars().collect();
    let split = chars.iter().position(|c| !c.is_ascii_digit() && *c != '.');

    match split {
        None => v.parse::<f64>().ok(),
        Some(i) => {
            let mult = multiplier(chars[i])?;
            let int_part: String = chars[..i].iter().collect();
            let frac_part: String = chars[i + 1..].iter().collect();
            if !frac_part.chars().all(|c| c.is_ascii_digit()) {
                return None;
            }
            if int_part.is_empty() && frac_part.is_empty() {
                return None;
            }
            let number: f64 = if frac_part.is_empty() {
                int_part.parse().ok()?
            } else {
                format!("{int_part}.{frac_part}").parse().ok()?
            };
            Some(number * mult)
        }
    }
}

// ============================================================
// Errors
// ============================================================

/// Classification failure (tier 3, or an auto-tier validation violation).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RegistryError {
    /// No auto-tier match and no registry entry — the board does not build.
    /// Names the reference, the part and the value, because on a netlist
    /// with no libsource the part is empty and the value is the only name
    /// the part has; and the manufacturer part number where the export
    /// carries one, because that is the key a library entry would take.
    UnknownPart {
        /// Component reference designator.
        reference: String,
        /// Normalized part name that failed to match.
        part: String,
        /// The component's value field.
        value: String,
        /// The manufacturer part number the export carries, if any.
        mpn: Option<String>,
    },
    /// An auto-classified class with a different netlist pin count than it
    /// requires (a 2-terminal primitive, a 1-pin test point).
    BadPinCount {
        /// Component reference designator.
        reference: String,
        /// Normalized part name.
        part: String,
        /// Pins the class requires.
        expected: usize,
        /// Pins the netlist has.
        found: usize,
    },
}

impl fmt::Display for RegistryError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            RegistryError::UnknownPart {
                reference,
                part,
                value,
                mpn,
            } => {
                write!(
                    f,
                    "{reference}: no classification or registry entry for part {part:?} with value {value:?}"
                )?;
                match mpn {
                    Some(mpn) => write!(f, " (manufacturer part number {mpn:?})"),
                    None => Ok(()),
                }
            }
            RegistryError::BadPinCount { reference, part, expected, found } => write!(
                f,
                "{reference}: part {part:?} classifies as a {expected}-terminal primitive but has {found} pins"
            ),
        }
    }
}

impl std::error::Error for RegistryError {}

#[cfg(test)]
mod tests {
    use rstest::rstest;
    use vibes_behaviour::{behaviour, expect, Test};

    use super::*;
    use crate::component::{AttachError, ComponentNetIo, PinDecl};

    struct NullComponent;

    impl Component for NullComponent {
        fn pins(&self) -> &[PinDecl] {
            &[]
        }
        fn attach(&mut self, _io: ComponentNetIo) -> Result<(), AttachError> {
            Ok(())
        }
    }

    fn decl(part: &str, value: &str) -> ComponentDecl {
        ComponentDecl {
            reference: "U1".to_string(),
            value: value.to_string(),
            footprint: String::new(),
            lib: String::new(),
            part: part.to_string(),
            sheetpath: "/".to_string(),
            dnp: false,
            mpn: None,
        }
    }

    fn decl_with_lib(lib: &str, part: &str, value: &str) -> ComponentDecl {
        ComponentDecl {
            lib: lib.to_string(),
            ..decl(part, value)
        }
    }

    /// Relative-tolerance assertion for parsed SI values (multiplier
    /// arithmetic accumulates ULP-level float error).
    fn assert_parses_to(input: &str, expected: f64) {
        let v = parse_passive_value(input).unwrap_or_else(|| panic!("{input:?} failed to parse"));
        assert!(
            ((v - expected) / expected).abs() < 1e-12,
            "{input:?} parsed to {v}, expected {expected}"
        );
    }

    #[rstest]
    fn passive_values_parse_across_notations() {
        assert_parses_to("47R", 47.0);
        assert_parses_to("4k7", 4700.0);
        assert_parses_to("2R2", 2.2);
        assert_parses_to("10k", 10_000.0);
        assert_parses_to("1M", 1_000_000.0);
        assert_parses_to("0.1uF", 1e-7);
        assert_parses_to("2.2nF", 2.2e-9);
        assert_parses_to("100n", 1e-7);
        assert_parses_to("10uH", 1e-5);
        assert_parses_to("330", 330.0);
        assert_eq!(parse_passive_value("X"), None);
        assert_eq!(parse_passive_value("ADS122U04"), None);
        assert_eq!(parse_passive_value(""), None);
    }

    /// The value is the first token; what follows is a rating, a tolerance
    /// or an alias, and a DC model reads none of it.
    #[rstest]
    #[case::cap_with_voltage_rating("4.7uF 6.3V", 4.7e-6)]
    #[case::cap_with_higher_rating("22uF 25V", 22e-6)]
    #[case::inductor_with_current_rating("47uH/3A", 47e-6)]
    #[case::inductor_with_current_and_dcr("3.3uH 6.6A 28.6mOhm", 3.3e-6)]
    #[case::cap_with_alias_in_parens("1nF (1000pF)", 1e-9)]
    #[case::upper_case_kilo("10.5K", 10_500.0)]
    #[case::embedded_kilo("4k7", 4700.0)]
    #[case::resistor_r_unit("240R", 240.0)]
    #[case::resistor_with_tolerance("10k 1%", 10_000.0)]
    #[case::ohm_unit_word("28.6mOhm", 28.6e-3)]
    #[case::ohm_symbol("47Ω", 47.0)]
    #[case::comma_separated_rating("100nF,50V", 1e-7)]
    fn the_first_token_of_a_value_field_is_the_value(#[case] field: &str, #[case] expected: f64) {
        assert_parses_to(field, expected);
    }

    /// A field whose first token is a word is not a value, whatever follows.
    #[rstest]
    #[case::dnp("X")]
    #[case::part_number("ADS122U04")]
    #[case::colour("White")]
    #[case::switch("DIP Switch 4 way")]
    #[case::regulator("LDO 300mA, 3.3V")]
    #[case::processor("P2X8C4M64P")]
    #[case::empty("")]
    #[case::blank("   ")]
    fn a_field_that_starts_with_a_word_has_no_value(#[case] field: &str) {
        assert_eq!(parse_passive_value(field), None);
    }

    #[rstest]
    fn classifies_real_ds2addon_components() {
        let registry = {
            let mut r = PartRegistry::new();
            r.register("ADS122U04", |_| Box::new(NullComponent));
            r
        };

        // R3 47R — populated series resistor.
        assert_eq!(
            registry.classify(&decl_with_lib("Device", "R_Small", "47R"), 2),
            Ok(Classification::Passive {
                kind: PassiveKind::Resistor,
                value: Some(47.0)
            })
        );
        // R6 X — DNP by value; still classifies (absence is the board's call).
        assert_eq!(
            registry.classify(&decl_with_lib("Device", "R_Small", "X"), 2),
            Ok(Classification::Passive {
                kind: PassiveKind::Resistor,
                value: None
            })
        );
        // JP1 — rescue-mangled jumper, defaults open.
        assert_eq!(
            registry.classify(
                &decl_with_lib("DS2_Addon-rescue", "Jumper_NO_Small-Device", "A0_bypass"),
                2
            ),
            Ok(Classification::Jumper {
                default: JumperState::Open,
            })
        );
        // J1 — generic connector = boundary.
        assert_eq!(
            registry.classify(&decl_with_lib("Connector_Generic", "Conn_01x05", "MCU"), 5),
            Ok(Classification::Boundary)
        );
        // U1 — registry part.
        assert_eq!(
            registry.classify(&decl_with_lib("DS2_Addon", "ADS122U04", "ADS122U04"), 16),
            Ok(Classification::Registered)
        );
        // Unknown part with no registry entry -> hard error naming the part
        // and its value.
        assert_eq!(
            registry.classify(&decl_with_lib("Weird", "FrobulatorX", "?"), 4),
            Err(RegistryError::UnknownPart {
                reference: "U1".to_string(),
                part: "FrobulatorX".to_string(),
                value: "?".to_string(),
                mpn: None,
            })
        );
    }

    /// The error a consumer reads names all three things they need to fix
    /// it: which part, which symbol, which value.
    #[rstest]
    fn an_unknown_part_error_names_the_reference_the_part_and_the_value() {
        let registry = PartRegistry::new();
        let error = registry
            .classify(&decl_with_lib("Weird", "FrobulatorX", "FX-7"), 4)
            .expect_err("nothing classifies a FrobulatorX");
        let rendered = error.to_string();
        for needle in ["U1", "FrobulatorX", "FX-7"] {
            assert!(rendered.contains(needle), "{rendered:?} lacks {needle}");
        }
    }

    #[rstest]
    fn passive_matching_is_anchored_and_pin_count_validated() {
        let registry = PartRegistry::new();
        // RJ45 must NOT classify as a resistor (and has no registry entry).
        assert!(matches!(
            registry.classify(&decl_with_lib("Connector", "RJ45", "RJ45"), 8),
            Err(RegistryError::UnknownPart { .. })
        ));
        // A 3-pin "R_Small" is a pin-count violation, not a resistor.
        assert_eq!(
            registry.classify(&decl_with_lib("Device", "R_Small", "10k"), 3),
            Err(RegistryError::BadPinCount {
                reference: "U1".to_string(),
                part: "R_Small".to_string(),
                expected: 2,
                found: 3
            })
        );
        // EdgeBoard jumpers: Jumper_2_Open is a jumper, Jumper_3_Open a
        // two-pole switch from its common pad 2 (see the three-pad case).
        assert_eq!(
            registry.classify(&decl_with_lib("Jumper", "Jumper_2_Open", "JP"), 2),
            Ok(Classification::Jumper {
                default: JumperState::Open,
            })
        );
        assert_eq!(
            registry.classify(&decl_with_lib("Jumper", "Jumper_3_Open", "JP"), 3),
            Ok(Classification::Switch {
                poles: vec![SwitchPole::open("1", "2"), SwitchPole::open("2", "3")]
            })
        );
    }

    /// A three-pad jumper is two poles from its common pad, open or bridged
    /// as the symbol name says; a registration for the part name pairs the
    /// pads its own way.
    #[rstest]
    #[case::open("Jumper_3_Open", false, false)]
    #[case::bridged12("Jumper_3_Bridged12", true, false)]
    #[case::bridged23("SolderJumper_3_Bridged23", false, true)]
    #[case::bridged123("SolderJumper_3_Bridged123", true, true)]
    fn a_three_pad_jumper_is_a_two_pole_switch(
        #[case] part: &str,
        #[case] pole0_closed: bool,
        #[case] pole1_closed: bool,
    ) {
        behaviour!(Test {
            id: "registry.three-pad-jumper-two-poles",
            covers: Some("board/src/registry.rs#PartRegistry::classify"),
            given: "a three-pad jumper symbol, open or bridged on one or both throws by its name",
        });
        expect!(
            "two-poles",
            "the part is a switch with two poles, each from the common pad to one throw, \
             closed exactly where the symbol name bridges it",
            "a three-pad jumper selects one throw at a time, and a single short across all \
             three pads would tie the throws to each other"
        );
        let registry = PartRegistry::new();
        let state = |closed: bool| {
            if closed {
                JumperState::Closed
            } else {
                JumperState::Open
            }
        };
        assert_eq!(
            registry.classify(&decl_with_lib("Jumper", part, "JP1"), 3),
            Ok(Classification::Switch {
                poles: vec![
                    SwitchPole {
                        a: "1".into(),
                        b: "2".into(),
                        state: state(pole0_closed)
                    },
                    SwitchPole {
                        a: "2".into(),
                        b: "3".into(),
                        state: state(pole1_closed)
                    },
                ]
            })
        );
        let mut registry = PartRegistry::new();
        registry.register_switch(
            "Jumper_3_Open",
            vec![SwitchPole::open("A", "C"), SwitchPole::open("C", "B")],
        );
        assert_eq!(
            registry.classify(&decl_with_lib("Jumper", "Jumper_3_Open", "JP1"), 3),
            Ok(Classification::Switch {
                poles: vec![SwitchPole::open("A", "C"), SwitchPole::open("C", "B")]
            })
        );
    }

    // --------------------------------------------------------
    // Mechanical parts, test points, switches
    // --------------------------------------------------------

    /// A mounting hole, a logo or a fiducial is a mechanical node — a part
    /// the board carries — and a test point a probe node; a test point on
    /// no net is a pad, mechanical.
    #[rstest]
    #[case::mounting_hole("Mechanical", "MountingHole_Pad", 1, Classification::Mechanical)]
    #[case::logo("Graphic", "Logo_Open_Hardware_Small", 0, Classification::Mechanical)]
    #[case::fiducial("Mechanical", "Fiducial", 0, Classification::Mechanical)]
    #[case::test_point("Connector", "TestPoint", 1, Classification::Probe)]
    #[case::unconnected_test_point("Connector", "TestPoint", 0, Classification::Mechanical)]
    fn mechanical_symbols_and_test_points_are_nodes(
        #[case] lib: &str,
        #[case] part: &str,
        #[case] pins: usize,
        #[case] expected: Classification,
    ) {
        let registry = PartRegistry::new();
        assert_eq!(
            registry.classify(&decl_with_lib(lib, part, "~"), pins),
            Ok(expected)
        );
    }

    /// A probe has one pin; a `TestPoint` symbol with more is not one, and
    /// nothing registered for it is the pin-count error.
    #[rstest]
    fn a_test_point_with_more_than_one_pin_is_a_pin_count_error() {
        let registry = PartRegistry::new();
        assert_eq!(
            registry.classify(&decl_with_lib("Connector", "TestPoint_2Pole", "TP"), 2),
            Err(RegistryError::BadPinCount {
                reference: "U1".to_string(),
                part: "TestPoint_2Pole".to_string(),
                expected: 1,
                found: 2
            })
        );
    }

    /// A multi-pin `TestPoint` symbol is what the consumer registers it as
    /// — a two-pole probe header is a switch across its pads, a probe
    /// connector a model — reached through the same registry tier as any
    /// other part the auto tier cannot place.
    #[rstest]
    fn a_multi_pin_test_point_symbol_classifies_as_registered() {
        let mut registry = PartRegistry::new();
        registry.register_switch("TestPoint_2Pole", vec![SwitchPole::open("1", "2")]);
        assert_eq!(
            registry.classify(&decl_with_lib("Connector", "TestPoint_2Pole", "TP"), 2),
            Ok(Classification::Switch {
                poles: vec![SwitchPole::open("1", "2")]
            })
        );
        registry.register("TestPoint_Probe", |_| Box::new(NullComponent));
        assert_eq!(
            registry.classify(&decl_with_lib("Connector", "TestPoint_Probe", "TP"), 4),
            Ok(Classification::Registered)
        );
    }

    /// A two-pin `SW_*` symbol is a one-pole switch across pins 1 and 2,
    /// open; a `SW_*` symbol with more pins is a registry declaration.
    #[rstest]
    fn two_pin_switch_symbols_get_one_open_pole() {
        let registry = PartRegistry::new();
        assert_eq!(
            registry.classify(&decl_with_lib("Switch", "SW_Push", "RESET"), 2),
            Ok(Classification::Switch {
                poles: vec![SwitchPole::open("1", "2")]
            })
        );
        // A DPDT symbol pairs six pins in a way its name does not say.
        assert!(matches!(
            registry.classify(&decl_with_lib("Switch", "SW_DPDT_x2", "S1"), 6),
            Err(RegistryError::UnknownPart { .. })
        ));
    }

    /// The two-pin `SW_*` pole across pins 1 and 2 is a guess from the
    /// library convention; a registration for the part name — a switch
    /// whose pads are named otherwise, or a model — beats it.
    #[rstest]
    fn an_explicit_registration_beats_the_two_pin_switch_guess() {
        let mut registry = PartRegistry::new();
        registry.register_switch("SW_Reset", vec![SwitchPole::open("A", "B")]);
        assert_eq!(
            registry.classify(&decl_with_lib("Project", "SW_Reset", "RESET"), 2),
            Ok(Classification::Switch {
                poles: vec![SwitchPole::open("A", "B")]
            })
        );
        registry.register("SW_Push", |_| Box::new(NullComponent));
        assert_eq!(
            registry.classify(&decl_with_lib("Switch", "SW_Push", "RESET"), 2),
            Ok(Classification::Registered)
        );
    }

    #[rstest]
    fn rescue_normalization_requires_rescue_lib() {
        assert_eq!(
            normalize_part(&decl_with_lib(
                "DS2_Addon-rescue",
                "Jumper_NO_Small-Device",
                ""
            )),
            "Jumper_NO_Small"
        );
        // A hyphenated part in a normal lib is left alone.
        assert_eq!(
            normalize_part(&decl_with_lib("SomeLib", "PART-7", "")),
            "PART-7"
        );
    }

    /// A registered switch classifies with exactly the poles it declared,
    /// in order, with their default states.
    #[rstest]
    fn a_registered_switch_carries_its_declared_poles() {
        let mut registry = PartRegistry::new();
        registry.register_switch(
            "DIP Switch 4 way",
            vec![
                SwitchPole::open("1_ON", "1_OFF"),
                SwitchPole::closed("2_ON", "2_OFF"),
            ],
        );
        assert_eq!(
            registry.classify(&decl("DIP Switch 4 way", "S301"), 4),
            Ok(Classification::Switch {
                poles: vec![
                    SwitchPole::open("1_ON", "1_OFF"),
                    SwitchPole::closed("2_ON", "2_OFF"),
                ]
            })
        );
        assert!(registry.has_part("DIP Switch 4 way"));
        // A switch is not a component: nothing to construct.
        assert!(registry
            .construct(&decl("DIP Switch 4 way", "S301"))
            .is_none());
    }

    /// A registered piecewise-linear element classifies with its
    /// specification — its pins and branches.
    #[rstest]
    fn a_registered_pwl_element_carries_its_spec() {
        let mut registry = PartRegistry::new();
        let spec = PwlSpec::diode("A", "K", 0.75, 0.0);
        registry.register_pwl("SS36", spec.clone());
        assert_eq!(
            registry.classify(&decl_with_lib("Diode", "SS36", "SS36"), 2),
            Ok(Classification::Pwl { spec: spec.clone() })
        );
        assert_eq!(spec.pins, vec!["A".to_string(), "K".to_string()]);
        assert_eq!(spec.branches.len(), 1);
    }

    /// A diode or LED symbol yields to an explicit registration — by the
    /// manufacturer part number the export carries, or by value — and
    /// stays the open passive it always was when there is none; a resistor
    /// symbol never yields.
    #[rstest]
    fn a_diode_symbol_yields_to_a_registered_element_and_stays_open_otherwise() {
        let mut registry = PartRegistry::new();
        registry.register_pwl("LTST-C190KGKT", PwlSpec::diode("2", "1", 2.0, 0.0));
        registry.register_pwl("SS36", PwlSpec::diode("2", "1", 0.75, 0.0));
        let mut led = decl_with_lib("Device", "LED", "LED");
        led.mpn = Some("LTST-C190KGKT".to_string());
        assert!(matches!(
            registry.classify(&led, 2),
            Ok(Classification::Pwl { .. })
        ));
        // An LED or diode symbol with no entry is an unknown part — the
        // symbol carries no knee — and the error names the number the
        // export gave it, the key an entry would take.
        assert_eq!(
            registry.classify(&decl_with_lib("Device", "LED", "LED"), 2),
            Err(RegistryError::UnknownPart {
                reference: "U1".to_string(),
                part: "LED".to_string(),
                value: "LED".to_string(),
                mpn: None,
            })
        );
        assert!(matches!(
            registry.classify(&decl_with_lib("Device", "D_Schottky_Small", "SS36"), 2),
            Ok(Classification::Pwl { .. })
        ));
        let mut unknown = decl_with_lib("Device", "D_Schottky_Small", "1N5819");
        unknown.mpn = Some("1N5819HW-7-F".to_string());
        let error = registry
            .classify(&unknown, 2)
            .expect_err("no entry keys the part, its number or its value");
        assert_eq!(
            error,
            RegistryError::UnknownPart {
                reference: "U1".to_string(),
                part: "D_Schottky_Small".to_string(),
                value: "1N5819".to_string(),
                mpn: Some("1N5819HW-7-F".to_string()),
            }
        );
        assert!(
            error.to_string().contains("1N5819HW-7-F"),
            "the message names the number: {error}"
        );
        // A resistor symbol is a resistor whatever is registered for its value.
        registry.register_mechanical("220");
        assert_eq!(
            registry.classify(&decl_with_lib("Device", "R", "220"), 2),
            Ok(Classification::Passive {
                kind: PassiveKind::Resistor,
                value: Some(220.0)
            })
        );
    }

    /// The netlist's manufacturer part number is a registry key between the
    /// part name and the value: an entry keyed on it classifies a part
    /// whose part name matches nothing, and the part name still wins over
    /// it.
    #[rstest]
    fn a_manufacturer_part_number_is_looked_up_between_the_part_name_and_the_value() {
        let mut registry = PartRegistry::new();
        registry.register_pwl("SS36-E3/57T", PwlSpec::diode("2", "1", 0.75, 0.0));
        registry.register_mechanical("SS36");
        let mut with_mpn = decl_with_lib("Diode", "Schottky", "SS36");
        with_mpn.mpn = Some("SS36-E3/57T".to_string());
        assert!(matches!(
            registry.classify(&with_mpn, 2),
            Ok(Classification::Pwl { .. })
        ));
        // Without the number the value is the fallback.
        assert_eq!(
            registry.classify(&decl_with_lib("Diode", "Schottky", "SS36"), 2),
            Ok(Classification::Mechanical)
        );
        // The part name beats the number.
        registry.register_mechanical("Schottky");
        assert_eq!(
            registry.classify(&with_mpn, 2),
            Ok(Classification::Mechanical)
        );
    }

    /// One table: registering a key again replaces the earlier entry
    /// whatever its kind, so a model can take over a part that was declared
    /// a class before it existed.
    #[rstest]
    fn a_later_registration_replaces_an_earlier_one_of_any_kind() {
        let mut registry = PartRegistry::new();
        registry.register_mechanical("THING");
        assert_eq!(
            registry.classify(&decl("THING", "?"), 3),
            Ok(Classification::Mechanical)
        );
        registry.register("THING", |_| Box::new(NullComponent));
        assert_eq!(
            registry.classify(&decl("THING", "?"), 3),
            Ok(Classification::Registered)
        );
    }

    // --------------------------------------------------------
    // Declared boundaries
    // --------------------------------------------------------

    /// A project-library connector symbol matches no `Conn*` prefix; declaring
    /// the part name puts it in the auto boundary tier instead of forcing a
    /// per-board stub that would claim "electrically absent".
    #[rstest]
    fn declared_boundary_parts_classify_as_boundaries() {
        let mut registry = PartRegistry::new();
        // Undeclared: the 80-finger edge socket is a hard error.
        assert!(matches!(
            registry.classify(
                &decl_with_lib(
                    "P2_EDGE_MODULE_SOCKET",
                    "P2_EDGE_MODULE_SOCKET",
                    "P2_EDGE_MODULE_SOCKET"
                ),
                80
            ),
            Err(RegistryError::UnknownPart { .. })
        ));

        registry.register_boundary("P2_EDGE_MODULE_SOCKET");
        assert!(registry.is_boundary("P2_EDGE_MODULE_SOCKET"));
        assert_eq!(
            registry.classify(
                &decl_with_lib(
                    "P2_EDGE_MODULE_SOCKET",
                    "P2_EDGE_MODULE_SOCKET",
                    "P2_EDGE_MODULE_SOCKET"
                ),
                80
            ),
            Ok(Classification::Boundary)
        );
    }

    /// A boundary declaration is the more specific statement about the board,
    /// so it wins over a registry constructor for the same part name.
    #[rstest]
    fn boundary_declaration_wins_over_a_registry_entry() {
        let mut registry = PartRegistry::new();
        registry.register("MYSTERY_SOCKET", |_| Box::new(NullComponent));
        assert_eq!(
            registry.classify(&decl_with_lib("Lib", "MYSTERY_SOCKET", "X1"), 4),
            Ok(Classification::Registered)
        );
        registry.register_boundary("MYSTERY_SOCKET");
        assert_eq!(
            registry.classify(&decl_with_lib("Lib", "MYSTERY_SOCKET", "X1"), 4),
            Ok(Classification::Boundary)
        );
    }

    // --------------------------------------------------------
    // Reference-designator fallback (no-libsource netlists)
    // --------------------------------------------------------

    #[rstest]
    #[case::resistor("R100", Some("R"))]
    #[case::resistor_bare("R", Some("R"))]
    #[case::capacitor("C516", Some("C"))]
    #[case::inductor("L401", Some("L"))]
    #[case::diode("D601", Some("D"))]
    #[case::connector("J203", Some("Conn"))]
    #[case::plug("P1", Some("Conn"))]
    #[case::test_point("TP4", Some("TestPoint"))]
    #[case::mounting_hole("H5", Some("MountingHole"))]
    #[case::mounting_hole_alt("MK1", Some("MountingHole"))]
    #[case::lowercase("r7", Some("R"))]
    // The whole alphabetic run must match: a resistor NETWORK is not a
    // resistor, and these three must not be mistaken for `C`/`P`/`R`.
    #[case::resistor_network("RN7", None)]
    #[case::pcb("PCB", None)]
    #[case::layout_node("NC_Net", None)]
    // Active silicon always falls through to the consumer's registry.
    #[case::ic("U100", None)]
    #[case::ic_alt("IC14", None)]
    #[case::transistor("Q1", None)]
    #[case::crystal("X100", None)]
    #[case::switch("S301", None)]
    #[case::empty("", None)]
    fn reference_designator_classes_match_the_whole_alphabetic_run(
        #[case] reference: &str,
        #[case] expected: Option<&str>,
    ) {
        assert_eq!(reference_designator_class(reference), expected);
    }

    /// One `ComponentDecl` shaped like the `p2_ec32mb.net` fixture's: a value
    /// and a reference, and **no libsource at all**.
    fn unnamed(reference: &str, value: &str) -> ComponentDecl {
        ComponentDecl {
            reference: reference.to_string(),
            value: value.to_string(),
            footprint: String::new(),
            lib: String::new(),
            part: String::new(),
            sheetpath: "/".to_string(),
            dnp: false,
            mpn: None,
        }
    }

    /// With the fallback off (the default) a no-libsource board is entirely
    /// unclassified — the state that makes the opt-in worth having.
    #[rstest]
    fn without_the_fallback_unnamed_parts_are_hard_errors() {
        let registry = PartRegistry::new();
        assert_eq!(
            registry.classify(&unnamed("R100", "10.5K"), 2),
            Err(RegistryError::UnknownPart {
                reference: "R100".to_string(),
                part: String::new(),
                value: "10.5K".to_string(),
                mpn: None,
            })
        );
        assert!(matches!(
            registry.classify(&unnamed("J203", "Edge Socket Pads"), 60),
            Err(RegistryError::UnknownPart { .. })
        ));
    }

    /// With the fallback on, the real fixture's passives and pad/socket
    /// symbols classify — values parsed — while its active silicon still has
    /// to be registered.
    #[rstest]
    fn reference_fallback_classifies_the_ec32mb_fixture_shapes() {
        let mut registry = PartRegistry::new();
        registry.classify_unnamed_by_reference(true);
        registry.register("P2X8C4M64P", |_| Box::new(NullComponent));

        // R100: the 10.5 kΩ reset pull-up — parsed, so it conducts.
        assert_eq!(
            registry.classify(&unnamed("R100", "10.5K"), 2),
            Ok(Classification::Passive {
                kind: PassiveKind::Resistor,
                value: Some(10_500.0)
            })
        );
        // C100: a decoupling cap whose value field carries a voltage rating —
        // classifies, value parsed from the first token (compared with a
        // tolerance: multiplier arithmetic carries ULP-level error).
        let parsed_passive = |reference: &str, value: &str| -> (PassiveKind, f64) {
            match registry.classify(&unnamed(reference, value), 2) {
                Ok(Classification::Passive {
                    kind,
                    value: Some(v),
                }) => (kind, v),
                other => panic!("{reference} {value:?} classified as {other:?}"),
            }
        };
        let (kind, farads) = parsed_passive("C100", "4.7uF 6.3V");
        assert_eq!(kind, PassiveKind::Capacitor);
        assert!(((farads - 4.7e-6) / 4.7e-6).abs() < 1e-12, "{farads}");
        let (kind, henries) = parsed_passive("L401", "3.3uH 6.6A 28.6mOhm");
        assert_eq!(kind, PassiveKind::Inductor);
        assert!(((henries - 3.3e-6) / 3.3e-6).abs() < 1e-12, "{henries}");
        // D601 is a white LED; the `D` prefix says diode and nothing of its
        // knee, so without the library entry that keys its number it is an
        // unknown part like any active silicon.
        assert_eq!(
            registry.classify(&unnamed("D601", "White"), 2),
            Err(RegistryError::UnknownPart {
                reference: "D601".to_string(),
                part: String::new(),
                value: "White".to_string(),
                mpn: None,
            })
        );
        // The 80-finger edge socket is a boundary.
        assert_eq!(
            registry.classify(&unnamed("J203", "Edge Socket Pads"), 60),
            Ok(Classification::Boundary)
        );
        // A mounting hole drawn as a `J` pad symbol is a boundary until its
        // value is declared mechanical; then the declaration wins over the
        // synthesized class.
        assert_eq!(
            registry.classify(&unnamed("J701", "Mounting Hole Vss"), 1),
            Ok(Classification::Boundary)
        );
        registry.register_mechanical("Mounting Hole Vss");
        assert_eq!(
            registry.classify(&unnamed("J701", "Mounting Hole Vss"), 1),
            Ok(Classification::Mechanical)
        );
        // Registered active silicon keys on the VALUE (there is no part name).
        assert_eq!(
            registry.classify(&unnamed("U100", "P2X8C4M64P"), 86),
            Ok(Classification::Registered)
        );
        // Unregistered active silicon still fails loudly, naming the value —
        // the only name it has.
        assert_eq!(
            registry.classify(&unnamed("U301", "SPI Flash 16MB (128Mb)"), 8),
            Err(RegistryError::UnknownPart {
                reference: "U301".to_string(),
                part: String::new(),
                value: "SPI Flash 16MB (128Mb)".to_string(),
                mpn: None,
            })
        );
        // A BOM-only refdes has no class until its value is declared
        // mechanical — the raw board is a node with no pads.
        assert!(matches!(
            registry.classify(&unnamed("PCB", "PCB for P2 EC Module"), 0),
            Err(RegistryError::UnknownPart { .. })
        ));
        registry.register_mechanical("PCB for P2 EC Module");
        assert_eq!(
            registry.classify(&unnamed("PCB", "PCB for P2 EC Module"), 0),
            Ok(Classification::Mechanical)
        );
        // A switch registered by value on a reference the fallback does not
        // classify.
        registry.register_switch("DIP Switch 4 way", vec![SwitchPole::open("1_ON", "1_OFF")]);
        assert_eq!(
            registry.classify(&unnamed("S301", "DIP Switch 4 way"), 2),
            Ok(Classification::Switch {
                poles: vec![SwitchPole::open("1_ON", "1_OFF")]
            })
        );
        // A solder link is a `J` — a boundary by prefix — until its value is
        // declared the one-pole switch it is.
        registry.register_switch("Solder Link Pads", vec![SwitchPole::open("1", "2")]);
        assert_eq!(
            registry.classify(&unnamed("J101", "Solder Link Pads"), 2),
            Ok(Classification::Switch {
                poles: vec![SwitchPole::open("1", "2")]
            })
        );
    }

    /// The fallback fires only for an EMPTY part name: a component that names
    /// its symbol keeps classifying on the symbol, even when its reference
    /// prefix says something else.
    #[rstest]
    fn reference_fallback_never_overrides_a_named_part() {
        let mut registry = PartRegistry::new();
        registry.classify_unnamed_by_reference(true);
        // `J1` with an ADS122U04 symbol is a chip, not a connector.
        registry.register("ADS122U04", |_| Box::new(NullComponent));
        assert_eq!(
            registry.classify(&decl_with_lib("DS2", "ADS122U04", "ADS122U04"), 16),
            Ok(Classification::Registered)
        );
        // And `U1` with a resistor symbol is still a resistor.
        assert_eq!(
            registry.classify(&decl_with_lib("Device", "R_Small", "47R"), 2),
            Ok(Classification::Passive {
                kind: PassiveKind::Resistor,
                value: Some(47.0)
            })
        );
    }

    /// Pin-count validation still applies to a reference-classified passive,
    /// and the error reports the real (empty) part name rather than the
    /// synthetic class.
    #[rstest]
    fn reference_classified_passives_are_pin_count_validated() {
        let mut registry = PartRegistry::new();
        registry.classify_unnamed_by_reference(true);
        assert_eq!(
            registry.classify(&unnamed("R9", "10k"), 3),
            Err(RegistryError::BadPinCount {
                reference: "R9".to_string(),
                part: String::new(),
                expected: 2,
                found: 3
            })
        );
    }

    #[rstest]
    fn construct_keys_on_part_name_with_value_fallback() {
        let mut registry = PartRegistry::new();
        registry.register("ADS122U04", |_decl| Box::new(NullComponent));

        assert!(registry.has_part("ADS122U04"));
        assert!(registry.construct(&decl("ADS122U04", "whatever")).is_some());
        assert!(registry
            .construct(&decl("SomeSymbol", "ADS122U04"))
            .is_some());
        assert!(registry.construct(&decl("Unknown", "Unknown")).is_none());
    }
}
