//! PartRegistry: component identity → constructor; auto-classification tiers.
//!
//! Classification is three-tier and keyed primarily on the libsource **part**
//! name (the lib name is best-effort only; rescue mangling like
//! `DS2_Addon-rescue::Jumper_NO_Small-Device` is normalized before matching):
//!
//! 1. **auto** — passive primitives (`R*`/`C*`/`L*`/`LED`/`D_*`, pin-count
//!    validated), connectors/screw terminals (board boundary pins), jumpers
//!    (stateful shorts), and ignored mechanicals (mounting holes, logos,
//!    test points).
//! 2. **registry** — anything else, keyed by part name, falling back to
//!    `value`; consumer-registered [`Component`] constructor, with an
//!    expansion hook (one netlist component → N primitives, for
//!    arrays/multi-unit symbols).
//! 3. **error** — no registry match: system construction fails; a per-board
//!    explicit stub list is the only escape.
//!
//! Slice status: types and API surface are final; the classification and
//! value-parsing bodies are the classification slice.

use std::collections::HashMap;
use std::fmt;

use crate::component::Component;
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

/// State of a jumper's stateful short (scenario-overridable).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum JumperState {
    /// Terminals disconnected (`_NO`/`_Open` default).
    Open,
    /// Terminals shorted (`_NC`/`_Bridged` default).
    Closed,
}

/// Result of classifying one netlist component.
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
    /// Auto tier: a jumper (stateful short).
    Jumper {
        /// Default state from the part name.
        default: JumperState,
        /// True for 3-pin `Jumper_3_*` variants with a selectable position.
        selectable: bool,
    },
    /// Auto tier: mounting holes / logos / test points — absent from the
    /// built board.
    Ignored,
    /// Registry tier: a consumer-registered [`Component`] constructor exists
    /// for this part.
    Registered,
    /// A component the consumer explicitly stubbed out for this board (the
    /// only escape from the error tier).
    Stubbed,
}

// ============================================================
// Registry
// ============================================================

/// Constructor for a consumer-registered component.
pub type ComponentCtor = Box<dyn Fn(&ComponentDecl) -> Box<dyn Component> + Send + Sync>;

/// Expansion hook: one netlist component → N primitive declarations
/// (resistor arrays, multi-unit symbols).
pub type ExpansionHook = Box<dyn Fn(&ComponentDecl) -> Vec<ComponentDecl> + Send + Sync>;

/// Consumer part registry: maps part identity to component constructors and
/// expansion hooks. Lookup keys on the rescue-normalized part name, falling
/// back to the component `value`.
#[derive(Default)]
pub struct PartRegistry {
    constructors: HashMap<String, ComponentCtor>,
    expansions: HashMap<String, ExpansionHook>,
}

impl fmt::Debug for PartRegistry {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("PartRegistry")
            .field(
                "constructors",
                &self.constructors.keys().collect::<Vec<_>>(),
            )
            .field("expansions", &self.expansions.keys().collect::<Vec<_>>())
            .finish()
    }
}

impl PartRegistry {
    /// Empty registry.
    pub fn new() -> Self {
        Self::default()
    }

    /// Register a component constructor for a part name.
    pub fn register(
        &mut self,
        part: impl Into<String>,
        ctor: impl Fn(&ComponentDecl) -> Box<dyn Component> + Send + Sync + 'static,
    ) {
        self.constructors.insert(part.into(), Box::new(ctor));
    }

    /// Register an expansion hook for a part name (one netlist component →
    /// N primitives).
    pub fn register_expansion(
        &mut self,
        part: impl Into<String>,
        hook: impl Fn(&ComponentDecl) -> Vec<ComponentDecl> + Send + Sync + 'static,
    ) {
        self.expansions.insert(part.into(), Box::new(hook));
    }

    /// True when a constructor is registered for the (already normalized)
    /// part name.
    pub fn has_part(&self, part: &str) -> bool {
        self.constructors.contains_key(part)
    }

    /// Construct the registered component for a declaration, keyed by
    /// normalized part name with fallback to `value`. `None` when no
    /// registry entry matches (the error tier).
    pub fn construct(&self, decl: &ComponentDecl) -> Option<Box<dyn Component>> {
        let part = normalize_part(decl);
        self.constructors
            .get(part.as_str())
            .or_else(|| self.constructors.get(decl.value.as_str()))
            .map(|ctor| ctor(decl))
    }

    /// Classify one netlist component through the three tiers. `pin_count`
    /// is the component's node count from the netlist — a 2-terminal passive
    /// class with a different pin count is a hard classification error
    /// (resistor arrays etc. need a registry expansion entry).
    pub fn classify(
        &self,
        decl: &ComponentDecl,
        pin_count: usize,
    ) -> Result<Classification, RegistryError> {
        let part = normalize_part(decl);

        // Tier 1a: ignored mechanicals — absent from the built board.
        if starts_with_any(&part, &["MountingHole", "Logo", "TestPoint", "Fiducial"]) {
            return Ok(Classification::Ignored);
        }

        // Tier 1b: connectors / screw terminals — board boundary pins.
        if starts_with_any(&part, &["Conn", "Screw_Terminal"]) {
            return Ok(Classification::Boundary);
        }

        // Tier 1c: jumpers — stateful shorts, default state from the name.
        if part.starts_with("Jumper") || part.starts_with("SolderJumper") {
            let default = if part.contains("_NC") || part.contains("_Bridged") {
                JumperState::Closed
            } else {
                // `_NO` / `_Open` / unmarked jumpers default open.
                JumperState::Open
            };
            let selectable = part.starts_with("Jumper_3") || part.starts_with("SolderJumper_3");
            return Ok(Classification::Jumper {
                default,
                selectable,
            });
        }

        // Tier 1d: passive primitives — the part-name class is anchored so
        // e.g. "RJ45" never classifies as a resistor.
        if let Some(kind) = passive_kind(&part) {
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

        // Tier 2: consumer registry, keyed on normalized part name with a
        // fallback to the value field.
        if self.constructors.contains_key(part.as_str())
            || self.constructors.contains_key(decl.value.as_str())
        {
            return Ok(Classification::Registered);
        }

        // Tier 3: hard error — the board's explicit stub list is the only
        // escape (applied by the board build, not here).
        Err(RegistryError::UnknownPart {
            reference: decl.reference.clone(),
            part,
        })
    }
}

/// True when `part` starts with any of the given prefixes.
fn starts_with_any(part: &str, prefixes: &[&str]) -> bool {
    prefixes.iter().any(|p| part.starts_with(p))
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
/// Grammar (case significant for multipliers): digits with either a decimal
/// point or an embedded multiplier letter acting as one (`4k7` = 4.7 k,
/// `2R2` = 2.2), an optional multiplier (`p n u µ m k K M G`, plus `R` = ×1
/// for resistors), and an optional unit letter (`R Ω F H`) which is ignored.
/// Returns `None` for non-numeric fields (`"X"`, `"ADS122U04"`).
pub fn parse_passive_value(value: &str) -> Option<f64> {
    let v = value.trim().trim_end_matches('Ω');
    // Strip a trailing unit letter (F/H, or R when it follows the number and
    // a multiplier was already seen: "4k7R" — keep simple: strip one trailing
    // F or H always; R is handled as a multiplier below).
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
    /// No auto-tier match and no registry entry — system construction fails
    /// unless the board stubs the reference explicitly.
    UnknownPart {
        /// Component reference designator.
        reference: String,
        /// Normalized part name that failed to match.
        part: String,
    },
    /// An auto-classified 2-terminal class with a different netlist pin
    /// count.
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
            RegistryError::UnknownPart { reference, part } => {
                write!(f, "{reference}: no classification or registry entry for part {part:?}")
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
                selectable: false
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
        // Unknown part with no registry entry -> hard error naming the part.
        assert_eq!(
            registry.classify(&decl_with_lib("Weird", "FrobulatorX", "?"), 4),
            Err(RegistryError::UnknownPart {
                reference: "U1".to_string(),
                part: "FrobulatorX".to_string()
            })
        );
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
        // EdgeBoard jumpers: Jumper_2_Open / Jumper_3_Open.
        assert_eq!(
            registry.classify(&decl_with_lib("Jumper", "Jumper_2_Open", "JP"), 2),
            Ok(Classification::Jumper {
                default: JumperState::Open,
                selectable: false
            })
        );
        assert_eq!(
            registry.classify(&decl_with_lib("Jumper", "Jumper_3_Open", "JP"), 3),
            Ok(Classification::Jumper {
                default: JumperState::Open,
                selectable: true
            })
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
