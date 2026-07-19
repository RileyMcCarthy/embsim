//! KiCad s-expression netlist parser → [`ComponentDecl`]/[`NetDecl`] graph.
//!
//! Input: a KiCad netlist export (`kicad-cli sch export netlist`). Parsing is
//! **version-gated** on `(export (version …))` — unsupported versions fail
//! with a named error, and the test suite carries one fixture per supported
//! KiCad major (`tests/fixtures/`).
//!
//! Slice status: the shared declaration types are final; [`parse`] and the
//! hand-written s-expression tokenizer (no external deps) are the follow-up
//! parser slice.

use std::fmt;

// ============================================================
// Parsed declarations
// ============================================================

/// One `(comp …)` entry from the netlist's `(components …)` section.
#[derive(Debug, Clone, PartialEq)]
pub struct ComponentDecl {
    /// Reference designator (`"U1"`).
    pub reference: String,
    /// Value field (`"47R"`, `"X"` = DNP by consumer convention).
    pub value: String,
    /// Footprint (`"Resistor_SMD:R_0805_2012Metric"`).
    pub footprint: String,
    /// Libsource lib name — best-effort only (real exports contain empty lib
    /// names and KiCad `*-rescue` libs); classification keys on `part`.
    pub lib: String,
    /// Libsource part name (`"C_Small"`), pre-normalization.
    pub part: String,
    /// Hierarchical sheet path (`"/"` for flat designs).
    pub sheetpath: String,
    /// True when the KiCad `dnp` property is set (the `value == "X"` consumer
    /// convention is applied at classification time, not here).
    pub dnp: bool,
}

/// One `(node …)` membership entry of a net.
#[derive(Debug, Clone, PartialEq)]
pub struct NodeDecl {
    /// Component reference designator (`"U1"`).
    pub reference: String,
    /// Pin number (`"3"`).
    pub pin: String,
    /// KiCad pinfunction alias (`"AIN0"`), when the symbol names the pin.
    pub pinfunction: Option<String>,
}

/// One `(net …)` entry from the netlist's `(nets …)` section.
#[derive(Debug, Clone, PartialEq)]
pub struct NetDecl {
    /// Netlist net code (`"6"`).
    pub code: String,
    /// Net name (`"AIN0"`).
    pub name: String,
    /// Member pins.
    pub nodes: Vec<NodeDecl>,
}

/// A fully parsed netlist: the input to [`crate::board::Board::from_netlist`].
#[derive(Debug, Clone, PartialEq)]
pub struct ParsedNetlist {
    /// The `(export (version …))` string (`"E"` for KiCad 9 exports).
    pub version: String,
    /// All component declarations.
    pub components: Vec<ComponentDecl>,
    /// All net declarations.
    pub nets: Vec<NetDecl>,
}

// ============================================================
// Errors
// ============================================================

/// Netlist parse failure.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum NetlistError {
    /// The `(export (version …))` value is not a supported KiCad export
    /// version.
    UnsupportedVersion {
        /// The version string found in the export.
        found: String,
    },
    /// The input is not a well-formed KiCad s-expression netlist.
    Malformed {
        /// Human-readable cause.
        message: String,
    },
}

impl fmt::Display for NetlistError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            NetlistError::UnsupportedVersion { found } => {
                write!(f, "unsupported KiCad netlist export version {found:?}")
            }
            NetlistError::Malformed { message } => {
                write!(f, "malformed KiCad netlist: {message}")
            }
        }
    }
}

impl std::error::Error for NetlistError {}

// ============================================================
// Parsing
// ============================================================

/// KiCad export versions this parser is tested against (one fixture per
/// supported version in `tests/fixtures/`).
pub const SUPPORTED_VERSIONS: &[&str] = &["E"];

// ------------------------------------------------------------
// S-expression layer (hand-written, no external deps)
// ------------------------------------------------------------

/// A parsed s-expression node.
#[derive(Debug, Clone, PartialEq)]
enum Sexp {
    /// Bare or quoted atom.
    Atom(String),
    /// Parenthesized list.
    List(Vec<Sexp>),
}

impl Sexp {
    /// The head atom of a list (`(comp …)` → `"comp"`).
    fn head(&self) -> Option<&str> {
        match self {
            Sexp::List(items) => match items.first() {
                Some(Sexp::Atom(a)) => Some(a.as_str()),
                _ => None,
            },
            Sexp::Atom(_) => None,
        }
    }

    /// Child lists whose head matches `name`.
    fn children(&self, name: &str) -> Vec<&Sexp> {
        let items: &[Sexp] = match self {
            Sexp::List(items) => &items[1..],
            Sexp::Atom(_) => &[],
        };
        items.iter().filter(|c| c.head() == Some(name)).collect()
    }

    /// First child list whose head matches `name`.
    fn child(&self, name: &str) -> Option<&Sexp> {
        let items: &[Sexp] = match self {
            Sexp::List(items) => &items[1..],
            Sexp::Atom(_) => &[],
        };
        items.iter().find(|c| c.head() == Some(name))
    }

    /// The single atom argument of a `(name "value")` form.
    fn arg(&self) -> Option<&str> {
        match self {
            Sexp::List(items) => match items.get(1) {
                Some(Sexp::Atom(a)) => Some(a.as_str()),
                _ => None,
            },
            Sexp::Atom(_) => None,
        }
    }

    /// Convenience: the atom argument of the first child named `name`.
    fn child_arg(&self, name: &str) -> Option<&str> {
        self.child(name).and_then(Sexp::arg)
    }
}

/// Tokenize + parse one top-level s-expression from the input.
fn parse_sexp(input: &str) -> Result<Sexp, NetlistError> {
    let bytes = input.as_bytes();
    let mut pos = 0usize;
    let root = parse_list(bytes, &mut pos)?;
    Ok(root)
}

/// Skip whitespace and `;`-to-end-of-line comments. KiCad itself never emits
/// comments, but committed netlist artifacts carry provenance headers (which
/// tool exported them, the regeneration policy), and `;` line comments are
/// the standard s-expression form. Quoted atoms are unaffected —
/// `parse_quoted` consumes its bytes directly.
fn skip_ws(bytes: &[u8], pos: &mut usize) {
    loop {
        while *pos < bytes.len() && (bytes[*pos] as char).is_whitespace() {
            *pos += 1;
        }
        if bytes.get(*pos) == Some(&b';') {
            while *pos < bytes.len() && bytes[*pos] != b'\n' {
                *pos += 1;
            }
        } else {
            return;
        }
    }
}

fn parse_list(bytes: &[u8], pos: &mut usize) -> Result<Sexp, NetlistError> {
    skip_ws(bytes, pos);
    if *pos >= bytes.len() || bytes[*pos] != b'(' {
        return Err(NetlistError::Malformed {
            message: format!("expected '(' at byte {pos}", pos = *pos),
        });
    }
    *pos += 1; // consume '('
    let mut items = Vec::new();
    loop {
        skip_ws(bytes, pos);
        match bytes.get(*pos) {
            None => {
                return Err(NetlistError::Malformed {
                    message: "unexpected end of input inside list".to_string(),
                })
            }
            Some(b')') => {
                *pos += 1;
                return Ok(Sexp::List(items));
            }
            Some(b'(') => items.push(parse_list(bytes, pos)?),
            Some(b'"') => items.push(parse_quoted(bytes, pos)?),
            Some(_) => items.push(parse_bare(bytes, pos)),
        }
    }
}

fn parse_quoted(bytes: &[u8], pos: &mut usize) -> Result<Sexp, NetlistError> {
    *pos += 1; // consume opening quote
    let mut out = String::new();
    while let Some(&b) = bytes.get(*pos) {
        match b {
            b'"' => {
                *pos += 1;
                return Ok(Sexp::Atom(out));
            }
            b'\\' => {
                // KiCad escapes quotes and backslashes inside quoted atoms.
                *pos += 1;
                if let Some(&esc) = bytes.get(*pos) {
                    out.push(esc as char);
                    *pos += 1;
                }
            }
            _ => {
                out.push(b as char);
                *pos += 1;
            }
        }
    }
    Err(NetlistError::Malformed {
        message: "unterminated quoted string".to_string(),
    })
}

fn parse_bare(bytes: &[u8], pos: &mut usize) -> Sexp {
    let start = *pos;
    while let Some(&b) = bytes.get(*pos) {
        if b == b'(' || b == b')' || b == b'"' || (b as char).is_whitespace() {
            break;
        }
        *pos += 1;
    }
    Sexp::Atom(String::from_utf8_lossy(&bytes[start..*pos]).into_owned())
}

// ------------------------------------------------------------
// Netlist extraction
// ------------------------------------------------------------

/// Parse a KiCad s-expression netlist export into the declaration graph.
///
/// Version-gated on `(export (version …))` — [`SUPPORTED_VERSIONS`] only;
/// anything else is [`NetlistError::UnsupportedVersion`].
pub fn parse(input: &str) -> Result<ParsedNetlist, NetlistError> {
    let root = parse_sexp(input)?;
    if root.head() != Some("export") {
        return Err(NetlistError::Malformed {
            message: "top-level form is not (export …)".to_string(),
        });
    }

    let version = root
        .child_arg("version")
        .ok_or_else(|| NetlistError::Malformed {
            message: "missing (version …)".to_string(),
        })?
        .to_string();
    if !SUPPORTED_VERSIONS.contains(&version.as_str()) {
        return Err(NetlistError::UnsupportedVersion { found: version });
    }

    let mut components = Vec::new();
    if let Some(comps) = root.child("components") {
        for comp in comps.children("comp") {
            components.push(parse_component(comp)?);
        }
    }

    let mut nets = Vec::new();
    if let Some(net_list) = root.child("nets") {
        for net in net_list.children("net") {
            nets.push(parse_net(net)?);
        }
    }

    Ok(ParsedNetlist {
        version,
        components,
        nets,
    })
}

fn parse_component(comp: &Sexp) -> Result<ComponentDecl, NetlistError> {
    let reference = comp
        .child_arg("ref")
        .ok_or_else(|| NetlistError::Malformed {
            message: "(comp …) missing (ref …)".to_string(),
        })?
        .to_string();
    let value = comp.child_arg("value").unwrap_or_default().to_string();
    let footprint = comp.child_arg("footprint").unwrap_or_default().to_string();

    let (lib, part) = match comp.child("libsource") {
        Some(libsource) => (
            libsource.child_arg("lib").unwrap_or_default().to_string(),
            libsource.child_arg("part").unwrap_or_default().to_string(),
        ),
        None => (String::new(), String::new()),
    };

    let sheetpath = comp
        .child("sheetpath")
        .and_then(|sp| sp.child_arg("names"))
        .unwrap_or("/")
        .to_string();

    // KiCad 8+ exports DNP as a property named "dnp"; the `value == "X"`
    // consumer convention is applied at classification time, not here.
    let dnp = comp.children("property").iter().any(|p| {
        p.child_arg("name")
            .is_some_and(|n| n.eq_ignore_ascii_case("dnp"))
    });

    Ok(ComponentDecl {
        reference,
        value,
        footprint,
        lib,
        part,
        sheetpath,
        dnp,
    })
}

fn parse_net(net: &Sexp) -> Result<NetDecl, NetlistError> {
    let code = net
        .child_arg("code")
        .ok_or_else(|| NetlistError::Malformed {
            message: "(net …) missing (code …)".to_string(),
        })?
        .to_string();
    let name = net.child_arg("name").unwrap_or_default().to_string();

    let mut nodes = Vec::new();
    for node in net.children("node") {
        let reference = node
            .child_arg("ref")
            .ok_or_else(|| NetlistError::Malformed {
                message: format!("(node …) in net {name:?} missing (ref …)"),
            })?
            .to_string();
        let pin = node
            .child_arg("pin")
            .ok_or_else(|| NetlistError::Malformed {
                message: format!("(node …) in net {name:?} missing (pin …)"),
            })?
            .to_string();
        let pinfunction = node.child_arg("pinfunction").map(str::to_string);
        nodes.push(NodeDecl {
            reference,
            pin,
            pinfunction,
        });
    }

    Ok(NetDecl { code, name, nodes })
}

/// Normalize KiCad overline pin-name syntax: `~{RESET}` ≡ `~RESET`.
pub fn normalize_pin_name(name: &str) -> String {
    match name
        .strip_prefix("~{")
        .and_then(|rest| rest.strip_suffix('}'))
    {
        Some(inner) => format!("~{inner}"),
        None => name.to_string(),
    }
}

#[cfg(test)]
mod tests {
    use rstest::rstest;

    use super::*;

    #[rstest]
    fn overline_syntax_normalizes() {
        assert_eq!(normalize_pin_name("~{RESET}"), "~RESET");
        assert_eq!(normalize_pin_name("~RESET"), "~RESET");
        assert_eq!(normalize_pin_name("AIN0"), "AIN0");
    }

    /// `;` line comments — the provenance headers committed netlist
    /// artifacts carry — are skipped anywhere whitespace is legal, and a
    /// `;` inside a quoted atom stays data.
    #[rstest]
    fn line_comments_are_skipped() {
        let input = "; provenance: exported by kicad-cli\n\
                     ; regeneration: CI diff-check\n\
                     (export (version \"E\")\n\
                       ; components section\n\
                       (components\n\
                         (comp (ref \"R1\") (value \"47R; not a comment\")\n\
                           (libsource (lib \"Device\") (part \"R_Small\")))))";
        let parsed = parse(input).expect("commented netlist parses");
        assert_eq!(parsed.version, "E");
        assert_eq!(parsed.components.len(), 1);
        assert_eq!(parsed.components[0].value, "47R; not a comment");
    }

    #[rstest]
    fn parses_the_real_ds2addon_fixture() {
        let input = include_str!("../tests/fixtures/ds2_addon.net");
        let parsed = parse(input).expect("fixture parses");
        assert_eq!(parsed.version, "E");
        assert_eq!(parsed.components.len(), 31);

        let u1 = parsed
            .components
            .iter()
            .find(|c| c.reference == "U1")
            .expect("U1 present");
        assert_eq!(u1.part, "ADS122U04");
        assert_eq!(u1.value, "ADS122U04");

        let r6 = parsed
            .components
            .iter()
            .find(|c| c.reference == "R6")
            .expect("R6 present");
        assert_eq!(r6.value, "X", "DNP-by-value convention survives parsing");

        let reset = parsed
            .nets
            .iter()
            .find(|n| normalize_pin_name(&n.name) == "~RESET")
            .expect("~{RESET} net present");
        assert_eq!(reset.nodes.len(), 1, "the floating-reset net has one pin");
        assert_eq!(reset.nodes[0].reference, "U1");
        assert_eq!(reset.nodes[0].pin, "3");
    }

    #[rstest]
    fn rejects_unsupported_versions_and_malformed_input() {
        let bad_version = r#"(export (version "Z") (components) (nets))"#;
        assert_eq!(
            parse(bad_version),
            Err(NetlistError::UnsupportedVersion {
                found: "Z".to_string()
            })
        );
        assert!(matches!(
            parse("(export (version \"E\") (components"),
            Err(NetlistError::Malformed { .. })
        ));
        assert!(matches!(
            parse("(design)"),
            Err(NetlistError::Malformed { .. })
        ));
    }

    #[rstest]
    fn quoted_atoms_unescape() {
        let input = r#"(export (version "E")
            (components (comp (ref "U1") (value "a \"b\" c")))
            (nets))"#;
        let parsed = parse(input).unwrap();
        assert_eq!(parsed.components[0].value, "a \"b\" c");
    }

    #[rstest]
    fn errors_display_named_causes() {
        let err = NetlistError::UnsupportedVersion {
            found: "Z".to_string(),
        };
        assert!(err.to_string().contains("\"Z\""));
    }
}
