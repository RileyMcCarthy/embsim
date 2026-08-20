//! KiCad s-expression netlist parser → [`ComponentDecl`]/[`NetDecl`] graph.
//!
//! Input: a KiCad netlist export (`kicad-cli sch export netlist`). Parsing is
//! **version-gated** on `(export (version …))` — unsupported versions fail
//! with a named error, and the test suite carries one fixture per supported
//! KiCad major (`tests/fixtures/`).
//!
//! Both flat and **hierarchical** (multi-sheet) exports are supported: the
//! reference fixtures are `ds2_addon.net` (flat, one sheet) and
//! `mad_edge.net` (three sheets, 168 components / 243 nets). Hierarchical
//! designs bring two extra concerns, both handled here:
//!
//! - components carry a `(sheetpath (names "/MaD_Edge_Sheet2/"))` naming the
//!   sheet instance they were placed on ([`ComponentDecl::sheetpath`]);
//! - **local labels are sheet-scoped**, so their nets export as
//!   `/<sheet>/<label>` — see "Net names" below for the canonical spelling.
//!
//! # Net names
//!
//! The canonical net name is the **full exported name, sheet path included**.
//! [`normalize_net_name`] only canonicalizes overline syntax on the leaf; it
//! never strips the path. Rationale and the display-only helpers are on
//! [`normalize_net_name`], [`net_sheet_path`], and [`net_short_label`].
//!
//! # Deliberately ignored export forms
//!
//! These appear in every real export and carry no engine meaning:
//!
//! - `(design …)` — source path, export date, per-sheet title blocks. The
//!   absolute source path and the timestamp differ on every re-export, which
//!   is why fixture checks compare parsed *shape*, never bytes.
//! - `(libparts …)` / `(libraries …)` — per-symbol pin tables and library
//!   search paths. Pin identity and electrical descriptors come from a
//!   component's own [`crate::component::PinDecl`] facade, not from the
//!   schematic symbol, so the symbol's `(pin (num …) (name …) (type …))` rows
//!   are not authoritative here.
//! - comp `(datasheet …)`, `(description …)`, `(fields …)`, and every
//!   `(property …)` except `dnp` — BOM and documentation metadata
//!   (`LCSC`, `DIGIKEY`, `Height`, `ki_keywords`, …).
//! - `(tstamps …)`, both at comp level and inside `(sheetpath …)` — UUID
//!   instance paths. Sheet *names* are already unique per sheet instance, so
//!   the UUIDs add nothing structural.

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
    /// Hierarchical sheet-instance path the symbol was placed on (`"/"` for
    /// the root sheet, `"/MaD_Edge_Sheet2/"` for a sub-sheet). Taken from
    /// `(sheetpath (names …))`; the companion `(tstamps …)` UUID path is
    /// ignored because sheet names are already unique per instance.
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
    /// KiCad electrical pin type from the *schematic symbol* (`"passive"`,
    /// `"power_in"`, `"tri_state"`, `"open_collector"`, and the
    /// `"+no_connect"`-suffixed variants), when the export carries one.
    ///
    /// **Informational only.** The engine's electrical descriptors come from
    /// the component's own [`crate::component::PinDecl`], never from the
    /// schematic — this is kept so diagnostics can say "the schematic marks
    /// this pin no-connect" instead of reporting a mystery floating sense.
    pub pintype: Option<String>,
}

/// One `(net …)` entry from the netlist's `(nets …)` section.
#[derive(Debug, Clone, PartialEq)]
pub struct NetDecl {
    /// Netlist net code (`"6"`).
    pub code: String,
    /// Net name exactly as exported — sheet path included for sheet-scoped
    /// local labels (`"AIN0"`, `"/MaD_Edge_Sheet2/IFG_RX"`). Canonicalize
    /// with [`normalize_net_name`]; never strip the path.
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
        // ASCII-only whitespace on purpose: s-expression whitespace is ASCII,
        // and a byte-wise `as char` test would treat UTF-8 continuation bytes
        // (0xA0 in many 2- and 3-byte sequences) as NBSP whitespace and split
        // a multi-byte atom in half.
        while *pos < bytes.len() && bytes[*pos].is_ascii_whitespace() {
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

/// Consume a quoted atom.
///
/// Accumulates **bytes** and decodes once at the close quote: real exports
/// carry multi-byte UTF-8 inside quoted atoms (`"470 µF"`, `"60 mΩ@10V"`,
/// `"±20%"`), and a per-byte `as char` push would transliterate each
/// continuation byte into its own Latin-1 character (`µ` → `Âµ`) — which for a
/// value field such as `"4µ7"` would then defeat
/// [`crate::registry::parse_passive_value`]'s `µ` multiplier.
fn parse_quoted(bytes: &[u8], pos: &mut usize) -> Result<Sexp, NetlistError> {
    *pos += 1; // consume opening quote
    let mut out: Vec<u8> = Vec::new();
    while let Some(&b) = bytes.get(*pos) {
        match b {
            b'"' => {
                *pos += 1;
                return Ok(Sexp::Atom(String::from_utf8_lossy(&out).into_owned()));
            }
            b'\\' => {
                // KiCad escapes quotes and backslashes inside quoted atoms;
                // only ASCII is ever escaped, so the escapee is one byte.
                *pos += 1;
                if let Some(&esc) = bytes.get(*pos) {
                    out.push(esc);
                    *pos += 1;
                }
            }
            _ => {
                out.push(b);
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
        // ASCII-only whitespace, for the reason given in `skip_ws`.
        if b == b'(' || b == b')' || b == b'"' || b.is_ascii_whitespace() {
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
        let pintype = node.child_arg("pintype").map(str::to_string);
        nodes.push(NodeDecl {
            reference,
            pin,
            pinfunction,
            pintype,
        });
    }

    Ok(NetDecl { code, name, nodes })
}

/// Normalize KiCad overline pin-name syntax: `~{RESET}` ≡ `~RESET`.
///
/// Pin identities never contain a sheet path; for **net** names use
/// [`normalize_net_name`], which applies this to the leaf label only.
pub fn normalize_pin_name(name: &str) -> String {
    match name
        .strip_prefix("~{")
        .and_then(|rest| rest.strip_suffix('}'))
    {
        Some(inner) => format!("~{inner}"),
        None => name.to_string(),
    }
}

// ============================================================
// Net names (hierarchical sheets)
// ============================================================

/// Canonicalize an exported net name.
///
/// **The canonical name is the full exported name, sheet path included.** A
/// KiCad local label is scoped to the sheet instance it was drawn on and
/// exports as `/<sheet>/<label>` (`/MaD_Edge_Sheet2/IFG_RX`), while global
/// labels, power symbols, and root-sheet labels export bare (`GND`, `+3.3V`,
/// `P16`). This function keeps the path verbatim and only rewrites overline
/// syntax on the **leaf** label, so `/Sheet2/~{OE}` becomes `/Sheet2/~OE`
/// while a bare `~{RESET}` net normalizes exactly as
/// [`normalize_pin_name`] always did.
///
/// The path is **never** stripped. Two sheets may carry the same leaf label —
/// `/Sheet2/SIGNAL` and `/Sheet3/SIGNAL` are two electrically distinct nets —
/// so stripping would collapse them to one name and hand every name-keyed
/// lookup (scenario `net_stuck`, diagnostics, dumps) a silent cross-sheet
/// merge. That is precisely the failure mode the design's "no implicit
/// net-name merging" rule exists to prevent, and it would be invisible: the
/// nets stay separate in the graph while every human-readable artifact claims
/// they are one. [`net_short_label`] provides the shortened spelling for
/// display, where ambiguity is cosmetic rather than structural.
pub fn normalize_net_name(name: &str) -> String {
    match net_sheet_path(name) {
        Some(path) => format!(
            "{path}{leaf}",
            leaf = normalize_pin_name(&name[path.len()..])
        ),
        None => normalize_pin_name(name),
    }
}

/// The sheet-instance prefix of a sheet-scoped net name, both slashes
/// included (`"/MaD_Edge_Sheet2/"` for `"/MaD_Edge_Sheet2/IFG_RX"`), or
/// `None` for a globally-scoped name (`"GND"`).
///
/// Recognized only when the name starts with `/`, so a label that merely
/// contains a slash (`"A/B"` drawn on the root sheet) is not mistaken for a
/// path. Nested sheets keep their whole prefix (`"/Top/Inner/"`), and a name
/// that is just `"/GND"` yields the root path `"/"`.
pub fn net_sheet_path(name: &str) -> Option<&str> {
    if !name.starts_with('/') {
        return None;
    }
    name.rfind('/').map(|i| &name[..=i])
}

/// Display-only leaf label of a net name: the segment after the sheet path
/// (`"IFG_RX"` for `"/MaD_Edge_Sheet2/IFG_RX"`, `"GND"` for `"GND"`).
///
/// **Not an identity.** Leaf labels are not unique across sheets — identify a
/// net by its full name or by [`crate::net::NetId`], and use this only where
/// a shorter label is wanted for a human (trace views, dumps, log lines).
pub fn net_short_label(name: &str) -> &str {
    match net_sheet_path(name) {
        Some(path) => &name[path.len()..],
        None => name,
    }
}

/// True for KiCad's synthesized net names: `Net-(U1-TX)` for an unlabeled net
/// and `unconnected-(U9-NC-Pad1)` for a no-connect stub.
///
/// A *naming* heuristic with no electrical meaning — callers use it to keep
/// expected single-pin stubs out of the findings noise (in the reference
/// hierarchical fixture all 47 `unconnected-(…)` nets have exactly one node
/// whose [`NodeDecl::pintype`] carries `no_connect`).
pub fn is_autogenerated_net_name(name: &str) -> bool {
    let leaf = net_short_label(name);
    leaf.starts_with("Net-(") || leaf.starts_with("unconnected-(")
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

    // --------------------------------------------------------
    // Hierarchical sheets: net names
    // --------------------------------------------------------

    /// Two sheets, each with a local label spelled `SIGNAL`, plus one global
    /// net. The shape the canonical-naming decision has to survive.
    const TWO_SHEET_NETLIST: &str = r#"(export (version "E")
        (components
          (comp (ref "R1") (value "10k")
            (libsource (lib "Device") (part "R_Small"))
            (sheetpath (names "/Sheet2/") (tstamps "/1111/"))
            (tstamps "00000000-0000-0000-0000-000000000001"))
          (comp (ref "R2") (value "10k")
            (libsource (lib "Device") (part "R_Small"))
            (sheetpath (names "/Sheet3/") (tstamps "/2222/"))
            (tstamps "00000000-0000-0000-0000-000000000002")))
        (nets
          (net (code "1") (name "/Sheet2/SIGNAL")
            (node (ref "R1") (pin "1") (pinfunction "1") (pintype "passive")))
          (net (code "2") (name "/Sheet3/SIGNAL")
            (node (ref "R2") (pin "1") (pinfunction "1") (pintype "passive")))
          (net (code "3") (name "GND")
            (node (ref "R1") (pin "2") (pinfunction "2") (pintype "passive"))
            (node (ref "R2") (pin "2") (pinfunction "2") (pintype "passive")))))"#;

    #[rstest]
    #[case::sheet_scoped("/MaD_Edge_Sheet2/IFG_RX", Some("/MaD_Edge_Sheet2/"), "IFG_RX")]
    #[case::nested("/Top/Inner/CLK", Some("/Top/Inner/"), "CLK")]
    #[case::global("GND", None, "GND")]
    #[case::power("+3.3V", None, "+3.3V")]
    #[case::autogen("Net-(U1-TX)", None, "Net-(U1-TX)")]
    // A label that merely contains a slash is not a path (no leading `/`).
    #[case::slash_in_label("A/B", None, "A/B")]
    // Degenerate: a bare leading slash is the root sheet.
    #[case::root_only("/GND", Some("/"), "GND")]
    #[case::empty("", None, "")]
    fn sheet_paths_and_short_labels_split_on_the_leading_slash(
        #[case] name: &str,
        #[case] path: Option<&str>,
        #[case] leaf: &str,
    ) {
        assert_eq!(net_sheet_path(name), path);
        assert_eq!(net_short_label(name), leaf);
    }

    /// Canonicalization keeps the sheet path verbatim and rewrites overline
    /// syntax on the leaf only.
    #[rstest]
    #[case::bare_overline("~{RESET}", "~RESET")]
    #[case::scoped_overline("/Sheet2/~{OE}", "/Sheet2/~OE")]
    #[case::scoped_plain("/MaD_Edge_Sheet2/IFG_RX", "/MaD_Edge_Sheet2/IFG_RX")]
    #[case::already_normal("/Sheet2/~OE", "/Sheet2/~OE")]
    // Overline inside an autogenerated name is not a prefix — left alone.
    #[case::autogen_overline("Net-(U1-~{DRDY})", "Net-(U1-~{DRDY})")]
    #[case::global("GND", "GND")]
    fn normalize_net_name_never_strips_the_sheet_path(#[case] name: &str, #[case] expected: &str) {
        assert_eq!(normalize_net_name(name), expected);
    }

    /// The reason the path is never stripped: same-leaf labels on two sheets
    /// are two electrically distinct nets, and the leaf alone cannot tell
    /// them apart.
    #[rstest]
    fn same_leaf_label_on_two_sheets_stays_two_distinct_nets() {
        let parsed = parse(TWO_SHEET_NETLIST).expect("two-sheet netlist parses");
        assert_eq!(parsed.nets.len(), 3);

        let scoped: Vec<&NetDecl> = parsed
            .nets
            .iter()
            .filter(|n| net_sheet_path(&n.name).is_some())
            .collect();
        assert_eq!(scoped.len(), 2);
        assert_eq!(scoped[0].name, "/Sheet2/SIGNAL");
        assert_eq!(scoped[1].name, "/Sheet3/SIGNAL");
        assert_ne!(scoped[0].code, scoped[1].code);
        assert_ne!(
            normalize_net_name(&scoped[0].name),
            normalize_net_name(&scoped[1].name),
            "canonical names must stay distinct"
        );

        // Membership proves they are separate nets, not one net named twice.
        assert_eq!(scoped[0].nodes[0].reference, "R1");
        assert_eq!(scoped[1].nodes[0].reference, "R2");

        // And the counterfactual: stripping to leaf labels collapses three
        // names into two — the silent cross-sheet merge we refuse.
        let canonical: std::collections::HashSet<String> =
            parsed.nets.iter().map(|n| n.name.clone()).collect();
        let stripped: std::collections::HashSet<&str> = parsed
            .nets
            .iter()
            .map(|n| net_short_label(&n.name))
            .collect();
        assert_eq!(canonical.len(), 3);
        assert_eq!(stripped.len(), 2);
    }

    #[rstest]
    #[case::unlabeled("Net-(U1-TX)", true)]
    #[case::no_connect("unconnected-(U9-NC-Pad1)", true)]
    #[case::scoped_unlabeled("/Sheet2/Net-(U1-TX)", true)]
    #[case::named("GND", false)]
    #[case::named_lookalike("Net_Control", false)]
    #[case::scoped_named("/MaD_Edge_Sheet2/IFG_RX", false)]
    fn autogenerated_net_names_are_recognized(#[case] name: &str, #[case] expected: bool) {
        assert_eq!(is_autogenerated_net_name(name), expected);
    }

    // --------------------------------------------------------
    // Hierarchical sheets: component + node fields
    // --------------------------------------------------------

    /// `(sheetpath (names …))` is captured; the companion `(tstamps …)` and
    /// the comp-level `(tstamps …)` are ignored by design.
    #[rstest]
    fn component_sheetpaths_are_captured() {
        let parsed = parse(TWO_SHEET_NETLIST).unwrap();
        assert_eq!(parsed.components[0].sheetpath, "/Sheet2/");
        assert_eq!(parsed.components[1].sheetpath, "/Sheet3/");
    }

    /// A comp without `(sheetpath …)` defaults to the root sheet.
    #[rstest]
    fn missing_sheetpath_defaults_to_root() {
        let input = r#"(export (version "E")
            (components (comp (ref "R1") (value "10k")))
            (nets))"#;
        assert_eq!(parse(input).unwrap().components[0].sheetpath, "/");
    }

    /// `(pintype …)` is captured when present and stays `None` otherwise —
    /// informational, so its absence is never an error.
    #[rstest]
    fn node_pintype_is_captured_when_present() {
        let parsed = parse(TWO_SHEET_NETLIST).unwrap();
        assert_eq!(parsed.nets[0].nodes[0].pintype.as_deref(), Some("passive"));

        let bare = r#"(export (version "E") (components)
            (nets (net (code "1") (name "N") (node (ref "U1") (pin "1")))))"#;
        let node = &parse(bare).unwrap().nets[0].nodes[0];
        assert_eq!(node.pintype, None);
        assert_eq!(node.pinfunction, None);
    }

    /// Real exports carry multi-byte UTF-8 inside quoted atoms (`470 µF`,
    /// `60 mΩ@10V`, `±20%`). Bytes must survive intact — a per-byte push
    /// would turn `µ` into `Âµ` and defeat the `µ` value multiplier.
    #[rstest]
    fn quoted_atoms_preserve_multibyte_utf8() {
        let input = r#"(export (version "E")
            (components
              (comp (ref "C1") (value "4µ7")
                (footprint "R_0805")
                (libsource (lib "Device") (part "C_Small") (description "470 µF ±20% 60 mΩ"))))
            (nets))"#;
        let comp = &parse(input).unwrap().components[0];
        assert_eq!(comp.value, "4µ7");
        assert_eq!(
            crate::registry::parse_passive_value(&comp.value),
            Some(4.7e-6)
        );
    }

    /// An empty libsource `lib` is normal in real exports (project-local /
    /// cached symbols) — classification keys on `part`, so it must parse
    /// without complaint.
    #[rstest]
    fn empty_libsource_lib_is_accepted() {
        let input = r#"(export (version "E")
            (components
              (comp (ref "IC6") (value "NSI50010YT1G")
                (libsource (lib "") (part "NSI50010YT1G_1") (description ""))))
            (nets))"#;
        let comp = &parse(input).unwrap().components[0];
        assert_eq!(comp.lib, "");
        assert_eq!(comp.part, "NSI50010YT1G_1");
    }
}
