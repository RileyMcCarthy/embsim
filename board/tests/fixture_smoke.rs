//! Sanity checks on the committed netlist fixtures.
//!
//! Cheap text-level landmarks, so a bad re-export is caught immediately even
//! before the parser runs. The parse-level truth lives with the fixture that
//! owns it: `netlist.rs`'s unit tests for `ds2_addon.net` (flat) and
//! `hierarchical_netlist.rs` for `mad_edge.net` (three sheets), which also
//! documents both fixtures' provenance and regeneration policy.

use rstest::rstest;

const DS2_ADDON: &str = include_str!("fixtures/ds2_addon.net");
const MAD_EDGE: &str = include_str!("fixtures/mad_edge.net");

#[rstest]
fn ds2_addon_fixture_is_a_kicad_sexpr_netlist_export() {
    assert!(
        DS2_ADDON.starts_with("(export (version \"E\")"),
        "fixture must be a KiCad s-expression netlist export (version E)"
    );
    assert!(DS2_ADDON.contains("(components"));
    assert!(DS2_ADDON.contains("(nets"));
    // Known landmarks of the DS2Addon board.
    assert!(DS2_ADDON.contains("(part \"ADS122U04"));
    assert!(DS2_ADDON.contains("(name \"AIN0\")"));
    assert_eq!(DS2_ADDON.matches("(comp (ref ").count(), 31);
    assert_eq!(DS2_ADDON.matches("(net (code ").count(), 25);
}

#[rstest]
fn mad_edge_fixture_is_a_hierarchical_kicad_sexpr_netlist_export() {
    // This fixture leads with a `;` provenance comment header (the parser
    // skips it); the export itself follows.
    assert!(
        MAD_EDGE.starts_with("; embsim-board netlist fixture"),
        "fixture must keep its provenance header"
    );
    assert!(MAD_EDGE.contains("\n(export (version \"E\")"));
    assert!(MAD_EDGE.contains("(components"));
    assert!(MAD_EDGE.contains("(nets"));
    // Three hierarchical sheets, and the sheet-scoped local labels they bring.
    assert!(MAD_EDGE.contains("(sheetpath (names \"/MaD_Edge_Sheet2/\")"));
    assert!(MAD_EDGE.contains("(sheetpath (names \"/MaD_Edge_Sheet3/\")"));
    assert!(MAD_EDGE.contains("(name \"/MaD_Edge_Sheet2/IFG_RX\")"));
    // Known landmarks of the EdgeBoard: the 80-finger P2 Edge module socket.
    assert!(MAD_EDGE.contains("(part \"P2_EDGE_MODULE_SOCKET\")"));
    assert_eq!(MAD_EDGE.matches("(comp (ref ").count(), 168);
    assert_eq!(MAD_EDGE.matches("(net (code ").count(), 243);
}
