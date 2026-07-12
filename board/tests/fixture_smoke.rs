//! Sanity checks on the committed netlist fixtures.
//!
//! The parser slice adds golden component/net-graph assertions per supported
//! KiCad major; this smoke test pins the fixture's shape so a bad re-export
//! is caught immediately.

const DS2_ADDON: &str = include_str!("fixtures/ds2_addon.net");

#[test]
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
