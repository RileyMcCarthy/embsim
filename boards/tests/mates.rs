//! Connectors mated by a project, built only (no engine, no clock): a
//! `[[mate]]` joins each pin of its first connector to the second's pin of
//! the same number, or the pairs a cable's `map` names, and nothing else.

use std::path::PathBuf;

use embsim_board::{BuiltSystem, Project};
use embsim_boards::catalog::StandardCatalog;
use rstest::rstest;
use vibes_behaviour::{behaviour, expect, Test};

/// Two header boards, `LEFT` and `RIGHT` (`projects/header.net`: a
/// connector `J1`, pin 1 on `SIG` and pin 2 on `GND`), mated by `mate`.
fn mated(mate: &str) -> BuiltSystem {
    let projects: PathBuf = [env!("CARGO_MANIFEST_DIR"), "projects"].iter().collect();
    let text = format!(
        "[[board]]\nname = \"LEFT\"\nkind = \"netlist\"\nnetlist = \"header.net\"\n\
         [[board]]\nname = \"RIGHT\"\nkind = \"netlist\"\nnetlist = \"header.net\"\n\
         [[mate]]\n{mate}"
    );
    Project::parse(&text)
        .expect("the text is a project")
        .relative_to(projects)
        .instantiate(&StandardCatalog)
        .expect("the mate fits")
        .build()
        .expect("the system builds")
}

#[rstest]
fn a_mate_joins_two_connectors_pin_for_pin() {
    behaviour!(Test {
        id: "project.mate-by-number",
        covers: Some("board/src/project.rs#Project::instantiate"),
        given: "two boards whose two-pin connectors a project mates with no map, each \
                board's pin 1 on its signal net and pin 2 on its ground",
    });
    expect!(
        "same-numbers-joined",
        "each board's signal is joined to the other's signal, and ground to ground",
        "without a map each pin lands on the other connector's pin of the same number"
    );
    expect!("nothing-else", "a signal is joined to no ground");
    let built = mated("a = \"LEFT.J1\"\nb = \"RIGHT.J1\"\n");
    assert!(built.names_are_merged("LEFT.SIG", "RIGHT.SIG"));
    assert!(built.names_are_merged("LEFT.GND", "RIGHT.GND"));
    assert!(!built.names_are_merged("LEFT.SIG", "RIGHT.GND"));
    assert!(!built.names_are_merged("LEFT.SIG", "LEFT.GND"));
}

#[rstest]
fn a_cable_map_joins_the_pins_it_names_and_only_those() {
    behaviour!(Test {
        id: "project.mate-by-map",
        covers: Some("board/src/project.rs#Project::instantiate"),
        given: "the same two connectors mated by a crossed cable, pin 1 to pin 2 and pin 2 to \
                pin 1, and by a cable that carries pin 1 alone",
    });
    expect!(
        "crossed",
        "the crossed cable joins each board's signal to the other's ground"
    );
    expect!(
        "unmapped-open",
        "the one-wire cable joins the signals and leaves the grounds apart",
        "a cable joins the pins it wires, and a pin it does not wire stays open"
    );
    let crossed =
        mated("a = \"LEFT.J1\"\nb = \"RIGHT.J1\"\nmap = [[\"1\", \"2\"], [\"2\", \"1\"]]\n");
    assert!(crossed.names_are_merged("LEFT.SIG", "RIGHT.GND"));
    assert!(crossed.names_are_merged("LEFT.GND", "RIGHT.SIG"));
    assert!(!crossed.names_are_merged("LEFT.SIG", "RIGHT.SIG"));

    let one = mated("a = \"LEFT.J1\"\nb = \"RIGHT.J1\"\nmap = [[\"1\", \"1\"]]\n");
    assert!(one.names_are_merged("LEFT.SIG", "RIGHT.SIG"));
    assert!(!one.names_are_merged("LEFT.GND", "RIGHT.GND"));
}
