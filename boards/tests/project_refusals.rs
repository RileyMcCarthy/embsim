//! What a project refuses, and what the refusal tells its author to fix.
//! Build only: every case fails before an engine exists.

use std::path::PathBuf;

use embsim_board::{Project, ProjectError};
use embsim_boards::catalog::StandardCatalog;
use rstest::rstest;
use vibes_behaviour::{behaviour, expect, Test};

/// The directory the shipped example projects live in; a project parsed
/// here resolves its netlist paths against it, as a file there would.
fn projects() -> PathBuf {
    [env!("CARGO_MANIFEST_DIR"), "projects"].iter().collect()
}

/// The DS2 add-on board from its KiCad export, plus `rest`.
fn ds2(rest: &str) -> String {
    format!(
        "[[board]]\nname = \"DS2Addon\"\nkind = \"netlist\"\n\
         netlist = \"../../board/tests/fixtures/ds2_addon.net\"\n{rest}"
    )
}

/// The header board (a connector `J1` and a resistor `R1`), plus `rest`.
fn header(rest: &str) -> String {
    format!("[[board]]\nname = \"HDR\"\nkind = \"netlist\"\nnetlist = \"header.net\"\n{rest}")
}

fn refused(text: &str) -> String {
    let project = Project::parse(text)
        .expect("the text is a project")
        .relative_to(projects());
    let error: ProjectError = project
        .instantiate(&StandardCatalog)
        .expect_err("the project is refused");
    error.to_string()
}

fn assert_says(message: &str, needles: &[&str]) {
    for needle in needles {
        assert!(
            message.contains(needle),
            "{needle:?} missing from:\n{message}"
        );
    }
}

#[rstest]
fn an_integrated_circuit_with_no_model_is_named_with_the_keys_to_assign_it_by() {
    behaviour!(Test {
        id: "project.refuses-unassigned-part",
        covers: Some("board/src/project.rs#Project::instantiate"),
        given: "the force-gauge add-on's netlist in a project with no model assigned to its \
                analog-to-digital converter, which carries no manufacturer part number",
    });
    expect!(
        "names-the-part",
        "the project is refused, naming the converter's reference, its symbol's part name \
         and its value as a part that needs a model",
        "those are the keys a model can be assigned by, and the reference is where it sits"
    );
    expect!(
        "says-how",
        "the refusal says to give it a model entry keyed by its part, number or value, and \
         lists the part kinds that can be given"
    );
    let message = refused(&ds2(""));
    assert_says(
        &message,
        &[
            "board DS2Addon is not ready to build",
            "needs a model:",
            "U1  part \"ADS122U04\"  value \"ADS122U04\"  (16 pins)",
            "give each part that needs a model a [[board.model]] with its part, mpn or value",
            "\"ads122u04\"",
        ],
    );
}

#[rstest]
#[case::not_a_connector(
    "HDR.R1.1",
    &["HDR.R1.1: R1 is not a connector, and a wire lands on a connector pin", "HDR's connectors are J1"]
)]
#[case::no_such_part("HDR.J9.1", &["HDR has no part J9", "HDR's connectors are J1"])]
#[case::no_such_pin("HDR.J1.9", &["J1 has no pin \"9\"; its pins are 1, 2"])]
#[case::no_connector_named(
    "HDR.SIG",
    &["names a board without its connector", "HDR.Connector.Pin", "HDR's connectors are J1"]
)]
fn a_wire_that_misses_a_connector_pin_is_refused_naming_the_connectors(
    #[case] to: &str,
    #[case] says: &[&str],
) {
    behaviour!(Test {
        id: "project.refuses-wire-off-a-connector",
        covers: Some("board/src/project.rs#Project::instantiate"),
        given: "a project wire from a bench supply onto a board endpoint that is no \
                connector pin: a resistor's pin, a missing part, a missing pin, or no \
                connector named",
    });
    expect!(
        "names-the-connectors",
        "the project is refused, naming what the endpoint misses and listing the board's \
         connectors, or the connector's pins",
        "a harness attaches to a board at its boundary, and those are the endpoints the \
         author can choose from"
    );
    let message = refused(&header(&format!(
        "[[wire]]\nfrom = \"BENCH.3V3\"\nto = \"{to}\"\nvolts = 3.3\n"
    )));
    assert_says(&message, says);
}

#[rstest]
fn an_unknown_part_kind_is_refused_naming_the_kinds_there_are() {
    behaviour!(Test {
        id: "project.refuses-unknown-part-kind",
        covers: Some("board/src/project.rs#Project::instantiate"),
        given: "a project that assigns the force-gauge add-on's converter a part kind the \
                catalog does not ship",
    });
    expect!(
        "names-the-kinds",
        "the project is refused, naming the unknown kind and every part kind the catalog \
         ships"
    );
    let message = refused(&ds2(
        "[[board.model]]\npart = \"ADS122U04\"\nkind = \"ads1234\"\n",
    ));
    assert_says(
        &message,
        &[
            "board DS2Addon: [[board.model]] part = \"ADS122U04\": unknown kind \"ads1234\"",
            "the part kinds are \"p2\", \"tg2520smn\"",
            "\"ads122u04\", \"switch\", \"mechanical\", \"boundary\"",
        ],
    );
}

#[rstest]
fn an_unknown_board_kind_is_refused_naming_the_board_kinds() {
    let message = refused("[[board]]\nname = \"B\"\nkind = \"p2-ec64\"\n");
    assert_says(
        &message,
        &["board B: unknown kind \"p2-ec64\"; the board kinds are \"netlist\", \"p2-ec32mb\""],
    );
}

#[rstest]
#[case::no_part_has_it(
    "value = \"ADS1234\"",
    &["[[board.model]] value = \"ADS1234\" matches no part on the board", "the values it has include"]
)]
#[case::another_field_has_it(
    "mpn = \"ADS122U04\"",
    &["[[board.model]] mpn = \"ADS122U04\" matches no part on the board", "U1 has part \"ADS122U04\" — match it by part = \"ADS122U04\""]
)]
fn an_assignment_that_matches_no_part_is_refused(#[case] key: &str, #[case] says: &[&str]) {
    behaviour!(Test {
        id: "project.refuses-assignment-matching-nothing",
        covers: Some("board/src/project.rs#Project::instantiate"),
        given: "a model entry on the force-gauge add-on whose key is no part's value, or is \
                the converter's part name given as a manufacturer part number",
    });
    expect!(
        "names-the-key",
        "the project is refused, naming the field and key that match nothing",
        "an entry that places nothing is a typo or a misunderstanding, and building on \
         without it would leave the author's intent silently unapplied"
    );
    expect!(
        "points-at-the-part",
        "a key another field carries names that part and the field to match it by; any \
         other key lists the keys the field carries"
    );
    let message = refused(&ds2(&format!(
        "[[board.model]]\n{key}\nkind = \"ads122u04\"\n"
    )));
    assert_says(&message, says);
}

#[rstest]
fn an_assignment_another_entry_comes_before_is_refused_naming_the_key_that_wins() {
    behaviour!(Test {
        id: "project.refuses-shadowed-assignment",
        covers: Some("board/src/project.rs#Project::instantiate"),
        given: "the P2-EC32MB's netlist with its low-dropout regulators assigned a model by \
                their value, while the catalog already places them by their manufacturer part \
                number",
    });
    expect!(
        "names-the-winner",
        "the project is refused, naming the first regulator the entry does not reach and the \
         manufacturer part number the lookup reaches first",
        "a part is looked up by part name, then manufacturer part number, then value, so a \
         part the catalog knows by number takes the catalog's model before any entry by value"
    );
    let text = "[[board]]\nname = \"EC32\"\nkind = \"netlist\"\n\
                netlist = \"../netlists/p2_ec32mb.net\"\n\
                [[board.model]]\nvalue = \"LDO 300mA, 3.3V\"\nkind = \"ncp114\"\n\
                [board.model.options]\npins = \"by-function\"\n";
    let message = refused(text);
    assert_says(
        &message,
        &[
            "board EC32: [[board.model]] value = \"LDO 300mA, 3.3V\" does not reach U501",
            "the registry reaches it first by its mpn \"NCP114AMX330TCG\"; assign by mpn = \
             \"NCP114AMX330TCG\" instead",
        ],
    );
}

#[rstest]
fn an_assignment_a_parts_own_symbol_decides_is_refused() {
    let message = refused(&header(
        "[[board.model]]\nvalue = \"10k\"\nkind = \"mechanical\"\n",
    ));
    assert_says(
        &message,
        &[
            "[[board.model]] value = \"10k\" does not reach R1",
            "its symbol makes it a passive (a resistor, capacitor or inductor) by itself",
        ],
    );
}

#[rstest]
#[case::unknown_option(
    "[board.model.options]\nspeed = \"fast\"\n",
    &["unknown option \"speed\"; this kind takes \"pins\""]
)]
#[case::unknown_table(
    "[board.model.options]\npins = \"dip16\"\n",
    &["options.pins = \"dip16\" is not one this kind offers; it offers \"tssop16\""]
)]
fn an_option_the_kind_does_not_offer_is_refused_naming_what_it_does(
    #[case] options: &str,
    #[case] says: &[&str],
) {
    let message = refused(&ds2(&format!(
        "[[board.model]]\npart = \"ADS122U04\"\nkind = \"ads122u04\"\n{options}"
    )));
    assert_says(&message, says);
}

#[rstest]
fn a_name_of_its_own_is_a_supply_a_wire_with_volts_creates() {
    // A second wire may join the supply the first one's volts create.
    let text = "[[board]]\nname = \"A\"\nkind = \"netlist\"\nnetlist = \"header.net\"\n\
                [[board]]\nname = \"B\"\nkind = \"netlist\"\nnetlist = \"header.net\"\n\
                [[wire]]\nfrom = \"BENCH.GND\"\nto = \"A.J1.2\"\nvolts = 0.0\n\
                [[wire]]\nfrom = \"BENCH.GND\"\nto = \"B.J1.2\"\n";
    let built = Project::parse(text)
        .expect("the text is a project")
        .relative_to(projects())
        .instantiate(&StandardCatalog)
        .expect("a wire may join a supply")
        .build()
        .expect("the system builds");
    assert!(built.names_are_merged("A.GND", "B.GND"));

    // A name no volts create is not a place a wire can land.
    let message = refused(&header(
        "[[wire]]\nfrom = \"BENCH.SIG\"\nto = \"HDR.J1.1\"\n",
    ));
    assert_says(
        &message,
        &[
            "BENCH is not a board or bench component in this project (boards: HDR; \
             components: none)",
            "a name of its own is a supply, created by the from of a wire with volts",
        ],
    );
}

#[rstest]
fn a_switch_pole_the_part_does_not_have_is_refused() {
    let message = refused(
        "[[board]]\nname = \"EC32\"\nkind = \"p2-ec32mb\"\n\
         [[board.model]]\nvalue = \"P2X8C4M64P\"\nkind = \"p2\"\n\
         [board.model.options]\ncore = \"held-in-reset\"\n\
         [[switch]]\npart = \"EC32.S301\"\npole = 4\nstate = \"closed\"\n\
         [[switch]]\npart = \"EC32.U100\"\npole = 0\nstate = \"closed\"\n",
    );
    assert_says(&message, &["S301 has 4 poles, numbered from 0"]);
    let message = refused(
        "[[board]]\nname = \"EC32\"\nkind = \"p2-ec32mb\"\n\
         [[board.model]]\nvalue = \"P2X8C4M64P\"\nkind = \"p2\"\n\
         [board.model.options]\ncore = \"held-in-reset\"\n\
         [[switch]]\npart = \"EC32.U100\"\npole = 0\nstate = \"closed\"\n",
    );
    assert_says(
        &message,
        &["U100 is not a switch on EC32; its switches: J101, S301"],
    );
}
