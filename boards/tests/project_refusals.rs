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
    // JP1 is a jumper by its symbol: a switch's poles cannot replace that.
    let message = refused(&ds2(
        "[[board.model]]\nvalue = \"A0_bypass\"\nkind = \"switch\"\n\
         [board.model.options]\npoles = [[\"1\", \"2\"]]\n",
    ));
    assert_says(
        &message,
        &[
            "[[board.model]] value = \"A0_bypass\" does not reach JP1",
            "its symbol makes it a jumper by itself",
        ],
    );
}

/// The MaD Edge board from its KiCad export, plus `rest`.
fn edge(rest: &str) -> String {
    format!(
        "[[board]]\nname = \"EDGE\"\nkind = \"netlist\"\n\
         netlist = \"../../board/tests/fixtures/mad_edge.net\"\n{rest}"
    )
}

#[rstest]
#[case::converter_as_mechanical(
    ds2("[[board.model]]\npart = \"ADS122U04\"\nkind = \"mechanical\"\n"),
    &[
        "kind \"mechanical\" is for a part whose pads sit on one net at most",
        "U1's pins join 16 nets",
    ]
)]
#[case::converter_as_connector(
    ds2("[[board.model]]\npart = \"ADS122U04\"\nkind = \"boundary\"\n"),
    &[
        "kind \"boundary\" is for a connector: designator J, P or CN, or a Conn… symbol",
        "U1 (designator U, symbol \"ADS122U04\") is neither",
    ]
)]
#[case::converter_as_switch(
    ds2("[[board.model]]\npart = \"ADS122U04\"\nkind = \"switch\"\n\
         [board.model.options]\npoles = [[\"1\", \"2\"]]\n"),
    &["kind \"switch\" is for a switch or jumper", "U1 (designator U, symbol \"ADS122U04\") is neither"]
)]
#[case::converter_as_a_supply(
    ds2("[[board.model]]\npart = \"ADS122U04\"\nkind = \"ucc12040\"\n"),
    &[
        "kind \"ucc12040\" is for a part whose part name, mpn or value contains UCC12040",
        "U1's part name \"ADS122U04\" and value \"ADS122U04\" do not",
    ]
)]
#[case::line_driver_as_a_converter(
    edge("[[board.model]]\nmpn = \"AM26LS31CD\"\nkind = \"ads122u04\"\n"),
    &[
        "kind \"ads122u04\" is for a part whose part name, mpn or value contains ADS122U04",
        "U24's part name \"AM26LS31CD\", mpn \"AM26LS31CD\" and value \"AM26LS31CD\" do not",
    ]
)]
fn a_kind_the_board_says_a_part_is_not_is_refused(#[case] text: String, #[case] says: &[&str]) {
    behaviour!(Test {
        id: "project.refuses-kind-the-part-is-not",
        covers: Some("board/src/kind.rs#KindGuide::check"),
        given: "an integrated circuit given a kind its board says it is not: the force-gauge \
                converter as mechanical, a connector, a switch or a supply, and the Edge line \
                driver as the converter",
    });
    expect!(
        "names-what-the-kind-is-for",
        "the project is refused, naming the part, what the kind is for, and what the board says of it: its nets, designator and symbol, or names",
        "a kind says what a part is, and a part whose pins match a kind's table is any part \
         with as many pins"
    );
    expect!(
        "says-a-model-is-needed",
        "the refusal says a part no kind is for needs a model"
    );
    let message = refused(&text);
    assert_says(&message, says);
    assert_says(
        &message,
        &[
            "is not the part this kind says it is",
            "a part no kind is for needs a model (PROJECTS.md §7)",
        ],
    );
}

#[rstest]
#[case::two_fields(
    "value = \"ADS122U04\"\nkind = \"mechanical\"\n",
    "[[board.model]] part = \"ADS122U04\" and value = \"ADS122U04\" are one registry key"
)]
#[case::one_field(
    "part = \"ADS122U04\"\nkind = \"ads122u04\"\n",
    "two [[board.model]] entries have part = \"ADS122U04\""
)]
fn two_entries_under_one_key_are_refused_whatever_fields_they_name(
    #[case] second: &str,
    #[case] says: &str,
) {
    behaviour!(Test {
        id: "project.refuses-one-key-twice",
        covers: Some("board/src/project.rs#Project::instantiate"),
        given: "two model entries on the force-gauge add-on with the same key, the \
                converter's name, once as its part name and again as its part name or as its \
                value",
    });
    expect!(
        "names-both",
        "the project is refused, naming both entries and that they are one key",
        "the registry looks every key up in one table, so the second entry would replace the \
         first and the model the author gave the part would be gone without a word"
    );
    let message = refused(&ds2(&format!(
        "[[board.model]]\npart = \"ADS122U04\"\nkind = \"ads122u04\"\n\
         [[board.model]]\n{second}"
    )));
    assert_says(&message, &[says, "a key takes one model; keep one"]);
}

#[rstest]
fn a_second_source_with_the_name_of_a_first_is_refused_naming_the_first() {
    behaviour!(Test {
        id: "project.refuses-two-sources-one-name",
        covers: Some("board/src/project.rs#Project::instantiate"),
        given: "two wires with a voltage from the same bench supply name, 0 volts onto one \
                header board's ground and 5 volts onto another's",
    });
    expect!(
        "names-the-first",
        "the project is refused, naming the supply, the first wire's voltage and the first \
         wire",
        "two voltages on one name are two sources fighting through the boards, and a wire \
         joining that name could not say which it joins"
    );
    expect!(
        "says-how",
        "the refusal says to join the supply with a wire that has no voltage, or to give the \
         second source a name of its own"
    );
    let text = "[[board]]\nname = \"A\"\nkind = \"netlist\"\nnetlist = \"header.net\"\n\
                [[board]]\nname = \"B\"\nkind = \"netlist\"\nnetlist = \"header.net\"\n\
                [[wire]]\nfrom = \"BENCH.GND\"\nto = \"A.J1.2\"\nvolts = 0.0\n\
                [[wire]]\nfrom = \"BENCH.GND\"\nto = \"B.J1.2\"\nvolts = 5.0\n";
    let message = refused(text);
    assert_says(
        &message,
        &[
            "[[wire]] BENCH.GND to B.J1.2: BENCH.GND is already a source, at 0 V, made by \
             [[wire]] BENCH.GND to A.J1.2",
            "join it with a wire that has no volts, or give the second source a name of its own",
        ],
    );
}

#[rstest]
#[case::a_pin_b_lacks(
    "a = \"DS2.J1\"\nb = \"HDR.J1\"\n",
    &["[[mate]] DS2.J1 to HDR.J1: HDR.J1 has no pin 3, 4, 5 that DS2.J1 has", "map = [[\"a pin\", \"b pin\"]"]
)]
#[case::not_a_connector(
    "a = \"HDR.R1\"\nb = \"DS2.J1\"\n",
    &["R1 is not a connector, and a mate joins two connectors", "HDR's connectors are J1"]
)]
#[case::map_names_a_missing_pin(
    "a = \"HDR.J1\"\nb = \"DS2.J1\"\nmap = [[\"1\", \"9\"]]\n",
    &["map names pin \"9\" of DS2.J1, which has pins 1, 2, 3, 4, 5"]
)]
#[case::map_names_a_pin_twice(
    "a = \"HDR.J1\"\nb = \"DS2.J1\"\nmap = [[\"1\", \"3\"], [\"2\", \"3\"]]\n",
    &["map names pin \"3\" of DS2.J1 twice; a pin has one mate"]
)]
fn a_mate_that_does_not_fit_its_connectors_is_refused(#[case] mate: &str, #[case] says: &[&str]) {
    behaviour!(Test {
        id: "project.refuses-mate-that-does-not-fit",
        covers: Some("board/src/project.rs#Project::instantiate"),
        given: "a project mating the add-on's five-pin header with a two-pin header: the wider in the narrower, a resistor as a connector, or a cable map naming a missing pin or one pin twice",
    });
    expect!(
        "names-the-misfit",
        "the project is refused, naming the pins one side lacks, the part that is no connector, or the pin the map names wrongly, and what exists",
        "a mate joins every pin of its first connector, so a pin with nothing to land on is a \
         connector pair that does not mate"
    );
    let text = format!(
        "{}{}[[mate]]\n{mate}",
        header(""),
        "[[board]]\nname = \"DS2\"\nkind = \"netlist\"\n\
         netlist = \"../../board/tests/fixtures/ds2_addon.net\"\n\
         [[board.model]]\npart = \"ADS122U04\"\nkind = \"ads122u04\"\n"
    );
    assert_says(&refused(&text), says);
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

#[rstest]
fn the_three_board_machine_project_waits_on_the_two_parts_the_catalog_lacks() {
    behaviour!(Test {
        id: "project.machine-waits-on-two-parts",
        covers: Some("board/src/project.rs#Project::instantiate"),
        given: "the shipped project of the MaD machine's three boards, the Edge carrier, the P2 \
                module in its socket and the force-gauge add-on on its cable, built with the \
                standard catalog alone",
    });
    expect!(
        "names-the-two",
        "the project is refused naming the carrier's RS-422 line driver and line receiver, \
         and only them, as the parts that need a model",
        "every other part of the three boards is placed by the catalog or by the file, and \
         the catalog has no model of either line part yet"
    );
    let path = projects().join("edge-ec32-ds2.toml");
    let message = Project::load(&path)
        .expect("the project loads")
        .instantiate(&StandardCatalog)
        .expect_err("two parts have no model")
        .to_string();
    assert_says(
        &message,
        &[
            "board EDGE is not ready to build",
            "168 parts: 166 classified, 2 need a model, 0 with pins the netlist does not have, \
             0 refused",
            "U24  part \"AM26LS31CD\"",
            "U25  part \"AM26LV32xD\"",
        ],
    );
    let listed = message
        .lines()
        .skip_while(|line| *line != "needs a model:")
        .skip(1)
        .take_while(|line| line.starts_with("  "))
        .count();
    assert_eq!(listed, 2, "{message}");
}
