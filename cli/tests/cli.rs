//! The `embsim` command, run as a user runs it: the built binary, on the
//! netlists and projects the workspace ships, and — where QEMU is linked —
//! booting the P2 off the P2-EC32MB's flash.
//!
//! Each case starts the binary as its own process (the virtual clock and a
//! P2 core are one per process), reads what it printed and how it exited,
//! and writes whatever it generates under the test's own directory in
//! Cargo's per-target scratch space. A run is stepped inside the binary
//! (`TESTING.md` rule 9): it ends at exactly the virtual instant asked for
//! and reads the system at rest there.

use std::path::{Path, PathBuf};
use std::process::{Command, Output};

use rstest::rstest;
use vibes_behaviour::{behaviour, expect, Test};

/// The workspace root: the netlists and projects live under it.
fn workspace() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("the CLI crate sits in the workspace")
        .to_path_buf()
}

fn ec32_netlist() -> PathBuf {
    workspace().join("boards/netlists/p2_ec32mb.net")
}

fn ds2_netlist() -> PathBuf {
    workspace().join("board/tests/fixtures/ds2_addon.net")
}

fn header_netlist() -> PathBuf {
    workspace().join("boards/projects/header.net")
}

/// A directory of the test's own, emptied.
fn scratch(test: &str) -> PathBuf {
    let dir = Path::new(env!("CARGO_TARGET_TMPDIR")).join(test);
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).expect("the scratch directory can be made");
    dir
}

/// `embsim` with `args`, to completion.
fn embsim(args: &[&str]) -> Output {
    Command::new(env!("CARGO_BIN_EXE_embsim"))
        .args(args)
        .output()
        .expect("the embsim binary runs")
}

fn stdout(output: &Output) -> String {
    String::from_utf8_lossy(&output.stdout).into_owned()
}

fn stderr(output: &Output) -> String {
    String::from_utf8_lossy(&output.stderr).into_owned()
}

/// `text` with every run of spaces made one, so a row reads the same
/// however its columns are padded.
fn squeezed(text: &str) -> String {
    text.lines()
        .map(|line| line.split_whitespace().collect::<Vec<_>>().join(" "))
        .collect::<Vec<_>>()
        .join("\n")
}

fn assert_says(text: &str, needles: &[&str]) {
    let text = squeezed(text);
    for needle in needles {
        assert!(text.contains(needle), "{needle:?} missing from:\n{text}");
    }
}

fn path(path: &Path) -> &str {
    path.to_str().expect("the workspace path is text")
}

/// The lines of the stub that starts `# [[board.model]]` then `key_line`,
/// uncommented, with `kind` given when the stub left it empty.
fn fill_stub(text: &str, key_line: &str, kind: &str, options: &str) -> String {
    let head = format!("# [[board.model]]\n# {key_line}\n");
    let start = text
        .find(&head)
        .unwrap_or_else(|| panic!("no stub keyed {key_line}:\n{text}"));
    let end = text[start..]
        .find("\n\n")
        .map_or(text.len(), |offset| start + offset);
    let filled: Vec<String> = text[start..end]
        .lines()
        .map(|line| line.strip_prefix("# ").unwrap_or(line).to_string())
        .map(|line| {
            if line == "kind = \"\"" {
                format!("kind = {kind:?}")
            } else {
                line
            }
        })
        .collect();
    format!(
        "{}{}{options}{}",
        &text[..start],
        filled.join("\n"),
        &text[end..]
    )
}

// ============================================================
// embsim survey
// ============================================================

#[rstest]
fn the_survey_lists_every_ec32_connector_pin_with_its_name_and_net() {
    behaviour!(Test {
        id: "cli.survey-connectors",
        covers: Some("cli/src/checklist.rs#survey"),
        given: "the P2-EC32MB's transcribed netlist, surveyed from the command line",
    });
    expect!(
        "counts",
        "the survey counts 114 parts: 91 populated, 4 needing a model, 19 placed with a pin \
         table the netlist does not use, 5 connectors",
        "the catalog places a part by its symbol, its reference designator or its part number; \
         what is left is what a project still has to say"
    );
    expect!(
        "every-connector",
        "the solder link, 60 edge fingers, card socket and two mounting holes are each listed \
         as a connector with its value and pin count"
    );
    expect!(
        "finger-rows",
        "each edge finger is a row of its printed label and its net: 41 is 5V on the input \
         rail, 43 is GND on ground",
        "a connector pin is where a wire may join the board to another board, a bench \
         component or a supply, so its name and net are what a project author wires by"
    );
    let output = embsim(&["survey", path(&ec32_netlist())]);
    assert!(output.status.success(), "{}", stderr(&output));
    let text = stdout(&output);
    assert_says(
        &text,
        &[
            "p2_ec32mb.net: 114 parts",
            "91 populated by the catalog",
            "4 need a model",
            "19 placed with a pin table the netlist does not use",
            "5 connectors",
        ],
    );
    assert_says(
        &text,
        &[
            "J101 value \"Solder Link Pads\" 2 pins",
            "J203 value \"Edge Socket Pads\" 60 pins",
            "J301 value \"MicroSD Socket\" 8 pins",
            "J701 value \"Mounting Hole Vss\" 1 pin",
            "J702 value \"Mounting Hole Vss\" 1 pin",
        ],
    );
    assert_says(
        &text,
        &[
            "41 5V VIN_Edge",
            "43 GND GND",
            "50 P62/PRG_DBG_TXD P2_IO62_TXD",
            "CD_DAT3_CS CD/DAT3/CS P2_IO60",
        ],
    );
}

#[rstest]
fn the_survey_names_what_each_unmodelled_part_could_be_and_the_pin_table_that_fits() {
    behaviour!(Test {
        id: "cli.survey-checklist",
        covers: Some("cli/src/checklist.rs#survey"),
        given: "the P2-EC32MB's transcribed netlist, surveyed from the command line",
    });
    expect!(
        "processor-by-number",
        "the processor is listed as needing a model, with its 86 pins, and the P2 package as \
         the kind its part number names",
        "the catalog never places a processor by itself: what runs inside it is the project's \
         choice"
    );
    expect!(
        "switch-by-designator",
        "the eight-pin option switch, whose number no model is for, is offered the switch kind \
         alone, by its S designator",
        "a pin table says how many pins a part has, and the designator says what the part is"
    );
    expect!(
        "table-that-fits",
        "every part placed with the datasheet's numbered pins is listed with both pin lists \
         and the function-named table that has the netlist's pins",
        "the transcription names pins by function, and each model offers that table beside \
         its numbered one"
    );
    let output = embsim(&["survey", path(&ec32_netlist())]);
    assert!(output.status.success(), "{}", stderr(&output));
    let text = squeezed(&stdout(&output));
    assert_says(
        &text,
        &[
            "U100 value \"P2X8C4M64P\" mpn \"P2X8C4M64P\"",
            "86 pins: GND, P0, P1,",
            "could be: p2 (part number P2X8C4M64P)",
        ],
    );
    assert_says(
        &text,
        &[
            "S301 value \"DIP Switch 4 way\" mpn \"218-4LPSTJR\"",
            "8 pins: 1_OFF, 1_ON, 2_OFF, 2_ON, 3_OFF, 3_ON, 4_OFF, 4_ON\n\
             no catalog model is for this part; it may be switch (its designator S)",
        ],
    );
    assert_says(
        &text,
        &[
            "U402, U403 mpn \"AP62301Z6-7\": ap62301, pins = \"sot563\"",
            "declares 1, 2, 3, 4, 5, 6",
            "the netlist has BST, FB, GND, SW, VIN",
        ],
    );
    assert_eq!(
        text.matches("pins = \"by-function\" declares the netlist's pins")
            .count(),
        7,
        "one fix per model the catalog placed by number:\n{text}"
    );
}

#[rstest]
fn the_edge_boards_survey_names_the_parts_no_kind_is_for() {
    behaviour!(Test {
        id: "cli.survey-edge-gaps",
        covers: Some("cli/src/checklist.rs#survey"),
        given: "the MaD Edge board's KiCad export, surveyed from the command line",
    });
    expect!(
        "line-parts-need-models",
        "the RS-422 line driver and line receiver are listed as needing a model, each told it \
         needs one written for it",
        "no kind the catalog ships is for either part, by its numbers or by what the board \
         says it is"
    );
    expect!(
        "socket-is-a-connector",
        "the module socket, a symbol of the board's own library, is offered the connector kind \
         by its J designator"
    );
    expect!(
        "names-disagree",
        "the receiver is reported with its symbol and its manufacturer part number naming two \
         different parts",
        "the symbol names the 3.3 volt AM26LV32 and the part number the 5 volt AM26LS32, and a \
         model is one part's"
    );
    let netlist = workspace().join("board/tests/fixtures/mad_edge.net");
    let output = embsim(&["survey", path(&netlist)]);
    assert!(output.status.success(), "{}", stderr(&output));
    let text = squeezed(&stdout(&output));
    assert_says(&text, &["168 parts", "3 need a model"]);
    assert_says(
        &text,
        &[
            "U24 part \"AM26LS31CD\" value \"AM26LS31CD\" mpn \"AM26LS31CD\"\n\
             16 pins: 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16\n\
             no catalog kind is for this part: it needs a model (PROJECTS.md §7)",
            "U25 part \"AM26LV32xD\" value \"AM26LV32xD\" mpn \"AM26LS32CD\"\n\
             16 pins: 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16\n\
             its symbol names \"AM26LV32xD\" and its mpn \"AM26LS32CD\": two parts; give it the \
             model of the one the board carries\n\
             no catalog kind is for this part: it needs a model (PROJECTS.md §7)",
            "J3 part \"P2_EDGE_MODULE_SOCKET\" value \"P2_EDGE_MODULE_SOCKET\" mpn \"450-00309\"",
            "no catalog model is for this part; it may be boundary (its designator J)",
        ],
    );
}

#[rstest]
fn a_board_kind_the_catalog_ships_is_surveyed_with_the_registry_it_builds_with() {
    behaviour!(Test {
        id: "cli.survey-kind",
        covers: Some("cli/src/checklist.rs#survey_kind"),
        given: "the P2-EC32MB board kind the catalog ships, surveyed from the command line by \
                its kind",
    });
    expect!(
        "slot-left",
        "the processor is the one part left to the project, offered the P2 kind by its part \
         number",
        "the board kind places every other part of the module the way the board library \
         builds it"
    );
    expect!(
        "finger-rows",
        "each edge finger is listed with its printed label and its net: 41 is 5V on the input \
         rail, 43 is GND on ground",
        "a project wires or mates the module by these connector pins"
    );
    let output = embsim(&["survey", "--kind", "p2-ec32mb"]);
    assert!(output.status.success(), "{}", stderr(&output));
    let text = squeezed(&stdout(&output));
    assert_says(
        &text,
        &[
            "kind \"p2-ec32mb\": 114 parts",
            "113 populated by the catalog",
            "1 need a model",
            "0 placed with a pin table the netlist does not use",
            "U100 value \"P2X8C4M64P\" mpn \"P2X8C4M64P\"",
            "could be: p2 (part number P2X8C4M64P)",
            "J203 value \"Edge Socket Pads\" 60 pins",
            "41 5V VIN_Edge",
            "43 GND GND",
        ],
    );
    let refused = embsim(&["survey", "--kind", "p2-ec64"]);
    assert!(!refused.status.success());
    assert_says(
        &stderr(&refused),
        &["unknown kind \"p2-ec64\"; the board kinds are \"netlist\", \"p2-ec32mb\""],
    );
}

#[rstest]
fn a_netlist_that_cannot_be_read_is_an_error_naming_it() {
    let output = embsim(&["survey", "no/such/board.net"]);
    assert!(!output.status.success());
    assert_says(
        &stderr(&output),
        &["error:", "cannot read netlist no/such/board.net"],
    );
}

// ============================================================
// embsim new, embsim check
// ============================================================

/// The header board's starter project, as `embsim new` writes it.
fn header_starter(test: &str) -> (PathBuf, String) {
    let dir = scratch(test);
    let project = dir.join("header.toml");
    let output = embsim(&[
        "new",
        path(&header_netlist()),
        "--name",
        "HDR",
        "-o",
        path(&project),
    ]);
    assert!(output.status.success(), "{}", stderr(&output));
    let text = std::fs::read_to_string(&project).expect("new wrote the project");
    (project, text)
}

#[rstest]
fn the_header_boards_starter_project_lists_its_pins_and_checks_as_written() {
    behaviour!(Test {
        id: "cli.new-header",
        covers: Some("cli/src/checklist.rs#new_project"),
        given: "a starter project written for a board of one two-pin connector and a 10 \
                kilohm resistor",
    });
    expect!(
        "pins-listed",
        "the file lists each connector pin as the endpoint a wire names, with its pin name and \
         its net"
    );
    expect!("nothing-to-fill", "the file has no model left to fill in");
    expect!(
        "checks-as-written",
        "the file passes the check exactly as written",
        "every part of the board is populated by the catalog, so nothing is left to fill in"
    );
    let (project, text) = header_starter("header_starter");
    assert_says(&text, &["# HDR.J1.1 Pin_1 SIG", "# HDR.J1.2 Pin_2 GND"]);
    assert!(!text.contains("# [[board.model]]"), "{text}");
    let checked = embsim(&["check", path(&project)]);
    assert!(checked.status.success(), "{}", stderr(&checked));
    assert_says(
        &stdout(&checked),
        &["board HDR (netlist): 2 parts: 2 classified", "ok:"],
    );
}

#[rstest]
fn a_wire_to_the_pin_the_starter_project_lists_on_ground_holds_the_signal_low() {
    behaviour!(Test {
        id: "cli.new-header-wired",
        covers: Some("cli/src/live.rs#run"),
        given: "the header board's starter project with one wire added, a 0 volt supply to \
                the endpoint the file lists on the ground net, run for a microsecond",
    });
    expect!(
        "endpoint-copied",
        "that endpoint is the connector's second pin"
    );
    expect!(
        "checks-wired",
        "the project passes the check with its one wire"
    );
    expect!(
        "signal-pulled-low",
        "the signal net reads pulled low through the resistor's 10 kilohms",
        "the resistor joins the signal net to the grounded pin, and nothing else drives it"
    );
    let (project, text) = header_starter("header_wired");
    let row = squeezed(&text)
        .lines()
        .find(|line| line.ends_with(" GND") && line.starts_with("# HDR."))
        .expect("a row on the ground net")
        .to_string();
    let endpoint = row.split(' ').nth(1).expect("the row names its endpoint");
    assert_eq!(endpoint, "HDR.J1.2");
    std::fs::write(
        &project,
        format!("{text}\n[[wire]]\nfrom = \"BENCH.GND\"\nto = \"{endpoint}\"\nvolts = 0.0\n"),
    )
    .expect("the project is writable");
    let checked = embsim(&["check", path(&project)]);
    assert!(checked.status.success(), "{}", stderr(&checked));
    assert_says(
        &stdout(&checked),
        &["1 board, 0 bench components, 1 wire, 0 mates", "ok:"],
    );

    let ran = embsim(&["run", path(&project), "--for", "1us", "--net", "HDR.SIG"]);
    assert!(ran.status.success(), "{}", stderr(&ran));
    assert_says(
        &stdout(&ran),
        &[
            "ran 0.001000 ms of virtual time",
            "net HDR.SIG: Pulled(Low, 10000.0)",
        ],
    );
}

#[rstest]
fn new_refuses_to_replace_a_project_unless_forced() {
    let dir = scratch("new_no_clobber");
    let project = dir.join("header.toml");
    std::fs::write(&project, "# mine\n").expect("the file is writable");
    let output = embsim(&["new", path(&header_netlist()), "-o", path(&project)]);
    assert!(!output.status.success());
    assert_says(&stderr(&output), &["exists; pass --force to replace it"]);
    assert_eq!(
        std::fs::read_to_string(&project).expect("the file is still there"),
        "# mine\n"
    );
    let forced = embsim(&[
        "new",
        path(&header_netlist()),
        "-o",
        path(&project),
        "--force",
    ]);
    assert!(forced.status.success(), "{}", stderr(&forced));
}

/// The DS2 add-on's starter project, as `embsim new` writes it.
fn ds2_starter(test: &str) -> (PathBuf, String) {
    let dir = scratch(test);
    let project = dir.join("ds2.toml");
    let output = embsim(&[
        "new",
        path(&ds2_netlist()),
        "--name",
        "DS2Addon",
        "-o",
        path(&project),
    ]);
    assert!(output.status.success(), "{}", stderr(&output));
    let text = std::fs::read_to_string(&project).expect("new wrote the project");
    (project, text)
}

#[rstest]
fn check_refuses_a_board_whose_converter_has_no_model_with_the_survey() {
    behaviour!(Test {
        id: "cli.check-refuses-unassigned-ic",
        covers: Some("cli/src/live.rs#check"),
        given: "the starter project written for the force-gauge add-on, whose converter the \
                catalog cannot place, with the model it suggests for the converter left \
                commented out",
    });
    expect!(
        "suggests-the-kind",
        "the file suggests the ADS122U04 kind for the converter, keyed by its symbol's part name",
        "the part carries no manufacturer part number, and its part name is the start of the \
         numbers the kind is for"
    );
    expect!("exits-non-zero", "the check fails with a non-zero exit");
    expect!(
        "survey-text",
        "the error is the board's survey: the converter by reference, part name, value and pin \
         count, and how to give it a model"
    );
    let (project, text) = ds2_starter("ds2_unassigned");
    assert!(
        text.contains("# [[board.model]]\n# part = \"ADS122U04\"\n# kind = \"ads122u04\"\n"),
        "{text}"
    );
    let checked = embsim(&["check", path(&project)]);
    assert!(!checked.status.success(), "{}", stdout(&checked));
    let error = stderr(&checked);
    assert!(
        error.contains("U1  part \"ADS122U04\"  value \"ADS122U04\"  (16 pins)"),
        "{error}"
    );
    assert_says(
        &error,
        &[
            "error: board DS2Addon is not ready to build",
            "needs a model:",
            "give each part that needs a model a [[board.model]] with its part, mpn or value",
        ],
    );
}

#[rstest]
fn the_add_ons_starter_project_checks_once_its_stub_is_uncommented() {
    behaviour!(Test {
        id: "cli.new-suggestion-taken",
        covers: Some("cli/src/checklist.rs#new_project"),
        given: "the starter project written for the force-gauge add-on with the model it \
                suggests for the converter uncommented as written",
    });
    expect!(
        "checks",
        "the project passes the check, every part of the add-on classified"
    );
    let (project, text) = ds2_starter("ds2_filled");
    let filled = fill_stub(&text, "part = \"ADS122U04\"", "", "");
    std::fs::write(&project, filled).expect("the project is writable");
    let checked = embsim(&["check", path(&project)]);
    assert!(checked.status.success(), "{}", stderr(&checked));
    assert_says(&stdout(&checked), &["0 need a model", "ok:"]);
}

/// Each `[[board.model]]` of the first board that picks a pin table: its
/// key, kind and table.
fn pin_table_entries(text: &str) -> Vec<(String, String, String)> {
    let table: toml::Table = toml::from_str(text).expect("the project is TOML");
    let models = table["board"][0]
        .get("model")
        .and_then(toml::Value::as_array)
        .cloned()
        .unwrap_or_default();
    let mut entries: Vec<(String, String, String)> = models
        .iter()
        .filter_map(|model| {
            let pins = model.get("options")?.get("pins")?.as_str()?.to_string();
            let key = ["part", "mpn", "value"]
                .iter()
                .find_map(|field| model.get(*field))?
                .as_str()?
                .to_string();
            let kind = model.get("kind")?.as_str()?.to_string();
            Some((key, kind, pins))
        })
        .collect();
    entries.sort();
    entries
}

/// The P2-EC32MB's starter project, as `embsim new` writes it.
fn ec32_starter(test: &str) -> (PathBuf, String) {
    let dir = scratch(test);
    let project = dir.join("ec32.toml");
    let output = embsim(&[
        "new",
        path(&ec32_netlist()),
        "--name",
        "EC32",
        "-o",
        path(&project),
    ]);
    assert!(output.status.success(), "{}", stderr(&output));
    let text = std::fs::read_to_string(&project).expect("new wrote the project");
    (project, text)
}

#[rstest]
fn the_ec32_starter_project_chooses_the_tables_the_hand_written_project_does() {
    behaviour!(Test {
        id: "cli.new-ec32",
        covers: Some("cli/src/checklist.rs#new_project"),
        given: "the starter project written for the P2-EC32MB's transcribed netlist",
    });
    expect!(
        "tables-chosen",
        "the file picks the function-named pin table for the same seven part numbers the \
         project written by hand for this board picks it for",
        "each is the one table of its model that has the netlist's pins"
    );
    expect!(
        "four-suggestions",
        "the processor, the option switch and the two bill-of-materials lines each get a \
         commented-out model entry, the processor's naming the P2 package"
    );
    let (_, text) = ec32_starter("ec32_starter");
    let by_hand = std::fs::read_to_string(workspace().join("boards/projects/ec32-netlist.toml"))
        .expect("the hand-written project");
    let generated = pin_table_entries(&text);
    assert_eq!(generated.len(), 7, "{generated:?}");
    assert_eq!(generated, pin_table_entries(&by_hand));
    for key_line in [
        "mpn = \"P2X8C4M64P\"",
        "mpn = \"218-4LPSTJR\"",
        "mpn = \"300-64002\"",
        "value = \"Layout node\"",
    ] {
        assert!(
            text.contains(&format!("# [[board.model]]\n# {key_line}\n")),
            "no stub keyed {key_line}"
        );
    }
    assert!(
        text.contains("# mpn = \"P2X8C4M64P\"\n# kind = \"p2\"\n"),
        "{text}"
    );
}

#[rstest]
fn the_ec32_starter_project_checks_once_its_stubs_are_filled() {
    behaviour!(Test {
        id: "cli.new-ec32-filled",
        covers: Some("cli/src/checklist.rs#new_project"),
        given: "the starter project written for the P2-EC32MB's transcribed netlist with its \
                four stubs filled: the processor with no core running, the option switch's \
                four poles, and the two bill-of-materials lines as mechanical",
    });
    expect!(
        "checks-filled",
        "the project passes the check with all 114 parts classified"
    );
    let (project, text) = ec32_starter("ec32_filled");
    let text = fill_stub(&text, "mpn = \"P2X8C4M64P\"", "", "");
    let text = fill_stub(
        &text,
        "mpn = \"218-4LPSTJR\"",
        "switch",
        "\n[board.model.options]\npoles = [[\"1_ON\", \"1_OFF\"], [\"2_ON\", \"2_OFF\"], \
         [\"3_ON\", \"3_OFF\"], [\"4_ON\", \"4_OFF\"]]",
    );
    let text = fill_stub(&text, "mpn = \"300-64002\"", "mechanical", "");
    let text = fill_stub(&text, "value = \"Layout node\"", "mechanical", "");
    std::fs::write(&project, text).expect("the project is writable");
    let checked = embsim(&["check", path(&project)]);
    assert!(checked.status.success(), "{}", stderr(&checked));
    assert_says(
        &stdout(&checked),
        &[
            "board EC32 (netlist): 114 parts: 114 classified, 0 need a model",
            "ok:",
        ],
    );
}

// ============================================================
// embsim run
// ============================================================

/// The report of a run, less the one line that is wall time.
fn virtual_report(output: &Output) -> String {
    stdout(output)
        .lines()
        .filter(|line| !line.starts_with("ran "))
        .collect::<Vec<_>>()
        .join("\n")
}

#[rstest]
fn a_run_of_the_ec32_project_reads_its_rails_up_and_repeats_exactly() {
    behaviour!(Test {
        id: "cli.run-ec32-rails",
        covers: Some("cli/src/live.rs#run"),
        given: "the P2-EC32MB project powered from its carrier's edge fingers, run from the \
                command line for 10 milliseconds of virtual time, twice",
    });
    expect!(
        "ran-the-duration",
        "the run reports exactly 10 milliseconds of virtual time"
    );
    expect!(
        "rails-up",
        "the core rail reads the buck's 1.813 volts and a bank rail the regulator's 3.3 \
         volts",
        "both regulators' soft-starts end at 2.5 milliseconds, well inside the run"
    );
    expect!(
        "same-report",
        "the two runs print the same report, line for line, but for the wall time they took",
        "virtual time is stepped: it advances only to the next instant something happens"
    );
    expect!(
        "build-findings-apart",
        "the findings the build made are printed under a heading that says they are the \
         system before its first wake",
        "every rail with a soft-start is down then, and the build reports each as unsourced"
    );
    expect!(
        "cleared-at-the-end",
        "at the end, the core rail's unsourced finding is listed as cleared with the rail's \
         voltage, and the undriven floating pins as still true"
    );
    let project = workspace().join("boards/projects/ec32-netlist.toml");
    let args = [
        "run",
        path(&project),
        "--for",
        "10ms",
        "--net",
        "EC32.Common_VDD",
        "--net",
        "EC32.VIO_56_63",
    ];
    let first = embsim(&args);
    assert!(first.status.success(), "{}", stderr(&first));
    let text = stdout(&first);
    assert_says(&text, &["ran 10.000000 ms of virtual time"]);
    let core = text
        .lines()
        .find_map(|line| line.strip_prefix("net EC32.Common_VDD: Analog("))
        .and_then(|rest| rest.strip_suffix(')'))
        .and_then(|volts| volts.parse::<f64>().ok())
        .unwrap_or_else(|| panic!("the core rail reads a voltage:\n{text}"));
    assert!((core - 0.8 * (1.0 + 13.3 / 10.5)).abs() < 1e-9, "{core}");
    assert_says(&text, &["net EC32.VIO_56_63: Analog(3.3)"]);
    assert_says(
        &text,
        &[
            "findings at build, before any wake (35):\nFloatingSense",
            "PowerNetUnsourced { net: \"EC32.Common_VDD\" }\n",
            "findings: 35 (35 at build, 0 while running)",
            "at 10.000000 ms, each finding's net read again:\nno longer true (18):",
            "PowerNetUnsourced { net: \"EC32.Common_VDD\" }: EC32.Common_VDD reads Analog(1.81",
            "still true (17):\nFloatingSense { net: \"EC32.P2_IO59\", kind: Digital }",
        ],
    );
    let second = embsim(&args);
    assert!(second.status.success(), "{}", stderr(&second));
    assert_eq!(virtual_report(&first), virtual_report(&second));
}

#[rstest]
fn a_run_asked_for_a_net_the_system_does_not_have_is_refused() {
    let project = workspace().join("boards/projects/header.toml");
    let output = embsim(&["run", path(&project), "--for", "1us", "--net", "HDR.NOPE"]);
    assert!(!output.status.success());
    assert_says(&stderr(&output), &["--net HDR.NOPE: no such net"]);
}

#[rstest]
#[case::no_unit("20")]
#[case::bad_unit("20min")]
fn a_duration_without_a_unit_of_time_is_a_usage_error(#[case] duration: &str) {
    let project = workspace().join("boards/projects/header.toml");
    let output = embsim(&["run", path(&project), "--for", duration]);
    assert_eq!(output.status.code(), Some(2), "{}", stderr(&output));
    assert_says(&stderr(&output), &["give one of ns, us, ms, s"]);
}

#[rstest]
fn an_interrupted_run_says_when_and_prints_its_summary() {
    behaviour!(Test {
        id: "cli.run-interrupted",
        covers: Some("cli/src/live.rs#run"),
        given: "the header board's project run from the command line with no duration, sent \
                an interrupt once it says it is running",
    });
    expect!(
        "says-when",
        "the run says it was interrupted and at which instant of virtual time"
    );
    expect!(
        "summary",
        "it then prints the summary a run that reaches its duration prints: the time it ran \
         and its findings read again"
    );
    expect!("exits-ok", "the command's exit status is zero");
    use std::io::{BufRead, BufReader, Read};
    use std::process::Stdio;
    let project = workspace().join("boards/projects/header.toml");
    let mut child = Command::new(env!("CARGO_BIN_EXE_embsim"))
        .args(["run", path(&project)])
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("the embsim binary runs");
    let mut stdout = BufReader::new(child.stdout.take().expect("stdout is piped"));
    let mut head = String::new();
    loop {
        let mut line = String::new();
        let read = stdout.read_line(&mut line).expect("stdout reads");
        assert!(
            read > 0,
            "the run ended before it said it was running:\n{head}"
        );
        head.push_str(&line);
        if line.starts_with("running until interrupted") {
            break;
        }
    }
    // SAFETY: signalling a child this test spawned and has not reaped.
    let sent = unsafe { libc::kill(child.id() as libc::pid_t, libc::SIGINT) };
    assert_eq!(sent, 0, "SIGINT reaches the run");
    let mut rest = String::new();
    stdout
        .read_to_string(&mut rest)
        .expect("the rest of stdout reads");
    let status = child.wait().expect("the run ends");
    let text = format!("{head}{rest}");
    assert!(status.success(), "{status:?}\n{text}");
    assert_says(
        &text,
        &[
            "running until interrupted",
            "interrupted at ",
            "ms of virtual time\nran ",
            "findings: ",
        ],
    );
}

/// A project of one host on its own rail, and nothing else.
const HOST_PROJECT: &str = r#"
[[component]]
name = "HOST"
kind = "host-serial"
[component.options]
baud = 115200

[[wire]]
from = "RAIL.3V3"
to = "HOST.VIO"
volts = 3.3

[[wire]]
from = "RAIL.GND"
to = "HOST.GND"
volts = 0.0
"#;

#[rstest]
fn a_run_puts_the_hosts_pty_where_pty_says_and_prints_its_path() {
    behaviour!(Test {
        id: "cli.run-pty",
        covers: Some("cli/src/live.rs#apply_ptys"),
        given: "a project with one host serial port, run for a millisecond with --pty naming \
                a path",
    });
    expect!(
        "path-printed",
        "the run prints the port's path, its baud rate and its framing at its first look, \
         before virtual time moves",
        "a host opens the path while the run goes on"
    );
    expect!(
        "summary-counts",
        "the summary says how many bytes crossed each way"
    );
    let dir = scratch("pty");
    let project = dir.join("host.toml");
    std::fs::write(&project, HOST_PROJECT).expect("the project is writable");
    let pty = dir.join("tty.host");
    let output = embsim(&["run", path(&project), "--for", "1ms", "--pty", path(&pty)]);
    assert!(output.status.success(), "{}", stderr(&output));
    let text = stdout(&output);
    assert_says(
        &text,
        &[
            &format!(
                "0.000000 ms] HOST: host serial at {}, 115200 baud 8N1",
                pty.display()
            ),
            &format!(
                "HOST: host serial at {}: 0 bytes from the host, 0 to it, 0 framing errors",
                pty.display()
            ),
        ],
    );
}

#[rstest]
fn a_pty_without_a_name_is_refused_when_the_project_has_two_hosts() {
    let dir = scratch("pty_two");
    let project = dir.join("hosts.toml");
    let two = format!(
        "{HOST_PROJECT}\n[[component]]\nname = \"HOST2\"\nkind = \"host-serial\"\n\
         [component.options]\nbaud = 9600\n"
    );
    std::fs::write(&project, two).expect("the project is writable");
    let output = embsim(&[
        "run",
        path(&project),
        "--for",
        "1ms",
        "--pty",
        path(&dir.join("tty")),
    ]);
    assert!(!output.status.success());
    assert_says(
        &stderr(&output),
        &["has 2 host-serial components, HOST, HOST2; say which with --pty NAME=PATH"],
    );
}

// ============================================================
// embsim run, with QEMU as the P2's core
// ============================================================
//
// `embsim run` booting the P2 on QEMU the way
// `p2-qemu/tests/rom_boot_ec32mb.rs` boots it: the P2-EC32MB from the
// catalog, powered from its carrier's fingers, `S301` positions 2 and 4
// closed, the boot flash holding stage-1 and a three-instruction program
// that writes `B` to the debug pin — all said by a project file.
//
// Which case is built depends on whether QEMU is linked
// (`EMBSIM_QEMU_P2_BUILD`, `cfg(qemu_linked)` from this crate's build
// script), as with `rom_boot_ec32mb.rs`: with it, the boot; without it, the
// refusal that says how to get it. Neither declares a behaviour: which one
// runs is the build's choice, not the project's.

/// The project, its flash image beside it.
const PROJECT: &str = r#"
[[board]]
name = "EC32"
kind = "p2-ec32mb"

[[board.model]]
value = "P2X8C4M64P"
kind = "p2"
[board.model.options]
core = "qemu"

[[board.model]]
value = "SPI Flash 16MB (128Mb)"
kind = "w25q128jv"
[board.model.options]
pins = "by-function"
image = "boot.bin"

[[wire]]
from = "CARRIER.5V"
to = "EC32.J203.41"
volts = 5.0

[[wire]]
from = "CARRIER.5Vb"
to = "EC32.J203.42"
volts = 5.0

[[wire]]
from = "CARRIER.GND"
to = "EC32.J203.43"
volts = 0.0

[[wire]]
from = "CARRIER.GNDb"
to = "EC32.J203.44"
volts = 0.0

[[wire]]
from = "CARRIER.GNDc"
to = "EC32.J203.45"
volts = 0.0

[[switch]]
part = "EC32.S301"
pole = 1
state = "closed"

[[switch]]
part = "EC32.S301"
pole = 3
state = "closed"
"#;

/// The project and its flash image in a directory of the test's own.
fn boot_project(test: &str) -> PathBuf {
    let dir = scratch(test);
    // mov pa,#"B" / wypin pa,#62 / jmp #$
    let program: Vec<u8> = [0xF607_EC42u32, 0xFC27_EC3E, 0xFD9F_FFFC]
        .iter()
        .flat_map(|word| word.to_le_bytes())
        .collect();
    let image = embsim_p2_qemu::flashimage::boot_flash(embsim_p2_qemu::STAGE1, &program)
        .expect("stage-1 fits its kilobyte");
    std::fs::write(dir.join("boot.bin"), image).expect("the image is writable");
    let project = dir.join("boot.toml");
    std::fs::write(&project, PROJECT).expect("the project is writable");
    project
}

#[cfg(qemu_linked)]
#[test]
fn run_boots_the_p2_off_the_modules_flash() {
    let project = boot_project("qemu_boot");
    let output = embsim(&[
        "run",
        project.to_str().expect("text"),
        "--for",
        "20ms",
        "--net",
        "EC32.Common_VDD",
    ]);
    let text = String::from_utf8_lossy(&output.stdout);
    assert!(
        output.status.success(),
        "{}\n{text}",
        String::from_utf8_lossy(&output.stderr)
    );
    // The START gate: 3 ms after the bucks' 2.5 ms soft-start releases the
    // reset (rom_boot_ec32mb.rs, START_NS).
    assert!(
        text.contains("EC32.U100: the core started at 5.500000 ms"),
        "{text}"
    );
    // The program the flash served reached the debug pin.
    assert!(text.contains("EC32.U100: P62 \"B\""), "{text}");
    assert!(
        text.contains("EC32.U100: core \"qemu\": started at 5.500000 ms"),
        "{text}"
    );
    assert!(text.contains("EC32.U100: QEMU: "), "{text}");
    assert!(text.contains("console P62 \"B\""), "{text}");
    assert!(text.contains("ran 20.000000 ms of virtual time"), "{text}");
}

#[cfg(not(qemu_linked))]
#[test]
fn without_qemu_a_qemu_core_is_refused_saying_how_to_link_it() {
    let project = boot_project("qemu_refused");
    let output = embsim(&["check", project.to_str().expect("text")]);
    assert!(!output.status.success());
    let error = String::from_utf8_lossy(&output.stderr);
    assert!(
        error.contains(
            "board EC32: [[board.model]] value = \"P2X8C4M64P\" (kind \"p2\"): \
             embsim-p2-qemu was built without a QEMU tree; set EMBSIM_QEMU_P2_BUILD"
        ),
        "{error}"
    );
}
