//! The `embsim` command, run as a user runs it: the built binary, on the
//! netlists and projects the workspace ships, and — where a
//! `qemu-system-p2` is installed — booting the P2 off the P2-EC32MB's flash.
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
    expect!(
        "release-named",
        "the file names the embsim release it is written for: this embsim's",
        "an embsim of another release then refuses the file, naming the release it is \
         written for"
    );
    let (project, text) = header_starter("header_starter");
    assert_says(&text, &["# HDR.J1.1 Pin_1 SIG", "# HDR.J1.2 Pin_2 GND"]);
    let release: Vec<&str> = env!("CARGO_PKG_VERSION").split('.').take(2).collect();
    assert_says(
        &text,
        &[&format!("requires-embsim = \"{}\"", release.join("."))],
    );
    assert!(!text.contains("# [[board.model]]"), "{text}");
    let checked = embsim(&["check", path(&project)]);
    assert!(checked.status.success(), "{}", stderr(&checked));
    assert_says(
        &stdout(&checked),
        &["board HDR (netlist): 2 parts: 2 classified", "ok:"],
    );
}

#[rstest]
fn a_check_says_what_its_binary_is_made_of() {
    behaviour!(Test {
        id: "cli.check-provenance",
        covers: Some("cli/src/provenance.rs#lines"),
        given: "the header board's starter project checked by the `embsim` binary, started \
                with the facts a tool measured of a runner's crates in the environment",
    });
    expect!(
        "embsim-and-build",
        "under the project line the check names this embsim's version, the git revision of \
         its sources and where they were, and the compiler, target and profile that built it",
        "a run says what made it (DESIGN.md rule 9)"
    );
    expect!(
        "measured-lines",
        "the facts handed in the environment are printed beside them, as they were given"
    );
    let (project, _) = header_starter("check_provenance");
    let checked = Command::new(env!("CARGO_BIN_EXE_embsim"))
        .args(["check", path(&project)])
        .env(
            embsim_cli::PROVENANCE_ENV,
            "catalog crate rig-catalog 0.1.0: /rig/catalog, git rev 0123456789ab",
        )
        .output()
        .expect("the embsim binary runs");
    assert!(checked.status.success(), "{}", stderr(&checked));
    let text = stdout(&checked);
    assert_says(
        &text,
        &[
            &format!("embsim {}, ", env!("CARGO_PKG_VERSION")),
            &format!(", from {}", workspace().display()),
            "built by rustc ",
            ", profile ",
            "catalog crate rig-catalog 0.1.0: /rig/catalog, git rev 0123456789ab",
        ],
    );
    let lines: Vec<&str> = text.lines().collect();
    let at = lines
        .iter()
        .position(|line| line.starts_with("project "))
        .expect("the project line");
    assert!(lines[at + 2].trim_start().starts_with("embsim "), "{text}");
}

#[rstest]
fn a_check_prints_the_builds_findings_in_plain_words() {
    behaviour!(Test {
        id: "cli.check-findings-in-plain-words",
        covers: Some("cli/src/live.rs#check"),
        given: "the force-gauge add-on's project, whose connector pins and converter inputs \
                nothing on the bench drives, checked from the command line",
    });
    expect!(
        "plain-words",
        "each build finding prints as one line in plain words: the net no source reaches and \
         whether a digital or an analog input reads it",
        "someone reading the check in a terminal or a CI log is told what is wrong on the board"
    );
    expect!(
        "words-and-names",
        "every finding line is made of words and the board's own names"
    );
    let project = workspace().join("boards/projects/ds2-addon.toml");
    let checked = embsim(&["check", path(&project)]);
    assert!(checked.status.success(), "{}", stderr(&checked));
    let text = stdout(&checked);
    assert_says(
        &text,
        &[
            "build findings (11), the system before its first wake:\nno source reaches \
             DS2Addon.GPIO1, which a digital input reads\n",
            "\nno source reaches DS2Addon.AIN0, which an analog input reads\n",
            "ok: ",
        ],
    );
    for rust in ["FloatingSense", "{ net: ", "kind: "] {
        assert!(
            !text.contains(rust),
            "a finding printed in its Rust form ({rust}):\n{text}"
        );
    }
}

#[rstest]
fn version_says_what_the_binary_is_made_of() {
    behaviour!(Test {
        id: "cli.version-provenance",
        covers: Some("cli/src/provenance.rs#long_version"),
        given: "the `embsim` binary asked for its version, short and long",
    });
    expect!(
        "short",
        "the short form is the release and the first twelve digits of its git revision"
    );
    expect!(
        "long",
        "the long form adds where its sources were and the compiler, target and profile that \
         built it"
    );
    let short = stdout(&embsim(&["-V"]));
    assert!(
        short.starts_with(&format!("embsim {}", env!("CARGO_PKG_VERSION"))),
        "{short}"
    );
    let long = stdout(&embsim(&["--version"]));
    assert_says(
        &long,
        &[
            &format!("embsim {}, ", env!("CARGO_PKG_VERSION")),
            "built by rustc ",
            ", for ",
            ", profile ",
        ],
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

/// A project whose one board is `netlist`, with no `[[board.model]]`.
fn bare_project(dir: &Path, netlist: &Path) -> PathBuf {
    let project = dir.join("bare.toml");
    std::fs::write(
        &project,
        format!(
            "[[board]]\nname = \"BARE\"\nkind = \"netlist\"\nnetlist = {:?}\n",
            path(netlist)
        ),
    )
    .expect("the project is writable");
    project
}

#[rstest]
fn check_names_the_pin_table_that_fits_as_the_survey_does() {
    behaviour!(Test {
        id: "cli.check-names-pin-table",
        covers: Some("board/src/project.rs#not_ready_hint"),
        given: "a project of the P2-EC32MB's transcribed netlist alone, no model chosen for any \
                part, checked from the command line",
    });
    expect!(
        "refused",
        "the check fails with a non-zero exit, the board not ready to build"
    );
    expect!(
        "names-each-table",
        "each part placed with the datasheet's numbered pins is named once by its part number, \
         with the function-named table that has the netlist's pins",
        "the catalog places the part by its number with its default table, and the fix is a \
         [[board.model]] that picks the other one"
    );
    expect!(
        "same-as-survey",
        "the check names the same table for the same parts as the survey of the netlist",
        "the survey and the check find the table the same way, so what one says the other says"
    );
    let dir = scratch("check_pin_table");
    let project = bare_project(&dir, &ec32_netlist());
    let checked = embsim(&["check", path(&project)]);
    assert!(!checked.status.success(), "{}", stdout(&checked));
    let error = stderr(&checked);
    assert_says(
        &error,
        &[
            "error: board BARE is not ready to build",
            "give the part a [[board.model]] whose options.pins picks the table with the \
             netlist's pins",
            "U101, U601 mpn \"74LVC2G04GW,125\": pins = \"by-function\" declares the netlist's \
             pins",
            "U501, U502, U503, U504, U505, U506, U507, U508 mpn \"NCP114AMX330TCG\": pins = \
             \"by-function\" declares the netlist's pins",
        ],
    );
    let in_check: Vec<String> = squeezed(&error)
        .lines()
        .filter(|line| line.ends_with(": pins = \"by-function\" declares the netlist's pins"))
        .map(|line| line.trim().to_string())
        .collect();
    assert_eq!(
        in_check.len(),
        7,
        "one fix per model the catalog placed:\n{error}"
    );

    let surveyed = embsim(&["survey", path(&ec32_netlist())]);
    assert!(surveyed.status.success(), "{}", stderr(&surveyed));
    // The survey's group head, `U101, U601  mpn "…": model`, then its fix
    // three lines on: the same as the check's one line.
    let survey = squeezed(&stdout(&surveyed));
    let lines: Vec<&str> = survey
        .lines()
        .skip_while(|line| *line != "placed with a pin table the netlist does not use:")
        .skip(1)
        .take_while(|line| !line.is_empty())
        .collect();
    let in_survey: Vec<String> = lines
        .chunks(4)
        .map(|group| {
            let head = group[0].trim();
            let parts = &head[..head.find(": ").expect("a group head names its model")];
            format!("{parts}: {}", group[3].trim())
        })
        .collect();
    assert_eq!(in_check, in_survey);
}

/// One NCP114 LDO wired by function, one wired with three pins no table
/// of the model has.
const TWO_LDOS: &str = r#"(export (version "E")
  (components
    (comp (ref "U1") (value "LDO")
      (fields (field (name "MPN") "NCP114AMX330TCG")))
    (comp (ref "U2") (value "LDO")
      (fields (field (name "MPN") "NCP114AMX330TCG"))))
  (nets
    (net (code "1") (name "VIN")
      (node (ref "U1") (pin "IN"))
      (node (ref "U2") (pin "1")))
    (net (code "2") (name "VOUT")
      (node (ref "U1") (pin "OUT"))
      (node (ref "U2") (pin "2")))
    (net (code "3") (name "GND")
      (node (ref "U1") (pin "GND"))
      (node (ref "U1") (pin "GND_P"))
      (node (ref "U1") (pin "EN"))
      (node (ref "U2") (pin "3")))))
"#;

#[rstest]
fn check_says_plainly_when_no_pin_table_of_the_model_fits() {
    behaviour!(Test {
        id: "cli.check-no-pin-table",
        covers: Some("board/src/project.rs#not_ready_hint"),
        given: "a board of two LDOs of one part number, one wired by pin function and one with \
                three pins no table of the LDO's model has, checked from the command line",
    });
    expect!(
        "table-for-one",
        "the check names the function-named table for the LDO wired by function"
    );
    expect!(
        "none-for-other",
        "the check says no pin table of the model has the other LDO's pins",
        "a part whose pins are in none of its model's tables takes another model"
    );
    let dir = scratch("check_no_pin_table");
    let netlist = dir.join("ldos.net");
    std::fs::write(&netlist, TWO_LDOS).expect("the netlist is writable");
    let project = bare_project(&dir, &netlist);
    let checked = embsim(&["check", path(&project)]);
    assert!(!checked.status.success(), "{}", stdout(&checked));
    let error = stderr(&checked);
    assert_says(
        &error,
        &[
            "U1 mpn \"NCP114AMX330TCG\": pins = \"by-function\" declares the netlist's pins",
            "U2 mpn \"NCP114AMX330TCG\": no pin table of this model declares the netlist's pins",
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
    expect!(
        "findings-in-plain-words",
        "every finding prints in plain words: an unsourced rail as a power net with no source, \
         an undriven pin as a net no source reaches that a digital input reads",
        "someone reading the report in a terminal or a CI log is told what is wrong on the board"
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
            "findings at build, before any wake (35):\nno source reaches EC32.P2_RESN, which a \
             digital input reads\n",
            "\npower net EC32.Common_VDD has no source\n",
            "findings: 35 (35 at build, 0 while running)",
            "at 10.000000 ms, each finding's net read again:\nno longer true (18):",
            "\npower net EC32.Common_VDD has no source; EC32.Common_VDD now reads Analog(1.81",
            "still true (17):\nno source reaches EC32.P2_IO59, which a digital input reads\n",
        ],
    );
    for rust in ["FloatingSense", "PowerNetUnsourced", "{ net: "] {
        assert!(
            !text.contains(rust),
            "a finding printed in its Rust form ({rust}):\n{text}"
        );
    }
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
fn a_pty_naming_a_file_is_refused_and_the_file_kept() {
    behaviour!(Test {
        id: "cli.run-pty-guard",
        covers: Some("boards/src/catalog.rs#host_serial"),
        given: "a project with one host serial port, run for a millisecond with --pty naming \
                a file of notes, and checked with its own path option naming the project file \
                itself",
    });
    expect!(
        "refused-and-kept",
        "the run and the check each exit non-zero naming the component, the path and that \
         it is not a PTY link, and the notes and the project file are unchanged",
        "a run puts its PTY link only on a free path or over a link an earlier run left"
    );
    let dir = scratch("pty_file");
    let project = dir.join("host.toml");
    std::fs::write(&project, HOST_PROJECT).expect("the project is writable");
    let notes = dir.join("notes.txt");
    std::fs::write(&notes, "precious notes").expect("the notes are writable");
    let run = embsim(&["run", path(&project), "--for", "1ms", "--pty", path(&notes)]);
    assert!(!run.status.success(), "{}", stdout(&run));
    assert_says(
        &stderr(&run),
        &[&format!(
            "component HOST (kind \"host-serial\"): {} exists and is not a PTY link; name a \
             free path",
            notes.display()
        )],
    );
    assert_eq!(
        std::fs::read_to_string(&notes).expect("the notes are there"),
        "precious notes"
    );

    let itself = dir.join("itself.toml");
    let text = HOST_PROJECT.replace("baud = 115200", "baud = 115200\npath = \"itself.toml\"");
    std::fs::write(&itself, &text).expect("the project is writable");
    let check = embsim(&["check", path(&itself)]);
    assert!(!check.status.success(), "{}", stdout(&check));
    assert_says(&stderr(&check), &["exists and is not a PTY link"]);
    assert_eq!(
        std::fs::read_to_string(&itself).expect("the project is there"),
        text
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
// The boot needs a `qemu-system-p2` (`embsim qemu install`), so it is
// `#[ignore]`d here and run by CI's `p2-qemu-boot` job, as
// `rom_boot_ec32mb.rs` is, and declares no behaviour: the ledger's run
// installs no QEMU. Without the program the entry is refused, saying how to
// install it, which runs everywhere: the command is started with nothing to
// find.

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

/// `mov pa,#"B"` / `wypin pa,#62` / `jmp #$`: write `B` to the debug pin,
/// and stay.
const B_PROGRAM: [u32; 3] = [0xF607_EC42, 0xFC27_EC3E, 0xFD9F_FFFC];

/// The project, and its flash image laid out by `embsim flash-image` from
/// [`B_PROGRAM`], in a directory of the test's own.
fn boot_project(test: &str) -> PathBuf {
    let dir = scratch(test);
    let program: Vec<u8> = B_PROGRAM
        .iter()
        .flat_map(|word| word.to_le_bytes())
        .collect();
    std::fs::write(dir.join("b.binary"), program).expect("the program is writable");
    let made = embsim(&[
        "flash-image",
        path(&dir.join("b.binary")),
        "-o",
        path(&dir.join("boot.bin")),
    ]);
    assert!(made.status.success(), "{}", stderr(&made));
    let project = dir.join("boot.toml");
    std::fs::write(&project, PROJECT).expect("the project is writable");
    project
}

#[rstest]
fn flash_image_lays_out_a_program_the_p2s_boot_rom_boots() {
    behaviour!(Test {
        id: "cli.flash-image",
        covers: Some("cli/src/qemu.rs#flash_image"),
        given: "`embsim flash-image` for a three-instruction P2 program, writing the image to a \
                file",
    });
    expect!(
        "layout",
        "the image is embsim's stage-1 loader in its first kilobyte, then the program's \
         length in bytes and the program itself, from byte $400",
        "stage-1 reads the length at $400 and copies the program behind it into hub RAM"
    );
    expect!(
        "sums-to-prop",
        "the first kilobyte's 256 little-endian longs sum to the word Prop",
        "the boot ROM runs the first kilobyte of the flash only when they do"
    );
    expect!(
        "said",
        "the command says where it wrote the image, what lies at $000, $400 and $404, and \
         that a w25q128jv part's image option names the file"
    );
    let dir = scratch("flash_image");
    let program: Vec<u8> = B_PROGRAM
        .iter()
        .flat_map(|word| word.to_le_bytes())
        .collect();
    std::fs::write(dir.join("b.binary"), &program).expect("writable");
    let image_path = dir.join("boot.bin");
    let output = embsim(&[
        "flash-image",
        path(&dir.join("b.binary")),
        "-o",
        path(&image_path),
    ]);
    assert!(output.status.success(), "{}", stderr(&output));
    let image = std::fs::read(&image_path).expect("the image is there");
    let stage1 = embsim_p2_qemu::STAGE1;
    assert_eq!(&image[..stage1.len()], stage1);
    assert_eq!(&image[0x400..0x404], &12u32.to_le_bytes());
    assert_eq!(&image[0x404..], &program[..]);
    let sum = image[..0x400]
        .as_chunks::<4>()
        .0
        .iter()
        .map(|long| u32::from_le_bytes(*long))
        .fold(0u32, u32::wrapping_add);
    assert_eq!(sum, u32::from_le_bytes(*b"Prop"));
    assert_says(
        &stdout(&output),
        &[
            &format!(
                "wrote {}: {} bytes, a flash image the P2's boot ROM boots",
                image_path.display(),
                image.len()
            ),
            &format!(
                "$000 embsim's stage-1 loader ({} bytes), its first kilobyte summing to \"Prop\"",
                stage1.len()
            ),
            "$400 the program's length, 12 bytes",
            "$404 ",
            "a w25q128jv part's `image` option names it, relative to the project file",
        ],
    );
}

#[rstest]
fn a_run_off_a_flash_image_a_p2_does_not_boot_says_so_at_its_first_look() {
    behaviour!(Test {
        id: "cli.run-flash-not-bootable",
        covers: Some("boards/src/catalog.rs#w25q128jv_kind"),
        given: "the P2-EC32MB powered from its carrier fingers, its processor held in reset, \
                and its boot flash holding a raw three-instruction P2 program with no stage-1 \
                loader in front of it, run for a millisecond",
    });
    expect!(
        "said-at-start",
        "the run prints at 0 milliseconds, under the flash's name, that a P2 does not boot \
         from the image and that `embsim flash-image` lays out one it does",
        "the boot ROM refuses a first kilobyte that does not sum to Prop, and every cog stops \
         with nothing on the board to say why"
    );
    let dir = scratch("flash_not_bootable");
    let program: Vec<u8> = B_PROGRAM
        .iter()
        .flat_map(|word| word.to_le_bytes())
        .collect();
    std::fs::write(dir.join("boot.bin"), program).expect("writable");
    let project = dir.join("raw.toml");
    std::fs::write(
        &project,
        PROJECT.replace("core = \"qemu\"", "core = \"held-in-reset\""),
    )
    .expect("writable");
    let output = embsim(&["run", path(&project), "--for", "1ms"]);
    assert!(output.status.success(), "{}", stderr(&output));
    assert_says(
        &stdout(&output),
        &[
            "[ 0.000000 ms] EC32.U301: flash image \"boot.bin\": a P2 does not boot from it.",
            "`embsim flash-image PROGRAM -o IMAGE` lays out a P2 program behind a stage-1 \
             loader that does",
        ],
    );
}

#[rstest]
#[case::missing(None, "cannot read the program")]
#[case::empty(Some(0), "is empty; it is the P2 binary the compiler wrote")]
#[case::too_big(Some(512 * 1024 + 1), "is 524289 bytes, and stage-1 copies it into the P2's hub RAM, which holds 524288")]
fn flash_image_refuses_a_program_stage_1_cannot_load(
    #[case] len: Option<usize>,
    #[case] says: &str,
) {
    behaviour!(Test {
        id: "cli.flash-image-refused",
        covers: Some("cli/src/qemu.rs#flash_image"),
        given: "`embsim flash-image` for a program file that is not there, one that is empty, \
                and one a byte larger than the P2's 512 kilobytes of hub RAM",
    });
    expect!(
        "refused",
        "each is refused, saying which and why, and no image is written",
        "stage-1 copies the program into hub RAM from address zero, so an image of nothing \
         or of more than the hub holds would never run"
    );
    let dir = scratch(&format!(
        "flash_image_refused_{}",
        len.map_or(-1, |len| len as i64)
    ));
    let program = dir.join("p.binary");
    if let Some(len) = len {
        std::fs::write(&program, vec![0u8; len]).expect("writable");
    }
    let image = dir.join("boot.bin");
    let output = embsim(&["flash-image", path(&program), "-o", path(&image)]);
    assert!(!output.status.success());
    assert_says(&stderr(&output), &[says]);
    assert!(!image.exists());
}

#[test]
#[ignore = "needs qemu-system-p2 (embsim qemu install); CI's p2-qemu-boot job runs it"]
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
    // Which program ran the core, said at the first look.
    assert!(text.contains("EC32.U100: qemu-system-p2 "), "{text}");
    assert!(text.contains("console P62 \"B\""), "{text}");
    assert!(text.contains("ran 20.000000 ms of virtual time"), "{text}");
}

#[test]
#[ignore = "needs qemu-system-p2 (embsim qemu install); CI's p2-qemu-boot job runs it"]
fn run_stops_when_its_qemu_core_dies() {
    // The boot project with a guest that toggles P0 for ever in place of
    // the boot ROM (`rom`), run for a minute; its qemu-system-p2 is killed
    // once it runs. The core's report says the program died, the run stops
    // at that look with its summary, and the command exits non-zero naming
    // the part (`failure.rs` proves the run's half on any machine).
    let project = boot_project("qemu_dies");
    let dir = project.parent().expect("a directory").to_path_buf();
    // drvnot #0 / jmp #\0
    let toggle: Vec<u8> = [0xFD64_005Fu32, 0xFD80_0000]
        .iter()
        .flat_map(|word| word.to_le_bytes())
        .collect();
    std::fs::write(dir.join("toggle.bin"), toggle).expect("writable");
    let text = std::fs::read_to_string(&project)
        .expect("the project reads")
        .replace(
            "core = \"qemu\"\n",
            "core = \"qemu\"\nrom = \"toggle.bin\"\n",
        );
    std::fs::write(&project, text).expect("writable");
    let child = Command::new(env!("CARGO_BIN_EXE_embsim"))
        .args(["run", path(&project), "--for", "60s"])
        .stdout(std::process::Stdio::piped())
        .stderr(std::process::Stdio::piped())
        .spawn()
        .expect("the embsim binary runs");
    // The program is the run's child, in a process group of its own.
    let started = std::time::Instant::now();
    let program = loop {
        let found = Command::new("pgrep")
            .args(["-P", &child.id().to_string(), "qemu-system-p2"])
            .output()
            .expect("pgrep runs");
        if let Some(pid) = String::from_utf8_lossy(&found.stdout)
            .lines()
            .next()
            .and_then(|line| line.trim().parse::<i32>().ok())
        {
            break pid;
        }
        assert!(
            started.elapsed() < std::time::Duration::from_secs(60),
            "no qemu-system-p2 under the run"
        );
        std::thread::sleep(std::time::Duration::from_millis(20));
    };
    std::thread::sleep(std::time::Duration::from_millis(500));
    // SAFETY: the run's own child, ended the way `kill -9` ends it.
    assert_eq!(unsafe { libc::kill(program, libc::SIGKILL) }, 0);
    let output = child.wait_with_output().expect("the run ends");
    let text = stdout(&output);
    assert!(!output.status.success(), "{text}");
    assert_says(
        &text,
        &[
            "EC32.U100: QEMU stopped: qemu-system-p2 (",
            &format!("pid {program}) was killed by signal 9 (SIGKILL) during a run"),
            "of virtual time: EC32.U100 failed",
        ],
    );
    assert!(!text.contains("ran 60000.000000 ms"), "{text}");
    assert_says(
        &stderr(&output),
        &[
            "error: EC32.U100 failed at ",
            "and the run stopped there: qemu-system-p2 (",
        ],
    );
}

/// `embsim` with `args`, started where there is no `qemu-system-p2` to
/// find: the variable unset, a `PATH` of an empty directory, and a home
/// with nothing installed.
fn embsim_without_qemu(test: &str, args: &[&str]) -> (Output, PathBuf) {
    let home = scratch(test);
    let empty = home.join("bin");
    std::fs::create_dir_all(&empty).expect("an empty PATH");
    let output = Command::new(env!("CARGO_BIN_EXE_embsim"))
        .args(args)
        .env_remove("EMBSIM_QEMU_SYSTEM_P2")
        .env("PATH", &empty)
        .env("HOME", &home)
        .output()
        .expect("the embsim binary runs");
    (output, home)
}

#[rstest]
fn without_qemu_system_p2_a_qemu_core_is_refused_saying_how_to_install_it() {
    behaviour!(Test {
        id: "cli.qemu-not-installed",
        covers: Some("p2-qemu/src/catalog.rs#QemuCores::seat"),
        given: "a project whose P2 runs on QEMU, checked where no qemu-system-p2 is installed, \
                named or on the PATH",
    });
    expect!(
        "where-it-looked",
        "the check fails at the P2's entry, naming each place it looked for qemu-system-p2"
    );
    expect!(
        "how-to-install",
        "the error says `embsim qemu install` builds and installs it, and the directory it goes \
         to",
        "the program is built from the P2 target this embsim carries, so the install directory \
         is named by that target"
    );
    let project = boot_project("qemu_refused_project");
    let (output, home) =
        embsim_without_qemu("qemu_refused", &["check", project.to_str().expect("text")]);
    assert!(!output.status.success());
    let error = stderr(&output);
    let install_dir = home
        .join(".embsim/qemu")
        .join(embsim_p2_qemu::target::identity());
    assert_says(
        &error,
        &[
            "board EC32: [[board.model]] value = \"P2X8C4M64P\" (kind \"p2\"): no \
             qemu-system-p2: EMBSIM_QEMU_SYSTEM_P2 is unset, none is on PATH, and none at",
            &install_dir.join("qemu-system-p2").display().to_string(),
            "`embsim qemu install` builds it",
        ],
    );
}

#[rstest]
fn qemu_path_says_what_this_embsim_needs_and_where_it_looked() {
    behaviour!(Test {
        id: "cli.qemu-path-none",
        covers: Some("cli/src/qemu.rs#path"),
        given: "`embsim qemu path` where no qemu-system-p2 is installed, named or on the PATH",
    });
    expect!(
        "needs",
        "it prints the protocol, P2 target and QEMU release this embsim needs"
    );
    expect!(
        "fails-saying-how",
        "it exits non-zero, saying where it looked and how to install the program"
    );
    let (output, _) = embsim_without_qemu("qemu_path_none", &["qemu", "path"]);
    assert!(!output.status.success());
    let pin = embsim_p2_qemu::target::qemu_pin();
    assert_says(
        &stdout(&output),
        &[&format!(
            "this embsim needs: protocol {}, P2 target {}, QEMU {}",
            embsim_p2_qemu::protocol::PROTOCOL,
            embsim_p2_qemu::target::identity(),
            pin.version()
        )],
    );
    assert_says(
        &stderr(&output),
        &["error: no qemu-system-p2:", "`embsim qemu install`"],
    );
}

#[rstest]
fn qemu_install_dry_run_names_the_release_the_target_and_where_it_goes() {
    behaviour!(Test {
        id: "cli.qemu-install-plan",
        covers: Some("p2-qemu/src/install.rs#Plan::describe"),
        given: "`embsim qemu install --dry-run` with a directory to install into",
    });
    expect!(
        "plan",
        "it prints the P2 target it would build, the QEMU tag and commit it would fetch, the \
         configure line, and the program's path in that directory, and builds nothing"
    );
    let dir = scratch("qemu_install_plan");
    let prefix = dir.join("qemu");
    let output = embsim(&[
        "qemu",
        "install",
        "--dry-run",
        "--prefix",
        prefix.to_str().expect("text"),
    ]);
    assert!(output.status.success(), "{}", stderr(&output));
    let pin = embsim_p2_qemu::target::qemu_pin();
    assert_says(
        &stdout(&output),
        &[
            &format!("target {}", embsim_p2_qemu::target::identity()),
            &format!(
                "qemu {} {} from https://gitlab.com/qemu-project/qemu.git",
                pin.tag, pin.commit
            ),
            "configure --target-list=p2-softmmu --without-default-features",
            &format!("install {}", prefix.join("qemu-system-p2").display()),
        ],
    );
    assert!(!prefix.exists(), "a dry run installs nothing");
}

#[test]
#[ignore = "needs qemu-system-p2 (embsim qemu install); CI's p2-qemu-boot job runs it"]
fn qemu_path_finds_the_installed_program_and_says_it_is_the_one() {
    let output = embsim(&["qemu", "path"]);
    let text = stdout(&output);
    assert!(output.status.success(), "{}\n{text}", stderr(&output));
    assert_says(
        &text,
        &["qemu-system-p2: ", "ok: the one this embsim needs"],
    );
}
