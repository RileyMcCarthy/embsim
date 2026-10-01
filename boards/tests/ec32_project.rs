//! The P2-EC32MB as a project, built only (no engine, no clock).
//!
//! * `ec32-netlist.toml` builds the module from its netlist through the
//!   catalog's base registry and the file's `[[board.model]]` entries, and
//!   the board it builds is the one `Ec32mb` builds with the same processor:
//!   the same parts in the same classes, the same components, the same nets,
//!   the same conduction clusters, and every part the same model with the
//!   same options. `Ec32mb` registers the catalog's own model constructors,
//!   so the two paths share every model; what the comparison checks is that
//!   the file chooses each the way the module does — its pin table, the
//!   flash's ID — and that the switch poles it writes out are the module's.
//! * The survey of that netlist with nothing assigned is the checklist the
//!   file answers: the processor, the switch and the BOM lines with no model,
//!   and every part placed by number listed against the pins the transcribed
//!   netlist gives it.
//! * `ec32-carrier.toml` builds the catalog's `p2-ec32mb` kind with its
//!   processor slot filled and two DIP-switch positions closed, and exactly
//!   those two poles join their nets.
//!
//! The same project's power tree, live, is `ec32_project_power.rs`.

use std::collections::BTreeSet;
use std::path::PathBuf;

use embsim_board::{netlist, Board, BoardSurvey, BuiltSystem, Project, System};
use embsim_boards::catalog::StandardCatalog;
use embsim_boards::ec32mb::{self, Ec32mb, P2_PART};
use embsim_boards::p2::P2Package;
use rstest::rstest;
use vibes_behaviour::{behaviour, expect, Test};

fn project(file: &str) -> Project {
    let path: PathBuf = [env!("CARGO_MANIFEST_DIR"), "projects", file]
        .iter()
        .collect();
    Project::load(&path).unwrap_or_else(|err| panic!("{file}: {err}"))
}

/// The module as `Ec32mb` builds it with a P2 package held in reset in its
/// processor slot — the configuration `ec32-netlist.toml` names.
fn shipped() -> Board {
    Ec32mb::new()
        .with_p2(|_decl| Box::new(P2Package::held_in_reset()))
        .build()
        .expect("the module builds")
}

/// The conduction clusters of a built board, each as the sorted names of
/// its nets, sorted: a partition that compares by content.
fn clusters(built: &BuiltSystem) -> Vec<Vec<String>> {
    let mut clusters: Vec<Vec<String>> = built
        .cluster_roots()
        .iter()
        .map(|roots| {
            let mut names: Vec<String> = roots
                .iter()
                .map(|id| built.nets()[id.0].name.clone())
                .collect();
            names.sort();
            names
        })
        .collect();
    clusters.sort();
    clusters
}

/// The P2-EC32MB's census (`board/tests/cluster_census.rs`, the `EC32MB`
/// fixture): 87 conduction clusters, the largest of 6 nodes.
const EC32MB_CLUSTERS: usize = 87;
const EC32MB_LARGEST_CLUSTER: usize = 6;

#[rstest]
fn the_ec32_built_from_its_netlist_is_the_board_ec32mb_builds() {
    behaviour!(Test {
        id: "project.ec32-netlist-equals-the-module",
        covers: Some("board/src/project.rs#Project::build_board"),
        given: "the P2-EC32MB built from its vendor netlist by a project that models every \
                part its checklist names, beside the board library's module, both with a \
                processor package running no core",
    });
    expect!(
        "same-parts-same-classes",
        "every one of the 114 parts has the same class on both boards, down to each model's \
         pin table and each switch's poles",
        "both are built by one board constructor from one netlist, so a difference could only \
         come from a model the project placed differently"
    );
    expect!(
        "same-components",
        "the same parts are live models on both boards, in the same order"
    );
    expect!(
        "same-nets",
        "every net has the same name and the same member pins on both boards"
    );
    expect!(
        "same-models",
        "every part but the processor has the same model on both boards, named with its options: \
         its pin table, and the flash's device ID",
        "the module registers the catalog's own models, so a part a project models differently \
         is a project that chose another option"
    );
    expect!(
        "same-census",
        "both boards have the same conduction clusters, 87 of them, the largest joining 6 \
         nodes — the module's committed census",
        "a cluster is what the resistors, switches and regulators join, so an equal partition \
         says the two boards' models join the same nets into the same circuits"
    );
    let project = project("ec32-netlist.toml");
    let from_project = project
        .build_board(&StandardCatalog, "EC32")
        .expect("the project builds the module");
    let from_library = shipped();

    // Each part's model as its registration names it, options included,
    // from the surveys the two registries make of the one netlist. The
    // processor's slot is filled by a constructor the module cannot name.
    let models = |survey: &BoardSurvey| {
        survey
            .parts()
            .filter(|part| part.value != P2_PART)
            .map(|part| (part.reference.clone(), part.model.clone()))
            .collect::<Vec<_>>()
    };
    let project_survey = project
        .survey(&StandardCatalog, "EC32")
        .expect("the project surveys");
    let library_survey = BoardSurvey::of(
        &netlist::parse(ec32mb::NETLIST).expect("the bundled netlist parses"),
        &Ec32mb::new()
            .with_p2(|_decl| Box::new(P2Package::held_in_reset()))
            .registry(),
    );
    let (project_models, library_models) = (models(&project_survey), models(&library_survey));
    for (a, b) in project_models.iter().zip(&library_models) {
        assert_eq!(a, b, "{} is another model on the two paths", a.0);
    }
    assert_eq!(project_models, library_models);
    let modelled = project_models
        .iter()
        .filter(|(_, model)| model.is_some())
        .count();
    // The TCXO, the two inverters, the four PSRAMs, the flash, the two
    // bucks, the eight LDOs and the detector.
    assert_eq!(modelled, 19, "{project_models:?}");

    let classes = |board: &Board| {
        board
            .nodes()
            .map(|(reference, class)| (reference.to_string(), class.clone()))
            .collect::<Vec<_>>()
    };
    let (project_classes, library_classes) = (classes(&from_project), classes(&from_library));
    assert_eq!(project_classes.len(), 114, "every part is a node");
    for (a, b) in project_classes.iter().zip(&library_classes) {
        assert_eq!(a, b, "{} differs between the two paths", a.0);
    }
    assert_eq!(project_classes, library_classes);

    let refs = |board: &Board| {
        board
            .component_refs()
            .map(str::to_string)
            .collect::<Vec<_>>()
    };
    assert_eq!(refs(&from_project), refs(&from_library));

    let nets = |board: &Board| {
        board
            .nets()
            .iter()
            .map(|net| (net.name.clone(), net.nodes.clone()))
            .collect::<Vec<_>>()
    };
    assert_eq!(nets(&from_project), nets(&from_library));

    let built_project = System::new()
        .board("EC32", from_project)
        .build()
        .expect("the project's board builds bare");
    let built_library = System::new()
        .board("EC32", from_library)
        .build()
        .expect("the module builds bare");
    let (project_clusters, library_clusters) = (clusters(&built_project), clusters(&built_library));
    assert_eq!(project_clusters.len(), EC32MB_CLUSTERS);
    assert_eq!(
        project_clusters.iter().map(Vec::len).max(),
        Some(EC32MB_LARGEST_CLUSTER)
    );
    assert_eq!(project_clusters, library_clusters);
}

/// What the netlist asks of a project before anything is assigned.
#[rstest]
fn the_ec32_netlist_alone_asks_for_its_processor_its_switch_and_its_pin_tables() {
    behaviour!(Test {
        id: "project.ec32-netlist-checklist",
        covers: Some("board/src/survey.rs#BoardSurvey::of"),
        given: "the P2-EC32MB's transcribed netlist as a project board with no model assigned, \
                surveyed with every model the catalog places by manufacturer part number",
    });
    expect!(
        "needs-a-model",
        "the parts named as needing a model are exactly the processor, the DIP switch and \
         the two bill-of-materials lines with nothing electrical",
        "the catalog places a part by its number only where the number says everything: a \
         processor's core and a switch's poles are the project's to say"
    );
    expect!(
        "pin-tables-listed",
        "each of the 19 parts placed by number is listed with the datasheet's numbered pins \
         beside the function names the netlist gives them",
        "the transcription names pins by function, and a model's default table is the \
         numbered one an EDA export uses"
    );
    expect!(
        "connectors-listed",
        "the card-edge fingers, the card socket, the solder link and the two mounting holes \
         are listed as connectors",
        "a part with no symbol name is classified by its reference designator, and all five \
         are drawn with a J"
    );
    let netlists: PathBuf = [env!("CARGO_MANIFEST_DIR"), "netlists"].iter().collect();
    let project = Project::parse(
        "[[board]]\nname = \"EC32\"\nkind = \"netlist\"\nnetlist = \"p2_ec32mb.net\"\n",
    )
    .expect("the text is a project")
    .relative_to(netlists);
    let survey = project
        .survey(&StandardCatalog, "EC32")
        .expect("the board surveys");
    let needs: Vec<&str> = survey
        .needs_model
        .iter()
        .map(|part| part.reference.as_str())
        .collect();
    assert_eq!(needs, ["NC_Net", "PCB", "S301", "U100"]);
    assert!(!survey.compliant());

    let mismatched: BTreeSet<&str> = survey
        .mismatched
        .iter()
        .map(|part| part.reference.as_str())
        .collect();
    let expected: BTreeSet<&str> = [
        "U101", "U601", "X100", "U301", "U302", "U303", "U304", "U305", "U402", "U403", "U404",
        "U501", "U502", "U503", "U504", "U505", "U506", "U507", "U508",
    ]
    .into_iter()
    .collect();
    assert_eq!(mismatched, expected);
    let u101 = survey
        .mismatched
        .iter()
        .find(|part| part.reference == "U101")
        .expect("U101 is listed");
    assert_eq!(u101.declared, ["1", "2", "3", "4", "5", "6"]);
    assert_eq!(u101.netlist, ["1A", "1Y", "2A", "2Y", "GND", "VCC"]);

    let connectors: Vec<&str> = survey
        .connectors
        .iter()
        .map(|conn| conn.reference.as_str())
        .collect();
    assert_eq!(connectors, ["J101", "J203", "J301", "J701", "J702"]);
}

/// The catalog board kind, its slot filled by the file, two switch
/// positions closed.
#[rstest]
fn the_carrier_project_closes_the_two_switch_positions_it_names() {
    behaviour!(Test {
        id: "project.ec32-carrier-switch-positions",
        covers: Some("board/src/project.rs#Project::instantiate"),
        given: "the P2-EC32MB from the board library, its processor slot given a package that \
                runs no core, with the project closing option-switch positions 2 (flash) and \
                4 (the pull-down that boots from flash)",
    });
    expect!(
        "flash-select-joined",
        "the processor pin that selects the flash is joined to the flash's chip select",
        "position 2 is the flash-select contact, pole 1 in the switch's order"
    );
    expect!(
        "boot-pull-down-joined",
        "the pull-down resistor is joined to the boot-mode pin",
        "position 4 is the boot pull-down contact, pole 3"
    );
    expect!(
        "others-open",
        "the LED supply and the boot pull-up stay apart from what their open positions would \
         join"
    );
    let built = project("ec32-carrier.toml")
        .instantiate(&StandardCatalog)
        .expect("the carrier project builds")
        .build()
        .expect("the system builds");
    for net in [
        "EC32.P2_IO61",
        "EC32.SPI_CS",
        "EC32.P2_IO59",
        "EC32.Net-(S301-4_OFF)",
        "EC32.Common_LDOin",
        "EC32.DIPSW_LEDPWR",
        "EC32.Net-(S301-3_OFF)",
    ] {
        assert!(built.net_id(net).is_some(), "{net} is a net");
    }
    assert!(built.names_are_merged("EC32.P2_IO61", "EC32.SPI_CS"));
    assert!(built.names_are_merged("EC32.P2_IO59", "EC32.Net-(S301-4_OFF)"));
    assert!(!built.names_are_merged("EC32.Common_LDOin", "EC32.DIPSW_LEDPWR"));
    assert!(!built.names_are_merged("EC32.P2_IO59", "EC32.Net-(S301-3_OFF)"));
}
