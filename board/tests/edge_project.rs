//! The MaD machine's three boards as a project, built only: the Edge
//! carrier, the P2-EC32MB seated in its socket and the DS2 add-on on its
//! force cable, all said by `boards/projects/edge-ec32-ds2.toml`.
//!
//! The file builds with the standard catalog alone: every part it names is
//! the catalog's, the Edge board's AM26LS31 line driver `U24` and AM26LV32
//! line receiver `U25` included, each placed by its part number (`am26ls31`,
//! `am26lv32`).
//!
//! What it holds the file to is the hand-written harnesses the machine
//! tests assemble the three boards with (`machine_parts`): the socket's
//! `[[mate]]` joins every finger `module_socket_harness` wires, and the
//! cable's `map` every pin `force_gauge_harness` wires, and nothing joins
//! where they leave a contact open. The same system live is
//! `edge_project_live.rs`.

mod machine_parts;

use std::collections::HashMap;
use std::path::PathBuf;

use embsim_board::{BuiltSystem, EndpointRef, PinRef, Project};
use embsim_boards::catalog::StandardCatalog;
use embsim_core::virtual_clock::{self, ClockMode};
use rstest::rstest;
use vibes_behaviour::{behaviour, expect, Test};

use machine_parts::{edge_fingers, force_gauge_harness, module_socket_harness};

fn project() -> Project {
    let path: PathBuf = [
        env!("CARGO_MANIFEST_DIR"),
        "..",
        "boards",
        "projects",
        "edge-ec32-ds2.toml",
    ]
    .iter()
    .collect();
    Project::load(&path).expect("the project loads")
}

/// Each board pin's net, by `(board, pin)`.
fn nets_of_pins(system: &BuiltSystem) -> HashMap<(String, PinRef), String> {
    let mut map = HashMap::new();
    for net in system.nets() {
        let (board, _) = net.name.split_once('.').unwrap_or((net.name.as_str(), ""));
        for node in &net.nodes {
            map.insert((board.to_string(), node.clone()), net.name.clone());
        }
    }
    map
}

fn net_of(map: &HashMap<(String, PinRef), String>, end: &EndpointRef) -> String {
    let connector = end
        .connector
        .as_deref()
        .expect("a board end names its connector");
    map.get(&(end.board.clone(), PinRef::new(connector, &end.pin)))
        .cloned()
        .unwrap_or_else(|| panic!("{}.{connector}.{} is on a net", end.board, end.pin))
}

#[rstest]
fn the_three_board_projects_mates_join_what_the_machines_harnesses_join() {
    behaviour!(Test {
        id: "project.edge-ec32-ds2-mates",
        covers: Some("board/src/project.rs#Project::instantiate"),
        given: "the MaD machine's three boards in one project file: the P2 module mated into the \
                carrier's socket by finger, the force-gauge add-on mated to the force cable by a \
                crossed map",
    });
    expect!(
        "socket-fingers",
        "every finger the hand-written socket harness wires is one node with the socket \
         contact of its number, and so are the module's two no-connect fingers",
        "both netlists number the card edge by finger, and a seated card touches every \
         contact it has a finger for"
    );
    expect!(
        "empty-contacts-open",
        "the twenty socket contacts the module has no finger for are joined to nothing on the \
         module"
    );
    expect!(
        "cable-crossed",
        "each cable pin joins the add-on pin the hand-written cable harness joins it to, so the \
         carrier's transmit line reaches the converter's receive side",
        "the cable crosses transmit and receive, and its map says so pin by pin"
    );
    expect!(
        "shield-open",
        "the cable's sixth pin, on the carrier's shield net, joins nothing on the add-on"
    );
    // Building attaches the converter, whose protocol thread joins the clock.
    virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);
    let built = project()
        .instantiate(&StandardCatalog)
        .expect("the project builds with the standard catalog alone")
        .build()
        .expect("the system builds");
    let map = nets_of_pins(&built);
    let merged = |a: &EndpointRef, b: &EndpointRef| {
        built.names_are_merged(&net_of(&map, a), &net_of(&map, b))
    };
    let ep = |text: String| EndpointRef::parse(&text).expect("endpoint parses");

    // The socket: what the machine's harness wires, and the module's two
    // no-connect fingers beside it.
    let socket = module_socket_harness("EC32", "EDGE");
    for wire in socket.connections() {
        assert!(
            merged(&wire.from, &wire.to),
            "{:?} to {:?}",
            wire.from,
            wire.to
        );
    }
    assert_eq!(socket.connections().len(), 58);
    for finger in [1, 2] {
        assert!(merged(
            &ep(format!("EC32.J203.{finger}")),
            &ep(format!("EDGE.J3.{finger}"))
        ));
    }
    let fingers: Vec<u32> = edge_fingers().chain([1, 2]).collect();
    let module_nets: Vec<String> = built
        .nets()
        .iter()
        .filter(|net| net.name.starts_with("EC32."))
        .map(|net| net.name.clone())
        .collect();
    for contact in (1..=80u32).filter(|contact| !fingers.contains(contact)) {
        let edge = net_of(&map, &ep(format!("EDGE.J3.{contact}")));
        for module in &module_nets {
            assert!(
                !built.names_are_merged(&edge, module),
                "socket contact {contact} ({edge}) joins {module}"
            );
        }
    }

    // The cable.
    let cable = force_gauge_harness("EDGE", "DS2");
    for wire in cable.connections() {
        assert!(
            merged(&wire.from, &wire.to),
            "{:?} to {:?}",
            wire.from,
            wire.to
        );
    }
    assert_eq!(cable.connections().len(), 5);
    assert!(merged(&ep("EDGE.J9.4".into()), &ep("DS2.J1.3".into())));
    let shield = net_of(&map, &ep("EDGE.J9.6".into()));
    for pin in 1..=5 {
        let addon = net_of(&map, &ep(format!("DS2.J1.{pin}")));
        assert!(
            !built.names_are_merged(&shield, &addon),
            "J9.6 joins {addon}"
        );
    }
}

#[rstest]
fn the_edge_boards_line_parts_are_the_standard_catalogs() {
    behaviour!(Test {
        id: "project.edge-line-driver-from-catalog",
        covers: Some("boards/src/catalog.rs#StandardCatalog::base_registry"),
        given: "the MaD Edge carrier of the three-board project file, surveyed with the \
                standard catalog alone",
    });
    expect!(
        "placed-by-number",
        "the RS-422 line driver U24 is placed by its part number AM26LS31CD, as the \
         catalog's am26ls31 kind with its numbered pin table",
        "the catalog ships the AM26LS31 as a model of its datasheet, placed by the ordering \
         codes that datasheet lists"
    );
    expect!(
        "receiver-left",
        "U25 is placed by its part number AM26LV32IDR as the catalog's am26lv32 kind, and no \
         part of the board needs a model",
        "U25's part number now names the part its LCSC code says was bought, and the catalog \
         ships the AM26LV32 placed by the ordering codes its datasheet lists"
    );
    let survey = project()
        .survey(&StandardCatalog, "EDGE")
        .expect("the carrier surveys");
    let model = |reference: &str| {
        let part = survey
            .parts()
            .find(|part| part.reference == reference)
            .unwrap_or_else(|| panic!("the carrier has a {reference}"));
        (part.key.clone(), part.model.clone())
    };
    assert_eq!(
        model("U24"),
        (
            Some("AM26LS31CD".to_string()),
            Some("am26ls31, pins = \"numbered\"".to_string())
        )
    );
    assert_eq!(
        model("U25"),
        (
            Some("AM26LV32IDR".to_string()),
            Some("am26lv32, pins = \"numbered\"".to_string())
        )
    );
    let unplaced: Vec<&str> = survey
        .needs_model
        .iter()
        .map(|part| part.reference.as_str())
        .collect();
    assert!(unplaced.is_empty(), "{unplaced:?}");
}
