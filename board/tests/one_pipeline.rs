//! One pipeline (`DESIGN.md` rule 1): a netlist part is a node whose class
//! has behaviour, or the board refuses to build and names the part and its
//! value. No stub list, no ignored tier, no allow-list.
//!
//! The proof list of `NODES.md` §8 phase 1, on the three reference boards:
//!
//! - every passive's value field parses, first token first — 66 of the
//!   EC32MB's 66 capacitors, both inductors, all fifteen resistors;
//! - the BOM-only lines and the mounting holes are **mechanical nodes**
//!   (`PCB`, `NC_Net`, `J701`, `J702` on the module; `H5`–`H8` on the Edge
//!   board), parts the board carries with nothing electrical;
//! - the DIP switch `S301` is a four-pole **switch** whose poles a scenario
//!   closes by index, with identity-union semantics: a closed pole makes
//!   its two nets one node, honouring a detached contact; the solder link
//!   `J101` is a one-pole switch a `jumper` call also closes; the Edge
//!   board's reset button is a switch the auto tier classifies;
//! - a part the registry cannot classify fails the build naming the
//!   reference, the part and the value.
//!
//! What a closed pole *does* to the P59 boot strap is read here from the
//! resolved net — pulled high through position 3, pulled low through
//! position 4 — because that is the observable the ROM boot depends on.

mod machine_parts;

use embsim_board::registry::parse_passive_value;
use embsim_board::{
    Board, BoardError, DnpState, EndpointRef, Finding, Harness, JumperState, Level, NetState,
    PartClass, PartRegistry, PwlSpec, Scenario, SenseKind, System, SystemError,
};
use embsim_boards::ec32mb::{
    FLASH_SELECT_POLE, FLASH_SELECT_SWITCH, P59_PULL_DOWN_POLE, P59_PULL_UP_POLE,
};
use machine_parts::{edge_board, shipped_ec32mb_board};
use rstest::rstest;
use vibes_behaviour::{behaviour, expect, Test};

const EC32MB_NETLIST: &str = include_str!("fixtures/p2_ec32mb.net");

/// The system name the module is built under here.
const MODULE: &str = "EC32MB";

/// The module built bare with a scenario, the bench rails it needs for a
/// pull to read as a level: ground at 0 V and the `VIO_56_63` I/O rail up,
/// exactly what the ROM boot fixture supplies. Neither is implicit.
fn module_with(scenario: Scenario) -> embsim_board::BuiltSystem {
    System::new()
        .board(MODULE, shipped_ec32mb_board())
        .scenario(
            scenario
                .net_stuck(&format!("{MODULE}.GND"), 0.0)
                .net_stuck(&format!("{MODULE}.VIO_56_63"), 3.3),
        )
        .build()
        .expect("the module builds")
}

/// The strap that reaches the P59 pull-down: with the DIP switch open it is
/// `R303`'s far end alone, tied to ground through nothing but the resistor.
/// Ground arrives the way a carrier supplies it, over a `J203` finger — a
/// declared rail, not a stuck net — so the node is one resistor from the
/// only source that reaches it.
#[rstest]
fn a_net_one_resistor_from_ground_reports_that_resistor() {
    behaviour!(Test {
        id: "engine.pulled-ohms-are-the-winners-path",
        covers: Some("board/src/engine.rs#project_root"),
        given:
            "the module with ground supplied over a carrier finger and every DIP position open, \
                read at the far end of the P59 pull-down resistor",
    });
    expect!(
        "pulled-low-through-r303",
        "the net is pulled low through the pull-down's own 10.5 kilohms",
        "the ohms a pulled net reports are the series path of the source that wins it, \
         however many other resistors share its cluster",
    );
    let harness = Harness::new().power(
        EndpointRef::parse("CARRIER.GND").expect("endpoint"),
        EndpointRef::parse(&format!("{MODULE}.J203.43")).expect("endpoint"),
        0.0,
    );
    let system = System::new()
        .board(MODULE, shipped_ec32mb_board())
        .harness(harness)
        .build()
        .expect("the module builds");
    let net = format!("{MODULE}.Net-(S301-4_OFF)");
    let state = system.nets()[system.net_id(&net).expect("the strap net exists").0].state;
    assert_eq!(
        state,
        NetState::Pulled(Level::Low, 10_500.0),
        "R303 is 10.5K; the net is ground through it and nothing else"
    );
}

/// The name of the net a pin sits on, board-local.
fn net_of(board: &Board, reference: &str, pin: &str) -> String {
    board
        .nets()
        .iter()
        .find(|n| {
            n.nodes
                .iter()
                .any(|p| p.reference == reference && p.pin == pin)
        })
        .unwrap_or_else(|| panic!("{reference}.{pin} is on a net"))
        .name
        .clone()
}

// ============================================================
// The value parser, over the whole module
// ============================================================

/// Every passive on the module carries a value the parser reads: the
/// capacitors' fields carry a voltage rating after the value, the
/// inductors' a current rating and a DC resistance, and one capacitor an
/// alias in parentheses — none of which is the value.
#[rstest]
#[case::capacitors('C', 66)]
#[case::inductors('L', 2)]
#[case::resistors('R', 15)]
fn every_passive_value_on_the_module_parses(#[case] prefix: char, #[case] expected: usize) {
    behaviour!(Test {
        id: "registry.value-field-first-token",
        covers: Some("board/src/registry.rs#parse_passive_value"),
        given: "the P2-EC32MB module netlist, whose passive value fields carry ratings, DC \
                resistances and aliases after the value itself",
    });
    expect!(
        "all-parse",
        "every capacitor, inductor and resistor value on the module parses to a number in \
         base units",
        "the value is the first token of the field; what follows it is a rating or an alias \
         a DC model does not read"
    );

    let parsed = embsim_board::netlist::parse(EC32MB_NETLIST).expect("the fixture parses");
    let passives: Vec<&embsim_board::ComponentDecl> = parsed
        .components
        .iter()
        .filter(|c| {
            let mut chars = c.reference.chars();
            chars.next() == Some(prefix) && chars.all(|ch| ch.is_ascii_digit())
        })
        .collect();
    assert_eq!(passives.len(), expected, "{prefix}-parts on the module");
    let unparsed: Vec<(&str, &str)> = passives
        .iter()
        .filter(|c| parse_passive_value(&c.value).is_none())
        .map(|c| (c.reference.as_str(), c.value.as_str()))
        .collect();
    assert_eq!(
        unparsed,
        Vec::<(&str, &str)>::new(),
        "every {prefix}-part's value field parses"
    );
    for c in &passives {
        let v = parse_passive_value(&c.value).unwrap();
        assert!(
            v > 0.0 && v.is_finite(),
            "{}: {:?} → {v}",
            c.reference,
            c.value
        );
    }
}

// ============================================================
// Mechanical nodes
// ============================================================

/// A mounting hole, a raw-board BOM line, a layout node: parts the board
/// carries with nothing electrical, and every one a node — the register the
/// classifier used to skip.
#[rstest]
#[case::module_bom_lines(shipped_ec32mb_board as fn() -> Board, &["PCB", "NC_Net", "J701", "J702"])]
#[case::edge_mounting_holes(edge_board as fn() -> Board, &["H5", "H6", "H7", "H8"])]
fn mechanical_parts_are_nodes_with_nothing_electrical(
    #[case] build: fn() -> Board,
    #[case] references: &[&str],
) {
    behaviour!(Test {
        id: "pipeline.mechanical-parts-are-nodes",
        covers: Some("board/src/board.rs#Board::nodes"),
        given: "a reference board whose netlist lists mounting holes, a raw-board BOM line \
                and a layout node beside its electrical parts",
    });
    expect!(
        "mechanical-class",
        "each of those parts is a node of the mechanical class",
        "a part the board carries is a node whatever it does, so nothing on the netlist is \
         skipped or excused"
    );
    expect!(
        "no-electrical-existence",
        "none of them is a component, and the board builds with no finding that names \
         their nets",
        "a mechanical node has pads and no drive, no sense and no rail"
    );

    let board = build();
    for reference in references {
        assert_eq!(
            board.node_class(reference),
            Some(&PartClass::Mechanical),
            "{reference}"
        );
        assert!(
            !board.component_refs().any(|r| r == *reference),
            "{reference} is not a component"
        );
    }
    let nets: Vec<String> = references
        .iter()
        .flat_map(|r| {
            board
                .nets()
                .iter()
                .filter(|n| n.nodes.iter().any(|p| p.reference == *r))
                .map(|n| n.name.clone())
                .collect::<Vec<_>>()
        })
        .collect();
    let built = System::new().board("B", board).build().expect("builds");
    for finding in built.diagnostics().findings() {
        let named = match finding {
            Finding::FloatingSense { net, .. } | Finding::PowerNetUnsourced { net } => Some(net),
            Finding::Contention { net, .. } => Some(net),
            _ => None,
        };
        if let Some(named) = named {
            assert!(
                !nets.iter().any(|n| format!("B.{n}") == *named),
                "a mechanical node's net carries no finding: {finding:?}"
            );
        }
    }
}

// ============================================================
// Switch poles
// ============================================================

/// The module's DIP switch, position by position. Closing a pole joins its
/// `ON` and `OFF` nets into one node, so the resistor behind the `OFF` side
/// reaches the P59 strap: position 3 pulls it up through `R302`, position 4
/// down through `R303`; with every position open the strap floats, as the
/// module's own docs say it does.
#[rstest]
#[case::all_open(None, false, None)]
#[case::pull_up_closed(Some(P59_PULL_UP_POLE), true, Some(Level::High))]
#[case::pull_down_closed(Some(P59_PULL_DOWN_POLE), true, Some(Level::Low))]
fn closing_a_dip_switch_pole_joins_its_two_nets(
    #[case] closed: Option<usize>,
    #[case] merged: bool,
    #[case] strap: Option<Level>,
) {
    behaviour!(Test {
        id: "switch.closed-pole-is-one-node",
        covers: Some("board/src/system.rs#System::assemble"),
        given: "the P2-EC32MB module with its ground and I/O rail supplied by the bench, and \
                one pole of the four-way DIP switch closed by the scenario, or none",
    });
    expect!(
        "nets-joined",
        "the two nets on either side of a closed pole are one electrical node, and the nets \
         of an open pole stay two",
        "a closed contact is a short, so the engine merges its nets at build the way a \
         solder bridge is merged"
    );
    expect!(
        "strap-selected",
        "the P59 boot strap reads pulled high with the pull-up position closed, pulled low \
         with the pull-down position closed, and floats with every position open",
        "the switch selects which of the two resistors reaches the strap, which is how the \
         module's boot mode is set"
    );

    let scenario = match closed {
        Some(pole) => Scenario::default().switch(
            &format!("{MODULE}.{FLASH_SELECT_SWITCH}"),
            pole,
            JumperState::Closed,
        ),
        None => Scenario::default(),
    };
    let system = module_with(scenario);
    let strap_net = format!("{MODULE}.P2_IO59");

    // The side the resistor hangs on, per position (vendor pin labels).
    let off_side = |pole: usize| format!("{MODULE}.Net-(S301-{}_OFF)", pole + 1);
    for pole in [P59_PULL_UP_POLE, P59_PULL_DOWN_POLE] {
        assert_eq!(
            system.names_are_merged(&strap_net, &off_side(pole)),
            merged && closed == Some(pole),
            "P2_IO59 and {} merged?",
            off_side(pole)
        );
    }

    let state = system
        .net_id(&strap_net)
        .map(|id| system.nets()[id.0].state)
        .expect("the strap net exists");
    match strap {
        Some(level) => assert!(
            matches!(state, NetState::Pulled(l, _) if l == level),
            "P2_IO59 pulled {level:?}; got {state:?}"
        ),
        None => {
            assert_eq!(state, NetState::Floating, "with every pole open");
            assert!(system.diagnostics().contains(&Finding::FloatingSense {
                net: strap_net.clone(),
                kind: SenseKind::Digital,
            }));
        }
    }
}

/// A detached contact on a closed pole conducts nothing: the pole honours
/// `pin_detach` on either of its pins, like a passive edge does.
#[rstest]
#[case::on_side("3_ON")]
#[case::off_side("3_OFF")]
fn a_detached_contact_leaves_a_closed_pole_open(#[case] lifted: &str) {
    behaviour!(Test {
        id: "switch.detached-contact-does-not-conduct",
        covers: Some("board/src/system.rs#System::assemble"),
        given: "the module with the DIP switch's pull-up position closed and one of that \
                pole's two contacts detached from its net by the scenario",
    });
    expect!(
        "not-joined",
        "the pole's two nets stay separate and the P59 strap floats",
        "a lifted contact is an open circuit whatever the switch position says"
    );

    let system = module_with(
        Scenario::default()
            .switch(
                &format!("{MODULE}.{FLASH_SELECT_SWITCH}"),
                P59_PULL_UP_POLE,
                JumperState::Closed,
            )
            .pin_detach(&format!("{MODULE}.{FLASH_SELECT_SWITCH}.{lifted}")),
    );
    let strap_net = format!("{MODULE}.P2_IO59");
    assert!(!system.names_are_merged(&strap_net, &format!("{MODULE}.Net-(S301-3_OFF)")));
    assert!(system.diagnostics().contains(&Finding::FloatingSense {
        net: strap_net,
        kind: SenseKind::Digital,
    }));
}

/// The FLASH position is the one the ROM boot depends on: closed, the
/// P2's `P61` and the flash's `~CS` are one node, and the pull-up on the
/// select reaches the pin.
#[rstest]
fn the_flash_position_puts_the_chip_select_on_p61() {
    behaviour!(Test {
        id: "switch.flash-position-selects-the-flash",
        covers: Some("board/src/system.rs#System::assemble"),
        given: "the module with the DIP switch's FLASH position closed and the I/O rail up",
    });
    expect!(
        "select-on-p61",
        "the P2's pin 61 and the flash's chip select are one node, pulled high by the \
         select's pull-up",
        "the FLASH position is the module's switch between the flash and the card on the \
         shared bus"
    );

    let system = module_with(Scenario::default().switch(
        &format!("{MODULE}.{FLASH_SELECT_SWITCH}"),
        FLASH_SELECT_POLE,
        JumperState::Closed,
    ));
    let p61 = format!("{MODULE}.P2_IO61");
    let cs = format!("{MODULE}.SPI_CS");
    assert!(system.names_are_merged(&p61, &cs));
    let state = system.nets()[system.net_id(&p61).unwrap().0].state;
    assert!(
        matches!(state, NetState::Pulled(Level::High, _)),
        "P61 is pulled high through the select's pull-up; got {state:?}"
    );
}

/// A jumper is a one-pole switch: `jumper` on a switch sets its pole 0,
/// `switch` on pole 0 sets the same thing — the solder link `J101` either
/// way.
#[rstest]
#[case::by_jumper(true)]
#[case::by_switch(false)]
fn a_jumper_call_closes_pole_zero_of_a_switch(#[case] via_jumper: bool) {
    behaviour!(Test {
        id: "switch.jumper-is-pole-zero",
        covers: Some("board/src/system.rs#Scenario::jumper"),
        given: "the module's oscillator-option solder link, a one-pole switch, closed by the \
                scenario either as a jumper or as pole zero of a switch",
    });
    expect!(
        "link-closed",
        "the link's two pads are one node either way",
        "a jumper is a switch with one pole, so both spellings name the same contact"
    );

    let reference = format!("{MODULE}.J101");
    let scenario = if via_jumper {
        Scenario::default().jumper(&reference, JumperState::Closed)
    } else {
        Scenario::default().switch(&reference, 0, JumperState::Closed)
    };
    let system = module_with(scenario);
    assert!(system.names_are_merged(
        &format!("{MODULE}.P2_IO32"),
        &format!("{MODULE}.Osc_Option")
    ));

    let open = module_with(Scenario::default());
    assert!(!open.names_are_merged(
        &format!("{MODULE}.P2_IO32"),
        &format!("{MODULE}.Osc_Option")
    ));
}

/// A pole the part does not have is a build error naming the part, the
/// pole asked for and the poles it has.
#[rstest]
#[case::fifth_dip_position("S301", 4, 4)]
#[case::second_pole_of_a_link("J101", 1, 1)]
#[case::a_resistor_has_none("R100", 0, 0)]
fn a_pole_the_part_does_not_have_fails_the_build(
    #[case] reference: &str,
    #[case] pole: usize,
    #[case] poles: usize,
) {
    behaviour!(Test {
        id: "switch.unknown-pole-fails-the-build",
        covers: Some("board/src/system.rs#Scenario::switch"),
        given: "a scenario that sets a pole a part does not have — a fifth position on a \
                four-way switch, a second pole on a solder link, any pole on a resistor",
    });
    expect!(
        "build-refused",
        "the system does not build, and the error names the part, the pole asked for and \
         how many poles the part has",
        "a scenario line that names nothing on the board is a mistake the build must catch"
    );

    let error = System::new()
        .board(MODULE, shipped_ec32mb_board())
        .scenario(Scenario::default().switch(
            &format!("{MODULE}.{reference}"),
            pole,
            JumperState::Closed,
        ))
        .build()
        .expect_err("no such pole");
    match &error {
        SystemError::UnknownSwitchPole {
            reference: named,
            pole: asked,
            poles: has,
        } => {
            assert_eq!(named, &format!("{MODULE}.{reference}"));
            assert_eq!(*asked, pole);
            assert_eq!(*has, poles);
        }
        other => panic!("expected UnknownSwitchPole, got {other:?}"),
    }
    let rendered = error.to_string();
    assert!(rendered.contains(reference), "{rendered}");
}

/// The Edge board's reset button is a `SW_Push` symbol: the auto tier makes
/// it a one-pole switch, open, and closing it presses the button — the
/// reset net and ground become one node.
#[rstest]
fn the_reset_button_is_a_switch_the_auto_tier_classifies() {
    behaviour!(Test {
        id: "switch.push-button-auto-classified",
        covers: Some("board/src/registry.rs#PartRegistry::classify"),
        given: "the MaD EdgeBoard, whose reset button is drawn with the standard two-pin \
                push-button symbol and registered nowhere",
    });
    expect!(
        "one-open-pole",
        "the button is a switch node with one pole, open",
        "a two-pin switch symbol pairs its two pins, and a button rests unpressed"
    );
    expect!(
        "press-shorts-reset",
        "closing that pole by scenario makes the reset net and ground one node",
        "pressing the button is the short the symbol draws"
    );

    let board = edge_board();
    let pole = match board.node_class("SW1") {
        Some(PartClass::Switch { poles }) if poles.len() == 1 => poles[0].clone(),
        other => panic!("SW1 is a one-pole switch; got {other:?}"),
    };
    assert_eq!(pole.state, JumperState::Open);
    let reset = net_of(&board, "SW1", &pole.a);
    let ground = net_of(&board, "SW1", &pole.b);
    assert_ne!(reset, ground);

    let pressed = System::new()
        .board("Edge", board)
        .scenario(Scenario::default().switch("Edge.SW1", 0, JumperState::Closed))
        .build()
        .expect("builds");
    assert!(pressed.names_are_merged(&format!("Edge.{reset}"), &format!("Edge.{ground}")));

    let released = System::new()
        .board("Edge", edge_board())
        .build()
        .expect("builds");
    assert!(!released.names_are_merged(&format!("Edge.{reset}"), &format!("Edge.{ground}")));
}

// ============================================================
// The error tier
// ============================================================

/// A part the registry cannot classify is a build error, and the error
/// names the reference, the part and the value — on a netlist with no
/// libsource the value is the only name the part has.
#[rstest]
fn an_unmodelled_part_fails_the_build_naming_reference_part_and_value() {
    behaviour!(Test {
        id: "pipeline.unknown-part-names-the-value",
        covers: Some("board/src/registry.rs#PartRegistry::classify"),
        given: "the P2-EC32MB netlist, which names no symbol library, built with a registry \
                that knows its passives but has no entry for the processor",
    });
    expect!(
        "build-refused",
        "the board does not build",
        "a part is a node whose class has behaviour, or it is an error"
    );
    expect!(
        "error-names-the-value",
        "the error names the processor's reference, its empty part name and its value",
        "with no symbol library the value is the only name the part has, so an error \
         without it would name nothing a reader could find"
    );

    let mut registry = PartRegistry::new();
    registry.classify_unnamed_by_reference(true);
    let parsed = embsim_board::netlist::parse(EC32MB_NETLIST).expect("the fixture parses");
    let error = Board::from_netlist(parsed, &registry).expect_err("U100 has no class");
    let rendered = error.to_string();
    for needle in ["U100", "\"\"", "P2X8C4M64P"] {
        assert!(rendered.contains(needle), "{rendered:?} lacks {needle}");
    }
}

/// A piecewise-linear element is a class declared ahead of its behaviour:
/// its specification is the placeholder until the elements land, so a
/// fitted one would be a node with nothing behind it. The build refuses
/// it and names the part; the same part unpopulated builds, like any
/// other absent part.
#[rstest]
#[case::fitted(false)]
#[case::unpopulated(true)]
fn a_declared_element_without_behaviour_builds_only_unpopulated(#[case] absent: bool) {
    behaviour!(Test {
        id: "pipeline.element-declared-ahead-of-behaviour",
        covers: Some("board/src/system.rs#System::assemble"),
        given: "a board whose diode is registered as a piecewise-linear element with the \
                placeholder specification, fitted or made absent by the scenario",
    });
    expect!(
        "fitted-refused",
        "fitted, the system does not build and the error names the diode",
        "a part is a node whose class has behaviour, and a declaration of intent is not \
         behaviour"
    );
    expect!(
        "absent-builds",
        "made absent by the scenario, the system builds",
        "an unpopulated part contributes nothing, whatever its class"
    );

    const NETLIST: &str = r#"(export (version "E")
  (components
    (comp (ref "D1") (value "SS36") (libsource (lib "Diode") (part "SS36") (description "")))
    (comp (ref "R1") (value "220") (libsource (lib "Device") (part "R") (description ""))))
  (nets
    (net (code "1") (name "A") (class "Default")
      (node (ref "D1") (pin "A") (pintype "passive"))
      (node (ref "R1") (pin "1") (pintype "passive")))
    (net (code "2") (name "K") (class "Default")
      (node (ref "D1") (pin "K") (pintype "passive")))
    (net (code "3") (name "S") (class "Default")
      (node (ref "R1") (pin "2") (pintype "passive")))))"#;

    let mut registry = PartRegistry::new();
    registry.register_pwl("SS36", ["A", "K"], PwlSpec::default());
    let board = Board::from_netlist(
        embsim_board::netlist::parse(NETLIST).expect("the fixture parses"),
        &registry,
    )
    .expect("the class is declared, so the board builds");
    let mut scenario = Scenario::default();
    if absent {
        scenario = scenario.dnp_override("B.D1", DnpState::Absent);
    }
    let result = System::new().board("B", board).scenario(scenario).build();
    if absent {
        assert!(result.is_ok(), "{:?}", result.err());
    } else {
        let error = result.expect_err("a fitted element with no behaviour is refused");
        assert!(
            matches!(
                &error,
                SystemError::Board {
                    name,
                    error: BoardError::UnmodelledElement { reference }
                } if name == "B" && reference == "D1"
            ),
            "{error:?}"
        );
        assert!(error.to_string().contains("D1"), "{error}");
    }
}
