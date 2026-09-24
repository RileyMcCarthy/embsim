//! The census `NODES.md` §8 phase 0 asks for: how big the conduction
//! clusters on the three reference boards are, and how many of their parts
//! have nothing behind them.
//!
//! Two of `DESIGN.md`'s rules are numbers, and this is where the numbers
//! live. Rule 4 keeps every solve tractable by keeping every cluster small —
//! the plan bounds `m ≤ 8` on every board once terminals are cluster
//! boundaries (phase 4) — and rule 1 says a netlist part is a node whose
//! class has behaviour, which is the `stub_count` reading 0 once the last
//! pin facade has become a model. Neither can be held to without a baseline,
//! so this test commits today's figures as a fixture and asserts them
//! exactly: a phase that moves one has to say so in its proof list, and a
//! change that moves one by accident fails here first. The stub count in
//! particular is a **never-rises gate** from phase 1 on: it can only be
//! lowered, by a phase that replaces a facade with a model.
//!
//! What is counted, per board built bare from its vendor netlist (no
//! harness, no scenario — the topology as drawn):
//!
//! * **clusters** — conduction clusters, what a resistor, an inductor or a
//!   closed jumper joins and what nothing else crosses;
//! * **largest cluster** — identity roots in the biggest one: the size `m`
//!   of the largest matrix an escalated solve on that board can build;
//! * **stub count** — parts with nothing behind them: the registered pin
//!   facades with no behaviour (`StubPart`s). Since phase 1 every netlist
//!   part is a node ([`Board::nodes`]) and there is no stub list and no
//!   ignored tier,
//!   so the only stub left is a facade the board cannot tell from a model:
//!   the census names each board's modelled parts and counts the other
//!   registered components;
//! * **mechanical nodes** — the exact set, by reference. A part registered
//!   mechanical is a node with pads and nothing electrical, which is also
//!   what a stub would be if it were declared that way instead — so the set
//!   is a fixture, and a new mechanical registration is a census change
//!   the phase has to say;
//! * **piecewise-linear elements** — the parts registered by specification
//!   (`register_pwl`, `NODES.md` §8 phase 3): the diodes, LEDs, polarity
//!   FETs, transistor and current regulators the element library classifies
//!   by manufacturer part number. An element is a membership edge among its
//!   non-terminal nets, and a bare board declares no terminal, so the LED
//!   cathodes on the Edge board's ground and the FET gates on both grounds
//!   grow the ground clusters here; with the bench rails in (every board
//!   test) the same elements stamp against those rails as constants
//!   (`engine.rs`, `build_topology`) — which makes the twelve sinking LED
//!   chains two-net clusters, while the nine chains sourced from `+3.3V`
//!   still meet through their series resistors on that rail, since a
//!   resistor edge unions through a terminal until phase 4, and form one
//!   cluster of 20 roots with the `D2` cathode. That figure is the last
//!   case below, the one `DESIGN.md` rule 4's bound waits on.
//!
//! Run with `--nocapture` to see the table.

use std::collections::BTreeSet;

mod machine_parts;

use embsim_board::{Board, PartClass, System};
use machine_parts::{bench_rails, ds2_board, edge_board, shipped_ec32mb_board};
use rstest::rstest;
use vibes_behaviour::{behaviour, expect, Test};

// ============================================================
// The fixture
// ============================================================

/// One board's census.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Census {
    /// Conduction clusters on the bare board.
    clusters: usize,
    /// Identity roots in the largest cluster — the biggest matrix an
    /// escalated solve on this board builds.
    largest_cluster_roots: usize,
    /// Parts with nothing behind them (see the module docs).
    stub_count: usize,
    /// The mechanical nodes, by reference, sorted.
    mechanical: &'static [&'static str],
    /// Piecewise-linear elements: the parts the element library registers
    /// by specification.
    pwl: usize,
}

/// The P2-EC32MB as `embsim-boards` ships it, its processor slot filled by
/// a P2 package held in reset (`P2Package::held_in_reset`: every pad
/// released, the rails and reset sensed, `XI` taking the board's rate — a
/// node with the package's behaviour and no core). 114 netlist parts,
/// every one a node: the P2 package, the boot flash, the TCXO, the two
/// inverters and the four PSRAMs are modelled; the polarity FET `U401`
/// and the white LEDs `D601`/`D602` are elements by specification (phase
/// 3); the two bucks, the eight LDOs and the detector are 11 registered
/// facades (phase 2 took the count from 20 to 13 with the models, then to
/// 12 with the package, phase 3 to 11 with the FET); the DIP switch and
/// the solder link are switches, the mounting holes, `PCB` and `NC_Net`
/// mechanical nodes. The 10-root cluster is the 8-root one `NODES.md` §6
/// sized offline — `GND`, `Common_VDD` and `Common_LDOin`, the two bucks'
/// `SW` and `FB` nodes joined to them through the output inductors and the
/// feedback dividers, and the P59 pull-down net `R303` ties to ground —
/// plus `VIN_Edge` and `VIN_Edge_Protected`, which the FET's gate on the
/// bare board's ground joins to it (83 → 79 clusters: those two and the
/// two LED cathode nets `U601` drives, which the LEDs join to their anodes'
/// net). With the carrier's ground strapped, ground is a terminal and the
/// FET stamps against it from a cluster of its own.
const EC32MB: Census = Census {
    clusters: 79,
    largest_cluster_roots: 10,
    stub_count: 11,
    mechanical: &["J701", "J702", "NC_Net", "PCB"],
    pwl: 3,
};

/// The MaD EdgeBoard from `fixtures/mad_edge.net`. 168 netlist parts: the
/// RS-422 driver and receiver, the five isolators, the 21 Schmitt
/// inverters and the five optocouplers are modelled; 33 elements by
/// specification (the two Schottky diodes, the 21 indicator LEDs, the
/// eight current regulators, the polarity FET and the transistor — phase
/// 3); 4 registered facades (the isolated DC/DCs and the two bucks — phase
/// 2 took the count from 45 to 19, phase 3 to 4); the push button is a
/// switch, the three-pad jumper `JP1` a two-pole switch, and the four
/// mounting holes mechanical nodes. The 29-root cluster is the bare
/// board's `GND` with everything its elements join to it: the twelve LED
/// chains whose cathodes sit on ground (anode net and inverter output
/// each), the polarity FET's drain and source nets through its gate on
/// ground, the charge-pump opto's LED anode net and the pin it is driven
/// from — the 47-net collapse `NODES.md` §2 rule 1 describes, short of
/// the nine chains that source from `+3.3V` (a 20-root cluster of its
/// own) and the two catch diodes, whose cathodes are the bucks' output
/// nodes, declared rails and so barriers. With the bench rails in, ground
/// and `+3.3V` are terminals, every chain is a two-net cluster and the FET
/// a one-net one (209 → 167 clusters bare).
const EDGE: Census = Census {
    clusters: 167,
    largest_cluster_roots: 29,
    stub_count: 4,
    mechanical: &["H5", "H6", "H7", "H8"],
    pwl: 33,
};

/// The DS2 force-gauge add-on from `fixtures/ds2_addon.net`. 31 netlist
/// parts: the ADS122U04 is the one registered part and it is a model; the
/// rest classify in the auto tier.
const DS2: Census = Census {
    clusters: 22,
    largest_cluster_roots: 2,
    stub_count: 0,
    mechanical: &[],
    pwl: 0,
};

/// The reference designators behind which a real model sits, per board.
/// Every other registered component is a facade with no behaviour and is
/// counted as a stub.
const EC32MB_MODELLED: &[&str] = &[
    "U100", "U301", "X100", "U101", "U601", "U302", "U303", "U304", "U305",
];
#[rustfmt::skip]
const EDGE_MODELLED: &[&str] = &[
    "U24", "U25", "IC5", "IC1", "IC2", "IC14", "IC15", "IC16",
    // The 21 SN74LVC1G14 LED drivers.
    "U9", "U10", "U11", "U12", "U13", "U14", "U15", "U16", "U17", "U18", "U19", "U21", "U22",
    "U27", "U28", "U29", "U30", "U31", "U32", "U33", "U34",
    // The five optocouplers: the 6N137 and four VO2631s.
    "U4", "U5", "U6", "U7", "U8",
];
const DS2_MODELLED: &[&str] = &["U1"];

// ============================================================
// Taking the census
// ============================================================

/// The registered components of a board that are not named as modelled:
/// the facades with nothing behind them, by name.
fn stubs_of(name: &str, board: &Board, modelled: &[&str]) -> Vec<String> {
    for reference in modelled {
        assert!(
            board.component_refs().any(|r| r == *reference),
            "{name}: {reference} is named as a modelled part but is not a registered component"
        );
    }
    board
        .component_refs()
        .filter(|r| !modelled.contains(r))
        .map(str::to_string)
        .collect()
}

/// The mechanical nodes of a board, by reference, sorted — a fixture is
/// compared by content, not by netlist order.
fn mechanical_of(board: &Board) -> Vec<String> {
    let mut refs: Vec<String> = board
        .nodes()
        .filter(|(_, class)| matches!(class, PartClass::Mechanical))
        .map(|(reference, _)| reference.to_string())
        .collect();
    refs.sort_unstable();
    refs
}

/// Measure one board and print its row.
fn take_census(name: &str, board: Board, modelled: &[&str]) -> Census {
    let stubs = stubs_of(name, &board, modelled);
    let mechanical = mechanical_of(&board);
    let pwl = board
        .nodes()
        .filter(|(_, class)| matches!(class, PartClass::Pwl { .. }))
        .count();
    // Every netlist part is a node; the classes, counted, so the table says
    // what the board is made of.
    let mut classes: Vec<(&'static str, usize)> = Vec::new();
    for (_, class) in board.nodes() {
        let label = match class {
            PartClass::Passive { .. } => "passive",
            PartClass::Jumper { .. } => "jumper",
            PartClass::Switch { .. } => "switch",
            PartClass::Pwl { .. } => "pwl",
            PartClass::Boundary => "boundary",
            PartClass::Probe => "probe",
            PartClass::Mechanical => "mechanical",
            PartClass::Registered { .. } => "registered",
        };
        match classes.iter_mut().find(|(l, _)| *l == label) {
            Some((_, n)) => *n += 1,
            None => classes.push((label, 1)),
        }
    }
    let nodes = board.nodes().count();

    let built = System::new()
        .board(name, board)
        .build()
        .unwrap_or_else(|e| panic!("{name} builds bare: {e}"));
    let clusters = built.cluster_roots();
    let mut sizes: Vec<usize> = clusters.iter().map(Vec::len).collect();
    sizes.sort_unstable_by(|a, b| b.cmp(a));
    let largest = clusters
        .iter()
        .max_by_key(|roots| roots.len())
        .map(|roots| {
            roots
                .iter()
                .map(|id| built.nets()[id.0].name.as_str())
                .collect::<Vec<_>>()
        })
        .unwrap_or_default();

    // The fixture holds a `&'static` slice; the measured set is compared
    // by content, and a census is taken a handful of times per process.
    let mechanical: &'static [&'static str] = Vec::leak(
        mechanical
            .into_iter()
            .map(|r| &*r.leak())
            .collect::<Vec<&'static str>>(),
    );
    let census = Census {
        clusters: clusters.len(),
        largest_cluster_roots: sizes.first().copied().unwrap_or(0),
        stub_count: stubs.len(),
        mechanical,
        pwl,
    };
    eprintln!(
        "census {name}: clusters={} largest_cluster_roots={} stub_count={} pwl={} \
         escalated_solves_at_build={} nodes={nodes} {classes:?}\n  \
         cluster sizes (roots, descending): {:?}\n  \
         largest cluster: {largest:?}\n  nothing behind: {stubs:?}\n  \
         mechanical: {mechanical:?}",
        census.clusters,
        census.largest_cluster_roots,
        census.stub_count,
        census.pwl,
        built.escalated_solves(),
        &sizes[..sizes.len().min(16)],
    );
    census
}

/// The fixture, asserted exactly. A phase that changes a figure changes the
/// constant above and says so in its proof list; the stub count may only be
/// lowered.
#[rstest]
#[case::ec32mb("EC32MB", shipped_ec32mb_board as fn() -> Board, EC32MB_MODELLED, EC32MB)]
#[case::edge("EdgeBoard", edge_board as fn() -> Board, EDGE_MODELLED, EDGE)]
#[case::ds2("DS2Addon", ds2_board as fn() -> Board, DS2_MODELLED, DS2)]
fn the_census_of_a_reference_board_is_the_committed_fixture(
    #[case] name: &str,
    #[case] build: fn() -> Board,
    #[case] modelled: &[&str],
    #[case] expected: Census,
) {
    behaviour!(Test {
        id: "census.reference-board-fixture",
        covers: Some("board/src/system.rs#BuiltSystem::cluster_roots"),
        given: "one of the three reference boards — the P2-EC32MB module, the MaD EdgeBoard \
                or the DS2 force-gauge add-on — built bare from its vendor netlist, with no \
                harness and no scenario",
    });
    expect!(
        "cluster-count",
        "the number of conduction clusters is the committed census figure for that board",
        "a cluster is what resistors, inductors and closed jumpers join, so its count is a \
         property of the drawing and changes only when the model of a part changes"
    );
    expect!(
        "largest-cluster",
        "the largest cluster holds exactly the committed number of electrical nodes, which \
         is the biggest matrix any solve on that board can build",
        "every solve is a dense elimination over one cluster, so the largest cluster is the \
         board's whole solve cost"
    );
    expect!(
        "stub-count",
        "the number of registered parts with no behaviour behind them is the committed \
         figure for that board",
        "every netlist part is a node whose class has behaviour, so this figure may only \
         fall, one model at a time, and reads zero when the last pin facade is a model"
    );
    expect!(
        "mechanical-set",
        "the mechanical nodes are exactly the committed references for that board",
        "a mechanical node has pads and nothing electrical, so registering an active part \
         as one would lower the stub count without giving the part behaviour; the exact \
         set makes that a visible change to the fixture"
    );
    expect!(
        "elements",
        "the number of piecewise-linear elements — the diodes, LEDs, polarity FETs, \
         transistors and current regulators registered by specification — is the committed \
         figure for that board",
        "an element is a node with branches the solve chooses regions for; a part registered \
         by specification without one would hide behind this figure"
    );

    let census = take_census(name, build(), modelled);
    assert_eq!(
        census.clusters, expected.clusters,
        "{name}: conduction clusters — a phase that changes the topology of a part changes \
         this figure and says so"
    );
    assert_eq!(
        census.largest_cluster_roots, expected.largest_cluster_roots,
        "{name}: roots in the largest cluster — the bound every solve on this board is under"
    );
    assert_eq!(
        census.stub_count, expected.stub_count,
        "{name}: parts with nothing behind them — this figure must never rise"
    );
    assert_eq!(
        census.mechanical, expected.mechanical,
        "{name}: the mechanical nodes — a part registered mechanical is a census change"
    );
    assert_eq!(
        census.pwl, expected.pwl,
        "{name}: piecewise-linear elements — a part registered by specification is a census \
         change"
    );
    assert_eq!(census, expected, "{name}: the whole fixture");
}

/// The Edge board under its bench rails (`machine_parts::bench_rails`, the
/// state every board test runs in): the roots of its largest cluster. The
/// twelve sinking LED chains and the FET are two- and one-net clusters
/// against the rails as constants; the nine chains sourced from `+3.3V`
/// are one cluster with the `D2` cathode, because a resistor edge still
/// unions through a terminal — so every toggle of one of those nine
/// inverters re-solves all nine chains. This is the figure `DESIGN.md`
/// rule 4's `m ≤ 8` bound waits on: phase 4, which stops resistor edges
/// at terminals too, takes it to [`EDGE_UNDER_RAILS_PHASE_4_ROOTS`] — a
/// chain's inverter output and LED anode — and makes the bound the gate.
const EDGE_UNDER_RAILS_LARGEST_CLUSTER_ROOTS: usize = 20;

/// What the same cluster reads once terminals bound resistor edges as they
/// bound elements (`NODES.md` §6, "every LED chain {Y, anode}").
const EDGE_UNDER_RAILS_PHASE_4_ROOTS: usize = 2;

/// The `m ≤ 8` every board is held to once phase 4 lands (`DESIGN.md`
/// rule 4, "Enforced by").
const RULE_4_LARGEST_CLUSTER_ROOTS: usize = 8;
const _: () = assert!(EDGE_UNDER_RAILS_PHASE_4_ROOTS <= RULE_4_LARGEST_CLUSTER_ROOTS);

/// The Edge board under the bench rails: the largest cluster is a
/// never-rises figure, with the bound it will be held to named beside it.
#[rstest]
fn the_edge_board_under_the_bench_rails_has_a_largest_cluster_that_never_grows() {
    behaviour!(Test {
        id: "census.edge-under-rails-largest-cluster",
        covers: Some("board/src/system.rs#BuiltSystem::cluster_roots"),
        given: "the MaD EdgeBoard under the bench rails every board test runs it in, with no \
                scenario",
    });
    expect!(
        "largest-cluster",
        "the largest conduction cluster holds exactly the committed number of electrical nodes: \
         the nine indicator-LED chains sourced from the 3.3 volt rail, joined through their \
         series resistors on that rail, with the input diode's cathode",
        "a rail is a boundary for an element and, until resistor edges stop at it too, joins \
         the resistors that reach it; this figure may only fall, to the two nodes of one chain, \
         and every board's largest cluster is then held to eight"
    );
    expect!(
        "solves-at-build-bounded",
        "the build escalates at most sixty-four solves: the element clusters' cold-start solves \
         and their fixed-point re-solves",
        "a rail is a constant of the solves that stamp against it and asks for none of its own"
    );
    let built = System::new()
        .board("EdgeBoard", edge_board())
        .harness(bench_rails("EdgeBoard"))
        .build()
        .expect("the Edge board builds under the bench rails");
    let clusters = built.cluster_roots();
    let largest = clusters
        .iter()
        .max_by_key(|roots| roots.len())
        .expect("a board has clusters");
    let names: BTreeSet<&str> = largest
        .iter()
        .map(|id| built.nets()[id.0].name.as_str())
        .collect();
    eprintln!(
        "census EdgeBoard under bench_rails: clusters={} largest_cluster_roots={} \
         escalated_solves_at_build={}\n  largest cluster: {names:?}",
        clusters.len(),
        largest.len(),
        built.escalated_solves(),
    );
    assert_eq!(
        largest.len(),
        EDGE_UNDER_RAILS_LARGEST_CLUSTER_ROOTS,
        "the largest cluster under the rails — this figure may only fall; the phase that \
         lowers it changes the constant and says so"
    );
    assert!(names.contains("BENCH.3V3"), "{names:?}");
    assert!(names.contains("EdgeBoard.Net-(D2-K)"), "{names:?}");
    // The escalations at build are the element clusters' solves and their
    // fixed-point re-solves, none from the rails: a bound, not a fixture.
    assert!(
        built.escalated_solves() <= 64,
        "{} escalated solves at build",
        built.escalated_solves()
    );
}

/// The one figure the plan needs today: `stub_count` reads what it reads,
/// and a board whose every part is a node with behaviour reads 0. The DS2
/// add-on already does; the other two get there in phase 4.
#[rstest]
fn the_ds2_addon_has_nothing_behind_no_part() {
    behaviour!(Test {
        id: "census.ds2-every-part-a-node",
        covers: Some("board/src/board.rs#Board::nodes"),
        given: "the DS2 force-gauge add-on built from its vendor netlist with the ADS122U04 \
                model as its converter",
    });
    expect!(
        "no-stubs",
        "every part of the board is a passive, a jumper, a connector or the modelled \
         converter, and none is a registered facade without behaviour",
        "every part on the add-on is either a primitive the auto tier classifies or the one \
         modelled converter"
    );

    let board = ds2_board();
    assert_eq!(
        stubs_of("DS2Addon", &board, DS2_MODELLED),
        Vec::<String>::new()
    );
    let unexpected: Vec<(&str, &PartClass)> = board
        .nodes()
        .filter(|(reference, class)| match class {
            PartClass::Passive { .. } | PartClass::Jumper { .. } | PartClass::Boundary => false,
            PartClass::Registered { .. } => !DS2_MODELLED.contains(reference),
            _ => true,
        })
        .collect();
    assert_eq!(unexpected, Vec::<(&str, &PartClass)>::new());
}
