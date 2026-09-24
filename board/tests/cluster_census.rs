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
//! * **piecewise-linear elements** — zero on every board until the elements
//!   land (`NODES.md` §8 phase 3). A `register_pwl` today is a declaration
//!   of intent the system refuses to fit, and the count here is the other
//!   half of that gate: a part cannot leave the stub count by becoming a
//!   node with a placeholder behind it.
//!
//! Run with `--nocapture` to see the table.

mod machine_parts;

use embsim_board::{Board, PartClass, System};
use machine_parts::{ds2_board, edge_board, shipped_ec32mb_board};
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
    /// Piecewise-linear elements: zero until phase 3.
    pwl: usize,
}

/// The P2-EC32MB as `embsim-boards` ships it, its processor slot filled by
/// a P2 package held in reset (`P2Package::held_in_reset`: every pad
/// released, the rails and reset sensed, `XI` taking the board's rate — a
/// node with the package's behaviour and no core). 114 netlist parts,
/// every one a node: the P2 package, the boot flash, the TCXO, the two
/// inverters and the four PSRAMs are modelled; the polarity FET, the two
/// bucks, the eight LDOs and the detector are 12 registered facades (phase
/// 2 took the count from 20 to 13 with the models, then to 12 with the
/// package); the DIP switch and the solder link are
/// switches, the mounting holes, `PCB` and `NC_Net` mechanical nodes. The
/// 8-root cluster is the one `NODES.md` §6 sized offline: `GND`,
/// `Common_VDD` and `Common_LDOin`, the two bucks' `SW` and `FB` nodes
/// joined to them through the output inductors and the feedback dividers,
/// and the P59 pull-down net `R303` ties to ground.
const EC32MB: Census = Census {
    clusters: 83,
    largest_cluster_roots: 8,
    stub_count: 12,
    mechanical: &["J701", "J702", "NC_Net", "PCB"],
    pwl: 0,
};

/// The MaD EdgeBoard from `fixtures/mad_edge.net`. 168 netlist parts: the
/// RS-422 driver and receiver, the five isolators and the 21 Schmitt
/// inverters are modelled; 19 registered facades (the isolated DC/DCs, the
/// eight current regulators, the optos, the two bucks, the polarity FET,
/// the transistor — phase 2 took the count from 45); the push button is a
/// switch, the three-pad jumper `JP1` a two-pole switch, and the four
/// mounting holes mechanical nodes. The 11-root cluster is `+3.3V` with
/// the nine LED anodes that reach it through their 220 Ω series resistors
/// and the `D2` cathode the buck's output inductor `L2` ties to the rail,
/// as §6 sized it.
const EDGE: Census = Census {
    clusters: 209,
    largest_cluster_roots: 11,
    stub_count: 19,
    mechanical: &["H5", "H6", "H7", "H8"],
    pwl: 0,
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
        "no-elements",
        "the board carries no piecewise-linear element",
        "the element class is declared ahead of its behaviour, and a part that reached it \
         would leave the stub count with a placeholder behind it"
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
        "{name}: piecewise-linear elements — none until the elements land"
    );
    assert_eq!(census, expected, "{name}: the whole fixture");
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
