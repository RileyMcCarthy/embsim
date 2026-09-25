//! The census `NODES.md` §8 phase 0 asks for: how big the conduction
//! clusters on the three reference boards are, and how many of their parts
//! have nothing behind them.
//!
//! Two of `DESIGN.md`'s rules are numbers, and this is where the numbers
//! live. Rule 4 keeps every solve tractable by keeping every cluster small
//! — `m ≤ 8` on every board under its reference harness, now that declared
//! terminals are cluster boundaries (phase 4) — and rule 1 says a netlist
//! part is a node whose class has behaviour, which is the `stub_count`
//! reading 0 once the last pin facade has become a model. Neither can be
//! held to without a baseline, so this test commits today's figures as a
//! fixture and asserts them exactly: a phase that moves one has to say so
//! in its proof list, and a change that moves one by accident fails here
//! first. The stub count in particular is a **never-rises gate** from
//! phase 1 on: it can only be lowered, by a phase that replaces a facade
//! with a model.
//!
//! What is counted, per board built bare from its vendor netlist (no
//! harness, no scenario — the topology as drawn):
//!
//! * **clusters** — conduction clusters, what a resistor joins and what
//!   nothing else crosses — a declared terminal included: a `PowerOut`
//!   pin's net is a cluster of its own, and an edge ending on it stops
//!   there. A closed jumper and an inductor are identity unions (one
//!   node, `pin_short` semantics), so a regulator's output inductor makes
//!   the rail the terminal's node rather than a member of its loads'
//!   cluster;
//! * **largest cluster** — identity roots in the biggest one: the size `m`
//!   of the largest matrix an escalated solve on that board can build;
//! * **stub count** — parts with nothing behind them. Since phase 1 every
//!   netlist part is a node ([`Board::nodes`]) and there is no stub list
//!   and no ignored tier, so the only stub there could be is a facade the
//!   board cannot tell from a model: the census names each board's
//!   modelled parts and counts the other registered components. Since
//!   phase 4 the figure is 0 on every board, and stays there;
//! * **mechanical nodes** — the exact set, by reference. A part registered
//!   mechanical is a node with pads and nothing electrical, which is also
//!   what a stub would be if it were declared that way instead — so the set
//!   is a fixture, and a new mechanical registration is a census change
//!   the phase has to say;
//! * **piecewise-linear elements** — the parts registered by specification
//!   (`register_pwl`, `NODES.md` §8 phase 3): the diodes, LEDs, polarity
//!   FETs, transistor and current regulators the element library classifies
//!   by manufacturer part number. An element is a membership edge among its
//!   non-terminal nets, and a bare board declares no ground, so the LED
//!   cathodes on the Edge board's ground and the FET gates on both grounds
//!   grow the ground clusters here; with the bench rails in (every board
//!   test) ground is a terminal, the same elements stamp against it as a
//!   constant, and every LED chain is a two-net cluster — the sinking
//!   twelve and, since phase 4 made a resistor edge stop at a terminal as
//!   an element does, the nine sourced from `+3.3V` too.
//!
//! The bare figures are fixtures, not the bound: a bare board declares no
//! ground (`NODES.md` §2, "Ground": not implicit — the bench return is a
//! harness terminal), so its ground cluster holds everything that returns
//! to it. Rule 4's bound is asserted on every board under its reference
//! harness — its declared supplies, every one a terminal — in the last
//! cases below: the EC32MB from its `J203` fingers, the DS2 add-on under
//! its force-domain rails, the Edge board under the bench rails with the
//! one socket finger the module sources held at the module LDO's 3.3 V,
//! and the Edge board with the module in its socket. The Edge board under
//! the bench rails **alone**, that finger unsourced — the state most Edge
//! board tests run in — is above the bound by design, and the case that
//! holds it says so: it is a never-rises fixture with its reason.
//!
//! Run with `--nocapture` to see the table.

use std::collections::BTreeSet;

mod machine_parts;

use embsim_board::{Board, BuiltSystem, EndpointRef, Harness, PartClass, System};
use machine_parts::{
    bench_rails, ds2_board, ec32mb_board, edge_board, force_domain_ground, force_domain_rails,
    force_gauge_harness, module_socket_harness, shipped_ec32mb_board,
};
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
/// inverters, the four PSRAMs, the two bucks, the eight LDOs and the
/// detector are modelled (phase 4 took the last eleven facades: no part
/// has nothing behind it); the polarity FET `U401` and the white LEDs
/// `D601`/`D602` are elements by specification (phase 3); the DIP switch
/// and the solder link are switches, the mounting holes, `PCB` and
/// `NC_Net` mechanical nodes. The 6-root cluster is `GND` with the two
/// bucks' `FB` nodes joined to it through the feedback dividers' lower
/// resistors, the P59 pull-down net `R303` ties to ground, and `VIN_Edge`
/// / `VIN_Edge_Protected`, which the FET's gate on the bare board's
/// ground joins to it. Phase 4 took it from 8 roots: the bucks' `SW`
/// nodes are `PowerOut` terminals, and their output inductors are
/// identity unions, so `Common_VDD` and `Common_LDOin` *are* the two
/// terminals' nodes — clusters of their own, boundaries of the divider
/// and of every load. The same split took the eight LDO outputs out of
/// the clusters their pull-ups formed (`VIO_56_63` with the `R301`–`R303`
/// nets, the RESN and debug-serial pull-ups; `VIO_40_47` with `P2_IO57`).
/// With the carrier's ground strapped, ground is a terminal too and the
/// FET stamps against it from a cluster of its own.
const EC32MB: Census = Census {
    clusters: 87,
    largest_cluster_roots: 6,
    stub_count: 0,
    mechanical: &["J701", "J702", "NC_Net", "PCB"],
    pwl: 3,
};

/// The MaD EdgeBoard from `fixtures/mad_edge.net`. 168 netlist parts: the
/// RS-422 driver and receiver, the five isolators, the 21 Schmitt
/// inverters, the five optocouplers, the two bucks and the two isolated
/// DC/DCs are modelled (phase 4 took the last four facades); 33 elements
/// by specification (the two Schottky diodes, the 21 indicator LEDs, the
/// eight current regulators, the polarity FET and the transistor — phase
/// 3); the push button is a switch, the three-pad jumper `JP1` a two-pole
/// switch, and the four mounting holes mechanical nodes. The 29-root
/// cluster is the bare board's `GND` with everything its elements join to
/// it: the twelve LED chains whose cathodes sit on ground (anode net and
/// inverter output each), the polarity FET's drain and source nets
/// through its gate on ground, the charge-pump opto's LED anode net and
/// the pin it is driven from — the 47-net collapse `NODES.md` §2 rule 1
/// describes. Phase 4 took the count from 169 to 176: the bucks' output
/// inductors are identity unions, so `+5V` and `+3.3V` are the nodes of
/// `U1`/`U2`'s output terminals — clusters of their own — and the nine
/// chains sourced from `+3.3V`, one 19-root cluster while a 0 Ω edge from
/// the switch node made the rail their member, are nine two-net clusters.
/// With the bench rails in, ground is a terminal and the FET a one-net
/// cluster.
const EDGE: Census = Census {
    clusters: 176,
    largest_cluster_roots: 29,
    stub_count: 0,
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
#[rustfmt::skip]
const EC32MB_MODELLED: &[&str] = &[
    "U100", "U301", "X100", "U101", "U601", "U302", "U303", "U304", "U305",
    // The power tree: the two bucks, the eight LDOs, the detector.
    "U402", "U403", "U501", "U502", "U503", "U504", "U505", "U506", "U507", "U508", "U404",
];
#[rustfmt::skip]
const EDGE_MODELLED: &[&str] = &[
    "U24", "U25", "IC5", "IC1", "IC2", "IC14", "IC15", "IC16",
    // The power tree: the two bucks and the two isolated DC/DCs.
    "U1", "U2", "IC3", "IC4",
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

/// The largest cluster of a built system: its root count and its net
/// names, sorted.
fn largest_cluster(built: &BuiltSystem) -> (usize, BTreeSet<&str>) {
    let clusters = built.cluster_roots();
    let largest = clusters
        .iter()
        .max_by_key(|roots| roots.len())
        .expect("a board has clusters");
    let names: BTreeSet<&str> = largest
        .iter()
        .map(|id| built.nets()[id.0].name.as_str())
        .collect();
    (largest.len(), names)
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
    let (_, largest) = largest_cluster(&built);

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
        "a cluster is what resistors, inductors and closed jumpers join, ending at a declared \
         terminal, so its count is a property of the drawing and changes only when the model \
         of a part changes"
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

// ============================================================
// Rule 4's bound, under the reference harnesses
// ============================================================

/// The `m ≤ 8` every board is held to under its reference harness
/// (`DESIGN.md` rule 4, "Enforced by").
const RULE_4_LARGEST_CLUSTER_ROOTS: usize = 8;

/// An LED chain under the bench rails: the inverter output and the LED's
/// other end, and nothing else (`NODES.md` §6, "every LED chain {Y,
/// anode}").
const LED_CHAIN_ROOTS: usize = 2;
const _: () = assert!(LED_CHAIN_ROOTS <= RULE_4_LARGEST_CLUSTER_ROOTS);

/// The Edge board under the bench rails alone: its largest cluster is the
/// P2 bank rail `VIO_16_23` — a socket finger the module sources — with the
/// eight opto outputs `P16`–`P23` pulled up to it through `R1`–`R8`. The
/// bench has no supply for it (the rail is the module's LDO `U503`), so on
/// the board alone it is a 9-root cluster of an unsourced rail and its
/// pull-ups — **above rule 4's bound**, which is why the bound is asserted
/// with the finger sourced ([`edge_reference_rails`]) and with the module
/// in its socket, where the rail is a terminal and each pull-up net a
/// cluster of its own. A never-rises figure with its reason, not a bound.
const EDGE_UNDER_RAILS_LARGEST_CLUSTER_ROOTS: usize = 9;

/// The bound the build's escalations are held under on the Edge board
/// under the bench rails: the element clusters' cold-start solves and their
/// fixed-point re-solves. Phase 3 read 48 with the nine `+3.3V` chains one
/// 20-root solve; the phase-4 engine half 72 with each chain its own
/// two-root one — more solves, each a nineteenth the size; the parts half
/// 63, the rails real: `+3.3V` is its buck's terminal node from the round
/// the buck publishes in (the XL1509 names no soft-start), and the chains
/// solve against it as a constant.
const EDGE_UNDER_RAILS_BUILD_SOLVES: u64 = 80;

/// The nine indicator chains sourced from `+3.3V` on the Edge board, by
/// the inverter output net that sinks each — `Net-(Dn-K)` — with the
/// anode net `Net-(Dn-A)` on the other side of the LED.
const EDGE_SOURCED_CHAINS: &[u32] = &[7, 10, 11, 13, 16, 18, 20, 22, 24];

/// The Edge board under its bench rails (`machine_parts::bench_rails`, the
/// state most board tests run it in — the 12 V input and the servo domain,
/// with `+3.3V` the board's own buck `U2`, and the module's socket empty):
/// every chain sourced from the `+3.3V` rail is the two-root cluster the
/// plan sized it at, the rail itself — one node with the buck's output
/// through `L2` — a cluster of its own, and the largest cluster on the
/// board is the P2 bank rail the module would source, above the bound and
/// held as a figure that may only fall.
#[rstest]
fn the_edge_board_under_the_bench_rails_has_two_root_led_chains_and_a_largest_cluster_that_never_grows(
) {
    behaviour!(Test {
        id: "census.edge-under-rails-largest-cluster",
        covers: Some("board/src/system.rs#BuiltSystem::cluster_roots"),
        given: "the MaD EdgeBoard under the bench rails every board test runs it in, with no \
                scenario",
    });
    expect!(
        "led-chains-are-two-roots",
        "each of the nine chains sourced from the 3.3 volt rail is a cluster of exactly its \
         LED's two ends",
        "a declared terminal is a cluster boundary for a resistor edge as it is for an \
         element: the series resistor ends on the rail and unions nothing through it"
    );
    expect!(
        "rail-alone",
        "the 3.3 volt rail is a cluster of its own: one node with its buck's output through \
         the output inductor, and nothing else",
        "an inductor is a DC short — one node — so the rail is the buck's declared terminal, \
         a boundary of every cluster that hangs off it"
    );
    expect!(
        "largest-cluster",
        "the largest cluster is the module-sourced P2 bank rail with its eight pulled-up opto \
         outputs, exactly the committed nine nodes",
        "the bench supplies no module rail, so on the board alone that rail is an unsourced \
         finger its pull-ups meet on — above the bound, which is asserted with the finger \
         sourced; with a source on it the rail is a terminal, and the figure may only fall"
    );
    expect!(
        "solves-at-build-bounded",
        "the build escalates at most eighty solves: the element clusters' cold-start solves \
         and their fixed-point re-solves",
        "a rail is a constant of the solves that stamp against it and asks for none of its own"
    );
    let built = System::new()
        .board("EdgeBoard", edge_board())
        .harness(bench_rails("EdgeBoard"))
        .build()
        .expect("the Edge board builds under the bench rails");
    let clusters = built.cluster_roots();
    // The cluster holding a net's node: a cluster lists identity roots,
    // and a net merged into another (the rail into its buck's output
    // through the inductor) is named by whichever net is the root.
    let cluster_of = |net: &str| -> BTreeSet<String> {
        let id = built.net_id(net).unwrap_or_else(|| panic!("{net} exists"));
        clusters
            .iter()
            .find(|roots| roots.iter().any(|root| built.nets_are_merged(*root, id)))
            .unwrap_or_else(|| panic!("{net} is in a cluster"))
            .iter()
            .map(|id| built.nets()[id.0].name.clone())
            .collect()
    };
    for d in EDGE_SOURCED_CHAINS {
        let chain = cluster_of(&format!("EdgeBoard.Net-(D{d}-K)"));
        let expected: BTreeSet<String> = [
            format!("EdgeBoard.Net-(D{d}-A)"),
            format!("EdgeBoard.Net-(D{d}-K)"),
        ]
        .into_iter()
        .collect();
        assert_eq!(chain, expected, "the D{d} chain");
        assert_eq!(chain.len(), LED_CHAIN_ROOTS);
    }
    let rail = cluster_of("EdgeBoard.+3.3V");
    assert_eq!(rail.len(), 1, "the rail is a cluster of its own: {rail:?}");
    assert!(
        built.names_are_merged("EdgeBoard.+3.3V", "EdgeBoard.Net-(D2-K)"),
        "the rail and the buck's output are one node through L2"
    );
    let (largest, names) = largest_cluster(&built);
    eprintln!(
        "census EdgeBoard under bench_rails: clusters={} largest_cluster_roots={largest} \
         escalated_solves_at_build={}\n  largest cluster: {names:?}",
        clusters.len(),
        built.escalated_solves(),
    );
    assert_eq!(
        largest, EDGE_UNDER_RAILS_LARGEST_CLUSTER_ROOTS,
        "the largest cluster under the rails — this figure may only fall; the phase that \
         lowers it changes the constant and says so"
    );
    assert!(names.contains("EdgeBoard.VIO_16_23"), "{names:?}");
    for p in 16..=23 {
        assert!(
            names.contains(format!("EdgeBoard.P{p}").as_str()),
            "{names:?}"
        );
    }
    assert!(
        built.escalated_solves() <= EDGE_UNDER_RAILS_BUILD_SOLVES,
        "{} escalated solves at build",
        built.escalated_solves()
    );
}

fn ep(endpoint: &str) -> EndpointRef {
    EndpointRef::parse(endpoint).expect("endpoint parses")
}

/// The module powered the way a carrier powers it: 5 V into the two `5V`
/// fingers and 0 V into the three `GND` fingers of `J203`.
fn module_carrier_rails(module: &str) -> Harness {
    Harness::new()
        .power(ep("CARRIER.5V"), ep(&format!("{module}.J203.41")), 5.0)
        .power(ep("CARRIER.5Vb"), ep(&format!("{module}.J203.42")), 5.0)
        .power(ep("CARRIER.GND"), ep(&format!("{module}.J203.43")), 0.0)
        .power(ep("CARRIER.GNDb"), ep(&format!("{module}.J203.44")), 0.0)
        .power(ep("CARRIER.GNDc"), ep(&format!("{module}.J203.45")), 0.0)
}

/// The socket finger the Edge board's `VIO_16_23` arrives on: `J3` pin 58
/// (`mad_edge.net`, net 190, pin function `V16`), which the module's LDO
/// `U503` sources ("IC REG LDO CMOS 3.3V UDFN4 (VIO_16_23)",
/// `p2_ec32mb.net`).
const EDGE_VIO_16_23_FINGER: &str = "EdgeBoard.J3.58";

/// The Edge board's reference harness on the bench alone: the bench rails
/// (`machine_parts::bench_rails`, the 12 V input and the servo domain)
/// plus the one socket finger a module in the socket would source, held
/// at that LDO's 3.3 V — the voltage `U503`'s value names (`LDO 300mA,
/// 3.3V`), a declared harness terminal standing in for the part
/// (`DESIGN.md` rule 6: named by the netlist, not invented). The eight
/// pull-ups `R1`–`R8` end on it, each `P16`–`P23` net a cluster of its
/// own, as they are with the module in the socket.
fn edge_reference_rails(edge: &str) -> Harness {
    bench_rails(edge).power(ep("MODULE.U503_OUT"), ep(EDGE_VIO_16_23_FINGER), 3.3)
}

/// Each board under its reference harness — the EC32MB from its `J203`
/// fingers, the DS2 add-on under its force-domain rails, the Edge board
/// alone under the bench rails with its module-sourced socket finger held
/// at the module LDO's 3.3 V ([`edge_reference_rails`]), and the Edge
/// board with the module in its socket under the bench rails, the force
/// domain's references held over the cable (its 5 V is the board's own
/// `IC4`). Under the bench rails alone, that finger unsourced, the Edge
/// board's largest cluster is the module's bank rail, above.
fn reference_systems() -> Vec<(&'static str, BuiltSystem)> {
    let _module = machine_parts::lock_module_instance();
    vec![
        (
            "EC32MB from its J203 fingers",
            System::new()
                .board("EC32MB", shipped_ec32mb_board())
                .harness(module_carrier_rails("EC32MB"))
                .build()
                .expect("the module builds from its fingers"),
        ),
        (
            "DS2Addon under force_domain_rails",
            System::new()
                .board("DS2Addon", ds2_board())
                .harness(force_domain_rails("DS2Addon"))
                .build()
                .expect("the add-on builds under its rails"),
        ),
        (
            "EdgeBoard alone under bench_rails with its VIO_16_23 finger at the module LDO's 3.3 V",
            System::new()
                .board("EdgeBoard", edge_board())
                .harness(edge_reference_rails("EdgeBoard"))
                .build()
                .expect("the Edge board builds under its reference rails"),
        ),
        (
            "EdgeBoard with the module in its socket under bench_rails",
            System::new()
                .board("EC32MB", ec32mb_board())
                .board("EdgeBoard", edge_board())
                .board("DS2Addon", ds2_board())
                .harness(module_socket_harness("EC32MB", "EdgeBoard"))
                .harness(force_gauge_harness("EdgeBoard", "DS2Addon"))
                .harness(bench_rails("EdgeBoard"))
                .harness(force_domain_ground("DS2Addon"))
                .build()
                .expect("the machine builds"),
        ),
    ]
}

/// Rule 4's bound: `m ≤ 8` on every board under its reference harness —
/// its declared supplies, every one a terminal.
#[rstest]
fn every_board_under_its_reference_harness_is_within_rule_4s_bound() {
    behaviour!(Test {
        id: "census.rule-4-bound",
        covers: Some("board/src/engine.rs#Resolver::build_topology"),
        given: "each reference board under its declared supplies: the module from its \
                fingers, the add-on under its rails, the EdgeBoard alone with its \
                module-sourced finger at 3.3 volts, and the assembled machine",
    });
    expect!(
        "largest-cluster-at-most-eight",
        "the largest conduction cluster of each of the four systems holds at most eight \
         electrical nodes",
        "every declared terminal — a regulator output, a bench supply, a stuck net — is a \
         cluster of its own and a boundary of every cluster around it, so no rail joins its \
         loads into one solve"
    );
    for (name, built) in reference_systems() {
        let (largest, names) = largest_cluster(&built);
        eprintln!(
            "census {name}: clusters={} largest_cluster_roots={largest} \
             escalated_solves_at_build={}\n  largest cluster: {names:?}",
            built.cluster_roots().len(),
            built.escalated_solves(),
        );
        assert!(
            largest <= RULE_4_LARGEST_CLUSTER_ROOTS,
            "{name}: the largest cluster holds {largest} roots: {names:?}"
        );
    }
}

/// The figure `DESIGN.md` rule 1 is held to: no board carries a part with
/// nothing behind it. Every registered part is a model or an element by
/// specification, and every other part a primitive the auto tier
/// classifies — on all three boards, since phase 4 took the last facades.
#[rstest]
#[case::ec32mb("EC32MB", shipped_ec32mb_board as fn() -> Board, EC32MB_MODELLED, &["U401"])]
#[case::edge("EdgeBoard", edge_board as fn() -> Board, EDGE_MODELLED, &[])]
#[case::ds2("DS2Addon", ds2_board as fn() -> Board, DS2_MODELLED, &[])]
fn no_board_contains_a_stub_part(
    #[case] name: &str,
    #[case] build: fn() -> Board,
    #[case] modelled: &[&str],
    #[case] _unused: &[&str],
) {
    behaviour!(Test {
        id: "census.every-part-a-node",
        covers: Some("board/src/board.rs#Board::nodes"),
        given: "one of the three reference boards built from its vendor netlist with the \
                registry that ships it",
    });
    expect!(
        "no-stubs",
        "every registered part of the board is a named model, and every other part is a \
         primitive of a class the board build knows",
        "a netlist part is a node whose class has behaviour, or the board refuses to build \
         naming it; a registered facade without behaviour would be neither"
    );

    let board = build();
    assert_eq!(
        stubs_of(name, &board, modelled),
        Vec::<String>::new(),
        "{name}: parts with nothing behind them"
    );
    let unexpected: Vec<(&str, &PartClass)> = board
        .nodes()
        .filter(|(reference, class)| match class {
            PartClass::Passive { .. }
            | PartClass::Jumper { .. }
            | PartClass::Switch { .. }
            | PartClass::Boundary
            | PartClass::Mechanical
            | PartClass::Probe
            | PartClass::Pwl { .. } => false,
            PartClass::Registered { .. } => !modelled.contains(reference),
        })
        .collect();
    assert_eq!(unexpected, Vec::<(&str, &PartClass)>::new());
}
