//! Piecewise-linear elements on the bench — `NODES.md` §8 phase 3's proof
//! for the solver half: a diode registered from the library conducts and
//! blocks; a switched channel follows its control in both directions; a
//! pair of elements that chase each other is reported non-convergent with
//! their nodes floating; two histories reaching one drive table publish
//! identical states, because every solve starts cold; a node only leakage
//! reaches floats; the current instrument reads what a sink carries; the
//! build snapshot equals the live system's state for a cluster with
//! elements.
//!
//! Every fixture is a bench: a hand-written netlist (or a bench component)
//! with its rails as `net_stuck` terminals, so the numbers are the
//! datasheet's and the test's own. Stepped mode (`TESTING.md` rule 9), its
//! own binary (rule 5).

use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};

use embsim_board::{
    digital_drive, Amps, AttachError, Board, BoardError, Component, ComponentNetIo, EndpointRef,
    Finding, Harness, IdleDrive, Level, NetState, PartRegistry, PinDecl, PinHandle, PwlCurve,
    PwlSpec, RegionTest, Scenario, System, SystemError, PWL_SOLVES_PER_ELEMENT,
};
use embsim_core::virtual_clock::{self, ClockMode};
use embsim_models::pwl_library::{self, SS36_VF_VOLTS};
use rstest::rstest;
use vibes_behaviour::{behaviour, expect, Test};

// ============================================================
// Plumbing
// ============================================================

static SUITE_LOCK: Mutex<()> = Mutex::new(());

fn suite_lock() -> MutexGuard<'static, ()> {
    SUITE_LOCK.lock().unwrap_or_else(|poisoned| {
        SUITE_LOCK.clear_poison();
        poisoned.into_inner()
    })
}

fn stepped() -> MutexGuard<'static, ()> {
    let guard = suite_lock();
    virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);
    guard
}

fn wait_for(mut pred: impl FnMut() -> bool, timeout: Duration) -> bool {
    let start = Instant::now();
    while start.elapsed() < timeout {
        if pred() {
            return true;
        }
        std::thread::sleep(Duration::from_millis(1));
    }
    pred()
}

const SETTLE: Duration = Duration::from_secs(5);

fn ep(endpoint: &str) -> EndpointRef {
    EndpointRef::parse(endpoint).expect("endpoint parses")
}

fn parse(netlist: &str) -> embsim_board::ParsedNetlist {
    embsim_board::netlist::parse(netlist).expect("the fixture parses")
}

/// The voltage a net reads, or a panic naming what it read instead.
fn volts(state: Option<NetState>) -> f64 {
    match state {
        Some(NetState::Analog(v)) => v,
        other => panic!("expected an analog voltage, got {other:?}"),
    }
}

/// Bitwise equality of two states — the identity the cold start promises.
fn same(a: Option<NetState>, b: Option<NetState>) -> bool {
    match (a, b) {
        (Some(NetState::Analog(x)), Some(NetState::Analog(y))) => x.total_cmp(&y).is_eq(),
        (Some(NetState::Pulled(la, xa)), Some(NetState::Pulled(lb, xb))) => {
            la == lb && xa.total_cmp(&xb).is_eq()
        }
        (a, b) => a == b,
    }
}

/// A pin the test drives from its own thread; idles driven high.
struct Driver {
    pins: [PinDecl; 1],
    handle: Arc<Mutex<Option<PinHandle>>>,
}

impl Driver {
    fn new() -> (Self, Arc<Mutex<Option<PinHandle>>>) {
        let handle = Arc::new(Mutex::new(None));
        (
            Self {
                pins: [PinDecl::digital_out("Q")],
                handle: Arc::clone(&handle),
            },
            handle,
        )
    }
}

impl Component for Driver {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        *self.handle.lock().unwrap() = Some(io.pin("Q")?);
        Ok(())
    }
}

fn handle_of(slot: &Arc<Mutex<Option<PinHandle>>>) -> PinHandle {
    slot.lock().unwrap().clone().expect("attached")
}

// ============================================================
// A diode from the library
// ============================================================

/// One SS36 from the library — registered by the manufacturer part number
/// the export carries, its value being the family name — fed through
/// 220 Ω between two terminals. `S` is the resistor's far end, `A` the
/// anode net, `K` the cathode net.
const DIODE_NETLIST: &str = r#"(export (version "E")
  (components
    (comp (ref "D1") (value "SS36") (libsource (lib "Diode") (part "D_Schottky_SMC"))
      (property (name "Manufacturer_Part_Number") (value "SS36-E3/57T")))
    (comp (ref "R1") (value "220") (libsource (lib "Device") (part "R"))))
  (nets
    (net (code "1") (name "S") (node (ref "R1") (pin "1")))
    (net (code "2") (name "A") (node (ref "R1") (pin "2")) (node (ref "D1") (pin "2") (pinfunction "A")))
    (net (code "3") (name "K") (node (ref "D1") (pin "1") (pinfunction "K")))))"#;

fn diode_board() -> Board {
    let mut registry = PartRegistry::new();
    pwl_library::register(&mut registry);
    Board::from_netlist(parse(DIODE_NETLIST), &registry).expect("the library classifies the diode")
}

/// Forward: the anode sits at the knee and the resistor sets the current.
/// Reversed: the diode is off, the anode net at its own source, and the
/// current is leakage.
#[rstest]
#[case::forward(3.3, 0.0)]
#[case::reversed(0.0, 3.3)]
fn a_library_diode_conducts_forward_and_blocks_reversed(
    #[case] s_volts: f64,
    #[case] k_volts: f64,
) {
    behaviour!(Test {
        id: "pwl.library-diode-conducts-and-blocks",
        covers: Some("board/src/engine.rs#Resolver::resolve_cluster"),
        given: "a Schottky diode from the element library, keyed by the manufacturer part number \
                in the export, fed through 220 ohms between a 3.3 volt terminal and a 0 volt \
                one, first forward and then reversed",
    });
    expect!(
        "forward-current",
        "forward biased, the diode carries the supply less its datasheet knee divided by the \
         resistor, within one percent, and its anode net sits at the knee",
        "a conducting diode drops its forward voltage and the resistor takes the rest"
    );
    expect!(
        "reversed-blocks",
        "reverse biased, the anode net sits at its own terminal and the diode carries under \
         ten nanoamps",
        "an off element conducts only its leakage"
    );
    expect!(
        "pin-currents",
        "the current into the anode pin is the branch current and the current into the \
         cathode pin its negative",
        "a branch's current enters at one pin and leaves at the other"
    );
    let _guard = stepped();
    let system = System::new()
        .board("B", diode_board())
        .scenario(
            Scenario::default()
                .net_stuck("B.S", s_volts)
                .net_stuck("B.K", k_volts),
        )
        .start()
        .expect("the bench starts");
    assert!(
        wait_for(|| system.branch_current("B.D1").is_some(), SETTLE),
        "the cluster solves"
    );
    let current = system.branch_current("B.D1").unwrap();
    let anode = volts(system.net_state("B.A"));
    if s_volts > k_volts {
        let expected = (3.3 - SS36_VF_VOLTS) / 220.0;
        assert!(
            (current - expected).abs() < expected * 0.01,
            "{current} vs {expected}"
        );
        assert!((anode - SS36_VF_VOLTS).abs() < 1e-3, "{anode}");
    } else {
        assert!(current.abs() < 10e-9, "{current}");
        // Its own terminal, less the leakage drop (3.3 nA through 220 Ω).
        assert!((anode - s_volts).abs() < 1e-5, "{anode}");
    }
    let into_anode = system.pin_current("B.D1.2").unwrap();
    let into_cathode = system.pin_current("B.D1.1").unwrap();
    assert!(into_anode.total_cmp(&current).is_eq());
    assert!(into_cathode.total_cmp(&(-current)).is_eq());
    assert!(system.escalated_solves() >= 1, "an element cluster solves");
}

/// A part whose manufacturer part number and value match nothing in the
/// library is not a node when its symbol is no primitive either: the board
/// refuses to build and names it. A diode primitive's symbol with no entry
/// keeps the open passive class it always had.
#[rstest]
fn a_diode_the_library_does_not_hold_is_an_unknown_part() {
    behaviour!(Test {
        id: "pwl.unknown-diode-refused",
        covers: Some("board/src/registry.rs#PartRegistry::classify"),
        given: "a diode with a manufacturer part number and a value the element library does \
                not hold, once with a symbol no primitive matches and once with a diode symbol",
    });
    expect!(
        "refused-by-name",
        "for the unknown symbol the board does not build and the error names the part and its \
         value",
        "a part is a node whose class has behaviour, and nothing invents a diode's knee"
    );
    expect!(
        "refused-by-number",
        "with a diode symbol the build is refused too, and the error names the part and the \
         manufacturer part number the export gave it",
        "a diode symbol carries no forward drop, and the number is the key a library entry \
         for the purchasable part would take"
    );
    let unknown = DIODE_NETLIST
        .replace("SS36-E3/57T", "1N5819HW-7-F")
        .replace("\"SS36\"", "\"1N5819\"");
    let mut registry = PartRegistry::new();
    pwl_library::register(&mut registry);
    let error = Board::from_netlist(
        parse(&unknown.replace("D_Schottky_SMC", "1N5819")),
        &registry,
    )
    .expect_err("nothing classifies it");
    let rendered = error.to_string();
    for needle in ["D1", "1N5819"] {
        assert!(rendered.contains(needle), "{rendered:?} lacks {needle}");
    }
    let error = Board::from_netlist(parse(&unknown), &registry)
        .expect_err("a diode symbol with no entry is an unknown part");
    let rendered = error.to_string();
    for needle in ["D1", "1N5819HW-7-F"] {
        assert!(rendered.contains(needle), "{rendered:?} lacks {needle}");
    }
}

// ============================================================
// A switched channel with a control pin
// ============================================================

/// A switched channel: its drain pulled up through 1 kΩ to a terminal, its
/// source on another, its gate on a connector the bench drives. Not a
/// part — the numbers are the fixture's own.
const CHANNEL_NETLIST: &str = r#"(export (version "E")
  (components
    (comp (ref "Q1") (value "SWITCH") (libsource (lib "Bench") (part "SWITCH")))
    (comp (ref "R1") (value "1k") (libsource (lib "Device") (part "R")))
    (comp (ref "J1") (value "Conn_01x01") (libsource (lib "Connector") (part "Conn_01x01"))))
  (nets
    (net (code "1") (name "VDD") (node (ref "R1") (pin "1")))
    (net (code "2") (name "DRAIN") (node (ref "R1") (pin "2")) (node (ref "Q1") (pin "D")))
    (net (code "3") (name "SRC") (node (ref "Q1") (pin "S")))
    (net (code "4") (name "GATE") (node (ref "Q1") (pin "G")) (node (ref "J1") (pin "1")))))"#;

/// The channel's on test: the fixture's threshold, 2 V past the source
/// either way.
const V_TH: f64 = 2.0;

fn channel_board(test: RegionTest) -> Board {
    let mut registry = PartRegistry::new();
    registry.register_pwl(
        "SWITCH",
        PwlSpec::new(["D", "G", "S"]).with_controlled_branch(
            "D",
            "S",
            PwlCurve::Channel { r_on: 1.0 },
            "G",
            test,
        ),
    );
    Board::from_netlist(parse(CHANNEL_NETLIST), &registry).expect("the switch classifies")
}

/// A running channel bench: the system, the gate driver's handle, and the
/// rails it was built with.
struct ChannelBench {
    system: embsim_board::SystemHandle,
    gate: PinHandle,
    vdd: f64,
    src: f64,
}

/// Build the bench for one test polarity: an N-type test has its source
/// at 0 V and the pull-up to 3.3 V; a P-type test the reverse.
fn channel_bench(test: RegionTest) -> ChannelBench {
    let (vdd, src) = match test {
        RegionTest::AtLeast(_) => (3.3, 0.0),
        RegionTest::AtMost(_) => (0.0, 3.3),
    };
    let (driver, slot) = Driver::new();
    let system = System::new()
        .board("B", channel_board(test))
        .component("G", Box::new(driver))
        .harness(Harness::new().connect(ep("B.J1.1"), ep("G.Q")))
        .scenario(
            Scenario::default()
                .net_stuck("B.VDD", vdd)
                .net_stuck("B.SRC", src),
        )
        .start()
        .expect("the bench starts");
    let gate = handle_of(&slot);
    ChannelBench {
        system,
        gate,
        vdd,
        src,
    }
}

/// The drain reads within the on-resistance's share of the pull-up (1 Ω
/// against 1 kΩ) of the source.
fn drain_is_on(bench: &ChannelBench) -> bool {
    matches!(bench.system.net_state("B.DRAIN"), Some(NetState::Analog(v)) if (v - bench.src).abs() < 3.3 / 1_000.0 * 1.001)
}

/// The drain reads at the pull-up's rail, within the off leakage.
fn drain_is_off(bench: &ChannelBench) -> bool {
    matches!(bench.system.net_state("B.DRAIN"), Some(NetState::Analog(v)) if (v - bench.vdd).abs() < 1e-5)
}

/// The channel conducts when the gate-to-source voltage passes the declared
/// test and blocks when it does not, in both directions of the test and in
/// both directions of a change.
#[rstest]
#[case::n_type(RegionTest::AtLeast(V_TH))]
#[case::p_type(RegionTest::AtMost(-V_TH))]
fn a_channel_follows_its_gate_both_ways(#[case] test: RegionTest) {
    behaviour!(Test {
        id: "pwl.channel-follows-its-gate",
        covers: Some("board/src/cluster.rs#QuasiStaticMna::solve"),
        given: "a switched channel, its output pulled through 1 kilohm to one rail and its source \
                on the other, with the bench driving its gate past the declared threshold, then \
                short of it, then past it again",
    });
    expect!(
        "on-when-the-test-passes",
        "with the gate past the threshold the output sits at the source, within the \
         on-resistance's share of the pull-up",
    );
    expect!(
        "off-when-it-does-not",
        "with the gate short of the threshold the output sits at the pull-up's rail",
        "an off channel carries only leakage"
    );
    expect!(
        "on-again",
        "driven past the threshold again the output returns to the source",
        "every solve chooses the region afresh, so a channel that was off turns on the moment \
         its control says so"
    );
    let _guard = stepped();
    let bench = channel_bench(test);
    let (on_level, off_level) = match test {
        RegionTest::AtLeast(_) => (Level::High, Level::Low),
        RegionTest::AtMost(_) => (Level::Low, Level::High),
    };
    bench.gate.set_drive(Some(digital_drive(on_level)));
    assert!(
        wait_for(|| drain_is_on(&bench), SETTLE),
        "{:?}",
        bench.system.net_state("B.DRAIN")
    );
    bench.gate.set_drive(Some(digital_drive(off_level)));
    assert!(
        wait_for(|| drain_is_off(&bench), SETTLE),
        "{:?}",
        bench.system.net_state("B.DRAIN")
    );
    bench.gate.set_drive(Some(digital_drive(on_level)));
    assert!(
        wait_for(|| drain_is_on(&bench), SETTLE),
        "{:?}",
        bench.system.net_state("B.DRAIN")
    );
}

/// Two histories that end at the same drive table publish the same states
/// and the same currents, bit for bit: one toggles the gate off and on
/// again, the other never leaves the on rail. A warm-started region loop
/// would let the path taken show in the answer.
#[rstest]
fn two_histories_reaching_one_drive_table_publish_identical_states() {
    behaviour!(Test {
        id: "pwl.cold-start-forgets-the-history",
        covers: Some("board/src/cluster.rs#QuasiStaticMna::solve"),
        given: "two copies of the switched-channel bench, one whose gate is driven on, off and \
                on again, the other whose gate is driven on once",
    });
    expect!(
        "identical-states",
        "every net of the two benches reads the same state, bit for bit",
        "every solve starts with every element off and chooses regions in declaration order, \
         so the operating point is a function of the drive table alone"
    );
    expect!(
        "identical-currents",
        "the channel carries the same current in both, bit for bit",
    );
    let _guard = stepped();
    let test = RegionTest::AtLeast(V_TH);
    let toggled = channel_bench(test);
    let direct = channel_bench(test);
    for level in [Level::High, Level::Low, Level::High] {
        toggled.gate.set_drive(Some(digital_drive(level)));
        let expect_on = level == Level::High;
        assert!(wait_for(
            || if expect_on {
                drain_is_on(&toggled)
            } else {
                drain_is_off(&toggled)
            },
            SETTLE
        ));
    }
    direct.gate.set_drive(Some(digital_drive(Level::High)));
    assert!(wait_for(|| drain_is_on(&direct), SETTLE));
    for net in ["B.VDD", "B.DRAIN", "B.SRC", "B.GATE", "G.Q"] {
        assert!(
            same(toggled.system.net_state(net), direct.system.net_state(net)),
            "{net}: {:?} vs {:?}",
            toggled.system.net_state(net),
            direct.system.net_state(net)
        );
    }
    let (a, b) = (
        toggled.system.branch_current("B.Q1").unwrap(),
        direct.system.branch_current("B.Q1").unwrap(),
    );
    assert!(a.total_cmp(&b).is_eq(), "{a} vs {b}");
}

/// The build snapshot of a cluster with elements equals the live system's
/// state before its first wake: the build stamps the elements too.
#[rstest]
fn the_build_snapshot_equals_the_live_state_for_a_cluster_with_elements() {
    behaviour!(Test {
        id: "pwl.build-snapshot-equals-live",
        covers: Some("board/src/system.rs#System::build"),
        given: "the switched-channel bench, its gate idling high, analyzed at build and then \
                started live with virtual time held",
    });
    expect!(
        "same-states",
        "every net's live state is exactly the state the build snapshot recorded for it",
        "build and live share one resolver, and both stamp the elements and choose their \
         regions the same way"
    );
    expect!(
        "same-current",
        "the channel's current is the same in the snapshot and live",
    );
    let _guard = stepped();
    let test = RegionTest::AtLeast(V_TH);
    let build = |driver: Driver| {
        System::new()
            .board("B", channel_board(test))
            .component("G", Box::new(driver))
            .harness(Harness::new().connect(ep("B.J1.1"), ep("G.Q")))
            .scenario(
                Scenario::default()
                    .net_stuck("B.VDD", 3.3)
                    .net_stuck("B.SRC", 0.0),
            )
    };
    let built = build(Driver::new().0).build().expect("builds");
    let live = build(Driver::new().0).hold_time().start().expect("starts");
    let mismatches = || -> Vec<(String, NetState, Option<NetState>)> {
        built
            .nets()
            .iter()
            .enumerate()
            .filter(|(i, net)| !same(Some(net.state), live.net_state_of(embsim_board::NetId(*i))))
            .map(|(i, net)| {
                (
                    net.name.clone(),
                    net.state,
                    live.net_state_of(embsim_board::NetId(i)),
                )
            })
            .collect()
    };
    assert!(
        wait_for(|| mismatches().is_empty(), SETTLE),
        "{:?}",
        mismatches()
    );
    // The gate idles high, so the channel is on in both: the drain at the
    // source within the on-resistance's share, and the pull-up's current
    // through the channel — the same figure from the snapshot and live.
    let drain = volts(live.net_state("B.DRAIN"));
    assert!(drain < 3.3 / 1_000.0 * 1.001, "{drain}");
    let snapshot = built
        .branch_current("B.Q1")
        .expect("the build solved the cluster");
    let live_current = live
        .branch_current("B.Q1")
        .expect("the engine solved the cluster");
    assert!(
        snapshot.total_cmp(&live_current).is_eq(),
        "{snapshot} vs {live_current}"
    );
    assert!((snapshot - 3.3 / 1_001.0).abs() < 1e-6, "{snapshot}");
}

// ============================================================
// Non-convergence
// ============================================================

/// Two channels that chase each other: the first turns on when the second's
/// node is high, the second when the first's node is low.
const CHASE_NETLIST: &str = r#"(export (version "E")
  (components
    (comp (ref "Q1") (value "N") (libsource (lib "Bench") (part "N")))
    (comp (ref "Q2") (value "P") (libsource (lib "Bench") (part "P")))
    (comp (ref "R1") (value "1k") (libsource (lib "Device") (part "R")))
    (comp (ref "R2") (value "1k") (libsource (lib "Device") (part "R"))))
  (nets
    (net (code "1") (name "VDD") (node (ref "R1") (pin "1")) (node (ref "R2") (pin "1")))
    (net (code "2") (name "P") (node (ref "R1") (pin "2")) (node (ref "Q1") (pin "D")) (node (ref "Q2") (pin "G")))
    (net (code "3") (name "Q") (node (ref "R2") (pin "2")) (node (ref "Q2") (pin "D")) (node (ref "Q1") (pin "G")))
    (net (code "4") (name "GND") (node (ref "Q1") (pin "S")) (node (ref "Q2") (pin "S")))))"#;

fn chase_board() -> Board {
    let channel = |test| {
        PwlSpec::new(["D", "G", "S"]).with_controlled_branch(
            "D",
            "S",
            PwlCurve::Channel { r_on: 1.0 },
            "G",
            test,
        )
    };
    let mut registry = PartRegistry::new();
    registry.register_pwl("N", channel(RegionTest::AtLeast(1.5)));
    registry.register_pwl("P", channel(RegionTest::AtMost(1.5)));
    Board::from_netlist(parse(CHASE_NETLIST), &registry).expect("classifies")
}

/// Two elements whose region tests chase each other have no operating
/// point: the loop runs its bound and the cluster is reported, its nodes
/// floating, nothing NaN, at build and live alike.
#[rstest]
fn elements_that_chase_each_other_are_non_convergent_and_float() {
    behaviour!(Test {
        id: "pwl.non-convergence-reported",
        covers: Some("board/src/engine.rs#Resolver::resolve_cluster"),
        given: "two switched channels each pulling its own node down from a 3.3 volt terminal, \
                the first turned on by the second's node being high and the second by the first's \
                being low",
    });
    expect!(
        "reported",
        "the cluster is reported non-convergent, naming both channels and a solve count of two \
         per element",
        "the region loop is bounded at two solves per element, so a pair with no rest state \
         is found out at a fixed cost and named"
    );
    expect!(
        "nodes-float",
        "both switched nodes float while the terminals keep their voltages",
        "a cluster with no operating point publishes no voltage for the nodes the elements \
         decide"
    );
    expect!(
        "no-nan-no-current",
        "no net carries a non-finite voltage and neither channel reports a current",
    );
    expect!(
        "build-and-live-agree",
        "the build snapshot and the live system report the same",
    );
    let _guard = stepped();
    let scenario = || {
        Scenario::default()
            .net_stuck("B.VDD", 3.3)
            .net_stuck("B.GND", 0.0)
    };
    let built = System::new()
        .board("B", chase_board())
        .scenario(scenario())
        .build()
        .expect("builds");
    let finding = Finding::NonConvergent {
        cluster: "B.VDD".to_string(),
        elements: vec!["B.Q1".to_string(), "B.Q2".to_string()],
        solves: PWL_SOLVES_PER_ELEMENT * 2,
    };
    assert!(
        built.diagnostics().contains(&finding),
        "{:?}",
        built.diagnostics().findings()
    );
    let state_of = |name: &str| built.nets()[built.net_id(name).unwrap().0].state;
    assert_eq!(state_of("B.P"), NetState::Floating);
    assert_eq!(state_of("B.Q"), NetState::Floating);
    assert_eq!(state_of("B.VDD"), NetState::Analog(3.3));
    assert_eq!(state_of("B.GND"), NetState::Analog(0.0));
    for net in built.nets() {
        if let NetState::Analog(v) = net.state {
            assert!(v.is_finite(), "{}: {v}", net.name);
        }
    }
    assert_eq!(built.branch_current("B.Q1"), None);
    assert_eq!(built.branch_current("B.Q2"), None);

    let live = System::new()
        .board("B", chase_board())
        .scenario(scenario())
        .start()
        .expect("starts");
    assert!(wait_for(|| live.findings().contains(&finding), SETTLE));
    assert_eq!(live.net_state("B.P"), Some(NetState::Floating));
    assert_eq!(live.net_state("B.Q"), Some(NetState::Floating));
    assert_eq!(live.branch_current("B.Q1"), None);
}

/// The chase with a rail nobody has modelled a path away from it: `R3`
/// from the first channel's node to a `PowerOut` pin sourced at no voltage.
const CHASE_WITH_STUB_RAIL_NETLIST: &str = r#"(export (version "E")
  (components
    (comp (ref "Q1") (value "N") (libsource (lib "Bench") (part "N")))
    (comp (ref "Q2") (value "P") (libsource (lib "Bench") (part "P")))
    (comp (ref "R1") (value "1k") (libsource (lib "Device") (part "R")))
    (comp (ref "R2") (value "1k") (libsource (lib "Device") (part "R")))
    (comp (ref "R3") (value "1k") (libsource (lib "Device") (part "R")))
    (comp (ref "U1") (value "RAIL") (libsource (lib "Bench") (part "RAIL"))))
  (nets
    (net (code "1") (name "VDD") (node (ref "R1") (pin "1")) (node (ref "R2") (pin "1")))
    (net (code "2") (name "P") (node (ref "R1") (pin "2")) (node (ref "Q1") (pin "D")) (node (ref "Q2") (pin "G")) (node (ref "R3") (pin "2")))
    (net (code "3") (name "Q") (node (ref "R2") (pin "2")) (node (ref "Q2") (pin "D")) (node (ref "Q1") (pin "G")))
    (net (code "4") (name "GND") (node (ref "Q1") (pin "S")) (node (ref "Q2") (pin "S")))
    (net (code "5") (name "RAIL") (node (ref "R3") (pin "1")) (node (ref "U1") (pin "1")))))"#;

/// A rail nobody has modelled: one `PowerOut` pin, sourced at no voltage.
struct StubRail {
    pins: [PinDecl; 1],
}

impl Component for StubRail {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, _io: ComponentNetIo) -> Result<(), AttachError> {
        Ok(())
    }
}

fn chase_board_with_stub_rail() -> Board {
    let channel = |test| {
        PwlSpec::new(["D", "G", "S"]).with_controlled_branch(
            "D",
            "S",
            PwlCurve::Channel { r_on: 1.0 },
            "G",
            test,
        )
    };
    let mut registry = PartRegistry::new();
    registry.register_pwl("N", channel(RegionTest::AtLeast(1.5)));
    registry.register_pwl("P", channel(RegionTest::AtMost(1.5)));
    registry.register("RAIL", |_| {
        Box::new(StubRail {
            pins: [PinDecl::power_out("1")],
        })
    });
    Board::from_netlist(parse(CHASE_WITH_STUB_RAIL_NETLIST), &registry).expect("classifies")
}

/// A cluster with no operating point floats every node the elements
/// decide, a path to an unmodelled rail notwithstanding: the rail's "up
/// through the path" presentation is for a node nothing numeric reaches,
/// and a non-convergent solve is no voltage at all.
#[rstest]
fn a_non_convergent_cluster_floats_even_where_an_unmodelled_rail_reaches() {
    behaviour!(Test {
        id: "pwl.non-convergence-floats-past-unmodelled-rail",
        covers: Some("board/src/engine.rs#Resolver::resolve_cluster"),
        given: "the pair of switched channels that chase each other, with a resistor from the \
                first channel's node to a power-output pin of a part whose rail voltage nothing \
                models",
    });
    expect!(
        "nodes-float",
        "both switched nodes float, at build and once live",
        "a cluster whose region loop finds no operating point publishes no voltage and no \
         level for the nodes its elements decide; a rail of unknown voltage presents as up \
         only on a node nothing numeric reaches"
    );
    expect!(
        "reported",
        "the cluster is reported non-convergent, naming both channels",
    );
    let _guard = stepped();
    let scenario = || {
        Scenario::default()
            .net_stuck("B.VDD", 3.3)
            .net_stuck("B.GND", 0.0)
    };
    let built = System::new()
        .board("B", chase_board_with_stub_rail())
        .scenario(scenario())
        .build()
        .expect("builds");
    let state_of = |name: &str| built.nets()[built.net_id(name).unwrap().0].state;
    assert_eq!(state_of("B.P"), NetState::Floating);
    assert_eq!(state_of("B.Q"), NetState::Floating);
    assert!(
        built.diagnostics().findings().iter().any(|f| matches!(
            f,
            Finding::NonConvergent { elements, .. }
                if elements == &["B.Q1".to_string(), "B.Q2".to_string()]
        )),
        "{:?}",
        built.diagnostics().findings()
    );

    let live = System::new()
        .board("B", chase_board_with_stub_rail())
        .scenario(scenario())
        .start()
        .expect("starts");
    assert!(wait_for(
        || {
            live.findings()
                .iter()
                .any(|f| matches!(f, Finding::NonConvergent { .. }))
        },
        SETTLE
    ));
    assert_eq!(live.net_state("B.P"), Some(NetState::Floating));
    assert_eq!(live.net_state("B.Q"), Some(NetState::Floating));
}

// ============================================================
// Leakage only
// ============================================================

/// A diode whose cathode net has nothing else on it: the node is reached
/// only through the off diode's leakage, and floats.
#[rstest]
fn a_node_reached_only_through_leakage_floats() {
    behaviour!(Test {
        id: "pwl.leakage-only-node-floats",
        covers: Some("board/src/cluster.rs#QuasiStaticMna::solve"),
        given: "a library diode whose anode net is held at 3.3 volts by a terminal and whose \
                cathode net carries nothing else",
    });
    expect!(
        "cathode-floats",
        "the cathode net floats",
        "an off element's gigaohm keeps the solve well posed but is not a source; a node with \
         a hundred megohms or more behind it has no voltage"
    );
    expect!("no-current", "the diode carries no current");
    let _guard = stepped();
    let system = System::new()
        .board("B", diode_board())
        .scenario(Scenario::default().net_stuck("B.A", 3.3))
        .start()
        .expect("starts");
    assert!(wait_for(
        || system.net_state("B.K") == Some(NetState::Floating)
            && system.branch_current("B.D1").is_some(),
        SETTLE
    ));
    assert_eq!(system.net_state("B.K"), Some(NetState::Floating));
    assert_eq!(system.branch_current("B.D1"), Some(0.0));
}

// ============================================================
// The current instrument
// ============================================================

/// A sink on a pulled-up line, with the instrument on its own pin.
struct Sink {
    pins: [PinDecl; 1],
    handle: Arc<Mutex<Option<PinHandle>>>,
    readings: Arc<Mutex<Vec<Option<Amps>>>>,
}

impl Component for Sink {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let readings = Arc::clone(&self.readings);
        io.on_branch("Q", move |amps| readings.lock().unwrap().push(amps))?;
        *self.handle.lock().unwrap() = Some(io.pin("Q")?);
        Ok(())
    }
}

/// A 3.3 V terminal through 1 kΩ to a connector the sink hangs on.
const PULL_UP_NETLIST: &str = r#"(export (version "E")
  (components
    (comp (ref "R1") (value "1k") (libsource (lib "Device") (part "R")))
    (comp (ref "J1") (value "Conn_01x01") (libsource (lib "Connector") (part "Conn_01x01"))))
  (nets
    (net (code "1") (name "VDD") (node (ref "R1") (pin "1")))
    (net (code "2") (name "LINE") (node (ref "R1") (pin "2")) (node (ref "J1") (pin "1")))))"#;

/// The instrument on a sink holding a pulled-up line low reads the current
/// the resistor pushes into it, and it is what escalates the cluster.
#[rstest]
fn the_current_into_a_low_sink_is_the_pull_ups_current() {
    behaviour!(Test {
        id: "pwl.sense-current-on-a-sink",
        covers: Some("board/src/component.rs#PinHandle::sense_current"),
        given: "a pin sinking a line low through 25 ohms, the line pulled to 3.3 volts through \
                1 kilohm, with the current instrument subscribed on the sinking pin",
    });
    expect!(
        "current-matches",
        "the current into the pin equals the supply less the line's voltage, divided by the \
         pull-up, to a nanoamp",
        "the instrument reads the same solve the line's voltage comes from"
    );
    expect!(
        "delivered-on-change",
        "the instrument is delivered its reading at registration and again when the sink \
         releases the line, and the released reading is zero",
    );
    expect!(
        "escalated",
        "the line's cluster solves once the instrument is on it",
        "only a solved cluster has node voltages to take a current from"
    );
    let _guard = stepped();
    let handle = Arc::new(Mutex::new(None));
    let readings: Arc<Mutex<Vec<Option<Amps>>>> = Arc::new(Mutex::new(Vec::new()));
    let sink = Sink {
        pins: [PinDecl::digital_out("Q").with_idle(IdleDrive::Thevenin(digital_drive(Level::Low)))],
        handle: Arc::clone(&handle),
        readings: Arc::clone(&readings),
    };
    let board = Board::from_netlist(parse(PULL_UP_NETLIST), &PartRegistry::new()).unwrap();
    let system = System::new()
        .board("B", board)
        .component("SINK", Box::new(sink))
        .harness(Harness::new().connect(ep("B.J1.1"), ep("SINK.Q")))
        .scenario(Scenario::default().net_stuck("B.VDD", 3.3))
        .start()
        .expect("starts");
    let settled = wait_for(
        || {
            readings
                .lock()
                .unwrap()
                .last()
                .is_some_and(|r| r.is_some_and(|a| a > 1e-3))
        },
        SETTLE,
    );
    assert!(settled, "{:?}", readings.lock().unwrap());
    let line = volts(system.net_state("B.LINE"));
    let expected = (3.3 - line) / 1_000.0;
    let read = system.pin_current("SINK.Q").unwrap();
    assert!((read - expected).abs() < 1e-9, "{read} vs {expected}");
    let through_handle = handle_of(&handle).sense_current().unwrap();
    assert!(through_handle.total_cmp(&read).is_eq());
    // The hand value: 3.3 V across 1 kΩ + 25 Ω.
    assert!((read - 3.3 / 1_025.0).abs() < 1e-9, "{read}");
    assert!(system.escalated_solves() >= 1);

    handle_of(&handle).release();
    assert!(wait_for(
        || readings.lock().unwrap().last() == Some(&Some(0.0)),
        SETTLE
    ));
    let delivered = readings.lock().unwrap().clone();
    assert!(delivered.len() >= 2, "{delivered:?}");
    assert_eq!(delivered.last(), Some(&Some(0.0)));
}

/// A pin with nothing the solve accounts a current for — a power pin on no
/// branch — refuses the instrument at attach, and the system does not
/// start.
#[rstest]
fn the_instrument_is_refused_on_a_pin_that_carries_no_current() {
    behaviour!(Test {
        id: "pwl.sense-current-refused-on-a-terminal",
        covers: Some("board/src/component.rs#ComponentNetIo::on_branch"),
        given: "a bench part that subscribes the current instrument on its supply pin, which has \
                no drive and terminates no declared branch",
    });
    expect!(
        "refused-at-attach",
        "the system does not start and the error names the part and the pin",
        "a supply pin's current spans clusters and the solve has nothing to account it with"
    );
    struct OnSupply {
        pins: [PinDecl; 1],
    }
    impl Component for OnSupply {
        fn pins(&self) -> &[PinDecl] {
            &self.pins
        }
        fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
            io.on_branch("VCC", |_| {})
        }
    }
    let _guard = stepped();
    let error = System::new()
        .component(
            "X",
            Box::new(OnSupply {
                pins: [PinDecl::power_in("VCC")],
            }),
        )
        .start()
        .expect_err("refused");
    assert!(
        matches!(
            &error,
            SystemError::Board {
                name,
                error: BoardError::Attach { reference, error: AttachError::Failed { message } }
            } if name == "X" && reference == "X" && message.contains("VCC")
        ),
        "{error:?}"
    );
}

/// The kinds a pin declares still decide what it is: an element's pins are
/// passive terminals and a channel's control is a passive terminal too.
#[rstest]
fn an_elements_pins_are_passive_terminals_of_its_branch() {
    behaviour!(Test {
        id: "pwl.element-pins-are-branch-terminals",
        covers: Some("board/src/board.rs#Board::from_netlist"),
        given: "a switched channel registered by specification with its three pins named",
    });
    expect!(
        "pins-declared",
        "the board records the part as an element whose specification names its three pins",
        "an element's facade is its pin list, validated against the netlist both ways"
    );
    let board = channel_board(RegionTest::AtLeast(V_TH));
    match board.node_class("Q1") {
        Some(embsim_board::PartClass::Pwl { spec }) => {
            assert_eq!(spec.pins, vec!["D", "G", "S"]);
            assert_eq!(spec.branches.len(), 1);
        }
        other => panic!("{other:?}"),
    }
}
