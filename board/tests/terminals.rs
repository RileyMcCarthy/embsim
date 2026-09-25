//! Terminals as cluster boundaries — the engine half of `NODES.md` §8
//! phase 4, on the wire.
//!
//! A declared terminal — a `PowerOut` pin's net, a harness `power(V)`
//! endpoint, a `net_stuck` — is a cluster of its own whose state is decided
//! once from its sources, and a boundary of every cluster around it: a
//! resistor or an element ends on it, nothing unions through it, and what
//! it holds enters each dependent's solve as a constant and each
//! dependent's ranking as an ideal source through the path to it. Membership
//! is fixed at build, so the terminal is declared whatever it holds; what it
//! holds is the one thing that moves at run time, and when it does the
//! engine re-resolves the terminal's own cluster and every cluster in its
//! fan-out. These cases hold that on a bench board and on the module:
//!
//! * a `PowerOut` pin's declared idle drive is what its rail holds before
//!   the part publishes — released (floating, its loads unsourced), a
//!   voltage (exact on the rail, a pull through the resistor to it), or the
//!   `PinDecl::power_out`'s default, the unmodelled rail a facade still
//!   declares;
//! * a part that drives its `PowerOut` pin live re-resolves the nets that
//!   read the rail through a resistor **and** through a diode — the
//!   fan-out the phase-3 element clusters' foreign constants were missing;
//! * a released rail accepts a bench strap onto its net without a fight;
//! * two declared sources that disagree on one terminal — a rail against a
//!   `net_stuck` — are one fight, reported once at the terminal, and every
//!   dependent reads the fight's operating point;
//! * a current instrument is refused on a `PowerOut` pin;
//! * on the P2-EC32MB with a bench ground and the core rail held from the
//!   bench — the configuration the phase-2 record measured at 30
//!   escalated solves, one per P59 edge — a pad toggling P59 two hundred
//!   times escalates nothing: the feedback divider is a cluster of its
//!   own between two terminals, solved once, and P59's is another.
//!
//! Stepped mode throughout (`TESTING.md` rule 9), own binary.

use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};

use embsim_board::{
    jesd8c01_lvcmos_thresholds, AttachError, Board, BoardError, Component, ComponentNetIo,
    DeadBand, EndpointRef, Finding, Harness, JumperState, Level, NetState, PartRegistry, PinDecl,
    PinHandle, PwlSpec, Scenario, SenseKind, System, SystemError, TheveninDrive,
};
use embsim_boards::ec32mb::{FLASH_SELECT_SWITCH, P59_PULL_DOWN_POLE};
use embsim_core::virtual_clock::{self, ClockMode};
use rstest::rstest;
use vibes_behaviour::{behaviour, expect, Test};

mod machine_parts;

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

/// The voltage a state reads, or a panic naming what it read instead.
fn volts(state: Option<NetState>) -> f64 {
    match state {
        Some(NetState::Analog(v)) => v,
        other => panic!("expected an analog voltage, got {other:?}"),
    }
}

/// A rail: one `PowerOut` pin, `OUT`, declared at `idle`, that publishes
/// the drives of `script` at their instants — the shape a regulator model
/// takes, without the datasheet.
struct Rail {
    pins: [PinDecl; 1],
    script: Vec<(u64, Option<TheveninDrive>)>,
    handle: Arc<Mutex<Option<PinHandle>>>,
}

/// What a rail's declaration says it idles at: a declared idle drive
/// (`None` released), or what [`PinDecl::power_out`] declares when the
/// declaration names none.
#[derive(Debug, Clone, Copy)]
enum Idle {
    Declared(Option<TheveninDrive>),
    ConstructorDefault,
}

impl Rail {
    fn new(idle: Idle, script: Vec<(u64, Option<TheveninDrive>)>) -> Self {
        let pin = PinDecl::power_out("OUT");
        Self {
            pins: [match idle {
                Idle::Declared(drive) => pin.with_idle(drive),
                Idle::ConstructorDefault => pin,
            }],
            script,
            handle: Arc::new(Mutex::new(None)),
        }
    }
}

impl Component for Rail {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }
    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let out = io.pin("OUT")?;
        *self.handle.lock().unwrap() = Some(out.clone());
        let script = self.script.clone();
        for &(at_ns, _) in &script {
            io.schedule_at_ns(at_ns);
        }
        io.on_wake_ns(move |now_ns| {
            for (at_ns, drive) in &script {
                if *at_ns == now_ns {
                    out.set_drive(*drive);
                }
            }
        });
        Ok(())
    }
}

/// A rail whose attach asks for the current into its own output — the
/// instrument a terminal refuses.
struct InstrumentedRail {
    pins: [PinDecl; 1],
}

impl Component for InstrumentedRail {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }
    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        io.on_branch("OUT", |_| {})
    }
}

/// A digital input that records every state it is delivered.
struct Sensor {
    pins: [PinDecl; 1],
    seen: Arc<Mutex<Vec<NetState>>>,
}

impl Component for Sensor {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }
    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let seen = Arc::clone(&self.seen);
        io.on_net_report("1", move |state| seen.lock().unwrap().push(state))
    }
}

/// A bench pad the test drives from its own thread.
struct Pad {
    pins: [PinDecl; 1],
    handle: Arc<Mutex<Option<PinHandle>>>,
}

impl Component for Pad {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }
    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        *self.handle.lock().unwrap() = Some(io.pin("P")?);
        Ok(())
    }
}

/// The bench board: a rail feeding a sensed node through 10 kΩ and a
/// second sensed node through a diode with a 10 kΩ pull-down to a ground
/// the scenario declares.
const RAIL_NETLIST: &str = r#"(export (version "E")
  (components
    (comp (ref "U1") (value "rail") (libsource (lib "Bench") (part "RAIL")))
    (comp (ref "U2") (value "sensor") (libsource (lib "Bench") (part "SENSOR")))
    (comp (ref "U3") (value "sensor") (libsource (lib "Bench") (part "SENSOR")))
    (comp (ref "D1") (value "diode") (libsource (lib "Bench") (part "DIODE")))
    (comp (ref "R1") (value "10k") (libsource (lib "Device") (part "R")))
    (comp (ref "R2") (value "10k") (libsource (lib "Device") (part "R"))))
  (nets
    (net (code "1") (name "RAIL") (node (ref "U1") (pin "OUT")) (node (ref "R1") (pin "1")) (node (ref "D1") (pin "A")))
    (net (code "2") (name "LOAD") (node (ref "R1") (pin "2")) (node (ref "U2") (pin "1")))
    (net (code "3") (name "LED") (node (ref "D1") (pin "K")) (node (ref "R2") (pin "1")) (node (ref "U3") (pin "1")))
    (net (code "4") (name "GND") (node (ref "R2") (pin "2")))))"#;

/// The bench diode: a 0.75 V knee with a vertical on-segment.
const DIODE_VF: f64 = 0.75;

type Seen = Arc<Mutex<Vec<NetState>>>;

/// The bench board with `rail` as `U1`, returning the sensors' logs
/// (`LOAD`, `LED`) and the rail's pin handle.
fn rail_board(rail: Rail) -> (Board, Seen, Seen, Arc<Mutex<Option<PinHandle>>>) {
    let load: Seen = Arc::default();
    let led: Seen = Arc::default();
    let handle = Arc::clone(&rail.handle);
    let rail = Arc::new(Mutex::new(Some(rail)));
    let mut registry = PartRegistry::new();
    registry.register("RAIL", move |_| {
        Box::new(
            rail.lock()
                .unwrap()
                .take()
                .expect("the fixture places one rail"),
        )
    });
    {
        let (load, led) = (Arc::clone(&load), Arc::clone(&led));
        registry.register("SENSOR", move |decl| {
            Box::new(Sensor {
                pins: [PinDecl::digital_in(
                    "1",
                    jesd8c01_lvcmos_thresholds(DeadBand::Unknown),
                )],
                seen: if decl.reference == "U2" {
                    Arc::clone(&load)
                } else {
                    Arc::clone(&led)
                },
            })
        });
    }
    registry.register_pwl("DIODE", PwlSpec::diode("A", "K", DIODE_VF, 0.0));
    let board = Board::from_netlist(parse(RAIL_NETLIST), &registry).expect("classifies");
    (board, load, led, handle)
}

fn grounded() -> Scenario {
    Scenario::default().net_stuck("B.GND", 0.0)
}

// ============================================================
// The cases
// ============================================================

/// What a `PowerOut` pin's declared idle drive makes of its rail before
/// the part publishes anything.
#[rstest]
#[case::released(Idle::Declared(None))]
#[case::three_volts_three(Idle::Declared(Some(TheveninDrive { volts: 3.3, impedance: 0.1 })))]
#[case::kind_default(Idle::ConstructorDefault)]
fn a_power_out_pins_idle_drive_is_what_its_rail_holds_before_the_part_publishes(
    #[case] idle: Idle,
) {
    behaviour!(Test {
        id: "terminal.power-out-idle-drive",
        covers: Some("board/src/system.rs#add_pin_descriptor"),
        given: "a power-out pin declaring an idle drive — released, 3.3 volts, or the \
                constructor's default — feeding one sensed node through 10 kilohms and another through a \
                diode to a declared ground",
    });
    expect!(
        "released-rail-floats",
        "a released rail reads floating, the resistor-fed node floats and is reported as a \
         floating input, and the diode-fed node sits at its ground",
        "a released terminal sources nothing: it is declared, a cluster of its own and a \
         boundary, but holds no voltage for its dependents to rank or stamp"
    );
    expect!(
        "declared-voltage-is-exact",
        "a rail declared at 3.3 volts reads exactly that, its resistor-fed node pulled high \
         through the 10 kilohms, and its diode-fed node one knee below",
        "a terminal holding a voltage enters every dependent solve as a constant, so the rail \
         net is its voltage to the bit and its dependents rank it as an ideal source"
    );
    expect!(
        "default-is-unmodelled",
        "the default idle drive reads pulled high through nothing, its resistor-fed node \
         pulled high through the 10 kilohms, and nothing is reported for its loads",
        "the default is the unmodelled rail a facade declares: sourced at a voltage no model \
         names, presented as up through the path to it"
    );
    let (board, _, _, _) = rail_board(Rail::new(idle, Vec::new()));
    let built = System::new()
        .board("B", board)
        .scenario(grounded())
        .build()
        .expect("builds");
    let state = |name: &str| built.nets()[built.net_id(name).unwrap().0].state;
    let findings = built.diagnostics().findings();
    match idle {
        Idle::Declared(None) => {
            assert_eq!(state("B.RAIL"), NetState::Floating);
            assert_eq!(state("B.LOAD"), NetState::Floating);
            assert!(
                findings.contains(&Finding::FloatingSense {
                    net: "B.LOAD".to_string(),
                    kind: SenseKind::Digital,
                }),
                "{findings:?}"
            );
            assert!(
                (volts(Some(state("B.LED")))).abs() < 1e-9,
                "{:?}",
                state("B.LED")
            );
        }
        Idle::Declared(Some(drive)) => {
            assert_eq!(state("B.RAIL"), NetState::Analog(drive.volts));
            assert_eq!(state("B.LOAD"), NetState::Pulled(Level::High, 10_000.0));
            let led = volts(Some(state("B.LED")));
            assert!((led - (drive.volts - DIODE_VF)).abs() < 1e-6, "{led}");
            assert!(
                !findings
                    .iter()
                    .any(|f| matches!(f, Finding::FloatingSense { .. })),
                "{findings:?}"
            );
        }
        Idle::ConstructorDefault => {
            assert_eq!(state("B.RAIL"), NetState::Pulled(Level::High, 0.0));
            assert_eq!(state("B.LOAD"), NetState::Pulled(Level::High, 10_000.0));
            assert!(
                !findings
                    .iter()
                    .any(|f| matches!(f, Finding::FloatingSense { .. })),
                "{findings:?}"
            );
        }
    }
    assert!(
        !findings
            .iter()
            .any(|f| matches!(f, Finding::Contention { .. })),
        "{findings:?}"
    );
}

/// A part that drives its `PowerOut` pin live: the rail's fan-out is
/// re-resolved on every publish, the resistor path and the diode path
/// alike.
#[rstest]
fn a_rail_that_publishes_live_re_resolves_every_cluster_that_reads_it() {
    behaviour!(Test {
        id: "terminal.rail-publish-fan-out",
        covers: Some("board/src/engine.rs#Resolver::mark_terminal_dirty"),
        given: "a released rail feeding one sensed node through 10 kilohms and another through \
                a diode to a declared ground, whose part drives 3.3 volts at one millisecond \
                and releases at two",
    });
    expect!(
        "resistor-path-follows",
        "the resistor-fed sense is delivered floating at registration, pulled high through \
         the 10 kilohms at one millisecond, and floating again at two",
        "a rail is a terminal whose fan-out names every cluster an edge reaches it from; a \
         change to what it holds dirties them all"
    );
    expect!(
        "diode-path-follows",
        "the diode-fed sense is delivered its ground at registration, one knee below the \
         rail at one millisecond, and its ground again at two",
        "an element cluster stamps the rail as a foreign constant, and is in the fan-out too \
         — the cluster the phase-3 property test showed was never re-resolved"
    );
    expect!(
        "rail-net-follows",
        "the rail's own net reads floating, then exactly 3.3 volts, then floating",
    );
    let _guard = stepped();
    let high = TheveninDrive {
        volts: 3.3,
        impedance: 0.1,
    };
    let (board, load, led, _) = rail_board(Rail::new(
        Idle::Declared(None),
        vec![(1_000_000, Some(high)), (2_000_000, None)],
    ));
    let live = System::new()
        .board("B", board)
        .scenario(grounded())
        .start()
        .expect("starts");
    assert!(
        wait_for(|| load.lock().unwrap().len() >= 3, SETTLE),
        "the load sense saw {:?}",
        load.lock().unwrap()
    );
    assert!(
        wait_for(|| led.lock().unwrap().len() >= 3, SETTLE),
        "the led sense saw {:?}",
        led.lock().unwrap()
    );
    assert_eq!(
        *load.lock().unwrap(),
        vec![
            NetState::Floating,
            NetState::Pulled(Level::High, 10_000.0),
            NetState::Floating,
        ]
    );
    let led = led.lock().unwrap().clone();
    assert_eq!(led.len(), 3, "{led:?}");
    assert!(volts(Some(led[0])).abs() < 1e-9, "{led:?}");
    assert!(
        (volts(Some(led[1])) - (3.3 - DIODE_VF)).abs() < 1e-6,
        "{led:?}"
    );
    assert!(volts(Some(led[2])).abs() < 1e-9, "{led:?}");
    assert_eq!(live.net_state("B.RAIL"), Some(NetState::Floating));
    live.shutdown();
}

/// A released rail and a bench strap on its net: the strap sources it and
/// nothing fights.
#[rstest]
#[case::released(Idle::Declared(None))]
#[case::unmodelled(Idle::ConstructorDefault)]
fn a_released_rail_accepts_a_bench_strap_without_a_fight(#[case] idle: Idle) {
    behaviour!(Test {
        id: "terminal.released-rail-takes-a-strap",
        covers: Some("board/src/engine.rs#decide_terminal"),
        given: "a rail that is released, or unmodelled, with a bench supply of 3.3 volts \
                strapped onto its net",
    });
    expect!(
        "strap-sources-the-rail",
        "the rail net reads exactly 3.3 volts and its resistor-fed load pulled high through \
         the 10 kilohms",
        "a terminal's sources are reconciled once: a released or unmodelled source holds \
         nothing against a declared voltage, so the strap is the rail"
    );
    expect!("nothing-reported", "no contention is reported anywhere",);
    let (board, _, _, _) = rail_board(Rail::new(idle, Vec::new()));
    let built = System::new()
        .board("B", board)
        .harness(Harness::new().power(ep("BENCH.3V3"), ep("B.U1.OUT"), 3.3))
        .scenario(grounded())
        .build()
        .expect("builds");
    let state = |name: &str| built.nets()[built.net_id(name).unwrap().0].state;
    assert_eq!(state("B.RAIL"), NetState::Analog(3.3));
    assert_eq!(state("B.LOAD"), NetState::Pulled(Level::High, 10_000.0));
    let findings = built.diagnostics().findings();
    assert!(
        !findings
            .iter()
            .any(|f| matches!(f, Finding::Contention { .. })),
        "{findings:?}"
    );
}

/// Two declared sources that disagree on one terminal are one fight,
/// decided at the terminal and reported once — not once per dependent.
#[rstest]
fn two_sources_that_disagree_on_a_terminal_fight_once() {
    behaviour!(Test {
        id: "terminal.fought-once",
        covers: Some("board/src/engine.rs#Resolver::resolve_terminal"),
        given: "a rail declared at 3.3 volts with a short to 0 volts injected on its net, the \
                net feeding one node through a resistor and another through a diode",
    });
    expect!(
        "one-contention",
        "exactly one contention finding is reported, on the rail net, naming no pin, with \
         one ambiguous-level finding at the fight's 1.65 volts beside it",
        "the terminal's state is assigned once at its own cluster; its dependents read the \
         decision and report nothing of their own"
    );
    expect!("terminal-in-contention", "the rail net reads contention",);
    expect!(
        "dependents-read-the-operating-point",
        "the resistor-fed node reads the fight's 1.65 volts through its pull, and the \
         diode-fed node one knee below it",
        "a fought terminal holds the fight's own operating point for every cluster that reads it"
    );
    expect!(
        "two-solves",
        "the build escalates exactly two solves: the fight's own, and the diode cluster's",
        "a fought terminal is decided by one solve of its own one-node cluster; the element \
         cluster solves as every element cluster does; the resistor-fed node projects"
    );
    let (board, _, _, _) = rail_board(Rail::new(
        Idle::Declared(Some(TheveninDrive {
            volts: 3.3,
            impedance: 0.1,
        })),
        Vec::new(),
    ));
    let built = System::new()
        .board("B", board)
        .scenario(grounded().net_stuck("B.RAIL", 0.0))
        .build()
        .expect("builds");
    let state = |name: &str| built.nets()[built.net_id(name).unwrap().0].state;
    let findings = built.diagnostics().findings();
    let contention: Vec<&Finding> = findings
        .iter()
        .filter(|f| matches!(f, Finding::Contention { .. }))
        .collect();
    assert_eq!(
        contention,
        vec![&Finding::Contention {
            net: "B.RAIL".to_string(),
            drivers: Vec::new(),
        }],
        "{findings:?}"
    );
    let ambiguous: Vec<&Finding> = findings
        .iter()
        .filter(|f| matches!(f, Finding::AmbiguousLevel { .. }))
        .collect();
    assert_eq!(ambiguous.len(), 1, "{findings:?}");
    let Finding::AmbiguousLevel { net, volts: mid } = ambiguous[0] else {
        unreachable!()
    };
    assert_eq!(net, "B.RAIL");
    assert!((mid - 1.65).abs() < 1e-9, "{mid}");
    assert_eq!(state("B.RAIL"), NetState::Contention);
    let load = volts(Some(state("B.LOAD")));
    assert!((load - 1.65).abs() < 1e-9, "{load}");
    let led = volts(Some(state("B.LED")));
    assert!((led - (1.65 - DIODE_VF)).abs() < 1e-6, "{led}");
    assert_eq!(built.escalated_solves(), 2);
}

/// A current instrument on a `PowerOut` pin is refused at attach: a
/// terminal's current spans clusters.
#[rstest]
fn a_current_instrument_is_refused_on_a_power_out_pin() {
    behaviour!(Test {
        id: "terminal.no-instrument-on-a-terminal",
        covers: Some("board/src/component.rs#PinHandle::carries_current"),
        given: "a part whose attach subscribes to the current into its own power-out pin",
    });
    expect!(
        "attach-refused",
        "the build fails at the part's attach, naming it",
        "a terminal's current is the sum over every cluster it bounds, which no one cluster's \
         solve accounts for"
    );
    let mut registry = PartRegistry::new();
    registry.register("RAIL", |_| {
        Box::new(InstrumentedRail {
            pins: [PinDecl::power_out("OUT")],
        })
    });
    registry.register("SENSOR", |_| {
        Box::new(Sensor {
            pins: [PinDecl::digital_in(
                "1",
                jesd8c01_lvcmos_thresholds(DeadBand::Unknown),
            )],
            seen: Arc::default(),
        })
    });
    registry.register_pwl("DIODE", PwlSpec::diode("A", "K", DIODE_VF, 0.0));
    let board = Board::from_netlist(parse(RAIL_NETLIST), &registry).expect("classifies");
    let error = System::new()
        .board("B", board)
        .scenario(grounded())
        .build()
        .expect_err("refused");
    assert!(
        matches!(
            &error,
            SystemError::Board {
                error: BoardError::Attach { reference, .. },
                ..
            } if reference == "U1"
        ),
        "{error:?}"
    );
}

/// The module with a bench ground on its fingers and the core rail held
/// from the bench: the configuration the phase-2 record measured and
/// refused at 30 escalated solves — the feedback divider re-solved on every
/// P59 edge, because ground, the core rail and the P59 pull-down were one
/// cluster. With terminals as boundaries the divider is a cluster of its
/// own between two terminals, solved once, and P59's net is another, a pad
/// against a pull: two hundred edges escalate nothing.
#[rstest]
fn the_p59_pull_down_projects_beside_a_held_core_rail_on_the_module() {
    behaviour!(Test {
        id: "terminal.p59-projects-beside-held-core-rail",
        covers: Some("board/src/engine.rs#Resolver::build_topology"),
        given: "the P2-EC32MB with a bench ground on its fingers, its core rail held at 1.8 \
                volts, DIP position 4 closed, and a bench pad toggling P59 two hundred times",
    });
    expect!(
        "divider-solved",
        "the buck's feedback node reads the divider's 0.794 volts from the build on",
        "1.8 volts across the 13.3 and 10.5 kilohm feedback resistors"
    );
    expect!(
        "edges-escalate-nothing",
        "the escalated-solve count is the same before and after the two hundred edges",
        "the divider's cluster is bounded by the two terminals it hangs between and shares no \
         cluster with the pad, so a P59 edge dirties nothing of it"
    );
    expect!(
        "p59-projects",
        "P59 reads driven to each level the pad drives and pulled low through the 10.5 \
         kilohm pull-down when the pad releases, with no solve",
        "a 25 ohm pad against a 10.5 kilohm pull to a terminal is a projection"
    );
    let _guard = stepped();
    let _module = machine_parts::lock_module_instance();
    let pad = Pad {
        pins: [PinDecl::digital_out("P")],
        handle: Arc::default(),
    };
    let pad_handle = Arc::clone(&pad.handle);
    let live = System::new()
        .board("EC32MB", machine_parts::ec32mb_board())
        .component("PAD", Box::new(pad))
        .harness(
            Harness::new()
                .power(ep("CARRIER.GND"), ep("EC32MB.J203.43"), 0.0)
                .connect(ep("PAD.P"), ep("EC32MB.J203.53")),
        )
        .scenario(
            Scenario::default()
                .switch(
                    &format!("EC32MB.{FLASH_SELECT_SWITCH}"),
                    P59_PULL_DOWN_POLE,
                    JumperState::Closed,
                )
                .net_stuck("EC32MB.Common_VDD", 1.8),
        )
        .start()
        .expect("starts");
    let pad = pad_handle
        .lock()
        .unwrap()
        .clone()
        .expect("the pad attached");
    assert!(
        wait_for(
            || live.net_state("EC32MB.P2_IO59") == Some(NetState::Driven(Level::High)),
            SETTLE
        ),
        "{:?}",
        live.net_state("EC32MB.P2_IO59")
    );
    // R401 13.3 kΩ over R403 10.5 kΩ (`p2_ec32mb.net`).
    let divider = 1.8 * 10.5 / (13.3 + 10.5);
    let fb = volts(live.net_state("EC32MB.Net-(U402-FB)"));
    assert!((fb - divider).abs() < 1e-3, "{fb} against {divider}");
    assert_eq!(
        live.net_state("EC32MB.Common_VDD"),
        Some(NetState::Analog(1.8))
    );
    // The count is read after the first edge has landed, so the engine's
    // own start-up pass — a full one, the divider's solve included — is
    // behind it.
    let pad_at = |volts: f64| {
        pad.set_drive(Some(TheveninDrive {
            volts,
            impedance: 25.0,
        }));
    };
    pad_at(0.0);
    assert!(
        wait_for(
            || live.net_state("EC32MB.P2_IO59") == Some(NetState::Driven(Level::Low)),
            SETTLE
        ),
        "{:?}",
        live.net_state("EC32MB.P2_IO59")
    );
    let before = live.escalated_solves();
    for i in 0..200 {
        pad_at(if i % 2 == 0 { 3.3 } else { 0.0 });
    }
    pad.release();
    assert!(
        wait_for(
            || live.net_state("EC32MB.P2_IO59") == Some(NetState::Pulled(Level::Low, 10_500.0)),
            SETTLE
        ),
        "{:?}",
        live.net_state("EC32MB.P2_IO59")
    );
    let after = live.escalated_solves();
    eprintln!("escalated solves: {before} before the edges, {after} after");
    assert_eq!(after, before, "the edges must escalate nothing");
    let fb = volts(live.net_state("EC32MB.Net-(U402-FB)"));
    assert!((fb - divider).abs() < 1e-3, "{fb} against {divider}");
    live.shutdown();
}
