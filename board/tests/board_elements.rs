//! The nonlinear parts on the reference boards as the elements they are —
//! `NODES.md` §8 phase 3's proof for the parts half: an indicator LED on
//! the EdgeBoard lit by its inverter at the current its series resistor
//! sets; the polarity FET passing the input forward and blocking it
//! reversed with no `pin_short` standing in; an end-switch loop regulated
//! at the current regulator's 10 mA; an optocoupler's output sinking only
//! above its input threshold; the servo-enable transistor saturating under
//! a light load and sagging under a heavy one, at the base current its
//! resistor sets.
//!
//! Every fixture is the real EdgeBoard (`fixtures/mad_edge.net`) under the
//! bench rails `machine_parts::bench_rails` describes, or a hand-written
//! bench netlist; every number is a datasheet's or the fixture's own. The
//! EC32MB's power tree is not asked to read volts here: its rails are phase
//! 4's, and the module's polarity FET is proven from its carrier fingers in
//! `ec32mb_module.rs`. Stepped mode (`TESTING.md` rule 9), its own binary
//! (rule 5).

mod machine_parts;

use std::sync::{Mutex, MutexGuard};
use std::time::{Duration, Instant};

use embsim_board::{
    Board, Finding, Harness, JumperState, NetState, PartRegistry, PwlSpec, Scenario, System,
    SystemHandle,
};
use embsim_core::virtual_clock::{self, ClockMode};
use embsim_models::logic_gate::LVC1G14_R_OH_OHMS;
use embsim_models::opto::{Opto, VO2631_LED_VF_VOLTS, VO2631_THRESHOLD_AMPS};
use embsim_models::pwl_library::{
    self, LTST_C190KGKT_I_ON_AMPS, LTST_C190KGKT_VF_VOLTS, MMBT3904_HFE_MIN, MMBT3904_R_SAT_OHMS,
    MMBT3904_VBE_VOLTS, NSI50010_I_REG_AMPS, NSI50010_V_REG_VOLTS,
};
use machine_parts::{bench_rails, edge_board, ep, LOGIC_RAIL_VOLTS};
use rstest::rstest;
use vibes_behaviour::{behaviour, expect, Test};

// ============================================================
// Plumbing
// ============================================================

const EDGE: &str = "EdgeBoard";

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

fn parse(netlist: &str) -> embsim_board::ParsedNetlist {
    embsim_board::netlist::parse(netlist).expect("the fixture parses")
}

/// The voltage a live net reads, if it reads one.
fn volts(system: &SystemHandle, net: &str) -> Option<f64> {
    match system.net_state(net) {
        Some(NetState::Analog(v)) => Some(v),
        _ => None,
    }
}

fn edge_net(net: &str) -> String {
    format!("{EDGE}.{net}")
}

// ============================================================
// An indicator LED and its inverter
// ============================================================

/// `D3` is lit when `U9`'s output is high: the inverter's input is `P16`
/// (a socket finger the bench holds), its output `Y` feeds `R9` (220 Ω)
/// into `D3`'s anode, and `D3`'s cathode is on the bench ground. The
/// current is the rail less the diode's knee over the resistor and the
/// gate's own output resistance — `(3.3 − V_F) / 220` to within the gate's
/// share, which the plan's figure leaves out.
#[rstest]
#[case::input_low_lights_it(0.0, true)]
#[case::input_high_darkens_it(LOGIC_RAIL_VOLTS, false)]
fn an_indicator_led_is_lit_by_its_inverters_high_output(
    #[case] input_volts: f64,
    #[case] lit: bool,
) {
    behaviour!(Test {
        id: "board.led-lit-by-its-inverter",
        covers: Some("board/src/system.rs#BuiltSystem::branch_current"),
        given: "the MaD EdgeBoard under its bench rails, with the input of the Schmitt \
                inverter that drives indicator LED D3 held at 0 volts and then at 3.3 volts",
    });
    expect!(
        "lit-at-the-resistors-current",
        "with the input low the inverter output is high and the LED carries the rail less its \
         forward voltage over the 220 ohm series resistor and the gate's output resistance, \
         within one percent, and is lit",
        "an inverter's high output sources the chain through its own resistance and the LED \
         drops its knee"
    );
    expect!(
        "anode-at-the-knee",
        "with the input low the anode sits at the LED's forward voltage",
    );
    expect!(
        "dark-when-the-output-is-low",
        "with the input high the inverter output is low, the anode sits within a millivolt of \
         ground, the LED carries under a microamp and is dark",
        "a low output sinks the resistor's far end to ground and the diode is below its knee"
    );
    let _guard = stepped();
    let system = System::new()
        .board(EDGE, edge_board())
        .harness(bench_rails(EDGE))
        .scenario(Scenario::default().net_stuck(&edge_net("P16"), input_volts))
        .start()
        .expect("the board starts");
    let d3 = edge_net("D3");
    let anode = edge_net("Net-(D3-A)");
    let output = edge_net("Net-(R9-Pad1)");
    // Settled: the inverter has driven its output (a released output
    // leaves the chain floating with a current of nothing) and the LED's
    // current is the case's.
    let settled = || {
        volts(&system, &anode).is_some()
            && system
                .branch_current(&d3)
                .is_some_and(|amps| if lit { amps > 1e-3 } else { amps < 1e-6 })
    };
    assert!(
        wait_for(settled, SETTLE),
        "D3 {:?} anode {:?} Y {:?}",
        system.branch_current(&d3),
        system.net_state(&anode),
        system.net_state(&output)
    );
    let current = system.branch_current(&d3).unwrap();
    assert_eq!(
        pwl_library::is_lit(Some(current), LTST_C190KGKT_I_ON_AMPS),
        lit,
        "{current}"
    );
    if lit {
        let expected = (LOGIC_RAIL_VOLTS - LTST_C190KGKT_VF_VOLTS) / (220.0 + LVC1G14_R_OH_OHMS);
        assert!(
            (current - expected).abs() < expected * 0.01,
            "{current} vs {expected}"
        );
        // The plan's figure, `(3.3 − V_F) / 220`, to within the gate's share.
        let approximate = (LOGIC_RAIL_VOLTS - LTST_C190KGKT_VF_VOLTS) / 220.0;
        assert!((current - approximate).abs() < approximate * 0.15);
        let v_anode = volts(&system, &anode).expect("the chain solved");
        assert!((v_anode - LTST_C190KGKT_VF_VOLTS).abs() < 1e-3, "{v_anode}");
        let v_y = volts(&system, &output).expect("the chain solved");
        assert!(
            (v_y - (LOGIC_RAIL_VOLTS - current * LVC1G14_R_OH_OHMS)).abs() < 1e-3,
            "{v_y}"
        );
    } else {
        let v_anode = volts(&system, &anode).expect("the chain solved");
        assert!(v_anode.abs() < 1e-3, "{v_anode}");
        assert!(current.abs() < 1e-6, "{current}");
    }
}

// ============================================================
// The polarity FET
// ============================================================

/// `U3` passes the screw-terminal input to `V_IN` forward and blocks it
/// reversed, with nothing in the scenario about it: the 12 V strap on
/// `J2.1` is the FET's drain, its gate is on the bench ground, and the
/// channel turns on once the body diode has lifted the source; with the
/// strap 12 V below ground the gate is above the source, the body diode
/// is reverse-biased, and `V_IN` is a rail nothing reaches.
#[rstest]
#[case::forward(12.0)]
#[case::reversed(-12.0)]
fn the_polarity_fet_passes_the_input_forward_and_blocks_it_reversed(#[case] input_volts: f64) {
    behaviour!(Test {
        id: "board.polarity-fet-passes-and-blocks",
        covers: Some("board/src/engine.rs#Resolver::build_topology"),
        given: "the MaD EdgeBoard under its bench rails, its screw-terminal input first 12 volts \
                above the bench ground and then 12 volts below it, with no scenario line about \
                its reverse-polarity FET",
    });
    expect!(
        "forward-passes",
        "with the input above ground the board's input rail behind the FET reads the input \
         within a millivolt and the input regulators' supply pins are sourced",
        "the FET's gate is on ground, 12 volts below the source its body diode lifts, so the \
         channel conducts and nothing loads it"
    );
    expect!(
        "reversed-blocks",
        "with the input below ground the input rail floats, the FET carries under a microamp, \
         and the regulators' supply pins are reported unsourced",
        "the gate is above the source and the body diode is reverse-biased, so only leakage \
         reaches the rail"
    );
    let _guard = stepped();
    let rails = Harness::new()
        .power(ep("BENCH.12V"), ep(&format!("{EDGE}.J2.1")), input_volts)
        .power(ep("BENCH.GND"), ep(&format!("{EDGE}.J2.2")), 0.0)
        .power(
            ep("BENCH.3V3"),
            ep(&format!("{EDGE}.J19.1")),
            LOGIC_RAIL_VOLTS,
        )
        .power(ep("BENCH.5V"), ep(&format!("{EDGE}.J22.1")), 5.0)
        .power(ep("BENCH.SERVO5V"), ep(&format!("{EDGE}.J21.1")), 5.0)
        .power(ep("BENCH.SERVOGND"), ep(&format!("{EDGE}.J21.8")), 0.0);
    let system = System::new()
        .board(EDGE, edge_board())
        .harness(rails)
        .start()
        .expect("the board starts");
    let v_in = edge_net("V_IN");
    let source_pin = format!("{EDGE}.U3.1");
    let unsourced = || {
        system
            .findings()
            .iter()
            .any(|f| matches!(f, Finding::PowerNetUnsourced { net } if *net == v_in))
    };
    if input_volts > 0.0 {
        assert!(
            wait_for(
                || volts(&system, &v_in).is_some_and(|v| (v - input_volts).abs() < 1e-3),
                SETTLE
            ),
            "{:?}",
            system.net_state(&v_in)
        );
        let through = system
            .pin_current(&source_pin)
            .expect("the FET's cluster solved");
        assert!(
            through.abs() < 1e-6,
            "no load draws through the channel: {through}"
        );
        assert!(!unsourced(), "{:?}", system.findings());
    } else {
        assert!(
            wait_for(
                || system.net_state(&v_in) == Some(NetState::Floating),
                SETTLE
            ),
            "{:?}",
            system.net_state(&v_in)
        );
        // The FET's cluster still solves — the reversed input is a
        // terminal that sources it through the body diode's leakage — so
        // the reading is a zero, not the absence of one.
        let through = system
            .pin_current(&source_pin)
            .expect("the FET's cluster solves from the reversed input");
        assert!(through.abs() < 1e-6, "{through}");
        assert!(unsourced(), "{:?}", system.findings());
    }
}

// ============================================================
// The end-switch loop
// ============================================================

/// The upper end-switch loop with a bench supply on it and its contact
/// closed by a bench return: `IC9` regulates the loop at its 10 mA
/// whatever the 24 V bench rail would push through the LED alone, the
/// opto `U6` lights and sinks `P19` against its 1 kΩ pull-up; open, the
/// loop carries nothing and `P19` sits at the pull-up.
#[rstest]
#[case::closed(true)]
#[case::open(false)]
fn the_end_switch_loop_is_regulated_at_the_ccr_current(#[case] closed: bool) {
    behaviour!(Test {
        id: "board.end-switch-loop-regulated",
        covers: Some("board/src/cluster.rs#QuasiStaticMna::solve"),
        given: "the MaD EdgeBoard under its bench rails with the P2's I/O-bank rail strapped, \
                24 volts on the upper end-switch loop's supply terminal, and the loop's return \
                terminal first held at 0 volts (a closed contact) and then left open",
    });
    expect!(
        "regulated-at-10-milliamps",
        "with the contact closed the loop carries the current regulator's 10 milliamps within \
         one percent, through the regulator and the optocoupler's LED alike",
        "the regulator sees 22 volts of overhead, far past its knee, and holds the loop at its \
         regulation current whatever the supply would push through the LED alone"
    );
    expect!(
        "opto-sinks-the-input",
        "with the contact closed the optocoupler's LED sits at its forward voltage and its \
         output holds the P2's input pin low",
        "the loop current is above the optocoupler's input threshold and its detector is \
         powered"
    );
    expect!(
        "open-loop-dark",
        "with the contact open the loop carries nothing and the P2's input pin sits at its \
         pull-up",
    );
    let _guard = stepped();
    let mut rails = bench_rails(EDGE)
        // The P2's I/O-bank rail, which pulls P19 up through R6, arrives
        // over the module socket.
        .power(
            ep("BENCH.VIO"),
            ep(&format!("{EDGE}.J3.58")),
            LOGIC_RAIL_VOLTS,
        )
        // The loop's supply: 24 V onto IEND_U+, the regulator's anode.
        .power(ep("BENCH.ENDLOOP"), ep(&format!("{EDGE}.J16.2")), 24.0);
    if closed {
        // A closed contact: the loop's return at the bench ground.
        rails = rails.power(ep("BENCH.ENDRETURN"), ep(&format!("{EDGE}.J16.1")), 0.0);
    }
    let system = System::new()
        .board(EDGE, edge_board())
        .harness(rails)
        .start()
        .expect("the board starts");
    let regulator = edge_net("IC9");
    let led_anode_pin = format!("{EDGE}.U6.4");
    let p19 = edge_net("P19");
    if closed {
        assert!(
            wait_for(
                || system.net_state(&p19) == Some(NetState::Driven(embsim_board::Level::Low)),
                SETTLE
            ),
            "{:?}",
            system.net_state(&p19)
        );
        let through_regulator = system.branch_current(&regulator).expect("the loop solved");
        assert!(
            (through_regulator - NSI50010_I_REG_AMPS).abs() < NSI50010_I_REG_AMPS * 0.01,
            "{through_regulator}"
        );
        let into_led = system.pin_current(&led_anode_pin).expect("the loop solved");
        assert!((into_led - through_regulator).abs() < 1e-9, "{into_led}");
        // The LED sits at its knee, and the regulator has the rest of the
        // 24 V as overhead — well past the knee it regulates from.
        let cathode_side = volts(&system, &edge_net("Net-(IC9-K)")).expect("solved");
        assert!(
            (cathode_side - VO2631_LED_VF_VOLTS).abs() < 1e-3,
            "{cathode_side}"
        );
        assert!(24.0 - cathode_side > NSI50010_V_REG_VOLTS);
        assert!(through_regulator >= VO2631_THRESHOLD_AMPS);
    } else {
        assert!(
            wait_for(
                || matches!(
                    system.net_state(&p19),
                    Some(NetState::Pulled(embsim_board::Level::High, _))
                ),
                SETTLE
            ),
            "{:?}",
            system.net_state(&p19)
        );
        // The loop's cluster solves — the 24 V supply on its anode sources
        // it — and reads nothing through the regulator: a zero, not the
        // absence of a reading.
        let through_regulator = system
            .branch_current(&regulator)
            .expect("the loop's cluster solves from the 24 V supply on its anode");
        assert!(through_regulator.abs() < 1e-9, "{through_regulator}");
    }
}

/// The optocouplers' input loops under the bench rails, every contact
/// open — the bench's normal state: each LED's anode net is read by the
/// opto's current instrument, and an open loop is not a floating input.
/// Nothing is reported for the instruments, at build or live.
#[rstest]
fn an_open_opto_loop_is_not_a_floating_input() {
    behaviour!(Test {
        id: "elements.open-opto-loop-reports-nothing",
        covers: Some("board/src/engine.rs#Resolver::resolve_cluster"),
        given: "the EdgeBoard under its bench rails with every end-switch and enable contact \
                open, so no optocoupler's LED loop carries a current",
    });
    expect!(
        "no-floating-instrument",
        "no LED anode net is reported as a floating sense, at build or once live",
        "a current instrument on a pin escalates its cluster to a solve and reports nothing \
         else: an open contact loop is the bench's normal state, and the floating input the \
         finding names is a pin that reads a level nothing drives"
    );
    let _guard = stepped();
    // The LED anode nets: the eight end-switch and enable loops' regulator
    // cathodes (`IC6`–`IC13` feed the VO2631s' LEDs) and the charge-pump
    // 6N137's anode.
    let anodes: Vec<String> = [
        "Net-(IC6-K)",
        "Net-(IC7-K)",
        "Net-(IC8-K)",
        "Net-(IC9-K)",
        "Net-(IC10-K)",
        "Net-(IC11-K)",
        "Net-(IC12-K)",
        "Net-(IC13-K)",
        "Net-(U4-A)",
    ]
    .iter()
    .map(|net| edge_net(net))
    .collect();
    // The RS-422 receiver's differential inputs float on this bench with
    // nothing on `J21` — real analog senses, reported as they should be —
    // so the assertion is over the instruments' nets, not every kind.
    let floating_senses = |findings: &[Finding]| -> Vec<Finding> {
        findings
            .iter()
            .filter(|f| matches!(f, Finding::FloatingSense { net, .. } if anodes.contains(net)))
            .cloned()
            .collect()
    };
    let built = System::new()
        .board(EDGE, edge_board())
        .harness(bench_rails(EDGE))
        .build()
        .expect("the board builds");
    for anode in &anodes {
        assert!(
            built.net_id(anode).is_some(),
            "{anode} is a net of the board"
        );
    }
    assert_eq!(
        floating_senses(built.diagnostics().findings()),
        Vec::<Finding>::new()
    );
    let system = System::new()
        .board(EDGE, edge_board())
        .harness(bench_rails(EDGE))
        .start()
        .expect("the board starts");
    assert!(wait_for(
        || system.net_state(&edge_net("P19")).is_some(),
        SETTLE
    ));
    assert_eq!(floating_senses(&system.findings()), Vec::<Finding>::new());
}

// ============================================================
// An optocoupler's threshold
// ============================================================

/// One VO2631 on a bench: its second channel's LED fed from a 5 V terminal
/// through a series resistor to a 0 V return, its detector powered, its
/// output pulled up through 1 kΩ to 3.3 V. The resistor puts the LED
/// current a hair under the input threshold, then a hair over it.
const OPTO_NETLIST: &str = r#"(export (version "E")
  (components
    (comp (ref "U1") (value "VO2631") (libsource (lib "Isolator") (part "VO2631")))
    (comp (ref "R1") (value "SERIES") (libsource (lib "Device") (part "R")))
    (comp (ref "R2") (value "1k") (libsource (lib "Device") (part "R"))))
  (nets
    (net (code "1") (name "S") (node (ref "R1") (pin "1")))
    (net (code "2") (name "A") (node (ref "R1") (pin "2")) (node (ref "U1") (pin "4")))
    (net (code "3") (name "K") (node (ref "U1") (pin "3")))
    (net (code "4") (name "VCC") (node (ref "U1") (pin "8")))
    (net (code "5") (name "GND") (node (ref "U1") (pin "5")))
    (net (code "6") (name "PU") (node (ref "R2") (pin "1")))
    (net (code "7") (name "OUT") (node (ref "R2") (pin "2")) (node (ref "U1") (pin "6")))
    (net (code "8") (name "A1") (node (ref "U1") (pin "1")))
    (net (code "9") (name "C1") (node (ref "U1") (pin "2")))
    (net (code "10") (name "VO1") (node (ref "U1") (pin "7")))))"#;

fn opto_board(series_ohms: u32) -> Board {
    let mut registry = PartRegistry::new();
    registry.register("VO2631", |_decl| Box::new(Opto::vo2631()));
    Board::from_netlist(
        parse(&OPTO_NETLIST.replace("SERIES", &series_ohms.to_string())),
        &registry,
    )
    .expect("the bench builds")
}

/// The output sinks only once the LED carries the input threshold: a
/// series resistor that leaves the LED 2 % short of it leaves the output
/// at its pull-up; one that puts the LED 3 % past it pulls the output
/// down.
#[rstest]
#[case::under_the_threshold(740, false)]
#[case::over_the_threshold(700, true)]
fn an_optocoupler_output_sinks_only_above_its_input_threshold(
    #[case] series_ohms: u32,
    #[case] sinks: bool,
) {
    behaviour!(Test {
        id: "board.opto-sinks-above-threshold",
        covers: Some("models/src/opto.rs#Opto::attach"),
        given: "a dual optocoupler on a bench, its detector powered from 5 volts, one channel's \
                LED fed from 5 volts through a series resistor that puts its current two percent \
                under the datasheet input threshold and then three percent over it, its output \
                pulled up through 1 kilohm to 3.3 volts",
    });
    expect!(
        "led-current-set-by-the-resistor",
        "the LED carries the supply less its forward voltage over the series resistor, within \
         one percent",
    );
    expect!(
        "released-under-threshold",
        "with the LED under the threshold the output sits at its pull-up",
        "the detector switches at the datasheet's guaranteed input threshold and no lower"
    );
    expect!(
        "sinks-over-threshold",
        "with the LED over the threshold the output is driven low",
        "a lit, powered detector sinks its open-collector output"
    );
    let _guard = stepped();
    let system = System::new()
        .board("B", opto_board(series_ohms))
        .scenario(
            Scenario::default()
                .net_stuck("B.S", 5.0)
                .net_stuck("B.K", 0.0)
                .net_stuck("B.VCC", 5.0)
                .net_stuck("B.GND", 0.0)
                .net_stuck("B.PU", LOGIC_RAIL_VOLTS),
        )
        .start()
        .expect("the bench starts");
    let expected = (5.0 - VO2631_LED_VF_VOLTS) / f64::from(series_ohms);
    assert!(
        wait_for(
            || system
                .pin_current("B.U1.4")
                .is_some_and(|amps| (amps - expected).abs() < expected * 0.01),
            SETTLE
        ),
        "{:?} vs {expected}",
        system.pin_current("B.U1.4")
    );
    let want = if sinks {
        NetState::Driven(embsim_board::Level::Low)
    } else {
        NetState::Pulled(embsim_board::Level::High, 1_000.0)
    };
    assert!(
        wait_for(|| system.net_state("B.OUT") == Some(want), SETTLE),
        "{:?}",
        system.net_state("B.OUT")
    );
    assert_eq!(
        system.pin_current("B.U1.4").unwrap() >= VO2631_THRESHOLD_AMPS,
        sinks
    );
}

// ============================================================
// The servo-enable transistor under a load
// ============================================================

/// The servo drive's enable input as a bench sees it: a series resistor
/// and an optocoupler-class LED (the VO2631's 1.38 V) from the drive's own
/// 5 V to the board's `SC_ENA` line — the numbers are the bench's, since
/// no drive is in the netlist.
const DRIVE_INPUT_NETLIST: &str = r#"(export (version "E")
  (components
    (comp (ref "R1") (value "SERIES") (libsource (lib "Device") (part "R")))
    (comp (ref "D1") (value "OPTO_INPUT") (libsource (lib "Bench") (part "OPTO_INPUT")))
    (comp (ref "J1") (value "Conn_01x01") (libsource (lib "Connector") (part "Conn_01x01"))))
  (nets
    (net (code "1") (name "5V") (node (ref "R1") (pin "1")))
    (net (code "2") (name "A") (node (ref "R1") (pin "2")) (node (ref "D1") (pin "2")))
    (net (code "3") (name "ENA") (node (ref "D1") (pin "1")) (node (ref "J1") (pin "1")))))"#;

fn drive_input_board(series_ohms: u32) -> Board {
    let mut registry = PartRegistry::new();
    registry.register_pwl(
        "OPTO_INPUT",
        PwlSpec::diode("2", "1", VO2631_LED_VF_VOLTS, 0.0),
    );
    Board::from_netlist(
        parse(&DRIVE_INPUT_NETLIST.replace("SERIES", &series_ohms.to_string())),
        &registry,
    )
    .expect("the bench builds")
}

/// `Q1` sinks the drive's enable input through `JP1`'s TTL-SINK position:
/// `P6` high reaches its base through `IC14` and `R24` (43 kΩ), the base
/// sits at its knee and carries `(V_OUTC − V_BE) / 43 kΩ` ≈ 100 µA, so the
/// collector can carry at most a hundred times that. A light load (1 kΩ
/// in the drive's input) saturates it — the collector a few tens of
/// millivolts above the isolated ground; a heavy one (220 Ω) asks for
/// more than the base supports and the collector sags to volts, the
/// current exactly the base's hundredfold.
///
/// `IC14` measures its secondary supply against its own `GND2` pins, which
/// the schematic leaves on a net only `C26` shares (`edgeboard.rs`,
/// `the_servo_isolator_secondary_ground_is_unconnected`): as drawn its
/// secondary side is down and `OUTC` drives nothing. The bench wires that
/// net to `EN_GND` — the rework the defect needs, said out loud.
#[rstest]
#[case::light_load_saturates(1_000, true)]
#[case::heavy_load_sags(220, false)]
fn the_servo_enable_transistor_saturates_under_a_light_load_and_sags_under_a_heavy_one(
    #[case] series_ohms: u32,
    #[case] saturates: bool,
) {
    behaviour!(Test {
        id: "board.enable-transistor-under-load",
        covers: Some("board/src/cluster.rs#QuasiStaticMna::solve"),
        given: "the MaD EdgeBoard under its bench rails with the servo-enable jumper on its \
                transistor position, the P2's enable finger held high, and a bench servo drive \
                whose enable input is an optocoupler LED from its own 5 volts through 1 kilohm \
                and then through 220 ohms onto the board's enable line, the servo isolator's \
                secondary ground wired to the servo ground",
    });
    expect!(
        "base-at-the-knee",
        "the transistor's base sits between 0.65 and 0.75 volts and carries the isolator's \
         output less the base voltage over the 43 kilohm base resistor, within one percent",
        "a driven base-emitter junction drops its knee and the base resistor sets the current"
    );
    expect!(
        "light-load-saturates",
        "with the 1 kilohm input the collector carries the drive's 5 volts less the LED's \
         forward voltage over the resistor and the saturated 20 ohms, and the enable line sits \
         that current times 20 ohms above the isolated ground",
        "the base supports a hundred times its own current, more than the input asks for"
    );
    expect!(
        "heavy-load-sags",
        "with the 220 ohm input the collector carries exactly a hundred times the base current \
         and the enable line sits volts above the isolated ground, where the drive's load line \
         puts it",
        "the input asks for more than the base supports, so the transistor is active and the \
         collector sags to where the load line puts it"
    );
    let _guard = stepped();
    let system = System::new()
        .board(EDGE, edge_board())
        .board("DRIVE", drive_input_board(series_ohms))
        .harness(bench_rails(EDGE))
        .harness(
            Harness::new()
                .connect(ep("DRIVE.J1.1"), ep(&format!("{EDGE}.J21.7")))
                // The rework: `IC14`'s orphaned `GND2` net onto `EN_GND`.
                .connect(ep(&format!("{EDGE}.IC14.9")), ep(&format!("{EDGE}.J21.8"))),
        )
        .scenario(
            Scenario::default()
                // JP1 pole 1 (pads 2–3): Q1's collector on SC_ENA.
                .switch(&edge_net("JP1"), 1, JumperState::Closed)
                .net_stuck(&edge_net("P6"), LOGIC_RAIL_VOLTS)
                .net_stuck("DRIVE.5V", 5.0),
        )
        .start()
        .expect("the board starts");
    let base = edge_net("Net-(Q1-B)");
    let outc = edge_net("Net-(IC14-OUTC)");
    let ena = edge_net("/MaD_Edge_Sheet3/SC_ENA");
    let base_pin = format!("{EDGE}.Q1.2");
    let collector_pin = format!("{EDGE}.Q1.3");
    assert!(
        wait_for(
            || volts(&system, &base).is_some_and(|v| (MMBT3904_VBE_VOLTS..=0.75).contains(&v))
                && system.pin_current(&collector_pin).is_some_and(|i| i > 1e-3),
            SETTLE
        ),
        "base {:?} collector {:?}",
        system.net_state(&base),
        system.pin_current(&collector_pin)
    );
    let v_base = volts(&system, &base).unwrap();
    let v_outc = volts(&system, &outc).expect("the base network solved");
    let i_b = system.pin_current(&base_pin).unwrap();
    let expected_i_b = (v_outc - v_base) / 43_000.0;
    assert!(
        (i_b - expected_i_b).abs() < expected_i_b * 0.01,
        "{i_b} vs {expected_i_b}"
    );
    let i_c = system.pin_current(&collector_pin).unwrap();
    let v_ena = volts(&system, &ena).expect("the enable line solved");
    let supports = MMBT3904_HFE_MIN * i_b;
    if saturates {
        let expected_i_c =
            (5.0 - VO2631_LED_VF_VOLTS) / (f64::from(series_ohms) + MMBT3904_R_SAT_OHMS);
        assert!(
            (i_c - expected_i_c).abs() < expected_i_c * 0.01,
            "{i_c} vs {expected_i_c}"
        );
        assert!(i_c < supports, "{i_c} within the base's {supports}");
        assert!((v_ena - i_c * MMBT3904_R_SAT_OHMS).abs() < 1e-3, "{v_ena}");
    } else {
        assert!(
            (i_c - supports).abs() < supports * 0.001,
            "{i_c} vs {supports}"
        );
        let expected_v = 5.0 - VO2631_LED_VF_VOLTS - i_c * f64::from(series_ohms);
        assert!((v_ena - expected_v).abs() < 1e-3, "{v_ena} vs {expected_v}");
        assert!(v_ena > 1.0, "a sagging collector, not a switch: {v_ena}");
    }
}
