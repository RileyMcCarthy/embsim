//! The TI AM26LV32 line receiver (`embsim_models::am26lv32`) as the
//! standard catalog places it, alone on a bench: its output lines, its
//! function table with the fail-safe, and the supply range its `VCC` pin
//! declares, which the engine checks and reports above, each against TI
//! SLLS202H.
//!
//! The part is placed by the base registry's number (`AM26LV32IDR`, the
//! MaD Edge board's `U25`), so what is tested is what a project gets. Its
//! supply comes from the bench, its ground, inputs and enables are held by
//! the scenario (or left open), and channels 1 and 2 may carry the two
//! loads the datasheet's output lines are tested with: 5 mA out of `1Y`
//! into a resistor to ground, and 5 mA into `2Y` from a resistor to the
//! supply.
//!
//! Stepped (`TESTING.md` rule 9): a suite lock, the clock re-anchored
//! stepped, the system started with time held, the case's thread a
//! registered actor, one virtual settle, no `QuiescenceTimeout` at the end.

use std::sync::{Mutex, MutexGuard};

use embsim_board::{
    netlist, Board, EndpointRef, Finding, Harness, Level, NetState, Scenario, System, Volts,
};
use embsim_boards::catalog::StandardCatalog;
use embsim_core::virtual_clock::{self, ClockMode};
use embsim_models::am26lv32::AM26LV32_SUPPLY_NOTE;
use rstest::rstest;
use vibes_behaviour::{behaviour, expect, Test};

static SUITE_LOCK: Mutex<()> = Mutex::new(());

fn suite_lock() -> MutexGuard<'static, ()> {
    SUITE_LOCK.lock().unwrap_or_else(|poisoned| {
        SUITE_LOCK.clear_poison();
        poisoned.into_inner()
    })
}

/// The virtual time a case is handed before it reads. Nothing on the bench
/// arms an instant (the model drives in its sense callbacks, and its
/// propagation delay is not modelled), so any window past the attach
/// cascade's instant is at rest.
const SETTLE_NS: u64 = 10_000;

/// `1Y`'s load: `V_OH` min over `I_OH`, so the line's test condition is
/// exactly the load's (SLLS202H §6.5): 480 Ω to ground.
const Y1_LOAD_OHMS: f64 = 2.4 / 0.005;

/// `2Y`'s load: from the supply at `V_CC` min, `I_OL` into `V_OL` max
/// (SLLS202H §6.3, §6.5): 500 Ω to the supply.
const Y2_LOAD_OHMS: f64 = (3.0 - 0.5) / 0.005;

/// The datasheet's numbers, written out rather than taken from the model,
/// so a model that drifts from them fails here (SLLS202H §6.3, §6.5).
const VCC_MIN: Volts = 3.0;
const VCC_MAX: Volts = 3.6;
const VOH_MIN: Volts = 2.4;
const VOL_MAX: Volts = 0.5;
/// `(V_CC min − V_OH min) / I_OH` and `V_OL max / I_OL`.
const R_OH: f64 = 120.0;
const R_OL: f64 = 100.0;

/// The receiver, every pin on a net of its own, and the two loads when
/// `loaded`.
fn netlist_text(loaded: bool) -> String {
    let mut comps = String::from(
        "    (comp (ref \"U1\") (value \"AM26LV32IDR\") (libsource (lib \"Interface\") (part \"AM26LV32IDR\")))\n",
    );
    let extra = |net: &str| -> String {
        if !loaded {
            return String::new();
        }
        match net {
            "1Y" => " (node (ref \"R1\") (pin \"1\"))".to_string(),
            "GND" => " (node (ref \"R1\") (pin \"2\"))".to_string(),
            "2Y" => " (node (ref \"R2\") (pin \"1\"))".to_string(),
            "VCC" => " (node (ref \"R2\") (pin \"2\"))".to_string(),
            _ => String::new(),
        }
    };
    let pins = [
        ("1", "1B"),
        ("2", "1A"),
        ("3", "1Y"),
        ("4", "G"),
        ("5", "2Y"),
        ("6", "2A"),
        ("7", "2B"),
        ("8", "GND"),
        ("9", "3B"),
        ("10", "3A"),
        ("11", "3Y"),
        ("12", "NG"),
        ("13", "4Y"),
        ("14", "4A"),
        ("15", "4B"),
        ("16", "VCC"),
    ];
    let nets: String = pins
        .iter()
        .enumerate()
        .map(|(code, (pin, net))| {
            format!(
                "    (net (code \"{}\") (name \"{net}\") (node (ref \"U1\") (pin \"{pin}\")){})\n",
                code + 1,
                extra(net)
            )
        })
        .collect();
    if loaded {
        comps.push_str(&format!(
            "    (comp (ref \"R1\") (value \"{Y1_LOAD_OHMS}\") (libsource (lib \"Device\") (part \"R\")))\n\
             \x20   (comp (ref \"R2\") (value \"{Y2_LOAD_OHMS}\") (libsource (lib \"Device\") (part \"R\")))\n"
        ));
    }
    format!("(export (version \"E\")\n  (components\n{comps})\n  (nets\n{nets}))\n")
}

fn ep(endpoint: &str) -> EndpointRef {
    EndpointRef::parse(endpoint).expect("endpoint parses")
}

fn system(loaded: bool, vcc: Volts, held: &[(&str, Volts)]) -> System {
    let parsed = netlist::parse(&netlist_text(loaded)).expect("the bench netlist parses");
    let board = Board::from_netlist(parsed, &StandardCatalog::base_registry())
        .expect("the catalog places the receiver by its number");
    let mut scenario = Scenario::default().net_stuck("B.GND", 0.0);
    for (net, volts) in held {
        scenario = scenario.net_stuck(&format!("B.{net}"), *volts);
    }
    System::new()
        .board("B", board)
        .harness(Harness::new().power(ep("BENCH.VCC"), ep("B.U1.16"), vcc))
        .scenario(scenario)
}

/// Run the bench at `vcc` with the scenario's held nets (`B.<net>`, volts)
/// and return each named net's state at rest, with the findings the run
/// made.
fn run(
    loaded: bool,
    vcc: Volts,
    held: &[(&str, Volts)],
    read: &[&str],
) -> (Vec<NetState>, Vec<Finding>) {
    let _lock = suite_lock();
    virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);
    let system = system(loaded, vcc, held)
        .hold_time()
        .start()
        .expect("the bench starts");
    let actor = virtual_clock::register_actor("am26lv32-receiver-case");
    system.release_time();
    virtual_clock::wait_virtual_ns(SETTLE_NS);
    let states = read
        .iter()
        .map(|net| {
            system
                .net_state(&format!("B.{net}"))
                .unwrap_or_else(|| panic!("B.{net} is a net"))
        })
        .collect();
    let findings = system.findings();
    let stalled: Vec<&Finding> = findings
        .iter()
        .filter(|finding| matches!(finding, Finding::QuiescenceTimeout { .. }))
        .collect();
    assert!(
        stalled.is_empty(),
        "the engine stopped waiting: {stalled:?}"
    );
    drop(actor);
    system.shutdown();
    (states, findings)
}

fn volts(state: NetState, net: &str) -> Volts {
    match state {
        NetState::Analog(volts) => volts,
        other => panic!("{net}: expected a solved voltage, got {other:?}"),
    }
}

/// Enabled by `G`, channel 1's pair at +1 V and channel 2's at −1 V.
const ENABLED_ONE_HIGH_TWO_LOW: [(&str, Volts); 6] = [
    ("G", 3.3),
    ("NG", 3.3),
    ("1A", 2.0),
    ("1B", 1.0),
    ("2A", 1.0),
    ("2B", 2.0),
];

#[rstest]
#[case::at_vcc_min(VCC_MIN)]
#[case::at_vcc_nominal(3.3)]
#[case::at_vcc_max(VCC_MAX)]
fn the_am26lv32_drives_its_output_lines_from_its_own_supply(#[case] vcc: Volts) {
    behaviour!(Test {
        id: "am26lv32.output-lines",
        covers: Some("models/src/am26lv32.rs#Am26lv32"),
        given: "an enabled AM26LV32 from the catalog at 3, 3.3 or 3.6 volts, channel 1 high \
                into 480 ohms to ground, channel 2 low under 500 ohms from the supply",
    });
    expect!(
        "voh-line",
        "at a 3 volt supply the high output sources 5 milliamps at 2.4 volts",
        "SLLS202H guarantees a high output of at least 2.4 volts at 5 milliamps over its \
         supply range, which from its 3 volt minimum sets the high-side impedance at 120 ohms"
    );
    expect!(
        "vol-line",
        "at a 3 volt supply the low output sinks 5 milliamps at 0.5 volts",
        "SLLS202H guarantees a low output of at most 0.5 volts at 5 milliamps, which sets the \
         low-side impedance at 100 ohms"
    );
    expect!(
        "from-own-supply",
        "at every supply each output reads its own supply pin's voltage divided between its \
         load and the output's 120 or 100 ohms",
        "the output's high level is the part's own supply, so a sagging rail sags the line"
    );
    let (states, _) = run(true, vcc, &ENABLED_ONE_HIGH_TWO_LOW, &["1Y", "2Y"]);
    let y1 = volts(states[0], "1Y");
    let y2 = volts(states[1], "2Y");
    let y1_expected = vcc * Y1_LOAD_OHMS / (Y1_LOAD_OHMS + R_OH);
    let y2_expected = vcc * R_OL / (Y2_LOAD_OHMS + R_OL);
    assert!(
        (y1 - y1_expected).abs() < 1e-9,
        "1Y {y1} V, not {y1_expected}"
    );
    assert!(
        (y2 - y2_expected).abs() < 1e-9,
        "2Y {y2} V, not {y2_expected}"
    );
    if vcc == VCC_MIN {
        assert!((y1 - VOH_MIN).abs() < 1e-9, "1Y {y1} V");
        assert!((y2 - VOL_MAX).abs() < 1e-9, "2Y {y2} V");
    }
}

/// What one output presents.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Out {
    Drives(Level),
    Released,
}

const H: Out = Out::Drives(Level::High);
const L: Out = Out::Drives(Level::Low);
const RELEASED: Out = Out::Released;

/// Each channel's pair: `Some((A, B))` held, or `None` left open.
type Pairs = [Option<(Volts, Volts)>; 4];

/// The function table's three pair rows: +1 V, −1 V, and a pair whose
/// difference sits exactly on `V_IT−` (0 V against 0.2 V).
const ROWS: Pairs = [
    Some((2.0, 1.0)),
    Some((1.0, 2.0)),
    Some((0.0, 0.2)),
    Some((2.0, 1.0)),
];

#[rstest]
#[case::enabled_by_g(3.3, 3.3, 3.3, ROWS, [H, L, L, H])]
#[case::enabled_by_not_g(3.3, 0.0, 0.0, ROWS, [H, L, L, H])]
#[case::disabled(3.3, 0.0, 3.3, ROWS, [RELEASED; 4])]
#[case::in_band_enable_enables_nothing(3.3, 1.4, 3.3, ROWS, [RELEASED; 4])]
#[case::below_vcc_min(2.9, 3.3, 0.0, ROWS, [RELEASED; 4])]
#[case::in_band_pair(3.3, 3.3, 3.3, [Some((1.1, 1.0)), Some((1.0, 1.1)), Some((1.0, 2.0)), Some((2.0, 1.0))], [H, H, L, H])]
#[case::shorted_pair(3.3, 3.3, 3.3, [Some((0.0, 0.0)), Some((1.0, 2.0)), Some((1.5, 1.5)), Some((2.0, 1.0))], [H, L, H, H])]
#[case::open_pairs(3.3, 3.3, 3.3, [None, Some((1.0, 2.0)), None, None], [H, L, H, H])]
fn the_am26lv32_follows_its_function_table_and_fails_safe_high(
    #[case] vcc: Volts,
    #[case] g: Volts,
    #[case] not_g: Volts,
    #[case] pairs: Pairs,
    #[case] outs: [Out; 4],
) {
    behaviour!(Test {
        id: "am26lv32.function-table",
        covers: Some("models/src/am26lv32.rs#Am26lv32"),
        given: "an AM26LV32 from the catalog, each enable held high, low or at 1.4 volts, and \
                each input pair held at a differential voltage, shorted, or left open",
    });
    expect!(
        "follows-the-difference",
        "enabled by G high or not-G low, a pair 1 volt positive reads high and one at or \
         below minus 0.2 volts reads low",
        "SLLS202H Table 8-1 and its 0.2 volt differential thresholds"
    );
    expect!(
        "released-when-off",
        "with both enables off, with one at 1.4 volts and the other off, or under a 3 volt \
         supply, all four outputs are released",
        "the function table's high-impedance row, an enable with no level, and a part under \
         its recommended supply"
    );
    expect!(
        "fails-safe-high",
        "an open pair, a shorted pair and a pair 0.1 volts apart each read high",
        "SLLS202H's fail-safe: an open input rests at its own bias, 130 millivolts across the \
         pair, and a shorted or idle pair puts the output high"
    );
    let mut held = vec![("G", g), ("NG", not_g)];
    let names = [("1A", "1B"), ("2A", "2B"), ("3A", "3B"), ("4A", "4B")];
    for ((a, b), pair) in names.iter().zip(pairs) {
        if let Some((va, vb)) = pair {
            held.push((a, va));
            held.push((b, vb));
        }
    }
    let read = ["1Y", "2Y", "3Y", "4Y"];
    let (states, _) = run(false, vcc, &held, &read);
    for ((net, state), out) in read.iter().zip(&states).zip(outs) {
        match out {
            Out::Drives(level) => assert_eq!(*state, NetState::Driven(level), "{net}"),
            Out::Released => assert_eq!(*state, NetState::Floating, "{net}"),
        }
    }
}

fn supply_findings(findings: &[Finding]) -> Vec<&Finding> {
    findings
        .iter()
        .filter(|finding| matches!(finding, Finding::PinAboveRecommended { .. }))
        .collect()
}

#[rstest]
#[case::at_vcc_nominal(3.3, false)]
#[case::at_vcc_max(VCC_MAX, false)]
#[case::from_the_edge_boards_5_volts(5.0, true)]
fn the_am26lv32s_declared_supply_range_is_checked_by_the_engine(
    #[case] vcc: Volts,
    #[case] over: bool,
) {
    behaviour!(Test {
        id: "am26lv32.supply-range",
        covers: Some("models/src/am26lv32.rs#AM26LV32_VCC_LIMITS"),
        given: "an enabled AM26LV32 from the catalog, channel 1 reading high, its supply at 3.3 \
                volts, at its 3.6 volt recommended maximum, or at 5 volts",
    });
    expect!(
        "over-range-finding",
        "at 5 volts, the build and the live run each report once that the supply pin is above \
         the 3.6 volts recommended",
        "the supply pin declares SLLS202H's 3 to 3.6 volts recommended and 6 volts absolute, \
         and the engine checks the solved supply against them; the part runs there, but its \
         open-input bias is not characterised"
    );
    expect!(
        "in-range-silent",
        "at 3.3 volts and at exactly 3.6 volts, nothing is reported about the supply"
    );
    expect!(
        "runs-over-range",
        "at every supply the part still drives channel 1 high",
        "an over-range supply is a finding, and the part keeps to its function table"
    );
    let held = [("G", 3.3), ("NG", 3.3), ("1A", 2.0), ("1B", 1.0)];
    let expected = Finding::PinAboveRecommended {
        part: "B.U1".to_string(),
        pin: "16".to_string(),
        volts: vcc,
        min: VCC_MIN,
        max: VCC_MAX,
        absolute_max: Some(6.0),
        note: AM26LV32_SUPPLY_NOTE.to_string(),
    };
    let built = {
        let _lock = suite_lock();
        system(false, vcc, &held).build().expect("the bench builds")
    };
    let (states, live) = run(false, vcc, &held, &["1Y"]);
    assert_eq!(states[0], NetState::Driven(Level::High), "1Y");
    for (path, findings) in [
        ("build", built.diagnostics().findings().to_vec()),
        ("live", live),
    ] {
        let reported = supply_findings(&findings);
        if over {
            assert_eq!(reported, [&expected], "{path}: {findings:?}");
        } else {
            assert!(reported.is_empty(), "{path}: {findings:?}");
        }
    }
}
