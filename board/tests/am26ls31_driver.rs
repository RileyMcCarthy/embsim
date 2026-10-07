//! The TI AM26LS31 line driver (`embsim_models::am26ls31`) as the standard
//! catalog places it, alone on a bench: its output lines, its function
//! table and its power-on threshold, each against TI SLLS114N.
//!
//! The part is placed by the base registry's number (`AM26LS31CD`, the MaD
//! Edge board's `U24`), so what is tested is what a project gets. Its
//! supply comes from the bench, its ground, inputs and enables are held by
//! the scenario, and channel 1 may carry the two loads the datasheet's
//! output lines are tested with: 20 mA out of `1Y` into a resistor to
//! ground, and 20 mA into `1Z` from a resistor to the supply.
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
use embsim_models::am26ls31::{
    AM26LS31_OUTPUT_TEST_AMPS, AM26LS31_R_OH_OHMS, AM26LS31_R_OL_OHMS, AM26LS31_VCC_MAX_VOLTS,
    AM26LS31_VCC_MIN_VOLTS, AM26LS31_VOH_MIN_VOLTS, AM26LS31_VOL_MAX_VOLTS,
    AM26LS31_VPOR_MAX_VOLTS,
};
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
/// exactly the load's (SLLS114N §5.5): 125 Ω to ground.
const Y_LOAD_OHMS: f64 = AM26LS31_VOH_MIN_VOLTS / AM26LS31_OUTPUT_TEST_AMPS;

/// `1Z`'s load: from the supply at `V_CC` MIN, `I_OL` into `V_OL` max
/// (SLLS114N §5.5): 212.5 Ω to the supply.
const Z_LOAD_OHMS: f64 =
    (AM26LS31_VCC_MIN_VOLTS - AM26LS31_VOL_MAX_VOLTS) / AM26LS31_OUTPUT_TEST_AMPS;

/// The driver, every pin on a net of its own, and channel 1's two loads
/// when `loaded`.
fn netlist_text(loaded: bool) -> String {
    let mut comps = String::from(
        "    (comp (ref \"U1\") (value \"AM26LS31CD\") (libsource (lib \"Interface\") (part \"AM26LS31CD\")))\n",
    );
    let extra = |net: &str| -> String {
        if !loaded {
            return String::new();
        }
        match net {
            "1Y" => " (node (ref \"R1\") (pin \"1\"))".to_string(),
            "GND" => " (node (ref \"R1\") (pin \"2\"))".to_string(),
            "1Z" => " (node (ref \"R2\") (pin \"1\"))".to_string(),
            "VCC" => " (node (ref \"R2\") (pin \"2\"))".to_string(),
            _ => String::new(),
        }
    };
    let pins = [
        ("1", "1A"),
        ("2", "1Y"),
        ("3", "1Z"),
        ("4", "G"),
        ("5", "2Z"),
        ("6", "2Y"),
        ("7", "2A"),
        ("8", "GND"),
        ("9", "3A"),
        ("10", "3Y"),
        ("11", "3Z"),
        ("12", "NG"),
        ("13", "4Z"),
        ("14", "4Y"),
        ("15", "4A"),
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
            "    (comp (ref \"R1\") (value \"{Y_LOAD_OHMS}\") (libsource (lib \"Device\") (part \"R\")))\n\
             \x20   (comp (ref \"R2\") (value \"{Z_LOAD_OHMS}\") (libsource (lib \"Device\") (part \"R\")))\n"
        ));
    }
    format!("(export (version \"E\")\n  (components\n{comps})\n  (nets\n{nets}))\n")
}

fn ep(endpoint: &str) -> EndpointRef {
    EndpointRef::parse(endpoint).expect("endpoint parses")
}

/// Run the bench at `vcc` with the scenario's held nets (`B.<net>`, volts)
/// and return each named net's state at rest.
fn run(loaded: bool, vcc: Volts, held: &[(&str, Volts)], read: &[&str]) -> Vec<NetState> {
    let _lock = suite_lock();
    virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);
    let parsed = netlist::parse(&netlist_text(loaded)).expect("the bench netlist parses");
    let board = Board::from_netlist(parsed, &StandardCatalog::base_registry())
        .expect("the catalog places the driver by its number");
    let mut scenario = Scenario::default().net_stuck("B.GND", 0.0);
    for (net, volts) in held {
        scenario = scenario.net_stuck(&format!("B.{net}"), *volts);
    }
    let system = System::new()
        .board("B", board)
        .harness(Harness::new().power(ep("BENCH.VCC"), ep("B.U1.16"), vcc))
        .scenario(scenario)
        .hold_time()
        .start()
        .expect("the bench starts");
    let actor = virtual_clock::register_actor("am26ls31-driver-case");
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
    let stalled: Vec<Finding> = system
        .findings()
        .into_iter()
        .filter(|finding| matches!(finding, Finding::QuiescenceTimeout { .. }))
        .collect();
    assert!(
        stalled.is_empty(),
        "the engine stopped waiting: {stalled:?}"
    );
    drop(actor);
    system.shutdown();
    states
}

fn volts(state: NetState, net: &str) -> Volts {
    match state {
        NetState::Analog(volts) => volts,
        other => panic!("{net}: expected a solved voltage, got {other:?}"),
    }
}

#[rstest]
#[case::at_vcc_min(AM26LS31_VCC_MIN_VOLTS)]
#[case::at_vcc_nominal(5.0)]
#[case::at_vcc_max(AM26LS31_VCC_MAX_VOLTS)]
fn the_am26ls31_drives_its_output_lines_from_its_own_supply(#[case] vcc: Volts) {
    behaviour!(Test {
        id: "am26ls31.output-lines",
        covers: Some("models/src/am26ls31.rs#Am26ls31"),
        given: "an enabled AM26LS31 from the catalog at a 4.75, 5 or 5.25 volt supply, channel \
                1 driven high, its outputs loaded 125 ohms to ground and 212.5 ohms to the \
                supply",
    });
    expect!(
        "voh-line",
        "at a 4.75 volt supply the true output sources 20 milliamps at 2.5 volts",
        "SLLS114N guarantees a high output of at least 2.5 volts at the lowest supply and 20 \
         milliamps, which sets the output's high-side impedance at 112.5 ohms"
    );
    expect!(
        "vol-line",
        "at a 4.75 volt supply the complement sinks 20 milliamps at 0.5 volts",
        "SLLS114N guarantees a low output of at most 0.5 volts at 20 milliamps, which sets the \
         low-side impedance at 25 ohms"
    );
    expect!(
        "from-own-supply",
        "at every supply each output reads its own supply pin's voltage divided between its \
         load and the output's 112.5 or 25 ohms",
        "the output's high level is the part's own supply, so a sagging rail sags the line"
    );
    let states = run(
        true,
        vcc,
        &[("G", vcc), ("NG", 0.0), ("1A", 3.3)],
        &["1Y", "1Z"],
    );
    let y = volts(states[0], "1Y");
    let z = volts(states[1], "1Z");
    let y_expected = vcc * Y_LOAD_OHMS / (Y_LOAD_OHMS + AM26LS31_R_OH_OHMS);
    let z_expected = vcc * AM26LS31_R_OL_OHMS / (Z_LOAD_OHMS + AM26LS31_R_OL_OHMS);
    assert!((y - y_expected).abs() < 1e-9, "1Y {y} V, not {y_expected}");
    assert!((z - z_expected).abs() < 1e-9, "1Z {z} V, not {z_expected}");
    if vcc == AM26LS31_VCC_MIN_VOLTS {
        assert!((y - AM26LS31_VOH_MIN_VOLTS).abs() < 1e-9, "1Y {y} V");
        assert!((z - AM26LS31_VOL_MAX_VOLTS).abs() < 1e-9, "1Z {z} V");
    }
}

/// What the four pairs present: driven from `A` (channels 1 and 3 high,
/// 2 and 4 low), or every output released.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Pairs {
    Driven,
    Released,
}

#[rstest]
#[case::enabled_by_g(5.0, 5.0, 5.0, Pairs::Driven)]
#[case::enabled_by_not_g(5.0, 0.0, 0.0, Pairs::Driven)]
#[case::enabled_by_both(5.0, 5.0, 0.0, Pairs::Driven)]
#[case::disabled(5.0, 0.0, 5.0, Pairs::Released)]
#[case::at_power_on_reset(AM26LS31_VPOR_MAX_VOLTS, 3.0, 0.0, Pairs::Driven)]
#[case::below_power_on_reset(3.0, 3.0, 0.0, Pairs::Released)]
fn the_am26ls31_follows_its_function_table(
    #[case] vcc: Volts,
    #[case] g: Volts,
    #[case] not_g: Volts,
    #[case] pairs: Pairs,
) {
    behaviour!(Test {
        id: "am26ls31.function-table",
        covers: Some("models/src/am26ls31.rs#Am26ls31"),
        given: "an AM26LS31 from the catalog, inputs 1 and 3 high and 2 and 4 low, each \
                enable high or low, its supply at 5 volts, at its 3.04 volt power-on \
                threshold or under it",
    });
    expect!(
        "enabled-by-either",
        "with G high or not-G low, every channel's Y output is its input's level and its Z \
         output the complement, all four channels",
        "SLLS114N Table 7-1: either enable alone enables every driver"
    );
    expect!(
        "high-z-when-both-off",
        "with G low and not-G high, all eight outputs are released",
        "the function table's high-impedance row, the only one"
    );
    expect!(
        "released-below-por",
        "under 3.04 volts of supply all eight outputs are released, and at 3.04 volts they \
         are driven",
        "3.04 volts is SLLS114N's power-on-reset maximum, where every part is out of reset; an \
         unpowered part's outputs are high impedance"
    );
    let channels = [
        ("1", Level::High),
        ("2", Level::Low),
        ("3", Level::High),
        ("4", Level::Low),
    ];
    let mut held = vec![("G", g), ("NG", not_g)];
    let inputs: Vec<(String, Volts)> = channels
        .iter()
        .map(|(n, level)| {
            let volts = if *level == Level::High { 3.0 } else { 0.0 };
            (format!("{n}A"), volts)
        })
        .collect();
    held.extend(inputs.iter().map(|(net, volts)| (net.as_str(), *volts)));
    let outputs: Vec<String> = channels
        .iter()
        .flat_map(|(n, _)| [format!("{n}Y"), format!("{n}Z")])
        .collect();
    let read: Vec<&str> = outputs.iter().map(String::as_str).collect();
    let states = run(false, vcc, &held, &read);
    for (i, (n, level)) in channels.iter().enumerate() {
        let (y, z) = (states[2 * i], states[2 * i + 1]);
        match pairs {
            Pairs::Driven => {
                let complement = match level {
                    Level::High => Level::Low,
                    Level::Low => Level::High,
                };
                assert_eq!(y, NetState::Driven(*level), "{n}Y");
                assert_eq!(z, NetState::Driven(complement), "{n}Z");
            }
            Pairs::Released => {
                assert_eq!(y, NetState::Floating, "{n}Y");
                assert_eq!(z, NetState::Floating, "{n}Z");
            }
        }
    }
}

/// A held voltage inside `A`'s and the enables' 0.8 V to 2 V band
/// (SLLS114N §5.3): neither level is guaranteed, so the model reads none.
const IN_BAND: Volts = 1.4;

/// `V_IL` max and `V_IH` min (SLLS114N §5.3), the band's two edges, as
/// the datasheet prints them rather than read back from the model, so a
/// drifted threshold constant fails here.
const V_IL: Volts = 0.8;
const V_IH: Volts = 2.0;

/// What one channel's pair presents: `Y` at the level and `Z` its
/// complement, or both released.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Pair {
    Drives(Level),
    Released,
}

const H: Pair = Pair::Drives(Level::High);
const L: Pair = Pair::Drives(Level::Low);
const RELEASED: Pair = Pair::Released;

#[rstest]
#[case::in_band_input_on_channel_1(5.0, 0.0, [IN_BAND, 0.0, 3.0, 0.0], [RELEASED, L, H, L])]
#[case::in_band_input_on_channel_4(5.0, 0.0, [3.0, 0.0, 3.0, IN_BAND], [H, L, H, RELEASED])]
#[case::inputs_at_v_il(5.0, 0.0, [V_IL; 4], [L; 4])]
#[case::inputs_at_v_ih(5.0, 0.0, [V_IH; 4], [H; 4])]
#[case::in_band_g_not_g_high(IN_BAND, 5.0, [3.0, 0.0, 3.0, 0.0], [RELEASED; 4])]
#[case::in_band_g_not_g_low(IN_BAND, 0.0, [3.0, 0.0, 3.0, 0.0], [H, L, H, L])]
#[case::in_band_not_g_g_low(0.0, IN_BAND, [3.0, 0.0, 3.0, 0.0], [RELEASED; 4])]
#[case::in_band_not_g_g_high(5.0, IN_BAND, [3.0, 0.0, 3.0, 0.0], [H, L, H, L])]
#[case::both_enables_in_band(IN_BAND, IN_BAND, [3.0, 0.0, 3.0, 0.0], [RELEASED; 4])]
fn the_am26ls31_reads_its_input_and_enable_levels_at_its_thresholds(
    #[case] g: Volts,
    #[case] not_g: Volts,
    #[case] inputs: [Volts; 4],
    #[case] pairs: [Pair; 4],
) {
    behaviour!(Test {
        id: "am26ls31.input-levels",
        covers: Some("models/src/am26ls31.rs#Am26ls31"),
        given: "an AM26LS31 from the catalog at a 5 volt supply, each input and enable held \
                at a steady voltage",
    });
    expect!(
        "in-band-releases-pair",
        "with the part enabled and one input held at 1.4 volts, that channel's outputs are \
         released while the other three channels still drive",
        "SLLS114N guarantees a low input only up to 0.8 volts and a high one only from 2 \
         volts, and Table 7-1 names no output for an input with neither level"
    );
    expect!(
        "threshold-edges",
        "an input at exactly 0.8 volts drives its channel low, and one at exactly 2 volts \
         drives it high",
        "0.8 volts is SLLS114N's low-level input maximum and 2 volts its high-level input \
         minimum, so each edge still reads its level"
    );
    expect!(
        "in-band-enable-enables-nothing",
        "an enable held at 1.4 volts enables nothing: the outputs are released unless the \
         other enable is at G high or not-G low",
        "an enable inside SLLS114N's 0.8 to 2 volt band has neither level, and Table 7-1 \
         enables the drivers only on G high or not-G low"
    );
    let mut held = vec![("G", g), ("NG", not_g)];
    let names = ["1A", "2A", "3A", "4A"];
    held.extend(names.iter().copied().zip(inputs));
    let outputs: Vec<String> = (1..=4)
        .flat_map(|n| [format!("{n}Y"), format!("{n}Z")])
        .collect();
    let read: Vec<&str> = outputs.iter().map(String::as_str).collect();
    let states = run(false, 5.0, &held, &read);
    for (i, pair) in pairs.iter().enumerate() {
        let n = i + 1;
        let (y, z) = (states[2 * i], states[2 * i + 1]);
        match pair {
            Pair::Drives(level) => {
                let complement = match level {
                    Level::High => Level::Low,
                    Level::Low => Level::High,
                };
                assert_eq!(y, NetState::Driven(*level), "{n}Y");
                assert_eq!(z, NetState::Driven(complement), "{n}Z");
            }
            Pair::Released => {
                assert_eq!(y, NetState::Floating, "{n}Y");
                assert_eq!(z, NetState::Floating, "{n}Z");
            }
        }
    }
}
