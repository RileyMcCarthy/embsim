//! The phase-1 findings gate: the July 2026 DS2Addon bench bring-up bugs,
//! reproduced as build-time findings against the board netlist.
//!
//! Each test uses `tests/fixtures/ds2_addon.net`. `R10` (10 kΩ) pulls
//! `~RESET` up to `+3V3`. Lifting that pull-up (`pin_detach` on `R10.2`)
//! is the floating-reset bench bug. An unstrapped AVDD is the other.

use embsim_board::{
    AttachError, Board, Component, ComponentNetIo, Diagnostics, DnpState, EndpointRef, Finding,
    Harness, JumperState, Level, NetState, PartRegistry, PinDecl, Scenario, SenseKind, System,
};
use embsim_models::ads122u04_component::ADS122U04_PINS;
use rstest::rstest;
use vibes_behaviour::{behaviour, expect, Test};

// ============================================================
// ADS122U04 pin facade (TSSOP-16, TI SBAS752B pin table p.3)
// ============================================================

/// Build-time pin facade of the ADS122U04. The pin table is the shared
/// truth exported by the live component (`embsim-models`), so this analysis
/// facade and the live component can never disagree on the pinout.
struct Ads122u04Facade;

impl Component for Ads122u04Facade {
    fn pins(&self) -> &[PinDecl] {
        &ADS122U04_PINS
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        // Typed handle lookup by name and by number must both resolve.
        io.pin("~RESET")?;
        io.pin("16")?;
        Ok(())
    }
}

// ============================================================
// Shared fixtures
// ============================================================

fn registry() -> PartRegistry {
    let mut registry = PartRegistry::new();
    registry.register("ADS122U04", |_decl| Box::new(Ads122u04Facade));
    registry
}

fn ds2_board() -> Board {
    let input = include_str!("fixtures/ds2_addon.net");
    let parsed = embsim_board::netlist::parse(input).expect("fixture parses");
    Board::from_netlist(parsed, &registry()).expect("board builds")
}

fn ep(s: &str) -> EndpointRef {
    EndpointRef::parse(s).expect("endpoint parses")
}

/// Bench harness: digital + analog supply straps as used on the real rig
/// (J1.1 = DVDD, J1.2 = GND, J2.1 = VDDA/AVDD strap, J2.2 = VSS/AGND strap).
fn powered_bench_harness() -> Harness {
    Harness::new()
        .power(ep("BENCH.3V3"), ep("DS2Addon.J1.1"), 3.3)
        .power(ep("BENCH.GND"), ep("DS2Addon.J1.2"), 0.0)
        .power(ep("BENCH.3V3A"), ep("DS2Addon.J2.1"), 3.3)
        .power(ep("BENCH.GNDA"), ep("DS2Addon.J2.2"), 0.0)
}

fn floating(diags: &Diagnostics, net: &str, kind: SenseKind) -> bool {
    diags.contains(&Finding::FloatingSense {
        net: net.to_string(),
        kind,
    })
}

// ============================================================
// Regression 1: ~RESET pulled up by R10
// ============================================================

/// `R10` ties the converter's active-low reset to the digital supply. On a
/// powered bench the reset net reads high, and lifting `R10`'s reset pad
/// leaves the one remaining pin floating.
#[rstest]
fn the_reset_pullup_holds_reset_high() {
    behaviour!(Test {
        id: "ds2.reset-pullup-holds-high",
        covers: Some("board/tests/fixtures/ds2_addon.net"),
        given: "the force-gauge add-on on a powered bench, its reset pull-up fitted",
    });
    expect!(
        "reset-high",
        "the converter's reset input reads high through the pull-up"
    );
    expect!(
        "lifted-pullup-floats",
        "with the pull-up's reset pad lifted, the reset input reads as floating"
    );
    let system = System::new()
        .board("DS2Addon", ds2_board())
        .harness(powered_bench_harness())
        .build()
        .expect("system builds");

    let diags = system.diagnostics();
    assert!(
        !floating(diags, "DS2Addon.~RESET", SenseKind::Digital),
        "R10 must hold ~RESET; got {:?}",
        diags.findings()
    );
    let reset = system
        .nets()
        .iter()
        .find(|net| net.name == "DS2Addon.~RESET")
        .expect("~RESET");
    assert!(
        matches!(reset.state, NetState::Pulled(Level::High, _)),
        "reset should read pulled high, got {:?}",
        reset.state
    );

    assert!(!floating(diags, "DS2Addon.+3V3", SenseKind::Digital));
    assert!(!floating(diags, "DS2Addon.VDDA", SenseKind::Analog));

    let lifted = System::new()
        .board("DS2Addon", ds2_board())
        .harness(powered_bench_harness())
        .scenario(Scenario::default().pin_detach("DS2Addon.R10.2"))
        .build()
        .expect("system builds");
    assert!(
        floating(lifted.diagnostics(), "DS2Addon.~RESET", SenseKind::Digital),
        "a lifted pull-up leaves ~RESET floating; got {:?}",
        lifted.diagnostics().findings()
    );
}

// ============================================================
// Regression 2: AVDD unstrapped (POR never releases)
// ============================================================

/// Bench bug: the analog domain is fully isolated on the PCB (AVDD/AGND
/// arrive only via the J2 harness), and the datasheet's power-on reset waits
/// for BOTH supplies — an unstrapped AVDD is a permanently silent chip.
#[rstest]
fn avdd_unstrapped_reports_power_net_unsourced() {
    let digital_only = Harness::new()
        .power(ep("BENCH.3V3"), ep("DS2Addon.J1.1"), 3.3)
        .power(ep("BENCH.GND"), ep("DS2Addon.J1.2"), 0.0);

    let system = System::new()
        .board("DS2Addon", ds2_board())
        .harness(digital_only)
        .build()
        .expect("system builds");

    let diags = system.diagnostics();
    assert!(
        diags.contains(&Finding::PowerNetUnsourced {
            net: "DS2Addon.VDDA".to_string()
        }),
        "unstrapped analog rail must be reported; got {:?}",
        diags.findings()
    );
    // The strapped digital rail is fine.
    assert!(!diags.contains(&Finding::PowerNetUnsourced {
        net: "DS2Addon.+3V3".to_string()
    }));

    // And with the full bench harness, the finding disappears.
    let system = System::new()
        .board("DS2Addon", ds2_board())
        .harness(powered_bench_harness())
        .build()
        .expect("system builds");
    assert!(!system.diagnostics().contains(&Finding::PowerNetUnsourced {
        net: "DS2Addon.VDDA".to_string()
    }));
}

// ============================================================
// Every domain referenced: the build's domain lints report nothing
// ============================================================

/// Under the full bench harness every supply pin on the add-on is measured
/// against a ground the harness holds — the ADC's digital supply against
/// its digital ground, its analog supply against its analog ground — so
/// the build's domain lints (`NODES.md` §8 phase 4) have nothing to say:
/// no domain measured against nothing, no rail down. The Edge board alone
/// under its bench rails is the case that reports both
/// (`power_tree.rs`).
#[rstest]
fn the_referenced_add_on_reports_no_unreferenced_domain_and_no_rail_down() {
    behaviour!(Test {
        id: "lint.referenced-add-on-reports-nothing",
        covers: Some("board/src/system.rs#lint_build"),
        given: "the DS2 add-on under the bench harness that holds both its supply pins and \
                both its grounds, analyzed at build",
    });
    expect!(
        "no-domain-lint",
        "the build reports no domain measured against nothing and no rail down",
        "every supply pin on the add-on is measured against a ground the harness holds, and \
         the add-on has no regulator whose output could be down"
    );
    let system = System::new()
        .board("DS2Addon", ds2_board())
        .harness(powered_bench_harness())
        .build()
        .expect("system builds");
    let lints: Vec<&Finding> = system
        .diagnostics()
        .findings()
        .iter()
        .filter(|f| {
            matches!(
                f,
                Finding::UnreferencedDomain { .. } | Finding::RailDown { .. }
            )
        })
        .collect();
    assert_eq!(lints, Vec::<&Finding>::new());
}

// ============================================================
// Regression 3: JP1/JP2 open → floating ADC inputs
// ============================================================

/// Bench bug: the analog input path only reaches the ADC through solder
/// jumpers JP1/JP2 (the filter resistors are DNP). With the jumpers open —
/// their symbol default (`Jumper_NO`) and the state the boards shipped in —
/// AIN0/AIN1 float and conversions slam rail to rail.
#[rstest]
fn open_input_jumpers_float_the_adc_inputs() {
    // Default jumper state (open): AIN0 floats.
    let system = System::new()
        .board("DS2Addon", ds2_board())
        .harness(powered_bench_harness())
        .build()
        .expect("system builds");
    assert!(
        floating(system.diagnostics(), "DS2Addon.AIN0", SenseKind::Analog),
        "open JP1 must leave AIN0 floating; got {:?}",
        system.diagnostics().findings()
    );

    // Closing the jumpers and driving the bridge terminals clears it.
    let driven = powered_bench_harness()
        .power(ep("BENCH.A0"), ep("DS2Addon.J2.3"), 1.65)
        .power(ep("BENCH.A1"), ep("DS2Addon.J2.4"), 1.65);
    let system = System::new()
        .board("DS2Addon", ds2_board())
        .harness(driven)
        .scenario(
            Scenario::default()
                .jumper("DS2Addon.JP1", JumperState::Closed)
                .jumper("DS2Addon.JP2", JumperState::Closed),
        )
        .build()
        .expect("system builds");
    let diags = system.diagnostics();
    assert!(
        !floating(diags, "DS2Addon.AIN0", SenseKind::Analog),
        "closed JP1 with a driven A0 must source AIN0; got {:?}",
        diags.findings()
    );
    assert!(!floating(diags, "DS2Addon.AIN1", SenseKind::Analog));
}

// ============================================================
// Fault algebra: pin_short and net_stuck source a lifted reset
// ============================================================

/// With `R10` lifted, `~RESET` is the converter pin alone. A short from that
/// pin to DVDD, or an ideal source on the net, sources it again.
#[rstest]
fn pin_short_and_net_stuck_source_a_lifted_reset() {
    let lifted = Scenario::default().pin_detach("DS2Addon.R10.2");
    let system = System::new()
        .board("DS2Addon", ds2_board())
        .harness(powered_bench_harness())
        .scenario(lifted.clone())
        .build()
        .expect("system builds");
    assert!(floating(
        system.diagnostics(),
        "DS2Addon.~RESET",
        SenseKind::Digital
    ));

    let system = System::new()
        .board("DS2Addon", ds2_board())
        .harness(powered_bench_harness())
        .scenario(lifted.clone().pin_short("DS2Addon.U1.3", "DS2Addon.U1.13"))
        .build()
        .expect("system builds");
    assert!(
        !floating(system.diagnostics(), "DS2Addon.~RESET", SenseKind::Digital),
        "shorting reset to DVDD must source it; got {:?}",
        system.diagnostics().findings()
    );

    let system = System::new()
        .board("DS2Addon", ds2_board())
        .harness(powered_bench_harness())
        .scenario(lifted.net_stuck("DS2Addon.~RESET", 3.3))
        .build()
        .expect("system builds");
    assert!(!floating(
        system.diagnostics(),
        "DS2Addon.~RESET",
        SenseKind::Digital
    ));
}

// ============================================================
// Fault algebra: net_stuck fighting the powered rail
// ============================================================

/// An injected short-to-ground on the powered digital rail is two
/// disagreeing ideal sources on one net. It must be observable —
/// `Contention` plus a finding — never a silent first-source-wins 3.3 V
/// projection under which the injected fault has zero effect anywhere.
#[rstest]
fn net_stuck_fighting_the_powered_rail_is_observable() {
    let system = System::new()
        .board("DS2Addon", ds2_board())
        .harness(powered_bench_harness())
        .scenario(Scenario::default().net_stuck("DS2Addon.+3V3", 0.0))
        .build()
        .expect("system builds");
    let rail = system
        .nets()
        .iter()
        .find(|n| n.name == "DS2Addon.+3V3")
        .expect("rail net exists");
    assert_eq!(
        rail.state,
        NetState::Contention,
        "a stuck-at-0 on the 3.3 V rail must contend, not vanish"
    );
    assert!(
        system
            .diagnostics()
            .findings()
            .iter()
            .any(|f| matches!(f, Finding::Contention { net, .. } if net == "DS2Addon.+3V3")),
        "the short must raise a finding; got {:?}",
        system.diagnostics().findings()
    );
}

// ============================================================
// Bridge-fed analog inputs through the MNA (no push-pull driver)
// ============================================================

/// The board's measurement path: the external bridge presents its output at
/// the J2 terminals and reaches AIN0/AIN1 through the scenario-populated
/// 4.7 kΩ filter resistors — a sourced passive network with **no push-pull
/// driver anywhere**. The analog inputs must resolve through the cluster
/// solver to the solved node voltage, never the `Pulled` upper-bound
/// fallback. Shorting the ADC input pair (fault algebra) then turns the two
/// filter legs into a genuine two-source divider: the MNA reports the
/// 1.65 V midpoint.
#[rstest]
fn bridge_fed_analog_inputs_solve_through_the_mna() {
    let bridge_fed = powered_bench_harness()
        .power(ep("BENCH.A0"), ep("DS2Addon.J2.3"), 3.3)
        .power(ep("BENCH.A1"), ep("DS2Addon.J2.4"), 0.0);
    let populated_filters = Scenario::default()
        .dnp_override("DS2Addon.R6", DnpState::Populated)
        .value_override("DS2Addon.R6", "4k7")
        .dnp_override("DS2Addon.R7", DnpState::Populated)
        .value_override("DS2Addon.R7", "4k7");

    // Live path: the engine publishes the MNA-solved node voltage.
    let system = System::new()
        .board("DS2Addon", ds2_board())
        .harness(bridge_fed.clone())
        .scenario(populated_filters.clone())
        .start()
        .expect("system starts");
    let ain0 = system.net_state("DS2Addon.AIN0").expect("net exists");
    let NetState::Analog(v) = ain0 else {
        panic!("AIN0 must solve numerically, got {ain0:?}");
    };
    assert!(
        (v - 3.3).abs() < 1e-6,
        "unloaded filter passes the terminal voltage; got {v}"
    );
    system.shutdown();

    let system = System::new()
        .board("DS2Addon", ds2_board())
        .harness(bridge_fed)
        .scenario(populated_filters.pin_short("DS2Addon.U1.11", "DS2Addon.U1.10"))
        .start()
        .expect("system starts");
    let ain = system.net_state("DS2Addon.AIN0").expect("net exists");
    let NetState::Analog(v) = ain else {
        panic!("shorted AIN pair must solve numerically, got {ain:?}");
    };
    assert!((v - 1.65).abs() < 1e-6, "hand check 1.65 V, got {v}");
}
