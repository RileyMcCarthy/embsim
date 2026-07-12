//! The phase-1 findings gate: the July 2026 DS2Addon bench bring-up bugs,
//! reproduced as build-time findings against the real board netlist.
//!
//! Each test uses `tests/fixtures/ds2_addon.net` — the actual
//! `kicad-cli sch export netlist` of the board that was debugged on the
//! bench — so these regressions hold the engine to the same truth the
//! hardware exhibited.

use embsim_board::{
    AttachError, Board, Component, ComponentNetIo, Diagnostics, EndpointRef, Finding, Harness,
    JumperState, PartRegistry, PinDecl, PinKind, Scenario, SenseKind, StreamRole, System,
};

// ============================================================
// ADS122U04 pin facade (TSSOP-16, TI SBAS752B pin table p.3)
// ============================================================

/// Build-time pin facade of the ADS122U04 (the protocol model lives in
/// `embsim-models`; this facade is what the board engine wires and checks).
struct Ads122u04Facade;

const NONE: Option<StreamRole> = None;

/// TSSOP-16 (PW) pinout per SBAS752B: 1 GPIO1, 2 GPIO0, 3 ~RESET, 4 DGND,
/// 5 AVSS, 6 AIN3, 7 AIN2, 8 REFN, 9 REFP, 10 AIN1, 11 AIN0, 12 AVDD,
/// 13 DVDD, 14 GPIO2/DRDY, 15 TX, 16 RX.
const ADS122U04_PINS: [PinDecl; 16] = [
    pin("1", Some("GPIO1"), PinKind::DigitalIn, NONE),
    pin("2", Some("GPIO0"), PinKind::DigitalIn, NONE),
    pin("3", Some("~RESET"), PinKind::DigitalIn, NONE),
    pin("4", Some("DGND"), PinKind::PowerIn, NONE),
    pin("5", Some("AVSS"), PinKind::PowerIn, NONE),
    pin("6", Some("AIN3"), PinKind::Analog, NONE),
    pin("7", Some("AIN2"), PinKind::Analog, NONE),
    pin("8", Some("REFN"), PinKind::Analog, NONE),
    pin("9", Some("REFP"), PinKind::Analog, NONE),
    pin("10", Some("AIN1"), PinKind::Analog, NONE),
    pin("11", Some("AIN0"), PinKind::Analog, NONE),
    pin("12", Some("AVDD"), PinKind::PowerIn, NONE),
    pin("13", Some("DVDD"), PinKind::PowerIn, NONE),
    pin("14", Some("DRDY"), PinKind::DigitalIn, NONE),
    pin(
        "15",
        Some("TX"),
        PinKind::DigitalOut,
        Some(StreamRole::Producer { baud_hz: 115_200 }),
    ),
    pin(
        "16",
        Some("RX"),
        PinKind::DigitalIn,
        Some(StreamRole::Consumer { baud_hz: 115_200 }),
    ),
];

const fn pin(
    number: &'static str,
    name: Option<&'static str>,
    kind: PinKind,
    stream: Option<StreamRole>,
) -> PinDecl {
    PinDecl {
        number,
        name,
        kind,
        stream,
        drive_impedance: None,
    }
}

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
// Regression 1: floating ~RESET (net has exactly one pin)
// ============================================================

/// Bench bug: the board leaves the ADS122U04's active-low reset floating
/// (the `~{RESET}` net contains only U1 pin 3). Two chips appeared dead for
/// days before a multimeter found it. The engine must report it at build,
/// before any traffic.
#[test]
fn floating_reset_is_reported_at_build() {
    let system = System::new()
        .board("DS2Addon", ds2_board())
        .harness(powered_bench_harness())
        .build()
        .expect("system builds");

    let diags = system.diagnostics();
    assert!(
        floating(diags, "DS2Addon.~RESET", SenseKind::Digital),
        "the one-pin ~RESET net must surface as a floating digital sense; got {:?}",
        diags.findings()
    );

    // Powered rails must NOT be floating: the finding is specific.
    assert!(!floating(diags, "DS2Addon.+3V3", SenseKind::Digital));
    assert!(!floating(diags, "DS2Addon.VDDA", SenseKind::Analog));
}

// ============================================================
// Regression 2: AVDD unstrapped (POR never releases)
// ============================================================

/// Bench bug: the analog domain is fully isolated on the PCB (AVDD/AGND
/// arrive only via the J2 harness), and the datasheet's power-on reset waits
/// for BOTH supplies — an unstrapped AVDD is a permanently silent chip.
#[test]
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
// Regression 3: JP1/JP2 open → floating ADC inputs
// ============================================================

/// Bench bug: the analog input path only reaches the ADC through solder
/// jumpers JP1/JP2 (the filter resistors are DNP). With the jumpers open —
/// their symbol default (`Jumper_NO`) and the state the boards shipped in —
/// AIN0/AIN1 float and conversions slam rail to rail.
#[test]
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
// Fault algebra: pin_short models the reset bodge wire
// ============================================================

/// The bench fix for the floating reset was a bodge wire from the reset pad
/// to the 3.3 V rail. `pin_short` is exactly that fault-algebra primitive:
/// union the two pins' nets. With the bodge applied, the floating finding
/// must disappear; `net_stuck` (an injected ideal source) must clear it too.
#[test]
fn pin_short_and_net_stuck_model_the_reset_bodge() {
    // Baseline: floating (regression 1).
    let system = System::new()
        .board("DS2Addon", ds2_board())
        .harness(powered_bench_harness())
        .build()
        .expect("system builds");
    assert!(floating(
        system.diagnostics(),
        "DS2Addon.~RESET",
        SenseKind::Digital
    ));

    // Bodge wire: short U1.3 (~RESET) to U1.13 (DVDD, on the powered +3V3
    // net) — the finding clears.
    let system = System::new()
        .board("DS2Addon", ds2_board())
        .harness(powered_bench_harness())
        .scenario(Scenario::default().pin_short("DS2Addon.U1.3", "DS2Addon.U1.13"))
        .build()
        .expect("system builds");
    assert!(
        !floating(system.diagnostics(), "DS2Addon.~RESET", SenseKind::Digital),
        "the pin_short bodge must source the reset net; got {:?}",
        system.diagnostics().findings()
    );

    // Equivalent via net_stuck (an injected rail source on the net).
    let system = System::new()
        .board("DS2Addon", ds2_board())
        .harness(powered_bench_harness())
        .scenario(Scenario::default().net_stuck("DS2Addon.~RESET", 3.3))
        .build()
        .expect("system builds");
    assert!(!floating(
        system.diagnostics(),
        "DS2Addon.~RESET",
        SenseKind::Digital
    ));
}
