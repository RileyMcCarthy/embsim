//! The assembled machine's power tree — the module in the Edge board's
//! socket, the DS2 add-on on its cable — once every soft-start has
//! elapsed (`NODES.md` §8 phase 4). Its own binary: the add-on's ADC starts
//! a protocol thread that lives for the rest of the process and parks on
//! the virtual clock every 250 µs (`TESTING.md` rule 5), which is why no
//! other stepped case shares this process with it.
//!
//! Stepped mode (`TESTING.md` rule 9).

use std::collections::BTreeSet;
use std::time::{Duration, Instant};

use embsim_board::{EndpointRef, Level, NetState, System, SystemHandle};
use embsim_core::virtual_clock::{self, ClockMode};
use embsim_models::rail::{AP62301_V_FB_VOLTS, UCC12040_RISE_NS, UCC12040_VISO_SEL_TO_VISO_VOLTS};
use rstest::rstest;
use vibes_behaviour::{behaviour, expect, Test};

mod machine_parts;
use machine_parts::{
    bench_rails, ds2_board, edge_board, force_domain_ground, force_gauge_harness,
    module_socket_harness, shipped_ec32mb_board,
};

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

#[allow(dead_code)]
fn ep(endpoint: &str) -> EndpointRef {
    EndpointRef::parse(endpoint).expect("endpoint parses")
}

/// The voltage a live net reads, or a panic naming what it read instead.
fn volts(system: &SystemHandle, net: &str) -> f64 {
    match system.net_state(net) {
        Some(NetState::Analog(v)) => v,
        other => panic!("{net}: expected an analog voltage, got {other:?}"),
    }
}

/// The module's eight bank rails, by net.
const VIO_RAILS: [&str; 8] = [
    "VIO_00_07",
    "VIO_08_15",
    "VIO_16_23",
    "VIO_24_31",
    "VIO_32_39",
    "VIO_40_47",
    "VIO_48_55",
    "VIO_56_63",
];

/// `U402`'s setpoint from its divider, `R401` 13.3 kΩ over `R403` 10.5 kΩ
/// at `V_FB` = 0.800 V (DS41958 Eq. 8): 1.8133 V.
const U402_V_SET: f64 = AP62301_V_FB_VOLTS * (1.0 + 13.3 / 10.5);
/// The LDOs' 3.3 V, from their value.
const LDO_V_SET: f64 = 3.3;

// ============================================================
// The assembled machine
// ============================================================

/// The assembled machine — the module in its socket, the add-on on its
/// cable, the bench on the 12 V input and the servo domain, the force
/// domain's return held — once its soft-starts elapse: the module's
/// rails, the isolated force domain and the eight isolated inputs' pull-up
/// rail up; the ports nothing is plugged into, the isolated I/O domain
/// whose return nothing ties, and the isolator's orphan ground dark.
#[rstest]
fn the_assembled_machine_powers_its_rails_once_the_soft_starts_elapse() {
    behaviour!(Test {
        id: "power.assembled-machine-rails",
        covers: Some("models/src/rail.rs#Rail::attach"),
        given: "the assembled machine on its bench — 12 volts in, the servo domain and the \
                force domain's return held — run past every soft-start",
    });
    expect!(
        "module-rails-up",
        "every bank rail of the module reads 3.3 volts and its core rail 1.813 volts, each \
         within a millivolt",
        "the carrier's 5 volts is the Edge board's own 5 volt buck, passed through the \
         module's polarity FET to its bucks"
    );
    expect!(
        "force-domain-at-five-volts",
        "the isolated force domain reads the isolated DC/DC's 5.0 volts",
        "its select pin is shorted to its output and its return is held over the cable"
    );
    expect!(
        "isolated-inputs-pulled-up",
        "the eight isolated inputs read pulled high through their 1 kilohm pull-ups",
        "their pull-up rail is the module's bank rail for pins 16 to 23, up once the LDO \
         behind it has risen"
    );
    expect!(
        "unplugged-ports-dark",
        "the Raspberry-Pi header's supply, the isolated I/O domain and the servo isolator's \
         secondary ground float",
        "nothing is plugged into the header, nothing ties the isolated I/O return, and the \
         isolator's secondary ground is on a net only its decoupling capacitor shares"
    );
    // Stepped mode first, before the add-on's ADC exists (its protocol
    // thread is a registered clock actor from its first instant).
    virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);
    let system = System::new()
        .board("EC32MB", shipped_ec32mb_board())
        .board("EdgeBoard", edge_board())
        .board("DS2Addon", ds2_board())
        .harness(module_socket_harness("EC32MB", "EdgeBoard"))
        .harness(force_gauge_harness("EdgeBoard", "DS2Addon"))
        .harness(bench_rails("EdgeBoard"))
        .harness(force_domain_ground("DS2Addon"))
        .start()
        .expect("the machine starts");
    // Wait on every voltage asserted, not on one rail reading anything:
    // the eight LDOs publish one after another as their senses are
    // delivered, so a poller can see one bank rail up while another is
    // still floating — and an LDO whose `IN` and `EN` share the buck's rail
    // is delivered `IN` first, publishing its 100 Ω active discharge (input
    // up, enable still floating-to-off), 0 V, before the `EN` delivery
    // lifts it to 3.3 V, at the same instant.
    let at = |net: &str, volts: f64| matches!(system.net_state(net), Some(NetState::Analog(v)) if (v - volts).abs() < 1e-3);
    let rails_up = || {
        VIO_RAILS
            .iter()
            .all(|rail| at(&format!("EC32MB.{rail}"), LDO_V_SET))
            && at("EC32MB.Common_VDD", U402_V_SET)
    };
    assert!(
        wait_for(rails_up, SETTLE),
        "the module's rails rise to their setpoints; got {:?}",
        VIO_RAILS
            .iter()
            .map(|rail| format!("{rail}={:?}", system.net_state(&format!("EC32MB.{rail}"))))
            .chain([format!(
                "Common_VDD={:?}",
                system.net_state("EC32MB.Common_VDD")
            )])
            .collect::<Vec<_>>()
    );
    for rail in VIO_RAILS {
        let v = volts(&system, &format!("EC32MB.{rail}"));
        assert!((v - LDO_V_SET).abs() < 1e-3, "{rail} reads {v}");
    }
    let core = volts(&system, "EC32MB.Common_VDD");
    assert!((core - U402_V_SET).abs() < 1e-3, "Common_VDD reads {core}");
    assert!(
        wait_for(
            || matches!(
                system.net_state("EdgeBoard./MaD_Edge_Sheet2/IFG_5V"),
                Some(NetState::Analog(v)) if (v - UCC12040_VISO_SEL_TO_VISO_VOLTS).abs() < 1e-9
            ),
            SETTLE
        ),
        "the force domain rises {} ns after its input; got {:?}",
        UCC12040_RISE_NS,
        system.net_state("EdgeBoard./MaD_Edge_Sheet2/IFG_5V")
    );
    let force = volts(&system, "EdgeBoard./MaD_Edge_Sheet2/IFG_5V");
    assert!(
        (force - UCC12040_VISO_SEL_TO_VISO_VOLTS).abs() < 1e-9,
        "IFG_5V reads {force}"
    );
    for pin in 16..=23u32 {
        let net = format!("EdgeBoard.P{pin}");
        assert!(
            wait_for(
                || system.net_state(&net) == Some(NetState::Pulled(Level::High, 1_000.0)),
                SETTLE
            ),
            "{net}: {:?}",
            system.net_state(&net)
        );
    }
    let dark: BTreeSet<&str> = [
        "EdgeBoard./MaD_Edge_Sheet2/RPI_5V",
        "EdgeBoard./MaD_Edge_Sheet2/5V_IO",
        "EdgeBoard.Net-(IC14-GND2_1)",
    ]
    .into_iter()
    .collect();
    for net in &dark {
        assert_eq!(
            system.net_state(net),
            Some(NetState::Floating),
            "{net} stays dark"
        );
    }
    system.shutdown();
}
