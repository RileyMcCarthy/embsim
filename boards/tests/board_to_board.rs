//! Two boards wired connector to connector by a project, live.
//!
//! `projects/header-pair.toml` builds two copies of the header board — a
//! two-pin connector `J1` (`SIG`, `GND`) and a 10 kΩ resistor `R1` from
//! `SIG` to `GND` — as `LEFT` and `RIGHT`, and mates their `J1`s pin for
//! pin (`[[mate]]`), the bench's 0 V on `LEFT`'s ground. The case adds one
//! thing the project does not: a pad on `LEFT.J1.1` that it drives from its
//! own thread, and it reads what arrives on `RIGHT`'s side of the mate.
//!
//! Stepped (`TESTING.md` rule 9), its own binary: the clock re-anchored
//! stepped, the system started with time held, the case's thread a
//! registered actor, every read after a virtual settle, no
//! `QuiescenceTimeout` at the end.

use std::path::PathBuf;
use std::sync::{Arc, Mutex};

use embsim_board::{
    AttachError, Component, ComponentNetIo, EndpointRef, Finding, Harness, NetState, PinDecl,
    PinHandle, Project, SystemHandle, TheveninDrive,
};
use embsim_boards::catalog::StandardCatalog;
use embsim_core::virtual_clock::{self, ClockMode};
use rstest::rstest;
use vibes_behaviour::{behaviour, expect, Test};

/// The virtual time the case hands the engine after each stimulus: 1 µs.
/// Nothing on either board arms an instant — two resistors and two
/// connectors — so any window reads the system at rest.
const SETTLE_NS: u64 = 1_000;

/// The header board's `R1`, 10 kΩ (`projects/header.net`).
const R1_OHMS: f64 = 10_000.0;

/// The pad's source impedance, a push-pull output's.
const PAD_OHMS: f64 = 25.0;

/// The level the pad drives high.
const LOGIC_VOLTS: f64 = 3.3;

/// A bench pad the case drives: one output, released until driven, with
/// the current instrument on it — which also has its node solved, so the
/// node reads its voltage.
struct Pad {
    pins: [PinDecl; 1],
    handle: Arc<Mutex<Option<PinHandle>>>,
}

impl Component for Pad {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        *self.handle.lock().unwrap() = Some(io.pin("OUT")?);
        io.on_branch("OUT", |_| {})
    }
}

fn settle() {
    virtual_clock::wait_virtual_ns(SETTLE_NS);
}

fn right_sig(system: &SystemHandle) -> NetState {
    system.net_state("RIGHT.SIG").expect("RIGHT.SIG is a net")
}

#[rstest]
fn a_level_driven_on_one_boards_connector_reads_on_the_other_board() {
    behaviour!(Test {
        id: "project.board-to-board-wire",
        covers: Some("board/src/project.rs#Project::instantiate"),
        given: "two boards, each a connector with a 10 kilohm resistor from signal to \
                ground, their connectors joined by a project file, one grounded by the \
                bench, a pad on the first's signal",
    });
    expect!(
        "released-at-ground",
        "with the pad released, the second board's signal reads the bench's 0 volts"
    );
    expect!(
        "high-divides-over-both",
        "the pad's 3.3 volts reach the second board divided between the pad's 25 ohms and \
         both resistors in parallel",
        "the mate makes the two boards' signal pins one node, so both resistors load the pad"
    );
    expect!(
        "pad-current",
        "the pad sources 3.3 volts over the two resistors in parallel plus its own 25 ohms, \
         to a picoamp"
    );
    expect!(
        "low-arrives",
        "with the pad driving 0 volts, the second board's signal reads 0 volts"
    );
    let path: PathBuf = [env!("CARGO_MANIFEST_DIR"), "projects", "header-pair.toml"]
        .iter()
        .collect();
    let handle: Arc<Mutex<Option<PinHandle>>> = Arc::new(Mutex::new(None));
    let pad = Pad {
        pins: [PinDecl::digital_out("OUT").with_idle(None)],
        handle: Arc::clone(&handle),
    };

    virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);
    let system = Project::load(&path)
        .expect("the project loads")
        .instantiate(&StandardCatalog)
        .expect("the project builds")
        .component("PAD", Box::new(pad))
        .harness(Harness::new().connect(
            EndpointRef::parse("PAD.OUT").expect("endpoint parses"),
            EndpointRef::parse("LEFT.J1.1").expect("endpoint parses"),
        ))
        .hold_time()
        .start()
        .expect("the two boards start");
    let actor = virtual_clock::register_actor("board-to-board-case");
    system.release_time();
    settle();
    let pad = handle.lock().unwrap().clone().expect("the pad attached");

    assert_eq!(right_sig(&system), NetState::Analog(0.0));

    let drive = |volts| {
        pad.set_drive(Some(TheveninDrive {
            volts,
            impedance: PAD_OHMS,
        }));
        settle();
        right_sig(&system)
    };
    // Both boards' R1 from the joined node to ground, in parallel.
    let load = R1_OHMS / 2.0;
    match drive(LOGIC_VOLTS) {
        NetState::Analog(v) => {
            let divided = LOGIC_VOLTS * load / (load + PAD_OHMS);
            assert!(
                (v - divided).abs() < 1e-9,
                "RIGHT.SIG reads {v}, divided {divided}"
            );
        }
        other => panic!("RIGHT.SIG: expected the solved voltage, got {other:?}"),
    }
    let into_pad = system
        .pin_current("PAD.OUT")
        .expect("the pad is an instrument");
    let sourced = LOGIC_VOLTS / (load + PAD_OHMS);
    assert!(
        (into_pad + sourced).abs() < 1e-12,
        "into the pad {into_pad} A; it sources {sourced} A"
    );
    assert_eq!(drive(0.0), NetState::Analog(0.0));

    let stalled: Vec<Finding> = system
        .findings()
        .into_iter()
        .filter(|finding| matches!(finding, Finding::QuiescenceTimeout { .. }))
        .collect();
    assert!(
        stalled.is_empty(),
        "the engine stopped waiting for the case's thread: {stalled:?}"
    );
    drop(actor);
    system.shutdown();
}
