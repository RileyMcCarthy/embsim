//! The build-time fixed point (`NODES.md` §5): `System::build` replays the
//! drives components issue in response to the states they are delivered,
//! and repeats until nothing changes, so the build snapshot is the state
//! the live engine settles to before its first wake — a chain of
//! sense→drive components (the GPIO bridge driving its power-on level, an
//! isolator driving at the rail it senses, the EC32MB's power tree three
//! senses deep) is analyzed where it rests, not one hop short. Bounded:
//! a system still changing after `BUILD_FIXED_POINT_BOUND` rounds gets the
//! last round's snapshot and `Finding::BuildNotSettled`.
//!
//! Beside it, the declared idle drive (`PinDecl::idle`): the drive a pin
//! presents from attach, said once on the declaration instead of released
//! in `attach` — and refused on a pin that has no drive slot, since the
//! engine could only drop it. And what the build leaves behind: nothing.
//! The sense callbacks it records at attach capture the handles their
//! components keep, so the build must release them when it is done.
//!
//! Its own binary per `TESTING.md` rule 5: the live comparisons start
//! engines against the process-global clock.

mod machine_parts;

use std::sync::{Arc, Mutex, Once};
use std::time::{Duration, Instant};

use embsim_board::{
    digital_drive, level_of, AttachError, Board, BoardError, Component, ComponentNetIo, Finding,
    IdleDrive, Level, NetId, NetState, PartRegistry, PinDecl, PinHandle, PinKind, System,
    SystemError, TheveninDrive, BUILD_FIXED_POINT_BOUND,
};
use embsim_core::virtual_clock;
use machine_parts::{edge_board, shipped_ec32mb_board};
use rstest::rstest;
use vibes_behaviour::{behaviour, expect, Test};

// ============================================================
// Plumbing
// ============================================================

/// The engine's timer wheel samples the process-global virtual clock, and
/// `init` re-anchors it — so it runs once per binary. Unpaced: nothing here
/// waits on virtual time.
fn ensure_clock() {
    static CLOCK: Once = Once::new();
    CLOCK.call_once(|| virtual_clock::init(0.0, 1_000_000));
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

/// Bitwise state equality: the derived `PartialEq` compares voltages with
/// IEEE semantics, under which a NaN-carrying state never equals itself.
fn same_state(a: NetState, b: NetState) -> bool {
    match (a, b) {
        (NetState::Analog(x), NetState::Analog(y)) => x.total_cmp(&y).is_eq(),
        (NetState::Pulled(la, xa), NetState::Pulled(lb, xb)) => {
            la == lb && xa.total_cmp(&xb).is_eq()
        }
        _ => a == b,
    }
}

// ============================================================
// Fixtures: a driver, a repeater, a ring
// ============================================================

/// One output, idling at the kind's default (driven high); drives low at
/// attach — the attach-time drive the replay exists for.
struct Source {
    pins: [PinDecl; 1],
}

impl Source {
    fn new() -> Self {
        Self {
            pins: [PinDecl::digital_out("1")],
        }
    }
}

impl Component for Source {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }
    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        io.pin("1")?.set_drive(Some(digital_drive(Level::Low)));
        Ok(())
    }
}

/// Senses pin 1, drives pin 2 at the sensed level (released when the input
/// has no level). Its output idles **released by declaration**, so a
/// repeater whose input floats presents nothing — with no release in
/// `attach`.
struct Repeater {
    pins: [PinDecl; 2],
    invert: bool,
}

impl Repeater {
    fn new(invert: bool, idle: IdleDrive) -> Self {
        Self {
            pins: [
                PinDecl::digital_in("1"),
                PinDecl::digital_out("2").with_idle(idle),
            ],
            invert,
        }
    }
}

impl Component for Repeater {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }
    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let out: PinHandle = io.pin("2")?;
        let invert = self.invert;
        io.on_sense("1", move |state| {
            let drive = level_of(state).map(|level| {
                let level = if invert {
                    match level {
                        Level::High => Level::Low,
                        Level::Low => Level::High,
                    }
                } else {
                    level
                };
                digital_drive(level)
            });
            out.set_drive(drive);
        })
    }
}

/// A pin that only declares an idle drive and never drives it.
struct Idler {
    pins: [PinDecl; 1],
}

impl Component for Idler {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }
    fn attach(&mut self, _io: ComponentNetIo) -> Result<(), AttachError> {
        Ok(())
    }
}

/// A sensor that records what it is delivered.
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
        io.on_sense("1", move |state| seen.lock().unwrap().push(state))
    }
}

/// A sensor whose callback captures its own pin handle and a token, the
/// shape every model's sense closure has (a handle to publish through, the
/// state it publishes from): what the build records and must let go of.
struct Holder {
    pins: [PinDecl; 1],
    token: Arc<()>,
}

impl Component for Holder {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }
    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let pin: PinHandle = io.pin("1")?;
        let token = Arc::clone(&self.token);
        io.on_sense("1", move |_| {
            let _ = (&pin, &token);
        })
    }
}

/// One holder on one net.
const HOLD: &str = r#"(export (version "E")
  (components
    (comp (ref "U1") (value "holder") (libsource (lib "t") (part "HOLDER") (description ""))))
  (nets
    (net (code "1") (name "N") (class "Default")
      (node (ref "U1") (pin "1") (pintype "input")))))"#;

/// U1 drives X; U2 repeats X onto Y; U3 repeats Y onto Z — two senses deep.
const CHAIN: &str = r#"(export (version "E")
  (components
    (comp (ref "U1") (value "src") (libsource (lib "t") (part "SOURCE") (description "")))
    (comp (ref "U2") (value "rep") (libsource (lib "t") (part "REPEATER") (description "")))
    (comp (ref "U3") (value "rep") (libsource (lib "t") (part "REPEATER") (description ""))))
  (nets
    (net (code "1") (name "X") (class "Default")
      (node (ref "U1") (pin "1") (pintype "output"))
      (node (ref "U2") (pin "1") (pintype "input")))
    (net (code "2") (name "Y") (class "Default")
      (node (ref "U2") (pin "2") (pintype "output"))
      (node (ref "U3") (pin "1") (pintype "input")))
    (net (code "3") (name "Z") (class "Default")
      (node (ref "U3") (pin "2") (pintype "output")))))"#;

/// An inverter whose output is its own input: never at rest.
const RING: &str = r#"(export (version "E")
  (components
    (comp (ref "U1") (value "inv") (libsource (lib "t") (part "INVERTER") (description ""))))
  (nets
    (net (code "1") (name "A") (class "Default")
      (node (ref "U1") (pin "1") (pintype "input"))
      (node (ref "U1") (pin "2") (pintype "output")))))"#;

/// A pin that idles as declared, and a sensor on its net.
const IDLE: &str = r#"(export (version "E")
  (components
    (comp (ref "U1") (value "idler") (libsource (lib "t") (part "IDLER") (description "")))
    (comp (ref "U2") (value "sensor") (libsource (lib "t") (part "SENSOR") (description ""))))
  (nets
    (net (code "1") (name "N") (class "Default")
      (node (ref "U1") (pin "1") (pintype "output"))
      (node (ref "U2") (pin "1") (pintype "input")))))"#;

fn chain_board() -> Board {
    let mut registry = PartRegistry::new();
    registry.register("SOURCE", |_| Box::new(Source::new()));
    registry.register("REPEATER", |_| {
        Box::new(Repeater::new(false, IdleDrive::Released))
    });
    Board::from_netlist(embsim_board::netlist::parse(CHAIN).unwrap(), &registry).unwrap()
}

fn state_of(system: &embsim_board::BuiltSystem, net: &str) -> NetState {
    let id = system.net_id(net).unwrap_or_else(|| panic!("{net} exists"));
    system.nets()[id.0].state
}

// ============================================================
// The fixed point
// ============================================================

/// The replay iterates: a drive that changes a state the next component
/// senses gets that component's answer replayed too, until nothing moves.
#[rstest]
fn a_chain_of_sense_to_drive_components_settles_in_the_build() {
    behaviour!(Test {
        id: "build.fixed-point-settles-a-chain",
        covers: Some("board/src/system.rs#System::build"),
        given: "a driver that drives low when it attaches, feeding a repeater onto a second \
                net, feeding a second repeater onto a third",
    });
    expect!(
        "chain-at-rest",
        "the build snapshot reads all three nets driven low",
        "each repeater answers the state it is delivered, and the build delivers every \
         state its replay changes until no component answers with a different drive"
    );
    expect!(
        "settled",
        "the build reports nothing unsettled",
        "a chain with no loop reaches rest in as many rounds as it is deep"
    );

    let built = System::new().board("B", chain_board()).build().unwrap();
    for net in ["B.X", "B.Y", "B.Z"] {
        assert_eq!(state_of(&built, net), NetState::Driven(Level::Low), "{net}");
    }
    assert!(
        !built
            .diagnostics()
            .findings()
            .iter()
            .any(|f| matches!(f, Finding::BuildNotSettled { .. })),
        "{:?}",
        built.diagnostics().findings()
    );
}

/// The declared idle drive is the pin's state from attach: released by
/// declaration floats, the kind's default drives high, a declared Thevenin
/// drives what it says.
#[rstest]
#[case::released(IdleDrive::Released, NetState::Floating)]
#[case::kind_default(IdleDrive::KindDefault, NetState::Driven(Level::High))]
#[case::declared_low(IdleDrive::Thevenin(TheveninDrive { volts: 0.0, impedance: 25.0 }), NetState::Driven(Level::Low))]
fn a_pins_declared_idle_drive_is_its_state_at_build(
    #[case] idle: IdleDrive,
    #[case] expected: NetState,
) {
    behaviour!(Test {
        id: "pin.declared-idle-drive",
        covers: Some("board/src/system.rs#add_pin_descriptor"),
        given: "a push-pull output that declares what it idles at and never drives, with a \
                sensor on its net",
    });
    expect!(
        "net-as-declared",
        "the net reads what the declaration says: floating for a released idle, driven \
         high for the kind's default, the declared level for a declared drive",
        "the drive a pin presents until its component drives it is a static fact of the \
         part, declared once on the pin"
    );
    expect!(
        "sensor-delivered",
        "the sensor on that net is delivered exactly that state, once"
    );

    let seen = Arc::new(Mutex::new(Vec::new()));
    let sink = Arc::clone(&seen);
    let mut registry = PartRegistry::new();
    registry.register("IDLER", move |_| {
        Box::new(Idler {
            pins: [PinDecl::digital_out("1").with_idle(idle)],
        })
    });
    registry.register("SENSOR", move |_| {
        Box::new(Sensor {
            pins: [PinDecl::digital_in("1")],
            seen: Arc::clone(&sink),
        })
    });
    let board =
        Board::from_netlist(embsim_board::netlist::parse(IDLE).unwrap(), &registry).unwrap();
    let built = System::new().board("B", board).build().unwrap();
    assert_eq!(state_of(&built, "B.N"), expected);
    assert_eq!(*seen.lock().unwrap(), vec![expected]);
}

/// A ring never rests, and the build says so instead of looping: the
/// bound is the finding.
#[rstest]
fn a_ring_that_never_settles_is_reported_within_the_bound() {
    behaviour!(Test {
        id: "build.fixed-point-bounded",
        covers: Some("board/src/system.rs#System::build"),
        given: "an inverter whose output is wired back to its own input, its output idling \
                high by declaration",
    });
    expect!(
        "reported",
        "the build finishes and reports the system unsettled after its bounded number of \
         rounds, naming the net still changing",
        "a loop that inverts itself has no rest state, and the build's job is to say so"
    );

    let mut registry = PartRegistry::new();
    registry.register("INVERTER", |_| {
        Box::new(Repeater::new(
            true,
            IdleDrive::Thevenin(digital_drive(Level::High)),
        ))
    });
    let board =
        Board::from_netlist(embsim_board::netlist::parse(RING).unwrap(), &registry).unwrap();
    let built = System::new().board("B", board).build().unwrap();
    assert!(
        built.diagnostics().contains(&Finding::BuildNotSettled {
            passes: BUILD_FIXED_POINT_BOUND,
            nets: vec!["B.A".to_string()],
        }),
        "{:?}",
        built.diagnostics().findings()
    );
}

/// What the build leaves behind: the callbacks it recorded, and everything
/// they captured, are released when it returns. A recorded callback holds
/// its component's pin handle, and the handle holds a link back to the
/// build's own sense log — a cycle unless the link's reference is weak.
#[rstest]
fn the_build_releases_every_sense_callback_it_recorded() {
    behaviour!(Test {
        id: "build.releases-recorded-senses",
        covers: Some("board/src/system.rs#System::build"),
        given: "a board with one part whose sense callback captures its own pin handle and a \
                token, analyzed at build and then dropped along with the registry that made it",
    });
    expect!(
        "token-freed",
        "nothing holds the token once the build result and the registry are gone",
        "the build owns the callbacks it records and releases them with the analysis; a \
         model's captured state (a flash image, say) must not outlive the build that read it"
    );

    let token = Arc::new(());
    let weak = Arc::downgrade(&token);
    let mut registry = PartRegistry::new();
    registry.register("HOLDER", move |_| {
        Box::new(Holder {
            pins: [PinDecl::digital_in("1")],
            token: Arc::clone(&token),
        })
    });
    let board =
        Board::from_netlist(embsim_board::netlist::parse(HOLD).unwrap(), &registry).unwrap();
    let built = System::new().board("B", board).build().unwrap();
    assert_eq!(state_of(&built, "B.N"), NetState::Floating);
    drop(built);
    drop(registry);
    assert_eq!(
        weak.strong_count(),
        0,
        "the build kept a reference to the part's sense callback alive"
    );
}

/// The route a declaration reaches the build by: a netlist part through the
/// registry, or a bench component attached to the system directly.
#[derive(Debug, Clone, Copy)]
enum Route {
    Netlist,
    Bench,
}

/// An idle drive is honoured on every pin with a drive slot; a power or
/// passive pin has none, so an idle drive declared on one is refused at
/// build, naming the pin, by both routes a declaration can arrive.
#[rstest]
#[case::power_in_released(PinKind::PowerIn, IdleDrive::Released)]
#[case::power_in_thevenin(PinKind::PowerIn, IdleDrive::Thevenin(TheveninDrive { volts: 3.3, impedance: 0.1 }))]
#[case::power_out_released(PinKind::PowerOut, IdleDrive::Released)]
#[case::power_out_thevenin(PinKind::PowerOut, IdleDrive::Thevenin(TheveninDrive { volts: 3.3, impedance: 0.1 }))]
#[case::passive_released(PinKind::Passive, IdleDrive::Released)]
#[case::passive_thevenin(PinKind::Passive, IdleDrive::Thevenin(TheveninDrive { volts: 0.0, impedance: 25.0 }))]
fn an_idle_drive_on_a_pin_without_a_drive_slot_is_refused_at_build(
    #[case] kind: PinKind,
    #[case] idle: IdleDrive,
    #[values(Route::Netlist, Route::Bench)] route: Route,
) {
    behaviour!(Test {
        id: "pin.idle-drive-needs-a-drive-slot",
        covers: Some("board/src/board.rs#validate_idle_drives"),
        given: "a part declaring an idle drive — released, or a voltage behind an impedance — \
                on a power-in, power-out or passive pin, brought to the build as a netlist part \
                or as a bench component",
    });
    expect!(
        "build-refused",
        "the build fails and the error names the part and the pin",
        "a power or passive pin has no drive slot, so the declaration is a static fact the \
         engine could only drop; refusing it keeps every declaration honoured"
    );

    let decl = PinDecl::new("1", kind).with_idle(idle);
    match route {
        Route::Netlist => {
            let mut registry = PartRegistry::new();
            registry.register("IDLER", move |_| Box::new(Idler { pins: [decl] }));
            registry.register("SENSOR", |_| {
                Box::new(Sensor {
                    pins: [PinDecl::digital_in("1")],
                    seen: Arc::default(),
                })
            });
            let error = Board::from_netlist(embsim_board::netlist::parse(IDLE).unwrap(), &registry)
                .expect_err("an idle drive on a slotless pin is refused");
            assert!(
                matches!(
                    &error,
                    BoardError::IdleOnSlotlessPin { reference, pin } if reference == "U1" && pin == "1"
                ),
                "{error:?}"
            );
        }
        Route::Bench => {
            let error = System::new()
                .component("BENCH", Box::new(Idler { pins: [decl] }))
                .build()
                .expect_err("an idle drive on a slotless pin is refused");
            assert!(
                matches!(
                    &error,
                    SystemError::Board {
                        name,
                        error: BoardError::IdleOnSlotlessPin { reference, pin }
                    } if name == "BENCH" && reference == "BENCH" && pin == "1"
                ),
                "{error:?}"
            );
        }
    }
}

// ============================================================
// Build == live, before the first wake
// ============================================================

/// The invariant `NODES.md` §5 asks for on both reference boards: the
/// build snapshot equals the live system's state before its first wake.
/// The chain fixture is the case the old single replay got wrong.
#[rstest]
#[case::chain(chain_board as fn() -> Board)]
#[case::ec32mb(shipped_ec32mb_board as fn() -> Board)]
#[case::edge(edge_board as fn() -> Board)]
fn the_build_snapshot_is_the_live_systems_state_before_its_first_wake(
    #[case] build: fn() -> Board,
) {
    behaviour!(Test {
        id: "build.snapshot-equals-live-pre-wake",
        covers: Some("board/src/system.rs#System::build"),
        given: "a board — the P2-EC32MB module, the MaD EdgeBoard, or a chain of sense-to-\
                drive parts — analyzed at build and then started live with nothing scheduled",
    });
    expect!(
        "same-states",
        "every net's live state, once the engine has applied the parts' attach-time drives, \
         is exactly the state the build snapshot recorded for it",
        "build and live share one resolver and the build replays the same cascade of \
         drives the live engine settles, so the two can never fork"
    );
    expect!(
        "settled",
        "the build reports nothing unsettled",
        "a board's power tree and sense-to-drive chains reach rest within the bound"
    );

    ensure_clock();
    let built = System::new().board("B", build()).build().unwrap();
    assert!(
        !built
            .diagnostics()
            .findings()
            .iter()
            .any(|f| matches!(f, Finding::BuildNotSettled { .. })),
        "{:?}",
        built.diagnostics().findings()
    );

    let live = System::new().board("B", build()).start().unwrap();
    let mismatches = || -> Vec<(String, NetState, Option<NetState>)> {
        built
            .nets()
            .iter()
            .enumerate()
            .filter_map(|(i, net)| {
                let live_state = live.net_state_of(NetId(i));
                (live_state.is_none_or(|s| !same_state(s, net.state)))
                    .then(|| (net.name.clone(), net.state, live_state))
            })
            .collect()
    };
    let settled = wait_for(|| mismatches().is_empty(), SETTLE);
    assert!(
        settled,
        "the live system rests where the build said, on every net; still differing: {:?}",
        mismatches()
    );
    drop(live);
}
