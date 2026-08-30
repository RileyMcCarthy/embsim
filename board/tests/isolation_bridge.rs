//! The MaD EdgeBoard with its isolation parts **promoted from stubs to
//! models** — the seam between the P2 and the machine, live on the real
//! netlist.
//!
//! # What this binary is for
//!
//! `edgeboard.rs` proves the two RS-422 parts around the barrier resolve real
//! levels, and injects on the isolator output nets because "once the servo
//! isolator is modeled" was a future tense. This binary is that future tense:
//! it registers [`embsim_models::isolation`] for the five parts that stood
//! between the MCU pins and the machine and asserts the signals now cross.
//!
//! | Path | Part that blocked it | Asserted here |
//! |---|---|---|
//! | `P8` STEP → the stepper driver | `IC14` `ISO6741DWR` | a level, and a rate-carried train |
//! | `P7` DIR → the stepper driver | `IC14`, same part | a level, independently of `P8` |
//! | `P6` ENA → the enable sink | `IC14`, then `Q1` `NPN` | a level through both |
//! | encoder → `P9`..`P12` | `IC16` `ISO6740FDWR` | the receiver's output, and the fail-safe |
//! | end switch → `P19` | `IC9` `NSI50010` + `U6` `VO2631` | a closed contact lighting the opto |
//!
//! Plus the load-bearing budget: [`a_step_train_crosses_the_barrier_at_a_bounded_engine_cost`].
//!
//! # The rig
//!
//! One real board (`fixtures/mad_edge.net`), the bench straps `edgeboard.rs`
//! uses, and two additions the board itself cannot supply:
//!
//! - a strap on the module socket's `VIO_16_23` finger, because the P2's own
//!   I/O-bank supply is what pulls `P18`/`P19` up through `R2`/`R6`;
//! - a rail on the end-switch loop, which the schematic genuinely does not
//!   have — `IEND_U+` reaches only `IC9`'s anode and the `J16` screw terminal,
//!   so the loop is drawn closed and unpowered. The strap says out loud what a
//!   working machine has to provide.

mod machine_parts;

use std::collections::HashMap;
use std::sync::{Arc, Mutex, Once};
use std::time::{Duration, Instant};

use rstest::rstest;

use embsim_board::netlist::{self, ComponentDecl};
use embsim_board::registry::normalize_part;
use embsim_board::{
    AttachError, Board, Component, ComponentNetIo, EventLog, Finding, Harness, Level, NetState,
    PartRegistry, PinDecl, PinHandle, PinKind, PulseDirection, PulseSegment, PulseTrain, PulseTx,
    Scenario, StreamRole, System, SystemHandle,
};
use embsim_models::isolation::iso67xx;
use embsim_models::isolation::{
    npn_switch, nsi50010, vo2631, Channel, Iso67xx, Iso67xxMonitor, NpnSwitch, NpnSwitchMonitor,
    Nsi50010, Nsi50010Regulator, Vo2631, Vo2631Monitor,
};
use embsim_models::machine::{end_switch, ActuationSense, EndSwitch, EndSwitchActuator};
use machine_parts::{bench_rails, edge_polarity_fet_conducting, edge_registry, ep};

// ============================================================
// Shared fixtures
// ============================================================

const EDGE: &str = "EdgeBoard";
const SETTLE: Duration = Duration::from_secs(5);

/// The engine's timer wheel samples the process-global virtual clock, and
/// `init` re-anchors it — so it runs once per binary.
fn ensure_clock() {
    static CLOCK: Once = Once::new();
    CLOCK.call_once(|| embsim_core::virtual_clock::init(1.0, 1_000_000));
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

fn settled_state(system: &SystemHandle, net: &str, expected: NetState) -> NetState {
    wait_for(|| system.net_state(net) == Some(expected), SETTLE);
    system
        .net_state(net)
        .unwrap_or_else(|| panic!("net {net} exists"))
}

// ============================================================
// The promoted parts
// ============================================================

/// Handles onto every promoted instance, keyed by reference designator.
///
/// A `PartRegistry` constructor is handed a [`ComponentDecl`] and returns a
/// boxed component the system then owns, so the only way to keep a handle on
/// a *particular* instance is to record it as it is built. That is what this
/// is: `IC14`'s monitor, `U6`'s, `Q1`'s, each under its own reference.
#[derive(Clone, Default)]
struct Promoted {
    isolators: Arc<Mutex<HashMap<String, Iso67xxMonitor>>>,
    optos: Arc<Mutex<HashMap<String, Vo2631Monitor>>>,
    regulators: Arc<Mutex<HashMap<String, Nsi50010Regulator>>>,
    switches: Arc<Mutex<HashMap<String, NpnSwitchMonitor>>>,
}

impl Promoted {
    fn isolator(&self, reference: &str) -> Iso67xxMonitor {
        self.isolators
            .lock()
            .unwrap()
            .get(reference)
            .cloned()
            .unwrap_or_else(|| panic!("{reference} was promoted"))
    }

    fn opto(&self, reference: &str) -> Vo2631Monitor {
        self.optos
            .lock()
            .unwrap()
            .get(reference)
            .cloned()
            .unwrap_or_else(|| panic!("{reference} was promoted"))
    }

    fn regulator(&self, reference: &str) -> Nsi50010Regulator {
        self.regulators
            .lock()
            .unwrap()
            .get(reference)
            .cloned()
            .unwrap_or_else(|| panic!("{reference} was promoted"))
    }

    fn switch(&self, reference: &str) -> NpnSwitchMonitor {
        self.switches
            .lock()
            .unwrap()
            .get(reference)
            .cloned()
            .unwrap_or_else(|| panic!("{reference} was promoted"))
    }
}

/// Which of `IC14`'s channels carries the step clock: `INA` (pin 3, on `P8`)
/// to `OUTA` (pin 14).
const STEP_CHANNEL: Channel = Channel::A;

/// [`machine_parts::edge_registry`] with the five blocking parts promoted from
/// topology-only stubs to real models.
///
/// **This function is the promotion instruction.** A consumer replaces its own
/// `register_stub` lines with these four `register` calls and nothing else
/// changes: the pin facades come from the models' own datasheet tables, so the
/// build validates against the same netlist it always did.
fn promoted_registry(promoted: &Promoted) -> PartRegistry {
    let mut registry = edge_registry();

    // Every ISO67xx on the board, configured straight from its part name —
    // `ISO6740FDWR` picks up its fail-safe-low default without anyone
    // re-deriving it from the suffix.
    for part in ["ISO6742DWR", "ISO6741DWR", "ISO6740FDWR", "ISO6721BDR"] {
        let promoted = promoted.clone();
        registry.register(part, move |decl: &ComponentDecl| {
            let name = normalize_part(decl);
            let mut config = iso67xx::Config::from_part_name(&name)
                .unwrap_or_else(|| panic!("{name} is an ISO67xx"));
            // The servo isolator's STEP channel carries a rate, not edges.
            if decl.reference == "IC14" {
                config = config.with_pulse_channel(STEP_CHANNEL);
            }
            let isolator = Iso67xx::new(config).expect("a valid isolator configuration");
            promoted
                .isolators
                .lock()
                .unwrap()
                .insert(decl.reference.clone(), isolator.monitor());
            Box::new(isolator)
        });
    }

    {
        let promoted = promoted.clone();
        registry.register("VO2631", move |decl: &ComponentDecl| {
            let opto = Vo2631::new(vo2631::Config::new()).expect("a valid optocoupler");
            promoted
                .optos
                .lock()
                .unwrap()
                .insert(decl.reference.clone(), opto.monitor());
            Box::new(opto)
        });
    }
    {
        let promoted = promoted.clone();
        registry.register("NSI50010YT1G_1", move |decl: &ComponentDecl| {
            let ccr = Nsi50010::new(nsi50010::Config::new()).expect("a valid regulator");
            promoted
                .regulators
                .lock()
                .unwrap()
                .insert(decl.reference.clone(), ccr.regulator());
            Box::new(ccr)
        });
    }
    {
        let promoted = promoted.clone();
        registry.register("2N3904", move |decl: &ComponentDecl| {
            // `Q1`'s netlist pins are 1 `E`, 2 `B`, 3 `C` — the package order.
            let switch = NpnSwitch::new(npn_switch::Config::new()).expect("a valid switch");
            promoted
                .switches
                .lock()
                .unwrap()
                .insert(decl.reference.clone(), switch.monitor());
            Box::new(switch)
        });
    }
    registry
}

fn promoted_board(promoted: &Promoted) -> Board {
    let parsed = netlist::parse(include_str!("fixtures/mad_edge.net"))
        .expect("the EdgeBoard fixture parses");
    Board::from_netlist(parsed, &promoted_registry(promoted)).expect("the EdgeBoard builds")
}

// ============================================================
// The MCU side: a fake P2 on the module socket's fingers
// ============================================================

/// The edge fingers the P2 module presents the three servo outputs on
/// (`fixtures/mad_edge.net`: `P8` is `J3.32`, `P7` is `J3.33`, `P6` is
/// `J3.34`).
const STEP_FINGER: &str = "32";
const DIR_FINGER: &str = "33";
const ENA_FINGER: &str = "34";

/// A stand-in for the P2's driven pins: three push-pull outputs, with `STEP`
/// additionally a [`StreamRole::PulseSource`] so a rate-carried train can
/// reach the isolator.
///
/// Deliberately not an [`embsim_board::McuComponent`]: this binary needs no
/// firmware and no peripheral banks, and a component that claimed the
/// process-default banks would have to own its own suite lock (`TESTING.md`
/// rule 5).
struct FakePins {
    pins: Vec<PinDecl>,
    handles: Arc<Mutex<HashMap<&'static str, PinHandle>>>,
    step_tx: Arc<Mutex<Option<PulseTx>>>,
}

impl FakePins {
    fn new() -> Self {
        Self {
            pins: vec![
                PinDecl {
                    number: "STEP",
                    name: None,
                    kind: PinKind::DigitalOut,
                    stream: Some(StreamRole::PulseSource),
                    drive_impedance: None,
                },
                out("DIR"),
                out("ENA"),
            ],
            handles: Arc::new(Mutex::new(HashMap::new())),
            step_tx: Arc::new(Mutex::new(None)),
        }
    }
}

fn out(number: &'static str) -> PinDecl {
    PinDecl {
        number,
        name: None,
        kind: PinKind::DigitalOut,
        stream: None,
        drive_impedance: None,
    }
}

impl Component for FakePins {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let mut handles = self.handles.lock().unwrap();
        for number in ["STEP", "DIR", "ENA"] {
            handles.insert(number, io.pin(number)?);
        }
        *self.step_tx.lock().unwrap() = Some(io.pulse_tx("STEP")?);
        Ok(())
    }
}

/// A stand-in for whatever consumes the step clock on the isolated side. It is
/// harnessed onto the RS-422 driver's own `1A` input pin, so it watches the
/// exact net the schematic feeds the stepper driver from.
struct FakeStepSink {
    trains: Arc<Mutex<Vec<PulseTrain>>>,
}

const STEP_SINK_PINS: [PinDecl; 1] = [PinDecl {
    number: "IN",
    name: None,
    kind: PinKind::DigitalIn,
    stream: Some(StreamRole::PulseSink),
    drive_impedance: None,
}];

impl Component for FakeStepSink {
    fn pins(&self) -> &[PinDecl] {
        &STEP_SINK_PINS
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let trains = Arc::clone(&self.trains);
        io.on_pulse("IN", move |train| trains.lock().unwrap().push(train))
    }
}

// ============================================================
// Rig
// ============================================================

/// Everything a test drives or reads.
struct Rig {
    system: SystemHandle,
    promoted: Promoted,
    handles: Arc<Mutex<HashMap<&'static str, PinHandle>>>,
    step_tx: Arc<Mutex<Option<PulseTx>>>,
    trains: Arc<Mutex<Vec<PulseTrain>>>,
    end_switch: EndSwitchActuator,
}

impl Rig {
    fn pin(&self, number: &str) -> PinHandle {
        self.handles
            .lock()
            .unwrap()
            .get(number)
            .cloned()
            .unwrap_or_else(|| panic!("{number} shared at attach"))
    }

    fn drive(&self, number: &str, volts: f64) {
        self.pin(number)
            .set_drive(Some(embsim_board::TheveninDrive {
                volts,
                impedance: 25.0,
            }));
    }

    fn publish_step(&self, train: PulseTrain) {
        self.step_tx
            .lock()
            .unwrap()
            .as_ref()
            .expect("the pulse source attached")
            .set_train(train);
    }
}

/// Bench straps the board needs beyond [`bench_rails`].
///
/// `VIO_16_23` is the P2's own I/O-bank supply, which arrives over the module
/// socket (`J3.58`) and is what pulls `P18`/`P19` up through `R2`/`R6`.
/// The end-switch loop rail is discussed in the module docs.
fn extra_rails() -> Harness {
    Harness::new()
        .power(ep("BENCH.VIO"), ep(&format!("{EDGE}.J3.58")), 3.3)
        // The end-switch current loop: 24 V onto `IEND_U+` (the CCR's anode)
        // and the switch's common terminal at the loop return.
        .power(ep("BENCH.ENDLOOP"), ep(&format!("{EDGE}.J16.2")), 24.0)
        .power(ep("BENCH.ENDRETURN"), ep("END_U.COM"), 0.0)
}

/// Wire the fake MCU pins to the module socket's fingers, the step sink to the
/// RS-422 driver's input, and the end switch to the `J16` screw terminal.
fn rig_harness() -> Harness {
    Harness::new()
        .connect(ep("MCU.STEP"), ep(&format!("{EDGE}.J3.{STEP_FINGER}")))
        .connect(ep("MCU.DIR"), ep(&format!("{EDGE}.J3.{DIR_FINGER}")))
        .connect(ep("MCU.ENA"), ep(&format!("{EDGE}.J3.{ENA_FINGER}")))
        // `U24.1` is the AM26LS31's `1A`: the isolated-side net the step
        // signal has to reach.
        .connect(ep("STEPSINK.IN"), ep(&format!("{EDGE}.U24.1")))
        .connect(ep(&format!("{EDGE}.J16.1")), ep("END_U.NO"))
}

/// Build and start the promoted board.
///
/// `servo_domain` powers `SC_5V` — the isolated servo rail `IC14`'s side 2 and
/// `IC16`'s side 1 run from. Dropping it is how a test asks "what does an
/// isolator with one side dead do?".
fn start(servo_domain: bool, event_log: bool, sources: &[(&str, f64)]) -> Rig {
    ensure_clock();
    let promoted = Promoted::default();
    let board = promoted_board(&promoted);

    let mcu = FakePins::new();
    let handles = Arc::clone(&mcu.handles);
    let step_tx = Arc::clone(&mcu.step_tx);
    let trains: Arc<Mutex<Vec<PulseTrain>>> = Arc::new(Mutex::new(Vec::new()));
    let sink = FakeStepSink {
        trains: Arc::clone(&trains),
    };

    let switch = EndSwitch::new(end_switch::Config::new(100.0, ActuationSense::Increasing))
        .expect("a valid end switch");
    let end_switch = switch.actuator();

    let mut scenario = edge_polarity_fet_conducting(Scenario::default(), EDGE);
    scenario = machine_parts::encoder_jumpers_closed(scenario, EDGE);
    for (net, volts) in sources {
        scenario = scenario.net_stuck(net, *volts);
    }

    let mut rails = bench_rails(EDGE);
    if !servo_domain {
        // `bench_rails` powers `SC_5V` through `J21.1`; rebuild it without
        // that strap rather than trying to take one back out.
        rails = Harness::new()
            .power(ep("BENCH.12V"), ep(&format!("{EDGE}.J2.1")), 12.0)
            .power(ep("BENCH.GND"), ep(&format!("{EDGE}.J2.2")), 0.0)
            .power(ep("BENCH.3V3"), ep(&format!("{EDGE}.J19.1")), 3.3)
            .power(ep("BENCH.5V"), ep(&format!("{EDGE}.J22.1")), 5.0)
            .power(ep("BENCH.SERVOGND"), ep(&format!("{EDGE}.J21.8")), 0.0);
    }

    let mut system = System::new()
        .board(EDGE, board)
        .component("MCU", Box::new(mcu))
        .component("STEPSINK", Box::new(sink))
        .component("END_U", Box::new(switch))
        .harness(rails)
        .harness(extra_rails())
        .harness(rig_harness())
        .scenario(scenario);
    if event_log {
        system = system.event_log();
    }
    let system = system.start().expect("the promoted EdgeBoard starts");

    Rig {
        system,
        promoted,
        handles,
        step_tx,
        trains,
        end_switch,
    }
}

// ============================================================
// STEP and DIR: a level crosses the servo isolator
// ============================================================

/// The claim the whole change exists to make: a level driven on the P2's `P8`
/// finger comes out of `IC14`'s `OUTA` on the isolated side, on the real
/// netlist, through the real part.
///
/// The output sits in the 5 V servo domain while the input is 3.3 V logic, so
/// this is also the family's level translation: the isolator drives *its own
/// side's* rail, not the one it was fed.
#[rstest]
#[case::high(3.3, Level::High)]
#[case::low(0.0, Level::Low)]
fn a_level_crosses_the_step_isolator_end_to_end(#[case] volts: f64, #[case] expect: Level) {
    let rig = start(true, false, &[]);
    let monitor = rig.promoted.isolator("IC14");
    // Both rails have to have reached the part before its output means
    // anything: an isolator whose input side is still dark drives its
    // *default* state, which for this non-F part is also high.
    assert!(
        wait_for(|| monitor.is_passing(STEP_CHANNEL), SETTLE),
        "both of IC14's supplies must come up"
    );
    rig.drive("STEP", volts);

    let net = format!("{EDGE}.Net-(IC14-OUTA)");
    assert_eq!(
        settled_state(&rig.system, &net, NetState::Driven(expect)),
        NetState::Driven(expect),
        "P8 must reach the stepper driver's input across IC14"
    );
    rig.system.shutdown();
}

/// `DIR` is a second, independent channel of the same part — proof the model
/// is per-channel and not one hard-wired path, and that driving one channel
/// leaves the others alone.
#[rstest]
fn the_direction_channel_is_independent_of_the_step_channel() {
    let rig = start(true, false, &[]);
    rig.drive("STEP", 3.3);
    rig.drive("DIR", 0.0);

    assert_eq!(
        settled_state(
            &rig.system,
            &format!("{EDGE}.Net-(IC14-OUTA)"),
            NetState::Driven(Level::High)
        ),
        NetState::Driven(Level::High)
    );
    assert_eq!(
        settled_state(
            &rig.system,
            &format!("{EDGE}.Net-(IC14-OUTB)"),
            NetState::Driven(Level::Low)
        ),
        NetState::Driven(Level::Low)
    );
    rig.system.shutdown();
}

/// One side powered is no isolator at all. With `SC_5V` dark, `IC14`'s output
/// buffer has no supply, so `OUTA` is released and the net floats — and the
/// engine says why.
#[rstest]
fn an_unpowered_isolated_side_stops_the_step_path() {
    let rig = start(false, false, &[]);
    rig.drive("STEP", 3.3);

    let net = format!("{EDGE}.Net-(IC14-OUTA)");
    assert_eq!(
        settled_state(&rig.system, &net, NetState::Floating),
        NetState::Floating,
        "an isolator with only one side powered must pass nothing"
    );
    assert!(!rig.promoted.isolator("IC14").is_passing(STEP_CHANNEL));
    assert!(
        rig.system.findings().iter().any(|f| matches!(
            f,
            Finding::PowerNetUnsourced { net } if net == "EdgeBoard./MaD_Edge_Sheet3/SC_5V"
        )),
        "and the reason must be reported"
    );

    // The enable path dies with it, all the way through the transistor: a
    // released `OUTC` leaves the base network unsourced, so `Q1` is off and
    // its collector is an open circuit rather than a plausible enable.
    rig.drive("ENA", 3.3);
    assert_eq!(
        settled_state(
            &rig.system,
            &format!("{EDGE}.Net-(Q1-B)"),
            NetState::Floating
        ),
        NetState::Floating
    );
    assert!(!rig.promoted.switch("Q1").is_on());
    assert_eq!(
        settled_state(
            &rig.system,
            &format!("{EDGE}.Net-(JP1-B)"),
            NetState::Floating
        ),
        NetState::Floating,
        "an off transistor releases its collector"
    );
    rig.system.shutdown();
}

// ============================================================
// ENA: a level crosses the isolator and then the transistor
// ============================================================

/// The enable path is the one that crosses **two** modeled parts: `P6` into
/// `IC14`'s channel C, out of `OUTC` through `R24` into `Q1`'s base, and the
/// transistor's collector sinks the enable net.
///
/// `Q1`'s emitter sits on the isolated servo ground, so this also exercises
/// the switch's "short to the emitter, not to ground" rule against a real
/// netlist.
///
/// # A pinned engine limitation
///
/// Driving `P6` **low** does reach `OUTC`, and this test asserts that — but it
/// does not reach `Q1`'s base. `Net-(Q1-B)` is reached only through `R24`
/// (43 kΩ), and the resolver's projection for a net reached solely through
/// conduction edges takes its *level* from the cluster's **power** source
/// (`engine.rs`, "Reached only through conduction edges"); a cluster fed by a
/// signal driver rather than a rail has none, so the projection defaults to
/// `Pulled(High)` whichever way the driver is pointing. The assertion below
/// pins that: it is the truth about the engine today, not about the
/// transistor. `Q1` turning **off** is asserted where the engine can express
/// it — on a released `OUTC`, in
/// [`an_unpowered_isolated_side_stops_the_step_path`] — and exhaustively in
/// the model's own unit tests.
#[rstest]
fn the_enable_path_crosses_the_isolator_and_the_transistor() {
    let rig = start(true, false, &[]);
    let outc = format!("{EDGE}.Net-(IC14-OUTC)");
    let collector = format!("{EDGE}.Net-(JP1-B)");
    let switch = rig.promoted.switch("Q1");
    assert!(wait_for(
        || rig.promoted.isolator("IC14").is_passing(Channel::C),
        SETTLE
    ));

    // Enable asserted: the isolator drives OUTC high, the base resistor takes
    // it to Q1, and the collector sinks.
    rig.drive("ENA", 3.3);
    assert_eq!(
        settled_state(&rig.system, &outc, NetState::Driven(Level::High)),
        NetState::Driven(Level::High),
        "P6 must reach the base-drive network across IC14"
    );
    assert!(wait_for(|| switch.is_on(), SETTLE), "Q1 must saturate");
    assert_eq!(
        settled_state(&rig.system, &collector, NetState::Driven(Level::Low)),
        NetState::Driven(Level::Low),
        "a saturated Q1 sinks its collector to the isolated ground"
    );

    // Enable released: the isolator follows P6 down...
    rig.drive("ENA", 0.0);
    assert_eq!(
        settled_state(&rig.system, &outc, NetState::Driven(Level::Low)),
        NetState::Driven(Level::Low),
        "and back down again — the channel is a repeater, not a latch"
    );
    // ...but the pinned limitation above means the base network does not
    // carry the low, so Q1 stays on. Asserted so the day the projection
    // learns to carry a driver's level, this test fails and says so.
    assert!(
        matches!(
            rig.system.net_state(&format!("{EDGE}.Net-(Q1-B)")),
            Some(NetState::Pulled(Level::High, _))
        ),
        "pinned: a net reached only through a signal-driven resistor projects \
         high regardless of the driver; got {:?}",
        rig.system.net_state(&format!("{EDGE}.Net-(Q1-B)"))
    );
    rig.system.shutdown();
}

// ============================================================
// The encoder return path, and the fail-safe suffix
// ============================================================

/// The encoder path, end to end on the netlist: a differential on `A±`, the
/// RS-422 receiver's decision, `IC16`, and out on the P2's `P9` finger.
///
/// `edgeboard.rs` stops at `Net-(IC16-INA)` because that was as far as a
/// stubbed isolator let it go.
#[rstest]
#[case::forward(3.3, Level::High)]
#[case::reverse(0.0, Level::High)]
fn the_encoder_reaches_the_p2_across_the_isolator(#[case] a_plus: f64, #[case] expect: Level) {
    let rig = start(true, false, &[("EdgeBoard./MaD_Edge_Sheet3/A+", a_plus)]);
    assert!(
        wait_for(
            || rig.promoted.isolator("IC16").is_passing(Channel::A),
            SETTLE
        ),
        "both of IC16's supplies must come up"
    );
    let receiver = format!("{EDGE}.Net-(IC16-INA)");
    assert_eq!(
        settled_state(&rig.system, &receiver, NetState::Driven(expect)),
        NetState::Driven(expect),
        "the receiver's own output, as edgeboard.rs asserts it"
    );
    assert_eq!(
        settled_state(&rig.system, &format!("{EDGE}.P9"), NetState::Driven(expect)),
        NetState::Driven(expect),
        "and now it reaches P9 across IC16"
    );
    rig.system.shutdown();
}

/// `IC16` is an `ISO6740F` — the **fail-safe** part — and this is what the F
/// suffix is bought for: with the isolated servo domain dark, its four outputs
/// present a defined LOW on `P9`..`P12` rather than floating or idling high.
///
/// A plain `ISO6740` in the same socket would present HIGH, which on this
/// board would look to the firmware like four stuck encoder channels.
#[rstest]
fn the_encoder_isolator_fails_safe_low_when_its_input_side_dies() {
    let rig = start(false, false, &[]);
    for pin in ["P9", "P10", "P11", "P12"] {
        let net = format!("{EDGE}.{pin}");
        assert_eq!(
            settled_state(&rig.system, &net, NetState::Driven(Level::Low)),
            NetState::Driven(Level::Low),
            "{pin} must present the ISO6740F default, not float"
        );
    }
    let monitor = rig.promoted.isolator("IC16");
    assert!(!monitor.is_passing(Channel::A), "nothing is being relayed");
    assert_eq!(monitor.output_level(Channel::A), Some(Level::Low));
    assert_eq!(monitor.config().default_level(), Level::Low);
    rig.system.shutdown();
}

// ============================================================
// The end-switch loop: regulator, LED, detector
// ============================================================

/// The end-switch path, which no engine event could cross before: a closed
/// contact completes the current loop, the constant-current regulator sees
/// overhead, the optocoupler's LED lights, and its open-collector output pulls
/// `P19` down against the board's own 1 kΩ pull-up.
///
/// Open, every one of those is false — and `P19` sits at the pull-up, which is
/// exactly what "pulled to its rail, not floating" means.
#[rstest]
fn a_closed_end_switch_lights_the_optocoupler_and_pulls_p19_down() {
    let rig = start(true, false, &[]);
    let p19 = format!("{EDGE}.P19");
    let ccr = rig.promoted.regulator("IC9");
    let opto = rig.promoted.opto("U6");

    // Open contact: no return path, so the loop carries nothing.
    rig.end_switch.set_position_mm(0.0);
    assert!(
        wait_for(
            || matches!(
                rig.system.net_state(&p19),
                Some(NetState::Pulled(Level::High, _))
            ),
            SETTLE
        ),
        "an unlit optocoupler must leave P19 to the pull-up; got {:?}",
        rig.system.net_state(&p19)
    );
    assert_eq!(ccr.current_ma(), 0.0, "an open loop regulates nothing");
    assert!(!opto.is_lit(vo2631::OptoChannel::Two));
    assert!(!opto.is_sinking(vo2631::OptoChannel::Two));

    // Closed contact: the loop completes.
    rig.end_switch.set_position_mm(150.0);
    assert!(
        wait_for(
            || matches!(
                rig.system.net_state(&p19),
                Some(NetState::Driven(Level::Low))
            ),
            SETTLE
        ),
        "a closed contact must pull P19 down; got {:?}",
        rig.system.net_state(&p19)
    );
    assert!(
        (ccr.current_ma() - nsi50010::DEFAULT_REGULATION_MA).abs() < 1e-9,
        "the regulator holds its regulation current, got {} mA",
        ccr.current_ma()
    );
    assert!(
        opto.forward_ma(vo2631::OptoChannel::Two) >= vo2631::DEFAULT_THRESHOLD_MA,
        "the LED must be lit past ITH, got {} mA",
        opto.forward_ma(vo2631::OptoChannel::Two)
    );
    assert!(opto.is_sinking(vo2631::OptoChannel::Two));

    // And back: the path is not one-way.
    rig.end_switch.set_position_mm(0.0);
    assert!(wait_for(
        || matches!(
            rig.system.net_state(&p19),
            Some(NetState::Pulled(Level::High, _))
        ),
        SETTLE
    ));
    rig.system.shutdown();
}

/// An **unpowered** optocoupler cannot sink however brightly its LED is lit.
///
/// `U6` runs from the board's `+5V`, which the board's own regulator sources,
/// so the way to take it away is a fault rather than a missing strap: a
/// `net_stuck` at 0 V. The rail lands in `Contention` — a bench short against
/// a regulator output, which is exactly what that is — and a contended supply
/// is a down supply, so the detector cannot sink and `P19` stays at its
/// pull-up.
#[rstest]
fn an_unpowered_optocoupler_leaves_p19_at_its_pull_up() {
    let rig = start(true, false, &[("EdgeBoard.+5V", 0.0)]);
    let opto = rig.promoted.opto("U6");
    let p19 = format!("{EDGE}.P19");

    rig.end_switch.set_position_mm(150.0);
    assert!(
        wait_for(|| opto.is_lit(vo2631::OptoChannel::Two), SETTLE),
        "the LED loop is still powered and the contact is closed"
    );
    assert!(!opto.is_powered(), "but the detector is not");
    assert!(!opto.is_sinking(vo2631::OptoChannel::Two));
    assert!(
        wait_for(
            || matches!(
                rig.system.net_state(&p19),
                Some(NetState::Pulled(Level::High, _))
            ),
            SETTLE
        ),
        "P19 must sit at the pull-up, not be held low; got {:?}",
        rig.system.net_state(&p19)
    );
    rig.system.shutdown();
}

// ============================================================
// The budget: a step train crosses without scaling engine traffic
// ============================================================

/// Engine events a whole four-segment step profile is allowed to cost, from
/// the moment the system has gone quiet.
///
/// The alternative — an isolator that re-drove its output pin per STEP edge —
/// is ~8192 events per millimetre at the reference machine's resolution, so a
/// regression would miss this ceiling by orders of magnitude. The measured
/// number is printed by the test; the ceiling is headroom over it for
/// incidental engine bookkeeping, not for per-pulse traffic.
const RELAY_EVENT_CEILING: usize = 32;

/// Virtual time each constant-rate segment runs for.
const SEGMENT_US: u64 = 200_000;

/// Wait until the engine's event log stops growing, so a measurement taken
/// after this covers only what the test then does.
fn settle_log(log: &EventLog) {
    let deadline = Instant::now() + SETTLE;
    let mut last = log.records().len();
    while Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(50));
        let now = log.records().len();
        if now == last {
            return;
        }
        last = now;
    }
}

/// One four-segment profile at `base_hz`, `2 x base_hz`, `base_hz`, stop.
/// Returns `(pulses emitted, engine events, relays)`.
fn run_profile(
    rig: &Rig,
    log: &EventLog,
    monitor: &Iso67xxMonitor,
    base_hz: u32,
    published: &mut u64,
) -> (u64, usize, u64) {
    let before = log.records().len();
    let relays_before = monitor.train_count();
    let received_before = rig.trains.lock().unwrap().len();
    let mut pulses = 0u64;

    for (index, multiplier) in [1u32, 2, 1, 0].into_iter().enumerate() {
        let freq_hz = base_hz * multiplier;
        rig.publish_step(PulseTrain {
            pulses: PulseSegment {
                emitted: *published,
                freq_hz,
                total: None,
                since_us: embsim_core::virtual_clock::virtual_us(),
            },
            direction: PulseDirection::Forward,
        });
        assert!(
            wait_for(
                || rig.trains.lock().unwrap().len() > received_before + index,
                SETTLE
            ),
            "segment {index} at {base_hz} Hz must reach the isolated side"
        );
        if freq_hz > 0 {
            embsim_core::virtual_clock::wait_virtual_us(SEGMENT_US);
            let emitted = u64::from(freq_hz) * SEGMENT_US / 1_000_000;
            pulses += emitted;
            *published += emitted;
        }
    }
    settle_log(log);
    (
        pulses,
        log.records().len() - before,
        monitor.train_count() - relays_before,
    )
}

/// A rate-carried step train crosses the barrier, arrives at the isolated
/// side's stepper-driver input **verbatim**, and costs a number of engine
/// events that does not depend on the step rate.
///
/// The independence is asserted directly rather than inferred: the same
/// profile is run twice, a hundredfold apart in rate, and the two engine-event
/// counts must be **equal**.
#[rstest]
fn a_step_train_crosses_the_barrier_at_a_bounded_engine_cost() {
    let rig = start(true, true, &[]);
    let log: EventLog = rig.system.event_log();
    let monitor = rig.promoted.isolator("IC14");

    // Settle the level path first, so the measurement covers the train only.
    rig.drive("STEP", 3.3);
    assert!(wait_for(
        || rig.system.net_state(&format!("{EDGE}.Net-(IC14-OUTA)"))
            == Some(NetState::Driven(Level::High)),
        SETTLE
    ));
    // Nothing has published a train yet, so nothing has been relayed: a pulse
    // route delivers at registration only when its source already has a
    // segment, and an isolator relays only what it has received.
    assert!(rig.trains.lock().unwrap().is_empty());
    settle_log(&log);

    let mut published = 0u64;
    let slow = run_profile(&rig, &log, &monitor, 8_192, &mut published);
    let fast = run_profile(&rig, &log, &monitor, 819_200, &mut published);

    // The isolated side saw every segment, unaltered.
    let received = rig.trains.lock().unwrap().clone();
    let rates: Vec<u32> = received.iter().map(|t| t.pulses.freq_hz).collect();
    assert_eq!(
        rates,
        vec![8_192, 16_384, 8_192, 0, 819_200, 1_638_400, 819_200, 0],
        "every rate change crossed the barrier, and only rate changes did"
    );
    assert_eq!(
        received.last().expect("segments arrived").pulses.emitted,
        published,
        "the relayed count is the source's own, verbatim"
    );
    assert_eq!(
        (slow.2, fast.2),
        (4, 4),
        "one relay per rate change, no more"
    );
    assert!(
        fast.0 >= 100_000 && fast.0 >= slow.0 * 100,
        "the fast profile must deliver a hundredfold more pulses: {} vs {}",
        fast.0,
        slow.0
    );
    assert_eq!(
        slow.1, fast.1,
        "engine cost must be identical at a hundredfold higher rate: {} vs {} events",
        slow.1, fast.1
    );
    assert!(
        fast.1 <= RELAY_EVENT_CEILING,
        "engine events must not scale with the step rate: {} events for {} pulses \
         (ceiling {RELAY_EVENT_CEILING})",
        fast.1,
        fast.0
    );
    println!(
        "[budget] {} pulses crossed IC14 in {} engine events; {} pulses in {} events \
         (a hundredfold rate change, identical cost). One engine event per STEP edge \
         would have been at least {}",
        slow.0, slow.1, fast.0, fast.1, fast.0
    );
    rig.system.shutdown();
}

/// A level channel is a repeater, so its cost is one drive per transition —
/// not one per channel, and not one per unrelated delivery.
#[rstest]
fn a_level_transition_costs_one_drive_on_one_channel() {
    let rig = start(true, false, &[]);
    let monitor = rig.promoted.isolator("IC14");
    rig.drive("STEP", 0.0);
    rig.drive("DIR", 0.0);
    rig.drive("ENA", 0.0);
    assert!(wait_for(
        || rig.system.net_state(&format!("{EDGE}.Net-(IC14-OUTA)"))
            == Some(NetState::Driven(Level::Low)),
        SETTLE
    ));
    let settled = monitor.drive_count();

    rig.drive("STEP", 3.3);
    assert!(wait_for(
        || rig.system.net_state(&format!("{EDGE}.Net-(IC14-OUTA)"))
            == Some(NetState::Driven(Level::High)),
        SETTLE
    ));
    assert_eq!(
        monitor.drive_count(),
        settled + 1,
        "one transition on one channel is one drive across the whole part"
    );

    // Re-driving the same level, ten times over, costs nothing at all.
    for _ in 0..10 {
        rig.drive("STEP", 3.3);
    }
    std::thread::sleep(Duration::from_millis(50));
    assert_eq!(monitor.drive_count(), settled + 1);
    println!(
        "[budget] {} drives for a settled four-channel isolator plus one transition",
        monitor.drive_count()
    );
    rig.system.shutdown();
}
