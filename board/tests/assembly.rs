//! [`Assembly`]: several components as one bench component behind one set
//! of pins (`PROJECTS.md` §10, "A plant: an `Assembly`"; `NODES.md` §16).
//!
//! The axis is the plant MaD's machine is built around: embsim's
//! step/direction drive and quadrature encoder, the drive's shaft turning
//! the encoder — a link that is the assembly's own code, not a net — with
//! the drive's inputs measured against a return the assembly declares,
//! `DRIVE_GND`, and the encoder's outputs against `ENC_GND`. A controller
//! on the bench drives `STEP` with a rate-carried train
//! ([`Drive::Periodic`]) and holds `DIR` and `ENA`; the encoder's outputs
//! go to a board, a 74LVC2G04 dual inverter the catalog places by its
//! number, whose outputs a probe records as the engine reports them. And
//! members that only keep time, one of them an assembly itself, show whose
//! wakes are whose.
//!
//! Stepped (`TESTING.md` rule 9): a suite lock, the clock re-anchored
//! stepped, the system started with time held, the case's thread a
//! registered actor, every read after a virtual settle, no
//! `QuiescenceTimeout` at the end.

use std::sync::{Arc, Mutex, MutexGuard};

use embsim_board::{
    jesd8c01_lvcmos_thresholds, netlist, Assembly, AttachError, Board, Component, ComponentNetIo,
    DeadBand, Drive, EndpointRef, Finding, Harness, Level, NetState, PeriodicSchedule, PinDecl,
    PinHandle, Scenario, System, SystemHandle, TheveninDrive, Volts,
};
use embsim_boards::catalog::StandardCatalog;
use embsim_core::virtual_clock::{self, Actor, ClockMode};
use embsim_models::machine::quadrature_encoder::{self, EncoderInput};
use embsim_models::machine::stepper_motor::{self, DEFAULT_OBSERVE_INTERVAL_US, DEFAULT_TAU_S};
use embsim_models::machine::{MotorShaft, QuadratureEncoder, StepperMotor};
use rstest::rstest;
use vibes_behaviour::{behaviour, expect, Test};

static SUITE_LOCK: Mutex<()> = Mutex::new(());

fn suite_lock() -> MutexGuard<'static, ()> {
    SUITE_LOCK.lock().unwrap_or_else(|poisoned| {
        SUITE_LOCK.clear_poison();
        poisoned.into_inner()
    })
}

fn ep(endpoint: &str) -> EndpointRef {
    EndpointRef::parse(endpoint).expect("endpoint parses")
}

/// Assert the engine never stopped waiting for the case's thread, and shut
/// the system down.
fn finish(system: SystemHandle) {
    let stalled: Vec<Finding> = system
        .findings()
        .into_iter()
        .filter(|finding| matches!(finding, Finding::QuiescenceTimeout { .. }))
        .collect();
    assert!(
        stalled.is_empty(),
        "the engine stopped waiting: {stalled:?}"
    );
    system.shutdown();
}

// ============================================================
// The axis
// ============================================================

/// Steps (and encoder counts) per millimetre: MaD's machine's, 4
/// microsteps × 2048 steps a revolution (`stepper_motor`'s provenance).
/// The drive and the encoder share it, so one step is one count.
const STEPS_PER_MM: f64 = 8_192.0;

/// The train's rate. At most one count an observation: the carriage never
/// outruns the drive's command, and 500 steps a second is half a step a
/// millisecond, so every count reaches the board at an instant of its own.
const TRAIN_HZ: u32 = 500;
const _: () = assert!((TRAIN_HZ as u64) * DEFAULT_OBSERVE_INTERVAL_US < 1_000_000);

/// The steps the first train carries forward, and the second back: each
/// leaves the encoder at a different place in its four-state cycle.
const FORWARD_STEPS: u64 = 102;
const BACK_STEPS: u64 = 61;

/// The virtual time a case waits after a train starts before it reads: the
/// longest train, then twenty of the drive's time constants for the
/// carriage to come to rest, which leaves less than a millionth of a step
/// of travel, then a quarter observation more, so the deadline falls
/// between two of the drive's observations.
const REST_NS: u64 = FORWARD_STEPS * 1_000_000_000 / TRAIN_HZ as u64
    + (20.0 * DEFAULT_TAU_S * 1e9) as u64
    + DEFAULT_OBSERVE_INTERVAL_US * 250;

/// The first settle: the attach cascade at rest, between two observations.
const START_NS: u64 = DEFAULT_OBSERVE_INTERVAL_US * 250;

/// The controller's ports: the push-pull default's 25 Ω, at 3.3 V logic.
const HIGH: TheveninDrive = TheveninDrive {
    volts: 3.3,
    impedance: 25.0,
};
const LOW: TheveninDrive = TheveninDrive {
    volts: 0.0,
    impedance: 25.0,
};

/// The axis's two members, the drive's shaft turning the encoder: the link
/// is code, run inside the drive's observation. The test's drive carries
/// no load, so the carriage comes to rest where the steps put it.
fn members() -> (StepperMotor, QuadratureEncoder, MotorShaft, EncoderInput) {
    let drive = StepperMotor::new(stepper_motor::Config {
        load_loss: 0.0,
        ..stepper_motor::Config::new(STEPS_PER_MM)
    })
    .expect("the drive's configuration is valid");
    let encoder = QuadratureEncoder::new(quadrature_encoder::Config::new(STEPS_PER_MM))
        .expect("the encoder's configuration is valid");
    let (shaft, input) = (drive.shaft(), encoder.input());
    {
        let input = input.clone();
        shaft.on_position_change(move |mm| input.set_position_mm(mm));
    }
    (drive, encoder, shaft, input)
}

/// The axis: the two members as one assembly, each member's pins measured
/// against a return of the assembly's own.
fn axis(drive: StepperMotor, encoder: QuadratureEncoder) -> Assembly {
    Assembly::new()
        .member(
            "DRIVE",
            Box::new(drive),
            &[("STEP", "STEP"), ("DIR", "DIR"), ("ENA", "ENA")],
        )
        .and_then(|axis| {
            axis.member(
                "ENCODER",
                Box::new(encoder),
                &[("A", "ENC_A"), ("B", "ENC_B")],
            )
        })
        .and_then(|axis| axis.reference("DRIVE_GND", &["STEP", "DIR", "ENA"]))
        .and_then(|axis| axis.reference("ENC_GND", &["ENC_A", "ENC_B"]))
        .expect("the axis assembles")
}

// ============================================================
// The bench around it
// ============================================================

type Handles = Arc<Mutex<Vec<PinHandle>>>;

/// The controller: `STEP` resting low, `DIR` and `ENA` high, each a
/// push-pull port the case's thread drives through.
struct Controller {
    pins: [PinDecl; 3],
    handles: Handles,
}

impl Controller {
    fn new() -> Self {
        Self {
            pins: [
                PinDecl::digital_out("STEP").with_idle(Some(LOW)),
                PinDecl::digital_out("DIR").with_idle(Some(HIGH)),
                PinDecl::digital_out("ENA").with_idle(Some(HIGH)),
            ],
            handles: Handles::default(),
        }
    }
}

impl Component for Controller {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let mut handles = self.handles.lock().unwrap();
        for pin in ["STEP", "DIR", "ENA"] {
            handles.push(io.pin(pin)?);
        }
        Ok(())
    }
}

/// What the probe saw: each delivery of the engine's report of `NA` and
/// `NB`, in the engine's order.
type Reports = Arc<Mutex<Vec<(&'static str, NetState)>>>;

/// The probe on the inverters' outputs: an instrument recording the
/// engine's report of each net it is wired to.
struct Probe {
    pins: [PinDecl; 2],
    reports: Reports,
}

impl Component for Probe {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        for pin in ["NA", "NB"] {
            let reports = Arc::clone(&self.reports);
            io.on_net_report(pin, move |state| reports.lock().unwrap().push((pin, state)))?;
        }
        Ok(())
    }
}

/// The board: a 74LVC2G04 in its SOT363 package (`74LVC2G04GW,125`, the
/// number the catalog places it by), each inverter's input on the header
/// `J1` beside its output, its supply and its ground.
///
/// `J1`: 1 `ENC_A`, 2 `ENC_B`, 3 `+3V3`, 4 `GND`, 5 `NA`, 6 `NB`. `U1`
/// (NXP 74LVC2G04 Table 3): 1 `1A`, 2 `GND`, 3 `2A`, 4 `2Y`, 5 `V_CC`,
/// 6 `1Y`.
const READER: &str = r#"(export (version "E")
  (components
    (comp (ref "J1") (value "Conn_01x06")
      (libsource (lib "Connector") (part "Conn_01x06")))
    (comp (ref "U1") (value "74LVC2G04GW,125")
      (libsource (lib "Logic") (part "74LVC2G04GW,125"))))
  (nets
    (net (code "1") (name "ENC_A") (node (ref "J1") (pin "1")) (node (ref "U1") (pin "1")))
    (net (code "2") (name "ENC_B") (node (ref "J1") (pin "2")) (node (ref "U1") (pin "3")))
    (net (code "3") (name "+3V3") (node (ref "J1") (pin "3")) (node (ref "U1") (pin "5")))
    (net (code "4") (name "GND") (node (ref "J1") (pin "4")) (node (ref "U1") (pin "2")))
    (net (code "5") (name "NA") (node (ref "J1") (pin "5")) (node (ref "U1") (pin "6")))
    (net (code "6") (name "NB") (node (ref "J1") (pin "6")) (node (ref "U1") (pin "4")))))"#;

/// How the axis sits on the bench.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Shape {
    /// One assembly, `AXIS`, its drive's return held at these volts (0 V:
    /// on the board's ground, with the encoder's).
    Assembled { return_volts: Volts },
    /// The same two members and the same link as two bench components,
    /// `DRIVE` and `ENCODER`, measured against nothing.
    Separate,
}

/// The running bench, and what the case reads.
struct Rig {
    system: SystemHandle,
    actor: Actor,
    controller: Handles,
    reports: Reports,
    shaft: MotorShaft,
    encoder: EncoderInput,
}

impl Rig {
    /// The axis on the bench in `shape`, its engine recording its events,
    /// started with time held and released with the case's thread an
    /// actor, then settled.
    fn start(shape: Shape) -> Self {
        virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);
        let board = Board::from_netlist(
            netlist::parse(READER).expect("the reader's netlist parses"),
            &StandardCatalog::base_registry(),
        )
        .expect("the catalog places the inverter by its number");
        let (drive, encoder, shaft, input) = members();
        let controller = Controller::new();
        let handles = Arc::clone(&controller.handles);
        let reports = Reports::default();
        let digital = jesd8c01_lvcmos_thresholds(DeadBand::Unknown);
        let probe = Probe {
            pins: [
                PinDecl::digital_in("NA", digital),
                PinDecl::digital_in("NB", digital),
            ],
            reports: Arc::clone(&reports),
        };
        let harness = Harness::new()
            .power(ep("BENCH.VCC"), ep("RX.J1.3"), 3.3)
            .connect(ep("RX.J1.5"), ep("PROBE.NA"))
            .connect(ep("RX.J1.6"), ep("PROBE.NB"));
        let system = System::new()
            .board("RX", board)
            .component("CTRL", Box::new(controller));
        let (system, harness) = match shape {
            Shape::Assembled { return_volts } => {
                let harness = harness
                    .connect(ep("CTRL.STEP"), ep("AXIS.STEP"))
                    .connect(ep("CTRL.DIR"), ep("AXIS.DIR"))
                    .connect(ep("CTRL.ENA"), ep("AXIS.ENA"))
                    .connect(ep("AXIS.ENC_A"), ep("RX.J1.1"))
                    .connect(ep("AXIS.ENC_B"), ep("RX.J1.2"))
                    .connect(ep("AXIS.ENC_GND"), ep("RX.J1.4"));
                let harness = if return_volts == 0.0 {
                    harness.connect(ep("AXIS.DRIVE_GND"), ep("RX.J1.4"))
                } else {
                    harness.power(ep("BENCH.RETURN"), ep("AXIS.DRIVE_GND"), return_volts)
                };
                (
                    system.component("AXIS", Box::new(axis(drive, encoder))),
                    harness,
                )
            }
            Shape::Separate => (
                system
                    .component("DRIVE", Box::new(drive))
                    .component("ENCODER", Box::new(encoder)),
                harness
                    .connect(ep("CTRL.STEP"), ep("DRIVE.STEP"))
                    .connect(ep("CTRL.DIR"), ep("DRIVE.DIR"))
                    .connect(ep("CTRL.ENA"), ep("DRIVE.ENA"))
                    .connect(ep("ENCODER.A"), ep("RX.J1.1"))
                    .connect(ep("ENCODER.B"), ep("RX.J1.2")),
            ),
        };
        let system = system
            .component("PROBE", Box::new(probe))
            .harness(harness)
            .scenario(Scenario::default().net_stuck("RX.GND", 0.0))
            .event_log()
            .hold_time()
            .start()
            .expect("the bench starts");
        let actor = virtual_clock::register_actor("assembly-axis-case");
        system.release_time();
        virtual_clock::wait_virtual_ns(START_NS);
        Self {
            system,
            actor,
            controller: handles,
            reports,
            shaft,
            encoder: input,
        }
    }

    /// The engine's events so far.
    fn events(&self) -> usize {
        self.system.event_log().len()
    }

    /// Hand the clock back and shut the bench down (see [`finish`]).
    fn finish(self) {
        drop(self.actor);
        finish(self.system);
    }

    fn pin(&self, index: usize) -> PinHandle {
        self.controller.lock().unwrap()[index].clone()
    }

    /// Drive `STEP` with a train of `steps` at [`TRAIN_HZ`] from now, and
    /// wait until the carriage is at rest.
    fn train(&self, steps: u64) {
        self.pin(0).drive(Drive::Periodic {
            hi: HIGH,
            lo: LOW,
            segment: PeriodicSchedule {
                emitted: 0,
                freq_hz: TRAIN_HZ,
                total: Some(steps),
                since_ns: virtual_clock::virtual_ns(),
            },
        });
        virtual_clock::wait_virtual_ns(REST_NS);
    }

    /// The level each inverter output now drives, as the engine reports it.
    fn outputs(&self) -> (NetState, NetState) {
        let state = |net: &str| {
            self.system
                .net_state(net)
                .unwrap_or_else(|| panic!("{net} is a net"))
        };
        (state("RX.NA"), state("RX.NB"))
    }

    /// Changes of driven level on the inverter outputs the probe has seen.
    fn transitions(&self) -> usize {
        let reports = self.reports.lock().unwrap();
        let mut last: [Option<Level>; 2] = [None, None];
        let mut changes = 0;
        for (pin, state) in reports.iter() {
            let NetState::Driven(level) = *state else {
                continue;
            };
            let slot = &mut last[usize::from(*pin == "NB")];
            if slot.is_some_and(|previous| previous != level) {
                changes += 1;
            }
            *slot = Some(level);
        }
        changes
    }
}

/// The inverters' outputs for an encoder count: the inverse of its
/// quadrature state (A leads B: 0 (L, L), 1 (H, L), 2 (H, H), 3 (L, H)).
fn inverted(count: u64) -> (NetState, NetState) {
    let (a, b) = match count % 4 {
        0 => (Level::Low, Level::Low),
        1 => (Level::High, Level::Low),
        2 => (Level::High, Level::High),
        _ => (Level::Low, Level::High),
    };
    let not = |level| match level {
        Level::High => NetState::Driven(Level::Low),
        Level::Low => NetState::Driven(Level::High),
    };
    (not(a), not(b))
}

#[rstest]
fn an_assembled_axis_turns_a_step_train_into_encoder_edges_on_the_board() {
    behaviour!(Test {
        id: "assembly.axis-train-to-board",
        covers: Some("board/src/assembly.rs#Assembly"),
        given: "a drive turning an encoder as one assembly, stepped 102 forward and then 61 \
                back by trains carried as a rate, its encoder's outputs read by an inverter \
                on a board",
    });
    expect!(
        "count-is-the-steps",
        "at rest the encoder's count is the steps the drive folded out of the trains, 102 \
         and then 41",
        "the drive's carriage is a closed form read at each observation, so where it comes \
         to rest does not depend on how often it is observed"
    );
    expect!(
        "board-reads-the-encoder",
        "the board's inverters drive the inverse of the encoder's quadrature state for that \
         count"
    );
    expect!(
        "one-edge-a-count",
        "every count reaches the board as one change of one inverter output, 102 changes \
         and then 163 in all",
        "a train at half a step an observation moves the encoder one count at a time, each \
         at an instant of its own"
    );
    let _lock = suite_lock();
    let rig = Rig::start(Shape::Assembled { return_volts: 0.0 });
    assert_eq!(rig.outputs(), inverted(0), "at rest before any step");
    let before = rig.transitions();

    rig.train(FORWARD_STEPS);
    assert_eq!(rig.shaft.commanded_steps(), FORWARD_STEPS as i64);
    assert_eq!(rig.encoder.count(), FORWARD_STEPS as i64);
    assert_eq!(rig.outputs(), inverted(FORWARD_STEPS));
    assert_eq!(rig.transitions() - before, FORWARD_STEPS as usize);

    // Back: DIR low is reverse, latched as the net changes; then a train of
    // its own.
    rig.pin(1).drive(Drive::Thevenin(LOW));
    rig.train(BACK_STEPS);
    let at = FORWARD_STEPS - BACK_STEPS;
    assert_eq!(rig.shaft.commanded_steps(), at as i64);
    assert_eq!(rig.encoder.count(), at as i64);
    assert_eq!(rig.outputs(), inverted(at));
    assert_eq!(
        rig.transitions() - before,
        (FORWARD_STEPS + BACK_STEPS) as usize
    );
    assert_eq!(rig.encoder.snapped_updates(), 0, "every count walked");
    rig.finish();
}

#[rstest]
fn an_assembled_drive_reads_its_inputs_against_the_return_the_assembly_declares() {
    behaviour!(Test {
        id: "assembly.inputs-against-their-return",
        covers: Some("board/src/assembly.rs#Assembly::reference"),
        given: "the assembled drive and encoder, the drive's inputs measured against a return \
                the assembly declares, that return held at the 3.3 volts the enable is driven \
                at, and a train of 102 steps",
    });
    expect!(
        "train-moves-nothing",
        "the drive sees no enable and no step, and moves nothing: the encoder stays at 0 \
         and the board's outputs never change",
        "an input is read across its own return, so a return at the enable's own level \
         leaves no voltage across the enable"
    );
    let _lock = suite_lock();
    let rig = Rig::start(Shape::Assembled { return_volts: 3.3 });
    let before = rig.transitions();
    rig.train(FORWARD_STEPS);
    assert!(
        !rig.shaft.enabled(),
        "the enable reads low against its return"
    );
    assert_eq!(rig.shaft.commanded_steps(), 0);
    assert_eq!(rig.encoder.count(), 0);
    assert_eq!(rig.outputs(), inverted(0));
    assert_eq!(rig.transitions(), before);
    rig.finish();
}

#[rstest]
fn an_assembly_costs_the_engine_what_its_members_cost_as_separate_components() {
    behaviour!(Test {
        id: "assembly.engine-cost",
        covers: Some("board/src/assembly.rs#Assembly"),
        given: "the drive turning the encoder, once as one assembly and once as two bench \
                components linked the same way, each sent the same 102-step train",
    });
    expect!(
        "same-events",
        "the engine records exactly as many events for the train with the members \
         assembled as with them apart",
        "the engine sees one node where there were two: the same pins, drives, senses and \
         wakes, and nothing an assembly adds reaches it"
    );
    expect!(
        "same-outcome",
        "both ways the encoder ends at the 102 counts the train carried"
    );
    let _lock = suite_lock();
    let mut measured = Vec::new();
    for shape in [Shape::Assembled { return_volts: 0.0 }, Shape::Separate] {
        let rig = Rig::start(shape);
        let before = rig.events();
        rig.train(FORWARD_STEPS);
        measured.push((shape, rig.events() - before, rig.encoder.count()));
        rig.finish();
    }
    println!("engine events for the train: {measured:?}");
    let [(_, assembled, assembled_count), (_, separate, separate_count)] = measured[..] else {
        unreachable!("two shapes");
    };
    assert_eq!(assembled, separate);
    assert_eq!(assembled_count, FORWARD_STEPS as i64);
    assert_eq!(separate_count, FORWARD_STEPS as i64);
}

// ============================================================
// Whose wakes are whose
// ============================================================

/// The instants the timekeepers were woken at, after the system's start,
/// with their names, in the order they were woken.
type Woken = Arc<Mutex<Vec<(&'static str, u64)>>>;

/// How a timekeeper asks for its wakes.
#[derive(Clone, Copy)]
enum Asks {
    /// One wake at each instant after the start.
    At(&'static [u64]),
    /// A wake every period from the start, asked for at attach as the
    /// drive asks for its observations.
    Every(u64),
    /// One wake at an instant after the start, and once woken, one more at
    /// the instant it is handed.
    AtAndAgain(u64),
}

/// A member that only keeps time: no pins, its wakes recorded.
struct Timekeeper {
    name: &'static str,
    asks: Asks,
    woken: Woken,
    started_ns: Arc<Mutex<u64>>,
    io: Option<ComponentNetIo>,
}

impl Timekeeper {
    fn new(name: &'static str, asks: Asks, woken: &Woken) -> Box<Self> {
        Box::new(Self {
            name,
            asks,
            woken: Arc::clone(woken),
            started_ns: Arc::default(),
            io: None,
        })
    }
}

impl Component for Timekeeper {
    fn pins(&self) -> &[PinDecl] {
        &[]
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let (name, asks, woken) = (self.name, self.asks, Arc::clone(&self.woken));
        let started = Arc::clone(&self.started_ns);
        let again = io.clone();
        let asked_again = Arc::new(Mutex::new(false));
        io.on_wake_ns(move |now| {
            let start = *started.lock().unwrap();
            woken.lock().unwrap().push((name, now - start));
            if let Asks::AtAndAgain(_) = asks {
                let mut asked = asked_again.lock().unwrap();
                if !*asked {
                    *asked = true;
                    again.schedule_at_ns(now);
                }
            }
        });
        // The system's time is held until every component starts, so this
        // is the start instant.
        *self.started_ns.lock().unwrap() = virtual_clock::virtual_ns();
        if let Asks::Every(period) = asks {
            io.schedule_every_ns(period);
        }
        self.io = Some(io);
        Ok(())
    }

    fn start(&mut self) {
        let start = virtual_clock::virtual_ns();
        let io = self.io.as_ref().expect("start runs after attach");
        match self.asks {
            Asks::At(instants) => {
                for at in instants {
                    io.schedule_at_ns(start + at);
                }
            }
            Asks::AtAndAgain(at) => io.schedule_at_ns(start + at),
            Asks::Every(_) => {}
        }
    }
}

const MS: u64 = 1_000_000;

#[rstest]
fn each_member_of_an_assembly_is_woken_at_its_own_instants_in_the_order_added() {
    behaviour!(Test {
        id: "assembly.member-wakes",
        covers: Some("board/src/assembly.rs#Assembly"),
        given: "an assembly whose members only keep time, run for 5.5 milliseconds: one asks \
                for 1 and 3 milliseconds, one for every 2, one for 2 and again when woken, \
                and a nested assembly's member for 3",
    });
    expect!(
        "own-instants",
        "each member is woken at exactly the instants it asked for and at no other"
    );
    expect!(
        "order-added",
        "members due at one instant are woken in the order they were added to the assembly"
    );
    expect!(
        "again-at-the-instant",
        "a member that asks for the instant it is handed is woken again at that instant, \
         once",
        "the engine wakes a node once an instant and again if it asks again, and an \
         assembly hands its members the same"
    );
    expect!(
        "nested",
        "a member that is itself an assembly hands the instants on to its own members"
    );
    let _lock = suite_lock();
    virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);
    let woken = Woken::default();
    let inner = Assembly::new()
        .member(
            "TICK",
            Timekeeper::new("TICK", Asks::At(&[3 * MS]), &woken),
            &[],
        )
        .expect("the inner member joins");
    let outer = Assembly::new()
        .member(
            "EARLY",
            Timekeeper::new("EARLY", Asks::At(&[MS, 3 * MS]), &woken),
            &[],
        )
        .and_then(|a| {
            a.member(
                "EVERY",
                Timekeeper::new("EVERY", Asks::Every(2 * MS), &woken),
                &[],
            )
        })
        .and_then(|a| {
            a.member(
                "AGAIN",
                Timekeeper::new("AGAIN", Asks::AtAndAgain(2 * MS), &woken),
                &[],
            )
        })
        .and_then(|a| a.member("INNER", Box::new(inner), &[]))
        .expect("the members join");
    let system = System::new()
        .component("CLOCKS", Box::new(outer))
        .hold_time()
        .start()
        .expect("the assembly starts");
    let actor = virtual_clock::register_actor("assembly-wakes-case");
    system.release_time();
    virtual_clock::wait_virtual_ns(5 * MS + MS / 2);
    assert_eq!(
        *woken.lock().unwrap(),
        [
            ("EARLY", MS),
            ("EVERY", 2 * MS),
            ("AGAIN", 2 * MS),
            ("AGAIN", 2 * MS),
            ("EARLY", 3 * MS),
            ("TICK", 3 * MS),
            ("EVERY", 4 * MS),
        ]
    );
    drop(actor);
    finish(system);
}
