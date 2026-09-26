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
//! | `P6` ENA → the enable sink | `IC14`, then `Q1` `NPN` | the base at its knee, and its current |
//! | encoder → `P9`..`P12` | `IC16` `ISO6740FDWR` | the receiver's output, and the fail-safe |
//! | end switch → `P19` | `IC9` `NSI50010` + `U6` `VO2631` | a closed contact regulating the loop and lighting the opto |
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
//!   working machine has to provide;
//! - a wire from `IC14`'s secondary ground to `EN_GND`: the rework the
//!   schematic defect `edgeboard.rs` asserts
//!   (`the_servo_isolator_secondary_ground_is_unconnected`) needs. A part
//!   measures its supply against its own ground pin (`NODES.md` §12 item 5,
//!   the sense task), and as drawn `IC14`'s `GND2` pins reach only `C26`, so
//!   its secondary side is down and nothing crosses it — the bench
//!   behaviour the defect predicts. The wire is the fix, said out loud.
//!
//! # Time
//!
//! Every case runs alone, on the stepped clock, and reads what it asserts at
//! a settled virtual instant (`TESTING.md` rules 5 and 9): the case's thread
//! is a registered actor from the moment its system is assembled until it
//! shuts down, so the engine advances only while the case is parked in
//! [`settle`], and every read after a settle is of the system at rest at
//! that instant — never a wall-clock poll of a cascade in flight. Two flakes
//! of the free-running rig this replaces, both root-caused in `NODES.md`
//! §12 item 5 (the flake record after the final pass): the receiver output
//! read `Floating` in the middle of its own start-up, after a poll had
//! accepted the output pin's idle `Driven(High)` as settled; and the step
//! profile's event count split 54/58 when another case's clock jump left
//! this engine pacing against the wall with a wake due.

mod machine_parts;

use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};

use embsim_core::virtual_clock::{self, Actor, ClockMode};

use rstest::rstest;

use embsim_board::netlist::{self, ComponentDecl};
use embsim_board::registry::normalize_part;
use embsim_board::{
    jesd8c01_lvcmos_thresholds, AttachError, Board, Component, ComponentNetIo, DeadBand, Drive,
    EventLog, Finding, Harness, Level, NetState, PartRegistry, PeriodicSchedule, PinDecl,
    PinHandle, Scenario, System, SystemHandle, TheveninDrive,
};
use embsim_models::isolation::iso67xx;
use embsim_models::isolation::{Channel, Iso67xx, Iso67xxMonitor};
use embsim_models::logic_gate::LVC1G14_T_PD_NS;
use embsim_models::machine::{end_switch, ActuationSense, EndSwitch, EndSwitchActuator};
use embsim_models::opto::{Opto, OptoChannel, OptoMonitor};
use embsim_models::pwl_library::{MMBT3904_VBE_VOLTS, NSI50010_I_REG_AMPS};
use embsim_models::rail::UCC12040_RISE_NS;
use machine_parts::{bench_rails, edge_registry, ep};

// ============================================================
// Shared fixtures
// ============================================================

const EDGE: &str = "EdgeBoard";

/// One case at a time: the virtual clock is process-global, and each case
/// re-anchors it in stepped mode (`TESTING.md` rule 5).
///
/// This binary's cases once ran in parallel on one free-running clock, and
/// that sharing was one of the two flakes: a test thread parked on a 200 ms
/// virtual wait is released by whichever engine advances to it, *before*
/// that engine's pacing sleep, so every other engine in the process then
/// saw virtual time up to a segment ahead of the wall and paced against it
/// — with a wake due — while its own case's wall-clock "quiet" window
/// closed (`NODES.md` §12 item 5, the flake record).
static SUITE_LOCK: Mutex<()> = Mutex::new(());

fn suite_lock() -> MutexGuard<'static, ()> {
    SUITE_LOCK.lock().unwrap_or_else(|poisoned| {
        SUITE_LOCK.clear_poison();
        poisoned.into_inner()
    })
}

/// The virtual time a case hands the engine after a stimulus, before it
/// reads what the stimulus did: 1 ms. Longer than every instant a part in
/// the rig arms — an indicator inverter's `t_pd` (SN74LVC1G14 §5.6, 4.6 ns
/// max, 5 ns on the wheel: [`LVC1G14_T_PD_NS`]) and the longest start-up
/// any Edge board part declares, the UCC12040's `VISO` rise (SNVSBO5B
/// §6.9, 750 µs typ: [`UCC12040_RISE_NS`]) — so a settled read is the
/// system at rest, not a cascade in flight. The window is the harness's,
/// not a part's: any span past the longest armed instant reads the same.
const SETTLE_NS: u64 = 1_000_000;
const _: () = assert!(SETTLE_NS > UCC12040_RISE_NS && SETTLE_NS > LVC1G14_T_PD_NS);

/// Park the case's thread for [`SETTLE_NS`] of virtual time and return with
/// the system at rest.
///
/// The thread is a registered actor ([`Rig::actor`]), and the stepped
/// engine advances only while every actor is parked: here it drains every
/// command the case sent before the call, delivers every sense that moves,
/// fires every wake due in the window, and then releases the thread — and
/// it does nothing more until the thread parks again. A read between two
/// settles is exact. Never wait on the wall clock between them: the engine
/// is waiting for the case.
fn settle() {
    virtual_clock::wait_virtual_ns(SETTLE_NS);
}

/// The engine's report of `net`, read at the settled instant.
fn state(system: &SystemHandle, net: &str) -> NetState {
    system
        .net_state(net)
        .unwrap_or_else(|| panic!("net {net} exists"))
}

/// One virtual microsecond after a stimulus. Same idea as `edgeboard.rs`:
/// the engine drains the attach/drive cascade to a fixpoint, then the wake
/// samples settled DC — not a wall-race glance at a transient. Inside the
/// [`settle`] window the capture is read after.
const SETTLE_WAKE_US: u64 = 1;
const _: () = assert!(SETTLE_WAKE_US * 1_000 < SETTLE_NS);

/// End-switch / opto loop facts captured on the engine thread.
#[derive(Clone, Debug)]
struct EndSwitchSettled {
    p19: NetState,
    lit: bool,
    sinking: bool,
    current_ma: f64,
}

/// High-impedance probe on `P19` that also snapshots the opto monitor at
/// the settle wake — the loop current is the opto's LED current, which the
/// engine delivers it from the loop's solve. Re-armable so open → closed →
/// open cycles can each capture a virtual-time settled sample.
struct EndSwitchSettleProbe {
    pins: [PinDecl; 1],
    opto: OptoMonitor,
    capture: Arc<Mutex<Option<EndSwitchSettled>>>,
    done: Arc<AtomicBool>,
    io: Arc<Mutex<Option<ComponentNetIo>>>,
}

impl Component for EndSwitchSettleProbe {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let y = io.pin("Y")?;
        let opto = self.opto.clone();
        let capture = Arc::clone(&self.capture);
        let done = Arc::clone(&self.done);
        io.on_wake(move |_now_us| {
            *capture.lock().unwrap() = Some(EndSwitchSettled {
                p19: y.net_report(),
                lit: opto.is_lit(OptoChannel::Two),
                sinking: opto.is_sinking(OptoChannel::Two),
                current_ma: opto
                    .forward_amps(OptoChannel::Two)
                    .map_or(0.0, |amps| amps * 1e3),
            });
            done.store(true, Ordering::SeqCst);
        });
        *self.io.lock().unwrap() = Some(io.clone());
        io.schedule_at(virtual_clock::virtual_us() + SETTLE_WAKE_US);
        Ok(())
    }
}

fn settle_probe_pin() -> PinDecl {
    PinDecl::digital_in("Y", jesd8c01_lvcmos_thresholds(DeadBand::Unknown))
}

/// Re-arm the end-switch settle probe and settle past its wake: the capture
/// is the engine thread's, taken [`SETTLE_WAKE_US`] after the stimulus.
fn capture_end_switch_settled(
    done: &AtomicBool,
    capture: &Mutex<Option<EndSwitchSettled>>,
    io: &Mutex<Option<ComponentNetIo>>,
) -> EndSwitchSettled {
    done.store(false, Ordering::SeqCst);
    *capture.lock().unwrap() = None;
    let handle = io
        .lock()
        .unwrap()
        .clone()
        .expect("end-switch settle probe attached");
    handle.schedule_at(virtual_clock::virtual_us() + SETTLE_WAKE_US);
    settle();
    assert!(
        done.load(Ordering::SeqCst),
        "the end-switch settle wake fires inside the settle window"
    );
    capture
        .lock()
        .unwrap()
        .clone()
        .expect("settle wake fired without capturing")
}

// ============================================================
// The promoted parts
// ============================================================

/// Handles onto every promoted instance, keyed by reference designator.
///
/// A `PartRegistry` constructor is handed a [`ComponentDecl`] and returns a
/// boxed component the system then owns, so the only way to keep a handle on
/// a *particular* instance is to record it as it is built. That is what this
/// is: `IC14`'s monitor and `U6`'s, each under its own reference. The
/// regulator `IC9` and the transistor `Q1` are elements by specification
/// with no instance to hold: their currents are read from the system
/// (`branch_current`, `pin_current`).
#[derive(Clone, Default)]
struct Promoted {
    isolators: Arc<Mutex<HashMap<String, Iso67xxMonitor>>>,
    optos: Arc<Mutex<HashMap<String, OptoMonitor>>>,
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

    fn opto(&self, reference: &str) -> OptoMonitor {
        self.optos
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

/// [`machine_parts::edge_registry`] with the isolators and the optocouplers
/// re-registered so this binary holds a monitor on each instance.
///
/// The registrations mirror the board's own (`machine_parts::edge_registry`
/// registers the same models from the same part names; the regulator and
/// the transistor come from the element library there too, since `NODES.md`
/// §8 phase 3); the re-registration exists only to capture each instance's
/// monitor as it is built.
fn promoted_registry(promoted: &Promoted) -> PartRegistry {
    let mut registry = edge_registry();

    // Every ISO67xx on the board, configured straight from its part name —
    // `ISO6740FDWR` picks up its fail-safe-low default without anyone
    // re-deriving it from the suffix.
    for part in ["ISO6742DWR", "ISO6741DWR", "ISO6740FDWR", "ISO6721BDR"] {
        let promoted = promoted.clone();
        registry.register(part, move |decl: &ComponentDecl| {
            let name = normalize_part(decl);
            let config = iso67xx::Config::from_part_name(&name)
                .unwrap_or_else(|| panic!("{name} is an ISO67xx"));
            let isolator = Iso67xx::new(config).expect("a valid isolator configuration");
            promoted
                .isolators
                .lock()
                .unwrap()
                .insert(decl.reference.clone(), isolator.monitor());
            Box::new(isolator)
        });
    }

    for (part, build) in [
        ("VO2631", Opto::vo2631 as fn() -> Opto),
        ("6N137", Opto::lite_on_6n137 as fn() -> Opto),
    ] {
        let promoted = promoted.clone();
        registry.register(part, move |decl: &ComponentDecl| {
            let opto = build();
            promoted
                .optos
                .lock()
                .unwrap()
                .insert(decl.reference.clone(), opto.monitor());
            Box::new(opto)
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

/// A stand-in for the P2's driven pins: three push-pull outputs, `STEP`
/// driven with a periodic drive when a test clocks it.
///
/// Deliberately not an [`embsim_board::McuComponent`]: this binary needs no
/// firmware and no peripheral banks, and a component that claimed the
/// process-default banks would have to own its own suite lock (`TESTING.md`
/// rule 5).
struct FakePins {
    pins: Vec<PinDecl>,
    handles: Arc<Mutex<HashMap<&'static str, PinHandle>>>,
}

impl FakePins {
    fn new() -> Self {
        Self {
            pins: vec![out("STEP"), out("DIR"), out("ENA")],
            handles: Arc::new(Mutex::new(HashMap::new())),
        }
    }
}

fn out(number: &'static str) -> PinDecl {
    PinDecl::digital_out(number)
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
        Ok(())
    }
}

/// A stand-in for whatever consumes the step clock on the isolated side. It is
/// harnessed onto the RS-422 driver's own `1A` input pin, so it watches the
/// exact net the schematic feeds the stepper driver from, and records every
/// segment that net carries.
struct FakeStepSink {
    trains: Arc<Mutex<Vec<PeriodicSchedule>>>,
}

const STEP_SINK_PINS: [PinDecl; 1] = [PinDecl::digital_in(
    "IN",
    jesd8c01_lvcmos_thresholds(DeadBand::Unknown),
)];

impl Component for FakeStepSink {
    fn pins(&self) -> &[PinDecl] {
        &STEP_SINK_PINS
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let trains = Arc::clone(&self.trains);
        io.on_sense("IN", move |sense| {
            if let Some(clock) = sense.periodic {
                trains.lock().unwrap().push(clock.segment);
            }
        })
    }
}

// ============================================================
// Rig
// ============================================================

/// Everything a test drives or reads.
///
/// Field order is drop order, and it is load-bearing: the case's actor
/// registration goes first, so a case that panics stops holding the
/// engine's barrier before its system shuts down, and the suite lock goes
/// last, after the engine has been joined.
struct Rig {
    /// The case's thread as a registered virtual-clock actor, from the
    /// assembled system to [`Rig::finish`]: what makes [`settle`] exact.
    actor: Actor,
    system: SystemHandle,
    promoted: Promoted,
    handles: Arc<Mutex<HashMap<&'static str, PinHandle>>>,
    trains: Arc<Mutex<Vec<PeriodicSchedule>>>,
    end_switch: EndSwitchActuator,
    _suite: MutexGuard<'static, ()>,
}

impl Rig {
    /// End the case: the engine must never have stopped waiting for the
    /// case's thread (a `QuiescenceTimeout` would mean a settle read a
    /// system that was still moving), then the thread leaves the barrier
    /// and the system shuts down.
    fn finish(self) {
        let Rig {
            actor,
            system,
            _suite,
            ..
        } = self;
        let stalled: Vec<Finding> = system
            .findings()
            .into_iter()
            .filter(|f| matches!(f, Finding::QuiescenceTimeout { .. }))
            .collect();
        assert!(
            stalled.is_empty(),
            "the engine stopped waiting for the case's thread, so a settled read \
             may have raced the system: {stalled:?}"
        );
        drop(actor);
        system.shutdown();
    }

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

    /// Clock `STEP` with `segment`, rail to rail at the pad's 25 Ω.
    fn publish_step(&self, segment: PeriodicSchedule) {
        self.pin("STEP").drive(Drive::Periodic {
            hi: TheveninDrive {
                volts: 3.3,
                impedance: 25.0,
            },
            lo: TheveninDrive {
                volts: 0.0,
                impedance: 25.0,
            },
            segment,
        });
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
        // The rework: `IC14`'s orphaned `GND2_1`/`GND2_2` net onto `EN_GND`
        // (`J21.8`), the isolated ground its own supply `SC_5V` returns to.
        .connect(ep(&format!("{EDGE}.IC14.9")), ep(&format!("{EDGE}.J21.8")))
        .connect(ep("MCU.STEP"), ep(&format!("{EDGE}.J3.{STEP_FINGER}")))
        .connect(ep("MCU.DIR"), ep(&format!("{EDGE}.J3.{DIR_FINGER}")))
        .connect(ep("MCU.ENA"), ep(&format!("{EDGE}.J3.{ENA_FINGER}")))
        // `U24.1` is the AM26LS31's `1A`: the isolated-side net the step
        // signal has to reach.
        .connect(ep("STEPSINK.IN"), ep(&format!("{EDGE}.U24.1")))
        .connect(ep(&format!("{EDGE}.J16.1")), ep("END_U.NO"))
}

/// Probe wiring returned from `start_inner`.
struct EndSwitchSettleParts {
    capture: Arc<Mutex<Option<EndSwitchSettled>>>,
    done: Arc<AtomicBool>,
    io: Arc<Mutex<Option<ComponentNetIo>>>,
}

/// Handles for re-arming the end-switch settle probe after a position change.
/// One engine at a time is the suite lock's ([`SUITE_LOCK`]), so no sibling
/// case can advance the clock under the probe's wake.
struct EndSwitchSettle {
    parts: EndSwitchSettleParts,
}

impl EndSwitchSettle {
    fn capture(&self) -> EndSwitchSettled {
        capture_end_switch_settled(&self.parts.done, &self.parts.capture, &self.parts.io)
    }
}

/// Build and start the promoted board, settled.
///
/// `servo_domain` powers `SC_5V` — the isolated servo rail `IC14`'s side 2 and
/// `IC16`'s side 1 run from. Dropping it is how a test asks "what does an
/// isolator with one side dead do?".
fn start(servo_domain: bool, event_log: bool, sources: &[(&str, f64)]) -> Rig {
    start_inner(servo_domain, event_log, sources, false).0
}

/// Start the end-switch loop with an engine-hosted settle probe on U6.VO2 (net P19).
fn start_end_switch_loop(sources: &[(&str, f64)]) -> (Rig, EndSwitchSettle) {
    let (rig, parts) = start_inner(true, false, sources, true);
    let probe = EndSwitchSettle {
        parts: parts.expect("end-switch settle probe requested"),
    };
    (rig, probe)
}

/// Take the suite lock, re-anchor the clock in stepped mode, build and start
/// the rig with time held, register the case's thread as an actor, release
/// time and settle: the rig is handed back at rest at its first settled
/// instant, [`SETTLE_NS`] after the system was assembled, every attach-time
/// cascade drained.
///
/// Time is held until the thread has registered so that instant is the
/// same every run: released at once, the engine could advance past the
/// start-up wakes before the thread joined the barrier.
fn start_inner(
    servo_domain: bool,
    event_log: bool,
    sources: &[(&str, f64)],
    with_end_switch_settle: bool,
) -> (Rig, Option<EndSwitchSettleParts>) {
    let suite = suite_lock();
    virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);
    let promoted = Promoted::default();
    let board = promoted_board(&promoted);

    let mcu = FakePins::new();
    let handles = Arc::clone(&mcu.handles);
    let trains: Arc<Mutex<Vec<PeriodicSchedule>>> = Arc::new(Mutex::new(Vec::new()));
    let sink = FakeStepSink {
        trains: Arc::clone(&trains),
    };

    let switch = EndSwitch::new(end_switch::Config::new(100.0, ActuationSense::Increasing))
        .expect("a valid end switch");
    let end_switch = switch.actuator();

    let mut scenario = machine_parts::encoder_jumpers_closed(Scenario::default(), EDGE);
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

    let settle_handles = if with_end_switch_settle {
        let capture = Arc::new(Mutex::new(None));
        let done = Arc::new(AtomicBool::new(false));
        let io = Arc::new(Mutex::new(None));
        let probe = EndSwitchSettleProbe {
            pins: [settle_probe_pin()],
            opto: promoted.opto("U6"),
            capture: Arc::clone(&capture),
            done: Arc::clone(&done),
            io: Arc::clone(&io),
        };
        Some((probe, EndSwitchSettleParts { capture, done, io }))
    } else {
        None
    };

    let mut system = System::new()
        .board(EDGE, board)
        .component("MCU", Box::new(mcu))
        .component("STEPSINK", Box::new(sink))
        .component("END_U", Box::new(switch));
    let probe = if let Some((probe, handles)) = settle_handles {
        system = system
            .component("END_SETTLE", Box::new(probe))
            .harness(Harness::new().connect(ep("END_SETTLE.Y"), ep(&format!("{EDGE}.U6.6"))));
        Some(handles)
    } else {
        None
    };
    system = system
        .harness(rails)
        .harness(extra_rails())
        .harness(rig_harness())
        .scenario(scenario);
    if event_log {
        system = system.event_log();
    }
    let system = system
        .hold_time()
        .start()
        .expect("the promoted EdgeBoard starts");
    let actor = virtual_clock::register_actor("isolation-bridge-case");
    system.release_time();
    settle();

    (
        Rig {
            actor,
            system,
            promoted,
            handles,
            trains,
            end_switch,
            _suite: suite,
        },
        probe,
    )
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
        monitor.is_passing(STEP_CHANNEL),
        "both of IC14's supplies must come up"
    );
    rig.drive("STEP", volts);
    settle();

    let net = format!("{EDGE}.Net-(IC14-OUTA)");
    assert_eq!(
        state(&rig.system, &net),
        NetState::Driven(expect),
        "P8 must reach the stepper driver's input across IC14"
    );
    rig.finish();
}

/// `DIR` is a second, independent channel of the same part — proof the model
/// is per-channel and not one hard-wired path, and that driving one channel
/// leaves the others alone.
#[rstest]
fn the_direction_channel_is_independent_of_the_step_channel() {
    let rig = start(true, false, &[]);
    rig.drive("STEP", 3.3);
    rig.drive("DIR", 0.0);
    settle();

    assert_eq!(
        state(&rig.system, &format!("{EDGE}.Net-(IC14-OUTA)")),
        NetState::Driven(Level::High)
    );
    assert_eq!(
        state(&rig.system, &format!("{EDGE}.Net-(IC14-OUTB)")),
        NetState::Driven(Level::Low)
    );
    rig.finish();
}

/// One side powered is no isolator at all. With `SC_5V` dark, `IC14`'s output
/// buffer has no supply, so `OUTA` is released and the net floats — and the
/// engine says why.
#[rstest]
fn an_unpowered_isolated_side_stops_the_step_path() {
    let rig = start(false, false, &[]);
    rig.drive("STEP", 3.3);
    settle();

    let net = format!("{EDGE}.Net-(IC14-OUTA)");
    assert_eq!(
        state(&rig.system, &net),
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
    // released `OUTC` leaves the base network unsourced, so `Q1`'s junction
    // carries nothing and its collector is an open circuit rather than a
    // plausible enable.
    rig.drive("ENA", 3.3);
    settle();
    assert_eq!(
        state(&rig.system, &format!("{EDGE}.Net-(Q1-B)")),
        NetState::Floating
    );
    // The transistor's cluster still solves — its emitter is on the bench
    // ground, a terminal that sources it — and the solve says the base
    // carries nothing: a reading of zero, not the absence of one.
    let into_base = rig
        .system
        .pin_current(&format!("{EDGE}.Q1.2"))
        .expect("the transistor's cluster solves from its grounded emitter");
    assert!(
        into_base.abs() < 1e-9,
        "an unsourced base carries nothing: {into_base}"
    );
    assert_eq!(
        state(&rig.system, &format!("{EDGE}.Net-(JP1-B)")),
        NetState::Floating,
        "an off transistor releases its collector"
    );
    rig.finish();
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
/// # The transistor is an element
///
/// `Q1` is a base–emitter diode and a gated collector from the element
/// library (`NODES.md` §8 phase 3), solved with the base network around
/// it: `Net-(Q1-B)` is reached only through `R24` (43 kΩ) from the
/// isolator's `OUTC`, so with `P6` high the base sits at the datasheet's
/// knee and carries `(V_OUTC − V_BE) / 43 kΩ`, and with `P6` low the
/// junction is off and the base follows `OUTC` down. An earlier revision
/// read the base as `Pulled(Low, 43 000)` and the collector as a switch
/// closed by a model: the base is a solved voltage now — the whole base
/// network is an element cluster — and the collector on `Net-(JP1-B)` has
/// no load with `JP1` open, so it is reached only through the transistor
/// and floats; the loaded collector (a saturated switch under a light
/// load, a sagging one under a heavy load) is `board_elements.rs`'s proof.
#[rstest]
fn the_enable_path_crosses_the_isolator_and_the_transistor() {
    let rig = start(true, false, &[]);
    let outc = format!("{EDGE}.Net-(IC14-OUTC)");
    let base = format!("{EDGE}.Net-(Q1-B)");
    let collector = format!("{EDGE}.Net-(JP1-B)");
    let base_pin = format!("{EDGE}.Q1.2");
    assert!(rig.promoted.isolator("IC14").is_passing(Channel::C));
    let volts = |net: &str| match rig.system.net_state(net) {
        Some(NetState::Analog(v)) => Some(v),
        _ => None,
    };

    // Enable asserted: the isolator drives OUTC high, the base resistor
    // takes it to Q1, and the base–emitter junction turns on at its knee.
    rig.drive("ENA", 3.3);
    settle();
    assert!(
        volts(&base).is_some_and(|v| (MMBT3904_VBE_VOLTS..=0.75).contains(&v)),
        "P6 must reach the base across IC14 and R24, and the junction sit at its knee; got {:?}",
        rig.system.net_state(&base)
    );
    let v_outc = volts(&outc).expect("OUTC is solved with the base network");
    assert!(v_outc > 4.0, "OUTC high in the servo domain: {v_outc}");
    let i_b = rig
        .system
        .pin_current(&base_pin)
        .expect("the base is a branch terminal");
    let expected = (v_outc - volts(&base).unwrap()) / 43_000.0;
    assert!(
        (i_b - expected).abs() < expected * 0.01,
        "the base current is the drop across R24: {i_b} vs {expected}"
    );
    // The unloaded collector: nothing draws through it, so the saturated
    // channel holds it at the emitter — the isolated ground — and it
    // carries nothing.
    assert!(
        matches!(rig.system.net_state(&collector), Some(NetState::Analog(v)) if v.abs() < 1e-3),
        "a saturated collector with no load sits at its emitter: {:?}",
        rig.system.net_state(&collector)
    );
    assert!(
        rig.system
            .pin_current(&format!("{EDGE}.Q1.3"))
            .is_some_and(|amps| amps.abs() < 1e-9),
        "and carries nothing: {:?}",
        rig.system.pin_current(&format!("{EDGE}.Q1.3"))
    );

    // Enable released: the isolator follows P6 down, the base with it, and
    // the junction is off; the collector is reached only through the off
    // transistor and floats.
    rig.drive("ENA", 0.0);
    settle();
    assert!(
        volts(&base).is_some_and(|v| v < 0.1),
        "a released P6 reaches the base through R24; got {:?}",
        rig.system.net_state(&base)
    );
    assert!(
        rig.system
            .pin_current(&base_pin)
            .is_some_and(|i| i.abs() < 1e-9),
        "an off junction carries only leakage: {:?}",
        rig.system.pin_current(&base_pin)
    );
    assert_eq!(
        state(&rig.system, &collector),
        NetState::Floating,
        "an off transistor's unloaded collector is an open circuit"
    );
    rig.finish();
}

// ============================================================
// The encoder return path, and the fail-safe suffix
// ============================================================

/// The encoder path, end to end on the netlist: a differential on `A±`, the
/// RS-422 receiver's decision, `IC16`, and out on the P2's `P9` finger.
///
/// `edgeboard.rs` stops at `Net-(IC16-INA)` because that was as far as a
/// stubbed isolator let it go.
///
/// Read at the settled instant, not polled. The receiver's `1Y` declares a
/// push-pull output's idle, `Driven(High)` — the very state asserted — and
/// on the way to it the part releases `1Y` once: its supply is delivered
/// before its enables, and with `~G` not yet read it is disabled. The
/// free-running version polled until the net read `Driven(High)`, which
/// the idle satisfied before the receiver had run at all, then re-read it
/// inside that release: `Floating` (`NODES.md` §12 item 5, the flake
/// record).
#[rstest]
#[case::forward(3.3, Level::High)]
#[case::reverse(0.0, Level::High)]
fn the_encoder_reaches_the_p2_across_the_isolator(#[case] a_plus: f64, #[case] expect: Level) {
    let rig = start(true, false, &[("EdgeBoard./MaD_Edge_Sheet3/A+", a_plus)]);
    assert!(
        rig.promoted.isolator("IC16").is_passing(Channel::A),
        "both of IC16's supplies must come up"
    );
    let receiver = format!("{EDGE}.Net-(IC16-INA)");
    assert_eq!(
        state(&rig.system, &receiver),
        NetState::Driven(expect),
        "the receiver's own output, as edgeboard.rs asserts it"
    );
    assert_eq!(
        state(&rig.system, &format!("{EDGE}.P9")),
        NetState::Driven(expect),
        "and now it reaches P9 across IC16"
    );
    rig.finish();
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
            state(&rig.system, &net),
            NetState::Driven(Level::Low),
            "{pin} must present the ISO6740F default, not float"
        );
    }
    let monitor = rig.promoted.isolator("IC16");
    assert!(!monitor.is_passing(Channel::A), "nothing is being relayed");
    assert_eq!(monitor.output_level(Channel::A), Some(Level::Low));
    assert_eq!(monitor.config().default_level(), Level::Low);
    rig.finish();
}

// ============================================================
// The end-switch loop: regulator, LED, detector
// ============================================================

/// The end-switch path, which no engine event could cross before: a closed
/// contact completes the current loop, the constant-current regulator sees
/// overhead and regulates, the optocoupler's LED lights, and its
/// open-collector output pulls `P19` down against the board's own 1 kΩ
/// pull-up.
///
/// The loop is one cluster solve since `NODES.md` §8 phase 3: `IC9` is a
/// two-region regulating branch and `U6`'s LED a diode branch, so the loop
/// current is the regulator's 10 mA — an earlier revision's resistive
/// stand-ins carried 27 mA from the 24 V bench rail and *reported* 10 mA.
/// Open, every one of those is false — and `P19` sits at the pull-up, which
/// is exactly what "pulled to its rail, not floating" means.
#[rstest]
fn a_closed_end_switch_lights_the_optocoupler_and_pulls_p19_down() {
    // Engine-hosted settle probe (see edgeboard `SettleProbe`): open/closed
    // facts are sampled on the engine thread after a virtual-time wake, and
    // read after the case has settled past it, so a mid-cascade transient
    // cannot fail the lit/current asserts (ubuntu release smoke flake on
    // #50 / run 35007027583).
    let (rig, probe) = start_end_switch_loop(&[]);
    let opto = rig.promoted.opto("U6");
    let regulator = format!("{EDGE}.IC9");
    let regulation_ma = NSI50010_I_REG_AMPS * 1e3;

    // Open contact: no return path, so the loop carries nothing.
    rig.end_switch.set_position_mm(0.0);
    let open = probe.capture();
    assert!(
        matches!(open.p19, NetState::Pulled(Level::High, _)),
        "an unlit optocoupler must leave P19 to the pull-up; got {:?}",
        open.p19
    );
    assert_eq!(open.current_ma, 0.0, "an open loop regulates nothing");
    assert!(!open.lit, "open loop must leave the opto dark");
    assert!(!open.sinking, "open loop must release the open-collector");
    // The monitor and the system agree with the engine-thread snapshot.
    assert!(!opto.is_lit(OptoChannel::Two));
    assert!(!opto.is_sinking(OptoChannel::Two));
    // The loop's cluster solves — the 24 V bench supply on its anode is a
    // terminal that sources it — and the solve reads nothing through the
    // regulator: a zero, not the absence of a reading.
    let through_regulator = rig
        .system
        .branch_current(&regulator)
        .expect("the loop's cluster solves from the 24 V supply on its anode");
    assert!(
        through_regulator.abs() < 1e-9,
        "an open loop carries nothing through the regulator: {through_regulator}"
    );

    // Closed contact: the loop completes and the regulator holds it.
    rig.end_switch.set_position_mm(150.0);
    let closed = probe.capture();
    assert!(
        matches!(closed.p19, NetState::Driven(Level::Low)),
        "a closed contact must pull P19 down; got {:?}",
        closed.p19
    );
    assert!(
        (closed.current_ma - regulation_ma).abs() < regulation_ma * 0.01,
        "the regulator holds the loop at its regulation current, got {} mA",
        closed.current_ma
    );
    assert!(closed.lit, "the LED must be lit past ITH");
    assert!(closed.sinking, "a lit, powered detector must sink");
    let through_regulator = rig
        .system
        .branch_current(&regulator)
        .expect("the loop solved");
    assert!(
        (through_regulator - NSI50010_I_REG_AMPS).abs() < NSI50010_I_REG_AMPS * 0.01,
        "the regulator's own branch carries the loop current: {through_regulator}"
    );
    let through_led = opto.forward_amps(OptoChannel::Two).expect("delivered");
    assert!(
        (through_led - through_regulator).abs() < 1e-9,
        "a series loop carries one current: {through_led} vs {through_regulator}"
    );
    assert!(opto.is_sinking(OptoChannel::Two));

    // And back: the path is not one-way.
    rig.end_switch.set_position_mm(0.0);
    let reopen = probe.capture();
    assert!(
        matches!(reopen.p19, NetState::Pulled(Level::High, _)),
        "re-opening must return P19 to the pull-up; got {:?}",
        reopen.p19
    );
    assert!(!reopen.lit);
    assert!(!reopen.sinking);
    rig.finish();
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
    settle();
    assert!(
        opto.is_lit(OptoChannel::Two),
        "the LED loop is still powered and the contact is closed"
    );
    assert!(!opto.is_powered(), "but the detector is not");
    assert!(!opto.is_sinking(OptoChannel::Two));
    assert!(
        matches!(state(&rig.system, &p19), NetState::Pulled(Level::High, _)),
        "P19 must sit at the pull-up, not be held low; got {:?}",
        rig.system.net_state(&p19)
    );
    rig.finish();
}

// ============================================================
// The budget: a step train crosses without scaling engine traffic
// ============================================================

/// Engine events a whole four-segment step profile costs, from the settled
/// instant it starts at to the settled instant after its stop — asserted
/// exactly, at both rates: a span of virtual time, not a wall-clock quiet
/// window. (The free-running version closed its window after 50 ms of wall
/// quiet, and under load split one profile 54/58: the stop's `t_pd` wake
/// had not fired, then fired at the next profile's first instant, after
/// its drive — `NODES.md` §12 item 5, the flake record.)
///
/// The alternative — an isolator that re-drove its output pin per STEP edge —
/// is ~8192 events per millimetre at the reference machine's resolution, so a
/// regression would miss this ceiling by orders of magnitude.
///
/// **Measured, and the ceiling is the measurement: 57 events per
/// four-change profile, at 8 192 Hz and at 819 200 Hz alike** (the test
/// prints it). The step clock is a periodic drive on the STEP net, so a rate
/// change is a drive like any other, and it reaches everything on the net
/// that reads it — 12 records per running rate change: the source's drive
/// with its net resolved (two identity nets) and handed to `IC14`'s input
/// and to the `P8` indicator's SN74LVC1G14 (5); `IC14`'s relay, one periodic
/// drive, with the isolated net resolved (two identity nets) and handed to
/// the sink and to `U24`'s `1A` (5); and the indicator inverter's own relay
/// onto its LED net (2). `U24` re-issues nothing it already drives (the
/// harness model publishes a changed pair only, as the isolator and the
/// gates do), so it costs only where its input changes meaning: the first
/// rate change turns the resting line it drove from into a running clock it
/// reads no level from, and it releases its pair (2 drives, 2 resolutions:
/// 16); the stop hands it the held clock's resting low (the node names its
/// low phase's voltage, `NODES.md` §12 item 5, the review) and it drives the
/// pair again (4), while the inverter, out of rate mode, reads the same
/// resting low and drives its LED high `t_pd` later (a wake, a drive and a
/// resolution in place of its relay's two: 5 + 5 + 4 + 3 = 17). 16 + 12 +
/// 12 + 17 = 57. While the train rode a pulse channel beside the net the
/// same profile cost 19 events (the phase-4 tree, measured) and the ceiling
/// was 32 — the ~7× per rate change `sil-unified-drive.md` measured for a
/// drive on this rig ("level (drive) 14"; the two relays beyond it are the
/// readers the channel never reached), bounded, and still independent of
/// the rate, which is the property this guards.
const RELAY_EVENT_CEILING: usize = 57;

/// Engine events the one level-to-clock change costs: the STEP line, held
/// at a level, becomes a clock at rest (a held segment) before a profile
/// is measured. **Measured, and the ceiling is the measurement: 18.** The
/// source's drive, its net resolved, handed to `IC14` and the indicator,
/// its identity net resolved (5); `IC14`'s first relay onto the isolated
/// net, resolved and handed on (5); `U24`, handed the held clock's resting
/// low where it read the level's high, re-drives its pair (4); the
/// inverter, reading the same low, drives its LED `t_pd` later (a wake, a
/// drive, its two LED nets resolved: 4). Paid once per line, not per
/// profile, and asserted exactly here so the level path's cost stays under
/// test.
const PRIME_EVENT_CEILING: usize = 18;

/// Virtual time each segment of a profile holds for, the stop included: the
/// profile is one fixed span of virtual time, 800 ms.
const SEGMENT_US: u64 = 200_000;
const _: () = assert!(SEGMENT_US * 1_000 > SETTLE_NS);

/// One four-segment profile at `base_hz`, `2 x base_hz`, `base_hz`, stop.
/// Returns `(pulses emitted, engine events, relays)`.
///
/// Each segment is published at a settled instant — the case's thread holds
/// the engine there, so `since_ns` is that instant exactly — and held for
/// [`SEGMENT_US`] of virtual time, during which the engine delivers the
/// change, every relay and every wake it arms; the count is the log's
/// length from the first instant to the last.
fn run_profile(
    rig: &Rig,
    log: &EventLog,
    monitor: &Iso67xxMonitor,
    base_hz: u32,
    published: &mut u64,
) -> (u64, usize, u64) {
    let before = log.len();
    let relays_before = monitor.train_count();
    let received_before = rig.trains.lock().unwrap().len();
    let mut pulses = 0u64;

    for (index, multiplier) in [1u32, 2, 1, 0].into_iter().enumerate() {
        let freq_hz = base_hz * multiplier;
        rig.publish_step(PeriodicSchedule {
            emitted: *published,
            freq_hz,
            total: None,
            since_ns: virtual_clock::virtual_ns(),
        });
        virtual_clock::wait_virtual_us(SEGMENT_US);
        assert_eq!(
            rig.trains.lock().unwrap().len(),
            received_before + index + 1,
            "segment {index} at {base_hz} Hz must reach the isolated side, once"
        );
        let emitted = u64::from(freq_hz) * SEGMENT_US / 1_000_000;
        pulses += emitted;
        *published += emitted;
    }
    (
        pulses,
        log.len() - before,
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
    settle();
    assert_eq!(
        state(&rig.system, &format!("{EDGE}.Net-(IC14-OUTA)")),
        NetState::Driven(Level::High)
    );
    // Nothing has clocked the line yet, so nothing has been relayed: the
    // STEP net carries a level, and an isolator relays a clock only when its
    // input carries one.
    assert!(rig.trains.lock().unwrap().is_empty());
    // The line becomes a clock at rest — a held segment — before anything
    // is measured: the level-to-clock change is the level path's cost, paid
    // once here, so both profiles below start from the same line.
    let before_prime = log.len();
    rig.publish_step(PeriodicSchedule::IDLE);
    settle();
    assert_eq!(
        rig.trains.lock().unwrap().len(),
        1,
        "the held clock must reach the isolated side"
    );
    let prime = log.len() - before_prime;
    assert_eq!(
        prime, PRIME_EVENT_CEILING,
        "the level-to-clock change costs {prime} engine events (the measurement is \
         {PRIME_EVENT_CEILING})"
    );

    let mut published = 0u64;
    let slow = run_profile(&rig, &log, &monitor, 8_192, &mut published);
    let fast = run_profile(&rig, &log, &monitor, 819_200, &mut published);

    // The isolated side saw every segment, unaltered (after the held one it
    // was primed with).
    let received = rig.trains.lock().unwrap()[1..].to_vec();
    let rates: Vec<u32> = received.iter().map(|t| t.freq_hz).collect();
    assert_eq!(
        rates,
        vec![8_192, 16_384, 8_192, 0, 819_200, 1_638_400, 819_200, 0],
        "every rate change crossed the barrier, and only rate changes did"
    );
    assert_eq!(
        received.last().expect("segments arrived").emitted,
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
    assert_eq!(
        fast.1, RELAY_EVENT_CEILING,
        "engine events must not scale with the step rate: {} events for {} pulses \
         (the measurement is {RELAY_EVENT_CEILING})",
        fast.1, fast.0
    );
    println!(
        "[budget] {} pulses crossed IC14 in {} engine events; {} pulses in {} events \
         (a hundredfold rate change, identical cost). One engine event per STEP edge \
         would have been at least {}",
        slow.0, slow.1, fast.0, fast.1, fast.0
    );
    rig.finish();
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
    settle();
    assert_eq!(
        state(&rig.system, &format!("{EDGE}.Net-(IC14-OUTA)")),
        NetState::Driven(Level::Low)
    );
    let settled = monitor.drive_count();

    rig.drive("STEP", 3.3);
    settle();
    assert_eq!(
        state(&rig.system, &format!("{EDGE}.Net-(IC14-OUTA)")),
        NetState::Driven(Level::High)
    );
    assert_eq!(
        monitor.drive_count(),
        settled + 1,
        "one transition on one channel is one drive across the whole part"
    );

    // Re-driving the same level, ten times over, costs nothing at all —
    // read after a settle that delivered all ten, not after a wall sleep.
    for _ in 0..10 {
        rig.drive("STEP", 3.3);
    }
    settle();
    assert_eq!(monitor.drive_count(), settled + 1);
    println!(
        "[budget] {} drives for a settled four-channel isolator plus one transition",
        monitor.drive_count()
    );
    rig.finish();
}
