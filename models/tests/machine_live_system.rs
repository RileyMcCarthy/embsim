//! Live-system tests for [`embsim_models::machine`]: the machine components
//! wired to a fake MCU-ish peer through a real [`System`], so drives, senses,
//! and the timer wheel all go through the net engine.
//!
//! The peer ([`FakeMcu`]) is deliberately dumb: it drives `STEP`/`DIR`/`ENA`
//! from the test thread through shared [`PinHandle`]s (a stand-in for a
//! component's own protocol thread) and logs every `A`/`B`/`END` sense
//! delivery in engine order. Nothing here links firmware.
//!
//! Per `TESTING.md`, timing-dependent quantities are asserted as contracts —
//! edge counts, orderings, directions, and resolved net states — never as
//! wall-clock magnitudes. The exact plant arithmetic is unit-tested against
//! injected timestamps inside `embsim-models` itself.

use std::collections::HashMap;
use std::sync::{Arc, Mutex, MutexGuard, Once};
use std::time::{Duration, Instant};

use rstest::rstest;

use embsim_board::{
    AnalogBackend, AttachError, Board, Component, ComponentNetIo, EndpointRef, Finding, Harness,
    Level, NetState, PartRegistry, PinDecl, PinHandle, PinKind, SenseKind, System, SystemHandle,
    TheveninDrive,
};
use embsim_models::machine::{end_switch, quadrature_encoder, stepper_motor};
use embsim_models::machine::{
    ActuationSense, BounceConfig, ChannelOrder, EndSwitch, EndSwitchActuator, MotorShaft,
    QuadratureEncoder, StepperMotor,
};

// ============================================================
// Shared fixtures
// ============================================================

/// The timer wheel and every read-time plant sample need the virtual clock,
/// and `init` re-anchors virtual time to zero — so it runs exactly once for
/// this whole test binary, never per test.
fn ensure_clock() {
    static CLOCK: Once = Once::new();
    CLOCK.call_once(|| embsim_core::virtual_clock::init(1.0, 1_000_000));
}

/// The virtual clock and libngspice session are process-global. Parallel
/// live `System`s in this binary steal time-authority from each other
/// (TESTING.md rule 5). Hold this for the whole test, including `System` drop.
static CLOCK_LOCK: Mutex<()> = Mutex::new(());

fn lock_clock() -> MutexGuard<'static, ()> {
    CLOCK_LOCK.lock().unwrap_or_else(|p| {
        CLOCK_LOCK.clear_poison();
        p.into_inner()
    })
}

/// These cases are digital (Gray code, bounce, step/dir). Analog Off keeps
/// them off the process-global ngspice session.
fn digital_system() -> System {
    System::new().analog(AnalogBackend::Off)
}

const HIGH: TheveninDrive = TheveninDrive {
    volts: 3.3,
    impedance: 25.0,
};
const LOW: TheveninDrive = TheveninDrive {
    volts: 0.0,
    impedance: 25.0,
};

/// One logged sense delivery: which pin, and the state the engine published.
type SenseLog = Arc<Mutex<Vec<(&'static str, NetState)>>>;

/// A fake MCU-ish peer: outputs it drives from the test thread, inputs it
/// logs. `outputs` are shared out as [`PinHandle`]s at attach; every `inputs`
/// pin's sense deliveries land in one shared log, so cross-pin ordering is the
/// engine's authoritative order.
struct FakeMcu {
    pins: Vec<PinDecl>,
    inputs: Vec<&'static str>,
    outputs: Vec<&'static str>,
    handles: Arc<Mutex<HashMap<&'static str, PinHandle>>>,
    log: SenseLog,
}

impl FakeMcu {
    fn new(outputs: &[&'static str], inputs: &[&'static str]) -> Self {
        let pins = outputs
            .iter()
            .map(|number| pin(number, PinKind::DigitalOut))
            .chain(inputs.iter().map(|number| pin(number, PinKind::DigitalIn)))
            .collect();
        Self {
            pins,
            inputs: inputs.to_vec(),
            outputs: outputs.to_vec(),
            handles: Arc::new(Mutex::new(HashMap::new())),
            log: Arc::new(Mutex::new(Vec::new())),
        }
    }
}

fn pin(number: &'static str, kind: PinKind) -> PinDecl {
    PinDecl {
        number,
        name: None,
        kind,
        stream: None,
        drive_impedance: None,
    }
}

impl Component for FakeMcu {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        for number in &self.outputs {
            self.handles.lock().unwrap().insert(number, io.pin(number)?);
        }
        for number in &self.inputs {
            let log = Arc::clone(&self.log);
            let number = *number;
            io.on_sense(number, move |state| {
                log.lock().unwrap().push((number, state));
            })?;
        }
        Ok(())
    }
}

/// Poll `pred` until it holds or the timeout expires.
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

/// Emit `count` rising STEP pulses on a shared handle, low then high, with a
/// pause on each level so successive rising edges are a measurable interval
/// apart in virtual time.
fn pulse_step(step: &PinHandle, count: u32) {
    for _ in 0..count {
        step.set_drive(Some(LOW));
        std::thread::sleep(Duration::from_millis(1));
        step.set_drive(Some(HIGH));
        std::thread::sleep(Duration::from_millis(1));
    }
}

fn handle(mcu: &Arc<Mutex<HashMap<&'static str, PinHandle>>>, number: &str) -> PinHandle {
    mcu.lock()
        .unwrap()
        .get(number)
        .cloned()
        .unwrap_or_else(|| panic!("{number} handle shared at attach"))
}

// ============================================================
// Motor + encoder: a coupled machine axis
// ============================================================

/// Build the motor/encoder axis: a fake MCU driving `STEP`/`DIR`/`ENA` and
/// reading the encoder's `A`/`B`, with the motor's shaft feeding the encoder
/// exactly as a consumer's system description would wire it.
struct Axis {
    system: SystemHandle,
    handles: Arc<Mutex<HashMap<&'static str, PinHandle>>>,
    log: SenseLog,
    shaft: MotorShaft,
    encoder: quadrature_encoder::EncoderInput,
}

fn build_axis() -> Axis {
    ensure_clock();

    let mcu = FakeMcu::new(&["STEP", "DIR", "ENA"], &["A", "B"]);
    let handles = Arc::clone(&mcu.handles);
    let log = Arc::clone(&mcu.log);

    // One step per count, a lag far shorter than the pulse spacing, no load:
    // the axis reacts within a pulse or two so the live assertions are about
    // direction and coupling, not settling time.
    let motor = StepperMotor::new(stepper_motor::Config {
        tau_s: 0.001,
        load_loss: 0.0,
        observe_interval_us: Some(500),
        ..stepper_motor::Config::new(1.0)
    })
    .expect("valid motor config");
    let shaft = motor.shaft();

    let encoder =
        QuadratureEncoder::new(quadrature_encoder::Config::new(1.0)).expect("valid encoder config");
    let input = encoder.input();

    // The seam under test: the motor publishes millimetres, the encoder
    // applies its own counts/mm.
    {
        let input = input.clone();
        shaft.on_position_change(move |mm| input.set_position_mm(mm));
    }

    let harness = Harness::new()
        .connect_str("MCU.STEP", "MOTOR.STEP")
        .expect("endpoint")
        .connect_str("MCU.DIR", "MOTOR.DIR")
        .expect("endpoint")
        .connect_str("MCU.ENA", "MOTOR.ENA")
        .expect("endpoint")
        .connect_str("ENC.A", "MCU.A")
        .expect("endpoint")
        .connect_str("ENC.B", "MCU.B")
        .expect("endpoint");

    let system = digital_system()
        .component("MCU", Box::new(mcu))
        .component("MOTOR", Box::new(motor))
        .component("ENC", Box::new(encoder))
        .harness(harness)
        .start()
        .expect("live system starts");

    Axis {
        system,
        handles,
        log,
        shaft,
        encoder: input,
    }
}

/// Wait until the encoder has settled on its count-0 phase (both channels
/// low), then clear the log so a test sees only its own transitions.
fn settle_encoder(axis: &Axis) {
    assert!(
        wait_for(
            || axis.system.net_state("ENC.A") == Some(NetState::Driven(Level::Low))
                && axis.system.net_state("ENC.B") == Some(NetState::Driven(Level::Low)),
            SETTLE
        ),
        "the encoder must drive its count-0 phase at attach; A = {:?} B = {:?}",
        axis.system.net_state("ENC.A"),
        axis.system.net_state("ENC.B")
    );
    axis.log.lock().unwrap().clear();
}

/// Reduce the sense log to the channel-change sequence the peer observed.
fn channel_sequence(log: &SenseLog) -> Vec<(&'static str, Level)> {
    log.lock()
        .unwrap()
        .iter()
        .filter_map(|&(number, state)| match state {
            NetState::Driven(level) => Some((number, level)),
            _ => None,
        })
        .collect()
}

/// The peer sees the full Gray-code sequence, one channel per count, and the
/// order of the changes reverses with the direction of travel — the signature
/// a hardware quadrature counter decodes.
#[rstest]
fn encoder_walks_gray_code_in_order_and_reverses_with_direction() {
    let _clock = lock_clock();
    let axis = build_axis();
    settle_encoder(&axis);

    // Forward eight counts: A changes first (A leads B), then alternating.
    axis.encoder.set_position_counts(8);
    assert!(
        wait_for(|| channel_sequence(&axis.log).len() == 8, SETTLE),
        "eight counts must produce eight channel changes, saw {:?}",
        channel_sequence(&axis.log)
    );
    assert_eq!(
        channel_sequence(&axis.log),
        vec![
            ("A", Level::High),
            ("B", Level::High),
            ("A", Level::Low),
            ("B", Level::Low),
            ("A", Level::High),
            ("B", Level::High),
            ("A", Level::Low),
            ("B", Level::Low),
        ],
        "forward travel: A leads B"
    );
    assert_eq!(axis.encoder.count(), 8);
    assert_eq!(axis.encoder.snapped_updates(), 0);

    // Back to zero: the same states in the mirrored order, so B changes first.
    axis.log.lock().unwrap().clear();
    axis.encoder.set_position_counts(0);
    assert!(
        wait_for(|| channel_sequence(&axis.log).len() == 8, SETTLE),
        "eight counts back must produce eight channel changes, saw {:?}",
        channel_sequence(&axis.log)
    );
    assert_eq!(
        channel_sequence(&axis.log),
        vec![
            ("B", Level::High),
            ("A", Level::High),
            ("B", Level::Low),
            ("A", Level::Low),
            ("B", Level::High),
            ("A", Level::High),
            ("B", Level::Low),
            ("A", Level::Low),
        ],
        "reverse travel: B leads A"
    );
    assert_eq!(axis.encoder.count(), 0);
}

/// Exactly one channel changes per count, in both directions — the Gray
/// property, asserted on what the peer actually observed rather than on the
/// phase table.
#[rstest]
fn peer_never_observes_two_channels_changing_at_once() {
    let _clock = lock_clock();
    let axis = build_axis();
    settle_encoder(&axis);

    axis.encoder.set_position_counts(12);
    assert!(wait_for(|| channel_sequence(&axis.log).len() == 12, SETTLE));
    axis.encoder.set_position_counts(-12);
    assert!(wait_for(|| channel_sequence(&axis.log).len() == 36, SETTLE));

    let mut state = (Level::Low, Level::Low);
    for (index, (channel, level)) in channel_sequence(&axis.log).into_iter().enumerate() {
        let next = match channel {
            "A" => (level, state.1),
            _ => (state.0, level),
        };
        let changes = u8::from(next.0 != state.0) + u8::from(next.1 != state.1);
        assert_eq!(
            changes, 1,
            "delivery {index} changed {changes} channels ({channel} -> {level:?})"
        );
        state = next;
    }
}

/// The whole axis, driven the way firmware drives it: rising STEP edges with
/// `DIR` selecting travel. Edge counting is exact; travel is asserted
/// directionally (the plant's arithmetic is unit-tested against injected
/// timestamps, not against test-thread sleeps).
#[rstest]
fn step_pulses_move_the_axis_and_dir_reverses_it() {
    let _clock = lock_clock();
    let axis = build_axis();
    settle_encoder(&axis);
    let step = handle(&axis.handles, "STEP");
    let dir = handle(&axis.handles, "DIR");

    // The peer's ENA output idles high, and the drive enables active-high.
    assert!(
        wait_for(|| axis.shaft.enabled(), SETTLE),
        "the drive must see its enable through the harness"
    );
    assert!(axis.shaft.forward(), "DIR idles high = forward");

    pulse_step(&step, 20);
    assert!(
        wait_for(|| axis.shaft.commanded_steps() == 20, SETTLE),
        "every rising edge must be counted, saw {}",
        axis.shaft.commanded_steps()
    );
    let peak_mm = axis.shaft.position_mm();
    assert!(
        peak_mm > 0.0,
        "forward pulses must move the carriage forward, got {peak_mm}"
    );
    assert!(
        wait_for(|| axis.encoder.count() > 0, SETTLE),
        "the coupled encoder must follow the shaft, count = {}",
        axis.encoder.count()
    );
    let peak_count = axis.encoder.count();

    // Reverse: same pulses, opposite direction.
    dir.set_drive(Some(LOW));
    assert!(
        wait_for(|| !axis.shaft.forward(), SETTLE),
        "the drive must latch the new direction"
    );
    pulse_step(&step, 20);
    assert!(
        wait_for(|| axis.shaft.commanded_steps() == 0, SETTLE),
        "20 forward + 20 reverse edges cancel, saw {}",
        axis.shaft.commanded_steps()
    );
    assert!(
        wait_for(|| axis.shaft.position_mm() < peak_mm * 0.5, SETTLE),
        "the carriage must travel back, {peak_mm} -> {}",
        axis.shaft.position_mm()
    );
    assert!(
        wait_for(|| axis.encoder.count() < peak_count, SETTLE),
        "and the encoder must count back down, {peak_count} -> {}",
        axis.encoder.count()
    );
}

/// A drive whose ENA never presents a level ignores the step train: the
/// engine reports the floating enable, and the component chooses the
/// safe-machine reading.
#[rstest]
fn an_unwired_enable_leaves_the_drive_disabled() {
    let _clock = lock_clock();
    ensure_clock();

    let mcu = FakeMcu::new(&["STEP"], &[]);
    let handles = Arc::clone(&mcu.handles);
    let motor = StepperMotor::new(stepper_motor::Config::new(1.0)).expect("valid config");
    let shaft = motor.shaft();

    // STEP is wired; DIR and ENA are left unconnected, so their bench nets
    // have no source at all.
    let system = digital_system()
        .component("MCU", Box::new(mcu))
        .component("MOTOR", Box::new(motor))
        .harness(
            Harness::new()
                .connect_str("MCU.STEP", "MOTOR.STEP")
                .expect("endpoint"),
        )
        .start()
        .expect("live system starts");

    assert_eq!(system.net_state("MOTOR.ENA"), Some(NetState::Floating));
    assert!(!shaft.enabled(), "a floating ENA is not an enable");

    pulse_step(&handle(&handles, "STEP"), 10);
    assert_eq!(
        shaft.commanded_steps(),
        0,
        "a disabled drive must ignore the whole train"
    );
    assert!(shaft.position_counts().abs() < f64::EPSILON);

    assert!(
        system.findings().iter().any(|finding| matches!(
            finding,
            Finding::FloatingSense { net, kind: SenseKind::Digital } if net == "MOTOR.ENA"
        )),
        "the engine must report the floating enable: {:?}",
        system.findings()
    );
}

// ============================================================
// End switch: what the net does when the contact opens
// ============================================================

/// A pull-up board: 10 kΩ from `3V3` to `SENSE`, brought out on a 3-pin
/// connector so a harness can attach a switch and a rail. This is what makes
/// "open = pull-up decides the idle level" a real electrical claim rather
/// than a component-internal convention.
const PULLUP_NETLIST: &str = r#"(export (version "E")
  (components
    (comp (ref "R1")
      (value "10k")
      (libsource (lib "Device") (part "R") (description "")))
    (comp (ref "J1")
      (value "Conn_01x03")
      (libsource (lib "Connector") (part "Conn_01x03") (description ""))))
  (nets
    (net (code "1") (name "3V3") (class "Default")
      (node (ref "R1") (pin "1") (pintype "passive"))
      (node (ref "J1") (pin "1") (pintype "passive")))
    (net (code "2") (name "SENSE") (class "Default")
      (node (ref "R1") (pin "2") (pintype "passive"))
      (node (ref "J1") (pin "2") (pintype "passive")))
    (net (code "3") (name "GND") (class "Default")
      (node (ref "J1") (pin "3") (pintype "passive")))))"#;

/// An upper end stop with 0.5 mm of differential travel.
fn upper_switch() -> end_switch::Config {
    end_switch::Config::new(10.0, ActuationSense::Increasing).with_release(9.5)
}

/// Open contact, no pull-up anywhere: the sense net **floats**, and the engine
/// says so. The switch invents nothing — this is the physically honest result
/// of a dry contact with no bias.
#[rstest]
fn an_open_contact_with_no_pull_up_leaves_the_net_floating() {
    let _clock = lock_clock();
    ensure_clock();

    let mcu = FakeMcu::new(&[], &["END"]);
    let switch = EndSwitch::new(upper_switch()).expect("valid config");
    let actuator = switch.actuator();

    let system = digital_system()
        .component("MCU", Box::new(mcu))
        .component("SW", Box::new(switch))
        .harness(
            Harness::new()
                .connect_str("MCU.END", "SW.NO")
                .expect("endpoint")
                // The board biases the common terminal to ground; the machine
                // side of the switch supplies no level of its own.
                .power(
                    EndpointRef::parse("BENCH.GND").expect("endpoint"),
                    EndpointRef::parse("SW.COM").expect("endpoint"),
                    0.0,
                ),
        )
        .start()
        .expect("live system starts");

    // The engine assigns every DigitalOut an idle-high drive at assembly;
    // attach releases it, so the settled state is Floating.
    assert!(
        wait_for(
            || system.net_state("SW.NO") == Some(NetState::Floating),
            SETTLE
        ),
        "an open contact must contribute nothing, saw {:?}",
        system.net_state("SW.NO")
    );
    assert!(
        system.findings().iter().any(|finding| matches!(
            finding,
            Finding::FloatingSense { net, kind: SenseKind::Digital } if net == "MCU.END"
        )),
        "the peer's sense of an unbiased net must be reported: {:?}",
        system.findings()
    );

    // Closing the contact shorts the sense net to COM's level (ground).
    actuator.set_position_mm(11.0);
    assert!(
        wait_for(
            || system.net_state("SW.NO") == Some(NetState::Driven(Level::Low)),
            SETTLE
        ),
        "a closed contact must pull the sense net to COM, saw {:?}",
        system.net_state("SW.NO")
    );

    // Releasing it floats the net again — the contact is a short, not a driver.
    actuator.set_position_mm(9.0);
    assert!(
        wait_for(
            || system.net_state("SW.NO") == Some(NetState::Floating),
            SETTLE
        ),
        "releasing must return the net to floating, saw {:?}",
        system.net_state("SW.NO")
    );
}

/// The same switch against a real 10 kΩ pull-up: **the board** decides the
/// idle level, not the component. Open resolves `Pulled(High, 10 kΩ)`; closed
/// resolves `Driven(Low)`, because the contact resistance dominates the
/// pull-up by four orders of magnitude.
#[rstest]
fn a_pull_up_decides_the_idle_level_and_the_contact_wins_when_closed() {
    let _clock = lock_clock();
    ensure_clock();

    let board = Board::from_netlist(
        embsim_board::netlist::parse(PULLUP_NETLIST).expect("fixture parses"),
        &PartRegistry::new(),
    )
    .expect("board builds");

    let mcu = FakeMcu::new(&[], &["END"]);
    let switch = EndSwitch::new(upper_switch()).expect("valid config");
    let actuator = switch.actuator();

    let harness = Harness::new()
        .connect_str("Pullup.J1.2", "SW.NO")
        .expect("endpoint")
        .connect_str("Pullup.J1.2", "MCU.END")
        .expect("endpoint")
        .connect_str("Pullup.J1.3", "SW.COM")
        .expect("endpoint")
        .power(
            EndpointRef::parse("BENCH.3V3").expect("endpoint"),
            EndpointRef::parse("Pullup.J1.1").expect("endpoint"),
            3.3,
        )
        .power(
            EndpointRef::parse("BENCH.GND").expect("endpoint"),
            EndpointRef::parse("Pullup.J1.3").expect("endpoint"),
            0.0,
        );

    let system = digital_system()
        .board("Pullup", board)
        .component("MCU", Box::new(mcu))
        .component("SW", Box::new(switch))
        .harness(harness)
        .start()
        .expect("live system starts");

    assert!(
        wait_for(
            || system.net_state("Pullup.SENSE") == Some(NetState::Pulled(Level::High, 10_000.0)),
            SETTLE
        ),
        "an open contact must leave the pull-up in charge, saw {:?}",
        system.net_state("Pullup.SENSE")
    );

    actuator.set_position_mm(11.0);
    assert!(
        wait_for(
            || system.net_state("Pullup.SENSE") == Some(NetState::Driven(Level::Low)),
            SETTLE
        ),
        "a 0.1 Ohm contact must beat a 10 kOhm pull-up, saw {:?}",
        system.net_state("Pullup.SENSE")
    );

    actuator.set_position_mm(9.0);
    assert!(
        wait_for(
            || system.net_state("Pullup.SENSE") == Some(NetState::Pulled(Level::High, 10_000.0)),
            SETTLE
        ),
        "and releasing hands the net back to the pull-up, saw {:?}",
        system.net_state("Pullup.SENSE")
    );
}

/// Actuation hysteresis on a live net: a carriage wobbling inside the
/// differential travel actuates once. The peer's sense log is the assertion
/// target, so this is chatter as the *MCU* would see it.
#[rstest]
fn hysteresis_keeps_a_wobbling_carriage_from_chattering_the_net() {
    let _clock = lock_clock();
    ensure_clock();

    let mcu = FakeMcu::new(&[], &["END"]);
    let log = Arc::clone(&mcu.log);
    let switch = EndSwitch::new(upper_switch()).expect("valid config");
    let actuator = switch.actuator();

    let system = digital_system()
        .component("MCU", Box::new(mcu))
        .component("SW", Box::new(switch))
        .harness(
            Harness::new()
                .connect_str("MCU.END", "SW.NO")
                .expect("endpoint")
                .power(
                    EndpointRef::parse("BENCH.GND").expect("endpoint"),
                    EndpointRef::parse("SW.COM").expect("endpoint"),
                    0.0,
                ),
        )
        .start()
        .expect("live system starts");

    assert!(wait_for(
        || system.net_state("SW.NO") == Some(NetState::Floating),
        SETTLE
    ));
    log.lock().unwrap().clear();

    // Operate at 10.0, release at 9.5: every position below stays actuated
    // once the first crossing happens.
    for position in [9.98, 10.02, 9.9, 10.05, 9.7, 10.01, 9.6] {
        actuator.set_position_mm(position);
        std::thread::sleep(Duration::from_millis(2));
    }

    assert!(actuator.is_closed(), "the switch stays actuated");
    assert!(
        wait_for(
            || system.net_state("SW.NO") == Some(NetState::Driven(Level::Low)),
            SETTLE
        ),
        "and the net stays pulled down"
    );
    let transitions = log.lock().unwrap().len();
    assert_eq!(
        transitions,
        1,
        "differential travel must absorb the wobble; the peer saw {:?}",
        log.lock().unwrap()
    );
}

/// Contact bounce, opt-in: one actuation makes the net chatter a burst of
/// transitions on the engine's timer wheel and then settle closed — the peer
/// sees every one, which is the point (a debounce routine has something to
/// debounce).
#[rstest]
fn contact_bounce_chatters_the_net_and_settles_closed() {
    let _clock = lock_clock();
    ensure_clock();

    let mcu = FakeMcu::new(&[], &["END"]);
    let log = Arc::clone(&mcu.log);
    let switch = EndSwitch::new(upper_switch().with_bounce(BounceConfig {
        transitions: 4,
        window_us: 4_000,
    }))
    .expect("valid config");
    let actuator = switch.actuator();

    let system = digital_system()
        .component("MCU", Box::new(mcu))
        .component("SW", Box::new(switch))
        .harness(
            Harness::new()
                .connect_str("MCU.END", "SW.NO")
                .expect("endpoint")
                .power(
                    EndpointRef::parse("BENCH.GND").expect("endpoint"),
                    EndpointRef::parse("SW.COM").expect("endpoint"),
                    0.0,
                ),
        )
        .start()
        .expect("live system starts");

    assert!(wait_for(
        || system.net_state("SW.NO") == Some(NetState::Floating),
        SETTLE
    ));
    log.lock().unwrap().clear();

    actuator.set_position_mm(11.0);
    assert!(
        wait_for(|| actuator.pending_bounce() == 0, SETTLE),
        "the burst must drain through the timer wheel, {} left",
        actuator.pending_bounce()
    );
    assert!(
        wait_for(
            || system.net_state("SW.NO") == Some(NetState::Driven(Level::Low)),
            SETTLE
        ),
        "and settle closed, saw {:?}",
        system.net_state("SW.NO")
    );
    // `pending_bounce` drops when the wake pops the burst, which is *before*
    // the engine drains the resulting drives. Wait for the sense log too.
    assert!(
        wait_for(|| log.lock().unwrap().len() == 5, SETTLE),
        "five sense deliveries after drives drain, saw {:?}",
        log.lock().unwrap().clone()
    );

    // The immediate actuation edge plus four bounce changes: five deliveries,
    // strictly alternating between driven-low and floating.
    let seen = log.lock().unwrap().clone();
    assert_eq!(
        seen.len(),
        5,
        "one actuation edge + four bounce changes, saw {seen:?}"
    );
    let expected = [
        NetState::Driven(Level::Low),
        NetState::Floating,
        NetState::Driven(Level::Low),
        NetState::Floating,
        NetState::Driven(Level::Low),
    ];
    for (index, (state, expect)) in seen.iter().map(|&(_, s)| s).zip(expected).enumerate() {
        assert_eq!(state, expect, "bounce delivery {index}");
    }
}

// ============================================================
// Facade wiring
// ============================================================

/// A machine component's pins become `"{name}.{pin}"` nets, so a harness
/// addresses them by signal name — the bench-endpoint convention the machine
/// modules document.
#[rstest]
#[case::motor(&["MOTOR.STEP", "MOTOR.DIR", "MOTOR.ENA"])]
#[case::encoder(&["ENC.A", "ENC.B"])]
fn bench_pins_are_addressable_by_signal_name(#[case] names: &[&str]) {
    let _clock = lock_clock();
    let axis = build_axis();
    for name in names {
        assert!(
            axis.system.net_state(name).is_some(),
            "{name} must be a resolvable system net"
        );
    }
    assert_eq!(
        axis.system.component_refs().collect::<Vec<_>>(),
        ["MCU", "MOTOR", "ENC"]
    );
}

/// An index channel changes the facade, and its net resolves like any other
/// encoder output.
#[rstest]
fn a_declared_index_channel_reaches_the_harness() {
    let _clock = lock_clock();
    ensure_clock();

    let mcu = FakeMcu::new(&[], &["A", "B", "Z"]);
    let encoder = QuadratureEncoder::new(
        quadrature_encoder::Config {
            order: ChannelOrder::ALeadsB,
            ..quadrature_encoder::Config::new(1.0)
        }
        .with_index(quadrature_encoder::IndexConfig {
            counts_per_revolution: 8,
            width_counts: 1,
            active_level: Level::High,
        }),
    )
    .expect("valid config");
    let input = encoder.input();

    let system = digital_system()
        .component("MCU", Box::new(mcu))
        .component("ENC", Box::new(encoder))
        .harness(
            Harness::new()
                .connect_str("ENC.A", "MCU.A")
                .expect("endpoint")
                .connect_str("ENC.B", "MCU.B")
                .expect("endpoint")
                .connect_str("ENC.Z", "MCU.Z")
                .expect("endpoint"),
        )
        .start()
        .expect("live system starts");

    // Count 0 is inside the index window.
    assert!(
        wait_for(
            || system.net_state("ENC.Z") == Some(NetState::Driven(Level::High)),
            SETTLE
        ),
        "the index must assert at count 0, saw {:?}",
        system.net_state("ENC.Z")
    );

    // Mid-revolution it releases, and it asserts again one revolution on.
    input.set_position_counts(4);
    assert!(
        wait_for(
            || system.net_state("ENC.Z") == Some(NetState::Driven(Level::Low)),
            SETTLE
        ),
        "and release mid-revolution, saw {:?}",
        system.net_state("ENC.Z")
    );
    input.set_position_counts(8);
    assert!(
        wait_for(
            || system.net_state("ENC.Z") == Some(NetState::Driven(Level::High)),
            SETTLE
        ),
        "and mark the next revolution, saw {:?}",
        system.net_state("ENC.Z")
    );
}

/// The switch's actuator handle is the seam a motor shaft drives, exactly as a
/// consumer's system description wires it: no polling, no timer of its own.
#[rstest]
fn a_motor_shaft_can_actuate_an_end_switch_directly() {
    let _clock = lock_clock();
    ensure_clock();

    let switch = EndSwitch::new(upper_switch()).expect("valid config");
    let actuator: EndSwitchActuator = switch.actuator();
    let shaft = StepperMotor::new(stepper_motor::Config::new(1.0))
        .expect("valid config")
        .shaft();
    {
        let actuator = actuator.clone();
        shaft.on_position_change(move |mm| actuator.set_position_mm(mm));
    }

    // Feeding the observer directly is what the engine's wake would do.
    assert!(!actuator.is_closed());
    actuator.set_position_mm(10.5);
    assert!(actuator.is_closed());
    assert_eq!(actuator.position_mm(), Some(10.5));
}

/// Every machine facade also validates on the **build-time analysis path**
/// (`System::build`), where drives and schedules are inert and senses read the
/// one-shot resolved snapshot. A component that only worked against a live
/// engine would be the build/live divergence the shared resolver exists to
/// prevent.
#[rstest]
fn machine_components_attach_on_the_build_time_analysis_path() {
    let _clock = lock_clock();
    let motor = StepperMotor::new(stepper_motor::Config::new(8_192.0)).expect("valid config");
    let encoder =
        QuadratureEncoder::new(quadrature_encoder::Config::new(8_192.0)).expect("valid config");
    let switch = EndSwitch::new(upper_switch()).expect("valid config");

    let built = digital_system()
        .component("MCU", Box::new(FakeMcu::new(&[], &["END"])))
        .component("MOTOR", Box::new(motor))
        .component("ENC", Box::new(encoder))
        .component("SW", Box::new(switch))
        .harness(
            Harness::new()
                .connect_str("MCU.END", "SW.NO")
                .expect("endpoint"),
        )
        .build()
        .expect("the analysis pass must validate every machine facade");

    for name in [
        "MOTOR.STEP",
        "MOTOR.DIR",
        "MOTOR.ENA",
        "ENC.A",
        "ENC.B",
        "SW.COM",
        "SW.NO",
    ] {
        assert!(
            built.nets().iter().any(|net| net.name == name),
            "{name} must be a resolvable system net"
        );
    }

    // Nothing drives the drive's inputs in this description, and the analysis
    // pass says so — before any traffic, exactly as it would for a chip.
    for name in ["MOTOR.STEP", "MOTOR.DIR", "MOTOR.ENA", "SW.COM"] {
        assert!(
            built
                .diagnostics()
                .findings()
                .iter()
                .any(|finding| matches!(
                    finding,
                    Finding::FloatingSense { net, kind: SenseKind::Digital } if net == name
                )),
            "the analysis must report {name} as an unbiased sense: {:?}",
            built.diagnostics().findings()
        );
    }
}
