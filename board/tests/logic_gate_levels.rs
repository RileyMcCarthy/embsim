//! A logic gate on the net, live: the output changes exactly the datasheet
//! propagation delay after the input, driven through the datasheet output
//! resistance, and a Schmitt-trigger input does not flip inside its
//! hysteresis band — `NODES.md` §8 phase 2's proof for `LogicGate`.
//!
//! The rig is a bench: a driver pin, the gate, a probe on both of its nets
//! that stamps every state it is delivered with the virtual instant it
//! arrived. Stepped mode (`TESTING.md` rule 9) is what makes the instant
//! exact: the engine advances *to* the gate's wake, so a state delivered at
//! that wake carries that instant. Its own binary per rule 5.

use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};

use embsim_board::{
    digital_drive, AttachError, Component, ComponentNetIo, Drive, EndpointRef, Harness, IdleDrive,
    Level, NetState, PinDecl, PinHandle, PinKind, System, TheveninDrive,
};
use embsim_core::virtual_clock::{self, ClockMode};
use embsim_models::logic_gate::{
    self, GatePin, LogicGate, LogicGateMonitor, LVC1G14_PINS_SOT23, LVC1G14_R_OH_OHMS,
    LVC1G14_R_OL_OHMS, LVC1G14_T_PD_NS, LVC2G04_PINS_SOT363, LVC2G04_T_PD_NS,
};
use rstest::rstest;
use vibes_behaviour::{behaviour, expect, Test};

// ============================================================
// Plumbing
// ============================================================

static SUITE_LOCK: Mutex<()> = Mutex::new(());

fn suite_lock() -> MutexGuard<'static, ()> {
    SUITE_LOCK.lock().unwrap_or_else(|poisoned| {
        SUITE_LOCK.clear_poison();
        poisoned.into_inner()
    })
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

fn ep(endpoint: &str) -> EndpointRef {
    EndpointRef::parse(endpoint).expect("endpoint parses")
}

/// A pin the test drives from its own thread.
struct Driver {
    pins: [PinDecl; 1],
    handle: Arc<Mutex<Option<PinHandle>>>,
}

impl Component for Driver {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        *self.handle.lock().unwrap() = Some(io.pin("Q")?);
        Ok(())
    }
}

/// Every state a net took, with the virtual nanosecond it was delivered.
type Stamped = Arc<Mutex<Vec<(u64, NetState)>>>;

/// Two sense pins stamping their deliveries: `A` on the gate's input net,
/// `Y` on its output net.
struct Probe {
    pins: [PinDecl; 2],
    a: Stamped,
    y: Stamped,
}

impl Component for Probe {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        for (pin, log) in [("A", Arc::clone(&self.a)), ("Y", Arc::clone(&self.y))] {
            io.on_sense(pin, move |state| {
                log.lock()
                    .unwrap()
                    .push((virtual_clock::virtual_ns(), state));
            })?;
        }
        Ok(())
    }
}

/// The last state a log holds, and the instant it arrived.
fn last(log: &Stamped) -> Option<(u64, NetState)> {
    log.lock().unwrap().last().copied()
}

/// One gate on the bench: the driver on its input, the probe on both nets,
/// its supply at 3.3 V and its ground at 0 V. Returns the running system,
/// the driver's handle, the probe logs and the gate's monitor.
struct Bench {
    system: embsim_board::SystemHandle,
    q: PinHandle,
    a: Stamped,
    y: Stamped,
    gate: LogicGateMonitor,
}

fn bench(config: logic_gate::Config, pins: &'static [GatePin], input: &str, output: &str) -> Bench {
    virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);
    let gate = LogicGate::new(config, pins).expect("valid");
    let monitor = gate.monitor();
    let handle = Arc::new(Mutex::new(None));
    let a: Stamped = Arc::new(Mutex::new(Vec::new()));
    let y: Stamped = Arc::new(Mutex::new(Vec::new()));
    let sense = |number| PinDecl {
        number,
        name: None,
        kind: PinKind::DigitalIn,
        stream: None,
        drive_impedance: None,
        idle: IdleDrive::KindDefault,
    };
    let (vcc, gnd) = {
        let find = |name: &str| {
            pins.iter()
                .find(|p| p.name == Some(name) || p.number == name)
                .map(|p| p.number)
                .expect("the pin table names it")
        };
        (find("VCC"), find("GND"))
    };
    let system = System::new()
        .component(
            "DRV",
            Box::new(Driver {
                pins: [PinDecl {
                    number: "Q",
                    name: None,
                    kind: PinKind::DigitalOut,
                    stream: None,
                    drive_impedance: None,
                    idle: IdleDrive::Released,
                }],
                handle: Arc::clone(&handle),
            }),
        )
        .component("GATE", Box::new(gate))
        .component(
            "PRB",
            Box::new(Probe {
                pins: [sense("A"), sense("Y")],
                a: Arc::clone(&a),
                y: Arc::clone(&y),
            }),
        )
        .harness(
            Harness::new()
                .connect(ep("DRV.Q"), ep(&format!("GATE.{input}")))
                .connect(ep("PRB.A"), ep(&format!("GATE.{input}")))
                .connect(ep("PRB.Y"), ep(&format!("GATE.{output}")))
                .power(ep("BENCH.3V3"), ep(&format!("GATE.{vcc}")), 3.3)
                .power(ep("BENCH.GND"), ep(&format!("GATE.{gnd}")), 0.0),
        )
        .start()
        .expect("the bench starts");
    assert!(
        wait_for(|| handle.lock().unwrap().is_some(), SETTLE),
        "the driver is wired"
    );
    let q = handle.lock().unwrap().clone().unwrap();
    Bench {
        system,
        q,
        a,
        y,
        gate: monitor,
    }
}

// ============================================================
// Propagation delay and output resistance
// ============================================================

/// An SN74LVC1G14 on the bench: its input driven high then low, its output
/// read with the instant of every change.
#[rstest]
fn an_inverter_toggles_its_output_t_pd_after_the_input_edge() {
    behaviour!(Test {
        id: "logic-gate.t-pd-instant",
        covers: Some("models/src/logic_gate.rs#LogicGate"),
        given: "an SN74LVC1G14 Schmitt inverter on a 3.3 V bench whose input is driven high \
                and then low",
    });
    expect!(
        "inverted-after-t-pd",
        "the output goes low exactly 5 nanoseconds after the input went high, and high \
         exactly 5 nanoseconds after the input went low",
        "the datasheet's maximum propagation delay at 3.3 V is 4.6 ns, which the engine \
         schedules as a 5 ns instant; the output never moves in the same instant as the \
         input"
    );
    expect!(
        "datasheet-output-resistance",
        "the low output is driven through 22.9 ohms and the high output through 29.2 ohms",
        "the datasheet's worst-case output voltages at 24 milliamps, low and high, are the \
         resistances a load sees"
    );

    let _lock = suite_lock();
    let b = bench(logic_gate::Config::lvc1g14(), &LVC1G14_PINS_SOT23, "2", "4");

    b.q.drive(Drive::Thevenin(digital_drive(Level::High)));
    assert!(
        wait_for(
            || matches!(last(&b.y), Some((_, NetState::Driven(Level::Low)))),
            SETTLE
        ),
        "Y goes low; y={:?} findings={:?}",
        b.y.lock().unwrap(),
        b.system.findings()
    );
    let (t_a, _) =
        b.a.lock()
            .unwrap()
            .iter()
            .rev()
            .find(|(_, s)| *s == NetState::Driven(Level::High))
            .copied()
            .expect("the input edge was delivered");
    let (t_y, _) = last(&b.y).unwrap();
    assert_eq!(t_y, t_a + LVC1G14_T_PD_NS, "t_pd, as an instant");
    assert_eq!(
        b.gate.output_drive(0),
        Some(TheveninDrive {
            volts: 0.0,
            impedance: LVC1G14_R_OL_OHMS
        })
    );

    b.q.drive(Drive::Thevenin(digital_drive(Level::Low)));
    assert!(
        wait_for(
            || matches!(last(&b.y), Some((_, NetState::Driven(Level::High)))),
            SETTLE
        ),
        "Y goes high"
    );
    let (t_a, _) =
        b.a.lock()
            .unwrap()
            .iter()
            .rev()
            .find(|(_, s)| *s == NetState::Driven(Level::Low))
            .copied()
            .unwrap();
    let (t_y, _) = last(&b.y).unwrap();
    assert_eq!(t_y, t_a + LVC1G14_T_PD_NS);
    assert_eq!(
        b.gate.output_drive(0),
        Some(TheveninDrive {
            volts: 3.3,
            impedance: LVC1G14_R_OH_OHMS
        })
    );
    assert_eq!(b.gate.drive_count(0), 2, "one drive per input edge");
    drop(b.system);
}

// ============================================================
// Hysteresis
// ============================================================

/// The input taken high, then to a voltage inside the band, then below it.
/// The band is the datasheet's: for the SN74LVC1G14 `V_T−` min 0.84 V to
/// `V_T+` max 1.87 V; for the 74LVC2G04 `V_IL` max 0.8 V to `V_IH` min
/// 2.0 V.
#[rstest]
#[case::lvc1g14(logic_gate::Config::lvc1g14(), &LVC1G14_PINS_SOT23, "2", "4", 1.3, 0.5, LVC1G14_T_PD_NS)]
#[case::lvc2g04(logic_gate::Config::lvc2g04(), &LVC2G04_PINS_SOT363, "1", "6", 1.5, 0.5, LVC2G04_T_PD_NS)]
fn a_schmitt_input_does_not_flip_inside_its_hysteresis_band(
    #[case] config: logic_gate::Config,
    #[case] pins: &'static [GatePin],
    #[case] input: &str,
    #[case] output: &str,
    #[case] inside: f64,
    #[case] below: f64,
    #[case] t_pd: u64,
) {
    behaviour!(Test {
        id: "logic-gate.hysteresis-band",
        covers: Some("models/src/logic_gate.rs#LogicGate"),
        given: "an inverter on a 3.3 V bench whose input is driven to the rail, then to a \
                voltage inside its threshold band, then below its low threshold",
    });
    expect!(
        "holds-inside-the-band",
        "the output stays low while the input sits inside the band",
        "a Schmitt input keeps the level it last recognised until the input crosses the \
         opposite threshold"
    );
    expect!(
        "flips-below-the-band",
        "the output goes high one propagation delay after the input drops below the low \
         threshold, and that is its only change after the first",
        "the low threshold is where the datasheet says a falling input is recognised"
    );

    let _lock = suite_lock();
    let b = bench(config, pins, input, output);

    b.q.drive(Drive::Thevenin(digital_drive(Level::High)));
    assert!(
        wait_for(
            || matches!(last(&b.y), Some((_, NetState::Driven(Level::Low)))),
            SETTLE
        ),
        "Y low after a high input"
    );

    b.q.drive(Drive::Thevenin(TheveninDrive {
        volts: inside,
        impedance: 25.0,
    }));
    assert!(
        wait_for(
            || matches!(last(&b.a), Some((_, NetState::Analog(v))) if v == inside),
            SETTLE
        ),
        "the input rests inside the band: {:?}",
        b.a.lock().unwrap()
    );
    // The gate's propagation delay has long passed when the next stimulus
    // lands: whatever it would have done inside the band, it has done.
    b.q.drive(Drive::Thevenin(TheveninDrive {
        volts: below,
        impedance: 25.0,
    }));
    assert!(
        wait_for(
            || matches!(last(&b.y), Some((_, NetState::Driven(Level::High)))),
            SETTLE
        ),
        "Y high after the input dropped below the band"
    );

    let mut y: Vec<NetState> = b.y.lock().unwrap().iter().map(|(_, s)| *s).collect();
    y.dedup();
    assert_eq!(
        y,
        vec![
            NetState::Floating,
            NetState::Driven(Level::Low),
            NetState::Driven(Level::High)
        ],
        "released, low, high — nothing in between"
    );
    assert_eq!(b.gate.drive_count(0), 2);
    // The input below the band is a valid low, which the engine projects as
    // one: the entry after the mid-band voltage.
    let a = b.a.lock().unwrap();
    let inside_at = a
        .iter()
        .position(|(_, s)| matches!(s, NetState::Analog(v) if *v == inside))
        .expect("the mid-band voltage was delivered");
    let (t_a, below_state) = a[inside_at + 1..]
        .iter()
        .find(|(_, s)| !matches!(s, NetState::Analog(v) if *v == inside))
        .copied()
        .expect("the drop below the band was delivered");
    assert_eq!(
        below_state,
        NetState::Driven(Level::Low),
        "{below} V is a valid low"
    );
    let (t_y, _) = last(&b.y).unwrap();
    assert_eq!(t_y, t_a + t_pd);
    drop(a);
    drop(b.system);
}
