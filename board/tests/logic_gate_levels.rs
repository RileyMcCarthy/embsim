//! A logic gate on the net, live: the output changes exactly the datasheet
//! propagation delay after the input, driven through the datasheet output
//! resistance, and a Schmitt-trigger input does not flip inside its
//! hysteresis band — `NODES.md` §8 phase 2's proof for `LogicGate`. And a
//! clock driven straight onto a plain input is relayed only when its phases
//! cross the input's thresholds (`NODES.md` §12 item 5, the cleanup).
//!
//! The rig is a bench: a driver pin, the gate, a probe on both of its nets
//! that stamps every state it is delivered with the virtual instant it
//! arrived. Stepped mode (`TESTING.md` rule 9) is what makes the instant
//! exact: the engine advances *to* the gate's wake, so a state delivered at
//! that wake carries that instant. Its own binary per rule 5.

use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};

use embsim_board::{
    digital_drive, jesd8c01_lvcmos_thresholds, AttachError, Component, ComponentNetIo, DeadBand,
    Drive, EndpointRef, Harness, Level, NetState, PeriodicSchedule, PinDecl, PinHandle, System,
    TheveninDrive,
};
use embsim_core::virtual_clock::{self, ClockMode};
use embsim_models::logic_gate::{
    self, GatePin, LogicGate, LogicGateMonitor, Mode, LVC1G14_PINS_SOT23, LVC1G14_R_OH_OHMS,
    LVC1G14_R_OL_OHMS, LVC1G14_T_PD_NS, LVC2G04_PINS_SOT363, LVC2G04_R_OH_OHMS, LVC2G04_R_OL_OHMS,
    LVC2G04_T_PD_NS,
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
            io.on_net_report(pin, move |state| {
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
    let sense = |number| PinDecl::digital_in(number, jesd8c01_lvcmos_thresholds(DeadBand::Unknown));
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
                pins: [PinDecl::digital_out("Q").with_idle(None)],
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
/// `V_T+` max 1.87 V, where the Schmitt input keeps its state (its
/// declared dead-band policy, `HoldLast`). The 74LVC2G04 has no such band
/// to hold in: [`a_plain_input_reads_no_level_inside_its_band`].
#[rstest]
#[case::lvc1g14(logic_gate::Config::lvc1g14(), &LVC1G14_PINS_SOT23, "2", "4", 1.3, 0.5, LVC1G14_T_PD_NS)]
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

/// The 74LVC2G04's input — `V_IL` max 0.8 V, `V_IH` min 2.0 V (Table 7),
/// no hysteresis named — taken high, then to 1.5 V, then below `V_IL`.
/// Between its two figures the datasheet guarantees neither level, so the
/// input reads none (its declared dead-band policy, `Unknown`) and the
/// gate's answer for an input with no level applies: the output is
/// released, one propagation delay later.
#[rstest]
fn a_plain_input_reads_no_level_inside_its_band() {
    behaviour!(Test {
        id: "logic-gate.plain-input-dead-band",
        covers: Some("models/src/logic_gate.rs#LogicGate"),
        given: "a 74LVC2G04 inverter on a 3.3 V bench, its input driven to the rail, then to \
                1.5 volts between its input thresholds, then below them",
    });
    expect!(
        "released-inside-the-band",
        "the output is released one propagation delay after the input enters the band",
        "the datasheet guarantees neither level between the two thresholds and names no \
         hysteresis, so the input reads no level, and the gate drives nothing from an input \
         with no level"
    );
    expect!(
        "high-below-the-band",
        "the output goes high one propagation delay after the input drops below the low \
         threshold"
    );

    let _lock = suite_lock();
    let b = bench(
        logic_gate::Config::lvc2g04(),
        &LVC2G04_PINS_SOT363,
        "1",
        "6",
    );
    let inside = 1.5;

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
            || matches!(last(&b.y), Some((_, NetState::Floating))),
            SETTLE
        ),
        "Y released inside the band: {:?}",
        b.y.lock().unwrap()
    );
    let (t_inside, _) =
        *b.a.lock()
            .unwrap()
            .iter()
            .find(|(_, s)| matches!(s, NetState::Analog(v) if *v == inside))
            .expect("the mid-band voltage was delivered");
    let (t_released, _) = last(&b.y).unwrap();
    assert_eq!(t_released, t_inside + LVC2G04_T_PD_NS);

    b.q.drive(Drive::Thevenin(TheveninDrive {
        volts: 0.5,
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
            NetState::Floating,
            NetState::Driven(Level::High)
        ],
        "released, low, released, high"
    );
    assert_eq!(b.gate.drive_count(0), 3, "one drive per input change");
    drop(b.system);
}

// ============================================================
// A clock on a plain input
// ============================================================

/// A square wave the bench drives straight onto the input — no capacitor
/// between — from 0 V to `high_volts`, both phases behind 25 Ω, at 1 MHz
/// from `since_ns`.
fn square_wave(high_volts: f64, since_ns: u64) -> Drive {
    Drive::Periodic {
        hi: TheveninDrive {
            volts: high_volts,
            impedance: 25.0,
        },
        lo: TheveninDrive {
            volts: 0.0,
            impedance: 25.0,
        },
        segment: PeriodicSchedule {
            emitted: 0,
            freq_hz: 1_000_000,
            total: None,
            since_ns,
        },
    }
}

/// The 74LVC2G04's input — `V_IL` max 0.8 V, `V_IH` min 2.0 V (Table 7),
/// no level between — first held high, then driven by a 0 V / 1.2 V square
/// wave, then by a 0 V / 3.3 V one. A receiver sees a clock only when its
/// input crosses its switching point every cycle: the 1.2 V phase sits in
/// the band, where the input reads no level, so the wave is no clock and no
/// level there — the output is released, as for any input with no level;
/// the 3.3 V wave crosses both thresholds and is relayed at its own rate.
/// Nothing on the bench joins the output back to the input, so the input is
/// not self-biased.
#[rstest]
fn a_clock_on_a_plain_input_is_relayed_only_when_it_crosses_the_thresholds() {
    behaviour!(Test {
        id: "logic-gate.clock-must-cross-the-input",
        covers: Some("models/src/logic_gate.rs#LogicGate"),
        given: "a 74LVC2G04 inverter on a 3.3 volt bench whose input, first held high, is \
                driven directly by a square wave from 0 to 1.2 volts, then from 0 to 3.3 volts",
    });
    expect!(
        "low-swing-released",
        "while the 1.2 volt wave runs, the output rests released",
        "1.2 volts sits between the input's 0.8 and 2.0 volt thresholds, where the datasheet \
         guarantees no level, so that phase reads none and the input sees no edge"
    );
    expect!(
        "full-swing-relayed",
        "the 3.3 volt wave is relayed at its own rate between the part's own output levels",
        "each phase crosses a threshold, so the input switches every cycle"
    );

    let _lock = suite_lock();
    let b = bench(
        logic_gate::Config::lvc2g04(),
        &LVC2G04_PINS_SOT363,
        "1",
        "6",
    );
    assert!(!b.gate.self_biased(0), "nothing joins 1Y back to 1A");

    b.q.drive(Drive::Thevenin(digital_drive(Level::High)));
    assert!(
        wait_for(
            || matches!(last(&b.y), Some((_, NetState::Driven(Level::Low)))),
            SETTLE
        ),
        "Y low after a high input"
    );

    let low_swing = square_wave(1.2, virtual_clock::virtual_ns());
    b.q.drive(low_swing);
    assert!(
        wait_for(
            || matches!(last(&b.y), Some((_, NetState::Floating))),
            SETTLE
        ),
        "Y released under the 1.2 V wave: y={:?} a={:?}",
        b.y.lock().unwrap(),
        b.a.lock().unwrap()
    );
    assert!(
        matches!(last(&b.a), Some((_, NetState::Periodic { .. }))),
        "the input carries the wave: {:?}",
        last(&b.a)
    );
    assert_eq!(b.gate.mode(0), Mode::Level);
    assert_eq!(b.gate.relayed_segment(0), None);
    assert_eq!(b.gate.train_count(0), 0, "no rate relayed");

    let full_swing = square_wave(3.3, virtual_clock::virtual_ns());
    let Drive::Periodic { segment, .. } = full_swing else {
        unreachable!()
    };
    b.q.drive(full_swing);
    assert!(
        wait_for(|| b.gate.relayed_segment(0) == Some(segment), SETTLE),
        "the 3.3 V wave is relayed: y={:?}",
        b.y.lock().unwrap()
    );
    assert_eq!(b.gate.mode(0), Mode::Rate);
    assert_eq!(
        b.gate.output(0),
        Some(Drive::Periodic {
            hi: TheveninDrive {
                volts: 3.3,
                impedance: LVC2G04_R_OH_OHMS,
            },
            lo: TheveninDrive {
                volts: 0.0,
                impedance: LVC2G04_R_OL_OHMS,
            },
            segment,
        }),
        "between the datasheet's own output ports"
    );
    assert_eq!(b.gate.train_count(0), 1, "one relayed rate");
    drop(b.system);
}
