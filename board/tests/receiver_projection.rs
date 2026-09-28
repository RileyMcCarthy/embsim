//! The receiver's own projection, live: what a sensing pin is handed is a
//! voltage against its declared reference — [`Sense`] — and the digital
//! level is the receiver's projection of it through its own thresholds, its
//! hysteresis chosen by the level it last read, and its declared policy for
//! the dead band (`NODES.md` §10, "Delivered to a sensing pin"; §11 `Sense`;
//! §12 item 5, the sense task).
//!
//! The rig is a bench: a driver the test sets to a voltage, and receivers on
//! its net that record every [`Sense`] they are handed with the level they
//! project it to — the SN74LVC1G14's Schmitt figures under both dead-band
//! policies, the JESD8C.01 3.3 V LVCMOS pair, and a P2 pad's 0.3/0.7 of a
//! 1.8 V bank — plus an analog reader measured against a reference pin held
//! at 1.0 V, a digital sense nothing drives, and a receiver on a net two
//! drivers fight over. And a clock is the receiver's too: a stepper drive
//! counts a square wave on its `STEP` input only when the wave's phases
//! cross that input's thresholds, and each pulse of a segment once when it
//! stops crossing and crosses again; and a build's senses carry no instant.
//! Stepped mode (`TESTING.md` rule 9): every
//! delivery carries the virtual instant the engine delivered it at. Its own
//! binary per rule 5.

use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};

use embsim_board::{
    jesd8c01_lvcmos_thresholds, AttachError, Component, ComponentNetIo, DeadBand, DigitalReceiver,
    Drive, EndpointRef, Finding, Harness, Level, NetState, PeriodicSchedule, PinDecl, PinHandle,
    Sense, SenseKind, System, SystemHandle, TheveninDrive, Thresholds, Volts,
};
use embsim_core::virtual_clock::{self, ClockMode};
use embsim_models::machine::{stepper_motor, StepperMotor};
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

/// The SN74LVC1G14's input at `V_CC` = 3 V (TI SN74LVC1G14 datasheet §5.5):
/// `V_T−` min 0.84 V, `V_T+` max 1.87 V, `ΔV_T` min 0.56 V — the figures
/// `embsim_models::logic_gate` cites — under the declared `policy`.
const fn schmitt(policy: DeadBand) -> Thresholds {
    Thresholds::new(0.84, 1.87, 0.56, policy)
}

/// A P2 pad's input threshold: 0.3/0.7 of its bank's supply (P2X8C4M64P
/// datasheet, DC Characteristics, p. 47), no hysteresis, no level between
/// (`embsim_boards::p2::P2_PAD_THRESHOLDS`).
const P2_PAD: Thresholds = Thresholds::new(0.3, 0.7, 0.0, DeadBand::Unknown);

/// The bank supply the pad receiver reads against: 1.8 V.
const BANK_VOLTS: Volts = 1.8;

/// The voltage the reference receiver's reference pin is held at.
const REFERENCE_VOLTS: Volts = 1.0;

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

/// Every sense a receiver was handed — its instant and voltage — with the
/// level it projected it to.
type Readings = Arc<Mutex<Vec<(u64, Option<Volts>, Option<Level>)>>>;

/// A receiver on pin `IN`: projects every sense through the pin's declared
/// thresholds, chosen by the level it last read ([`DigitalReceiver`]), and
/// records both. A pin that declares no thresholds records the voltage and
/// no level.
struct Receiver {
    pins: Vec<PinDecl>,
    readings: Readings,
}

impl Receiver {
    fn new(pins: Vec<PinDecl>) -> (Self, Readings) {
        let readings: Readings = Arc::new(Mutex::new(Vec::new()));
        (
            Self {
                pins,
                readings: Arc::clone(&readings),
            },
            readings,
        )
    }
}

impl Component for Receiver {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let readings = Arc::clone(&self.readings);
        let receiver = DigitalReceiver::new(io.pin("IN")?);
        io.on_sense("IN", move |sense: Sense| {
            let level = receiver.read(&sense);
            readings
                .lock()
                .unwrap()
                .push((sense.at_ns, sense.volts, level));
        })
    }
}

/// The last reading a receiver recorded.
fn last(readings: &Readings) -> Option<(u64, Option<Volts>, Option<Level>)> {
    readings.lock().unwrap().last().copied()
}

fn near(a: Option<Volts>, b: Volts) -> bool {
    a.is_some_and(|a| (a - b).abs() < 1e-9)
}

struct Bench {
    system: SystemHandle,
    q: PinHandle,
    hold: Readings,
    unknown: Readings,
    lvcmos: Readings,
    pad: Readings,
    referenced: Readings,
    open: Readings,
    fought: Readings,
}

fn bench() -> Bench {
    virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);
    let handle = Arc::new(Mutex::new(None));
    let (hold, hold_log) =
        Receiver::new(vec![PinDecl::digital_in("IN", schmitt(DeadBand::HoldLast))]);
    let (unknown, unknown_log) =
        Receiver::new(vec![PinDecl::digital_in("IN", schmitt(DeadBand::Unknown))]);
    let (lvcmos, lvcmos_log) = Receiver::new(vec![PinDecl::digital_in(
        "IN",
        jesd8c01_lvcmos_thresholds(DeadBand::Unknown),
    )]);
    let (pad, pad_log) = Receiver::new(vec![
        PinDecl::digital_in("IN", P2_PAD)
            .with_supply("VIO")
            .with_reference("GND"),
        PinDecl::power_in("VIO").with_reference("GND"),
        PinDecl::power_in("GND"),
    ]);
    let (referenced, referenced_log) = Receiver::new(vec![
        PinDecl::analog("IN").with_reference("REF"),
        PinDecl::power_in("REF"),
    ]);
    let (open, open_log) = Receiver::new(vec![PinDecl::digital_in(
        "IN",
        jesd8c01_lvcmos_thresholds(DeadBand::Unknown),
    )]);
    let (fought, fought_log) = Receiver::new(vec![PinDecl::digital_in(
        "IN",
        jesd8c01_lvcmos_thresholds(DeadBand::Unknown),
    )]);
    let system = System::new()
        .component(
            "DRV",
            Box::new(Driver {
                pins: [PinDecl::digital_out("Q").with_idle(None)],
                handle: Arc::clone(&handle),
            }),
        )
        .component("HOLD", Box::new(hold))
        .component("UNKNOWN", Box::new(unknown))
        .component("LVCMOS", Box::new(lvcmos))
        .component("PAD", Box::new(pad))
        .component("ADC", Box::new(referenced))
        .component("OPEN", Box::new(open))
        .component("FOUGHT", Box::new(fought))
        // Two push-pull pads fighting over the fought receiver's net: one
        // idling high at 3.3 V, one low at 0 V, 25 Ω each.
        .component(
            "HI",
            Box::new(Driver {
                pins: [PinDecl::digital_out("Q")],
                handle: Arc::new(Mutex::new(None)),
            }),
        )
        .component(
            "LO",
            Box::new(Driver {
                pins: [PinDecl::digital_out("Q").with_idle(Some(TheveninDrive {
                    volts: 0.0,
                    impedance: 25.0,
                }))],
                handle: Arc::new(Mutex::new(None)),
            }),
        )
        .harness(
            Harness::new()
                .connect(ep("HOLD.IN"), ep("DRV.Q"))
                .connect(ep("UNKNOWN.IN"), ep("DRV.Q"))
                .connect(ep("LVCMOS.IN"), ep("DRV.Q"))
                .connect(ep("PAD.IN"), ep("DRV.Q"))
                .connect(ep("ADC.IN"), ep("DRV.Q"))
                .power(ep("BENCH.GND"), ep("PAD.GND"), 0.0)
                .power(ep("BENCH.VIO"), ep("PAD.VIO"), BANK_VOLTS)
                .power(ep("BENCH.REF"), ep("ADC.REF"), REFERENCE_VOLTS)
                .connect(ep("FOUGHT.IN"), ep("HI.Q"))
                .connect(ep("LO.Q"), ep("HI.Q")),
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
        hold: hold_log,
        unknown: unknown_log,
        lvcmos: lvcmos_log,
        pad: pad_log,
        referenced: referenced_log,
        open: open_log,
        fought: fought_log,
    }
}

impl Bench {
    /// Drive the node to `volts` behind 25 Ω and wait until every receiver
    /// on it has been handed the new voltage; return what each read, in
    /// the order hold, unknown, LVCMOS, pad.
    fn step(&self, volts: Volts) -> [Option<Level>; 4] {
        self.q.drive(embsim_board::Drive::Thevenin(TheveninDrive {
            volts,
            impedance: 25.0,
        }));
        let logs = [&self.hold, &self.unknown, &self.lvcmos, &self.pad];
        for log in logs {
            assert!(
                wait_for(|| last(log).is_some_and(|(_, v, _)| near(v, volts)), SETTLE),
                "every receiver is handed {volts} V: {:?}",
                log.lock().unwrap()
            );
        }
        logs.map(|log| last(log).unwrap().2)
    }
}

// ============================================================
// The receiver's projection
// ============================================================

/// One node walked 0 V → 1.2 V → 3.3 V → 1.2 V → 0 V → 1.2 V → 1.5 V,
/// read by four receivers at once. At 1.2 V the Schmitt figures put the
/// node inside the band from above (above `V_T+` − `ΔV_T` = 1.31 V is
/// guaranteed high, and 1.2 V is under it) and inside the guaranteed-low
/// reach from below (at or under `V_T−` + `ΔV_T` = 1.40 V).
#[rstest]
fn a_receiver_reads_its_node_through_its_own_thresholds() {
    behaviour!(Test {
        id: "sense.receiver-projection",
        covers: Some("board/src/component.rs#Sense::level"),
        given: "one node walked between 0 and 3.3 volts via 1.2 and 1.5, read by two Schmitt \
                receivers, an LVCMOS one and a P2 pad in a 1.8 volt bank",
    });
    expect!(
        "hysteresis-low-high-low",
        "the holding Schmitt receiver reads the 1.2 volt node low, then high after 3.3 volts, \
         then low after 0 volts",
        "between its thresholds a Schmitt input keeps the level it last recognised"
    );
    expect!(
        "hold-last-against-unknown",
        "at 1.2 volts after a high, inside the band hysteresis leaves open, the holding \
         receiver reads high and the other reads no level",
        "the policy for a voltage the datasheet guarantees neither level at is the \
         receiver's own declaration"
    );
    expect!(
        "same-volts-two-receivers",
        "at 1.5 volts the pad in the 1.8 volt bank reads high while the LVCMOS receiver on \
         the same node reads no level",
        "0.7 of 1.8 volts is 1.26 volts, and 1.5 volts sits between the LVCMOS pair's 0.8 \
         and 2.0 volts"
    );
    expect!(
        "delivered-instant",
        "every reading carries the virtual instant the engine delivered it at"
    );

    let _lock = suite_lock();
    let b = bench();
    use Level::{High, Low};

    let steps: [(Volts, [Option<Level>; 4]); 7] = [
        (0.0, [Some(Low), Some(Low), Some(Low), Some(Low)]),
        (1.2, [Some(Low), Some(Low), None, None]),
        (3.3, [Some(High), Some(High), Some(High), Some(High)]),
        (1.2, [Some(High), None, None, None]),
        (0.0, [Some(Low), Some(Low), Some(Low), Some(Low)]),
        (1.2, [Some(Low), Some(Low), None, None]),
        (1.5, [Some(Low), None, None, Some(High)]),
    ];
    let mut hold_at_1v2: Vec<Option<Level>> = Vec::new();
    for (volts, expected) in steps {
        let read = b.step(volts);
        assert_eq!(
            read, expected,
            "{volts} V read as (hold, unknown, LVCMOS, pad) {read:?}"
        );
        if volts == 1.2 {
            hold_at_1v2.push(read[0]);
        }
    }
    assert_eq!(
        hold_at_1v2,
        vec![Some(Low), Some(High), Some(Low)],
        "the 1.2 V node reads low, then high, then low"
    );

    // Every reading carries the instant it was delivered at: the engine's
    // virtual clock, never later than now, and in delivery order.
    let now = virtual_clock::virtual_ns();
    let hold = b.hold.lock().unwrap().clone();
    assert!(hold.windows(2).all(|w| w[0].0 <= w[1].0), "{hold:?}");
    assert!(hold.iter().all(|(at, _, _)| *at <= now), "{hold:?}");
    drop(b.system);
}

/// The analog reader's reference pin is held at 1.0 V: the node it reads
/// is handed to it against that pin.
#[rstest]
fn a_sense_is_the_nodes_voltage_against_the_pins_reference() {
    behaviour!(Test {
        id: "sense.against-reference",
        covers: Some("board/src/component.rs#ComponentNetIo::on_sense"),
        given: "an analog reader whose declared reference pin is held at 1 volt, on a node \
                driven to 3.3 volts and then to 0.5 volts",
    });
    expect!(
        "less-the-reference",
        "it is handed 2.3 volts and then -0.5 volts: the node's voltage less its reference's",
        "a pin's voltages are measured against the pin its part declares them against"
    );
    expect!(
        "no-level-without-thresholds",
        "it reads no level: a pin that declares no thresholds is handed volts only"
    );

    let _lock = suite_lock();
    let b = bench();
    for (volts, against) in [(3.3, 2.3), (0.5, -0.5)] {
        b.step(volts);
        assert!(
            wait_for(
                || last(&b.referenced).is_some_and(|(_, v, _)| near(v, against)),
                SETTLE
            ),
            "{volts} V against the 1 V reference: {:?}",
            b.referenced.lock().unwrap()
        );
        assert_eq!(last(&b.referenced).unwrap().2, None);
    }
    drop(b.system);
}

/// A digital receiver nothing drives, and one on a net two strong pads
/// fight over.
#[rstest]
fn a_floating_node_names_no_voltage_and_a_fought_one_names_its_operating_point() {
    behaviour!(Test {
        id: "sense.floating-and-fought",
        covers: Some("board/src/component.rs#Sense"),
        given: "a digital receiver on a net nothing drives, and another on a net where a \
                25 ohm pad driving 3.3 volts meets a 25 ohm pad driving 0 volts",
    });
    expect!(
        "floating-no-voltage",
        "the receiver nothing drives is handed no voltage and reads no level, and the build \
         reports it as a floating digital sense"
    );
    expect!(
        "fought-operating-point",
        "the fought receiver is handed 1.65 volts, the voltage the fight settles at, and \
         reads no level through the 3.3 volt LVCMOS pair",
        "a fight with one operating point hands a reader that voltage; the fight itself \
         is a finding, reported beside it"
    );
    expect!(
        "fight-reported",
        "the fight is reported as contention naming both pads, with the ambiguous level at \
         1.65 volts"
    );

    let _lock = suite_lock();
    let b = bench();
    assert!(
        wait_for(|| last(&b.open).is_some(), SETTLE),
        "the open receiver is handed its node at registration"
    );
    assert_eq!(
        last(&b.open).map(|(_, v, level)| (v, level)),
        Some((None, None))
    );
    let findings = b.system.findings();
    assert!(
        findings.iter().any(|f| matches!(
            f,
            Finding::FloatingSense { net, kind: SenseKind::Digital } if net == "OPEN.IN"
        )),
        "{findings:?}"
    );

    assert!(
        wait_for(
            || last(&b.fought).is_some_and(|(_, v, _)| near(v, 1.65)),
            SETTLE
        ),
        "{:?}",
        b.fought.lock().unwrap()
    );
    assert_eq!(last(&b.fought).unwrap().2, None);
    let findings = b.system.findings();
    let fight = findings
        .iter()
        .find_map(|f| match f {
            Finding::Contention { net, drivers }
                if net.starts_with("FOUGHT.IN")
                    || net.starts_with("HI.Q")
                    || net.starts_with("LO.Q") =>
            {
                Some(drivers.clone())
            }
            _ => None,
        })
        .unwrap_or_else(|| panic!("a contention on the fought net: {findings:?}"));
    assert_eq!(fight.len(), 2, "both pads named: {fight:?}");
    assert!(
        findings.iter().any(|f| matches!(
            f,
            Finding::AmbiguousLevel { volts, .. } if (volts - 1.65).abs() < 1e-9
        )),
        "{findings:?}"
    );
    drop(b.system);
}

/// A pad whose thresholds are relative to its bank supply, held at 1.3 V
/// while the supply steps from 3.3 V to 1.8 V: 1.3 V sits between 0.3 and
/// 0.7 of 3.3 V (0.99 V, 2.31 V) and above 0.7 of 1.8 V (1.26 V).
#[rstest]
fn a_supply_that_moves_re_delivers_the_pins_sense() {
    behaviour!(Test {
        id: "sense.supply-move-re-delivers",
        covers: Some("board/src/engine.rs#EngineCore::deliver_senses"),
        given: "a P2 pad reading 0.3 and 0.7 of its bank supply, its node held at 1.3 volts \
                while the bank steps from 3.3 volts to 1.8 volts",
    });
    expect!(
        "no-level-at-3v3",
        "at a 3.3 volt bank the pad reads no level"
    );
    expect!(
        "re-delivered-high",
        "when only the bank moves, the pad is handed its unchanged 1.3 volts again and reads \
         high",
        "a relative threshold scales with its supply, so the supply is an input of the \
         receiver's projection like the node itself"
    );

    let _lock = suite_lock();
    virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);
    let node = Arc::new(Mutex::new(None));
    let supply = Arc::new(Mutex::new(None));
    let (pad, pad_log) = Receiver::new(vec![
        PinDecl::digital_in("IN", P2_PAD)
            .with_supply("VIO")
            .with_reference("GND"),
        PinDecl::power_in("VIO").with_reference("GND"),
        PinDecl::power_in("GND"),
    ]);
    let system = System::new()
        .component(
            "DRV",
            Box::new(Driver {
                pins: [PinDecl::digital_out("Q").with_idle(None)],
                handle: Arc::clone(&node),
            }),
        )
        .component(
            "SUP",
            Box::new(Driver {
                pins: [PinDecl::digital_out("Q").with_idle(None)],
                handle: Arc::clone(&supply),
            }),
        )
        .component("PAD", Box::new(pad))
        .harness(
            Harness::new()
                .connect(ep("PAD.IN"), ep("DRV.Q"))
                .connect(ep("PAD.VIO"), ep("SUP.Q"))
                .power(ep("BENCH.GND"), ep("PAD.GND"), 0.0),
        )
        .start()
        .expect("the bench starts");
    assert!(
        wait_for(
            || node.lock().unwrap().is_some() && supply.lock().unwrap().is_some(),
            SETTLE
        ),
        "the drivers are wired"
    );
    let drive = |pin: &Arc<Mutex<Option<PinHandle>>>, volts: Volts| {
        pin.lock()
            .unwrap()
            .as_ref()
            .unwrap()
            .drive(embsim_board::Drive::Thevenin(TheveninDrive {
                volts,
                impedance: 25.0,
            }));
    };
    // The bank first, and seen published before the node is driven: the
    // node's delivery is then projected against a 3.3 V bank.
    drive(&supply, 3.3);
    let supply_net = supply.lock().unwrap().as_ref().unwrap().net();
    assert!(
        wait_for(
            || system.net_state_of(supply_net) == Some(NetState::Driven(Level::High)),
            SETTLE
        ),
        "the bank is at 3.3 V: {:?}",
        system.net_state_of(supply_net)
    );
    drive(&node, 1.3);
    assert!(
        wait_for(
            || last(&pad_log).is_some_and(|(_, v, _)| near(v, 1.3)),
            SETTLE
        ),
        "{:?}",
        pad_log.lock().unwrap()
    );
    assert_eq!(last(&pad_log).unwrap().2, None, "1.3 V in a 3.3 V bank");

    // Only the bank moves: the pad is handed its unchanged node again, and
    // reads it through the thresholds the new bank scales. Wait for that
    // observable itself — the last delivery at 1.3 V, read high.
    drive(&supply, 1.8);
    assert!(
        wait_for(
            || last(&pad_log)
                .is_some_and(|(_, v, level)| near(v, 1.3) && level == Some(Level::High)),
            SETTLE
        ),
        "the bank's move re-delivers the pad's sense, read high: {:?}",
        pad_log.lock().unwrap()
    );
    drop(system);
}

/// A build is a snapshot with no instant: what it hands a receiver
/// carries instant 0, whatever the process's virtual clock reads — here
/// advanced to 5 ms, as a run before the build would have left it.
#[rstest]
fn a_sense_handed_at_build_carries_no_instant() {
    behaviour!(Test {
        id: "sense.build-has-no-instant",
        covers: Some("board/src/component.rs#PinHandle::measure"),
        given: "a receiver whose input a 3.3 volt bench rail holds, built while the virtual \
                clock a previous run left behind reads 5 milliseconds",
    });
    expect!(
        "instant-zero",
        "every sense the build hands the receiver carries instant 0 and the rail's 3.3 volts",
        "a build resolves the board before any time passes, and a clock another run advanced \
         is not the build's"
    );

    let _lock = suite_lock();
    virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);
    virtual_clock::advance_to_ns(5_000_000).expect("time moves forward");
    assert_eq!(virtual_clock::virtual_ns(), 5_000_000);
    let (receiver, readings) = Receiver::new(vec![PinDecl::digital_in(
        "IN",
        jesd8c01_lvcmos_thresholds(DeadBand::Unknown),
    )]);
    let built = System::new()
        .component("RX", Box::new(receiver))
        .harness(Harness::new().power(ep("BENCH.3V3"), ep("RX.IN"), 3.3))
        .build()
        .expect("the bench builds");
    let readings = readings.lock().unwrap().clone();
    assert!(!readings.is_empty(), "the build hands the receiver its net");
    assert!(
        readings.iter().all(|(at_ns, _, _)| *at_ns == 0),
        "{readings:?}"
    );
    assert_eq!(readings.last(), Some(&(0, Some(3.3), Some(Level::High))));
    drop(built);
}

// ============================================================
// A clock is the receiver's too
// ============================================================

/// A stepper drive — `STEP`, `DIR` and `ENA` read through the JESD8C.01
/// pair, 0.8 V / 2.0 V, no level between (`stepper_motor::STEPPER_PINS`) —
/// enabled and pointed one way from the bench, its `STEP` input driven
/// straight by a 10 kHz square wave from 0 V to 1.2 V for 3 ms, then by one
/// from 0 V to 3.3 V. The 1.2 V phase sits between the thresholds, where
/// the input reads no level: the wave never crosses the input's switching
/// point, so the drive sees no clock and counts nothing. The 3.3 V wave
/// crosses both and is counted pulse for pulse from its anchor.
#[rstest]
fn a_step_input_counts_a_clock_only_when_it_crosses_the_thresholds() {
    behaviour!(Test {
        id: "sense.step-clock-must-cross",
        covers: Some("models/src/machine/stepper_motor.rs#StepperMotor"),
        given: "an enabled stepper drive whose step input is driven directly by a 10 kilohertz \
                square wave from 0 to 1.2 volts for 3 milliseconds, then by one from 0 to 3.3 \
                volts",
    });
    expect!(
        "low-swing-uncounted",
        "the 1.2 volt wave leaves the commanded step count at zero",
        "1.2 volts sits between the step input's 0.8 and 2.0 volt thresholds, where it reads \
         no level, so the input never switches"
    );
    expect!(
        "full-swing-counted",
        "the 3.3 volt wave is counted pulse for pulse from the instant it was driven",
        "each phase crosses a threshold, so the input switches every cycle"
    );

    let _lock = suite_lock();
    virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);
    let step = Arc::new(Mutex::new(None));
    let held = |volts: Volts| {
        Some(TheveninDrive {
            volts,
            impedance: 25.0,
        })
    };
    let motor = StepperMotor::new(stepper_motor::Config::new(100.0)).expect("valid");
    let shaft = motor.shaft();
    let system = System::new()
        .component(
            "DRV",
            Box::new(Driver {
                pins: [PinDecl::digital_out("Q").with_idle(None)],
                handle: Arc::clone(&step),
            }),
        )
        // ENA active high (the configuration's default), DIR low.
        .component(
            "ENA",
            Box::new(Driver {
                pins: [PinDecl::digital_out("Q").with_idle(held(3.3))],
                handle: Arc::new(Mutex::new(None)),
            }),
        )
        .component(
            "DIR",
            Box::new(Driver {
                pins: [PinDecl::digital_out("Q").with_idle(held(0.0))],
                handle: Arc::new(Mutex::new(None)),
            }),
        )
        .component("MOTOR", Box::new(motor))
        .harness(
            Harness::new()
                .connect(ep("MOTOR.STEP"), ep("DRV.Q"))
                .connect(ep("MOTOR.ENA"), ep("ENA.Q"))
                .connect(ep("MOTOR.DIR"), ep("DIR.Q")),
        )
        .start()
        .expect("the bench starts");
    assert!(
        wait_for(|| step.lock().unwrap().is_some() && shaft.enabled(), SETTLE),
        "the driver is wired and the drive enabled"
    );
    let q = step.lock().unwrap().clone().unwrap();
    let square = |high_volts: Volts, segment: PeriodicSchedule| Drive::Periodic {
        hi: TheveninDrive {
            volts: high_volts,
            impedance: 25.0,
        },
        lo: TheveninDrive {
            volts: 0.0,
            impedance: 25.0,
        },
        segment,
    };
    let segment_from = |since_ns: u64| PeriodicSchedule {
        emitted: 0,
        freq_hz: 10_000,
        total: None,
        since_ns,
    };

    // The low-swing wave: wait until the engine reports it on the net, then
    // until 3 ms of virtual time have passed — time moves only once every
    // delivery of that pass has run.
    let low = segment_from(virtual_clock::virtual_ns());
    q.drive(square(1.2, low));
    assert!(
        wait_for(
            || matches!(
                system.net_state_of(q.net()),
                Some(NetState::Periodic { segment, .. }) if segment == low
            ),
            SETTLE
        ),
        "the step net carries the 1.2 V wave: {:?}",
        system.net_state_of(q.net())
    );
    let reported_at = virtual_clock::virtual_ns();
    assert!(
        wait_for(
            || virtual_clock::virtual_ns() >= reported_at + 3_000_000,
            SETTLE
        ),
        "virtual time runs on the drive's own observation wakes"
    );
    assert_eq!(shaft.train(), None, "no train presented");
    assert_eq!(shaft.commanded_steps(), 0, "nothing counted");

    // The full-swing wave, counted from its anchor.
    let full = segment_from(virtual_clock::virtual_ns());
    q.drive(square(3.3, full));
    assert!(
        wait_for(|| shaft.train() == Some(full), SETTLE),
        "the 3.3 V wave is presented as a train: {:?}",
        shaft.train()
    );
    assert!(
        wait_for(
            || virtual_clock::virtual_ns() >= full.since_ns + 3_000_000,
            SETTLE
        ),
        "virtual time runs"
    );
    let before = virtual_clock::virtual_ns();
    let counted = shaft.commanded_steps().unsigned_abs();
    let after = virtual_clock::virtual_ns();
    assert!(
        (full.emitted_at_ns(before)..=full.emitted_at_ns(after)).contains(&counted),
        "every pulse since the anchor, and only those: {counted} in {}..={}",
        full.emitted_at_ns(before),
        full.emitted_at_ns(after)
    );
    assert!(counted >= 30, "3 ms at 10 kHz: {counted}");
    drop(system);
}

/// A step source that drives `STEP` from its own wakes, at the instants
/// its schedule names: each drive published at its instant is resolved and
/// handed on at that instant (`NODES.md` §11 contract lines 1 and 2), so
/// every instant the drive reads is one the test wrote down.
struct ScheduledStep {
    pins: [PinDecl; 1],
    schedule: Vec<(u64, Drive)>,
}

impl Component for ScheduledStep {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let out = io.pin("Q")?;
        let schedule = self.schedule.clone();
        io.on_wake_ns(move |now| {
            if let Some((_, drive)) = schedule.iter().find(|(at, _)| *at == now) {
                out.drive(*drive);
            }
        });
        for (at_ns, _) in &self.schedule {
            io.schedule_at_ns(*at_ns);
        }
        Ok(())
    }
}

/// A millisecond of virtual time, in the engine's nanoseconds.
const MS: u64 = 1_000_000;

/// One 10 kHz step segment, anchored at 1 ms, whose high phase the source
/// drives at 3.3 V, then at 1.2 V from 4 ms — inside `STEP`'s 0.8 V / 2.0 V
/// band, so the input stops switching — then at 3.3 V again from
/// `resume_ns`, under the same schedule throughout; at 12 ms the source
/// stops the train (a held segment banking its count). The drive counts the
/// pulses of the two spans the input switched on, 1–4 ms and `resume_ns`–12
/// ms, each once — wherever the resume lands against the drive's own
/// position samples (`stepper_motor::Config::observe_interval_us`, 1 ms by
/// default): on one (9 ms), between two (9.5 ms), or with none armed, so
/// nothing moves the drive between the stop and the resume (9 ms).
#[rstest]
#[case::on_a_position_sample(9 * MS, Some(stepper_motor::DEFAULT_OBSERVE_INTERVAL_US), 30 + 30)]
#[case::between_position_samples(
    9 * MS + MS / 2,
    Some(stepper_motor::DEFAULT_OBSERVE_INTERVAL_US),
    30 + 25
)]
#[case::with_no_position_samples(9 * MS, None, 30 + 30)]
fn a_step_clock_that_stops_and_resumes_crossing_counts_each_pulse_once(
    #[case] resume_ns: u64,
    #[case] observe_interval_us: Option<u64>,
    #[case] crossed: u64,
) {
    behaviour!(Test {
        id: "sense.step-clock-resumes-crossing",
        covers: Some("models/src/machine/stepper_motor.rs#StepperMotor"),
        given: "an enabled stepper drive whose step input carries a 10 kilohertz pulse train \
                from 1 to 12 milliseconds, its high phase 1.2 volts from 4 to 9 or 9.5, else 3.3",
    });
    expect!(
        "spans-that-crossed",
        "the commanded step count is the pulses of the two spans the input switched on, each \
         counted once",
        "a pulse the input never switched on is no step and a pulse already counted is counted \
         once, wherever the input switches again against the drive's own position samples"
    );

    let _lock = suite_lock();
    virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);
    let running = PeriodicSchedule {
        emitted: 0,
        freq_hz: 10_000,
        total: None,
        since_ns: MS,
    };
    let square = |high_volts: Volts, segment: PeriodicSchedule| Drive::Periodic {
        hi: TheveninDrive {
            volts: high_volts,
            impedance: 25.0,
        },
        lo: TheveninDrive {
            volts: 0.0,
            impedance: 25.0,
        },
        segment,
    };
    let stop = PeriodicSchedule {
        emitted: running.emitted_at_ns(12 * MS),
        freq_hz: 0,
        total: None,
        since_ns: 12 * MS,
    };
    let held = |volts: Volts| {
        Some(TheveninDrive {
            volts,
            impedance: 25.0,
        })
    };
    let motor = StepperMotor::new(stepper_motor::Config {
        observe_interval_us,
        ..stepper_motor::Config::new(100.0)
    })
    .expect("valid");
    let shaft = motor.shaft();
    let system = System::new()
        .component(
            "DRV",
            Box::new(ScheduledStep {
                pins: [PinDecl::digital_out("Q").with_idle(None)],
                schedule: vec![
                    (MS, square(3.3, running)),
                    (4 * MS, square(1.2, running)),
                    (resume_ns, square(3.3, running)),
                    (12 * MS, square(3.3, stop)),
                ],
            }),
        )
        // ENA active high (the configuration's default), DIR low.
        .component(
            "ENA",
            Box::new(Driver {
                pins: [PinDecl::digital_out("Q").with_idle(held(3.3))],
                handle: Arc::new(Mutex::new(None)),
            }),
        )
        .component(
            "DIR",
            Box::new(Driver {
                pins: [PinDecl::digital_out("Q").with_idle(held(0.0))],
                handle: Arc::new(Mutex::new(None)),
            }),
        )
        .component("MOTOR", Box::new(motor))
        .harness(
            Harness::new()
                .connect(ep("MOTOR.STEP"), ep("DRV.Q"))
                .connect(ep("MOTOR.ENA"), ep("ENA.Q"))
                .connect(ep("MOTOR.DIR"), ep("DIR.Q")),
        )
        .start()
        .expect("the bench starts");

    assert!(
        wait_for(|| shaft.train() == Some(stop), SETTLE),
        "the stop reaches the drive: {:?}",
        shaft.train()
    );
    assert_eq!(
        (running.emitted_at_ns(4 * MS) - running.emitted_at_ns(MS))
            + (running.emitted_at_ns(12 * MS) - running.emitted_at_ns(resume_ns)),
        crossed,
        "the segment's own pulses over the two spans: one every 100 µs"
    );
    assert_eq!(shaft.commanded_steps().unsigned_abs(), crossed);
    drop(system);
}
