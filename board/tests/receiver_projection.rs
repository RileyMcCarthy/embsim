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
//! drivers fight over. Stepped mode (`TESTING.md` rule 9): every delivery
//! carries the virtual instant the engine delivered it at. Its own binary
//! per rule 5.

use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};

use embsim_board::{
    jesd8c01_lvcmos_thresholds, AttachError, Component, ComponentNetIo, DeadBand, DigitalReceiver,
    EndpointRef, Finding, Harness, Level, PinDecl, PinHandle, Sense, SenseKind, System,
    SystemHandle, TheveninDrive, Thresholds, Volts,
};
use embsim_core::virtual_clock::{self, ClockMode};
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
    drive(&supply, 3.3);
    drive(&node, 1.3);
    assert!(
        wait_for(
            || last(&pad_log).is_some_and(|(_, v, _)| near(v, 1.3)),
            SETTLE
        ),
        "{:?}",
        pad_log.lock().unwrap()
    );
    // The supply's own settling may re-deliver; wait for the log to rest.
    std::thread::sleep(Duration::from_millis(50));
    assert_eq!(last(&pad_log).unwrap().2, None, "1.3 V in a 3.3 V bank");
    let before = pad_log.lock().unwrap().len();

    drive(&supply, 1.8);
    assert!(
        wait_for(|| pad_log.lock().unwrap().len() > before, SETTLE),
        "the bank's move re-delivers the pad's sense: {:?}",
        pad_log.lock().unwrap()
    );
    let (_, volts, level) = last(&pad_log).unwrap();
    assert!(near(volts, 1.3), "the node did not move: {volts:?}");
    assert_eq!(level, Some(Level::High), "1.3 V in a 1.8 V bank");
    drop(system);
}
