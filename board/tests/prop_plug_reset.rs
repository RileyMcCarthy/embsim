//! A Parallax Prop Plug's reset stage on a bench: its `DTR` driven through
//! a schedule of levels, its `RES` pulled up by 10 kΩ to 3.3 V as a
//! Propeller's reset is, and a probe reading `RES` (`embsim_models::
//! prop_plug`). Stepped (`TESTING.md` rule 9): every instant is asserted
//! exactly.

use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};

use embsim_board::{
    digital_drive, jesd8c01_lvcmos_thresholds, AttachError, Component, ComponentNetIo, DeadBand,
    DigitalReceiver, EndpointRef, Harness, Level, PinDecl, System, TheveninDrive,
};
use embsim_core::virtual_clock::{self, ClockMode};
use embsim_models::prop_plug::{PropPlugReset, PROP_PLUG_RESET_PULSE_NS};
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

const US: u64 = 1_000;
const MS: u64 = 1_000_000;

/// A logic line stepping through a schedule: driven at the first level from
/// the build, then at each `(instant, level)` from its wake at that instant.
struct BenchLine {
    pins: [PinDecl; 1],
    steps: Vec<(u64, Level)>,
}

impl BenchLine {
    fn stepping(first: Level, steps: &[(u64, Level)]) -> Self {
        Self {
            pins: [PinDecl::digital_out("A").with_idle(Some(digital_drive(first)))],
            steps: steps.to_vec(),
        }
    }
}

impl Component for BenchLine {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let out = io.pin("A")?;
        let steps = self.steps.clone();
        io.on_wake_ns(move |now| {
            if let Some(&(_, level)) = steps.iter().find(|(at, _)| *at == now) {
                out.set_drive(Some(digital_drive(level)));
            }
        });
        for &(at_ns, _) in &self.steps {
            io.schedule_at_ns(at_ns);
        }
        Ok(())
    }
}

/// The target's reset pull-up: 10 kΩ to 3.3 V.
struct PullUp {
    pins: [PinDecl; 1],
}

impl Component for PullUp {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, _io: ComponentNetIo) -> Result<(), AttachError> {
        Ok(())
    }
}

/// Every level a probe read, with the instant.
type Levels = Arc<Mutex<Vec<(u64, Option<Level>)>>>;

/// A probe: every level its line reads, with the instant, and a wake at
/// `until_ns` that says the engine has passed it.
struct Probe {
    pins: [PinDecl; 1],
    until_ns: u64,
    seen: Levels,
    done: Arc<Mutex<bool>>,
}

impl Component for Probe {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let receiver = DigitalReceiver::new(io.pin("IN")?);
        let seen = Arc::clone(&self.seen);
        io.on_sense("IN", move |sense| {
            let level = receiver.read(&sense);
            let mut seen = seen.lock().unwrap();
            if seen.last().map(|&(_, last)| last) != Some(level) {
                seen.push((sense.at_ns, level));
            }
        })?;
        let done = Arc::clone(&self.done);
        io.on_wake_ns(move |_| *done.lock().unwrap() = true);
        io.schedule_at_ns(self.until_ns);
        Ok(())
    }
}

#[test]
fn the_reset_stage_pulses_res_low_on_each_rising_edge_of_dtr() {
    behaviour!(Test {
        id: "prop-plug.reset-pulse",
        covers: Some("models/src/prop_plug.rs#PropPlugReset"),
        given: "a Prop Plug's reset stage with a pulled-up output, its DTR starting high, falling \
                at 1 ms, rising at 2, falling at 3, rising at 4 and again after a 5 µs dip",
    });
    expect!(
        "settled-at-start",
        "the level the input reads when the bench starts pulls nothing: the output rests high",
        "the adapter is taken as powered and settled at the start"
    );
    expect!(
        "falling-edge-nothing",
        "a falling edge leaves the output released"
    );
    expect!(
        "rising-edge-pulse",
        "a rising edge pulls the output low from its instant for 20 microseconds",
        "the Prop Plug's product page gives a typical reset pulse of 20 microseconds"
    );
    expect!(
        "pulse-restarts",
        "a rising edge inside a pulse starts the pulse again, so the output stays low for 20 \
         microseconds from the later edge"
    );
    expect!(
        "pulses-counted",
        "the stage counts one pulse for each rising edge"
    );

    let _lock = suite_lock();
    virtual_clock::init_mode(ClockMode::Stepped, 1_000_000);
    let stage = PropPlugReset::new();
    let handle = stage.handle();
    let seen = Arc::default();
    let done = Arc::new(Mutex::new(false));
    let probe = Probe {
        pins: [PinDecl::digital_in(
            "IN",
            jesd8c01_lvcmos_thresholds(DeadBand::Unknown),
        )],
        until_ns: 6 * MS,
        seen: Arc::clone(&seen),
        done: Arc::clone(&done),
    };
    let system = System::new()
        .component("PLUG", Box::new(stage))
        .component(
            "DTR",
            Box::new(BenchLine::stepping(
                Level::High,
                &[
                    (MS, Level::Low),
                    (2 * MS, Level::High),
                    (3 * MS, Level::Low),
                    (4 * MS, Level::High),
                    (4 * MS + 5 * US, Level::Low),
                    (4 * MS + 10 * US, Level::High),
                ],
            )),
        )
        .component(
            "PULL",
            Box::new(PullUp {
                pins: [PinDecl::analog_source("A").with_idle(Some(TheveninDrive {
                    volts: 3.3,
                    impedance: 10_000.0,
                }))],
            }),
        )
        .component("PROBE", Box::new(probe))
        .harness(
            Harness::new()
                .power(ep("BENCH.GND"), ep("PLUG.VSS"), 0.0)
                .connect_str("DTR.A", "PLUG.DTR")
                .expect("endpoints parse")
                .connect_str("PLUG.RES", "PULL.A")
                .expect("endpoints parse")
                .connect_str("PLUG.RES", "PROBE.IN")
                .expect("endpoints parse"),
        )
        .start()
        .expect("the bench starts");
    let started = Instant::now();
    while !*done.lock().unwrap() {
        assert!(
            started.elapsed() < Duration::from_secs(30),
            "the bench never reached 6 ms"
        );
        std::thread::sleep(Duration::from_millis(1));
    }
    let seen = seen.lock().unwrap().clone();
    assert_eq!(
        seen,
        vec![
            (0, Some(Level::High)),
            (2 * MS, Some(Level::Low)),
            (2 * MS + PROP_PLUG_RESET_PULSE_NS, Some(Level::High)),
            (4 * MS, Some(Level::Low)),
            (
                4 * MS + 10 * US + PROP_PLUG_RESET_PULSE_NS,
                Some(Level::High)
            ),
        ],
        "the reset line, level by level"
    );
    assert_eq!(handle.pulses(), 3, "pulses-counted");
    assert_eq!(handle.last_pulse_ns(), Some(4 * MS + 10 * US));
    drop(system);
}
