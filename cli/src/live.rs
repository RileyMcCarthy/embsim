//! A project built and started: `embsim check` and `embsim run`.
//!
//! Both load the project, survey each board with the registry it builds
//! with, and build the system through the catalog. `check` starts it with
//! virtual time held — every part attached, every attach-time drive
//! resolved, no wake fired: the state `System::build` analyzes — reads what
//! the build found, and stops. `run` releases time and runs.
//!
//! The clock is stepped (`TESTING.md` rule 9): this thread is a registered
//! virtual-clock actor, and the engine advances only while it is parked in
//! a virtual wait, so a run for a duration ends at exactly that virtual
//! instant and reads the system at rest there, the same on every machine.

use std::path::Path;
use std::time::Instant;

use embsim_board::{Finding, Project, System, SystemHandle};
use embsim_boards::p2::StartState;
use embsim_core::virtual_clock::{self, ClockMode};
use embsim_p2_qemu::catalog::{QemuCatalog, QemuSeat};

/// The virtual clock's cycle rate. Nothing on a board reads it — the engine
/// keeps nanoseconds, and a P2 core its own clock from the mode it set —
/// so it is the 1 MHz the board tests use, a cycle a microsecond.
const CYCLE_HZ: u32 = 1_000_000;

/// How far the run goes between looks at the system: findings and console
/// output are printed when a look finds them, stamped with the instant of
/// that look, at most this long after they happened.
const LOOK_NS: u64 = 100_000;

/// The P2's smart pins, the ones a console can be on.
const P2_PADS: u8 = 64;

/// Parse a duration of virtual time: a number and a unit, `ns`, `us`,
/// `ms` or `s` (`20ms`, `1.5 s`, `250us`), to whole nanoseconds.
pub fn parse_duration(text: &str) -> Result<u64, String> {
    let text = text.trim();
    let split = text
        .find(|c: char| !(c.is_ascii_digit() || c == '.'))
        .ok_or_else(|| format!("{text:?} has no unit; give one of ns, us, ms, s (20ms)"))?;
    let (number, unit) = text.split_at(split);
    if number.is_empty() {
        return Err(format!(
            "{text:?} does not start with a number; give a number and a unit (20ms)"
        ));
    }
    let per_unit: f64 = match unit.trim() {
        "ns" => 1.0,
        "us" | "µs" => 1e3,
        "ms" => 1e6,
        "s" => 1e9,
        other => {
            return Err(format!(
                "{other:?} is not a unit of time; give one of ns, us, ms, s (20ms)"
            ))
        }
    };
    let value: f64 = number
        .parse()
        .map_err(|_| format!("{number:?} is not a number of {}", unit.trim()))?;
    let ns = (value * per_unit).round();
    if !ns.is_finite() || ns > u64::MAX as f64 {
        return Err(format!("{text:?} is longer than a run can be"));
    }
    Ok(ns as u64)
}

/// A virtual instant as milliseconds, exactly: `5.500000 ms`.
fn ms(ns: u64) -> String {
    format!("{}.{:06} ms", ns / 1_000_000, ns % 1_000_000)
}

/// Load `path`, print each board's survey line, and build the system.
fn build(path: &Path, catalog: &QemuCatalog) -> Result<System, String> {
    let project = Project::load(path).map_err(|err| err.to_string())?;
    println!("project {}", path.display());
    for spec in project.boards() {
        let survey = project
            .survey(catalog, &spec.name)
            .map_err(|err| err.to_string())?;
        let text = survey.to_string();
        let line = text.lines().next().unwrap_or_default();
        println!("  board {} ({}): {line}", spec.name, spec.kind);
    }
    let system = project
        .instantiate(catalog)
        .map_err(|err| err.to_string())?;
    println!(
        "  {} board{}, {} bench component{}, {} wire{}",
        project.boards().len(),
        plural(project.boards().len()),
        project.components().len(),
        plural(project.components().len()),
        project.wires().len(),
        plural(project.wires().len()),
    );
    Ok(system)
}

fn plural(count: usize) -> &'static str {
    if count == 1 {
        ""
    } else {
        "s"
    }
}

/// Start the stepped clock at zero. Before anything is built: a model can
/// start a thread that joins the clock as an actor when it is constructed
/// (the ADS122U04's protocol pump), and it joins the clock this run keeps.
fn start_clock() {
    virtual_clock::init_mode(ClockMode::Stepped, CYCLE_HZ);
}

/// Start `system` with virtual time held.
fn start_held(path: &Path, system: System) -> Result<SystemHandle, String> {
    system
        .hold_time()
        .start()
        .map_err(|err| format!("{}: the system does not start: {err}", path.display()))
}

/// `embsim check <project>`.
pub fn check(path: &Path) -> Result<(), String> {
    start_clock();
    let catalog = QemuCatalog::new();
    let system = build(path, &catalog)?;
    let handle = start_held(path, system)?;
    let findings = handle.findings();
    if findings.is_empty() {
        println!("build findings: none");
    } else {
        println!(
            "build findings ({}), the system before its first wake:",
            findings.len()
        );
        for finding in &findings {
            println!("  {finding:?}");
        }
    }
    handle.shutdown();
    println!("ok: {} builds", path.display());
    Ok(())
}

/// What a run has printed so far, so each look prints only what is new.
struct Reporter {
    seats: Vec<QemuSeat>,
    findings: usize,
    started: Vec<bool>,
    halted: Vec<bool>,
    /// Console characters printed, per seat and pad.
    console: Vec<Vec<usize>>,
}

impl Reporter {
    fn new(seats: Vec<QemuSeat>) -> Self {
        let count = seats.len();
        Self {
            seats,
            findings: 0,
            started: vec![false; count],
            halted: vec![false; count],
            console: vec![vec![0; usize::from(P2_PADS)]; count],
        }
    }

    /// Print what appeared since the last look, stamped `now`.
    fn look(&mut self, system: &SystemHandle, now: u64) {
        let stamp = format!("[{:>14}]", ms(now));
        let findings = system.findings();
        for finding in findings.iter().skip(self.findings) {
            println!("{stamp} {finding:?}");
        }
        self.findings = findings.len();
        for (index, seat) in self.seats.iter().enumerate() {
            let name = format!("{}.{}", seat.board, seat.reference);
            if !self.started[index] {
                if let Some(at) = seat.package.started_at_ns() {
                    self.started[index] = true;
                    println!("{stamp} {name}: the core started at {}", ms(at));
                }
            }
            for pad in 0..P2_PADS {
                let text = seat.core.console(pad);
                let printed = &mut self.console[index][usize::from(pad)];
                let count = text.chars().count();
                if count > *printed {
                    let new: String = text.chars().skip(*printed).collect();
                    println!("{stamp} {name} P{pad}: {new:?}");
                    *printed = count;
                }
            }
            if !self.halted[index] && seat.core.halted() {
                self.halted[index] = true;
                println!("{stamp} {name}: every cog has stopped");
            }
        }
    }

    /// The summary of every core at the end of a run.
    fn summary(&self) {
        for seat in &self.seats {
            let name = format!("{}.{}", seat.board, seat.reference);
            let state = match seat.package.start_state() {
                StartState::Started { at_ns } => format!("started at {}", ms(at_ns)),
                StartState::BrownoutWithoutReset {
                    started_at_ns,
                    at_ns,
                    ..
                } => format!(
                    "started at {}, held by a brownout without a reset at {}",
                    ms(started_at_ns),
                    ms(at_ns)
                ),
                StartState::Restarting { starts_at_ns, .. } => {
                    format!("reset released, starting at {}", ms(starts_at_ns))
                }
                StartState::Held { reset } => format!("held in reset ({reset:?})"),
            };
            let consoles: Vec<String> = (0..P2_PADS)
                .filter_map(|pad| {
                    let text = seat.core.console(pad);
                    (!text.is_empty()).then(|| format!("P{pad} {text:?}"))
                })
                .collect();
            println!(
                "{name} (core \"qemu\"): {state}; {} pad yields; {}; console {}",
                seat.core.yields(),
                if seat.core.halted() {
                    "halted"
                } else {
                    "running"
                },
                if consoles.is_empty() {
                    "empty".to_string()
                } else {
                    consoles.join(", ")
                }
            );
        }
    }
}

/// `embsim run <project> [--for DURATION] [--net BOARD.NET]...`.
pub fn run(path: &Path, duration: Option<u64>, nets: &[String]) -> Result<(), String> {
    start_clock();
    let catalog = QemuCatalog::new();
    let system = build(path, &catalog)?;
    let handle = start_held(path, system)?;
    for net in nets {
        if handle.net_state(net).is_none() {
            return Err(format!(
                "--net {net}: no such net; a net is Board.Net, as its board's netlist names it"
            ));
        }
    }
    let mut reporter = Reporter::new(catalog.seats());
    match duration {
        Some(ns) => println!("running for {} of virtual time", ms(ns)),
        None => println!("running until interrupted"),
    }

    let actor = virtual_clock::register_actor("embsim run");
    let wall = Instant::now();
    handle.release_time();
    let origin = virtual_clock::virtual_ns();
    reporter.look(&handle, 0);
    loop {
        let elapsed = virtual_clock::virtual_ns() - origin;
        let step = match duration {
            Some(total) => LOOK_NS.min(total - elapsed.min(total)),
            None => LOOK_NS,
        };
        if step == 0 {
            break;
        }
        virtual_clock::wait_virtual_ns(step);
        reporter.look(&handle, virtual_clock::virtual_ns() - origin);
        if !handle.engine_is_alive() {
            drop(actor);
            return Err("the engine stopped: a component's model failed".to_string());
        }
    }
    let elapsed = virtual_clock::virtual_ns() - origin;
    println!(
        "ran {} of virtual time in {:.3} s",
        ms(elapsed),
        wall.elapsed().as_secs_f64()
    );
    reporter.summary();
    for net in nets {
        let state = handle
            .net_state(net)
            .expect("every named net was checked before the run");
        println!("net {net}: {state:?}");
    }
    let findings = handle.findings();
    let stalled = findings
        .iter()
        .any(|finding| matches!(finding, Finding::QuiescenceTimeout { .. }));
    println!("findings: {}", findings.len());
    if stalled {
        println!(
            "the engine advanced without waiting for a part (QuiescenceTimeout): this run is \
             not reproducible"
        );
    }
    drop(actor);
    handle.shutdown();
    Ok(())
}

#[cfg(test)]
mod tests {
    use rstest::rstest;

    use super::*;

    #[rstest]
    #[case::milliseconds("20ms", 20_000_000)]
    #[case::fraction("5.5ms", 5_500_000)]
    #[case::spaced("1.5 s", 1_500_000_000)]
    #[case::micro("250us", 250_000)]
    #[case::micro_sign("250µs", 250_000)]
    #[case::nano("100ns", 100)]
    #[case::zero("0ms", 0)]
    fn a_duration_is_a_number_and_a_unit(#[case] text: &str, #[case] ns: u64) {
        assert_eq!(parse_duration(text), Ok(ns));
    }

    #[rstest]
    #[case::no_unit("20", "has no unit")]
    #[case::bad_unit("20min", "is not a unit of time")]
    #[case::bad_number("1.2.3ms", "is not a number")]
    #[case::negative("-5ms", "does not start with a number")]
    fn a_duration_that_is_not_one_says_why(#[case] text: &str, #[case] why: &str) {
        let err = parse_duration(text).expect_err("not a duration");
        assert!(err.contains(why), "{err}");
    }

    #[rstest]
    fn an_instant_prints_in_milliseconds_to_the_nanosecond() {
        assert_eq!(ms(5_500_000), "5.500000 ms");
        assert_eq!(ms(1), "0.000001 ms");
    }
}
