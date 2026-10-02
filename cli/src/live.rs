//! A project built and started: `embsim check` and `embsim run`.
//!
//! Both load the project, survey each board with the registry it builds
//! with, and build the system through the catalog set. `check` starts it
//! with virtual time held — every part attached, every attach-time drive
//! resolved, no wake fired: the state `System::build` analyzes — reads what
//! the build found, and stops. `run` releases time and runs.
//!
//! The clock is stepped (`TESTING.md` rule 9): this thread is a registered
//! virtual-clock actor, and the engine advances only while it is parked in
//! a virtual wait, so a run for a duration ends at exactly that virtual
//! instant and reads the system at rest there, the same on every machine.

use std::collections::BTreeMap;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::time::Instant;

use embsim_board::report::instant as ms;
use embsim_board::{Finding, NetState, Project, Report, Reports, System, SystemHandle};
use embsim_boards::catalog::CatalogSet;
use embsim_core::virtual_clock::{self, ClockMode};

use crate::signals::Watch;

/// The virtual clock's cycle rate. Nothing on a board reads it — the engine
/// keeps nanoseconds, and a P2 core its own clock from the mode it set —
/// so it is the 1 MHz the board tests use, a cycle a microsecond.
const CYCLE_HZ: u32 = 1_000_000;

/// How far the run goes between looks at the system: findings and what its
/// parts report are printed when a look finds them, stamped with the
/// instant of that look, at most this long after they happened; an
/// interrupt ends the run at the next look.
const LOOK_NS: u64 = 100_000;

/// The component kind whose path `--pty` sets.
const HOST_SERIAL: &str = "host-serial";

/// Write a line to the run's output. A write that fails (a closed pipe)
/// loses the line and nothing else: the run's outcome is its exit status.
macro_rules! say {
    ($out:expr) => {{
        let _ = writeln!($out);
    }};
    ($out:expr, $($arg:tt)*) => {{
        let _ = writeln!($out, $($arg)*);
    }};
}

/// What `embsim run` was asked for besides the project.
#[derive(Debug, Default)]
pub struct RunOptions {
    /// How long to run, in virtual nanoseconds; `None` until interrupted.
    pub duration: Option<u64>,
    /// The nets to read at the end, `Board.Net`.
    pub nets: Vec<String>,
    /// `--pty`: `PATH`, or `NAME=PATH`.
    pub ptys: Vec<String>,
    /// What the binary is made of, a fact a line, printed under the
    /// project line.
    pub provenance: Vec<String>,
}

/// The net each board pin sits on: `Board.Ref.Pin` to `Board.Net`, from
/// the boards' surveys — for a finding that names a pin.
type PinNets = BTreeMap<String, String>;

/// Apply each `--pty` to the project's `host-serial` components: `PATH` to
/// its one host, `NAME=PATH` to the one named. A relative path is the
/// current directory's.
fn apply_ptys(project: &mut Project, ptys: &[String]) -> Result<(), String> {
    let hosts: Vec<String> = project
        .components()
        .iter()
        .filter(|spec| spec.kind == HOST_SERIAL)
        .map(|spec| spec.name.clone())
        .collect();
    for pty in ptys {
        let named = pty
            .split_once('=')
            .filter(|(name, _)| hosts.iter().any(|host| host == name));
        let (name, path) = match named {
            Some((name, path)) => (name.to_string(), path),
            None => match hosts.as_slice() {
                [only] => (only.clone(), pty.as_str()),
                [] => {
                    return Err(format!(
                        "--pty {pty}: the project has no {HOST_SERIAL} component for a PTY to \
                         belong to"
                    ))
                }
                several => {
                    return Err(format!(
                        "--pty {pty}: the project has {} {HOST_SERIAL} components, {}; say \
                         which with --pty NAME=PATH",
                        several.len(),
                        several.join(", ")
                    ))
                }
            },
        };
        let path = PathBuf::from(path);
        let path = if path.is_absolute() {
            path
        } else {
            std::env::current_dir()
                .map_err(|err| format!("--pty {pty}: no current directory: {err}"))?
                .join(path)
        };
        project
            .set_component_option(&name, "path", path.to_string_lossy().into_owned())
            .map_err(|err| format!("--pty {pty}: {err}"))?;
    }
    Ok(())
}

/// Load `path`, apply `ptys`, print what the binary is made of and each
/// board's survey line, and build the system, its constructors reporting to
/// `reports`.
fn build(
    set: &CatalogSet,
    path: &Path,
    ptys: &[String],
    provenance: &[String],
    reports: &Reports,
    out: &mut dyn Write,
) -> Result<(System, PinNets), String> {
    let mut project = Project::load(path).map_err(|err| err.to_string())?;
    apply_ptys(&mut project, ptys)?;
    say!(out, "project {}", path.display());
    say!(out, "  catalogs: {}", set.catalogs().join(", "));
    for line in provenance {
        say!(out, "  {line}");
    }
    let mut pin_nets = PinNets::new();
    for spec in project.boards() {
        let survey = project
            .survey(set, &spec.name)
            .map_err(|err| err.to_string())?;
        let text = survey.to_string();
        let line = text.lines().next().unwrap_or_default();
        say!(out, "  board {} ({}): {line}", spec.name, spec.kind);
        for part in survey.parts() {
            for site in &part.pins {
                pin_nets.insert(
                    format!("{}.{}.{}", spec.name, part.reference, site.pin),
                    format!("{}.{}", spec.name, site.net),
                );
            }
        }
    }
    let system = project
        .instantiate_with(set, reports)
        .map_err(|err| err.to_string())?;
    say!(
        out,
        "  {} board{}, {} bench component{}, {} wire{}, {} mate{}",
        project.boards().len(),
        plural(project.boards().len()),
        project.components().len(),
        plural(project.components().len()),
        project.wires().len(),
        plural(project.wires().len()),
        project.mates().len(),
        plural(project.mates().len()),
    );
    Ok((system, pin_nets))
}

/// What a finding says of the system now, read off its nets.
enum Now {
    /// What the finding names still reads as it did.
    Holds,
    /// It no longer does: the net and what it reads.
    Cleared(String, NetState),
    /// A fact about the board as it is built and wired, which a run does
    /// not change: no net of its own to read again.
    Standing,
}

/// Re-read the net a finding is about. A net no source reaches reads
/// `Floating`: a floating sense, an unsourced power net and a down rail
/// hold while their net does; a fight holds while its net reads one.
fn now(finding: &Finding, system: &SystemHandle, pins: &PinNets) -> Now {
    let read = |net: &str, holds: fn(&NetState) -> bool| match system.net_state(net) {
        Some(state) if !holds(&state) => Now::Cleared(net.to_string(), state),
        Some(_) => Now::Holds,
        None => Now::Standing,
    };
    let floating = |state: &NetState| matches!(state, NetState::Floating);
    let pin = |part: &str, pin: &str| pins.get(&format!("{part}.{pin}")).cloned();
    match finding {
        Finding::FloatingSense { net, .. } | Finding::PowerNetUnsourced { net } => {
            read(net, floating)
        }
        Finding::Contention { net, .. } | Finding::AmbiguousLevel { net, .. } => {
            read(net, |state| matches!(state, NetState::Contention))
        }
        Finding::RailDown { part, pin: out, .. } => match pin(part, out) {
            Some(net) => read(&net, floating),
            None => Now::Standing,
        },
        Finding::UnreferencedDomain {
            part, reference, ..
        } => match pin(part, reference) {
            Some(net) => read(&net, floating),
            None => Now::Standing,
        },
        _ => Now::Standing,
    }
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

/// `embsim check <project>`, by a binary made of what `provenance` says.
pub fn check(
    set: &CatalogSet,
    path: &Path,
    provenance: &[String],
    out: &mut dyn Write,
) -> Result<(), String> {
    start_clock();
    let reports = Reports::new();
    let (system, _) = build(set, path, &[], provenance, &reports, out)?;
    let handle = start_held(path, system)?;
    let findings = handle.findings();
    if findings.is_empty() {
        say!(out, "build findings: none");
    } else {
        say!(
            out,
            "build findings ({}), the system before its first wake:",
            findings.len()
        );
        for finding in &findings {
            say!(out, "  {finding:?}");
        }
    }
    handle.shutdown();
    say!(out, "ok: {} builds", path.display());
    Ok(())
}

/// What a run has printed so far, so each look prints only what is new.
struct Reporter {
    reports: Vec<Box<dyn Report>>,
    findings: usize,
    /// How many of the findings the build made.
    at_build: usize,
}

impl Reporter {
    fn new(reports: Vec<Box<dyn Report>>) -> Self {
        Self {
            reports,
            findings: 0,
            at_build: 0,
        }
    }

    /// Print the findings the build made, the system before its first
    /// wake: every one is about that instant, and the run may clear it.
    fn build_snapshot(&mut self, system: &SystemHandle, out: &mut dyn Write) {
        let findings = system.findings();
        if findings.is_empty() {
            say!(out, "findings at build, before any wake: none");
        } else {
            say!(
                out,
                "findings at build, before any wake ({}):",
                findings.len()
            );
        }
        for finding in &findings {
            say!(out, "  {finding:?}");
        }
        self.findings = findings.len();
        self.at_build = findings.len();
    }

    /// Print what appeared since the last look, stamped `now`.
    fn look(&mut self, system: &SystemHandle, now: u64, out: &mut dyn Write) {
        let stamp = format!("[{:>14}]", ms(now));
        let findings = system.findings();
        for finding in findings.iter().skip(self.findings) {
            say!(out, "{stamp} {finding:?}");
        }
        self.findings = findings.len();
        for report in &mut self.reports {
            let subject = report.subject();
            for line in report.look(now) {
                say!(out, "{stamp} {subject}: {line}");
            }
        }
    }

    /// What every report says at the end of a run.
    fn summary(&self, out: &mut dyn Write) {
        for report in &self.reports {
            let subject = report.subject();
            for line in report.summary() {
                say!(out, "{subject}: {line}");
            }
        }
    }
}

/// `embsim run <project> [--for DURATION] [--net BOARD.NET]... [--pty
/// [NAME=]PATH]...`.
pub fn run(
    set: &CatalogSet,
    path: &Path,
    options: &RunOptions,
    out: &mut dyn Write,
) -> Result<(), String> {
    start_clock();
    let reports = Reports::new();
    let (system, pin_nets) = build(set, path, &options.ptys, &options.provenance, &reports, out)?;
    let handle = start_held(path, system)?;
    for net in &options.nets {
        if handle.net_state(net).is_none() {
            handle.shutdown();
            return Err(format!(
                "--net {net}: no such net; a net is Board.Net, as its board's netlist names it"
            ));
        }
    }
    let mut reporter = Reporter::new(reports.take());
    // Before the line that says the run is running: a signal sent once it
    // says so ends the run with its summary.
    let watch = Watch::install();
    match options.duration {
        Some(ns) => say!(out, "running for {} of virtual time", ms(ns)),
        None => say!(out, "running until interrupted"),
    }
    let _ = out.flush();

    let actor = virtual_clock::register_actor("embsim run");
    reporter.build_snapshot(&handle, out);
    let wall = Instant::now();
    handle.release_time();
    let origin = virtual_clock::virtual_ns();
    reporter.look(&handle, 0, out);
    let mut interrupted = false;
    loop {
        let _ = out.flush();
        if watch.interrupted() {
            interrupted = true;
            break;
        }
        let elapsed = virtual_clock::virtual_ns() - origin;
        let step = match options.duration {
            Some(total) => LOOK_NS.min(total - elapsed.min(total)),
            None => LOOK_NS,
        };
        if step == 0 {
            break;
        }
        virtual_clock::wait_virtual_ns(step);
        reporter.look(&handle, virtual_clock::virtual_ns() - origin, out);
        if !handle.engine_is_alive() {
            drop(actor);
            drop(watch);
            return Err("the engine stopped: a component's model failed".to_string());
        }
    }
    let elapsed = virtual_clock::virtual_ns() - origin;
    if interrupted {
        say!(out, "interrupted at {} of virtual time", ms(elapsed));
    }
    say!(
        out,
        "ran {} of virtual time in {:.3} s",
        ms(elapsed),
        wall.elapsed().as_secs_f64()
    );
    reporter.summary(out);
    for net in &options.nets {
        let state = handle
            .net_state(net)
            .expect("every named net was checked before the run");
        say!(out, "net {net}: {state:?}");
    }
    let findings = handle.findings();
    let stalled = findings
        .iter()
        .any(|finding| matches!(finding, Finding::QuiescenceTimeout { .. }));
    say!(
        out,
        "findings: {} ({} at build, {} while running)",
        findings.len(),
        reporter.at_build,
        findings.len() - reporter.at_build.min(findings.len())
    );
    if !findings.is_empty() {
        let mut holds = Vec::new();
        let mut cleared = Vec::new();
        let mut standing = Vec::new();
        for finding in &findings {
            match now(finding, &handle, &pin_nets) {
                Now::Holds => holds.push(format!("{finding:?}")),
                Now::Cleared(net, state) => {
                    cleared.push(format!("{finding:?}: {net} reads {state:?}"));
                }
                Now::Standing => standing.push(format!("{finding:?}")),
            }
        }
        say!(out, "at {}, each finding's net read again:", ms(elapsed));
        for (title, lines) in [
            ("no longer true", &cleared),
            ("still true", &holds),
            ("about the board as built and wired", &standing),
        ] {
            say!(out, "  {title} ({}):", lines.len());
            for line in lines {
                say!(out, "    {line}");
            }
        }
    }
    if stalled {
        say!(
            out,
            "the engine advanced without waiting for a part (QuiescenceTimeout): this run is \
             not reproducible"
        );
    }
    drop(actor);
    drop(watch);
    handle.shutdown();
    Ok(())
}

#[cfg(test)]
mod tests {
    use embsim_board::parse_duration;
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
