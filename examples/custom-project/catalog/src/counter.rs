//! `example-edge-counter`: a bench instrument that counts rising edges.
//!
//! Two pins: `IN`, read against `REF`, the instrument's ground lead. A
//! rising edge is `IN` reading high after it read low, through the two
//! trigger levels the project gives it — low at or below `low`, high at or
//! above `high`, and between them the level it last read, as an
//! instrument's trigger with hysteresis does. The levels are the bench's
//! to choose, so both are required: the kind invents neither
//! (`DESIGN.md` rule 6).
//!
//! It drives nothing. Each edge is counted at the instant the engine hands
//! it the sense (`Sense::at_ns`), and the run prints them: each new edge at
//! the look that finds it, and at the end how many there were and how far
//! apart.

use std::sync::{Arc, Mutex};

use embsim_board::report::instant;
use embsim_board::{
    AttachError, Component, ComponentNetIo, ComponentRequest, DeadBand, DigitalReceiver, Level,
    PinDecl, ProjectError, Report, Thresholds,
};

/// The edges seen, shared with the report.
#[derive(Default)]
struct Edges {
    /// The instant of every rising edge, in order.
    at_ns: Vec<u64>,
}

/// Build the counter a `[[component]]` names.
pub fn build(request: ComponentRequest<'_>) -> Result<Box<dyn Component>, ProjectError> {
    let ComponentRequest {
        spec,
        mut options,
        reports,
        ..
    } = request;
    let low = options
        .number("low")?
        .ok_or_else(|| options.error("options.low is the level IN reads low at or below"))?;
    let high = options
        .number("high")?
        .ok_or_else(|| options.error("options.high is the level IN reads high at or above"))?;
    if low >= high {
        return Err(options.error(format!(
            "options.low = {low} is not below options.high = {high}"
        )));
    }
    options.finish()?;
    let edges = Arc::new(Mutex::new(Edges::default()));
    reports.add(CounterReport {
        subject: spec.name.clone(),
        edges: Arc::clone(&edges),
        said: 0,
    });
    Ok(Box::new(Counter {
        pins: [
            PinDecl::digital_in("IN", Thresholds::new(low, high, 0.0, DeadBand::HoldLast))
                .with_reference("REF"),
            PinDecl::power_in("REF"),
        ],
        edges,
    }))
}

/// The instrument.
struct Counter {
    pins: [PinDecl; 2],
    edges: Arc<Mutex<Edges>>,
}

impl Component for Counter {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let receiver = DigitalReceiver::new(io.pin("IN")?);
        let edges = Arc::clone(&self.edges);
        io.on_sense("IN", move |sense| {
            let before = receiver.last();
            let now = receiver.read(&sense);
            if before == Some(Level::Low) && now == Some(Level::High) {
                edges
                    .lock()
                    .expect("never poisoned")
                    .at_ns
                    .push(sense.at_ns);
            }
        })
    }
}

/// What the counter says.
struct CounterReport {
    subject: String,
    edges: Arc<Mutex<Edges>>,
    /// How many edges the run has printed.
    said: usize,
}

impl Report for CounterReport {
    fn subject(&self) -> String {
        self.subject.clone()
    }

    fn look(&mut self, _now_ns: u64) -> Vec<String> {
        let edges = self.edges.lock().expect("never poisoned");
        let lines = edges.at_ns[self.said..]
            .iter()
            .enumerate()
            .map(|(index, at)| {
                format!(
                    "rising edge {} on IN at {}",
                    self.said + index + 1,
                    instant(*at)
                )
            })
            .collect();
        self.said = edges.at_ns.len();
        lines
    }

    fn summary(&self) -> Vec<String> {
        let edges = self.edges.lock().expect("never poisoned");
        let at = &edges.at_ns;
        let (Some(first), Some(last)) = (at.first(), at.last()) else {
            return vec!["no rising edge on IN".to_string()];
        };
        let gaps: Vec<u64> = at.windows(2).map(|pair| pair[1] - pair[0]).collect();
        let spacing = match (gaps.iter().min(), gaps.iter().max()) {
            (Some(min), Some(max)) if min == max => format!(", every {}", instant(*min)),
            (Some(min), Some(max)) => {
                format!(", {} to {} apart", instant(*min), instant(*max))
            }
            _ => String::new(),
        };
        vec![format!(
            "{} rising edges on IN, the first at {} and the last at {}{spacing}",
            at.len(),
            instant(*first),
            instant(*last)
        )]
    }
}
