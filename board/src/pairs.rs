//! The engine's watch on the differential pairs parts declare on their
//! inputs ([`crate::DifferentialPair`], `PinDecl::with_pair`; `NODES.md`
//! §20): one watch per declared pair, observed against the solved nets —
//! by [`crate::System::build`] once over its settled snapshot, and by the
//! live engine after every pass that moved either input's net or their
//! reference — and judged when the pair has spent time inside its band:
//! at the build's snapshot, and live when the engine's clock moves on.
//!
//! The judgement is the one question an observation cannot answer from
//! the voltages alone: whether a source other than the part's own input
//! ports reaches the pair. Only the part's ports reaching it is the
//! datasheet's open, shorted or idle-terminated pair, which rests at their
//! bias and is the part's fail-safe to decide; anything else is a
//! common-mode voltage applied, inside a band the datasheet gives no level
//! in. The judgement walks the pair's clusters, so it runs only for a pair
//! that has stayed in its band past an instant and has not been reported
//! (a silent one, resting at its own bias, again each time its nets move):
//! a valid pair costs one subtraction per pass that moves it, and a
//! reported one nothing.
//!
//! Live, a pair is judged only when the clock moves on: one that enters
//! its band at the run's last instant, or while the clock never moves
//! again, is not judged, and the findings read then do not name it.
//! Judging when the findings are read, from another thread at any point in
//! an instant, could catch a line driver's pair between its two legs.

use std::ops::Range;

use crate::component::DifferentialPair;
use crate::diagnostics::Finding;
use crate::net::{Net, Volts};

/// One declared pair, as the engine watches it.
#[derive(Debug, Clone)]
pub(crate) struct PairWatch {
    /// The part, as `Board.Reference`.
    part: String,
    /// The non-inverting input, by number.
    pin: String,
    /// The inverting input, by number.
    inverting: String,
    /// The global nets of the two inputs.
    nets: (usize, usize),
    /// The global net of the inputs' declared reference, when they declare
    /// one; the engine's 0 V otherwise.
    reference: Option<usize>,
    pair: DifferentialPair,
    /// The part's drive slots, in the resolver's table: the input ports
    /// among them are the part's own bias, which the judgement looks past.
    own: Range<usize>,
    /// The last observation inside the band, `(difference, common mode)`,
    /// while it waits to be judged; `None` outside the band, with no
    /// voltage, or once judged.
    pending: Option<(Volts, Volts)>,
    /// Whether the pair has been reported: once per pair, and then the
    /// watch is done.
    reported: bool,
}

impl PairWatch {
    /// A watch on `part`'s pair, its inputs `pins` by number on `nets`,
    /// measured against `reference`'s net, its own slots `own`, not yet
    /// observed.
    pub(crate) fn new(
        part: String,
        pins: (&str, &str),
        nets: (usize, usize),
        reference: Option<usize>,
        pair: DifferentialPair,
        own: Range<usize>,
    ) -> Self {
        Self {
            part,
            pin: pins.0.to_string(),
            inverting: pins.1.to_string(),
            nets,
            reference,
            pair,
            own,
            pending: None,
            reported: false,
        }
    }

    /// The two input nets, for the engine's index of which watch a moved
    /// net touches.
    pub(crate) fn nets(&self) -> [usize; 2] {
        [self.nets.0, self.nets.1]
    }

    /// The reference's net, when the pair declares one.
    pub(crate) fn reference(&self) -> Option<usize> {
        self.reference
    }

    /// The part's own slots ([`Self::own`]).
    pub(crate) fn own(&self) -> Range<usize> {
        self.own.clone()
    }

    /// Whether the watch has an observation inside the band waiting to be
    /// judged.
    pub(crate) fn is_pending(&self) -> bool {
        self.pending.is_some()
    }

    /// `(difference, common mode)` when both inputs and the reference
    /// name a voltage (a floating node, a periodic node or an unmodelled
    /// rail names none, and nothing is measured).
    fn reading(&self, nets: &[Net]) -> Option<(Volts, Volts)> {
        let a = nets.get(self.nets.0)?.volts.dc?;
        let b = nets.get(self.nets.1)?.volts.dc?;
        let reference = match self.reference {
            Some(reference) => nets.get(reference)?.volts.dc?,
            None => 0.0,
        };
        Some((a - b, (a + b) / 2.0 - reference))
    }

    /// Observe the pair on `nets`: an observation inside the band waits to
    /// be judged; one outside it, or with no voltage, clears what was
    /// waiting. Nothing once the pair has been reported.
    pub(crate) fn observe(&mut self, nets: &[Net]) {
        if self.reported {
            return;
        }
        self.pending = self
            .reading(nets)
            .filter(|&(difference, _)| difference.abs() < self.pair.band);
    }

    /// Judge the observation waiting, given whether a source other than
    /// the part's own input ports reaches either input: the finding when
    /// one does, once per pair. The observation is spent either way, so a
    /// pair resting at its own bias is judged again only after its nets
    /// move.
    pub(crate) fn judge(&mut self, common_mode_applied: bool) -> Option<Finding> {
        let (volts, common_mode) = self.pending.take()?;
        if !common_mode_applied {
            return None;
        }
        self.reported = true;
        Some(Finding::PairInBand {
            part: self.part.clone(),
            pin: self.pin.clone(),
            inverting: self.inverting.clone(),
            volts,
            common_mode,
            band: self.pair.band,
            output: self.pair.fail_safe,
        })
    }
}
