//! The engine's check of the operating limits parts declare on their pins
//! ([`crate::PinLimits`], `PinDecl::with_limits`): one watch per declaring
//! pin, observed against the solved nets — by [`crate::System::build`] once
//! over its settled snapshot, and by the live engine after every pass that
//! moved the pin's net or its reference. A part declares what its
//! datasheet allows; whether the board keeps to it is the engine's to say.

use crate::component::PinLimits;
use crate::diagnostics::Finding;
use crate::net::{Net, Volts};

/// One declaring pin, as the engine watches it: where it sits, what it is
/// measured against, its limits, and whether its last observation was
/// above the recommended range — so an excursion is reported once, at its
/// first voltage, and a pin that comes back into range (or to no voltage)
/// and leaves it again is reported again.
#[derive(Debug, Clone)]
pub(crate) struct LimitWatch {
    /// The part, as `Board.Reference`.
    part: String,
    /// The pin, by number.
    pin: String,
    /// The global net the pin is on.
    net: usize,
    /// The global net of the pin's declared reference, when it declares
    /// one; the engine's 0 V otherwise.
    reference: Option<usize>,
    limits: PinLimits,
    /// Whether the last observation was above the recommended range.
    above: bool,
}

impl LimitWatch {
    /// A watch on `part`'s pin `pin` on `net`, measured against
    /// `reference`'s net, not yet observed.
    pub(crate) fn new(
        part: String,
        pin: &str,
        net: usize,
        reference: Option<usize>,
        limits: PinLimits,
    ) -> Self {
        Self {
            part,
            pin: pin.to_string(),
            net,
            reference,
            limits,
            above: false,
        }
    }

    /// Whether a pass that moved `net` can have moved what the watch
    /// measures: the pin's own net or its reference's.
    pub(crate) fn depends_on(&self, net: usize) -> bool {
        self.net == net || self.reference == Some(net)
    }

    /// The pin's voltage against its reference, when both nets name one
    /// (a floating node, an unmodelled rail or a periodic node names none,
    /// and nothing is measured).
    fn volts(&self, nets: &[Net]) -> Option<Volts> {
        let node = nets.get(self.net)?.volts.dc?;
        let reference = match self.reference {
            Some(reference) => nets.get(reference)?.volts.dc?,
            None => 0.0,
        };
        Some(node - reference)
    }

    /// Observe the pin on `nets`: the finding when the pin has just risen
    /// above its recommended range, `None` while it stays there, stays
    /// inside it, or names no voltage.
    pub(crate) fn observe(&mut self, nets: &[Net]) -> Option<Finding> {
        let volts = self
            .volts(nets)
            .filter(|&volts| volts > self.limits.recommended.1);
        let entered = volts.is_some() && !self.above;
        self.above = volts.is_some();
        let volts = volts.filter(|_| entered)?;
        let (min, max) = self.limits.recommended;
        Some(Finding::PinAboveRecommended {
            part: self.part.clone(),
            pin: self.pin.clone(),
            volts,
            min,
            max,
            absolute_max: self.limits.absolute_max,
            note: self.limits.note.to_string(),
        })
    }
}
