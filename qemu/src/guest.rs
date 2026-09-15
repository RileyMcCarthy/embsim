//! What a [`QemuNode`](crate::QemuNode) meters.

use std::io;
use std::os::fd::RawFd;

/// A computer the node can freeze, thaw, and talk to over a serial port.
///
/// The node calls [`resume`](Self::resume) at the start of every slice and
/// [`pause`](Self::pause) at its end; between the two the guest runs at wall
/// speed and the node services [`serial_fd`](Self::serial_fd). A guest is
/// born paused: nothing runs before the first slice.
///
/// [`QemuVm`](crate::QemuVm) is the production implementation. Tests
/// implement this over a socketpair and a stopwatch to exercise the whole
/// node with no QEMU installed.
pub trait Guest: Send {
    /// Let the guest run; its clock advances from here until [`pause`](Self::pause).
    fn resume(&mut self) -> io::Result<()>;

    /// Freeze the guest and its clock.
    fn pause(&mut self) -> io::Result<()>;

    /// The guest's serial port: a non-blocking, bidirectional descriptor the
    /// node polls, reads and writes for as long as the guest lives.
    fn serial_fd(&self) -> RawFd;

    /// The guest's own monotonic clock, in nanoseconds, if it can be read.
    ///
    /// Called once per slice, immediately after [`resume`](Self::resume),
    /// while the guest is running. The node uses the difference between two
    /// consecutive readings as the exact length of the slice between them —
    /// whatever latency the read has, it has at both ends and cancels — and
    /// falls back to its own stopwatch when this returns `None`. The
    /// stopwatch is good to a tenth of a millisecond per slice but biased,
    /// so a long run drifts about a percent at 10 ms slices; a guest that
    /// can answer this question does not drift at all.
    fn clock_ns(&mut self) -> Option<u64> {
        None
    }
}
