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

    /// Whether the guest's serial port is currently attached.
    ///
    /// A node still runs the guest while this is false -- an unplugged port is
    /// the point of the test and the guest has to keep executing to notice it
    /// -- but it moves no bytes for that slice.
    fn serial_attached(&self) -> bool {
        true
    }

    /// Attach or detach the guest's serial port, as a cable would.
    ///
    /// The default is a no-op for guests whose port cannot be unplugged. For
    /// QEMU this closes the chardev the emulated USB serial adapter is backed
    /// by, which QEMU turns into a real USB detach (`usb_serial_event` maps
    /// `CHR_EVENT_CLOSED` onto `usb_device_detach` unless the device was
    /// created `always-plugged`), so the guest kernel removes the tty and a
    /// browser sees a genuine disconnect rather than a simulated one.
    fn set_serial_attached(&mut self, attached: bool) -> io::Result<()> {
        let _ = attached;
        Err(io::Error::new(
            io::ErrorKind::Unsupported,
            "this guest's serial port cannot be unplugged",
        ))
    }

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
