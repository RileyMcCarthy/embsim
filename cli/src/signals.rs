//! Ending a run from outside: SIGINT (Ctrl-C) and SIGTERM.
//!
//! While a run holds a [`Watch`], the first SIGINT or SIGTERM only sets a
//! flag: the run sees it at its next look, says it was interrupted, and
//! prints the same summary as a run that reached its `--for`. The handler
//! puts the default action back as it returns, so a second signal ends the
//! process at once — a run that does not reach its next look (an engine
//! wedged in a part's model) can still be stopped. Dropping the watch puts
//! back whatever handlers were there before.

use std::sync::atomic::{AtomicBool, Ordering};

/// Set by the handler; read by the run at each look.
static INTERRUPTED: AtomicBool = AtomicBool::new(false);

extern "C" fn on_signal(_signal: libc::c_int) {
    INTERRUPTED.store(true, Ordering::SeqCst);
    // SAFETY: `signal` is async-signal-safe (POSIX.1-2008, 2.4.3), and so
    // is a store to an atomic.
    unsafe {
        libc::signal(libc::SIGINT, libc::SIG_DFL);
        libc::signal(libc::SIGTERM, libc::SIG_DFL);
    }
}

/// The run's handlers, installed for as long as it lives.
pub struct Watch {
    previous_int: libc::sighandler_t,
    previous_term: libc::sighandler_t,
}

impl Watch {
    /// Install the handlers, with nothing interrupted yet.
    pub fn install() -> Self {
        INTERRUPTED.store(false, Ordering::SeqCst);
        let handler: extern "C" fn(libc::c_int) = on_signal;
        // SAFETY: the handler only stores to an atomic and calls `signal`,
        // both async-signal-safe.
        unsafe {
            Self {
                previous_int: libc::signal(libc::SIGINT, handler as libc::sighandler_t),
                previous_term: libc::signal(libc::SIGTERM, handler as libc::sighandler_t),
            }
        }
    }

    /// Whether a signal arrived since the handlers were installed.
    pub fn interrupted(&self) -> bool {
        INTERRUPTED.load(Ordering::SeqCst)
    }
}

impl Drop for Watch {
    fn drop(&mut self) {
        // SAFETY: putting back the handlers `signal` handed out.
        unsafe {
            libc::signal(libc::SIGINT, self.previous_int);
            libc::signal(libc::SIGTERM, self.previous_term);
        }
    }
}
