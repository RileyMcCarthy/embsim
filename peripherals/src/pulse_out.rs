//! Pulse Out — generic timed pulse emission peripheral.
//!
//! Models the abstract behavior every pulse-output peripheral exposes:
//! "emit `N` pulses at frequency `F` starting now, tell me how many have
//! gone out so far, stop on request." It is intentionally hardware-agnostic
//! — whether a real MCU implements the channel with a smart pin, a hardware
//! timer, DMA, or software bit-bang is irrelevant to anything downstream of
//! `HAL_pulseOut_run`. All of those produce the same observable behavior:
//! a monotonically-increasing emitted count that reaches `N` after `N / F`
//! seconds.
//!
//! ## Single source of truth
//!
//! `run()` integrates `frequency × elapsed_virtual_time` and is what firmware
//! reads through `HAL_pulseOut_run`. Encoder feedback and kinematic models
//! subscribe via [`on_progress`] and receive the **same integer** the firmware
//! sees on the same call, so they cannot drift from the firmware's view.
//!
//! ## Rate changes, not edges
//!
//! [`on_progress`] fires once per `run()` poll, which is a *sampling* seam: it
//! only ever reports what the integrator already knew. [`on_rate_change`] is
//! the **event** seam — it fires exactly when the commanded rate changes
//! (`start`, `start_velocity`, `set_frequency`, `stop`) and hands over a
//! [`PulseSegment`] describing the constant-rate segment that just began.
//! A subscriber can reconstruct the emitted count at *any* virtual instant
//! from that one value ([`PulseSegment::emitted_at`]) using the same integer
//! arithmetic `run()` uses, so it never has to observe individual pulses.
//!
//! This is what lets a pulse train cross a board-engine net without one event
//! per step: `embsim_board::mcu` bridges this callback onto a `PulseSource`
//! pin, and the consumer integrates at read time. At 8192 steps/mm, one
//! mm/s of carriage speed is 8192 pulses/s and **one** rate-change event.
//!
//! ## Core occupancy
//!
//! `run()` sleeps for `POLL_TICK_US` of virtual time between polls when the
//! sequence is still in progress. This bounds the calling core's polling rate
//! without tying it to the pulse frequency, and lets the scheduler service
//! other cores while the pulse train continues. When the sequence is complete
//! the call returns immediately so the core can move on to the next move.
//!
//! ## Concurrency
//!
//! Per-channel state is protected by a single per-instance mutex held only
//! across field reads/writes — never across a sleep — so multiple cores can
//! drive independent channels in parallel without false serialization. Two
//! cores driving the *same* channel is undefined (just like sharing an output
//! pin on real hardware).
//!
//! State lives in a per-MCU [`PulseOut`] bank owned by
//! `instance::PeripheralInstance`. The module-level free functions route to
//! the calling thread's instance (see `crate::instance`), so existing
//! single-MCU consumers are unaffected.

use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Mutex;
use tracing::trace;

/// Polling cadence of `run()` while a sequence is in progress (virtual µs).
///
/// Bounds core occupancy without coupling to the configured pulse frequency.
/// On every poll, `on_progress` fires the running emitted count so the encoder
/// atomic and downstream physics stay in sync with virtual time. Anything that
/// reads the encoder afterwards will see a value at most `POLL_TICK_US` stale.
///
/// This must be **substantially finer** than the rate at which the encoder is
/// read by firmware (a typical monitor loop runs at ~1 ms). A matching cadence aliases
/// against the read clock and produces visible "stutter" — every few samples
/// land in the same encoder window and report a 0-µm delta. 250 µs gives a
/// clean 4:1 oversample of the 1 ms read rate, eliminating the artifact while
/// keeping trace volume manageable.
const POLL_TICK_US: u64 = 250;

/// Maximum pulse out channels supported (hard ceiling of the backing array).
pub const MAX_CHANNELS: usize = 16;

/// One constant-rate segment of a pulse train — the payload of a
/// [`PulseOut::on_rate_change`] event.
///
/// A segment is the whole truth about the channel from `since_us` onward: a
/// subscriber that keeps the latest segment can compute the emitted count at
/// any later virtual instant with [`PulseSegment::emitted_at`], which is
/// **bit-identical** to what [`PulseOut::run`] hands the firmware at that same
/// instant. That equality is the contract: a downstream plant and the firmware
/// can never disagree about how many pulses went out.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PulseSegment {
    /// Pulses emitted in this train *before* `since_us`. Zero for a freshly
    /// started train; the banked total for a velocity retarget.
    pub emitted: u64,
    /// Pulse rate from `since_us` onward, in Hz. `0` holds the count (a
    /// stopped or held channel).
    pub freq_hz: u32,
    /// Cumulative pulse ceiling for a finite train (`start`), or `None` for an
    /// unbounded continuous-velocity train (`start_velocity`).
    pub total: Option<u64>,
    /// Virtual time (µs) at which this segment began.
    pub since_us: u64,
}

impl PulseSegment {
    /// A held channel: nothing emitted, no rate, and a ceiling of zero —
    /// exactly what a bank channel reports before its first train, so an
    /// unconfigured channel and a fresh one are indistinguishable to a
    /// subscriber.
    pub const IDLE: Self = Self {
        emitted: 0,
        freq_hz: 0,
        total: Some(0),
        since_us: 0,
    };

    /// Cumulative pulses emitted by this train at virtual time `now_us`.
    ///
    /// Deliberately the same integer arithmetic as [`PulseOut::run`]:
    /// `emitted + elapsed_us × freq / 1_000_000`, clamped to `total`. A
    /// `now_us` before `since_us` reads the segment's baseline.
    pub fn emitted_at(&self, now_us: u64) -> u64 {
        let elapsed = now_us.saturating_sub(self.since_us);
        let grown = self
            .emitted
            .saturating_add(elapsed.saturating_mul(u64::from(self.freq_hz)) / 1_000_000);
        match self.total {
            Some(total) => grown.min(total),
            None => grown,
        }
    }

    /// Virtual time (µs) at which a finite train has emitted its last pulse,
    /// or `None` for an unbounded or held train. Past that instant
    /// [`Self::emitted_at`] is constant.
    pub fn completes_at(&self) -> Option<u64> {
        let total = self.total?;
        if self.freq_hz == 0 {
            return None;
        }
        let remaining = total.saturating_sub(self.emitted);
        Some(
            self.since_us.saturating_add(
                remaining
                    .saturating_mul(1_000_000)
                    .div_ceil(u64::from(self.freq_hz)),
            ),
        )
    }

    /// The same train re-anchored at `at_us`: identical rate and ceiling, with
    /// `emitted` advanced to the count at that instant.
    ///
    /// The re-based segment's baseline is exactly the count at `at_us`, so
    /// folding `emitted` differences across a re-base can neither double-count
    /// a pulse nor lose one. What re-basing *does* discard is the source's
    /// **pulse phase**: the re-based segment restarts its period at `at_us`,
    /// so from then on it can trail the un-re-based integration by up to one
    /// pulse. Integer microseconds cannot represent a mid-pulse phase, so this
    /// is a floor, not an implementation shortcut.
    ///
    /// The rule that follows: **re-base at segment boundaries, never on every
    /// read.** A rate or direction change is a boundary the source is
    /// publishing anyway (one truncation, at an instant the machine is
    /// changing state); a consumer that wants a running count reads
    /// [`Self::emitted_at`] against the published anchor instead
    /// (`embsim_models::machine::stepper_motor` is the reference).
    pub fn rebased_at(&self, at_us: u64) -> Self {
        Self {
            emitted: self.emitted_at(at_us),
            since_us: at_us.max(self.since_us),
            ..*self
        }
    }
}

#[derive(Clone, Copy)]
struct PulseState {
    total_pulses: u32,
    frequency: u32,
    start_us: u64,
    /// Continuous-velocity (NCO) mode: an unbounded train whose rate can be
    /// retargeted on the fly. `emitted_base` carries the cumulative pulse count
    /// from before the latest `set_frequency`, so the running total stays
    /// monotonic across rate changes.
    velocity_mode: bool,
    emitted_base: u64,
}

const PULSE_STATE_INIT: PulseState = PulseState {
    total_pulses: 0,
    frequency: 1,
    start_us: 0,
    velocity_mode: false,
    emitted_base: 0,
};

impl PulseState {
    /// This channel's current constant-rate segment, in the vocabulary a
    /// [`PulseOut::on_rate_change`] subscriber consumes.
    ///
    /// An idle channel (`total_pulses == 0`, i.e. never started or stopped)
    /// reports a **held** segment — no rate, and a ceiling equal to the count
    /// already banked — so a subscriber that folds rate changes sees "stopped
    /// at N" rather than a stale rate or a phantom rewind to zero. A channel
    /// that was never started banks nothing, so it reads zero.
    fn segment(&self) -> PulseSegment {
        if self.total_pulses == 0 {
            return PulseSegment {
                emitted: self.emitted_base,
                freq_hz: 0,
                total: Some(self.emitted_base),
                since_us: self.start_us,
            };
        }
        PulseSegment {
            emitted: self.emitted_base,
            freq_hz: self.frequency,
            total: (!self.velocity_mode).then_some(u64::from(self.total_pulses)),
            since_us: self.start_us,
        }
    }
}

/// One optional per-channel callback fired when a pulse train starts,
/// carrying `(total_pulses, frequency)`.
type StartCallback = Option<Box<dyn Fn(u32, u32) + Send>>;
/// One optional per-channel callback fired when a pulse train stops.
type StopCallback = Option<Box<dyn Fn() + Send>>;
/// One optional per-channel callback fired on progress, carrying the
/// cumulative emitted-pulse count.
type ProgressCallback = Option<Box<dyn Fn(u32) + Send>>;
/// One optional per-channel callback fired when the commanded rate changes,
/// carrying the constant-rate segment that just began.
type RateCallback = Option<Box<dyn Fn(PulseSegment) + Send>>;

/// Pulse-output channel bank for one MCU instance.
pub struct PulseOut {
    count: AtomicUsize,
    start_callbacks: Mutex<Vec<StartCallback>>,
    stop_callbacks: Mutex<Vec<StopCallback>>,
    progress_callbacks: Mutex<Vec<ProgressCallback>>,
    rate_callbacks: Mutex<Vec<RateCallback>>,
    state: Mutex<[PulseState; MAX_CHANNELS]>,
}

impl PulseOut {
    /// Create a bank with no channels configured and no callbacks.
    pub const fn new() -> Self {
        Self {
            count: AtomicUsize::new(0),
            start_callbacks: Mutex::new(Vec::new()),
            stop_callbacks: Mutex::new(Vec::new()),
            progress_callbacks: Mutex::new(Vec::new()),
            rate_callbacks: Mutex::new(Vec::new()),
            state: Mutex::new([PULSE_STATE_INIT; MAX_CHANNELS]),
        }
    }

    /// Configure the peripheral with the number of channels.
    /// Resets all per-channel callbacks and pulse state, so re-init is a clean start.
    ///
    /// # Panics
    /// If `count` exceeds [`MAX_CHANNELS`].
    pub fn init(&self, count: usize) {
        assert!(
            count <= MAX_CHANNELS,
            "PulseOut count {} exceeds max {}",
            count,
            MAX_CHANNELS
        );
        self.reset();
        self.count.store(count, Ordering::Relaxed);
        self.start_callbacks
            .lock()
            .unwrap()
            .resize_with(count, || None);
        self.stop_callbacks
            .lock()
            .unwrap()
            .resize_with(count, || None);
        self.progress_callbacks
            .lock()
            .unwrap()
            .resize_with(count, || None);
        self.rate_callbacks
            .lock()
            .unwrap()
            .resize_with(count, || None);
    }

    /// Configured channel count.
    pub fn count(&self) -> usize {
        self.count.load(Ordering::Relaxed)
    }

    /// Clear all channel callbacks and pulse state (used by `init` and teardown).
    pub fn reset(&self) {
        self.count.store(0, Ordering::Relaxed);
        self.start_callbacks.lock().unwrap().clear();
        self.stop_callbacks.lock().unwrap().clear();
        self.progress_callbacks.lock().unwrap().clear();
        self.rate_callbacks.lock().unwrap().clear();
        let mut state = self.state.lock().unwrap();
        for s in state.iter_mut() {
            *s = PULSE_STATE_INIT;
        }
    }

    /// Register a per-channel callback fired when `start()` is called. Useful
    /// for snapshotting baseline state (encoder origin, GPIO direction).
    pub fn on_start(&self, channel: usize, cb: impl Fn(u32, u32) + Send + 'static) {
        register(&self.start_callbacks, channel, Box::new(cb));
    }

    /// Register a per-channel callback fired when `stop()` is called.
    pub fn on_stop(&self, channel: usize, cb: impl Fn() + Send + 'static) {
        register(&self.stop_callbacks, channel, Box::new(cb));
    }

    /// Register a per-channel callback fired with the cumulative `emitted` pulse
    /// count every time progress is re-evaluated. The argument is the **same
    /// integer** the firmware will read from `HAL_pulseOut_run` on the same call,
    /// so subscribers (encoders, physics models) cannot drift from that view.
    pub fn on_progress(&self, channel: usize, cb: impl Fn(u32) + Send + 'static) {
        register(&self.progress_callbacks, channel, Box::new(cb));
    }

    /// Register a per-channel callback fired **only when the commanded rate
    /// changes** — `start`, `start_velocity`, `set_frequency`, `stop` — with
    /// the constant-rate [`PulseSegment`] that just began.
    ///
    /// This is the low-rate seam a pulse train crosses a net on
    /// (`embsim_board::mcu`): the subscriber integrates the segment at read
    /// time instead of observing pulses, so a 100 kHz train costs the same
    /// number of events as a 1 Hz one. One callback per channel; re-registering
    /// replaces it.
    ///
    /// The callback runs on the thread that changed the rate (the firmware's
    /// motion core), with **no pulse-out lock held**, so it may call back into
    /// this bank or into a board engine without deadlocking.
    pub fn on_rate_change(&self, channel: usize, cb: impl Fn(PulseSegment) + Send + 'static) {
        register(&self.rate_callbacks, channel, Box::new(cb));
    }

    fn fire_progress(&self, channel: usize, emitted: u32) {
        if let Ok(cbs) = self.progress_callbacks.lock() {
            if let Some(cb) = cbs.get(channel).and_then(|c| c.as_ref()) {
                cb(emitted);
            }
        }
    }

    /// Publish a new constant-rate segment. Callers must hold **no** lock.
    fn fire_rate_change(&self, channel: usize, segment: PulseSegment) {
        if let Ok(cbs) = self.rate_callbacks.lock() {
            if let Some(cb) = cbs.get(channel).and_then(|c| c.as_ref()) {
                cb(segment);
            }
        }
    }

    /// Cumulative pulses emitted by `channel`'s **current train** at the
    /// current virtual instant, without polling, sleeping, or firing
    /// `on_progress`.
    ///
    /// While a train is running this is the same integer the next
    /// [`PulseOut::run`] would return, so a wiring layer can sample the count
    /// without perturbing the core-occupancy model `run()` implements. After a
    /// [`PulseOut::stop`] it holds the train's frozen final count, where
    /// `run()` reports an idle channel as `0` — the difference is deliberate:
    /// `run()` answers "is there anything left to do", this answers "how many
    /// pulses went out". A channel that was never started reads 0, and each
    /// [`PulseOut::start`] re-bases the count to 0 for the new train.
    pub fn emitted(&self, channel: usize) -> u64 {
        if channel >= self.count.load(Ordering::Relaxed) {
            return 0;
        }
        let segment = self.state.lock().unwrap()[channel].segment();
        segment.emitted_at(embsim_core::virtual_clock::virtual_us())
    }

    /// This channel's current constant-rate segment (the value the last
    /// [`PulseOut::on_rate_change`] event carried). An unconfigured channel
    /// reads [`PulseSegment::IDLE`].
    pub fn segment(&self, channel: usize) -> PulseSegment {
        if channel >= self.count.load(Ordering::Relaxed) {
            return PulseSegment::IDLE;
        }
        self.state.lock().unwrap()[channel].segment()
    }

    /// Start a pulse sequence. Records timing state and fires `on_start` followed
    /// by an initial `on_progress(0)` so subscribers can align with the start
    /// position before any pulses elapse.
    pub fn start(&self, channel: usize, pulses: u32, frequency: u32) {
        if channel >= self.count.load(Ordering::Relaxed) {
            crate::access::report("pulse_out", &format!("start channel {channel}"));
            return;
        }
        let freq = frequency.max(1);

        trace!(
            "pulse_out::start(ch={}, pulses={}, freq={})",
            channel,
            pulses,
            freq
        );

        let segment = {
            let mut state = self.state.lock().unwrap();
            state[channel] = PulseState {
                total_pulses: pulses,
                frequency: freq,
                start_us: embsim_core::virtual_clock::virtual_us(),
                velocity_mode: false,
                emitted_base: 0,
            };
            state[channel].segment()
        };

        if let Ok(cbs) = self.start_callbacks.lock() {
            if let Some(cb) = cbs.get(channel).and_then(|c| c.as_ref()) {
                cb(pulses, freq);
            }
        }
        self.fire_progress(channel, 0);
        self.fire_rate_change(channel, segment);
    }

    /// Begin (or re-baseline) a continuous-velocity (NCO) pulse train at `frequency`
    /// steps/s. Resets the emitted counter to 0 and fires `on_start` so subscribers
    /// can re-anchor their own state (e.g. snapshot the current direction, or reset a
    /// dt baseline so the first post-restart tick doesn't see a huge interval).
    /// Callers re-invoke this on a direction reversal, where the rate passes through
    /// ~0. `frequency` 0 holds (no pulses).
    pub fn start_velocity(&self, channel: usize, frequency: u32) {
        if channel >= self.count.load(Ordering::Relaxed) {
            crate::access::report("pulse_out", &format!("start_velocity channel {channel}"));
            return;
        }
        trace!(
            "pulse_out::start_velocity(ch={}, freq={})",
            channel,
            frequency
        );
        let segment = {
            let mut state = self.state.lock().unwrap();
            state[channel] = PulseState {
                total_pulses: u32::MAX, // unbounded; velocity mode never "completes"
                frequency,
                start_us: embsim_core::virtual_clock::virtual_us(),
                velocity_mode: true,
                emitted_base: 0,
            };
            state[channel].segment()
        };
        if let Ok(cbs) = self.start_callbacks.lock() {
            if let Some(cb) = cbs.get(channel).and_then(|c| c.as_ref()) {
                cb(0, frequency);
            }
        }
        self.fire_progress(channel, 0);
        self.fire_rate_change(channel, segment);
    }

    /// Retarget the continuous-velocity rate without resetting the emitted counter.
    /// The pulses already emitted at the previous rate are banked into `emitted_base`
    /// so the running total stays monotonic. No-op outside velocity mode.
    pub fn set_frequency(&self, channel: usize, frequency: u32) {
        if channel >= self.count.load(Ordering::Relaxed) {
            crate::access::report("pulse_out", &format!("set_frequency channel {channel}"));
            return;
        }
        let segment = {
            let mut state = self.state.lock().unwrap();
            let s = &mut state[channel];
            if !s.velocity_mode {
                return;
            }
            let now = embsim_core::virtual_clock::virtual_us();
            let elapsed = now.saturating_sub(s.start_us);
            let emitted_at_old = elapsed.saturating_mul(s.frequency as u64) / 1_000_000;
            s.emitted_base = s.emitted_base.saturating_add(emitted_at_old);
            s.frequency = frequency;
            s.start_us = now;
            s.segment()
        };
        self.fire_rate_change(channel, segment);
    }

    /// Current commanded pulse frequency (steps/s) for `channel`, or `0` when
    /// the channel is idle (never started, or stopped). Plant models
    /// integrate this *commanded* velocity (× direction) instead of the running
    /// emitted count, which sidesteps the sub-pulse-per-tick truncation that the
    /// integer emitted total suffers at low rates.
    pub fn frequency(&self, channel: usize) -> u32 {
        if channel >= self.count.load(Ordering::Relaxed) {
            return 0;
        }
        self.state.lock().unwrap()[channel].frequency
    }

    /// Poll a running pulse sequence. Returns `(emitted_pulses, done)`.
    ///
    /// `emitted` advances monotonically with virtual time at the configured rate
    /// and is clamped to `total`. The call sleeps for `POLL_TICK_US` of virtual
    /// time when the sequence is still in progress, returning immediately once
    /// `done = true` so the caller can move on without an extra tick of latency.
    pub fn run(&self, channel: usize) -> (u32, bool) {
        if channel >= self.count.load(Ordering::Relaxed) {
            crate::access::report("pulse_out", &format!("run channel {channel}"));
            return (0, true);
        }

        // Snapshot state — never hold the lock across a sleep.
        let snapshot = {
            let state = self.state.lock().unwrap();
            state[channel]
        };

        if snapshot.total_pulses == 0 {
            return (0, true);
        }

        let now = embsim_core::virtual_clock::virtual_us();
        let elapsed_us = now.saturating_sub(snapshot.start_us);

        // Continuous-velocity mode: cumulative emitted = banked + rate × elapsed.
        // Never completes (the caller stops it); still yields the core via the poll
        // sleep and fires progress so the encoder/physics track virtual time.
        if snapshot.velocity_mode {
            let emitted = snapshot
                .emitted_base
                .saturating_add(elapsed_us.saturating_mul(snapshot.frequency as u64) / 1_000_000)
                as u32;
            self.fire_progress(channel, emitted);
            sleep_virtual_us(POLL_TICK_US);
            return (emitted, false);
        }

        let emitted = ((elapsed_us.saturating_mul(snapshot.frequency as u64)) / 1_000_000)
            .min(snapshot.total_pulses as u64) as u32;
        let done = emitted >= snapshot.total_pulses;

        trace!(
            "pulse_out::run(ch={}): {}/{} elapsed={}us done={}",
            channel,
            emitted,
            snapshot.total_pulses,
            elapsed_us,
            done
        );

        self.fire_progress(channel, emitted);

        if !done {
            sleep_virtual_us(POLL_TICK_US);
        }

        (emitted, done)
    }

    /// Stop a running pulse sequence and fire the `on_stop` callback.
    pub fn stop(&self, channel: usize) {
        trace!("pulse_out::stop(ch={})", channel);
        if channel >= self.count.load(Ordering::Relaxed) {
            crate::access::report("pulse_out", &format!("stop channel {channel}"));
            return;
        }
        // Stopping freezes the count at whatever had gone out and banks it, so
        // a rate-change subscriber's total matches the firmware's last `run()`
        // exactly rather than losing the tail of the train — and so a later
        // `emitted()`/`segment()` poll reads the same frozen number instead of
        // appearing to rewind to zero.
        let now = embsim_core::virtual_clock::virtual_us();
        let segment = {
            let mut state = self.state.lock().unwrap();
            let emitted = state[channel].segment().emitted_at(now);
            let s = &mut state[channel];
            s.emitted_base = emitted;
            s.total_pulses = 0;
            s.velocity_mode = false;
            s.frequency = 0;
            s.start_us = now;
            s.segment()
        };
        if let Ok(cbs) = self.stop_callbacks.lock() {
            if let Some(cb) = cbs.get(channel).and_then(|c| c.as_ref()) {
                cb();
            }
        }
        self.fire_rate_change(channel, segment);
    }
}

impl Default for PulseOut {
    fn default() -> Self {
        Self::new()
    }
}

fn register<F: ?Sized>(slot: &Mutex<Vec<Option<Box<F>>>>, channel: usize, cb: Box<F>) {
    if channel >= MAX_CHANNELS {
        return;
    }
    let mut cbs = slot.lock().unwrap();
    if channel >= cbs.len() {
        cbs.resize_with(channel + 1, || None);
    }
    cbs[channel] = Some(cb);
}

/// Park the pulse-train thread for one span of virtual time.
///
/// Thin alias for [`embsim_core::virtual_clock::wait_virtual_us`] — kept so
/// the pulse-train loop reads in its own vocabulary, but the wait itself lives
/// in the clock's single chokepoint (`DETERMINISM.md` T1 §5).
fn sleep_virtual_us(virtual_us: u64) {
    embsim_core::virtual_clock::wait_virtual_us(virtual_us);
}

// ============================================================
// Free functions — route to the calling thread's instance
// ============================================================

/// Configure the peripheral with the number of channels.
/// Resets all per-channel callbacks and pulse state, so re-init is a clean start.
pub fn init(count: usize) {
    crate::instance::current().pulse_out.init(count);
}

/// Clear all channel callbacks and pulse state (used by `init` and teardown).
pub fn reset() {
    crate::instance::current().pulse_out.reset();
}

/// Register a per-channel callback fired when `start()` is called. Useful
/// for snapshotting baseline state (encoder origin, GPIO direction).
pub fn on_start(channel: usize, cb: impl Fn(u32, u32) + Send + 'static) {
    crate::instance::current().pulse_out.on_start(channel, cb);
}

/// Register a per-channel callback fired when `stop()` is called.
pub fn on_stop(channel: usize, cb: impl Fn() + Send + 'static) {
    crate::instance::current().pulse_out.on_stop(channel, cb);
}

/// Register a per-channel callback fired with the cumulative `emitted` pulse
/// count every time progress is re-evaluated. See [`PulseOut::on_progress`].
pub fn on_progress(channel: usize, cb: impl Fn(u32) + Send + 'static) {
    crate::instance::current()
        .pulse_out
        .on_progress(channel, cb);
}

/// Register a per-channel callback fired only when the commanded rate
/// changes. See [`PulseOut::on_rate_change`].
pub fn on_rate_change(channel: usize, cb: impl Fn(PulseSegment) + Send + 'static) {
    crate::instance::current()
        .pulse_out
        .on_rate_change(channel, cb);
}

/// Cumulative pulses emitted on `channel` at the current virtual instant,
/// without polling or sleeping. See [`PulseOut::emitted`].
pub fn emitted(channel: usize) -> u64 {
    crate::instance::current().pulse_out.emitted(channel)
}

/// This channel's current constant-rate segment. See [`PulseOut::segment`].
pub fn segment(channel: usize) -> PulseSegment {
    crate::instance::current().pulse_out.segment(channel)
}

/// Start a pulse sequence. See [`PulseOut::start`].
pub fn start(channel: usize, pulses: u32, frequency: u32) {
    crate::instance::current()
        .pulse_out
        .start(channel, pulses, frequency);
}

/// Begin (or re-baseline) a continuous-velocity (NCO) pulse train at `frequency`
/// steps/s. See [`PulseOut::start_velocity`].
pub fn start_velocity(channel: usize, frequency: u32) {
    crate::instance::current()
        .pulse_out
        .start_velocity(channel, frequency);
}

/// Retarget the continuous-velocity rate without resetting the emitted counter.
/// See [`PulseOut::set_frequency`].
pub fn set_frequency(channel: usize, frequency: u32) {
    crate::instance::current()
        .pulse_out
        .set_frequency(channel, frequency);
}

/// Current commanded pulse frequency (steps/s) for `channel`. See
/// [`PulseOut::frequency`].
pub fn frequency(channel: usize) -> u32 {
    crate::instance::current().pulse_out.frequency(channel)
}

/// Poll a running pulse sequence. Returns `(emitted_pulses, done)`.
/// See [`PulseOut::run`].
pub fn run(channel: usize) -> (u32, bool) {
    crate::instance::current().pulse_out.run(channel)
}

/// Stop a running pulse sequence and fire the `on_stop` callback.
pub fn stop(channel: usize) {
    crate::instance::current().pulse_out.stop(channel);
}

// ============================================================
// Tests
// ============================================================

#[cfg(test)]
mod tests {
    use rstest::rstest;

    use super::*;
    use std::sync::{
        atomic::{AtomicU32, Ordering as AtomicOrdering},
        Arc, Mutex,
    };

    /// Take the crate-wide test lock, pin the shared virtual clock, and reset
    /// pulse-out state to a clean `channels`-wide bank. `init` fully clears all
    /// per-channel callbacks and pulse state, so no manual clearing is needed.
    fn test_setup(channels: usize) {
        crate::test_support::ensure_clock();
        init(channels);
    }

    #[rstest]
    fn out_of_range_channel_is_a_no_op() {
        let _g = crate::test_support::guard();
        test_setup(1);
        // Channel 99 was never configured; calls return safely.
        start(99, 100, 1000);
        assert_eq!(run(99), (0, true));
        stop(99);
    }

    #[rstest]
    fn idle_channel_reports_done_immediately() {
        let _g = crate::test_support::guard();
        test_setup(1);
        // start() never called → total_pulses == 0 → run() reports done.
        assert_eq!(run(0), (0, true));
    }

    #[rstest]
    fn start_fires_initial_progress_at_zero() {
        let _g = crate::test_support::guard();
        test_setup(1);
        let progress = Arc::new(AtomicU32::new(u32::MAX));
        {
            let p = Arc::clone(&progress);
            on_progress(0, move |emitted| p.store(emitted, AtomicOrdering::Relaxed));
        }
        start(0, 100, 1000);
        assert_eq!(progress.load(AtomicOrdering::Relaxed), 0);
    }

    #[rstest]
    fn run_emits_progress_and_eventually_completes() {
        let _g = crate::test_support::guard();
        test_setup(1);
        let progress = Arc::new(AtomicU32::new(0));
        {
            let p = Arc::clone(&progress);
            on_progress(0, move |emitted| p.store(emitted, AtomicOrdering::Relaxed));
        }

        // 50 pulses at 5 kHz = 10ms — easily completes within the test.
        start(0, 50, 5_000);
        let mut last = 0u32;
        for _ in 0..200 {
            let (emitted, done) = run(0);
            assert!(emitted >= last, "emitted must be monotonic");
            assert!(emitted <= 50, "emitted must be clamped to total");
            last = emitted;
            if done {
                assert_eq!(emitted, 50, "done implies all pulses emitted");
                assert_eq!(progress.load(AtomicOrdering::Relaxed), 50);
                return;
            }
        }
        panic!("sequence never completed");
    }

    #[rstest]
    fn velocity_mode_integrates_continuously_and_retargets() {
        let _g = crate::test_support::guard();
        test_setup(1);
        let progress = Arc::new(AtomicU32::new(0));
        {
            let p = Arc::clone(&progress);
            on_progress(0, move |emitted| p.store(emitted, AtomicOrdering::Relaxed));
        }

        // Continuous 1 kHz train: emitted advances with virtual time, never "done".
        start_velocity(0, 1_000);
        let mut last = 0u32;
        for _ in 0..50 {
            let (emitted, done) = run(0);
            assert!(!done, "velocity mode never completes on its own");
            assert!(emitted >= last, "emitted is monotonic");
            last = emitted;
        }
        assert!(last > 0, "continuous velocity advanced the emitted count");
        assert_eq!(
            progress.load(AtomicOrdering::Relaxed),
            last,
            "progress matches run()"
        );

        // Retarget to 0 (hold) → the cumulative count freezes (no rewind).
        set_frequency(0, 0);
        let (a, _) = run(0);
        let (b, _) = run(0);
        assert_eq!(a, b, "rate 0 holds the emitted count");
        assert!(a >= last, "held count never goes backwards");

        // Stop leaves velocity mode.
        stop(0);
        assert_eq!(run(0), (0, true), "stop ends the velocity train");
    }

    #[rstest]
    fn stop_cancels_in_flight_sequence() {
        let _g = crate::test_support::guard();
        test_setup(1);
        let stops = Arc::new(AtomicU32::new(0));
        {
            let s = Arc::clone(&stops);
            on_stop(0, move || {
                s.fetch_add(1, AtomicOrdering::Relaxed);
            });
        }
        start(0, 10_000, 1_000);
        stop(0);
        assert_eq!(stops.load(AtomicOrdering::Relaxed), 1);
        // Subsequent run() reports done with no further pulses.
        assert_eq!(run(0), (0, true));
    }

    #[rstest]
    fn restart_resets_baseline() {
        let _g = crate::test_support::guard();
        test_setup(1);
        start(0, 10, 1_000);
        // Drain to completion.
        loop {
            let (_, done) = run(0);
            if done {
                break;
            }
        }
        // A fresh start re-zeroes the timeline; first poll should be small.
        start(0, 10, 1_000);
        let (emitted, _) = run(0);
        assert!(emitted <= 10);
    }

    #[rstest]
    fn frequency_zero_is_clamped() {
        let _g = crate::test_support::guard();
        test_setup(1);
        // Frequency of 0 would divide-by-zero; the driver clamps to 1 Hz.
        start(0, 5, 0);
        let (emitted, _) = run(0);
        assert!(emitted <= 5);
    }

    #[rstest]
    #[should_panic(expected = "exceeds max")]
    fn init_above_max_channels_panics() {
        let _g = crate::test_support::guard();
        crate::test_support::ensure_clock();
        // A count above the backing-array ceiling is a configuration error.
        init(MAX_CHANNELS + 1);
    }

    #[rstest]
    fn on_start_fires_with_pulses_and_frequency() {
        let _g = crate::test_support::guard();
        test_setup(1);
        // on_start receives the exact (pulses, frequency) passed to start(),
        // with frequency clamped to at least 1.
        let seen = Arc::new(std::sync::Mutex::new((0u32, 0u32)));
        {
            let s = Arc::clone(&seen);
            on_start(0, move |pulses, freq| *s.lock().unwrap() = (pulses, freq));
        }
        start(0, 42, 0); // freq 0 clamps to 1
        assert_eq!(*seen.lock().unwrap(), (42, 1));
    }

    #[rstest]
    fn callbacks_are_one_per_channel_and_overwrite() {
        let _g = crate::test_support::guard();
        test_setup(1);
        // Re-registering on_start replaces the previous callback (one per channel).
        let first = Arc::new(AtomicU32::new(0));
        let second = Arc::new(AtomicU32::new(0));
        {
            let f = Arc::clone(&first);
            on_start(0, move |_, _| {
                f.fetch_add(1, AtomicOrdering::Relaxed);
            });
        }
        {
            let s = Arc::clone(&second);
            on_start(0, move |_, _| {
                s.fetch_add(1, AtomicOrdering::Relaxed);
            });
        }
        start(0, 1, 1);
        assert_eq!(
            first.load(AtomicOrdering::Relaxed),
            0,
            "first cb overwritten"
        );
        assert_eq!(
            second.load(AtomicOrdering::Relaxed),
            1,
            "only second cb fires"
        );
    }

    #[rstest]
    fn register_out_of_range_channel_is_ignored() {
        let _g = crate::test_support::guard();
        test_setup(1);
        // Registering a callback past MAX_CHANNELS is silently dropped, not a panic.
        let hits = Arc::new(AtomicU32::new(0));
        {
            let h = Arc::clone(&hits);
            on_progress(MAX_CHANNELS, move |_| {
                h.fetch_add(1, AtomicOrdering::Relaxed);
            });
        }
        // Configured channel 0 still works and fires its own (unset) progress.
        start(0, 1, 1);
        assert_eq!(hits.load(AtomicOrdering::Relaxed), 0);
    }

    #[rstest]
    fn run_clamps_emitted_to_total_after_overrun() {
        let _g = crate::test_support::guard();
        test_setup(1);
        // A tiny, high-frequency sequence finishes well before we poll, so the
        // raw integration would exceed `total`; run() must clamp to exactly total.
        start(0, 1, 1_000_000);
        // Drain to completion; emitted is never allowed above total.
        let mut last = (0u32, false);
        for _ in 0..200 {
            last = run(0);
            assert!(last.0 <= 1, "emitted clamped to total");
            if last.1 {
                break;
            }
        }
        assert_eq!(last, (1, true), "completes with exactly total emitted");
    }

    #[rstest]
    fn reset_clears_channel_count_and_callbacks() {
        let _g = crate::test_support::guard();
        test_setup(1);
        let hits = Arc::new(AtomicU32::new(0));
        {
            let h = Arc::clone(&hits);
            on_progress(0, move |_| {
                h.fetch_add(1, AtomicOrdering::Relaxed);
            });
        }
        reset();
        // After reset, channel 0 is no longer configured: start/run are no-ops
        // and the previously-registered callback can never fire.
        start(0, 5, 1);
        assert_eq!(run(0), (0, true));
        assert_eq!(hits.load(AtomicOrdering::Relaxed), 0);
    }

    /// Finite train of `N` pulses at frequency `F` completes only after about
    /// `N/F` virtual seconds. `run()` sleeps `POLL_TICK_US` per poll, so elapsed
    /// virtual time is ≥ the ideal duration and within a small number of ticks
    /// of overshoot.
    #[rstest]
    #[case::n50_f5k(50, 5_000)]
    #[case::n20_f10k(20, 10_000)]
    #[case::n100_f20k(100, 20_000)]
    fn finite_train_completes_near_n_over_f_virtual_seconds(#[case] n: u32, #[case] freq: u32) {
        let _g = crate::test_support::guard();
        test_setup(1);

        let ideal_us = (n as u64).saturating_mul(1_000_000) / freq as u64;
        start(0, n, freq);
        let t0 = embsim_core::virtual_clock::virtual_us();
        let mut last = (0u32, false);
        for _ in 0..50_000 {
            last = run(0);
            if last.1 {
                break;
            }
        }
        let elapsed = embsim_core::virtual_clock::virtual_us().saturating_sub(t0);
        assert_eq!(last, (n, true), "must complete with exactly N emitted");
        // Lower bound: integration cannot finish early of ideal duration.
        assert!(
            elapsed + 1 >= ideal_us,
            "finished too early: elapsed={elapsed}us ideal={ideal_us}us"
        );
        // Upper bound: this catches a hang or a runaway integration, NOT
        // scheduling jitter. `run()` sleeps `POLL_TICK_US` of *virtual* time
        // per poll, and the virtual clock is scaled wall time
        // (`DETERMINISM.md`, "why timing assertions are tier-dependent"), so
        // on a contended runner every sleep overshoots and the measured
        // elapsed virtual time inflates without anything being wrong — a
        // tight bound here fails on a busy CI box while passing locally.
        // Precision belongs to the strict assertions above (exact emitted
        // count, and cannot-finish-early); this one only has to notice
        // "never finished". Under the stepped clock this can become exact.
        let max_us = ideal_us.saturating_mul(4).saturating_add(1_000_000);
        assert!(
            elapsed <= max_us,
            "finished too late (hang?): elapsed={elapsed}us max={max_us}us"
        );
    }

    /// Frequency / pulse-count matrix: emitted is always clamped to total.
    #[rstest]
    #[case::one_pulse(1, 1_000)]
    #[case::many_fast(200, 100_000)]
    #[case::slow(5, 500)]
    fn run_emitted_never_exceeds_total(#[case] n: u32, #[case] freq: u32) {
        let _g = crate::test_support::guard();
        test_setup(1);
        start(0, n, freq);
        for _ in 0..10_000 {
            let (emitted, done) = run(0);
            assert!(emitted <= n, "emitted {emitted} > total {n}");
            if done {
                assert_eq!(emitted, n);
                return;
            }
        }
        panic!("sequence never completed");
    }

    // ========================================================
    // Rate changes, not edges — the `PulseSegment` seam
    // ========================================================

    /// A segment integrates the *same* floor-division `run()` does, so the two
    /// views of a train can never disagree by a pulse.
    #[rstest]
    #[case::exact(8_192, 1_000_000, 8_192)]
    #[case::half_second(8_192, 500_000, 4_096)]
    #[case::truncates_down(3, 1_000, 0)]
    #[case::one_pulse_worth(1_000, 1_000, 1)]
    fn a_segment_integrates_like_run(
        #[case] freq_hz: u32,
        #[case] elapsed_us: u64,
        #[case] expect: u64,
    ) {
        let segment = PulseSegment {
            emitted: 0,
            freq_hz,
            total: None,
            since_us: 7,
        };
        assert_eq!(segment.emitted_at(7 + elapsed_us), expect);
    }

    /// A finite segment clamps at its ceiling, and `completes_at` names the
    /// instant past which the count no longer moves.
    #[rstest]
    fn a_finite_segment_clamps_and_reports_its_completion() {
        let segment = PulseSegment {
            emitted: 0,
            freq_hz: 1_000,
            total: Some(10),
            since_us: 0,
        };
        let end = segment.completes_at().expect("finite trains complete");
        assert_eq!(end, 10_000, "10 pulses at 1 kHz is 10 ms");
        assert_eq!(segment.emitted_at(end), 10);
        assert_eq!(
            segment.emitted_at(end * 100),
            10,
            "past completion the count is frozen"
        );
        assert_eq!(
            PulseSegment {
                total: None,
                ..segment
            }
            .completes_at(),
            None,
            "an unbounded train never completes"
        );
        assert_eq!(PulseSegment::IDLE.completes_at(), None);
    }

    /// Re-basing hands over the exact count at the re-base instant, and is
    /// idempotent there — so folding baseline differences across a re-base can
    /// neither double-count a pulse nor lose one.
    #[rstest]
    #[case::at_start(0)]
    #[case::mid(333)]
    #[case::later(1_000_000)]
    fn rebasing_a_segment_hands_over_the_exact_count(#[case] at_us: u64) {
        let segment = PulseSegment {
            emitted: 17,
            freq_hz: 8_192,
            total: None,
            since_us: 0,
        };
        let rebased = segment.rebased_at(at_us);
        assert_eq!(
            rebased.emitted, // the handover point
            segment.emitted_at(at_us),
            "the re-based baseline is the count at that instant"
        );
        assert_eq!(
            rebased.rebased_at(at_us),
            rebased,
            "re-basing twice at the same instant is a no-op"
        );
    }

    /// The documented cost of a re-base: it discards the source's pulse
    /// *phase*, so from then on the re-based segment can trail the original by
    /// at most one pulse — never more, and never ahead of it. This is the
    /// fidelity limit that makes "re-base at segment boundaries, not on every
    /// read" a rule rather than a preference.
    #[rstest]
    #[case::odd_rate(8_192, 333)]
    #[case::prime_rate(9_973, 1_237)]
    #[case::slow(37, 500_001)]
    fn rebasing_trails_the_original_by_at_most_one_pulse(#[case] freq_hz: u32, #[case] at_us: u64) {
        let segment = PulseSegment {
            emitted: 0,
            freq_hz,
            total: None,
            since_us: 0,
        };
        let rebased = segment.rebased_at(at_us);
        for probe in [at_us, at_us + 1, at_us + 125_000, at_us + 9_000_001] {
            let (original, after) = (segment.emitted_at(probe), rebased.emitted_at(probe));
            assert!(
                original >= after && original - after <= 1,
                "at {probe}us the re-based segment read {after} against {original}"
            );
        }
    }

    /// One event per *rate change* — not per pulse and not per poll. A finite
    /// train at 20 kHz emits 200 pulses and exactly two rate-change events
    /// (the start and the stop).
    #[rstest]
    fn rate_changes_fire_once_per_command_never_per_pulse() {
        let _g = crate::test_support::guard();
        test_setup(1);
        let events = Arc::new(Mutex::new(Vec::<PulseSegment>::new()));
        {
            let events = Arc::clone(&events);
            on_rate_change(0, move |segment| events.lock().unwrap().push(segment));
        }

        start(0, 200, 20_000);
        while !run(0).1 {}
        stop(0);

        let events = events.lock().unwrap();
        assert_eq!(
            events.len(),
            2,
            "start + stop only; got {events:?} — one event per pulse would be 200"
        );
        assert_eq!(events[0].freq_hz, 20_000);
        assert_eq!(events[0].total, Some(200));
        assert_eq!(events[0].emitted, 0);
        assert_eq!(
            events[1],
            PulseSegment {
                emitted: 200,
                freq_hz: 0,
                total: Some(200),
                since_us: events[1].since_us,
            },
            "the stop event freezes the train's exact final count"
        );
    }

    /// A velocity retarget banks the pulses already emitted into the new
    /// segment's baseline, so folding segment deltas reconstructs the running
    /// total the firmware sees — across any number of rate changes.
    #[rstest]
    fn a_velocity_retarget_banks_the_count_into_the_next_segment() {
        let _g = crate::test_support::guard();
        test_setup(1);
        let events = Arc::new(Mutex::new(Vec::<PulseSegment>::new()));
        {
            let events = Arc::clone(&events);
            on_rate_change(0, move |segment| events.lock().unwrap().push(segment));
        }

        start_velocity(0, 8_192);
        run(0); // one poll tick of virtual time at 8192 Hz
        set_frequency(0, 16_384);
        let banked = events.lock().unwrap()[1].emitted;
        assert_eq!(
            banked,
            emitted(0),
            "the retarget's baseline is the count at that instant"
        );
        run(0);
        stop(0);

        let events = events.lock().unwrap();
        assert_eq!(events.len(), 3, "start + retarget + stop");
        assert_eq!(events[0].freq_hz, 8_192);
        assert_eq!(events[0].total, None, "velocity trains are unbounded");
        assert_eq!(events[1].freq_hz, 16_384);
        assert!(events[1].emitted >= events[0].emitted);
        assert_eq!(events[2].freq_hz, 0);
        assert!(
            events[2].emitted >= events[1].emitted,
            "the count is monotonic across rate changes"
        );
    }

    /// A stopped channel keeps its frozen count for a poller, where `run()`
    /// reports the channel idle. The two answer different questions, and the
    /// count must never appear to rewind.
    #[rstest]
    fn a_stopped_channel_holds_its_final_count() {
        let _g = crate::test_support::guard();
        test_setup(1);
        assert_eq!(emitted(0), 0, "a channel that never ran emitted nothing");
        assert_eq!(segment(0), PulseSegment::IDLE);

        start(0, 40, 20_000);
        while !run(0).1 {}
        let before = emitted(0);
        assert_eq!(before, 40);
        stop(0);

        assert_eq!(emitted(0), 40, "the stop froze the count, it did not clear");
        assert_eq!(segment(0).freq_hz, 0);
        assert_eq!(segment(0).total, Some(40));
        assert_eq!(frequency(0), 0, "an idle channel commands no rate");
        assert_eq!(run(0), (0, true), "run() still reports the channel idle");

        // A fresh train re-bases: each train counts from zero, consumers fold.
        start(0, 5, 20_000);
        assert_eq!(emitted(0), 0);
    }

    /// Reading rate-change state on an unconfigured channel is inert, like
    /// every other out-of-range call in this bank.
    #[rstest]
    fn rate_state_of_an_unconfigured_channel_is_idle() {
        let _g = crate::test_support::guard();
        test_setup(1);
        assert_eq!(segment(99), PulseSegment::IDLE);
        assert_eq!(emitted(99), 0);
        on_rate_change(99, |_| panic!("never fires"));
        start(99, 10, 1_000);
    }
}
