//! Board-engine adapter: the ADS122U04 protocol model as a live
//! [`embsim_board::Component`].
//!
//! [`crate::ads122u04::Ads122u04`] stays a pure UART protocol state machine
//! over a socketpair; this module is the seam that mounts that model on a
//! netlist, so a *system description* (board + harness + scenario) drives it
//! instead of hand wiring:
//!
//! ```text
//!  net engine                        adapter                     protocol model
//!  ──────────                        ───────                     ──────────────
//!  RX pin 16  on_sense ──deframe──► write(firmware_fd) ──► read(model_fd) loop
//!  TX pin 15  bit clock ◄──frame─── engine wake ◄── read(firmware_fd) ◄── write
//!  AIN0/AIN1  on_sense ──► V(AIN0) − V(AIN1) [mV] ──► set_voltage()
//!  ~RESET / DVDD / AVDD  on_sense ──► power/reset gate (adapter-level)
//! ```
//!
//! # The UART is on the net, not beside it
//!
//! Pins 15/16 carry **levels**, through the shared
//! [`embsim_board::SerialLevelBridge`]: a byte leaves as a
//! start bit, eight data bits and a stop bit at [`ADS122U04_BAUD_HZ`], and
//! arrives the same way. They used to carry stream `Producer`/`Consumer` roles,
//! where the net decided *reachability* and the payload never became a level —
//! so a command could not be corrupted by a driver fighting it, and the chip's
//! answer could not break on a wire that was also being driven by something
//! else. On the DS2Addon, where `~RESET` ships on a one-pin net and the fix is
//! a bodge wire, that distinction is the difference between modelling the
//! bench and modelling a tidier board than the one that exists.
//!
//! It also lets the gate be **electrical**: an unpowered or held-in-reset chip
//! releases TX to high-Z ([`SerialLevelBridge::set_output_enabled`]) instead of
//! politely discarding bytes behind a line it is still driving.
//!
//! # Datasheet provenance
//!
//! Modeled against **TI SBAS752B (May 2017, revised Oct 2018)**, like the
//! protocol model. The pin facade is the TSSOP-16 (PW) pin table (SBAS752B
//! p.3). The gate implements the chip's power/reset envelope:
//!
//! - Power-on reset requires **both** supplies — an unpowered AVDD or DVDD is
//!   a silent chip (SBAS752B power-on reset; supplies specified 2.3 V–5.5 V,
//!   §6.3 Recommended Operating Conditions). This is the July 2026 bench
//!   "AVDD unstrapped" failure made live.
//! - **`~RESET` is active low**; held low the interface never answers, and a
//!   *floating* digital input is out of spec (SBAS752B, unused-inputs
//!   guidance) — the DS2Addon PCB ships `~RESET` on a one-pin net, so a
//!   system description without the reset bodge (`pin_short` to a rail, or a
//!   board rev with the R10 pull-up) gets exactly the bench symptom: perfect
//!   commands in, silence out. The engine reports the floating sense; the
//!   adapter chooses the datasheet behavior (silence).
//! - The high projection of `~RESET` uses **V_IH = 0.7 · DVDD** (§6.5
//!   Electrical Characteristics, Digital Inputs/Outputs),
//!   against the solved DVDD rail voltage when the engine publishes one.
//!
//! ## Deliberate simplifications
//!
//! - The gate pauses I/O at the adapter boundary (RX bytes ignored, TX bytes
//!   discarded); it does not model the POR release delay (~600 µs after both
//!   supplies) nor reset the model's registers on a reset edge — the protocol
//!   thread is untouched, per the adapter contract.
//! - Floating/unsolvable analog inputs hold the last fed differential; the
//!   datasheet floating-input noise policy is a later slice.
//! - Baud-rate auto-detection is unmodeled in the protocol model, so the UART
//!   pins frame at the fixed rate the consuming firmware uses
//!   ([`ADS122U04_BAUD_HZ`]). A peer at another rate now produces *framing
//!   errors* rather than silently working, which is what real hardware does.
//!
//! # Slice note (deferred inversion)
//!
//! This slice makes the **chip** side of the force path a live board-engine
//! component. The MCU-side inversion — the engine spawning the firmware
//! entry (`BOARD_ENGINE.md`, "The MCU as a component", item 1) — is
//! **deferred**: consumers keep booting firmware via
//! `embsim_runtime::Emulator::run` on the main thread and bridge the MCU's
//! serial channels to stream pins in their wiring layer.

use std::os::fd::{AsFd, BorrowedFd, FromRawFd, OwnedFd};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

use embsim_board::uart::{FramingError, UartFraming};
use embsim_board::{
    AttachError, Component, ComponentNetIo, DeadBand, Level, PinDecl, Sense, SerialLevelBridge,
    Thresholds, Volts,
};
use tracing::{debug, trace, warn};

use crate::ads122u04::{Ads122u04, Config};

// ============================================================
// Pin facade (single source of truth)
// ============================================================

/// UART rate of the TX/RX pins. The protocol model has no baud-rate
/// auto-detection (a byte pipe has no sync-word timing to measure), so the
/// facade pins the rate the consuming firmware runs the interface at.
pub const ADS122U04_BAUD_HZ: u32 = 115_200;

/// Digital input low level, as a fraction of `DVDD`: `V_IL` max 0.3 · DVDD
/// (SBAS752B §6.5 Electrical Characteristics, Digital Inputs/Outputs, p. 6).
const VIL_DVDD_RATIO: f64 = 0.3;

/// Digital input high level, as a fraction of `DVDD`: `V_IH` min 0.7 · DVDD
/// (SBAS752B §6.5 Electrical Characteristics, Digital Inputs/Outputs, p. 6).
const VIH_DVDD_RATIO: f64 = 0.7;

/// The digital inputs' thresholds, **relative** to `DVDD` and measured
/// against `DGND`: `V_IL` 0.3 · DVDD, `V_IH` 0.7 · DVDD; the datasheet names
/// no input hysteresis (SBAS752B §6.5, p. 6), so between the two it
/// guarantees neither level ([`DeadBand::Unknown`]).
pub const ADS122U04_INPUT_THRESHOLDS: Thresholds =
    Thresholds::new(VIL_DVDD_RATIO, VIH_DVDD_RATIO, 0.0, DeadBand::Unknown);

/// A digital input: the datasheet's thresholds against its `DVDD`/`DGND`.
/// No pin declares a byte route, TX and RX included: the UART is framed
/// onto the net as levels.
const fn digital_in(number: &'static str, name: &'static str) -> PinDecl {
    PinDecl::digital_in(number, ADS122U04_INPUT_THRESHOLDS)
        .with_name(name)
        .with_supply("DVDD")
        .with_reference("DGND")
}

/// An analog input or reference input, read against `AVSS` — the
/// converter's analog ground.
const fn analog_in(number: &'static str, name: &'static str) -> PinDecl {
    PinDecl::analog(number)
        .with_name(name)
        .with_reference("AVSS")
}

/// TSSOP-16 (PW) pinout per SBAS752B p.3: 1 GPIO1, 2 GPIO0, 3 ~RESET,
/// 4 DGND, 5 AVSS, 6 AIN3, 7 AIN2, 8 REFN, 9 REFP, 10 AIN1, 11 AIN0,
/// 12 AVDD, 13 DVDD, 14 GPIO2/DRDY, 15 TX, 16 RX.
///
/// This table is the pin truth shared by [`Ads122u04Component`] and the
/// build-time facades in the `embsim-board` regression tests — one table, so
/// the analysis pass and the live component can never disagree on the pinout.
pub const ADS122U04_PINS: [PinDecl; 16] = [
    digital_in("1", "GPIO1"),
    digital_in("2", "GPIO0"),
    digital_in("3", "~RESET"),
    PinDecl::power_in("4").with_name("DGND"),
    PinDecl::power_in("5").with_name("AVSS"),
    analog_in("6", "AIN3"),
    analog_in("7", "AIN2"),
    analog_in("8", "REFN"),
    analog_in("9", "REFP"),
    analog_in("10", "AIN1"),
    analog_in("11", "AIN0"),
    PinDecl::power_in("12")
        .with_name("AVDD")
        .with_reference("AVSS"),
    PinDecl::power_in("13")
        .with_name("DVDD")
        .with_reference("DGND"),
    digital_in("14", "DRDY"),
    PinDecl::digital_out("15").with_name("TX"),
    digital_in("16", "RX"),
];

/// The framing the chip's UART uses: 8N1 at [`ADS122U04_BAUD_HZ`].
pub fn ads122u04_framing() -> UartFraming {
    UartFraming::new_8n1(ADS122U04_BAUD_HZ)
}

// ============================================================
// Power/reset gate
// ============================================================

/// Minimum operating supply voltage: AVDD and DVDD are specified
/// 2.3 V–5.5 V (SBAS752B §6.3 Recommended Operating Conditions). A rail
/// solved below this — including a rail stuck at 0 V — is a down domain.
const SUPPLY_MIN_VOLTS: f64 = 2.3;

/// The three gating inputs, as their pins were last handed them: each a
/// voltage against the pin's own ground (`DGND` for `~RESET` and `DVDD`,
/// `AVSS` for `AVDD`), `None` where no source reaches the net or no voltage
/// can be named for it.
#[derive(Debug, Clone, Copy, Default)]
struct GateInputs {
    reset: Option<Volts>,
    dvdd: Option<Volts>,
    avdd: Option<Volts>,
    /// The level `~RESET` last read — its receiver's last level, which the
    /// next projection is chosen by.
    reset_level: Option<Level>,
}

impl GateInputs {
    /// `~RESET` through the datasheet's thresholds, 0.3/0.7 × `DVDD`
    /// (SBAS752B §6.5), scaled by the `DVDD` it has now. No `DVDD`, no
    /// level: the thresholds have no supply to scale by.
    fn project_reset(&mut self) {
        self.reset_level = match (self.reset, self.dvdd) {
            (Some(reset), Some(dvdd)) => ADS122U04_INPUT_THRESHOLDS
                .scaled(dvdd)
                .project(reset, self.reset_level),
            _ => None,
        };
    }
}

/// Adapter-level power/reset gate. Sense callbacks (engine thread) write the
/// inputs; the RX handler (engine thread) and the TX pump thread read
/// `alive()` at each delivery, so the protocol thread itself never needs to
/// know about power domains.
#[derive(Debug)]
struct Gate {
    inputs: Mutex<GateInputs>,
}

impl Gate {
    /// Nothing handed yet, chip dead until the engine says otherwise — the
    /// engine delivers the current sense of every sensed net once at
    /// registration, so a live system settles the gate before any stream
    /// traffic lands.
    fn new() -> Self {
        Self {
            inputs: Mutex::new(GateInputs::default()),
        }
    }

    fn set_reset(&self, sense: &Sense) {
        let mut inputs = self.inputs.lock().unwrap();
        inputs.reset = sense.volts;
        inputs.project_reset();
    }

    fn set_dvdd(&self, sense: &Sense) {
        let mut inputs = self.inputs.lock().unwrap();
        inputs.dvdd = sense.volts;
        inputs.project_reset();
    }

    fn set_avdd(&self, sense: &Sense) {
        self.inputs.lock().unwrap().avdd = sense.volts;
    }

    /// True when the chip is electrically alive: both supplies up (power-on
    /// reset requires BOTH AVDD and DVDD — SBAS752B) and `~RESET` high.
    /// Anything else — a floating one-pin reset net, an unstrapped analog
    /// domain, a rail fight — is the bench-observed silent chip.
    fn alive(&self) -> bool {
        let inputs = *self.inputs.lock().unwrap();
        supply_ok(inputs.dvdd) && supply_ok(inputs.avdd) && inputs.reset_level == Some(Level::High)
    }
}

/// Apply the gate to the chip's output driver: a dead part does not drive its
/// TX pin, it releases it.
fn regate_output(gate: &Gate, uart: &SerialLevelBridge) {
    uart.set_output_enabled(gate.alive());
}

/// A supply rail counts as up when it is at an operating voltage against
/// its ground, [`SUPPLY_MIN_VOLTS`] or more. A rail that names no voltage —
/// floating, a clock, fought for half of every cycle, only an unmodelled
/// rail behind it, a ground that is not held — is down: the engine never
/// invents a value, and neither does the chip model (`DESIGN.md` rule 6).
fn supply_ok(volts: Option<Volts>) -> bool {
    volts.is_some_and(|v| v >= SUPPLY_MIN_VOLTS)
}

// ============================================================
// Component
// ============================================================

/// The ADS122U04 as a live board-engine component: wraps one
/// [`Ads122u04`] protocol model instance and bridges it to the nets the
/// netlist actually connects to its pins.
///
/// Created via [`Ads122u04Component::new`], registered with a
/// `PartRegistry` under the `ADS122U04` part name. Dropping the component
/// closes the firmware-side pipe end.
///
/// # Timing is engine-owned
///
/// The model's output is drained on an **engine wakeup**
/// (`io.schedule_every(`[`PUMP_POLL_VIRTUAL_US`]`)`), not by a pump thread of
/// its own — this is `DETERMINISM.md` T1 §4's "move it onto the engine wheel"
/// for the reference component. Consequences worth knowing:
///
/// - The drain runs **on the engine thread**, so it is ordered against every
///   other engine event and its cadence is exact in stepped clock mode (wakes
///   land at 250 µs, 500 µs, …, not at sampled wall instants). One thread and
///   one poll loop are deleted rather than made deterministic.
/// - The drain is a **non-blocking** read: it never parks the engine thread.
/// - **What is still not deterministic** (stated here rather than papered
///   over): the byte path itself is a real `socketpair`, and *whether the
///   model's protocol thread has written a byte yet* when the engine drains is
///   an OS scheduling decision. Moving the cadence onto the wheel fixes
///   *when* the engine looks, not *what the kernel has for it*. Making that
///   deterministic is the in-process transport of `DETERMINISM.md` Phase D2.
pub struct Ads122u04Component {
    /// The protocol model (owns the socketpair's model end and the
    /// `protocol_loop` thread).
    model: Arc<Ads122u04>,
    /// Firmware-side pipe end, owned here: RX bytes are written into it, and
    /// the engine wakeup reads the model's output from it.
    firmware_fd: Arc<OwnedFd>,
    gate: Arc<Gate>,
    /// Last numerically solved AIN0/AIN1 node voltages (V), for the
    /// differential feed.
    ain_volts: Arc<Mutex<[f64; 2]>>,
    /// Set on drop, so a callback that outlives the component stops driving.
    shutdown: Arc<AtomicBool>,
}

impl Ads122u04Component {
    /// Create the component around a fresh protocol model instance.
    pub fn new(config: Config) -> Self {
        let (model, firmware_fd) = Ads122u04::new(config);
        // SAFETY: `Ads122u04::new` creates the socketpair and returns the
        // firmware-side descriptor to exactly one caller; wrapping it here
        // transfers that ownership, so it closes when the component drops.
        let firmware_fd = unsafe { OwnedFd::from_raw_fd(firmware_fd) };
        Self {
            model,
            firmware_fd: Arc::new(firmware_fd),
            gate: Arc::new(Gate::new()),
            ain_volts: Arc::new(Mutex::new([0.0; 2])),
            shutdown: Arc::new(AtomicBool::new(false)),
        }
    }
}

impl Component for Ads122u04Component {
    fn pins(&self) -> &[PinDecl] {
        &ADS122U04_PINS
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        // -- the chip's UART, framed onto the net ----------------------
        // Built first: the gate's sense callbacks fire at registration, and a
        // gate transition has to be able to release the output driver.
        let uart = Arc::new(SerialLevelBridge::new(
            ads122u04_framing(),
            io.pin("TX")?,
            io.clone(),
            Arc::clone(&self.shutdown),
        ));
        // Dead until the engine says otherwise: the chip is not driving TX
        // before its supplies resolve, and a peer must see high-Z rather than
        // an idle line it can mistake for a powered part.
        uart.set_output_enabled(false);
        uart.idle();

        // -- power/reset gate ------------------------------------------
        for (pin, set) in [
            ("~RESET", Gate::set_reset as fn(&Gate, &Sense)),
            ("DVDD", Gate::set_dvdd),
            ("AVDD", Gate::set_avdd),
        ] {
            let gate = Arc::clone(&self.gate);
            let uart = Arc::clone(&uart);
            let is_dvdd = pin == "DVDD";
            io.on_sense(pin, move |sense| {
                set(&gate, &sense);
                // The UART's output high is the digital supply: `V_OH` is
                // 0.8 × `DVDD` min at 1 mA (SBAS752B §6.5), so a part on a
                // 5 V `DVDD` drives 5 V, not the crate's 3.3 V logic rail.
                if let (true, Some(dvdd)) = (is_dvdd, sense.volts) {
                    uart.set_high_volts(dvdd);
                }
                regate_output(&gate, &uart);
            })?;
        }

        // -- differential analog input ---------------------------------
        // The engine delivers solved node voltages in volts; the model's
        // `set_voltage` input is the differential in millivolts. Sign
        // convention matches the hand-wired force path (and the firmware's
        // MUX config, AINP = AIN0 / AINN = AIN1): a strain-gauge output of
        // +x mV presents V(AIN0) − V(AIN1) = +x mV, fed in directly.
        for (index, pin) in [(0usize, "AIN0"), (1usize, "AIN1")] {
            let ain_volts = Arc::clone(&self.ain_volts);
            let model = Arc::clone(&self.model);
            io.on_sense(pin, move |sense| {
                let Some(volts) = sense.volts else {
                    trace!(
                        pin,
                        ?sense,
                        "ADS122U04: input names no voltage; holding last differential"
                    );
                    return;
                };
                let diff_mv = {
                    let mut ain = ain_volts.lock().unwrap();
                    ain[index] = volts;
                    (ain[0] - ain[1]) * 1_000.0
                };
                model.set_voltage(diff_mv);
            })?;
        }

        // -- RX: edges on pin 16 → bytes → firmware-side pipe end -------
        // Gated so an unpowered / held-in-reset chip never sees the command
        // stream. The deframer keeps running either way: a command that
        // arrives while the part is dead is *lost*, not queued, which is the
        // bench symptom (perfect commands in, silence out).
        {
            let gate = Arc::clone(&self.gate);
            let fd = Arc::clone(&self.firmware_fd);
            let uart = Arc::clone(&uart);
            let rx = io.pin("RX")?;
            io.on_sense("RX", move |sense| {
                deliver_rx(&fd, &gate, uart.receive_sense(&rx, &sense));
            })?;
        }

        // -- TX: model output → bytes → edges on pin 15 -----------------
        // Drained on an engine wakeup. The whole callback runs on the engine
        // thread; `drain_model_output` never blocks (the socketpair is
        // non-blocking), so it cannot stall the engine, and in stepped mode
        // its cadence is exact.
        {
            let fd = Arc::clone(&self.firmware_fd);
            let gate = Arc::clone(&self.gate);
            let uart = Arc::clone(&uart);
            io.on_wake_ns(move |now_ns| {
                let out = drain_model_output(&fd, &gate);
                if !out.is_empty() {
                    uart.transmit(&out);
                }
                // One handler for both directions: it clocks the next TX bit
                // and closes any RX frame whose tail carried no transition.
                // The bridge arms its own next wake.
                deliver_rx(&fd, &gate, uart.service(now_ns));
            });
        }
        // A periodic wheel entry rather than a thread: idle components cost
        // nothing, and the cadence belongs to the engine (`BOARD_ENGINE.md`,
        // "no broadcast tick(), engine-owned wakeups"). On the inert
        // build-analysis path this is traced and dropped, which is strictly
        // better than the pump thread it replaces — that one was spawned by
        // `System::build` too, and outlived it.
        //
        // The bit clock rides the *same* handler on its own one-shot wakes, so
        // this cadence only has to be fine enough to notice new model output;
        // it does not have to resolve a bit.
        io.schedule_every_ns(PUMP_POLL_VIRTUAL_US * 1_000);
        Ok(())
    }
}

impl Drop for Ads122u04Component {
    fn drop(&mut self) {
        // Stop the UART bridge before the FDs go: a wake callback the engine
        // has not yet dropped must be inert, not driving a dead pin.
        self.shutdown.store(true, Ordering::Relaxed);
        // Dropping `firmware_fd` closes the pipe end once the engine has
        // dropped its wake callback (SystemHandle joins the engine *before* it
        // drops components — the documented drop order). The model's protocol
        // thread then reads EOF and idles, exactly as in the hand-wired setup.
        debug!("ADS122U04 component shut down");
    }
}

// ============================================================
// Stream drain (model output → TX pin), on an engine wakeup
// ============================================================

/// Wakeup interval (virtual µs) for draining the model's output, following the
/// protocol model's own `protocol_loop` pacing rationale: substantially
/// finer than the fastest configured conversion interval (1 ms at 1000 SPS).
pub const PUMP_POLL_VIRTUAL_US: u64 = 250;

/// Maximum bytes drained from the model per wakeup. A bound rather than
/// "drain to EAGAIN": this runs on the engine thread, and a model that
/// out-produces the wire must not be able to hold the engine in a read loop.
/// At 115.2 kbaud, 4 kB is far more than one 250 µs slot can carry, so the
/// bound is unreachable in normal operation — it is a backstop, not a policy.
const DRAIN_MAX_BYTES_PER_WAKE: usize = 4096;

/// Drain whatever the model has emitted (readable on the firmware-side pipe
/// end), gated exactly like RX — a dead chip's output never reaches the wire,
/// and bytes produced while gated are discarded, not spooled for a later
/// power-up.
///
/// Runs on the **engine thread**, and never blocks: the socketpair is
/// non-blocking, so this cannot stall the engine.
fn drain_model_output(fd: &OwnedFd, gate: &Gate) -> Vec<u8> {
    let mut out = Vec::new();
    let mut buf = [0u8; 64];
    let mut drained = 0usize;
    while drained < DRAIN_MAX_BYTES_PER_WAKE {
        match nix::unistd::read(fd.as_fd(), &mut buf) {
            Ok(0) => break, // peer end closed
            Ok(n) => {
                drained += n;
                if gate.alive() {
                    out.extend_from_slice(&buf[..n]);
                } else {
                    trace!(
                        discarded = n,
                        "ADS122U04: TX bytes discarded (unpowered or in reset)"
                    );
                }
            }
            Err(nix::errno::Errno::EAGAIN) => break, // nothing pending
            Err(e) => {
                warn!("ADS122U04: model read error: {e}");
                break;
            }
        }
    }
    out
}

/// Hand deframed commands to the model, gated.
///
/// A frame that did not decode is dropped with a trace rather than passed on:
/// the chip's shift register hands its command decoder bytes, not framing
/// errors. A run of them means the wire was contended or the peer's baud rate
/// disagrees — which is exactly what this path exists to be able to say.
fn deliver_rx(
    fd: &OwnedFd,
    gate: &Gate,
    frames: impl IntoIterator<Item = Result<u8, FramingError>>,
) {
    for frame in frames {
        match frame {
            Ok(byte) if gate.alive() => write_all(fd.as_fd(), &[byte]),
            Ok(byte) => trace!(byte, "ADS122U04: RX byte ignored (unpowered or in reset)"),
            Err(error) => trace!(
                ?error,
                "ADS122U04: RX frame dropped (bad framing on the wire)"
            ),
        }
    }
}

/// Write all bytes to the pipe end, yielding on EAGAIN (the socketpair
/// buffer is far deeper than any protocol exchange, so this never spins
/// meaningfully).
fn write_all(fd: BorrowedFd<'_>, data: &[u8]) {
    let mut written = 0;
    while written < data.len() {
        match nix::unistd::write(fd, &data[written..]) {
            Ok(n) => written += n,
            Err(nix::errno::Errno::EAGAIN) => std::thread::yield_now(),
            Err(e) => {
                warn!("ADS122U04: pipe write error: {e}");
                break;
            }
        }
    }
}

// ============================================================
// Tests
// ============================================================
//
// The gate's pure predicate logic is tested here; the live end-to-end
// behavior (RDATA over the real DS2Addon netlist, the silent-chip
// regression) lives in `embsim-board`'s integration tests next to the
// netlist fixture (`board/tests/ds2_live_force_path.rs`).

#[cfg(test)]
mod tests {
    use rstest::rstest;

    use super::*;

    const RAIL_3V3: Option<Volts> = Some(3.3);

    /// A sense handed `volts` (the instant plays no part in the gate).
    fn handed(volts: Option<Volts>) -> Sense {
        Sense {
            volts,
            periodic: None,
            at_ns: 0,
        }
    }

    fn gate_with(reset: Option<Volts>, dvdd: Option<Volts>, avdd: Option<Volts>) -> Gate {
        let gate = Gate::new();
        gate.set_dvdd(&handed(dvdd));
        gate.set_avdd(&handed(avdd));
        gate.set_reset(&handed(reset));
        gate
    }

    /// Fresh gate: nothing handed, chip dead — the engine has not yet
    /// delivered any sense, and the adapter must not assume power.
    #[rstest]
    fn gate_starts_dead() {
        assert!(!Gate::new().alive());
    }

    /// The full bench-good configuration: both rails at 3.3 V, reset tied
    /// high (the bodge, or the R10 pull-up rev's 3.3 V) — alive.
    #[rstest]
    fn gate_alive_with_both_rails_and_reset_high() {
        assert!(gate_with(RAIL_3V3, RAIL_3V3, RAIL_3V3).alive());
    }

    /// The DS2Addon bench bug: a floating `~RESET` is a silent chip even
    /// with both supplies up.
    #[rstest]
    fn gate_dead_with_floating_reset() {
        assert!(!gate_with(None, RAIL_3V3, RAIL_3V3).alive());
    }

    /// Reset held low, or fought to a voltage inside its dead band (two
    /// 25 Ω drivers at 1.65 V), is a held-in-reset chip.
    #[rstest]
    fn gate_dead_with_reset_low_or_contended() {
        assert!(!gate_with(Some(0.0), RAIL_3V3, RAIL_3V3).alive());
        assert!(!gate_with(Some(1.65), RAIL_3V3, RAIL_3V3).alive());
    }

    /// POR requires BOTH supplies (SBAS752B): an unstrapped (floating) AVDD
    /// or DVDD — or a rail sourced at 0 V — is a dead chip.
    #[rstest]
    fn gate_dead_with_either_supply_down() {
        assert!(!gate_with(RAIL_3V3, None, RAIL_3V3).alive());
        assert!(!gate_with(RAIL_3V3, RAIL_3V3, None).alive());
        assert!(!gate_with(RAIL_3V3, RAIL_3V3, Some(0.0)).alive());
        assert!(!gate_with(RAIL_3V3, Some(1.0), RAIL_3V3).alive());
    }

    /// V_IH scales with the DVDD rail (0.7 · DVDD, SBAS752B §6.5): 2.4 V
    /// clears the threshold at DVDD = 3.3 V (V_IH = 2.31 V) but not at
    /// DVDD = 5.0 V (V_IH = 3.5 V).
    #[rstest]
    fn reset_threshold_tracks_dvdd() {
        assert!(gate_with(Some(2.4), RAIL_3V3, RAIL_3V3).alive());
        assert!(!gate_with(Some(2.4), Some(5.0), Some(5.0)).alive());
        // Just below the 3.3 V threshold, inside the dead band: dead.
        assert!(!gate_with(Some(2.3), RAIL_3V3, RAIL_3V3).alive());
    }

    /// A DVDD that moves re-projects the reset the gate holds: the same
    /// 2.4 V is high at 3.3 V and not once DVDD rises to 5 V.
    #[rstest]
    fn a_dvdd_move_reprojects_the_held_reset() {
        let gate = gate_with(Some(2.4), RAIL_3V3, RAIL_3V3);
        assert!(gate.alive());
        gate.set_dvdd(&handed(Some(5.0)));
        assert!(!gate.alive());
    }

    /// The shared pin table stays the SBAS752B p.3 truth: 16 pins, and the
    /// UART is a pair of plain digital pins. No pin declares a byte route —
    /// TX and RX carry levels, so there is nothing for the engine to route.
    #[rstest]
    fn the_uart_pins_are_plain_digital_pins() {
        assert_eq!(ADS122U04_PINS.len(), 16);
        let tx = ADS122U04_PINS
            .iter()
            .find(|p| p.number == "15")
            .expect("TX declared");
        assert_eq!(*tx, PinDecl::digital_out("15").with_name("TX"));
        let rx = ADS122U04_PINS
            .iter()
            .find(|p| p.number == "16")
            .expect("RX declared");
        assert_eq!(
            rx.senses_at_build(),
            Some(embsim_board::SenseKind::Digital),
            "RX reads through the datasheet's thresholds"
        );
        assert_eq!(rx.thresholds, Some(ADS122U04_INPUT_THRESHOLDS));
    }

    /// The framing is 8N1 at the rate the firmware runs the interface at.
    #[rstest]
    fn the_framing_matches_the_declared_baud() {
        let framing = ads122u04_framing();
        assert_eq!(framing.data_bits, 8);
        assert_eq!(framing.stop_bits, 1);
        assert!(framing.lsb_first);
        assert_eq!(
            framing.bit_period_ns,
            1_000_000_000 / u64::from(ADS122U04_BAUD_HZ)
        );
    }
}
