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
//!  RX pin 16  on_byte ──gate──► write(firmware_fd) ──► read(model_fd) protocol_loop
//!  TX pin 15  stream_tx ◄─gate── pump thread ◄── read(firmware_fd) ◄── write(model_fd)
//!  AIN0/AIN1  on_sense ──► V(AIN0) − V(AIN1) [mV] ──► set_voltage()
//!  ~RESET / DVDD / AVDD  on_sense ──► power/reset gate (adapter-level)
//! ```
//!
//! # Datasheet provenance
//!
//! Modeled against **TI SBAS752B (May 2017, revised Oct 2018)**, like the
//! protocol model. The pin facade is the TSSOP-16 (PW) pin table (SBAS752B
//! p.3). The gate implements the chip's power/reset envelope:
//!
//! - Power-on reset requires **both** supplies — an unpowered AVDD or DVDD is
//!   a silent chip (SBAS752B power-on reset; supplies specified 2.3 V–5.5 V,
//!   §7.3 Recommended Operating Conditions). This is the July 2026 bench
//!   "AVDD unstrapped" failure made live.
//! - **`~RESET` is active low**; held low the interface never answers, and a
//!   *floating* digital input is out of spec (SBAS752B, unused-inputs
//!   guidance) — the DS2Addon PCB ships `~RESET` on a one-pin net, so a
//!   system description without the reset bodge (`pin_short` to a rail, or a
//!   board rev with the R10 pull-up) gets exactly the bench symptom: perfect
//!   commands in, silence out. The engine reports the floating sense; the
//!   adapter chooses the datasheet behavior (silence).
//! - The high projection of `~RESET` uses **V_IH = 0.7 · DVDD** (§7.3),
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
//! - Baud-rate auto-detection is unmodeled in the protocol model, so the
//!   stream pins declare the fixed rate the consuming firmware uses
//!   ([`ADS122U04_BAUD_HZ`]).
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
use std::thread::JoinHandle;
use std::time::Duration;

use embsim_board::component::StreamTx;
use embsim_board::{
    AttachError, Component, ComponentNetIo, Level, NetState, PinDecl, PinKind, StreamRole,
};
use embsim_core::virtual_clock;
use tracing::{debug, trace, warn};

use crate::ads122u04::{Ads122u04, Config};

// ============================================================
// Pin facade (single source of truth)
// ============================================================

/// UART rate of the stream pins. The protocol model has no baud-rate
/// auto-detection (a byte pipe has no sync-word timing to measure), so the
/// facade pins the rate the consuming firmware runs the interface at.
pub const ADS122U04_BAUD_HZ: u32 = 115_200;

const NONE: Option<StreamRole> = None;

const fn pin(
    number: &'static str,
    name: Option<&'static str>,
    kind: PinKind,
    stream: Option<StreamRole>,
) -> PinDecl {
    PinDecl {
        number,
        name,
        kind,
        stream,
        drive_impedance: None,
    }
}

/// TSSOP-16 (PW) pinout per SBAS752B p.3: 1 GPIO1, 2 GPIO0, 3 ~RESET,
/// 4 DGND, 5 AVSS, 6 AIN3, 7 AIN2, 8 REFN, 9 REFP, 10 AIN1, 11 AIN0,
/// 12 AVDD, 13 DVDD, 14 GPIO2/DRDY, 15 TX, 16 RX.
///
/// This table is the pin truth shared by [`Ads122u04Component`] and the
/// build-time facades in the `embsim-board` regression tests — one table, so
/// the analysis pass and the live component can never disagree on the pinout.
pub const ADS122U04_PINS: [PinDecl; 16] = [
    pin("1", Some("GPIO1"), PinKind::DigitalIn, NONE),
    pin("2", Some("GPIO0"), PinKind::DigitalIn, NONE),
    pin("3", Some("~RESET"), PinKind::DigitalIn, NONE),
    pin("4", Some("DGND"), PinKind::PowerIn, NONE),
    pin("5", Some("AVSS"), PinKind::PowerIn, NONE),
    pin("6", Some("AIN3"), PinKind::Analog, NONE),
    pin("7", Some("AIN2"), PinKind::Analog, NONE),
    pin("8", Some("REFN"), PinKind::Analog, NONE),
    pin("9", Some("REFP"), PinKind::Analog, NONE),
    pin("10", Some("AIN1"), PinKind::Analog, NONE),
    pin("11", Some("AIN0"), PinKind::Analog, NONE),
    pin("12", Some("AVDD"), PinKind::PowerIn, NONE),
    pin("13", Some("DVDD"), PinKind::PowerIn, NONE),
    pin("14", Some("DRDY"), PinKind::DigitalIn, NONE),
    pin(
        "15",
        Some("TX"),
        PinKind::DigitalOut,
        Some(StreamRole::Producer {
            baud_hz: ADS122U04_BAUD_HZ,
        }),
    ),
    pin(
        "16",
        Some("RX"),
        PinKind::DigitalIn,
        Some(StreamRole::Consumer {
            baud_hz: ADS122U04_BAUD_HZ,
        }),
    ),
];

// ============================================================
// Power/reset gate
// ============================================================

/// Minimum operating supply voltage: AVDD and DVDD are specified
/// 2.3 V–5.5 V (SBAS752B §7.3 Recommended Operating Conditions). A rail
/// solved below this — including a rail stuck at 0 V — is a down domain.
const SUPPLY_MIN_VOLTS: f64 = 2.3;

/// Digital input high threshold: V_IH = 0.7 · DVDD (SBAS752B §7.3).
const VIH_DVDD_RATIO: f64 = 0.7;

/// Nominal DVDD used for the V_IH projection while the DVDD net has no
/// numeric solve (`Driven`/`Pulled` states carry a level, not volts).
const NOMINAL_DVDD_VOLTS: f64 = 3.3;

/// Last engine-published states of the three gating nets.
#[derive(Debug, Clone, Copy)]
struct GateInputs {
    reset: NetState,
    dvdd: NetState,
    avdd: NetState,
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
    /// Everything floating until the engine says otherwise — the engine
    /// delivers the current state of every sensed net once at registration,
    /// so a live system settles the gate before any stream traffic lands.
    fn new() -> Self {
        Self {
            inputs: Mutex::new(GateInputs {
                reset: NetState::Floating,
                dvdd: NetState::Floating,
                avdd: NetState::Floating,
            }),
        }
    }

    fn set_reset(&self, state: NetState) {
        self.inputs.lock().unwrap().reset = state;
    }

    fn set_dvdd(&self, state: NetState) {
        self.inputs.lock().unwrap().dvdd = state;
    }

    fn set_avdd(&self, state: NetState) {
        self.inputs.lock().unwrap().avdd = state;
    }

    /// True when the chip is electrically alive: both supplies up (power-on
    /// reset requires BOTH AVDD and DVDD — SBAS752B) and `~RESET` high.
    /// Anything else — a floating one-pin reset net, an unstrapped analog
    /// domain, a rail fight — is the bench-observed silent chip.
    fn alive(&self) -> bool {
        let inputs = *self.inputs.lock().unwrap();
        supply_ok(inputs.dvdd) && supply_ok(inputs.avdd) && reset_high(inputs.reset, inputs.dvdd)
    }
}

/// A supply rail counts as up when it is sourced at an operating voltage.
fn supply_ok(state: NetState) -> bool {
    match state {
        NetState::Analog(v) => v >= SUPPLY_MIN_VOLTS,
        // A rail known only as a digital projection (e.g. an unmodeled-
        // voltage `PowerOut` presenting `Pulled(High)`) counts as up; a rail
        // held low, floating, or fought over does not.
        NetState::Driven(Level::High) | NetState::Pulled(Level::High, _) => true,
        NetState::Driven(Level::Low) | NetState::Pulled(Level::Low, _) => false,
        NetState::Floating | NetState::Contention => false,
    }
}

/// Digital-high projection of the `~RESET` net: V_IH = 0.7 · DVDD against
/// the solved rail voltage (nominal 3.3 V while DVDD has no numeric solve).
/// `Floating` is deliberately NOT high — the engine never invents a value
/// for a floating sense, and neither does the chip model.
fn reset_high(reset: NetState, dvdd: NetState) -> bool {
    let dvdd_volts = match dvdd {
        NetState::Analog(v) => v,
        _ => NOMINAL_DVDD_VOLTS,
    };
    match reset {
        NetState::Driven(Level::High) | NetState::Pulled(Level::High, _) => true,
        NetState::Analog(v) => v >= VIH_DVDD_RATIO * dvdd_volts,
        _ => false,
    }
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
/// stops the stream pump thread and closes the firmware-side pipe end.
pub struct Ads122u04Component {
    /// The protocol model (owns the socketpair's model end and the
    /// `protocol_loop` thread).
    model: Arc<Ads122u04>,
    /// Firmware-side pipe end, owned here: RX bytes are written into it,
    /// and the pump thread reads the model's output from it.
    firmware_fd: Arc<OwnedFd>,
    gate: Arc<Gate>,
    /// Last numerically solved AIN0/AIN1 node voltages (V), for the
    /// differential feed.
    ain_volts: Arc<Mutex<[f64; 2]>>,
    shutdown: Arc<AtomicBool>,
    pump: Option<JoinHandle<()>>,
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
            pump: None,
        }
    }
}

impl Component for Ads122u04Component {
    fn pins(&self) -> &[PinDecl] {
        &ADS122U04_PINS
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        // -- power/reset gate ------------------------------------------
        {
            let gate = Arc::clone(&self.gate);
            io.on_sense("~RESET", move |state| gate.set_reset(state))?;
        }
        {
            let gate = Arc::clone(&self.gate);
            io.on_sense("DVDD", move |state| gate.set_dvdd(state))?;
        }
        {
            let gate = Arc::clone(&self.gate);
            io.on_sense("AVDD", move |state| gate.set_avdd(state))?;
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
            io.on_sense(pin, move |state| {
                let NetState::Analog(volts) = state else {
                    trace!(
                        pin,
                        ?state,
                        "ADS122U04: input has no numeric solve; holding last differential"
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

        // -- streams ----------------------------------------------------
        // RX pin → firmware-side pipe end (the protocol thread reads the
        // other end); gated so an unpowered / held-in-reset chip never sees
        // the command stream.
        {
            let gate = Arc::clone(&self.gate);
            let fd = Arc::clone(&self.firmware_fd);
            io.on_byte("RX", move |byte| {
                if gate.alive() {
                    write_all(fd.as_fd(), &[byte]);
                } else {
                    trace!(byte, "ADS122U04: RX byte ignored (unpowered or in reset)");
                }
            })?;
        }
        // Model output → TX pin, via the pump thread below.
        let tx = io.stream_tx("TX")?;
        if self.pump.is_none() {
            let fd = Arc::clone(&self.firmware_fd);
            let gate = Arc::clone(&self.gate);
            let shutdown = Arc::clone(&self.shutdown);
            self.pump = Some(
                std::thread::Builder::new()
                    .name("ads122u04-pump".into())
                    .spawn(move || pump_loop(&fd, &tx, &gate, &shutdown))
                    .expect("Failed to start ADS122U04 pump thread"),
            );
        }
        Ok(())
    }
}

impl Drop for Ads122u04Component {
    fn drop(&mut self) {
        self.shutdown.store(true, Ordering::Relaxed);
        if let Some(pump) = self.pump.take() {
            if pump.join().is_err() {
                warn!("ADS122U04: pump thread panicked");
            }
        }
        // Dropping `firmware_fd` (last Arc here once the pump has joined)
        // closes the pipe end; the model's protocol thread then reads EOF
        // and idles, exactly as it does in the hand-wired setup.
        debug!("ADS122U04 component shut down");
    }
}

// ============================================================
// Stream pump (model output → TX pin)
// ============================================================

/// Poll interval (virtual µs) for draining the model's output, following the
/// protocol model's own `protocol_loop` pacing rationale: substantially
/// finer than the fastest configured conversion interval (1 ms at 1000 SPS).
const PUMP_POLL_VIRTUAL_US: u64 = 250;

/// Wall-clock poll fallback while the virtual clock is uninitialized (e.g.
/// the build-time analysis attach, where the inert stream handle drops
/// writes anyway).
const PUMP_POLL_FALLBACK_WALL_US: u64 = 1_000;

/// Pump thread body: bytes the model emits (readable on the firmware-side
/// pipe end) are `stream_write()`n out the TX pin, gated exactly like RX —
/// a dead chip's output never reaches the wire, and bytes produced while
/// gated are discarded (not spooled for a later power-up). Exits when the
/// owning component drops.
fn pump_loop(fd: &OwnedFd, tx: &StreamTx, gate: &Gate, shutdown: &AtomicBool) {
    let mut buf = [0u8; 64];
    while !shutdown.load(Ordering::Relaxed) {
        match nix::unistd::read(fd.as_fd(), &mut buf) {
            Ok(0) => {} // peer end closed; keep idling until shutdown
            Ok(n) => {
                if gate.alive() {
                    tx.write(&buf[..n]);
                } else {
                    trace!(
                        discarded = n,
                        "ADS122U04: TX bytes discarded (unpowered or in reset)"
                    );
                }
                continue; // keep draining while data is flowing
            }
            Err(nix::errno::Errno::EAGAIN) => {}
            Err(e) => {
                warn!("ADS122U04: pump read error: {e}");
                break;
            }
        }
        let wall_us = if virtual_clock::is_initialized() {
            virtual_clock::virtual_to_wall_us(PUMP_POLL_VIRTUAL_US)
        } else {
            PUMP_POLL_FALLBACK_WALL_US
        };
        if wall_us > 0 {
            std::thread::sleep(Duration::from_micros(wall_us));
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

    const ANALOG_3V3: NetState = NetState::Analog(3.3);

    fn gate_with(reset: NetState, dvdd: NetState, avdd: NetState) -> Gate {
        let gate = Gate::new();
        gate.set_reset(reset);
        gate.set_dvdd(dvdd);
        gate.set_avdd(avdd);
        gate
    }

    /// Fresh gate: everything floating, chip dead — the engine has not yet
    /// delivered any sense, and the adapter must not assume power.
    #[rstest]
    fn gate_starts_dead() {
        assert!(!Gate::new().alive());
    }

    /// The full bench-good configuration: both rails at 3.3 V, reset tied
    /// high (the bodge) — alive.
    #[rstest]
    fn gate_alive_with_both_rails_and_reset_high() {
        assert!(gate_with(ANALOG_3V3, ANALOG_3V3, ANALOG_3V3).alive());
        // A digitally projected reset (e.g. the R10 pull-up rev) works too.
        assert!(gate_with(
            NetState::Pulled(Level::High, 10_000.0),
            ANALOG_3V3,
            ANALOG_3V3
        )
        .alive());
        assert!(gate_with(NetState::Driven(Level::High), ANALOG_3V3, ANALOG_3V3).alive());
    }

    /// The DS2Addon bench bug: a floating `~RESET` is a silent chip even
    /// with both supplies up.
    #[rstest]
    fn gate_dead_with_floating_reset() {
        assert!(!gate_with(NetState::Floating, ANALOG_3V3, ANALOG_3V3).alive());
    }

    /// Reset held low (or fought over) is a held-in-reset chip.
    #[rstest]
    fn gate_dead_with_reset_low_or_contended() {
        assert!(!gate_with(NetState::Driven(Level::Low), ANALOG_3V3, ANALOG_3V3).alive());
        assert!(!gate_with(NetState::Contention, ANALOG_3V3, ANALOG_3V3).alive());
    }

    /// POR requires BOTH supplies (SBAS752B): an unstrapped (floating) AVDD
    /// or DVDD — or a rail sourced at 0 V — is a dead chip.
    #[rstest]
    fn gate_dead_with_either_supply_down() {
        assert!(!gate_with(ANALOG_3V3, NetState::Floating, ANALOG_3V3).alive());
        assert!(!gate_with(ANALOG_3V3, ANALOG_3V3, NetState::Floating).alive());
        assert!(!gate_with(ANALOG_3V3, ANALOG_3V3, NetState::Analog(0.0)).alive());
        assert!(!gate_with(ANALOG_3V3, NetState::Analog(1.0), ANALOG_3V3).alive());
    }

    /// V_IH scales with the solved DVDD rail (0.7 · DVDD, SBAS752B §7.3):
    /// 2.4 V clears the threshold at DVDD = 3.3 V (V_IH = 2.31 V) but not at
    /// DVDD = 5.0 V (V_IH = 3.5 V).
    #[rstest]
    fn reset_threshold_tracks_dvdd() {
        let reset = NetState::Analog(2.4);
        assert!(gate_with(reset, ANALOG_3V3, ANALOG_3V3).alive());
        assert!(!gate_with(reset, NetState::Analog(5.0), NetState::Analog(5.0)).alive());
        // Just below the 3.3 V threshold: dead.
        assert!(!gate_with(NetState::Analog(2.3), ANALOG_3V3, ANALOG_3V3).alive());
    }

    /// The shared pin table stays the SBAS752B p.3 truth: 16 pins, UART
    /// stream roles on TX (15, producer) and RX (16, consumer) only.
    #[rstest]
    fn pin_table_declares_the_uart_stream_pins() {
        assert_eq!(ADS122U04_PINS.len(), 16);
        for decl in &ADS122U04_PINS {
            match decl.number {
                "15" => assert_eq!(
                    decl.stream,
                    Some(StreamRole::Producer {
                        baud_hz: ADS122U04_BAUD_HZ
                    })
                ),
                "16" => assert_eq!(
                    decl.stream,
                    Some(StreamRole::Consumer {
                        baud_hz: ADS122U04_BAUD_HZ
                    })
                ),
                _ => assert_eq!(decl.stream, None, "pin {} must not stream", decl.number),
            }
        }
    }
}
