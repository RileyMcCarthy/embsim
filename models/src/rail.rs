//! Model: a **regulator / rail** — a buck, an LDO, an isolated DC/DC —
//! whose output is a *declared terminal* (`NODES.md` §2, the Regulator /
//! rail row; "Three rules the taxonomy rests on", 1).
//!
//! The part senses its input against its own ground pin and, where it has
//! one, its enable; from the instant the input is above the part's rising
//! threshold and the enable asserts, plus the datasheet's soft-start, it
//! publishes on its output pin a Thevenin drive of `V(reference) + v_set`
//! behind its output impedance — one drive, at a real scheduled instant, a
//! **step** (the ramp is not modelled). Down, the output is **released**:
//! a real buck or LDO output is high-impedance, so a bench strap onto a
//! down rail sources it instead of fighting it, and the build reports
//! [`embsim_board::Finding::RailDown`]. A 0 V drive is published only by a
//! part with an *active output discharge*, cited, and only while it is
//! disabled with its input up.
//!
//! `v_set` is one of three things, each read **once at attach** and never
//! again: a fixed voltage from the part's ordering suffix; the feedback
//! divider the netlist carries, read through the build-time topology query
//! [`ComponentNetIo::resistors_at`] as `V_FB · (1 + R_top / R_bot)`, so
//! two bucks with one registry key come out different, as they are; or the
//! strap on a select pin (shorted to the output, to the ground, or through
//! a resistor to either).
//!
//! An isolated DC/DC declares its isolated ground as a terminal too
//! ([`RailRole::IsolatedGround`], a `PowerOut` pin that idles released and
//! is never driven): the isolated domain's reference is whatever the board
//! or the harness ties it to (`DESIGN.md` rule 6 — no implicit ground), and
//! a domain nothing ties has no reference, so its rail stays down with the
//! reason named. The output is published as an absolute voltage: the
//! engine's terminal holds absolute volts, and the model adds its
//! reference itself (`NODES.md` §8, the phase-4 engine record).
//!
//! # Datasheet provenance
//!
//! Four parts, each governed by its own document; every number below
//! carries its section and printed page.
//!
//! **Diodes Incorporated AP62301** — *AP62300/AP62301/AP62300T, 4.2V to
//! 18V Input, 3A Low IQ Synchronous Buck Converter*, document DS41958
//! Rev. 4-2, July 2023. Pin Descriptions (p.3): SOT563 1 `VIN`, 2 `SW`,
//! 3 `GND`, 4 `BST`, 5 `EN` ("Leave floating for automatic startup"),
//! 6 `FB`. Electrical Characteristics (p.5): `V_FB` 0.800 V (0.792–0.808 at
//! 25 °C); `POR` VIN power-on-reset rising 3.90 V; `UVLO` VIN falling 3.6 V;
//! `V_EN_H` 1.20 V / `V_EN_L` 1.10 V; `t_SS` 2.5 ms. Application
//! Information §3 Enable (p.13): "An internal 1.5µA pull-up current source …
//! guarantees that if EN is left floating, the device still automatically
//! enables"; "built-in 2.5ms soft-start time". §9 Setting the Output
//! Voltage, Eq. 8 (p.17): `R1 = R2 · (VOUT / V_FB − 1)`, `R1` from `FB` to
//! the output, `R2` from `FB` to ground. Figure 28 Load Regulation (p.10):
//! 4.99 V at 0 A to 4.965 V at 3 A, read from the plot — the output
//! impedance recorded for the I-V port. No active output discharge is
//! specified (Figure 11 shows the output decaying through its load).
//! The P2-EC32MB carries two: `U402` (`R401` 13.3 kΩ / `R403` 10.5 kΩ →
//! 1.813 V on `Common_VDD`) and `U403` (`R402` 37.4 kΩ / `R404` 10.5 kΩ →
//! 3.649 V on `Common_LDOin`), both with `EN` unconnected (the vendor's
//! note in the netlist header: "EN NC = Default 2.5ms startup").
//!
//! **onsemi NCP114** — *Voltage Regulator – CMOS Low Dropout, 300 mA*,
//! publication NCP114/D Rev. 31, January 2024. Pin Function Description
//! (p.2): UDFN4 1 `OUT`, 2 `GND`, 3 `EN` ("Driving EN over 0.9 V turns on
//! the regulator. Driving EN below 0.4 V puts the regulator into shutdown
//! mode"), 4 `IN`; the block-diagram note: "Active output discharge
//! function is present only in NCP114AMXyyyTCG devices". Electrical
//! Characteristics (p.3): `V_IN` 1.7–5.5 V; output voltage accuracy ±2 %
//! (`V_OUT` > 2.0 V); load regulation UDFN 12 mV typ (1 mA to 300 mA);
//! `V_EN_HI` 0.9 V min, `V_EN_LO` 0.4 V max; `R_DIS` active output discharge
//! resistance 100 Ω typ (`V_EN` < 0.4 V, Version A only); dropout 135 mV
//! typ at 300 mA for 3.3 V. Applications Information, Turn-On Time (p.14):
//! no figure is specified for the 3.3 V option — the paragraph gives one
//! worked example (90 µs at `V_OUT` = 1.2 V, `C_OUT` = 1 µF) and says the
//! time depends on `V_OUT`, `C_OUT` and `T_A` — so the output is a step at
//! the enable instant, stated. The module's `U501`–`U508` are
//! `NCP114AMX330TCG`: Version A, 3.3 V, with the discharge.
//!
//! **XLSEMI XL1509** — *2A 150KHz 40V Buck DC to DC Converter*, datasheet
//! Rev 2.6 (xlsemi.com). Table 1 Pin Description (p.2): SOP8 1 `VIN`
//! (4.5–40 V), 2 `SW` ("the switch node that supplies power to the
//! output"), 3 `FB` ("The feedback threshold voltage is 1.23V"), 4 `EN`
//! ("Drive EN pin low to turn on the device, drive it high to turn it off.
//! Floating is default low"), 5–8 `GND`. Electrical Characteristics (p.5):
//! XL1509-3.3 `V_OUT` 3.168 / 3.3 / 3.432 V; XL1509-5.0 4.8 / 5 / 5.2 V;
//! XL1509-12 11.52 / 12 / 12.48 V. DC Parameters (p.6): input operation
//! voltage 4.5–40 V; `EN` pin threshold 1.4 V high (regulator OFF), 0.8 V
//! low (regulator ON). No soft-start or start-up time is specified, so the
//! output is a step at the instant the input and enable allow; no load
//! regulation figure is given ("Excellent line and load regulation",
//! Features p.1), so no output impedance is recorded and the terminal is
//! the ideal source every terminal is in the solve. The Edge board's `U1`
//! (`XL1509-5V`) and `U2` (`XL1509-3.3V`) name their version in the value.
//!
//! **Texas Instruments UCC12040** — *High-Density, Low-EMI, 3-kVRMS Basic
//! Isolation DC/DC Module*, SNVSBO5B, December 2019 – revised May 2021.
//! Table 5-1 Pin Functions (p.3): DVE SOIC-16 1 `EN`, 2 `GNDP`, 3 `VINP`,
//! 4 `SYNC`, 5 `SYNC_OK`, 6–8 `NC` (primary side), 9 `GNDS`, 10–12 `NC`
//! (isolated side), 13 `SEL` ("V_ISO setpoint is 5.0 V when SEL is shorted
//! to VISO, 5.4 V when SEL is connected to VISO through a 100-kΩ resistor,
//! 3.3 V when SEL is shorted to GNDS, and 3.7 V when SEL is connected to
//! GNDS through a 100-kΩ resistor"), 14 `VISO`, 15 `GNDS` ("Secondary side
//! ground return connection for VISO"), 16 `GNDS`. §6.9 Electrical
//! Characteristics (p.7): `V_UVPR` 4.2 V, `V_UVPF` 3.7 V; `V_IR` logic high
//! rising 2.2 V max, `V_IF` logic low falling 0.8 V min; `V_ISO` (SEL
//! shorted to VISO) 4.7 / 5 / 5.3 V. (p.8): `V_ISO(LOAD)` 1.5 % (0 to
//! 100 mA, 5.0 V output); `t_RISE` VISO rise time 10 %–90 % 750 µs (EN low
//! to high, SEL shorted to VISO). §7.3.1 (p.15): "The EN pin has a weak
//! internal pull-down resistor, so the device floats to the disable state
//! if the pin is left open." Table 7-1 Device Functional Modes (p.18): EN
//! low → 0 V (a setpoint, no discharge path is characterised: released
//! here). The Edge board's `IC3` and `IC4` tie `SEL` to `VISO`: 5.0 V.
//!
//! # Deliberate simplifications
//!
//! - **The soft-start is a step at its end**, at `t(input above threshold
//!   and enabled) + t_SS`, on the wheel as an integer nanosecond; the ramp
//!   between is not modelled (`DESIGN.md` rule 5: no timestep).
//! - **Tolerances are recorded, not applied**: the nominal is published
//!   (`DESIGN.md` §4).
//! - **Dropout is not modelled**: the output is `v_set` whenever the input
//!   is above the part's threshold. On the reference boards every input
//!   has headroom (the module's LDOs see 3.649 V for 3.3 V out; the Edge
//!   board's bucks see 12 V).
//! - **Current limit, thermal shutdown, hiccup and the load's effect on
//!   the output are not modelled**: a sagging rail is not a terminal
//!   (`NODES.md` §2). The output impedance is recorded on the terminal for
//!   the I-V port and never solved.
//! - **A level with no voltage on the input** (`Driven`/`Pulled` from a
//!   source the engine has no numeric voltage for) reads as above the
//!   threshold when high and below it when low, the projection the
//!   isolators' supply gate makes too; a level on the reference pin reads
//!   as 0 V when low and as no reference when high.
//! - **A reference no source reaches decides nothing.** A voltage on the
//!   input or the enable is a voltage *relative to the part's ground pin*,
//!   and with that pin floating there is nothing to take it relative to:
//!   the input reads as not up and an analog enable as undecided (the
//!   part disabled), so the monitor never reports an input "up" against
//!   an assumed 0 V (`DESIGN.md` rule 6: no implicit ground). A level on
//!   the enable needs no reference and is read as before. The output was
//!   already held released without a reference; this keeps the monitor's
//!   reasons as honest as its drive.
//! - **`SYNC`, `SYNC_OK`** (UCC12040): the internal oscillator is assumed
//!   (`SYNC` is tied to `GNDP` on the reference board) and `SYNC_OK`, an
//!   open-drain diagnostic, rests released.

use std::fmt;
use std::sync::{Arc, Mutex};

use embsim_board::{
    Amps, AttachError, Component, ComponentNetIo, DeadBand, Level, NetId, Ohms, PinDecl, PinHandle,
    Sense, TheveninDrive, Thresholds, Volts,
};
use embsim_core::virtual_clock;

// ============================================================
// Datasheet constants
// ============================================================

/// AP62301 `V_FB`, 0.800 V typ (DS41958 Electrical Characteristics, p.5;
/// 0.792–0.808 V at 25 °C, CCM).
pub const AP62301_V_FB_VOLTS: Volts = 0.800;
/// AP62301 reference tolerance, ±1 % ("0.8V ± 1% Reference Voltage",
/// DS41958 Features, p.1).
pub const AP62301_V_FB_TOLERANCE: f64 = 0.01;
/// AP62301 `POR`, VIN power-on-reset rising threshold, 3.90 V typ
/// (DS41958 p.5).
pub const AP62301_POR_RISING_VOLTS: Volts = 3.90;
/// AP62301 `UVLO`, VIN undervoltage lockout falling threshold, 3.6 V typ
/// (DS41958 p.5).
pub const AP62301_UVLO_FALLING_VOLTS: Volts = 3.6;
/// AP62301 `t_SS`, soft-start time, 2.5 ms typ (DS41958 p.5; "built-in
/// 2.5ms soft-start time", §3 Enable, p.13).
pub const AP62301_SOFT_START_NS: u64 = 2_500_000;
/// AP62301 `V_EN_H`, EN logic high threshold, 1.20 V typ (DS41958 p.5).
pub const AP62301_EN_HIGH_VOLTS: Volts = 1.20;
/// AP62301 `V_EN_L`, EN logic low threshold, 1.10 V typ (DS41958 p.5).
pub const AP62301_EN_LOW_VOLTS: Volts = 1.10;
/// AP62301 load regulation: 4.99 V at 0 A to 4.965 V at 3 A, read from
/// Figure 28 (DS41958 p.10, `VIN` = 12 V, `VOUT` = 5 V).
pub const AP62301_LOAD_REGULATION_VOLTS: Volts = 0.025;
/// The load span of [`AP62301_LOAD_REGULATION_VOLTS`].
pub const AP62301_LOAD_REGULATION_AMPS: Amps = 3.0;
/// AP62301 output impedance derived from its load regulation, ≈ 8.3 mΩ.
pub const AP62301_Z_OUT_OHMS: Ohms = AP62301_LOAD_REGULATION_VOLTS / AP62301_LOAD_REGULATION_AMPS;

/// NCP114 operating input voltage, 1.7 V min (NCP114/D Electrical
/// Characteristics, p.3; 5.5 V max).
pub const NCP114_V_IN_MIN_VOLTS: Volts = 1.7;
/// NCP114 `V_EN_HI`, 0.9 V min: "Driving EN over 0.9 V turns on the
/// regulator" (NCP114/D p.2, p.3).
pub const NCP114_EN_ON_VOLTS: Volts = 0.9;
/// NCP114 `V_EN_LO`, 0.4 V max: "Driving EN below 0.4 V puts the regulator
/// into shutdown mode" (NCP114/D p.2, p.3).
pub const NCP114_EN_OFF_VOLTS: Volts = 0.4;
/// NCP114 output voltage accuracy, ±2 % for `V_OUT` > 2.0 V (NCP114/D p.3).
pub const NCP114_OUTPUT_TOLERANCE: f64 = 0.02;
/// NCP114 load regulation, UDFN package, 12 mV typ over `I_OUT` = 1 mA to
/// 300 mA (NCP114/D p.3).
pub const NCP114_LOAD_REGULATION_VOLTS: Volts = 0.012;
/// The load span of [`NCP114_LOAD_REGULATION_VOLTS`], 299 mA.
pub const NCP114_LOAD_REGULATION_AMPS: Amps = 0.299;
/// NCP114 output impedance derived from its load regulation, ≈ 40 mΩ.
pub const NCP114_Z_OUT_OHMS: Ohms = NCP114_LOAD_REGULATION_VOLTS / NCP114_LOAD_REGULATION_AMPS;
/// NCP114 `R_DIS`, active output discharge resistance, 100 Ω typ at
/// `V_EN` < 0.4 V — Version A (`NCP114AMXyyyTCG`) only (NCP114/D p.3; the
/// block-diagram note, p.2; Ordering Information, p.15).
pub const NCP114_DISCHARGE_OHMS: Ohms = 100.0;
/// NCP114 dropout, 135 mV typ at 300 mA for the 3.3 V option, UDFN
/// (NCP114/D p.3). Recorded; not modelled.
pub const NCP114_DROPOUT_VOLTS: Volts = 0.135;

/// XL1509 input operation voltage, 4.5 V min (Rev 2.6 DC Parameters, p.6;
/// 40 V max).
pub const XL1509_V_IN_MIN_VOLTS: Volts = 4.5;
/// XL1509 `EN` pin threshold, high — regulator OFF — 1.4 V typ (Rev 2.6
/// p.6).
pub const XL1509_EN_OFF_VOLTS: Volts = 1.4;
/// XL1509 `EN` pin threshold, low — regulator ON — 0.8 V typ (Rev 2.6
/// p.6).
pub const XL1509_EN_ON_VOLTS: Volts = 0.8;
/// XL1509-5.0 `V_OUT`, 5 V typ, 4.8–5.2 V (Rev 2.6 p.5).
pub const XL1509_5V0_VOLTS: Volts = 5.0;
/// XL1509-3.3 `V_OUT`, 3.3 V typ, 3.168–3.432 V (Rev 2.6 p.5).
pub const XL1509_3V3_VOLTS: Volts = 3.3;
/// XL1509-12 `V_OUT`, 12 V typ, 11.52–12.48 V (Rev 2.6 p.5).
pub const XL1509_12V_VOLTS: Volts = 12.0;
/// XL1509 fixed-output tolerance, ±4 % (Rev 2.6 p.5: 4.8–5.2 V on 5 V,
/// 3.168–3.432 V on 3.3 V).
pub const XL1509_OUTPUT_TOLERANCE: f64 = 0.04;
/// XL1509 feedback threshold, 1.23 V (Rev 2.6 Table 1, p.2; the ADJ
/// version's `V_OUT` 1.193–1.267 V, p.5).
pub const XL1509_V_FB_VOLTS: Volts = 1.23;
/// XL1509 minimum dropout, 1.5 V (Rev 2.6 Features, p.1). Recorded; not
/// modelled.
pub const XL1509_DROPOUT_VOLTS: Volts = 1.5;

/// UCC12040 `V_UVPR`, VINP under-voltage lockout rising threshold, 4.2 V
/// typ (SNVSBO5B §6.9, p.7).
pub const UCC12040_UVLO_RISING_VOLTS: Volts = 4.2;
/// UCC12040 `V_UVPF`, VINP under-voltage lockout falling threshold, 3.7 V
/// typ (SNVSBO5B §6.9, p.7).
pub const UCC12040_UVLO_FALLING_VOLTS: Volts = 3.7;
/// UCC12040 `V_IR`, EN input threshold logic high, rising edge, 2.2 V max
/// (SNVSBO5B §6.9, p.7).
pub const UCC12040_EN_HIGH_VOLTS: Volts = 2.2;
/// UCC12040 `V_IF`, EN input threshold logic low, falling edge, 0.8 V min
/// (SNVSBO5B §6.9, p.7).
pub const UCC12040_EN_LOW_VOLTS: Volts = 0.8;
/// UCC12040 `V_ISO` with `SEL` shorted to `VISO`, 5.0 V (SNVSBO5B Table
/// 5-1 p.3, Table 7-1 p.18; 4.7–5.3 V, §6.9 p.7).
pub const UCC12040_VISO_SEL_TO_VISO_VOLTS: Volts = 5.0;
/// UCC12040 `V_ISO` with `SEL` through 100 kΩ to `VISO`, 5.4 V (Table
/// 5-1 p.3, Table 7-1 p.18).
pub const UCC12040_VISO_SEL_100K_TO_VISO_VOLTS: Volts = 5.4;
/// UCC12040 `V_ISO` with `SEL` shorted to `GNDS`, 3.3 V (Table 5-1 p.3,
/// Table 7-1 p.18).
pub const UCC12040_VISO_SEL_TO_GNDS_VOLTS: Volts = 3.3;
/// UCC12040 `V_ISO` with `SEL` through 100 kΩ to `GNDS`, 3.7 V (Table 5-1
/// p.3, Table 7-1 p.18).
pub const UCC12040_VISO_SEL_100K_TO_GNDS_VOLTS: Volts = 3.7;
/// The `SEL` strap resistor Table 5-1 names, 100 kΩ.
pub const UCC12040_SEL_RESISTOR_OHMS: Ohms = 100_000.0;
/// UCC12040 `V_ISO` tolerance at the 5.0 V setpoint, ±6 % (4.7–5.3 V,
/// SNVSBO5B §6.9, p.7).
pub const UCC12040_VISO_TOLERANCE: f64 = 0.06;
/// UCC12040 `V_ISO(LOAD)`, DC load regulation, 1.5 % typ over 0 to 100 mA
/// at the 5.0 V output (SNVSBO5B §6.9, p.8).
pub const UCC12040_LOAD_REGULATION: f64 = 0.015;
/// The load span of [`UCC12040_LOAD_REGULATION`], 100 mA.
pub const UCC12040_LOAD_REGULATION_AMPS: Amps = 0.100;
/// UCC12040 output impedance at 5.0 V derived from its load regulation,
/// 0.75 Ω.
pub const UCC12040_Z_OUT_OHMS: Ohms =
    UCC12040_LOAD_REGULATION * UCC12040_VISO_SEL_TO_VISO_VOLTS / UCC12040_LOAD_REGULATION_AMPS;
/// UCC12040 `t_RISE`, VISO rise time 10 %–90 %, 750 µs typ, EN low to high
/// with `SEL` shorted to `VISO` (SNVSBO5B §6.9, p.8) — the rail's
/// soft-start: the step lands at its end.
pub const UCC12040_RISE_NS: u64 = 750_000;
/// UCC12040 `V_INP`, primary side supply voltage, 4.5–5.5 V (SNVSBO5B
/// §6.3, p.4). Recorded; the UVLO thresholds are the gate.
pub const UCC12040_V_INP_MIN_VOLTS: Volts = 4.5;

// ============================================================
// Configuration
// ============================================================

/// How the part's output voltage is set — each read once at attach.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum VSet {
    /// A fixed voltage the part's ordering suffix names.
    Fixed(Volts),
    /// The feedback divider the netlist carries on the [`RailRole::Feedback`]
    /// pin: `v_fb · (1 + R_top / R_bot)`, `R_bot` the resistor returning to
    /// the part's own ground pin, `R_top` the other.
    Divider {
        /// The feedback reference voltage.
        v_fb: Volts,
    },
    /// A strap on the [`RailRole::Select`] pin: shorted to the output, to the
    /// isolated ground, or through a resistor to either.
    Select {
        /// `SEL` shorted to the output.
        shorted_to_output: Volts,
        /// `SEL` through [`VSet::Select::resistor_ohms`] to the output.
        resistor_to_output: Volts,
        /// `SEL` shorted to the ground.
        shorted_to_ground: Volts,
        /// `SEL` through the resistor to the ground.
        resistor_to_ground: Volts,
        /// The strap resistor the datasheet names.
        resistor_ohms: Ohms,
    },
}

/// Which way an enable pin asserts.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EnableSense {
    /// Enabled when the pin is high.
    High,
    /// Enabled when the pin is low.
    Low,
}

/// The enable pin's thresholds, with hysteresis: the part runs once the
/// pin crosses `assert_volts` in the asserting direction and stops once it
/// crosses `deassert_volts` in the other, holding its state between.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EnableSpec {
    /// Which way the pin asserts.
    pub sense: EnableSense,
    /// The pin voltage at or beyond which the part is enabled (above it
    /// for [`EnableSense::High`], below it for [`EnableSense::Low`]).
    pub assert_volts: Volts,
    /// The pin voltage at or beyond which the part is disabled.
    pub deassert_volts: Volts,
    /// Whether a pin no source reaches — left open — enables the part (an
    /// internal pull-up) or disables it (an internal pull-down).
    pub floating_enables: bool,
}

/// A rail's datasheet configuration.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Config {
    /// The part, for messages.
    pub part: &'static str,
    /// How the output voltage is set.
    pub v_set: VSet,
    /// The input voltage (against the part's ground) at or above which the
    /// part starts: its POR / UVLO rising threshold, or its minimum
    /// operating input where the datasheet names no lockout.
    pub input_on_volts: Volts,
    /// The input voltage below which a running part stops: its UVLO
    /// falling threshold, or `input_on_volts` where none is named.
    pub input_off_volts: Volts,
    /// The enable pin's thresholds; `None` for a part with no enable. A
    /// part whose facade declares no [`RailRole::Enable`] pin reads the
    /// enable as left open ([`EnableSpec::floating_enables`]).
    pub enable: Option<EnableSpec>,
    /// The soft-start: the output steps up this long after the input and
    /// enable allow. Zero is the same instant.
    pub start_up_ns: u64,
    /// The output impedance recorded on the terminal for the I-V port,
    /// derived from the load regulation; `None` where the datasheet names
    /// none (the terminal is then the ideal source every terminal is in
    /// the solve).
    pub z_out_ohms: Option<Ohms>,
    /// The active output discharge, if the part has one: the output is
    /// held at its reference through this while the part is disabled with
    /// its input up.
    pub discharge_ohms: Option<Ohms>,
    /// The output tolerance, as a fraction of the setpoint. Recorded; the
    /// nominal is published.
    pub tolerance: f64,
}

impl Config {
    /// The Diodes AP62301 buck: `v_set` from the netlist's feedback divider
    /// at `V_FB` = 0.800 V, POR 3.90 V / UVLO 3.6 V on `VIN`, a 2.5 ms
    /// soft-start, an enable that floats to on.
    pub const fn ap62301() -> Self {
        Self {
            part: "AP62301",
            v_set: VSet::Divider {
                v_fb: AP62301_V_FB_VOLTS,
            },
            input_on_volts: AP62301_POR_RISING_VOLTS,
            input_off_volts: AP62301_UVLO_FALLING_VOLTS,
            enable: Some(EnableSpec {
                sense: EnableSense::High,
                assert_volts: AP62301_EN_HIGH_VOLTS,
                deassert_volts: AP62301_EN_LOW_VOLTS,
                floating_enables: true,
            }),
            start_up_ns: AP62301_SOFT_START_NS,
            z_out_ohms: Some(AP62301_Z_OUT_OHMS),
            discharge_ohms: None,
            tolerance: AP62301_V_FB_TOLERANCE,
        }
    }

    /// The onsemi NCP114 LDO, Version A (with the active output discharge),
    /// at a fixed `v_set`: runs from 1.7 V in with `EN` over 0.9 V, stops
    /// with `EN` under 0.4 V, no soft-start figure (a step), 100 Ω to
    /// ground while disabled.
    pub const fn ncp114(v_set: Volts) -> Self {
        Self {
            part: "NCP114",
            v_set: VSet::Fixed(v_set),
            input_on_volts: NCP114_V_IN_MIN_VOLTS,
            input_off_volts: NCP114_V_IN_MIN_VOLTS,
            enable: Some(EnableSpec {
                sense: EnableSense::High,
                assert_volts: NCP114_EN_ON_VOLTS,
                deassert_volts: NCP114_EN_OFF_VOLTS,
                floating_enables: false,
            }),
            start_up_ns: 0,
            z_out_ohms: Some(NCP114_Z_OUT_OHMS),
            discharge_ohms: Some(NCP114_DISCHARGE_OHMS),
            tolerance: NCP114_OUTPUT_TOLERANCE,
        }
    }

    /// The NCP114 at the voltage its value field names — `"LDO 300mA,
    /// 3.3V"` → 3.3 V: the first token that is a decimal number followed by
    /// `V`. `None` when no token says a voltage.
    pub fn ncp114_from_value(value: &str) -> Option<Self> {
        parse_volts_token(value).map(Self::ncp114)
    }

    /// The XLSEMI XL1509 fixed-output buck at `v_set`: runs from 4.5 V in
    /// with `EN` under 0.8 V (the pin floats to on), stops with `EN` over
    /// 1.4 V, no soft-start or load regulation figure.
    pub const fn xl1509(v_set: Volts) -> Self {
        Self {
            part: "XL1509",
            v_set: VSet::Fixed(v_set),
            input_on_volts: XL1509_V_IN_MIN_VOLTS,
            input_off_volts: XL1509_V_IN_MIN_VOLTS,
            enable: Some(EnableSpec {
                sense: EnableSense::Low,
                assert_volts: XL1509_EN_ON_VOLTS,
                deassert_volts: XL1509_EN_OFF_VOLTS,
                floating_enables: true,
            }),
            start_up_ns: 0,
            z_out_ohms: None,
            discharge_ohms: None,
            tolerance: XL1509_OUTPUT_TOLERANCE,
        }
    }

    /// The XL1509 version a value field names: `"XL1509-5V"` → 5.0 V,
    /// `"XL1509-3.3V"` → 3.3 V, `"XL1509-12V"` → 12 V — the suffix after the
    /// `-` as a decimal number ending in `V` (an ordering-code spelling such
    /// as `5.0E1` reads the same). `None` for the adjustable version or a
    /// value naming no voltage.
    pub fn xl1509_from_value(value: &str) -> Option<Self> {
        let suffix = value.trim().strip_prefix("XL1509-")?;
        let digits: String = suffix
            .chars()
            .take_while(|c| c.is_ascii_digit() || *c == '.')
            .collect();
        if digits.is_empty() || digits.matches('.').count() > 1 {
            return None;
        }
        let volts: f64 = digits.parse().ok()?;
        (volts > 0.0).then(|| Self::xl1509(volts))
    }

    /// The TI UCC12040 isolated DC/DC: `v_set` from the `SEL` strap, UVLO
    /// 4.2 V / 3.7 V on `VINP`, `EN` over 2.2 V on and under 0.8 V off (the
    /// pin floats to off), a 750 µs rise.
    pub const fn ucc12040() -> Self {
        Self {
            part: "UCC12040",
            v_set: VSet::Select {
                shorted_to_output: UCC12040_VISO_SEL_TO_VISO_VOLTS,
                resistor_to_output: UCC12040_VISO_SEL_100K_TO_VISO_VOLTS,
                shorted_to_ground: UCC12040_VISO_SEL_TO_GNDS_VOLTS,
                resistor_to_ground: UCC12040_VISO_SEL_100K_TO_GNDS_VOLTS,
                resistor_ohms: UCC12040_SEL_RESISTOR_OHMS,
            },
            input_on_volts: UCC12040_UVLO_RISING_VOLTS,
            input_off_volts: UCC12040_UVLO_FALLING_VOLTS,
            enable: Some(EnableSpec {
                sense: EnableSense::High,
                assert_volts: UCC12040_EN_HIGH_VOLTS,
                deassert_volts: UCC12040_EN_LOW_VOLTS,
                floating_enables: false,
            }),
            start_up_ns: UCC12040_RISE_NS,
            z_out_ohms: Some(UCC12040_Z_OUT_OHMS),
            discharge_ohms: None,
            tolerance: UCC12040_VISO_TOLERANCE,
        }
    }
}

/// The first token of `value` that is a decimal number followed by `V`
/// (`"3.3V"`), as volts.
fn parse_volts_token(value: &str) -> Option<Volts> {
    value
        .split(|c: char| c.is_whitespace() || c == ',')
        .find_map(|token| {
            let digits = token.strip_suffix('V')?;
            if digits.is_empty()
                || !digits.chars().all(|c| c.is_ascii_digit() || c == '.')
                || digits.matches('.').count() > 1
            {
                return None;
            }
            let volts: f64 = digits.parse().ok()?;
            (volts > 0.0).then_some(volts)
        })
}

// ============================================================
// Pin facades
// ============================================================

/// What a rail's pin is for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RailRole {
    /// The supply the part regulates from (`PowerIn`), read against
    /// [`RailRole::Ground`].
    Input,
    /// The part's ground (`PowerIn`): the input's reference and, unless the
    /// part declares an [`RailRole::IsolatedGround`], the output's.
    Ground,
    /// A further ground pin (`PowerIn`), no role of its own.
    ExtraGround,
    /// The regulated output — a declared terminal (`PowerOut`), released
    /// until the part publishes.
    Output,
    /// An isolated output's ground — a declared terminal (`PowerOut`) the
    /// part never drives: the isolated domain's reference, held by
    /// whatever the board or the harness ties it to.
    IsolatedGround,
    /// The enable (a digital input), read through [`EnableSpec`].
    Enable,
    /// The feedback pin (`Passive`): the divider on it is read once at
    /// attach; the part senses and drives nothing on it live.
    Feedback,
    /// The select pin (`Passive`): its strap is read once at attach.
    Select,
    /// An open-drain status output (sinks only, released, not modelled).
    StatusOut,
    /// A pin the model reads nothing from (`Passive`): a bootstrap node, a
    /// sync input tied off, a no-connect.
    Passive,
}

/// One row of a rail's pin table.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RailPin {
    /// The identifier the netlist uses (a number, or a function on a
    /// transcribed netlist).
    pub number: &'static str,
    /// An alias, when the identifier is not already the function.
    pub name: Option<&'static str>,
    /// The pin's role.
    pub role: RailRole,
}

const fn rail_pin(number: &'static str, name: Option<&'static str>, role: RailRole) -> RailPin {
    RailPin { number, name, role }
}

/// AP62301 keyed by **function**, as the P2-EC32MB netlist names
/// `U402`/`U403`'s pins: the vendor left `EN` unconnected, and the
/// transcription emits no node for it (DS41958 p.3: floating = enabled).
pub const AP62301_PINS_BY_FUNCTION: [RailPin; 5] = [
    rail_pin("VIN", None, RailRole::Input),
    rail_pin("GND", None, RailRole::Ground),
    rail_pin("BST", None, RailRole::Passive),
    rail_pin("FB", None, RailRole::Feedback),
    rail_pin("SW", None, RailRole::Output),
];

/// AP62301 in the SOT563 package by pin number (DS41958 Pin Descriptions,
/// p.3): 1 `VIN`, 2 `SW`, 3 `GND`, 4 `BST`, 5 `EN`, 6 `FB`.
pub const AP62301_PINS_SOT563: [RailPin; 6] = [
    rail_pin("1", Some("VIN"), RailRole::Input),
    rail_pin("2", Some("SW"), RailRole::Output),
    rail_pin("3", Some("GND"), RailRole::Ground),
    rail_pin("4", Some("BST"), RailRole::Passive),
    rail_pin("5", Some("EN"), RailRole::Enable),
    rail_pin("6", Some("FB"), RailRole::Feedback),
];

/// NCP114 keyed by **function**, as the P2-EC32MB netlist names
/// `U501`–`U508`'s pins (`GND_P` is the vendor's "GND-P", the exposed pad
/// drawn as a pin).
pub const NCP114_PINS_BY_FUNCTION: [RailPin; 5] = [
    rail_pin("EN", None, RailRole::Enable),
    rail_pin("IN", None, RailRole::Input),
    rail_pin("GND", None, RailRole::Ground),
    rail_pin("GND_P", None, RailRole::ExtraGround),
    rail_pin("OUT", None, RailRole::Output),
];

/// NCP114 in the UDFN4 package by pin number (NCP114/D Pin Function
/// Description, p.2): 1 `OUT`, 2 `GND`, 3 `EN`, 4 `IN`.
pub const NCP114_PINS_UDFN4: [RailPin; 4] = [
    rail_pin("1", Some("OUT"), RailRole::Output),
    rail_pin("2", Some("GND"), RailRole::Ground),
    rail_pin("3", Some("EN"), RailRole::Enable),
    rail_pin("4", Some("IN"), RailRole::Input),
];

/// XL1509 in the SOP8 package by pin number (Rev 2.6 Table 1, p.2), with
/// the aliases the MaD Edge board's symbol prints: 1 `VIN`, 2 `OUT` (the
/// datasheet's `SW`), 3 `FDB` (the fixed version's feedback, sensing the
/// output through the internal divider — nothing to read), 4 `~ON` (the
/// datasheet's `EN`, low = on), 5–8 `GND`.
pub const XL1509_PINS_SOP8: [RailPin; 8] = [
    rail_pin("1", Some("VIN"), RailRole::Input),
    rail_pin("2", Some("OUT"), RailRole::Output),
    rail_pin("3", Some("FDB"), RailRole::Passive),
    rail_pin("4", Some("~ON"), RailRole::Enable),
    rail_pin("5", Some("GND"), RailRole::Ground),
    rail_pin("6", Some("GND"), RailRole::ExtraGround),
    rail_pin("7", Some("GND"), RailRole::ExtraGround),
    rail_pin("8", Some("GND"), RailRole::ExtraGround),
];

/// UCC12040 in the DVE SOIC-16 package by pin number (SNVSBO5B Table
/// 5-1, p.3). Pin 15 is the isolated domain's reference terminal ("ground
/// return connection for VISO"); pins 9 and 16 are the further `GNDS`
/// connections ("Do not use as only ground connection for VISO").
pub const UCC12040_PINS_SOIC16: [RailPin; 16] = [
    rail_pin("1", Some("EN"), RailRole::Enable),
    rail_pin("2", Some("GNDP"), RailRole::Ground),
    rail_pin("3", Some("VINP"), RailRole::Input),
    rail_pin("4", Some("SYNC"), RailRole::Passive),
    rail_pin("5", Some("SYNC_OK"), RailRole::StatusOut),
    rail_pin("6", Some("NC_1"), RailRole::Passive),
    rail_pin("7", Some("NC_2"), RailRole::Passive),
    rail_pin("8", Some("NC_3"), RailRole::Passive),
    rail_pin("9", Some("GNDS_1"), RailRole::ExtraGround),
    rail_pin("10", Some("NC_4"), RailRole::Passive),
    rail_pin("11", Some("NC_5"), RailRole::Passive),
    rail_pin("12", Some("NC_6"), RailRole::Passive),
    rail_pin("13", Some("SEL"), RailRole::Select),
    rail_pin("14", Some("VISO"), RailRole::Output),
    rail_pin("15", Some("GNDS_2"), RailRole::IsolatedGround),
    rail_pin("16", Some("GNDS_3"), RailRole::ExtraGround),
];

/// An enable pin's thresholds, absolute against the ground pin: the lower
/// of the spec's two figures as `V_IL`, the higher as `V_IH`, hysteresis 0 —
/// none of the AP62301, NCP114, XL1509 and UCC12040 sheets names one apart
/// from the pair — and between them the comparator keeps the state it is
/// in ([`DeadBand::HoldLast`]).
fn enable_thresholds(spec: EnableSpec) -> Thresholds {
    Thresholds::new(
        spec.assert_volts.min(spec.deassert_volts),
        spec.assert_volts.max(spec.deassert_volts),
        0.0,
        DeadBand::HoldLast,
    )
}

/// Turn one pin-table row into a [`PinDecl`]: the input measured against
/// the ground pin and the output against the isolated ground where the
/// part has one, else the ground — the references the build's domain
/// lints read (`embsim_board::Finding::UnreferencedDomain`,
/// `embsim_board::Finding::RailDown`'s reason); the enable reading through
/// the configuration's own thresholds ([`EnableSpec`], each part's
/// datasheet figure), absolute against the ground pin, the lower of the
/// two its `V_IL` and the higher its `V_IH` (none of the four datasheets
/// names a hysteresis figure apart from the pair); the status output an
/// open drain.
fn declare(pin: &RailPin, table: &[RailPin], config: &Config) -> PinDecl {
    let of = |role: RailRole| table.iter().find(|p| p.role == role).map(|p| p.number);
    let ground = of(RailRole::Ground).expect("checked at new");
    let decl = match pin.role {
        RailRole::Input => PinDecl::power_in(pin.number).with_reference(ground),
        RailRole::Ground | RailRole::ExtraGround => PinDecl::power_in(pin.number),
        // Released until the part publishes: a rail that is down floats
        // (`NODES.md` §2), and the build snapshot says so.
        RailRole::Output => PinDecl::power_out(pin.number)
            .with_idle(None)
            .with_reference(match of(RailRole::IsolatedGround) {
                Some(isolated) => isolated,
                None => ground,
            }),
        RailRole::IsolatedGround => PinDecl::power_out(pin.number).with_idle(None),
        RailRole::Enable => match config.enable {
            Some(spec) => {
                PinDecl::digital_in(pin.number, enable_thresholds(spec)).with_reference(ground)
            }
            // A table with an enable pin behind a configuration that names
            // no enable: nothing the model does reads the pin, so it
            // declares nothing — no thresholds stand in for a projection no
            // one makes, and an open one is no floating input.
            None => PinDecl::passive(pin.number),
        },
        RailRole::Feedback | RailRole::Select | RailRole::Passive => PinDecl::passive(pin.number),
        RailRole::StatusOut => PinDecl::digital_out(pin.number).sink_only(),
    };
    match pin.name {
        Some(name) => decl.with_name(name),
        None => decl,
    }
}

// ============================================================
// Errors
// ============================================================

/// A rail table or configuration the model refuses.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RailConfigError {
    /// The table declares `count` pins of a role that needs exactly one.
    RoleCount {
        /// The role.
        role: &'static str,
        /// How many the table declares.
        count: usize,
    },
    /// The configuration sets the output from a pin the table lacks.
    MissingPin {
        /// The role the configuration reads.
        role: &'static str,
    },
}

impl fmt::Display for RailConfigError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            RailConfigError::RoleCount { role, count } => {
                write!(
                    f,
                    "a rail needs exactly one {role} pin; the table declares {count}"
                )
            }
            RailConfigError::MissingPin { role } => {
                write!(
                    f,
                    "the configuration reads a {role} pin the table does not declare"
                )
            }
        }
    }
}

impl std::error::Error for RailConfigError {}

// ============================================================
// Core
// ============================================================

/// What the rail is doing, as its monitor reports it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RailState {
    /// The output is released, and why: which of the three conditions the
    /// part waits for is unmet.
    Down {
        /// The input is at or above the part's threshold.
        input_up: bool,
        /// The enable asserts (or the part has none).
        enabled: bool,
        /// The output's reference pin reads a voltage.
        referenced: bool,
    },
    /// Every condition is met and the soft-start is running: the output
    /// steps up at `at_ns`.
    Rising {
        /// The armed instant.
        at_ns: u64,
    },
    /// The output holds `volts` (absolute) since `since_ns`.
    Up {
        /// The published voltage.
        volts: Volts,
        /// The instant it was published.
        since_ns: u64,
    },
    /// The part is disabled with its input up and holds the output at its
    /// reference through the active discharge.
    Discharging,
}

/// A pin nothing has been handed yet: no voltage, no clock.
const NOTHING: Sense = Sense {
    volts: None,
    periodic: None,
    at_ns: 0,
};

#[derive(Debug)]
struct State {
    /// The input pin's sense: against the ground pin.
    input: Sense,
    /// The ground pin's sense: in the engine's frame (it declares no
    /// reference) — the voltage the output is published above.
    ground: Sense,
    /// The isolated ground's sense, when the part declares one: in the
    /// engine's frame.
    isolated_ground: Option<Sense>,
    /// The enable pin's sense: against the ground pin.
    enable: Option<Sense>,
    /// The level the enable last read — its receiver's last level, which
    /// the next projection is chosen by (its hysteresis).
    enable_level: Option<Level>,
    /// The input's hysteresis latch.
    input_up: bool,
    /// The enable's hysteresis latch (`None`: not yet decided).
    enabled: Option<bool>,
    /// `v_set` as resolved at attach.
    v_set: Option<Volts>,
    armed_at: Option<u64>,
    /// The drive the output pin holds: `None` released.
    published: Option<TheveninDrive>,
    up_since_ns: Option<u64>,
    output: Option<PinHandle>,
    io: Option<ComponentNetIo>,
    publishes: u64,
}

#[derive(Debug)]
struct Core {
    config: Config,
    state: Mutex<State>,
}

impl Core {
    /// Fold a new input reading into the hysteresis latch. The input is
    /// handed its voltage against the ground pin; one that names no
    /// voltage — nothing reaches the input, or nothing holds the ground to
    /// take it against (no assumed 0 V) — is not up.
    fn latch_input(&self, state: &mut State) {
        state.input_up = match state.input.volts {
            Some(relative) if relative >= self.config.input_on_volts => true,
            Some(relative) if relative < self.config.input_off_volts => false,
            Some(_) => state.input_up,
            None => false,
        };
    }

    /// Fold a new enable reading into the hysteresis latch: the enable
    /// pin's projection through its declared thresholds — the lower of the
    /// spec's two figures as `V_IL`, the higher as `V_IH`, holding its last
    /// level between them ([`DeadBand::HoldLast`], the comparator's
    /// hysteresis) — read as asserting per [`EnableSpec::sense`].
    ///
    /// An enable in its band that has read nothing yet is not asserted. One
    /// handed no voltage while the ground names one has no source: the
    /// part's own open-pin answer ([`EnableSpec::floating_enables`]). A
    /// clock on EN is read like every receiver reads one
    /// ([`Sense::level`]): a wave whose phases settle to one level through
    /// the enable's thresholds is that level; one that toggles the input,
    /// or has a phase that reads none, names no single level and takes the
    /// open-pin answer too (the wildcard audit, `NODES.md` §12 item 5; the
    /// final pass). With no ground to measure against the enable is
    /// undecided (`None`, which evaluates as disabled) rather than read
    /// against an assumed 0 V.
    fn latch_enable(&self, state: &mut State) {
        let Some(spec) = self.config.enable else {
            state.enabled = Some(true);
            return;
        };
        let Some(pin) = state.enable else {
            // No enable pin on the facade: the pin is left open.
            state.enabled = Some(spec.floating_enables);
            return;
        };
        let asserting = |level: Level| match spec.sense {
            EnableSense::High => level == Level::High,
            EnableSense::Low => level == Level::Low,
        };
        state.enabled = match (pin.volts, pin.periodic) {
            (Some(_), _) => {
                state.enable_level = pin.level(&enable_thresholds(spec), state.enable_level);
                Some(state.enable_level.is_some_and(asserting))
            }
            (None, Some(_)) => {
                state.enable_level = pin.level(&enable_thresholds(spec), state.enable_level);
                Some(match state.enable_level {
                    Some(level) => asserting(level),
                    None => spec.floating_enables,
                })
            }
            (None, None) if state.ground.volts.is_some() => Some(spec.floating_enables),
            (None, None) => None,
        };
    }

    /// The output's reference voltage: the isolated ground where the part
    /// declares one, its own ground otherwise — in the engine's frame, the
    /// frame the output is published in.
    fn output_reference(state: &State) -> Option<Volts> {
        state.isolated_ground.unwrap_or(state.ground).volts
    }

    fn publish(&self, state: &mut State, drive: Option<TheveninDrive>, now_ns: u64) {
        if state.published == drive {
            return;
        }
        state.published = drive;
        state.publishes += 1;
        state.up_since_ns = match drive {
            Some(d) if d.impedance == self.config.z_out_ohms.unwrap_or(0.0) => Some(now_ns),
            _ => None,
        };
        if let Some(out) = &state.output {
            out.set_drive(drive);
        }
    }

    /// Re-decide the output from the latched conditions, at `now_ns`.
    fn evaluate(&self, state: &mut State, now_ns: u64) {
        let enabled = state.enabled.unwrap_or(false);
        let reference = Self::output_reference(state);
        let v_set = state.v_set;
        match (state.input_up && enabled, reference, v_set) {
            (true, Some(reference), Some(v_set)) => {
                let drive = TheveninDrive {
                    volts: reference + v_set,
                    impedance: self.config.z_out_ohms.unwrap_or(0.0),
                };
                if state.up_since_ns.is_some() {
                    // Up: follow a reference that moved, silently otherwise.
                    let since = state.up_since_ns;
                    self.publish(state, Some(drive), now_ns);
                    state.up_since_ns = since;
                } else if self.config.start_up_ns == 0 {
                    state.armed_at = None;
                    self.publish(state, Some(drive), now_ns);
                } else if state.armed_at.is_none() {
                    let at_ns = now_ns.saturating_add(self.config.start_up_ns);
                    state.armed_at = Some(at_ns);
                    if let Some(io) = &state.io {
                        io.schedule_at_ns(at_ns);
                    }
                }
            }
            _ => {
                state.armed_at = None;
                // Disabled with the input up: the active discharge, where
                // the part has one, holds the output at its reference.
                let discharge = match (self.config.discharge_ohms, reference) {
                    (Some(ohms), Some(reference)) if state.input_up && !enabled => {
                        Some(TheveninDrive {
                            volts: reference,
                            impedance: ohms,
                        })
                    }
                    _ => None,
                };
                self.publish(state, discharge, now_ns);
                if discharge.is_some() {
                    state.up_since_ns = None;
                }
            }
        }
    }

    fn on_wake(&self, state: &mut State, now_ns: u64) {
        let Some(at_ns) = state.armed_at else {
            return;
        };
        if now_ns < at_ns {
            return;
        }
        state.armed_at = None;
        let enabled = state.enabled.unwrap_or(false);
        if let (true, Some(reference), Some(v_set)) = (
            state.input_up && enabled,
            Self::output_reference(state),
            state.v_set,
        ) {
            let drive = TheveninDrive {
                volts: reference + v_set,
                impedance: self.config.z_out_ohms.unwrap_or(0.0),
            };
            self.publish(state, Some(drive), now_ns);
        }
    }

    fn rail_state(&self, state: &State) -> RailState {
        if let Some(published) = state.published {
            if published.impedance == self.config.z_out_ohms.unwrap_or(0.0) {
                return RailState::Up {
                    volts: published.volts,
                    since_ns: state.up_since_ns.unwrap_or(0),
                };
            }
            return RailState::Discharging;
        }
        if let Some(at_ns) = state.armed_at {
            return RailState::Rising { at_ns };
        }
        RailState::Down {
            input_up: state.input_up,
            enabled: state.enabled.unwrap_or(false),
            referenced: Self::output_reference(state).is_some(),
        }
    }
}

// ============================================================
// Monitor
// ============================================================

/// Cheap cloneable read handle onto a live [`Rail`].
#[derive(Clone, Debug)]
pub struct RailMonitor {
    core: Arc<Core>,
}

impl RailMonitor {
    /// What the rail is doing.
    pub fn state(&self) -> RailState {
        let state = self.core.state.lock().unwrap();
        self.core.rail_state(&state)
    }

    /// `v_set` as resolved at attach: the fixed voltage, the divider's, or
    /// the strap's. `None` before attach.
    pub fn v_set(&self) -> Option<Volts> {
        self.core.state.lock().unwrap().v_set
    }

    /// Drives published on the output since construction: one per state
    /// change, nothing else.
    pub fn publish_count(&self) -> u64 {
        self.core.state.lock().unwrap().publishes
    }

    /// The configuration in force.
    pub fn config(&self) -> &Config {
        &self.core.config
    }
}

// ============================================================
// Component
// ============================================================

/// A regulator as a live board-engine component.
///
/// ```rust
/// use embsim_board::Component;
/// use embsim_models::rail::{Config, Rail, NCP114_PINS_BY_FUNCTION, XL1509_PINS_SOP8};
///
/// // The P2-EC32MB's LDOs, straight off their netlist value.
/// let config = Config::ncp114_from_value("LDO 300mA, 3.3V").expect("a voltage");
/// let ldo = Rail::new(config, &NCP114_PINS_BY_FUNCTION).expect("a valid table");
/// assert_eq!(ldo.pins().len(), 5);
///
/// // The MaD Edge board's 5 V buck.
/// let buck = Rail::new(Config::xl1509_from_value("XL1509-5V").unwrap(), &XL1509_PINS_SOP8);
/// assert!(buck.is_ok());
/// ```
#[derive(Debug)]
pub struct Rail {
    pins: Vec<PinDecl>,
    table: &'static [RailPin],
    core: Arc<Core>,
}

impl Rail {
    /// A rail with `config` behind the pin table `table`.
    pub fn new(config: Config, table: &'static [RailPin]) -> Result<Self, RailConfigError> {
        let count = |role: RailRole| table.iter().filter(|p| p.role == role).count();
        for (role, name) in [
            (RailRole::Input, "input"),
            (RailRole::Ground, "ground"),
            (RailRole::Output, "output"),
        ] {
            if count(role) != 1 {
                return Err(RailConfigError::RoleCount {
                    role: name,
                    count: count(role),
                });
            }
        }
        for (role, name) in [
            (RailRole::IsolatedGround, "isolated ground"),
            (RailRole::Enable, "enable"),
            (RailRole::Feedback, "feedback"),
            (RailRole::Select, "select"),
        ] {
            if count(role) > 1 {
                return Err(RailConfigError::RoleCount {
                    role: name,
                    count: count(role),
                });
            }
        }
        match config.v_set {
            VSet::Divider { .. } if count(RailRole::Feedback) == 0 => {
                return Err(RailConfigError::MissingPin { role: "feedback" });
            }
            VSet::Select { .. } if count(RailRole::Select) == 0 => {
                return Err(RailConfigError::MissingPin { role: "select" });
            }
            _ => {}
        }
        let pin_of = |role: RailRole| table.iter().find(|p| p.role == role).map(|p| p.number);
        Ok(Self {
            pins: table
                .iter()
                .map(|pin| declare(pin, table, &config))
                .collect(),
            table,
            core: Arc::new(Core {
                config,
                state: Mutex::new(State {
                    input: NOTHING,
                    ground: NOTHING,
                    isolated_ground: pin_of(RailRole::IsolatedGround).map(|_| NOTHING),
                    enable: pin_of(RailRole::Enable).map(|_| NOTHING),
                    enable_level: None,
                    input_up: false,
                    enabled: None,
                    v_set: None,
                    armed_at: None,
                    published: None,
                    up_since_ns: None,
                    output: None,
                    io: None,
                    publishes: 0,
                }),
            }),
        })
    }

    /// A read handle onto this rail.
    pub fn monitor(&self) -> RailMonitor {
        RailMonitor {
            core: Arc::clone(&self.core),
        }
    }

    fn pin_number(&self, role: RailRole) -> Option<&'static str> {
        self.table.iter().find(|p| p.role == role).map(|p| p.number)
    }

    /// Resolve `v_set` from the topology at attach.
    fn resolve_v_set(&self, io: &ComponentNetIo) -> Result<Volts, AttachError> {
        let part = self.core.config.part;
        let failed = |message: String| AttachError::Failed { message };
        match self.core.config.v_set {
            VSet::Fixed(volts) => Ok(volts),
            VSet::Divider { v_fb } => {
                let feedback = self.pin_number(RailRole::Feedback).expect("checked at new");
                let ground = io.node(self.pin_number(RailRole::Ground).expect("checked"))?;
                let resistors = io.resistors_at(feedback)?;
                let (bottom, top): (Vec<_>, Vec<_>) =
                    resistors.iter().partition(|r| r.far == ground);
                match (bottom.as_slice(), top.as_slice()) {
                    ([bottom], [top]) => Ok(v_fb * (1.0 + top.ohms / bottom.ohms)),
                    _ => Err(failed(format!(
                        "{part}: the feedback pin {feedback:?} needs one resistor to the part's \
                         ground and one to the output; found {}",
                        resistors
                            .iter()
                            .map(|r| format!("{} {} Ω", r.reference, r.ohms))
                            .collect::<Vec<_>>()
                            .join(", ")
                    ))),
                }
            }
            VSet::Select {
                shorted_to_output,
                resistor_to_output,
                shorted_to_ground,
                resistor_to_ground,
                resistor_ohms,
            } => {
                let select = self.pin_number(RailRole::Select).expect("checked at new");
                let output = io.node(self.pin_number(RailRole::Output).expect("checked"))?;
                let ground = io.node(
                    self.pin_number(RailRole::IsolatedGround)
                        .unwrap_or_else(|| self.pin_number(RailRole::Ground).expect("checked")),
                )?;
                let node = io.node(select)?;
                if node == output {
                    return Ok(shorted_to_output);
                }
                if node == ground {
                    return Ok(shorted_to_ground);
                }
                let resistors = io.resistors_at(select)?;
                let strap = |to: NetId| {
                    resistors.iter().find(|r| {
                        r.far == to && (r.ohms - resistor_ohms).abs() <= resistor_ohms * 0.05
                    })
                };
                if strap(output).is_some() {
                    return Ok(resistor_to_output);
                }
                if strap(ground).is_some() {
                    return Ok(resistor_to_ground);
                }
                Err(failed(format!(
                    "{part}: the select pin {select:?} is neither shorted nor strapped through \
                     {resistor_ohms} Ω to the output or the ground (the datasheet supports no \
                     other setting)"
                )))
            }
        }
    }
}

impl Component for Rail {
    fn pins(&self) -> &[PinDecl] {
        &self.pins
    }

    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError> {
        let v_set = self.resolve_v_set(&io)?;
        let output = self.pin_number(RailRole::Output).expect("checked at new");
        {
            let mut state = self.core.state.lock().unwrap();
            state.v_set = Some(v_set);
            state.output = Some(io.pin(output)?);
            state.io = Some(io.clone());
        }

        let core = Arc::clone(&self.core);
        io.on_wake_ns(move |now_ns| {
            let mut state = core.state.lock().unwrap();
            core.on_wake(&mut state, now_ns);
        });

        // The references first, so a level delivered on the input or the
        // enable is read against a ground already known.
        let ground = self.pin_number(RailRole::Ground).expect("checked at new");
        let core = Arc::clone(&self.core);
        io.on_sense(ground, move |sensed| {
            let mut state = core.state.lock().unwrap();
            state.ground = sensed;
            core.latch_input(&mut state);
            core.latch_enable(&mut state);
            core.evaluate(&mut state, virtual_clock::virtual_ns());
        })?;
        if let Some(isolated) = self.pin_number(RailRole::IsolatedGround) {
            let core = Arc::clone(&self.core);
            io.on_sense(isolated, move |sensed| {
                let mut state = core.state.lock().unwrap();
                state.isolated_ground = Some(sensed);
                core.evaluate(&mut state, virtual_clock::virtual_ns());
            })?;
        }
        let input = self.pin_number(RailRole::Input).expect("checked at new");
        let core = Arc::clone(&self.core);
        io.on_sense(input, move |sensed| {
            let mut state = core.state.lock().unwrap();
            state.input = sensed;
            core.latch_input(&mut state);
            core.evaluate(&mut state, virtual_clock::virtual_ns());
        })?;
        match self.pin_number(RailRole::Enable) {
            Some(enable) => {
                let core = Arc::clone(&self.core);
                io.on_sense(enable, move |sensed| {
                    let mut state = core.state.lock().unwrap();
                    state.enable = Some(sensed);
                    core.latch_enable(&mut state);
                    core.evaluate(&mut state, virtual_clock::virtual_ns());
                })?;
            }
            None => {
                let mut state = self.core.state.lock().unwrap();
                self.core.latch_enable(&mut state);
                self.core.evaluate(&mut state, virtual_clock::virtual_ns());
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use embsim_board::{PinRole, SenseKind};
    use rstest::rstest;

    use super::*;

    /// A sense handed `volts` (the instant plays no part in the rail).
    fn handed(volts: Option<Volts>) -> Sense {
        Sense {
            volts,
            periodic: None,
            at_ns: 0,
        }
    }

    #[rstest]
    #[case::module_ldo("LDO 300mA, 3.3V", Some(3.3))]
    #[case::bare("1.8V", Some(1.8))]
    #[case::no_voltage("LDO 300mA", None)]
    #[case::two_points("1.2.3V", None)]
    fn the_ldo_value_names_its_output(#[case] value: &str, #[case] volts: Option<Volts>) {
        assert_eq!(
            Config::ncp114_from_value(value).map(|c| c.v_set),
            volts.map(VSet::Fixed)
        );
    }

    #[rstest]
    #[case::five("XL1509-5V", Some(5.0))]
    #[case::three_three("XL1509-3.3V", Some(3.3))]
    #[case::twelve("XL1509-12V", Some(12.0))]
    #[case::ordering_code("XL1509-5.0E1", Some(5.0))]
    #[case::adjustable("XL1509-ADJ", None)]
    #[case::other_part("XL1583-5V", None)]
    fn the_buck_value_names_its_version(#[case] value: &str, #[case] volts: Option<Volts>) {
        assert_eq!(
            Config::xl1509_from_value(value).map(|c| c.v_set),
            volts.map(VSet::Fixed)
        );
    }

    #[rstest]
    fn the_tables_carry_exactly_the_roles_a_rail_needs() {
        assert!(Rail::new(Config::ap62301(), &AP62301_PINS_BY_FUNCTION).is_ok());
        assert!(Rail::new(Config::ap62301(), &AP62301_PINS_SOT563).is_ok());
        assert!(Rail::new(Config::ncp114(3.3), &NCP114_PINS_BY_FUNCTION).is_ok());
        assert!(Rail::new(Config::ncp114(3.3), &NCP114_PINS_UDFN4).is_ok());
        assert!(Rail::new(Config::xl1509(5.0), &XL1509_PINS_SOP8).is_ok());
        assert!(Rail::new(Config::ucc12040(), &UCC12040_PINS_SOIC16).is_ok());
        // A divider needs a feedback pin; a strap a select pin.
        assert_eq!(
            Rail::new(Config::ap62301(), &NCP114_PINS_UDFN4).err(),
            Some(RailConfigError::MissingPin { role: "feedback" })
        );
        assert_eq!(
            Rail::new(Config::ucc12040(), &XL1509_PINS_SOP8).err(),
            Some(RailConfigError::MissingPin { role: "select" })
        );
    }

    #[rstest]
    fn the_output_is_a_released_terminal_and_the_references_name_the_grounds() {
        let rail = Rail::new(Config::ucc12040(), &UCC12040_PINS_SOIC16).unwrap();
        let pin = |n: &str| rail.pins().iter().find(|p| p.number == n).copied().unwrap();
        assert_eq!(pin("14").role, PinRole::PowerOut);
        assert_eq!(pin("14").idle, None);
        assert_eq!(pin("15").role, PinRole::PowerOut);
        assert_eq!(pin("15").idle, None);
        assert_eq!(pin("9").role, PinRole::PowerIn);
        assert_eq!(pin("1").senses_at_build(), Some(SenseKind::Digital));
        assert_eq!(
            pin("1").thresholds,
            Some(Thresholds::new(
                UCC12040_EN_LOW_VOLTS,
                UCC12040_EN_HIGH_VOLTS,
                0.0,
                DeadBand::HoldLast,
            )),
            "EN reads through the datasheet's V_IF/V_IR"
        );
        assert_eq!(pin("13").role, PinRole::Passive);
        let references: Vec<(&str, &str)> = rail
            .pins()
            .iter()
            .filter(|p| matches!(p.role, PinRole::PowerIn | PinRole::PowerOut))
            .filter_map(|p| Some((p.number, p.reference?)))
            .collect();
        assert_eq!(references, [("3", "2"), ("14", "15")]);
    }

    /// The core without an engine: the module's LDO (no soft-start) comes
    /// up the instant its input and enable read above their thresholds,
    /// drops when the enable falls below its low threshold — through the
    /// discharge, since the part has one — and holds inside the band.
    #[rstest]
    fn an_ldo_follows_its_input_and_enable_with_hysteresis() {
        let rail = Rail::new(Config::ncp114(3.3), &NCP114_PINS_BY_FUNCTION).unwrap();
        let core = Arc::clone(&rail.core);
        let mut state = core.state.lock().unwrap();
        state.v_set = Some(3.3);
        state.ground = handed(Some(0.0));
        state.input = handed(Some(3.649));
        core.latch_input(&mut state);
        state.enable = Some(handed(Some(3.649)));
        core.latch_enable(&mut state);
        core.evaluate(&mut state, 7);
        assert_eq!(
            core.rail_state(&state),
            RailState::Up {
                volts: 3.3,
                since_ns: 7
            }
        );
        assert_eq!(state.publishes, 1);

        // Inside the enable band: held.
        state.enable = Some(handed(Some(0.6)));
        core.latch_enable(&mut state);
        core.evaluate(&mut state, 8);
        assert_eq!(state.publishes, 1, "0.6 V is between 0.4 V and 0.9 V");

        // Below it: the discharge holds the output at ground through 100 Ω.
        state.enable = Some(handed(Some(0.3)));
        core.latch_enable(&mut state);
        core.evaluate(&mut state, 9);
        assert_eq!(core.rail_state(&state), RailState::Discharging);
        assert_eq!(
            state.published,
            Some(TheveninDrive {
                volts: 0.0,
                impedance: NCP114_DISCHARGE_OHMS
            })
        );

        // The input gone: released, no discharge without a supply.
        state.input = handed(None);
        core.latch_input(&mut state);
        core.evaluate(&mut state, 10);
        assert_eq!(state.published, None);
        assert_eq!(
            core.rail_state(&state),
            RailState::Down {
                input_up: false,
                enabled: false,
                referenced: true
            }
        );
    }

    /// A ground pin no source reaches decides nothing: the input and the
    /// enable, measured against no reference, are handed no voltage — the
    /// input is not up and the enable undecided, a driven enable included,
    /// and the monitor says so; the ground arriving decides both from the
    /// voltages they are then handed.
    #[rstest]
    fn an_unreferenced_ground_reads_the_input_as_not_up_and_the_enable_as_undecided() {
        let rail = Rail::new(Config::ncp114(3.3), &NCP114_PINS_BY_FUNCTION).unwrap();
        let core = Arc::clone(&rail.core);
        let mut state = core.state.lock().unwrap();
        state.v_set = Some(3.3);
        state.ground = handed(None);
        state.input = handed(None);
        core.latch_input(&mut state);
        state.enable = Some(handed(None));
        core.latch_enable(&mut state);
        core.evaluate(&mut state, 5);
        assert!(!state.input_up, "a voltage relative to nothing is not up");
        assert_eq!(
            state.enabled, None,
            "a voltage relative to nothing decides no enable"
        );
        assert_eq!(state.published, None);
        assert_eq!(
            core.rail_state(&state),
            RailState::Down {
                input_up: false,
                enabled: false,
                referenced: false
            }
        );

        // The ground arrives: the readings against it now decide.
        state.ground = handed(Some(0.0));
        state.input = handed(Some(3.649));
        state.enable = Some(handed(Some(3.649)));
        core.latch_input(&mut state);
        core.latch_enable(&mut state);
        core.evaluate(&mut state, 6);
        assert!(state.input_up);
        assert_eq!(state.enabled, Some(true));
        assert_eq!(
            core.rail_state(&state),
            RailState::Up {
                volts: 3.3,
                since_ns: 6
            }
        );
    }

    /// A buck with a soft-start arms its wake at the instant the input
    /// crosses the rising threshold and publishes at exactly that instant
    /// plus `t_SS`; a wake before it publishes nothing, and an input that
    /// drops below the falling threshold in between cancels it.
    #[rstest]
    fn a_soft_start_publishes_at_the_armed_instant_and_a_drop_cancels_it() {
        let rail = Rail::new(Config::ap62301(), &AP62301_PINS_BY_FUNCTION).unwrap();
        let core = Arc::clone(&rail.core);
        let mut state = core.state.lock().unwrap();
        state.v_set = Some(1.813);
        state.ground = handed(Some(0.0));
        core.latch_enable(&mut state);
        assert_eq!(state.enabled, Some(true), "EN left floating enables");

        state.input = handed(Some(3.8));
        core.latch_input(&mut state);
        core.evaluate(&mut state, 1_000);
        assert!(!state.input_up, "3.8 V is under the 3.90 V POR");

        state.input = handed(Some(5.0));
        core.latch_input(&mut state);
        core.evaluate(&mut state, 1_000);
        assert_eq!(
            core.rail_state(&state),
            RailState::Rising {
                at_ns: 1_000 + AP62301_SOFT_START_NS
            }
        );
        core.on_wake(&mut state, 1_000 + AP62301_SOFT_START_NS - 1);
        assert_eq!(state.published, None, "nothing before the instant");
        core.on_wake(&mut state, 1_000 + AP62301_SOFT_START_NS);
        assert_eq!(
            core.rail_state(&state),
            RailState::Up {
                volts: 1.813,
                since_ns: 1_000 + AP62301_SOFT_START_NS
            }
        );
        let published = state.published.unwrap();
        assert!((published.impedance - AP62301_Z_OUT_OHMS).abs() < 1e-12);

        // 3.7 V: under the POR, over the UVLO — still up (hysteresis).
        state.input = handed(Some(3.7));
        core.latch_input(&mut state);
        core.evaluate(&mut state, 5_000_000);
        assert!(state.input_up);
        assert_eq!(state.publishes, 1);

        // Under the UVLO: released.
        state.input = handed(Some(3.5));
        core.latch_input(&mut state);
        core.evaluate(&mut state, 5_000_001);
        assert_eq!(state.published, None);

        // Up again, then down during the soft-start: the wake fires into a
        // part whose input is down and publishes nothing.
        state.input = handed(Some(5.0));
        core.latch_input(&mut state);
        core.evaluate(&mut state, 6_000_000);
        let RailState::Rising { at_ns } = core.rail_state(&state) else {
            panic!("rising");
        };
        state.input = handed(None);
        core.latch_input(&mut state);
        core.evaluate(&mut state, 6_000_001);
        core.on_wake(&mut state, at_ns);
        assert_eq!(state.published, None);
    }

    /// An isolated rail publishes against its isolated ground: with that
    /// ground unheld it stays down, referenced or not by the primary
    /// ground; held at 0 V it comes up at 5 V; held at 2 V it re-publishes
    /// 7 V (absolute), silently keeping its up instant.
    #[rstest]
    fn an_isolated_rail_is_referenced_to_its_isolated_ground() {
        let rail = Rail::new(Config::ucc12040(), &UCC12040_PINS_SOIC16).unwrap();
        let core = Arc::clone(&rail.core);
        let mut state = core.state.lock().unwrap();
        state.v_set = Some(UCC12040_VISO_SEL_TO_VISO_VOLTS);
        state.ground = handed(Some(0.0));
        state.input = handed(Some(5.0));
        core.latch_input(&mut state);
        state.enable = Some(handed(Some(5.0)));
        core.latch_enable(&mut state);
        core.evaluate(&mut state, 0);
        assert_eq!(
            core.rail_state(&state),
            RailState::Down {
                input_up: true,
                enabled: true,
                referenced: false
            }
        );
        state.isolated_ground = Some(handed(Some(0.0)));
        core.evaluate(&mut state, 0);
        assert_eq!(
            core.rail_state(&state),
            RailState::Rising {
                at_ns: UCC12040_RISE_NS
            }
        );
        core.on_wake(&mut state, UCC12040_RISE_NS);
        assert_eq!(
            core.rail_state(&state),
            RailState::Up {
                volts: 5.0,
                since_ns: UCC12040_RISE_NS
            }
        );
        state.isolated_ground = Some(handed(Some(2.0)));
        core.evaluate(&mut state, 900_000);
        assert_eq!(
            core.rail_state(&state),
            RailState::Up {
                volts: 7.0,
                since_ns: UCC12040_RISE_NS
            }
        );
        assert_eq!(state.publishes, 2);
    }

    /// A clock on EN is read through the enable's thresholds like any
    /// receiver reads one: a wave whose phases settle to one level is that
    /// level — above the NCP114's 0.9 V `V_EN_HI` in both phases is on,
    /// under the 0.4 V `V_EN_LO` (and the AP62301's 1.10 V `V_EN_L`) off —
    /// and one that toggles the input names no single level and takes the
    /// part's open-pin answer: off for the NCP114, on for the AP62301.
    #[rstest]
    #[case::ncp114_steady_high(Config::ncp114(3.3), 3.3, 1.5, true)]
    #[case::ncp114_steady_low(Config::ncp114(3.3), 0.3, 0.0, false)]
    #[case::ncp114_toggling(Config::ncp114(3.3), 3.3, 0.0, false)]
    #[case::ap62301_steady_low(Config::ap62301(), 0.3, 0.0, false)]
    #[case::ap62301_toggling(Config::ap62301(), 3.3, 0.0, true)]
    fn a_clock_on_the_enable_is_the_level_its_phases_settle_to(
        #[case] config: Config,
        #[case] hi: Volts,
        #[case] lo: Volts,
        #[case] enabled: bool,
    ) {
        let table: &'static [RailPin] = if config.part == Config::ap62301().part {
            &AP62301_PINS_BY_FUNCTION
        } else {
            &NCP114_PINS_BY_FUNCTION
        };
        let rail = Rail::new(config, table).unwrap();
        let core = Arc::clone(&rail.core);
        let mut state = core.state.lock().unwrap();
        state.ground = handed(Some(0.0));
        state.enable = Some(Sense {
            volts: None,
            periodic: Some(embsim_board::PeriodicSense {
                hi: Some(hi),
                lo: Some(lo),
                segment: embsim_board::PeriodicSchedule {
                    emitted: 0,
                    freq_hz: 1_000,
                    total: None,
                    since_ns: 0,
                },
            }),
            at_ns: 0,
        });
        core.latch_enable(&mut state);
        assert_eq!(state.enabled, Some(enabled));
    }

    /// A table with an enable pin behind a configuration that names no
    /// enable declares the pin passive: nothing reads it, so it carries no
    /// thresholds and is no input that could float.
    #[rstest]
    fn an_enable_pin_no_configuration_reads_is_passive() {
        let config = Config {
            enable: None,
            ..Config::ncp114(3.3)
        };
        let rail = Rail::new(config, &NCP114_PINS_BY_FUNCTION).unwrap();
        let enable = NCP114_PINS_BY_FUNCTION
            .iter()
            .find(|p| p.role == RailRole::Enable)
            .expect("the table has an enable pin");
        let pin = rail
            .pins()
            .iter()
            .find(|p| p.number == enable.number)
            .copied()
            .unwrap();
        assert_eq!(pin.role, PinRole::Passive);
        assert_eq!(pin.thresholds, None);
        assert_eq!(pin.senses_at_build(), None);
    }

    /// The XL1509's enable is active-low and floats to on; its output is
    /// an ideal source (no load regulation figure).
    #[rstest]
    fn an_active_low_enable_reads_low_as_on_and_open_as_on() {
        let rail = Rail::new(Config::xl1509(5.0), &XL1509_PINS_SOP8).unwrap();
        let core = Arc::clone(&rail.core);
        let mut state = core.state.lock().unwrap();
        state.v_set = Some(5.0);
        state.ground = handed(Some(0.0));
        state.input = handed(Some(12.0));
        core.latch_input(&mut state);
        state.enable = Some(handed(None));
        core.latch_enable(&mut state);
        core.evaluate(&mut state, 3);
        assert_eq!(
            core.rail_state(&state),
            RailState::Up {
                volts: 5.0,
                since_ns: 3
            }
        );
        assert_eq!(state.published.unwrap().impedance, 0.0);
        state.enable = Some(handed(Some(2.0)));
        core.latch_enable(&mut state);
        core.evaluate(&mut state, 4);
        assert_eq!(state.published, None, "2 V is over the 1.4 V OFF threshold");
        state.enable = Some(handed(Some(0.0)));
        core.latch_enable(&mut state);
        core.evaluate(&mut state, 5);
        assert!(matches!(core.rail_state(&state), RailState::Up { .. }));
    }
}
