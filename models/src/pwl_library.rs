//! The library of **piecewise-linear elements registered by specification**
//! — the parts on the reference boards that have no Rust model and need
//! none: a part is a [`PwlSpec`] (its pins and the branches between them),
//! and the engine stamps the branches and chooses their regions
//! (`NODES.md` §2, the Diode / LED, FET / BJT and CCR rows; §8 phase 3).
//! Diodes and LEDs are one branch each; a FET is its channel and its body
//! diode; a transistor is its base–emitter diode and its collector; a
//! constant-current regulator is one two-region branch.
//!
//! # Keys
//!
//! Every entry is keyed on the netlist's **manufacturer part number** (the
//! `Manufacturer_Part_Number` / `MPN` field a KiCad export carries,
//! [`embsim_board::ComponentDecl::mpn`]) and, where the part name or the
//! value field is the part's own name, on that too — the registry looks a
//! part up by part name, then by manufacturer part number, then by value
//! ([`PartRegistry`]). A part whose number, name and value match no entry
//! resolves to nothing: registering the library gives it no class, and an
//! active part without one is `RegistryError::UnknownPart` (`DESIGN.md`
//! rule 1). A colour (`White`) or a function (`NPN`, `P Mosfet 30V 8A`) is
//! never a key: it names no purchasable part.
//!
//! # Provenance
//!
//! Every number cites its datasheet, table and condition beside its
//! constant. Where a datasheet gives a range, the entry takes the **bound
//! that guarantees the region it names** — a diode's maximum forward drop
//! (the on-segment is at most this far up), a channel's maximum
//! on-resistance, the threshold magnitude every part of the type has
//! turned on beyond, a transistor's minimum gain — and records the
//! typical beside it where the datasheet tabulates one, so a consumer can
//! judge a modelled figure against the spread. Every on-segment is
//! **vertical** (`r_d` = 0): each datasheet tabulates one forward-voltage
//! point, and a slope needs two, so the drop is the tabulated one at any
//! current — an upper bound below the test current (a diode drops less at
//! less current, which the datasheets' typical curves show and no constant
//! here reads off). A generic "red LED" or "small NPN" entry with no
//! datasheet behind it is deliberately absent.
//!
//! # Pins
//!
//! Pin ids are the ids the reference netlists carry for the part: the
//! KiCad `Device:D*` / `Device:LED` symbols' `1` = K, `2` = A; a
//! transistor symbol's package order; a multi-unit FET symbol's placed
//! unit; the transcribed module's functional ids. An entry is therefore
//! tied to the symbol the board was drawn with, as every facade is
//! (`BOARD_ENGINE.md`, the facade check) — a board drawn with another
//! symbol registers its own entry under the same number.
//!
//! # Lit
//!
//! An LED is **lit** when its branch carries at least the entry's
//! [`LibraryEntry::lit_amps`] from anode to cathode ([`is_lit`]): the
//! smallest current the datasheet distinguishes from a dark part, cited
//! per entry. The engine reports the current
//! (`BuiltSystem::branch_current("Board.D3")`); the threshold is the
//! part's.

use embsim_board::{Amps, Ohms, PartRegistry, PwlCurve, PwlSpec, RegionTest, Volts};

// ============================================================
// Vishay SS36 (SS32 thru SS36), Schottky barrier rectifier
// ============================================================

/// The SS36's forward drop: `V_F` maximum instantaneous forward voltage
/// **0.75 V** at `I_F` = 3.0 A, `T_A` = 25 °C, pulse test (Vishay General
/// Semiconductor, SS32/SS33/SS34/SS35/SS36, Document Number 88751, Revision
/// 04-Aug-15, "Electrical Characteristics", SS35–SS36 column). The
/// datasheet's one tabulated point; see the module docs for what a vertical
/// on-segment at it means.
pub const SS36_VF_VOLTS: Volts = 0.75;

/// The SS36's on-segment slope: none tabulated (Document 88751 gives one
/// `V_F` point and a typical curve), so the segment is vertical.
pub const SS36_R_D_OHMS: Ohms = 0.0;

/// The SS36's maximum average forward rectified current, `I_F(AV)` =
/// 3.0 A (Document 88751, "Maximum Ratings") — the rating the `V_F` point
/// is taken at, recorded so a consumer can judge a modelled current
/// against it.
pub const SS36_IF_AV_AMPS: f64 = 3.0;

/// The keys the SS36 entry answers to: the orderable number the EdgeBoard's
/// export carries, and the family name its value field spells.
pub const SS36_KEYS: &[&str] = &["SS36-E3/57T", "SS36"];

/// The SS36 as a two-pin element: anode `2` to cathode `1`.
pub fn ss36() -> PwlSpec {
    PwlSpec::diode("2", "1", SS36_VF_VOLTS, SS36_R_D_OHMS)
}

// ============================================================
// Lite-On LTST-C190KGKT, AlInGaP green 0603 LED
// ============================================================

/// The LTST-C190KGKT's forward drop: `V_F` **2.0 V typical** at `I_F` =
/// 20 mA, `T_a` = 25 °C (Lite-On Technology, LTST-C190KGKT, document
/// BNS-OD-C131/A4, "Electrical Optical Characteristics At Ta=25 °C",
/// p. 3 of 12). The typical is the entry's drop; the maximum is beside it.
pub const LTST_C190KGKT_VF_VOLTS: Volts = 2.0;

/// The LTST-C190KGKT's `V_F` maximum, **2.4 V** at `I_F` = 20 mA (the same
/// table) — the bin-code ceiling, recorded so a consumer can bound the
/// drop.
pub const LTST_C190KGKT_VF_MAX_VOLTS: Volts = 2.4;

/// The LTST-C190KGKT's on-segment slope: none tabulated (the table gives
/// one `V_F` point; Fig. 2 is a curve), so the segment is vertical.
pub const LTST_C190KGKT_R_D_OHMS: Ohms = 0.0;

/// The LTST-C190KGKT's continuous forward current rating, **30 mA**
/// (BNS-OD-C131/A4, "Absolute Maximum Ratings At Ta=25 °C", p. 1 of 12).
pub const LTST_C190KGKT_IF_MAX_AMPS: f64 = 30e-3;

/// The current the LTST-C190KGKT's luminous intensity is specified at,
/// `I_F` = **20 mA** (the "Electrical Optical Characteristics" test
/// condition): the current at which the datasheet rates the part's
/// brightness (18–71 mcd).
pub const LTST_C190KGKT_IF_TEST_AMPS: f64 = 20e-3;

/// The current above which the LTST-C190KGKT is **lit**: the datasheet's
/// one bound on a dark part's current, reverse current `I_R` **10 µA**
/// maximum at `V_R` = 5 V (BNS-OD-C131/A4, "Electrical Optical
/// Characteristics", p. 3 of 12, Note 5). A branch carrying more than the
/// part leaks is a forward junction emitting, at an intensity the
/// datasheet rates only at [`LTST_C190KGKT_IF_TEST_AMPS`]; the datasheet
/// tabulates no lower emission point, so this is the smallest current it
/// distinguishes from dark.
pub const LTST_C190KGKT_I_ON_AMPS: Amps = 10e-6;

/// The key the LTST-C190KGKT entry answers to: the EdgeBoard's LEDs carry
/// it as their manufacturer part number, their value being the bare `LED`.
pub const LTST_C190KGKT_KEYS: &[&str] = &["LTST-C190KGKT"];

/// The LTST-C190KGKT as a two-pin element: anode `2` to cathode `1`.
pub fn ltst_c190kgkt() -> PwlSpec {
    PwlSpec::diode("2", "1", LTST_C190KGKT_VF_VOLTS, LTST_C190KGKT_R_D_OHMS)
}

// ============================================================
// Inolux IN-S63AS5UW, InGaN white 0603 side-view LED
// ============================================================

/// The IN-S63AS5UW's forward drop: `V_F` **2.9 V typical** at `I_F` =
/// 5 mA (Inolux Corporation, IN-S63AS series, datasheet version 1.0,
/// 03-16-2017, "Ordering Information", the IN-S63AS5UW row). The
/// "Electrical Characteristics T_A = 25 °C" table bounds it at 2.7 V
/// minimum and 3.1 V maximum at the same 5 mA ([`IN_S63AS5UW_VF_MIN_VOLTS`],
/// [`IN_S63AS5UW_VF_MAX_VOLTS`]).
pub const IN_S63AS5UW_VF_VOLTS: Volts = 2.9;

/// The IN-S63AS5UW's `V_F` minimum, **2.7 V** at `I_F` = 5 mA (IN-S63AS
/// series V1.0, "Electrical Characteristics").
pub const IN_S63AS5UW_VF_MIN_VOLTS: Volts = 2.7;

/// The IN-S63AS5UW's `V_F` maximum, **3.1 V** at `I_F` = 5 mA (IN-S63AS
/// series V1.0, "Electrical Characteristics").
pub const IN_S63AS5UW_VF_MAX_VOLTS: Volts = 3.1;

/// The IN-S63AS5UW's on-segment slope: none tabulated (one `V_F` point at
/// 5 mA), so the segment is vertical.
pub const IN_S63AS5UW_R_D_OHMS: Ohms = 0.0;

/// The IN-S63AS5UW's continuous forward current rating, **25 mA**
/// (IN-S63AS series V1.0, "Absolute Maximum Rating at 25 °C").
pub const IN_S63AS5UW_IF_MAX_AMPS: f64 = 25e-3;

/// The current the IN-S63AS5UW's luminous intensity is specified at,
/// `I_F` = **5 mA** (IN-S63AS series V1.0, "Electrical Characteristics",
/// 285 mcd typical).
pub const IN_S63AS5UW_IF_TEST_AMPS: f64 = 5e-3;

/// The current above which the IN-S63AS5UW is **lit**: the datasheet
/// tabulates no dark-current bound (its reverse voltage rating carries no
/// current), so the smallest current it distinguishes from dark is the
/// one it rates the part's brightness at, [`IN_S63AS5UW_IF_TEST_AMPS`].
/// Stated consequence: this entry calls the part lit only at its rated
/// brightness, where the green entry calls a forward junction lit.
pub const IN_S63AS5UW_I_ON_AMPS: Amps = IN_S63AS5UW_IF_TEST_AMPS;

/// The key the IN-S63AS5UW entry answers to: the P2-EC32MB's `D601`/`D602`
/// carry it as their `MPN` field, their value being the colour `White`.
pub const IN_S63AS5UW_KEYS: &[&str] = &["IN-S63AS5UW"];

/// The IN-S63AS5UW as a two-pin element: anode `A` to cathode `K`, the
/// functional ids the transcribed module netlist gives its LEDs.
pub fn in_s63as5uw() -> PwlSpec {
    PwlSpec::diode("A", "K", IN_S63AS5UW_VF_VOLTS, IN_S63AS5UW_R_D_OHMS)
}

// ============================================================
// Anpec APM4953, dual P-channel MOSFET (the EdgeBoard's polarity FET)
// ============================================================

/// The APM4953's gate threshold magnitude, `V_GS(th)` **−2 V maximum**
/// (−1 V minimum, −1.5 V typical) at `V_GS` = `V_DS`, `I_DS` = −250 µA
/// (ANPEC Electronics, APM4953 Dual P-Channel Enhancement Mode MOSFET,
/// "Electrical Characteristics (T_A = 25 °C)", Static). The channel is on
/// where `V_GS` ≤ −2 V: the bound beyond which every part of the type has
/// turned on. A gate between −1 V and −2 V is inside the datasheet's
/// spread and reads off here.
pub const APM4953_VGS_TH_VOLTS: Volts = 2.0;

/// The APM4953's on-resistance, `R_DS(on)` **60 mΩ maximum** (53 mΩ
/// typical) at `V_GS` = −10 V, `I_DS` = −4.9 A (the same table) — the row
/// for the gate drive the EdgeBoard gives it, `V_GS` = −12 V. (At −4.5 V the
/// table gives 95 mΩ maximum.)
pub const APM4953_R_DS_ON_OHMS: Ohms = 60e-3;

/// The APM4953's body diode drop, `V_SD` **1.3 V maximum** (0.7 V typical)
/// at `I_SD` = −1.7 A, `V_GS` = 0 V (the same table, note b: guaranteed by
/// design).
pub const APM4953_VSD_VOLTS: Volts = 1.3;

/// The keys the APM4953 entry answers to: the EdgeBoard's manufacturer
/// part number and its symbol's part name / value.
pub const APM4953_KEYS: &[&str] = &["APM4953KC-TRG", "APM4953"];

/// The APM4953 as the EdgeBoard places it: **one half**, pins `1` (S1),
/// `2` (G1), `7` and `8` (D1, two drain fingers on one net) — the pins the
/// export carries for the placed unit of the dual symbol (the second half
/// is not placed and has no nodes). The channel is drain `7` to source
/// `1`, on while `V(G1) − V(S1)` ≤ −[`APM4953_VGS_TH_VOLTS`]; the body
/// diode is drain `7` to source `1` (a P-channel's body diode conducts from
/// drain to source), declared first so the start-up reads in its order:
/// the diode lifts the source, the gate falls below threshold, the channel
/// shorts the diode off. The second drain finger `8` is the same drain;
/// it declares no branch of its own, and on this board shares `7`'s net.
pub fn apm4953_half() -> PwlSpec {
    PwlSpec::new(["1", "2", "7", "8"])
        .with_branch(
            "7",
            "1",
            PwlCurve::Diode {
                vf: APM4953_VSD_VOLTS,
                r_d: 0.0,
            },
        )
        .with_controlled_branch(
            "7",
            "1",
            PwlCurve::Channel {
                r_on: APM4953_R_DS_ON_OHMS,
            },
            "2",
            RegionTest::AtMost(-APM4953_VGS_TH_VOLTS),
        )
}

// ============================================================
// Vishay Si3417DV, P-channel MOSFET (the P2-EC32MB's polarity FET)
// ============================================================

/// The Si3417DV's gate threshold magnitude, `V_GS(th)` **−3 V maximum**
/// (−1 V minimum) at `V_DS` = `V_GS`, `I_D` = −250 µA (Vishay Siliconix,
/// Si3417DV, Document Number 62890, S13-1815-Rev. A, 12-Aug-13,
/// "Specifications (T_J = 25 °C)", Static). The channel is on where
/// `V_GS` ≤ −3 V, the bound every part has turned on beyond.
pub const SI3417DV_VGS_TH_VOLTS: Volts = 3.0;

/// The Si3417DV's on-resistance, `R_DS(on)` **0.0360 Ω maximum** (0.0300 Ω
/// typical) at `V_GS` = −4.5 V, `I_D` = −6.1 A (Document 62890, the same
/// table) — the row for the gate drive the module gives it, `V_GS` = −5 V
/// from the 5 V edge fingers. (At −10 V the table gives 0.0252 Ω maximum.)
pub const SI3417DV_R_DS_ON_OHMS: Ohms = 36e-3;

/// The Si3417DV's body diode drop, `V_SD` **1.2 V maximum** (0.75 V
/// typical) at `I_S` = −5.8 A, `V_GS` = 0 V (Document 62890, "Drain-Source
/// Body Diode Characteristics").
pub const SI3417DV_VSD_VOLTS: Volts = 1.2;

/// The keys the Si3417DV entry answers to: the module's `MPN` field and
/// the part family.
pub const SI3417DV_KEYS: &[&str] = &["SI3417DV-T1-GE3", "SI3417DV"];

/// The Si3417DV as the transcribed module netlist names it: pins `D`, `G`,
/// `S`. Body diode drain to source first, then the channel drain to source
/// on while `V(G) − V(S)` ≤ −[`SI3417DV_VGS_TH_VOLTS`].
pub fn si3417dv() -> PwlSpec {
    PwlSpec::new(["D", "G", "S"])
        .with_branch(
            "D",
            "S",
            PwlCurve::Diode {
                vf: SI3417DV_VSD_VOLTS,
                r_d: 0.0,
            },
        )
        .with_controlled_branch(
            "D",
            "S",
            PwlCurve::Channel {
                r_on: SI3417DV_R_DS_ON_OHMS,
            },
            "G",
            RegionTest::AtMost(-SI3417DV_VGS_TH_VOLTS),
        )
}

// ============================================================
// onsemi NSI50010Y, constant-current regulator (the end-switch loops)
// ============================================================

/// The NSI50010Y's regulation current, `I_reg(SS)` **10 mA typical**
/// (7.0 mA minimum, 13 mA maximum) at `V_ak` = 7.5 V (onsemi NSI50010Y/D,
/// Rev. 3, "Electrical Characteristics"). The typical: the ±30 % spread
/// is the part's, and the loop it regulates reads the nominal.
pub const NSI50010_I_REG_AMPS: Amps = 10e-3;

/// The NSI50010Y's knee: `V_overhead` **1.8 V typical** (NSI50010Y/D
/// Rev. 3, "Electrical Characteristics", note 2: "typical value for 80 %
/// I_reg(SS)"). Below it the branch is the ohmic segment from the origin
/// to the knee, `V_overhead / I_reg(SS)` = 180 Ω; at or above it the
/// branch carries [`NSI50010_I_REG_AMPS`] (`PwlCurve::Regulator`). Stated
/// consequence of two regions: at the knee the model carries 100 % where
/// the part carries 80 %, and at 0.5 V it carries 2.8 mA where the
/// datasheet's "40 % of regulation with only 0.5 V V_ak" (Features) is
/// 4 mA — the curve is between the two segments, not on them.
pub const NSI50010_V_REG_VOLTS: Volts = 1.8;

/// The NSI50010Y's reverse voltage rating, `V_R` **500 mV** (NSI50010Y/D
/// Rev. 3, "Maximum Ratings"): reverse bias conducts through the ohmic
/// segment in this model, a stated simplification of a part rated for a
/// few hundred millivolts of it.
pub const NSI50010_VR_MAX_VOLTS: Volts = 0.5;

/// The key the NSI50010Y entry answers to: the EdgeBoard's eight
/// regulators carry `NSI50010YT1G` as both their manufacturer part number
/// and their value (their symbol's part name is `NSI50010YT1G_1`).
pub const NSI50010_KEYS: &[&str] = &["NSI50010YT1G"];

/// The NSI50010Y as a two-pin element: anode `2` to cathode `1` (SOD-123,
/// NSI50010Y/D Rev. 3 package drawing, CASE 425 STYLE 1: pin 1 cathode,
/// pin 2 anode — the EdgeBoard's `pinfunction` labels say the same).
pub fn nsi50010() -> PwlSpec {
    PwlSpec::new(["1", "2"]).with_branch(
        "2",
        "1",
        PwlCurve::Regulator {
            i_reg: NSI50010_I_REG_AMPS,
            v_reg: NSI50010_V_REG_VOLTS,
        },
    )
}

// ============================================================
// onsemi 2N3904 / MMBT3904, small-signal NPN (the servo-enable sink)
// ============================================================

/// The 2N3904's base–emitter knee: `V_BE(sat)` **0.65 V minimum** (0.85 V
/// maximum) at `I_C` = 10 mA, `I_B` = 1.0 mA (onsemi 2N3903/2N3904,
/// document 2N3903/D, Rev. 9, August 2021, "On Characteristics"). The
/// minimum: the smallest drop a driven base is guaranteed to have, so the
/// base reads its knee once any current flows.
pub const MMBT3904_VBE_VOLTS: Volts = 0.65;

/// The 2N3904's minimum DC current gain, `h_FE` **100** at `I_C` = 10 mA,
/// `V_CE` = 1.0 V (2N3903/D Rev. 9, "On Characteristics", the 2N3904
/// column; 300 maximum). The minimum, at the collector current the
/// EdgeBoard's servo-enable load draws: the collector current a base
/// current is guaranteed to support is `h_FE(min) · I_B`, and a load
/// asking for more leaves the part active — the sagging collector the
/// model reports rather than a clean switch (`PwlCurve::Bjt`).
pub const MMBT3904_HFE_MIN: f64 = 100.0;

/// The 2N3904's saturated collector–emitter resistance: `V_CE(sat)` 0.2 V
/// maximum at `I_C` = 10 mA, `I_B` = 1.0 mA (2N3903/D Rev. 9, "On
/// Characteristics") over that current, **20 Ω**.
pub const MMBT3904_R_SAT_OHMS: Ohms = 0.2 / 10e-3;

/// The keys the 2N3904 entry answers to: the EdgeBoard's manufacturer part
/// number (`MMBT3904-TP`, the SOT-23 part), the symbol's part name
/// (`2N3904`) and the plain SOT-23 number.
pub const MMBT3904_KEYS: &[&str] = &["MMBT3904-TP", "2N3904", "MMBT3904"];

/// The 2N3904 in the package order the EdgeBoard's `Q1` uses: `1` emitter,
/// `2` base, `3` collector (2N3903/D Rev. 9 TO-92 / SOT-23 drawings; the
/// netlist's `pinfunction` labels `E`, `B`, `C`). The base–emitter diode
/// is declared first, then the collector–emitter branch it gates: on
/// while `V(B) − V(E)` ≥ [`MMBT3904_VBE_VOLTS`], saturated at
/// [`MMBT3904_R_SAT_OHMS`] while the base supports the collector current,
/// active — `h_FE(min) · I_B` — when it does not.
pub fn mmbt3904() -> PwlSpec {
    PwlSpec::new(["1", "2", "3"])
        .with_branch(
            "2",
            "1",
            PwlCurve::Diode {
                vf: MMBT3904_VBE_VOLTS,
                r_d: 0.0,
            },
        )
        .with_controlled_branch(
            "3",
            "1",
            PwlCurve::Bjt {
                hfe: MMBT3904_HFE_MIN,
                r_sat: MMBT3904_R_SAT_OHMS,
            },
            "2",
            RegionTest::AtLeast(MMBT3904_VBE_VOLTS),
        )
}

// ============================================================
// The library
// ============================================================

/// One library entry: the keys it answers to, the specification and, for
/// an LED, the current above which it is lit.
#[derive(Debug, Clone)]
pub struct LibraryEntry {
    /// The manufacturer part numbers, part names and values this entry
    /// classifies.
    pub keys: &'static [&'static str],
    /// The datasheet the entry's numbers come from.
    pub provenance: &'static str,
    /// The specification.
    pub spec: fn() -> PwlSpec,
    /// For an LED, the branch current from anode to cathode at or above
    /// which the part is lit (see the module docs, "Lit"); `None` for a
    /// part that emits nothing.
    pub lit_amps: Option<Amps>,
}

/// Every entry in the library, in a fixed order.
pub const ENTRIES: &[LibraryEntry] = &[
    LibraryEntry {
        keys: SS36_KEYS,
        provenance: "Vishay SS32 thru SS36, Document Number 88751, Rev. 04-Aug-15",
        spec: ss36,
        lit_amps: None,
    },
    LibraryEntry {
        keys: LTST_C190KGKT_KEYS,
        provenance: "Lite-On LTST-C190KGKT, BNS-OD-C131/A4",
        spec: ltst_c190kgkt,
        lit_amps: Some(LTST_C190KGKT_I_ON_AMPS),
    },
    LibraryEntry {
        keys: IN_S63AS5UW_KEYS,
        provenance: "Inolux IN-S63AS series, datasheet V1.0, 03-16-2017",
        spec: in_s63as5uw,
        lit_amps: Some(IN_S63AS5UW_I_ON_AMPS),
    },
    LibraryEntry {
        keys: APM4953_KEYS,
        provenance:
            "ANPEC APM4953 Dual P-Channel Enhancement Mode MOSFET, Electrical Characteristics",
        spec: apm4953_half,
        lit_amps: None,
    },
    LibraryEntry {
        keys: SI3417DV_KEYS,
        provenance: "Vishay Siliconix Si3417DV, Document Number 62890, S13-1815-Rev. A, 12-Aug-13",
        spec: si3417dv,
        lit_amps: None,
    },
    LibraryEntry {
        keys: NSI50010_KEYS,
        provenance: "onsemi NSI50010Y/D, Rev. 3",
        spec: nsi50010,
        lit_amps: None,
    },
    LibraryEntry {
        keys: MMBT3904_KEYS,
        provenance: "onsemi 2N3903/2N3904, 2N3903/D, Rev. 9, August 2021",
        spec: mmbt3904,
        lit_amps: None,
    },
];

/// The entry a manufacturer part number, part name or value names, if any.
pub fn entry_for(key: &str) -> Option<&'static LibraryEntry> {
    ENTRIES.iter().find(|entry| entry.keys.contains(&key))
}

/// The specification the library holds for a manufacturer part number, a
/// part name or a value, if any.
pub fn spec_for(key: &str) -> Option<PwlSpec> {
    entry_for(key).map(|entry| (entry.spec)())
}

/// The current at or above which the LED a key names is lit, if the key
/// names an LED.
pub fn lit_threshold(key: &str) -> Option<Amps> {
    entry_for(key).and_then(|entry| entry.lit_amps)
}

/// Whether a branch current lights an LED whose threshold is `i_on`: the
/// current is known and at least the threshold. `None` — the LED's cluster
/// was not solved — is dark.
pub fn is_lit(current: Option<Amps>, i_on: Amps) -> bool {
    current.is_some_and(|amps| amps >= i_on)
}

/// Register every library entry under each of its keys.
pub fn register(registry: &mut PartRegistry) {
    for entry in ENTRIES {
        for key in entry.keys {
            registry.register_pwl(*key, (entry.spec)());
        }
    }
}

#[cfg(test)]
mod tests {
    use embsim_board::Classification;
    use rstest::rstest;

    use super::*;

    /// Every diode entry is a two-pin diode from its anode pin to its
    /// cathode pin, at a drop the datasheet tabulates.
    #[rstest]
    #[case::ss36("SS36-E3/57T", "2", "1", SS36_VF_VOLTS)]
    #[case::ss36_by_value("SS36", "2", "1", SS36_VF_VOLTS)]
    #[case::ltst("LTST-C190KGKT", "2", "1", LTST_C190KGKT_VF_VOLTS)]
    #[case::white("IN-S63AS5UW", "A", "K", IN_S63AS5UW_VF_VOLTS)]
    fn a_diode_entry_is_one_branch_from_the_anode_pin_to_the_cathode_pin(
        #[case] key: &str,
        #[case] anode: &str,
        #[case] cathode: &str,
        #[case] vf: f64,
    ) {
        let spec = spec_for(key).expect("the library holds the key");
        assert_eq!(spec.pins, vec![anode.to_string(), cathode.to_string()]);
        assert_eq!(spec.branches.len(), 1);
        let branch = &spec.branches[0];
        assert_eq!((branch.a.as_str(), branch.b.as_str()), (anode, cathode));
        assert!(branch.control.is_none());
        assert_eq!(branch.curve, PwlCurve::Diode { vf, r_d: 0.0 });
    }

    /// A FET entry is its body diode drain to source, then its channel
    /// drain to source gated by the gate at or below minus the threshold.
    #[rstest]
    #[case::apm4953(
        "APM4953KC-TRG",
        "7",
        "1",
        "2",
        APM4953_VSD_VOLTS,
        APM4953_R_DS_ON_OHMS,
        APM4953_VGS_TH_VOLTS
    )]
    #[case::apm4953_by_name(
        "APM4953",
        "7",
        "1",
        "2",
        APM4953_VSD_VOLTS,
        APM4953_R_DS_ON_OHMS,
        APM4953_VGS_TH_VOLTS
    )]
    #[case::si3417dv(
        "SI3417DV-T1-GE3",
        "D",
        "S",
        "G",
        SI3417DV_VSD_VOLTS,
        SI3417DV_R_DS_ON_OHMS,
        SI3417DV_VGS_TH_VOLTS
    )]
    fn a_fet_entry_is_a_body_diode_and_a_gated_channel(
        #[case] key: &str,
        #[case] drain: &str,
        #[case] source: &str,
        #[case] gate: &str,
        #[case] vsd: f64,
        #[case] r_on: f64,
        #[case] vth: f64,
    ) {
        let spec = spec_for(key).expect("the library holds the key");
        assert_eq!(spec.branches.len(), 2);
        let diode = &spec.branches[0];
        assert_eq!((diode.a.as_str(), diode.b.as_str()), (drain, source));
        assert_eq!(diode.curve, PwlCurve::Diode { vf: vsd, r_d: 0.0 });
        assert!(diode.control.is_none());
        let channel = &spec.branches[1];
        assert_eq!((channel.a.as_str(), channel.b.as_str()), (drain, source));
        assert_eq!(channel.curve, PwlCurve::Channel { r_on });
        assert_eq!(
            channel
                .control
                .as_ref()
                .map(|(pin, test)| (pin.as_str(), *test)),
            Some((gate, RegionTest::AtMost(-vth)))
        );
    }

    /// The regulator entry is one two-region branch from anode to cathode
    /// at the datasheet's regulation current and overhead.
    #[rstest]
    fn the_regulator_entry_is_one_regulating_branch() {
        let spec = spec_for("NSI50010YT1G").expect("the library holds the key");
        assert_eq!(spec.pins, vec!["1".to_string(), "2".to_string()]);
        assert_eq!(spec.branches.len(), 1);
        let branch = &spec.branches[0];
        assert_eq!((branch.a.as_str(), branch.b.as_str()), ("2", "1"));
        assert_eq!(
            branch.curve,
            PwlCurve::Regulator {
                i_reg: NSI50010_I_REG_AMPS,
                v_reg: NSI50010_V_REG_VOLTS,
            }
        );
    }

    /// The transistor entry is its base–emitter diode, then its collector
    /// gated by the base at the same knee.
    #[rstest]
    #[case::by_number("MMBT3904-TP")]
    #[case::by_part("2N3904")]
    fn the_transistor_entry_is_a_base_diode_and_a_gated_collector(#[case] key: &str) {
        let spec = spec_for(key).expect("the library holds the key");
        assert_eq!(
            spec.pins,
            vec!["1".to_string(), "2".to_string(), "3".to_string()]
        );
        assert_eq!(spec.branches.len(), 2);
        let base = &spec.branches[0];
        assert_eq!((base.a.as_str(), base.b.as_str()), ("2", "1"));
        assert_eq!(
            base.curve,
            PwlCurve::Diode {
                vf: MMBT3904_VBE_VOLTS,
                r_d: 0.0
            }
        );
        let collector = &spec.branches[1];
        assert_eq!((collector.a.as_str(), collector.b.as_str()), ("3", "1"));
        assert_eq!(
            collector.curve,
            PwlCurve::Bjt {
                hfe: MMBT3904_HFE_MIN,
                r_sat: MMBT3904_R_SAT_OHMS
            }
        );
        assert_eq!(
            collector
                .control
                .as_ref()
                .map(|(pin, test)| (pin.as_str(), *test)),
            Some(("2", RegionTest::AtLeast(MMBT3904_VBE_VOLTS)))
        );
    }

    /// Only the LEDs have a lit threshold, and it is the current the
    /// datasheet distinguishes from dark.
    #[rstest]
    fn only_the_leds_have_a_lit_threshold() {
        assert_eq!(
            lit_threshold("LTST-C190KGKT"),
            Some(LTST_C190KGKT_I_ON_AMPS)
        );
        assert_eq!(lit_threshold("IN-S63AS5UW"), Some(IN_S63AS5UW_I_ON_AMPS));
        for key in ["SS36", "APM4953", "SI3417DV", "NSI50010YT1G", "2N3904"] {
            assert_eq!(lit_threshold(key), None, "{key}");
        }
        assert!(is_lit(Some(5e-3), LTST_C190KGKT_I_ON_AMPS));
        assert!(!is_lit(Some(5e-6), LTST_C190KGKT_I_ON_AMPS));
        assert!(!is_lit(None, LTST_C190KGKT_I_ON_AMPS));
    }

    /// A key the library does not hold resolves to nothing — no generic
    /// part is invented; a colour or a function is not a key.
    #[rstest]
    fn an_unknown_key_has_no_entry() {
        for key in ["1N4148", "LED", "White", "NPN", "P Mosfet 30V 8A"] {
            assert!(spec_for(key).is_none(), "{key}");
        }
    }

    /// Registering the library classifies a part by its manufacturer part
    /// number, whatever its symbol is called, and by its part name.
    #[rstest]
    #[case::led_by_number("D3", "LED", "Lamp", Some("LTST-C190KGKT"))]
    #[case::fet_by_number("U3", "APM4953", "APM4953", Some("APM4953KC-TRG"))]
    #[case::regulator_by_number("IC6", "NSI50010YT1G", "NSI50010YT1G_1", Some("NSI50010YT1G"))]
    #[case::transistor_by_part("Q1", "NPN", "2N3904", None)]
    #[case::module_fet_by_number("U401", "P Mosfet 30V 8A", "", Some("SI3417DV-T1-GE3"))]
    #[case::white_by_number("D601", "White", "", Some("IN-S63AS5UW"))]
    fn the_library_registers_every_key(
        #[case] reference: &str,
        #[case] value: &str,
        #[case] part: &str,
        #[case] mpn: Option<&str>,
    ) {
        let mut registry = PartRegistry::new();
        register(&mut registry);
        for entry in ENTRIES {
            for key in entry.keys {
                assert!(registry.has_part(key), "{key}");
            }
        }
        let decl = embsim_board::ComponentDecl {
            reference: reference.to_string(),
            value: value.to_string(),
            footprint: String::new(),
            lib: "Device".to_string(),
            part: part.to_string(),
            sheetpath: "/".to_string(),
            dnp: false,
            mpn: mpn.map(str::to_string),
        };
        let pins = spec_for(mpn.unwrap_or(part)).expect("an entry").pins.len();
        assert!(
            matches!(
                registry.classify(&decl, pins),
                Ok(Classification::Pwl { .. })
            ),
            "{reference}: {:?}",
            registry.classify(&decl, pins)
        );
    }
}
