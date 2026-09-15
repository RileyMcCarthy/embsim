//! Choosing the inputs, which decides whether the corpus can prove anything.
//!
//! A silicon corpus is only as good as the operands it was captured with, and
//! this is far easier to get wrong than it looks. A real example, from a P2
//! sweep that ran two operand pairs — `$80000000 op $00000001` and
//! `$00000002 op $00000003`:
//!
//! > `SAL` ("shift arithmetic left") fills the vacated low bits with D's
//! > original bit 0. The simulator implemented it as a plain left shift. Both
//! > captured D operands have bit 0 clear, so fill-with-zero and
//! > fill-with-bit-0 give the same answer for **every case in the corpus**.
//! > 1550 records, all green, and one of the most basic shifts in the ISA was
//! > wrong.
//!
//! The same sweep left `C` and `Z` clear in 1548 of 1550 records, so every
//! flag-conditioned instruction — add-with-carry, the conditional writes, the
//! conditional rotates — was only ever exercised on one of its two branches.
//! Widening to the vectors below moved that corpus from 12 divergent mnemonics
//! to 30: eighteen instructions had been passing for lack of an input that
//! could tell them apart.
//!
//! Hence [`DEFAULT_OPERANDS`] and [`DEFAULT_FLAG_STATES`]. They are not
//! arbitrary: each one exists to separate a rule from a plausible wrong rule.

/// One (destination, source) input, with the suffix that names its cases.
///
/// 32-bit because that is what the ISAs this has been used on are; an adapter
/// for a wider machine supplies its own table rather than bending this one.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OperandPair {
    /// Appended to the case name, so `_c` variants are greppable in a golden.
    pub suffix: &'static str,
    pub d: u32,
    pub s: u32,
}

/// Inputs chosen so that a wrong implementation has somewhere to show itself.
///
/// * `""` and `_b` — the sign boundary and a pair of small positives. These
///   two alone leave bit 0 clear in both destinations, which is exactly how a
///   shift bug survives.
/// * `_c` — all ones. Separates fills, saturations and masks from no-ops.
/// * `_d` — the other sign boundary, for anything that treats overflow
///   specially.
/// * `_e` — alternating bits, disjoint between D and S. This is the one that
///   catches bit permutations (merge/split/rev) and multiplies, where the
///   tidy operands above collide.
/// * `_f` — both zero: the identity case, and the shift count that must not
///   become 32.
pub const DEFAULT_OPERANDS: &[OperandPair] = &[
    OperandPair {
        suffix: "",
        d: 0x8000_0000,
        s: 0x0000_0001,
    },
    OperandPair {
        suffix: "_b",
        d: 0x0000_0002,
        s: 0x0000_0003,
    },
    OperandPair {
        suffix: "_c",
        d: 0xFFFF_FFFF,
        s: 0x0000_0001,
    },
    OperandPair {
        suffix: "_d",
        d: 0x7FFF_FFFF,
        s: 0x0000_0001,
    },
    OperandPair {
        suffix: "_e",
        d: 0xA5A5_A5A5,
        s: 0x5A5A_5A5A,
    },
    OperandPair {
        suffix: "_f",
        d: 0x0000_0000,
        s: 0x0000_0000,
    },
];

/// Incoming flag states to run the base operand against: none, C, Z, both.
///
/// Bit 0 is carry, bit 1 is zero. A corpus captured only at 0 cannot tell a
/// conditional instruction from its own opposite, because it only ever sees
/// one branch taken.
pub const DEFAULT_FLAG_STATES: &[u8] = &[0, 1, 2, 3];

/// One planned case: an encoding plus the inputs to run it with.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CasePlan {
    pub name: String,
    pub encoding: u32,
    pub d: u32,
    pub s: u32,
    /// Bit 0 carry, bit 1 zero.
    pub flags: u8,
}

/// An encoding to sweep, named by its mnemonic.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Encoding {
    pub word: u32,
    pub mnemonic: &'static str,
}

/// Cross `encodings` with the operand and flag vectors.
///
/// Deliberately not a full cross product. Every encoding gets every operand
/// pair at flags = 0, and the *base* pair at each remaining flag state. The
/// full product multiplies board time by the number of flag states for inputs
/// that, for most instructions, cannot interact with the flags at all — and
/// board time is the scarce resource: these captures run at roughly ten cases
/// a second over a serial link.
pub fn plan_cases(
    encodings: &[Encoding],
    operands: &[OperandPair],
    flag_states: &[u8],
) -> Vec<CasePlan> {
    let mut out = Vec::new();
    for enc in encodings {
        let stem = format!("{}_{:08x}", enc.mnemonic, enc.word);
        for pair in operands {
            out.push(CasePlan {
                name: format!("{stem}{}", pair.suffix),
                encoding: enc.word,
                d: pair.d,
                s: pair.s,
                flags: 0,
            });
        }
        let Some(base) = operands.first() else {
            continue;
        };
        for &f in flag_states {
            if f == 0 {
                continue;
            }
            out.push(CasePlan {
                name: format!("{stem}_f{f}"),
                encoding: enc.word,
                d: base.d,
                s: base.s,
                flags: f,
            });
        }
    }
    out
}

/// Cases per encoding that [`plan_cases`] will emit for the given vectors.
pub fn cases_per_encoding(operands: &[OperandPair], flag_states: &[u8]) -> usize {
    operands.len() + flag_states.iter().filter(|&&f| f != 0).count()
}

#[cfg(test)]
mod tests {
    use super::*;

    const ENC: &[Encoding] = &[Encoding {
        word: 0xF103_C1E1,
        mnemonic: "add",
    }];

    #[test]
    fn the_default_operands_vary_bit_zero() {
        // The bug this table exists to catch: a corpus where every destination
        // has bit 0 clear cannot see a shift that fills from bit 0.
        assert!(
            DEFAULT_OPERANDS.iter().any(|p| p.d & 1 == 1),
            "at least one destination must have bit 0 set"
        );
        assert!(DEFAULT_OPERANDS.iter().any(|p| p.d & 1 == 0));
    }

    #[test]
    fn the_default_operands_vary_the_sign_and_include_disjoint_bits() {
        assert!(DEFAULT_OPERANDS.iter().any(|p| p.d >> 31 == 1));
        assert!(DEFAULT_OPERANDS.iter().any(|p| p.d >> 31 == 0));
        // A permutation of bits is invisible if D and S share every bit.
        assert!(
            DEFAULT_OPERANDS.iter().any(|p| p.d & p.s == 0 && p.d != 0),
            "a disjoint pair is what catches bit permutations"
        );
    }

    #[test]
    fn both_branches_of_a_conditional_are_reachable() {
        assert!(DEFAULT_FLAG_STATES.contains(&0));
        assert!(DEFAULT_FLAG_STATES.iter().any(|&f| f & 1 == 1), "carry set");
        assert!(DEFAULT_FLAG_STATES.iter().any(|&f| f & 2 == 2), "zero set");
    }

    #[test]
    fn each_encoding_gets_every_operand_and_every_nonzero_flag_state() {
        let cases = plan_cases(ENC, DEFAULT_OPERANDS, DEFAULT_FLAG_STATES);
        assert_eq!(
            cases.len(),
            cases_per_encoding(DEFAULT_OPERANDS, DEFAULT_FLAG_STATES)
        );
        assert_eq!(cases.len(), 6 + 3);
        assert_eq!(cases.iter().filter(|c| c.flags == 0).count(), 6);
        for &f in &[1u8, 2, 3] {
            assert_eq!(cases.iter().filter(|c| c.flags == f).count(), 1);
        }
    }

    #[test]
    fn case_names_are_unique_and_carry_the_encoding() {
        let cases = plan_cases(ENC, DEFAULT_OPERANDS, DEFAULT_FLAG_STATES);
        let mut names: Vec<&str> = cases.iter().map(|c| c.name.as_str()).collect();
        names.sort_unstable();
        let before = names.len();
        names.dedup();
        assert_eq!(names.len(), before, "case names must be unique");
        assert!(cases.iter().all(|c| c.name.starts_with("add_f103c1e1")));
    }

    #[test]
    fn no_encodings_plans_nothing() {
        assert!(plan_cases(&[], DEFAULT_OPERANDS, DEFAULT_FLAG_STATES).is_empty());
    }
}
