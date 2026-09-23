//! The flash layout the Parallax boot ROM and this crate's stage-1 loader
//! agree on.
//!
//! The ROM loads the first kilobyte of the flash into cog RAM, sums its 256
//! longs, and runs it only if they total `"Prop"`. That kilobyte is
//! `rom/stage1.spin2`, which reads the application's length from `$400` and
//! the application itself from `$404`, copies it to hub `$0`, and relaunches
//! cog 0 on it — the same chain the P2's own loaders use.

/// What the ROM wants the first 256 longs to sum to.
const PROP: u32 = u32::from_le_bytes(*b"Prop");

/// One long in the first kilobyte the stage-1 loader leaves free, so the sum
/// can be balanced.
const FIXUP_AT: usize = 0x3FC;

/// The application's length lives here; the application follows.
const PROGRAM_LEN_AT: usize = 0x400;

/// Why an image could not be built.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FlashImageError {
    /// Stage-1 must leave the fix-up long free.
    Stage1TooBig {
        /// The stage-1 length offered.
        len: usize,
    },
}

impl std::fmt::Display for FlashImageError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            FlashImageError::Stage1TooBig { len } => write!(
                f,
                "stage-1 is {len} bytes; it must leave the fix-up long at ${FIXUP_AT:X} free"
            ),
        }
    }
}

impl std::error::Error for FlashImageError {}

/// A bootable image: `stage1` in the first kilobyte balanced to `"Prop"`,
/// then `program`'s length and `program` at `$400`.
pub fn boot_flash(stage1: &[u8], program: &[u8]) -> Result<Vec<u8>, FlashImageError> {
    if stage1.len() > FIXUP_AT {
        return Err(FlashImageError::Stage1TooBig { len: stage1.len() });
    }
    let mut img = vec![0u8; PROGRAM_LEN_AT + 4 + program.len()];
    img[..stage1.len()].copy_from_slice(stage1);
    img[PROGRAM_LEN_AT..PROGRAM_LEN_AT + 4].copy_from_slice(&(program.len() as u32).to_le_bytes());
    img[PROGRAM_LEN_AT + 4..].copy_from_slice(program);

    // Balance the first kilobyte to "Prop". The ROM sums 256 longs and
    // compares; one free long absorbs the difference.
    let mut sum = 0u32;
    for i in (0..PROGRAM_LEN_AT).step_by(4) {
        sum = sum.wrapping_add(u32::from_le_bytes(img[i..i + 4].try_into().unwrap()));
    }
    let fix = PROP.wrapping_sub(sum);
    img[FIXUP_AT..FIXUP_AT + 4].copy_from_slice(&fix.to_le_bytes());
    Ok(img)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_first_kilobyte_sums_to_prop() {
        let img = boot_flash(&[0xAA; 100], &[1, 2, 3, 4]).expect("builds");
        let mut sum = 0u32;
        for i in (0..0x400).step_by(4) {
            sum = sum.wrapping_add(u32::from_le_bytes(img[i..i + 4].try_into().unwrap()));
        }
        assert_eq!(sum, PROP);
        assert_eq!(&img[0x400..0x404], &4u32.to_le_bytes());
        assert_eq!(&img[0x404..], &[1, 2, 3, 4]);
    }

    #[test]
    fn a_stage1_over_the_fixup_long_is_refused() {
        assert_eq!(
            boot_flash(&[0; 0x3FD], &[]),
            Err(FlashImageError::Stage1TooBig { len: 0x3FD })
        );
    }
}
