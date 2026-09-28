//! A FAT16 card image, built in memory, for a guest filesystem to mount.
//!
//! A block-device model is only half a card: an embedded filesystem mounting a
//! blank one fails, and most firmware cannot format — `FF_USE_MKFS` is off in
//! a typical FatFs build — so the image has to arrive already formatted. This
//! builds one, with whatever files and directories the guest expects, without
//! going near a host filesystem or a `mkfs` binary.
//!
//! Pairs with [`crate::sd_card`]: build an image here, hand it to
//! `SdCard::with_image`, and the guest has a card it can actually mount.
//!
//! # The constraints, and where they come from
//!
//! These are read out of FatFs R0.14b's `ffconf.h` as FlexC 7.4.3 ships it
//! (`include/filesys/fatfs/ffconf.h`, `FFCONF_DEF` 86631) rather than assumed,
//! and they are the common embedded configuration:
//!
//! | knob | value | consequence |
//! |---|---|---|
//! | `FF_MIN_SS` / `FF_MAX_SS` | 512 / 512 | sectors are exactly 512 bytes |
//! | `FF_FS_EXFAT` | 0 | FAT12/16/32 only |
//! | `FF_USE_LFN` | 0 | **8.3 short names only** |
//! | `FF_USE_MKFS` | 0 | it cannot format; the image must arrive formatted |
//! | `FF_MULTI_PARTITION` | 0 | one volume |
//!
//! # Why FAT16, and why no partition table
//!
//! FAT16 is the middle child that avoids both awkward ends: no FAT12
//! twelve-bit packing, no FAT32 `FSInfo` bookkeeping. The image is a
//! *superfloppy* — a boot sector at LBA 0 with no MBR — which FatFs accepts
//! directly, because `check_fs` looks for a FAT VBR before it looks for a
//! partition table.
//!
//! The FAT type is **derived from the cluster count**, not from the label in
//! the boot sector, so geometry that lands outside FAT16's window is read as
//! FAT12 or FAT32 whatever the image claims to be. The window is 4086..=65524
//! data clusters: the counts that both FatFs R0.14b (`MAX_FAT12` and
//! `MAX_FAT16` in `ff.c`) and Microsoft's fatgen103 v1.03 ("FAT Type
//! Determination") read as FAT16. The two disagree at 4085, which FatFs reads
//! as FAT12, and at 65525, which fatgen103 reads as FAT32, so neither is in it.
//! [`build`] refuses a size outside the window rather than emitting a volume
//! that mounts as the wrong thing — see [`ImageError::NotFat16`].

use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;

/// Bytes per sector. Not a choice — `FF_MIN_SS == FF_MAX_SS == 512`.
const SECTOR: usize = 512;
/// Sectors per cluster. 4 gives 2 KiB clusters, which keeps a 32 MiB card
/// comfortably inside FAT16's cluster-count window (see [`FAT16_MIN_CLUSTERS`]).
const SECTORS_PER_CLUSTER: usize = 4;
/// Reserved sectors before the first FAT. One is the boot sector itself.
const RESERVED_SECTORS: usize = 1;
/// Two FATs, as every real card has: FatFs reads the first and the second is
/// the mirror a recovery tool would use.
const NUM_FATS: usize = 2;
/// Root directory entries. FAT16's root is a fixed-size area, not a cluster
/// chain; 512 entries is the conventional size and 32 sectors of it.
const ROOT_ENTRIES: usize = 512;
/// One directory entry is 32 bytes.
const DIR_ENTRY: usize = 32;

/// The fewest data clusters a FAT16 volume can have, for every reader this
/// image is for. The type is *derived* from the count, whatever the boot
/// sector says, so an image under this is misread however it labels itself.
///
/// - FatFs R0.14b (`ff.c`, as FlexC 7.4.3 ships it) defines
///   `MAX_FAT12 0xFF5` and classifies `nclst <= MAX_FAT12` as FAT12, so it
///   reads 4085 clusters as FAT12. Its `nclst` is counted the way [`build`]
///   counts: sectors past the reserved area, the FATs and the root directory,
///   divided by sectors per cluster.
/// - Microsoft fatgen103 v1.03, "FAT Type Determination": FAT12 below 4085,
///   FAT16 below 65525, so it reads 4085 as FAT16.
///
/// They disagree at exactly 4085; 4086 is the first count both call FAT16.
const FAT16_MIN_CLUSTERS: usize = 4086;
/// The most data clusters a FAT16 volume can have, for every reader.
///
/// - FatFs R0.14b defines `MAX_FAT16 0xFFF5` and classifies
///   `nclst <= MAX_FAT16` as FAT16, so up to 65525.
/// - fatgen103 v1.03, "FAT Type Determination": FAT16 below 65525, FAT32 from
///   there, so up to 65524.
///
/// `FAT16_MIN_CLUSTERS..=FAT16_MAX_CLUSTERS`, 4086..=65524, is the window both
/// read as FAT16.
const FAT16_MAX_CLUSTERS: usize = 65524;

/// First cluster number that addresses data. 0 and 1 are reserved.
const FIRST_DATA_CLUSTER: u16 = 2;

const ATTR_DIRECTORY: u8 = 0x10;

/// `BPB_Media`: fatgen103 v1.03, "Boot Sector and BPB Structure" — 0xF8 is
/// the standard value for fixed (non-removable) media, and whatever is put
/// there must also be the low byte of `FAT[0]` ("FAT Data Structure").
const MEDIA_FIXED: u8 = 0xF8;
/// `BS_VolID`. fatgen103 has a formatter derive it from the date and time;
/// this is a fixed value instead, so an image built twice from the same tree
/// is byte-identical — the same reason as the fixed timestamp in `dir_entry`.
const VOLUME_SERIAL: u32 = 0x1234_5678;
/// `BS_VolLab`: fatgen103 v1.03, "FAT12 and FAT16 Structure Starting at
/// Offset 36" — `"NO NAME    "` is the value when a volume has no label.
const VOLUME_LABEL: &[u8; 11] = b"NO NAME    ";

/// A directory being built, before it is laid out on the card.
#[derive(Debug, Default)]
pub struct Dir {
    files: BTreeMap<String, Vec<u8>>,
    dirs: BTreeMap<String, Dir>,
}

impl Dir {
    pub fn new() -> Self {
        Self::default()
    }

    /// Add a file. `name` is converted to 8.3 and must fit.
    pub fn file(&mut self, name: &str, contents: Vec<u8>) -> &mut Self {
        self.files.insert(name.to_string(), contents);
        self
    }

    /// Add (or reach into) a subdirectory.
    pub fn dir(&mut self, name: &str) -> &mut Dir {
        self.dirs.entry(name.to_string()).or_default()
    }

    /// Mirror a host directory onto the card.
    ///
    /// Every I/O error comes back with the path it happened on; a caller whose
    /// fixture is optional checks `path.exists()` first.
    ///
    /// A symlink is followed, to a file or a directory alike. An entry that is
    /// still neither a file nor a directory (a socket, a FIFO, a device) is
    /// skipped, and so is a dotfile, which has no 8.3 form.
    pub fn from_host_dir(path: &Path) -> std::io::Result<Dir> {
        let mut out = Dir::new();
        for entry in std::fs::read_dir(path).map_err(at_path(path))? {
            let entry = entry.map_err(at_path(path))?;
            let name = entry.file_name().to_string_lossy().to_string();
            if name.starts_with('.') {
                continue;
            }
            let entry_path = entry.path();
            // `metadata`, not `entry.file_type()`: the latter describes a
            // symlink as itself, which is neither a file nor a directory.
            let meta = std::fs::metadata(&entry_path).map_err(at_path(&entry_path))?;
            if meta.is_dir() {
                out.dirs.insert(name, Dir::from_host_dir(&entry_path)?);
            } else if meta.is_file() {
                let bytes = std::fs::read(&entry_path).map_err(at_path(&entry_path))?;
                out.files.insert(name, bytes);
            }
        }
        Ok(out)
    }
}

/// Wrap an I/O error with the path it happened on, keeping its kind.
fn at_path(path: &Path) -> impl FnOnce(std::io::Error) -> std::io::Error + '_ {
    move |e| std::io::Error::new(e.kind(), format!("{}: {e}", path.display()))
}

/// Why an image could not be built.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ImageError {
    /// The requested size lands outside FAT16's cluster-count window, where
    /// the volume would be read as FAT12 or FAT32 regardless of its label.
    NotFat16 {
        /// Data clusters the geometry produced.
        clusters: usize,
    },
    /// A name does not fit 8.3, and this FatFs has long names disabled.
    BadName {
        /// The offending name.
        name: String,
    },
    /// The contents do not fit in the card.
    Full {
        /// Clusters the contents need.
        needed: usize,
        /// Clusters the card has.
        available: usize,
    },
    /// A directory holds more entries than its area can list.
    DirFull {
        /// Which directory.
        name: String,
    },
    /// Two names in one directory fold to the same 8.3 entry — `a.bin` and
    /// `A.BIN`, or a file and a subdirectory both called `data` — and a
    /// directory that lists one short name twice is corrupt.
    DuplicateName {
        /// The second of the two, in the order the directory is laid out.
        name: String,
    },
}

impl std::fmt::Display for ImageError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ImageError::NotFat16 { clusters } => write!(
                f,
                "{clusters} data clusters is outside FAT16's window \
                 ({FAT16_MIN_CLUSTERS}..={FAT16_MAX_CLUSTERS}); pick another card size"
            ),
            ImageError::BadName { name } => {
                write!(
                    f,
                    "{name:?} does not fit 8.3 and this FatFs has FF_USE_LFN=0"
                )
            }
            ImageError::Full { needed, available } => {
                write!(
                    f,
                    "contents need {needed} clusters; the card has {available}"
                )
            }
            ImageError::DirFull { name } => write!(f, "directory {name:?} has too many entries"),
            ImageError::DuplicateName { name } => write!(
                f,
                "{name:?} folds to the same 8.3 entry as another name in its directory"
            ),
        }
    }
}

impl std::error::Error for ImageError {}

/// Convert to a FAT 8.3 name: eight bytes of stem, three of extension, space
/// padded, upper case.
fn short_name(name: &str) -> Result<[u8; 11], ImageError> {
    let bad = || ImageError::BadName {
        name: name.to_string(),
    };
    let (stem, ext) = match name.rsplit_once('.') {
        Some((s, e)) => (s, e),
        None => (name, ""),
    };
    if stem.is_empty() || stem.len() > 8 || ext.len() > 3 {
        return Err(bad());
    }
    let mut out = [b' '; 11];
    for (i, c) in stem.bytes().enumerate() {
        if !c.is_ascii_alphanumeric() && !b"_-~".contains(&c) {
            return Err(bad());
        }
        out[i] = c.to_ascii_uppercase();
    }
    for (i, c) in ext.bytes().enumerate() {
        if !c.is_ascii_alphanumeric() {
            return Err(bad());
        }
        out[8 + i] = c.to_ascii_uppercase();
    }
    Ok(out)
}

/// A FAT16 volume under construction.
struct Layout {
    bytes: Vec<u8>,
    fat: Vec<u16>,
    sectors_per_fat: usize,
    data_start: usize,
    total_clusters: usize,
    next_free: u16,
}

impl Layout {
    fn cluster_bytes(&self) -> usize {
        SECTORS_PER_CLUSTER * SECTOR
    }

    fn cluster_offset(&self, cluster: u16) -> usize {
        self.data_start + (cluster as usize - FIRST_DATA_CLUSTER as usize) * self.cluster_bytes()
    }

    /// Allocate a chain long enough for `len` bytes, returning its first
    /// cluster (0 for an empty file — FAT's convention for "no data").
    fn alloc(&mut self, len: usize) -> Result<u16, ImageError> {
        if len == 0 {
            return Ok(0);
        }
        let need = len.div_ceil(self.cluster_bytes());
        let first = self.next_free;
        for i in 0..need {
            let c = self.next_free;
            if (c as usize) >= FIRST_DATA_CLUSTER as usize + self.total_clusters {
                return Err(ImageError::Full {
                    needed: need,
                    available: self.total_clusters,
                });
            }
            self.next_free += 1;
            // 0xFFFF ends the chain; otherwise point at the next.
            self.fat[c as usize] = if i + 1 == need {
                0xFFFF
            } else {
                self.next_free
            };
        }
        Ok(first)
    }

    fn write_chain(&mut self, first: u16, data: &[u8]) {
        let mut cluster = first;
        let size = self.cluster_bytes();
        for chunk in data.chunks(size) {
            let at = self.cluster_offset(cluster);
            self.bytes[at..at + chunk.len()].copy_from_slice(chunk);
            cluster = self.fat[cluster as usize];
        }
    }
}

/// One 32-byte directory entry.
fn dir_entry(name: [u8; 11], attr: u8, cluster: u16, size: u32) -> [u8; DIR_ENTRY] {
    let mut e = [0u8; DIR_ENTRY];
    e[0..11].copy_from_slice(&name);
    e[11] = attr;
    // A fixed timestamp, not the wall clock: an image built twice from the
    // same tree must be byte-identical, or a golden trace of the boot would
    // depend on when it ran. 1980-01-01 00:00 is FAT's own epoch.
    e[22..24].copy_from_slice(&0u16.to_le_bytes()); // time
    e[24..26].copy_from_slice(&0x21u16.to_le_bytes()); // date: 1980-01-01
    e[26..28].copy_from_slice(&cluster.to_le_bytes());
    e[28..32].copy_from_slice(&size.to_le_bytes());
    e
}

/// Lay a directory's contents out, returning its packed entry table.
///
/// Recursive: a subdirectory's own cluster is allocated here, filled with its
/// `.`/`..` pair and its children, and written before returning.
fn build_dir(
    layout: &mut Layout,
    dir: &Dir,
    self_cluster: u16,
    parent_cluster: u16,
    is_root: bool,
) -> Result<Vec<u8>, ImageError> {
    let mut entries: Vec<u8> = Vec::new();

    if !is_root {
        // Every non-root directory opens with `.` and `..`. FatFs walks these
        // to resolve a path, so a directory without them is unreachable.
        let mut dot = [b' '; 11];
        dot[0] = b'.';
        entries.extend_from_slice(&dir_entry(dot, ATTR_DIRECTORY, self_cluster, 0));
        let mut dotdot = [b' '; 11];
        dotdot[0] = b'.';
        dotdot[1] = b'.';
        // The root is cluster 0 from a child's point of view.
        entries.extend_from_slice(&dir_entry(dotdot, ATTR_DIRECTORY, parent_cluster, 0));
    }

    // Every short name this directory lists, files and subdirectories alike.
    // Two host names can fold to one (`a.bin` and `A.BIN`), and a directory
    // that lists the same entry twice is corrupt, so a collision is refused
    // before anything is allocated for it.
    let mut listed: BTreeSet<[u8; 11]> = BTreeSet::new();
    let mut claim = |name: &str| -> Result<[u8; 11], ImageError> {
        let short = short_name(name)?;
        if !listed.insert(short) {
            return Err(ImageError::DuplicateName {
                name: name.to_string(),
            });
        }
        Ok(short)
    };

    for (name, contents) in &dir.files {
        let short = claim(name)?;
        let first = layout.alloc(contents.len())?;
        if first != 0 {
            layout.write_chain(first, contents);
        }
        entries.extend_from_slice(&dir_entry(
            short,
            0x20, // archive
            first,
            contents.len() as u32,
        ));
    }

    for (name, sub) in &dir.dirs {
        let short = claim(name)?;
        // A directory always occupies at least one cluster, even when empty:
        // it has to hold `.` and `..`.
        let cluster = layout.alloc(1)?;
        let sub_entries = build_dir(layout, sub, cluster, self_cluster, false)?;
        if sub_entries.len() > layout.cluster_bytes() {
            return Err(ImageError::DirFull { name: name.clone() });
        }
        layout.write_chain(cluster, &sub_entries);
        entries.extend_from_slice(&dir_entry(short, ATTR_DIRECTORY, cluster, 0));
    }

    Ok(entries)
}

/// Build a FAT16 image of `size_bytes` holding `root`.
///
/// The result is a raw block device: hand it to
/// [`crate::sd_card::SdCard::with_image`].
pub fn build(size_bytes: usize, root: &Dir) -> Result<Vec<u8>, ImageError> {
    let total_sectors = size_bytes / SECTOR;
    let root_dir_sectors = ROOT_ENTRIES * DIR_ENTRY / SECTOR;

    // Sectors per FAT has to be solved for, because the FAT sizes itself to
    // the cluster count and the cluster count depends on how much the FATs
    // take. Converge rather than approximate: at 2 KiB clusters this settles
    // in two or three rounds.
    let mut sectors_per_fat = 1usize;
    let mut clusters;
    loop {
        let overhead = RESERVED_SECTORS + NUM_FATS * sectors_per_fat + root_dir_sectors;
        clusters = total_sectors.saturating_sub(overhead) / SECTORS_PER_CLUSTER;
        // Two bytes per FAT16 entry, plus the two reserved entries.
        let need = ((clusters + 2) * 2).div_ceil(SECTOR);
        if need <= sectors_per_fat {
            break;
        }
        sectors_per_fat = need;
    }

    if !(FAT16_MIN_CLUSTERS..=FAT16_MAX_CLUSTERS).contains(&clusters) {
        return Err(ImageError::NotFat16 { clusters });
    }

    let fat_start = RESERVED_SECTORS;
    let root_start = fat_start + NUM_FATS * sectors_per_fat;
    let data_start = (root_start + root_dir_sectors) * SECTOR;

    let mut layout = Layout {
        bytes: vec![0u8; total_sectors * SECTOR],
        fat: vec![0u16; clusters + FIRST_DATA_CLUSTER as usize],
        sectors_per_fat,
        data_start,
        total_clusters: clusters,
        next_free: FIRST_DATA_CLUSTER,
    };
    // The two reserved FAT entries, fatgen103 v1.03 "FAT Data Structure":
    // FAT[0] holds BPB_Media in its low byte with every other bit set, and
    // FAT[1] holds the end-of-chain mark a formatter writes there.
    layout.fat[0] = 0xFF00 | u16::from(MEDIA_FIXED);
    layout.fat[1] = 0xFFFF;

    let root_entries = build_dir(&mut layout, root, 0, 0, true)?;
    if root_entries.len() > ROOT_ENTRIES * DIR_ENTRY {
        return Err(ImageError::DirFull {
            name: "/".to_string(),
        });
    }

    // -- boot sector ---------------------------------------------------
    // Field by field from Microsoft fatgen103 v1.03: offsets 0..36 are its
    // "Boot Sector and BPB Structure" table, 36..62 its "FAT12 and FAT16
    // Structure Starting at Offset 36" table. Fields not written stay zero:
    // BPB_HiddSec is 0 because nothing precedes this volume (no partition
    // table), and BS_Reserved1 is 0 as the table requires.
    let bs = &mut layout.bytes[0..SECTOR];

    // BS_jmpBoot, the short-jump form EB xx 90. 0x3C lands on offset 0x3E,
    // the first byte past BS_FilSysType, where FAT12/16 boot code starts.
    // FatFs's `check_fs` only takes a sector whose first byte is a jump.
    bs[0..3].copy_from_slice(&[0xEB, 0x3C, 0x90]);
    // BS_OEMName: fatgen103 recommends "MSWIN4.1" as the value least likely
    // to cause compatibility problems.
    bs[3..11].copy_from_slice(b"MSWIN4.1");

    bs[11..13].copy_from_slice(&(SECTOR as u16).to_le_bytes()); // BPB_BytsPerSec
    bs[13] = SECTORS_PER_CLUSTER as u8; // BPB_SecPerClus
    bs[14..16].copy_from_slice(&(RESERVED_SECTORS as u16).to_le_bytes()); // BPB_RsvdSecCnt
    bs[16] = NUM_FATS as u8; // BPB_NumFATs
    bs[17..19].copy_from_slice(&(ROOT_ENTRIES as u16).to_le_bytes()); // BPB_RootEntCnt

    // BPB_TotSec16 when the count fits sixteen bits; otherwise that field is
    // 0 and the count goes in BPB_TotSec32.
    if total_sectors < 0x1_0000 {
        bs[19..21].copy_from_slice(&(total_sectors as u16).to_le_bytes());
    } else {
        bs[32..36].copy_from_slice(&(total_sectors as u32).to_le_bytes());
    }
    bs[21] = MEDIA_FIXED; // BPB_Media
    bs[22..24].copy_from_slice(&(sectors_per_fat as u16).to_le_bytes()); // BPB_FATSz16

    // BPB_SecPerTrk and BPB_NumHeads are INT 13h geometry, which fatgen103
    // says matters only to media INT 13h sees; FatFs reads neither. 63
    // sectors by 255 heads is what FatFs R0.14b's own `f_mkfs` writes.
    bs[24..26].copy_from_slice(&63u16.to_le_bytes()); // BPB_SecPerTrk
    bs[26..28].copy_from_slice(&255u16.to_le_bytes()); // BPB_NumHeads

    // BS_DrvNum: 0x80 for a hard disk, 0x00 for a floppy.
    bs[36] = 0x80;
    // BS_BootSig: 0x29 says the three fields after it are present.
    bs[38] = 0x29;
    bs[39..43].copy_from_slice(&VOLUME_SERIAL.to_le_bytes()); // BS_VolID
    bs[43..54].copy_from_slice(VOLUME_LABEL); // BS_VolLab

    // BS_FilSysType: informational only. fatgen103 is explicit that the type
    // is never decided from this string; see `FAT16_MIN_CLUSTERS`.
    bs[54..62].copy_from_slice(b"FAT16   ");
    // The signature fatgen103 requires at offsets 510 and 511 of sector 0.
    bs[510] = 0x55;
    bs[511] = 0xAA;

    // -- the FATs, mirrored --------------------------------------------
    let fat_bytes: Vec<u8> = layout.fat.iter().flat_map(|e| e.to_le_bytes()).collect();
    for copy in 0..NUM_FATS {
        let at = (fat_start + copy * layout.sectors_per_fat) * SECTOR;
        layout.bytes[at..at + fat_bytes.len()].copy_from_slice(&fat_bytes);
    }

    // -- root directory -------------------------------------------------
    let at = root_start * SECTOR;
    layout.bytes[at..at + root_entries.len()].copy_from_slice(&root_entries);

    Ok(layout.bytes)
}

#[cfg(test)]
mod tests {
    use super::*;
    use rstest::rstest;

    /// 32 MiB at 2 KiB clusters lands mid-window, not near either edge where a
    /// small geometry change would silently reclassify the volume.
    #[test]
    fn a_32mib_card_is_unambiguously_fat16() {
        let img = build(32 * 1024 * 1024, &Dir::new()).expect("builds");
        assert_eq!(img.len(), 32 * 1024 * 1024);
        assert_eq!(&img[54..62], b"FAT16   ");
        assert_eq!(img[510], 0x55);
        assert_eq!(img[511], 0xAA);
        assert_eq!(u16::from_le_bytes([img[11], img[12]]), 512, "sector size");
    }

    #[test]
    fn a_size_outside_the_window_is_refused_rather_than_mislabelled() {
        // 1 MiB at 2 KiB clusters is ~500 clusters: FAT12 territory. Labelling
        // it FAT16 would produce an image every driver reads differently.
        let err = build(1024 * 1024, &Dir::new()).expect_err("must refuse");
        assert!(matches!(err, ImageError::NotFat16 { .. }), "got {err:?}");
    }

    /// FatFs classifies `nclst <= MAX_FAT12` (`0xFF5`) as FAT12, so it reads
    /// exactly 4085 clusters as FAT12 although fatgen103 calls that FAT16. A
    /// FAT16-structured image there mounts as the wrong thing, so it is
    /// refused. 16 405 sectors is where the sizing lands on 4085; 16 409 is
    /// where it first lands on 4086, and that image must build — and count
    /// 4086 clusters when read back the way `ff.c` counts them.
    #[test]
    fn a_4085_cluster_volume_is_refused_because_fatfs_reads_it_as_fat12() {
        // `.err()` rather than `expect_err`: an unexpected Ok would print the
        // whole 8 MiB image.
        assert_eq!(
            build(16_405 * 512, &Dir::new()).err(),
            Some(ImageError::NotFat16 { clusters: 4085 }),
            "4085 clusters is FAT12 to FatFs"
        );

        let img = build(16_409 * 512, &Dir::new()).expect("4086 clusters is FAT16 to both");
        let (reserved, fats, spf, root_entries) = bpb(&img);
        let total_sectors = u16::from_le_bytes([img[19], img[20]]) as usize;
        let system_sectors = reserved + fats * spf + root_entries * DIR_ENTRY / SECTOR;
        let clusters = (total_sectors - system_sectors) / img[13] as usize;
        assert_eq!(
            clusters, 4086,
            "the cluster count FatFs derives from the BPB"
        );
    }

    /// Names are upper-cased on the way to 8.3, so two host names can fold to
    /// one entry. Writing both would list one short name twice — a corrupt
    /// directory in which a driver only ever finds the first.
    #[rstest]
    #[case::two_files_differing_only_in_case(&["a.bin", "A.BIN"], &[], "a.bin")]
    #[case::a_file_and_a_directory_of_one_name(&["data"], &["data"], "data")]
    fn two_names_that_fold_to_one_8_3_entry_are_refused(
        #[case] files: &[&str],
        #[case] dirs: &[&str],
        #[case] second: &str,
    ) {
        let mut root = Dir::new();
        for name in files {
            root.file(name, vec![1]);
        }
        for name in dirs {
            root.dir(name);
        }
        assert_eq!(
            build(32 * 1024 * 1024, &root).err(),
            Some(ImageError::DuplicateName {
                name: second.to_string()
            })
        );
    }

    #[test]
    fn a_missing_host_directory_is_an_error_that_names_it() {
        let path = std::env::temp_dir().join(format!("embsim-fat16-{}-absent", std::process::id()));
        let err = Dir::from_host_dir(&path).expect_err("a missing directory is an error");
        assert_eq!(err.kind(), std::io::ErrorKind::NotFound);
        assert!(
            err.to_string().contains(&path.display().to_string()),
            "the error names the path: {err}"
        );
    }

    /// One file, one subdirectory and (on Unix) a symlink to the file, all of
    /// which must reach the card's root directory under their 8.3 names.
    #[test]
    fn a_host_directory_mirrors_onto_the_card() {
        let host = std::env::temp_dir().join(format!("embsim-fat16-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&host);
        std::fs::create_dir_all(host.join("gcode")).expect("make the subdirectory");
        std::fs::write(host.join("profile.bin"), [1, 2, 3]).expect("write the file");
        #[cfg(unix)]
        std::os::unix::fs::symlink(host.join("profile.bin"), host.join("link.bin"))
            .expect("make the symlink");
        let mirrored = Dir::from_host_dir(&host);
        // Clean up before asserting, so a failure leaves nothing behind.
        std::fs::remove_dir_all(&host).expect("remove the tree");

        let img = build(32 * 1024 * 1024, &mirrored.expect("the tree reads")).expect("builds");
        let root_start = find_root_offset(&img);
        let names: Vec<String> = img[root_start..root_start + 512 * 32]
            .chunks(32)
            .take_while(|e| e[0] != 0)
            .map(|e| String::from_utf8_lossy(&e[0..11]).to_string())
            .collect();
        assert!(names.iter().any(|n| n == "PROFILE BIN"), "got {names:?}");
        assert!(names.iter().any(|n| n == "GCODE      "), "got {names:?}");
        #[cfg(unix)]
        assert!(
            names.iter().any(|n| n == "LINK    BIN"),
            "a symlink to a file is followed: {names:?}"
        );
    }

    #[test]
    fn names_convert_to_8_3_and_bad_ones_are_refused() {
        assert_eq!(&short_name("profile.bin").unwrap(), b"PROFILE BIN");
        assert_eq!(&short_name("test").unwrap(), b"TEST       ");
        assert!(short_name("a-name-far-too-long.bin").is_err());
        assert!(short_name("name.long").is_err());
    }

    /// A subdirectory must exist as an entry AND open with `.` and `..`, or a
    /// FAT driver cannot walk into it.
    ///
    /// This is the failure worth guarding: firmware typically opens files
    /// *inside* a directory without creating the parent, so a card missing one
    /// mounts cleanly and then fails on first use — the mount succeeds, which
    /// is exactly what makes it hard to find.
    #[test]
    fn a_subdirectory_exists_and_opens_with_its_dot_entries() {
        let mut root = Dir::new();
        root.dir("test");
        root.dir("gcode");
        let img = build(32 * 1024 * 1024, &root).expect("builds");
        let root_start = find_root_offset(&img);
        let names: Vec<String> = img[root_start..root_start + 512 * 32]
            .chunks(32)
            .take_while(|e| e[0] != 0)
            .map(|e| String::from_utf8_lossy(&e[0..11]).to_string())
            .collect();
        assert!(names.iter().any(|n| n == "TEST       "), "got {names:?}");
        assert!(names.iter().any(|n| n == "GCODE      "), "got {names:?}");

        // Walk into TEST and check its dot entries.
        let entry = img[root_start..]
            .chunks(32)
            .find(|e| &e[0..11] == b"TEST       ")
            .expect("TEST entry");
        let cluster = u16::from_le_bytes([entry[26], entry[27]]);
        assert!(cluster >= 2, "a directory owns a cluster");
        let at = data_offset(&img, cluster);
        assert_eq!(&img[at..at + 11], b".          ");
        assert_eq!(&img[at + 32..at + 43], b"..         ");
    }

    #[test]
    fn a_file_round_trips_through_its_cluster_chain() {
        // Larger than one 2 KiB cluster, so the chain has to be followed.
        let payload: Vec<u8> = (0..5000u32).map(|i| (i % 251) as u8).collect();
        let mut root = Dir::new();
        root.file("profile.bin", payload.clone());
        let img = build(32 * 1024 * 1024, &root).expect("builds");

        let root_start = find_root_offset(&img);
        let entry = img[root_start..]
            .chunks(32)
            .find(|e| &e[0..11] == b"PROFILE BIN")
            .expect("PROFILE.BIN entry");
        assert_eq!(
            u32::from_le_bytes([entry[28], entry[29], entry[30], entry[31]]),
            payload.len() as u32
        );

        let mut cluster = u16::from_le_bytes([entry[26], entry[27]]);
        let mut got = Vec::new();
        while (FIRST_DATA_CLUSTER..0xFFF8).contains(&cluster) {
            let at = data_offset(&img, cluster);
            got.extend_from_slice(&img[at..at + 4 * 512]);
            cluster = fat_entry(&img, cluster);
        }
        got.truncate(payload.len());
        assert_eq!(got, payload, "the chain must reassemble the file");
    }

    /// Two builds of the same tree are byte-identical — no wall-clock
    /// timestamps, so a golden boot trace cannot depend on when it ran.
    #[test]
    fn the_image_is_reproducible() {
        let mut root = Dir::new();
        root.file("profile.bin", vec![1, 2, 3]);
        root.dir("gcode").file("a.bin", vec![4, 5]);
        let a = build(32 * 1024 * 1024, &root).expect("builds");
        let b = build(32 * 1024 * 1024, &root).expect("builds");
        assert_eq!(a, b);
    }

    // -- helpers that read the image back the way a driver would ---------

    fn bpb(img: &[u8]) -> (usize, usize, usize, usize) {
        let reserved = u16::from_le_bytes([img[14], img[15]]) as usize;
        let fats = img[16] as usize;
        let spf = u16::from_le_bytes([img[22], img[23]]) as usize;
        let root_entries = u16::from_le_bytes([img[17], img[18]]) as usize;
        (reserved, fats, spf, root_entries)
    }

    fn find_root_offset(img: &[u8]) -> usize {
        let (reserved, fats, spf, _) = bpb(img);
        (reserved + fats * spf) * SECTOR
    }

    fn data_offset(img: &[u8], cluster: u16) -> usize {
        let (reserved, fats, spf, root_entries) = bpb(img);
        let root_sectors = root_entries * DIR_ENTRY / SECTOR;
        let data_start = (reserved + fats * spf + root_sectors) * SECTOR;
        data_start + (cluster as usize - 2) * SECTORS_PER_CLUSTER * SECTOR
    }

    fn fat_entry(img: &[u8], cluster: u16) -> u16 {
        let (reserved, ..) = bpb(img);
        let at = reserved * SECTOR + cluster as usize * 2;
        u16::from_le_bytes([img[at], img[at + 1]])
    }
}
