//! Filesystem — Mount/unmount stubs for SD card or local filesystem.
//!
//! The configured mount path lives in a per-MCU [`Filesystem`] owned by
//! `instance::PeripheralInstance`. The module-level free functions route to
//! the calling thread's instance (see `crate::instance`), so existing
//! single-MCU consumers are unaffected.

use std::ffi::CStr;
use std::fs;
use std::sync::OnceLock;
use tracing::info;

/// Filesystem mount configuration for one MCU instance.
pub struct Filesystem {
    /// Filesystem mount path (set from project config, once per instance).
    path: OnceLock<String>,
}

impl Filesystem {
    /// Create an unconfigured filesystem (no path set).
    pub const fn new() -> Self {
        Self {
            path: OnceLock::new(),
        }
    }

    /// Initialize the filesystem path (first call per instance wins).
    pub fn init(&self, path: &str) {
        self.path.get_or_init(|| path.to_string());
        let _ = fs::create_dir_all(path);
        info!("Filesystem path: {}", path);
    }

    /// The configured mount path, if `init` has run for this instance.
    pub fn path(&self) -> Option<&str> {
        self.path.get().map(String::as_str)
    }
}

impl Default for Filesystem {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================
// Free functions — route to the calling thread's instance
// ============================================================

/// Initialize the filesystem path.
pub fn init(path: &str) {
    crate::instance::current().filesystem.init(path);
}

/// Mount a filesystem path (create directory).
///
/// # Safety
/// `user_name` must be a valid C string pointer.
pub unsafe fn mount(user_name: *const std::ffi::c_char) -> i32 {
    if user_name.is_null() {
        return -1;
    }
    let name = CStr::from_ptr(user_name).to_string_lossy();
    info!("mount(\"{}\")", name);
    match fs::create_dir_all(name.as_ref()) {
        Ok(_) => 0,
        Err(e) => {
            tracing::error!("mount failed: {}", e);
            -1
        }
    }
}

/// Unmount a filesystem path (no-op).
///
/// # Safety
/// `user_name` must be a valid C string pointer.
pub unsafe fn umount(user_name: *const std::ffi::c_char) -> i32 {
    if user_name.is_null() {
        return -1;
    }
    let name = CStr::from_ptr(user_name).to_string_lossy();
    info!("umount(\"{}\")", name);
    0
}

/// Open VFS (no-op in emulation).
pub fn vfs_open_sdcard() -> *mut std::ffi::c_void {
    std::ptr::null_mut()
}

// ============================================================
// Tests
// ============================================================

#[cfg(test)]
mod tests {
    use super::*;
    use std::ffi::CString;

    /// Build a unique temp path under the OS temp dir without creating it.
    /// A monotonic counter keeps paths distinct across tests in one process.
    fn temp_path(tag: &str) -> std::path::PathBuf {
        use std::sync::atomic::{AtomicU64, Ordering};
        static N: AtomicU64 = AtomicU64::new(0);
        let n = N.fetch_add(1, Ordering::Relaxed);
        std::env::temp_dir().join(format!("embsim_fs_{}_{}_{}", tag, std::process::id(), n))
    }

    #[test]
    fn init_creates_the_directory() {
        let _g = crate::test_support::guard();
        crate::test_support::ensure_clock();
        let dir = temp_path("init");
        assert!(!dir.exists());
        // The default instance's path is once-per-process; first init wins. We
        // assert the directory-creation side effect, NOT that the path changed.
        init(dir.to_str().unwrap());
        assert!(dir.exists() && dir.is_dir());
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn per_instance_path_is_independent() {
        let _g = crate::test_support::guard();
        crate::test_support::ensure_clock();
        // Two instances each latch their own path — no process-global winner.
        let a = Filesystem::new();
        let b = Filesystem::new();
        let dir_a = temp_path("inst_a");
        let dir_b = temp_path("inst_b");
        a.init(dir_a.to_str().unwrap());
        b.init(dir_b.to_str().unwrap());
        assert_eq!(a.path(), dir_a.to_str());
        assert_eq!(b.path(), dir_b.to_str());
        let _ = std::fs::remove_dir_all(&dir_a);
        let _ = std::fs::remove_dir_all(&dir_b);
    }

    #[test]
    fn mount_valid_path_returns_zero_and_creates_dir() {
        let _g = crate::test_support::guard();
        crate::test_support::ensure_clock();
        let dir = temp_path("mount");
        let cpath = CString::new(dir.to_str().unwrap()).unwrap();
        assert!(!dir.exists());
        let rc = unsafe { mount(cpath.as_ptr()) };
        assert_eq!(rc, 0, "mount of a valid path succeeds");
        assert!(dir.exists() && dir.is_dir(), "mount creates the directory");
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn mount_null_returns_minus_one() {
        let _g = crate::test_support::guard();
        crate::test_support::ensure_clock();
        // A null C-string pointer is rejected.
        let rc = unsafe { mount(std::ptr::null()) };
        assert_eq!(rc, -1);
    }

    #[test]
    fn umount_valid_returns_zero() {
        let _g = crate::test_support::guard();
        crate::test_support::ensure_clock();
        let dir = temp_path("umount");
        let cpath = CString::new(dir.to_str().unwrap()).unwrap();
        // umount is a no-op that simply reports success for a valid name.
        let rc = unsafe { umount(cpath.as_ptr()) };
        assert_eq!(rc, 0);
    }

    #[test]
    fn umount_null_returns_minus_one() {
        let _g = crate::test_support::guard();
        crate::test_support::ensure_clock();
        let rc = unsafe { umount(std::ptr::null()) };
        assert_eq!(rc, -1);
    }

    #[test]
    fn vfs_open_sdcard_returns_null() {
        let _g = crate::test_support::guard();
        crate::test_support::ensure_clock();
        // Emulation has no real VFS handle.
        assert!(vfs_open_sdcard().is_null());
    }
}
