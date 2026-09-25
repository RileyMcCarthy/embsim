//! The C surface of the node: the vtable QEMU's P2 target calls into, and the
//! handful of host-drive entry points `hostdrive.c` exports.
//!
//! Everything here is `#[cfg(qemu_linked)]`; without a QEMU tree the crate
//! builds a stub of the same shape whose every function is unreachable, because
//! [`crate::P2Qemu::with_boot_rom`] refuses before anything could call it.

use std::os::raw::{c_char, c_int, c_uint, c_void};

/// `target/p2/pinbus.h`'s `P2PinBusOps`, field for field. The order is the
/// ABI; a field added there must be added here in the same place.
#[repr(C)]
#[allow(missing_docs)] // documented in the C header it mirrors
pub struct P2PinBusOps {
    pub ina: unsafe extern "C" fn(*mut c_void) -> u32,
    pub inb: unsafe extern "C" fn(*mut c_void) -> u32,
    pub dir_out_changed: unsafe extern "C" fn(*mut c_void, c_uint, c_uint, u32),
    pub wrpin: unsafe extern "C" fn(*mut c_void, c_uint, u32),
    pub wxpin: unsafe extern "C" fn(*mut c_void, c_uint, u32),
    pub wypin: unsafe extern "C" fn(*mut c_void, c_uint, u32),
    pub pin_cfg: unsafe extern "C" fn(*mut c_void, c_uint) -> u32,
    pub rdpin: unsafe extern "C" fn(*mut c_void, c_uint, *mut bool) -> u32,
    pub testp: unsafe extern "C" fn(*mut c_void, c_uint) -> bool,
    pub akpin: unsafe extern "C" fn(*mut c_void, c_uint),
}

#[cfg(qemu_linked)]
mod linked {
    use super::*;

    /// `system/main.c` owns this, and that object is the ONE dropped from the
    /// link because it also defines `main()`. The UI backends reference it
    /// unconditionally, so something must define it; with `-display none`
    /// nothing ever calls it.
    ///
    /// Defined in Rust rather than in the C shim on purpose: the shim is an
    /// archive member, only pulled in to satisfy a symbol already undefined
    /// when the linker reaches it, and the UI objects come later. A Rust
    /// static is linked as a plain object and has no such ordering rule.
    #[no_mangle]
    pub static mut qemu_main: *const c_void = std::ptr::null();

    #[allow(missing_docs)] // documented in hostdrive.c
    extern "C" {
        pub fn p2host_boot(argc: c_int, argv: *mut *mut c_char);
        pub fn p2host_attach_thread();
        pub fn p2host_install_bus(ops: *const P2PinBusOps, opaque: *mut c_void);
        pub fn p2host_slice(cog: c_uint, budget: i64) -> c_int;
        pub fn p2host_cog_running(cog: c_uint) -> bool;
        pub fn p2host_cog_clocks(cog: c_uint) -> u64;
        pub fn p2host_cog_pc(cog: c_uint) -> u32;
        pub fn p2host_current_cog() -> c_uint;
        pub fn p2host_clock_mode() -> u32;
        pub fn p2host_clock_mode_at() -> u64;
        pub fn p2host_request_yield();
        pub fn p2host_take_yield() -> bool;
    }
}

#[cfg(qemu_linked)]
pub use linked::*;

/// The same names with no QEMU behind them. Reaching any of these is a bug:
/// the constructor reports the node unavailable before a bus could exist.
#[cfg(not(qemu_linked))]
mod stub {
    use super::*;

    const MSG: &str = "embsim-p2-qemu was built without a QEMU tree (EMBSIM_QEMU_P2_BUILD unset)";

    pub unsafe fn p2host_boot(_argc: c_int, _argv: *mut *mut c_char) {
        unreachable!("{MSG}")
    }
    pub unsafe fn p2host_attach_thread() {
        unreachable!("{MSG}")
    }
    pub unsafe fn p2host_install_bus(_ops: *const P2PinBusOps, _opaque: *mut c_void) {
        unreachable!("{MSG}")
    }
    pub unsafe fn p2host_slice(_cog: c_uint, _budget: i64) -> c_int {
        unreachable!("{MSG}")
    }
    pub unsafe fn p2host_cog_running(_cog: c_uint) -> bool {
        unreachable!("{MSG}")
    }
    pub unsafe fn p2host_cog_clocks(_cog: c_uint) -> u64 {
        unreachable!("{MSG}")
    }
    pub unsafe fn p2host_cog_pc(_cog: c_uint) -> u32 {
        unreachable!("{MSG}")
    }
    pub unsafe fn p2host_current_cog() -> c_uint {
        unreachable!("{MSG}")
    }
    pub unsafe fn p2host_clock_mode() -> u32 {
        unreachable!("{MSG}")
    }
    pub unsafe fn p2host_clock_mode_at() -> u64 {
        unreachable!("{MSG}")
    }
    pub unsafe fn p2host_request_yield() {
        unreachable!("{MSG}")
    }
    pub unsafe fn p2host_take_yield() -> bool {
        unreachable!("{MSG}")
    }
}

#[cfg(not(qemu_linked))]
pub use stub::*;

/// Whether this build carries QEMU at all.
pub const fn linked() -> bool {
    cfg!(qemu_linked)
}
