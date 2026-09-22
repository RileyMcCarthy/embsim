//! A C ABI over embsim's device models.
//!
//! The reason this exists is a rule, not a convenience: **there is one model of
//! each device, and it lives in Rust**. A C host that needs a SPI flash — the
//! QEMU Propeller 2 target is the first — links this rather than growing its
//! own copy, because a second implementation is a second set of bugs and the
//! differential tests that make the first one trustworthy do not cover it.
//!
//! # Why in-process, and not over the net engine
//!
//! embsim's board engine is the natural place to attach a device, and for a
//! peripheral-clocked bus it is the right one. A **CPU-bit-banged** bus is
//! different: a boot ROM drives a clock edge and samples the data line
//! microseconds later, far sooner than the engine resolves a net between wakes,
//! and the P2's boot ROM does roughly 16 600 edges to load one kilobyte. So the
//! transport here is a direct call — nanoseconds — and the *model* is still the
//! shared one. Generic model, in-process transport; those are separable choices
//! and this crate is where they separate.
//!
//! # Safety contract for every function here
//!
//! - A function that fills a caller's buffer returns **how many items it
//!   wrote**, never a total the buffer might not hold. Getting that backwards
//!   is how a C caller walks off the end of a fixed array while looking like
//!   it is doing the obvious thing; use the matching `_count`/`_capacity`
//!   accessor to size.
//! - Handles are opaque and must come from a `_new`/`_blank` in this module and
//!   be released with the matching `_free` exactly once.
//! - A null handle is tolerated and treated as "no device" rather than
//!   dereferenced, because a C host that failed to construct one should get a
//!   bus that reads as empty rather than a crash.
//! - **No panic crosses the boundary.** Unwinding into C is undefined
//!   behaviour, so every entry point catches and aborts with a message instead.
//!   An abort is a bad day; UB is a worse one that shows up somewhere else.

use std::panic::{catch_unwind, AssertUnwindSafe};

use embsim_models::spi_flash::SpiNorFlash;

/// Run `f`, and abort rather than let a panic unwind into C.
fn guard<T>(what: &str, f: impl FnOnce() -> T) -> T {
    match catch_unwind(AssertUnwindSafe(f)) {
        Ok(value) => value,
        Err(_) => {
            eprintln!("embsim-cffi: panic in {what}; aborting rather than unwinding into C");
            std::process::abort()
        }
    }
}

// ============================================================
// SPI NOR flash
// ============================================================

/// Opaque handle to a [`SpiNorFlash`].
pub struct EmbsimSpiFlash {
    inner: SpiNorFlash,
}

/// A blank part of `capacity` bytes — erased, so every byte reads `$FF`.
///
/// # Safety
/// The returned pointer must be released with [`embsim_spi_flash_free`].
#[no_mangle]
pub extern "C" fn embsim_spi_flash_blank(capacity: usize) -> *mut EmbsimSpiFlash {
    guard("embsim_spi_flash_blank", || {
        Box::into_raw(Box::new(EmbsimSpiFlash {
            inner: SpiNorFlash::blank(capacity),
        }))
    })
}

/// A part of `capacity` bytes preloaded with `len` bytes from `image` at offset
/// zero; the rest reads `$FF` as an erased array does.
///
/// Taking a capacity separately from the image length is deliberate: a boot
/// image is kilobytes and the part is megabytes, and a ROM that reads past the
/// image must see erased flash rather than the end of a short array.
///
/// # Safety
/// `image` must point to at least `len` readable bytes, or be null with
/// `len == 0`. The result must be released with [`embsim_spi_flash_free`].
#[no_mangle]
pub unsafe extern "C" fn embsim_spi_flash_with_image(
    capacity: usize,
    image: *const u8,
    len: usize,
) -> *mut EmbsimSpiFlash {
    guard("embsim_spi_flash_with_image", || {
        let mut array = vec![0xFFu8; capacity];
        if !image.is_null() && len > 0 {
            let end = len.min(capacity);
            // SAFETY: the caller promises `len` readable bytes at `image`.
            let src = unsafe { std::slice::from_raw_parts(image, len) };
            array[..end].copy_from_slice(&src[..end]);
        }
        Box::into_raw(Box::new(EmbsimSpiFlash {
            inner: SpiNorFlash::with_image(array),
        }))
    })
}

/// Release a handle. Null is a no-op.
///
/// # Safety
/// `flash` must have come from this module and not been freed already.
#[no_mangle]
pub unsafe extern "C" fn embsim_spi_flash_free(flash: *mut EmbsimSpiFlash) {
    if flash.is_null() {
        return;
    }
    guard("embsim_spi_flash_free", || {
        // SAFETY: the caller promises this came from `Box::into_raw` here.
        drop(unsafe { Box::from_raw(flash) })
    })
}

/// Drive chip select. `selected` is the ASSERTED sense — inverting an active-low
/// `~CS` is the caller's job, because active-low is a property of the wiring and
/// not of the part.
///
/// # Safety
/// `flash` must be a valid handle or null.
#[no_mangle]
pub unsafe extern "C" fn embsim_spi_flash_set_selected(flash: *mut EmbsimSpiFlash, selected: bool) {
    // SAFETY: the caller promises a valid handle; null is checked.
    let Some(f) = (unsafe { flash.as_mut() }) else {
        return;
    };
    guard("embsim_spi_flash_set_selected", || {
        f.inner.set_selected(selected)
    })
}

/// Present the clock at `clk_high` with `mosi` on the data line.
///
/// Idempotent in the level: calling it twice with the same clock moves nothing,
/// so a host may forward every pin write without tracking edges itself.
///
/// # Safety
/// `flash` must be a valid handle or null.
#[no_mangle]
pub unsafe extern "C" fn embsim_spi_flash_clock(
    flash: *mut EmbsimSpiFlash,
    clk_high: bool,
    mosi: bool,
) {
    // SAFETY: the caller promises a valid handle; null is checked.
    let Some(f) = (unsafe { flash.as_mut() }) else {
        return;
    };
    guard("embsim_spi_flash_clock", || f.inner.clock(clk_high, mosi))
}

/// The level the part is presenting on its data-out line.
///
/// A null handle reads `true`: no device fitted means nothing drives the line,
/// and a pulled-up bus reads as ones — which is how a master concludes the part
/// is absent rather than reading zeros and believing them.
///
/// # Safety
/// `flash` must be a valid handle or null.
#[no_mangle]
pub unsafe extern "C" fn embsim_spi_flash_miso(flash: *const EmbsimSpiFlash) -> bool {
    // SAFETY: the caller promises a valid handle; null is checked.
    let Some(f) = (unsafe { flash.as_ref() }) else {
        return true;
    };
    guard("embsim_spi_flash_miso", || f.inner.miso())
}

/// Whether an array is fitted at all.
///
/// # Safety
/// `flash` must be a valid handle or null.
#[no_mangle]
pub unsafe extern "C" fn embsim_spi_flash_present(flash: *const EmbsimSpiFlash) -> bool {
    // SAFETY: the caller promises a valid handle; null is checked.
    let Some(f) = (unsafe { flash.as_ref() }) else {
        return false;
    };
    guard("embsim_spi_flash_present", || f.inner.present())
}

/// The part's capacity in bytes — what [`embsim_spi_flash_image`] will yield.
///
/// # Safety
/// `flash` must be a valid handle or null.
#[no_mangle]
pub unsafe extern "C" fn embsim_spi_flash_capacity(flash: *const EmbsimSpiFlash) -> usize {
    // SAFETY: the caller promises a valid handle; null is checked.
    let Some(f) = (unsafe { flash.as_ref() }) else {
        return 0;
    };
    guard("embsim_spi_flash_capacity", || f.inner.capacity())
}

/// Copy up to `cap` bytes of the backing image into `out`, and return **how
/// many were copied**.
///
/// The count is what was WRITTEN, never a total the buffer might not hold —
/// see [`embsim_spi_flash_reads`] for why that distinction is load-bearing.
/// Use [`embsim_spi_flash_capacity`] to size a buffer.
///
/// # Safety
/// `out` must point to at least `cap` writable bytes, or be null with
/// `cap == 0`.
#[no_mangle]
pub unsafe extern "C" fn embsim_spi_flash_image(
    flash: *const EmbsimSpiFlash,
    out: *mut u8,
    cap: usize,
) -> usize {
    // SAFETY: the caller promises a valid handle; null is checked.
    let Some(f) = (unsafe { flash.as_ref() }) else {
        return 0;
    };
    guard("embsim_spi_flash_image", || {
        let image = f.inner.image_bytes();
        if out.is_null() || cap == 0 {
            return 0;
        }
        let n = cap.min(image.len());
        // SAFETY: the caller promises `cap` writable bytes at `out`.
        unsafe { std::ptr::copy_nonoverlapping(image.as_ptr(), out, n) };
        n
    })
}

/// How many reads the part has served.
///
/// # Safety
/// `flash` must be a valid handle or null.
#[no_mangle]
pub unsafe extern "C" fn embsim_spi_flash_read_count(flash: *const EmbsimSpiFlash) -> usize {
    // SAFETY: the caller promises a valid handle; null is checked.
    let Some(f) = (unsafe { flash.as_ref() }) else {
        return 0;
    };
    guard("embsim_spi_flash_read_count", || f.inner.reads.len())
}

/// Copy up to `cap` read start-addresses into `out`, beginning at index
/// `start`, and return **how many were copied**.
///
/// This is the cheapest way for a host test to say *where a boot actually
/// looked*, which is a much sharper assertion than whether it finished. The
/// `start` index is what lets a caller drain new entries as they appear
/// without re-reading the whole list.
///
/// # Why the return is the COPIED count and not the total
///
/// It used to be the total, so a caller could size a buffer with `cap == 0`.
/// That reads fine and is a trap: the obvious loop takes the return value as
/// the number of valid entries in `out` and walks off the end of a fixed
/// buffer the moment more reads exist than it holds. The QEMU pin bus did
/// exactly that, and it was invisible because a ROM boot serves two reads —
/// it would have fired on the first firmware that paged from flash. Use
/// [`embsim_spi_flash_read_count`] to size, this to fill, and the two can no
/// longer be confused.
///
/// # Safety
/// `out` must point to at least `cap` writable `uint32_t`s, or be null with
/// `cap == 0`.
#[no_mangle]
pub unsafe extern "C" fn embsim_spi_flash_reads(
    flash: *const EmbsimSpiFlash,
    start: usize,
    out: *mut u32,
    cap: usize,
) -> usize {
    // SAFETY: the caller promises a valid handle; null is checked.
    let Some(f) = (unsafe { flash.as_ref() }) else {
        return 0;
    };
    guard("embsim_spi_flash_reads", || {
        let reads = &f.inner.reads;
        if out.is_null() || cap == 0 || start >= reads.len() {
            return 0;
        }
        let n = cap.min(reads.len() - start);
        // SAFETY: the caller promises `cap` writable u32s at `out`, and `n` is
        // bounded by both `cap` and what remains after `start`.
        unsafe { std::ptr::copy_nonoverlapping(reads[start..].as_ptr(), out, n) };
        n
    })
}
