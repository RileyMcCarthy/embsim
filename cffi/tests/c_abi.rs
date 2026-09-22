//! The C ABI, exercised the way C will exercise it.
//!
//! These call the `extern "C"` entry points directly rather than the Rust model
//! behind them, so a mistake in the shim — a swapped argument, a handle freed
//! twice, a null path that dereferences — fails here rather than in a C host
//! where the symptom is a crash with no Rust in the backtrace.
//!
//! `c_link.rs` complements this by compiling and running actual C against the
//! static library: this file is the behaviour, that one is the linkage.

use embsim_cffi::*;

/// Shift one byte out, MSB first, presenting each bit before the rising edge —
/// the order a bit-banging master uses.
unsafe fn send(flash: *mut EmbsimSpiFlash, byte: u8) {
    for i in (0..8).rev() {
        let bit = (byte >> i) & 1 != 0;
        unsafe {
            embsim_spi_flash_clock(flash, false, bit);
            embsim_spi_flash_clock(flash, true, bit);
        }
    }
}

/// Clock a byte in, MSB first.
///
/// The sample goes AFTER the rising edge, not before it. The part does all its
/// work on that edge — it takes MOSI and then presents its own next bit — so a
/// master that reads the line before raising the clock reads the previous bit
/// and every byte comes back shifted. Getting this backwards produces plausible
/// garbage rather than an obvious failure, which is why it is worth stating.
unsafe fn recv(flash: *mut EmbsimSpiFlash) -> u8 {
    let mut byte = 0u8;
    for _ in 0..8 {
        unsafe {
            embsim_spi_flash_clock(flash, false, true);
            embsim_spi_flash_clock(flash, true, true);
            byte = (byte << 1) | u8::from(embsim_spi_flash_miso(flash));
        }
    }
    byte
}

#[test]
fn a_null_handle_reads_as_an_empty_socket_rather_than_crashing() {
    // The contract that keeps a C host which failed to construct a part from
    // taking the process down with it.
    unsafe {
        assert!(
            embsim_spi_flash_miso(std::ptr::null()),
            "nothing drives the line, so a pulled-up bus reads ones"
        );
        assert!(!embsim_spi_flash_present(std::ptr::null()));
        assert_eq!(
            embsim_spi_flash_image(std::ptr::null(), std::ptr::null_mut(), 0),
            0
        );
        assert_eq!(
            embsim_spi_flash_reads(std::ptr::null(), std::ptr::null_mut(), 0),
            0
        );
        // And the mutating ones are no-ops, not faults.
        embsim_spi_flash_set_selected(std::ptr::null_mut(), true);
        embsim_spi_flash_clock(std::ptr::null_mut(), true, true);
        embsim_spi_flash_free(std::ptr::null_mut());
    }
}

#[test]
fn an_image_shorter_than_the_part_leaves_the_rest_erased() {
    let image = [0xA5u8; 16];
    let flash = unsafe { embsim_spi_flash_with_image(1024, image.as_ptr(), image.len()) };
    assert_eq!(
        unsafe { embsim_spi_flash_image(flash, std::ptr::null_mut(), 0) },
        1024,
        "capacity is the part's, not the image's"
    );

    let mut out = vec![0u8; 1024];
    let n = unsafe { embsim_spi_flash_image(flash, out.as_mut_ptr(), out.len()) };
    assert_eq!(n, 1024);
    assert_eq!(&out[..16], &image[..], "the image landed at offset zero");
    assert!(
        out[16..].iter().all(|&b| b == 0xFF),
        "and everything past it is erased, as a real part is"
    );
    unsafe { embsim_spi_flash_free(flash) };
}

#[test]
fn a_read_over_the_c_abi_returns_the_bytes_and_records_where_it_looked() {
    // $03 READ DATA with a 24-bit address is what the P2's boot ROM issues.
    let image: Vec<u8> = (0..64u8).collect();
    let flash = unsafe { embsim_spi_flash_with_image(4096, image.as_ptr(), image.len()) };
    assert!(unsafe { embsim_spi_flash_present(flash) });

    unsafe {
        embsim_spi_flash_set_selected(flash, true);
        send(flash, 0x03);
        send(flash, 0x00);
        send(flash, 0x00);
        send(flash, 0x08); // address $000008
        let got: Vec<u8> = (0..8).map(|_| recv(flash)).collect();
        embsim_spi_flash_set_selected(flash, false);

        assert_eq!(
            got,
            vec![8, 9, 10, 11, 12, 13, 14, 15],
            "the part served the bytes at the address it was given"
        );

        let n = embsim_spi_flash_reads(flash, std::ptr::null_mut(), 0);
        assert_eq!(n, 1, "exactly one read");
        let mut looked = vec![0u32; n];
        embsim_spi_flash_reads(flash, looked.as_mut_ptr(), looked.len());
        assert_eq!(looked, vec![8], "and it looked where it was told to");

        embsim_spi_flash_free(flash);
    }
}

#[test]
fn a_deselected_part_ignores_the_bus() {
    let flash = embsim_spi_flash_blank(1024);
    unsafe {
        // Never selected: a whole command frame goes nowhere.
        send(flash, 0x03);
        send(flash, 0x00);
        send(flash, 0x00);
        send(flash, 0x00);
        assert_eq!(
            embsim_spi_flash_reads(flash, std::ptr::null_mut(), 0),
            0,
            "a part that was never addressed served nothing"
        );
        embsim_spi_flash_free(flash);
    }
}
