/*
 * embsim device models, for a C host.
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 *
 * There is ONE model of each device and it lives in Rust. A C host links this
 * rather than growing its own copy: a second implementation is a second set of
 * bugs, and the differential tests that make the first one trustworthy do not
 * cover it.
 *
 * Link against `libembsim_cffi.a`. It is a Rust staticlib, so the final link
 * also needs the platform's usual C runtime bits -- on macOS `-framework
 * CoreFoundation -lSystem`, on Linux `-lpthread -ldl -lm`.
 *
 * Every function tolerates a NULL handle and treats it as "no device fitted"
 * rather than dereferencing it, so a host that failed to construct one gets a
 * bus that reads empty instead of a crash. No panic crosses this boundary: the
 * Rust side catches and aborts, because unwinding into C is undefined
 * behaviour.
 */
#ifndef EMBSIM_H
#define EMBSIM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------- *
 * SPI NOR flash
 *
 * Bit-level and bus-agnostic: anything that can produce a chip select, a clock
 * edge and a data bit can talk to it, bit-banged or peripheral-clocked.
 * ---------------------------------------------------------------- */

typedef struct EmbsimSpiFlash EmbsimSpiFlash;

/* A blank part of `capacity` bytes: erased, so every byte reads $FF. */
EmbsimSpiFlash *embsim_spi_flash_blank(size_t capacity);

/*
 * A part of `capacity` bytes preloaded with `len` bytes from `image` at offset
 * zero; the rest reads $FF. Capacity is separate from image length on purpose:
 * a boot image is kilobytes and the part is megabytes, and a ROM reading past
 * the image must see erased flash rather than the end of a short array.
 */
EmbsimSpiFlash *embsim_spi_flash_with_image(size_t capacity, const uint8_t *image,
                                            size_t len);

/* Release a handle. NULL is a no-op. */
void embsim_spi_flash_free(EmbsimSpiFlash *flash);

/*
 * Drive chip select. `selected` is the ASSERTED sense -- inverting an
 * active-low ~CS is the caller's job, because active-low is a property of the
 * wiring and not of the part.
 */
void embsim_spi_flash_set_selected(EmbsimSpiFlash *flash, bool selected);

/*
 * Present the clock at `clk_high` with `mosi` on the data line. Idempotent in
 * the level, so a host may forward every pin write without tracking edges.
 */
void embsim_spi_flash_clock(EmbsimSpiFlash *flash, bool clk_high, bool mosi);

/*
 * The level the part presents on data-out. A NULL handle reads TRUE: no device
 * means nothing drives the line, and a pulled-up bus reads as ones -- which is
 * how a master concludes the part is absent instead of reading zeros and
 * believing them.
 */
bool embsim_spi_flash_miso(const EmbsimSpiFlash *flash);

/* Whether an array is fitted at all. */
bool embsim_spi_flash_present(const EmbsimSpiFlash *flash);

/*
 * Copy up to `cap` bytes of the backing image into `out`; returns the part's
 * full capacity, so passing cap == 0 sizes a buffer.
 */
size_t embsim_spi_flash_image(const EmbsimSpiFlash *flash, uint8_t *out, size_t cap);

/*
 * Copy up to `cap` read start-addresses into `out`, oldest first; returns how
 * many the part has served. The cheapest way for a test to say WHERE a boot
 * looked, which is a sharper assertion than whether it finished.
 */
size_t embsim_spi_flash_reads(const EmbsimSpiFlash *flash, uint32_t *out, size_t cap);

#ifdef __cplusplus
}
#endif

#endif /* EMBSIM_H */
