/*
 * The standalone emulator's flash bus -- see flashbus.c.
 * SPDX-License-Identifier: LGPL-2.1-or-later
 * Copyright (c) 2026 Riley McCarthy
 *
 * Only in a build with CONFIG_P2_EMBSIM_FLASH (Kconfig).
 */
#ifndef HW_P2_FLASHBUS_H
#define HW_P2_FLASHBUS_H

/*
 * Install the board model with the boot flash on it. The part is `capacity`
 * bytes with `len` bytes of `image` at offset zero; the rest reads $FF, as an
 * erased array does.
 */
void p2_flashbus_init(const uint8_t *image, size_t len, size_t capacity);

#endif
