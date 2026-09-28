# SPDX-License-Identifier: LGPL-2.1-or-later
# Copyright (c) 2026 Riley McCarthy
#
# Default configuration for p2-softmmu: the standalone qemu-system-p2.
#
# The P2 has one board, and its one optional device is the standalone flash
# bus. `CONFIG_P2_BOARD` and `CONFIG_P2_EMBSIM_FLASH` are both `default y` in
# hw/p2/Kconfig and depend on the target, so there is nothing to select here --
# but meson requires the file to exist.
