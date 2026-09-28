# SPDX-License-Identifier: LGPL-2.1-or-later
# Copyright (c) 2026 Riley McCarthy
#
# Default configuration for p2-softmmu: the standalone qemu-system-p2.
#
# The P2 has one board. The boot flash is a component on embsim's nets, so
# this binary has no flash device. `CONFIG_P2_BOARD` is `default y` in
# hw/p2/Kconfig and depends on the target, so there is nothing to select
# here -- but meson requires the file to exist.
