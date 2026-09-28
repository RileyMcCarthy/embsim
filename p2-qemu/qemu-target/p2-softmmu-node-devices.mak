# SPDX-License-Identifier: LGPL-2.1-or-later
# Copyright (c) 2026 Riley McCarthy
#
# embsim-p2-qemu's configuration for p2-softmmu, selected with
# `--with-devices-p2=node`.
#
# The node's flash is a component on the board's nets, so the standalone flash
# bus -- and the embsim-cffi archive it links -- stays out of the tree.
CONFIG_P2_EMBSIM_FLASH=n
