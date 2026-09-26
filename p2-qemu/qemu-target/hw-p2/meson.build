# SPDX-License-Identifier: LGPL-2.1-or-later
# Copyright (c) 2026 Riley McCarthy

p2_ss = ss.source_set()
p2_ss.add(when: 'CONFIG_P2_BOARD', if_true: files('p2_board.c'))
p2_ss.add(when: 'CONFIG_P2_EMBSIM_FLASH', if_true: files('flashbus.c'))
hw_arch += {'p2': p2_ss}
