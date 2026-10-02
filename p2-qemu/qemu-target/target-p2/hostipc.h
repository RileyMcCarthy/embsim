/*
 * Host-driven mode: qemu-system-p2 run by embsim, out of process.
 * SPDX-License-Identifier: LGPL-2.1-or-later
 * Copyright (c) 2026 Riley McCarthy
 *
 * The machine property `hostipc` turns it on (hw/p2/p2_board.c):
 *
 *     -M p2,hostipc=<transport>:<channel-fd>:<watch-fd>[:<spin-ns>]
 *
 * `transport` is `shm` (a shared page embsim made and handed down as
 * `channel-fd`) or `sock` (one end of a socket pair). `watch-fd` is the read
 * end of a pipe only embsim holds the write end of: when it reads end of file
 * embsim is gone, and the process exits at once. `spin-ns` is how long the
 * shared-page wait spins before it blocks (20 000 when absent).
 *
 * hostipc.c says what crosses the channel and why.
 */
#ifndef P2_HOSTIPC_H
#define P2_HOSTIPC_H

#include "qapi/error.h"

/*
 * Take the `hostipc` spec. Called while the machine's options are applied,
 * before the board initialises and before any vCPU thread exists, which is
 * where the three host-driven flags must be set (pinbus.h).
 */
void p2_hostipc_configure(const char *spec, Error **errp);

#endif
