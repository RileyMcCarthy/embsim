/*
 * The host side of the seam: embsim's engine thread drives QEMU's cogs.
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Riley McCarthy
 *
 * QEMU is linked into the process as a library (build.rs replays its link
 * line minus main()). Its own vCPU thread parks at start-up (host-thread.patch)
 * and never executes a block; instead the engine thread registers itself with
 * RCU and TCG and runs the round-robin loop's own slice triple, one cog and
 * one bounded instruction budget at a time:
 *
 *      current_cpu = cpu;
 *      icount_prepare_for_run(cpu, budget);
 *      tcg_cpu_exec(cpu);
 *      icount_process_data(cpu);
 *
 * Spike 1d measured that at 172-254 ns a slice, exact to the instruction. It
 * is what makes a pin edge a function call rather than a cross-thread
 * park/wake (10.9 us), which is the whole reason the CPU lives on the engine
 * thread.
 *
 * Everything here is glue over public QEMU headers; the electrical model is on
 * the Rust side, reached through the P2PinBusOps vtable (target/p2/pinbus.h).
 */
#include "qemu/osdep.h"
#include "qemu/main-loop.h"
#include "qemu/rcu.h"
#include "tcg/startup.h"
#include "hw/core/cpu.h"
#include "system/system.h"
#include "system/replay.h"
#include "exec/icount.h"
#include "exec/cpu-common.h"
#include "accel/tcg/tcg-accel-ops.h"
#include "accel/tcg/tcg-accel-ops-icount.h"
#include "accel/tcg/tcg-accel-ops-rr.h"
#include "cpu.h"
#include "pinbus.h"

/*
 * src/ffi.rs mirrors P2PinBusOps field for field, and nothing else ties the
 * two together: a pointer added on this side would be read past the end of
 * the Rust struct. Ten function pointers is the shape both sides agree on.
 */
QEMU_BUILD_BUG_MSG(sizeof(P2PinBusOps) != 10 * sizeof(void (*)(void)),
                   "P2PinBusOps changed: update p2-qemu/src/ffi.rs to match, "
                   "field for field");

/*
 * Build the machine. Returns with the vCPU thread parked and both locks
 * qemu_init hands back (the BQL and the replay mutex) released, exactly as
 * system/main.c releases them before its loop.
 *
 * The flags are set BEFORE qemu_init: the round-robin thread reads
 * `rr_host_driven` when it starts (host-thread.patch), the board reads
 * `p2_host_driven` in its init, and `p2_pin_ops_end_tb` is read at translate
 * time, so all three must be in place before any thread or block exists.
 */
void p2host_boot(int argc, char **argv)
{
    rr_host_driven = true;
    p2_host_driven = true;
    p2_pin_ops_end_tb = true;
    qemu_init(argc, argv);
    bql_unlock();
    replay_mutex_unlock();
}

/* Make the CALLING thread the one TCG executes on. Once, on the engine thread,
 * before its first slice. */
void p2host_attach_thread(void)
{
    rcu_register_thread();
    tcg_register_thread();
}

void p2host_install_bus(const P2PinBusOps *ops, void *opaque)
{
    p2_pinbus_set(ops, opaque);
}

/*
 * One bounded slice of one cog, on the calling thread. Returns cpu_exec's
 * result, or -1 for a cog that does not exist. A halted cog returns at once.
 *
 * The BQL is NOT held around the icount bookkeeping. The round-robin loop
 * (accel/tcg/tcg-accel-ops-rr.c) drops it before icount_prepare_for_run and
 * takes it back after icount_process_data, and for a reason: when the budget
 * clamps to zero -- a virtual-clock deadline has expired -- prepare takes the
 * BQL ITSELF to run the timers, and the BQL is not recursive. This machine
 * inherits the parked vCPU thread's 100 ms kick timer, so a zero budget is a
 * matter of time, and the caller treats a slice that retired nothing as
 * ordinary.
 */
int p2host_slice(unsigned cog, int64_t budget)
{
    CPUState *cpu = qemu_get_cpu((int)cog);
    int r;

    if (!cpu) {
        return -1;
    }
    if (cpu->halted) {
        return EXCP_HALTED;
    }
    current_cpu = cpu;
    icount_prepare_for_run(cpu, budget);
    r = tcg_cpu_exec(cpu);
    icount_process_data(cpu);
    return r;
}

bool p2host_cog_running(unsigned cog)
{
    CPUState *cpu = qemu_get_cpu((int)cog);

    return cpu && cpu_env(cpu)->running && !cpu->halted;
}

uint64_t p2host_cog_clocks(unsigned cog)
{
    CPUState *cpu = qemu_get_cpu((int)cog);

    return cpu ? cpu_env(cpu)->clocks : 0;
}

uint32_t p2host_cog_pc(unsigned cog)
{
    CPUState *cpu = qemu_get_cpu((int)cog);

    return cpu ? cpu_env(cpu)->pc : 0;
}

/* The cog executing right now -- valid only inside a bus callback. */
unsigned p2host_current_cog(void)
{
    return current_cpu ? cpu_env(current_cpu)->cogid : 0;
}

/* The clock setting the guest last wrote with HUBSET (0 = none yet, RCFAST),
 * and the executing cog's clock count when it did. */
uint32_t p2host_clock_mode(void)
{
    return p2_clock_mode;
}

uint64_t p2host_clock_mode_at(void)
{
    return p2_clock_mode_at;
}

/*
 * From inside a bus callback: stop the executing cog after this instruction
 * and make cpu_exec return to the host. The target checks `p2_pinbus_yield`
 * at the same point in both engines; cpu_exit() is what turns "the block
 * ended" into "cpu_exec returned".
 */
void p2host_request_yield(void)
{
    p2_pinbus_yield = true;
    if (current_cpu) {
        cpu_exit(current_cpu);
    }
}

bool p2host_take_yield(void)
{
    bool y = p2_pinbus_yield;

    p2_pinbus_yield = false;
    return y;
}
