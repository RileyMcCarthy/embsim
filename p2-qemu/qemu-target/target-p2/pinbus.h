/*
 * The CPU's only outward surface.
 * SPDX-License-Identifier: LGPL-2.1-or-later
 *
 * This mirrors p2core's `PinBus` trait (SIL/p2core/src/pins.rs) deliberately:
 * the CPU forwards what the firmware executed and nothing electrical lives on
 * this side of the line. Smart-pin state machines, nets and UART peers belong
 * to the implementation behind these ops, which is how embsim will eventually
 * plug in without the target changing.
 *
 * Polarity, from p2core's trait documentation -- both of these hang the guest
 * if inverted:
 *   - rdpin()'s `busy` becomes C, and C means BUSY, not ready.
 *     `__system___txraw` spins on `rdpin #62 wc` / `if_b jmp`.
 *   - testp() reports the pin's IN flag, and drivers spin with `if_nc jmp`
 *     waiting for it, so it must read true once an operation has completed.
 *
 * The bus is MACHINE state, not CPU state: all eight cogs share one, exactly
 * as p2core has one `pins` field on `Machine`. It is therefore reached through
 * a process-wide pointer rather than through CPUP2State.
 */
#ifndef P2_PINBUS_H
#define P2_PINBUS_H

typedef struct P2PinBusOps {
    /* Pin input states 0..31 (INA) and 32..63 (INB). */
    uint32_t (*ina)(void *opaque);
    uint32_t (*inb)(void *opaque);
    /*
     * A cog wrote DIRA/DIRB/OUTA/OUTB. `cog` matters: these are PER-COG
     * registers and the pad sees the OR across all eight, so a bus that
     * mirrors them globally lets one cog's write erase another's.
     */
    void (*dir_out_changed)(void *opaque, unsigned cog, unsigned reg,
                            uint32_t value);
    void (*wrpin)(void *opaque, unsigned pin, uint32_t cfg);
    void (*wxpin)(void *opaque, unsigned pin, uint32_t x);
    void (*wypin)(void *opaque, unsigned pin, uint32_t y);
    /* Last WRPIN mode word, or 0 if never configured. */
    uint32_t (*pin_cfg)(void *opaque, unsigned pin);
    /* RDPIN/RQPIN. Returns the value; *busy becomes C. */
    uint32_t (*rdpin)(void *opaque, unsigned pin, bool *busy);
    /* TESTP -- sample the IN flag without consuming it. */
    bool (*testp)(void *opaque, unsigned pin);
    /* AKPIN -- acknowledge, clearing the IN flag. */
    void (*akpin)(void *opaque, unsigned pin);
} P2PinBusOps;

extern const P2PinBusOps *p2_pinbus_ops;
extern void *p2_pinbus_opaque;

void p2_pinbus_set(const P2PinBusOps *ops, void *opaque);

/*
 * A bus that lifts pins onto a discrete-event net sets this from inside a
 * pin op: "stop this cog after the current instruction". The CPU drove a pin
 * whose net another component may answer, and it must not read anything back
 * until that answer has resolved -- which happens outside the CPU, on the
 * host's schedule. The interpreter checks it after every instruction; a
 * translated block ends after every pin op when `p2_pin_ops_end_tb` is set,
 * so the check lands at the same place in both engines. The bus also calls
 * cpu_exit() so cpu_exec returns; the host clears the flag when it resumes.
 *
 * A self-contained bus (the bring-up model, the C flash bus) never sets it,
 * and pays nothing.
 */
extern bool p2_pinbus_yield;

/*
 * Read at TRANSLATE time: end a translated block after every instruction that
 * can change a pad (the DIR/OUT/FLT/DRV family, a store to DIRx/OUTx, WRPIN).
 * Set by a host that drives the CPU slice by slice BEFORE the first block is
 * translated, and never changed after. Spike 0d measured a block end at
 * ~53 ns, which is why it is not the default: the standalone emulator has no
 * net to yield to.
 */
extern bool p2_pin_ops_end_tb;

/*
 * A host outside QEMU's main loop drives the vCPUs (see embsim's
 * `embsim-p2-qemu`). The board then arms no quantum timer: the host does the
 * slicing, and a virtual-clock deadline that nothing services would cap every
 * icount budget at zero.
 */
extern bool p2_host_driven;

/*
 * The chip's clock setting, as the guest last wrote it with HUBSET
 * (`%0000_000E_DDDD_DDMM_MMMM_MMMM_PPPP_CC_SS`), and the executing cog's
 * clock count at that instruction. Zero until the guest sets one: the chip
 * comes up on RCFAST. Neither engine derives anything from it -- a cog's
 * `clocks` counts instructions regardless of frequency -- but a host that
 * places pin edges on a nanosecond timeline needs the frequency those clocks
 * tick at, and this word is where the guest says so.
 */
extern uint32_t p2_clock_mode;
extern uint64_t p2_clock_mode_at;
/* Install the bring-up model -- see pinbus.c. */
void p2_pinbus_bringup_init(void);

/*
 * Install the board model with the boot flash on it -- see flashbus.c. The
 * part is `capacity` bytes with `len` bytes of `image` at offset zero; the
 * rest reads $FF, as an erased array does.
 */
void p2_flashbus_init(const uint8_t *image, size_t len, size_t capacity);
/* What the guest has written to the debug pin since reset. */
const char *p2_flashbus_console(void);
/* Read start-addresses the flash has served, oldest first; returns the count. */
size_t p2_flashbus_reads(uint32_t *out, size_t cap);

#endif
