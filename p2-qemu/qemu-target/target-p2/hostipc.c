/*
 * Host-driven mode: qemu-system-p2 as embsim's P2 core, out of process.
 * SPDX-License-Identifier: LGPL-2.1-or-later
 * Copyright (c) 2026 Riley McCarthy
 *
 * Inert unless the machine property `hostipc` names a channel (hostipc.h).
 * With it, the round-robin vCPU thread parks (host-thread.patch) and a thread
 * of this file registers with RCU and TCG and runs the cogs itself, one
 * bounded slice at a time, as embsim asks. embsim and this process take turns
 * in lockstep: one message pair crosses per STOP.
 *
 *   embsim -> qemu   RUN: run until cog clock H, with these net levels
 *                    (in_ext), these pads reading their own OUT (strong),
 *                    and these banks powered.
 *   qemu -> embsim   STOP: why (a pad's drive changed / H reached / the clock
 *                    word changed / nothing ran / a cog stalled / the console
 *                    tap filled), the stop instant in clocks, the new
 *                    DIR/OUT, mode words that changed, WYPIN bytes.
 *
 * WHAT LIVES HERE, ONCE: the guest-facing half of the pin bus. Per-cog
 * DIR/OUT ORed at the pad, the WRPIN mode words, the IN flags, the
 * TESTP/RDPIN/WYPIN/AKPIN rules, and the drive key that decides whether a
 * write changed what a pad presents. Every read the guest makes inside a
 * slice is answered here, from the snapshot the RUN carried: a slice ends at
 * the first instruction that changes what the chip drives, so nothing the
 * chip reads can change before the slice ends (the nets move only when embsim
 * resolves them, and embsim resolves only between slices).
 *
 * WHAT LIVES IN EMBSIM: the electrical model. The Thevenin drive a pad
 * presents (its bank's supply, the HHH/LLL impedance), the bank supplies, the
 * nets, and the clock: the HUBSET word is decoded there against the crystal
 * the board puts on XI. This side decides only WHETHER a pad's drive changed,
 * by a key that is equal exactly when embsim's drive is equal, given the bank
 * states embsim sends with each RUN (drive_key).
 *
 * THE PROCESS: it lives exactly as long as embsim wants it. embsim holds the
 * write end of a pipe whose read end is `watch-fd`; a thread here blocks on
 * it, and end of file -- embsim exited, crashed or was killed -- ends this
 * process at once. A QUIT, the socket closing, or SIGTERM/SIGINT/SIGHUP end
 * it at once too: QEMU's own shutdown waits for vCPUs that are parked in this
 * mode, so it is never asked.
 *
 * DETERMINISM: no wall-clock input reaches the guest. The budget of a slice
 * is the instruction count asked for, bounded by virtual-clock deadlines
 * only: host-thread.patch keeps real-time timers out of the icount limit in
 * this mode (icount_get_limit), and the board arms no quantum timer.
 *
 * The wire format is mirrored field for field by embsim-p2-qemu's
 * src/protocol.rs; P2IPC_PROTOCOL changes whenever either moves, and the
 * handshake refuses a peer of another protocol or another target.
 */
#include "qemu/osdep.h"
#include "qemu/main-loop.h"
#include "qemu/rcu.h"
#include "qemu/thread.h"
#include "qemu/notify.h"
#include "qemu/timer.h"
#include "qemu/processor.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "tcg/startup.h"
#include "hw/core/cpu.h"
#include "system/system.h"
#include "system/runstate.h"
#include "exec/icount.h"
#include "exec/cpu-common.h"
#include "accel/tcg/tcg-accel-ops.h"
#include "accel/tcg/tcg-accel-ops-icount.h"
#include "accel/tcg/tcg-accel-ops-rr.h"
#include "cpu.h"
#include "pinbus.h"
#include "hostipc.h"
#include <stdatomic.h>
#include <signal.h>
#include <sys/mman.h>

/*
 * stage.sh writes this header: the identity of the target sources this binary
 * was built from, which the handshake reports and embsim checks against its
 * own copy of them.
 */
#if __has_include("hostipc-identity.h")
#include "hostipc-identity.h"
#else
#error "hostipc-identity.h is missing: stage the target with p2-qemu/qemu-target/stage.sh"
#endif

/* ---- the wire format (embsim-p2-qemu src/protocol.rs mirrors it) -------- */

#define P2IPC_MAGIC     0x50324950u /* "PI2P" */
#define P2IPC_PROTOCOL  1u

/*
 * The shared page. Explicit offsets, so embsim mirrors them without a shared
 * header; each handoff word on a 128-byte line of its own (an M2's line).
 */
#define SHM_MAGIC       0       /* u32, embsim writes it before the spawn */
#define SHM_HELLO       64      /* P2IpcHello, written here */
#define SHM_READY       256     /* u32: 1 once the hello is in place */
#define SHM_REQ_SEQ     384     /* u32: embsim bumps it per request */
#define SHM_REQ_SLEEP   512     /* u32: this side is blocked on REQ_SEQ */
#define SHM_REP_SEQ     640     /* u32: the request seq a reply answers */
#define SHM_REP_SLEEP   768     /* u32: embsim is blocked on REP_SEQ */
#define SHM_REQ         1024    /* P2IpcRun */
#define SHM_REP         2048    /* P2IpcStop and its tail */
#define SHM_SIZE        16384
#define REP_CAP         (SHM_SIZE - SHM_REP)

enum { OP_RUN = 1, OP_QUIT = 2 };

enum {
    R_YIELD   = 1,   /* a pad's drive changed: publish at pending_at */
    R_HORIZON = 2,   /* every running cog reached H */
    R_CLOCK   = 4,   /* the HUBSET clock word changed */
    R_NOSTEP  = 8,   /* no cog could run */
    R_STALL   = 16,  /* a running cog retired nothing for 1000 slices */
    R_CONSOLE = 32,  /* the WYPIN tap is nearly full: come back */
};

typedef struct {
    uint32_t magic;
    uint32_t protocol;
    uint32_t pid;
    uint32_t reserved;
    char target[32];            /* the target identity, NUL-padded */
    char qemu[32];              /* QEMU_VERSION, NUL-padded */
    uint8_t pad[48];
} P2IpcHello;                       /* 128 bytes */

typedef struct {
    uint32_t op;
    uint32_t start_cog;
    uint64_t horizon_clocks;
    uint32_t in_ext[2];
    uint32_t strong[2];
    uint32_t banks_powered;     /* bit b: bank b's supply names a voltage */
    uint32_t banks_high;        /* bit b: ... and that voltage is not 0 V */
    uint8_t reserved[24];
} P2IpcRun;                         /* 64 bytes */

typedef struct {
    uint32_t reason;
    uint32_t n_mode;
    uint32_t n_console;
    uint32_t clock_mode;
    uint64_t clock_mode_at;
    uint64_t pending_at_clocks;
    uint64_t now_clocks;
    uint64_t dirty;
    uint32_t dir[2];
    uint32_t out[2];
    uint32_t slices;
    uint32_t any_running;
    uint32_t last_cog;
    uint32_t run_ns;            /* wall time this side spent in the run */
} P2IpcStop;                        /* 80 bytes, then n_mode x {u32 pin,
                                       u32 cfg}, then n_console x {u8 pin,
                                       u8 byte} */

QEMU_BUILD_BUG_ON(sizeof(P2IpcHello) != 128);
QEMU_BUILD_BUG_ON(sizeof(P2IpcRun) != 64);
QEMU_BUILD_BUG_ON(sizeof(P2IpcStop) != 80);

#define NUM_COGS 8
#define NUM_PINS 64

/* The WYPIN tap's room in one reply, and where a run stops to empty it: a
 * slice is at most one quantum of instructions, so the margin is never
 * reached inside one. */
#define MAX_CONSOLE ((REP_CAP - sizeof(P2IpcStop) - NUM_PINS * 8) / 2)
#define CONSOLE_STOP (MAX_CONSOLE - 1024)

/* ---- the guest-facing half of the bus ------------------------------------ */

#define REG_DIRA 0x1FA
#define REG_DIRB 0x1FB
#define REG_OUTA 0x1FC
#define REG_OUTB 0x1FD
/* Below it a cog runs interpreted; a cog-space slice takes a budget of one
 * and still retires up to the interpreter's run. */
#define HUB_EXEC_BASE 0x400
/* Instructions one cog runs before the next gets a turn: round-robin TCG's
 * own quantum, and what spike 0c measured the firmware tolerates. */
#define COG_QUANTUM 48
/* Slices in a row that retire nothing before the run gives embsim a turn. */
#define STALL_SLICES 1000

/* Smart-pin mode field values the bus interprets: bits 5..1 of the WRPIN
 * word, above the %0 in bit 0. */
#define SMART_ASYNC_TX 0x1E
#define SMART_ASYNC_RX 0x1F
#define SMART_SYNC_RX  0x1D
/* Pin-configuration field values $10..$17 in the high byte select the ADC
 * modes (P_ADC_GIO .. P_ADC_100X). */
#define PIN_CFG_ADC_MASK 0x00F80000u
#define PIN_CFG_ADC      0x00100000u
/* A DAC/ADC/comparator mode selector: the low bits are not drive modes, and
 * embsim's decode reads such a pad as fast both ways. */
#define P_MODE_SELECT_MASK 0x001E0000u
#define P_HIGH_SHIFT 11
#define P_LOW_SHIFT 8

typedef struct {
    /* DIRx/OUTx are PER-COG registers and the pad sees the OR across all
     * eight: one cog's write must not erase another's. */
    uint32_t dir_cog[NUM_COGS][2], out_cog[NUM_COGS][2];
    uint32_t dir[2], out[2];
    /* The snapshot of the outside the current RUN carried. */
    uint32_t in_ext[2], strong[2];
    uint32_t banks_powered, banks_high;
    uint32_t mode[NUM_PINS], x[NUM_PINS];
    bool in_flag[NUM_PINS];
    /* The drive key of what embsim last published on each pad. */
    uint8_t pub_key[NUM_PINS];
    /* Pads whose key differs from what was published, and the instant (the
     * changing cog's clock) of the first change since the last report. */
    uint64_t dirty;
    bool pending;
    uint64_t pending_at_clocks;
    uint64_t mode_changed;
    uint32_t reported_clock_mode;
    uint8_t console[MAX_CONSOLE][2];
    uint32_t n_console;
} IpcBus;

static IpcBus ib;

static inline uint32_t smart_mode(uint32_t cfg)
{
    return (cfg >> 1) & 0x1F;
}

static inline unsigned bank_of(unsigned pin)
{
    return pin / 4;
}

/*
 * What a pad presents, as a key that is equal for two states exactly when
 * embsim's drive (embsim_boards::p2::BankSupplies::pad_drive) is equal for
 * them: 0 is released (DIR clear, a bank whose supply names no voltage, the
 * float mode, or a current-source mode, which the package does not map);
 * anything else is 1 + the impedance field the OUT bit selects + 4 when the
 * pad drives a voltage above 0 V (OUT high in a bank whose supply is not
 * 0 V). The field's four values are four distinct impedances, and a pad
 * driving high in a 0 V bank presents 0 V, as its low does.
 */
static uint8_t drive_key(unsigned pin)
{
    unsigned bank = bank_of(pin);
    uint32_t bit = 1u << (pin & 31);
    bool out;
    uint32_t cfg;
    unsigned field;

    if (!(ib.dir[pin >> 5] & bit) || !((ib.banks_powered >> bank) & 1)) {
        return 0;
    }
    out = (ib.out[pin >> 5] & bit) != 0;
    cfg = ib.mode[pin];
    field = (cfg & P_MODE_SELECT_MASK) ? 0
          : (cfg >> (out ? P_HIGH_SHIFT : P_LOW_SHIFT)) & 7;
    if (field >= 4) {
        return 0;
    }
    return 1 + field + ((out && ((ib.banks_high >> bank) & 1)) ? 4 : 0);
}

/* What a bank reads: the guest's own OUT where the published drive is
 * strong, the net everywhere else. */
static uint32_t sensed(unsigned half)
{
    return (ib.strong[half] & ib.out[half]) | (~ib.strong[half] & ib.in_ext[half]);
}

/* Recompute the ORed DIR/OUT and the pads whose key moved. Whether a change
 * is newly pending: something is dirty and no instant was taken for it. */
static bool mark_changes(void)
{
    uint64_t changed = 0;
    unsigned c, pin;

    ib.dir[0] = ib.dir[1] = ib.out[0] = ib.out[1] = 0;
    for (c = 0; c < NUM_COGS; c++) {
        ib.dir[0] |= ib.dir_cog[c][0];
        ib.dir[1] |= ib.dir_cog[c][1];
        ib.out[0] |= ib.out_cog[c][0];
        ib.out[1] |= ib.out_cog[c][1];
    }
    for (pin = 0; pin < NUM_PINS; pin++) {
        if (drive_key(pin) != ib.pub_key[pin]) {
            changed |= 1ull << pin;
        }
    }
    ib.dirty = changed;
    return ib.dirty != 0 && !ib.pending;
}

static uint64_t cog_clocks(unsigned cog)
{
    CPUState *cpu = qemu_get_cpu((int)cog);

    return cpu ? cpu_env(cpu)->clocks : 0;
}

/* From a bus op: the first change in an instruction takes the executing
 * cog's clock as its instant and stops the cog after the instruction; later
 * ones in the same instruction (DRVH writes OUT then DIR) share it. */
static void recompute_and_mark(unsigned cog)
{
    if (mark_changes()) {
        ib.pending = true;
        ib.pending_at_clocks = cog_clocks(cog);
        p2_pinbus_yield = true;
        if (current_cpu) {
            cpu_exit(current_cpu);
        }
    }
}

static uint32_t ipc_ina(void *o)
{
    return sensed(0);
}

static uint32_t ipc_inb(void *o)
{
    return sensed(1);
}

static void ipc_dir_out_changed(void *o, unsigned cog, unsigned reg,
                                uint32_t v)
{
    unsigned c = cog & (NUM_COGS - 1);

    switch (reg) {
    case REG_DIRA:
        ib.dir_cog[c][0] = v;
        break;
    case REG_DIRB:
        ib.dir_cog[c][1] = v;
        break;
    case REG_OUTA:
        ib.out_cog[c][0] = v;
        break;
    case REG_OUTB:
        ib.out_cog[c][1] = v;
        break;
    default:
        return;
    }
    recompute_and_mark(c);
}

/* A mode write. On a pad the guest drives, a new strength is a pad change
 * like a DIR write. AKPIN assembles as WRPIN #1 and never arrives as a
 * distinct op: a cfg of 1 is an acknowledge, not a mode. */
static void ipc_wrpin(void *o, unsigned pin, uint32_t cfg)
{
    unsigned p = pin & 63;

    if (cfg == 1) {
        ib.in_flag[p] = false;
        return;
    }
    if (ib.mode[p] != cfg) {
        ib.mode_changed |= 1ull << p;
    }
    ib.mode[p] = cfg;
    ib.in_flag[p] = cfg != 0;
    if (ib.dir[p >> 5] & (1u << (p & 31))) {
        unsigned cog = current_cpu ? cpu_env(current_cpu)->cogid : 0;
        recompute_and_mark(cog & (NUM_COGS - 1));
    }
}

static void ipc_wxpin(void *o, unsigned pin, uint32_t x)
{
    ib.x[pin & 63] = x;
    ib.in_flag[pin & 63] = true;
}

/* An instruction-stream tap, configured or not: the boot chain's whole
 * observable is often one byte written to the debug pin without the pin ever
 * being configured. The bytes travel back with the STOP. */
static void ipc_wypin(void *o, unsigned pin, uint32_t y)
{
    unsigned p = pin & 63;

    if (ib.n_console >= MAX_CONSOLE) {
        error_report("p2 hostipc: the WYPIN tap overflowed its reply "
                     "(%u bytes in one run)", (unsigned)MAX_CONSOLE);
        abort();
    }
    ib.console[ib.n_console][0] = p;
    ib.console[ib.n_console][1] = y & 0xFF;
    ib.n_console++;
    ib.in_flag[p] = true;
}

static uint32_t ipc_pin_cfg(void *o, unsigned pin)
{
    return ib.mode[pin & 63];
}

/* $FF reads as an idle, pulled-high line; C reports BUSY and nothing here
 * ever is. Smart-pin receivers are not modelled. */
static uint32_t ipc_rdpin(void *o, unsigned pin, bool *busy)
{
    ib.in_flag[pin & 63] = false;
    *busy = false;
    return 0xFF;
}

static bool ipc_testp(void *o, unsigned pin)
{
    unsigned p = pin & 63;
    uint32_t mode = ib.mode[p];

    /*
     * In an ADC mode IN carries the sigma-delta bit stream, not the pad's
     * level. The boot ROM seeds its RNG by sampling its RX pin in
     * ADC-calibration mode 1550 times; the stream is not modelled and the
     * reference (p2core) reads it as zeros.
     */
    if ((mode & PIN_CFG_ADC_MASK) == PIN_CFG_ADC) {
        return false;
    }
    /* Plain GPIO: TESTP reads the LEVEL. The ROM floats P58 and samples it
     * to read the flash; a GPIO driver's _pinr() compiles to this. */
    if (smart_mode(mode) == 0) {
        return (sensed(p >> 5) >> (p & 31)) & 1;
    }
    /* A receiver reports a byte waiting: no serial peer exists yet. */
    if (smart_mode(mode) == SMART_ASYNC_RX ||
        smart_mode(mode) == SMART_SYNC_RX) {
        return false;
    }
    /* Any other configured pin completes its operation at once. */
    return true;
}

static void ipc_akpin(void *o, unsigned pin)
{
    ib.in_flag[pin & 63] = false;
}

static const P2PinBusOps ipc_ops = {
    .ina = ipc_ina,
    .inb = ipc_inb,
    .dir_out_changed = ipc_dir_out_changed,
    .wrpin = ipc_wrpin,
    .wxpin = ipc_wxpin,
    .wypin = ipc_wypin,
    .pin_cfg = ipc_pin_cfg,
    .rdpin = ipc_rdpin,
    .testp = ipc_testp,
    .akpin = ipc_akpin,
};

/*
 * The bank states a RUN brings. A bank whose state moved had its pads
 * republished by embsim before this RUN, at their drive under the new
 * supply, so their published key is the key they have now.
 */
static void take_banks(uint32_t powered, uint32_t high)
{
    uint32_t moved = (powered ^ ib.banks_powered) | (high ^ ib.banks_high);
    unsigned pin;

    ib.banks_powered = powered;
    ib.banks_high = high;
    if (!moved) {
        return;
    }
    for (pin = 0; pin < NUM_PINS; pin++) {
        if ((moved >> bank_of(pin)) & 1) {
            ib.pub_key[pin] = drive_key(pin);
        }
    }
}

/* ---- the slice loop ------------------------------------------------------ */

/*
 * One bounded slice of one cog, on this thread. The BQL is not held around
 * the icount bookkeeping, as the round-robin loop does not hold it: when the
 * budget clamps to zero, prepare takes the BQL itself to run the timers.
 */
static void ipc_slice(unsigned cog, int64_t budget)
{
    CPUState *cpu = qemu_get_cpu((int)cog);

    if (!cpu || cpu->halted) {
        return;
    }
    current_cpu = cpu;
    icount_prepare_for_run(cpu, budget);
    tcg_cpu_exec(cpu);
    icount_process_data(cpu);
}

static bool cog_running(unsigned cog)
{
    CPUState *cpu = qemu_get_cpu((int)cog);

    return cpu && cpu_env(cpu)->running && !cpu->halted;
}

static uint32_t cog_pc(unsigned cog)
{
    CPUState *cpu = qemu_get_cpu((int)cog);

    return cpu ? cpu_env(cpu)->pc : 0;
}

/* The machine's "now": the least-advanced running cog, as p2core's
 * system_clocks; the most advanced when none runs. */
static uint64_t machine_now_clocks(bool *any)
{
    uint64_t lo = UINT64_MAX, hi = 0;
    unsigned c;

    *any = false;
    for (c = 0; c < NUM_COGS; c++) {
        uint64_t k = cog_clocks(c);

        if (cog_running(c)) {
            *any = true;
            if (k < lo) {
                lo = k;
            }
        }
        if (k > hi) {
            hi = k;
        }
    }
    return *any ? lo : hi;
}

/*
 * Run the cogs round-robin from `start_cog` until one changes a pad, the
 * clock word changes, every running cog reaches the horizon, or nothing can
 * run, and write the STOP into `out_buf`. Returns its length.
 *
 * `start_cog` continues a pass: after a clock change embsim re-reads the
 * horizon in the new clock's counts and sends the RUN on from the next cog,
 * which keeps the cogs' interleaving what one unbroken pass gives.
 */
static size_t ipc_run(const P2IpcRun *rq, uint8_t *out_buf)
{
    P2IpcStop *st = (P2IpcStop *)out_buf;
    uint64_t horizon = rq->horizon_clocks;
    uint32_t reason = 0, slices = 0, stalls = 0;
    unsigned from = rq->start_cog % NUM_COGS, last_cog = 0, pin;
    bool mid_pass = from != 0, any;
    uint8_t *tail;
    int64_t t0 = get_clock();

    ib.in_ext[0] = rq->in_ext[0];
    ib.in_ext[1] = rq->in_ext[1];
    ib.strong[0] = rq->strong[0];
    ib.strong[1] = rq->strong[1];
    take_banks(rq->banks_powered, rq->banks_high);

    for (;;) {
        bool stepped = false;
        unsigned c;

        if (!mid_pass && machine_now_clocks(&any) >= horizon) {
            reason |= R_HORIZON;
            break;
        }
        for (c = from; c < NUM_COGS; c++) {
            uint64_t before;

            if (!cog_running(c) || cog_clocks(c) >= horizon) {
                continue;
            }
            before = cog_clocks(c);
            ipc_slice(c, cog_pc(c) < HUB_EXEC_BASE ? 1 : COG_QUANTUM);
            slices++;
            stepped = true;
            /* The slice's yield is taken with the slice, whatever else it
             * did: the flag is the change's, consumed here. */
            if (p2_pinbus_yield) {
                p2_pinbus_yield = false;
                reason |= R_YIELD;
            }
            if (p2_clock_mode != ib.reported_clock_mode) {
                ib.reported_clock_mode = p2_clock_mode;
                reason |= R_CLOCK;
            }
            if (ib.n_console >= CONSOLE_STOP) {
                reason |= R_CONSOLE;
            }
            if (reason) {
                last_cog = c;
                goto done;
            }
            if (cog_clocks(c) == before) {
                if (++stalls > STALL_SLICES) {
                    reason |= R_STALL;
                    last_cog = c;
                    goto done;
                }
            } else {
                stalls = 0;
            }
        }
        if (!stepped && !mid_pass) {
            reason |= R_NOSTEP;
            break;
        }
        mid_pass = false;
        from = 0;
    }
done:
    memset(st, 0, sizeof(*st));
    st->reason = reason;
    st->clock_mode = p2_clock_mode;
    st->clock_mode_at = p2_clock_mode_at;
    st->now_clocks = machine_now_clocks(&any);
    st->any_running = any;
    st->dir[0] = ib.dir[0];
    st->dir[1] = ib.dir[1];
    st->out[0] = ib.out[0];
    st->out[1] = ib.out[1];
    st->slices = slices;
    st->last_cog = last_cog;
    if (reason & R_YIELD) {
        st->pending_at_clocks = ib.pending_at_clocks;
        st->dirty = ib.dirty;
        /* embsim publishes exactly these before it sends the next RUN. */
        for (pin = 0; pin < NUM_PINS; pin++) {
            if (ib.dirty & (1ull << pin)) {
                ib.pub_key[pin] = drive_key(pin);
            }
        }
        ib.dirty = 0;
        ib.pending = false;
    }
    tail = out_buf + sizeof(*st);
    for (pin = 0; pin < NUM_PINS; pin++) {
        if (ib.mode_changed & (1ull << pin)) {
            uint32_t pc[2] = { pin, ib.mode[pin] };

            memcpy(tail, pc, sizeof(pc));
            tail += sizeof(pc);
            st->n_mode++;
        }
    }
    ib.mode_changed = 0;
    memcpy(tail, ib.console, ib.n_console * 2);
    tail += ib.n_console * 2;
    st->n_console = ib.n_console;
    ib.n_console = 0;
    st->run_ns = (uint32_t)MIN(get_clock() - t0, (int64_t)UINT32_MAX);
    return tail - out_buf;
}

/* ---- the process --------------------------------------------------------- */

static void die_now(int code)
{
    /* The trace QEMU logs with -d/-D is a stdio stream: flushed, so a run
     * that ends cleanly keeps every line it wrote. */
    fflush(NULL);
    _exit(code);
}

static int watch_fd = -1;

/* embsim holds the only write end. End of file is embsim gone. */
static void *p2ipc_watch(void *arg)
{
    char byte;

    for (;;) {
        ssize_t r = read(watch_fd, &byte, 1);

        if (r < 0 && errno == EINTR) {
            continue;
        }
        if (r <= 0) {
            die_now(0);
        }
    }
    return NULL;
}

/* ---- the transports ------------------------------------------------------ */

#ifdef __APPLE__
#define UL_COMPARE_AND_WAIT_SHARED 3
#define ULF_NO_ERRNO 0x01000000
extern int __ulock_wait(uint32_t operation, void *addr, uint64_t value,
                        uint32_t timeout_us);
extern int __ulock_wake(uint32_t operation, void *addr, uint64_t wake_value);

static void futex_wait(_Atomic uint32_t *a, uint32_t old)
{
    __ulock_wait(UL_COMPARE_AND_WAIT_SHARED | ULF_NO_ERRNO, (void *)a, old,
                 100000);
}

static void futex_wake(_Atomic uint32_t *a)
{
    __ulock_wake(UL_COMPARE_AND_WAIT_SHARED | ULF_NO_ERRNO, (void *)a, 0);
}
#elif defined(__linux__)
#include <linux/futex.h>
#include <sys/syscall.h>

/* Shared futexes: no FUTEX_PRIVATE_FLAG, the word is in a page two
 * processes map. */
static void futex_wait(_Atomic uint32_t *a, uint32_t old)
{
    struct timespec ts = { 0, 100000000 };

    syscall(SYS_futex, a, FUTEX_WAIT, old, &ts, NULL, 0);
}

static void futex_wake(_Atomic uint32_t *a)
{
    syscall(SYS_futex, a, FUTEX_WAKE, 1, NULL, NULL, 0);
}
#else
/* No shared futex: the shared page still works, polling. */
static void futex_wait(_Atomic uint32_t *a, uint32_t old)
{
    g_usleep(50);
}

static void futex_wake(_Atomic uint32_t *a)
{
}
#endif

typedef enum { CHAN_SHM, CHAN_SOCK } ChannelKind;

static ChannelKind chan_kind;
static int chan_fd = -1;
static uint64_t spin_ns = 20000;
static uint8_t *shm;
static uint32_t req_seen;
static uint8_t sock_rep[REP_CAP];

static inline _Atomic uint32_t *shm_word(size_t off)
{
    return (_Atomic uint32_t *)(shm + off);
}

/* Spin first: embsim's engine turn between stops is a few microseconds, and
 * a futex wake costs about as much again. Then block. */
static void shm_recv(P2IpcRun *rq)
{
    _Atomic uint32_t *seq = shm_word(SHM_REQ_SEQ);
    _Atomic uint32_t *sleeping = shm_word(SHM_REQ_SLEEP);
    int64_t deadline = get_clock() + (int64_t)spin_ns;
    unsigned i = 0;

    while (atomic_load_explicit(seq, memory_order_acquire) == req_seen) {
        if (spin_ns && ((++i & 63) != 0 || get_clock() < deadline)) {
            cpu_relax();
            continue;
        }
        atomic_store(sleeping, 1);
        while (atomic_load(seq) == req_seen) {
            futex_wait(seq, req_seen);
        }
        atomic_store(sleeping, 0);
    }
    req_seen = atomic_load_explicit(seq, memory_order_acquire);
    memcpy(rq, shm + SHM_REQ, sizeof(*rq));
}

static void shm_reply(void)
{
    atomic_store(shm_word(SHM_REP_SEQ), req_seen);
    if (atomic_load(shm_word(SHM_REP_SLEEP))) {
        futex_wake(shm_word(SHM_REP_SEQ));
    }
}

static bool rd_full(int fd, void *b, size_t n)
{
    size_t got = 0;

    while (got < n) {
        ssize_t r = read(fd, (char *)b + got, n - got);

        if (r <= 0) {
            if (r < 0 && errno == EINTR) {
                continue;
            }
            return false;
        }
        got += r;
    }
    return true;
}

static bool wr_full(int fd, const void *b, size_t n)
{
    size_t put = 0;

    while (put < n) {
        ssize_t r = write(fd, (const char *)b + put, n - put);

        if (r <= 0) {
            if (r < 0 && errno == EINTR) {
                continue;
            }
            return false;
        }
        put += r;
    }
    return true;
}

static void hello(P2IpcHello *h)
{
    memset(h, 0, sizeof(*h));
    h->magic = P2IPC_MAGIC;
    h->protocol = P2IPC_PROTOCOL;
    h->pid = (uint32_t)getpid();
    g_strlcpy(h->target, P2IPC_TARGET_IDENTITY, sizeof(h->target));
    g_strlcpy(h->qemu, QEMU_VERSION, sizeof(h->qemu));
}

static void *p2ipc_main(void *arg)
{
    P2IpcRun rq;
    P2IpcHello h;

    rcu_register_thread();
    tcg_register_thread();

    while (!runstate_is_running()) {
        g_usleep(1000);
    }
    p2_pinbus_set(&ipc_ops, &ib);

    hello(&h);
    if (chan_kind == CHAN_SHM) {
        memcpy(shm + SHM_HELLO, &h, sizeof(h));
        atomic_store(shm_word(SHM_READY), 1);
        futex_wake(shm_word(SHM_READY));
    } else if (!wr_full(chan_fd, &h, sizeof(h))) {
        die_now(0);
    }

    for (;;) {
        size_t len;

        if (chan_kind == CHAN_SHM) {
            shm_recv(&rq);
        } else if (!rd_full(chan_fd, &rq, sizeof(rq))) {
            die_now(0);         /* embsim closed the socket */
        }
        if (rq.op != OP_RUN) {
            die_now(0);         /* OP_QUIT */
        }
        if (chan_kind == CHAN_SHM) {
            ipc_run(&rq, shm + SHM_REP);
            shm_reply();
        } else {
            len = ipc_run(&rq, sock_rep);
            if (!wr_full(chan_fd, sock_rep, len)) {
                die_now(0);
            }
        }
    }
    return NULL;
}

/*
 * The machine exists and its cogs are reset: start the threads, and take the
 * terminating signals back from QEMU, whose handler asks for a shutdown that
 * waits on vCPUs this mode has parked. The process is in a group of its own
 * (embsim puts it there), so a terminal's ^C reaches embsim, not it.
 */
static void p2ipc_machine_ready(Notifier *n, void *data)
{
    static QemuThread main_thread, watch_thread;

    signal(SIGTERM, SIG_DFL);
    signal(SIGINT, SIG_DFL);
    signal(SIGHUP, SIG_DFL);
    qemu_thread_create(&watch_thread, "p2ipc-watch", p2ipc_watch, NULL,
                       QEMU_THREAD_DETACHED);
    qemu_thread_create(&main_thread, "p2ipc", p2ipc_main, NULL,
                       QEMU_THREAD_DETACHED);
}

static Notifier p2ipc_notifier = { .notify = p2ipc_machine_ready };

void p2_hostipc_configure(const char *spec, Error **errp)
{
    char kind[8];
    int cfd, wfd, used = 0;
    uint64_t spin = 20000;

    if (sscanf(spec, "%7[a-z]:%d:%d%n", kind, &cfd, &wfd, &used) != 3) {
        goto bad;
    }
    if (spec[used] == ':') {
        char *end;

        spin = g_ascii_strtoull(spec + used + 1, &end, 10);
        if (end == spec + used + 1 || *end) {
            goto bad;
        }
    } else if (spec[used]) {
        goto bad;
    }
    if (cfd < 0 || wfd < 0 || fcntl(cfd, F_GETFD) < 0 ||
        fcntl(wfd, F_GETFD) < 0) {
        error_setg(errp, "hostipc=%s: fd %d or %d is not open in this process; "
                   "embsim hands both down when it starts qemu-system-p2",
                   spec, cfd, wfd);
        return;
    }
    if (!strcmp(kind, "shm")) {
        void *page;

        page = mmap(NULL, SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, cfd, 0);
        if (page == MAP_FAILED) {
            error_setg_errno(errp, errno, "hostipc: cannot map the shared page "
                             "on fd %d", cfd);
            return;
        }
        shm = page;
        if (*(uint32_t *)(shm + SHM_MAGIC) != P2IPC_MAGIC) {
            error_setg(errp, "hostipc: fd %d is not embsim's shared page", cfd);
            return;
        }
        chan_kind = CHAN_SHM;
    } else if (!strcmp(kind, "sock")) {
        chan_kind = CHAN_SOCK;
    } else {
        goto bad;
    }
    chan_fd = cfd;
    watch_fd = wfd;
    spin_ns = spin;

    /*
     * The three host-driven flags, before anything reads them: the
     * round-robin thread reads rr_host_driven when it is created, the board
     * reads p2_host_driven in its init, and p2_pin_ops_end_tb is read at
     * translate time.
     */
    rr_host_driven = true;
    p2_host_driven = true;
    p2_pin_ops_end_tb = true;
    qemu_add_machine_init_done_notifier(&p2ipc_notifier);
    return;

bad:
    error_setg(errp, "hostipc=%s: want <shm|sock>:<channel-fd>:<watch-fd>"
               "[:<spin-ns>]", spec);
}
