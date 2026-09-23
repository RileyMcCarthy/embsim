/*
 * A bare Propeller 2: 512 KB of hub RAM and eight cogs.
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */
#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/error-report.h"
#include "system/system.h"
#include "qemu/datadir.h"
#include "qapi/error.h"
#include "hw/boards.h"
#include "hw/qdev-properties.h"
#include "system/address-spaces.h"
#include "system/system.h"
#include "target/p2/cpu.h"
#include "target/p2/pinbus.h"
#include "qemu/timer.h"
#include "exec/icount.h"
#include "system/address-spaces.h"
#include "hw/loader.h"
#include "qemu/error-report.h"

/* One definition, in cpu.h: the CPU's ROM-boot base is derived from it. */
#define P2_HUB_SIZE P2_HUB_BYTES

/*
 * The scheduling quantum, in instructions (under -icount shift=0 one
 * instruction is one nanosecond of virtual time).
 *
 * Round-robin TCG only moves to the next vCPU when cpu_exec returns, and its
 * instruction budget comes from the next QEMU_CLOCK_VIRTUAL deadline -- with
 * no timer armed that budget is INT32_MAX. A cog parked in a spin loop then
 * never returns and starves every other cog: COGINIT appears to work, the new
 * cog runs, and the cog that started it never executes another instruction.
 * So the machine arms a timer that does nothing except exist.
 *
 * 48 is the quantum Spike 0c measured the firmware tolerates.
 *
 * Only under -icount, where one instruction is one nanosecond of virtual time
 * and the deadline therefore means what it says. Without icount
 * QEMU_CLOCK_VIRTUAL runs on host time, a 48 ns period fires continuously, and
 * the round-robin loop trips its own "instruction counter expired" assertion.
 * There the accelerator's own wall-clock kick timer does the switching -- less
 * deterministic, which is why D1 asks for icount in the first place.
 */
#define P2_QUANTUM_NS 48

static QEMUTimer *p2_quantum;

/*
 * `-M p2,flash=<file>` -- the image the boot flash comes up holding, which is
 * what a loader would have left behind. 16 MiB is the density of the part on
 * the Parallax P2-EC32MB module (a Winbond W25Q128JV); an image shorter than
 * that leaves the rest erased, as a real part does.
 */
#define P2_FLASH_CAPACITY (16 * 1024 * 1024)
static char *p2_flash_file;

static char *p2_get_flash(Object *obj, Error **errp)
{
    return g_strdup(p2_flash_file);
}

static void p2_set_flash(Object *obj, const char *value, Error **errp)
{
    g_free(p2_flash_file);
    p2_flash_file = g_strdup(value);
}

static void p2_quantum_tick(void *opaque)
{
    timer_mod(p2_quantum,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + P2_QUANTUM_NS);
}

static void p2_machine_init(MachineState *machine)
{
    MemoryRegion *hub = g_new(MemoryRegion, 1);
    MemoryRegion *wrap = g_new(MemoryRegion, 1);
    int i;

    /*
     * Hub RAM is the only real guest memory. Cog RAM and LUT are CPU state
     * (Spike 0b), so they are deliberately absent from the address space.
     */
    memory_region_init_ram(hub, NULL, "p2.hub", P2_HUB_SIZE, &error_fatal);
    memory_region_add_subregion(get_system_memory(), 0, hub);

    /*
     * Hub addressing wraps: silicon (and p2core, which assembles each byte
     * through `addr & (HUB_BYTES - 1)`) lets a RDLONG at $7FFFD read its last
     * byte from $00000. The translator masks the address to 19 bits, so the
     * only case left is an access that straddles the top -- an alias of the
     * whole hub at $80000 makes that wrap exactly, at no cost in memory.
     */
    memory_region_init_alias(wrap, NULL, "p2.hub.wrap", hub, 0, P2_HUB_SIZE);
    memory_region_add_subregion(get_system_memory(), P2_HUB_SIZE, wrap);

    /*
     * Not when a host outside the main loop drives the cogs itself
     * (pinbus.h, `p2_host_driven`): it does the slicing, and a virtual-clock
     * deadline nothing ever services would cap every icount budget at zero.
     */
    if (icount_enabled() && !p2_host_driven) {
        p2_quantum = timer_new_ns(QEMU_CLOCK_VIRTUAL, p2_quantum_tick, NULL);
        p2_quantum_tick(NULL);
    }

    /*
     * `-kernel <image>` is the way to run a real P2 image: it goes into hub at
     * $0 and the first $1F8 longs become cog 0's RAM, which the CPU's reset
     * does. Loading it HERE, before the CPUs exist, is what makes that
     * ordering work -- a `-device loader` file is written by a reset handler
     * registered after the board's, so the cog seed would read zeros.
     *
     * The harness keeps using `-device loader` with an explicit entry PC,
     * which starts a generated program in hub space and needs no cog seed.
     */
    if (machine->kernel_filename) {
        gsize len;
        char *buf;

        if (!g_file_get_contents(machine->kernel_filename, &buf, &len, NULL)) {
            error_report("p2: cannot read %s", machine->kernel_filename);
            exit(1);
        }
        if (len > P2_HUB_SIZE) {
            len = P2_HUB_SIZE;
        }
        cpu_physical_memory_write(0, buf, len);
        g_free(buf);
        p2_boot_from_hub = true;
    }

    /*
     * `-bios <rom>` boots the chip the way silicon does: the 16 KB Parallax
     * boot ROM goes to the top of hub, cog 0 is seeded from its base, and
     * everything else has to arrive over the flash bus. Nothing else is
     * preloaded -- that is the whole point of the mode.
     *
     * Loaded HERE, before the CPUs exist, for the same reason -kernel is: a
     * `-device loader` file is written by a reset handler registered after the
     * board's, so the cog seed would read zeros.
     */
    if (machine->firmware) {
        char *path = qemu_find_file(QEMU_FILE_TYPE_BIOS, machine->firmware);
        gsize len;
        char *buf;

        if (!path || !g_file_get_contents(path, &buf, &len, NULL)) {
            error_report("p2: cannot read boot ROM %s", machine->firmware);
            exit(1);
        }
        if (len > P2_BOOT_ROM_SIZE) {
            len = P2_BOOT_ROM_SIZE;
        }
        cpu_physical_memory_write(P2_BOOT_ROM_BASE, buf, len);
        g_free(buf);
        g_free(path);
        p2_boot_from_rom = true;

        /*
         * And put the flash on the pins. Installed even with no image: an
         * erased part that answers is a different thing from no part at all,
         * and the ROM distinguishes them -- so a test for "the ROM gives up
         * gracefully" needs the empty case to be reachable.
         */
        if (p2_flash_file) {
            gsize flen;
            char *fbuf;

            if (!g_file_get_contents(p2_flash_file, &fbuf, &flen, NULL)) {
                error_report("p2: cannot read flash image %s", p2_flash_file);
                exit(1);
            }
            p2_flashbus_init((const uint8_t *)fbuf, flen, P2_FLASH_CAPACITY);
            g_free(fbuf);
        } else {
            p2_flashbus_init(NULL, 0, P2_FLASH_CAPACITY);
        }
    }

    for (i = 0; i < P2_NUM_COGS; i++) {
        Object *cpu = object_new(TYPE_P2_CPU);
        object_property_set_uint(cpu, "cogid", i, &error_fatal);
        qdev_realize(DEVICE(cpu), NULL, &error_fatal);
    }
}

static void p2_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "Parallax Propeller 2";
    mc->init = p2_machine_init;
    mc->max_cpus = P2_NUM_COGS;
    mc->default_cpu_type = TYPE_P2_CPU;
    mc->no_floppy = 1;
    mc->no_cdrom = 1;
    mc->no_parallel = 1;

    object_class_property_add_str(oc, "flash", p2_get_flash, p2_set_flash);
    object_class_property_set_description(oc, "flash",
        "Image the boot SPI flash comes up holding (with -bios)");
}

static const TypeInfo p2_machine_types[] = {
    {
        .name = MACHINE_TYPE_NAME("p2"),
        .parent = TYPE_MACHINE,
        .class_init = p2_machine_class_init,
    },
};

DEFINE_TYPES(p2_machine_types)
