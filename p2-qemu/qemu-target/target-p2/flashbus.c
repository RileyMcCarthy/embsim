/*
 * A pin bus with the boot flash on it, so the chip can boot the way silicon
 * does: nothing in hub RAM but the 16 KB Parallax boot ROM, and everything
 * else arriving over a bit-banged SPI bus.
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 *
 * This is the SECOND pin bus. `pinbus.c` mirrors p2core's small `SmartPins`
 * and is what the CPU differential harness runs against; this one mirrors
 * p2core's `Board` far enough to boot, and is what the ROM harness runs
 * against. They are separate because they answer separate questions, and the
 * bring-up bus deliberately has no peripherals at all.
 *
 * THE FLASH MODEL IS NOT HERE. It is embsim's, reached through `embsim.h` --
 * one model of the device, shared by every host that needs one, rather than a
 * C reimplementation that would be a second set of bugs uncovered by the
 * differential tests that make the first one trustworthy.
 *
 * The transport is a direct call rather than embsim's net engine, and that is
 * a deliberate and separate choice. The boot ROM bit-bangs this bus with
 * drvh/drvl/testp and samples a floated pin microseconds after driving the
 * clock -- sooner than a net resolves between engine wakes -- and it spends on
 * the order of 16 600 edges loading one kilobyte. A peripheral-clocked bus (the
 * SD card, the serial links) goes on nets; a CPU-bit-banged one cannot.
 *
 * Wiring, from p2core's flash.rs and confirmed against the P2-EC32MB netlist:
 *
 *      P58  DO   flash data out, read by the CPU with TESTP while floated
 *      P59  DI   flash data in
 *      P60  CLK  flash clock
 *      P61  ~CS  chip select, ACTIVE LOW -- and also the pull-up strap the ROM
 *                samples before it drives anything, to decide whether to try
 *                SPI at all
 *
 * P61 doing both jobs is the subtle part, and it falls out of the pin model
 * rather than needing a special case: a pin reads what the guest drives where
 * DIR is set and what the outside presents everywhere else, so the strap is
 * simply an external level that the ROM's own drive overrides once it starts
 * using the pin as chip select.
 */
#include "qemu/osdep.h"
#include "cpu.h"
#include "pinbus.h"
#include "embsim.h"

#define P2_PINS 64

/* The flash's four pins. */
#define P2_FLASH_DO  58
#define P2_FLASH_DI  59
#define P2_FLASH_CLK 60
#define P2_FLASH_CS  61
/* The debug console the boot chain's payload writes to with WYPIN. */
#define P2_PIN_TX    62
/* The async receive pins, which report "a byte is waiting" -- see testp. */
#define P2_PIN_RX       63
#define P2_PIN_PROTO_RX 53

typedef struct P2FlashBus {
    /*
     * DIR and OUT are PER-COG registers and the pad sees the OR across all
     * eight. Mirroring them globally lets one cog's write erase another's --
     * in p2core that showed up as a DEBUG print on P62 resetting the SD card's
     * shifter mid-block, so the cost of getting this wrong is not theoretical.
     */
    uint32_t dir_cog[P2_NUM_COGS][2];
    uint32_t out_cog[P2_NUM_COGS][2];
    uint32_t dir[2];
    uint32_t out[2];
    /* What the OUTSIDE presents: pull-ups, straps, and the flash's data out. */
    uint32_t in_ext[2];

    uint32_t mode[P2_PINS];     /* last WRPIN mode word; non-zero = configured */
    uint32_t x[P2_PINS];        /* last WXPIN parameter */
    bool     in_flag[P2_PINS];  /* pending IN flag, for configured pins */

    EmbsimSpiFlash *flash;      /* embsim's model; NULL means no part fitted */

    /*
     * How many reads the part had served last time we looked. A boot's whole
     * shape is in WHERE it looked -- the ROM takes stage-1 from 0 and stage-1
     * takes the application from $400 -- and that is a far sharper assertion
     * than whether the boot finished. Announcing each new one as it happens
     * keeps the harness from needing a way to ask at the end, which a guest
     * that never halts would not give it.
     */
    size_t reads_seen;

    /* What the guest has written to the debug pin, which is how a boot test
     * observes that the program it loaded actually ran. */
    GString *console;
} P2FlashBus;

static P2FlashBus p2_flashbus;

static void flashbus_announce_reads(void);

static inline unsigned bank_of(unsigned pin)  { return (pin & 63) >> 5; }
static inline uint32_t  bit_of(unsigned pin)  { return 1u << (pin & 31); }

/*
 * What one bank actually reads: what the guest drives where DIR is set, and
 * what the outside presents everywhere else.
 *
 * This single line is why the ROM can use P61 as both a strap and a chip
 * select, and why it can float P58 to read the flash.
 */
static uint32_t flashbus_sensed(unsigned bank)
{
    uint32_t driven = p2_flashbus.dir[bank];

    return (driven & p2_flashbus.out[bank]) | (~driven & p2_flashbus.in_ext[bank]);
}

static bool flashbus_pin_state(unsigned pin)
{
    return (flashbus_sensed(bank_of(pin)) & bit_of(pin)) != 0;
}

static void flashbus_set_input_level(unsigned pin, bool level)
{
    unsigned bank = bank_of(pin);

    if (level) {
        p2_flashbus.in_ext[bank] |= bit_of(pin);
    } else {
        p2_flashbus.in_ext[bank] &= ~bit_of(pin);
    }
}

/*
 * Drive the flash from its pins, right now.
 *
 * `clock` is idempotent in the level, so forwarding every pin write is safe
 * and the bus need not track edges itself. Publishing MISO afterwards is what
 * makes a TESTP on a floated P58 read the bit the part is presenting.
 */
static void flashbus_clock_flash(void)
{
    if (!embsim_spi_flash_present(p2_flashbus.flash)) {
        return;
    }
    /* ~CS is active low; inverting it is the board's job, not the part's. */
    embsim_spi_flash_set_selected(p2_flashbus.flash,
                                  !flashbus_pin_state(P2_FLASH_CS));
    embsim_spi_flash_clock(p2_flashbus.flash,
                           flashbus_pin_state(P2_FLASH_CLK),
                           flashbus_pin_state(P2_FLASH_DI));
    flashbus_set_input_level(P2_FLASH_DO,
                             embsim_spi_flash_miso(p2_flashbus.flash));
    flashbus_announce_reads();
}

/*
 * Print any read the part has served since we last looked.
 *
 * On stdout and unconditional, deliberately: these lines and the console ones
 * below are the only window a harness has into a guest that never halts, and
 * making them conditional on a log mask would mean a test that silently
 * asserted nothing when the mask was wrong.
 *
 * Drained in windows from `reads_seen`, and the fill reports how many it
 * WROTE. An earlier version took the fill's return as the number of valid
 * entries when it was the running total, and indexed past this array the
 * moment more than sixteen reads existed. It could not fire on a ROM boot,
 * which serves exactly two -- it was waiting for the first firmware to page
 * from flash.
 */
static void flashbus_announce_reads(void)
{
    uint32_t addrs[16];
    size_t got, i;

    while ((got = embsim_spi_flash_reads(p2_flashbus.flash,
                                         p2_flashbus.reads_seen,
                                         addrs, ARRAY_SIZE(addrs))) > 0) {
        for (i = 0; i < got; i++) {
            printf("P2FLASHREAD %u\n", addrs[i]);
        }
        fflush(stdout);
        p2_flashbus.reads_seen += got;
    }
}

static uint32_t flashbus_ina(void *o) { return flashbus_sensed(0); }
static uint32_t flashbus_inb(void *o) { return flashbus_sensed(1); }

static void flashbus_dir_out_changed(void *o, unsigned cog, unsigned reg,
                                     uint32_t value)
{
    unsigned c = cog & (P2_NUM_COGS - 1);
    int i;

    switch (reg) {
    case P2_REG_DIRA:     p2_flashbus.dir_cog[c][0] = value; break;
    case P2_REG_DIRA + 1: p2_flashbus.dir_cog[c][1] = value; break;
    case P2_REG_OUTA:     p2_flashbus.out_cog[c][0] = value; break;
    case P2_REG_OUTA + 1: p2_flashbus.out_cog[c][1] = value; break;
    default: return;
    }

    p2_flashbus.dir[0] = p2_flashbus.dir[1] = 0;
    p2_flashbus.out[0] = p2_flashbus.out[1] = 0;
    for (i = 0; i < P2_NUM_COGS; i++) {
        p2_flashbus.dir[0] |= p2_flashbus.dir_cog[i][0];
        p2_flashbus.dir[1] |= p2_flashbus.dir_cog[i][1];
        p2_flashbus.out[0] |= p2_flashbus.out_cog[i][0];
        p2_flashbus.out[1] |= p2_flashbus.out_cog[i][1];
    }
    flashbus_clock_flash();
}

static void flashbus_wrpin(void *o, unsigned pin, uint32_t cfg)
{
    p2_flashbus.mode[pin & 63] = cfg;
    p2_flashbus.in_flag[pin & 63] = cfg != 0;
}

static void flashbus_wxpin(void *o, unsigned pin, uint32_t x)
{
    p2_flashbus.x[pin & 63] = x;
    p2_flashbus.in_flag[pin & 63] = true;
}

static void flashbus_wypin(void *o, unsigned pin, uint32_t y)
{
    if ((pin & 63) == P2_PIN_TX && p2_flashbus.console) {
        /*
         * The debug pin. A boot chain's whole observable is often one byte
         * arriving here -- it means the program that was loaded off the flash
         * really executed, which nothing else in the trace can show.
         */
        g_string_append_c(p2_flashbus.console, (char)(y & 0xFF));
        printf("P2CON %02X\n", (unsigned)(y & 0xFF));
        fflush(stdout);
    }
    p2_flashbus.in_flag[pin & 63] = true;
}

static uint32_t flashbus_pin_cfg(void *o, unsigned pin)
{
    return p2_flashbus.mode[pin & 63];
}

static uint32_t flashbus_rdpin(void *o, unsigned pin, bool *busy)
{
    p2_flashbus.in_flag[pin & 63] = false;
    *busy = false;              /* C reports BUSY; nothing here ever is */
    return 0xFF;                /* an idle, pulled-high line */
}

static bool flashbus_testp(void *o, unsigned pin)
{
    unsigned p = pin & 63;

    /*
     * An ASYNC RX pin reports "a byte is waiting", not "an operation
     * finished". No serial peer is attached here, so nothing is ever waiting.
     *
     * This is not a detail. The boot ROM's very first act is to seed the
     * hardware RNG by sampling the RX pin in ADC calibration mode --
     *
     *     wrpin ##$00100000,#rx_pin
     *     rep   #2,#31
     *     testp #rx_pin  wc
     *     rcl   y,#1
     *
     * -- 1550 times. WRPIN leaves the pin configured, so a bus that answered
     * "configured means ready" returns C=1 on every one of those and the trace
     * diverges from p2core's on the FIFTH instruction of the boot, long before
     * the flash is ever touched. That is exactly how this was found.
     */
    if (p == P2_PIN_RX || p == P2_PIN_PROTO_RX || p == 0 || p == 1) {
        return false;
    }

    /*
     * A pin with no smart-pin mode is plain GPIO and TESTP reads its LEVEL,
     * not an IN flag. That is the path the boot ROM takes to read the flash --
     * it floats P58 and samples it -- so reporting a flag here would make the
     * ROM read the same bit forever and the boot would never get past its
     * first byte.
     */
    if (p2_flashbus.mode[p] == 0) {
        return flashbus_pin_state(p);
    }
    return p2_flashbus.in_flag[p] || p2_flashbus.mode[p] != 0;
}

static void flashbus_akpin(void *o, unsigned pin)
{
    p2_flashbus.in_flag[pin & 63] = false;
}

static const P2PinBusOps p2_flashbus_ops = {
    .ina = flashbus_ina,
    .inb = flashbus_inb,
    .dir_out_changed = flashbus_dir_out_changed,
    .wrpin = flashbus_wrpin,
    .wxpin = flashbus_wxpin,
    .wypin = flashbus_wypin,
    .pin_cfg = flashbus_pin_cfg,
    .rdpin = flashbus_rdpin,
    .testp = flashbus_testp,
    .akpin = flashbus_akpin,
};

void p2_flashbus_init(const uint8_t *image, size_t len, size_t capacity)
{
    memset(&p2_flashbus, 0, sizeof(p2_flashbus));
    p2_flashbus.console = g_string_new(NULL);
    p2_flashbus.flash = embsim_spi_flash_with_image(capacity, image, len);

    /*
     * The flash-boot strap: a pull-up on P61 is what tells the ROM the flash
     * is the boot source. Without it the ROM samples a low pin, decides there
     * is nothing to boot from, and falls through to the serial loader --
     * which looks exactly like a flash that failed to answer.
     */
    flashbus_set_input_level(P2_FLASH_CS, true);
    /* And publish the part's first bit, so a TESTP before any clock is right. */
    flashbus_set_input_level(P2_FLASH_DO,
                             embsim_spi_flash_miso(p2_flashbus.flash));

    p2_pinbus_set(&p2_flashbus_ops, &p2_flashbus);
}

const char *p2_flashbus_console(void)
{
    return p2_flashbus.console ? p2_flashbus.console->str : "";
}

size_t p2_flashbus_reads(uint32_t *out, size_t cap)
{
    return embsim_spi_flash_reads(p2_flashbus.flash, 0, out, cap);
}
