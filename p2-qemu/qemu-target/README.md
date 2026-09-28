# `target/p2` — the QEMU Propeller 2 target

The source of truth for the P2 target `embsim-p2-qemu` links, and for the
standalone `qemu-system-p2` the MaD differential harnesses run. It is carried
here rather than in a QEMU fork so a change to the target and the node that
drives it land in one review; a fork pinned as a submodule is the eventual
shape, as `embsim` and `ProtoEmb` are to MaD.

## Layout

| path | goes to |
|---|---|
| `target-p2/` | `target/p2/` in the QEMU tree |
| `hw-p2/` | `hw/p2/` |
| `p2-softmmu.mak` | `configs/targets/p2-softmmu.mak` |
| `p2-softmmu-devices.mak` | `configs/devices/p2-softmmu/default.mak` — the P2 has one board and no optional devices, but meson requires the file to exist |
| `register-p2.patch` | the five registration edits (`target/meson.build`, `hw/meson.build`, both `Kconfig`s, `QEMU_ARCH_P2` in `include/system/arch_init.h`) |
| `host-thread.patch` | parks QEMU's own vCPU thread when `EMBSIM_QEMU_HOST_THREAD` is set, so a host thread — embsim's engine — can run the cogs itself (`../hostdrive.c`) |

`target-p2/insn.decode` is **generated** from p2core (`tools/gen_decoder.py
--decodetree` in MaD): the 359 encodings stay single-source with p2core's Rust
decoder, so the two cannot drift. Regenerate it there and copy it here.

Two more files are generated at build time and not copied: `trans_stub.c.inc`
and `interp_stub.c.inc`, emitted by `target-p2/gen_stubs.py`, one per
dispatcher. Every DecodeTree pattern needs a function to link; anything not
hand-written halts the cog and logs the opcode and PC rather than silently
doing the wrong thing.

**QEMU is pinned at `v10.1.0`**, and the pin lives in one place: the
`QEMU_TAG` of the `p2-qemu-boot` job in `.github/workflows/ci.yml`. Both
patches carry upstream context, so they will reject on a different tree — if
one does, that tag is what moved.

```bash
cp -r target-p2/* <qemu>/target/p2/ ; cp -r hw-p2/* <qemu>/hw/p2/
cp p2-softmmu.mak <qemu>/configs/targets/
mkdir -p <qemu>/configs/devices/p2-softmmu
cp p2-softmmu-devices.mak <qemu>/configs/devices/p2-softmmu/default.mak
cd <qemu> && patch -p1 < .../register-p2.patch && patch -p1 < .../host-thread.patch

# The stub files are #included, so they have to exist before the build. One
# insn.decode, two dispatchers -- see "Two engines" below.
python3 scripts/decodetree.py --static-decode=decode_p2 --insnwidth=32 \
        -o /tmp/d.c.inc target/p2/insn.decode
python3 scripts/decodetree.py --static-decode=interp_p2 --translate=iexec \
        --insnwidth=32 -o /tmp/i.c.inc target/p2/insn.decode
python3 target/p2/gen_stubs.py /tmp/d.c.inc target/p2/trans_stub.c.inc
python3 target/p2/gen_stubs.py /tmp/i.c.inc target/p2/interp_stub.c.inc --interp

# The standalone binary's flash bus links embsim's flash model as a static
# library -- see "The flash is embsim's" below. Build it first with
# `cargo build -p embsim-cffi` (from this repository's root).
EMB=<embsim>
./configure --target-list=p2-softmmu --disable-containers --enable-pie \
    --extra-cflags="-I$EMB/cffi/include" \
    --extra-ldflags="$EMB/target/debug/libembsim_cffi.a \
                     -framework CoreFoundation -framework Security"
make
# Then the node:
EMBSIM_QEMU_P2_BUILD=<qemu>/build cargo test -p embsim-p2-qemu
```

`--disable-containers` matters on macOS: `configure` otherwise hangs in
`docker version`. On Linux the link needs `-lpthread -ldl -lm` instead of the
two frameworks, and `--enable-pie` is what lets the objects link into a Rust
test binary, which is position-independent.

`embsim-p2-qemu` itself does NOT link `flashbus.c` or the cffi archive: its
flash is a component on the board (`build.rs`). The archive is only for the
standalone binary's own link.

## Three host-facing flags (`target-p2/pinbus.h`)

- `p2_pinbus_yield` — a bus sets it from inside a pin op: stop this cog after
  the current instruction. The interpreter checks it after every instruction;
  the bus also calls `cpu_exit()` so `cpu_exec` returns.
- `p2_pin_ops_end_tb` — read at translate time: end a translated block after
  every pad-changing instruction, so the check above lands at the same place
  in both engines. Off in the standalone emulator (a block end is ~53 ns).
- `p2_host_driven` — the board arms no quantum timer; the host slices.
- `p2_clock_mode` / `p2_clock_mode_at` — the clock word the guest last wrote
  with `HUBSET`, and the executing cog's clock at that instruction. Neither
  engine derives anything from it; a host placing edges on a nanosecond
  timeline needs the frequency the clocks tick at.

## What it does today

Runs the real MaDCore firmware. `qemu-system-p2 -M p2 -kernel <image>` boots it
the way silicon does — the first `$1F8` longs of hub become cog 0's RAM and it
runs them from cog `$000`, because a P2 image is a cog program, not a hub one —
and **the first million instructions are identical to p2core's**, state by
state, registers, flags, stack pointer and cycle count
(`../p2core/tools/fwtest.sh`).

220 DecodeTree patterns are hand-written in each engine, covering every
mnemonic the firmware executes: 80 in hub space, 41 in cog space, measured over
20 M instructions with `../p2core/examples/ophist.rs`.

It also **boots the way silicon does**. `qemu-system-p2 -M p2,flash=<image>
-bios <rom>` puts nothing in hub but Parallax's own 16 KB boot ROM; the ROM
samples the pull-up strap on P61, bit-bangs the SPI flash, loads its first
kilobyte, verifies the 256 longs sum to `"Prop"`, copies them into cog RAM and
jumps. That kilobyte is this repository's stage-1 loader, which reads the
application from flash `$400` and relaunches the cog on it. The whole chain is
**identical to p2core's, state by state** (`../p2core/tools/romtest.sh`), and
the two things it actually DID -- the flash served reads at `0` and `$400`, and
the booted payload's byte reached the console -- are the same assertions
`../p2core/tests/rom_boot_chain.rs` makes.

## The flash is embsim's, not a copy

`flashbus.c` is a second pin bus, mirroring p2core's `Board` far enough to
boot. What it is NOT is a second flash model: the device is
`embsim/models/src/spi_flash.rs`, reached from C through `embsim-cffi`. One
model of the part, shared by every host that needs one -- a C reimplementation
would be a second set of bugs, uncovered by the differential tests that make
the first one trustworthy.

The TRANSPORT is a direct call rather than embsim's net engine, and that is a
separate decision from where the model lives. The boot ROM drives a clock edge
and samples a floated pin microseconds later -- sooner than a net resolves
between engine wakes -- and spends about 8 300 clock pulses -- 16 600 edges --
loading one kilobyte. A peripheral-clocked bus (the SD card, the serial links) goes on
nets; a CPU-bit-banged one cannot.

## Two engines, one instruction set

Hub-exec is translated (`translate.c`, TCG) and cog-exec is interpreted
(`interp.c`) — the hybrid Spike 0b measured at 4.7x against 0.3x for making cog
RAM ordinary guest memory. Three things stop the interpreter becoming a second
opinion about the ISA:

- the **decoder is shared**. `decodetree --translate=iexec` emits a second
  dispatcher over the same `insn.decode`, so `trans_*` and `iexec_*` cannot
  disagree about an encoding;
- everything with real machinery behind it — the pin bus, the lock pool,
  CORDIC, hub block transfers, the hardware stack, COGINIT, REP, SKIP — calls
  the **same helpers** from both;
- and the differential harness runs generated programs in **both** spaces, so
  the interpreter is diffed against p2core exactly as the translator is.

What is genuinely duplicated is the ALU core, and that is what the harness
covers most densely.

## The Phase 0 design decisions, built in

- **Cog RAM and LUT live in `CPUArchState`**, deliberately absent from the
  address space. Routing the register file through softmmu costs +194 ns per
  write (Spike 0b) and the firmware does 0.62 of them per instruction.
- **Hub-exec translated, cog-exec interpreted** in runs of 48 via
  `helper_p2_interp_cog`: hub RAM takes zero SMC invalidations over a whole
  firmware run and is 93 % of instructions; cog RAM is the register file and
  would re-translate 880 000 times.
- **Operands resolve at translate time** to constant env offsets. `ALTx` is a
  prefix the translator sees, and only the instruction after one needs runtime
  indexing: 0.0999 % of the stream.
- **Instruction-stream state travels in the TB key** — which prefixes are live,
  whether a REP or SKIP is running — so a block with none pending emits nothing
  for them at all.
- **Pin ops are helpers that never end a block** (Spike 0d: ~0 ns vs 53.5 ns)
  — in the standalone emulator. A host that lifts pins onto nets asks for the
  block end with `p2_pin_ops_end_tb`, and pays it only there.
- **The board arms a quantum timer under `-icount`.** Round-robin TCG only
  switches vCPUs when `cpu_exec` returns, and its budget comes from the next
  virtual deadline — with no timer armed, the first cog to spin starves every
  other one and COGINIT appears to work while the cog that called it never runs
  again.

## Test harnesses

All in MaD's `SIL/p2core/tools/`, all diffing against p2core state by state:

| | |
|---|---|
| `difftest.sh` | randomised programs; `cog=1` runs the body in cog space |
| `edgetest.sh` | hand-built probes for stream edges a random program reaches only by luck |
| `cogtest.sh` | two cogs: COGINIT, compared on final register state, not timing |
| `fwtest.sh` | the real firmware |
| `romtest.sh` | the boot ROM loading a program off SPI flash |

Here, `p2-qemu/tests/rom_boot_ec32mb.rs` boots the same chain through the
node, on the P2-EC32MB board, over nets — `p2-qemu-boot` in CI, in
`ci-gate.needs`.

## What it does NOT do yet

- **No SD card and no serial peer in the standalone binary.** `flashbus.c`
  has the boot flash, which is all the ROM chain needs; `pinbus.c` remains the
  bring-up model for the CPU differential tests. On embsim's engine
  (`embsim-p2-qemu`) the flash is already a board component and the SD card
  can be; the UART peers are the next seam.
- **No streamer** (`XINIT`/`XZERO`/`XCONT`/`SETXFRQ`), so the transition-mode
  smart-pin clock path halts rather than guessing.
- **The refused set matches p2core's**, deliberately: an instruction the oracle
  traps on cannot be differentially tested, so implementing it would ship
  untested code. `POLLCT1-3` are refused for a stronger reason — the CT
  deadline *is* modelled, so a poll that always reported not-set would
  contradict `WAITCT1`.
- **Interrupts and `SKIPF`/`EXECF`** are not modelled. `SETINT1-3` store their
  source select and nothing reads it, which is exactly what p2core does — so
  the two agree, and the harness will find it the moment either starts
  modelling interrupts for real.
- **The hub FIFO is an address pointer**, not a FIFO: no block-wrap count, no
  prefetch depth. That is what p2core models, so it is the most the harness can
  check, and every user in reach streams sequentially with a wrap count of
  zero.
- The silicon goldens in MaD's `SIL/p2core/hwtest/` are still the Phase 2 target.
