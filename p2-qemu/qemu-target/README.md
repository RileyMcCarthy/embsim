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
| `hw-p2/` | `hw/p2/`: the board |
| `p2-softmmu.mak` | `configs/targets/p2-softmmu.mak` |
| `p2-softmmu-devices.mak` | `configs/devices/p2-softmmu/default.mak`: the standalone build |
| `p2-softmmu-node-devices.mak` | `configs/devices/p2-softmmu/node.mak`: `embsim-p2-qemu`'s build (`--with-devices-p2=node`) |
| `register-p2.patch` | the five registration edits (`target/meson.build`, `hw/meson.build`, both `Kconfig`s, `QEMU_ARCH_P2` in `include/system/arch_init.h`) |
| `host-thread.patch` | parks QEMU's own vCPU thread when the host sets `rr_host_driven`, so a host thread — embsim's engine — can run the cogs itself (`../hostdrive.c`) |
| `stage.sh` | puts all of the above into a QEMU source tree |
| `LICENSE-PNut-TS` | the notice `target-p2/insn.decode`'s source carries |

`target-p2/insn.decode` is **generated** from p2core's decoder table, so the
359 encodings stay single-source with p2core's Rust decoder and the two cannot
drift. It is the output of

```bash
python3 SIL/p2core/tools/gen_decoder.py --decodetree <embsim>/p2-qemu/qemu-target/target-p2/insn.decode
```

run in RileyMcCarthy/MaD at `6e1581634b69368effe48b7476bb483d6f67dfc0` (to be
re-pinned to MaD's main once that branch merges), and that command reproduces
this file byte-for-byte. Its input is MaD's `SIL/p2core/vendor/parseUtils.ts`,
which is PNut-TS's `src/classes/parseUtils.ts` at
ironsheep/PNut-TS@`d9e46a378c7d0efdad45c9fee5f029b7314cfeb3`, under the MIT
License in [`LICENSE-PNut-TS`](LICENSE-PNut-TS). Regenerate it there and copy
it here; do not edit it.

Two more files are generated at build time, into the build directory, and not
copied: `target/p2/trans_stub.c.inc` and `target/p2/interp_stub.c.inc`, which
`target-p2/meson.build` has `target-p2/gen_stubs.py` emit from each
dispatcher's decoder. Every DecodeTree pattern needs a function to link;
anything not hand-written halts the cog and reports the opcode and PC on
stderr rather than silently doing the wrong thing.

## Building

**QEMU is pinned at `v10.1.0`**, and the pin lives in one place: `QEMU_TAG`
and `QEMU_COMMIT` of the `p2-qemu-boot` job in `.github/workflows/ci.yml`,
which checks the fetched tag against the commit. Both patches carry upstream
context, so they will reject on a different tree — if one does, that tag is
what moved.

`stage.sh` copies this directory into a QEMU checkout and applies both
patches. It replaces `target/p2` and `hw/p2` outright and skips a patch that
is already applied, so re-run it after every edit here:

```bash
git clone --depth 1 --branch v10.1.0 https://gitlab.com/qemu-project/qemu.git <qemu>
p2-qemu/qemu-target/stage.sh <qemu>
```

`embsim-p2-qemu` links a tree configured with `--with-devices-p2=node`:

```bash
mkdir <qemu>/build-p2 && cd <qemu>/build-p2
../configure --target-list=p2-softmmu --disable-containers --enable-pie \
    --disable-docs --disable-werror --with-devices-p2=node
ninja qemu-system-p2
# Then, from this repository's root:
EMBSIM_QEMU_P2_BUILD=<qemu>/build-p2 cargo test -p embsim-p2-qemu
```

The standalone `qemu-system-p2` MaD's CPU differentials run (`difftest.sh`,
`edgetest.sh`, `cogtest.sh`, `fwtest.sh`) is the same configure without
`--with-devices-p2=node`. Its pin model is `target-p2/pinbus.c`. The boot
ROM's flash is not a device of that binary: `romtest.sh` diffs p2core against
the node, which bit-bangs the flash over the EC32MB nets.

`--disable-containers` matters on macOS: `configure` otherwise hangs in
`docker version`. `--enable-pie` is what lets the objects link into a Rust
test binary, which is position-independent; leave it off on macOS, whose
toolchain builds position-independent executables anyway and fails
`configure`'s `-pie` probe.

`embsim-p2-qemu`'s `build.rs` refuses a tree that is not what this directory
says it should be: a file here that differs from the staged copy, a tree
without `host-thread.patch`, or a stale build that still links `flashbus.c.o`.
Re-stage and run `ninja` after an edit here, and the next `cargo test` relinks.

## Adding an instruction

1. Write `trans_<name>` in `translate.c` (hub-exec) and `iexec_<name>` in
   `interp.c` (cog-exec). Anything with machinery behind it — the pin bus, the
   lock pool, CORDIC, hub block transfers — calls `op_helper.c`'s shared
   helpers from both.
2. Add the name and each operand form (`_2`, `_3`) to `TRANS` and/or `INTERP`
   in `gen_stubs.py`. They are matched by exact name.
3. Re-stage and run `ninja -C <build>`. A listed name with no function, or a
   function with no listed name, fails the build.
4. Diff it against p2core with MaD's `SIL/p2core/tools/difftest.sh` (`cog=1`
   for the interpreter).

## Host-facing flags (`target-p2/pinbus.h`)

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
state, registers, flags, stack pointer and cycle count (MaD's
`SIL/p2core/tools/fwtest.sh`).

220 DecodeTree patterns are hand-written in each engine, covering every
mnemonic the firmware executes: 80 in hub space, 41 in cog space, measured over
20 M instructions with MaD's `SIL/p2core/examples/ophist.rs`.

It also **boots the way silicon does**, on the node rather than in
`qemu-system-p2`. `p2-qemu/tests/rom_boot_ec32mb.rs` puts nothing in hub but
Parallax's own 16 KB boot ROM; the ROM samples the pull-up strap on P61,
bit-bangs the module's SPI flash over the nets, loads its first kilobyte,
verifies the 256 longs sum to `"Prop"`, copies them into cog RAM and jumps.
That kilobyte is this repository's stage-1 loader, which reads the application
from flash `$400` and relaunches the cog on it. The whole chain is **identical
to p2core's, state by state** (MaD's `SIL/p2core/tools/romtest.sh`), and the
two things it actually DID -- the flash served reads at `0` and `$400`, and
the booted payload's byte reached the debug pin -- are the same assertions
MaD's `SIL/p2core/tests/rom_boot_chain.rs` makes. The flash model is
`models/src/spi_flash.rs`, a node on the nets.

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

## Design decisions

"Spike Nx" figures quoted in these sources are measurements recorded in MaD's
`docs/dev/p2-qemu-target-plan.md`
(RileyMcCarthy/MaD@5ef1d19c3a94d64365c974bca00f891a528018b7).

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
| `romtest.sh` | the boot ROM on the node, diffed against p2core |

Here, `p2-qemu/tests/rom_boot_ec32mb.rs` boots the same chain through the
node, on the P2-EC32MB board, over nets — `p2-qemu-boot` in CI, in
`ci-gate.needs`.

## What it does NOT do yet

- **No SD card and no serial peer in the standalone binary.**
  `target-p2/pinbus.c` is the bring-up model for the CPU differential tests.
  On embsim's engine (`embsim-p2-qemu`) the flash is a board component and
  the SD card can be; the UART peers are the next seam.
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
- The silicon goldens in MaD's `SIL/p2core/hwtest/` are not yet diffed against
  this target.

## License

- The target, the board and their build files here (`target-p2/`, `hw-p2/`,
  the `.mak` files, `stage.sh`) are LGPL-2.1-or-later, as QEMU targets are:
  [`LICENSES/LGPL-2.1-or-later.txt`](../../LICENSES/LGPL-2.1-or-later.txt).
- `target-p2/insn.decode` derives from PNut-TS and is MIT
  ([`LICENSE-PNut-TS`](LICENSE-PNut-TS)). It is generated, so it carries no
  licence line of its own.
- Each patch carries the licence of the QEMU files it modifies, stated at its
  top: `register-p2.patch` GPL-2.0-or-later; `host-thread.patch` MIT for
  `tcg-accel-ops-rr.c` and GPL-2.0-or-later for `tcg-accel-ops-rr.h`.
- `../hostdrive.c`, like the rest of `embsim-p2-qemu`, is MIT.

Linked into QEMU, the whole is a GPL-2.0-or-later work: a `qemu-system-p2`
built from a tree staged from here, or an `embsim-p2-qemu` test binary built
with `EMBSIM_QEMU_P2_BUILD`, is distributable only under the GPL, version 2 or
later, with its corresponding source.
