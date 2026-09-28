# `embsim-p2-qemu` — the QEMU Propeller 2 target as a board component

A `P2X8C4M64P` that boots the way silicon does: Parallax's own 16 KB boot ROM
in the top of hub, and everything else arriving over pins that are **nets**.
The node drives pads and senses pads. It knows nothing about flashes, cards or
UARTs — those are other components on the board, and the P2 sees them the way
it sees anything else on a wire.

`tests/rom_boot_ec32mb.rs` is the whole claim in one test: the ROM, on the
P2-EC32MB board from its vendor netlist, bit-bangs the module's SPI flash
(`embsim_models`' generic part) over four shared nets, loads stage-1, which
loads and runs a program — and every one of the ~16 600 clock edges lands on
the net at the guest's own instant, two ROM instructions apart, never
collapsed and never a slice late. The same boot diffs state-for-state against
p2core (`SIL/p2core/tools/romtest.sh` in MaD) — 60 000 states identical.

## Building it

QEMU is linked into the process as a library. The crate finds a configured
QEMU build tree through one variable:

```bash
EMBSIM_QEMU_P2_BUILD=/path/to/qemu/build-p2 cargo test -p embsim-p2-qemu
```

Unset, the crate compiles to a stub: `P2Qemu::with_boot_rom` returns
`Err(P2QemuError::Unavailable)`, the integration test prints `SKIPPED` and
asserts nothing, and the workspace builds on a machine with no QEMU.

Making that tree is [`qemu-target/README.md`](qemu-target/README.md): QEMU
`v10.1.0`, the `target/p2` and `hw/p2` sources from this directory, two
patches. `build.rs` then replays QEMU's own link line out of `build.ninja` —
every object but `system_main.c.o` (the only `main()`) and
`target_p2_flashbus.c.o` (the standalone emulator's flash bus, which drags in
a second Rust runtime through `embsim-cffi`) — and compiles `hostdrive.c`
against QEMU's headers with the target's own flags. `.github/workflows/ci.yml`'s
`p2-qemu-boot` job does all of it from a clean checkout.

## Where the CPU runs, and why

On the engine thread, called from the node's own wake — the same shape `p2iss`
runs `p2core` in. QEMU's vCPU thread parks at start-up (`host-thread.patch`);
the engine thread registers itself with RCU and TCG and runs the round-robin
loop's own slice triple, one cog and one bounded instruction budget at a time
(`hostdrive.c`). Spike 1d measured a slice at 172–254 ns, exact to the
instruction, against 10 900 ns for a cross-thread park/wake. A pin edge is a
function call.

## How an edge gets its instant

**The guest leads; the engine follows it to each edge.**

1. A wake runs the cogs forward from their own clocks (round-robin, the
   least-advanced running cog's clock is the machine's "now", as in p2core).
2. The moment a cog changes a pad, the bus records the cog's clock as the
   edge's instant, sets `p2_pinbus_yield`, and calls `cpu_exit()`. The
   interpreter returns after that instruction; a translated block ends after
   it (`p2_pin_ops_end_tb`). `cpu_exec` returns to the host.
3. The wake arms itself at that instant and returns. The engine advances
   there.
4. The next wake **publishes** the drive — so it is stamped at the guest's
   instant, not the wake's — and arms one nanosecond on.
5. The engine resolves the net and delivers any response (a flash presenting
   its next bit) before the wake after that resumes the guest.

Two wakes per edge, each edge at its true instant, and a device on the net sees
every transition. These are rules R1–R5 of `docs/dev/sil-unified-drive.md` in
MaD, made concrete.

## Where the nanoseconds come from

A cog's `clocks` counts system-clock ticks whatever the frequency; the node
turns them into nanoseconds with the frequency the guest itself set. The chip
comes up on RCFAST (20 MHz nominal) and stays there until the guest writes a
clock word with `HUBSET` — the ROM never does; a flexspin program sets its
PLL in its first instructions. The target records that word and the cog
clock it was written at (`p2_clock_mode`), and the node decodes it: RCFAST,
RCSLOW, the crystal on `XI` (`with_crystal_hz`, 20 MHz by default — the
P2-EC32MB's TCXO), or the PLL `crystal / (D+1) * (M+1) / P`. A change adds a
segment to a piecewise mapping, so instants before it keep their timestamps.

Hub `$14`, where loaders store `clkfreq`, is deliberately not consulted: the
boot ROM overwrites it with its base64 table, and reading it placed the first
flash edge at fourteen seconds.

## What the bus models

`P2PinBusOps` (`qemu-target/target-p2/pinbus.h`), in Rust:

- `DIRx`/`OUTx` are per-cog registers and the pad sees the OR across all
  eight. A bank reads what the guest drives where DIR is set and what the net
  presents elsewhere — which is why the ROM can use P61 as both a strap and a
  chip select, and float P58 to read the flash.
- `TESTP` on a pin with no smart-pin mode reads its **level**. In an ADC mode
  it reads the (unmodelled) bit stream as zeros, matching p2core; a receiver
  reports no byte waiting. Anything else configured reads ready.
- `WYPIN` is recorded per pin as an instruction-stream tap (`console(pin)`),
  configured or not: the P2's own boot chain writes its debug byte without ever
  configuring the pin. Framing bytes onto a net as UART levels is the next
  seam, not this one.
- A net that floats or contends holds the pin's last level. An unresolvable
  net is not a logic value; inventing one hides the fault.

## One machine per process

`qemu_init` is process-global and not repeatable. The first
`P2Qemu::with_boot_rom` boots it; a second in the same process is refused with
`AlreadyBooted`. Put each system that needs a P2 in its own test binary.

## The state trace, and a warning

`EMBSIM_P2_QEMU_TRACE=<file>` makes the boot test pass `-d cpu -D <file>` to
QEMU: one `P2STATE` line per instruction, the log `romtest.sh` diffs against
p2core. It is unbounded — about 450 bytes an instruction — and a payload that
spins without reaching its byte will fill a disk in minutes. The test cuts its
wait to 20 s when tracing; do not leave a traced run unattended.

## Facts of the board the boot test states as scenario

- `S301` position 2 (FLASH) closed: `P61` reaches the flash's `~CS`.
- `S301` position 4 (P59 pull-down) closed: the ROM boots the program it
  loaded instead of waiting for a serial loader. It really samples this —
  drives P59 high, floats it, waits, reads it back.
- `GND` stuck at 0 V and `VIO_56_63` at 3.3 V: the engine has no idea of
  ground, and without the first, ground is just another node the pull-ups
  reach, which reads high.

Each of those was a divergence from p2core before it was a line of scenario.
