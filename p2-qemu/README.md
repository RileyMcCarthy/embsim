# `embsim-p2-qemu` — the QEMU Propeller 2 target as a board component

A `P2X8C4M64P` that boots the way silicon does: Parallax's own 16 KB boot ROM
in the top of hub, and everything else arriving over pins that are **nets**.
The node drives pads and senses pads. It knows nothing about flashes, cards or
UARTs — those are other components on the board, and the P2 sees them the way
it sees anything else on a wire.

`P2Qemu` is the **core** inside `embsim_boards::p2::P2Package`: the package
declares the 86 package pins (64 pads released, the rails, `RESN`/`TEST`
sensed, `XI` a rate sink, `XO` released), hands the core its pads, and
delivers the two package-level facts — the crystal, which is whatever rate
the board puts on `XI`, and the `RESN`/`VDD` state. A board fills its
processor slot with `P2Package::new(P2Qemu::with_boot_rom(…)?)`.

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
RCSLOW, the crystal on `XI`, or the PLL `crystal / (D+1) * (M+1) / P`. A
change adds a segment to a piecewise mapping, so instants before it keep
their timestamps.

The crystal is not a number handed to the node: it is the **rate the board
delivers on `XI`** (the P2-EC32MB's TCXO, through its buffer and coupling
capacitor), which the package passes to the core as it arrives. A guest
that selects a crystal-derived clock while nothing reaches `XI` has no
clock and **stalls** — no instructions run — until a rate arrives, at which
point its clock segment starts at that instant (`tests/crystal_pll.rs`).

Hub `$14`, where loaders store `clkfreq`, is deliberately not consulted: the
boot ROM overwrites it with its base64 table, and reading it placed the first
flash edge at fourteen seconds.

## What the bus models

`P2PinBusOps` (`qemu-target/target-p2/pinbus.h`), in Rust:

- `DIRx`/`OUTx` are per-cog registers and the pad sees the OR across all
  eight. A pad the guest drives is a Thevenin source at the strength its
  `WRPIN` word configured (`embsim_boards::p2::pad_drive`: fast, 1.5 k /
  15 k / 150 kΩ, float; the current-source modes are not mapped and present
  nothing), and a `WRPIN` on a driven pad republishes it at its new
  strength. A bank reads the guest's own `OUT` bit where the pad's published
  drive is fast, and the **net** everywhere else — released pads and pads
  pulling through a resistive mode alike. That is why the ROM can use P61 as
  both a strap and a chip select and float P58 to read the flash, and why a
  pad pulling a line high through 15 kΩ reads the sink holding it low
  (`tests/pad_modes.rs`) — the read an I2C master's clock stretch and ACK
  depend on.
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

## The state trace, and the p2core differential

`EMBSIM_P2_QEMU_TRACE=<file>` makes the boot test pass `-d cpu -D <file>` to
QEMU: one `P2STATE` line per instruction, the log `romtest.sh` diffs against
p2core. It is unbounded — about 450 bytes an instruction — and a payload that
spins without reaching its byte will fill a disk in minutes. The test cuts its
wait to 20 s when tracing; do not leave a traced run unattended.

The "60 000 states identical" figure the phase records cite is this
comparison, run by hand; it is reproducible from this tree and MaD's
`SIL/p2core` with the commands below (the reference and the comparison are
`romtest.sh`'s own, lifted out so the QEMU side can be the node instead of
`qemu-system-p2`). `W` is a scratch directory; the trace is deleted at the
end because it is large and carries nothing the numbers do not.

```bash
SIL=~/Documents/MaD/SIL              # the MaD checkout; p2core is not a dependency of embsim
W=$(mktemp -d)

# 1. The flash image the boot test builds (flashimage::boot_flash): stage-1
#    in the first KB balanced to "Prop", the payload's length and image at
#    $400 — the same bytes as romtest.sh's python.
python3 - p2-qemu/rom/stage1.bin "$W/flash.bin" <<'PY'
import sys, struct
stage1 = open(sys.argv[1], 'rb').read()
payload = b''.join(struct.pack('<I', w) for w in (0xF607EC42, 0xFC27EC3E, 0xFD9FFFFC))
img = bytearray(0x404 + len(payload)); img[:len(stage1)] = stage1
img[0x400:0x404] = struct.pack('<I', len(payload)); img[0x404:] = payload
PROP = struct.unpack('<I', b'Prop')[0]
s = sum(struct.unpack_from('<I', img, i)[0] for i in range(0, 0x400, 4)) & 0xFFFFFFFF
img[0x3FC:0x400] = struct.pack('<I', (PROP - s) & 0xFFFFFFFF)
open(sys.argv[2], 'wb').write(bytes(img))
PY

# 2. The reference: p2core's cog-0 state, one line per instruction, 60 000
#    of them, booting the same ROM off the same image (P2CORE_NO_FF turns
#    off the fast-forward so every state is visited).
cargo build --release --quiet --manifest-path "$SIL/Cargo.toml" -p p2core --example p2state
P2CORE_NO_FF=1 P2STATE_ROM=p2-qemu/rom/rom_booter_v33k.bin P2STATE_FLASH="$W/flash.bin" \
    "$SIL/target/release/examples/p2state" - 60000 > "$W/ref.txt"

# 3. The node's trace: the boot test, traced.
EMBSIM_QEMU_P2_BUILD=~/Documents/qemu-p2/build-p2 EMBSIM_P2_QEMU_TRACE="$W/trace.txt" \
    cargo test -p embsim-p2-qemu --test rom_boot_ec32mb -- --nocapture

# 4. Cog 0's first 60 000 states, and the comparison romtest.sh makes
#    (a QEMU line repeated where the reference does not repeat is a
#    block-entry logging artifact and is dropped; anything else stops it).
awk '/^P2STATE cog=0/ { print; if (++c >= 60000) exit }' "$W/trace.txt" > "$W/qemu.txt"
python3 - "$W/ref.txt" "$W/qemu.txt" <<'PY'
import sys
ref = open(sys.argv[1]).read().splitlines(); qemu = open(sys.argv[2]).read().splitlines()
i = j = dropped = 0
while i < len(ref) and j < len(qemu):
    if ref[i] == qemu[j]: i += 1; j += 1; continue
    if j and qemu[j] == qemu[j - 1] and not (i and ref[i] == ref[i - 1]): j += 1; dropped += 1; continue
    break
print("reference %d | qemu %d | compared %d | dropped %d" % (len(ref), len(qemu), i, dropped))
if i < len(ref) and j < len(qemu): print("DIVERGED at state %d\n  ref : %s\n  qemu: %s" % (i, ref[i], qemu[j])); sys.exit(1)
print("identical" if i else "nothing compared"); sys.exit(0 if i else 1)
PY
rm -rf "$W"
```

Recorded runs: 2026-09-23 (the node's first boot) and 2026-09-24 (the phase-2
review pass) — `compared 60000, identical`.

## Facts of the board the boot test states as scenario

- `S301` position 2 (FLASH) closed: `P61` reaches the flash's `~CS`.
- `S301` position 4 (P59 pull-down) closed: the ROM boots the program it
  loaded instead of waiting for a serial loader. It really samples this —
  drives P59 high, floats it, waits, reads it back.
- `GND` stuck at 0 V and `VIO_56_63` at 3.3 V: the engine has no idea of
  ground, and without the first, ground is just another node the pull-ups
  reach, which reads high.

Each of those was a divergence from p2core before it was a line of scenario.
