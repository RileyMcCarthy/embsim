# `embsim-p2-qemu` — the QEMU Propeller 2 target as a board component

A `P2X8C4M64P` that boots the way silicon does: Parallax's own 16 KB boot ROM
in the top of hub, and everything else arriving over pins that are **nets**.
The node drives pads and senses pads. It knows nothing about flashes, cards or
UARTs — those are other components on the board, and the P2 sees them the way
it sees anything else on a wire.

`P2Qemu` is the **core** inside `embsim_boards::p2::P2Package`: the package
declares the 86 package pins (64 pads released, the rails, `RESN`/`TEST`
sensed, `XI` a rate sink, `XO` released), hands the core its pads, and
delivers the package-level facts — the crystal, which is whatever rate the
board puts on `XI`; the `RESN`/`VDD` state; and the sixteen bank supplies,
which are what a pad drives high at. The package's **START gate** holds
the core — its start and every wake it asks for — until the datasheet's
3 ms restart delay has run out after `RESN` read released with `VDD` inside
its 1.7–1.9 V window; that instant is where the guest's clock begins. A
`VDD` that leaves its window while the guest runs, with `RESN` not asserted,
holds the guest for good (`P2Core::reset`: no further slice, the pads as
they were) and the package reports the brownout. A board fills its processor
slot with
`P2Package::new(P2Qemu::with_boot_rom(…)?)`.

`tests/rom_boot_ec32mb.rs` is the whole claim in one test: the ROM, on the
P2-EC32MB board from its vendor netlist powered from its carrier's `J203`
fingers — its own bucks, LDOs and brownout detector raising the rails and
releasing the reset 2.5 ms in, the core started by the package the
datasheet's 3 ms later, at 5.5 ms —
bit-bangs the module's SPI flash (`embsim_models`' generic part) over four
shared nets, loads stage-1, which loads and runs a program — and every one
of the ~16 600 clock edges lands on the net at the guest's own instant, two
ROM instructions apart, never collapsed and never a slice late. The same
boot diffs state-for-state against p2core (`SIL/p2core/tools/romtest.sh` in
MaD) — 60 000 states identical.

## Getting `qemu-system-p2`

QEMU is not linked into embsim. The P2 runs in `qemu-system-p2`, a program
of its own, built from the target this crate carries
([`qemu-target/`](qemu-target/README.md), embedded in the crate so any
embsim can build it) and installed once:

```bash
embsim qemu install      # fetch QEMU v10.1.0, stage the target, build, install
embsim qemu path         # which qemu-system-p2 a run would start, and whether it fits
```

`install` checks out QEMU at the tag and commit `qemu-target/QEMU_PIN`
names, stages the target with `qemu-target/stage.sh`, configures the one
target and none of QEMU's default features (`install::CONFIGURE`; an
8 MB binary needing only the system's libraries and glib), builds with
ninja and installs into `~/.embsim/qemu/<identity>/` (or `--prefix DIR`),
then starts the installed program and checks its handshake. It needs git, a
C compiler, ninja, pkg-config, glib's development files and python3, and
says how to get them when one is missing; a few minutes the first time.
`--build-dir DIR` builds there (and an install that stopped picks up where
it was), `--keep-build` keeps it, `--qemu-git URL` fetches from a mirror or
a local clone (the pinned commit is checked whatever the source),
`--dry-run` says what it would do.

The node finds the program in three places, in order: the file
`EMBSIM_QEMU_SYSTEM_P2` names; `qemu-system-p2` on `PATH`; and
`~/.embsim/qemu/<identity>/`. `<identity>` is the target's identity
(`target::identity`): the first 16 hex digits of the SHA-256 over the
target's sources, which `stage.sh` computes the same way (`stage.sh
--identity` prints it). The program reports it in its handshake, with its
protocol version and its QEMU version, and a program that differs in any is
refused, the error saying which and how to install the matching one — so
two embsims whose targets differ keep two programs side by side, and an
out-of-date program is never driven by a newer node.

With no program to find, the crate still builds and its unit tests run;
starting a node fails with where it looked and how to install one. The
tests that boot the P2 (`tests/rom_boot_ec32mb.rs`, `crystal_pll.rs`,
`pad_modes.rs`, `lifecycle.rs`) are `#[ignore]`d for that reason, so a
workspace run without QEMU cannot report a boot it did not run; with one
installed:

```bash
cargo test -p embsim-p2-qemu -- --include-ignored
EMBSIM_P2_QEMU_TRANSPORT=socket cargo test -p embsim-p2-qemu -- --include-ignored
```

CI's `p2-qemu-boot` job installs the program with `embsim qemu install`
(cached on the target's identity and the QEMU pin) and runs exactly those,
on both channels, and the command's own QEMU tests. `tests/program.rs`
runs everywhere: a stand-in program speaking the protocol proves the node's
half — the refusals, a death reported with its status and standard error,
a program that will not quit killed — and that `stage.sh` and the crate
compute one identity.

### The licence of the program

`qemu-system-p2` is QEMU, and QEMU as a whole is GPL-2.0: the installed
program is a GPL-2.0 work. `install` puts QEMU's licence texts beside it,
the target sources it was built from under `source/`, and a `NOTICE`
saying how it was built — its corresponding source. embsim runs it as a
separate program and talks to it over a small fixed protocol (a shared page
or a socket pair, `src/protocol.rs`); it links none of QEMU, and nothing of
embsim is in the program. The crate's Rust is MIT, as every embsim
crate's is; it also carries the target's sources, each file under its own
licence (`qemu-target/README.md`, "License"), and embeds them verbatim in
every binary that links it, as data it never compiles — so its licence is
`MIT AND LGPL-2.1-or-later AND GPL-2.0-or-later`. This is a statement of
how the pieces are put together, not legal advice.

## In a project

`catalog::QemuCores` is a core catalog of one kind, `qemu`, for the `p2`
part kind, and `catalog::register` adds it to a catalog set: the package
holds a `P2Qemu` that boots `BOOT_ROM` (or the file the core's `rom` option
names) when the board is built, never when it is only surveyed. Seating the
entry finds the program, and with none refuses it saying how to install
one. Each part the key reaches boots a program of its own. The core
reports its console, per pad, its yields, and — if its program died — why
it stopped. The `embsim` command's set (`embsim_cli::shipped`) holds it:
`embsim run` on a project whose P2 is `core = "qemu"` boots the ROM off the
board's flash, as `tests/rom_boot_ec32mb.rs` does.

## Where the CPU runs, and why

In `qemu-system-p2`, in host-driven mode (`qemu-target/target-p2/hostipc.c`,
turned on by the machine property `-M p2,hostipc=…` the node passes). QEMU's
own vCPU thread parks at start-up (`host-thread.patch`); a thread of
`hostipc.c` registers with RCU and TCG and runs the round-robin loop's own
slice triple, one cog and one bounded instruction budget at a time, as the
node asks.

The node and the program take turns in lockstep: one RUN — run the cogs
until this cog clock, with these net levels, these pads reading their own
`OUT` and these banks powered — and one STOP — why it stopped (a pad
changed, the horizon, the clock word, nothing could run), at what cog
clock, and what the guest now drives — per stop (`src/protocol.rs`). The
guest-facing half of the pin bus lives in the program, next to the CPU, so
not one pin operation crosses: every read inside a slice is answered from
the RUN's snapshot, because a slice ends at the first instruction that
changes what the chip drives and the nets move only between slices. The
electrical model — what a pad presents, the banks, the nets, the clock —
stays here. The program decides only *whether* a pad's drive changed, by a
key that is equal exactly when the node's drive is equal given the bank
states each RUN carries.

Over the shared page each side spins 20 µs for the other's turn, then
blocks on a futex (`__ulock` on macOS, `futex(2)` on Linux; both in CI), so
a turn costs a fraction of a microsecond while both turns are short; a
socket pair is the fallback (`EMBSIM_P2_QEMU_TRANSPORT=socket`, about 5 µs
a turn). Measured by the spike that chose this (2026-10-01, the M2 under
load): the ROM boot in 29–34 ms against 28–32 ms with QEMU linked, the
transport adding 0.24–0.39 µs a stop; the boot's edges, instants, yields,
publishes and 60 000 p2core states, and a two-cog race over 1.2e8
instructions, bit-identical across both transports and linked. The cost is
a second core spinning while the two take turns; with the engine's turn
longer than the spin, a turn pays a futex wake (about 5 µs).

The program never outlives its node. It is started in a process group of
its own (a terminal's ^C reaches embsim, whose run ends cleanly, and not
it), with three descriptors and nothing on disk: the channel, the boot ROM
as an unlinked file read as `/dev/fd/N`, and the read end of a pipe whose
write end only embsim holds — when embsim dies, however it dies, the
program's watch thread reads end of file and the program exits at once.
Dropping a node asks its program to quit and kills it if it has not within
a second. A program that dies is noticed within 100 ms (a blocked wait asks
the kernel every 100 ms; the socket closes at once) and reported with its
exit status and the last of its standard error, which the node also passes
through to embsim's own; the core then runs no further
(`P2QemuHandle::failure`), and `embsim run` stops there and exits
non-zero. One that lives but has not answered a turn in 30 s is killed and
reported as unresponsive. A program sent `SIGTERM`, `SIGINT` or `SIGHUP`
ends at once: QEMU's own handler takes the signal, and the program exits
from its main loop before QEMU's shutdown would pause the parked vCPUs
(`qemu-target/README.md`, "Host-driven mode").

No wall-clock input reaches the guest. A slice is exactly the instruction
count asked for: the parked vCPU thread parks before it arms QEMU's kick
timer, the board arms no quantum timer, and `host-thread.patch` keeps
real-time timers out of the icount limit in host-driven mode, so no timer a
main-loop option arms can cut a slice short; and it makes a kick from any
thread that runs no cog do nothing, so neither can QEMU's main loop.

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
RCSLOW, the crystal on `XI`, or the PLL `crystal / (D+1) * (M+1) / P`. `XI`
and the PLL are read with the fields the datasheet's `%SS` notes name
(System Clock, p. 18): `XI` only with its input on, `%CC` ≠ `%00`, and the
PLL only with that and `%E` set — a word that selects either without them
has no clock, and the guest stalls for good. A change adds a segment to a
piecewise mapping, so instants before it keep their timestamps.

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

The guest-facing bus, in `qemu-target/target-p2/hostipc.c`:

- `DIRx`/`OUTx` are per-cog registers and the pad sees the OR across all
  eight. A pad the guest drives is a Thevenin source at the strength its
  `WRPIN` word configured (`embsim_boards::p2::pad_drive`: fast at
  `P2_FAST_OHMS`, 17.99 Ω fitted to the datasheet's `Voh`/`Vol` table,
  1.5 k / 15 k / 150 kΩ, float; the current-source modes are not mapped and
  present nothing), and a `WRPIN` on a driven pad republishes it at its new
  strength. A bank reads the guest's own `OUT` bit where the pad's published
  drive is fast, and the **net** everywhere else — released pads and pads
  pulling through a resistive mode alike. That is why the ROM can use P61 as
  both a strap and a chip select and float P58 to read the flash, and why a
  pad pulling a line high through 15 kΩ reads the sink holding it low
  (`tests/pad_modes.rs`) — the read an I2C master's clock stretch and ACK
  depend on.
- A pad's high is its bank's supply; a pad driven in a bank whose supply
  names no voltage presents nothing and the package reports the bank once;
  a supply that moves takes the pads driven in its bank with it at the
  node's next wake.
- `TESTP` on a pin with no smart-pin mode reads its **level**. In an ADC mode
  it reads the (unmodelled) bit stream as zeros, matching p2core; a receiver
  reports no byte waiting. Anything else configured reads ready.
- `WYPIN` is recorded per pin as an instruction-stream tap (`console(pin)`),
  configured or not: the P2's own boot chain writes its debug byte without ever
  configuring the pin. Framing bytes onto a net as UART levels is the next
  seam, not this one.
- A net that floats or contends holds the pin's last level. An unresolvable
  net is not a logic value; inventing one hides the fault.

## One program per P2

Each `P2Qemu` starts a `qemu-system-p2` of its own, so a board may carry two
P2s and a test binary may hold as many as it likes: `tests/pad_modes.rs`
runs two on one bench, each reading its own nets.

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

# 1. The flash image the boot test builds (`embsim flash-image`, over
#    flashimage::boot_flash): stage-1 in the first KB balanced to "Prop",
#    the payload's length and image at $400 — the same bytes as
#    romtest.sh's python. The payload is mov pa,#"B" / wypin pa,#62 / jmp #$.
printf '\102\354\007\366\076\354\047\374\374\377\237\375' > "$W/payload.binary"
cargo run --quiet -p embsim-cli -- flash-image "$W/payload.binary" -o "$W/flash.bin"

# 2. The reference: p2core's cog-0 state, one line per instruction, 60 000
#    of them, booting the same ROM off the same image (P2CORE_NO_FF turns
#    off the fast-forward so every state is visited).
cargo build --release --quiet --manifest-path "$SIL/Cargo.toml" -p p2core --example p2state
P2CORE_NO_FF=1 P2STATE_ROM=p2-qemu/rom/rom_booter_v33k.bin P2STATE_FLASH="$W/flash.bin" \
    "$SIL/target/release/examples/p2state" - 60000 > "$W/ref.txt"

# 3. The node's trace: the boot test, traced (the installed qemu-system-p2;
#    the trace is flushed when the node asks the program to quit).
EMBSIM_P2_QEMU_TRACE="$W/trace.txt" \
    cargo test -p embsim-p2-qemu --test rom_boot_ec32mb -- --include-ignored --nocapture

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

Recorded runs: 2026-09-23 (the node's first boot), 2026-09-24 (the phase-2
review pass), and 2026-09-24 again with the module powered from its `J203`
fingers and the core started by the package's START gate at 2.5 ms (phase
4), 2026-09-25 with every pad read through the package's own projection
(the interface phase's sense task, `NODES.md` §12 item 5), 2026-09-25 again
with the fights reported beside an analog reader's operating point, the
single-source rule and the stamped input ports and clamps (its rules task),
and 2026-09-25 once more with the core started 3 ms after the reset releases
(5.5 ms), the fast pads at 17.99 Ω and the pulse schedule in nanoseconds
(its P2 task; the p2core reference reused, p2core not having moved), and
2026-09-25 after that phase's review (a clock's phases combined on their
voltages, a supply's move re-delivering its pads, the resolver's scratch
buffers; the reference reused again), and 2026-09-26 with the clock decode
reading `%CC` and `%E` beside `%SS` (the final lows; the reference reused),
and 2026-10-02 with QEMU out of process, the `qemu-system-p2` `embsim qemu
install` built (`NODES.md` §14; the reference regenerated from MaD's
prebuilt `p2state`, byte-identical to the last)
— `compared 60000, identical` each time.

## Facts of the board the boot test states as scenario

- `S301` position 2 (FLASH) closed: `P61` reaches the flash's `~CS`.
- `S301` position 4 (P59 pull-down) closed: the ROM boots the program it
  loaded instead of waiting for a serial loader. It really samples this —
  drives P59 high, floats it, waits, reads it back.
- Nothing is stuck. The module is powered the way a carrier powers it —
  5 V on `J203`'s two `5V` fingers and 0 V on its three `GND` fingers, as a
  harness — and every rail the boot depends on is a part's output: the
  ground the pull-downs return to is the finger's terminal, the `VIO_56_63`
  rail `R301` pulls the boot strap to is an LDO's, the core rail the START
  gate reads is `U402`'s, and the TCXO's 20 MHz reaches `XI` from the same
  rail. The engine has no idea of ground: without the declared return,
  ground is just another node the pull-ups reach, which reads high.

Each of those was a divergence from p2core before it was a line of scenario
or a wire of harness.
