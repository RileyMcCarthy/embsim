# embsim

[![CI](https://github.com/RileyMcCarthy/embsim/actions/workflows/ci.yml/badge.svg)](https://github.com/RileyMcCarthy/embsim/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

A generic **software-in-the-loop (SIL) emulator framework** for embedded firmware.

embsim links your real firmware C code against Rust implementations of its
hardware-access (HAL) layer, plus emulated peripherals and physical models, so
the firmware runs unmodified on a host with **no physical hardware**. Host
software (a desktop app, a test harness) talks to the emulated serial port
through a `/dev` PTY symlink, exactly as it would to a real board.

It was extracted from the [MaD tensile tester](https://github.com/RileyMcCarthy/MaD)
and is designed to be reused: the `core`, `peripherals`, `models`, `runtime`,
and `tools` crates carry no project- or Propeller-2-specific assumptions. A new
project supplies a *platform crate* and a *machine*, and gets a runnable
emulator.

## What embsim is for, and what it is not

embsim validates a **PCBA with its firmware** before the board exists: the real
firmware binary on an instruction-set simulator, every part on the board a node,
and the nets between them resolved as circuits at the instants things happen.
It sits between a unit test with a mocked HAL and the bench. It is not SPICE
and it is not a transaction-level emulator; it is deliberately in between, and
that is where it finds the bugs the two ends cannot: a fought line, a missing
pull-up, a floating input, a strap on the wrong pin, a shared bus that clobbers
itself, a boot that needs a switch position — while running fast enough to gate
every pull request.

**What it validates**

- **The design as drawn.** The vendor netlist is the model. Every net, every
  pin, every resistor, switch position, jumper, DNP and rail is what the
  firmware sees; a part the registry cannot classify is a build error naming it.
- **Every operating point.** Levels, drivers against pulls, two drivers on one
  net, floating inputs, rails up or down, enable trees, brown-out and reset
  order — resolved as a circuit (Thevenin sources, resistor networks, a nodal
  solve where sources compete) at each event, and reported as findings.
- **Protocols on the wires, bit by bit.** SPI on shared pins, I2C as open-drain
  and pull-up (wired-AND, clock stretching, arbitration), UART framing at the
  real baud, step/dir with exact counts, differential receivers with their
  failsafe — carried as levels on nets, not as bytes handed across.
- **Timing at the event level.** Each edge lands on the net at the guest's own
  nanosecond; RC delays are single-pole closed forms; the CPU's clocks come
  from the clock it set. Virtual time is stepped and deterministic, so a run
  is reproducible bit for bit and a trace can be a golden.
- **The firmware, whole.** The real ROM boot off the real flash, the real HAL,
  all cogs interleaved, the shipped app end to end — in CI, on every push.
- **What-ifs, cheaply.** Switch and jumper positions, DNP parts, shorts,
  detached pins, stuck rails, missing parts: a line of scenario each, and the
  same run again.

**What it does not model** — the limits, so they are never a surprise

- **Signal integrity.** No transmission lines, reflections, ringing, crosstalk
  or ground bounce. An edge is instantaneous unless a capacitance is declared,
  and then it is one RC pole.
- **Transients beyond one pole.** No multi-pole filters (one stated exception,
  the symmetric differential pair), no inductor dynamics, no resonance, no
  switching-regulator ripple. A regulator is a DC source with a soft-start
  instant.
- **Nonlinear devices beyond regions.** A diode is on or off, a FET on or off,
  a transistor saturated or active — piecewise-linear regions in one DC solve,
  not curves. No amplifier loops or oscillator start-up beyond what a model
  declares.
- **Tolerances and corners.** Values are nominal. No temperature, aging or
  Monte Carlo unless a scenario overrides a value.
- **Power integrity.** No current budgets, IR drop, regulator current limits
  or thermal, unless a model declares a load.
- **Noise and metastability.** Digital levels are projected through declared
  thresholds; there is no noise, no glitch filtering beyond a declared RC.
- **The CPU below the instruction.** The P2 targets are instruction-accurate
  and differentially checked against each other and against silicon captures,
  not cycle-exact: two clocks per instruction, hub windows approximated,
  interrupts and some smart-pin modes not yet modelled. Firmware timing
  measurements are approximate.
- **Faults you did not name.** A short, a detached pin or a stuck rail is
  found only when a scenario injects it; the tool does not guess at
  manufacturing defects.
- **EMI/EMC, ESD, mechanical.** Out of scope.

**The path.** What can be simulated, today and by phase of [`NODES.md`](NODES.md); the last column is out of scope by design.

| Area | Today | The plan adds | Not in scope |
|---|---|---|---|
| Node interface | one per-instant message, `Drive` — a Thevenin port, a current, or a periodic drive (the step clock, two ports and an integer-nanosecond schedule); one delivery, `Sense` — the node's voltage against the pin's declared reference, or none; a pin declared once as a `PinRole` plus its facts (thresholds with hysteresis and a dead-band policy, input port, clamps, reference, supply); every level projected by its receiver (`NODES.md` §11, §12 item 5) | a pin's declared capacitance stamped as its RC pole (5) | a second channel between nodes |
| Connectivity | every net and pin from the vendor netlist, facade checked both ways; resistors as circuit edges; jumpers | switches, capacitors, diodes, FETs, regulators, oscillators, gates as nodes; unclassified part = build error (phases 1–4) | parasitics the netlist does not name |
| DC operating point | drivers vs pulls, contention, floating, stuck rails, a nodal solve where sources compete | impedance-aware ranking (a 15 kΩ pull vs a sink is not contention), real rail voltages, ground as a declared terminal, LED lit, body diodes, the missing-pull-up lint (1, 3, 4) | current budgets, IR drop, thermal |
| Protocols on wires | SPI on shared pins, UART as levels at real baud, step/dir as exact counts, RS-422 receivers, the ROM boot off the flash | I2C wired-AND with clock stretching and arbitration, the P2 pad reading the net in its pull modes (2, 6); CAN/USB only if a model is written | transaction-level bus models |
| Timing | every edge at its own nanosecond; deterministic stepped clock; golden traces | RC delays as one pole with exact integer-ns crossings; rail soft-start instants; symmetric differential filters (5) | slew, setup/hold against slow edges, multi-pole transients, ringing |
| Power | rails present or absent; enable trees as senses | rails as sources with soft-start and UVLO; supervisor with hysteresis; isolated domains; brown-out ordering (4) | regulator ripple, current limit, load transients |
| Analog | ADS122U04 front end at settled values; force and encoder plants; input ports stamped on senses (the AM26LV32's open-input bias); a single source into an analog reader delivered unsolved | RC settling at conversion instants (5) | noise, amplifier loops, oscillator start-up |
| CPU | P2 on QEMU or p2core, instruction-accurate, verified against each other and silicon captures; hub-exec and cog-exec; HUBSET clock; inside the P2 package, any core — QEMU, an ISS, the native firmware — started 3 ms after `RESN` releases inside `VDD`'s window, its pads at their bank's supply and the `WRPIN` strengths (fast fitted to the datasheet), the crystal the rate on `XI`, a brownout without a reset reported and the core held | — (a `RESN` pulse after START re-running the boot is open: no core has the entry) | cycle-exact hub timing, interrupts until modelled |
| Faults and what-ifs | shorts, detached pins, stuck nets, DNP, value overrides, jumper states | switch positions by name, declared leaks, capacitance on a harness (1, 5) | faults nobody injects; tolerances and corners |
| Speed | 16 901 flash edges in 0.2 s; step trains as rates | a census and a solve benchmark as CI gates; nothing added to the fast path (0, 6) | a timestep, ever |

**Why it stays fast.** Nothing is integrated per tick: cost is per event, and a
solve runs only where sources within a factor of ten disagree or an analog
sense asks. Everything else is a projection. A ROM boot that bit-bangs 16 901
flash edges through the net engine takes 0.2 s wall; a step train travels as a
rate, one event per rate change, because 820 000 edges a second was measured
and refused.

## Crate layering

```
   consumer      your board, your CPU core, your host on the PTY
                      │
   boards        embsim-boards     off-the-shelf modules (P2-EC32MB) and the P2 package
   cpu           embsim-p2-qemu    QEMU Propeller 2, pads on nets
                      │
   board         embsim-board      netlist, nets, one quasi-static solve
   models        embsim-models     flash, SD, regulators, gates, the plant
                      │
   core          embsim-core       virtual clock, serial PTY, observers
```

The dependency graph is acyclic: **no generic crate depends on a project crate.**
Project-specific wiring lives in the consumer's repo.

## Repository layout

| Crate | Path | What it is |
|-------|------|------------|
| `embsim-core` | [`core/`](core) | Virtual clock, serial PTY, event observers |
| `embsim-board` | [`board/`](board) | Netlist ingestion, net resolution, the one drive/sense interface |
| `embsim-models` | [`models/`](models) | Device models: ADS122U04, serial NOR flash, SD card, FAT16, regulators, gates, oscillators |
| `embsim-p2-qemu` | [`p2-qemu/`](p2-qemu) | The QEMU Propeller 2 target as a board component: boots the real ROM off a flash on the board's nets. Carries the `target/p2` sources |
| `embsim-boards` | [`boards/`](boards) | The P2-EC32MB from its vendor netlist, and the P2 package a core sits in |
| `embsim-memory-inspect` | [`tools/memory-inspect/`](tools/memory-inspect) | DWARF reader — recover C enums/structs/variables from an archive |
| `embsim-trace` | [`tools/trace/`](tools/trace) | Time-series trace recorder + live web viewer (feature `web`) |
| `embsim-ui` | [`tools/ui/`](tools/ui) | Pluggable web shell the trace viewer mounts into |
| `embsim-cpu-oracle` | [`cpu-oracle/`](cpu-oracle) | ISS-vs-silicon golden records (parse, diff). CPU adapters supply the image and ISS. |

The plan for making every netlist part a node in one pipeline — switches, capacitors, diodes, rails, the P2 package — is [`NODES.md`](NODES.md).

## What a new project provides

A board from its netlist, a core in the processor slot, and a host on the PTY.
The core drives and senses pads. The host is [`HostPty`](board/src/host_pty.rs):
bytes on `TX`/`RX` become levels on the nets. The P2-EC32MB and the QEMU P2
core are the reference:

```rust
let p2 = embsim_p2_qemu::P2Qemu::with_boot_rom(&rom, &[])?;
let board = embsim_boards::ec32mb::Ec32mb::new()
    .with_p2(|_decl| Box::new(embsim_boards::p2::P2Package::new(p2)))
    .build()?;
```

## Using embsim in your project

embsim is a Cargo workspace of path crates (not yet on crates.io). Consume it
as a **git submodule** and point path dependencies at the crates you need:

```bash
git submodule add https://github.com/RileyMcCarthy/embsim.git vendor/embsim
```

```toml
# your-emulator/Cargo.toml
[dependencies]
embsim-core   = { path = "../vendor/embsim/core" }
embsim-board  = { path = "../vendor/embsim/board" }
embsim-models = { path = "../vendor/embsim/models" }
embsim-boards = { path = "../vendor/embsim/boards" }
```

Your workspace should `exclude` the submodule directory (embsim is its own
workspace root) — path dependencies across the boundary work fine:

```toml
[workspace]
exclude = ["vendor/embsim"]
```

## One QEMU P2 per OS process

`P2Qemu::with_boot_rom` boots a process-global QEMU. A second call in the same
process is refused. Run one machine per process.

## Building & testing

Every crate is testable **without any firmware**:

```bash
cargo build --workspace            # build everything
cargo test  --workspace            # run every crate's suite
```

Per-crate, if you want to iterate on one area:

```bash
cargo test -p embsim-core           # virtual clock, observers, serial PTY
cargo test -p embsim-models         # ADS122U04, flash, SD, regulators, the plant
cargo test -p embsim-p2-qemu        # P2 core: stub without a QEMU tree; EMBSIM_QEMU_P2_BUILD=<build> boots the ROM, the pad-mode and PLL benches
cargo test -p embsim-boards         # the P2-EC32MB board against its netlist
cargo test -p embsim-memory-inspect # DWARF parser (compiles a tiny C fixture at test time)
cargo test -p embsim-trace          # trace recorder
cargo test -p embsim-ui             # web shell render + handlers
```

Release-mode smoke:

```bash
cargo test -p embsim-board --release
```

Determinism (the `determinism` CI job). In **stepped clock mode** the board
engine is a discrete-event simulator in virtual time, and a scenario's engine
event log — order *and* every timestamp — is identical across runs, across
processes, and against a blessed golden trace. Free-running (the default) is
unchanged and is held only to the event order:

```bash
cargo test -p embsim-board --test determinism -- --nocapture
cargo test -p embsim-board --test stepped_clock --test ads122u04_stepped
```

The design, the honest limits, and the measured numbers are in
[`DETERMINISM.md`](DETERMINISM.md).

Coverage (optional locally; published by the `coverage` CI job):

```bash
cargo llvm-cov --workspace --summary-only
```

**Test conventions** (required for new tests) live in [`TESTING.md`](TESTING.md):
prefer `#[rstest]` + named `#[case]`s; peripheral free-function tests take
`test_support::guard()` + `ensure_clock()`; assert virtual-time contracts and
clamps rather than flaky wall timing. Consumer repos (e.g. MaD) should re-run
this suite against the pinned submodule commit on SIL-related PRs.

Platform support: Linux and macOS (the serial PTY and thread emulation use
Unix APIs; Windows is not supported). The `embsim-memory-inspect` DWARF test
compiles a small C fixture with `clang` (preferred) + `ar` and **skips
gracefully** when no C toolchain is present.

## License

MIT — see [LICENSE](LICENSE).
