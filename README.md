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
                 ┌──────────────────────────────────────────────┐
   consumer      │  your-emulator (binary)  +  your Machine impl │
   (project)     └───────────────┬──────────────────────────────┘
                                 │
   platform      ┌───────────────▼──────────────┐   #[no_mangle] HAL trampolines
   (per-MCU)     │  embsim-p2  (or your-platform)│   + Platform impl (constants)
                 └───────────────┬──────────────┘
                                 │
   framework     ┌───────────────▼──────────────┐
                 │        embsim-runtime         │   Emulator builder + Platform/Machine traits
                 └───────┬───────────────┬──────┘
                         │               │
          ┌──────────────▼──┐   ┌────────▼─────────┐
          │ embsim-peripherals│   │   embsim-models  │   GPIO/serial/encoder/…   device & IC models
          └──────────────┬──┘   └────────┬─────────┘   + EdgeDetector
                         │               │
                 ┌───────▼───────────────▼──────┐
                 │          embsim-core          │   virtual clock · serial PTY · event (Observers)
                 └───────────────────────────────┘

   tools (beside the stack):  memory-inspect (DWARF reader) · trace (live viewer) ·
                              ui (web shell) · build-support (firmware linking)
```

The dependency graph is acyclic: **no generic crate depends on a project crate.**
Project-specific code (machine wiring, physics models, the emulator binary)
lives in the consumer's repo — see MaD's
[`SIL/`](https://github.com/RileyMcCarthy/MaD/tree/main/SIL) for a complete
reference consumer.

## Repository layout

| Crate | Path | What it is |
|-------|------|------------|
| `embsim-core` | [`core/`](core) | Virtual clock, serial PTY, event observers |
| `embsim-peripherals` | [`peripherals/`](peripherals) | GPIO, serial, encoder, pulse trains, timer, locks, threads, I2C, filesystem |
| `embsim-models` | [`models/`](models) | Generic device/IC models (ADS122U04 ADC, limit switch, edge detector) |
| `embsim-qemu` | [`qemu/`](qemu) | A QEMU VM as a board component: guest clock metered by the virtual clock, serial port on a net |
| `embsim-p2-qemu` | [`p2-qemu/`](p2-qemu) | The QEMU Propeller 2 target as a board component: boots the real ROM off a flash on the board's nets, every edge at its own instant. Carries the `target/p2` sources |
| `embsim-boards` | [`boards/`](boards) | Real boards from vendor netlists (the P2-EC32MB), with slots for the parts under test; the P2 package (`p2::P2Package`) that any P2 core — QEMU, an ISS, the native firmware — goes inside |
| `embsim-cffi` | [`cffi/`](cffi) | C ABI over the device models, for hosts that are not Rust |
| `embsim-runtime` | [`runtime/`](runtime) | `Emulator` builder, `Platform`/`Machine` traits, init ordering |
| `embsim-p2` | [`platforms/p2/`](platforms/p2) | Reference platform: Parallax Propeller 2 HAL trampolines + constants |
| `embsim-build` | [`build-support/`](build-support) | Two-line `build.rs` helper to find & link `lib<firmware>.a` |
| `embsim-memory-inspect` | [`tools/memory-inspect/`](tools/memory-inspect) | DWARF reader — recover C enums/structs/variables from the firmware archive |
| `embsim-trace` | [`tools/trace/`](tools/trace) | Time-series trace recorder + live web viewer (feature `web`) |
| `embsim-ui` | [`tools/ui/`](tools/ui) | Pluggable web shell the trace viewer (and your custom views) mount into |
| `embsim-minimal-example` | [`examples/minimal/`](examples/minimal) | Complete runnable firmware-free template |
| `embsim-cpu-oracle` | [`cpu-oracle/`](cpu-oracle) | ISS-vs-silicon golden records (parse, diff). CPU adapters supply the image and ISS. |

The plan for making every netlist part a node in one pipeline — switches, capacitors, diodes, rails, the P2 package — is [`NODES.md`](NODES.md).

## What a new project provides

Just two things:

### 1. A platform crate — `#[no_mangle]` HAL trampolines + a `Platform`

Your firmware calls C functions like `HAL_GPIO_setActive`. A platform crate
provides a Rust `#[no_mangle] extern "C"` function for each, delegating to the
generic peripheral, and implements the `Platform` trait to supply MCU constants:

```rust
pub struct MyMcu;
impl embsim_runtime::Platform for MyMcu {
    fn clock_freq_hz(&self) -> u32 { 16_000_000 }
    fn max_cores(&self)    -> usize { 1 }
    fn max_locks(&self)    -> usize { 8 }
}
```

See [`CONTRACT.md`](CONTRACT.md) for the full list of symbols a platform must
export and the ABI rules, and `embsim-p2` for a complete reference.

### 2. A `Machine` — the project wiring

The machine declares peripheral channel counts and connects peripheral events to
physical models:

```rust
impl embsim_runtime::Machine for MyMachine {
    fn peripheral_counts(&self, fw: &FirmwareInfo) -> PeripheralCounts { /* ... */ }
    fn host_serial_channel(&self, fw: &FirmwareInfo) -> usize { /* ... */ }
    fn wire(&self, fw: &FirmwareInfo) { /* register callbacks, set initial states */ }
}
```

### Then the whole emulator is ~10 lines

```rust
let fw = FirmwareInfo::from_archive("path/to/libfirmware.a")?;
Emulator::builder(MyMcu)
    .firmware(fw)
    .machine(Box::new(MyMachine))
    .clock_speed(1.0)
    .host_pty("/tmp/tty.sim_client")
    .sd_path("./sd")
    .entry(|| unsafe { firmware_begin() })
    .build()?
    .run()?;
```

The **runtime owns the init ordering** (clock before peripherals; `serial::init`
before bridging the PTY; …) so consumers can't get it wrong. It also preflights
every symbol the machine declares in `required_symbols()` and reports *all*
missing ones at once (`EmulatorError::MissingSymbols`) — invaluable when porting
to firmware whose enums were renamed.

A complete, runnable, firmware-free template is in
[`examples/minimal/`](examples/minimal/src/main.rs) — `cargo run -p embsim-minimal-example`.

## Using embsim in your project

embsim is a Cargo workspace of path crates (not yet on crates.io). Consume it
as a **git submodule** and point path dependencies at the crates you need:

```bash
git submodule add https://github.com/RileyMcCarthy/embsim.git vendor/embsim
```

```toml
# your-emulator/Cargo.toml
[dependencies]
embsim-core        = { path = "../vendor/embsim/core" }
embsim-peripherals = { path = "../vendor/embsim/peripherals" }
embsim-runtime     = { path = "../vendor/embsim/runtime" }
embsim-p2          = { path = "../vendor/embsim/platforms/p2" }   # or your own platform crate
embsim-models      = { path = "../vendor/embsim/models" }

[build-dependencies]
embsim-build       = { path = "../vendor/embsim/build-support" }
```

Your workspace should `exclude` the submodule directory (embsim is its own
workspace root) — path dependencies across the boundary work fine:

```toml
[workspace]
exclude = ["vendor/embsim"]
```

Build your firmware as a static library with its HAL symbols left undefined and
debug info enabled (`-g`), then link it from `build.rs`:

```rust
// build.rs
fn main() {
    embsim_build::link_firmware_static("../firmware/build", "firmware");
}
```

The archive location can be overridden without editing `build.rs` via
`EMBSIM_FIRMWARE_LIB_DIR` / `EMBSIM_FIRMWARE_LIB_NAME` (see the `embsim-build`
crate docs).

## One firmware per OS process (by construction)

The firmware HAL is bound through process-global `#[no_mangle]` symbols against a
single `libfirmware.a`. **There is therefore exactly one firmware per OS
process.** To run several instances, run several processes (MaD's Playwright
suite does this with `workers: 1`). Do **not** try to instance-scope the HAL
layer — the Rust statics in `peripherals` are not the constraint; the single C
symbol set is. (The host-side *tools* — trace store, UI registry — are separate
and may be reset between runs.)

## Building & testing

Every crate is testable **without any firmware**:

```bash
cargo build --workspace            # build everything
cargo test  --workspace            # run every crate's suite
cargo run -p embsim-minimal-example  # the firmware-free template end-to-end
```

Per-crate, if you want to iterate on one area:

```bash
cargo test -p embsim-core           # virtual clock, observers, serial PTY
cargo test -p embsim-peripherals    # gpio/serial/encoder/pulse_out/timer/lock/system/i2c/fs
cargo test -p embsim-models         # ADS122U04, limit switch, edge detector
cargo test -p embsim-qemu           # QEMU computer node (fake guest; real QEMU when installed)
cargo test -p embsim-p2-qemu        # P2 core: stub without a QEMU tree; EMBSIM_QEMU_P2_BUILD=<build> boots the ROM, the pad-mode and PLL benches
cargo test -p embsim-boards         # the P2-EC32MB board against its netlist
cargo test -p embsim-runtime        # Emulator builder + full no-firmware run
cargo test -p embsim-memory-inspect # DWARF parser (compiles a tiny C fixture at test time)
cargo test -p embsim-trace          # trace recorder + firmware-variable discovery
cargo test -p embsim-ui             # web shell render + handlers
cargo test -p embsim-p2             # P2 HAL trampolines + constants
cargo test -p embsim-build          # firmware-link resolution
```

Release-mode smoke (timing paths):

```bash
cargo test -p embsim-peripherals -p embsim-board --release
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
