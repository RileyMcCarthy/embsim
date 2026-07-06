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
| `embsim-runtime` | [`runtime/`](runtime) | `Emulator` builder, `Platform`/`Machine` traits, init ordering |
| `embsim-p2` | [`platforms/p2/`](platforms/p2) | Reference platform: Parallax Propeller 2 HAL trampolines + constants |
| `embsim-build` | [`build-support/`](build-support) | Two-line `build.rs` helper to find & link `lib<firmware>.a` |
| `embsim-memory-inspect` | [`tools/memory-inspect/`](tools/memory-inspect) | DWARF reader — recover C enums/structs/variables from the firmware archive |
| `embsim-trace` | [`tools/trace/`](tools/trace) | Time-series trace recorder + live web viewer (feature `web`) |
| `embsim-ui` | [`tools/ui/`](tools/ui) | Pluggable web shell the trace viewer (and your custom views) mount into |
| `embsim-minimal-example` | [`examples/minimal/`](examples/minimal) | Complete runnable firmware-free template |

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
cargo test -p embsim-runtime        # Emulator builder + full no-firmware run
cargo test -p embsim-memory-inspect # DWARF parser (compiles a tiny C fixture at test time)
cargo test -p embsim-trace          # trace recorder + firmware-variable discovery
cargo test -p embsim-ui             # web shell render + handlers
cargo test -p embsim-p2             # P2 HAL trampolines + constants
cargo test -p embsim-build          # firmware-link resolution
```

A few conventions the tests follow (see [`peripherals/src/pulse_out.rs`](peripherals/src/pulse_out.rs)
for the canonical example): peripheral modules keep process-global state, so
tests that touch a shared global serialize behind a crate-local `TEST_LOCK`
(with poison recovery) and pin the virtual clock once; assertions check
monotonicity / bounds / clamping rather than exact wall-clock timing. The
`embsim-memory-inspect` DWARF test compiles a small C fixture with `clang`
(preferred — the parser targets clang-emitted DWARF) + `ar` and **skips
gracefully** when no C toolchain is present.

Platform support: Linux and macOS (the serial PTY and thread emulation use
Unix APIs; Windows is not supported).

## License

MIT — see [LICENSE](LICENSE).
