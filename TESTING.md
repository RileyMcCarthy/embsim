# embsim testing conventions

Every crate in this workspace is firmware-free and unit-testable. This document
is the contract for **how** tests are written so coverage stays uniform across
peripherals, models, board engine, runtime, and tools.

## Running the suite

```bash
cargo test --workspace --all-targets
cargo test --workspace --doc
cargo test -p embsim-trace --no-default-features   # headless recorder path
cargo test -p embsim-peripherals -p embsim-board --release   # timing-sensitive smoke

# Determinism (Oracle 1). `--nocapture` prints, per case, the asserted stepped
# identity and the measured free-running divergence; see rule 8 below.
cargo test -p embsim-board --test determinism -- --nocapture
# Stepped-clock mechanics (the barrier, the time-release, the wedge report).
cargo test -p embsim-board --test stepped_clock --test ads122u04_stepped

# Peripheral pin bridges. Each is its own binary because it owns the
# process-default peripheral banks (see rule 5). `--nocapture` prints the
# measured engine-event budget and the stepped N-run identity.
cargo test -p embsim-board --test pulse_bridge --test carriage_seam -- --nocapture
cargo test -p embsim-board --test pulse_bridge_stepped -- --nocapture

# The isolation parts, promoted from stubs on the real EdgeBoard netlist:
# levels and a rate-carried step train crossing the barrier, the fail-safe
# output of an unpowered side, and the end-switch current loop. `--nocapture`
# prints the measured engine-event cost of a step train at two rates a
# hundredfold apart.
cargo test -p embsim-board --test isolation_bridge -- --nocapture

# Re-bless the golden traces after an INTENDED engine/model behavior change.
# Review the diff: it is the wire behavior of the system.
EMBSIM_BLESS=1 cargo test -p embsim-board --test determinism

# The phase-0 baselines of NODES.md (the numbers DESIGN.md rules 1, 4 and 8
# are held to). The census asserts, per reference board, the cluster count,
# the largest cluster's root count and the number of parts with nothing
# behind them, as a committed fixture (the last a never-rises gate), and —
# phase 4 — rule 4's bound, `m ≤ 8` on every board under its reference
# harness (the module from its fingers, the add-on under its rails, the
# Edge board under the bench rails with its module-sourced socket finger
# at the LDO's 3.3 V, and with the module in its socket); the Edge board
# under the bench rails alone is above it, a never-rises fixture that
# says why. `--nocapture` prints the table with the largest cluster's
# nets, the node classes and the stubs by name.
cargo test -p embsim-board --test cluster_census -- --nocapture
# Phase 4, the engine half (stepped, own binary): a declared terminal — a
# `PowerOut` pin's net, a bench supply, a stuck net — is a cluster of its
# own and a boundary of every cluster around it. A power-out pin's idle
# drive is what its rail holds before the part publishes; a rail that
# publishes live re-resolves every cluster that reads it, through a
# resistor and through a diode; a released rail takes a bench strap without
# a fight; two sources disagreeing on one terminal are one fight, reported
# once; the module's P59 pull-down projects beside a core rail held from
# the bench (two hundred edges, no solve).
cargo test -p embsim-board --test terminals
# The resolver's side of it: the incremental-vs-full oracle with random
# terminal changes and rail publishes mid-run.
cargo test -p embsim-board --lib incremental_oracle
# Phase 4, the parts half (stepped, own binary): the module's power tree
# from its two J203 fingers — every bank rail at 3.3 V and the core rail at
# 1.813 V from the instant the bucks' 2.5 ms soft-start elapses, two
# setpoints from one registry key — the P2's reset through the rails'
# rise and under a held core rail, the reset node floating with one pad of
# its pull-up lifted, the debug-serial pins at their pull-ups, the build
# lints (a domain measured against nothing, a mechanical pad on a driven
# net, a rail down naming its input, a supply pin with no capacitor to its
# reference), the current instrument on a real board, and build ==
# live-held under each board's reference harness. Every rail wait waits on
# every module rail at its setpoint: the LDOs publish one after another.
cargo test -p embsim-board --test power_tree
# The assembled machine's rails once its soft-starts elapse — its own
# binary, because the add-on's ADC starts a protocol thread that lives for
# the rest of the process (rule 5 below) and parks on the clock every
# 250 µs, which idle-jumps the clock of every later stepped case sharing
# the process.
cargo test -p embsim-board --test power_tree_machine
# The rail models and the detector without an engine: soft-start instants,
# enable hysteresis, the discharge, the isolated reference, the comparator
# and its delays.
cargo test -p embsim-models --lib rail::
cargo test -p embsim-models --lib supervisor::
# The one pipeline (phase 1): every part a node, mechanical nodes, switch
# poles as identity unions (the three-pad jumper's two poles among them),
# the value parser over the whole module, the error that names the value;
# and the build fixed point: build == live before the first wake on the
# EC32MB and the Edge board — the live system started with time held
# (`System::hold_time`), since the module's oscillator arms a wake at
# attach — and the bound.
cargo test -p embsim-board --test one_pipeline --test build_fixed_point
# Rule 2, source-strength projection (phase 1, engine half): a pull never
# contends, ten times weaker loses with a finding, comparable sources solve
# and project through the dead band, the pulled ohms are the winner's path,
# ∞ Ω is a release, a current injection reads I·R or strands with a finding,
# a lone source inside the dead band reads its voltage.
cargo test -p embsim-board --lib source_strength
# Phase 2, the models on the boards (each in stepped mode, its own binary):
# the TCXO's 20 MHz reaching the P2's XI as a rate across the coupling
# capacitor with the buffer's self-biased stage at its mid-rail fixed point,
# and the AC-coupling rule stopping a rate at a capacitor too small for it;
# a gate's output moving exactly t_pd after its input through the datasheet
# output resistance, a Schmitt input holding inside its band and a plain
# one reading no level there (its output released); a PSRAM
# Read ID answered over the module's own nets.
cargo test -p embsim-board --test oscillator_chain --test logic_gate_levels --test psram_spi
# Phase 2, the P2 package (stepped, own binary): the rate on XI is the
# crystal the package reports, the reset inputs as it projects them, every
# pad released with no core so a bench pin takes one without a fight. Phase
# 4 added the START gate — a core held with the reason readable under, over,
# or with reset low — and pads at their bank's supply: a pad in a 1.8 V bank
# sits at 1.8 V, a pad in a bank whose supply pin reaches nothing floats and
# the bank is named. The interface phase (`NODES.md` §12 item 5, the P2
# task) added the datasheet's 3 ms restart: a core started, and its first
# wake delivered, 3 ms after RESN rises inside VDD's 1.7–1.9 V window and
# reported restarting in between, a release shorter than the delay starting
# nothing; the brownout without a reset (VDD out of its window while the
# core runs and RESN is not asserted: reported, the core held, its wakes
# stopped — and nothing with RESN asserted first); the native firmware
# image's pads through the package's bank supplies, floating before START;
# and the fast pad's strength fitted to the datasheet's output table.
cargo test -p embsim-board --test p2_package
# Phase 3, the solver half (stepped, own binary): a diode from the element
# library conducts at (V − V_F)/R and blocks reversed, a switched channel
# follows its gate both ways, two elements that chase each other are
# reported non-convergent at exactly two solves per element with their
# nodes floating, two histories reaching one drive table publish identical
# states (every solve starts cold), a node only leakage reaches floats, and
# the current instrument on a sink reads the pull-up's current to a nanoamp.
cargo test -p embsim-board --test pwl_elements
# The resolver's side of it: the LED chain solving with two unknowns (the
# rail handed over as a constant), the far side of an off diode in its
# cluster and floating, and the incremental-vs-full oracle with random
# diodes and channels in its boards.
cargo test -p embsim-board --lib elements
cargo test -p embsim-board --lib incremental_oracle
# Phase 3, the parts half (stepped, own binary): on the real EdgeBoard the
# indicator LED D3 lit by its inverter at (V_CC − V_F) / (220 Ω + R_OH) and
# dark when the output is low; the polarity FET passing the input forward
# and blocking it reversed with no `pin_short`; the end-switch loop
# regulated at the current regulator's 10 mA with the opto sinking P19; an
# opto output sinking only above its input threshold; the servo-enable
# transistor saturating under a 1 kΩ drive input and sagging under 220 Ω at
# exactly a hundred times its base current.
cargo test -p embsim-board --test board_elements
# The solver's three-region curves on hand-computed circuits: the polarity
# FET's start-up in exactly four solves (body diode on, channel on, diode
# off), the regulator ohmic below its knee and a source above it, the
# transistor saturated under a light load and active under a heavy one.
cargo test -p embsim-board --lib cluster::tests
# The module's polarity FET passing the carrier's 5 V to the protected rail
# from its fingers alone, and the isolation seam on the elements (the base
# at its knee, the loop at 10 mA).
cargo test -p embsim-board --test ec32mb_module --test isolation_bridge
# ns/solve of the MNA at m = 2, 4, 8, 11, 47 (not a test; run in release).
cargo run -p embsim-board --release --example solve_bench
# The QEMU core inside the package (needs a QEMU P2 tree): the ROM boot
# prints its edges / yields / publishes / START instant / wall time and
# holds its escalated-solve count exactly (since phase 4 the module is
# powered from its J203 fingers, nothing stuck: the reset releases at the
# bucks' 2.5 ms soft-start and the core starts the datasheet's 3 ms later,
# at 5.5 ms, the TCXO's 20 MHz reaches XI, and the count is the power
# tree's three solves before the first edge — none per edge, and none for
# the core's pad-read declarations since the sense task);
# `pad_modes` runs a hand-assembled guest whose 15 kΩ pull-up reads the
# sink holding its net low through `testp`; `crystal_pll` stalls a guest
# that selected the PLL with nothing on XI, then clocks it at 160 MHz from
# the 20 MHz that arrives — the scope reads its pad writes 12–13 ns apart,
# and one yield per pad write. Both benches supply VDD, RESN and the bank
# the guest drives, as the START gate and the pads' bank rule need; their
# guests start 3 ms in, the restart delay after the build.
EMBSIM_QEMU_P2_BUILD=<qemu-p2 build dir> cargo test -p embsim-p2-qemu -- --nocapture
# The interface phase's sense task (`NODES.md` §12 item 5; stepped, own
# binary): what a pin is handed is a
# voltage against its declared reference, and the level is the receiver's
# own projection — a 1.2 V node read low, high, low through a Schmitt
# receiver's hysteresis, a holding receiver against one that reads no level
# inside the dead band, the same 1.5 V read high by a P2 pad in a 1.8 V bank
# and no level by an LVCMOS receiver, a reader against a reference pin at
# 1 V, a floating sense handed no voltage with its finding, a fought net
# handed its 1.65 V operating point with the contention beside it.
cargo test -p embsim-board --test receiver_projection --test pin_declarations
# The interface phase's rules task (stepped, own binary): a fought node
# under an ADC
# input reads its 1.65 V operating point with the contention beside it (two
# pads, and a rail against a short); a rail through one resistor into an ADC
# input is handed 3.3 V exactly with no solve, and one solve once a pad
# drives the node; an AM26LV32's open inputs sit at their own 0.83 V /
# 0.70 V bias and the fail-safe holds the output high until a pad pulls A
# low; a clamped pin sits one knee above its supply; an open drain no
# pull-up reaches is a build finding. The two analog goldens were re-blessed
# by this task (`NODES.md` §12 item 5) — finding lines only.
cargo test -p embsim-board --test resolution_rules
# The p2core differential behind "60 000 states identical" is a manual run
# against MaD's `SIL/p2core`; the exact commands (flash image, reference,
# traced boot, comparison) are in `p2-qemu/README.md`, "The state trace and
# the p2core differential". Run it when the node's instruction path moves.
# The pulse-out schedule (`PeriodicSchedule`, nanoseconds since the
# interface phase) and the stepper plant that folds it are unit-tested in
# their crates: an hour of a 10 MHz train counts exactly, a rate change
# between two microseconds folds each side exactly.
cargo test -p embsim-peripherals --lib pulse_out
cargo test -p embsim-models --lib stepper_motor
```

Per-crate iteration:

```bash
cargo test -p embsim-core
cargo test -p embsim-peripherals
cargo test -p embsim-models
cargo test -p embsim-qemu            # QEMU node; the real-QEMU case skips loudly without qemu-system-aarch64
cargo test -p embsim-runtime
cargo test -p embsim-board
cargo test -p embsim-p2
cargo test -p embsim-memory-inspect
cargo test -p embsim-trace
cargo test -p embsim-ui
cargo test -p embsim-build
cargo test -p embsim-minimal-example
cargo test -p embsim-cpu-oracle          # ISS-vs-silicon golden parse/diff
```

Coverage (requires `cargo-llvm-cov`):

```bash
cargo llvm-cov --workspace --summary-only
```

## Style rules

1. **Prefer `#[rstest]`** over bare `#[test]` so filters and case names are
   consistent (`cargo test feature_ -- --list` shows named cases).
2. **Multi-value inputs use cases**, not copy-pasted functions:

   ```rust
   #[rstest]
   #[case::zero(0)]
   #[case::one(1)]
   #[case::max(MAX_CHANNELS)]
   fn init_count_allowed(#[case] n: usize) { … }
   ```

3. **Peripheral free-function tests** always start with:

   ```rust
   let _g = crate::test_support::guard();
   crate::test_support::ensure_clock();
   ```

   Never call `virtual_clock::init` / `set_scale` from `embsim-peripherals`
   tests (the shared clock is pinned once — see `peripherals/src/lib.rs`).

4. **Assert contracts, not wall flakiness.** Prefer virtual-time schedules,
   monotonicity, clamps, and ε windows. Dedicated paced-stream tests that pin
   scale and assert wall delay are the exception (document why).

5. **Board / process-global clock isolation.** Integration cases that must
   *not* see a pre-initialized clock live in their own `board/tests/*.rs`
   binary (see `clock_guard.rs`). The same applies in reverse to cases that
   *re-anchor* the clock between runs: `determinism.rs` calls
   `virtual_clock::init` before every run of its N-run matrix, so it must not
   share a process with cases that assume a monotonically accumulating clock.

   **The clock is process-global.** A binary that re-`init`s (paced vs unpaced,
   or a fresh `now = 0`) must serialize every case behind one suite mutex
   (`determinism.rs` and `stepped_clock.rs` both do). Re-`init` with a live
   actor is allowed — the actor stays registered — so a binary whose cases
   spawn long-lived actor threads (the ADS122U04 model does) still belongs in
   its own test binary so leftover actors cannot hold a later case's barrier.

   **The process-default peripheral banks are the same kind of global.** A case
   that plays firmware through the `embsim-peripherals` free functions
   (`pulse_out::start`, `gpio::set_active`, …) shares one bank with every other
   case in its process, so those cases live in their own binary, take one suite
   lock, and `reset()` the banks they used on the way out —
   `pulse_bridge.rs`, `pulse_bridge_stepped.rs` and `carriage_seam.rs` are the
   pattern. Keeping them out of `determinism.rs` is deliberate: its cases are
   pure board components, and a global bank underneath them would make an
   unrelated failure look like a determinism regression.

6. **Property tests (`proptest`)** only for continuous domains (e.g. analog
   resistor ladders). Use fixed seeds when non-determinism would flake CI.

7. **Strengthen, don't weaken.** Rewrites and refactors must keep or tighten
   existing assertions.

8. **Determinism suites assert what their mode can promise, and report the
   rest.** `determinism.rs` compares N normalized engine event logs in *both*
   clock modes. **Stepped**: the full timestamped projection must be identical
   across runs, across processes, and against a golden trace — a drift is a
   regression. **Free-running**: the event *order* is asserted and the timestamp
   divergence is **printed**, never failed on, because wall-clock jitter is the
   thing that mode has. Never "fix" a free-running flake by asserting timestamps
   there; move the case to stepped mode.

   A suite that reports must still be unable to pass vacuously.
   `determinism.rs` fails on an empty log, checks its own comparator against
   reordered/truncated/mutated synthetic logs, asserts that the full and
   shape projections really do differ on a 1 µs timestamp change, asserts that
   free-running *does* diverge where stepped does not, and requires every named
   case to have a golden.

9. **A model's proving tests run in stepped mode.** A free-running system
   with no firmware in it can still settle two ways from one input:
   `board/tests/rs422_determinism.rs` holds an open divergence of exactly
   that kind (the receiver output rests `Floating` in about one run in twelve,
   a drive release from a component's attach interleaving differently with a
   sense delivery — reproduced 2026-09-23, see the test's docs for the
   recipe). Until it is closed, a new model's proving test — the tests
   `NODES.md` §8 lists as each phase's proof — starts its system in stepped
   mode (`virtual_clock::init_mode(ClockMode::Stepped, …)`, the pattern in
   `determinism.rs` and `pulse_bridge_stepped.rs`), where the engine quiesces
   every actor before it advances and the interleaving is the engine's own.
   A free-running case may still exist beside it to measure divergence, under
   rule 8; it is not the proof. Every sense→drive cascade a new model adds is
   one more place the open divergence can show, which is why the rule lands
   before the models do.

## What each layer should cover

| Layer | Happy path | Edge | Parameterized |
|-------|------------|------|----------------|
| Peripherals | in-range I/O | OOR no-op, reset, max+1 panic | channel counts, baud/frame, pulse N×F |
| Instance bind | free fn → bound bank | LIFO drop panic, inheritance | multi-bank isolation matrix |
| Models | protocol/state | clamp, invalid cmd | DR/gain tables, thresholds |
| Runtime | full no-firmware run | missing symbols, ceilings | TooManyChannels per peripheral |
| Board | drive/sense/stream | contention, facade mismatch | net truth table, drop policies |
| Pin bridges | exact counts, GPIO both ways, encoder counts | slip, floating input, unbridged channel | polarity matrix, direction mapping, level projection |
| Interface parts | the datasheet function table, on a real netlist | unpowered side, disabled output, open loop | variant/pinout matrix, fail-safe vs default-high |
| Stepped clock | N-run + golden identity | wedged actor, held time-release | case matrix × {free-running, stepped} |
| P2 trampolines | null/neg guards | bind routing | channel index grids |
| Tools | parse/record/render | empty/unknown | DWARF flag matrices |

## Deferred features (no tests yet)

When these land, each needs a dedicated integration binary:

- `Harness::from_toml`
- Live topology mutation after `System::start`
- Dual-MCU firmware entry inversion (one image per process still applies)

## MaD pin bumps

Consumer repos (e.g. MaD) should re-run this suite against the **pinned**
submodule commit on SIL-related PRs (`cd vendor/embsim && cargo test
--workspace --all-targets`), mirroring how ProtoEmb is gated. Upstream CI on
this repo remains the primary gate for commits that land here.
