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

# Re-bless the golden traces after an INTENDED engine/model behavior change.
# Review the diff: it is the wire behavior of the system.
EMBSIM_BLESS=1 cargo test -p embsim-board --test determinism
```

Per-crate iteration:

```bash
cargo test -p embsim-core
cargo test -p embsim-peripherals
cargo test -p embsim-models
cargo test -p embsim-runtime
cargo test -p embsim-board
cargo test -p embsim-p2
cargo test -p embsim-memory-inspect
cargo test -p embsim-trace
cargo test -p embsim-ui
cargo test -p embsim-build
cargo test -p embsim-minimal-example
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

## What each layer should cover

| Layer | Happy path | Edge | Parameterized |
|-------|------------|------|----------------|
| Peripherals | in-range I/O | OOR no-op, reset, max+1 panic | channel counts, baud/frame, pulse N×F |
| Instance bind | free fn → bound bank | LIFO drop panic, inheritance | multi-bank isolation matrix |
| Models | protocol/state | clamp, invalid cmd | DR/gain tables, thresholds |
| Runtime | full no-firmware run | missing symbols, ceilings | TooManyChannels per peripheral |
| Board | drive/sense/stream | contention, facade mismatch | net truth table, drop policies |
| Pin bridges | exact counts, GPIO both ways, encoder counts | slip, floating input, unbridged channel | polarity matrix, direction mapping, level projection |
| Stepped clock | N-run + golden identity | wedged actor, held time-release | case matrix × {free-running, stepped} |
| P2 trampolines | null/neg guards | bind routing | channel index grids |
| Tools | parse/record/render | empty/unknown | DWARF flag matrices |

## Deferred features (no tests yet)

When these land, each needs a dedicated integration binary:

- `Harness::from_toml`
- `AmbiguousLevel` dead-band projection
- Live topology mutation after `System::start`
- Dual-MCU firmware entry inversion (one image per process still applies)

## MaD pin bumps

Consumer repos (e.g. MaD) should re-run this suite against the **pinned**
submodule commit on SIL-related PRs (`cd vendor/embsim && cargo test
--workspace --all-targets`), mirroring how ProtoEmb is gated. Upstream CI on
this repo remains the primary gate for commits that land here.
