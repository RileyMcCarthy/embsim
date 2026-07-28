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

# Determinism baseline (Oracle 1). `--nocapture` prints the measured
# free-running divergence per scenario; see rule 8 below.
cargo test -p embsim-board --test determinism -- --nocapture
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

6. **Property tests (`proptest`)** only for continuous domains (e.g. MNA
   resistor ladders). Use fixed seeds when non-determinism would flake CI.

7. **Strengthen, don't weaken.** Rewrites and refactors must keep or tighten
   existing assertions.

8. **Determinism suites report what they cannot yet assert.** `determinism.rs`
   compares N normalized engine event logs. In free-running mode it asserts the
   event *order* (what `DETERMINISM.md` T0 lists as determined) and **prints**
   the timestamp divergence rather than failing on wall-clock jitter — the
   baseline is measured, not guessed. A suite that reports must still be unable
   to pass vacuously: `determinism.rs` fails on an empty log and checks its own
   comparator against reordered/truncated/mutated synthetic logs.

## What each layer should cover

| Layer | Happy path | Edge | Parameterized |
|-------|------------|------|----------------|
| Peripherals | in-range I/O | OOR no-op, reset, max+1 panic | channel counts, baud/frame, pulse N×F |
| Instance bind | free fn → bound bank | LIFO drop panic, inheritance | multi-bank isolation matrix |
| Models | protocol/state | clamp, invalid cmd | DR/gain tables, thresholds |
| Runtime | full no-firmware run | missing symbols, ceilings | TooManyChannels per peripheral |
| Board | drive/sense/stream | contention, facade mismatch | net truth table, drop policies |
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
