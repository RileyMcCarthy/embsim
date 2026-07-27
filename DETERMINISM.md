# Deterministic execution mode

**Status:** design only, nothing implemented (2026-07-26). Companion to
[`BOARD_ENGINE.md`](BOARD_ENGINE.md) ("Execution model") and
[`CONTRACT.md`](CONTRACT.md).

The requirement this document answers is a consumer's, stated plainly: *"once
it's all connected the SIL model runs deterministically."* Today it does not,
and the gap is larger than a clock rewrite. This document says exactly what
"deterministically" can mean for embsim, which parts of it are cheap, which
part is expensive, which part is out of scope, and how CI would prove any of
it.

Two words are used precisely throughout:

- **Consistent** — every observer agrees on one order of events. The engine is
  already consistent by construction (single-writer, enqueue-seq serialized).
- **Reproducible** — running the same scenario twice produces the *same* order.
  The engine is **not** reproducible today, and consistency does not imply it.

## Contents

1. [What determinism can mean here — the tier ladder](#the-tier-ladder)
2. [T0: what is already determined, and what is not](#t0-the-honest-baseline)
3. [T1: the concrete design](#t1-the-concrete-design)
4. [T2: the sketch, and what it breaks](#t2-whole-system-schedule-determinism-sketch)
5. [Proving it in CI](#proving-it-determinism-testing)
6. [Recommendation and phasing](#recommendation-and-phasing)
7. [Non-goals](#non-goals)

## The tier ladder

| Tier | Guarantee | Holds for | Cost |
|---|---|---|---|
| **T0** (today) | Consistent event order; wall-clock-coupled timestamps | everything | shipped |
| **T1** | Engine is a pure discrete-event machine in virtual time; identical event trace for a fixed stimulus sequence | fully: systems whose only actors are engine-hosted components (board + models + faults + streams, scripted stimulus). partially: systems with firmware — removes wall-clock/host-load coupling, leaves the firmware's HAL-call order free | moderate; ~1 new core module + engine loop branch + transport seam |
| **T2** | Whole-system schedule determinism: firmware threads are cooperatively scheduled actors, the HAL is the only yield surface, one runnable at a time under a deterministic policy | everything embsim runs natively | high; touches every trampoline + a virtual-cost policy decision |
| **T3** | Instruction-level lockstep | n/a | out of scope — needs a P2 ISA core |

**T3 is explicitly out of scope.** embsim's entire value proposition is running
the *real compiled firmware* natively — the firmware `.a` from
`pio run -e native_emulator` is host machine code, and the platform crate
resolves its HAL symbols at link time (`CONTRACT.md`, "Why it exists"). Lockstep
instruction determinism would require replacing that with a Propeller 2
instruction-set simulator: a different project, an order of magnitude more
work, and it would abandon the "test the code you ship" property. It is also
already ruled out by `BOARD_ENGINE.md`'s stated fidelity boundary ("no
cycle-accurate silicon emulation"). What T3 would buy over T2 — cycle-exact
timing races *inside* one cog, hub-arbitration effects, `waitx` granularity —
is real, but it is not the class of bug this machine's SIL suite is hunting.

## T0: the honest baseline

### Already determined (verified by reading the code, not assumed)

- **Drive application order.** `PinHandle::set_drive`
  (`board/src/component.rs`) reserves a global sequence
  (`EngineLink::next_drive_seq`) and posts `Command::Drive { seq, .. }`;
  `EngineCore::apply_ready_drives` (`board/src/engine.rs`) applies strictly in
  seq order out of a `BTreeMap`, resolving after each. Channel arrival order
  cannot reorder drives.
- **Net resolution is pure.** `Resolver::resolve` is a function of (identity
  merges, conduction edges, the drive table, power/stuck sources, sense
  registrations). It contains no clock read and no I/O.
- **Hash-order hygiene is mostly already right.** `resolve` sorts everywhere
  iteration order could reach an outcome (`driver_roots.sort_unstable()`,
  `fighting.sort_unstable()`, `extra_clusters.sort_unstable()`), and the
  engine's `HashMap` fields (`sense_subs`, `wake_subs`, `routes`,
  `stream_subs`, `drop_state`) are only ever accessed by key — sense delivery
  walks `self.nets` by index, and per-net callbacks are a `Vec` in registration
  order. `route_streams` walks `self.streams` in registration order and sorts
  `path_roots`.
- **Timer tie-breaks.** `TimerEntry::cmp` orders by `(deadline_us, seq)`, so
  simultaneous and late deadlines fire in schedule order.
- **Per-producer stream FIFO.** `Command::StreamWrite` carries no seq because
  the channel's own order *is* the wire contract per producer.
- **The MNA solve** (`board/src/cluster.rs`, `QuasiStaticMna::solve`) is
  deterministic given identical inputs *in identical order* — dense Gaussian
  elimination with partial pivoting over `Vec`s, no hashing on the numeric
  path.

### One real hash-order defect, worth fixing regardless of tier

`Resolver::resolve` builds the per-cluster source list by iterating a `HashMap`:

```rust
// board/src/engine.rs — cluster_sources assembly
for slots in net_drivers.values() {          // <-- HashMap iteration
    for &si in slots {
        cluster_sources.entry(cluster_of[net]).or_default().push(ClusterSource { .. });
```

That `Vec` order reaches `QuasiStaticMna::solve`, where it becomes the
accumulation order of `matrix[c][c] += g` and `rhs[c] += i`. Rust's default
hasher is **randomly seeded per process**, so a cluster with two or more driver
sources can produce last-bit-different node voltages from one process to the
next — published as `NetState::Analog(volts)`, delivered to senses, and
compared against thresholds. Fix: iterate `self.slots` by dense index (the
drive table is already a `Vec`, and `net_drivers`' member lists are built in
that order) instead of `net_drivers.values()`. Three lines, no semantic change,
and it removes the only place where a process-level coin flip can reach a
numeric output.

### Not determined

- **Every virtual timestamp.** `virtual_clock::virtual_us`
  (`core/src/virtual_clock.rs`) is `PROCESS_ORIGIN.elapsed() * scale`: sampled
  wall time. So `fire_due_timers` stamps each wake with whatever instant the
  engine happened to wake at; `trace::resample_all` stamps every `Sample`
  likewise; and models that compare virtual time against an interval —
  `models/src/ads122u04.rs::protocol_loop` testing
  `now_us >= last_conversion_us + interval_us` — decide differently run to run.
- **The seq numbers themselves.** `next_drive_seq` is a racing `fetch_add`
  across component threads. Enqueue-seq makes the applied order *consistent*;
  it does not make it *the same order twice*. This is the single most
  misread property in `BOARD_ENGINE.md`, and the reason T1 needs actor
  scheduling and not just a stepped clock.
- **Drive-vs-timer interleaving.** Whether a given drive lands before or after a
  wake at the same nominal virtual time is a race between the enqueuing thread
  and the engine's `recv_timeout` return (`EngineCore::run`).
- **OS thread scheduling** among: firmware cog threads
  (`peripherals/src/system.rs::start_thread` spawns real
  `std::thread`s), the MCU serial pumps (`board/src/mcu.rs::pump_loop`), the
  ADS122U04 component pump (`models/src/ads122u04_component.rs::pump_loop`),
  the model protocol thread (`models/src/ads122u04.rs::protocol_loop`), the
  trace poller (`tools/trace/src/recorder.rs::poll_loop`), and the engine.
- **File-descriptor readiness.** Every firmware↔model byte crosses a real
  `socketpair` (`board/src/mcu.rs::create_pipe_pair`,
  `models/src/ads122u04.rs::create_pipe_pair`) with kernel buffering and
  `poll(2)` timeouts (`PUMP_POLL_TIMEOUT_MS = 10`).
- **Wall-clock deadlines inside the simulation.**
  `DRIVE_SEQ_STALL_TIMEOUT` (`Instant::now`) in `check_drive_stall`;
  `Serial::receive_data_timeout`'s `Instant`-based deadline plus its 100 µs
  `EAGAIN` sleep; `poll_loop`'s 500 ms warm-up sleep.
- **Real sleeps standing in for virtual waits** — 9 sites:
  `peripherals/src/timer.rs` (`wait_ms`, `wait_us`),
  `peripherals/src/serial.rs` (`pace_bytes`, two timeout paths, the EAGAIN
  poll), `peripherals/src/pulse_out.rs` (`sleep_virtual_us`),
  `platforms/p2/src/ffi.rs` (`HAL_serial_recieveDataTimeout` guard path),
  `models/src/ads122u04.rs` and `models/src/ads122u04_component.rs` (poll
  cadence), `tools/trace/src/recorder.rs`.

The practical shape of T0: a scenario *usually* produces the same outcome and
flakes when a threshold sits near a timing boundary. `TESTING.md` rule 4
("assert contracts, not wall flakiness") is the workaround, and its existence is
the evidence that T0 is not enough.

## T1: the concrete design

**Goal:** the engine becomes a discrete-event simulator in virtual time. Time
advances to the next scheduled event when all actors are quiescent, instead of
tracking wall time. For a system whose actors are all engine-hosted, the event
trace is then bit-identical across runs and across machines (modulo the float
caveat in [Proving it](#proving-it-determinism-testing)).

### 1. Stepped clock mode in `embsim-core`

Add an explicit "now" and a mode discriminant to `core/src/virtual_clock.rs`:

```rust
/// How virtual time advances.
pub enum ClockMode {
    /// Today's behavior: virtual time is scaled wall time.
    FreeRunning { speed: f64 },
    /// Virtual time is a value the scheduler sets; it advances only when
    /// every registered actor is parked.
    Stepped,
}

pub fn init_mode(mode: ClockMode, freq: u32);
pub fn mode() -> ClockMode;

/// Park the caller until virtual time reaches `v_us`. In `FreeRunning` this is
/// today's `sleep(virtual_to_wall_us(..))`. In `Stepped` it registers a pending
/// deadline, marks the caller parked, and blocks until the scheduler advances.
pub fn wait_until(v_us: u64);

/// Register a thread that can create simulation work. Stepped mode only
/// advances time when every registered actor is parked.
pub fn register_actor(name: &str) -> Actor;

/// Scheduler-only: advance virtual now. Monotonic; rejects going backwards.
pub fn advance_to(v_us: u64) -> Result<(), TimeWentBackwards>;
```

`virtual_us()` becomes a branch: `FreeRunning` → today's arithmetic; `Stepped`
→ `NOW_US.load(Relaxed)`. That is one relaxed load plus one branch added to the
hot path — at the ~100 Hz sample rates these consumers run, noise; worth a
micro-benchmark before landing, not worth a design compromise.

**Mode is chosen at `init_mode` and immutable thereafter**, like today's
`PROCESS_ORIGIN` `OnceLock`. `set_scale` in stepped mode is a loud no-op
(warn + no state change), since scaling a stepped clock is meaningless.

### 2. Runtime mode enum, not a Cargo feature — and why

Recommendation: **a runtime `ClockMode`, one build**. A feature was considered
and rejected:

- The clock is already a process-global configured by a single `init(speed,
  freq)` call. A mode argument fits the existing shape; a feature would make
  `virtual_us` behave differently per *build*, which is far harder to reason
  about at a call site.
- Cargo features are additive and unifying. `--all-features` CI, and any
  consumer that pulls two embsim-dependent crates with different feature sets,
  would have to pick a winner — silently.
- Consumers need both modes in one binary: `mad-emulator --deterministic` for
  scenario regressions, free-running for `make playground` with the live trace
  UI and a human at the PTY.
- The test matrix already pins the clock once per crate
  (`peripherals/src/lib.rs::test_support::ensure_clock`), so mode-per-build
  would fragment it. Stepped-mode tests instead get their own test binaries,
  exactly as `board/tests/clock_guard.rs` already does for the
  uninitialized-clock path (`TESTING.md` rule 5).

Free-running stays the **default**, so nothing about the interactive playground,
the trace viewer, or the PTY-driven Playwright flow changes byte-for-byte.

### 3. The engine is the time authority

The net-engine thread already owns the only ordered event queue and the only
timer wheel, so it is the natural scheduler. `EngineCore::run` gains a stepped
branch:

1. **Drain the command queue completely.** `COMMAND_DRAIN_BATCH_MAX = 64` exists
   only to stop a command flood from starving *wall-clock* timers
   (`sustained_drive_flood_does_not_starve_the_timer_wheel`); with time advanced
   by the engine itself, starvation is impossible and the cap must go — a
   partial drain would make the applied prefix depend on arrival timing.
2. **Fire every wheel entry due at `now`**, in `(deadline, seq)` order — already
   correct.
3. **Wait for quiescence** (§4). Callbacks fired in step 2 may have enqueued
   drives, so loop 1–3 to a fixpoint.
4. **Advance.** `next = min(wheel head deadline, earliest parked-actor
   deadline)`; `advance_to(next)`; goto 1.
5. **Empty wheel + no parked deadline + no runnable actor** = the system is
   finished or wedged. Report it (`Finding::NoRunnableActor` with the parked
   set) — never spin, never park forever.

`next_wall_wait_us` becomes free-running-only; stepped mode uses a
`next_virtual_deadline` sibling. `check_drive_stall` is **disabled in stepped
mode**: its `Instant`-based `DRIVE_SEQ_STALL_TIMEOUT` guards against an enqueuer
dying between the seq reservation and the send, which under actor accounting is
detected exactly (the actor never re-parks) rather than by timeout.

### 4. What "quiescent" means — and the honest problem

**Definition.** An actor is *runnable* unless it is parked at a virtual deadline
via `wait_until`, or blocked reading an in-process queue that is empty. Time
advances only when `running == 0`. This is a conservative barrier in the easy
case: one central scheduler, all actors parking with an explicit deadline.

Which of today's actors fit:

- **Engine thread** — it is the scheduler. Fits by construction.
- **Pure components** — `models/src/limit_switch.rs`, and the gantry/sampling
  models when they move onto the wheel. These are *not separate actors at all*:
  their wake and sense callbacks run **on the engine thread**. This is the
  payoff of `BOARD_ENGINE.md`'s "no broadcast `tick()`, engine-owned wakeups"
  decision — every component that gets its time from `io.schedule_at` /
  `io.schedule_every` is deterministic for free.
- **Model protocol threads** — `ads122u04.rs::protocol_loop` polls an fd and
  compares `virtual_us()` against a conversion interval. Two options: (a) move
  it onto the engine wheel (its conversion cadence already *is* a virtual-time
  period), or (b) make it a registered actor that parks via `wait_until`.
  **Recommend (a):** it deletes a thread and a poll loop instead of making
  them deterministic.

Where quiescence does **not** hold, and the bounded-nondeterminism boundaries
that answer it:

- **Real file descriptors are not deterministic, full stop.** The
  firmware↔model byte path is a `socketpair` with a `poll(2)`-driven pump
  thread on one side. Even with a stepped clock, *whether the pump has read the
  byte yet* decides which virtual instant it enters the engine, and that is an
  OS scheduling decision. A thread spinning on `EAGAIN` is neither running-with-
  work nor parked-at-a-deadline; the barrier cannot classify it.

  **Boundary: in stepped mode, byte transports become in-process deterministic
  queues.** The engine side already is one (`StreamTx::write` and `on_byte` are
  engine commands). It is the *HAL* side that must change: add a
  `serial::Transport` seam in `embsim-peripherals` with two implementations —
  `FdTransport` (today's, free-running) and `QueueTransport` (an in-process
  ring, whose blocking read is a `wait_until`-style actor park). `McuComponent`
  selects by clock mode at attach; `Serial::init_channel_fd` gains a
  transport-installing sibling. **This is the single largest piece of T1.**

- **The host PTY is excluded.** `core/src/serial_pty.rs` + `Emulator::run` step
  3 bridge a real terminal device driven by a human or by Playwright over
  `node-serialport`. There is no deterministic model of when the host writes.
  Rule: enabling stepped mode with a PTY bridged is an error unless the
  consumer opts into an explicit `stepped_with_host_io()`, which downgrades the
  guarantee to "deterministic between host inputs" and stamps the event trace
  with a `HostIoNondeterminism` marker so a golden-trace comparison fails
  *loudly* rather than mysteriously. Honest consequence for the reference
  consumer: the existing Playwright suite drives the UI over that PTY, so
  **UI-driven E2E stays T0/T1-with-host-io**. The fully deterministic scenarios
  are the ones scripted inside the emulator process — which is what the
  bench-bug suite wants anyway.

- **Wall-clock deadlines inside the simulation** must become virtual:
  `Serial::receive_data_timeout`'s `Instant` deadline and its EAGAIN sleep,
  and the trace poller's 500 ms warm-up. A slow host must not be a behavioral
  difference.

- **The trace poller must move onto the wheel.** `recorder.rs::poll_loop` runs
  at a virtual cadence but samples at wall-determined instants, so every
  recorded `Sample.time_us` differs run to run. Since the trace is one of the
  determinism oracles (§5), this is load-bearing, not cosmetic.

### 5. Migration lever

The cheapest path to T1 is a **behavior-preserving refactor first**: introduce
`virtual_clock::wait_until` whose free-running implementation is exactly
today's `sleep(virtual_to_wall_us(d))`, then mechanically convert all 9 sleep
sites. Nothing changes observably, every wait becomes mode-agnostic, and
stepped mode afterwards is a new implementation behind calls that already exist
— not a rewrite spread across five crates.

## T2: whole-system schedule determinism (sketch)

### What T1 does not give you

Firmware cog threads (`peripherals/src/system.rs::start_thread`) are real,
preemptively scheduled OS threads. Between two HAL calls a cog executes an
unbounded, host-scheduled amount of native code. So at T1 the *engine* sees an
identical trace only if the cogs happen to call the HAL in the same order —
which is exactly what is not guaranteed. The reference consumer's firmware runs
several cogs (its `dev_cogManager` allocates them), so:

- **T1 buys, for a whole-machine scenario:** immunity to host load and wall
  clock, virtual timestamps that are integers the scheduler chose, no timing
  tolerance windows in tests, and high-probability *outcome* repeatability
  (the cogs are loosely coupled through virtual-time-paced HAL calls).
- **T1 does not buy:** trace identity, or reproducibility of a cog-to-cog race.

### What T2 requires — CONTRACT.md additions

1. **The HAL is the only scheduling boundary.** Every trampoline in
   `platforms/p2/src/ffi.rs` becomes a yield point: on entry, release the run
   token; the scheduler picks the next runnable actor by a deterministic rule
   (e.g. lowest cog id among those at the earliest virtual deadline); the
   resumed thread reacquires the token before touching peripheral state.
2. **A one-runnable-at-a-time token**, owned by a cooperative scheduler in
   `embsim-peripherals` (`instance`/`system`) or `embsim-core`.
   `HAL_system_startThread` *registers* a thread on a run queue instead of
   racing it into existence.
3. **A virtual-cost policy for code between HAL calls.** With no instruction
   counting, the scheduler must charge *something*. This is the hardest honest
   problem in T2 and needs a recorded decision **before** any code:
   - charge zero (all firmware compute is instantaneous) — simple and
     deterministic, but a busy-wait loop that today makes progress in virtual
     time becomes an infinite loop;
   - charge a fixed cost per HAL call;
   - additionally make repeated `HAL_time_getUs`/`getCycles` reads advance by an
     epsilon so spin-until-timeout loops terminate.

   **Recommendation:** fixed per-HAL-call cost *plus* the `HAL_time_*` epsilon.
   It is the only combination under which existing firmware idioms terminate.
4. **CONTRACT.md must promise:** no trampoline blocks on a host primitive
   except through the scheduler; every blocking HAL receive
   (`HAL_serial_recieveDataTimeout`) is a park with a virtual deadline;
   `HAL_time_waitMs`/`waitUs` are parks (they nearly are already); no
   firmware-visible use of `Instant`.

### What breaks

- Blocking HAL receive loops that rely on real fd readiness
  (`Serial::receive_data_timeout`'s `Instant` deadline + EAGAIN sleep) must be
  rewritten against the `QueueTransport`.
- `Emulator::run`'s "call the entry on the caller's thread" flow: the main
  thread becomes just another registered actor, or entry-mode
  (`McuBuilder::entry`, already delivered) becomes mandatory in stepped mode.
- **Firmware that busy-waits on a memory flag written by another cog, with no
  HAL call in the loop, deadlocks** under a cooperative scheduler. This is the
  one way T2 can turn a working SIL run into a hang. Mitigations: the
  `NoRunnableActor` finding naming the parked set, and a documented escape
  hatch (a HAL no-op yield the firmware can call). Note this pattern is
  *already* fragile — an optimizing `native_emulator` build may hoist the flag
  read out of the loop entirely; T0's preemption merely hides it.
- HAL locks (`peripherals/src/lock.rs`) must park through the scheduler. That
  is partly a *benefit*: with one runnable at a time and a wait-for graph,
  deadlock detection (including the cross-cog ABBA case the consumer's locking
  rules exist to prevent) becomes mechanical rather than aspirational.

### Blast radius per crate (rough LoC touched, tests included)

| Crate | T1 | T2 |
|---|---|---|
| `embsim-core` | new: `ClockMode`, `NOW_US`, `wait_until`, actor registry, `advance_to` — ~250–400 | scheduler run queue + run token — ~300–500 |
| `embsim-board` | stepped branch in `EngineCore::run`, unbounded drain, disable stall watchdog, `next_virtual_deadline`, event log, `cluster_sources` order fix — ~300 in `engine.rs` | resume points visible to the engine — small |
| `embsim-peripherals` | `serial::Transport` seam + `QueueTransport`; all `sleep_virtual_us` → `wait_until` (`timer.rs`, `serial.rs`, `pulse_out.rs`) — ~400, **the bulk of T1** | `system.rs::start_thread` cooperative registration; `lock.rs` parks through the scheduler — ~400 |
| `embsim-models` | `protocol_loop` and the component pump replaced by engine wakeups — ~200, mostly *deletion* | none beyond T1 |
| `platforms/p2` | 2 sleep sites in `ffi.rs` — ~20 | **every** trampoline gains yield accounting — the largest T2 diff |
| `embsim-runtime` | mode plumbed through `EmulatorBuilder`; PTY guard — ~80 | entry-on-caller-thread flow reworked — ~100 |
| `tools/trace` | poller becomes a wheel entry — ~80 | none |
| consumer (MaD) | pick the mode in `MaDSim`; the system description is unchanged; Playwright stays free-running | firmware may need explicit yield points |

## Proving it: determinism testing

Determinism that is not tested is a claim, not a property. Two oracles, one
new and one existing.

### Oracle 1 (new, and the real one): the engine event log

An append-only, opt-in sink on the engine — `Diagnostics`-shaped, emitted from
the engine thread so it is totally ordered by construction:

```rust
pub struct EngineEventRecord { pub seq: u64, pub v_us: u64, pub event: EngineEvent }

pub enum EngineEvent {
    DriveApplied { seq: u64, endpoint: EndpointId, drive: Option<TheveninDrive> },
    NetResolved  { net: NetId, state: NetState },
    SenseDelivered { net: NetId, state: NetState },
    Wake { component: ComponentId },
    StreamByte { producer: EndpointId, consumer: EndpointId, byte: u8 },
    Reroute { epoch: u64 },
    Finding(Finding),
}
```

Off by default, enabled per system (`System::event_log()`), zero cost when off.
This log is precisely the set of things T1 claims to determine, which is what
makes it the right oracle rather than the trace store.

### Oracle 2 (existing): the `embsim-trace` recorder

`tools/trace/src/recorder.rs` already records timestamped samples and already
has a headless path in CI (`cargo test -p embsim-trace --no-default-features`).
It is the human-facing artifact and a decent coarse regression signal, but it
needs normalization before it can be compared.

### Trace normalization spec (this *is* the contract)

- **Drop wall-clock fields entirely.** The event log has none by design; the
  trace store's `Sample.time_us` is virtual and kept.
- **Quantize floats:** voltages to 1 µV, resistances to 1 mΩ — matching
  `BOARD_ENGINE.md`'s MNA hand-check tolerance ("asserted to µV"). Same binary
  + same inputs + same order gives bit-identical IEEE-754 results, so this is
  not needed *within* a host; it is needed the moment goldens are shared across
  architectures (x86-64 vs aarch64 contraction/rounding differences are real).
- **Canonicalize identity** to the dense ids that already exist
  (`ComponentId`, `EndpointId`, net index) — never
  `std::thread::current().name()`, never a pointer, never a `HashMap` order.
- **Elide host paths** from `Finding` payload strings (SD paths, PTY symlinks).
- **Keep virtual timestamps exactly.** In stepped mode they are integers the
  scheduler chose; any drift is a regression, not noise.

### Tests

- **N-run identity** — `board/tests/determinism.rs` (its own binary, per
  `TESTING.md` rule 5, because it initializes the clock in stepped mode). Run
  the same `System` + `Scenario` N = 5 times in-process, `assert_eq!` the event
  logs. `#[rstest]` cases over a scenario matrix: nominal, `pin_detach` on
  AVDD, `stream_drop(EveryNth(3))`, crossed-TX/RX harness, jumper open/closed.
- **Multi-process identity** — in-process repetition shares one `HashMap` seed,
  so it *cannot* catch hash-order nondeterminism (exactly the
  `cluster_sources` defect above). CI must run the binary N times as separate
  processes. Keep this even after the `BTreeMap`/sorted-iteration hygiene fixes:
  it is the backstop that proves the hygiene.
- **Golden traces** — `board/tests/fixtures/traces/*.jsonl`, normalized as
  above, for a handful of canonical scenarios; compare, and rewrite under
  `EMBSIM_BLESS=1`. This turns "did this firmware/model change alter the wire
  behavior?" into a diff.
- **Negative control** — one test that runs the same scenario in *free-running*
  mode and asserts the traces are **not** required to match (and, where it is
  stable enough to assert, that timestamps differ). Without it, someone later
  "fixes" a flake by freezing the clock in the wrong mode and the whole suite
  quietly stops testing determinism.
- **Wall-sleep tripwire** — a `debug_assert`/`tracing::error!` (better: a
  `Finding`) when `virtual_to_wall_us` is called while mode is `Stepped`. A
  grep-level CI check is a crude second line; the finding is the real one.

### CI shape

Add a `determinism` job alongside the existing `test` / `lint` / `docs` /
`supply-chain` / `msrv` jobs, wired into the `CI Gate` aggregator: build once,
run the determinism binary 5× as separate processes, then the golden compare.
Run it on the ubuntu leg first (the release-smoke leg is the precedent for a
single-OS extra leg); add macOS once the float-quantization question has real
data instead of speculation.

## Recommendation and phasing

Four phases, each independently shippable and CI-gated. **Do not start at T1.**

- **Phase D0 — determinism hygiene (do now, no new mode).** (a) Fix the
  `cluster_sources` hash-order defect. (b) Introduce `wait_until` and
  mechanically convert all 12 non-test sleep sites (peripherals/src/serial.rs x4, peripherals/src/timer.rs x2, tools/trace/src/recorder.rs x2, peripherals/src/pulse_out.rs, platforms/p2/src/ffi.rs, models/src/ads122u04.rs, models/src/ads122u04_component.rs) — behavior-preserving. (c) Land the
  engine `EventLog` and the N-run comparison test *in free-running mode as an
  observational test* so the baseline is measured rather than guessed. (d) Add
  a review rule: no `HashMap`/`HashSet` iteration on an engine path without a
  sort. Cheap, no behavior change, and it turns T1 from a rewrite into a swap.
- **Phase D1 — T1 for the board and models.** Stepped clock mode, engine as
  time authority, actor registry, models moved onto engine wakeups.
  **Deliberately defer the serial queue transport:** first prove determinism
  for systems whose I/O is already engine-side (net resolution, faults,
  stream routing, pacing, the ADS122U04 component). Gate: N-run identity +
  goldens, multi-process. This is the phase that makes the bench-bug suite
  exactly repeatable.
- **Phase D2 — T1 with firmware I/O.** `serial::Transport` +
  `QueueTransport`, the PTY exclusion rule, virtualized wall deadlines. After
  this, a whole-machine scenario (firmware + EdgeBoard + DS2 Addon + gantry) is
  free of wall-clock and host-load coupling, with the firmware's internal
  interleaving still free.
- **Phase D3 — T2, only on demand.** Cooperative firmware scheduling. **Do not
  build speculatively.** The trigger is a concrete reproducibility failure that
  D2 surfaces but cannot reproduce — e.g. a cog-to-cog race in the protocol
  layer or a static queue. Record the virtual-cost decision (T2 §3) before
  writing code.
- **Never — T3.** Instruction-level lockstep.

### What the consumer actually gets, per tier

Concretely, for the reference machine (`MaDSim` + EdgeBoard + DS2 Addon +
force path + gantry):

- **T0 (today).** Scenario suites that usually pass. Timing-boundary flakes
  papered over with tolerance windows. A bench bug reproduced by patience.
- **T1 (D1 + D2).** The bench-bug suite becomes *exactly* repeatable: the
  floating `~RESET` case, the crossed-TX/RX harness, the unstrapped AVDD case,
  `stream_drop` loss handling, and the force-path sample cadence all produce
  identical event traces run to run and machine to machine. Golden traces make
  wire-behavior changes a diff. Wall-clock tolerance windows come out of the
  tests. CI stops needing single-worker execution *for timing reasons* (the
  single-instance constraint still applies for other reasons).
- **T2 (D3).** A firmware concurrency bug — two cogs racing a protocol buffer,
  a missed lock, an ABBA between HAL locks — becomes a *reproducible failing
  test* instead of a 1-in-500 CI flake. And once a deterministic scheduler
  exists, systematic interleaving search follows almost for free: run the same
  scenario under N different deterministic schedules and look for a
  disagreement. That is the real prize, and the reason T2 is worth its cost
  *when* a race demands it.
- **T3.** Nothing this machine needs.

## Non-goals

- Deterministic host-PTY or browser-driven E2E (structurally impossible;
  flagged and excluded instead).
- Cross-architecture bit-identical floats without quantization.
- Instruction-level Propeller 2 emulation.
- Deterministic *wall* runtime. Stepped mode is faster or slower than real time
  by construction; how long a scenario takes on the host is not a guarantee and
  must never be asserted.
