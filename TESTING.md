# embsim testing conventions

Every crate in this workspace is firmware-free and unit-testable. This document
is the contract for **how** tests are written so coverage stays uniform across
peripherals, models, board engine, runtime, and tools.

## Running the suite

```bash
cargo test --workspace --all-targets
cargo test --workspace --doc
cargo test -p embsim-trace --no-default-features   # headless recorder path
cargo test -p embsim-board --release   # timing-sensitive smoke

# Determinism (Oracle 1). `--nocapture` prints, per case, the asserted stepped
# identity and the measured free-running divergence; see rule 8 below.
cargo test -p embsim-board --test determinism -- --nocapture
# Stepped-clock mechanics (the barrier, the time-release, the wedge report).
cargo test -p embsim-board --test stepped_clock --test ads122u04_stepped

# The EdgeBoard's RS-422 pair live, an SD card driven bit by bit over nets,
# and the RS-422 receiver started twelve times in one process (each stepped,
# its own binary; live reads at a settled instant, rule 9).
cargo test -p embsim-board --test edgeboard --test sd_card_spi --test rs422_determinism

# The isolation parts, promoted from stubs on the real EdgeBoard netlist
# (stepped, own binary; every read after a 1 ms virtual settle, rule 9):
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
# Read ID answered over the module's own nets. Since the interface phase's
# cleanup: only the buffer's fed-back stage is self-biased (the build finds
# `R101` from `2Y` back to `2A`), and a clock driven straight onto a plain
# input is relayed only when its phases cross the input's thresholds. The
# final pass: a fed-back stage is self-biased only where it inverts with a
# plain input — a buffer fed back the same way reads a coupled 0.8 V swing
# as a steady low, and a Schmitt inverter drives levels and relays none of
# it (read at a settled instant, the case's thread an actor).
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
# stopped — and nothing with RESN asserted first); and the fast pad's
# strength fitted to the datasheet's output table. The
# final pass: a coupled 0.8 V swing on `XI` is the crystal from the instant
# the core's clock word turns `XI`'s 1 MΩ feedback on, and none in the
# clock mode the chip starts in.
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
# The QEMU core inside the package (needs `embsim qemu install`'s
# qemu-system-p2, so these are #[ignore]d without --include-ignored): the
# ROM boot prints its edges / yields / publishes / START instant / wall time
# / turns and the channel's cost per turn, holds its yields, publishes and
# clock edges exactly, and its escalated-solve count (since phase 4 the module is
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
# guests start 3 ms in, the restart delay after the build; `pad_modes` runs
# two P2s, each its own program. `lifecycle`: a program killed mid-run stops
# its core with the signal in the error, one sent SIGTERM, SIGINT or SIGHUP
# mid-run ends within a second over either channel, a dropped core takes
# its program with it, and a program whose parent is killed ends itself.
# EMBSIM_P2_QEMU_TRANSPORT=socket runs them over the fallback channel.
cargo test -p embsim-p2-qemu -- --include-ignored --nocapture
# Without QEMU (runs everywhere): `program` starts a stand-in that speaks
# the protocol — a program of another protocol or target refused naming
# both and saying how to install the right one, one that exits before its
# handshake or dies mid-run reported with its status and standard error
# within a second, one that will not quit killed when its node drops — and
# checks that stage.sh and the crate compute one target identity.
cargo test -p embsim-p2-qemu --test program
# The interface phase's sense task (`NODES.md` §12 item 5; stepped, own
# binary): what a pin is handed is a
# voltage against its declared reference, and the level is the receiver's
# own projection — a 1.2 V node read low, high, low through a Schmitt
# receiver's hysteresis, a holding receiver against one that reads no level
# inside the dead band, the same 1.5 V read high by a P2 pad in a 1.8 V bank
# and no level by an LVCMOS receiver, a reader against a reference pin at
# 1 V, a floating sense handed no voltage with its finding, a fought net
# handed its 1.65 V operating point with the contention beside it, a supply
# move re-delivering a pad's sense (waited on as the reading itself), and —
# the cleanup — a stepper counting a step clock only when its phases cross
# its `STEP` thresholds; the final pass — a step clock that stops crossing
# and crosses again under one schedule counted once per pulse, from the
# instant it crosses again, wherever that lands against the drive's own
# position samples, and a build's senses carrying instant 0 whatever the
# clock reads.
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
# interface phase) lives in `board/src/net.rs`; its arithmetic unit tests
# (floor-division, 128-bit span, clamp, rebase fidelity) run under the board
# lib. The stepper plant that folds it is unit-tested in models: an hour of
# a 10 MHz train counts exactly, a rate change between two microseconds
# folds each side exactly.
cargo test -p embsim-board --lib a_segment_integrates_like_run
cargo test -p embsim-board --lib a_long_fast_train_counts_without_overflow
cargo test -p embsim-board --lib a_finite_segment_clamps_and_reports_its_completion
cargo test -p embsim-board --lib rebasing_a_segment_hands_over_the_exact_count
cargo test -p embsim-board --lib rebasing_trails_the_original_by_at_most_one_pulse
# Or all schedule-arithmetic cases together:
# cargo test -p embsim-board --lib
cargo test -p embsim-models --lib stepper_motor
# Projects (`board/src/project.rs`, `boards/src/catalog.rs`): a system written
# down as a TOML file of boards, part models, wires and a scenario. Build
# only: the P2-EC32MB from its netlist through the catalog equals the module
# `Ec32mb` builds, part for part, net for net and cluster for cluster; the
# checklist its netlist asks for with nothing assigned; the carrier file's
# switch positions; connectors mated pin for pin and by a cable's map; and
# every refusal, with the text that says what to fix — a kind on a part it is
# not, one key twice, one supply name twice, a mate that does not fit, and
# the MaD machine's three-board project waiting on its two unmodelled parts.
cargo test -p embsim-boards --test ec32_project --test project_refusals --test mates
# Live, stepped, each its own binary: the EC32 netlist project's power tree
# from the carrier's fingers (every rail equal to the library module's); the
# DS2 add-on project on its bench supplies (its ADC's protocol thread lives
# for the process, rule 5); two boards joined connector to connector, a pad
# on one read through the other's resistor.
cargo test -p embsim-boards --test ec32_project_power --test ds2_project --test board_to_board
# The MaD machine's three boards as a project (`boards/projects/edge-ec32-ds2.toml`),
# with the Edge board's RS-422 pair given the board tests' models: its mates
# join what the machine tests' hand-written harnesses join (build only), and
# live, stepped, the module and the add-on run from the carrier's rails.
cargo test -p embsim-board --test edge_project --test edge_project_live
# The catalog's guide (`boards/src/catalog.rs`): every pin table it lists is
# the one its kind registers; how a part fits a kind — by part number, by
# part family, never by pins — and which kinds without a model a part may
# take by its designator, symbol and nets; and PROJECTS.md's tables of
# board and part kinds,
# which are generated from the catalog (each option read off the kind's own
# registration) and must match the document character for character — the
# failure prints the tables to paste.
cargo test -p embsim-boards --lib catalog
# PROJECTS.md's Rust examples, run as doc tests of embsim-boards from its
# directory: a project loaded, surveyed and built, a catalog of one's own
# adding a board kind and a part kind to a catalog set, a core catalog
# adding a P2 core, a board kind that brings its own entry for a socket of
# its own symbol library, and a report as a run takes and reads it.
cargo test -p embsim-boards --doc
# The guides' quotations (build only): every code block README.md,
# PROJECTS.md, TESTING.md, MIGRATING-MAD.md or the example's README marks
# `<!-- quoted from PATH -->` is that file's text, line for line, and
# PROJECTS.md quotes the worked example's registration function and its own
# binary. The quoted files are the example's, which every gate compiles, so
# the guide's code that is not a doc test is still code that builds.
cargo test -p embsim-boards --test guide_quotes
# Catalogs composed into one (`boards/src/set.rs`), build only: a kind two
# catalogs provide refused where a project names it, naming both, and a
# project that does not name it surveyed; a core and a base part number
# the same; what a set refuses when a catalog joins; an added kind refused
# on a part it is not before its catalog is asked; a board kind's own
# entries, and a project entry replacing one.
cargo test -p embsim-boards --test catalog_set
# The standard bench component kinds, stepped, each its own binary: a
# scripted source's steps landing at their instants across a divider (read a
# nanosecond either side of each, and by a reader handed each at its
# instant), two host serial ports as a null-modem cable carrying bytes
# between their PTYs both ways (written once running, and before the
# system starts, none shed), a host's TX driving at whatever its VIO
# reads (unsourced, 1.8 V, 3.3 V, each at its instant), and a PTY path
# holding a file or a directory refused and left; and what each kind
# refuses. (`cargo test -p embsim-core --test serial_pty` holds the PTY
# link's own rule: it replaces only a link, and removes only its own.)
cargo test -p embsim-boards --test scripted_source --test host_serial
# The `embsim` command (`cli/`), run as a user runs it, a process a case:
# survey the EC32's netlist (every connector pin, what needs a model and what
# could be it, the pin table that fits), the Edge board's (the two parts no
# kind is for, the socket offered as a connector, a symbol and a part number
# that name two parts) and the p2-ec32mb kind; `new` then `check` for the header
# board, the DS2 add-on (refused with its survey until its stub is filled)
# and the EC32 (the pin tables the hand-written project chooses); `run` of
# the EC32 project for 10 ms, its rails up, the build's findings apart and
# the ones the run cleared listed at the end, the report the same twice.
# Where no qemu-system-p2 can be found, `check` of a `core = "qemu"` project
# refused saying where it looked and how to install one, `qemu path` failing
# the same way, and `qemu install --dry-run` naming the target, the QEMU
# release and where it would go; with one installed (--include-ignored),
# `run` boots the P2 off the module's flash (its image made by `embsim
# flash-image`), a run whose qemu-system-p2 is killed stops there and exits
# non-zero, and `qemu path` finds it. `flash-image` laying out a program and
# refusing one stage-1 cannot load, and a run off a raw image saying a P2
# does not boot from it, run everywhere. `run` with no
# duration interrupted by SIGINT, its summary printed; `run --pty` putting a
# host's PTY where it says and printing its path, and refusing a path that
# holds a file. And the command as a
# library (`cli/tests/library.rs`, in process): a catalog the test defines
# adds a board kind, a part kind, a P2 core and a bench component, and a
# project naming all four checks and runs through `embsim_cli::run`; and
# (`cli/tests/failure.rs`) a part whose report fails stops the run there,
# with its summary, and the command exits non-zero. A
# project's own catalog crates (`cli/tests/runner.rs`): `new --catalog`
# starting a crate (its embsim by path inside the project, else what
# `--embsim` names, else the project's crates' own) and its own runner (a
# `.gitignore` for its builds when it is its own workspace), naming them
# in a project, the tool's refusals before Cargo, the runner's files written
# when no Cargo starts, a runner refusing a project naming other crates,
# `survey` and `new` with `--project` through a runner, a command line the
# tool cannot parse handed over; and, with a stand-in `$CARGO` that reads
# manifests and builds nothing, a lone crate's runner built under
# `.embsim/target`, the runner's embsim taken from the crates' git source,
# a crate on another embsim refused before any build, an embsim Cargo cannot
# fetch named with where to point the crates, a crate's dependency
# on another checkout named as two copies when Cargo refuses the second
# `links = "embsim-core"`, two copies refused by Cargo's resolver, a
# symlinked checkout spelled as the crate spells it, `embsim.lock` copied in
# and built `--locked`, and a project's own runner built in its workspace.
# What a binary is made of, on every check and in `--version`
# (`cli/tests/cli.rs`). The crate `new --catalog` starts,
# its four kinds run in process, its source driving at its instant
# (`cli/tests/template.rs`).
cargo test -p embsim-cli

# The runner built with Cargo (`#[ignore]`d above; CI's project-runner job):
# examples/custom-project checked and run through the real binary and the
# runner it builds, --locked against its committed embsim.lock, a quiet
# second build and a --rebuild that keeps the lock, a started crate built
# outside any workspace whose first run writes embsim.lock, a project's own
# runner built in its workspace and run, a runner crate that is its own
# workspace, committed, whose line names its commit and no changes after a
# build, and a crate that does not compile
# shown with rustc's errors. The first build of embsim in the release
# profile is the slow part.
cargo test -p embsim-cli --test runner -- --ignored

# The worked example's own test: its project run as its runner runs it, in
# process, every edge at its nanosecond (stepped, own binary). The crate is
# a workspace member, so `cargo test --workspace` (CI's test job) runs this
# and compiles its own binary (`examples/own_binary.rs`); clippy and doc
# take it too. Run it as a user would, from examples/custom-project:
#   cargo run -p custom-project-catalog --example own_binary -- run project.toml --for 10ms
cargo test -p custom-project-catalog
```

How CI runs the project pieces: the `test` job's `cargo test --workspace
--all-targets` and `--doc` run everything above but the `#[ignore]`d runner
builds, among them the example's test, the started crate's test
(`cli/tests/template.rs`), the guide's doc tests and `guide_quotes`; the
`project-runner` job runs `cargo test -p embsim-cli --test runner --
--ignored`, which builds real runners with Cargo; the `p2-qemu-boot` job
installs `qemu-system-p2` with `embsim qemu install` and runs `cargo test
-p embsim-p2-qemu` and `cargo test -p embsim-cli --test cli` with
`--include-ignored`, where `run` boots the P2 off the module's flash. The
behaviour ledger's suite (`vibes.suite.json`) runs `embsim-board`,
`embsim-boards`, `embsim-cli`, `embsim-p2-qemu` and `custom-project-catalog`,
so the runner builds and the QEMU boots declare no behaviours: the ledger's
run builds no runner and installs no QEMU. What runs without them (the
stand-in program, the refusals) declares its behaviours.

**A project's own catalog** is tested the way the example's is, in the
project's repository: a test that runs the project file through
`embsim_cli::run_with_crates` with the crate's registration function, in
process and stepped inside the command, asserting on what the run prints
(`examples/custom-project/catalog/tests/project.rs`); and, for a property
the printout does not carry (a byte crossing a PTY, a net's level at an
instant), a test that loads the file with `embsim_board::Project`, builds
it with the shipped set and the crate's kinds, and runs the system on the
stepped clock as `board/tests/edge_project_live.rs` does (rule 9). Neither
builds a runner. `MIGRATING-MAD.md` lists MaD's.

Per-crate iteration:

```bash
cargo test -p embsim-core
cargo test -p embsim-models
cargo test -p embsim-board
cargo test -p embsim-memory-inspect
cargo test -p embsim-trace
cargo test -p embsim-ui
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

3. **Assert contracts, not wall flakiness.** Prefer virtual-time schedules,
   monotonicity, clamps, and ε windows. Dedicated paced-stream tests that pin
   scale and assert wall delay are the exception (document why). A wall-clock
   wait for what a stepped run must do *eventually* — a burst to finish
   crossing, a node to reach its next slice — is sized for a hang, never for
   a speed: the wall time a stepped run takes is the engine's cost per edge
   times its edges, and the runner decides that.

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
   spawn long-lived actor threads still belongs in its own test binary so
   leftover actors cannot hold a later case's barrier. The ADS122U04 model's
   protocol thread is an actor only while its part lives: it ends when the
   system drops the part, so a binary of several converter cases waits after
   each shutdown for the actor count to fall back before the next case
   re-anchors the clock (`board/tests/ads122u04_registers.rs`).

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

9. **A model's proving tests run in stepped mode, and read at a settled
   instant.** A free-running read cannot tell a settled system from a cascade
   in flight: `System::start` returns with the attach cascade still running,
   and a wall-clock poll for the state a case expects passes early whenever
   that state is also a pin's idle. The AM26LV32's `1Y` idles `Driven(High)`
   and is released twice on its way to driving it (the test-tree model
   publishes at its supply's delivery and `G`'s, before `~G` enables it), so
   a poll that accepted the idle and then read again read `Floating`. That was
   `board/tests/rs422_determinism.rs`'s one run in twelve — reproduced
   2026-09-23 and taken then for a system settling two ways from one input,
   root-caused 2026-09-25 as this read race (`NODES.md` §12 item 5, the flake
   record and the performance and stepped-tests record: every run reached the
   one settled state) — and the `Floating` reads that flaked
   `isolation_bridge.rs` and `edgeboard.rs`; a fixed wall wait for a card's
   answer flaked `sd_card_spi.rs`. So a new model's proving test — the tests
   `NODES.md` §8 lists as each phase's proof — starts its system in stepped
   mode (`virtual_clock::init_mode(ClockMode::Stepped, …)`), and a case that
   reads a live system follows `isolation_bridge.rs`: a suite lock, the clock
   re-anchored stepped, the system started with time held, the case's thread
   registered as a virtual-clock actor and time released, every wait a virtual
   `settle()` longer than any instant the rig arms (asserted at compile time),
   and no `QuiescenceTimeout` at the end. The engine then advances only while
   the case is parked, so a read between two settles is deterministic: every
   wake due before the settle's deadline has fired. One due at the deadline
   itself fires after the case parks again (at an instant, released actors
   run before the engine fires what is due there), which is why the window
   is longer than any span the rig arms after a settled instant — then no
   wake falls on a deadline and the read is the system at rest. The order of
   the held start-up instant is still the attach thread's race with the
   engine's deliveries — the state it comes to rest in is not — so such a case
   compares states at rest, not start-up logs. A free-running case may still
   exist beside it to measure divergence, under rule 8; it is not the proof,
   and it never waits for a value an idle can satisfy. Every sense→drive
   cascade a new model adds is one more transient such a wait can land in.

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

- Live topology mutation after `System::start`
- Dual-MCU firmware entry inversion (one image per process still applies)

## MaD pin bumps

Consumer repos (e.g. MaD) should re-run this suite against the **pinned**
submodule commit on SIL-related PRs (`cd vendor/embsim && cargo test
--workspace --all-targets`), mirroring how ProtoEmb is gated, and check
their own project files with the `embsim` built from that commit, which
builds their catalog crates against it (`MIGRATING-MAD.md`, step 9).
Upstream CI on this repo remains the primary gate for commits that land
here.

## Releases

A release is a `v*` tag on a commit the CI Gate passed. CI's
`release-binaries` job builds `embsim` for each platform the release
ships, where it runs, and runs it before packaging it; `release` then
publishes, on the tag's push only, once the gate and every binary have
passed (`NODES.md` §13, "The release"). Both run
`.github/scripts/release.py`, which a maintainer runs the same way.

Before tagging:

```bash
# The version in Cargo.toml ([workspace.package]) is the release's, and
# CHANGELOG.md has its `## [X.Y.Z]` section.
python3 .github/scripts/release.py check --tag vX.Y.Z

# This machine's binary, run and packaged as CI does (it must be built from
# HEAD with no changes to embsim's crates), then the P2 target's sources
# and the checksums. target/ keeps the archives out of git.
cargo build --release --locked -p embsim-cli --target aarch64-apple-darwin
python3 .github/scripts/release.py binary --target aarch64-apple-darwin --out target/dist
python3 .github/scripts/release.py qemu-target --out target/dist
python3 .github/scripts/release.py checksums target/dist
```

Every platform's binaries, without publishing anything: run the CI
workflow by hand on the branch (Actions, CI, "Run workflow"). The
`release-binaries` legs keep their archives as the run's artifacts. Then
tag the merged commit, `git tag -a vX.Y.Z -m "embsim X.Y.Z"`, and push
the tag.
