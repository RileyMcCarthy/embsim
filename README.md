# embsim

[![CI](https://github.com/RileyMcCarthy/embsim/actions/workflows/ci.yml/badge.svg)](https://github.com/RileyMcCarthy/embsim/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

A **software-in-the-loop (SIL) simulator** for embedded firmware and the
boards it runs on, used as a tool: the `embsim` command.

embsim runs a system of boards with **no physical hardware**: each board built
from its vendor netlist with every part a node, the real firmware on an
instruction-set simulator in the processor's package, and the nets between
them resolved as circuits at the instants things happen.

You use it by writing a **project**: a TOML file that names the boards (a
KiCad netlist each, or a board a catalog ships), the model each part takes,
the bench components, the wires between the boards' connectors, and the
scenario. The `embsim` command takes a netlist to a running system through
one:

```bash
cargo install --path cli                    # from this checkout: the `embsim` tool
embsim survey board.net                     # what the netlist asks for
embsim new board.net -o board.toml          # a starter project answering it
embsim check board.toml                     # build it, time held; say what is left
embsim run board.toml --for 20ms            # run it in virtual time
```

**A project extends the tool.** When its boards need what embsim does not
ship (a part model, a board of its own, a processor core such as an
instruction-set simulator, a bench part such as a machine's mechanism), the
project writes them in Rust, in a **catalog crate** of its own, and names
the crate in its file (`[catalog] crates = ["sim/catalog"]`). The `embsim`
tool then builds the crate and embsim into one binary with Cargo, a
**runner** kept beside the project, and runs the project through it: the
same command, with the project's kinds beside embsim's. There is no plugin
interface; Cargo compiles one binary against one copy of embsim.
`embsim new --catalog DIR` starts such a crate.
[`examples/custom-project`](examples/custom-project/README.md) is one,
worked end to end, and [`PROJECTS.md`](PROJECTS.md) is the guide (section
10 for extending embsim).

It was extracted from the [MaD tensile tester](https://github.com/RileyMcCarthy/MaD),
and MaD's own move onto a project with a catalog crate is
[`MIGRATING-MAD.md`](MIGRATING-MAD.md). No generic crate depends on a
project crate (below). Rust code can also use the crates directly: a
project file loads with `embsim_board::Project`, and the command itself is
a library, `embsim_cli`.

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
   project       your project file, and your catalog crate: your boards,
                 models, CPU cores and bench parts
                      │
   command       embsim-cli        embsim survey / new / check / run; the runner
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
A project's kinds live in its own repository, in a catalog crate that
depends on embsim's crates; the runner the `embsim` tool builds is the one
place the two meet.

## Repository layout

| Crate | Path | What it is |
|-------|------|------------|
| `embsim-core` | [`core/`](core) | Virtual clock, serial PTY, event observers |
| `embsim-board` | [`board/`](board) | Netlist ingestion, net resolution, the one drive/sense interface, projects and the netlist survey |
| `embsim-models` | [`models/`](models) | Device models: ADS122U04, serial NOR flash, SD card, FAT16, regulators, gates, oscillators |
| `embsim-p2-qemu` | [`p2-qemu/`](p2-qemu) | The QEMU Propeller 2 target as a board component: boots the real ROM off a flash on the board's nets, with QEMU in a `qemu-system-p2` of its own. Carries the `target/p2` sources that program is built from |
| `embsim-boards` | [`boards/`](boards) | The P2-EC32MB from its vendor netlist, the P2 package a core sits in, and the standard catalog of board and part kinds a project names |
| `embsim-cli` | [`cli/`](cli) | The `embsim` command: survey a netlist, write a starter project, check it, run it (with QEMU as the P2's core, `embsim qemu install` installing the program it runs in), and build a project's own catalog crates into the runner that runs it. The guide is [`PROJECTS.md`](PROJECTS.md) |
| `yourproject-catalog` | [`cli/catalog-template/`](cli/catalog-template) | The catalog crate `embsim new --catalog` starts, compiled here so it stays true to the API |
| `custom-project-catalog` | [`examples/custom-project/catalog/`](examples/custom-project/catalog) | A worked example: a project's own part model, board, P2 core and bench component ([`examples/custom-project`](examples/custom-project/README.md)) |
| `embsim-memory-inspect` | [`tools/memory-inspect/`](tools/memory-inspect) | DWARF reader — recover C enums/structs/variables from an archive |
| `embsim-trace` | [`tools/trace/`](tools/trace) | Time-series trace recorder + live web viewer (feature `web`) |
| `embsim-ui` | [`tools/ui/`](tools/ui) | Pluggable web shell the trace viewer mounts into |
| `embsim-cpu-oracle` | [`cpu-oracle/`](cpu-oracle) | ISS-vs-silicon golden records (parse, diff). CPU adapters supply the image and ISS. |

The plan for making every netlist part a node in one pipeline — switches, capacitors, diodes, rails, the P2 package — is [`NODES.md`](NODES.md).

## What a new project provides

A project's netlists, its project file, and, for what embsim does not ship,
a catalog crate. The crate is an ordinary library whose root exports one
function the runner calls. The worked example's:

<!-- quoted from examples/custom-project/catalog/src/lib.rs -->
```rust
/// Add the example's kinds to `set`: the board, part and bench component
/// kinds as one catalog ([`board::ExampleCatalog`]), the core as a core
/// catalog ([`blinker::BlinkerCores`]). Starts nothing.
pub fn register(set: &mut CatalogSet) -> Result<(), ProjectError> {
    set.add(board::ExampleCatalog)?;
    set.add_cores(blinker::BlinkerCores)?;
    Ok(())
}
```

Each kind it adds is built through the one interface every embsim part
uses (a `Component`, a model in a board's part registry, or a `P2Core` in
the P2's package) and goes through the same survey and checks as embsim's
own. A kind's numbers carry their citations ([`DESIGN.md`](DESIGN.md)).
`embsim new --catalog DIR` starts a crate with one commented example of
each sort of kind; [`PROJECTS.md`](PROJECTS.md) §10 is the contract.

## Projects: a system in a file

A project is the system written down: the boards, the model each part takes,
the wires between connectors, and the scenario. The file names **kinds**; a
catalog (`embsim_boards::catalog::StandardCatalog`) turns each into a netlist
and registry, a part model, or a bench component. The file holds no
behaviour: every number stays in its model, with its citation.

```toml
[[board]]
name = "DS2Addon"
kind = "netlist"                       # any KiCad netlist export
netlist = "ds2_addon.net"              # relative to this file

[[board.model]]                        # a part the survey named
part = "ADS122U04"                     # exactly one of part, mpn, value
kind = "ads122u04"

[[wire]]                               # a supply of its own on a connector pin
from = "BENCH.3V3"
to = "DS2Addon.J1.1"
volts = 3.3
```

```rust
let project = embsim_board::Project::load("ds2-addon.toml")?;
let system = project.instantiate(&embsim_boards::catalog::StandardCatalog)?.start()?;
```

Every board is built by `Board::from_netlist` with a part registry: a
`kind = "netlist"` board from the catalog's base registry, which places every
model it knows by its manufacturer part number, and a catalog board kind
(`p2-ec32mb`) from its own. Before it builds, each board is **surveyed** with
the registry it will build with (`Project::survey`): the parts with no model,
named by the part name, number and value a `[[board.model]]` can key on; the
parts whose model declares other pins than the netlist gives them (a numbered
datasheet table against a netlist that names pins by function —
`options.pins` picks the other table); and the connectors, with their pins. A
board whose survey is not clean is refused with the survey as the error. A
wire's board end is a connector pin (`Board.Connector.Pin`); its other end is
another board's connector, a bench component's pin, or, with `volts`, a supply.
A `[[mate]]` joins two connectors at once, pin for pin by number or by a
cable's `map`: a module in its socket, a cable between two boards. A kind
seats only on a part that is what it says (a model's kind on a part whose
keys name its part family; `switch`, `boundary` and `mechanical` where the
board says so), so a part no kind is for needs a model.
The part kinds, their options and pin tables, the workflow from a netlist to
a running system, wiring boards to each other, and adding kinds of your own
are in [`PROJECTS.md`](PROJECTS.md); the example projects are in
`boards/projects/`.

### The `embsim` command

`cargo install --path cli` (or `cargo run -p embsim-cli --`) gives the
command that takes a netlist to a running system. Every step goes through the
same project, catalog and survey as the Rust above.

```bash
embsim survey board.net                 # the checklist
embsim new board.net -o board.toml      # a starter project
embsim check board.toml                 # build it, time held; say what is left
embsim run board.toml --for 20ms --net BOARD.VCC
```

- **`survey`** lists what the catalog populates (by class, and by part
  number), each part that needs a model with the kinds that could be it (by
  part number, else by part family; else the kinds without a model its
  designator, symbol or nets allow; else that it needs a model), each part
  placed with a pin table the netlist does not use with the table that fits,
  and every connector pin with its name and net. `embsim survey --kind
  p2-ec32mb` surveys a board kind the catalog ships the same way.
- **`new`** writes the project that answers the checklist as far as the
  catalog can: the board, a `[[board.model]]` choosing the pin table that
  fits for each part the catalog placed with another, a commented stub for
  each part that needs a model (the kind filled in when exactly one part
  number names it),
  and every connector's pins as the endpoints a `[[wire]]` names. Uncomment
  the stubs, choose the kinds, wire the connectors.
- **`check`** loads, surveys and builds the system and starts it with virtual
  time held: every part attached, nothing yet run. It prints each board's
  survey line and what the build found, and exits non-zero with the reason —
  the survey, for a board with a part still unmodelled — on any refusal.
- **`run`** starts the system on a stepped clock and runs it for `--for` of
  virtual time, or until interrupted (Ctrl-C), printing the build's
  findings, then findings and what the parts report (a P2's start and
  console, a host port's path) as the run reaches them, then what each part
  reports of itself, the nets asked for with `--net`, and each finding's net
  read again: the ones the run cleared (a rail that came up) apart from the
  ones still true. `--pty` says where a `host-serial` component's PTY goes.

A project whose file names catalog crates of its own (`[catalog]`) is
checked and run through a **runner**: `embsim` writes a small crate beside
the project (`.embsim/runner-<id>/`, kept out of git), builds it with Cargo
— the project's crates and embsim in one binary, reusing what the crates'
workspace built, and with nothing changed only Cargo's no-op check — and
hands the command line to it. `embsim new --catalog DIR` starts such a
crate with one commented example of each sort of kind, and names it in the
project (`--add-to PROJECT`, or the starter project `new` writes);
`check --rebuild` builds the runner afresh. `survey` and `new` take
`--project FILE` to run in that project's runner, so the checklist offers
the project's own kinds. The tool takes embsim's crates from the checkout
it was built from, or the project's `[catalog] embsim` (`PROJECTS.md`
section 10, "The runner").

The standard catalog's bench component kinds are `host-serial`, a host's
serial port as a PTY on the host's own rail, and `scripted-source`, a pin
driven through a list of steps (`PROJECTS.md` section 5).

The `embsim` command's set adds one core for the `p2` kind to the standard
catalog's: `core = "qemu"` seats the QEMU P2 in the package, which boots its
ROM (`rom = "file"` for another) off whatever the board gives it — on the
P2-EC32MB, the flash, which `image = "boot.bin"` on the `w25q128jv` kind
fills (`embsim_p2_qemu::flashimage` lays out stage-1 and a program). QEMU
is not linked into embsim: the P2 runs in `qemu-system-p2`, a program of its
own that `embsim qemu install` builds from the target embsim carries and
installs where the core looks (`embsim qemu path` says which one a run
would start; see `p2-qemu/README.md`). Without it the entry is refused,
saying how to install it. The boot as a project file is in
`cli/tests/cli.rs` (`run_boots_the_p2_off_the_modules_flash`).

## Using embsim in your project

embsim is a Cargo workspace of path crates (not yet on crates.io). A project
keeps it as a **git submodule**, so its catalog crate and the `embsim` tool
come from one pinned checkout:

```bash
git submodule add https://github.com/RileyMcCarthy/embsim.git vendor/embsim
cargo install --path vendor/embsim/cli      # or run it in place:
cargo run --release --manifest-path vendor/embsim/Cargo.toml -p embsim-cli -- check sim.toml
```

The catalog crate depends on embsim's crates by path, into that checkout:

```toml
# sim/catalog/Cargo.toml
[dependencies]
embsim-board  = { path = "../../vendor/embsim/board" }
embsim-boards = { path = "../../vendor/embsim/boards" }
embsim-core   = { path = "../../vendor/embsim/core" }   # the virtual clock
```

The runner takes embsim from the checkout the tool was built from, or from
the project's `[catalog] embsim`, refuses a catalog crate whose embsim
dependencies are another copy before it builds, and spells the checkout as
the crates do (`PROJECTS.md` §10, "Which embsim the runner builds
against"). A project
workspace should `exclude` the submodule directory (embsim is its own
workspace root); path dependencies across the boundary work fine:

```toml
[workspace]
exclude = ["vendor/embsim"]
```

A project that would rather own its binary than have the tool build a
runner writes ten lines over the command's library (`embsim_cli::shipped`,
its registration function, `embsim_cli::main_with`); the example's is
`examples/custom-project/catalog/examples/own_binary.rs`.

## The QEMU P2 is a program of its own

Each `P2Qemu` starts a `qemu-system-p2` of its own and takes turns with it
over a shared page, so a board may carry several P2s and a process may run
several systems' worth. The program dies with its node, and with the
process that started it however that ends (`p2-qemu/README.md`, "Where the
CPU runs, and why"). It is QEMU, a GPL-2.0 program, installed beside its
licence and source; embsim links none of it.

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
cargo test -p embsim-p2-qemu        # P2 core; with `embsim qemu install` done, -- --include-ignored boots the ROM, the pad-mode and PLL benches
cargo test -p embsim-boards         # the P2-EC32MB board against its netlist
cargo test -p embsim-memory-inspect # DWARF parser (compiles a tiny C fixture at test time)
cargo test -p embsim-trace          # trace recorder
cargo test -p embsim-ui             # web shell render + handlers
cargo test -p embsim-cli            # the command as a user runs it, and as a library
cargo test -p custom-project-catalog                # the worked example's project, in process
cargo test -p embsim-cli --test runner -- --ignored # the tool building runners with Cargo (CI: project-runner)
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
