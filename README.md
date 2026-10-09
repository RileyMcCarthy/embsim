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
scenario. The `embsim` command ([Install](#install)) takes a netlist to a
running system through one:

```bash
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
**runner** — the project's own crate, or one the tool keeps beside the
project — and runs the project through it: the same command, with the
project's kinds beside embsim's, against the embsim the catalog crate
itself depends on. There is no plugin interface; Cargo compiles one binary
against one copy of embsim.
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

## Install

`embsim` runs on macOS and Linux. Take a release's prebuilt binary, or
build it from source with Cargo; the P2 on QEMU needs one more program,
which `embsim` builds for you.

**A prebuilt binary**, from the
[releases](https://github.com/RileyMcCarthy/embsim/releases): an
`embsim-<version>-<target>.tar.gz` for macOS on Apple silicon
(`aarch64-apple-darwin`), macOS on Intel (`x86_64-apple-darwin`) and Linux
on x86_64 (`x86_64-unknown-linux-gnu`, glibc 2.35 or newer: Ubuntu 22.04,
Debian 12), each with its SHA-256 beside it, and every one in
`SHA256SUMS`:

```bash
v=0.2.0 t=aarch64-apple-darwin
curl -fLO "https://github.com/RileyMcCarthy/embsim/releases/download/v$v/embsim-$v-$t.tar.gz"
curl -fLO "https://github.com/RileyMcCarthy/embsim/releases/download/v$v/embsim-$v-$t.tar.gz.sha256"
shasum -a 256 -c "embsim-$v-$t.tar.gz.sha256"           # or sha256sum -c
tar -xzf "embsim-$v-$t.tar.gz"
mkdir -p ~/.local/bin && install -m 755 "embsim-$v-$t/embsim" ~/.local/bin/   # any directory on PATH
embsim --version
```

Beside the binary are this README, the changelog, the licences, a
`NOTICE` for what the binary carries and `THIRD-PARTY-LICENSES.txt` for
the crates compiled into it. The binaries are not signed: on macOS a copy
a browser downloaded is quarantined, and `xattr -d com.apple.quarantine
embsim` clears that (curl sets no quarantine).

**From source, with Cargo** (Rust 1.88 or newer):

```bash
cargo install --locked --git https://github.com/RileyMcCarthy/embsim --tag v0.2.0 embsim-cli
```

or, in a checkout, `cargo install --locked --path cli`, or `cargo run -p
embsim-cli --` in place. However it is installed, a project that names
catalog crates of its own needs Cargo too: `embsim` builds the project's
runner with it ([Using embsim from another
repository](#using-embsim-from-another-repository)).

**The P2 on QEMU** (`core = "qemu"` on the `p2` kind) runs in
`qemu-system-p2`, a program of its own that embsim does not include. Build
and install it once, for the embsim you have:

```bash
embsim qemu install   # QEMU v10.1.0 and embsim's P2 target, into ~/.embsim/qemu/<target>/
embsim qemu path      # which qemu-system-p2 a run would start, and whether it fits
```

It needs git, a C compiler, ninja, pkg-config, glib's development files
and python3 (on Debian or Ubuntu, `sudo apt-get install git
build-essential ninja-build pkg-config libglib2.0-dev python3-venv flex
bison`; on macOS, `xcode-select --install`, then `brew install ninja
pkgconf glib`), and a few minutes the first time. Each release also
attaches `embsim-<version>-qemu-target.tar.gz`: the target's sources,
the same files `embsim qemu install` builds from, with the steps to build
them by hand. [`p2-qemu/README.md`](p2-qemu/README.md) says where a run
looks for the program, and the program's licence.

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
| Analog | ADS122U04 front end at settled values, its input, gain and reference the firmware's register writes; force and encoder plants; input ports stamped on senses (the AM26LV32's open-input bias); a single source into an analog reader delivered unsolved | RC settling at conversion instants (5) | noise, amplifier loops, oscillator start-up |
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
   host          embsim-qemu       a VM the board's clock meters (Chrome guest)
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
| `embsim-qemu` | [`qemu/`](qemu) | The host computer as a board component: a virtual machine on the host's own system QEMU, its serial port levels on nets, its guest run only while the board's clock advances (`qemu-vm`, `chrome-vm`). Carries the Chrome guest's image recipe, [`qemu/guest/chrome`](qemu/guest/chrome/README.md) |
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

The command ([Install](#install)) takes a netlist to a running system.
`embsim --version` says which release, git revision, compiler, target and
profile it is, and every `check` and `run` prints the same under its
project line. Every step goes through the same project, catalog and survey
as the Rust above.

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
  A part that fails — a P2 core whose `qemu-system-p2` died — ends the run
  at that look, and the command exits non-zero with the reason.

A project whose file names catalog crates of its own (`[catalog]`) is
checked and run through a **runner**: the project's crates and embsim in
one binary, which `embsim` builds with Cargo — with nothing changed only
Cargo's no-op check — and hands the command line to. The runner is the
project's own crate when the file names one (`[catalog] runner`, built in
the project's Cargo workspace against its lock file), else a small crate
the tool writes beside the project (`.embsim/runner-<id>/`, kept out of
git) whose lock it keeps as `embsim.lock`, for the project to commit.
Either way the runner's embsim is the one the catalog crates depend on, by
path, git revision or release, whichever `embsim` tool started it.
`embsim new --catalog DIR` starts such a crate with one commented example
of each sort of kind, and names it in the project (`--add-to PROJECT`, or
the starter project `new` writes); `--own-runner` starts the runner crate
beside it. `check --rebuild` builds the runner afresh. `survey` and `new`
take `--project FILE` to run in that project's runner, so the checklist
offers the project's own kinds (`PROJECTS.md` section 10, "The runner").
A project names the embsim release it is written for with
`requires-embsim = "0.2"`, which every embsim reads first (`PROJECTS.md`
section 2).

The standard catalog's bench component kinds are `host-serial`, a host's
serial port as a PTY on the host's own rail, and `scripted-source`, a pin
driven through a list of steps (`PROJECTS.md` section 5).

The `embsim` command's set adds one core for the `p2` kind to the standard
catalog's: `core = "qemu"` seats the QEMU P2 in the package, which boots its
ROM (`rom = "file"` for another) off whatever the board gives it — on the
P2-EC32MB, the flash, which `image = "boot.bin"` on the `w25q128jv` kind
fills: `embsim flash-image PROGRAM -o boot.bin` lays out a P2 program
behind embsim's stage-1 loader as the flash the ROM boots. QEMU
is not linked into embsim: the P2 runs in `qemu-system-p2`, a program of its
own that `embsim qemu install` builds from the target embsim carries and
installs where the core looks (`embsim qemu path` says which one a run
would start; see `p2-qemu/README.md`). Without it the entry is refused,
saying how to install it. The boot as a project file is in
`cli/tests/cli.rs` (`run_boots_the_p2_off_the_modules_flash`).

## Using embsim from another repository

A project with no kinds of its own needs only the `embsim` tool
([Install](#install)), and `requires-embsim = "0.2"` in its file to say
which release it is written for (`embsim new` writes it).

A project with a catalog crate names embsim in that crate's `Cargo.toml`,
and that is the embsim its runner is built against, whichever `embsim`
tool starts the build (`PROJECTS.md` §10, "Which embsim the runner builds
against"). embsim is not on crates.io; take it as a **git dependency** at a
release:

```toml
# sim/catalog/Cargo.toml
[dependencies]
embsim-board  = { git = "https://github.com/RileyMcCarthy/embsim", tag = "v0.2.0", version = "0.2" }
embsim-boards = { git = "https://github.com/RileyMcCarthy/embsim", tag = "v0.2.0", version = "0.2" }
embsim-core   = { git = "https://github.com/RileyMcCarthy/embsim", tag = "v0.2.0", version = "0.2" }   # the virtual clock
```

or as a **git submodule**, by path, so the catalog crate and the tool come
from one pinned commit (a Cargo workspace around it `exclude`s
`vendor/embsim`, which is a workspace of its own):

```bash
git submodule add https://github.com/RileyMcCarthy/embsim.git vendor/embsim
git -C vendor/embsim checkout v0.2.0
cargo run --release --manifest-path vendor/embsim/Cargo.toml -p embsim-cli -- check sim.toml
```

```toml
# sim/catalog/Cargo.toml
[dependencies]
embsim-board  = { path = "../../vendor/embsim/board" }
embsim-boards = { path = "../../vendor/embsim/boards" }
embsim-core   = { path = "../../vendor/embsim/core" }   # the virtual clock
```

`embsim new --catalog DIR` writes whichever fits: the path when the tool's
own checkout sits inside the project's repository, else the repository at
the tool's revision when a remote holds it — a tool built from a commit
never pushed, or with uncommitted changes, writes its checkout's path and
says why — or what `--embsim PATH|URL@REF` names
(`--embsim https://github.com/RileyMcCarthy/embsim@v0.2.0`). A build that
cannot fetch embsim from where the crates say is reported as that, with
where to point them. Every catalog crate names the same embsim: the tool
refuses another before it builds, and Cargo refuses a second copy anywhere
in the graph (`embsim-core` claims `links = "embsim-core"`).

**Who owns the runner.** By default the tool writes it beside the project,
under `.embsim/` (which git ignores), and keeps its lock as `embsim.lock`
next to the project file: commit that, and every machine builds the same
runner. A project in a Cargo workspace of its own owns its runner instead:
a member crate whose `main` is `embsim_cli::runner_main` over the catalog
crates, named in the file and built `--locked` against the workspace's
`Cargo.lock`. `embsim new --catalog sim/catalog --own-runner` starts both:

```toml
[catalog]
crates = ["sim/catalog"]
runner = "sim/runner"
```

A project that would rather own the whole binary writes ten lines over the
command's library (`embsim_cli::main_with`); the example's is
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
cargo test -p embsim-qemu          # the VM host on the board's clock; -- --ignored runs it against real QEMU (qemu-system-aarch64), and the Chrome guest once its image is built
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

embsim's code, in every crate, is MIT: see [LICENSE](LICENSE). Two crates
also carry material under other licences, which their licence fields name
beside MIT, and every `embsim` binary carries it too:

- `embsim-boards` compiles in `boards/netlists/p2_ec32mb.net`, a
  transcription of Parallax's P2-EC32MB schematic, which Parallax
  publishes under CC BY-SA 4.0; the transcription is CC BY-SA 4.0 too, so
  that crate's licence is `MIT AND CC-BY-SA-4.0` (`boards/netlists/LICENSE`).
- `embsim-p2-qemu` carries the QEMU Propeller 2 target in
  `p2-qemu/qemu-target/`: the target and board are LGPL-2.1-or-later, its
  two QEMU patches carry the licences of the QEMU files they modify
  (GPL-2.0-or-later and MIT), its decode table is MIT (from PNut-TS); each
  file says which, and the texts are in [`LICENSES/`](LICENSES) and
  `p2-qemu/qemu-target/LICENSE-PNut-TS`. The crate embeds every one of
  those files, verbatim, in each binary that links it, as data for `embsim
  qemu install` to write out and build into QEMU; none is compiled as
  code. So that crate's licence is `MIT AND LGPL-2.1-or-later AND
  GPL-2.0-or-later`: MIT for its Rust, and the other two for the files it
  carries. A release's binary archive says the same in its `NOTICE`.

**`qemu-system-p2` is a separate program**, a GPL-2.0 work: QEMU is
released as a whole under version 2 of the GPL, and the target becomes part
of it. `embsim qemu install` builds it from QEMU at the pinned commit and
those target sources, and installs it with QEMU's licence texts, the target
sources it was built from and a `NOTICE` saying how — its corresponding
source. embsim runs it as a separate process and talks to it over a small
fixed protocol; it links none of QEMU, and nothing of embsim is in the
program (`p2-qemu/qemu-target/README.md`, "License"). This says how the
pieces are put together; it is not legal advice.
