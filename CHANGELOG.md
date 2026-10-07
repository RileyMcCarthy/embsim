# Changelog

What each release of embsim changes for someone using it. Releases are
git tags, `vX.Y.Z`; while embsim is 0.x, a minor release may break what
the one before it promised, and says so here under **Breaking**.

## [0.2.0]

embsim is now a board simulator you use as a tool. 0.1.0 linked a
project's firmware C against Rust stand-ins for its HAL. 0.2.0 builds each
board from its vendor netlist, makes every part on it a node, and runs
the real firmware image on an instruction-set simulator inside the
processor's package. You describe the system in a project file and run it
with the `embsim` command; a project adds its own models, boards, cores and
bench parts in a Rust crate of its own. [`DESIGN.md`](DESIGN.md) has the
rules the engine keeps, [`PROJECTS.md`](PROJECTS.md) is the guide, and
[`MIGRATING-MAD.md`](MIGRATING-MAD.md) is a whole project's move onto it.

### Breaking

- **The HAL path is gone**, and with it five crates: `embsim-runtime`,
  `embsim-peripherals`, `embsim-p2` (`platforms/p2`), `embsim-build`
  (`build-support`) and the `examples/minimal` template. Firmware is no
  longer linked into the simulator as C behind `#[no_mangle]`
  trampolines. Its image runs on a CPU core in the processor's package, on
  a board built from its netlist. A 0.1.0 consumer, a platform crate plus a
  `Machine`, becomes a project file, plus a catalog crate for whatever
  embsim does not ship (below). `embsim_models::limit_switch`, a model of
  the HAL path, went with it. [`MIGRATING-MAD.md`](MIGRATING-MAD.md) works
  this through for MaD, the reference consumer.
- **A byte cannot cross a net unrouted.** The byte route
  (`StreamRole::Producer`/`Consumer`) is gone: a UART byte travels as
  levels on the wire, framed at its baud, so a fought or floating line
  breaks it the way it would on the board.
- **One interface between a part and the board** (below) replaces
  `PinKind`, `StreamRole`, `PinDecl::stream`, `pulse_tx`/`on_pulse` and the
  pulse commands. A step train is a `Drive::Periodic`, and `PulseSegment` is
  `PeriodicSchedule`, counted in nanoseconds. `NetState` is the engine's
  report of a net, not something a part is handed.
- **No stub tier.** `Stubbed`, `Ignored`, `from_netlist_with_stubs` and the
  `STUB_REFS` lists are gone. A part with no model is a build error that
  names it (`RegistryError::UnknownPart { reference, part, value }`).
- **Virtual time is counted in nanoseconds** (`virtual_ns`). Every
  microsecond call is still there, as a wrapper. Paced serial is exact to
  the nanosecond, so golden traces recorded with 0.1.0 move.

### Every component a node

- **The netlist is the model.** Every part on a board is a node, or the
  board refuses to build and names the part. Resistors and closed switch
  poles are conductances, and capacitors are single-pole closed forms.
  Diodes, LEDs, current regulators, FETs with their body diodes,
  transistors and optocouplers are piecewise-linear elements solved in one
  nodal solve. Regulators are rails with their soft-start. Oscillators,
  logic gates, supervisors, isolators, PSRAM, SPI NOR flash and SD cards
  are models, and every number in them cites its datasheet. There is no
  timestep: time enters only as instants armed on a stepped virtual clock.
- **The engine resolves every net.** It ranks the sources on a net by
  their strength, runs a solve only where comparable sources disagree,
  and reports what it finds: contention, floating inputs, rails down,
  domains with no reference, open drains no pull-up reaches, supply pins
  with no decoupling capacitor.
- **One interface between a part and the board.** A pin is declared once:
  its role, thresholds with hysteresis, reference and supply pins, clamps,
  input port, and whether it can source or sink. A part publishes one
  message, `Drive::{Thevenin, Current, Periodic}`, and is handed one,
  `Sense`: the node's voltage against the pin's reference, or none when
  nothing reaches it. Each receiver turns that voltage into a level itself.
- **Deterministic.** Instants are integer nanoseconds. In stepped mode a
  run's event log is identical across runs and processes, and golden
  traces hold it.

### Boards, models and the P2

- **`embsim-boards`** ships the Parallax P2-EC32MB, built from its vendor
  netlist, and the P2 package that any CPU core sits in. The package's
  START gate starts the core the datasheet's 3 ms after `RESN` releases
  with `VDD` in its window. Pads drive at their bank's supply with their
  `WRPIN` strength, and the crystal is the rate the board puts on `XI`.
- **`embsim-models`** adds the W25Q128JV SPI flash (a project can boot from
  a flash image with `image = "boot.bin"`), the SD card with a FAT16
  builder, the APS6404L PSRAM, and the regulators, gates, isolators,
  optocouplers, supervisor and oscillator the reference boards carry.
- **`embsim-p2-qemu`** runs the P2 on QEMU. It boots Parallax's own ROM
  off the module's flash over the nets, every edge at the guest's own
  nanosecond, and matches p2core state for state over 60 000 states.
- **`embsim-cpu-oracle`** reads and diffs ISS-against-silicon golden
  records.

### Projects and the `embsim` command

- **A project is a TOML file** that names the boards (a KiCad netlist, or
  a board a catalog ships), the model each part takes (`[[board.model]]`),
  the bench components, the wires (`[[wire]]`), the connectors that mate
  (`[[mate]]`), and the scenario. Each board is surveyed before it is
  built, and a kind is placed only on a part that is what the kind says.
  `requires-embsim = "0.2"` names the release a project is written for.
- **The `embsim` command** (`embsim-cli`):
  - `survey` lists a netlist's checklist;
  - `new` writes a starter project;
  - `check` builds the system with time held and says what is left;
  - `run` runs it in virtual time. `--for` sets how long, `--net` prints a
    net's state, `--pty` places a host serial port, and Ctrl-C ends the
    run with its summary. A part that fails — a P2 core whose program
    died — ends the run there, and the command exits non-zero;
  - `flash-image PROGRAM -o IMAGE` lays out a P2 program behind embsim's
    stage-1 loader as the boot flash the ROM boots, and a `w25q128jv` whose
    image a P2 would not boot says so at the run's first look;
  - `--version`, `check` and `run` say which release, git revision,
    compiler, target and profile the binary is.

  The command is also a library (`embsim_cli::run`, `main_with`,
  `shipped`, `runner_main`).
- **Bench components:** `host-serial` (a host's serial port as a PTY on
  the host's own rail) and `scripted-source` (a pin driven through timed
  steps).

### Extending embsim from a project

- **Catalog crates.** A project names Rust crates of its own in
  `[catalog] crates`. Each exports `register(&mut CatalogSet)`, which adds
  board kinds, part kinds, P2 core kinds and bench component kinds beside
  embsim's own. A name two catalogs both provide is refused where a
  project uses it, and the refusal names both catalogs.
- **The runner.** `embsim check` and `embsim run` build the project's
  crates and embsim into one binary, the runner, and hand the command line
  to it. Either the tool writes the runner beside the project (under
  `.embsim/`, with its lock kept as `embsim.lock` for you to commit), or the
  project owns one in its Cargo workspace (`[catalog] runner`), built
  `--locked`. The runner's embsim is the one the catalog crates depend on:
  a path, a git revision or a release. Cargo refuses two copies of embsim
  in one graph (`links = "embsim-core"`).
- `embsim new --catalog DIR` starts a catalog crate, `--own-runner` starts
  the runner crate beside it (with a `.gitignore` for its builds when it is
  a Cargo workspace of its own), and `--add-to PROJECT` names the crate in
  an existing project. The crate takes embsim from `--embsim PATH|URL@REF`
  when given; else from the tool's checkout when it sits in the project,
  or from embsim's repository at the tool's revision when a remote holds
  it, or, for a tool built from an unpushed commit or with uncommitted
  changes, from its checkout by path, saying why. `survey` and `new` take `--project FILE` to offer the
  project's own kinds. A command line the tool cannot parse still goes to
  the project's runner when the project names catalog crates, so an older
  `embsim` runs a newer project rather than refusing a flag it does not
  know.
- The structs a catalog fills in are `#[non_exhaustive]`, with
  constructors, so later releases can add fields without breaking catalog
  crates. Every sort of kind describes itself the same way (`KindInfo`).
- [`examples/custom-project`](examples/custom-project/README.md) is a
  complete worked example: a part model, a board, a P2 core and a bench
  instrument.

### QEMU as a separate install

- **QEMU is not linked into embsim.** Each P2 runs in a `qemu-system-p2` of
  its own. That program and embsim take turns in lockstep over a shared
  page, or over a socket pair with `EMBSIM_P2_QEMU_TRANSPORT=socket`. A
  board may carry several P2s, and the program exits when its node, or
  the process that started it, ends.
- `embsim qemu install` builds `qemu-system-p2` from QEMU v10.1.0 and the
  P2 target embsim carries, and installs it into
  `~/.embsim/qemu/<target identity>/` with QEMU's licence and the source
  it was built from. It needs git, a C compiler, ninja, pkg-config, glib
  and python3. `embsim qemu path` says which program a run would start
  and whether it fits. A project with `core = "qemu"` and no program
  installed is refused, and the error says how to install one.

### Releases and licences

- Each release attaches prebuilt `embsim` binaries for macOS (arm64 and
  x86_64) and Linux (x86_64, glibc 2.35 or newer), with SHA-256 checksums,
  and the P2 target's sources that `embsim qemu install` builds from. See
  the README's [Install](README.md#install).
- embsim's code is MIT in every crate. `embsim-boards` is
  `MIT AND CC-BY-SA-4.0`, because it compiles in a transcription of
  Parallax's CC BY-SA 4.0 P2-EC32MB schematic. `embsim-p2-qemu` is
  `MIT AND LGPL-2.1-or-later AND GPL-2.0-or-later`, because it embeds the
  P2 target's sources, which `embsim qemu install` builds from, in every
  binary. `qemu-system-p2` is a separate GPL-2.0 program that embsim runs
  and does not link.

### Fixed

- An actor parked on the virtual clock when the last time authority is
  released no longer sleeps forever.
- A serial PTY's path never deletes what it names:
  `embsim_core::serial_pty::Pty::new` replaces only a symlink and refuses
  a file or a directory, and drop removes the link only while it is still
  its own.

### Known issues

- Windows is not supported: the serial PTY and the QEMU channel use Unix
  APIs.
- No prebuilt `qemu-system-p2` yet: `embsim qemu install` builds it, so the
  P2 on QEMU needs a C toolchain.

## [0.1.0]

The first release: a software-in-the-loop framework that linked a
project's firmware C against Rust implementations of its HAL, with
emulated peripherals, device models and a virtual clock, extracted from
the MaD tensile tester.

[0.2.0]: https://github.com/RileyMcCarthy/embsim/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/RileyMcCarthy/embsim/releases/tag/v0.1.0
