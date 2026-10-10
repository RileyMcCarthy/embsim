# Changelog

What each release of embsim changes for someone using it. Releases are
git tags, `vX.Y.Z`; while embsim is 0.x, a minor release may break what
the one before it promised, and says so here under **Breaking**.

## [Unreleased]

### Added

- **`chrome-cdp`, the host's Chrome on the board** (`embsim-cdp`, in the
  `embsim` command's set). A bench component on `host-serial`'s four pins
  whose far end is Web Serial in every page of the host's Chrome, launched
  or attached to at the first slice. Every page, every dedicated worker and
  every worker those start is held from birth and its clock metered by the
  board's over the Chrome DevTools Protocol, a quantum (1 ms) at a time:
  timers, `Date.now()`, `performance.now()` and a WASM module's clock
  advance only with the board's, so a web app's timeouts hold in board
  time. Pages that share a renderer share one clock, granted once; a new
  document starts level with the board; a hidden page is metered too. The
  port follows Chrome's rules — a `close()` refused while a stream is
  locked, streams released a slice after their flush, a disconnect that
  errors both streams once the pipe is read, a replug that hands out a new
  `SerialPort`, a write paced by the line — and the cable can be pulled
  from the page (`__embsim.link('unplug')`) or from Rust. A drain barrier
  waits for a worker that owns the port's transferred stream to read what
  it was handed before the board's clock moves on. A stuck grant, a page
  that does not return (an unanswered dialog), a crash, a page Chrome made
  with a URL (which it does not hold) and a URL that does not open each stop
  the run saying which. Options: `baud`, `quantum`, `max_lead`,
  `stuck_after`, `chrome`, `devtools_port`, `headless`, `attach`, `url`,
  `usb_vendor_id`, `usb_product_id`, `granted` ([`PROJECTS.md`](PROJECTS.md)
  §5, [`NODES.md`](NODES.md) §19). `examples/chrome-ping` is a project with
  one, and a Playwright harness for it.
- `embsim_board::HostRailLine` and `HOST_RAIL_PINS`: a host's serial line at
  its own rail, the one `host-serial` attaches, for any bench host whose
  bytes come from elsewhere; `SerialLevelBridge::tx_idle`.
- `PartOptions::boolean`: a kind's `true`/`false` option, read as the
  project file wrote it.

### Changed

- **A project's committed `embsim.lock` is stale until refreshed.** Every
  runner now links `embsim-cdp`, and with it `tungstenite`'s dependency
  tree, so a runner's `--locked` build against a lock written by 0.3.0
  fails, and the tool says to remove the file and run again; the new
  `embsim.lock` it writes is the one to commit ([`PROJECTS.md`](PROJECTS.md)
  §10, "Its lock file is the project's"). `examples/custom-project`'s is
  refreshed.

## [0.3.0] - 2026-10-09

What a project like MaD needs from embsim to move onto a project file,
beyond 0.2.0: the Edge carrier's RS-422 parts in the standard catalog, an
ADS122U04 the firmware configures over its own pins, a plant of several
models as one component, and an encoder that presents RS-422 pairs.
[`MIGRATING-MAD.md`](MIGRATING-MAD.md) §2 lists what is still owed; the
host the board's clock meters is one, and `NODES.md` §18 records why it
will be the host's Chrome metered over DevTools, not a VM.

### Breaking

- **`ads122u04::Config` has no gain or reference.** The converter takes
  both from its registers (below), so `vref_mv` and `gain` are gone;
  `zero_offset` stays, and `Config::default()` replaces
  `Config::at_reset()`. `Ads122u04::sense(pin, volts)`, each analog pin
  against `AVSS`, replaces `set_voltage(mv)`; `INTERNAL_VREF_VOLTS`
  replaces `INTERNAL_VREF_MV`, and `RESET_GAIN` is gone (a gain is the
  register's).
- `machine::quadrature_encoder::Config` has a `complements` field (below):
  a struct literal names it, or ends `..Config::new(counts_per_mm)`.
- `Finding` has a new variant, `PinAboveRecommended` (below): an
  exhaustive `match` over it needs an arm for it.

### Added

- **`embsim_board::Assembly`: a plant as one component** (`NODES.md`
  §16). A bench component kind can return several components — embsim's
  models and a project's own — as one: `Assembly::member` renames each
  member's pins onto the assembly's, declarations kept;
  `Assembly::reference` measures pins against a return the assembly
  declares (a drive's `DRIVE_GND`, an encoder's `ENC_GND`); the members'
  wakes go through the assembly, each member woken at its own instants in
  the order added; and the links between members (a shaft turning an
  encoder, a carriage reaching a switch) are closures between their
  handles, run on the engine's thread inside the member that emits. The
  engine sees one node and records the same events as for the members
  apart. `PROJECTS.md` §10, "A plant: an `Assembly`", has the guide, with
  MaD's machine as the example.
- **`QuadratureEncoder` as an RS-422 encoder**
  (`quadrature_encoder::Config::with_complements`, `NODES.md` §17): `A-`,
  `B-` and, with an index, `Z-`, each driven to the inverse of its leg in
  the same publish, the pairs a differential receiver such as the Edge
  board's `U25` reads. `Config` has a `complements` field, `false` by
  default.
- **The Edge carrier's RS-422 pair in the standard catalog.** `am26ls31`,
  the AM26LS31 line driver (TI SLLS114N; the carrier's `U24`, placed by
  `AM26LS31CD`, `CDR`, `CDBR`, `CN` and `CNSR`), and `am26lv32`, the
  AM26LV32 line receiver (TI SLLS202H; the carrier's `U25`, placed by
  `AM26LV32IDR`, `IDRG4` and `INSR` and the obsolete `CD` and `ID`), each
  with its numbered pin table and its outputs driven from its own `VCC`.
  `boards/projects/edge-ec32-ds2.toml` builds with the standard catalog
  alone.
- **A pin's operating limits.** `PinDecl::with_limits(PinLimits {
  recommended, absolute_max, note })` declares a pin's range against its
  declared reference; the engine checks it against the solved net when the
  system is built and after every pass that moves the pin's net or its
  reference, and raises `Finding::PinAboveRecommended` once per excursion.
  The `am26lv32` declares `VCC` 3.0–3.6 V recommended, 6 V absolute.
- **Findings in plain words.** `Finding` implements `Display`: one line
  naming the net, the pins as `Reference.Pin` or the part, and what is
  wrong. `check` and `run` print it in place of the Rust form.

### Changed

- **The `ads122u04` kind converts as its register writes set it up**
  (`NODES.md` §15). Every conversion reads the register file the host
  writes over the part's own serial pins: the input multiplexer, the gain
  (the PGA's bypass included, which leaves every gain as it is and limits
  a pin read against `AVSS` to 4), and the reference — the internal
  2.048 V, `REFP − REFN`, or the analog supply as the part's pins sense it;
  the two system monitors convert at gain 1 against the internal reference
  whatever the gain and reference bits say (SBAS752B §8.3.9).
  A part held in reset or without both supplies goes back to its defaults,
  gain 1 against 2.048 V, as the RESET command already did; single-shot
  mode sends one conversion per START. MaD's firmware start-up, sent to the
  DS2 add-on's project, reads the force path's code at gain 128 against the
  3.3 V analog supply. The model's protocol thread ends when its part is
  dropped.
- `embsim check` names the pin table that fits a board's parts, as
  `survey` does: per group of parts placed with a table the netlist does
  not use, the `options.pins` table that declares the netlist's pins, or
  that no table of the model does (`fitting_option_table`,
  `BoardSurvey::pin_table_groups`, `PinTableGroup`).

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

[0.3.0]: https://github.com/RileyMcCarthy/embsim/compare/v0.2.0...v0.3.0
[0.2.0]: https://github.com/RileyMcCarthy/embsim/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/RileyMcCarthy/embsim/releases/tag/v0.1.0
