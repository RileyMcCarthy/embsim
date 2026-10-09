# Moving MaD's SIL onto an embsim project

MaD (`RileyMcCarthy/MaD`) is the tensile tester embsim was extracted from,
and the first project to extend it. Its SIL today is `mad-emulator`
(`SIL/MaDSim`), a Rust program that assembles the system by hand: the P2
instruction-set simulator as a component on no board, the DS2 add-on, a
PTY, bench pulls, the machine's parts coupled by callbacks, and an SD card
node. This document is the plan for replacing it with what every embsim
project has: a catalog crate of MaD's own kinds, a project file, and the
`embsim` command ([`PROJECTS.md`](PROJECTS.md) §10).

*Status, 2026-10-02, against embsim 0.2.0 (the runner MaD owns, and the
embsim it holds through the submodule, are 2026-10-02's; `NODES.md` §13,
"Packaging for the release"). A plan: nothing in MaD has changed. It was checked
against embsim's `feat/embsim-catalogs` and against MaD's working tree on
2026-10-01: branch `feat/iss-rom-serial-flash` at `a2b20efe2` with changes
no commit holds yet (`SIL/MaDSim/src/main.rs`, `iss_description.rs` and
`system_description.rs` modified; `wiring.rs`, `machine_ui.rs`,
`machine_view.rs`, `MaDSim/static/` and `p2iss/src/host_pty.rs` deleted).
Step 0 lands them.
MaD's code is cited by file and symbol, not by line, so each citation holds
in the commit that lands them. The project file was checked against the Edge
carrier's netlist by reading it, and its pieces embsim can already build
were built: the module with a flash image and a card image (`embsim check`,
for this plan) and the add-on (as `boards/projects/ds2-addon.toml` builds
it). The Edge carrier builds with the catalogs embsim ships since E1
(`boards/projects/edge-ec32-ds2.toml`; a netlist exported from MaD's
schematic before `U25`'s part number is corrected does not, section 2),
since E2 the add-on's converter is configured by the firmware's own
register writes, since E3 the machine's parts are one component, an
`embsim_board::Assembly`, and since E6 the encoder presents the pairs
`U25` reads; the host the board's clock meters, `chrome-cdp`, is owed
(E4). Revised 2026-10-09: E4's first answer, Chrome in a VM, was built and
withdrawn for Chrome on the host metered over DevTools (`NODES.md` §18),
and section 1's four files, steps 1b, 9, 10 and 11 and section 6 follow
that route; the whole file waits on MaD's own kinds. The Rust below is MaD's to write and is marked
`ignore`; each piece names the compiled file in embsim's worked example,
[`examples/custom-project`](examples/custom-project/README.md), that has
its shape. Why each choice was made is [`NODES.md`](NODES.md) §13.*

## 1. Where MaD ends up

| Today (`mad-emulator`) | After |
|---|---|
| `SIL/MaDSim`, 1 078 lines of assembly, flags, clock set-up, signals and telemetry (`main.rs`, `iss_description.rs` and `system_description.rs`, as step 0 lands them) | `SIL/mad-catalog`: four kinds and their tests; `SIL/mad-runner`: ten lines, the embsim command over them. The command, its checks and its report are embsim's |
| `mad-emulator` built against whatever `SIL/embsim` holds | the runner a member of MaD's SIL workspace, built `--locked` against the committed `SIL/Cargo.lock`, embsim a path dependency through the `SIL/embsim` submodule: the pinned commit is the embsim every machine builds |
| `P2Iss` on no board, declaring the pins its lists name | `p2iss`'s core in the P2-EC32MB's package (`core = "mad-p2iss"`), all 64 pads on the module's nets |
| the firmware image put straight into hub RAM | the ROM booting stage-1 and the firmware off the module's flash, as the chip does |
| the DS2 built from `MaDSim/boards/ds2_addon.net`, its converter pre-configured | the `mad-ds2` board, its converter configured by the firmware's own register writes |
| the Edge carrier not modelled; harness wires to bare `P2.Pnn` | the `mad-edge` board in its socket, every wire on a connector of the carrier or the add-on |
| bench pulls on every input (`BenchPulls`, `IDLE_PULLS`) | nothing: the carrier's isolators, optocouplers and resistors set those lines |
| `BenchSd` on four bare pins with a bench pull-up | embsim's `sd-card` in the module's own socket `J301` |
| the stepper, encoder, switches, gantry, sample and load cell, coupled by callbacks | the `mad-machine` bench component, one plant with electrical pins |
| `make playground`, `e2e-emulator`, `playground-iss`, `playground-rom`, and the nightly | `embsim run mad-cosim.toml …` and `embsim run mad-serial-boot-cosim.toml …`: the app in the host's Chrome, its clock metered by the board's over DevTools |

Four project files, two machines each with two hosts. The user's rule for
MaD's SIL (`NODES.md` §18) is that the firmware on the ISS runs with the
control app in the host's Chrome, its page and workers held to the board's
clock over the DevTools protocol by a `chrome-cdp` (E4), and with no host
that keeps wall time: every run of the app is a `-cosim` file. The stepped
test sends the host's bytes itself, so the files it and `make check` load
keep a `host-serial`, which needs no browser:

- `SIL/mad.toml`: the machine as it runs, booting the firmware off the
  module's flash, its host a `host-serial` on the Pi's connector. `make
  check` and `mad-catalog/tests/mad.rs` load it (step 9).
- `SIL/mad-cosim.toml`: the same file with `HOST` a `chrome-cdp` on the same
  four pins and wires (section 4 shows the entry). The e2e suite and the
  playgrounds run it (steps 10 and 11).
- `SIL/mad-serial-boot.toml`: the same boards with the module's option
  switch set for a serial boot, an erased flash, and the host on the
  carrier's debug header (`J1`: `P62`, `P63`, `RESn`, `GND`), checked by
  `make check`.
- `SIL/mad-serial-boot-cosim.toml`: that file with the `chrome-cdp` host.
  The browser flashes the firmware through it (`make playground-rom`
  today).

Each `-cosim` file differs from its base in the `HOST` entry alone.

## 2. What embsim owes first

Each item is generic, so it belongs in embsim, not in a MaD crate: a
project crate does not fork a model.

| | What | Why MaD needs it | MaD's step that waits |
|---|---|---|---|
| E1 | *shipped:* `am26lv32`, the AM26LV32 line receiver's kind, placing `U25` by its part number `AM26LV32IDR`, beside the AM26LS31 line driver's `am26ls31`. MaD's schematic owes the matching fix: `U25`'s `Manufacturer_Part_Number` and value set to `AM26LV32IDR` in `Hardware/EdgeBoard/KiCad` | the Edge carrier's encoder pairs (`U25`, `J20`), beside the servo step and direction pairs (`U24`, `J21`); until the schematic fix, a netlist exported from MaD names `AM26LS32CD` and `check` refuses `EDGE` naming `U25` | 9 |
| E2 | the `ads122u04` model applying `GAIN` and `VREF` from the firmware's register writes, `VREF = AVDD` read as the sensed `AVDD − AVSS`. *Built 2026-10-08* (`NODES.md` §15): every conversion reads the register file — the multiplexer, the gain (the PGA's bypass included), the reference — and a held reset or a lost supply returns it to its defaults; MaD's start-up bytes, sent to the add-on as `boards/projects/ds2-addon.toml` builds it, give the force path the code the firmware expects (`board/tests/ads122u04_registers.rs`) | `mad-emulator` registers the converter pre-configured (gain 128, `VREF` the 3.3 V excitation); the kind starts as the chip leaves reset, so without this the force path reads about 79 times low | 9 |
| E3 | `embsim_board::Assembly`: one component hosting several of embsim's models (`PROJECTS.md` §10, "Adding a bench component"). *Built 2026-10-08* (`NODES.md` §16): members' pins renamed onto the assembly's, returns it declares (`DRIVE_GND`, `ENC_GND`), members' wakes through it in the order added, the links between them code on the engine's time; `PROJECTS.md` §10, "MaD's plant", is the machine as one | the machine is embsim's `StepperMotor`, `QuadratureEncoder` and `EndSwitch` models and MaD's gantry, sample and strain gauge on one carriage | 8 |
| E4 | *owed:* `chrome-cdp`, the host the board's clock meters (`NODES.md` §18): a bench kind on `host-serial`'s four pins, `TX`, `RX`, `VIO` and `GND`, driven and read the same way, so a project swaps one for the other without touching a wire. It launches the host's Chrome and holds every page and dedicated worker to the board's time over one DevTools connection, which carries the line's bytes too: each quantum (1 ms) the board's time is granted with `Emulation.setVirtualTimePolicy`, one evaluation hands the page the board's bytes and reads its clock, and the page's bytes come back on a binding, so no second channel races the grants. The page's `navigator.serial` is a shim the node owns, the host's half of the line (the port, its USB ids, `connect` and `disconnect`, an unplug and a replug), so MaD's e2e installs no serial fake of its own. The first answer, the `qemu-vm` and `chrome-vm` kinds, was built and withdrawn (§18) | the user's rule (§18): the ISS runs with the app in a Chrome whose clock the board's meters, and with no host that keeps wall time. Unmetered, the app's 2000 ms response timeout fired at 200 ms of the board's time; metered, at 2000 | 1b, 10, 11 |
| E5 | *not blocking:* a P2 flash-layout option on `w25q128jv` (a program laid out behind stage-1 when the board is built) and a `dir` option on `sd-card` (a FAT16 card holding a directory) | until then MaD writes both images with two make targets (step 4) | none |
| E6 | complementary outputs on `embsim_models::machine::QuadratureEncoder`. *Built 2026-10-09* (`NODES.md` §17): `Config::with_complements()` declares `A-`, `B-` and, with an index, `Z-`, each driven to the inverse of its leg in the same publish; `board/tests/edge_encoder_pairs.rs` counts the quadrature at `P9`/`P10` through `U25` with `JP2`, `JP3` and `JP5` open | the carrier's `J20` takes an RS-422 encoder and `U25` reads differences: a single-ended encoder on `A+`/`B+` with `JP2`/`JP3` grounding `A−`/`B−` gives no differential for its low, which `U25` reads as its fail-safe high (SLLS202H §8.4.1), so `P9` and `P10` never move (section 6, "The encoder through `U25`") | 8 (the machine's pins), 9 (the encoder case) |
| E7 | *not blocking:* a control surface for `run`, such as `--control <port>` serving `embsim-ui`'s actions, a host kind registering `<NAME>/link/unplug` and `<NAME>/link/plug` | three of the e2e's scenarios (B5's reconnect, M11's idle drop and its mid-test drop) pull the host's cable. A `chrome-cdp` takes that from the page, on its own binding, so they wait on nothing; a project whose host is a `host-serial` has no way to pull it | none |

## 3. The catalog crate, `SIL/mad-catalog`

A member of MaD's SIL workspace. `embsim new --catalog mad-catalog
--own-runner`, run in `SIL/` with the tool built from the submodule (step
5), starts it with the name `mad-catalog` and every example kind named
`mad-…`, and starts the runner beside it (below). Its embsim dependencies
are paths into `SIL/embsim`: the tool's own checkout sits inside MaD's
repository, so `new` writes paths, not a git source (`PROJECTS.md` §10,
"The catalog crate").

```text
SIL/mad-catalog/
  Cargo.toml
  netlists/
    mad_edge.net        kicad-cli export of Hardware/EdgeBoard/KiCad/MaD_Edge.kicad_sch
    ds2_addon.net       moved from SIL/MaDSim/boards/
  src/
    lib.rs              register: the board and component kinds, the core
    boards.rs           mad-edge, mad-ds2
    iss.rs              mad-p2iss and its report
    machine.rs          mad-machine and its report
  tests/
    boards.rs           each board kind surveyed and built
    boot.rs             the firmware booting off the module's flash (skips without the image)
    machine.rs          the plant driven from its pins
    mad.rs              mad.toml, stepped (review item 1 of NODES.md §13)
```

### `Cargo.toml`

```toml
[package]
name = "mad-catalog"
version.workspace = true
edition.workspace = true
publish = false

[dependencies]
# One copy of embsim: the pinned submodule, by path. The runner takes
# embsim-cli from the same checkout, and SIL/Cargo.lock locks the rest.
embsim-board = { path = "../embsim/board" }
embsim-boards = { path = "../embsim/boards" }
embsim-core = { path = "../embsim/core" }
embsim-models = { path = "../embsim/models" }
models = { path = "../models" }
p2core = { path = "../p2core" }
p2iss = { path = "../p2iss" }

[dev-dependencies]
# The tests run MaD's projects as the runner does, in process.
embsim-cli = { path = "../embsim/cli" }
rstest.workspace = true
vibes-behaviour = { path = "../../Vibes/bindings/rust" }
```

### The runner, `SIL/mad-runner`

MaD owns its runner: a binary crate in the SIL workspace, which `embsim
new --catalog mad-catalog --own-runner` writes and names in the project's
`[catalog] runner` (`PROJECTS.md` §10, "A runner the project owns"). The
`embsim` tool builds it with Cargo against `SIL/Cargo.lock`, `--locked`, in
`SIL/target`, and runs the project through it; `cargo run -p mad-runner --
check mad.toml` is the same without the tool.

```toml
[package]
name = "mad-runner"
version = "0.1.0"
edition = "2021"
publish = false

[[bin]]
name = "mad-runner"
path = "src/main.rs"

[dependencies]
# The same embsim as the catalog crate's: one copy of embsim in the runner.
embsim-cli = { path = "../embsim/cli" }
mad-catalog = { path = "../mad-catalog" }
```

```rust,ignore
use std::process::ExitCode;

fn main() -> ExitCode {
    embsim_cli::runner_main(&[embsim_cli::CatalogCrate::new(
        "mad-catalog",
        concat!(env!("CARGO_MANIFEST_DIR"), "/../mad-catalog"),
        mad_catalog::register,
    )])
}
```

### `src/lib.rs`

The shape of the example's `lib.rs`
(`examples/custom-project/catalog/src/lib.rs`):

```rust,ignore
pub mod boards;
pub mod iss;
pub mod machine;

/// The catalog's name: the crate's.
pub const NAME: &str = "mad-catalog";

/// Add MaD's kinds to `set`: the two boards and the machine as one catalog,
/// the ISS as a core catalog. Starts nothing.
pub fn register(set: &mut CatalogSet) -> Result<(), ProjectError> {
    set.add(boards::MadCatalog)?;
    set.add_cores(iss::IssCores)?;
    Ok(())
}
```

### `src/boards.rs`: `mad-edge` and `mad-ds2`

The shape of the example's `board.rs` and of `PROJECTS.md` §10's board doc
test. Each board kind bundles its netlist with `include_str!` and starts
from the base registrations (`CatalogBoard::from_base`), with the entries
every project of that board needs:

| Kind | Netlist | Its own entries |
|---|---|---|
| `mad-edge` | `netlists/mad_edge.net`, exported with `kicad-cli sch export netlist` from `Hardware/EdgeBoard/KiCad/MaD_Edge.kicad_sch` (three sheets), with the provenance header embsim's fixture `board/tests/fixtures/mad_edge.net` has | `ModelSpec::by_part("P2_EDGE_MODULE_SOCKET", "boundary")`: `J3`, the module socket, a symbol from the carrier's own library |
| `mad-ds2` | `netlists/ds2_addon.net`, today's file moved, its provenance header kept | `ModelSpec::by_part("ADS122U04", "ads122u04")` |

```rust,ignore
fn board(&self, spec: &BoardSpec) -> Result<CatalogBoard, ProjectError> {
    let parse = |text| {
        netlist::parse(text)
            .map_err(|err| ProjectError::message(format!("board {}: {err}", spec.name)))
    };
    match spec.kind.as_str() {
        EDGE => Ok(CatalogBoard::from_base(parse(MAD_EDGE)?)
            .with_model(ModelSpec::by_part("P2_EDGE_MODULE_SOCKET", "boundary"))),
        DS2 => Ok(CatalogBoard::from_base(parse(DS2_ADDON)?)
            .with_model(ModelSpec::by_part("ADS122U04", "ads122u04"))),
        other => Err(ProjectError::message(format!("{other} is not a board kind of {NAME}"))),
    }
}
```

The scenario lines stay in the project files: the DS2's `JP1`/`JP2` and its
`~RESET` strap are how a bench is set up, not what the board is.

### `src/iss.rs`: `mad-p2iss`

The shape of the example's `blinker.rs` and of QEMU's core catalog
(`p2-qemu/src/catalog.rs`). One core kind, `mad-p2iss`, with one option:

| Option | What it says |
|---|---|
| `rom` | the boot ROM, a file relative to the project; by default `p2iss/rom/rom_booter_v33k.bin`, bundled with `include_bytes!` |

The core always boots the ROM off whatever the board gives it, as `qemu`
does: on the P2-EC32MB, the module's flash and `S301`'s straps. There is no
option that puts the program straight into hub RAM; a fast load skips the
boot chain a board simulation exists to check, and adding one is a
`NODES.md` decision first (§13, decision (m)).

```rust,ignore
fn seat(
    &self,
    _core: &str,
    assignment: &Assignment<'_>,
    mut options: PartOptions,
) -> Result<CoreCtor, ProjectError> {
    let rom = options.string("rom")?;
    options.finish()?;
    let rom = match rom {
        None => BOOT_ROM.to_vec(),
        Some(file) => {
            let path = assignment.dir.join(&file);
            std::fs::read(&path).map_err(|err| {
                assignment.error(format!("cannot read boot ROM {}: {err}", path.display()))
            })?
        }
    };
    let reports = assignment.reports.clone();
    let board = assignment.board.to_string();
    // Called once per part when the board is built; a survey never calls it.
    Ok(Box::new(move |decl| {
        let core = p2iss::P2IssCore::with_boot_rom(&rom);
        reports.add(IssReport::new(format!("{board}.{}", decl.reference), core.handle()));
        Ok(Box::new(core) as Box<dyn P2Core>)
    }))
}
```

`P2IssCore` is step 2's type. Its report (`IssReport`, an
`embsim_board::Report`) says what `mad-emulator`'s telemetry thread says
(the thread `run_iss` spawns, `MaDSim/src/main.rs`), the parts that do not
depend on the wall clock:
the guest's debug console (`P62`) line by line as a look finds it, and at
the end the running cogs, the rate the guest programmed on each async smart
pin it transmitted on, the framing errors and the dropped edges. The rate
against real time is not a report line: two runs of a stepped project print
the same report, and `run` prints the wall time already.

### `src/machine.rs`: `mad-machine`

One bench component: the machine's mechanism inside, electrical pins
outside (`PROJECTS.md` §10, "What a project can add"): an
`embsim_board::Assembly` (E3) of embsim's `StepperMotor`, `QuadratureEncoder`
and two `EndSwitch`es and MaD's own load cell, the gantry, the sample and
the strain gauge the link from the shaft to the load cell (`PROJECTS.md`
§10, "MaD's plant", sketches the constructor). `DRIVE_GND` and `ENC_GND`
are returns the assembly declares (`Assembly::reference`): neither model
has a return pin of its own. A switch loop is a contact and the loop
supply; embsim's `EndSwitch` is the contact, `COM` to `NO`, `NO` the
loop's `+`. Where the supply comes from (section 6, "The loop supply")
decides the rest: a supply of the machine's own makes `COM` a pin the file
powers at `loop_volts`, with `-` its return, or a loop member of MaD's own
with the supply and the contact inside it; the carrier's `5V_IO` makes
`COM` a pin wired to it.

| Pins | What they are |
|---|---|
| `STEP`, `DIR`, `ENA` | the servo drive's inputs (`embsim_models::machine::StepperMotor`, `DIR` low forward, enable active low), read against `DRIVE_GND` |
| `DRIVE_GND` | the drive's input return, the carrier's `EN_GND` (`J21.8` and `J21.9`) |
| `ENC_A+`, `ENC_A-`, `ENC_B+`, `ENC_B-`, `ENC_Z+`, `ENC_Z-` | the encoder's pairs (`QuadratureEncoder` with its complements, E6, as many counts per millimetre as steps; each `-` the inverse of its `+`), against `ENC_GND`: what an RS-422 encoder presents on `J20`, and what `U25` reads |
| `ENC_GND` | the encoder's return, `EN_GND` (`J20.5`) |
| `UPPER±`, `LOWER±`, `DOOR±`, `ESD_U±`, `ESD_L±`, `ESD_A±` | each switch as the sourced loop the carrier reads: `+` the machine's loop supply behind a normally closed contact while the contact is closed, released while it is open; `-` that supply's return. The carrier's loops are a current regulator and an opto LED between `+` and `-` (`IC9` and `U6` for the upper end switch), and power nothing themselves |
| `E+`, `E-` | the load cell's excitation, sensed: the bridge reads its excitation off the add-on |
| `S+`, `S-` | the bridge's outputs, each behind the cell's 350 Ω, at the excitation's midpoint ± half the strain gauge's output |

| Option | What it says |
|---|---|
| `sample` | the sample in the grips, one the crate carries with its provenance: `sil-linear-reference` (the `MaterialProperties` named `SIL-Linear-Reference` in `run_iss`, `MaDSim/src/main.rs`) |
| `loop_volts` | required: the machine's switch-loop supply. A bench figure, so the file names it (`DESIGN.md` rule 6) |

What moves into it, and from where:

| From | What |
|---|---|
| `MaDSim/src/iss_description.rs`, `STEPS_PER_MM` to `impl BenchMachine` | `STEPS_PER_MM` (8192), `TRAVEL_MM` (100, from the firmware's failsafe profile), `BenchMachine`'s drive conventions (`DIR` high reverse, enable active low, no load loss), the encoder, the two end switches, and `CarriageTravel`, which becomes the machine's report |
| `MaDSim/src/main.rs`, in `run_iss`: `gantry_model`, `strain`, `sample` and what chains them | the gantry (15 mm of slack), the sample, the strain gauge (100 N full scale, −4.868009 mV/V) and the callbacks that chain them, now inside one component |
| `MaDSim/src/system_description.rs`: `BRIDGE_EXCITATION_V`, `BridgeDrive`, `LoadCellBridge` | `LoadCellBridge` and `BridgeDrive`: `S±` behind 350 Ω, centred on the excitation sensed on `E±` instead of the constant `BRIDGE_EXCITATION_V` |

Every figure keeps the citation it has today. The links between the parts
(the shaft turning the encoder and opening the switches, the carriage
straining the sample) stay Rust inside the component; the engine sees one
node with pins.

## 4. The project files

`SIL/mad.toml`, as it would read. Every wire lands on a connector; the
harness was checked against `mad_edge.net` (the nets of `J4`, `J9`–`J16`,
`J20`, `J21` and `JP1`–`JP5` read from it).

```toml
# The MaD tensile tester: the Edge carrier, the P2-EC32MB module in its
# socket running the firmware on MaD's instruction-set simulator, the DS2
# force-gauge add-on on the force cable, the machine on the carrier's
# connectors, and the Raspberry Pi's serial port as a PTY.

requires-embsim = "0.3"

[catalog]
crates = ["mad-catalog"]
runner = "mad-runner"

[[board]]
name = "EDGE"
kind = "mad-edge"

[[board]]
name = "EC32"
kind = "p2-ec32mb"

[[board.model]]
value = "P2X8C4M64P"
kind = "p2"
[board.model.options]
core = "mad-p2iss"

# Stage-1 and the propeller2_debug program, laid out by `make flash-image`:
# the ROM boots the firmware off the module's flash.
[[board.model]]
value = "SPI Flash 16MB (128Mb)"
kind = "w25q128jv"
[board.model.options]
pins = "by-function"
image = "build/flash.bin"

# The card in the module's socket J301, a FAT16 image of ./sd written by
# `make sd-image`.
[[board.model]]
value = "MicroSD Socket"
kind = "sd-card"
[board.model.options]
pins = "by-function"
image = "build/sd.img"

[[board]]
name = "DS2"
kind = "mad-ds2"

[[component]]
name = "MACHINE"
kind = "mad-machine"
[component.options]
sample = "sil-linear-reference"
loop_volts = 24.0

[[component]]
name = "HOST"
kind = "host-serial"
[component.options]
baud = 2000000

# ---- The module in its socket, and the force cable ------------------------

[[mate]]
a = "EC32.J203"
b = "EDGE.J3"

[[mate]]
a = "EDGE.J9"
b = "DS2.J1"
map = [["1", "1"], ["5", "2"], ["4", "3"], ["2", "4"], ["3", "5"]]

# ---- Supplies: the bench's, and the Pi's side of the isolator IC2 ----------

[[wire]]
from = "BENCH.12V"
to = "EDGE.J2.1"
volts = 12.0

[[wire]]
from = "BENCH.GND"
to = "EDGE.J2.2"
volts = 0.0

[[wire]]
from = "BENCH.SERVO5V"
to = "EDGE.J21.1"
volts = 5.0

[[wire]]
from = "BENCH.SERVOGND"
to = "EDGE.J21.8"
volts = 0.0

[[wire]]
from = "BENCH.IFGGND"
to = "DS2.J1.2"
volts = 0.0

[[wire]]
from = "BENCH.VDDA"
to = "DS2.J2.1"
volts = 3.3

[[wire]]
from = "BENCH.AGND"
to = "DS2.J2.2"
volts = 0.0

[[wire]]                 # GND_IO, the isolated I/O domain's return
from = "BENCH.IOGND"
to = "EDGE.J10.2"
volts = 0.0

[[wire]]                 # RPI_5V: IC2's Pi-side supply
from = "PI.5V"
to = "EDGE.J4.1"
volts = 5.0

[[wire]]                 # RPI_GND
from = "PI.GND"
to = "EDGE.J4.6"
volts = 0.0

# ---- The host on the Pi's connector ----------------------------------------

[[wire]]                 # the Pi's I/O rail: its GPIO are 3.3 V
from = "PI.3V3"
to = "HOST.VIO"
volts = 3.3

[[wire]]
from = "PI.GND"
to = "HOST.GND"

[[wire]]                 # RPI_RX: IC2's input INC, what the Pi sends
from = "HOST.TX"
to = "EDGE.J4.3"

[[wire]]                 # RPI_TX: IC2's output OUTA, what the Pi receives
from = "EDGE.J4.2"
to = "HOST.RX"

# ---- The machine on the carrier's connectors -------------------------------

[[wire]]                 # SC_PUL+, from the line driver U24
from = "EDGE.J21.2"
to = "MACHINE.STEP"

[[wire]]                 # SC_DIR+
from = "EDGE.J21.5"
to = "MACHINE.DIR"

[[wire]]                 # SC_ENA, from JP1's common pad
from = "EDGE.J21.7"
to = "MACHINE.ENA"

[[wire]]                 # EN_GND, the drive's input return
from = "EDGE.J21.9"
to = "MACHINE.DRIVE_GND"

[[wire]]                 # A+, into the line receiver U25
from = "MACHINE.ENC_A+"
to = "EDGE.J20.1"

[[wire]]                 # A-
from = "MACHINE.ENC_A-"
to = "EDGE.J20.2"

[[wire]]                 # B+
from = "MACHINE.ENC_B+"
to = "EDGE.J20.3"

[[wire]]                 # B-
from = "MACHINE.ENC_B-"
to = "EDGE.J20.4"

[[wire]]                 # ZI+, U25's third channel (J20.7 and .8 are its enables)
from = "MACHINE.ENC_Z+"
to = "EDGE.J20.9"

[[wire]]                 # ZI-
from = "MACHINE.ENC_Z-"
to = "EDGE.J20.10"

[[wire]]                 # EN_GND, the encoder's return
from = "MACHINE.ENC_GND"
to = "EDGE.J20.5"

# The switch loops follow the nets: IEND_U is on J16, IEND_L on J15 and
# IDOOR on J14, whatever the silkscreen says. Pin 2 of each is the loop's +
# (the current regulator's anode), pin 1 its - (the opto LED's cathode).
[[wire]]
from = "MACHINE.UPPER+"
to = "EDGE.J16.2"

[[wire]]
from = "EDGE.J16.1"
to = "MACHINE.UPPER-"

[[wire]]
from = "MACHINE.LOWER+"
to = "EDGE.J15.2"

[[wire]]
from = "EDGE.J15.1"
to = "MACHINE.LOWER-"

[[wire]]
from = "MACHINE.DOOR+"
to = "EDGE.J14.2"

[[wire]]
from = "EDGE.J14.1"
to = "MACHINE.DOOR-"

[[wire]]                 # IESD_U, J11
from = "MACHINE.ESD_U+"
to = "EDGE.J11.2"

[[wire]]
from = "EDGE.J11.1"
to = "MACHINE.ESD_U-"

[[wire]]                 # IESD_L, J12
from = "MACHINE.ESD_L+"
to = "EDGE.J12.2"

[[wire]]
from = "EDGE.J12.1"
to = "MACHINE.ESD_L-"

[[wire]]                 # IESD_A, J13
from = "MACHINE.ESD_A+"
to = "EDGE.J13.2"

[[wire]]
from = "EDGE.J13.1"
to = "MACHINE.ESD_A-"

[[wire]]                 # the bridge's excitation: VDDA and its return
from = "MACHINE.E+"
to = "DS2.J2.1"

[[wire]]
from = "MACHINE.E-"
to = "DS2.J2.2"

[[wire]]                 # A0
from = "MACHINE.S+"
to = "DS2.J2.3"

[[wire]]                 # A1
from = "MACHINE.S-"
to = "DS2.J2.4"

# ---- Scenario ---------------------------------------------------------------

[[switch]]               # FLASH: P61 is the flash's chip select
part = "EC32.S301"
pole = 1
state = "closed"

[[switch]]               # R303 holds P59 down: boot the program in flash
part = "EC32.S301"
pole = 3
state = "closed"

[[switch]]               # JP1, TTL-SINK: SC_ENA from IC14's TTL output
part = "EDGE.JP1"        # (pads 1 and 2)
pole = 0
state = "closed"

# JP2, JP3 and JP5 (A_GND, B_GND, ZI_GND) stay open: they ground A-, B-
# and ZI- for a single-ended encoder, and the machine's encoder drives
# both legs of each pair (E6).

[[jumper]]               # Z_GND: Z-, U25's active-low enable, to EN_GND
part = "EDGE.JP4"
state = "closed"

[[jumper]]               # A0 and A1 to the converter (R6 and R7 are DNP)
part = "DS2.JP1"
state = "closed"

[[jumper]]
part = "DS2.JP2"
state = "closed"

[[pin_short]]            # the bench's ~RESET strap: the stock board leaves
a = "DS2.U1.3"           # U1's reset on a net of its own
b = "DS2.U1.13"
```

`SIL/mad-serial-boot.toml` is the same file with three differences:

- `S301` pole 2 closed (the `P59` pull-up `R302`, the serial strap
  `mad-emulator`'s `STRAP` stands in for today: the `Pull` that
  `run_iss_rom` wires to `P2.P59`, `MaDSim/src/main.rs`)
  and pole 3 open;
- the `w25q128jv` entry without `image`: the module's flash erased;
- the host on the carrier's debug header instead of the Pi's connector:
  `HOST.TX` to `EDGE.J1.2` (`P63`, the chip's receive pin), `EDGE.J1.1`
  (`P62`) to `HOST.RX`, `HOST.GND` to `EDGE.J1.4`, and `HOST.VIO` from a
  `[[wire]]` at the adapter's 3.3 V.

`SIL/mad-cosim.toml` and `SIL/mad-serial-boot-cosim.toml` are their bases
with the `HOST` entry a `chrome-cdp` (E4) on the same four pins and the same
wires, the DevTools port fixed so the e2e's `CDP_URL` holds, and the port's
USB ids the adapter's, so the app finds it again by them after a replug as
it does on hardware:

```toml
[[component]]
name = "HOST"
kind = "chrome-cdp"
[component.options]
baud = 2000000
devtools_port = 9222
usb_vendor_id = 0x0403
usb_product_id = 0x6001
```

The option names are the design's (`NODES.md` §18) until E4 ships them. The
node launches Chrome at its first slice, the board held there, and prints
the DevTools URL; the e2e attaches to it with Playwright's
`connectOverCDP`.

## 5. The ordered changes

Each step says what changes, the files it touches, and what shows it is
done. Steps 0 to 7 need nothing more from embsim than `feat/embsim-catalogs`;
the rest wait on section 2's items and section 6's questions, as marked.

**0. Land MaD's working tree.** This plan was read off changes on
`feat/iss-rom-serial-flash` that no commit holds (the status note above).
Commit them, or merge the branch, before step 1, and record that commit
here. *Files:* the ones the status note lists. *Done when:* MaD's CI is
green on the commit, and each symbol this plan cites is in it.

**1. Pin the submodule.** Bump `SIL/embsim` to embsim 0.2.0, the release
with `feat/embsim-catalogs` merged (MaD's CI gates embsim pin bumps).
*Files:* the `SIL/embsim` gitlink, `SIL/Cargo.lock` (embsim's crates at
0.2.0). *Done when:* MaD's CI is green on the bump, with `mad-emulator`
unchanged.

**1b. The metered host on `mad-emulator`.** Once E4 ships, the pin bump
that brings it puts the e2e, the nightly and a playground in the one valid
configuration, long before steps 2–9 move the machine onto a project.
`mad-emulator` places the `chrome-cdp` node where its host is today, on
the harness's host pins (`HOST.TX` to `P2.P53`, `P2.P55` to `HOST.RX`), with
the two wires a host's rail needs: `.power(ep("BENCH.HOST3V3"),
ep("HOST.VIO"), 3.3)`, the P2's I/O rail, and `.power(ep("BENCH.HOSTGND"),
ep("HOST.GND"), 0.0)`. Without them the line is unpowered and carries
nothing, as a `host-serial`'s does. The e2e attaches to the Chrome the node
launched, a fresh context per scenario, and changes with it:

- `installFakeSerial` goes: the node's shim is the page's
  `navigator.serial`. `installOpfsDataDir` stays.
- `dropLink` asks the shim to unplug and replug (`__embsim.link` in the
  design), which the node does at its next slice.
- Waits count the board's time: each `waitForTimeout` becomes a helper
  that polls the page's `performance.now()`, and `T()` takes its scale
  from the board's measured speed, not a fixed `E2E_TIMEOUT_SCALE`.
- Clicks: animation frames do not run on virtual time, so a plain
  `click()` waits forever for a stable element. A `press()` helper checks
  that the element is visible and enabled and is what `elementFromPoint`
  finds at its centre, then forces the click.
- `APP_URL` serves the production bundle from the host, not the dev
  server. Only the production build compiles the worker's
  `this.sink?.(events)` (`DeviceSession.worker.ts`) into `n.call(this, e)`,
  which on a Comlink proxy asks to clone the session object and throws a
  `DataCloneError` nothing reports, so no device event reaches the UI; the
  dev server hides it.
- Two scenarios join: Disconnect then Connect, and flashing while
  connected. Chrome refuses `close()` while a stream of the port is still
  locked and keeps the port open until the stream lets go; the shim must
  do the same, which today's fake does not.

*Files:* `SIL/MaDSim/Cargo.toml` (the crate E4 ships), `SIL/MaDSim/src/main.rs`
(the node on the host pins, the two power wires, the DevTools port
printed), `Software/Control/e2e/fixtures.mjs` and `run-all.mjs`,
`SIL/makefile`, `.github/workflows/e2e-nightly.yml` (the nightly on this
route). *Done when:* the e2e passes on the bump, the three link-drop
scenarios (B5, M11's two) and the two new ones among them. Steps 10 and 11
retire this route.

**2. `p2iss` gets a core.** A `P2IssCore` beside today's `P2Iss`, sharing
its machine, implementing `embsim_boards::p2::P2Core`:

- `attach(P2Pads)` keeps all 64 pad handles and subscribes to every pad's
  sense (`P2Pads::on_pad_sense`), as QEMU's core does;
- a pad drives through `P2Pads::bank_supplies().pad_drive(pin, wrpin, dir,
  out)`, which reads the pad's mode word: the role a pin plays (a level, an
  async smart pin and its rate, a step train) comes from the guest, so the
  pin lists (`SerialLink`, and `P2Iss::with_level_pins`, `with_input_pins`
  and `with_pulse_pins`, in `p2iss/src/lib.rs`) have no part in it;
- `start()` anchors the guest's clock at the package's START instant, 3 ms
  after the reset releases, on the engine thread; any MaD assertion on a
  boot instant moves by that delay;
- `reset()` stops the guest, as `P2Core::reset` says;
- the guest's `HUBSET` reports its clock word (`P2Pads::set_clock_mode`),
  and the crystal comes from `XI` (`P2Pads::on_crystal`);
- the synchronous-serial smart pins run on the nets (today's
  `P2Iss::with_sync_serial` path), so the module's flash
  and card share `P58`–`P61` as they do on the board;
- `Board::sensed` (`p2core/src/board.rs`) takes the strong-mask rule, and
  the `P59` fiat (`set_input_level(59, true)` in `P2Iss::with_boot_rom`,
  `p2iss/src/lib.rs`) has no place in the core: the
  board's `S301` straps the pin.

`P2Iss` keeps its `Component` impl and its pin lists until `mad-emulator`
retires (step 11), so nothing that runs today stops. *Files:*
`p2iss/Cargo.toml` (adds `embsim-boards`), `p2iss/src/lib.rs`,
`p2core/src/board.rs`. *Done when:* a new stepped test,
`p2iss/tests/ec32mb_flash_boot.rs`, boots the ROM, stage-1 and the
firmware off the P2-EC32MB's own flash inside the package, the module
powered from its fingers, and reads the firmware's first console line on
`P62` (the chain `p2iss/tests/rom_boot_net.rs` proves on bare nets).

**3. Measure.** The two costs `NODES.md` §13 review item 10 left open: the
firmware's boot off the flash, and every pad on a net. *Files:*
`p2iss/examples/iss_speed.rs`. *Done when:* both numbers, with the command
that made them, are recorded in `NODES.md` §13. A fast load is a decision
recorded there first, if the numbers call for one.

**4. The images.** Two make targets write the files the project names, in
`SIL/build/`: `flash-image` (stage-1 and the `propeller2_debug` program:
`$(EMBSIM) flash-image <program> -o build/flash.bin`, embsim's own command
since 0.2.0, which lays out the same stage-1 `p2iss::flashimage::boot_flash`
does) and `sd-image` (a 32 MiB FAT16 card mirroring `SIL/sd`,
`p2iss::sdimage::mad_card`). *Files:* `p2iss/examples/dump_card.rs` (takes
the directory to mirror), `SIL/makefile`; MaD's root `.gitignore` already
ignores `build/`.
*Done when:* `make flash-image sd-image` writes both, and an EC32 project naming them (the module from
its fingers, the two image entries of `mad.toml`) passes `embsim check`;
the card entry was checked this way with a stand-in image. E5 retires both
targets.

**5. The tool.** The makefile runs `embsim` from the pinned submodule.
Which embsim the project runs on is the runner's, `mad-runner`, whose
`embsim-cli` is the submodule by path and whose lock is `SIL/Cargo.lock`,
whatever tool starts it; running the tool from the same commit keeps its
own checks (the head it reads first, the hand-over) that commit's too:

```make
# The embsim tool, built from the pinned submodule; it builds and runs
# mad-runner, which holds MaD's kinds (PROJECTS.md §10, "A runner the
# project owns"). EMBSIM=embsim uses an installed one.
EMBSIM ?= cargo run --release --quiet --manifest-path embsim/Cargo.toml -p embsim-cli --
```

A developer may `cargo install --locked --path SIL/embsim/cli` instead, or
use any installed embsim 0.2: either hands MaD's projects to
`mad-runner`. *Files:* `SIL/makefile`. *Done when:* `make check` (step 9)
runs through it.

**6. Start the crates.** In `SIL/`: `embsim new --catalog mad-catalog
--own-runner` writes `mad-catalog/Cargo.toml` and `src/lib.rs` with one
example of each sort of kind, named `mad-board`, `mad-sensor`, `mad-core`
and `mad-source`, and `mad-runner/` (section 3), and says to add both to
the workspace's members. Keep the registration function; replace the four
examples with section 3's kinds as the steps below write them. *Files:*
`SIL/mad-catalog/` and `SIL/mad-runner/` (new), `SIL/Cargo.toml`
(`members` gains `"mad-catalog"` and `"mad-runner"`), `SIL/Cargo.lock`
(the two crates locked; commit it), `.github/workflows/ci.yml` (the
`rustfmt (gating)` step's `cargo fmt -p mad-emulator -p models` gains `-p
mad-catalog -p mad-runner`). *Done when:* `cargo test -p mad-catalog`
passes in MaD's workspace, and `cargo build --locked -p mad-runner`
builds.

**7. The boards.** `mad-edge` and `mad-ds2` (section 3). Export
`mad_edge.net` with `kicad-cli sch export netlist` and give it the
provenance header embsim's fixture has; `git mv` the DS2 netlist and point
`DS2_NETLIST` (`MaDSim/src/system_description.rs`) at its new path, so `mad-emulator`
keeps building. *Files:* `SIL/mad-catalog/netlists/`,
`SIL/mad-catalog/src/boards.rs`, `SIL/MaDSim/src/system_description.rs`.
*Done when:* `SIL/mad-catalog/tests/boards.rs` builds `mad-ds2` on its
bench supplies (the scenario lines of `mad.toml`, as
`boards/projects/ds2-addon.toml` does), and builds `mad-edge` too once its
netlist is exported from a schematic carrying `U25`'s corrected part number
(E1). Exported before that fix, it refuses `mad-edge` naming exactly `U25`.

**8. The core, then the machine.** `mad-p2iss` (section 3) once step 2 is
done; `mad-machine` now that E3 is built (`NODES.md` §16). *Files:* `SIL/mad-catalog/src/iss.rs`,
`SIL/mad-catalog/src/machine.rs`; `MaDSim/src/iss_description.rs`,
`system_description.rs` and `main.rs` lose what moved (section 3's table)
only at step 11. *Done when:* `tests/boot.rs` runs a project of the
module, its flash image and a `host-serial` on `P62`, and the core's
report carries the firmware's boot console (skipped, saying why, when the
`propeller2_debug` image is absent, as `p2iss`'s tests are); and
`tests/machine.rs`, stepped, holds two cases:

- **Steps as edges.** Two `scripted-source`s drive `STEP` through a few
  steps and `DIR` between them (four rising edges with `DIR` low, then
  four with it high), and the encoder's quadrature on `ENC_A+`/`ENC_B+`
  counts one a step, up and then back down (the drive's convention, `DIR`
  low forward), each `-` leg the inverse of its `+` at every step (E6).
- **Travel as a rate.** A component of the test's own drives `STEP` with
  `embsim_board::Drive::Periodic`, a `PeriodicSchedule` at a fixed rate,
  the path `embsim_models::machine::StepperMotor` takes for a step train;
  the case reads the upper loop released at 100 mm. That is 819 200 steps
  (`STEPS_PER_MM` × `TRAVEL_MM`): a `scripted-source` would list about
  1.6 million steps, each a wake of its own, and embsim carries a step
  train as a rate, not edge by edge (`README.md`, "Why it stays fast").
  A periodic stimulus kind would let a project file say the same; this
  test does not need one, and adding one is an item for section 2 first.

**9. The project files.** The four files of section 1 (section 4), and a
`make check` target running `$(EMBSIM) check` on `mad.toml` and
`mad-serial-boot.toml`, and on the two `-cosim` files once E4 ships.
*Needs:* E1, E2 and E6 (the encoder's pairs). The first pin bump past
0.2.0 brings E2 (embsim 0.3.0) and changes
`embsim_models::ads122u04::Config`, which no longer has `vref_mv` or
`gain` (`CHANGELOG.md`): until step 11 retires it, `mad-emulator`'s
`BenchForcePath::build` (`MaDSim/src/iss_description.rs`) registers
`Config::default()` on that bump, and the firmware's own writes set the
gain and the reference it set by hand. *Files:* `SIL/mad.toml`,
`SIL/mad-cosim.toml`, `SIL/mad-serial-boot.toml`,
`SIL/mad-serial-boot-cosim.toml`, `SIL/makefile`,
`SIL/MaDSim/src/iss_description.rs` (the registration). *Done when:* `make
check` exits 0, and `SIL/mad-catalog/tests/mad.rs`, stepped, loading
`mad.toml` (its `host-serial`), holds what `NODES.md` §13 review item 1
proposed: a byte from the host reaches `P53`, a byte the firmware sends on
`P55` reaches `HOST.RX`, `P19`–`P21` read inactive at rest, the drive is
enabled, and the encoder's edges reach `P9` and `P10`. The
encoder's case holds only with the pairs section 4 wires: a single-ended
encoder on `A+`/`B+` with `JP2`/`JP3` closed leaves `P9` and `P10` high
at every count, each low no differential at `U25`, which an AM26LV32 reads
as its fail-safe high (SLLS202H §8.4.1, Table 8-1; section 6, "The encoder
through `U25`"); `board/tests/edge_encoder_pairs.rs` shows both on
`boards/projects/edge-ec32-ds2.toml`. The host's
byte crosses `IC2` on the part's default state, not on a valid high
(section 6): with `IC2`'s Pi side on `RPI_5V` its input thresholds are
1.5 V and 3.5 V, so the host's 0 V low is a valid low and its 3.3 V high
is an open input, for which an `ISO6742DWR`, without the `F` option,
drives its default high (SLLSFJ6G's "INx open" row;
`models/src/isolation/iso67xx.rs`, `an_undecidable_input_gets_the_default_output`).
`IC2` alone on those rails, a `scripted-source` on `INC`, gave `P53`
`Driven(High)` for 3.3 V and `Driven(Low)` for 0 V; the `ISO6742FDWR`
gave `Driven(Low)` for both. That test builds the
system from the file with `Project::load` and the set
(`embsim_cli::shipped()` and `mad_catalog::register`), as
`board/tests/edge_project_live.rs` builds its file.

**10. The e2e target.** The make target keeps its name and runs the tool
on the cosim file, as step 1b ran `mad-emulator`:

| Target | Becomes |
|---|---|
| `e2e-emulator` | `$(EMBSIM) run mad-cosim.toml`, unpaced (the page lives only the time the board grants it), until SIGTERM, then the summary; the suite attaches with `CDP_URL=http://127.0.0.1:9222` and serves `APP_URL` from the host |
| `playground`, `playground-iss`, `playground-rom` | stay on `mad-emulator` until step 11, a person using step 1b's route meanwhile: on a PTY they are the ISS with a host that keeps wall time, which the rule does not keep |

`MaDSim/tests/pty_protocol.rs` (one protocol round trip on the host's PTY)
moves to `SIL/mad-catalog/tests/pty_protocol.rs`, spawning `embsim run
mad.toml --pty <path>`: a test of the line, not of the app, which needs no
browser. *Needs:* E4 and step 9's cosim file. *Files:* `SIL/makefile`,
`SIL/mad-catalog/tests/pty_protocol.rs`, `.github/workflows/ci.yml` (a
`make check` step joins the e2e job, whose `make e2e-emulator` runs on the
runner's own Chrome: no image, no VM), `.github/workflows/e2e-nightly.yml`
(`make e2e-emulator` in place of step 1b's invocation), and the docs that
describe the emulator: `docs/dev/sil-testing.md`,
`docs/how-it-works/sil-emulator.md`, `docs/dev/sil-iss-components.md`,
`docs/dev/reusing-embsim.md`, `CLAUDE.md` ("SIL Testing", "SIL Emulator
Architecture"), `README.md`. The comments in
`Software/Control/e2e/{run-all,sil-smoke,sil-playground,capture-screenshots}.mjs`
name the make targets and need no change. *Done when:* the e2e suite
passes against `make e2e-emulator` on the project.

**11. Retire `mad-emulator`.** The playgrounds move onto the tool with the
same host: `playground` and `playground-iss` to `$(EMBSIM) run
mad-cosim.toml`, `playground-rom` to `$(EMBSIM) run
mad-serial-boot-cosim.toml`, each printing the DevTools URL; a person
watches the app in the Chrome the node launched, headed. The page lives the
board's time, so a playground of the ISS runs as fast as the board does and
no faster, and needs no pace; `run --pace` (`NODES.md` §13, "Open") is for
a fast board with a host that keeps wall time, which MaD's rule rules out
for the ISS. Animation frames do not run on virtual time, so the live
charts redraw only now and then (`NODES.md` §18). *Needs:* step 10, and
step 3's numbers. *Files:* `SIL/makefile`, `SIL/MaDSim/` (removed),
`SIL/Cargo.toml` (`members`), `.github/workflows/ci.yml` (`-p
mad-emulator` dropped from the `rustfmt (gating)` step),
`p2iss/src/lib.rs` (the `Component` impl, `SerialLink` and the pin lists
removed), the `p2iss` tests and examples that use them
(`tests/level_pins.rs`, `protocol_on_levels.rs`, `rom_boot_net.rs`,
`rom_serial_net.rs`, `sd_mount.rs`, `sd_node.rs`, `pty_protocol.rs`;
`examples/iss_speed.rs`, `sd_probe.rs`) moved onto
the core in the package. *Done when:* `make test` and the e2e suite pass
with no `mad-emulator` in the tree.

## 6. Questions for MaD

These are the machine's and the board's, not embsim's; `NODES.md` §13
("Open") keeps the list.

- **The ISS with a host that keeps wall time — answered.** The user's rule
  (2026-10-09, `NODES.md` §18) is that the firmware on the ISS runs with
  the control app in the host's Chrome, its clock metered by the board's
  over DevTools, and with no host that keeps wall time; `make e2e-emulator`
  as the ISS with a PTY host is not a configuration MaD keeps. The plan
  follows the rule: step 1b puts the e2e, the nightly and a playground on
  `mad-emulator` with a `chrome-cdp` host once E4 ships, and steps 10 and 11
  move them onto the `-cosim` files. Chrome's own Web Serial and the OS's
  serial driver are not in that path; MaD tests them on the machine.
- **The loop supply.** `loop_volts = 24.0` is the isolation test's bench
  figure. Does the real machine source its switch loops from a supply of
  its own, or from the carrier's `5V_IO`/`GND_IO` (`J5`–`J8`)?
- **`JP1`'s position.** The file closes pads 1–2, `SC_ENA` from `IC14`'s
  TTL output; pads 2–3 put it on `Q1`'s open collector.
- **The Pi's TX against `IC2`.** `IC2`'s Pi side is supplied from `RPI_5V`
  (`J4.1`), so its `V_IH` is 3.5 V (0.7 × 5 V, SLLSFJ6G §7.3), above a
  Raspberry Pi's 3.3 V output. With the Pi's real rail on `HOST.VIO`,
  embsim reads the host's 3.3 V high as no valid level at `IC2`, an open
  input, and the 0 V low as a valid low. `IC2` is an `ISO6742DWR`, whose
  default output for an open input is high, so a byte from the host still
  reaches `P53` (step 9): its highs ride the default state, not a
  guaranteed level, and the same board with the `F` part would hold `P53`
  low. On the same side, `OUTA` drives the Pi's receive line (`RPI_TX`) at
  5 V, above a Pi's 3.3 V GPIO. Both are findings about the board;
  supplying `IC2`'s Pi side from the Pi's 3.3 V would clear both.
- **The encoder through `U25`.** `J20` takes an RS-422 encoder (`A±`,
  `B±`, `ZI±`), and `U25`, an AM26LV32, reads each pair's difference
  against ±200 mV. A single-ended encoder on `A+` and `B+` with `JP2` and
  `JP3` closed, `A−` and `B−` at `EN_GND`, gives a full differential for
  its high and none for its low, `A+` and `A−` both at ground: SLLS202H's
  Table 8-1 gives that input no level ("?", the fail-safe not guaranteed
  with a common-mode voltage applied, §8.4.1), and embsim's `am26lv32`
  reads it as the fail-safe's high. So `1Y` and `2Y` stay high, and `P9`
  and `P10` with them, at every count (`board/tests/edge_encoder_pairs.rs`,
  the single-ended case). Section 4 wires the pairs instead, `mad-machine`
  driving both legs of each channel (E6) with `JP2`, `JP3` and `JP5` open,
  which the same test counts at `P9` and `P10`. If the real machine's
  encoder is single-ended, the carrier wants each `−` leg held between its
  output's two levels, not at ground; which encoder the machine has is
  MaD's to say before step 8 settles the machine's pins.
- **The drive's ready output.** `mad-machine` has no pin for the servo
  drive's ready line, `SC_SRDY` (`J21.6`, the firmware's `SERVO_RDY` on
  `P5`); `mad-emulator` holds `P5` at its inactive level with a bench pull
  (`IDLE_PULLS`' `SERVO_RDY` entry, `MaDSim/src/iss_description.rs`).
  Whether the machine presents it,
  and at what level the carrier reads it open, is to settle before
  step 8.
